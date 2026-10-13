// SPDX-License-Identifier: MIT
///
/// @file interpolator_test.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Unit tests for the interpolation classes declared in interpolator.hxx.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <cmath>
#include <gtest/gtest.h>
#include <initializer_list>

#include "common.hxx"
#include "common_auxiliary.hxx"
#include "egm_wrapper.pb.h"
#include "egm_wrapper_trajectory.pb.h"
#include "interpolator.hxx"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr double TOLERANCE = 1e-9;
constexpr double TRIG_TOLERANCE = 1e-6;
// The linear branch of Slerp is a normalized lerp, so its angular velocity only approaches the
// angular velocity of the corresponding Slerp.
constexpr double LINEAR_TOLERANCE = 1e-5;

// --- helpers ---

wrapper::Euler makeEuler(const double x, const double y, const double z) {
    wrapper::Euler euler;
    euler.set_x(x);
    euler.set_y(y);
    euler.set_z(z);

    return euler;
}

wrapper::Quaternion makeQuaternion(const double u0, const double u1, const double u2, const double u3) {
    wrapper::Quaternion quaternion;
    quaternion.set_u0(u0);
    quaternion.set_u1(u1);
    quaternion.set_u2(u2);
    quaternion.set_u3(u3);

    return quaternion;
}

wrapper::Quaternion makeYawQuaternion(const double degrees) {
    wrapper::Quaternion quaternion;
    convert(&quaternion, makeEuler(0.0, 0.0, degrees));

    return quaternion;
}

wrapper::trajectory::JointGoal makeJointGoal(
    const std::initializer_list<double> position, const std::initializer_list<double> velocity = {},
    const std::initializer_list<double> acceleration = {}
) {
    wrapper::trajectory::JointGoal goal;

    for (const double value : position) { goal.mutable_position()->add_values(value); }
    for (const double value : velocity) { goal.mutable_velocity()->add_values(value); }
    for (const double value : acceleration) { goal.mutable_acceleration()->add_values(value); }

    return goal;
}

wrapper::trajectory::JointGoal makeZeroJointGoal(const int size) {
    wrapper::trajectory::JointGoal goal;

    for (int i = 0; i < size; ++i) {
        goal.mutable_position()->add_values(0.0);
        goal.mutable_velocity()->add_values(0.0);
        goal.mutable_acceleration()->add_values(0.0);
    }

    return goal;
}

// Note: The interpolator writes into already existing elements, so an output point goal must be sized up front.
wrapper::trajectory::PointGoal makeOutputPointGoal(const int robot_joints, const int external_joints) {
    wrapper::trajectory::PointGoal point;

    for (int i = 0; i < robot_joints; ++i) {
        point.mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
        point.mutable_robot()->mutable_joints()->mutable_velocity()->add_values(0.0);
        point.mutable_robot()->mutable_joints()->mutable_acceleration()->add_values(0.0);
    }

    for (int i = 0; i < external_joints; ++i) {
        point.mutable_external()->mutable_joints()->mutable_position()->add_values(0.0);
        point.mutable_external()->mutable_joints()->mutable_velocity()->add_values(0.0);
        point.mutable_external()->mutable_joints()->mutable_acceleration()->add_values(0.0);
    }

    return point;
}

SplineConditions makeSplineConditions(const SplineMethod method, const double duration) {
    Conditions conditions;
    conditions.duration = duration;
    conditions.spline_method = method;

    return SplineConditions(conditions);
}

// --- SplineConditions ---

TEST(SplineConditions, copiesTheGeneralConditions) {
    Conditions conditions;
    conditions.duration = 1.5;
    conditions.spline_method = Cubic;
    conditions.operation = Operation::RampDown;
    conditions.ramp_down_factor = 0.25;

    const SplineConditions spline_conditions(conditions);

    EXPECT_DOUBLE_EQ(1.5, spline_conditions.duration);
    EXPECT_EQ(Cubic, spline_conditions.spline_method);
    EXPECT_TRUE(spline_conditions.do_ramp_down);
    EXPECT_DOUBLE_EQ(0.25, spline_conditions.ramp_down_factor);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.alfa);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.d_alfa);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.dd_alfa);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.beta);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.d_beta);
    EXPECT_DOUBLE_EQ(0.0, spline_conditions.dd_beta);
}

TEST(SplineConditions, normalOperationDoesNotRequestRampDown) {
    Conditions conditions;
    conditions.operation = Operation::Normal;

    const SplineConditions spline_conditions(conditions);

    EXPECT_FALSE(spline_conditions.do_ramp_down);
}

TEST(SplineConditions, extractsTheJointBoundaryValues) {
    const wrapper::trajectory::JointGoal start = makeJointGoal({ 1.0, 2.0 }, { 3.0, 4.0 }, { 5.0, 6.0 });
    const wrapper::trajectory::JointGoal goal = makeJointGoal({ 7.0, 8.0 }, { 9.0, 10.0 }, { 11.0, 12.0 });

    SplineConditions conditions = makeSplineConditions(Quintic, 1.0);
    conditions.setConditions(1, start, goal);

    EXPECT_DOUBLE_EQ(2.0, conditions.alfa);
    EXPECT_DOUBLE_EQ(4.0, conditions.d_alfa);
    EXPECT_DOUBLE_EQ(6.0, conditions.dd_alfa);
    EXPECT_DOUBLE_EQ(8.0, conditions.beta);
    EXPECT_DOUBLE_EQ(10.0, conditions.d_beta);
    EXPECT_DOUBLE_EQ(12.0, conditions.dd_beta);
}

TEST(SplineConditions, extractsTheCartesianBoundaryValuesForTheRequestedAxis) {
    wrapper::trajectory::CartesianGoal start;
    start.mutable_pose()->mutable_position()->set_x(1.0);
    start.mutable_pose()->mutable_position()->set_y(2.0);
    start.mutable_pose()->mutable_position()->set_z(3.0);
    start.mutable_velocity()->set_x(4.0);
    start.mutable_velocity()->set_y(5.0);
    start.mutable_velocity()->set_z(6.0);
    start.mutable_acceleration()->set_x(7.0);
    start.mutable_acceleration()->set_y(8.0);
    start.mutable_acceleration()->set_z(9.0);

    wrapper::trajectory::CartesianGoal goal;
    goal.mutable_pose()->mutable_position()->set_x(10.0);
    goal.mutable_pose()->mutable_position()->set_y(20.0);
    goal.mutable_pose()->mutable_position()->set_z(30.0);
    goal.mutable_velocity()->set_x(11.0);
    goal.mutable_velocity()->set_y(12.0);
    goal.mutable_velocity()->set_z(13.0);
    goal.mutable_acceleration()->set_x(14.0);
    goal.mutable_acceleration()->set_y(15.0);
    goal.mutable_acceleration()->set_z(16.0);

    SplineConditions conditions = makeSplineConditions(Quintic, 1.0);

    conditions.setConditions(Axis::X, start, goal);
    EXPECT_DOUBLE_EQ(1.0, conditions.alfa);
    EXPECT_DOUBLE_EQ(4.0, conditions.d_alfa);
    EXPECT_DOUBLE_EQ(7.0, conditions.dd_alfa);
    EXPECT_DOUBLE_EQ(10.0, conditions.beta);
    EXPECT_DOUBLE_EQ(11.0, conditions.d_beta);
    EXPECT_DOUBLE_EQ(14.0, conditions.dd_beta);

    conditions.setConditions(Axis::Y, start, goal);
    EXPECT_DOUBLE_EQ(2.0, conditions.alfa);
    EXPECT_DOUBLE_EQ(5.0, conditions.d_alfa);
    EXPECT_DOUBLE_EQ(8.0, conditions.dd_alfa);
    EXPECT_DOUBLE_EQ(20.0, conditions.beta);
    EXPECT_DOUBLE_EQ(12.0, conditions.d_beta);
    EXPECT_DOUBLE_EQ(15.0, conditions.dd_beta);

    conditions.setConditions(Axis::Z, start, goal);
    EXPECT_DOUBLE_EQ(3.0, conditions.alfa);
    EXPECT_DOUBLE_EQ(6.0, conditions.d_alfa);
    EXPECT_DOUBLE_EQ(9.0, conditions.dd_alfa);
    EXPECT_DOUBLE_EQ(30.0, conditions.beta);
    EXPECT_DOUBLE_EQ(13.0, conditions.d_beta);
    EXPECT_DOUBLE_EQ(16.0, conditions.dd_beta);
}

// --- SplinePolynomial ---

TEST(SplinePolynomial, linearHitsBothBoundaryPositionsWithAConstantVelocity) {
    SplineConditions conditions = makeSplineConditions(Linear, 2.0);
    conditions.alfa = 1.0;
    conditions.beta = 5.0;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    polynomial.evaluate(&output, 0, 0.0);
    EXPECT_NEAR(1.0, output.position().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 1.0);
    EXPECT_NEAR(3.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(2.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.acceleration().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 2.0);
    EXPECT_NEAR(5.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(2.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.acceleration().values(0), TOLERANCE);
}

TEST(SplinePolynomial, squareHitsBothBoundaryPositionsWithTheStartVelocity) {
    SplineConditions conditions = makeSplineConditions(Square, 2.0);
    conditions.alfa = 0.0;
    conditions.d_alfa = 1.0;
    conditions.beta = 5.0;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    polynomial.evaluate(&output, 0, 0.0);
    EXPECT_NEAR(0.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(1.0, output.velocity().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 2.0);
    EXPECT_NEAR(5.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(4.0, output.velocity().values(0), TOLERANCE);
}

TEST(SplinePolynomial, cubicHitsBothBoundaryPositionsAndVelocities) {
    SplineConditions conditions = makeSplineConditions(Cubic, 2.0);
    conditions.alfa = 0.0;
    conditions.d_alfa = 0.0;
    conditions.beta = 4.0;
    conditions.d_beta = 0.0;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    polynomial.evaluate(&output, 0, 0.0);
    EXPECT_NEAR(0.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.velocity().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 1.0);
    EXPECT_NEAR(2.0, output.position().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 2.0);
    EXPECT_NEAR(4.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.velocity().values(0), TOLERANCE);
}

TEST(SplinePolynomial, quinticHitsEveryBoundaryCondition) {
    SplineConditions conditions = makeSplineConditions(Quintic, 2.0);
    conditions.alfa = 1.0;
    conditions.d_alfa = 0.5;
    conditions.dd_alfa = 2.0;
    conditions.beta = 3.0;
    conditions.d_beta = -1.0;
    conditions.dd_beta = 0.5;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    polynomial.evaluate(&output, 0, 0.0);
    EXPECT_NEAR(1.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(0.5, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(2.0, output.acceleration().values(0), TOLERANCE);

    polynomial.evaluate(&output, 0, 2.0);
    EXPECT_NEAR(3.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(-1.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.5, output.acceleration().values(0), TOLERANCE);
}

TEST(SplinePolynomial, quinticIsSymmetricForAUnitStep) {
    SplineConditions conditions = makeSplineConditions(Quintic, 1.0);
    conditions.alfa = 0.0;
    conditions.d_alfa = 0.0;
    conditions.dd_alfa = 0.0;
    conditions.beta = 10.0;
    conditions.d_beta = 0.0;
    conditions.dd_beta = 0.0;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    // S(t) = 10*(10t^3 - 15t^4 + 6t^5), so S(0.5) = 5 and both ends are at rest.
    polynomial.evaluate(&output, 0, 0.5);
    EXPECT_NEAR(5.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(18.75, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.acceleration().values(0), TOLERANCE);
}

TEST(SplinePolynomial, rampDownReachesTheRequestedFractionOfTheStartVelocity) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.operation = Operation::RampDown;
    conditions.ramp_down_factor = 0.5;

    SplineConditions spline_conditions(conditions);
    spline_conditions.alfa = 0.0;
    spline_conditions.d_alfa = 1.0;

    SplinePolynomial polynomial;
    polynomial.update(spline_conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    polynomial.evaluate(&output, 0, 0.0);
    EXPECT_NEAR(0.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(1.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(-1.0, output.acceleration().values(0), TOLERANCE);

    // d_S(T) = K*d_alfa and dd_S(T) = 0.
    polynomial.evaluate(&output, 0, 1.0);
    EXPECT_NEAR(2.0 / 3.0, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(0.5, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.acceleration().values(0), TOLERANCE);
}

TEST(SplinePolynomial, rampDownFactorIsSaturated) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.operation = Operation::RampDown;

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(1);

    // A factor above 1.0 saturates to a constant velocity, i.e. no ramp down at all.
    conditions.ramp_down_factor = 5.0;
    SplineConditions spline_conditions(conditions);
    spline_conditions.d_alfa = 1.0;

    SplinePolynomial polynomial;
    polynomial.update(spline_conditions);
    polynomial.evaluate(&output, 0, 1.0);
    EXPECT_NEAR(1.0, output.velocity().values(0), TOLERANCE);

    // A negative factor saturates to a full stop.
    conditions.ramp_down_factor = -1.0;
    spline_conditions = SplineConditions(conditions);
    spline_conditions.d_alfa = 1.0;

    polynomial.update(spline_conditions);
    polynomial.evaluate(&output, 0, 1.0);
    EXPECT_NEAR(0.0, output.velocity().values(0), TOLERANCE);
}

TEST(SplinePolynomial, evaluatesCartesianAxesIndependently) {
    SplineConditions conditions = makeSplineConditions(Linear, 1.0);
    conditions.alfa = 1.0;
    conditions.beta = 11.0;

    SplinePolynomial polynomial;
    polynomial.update(conditions);

    wrapper::trajectory::CartesianGoal output;
    polynomial.evaluate(&output, Axis::Y, 1.0);

    EXPECT_NEAR(11.0, output.pose().position().y(), TOLERANCE);
    EXPECT_NEAR(10.0, output.velocity().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().position().x(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().position().z(), TOLERANCE);
}

// --- Slerp ---

TEST(Slerp, identicalQuaternionsUseTheLinearBranchAndReportNoAngularVelocity) {
    Conditions conditions;
    conditions.duration = 1.0;

    Slerp slerp;
    const wrapper::Quaternion identity = makeQuaternion(1.0, 0.0, 0.0, 0.0);
    slerp.update(identity, identity, conditions);

    wrapper::trajectory::CartesianGoal output;
    slerp.evaluate(&output, 0.5);

    EXPECT_NEAR(1.0, output.pose().quaternion().u0(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().quaternion().u3(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().euler().z(), TOLERANCE);
}

TEST(Slerp, smallRotationsStayOnTheLinearBranch) {
    Conditions conditions;
    conditions.duration = 1.0;

    Slerp slerp;
    slerp.update(makeQuaternion(1.0, 0.0, 0.0, 0.0), makeYawQuaternion(1.0), conditions);

    wrapper::trajectory::CartesianGoal output;
    slerp.evaluate(&output, 0.5);

    // The linear branch is a normalized lerp, so the midpoint is the half angle rotation itself.
    EXPECT_NEAR(std::cos(0.25 * DEG_TO_RAD), output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(0.25 * DEG_TO_RAD), output.pose().quaternion().u3(), TRIG_TOLERANCE);
    EXPECT_NEAR(1.0, output.pose().euler().z(), LINEAR_TOLERANCE);
}

TEST(Slerp, interpolatesAFullQuarterTurnWithAUniformAngularVelocity) {
    Conditions conditions;
    conditions.duration = 1.0;

    Slerp slerp;
    slerp.update(makeQuaternion(1.0, 0.0, 0.0, 0.0), makeYawQuaternion(90.0), conditions);

    wrapper::trajectory::CartesianGoal output;

    slerp.evaluate(&output, 0.0);
    EXPECT_NEAR(1.0, output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().quaternion().u3(), TRIG_TOLERANCE);

    slerp.evaluate(&output, 0.5);
    EXPECT_NEAR(std::cos(PI / 8.0), output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(PI / 8.0), output.pose().quaternion().u3(), TRIG_TOLERANCE);
    EXPECT_NEAR(90.0, output.pose().euler().z(), TRIG_TOLERANCE);

    slerp.evaluate(&output, 1.0);
    EXPECT_NEAR(std::cos(PI / 4.0), output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(PI / 4.0), output.pose().quaternion().u3(), TRIG_TOLERANCE);
    EXPECT_NEAR(90.0, output.pose().euler().z(), TRIG_TOLERANCE);
}

TEST(Slerp, negativeDotProductFlipsTheGoalToTakeTheShorterPath) {
    Conditions conditions;
    conditions.duration = 1.0;

    Slerp slerp;
    slerp.update(makeQuaternion(1.0, 0.0, 0.0, 0.0), makeYawQuaternion(270.0), conditions);

    wrapper::trajectory::CartesianGoal output;
    slerp.evaluate(&output, 1.0);

    // The goal is negated, so the interpolation ends on the equivalent -q1 (a -90 degree yaw).
    EXPECT_NEAR(std::cos(PI / 4.0), output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(-std::sin(PI / 4.0), output.pose().quaternion().u3(), TRIG_TOLERANCE);

    slerp.evaluate(&output, 0.5);
    EXPECT_NEAR(-90.0, output.pose().euler().z(), TRIG_TOLERANCE);
}

TEST(Slerp, saturatesTheTimeInterpolation) {
    Conditions conditions;
    conditions.duration = 2.0;

    Slerp slerp;
    slerp.update(makeQuaternion(1.0, 0.0, 0.0, 0.0), makeYawQuaternion(90.0), conditions);

    wrapper::trajectory::CartesianGoal output;
    slerp.evaluate(&output, 100.0);

    EXPECT_NEAR(std::cos(PI / 4.0), output.pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(PI / 4.0), output.pose().quaternion().u3(), TRIG_TOLERANCE);
}

// --- SoftRamp ---

TEST(SoftRamp, rampsTheCartesianPositionFromStartToGoal) {
    Conditions conditions;
    conditions.duration = 2.0;
    conditions.operation = Operation::RampInPosition;
    conditions.mode = EGMPose;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(0.0);
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(
        makeQuaternion(1.0, 0.0, 0.0, 0.0)
    );

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(10.0);
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(20.0);
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(30.0);
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(
        makeQuaternion(1.0, 0.0, 0.0, 0.0)
    );

    SoftRamp soft_ramp;
    soft_ramp.update(start, goal, conditions);

    wrapper::trajectory::CartesianGoal output;

    soft_ramp.evaluate(&output, 0.01, 0.0);
    EXPECT_NEAR(0.0, output.pose().position().x(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().position().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().position().z(), TOLERANCE);

    // The ramp factor is 0.5*cos(pi*t + pi) + 0.5, i.e. 0.5 halfway through.
    soft_ramp.evaluate(&output, 0.01, 1.0);
    EXPECT_NEAR(5.0, output.pose().position().x(), TOLERANCE);
    EXPECT_NEAR(10.0, output.pose().position().y(), TOLERANCE);
    EXPECT_NEAR(15.0, output.pose().position().z(), TOLERANCE);

    soft_ramp.evaluate(&output, 0.01, 2.0);
    EXPECT_NEAR(10.0, output.pose().position().x(), TOLERANCE);
    EXPECT_NEAR(20.0, output.pose().position().y(), TOLERANCE);
    EXPECT_NEAR(30.0, output.pose().position().z(), TOLERANCE);
}

TEST(SoftRamp, rampsTheJointVelocitiesAndIntegratesThemIntoPositions) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.operation = Operation::RampInVelocity;
    conditions.mode = EGMJoint;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 2.0, 4.0 }, { 0.0, 0.0 }));

    SoftRamp soft_ramp;
    soft_ramp.update(start, goal, conditions);

    wrapper::trajectory::JointGoal output = makeZeroJointGoal(2);

    soft_ramp.evaluate(&output, true, 0.1, 0.0);
    EXPECT_NEAR(0.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.position().values(0), TOLERANCE);

    soft_ramp.evaluate(&output, true, 0.1, 1.0);
    EXPECT_NEAR(2.0, output.velocity().values(0), TOLERANCE);
    EXPECT_NEAR(4.0, output.velocity().values(1), TOLERANCE);
    EXPECT_NEAR(0.2, output.position().values(0), TOLERANCE);
    EXPECT_NEAR(0.4, output.position().values(1), TOLERANCE);
    EXPECT_NEAR(0.0, output.acceleration().values(0), TOLERANCE);
}

TEST(SoftRamp, rampDownDecaysTheAngularVelocityToZero) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.operation = Operation::RampDown;
    conditions.mode = EGMPose;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->CopyFrom(makeEuler(10.0, 20.0, 30.0));
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(
        makeQuaternion(1.0, 0.0, 0.0, 0.0)
    );

    wrapper::trajectory::PointGoal goal;

    SoftRamp soft_ramp;
    soft_ramp.update(start, goal, conditions);

    wrapper::trajectory::CartesianGoal output;

    // The Euler field is internally used to carry the angular velocity; the ramp factor starts at 1.0.
    soft_ramp.evaluate(&output, 0.01, 0.0);
    EXPECT_NEAR(10.0, output.pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(20.0, output.pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(30.0, output.pose().euler().z(), TOLERANCE);

    soft_ramp.evaluate(&output, 0.01, 0.5);
    EXPECT_NEAR(5.0, output.pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(10.0, output.pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(15.0, output.pose().euler().z(), TOLERANCE);

    soft_ramp.evaluate(&output, 0.01, 1.0);
    EXPECT_NEAR(0.0, output.pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.pose().euler().z(), TOLERANCE);
}

// --- EGMInterpolator ---

TEST(EGMInterpolator, followsAQuinticSplineForRobotAndExternalJoints) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;
    conditions.spline_method = Quintic;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));
    start.mutable_external()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0 }, { 0.0 }, { 0.0 }));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 10.0, -20.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));
    goal.mutable_external()->mutable_joints()->CopyFrom(makeJointGoal({ 5.0 }, { 0.0 }, { 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    EXPECT_DOUBLE_EQ(1.0, interpolator.getDuration());

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(2, 1);

    interpolator.evaluate(&output, 0.004, 0.0);
    EXPECT_NEAR(0.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.external().joints().position().values(0), TOLERANCE);

    interpolator.evaluate(&output, 0.004, 0.5);
    EXPECT_NEAR(5.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(-10.0, output.robot().joints().position().values(1), TOLERANCE);
    EXPECT_NEAR(18.75, output.robot().joints().velocity().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().joints().acceleration().values(0), TOLERANCE);
    EXPECT_NEAR(2.5, output.external().joints().position().values(0), TOLERANCE);

    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(10.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(-20.0, output.robot().joints().position().values(1), TOLERANCE);
    EXPECT_NEAR(5.0, output.external().joints().position().values(0), TOLERANCE);
}

TEST(EGMInterpolator, clampsTheDurationToTheLowestSampleTime) {
    Conditions conditions;
    conditions.duration = 0.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeZeroJointGoal(1));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 1.0 }, { 0.0 }, { 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    EXPECT_DOUBLE_EQ(LOWEST_SAMPLE_TIME, interpolator.getDuration());
}

TEST(EGMInterpolator, saturatesTheEvaluationTimeToTheDuration) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;
    conditions.spline_method = Quintic;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0 }, { 0.0 }, { 0.0 }));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 10.0 }, { 0.0 }, { 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(1, 0);

    interpolator.evaluate(&output, 0.004, 1.0);
    const double at_duration = output.robot().joints().position().values(0);

    interpolator.evaluate(&output, 0.004, 100.0);
    EXPECT_DOUBLE_EQ(at_duration, output.robot().joints().position().values(0));
}

TEST(EGMInterpolator, interpolatesTheCartesianPoseWithSlerpForTheOrientation) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMPose;
    conditions.operation = Operation::Normal;
    conditions.spline_method = Quintic;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(
        makeQuaternion(1.0, 0.0, 0.0, 0.0)
    );

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(100.0);
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(makeYawQuaternion(90.0));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output;

    interpolator.evaluate(&output, 0.004, 0.0);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().position().x(), TOLERANCE);
    EXPECT_NEAR(1.0, output.robot().cartesian().pose().quaternion().u0(), TRIG_TOLERANCE);

    interpolator.evaluate(&output, 0.004, 0.5);
    EXPECT_NEAR(50.0, output.robot().cartesian().pose().position().x(), TOLERANCE);
    EXPECT_NEAR(std::cos(PI / 8.0), output.robot().cartesian().pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(PI / 8.0), output.robot().cartesian().pose().quaternion().u3(), TRIG_TOLERANCE);

    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(100.0, output.robot().cartesian().pose().position().x(), TOLERANCE);
    EXPECT_NEAR(std::cos(PI / 4.0), output.robot().cartesian().pose().quaternion().u0(), TRIG_TOLERANCE);
    EXPECT_NEAR(std::sin(PI / 4.0), output.robot().cartesian().pose().quaternion().u3(), TRIG_TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().position().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().position().z(), TOLERANCE);
}

TEST(EGMInterpolator, rampsDownTheCartesianAngularVelocity) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMPose;
    conditions.operation = Operation::RampDown;
    conditions.ramp_down_factor = 0.0;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->CopyFrom(makeEuler(10.0, 20.0, 30.0));
    start.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->CopyFrom(
        makeQuaternion(1.0, 0.0, 0.0, 0.0)
    );

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(0.0);

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output;

    interpolator.evaluate(&output, 0.004, 0.0);
    EXPECT_NEAR(10.0, output.robot().cartesian().pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(20.0, output.robot().cartesian().pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(30.0, output.robot().cartesian().pose().euler().z(), TOLERANCE);

    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().euler().x(), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().euler().y(), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().cartesian().pose().euler().z(), TOLERANCE);
}

TEST(EGMInterpolator, rampsTheJointPositionInForAStaticGoal) {
    Conditions conditions;
    conditions.duration = 2.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::RampInPosition;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 10.0, 20.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(2, 0);

    interpolator.evaluate(&output, 0.004, 0.0);
    EXPECT_NEAR(0.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().joints().position().values(1), TOLERANCE);

    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(5.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(10.0, output.robot().joints().position().values(1), TOLERANCE);
    EXPECT_NEAR(0.0, output.robot().joints().acceleration().values(0), TOLERANCE);

    interpolator.evaluate(&output, 0.004, 2.0);
    EXPECT_NEAR(10.0, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(20.0, output.robot().joints().position().values(1), TOLERANCE);
}

TEST(EGMInterpolator, rampsTheJointVelocityInForAStaticGoal) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::RampInVelocity;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 0.0, 0.0 }, { 2.0, 4.0 }, { 0.0, 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(2, 0);

    interpolator.evaluate(&output, 0.1, 0.0);
    EXPECT_NEAR(0.0, output.robot().joints().velocity().values(0), TOLERANCE);

    interpolator.evaluate(&output, 0.1, 1.0);
    EXPECT_NEAR(2.0, output.robot().joints().velocity().values(0), TOLERANCE);
    EXPECT_NEAR(4.0, output.robot().joints().velocity().values(1), TOLERANCE);
    EXPECT_NEAR(0.2, output.robot().joints().position().values(0), TOLERANCE);
    EXPECT_NEAR(0.4, output.robot().joints().position().values(1), TOLERANCE);
}

TEST(EGMInterpolator, leavesTheExternalJointsUntouchedWhenTheGoalHasNone) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeZeroJointGoal(1));

    wrapper::trajectory::PointGoal goal;
    goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 1.0 }, { 0.0 }, { 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(1, 0);
    interpolator.evaluate(&output, 0.004, 1.0);

    EXPECT_EQ(1, output.robot().joints().position().values_size());
    EXPECT_EQ(0, output.external().joints().position().values_size());
}

TEST(EGMInterpolator, reusesTheInterpolatorForANewGoal) {
    Conditions conditions;
    conditions.duration = 1.0;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;
    conditions.spline_method = Linear;

    wrapper::trajectory::PointGoal start;
    start.mutable_robot()->mutable_joints()->CopyFrom(makeZeroJointGoal(1));

    wrapper::trajectory::PointGoal first_goal;
    first_goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ 10.0 }, { 0.0 }, { 0.0 }));

    EGMInterpolator interpolator;
    interpolator.update(start, first_goal, conditions);

    wrapper::trajectory::PointGoal output = makeOutputPointGoal(1, 0);
    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(10.0, output.robot().joints().position().values(0), TOLERANCE);

    wrapper::trajectory::PointGoal second_goal;
    second_goal.mutable_robot()->mutable_joints()->CopyFrom(makeJointGoal({ -10.0 }, { 0.0 }, { 0.0 }));

    interpolator.update(start, second_goal, conditions);
    interpolator.evaluate(&output, 0.004, 1.0);
    EXPECT_NEAR(-10.0, output.robot().joints().position().values(0), TOLERANCE);
}

} // namespace
