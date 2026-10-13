// SPDX-License-Identifier: MIT
///
/// @file common_auxiliary_test.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Unit tests for the free functions declared in common_auxiliary.hxx.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "common.hxx"
#include "common_auxiliary.hxx"
#include "egm.pb.h"
#include "egm_wrapper.pb.h"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr double TOLERANCE = 1e-9;

wrapper::Quaternion makeQuaternion(const double u0, const double u1, const double u2, const double u3) {
    wrapper::Quaternion q;
    q.set_u0(u0);
    q.set_u1(u1);
    q.set_u2(u2);
    q.set_u3(u3);

    return q;
}

wrapper::Joints makeJoints(const std::initializer_list<double> values) {
    wrapper::Joints joints;
    for (const double value : values) { joints.add_values(value); }

    return joints;
}

wrapper::Euler makeEuler(const double x, const double y, const double z) {
    wrapper::Euler euler;
    euler.set_x(x);
    euler.set_y(y);
    euler.set_z(z);

    return euler;
}

// --- saturate ---

TEST(saturate, passesValuesWithinBoundsThrough) {
    EXPECT_DOUBLE_EQ(0.5, saturate(0.5, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(0.0, saturate(0.0, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(1.0, saturate(1.0, 0.0, 1.0));
}

TEST(saturate, clampsValuesOutsideBounds) {
    EXPECT_DOUBLE_EQ(0.0, saturate(-1.0, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(1.0, saturate(2.0, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(-1.0, saturate(-5.0, -1.0, 1.0));
}

TEST(saturate, handlesNonFiniteValues) {
    EXPECT_DOUBLE_EQ(1.0, saturate(std::numeric_limits<double>::infinity(), 0.0, 1.0));
    EXPECT_DOUBLE_EQ(0.0, saturate(-std::numeric_limits<double>::infinity(), 0.0, 1.0));
}

// --- multiply ---

TEST(multiply, scalesEveryJointsValue) {
    wrapper::Joints joints = makeJoints({ 1.0, -2.0, 3.0 });
    multiply(&joints, 2.0);

    ASSERT_EQ(3, joints.values_size());
    EXPECT_DOUBLE_EQ(2.0, joints.values(0));
    EXPECT_DOUBLE_EQ(-4.0, joints.values(1));
    EXPECT_DOUBLE_EQ(6.0, joints.values(2));
}

TEST(multiply, scalesCartesianEulerAndQuaternionComponents) {
    wrapper::Cartesian cartesian;
    cartesian.set_x(1.0);
    cartesian.set_y(2.0);
    cartesian.set_z(3.0);
    multiply(&cartesian, 0.5);
    EXPECT_DOUBLE_EQ(0.5, cartesian.x());
    EXPECT_DOUBLE_EQ(1.0, cartesian.y());
    EXPECT_DOUBLE_EQ(1.5, cartesian.z());

    wrapper::Euler euler = makeEuler(1.0, 2.0, 3.0);
    multiply(&euler, 2.0);
    EXPECT_DOUBLE_EQ(2.0, euler.x());
    EXPECT_DOUBLE_EQ(4.0, euler.y());
    EXPECT_DOUBLE_EQ(6.0, euler.z());

    wrapper::Quaternion quaternion = makeQuaternion(1.0, 2.0, 3.0, 4.0);
    multiply(&quaternion, 0.5);
    EXPECT_DOUBLE_EQ(0.5, quaternion.u0());
    EXPECT_DOUBLE_EQ(1.0, quaternion.u1());
    EXPECT_DOUBLE_EQ(1.5, quaternion.u2());
    EXPECT_DOUBLE_EQ(2.0, quaternion.u3());
}

TEST(multiply, toleratesNullPointers) {
    multiply(static_cast<wrapper::Joints*>(nullptr), 2.0);
    multiply(static_cast<wrapper::Cartesian*>(nullptr), 2.0);
    multiply(static_cast<wrapper::Euler*>(nullptr), 2.0);
    multiply(static_cast<wrapper::Quaternion*>(nullptr), 2.0);
}

TEST(multiply, quaternionProductFollowsTheHamiltonConvention) {
    // i * j = k.
    const wrapper::Quaternion i = makeQuaternion(0.0, 1.0, 0.0, 0.0);
    const wrapper::Quaternion j = makeQuaternion(0.0, 0.0, 1.0, 0.0);
    const wrapper::Quaternion k = multiply(i, j);

    EXPECT_NEAR(0.0, k.u0(), TOLERANCE);
    EXPECT_NEAR(0.0, k.u1(), TOLERANCE);
    EXPECT_NEAR(0.0, k.u2(), TOLERANCE);
    EXPECT_NEAR(1.0, k.u3(), TOLERANCE);
}

TEST(multiply, quaternionProductWithIdentityIsANoOp) {
    const wrapper::Quaternion identity = makeQuaternion(1.0, 0.0, 0.0, 0.0);
    const wrapper::Quaternion q = makeQuaternion(0.1, 0.2, 0.3, 0.4);
    const wrapper::Quaternion result = multiply(identity, q);

    EXPECT_DOUBLE_EQ(q.u0(), result.u0());
    EXPECT_DOUBLE_EQ(q.u1(), result.u1());
    EXPECT_DOUBLE_EQ(q.u2(), result.u2());
    EXPECT_DOUBLE_EQ(q.u3(), result.u3());
}

// --- dotProduct / euclideanNorm / normalize ---

TEST(dotProduct, sumsTheComponentProducts) {
    const wrapper::Quaternion q1 = makeQuaternion(1.0, 2.0, 3.0, 4.0);
    const wrapper::Quaternion q2 = makeQuaternion(4.0, 3.0, 2.0, 1.0);

    EXPECT_DOUBLE_EQ(20.0, dotProduct(q1, q2));
}

TEST(euclideanNorm, equalsTheSquareRootOfTheSelfDotProduct) {
    const wrapper::Quaternion q = makeQuaternion(1.0, 2.0, 2.0, 4.0);

    EXPECT_DOUBLE_EQ(5.0, euclideanNorm(q));
}

TEST(normalize, scalesAQuaternionToUnitLength) {
    wrapper::Quaternion q = makeQuaternion(0.0, 0.0, 3.0, 4.0);
    normalize(&q);

    EXPECT_NEAR(1.0, euclideanNorm(q), TOLERANCE);
}

TEST(normalize, leavesAZeroQuaternionUntouched) {
    wrapper::Quaternion q = makeQuaternion(0.0, 0.0, 0.0, 0.0);
    normalize(&q);

    EXPECT_DOUBLE_EQ(0.0, q.u0());
    EXPECT_DOUBLE_EQ(0.0, q.u1());
    EXPECT_DOUBLE_EQ(0.0, q.u2());
    EXPECT_DOUBLE_EQ(0.0, q.u3());
}

// --- convert ---

TEST(convert, eulerToQuaternionForAnIdentityRotation) {
    wrapper::Quaternion q;
    convert(&q, makeEuler(0.0, 0.0, 0.0));

    EXPECT_NEAR(1.0, q.u0(), TOLERANCE);
    EXPECT_NEAR(0.0, q.u1(), TOLERANCE);
    EXPECT_NEAR(0.0, q.u2(), TOLERANCE);
    EXPECT_NEAR(0.0, q.u3(), TOLERANCE);
}

TEST(convert, eulerToQuaternionForA90DegreeYaw) {
    wrapper::Quaternion q;
    convert(&q, makeEuler(0.0, 0.0, 90.0));

    const double half_angle = PI / 4.0;
    EXPECT_NEAR(std::cos(half_angle), q.u0(), TOLERANCE);
    EXPECT_NEAR(0.0, q.u1(), TOLERANCE);
    EXPECT_NEAR(0.0, q.u2(), TOLERANCE);
    EXPECT_NEAR(std::sin(half_angle), q.u3(), TOLERANCE);
}

TEST(convert, quaternionToEulerInvertsTheEulerToQuaternionConversion) {
    const wrapper::Euler original = makeEuler(10.0, -20.0, 30.0);

    wrapper::Quaternion q;
    convert(&q, original);

    wrapper::Euler round_trip;
    convert(&round_trip, q);

    EXPECT_NEAR(original.x(), round_trip.x(), 1e-9);
    EXPECT_NEAR(original.y(), round_trip.y(), 1e-9);
    EXPECT_NEAR(original.z(), round_trip.z(), 1e-9);
}

TEST(convert, quaternionToEulerUsesThePositiveGimbalLockBranch) {
    // A 90 degree pitch drives u0*u2 - u1*u3 to 0.5.
    const wrapper::Euler original = makeEuler(0.0, 90.0, 0.0);

    wrapper::Quaternion q;
    convert(&q, original);

    wrapper::Euler result;
    convert(&result, q);

    EXPECT_NEAR(90.0, result.y(), 1e-6);
}

TEST(convert, quaternionToEulerUsesTheNegativeGimbalLockBranch) {
    const wrapper::Euler original = makeEuler(0.0, -90.0, 0.0);

    wrapper::Quaternion q;
    convert(&q, original);

    wrapper::Euler result;
    convert(&result, q);

    EXPECT_NEAR(-90.0, result.y(), 1e-6);
}

TEST(convert, quaternionToEulerLeavesTheTargetUntouchedForAZeroQuaternion) {
    wrapper::Euler euler = makeEuler(1.0, 2.0, 3.0);
    convert(&euler, makeQuaternion(0.0, 0.0, 0.0, 0.0));

    EXPECT_DOUBLE_EQ(1.0, euler.x());
    EXPECT_DOUBLE_EQ(2.0, euler.y());
    EXPECT_DOUBLE_EQ(3.0, euler.z());
}

TEST(convert, angularVelocityToQuaternionDerivative) {
    const wrapper::Quaternion previous = makeQuaternion(1.0, 0.0, 0.0, 0.0);
    const wrapper::Euler angular_velocity = makeEuler(0.0, 0.0, 90.0);

    wrapper::Quaternion d_q;
    convert(&d_q, previous, angular_velocity);

    // d_q = 0.5 * (0, 0, 0, pi/2) * (1, 0, 0, 0) = (0, 0, 0, pi/4).
    EXPECT_NEAR(0.0, d_q.u0(), TOLERANCE);
    EXPECT_NEAR(0.0, d_q.u1(), TOLERANCE);
    EXPECT_NEAR(0.0, d_q.u2(), TOLERANCE);
    EXPECT_NEAR(PI / 4.0, d_q.u3(), TOLERANCE);
}

// --- estimateVelocities ---

TEST(estimateVelocities, jointsDividesTheDeltaByTheSampleTime) {
    const wrapper::Joints current = makeJoints({ 1.0, 2.0, 3.0 });
    const wrapper::Joints previous = makeJoints({ 0.0, 1.0, 1.0 });

    wrapper::Joints estimate;
    EXPECT_TRUE(estimateVelocities(&estimate, current, previous, 0.5));

    ASSERT_EQ(3, estimate.values_size());
    EXPECT_DOUBLE_EQ(2.0, estimate.values(0));
    EXPECT_DOUBLE_EQ(2.0, estimate.values(1));
    EXPECT_DOUBLE_EQ(4.0, estimate.values(2));
}

TEST(estimateVelocities, jointsFailsForANonPositiveSampleTime) {
    wrapper::Joints estimate;
    EXPECT_FALSE(estimateVelocities(&estimate, makeJoints({ 1.0 }), makeJoints({ 0.0 }), 0.0));
    EXPECT_FALSE(estimateVelocities(&estimate, makeJoints({ 1.0 }), makeJoints({ 0.0 }), -1.0));
}

TEST(estimateVelocities, jointsStopsAtTheShorterInput) {
    const wrapper::Joints current = makeJoints({ 1.0, 2.0, 3.0 });
    const wrapper::Joints previous = makeJoints({ 0.0, 0.0 });

    wrapper::Joints estimate;
    EXPECT_TRUE(estimateVelocities(&estimate, current, previous, 1.0));

    EXPECT_EQ(2, estimate.values_size());
}

TEST(estimateVelocities, jointsClearsAnyPreviousContent) {
    wrapper::Joints estimate = makeJoints({ 9.0, 9.0, 9.0, 9.0 });
    EXPECT_TRUE(estimateVelocities(&estimate, makeJoints({ 1.0 }), makeJoints({ 0.0 }), 1.0));

    EXPECT_EQ(1, estimate.values_size());
}

TEST(estimateVelocities, angularLeavesTheZComponentAtZero) {
    wrapper::Euler estimate;
    EXPECT_TRUE(
        estimateVelocities(&estimate, makeQuaternion(1.0, 0.0, 0.0, 0.0), makeQuaternion(1.0, 0.0, 0.0, 0.0), 0.1)
    );

    EXPECT_DOUBLE_EQ(0.0, estimate.z());
}

TEST(estimateVelocities, angularFailsForANonPositiveSampleTime) {
    wrapper::Euler estimate;
    EXPECT_FALSE(
        estimateVelocities(&estimate, makeQuaternion(1.0, 0.0, 0.0, 0.0), makeQuaternion(1.0, 0.0, 0.0, 0.0), 0.0)
    );
}

TEST(estimateVelocities, cartesianCombinesLinearAndAngularEstimates) {
    wrapper::CartesianPose current;
    current.mutable_position()->set_x(1.0);
    current.mutable_position()->set_y(2.0);
    current.mutable_position()->set_z(3.0);
    current.mutable_quaternion()->set_u0(1.0);

    wrapper::CartesianPose previous;
    previous.mutable_position()->set_x(0.0);
    previous.mutable_position()->set_y(0.0);
    previous.mutable_position()->set_z(0.0);
    previous.mutable_quaternion()->set_u0(1.0);

    wrapper::CartesianVelocity estimate;
    EXPECT_TRUE(estimateVelocities(&estimate, current, previous, 0.5));

    EXPECT_DOUBLE_EQ(2.0, estimate.linear().x());
    EXPECT_DOUBLE_EQ(4.0, estimate.linear().y());
    EXPECT_DOUBLE_EQ(6.0, estimate.linear().z());
}

// --- findMaxDifference ---

TEST(findMaxDifference, jointsReturnsZeroForIdenticalInputs) {
    EXPECT_DOUBLE_EQ(0.0, findMaxDifference(makeJoints({ 1.0, 2.0 }), makeJoints({ 1.0, 2.0 })));
}

TEST(findMaxDifference, jointsReturnsTheLargestAbsoluteDifference) {
    EXPECT_DOUBLE_EQ(3.0, findMaxDifference(makeJoints({ 1.0, 2.0, 3.0 }), makeJoints({ 1.0, 5.0, 1.0 })));
}

TEST(findMaxDifference, jointsOnlyComparesTheOverlappingPrefix) {
    EXPECT_DOUBLE_EQ(2.0, findMaxDifference(makeJoints({ 1.0, 2.0 }), makeJoints({ 1.0, 4.0, 100.0 })));
}

TEST(findMaxDifference, cartesianReturnsTheLargestComponentDifference) {
    wrapper::Cartesian c1;
    c1.set_x(1.0);
    c1.set_y(2.0);
    c1.set_z(3.0);

    wrapper::Cartesian c2;
    c2.set_x(1.5);
    c2.set_y(2.0);
    c2.set_z(0.0);

    EXPECT_DOUBLE_EQ(3.0, findMaxDifference(c1, c2));
}

TEST(findMaxDifference, eulerReturnsTheLargestComponentDifference) {
    EXPECT_DOUBLE_EQ(10.0, findMaxDifference(makeEuler(1.0, 2.0, 3.0), makeEuler(1.0, -8.0, 3.0)));
}

// --- copyPresent ---

TEST(copyPresent, jointsStopsAtTheTargetSize) {
    const wrapper::Joints source = makeJoints({ 1.0, 2.0, 3.0, 4.0 });

    wrapper::Joints target = makeJoints({ 0.0, 0.0 });
    copyPresent(&target, source);

    ASSERT_EQ(2, target.values_size());
    EXPECT_DOUBLE_EQ(1.0, target.values(0));
    EXPECT_DOUBLE_EQ(2.0, target.values(1));
}

TEST(copyPresent, cartesianCopiesOnlyThePresentFields) {
    wrapper::Cartesian source;
    source.set_x(1.0);
    source.set_z(3.0);

    wrapper::Cartesian target;
    target.set_y(9.0);
    copyPresent(&target, source);

    EXPECT_DOUBLE_EQ(1.0, target.x());
    EXPECT_DOUBLE_EQ(9.0, target.y());
    EXPECT_DOUBLE_EQ(3.0, target.z());
}

TEST(copyPresent, quaternionNormalizesTheTarget) {
    wrapper::Quaternion source = makeQuaternion(0.0, 0.0, 3.0, 4.0);

    wrapper::Quaternion target;
    copyPresent(&target, source);

    EXPECT_NEAR(1.0, euclideanNorm(target), TOLERANCE);
}

TEST(copyPresent, cartesianPosePrefersTheEulerSource) {
    wrapper::CartesianPose source;
    source.mutable_euler()->set_x(10.0);
    source.mutable_euler()->set_y(20.0);
    source.mutable_euler()->set_z(30.0);
    // The quaternion field is present too, but the euler branch takes precedence.
    source.mutable_quaternion()->set_u0(0.5);

    wrapper::CartesianPose target;
    copyPresent(&target, source);

    EXPECT_TRUE(target.has_euler());
    EXPECT_DOUBLE_EQ(10.0, target.euler().x());
    EXPECT_DOUBLE_EQ(20.0, target.euler().y());
    EXPECT_DOUBLE_EQ(30.0, target.euler().z());
    // The quaternion is derived from the copied euler angles.
    EXPECT_TRUE(target.has_quaternion());
    EXPECT_NEAR(1.0, euclideanNorm(target.quaternion()), TOLERANCE);
}

TEST(copyPresent, cartesianPoseDerivesTheEulerAnglesFromAQuaternionSource) {
    wrapper::CartesianPose source;
    source.mutable_quaternion()->set_u0(1.0);

    wrapper::CartesianPose target;
    copyPresent(&target, source);

    EXPECT_TRUE(target.has_quaternion());
    EXPECT_TRUE(target.has_euler());
    EXPECT_NEAR(0.0, target.euler().x(), TOLERANCE);
    EXPECT_NEAR(0.0, target.euler().y(), TOLERANCE);
    EXPECT_NEAR(0.0, target.euler().z(), TOLERANCE);
}

TEST(copyPresent, nestedMessagesRecurseOnlyIntoPresentFields) {
    wrapper::Robot source;
    source.mutable_joints()->mutable_position()->add_values(1.0);
    source.mutable_joints()->mutable_position()->add_values(2.0);

    // The joints overload writes into the elements the target already has, so it must be sized up front.
    wrapper::Robot target;
    target.mutable_joints()->mutable_position()->add_values(0.0);
    target.mutable_joints()->mutable_position()->add_values(0.0);
    copyPresent(&target, source);

    EXPECT_TRUE(target.has_joints());
    EXPECT_TRUE(target.joints().has_position());
    EXPECT_FALSE(target.joints().has_velocity());
    EXPECT_FALSE(target.has_cartesian());
    ASSERT_EQ(2, target.joints().position().values_size());
    EXPECT_DOUBLE_EQ(1.0, target.joints().position().values(0));
    EXPECT_DOUBLE_EQ(2.0, target.joints().position().values(1));
}

TEST(copyPresent, jointSpaceCopiesPositionAndVelocityIndependently) {
    wrapper::JointSpace source;
    source.mutable_position()->add_values(1.0);

    wrapper::JointSpace target;
    copyPresent(&target, source);

    EXPECT_TRUE(target.has_position());
    EXPECT_FALSE(target.has_velocity());
}

TEST(copyPresent, outputRecursesIntoRobotAndExternal) {
    wrapper::Output source;
    source.mutable_robot()->mutable_joints()->mutable_position()->add_values(1.0);
    source.mutable_external()->mutable_joints()->mutable_position()->add_values(2.0);

    wrapper::Output target;
    target.mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
    target.mutable_external()->mutable_joints()->mutable_position()->add_values(0.0);
    copyPresent(&target, source);

    EXPECT_TRUE(target.has_robot());
    EXPECT_TRUE(target.has_external());
    ASSERT_EQ(1, target.robot().joints().position().values_size());
    EXPECT_DOUBLE_EQ(1.0, target.robot().joints().position().values(0));
    EXPECT_DOUBLE_EQ(2.0, target.external().joints().position().values(0));
}

TEST(copyPresent, toleratesNullTargets) {
    copyPresent(static_cast<wrapper::Joints*>(nullptr), makeJoints({ 1.0 }));
    copyPresent(static_cast<wrapper::JointSpace*>(nullptr), wrapper::JointSpace());
    copyPresent(static_cast<wrapper::Cartesian*>(nullptr), wrapper::Cartesian());
    copyPresent(static_cast<wrapper::Euler*>(nullptr), wrapper::Euler());
    copyPresent(static_cast<wrapper::Quaternion*>(nullptr), wrapper::Quaternion());
    copyPresent(static_cast<wrapper::CartesianPose*>(nullptr), wrapper::CartesianPose());
    copyPresent(static_cast<wrapper::CartesianVelocity*>(nullptr), wrapper::CartesianVelocity());
    copyPresent(static_cast<wrapper::CartesianSpace*>(nullptr), wrapper::CartesianSpace());
    copyPresent(static_cast<wrapper::Robot*>(nullptr), wrapper::Robot());
    copyPresent(static_cast<wrapper::External*>(nullptr), wrapper::External());
    copyPresent(static_cast<wrapper::Output*>(nullptr), wrapper::Output());
}

// --- reset ---

TEST(reset, jointsClearsAndRefillsWithZeros) {
    wrapper::Joints joints = makeJoints({ 1.0, 2.0, 3.0 });
    reset(&joints, 4);

    ASSERT_EQ(4, joints.values_size());
    for (int i = 0; i < joints.values_size(); ++i) { EXPECT_DOUBLE_EQ(0.0, joints.values(i)); }
}

TEST(reset, cartesianAndEulerZeroAllComponents) {
    wrapper::Cartesian cartesian;
    cartesian.set_x(1.0);
    cartesian.set_y(2.0);
    cartesian.set_z(3.0);
    reset(&cartesian);
    EXPECT_DOUBLE_EQ(0.0, cartesian.x());
    EXPECT_DOUBLE_EQ(0.0, cartesian.y());
    EXPECT_DOUBLE_EQ(0.0, cartesian.z());

    wrapper::Euler euler = makeEuler(1.0, 2.0, 3.0);
    reset(&euler);
    EXPECT_DOUBLE_EQ(0.0, euler.x());
    EXPECT_DOUBLE_EQ(0.0, euler.y());
    EXPECT_DOUBLE_EQ(0.0, euler.z());
}

// --- verify ---

TEST(verify, rejectsNaNAndInfinityForDoubles) {
    EXPECT_TRUE(verify(0.0));
    EXPECT_FALSE(verify(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(verify(std::numeric_limits<double>::infinity()));
}

TEST(verify, jointsChecksEveryValue) {
    EXPECT_TRUE(verify(makeJoints({ 1.0, 2.0 })));

    wrapper::Joints joints = makeJoints({ 1.0 });
    joints.add_values(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(verify(joints));
}

TEST(verify, cartesianEulerAndQuaternionCheckOnlyThePresentFields) {
    wrapper::Cartesian cartesian;
    cartesian.set_x(1.0);
    EXPECT_TRUE(verify(cartesian));

    wrapper::Euler euler;
    euler.set_y(std::numeric_limits<double>::infinity());
    EXPECT_FALSE(verify(euler));

    wrapper::Quaternion quaternion;
    quaternion.set_u3(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(verify(quaternion));
}

TEST(verify, cartesianPoseAndVelocityRecurseIntoTheirSubMessages) {
    wrapper::CartesianPose pose;
    pose.mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(verify(pose));

    wrapper::CartesianVelocity velocity;
    velocity.mutable_linear()->set_z(std::numeric_limits<double>::infinity());
    EXPECT_FALSE(verify(velocity));

    wrapper::CartesianVelocity valid;
    valid.mutable_linear()->set_z(1.0);
    EXPECT_TRUE(verify(valid));
}

} // namespace
