// SPDX-License-Identifier: MIT
///
/// @file interpolator_benchmark.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Micro benchmarks for the interpolation classes declared in interpolator.hxx.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <benchmark/benchmark.h>

#include "common.hxx"
#include "egm_wrapper_trajectory.pb.h"
#include "interpolator.hxx"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr int NUMBER_OF_JOINTS = DEFAULT_NUM_OF_ROBOT_JOINTS;

wrapper::trajectory::JointGoal makeJointGoal(const double position, const double velocity, const double acceleration) {
    wrapper::trajectory::JointGoal goal;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        goal.mutable_position()->add_values(position + i);
        goal.mutable_velocity()->add_values(velocity);
        goal.mutable_acceleration()->add_values(acceleration);
    }

    return goal;
}

wrapper::trajectory::PointGoal makePointGoal(const double position, const double velocity, const double acceleration) {
    wrapper::trajectory::PointGoal goal;
    goal.set_duration(0.5);
    *goal.mutable_robot()->mutable_joints() = makeJointGoal(position, velocity, acceleration);

    return goal;
}

wrapper::Quaternion makeQuaternion(const double u0, const double u1, const double u2, const double u3) {
    wrapper::Quaternion quaternion;
    quaternion.set_u0(u0);
    quaternion.set_u1(u1);
    quaternion.set_u2(u2);
    quaternion.set_u3(u3);

    return quaternion;
}

wrapper::trajectory::CartesianGoal makeCartesianGoal() {
    wrapper::trajectory::CartesianGoal goal;
    goal.mutable_pose()->mutable_position()->set_x(0.0);
    goal.mutable_pose()->mutable_position()->set_y(0.0);
    goal.mutable_pose()->mutable_position()->set_z(0.0);
    *goal.mutable_pose()->mutable_quaternion() = makeQuaternion(1.0, 0.0, 0.0, 0.0);
    goal.mutable_velocity()->set_x(0.0);
    goal.mutable_velocity()->set_y(0.0);
    goal.mutable_velocity()->set_z(0.0);
    goal.mutable_acceleration()->set_x(0.0);
    goal.mutable_acceleration()->set_y(0.0);
    goal.mutable_acceleration()->set_z(0.0);

    return goal;
}

Conditions makeConditions(const Operation operation, const double duration) {
    Conditions conditions;
    conditions.duration = duration;
    conditions.operation = operation;
    conditions.spline_method = Quintic;

    return conditions;
}

// --- SplinePolynomial ---

void splinePolynomialUpdate(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::Normal, 0.5);
    const wrapper::trajectory::JointGoal start = makeJointGoal(0.0, 0.0, 0.0);
    const wrapper::trajectory::JointGoal goal = makeJointGoal(90.0, 0.0, 0.0);

    wrapper::trajectory::JointGoal output = makeJointGoal(0.0, 0.0, 0.0);

    for (auto _ : state) {
        SplineConditions spline_conditions(conditions);
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { spline_conditions.setConditions(i, start, goal); }

        SplinePolynomial polynomial;
        polynomial.update(spline_conditions);

        // The polynomial is consumed through its evaluation. DoNotOptimize can only name the scalar
        // register or memory operands its inline asm accepts, never a class object, so reading the
        // coefficients back is what keeps the update itself from being optimized away.
        polynomial.evaluate(&output, 0, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(splinePolynomialUpdate)->Unit(benchmark::kMicrosecond);

void splinePolynomialEvaluate(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::Normal, 0.5);
    const wrapper::trajectory::JointGoal start = makeJointGoal(0.0, 0.0, 0.0);
    const wrapper::trajectory::JointGoal goal = makeJointGoal(90.0, 0.0, 0.0);

    SplineConditions spline_conditions(conditions);
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { spline_conditions.setConditions(i, start, goal); }

    SplinePolynomial polynomial;
    polynomial.update(spline_conditions);

    wrapper::trajectory::JointGoal output = makeJointGoal(0.0, 0.0, 0.0);

    for (auto _ : state) {
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { polynomial.evaluate(&output, i, 0.25); }
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(splinePolynomialEvaluate)->Unit(benchmark::kNanosecond);

// --- Slerp ---

void slerpUpdateAndEvaluate(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::Normal, 0.5);
    const wrapper::Quaternion start = makeQuaternion(1.0, 0.0, 0.0, 0.0);
    const wrapper::Quaternion goal = makeQuaternion(0.7071, 0.0, 0.0, 0.7071);

    wrapper::trajectory::CartesianGoal output = makeCartesianGoal();

    for (auto _ : state) {
        Slerp slerp;
        slerp.update(start, goal, conditions);
        slerp.evaluate(&output, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(slerpUpdateAndEvaluate)->Unit(benchmark::kNanosecond);

// --- SoftRamp ---

void softRampEvaluateJoints(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::RampInPosition, 0.5);
    const wrapper::trajectory::PointGoal start = makePointGoal(0.0, 0.0, 0.0);
    const wrapper::trajectory::PointGoal goal = makePointGoal(90.0, 0.0, 0.0);

    SoftRamp ramp;
    ramp.update(start, goal, conditions);

    wrapper::trajectory::JointGoal output = makeJointGoal(0.0, 0.0, 0.0);

    for (auto _ : state) {
        ramp.evaluate(&output, true, LOWEST_SAMPLE_TIME, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(softRampEvaluateJoints)->Unit(benchmark::kNanosecond);

void softRampEvaluateCartesian(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::RampInPosition, 0.5);

    wrapper::trajectory::PointGoal start = makePointGoal(0.0, 0.0, 0.0);
    *start.mutable_robot()->mutable_cartesian() = makeCartesianGoal();
    wrapper::trajectory::PointGoal goal = makePointGoal(90.0, 0.0, 0.0);
    *goal.mutable_robot()->mutable_cartesian() = makeCartesianGoal();

    SoftRamp ramp;
    ramp.update(start, goal, conditions);

    wrapper::trajectory::CartesianGoal output = makeCartesianGoal();

    for (auto _ : state) {
        ramp.evaluate(&output, LOWEST_SAMPLE_TIME, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(softRampEvaluateCartesian)->Unit(benchmark::kNanosecond);

// --- EGMInterpolator ---

void interpolatorUpdate(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::Normal, 0.5);
    const wrapper::trajectory::PointGoal start = makePointGoal(0.0, 0.0, 0.0);
    const wrapper::trajectory::PointGoal goal = makePointGoal(90.0, 0.0, 0.0);

    wrapper::trajectory::PointGoal output = makePointGoal(0.0, 0.0, 0.0);

    for (auto _ : state) {
        EGMInterpolator interpolator;
        interpolator.update(start, goal, conditions);

        // See splinePolynomialUpdate: a class object cannot be handed to DoNotOptimize, so the
        // interpolator is consumed through a single evaluation instead.
        interpolator.evaluate(&output, LOWEST_SAMPLE_TIME, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(interpolatorUpdate)->Unit(benchmark::kMicrosecond);

void interpolatorEvaluate(benchmark::State& state) {
    const Conditions conditions = makeConditions(Operation::Normal, 0.5);
    const wrapper::trajectory::PointGoal start = makePointGoal(0.0, 0.0, 0.0);
    const wrapper::trajectory::PointGoal goal = makePointGoal(90.0, 0.0, 0.0);

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    // The interpolator writes into the elements the output already has, so it is sized up front.
    wrapper::trajectory::PointGoal output = makePointGoal(0.0, 0.0, 0.0);

    for (auto _ : state) {
        interpolator.evaluate(&output, LOWEST_SAMPLE_TIME, 0.25);
        benchmark::DoNotOptimize(output);
    }
}

BENCHMARK(interpolatorEvaluate)->Unit(benchmark::kNanosecond);

} // namespace
