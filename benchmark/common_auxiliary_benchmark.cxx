// SPDX-License-Identifier: MIT
///
/// @file common_auxiliary_benchmark.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Micro benchmarks for the free functions declared in common_auxiliary.hxx.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <benchmark/benchmark.h>

#include "common.hxx"
#include "common_auxiliary.hxx"
#include "egm.pb.h"
#include "egm_wrapper.pb.h"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr int NUMBER_OF_JOINTS = DEFAULT_NUM_OF_ROBOT_JOINTS;

wrapper::Quaternion makeQuaternion(const double u0, const double u1, const double u2, const double u3) {
    wrapper::Quaternion q;
    q.set_u0(u0);
    q.set_u1(u1);
    q.set_u2(u2);
    q.set_u3(u3);

    return q;
}

wrapper::Euler makeEuler(const double x, const double y, const double z) {
    wrapper::Euler euler;
    euler.set_x(x);
    euler.set_y(y);
    euler.set_z(z);

    return euler;
}

wrapper::Joints makeJoints(const double value) {
    wrapper::Joints joints;
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { joints.add_values(value + i); }

    return joints;
}

wrapper::CartesianPose makeCartesianPose(const double offset) {
    wrapper::CartesianPose pose;
    pose.mutable_position()->set_x(offset);
    pose.mutable_position()->set_y(offset);
    pose.mutable_position()->set_z(offset);
    *pose.mutable_quaternion() = makeQuaternion(1.0, 0.0, 0.0, 0.0);

    return pose;
}

// Prepare an inbound feedback message whose every field the parser is able to consume.
EgmFeedBack makeEgmFeedback() {
    EgmFeedBack source;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { source.mutable_joints()->add_joints(i * 10.0); }
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { source.mutable_externaljoints()->add_joints(i * 5.0); }

    source.mutable_cartesian()->mutable_pos()->set_x(100.0);
    source.mutable_cartesian()->mutable_pos()->set_y(200.0);
    source.mutable_cartesian()->mutable_pos()->set_z(300.0);
    source.mutable_cartesian()->mutable_orient()->set_u0(1.0);
    source.mutable_cartesian()->mutable_orient()->set_u1(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u2(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u3(0.0);
    source.mutable_time()->set_sec(1);
    source.mutable_time()->set_usec(2);

    return source;
}

// --- saturate ---

void saturateValue(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(saturate(0.25, 0.0, 1.0));
        benchmark::DoNotOptimize(saturate(1.25, 0.0, 1.0));
    }
}

BENCHMARK(saturateValue)->Unit(benchmark::kNanosecond);

// --- convert ---

void convertEulerToQuaternion(benchmark::State& state) {
    const wrapper::Euler euler = makeEuler(10.0, -20.0, 30.0);
    wrapper::Quaternion q;

    for (auto _ : state) {
        convert(&q, euler);
        benchmark::DoNotOptimize(q);
    }
}

BENCHMARK(convertEulerToQuaternion)->Unit(benchmark::kNanosecond);

void convertQuaternionToEuler(benchmark::State& state) {
    wrapper::Quaternion q;
    convert(&q, makeEuler(10.0, -20.0, 30.0));
    wrapper::Euler euler;

    for (auto _ : state) {
        convert(&euler, q);
        benchmark::DoNotOptimize(euler);
    }
}

BENCHMARK(convertQuaternionToEuler)->Unit(benchmark::kNanosecond);

// --- normalize / multiply ---

void normalizeQuaternion(benchmark::State& state) {
    for (auto _ : state) {
        wrapper::Quaternion q = makeQuaternion(0.1, 0.2, 0.3, 0.4);
        normalize(&q);
        benchmark::DoNotOptimize(q);
    }
}

BENCHMARK(normalizeQuaternion)->Unit(benchmark::kNanosecond);

void multiplyQuaternions(benchmark::State& state) {
    const wrapper::Quaternion q1 = makeQuaternion(0.1, 0.2, 0.3, 0.4);
    const wrapper::Quaternion q2 = makeQuaternion(0.4, 0.3, 0.2, 0.1);

    for (auto _ : state) {
        const wrapper::Quaternion product = multiply(q1, q2);
        benchmark::DoNotOptimize(product);
    }
}

BENCHMARK(multiplyQuaternions)->Unit(benchmark::kNanosecond);

void multiplyJoints(benchmark::State& state) {
    for (auto _ : state) {
        wrapper::Joints joints = makeJoints(0.0);
        multiply(&joints, 0.5);
        benchmark::DoNotOptimize(joints);
    }
}

BENCHMARK(multiplyJoints)->Unit(benchmark::kNanosecond);

// --- estimateVelocities ---

void estimateJointVelocities(benchmark::State& state) {
    const wrapper::Joints current = makeJoints(1.0);
    const wrapper::Joints previous = makeJoints(0.0);
    wrapper::Joints estimate;

    for (auto _ : state) {
        benchmark::DoNotOptimize(estimateVelocities(&estimate, current, previous, LOWEST_SAMPLE_TIME));
        benchmark::DoNotOptimize(estimate);
    }
}

BENCHMARK(estimateJointVelocities)->Unit(benchmark::kNanosecond);

void estimateCartesianVelocities(benchmark::State& state) {
    const wrapper::CartesianPose current = makeCartesianPose(1.0);
    const wrapper::CartesianPose previous = makeCartesianPose(0.0);
    wrapper::CartesianVelocity estimate;

    for (auto _ : state) {
        benchmark::DoNotOptimize(estimateVelocities(&estimate, current, previous, LOWEST_SAMPLE_TIME));
        benchmark::DoNotOptimize(estimate);
    }
}

BENCHMARK(estimateCartesianVelocities)->Unit(benchmark::kNanosecond);

// --- findMaxDifference ---

void findMaxDifferenceJoints(benchmark::State& state) {
    const wrapper::Joints j1 = makeJoints(0.0);
    const wrapper::Joints j2 = makeJoints(1.0);

    for (auto _ : state) { benchmark::DoNotOptimize(findMaxDifference(j1, j2)); }
}

BENCHMARK(findMaxDifferenceJoints)->Unit(benchmark::kNanosecond);

// --- copyPresent ---

void copyPresentJoints(benchmark::State& state) {
    const wrapper::Joints source = makeJoints(1.0);

    for (auto _ : state) {
        wrapper::Joints target = makeJoints(0.0);
        copyPresent(&target, source);
        benchmark::DoNotOptimize(target);
    }
}

BENCHMARK(copyPresentJoints)->Unit(benchmark::kNanosecond);

void copyPresentOutput(benchmark::State& state) {
    wrapper::Output source;
    *source.mutable_robot()->mutable_joints()->mutable_position() = makeJoints(1.0);
    *source.mutable_robot()->mutable_joints()->mutable_velocity() = makeJoints(0.0);
    *source.mutable_external()->mutable_joints()->mutable_position() = makeJoints(2.0);

    for (auto _ : state) {
        // The joints overloads only write into elements the target already holds.
        wrapper::Output target;
        *target.mutable_robot()->mutable_joints()->mutable_position() = makeJoints(0.0);
        *target.mutable_robot()->mutable_joints()->mutable_velocity() = makeJoints(0.0);
        *target.mutable_external()->mutable_joints()->mutable_position() = makeJoints(0.0);

        copyPresent(&target, source);
        benchmark::DoNotOptimize(target);
    }
}

BENCHMARK(copyPresentOutput)->Unit(benchmark::kNanosecond);

// --- parse ---

void parseFeedback(benchmark::State& state) {
    const EgmFeedBack source = makeEgmFeedback();
    wrapper::Feedback target;

    for (auto _ : state) {
        benchmark::DoNotOptimize(parse(&target, source, Six));
        benchmark::DoNotOptimize(target);
    }
}

BENCHMARK(parseFeedback)->Unit(benchmark::kNanosecond);

// --- verify ---

void verifyOutput(benchmark::State& state) {
    wrapper::Joints joints = makeJoints(1.0);

    for (auto _ : state) { benchmark::DoNotOptimize(verify(joints)); }
}

BENCHMARK(verifyOutput)->Unit(benchmark::kNanosecond);

} // namespace
