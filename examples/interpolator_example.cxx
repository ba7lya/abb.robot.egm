// SPDX-License-Identifier: MIT
///
/// @file interpolator_example.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Offline example of the interpolator classes. No EGM communication is involved.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <cstdio>

#include "common.hxx"
#include "egm_wrapper_trajectory.pb.h"
#include "interpolator.hxx"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr int NUMBER_OF_JOINTS = DEFAULT_NUM_OF_ROBOT_JOINTS;
constexpr double GOAL_DURATION = 2.0;
constexpr double SAMPLE_TIME = 0.1;
constexpr double START_ANGLE = 0.0;
constexpr double GOAL_ANGLE = 90.0;

/// @brief Build a joint goal in which every joint is at the same position.
/// @note Position, velocity and acceleration must all be sized, otherwise the spline conditions
///       cannot be extracted for joint indices that are missing.
wrapper::trajectory::JointGoal makeJointGoal(const double position) {
    wrapper::trajectory::JointGoal goal;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        goal.mutable_position()->add_values(position);
        goal.mutable_velocity()->add_values(0.0);
        goal.mutable_acceleration()->add_values(0.0);
    }

    return goal;
}

wrapper::trajectory::PointGoal makePointGoal(const double position) {
    wrapper::trajectory::PointGoal goal;
    goal.set_duration(GOAL_DURATION);
    *goal.mutable_robot()->mutable_joints() = makeJointGoal(position);

    return goal;
}

void runInterpolation() {
    const wrapper::trajectory::PointGoal start = makePointGoal(START_ANGLE);
    const wrapper::trajectory::PointGoal goal = makePointGoal(GOAL_ANGLE);

    Conditions conditions;
    conditions.duration = GOAL_DURATION;
    conditions.mode = EGMJoint;
    conditions.operation = Operation::Normal;
    conditions.spline_method = Quintic;

    EGMInterpolator interpolator;
    interpolator.update(start, goal, conditions);

    // The interpolator writes into the elements the output already holds, so it is sized up front.
    wrapper::trajectory::PointGoal output = makePointGoal(START_ANGLE);

    std::printf("Duration: %.3f s\n", interpolator.getDuration());
    std::printf("%8s", "t [s]");
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) { std::printf("%12s%d", "joint ", i + 1); }
    std::printf("\n");

    // The sample time is a multiple of LOWEST_SAMPLE_TIME, so the sample index is integral.
    const int number_of_samples = static_cast<int>(interpolator.getDuration() / SAMPLE_TIME) + 1;
    for (int sample = 0; sample < number_of_samples; ++sample) {
        const double t = static_cast<double>(sample) * SAMPLE_TIME;
        interpolator.evaluate(&output, LOWEST_SAMPLE_TIME, t);

        std::printf("%8.3f", t);
        for (int i = 0; i < output.robot().joints().position().values_size(); ++i) {
            std::printf("%12.3f", output.robot().joints().position().values(i));
        }
        std::printf("\n");
    }
}

} // namespace

int main() {
    std::printf("EGM interpolator example (offline, no communication)\n\n");
    runInterpolation();

    return 0;
}
