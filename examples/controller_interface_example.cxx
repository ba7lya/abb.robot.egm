// SPDX-License-Identifier: MIT
///
/// @file controller_interface_example.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Example of an external control loop built on EGMControllerInterface.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///
/// The interface mirrors the robot controller's feedback, and adds a slow oscillation to the
/// last robot joint. The asynchronous EGM communication runs on the asio io_context, while the
/// control loop below runs on its own thread. The interface's callback blocks until the control
/// loop has written its outputs, so the two must not share a thread.
///

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "common.hxx"
#include "controller_interface.hxx"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr double OSCILLATION_AMPLITUDE = 10.0; ///< Oscillation amplitude [degrees].
constexpr double OSCILLATION_FREQUENCY = 0.25; ///< Oscillation frequency [Hz].

/// @brief Make a copy of the joint references the robot controller most recently reported.
wrapper::Output makeMirrorOutput(const wrapper::Input& inputs) {
    const wrapper::Feedback& feedback = inputs.feedback();

    wrapper::Output outputs;
    outputs.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(feedback.robot().joints().position());
    outputs.mutable_external()->mutable_joints()->mutable_position()->CopyFrom(feedback.external().joints().position());

    return outputs;
}

/// @brief Add a slow oscillation to the last robot joint.
/// @param p_outputs for the outputs to modify.
/// @param t for the time instance [s] of the oscillation.
/// @param with_velocity indicating if matching velocity references should be added as well.
void applyOscillation(wrapper::Output* p_outputs, const double t, const bool with_velocity) {
    wrapper::Joints* p_position = p_outputs->mutable_robot()->mutable_joints()->mutable_position();
    const int number_of_joints = p_position->values_size();
    if (number_of_joints <= 0) { return; }

    const double omega = 2.0 * PI * OSCILLATION_FREQUENCY;
    const int last_joint = number_of_joints - 1;
    p_position->set_values(last_joint, p_position->values(last_joint) + OSCILLATION_AMPLITUDE * std::sin(omega * t));

    if (!with_velocity) { return; }

    wrapper::Joints* p_velocity = p_outputs->mutable_robot()->mutable_joints()->mutable_velocity();
    for (int i = 0; i < number_of_joints; ++i) { p_velocity->add_values(0.0); }
    p_velocity->set_values(last_joint, OSCILLATION_AMPLITUDE * omega * std::cos(omega * t));
}

/// @brief Run the external control loop until the EGM communication session ends.
void runControlLoop(EGMControllerInterface& interface) {
    const bool with_velocity = interface.getConfiguration().use_velocity_outputs;
    double t = 0.0;

    while (interface.isConnected()) {
        if (!interface.waitForMessage(1000)) {
            std::printf("Waiting for the robot controller...\n");
            continue;
        }

        wrapper::Input inputs;
        interface.read(&inputs);

        wrapper::Output outputs = makeMirrorOutput(inputs);
        applyOscillation(&outputs, t, with_velocity);

        interface.write(outputs);

        t += LOWEST_SAMPLE_TIME;
    }
}

} // namespace

int main(const int argc, char* argv[]) {
    const unsigned short port_number
        = (argc > 1) ? static_cast<unsigned short>(std::stoi(argv[1])) : DEFAULT_PORT_NUMBER;

    asio::io_context io_service;
    EGMControllerInterface interface(io_service, port_number);

    if (!interface.isInitialized()) {
        std::printf("Failed to bind the UDP server to port %u\n", port_number);
        return -1;
    }

    std::printf("EGM controller interface listening on port %u\n", port_number);
    std::printf("Run an EGMActJoint or EGMActPose RAPID instruction against this port.\n");

    // The interface's callback runs on the io_context and blocks until the control loop replies.
    std::thread control_loop(runControlLoop, std::ref(interface));
    io_service.run();
    control_loop.join();

    std::printf("EGM communication session ended.\n");

    return 0;
}
