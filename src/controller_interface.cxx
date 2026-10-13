// SPDX-License-Identifier: MIT
///
/// @file controller_interface.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2026-10-12
/// @copyright Copyright (c) 2026
///

#include "ba7lya/abb/robot/egm/controller_interface.hxx"

#include <sstream>

#include "ba7lya/abb/robot/egm/common_auxiliary.hxx"

namespace ba7lya::abb::robot::egm {

// See https://stackoverflow.com/questions/16957458/static-const-in-c-class-undefined-reference/16957554
const unsigned int ControllerMotion::WRITE_TIMEOUT_MS;

void ControllerMotion::initialize(const bool first_message) {
    if (first_message) {
        std::lock_guard<std::mutex> lock(read_mutex_);
        std::lock_guard<std::mutex> lock2(write_mutex_);

        read_data_ready_ = false;
        write_data_ready_ = false;
    }
}

void ControllerMotion::writeInputs(const wrapper::Input& inputs) {
    std::lock_guard<std::mutex> lock(read_mutex_);

    inputs_.CopyFrom(inputs);

    read_data_ready_ = true;
    read_condition_variable_.notify_all();
}

void ControllerMotion::readOutputs(wrapper::Output* p_outputs) {
    bool timed_out = false;

    std::unique_lock<std::mutex> lock(write_mutex_);

    while (!write_data_ready_ && !timed_out) {
        timed_out
            = (write_condition_variable_.wait_for(lock, std::chrono::milliseconds(WRITE_TIMEOUT_MS))
               == std::cv_status::timeout);
    }

    if (!timed_out && p_outputs) {
        copyPresent(p_outputs, outputs_);
        write_data_ready_ = false;
    }
}

bool ControllerMotion::waitForMessage(const unsigned int timeout_ms) {
    std::unique_lock<std::mutex> lock(read_mutex_);

    bool timed_out = false;

    while (!read_data_ready_ && !timed_out) {
        if (timeout_ms <= 0) { read_condition_variable_.wait(lock); }
        else {
            timed_out
                = (read_condition_variable_.wait_for(lock, std::chrono::milliseconds(timeout_ms))
                   == std::cv_status::timeout);
        }
    }

    return !timed_out;
}

void ControllerMotion::readInputs(wrapper::Input* p_inputs) {
    std::lock_guard<std::mutex> lock(read_mutex_);

    p_inputs->CopyFrom(inputs_);
    read_data_ready_ = false;
}

void ControllerMotion::writeOutputs(const wrapper::Output& outputs) {
    std::lock_guard<std::mutex> lock(write_mutex_);

    outputs_.CopyFrom(outputs);

    write_data_ready_ = true;
    write_condition_variable_.notify_all();
}

EGMControllerInterface::EGMControllerInterface(
    asio::io_context& io_service, const unsigned short port_number, const BaseConfiguration& configuration
)
    : EGMBaseInterface(io_service, port_number, configuration) {
    if (configuration_.active.use_logging) {
        std::stringstream ss;
        ss << "port_" << port_number << +"_log.csv";
        p_logger_.reset(new EGMLogger(ss.str()));
    }
}

const std::string& EGMControllerInterface::callback(const UDPServerData& server_data) {
    // Initialize the callback by:
    // - Parsing and extracting data from the received message.
    // - Updating any pending configuration changes.
    // - Preparing the outputs.
    if (initializeCallback(server_data)) {
        // Additional initialization for direct motion references.
        controller_motion_.initialize(inputs_.isFirstMessage());

        // Handle demo execution or external controller execution.
        if (configuration_.active.use_demo_outputs) { outputs_.generateDemoOutputs(inputs_); }
        else {
            // Make the current inputs available (to the external control loop), and notify that it is available.
            controller_motion_.writeInputs(inputs_.current());

            // Send a notification via the (optional) external condition variable.
            if (configuration_.active.p_new_message_cv) { configuration_.active.p_new_message_cv->notify_all(); }

            if (inputs_.isFirstMessage() || inputs_.statesOk()) {
                // Wait for new outputs (from the external control loop), or until a timeout occurs.
                controller_motion_.readOutputs(&outputs_.current);
            }
        }

        // Log inputs and outputs, if set to do so.
        if (configuration_.active.use_logging && p_logger_) {
            logData(inputs_, outputs_, configuration_.active.max_logging_duration);
        }

        // Constuct the reply message.
        outputs_.constructReply(configuration_.active);

        // Prepare for the next callback.
        inputs_.updatePrevious();
        outputs_.updatePrevious();
    }

    // Return the reply.
    return outputs_.reply();
}

bool EGMControllerInterface::waitForMessage(const unsigned int timeout_ms) {
    return controller_motion_.waitForMessage(timeout_ms);
}

void EGMControllerInterface::read(wrapper::Input* p_inputs) { controller_motion_.readInputs(p_inputs); }

void EGMControllerInterface::write(const wrapper::Output& outputs) { controller_motion_.writeOutputs(outputs); }

} // namespace ba7lya::abb::robot::egm
