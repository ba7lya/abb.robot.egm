// SPDX-License-Identifier: MIT
///
/// @file controller_interface.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2026-10-12
/// @copyright Copyright (c) 2026
///

#pragma once

#include "base_interface.hxx"

namespace ba7lya::abb::robot::egm {

///
/// @brief Class for managing controller motion data, between the inner loop and an external control loop.
///
class ControllerMotion {
public:
    /// @brief Default constructor.
    ControllerMotion()
        : read_data_ready_(false)
        , write_data_ready_(false) {}

    /// @brief Initialize the motion data for a new communication session.
    /// @param first_message indicating if it is the first message in a communication session.
    void initialize(const bool first_message);

    /// @brief Wait for the next message.
    /// @param timeout_ms for specifying a timeout in [ms]. If omitted, then the method waits forever.
    /// @return bool indicating if the wait was successful or not. I.e. returns false if a timeout has occurred.
    bool waitForMessage(const unsigned int timeout_ms);

    /// @brief Write the current inputs (from the inner loop, to the intermediate storage).
    /// @param inputs for containing the inputs.
    void writeInputs(const wrapper::Input& inputs);

    /// @brief Read the current inputs (from the intermediate storage, to the external loop).
    /// @param p_inputs for containing the inputs.
    void readInputs(wrapper::Input* p_inputs);

    /// @brief Write the current outputs (from the external loop, to the intermediate storage).
    /// @param outputs for containing the outputs.
    void writeOutputs(const wrapper::Output& outputs);

    /// @brief Read the current outputs (from the intermediate storage, to the inner loop).
    /// @param p_outputs for containing the outputs.
    void readOutputs(wrapper::Output* p_outputs);

private:
    /// @brief Static constant timeout [ms] for waiting on external control loop inputs.
    static const unsigned int WRITE_TIMEOUT_MS = 24;

    /// @brief Mutex for protecting read data.
    std::mutex read_mutex_;

    /// @brief Mutex for protecting write data.
    std::mutex write_mutex_;

    /// @brief Condition variable for waiting on read data.
    std::condition_variable read_condition_variable_;

    /// @brief Condition variable for waiting on write data.
    std::condition_variable write_condition_variable_;

    /// @brief Flag indicating if read data is ready.
    bool read_data_ready_;

    /// @brief Flag indicating if write data is ready.
    bool write_data_ready_;

    /// @brief Container for the inputs received from the robot controller.
    wrapper::Input inputs_;

    /// @brief Container for the outputs to send to the robot controller.
    wrapper::Output outputs_;
};

///
/// @brief Class for an EGM contoller user interface.
/// The class provides behavior for motion control, and this includes:
/// - Processing asynchronous callbacks from an UDP server.
/// - Notifying an external control loop about new messages.
/// - Incorporating external control loop inputs, and sending them to the robot controller.
/// Pseudocode for the usage of the class methods (inside an external control loop);
/// 1. if(waitForMessage(...))
/// 1.1. read(...)
/// 1.2. write(...)
/// 1.3. Repeat from 1.
///
class EGMControllerInterface : public EGMBaseInterface {
public:
    /// @brief A constructor.
    /// @param io_service for operating asio's asynchronous functions.
    /// @param port_number for the server's UDP socket.
    /// @param configuration for the interface's configuration.
    EGMControllerInterface(
        asio::io_context& io_service, const unsigned short port_number,
        const BaseConfiguration& configuration = BaseConfiguration()
    );

    /// @brief Wait for the next EGM message.
    /// @param timeout_ms for specifying a timeout in [ms]. If omitted, then the method waits forever.
    /// @return bool indicating if the wait was successful or not. I.e. returns false if a timeout has occurred.
    bool waitForMessage(const unsigned int timeout_ms = 0);

    /// @brief Read EGM inputs received from the robot controller.
    /// @param p_inputs for containing the inputs.
    void read(wrapper::Input* p_inputs);

    /// @brief Write EGM outputs to send to the robot controller.
    /// @param outputs containing the outputs.
    void write(const wrapper::Output& outputs);

private:
    /// @brief Handle callback requests from an UDP server.
    /// @param server_data containing the UDP server's callback data.
    /// @return string& containing the reply.
    const std::string& callback(const UDPServerData& server_data);

    /// @brief The interface's controller motion data (between internal loop and external controller loop).
    ControllerMotion controller_motion_;
};

} // namespace ba7lya::abb::robot::egm
