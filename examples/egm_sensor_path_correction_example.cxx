// SPDX-License-Identifier: MIT
///
/// @file egm_sensor_path_correction_example.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Sensor side of the EGM path correction, sending EgmSensorPathCorr messages.
/// @version 0.1
/// @date 2026-10-14
/// @copyright Copyright (c) 2026
///
/// EGM path correction corrects a programmed robot path with the measurements of a sensor that is
/// mounted on the tool flange. The corrections are applied in the path coordinate system, whose
/// x-axis is the tangent of the path, whose y-axis is the cross product of the path tangent and
/// the z-direction of the active tool frame, and whose z-axis is the cross product of the x and y
/// axes. The path correction has to start and end in a fine point, and the sensor measurements can
/// be provided at multiples of about 48 ms.
///
/// This sample is the sensor end of that flow. It receives the EgmRobot messages with which the
/// robot controller reports the path it is about to take, and answers each one with an
/// EgmSensorPathCorr message that carries a measurement and the age of that measurement. Two
/// things differ from the position guidance samples in this directory on purpose:
///
/// - The outbound message is an EgmSensorPathCorr rather than an EgmSensor, and its header is
///   marked MSGTYPE_PATH_CORRECTION rather than MSGTYPE_CORRECTION.
/// - The measurement is a plain Cartesian position in the sensor tool coordinate system with an
///   age in milliseconds, instead of a full pose.
///
/// Run it with the port number as the only argument, e.g.
/// `egm_example_sensor_path_correction 6510`, and start an EGMActPose RAPID instruction with a
/// Path level against the same port.
///

#include <asio.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "common.hxx"
#include "egm.pb.h" // Generated from proto/egm.proto by the Google protoc compiler.

namespace {

using namespace ba7lya::abb::robot::egm;

/// @brief Port this sample listens on for inbound messages.
constexpr unsigned short IP_PORT_NUMBER = 6514;

/// @brief Size [bytes] of the buffer one inbound datagram is received into.
constexpr std::size_t MESSAGE_BUFFER_SIZE = 1400;

/// @brief Correction [mm] applied along the path tangent, i.e. the x-axis of the path frame.
constexpr double CORRECTION_X = 0.0;

/// @brief Correction [mm] applied across the path, i.e. the y-axis of the path frame.
constexpr double CORRECTION_Y = 5.0;

/// @brief Correction [mm] applied normal to the path, i.e. the z-axis of the path frame.
constexpr double CORRECTION_Z = 0.0;

/// @brief Age [ms] of the measurement this sample sends.
/// @note The controller uses the age to compensate the movement that the robot performed since the
///       measurement was taken; a real sensor would report the time it measured at, here the value
///       is constant because the measurement is constant too.
constexpr std::uint32_t MEASUREMENT_AGE_MS = 20;

/// @brief Time since the start of this program, in milliseconds, used as the message time stamp.
/// @return std::uint32_t containing the elapsed time.
std::uint32_t elapsed_time_ms() {
    using clock = std::chrono::steady_clock;
    static const clock::time_point start = clock::now();

    return static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count()
    );
}

///
/// @brief Path correction sensor endpoint.
///
/// Receives the EgmRobot messages of the robot controller and answers each one with the current
/// measurement as an EgmSensorPathCorr message.
///
class path_correction_endpoint {
public:
    /// @brief A constructor.
    /// @param io_service for operating asio's functions.
    /// @param port_number for the socket this endpoint listens on.
    /// @note The socket is bound here rather than in run(), so that is_initialized() can report a
    ///       port that is already taken before the receive loop is entered.
    path_correction_endpoint(asio::io_context& io_service, const unsigned short port_number)
        : p_socket_(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), port_number))
        , port_number_(port_number)
        , initialized_(p_socket_.is_open()) {}

    /// @brief Check if the endpoint was bound successfully or not.
    /// @return bool true if and only if the endpoint was bound correctly.
    bool is_initialized() const { return initialized_; }

    /// @brief Run the receive and answer loop until the socket reports an error.
    /// @return int containing zero on a clean exit and a negative value otherwise.
    int run();

private:
    /// @brief Print the sequence number of an inbound robot controller message.
    /// @param robot_message for the deserialized inbound message.
    static void display_inbound_message(const EgmRobot& robot_message);

    /// @brief Fill an outbound path correction message with a header and the current measurement.
    /// @param p_sensor_message for the message to fill.
    void create_path_correction_message(EgmSensorPathCorr* p_sensor_message);

    /// @brief The socket this endpoint receives on and replies from.
    asio::ip::udp::socket p_socket_;

    /// @brief The port this endpoint listens on.
    unsigned short port_number_;

    /// @brief Flag indicating if the endpoint was bound successfully or not.
    bool initialized_;

    /// @brief Sequence number of the next message sent to the robot controller.
    std::uint32_t sequence_number_ = 0;
};

int path_correction_endpoint::run() {
    char message_buffer[MESSAGE_BUFFER_SIZE] = {};
    std::string serialized_message;

    while (true) {
        // The address of the robot controller is learned from the datagram that arrives, and the
        // reply is sent back to it.
        asio::ip::udp::endpoint robot_endpoint;
        asio::error_code error;

        const std::size_t bytes_received
            = p_socket_.receive_from(asio::buffer(message_buffer, MESSAGE_BUFFER_SIZE), robot_endpoint, 0, error);
        if (error) {
            std::cerr << "Error receive message: " << error.message() << std::endl;
            return -1;
        }

        // De-serialize the inbound message from the robot controller and display it.
        EgmRobot robot_message;
        if (!robot_message.ParseFromArray(message_buffer, static_cast<int>(bytes_received))) {
            std::cerr << "Error parse message" << std::endl;
            continue;
        }
        display_inbound_message(robot_message);

        // Create, serialize and send the current measurement back to the robot controller.
        EgmSensorPathCorr correction_message;
        create_path_correction_message(&correction_message);
        if (!correction_message.SerializeToString(&serialized_message)) {
            std::cerr << "Error serialize message" << std::endl;
            continue;
        }

        p_socket_.send_to(asio::buffer(serialized_message), robot_endpoint, 0, error);
        if (error) { std::cerr << "Error send message: " << error.message() << std::endl; }
    }
}

void path_correction_endpoint::display_inbound_message(const EgmRobot& robot_message) {
    const bool has_header
        = robot_message.has_header() && robot_message.header().has_seqno() && robot_message.header().has_tm();
    if (!has_header) {
        std::cerr << "No header in robot message" << std::endl;
        return;
    }

    // The path the correction is applied to is the planned position the controller reports; only
    // its presence is checked here, a real sensor would use it to place the measurement.
    if (!robot_message.has_planned()) {
        std::cerr << "No planned position in robot message" << std::endl;
        return;
    }

    std::cout << "Seq=" << robot_message.header().seqno() << " tm=" << robot_message.header().tm() << std::endl;
}

void path_correction_endpoint::create_path_correction_message(EgmSensorPathCorr* p_sensor_message) {
    // The header marks the message as a path correction from the sensor side.
    EgmHeader* p_header = p_sensor_message->mutable_header();
    p_header->set_seqno(sequence_number_++);
    p_header->set_tm(elapsed_time_ms());
    p_header->set_mtype(EgmHeader::MSGTYPE_PATH_CORRECTION);

    // The measurement, in the sensor tool coordinate system, and the age of that measurement.
    EgmPathCorr* p_path_corr = p_sensor_message->mutable_pathcorr();
    p_path_corr->mutable_pos()->set_x(CORRECTION_X);
    p_path_corr->mutable_pos()->set_y(CORRECTION_Y);
    p_path_corr->mutable_pos()->set_z(CORRECTION_Z);
    p_path_corr->set_age(MEASUREMENT_AGE_MS);
}

} // namespace

int main(const int argc, char* argv[]) {
    const unsigned short port_number = (argc > 1) ? static_cast<unsigned short>(std::stoi(argv[1])) : IP_PORT_NUMBER;

    asio::io_context io_service;
    path_correction_endpoint endpoint(io_service, port_number);

    if (!endpoint.is_initialized()) {
        std::cerr << "Failed to bind the UDP socket to port " << port_number << std::endl;
        return -1;
    }

    std::cout << "Listening on port " << port_number << ", press Ctrl+C to exit." << std::endl;
    std::cout << "Run an EGMActPose RAPID instruction with a Path level against this port." << std::endl;

    return endpoint.run();
}
