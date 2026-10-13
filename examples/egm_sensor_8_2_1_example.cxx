// SPDX-License-Identifier: MIT
///
/// @file egm_sensor_8_2_1_example.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Sensor side of the EGM communication, ported from the ABB C++ sample for RobotWare 8.2.1.
/// @version 0.1
/// @date 2026-10-14
/// @copyright Copyright (c) 2026
///
/// The original is a Windows console program built on Winsock2 and the C++ protobuf runtime. Its
/// flow is kept as it is: receive up to a hundred EgmRobot messages from the robot controller,
/// print the sequence number, the time stamp and the message type of each one, and answer each
/// with an EgmSensor message that carries a fixed Cartesian pose. Three things differ from the
/// original on purpose:
///
/// - The datagram socket is a standalone asio socket instead of a platform socket, so the sample
///   builds on every supported platform rather than on Windows only.
/// - The socket is bound in a constructor rather than in the receive loop, so a port that is
///   already taken is reported before the first receive.
/// - The protobuf messages live on the stack and their fields are filled through the mutable
///   accessors, so the sample needs no explicit new/delete and no set_allocated_*.
///
/// The C# sample that ships with RobotWare 6.16.03 is
/// [egm_sensor_6_16_03_example.cxx](egm_sensor_6_16_03_example.cxx); it is the same program with
/// that release's own example values.
///
/// Run it with the port number as the only argument, e.g. `egm_example_sensor_8_2_1 6510`, and
/// start an EGMActPose RAPID instruction against the same port.
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

/// @brief Port this sample listens on for inbound messages, as in the original C++ sample.
constexpr unsigned short IP_PORT_NUMBER = 6510;

/// @brief Number of messages the original C++ sample exchanges before it exits.
constexpr int NUMBER_OF_MESSAGES = 100;

/// @brief Size [bytes] of the buffer one inbound datagram is received into.
constexpr std::size_t MESSAGE_BUFFER_SIZE = 1400;

/// @brief Time since the start of this program, in milliseconds, used as the message time stamp.
/// @return std::uint32_t containing the elapsed time.
/// @note The original C++ sample uses GetTickCount() here, which is Windows only; a steady clock
///       keeps the field monotonic on every platform, which is what the robot controller compares
///       the time stamp against.
std::uint32_t elapsed_time_ms() {
    using clock = std::chrono::steady_clock;
    static const clock::time_point start = clock::now();

    return static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count()
    );
}

///
/// @brief Sensor endpoint.
///
/// Receives EgmRobot messages from the robot controller and answers each one with an EgmSensor
/// message.
///
class sensor_endpoint {
public:
    /// @brief A constructor.
    /// @param io_service for operating asio's functions.
    /// @param port_number for the socket this endpoint listens on.
    /// @note The socket is bound here rather than in run(), so that is_initialized() can report a
    ///       port that is already taken before the receive loop is entered.
    sensor_endpoint(asio::io_context& io_service, const unsigned short port_number)
        : p_socket_(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), port_number))
        , port_number_(port_number)
        , initialized_(p_socket_.is_open()) {}

    /// @brief Check if the endpoint was bound successfully or not.
    /// @return bool true if and only if the endpoint was bound correctly.
    bool is_initialized() const { return initialized_; }

    /// @brief Run the receive and answer loop for NUMBER_OF_MESSAGES messages.
    /// @return int containing zero on a clean exit and a negative value otherwise.
    int run();

private:
    /// @brief Print the sequence number, the time stamp and the type of an inbound robot message.
    /// @param robot_message for the deserialized inbound message.
    static void display_robot_message(const EgmRobot& robot_message);

    /// @brief Fill an outbound sensor message with a header and a fixed Cartesian pose.
    /// @param p_sensor_message for the message to fill.
    void create_sensor_message(EgmSensor* p_sensor_message);

    /// @brief The socket this endpoint receives on and replies from.
    asio::ip::udp::socket p_socket_;

    /// @brief The port this endpoint listens on.
    unsigned short port_number_;

    /// @brief Flag indicating if the endpoint was bound successfully or not.
    bool initialized_;

    /// @brief Sequence number of the next message sent to the robot controller.
    std::uint32_t sequence_number_ = 0;
};

int sensor_endpoint::run() {
    char message_buffer[MESSAGE_BUFFER_SIZE] = {};
    std::string serialized_message;

    for (int messages = 0; messages < NUMBER_OF_MESSAGES; messages++) {
        // The address of the robot controller is learned from the datagram that arrives, and the
        // reply is sent back to it. I.e. the same endpoint the original C++ sample receives from.
        asio::ip::udp::endpoint robot_endpoint;
        asio::error_code error;

        const std::size_t bytes_received
            = p_socket_.receive_from(asio::buffer(message_buffer, MESSAGE_BUFFER_SIZE), robot_endpoint, 0, error);
        if (error) {
            std::cerr << "Error receive message: " << error.message() << std::endl;
            continue;
        }

        // De-serialize the inbound message from the robot controller and display it.
        EgmRobot robot_message;
        if (!robot_message.ParseFromArray(message_buffer, static_cast<int>(bytes_received))) {
            std::cerr << "Error parse message" << std::endl;
            continue;
        }
        display_robot_message(robot_message);

        // Create, serialize and send a new outbound sensor message.
        EgmSensor sensor_message;
        create_sensor_message(&sensor_message);
        if (!sensor_message.SerializeToString(&serialized_message)) {
            std::cerr << "Error serialize message" << std::endl;
            continue;
        }

        p_socket_.send_to(asio::buffer(serialized_message), robot_endpoint, 0, error);
        if (error) { std::cerr << "Error send message: " << error.message() << std::endl; }
    }

    return 0;
}

void sensor_endpoint::display_robot_message(const EgmRobot& robot_message) {
    const bool has_header = robot_message.has_header() && robot_message.header().has_seqno()
                         && robot_message.header().has_tm() && robot_message.header().has_mtype();
    if (!has_header) {
        std::cerr << "No header" << std::endl;
        return;
    }

    std::cout << "SeqNo=" << robot_message.header().seqno() << " Tm=" << robot_message.header().tm()
              << " Type=" << robot_message.header().mtype() << std::endl;
}

void sensor_endpoint::create_sensor_message(EgmSensor* p_sensor_message) {
    // The header marks the message as a position guidance correction from the sensor side.
    EgmHeader* p_header = p_sensor_message->mutable_header();
    p_header->set_mtype(EgmHeader::MSGTYPE_CORRECTION);
    p_header->set_seqno(sequence_number_++);
    p_header->set_tm(elapsed_time_ms());

    // A fixed Cartesian pose, which is enough to verify that the EGM communication channel works.
    EgmPose* p_pose = p_sensor_message->mutable_planned()->mutable_cartesian();

    p_pose->mutable_pos()->set_x(1.1);
    p_pose->mutable_pos()->set_y(2.2);
    p_pose->mutable_pos()->set_z(3.3);

    p_pose->mutable_orient()->set_u0(1.0);
    p_pose->mutable_orient()->set_u1(0.0);
    p_pose->mutable_orient()->set_u2(0.0);
    p_pose->mutable_orient()->set_u3(0.0);
}

} // namespace

int main(const int argc, char* argv[]) {
    const unsigned short port_number = (argc > 1) ? static_cast<unsigned short>(std::stoi(argv[1])) : IP_PORT_NUMBER;

    asio::io_context io_service;
    sensor_endpoint endpoint(io_service, port_number);

    if (!endpoint.is_initialized()) {
        std::cerr << "Failed to bind the UDP socket to port " << port_number << std::endl;
        return -1;
    }

    std::cout << "Listening on port " << port_number << ", press Ctrl+C to exit." << std::endl;
    std::cout << "Run an EGMActPose RAPID instruction against this port." << std::endl;

    return endpoint.run();
}
