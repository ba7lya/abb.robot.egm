// SPDX-License-Identifier: MIT
///
/// @file udp_server.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2026-10-12
/// @copyright Copyright (c) 2026
///

#include "ba7lya/abb/robot/egm/udp_server.hxx"

#include <functional>

namespace ba7lya::abb::robot::egm {

UDPServer::UDPServer(asio::io_context& io_service, unsigned short port_number, AbstractUDPServerInterface* p_interface)
    : initialized_(false)
    , p_interface_(p_interface) {
    bool success = true;

    try {
        server_data_.port_number = port_number;
        p_socket_.reset(
            new asio::ip::udp::socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), port_number))
        );
    }
    catch (std::exception e) {
        success = false;
    }

    if (success) {
        initialized_ = true;
        startAsynchronousReceive();
    }
}

UDPServer::~UDPServer() {
    if (p_socket_) {
        p_socket_->close();
        p_socket_.reset();
    }
}

bool UDPServer::isInitialized() const { return initialized_; }

void UDPServer::startAsynchronousReceive() {
    if (p_socket_) {
        p_socket_->async_receive_from(
            asio::buffer(receive_buffer_), remote_endpoint_,
            std::bind(&UDPServer::receiveCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
}

void UDPServer::receiveCallback(const std::error_code& error, const std::size_t bytes_transferred) {
    server_data_.p_data = receive_buffer_;
    server_data_.bytes_transferred = (int)bytes_transferred;

    if (error == std::error_code() && p_interface_) {
        // Process the received data via the callback method (creates the reply message).
        const std::string& reply = p_interface_->callback(server_data_);

        if (!reply.empty() && p_socket_) {
            // Send the response message to the robot controller.
            p_socket_->async_send_to(
                asio::buffer(reply), remote_endpoint_,
                std::bind(&UDPServer::sendCallback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }
    }

    // Add another asynchronous operation to the asio io_context object.
    startAsynchronousReceive();
}

void UDPServer::sendCallback(const std::error_code& error, const std::size_t bytes_transferred) {}

} // namespace ba7lya::abb::robot::egm
