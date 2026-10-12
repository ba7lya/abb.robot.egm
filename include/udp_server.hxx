// SPDX-License-Identifier: MIT
///
/// @file udp_server.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2026-10-12
/// @copyright Copyright (c) 2026
///

#pragma once

#include <asio.hpp>
#include <memory>

namespace ba7lya::abb::robot::egm {

///
/// @brief Struct for containing data from the UDPServer class.
///
struct UDPServerData {
    UDPServerData()
        : port_number(0)
        , bytes_transferred(0) {}

    /// @brief Port number for the server's UDP socket.
    size_t port_number;

    /// @brief The received data.
    char* p_data;

    /// @brief Bytes transferred to the server.
    int bytes_transferred;
};

///
/// @brief Abstract class for a user interface to the UDPServer class.
///
class AbstractUDPServerInterface {
    /// @brief A friend to the interface.
    friend class UDPServer;

private:
    /// @brief Pure virtual method for handling callback requests from a UDPServer instance.
    /// @param server_data containing the UDP server's callback data.
    /// @return string& containing the reply.
    virtual const std::string& callback(const UDPServerData& data) = 0;
};

///
/// @brief Class for an asynchronous UDP server.
/// The server receives UDP messages from a client, passes the messages to a callback and returns a reply to the client.
///
class UDPServer {
public:
    /// @brief A constructor.
    /// @param io_service for operating asio's asynchronous functions.
    /// @param port_number for the server's UDP socket.
    /// @param p_interface that processes the received messages.
    UDPServer(asio::io_context& io_service, unsigned short port_number, AbstractUDPServerInterface* p_interface);

    /// @brief A destructor.
    ~UDPServer();

    /// @brief Checks if the server was successfully initialized or not.
    /// @return bool true if and only if the server was initialized correctly.
    bool isInitialized() const;

private:
    /// @brief Start an asynchronous receive.
    void startAsynchronousReceive();

    /// @brief Callback for handling an asynchronous receive.
    /// @param error for containing an error code.
    /// @param bytes_transferred is the number of bytes received.
    void receiveCallback(const std::error_code& error, const std::size_t bytes_transferred);

    /// @brief Callback for handling an asynchronous send.
    /// @param error for containing an error code.
    /// @param bytes_transferred is the number of bytes transmitted.
    void sendCallback(const std::error_code& error, const std::size_t bytes_transferred);

    /// @brief Static constant for the socket's buffer size.
    static const size_t BUFFER_SIZE = 1024;

    /// @brief The server's UDP socket.
    std::shared_ptr<asio::ip::udp::socket> p_socket_;

    /// @brief The address of the calling computer.
    /// @example an ABB robot controller or a virtual controller in RobotStudio).
    asio::ip::udp::endpoint remote_endpoint_;

    /// @brief A buffer for storing the server's serialized inbound messages (i.e. the robot's outbound messages).
    char receive_buffer_[BUFFER_SIZE];

    /// @brief Pointer to an object that is derived from AbstractUDPSeverInterface, which processes the received
    /// messages.
    AbstractUDPServerInterface* p_interface_;

    /// @brief Container for server data.
    UDPServerData server_data_;

    /// @brief Flag indicating if the server was initialized successfully or not.
    bool initialized_;
};

} // namespace ba7lya::abb::robot::egm
