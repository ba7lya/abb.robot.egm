// SPDX-License-Identifier: MIT
///
/// @file controller_interface_test.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Integration tests for EGMControllerInterface over a loopback UDP socket.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///
/// The tests drive the interface the way a robot controller does: a UDP client sends serialized
/// EgmRobot messages to the interface's server socket, and the interface answers with a
/// serialized EgmSensor message. Nothing here reaches the network beyond the loopback address.
///
/// The interface's callback blocks on the io_context thread until an external control loop has
/// returned its outputs, so the control loop runs on its own thread. The messages therefore
/// carry a complete feedback, planned and status body: extraction fails unless all three parse.
///

#include <array>
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "common.hxx"
#include "controller_interface.hxx"
#include "egm.pb.h"
#include "egm_wrapper.pb.h"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr int NUMBER_OF_JOINTS = DEFAULT_NUM_OF_ROBOT_JOINTS;

/// @brief Static constant number of messages exchanged in one session.
constexpr int NUMBER_OF_MESSAGES = 4;

/// @brief Static constant joint displacement between two consecutive messages [degrees].
constexpr double JOINT_STEP = 5.0;

/// @brief Static constant message period [ms], matching the lowest sample time.
constexpr int MESSAGE_PERIOD_MS = 4;

/// @brief Static constant time a control loop waits for a message before re-checking its flag [ms].
constexpr int MESSAGE_POLL_TIMEOUT_MS = 20;

/// @brief Static constant time a test waits for a reply before declaring the session broken [ms].
constexpr int REPLY_TIMEOUT_MS = 5000;

/// @brief Static constant size of the buffer one reply datagram is read into [bytes].
constexpr std::size_t REPLY_BUFFER_SIZE = 1024;

/// @brief Static constant port the loopback session runs on.
constexpr unsigned short LOOPBACK_PORT = DEFAULT_PORT_NUMBER;

/// @brief Static constant port the demo session runs on.
constexpr unsigned short DEMO_PORT = DEFAULT_PORT_NUMBER + 1;

/// @brief Static constant peak displacement of the demo motion [degrees].
constexpr double DEMO_AMPLITUDE = 2.0;

/// @brief Fill a joint group with six robot joints and six external joints.
/// @param p_robot for containing the robot joint values.
/// @param p_external for containing the external joint values.
/// @param offset for the value the first joint is set to.
void fill_joints(EgmJoints* p_robot, EgmJoints* p_external, const double offset) {
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        p_robot->add_joints(offset + i);
        p_external->add_joints(2.0 * offset + i);
    }
}

/// @brief Fill a Cartesian pose with a position and a unit quaternion.
/// @param p_pose for containing the pose.
/// @param offset for the value added to each position component.
void fill_pose(EgmPose* p_pose, const double offset) {
    p_pose->mutable_pos()->set_x(100.0 + offset);
    p_pose->mutable_pos()->set_y(200.0 + offset);
    p_pose->mutable_pos()->set_z(300.0 + offset);
    p_pose->mutable_orient()->set_u0(1.0);
    p_pose->mutable_orient()->set_u1(0.0);
    p_pose->mutable_orient()->set_u2(0.0);
    p_pose->mutable_orient()->set_u3(0.0);
}

/// @brief Build an EGM message that carries every field the interface extracts.
/// @param sequence_number for the message's sequence number. Zero marks the first message of a session.
/// @param joint_offset for the value the first robot joint is set to.
/// @return EgmRobot containing the message.
EgmRobot make_egm_robot(const std::uint32_t sequence_number, const double joint_offset) {
    EgmRobot robot;

    robot.mutable_header()->set_seqno(sequence_number);
    robot.mutable_header()->set_tm(sequence_number * MESSAGE_PERIOD_MS);
    robot.mutable_header()->set_mtype(EgmHeader::MSGTYPE_DATA);

    // The feedback and planned bodies share a layout; only the joint values distinguish them.
    fill_joints(
        robot.mutable_feedback()->mutable_joints(), robot.mutable_feedback()->mutable_externaljoints(), joint_offset
    );
    fill_pose(robot.mutable_feedback()->mutable_cartesian(), joint_offset);
    robot.mutable_feedback()->mutable_time()->set_sec(sequence_number);
    robot.mutable_feedback()->mutable_time()->set_usec(sequence_number * MESSAGE_PERIOD_MS * 1000);

    fill_joints(
        robot.mutable_planned()->mutable_joints(), robot.mutable_planned()->mutable_externaljoints(), joint_offset
    );
    fill_pose(robot.mutable_planned()->mutable_cartesian(), joint_offset);
    robot.mutable_planned()->mutable_time()->set_sec(sequence_number);
    robot.mutable_planned()->mutable_time()->set_usec(sequence_number * MESSAGE_PERIOD_MS * 1000);

    // Motors on, EGM running and RAPID running: the state the interface accepts as ok.
    robot.mutable_motorstate()->set_state(EgmMotorState::MOTORS_ON);
    robot.mutable_mcistate()->set_state(EgmMCIState::MCI_RUNNING);
    robot.mutable_rapidexecstate()->set_state(EgmRapidCtrlExecState::RAPID_RUNNING);
    robot.set_mciconvergencemet(true);

    return robot;
}

/// @brief Wait for one reply datagram, polling until the deadline expires.
/// @param socket for the client socket, which must be in non-blocking mode.
/// @param p_reply for containing the received datagram.
/// @return bool indicating if a datagram arrived before the deadline.
bool receive_reply(asio::ip::udp::socket& socket, std::string* p_reply) {
    std::array<char, REPLY_BUFFER_SIZE> buffer = {};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(REPLY_TIMEOUT_MS);

    while (std::chrono::steady_clock::now() < deadline) {
        try {
            const std::size_t received = socket.receive(asio::buffer(buffer));
            p_reply->assign(buffer.data(), received);

            return true;
        }
        catch (const std::system_error& error) {
            if (error.code() != asio::error::would_block && error.code() != asio::error::try_again) { throw; }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
}

/// @brief The reply the interface returned for one message.
struct reply_record {
    /// @brief The sequence number the interface stamped onto its reply.
    std::uint32_t sequence_number_ {};

    /// @brief The robot joint positions the reply carries.
    std::vector<double> robot_joint_positions_;
};

/// @brief Send a session's worth of messages and collect the replies.
/// @param p_records for containing one record per answered message.
/// @param server_port for the port the interface under test listens on.
/// @return bool indicating if every message was answered with a well formed reply.
bool exchange_messages(std::vector<reply_record>* p_records, const unsigned short server_port) {
    asio::io_context client_service;
    asio::ip::udp::socket client(client_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0));
    client.non_blocking(true);

    const asio::ip::udp::endpoint server_endpoint(asio::ip::make_address("127.0.0.1"), server_port);

    for (int i = 0; i < NUMBER_OF_MESSAGES; ++i) {
        const double joint_offset = i * JOINT_STEP;
        const EgmRobot datagram = make_egm_robot(static_cast<std::uint32_t>(i), joint_offset);

        std::string serialized;
        if (!datagram.SerializeToString(&serialized)) { return false; }
        client.send_to(asio::buffer(serialized), server_endpoint);

        std::string reply;
        if (!receive_reply(client, &reply)) { return false; }

        EgmSensor sensor;
        if (!sensor.ParseFromArray(reply.data(), static_cast<int>(reply.size()))) { return false; }

        reply_record record;
        record.sequence_number_ = sensor.header().seqno();
        for (int joint = 0; joint < sensor.planned().joints().joints_size(); ++joint) {
            record.robot_joint_positions_.push_back(sensor.planned().joints().joints(joint));
        }

        p_records->push_back(record);
    }

    return true;
}

// --- EGMControllerInterface ---

TEST(EGMControllerInterfaceIntegration, theServerBindsTheConfiguredPort) {
    asio::io_context io_service;
    EGMControllerInterface interface(io_service, LOOPBACK_PORT);

    EXPECT_TRUE(interface.isInitialized());
}

TEST(EGMControllerInterfaceIntegration, externalControlLoopReferencesAreSentToTheController) {
    asio::io_context io_service;
    EGMControllerInterface interface(io_service, LOOPBACK_PORT);
    ASSERT_TRUE(interface.isInitialized());

    std::atomic<bool> session_active(true);
    std::atomic<int> messages_mirrored(0);

    // The external control loop mirrors the feedback positions back to the controller. It runs on
    // its own thread, because the interface's callback blocks until these references arrive.
    const auto control_loop = [&interface, &session_active, &messages_mirrored]() {
        while (session_active.load()) {
            if (!interface.waitForMessage(MESSAGE_POLL_TIMEOUT_MS)) { continue; }

            wrapper::Input inputs;
            interface.read(&inputs);

            wrapper::Output outputs;
            outputs.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(
                inputs.feedback().robot().joints().position()
            );
            outputs.mutable_external()->mutable_joints()->mutable_position()->CopyFrom(
                inputs.feedback().external().joints().position()
            );

            interface.write(outputs);
            ++messages_mirrored;
        }
    };

    std::thread io_thread([&io_service]() { io_service.run(); });
    std::thread control_thread(control_loop);

    std::vector<reply_record> records;
    const bool complete = exchange_messages(&records, LOOPBACK_PORT);

    session_active.store(false);
    io_service.stop();
    control_thread.join();
    io_thread.join();

    ASSERT_TRUE(complete) << "the interface did not answer every message";
    ASSERT_EQ(NUMBER_OF_MESSAGES, static_cast<int>(records.size()));
    EXPECT_EQ(NUMBER_OF_MESSAGES, messages_mirrored.load());

    for (int i = 0; i < NUMBER_OF_MESSAGES; ++i) {
        const double joint_offset = i * JOINT_STEP;

        EXPECT_EQ(static_cast<std::uint32_t>(i), records.at(i).sequence_number_);
        ASSERT_EQ(NUMBER_OF_JOINTS, static_cast<int>(records.at(i).robot_joint_positions_.size()));

        // The reply carries exactly the references the control loop mirrored.
        for (int joint = 0; joint < NUMBER_OF_JOINTS; ++joint) {
            EXPECT_DOUBLE_EQ(joint_offset + joint, records.at(i).robot_joint_positions_.at(joint));
        }
    }
}

TEST(EGMControllerInterfaceIntegration, demoOutputsAreReturnedWithoutAnExternalControlLoop) {
    BaseConfiguration configuration;
    configuration.use_demo_outputs = true;

    asio::io_context io_service;
    EGMControllerInterface interface(io_service, DEMO_PORT, configuration);
    ASSERT_TRUE(interface.isInitialized());

    // Demo outputs need no external control loop, so the callback never blocks on one.
    std::thread io_thread([&io_service]() { io_service.run(); });

    std::vector<reply_record> records;
    const bool complete = exchange_messages(&records, DEMO_PORT);

    io_service.stop();
    io_thread.join();

    ASSERT_TRUE(complete) << "the interface did not answer every message";
    ASSERT_EQ(NUMBER_OF_MESSAGES, static_cast<int>(records.size()));
    ASSERT_EQ(NUMBER_OF_JOINTS, static_cast<int>(records.front().robot_joint_positions_.size()));

    // The demo ramps in over seconds, so a session this short never leaves the ramp: sequence number
    // zero reproduces the session's initial feedback unchanged, and every later sample shifts all
    // robot joints by one and the same offset, which the ramp bounds to its two degree amplitude.
    const std::vector<double>& initial_joints = records.front().robot_joint_positions_;

    for (int i = 0; i < NUMBER_OF_MESSAGES; ++i) {
        EXPECT_EQ(static_cast<std::uint32_t>(i), records.at(i).sequence_number_);
        ASSERT_EQ(NUMBER_OF_JOINTS, static_cast<int>(records.at(i).robot_joint_positions_.size()));

        if (i == 0) {
            for (int joint = 0; joint < NUMBER_OF_JOINTS; ++joint) {
                EXPECT_DOUBLE_EQ(joint, initial_joints.at(joint)) << "the demo did not start from the feedback";
            }
            continue;
        }

        const double offset = records.at(i).robot_joint_positions_.front() - initial_joints.front();
        EXPECT_LE(std::abs(offset), DEMO_AMPLITUDE);

        for (int joint = 0; joint < NUMBER_OF_JOINTS; ++joint) {
            EXPECT_DOUBLE_EQ(joint + offset, records.at(i).robot_joint_positions_.at(joint)) << "joint " << joint;
        }
    }
}

} // namespace
