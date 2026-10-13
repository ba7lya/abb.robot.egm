// SPDX-License-Identifier: MIT
///
/// @file parse_test.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief Unit tests for the parse() overloads declared in common_auxiliary.hxx.
/// @version 0.1
/// @date 2026-10-13
/// @copyright Copyright (c) 2026
///

#include <cmath>
#include <gtest/gtest.h>

#include "ba7lya/abb/robot/egm/common.hxx"
#include "ba7lya/abb/robot/egm/common_auxiliary.hxx"
#include "egm.pb.h"
#include "egm_wrapper.pb.h"

namespace {

using namespace ba7lya::abb::robot::egm;

constexpr double TOLERANCE = 1e-9;

EgmHeader makeHeader(
    const std::uint32_t sequence_number, const std::uint32_t time_stamp, const EgmHeader::MessageType type
) {
    EgmHeader header;
    header.set_seqno(sequence_number);
    header.set_tm(time_stamp);
    header.set_mtype(type);

    return header;
}

EgmJoints makeEgmJoints(const std::initializer_list<double> values) {
    EgmJoints joints;
    for (const double value : values) { joints.add_joints(value); }

    return joints;
}

EgmRobot makeRobotStatus(
    const EgmMotorState::MotorStateType motor_state, const EgmMCIState::MCIStateType mci_state,
    const EgmRapidCtrlExecState::RapidCtrlExecStateType rapid_state, const bool convergence_met
) {
    EgmRobot robot;
    robot.mutable_motorstate()->set_state(motor_state);
    robot.mutable_mcistate()->set_state(mci_state);
    robot.mutable_rapidexecstate()->set_state(rapid_state);
    robot.set_mciconvergencemet(convergence_met);

    return robot;
}

EgmClock makeClock(const std::uint64_t sec, const std::uint64_t usec) {
    EgmClock clock;
    clock.set_sec(sec);
    clock.set_usec(usec);

    return clock;
}

// --- parse(wrapper::Header*, const EgmHeader&) ---

TEST(parseHeader, dataMessageTypeIsAccepted) {
    const EgmHeader source = makeHeader(7, 1234, EgmHeader::MSGTYPE_DATA);

    wrapper::Header target;
    EXPECT_TRUE(parse(&target, source));

    EXPECT_EQ(7u, target.sequence_number());
    EXPECT_EQ(1234u, target.time_stamp());
    EXPECT_EQ(wrapper::Header::DATA, target.message_type());
}

TEST(parseHeader, nonDataMessageTypesAreRejectedButStillMapped) {
    const EgmHeader::MessageType rejected[] = {
        EgmHeader::MSGTYPE_UNDEFINED,
        EgmHeader::MSGTYPE_COMMAND,
        EgmHeader::MSGTYPE_CORRECTION,
        EgmHeader::MSGTYPE_PATH_CORRECTION,
    };

    for (const EgmHeader::MessageType type : rejected) {
        const EgmHeader source = makeHeader(1, 2, type);

        wrapper::Header target;
        EXPECT_FALSE(parse(&target, source));
        EXPECT_EQ(1u, target.sequence_number());
        EXPECT_EQ(2u, target.time_stamp());
        EXPECT_EQ(wrapper::Header::UNDEFINED, target.message_type());
    }
}

TEST(parseHeader, missingFieldsAreRejected) {
    wrapper::Header target;

    EgmHeader without_sequence_number;
    without_sequence_number.set_tm(1);
    without_sequence_number.set_mtype(EgmHeader::MSGTYPE_DATA);
    EXPECT_FALSE(parse(&target, without_sequence_number));

    EgmHeader without_time_stamp;
    without_time_stamp.set_seqno(1);
    without_time_stamp.set_mtype(EgmHeader::MSGTYPE_DATA);
    EXPECT_FALSE(parse(&target, without_time_stamp));

    // The message type has a default value, but the field itself is unset.
    EgmHeader without_message_type;
    without_message_type.set_seqno(1);
    without_message_type.set_tm(1);
    EXPECT_FALSE(parse(&target, without_message_type));
}

TEST(parseHeader, nullTargetIsRejected) {
    EXPECT_FALSE(parse(static_cast<wrapper::Header*>(nullptr), makeHeader(1, 1, EgmHeader::MSGTYPE_DATA)));
}

// --- parse(wrapper::Status*, const EgmRobot&) ---

TEST(parseStatus, mapsEveryStateCombination) {
    const EgmRobot source = makeRobotStatus(
        EgmMotorState::MOTORS_ON, EgmMCIState::MCI_RUNNING, EgmRapidCtrlExecState::RAPID_RUNNING, true
    );

    wrapper::Status target;
    EXPECT_TRUE(parse(&target, source));

    EXPECT_EQ(wrapper::Status::MOTORS_ON, target.motor_state());
    EXPECT_EQ(wrapper::Status::EGM_RUNNING, target.egm_state());
    EXPECT_EQ(wrapper::Status::RAPID_RUNNING, target.rapid_execution_state());
    EXPECT_TRUE(target.egm_convergence_met());
}

TEST(parseStatus, mapsTheRemainingStates) {
    wrapper::Status target;

    EXPECT_TRUE(parse(
        &target,
        makeRobotStatus(EgmMotorState::MOTORS_OFF, EgmMCIState::MCI_ERROR, EgmRapidCtrlExecState::RAPID_STOPPED, false)
    ));
    EXPECT_EQ(wrapper::Status::MOTORS_OFF, target.motor_state());
    EXPECT_EQ(wrapper::Status::EGM_ERROR, target.egm_state());
    EXPECT_EQ(wrapper::Status::RAPID_STOPPED, target.rapid_execution_state());
    EXPECT_FALSE(target.egm_convergence_met());

    EXPECT_TRUE(parse(
        &target,
        makeRobotStatus(
            EgmMotorState::MOTORS_UNDEFINED, EgmMCIState::MCI_STOPPED, EgmRapidCtrlExecState::RAPID_UNDEFINED, true
        )
    ));
    EXPECT_EQ(wrapper::Status::MOTORS_UNDEFINED, target.motor_state());
    EXPECT_EQ(wrapper::Status::EGM_STOPPED, target.egm_state());
    EXPECT_EQ(wrapper::Status::RAPID_UNDEFINED, target.rapid_execution_state());

    EXPECT_TRUE(parse(
        &target, makeRobotStatus(
                     EgmMotorState::MOTORS_ON, EgmMCIState::MCI_UNDEFINED, EgmRapidCtrlExecState::RAPID_RUNNING, true
                 )
    ));
    EXPECT_EQ(wrapper::Status::EGM_UNDEFINED, target.egm_state());
}

TEST(parseStatus, optionalUtilizationRateIsCopiedWhenPresent) {
    EgmRobot source = makeRobotStatus(
        EgmMotorState::MOTORS_ON, EgmMCIState::MCI_RUNNING, EgmRapidCtrlExecState::RAPID_RUNNING, true
    );
    source.set_utilizationrate(42.5);

    wrapper::Status target;
    EXPECT_TRUE(parse(&target, source));
    EXPECT_DOUBLE_EQ(42.5, target.utilization_rate());
}

TEST(parseStatus, everyStateFieldIsRequired) {
    wrapper::Status target;

    EgmRobot without_motor_state;
    without_motor_state.mutable_mcistate()->set_state(EgmMCIState::MCI_RUNNING);
    without_motor_state.mutable_rapidexecstate()->set_state(EgmRapidCtrlExecState::RAPID_RUNNING);
    without_motor_state.set_mciconvergencemet(true);
    EXPECT_FALSE(parse(&target, without_motor_state));

    EgmRobot without_mci_state;
    without_mci_state.mutable_motorstate()->set_state(EgmMotorState::MOTORS_ON);
    without_mci_state.mutable_rapidexecstate()->set_state(EgmRapidCtrlExecState::RAPID_RUNNING);
    without_mci_state.set_mciconvergencemet(true);
    EXPECT_FALSE(parse(&target, without_mci_state));

    EgmRobot without_rapid_state;
    without_rapid_state.mutable_motorstate()->set_state(EgmMotorState::MOTORS_ON);
    without_rapid_state.mutable_mcistate()->set_state(EgmMCIState::MCI_RUNNING);
    without_rapid_state.set_mciconvergencemet(true);
    EXPECT_FALSE(parse(&target, without_rapid_state));

    EgmRobot without_convergence;
    without_convergence.mutable_motorstate()->set_state(EgmMotorState::MOTORS_ON);
    without_convergence.mutable_mcistate()->set_state(EgmMCIState::MCI_RUNNING);
    without_convergence.mutable_rapidexecstate()->set_state(EgmRapidCtrlExecState::RAPID_RUNNING);
    EXPECT_FALSE(parse(&target, without_convergence));
}

// --- parse(wrapper::Clock*, const EgmClock&) ---

TEST(parseClock, copiesBothFields) {
    wrapper::Clock target;
    EXPECT_TRUE(parse(&target, makeClock(1234, 5678)));

    EXPECT_EQ(1234u, target.sec());
    EXPECT_EQ(5678u, target.usec());
}

TEST(parseClock, nullTargetIsRejected) { EXPECT_FALSE(parse(static_cast<wrapper::Clock*>(nullptr), makeClock(1, 2))); }

// --- parse(wrapper::Joints*, wrapper::Joints*, const EgmJoints&, const EgmJoints&, RobotAxes) ---

TEST(parseJoints, noneAxesCopiesOnlyTheExternalJoints) {
    const EgmJoints robot = makeEgmJoints({});
    const EgmJoints external = makeEgmJoints({ 1.0, 2.0 });

    wrapper::Joints target_robot;
    wrapper::Joints target_external;
    EXPECT_TRUE(parse(&target_robot, &target_external, robot, external, None));

    EXPECT_EQ(0, target_robot.values_size());
    ASSERT_EQ(2, target_external.values_size());
    EXPECT_DOUBLE_EQ(1.0, target_external.values(0));
    EXPECT_DOUBLE_EQ(2.0, target_external.values(1));
}

TEST(parseJoints, noneAxesRejectsRobotJoints) {
    const EgmJoints robot = makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    const EgmJoints external = makeEgmJoints({});

    wrapper::Joints target_robot;
    wrapper::Joints target_external;
    EXPECT_FALSE(parse(&target_robot, &target_external, robot, external, None));
}

TEST(parseJoints, sixAxesCopiesRobotAndExternalSeparately) {
    const EgmJoints robot = makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    const EgmJoints external = makeEgmJoints({ 7.0, 8.0 });

    wrapper::Joints target_robot;
    wrapper::Joints target_external;
    EXPECT_TRUE(parse(&target_robot, &target_external, robot, external, Six));

    ASSERT_EQ(6, target_robot.values_size());
    EXPECT_DOUBLE_EQ(1.0, target_robot.values(0));
    EXPECT_DOUBLE_EQ(6.0, target_robot.values(5));

    ASSERT_EQ(2, target_external.values_size());
    EXPECT_DOUBLE_EQ(7.0, target_external.values(0));
    EXPECT_DOUBLE_EQ(8.0, target_external.values(1));
}

TEST(parseJoints, sixAxesRequiresExactlySixRobotJoints) {
    wrapper::Joints target_robot;
    wrapper::Joints target_external;

    EXPECT_FALSE(parse(&target_robot, &target_external, makeEgmJoints({ 1.0, 2.0 }), makeEgmJoints({}), Six));
}

TEST(parseJoints, sevenAxesReordersTheFirstExternalJointIntoTheRobot) {
    const EgmJoints robot = makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    const EgmJoints external = makeEgmJoints({ 9.0, 10.0 });

    wrapper::Joints target_robot;
    wrapper::Joints target_external;
    EXPECT_TRUE(parse(&target_robot, &target_external, robot, external, Seven));

    ASSERT_EQ(7, target_robot.values_size());
    EXPECT_DOUBLE_EQ(1.0, target_robot.values(0));
    EXPECT_DOUBLE_EQ(2.0, target_robot.values(1));
    EXPECT_DOUBLE_EQ(9.0, target_robot.values(2));
    EXPECT_DOUBLE_EQ(3.0, target_robot.values(3));
    EXPECT_DOUBLE_EQ(4.0, target_robot.values(4));
    EXPECT_DOUBLE_EQ(5.0, target_robot.values(5));
    EXPECT_DOUBLE_EQ(6.0, target_robot.values(6));

    ASSERT_EQ(1, target_external.values_size());
    EXPECT_DOUBLE_EQ(10.0, target_external.values(0));
}

TEST(parseJoints, sevenAxesRequiresAtLeastOneExternalJoint) {
    wrapper::Joints target_robot;
    wrapper::Joints target_external;

    EXPECT_FALSE(parse(
        &target_robot, &target_external, makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }), makeEgmJoints({}), Seven
    ));
}

TEST(parseJoints, targetsAreClearedBeforeParsing) {
    const EgmJoints robot = makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    const EgmJoints external = makeEgmJoints({});

    wrapper::Joints target_robot;
    target_robot.add_values(99.0);
    wrapper::Joints target_external;
    target_external.add_values(99.0);

    EXPECT_TRUE(parse(&target_robot, &target_external, robot, external, Six));
    EXPECT_EQ(6, target_robot.values_size());
    EXPECT_EQ(0, target_external.values_size());
}

TEST(parseJoints, nullTargetsAreRejected) {
    const EgmJoints robot = makeEgmJoints({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    const EgmJoints external = makeEgmJoints({});

    wrapper::Joints target;
    EXPECT_FALSE(parse(static_cast<wrapper::Joints*>(nullptr), &target, robot, external, Six));
    EXPECT_FALSE(parse(&target, static_cast<wrapper::Joints*>(nullptr), robot, external, Six));
}

// --- parse(wrapper::CartesianPose*, const EgmPose&) ---

EgmPose makePose(const double x, const double y, const double z) {
    EgmPose pose;
    pose.mutable_pos()->set_x(x);
    pose.mutable_pos()->set_y(y);
    pose.mutable_pos()->set_z(z);
    pose.mutable_orient()->set_u0(1.0);
    pose.mutable_orient()->set_u1(0.0);
    pose.mutable_orient()->set_u2(0.0);
    pose.mutable_orient()->set_u3(0.0);

    return pose;
}

TEST(parsePose, copiesPositionAndOrientationAndDerivesTheEulerAngles) {
    const EgmPose source = makePose(1.0, 2.0, 3.0);

    wrapper::CartesianPose target;
    EXPECT_TRUE(parse(&target, source));

    EXPECT_DOUBLE_EQ(1.0, target.position().x());
    EXPECT_DOUBLE_EQ(2.0, target.position().y());
    EXPECT_DOUBLE_EQ(3.0, target.position().z());
    EXPECT_DOUBLE_EQ(1.0, target.quaternion().u0());
    EXPECT_NEAR(0.0, target.euler().x(), TOLERANCE);
    EXPECT_NEAR(0.0, target.euler().y(), TOLERANCE);
    EXPECT_NEAR(0.0, target.euler().z(), TOLERANCE);
}

TEST(parsePose, anExplicitEulerFieldTakesPrecedenceOverTheDerivedOne) {
    EgmPose source = makePose(1.0, 2.0, 3.0);
    source.mutable_euler()->set_x(10.0);
    source.mutable_euler()->set_y(20.0);
    source.mutable_euler()->set_z(30.0);

    wrapper::CartesianPose target;
    EXPECT_TRUE(parse(&target, source));

    EXPECT_DOUBLE_EQ(10.0, target.euler().x());
    EXPECT_DOUBLE_EQ(20.0, target.euler().y());
    EXPECT_DOUBLE_EQ(30.0, target.euler().z());
}

TEST(parsePose, aMissingPositionIsRejected) {
    EgmPose source;
    source.mutable_orient()->set_u0(1.0);
    source.mutable_orient()->set_u1(0.0);
    source.mutable_orient()->set_u2(0.0);
    source.mutable_orient()->set_u3(0.0);

    wrapper::CartesianPose target;
    EXPECT_FALSE(parse(&target, source));
}

TEST(parsePose, aMissingOrientationIsRejected) {
    EgmPose source;
    source.mutable_pos()->set_x(1.0);
    source.mutable_pos()->set_y(2.0);
    source.mutable_pos()->set_z(3.0);

    wrapper::CartesianPose target;
    EXPECT_FALSE(parse(&target, source));
}

// --- parse(wrapper::Feedback* / wrapper::Planned*, ...) ---

EgmFeedBack makeFeedback(
    const std::initializer_list<double> robot_joints, const std::initializer_list<double> external_joints
) {
    EgmFeedBack feedback;
    for (const double value : robot_joints) { feedback.mutable_joints()->add_joints(value); }
    for (const double value : external_joints) { feedback.mutable_externaljoints()->add_joints(value); }
    feedback.mutable_time()->set_sec(1);
    feedback.mutable_time()->set_usec(2);

    return feedback;
}

EgmPlanned makePlanned(
    const std::initializer_list<double> robot_joints, const std::initializer_list<double> external_joints
) {
    EgmPlanned planned;
    for (const double value : robot_joints) { planned.mutable_joints()->add_joints(value); }
    for (const double value : external_joints) { planned.mutable_externaljoints()->add_joints(value); }
    planned.mutable_time()->set_sec(1);
    planned.mutable_time()->set_usec(2);

    return planned;
}

TEST(parseFeedback, sixAxesWithoutCartesianDataSucceeds) {
    const EgmFeedBack source = makeFeedback({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, { 7.0 });

    wrapper::Feedback target;
    EXPECT_TRUE(parse(&target, source, Six));

    ASSERT_EQ(6, target.robot().joints().position().values_size());
    EXPECT_DOUBLE_EQ(1.0, target.robot().joints().position().values(0));
    ASSERT_EQ(1, target.external().joints().position().values_size());
    EXPECT_DOUBLE_EQ(7.0, target.external().joints().position().values(0));
    EXPECT_EQ(1u, target.time().sec());
    EXPECT_EQ(2u, target.time().usec());
}

TEST(parseFeedback, sixAxesWithCartesianDataParsesThePose) {
    EgmFeedBack source = makeFeedback({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, {});
    source.mutable_cartesian()->mutable_pos()->set_x(0.5);
    source.mutable_cartesian()->mutable_pos()->set_y(1.5);
    source.mutable_cartesian()->mutable_pos()->set_z(2.5);
    source.mutable_cartesian()->mutable_orient()->set_u0(1.0);
    source.mutable_cartesian()->mutable_orient()->set_u1(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u2(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u3(0.0);

    wrapper::Feedback target;
    EXPECT_TRUE(parse(&target, source, Six));

    EXPECT_TRUE(target.robot().has_cartesian());
    EXPECT_DOUBLE_EQ(0.5, target.robot().cartesian().pose().position().x());
}

TEST(parseFeedback, noneAxesRejectsAnyCartesianData) {
    const EgmFeedBack source = makeFeedback({}, { 1.0, 2.0 });

    wrapper::Feedback target;
    EXPECT_TRUE(parse(&target, source, None));

    EgmFeedBack with_cartesian = makeFeedback({}, { 1.0 });
    with_cartesian.mutable_cartesian()->mutable_pos()->set_x(0.0);
    with_cartesian.mutable_cartesian()->mutable_pos()->set_y(0.0);
    with_cartesian.mutable_cartesian()->mutable_pos()->set_z(0.0);
    with_cartesian.mutable_cartesian()->mutable_orient()->set_u0(1.0);
    with_cartesian.mutable_cartesian()->mutable_orient()->set_u1(0.0);
    with_cartesian.mutable_cartesian()->mutable_orient()->set_u2(0.0);
    with_cartesian.mutable_cartesian()->mutable_orient()->set_u3(0.0);

    wrapper::Feedback rejected;
    EXPECT_FALSE(parse(&rejected, with_cartesian, None));
}

TEST(parseFeedback, aMalformedJointCountFailsBeforeAnyCartesianParsing) {
    const EgmFeedBack source = makeFeedback({ 1.0, 2.0 }, {});

    wrapper::Feedback target;
    EXPECT_FALSE(parse(&target, source, Six));
}

TEST(parseFeedback, nullTargetIsRejected) {
    EXPECT_FALSE(parse(static_cast<wrapper::Feedback*>(nullptr), makeFeedback({}, {}), Six));
}

TEST(parsePlanned, sixAxesWithoutCartesianDataSucceeds) {
    const EgmPlanned source = makePlanned({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, {});

    wrapper::Planned target;
    EXPECT_TRUE(parse(&target, source, Six));

    ASSERT_EQ(6, target.robot().joints().position().values_size());
    EXPECT_DOUBLE_EQ(6.0, target.robot().joints().position().values(5));
    EXPECT_EQ(1u, target.time().sec());
}

TEST(parsePlanned, noneAxesRejectsAnyCartesianData) {
    EgmPlanned source = makePlanned({}, { 1.0 });
    source.mutable_cartesian()->mutable_pos()->set_x(0.0);
    source.mutable_cartesian()->mutable_pos()->set_y(0.0);
    source.mutable_cartesian()->mutable_pos()->set_z(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u0(1.0);
    source.mutable_cartesian()->mutable_orient()->set_u1(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u2(0.0);
    source.mutable_cartesian()->mutable_orient()->set_u3(0.0);

    wrapper::Planned target;
    EXPECT_FALSE(parse(&target, source, None));
}

} // namespace
