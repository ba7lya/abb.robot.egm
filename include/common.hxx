// SPDX-License-Identifier: MIT
///
/// @file common.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2026-10-12
/// @copyright Copyright (c) 2026
///

#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <numbers>

namespace ba7lya::abb::robot::egm {

///
/// Constants related to the robot controller system.
///
constexpr double LOWEST_SAMPLE_TIME = 0.004;        // Lowest sample time [s] used in EGM communication.
constexpr std::uint16_t DEFAULT_PORT_NUMBER = 6511; // Default port number assumed for EGM communication.
constexpr int DEFAULT_NUM_OF_ROBOT_JOINTS = 6;      // Default number of robot joints.
constexpr int DEFAULT_NUM_OF_EXT_JOINTS = 6;        // Default number of external joints.
constexpr int MAX_NUM_OF_JOINTS = DEFAULT_NUM_OF_ROBOT_JOINTS + DEFAULT_NUM_OF_EXT_JOINTS; // Maximum number of joints.

///
/// Constants related to the conversions between units.
///
constexpr double PI = std::numbers::pi_v<double>; // Value of pi.
constexpr double RAD_TO_DEG = 180.0 / PI;         // Conversion value from radians to degrees.
constexpr double DEG_TO_RAD = PI / 180.0;         // Conversion value from degrees to radians.
constexpr double MM_TO_M = 0.001;                 // Conversion value from millimeter to meter.
constexpr double MS_TO_S = 0.001;                 // Conversion value from milliseconds to seconds.
constexpr double S_TO_US = 1e6;                   // Conversion value from seconds to microseconds.

/// @brief Enum for the number of axes of the robot.
enum RobotAxes {
    None = 0, ///< @brief No robot axes are expected (i.e. only external axes).
    Six = 6,  ///< @brief A six axes robot.
    Seven = 7 ///< @brief A seven axes robot.
};

/// @brief Enum for the supported EGM modes (i.e. the corresponding RAPID instructions).
enum EGMModes {
    EGMJoint, ///< @brief The EGM joint mode.
    EGMPose   ///< @brief The EGM pose mode.
};

///
/// @brief Struct for an EGM user interface's base configuration.
///
struct BaseConfiguration {
    BaseConfiguration()
        : axes(Six)
        , use_demo_outputs(false)
        , use_velocity_outputs(false)
        , use_logging(false)
        , max_logging_duration(60.0) {}

    /// @brief Value specifying if a six or seven axes robot is used.
    /// @note If set to a seven axes robot, then an implicit mapping of joint values is performed.
    RobotAxes axes;

    /// @brief Flag indicating if demo outputs should be used.
    /// @note Overrides any other execution mode. Mainly used to verify that the EGM communication channel
    ///       works as intended.
    bool use_demo_outputs;

    /// @brief Flag indicating if the messages, sent to the robot controller, should include velocity outputs.
    /// @note If set to false, then no velocity values are sent (they are optional).
    bool use_velocity_outputs;

    /// @brief Flag indicating if the interface should log data.
    bool use_logging;

    /// @brief Maximum duration [s] to log data.
    double max_logging_duration;

    /// @brief Optional condition variable intended for notifying an external control loop that a new message is
    /// available.
    /// @note This is only intended to be used in the EGMControllerInterface class.
    std::shared_ptr<std::condition_variable> p_new_message_cv;
};

///
/// @brief Enum for the available spline polynomial interpolation methods.
/// @note Cartesian orientation uses Slerp interpolation instead.
/// Boundary conditions (between trajectory points):
/// |
/// |--Linear:
/// |  |-- Start and goal positions.
/// |
/// |--Square:
/// |  |-- Start and goal positions.
/// |  |-- Start velocity.
/// |
/// |--Cubic:
/// |  |-- Start and goal positions.
/// |  |-- Start and goal velocities.
/// |
/// |--Quintic:
///    |-- Start and goal positions.
///    |-- Start and goal velocities.
///    |-- Start and goal accelerations.
///
enum SplineMethod {
    Linear, ///< @brief Use a first degree polynomial.
    Square, ///< @brief Use a second degree polynomial.
    Cubic,  ///< @brief Use a third degree polynomial
    Quintic ///< @brief Use a fifth degree polynomial.
};

///
/// @brief Struct for the EGM trajectory user interface's configuration.
///
struct TrajectoryConfiguration {
    /// @brief A constructor.
    /// @param base_configuration specifying the base configurations.
    TrajectoryConfiguration(const BaseConfiguration& base_configuration = BaseConfiguration())
        : base(base_configuration)
        , spline_method(Quintic) {}

    /// @brief The base configurations.
    BaseConfiguration base;

    /// @brief Value specifying which spline method to use in the interpolation.
    SplineMethod spline_method;
};

} // namespace ba7lya::abb::robot::egm
