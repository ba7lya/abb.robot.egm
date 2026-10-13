# SPDX-License-Identifier: MIT

# Buildable examples. Each one is a standalone program; the library's own C++ standard
# requirement is PRIVATE, so it is not inherited by consumers and every target sets its
# own instead. asio propagates through the library's PUBLIC link interface, so the
# examples that include controller_interface.hxx do not need to link it themselves.

# Offline example: interpolates a joint move and prints the resulting trajectory.
# It performs no I/O, so it is safe to run as a smoke test.
add_executable(${PROJECT_NAME}_example_interpolator interpolator_example.cxx)

target_link_libraries(${PROJECT_NAME}_example_interpolator PRIVATE ba7lya::abb::robot::egm)

target_compile_features(${PROJECT_NAME}_example_interpolator PRIVATE cxx_std_23)

# Online example: runs an external control loop against a real robot controller, so it
# needs the EGM communication to be set up before it produces any output.
add_executable(${PROJECT_NAME}_example_controller controller_interface_example.cxx)

target_link_libraries(${PROJECT_NAME}_example_controller PRIVATE ba7lya::abb::robot::egm)

target_compile_features(${PROJECT_NAME}_example_controller PRIVATE cxx_std_23)

# Online example: the sensor end of the EGM communication, ported from the sample that
# ships with RobotWare 6.16.03. It answers the robot controller with a fixed pose.
add_executable(${PROJECT_NAME}_example_sensor_6_16_03 egm_sensor_6_16_03_example.cxx)

target_link_libraries(${PROJECT_NAME}_example_sensor_6_16_03 PRIVATE ba7lya::abb::robot::egm)

target_compile_features(${PROJECT_NAME}_example_sensor_6_16_03 PRIVATE cxx_std_23)

# Online example: the sensor end of the EGM communication, ported from the sample that
# ships with RobotWare 8.2.1. It answers the robot controller with a fixed pose, and it
# stops after a hundred messages.
add_executable(${PROJECT_NAME}_example_sensor_8_2_1 egm_sensor_8_2_1_example.cxx)

target_link_libraries(${PROJECT_NAME}_example_sensor_8_2_1 PRIVATE ba7lya::abb::robot::egm)

target_compile_features(${PROJECT_NAME}_example_sensor_8_2_1 PRIVATE cxx_std_23)

# Online example: the sensor end of EGM path correction. It answers the robot controller
# with the current measurement as an EgmSensorPathCorr message.
add_executable(${PROJECT_NAME}_example_sensor_path_correction egm_sensor_path_correction_example.cxx)

target_link_libraries(${PROJECT_NAME}_example_sensor_path_correction PRIVATE ba7lya::abb::robot::egm)

target_compile_features(${PROJECT_NAME}_example_sensor_path_correction PRIVATE cxx_std_23)

if(MSVC)
    target_compile_options(${PROJECT_NAME}_example_interpolator PRIVATE /utf-8)
    target_compile_options(${PROJECT_NAME}_example_controller PRIVATE /utf-8)
    target_compile_options(${PROJECT_NAME}_example_sensor_6_16_03 PRIVATE /utf-8)
    target_compile_options(${PROJECT_NAME}_example_sensor_8_2_1 PRIVATE /utf-8)
    target_compile_options(${PROJECT_NAME}_example_sensor_path_correction PRIVATE /utf-8)
endif()
