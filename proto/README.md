# Information

This directory contains the Protocol Buffers schemas used by `ba7lya.abb.robot.egm` to define the EGM communication protocol and the wrapper messages layered on top of it.

* [egm.proto](egm.proto): Created by [ABB Robotics](https://new.abb.com/products/robotics) for specifying the EGM communication protocol. Package `ba7lya.abb.robot.egm`.
  * The file can be found in the installation folder of RobotWare. For example on Windows with RobotWare `6.08.00.01`: `%localappdata%\ABB Industrial IT\Robotics IT\RobotWare\RobotWare_6.08.0135\utility\Template\EGM`
* [egm_wrapper.proto](egm_wrapper.proto): Custom made for `ba7lya.abb.robot.egm` for wrapping the supported EGM features. Package `ba7lya.abb.robot.egm.wrapper`.
* [egm_wrapper_trajectory.proto](egm_wrapper_trajectory.proto): Custom made for `ba7lya.abb.robot.egm` for extending `egm_wrapper.proto` with trajectory messages. Package `ba7lya.abb.robot.egm.wrapper.trajectory`.
