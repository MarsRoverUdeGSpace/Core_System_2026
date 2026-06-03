# Kukulcan Firmware

This directory groups the embedded firmware projects for the Kukulcan controller stack.

The controller firmware in this repository is part of the rover core system and the autonomous navigation stack for the Mars Rover UdeG Space Maya rover in URC 2026. It is responsible for the MCU-side integration of sensing, low-level actuation interfaces, and communication with the Jetson Orin Nano main processing unit.

## Status

The firmware work in this repository is split into two tracks with different roles:

- `upy/`: used for rapid prototyping, sensor bring-up, and integration testing validation
- `upio-ros2/`: current real implementation on ESP32-S3 using PlatformIO, Arduino, FreeRTOS, and micro-ROS

## Projects

- `upy/`: MicroPython-based rapid prototyping and validation path
- `upio-ros2/`: stable firmware implementation and primary release-facing embedded stack

## Current validation

Physical Kukulcan validation confirmed the `upio-ros2/` ROS-facing localization contract: firmware publishes epoch-stamped `/odom`, `/sensors/bno055/imu/data`, `/sensors/bno055/mag`, and `/sensors/gnss/fix` without the prior multi-second publish stalls. The AutoNav Jetson integration path consumes this odometry, IMU, magnetometer, and GNSS contract directly; detailed rates, transport constraints, scheduling behavior, and smoke-test commands are documented in [`upio-ros2/README.md`](upio-ros2/README.md).
