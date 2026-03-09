# Kukulcan Firmware

This directory groups the embedded firmware projects for the Kukulcan controller stack.

The controller firmware in this repository is part of the rover core system and the autonomous navigation stack for the Mars Rover UdeG Space Maya rover in URC 2026. It is responsible for the MCU-side integration of sensing, low-level actuation interfaces, and communication with the Jetson Orin Nano main processing unit.

## Status

The firmware work in this repository is split into two tracks with different roles:

- `upy/`: used for rapid prototyping, sensor bring-up, and integration testing validation
- `upio-ros2/`: current real implementation on ESP32-S3 using PlatformIO, Arduino, FreeRTOS, and micro-ROS

## Projects

- `upy/`: MicroPython-based rapid prototyping and validation path
- `upio-ros2/`: main firmware implementation and primary release-facing embedded stack
