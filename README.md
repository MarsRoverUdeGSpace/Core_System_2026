# Core System 2026

Core electronics, embedded firmware, and ROS 2 integration workspace for the UdeG Space 2026 rover platform.

![Project logo](docs/images/MarsRover.svg)

## Overview

This repository contains the validated `Kukulcan Rev A` controller hardware, the primary rover MCU firmware, and the ROS 2 workspace used to integrate the embedded system with the rest of the stack.

The current main firmware path, `kukulcan/firmware/upio-ros2/`, is a working ESP32-S3 implementation built on Arduino, FreeRTOS, and micro-ROS, with an AUTOSAR-inspired layered architecture deployed on real hardware.

This MCU sits at the intersection of the rover core system and the autonomous navigation stack for the Mars Rover UdeG Space Maya rover in URC 2026. It concentrates the odometry and control interfaces needed for vehicle state estimation and low-level motion integration, including wheel encoders, dual-IMU integration across the wider system, GNSS, altimeter data, communications with RoboClaw motor controllers, and communications with the main processing unit based on the NVIDIA Jetson Orin Nano.

The platform has already been validated on real hardware and is intended to continue system-level validation during field testing, including work planned for the Mars Desert Research Station.

## Current release state

This repository is past initial bring-up and already contains active work in four areas:

- `kukulcan/hardware/`: KiCad MCU design, 3D exports, fabrication outputs, and PCBA files for `MCU RevA`
- `kukulcan/firmware/upy/`: MicroPython-based sensor and UI bring-up
- `kukulcan/firmware/upio-ros2/`: ESP32-S3 PlatformIO firmware using Arduino, FreeRTOS, micro-ROS, and an AUTOSAR-inspired layered architecture
- `src/`: ROS 2 workspace with custom messages, a local `teleop_twistmux_node`, and vendored micro-ROS packages

As of June 2, 2026, this repository is the stable core-system release line for the rover MCU and ROS-facing embedded interface used by AutoNav.

Current status summary:

- hardware: validated RevA integration board with tracked manufacturing outputs
- firmware: stable release on real hardware with successful `pio run`
- localization interface: firmware-owned epoch-stamped `/odom`, direct BNO055 IMU, magnetometer, and GNSS streams validated on the Kukulcan PCB and consumed by the AutoNav stack
- ROS 2 workspace: active and buildable for the currently used packages
- control interface: `cmd_vel` response and motor timeout behavior corrected for the stable AutoNav integration path

Verified in this repository review:

- `colcon build --packages-select teleop_twistmux_node micro_ros_msgs drive_base_msgs`
- maintainer-provided successful `pio run` result for `upio-ros2`

## Stable release notes

This release promotes `feature/imu-mag-calibration` as the stable source line for `develop` and `main`.

Release highlights:

- integrated wheel odometry publication from RoboClaw encoder ticks
- added BNO055 magnetometer publishing for Jetson-side calibration and localization checks
- stabilized GNSS `NavSatFix`, IMU, magnetometer, and `/odom` timestamps in ROS epoch time
- made `/odom` reliable so full `nav_msgs/msg/Odometry` samples survive XRCE transport size limits
- bounded reliable publisher waits and scheduler catch-up behavior so delayed odometry acknowledgements do not starve IMU/GNSS publication
- calibrated odometry geometry and ticks-per-revolution values for the physical rover
- corrected `cmd_vel` response behavior and retained independent motor safety timeout handling

## Highlights

- `Kukulcan Rev A` is a 4-layer protected controller board built around an `ESP32-S3R8` with added `32 MB` flash
- the firmware architecture treats micro-ROS as the effective RTE boundary and FreeRTOS tasks as the application execution model
- the project has a reliable micro-ROS + FreeRTOS + Arduino implementation on the target MCU
- firmware `/odom` and direct IMU/GNSS topics now provide the validated inputs for Jetson-side EKF/global-localization testing
- the Jetson-side integration path is already designed into the hardware through the 40-pin expansion/control header

## Repository layout

| Path | Purpose |
| --- | --- |
| `kukulcan/hardware/` | MCU KiCad project, local libraries, manufacturing outputs |
| `kukulcan/firmware/` | Embedded firmware projects |
| `src/` | ROS 2 workspace packages and vendored dependencies |
| `docs/` | Datasheets, design notes, images, and release support material |
| `CONTRIBUTING.md` | Lightweight workflow and contribution rules |

## Primary subsystems

- Hardware: [kukulcan/hardware/README.md](kukulcan/hardware/README.md)
- Firmware: [kukulcan/firmware/README.md](kukulcan/firmware/README.md)
- Main firmware track: [kukulcan/firmware/upio-ros2/README.md](kukulcan/firmware/upio-ros2/README.md)

## Firmware architecture direction

The main embedded firmware track, `upio-ros2`, is not just an implementation of drivers and ROS transport. It is also the project's embedded software architecture reference:

- application logic is kept separate from hardware-facing details
- the runtime environment layer is effectively realized through the micro-ROS integration boundary
- hardware-dependent behavior is pushed into HAL-style modules and lower layers
- FreeRTOS tasking is used to preserve deterministic behavior across sensing, publishing, and actuation paths, effectively expressing the application layer execution model

That AUTOSAR-inspired separation is already a validated project strength and should be treated as part of the stable release narrative. The same applies to the reliable micro-ROS + FreeRTOS + Arduino implementation on the target MCU, which meaningfully reduces integration friction for the rover stack.

## Quick start

```bash
git clone git@github.com:MarsRoverUdeGSpace/Core_System_2026.git
cd Core_System_2026
git checkout main
```

ROS 2 packages can be listed with:

```bash
colcon list
```

The `upio-ros2` firmware lives in:

```bash
cd kukulcan/firmware/upio-ros2
pio run
```

The hardware project opens from:

```text
kukulcan/hardware/MCU/MCU.kicad_pro
```
