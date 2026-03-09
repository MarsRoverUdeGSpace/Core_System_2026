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

As of March 9, 2026, the repository should be treated as a mature beta integration branch preparing for additional beta releases before the first overall stable release.

Current status summary:

- hardware: validated RevA integration board with tracked manufacturing outputs
- firmware: stable beta on real hardware with successful `pio run`
- ROS 2 workspace: active and buildable for the currently used packages
- main remaining functional gap before the first overall stable release: closed-loop velocity behavior

Verified in this repository review:

- `colcon build --packages-select teleop_twistmux_node micro_ros_msgs drive_base_msgs`
- maintainer-provided successful `pio run` result for `upio-ros2`

## Release track

Planned release sequence:

1. More beta releases on `develop`
2. One pre-final beta release to harden documentation, validation evidence, and reproducible builds
3. First stable release after closed-loop behavior, validation evidence, and release documentation are finalized

## Highlights

- `Kukulcan Rev A` is a 4-layer protected controller board built around an `ESP32-S3R8` with added `32 MB` flash
- the firmware architecture treats micro-ROS as the effective RTE boundary and FreeRTOS tasks as the application execution model
- the project has a reliable micro-ROS + FreeRTOS + Arduino implementation on the target MCU
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

That AUTOSAR-inspired separation is already a validated project strength and should be treated as part of the beta release narrative. The same applies to the reliable micro-ROS + FreeRTOS + Arduino implementation on the target MCU, which meaningfully reduces integration friction for the rover stack.

## Quick start

```bash
git clone git@github.com:MarsRoverUdeGSpace/Core_System_2026.git
cd Core_System_2026
git checkout develop
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
