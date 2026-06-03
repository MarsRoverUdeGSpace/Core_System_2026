# AGENTS.md

This file is for Codex and other repository-aware coding agents. It is not user-facing release documentation. Its purpose is to describe how the repository is organized, what is already implemented, and how an agent should work safely inside it.

## Repository status

As of June 2, 2026, this repository is the stable core-system release line for the UdeG Space 2026 rover MCU and embedded ROS interface. It already contains:

- validated `Kukulcan Rev A` controller hardware sources and tracked manufacturing outputs
- a working primary MCU firmware track in `kukulcan/firmware/upio-ros2/`
- a legacy or bring-up-oriented MicroPython track in `kukulcan/firmware/upy/`
- a ROS 2 workspace with local and vendored packages under `src/`

Do not describe `src/` as future-only. ROS 2 packages already exist and build in this repository.

## Role of this file

Use this file as the top-level operating context for Codex work in this repository:

- prefer local subsystem READMEs for user-facing status and architecture descriptions
- use this file for agent workflow, repository constraints, and subsystem routing
- when a deeper `AGENTS.md` exists in a subdirectory, treat it as authoritative for work inside that subtree

## Repository structure

- `kukulcan/hardware/MCU/`: KiCad project files, board exports, and generated hardware outputs
- `kukulcan/hardware/Libraries/`: shared symbols, footprints, and 3D assets
- `kukulcan/hardware/mfr/`: tracked fabrication outputs and PCBA files
- `kukulcan/firmware/upy/`: MicroPython-based bring-up firmware
- `kukulcan/firmware/upio-ros2/`: main ESP32-S3 firmware based on PlatformIO, Arduino, FreeRTOS, and micro-ROS
- `src/`: ROS 2 workspace containing local packages and vendored dependencies
- `docs/`: images, datasheets, and support documentation
- `build/`, `install/`, `log/`: generated ROS 2 workspace outputs

## Agent working rules

- Treat `kukulcan/firmware/upio-ros2/` as the main embedded firmware path unless the user explicitly asks for `upy/`
- Treat `kukulcan/hardware/mfr/` as tracked release artifacts, not disposable exports
- Avoid editing `build/`, `install/`, or `log/`
- Be careful around KiCad-generated and binary hardware outputs; update them only when the hardware task requires it
- If documentation is changed, keep `README.md` files user-facing and keep `AGENTS.md` files agent-facing

## Build and validation commands

Use subsystem-specific commands when available.

- ROS 2 workspace:
  - `colcon list`
  - `colcon build`
  - `colcon build --packages-select <pkg>`
- Main firmware:
  - `cd kukulcan/firmware/upio-ros2`
  - `pio run`
  - `pio run -t upload`
  - `pio device monitor -b 921600`
- Hardware:
  - use KiCad project files under `kukulcan/hardware/MCU/`
  - manufacturing outputs in `kukulcan/hardware/mfr/` are tracked artifacts

Do not default to `idf.py` for this repository unless the user explicitly introduces an ESP-IDF subproject.

## Documentation expectations

- `README.md` files should describe current status, architecture, validation, and known limitations of the relevant subsystem
- `AGENTS.md` files should help Codex understand how to navigate and edit the repository safely
- avoid putting maintainer to-do notes, placeholders, or editorial guidance into release-facing READMEs

## Commit and PR guidance

- Branches:
  - `sw/*` for software and firmware
  - `hw/*` for hardware
  - `misc/*` for documentation, CI, and repository chores
- Prefer Conventional Commits with scoped subjects
- Keep logical hardware design edits separate from regenerated manufacturing outputs when practical
- Open PRs against `develop`

## Current repository realities Codex should know

- `Kukulcan Rev A` hardware is documented as validated integration hardware
- `upio-ros2` is a validated stable firmware track, not a speculative scaffold
- the firmware architecture is intentionally AUTOSAR-inspired, with micro-ROS acting as the effective RTE boundary and FreeRTOS tasks expressing the application layer
- as of 2026-05-27, use firmware-owned `/odom`, `/sensors/bno055/imu/data`, and `/sensors/gnss/fix` as the tested raw localization interface; stamps must remain ROS epoch time and `/odom` reliable publication must not starve sensor topics
- stable AutoNav integration depends on the current `/odom`, BNO055 IMU/magnetometer, GNSS, scheduler, and `cmd_vel` contract; do not relax epoch-only timestamps, reliable `/odom`, or bounded publisher waits without field retesting
