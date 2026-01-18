# Repository Guidelines

## Project Structure & Module Organization
- `src/` contains the Arduino entry point (`src/main.cpp`).
- `lib/` holds application modules:
  - `lib/app/` for application logic and FreeRTOS tasks.
  - `lib/rte/` for micro-ROS runtime integration (publish/subscribe + executor).
  - `lib/config/` for hardware configuration (UART + queue creation).
- `platformio.ini` defines the ESP32-S3 target, Arduino framework, and library dependencies.

## Architecture Overview
- Target: MCU firmware for odometry handling, aligned with AUTOSAR-style layering for microcontrollers.
- Hardware: ESP32-S3 dual-core MCU.
- Runtime: micro-ROS is the runtime environment (cmd_vel subscriber + app_hello publisher).
- Concurrency: FreeRTOS kernel with two tasks pinned to different cores for parallel processing.
- Data flow: micro-ROS `cmd_vel` -> RTE callback -> FreeRTOS queue -> motor control task, which will drive RoboClaw motor commands on the other core for real-time parallel processing.
- Navigation frame convention (Nav2-compatible): publish `nav_msgs/Odometry` in `odom` with `child_frame_id=base_link`, and provide the `odom -> base_link` TF from the MCU; `map -> odom` is expected from higher-level localization.
- Sensor publishing plan (pending): publish raw encoder-based odometry, `sensor_msgs/Imu`, and `sensor_msgs/NavSatFix` separately for fusion (e.g., `robot_localization`).

## Build, Test, and Development Commands
This project uses PlatformIO.
- `pio run` builds the firmware for `env:esp32-s3-devkitc-1`.
- `pio run -t upload` flashes the firmware to a connected board.
- `pio device monitor -b 921600` opens the serial monitor at the configured baud rate.

## Coding Style & Naming Conventions
- Language: C++ (Arduino + FreeRTOS).
- Indentation is consistent with existing files (primarily 2 spaces, no tabs).
- Doxygen-style file and function headers are used; follow the same style for new modules.
- Naming patterns:
  - Functions: `app_Init`, `app_Run` (lowercase prefix + `_`).
  - Types: `cmd_velQueueMsg_t` (`_t` suffix).
  - Constants: `static constexpr` for configuration, `_ticks` suffix for timing values.

## Testing Guidelines
- No automated test framework is configured in this repository.
- Validate changes with a local build (`pio run`) and, when applicable, on-device testing with serial output.

## Commit & Pull Request Guidelines
- Recent commits use Conventional Commit-style prefixes with scopes (e.g., `feat(kukulcan-fw): ...`, `docs(datasheets): ...`), but some commits are free-form.
- Prefer `type(scope): summary` when possible to keep history consistent.
- For pull requests, include:
  - A short summary of changes.
  - Build/test status (e.g., `pio run` or on-device verification).
  - Hardware used for validation (e.g., ESP32-S3 DevKitC-1).

## Configuration Tips
- `platformio.ini` selects `espressif32` + Arduino and pulls micro-ROS and device libraries via `lib_deps`.
- If serial output looks garbled, verify the monitor speed matches `monitor_speed = 921600`.
