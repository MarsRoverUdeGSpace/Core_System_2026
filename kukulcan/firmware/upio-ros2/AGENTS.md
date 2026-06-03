# AGENTS.md

This file is for Codex and other coding agents working inside `kukulcan/firmware/upio-ros2/`. It defines how to reason about this firmware subtree, what has already been implemented, and how to modify it safely.

## Subtree role

This is the main MCU firmware track for the Kukulcan controller board. It is not a placeholder project. It is a validated beta firmware built around:

- `ESP32-S3R8` target board configuration with additional external flash
- PlatformIO build flow
- Arduino framework
- FreeRTOS task-based concurrency
- micro-ROS communication runtime
- AUTOSAR-inspired layering across application, runtime, and hardware-facing code

Use this subtree as the primary firmware context unless the user explicitly asks to work on `kukulcan/firmware/upy/`.

## Role of this file

Use this file to guide Codex behavior when reading or editing this subtree:

- understand the intended architecture before modifying code
- keep edits aligned with the layered design
- preserve the existing concurrency model unless the user asks for architectural changes
- treat the local `README.md` as user-facing documentation and this file as agent-facing implementation context

## Implemented structure

- `src/main.cpp`: Arduino entry point
- `lib/app/`: application layer and FreeRTOS task startup
- `lib/rte/`: runtime environment boundary, implemented around micro-ROS
- `lib/config/`: board-specific configuration, buses, pins, shared objects, queues, and mutexes
- `lib/hal_*`: hardware-facing modules and complex-driver style abstractions
- `boards/kukulcan_esp32s3.json`: local PlatformIO board definition
- `platformio.ini`: build environment and dependencies

## Architecture model

This firmware should be understood as an AUTOSAR-inspired implementation adapted to the actual rover stack:

- Application Layer:
  - primarily `lib/app/`
  - expressed operationally through coordinated FreeRTOS tasks
- Runtime Environment:
  - `lib/rte/`
  - effectively the micro-ROS integration boundary
- ECU abstraction and complex drivers:
  - `lib/hal_*`
  - selected hardware-facing behavior in `lib/config/`
- MCU-specific configuration:
  - ESP32/Arduino details in `lib/config/` and board configuration files

When editing, preserve this separation unless the user explicitly asks for a refactor.

## Current implemented behavior

The subtree already supports and documents:

- micro-ROS transport over serial
- FreeRTOS-based task partitioning
- `cmd_vel` consumption for motor commands
- motor-control task separation from ROS executor servicing
- encoder publishing
- IMU publishing
- BNO055 magnetometer publishing as `sensor_msgs/MagneticField` on `/sensors/bno055/mag`
- BME publishing
- GNSS `NavSatFix` publishing
- app-layer encoder wheel odometry publishing on `/odom`
- LED status handling
- serial-agent workflow used with `micro_ros_agent`

Do not rewrite docs or code as if these features are unimplemented.

## Concurrency and tasking assumptions

Current architecture expectations:

- one task services the micro-ROS executor
- publishing is separated from direct sensor access
- hardware-facing work stays in HAL-style modules
- configuration and shared objects live in `lib/config/`
- application logic should not absorb hardware-specific details unnecessarily
- localization messages must use ROS epoch stamps consistently; never publish a fallback MCU uptime stamp once ROS consumers may fuse the stream
- `/odom` remains reliable because the covariance-bearing message exceeds the configured `512` byte best-effort XRCE MTU, but reliable publisher waits must stay bounded so odometry cannot stall IMU/GNSS output
- periodic RMW health/time calls must use the RCL transport mutex; the micro-ROS stack is not a concurrent side channel for publisher tasks
- after a delayed publication cycle, drop missed slots instead of burst-publishing overdue localization messages

If adding features, prefer extending the appropriate layer instead of bypassing it.

## Build and validation commands

- `pio run`
- `pio run -t upload`
- `pio device monitor -b 921600`

Validated agent command currently documented for the project:

- `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600`

When making firmware changes, prefer validating with `pio run` if the environment allows it.

For localization-affecting changes, verify on hardware before committing:

- `/sensors/bno055/imu/data`, `/sensors/gnss/fix`, and `/odom` stamps are all current ROS epoch time, not MCU uptime
- expected nominal output is IMU near `25 Hz`, magnetometer/GNSS near `5 Hz`, and `/odom` near `10 Hz`
- no multi-second publication gaps are introduced by reliable `/odom` delivery
- `status: -1` with zero GNSS coordinates is a valid no-fix report; require `status: 0` outdoors before evaluating navigation position

When changing the number of micro-ROS entities, also check the generated rmw micro XRCE-DDS limits. This firmware uses a project-local `kukulcan-colcon.meta` through `board_microros_user_meta` to raise `RMW_UXRCE_MAX_PUBLISHERS` above the default. If a new publisher/subscriber makes the ESP32 partially connect to the agent, show topics briefly, then restart, suspect a strict `RCCHECK(...)` failure during entity creation caused by these limits. After changing the meta file, force regeneration with:

- `pio run -t clean_microros`
- `pio run`

Confirm the generated value in `.pio/libdeps/kukulcan/micro_ros_platformio/libmicroros/include/rmw_microxrcedds_c/config.h`.

## Editing guidance

- Keep Doxygen-style headers consistent with existing files
- Follow existing naming conventions such as `app_Init`, `rte_*`, `Hal_*`, and `_t` suffixes for types
- Avoid collapsing `app`, `rte`, `config`, and `hal_*` responsibilities together
- Prefer small, architecture-consistent changes over broad rewrites
- If a requested change crosses multiple layers, explain the architectural impact clearly in docs or code comments only where needed

## Current project realities Codex should know

- this firmware is already considered a stable beta by the maintainer
- the architecture itself is part of the delivered value, not just an implementation detail
- micro-ROS on FreeRTOS with Arduino is intentionally emphasized because it is reliably implemented here on real hardware
- 2026-05-27 PCB validation confirmed epoch-aligned IMU/GNSS/odom stamps and removed multi-second publish stalls after bounding reliable transport waits; observed IMU rate was about `22-23.6 Hz`, slightly below its `25 Hz` configuration target
- the main known functional limitation before the first overall stable release is closed-loop velocity behavior
