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
- BME publishing
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

If adding features, prefer extending the appropriate layer instead of bypassing it.

## Build and validation commands

- `pio run`
- `pio run -t upload`
- `pio device monitor -b 921600`

Validated agent command currently documented for the project:

- `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600`

When making firmware changes, prefer validating with `pio run` if the environment allows it.

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
- the main known functional limitation before the first overall stable release is closed-loop velocity behavior
