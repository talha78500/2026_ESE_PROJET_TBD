# Project

Portable YDLIDAR X2 driver/parser written in C.

The program is initially developed and tested on Linux using the
YDLIDAR X2 USB-to-UART adapter. It will later be migrated to an STM32.

# Architecture

Keep hardware/platform-specific code separate from LiDAR logic.

- `lidar.c/.h`: YDLIDAR X2 protocol parsing and LiDAR application logic.
- `bsp.c/.h`: platform-specific communication and hardware access.
- `main.c`: application entry point.

The goal is for `lidar.c/.h` to migrate to STM32 with little or no
modification. The Linux BSP will eventually be replaced by an STM32 BSP.

## Execution model

The final robot application will be event driven and must serve multiple
motors, sensors, and outputs. An RTOS may be used on STM32, but that choice
and the event dispatch mechanism are not decided yet. Do not couple the
LiDAR logic to a particular scheduler or RTOS.

- Keep `main.c` minimal, with application initialization and orchestration.
- Do not put an infinite receive loop inside the LiDAR driver.
- Runtime reception and processing must be nonblocking: do not wait for
  incoming bytes, complete packets, or hardware readiness.
- Each processing call must perform bounded work and return control to the
  application so other components can run.
- Retain parser state and incomplete packets between calls. Serial reads
  may contain partial packets or multiple packets.
- Keep UART interrupts, DMA, and platform-specific buffering in the BSP.
  The portable LiDAR parser consumes bytes independently of how they arrive.

# Development

- Language: C.
- Current platform: Fedora Linux.
- LiDAR: YDLIDAR X2.
- Linux serial device will typically be `/dev/ttyUSB0`.
- Use embedded-compatible C.
- Do not use dynamic allocation.
- Do not introduce Linux-specific dependencies into `lidar.c`.
- Explain protocol/architectural decisions rather than blindly generating code.
- Ask for validation before generating code.
- Do not modify files unnecessarily.
- This is a team project: prioritize clear separation of responsibilities
  and readable code that teammates can review easily. Keep unrelated
  operations and their error checks separate; do not sacrifice clarity
  to reduce line count.
- Keep test captures and analysis outputs in `output/`.
