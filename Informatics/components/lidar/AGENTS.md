# Project

Portable YDLIDAR X2 driver/parser written in C.

The program is initially developed and tested on Linux using the
YDLIDAR USB-to-UART adapter. It will later be migrated to an STM32.

# Architecture

Keep hardware/platform-specific code separate from LiDAR logic.

- `lidar.c/.h`: YDLIDAR X2 protocol parsing and LiDAR application logic.
- `bsp.c/.h`: platform-specific communication and hardware access.
- `main.c`: application entry point.

The goal is for `lidar.c/.h` to migrate to STM32 with little or no
modification. The Linux BSP will eventually be replaced by an STM32 BSP.

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
