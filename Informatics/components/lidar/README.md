# YDLIDAR X2 project

- `linux_implementation/`: PC entry point, Linux BSP, terminal map, tests,
  and generated outputs.
- `STM32_implementation/`: STM32Cube project and the portable parser used
  by both platforms.
- `doc/`: shared LiDAR development manual and its text extraction.
- `AGENTS.md`: development and architecture instructions for both platforms.

The single portable parser resides in `STM32_implementation/App/lidar.c/.h`.
Its platform interface is declared in `STM32_implementation/BSP/bsp_lidar.h`.
The Linux Makefile uses those files directly and selects
`linux_implementation/bsp/bsp_lidar_linux.c`. The STM32 BSP is being built step
by step in `STM32_implementation/BSP/bsp_lidar.c` (startup, RX event
diagnostics, and circular-buffer reading so far). PC display modules stay
in the Linux project.

CubeIDE Debug and Release configurations include the `App/` and `BSP/`
source directories and header paths. Refresh the project after this move.
The STM32 entry point runs a bounded parser-integration test through
`App/lidar_test.c`, exposing parser statistics and the latest point to the
debugger. DMA overwrite detection and UART-error recovery remain unfinished.

The STM32 project also has a standalone CMake build. See
[the STM32 README](STM32_implementation/README.md) for build commands and
the current reception integration stage.

## Build and run on Linux

From this directory:

```sh
make -C linux_implementation
./linux_implementation/output/lidar-reader
```

Or enter `linux_implementation/` and use the existing commands:

```sh
cd linux_implementation
make
./output/lidar-reader
```

Run all checks with:

```sh
make -C linux_implementation test test-map test-replay
```

Captures and test results are in `linux_implementation/output/`.
See [the Linux README](linux_implementation/README.md) for map settings,
protocol details, and test descriptions.
