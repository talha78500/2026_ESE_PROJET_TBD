# STM32 LiDAR build

The CMake build compiles the existing CubeMX-generated code, portable
`App/lidar.c`, and STM32 `BSP/bsp_lidar.c`. It uses the existing STM32L476
startup assembly and flash linker script. The Linux BSP is not selected.

## Build with CMake

Install CMake and an `arm-none-eabi-gcc` toolchain with newlib, including
`nano.specs` and `nosys.specs`. The compiler must be on your `PATH`.

From the parent LiDAR directory:

```sh
cmake -S STM32_implementation -B STM32_implementation/output/cmake-debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build STM32_implementation/output/cmake-debug --parallel
```

The build directory contains `STM32_implementation.elf`, its `.map` file,
and `compile_commands.json`. For Release, use a separate directory and
`-DCMAKE_BUILD_TYPE=Release`. CMake builds the firmware; it does not flash
the board or run a hardware test.

## How App and BSP are connected

- `cmake/arm-none-eabi-gcc.cmake` selects the ARM compiler and tells CMake
  this is a bare-metal cross-build.
- `target_sources()` in `CMakeLists.txt` selects the C and assembly files
  to compile. Add future application modules to that list.
- `target_include_directories()` adds `App/` and `BSP/` to header search
  paths. `main.c` still needs to include `lidar.h` or `bsp_lidar.h` when
  using their APIs, preferably inside CubeMX's protected user sections.
- Compile definitions and CPU options select the STM32L476 and its
  Cortex-M4 floating-point ABI. Link options use the board's memory layout
  and enable removal of unused functions.

This is an additional command-line build. The existing CubeIDE project
continues to use its managed Make build unless separately reconfigured.
CubeMX regeneration may require updating the explicit HAL source list if
you enable new peripherals.

## Current integration stage

`main.c` currently calls `bsp_lidar_init()`. The BSP starts DMA reception
and records RX event diagnostics. The parser is compiled but is not yet
called by the STM32 application. Unused parser functions are removed from
the ELF; in particular, this build does not prove `lidar_process()` works
on STM32 because `bsp_lidar_read()` is not implemented yet.
