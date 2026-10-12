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

`main.c` calls `lidar_test_init()` after peripheral initialization and
`lidar_test_process()` on each loop iteration. The test module initializes
the parser and starts BSP reception, reporting startup failure to `main.c`.
Each processing call receives/parses at most 256 bytes, then retrieves at
most `LIDAR_QUEUE_CAPACITY` points. The latest point is retained for inspection.
The temporary raw-reader test in `main.c` has been replaced by this module.

## Inspect the parser test

Refresh and build the CubeIDE project, then load the new firmware in the
debugger. Add these expressions:

- `lidar_test_diagnostics.initialized`: true after successful startup.
- `lidar_test_diagnostics.parser.received_bytes`: bytes fed to the parser.
- `lidar_test_diagnostics.parser.valid_packets`: checksum-validated packets.
- `lidar_test_diagnostics.parser.rejected_packets`: invalid packet candidates.
- `lidar_test_diagnostics.parser.decoded_points`: decoded samples, including
  zero-distance returns.
- `lidar_test_diagnostics.parser.dropped_points`: parser queue overflows.
- `lidar_test_diagnostics.read_errors`: BSP read failures.
- `lidar_test_diagnostics.consumed_points`: points retrieved by the test.
- `lidar_test_diagnostics.has_point`: whether any point has been retrieved.
- `lidar_test_diagnostics.latest_point`: angle in degrees, distance in
  millimetres, and the rotation-start flag. Zero distance means no return.
- `lidar_test_diagnostics.zero_angle`: latest point in the inclusive
  358..360 / 0..2 degree window around sensor zero. `has_point` indicates
  that a matching sample has arrived; `has_return` indicates its distance
  is nonzero. `matched_points` counts all matching samples, including zero
  distances. A retained point can be old if reception stops; verify this
  count increases while observing the distance.

To test a target, place it in the sensor's documented zero-degree direction
at a measured distance. Observe `zero_angle.latest_point.distance_mm`, then
move or remove the target and confirm the readings change accordingly.
The diagnostic selects an angular window; it does not identify objects.

`lidar_test_diagnostics` is volatile to preserve writes for debugger
observation, including optimized builds. It is updated by application code;
volatile does not make its fields a synchronized snapshot.

Let the board run continuously between observations. Debugger pauses may
let DMA overwrite unread bytes, so rejected packets after a pause are not
conclusive evidence of a parser defect. The BSP does not yet detect DMA
overwrites or provide UART-error recovery. `read_errors` counts errors
returned by the BSP; it does not independently inspect the UART error flags.
This is a parser-integration test, not completed robot reception logic.
