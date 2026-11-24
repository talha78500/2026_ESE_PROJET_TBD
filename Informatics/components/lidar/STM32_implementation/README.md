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
`lidar_test_process(HAL_GetTick())` on each loop iteration. The test module initializes
the parser and starts BSP reception, reporting startup failure to `main.c`.
Each processing call receives/parses at most 256 bytes, then retrieves at
most `LIDAR_QUEUE_CAPACITY` points. The latest point is retained for inspection.
The temporary raw-reader test in `main.c` has been replaced by this module.
Initialization also accepts a transient BSP startup failure for background
retry, so the rest of the application can run. `initialized` indicates the
test module is ready; inspect `bsp.state` for actual reception readiness.

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
- `lidar_test_diagnostics.bsp`: transport state and cumulative diagnostics
  for copies, discards, overwrites, UART/DMA errors, restarts, and failures.
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

## DMA overwrite detection

The BSP snapshots the serviced DMA lap count, the hardware completion flag,
and the remaining-transfer counter. It briefly masks CPU interrupts while
sampling, then restores the previous interrupt mask. DMA keeps running;
the copy itself does not mask interrupts. A wrap during sampling defers
the read to a later application call rather than waiting in a retry loop.
One pending completion is accounted for locally without clearing HAL's flag.

Total production is compared with successful copies plus intentionally
discarded bytes. A backlog of exactly 1024 bytes is full, not empty. More
than 1024 means unread data was overwritten. The BSP also checks progress
after copying; if the original source may have been overwritten, that copy
is not published to the caller.

On overwrite, the BSP discards the entire unread backlog, moves the reader
to the sampled producer position, and returns an error with zero received
bytes. The portable parser then drops its unfinished packet, preserving
statistics and already-decoded points. Later bytes start a new packet search.

For debugging, inspect the private BSP variables:

- `rx_dma_wrap_count`: serviced transfer-complete notifications.
- `rx_consumed_bytes`: bytes successfully copied to callers.
- `rx_discarded_bytes`: unread bytes skipped during recovery, including
  both overwritten data and the rest of the discarded backlog.
- `rx_overrun_count`: detected overwrite/recovery events. This application-
  owned diagnostic is volatile to retain writes for debugger observation;
  volatile is not the synchronization mechanism used by the snapshot.

**Timing requirement:** DMA completion interrupts must be serviced at least
once per buffer lap (about 89 ms at a continuous 115200 baud, 8N1 stream).
A sticky completion flag cannot count multiple laps while the CPU is halted
or interrupts are masked for too long. Prefer live debugger observations;
restart the board after long halts before evaluating data-loss counters.
To test an overwrite, delay servicing the reader while leaving interrupts
running, rather than halting the CPU.

## UART/DMA fault recovery and lifecycle

`HAL_UART_ErrorCallback()` records UART/DMA error codes and counters, marks
reception unavailable, and returns. It does not parse bytes or restart UART.
Shared fault/abort flags, error generations, and ISR-written diagnostics are
volatile; brief critical sections protect compound snapshots and publishing
copied data. The reader discards any copy interrupted by a reported error.
Keep UART/DMA handlers serialized (the generated configuration uses the
same preemption priority for both).

Application code calls `bsp_lidar_process()` regularly, even when it is not
reading points. This services nonblocking `HAL_UART_AbortReceive_IT()` and
its completion callback, then retries reception after 100 ms. One call
performs one step; there are no polling waits or receive loops. Failed
starts are cleaned up before another attempt. A failed abort or an abort
completion missing for 100 ms produces FAULTED; it does not pretend DMA
has stopped. An explicit stop can retry/collect a late completion, but a
permanent hardware failure may require resetting the board.

`bsp_lidar_deinit()` requests STOPPING and immediately prevents further
byte delivery. Continue servicing `bsp_lidar_process()` until STOPPED,
then call `bsp_lidar_init()` to restart. A stop cancels automatic recovery.
Repeated stop requests are safe; init while already active/stopping is
rejected. This stops reception only: it does not switch LiDAR power or PWM.

The public states are STOPPED (0), RUNNING (1), STOPPING (2), RECOVERING (3),
and FAULTED (4). `bsp_lidar_get_stats()` returns a consistent snapshot.
Counters remain cumulative across restarts; private byte-position accounting
is reset for every new DMA session. A read error is reported at a session
boundary so an unfinished parser packet cannot span two sessions.

The Linux BSP implements the same service/statistics interface. Its service
call has no deferred work, and deinit closes the serial port immediately;
UART/DMA-specific statistics remain zero.

## Board-test commands

Use CubeIDE Live Expressions with ST-LINK while the CPU runs. Set any
parameters first, then write `lidar_test_controls.command` last. The command
is acknowledged by returning to zero. If you halt the CPU long enough to
invalidate DMA lap accounting, restart the board before testing.

| Command value | Action |
| --- | --- |
| 0 | No request |
| 1 | Pause the reader for `reader_pause_ms` (default 250 ms, maximum 500 ms), then resume automatically |
| 2 | Request reception stop |
| 3 | Start from STOPPED |
| 4 | Stop and then start automatically after confirmed shutdown |

The controls are volatile because the debugger changes them independently
of application execution. They are test inputs, not a thread-safe messaging
API. `command_status` is zero on an accepted request and -1 on rejection;
`reader_pause_active` shows whether the timed pause is in progress.

1. With normal live input, confirm RUNNING and increasing valid packets.
2. Set `reader_pause_ms` to 250, then command to 1. DMA/interrupts and BSP
   servicing keep running; only reading/parsing is temporarily skipped.
3. After it resumes, expect `bsp.overwrite_events` and `bsp.discarded_bytes`
   to increase if enough bytes arrived. Confirm valid packets resume and
   reception remains RUNNING. A silent/disconnected sensor will not create
   an overwrite merely because the timer elapsed.
4. Set command 2 and wait for STOPPED. No new decoded points should appear.
5. Set command 3 and confirm RUNNING and new valid packets. Repeat, or use
   command 4 for the combined restart path. `bsp.restarts` should increase.

`has_point` flags are cleared while stopped/recovering or after read errors;
retained point values should only be interpreted when those flags are true.
`read_errors` includes overwrite/session-discontinuity reports. UART/DMA
error counts and last error codes are separate in `bsp`.

Host tests inject UART/DMA errors, failed starts, synchronous/asynchronous
aborts, abort failure/timeout, errors during copying/startup, timed pauses,
stop/start commands, and counter/timestamp rollover. The actual parser is
linked into these tests and must resume decoding after overwrite/restart.
No ST-LINK was connected during implementation, so live overwrite recovery,
electrical UART fault recovery, and repeated board stop/start validation
are still pending. Test commands do not inject fake hardware error callbacks.
