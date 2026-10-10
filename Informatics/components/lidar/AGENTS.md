# Project

Portable YDLIDAR X2 driver/parser written in C.

The program is initially developed and tested on Linux using the
YDLIDAR X2 USB-to-UART adapter. It will later be migrated to an STM32.

# Architecture

The PC project is in `linux_implementation/`, including its
`bsp/`, `src/`, `tests/`, Makefile, and `output/` directories.
Run Linux builds and tests from that directory or use
`make -C linux_implementation` from the parent LiDAR directory.
The STM32 project is in `STM32_implementation/`. Its `App/lidar.c/.h`
contains the single portable parser used by both builds, and `BSP/bsp_lidar.h`
declares the common platform interface. The Linux Makefile compiles the
parser directly from that location. Platform implementations are
`linux_implementation/bsp/bsp_lidar_linux.c` and
`STM32_implementation/BSP/bsp_lidar.c`. Select only one BSP per build.
Shared protocol documentation stays in `doc/`.

Keep hardware/platform-specific code separate from LiDAR logic.

- `lidar.c/.h`: YDLIDAR X2 protocol parsing and LiDAR application logic.
- `bsp_lidar.h` and each platform implementation: LiDAR communication and hardware access.
  Prefix the common interface with `bsp_lidar_` to avoid conflicts with other components.
- `main.c`: application entry point.

Keep `lidar.c/.h` portable even though it resides inside the STM32 project.
Linux and STM32 use the same parser with their own BSP implementations.

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
- Default to guiding the user to write the code themselves. Present several
  related steps together, explain the reasoning and how to check them, and
  let the user implement the group before reviewing their combined changes.
- Include useful online search terms and pointers to relevant documentation
  or local library sources. Explain what to look for when researching a
  function, such as its parameters, return values, side effects, execution
  context, and blocking behavior.
- Only write or modify code when the user explicitly instructs you to
  "write". This applies to code in files and new code snippets in responses.
  Agreement with a proposal or instructions such as "go ahead" or
  "proceed" alone do not authorize writing code. Reading, explaining,
  reviewing, and checking existing code remain available as requested.
- Ask for validation before generating code unless the user has already
  explicitly authorized writing the concrete step under discussion.
- Keep development step by step, but group related steps into manageable
  increments rather than stopping after every small edit. Explain each
  step's purpose, implementation, and architectural choices thoroughly,
  then review the user's combined implementation before moving to the next
  group. When explicitly asked to write code, use the same grouped approach.
- Whenever using `volatile`, explicitly tell the user which variable or
  access uses it and explain why it is needed in that specific case,
  including who can modify the value independently of normal execution.
  Explain any relevant synchronization limits; `volatile` alone does not
  guarantee atomicity or thread safety.
- Do not modify files unnecessarily.
- This is a team project: prioritize clear separation of responsibilities
  and readable code that teammates can review easily. Keep unrelated
  operations and their error checks separate; do not sacrifice clarity
  to reduce line count.
- Keep Linux test captures and analysis outputs in
  `linux_implementation/output/` (`output/` when running inside the Linux project).
