# YDLIDAR X2 reception and parser

Build the Linux reader with `make`. Run `./output/lidar-reader`; Ctrl+C
closes the serial connection and reports the received byte count.
The PC application shows a live top-down ASCII map of the latest completed
rotation, refreshing at most four times per second. `R` marks the LiDAR and
`*` marks a return. Zero-distance returns are omitted. Reception and parsing
continue at full speed; the map does not classify cans or walls.
The default port is `/dev/ttyUSB0`, configured at 115200 baud, 8N1.

## PC terminal map

The default display radius is 2 metres, including measurements exactly at
that radius. To display a different radius:

```sh
LIDAR_MAP_RANGE_M=3 ./output/lidar-reader
```

The radius setting accepts 0.1 to 20 metres; changing the display radius
does not change the sensor's hardware range. Sensor 0 degrees points upward
and 90 degrees points right, following its clockwise angle convention.
These directions are not calibrated against the robot heading.

The grid fits the terminal dimensions detected at startup. Enlarge the
terminal and restart for a finer map. Cells assume a 2:1 character height
to width ratio; the display prints millimetres per column and row.
Several returns can share one cell, so the visible-point count is not the
number of occupied cells. Points at the origin cell are covered by `R`.

The initial partial rotation is ignored. A scan is published between two
rotation markers unless parser queue drops were observed. Checksum losses
can still leave gaps; the display includes rejected-packet and dropped-point
counts. Scan age indicates whether the last completed scan is becoming stale.

On an interactive terminal, ANSI escape sequences redraw an alternate screen
and hide the cursor. Ctrl+C restores the normal screen and cursor on a
best-effort basis. Redirected output contains plain-text frame snapshots.
Writes are nonblocking for terminals and pipes; slow display consumers cause
frames to be skipped rather than delaying reception. Partial writes finish
before another frame is generated. All display storage is fixed.

## Responsibilities

- `src/main.c`: standalone Linux application initialization and orchestration.
- `src/pc_map.c/.h`: PC-only scan accumulation, coordinate mapping, terminal
  rendering, and refresh timing. Consumes the public parser API without
  changing it. This module is omitted from STM32 builds.
- `src/pc_report.c/.h`: previous text-reporting module, retained but not linked
  into the current map application.
- `bsp/bsp.c`: Linux serial configuration and nonblocking reads.
- `app/lidar.c`: measurement framing, validation, decoding, and fixed queue.

The parser has no OS dependencies or dynamic allocation. Its single instance
must be accessed serially by the application; it is not an ISR or concurrent
producer/consumer API.

## Parser interface

Call `lidar_init()` to reset parser state, queue, and diagnostics.
`lidar_process()` reads and parses at most 256 bytes without waiting.
Alternatively, `lidar_feed()` parses supplied bytes, returning the number
consumed (at most 256). Supply any remaining bytes in later calls.

After processing, retrieve points with `lidar_get_point()` until the queue
is empty. Each point has a corrected angle in degrees, a distance in
millimetres, and a rotation-start marker. Zero distance means no return.
Choose the application-specific treatment of those missing returns.

The 512-point queue drops newest points when full and increments
`dropped_points` in `lidar_get_stats()`. Drain it regularly. The PC map
drains it on every application iteration, including points outside the
display range. Missed display frames are counted separately from parser
point drops; they do not prevent collecting subsequent scans.
The standalone PC main loop currently polls continuously.

## Protocol decisions

Measurement packets have a 10-byte header followed by two bytes per sample.
The one-byte count allows at most 255 samples, so packet storage is bounded
at 520 bytes. Fields are decoded explicitly in little-endian order.

The parser searches for `AA 55`, validates the sample count and encoded
angles, then verifies the 16-bit XOR checksum before emitting any points.
Only CT bit 0 identifies a new rotation; other CT bits are not interpreted.
Startup messages are not decoded in this version. Bytes outside validated
measurement packets are skipped, allowing attachment to an ongoing scan.

Distance uses the development manual's raw sample divided by four. Angle
decoding uses the encoded endpoints divided by 64 after removing their check
bit, interpolation across 360 degrees, and the manual's arctangent correction.
The single-sample case avoids division by zero. Portable C math functions
require linking the math library (`-lm` on Linux).

Incomplete candidates persist across calls. Invalid candidates advance by
one byte, retaining possible later headers for resynchronization. A corrupted
but plausible sample count can delay recovery until its advertised length
arrives (at most 520 bytes); it does not block execution. Rejected-packet
statistics count invalid header candidates, not necessarily physical packets.

## Validation

`make test` checks manual calculation examples, packet fragmentation,
wraparound, missing returns, malformed packets, corruption recovery,
maximum packet size, queue overflow, and transport errors with a mock BSP.

`make test-map` checks cardinal-direction plotting, scale, initial partial
rotation handling, missing/out-of-range returns, scan replacement, stale
scan indication, and the refresh limit using known points and a mock clock.

With the recorded capture and reference CSV in `output/`, `make test-replay`
compares every C measurement against the earlier reference. The replay uses
73-byte reads to exercise packet boundaries independently of the hardware.
The comparison also saves all nonzero returns below 1000 mm, in acquisition
order, to `output/lidar-c-under-1m.csv`. These are individual returns, including
repeated rotations, rather than identified objects.

All build artifacts and test outputs are kept in `output/`.
