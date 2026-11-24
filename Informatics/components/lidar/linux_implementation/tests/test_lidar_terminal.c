#include "bsp_lidar_terminal.h"
#include "lidar_cans.h"
#include "lidar_terminal.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static lidar_test_diagnostics_t diagnostics;
static bool uart_ready;
static char pending_key;
static char output[2048];
static size_t written;

int bsp_lidar_terminal_init(void)
{
    return 0;
}

bool bsp_lidar_terminal_try_read(char *byte)
{
    if (pending_key == '\0') {
        return false;
    }
    *byte = pending_key;
    pending_key = '\0';
    return true;
}

bool bsp_lidar_terminal_try_write(char byte)
{
    if (!uart_ready) {
        return false;
    }
    assert(written + 1 < sizeof output);
    output[written++] = byte;
    return true;
}

static void run_reporter(void)
{
    for (unsigned i = 0; i < 1000; ++i) {
        lidar_terminal_process(&diagnostics);
    }
}

int main(void)
{
    const char expected[] =
        "LiDAR [detailed] | scan 200 ms | valid | cans 3 | #1 0.0 deg 700 mm"
        " | #2 90.2 deg 1000 mm | #3 210.0 deg 1500 mm"
        " | rejected +0 | dropped +0 | read errors +0 | UART errors +0"
        " | DMA errors +0 | DMA overwrites +0 | DMA overwrites total 0 | motor 201 ms\r\n"
        "LiDAR [detailed] | scan 400 ms | valid | cans 0"
        " | rejected +2 | dropped +3 | read errors +1 | UART errors +4"
        " | DMA errors +5 | DMA overwrites +6 | DMA overwrites total 6 | motor 201 ms\r\n"
        "LiDAR [detailed] | scan 600 ms | too many candidates | cans 0"
        " | #1 60.0 deg 250 mm | #2 120.0 deg 250 mm | #3 180.0 deg 250 mm"
        " | #4 240.5 deg 260 mm (3 points)"
        " | rejected +0 | dropped +0 | read errors +0 | UART errors +0"
        " | DMA errors +0 | DMA overwrites +0 | DMA overwrites total 6 | motor 201 ms\r\n"
        "LiDAR [detailed] | scan 0 ms | unavailable | cans 0"
        " | rejected +0 | dropped +0 | read errors +0 | UART errors +0"
        " | DMA errors +0 | DMA overwrites +0 | DMA overwrites total 6 | motor n/a\r\n"
        "LiDAR [detailed] | scan 0 ms | unavailable | cans 0"
        " | rejected +1 | dropped +0 | read errors +0 | UART errors +0"
        " | DMA errors +0 | DMA overwrites +0 | DMA overwrites total 6 | motor n/a\r\n";

    assert(lidar_terminal_init() == 0);
    diagnostics.motor_period_ms = 201;
    diagnostics.cans = (lidar_cans_result_t){
        .status = LIDAR_CANS_VALID,
        .completed_at_ms = 200,
        .count = 3,
        .cans = {{359.99f, 699.8f}, {90.2f, 1000.0f}, {210.0f, 1500.0f}}
    };
    uart_ready = true;
    lidar_terminal_process(&diagnostics);
    assert(written == 1);

    /* Finish the first line even if a newer scan arrives during a stall. */
    uart_ready = false;
    diagnostics.cans = (lidar_cans_result_t){.status = LIDAR_CANS_VALID, .completed_at_ms = 400};
    diagnostics.parser.rejected_packets = 2;
    diagnostics.parser.dropped_points = 3;
    diagnostics.read_errors = 1;
    diagnostics.bsp.uart_errors = 4;
    diagnostics.bsp.dma_errors = 5;
    diagnostics.bsp.overwrite_events = 6;
    run_reporter();
    assert(written == 1);
    uart_ready = true;
    run_reporter();

    diagnostics.cans = (lidar_cans_result_t){
        .status = LIDAR_CANS_TOO_MANY,
        .completed_at_ms = 600,
        .cans = {{60.0f, 250.0f}, {120.0f, 250.0f}, {180.0f, 250.0f}},
        .overflow_candidate = {240.5f, 260.0f},
        .overflow_points = 3
    };
    run_reporter();
    diagnostics.cans = (lidar_cans_result_t){0};
    diagnostics.motor_period_ms = 0;
    run_reporter();
    ++diagnostics.parser.rejected_packets; /* Errors alone trigger a report. */
    run_reporter();

    assert(strcmp(output, expected) == 0);

    size_t mode_output_start = written;
    pending_key = 'c';
    run_reporter();
    diagnostics.cans = (lidar_cans_result_t){
        .status = LIDAR_CANS_VALID,
        .completed_at_ms = 800,
        .count = 1,
        .cans = {{180.0f, 249.8f}}
    };
    ++diagnostics.bsp.overwrite_events;
    lidar_terminal_process(&diagnostics); /* Start a compact line. */
    uart_ready = false;
    pending_key = 'd';
    run_reporter();
    uart_ready = true;
    run_reporter();
    assert(strcmp(output + mode_output_start,
                  "LiDAR [compact] | scan 0 ms | unavailable | cans 0 | DMA overwrites total 6\r\n"
                  "LiDAR [compact] | scan 800 ms | valid | cans 1 | #1 180.0 deg 250 mm"
                  " | DMA overwrites total 7\r\n"
                  "LiDAR [detailed] | scan 800 ms | valid | cans 1 | #1 180.0 deg 250 mm"
                  " | rejected +0 | dropped +0 | read errors +0 | UART errors +0"
                  " | DMA errors +0 | DMA overwrites +0 | DMA overwrites total 7 | motor n/a\r\n") == 0);
    puts("PASS: terminal modes, overwrite totals, error deltas, overflow, and switching during partial output.");
}
