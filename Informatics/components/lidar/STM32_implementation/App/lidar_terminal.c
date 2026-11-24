#include "lidar_terminal.h"

#include "bsp_lidar_terminal.h"
#include "lidar_cans.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    uint32_t rejected;
    uint32_t dropped;
    uint32_t read_errors;
    uint32_t uart_errors;
    uint32_t dma_errors;
    uint32_t overwrites;
} error_counts_t;

static char line[512];
static size_t line_size;
static size_t next_byte;
static bool have_report;
static bool detailed = true;
static bool reported_detailed;
static lidar_cans_status_t reported_status;
static uint32_t reported_scan_ms;
static error_counts_t reported_errors;

static int format_candidate(char *buffer, size_t capacity, unsigned number, lidar_can_t can)
{
    /* Print tenths of a degree without requiring float printf support. */
    unsigned bearing = (unsigned)lroundf(can.bearing_deg * 10.0f) % 3600U;
    unsigned distance = (unsigned)lroundf(can.distance_mm);
    return snprintf(buffer, capacity, " | #%u %u.%u deg %u mm",
                    number, bearing / 10U, bearing % 10U, distance);
}

/* Format one snapshot. Keep the line intact until every byte has been sent. */
static size_t format_report(lidar_cans_result_t result, error_counts_t errors,
                            const volatile lidar_test_diagnostics_t *diagnostics)
{
    bsp_lidar_stats_t bsp = diagnostics->bsp;
    const char *status;
    switch (result.status) {
    case LIDAR_CANS_VALID:
        status = "valid";
        break;
    case LIDAR_CANS_INCOMPLETE:
        status = "incomplete";
        break;
    case LIDAR_CANS_TOO_MANY:
        status = "too many candidates";
        break;
    default:
        status = "unavailable";
        break;
    }

    int length = snprintf(line, sizeof line,
                          "LiDAR [%s] | scan %" PRIu32 " ms | %s | cans %u",
                          detailed ? "detailed" : "compact",
                          result.completed_at_ms, status, (unsigned)result.count);
    if (length < 0 || (size_t)length >= sizeof line) {
        return 0;
    }
    size_t used = (size_t)length;
    size_t reported_count = result.count;
    if (detailed && result.status == LIDAR_CANS_TOO_MANY) {
        reported_count = LIDAR_CAN_CAPACITY;
    }
    for (size_t i = 0; i < reported_count; ++i) {
        length = format_candidate(line + used, sizeof line - used, (unsigned)i + 1U,
                                  result.cans[i]);
        if (length < 0 || (size_t)length >= sizeof line - used) {
            return 0;
        }
        used += (size_t)length;
    }
    if (!detailed) {
        length = snprintf(line + used, sizeof line - used,
                          " | DMA overwrites total %" PRIu32 "\r\n", bsp.overwrite_events);
        return length < 0 || (size_t)length >= sizeof line - used ? 0 : used + (size_t)length;
    }

    if (result.status == LIDAR_CANS_TOO_MANY) {
        length = format_candidate(line + used, sizeof line - used, LIDAR_CAN_CAPACITY + 1U,
                                  result.overflow_candidate);
        if (length < 0 || (size_t)length >= sizeof line - used) {
            return 0;
        }
        used += (size_t)length;
        length = snprintf(line + used, sizeof line - used, " (%u points)",
                          (unsigned)result.overflow_points);
        if (length < 0 || (size_t)length >= sizeof line - used) {
            return 0;
        }
        used += (size_t)length;
    }

    char motor[16];
    uint16_t motor_period_ms = diagnostics->motor_period_ms;
    if (motor_period_ms == 0) {
        snprintf(motor, sizeof motor, "n/a");
    } else {
        snprintf(motor, sizeof motor, "%u ms", (unsigned)motor_period_ms);
    }
    length = snprintf(line + used, sizeof line - used,
                      " | rejected +%" PRIu32 " | dropped +%" PRIu32
                      " | read errors +%" PRIu32 " | UART errors +%" PRIu32
                      " | DMA errors +%" PRIu32 " | DMA overwrites +%" PRIu32
                      " | DMA overwrites total %" PRIu32 " | motor %s",
                      errors.rejected, errors.dropped, errors.read_errors,
                      errors.uart_errors, errors.dma_errors, errors.overwrites,
                      bsp.overwrite_events, motor);
    if (length < 0 || (size_t)length >= sizeof line - used) {
        return 0;
    }
    used += (size_t)length;
    if (bsp.last_overwrite_unread_bytes != 0) {
        length = snprintf(line + used, sizeof line - used,
                          " | read gap max %" PRIu32 " ms | overwrite gap %" PRIu32
                          " ms | unread %" PRIu32 " bytes | parse max %" PRIu32
                          " ms | can scan max %" PRIu32 " ms",
                          bsp.max_read_gap_ms, bsp.last_overwrite_read_gap_ms,
                          bsp.last_overwrite_unread_bytes, diagnostics->max_parse_ms,
                          diagnostics->max_can_scan_ms);
        if (length < 0 || (size_t)length >= sizeof line - used) {
            return 0;
        }
        used += (size_t)length;
    }
    length = snprintf(line + used, sizeof line - used, "\r\n");
    return length < 0 || (size_t)length >= sizeof line - used ? 0 : used + (size_t)length;
}

int lidar_terminal_init(void)
{
    return bsp_lidar_terminal_init();
}

void lidar_terminal_process(const volatile lidar_test_diagnostics_t *diagnostics)
{
    char key;
    if (bsp_lidar_terminal_try_read(&key)) {
        if (key == 'c') {
            detailed = false;
        } else if (key == 'd') {
            detailed = true;
        }
    }

    /* A switch takes effect after the current line, so no text is mixed. */
    if (next_byte == line_size) {
        lidar_cans_result_t result = diagnostics->cans;
        error_counts_t counts = {
            .rejected = diagnostics->parser.rejected_packets,
            .dropped = diagnostics->parser.dropped_points,
            .read_errors = diagnostics->read_errors,
            .uart_errors = diagnostics->bsp.uart_errors,
            .dma_errors = diagnostics->bsp.dma_errors,
            .overwrites = diagnostics->bsp.overwrite_events
        };
        /* Count errors since the last queued report, including errors
         * that arrived while its bytes were still going out. */
        error_counts_t delta = {
            counts.rejected - reported_errors.rejected,
            counts.dropped - reported_errors.dropped,
            counts.read_errors - reported_errors.read_errors,
            counts.uart_errors - reported_errors.uart_errors,
            counts.dma_errors - reported_errors.dma_errors,
            counts.overwrites - reported_errors.overwrites
        };
        bool new_errors = delta.rejected || delta.dropped || delta.read_errors ||
                          delta.uart_errors || delta.dma_errors || delta.overwrites;
        if (have_report && result.status == reported_status &&
            result.completed_at_ms == reported_scan_ms &&
            detailed == reported_detailed && !new_errors) {
            return;
        }
        line_size = format_report(result, delta, diagnostics);
        next_byte = 0;
        reported_status = result.status;
        reported_scan_ms = result.completed_at_ms;
        reported_errors = counts;
        reported_detailed = detailed;
        have_report = true;
    }

    /* Try one byte per loop. An occupied UART leaves the byte pending. */
    if (next_byte < line_size && bsp_lidar_terminal_try_write(line[next_byte])) {
        ++next_byte;
    }
}
