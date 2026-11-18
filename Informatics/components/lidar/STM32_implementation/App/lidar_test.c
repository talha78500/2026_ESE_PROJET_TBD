#include "lidar_test.h"

#include "bsp_lidar.h"

#define LIDAR_TEST_DEFAULT_PAUSE_MS 250U
#define LIDAR_TEST_MAX_PAUSE_MS 500U

volatile lidar_test_diagnostics_t lidar_test_diagnostics;
volatile lidar_test_controls_t lidar_test_controls;

static const float zero_window_half_width_deg = 2.0f;
static uint32_t pause_started_at;
static uint32_t pause_duration_ms;
static bool restart_pending;

static void process_test_command(uint32_t now_ms)
{
    lidar_test_command_t command = lidar_test_controls.command;
    uint32_t duration;

    if (command == LIDAR_TEST_COMMAND_NONE) {
        return;
    }
    lidar_test_controls.command = LIDAR_TEST_COMMAND_NONE;
    lidar_test_diagnostics.command_status = 0;

    switch (command) {
    case LIDAR_TEST_COMMAND_PAUSE_READER:
        if (bsp_lidar_get_stats().state != BSP_LIDAR_RUNNING) {
            lidar_test_diagnostics.command_status = -1;
            break;
        }
        duration = lidar_test_controls.reader_pause_ms;
        if (duration == 0) {
            duration = LIDAR_TEST_DEFAULT_PAUSE_MS;
        }
        if (duration > LIDAR_TEST_MAX_PAUSE_MS) {
            duration = LIDAR_TEST_MAX_PAUSE_MS;
        }
        pause_duration_ms = duration;
        pause_started_at = now_ms;
        lidar_test_diagnostics.reader_pause_active = true;
        break;
    case LIDAR_TEST_COMMAND_STOP:
        restart_pending = false;
        lidar_test_diagnostics.reader_pause_active = false;
        bsp_lidar_deinit();
        break;
    case LIDAR_TEST_COMMAND_START:
        lidar_test_diagnostics.command_status = bsp_lidar_init();
        break;
    case LIDAR_TEST_COMMAND_RESTART:
        restart_pending = true;
        lidar_test_diagnostics.reader_pause_active = false;
        bsp_lidar_deinit();
        break;
    default:
        lidar_test_diagnostics.command_status = -1;
        break;
    }
}

int lidar_test_init(void)
{
    lidar_test_diagnostics_t initial = {0};
    lidar_test_controls_t controls = { .reader_pause_ms = LIDAR_TEST_DEFAULT_PAUSE_MS };
    int status;

    if (lidar_test_diagnostics.initialized) {
        return -1;
    }
    lidar_test_diagnostics = initial;
    lidar_test_controls = controls;
    restart_pending = false;
    lidar_init();

    status = bsp_lidar_init();
    lidar_test_diagnostics.last_status = status;
    lidar_test_diagnostics.bsp = bsp_lidar_get_stats();
    if (status != 0 && lidar_test_diagnostics.bsp.state != BSP_LIDAR_RECOVERING) {
        return -1;
    }

    lidar_test_diagnostics.initialized = true;
    return 0;
}

void lidar_test_process(uint32_t now_ms)
{
    lidar_point_t point;
    int status;

    if (!lidar_test_diagnostics.initialized) {
        return;
    }
    lidar_test_diagnostics.process_calls++;
    process_test_command(now_ms);
    bsp_lidar_process();
    if (restart_pending && bsp_lidar_get_stats().state == BSP_LIDAR_STOPPED) {
        restart_pending = false;
        lidar_test_diagnostics.command_status = bsp_lidar_init();
    }
    lidar_test_diagnostics.bsp = bsp_lidar_get_stats();
    if (lidar_test_diagnostics.reader_pause_active) {
        if ((uint32_t)(now_ms - pause_started_at) < pause_duration_ms) {
            return; /* DMA, interrupts, and BSP lifecycle continue running. */
        }
        lidar_test_diagnostics.reader_pause_active = false;
    }
    if (lidar_test_diagnostics.bsp.state != BSP_LIDAR_RUNNING) {
        lidar_test_diagnostics.has_point = false;
        lidar_test_diagnostics.zero_angle.has_point = false;
        lidar_test_diagnostics.last_status = -1;
        return;
    }

    /* The parser calls the BSP reader once, consuming at most 256 bytes. */
    status = lidar_process();
    lidar_test_diagnostics.last_status = status;
    if (status != 0) {
        lidar_test_diagnostics.read_errors++;
        lidar_test_diagnostics.has_point = false;
        lidar_test_diagnostics.zero_angle.has_point = false;
    }

    /* Drain at most the fixed queue capacity; no parsing runs in an ISR. */
    for (size_t count = 0; count < LIDAR_QUEUE_CAPACITY; count++) {
        if (!lidar_get_point(&point)) {
            break;
        }
        lidar_test_diagnostics.latest_point = point;
        lidar_test_diagnostics.has_point = true;
        lidar_test_diagnostics.consumed_points++;

        /* Corrected angles are in [0, 360): this window crosses zero. */
        if (point.angle_deg <= zero_window_half_width_deg ||
            point.angle_deg >= 360.0f - zero_window_half_width_deg) {
            lidar_test_diagnostics.zero_angle.latest_point = point;
            lidar_test_diagnostics.zero_angle.has_point = true;
            lidar_test_diagnostics.zero_angle.has_return = point.distance_mm > 0.0f;
            lidar_test_diagnostics.zero_angle.matched_points++;
        }
    }

    lidar_test_diagnostics.parser = lidar_get_stats();
    lidar_test_diagnostics.bsp = bsp_lidar_get_stats();
}
