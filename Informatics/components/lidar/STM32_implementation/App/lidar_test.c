#include "lidar_test.h"

#include "bsp_lidar.h"
#include "bsp_lidar_motor.h"
#include "stm32l4xx_hal.h"

#define LIDAR_TEST_DEFAULT_PAUSE_MS 250U
#define LIDAR_TEST_MAX_PAUSE_MS 500U
#define MOTOR_START_DUTY 350U
#define MOTOR_DUTY_STEP 5
#define MOTOR_TARGET_PERIOD_MS 200U
#define MOTOR_PERIOD_TOLERANCE_MS 3U
#define MOTOR_AVERAGE_REVOLUTIONS 5U
#define MOTOR_FEEDBACK_TIMEOUT_MS 500U

volatile lidar_test_diagnostics_t lidar_test_diagnostics;
volatile lidar_test_controls_t lidar_test_controls;

static const float zero_window_half_width_deg = 2.0f;
static uint32_t pause_started_at;
static uint32_t pause_duration_ms;
static bool restart_pending;
static bool have_rotation_marker;
static uint32_t last_rotation_ms;
static uint32_t rotation_total_ms;
static unsigned rotation_intervals;

/* Clear both the detector state and the copy shown in the debugger. */
static void invalidate_can_scan(void)
{
    lidar_cans_reset();
    lidar_test_diagnostics.cans = (lidar_cans_result_t){0};
}

static void set_motor_duty(uint16_t duty)
{
    bsp_lidar_motor_set_duty(duty);
    lidar_test_diagnostics.motor_duty_permille = duty;
}

static void reset_motor_feedback(void)
{
    have_rotation_marker = false;
    rotation_total_ms = 0;
    rotation_intervals = 0;
    lidar_test_diagnostics.motor_period_ms = 0;
}

static void record_rotation(uint32_t now_ms)
{
    uint32_t interval_ms = now_ms - last_rotation_ms;
    bool had_marker = have_rotation_marker;
    last_rotation_ms = now_ms;

    /* Ignore timing across a missing marker or markers drained in a burst. */
    if (!had_marker || interval_ms < 100U || interval_ms > 350U) {
        reset_motor_feedback();
        have_rotation_marker = true;
        return;
    }

    rotation_total_ms += interval_ms;
    if (++rotation_intervals < MOTOR_AVERAGE_REVOLUTIONS) {
        return;
    }

    uint32_t period_ms = (rotation_total_ms + MOTOR_AVERAGE_REVOLUTIONS / 2U) /
                         MOTOR_AVERAGE_REVOLUTIONS;
    lidar_test_diagnostics.motor_period_ms = period_ms;
    rotation_total_ms = 0;
    rotation_intervals = 0;

    int duty = lidar_test_diagnostics.motor_duty_permille;
    if (period_ms < MOTOR_TARGET_PERIOD_MS - MOTOR_PERIOD_TOLERANCE_MS &&
        duty >= MOTOR_DUTY_STEP) {
        set_motor_duty(duty - MOTOR_DUTY_STEP);
    } else if (period_ms > MOTOR_TARGET_PERIOD_MS + MOTOR_PERIOD_TOLERANCE_MS &&
               duty <= 1000 - MOTOR_DUTY_STEP) {
        set_motor_duty(duty + MOTOR_DUTY_STEP);
    }
}

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
    lidar_cans_reset();

    status = bsp_lidar_init();
    lidar_test_diagnostics.last_status = status;
    lidar_test_diagnostics.bsp = bsp_lidar_get_stats();
    if (status != 0 && lidar_test_diagnostics.bsp.state != BSP_LIDAR_RECOVERING) {
        return -1;
    }
    if (bsp_lidar_motor_start() != 0) {
        bsp_lidar_deinit();
        return -1;
    }
    reset_motor_feedback();
    set_motor_duty(MOTOR_START_DUTY);

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
        reset_motor_feedback();
        invalidate_can_scan();
        if ((uint32_t)(now_ms - pause_started_at) < pause_duration_ms) {
            return; /* DMA, interrupts, and BSP lifecycle continue running. */
        }
        lidar_test_diagnostics.reader_pause_active = false;
    }
    if (lidar_test_diagnostics.bsp.state != BSP_LIDAR_RUNNING) {
        reset_motor_feedback();
        invalidate_can_scan();
        lidar_test_diagnostics.has_point = false;
        lidar_test_diagnostics.zero_angle.has_point = false;
        lidar_test_diagnostics.last_status = -1;
        return;
    }

    /* The parser calls the BSP reader once, consuming at most 256 bytes. */
    uint32_t parse_started_ms = HAL_GetTick();
    status = lidar_process();
    uint32_t parse_ms = HAL_GetTick() - parse_started_ms;
    if (parse_ms > lidar_test_diagnostics.max_parse_ms) {
        lidar_test_diagnostics.max_parse_ms = parse_ms;
    }
    lidar_test_diagnostics.last_status = status;
    if (status != 0) {
        reset_motor_feedback();
        lidar_test_diagnostics.read_errors++;
        lidar_test_diagnostics.has_point = false;
        lidar_test_diagnostics.zero_angle.has_point = false;
    }

    if (have_rotation_marker &&
        now_ms - last_rotation_ms >= MOTOR_FEEDBACK_TIMEOUT_MS) {
        reset_motor_feedback();
        set_motor_duty(MOTOR_START_DUTY);
    }
    lidar_stats_t parser_stats = lidar_get_stats();
    bool timing_valid = status == 0 &&
                        parser_stats.dropped_points == lidar_test_diagnostics.parser.dropped_points &&
                        parser_stats.rejected_packets == lidar_test_diagnostics.parser.rejected_packets;
    if (!timing_valid) {
        reset_motor_feedback();
        invalidate_can_scan();
    }

    /* Drain at most the fixed queue capacity; no parsing runs in an ISR. */
    for (size_t count = 0; count < LIDAR_QUEUE_CAPACITY; count++) {
        if (!lidar_get_point(&point)) {
            break;
        }
        if (timing_valid) {
            if (point.starts_rotation) {
                uint32_t can_scan_started_ms = HAL_GetTick();
                lidar_cans_feed(&point, now_ms);
                uint32_t can_scan_ms = HAL_GetTick() - can_scan_started_ms;
                if (can_scan_ms > lidar_test_diagnostics.max_can_scan_ms) {
                    lidar_test_diagnostics.max_can_scan_ms = can_scan_ms;
                }
                record_rotation(now_ms);
            } else {
                lidar_cans_feed(&point, now_ms);
            }
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

    lidar_test_diagnostics.parser = parser_stats;
    lidar_test_diagnostics.bsp = bsp_lidar_get_stats();
    if (lidar_test_diagnostics.bsp.state == BSP_LIDAR_RUNNING) {
        lidar_test_diagnostics.cans = lidar_cans_get_result(now_ms);
    } else {
        invalidate_can_scan();
    }
}
