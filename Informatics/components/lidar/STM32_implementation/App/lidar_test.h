#ifndef LIDAR_TEST_H
#define LIDAR_TEST_H

#include "lidar.h"
#include "bsp_lidar.h"

typedef enum {
    LIDAR_TEST_COMMAND_NONE,
    LIDAR_TEST_COMMAND_PAUSE_READER,
    LIDAR_TEST_COMMAND_STOP,
    LIDAR_TEST_COMMAND_START,
    LIDAR_TEST_COMMAND_RESTART
} lidar_test_command_t;

typedef struct {
    lidar_test_command_t command;
    uint32_t reader_pause_ms;
} lidar_test_controls_t;

/* Debugger-written inputs: set parameters first and command last. Volatile
 * keeps application reads observable; this is not a thread-safe command API. */
extern volatile lidar_test_controls_t lidar_test_controls;

/* The latest sample in the +/-2 degree window around sensor zero. */
typedef struct {
    bool has_point;
    bool has_return;          /* Latest matching distance is nonzero. */
    uint32_t matched_points;  /* Includes matching zero-distance samples. */
    lidar_point_t latest_point;
} lidar_zero_angle_diagnostics_t;

/* STM32 parser-integration diagnostics, inspected through the debugger. */
typedef struct {
    bool initialized;
    bool has_point;
    int last_status;
    uint32_t process_calls;
    uint32_t read_errors;
    uint32_t consumed_points;
    bool reader_pause_active;
    int command_status;
    bsp_lidar_stats_t bsp;
    lidar_stats_t parser;
    lidar_point_t latest_point;
    lidar_zero_angle_diagnostics_t zero_angle;
    uint16_t motor_duty_permille;
    uint16_t motor_period_ms;  /* Average of five revolutions, zero when unavailable. */
} lidar_test_diagnostics_t;

/* Volatile preserves diagnostic writes for debugger observation. This is
 * application-owned state, not an interrupt synchronization mechanism. */
extern volatile lidar_test_diagnostics_t lidar_test_diagnostics;

/* Call once after CubeMX peripheral initialization. A transient start failure
 * is accepted for background recovery; inspect diagnostics.bsp.state for
 * actual reception readiness. Invalid configuration still returns -1. */
int lidar_test_init(void);
/* Pass a monotonic millisecond timestamp (HAL_GetTick() in main). Performs
 * one lifecycle step, one bounded parser call when running/not paused,
 * and a bounded drain of decoded points. */
void lidar_test_process(uint32_t now_ms);

#endif
