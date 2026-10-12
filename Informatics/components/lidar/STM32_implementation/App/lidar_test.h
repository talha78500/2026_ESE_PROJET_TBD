#ifndef LIDAR_TEST_H
#define LIDAR_TEST_H

#include "lidar.h"

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
    lidar_stats_t parser;
    lidar_point_t latest_point;
    lidar_zero_angle_diagnostics_t zero_angle;
} lidar_test_diagnostics_t;

/* Volatile preserves diagnostic writes for debugger observation. This is
 * application-owned state, not an interrupt synchronization mechanism. */
extern volatile lidar_test_diagnostics_t lidar_test_diagnostics;

/* Call once after CubeMX peripheral initialization. */
int lidar_test_init(void);
/* One bounded reception/parser call and a bounded drain of decoded points. */
void lidar_test_process(void);

#endif
