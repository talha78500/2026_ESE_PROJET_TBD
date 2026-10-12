#include "lidar_test.h"

#include "bsp_lidar.h"

volatile lidar_test_diagnostics_t lidar_test_diagnostics;

static const float zero_window_half_width_deg = 2.0f;

int lidar_test_init(void)
{
    lidar_test_diagnostics_t initial = {0};
    int status;

    lidar_test_diagnostics = initial;
    lidar_init();

    status = bsp_lidar_init();
    lidar_test_diagnostics.last_status = status;
    if (status != 0) {
        return -1;
    }

    lidar_test_diagnostics.initialized = true;
    return 0;
}

void lidar_test_process(void)
{
    lidar_point_t point;
    int status;

    if (!lidar_test_diagnostics.initialized) {
        return;
    }

    /* The parser calls the BSP reader once, consuming at most 256 bytes. */
    status = lidar_process();
    lidar_test_diagnostics.last_status = status;
    lidar_test_diagnostics.process_calls++;
    if (status != 0) {
        lidar_test_diagnostics.read_errors++;
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
}
