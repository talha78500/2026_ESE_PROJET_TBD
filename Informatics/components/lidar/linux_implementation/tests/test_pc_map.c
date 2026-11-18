#define _POSIX_C_SOURCE 200809L

#include "pc_map.h"
#include "lidar.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

static const lidar_point_t *input_points;
static size_t input_count;
static size_t input_offset;
static long clock_milliseconds;

/* Linked with --wrap=clock_gettime: deterministic refresh timing, no sleeps. */
int __wrap_clock_gettime(clockid_t clock_id, struct timespec *now)
{
    assert(clock_id == CLOCK_MONOTONIC);
    now->tv_sec = clock_milliseconds / 1000;
    now->tv_nsec = (clock_milliseconds % 1000) * 1000000;
    return 0;
}

bool lidar_get_point(lidar_point_t *point)
{
    if (input_offset == input_count) {
        return false;
    }
    *point = input_points[input_offset++];
    return true;
}

lidar_stats_t lidar_get_stats(void)
{
    return (lidar_stats_t){0};
}

static void supply_points(const lidar_point_t *points, size_t count)
{
    input_points = points;
    input_count = count;
    input_offset = 0;
}

static void test_static_can_detection(void)
{
    enum { SCAN_POINTS = 21 };
    const struct {
        float can_center_mm;
        bool wall_visible;
    } scenes[] = {
        {0.0f, true},
        {2500.0f, true},
        {0.0f, true},
        {2500.0f, true},
        {0.0f, true},
        {700.0f, false},
        {700.0f, true},
        {0.0f, true},
    };
    const size_t scene_count = sizeof scenes / sizeof scenes[0];
    lidar_point_t scans[sizeof scenes / sizeof scenes[0]][SCAN_POINTS];
    const float wall_mm = 2528.0f;
    const float radius_mm = 28.0f;

    for (size_t scene = 0; scene < scene_count; ++scene) {
        const float center_mm = scenes[scene].can_center_mm;
        for (int i = 0; i < SCAN_POINTS; ++i) {
            /* Sweep crosses 0 degrees at the rotation marker. */
            int beam = i <= SCAN_POINTS / 2 ? i : i - SCAN_POINTS;
            float angle_deg = 0.6f * beam;
            float angle_rad = angle_deg * 0.01745329252f;
            float range_mm = scenes[scene].wall_visible ? wall_mm / cosf(angle_rad) : 0.0f;
            float lateral_mm = center_mm * sinf(angle_rad);

            if (center_mm > 0.0f && fabsf(lateral_mm) <= radius_mm) {
                range_mm = center_mm * cosf(angle_rad) -
                           sqrtf(radius_mm * radius_mm - lateral_mm * lateral_mm);
            }
            scans[scene][i] = (lidar_point_t){
                angle_deg < 0.0f ? angle_deg + 360.0f : angle_deg,
                range_mm, i == 0
            };
        }
    }

    assert(setenv("LIDAR_MAP_RANGE_M", "3", 1) == 0);
    assert(pc_map_init() == 0);
    supply_points(&scans[0][0], 1);
    assert(pc_map_process() == 0);
    for (size_t scene = 0; scene < scene_count; ++scene) {
        lidar_point_t closing_marker = {0.0f, 0.0f, true};
        supply_points(&scans[scene][1], SCAN_POINTS - 1);
        assert(pc_map_process() == 0);
        if (scene + 1 < scene_count) {
            closing_marker = scans[scene + 1][0];
        }
        supply_points(&closing_marker, 1);
        clock_milliseconds += 250;
        assert(pc_map_process() == 0);
    }
    pc_map_deinit();
}

int main(void)
{
    const lidar_point_t initial_partial[] = {{180.0f, 300.0f, false}};
    lidar_point_t cardinal_scan[15] = {
        {0.0f, 1000.0f, true},
        {90.0f, 1000.0f, false},
        {180.0f, 1000.0f, false},
        {270.0f, 1000.0f, false},
        {45.0f, 0.0f, false},
        {45.0f, 3000.0f, false}
    };
    const lidar_point_t next_scan[] = {
        {270.0f, 300.0f, false},
        {0.0f, 1500.0f, true}
    };
    const lidar_point_t boundary_scan[] = {
        {0.0f, 2000.0f, true},
        {90.0f, 2000.0f, false},
        {180.0f, 2000.0f, false},
        {270.0f, 2000.0f, false},
        {0.0f, 1000.0f, true}
    };

    /* Fill all eight subcells of the character at (row 20, column 60).
     * The first dot is already supplied by the 90-degree cardinal point. */
    size_t index = 6;
    for (int dot_row = 0; dot_row < 4; dot_row++) {
        for (int dot_column = 0; dot_column < 2; dot_column++) {
            if (dot_row == 0 && dot_column == 0) {
                continue;
            }
            float right = 1000.0f + 25.0f * (float)dot_column;
            float forward = -25.0f * (float)dot_row;
            cardinal_scan[index++] = (lidar_point_t){
                atan2f(right, forward) * 57.29577951308232f,
                hypotf(right, forward), false
            };
        }
    }
    /* Two returns 25 mm apart vertically occupy distinct dots in one cell. */
    cardinal_scan[index++] = (lidar_point_t){0.0f, 975.0f, false};
    cardinal_scan[index++] = (lidar_point_t){0.0f, 900.0f, true};
    assert(index == sizeof cardinal_scan / sizeof cardinal_scan[0]);

    assert(pc_map_init() == 0);
    supply_points(initial_partial, 1);
    clock_milliseconds = 249;
    assert(pc_map_process() == 0); /* No frame before 250 ms. */
    clock_milliseconds = 250;
    assert(pc_map_process() == 0); /* Waiting: partial rotation was ignored. */

    supply_points(cardinal_scan, sizeof cardinal_scan / sizeof cardinal_scan[0]);
    clock_milliseconds = 500;
    assert(pc_map_process() == 0);

    supply_points(next_scan, sizeof next_scan / sizeof next_scan[0]);
    clock_milliseconds = 750;
    assert(pc_map_process() == 0);

    clock_milliseconds = 1000;
    assert(pc_map_process() == 0); /* Stale scan remains, with updated age. */

    supply_points(boundary_scan, sizeof boundary_scan / sizeof boundary_scan[0]);
    clock_milliseconds = 1250;
    assert(pc_map_process() == 0);
    pc_map_deinit();
    test_static_can_detection();
    return 0;
}
