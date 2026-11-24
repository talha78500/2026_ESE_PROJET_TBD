#include "lidar_cans.h"

#include <math.h>

enum { SCAN_POINT_CAPACITY = 1024, SCAN_TIMEOUT_MS = 500, MAX_MISSED_RETURNS = 6 };

/* Starting values for a 56 mm can and a scan plane above the border walls. */
static const float max_range_mm = 400.0f;
static const float max_width_mm = 70.0f;
static const float range_gap_mm = 50.0f;
static const float max_neighbor_angle_deg = 3.0f;
static const float degrees_to_radians = 0.01745329252f;
static const float max_gap_angle_deg = 6.0f;
static const float max_gap_depth_mm = 20.0f;
static const float max_gap_width_mm = 25.0f;

typedef struct {
    float angle_deg;
    float distance_mm;
} scan_return_t;

static scan_return_t scan[SCAN_POINT_CAPACITY];
static size_t point_count;
static uint32_t started_at_ms;
static bool collecting;
static bool scan_overflow;
static lidar_cans_result_t latest_result;

/* Take the short way around the circle. Going from 359 to 1 is a +2 degree step. */
static float angle_step(float from_deg, float to_deg)
{
    float step = to_deg - from_deg;
    if (step > 180.0f) {
        step -= 360.0f;
    } else if (step < -180.0f) {
        step += 360.0f;
    }
    return step;
}

/* Keep neighboring hits together while their angles and ranges stay close. */
static bool same_group(scan_return_t a, scan_return_t b)
{
    return a.distance_mm > 0.0f && b.distance_mm > 0.0f &&
           fabsf(a.distance_mm - b.distance_mm) < range_gap_mm &&
           fabsf(angle_step(a.angle_deg, b.angle_deg)) <= max_neighbor_angle_deg;
}

/* A narrow blind spot can span more samples at a lower motor speed.
 * Bound the lookahead and use the measured endpoints to check the gap. */
static size_t neighbor_in_group(size_t index, size_t limit, bool backward)
{
    scan_return_t edge = scan[index];
    for (size_t step = 1; step <= limit && step <= MAX_MISSED_RETURNS + 1U; ++step) {
        size_t neighbor_index = (index + (backward ? point_count - step : step)) % point_count;
        scan_return_t neighbor = scan[neighbor_index];
        if (neighbor.distance_mm <= 0.0f) {
            continue;
        }
        if (step == 1) {
            return same_group(edge, neighbor) ? step : 0;
        }

        /* Missing returns have uncorrected angles. Measure the gap using
         * the two real hits and keep every missing sample out of the fit. */
        float angle_deg = fabsf(angle_step(edge.angle_deg, neighbor.angle_deg));
        float width_mm = fmaxf(edge.distance_mm, neighbor.distance_mm) *
                         angle_deg * degrees_to_radians;
        if (angle_deg <= max_gap_angle_deg && width_mm <= max_gap_width_mm &&
            fabsf(edge.distance_mm - neighbor.distance_mm) <= max_gap_depth_mm) {
            return step;
        }
        return 0;
    }
    return 0;
}

/* Look just outside the group for a missing return, a jump farther away,
 * or a gap in the scan angles. Check both edges before reporting a can. */
static bool background_at_edge(scan_return_t edge, scan_return_t neighbor)
{
    return neighbor.distance_mm <= 0.0f ||
           neighbor.distance_mm - edge.distance_mm >= range_gap_mm ||
           fabsf(angle_step(edge.angle_deg, neighbor.angle_deg)) > max_neighbor_angle_deg;
}

/* Find can-sized groups in the completed rotation and publish the new list. */
static void finish_scan(uint32_t now_ms)
{
    lidar_cans_result_t result = {
        .status = LIDAR_CANS_VALID,
        .completed_at_ms = now_ms
    };
    if (scan_overflow || now_ms - started_at_ms >= SCAN_TIMEOUT_MS) {
        result.status = LIDAR_CANS_INCOMPLETE;
        latest_result = result;
        return;
    }

    /* Start at a break so a can straddling the rotation marker stays in
     * one group. With no break the sweep contains no isolated group. */
    size_t first = 0;
    while (first < point_count &&
           (scan[first].distance_mm <= 0.0f ||
            neighbor_in_group(first, point_count - 1, true) != 0)) {
        ++first;
    }

    for (size_t visited = 0; first < point_count && visited < point_count;) {
        size_t start = (first + visited) % point_count;
        size_t last = start;
        if (scan[start].distance_mm <= 0.0f) {
            ++visited;
            continue;
        }
        size_t hits = 1;
        size_t span = 1;
        scan_return_t point = scan[last];
        float nearest_mm = point.distance_mm;
        float farthest_mm = point.distance_mm;
        float offset_deg = 0.0f;
        float min_offset_deg = 0.0f;
        float max_offset_deg = 0.0f;

        while (visited + span < point_count) {
            size_t step = neighbor_in_group(last, point_count - visited - span, false);
            if (step == 0) {
                break;
            }
            size_t next = (last + step) % point_count;
            scan_return_t neighbor = scan[next];
            /* Unwrap the angles locally so 359 degrees and 0 degrees
             * describe adjacent rays even with small angle corrections. */
            offset_deg += angle_step(point.angle_deg, neighbor.angle_deg);
            min_offset_deg = fminf(min_offset_deg, offset_deg);
            max_offset_deg = fmaxf(max_offset_deg, offset_deg);
            nearest_mm = fminf(nearest_mm, neighbor.distance_mm);
            farthest_mm = fmaxf(farthest_mm, neighbor.distance_mm);
            point = neighbor;
            last = next;
            ++hits;
            span += step;
        }
        visited += span;

        /* Approximate transverse width at the nearest measured surface.
         * Also reject groups with more depth than a can can account for. */
        float width_mm = nearest_mm * (max_offset_deg - min_offset_deg) * degrees_to_radians;
        if (hits < 2 || nearest_mm > max_range_mm ||
            width_mm > max_width_mm || farthest_mm - nearest_mm > max_width_mm ||
            !background_at_edge(scan[start], scan[(start + point_count - 1) % point_count]) ||
            !background_at_edge(scan[last], scan[(last + 1) % point_count])) {
            continue;
        }
        /* Use the middle of the observed angles and the closest measured range. */
        float bearing = scan[start].angle_deg + (min_offset_deg + max_offset_deg) * 0.5f;
        if (bearing < 0.0f) {
            bearing += 360.0f;
        } else if (bearing >= 360.0f) {
            bearing -= 360.0f;
        }
        lidar_can_t candidate = {bearing, nearest_mm};
        /* Keep the extra group for debugging. An ambiguous scan has no
         * usable can list for the motion controller. */
        if (result.count == LIDAR_CAN_CAPACITY) {
            result.status = LIDAR_CANS_TOO_MANY;
            result.count = 0;
            result.overflow_candidate = candidate;
            result.overflow_points = hits;
            break;
        }
        result.cans[result.count++] = candidate;
    }
    latest_result = result;
}

/* Drop the partial rotation and the last result when reception is interrupted. */
void lidar_cans_reset(void)
{
    collecting = false;
    point_count = 0;
    scan_overflow = false;
    latest_result = (lidar_cans_result_t){0};
}

/* The first rotation marker starts collection. Each later marker closes the
 * previous scan and is also the first point of the next one. */
void lidar_cans_feed(const lidar_point_t *point, uint32_t now_ms)
{
    if (point->starts_rotation) {
        if (collecting) {
            finish_scan(now_ms);
        }
        collecting = true;
        point_count = 0;
        scan_overflow = false;
        started_at_ms = now_ms;
    }
    if (!collecting || scan_overflow) {
        return;
    }
    if (point_count == SCAN_POINT_CAPACITY) {
        scan_overflow = true;
        return;
    }
    scan[point_count++] = (scan_return_t){point->angle_deg, point->distance_mm};
}

/* Keep the last completed result available while the next scan is collected.
 * Expire it if the sensor stops delivering complete rotations. */
lidar_cans_result_t lidar_cans_get_result(uint32_t now_ms)
{
    if (now_ms - latest_result.completed_at_ms >= SCAN_TIMEOUT_MS) {
        latest_result = (lidar_cans_result_t){0};
    }
    return latest_result;
}
