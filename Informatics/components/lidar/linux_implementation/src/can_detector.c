#include "can_detector.h"

#include <math.h>
#include <string.h>

enum { WALL_POINTS_PER_SIDE = 5, MAX_CANDIDATE_POINTS = 48 };
static const float max_range_mm = 2500.0f;
static const float max_width_mm = 70.0f; /* 56 mm can plus measurement allowance. */
static const float min_protrusion_mm = 25.0f;
static const float wall_tolerance_mm = 18.0f;
static const float min_wall_baseline_mm = 50.0f;
static const float min_background_gap_mm = 50.0f;

typedef struct {
    float right;
    float forward;
} vector_t;

typedef struct {
    vector_t normal; /* Unit normal pointing from the sensor toward the wall. */
    float offset_mm;
} wall_t;

static float projection(wall_t wall, const can_scan_point_t *point)
{
    return wall.normal.right * point->right_mm + wall.normal.forward * point->forward_mm;
}

static bool find_wall(const can_scan_point_t *scan, size_t count,
                      size_t before, size_t after, wall_t *wall)
{
    const size_t starts[] = {before, after};
    vector_t centers[2] = {0};

    for (size_t side = 0; side < 2; ++side) {
        for (size_t i = 0; i < WALL_POINTS_PER_SIDE; ++i) {
            const can_scan_point_t *point = &scan[(starts[side] + i) % count];
            if (point->range_mm <= 0.0f) {
                return false;
            }
            centers[side].right += point->right_mm / WALL_POINTS_PER_SIDE;
            centers[side].forward += point->forward_mm / WALL_POINTS_PER_SIDE;
        }
    }

    /* Join the two flank centroids and verify that both flanks fit the line. */
    float right = centers[1].right - centers[0].right;
    float forward = centers[1].forward - centers[0].forward;
    float length = hypotf(right, forward);
    if (length < min_wall_baseline_mm) {
        return false;
    }
    wall->normal = (vector_t){forward / length, -right / length};
    wall->offset_mm = wall->normal.right * centers[0].right +
                      wall->normal.forward * centers[0].forward;
    if (wall->offset_mm < 0.0f) {
        wall->normal.right = -wall->normal.right;
        wall->normal.forward = -wall->normal.forward;
        wall->offset_mm = -wall->offset_mm;
    }

    for (size_t side = 0; side < 2; ++side) {
        for (size_t i = 0; i < WALL_POINTS_PER_SIDE; ++i) {
            const can_scan_point_t *point = &scan[(starts[side] + i) % count];
            if (fabsf(wall->offset_mm - projection(*wall, point)) > wall_tolerance_mm) {
                return false;
            }
        }
    }
    return true;
}

static bool has_background(const can_scan_point_t *edge, const can_scan_point_t *neighbor)
{
    return neighbor->range_mm <= 0.0f ||
           neighbor->range_mm - edge->range_mm >= min_background_gap_mm;
}

static bool exceeds_can_width(const can_scan_point_t *start, const can_scan_point_t *point)
{
    float right = point->right_mm - start->right_mm;
    float forward = point->forward_mm - start->forward_mm;
    return right * right + forward * forward > max_width_mm * max_width_mm;
}

static void mark_isolated(const can_scan_point_t *scan, size_t count, bool *marked)
{
    for (size_t first = 0; first < count; ++first) {
        size_t before = (first + count - 1) % count;
        size_t nearest = first;

        if (scan[first].range_mm <= 0.0f ||
            !has_background(&scan[first], &scan[before])) {
            continue;
        }
        for (size_t length = 1; length <= MAX_CANDIDATE_POINTS && length + 1 < count;
             ++length) {
            size_t last = (first + length - 1) % count;
            size_t after = (last + 1) % count;
            const can_scan_point_t *point = &scan[last];

            if (point->range_mm <= 0.0f || point->range_mm > max_range_mm ||
                exceeds_can_width(&scan[first], point) ||
                (length > 1 && has_background(point, &scan[(last + count - 1) % count]))) {
                break;
            }
            if (point->range_mm < scan[nearest].range_mm) {
                nearest = last;
            }
            if (!has_background(point, &scan[after])) {
                continue;
            }
            /* Require two measured hits for a free-standing candidate. */
            if (length >= 2) {
                marked[nearest] = true;
            }
            break;
        }
    }
}

static void mark_wall_protrusions(const can_scan_point_t *scan, size_t count, bool *marked)
{
    for (size_t first = 0; first < count; ++first) {
        size_t before = (first + count - WALL_POINTS_PER_SIDE) % count;
        size_t nearest = first;

        /* Try short runs of consecutive returns, including runs across the
         * sweep boundary. A missing return breaks the run. */
        for (size_t length = 1; length <= MAX_CANDIDATE_POINTS &&
             length + 2 * WALL_POINTS_PER_SIDE <= count; ++length) {
            size_t last = (first + length - 1) % count;
            const can_scan_point_t *point = &scan[last];
            if (point->range_mm <= 0.0f || point->range_mm > max_range_mm ||
                exceeds_can_width(&scan[first], point)) {
                break;
            }
            if (point->range_mm < scan[nearest].range_mm) {
                nearest = last;
            }

            wall_t wall;
            if (!find_wall(scan, count, before, (last + 1) % count, &wall)) {
                continue;
            }
            size_t hits = 0;
            bool already_marked = false;
            while (hits < length) {
                size_t index = (first + hits) % count;
                if (wall.offset_mm - projection(wall, &scan[index]) < min_protrusion_mm) {
                    break;
                }
                already_marked |= marked[index];
                ++hits;
            }
            if (hits == length) {
                if (!already_marked) {
                    marked[nearest] = true;
                }
                break;
            }
        }
    }
}

void can_detector_mark(const can_scan_point_t *scan, size_t count, bool *marked)
{
    memset(marked, 0, count * sizeof *marked);
    if (count > CAN_SCAN_CAPACITY) {
        return;
    }
    if (count >= 4) {
        mark_isolated(scan, count, marked);
    }
    if (count > 2 * WALL_POINTS_PER_SIDE) {
        mark_wall_protrusions(scan, count, marked);
    }
}
