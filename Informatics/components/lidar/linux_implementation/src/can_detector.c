#include "can_detector.h"

#include <math.h>
#include <string.h>

/* Bench-test thresholds for a 56 mm can. The detector uses the scan's
 * acquisition order, with the last and first samples treated as neighbors. */
enum {
    WALL_POINTS_PER_SIDE = 5, /* Consecutive returns used on each side of the wall protrusion. */
    MAX_CANDIDATE_POINTS = 48 /* Caps the number of returns examined in one candidate group. */
};

/* Maximum LiDAR range of any return in a candidate group. */
static const float max_range_mm = 2500.0f;

/* Maximum distance from the first return in the group, allowing for a 56 mm can and measurement error. */
static const float max_width_mm = 70.0f;

/* Minimum perpendicular distance each candidate return must sit in front of the fitted wall. */
static const float min_protrusion_mm = 25.0f;

/* Maximum perpendicular distance a supporting wall return can be from the fitted line. */
static const float wall_tolerance_mm = 18.0f;

/* Minimum distance between the average wall positions on either side of the candidate. */
static const float min_wall_baseline_mm = 50.0f;

/* Minimum range increase at either edge of an isolated group. A missing return also counts as an edge. */
static const float min_background_gap_mm = 50.0f;

/* In the LiDAR scan plane, x points right and y points forward. */
typedef struct {
    float x;
    float y;
} vector_t;

typedef struct {
    vector_t normal; /* Unit normal pointing from the sensor toward the wall. */
    float offset_mm; /* Perpendicular distance from the sensor to the wall line. */
} wall_t;

/* Dot product with the unit normal gives the point's signed distance along
 * that normal, measured from the LiDAR origin in millimeters. */
static float projection(wall_t wall, const can_scan_point_t *point)
{
    return wall.normal.x * point->right_mm + wall.normal.y * point->forward_mm;
}

/* 'before' and 'after' are the first sample indices of the two wall groups
 * bracketing the current candidate, with the candidate itself excluded. */
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
            centers[side].x += point->right_mm / WALL_POINTS_PER_SIDE;
            centers[side].y += point->forward_mm / WALL_POINTS_PER_SIDE;
        }
    }

    /* The vector between the two groups' average positions runs along the
     * wall. A short baseline makes its direction sensitive to scan noise. */
    float dx = centers[1].x - centers[0].x;
    float dy = centers[1].y - centers[0].y;
    float length = hypotf(dx, dy);
    if (length < min_wall_baseline_mm) {
        return false;
    }
    /* Rotate the wall direction by 90 degrees and normalize it. Either
     * normal describes the same line. Choose the sign so a point between
     * the sensor and the wall has a positive offset minus projection. */
    wall->normal = (vector_t){.x = dy / length, .y = -dx / length};
    wall->offset_mm = wall->normal.x * centers[0].x + wall->normal.y * centers[0].y;
    if (wall->offset_mm < 0.0f) {
        wall->normal.x = -wall->normal.x;
        wall->normal.y = -wall->normal.y;
        wall->offset_mm = -wall->offset_mm;
    }

    /* Check every supporting point so a line through the averages of two
     * different surfaces cannot pass on the averages alone. */
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

/* A gap or a jump to a farther surface can bound a foreground object. The
 * subtraction is directional, so a nearer neighbor does not count. */
static bool has_background(const can_scan_point_t *edge, const can_scan_point_t *neighbor)
{
    return neighbor->range_mm <= 0.0f ||
           neighbor->range_mm - edge->range_mm >= min_background_gap_mm;
}

static bool exceeds_can_width(const can_scan_point_t *start, const can_scan_point_t *point)
{
    /* The squared distance from the first hit gives a size check in the
     * scan plane without taking a square root for every return. */
    float right = point->right_mm - start->right_mm;
    float forward = point->forward_mm - start->forward_mm;
    return right * right + forward * forward > max_width_mm * max_width_mm;
}

static void mark_isolated(const can_scan_point_t *scan, size_t count, bool *marked)
{
    for (size_t first = 0; first < count; ++first) {
        /* Adding count before subtracting wraps index zero to the last
         * sample without underflowing the unsigned index. */
        size_t before = (first + count - 1) % count;
        size_t nearest = first;

        if (scan[first].range_mm <= 0.0f ||
            !has_background(&scan[first], &scan[before])) {
            continue;
        }
        /* Leave room for a distinct neighboring sample on each side.
         * End the run when a farther surface or a gap appears after it. */
        for (size_t length = 1; length <= MAX_CANDIDATE_POINTS && length + 1 < count;
             ++length) {
            size_t last = (first + length - 1) % count;
            size_t after = (last + 1) % count;
            const can_scan_point_t *point = &scan[last];

            /* A jump inward from the previous return starts a different
             * foreground group. It can be tried from its own first sample. */
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
            /* Two hits give a size sample and reject solitary returns.
             * The marker points to the nearest measured hit on the object. */
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

        /* Try short runs across the sweep boundary as needed. Leave room
         * for both wall groups so no sample is reused as wall evidence. */
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

            /* A longer run moves the right wall group outward. Try it
             * when these immediate neighboring samples fail the wall fit. */
            wall_t wall;
            if (!find_wall(scan, count, before, (last + 1) % count, &wall)) {
                continue;
            }
            size_t hits = 0;
            bool already_marked = false;
            while (hits < length) {
                size_t index = (first + hits) % count;
                /* With a unit normal, this difference is the perpendicular
                 * distance toward the sensor from the wall line. */
                if (wall.offset_mm - projection(wall, &scan[index]) < min_protrusion_mm) {
                    break;
                }
                already_marked |= marked[index];
                ++hits;
            }
            if (hits == length) {
                /* Reuse a marker already inside this run so both passes
                 * can agree on the object without marking it twice here. */
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
    /* The isolated pass needs two hits and two boundary samples. The
     * wall pass can use one hit when both wall groups support it. */
    if (count >= 4) {
        mark_isolated(scan, count, marked);
    }
    if (count > 2 * WALL_POINTS_PER_SIDE) {
        mark_wall_protrusions(scan, count, marked);
    }
}
