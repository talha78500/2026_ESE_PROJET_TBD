#ifndef LIDAR_CANS_H
#define LIDAR_CANS_H

#include "lidar.h"

enum { LIDAR_CAN_CAPACITY = 3 };

typedef struct {
    float bearing_deg; /* Sensor frame, clockwise from sensor zero, in [0, 360). */
    float distance_mm; /* Range to the nearest measured surface of this candidate. */
} lidar_can_t;

typedef enum {
    LIDAR_CANS_UNAVAILABLE,
    LIDAR_CANS_VALID,
    LIDAR_CANS_INCOMPLETE,
    LIDAR_CANS_TOO_MANY
} lidar_cans_status_t;

typedef struct {
    lidar_cans_status_t status;
    uint32_t completed_at_ms; /* Time the closing rotation marker was processed. */
    size_t count;             /* Zero for all non-valid results. */
    /* For TOO_MANY, these hold the first three groups for investigation. */
    lidar_can_t cans[LIDAR_CAN_CAPACITY];
    /* For TOO_MANY, describe the group that exceeded the list's capacity. */
    lidar_can_t overflow_candidate;
    size_t overflow_points;
} lidar_cans_result_t;

/* One application owner serializes these calls. Reset on input loss or stop. */
void lidar_cans_reset(void);
/* Feed each decoded point in acquisition order, including no-return samples. */
void lidar_cans_feed(const lidar_point_t *point, uint32_t now_ms);
/* Returns a snapshot. A result expires 500 ms after its closing marker. */
lidar_cans_result_t lidar_cans_get_result(uint32_t now_ms);

#endif
