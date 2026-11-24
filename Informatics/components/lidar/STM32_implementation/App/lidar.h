#ifndef LIDAR_H
#define LIDAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* These limits bound storage and work; they are not serial packet sizes. */
#define LIDAR_INPUT_LIMIT 256u
#define LIDAR_QUEUE_CAPACITY 512u

typedef struct {
    float angle_deg;          /* Corrected angle in [0, 360). */
    float distance_mm;        /* Zero means no return, not an obstacle at zero. */
    bool starts_rotation;     /* First sample of a CT bit-0 start packet. */
} lidar_point_t;

typedef struct {
    uint32_t received_bytes;
    uint32_t valid_packets;
    uint32_t rejected_packets;
    uint32_t decoded_points;
    uint32_t dropped_points;  /* Queue full: newest points are discarded. */
} lidar_stats_t;

/* Single instance. Calls must be serialized by the application, not from ISRs. */
void lidar_init(void);
/* One nonblocking read, at most LIDAR_INPUT_LIMIT bytes. -1 means BSP error. */
int lidar_process(void);
/* Parse supplied bytes without the BSP. Returns the number consumed, at most
 * LIDAR_INPUT_LIMIT. The caller retains and later supplies any remaining bytes.
 * NULL input consumes nothing. Partial packets persist until the next call. */
size_t lidar_feed(const uint8_t *data, size_t size);
/* Returns false when empty or point is NULL. Drain regularly to avoid overflow. */
bool lidar_get_point(lidar_point_t *point);
lidar_stats_t lidar_get_stats(void);
uint32_t lidar_received_bytes(void);

#endif
