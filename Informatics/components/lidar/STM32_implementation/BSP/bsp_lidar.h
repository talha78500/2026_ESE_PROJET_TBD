#ifndef BSP_LIDAR_H
#define BSP_LIDAR_H

#include <stddef.h>
#include <stdint.h>

/* Single LiDAR serial connection. Platform-specific device selection stays
 * in the LiDAR BSP implementation. */
int bsp_lidar_init(void);
/* Returns 0 on success, -1 on error. No available data means *received == 0.
 * Never waits for bytes; the caller owns the buffer. */
int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received);
void bsp_lidar_deinit(void);

#endif
