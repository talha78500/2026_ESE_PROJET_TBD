#ifndef BSP_LIDAR_H
#define BSP_LIDAR_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_LIDAR_STOPPED,
    BSP_LIDAR_RUNNING,
    BSP_LIDAR_STOPPING,
    BSP_LIDAR_RECOVERING,
    BSP_LIDAR_FAULTED
} bsp_lidar_state_t;

/* Cumulative diagnostics since boot; UART/DMA fields apply to STM32. */
typedef struct {
    bsp_lidar_state_t state;
    uint32_t copied_bytes;
    uint32_t discarded_bytes;
    uint32_t overwrite_events;
    uint32_t uart_errors;
    uint32_t dma_errors;
    uint32_t restarts;
    uint32_t start_failures;
    uint32_t abort_failures;
    uint32_t last_uart_error;
    uint32_t last_dma_error;
    uint32_t max_read_gap_ms;
    uint32_t last_overwrite_read_gap_ms;
    uint32_t last_overwrite_unread_bytes;
} bsp_lidar_stats_t;

/* Single LiDAR serial connection. Platform-specific device selection stays
 * in the LiDAR BSP implementation. Public calls must be serialized in
 * application context. On STM32 a failed start may enter RECOVERING;
 * callers can continue process() to retry rather than waiting. */
int bsp_lidar_init(void);
/* Call regularly, including while stopped or recovering. STM32 performs
 * one nonblocking lifecycle step; Linux has no deferred work. */
void bsp_lidar_process(void);
bsp_lidar_stats_t bsp_lidar_get_stats(void);
/* Returns 0 on success, -1 on error. No available data means *received == 0.
 * Never waits for bytes; the caller owns the buffer. Only the first
 * *received bytes are valid. STM32 also returns -1 with zero received on
 * detected DMA overwrite, after discarding the unread backlog. */
int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received);
/* STM32 requests an asynchronous stop: continue bsp_lidar_process() until
 * state is STOPPED before calling init again. Stops UART reception only,
 * not LiDAR power/motor rotation. Linux closes the serial port immediately. */
void bsp_lidar_deinit(void);

#endif
