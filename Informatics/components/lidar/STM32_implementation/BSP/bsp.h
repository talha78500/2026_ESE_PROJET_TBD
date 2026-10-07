#ifndef BSP_H
#define BSP_H

#include <stddef.h>
#include <stdint.h>

/* Single serial connection. Platform-specific device selection stays in bsp.c. */
int bsp_init(void);
/* Returns 0 on success, -1 on error. No available data means *received == 0.
 * Never waits for bytes; the caller owns the buffer. */
int bsp_read(uint8_t *buffer, size_t capacity, size_t *received);
void bsp_deinit(void);

#endif
