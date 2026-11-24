#ifndef BSP_LIDAR_TERMINAL_H
#define BSP_LIDAR_TERMINAL_H

#include <stdbool.h>

int bsp_lidar_terminal_init(void);
/* Returns false if no terminal key is ready. */
bool bsp_lidar_terminal_try_read(char *byte);
/* Returns false if the UART cannot accept the byte yet. */
bool bsp_lidar_terminal_try_write(char byte);

#endif
