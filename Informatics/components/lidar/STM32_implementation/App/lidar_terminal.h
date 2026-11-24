#ifndef LIDAR_TERMINAL_H
#define LIDAR_TERMINAL_H

#include "lidar_test.h"

int lidar_terminal_init(void);
/* Call after lidar_test_process() in the same application loop. */
void lidar_terminal_process(const volatile lidar_test_diagnostics_t *diagnostics);

#endif
