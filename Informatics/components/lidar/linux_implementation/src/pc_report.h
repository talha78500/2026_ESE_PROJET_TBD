#ifndef PC_REPORT_H
#define PC_REPORT_H

#include <stdint.h>

/* Linux terminal diagnostics, separate from portable LiDAR processing. */
int pc_report_init(void);
/* Drain up to one queue's worth of points and attempt nonblocking output.
 * Report at most every 250 ms, selecting the nearest valid point in that
 * interval. Zero-distance returns and distances >= 1000 mm are omitted. */
int pc_report_process(void);
uint32_t pc_report_dropped_messages(void);
void pc_report_deinit(void);

#endif
