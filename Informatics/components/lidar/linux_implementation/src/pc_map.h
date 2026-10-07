#ifndef PC_MAP_H
#define PC_MAP_H

/* PC-only terminal display. Reads decoded points through the public LiDAR API.
 * LIDAR_MAP_RANGE_M selects the radius; the default is 2 metres. */
int pc_map_init(void);
/* Bounded queue consumption and nonblocking output, at most 4 frames/second. */
int pc_map_process(void);
void pc_map_deinit(void);

#endif
