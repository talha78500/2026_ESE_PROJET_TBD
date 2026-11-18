#ifndef CAN_DETECTOR_H
#define CAN_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>

enum { CAN_SCAN_CAPACITY = 1024 };

typedef struct {
    float right_mm;
    float forward_mm;
    float range_mm; /* Zero means missing return. */
} can_scan_point_t;

/* Takes one complete scan in acquisition order. Marks one measured return
 * per candidate in the caller's count-element array. */
void can_detector_mark(const can_scan_point_t *scan, size_t count, bool *marked);

#endif
