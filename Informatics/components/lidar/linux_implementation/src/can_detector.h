#ifndef CAN_DETECTOR_H
#define CAN_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>

enum { CAN_SCAN_CAPACITY = 1024 };

typedef struct {
    /* Cartesian position relative to the LiDAR, in the scan plane. */
    float right_mm;
    float forward_mm;
    float range_mm; /* Zero means missing return. */
} can_scan_point_t;

/* Takes one complete scan in acquisition order, including missing returns
 * so gaps remain visible. Clears the caller's count-element output array,
 * then sets entries for the measured returns selected as candidate markers. */
void can_detector_mark(const can_scan_point_t *scan, size_t count, bool *marked);

#endif
