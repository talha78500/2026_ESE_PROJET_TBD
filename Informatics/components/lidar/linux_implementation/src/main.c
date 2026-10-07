#include "bsp.h"
#include "lidar.h"
#include "pc_map.h"

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>

static volatile sig_atomic_t stop_requested;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

int main(void)
{
    int result = 0;

    if (signal(SIGINT, request_stop) == SIG_ERR) {
        fputs("Failed to register the interrupt handler.\n", stderr);
        return 1;
    }
    if (signal(SIGTERM, request_stop) == SIG_ERR) {
        fputs("Failed to register the termination handler.\n", stderr);
        return 1;
    }

    if (bsp_init() != 0) {
        fputs("Serial connection initialization failed.\n", stderr);
        return 1;
    }
    lidar_init();

    if (pc_map_init() != 0) {
        fputs("Terminal map initialization failed.\n", stderr);
        bsp_deinit();
        return 1;
    }

    while (!stop_requested) {
        if (lidar_process() != 0) {
            fputs("LiDAR reception failed.\n", stderr);
            result = 1;
            break;
        }
        if (pc_map_process() != 0) {
            fputs("Terminal map failed.\n", stderr);
            result = 1;
            break;
        }
    }

    bsp_deinit();
    pc_map_deinit();
    printf("\nReceived bytes: %" PRIu32 "\n", lidar_received_bytes());
    return result;
}
