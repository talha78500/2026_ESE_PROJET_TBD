#define _POSIX_C_SOURCE 200809L

#include "pc_report.h"
#include "lidar.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

enum { OUTPUT_CAPACITY = 4096, MESSAGE_CAPACITY = 128 };
static const double report_interval_seconds = 0.25;

static char output_buffer[OUTPUT_CAPACITY];
static size_t output_bytes;
static int original_flags = -1;
static uint32_t dropped_messages;
static struct timespec last_report_time;
static lidar_point_t nearest_point;
static bool have_nearest_point;

static int flush_output(void)
{
    ssize_t written;

    if (output_bytes == 0) {
        return 0;
    }
    written = write(STDOUT_FILENO, output_buffer, output_bytes);
    if (written < 0) {
        /* Preserve pending output, but never wait for the terminal. */
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                   ? 0 : -1;
    }
    output_bytes -= (size_t)written;
    memmove(output_buffer, output_buffer + written, output_bytes);
    return 0;
}

static int report_point(const lidar_point_t *point)
{
    char message[MESSAGE_CAPACITY];
    int length = snprintf(message, sizeof message,
                          "Object detected at under 1 m: angle %.2f deg, distance %.1f mm\n",
                          (double)point->angle_deg, (double)point->distance_mm);

    if (length < 0 || (size_t)length >= sizeof message) {
        dropped_messages++;
        return 0;
    }
    if ((size_t)length > sizeof output_buffer - output_bytes) {
        if (flush_output() != 0) {
            return -1;
        }
    }
    if ((size_t)length > sizeof output_buffer - output_bytes) {
        dropped_messages++;
        return 0;
    }
    memcpy(output_buffer + output_bytes, message, (size_t)length);
    output_bytes += (size_t)length;
    return 0;
}

int pc_report_init(void)
{
    int flags;

    if (original_flags >= 0) {
        return -1;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &last_report_time) != 0) {
        return -1;
    }
    flags = fcntl(STDOUT_FILENO, F_GETFL);
    if (flags < 0) {
        return -1;
    }
    if (fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        return -1;
    }
    original_flags = flags;
    output_bytes = 0;
    dropped_messages = 0;
    have_nearest_point = false;
    return 0;
}

int pc_report_process(void)
{
    lidar_point_t point;
    struct timespec now;
    double elapsed;

    if (original_flags < 0 || flush_output() != 0) {
        return -1;
    }
    for (size_t count = 0; count < LIDAR_QUEUE_CAPACITY; count++) {
        if (!lidar_get_point(&point)) {
            break;
        }
        if (point.distance_mm > 0.0f && point.distance_mm < 1000.0f) {
            if (!have_nearest_point || point.distance_mm < nearest_point.distance_mm) {
                nearest_point = point;
                have_nearest_point = true;
            }
        }
    }

    /* Limit presentation, not reception. Never sleep in the processing path. */
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return -1;
    }
    elapsed = (double)(now.tv_sec - last_report_time.tv_sec)
              + (double)(now.tv_nsec - last_report_time.tv_nsec) / 1000000000.0;
    if (elapsed >= report_interval_seconds) {
        last_report_time = now;
        if (have_nearest_point) {
            if (report_point(&nearest_point) != 0) {
                return -1;
            }
            have_nearest_point = false;
        }
    }
    return flush_output();
}

uint32_t pc_report_dropped_messages(void)
{
    return dropped_messages;
}

void pc_report_deinit(void)
{
    if (original_flags >= 0) {
        /* Shutdown makes one final attempt; it does not wait to empty output. */
        (void)flush_output();
        (void)fcntl(STDOUT_FILENO, F_SETFL, original_flags);
        original_flags = -1;
        output_bytes = 0;
    }
}
