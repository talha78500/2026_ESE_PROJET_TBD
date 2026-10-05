#define _POSIX_C_SOURCE 200809L

#include "pc_map.h"
#include "lidar.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

enum { MAX_COLUMNS = 81, MAX_ROWS = 41, FRAME_CAPACITY = 8192 };
static const double refresh_interval_seconds = 0.25;
static const float degrees_to_radians = 0.017453292519943295f;

/* Fixed PC display storage; neither buffer is part of the portable parser. */
static char current_scan[MAX_ROWS][MAX_COLUMNS];
static char completed_scan[MAX_ROWS][MAX_COLUMNS];
static size_t current_points;
static size_t current_visible_points;
static size_t completed_points;
static size_t completed_visible_points;
static bool collecting_rotation;
static bool have_completed_rotation;
static uint32_t rotation_drop_baseline;

static int columns;
static int rows;
static int terminal_columns;
static float range_mm;
static int original_output_flags = -1;
static bool interactive_terminal;
static bool entered_screen;
static uint32_t skipped_frames;
static struct timespec last_refresh;
static struct timespec last_completed_rotation;

/* A partially written frame remains immutable until its final byte is sent. */
static char output_frame[FRAME_CAPACITY];
static size_t frame_size;
static size_t frame_offset;

static double elapsed_seconds(struct timespec now, struct timespec previous)
{
    return (double)(now.tv_sec - previous.tv_sec)
           + (double)(now.tv_nsec - previous.tv_nsec) / 1000000000.0;
}

static int configure_range(void)
{
    const char *setting = getenv("LIDAR_MAP_RANGE_M");
    char *end;
    float metres;

    range_mm = 2000.0f;
    if (setting == NULL) {
        return 0;
    }
    errno = 0;
    metres = strtof(setting, &end);
    if (end == setting || *end != '\0' || errno == ERANGE ||
        !isfinite(metres) || metres < 0.1f || metres > 20.0f) {
        fputs("LIDAR_MAP_RANGE_M must be between 0.1 and 20 metres.\n", stderr);
        return -1;
    }
    range_mm = metres * 1000.0f;
    return 0;
}

static int configure_grid(void)
{
    struct winsize dimensions;
    int half_rows = (MAX_ROWS - 1) / 2;

    terminal_columns = MAX_COLUMNS + 2;
    if (interactive_terminal && ioctl(STDOUT_FILENO, TIOCGWINSZ, &dimensions) == 0
        && dimensions.ws_col != 0 && dimensions.ws_row != 0) {
        int available_columns = (int)dimensions.ws_col - 3;
        int available_rows = (int)dimensions.ws_row - 8;

        terminal_columns = (int)dimensions.ws_col - 1;
        if (available_columns < 9 || available_rows < 5) {
            fputs("Terminal too small for the LiDAR map; enlarge it first.\n", stderr);
            return -1;
        }
        if ((available_columns - 1) / 4 < half_rows) {
            half_rows = (available_columns - 1) / 4;
        }
        if ((available_rows - 1) / 2 < half_rows) {
            half_rows = (available_rows - 1) / 2;
        }
    }
    /* Assume characters are twice as tall as wide. Equal physical ranges
     * therefore need twice as many horizontal intervals as vertical ones. */
    rows = 2 * half_rows + 1;
    columns = 4 * half_rows + 1;
    return 0;
}

static void accumulate_point(const lidar_point_t *point)
{
    float angle;
    float right;
    float forward;
    int column;
    int row;

    current_points++;
    if (point->distance_mm <= 0.0f || point->distance_mm > range_mm) {
        return;
    }
    /* X2 angles increase clockwise. Sensor 0 degrees is drawn upward;
     * this is the sensor frame, not a calibrated robot heading. */
    angle = point->angle_deg * degrees_to_radians;
    right = point->distance_mm * sinf(angle);
    forward = point->distance_mm * cosf(angle);
    column = (int)lroundf((float)(columns / 2) * (1.0f + right / range_mm));
    row = (int)lroundf((float)(rows / 2) * (1.0f - forward / range_mm));
    if (row >= 0 && row < rows && column >= 0 && column < columns) {
        current_scan[row][column] = '*';
        current_visible_points++;
    }
}

static void consume_points(struct timespec now)
{
    lidar_point_t point;
    lidar_stats_t stats = lidar_get_stats();

    for (size_t count = 0; count < LIDAR_QUEUE_CAPACITY; count++) {
        if (!lidar_get_point(&point)) {
            break;
        }
        if (point.starts_rotation) {
            /* Ignore the initial partial rotation after connecting. Only
             * publish scans bracketed by two rotation markers, without
             * parser-queue loss. Rejected packets remain visible in stats. */
            if (collecting_rotation && stats.dropped_points == rotation_drop_baseline) {
                memcpy(completed_scan, current_scan, sizeof completed_scan);
                completed_points = current_points;
                completed_visible_points = current_visible_points;
                last_completed_rotation = now;
                have_completed_rotation = true;
            }
            memset(current_scan, ' ', sizeof current_scan);
            current_points = 0;
            current_visible_points = 0;
            rotation_drop_baseline = stats.dropped_points;
            collecting_rotation = true;
        }
        if (collecting_rotation) {
            accumulate_point(&point);
        }
    }
}

static int append_text(const char *text)
{
    size_t length = strlen(text);

    if (length > sizeof output_frame - frame_size) {
        return -1;
    }
    memcpy(output_frame + frame_size, text, length);
    frame_size += length;
    return 0;
}

static int append_status_line(char *text)
{
    if (strlen(text) > (size_t)terminal_columns) {
        text[terminal_columns] = '\0';
    }
    if (append_text(text) != 0) {
        return -1;
    }
    return append_text("\n");
}

static int append_border(void)
{
    char border[MAX_COLUMNS + 4];

    border[0] = '+';
    memset(border + 1, '-', (size_t)columns);
    border[columns + 1] = '+';
    border[columns + 2] = '\n';
    border[columns + 3] = '\0';
    return append_text(border);
}

static int build_frame(struct timespec now)
{
    char line[256];
    lidar_stats_t stats = lidar_get_stats();

    frame_size = 0;
    frame_offset = 0;
    if (interactive_terminal) {
        if (!entered_screen) {
            if (append_text("\033[?1049h\033[?25l") != 0) {
                return -1;
            }
            entered_screen = true;
        }
        if (append_text("\033[H\033[2J") != 0) {
            return -1;
        }
    }
    snprintf(line, sizeof line, "LiDAR map: radius %.2f m | 0 deg up, 90 deg right",
             (double)range_mm / 1000.0);
    if (append_status_line(line) != 0) {
        return -1;
    }
    snprintf(line, sizeof line, "Scale: %.0f mm/column, %.0f mm/row",
             (double)range_mm / (columns / 2), (double)range_mm / (rows / 2));
    if (append_status_line(line) != 0) {
        return -1;
    }
    snprintf(line, sizeof line, "Points: %zu visible / %zu | drops: %u | rejected: %u",
             completed_visible_points, completed_points,
             (unsigned)stats.dropped_points, (unsigned)stats.rejected_packets);
    if (append_status_line(line) != 0) {
        return -1;
    }
    if (have_completed_rotation) {
        snprintf(line, sizeof line, "Scan age: %.2f s | skipped display frames: %u",
                 elapsed_seconds(now, last_completed_rotation), (unsigned)skipped_frames);
    } else {
        snprintf(line, sizeof line, "Waiting for a complete rotation...");
    }
    if (append_status_line(line) != 0 || append_border() != 0) {
        return -1;
    }
    for (int row = 0; row < rows; row++) {
        line[0] = '|';
        for (int column = 0; column < columns; column++) {
            char cell = have_completed_rotation ? completed_scan[row][column] : ' ';

            if (cell == ' ' && row == rows / 2) {
                cell = '-';
            }
            if (cell == ' ' && column == columns / 2) {
                cell = '|';
            }
            if (row == rows / 2 && column == columns / 2) {
                cell = 'R';
            }
            line[column + 1] = cell;
        }
        line[columns + 1] = '|';
        line[columns + 2] = '\n';
        line[columns + 3] = '\0';
        if (append_text(line) != 0) {
            return -1;
        }
    }
    if (append_border() != 0) {
        return -1;
    }
    snprintf(line, sizeof line, "R: LiDAR | *: return | Ctrl+C: stop");
    return append_status_line(line);
}

static int flush_frame(void)
{
    ssize_t written;

    if (frame_offset == frame_size) {
        return 0;
    }
    written = write(STDOUT_FILENO, output_frame + frame_offset, frame_size - frame_offset);
    if (written < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
    }
    frame_offset += (size_t)written;
    return 0;
}

int pc_map_init(void)
{
    int flags;

    if (original_output_flags >= 0) {
        return -1;
    }
    if (configure_range() != 0) {
        return -1;
    }
    interactive_terminal = isatty(STDOUT_FILENO) != 0;
    if (configure_grid() != 0) {
        return -1;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &last_refresh) != 0) {
        return -1;
    }
    flags = fcntl(STDOUT_FILENO, F_GETFL);
    if (flags < 0) {
        return -1;
    }
    if (fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        return -1;
    }
    original_output_flags = flags;
    collecting_rotation = false;
    have_completed_rotation = false;
    entered_screen = false;
    current_points = 0;
    current_visible_points = 0;
    completed_points = 0;
    completed_visible_points = 0;
    frame_size = 0;
    frame_offset = 0;
    skipped_frames = 0;
    return 0;
}

int pc_map_process(void)
{
    struct timespec now;

    if (original_output_flags < 0) {
        return -1;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return -1;
    }
    if (flush_frame() != 0) {
        return -1;
    }
    consume_points(now);
    if (elapsed_seconds(now, last_refresh) >= refresh_interval_seconds) {
        last_refresh = now;
        if (frame_offset != frame_size) {
            skipped_frames++;
        } else if (build_frame(now) != 0) {
            return -1;
        }
    }
    return flush_frame();
}

void pc_map_deinit(void)
{
    if (original_output_flags >= 0) {
        if (entered_screen) {
            /* Best-effort terminal restoration; do not wait for output. */
            static const char restore[] = "\033[?25h\033[?1049l";
            (void)write(STDOUT_FILENO, restore, sizeof restore - 1);
        }
        (void)fcntl(STDOUT_FILENO, F_SETFL, original_output_flags);
        original_output_flags = -1;
    }
}
