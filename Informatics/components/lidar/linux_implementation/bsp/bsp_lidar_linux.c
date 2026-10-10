#define _POSIX_C_SOURCE 200809L

#include "bsp_lidar.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifndef BSP_SERIAL_DEVICE
#define BSP_SERIAL_DEVICE "/dev/ttyUSB0"
#endif

static int serial_fd = -1;

int bsp_lidar_init(void)
{
    struct termios settings;
    int fd;

    if (serial_fd >= 0) {
        return -1;
    }
    fd = open(BSP_SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    if (tcgetattr(fd, &settings) < 0) {
        close(fd);
        return -1;
    }

    /* Raw binary reception, 115200 baud, 8N1, no flow control.
     * Do not toggle adapter modem lines: these may control the motor. */
    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;
    settings.c_cflag = CS8 | CREAD | CLOCAL;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B115200) < 0 ||
        cfsetospeed(&settings, B115200) < 0 ||
        tcsetattr(fd, TCSANOW, &settings) < 0) {
        close(fd);
        return -1;
    }
    serial_fd = fd;
    return 0;
}

int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received)
{
    ssize_t count;

    if (received == NULL) {
        return -1;
    }
    *received = 0;
    if (serial_fd < 0 || buffer == NULL || capacity == 0) {
        return -1;
    }
    count = read(serial_fd, buffer, capacity);
    if (count < 0) {
        /* No retry loop: interrupted reads return control to the scheduler. */
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                   ? 0 : -1;
    }
    *received = (size_t)count;
    return 0;
}

void bsp_lidar_deinit(void)
{
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}
