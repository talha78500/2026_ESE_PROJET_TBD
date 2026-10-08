#include "lidar.h"
#include "bsp_lidar.h"

#include <math.h>
#include <string.h>

enum {
    HEADER_SIZE = 10,
    MAX_SAMPLE_COUNT = 255,
    MAX_PACKET_SIZE = HEADER_SIZE + 2 * MAX_SAMPLE_COUNT
};

static const float radians_to_degrees = 57.29577951308232f;

static uint8_t receive_buffer[LIDAR_INPUT_LIMIT];
static uint8_t packet_buffer[MAX_PACKET_SIZE];
static size_t packet_bytes;

static lidar_point_t point_queue[LIDAR_QUEUE_CAPACITY];
static size_t queue_head;
static size_t queue_count;
static lidar_stats_t stats;

/* Decode explicitly: no alignment requirements or host-endian assumptions. */
static uint16_t read_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void discard_prefix(size_t count)
{
    packet_bytes -= count;
    memmove(packet_buffer, packet_buffer + count, packet_bytes);
}

static bool header_is_valid(void)
{
    uint8_t sample_count = packet_buffer[3];
    uint16_t first_angle = read_u16(packet_buffer + 4);
    uint16_t last_angle = read_u16(packet_buffer + 6);

    if (sample_count == 0) {
        return false;
    }
    if ((packet_buffer[2] & 1u) != 0 && sample_count != 1) {
        return false;
    }
    /* Angle bit 0 is fixed to 1; remaining bits are degrees times 64. */
    if ((first_angle & 1u) == 0 || (last_angle & 1u) == 0) {
        return false;
    }
    if ((first_angle >> 1) >= 360u * 64u ||
        (last_angle >> 1) >= 360u * 64u) {
        return false;
    }
    return true;
}

static bool checksum_is_valid(size_t packet_size)
{
    /* XOR 16-bit PH, combined CT/LSN, FSA, LSA, and every raw sample.
     * The checksum field at offsets 8..9 is excluded. */
    uint16_t checksum = read_u16(packet_buffer);

    checksum ^= read_u16(packet_buffer + 2);
    checksum ^= read_u16(packet_buffer + 4);
    checksum ^= read_u16(packet_buffer + 6);
    for (size_t offset = HEADER_SIZE; offset < packet_size; offset += 2) {
        checksum ^= read_u16(packet_buffer + offset);
    }
    return checksum == read_u16(packet_buffer + 8);
}

static float normalize_angle(float angle)
{
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    if (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

static void enqueue_point(lidar_point_t point)
{
    stats.decoded_points++;
    if (queue_count == LIDAR_QUEUE_CAPACITY) {
        stats.dropped_points++;
        return;
    }
    point_queue[(queue_head + queue_count) % LIDAR_QUEUE_CAPACITY] = point;
    queue_count++;
}

static void decode_packet(void)
{
    uint8_t sample_count = packet_buffer[3];
    float first_angle = (float)(read_u16(packet_buffer + 4) >> 1) / 64.0f;
    float last_angle = (float)(read_u16(packet_buffer + 6) >> 1) / 64.0f;
    float span = last_angle - first_angle;
    float angle_step = 0.0f;

    if (span < 0.0f) {
        span += 360.0f;
    }
    if (sample_count > 1) {
        angle_step = span / (float)(sample_count - 1);
    }

    for (size_t index = 0; index < sample_count; index++) {
        lidar_point_t point;
        uint16_t sample = read_u16(packet_buffer + HEADER_SIZE + 2 * index);
        float angle = normalize_angle(first_angle + angle_step * (float)index);
        float correction = 0.0f;

        /* Use the manual's raw-sample / 4 distance convention, including its
         * fractional millimetres; do not reinterpret undocumented flag bits. */
        point.distance_mm = (float)sample / 4.0f;
        if (point.distance_mm > 0.0f) {
            correction = atanf(21.8f * (155.3f - point.distance_mm) /
                               (155.3f * point.distance_mm)) * radians_to_degrees;
        }
        point.angle_deg = normalize_angle(angle + correction);
        point.starts_rotation = (packet_buffer[2] & 1u) != 0 && index == 0;
        enqueue_point(point);
    }
}

static void process_pending_bytes(void)
{
    /* Each iteration consumes buffered bytes or returns for more input.
     * A failed candidate advances only one byte so an embedded next header
     * is retained, even after a corrupt count or checksum. */
    while (packet_bytes >= 2) {
        size_t packet_size;

        if (packet_buffer[0] != 0xAA || packet_buffer[1] != 0x55) {
            discard_prefix(1);
            continue;
        }
        if (packet_bytes < HEADER_SIZE) {
            return;
        }
        if (!header_is_valid()) {
            stats.rejected_packets++;
            discard_prefix(1);
            continue;
        }
        packet_size = HEADER_SIZE + 2u * packet_buffer[3];
        if (packet_bytes < packet_size) {
            return;
        }
        if (!checksum_is_valid(packet_size)) {
            stats.rejected_packets++;
            discard_prefix(1);
            continue;
        }
        stats.valid_packets++;
        decode_packet();
        discard_prefix(packet_size);
    }
}

void lidar_init(void)
{
    packet_bytes = 0;
    queue_head = 0;
    queue_count = 0;
    stats = (lidar_stats_t){0};
}

int lidar_process(void)
{
    size_t received;

    if (bsp_lidar_read(receive_buffer, sizeof receive_buffer, &received) != 0) {
        return -1;
    }
    (void)lidar_feed(receive_buffer, received);
    return 0;
}

size_t lidar_feed(const uint8_t *data, size_t size)
{
    if (data == NULL) {
        return 0;
    }
    if (size > LIDAR_INPUT_LIMIT) {
        size = LIDAR_INPUT_LIMIT;
    }
    for (size_t index = 0; index < size; index++) {
        /* A complete maximum-size packet is processed as soon as it fills
         * the buffer, so the next append always has space. */
        packet_buffer[packet_bytes++] = data[index];
        process_pending_bytes();
    }
    stats.received_bytes += (uint32_t)size;
    return size;
}

bool lidar_get_point(lidar_point_t *point)
{
    if (point == NULL || queue_count == 0) {
        return false;
    }
    *point = point_queue[queue_head];
    queue_head = (queue_head + 1) % LIDAR_QUEUE_CAPACITY;
    queue_count--;
    return true;
}

lidar_stats_t lidar_get_stats(void)
{
    return stats;
}

uint32_t lidar_received_bytes(void)
{
    return stats.received_bytes;
}
