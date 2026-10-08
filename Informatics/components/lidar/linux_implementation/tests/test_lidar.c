#include "lidar.h"
#include "bsp_lidar.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Mock transport: parser tests never open hardware. */
static uint8_t mock_bytes[LIDAR_INPUT_LIMIT];
static size_t mock_count;
static int mock_error;

int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received)
{
    assert(capacity >= mock_count);
    memcpy(buffer, mock_bytes, mock_count);
    *received = mock_count;
    mock_count = 0;
    return mock_error;
}

static void write_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

/* Build protocol fixtures; assertions below use independently known results. */
static size_t make_packet(uint8_t *packet, uint8_t type, uint8_t count,
                          uint16_t first, uint16_t last, const uint16_t *samples)
{
    uint16_t checksum = 0x55AA ^ (uint16_t)(type | ((uint16_t)count << 8))
                        ^ first ^ last;

    write_u16(packet, 0x55AA);
    packet[2] = type;
    packet[3] = count;
    write_u16(packet + 4, first);
    write_u16(packet + 6, last);
    for (size_t index = 0; index < count; index++) {
        write_u16(packet + 10 + 2 * index, samples[index]);
        checksum ^= samples[index];
    }
    write_u16(packet + 8, checksum);
    return 10u + 2u * count;
}

static void feed_all(const uint8_t *data, size_t size)
{
    while (size > 0) {
        size_t consumed = lidar_feed(data, size);
        assert(consumed > 0 && consumed <= LIDAR_INPUT_LIMIT);
        data += consumed;
        size -= consumed;
    }
}

static void test_fragmentation_and_manual_examples(void)
{
    uint8_t packet[32];
    uint16_t samples[] = {4000, 32000};
    size_t length = make_packet(packet, 0, 2, 0x6FE5, 0x79BD, samples);

    /* Manual example: endpoint distances 1000/8000 mm, corrected angles
     * approximately 217.0178/235.6326 degrees (manual rounds intermediates). */
    for (size_t split = 0; split <= length; split++) {
        lidar_point_t point;
        lidar_init();
        assert(lidar_feed(packet, split) == split);
        if (split < length) {
            assert(!lidar_get_point(&point));
        }
        assert(lidar_feed(packet + split, length - split) == length - split);
        assert(lidar_get_point(&point));
        assert(point.distance_mm == 1000.0f);
        assert(fabsf(point.angle_deg - 217.0178f) < 0.005f);
        assert(!point.starts_rotation);
        assert(lidar_get_point(&point));
        assert(point.distance_mm == 8000.0f);
        assert(fabsf(point.angle_deg - 235.6326f) < 0.005f);
        assert(!lidar_get_point(&point));
        assert(lidar_get_stats().valid_packets == 1);
    }

    samples[0] = 0x6FE5; /* Manual's E5 6F -> 7161.25 mm example. */
    length = make_packet(packet, 1, 1, 1, 1, samples);
    lidar_init();
    for (size_t index = 0; index < length; index++) {
        assert(lidar_feed(packet + index, 1) == 1);
    }
    lidar_point_t point;
    assert(lidar_get_point(&point));
    assert(point.distance_mm == 7161.25f);
    assert(point.starts_rotation);
    assert(point.angle_deg >= 0.0f && point.angle_deg < 360.0f);
}

static void test_wrap_and_zero_returns(void)
{
    uint8_t packet[32];
    uint16_t samples[] = {0, 0, 0};
    size_t length = make_packet(packet, 0, 3, 359 * 128 + 1, 128 + 1, samples);
    const float expected[] = {359.0f, 0.0f, 1.0f};
    lidar_point_t point;

    lidar_init();
    feed_all(packet, length);
    for (size_t index = 0; index < 3; index++) {
        assert(lidar_get_point(&point));
        assert(point.angle_deg == expected[index]);
        assert(point.distance_mm == 0.0f);
    }
    assert(!lidar_get_point(&point));

    /* Very small nonzero samples have positive correction near 90 degrees.
     * Normalize both interpolation and correction, even for a wide span. */
    samples[0] = 1;
    samples[1] = 1;
    length = make_packet(packet, 0, 2, 359 * 128 + 1, 358 * 128 + 1, samples);
    feed_all(packet, length);
    for (size_t index = 0; index < 2; index++) {
        assert(lidar_get_point(&point));
        assert(point.angle_deg > 80.0f && point.angle_deg < 90.0f);
    }
}

static void test_corruption_and_resynchronization(void)
{
    uint8_t packet[12];
    uint16_t sample = 4000;
    uint8_t corrupt[520] = {0};
    const uint8_t startup[] = {
        0xA5, 0x5A, 0x14, 0, 0, 0, 4,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0xA5, 0x5A, 5, 0, 0, 0x40, 0x81, 0xAA
    };
    lidar_point_t point;

    make_packet(packet, 1, 1, 1, 1, &sample);
    lidar_init();
    feed_all(startup, sizeof startup);
    feed_all(packet, sizeof packet); /* Includes overlapping AA AA 55. */
    assert(lidar_get_point(&point));
    assert(point.distance_mm == 1000.0f);

    packet[8] ^= 1;
    feed_all(packet, sizeof packet);
    assert(!lidar_get_point(&point));
    assert(lidar_get_stats().rejected_packets == 1);
    packet[8] ^= 1;
    feed_all(packet, sizeof packet);
    assert(lidar_get_point(&point));

    /* Corrupt count advertises a long packet containing a valid next packet.
     * Once enough bytes arrive, recovery must preserve the embedded header. */
    corrupt[0] = 0xAA;
    corrupt[1] = 0x55;
    corrupt[3] = 255;
    corrupt[4] = 1;
    corrupt[6] = 1;
    memcpy(corrupt + 20, packet, sizeof packet);
    feed_all(corrupt, sizeof corrupt);
    assert(lidar_get_point(&point));
    assert(!lidar_get_point(&point));

    /* Invalid zero count, bad angle check bit, out-of-range angle,
     * and multi-sample rotation marker must not emit points. */
    const size_t fields[] = {3, 4, 5, 3};
    const uint8_t values[] = {0, 0, 255, 2};
    for (size_t index = 0; index < 4; index++) {
        lidar_init();
        uint8_t saved = packet[fields[index]];
        packet[fields[index]] = values[index];
        feed_all(packet, sizeof packet);
        assert(!lidar_get_point(&point));
        assert(lidar_get_stats().rejected_packets == 1);
        packet[fields[index]] = saved;
        feed_all(packet, sizeof packet);
        assert(lidar_get_point(&point));
    }
}

static void test_queue_and_limits(void)
{
    uint8_t packet[520];
    uint16_t samples[255];
    lidar_point_t point;

    for (size_t index = 0; index < 255; index++) {
        samples[index] = (uint16_t)(4000 + index);
    }
    size_t length = make_packet(packet, 0, 255, 1, 128 + 1, samples);
    lidar_init();
    assert(lidar_feed(NULL, 10) == 0);
    assert(lidar_feed(packet, length) == LIDAR_INPUT_LIMIT);
    feed_all(packet + LIDAR_INPUT_LIMIT, length - LIDAR_INPUT_LIMIT);
    assert(lidar_get_stats().valid_packets == 1);
    feed_all(packet, length);
    feed_all(packet, length);
    assert(lidar_get_stats().decoded_points == 765);
    assert(lidar_get_stats().dropped_points == 765 - LIDAR_QUEUE_CAPACITY);
    assert(!lidar_get_point(NULL));
    for (size_t index = 0; index < LIDAR_QUEUE_CAPACITY; index++) {
        assert(lidar_get_point(&point));
        assert(point.distance_mm == (float)samples[index % 255] / 4.0f);
    }
    assert(!lidar_get_point(&point));
    feed_all(packet, length); /* Ring indices wrap after draining. */
    for (size_t index = 0; index < 255; index++) {
        assert(lidar_get_point(&point));
        assert(point.distance_mm == (float)samples[index] / 4.0f);
    }
    lidar_init();
    assert(lidar_get_stats().received_bytes == 0);
    assert(!lidar_get_point(&point));
}

static void test_process_transport(void)
{
    uint16_t sample = 4000;
    lidar_point_t point;

    lidar_init();
    mock_error = 0;
    mock_count = 0;
    assert(lidar_process() == 0);
    assert(lidar_received_bytes() == 0);
    mock_count = make_packet(mock_bytes, 1, 1, 1, 1, &sample);
    assert(lidar_process() == 0);
    assert(lidar_get_point(&point));
    assert(point.distance_mm == 1000.0f);
    mock_error = -1;
    assert(lidar_process() == -1);
    assert(lidar_received_bytes() == 12);
}

static int replay_capture(const char *input_path, const char *output_path)
{
    uint8_t bytes[73]; /* Deliberately unrelated to packet boundaries. */
    FILE *input = fopen(input_path, "rb");
    if (input == NULL) {
        perror(input_path);
        return 1;
    }
    FILE *output = fopen(output_path, "w");
    if (output == NULL) {
        perror(output_path);
        fclose(input);
        return 1;
    }
    lidar_init();
    fputs("angle_deg,distance_mm\n", output);
    size_t received;
    while ((received = fread(bytes, 1, sizeof bytes, input)) > 0) {
        feed_all(bytes, received);
        lidar_point_t point;
        while (lidar_get_point(&point)) {
            fprintf(output, "%.6f,%.2f\n", (double)point.angle_deg,
                    (double)point.distance_mm);
        }
    }
    int result = ferror(input) ? 1 : 0;
    if (fclose(output) != 0) {
        result = 1;
    }
    fclose(input);
    lidar_stats_t stats = lidar_get_stats();
    printf("Replay: %u bytes, %u valid packets, %u rejected, %u points, %u dropped\n",
           (unsigned)stats.received_bytes, (unsigned)stats.valid_packets,
           (unsigned)stats.rejected_packets, (unsigned)stats.decoded_points,
           (unsigned)stats.dropped_points);
    return result;
}

int main(int argc, char **argv)
{
    test_fragmentation_and_manual_examples();
    test_wrap_and_zero_returns();
    test_corruption_and_resynchronization();
    test_queue_and_limits();
    test_process_transport();
    puts("PASS: manual examples, fragmentation, wraparound, missing returns, corruption recovery, queue overflow, input bounds, transport errors.");
    if (argc == 3) {
        return replay_capture(argv[1], argv[2]);
    }
    return argc == 1 ? 0 : 1;
}
