#include "lidar_cans.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void feed(float angle_deg, float distance_mm, bool starts_rotation, uint32_t now_ms)
{
    lidar_point_t point = {angle_deg, distance_mm, starts_rotation};
    lidar_cans_feed(&point, now_ms);
}

static void expect_can(lidar_cans_result_t result, float bearing_deg, float distance_mm)
{
    for (size_t i = 0; i < result.count; ++i) {
        if (fabsf(result.cans[i].bearing_deg - bearing_deg) < 0.01f &&
            fabsf(result.cans[i].distance_mm - distance_mm) < 0.01f) {
            return;
        }
    }
    assert(!"Expected bearing and distance were not reported");
}

static void test_three_cans_and_empty_scan(void)
{
    lidar_cans_reset();
    feed(170.0f, 900.0f, false, 10); /* Initial partial rotation. */
    feed(0.0f, 250.0f, true, 50);
    feed(1.0f, 0.0f, false, 51);
    feed(89.0f, 250.0f, false, 100);
    feed(90.0f, 230.0f, false, 101);
    feed(91.0f, 250.0f, false, 102);
    feed(92.0f, 2500.0f, false, 103);
    feed(239.0f, 0.0f, false, 160);
    feed(240.0f, 280.0f, false, 161);
    feed(241.0f, 300.0f, false, 162);
    feed(242.0f, 0.0f, false, 163);
    feed(358.0f, 0.0f, false, 245);
    feed(359.0f, 270.0f, false, 248);
    assert(lidar_cans_get_result(248).status == LIDAR_CANS_UNAVAILABLE);
    feed(0.0f, 0.0f, true, 250);

    lidar_cans_result_t result = lidar_cans_get_result(251);
    assert(result.status == LIDAR_CANS_VALID && result.count == 3);
    assert(result.completed_at_ms == 250);
    expect_can(result, 359.5f, 250.0f);
    expect_can(result, 90.0f, 230.0f);
    expect_can(result, 240.5f, 280.0f);

    feed(90.0f, 0.0f, false, 300);
    assert(lidar_cans_get_result(300).count == 3);
    feed(0.0f, 0.0f, true, 450);
    result = lidar_cans_get_result(450);
    assert(result.status == LIDAR_CANS_VALID && result.count == 0);
    assert(lidar_cans_get_result(949).status == LIDAR_CANS_VALID);
    assert(lidar_cans_get_result(950).status == LIDAR_CANS_UNAVAILABLE);
}

static void test_excess_candidates_and_lost_scan(void)
{
    lidar_cans_reset();
    feed(0.0f, 0.0f, true, 0);
    for (unsigned i = 1; i <= 4; ++i) {
        float angle = 60.0f * i;
        feed(angle, 260.0f, false, i * 30);
        feed(angle + 1.0f, 270.0f, false, i * 30 + 1);
        feed(angle + 2.0f, 0.0f, false, i * 30 + 2);
    }
    feed(0.0f, 0.0f, true, 200);
    lidar_cans_result_t result = lidar_cans_get_result(200);
    assert(result.status == LIDAR_CANS_TOO_MANY && result.count == 0);
    assert(result.overflow_candidate.bearing_deg == 240.5f);
    assert(result.overflow_candidate.distance_mm == 260.0f);
    assert(result.overflow_points == 2);

    for (unsigned i = 0; i < 1024; ++i) {
        feed(30.0f, 0.0f, false, 201);
    }
    feed(0.0f, 0.0f, true, 400);
    assert(lidar_cans_get_result(400).status == LIDAR_CANS_INCOMPLETE);
    feed(0.0f, 0.0f, true, 1000);
    assert(lidar_cans_get_result(1000).status == LIDAR_CANS_INCOMPLETE);
    lidar_cans_reset();
    assert(lidar_cans_get_result(1000).status == LIDAR_CANS_UNAVAILABLE);
}

static void test_missed_returns_in_one_can(void)
{
    lidar_cans_reset();
    feed(0.0f, 0.0f, true, 0);
    feed(0.2f, 0.0f, false, 0);
    feed(0.4f, 0.0f, false, 0);
    feed(1.0f, 210.0f, false, 1);
    feed(2.0f, 218.0f, false, 2);
    feed(3.0f, 0.0f, false, 3);

    /* A gap wider than the can keeps these two groups separate. */
    feed(80.0f, 250.0f, false, 45);
    feed(81.0f, 256.0f, false, 46);
    feed(82.0f, 0.0f, false, 47);
    feed(83.0f, 0.0f, false, 48);
    feed(84.0f, 0.0f, false, 49);
    feed(99.0f, 249.0f, false, 50);
    feed(100.0f, 255.0f, false, 51);
    feed(101.0f, 0.0f, false, 52);

    /* These returns and the first two hits form one can across 0 degrees. */
    feed(356.0f, 0.0f, false, 196);
    feed(357.0f, 220.0f, false, 197);
    feed(358.0f, 213.0f, false, 198);
    feed(358.5f, 0.0f, false, 198);
    feed(359.0f, 0.0f, false, 199);
    feed(359.5f, 0.0f, false, 199);
    feed(0.0f, 0.0f, true, 200);

    lidar_cans_result_t result = lidar_cans_get_result(200);
    assert(result.status == LIDAR_CANS_VALID && result.count == 3);
    expect_can(result, 359.5f, 210.0f);
    expect_can(result, 80.5f, 250.0f);
    expect_can(result, 99.5f, 249.0f);
}

static void test_size_cutoff_and_clock_rollover(void)
{
    lidar_cans_reset();
    feed(0.0f, 0.0f, true, UINT32_MAX - 190U);
    for (unsigned i = 0; i <= 6; ++i) {
        feed(90.0f + 3.0f * i, 250.0f, false, UINT32_MAX - 140U + i);
    }
    feed(109.0f, 0.0f, false, UINT32_MAX - 133U);
    feed(180.0f, 250.0f, false, UINT32_MAX - 95U); /* Single return. */
    feed(181.0f, 0.0f, false, UINT32_MAX - 94U);
    feed(240.0f, 2600.0f, false, UINT32_MAX - 65U);
    feed(241.0f, 2600.0f, false, UINT32_MAX - 64U);
    feed(242.0f, 0.0f, false, UINT32_MAX - 63U);
    feed(0.0f, 0.0f, true, 9);
    lidar_cans_result_t result = lidar_cans_get_result(9);
    assert(result.status == LIDAR_CANS_VALID && result.count == 0);
}

int main(void)
{
    test_three_cans_and_empty_scan();
    test_missed_returns_in_one_can();
    test_excess_candidates_and_lost_scan();
    test_size_cutoff_and_clock_rollover();
    puts("PASS: can list, rotation wrap, empty and stale scans, overflow, size and range filters.");
}
