/* bench_kth: the bench image's KTH travel counter (bench console plan 4.2, A1b-1 decision 2 and its reviews). */
#include "fw/bench_kth.h"
#include "fw/crc.h"
#include "fw_test.h"

#define V KTH_CRC_G704_BYTES
#define MS 1000u

static uint16_t fr(uint16_t raw)
{
    return (uint16_t)((uint16_t)(raw << 4) | crc4_kth7801(raw, V));
}

/* A frame acquired at t (µs) and consumed at the same time. */
static uint8_t at(bench_kth *k, uint16_t raw, uint64_t t)
{
    return bench_kth_tick(k, true, fr(raw), V, t, t);
}

static uint8_t none(bench_kth *k, uint64_t now)
{
    return bench_kth_tick(k, false, 0, V, 0, now);
}

static int32_t rel(const bench_kth *k, uint32_t ref)
{
    return bench_kth_diff(k->cum, ref);
}

static void test_first_sample_is_non_zero(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    CHECK_EQ_U(at(&k, 0, 1 * MS), 0);
    CHECK(!k.have);
    CHECK_EQ_U(at(&k, 100, 2 * MS), BENCH_KTH_TRUSTED);
    CHECK(k.have && !k.lost);
}

static void test_zero_counted_only_near_zero(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 512, 0);
    CHECK_EQ_U(at(&k, 0, 1 * MS), BENCH_KTH_COUNTED_ZERO);
    CHECK(k.last_zero);
    bench_kth_reset(&k);
    at(&k, 513, 0);
    CHECK_EQ_U(at(&k, 0, 1 * MS), 0); /* not countable */
    CHECK(!k.lost);
    bench_kth_reset(&k);
    at(&k, 4096 - 512, 0);
    CHECK_EQ_U(at(&k, 0, 1 * MS), BENCH_KTH_COUNTED_ZERO);
    bench_kth_reset(&k);
    at(&k, 4096 - 513, 0);
    CHECK_EQ_U(at(&k, 0, 1 * MS), 0);
}

static void test_fake_zero_step_is_undone_on_recovery(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 256, 0);
    uint32_t ref = k.cum;
    at(&k, 0, 1 * MS);
    CHECK_EQ_I(rel(&k, ref), -256); /* tentative */
    CHECK_EQ_U(at(&k, 256, 2 * MS), BENCH_KTH_TRUSTED);
    CHECK_EQ_I(rel(&k, ref), 0);
    CHECK(!k.last_zero);
}

static void test_reanchor_bound(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 512, 0);
    uint32_t ref = k.cum;
    at(&k, 0, 1 * MS);
    CHECK_EQ_U(at(&k, 256, 3 * MS), BENCH_KTH_TRUSTED); /* 3,000 µs: re-anchored, -256 */
    CHECK_EQ_I(rel(&k, ref), -256);
    bench_kth_reset(&k);
    at(&k, 512, 0);
    at(&k, 0, 1 * MS);
    CHECK(at(&k, 2560, 4 * MS) & BENCH_KTH_LOST); /* the 4 ms counterexample: lost, never -2048 */
    bench_kth_reset(&k);
    at(&k, 512, 0);
    at(&k, 0, 1 * MS);
    CHECK(at(&k, 256, 3001) & BENCH_KTH_LOST); /* 3,001 µs */
}

static void test_speed_bound(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 1000, 0);
    CHECK_EQ_U(at(&k, 1512, 1 * MS), BENCH_KTH_TRUSTED);
    CHECK(at(&k, 2025, 2 * MS) & BENCH_KTH_LOST); /* 513 in 1 ms */
}

static void test_recovery_from_the_detecting_frame(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 1000, 0);
    uint32_t losses = k.losses;
    CHECK(at(&k, 3000, 1 * MS) & BENCH_KTH_LOST); /* too fast: lost, 3000 is the base */
    CHECK_EQ_U(k.losses, losses + 1u);
    CHECK_EQ_U(at(&k, 3100, 2 * MS), BENCH_KTH_TRUSTED | BENCH_KTH_RECOVERED);
    CHECK(!k.lost);
    CHECK_EQ_U(at(&k, 3150, 3 * MS), BENCH_KTH_TRUSTED);
}

static void test_gap_and_age_order(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 1000, 50);
    CHECK(none(&k, 4000) & BENCH_KTH_LOST); /* no new frame: now - last 3,950 µs */
    bench_kth_reset(&k);
    at(&k, 1000, 50);
    CHECK_EQ_U(bench_kth_tick(&k, true, fr(1100), V, 3050, 4000), BENCH_KTH_TRUSTED); /* interval 3,000, age 950 */
    bench_kth_reset(&k);
    at(&k, 1000, 50);
    CHECK(bench_kth_tick(&k, true, fr(1010), V, 1050, 6000) & BENCH_KTH_LOST); /* interval 1,000, age 4,950 */
    CHECK(!k.base_valid);                                                     /* never a base */
}

static void test_rest_at_raw_zero_is_not_a_loss(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 4, 0);
    for (uint64_t t = 1; t <= 500; t++) {
        CHECK_EQ_U(at(&k, 0, t * MS), BENCH_KTH_COUNTED_ZERO);
    }
    CHECK(!k.lost);
}

static void test_slow_zero_crossing_is_lost(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 4095, 0);
    at(&k, 0, 1 * MS);
    at(&k, 0, 2 * MS);
    at(&k, 0, 3 * MS);
    CHECK(at(&k, 1, 4 * MS) & BENCH_KTH_LOST); /* 4 ms from the last trusted sample */
    CHECK_EQ_U(at(&k, 2, 5 * MS), BENCH_KTH_TRUSTED | BENCH_KTH_RECOVERED);
}

static void test_one_full_turn_while_stuck_is_lost(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 4032, 0);
    for (uint64_t t = 1; t <= 65; t++) {
        at(&k, 0, t * MS);
    }
    CHECK(at(&k, 64, 66 * MS) & BENCH_KTH_LOST);
}

static void test_modular_difference(void)
{
    CHECK_EQ_I(bench_kth_diff(0x80000000u, 0x7fffffffu), 1);
    CHECK_EQ_I(bench_kth_diff(0x7fffffffu, 0x80000000u), -1);
    CHECK_EQ_I(bench_kth_diff(0u, 0xffffffffu), 1);
    CHECK_EQ_I(bench_kth_diff(5u, 5u), 0);
    bench_kth k;
    bench_kth_reset(&k);
    k.cum = 0x7ffffff0u;
    at(&k, 100, 0);
    uint32_t ref = k.cum;
    for (uint64_t t = 1; t <= 4; t++) {
        at(&k, (uint16_t)(100 + 10 * t), t * MS);
    }
    CHECK_EQ_I(rel(&k, ref), 40);
    CHECK(k.cum > 0x7fffffffu || k.cum < 0x10000u);
}

static void test_crc_failure_is_no_frame(void)
{
    bench_kth k;
    bench_kth_reset(&k);
    at(&k, 1000, 0);
    CHECK_EQ_U(bench_kth_tick(&k, true, (uint16_t)(fr(1001) ^ 1u), V, 1 * MS, 1 * MS), 0);
    CHECK(none(&k, 3001) & BENCH_KTH_LOST);
}

int main(void)
{
    test_first_sample_is_non_zero();
    test_zero_counted_only_near_zero();
    test_fake_zero_step_is_undone_on_recovery();
    test_reanchor_bound();
    test_speed_bound();
    test_recovery_from_the_detecting_frame();
    test_gap_and_age_order();
    test_rest_at_raw_zero_is_not_a_loss();
    test_slow_zero_crossing_is_lost();
    test_one_full_turn_while_stuck_is_lost();
    test_modular_difference();
    test_crc_failure_is_no_frame();
    return fw_test_failures;
}
