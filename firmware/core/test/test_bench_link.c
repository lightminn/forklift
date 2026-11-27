/* Bench extension of mcu-link (contract section 8, wire generation bench5): the golden vectors shared with
   tests/unit/mcu_link/test_bench_messages.py (generated there with Python struct and binascii.crc_hqx), and the
   bench types round trip through the stream decoder. Built against the core with FW_CORE_BENCH. */
#include "fw/link.h"
#include "fw_test.h"

#include <string.h>

static const uint8_t BENCH_SET_000B[] = {
    0x04, 0x03, 0x06, 0x0b, 0x08, 0xef, 0xbe, 0xad, 0xde, 0xef, 0xcd, 0xab, 0x02, 0x01, 0x01, 0x01,
    0x07, 0x78, 0x56, 0x34, 0x12, 0x01, 0x02, 0x03, 0x01, 0x01, 0x01, 0x01, 0x0c, 0x10, 0x27, 0x10,
    0x27, 0xc4, 0x09, 0xdc, 0x05, 0x9b, 0xc4, 0x20, 0x04, 0x9b, 0xc4, 0x20, 0x03, 0x30, 0x75, 0x01,
    0x03, 0xd0, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0xe9, 0x42, 0x00};
static const uint8_t BENCH_ARM_000C[] = {
    0x04, 0x03, 0x08, 0x0c, 0x08, 0xef, 0xbe, 0xad, 0xde, 0xef, 0xcd, 0xab, 0x02, 0x01, 0x01, 0x01,
    0x07, 0x78, 0x56, 0x34, 0x12, 0x4c, 0x73, 0x00};
static const uint8_t BENCH_STATE_0103[] = {
    0x10, 0x03, 0x82, 0x03, 0x01, 0x02, 0x01, 0x78, 0x56, 0x34, 0x12, 0x44, 0x33, 0x22, 0x11, 0x02,
    0x01, 0x01, 0x0a, 0xbc, 0x0a, 0xc0, 0x1d, 0xfe, 0xff, 0xf1, 0xfb, 0x09, 0x06, 0xf9, 0xff, 0xff,
    0xff, 0x08, 0x01, 0x01, 0x02, 0x01, 0x02, 0x03, 0x02, 0x04, 0x04, 0xff, 0xff, 0xfa, 0x04, 0x01,
    0x13, 0x05, 0x02, 0x02, 0x03, 0x65, 0x97, 0x00};

static int encodes_to(const link_msg *m, const uint8_t *want, size_t want_len)
{
    uint8_t wire[LINK_WIRE_MAX];
    size_t n = link_encode(m, wire, sizeof wire);
    return n == want_len && memcmp(wire, want, n) == 0;
}

static int feed_all(link_rx *rx, const uint8_t *wire, size_t len, link_msg *last)
{
    int count = 0;
    for (size_t i = 0; i < len; i++) {
        if (link_rx_feed(rx, wire[i], last)) {
            count++;
        }
    }
    return count;
}

static link_msg bench_set_000b(void)
{
    link_msg m = {.type = LINK_BENCH_SET, .seq = 0x000B};
    m.u.bench_set = (link_bench_set){.boot_id = 0xDEADBEEFu,
                                      .echo_mcu_time_us = 0x0000000100ABCDEFull,
                                      .run_id = 0x12345678u,
                                      .mode = 1,
                                      .axis = 2,
                                      .source = 0,
                                      .count_sign = 1,
                                      .bypass = 1,
                                      .duty_max_e4 = {10000, 10000, 2500, 1500},
                                      .time_ms = {2147483, 2147483, 30000, 2000},
                                      .stop_counts = 0};
    return m;
}

static link_msg bench_arm_000c(void)
{
    link_msg m = {.type = LINK_BENCH_ARM, .seq = 0x000C};
    m.u.bench_arm = (link_bench_arm){.boot_id = 0xDEADBEEFu, .echo_mcu_time_us = 0x0000000100ABCDEFull,
                                      .run_id = 0x12345678u};
    return m;
}

static link_msg bench_state_0103(void)
{
    link_msg m = {.type = LINK_BENCH_STATE, .seq = 0x0103};
    m.u.bench_state = (link_bench_state){.pair_seq = 0x0102,
                                          .run_id = 0x12345678u,
                                          .result_run_id = 0x11223344u,
                                          .axis = 2,
                                          .raw_angle = 0x0ABC,
                                          .cumulative = -123456,
                                          .target = 654321,
                                          .distance_ref = -7,
                                          .cut_position = 8,
                                          .stop_reason = 1,
                                          .blocked = 3,
                                          .dropped_pairs = 4,
                                          .evidence_age_ms = 0xFFFF,
                                          .end_evidence_age_ms = 250,
                                          .mcu_bypass = 1,
                                          .drive = 0x13,
                                          .conn_gen = 5,
                                          .result_axis = 2};
    return m;
}

static void golden_vectors(void)
{
    link_msg set = bench_set_000b(), arm = bench_arm_000c(), st = bench_state_0103();
    CHECK(encodes_to(&set, BENCH_SET_000B, sizeof BENCH_SET_000B));
    CHECK(encodes_to(&arm, BENCH_ARM_000C, sizeof BENCH_ARM_000C));
    CHECK(encodes_to(&st, BENCH_STATE_0103, sizeof BENCH_STATE_0103));

    link_rx rx;
    link_rx_init(&rx);
    link_msg m;
    CHECK_EQ_I(feed_all(&rx, BENCH_SET_000B, sizeof BENCH_SET_000B, &m), 1);
    CHECK_EQ_U(m.type, LINK_BENCH_SET);
    CHECK_EQ_U(m.seq, 0x000B);
    CHECK_EQ_U(m.u.bench_set.echo_mcu_time_us, 0x0000000100ABCDEFull);
    CHECK_EQ_U(m.u.bench_set.run_id, 0x12345678u);
    CHECK_EQ_U(m.u.bench_set.axis, 2);
    CHECK_EQ_I(m.u.bench_set.count_sign, 1);
    CHECK_EQ_U(m.u.bench_set.duty_max_e4[3], 1500);
    CHECK_EQ_U(m.u.bench_set.time_ms[0], 2147483);
    CHECK_EQ_I(m.u.bench_set.stop_counts, 0);

    CHECK_EQ_I(feed_all(&rx, BENCH_ARM_000C, sizeof BENCH_ARM_000C, &m), 1);
    CHECK_EQ_U(m.type, LINK_BENCH_ARM);
    CHECK_EQ_U(m.u.bench_arm.run_id, 0x12345678u);
    CHECK_EQ_U(m.u.bench_arm.echo_mcu_time_us, 0x0000000100ABCDEFull);

    CHECK_EQ_I(feed_all(&rx, BENCH_STATE_0103, sizeof BENCH_STATE_0103, &m), 1);
    CHECK_EQ_U(m.type, LINK_BENCH_STATE);
    CHECK_EQ_U(m.u.bench_state.pair_seq, 0x0102);
    CHECK_EQ_U(m.u.bench_state.result_run_id, 0x11223344u);
    CHECK_EQ_I(m.u.bench_state.cumulative, -123456);
    CHECK_EQ_I(m.u.bench_state.distance_ref, -7);
    CHECK_EQ_U(m.u.bench_state.evidence_age_ms, 0xFFFF);
    CHECK_EQ_U(m.u.bench_state.drive, 0x13);
    CHECK_EQ_U(m.u.bench_state.result_axis, 2);
    CHECK_EQ_U(rx.errors, 0);
}

/* A negative stop count and the extreme run id survive the round trip. */
static void round_trip_extremes(void)
{
    link_msg set = bench_set_000b();
    set.u.bench_set.stop_counts = INT32_MIN;
    set.u.bench_set.run_id = 0xFFFFFFFFu;
    set.u.bench_set.count_sign = -1;
    uint8_t wire[LINK_WIRE_MAX];
    size_t n = link_encode(&set, wire, sizeof wire);
    CHECK(n > 0);
    link_rx rx;
    link_rx_init(&rx);
    link_msg m;
    CHECK_EQ_I(feed_all(&rx, wire, n, &m), 1);
    CHECK_EQ_I(m.u.bench_set.stop_counts, INT32_MIN);
    CHECK_EQ_U(m.u.bench_set.run_id, 0xFFFFFFFFu);
    CHECK_EQ_I(m.u.bench_set.count_sign, -1);
}

int main(void)
{
    golden_vectors();
    round_trip_extremes();
    return fw_test_failures;
}
