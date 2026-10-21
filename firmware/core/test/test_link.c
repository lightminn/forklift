#include "fw/link.h"
#include "fw_test.h"

#include <string.h>

/* mcu-link/1 contract section 6 golden vectors (generated once with Python struct and
   binascii.crc_hqx(body, 0xFFFF)); the implementation must not be used to recompute them. */
static const uint8_t ARM_1234[] = {0x0f, 0x01, 0x02, 0x34, 0x12, 0xef, 0xbe, 0xad,
                                   0xde, 0x04, 0x03, 0x02, 0x01, 0xcc, 0x69, 0x00};
static const uint8_t DRIVE_0007[] = {0x04, 0x01, 0x01, 0x07, 0x01, 0x01, 0x03, 0xc0, 0x3f, 0x01,
                                     0x03, 0x10, 0xc0, 0x01, 0x01, 0x07, 0x3e, 0x9a, 0x99, 0x99,
                                     0x3e, 0x96, 0x0b, 0x01, 0x03, 0x02, 0xef, 0xbe, 0xad, 0xde,
                                     0xef, 0xcd, 0xab, 0x03, 0xee, 0x1a, 0x00};
static const uint8_t CLEAR_0008[] = {0x04, 0x01, 0x04, 0x08, 0x07, 0xef,
                                     0xbe, 0xad, 0xde, 0x02, 0x4e, 0x00};
static const uint8_t DISARM_0009[] = {0x04, 0x01, 0x03, 0x09, 0x03, 0xbc, 0x11, 0x00};
static const uint8_t STATE_0102[] = {
    0x16, 0x01, 0x81, 0x02, 0x01, 0x44, 0x33, 0x22, 0x11, 0xef, 0xbe, 0xad, 0xde, 0x03, 0x02,
    0x02, 0x01, 0x02, 0x07, 0x01, 0x80, 0x01, 0x08, 0xc0, 0x1d, 0xfe, 0xff, 0xf1, 0xfb, 0x09,
    0x01, 0x01, 0x03, 0xa0, 0x3f, 0x01, 0x01, 0x02, 0xbf, 0x01, 0x03, 0x80, 0x3d, 0x01, 0x08,
    0x40, 0x3f, 0xdc, 0x05, 0x38, 0xff, 0x1e, 0x09, 0xa0, 0x0f, 0x88, 0x13, 0xf0, 0xd8, 0xc4,
    0x09, 0x01, 0x04, 0xc0, 0xc4, 0xe0, 0x02, 0x11, 0x02, 0x02, 0x04, 0x0b, 0x16, 0xdf, 0x00};

static link_msg drive_0007(void)
{
    link_msg m = {.type = LINK_DRIVE, .seq = 7};
    m.u.drive = (link_drive){.left_wheel_rad_s = 1.5f,
                             .right_wheel_rad_s = -2.25f,
                             .steering_rad = 0.125f,
                             .lift_m = 0.3f,
                             .valid_after_echo_ms = 150,
                             .flags = 1,
                             .arm_gen = 0x0203,
                             .boot_id = 0xDEADBEEFu,
                             .echo_mcu_time_us = 0x00ABCDEFu};
    return m;
}

static link_msg state_0102(void)
{
    link_msg m = {.type = LINK_STATE, .seq = 0x0102};
    link_state *s = &m.u.state;
    *s = (link_state){.mcu_time_us = 0x11223344u,
                      .boot_id = 0xDEADBEEFu,
                      .arm_gen = 0x0203,
                      .last_rx_seq = 0x0102,
                      .mode = 2,
                      .reject_reason = 7,
                      .fault_flags = 0x00018001u,
                      .wheel_count_left = -123456,
                      .wheel_count_right = 654321,
                      .wheel_rate_left_rad_s = 1.25f,
                      .wheel_rate_right_rad_s = -0.5f,
                      .steering_rad = 0.0625f,
                      .lift_m = 0.75f,
                      .current_ma = {1500, -200, 30, 4000},
                      .duty_e4 = {5000, -10000, 2500, 0},
                      .driver_status = {0xC0, 0xC4, 0xE0, 0x00},
                      .rx_errors = 17,
                      .cycle_overruns = 2,
                      .flags = 0x0B};
    return m;
}

static int encodes_to(const link_msg *m, const uint8_t *want, size_t want_len)
{
    uint8_t wire[LINK_WIRE_MAX];
    size_t n = link_encode(m, wire, sizeof wire);
    return n == want_len && memcmp(wire, want, n) == 0;
}

/* Feeds a whole wire buffer; returns the number of messages and the last one. */
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

static void golden_vectors_encode(void)
{
    link_msg arm = {.type = LINK_ARM, .seq = 0x1234};
    arm.u.arm = (link_arm){.boot_id = 0xDEADBEEFu, .echo_mcu_time_us = 0x01020304u};
    link_msg clear = {.type = LINK_CLEAR_FAULTS, .seq = 8};
    clear.u.clear = (link_clear){.boot_id = 0xDEADBEEFu};
    link_msg disarm = {.type = LINK_DISARM, .seq = 9};
    link_msg drive = drive_0007();
    link_msg state = state_0102();
    CHECK(encodes_to(&arm, ARM_1234, sizeof ARM_1234));
    CHECK(encodes_to(&drive, DRIVE_0007, sizeof DRIVE_0007));
    CHECK(encodes_to(&clear, CLEAR_0008, sizeof CLEAR_0008));
    CHECK(encodes_to(&disarm, DISARM_0009, sizeof DISARM_0009));
    CHECK(encodes_to(&state, STATE_0102, sizeof STATE_0102));
}

static void golden_vectors_decode(void)
{
    link_rx rx;
    link_rx_init(&rx);
    link_msg m;
    CHECK_EQ_I(feed_all(&rx, DRIVE_0007, sizeof DRIVE_0007, &m), 1);
    link_msg want = drive_0007();
    CHECK_EQ_U(m.type, LINK_DRIVE);
    CHECK_EQ_U(m.seq, 7);
    const link_drive *d = &m.u.drive, *w_d = &want.u.drive;
    CHECK(d->left_wheel_rad_s == w_d->left_wheel_rad_s);
    CHECK(d->right_wheel_rad_s == w_d->right_wheel_rad_s);
    CHECK(d->steering_rad == w_d->steering_rad && d->lift_m == w_d->lift_m);
    CHECK_EQ_U(d->valid_after_echo_ms, 150);
    CHECK_EQ_U(d->flags, 1);
    CHECK_EQ_U(d->arm_gen, 0x0203);
    CHECK_EQ_U(d->boot_id, 0xDEADBEEFu);
    CHECK_EQ_U(d->echo_mcu_time_us, 0x00ABCDEFu);

    CHECK_EQ_I(feed_all(&rx, STATE_0102, sizeof STATE_0102, &m), 1);
    link_state s = m.u.state, w = state_0102().u.state;
    CHECK_EQ_U(m.type, LINK_STATE);
    CHECK_EQ_U(s.mcu_time_us, w.mcu_time_us);
    CHECK_EQ_U(s.boot_id, w.boot_id);
    CHECK_EQ_U(s.arm_gen, w.arm_gen);
    CHECK_EQ_U(s.last_rx_seq, w.last_rx_seq);
    CHECK_EQ_U(s.mode, w.mode);
    CHECK_EQ_U(s.reject_reason, w.reject_reason);
    CHECK_EQ_U(s.fault_flags, w.fault_flags);
    CHECK_EQ_I(s.wheel_count_left, w.wheel_count_left);
    CHECK_EQ_I(s.wheel_count_right, w.wheel_count_right);
    CHECK(s.wheel_rate_left_rad_s == w.wheel_rate_left_rad_s);
    CHECK(s.wheel_rate_right_rad_s == w.wheel_rate_right_rad_s);
    CHECK(s.steering_rad == w.steering_rad && s.lift_m == w.lift_m);
    for (int i = 0; i < 4; i++) {
        CHECK_EQ_I(s.current_ma[i], w.current_ma[i]);
        CHECK_EQ_I(s.duty_e4[i], w.duty_e4[i]);
        CHECK_EQ_U(s.driver_status[i], w.driver_status[i]);
    }
    CHECK_EQ_U(s.rx_errors, w.rx_errors);
    CHECK_EQ_U(s.cycle_overruns, w.cycle_overruns);
    CHECK_EQ_U(s.flags, w.flags);

    CHECK_EQ_I(feed_all(&rx, ARM_1234, sizeof ARM_1234, &m), 1);
    CHECK_EQ_U(m.type, LINK_ARM);
    CHECK_EQ_U(m.seq, 0x1234);
    CHECK_EQ_U(m.u.arm.boot_id, 0xDEADBEEFu);
    CHECK_EQ_U(m.u.arm.echo_mcu_time_us, 0x01020304u);
    CHECK_EQ_I(feed_all(&rx, CLEAR_0008, sizeof CLEAR_0008, &m), 1);
    CHECK_EQ_U(m.type, LINK_CLEAR_FAULTS);
    CHECK_EQ_U(m.u.clear.boot_id, 0xDEADBEEFu);
    CHECK_EQ_I(feed_all(&rx, DISARM_0009, sizeof DISARM_0009, &m), 1);
    CHECK_EQ_U(m.type, LINK_DISARM);
    CHECK_EQ_U(rx.errors, 0);
}

static void stream_resynchronises(void)
{
    uint8_t stream[128];
    size_t n = 0;
    const uint8_t garbage[] = {0x55, 0x66, 0x00};
    memcpy(stream + n, garbage, sizeof garbage);
    n += sizeof garbage;
    memcpy(stream + n, ARM_1234, sizeof ARM_1234);
    n += sizeof ARM_1234;
    stream[n++] = 0x00; /* empty frames are not errors */
    stream[n++] = 0x00;
    memcpy(stream + n, DRIVE_0007, sizeof DRIVE_0007);
    stream[n + 10] ^= 0x01; /* corrupt a payload byte: CRC fails */
    n += sizeof DRIVE_0007;
    memcpy(stream + n, DISARM_0009, sizeof DISARM_0009);
    n += sizeof DISARM_0009;

    link_rx rx;
    link_rx_init(&rx);
    link_msg m, got[4];
    int count = 0;
    for (size_t i = 0; i < n; i++) {
        if (link_rx_feed(&rx, stream[i], &m) && count < 4) {
            got[count++] = m;
        }
    }
    CHECK_EQ_I(count, 2);
    CHECK_EQ_U(got[0].type, LINK_ARM);
    CHECK_EQ_U(got[1].type, LINK_DISARM);
    CHECK_EQ_U(rx.errors, 2);

    /* A long run without a delimiter is dropped once and the receiver recovers. */
    link_rx_init(&rx);
    for (int i = 0; i < 200; i++) {
        CHECK(!link_rx_feed(&rx, 0x11, &m));
    }
    CHECK(!link_rx_feed(&rx, 0x00, &m));
    CHECK_EQ_U(rx.errors, 1);
    CHECK_EQ_I(feed_all(&rx, ARM_1234, sizeof ARM_1234, &m), 1);
}

static size_t body_with_crc(uint8_t *body, size_t len)
{
    uint16_t crc = crc16_ccitt_false(body, len);
    body[len] = (uint8_t)crc;
    body[len + 1] = (uint8_t)(crc >> 8);
    return len + 2;
}

static void decode_rejects(void)
{
    link_msg m;
    uint8_t body[100] = {1, LINK_ARM, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8};
    size_t n = body_with_crc(body, 12);
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_OK);
    body[0] = 2;
    n = body_with_crc(body, 12);
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_BAD_VERSION);
    body[0] = 1;
    body[1] = 0x7F;
    n = body_with_crc(body, 12);
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_BAD_TYPE);
    body[1] = LINK_ARM;
    n = body_with_crc(body, 11); /* ARM with a 7-byte payload */
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_BAD_LENGTH);
    n = body_with_crc(body, 12);
    body[3] ^= 0x80;
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_BAD_CRC);
    memset(body, 0, sizeof body);
    body[0] = 1;
    body[1] = LINK_STATE;
    n = body_with_crc(body, 79); /* 81-byte body */
    CHECK_EQ_U(link_decode_body(body, n, &m), LINK_OVERSIZE);
}

static void seq_order(void)
{
    CHECK(link_seq_newer(0, 0xFFFF));
    CHECK(!link_seq_newer(0xFFFF, 0));
    CHECK(!link_seq_newer(5, 5));
    CHECK(link_seq_newer(6, 5));
}

int main(void)
{
    golden_vectors_encode();
    golden_vectors_decode();
    stream_resynchronises();
    decode_rejects();
    seq_order();
    return FW_TEST_RESULT();
}
