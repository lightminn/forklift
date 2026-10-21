#include "fw/wheel_enc.h"
#include "fw_test.h"

/* synthetic configuration */
static const wheel_enc_config CFG = {.counts_per_rev = 4096, .window = 20, .sign = 1};

static void unwraps_both_directions(void)
{
    wheel_enc e;
    CHECK(wheel_enc_init(&e, &CFG, 0xFFF0, 0));
    for (uint32_t k = 1; k <= 3; k++) {
        wheel_enc_update(&e, &CFG, (uint16_t)(0xFFF0 + 0x20 * k), 1000 * k);
    }
    CHECK_EQ_I((int32_t)e.count, 96);

    CHECK(wheel_enc_init(&e, &CFG, 0x0010, 0));
    for (uint32_t k = 1; k <= 3; k++) {
        wheel_enc_update(&e, &CFG, (uint16_t)(0x0010 - 0x20 * k), 1000 * k);
    }
    CHECK_EQ_I((int32_t)e.count, -96);

    wheel_enc_config reversed = CFG;
    reversed.sign = -1;
    CHECK(wheel_enc_init(&e, &reversed, 0, 0));
    wheel_enc_update(&e, &reversed, 0x20, 1000);
    CHECK_EQ_I((int32_t)e.count, -32);
}

static void count_is_32bit_modular(void)
{
    wheel_enc e;
    CHECK(wheel_enc_init(&e, &CFG, 0, 0));
    e.count = 0xFFFFFFF0u;
    wheel_enc_update(&e, &CFG, 0x20, 1000);
    CHECK_EQ_U(e.count, 0x10);
}

static void rate_from_timestamps(uint32_t t0)
{
    /* +4 counts per 1000 us: 4 * 2pi / 4096 / 0.001 = 6.1359 rad/s. */
    const double want = 4.0 * 2.0 * 3.14159265358979 / 4096.0 / 0.001;
    wheel_enc e;
    uint16_t hw = 0xFF00;
    uint32_t t = t0;
    CHECK(wheel_enc_init(&e, &CFG, hw, t));
    CHECK_NEAR(e.rate_rad_s, 0.0, 1e-9);
    for (int k = 0; k < 5; k++) { /* window not yet full: rate from the samples so far */
        hw = (uint16_t)(hw + 4);
        t += 1000;
        wheel_enc_update(&e, &CFG, hw, t);
    }
    CHECK_NEAR(e.rate_rad_s, want, 1e-3);
    for (int k = 0; k < 30; k++) {
        hw = (uint16_t)(hw + 4);
        t += 1000;
        wheel_enc_update(&e, &CFG, hw, t);
    }
    CHECK_NEAR(e.rate_rad_s, want, 1e-3);
    /* One late sample (2000 us, +8 counts) leaves the rate unchanged. */
    hw = (uint16_t)(hw + 8);
    t += 2000;
    wheel_enc_update(&e, &CFG, hw, t);
    CHECK_NEAR(e.rate_rad_s, want, want * 0.01);
}

static void rejects_bad_config(void)
{
    wheel_enc e;
    wheel_enc_config c = CFG;
    c.window = 0;
    CHECK(!wheel_enc_init(&e, &c, 0, 0));
    c.window = WHEEL_ENC_WINDOW_MAX + 1;
    CHECK(!wheel_enc_init(&e, &c, 0, 0));
    c = CFG;
    c.counts_per_rev = 0;
    CHECK(!wheel_enc_init(&e, &c, 0, 0));
    c = CFG;
    c.sign = 0;
    CHECK(!wheel_enc_init(&e, &c, 0, 0));
    c.sign = 2;
    CHECK(!wheel_enc_init(&e, &c, 0, 0));
}

int main(void)
{
    unwraps_both_directions();
    count_is_32bit_modular();
    rate_from_timestamps(0);
    rate_from_timestamps(0xFFFFFF00u); /* MCU clock wraps inside the window */
    rejects_bad_config();
    return FW_TEST_RESULT();
}
