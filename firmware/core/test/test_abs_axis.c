#include "fw/abs_axis.h"
#include "fw_test.h"

#define TWO_PI 6.28318530717958647692

/* synthetic configuration */
static abs_axis_config base_config(void)
{
    abs_axis_config c = {.crc = KTH_CRC_G704_BYTES,
                         .zero_offset_rad = 0.0f,
                         .sign = 1,
                         .sensor_per_joint = 1.0f,
                         .min_joint = -1.0f,
                         .max_joint = 1.0f,
                         .invalid_limit = 3};
    return c;
}

/* Frame = 12-bit angle << 4 | CRC; 0x0FF2 is the datasheet's example word. */
static uint16_t frame(uint16_t angle12)
{
    return (uint16_t)((angle12 << 4) | crc4_kth7801(angle12, KTH_CRC_G704_BYTES));
}

static void decodes_datasheet_word(void)
{
    uint16_t angle = 0;
    CHECK(kth7801_decode(0x0FF2, KTH_CRC_G704_BYTES, &angle));
    CHECK_EQ_U(angle, 0x0FF);
    CHECK(!kth7801_decode(0x0FF3, KTH_CRC_G704_BYTES, &angle));
}

static void crc_and_freshness_streaks(void)
{
    abs_axis_config c = base_config();
    abs_axis a;
    abs_axis_init(&a);
    CHECK(!a.valid);
    abs_axis_update(&a, &c, 0x0FF2, true);
    CHECK(a.valid);
    float held = a.position;
    CHECK_NEAR(held, 0x0FF * TWO_PI / 4096.0, 1e-5);

    abs_axis_update(&a, &c, 0x0FF3, true); /* bad CRC: position held */
    CHECK_EQ_U(a.crc_errors, 1);
    CHECK(a.valid);
    CHECK_NEAR(a.position, held, 0.0);
    abs_axis_update(&a, &c, 0x0FF2, false); /* not refreshed this cycle: also a failure */
    CHECK(a.valid);
    abs_axis_update(&a, &c, 0x0FF3, true); /* third failure in a row */
    CHECK(!a.valid);
    abs_axis_update(&a, &c, 0x0FF2, true); /* one good sample recovers */
    CHECK(a.valid);
    CHECK_EQ_U(a.bad_streak, 0);
}

static void continuous_across_sensor_zero(void)
{
    abs_axis_config c = base_config();
    c.zero_offset_rad = (float)(4000.0 * TWO_PI / 4096.0);
    abs_axis a;
    abs_axis_init(&a);
    abs_axis_update(&a, &c, frame(4090), true);
    float before = a.position;
    abs_axis_update(&a, &c, frame(10), true);
    CHECK(a.valid);
    CHECK_NEAR(a.position - before, 16.0 * TWO_PI / 4096.0, 1e-5);
}

static void out_of_range_flagged(void)
{
    abs_axis_config c = base_config();
    abs_axis a;
    abs_axis_init(&a);
    abs_axis_update(&a, &c, frame(800), true); /* 1.227 rad > max 1.0 */
    CHECK(a.out_of_range);
    CHECK(a.valid);
    abs_axis_update(&a, &c, frame(100), true);
    CHECK(!a.out_of_range);
}

static void config_must_avoid_wrap_cut(void)
{
    abs_axis_config c = base_config();
    CHECK(abs_axis_config_valid(&c));
    c.min_joint = -3.0f;
    c.max_joint = 3.0f;
    c.sensor_per_joint = 1.1f; /* 3.3 rad of sensor travel crosses +-pi */
    CHECK(!abs_axis_config_valid(&c));
    c = base_config();
    c.min_joint = 1.0f;
    c.max_joint = -1.0f;
    CHECK(!abs_axis_config_valid(&c));
    c = base_config();
    c.invalid_limit = 0;
    CHECK(!abs_axis_config_valid(&c));
    c = base_config();
    c.sensor_per_joint = 0.0f;
    CHECK(!abs_axis_config_valid(&c));
    c = base_config();
    c.sign = 0;
    CHECK(!abs_axis_config_valid(&c));
}

int main(void)
{
    decodes_datasheet_word();
    crc_and_freshness_streaks();
    continuous_across_sensor_zero();
    out_of_range_flagged();
    config_must_avoid_wrap_cut();
    return FW_TEST_RESULT();
}
