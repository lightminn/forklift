/* The shared plant model (board design 9), on the host. */
#include <math.h>
#include <string.h>

#include "fw/drv8244.h"
#include "fw_test.h"
#include "plant.h"

static drive_config config(void)
{
    drive_config c;
    memset(&c, 0, sizeof c);
    c.max_wheel_rad_s = 10.0f;
    c.wheel[0].counts_per_rev = 4096;
    c.wheel[1].counts_per_rev = 8192;
    c.steer = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                                .zero_offset_rad = 3.14159265f,
                                .sign = 1,
                                .sensor_per_joint = 1.0f,
                                .min_joint = -0.6f,
                                .max_joint = 0.6f,
                                .invalid_limit = 3};
    c.lift = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                               .zero_offset_rad = 3.14159265f,
                               .sign = -1,
                               .sensor_per_joint = 10.0f,
                               .min_joint = 0.02f,
                               .max_joint = 0.25f,
                               .invalid_limit = 3};
    return c;
}

/* A frame the plant makes decodes to the joint it was made from, under the board's axis
   configurations (zero offset and sign included). */
static void kth_round_trip(void)
{
    drive_config c = config();
    const double joints[] = {-0.5, -0.1, 0.0, 0.3, 0.55};
    for (unsigned i = 0; i < sizeof joints / sizeof joints[0]; i++) {
        abs_axis a;
        abs_axis_init(&a);
        abs_axis_update(&a, &c.steer, plant_kth_frame(joints[i], &c.steer), true);
        CHECK(a.valid && !a.out_of_range);
        CHECK(fabs((double)a.position - joints[i]) < 2e-3); /* 12-bit sensor step */
    }
    abs_axis a;
    abs_axis_init(&a);
    abs_axis_update(&a, &c.lift, plant_kth_frame(0.1, &c.lift), true);
    CHECK(a.valid && fabs((double)a.position - 0.1) < 2e-4);
}

/* Lift starts at zero held inside its range: the core sees a valid in-range axis at boot. */
static void starts_in_range(void)
{
    drive_config c = config();
    plant p;
    plant_init(&p, &c);
    CHECK(fabs(p.lift_m - (0.02 + 0.0023)) < 1e-6 && p.steer_rad == 0.0);
    drive_inputs in;
    memset(&in, 0, sizeof in);
    plant_inputs(&p, &in);
    abs_axis a;
    abs_axis_init(&a);
    abs_axis_update(&a, &c.lift, in.abs_frame[1], in.abs_fresh[1]);
    CHECK(a.valid && !a.out_of_range);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK(in.nfault_low[ch]); /* latched until CLR_FLT */
    }
}

static void wheels_follow_duty(void)
{
    drive_config c = config();
    plant p;
    plant_init(&p, &c);
    drive_outputs out;
    memset(&out, 0, sizeof out);
    out.duty[0] = 0.5f;
    out.duty[1] = 0.5f;
    out.drvoff[1] = true; /* off: no drive */
    for (int i = 0; i < 1000; i++) {
        plant_step(&p, &out, 1e-3);
    }
    double want = 10.0 * 1.25 * 0.5;
    CHECK(fabs(p.wheel_rad_s[0] - want) < 0.01 * want);
    CHECK(p.wheel_rad_s[1] == 0.0);
    /* counts = integral of speed: about (1 s - tau) x speed x cpr / 2pi */
    double counts = (1.0 - PLANT_WHEEL_TAU_S) * want * 4096.0 / (2.0 * 3.14159265358979);
    CHECK(fabs(p.wheel_counts[0] - counts) < 0.02 * counts);
    drive_inputs in;
    memset(&in, 0, sizeof in);
    p.enc_flip = 0;
    plant_inputs(&p, &in);
    CHECK_EQ_U(in.wheel_hw[0], (uint16_t)(int64_t)llround(-p.wheel_counts[0]));
}

static void steer_and_lift_integrate(void)
{
    drive_config c = config();
    plant p;
    plant_init(&p, &c);
    drive_outputs out;
    memset(&out, 0, sizeof out);
    out.duty[DRIVE_CH_M] = 0.2f;
    out.duty[DRIVE_CH_H] = 0.5f;
    for (int i = 0; i < 500; i++) {
        plant_step(&p, &out, 1e-3);
    }
    CHECK(fabs(p.steer_rad - 0.2 * PLANT_STEER_RAD_S_PER_DUTY * 0.5) < 1e-6);
    CHECK(fabs(p.lift_m - (0.0223 + 0.5 * PLANT_LIFT_M_S_PER_DUTY * 0.5)) < 1e-6);
}

static void drv_spi_registers(void)
{
    drive_config c = config();
    plant p;
    plant_init(&p, &c);
    uint8_t ch;
    uint16_t rx;
    uint32_t tag;
    CHECK(!plant_spi_take(&p, &ch, &rx, &tag));
    CHECK(plant_spi_start(&p, 2, (uint16_t)((0x0Au << 8) | 0x40u), 7)); /* write CONFIG reg */
    CHECK(plant_spi_take(&p, &ch, &rx, &tag));
    CHECK(ch == 2 && tag == 7 && (rx & 0xC000u) == 0xC000u);
    CHECK(plant_spi_start(&p, 2, (uint16_t)(0x4000u | (0x0Au << 8)), 8)); /* read it back */
    CHECK(plant_spi_take(&p, &ch, &rx, &tag));
    CHECK_EQ_U(rx, 0xC040u);
    CHECK(p.nfault_low[2]);
    CHECK(plant_spi_start(&p, 2, (uint16_t)((DRV8244_REG_COMMAND << 8) | 0x80u), 9)); /* CLR_FLT */
    CHECK(!p.nfault_low[2] && p.nfault_low[1]);
}

int main(void)
{
    kth_round_trip();
    starts_in_range();
    wheels_follow_duty();
    steer_and_lift_integrate();
    drv_spi_registers();
    return FW_TEST_RESULT();
}
