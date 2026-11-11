/* See plant.h. Moved from firmware/tools/mcu_model.c (board design 9) and generalised to an
   axis's zero offset and sign, so it fits the board's KTH configuration as well as the host
   model's. */
#include "plant.h"

#include <math.h>
#include <string.h>

#include "fw/crc.h"
#include "fw/drv8244.h"

#define TWO_PI 6.28318530717958647692

/* Zero, held 1 % of the span inside the range: a start on the edge could read out of range
   after the sensor's 12-bit quantisation and fault the HIL board at boot. */
static double start_in(const abs_axis_config *a)
{
    double lo = (double)a->min_joint, hi = (double)a->max_joint, m = 0.01 * (hi - lo);
    return 0.0 < lo + m ? lo + m : 0.0 > hi - m ? hi - m : 0.0;
}

void plant_init(plant *p, const drive_config *c)
{
    memset(p, 0, sizeof *p);
    p->wheel_full_rad_s = (double)c->max_wheel_rad_s * 1.25;
    p->counts_per_rev[0] = (double)c->wheel[0].counts_per_rev;
    p->counts_per_rev[1] = (double)c->wheel[1].counts_per_rev;
    p->steer_axis = c->steer;
    p->lift_axis = c->lift;
    p->steer_rad = start_in(&c->steer);
    p->lift_m = start_in(&c->lift);
    p->enc_flip = -1;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        p->nfault_low[ch] = true;
    }
}

static double applied(const drive_outputs *out, int ch)
{
    return out->drvoff[ch] ? 0.0 : (double)out->duty[ch];
}

void plant_step(plant *p, const drive_outputs *out, double dt)
{
    for (int i = 0; i < 2; i++) {
        double target = p->wheel_full_rad_s * applied(out, i);
        p->wheel_rad_s[i] += (target - p->wheel_rad_s[i]) / PLANT_WHEEL_TAU_S * dt;
        p->wheel_counts[i] += p->wheel_rad_s[i] * dt * p->counts_per_rev[i] / TWO_PI;
    }
    p->steer_rad += applied(out, DRIVE_CH_M) * PLANT_STEER_RAD_S_PER_DUTY * dt;
    p->lift_m += applied(out, DRIVE_CH_H) * PLANT_LIFT_M_S_PER_DUTY * dt;
}

uint16_t plant_kth_frame(double joint, const abs_axis_config *axis)
{
    double a = fmod((double)axis->sign * joint * (double)axis->sensor_per_joint +
                        (double)axis->zero_offset_rad,
                    TWO_PI);
    if (a < 0) {
        a += TWO_PI;
    }
    uint16_t angle = (uint16_t)((uint32_t)lround(a * 4096.0 / TWO_PI) & 0x0FFFu);
    return (uint16_t)((angle << 4) | crc4_kth7801(angle, axis->crc));
}

void plant_inputs(const plant *p, drive_inputs *in)
{
    for (int i = 0; i < 2; i++) {
        double c = p->enc_flip == i ? -p->wheel_counts[i] : p->wheel_counts[i];
        in->wheel_hw[i] = (uint16_t)(int64_t)llround(c);
    }
    in->abs_frame[0] = plant_kth_frame(p->steer_rad, &p->steer_axis);
    in->abs_frame[1] = plant_kth_frame(p->lift_m, &p->lift_axis);
    in->abs_fresh[0] = in->abs_fresh[1] = true;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        in->adc_raw[ch] = 0; /* no current */
        in->nfault_low[ch] = p->nfault_low[ch];
    }
    in->adc_fresh = true;
}

bool plant_spi_start(plant *p, uint8_t ch, uint16_t tx, uint32_t tag)
{
    uint8_t addr = (uint8_t)((tx >> 8) & 0x3Fu);
    bool rd = (tx & 0x4000u) != 0;
    if (!rd && addr < PLANT_DRV_REGS) {
        p->regs[ch][addr] = (uint8_t)tx;
        if (addr == DRV8244_REG_COMMAND && (tx & 0x80u)) { /* CLR_FLT */
            p->nfault_low[ch] = false;
        }
    }
    p->spi_done = true;
    p->spi_ch = ch;
    p->spi_tag = tag;
    p->spi_rx = (uint16_t)(0xC000u | ((rd && addr < PLANT_DRV_REGS) ? p->regs[ch][addr] : 0));
    return true;
}

bool plant_spi_take(plant *p, uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    if (!p->spi_done) {
        return false;
    }
    p->spi_done = false;
    *ch = p->spi_ch;
    *rx = p->spi_rx;
    *tag = p->spi_tag;
    return true;
}
