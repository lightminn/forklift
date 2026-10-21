#include "fw/wheel_enc.h"

#define TWO_PI_F 6.28318530717958647692f

static void push(wheel_enc *e, const wheel_enc_config *c, uint32_t now_us)
{
    uint16_t slots = (uint16_t)(c->window + 1u);
    e->head = (uint16_t)((e->head + 1u) % slots);
    e->hist_count[e->head] = e->count;
    e->hist_t_us[e->head] = now_us;
    if (e->filled < slots) {
        e->filled++;
    }
}

bool wheel_enc_init(wheel_enc *e, const wheel_enc_config *c, uint16_t hw, uint32_t now_us)
{
    if (c->counts_per_rev == 0 || c->window == 0 || c->window > WHEEL_ENC_WINDOW_MAX ||
        (c->sign != 1 && c->sign != -1)) {
        return false;
    }
    e->count = 0;
    e->rate_rad_s = 0.0f;
    e->last_hw = hw;
    e->head = 0;
    e->filled = 1;
    e->hist_count[0] = 0;
    e->hist_t_us[0] = now_us;
    return true;
}

void wheel_enc_update(wheel_enc *e, const wheel_enc_config *c, uint16_t hw, uint32_t now_us)
{
    int16_t step = (int16_t)(uint16_t)(hw - e->last_hw); /* modular 16-bit difference */
    e->last_hw = hw;
    int32_t signed_step = c->sign > 0 ? (int32_t)step : -(int32_t)step;
    e->count += (uint32_t)signed_step; /* modular 32-bit accumulation */
    push(e, c, now_us);

    /* Rate between the newest sample and the oldest one kept, by their own time stamps. */
    uint16_t slots = (uint16_t)(c->window + 1u);
    uint16_t oldest = (uint16_t)((e->head + slots + 1u - e->filled) % slots);
    uint32_t dt_us = now_us - e->hist_t_us[oldest];
    int32_t dcount = (int32_t)(e->count - e->hist_count[oldest]);
    e->rate_rad_s =
        dt_us == 0 ? 0.0f
                   : (float)dcount * TWO_PI_F / (float)c->counts_per_rev / ((float)dt_us * 1e-6f);
}
