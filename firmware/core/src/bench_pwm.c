#include "fw/bench_pwm.h"

#if defined(FW_CORE_BENCH)

#include <string.h>

void bench_pwm_track_reset(bench_pwm_track *t)
{
    memset(t, 0, sizeof *t);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        t->cut[ch] = true;
    }
}

void bench_pwm_wrote(bench_pwm_track *t, const int16_t w[DRIVE_CH_COUNT], const bool cut[DRIVE_CH_COUNT],
                     uint32_t now_after)
{
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        int16_t v = cut[ch] ? 0 : w[ch];
        t->w[ch] = v;
        t->cut[ch] = cut[ch];
        if (v != 0) {
            int8_t dir = v > 0 ? 1 : -1;
            t->wnz[ch] = true;
            if (!t->run_valid[ch] || t->run_sign[ch] != dir) {
                t->run_valid[ch] = true;
                t->run_since[ch] = now_after;
                t->run_sign[ch] = dir;
            }
        } else {
            t->run_valid[ch] = false; /* a 0 of any cause, or a cut, breaks the run */
        }
    }
}

void bench_pwm_read(bench_pwm_track *t, bool updated, uint32_t now, drive_inputs *in)
{
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        int16_t act;
        if (t->cut[ch]) {
            act = 0; /* DRVOFF acts at once */
        } else if (updated) {
            act = t->w[ch]; /* an update after the last write: what it wrote is active */
        } else {
            act = t->w[ch] != 0 ? t->w[ch] : t->active[ch]; /* unknown: the non-zero one (the stopping side) */
        }
        in->applied_e4[ch] = act;
        in->applied_any[ch] = t->active[ch] != 0 || t->wnz[ch] || act != 0;
        in->run_valid[ch] = t->run_valid[ch];
        in->run_since_us[ch] = t->run_since[ch];
        in->run_sign[ch] = t->run_valid[ch] ? t->run_sign[ch] : 0;
        t->active[ch] = act;
        t->wnz[ch] = false;
    }
    in->applied_window_us = t->have_read ? now - t->last_read : 0u;
    in->pwm_report = true;
    t->last_read = now;
    t->have_read = true;
}

void bench_pwm_model_init(bench_pwm_model *m)
{
    memset(m, 0, sizeof *m);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        m->ph[ch] = -1;
        m->cut[ch] = true;
    }
    bench_pwm_track_reset(&m->t);
}

static void model_set_ccr(bench_pwm_model *m, int ch, uint32_t v)
{
    if (v == 0u && m->ccr[ch] != 0u) {
        m->zero_pending[ch] = true;
    }
    m->ccr[ch] = v;
}

void bench_pwm_model_apply(bench_pwm_model *m, const drive_outputs *o, bool updated, uint32_t now_after)
{
    int16_t w[DRIVE_CH_COUNT];
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (updated) {
            m->zero_pending[ch] = false;
        }
    }
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (o->drvoff[ch] || (m->cut[ch] && m->zero_pending[ch])) {
            m->cut[ch] = true;
            model_set_ccr(m, ch, 0u);
            w[ch] = 0;
            continue;
        }
        float d = o->duty[ch];
        float mag = d < 0.0f ? -d : d;
        uint32_t ccr = mag >= 1.0f ? BENCH_PWM_TOP : (uint32_t)(mag * (float)BENCH_PWM_TOP + 0.5f);
        int8_t dir = d < 0.0f ? -1 : 1;
        if (ccr == 0u) {
            model_set_ccr(m, ch, 0u);
        } else if (dir != m->ph[ch] && (m->ccr[ch] != 0u || m->zero_pending[ch])) {
            model_set_ccr(m, ch, 0u); /* direction change: brake until the 0 is active */
        } else {
            if (dir != m->ph[ch]) {
                m->ph[ch] = dir;
            }
            model_set_ccr(m, ch, ccr);
        }
        m->cut[ch] = false;
        w[ch] = (int16_t)((int32_t)m->ph[ch] * (int32_t)m->ccr[ch]);
    }
    bench_pwm_wrote(&m->t, w, m->cut, now_after);
}

#endif /* FW_CORE_BENCH */
