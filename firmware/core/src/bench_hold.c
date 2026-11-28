/* Bench image: a hold run's target, control and progress watch (fw/bench_hold.h, plan 4.3 A1b-2 decisions 2-7). */
#include "fw/bench_hold.h"

#if defined(FW_CORE_BENCH)

#include <math.h>
#include <string.h>

#include "fw/bench_kth.h"
#include "fw/drive.h"

#define TWO_PI_F 6.28318530717958647692f
#define RAD_PER_COUNT (TWO_PI_F / (float)BENCH_HOLD_CPR)

bool bench_hold_values_ok(const link_bench_hold *c, uint16_t axis_duty_max_e4)
{
    if (!isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd)) {
        return false;
    }
    if (!(c->kp > 0.0f && c->kp <= 100.0f) || !(c->ki >= 0.0f && c->ki <= 1000.0f) ||
        !(c->kd >= 0.0f && c->kd <= 10.0f)) {
        return false;
    }
    if (c->duty_max_e4 < 1u || c->duty_max_e4 > axis_duty_max_e4 || c->min_duty_e4 > c->duty_max_e4) {
        return false;
    }
    if (c->valid_after_echo_ms == 0 || c->cpr != BENCH_HOLD_CPR || c->deadband_counts > 512u ||
        c->target_kind > BENCH_HOLD_TARGET_REL) {
        return false;
    }
    if (c->target_kind == BENCH_HOLD_TARGET_ABS && (c->target_value < 0 || c->target_value > 4095)) {
        return false;
    }
    if (c->target_kind == BENCH_HOLD_TARGET_REL &&
        (c->target_value <= -(int32_t)(1 << 30) || c->target_value >= (int32_t)(1 << 30))) {
        return false;
    }
    if ((c->target_kind == BENCH_HOLD_TARGET_ZERO || c->target_kind == BENCH_HOLD_TARGET_HERE) &&
        c->target_value != 0) {
        return false;
    }
    return true;
}

/* The shortest way from raw a to raw b on 4096 counts: exactly half a turn is -2048 (the counter's convention). */
static int32_t shortest(uint16_t from, uint16_t to)
{
    int32_t d = ((int32_t)to - (int32_t)from) & 0xFFF;
    return d >= 2048 ? d - 4096 : d;
}

uint32_t bench_hold_target(uint8_t kind, int32_t value, uint16_t raw, uint32_t cum)
{
    switch (kind) {
    case BENCH_HOLD_TARGET_ZERO:
        return cum + (uint32_t)shortest(raw, 0);
    case BENCH_HOLD_TARGET_ABS:
        return cum + (uint32_t)shortest(raw, (uint16_t)value);
    case BENCH_HOLD_TARGET_REL:
        return cum + (uint32_t)value;
    default:
        return cum; /* here */
    }
}

static uint32_t abs_err(const bench_hold *h, uint32_t cum)
{
    int32_t e = bench_kth_diff(h->target, cum);
    return e < 0 ? (uint32_t)(-(int64_t)e) : (uint32_t)e;
}

void bench_hold_start(bench_hold *h, const link_bench_hold *c, uint16_t raw, uint32_t cum, uint64_t trust_t,
                      bool lo_known, uint64_t lo)
{
    memset(h, 0, sizeof *h);
    h->cfg = *c;
    h->target = bench_hold_target(c->target_kind, c->target_value, raw, cum);
    h->rate_have = true;
    h->rate_cum = cum;
    h->rate_t = trust_t;
    h->r = abs_err(h, cum);
    h->in_band = h->r <= c->deadband_counts;
    h->prev_lo_known = lo_known;
    h->prev_lo = lo;
    h->prev_run_valid = false; /* no output before the run's first HOLD */
}

bool bench_hold_same(const bench_hold *h, const link_bench_hold *c)
{
    const link_bench_hold *a = &h->cfg;
    return a->run_id == c->run_id && a->axis == c->axis && a->source == c->source &&
           a->target_kind == c->target_kind && a->target_value == c->target_value && a->cpr == c->cpr &&
           memcmp(&a->kp, &c->kp, sizeof a->kp) == 0 && memcmp(&a->ki, &c->ki, sizeof a->ki) == 0 &&
           memcmp(&a->kd, &c->kd, sizeof a->kd) == 0 && a->deadband_counts == c->deadband_counts &&
           a->duty_max_e4 == c->duty_max_e4 && a->min_duty_e4 == c->min_duty_e4 && a->sign == c->sign;
}

float bench_hold_control(bench_hold *h, bool usable, bool trusted_new, uint32_t trust_cum, uint64_t trust_t,
                         float dt_s)
{
    if (!usable) {
        return 0.0f; /* a tentative 0x0000, no count or a loss: no output and no state update (decision 5) */
    }
    if (trusted_new) {
        /* the rate on the samples' own acquisition times, never the consuming ticks' (independent review P1-4) */
        if (h->rate_have && trust_t > h->rate_t) {
            float dt = (float)(trust_t - h->rate_t);
            float raw = (float)bench_kth_diff(trust_cum, h->rate_cum) * RAD_PER_COUNT / (dt * 1e-6f);
            float alpha = dt / (BENCH_HOLD_TAU_US + dt);
            h->rate += alpha * (raw - h->rate);
        }
        h->rate_have = true;
        h->rate_cum = trust_cum;
        h->rate_t = trust_t;
    }
    const link_bench_hold *c = &h->cfg;
    int32_t e = bench_kth_diff(h->target, trust_cum);
    uint32_t ae = e < 0 ? (uint32_t)(-(int64_t)e) : (uint32_t)e;
    if (ae <= c->deadband_counts) {
        return 0.0f; /* inside the deadband: no minimum duty, no D, the integral kept as it is */
    }
    float cap = (float)c->duty_max_e4 / 10000.0f;
    float er = (float)e * RAD_PER_COUNT;
    float integ = h->integ + c->ki * er * dt_s;
    if (!isfinite(integ) || !isfinite(h->rate)) {
        h->integ = 0.0f; /* checked before the clamp turns an infinity into the cap (Codex A1b-2a round 1) */
        return 0.0f;
    }
    integ = integ > cap ? cap : (integ < -cap ? -cap : integ);
    float q = c->kp * er + integ - c->kd * h->rate;
    if (!isfinite(q)) {
        h->integ = 0.0f;
        return 0.0f;
    }
    h->integ = integ;
    float min = (float)c->min_duty_e4 / 10000.0f;
    bool toward = (q > 0.0f && e > 0) || (q < 0.0f && e < 0);
    if (toward && fabsf(q) < min) {
        q = q > 0.0f ? min : -min; /* the minimum duty only toward the error (independent review P1-4) */
    }
    q = q > cap ? cap : (q < -cap ? -cap : q);
    return (float)c->sign * q;
}

void bench_hold_count_entry(bench_hold *h, bool trusted_new, uint32_t trust_cum)
{
    if (!trusted_new) {
        return;
    }
    bool in = abs_err(h, trust_cum) <= h->cfg.deadband_counts;
    if (in && !h->in_band) {
        h->entries++;
    }
    h->in_band = in;
}

uint8_t bench_hold_watch(bench_hold *h, const bench_hold_tick *t)
{
    uint32_t db = h->cfg.deadband_counts;
    uint32_t now_err = abs_err(h, t->last_zero ? t->cum : t->trust_cum); /* tentative when the last frame was 0 */
    /* a reported window covers the unreported ticks since the last report: only what they did not already add counts
       (Codex A1b-2a rounds 3-4) */
    uint64_t wt = t->window_us, wz = t->window_us;
    if (t->reported) {
        wt = wt > h->fb_timer_us ? wt - h->fb_timer_us : 0u;
        wz = wz > h->fb_zr_us ? wz - h->fb_zr_us : 0u;
        h->fb_timer_us = h->fb_zr_us = 0;
    }
    /* time: the window when a non-zero may have been active or the error is outside the deadband (decision 7) */
    if (t->applied_any || now_err > db) {
        h->timer_us += wt;
        if (!t->reported) {
            h->fb_timer_us += wt;
        }
    }
    if (t->last_zero) {
        if (t->applied_any) {
            h->zr_us += wz;
            if (!t->reported) {
                h->fb_zr_us += wz;
            }
        }
    } else {
        h->zr_us = 0;
        h->fb_zr_us = 0;
    }
    if (t->trusted_new) {
        uint32_t a = abs_err(h, t->trust_cum);
        int32_t e = bench_kth_diff(h->target, t->trust_cum);
        bool in = a <= db;
        h->trusted_seen = true;
        bool eligible = t->run_valid && t->lo_known && h->prev_lo_known && h->prev_run_valid &&
                        h->prev_run_since == t->run_since && h->prev_lo >= t->run_since &&
                        t->lo >= t->run_since + BENCH_HOLD_ELIGIBLE_US && e != 0 &&
                        (e > 0 ? 1 : -1) == t->run_dir;
        if (in) {
            h->timer_us = h->fb_timer_us = 0; /* 1: the target reached, eligible or not */
            h->r = a;
            h->trusted_seen = false;
        } else if (!eligible) {
            if (a < h->r) {
                h->r = a; /* 2: lower only */
            }
        } else if ((int64_t)a <= (int64_t)h->r - BENCH_HOLD_R) {
            h->timer_us = h->fb_timer_us = 0; /* 3 */
            h->r = a;
            h->trusted_seen = false;
        } else if ((int64_t)a >= (int64_t)h->r + 2 * BENCH_HOLD_R) {
            h->r = a; /* 4: moved away; the return will count */
        }
        h->prev_lo_known = t->lo_known;
        h->prev_lo = t->lo;
        h->prev_run_valid = t->run_valid;
        h->prev_run_since = t->run_since;
    }
    if (h->zr_us > BENCH_HOLD_ZERO_RUN_US) {
        return DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT;
    }
    if (h->timer_us >= BENCH_HOLD_WATCH_US) {
        return h->trusted_seen ? DRIVE_BENCH_STOP_NO_PROGRESS : DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT;
    }
    return 0;
}

#endif /* FW_CORE_BENCH */
