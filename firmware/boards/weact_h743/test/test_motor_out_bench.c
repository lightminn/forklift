/* bench7 (plan 4.3 A1b-2 decision 6): motor_out_tim1.c's report on the host register model equals bench_pwm_model's
   over the same writes and update events -- the model the host endpoint and the tests use is the board's rule. */
#include <string.h>

#include "board/board.h"
#include "board/board_hal.h"
#include "fw/bench_pwm.h"
#include "fw_test.h"
#include "regmodel.h"

static uint32_t g_now = 1000;
static bool g_uev_on_clock; /* an update event when the clock is read (between a report's UIF read and its clock) */

uint32_t timebase_now_us(void)
{
    if (g_uev_on_clock) {
        g_uev_on_clock = false;
        regmodel_uev();
    }
    return g_now;
}

static uint32_t lcg = 12345u;

static uint32_t next(void)
{
    lcg = lcg * 1103515245u + 12345u;
    return lcg >> 16;
}

static void same_report(const drive_inputs *a, const drive_inputs *b, int step)
{
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (a->applied_e4[ch] != b->applied_e4[ch] || a->applied_any[ch] != b->applied_any[ch] ||
            a->run_valid[ch] != b->run_valid[ch] || (a->run_valid[ch] && a->run_since_us[ch] != b->run_since_us[ch]) ||
            a->run_sign[ch] != b->run_sign[ch]) {
            fprintf(stderr, "step %d ch %d: hal %d/%d/%d/%u/%d model %d/%d/%d/%u/%d\n", step, ch, a->applied_e4[ch],
                    a->applied_any[ch], a->run_valid[ch], (unsigned)a->run_since_us[ch], a->run_sign[ch],
                    b->applied_e4[ch], b->applied_any[ch], b->run_valid[ch], (unsigned)b->run_since_us[ch],
                    b->run_sign[ch]);
            fw_test_failures++;
            return;
        }
    }
    CHECK_EQ_U(a->applied_window_us, b->applied_window_us);
}

static void hal_equals_model(void)
{
    static const float duties[] = {0.0f, 0.3f, -0.3f, 0.00004f, 0.5f, -0.12f, 1.2f};
    regmodel_reset();
    motor_out_safe_init();
    motor_out_wake_drivers();
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        regmodel_allow(ch, RM_ALLOW_FWD | RM_ALLOW_REV);
    }
    regmodel_uev();
    bench_pwm_model m;
    bench_pwm_model_init(&m);
    bool pending = true; /* the wake's update since the model's start */
    drive_inputs a, b;
    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);
    g_now += 100;
    motor_out_report(&a);
    bench_pwm_read(&m.t, pending, g_now, &b);
    for (int step = 0; step < 4000; step++) {
        if (next() % 4u != 0u) { /* an update event before the apply (most cycles) */
            regmodel_uev();
            pending = true;
        }
        drive_outputs o;
        memset(&o, 0, sizeof o);
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            uint32_t r = next();
            o.drvoff[ch] = r % 9u == 0u;
            o.duty[ch] = duties[(r >> 4) % (sizeof duties / sizeof duties[0])];
        }
        g_now += 900;
        motor_out_apply(&o);
        bench_pwm_model_apply(&m, &o, pending, g_now);
        pending = false;
        bool upd = next() % 3u != 0u; /* an update event between the apply and the read */
        if (upd) {
            regmodel_uev();
            pending = true;
        }
        g_now += 100;
        memset(&a, 0, sizeof a);
        memset(&b, 0, sizeof b);
        motor_out_report(&a);
        bench_pwm_read(&m.t, upd, g_now, &b);
        same_report(&a, &b, step);
        if (fw_test_failures) {
            return;
        }
    }
}

/* The register model's active output of a channel, physical sign x compare value (0 unless FWD or REV). */
static int active_of(int ch)
{
    rm_channel c = regmodel_channel(ch);
    int v = (int)(c.duty * 10000.0 + 0.5);
    return c.state == RM_FWD ? v : c.state == RM_REV ? -v : 0;
}

/* The stopping side against the register model itself (not the shared report code): an update injected after any
   write of a call, or none, between writes of 0 and of a first non-zero; whenever the output is non-zero at a read
   the report says non-zero of that sign, and a window that began or ended non-zero reports applied_any (plan 4.3
   decision 6, the four CCR 0 / first non-zero before/after cases; Codex A1b-2a round 1). */
static void report_never_misses_an_active_output(void)
{
    static const float duties[] = {0.0f, 0.3f, -0.3f, 0.5f};
    regmodel_reset();
    motor_out_safe_init();
    motor_out_wake_drivers();
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        regmodel_allow(ch, RM_ALLOW_FWD | RM_ALLOW_REV);
    }
    regmodel_uev();
    drive_inputs r;
    memset(&r, 0, sizeof r);
    motor_out_report(&r);
    int before = active_of(DRIVE_CH_M);
    for (int step = 0; step < 4000; step++) {
        drive_outputs o;
        memset(&o, 0, sizeof o);
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            o.drvoff[ch] = ch != DRIVE_CH_M;
        }
        uint32_t x = next();
        o.duty[DRIVE_CH_M] = duties[x % 4u];
        uint32_t when = (x >> 3) % 14u; /* 0: none; 1..12: after that write of the call (L and R write twice each: M's from 5); 13: before */
        if (when == 13u) {
            regmodel_uev();
        } else if (when != 0u) {
            regmodel_uev_after_write((int)when);
        }
        g_now += 900;
        motor_out_apply(&o);
        regmodel_uev_after_write(0);
        int mid = active_of(DRIVE_CH_M);
        if ((x >> 6) % 3u == 0u) {
            regmodel_uev(); /* an update between the call and the read */
        }
        g_now += 100;
        memset(&r, 0, sizeof r);
        motor_out_report(&r);
        int now_active = active_of(DRIVE_CH_M);
        int rep = r.applied_e4[DRIVE_CH_M];
        if (now_active != 0 && (rep == 0 || (rep > 0) != (now_active > 0))) {
            fprintf(stderr, "step %d: active %d reported %d\n", step, now_active, rep);
            fw_test_failures++;
            return;
        }
        if ((before != 0 || mid != 0 || now_active != 0) && !r.applied_any[DRIVE_CH_M]) {
            fprintf(stderr, "step %d: a non-zero window reported none (%d %d %d)\n", step, before, mid, now_active);
            fw_test_failures++;
            return;
        }
        before = now_active;
    }
}

/* an update between the report's UIF read and its clock: the report must not call 0 active before its time --
   +4000 active, 0 written, the update lands on the clock read; the report says 4000 and the next window is non-zero
   (with the clock read first the update would read as one before the time; Codex A1b-2a round 1 P1-1, round 2) */
static void update_between_uif_and_clock(void)
{
    regmodel_reset();
    motor_out_safe_init();
    motor_out_wake_drivers();
    regmodel_allow(DRIVE_CH_M, RM_ALLOW_FWD | RM_ALLOW_REV);
    regmodel_uev();
    drive_outputs o;
    memset(&o, 0, sizeof o);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        o.drvoff[ch] = ch != DRIVE_CH_M;
    }
    drive_inputs r;
    o.duty[DRIVE_CH_M] = 0.4f;
    g_now += 900;
    motor_out_apply(&o);
    regmodel_uev();
    g_now += 100;
    motor_out_report(&r);
    CHECK_EQ_I(r.applied_e4[DRIVE_CH_M], 4000);
    o.duty[DRIVE_CH_M] = 0.0f;
    g_now += 900;
    motor_out_apply(&o);
    g_now += 100;
    g_uev_on_clock = true;
    memset(&r, 0, sizeof r);
    motor_out_report(&r);
    CHECK_EQ_I(r.applied_e4[DRIVE_CH_M], 4000);
    g_now += 1000;
    memset(&r, 0, sizeof r);
    motor_out_report(&r);
    CHECK(r.applied_any[DRIVE_CH_M]);
}

int main(void)
{
    hal_equals_model();
    report_never_misses_an_active_output();
    update_between_uif_and_clock();
    return fw_test_failures;
}
