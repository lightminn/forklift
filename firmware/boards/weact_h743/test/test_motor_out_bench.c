/* bench7 (plan 4.3 A1b-2 decision 6): motor_out_tim1.c's report on the host register model equals bench_pwm_model's
   over the same writes and update events -- the model the host endpoint and the tests use is the board's rule. */
#include <string.h>

#include "board/board.h"
#include "board/board_hal.h"
#include "fw/bench_pwm.h"
#include "fw_test.h"
#include "regmodel.h"

static uint32_t g_now = 1000;

uint32_t timebase_now_us(void)
{
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
    motor_out_report(g_now, &a);
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
        motor_out_report(g_now, &a);
        bench_pwm_read(&m.t, upd, g_now, &b);
        same_report(&a, &b, step);
        if (fw_test_failures) {
            return;
        }
    }
}

int main(void)
{
    hal_equals_model();
    return fw_test_failures;
}
