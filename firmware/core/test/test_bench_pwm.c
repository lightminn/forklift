/* Bench image: the motor output report (fw/bench_pwm.h, plan 4.3 A1b-2 decision 6). */
#include "fw/bench_pwm.h"
#include "fw_test.h"

#include <string.h>

static bench_pwm_model m;
static drive_inputs in;
static uint32_t now;

static void apply_m(float duty, bool updated)
{
    drive_outputs o;
    memset(&o, 0, sizeof o);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        o.drvoff[ch] = ch != DRIVE_CH_M;
    }
    o.duty[DRIVE_CH_M] = duty;
    now += 900;
    bench_pwm_model_apply(&m, &o, updated, now);
}

static void cut_m(bool updated)
{
    drive_outputs o;
    memset(&o, 0, sizeof o);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        o.drvoff[ch] = true;
    }
    now += 900;
    bench_pwm_model_apply(&m, &o, updated, now);
}

static void read_m(bool updated)
{
    now += 100;
    memset(&in, 0, sizeof in);
    bench_pwm_read(&m.t, updated, now, &in);
}

static void start(void)
{
    bench_pwm_model_init(&m);
    now = 1000;
    read_m(true);
}

/* +0.4 -> -0.3 with update events: the reversal writes 0 first, the next call the new direction (a new run). */
static void normal_reversal(void)
{
    start();
    apply_m(0.4f, true);
    uint32_t since = now;
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 4000);
    CHECK(in.applied_any[DRIVE_CH_M]);
    CHECK(in.run_valid[DRIVE_CH_M]);
    CHECK_EQ_U(in.run_since_us[DRIVE_CH_M], since);
    CHECK_EQ_I(in.run_sign[DRIVE_CH_M], 1);
    CHECK_EQ_U(in.applied_window_us, 1000);
    apply_m(-0.3f, true); /* the direction change: compare 0, PH unchanged */
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 0);
    CHECK(in.applied_any[DRIVE_CH_M]); /* +4000 was active at the window's start */
    CHECK(!in.run_valid[DRIVE_CH_M]);
    apply_m(-0.3f, true);
    since = now;
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], -3000);
    CHECK(in.run_valid[DRIVE_CH_M]);
    CHECK_EQ_U(in.run_since_us[DRIVE_CH_M], since);
    CHECK_EQ_I(in.run_sign[DRIVE_CH_M], -1);
}

/* +0.4 -> -0.3 -> -0.3 with no update between the last two: the second -0.3 is still the brake's 0 (6th round P1-1). */
static void reentry_without_update(void)
{
    start();
    apply_m(0.4f, true);
    read_m(true);
    apply_m(-0.3f, true);
    read_m(false); /* no update since the 0 was written: the old +4000 may still be active */
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 4000);
    CHECK(in.applied_any[DRIVE_CH_M]);
    CHECK(!in.run_valid[DRIVE_CH_M]);
    apply_m(-0.3f, false);
    read_m(false);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 4000); /* still unknown: the stopping side */
    CHECK(!in.run_valid[DRIVE_CH_M]);            /* the HAL wrote 0 again */
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 0);
}

/* +u -> 0 -> +u with no update: the 0 preload is overwritten, the output never stopped, but the run is new. */
static void zero_overwritten_without_update(void)
{
    start();
    apply_m(0.4f, true);
    uint32_t first = now;
    read_m(true);
    apply_m(0.0f, true);
    read_m(false);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 4000);
    apply_m(0.4f, false);
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 4000);
    CHECK(in.applied_any[DRIVE_CH_M]);
    CHECK(in.run_valid[DRIVE_CH_M]);
    CHECK(in.run_since_us[DRIVE_CH_M] != first); /* written 0 broke the run */
}

/* 0 -> non-zero without an update before the read: the non-zero may be active (6th round P2-1). */
static void first_nonzero_before_its_update(void)
{
    start();
    apply_m(0.0f, true);
    read_m(true);
    CHECK(!in.applied_any[DRIVE_CH_M]);
    apply_m(0.2f, true);
    read_m(false);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 2000);
    CHECK(in.applied_any[DRIVE_CH_M]);
    apply_m(0.0f, true);
    read_m(true); /* the update came: 0 active, but the window held the 2000 */
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 0);
    CHECK(in.applied_any[DRIVE_CH_M]);
    read_m(true);
    CHECK(!in.applied_any[DRIVE_CH_M]);
}

/* A cut acts at once; a rounded-to-zero duty writes 0 and breaks the run. */
static void cut_and_rounding(void)
{
    start();
    apply_m(0.4f, true);
    read_m(true);
    cut_m(true);
    read_m(false);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 0);
    CHECK(in.applied_any[DRIVE_CH_M]);
    CHECK(!in.run_valid[DRIVE_CH_M]);
    start();
    apply_m(0.00004f, true); /* compare 0 */
    read_m(true);
    CHECK_EQ_I(in.applied_e4[DRIVE_CH_M], 0);
    CHECK(!in.applied_any[DRIVE_CH_M]);
    CHECK(!in.run_valid[DRIVE_CH_M]);
}

/* Same-sign writes keep the run's start; the run sign is physical. */
static void run_keeps_its_start(void)
{
    start();
    apply_m(-0.2f, true);
    uint32_t since = now;
    read_m(true);
    for (int i = 0; i < 5; i++) {
        apply_m(-0.1f - 0.01f * (float)i, true);
        read_m(true);
        CHECK(in.run_valid[DRIVE_CH_M]);
        CHECK_EQ_U(in.run_since_us[DRIVE_CH_M], since);
        CHECK_EQ_I(in.run_sign[DRIVE_CH_M], -1);
    }
}

/* A run that starts at HAL time 0 is valid (8th round P2: the flag, not the time, says so). */
static void run_at_time_zero(void)
{
    bench_pwm_model_init(&m);
    now = 0u - 900u;
    apply_m(0.3f, true); /* now wraps to 0 */
    CHECK_EQ_U(now, 0);
    read_m(true);
    CHECK(in.run_valid[DRIVE_CH_M]);
    CHECK_EQ_U(in.run_since_us[DRIVE_CH_M], 0);
}

int main(void)
{
    normal_reversal();
    reentry_without_update();
    zero_overwritten_without_update();
    first_nonzero_before_its_update();
    cut_and_rounding();
    run_keeps_its_start();
    run_at_time_zero();
    return fw_test_failures;
}
