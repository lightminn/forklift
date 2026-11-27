/* Bench image: a hold run's target, control and watch (fw/bench_hold.h, plan 4.3 A1b-2 decisions 2-7). */
#include "fw/bench_hold.h"
#include "fw/bench_kth.h"
#include "fw/drive.h"
#include "fw_test.h"

#include <math.h>
#include <string.h>

static link_bench_hold cfg_default(void)
{
    link_bench_hold c = {.boot_id = 1,
                         .arm_gen = 1,
                         .valid_after_echo_ms = 100,
                         .run_id = 7,
                         .axis = DRIVE_CH_M,
                         .source = 1,
                         .target_kind = BENCH_HOLD_TARGET_HERE,
                         .cpr = 4096,
                         .kp = 0.5f,
                         .deadband_counts = 12,
                         .duty_max_e4 = 1500,
                         .min_duty_e4 = 1200,
                         .sign = 1};
    return c;
}

static void targets(void)
{
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_HERE, 0, 100, 5000u), 5000u);
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_ZERO, 0, 100, 5000u), 4900u);
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_ZERO, 0, 4000, 5000u), 5096u);
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_ZERO, 0, 2048, 5000u), 5000u - 2048u); /* half a turn: -2048 */
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_ABS, 1, 4095, 0u), 2u);                 /* 4095 -> 1 = +2 */
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_REL, 1, 3, 0x7fffffffu), 0x80000000u);   /* modular */
    CHECK_EQ_U(bench_hold_target(BENCH_HOLD_TARGET_REL, 4096, 3, 10u), 4106u);              /* a full turn stays */
    CHECK_EQ_I(bench_kth_diff(0x80000000u, 0x7fffffffu), 1);
}

static void value_checks(void)
{
    link_bench_hold c = cfg_default();
    CHECK(bench_hold_values_ok(&c, 1500));
    CHECK(!bench_hold_values_ok(&c, 1499)); /* above the axis limit */
    c.min_duty_e4 = 1501;
    CHECK(!bench_hold_values_ok(&c, 2000));
    c = cfg_default();
    c.kp = 0.0f;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.ki = 2e38f;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.kd = NAN;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.cpr = 4095;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.deadband_counts = 513;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.target_kind = BENCH_HOLD_TARGET_ABS;
    c.target_value = 4096;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c.target_value = 4095;
    CHECK(bench_hold_values_ok(&c, 1500));
    c.target_kind = BENCH_HOLD_TARGET_REL;
    c.target_value = 1 << 30;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c.target_value = -(1 << 30) + 1;
    CHECK(bench_hold_values_ok(&c, 1500));
    c.target_kind = 4;
    CHECK(!bench_hold_values_ok(&c, 1500));
    c = cfg_default();
    c.valid_after_echo_ms = 0;
    CHECK(!bench_hold_values_ok(&c, 1500));
}

static bench_hold h;

static void start_at(link_bench_hold c, uint32_t cum, uint32_t target_offset)
{
    c.target_kind = BENCH_HOLD_TARGET_REL;
    c.target_value = (int32_t)target_offset;
    bench_hold_start(&h, &c, 100, cum, 0, true, 0);
}

/* the deadband first (no minimum duty, no D), then PI - D, the minimum duty toward the error only, the cap last */
static void control_order(void)
{
    link_bench_hold c = cfg_default();
    start_at(c, 0, 12);
    CHECK(bench_hold_control(&h, true, false, 0, 0, 0.001f) == 0.0f); /* e 12 = deadband: 0 (<=) */
    start_at(c, 0, 13);
    CHECK(fabsf(bench_hold_control(&h, true, false, 0, 0, 0.001f) - 0.12f) < 1e-6f); /* raised to the minimum */
    start_at(c, 0, 400);
    CHECK(fabsf(bench_hold_control(&h, true, false, 0, 0, 0.001f) - 0.15f) < 1e-6f); /* capped */
    start_at(c, 0, (uint32_t)-13);
    CHECK(fabsf(bench_hold_control(&h, true, false, 0, 0, 0.001f) + 0.12f) < 1e-6f);
    c.sign = -1;
    start_at(c, 0, 13);
    CHECK(fabsf(bench_hold_control(&h, true, false, 0, 0, 0.001f) + 0.12f) < 1e-6f); /* request space */
    /* a tentative frame (usable false): no output, no state */
    start_at(cfg_default(), 0, 400);
    CHECK(bench_hold_control(&h, false, false, 0, 0, 0.001f) == 0.0f);
    CHECK(h.integ == 0.0f);
}

/* the D term against the minimum duty: a small q opposite to e is not raised (independent review P1-4) */
static void min_duty_only_toward_the_error(void)
{
    link_bench_hold c = cfg_default();
    c.kd = 0.037f;
    start_at(c, 0, 13);
    /* moving fast toward the target: D brakes harder than P drives */
    float d1 = bench_hold_control(&h, true, true, 2, 1000, 0.001f); /* 2 counts in 1 ms */
    CHECK(d1 <= 0.0f && d1 > -0.12f);
    start_at(c, 0, 13);
    float d2 = bench_hold_control(&h, true, true, 2, 2000, 0.001f); /* the same 2 counts over 2 ms: half the rate */
    CHECK(d2 != -0.12f);
    /* the rate is per acquisition time: 2 counts over 1 ms vs 2 ms */
    start_at(c, 0, 400);
    (void)bench_hold_control(&h, true, true, 2, 1000, 0.001f);
    float r1 = h.rate;
    start_at(c, 0, 400);
    (void)bench_hold_control(&h, true, true, 2, 2000, 0.001f);
    float r2 = h.rate;
    float raw1 = 2.0f * 6.2831853f / 4096.0f / 0.001f, raw2 = raw1 / 2.0f;
    CHECK(fabsf(r1 - raw1 * (1000.0f / 11000.0f)) < 1e-3f);
    CHECK(fabsf(r2 - raw2 * (2000.0f / 12000.0f)) < 1e-3f);
}

static void control_non_finite_resets(void)
{
    link_bench_hold c = cfg_default();
    c.ki = 1000.0f;
    start_at(c, 0, 400);
    h.integ = INFINITY;
    CHECK(bench_hold_control(&h, true, false, 0, 0, 0.001f) == 0.0f || isfinite(h.integ));
    CHECK(isfinite(h.integ));
}

/* ---- watch -------------------------------------------------------------------------------------------- */
static uint64_t now_us;
static uint64_t run_since;
static bool run_on;
static int8_t run_dir;

/* one tick with a trusted sample a acquired at the tick's start; out: the output was non-zero this window */
static uint8_t wtick(int32_t e, bool out, bool trusted, bool zero_frame)
{
    now_us += 1000;
    bench_hold_tick t;
    memset(&t, 0, sizeof t);
    t.window_us = 1000;
    t.applied_any = out;
    t.last_zero = zero_frame;
    t.cum = h.target - (uint32_t)e;
    t.trusted_new = trusted && !zero_frame;
    t.trust_cum = h.target - (uint32_t)e;
    t.lo_known = true;
    t.lo = now_us;
    t.run_valid = run_on;
    t.run_since = run_since;
    t.run_dir = run_dir;
    return bench_hold_watch(&h, &t);
}

static void wstart(uint32_t err, uint16_t db)
{
    link_bench_hold c = cfg_default();
    c.deadband_counts = db;
    start_at(c, 0, err);
    now_us = 0;
    run_on = false;
}

static void drive_on(int8_t dir)
{
    run_on = true;
    run_since = now_us + 10; /* written just after this tick started */
    run_dir = dir;
}

static int run_until_stop(int32_t (*err)(int), bool (*out)(int), int n, uint8_t *why)
{
    for (int k = 0; k < n; k++) {
        *why = wtick(err(k), out(k), true, false);
        if (*why) {
            return k;
        }
    }
    return -1;
}

static int32_t steady(int k)
{
    return 400 - k;
}
static bool always(int k)
{
    (void)k;
    return true;
}

static void watch_steady_progress(void)
{
    wstart(400, 12);
    drive_on(1);
    uint8_t why;
    CHECK_EQ_I(run_until_stop(steady, always, 380, &why), -1);
}

static void watch_stalled_outside(void)
{
    wstart(30, 12);
    drive_on(1);
    int n = 0;
    uint8_t why = 0;
    while (!why && n < 400) {
        why = wtick(30, true, true, false);
        n++;
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 299 && n <= 301);
}

/* kp 0.001: the request rounds to compare 0, applied_any stays false; outside the deadband the time still runs
   (independent review round 4 P1-3) */
static void watch_rounded_zero_outside(void)
{
    wstart(13, 12);
    int n = 0;
    uint8_t why = 0;
    while (!why && n < 400) {
        why = wtick(13, false, true, false);
        n++;
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 299 && n <= 301);
}

/* inside the deadband with output 0: seconds without a stop (lead's request) */
static void watch_rest_in_band(void)
{
    wstart(5, 12);
    for (int k = 0; k < 3000; k++) {
        CHECK_EQ_U(wtick(5, false, true, false), 0);
    }
}

/* the reported value toggles 99/101 every 4 ms, D reverses each time: no real motion -> 300 ms (8th round P1-1) */
static void watch_noise_with_reversals(void)
{
    wstart(4, 0);
    uint8_t why = 0;
    int k = 0;
    for (; k < 400 && !why; k++) {
        int32_t e = (k / 4) % 2 == 0 ? 4 : 2;
        if (k % 4 == 0) {
            run_on = false; /* the reversal's brake write */
        } else if (k % 4 == 1) {
            drive_on(e > 0 ? 1 : -1);
        }
        why = wtick(e, true, true, false);
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(k >= 299 && k <= 302);
}

/* the lift sags 12 <-> 13 and comes back fast: the in-band sample resets even when not eligible (8th round P1-2) */
static void watch_fast_sag(void)
{
    wstart(12, 12);
    for (int i = 0; i < 200; i++) {
        CHECK_EQ_U(wtick(13, false, true, false), 0); /* sagged; the output starts after this sample */
        drive_on(1);
        CHECK_EQ_U(wtick(12, true, true, false), 0); /* back already: the run's first sample, not eligible */
        run_on = false;                               /* inside the deadband: output 0 at once */
        for (int j = 0; j < 8; j++) {
            CHECK_EQ_U(wtick(12, false, true, false), 0);
        }
    }
    CHECK_EQ_U(h.entries, 200);
}

/* 6th round: 20 -> 19, braked to 11, disturbed to 15 with output 0, resumed and fixed at 15: no reset */
static void watch_resume_after_brake(void)
{
    wstart(20, 0);
    drive_on(1);
    CHECK_EQ_U(wtick(20, true, true, false), 0);
    CHECK_EQ_U(wtick(19, true, true, false), 0);
    run_on = false;
    CHECK_EQ_U(wtick(11, true, true, false), 0);
    CHECK_EQ_U(wtick(15, false, true, false), 0);
    drive_on(1);
    int n = 0;
    uint8_t why = 0;
    while (!why && n < 400) {
        why = wtick(15, true, true, false);
        n++;
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n <= 300);
}

/* a disturbance away (13 -> 17) then a driven return to 15 counts */
static void watch_disturbance_return(void)
{
    wstart(13, 0);
    drive_on(1);
    CHECK_EQ_U(wtick(13, true, true, false), 0);
    CHECK_EQ_U(wtick(17, true, true, false), 0);
    CHECK_EQ_U(h.r, 17);
    CHECK_EQ_U(wtick(15, true, true, false), 0);
    CHECK(h.timer_us == 0);
}

/* 298 ms, then an output-0 tick enters the deadband, then a sag drives 2 ms: the timer reads 2 ms (13th round) */
static void watch_band_entry_resets(void)
{
    wstart(30, 12);
    drive_on(1);
    for (int k = 0; k < 298; k++) {
        CHECK_EQ_U(wtick(30, true, true, false), 0);
    }
    run_on = false;
    CHECK_EQ_U(wtick(12, false, true, false), 0);
    drive_on(1);
    CHECK_EQ_U(wtick(13, true, true, false), 0);
    CHECK_EQ_U(wtick(13, true, true, false), 0);
    CHECK_EQ_U(h.timer_us, 2000);
}

/* the previous trusted sample before the run: not eligible; 40, two CRC losses, 36 on the new run's first sample */
static void watch_previous_sample_outside_the_run(void)
{
    wstart(40, 0);
    CHECK_EQ_U(wtick(40, true, true, false), 0);
    run_on = false;
    (void)wtick(40, true, false, false); /* frames missing */
    (void)wtick(40, true, false, false);
    drive_on(1);
    CHECK_EQ_U(wtick(36, true, true, false), 0);
    CHECK_EQ_U(h.r, 36);
    CHECK(h.timer_us > 0); /* lowered, not reset */
}

/* the previous sample counted with the run valid but acquired before the run's first write: not eligible */
static void watch_previous_sample_before_the_write(void)
{
    wstart(30, 12);
    drive_on(1);
    CHECK_EQ_U(wtick(30, true, true, false), 0);
    run_since = now_us - 500; /* the same run, its first write after the previous sample's bundle start */
    h.prev_run_since = run_since;
    h.prev_lo = run_since - 1;
    CHECK_EQ_U(wtick(27, true, true, false), 0);
    CHECK(h.timer_us > 0);
    CHECK_EQ_U(h.r, 27);
}

/* a 2-tick correction: run written at +20 us, bundles at +20 and +1020 us -> the second sample is eligible */
static void watch_two_tick_correction(void)
{
    wstart(30, 12);
    now_us = 0;
    run_on = true;
    run_since = 1000 - 20; /* the next tick's bundle starts 20 us after the write */
    run_dir = 1;
    CHECK_EQ_U(wtick(30, true, true, false), 0); /* lo 1000 >= run_since but < run_since + 200 */
    uint64_t before = h.timer_us;
    CHECK(before > 0);
    CHECK_EQ_U(wtick(27, true, true, false), 0); /* eligible: 27 <= 30 - 2 */
    CHECK_EQ_U(h.timer_us, 0);
}

/* the 200 us eligibility: a sample less than 200 us after the run's first write lowers only */
static void watch_eligibility_window(void)
{
    wstart(30, 12);
    now_us = 0;
    CHECK_EQ_U(wtick(30, true, true, false), 0);
    run_on = true;
    run_since = now_us + 1000 - 150; /* 150 us before the next bundle */
    run_dir = 1;
    CHECK_EQ_U(wtick(27, true, true, false), 0);
    CHECK(h.timer_us > 0); /* the previous sample was before the run: not eligible anyway */
    CHECK_EQ_U(h.r, 27);
}

/* a 0x0000-only run: withheld output still counts outside the deadband, and the stop is 11 (independent round 2) */
static void watch_zero_frames_outside(void)
{
    wstart(16, 12);
    int n = 0;
    uint8_t why = 0;
    while (!why && n < 400) {
        why = wtick(16, false, false, true);
        n++;
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT);
    CHECK(n >= 299 && n <= 301);
    /* a zero hold resting on raw 0: inside the deadband, no time */
    wstart(0, 12);
    for (int k = 0; k < 2000; k++) {
        CHECK_EQ_U(wtick(0, false, false, true), 0);
    }
}

/* the window adds once when both conditions hold (independent round 3 P2-1) */
static void watch_time_added_once(void)
{
    wstart(30, 12);
    drive_on(1);
    CHECK_EQ_U(wtick(30, true, true, false), 0);
    CHECK_EQ_U(h.timer_us, 1000);
}

/* driven 0x0000 frames over 4 ms -> 11 */
static void watch_driven_zero_run(void)
{
    wstart(30, 12);
    uint8_t why = 0;
    int n = 0;
    while (!why && n < 10) {
        why = wtick(30, true, false, true);
        n++;
    }
    CHECK_EQ_U(why, DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT);
    CHECK_EQ_I(n, 5);
}

/* deadband entries wrap mod 65536 */
static void watch_entries_wrap(void)
{
    wstart(13, 12);
    h.entries = 0xFFFF;
    h.in_band = false;
    CHECK_EQ_U(wtick(12, false, true, false), 0);
    CHECK_EQ_U(h.entries, 0);
}

int main(void)
{
    targets();
    value_checks();
    control_order();
    min_duty_only_toward_the_error();
    control_non_finite_resets();
    watch_steady_progress();
    watch_stalled_outside();
    watch_rounded_zero_outside();
    watch_rest_in_band();
    watch_noise_with_reversals();
    watch_fast_sag();
    watch_resume_after_brake();
    watch_disturbance_return();
    watch_band_entry_resets();
    watch_previous_sample_outside_the_run();
    watch_previous_sample_before_the_write();
    watch_two_tick_correction();
    watch_eligibility_window();
    watch_zero_frames_outside();
    watch_time_added_once();
    watch_driven_zero_run();
    watch_entries_wrap();
    return fw_test_failures;
}
