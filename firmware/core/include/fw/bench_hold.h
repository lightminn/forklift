/* Bench image (bench console plan 4.3 A1b-2): a hold run's target, control and progress watch, as pure functions the
   drive core calls once a tick. Counts are the KTH travel counter's (fw/bench_kth.h), modular 32-bit. Compiled only
   with FW_CORE_BENCH. */
#ifndef FW_BENCH_HOLD_H
#define FW_BENCH_HOLD_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/link.h"

#if defined(FW_CORE_BENCH)

#define BENCH_HOLD_TARGET_ZERO 0u
#define BENCH_HOLD_TARGET_ABS 1u
#define BENCH_HOLD_TARGET_HERE 2u
#define BENCH_HOLD_TARGET_REL 3u
#define BENCH_HOLD_CPR 4096u
#define BENCH_HOLD_TAU_US 10000.0f   /* the D term's low-pass */
#define BENCH_HOLD_ELIGIBLE_US 200u  /* run start to a sample's acquisition: one PWM period + resolution + KTH lag */
#define BENCH_HOLD_R 2               /* the watch's progress step */
#define BENCH_HOLD_WATCH_US 300000u
#define BENCH_HOLD_ZERO_RUN_US 4000u /* driven 0x0000-only time */

typedef struct {
    link_bench_hold cfg; /* the first accepted BENCH_HOLD: the later ones must match it */
    uint32_t target;
    /* control */
    float integ; /* duty */
    float rate;  /* rad/s of the count, low-passed */
    bool rate_have;
    uint32_t rate_cum;
    uint64_t rate_t;
    /* watch (plan 4.3 decision 7) */
    uint32_t r;
    uint64_t timer_us, zr_us;
    uint64_t fb_timer_us, fb_zr_us; /* what unreported windows added to each since the last HAL report */
    bool trusted_seen; /* a trusted sample since the watch last reset: 10, else 11 */
    bool in_band;      /* the last trusted sample was inside the deadband */
    uint16_t entries;  /* deadband entries (out -> in), modular */
    /* the previous trusted sample, for the progress eligibility */
    bool prev_lo_known, prev_run_valid;
    uint64_t prev_lo, prev_run_since;
} bench_hold;

/* One tick's watch input (the run axis). */
typedef struct {
    bool reported;      /* a HAL report this tick: window_us is its read-to-read window, which covers the unreported
                           ticks since its last read (their additions are taken back out of it) */
    uint64_t window_us; /* this read's window (HAL read to HAL read), or the tick's when unreported */
    bool applied_any;   /* a non-zero may have been active in it */
    bool last_zero;     /* the last counted frame was 0x0000 */
    uint32_t cum;       /* the counter now (tentative when last_zero) */
    bool trusted_new;   /* a non-zero frame counted this tick */
    uint32_t trust_cum; /* the last trusted sample's count (this tick's when trusted_new) */
    bool lo_known; /* that frame's acquisition lower bound (its bundle start) is known */
    uint64_t lo;
    bool run_valid; /* the HAL's written values form one sign's run, now */
    uint64_t run_since;
    int8_t run_dir; /* that run's direction in count space (+1 raises the count) */
} bench_hold_tick;

/* The value checks of a BENCH_HOLD (plan 4.3 decision 2) against the run axis's duty limit. */
bool bench_hold_values_ok(const link_bench_hold *c, uint16_t axis_duty_max_e4);

/* The target the first HOLD fixes from the sample raw/cum (decision 4). */
uint32_t bench_hold_target(uint8_t kind, int32_t value, uint16_t raw, uint32_t cum);

/* The run's first accepted HOLD, on a trusted sample counted this tick (raw, cum, acquisition lower bound lo). */
void bench_hold_start(bench_hold *h, const link_bench_hold *c, uint16_t raw, uint32_t cum, uint64_t trust_t,
                      bool lo_known, uint64_t lo);

/* A later HOLD: true when every setting matches the first one. */
bool bench_hold_same(const bench_hold *h, const link_bench_hold *c);

/* This tick's duty in request space (decision 5). usable: a valid count, no loss, the last counted frame non-zero.
   trusted_new/trust_cum/trust_t: this tick's trusted sample. dt_s: the integrator step. */
float bench_hold_control(bench_hold *h, bool usable, bool trusted_new, uint32_t trust_cum, uint64_t trust_t,
                         float dt_s);

/* A deadband entry (out -> in) on this tick's trusted sample, counted when it is consumed (decision 7). */
void bench_hold_count_entry(bench_hold *h, bool trusted_new, uint32_t trust_cum);

/* This tick's watch (decision 7, time from decisions 5-6): 0, or the stop reason (DRIVE_BENCH_STOP_NO_PROGRESS,
   DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT). */
uint8_t bench_hold_watch(bench_hold *h, const bench_hold_tick *t);

#endif /* FW_CORE_BENCH */
#endif
