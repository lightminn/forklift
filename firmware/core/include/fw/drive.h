/* Drive core: safety state machine, command freshness/expiry and the four motor channels.
   Design: docs/design/2026-10-22-mcu-firmware-core.md sections 3-6; wire values follow the
   mcu-link/1 contract. Pure: time and inputs come in as arguments. */
#ifndef FW_DRIVE_H
#define FW_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/abs_axis.h"
#include "fw/drv8244.h"
#include "fw/bench_hold.h"
#include "fw/bench_kth.h"
#include "fw/link.h"
#include "fw/pi.h"
#include "fw/wheel_enc.h"

enum { DRIVE_CH_L, DRIVE_CH_R, DRIVE_CH_M, DRIVE_CH_H, DRIVE_CH_COUNT };

/* Modes (STATE.mode). */
#define DRIVE_MODE_BOOT 0u
#define DRIVE_MODE_DISARMED 1u
#define DRIVE_MODE_ARMED 2u
#define DRIVE_MODE_STOPPING 3u
#define DRIVE_MODE_FAULT 4u
#define DRIVE_MODE_ESTOP 5u

/* Reject reasons (STATE.reject_reason). */
#define DRIVE_REJECT_NONE 0u
#define DRIVE_REJECT_BAD_VALUE 1u
#define DRIVE_REJECT_NOT_ARMED 2u
#define DRIVE_REJECT_LATCHED 3u
#define DRIVE_REJECT_ESTOP_ACTIVE 4u
#define DRIVE_REJECT_DRIVER_NOT_READY 5u
#define DRIVE_REJECT_STALE_SEQ 6u
#define DRIVE_REJECT_ZERO_FIRST 7u
#define DRIVE_REJECT_BOOT_MISMATCH 8u
#define DRIVE_REJECT_ARM_GEN_MISMATCH 9u
#define DRIVE_REJECT_STALE_ECHO 10u
#define DRIVE_REJECT_STOPPING 11u
#define DRIVE_REJECT_NOT_DISARMED 12u
#define DRIVE_REJECT_SENSOR_INVALID 13u
#define DRIVE_REJECT_WRONG_BUILD 14u   /* v2: COMMISSION in a normal build, DRIVE in commission */
#define DRIVE_REJECT_ESTOP_UNWIRED 15u /* v2: no e-stop pin and the outputs are not locked */

/* Fault bits (STATE.fault_flags). */
#define DRIVE_FAULT_ESTOP_INPUT (1u << 0)
#define DRIVE_FAULT_COMMAND_TIMEOUT (1u << 1)
#define DRIVE_FAULT_DRIVER(ch) (1u << (2 + (ch)))
#define DRIVE_FAULT_STEER_SENSOR_INVALID (1u << 6)
#define DRIVE_FAULT_LIFT_SENSOR_INVALID (1u << 7)
#define DRIVE_FAULT_OVERCURRENT(ch) (1u << (8 + (ch)))
#define DRIVE_FAULT_DRIVER_NOT_READY (1u << 12)
#define DRIVE_FAULT_STEER_OUT_OF_RANGE (1u << 13)
#define DRIVE_FAULT_LIFT_OUT_OF_RANGE (1u << 14)
#define DRIVE_FAULT_CYCLE_OVERRUN (1u << 15)
#define DRIVE_FAULT_CURRENT_SENSE_INVALID (1u << 16)
#define DRIVE_FAULT_ENCODER_GAP (1u << 17)                 /* v2, both wheels */
#define DRIVE_FAULT_ENCODER_STUCK(w) (1u << (18 + (w)))    /* v2, w = 0 L, 1 R */
#define DRIVE_FAULT_ENCODER_POLARITY(w) (1u << (20 + (w))) /* v2 */
#define DRIVE_FAULT_ENCODER_JUMP(w) (1u << (22 + (w)))     /* v2 */
#define DRIVE_FAULT_COMMISSION_TIME_LIMIT (1u << 24)       /* v2 */

/* Bench extension (contract section 8, bench console plan A1): the checks a BENCH_SET may turn off for one
   run. The stopping side (overcurrent, current sense, driver faults, STOP/DISARM, command expiry, IWDG,
   cycle overrun) has no bit and never will. */
#define DRIVE_BENCH_STEER_RANGE (1u << 0) /* M: the steering joint range (KTH validity stays) */
#define DRIVE_BENCH_LIFT_RANGE (1u << 1)  /* H: the lift joint range (KTH validity stays) */
#define DRIVE_BENCH_KNOWN (DRIVE_BENCH_STEER_RANGE | DRIVE_BENCH_LIFT_RANGE)
#define DRIVE_BENCH_MODE_DUTY 1u
#define DRIVE_BENCH_MODE_HOLD 2u /* bench7 (plan 4.3 A1b-2): source kth, axis M or H, no stop_counts */
#define DRIVE_BENCH_SOURCE_NONE 0u
#define DRIVE_BENCH_SOURCE_KTH 1u /* bench6: the run axis's KTH travel count (M, H) */
#define DRIVE_BENCH_SOURCE_MT 2u  /* bench8 (plan 4.5 A1c): the run wheel's incremental count (L, R), relative */
/* BENCH_STATE.count_state */
#define DRIVE_BENCH_COUNT_NONE_YET 0u
#define DRIVE_BENCH_COUNT_VALID 1u
#define DRIVE_BENCH_COUNT_LOST 2u
/* BENCH_STATE.distance_confirm */
#define DRIVE_BENCH_CONFIRM_NONE 0u /* none, or waiting (with distance_provisional) */
#define DRIVE_BENCH_CONFIRM_REACHED 1u
#define DRIVE_BENCH_CONFIRM_HELD 2u
/* BENCH_STATE.run_flags (bench6, plan 4.2 A1b-1 decision 5) */
#define DRIVE_BENCH_FLAG_UNCLEAN 0x01u    /* this run: not valid at its drive start, or lost since the ARM */
#define DRIVE_BENCH_FLAG_RES_UNCLEAN 0x02u /* the result: the above, fixed at the end */
#define DRIVE_BENCH_FLAG_RES_LOSS 0x04u    /* the result: the provisional confirmation ended by a loss */
#define DRIVE_BENCH_FLAG_RES_TENTATIVE 0x08u /* the result: the last counted frame at the end was 0x0000 */
#define DRIVE_BENCH_FLAG_RES_NO_REF 0x10u    /* the result: no fresh non-zero frame at the drive start */
#define DRIVE_BENCH_FLAG_RES_HOLD 0x20u      /* bench7: the result run was a hold run */
#define DRIVE_BENCH_FLAG_RES_DISTANCE 0x40u  /* bench7: the result run was a distance run */
#define DRIVE_BENCH_ZERO_RUN_US 4000u     /* driven 0x0000-only time before kth_zero_suspect */
#define DRIVE_BENCH_WATCH_US 300000u      /* driven time without progress */
#define DRIVE_BENCH_PROGRESS_COUNTS 2     /* R */
#define DRIVE_BENCH_REVERSE_COUNTS 128    /* the wrong way from the push start */
/* BENCH_STATE.drive bits 0-1. */
#define DRIVE_BENCH_RUN_NONE 0u    /* no run consumed since boot */
#define DRIVE_BENCH_RUN_PREP 1u    /* armed, no accepted non-zero COMMISSION yet */
#define DRIVE_BENCH_RUN_DRIVING 2u /* from the first accepted non-zero COMMISSION */
#define DRIVE_BENCH_RUN_ENDED 3u
/* BENCH_STATE.stop_reason: the first cause that ended a run that drove. */
#define DRIVE_BENCH_STOP_NONE 0u
#define DRIVE_BENCH_STOP_END_ZERO 1u
#define DRIVE_BENCH_STOP_COMMAND_TIMEOUT 2u
#define DRIVE_BENCH_STOP_TIME_LIMIT 3u
#define DRIVE_BENCH_STOP_DISARM 4u
#define DRIVE_BENCH_STOP_FAULT 5u
#define DRIVE_BENCH_STOP_ESTOP 6u
#define DRIVE_BENCH_STOP_LINK_BOUNDARY 7u
#define DRIVE_BENCH_STOP_DISTANCE_REACHED 8u
#define DRIVE_BENCH_STOP_DISTANCE_PROVISIONAL 9u
#define DRIVE_BENCH_STOP_NO_PROGRESS 10u
#define DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT 11u
#define DRIVE_BENCH_STOP_KTH_LOST 12u
#define DRIVE_BENCH_SET_TTL_MS 1000u /* an unconsumed BENCH_SET expires */

/* STATE.flags bits. */
#define DRIVE_STATE_LIMITED 0x01u
#define DRIVE_STATE_COMMAND_FRESH 0x02u
#define DRIVE_STATE_ZERO_SEEN 0x04u
#define DRIVE_STATE_DRIVERS_READY 0x08u
#define DRIVE_STATE_COMMISSION_BUILD 0x10u /* v2 */
#define DRIVE_STATE_HIL_PLANT 0x20u        /* v2 */
#define DRIVE_STATE_OUTPUTS_LOCKED 0x40u   /* v2 */
#define DRIVE_STATE_ESTOP_UNWIRED 0x80u    /* v2 */

/* STATE.reset_cause (v2). */
#define DRIVE_RESET_POWER 0u
#define DRIVE_RESET_PIN 1u
#define DRIVE_RESET_IWDG 2u
#define DRIVE_RESET_SOFTWARE 3u
#define DRIVE_RESET_OTHER 4u

/* Every hardware value is required; there are no defaults (CONTRIBUTING 6). */
typedef struct {
    uint32_t period_us;          /* control period */
    uint32_t command_timeout_ms; /* upper bound on any command's validity, <= 65535 */
    uint32_t stop_grace_ms;      /* controlled stop before outputs go to their safe state */
    uint32_t driver_retry_ms;    /* wait before retrying a failed driver initialisation */
    uint32_t driver_wake_us;     /* DRV8244 tREADY before the first SPI frame */
    uint32_t spi_timeout_us;     /* lost-transaction detection, >= 2 * period_us */
    uint32_t status_poll_ms;     /* FAULT_SUMMARY poll interval per channel once ready */
    uint16_t overrun_limit;      /* consecutive late cycles that fault while driving */
    uint16_t adc_invalid_limit;  /* consecutive stale current samples -> invalid */
    wheel_enc_config wheel[2];   /* L, R */
    pi_config wheel_pi, steer_pi, lift_pi;
    abs_axis_config steer, lift; /* steering in rad, lift in m */
    float steer_rate_max;        /* rad/s */
    float lift_rate_max;         /* m/s */
    float max_wheel_rad_s;       /* DRIVE wheel targets are clipped to this */
    int8_t duty_sign[DRIVE_CH_COUNT];
    bool safe_brake[DRIVE_CH_COUNT]; /* idle output: true = brake (DRVOFF low, EN 0) */
    uint8_t drv_config3, drv_config4;
    float ipropi_ohm, ipropi_gain, adc_vref_v;
    uint32_t adc_full_scale;
    float overcurrent_a[DRIVE_CH_COUNT];
    uint32_t overcurrent_hold_ms;

    /* v2 build identity: compile-time constants of the board build, never changed by a
       message (design 2026-11-11-mcu-link-v2-firmware-core section 2). */
    bool commission_build;       /* FW_COMMISSION: COMMISSION only, DRIVE -> wrong_build */
    bool hil_plant;              /* FW_HIL_PLANT: an in-MCU model fills the inputs */
    bool outputs_locked;         /* the board never drives the motor outputs */
    bool estop_wired;            /* an e-stop input pin exists */
    /* V1.0 e-stop waiver (plan 2026-11-18): an open normal build without a pin may arm, behind a
       committed user record checked at build time. STATE still reports estop_unwired. */
    bool estop_waived;
    /* Per axis (L, R, M, H), commission builds only (design 2026-11-25-bench-commission-limits
       1.1): the |duty_e4| a COMMISSION may ask for, (0, 10000], and the continuous non-zero
       output time of an interval on that axis, (0, validator ms ceiling]. */
    uint16_t commission_duty_max_e4[DRIVE_CH_COUNT];
    uint32_t commission_time_ms[DRIVE_CH_COUNT];
    /* Single-axis commissioning (partial-parts plan 2026-11-25 PR-B, image fw-single): open
       commissioning builds only. The first accepted COMMISSION of an arming fixes its axis until
       DISARM; every other channel's DRVOFF stays high in every mode; the axis that drove last keeps
       its safe output after DISARM, and another axis may be picked only after an applied tick with
       all four DRVOFF high; M and H drive without their KTH (their bits are STATE only). Compiled
       only with FW_CORE_SINGLE_AXIS (the fw-single image and the host tests), so the other images
       keep their bytes. */
#if defined(FW_CORE_SINGLE_AXIS)
    bool commission_single_axis;
#endif
    /* Bench image (contract 8, plan A1): an open commissioning build that takes BENCH_SET/BENCH_ARM instead of
       ARM, one run per arming. Compiled only with FW_CORE_BENCH (the bench image and the host tests), so the
       other images keep their bytes. */
#if defined(FW_CORE_BENCH)
    bool bench_build;
#endif
    float wheel_duty_slew_per_s; /* wheel duty change limit while the outputs are on */
    /* Encoder fault detection (plan 3.3); board values wait for S3/S4 measurements. */
    uint32_t enc_gap_max_us;          /* longest gap between two wheel reads */
    float enc_phys_max_rad_s;         /* physical wheel maximum (not the command limit) */
    float enc_jump_rad_s;             /* a step implying a faster wheel is impossible */
    float enc_stuck_duty;             /* |applied duty| that must move the wheel ... */
    uint32_t enc_stuck_ms;            /* ... within this time ... */
    uint32_t enc_stuck_counts;        /* ... by at least this many counts */
    float enc_polarity_duty;          /* |applied duty| at which a wrong-way rise counts */
    uint32_t enc_polarity_ms;         /* how long the wrong way must last */
    float enc_polarity_rise_rad_s;    /* how much |rate| must grow the wrong way ... */
    uint32_t enc_polarity_persist_ms; /* ... or how long any wrong-way episode may last */
} drive_config;

typedef struct {
    uint32_t now_us; /* HAL clock, wraps; the core widens it to 64 bits */
    uint16_t wheel_hw[2];
    uint16_t abs_frame[2]; /* steering, lift */
    bool abs_fresh[2];
    uint32_t abs_time_us[2]; /* HAL clock when each fresh frame was acquired (SPI completion), never after now_us */
    uint32_t adc_raw[DRIVE_CH_COUNT];
    bool adc_fresh;
    bool nfault_low[DRIVE_CH_COUNT];
    bool estop_active;
    bool cycle_overrun; /* this cycle started late (set by the app) */
#if defined(FW_CORE_BENCH)
    /* bench7 (plan 4.3 A1b-2 decision 6): the motor output HAL's report at read_inputs (fw/bench_pwm.h). Without it
       (pwm_report false) the core takes its own last duty as written at its tick time. */
    bool pwm_report;
    int16_t applied_e4[DRIVE_CH_COUNT]; /* active now, physical sign x compare value (0 when cut) */
    bool applied_any[DRIVE_CH_COUNT];   /* a non-zero may have been active since the last read */
    uint32_t applied_window_us;         /* HAL clock from the last read to this one */
    bool run_valid[DRIVE_CH_COUNT];     /* the written values form one sign's unbroken non-zero run */
    uint32_t run_since_us[DRIVE_CH_COUNT]; /* HAL clock after that run's first write */
    int8_t run_sign[DRIVE_CH_COUNT];       /* physical */
    uint32_t abs_start_us[2];              /* each fresh frame's bundle start: a lower bound of its acquisition */
    bool abs_start_known[2];
    /* bench8 (plan 4.5 A1c decision 4): the wheel reads' report. Without it (wheel_report false) every read counts
       as a success taken at now_us. */
    bool wheel_report;
    bool wheel_fresh[2];       /* the stable read succeeded (else wheel_hw is the previous value) */
    uint32_t wheel_start_us[2]; /* HAL clock just before that wheel's read: its sample time */
#endif
} drive_inputs;

typedef struct {
    float duty[DRIVE_CH_COUNT]; /* -1..1, sign already applied */
    bool drvoff[DRIVE_CH_COUNT];
} drive_outputs;

typedef struct drive {
    /* Public, read-only for callers. */
    uint8_t mode, reject_reason;
    uint16_t arm_gen, last_rx_seq;
    uint32_t boot_id;
    uint8_t reset_cause;  /* STATE.reset_cause (v2) */
    uint64_t time_us;     /* MCU time, 64-bit (STATE.mcu_time_us) */
    uint32_t fault_flags; /* latched bits | live status bits */
    uint8_t driver_status[DRIVE_CH_COUNT];
    bool drivers_ready;
    float current_a[DRIVE_CH_COUNT];
    drive_outputs out;

    /* Internal. */
    const drive_config *cfg;
    uint32_t last_now32;
    uint32_t latched;
    bool estop_now;
    wheel_enc wheel[2];
    abs_axis steer, lift;
    uint16_t adc_bad_streak;
    bool adc_valid;
    bool over_active[DRIVE_CH_COUNT];
    uint64_t over_since_us[DRIVE_CH_COUNT];
    uint16_t overrun_streak;
    float wheel_target[2], steer_target, lift_target, steer_ref, lift_ref;
    pi_state wheel_pi[2], steer_pi, lift_pi;
    bool has_deadline;
    uint64_t deadline_us, stop_until_us;
    bool zero_seen, have_drive_seq, limited;
    uint16_t last_drive_seq;
    float wheel_duty[2]; /* applied wheel duty, control coordinates, after the slew limit */
    /* COMMISSION (commission builds). */
    uint8_t cm_axis;
    float cm_duty; /* control coordinates */
    bool cm_on;
    uint8_t cm_on_axis; /* the axis of the open output interval, fixed at its first non-zero
                           output and released with cm_on (never the last command's cm_axis) */
    uint64_t cm_on_since_us;
#if defined(FW_CORE_SINGLE_AXIS)
    uint8_t sa_last_axis; /* single-axis build: the axis an arming last fixed (kept across DISARM) */
#endif
#if defined(FW_CORE_BENCH)
    /* Bench image: the pending BENCH_SET (until a BENCH_ARM consumes it), the consumed run and its result. */
    bool bset_valid;
    link_bench_set bset;
    uint64_t bset_at_us;
    uint8_t bset_gen; /* the connection generation it arrived on */
    bool run_active;  /* from the BENCH_ARM until the mode leaves ARMED/STOPPING */
    uint32_t run_id, run_bypass;
    uint8_t run_axis, run_mode, run_source, run_drive;
    uint16_t run_duty_max_e4[DRIVE_CH_COUNT];
    uint32_t run_time_ms[DRIVE_CH_COUNT];
    uint32_t res_run_id; /* 0: no run has driven since boot */
    uint8_t res_stop, res_axis, res_source;
    uint16_t res_blocked; /* drive requests refused after the run ended (saturating) */
    uint8_t conn_gen;     /* link connection changes seen (mod 256) */
    /* bench6 (plan 4.2 A1b-1): the KTH travel counters (0 M, 1 H), this tick's result bits, the zero-suspect waits */
    bench_kth kth[2];
    uint8_t kth_bits[2];
    bool kth_zs_wait[2];
    uint64_t bench_last_us; /* the previous tick's time: the driven-time increments */
    /* the run */
    uint32_t run_losses0; /* the axis's loss count at the ARM */
    bool run_unclean, run_no_ref, run_ev;
    uint64_t run_ev_t; /* the last trusted sample's acquisition time since the ARM */
    int32_t run_stop;
    int8_t run_sign, run_dir;
    uint32_t run_ref, w_base;
    uint64_t w_timer_us, zr_us;
    bool w_trusted; /* a trusted sample since the watch last reset */
    /* the result's distance fields */
    uint32_t res_ref, res_cut, res_losses;
    int32_t res_stop_counts;
    uint8_t res_confirm, res_flags;
    uint16_t res_end_age;
    uint64_t boundary_us; /* after every STATE sent before the last one: bench commands echo a later STATE */
    uint32_t cm_on_limit_ms; /* the open output interval's time limit, fixed when it opened (a run's own) */
    /* bench7 (plan 4.3 A1b-2): the hold run, this tick's frames' acquisition lower bounds, the result's bypass */
    bench_hold hold;
    bool hold_started; /* the run's first BENCH_HOLD accepted: target and entries are this run's */
    uint64_t kth_lo[2];
    bool kth_lo_known[2];
    uint32_t res_bypass;
    /* the motor output report of this tick, widened (decision 6); without a HAL report the core's own writes */
    int16_t pwm_applied[DRIVE_CH_COUNT];
    bool pwm_any[DRIVE_CH_COUNT];
    uint64_t pwm_window_us, pwm_last_us;
    bool pwm_have_last, pwm_reported;
    bool pwm_run_valid[DRIVE_CH_COUNT];
    uint64_t pwm_run_since[DRIVE_CH_COUNT];
    int8_t pwm_run_sign[DRIVE_CH_COUNT]; /* physical */
    bool fb_run_valid[DRIVE_CH_COUNT];   /* the fallback's runs of the core's own written duty */
    uint64_t fb_run_since[DRIVE_CH_COUNT];
    int8_t fb_run_sign[DRIVE_CH_COUNT];
    bool lc_pending, lc_settled; /* a link change noted for the next drive_begin */
    uint32_t lc_since32;         /* HAL clock when it began */
    /* bench8 (plan 4.5 A1c): the MT views of the wheels (bench_kth fields: have, lost, losses, cum, trust_cum,
       trust_t; last_zero is never set), this tick's accepted sample bits and sample times, the wheels' loss state */
    bench_kth mt[2];
    uint8_t mt_bits[2];
    uint64_t mt_lo[2];
    bool mt_lo_known[2];
    bool wheel_based[2]; /* the wheel's accumulator has a base taken on a successful read */
    bool wheel_grace[2]; /* the late first sample's step may still be dropped without a loss (decision 4 (a)) */
    uint64_t first_tick_us;
    bool first_tick_seen;
    bool wheels_done;    /* this tick's wheel samples are processed (cleared at drive_finish) */
    bool res_pre_wheel;  /* the result was fixed this tick before the wheel samples (late attribution, decision 10) */
    bool w_base_set;     /* an MT distance run's watch base is set on this output interval (decision 7) */
    uint64_t w_base_since;
    bool mt_fresh[2];    /* this tick's read of the wheel succeeded */
    uint64_t tick_dt_us; /* this tick's length (the encoder watch's pause, decision 4) */
#endif
    uint64_t cm_hold_until_us; /* after the time limit: no ARM before this (zero output) */
    /* Encoder fault detection episodes. */
    bool first_cycle; /* no gap judgement before the first drive_begin */
    bool stuck_active[2], pol_active[2], pol_base_set[2];
    uint64_t stuck_t0_us[2], pol_t0_us[2], pol_tb_us[2];
    uint32_t pol_t0_hal[2]; /* HAL clock at the episode start */
    uint32_t stuck_c0[2];
    float pol_r0[2];
    /* Driver initialisation and SPI. */
    drv8244_init ini;
    uint8_t init_ch;
    uint16_t init_sdo;
    bool init_after_clear, retry_waiting;
    uint64_t retry_at_us;
    bool spi_pending, spi_inflight;
    uint32_t spi_tag; /* last started transaction; answers with another tag are dropped */
    uint8_t spi_ch;
    uint16_t spi_frame;
    uint64_t spi_since_us; /* pending or in flight since */
    uint8_t poll_ch;
    uint64_t next_poll_us;
} drive;

bool drive_config_valid(const drive_config *c);

/* false for an invalid config, boot_id 0 or a reset_cause outside DRIVE_RESET_*. */
bool drive_init(drive *d, const drive_config *c, const drive_inputs *first, uint32_t boot_id,
                uint8_t reset_cause);

/* Sensors, expiry (before any message), e-stop and faults for this cycle. */
void drive_begin(drive *d, const drive_inputs *in);

/* One received message, at the time given to drive_begin. */
void drive_handle(drive *d, const link_msg *m);

/* Controllers and outputs for this cycle (also left in d->out). */
void drive_finish(drive *d, drive_outputs *out);

/* One SPI transaction at a time: the frame to start (repeated until started) and its tag.
   The HAL hands the tag back with the answer; an answer with another tag (a transaction
   that was given up or belongs to an abandoned sequence) is dropped. */
bool drive_spi_next(drive *d, uint8_t *ch, uint16_t *frame, uint32_t *tag);
void drive_spi_started(drive *d);
void drive_spi_result(drive *d, uint8_t ch, uint16_t sdo, uint32_t tag);

/* Fills everything but rx_errors and cycle_overruns (owned by the app). */
void drive_fill_state(const drive *d, link_state *s);

#if defined(FW_CORE_BENCH)
/* A link connection change (bench image, contract 8.4), noted before drive_begin, which applies it with the
   tick's own time: one that began (since_hal_us) while armed ends the arming through STOPPING unless the command
   deadline or the output interval's time limit had passed before it began (then that is the first cause); once
   settled, the pending BENCH_SET is dropped and bench commands must echo a STATE sent from this tick on. No
   effect on other builds. */
void drive_note_link_change(drive *d, uint32_t since_hal_us, bool settled);

/* Fills everything but pair_seq and dropped_pairs (owned by the app). */
void drive_fill_bench_state(const drive *d, link_bench_state *s);
#endif

#endif
