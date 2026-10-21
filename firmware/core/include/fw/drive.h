/* Drive core: safety state machine, command freshness/expiry and the four motor channels.
   Design: docs/design/2026-10-22-mcu-firmware-core.md sections 3-6; wire values follow the
   mcu-link/1 contract. Pure: time and inputs come in as arguments. */
#ifndef FW_DRIVE_H
#define FW_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/abs_axis.h"
#include "fw/drv8244.h"
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

/* STATE.flags bits. */
#define DRIVE_STATE_LIMITED 0x01u
#define DRIVE_STATE_COMMAND_FRESH 0x02u
#define DRIVE_STATE_ZERO_SEEN 0x04u
#define DRIVE_STATE_DRIVERS_READY 0x08u

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
} drive_config;

typedef struct {
    uint32_t now_us; /* HAL clock, wraps; the core widens it to 64 bits */
    uint16_t wheel_hw[2];
    uint16_t abs_frame[2]; /* steering, lift */
    bool abs_fresh[2];
    uint32_t adc_raw[DRIVE_CH_COUNT];
    bool adc_fresh;
    bool nfault_low[DRIVE_CH_COUNT];
    bool estop_active;
    bool cycle_overrun; /* this cycle started late (set by the app) */
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

/* false for an invalid config or boot_id 0. */
bool drive_init(drive *d, const drive_config *c, const drive_inputs *first, uint32_t boot_id);

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

#endif
