/* mcu-link/3 messages (docs/interfaces/2026-10-22-mcu-link-contract.md). */
#ifndef FW_LINK_H
#define FW_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fw/crc.h"

#define LINK_VERSION 3u
#define LINK_BODY_MAX 90u /* v3: the 84-byte STATE payload + 6 */
#define LINK_WIRE_MAX 93u /* COBS(LINK_BODY_MAX) + delimiter, with room for one more code */

#define LINK_DRIVE 0x01u
#define LINK_ARM 0x02u
#define LINK_DISARM 0x03u
#define LINK_CLEAR_FAULTS 0x04u
#define LINK_COMMISSION 0x05u /* v2: FW_COMMISSION builds only */
#define LINK_BENCH_SET 0x06u
#define LINK_BENCH_HOLD 0x07u /* DIAGNOSTIC BRANCH ONLY (diag/fw-bench): hold M on the KTH travel counter */ /* DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the bench image's session overrides */
#define LINK_STATE 0x81u

#define LINK_DRIVE_FLAG_LIFT_HOLD 0x01u

typedef struct {
    float left_wheel_rad_s, right_wheel_rad_s, steering_rad, lift_m;
    uint16_t valid_after_echo_ms;
    uint8_t flags;
    uint16_t arm_gen;
    uint32_t boot_id;
    uint64_t echo_mcu_time_us;
} link_drive;

typedef struct {
    uint32_t boot_id;
    uint64_t echo_mcu_time_us;
} link_arm;

typedef struct {
    uint32_t boot_id;
} link_clear;

/* v2, contract 3.4: one axis's open-loop duty for axis-by-axis commissioning. */
typedef struct {
    uint8_t axis; /* 0 L, 1 R, 2 M (steering), 3 H (lift) */
    int16_t duty_e4;
    uint16_t valid_after_echo_ms;
    uint16_t arm_gen;
    uint32_t boot_id;
    uint64_t echo_mcu_time_us;
} link_commission;

/* DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the bench image's overrides for this boot's
   next arming(s) -- bypass bits (drive.h DRIVE_BENCH_*) and per-axis commissioning limits. Taken in
   DISARMED only; the boot starts with none. */
typedef struct {
    uint32_t boot_id;
    uint32_t bypass;
    uint16_t duty_max_e4[4];
    uint32_t time_ms[4];
} link_bench_set;

/* DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): hold an axis (M only) at a target of the bench
   image's KTH travel counter with a PI on the counter error; like a COMMISSION it is fresh only within
   its validity after the echo and needs a zero first. ``sign`` is the duty sign that raises the count. */
typedef struct {
    uint8_t axis;
    int8_t sign;
    uint32_t target_counts; /* the travel counter value (mod 2^32) */
    float kp, ki;           /* duty per rad of counter error, duty per rad*s */
    uint16_t deadband_counts;
    uint16_t duty_max_e4;
    uint16_t valid_after_echo_ms;
    uint16_t arm_gen;
    uint32_t boot_id;
    uint64_t echo_mcu_time_us;
} link_bench_hold;

typedef struct {
    uint64_t mcu_time_us;
    uint32_t boot_id;
    uint16_t arm_gen, last_rx_seq;
    uint8_t mode, reject_reason;
    uint32_t fault_flags;
    int32_t wheel_count_left, wheel_count_right;
    float wheel_rate_left_rad_s, wheel_rate_right_rad_s, steering_rad, lift_m;
    int16_t current_ma[4], duty_e4[4];
    uint8_t driver_status[4];
    uint16_t rx_errors, cycle_overruns;
    uint8_t flags;
    uint8_t reset_cause; /* v2 */
    /* v3, contract 3.3: per axis (0 M steering, 1 H lift) fresh KTH7801 frames, configured-variant CRC
       errors, other-variant CRC errors; each 16-bit modular, cleared only at boot. */
    uint16_t kth_frames[2], kth_crc_errors[2], kth_alt_crc_errors[2];
} link_state;

typedef struct {
    uint8_t type;
    uint16_t seq;
    union {
        link_drive drive;
        link_arm arm;
        link_clear clear;
        link_commission commission;
        link_bench_set bench;
        link_bench_hold hold;
        link_state state;
    } u;
} link_msg;

/* Wire bytes including the 0x00 delimiter, or 0 on failure. */
size_t link_encode(const link_msg *msg, uint8_t *wire, size_t cap);

typedef enum {
    LINK_OK = 0,
    LINK_BAD_COBS,
    LINK_BAD_CRC,
    LINK_BAD_VERSION,
    LINK_BAD_TYPE,
    LINK_BAD_LENGTH,
    LINK_OVERSIZE,
} link_status;

/* body = version .. crc (COBS already removed). */
link_status link_decode_body(const uint8_t *body, size_t len, link_msg *out);

typedef struct {
    uint8_t buf[LINK_WIRE_MAX];
    size_t len;
    bool overflow;
    uint16_t errors; /* saturating */
} link_rx;

void link_rx_init(link_rx *rx);

/* True when byte completes a valid message (written to out). Broken frames are counted. */
bool link_rx_feed(link_rx *rx, uint8_t byte, link_msg *out);

/* candidate is newer than last in modular 16-bit order. */
bool link_seq_newer(uint16_t candidate, uint16_t last);

#endif
