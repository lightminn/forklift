/* mcu-link/1 messages (docs/interfaces/2026-10-22-mcu-link-contract.md). */
#ifndef FW_LINK_H
#define FW_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fw/crc.h"

#define LINK_VERSION 1u
#define LINK_BODY_MAX 80u
#define LINK_WIRE_MAX 83u /* COBS(LINK_BODY_MAX) + delimiter, with room for one more code */

#define LINK_DRIVE 0x01u
#define LINK_ARM 0x02u
#define LINK_DISARM 0x03u
#define LINK_CLEAR_FAULTS 0x04u
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
} link_state;

typedef struct {
    uint8_t type;
    uint16_t seq;
    union {
        link_drive drive;
        link_arm arm;
        link_clear clear;
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
