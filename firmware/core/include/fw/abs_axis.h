/* KTH7801 absolute angle frames -> one joint position with validity (design section 5). */
#ifndef FW_ABS_AXIS_H
#define FW_ABS_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/crc.h"

typedef struct {
    kth_crc_variant crc;
    float zero_offset_rad;  /* sensor angle at joint zero */
    int8_t sign;            /* +1 or -1 */
    float sensor_per_joint; /* sensor radians per joint unit (> 0) */
    float min_joint, max_joint;
    uint16_t invalid_limit; /* consecutive failed samples before the axis is invalid (>= 1) */
} abs_axis_config;

typedef struct {
    float position; /* joint units; held through failed samples */
    bool valid;
    bool out_of_range;
    uint16_t bad_streak;
    uint32_t crc_errors;
} abs_axis;

/* 16-bit SPI word = angle (bits 15..4) | CRC-4 (bits 3..0). */
bool kth7801_decode(uint16_t frame, kth_crc_variant v, uint16_t *angle12);

/* The joint range must not reach the +-pi cut of the wrapped sensor angle. */
bool abs_axis_config_valid(const abs_axis_config *c);

void abs_axis_init(abs_axis *a);

/* fresh = the HAL received this frame in the current cycle. */
void abs_axis_update(abs_axis *a, const abs_axis_config *c, uint16_t frame, bool fresh);

#endif
