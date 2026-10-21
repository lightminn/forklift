/* Incremental wheel encoder: 16-bit hardware counter -> 32-bit modular count and rate. */
#ifndef FW_WHEEL_ENC_H
#define FW_WHEEL_ENC_H

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_ENC_WINDOW_MAX 64

typedef struct {
    uint32_t counts_per_rev; /* quadrature counts per wheel revolution */
    uint16_t window;         /* samples between the rate's end points, 1..WHEEL_ENC_WINDOW_MAX */
    int8_t sign;             /* +1 or -1: positive = vehicle-forward rolling */
} wheel_enc_config;

typedef struct {
    uint32_t count; /* modular; differences are taken mod 2^32 */
    float rate_rad_s;
    uint16_t last_hw;
    uint16_t head, filled;
    uint32_t hist_count[WHEEL_ENC_WINDOW_MAX + 1];
    uint32_t hist_t_us[WHEEL_ENC_WINDOW_MAX + 1];
} wheel_enc;

/* False for an invalid configuration. */
bool wheel_enc_init(wheel_enc *e, const wheel_enc_config *c, uint16_t hw, uint32_t now_us);

/* The 16-bit difference since the last read must stay within +-32767 counts. */
void wheel_enc_update(wheel_enc *e, const wheel_enc_config *c, uint16_t hw, uint32_t now_us);

#endif
