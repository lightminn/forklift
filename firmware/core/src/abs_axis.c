#include "fw/abs_axis.h"

#include <math.h>

#define PI_F 3.14159265358979323846f
#define TWO_PI_F 6.28318530717958647692f

bool kth7801_decode(uint16_t frame, kth_crc_variant v, uint16_t *angle12)
{
    uint16_t angle = (uint16_t)(frame >> 4);
    if (crc4_kth7801(angle, v) != (frame & 0xFu)) {
        return false;
    }
    *angle12 = angle;
    return true;
}

bool abs_axis_config_valid(const abs_axis_config *c)
{
    if ((c->sign != 1 && c->sign != -1) || c->invalid_limit == 0 || !isfinite(c->zero_offset_rad) ||
        !isfinite(c->sensor_per_joint) || !(c->sensor_per_joint > 0.0f) ||
        !isfinite(c->min_joint) || !isfinite(c->max_joint) || !(c->min_joint < c->max_joint)) {
        return false;
    }
    /* The joint range, seen on the sensor, must stay inside (-pi, pi). */
    float reach = fmaxf(fabsf(c->min_joint), fabsf(c->max_joint)) * c->sensor_per_joint;
    return reach < PI_F;
}

void abs_axis_init(abs_axis *a)
{
    a->position = 0.0f;
    a->valid = false;
    a->out_of_range = false;
    a->bad_streak = 0;
    a->crc_errors = 0;
}

static float wrap_pi(float x)
{
    while (x > PI_F) {
        x -= TWO_PI_F;
    }
    while (x <= -PI_F) {
        x += TWO_PI_F;
    }
    return x;
}

void abs_axis_update(abs_axis *a, const abs_axis_config *c, uint16_t frame, bool fresh)
{
    uint16_t angle = 0;
    bool good = fresh && kth7801_decode(frame, c->crc, &angle);
    if (fresh && !good) {
        a->crc_errors++;
    }
    if (!good) {
        if (a->bad_streak < UINT16_MAX) {
            a->bad_streak++;
        }
        if (a->bad_streak >= c->invalid_limit) {
            a->valid = false;
        }
        return;
    }
    a->bad_streak = 0;
    float sensor = wrap_pi((float)angle * TWO_PI_F / 4096.0f - c->zero_offset_rad);
    a->position = (float)c->sign * sensor / c->sensor_per_joint;
    a->valid = true;
    a->out_of_range = a->position < c->min_joint || a->position > c->max_joint;
}
