#include "fw/pi.h"

#include <math.h>
#include <stdbool.h>

static float clampf(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

void pi_reset(pi_state *s)
{
    s->integral = 0.0f;
}

static float clamp_range(float x, float lo, float hi)
{
    return x > hi ? hi : (x < lo ? lo : x);
}

float pi_step_range(const pi_config *c, pi_state *s, float target, float measured, float dt_s,
                    float lo, float hi)
{
    float e = target - measured;
    if (fabsf(e) < c->deadband) {
        return clamp_range(0.0f, lo, hi);
    }
    float unclamped = c->kff * target + c->kp * e + s->integral;
    /* Conditional integration: stop winding up while saturated in the error's direction. */
    bool saturated_same_way = (unclamped > hi && e > 0.0f) || (unclamped < lo && e < 0.0f);
    if (!saturated_same_way) {
        s->integral += c->ki * e * dt_s;
    }
    return clamp_range(c->kff * target + c->kp * e + s->integral, lo, hi);
}

float pi_step(const pi_config *c, pi_state *s, float target, float measured, float dt_s)
{
    return pi_step_range(c, s, target, measured, dt_s, -c->out_max, c->out_max);
}

float slew_step(float current, float target, float max_rate, float dt_s)
{
    float step = max_rate * dt_s;
    float delta = target - current;
    return current + clampf(delta, step);
}
