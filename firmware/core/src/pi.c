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

float pi_step(const pi_config *c, pi_state *s, float target, float measured, float dt_s)
{
    float e = target - measured;
    if (fabsf(e) < c->deadband) {
        return 0.0f;
    }
    float unclamped = c->kff * target + c->kp * e + s->integral;
    /* Conditional integration: stop winding up while saturated in the error's direction. */
    bool saturated_same_way =
        (unclamped > c->out_max && e > 0.0f) || (unclamped < -c->out_max && e < 0.0f);
    if (!saturated_same_way) {
        s->integral += c->ki * e * dt_s;
    }
    return clampf(c->kff * target + c->kp * e + s->integral, c->out_max);
}

float slew_step(float current, float target, float max_rate, float dt_s)
{
    float step = max_rate * dt_s;
    float delta = target - current;
    return current + clampf(delta, step);
}
