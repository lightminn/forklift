/* PI controller with feed-forward, deadband and conditional integration; slew limiter. */
#ifndef FW_PI_H
#define FW_PI_H

typedef struct {
    float kp, ki, kff;
    float out_max;  /* output clamp, > 0 */
    float deadband; /* |error| below this -> output 0, integral held */
} pi_config;

typedef struct {
    float integral;
} pi_state;

void pi_reset(pi_state *s);

float pi_step(const pi_config *c, pi_state *s, float target, float measured, float dt_s);

/* Moves current toward target by at most max_rate * dt_s. */
float slew_step(float current, float target, float max_rate, float dt_s);

#endif
