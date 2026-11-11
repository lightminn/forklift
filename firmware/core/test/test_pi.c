#include "fw/pi.h"
#include "fw_test.h"

int main(void)
{
    /* synthetic gains */
    pi_config p = {.kp = 2.0f, .ki = 0.0f, .kff = 0.0f, .out_max = 10.0f, .deadband = 0.0f};
    pi_state s;
    pi_reset(&s);
    CHECK_NEAR(pi_step(&p, &s, 1.0f, 0.25f, 0.001f), 1.5, 1e-6);
    p.out_max = 1.0f;
    CHECK_NEAR(pi_step(&p, &s, 1.0f, 0.25f, 0.001f), 1.0, 1e-6);

    /* Conditional integration: saturated with the error pushing further -> integral holds. */
    p.ki = 5.0f;
    pi_reset(&s);
    s.integral = 0.2f;
    for (int k = 0; k < 10; k++) {
        pi_step(&p, &s, 1.0f, 0.0f, 0.01f);
    }
    CHECK_NEAR(s.integral, 0.2, 1e-6);
    /* ...but an error of the other sign still unwinds it. */
    pi_step(&p, &s, 0.0f, 0.1f, 0.01f);
    CHECK(s.integral < 0.2f);

    /* Deadband: output 0, integral kept. */
    pi_config d = {.kp = 2.0f, .ki = 5.0f, .kff = 0.0f, .out_max = 1.0f, .deadband = 0.1f};
    pi_reset(&s);
    s.integral = 0.05f;
    CHECK_NEAR(pi_step(&d, &s, 0.05f, 0.0f, 0.01f), 0.0, 0.0);
    CHECK_NEAR(s.integral, 0.05, 1e-7);

    /* Feed-forward. */
    pi_config f = {.kp = 0.0f, .ki = 0.0f, .kff = 0.1f, .out_max = 1.0f, .deadband = 0.0f};
    pi_reset(&s);
    CHECK_NEAR(pi_step(&f, &s, 5.0f, 5.0f, 0.01f), 0.5, 1e-6);

    /* Slew limiter. */
    CHECK_NEAR(slew_step(0.0f, 1.0f, 2.0f, 0.1f), 0.2, 1e-6);
    CHECK_NEAR(slew_step(0.9f, 1.0f, 2.0f, 0.1f), 1.0, 1e-6);
    CHECK_NEAR(slew_step(0.0f, -1.0f, 2.0f, 0.1f), -0.2, 1e-6);

    /* pi_step_range: the output stays in [lo, hi] and the integral stops winding up when the
       range (e.g. a slew bound) holds the output back in the error's direction. */
    pi_config r = {.kp = 1.0f, .ki = 10.0f, .kff = 0.0f, .out_max = 1.0f, .deadband = 0.0f};
    pi_reset(&s);
    for (int k = 0; k < 50; k++) {
        float u = pi_step_range(&r, &s, 1.0f, 0.0f, 0.01f, -0.1f, 0.1f);
        CHECK(u <= 0.1f + 1e-7f && u >= -0.1f - 1e-7f);
    }
    CHECK_NEAR(s.integral, 0.0, 1e-7); /* kp*e = 1 > hi from the start: never integrates */
    /* Error of the other sign still integrates (unwinds). */
    s.integral = 0.05f;
    pi_step_range(&r, &s, 0.0f, 0.02f, 0.01f, -0.1f, 0.1f); /* unclamped 0.03: inside */
    CHECK(s.integral < 0.05f);
    /* Deadband inside a range that excludes 0: clamped into the range, integral kept. */
    pi_config rd = {.kp = 1.0f, .ki = 1.0f, .kff = 0.0f, .out_max = 1.0f, .deadband = 0.5f};
    pi_reset(&s);
    s.integral = 0.25f;
    CHECK_NEAR(pi_step_range(&rd, &s, 0.1f, 0.0f, 0.01f, 0.4f, 0.6f), 0.4, 1e-7);
    CHECK_NEAR(s.integral, 0.25, 1e-7);
    /* pi_step equals pi_step_range over [-out_max, out_max]. */
    pi_state a, b;
    pi_reset(&a);
    pi_reset(&b);
    pi_config g = {.kp = 0.3f, .ki = 4.0f, .kff = 0.05f, .out_max = 0.7f, .deadband = 0.01f};
    for (int k = 0; k < 200; k++) {
        float tgt = (k / 40) % 2 ? 3.0f : -1.0f, meas = 0.01f * (float)(k % 17);
        float ua = pi_step(&g, &a, tgt, meas, 0.001f);
        float ub = pi_step_range(&g, &b, tgt, meas, 0.001f, -0.7f, 0.7f);
        CHECK(ua == ub && a.integral == b.integral);
    }
    return FW_TEST_RESULT();
}
