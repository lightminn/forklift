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
    return FW_TEST_RESULT();
}
