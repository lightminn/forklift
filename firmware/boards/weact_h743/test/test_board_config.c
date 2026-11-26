/* The configuration the board actually runs (needs the vehicle header, so built only with
   FW_PYTHON). */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board/board_config.h"
#include "fw/link.h"
#include "fw_test.h"

/* STATE.flags the core reports for this configuration (printed for the host-side policy test,
   tests/integration/test_firmware_host.py). */
static uint8_t state_flags(const drive_config *c)
{
    static drive d;
    drive_inputs first;
    memset(&first, 0, sizeof first);
    CHECK(drive_init(&d, c, &first, 1u, 0u));
    link_state s;
    memset(&s, 0, sizeof s);
    drive_fill_state(&d, &s);
    return s.flags;
}

int main(void)
{
    const drive_config *c = board_drive_config();
    CHECK(drive_config_valid(c));
    CHECK(board_config_valid(c));
    /* IPROPI on the ADC pins is unprotected on schematic V1.0: outputs stay locked by default
       (plan 5 precondition, lead decision Q6); only the gated commissioning build opens them
       (motor output design 2). */
#if defined(FW_ESTOP_WAIVED)
    /* V1.0 e-stop waiver (plan 2026-11-18): the open normal build, behind both records. */
    CHECK(!c->outputs_locked && !c->commission_build && c->estop_waived);
#elif defined(FW_MOTOR_OUTPUT_OPEN)
    CHECK(!c->outputs_locked && c->commission_build && !c->estop_waived);
#else
    CHECK(c->outputs_locked && !c->estop_waived);
#endif
#if defined(FW_BUILD_hil)
    CHECK(c->hil_plant && !c->commission_build); /* board design 9 */
#elif defined(FW_BUILD_commission)
    CHECK(c->commission_build && !c->hil_plant);
#else
    CHECK(!c->hil_plant && !c->commission_build);
#endif
    CHECK(!c->estop_wired); /* no e-stop net on schematic V1.0 */
#if defined(FW_COMMISSION_SINGLE)
    CHECK(c->commission_single_axis); /* fw-single (partial-parts plan 2026-11-25) */
#if !defined(FW_CORE_SINGLE_AXIS)
#error "FW_COMMISSION_SINGLE needs the core's FW_CORE_SINGLE_AXIS"
#endif
#else
    CHECK(!c->commission_single_axis);
#endif
#if defined(FW_BENCH_IMAGE)
    CHECK(c->bench_build); /* diag/fw-bench */
#if !defined(FW_CORE_BENCH)
#error "FW_BENCH_IMAGE needs the core's FW_CORE_BENCH"
#endif
#elif defined(FW_CORE_BENCH)
    CHECK(!c->bench_build);
#endif
#if defined(FW_CORE_BENCH)
    /* diag/fw-bench: the core refuses the bench image on anything but an open commissioning build,
       and with single-axis at the same time. */
    drive_config bn = *c;
    bn.bench_build = true;
    bn.commission_single_axis = false;
    CHECK(drive_config_valid(&bn) == (bn.commission_build && !bn.outputs_locked && !bn.hil_plant));
    bn.commission_single_axis = true;
    CHECK(!drive_config_valid(&bn));
#endif
    /* The core refuses single-axis on anything but an open commissioning build. */
    drive_config sa = *c;
    sa.commission_single_axis = true;
#if defined(FW_CORE_BENCH)
    sa.bench_build = false;
#endif
    CHECK(drive_config_valid(&sa) == (sa.commission_build && !sa.outputs_locked && !sa.hil_plant));
    /* Every locked build reports outputs_locked (bit6), the IWDG stall test build included (lead
       condition 2026-11-12): the real executor then refuses teleop and autonomy. The output
       build clears it and is refused as a commissioning build instead. */
    uint8_t flags = state_flags(c);
#if defined(FW_MOTOR_OUTPUT_OPEN)
    CHECK((flags & DRIVE_STATE_OUTPUTS_LOCKED) == 0u);
#else
    CHECK((flags & DRIVE_STATE_OUTPUTS_LOCKED) != 0u);
#endif
    printf("STATE flags 0x%02x\n", flags);
    /* Commissioning limits (design 2026-11-25-bench-commission-limits 1.1): only the open
       commissioning build raises the wheels; steering and lift stay at 0.15 / 2 s. */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        bool wheel = ch == DRIVE_CH_L || ch == DRIVE_CH_R;
#if defined(FW_BUILD_commission) && defined(FW_MOTOR_OUTPUT_OPEN)
        CHECK_EQ_U(c->commission_duty_max_e4[ch], wheel ? 10000u : 1500u);
        CHECK_EQ_U(c->commission_time_ms[ch], wheel ? 0x7FFFFFFFu / 1000u : 2000u);
#else
        (void)wheel;
        CHECK_EQ_U(c->commission_duty_max_e4[ch], 1500u);
        CHECK_EQ_U(c->commission_time_ms[ch], 2000u);
#endif
    }
    printf("commission limits e4 %u %u %u %u ms %lu %lu %lu %lu\n", c->commission_duty_max_e4[0],
           c->commission_duty_max_e4[1], c->commission_duty_max_e4[2], c->commission_duty_max_e4[3],
           (unsigned long)c->commission_time_ms[0], (unsigned long)c->commission_time_ms[1],
           (unsigned long)c->commission_time_ms[2], (unsigned long)c->commission_time_ms[3]);
    CHECK(kth_raw_zero_outside_range(&c->steer));
    CHECK(kth_raw_zero_outside_range(&c->lift));
    /* Raw zero inside the steering range is refused by the board rules (not by the core). */
    drive_config bad = *c;
    bad.steer.zero_offset_rad = 0.0f;
    CHECK(drive_config_valid(&bad));
    CHECK(!board_config_valid(&bad));
    /* Outputs open only in a commissioning configuration (motor output design 2) or a normal one
       with the user's e-stop waiver (plan 2026-11-18). */
    drive_config open = *c;
    open.outputs_locked = false;
    open.commission_build = false;
    open.hil_plant = false;
    open.estop_waived = false;
    open.commission_single_axis = false; /* a normal build: single-axis is not for it */
#if defined(FW_CORE_BENCH)
    open.bench_build = false; /* nor the bench image's overrides (diag/fw-bench) */
#endif
    CHECK(!board_config_valid(&open));
    open.estop_waived = true;
    CHECK(board_config_valid(&open));
    open.estop_waived = false;
    open.commission_build = true;
    CHECK(board_config_valid(&open));
    return FW_TEST_RESULT();
}
