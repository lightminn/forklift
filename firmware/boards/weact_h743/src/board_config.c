/* The WeAct H743 board's drive_config (board design 3.7). Vehicle values come from the header
   tools/real_vehicle_header.py generates from config/real/vehicle.yaml (never written twice);
   every other value carries its source. "S<n> 실측 대기" = provisional until that bring-up step
   measures it (plan 5) -- none of these is a measured robot value. */
#include <string.h>

#include "board/board_config.h"
#include "vehicle_config.h"

#if defined(FW_BUILD_commission)
#define BOARD_COMMISSION true
#else
#define BOARD_COMMISSION false
#endif
#if defined(FW_BUILD_hil)
#define BOARD_HIL true
#else
#define BOARD_HIL false
#endif

#define PI_F 3.14159265358979323846f

static drive_config make(void)
{
    drive_config c;
    memset(&c, 0, sizeof c);
    c.period_us = 1000;         /* plan 3.3: 1 kHz */
    c.command_timeout_ms = 150; /* plan 3.4: the longest command validity */
    c.stop_grace_ms = 300;      /* plan 3.4: <= 300 ms; S5 실측 대기 */
    c.driver_retry_ms = 100;    /* board choice */
    c.driver_wake_us = 1000;    /* DRV8244 tREADY max 1 ms (SLVSG24C) */
    c.spi_timeout_us = 3000;    /* >= 2 periods (core design 3) */
    c.status_poll_ms = 20;      /* each channel's FAULT_SUMMARY every 20 ms */
    c.overrun_limit = 5;        /* board choice */
    c.adc_invalid_limit = 5;    /* board choice */
    c.wheel[0] =
        (wheel_enc_config){.counts_per_rev = FW_WHEEL_COUNTS_PER_REV_L, .window = 5, .sign = 1};
    c.wheel[1] =
        (wheel_enc_config){.counts_per_rev = FW_WHEEL_COUNTS_PER_REV_R, .window = 5, .sign = 1};
    /* Gains: deliberately low starting values (core design 7); S4 실측 대기. */
    c.wheel_pi = (pi_config){
        .kp = 0.02f, .ki = 0.1f, .kff = 1.0f / (FW_MAX_WHEEL_RAD_S * 1.25f), .out_max = 0.3f};
    c.steer_pi = (pi_config){.kp = 1.0f, .out_max = 0.2f};
    c.lift_pi = (pi_config){.kp = 2.0f, .out_max = 0.2f};
    /* KTH7801 axes: the CRC bit order is chosen on first hardware (S2), so is each axis's zero.
       Joint zero is placed half a turn from sensor 0 so that a stuck-low MISO (frame 0x0000,
       which passes the CRC) reads out of range (board design 3.7). S2 실측 대기. */
    c.steer = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                                .zero_offset_rad = PI_F,
                                .sign = 1,
                                .sensor_per_joint = 1.0f,
                                .min_joint = FW_STEER_MIN_RAD,
                                .max_joint = FW_STEER_MAX_RAD,
                                .invalid_limit = 3};
    /* Lift: the measurement scheme is undecided (plan 3.5); empty forks, less than one turn. */
    c.lift = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                               .zero_offset_rad = PI_F,
                               .sign = 1,
                               .sensor_per_joint = 10.0f,
                               .min_joint = -0.01f,
                               .max_joint = 0.25f,
                               .invalid_limit = 3};
    c.steer_rate_max = 0.5f; /* rad/s, S4 실측 대기 */
    c.lift_rate_max = 0.02f; /* m/s, S4 실측 대기 */
    c.max_wheel_rad_s = FW_MAX_WHEEL_RAD_S;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c.duty_sign[ch] = 1;        /* S3 실측 대기 */
        c.safe_brake[ch] = false;   /* idle = cut; S4/S5 decide per axis */
        c.overcurrent_a[ch] = 5.0f; /* below the 10 A stall figure; S3 실측 대기 */
    }
    c.drv_config3 = 0x40; /* DRV8244 CONFIG3/4 as the core tests; S2 확인 */
    c.drv_config4 = 0x04;
    c.ipropi_ohm = 1000.0f;   /* schematic V1.0 R5/R6/R15/R12 */
    c.ipropi_gain = 4750.0f;  /* SLVSG24C typical, VQFN-HR */
    c.adc_vref_v = 3.3f;      /* VREF+ = VDDA (WeAct R9) */
    c.adc_full_scale = 65535; /* 16-bit ADC */
    c.overcurrent_hold_ms = 20;
    /* Build identity (board design 2.2). Outputs locked: IPROPI reaches the ADC pins without
       protection on schematic V1.0 (plan 5 precondition); opening them is the user's decision. */
    c.commission_build = BOARD_COMMISSION;
    c.hil_plant = BOARD_HIL; /* board design 9: the vehicle model fills the inputs */
#if defined(FW_MOTOR_OUTPUT_OPEN)
    c.outputs_locked = false; /* behind the protection record gate (motor output design 2) */
#else
    c.outputs_locked = true;
#endif
    c.estop_wired = false;          /* no e-stop net on schematic V1.0 */
#if defined(FW_ESTOP_WAIVED)
    c.estop_waived = true; /* behind the e-stop waiver record gate (plan 2026-11-18) */
#else
    c.estop_waived = false;
#endif
    /* Commissioning limits per axis (plan 3.4; design 2026-11-25-bench-commission-limits 1.1).
       Every build starts at 0.15 / 2 s. Only the open commissioning build (on the stand, behind
       the protection record gate) raises the wheels to duty 1.0 and the validator's ms
       ceiling, so the bring-up tool's arguments set how hard and how long; steering and lift
       stay at 0.15 / 2 s. The locked commissioning build keeps 2 s on the wheels for
       rearm_hold_check. */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c.commission_duty_max_e4[ch] = 1500;
        c.commission_time_ms[ch] = 2000;
    }
#if defined(FW_BUILD_commission) && defined(FW_MOTOR_OUTPUT_OPEN)
    c.commission_duty_max_e4[DRIVE_CH_L] = c.commission_duty_max_e4[DRIVE_CH_R] = 10000;
    c.commission_time_ms[DRIVE_CH_L] = c.commission_time_ms[DRIVE_CH_R] = 0x7FFFFFFFu / 1000u;
#endif
    /* fw-single (partial-parts plan 2026-11-25 PR-B): one axis per arming, the rest cut; M and H
       keep 0.15 / 2 s and drive without their KTH. The build refuses it on any other kind. */
#if defined(FW_COMMISSION_SINGLE)
    c.commission_single_axis = true;
#elif defined(FW_CORE_SINGLE_AXIS)
    c.commission_single_axis = false;
#endif
    /* DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the bench image takes BENCH_SET (session
       overrides of the range checks and the limits above); the defaults are the limits above. */
#if defined(FW_BENCH_IMAGE)
    c.bench_build = true;
#elif defined(FW_CORE_BENCH)
    c.bench_build = false;
#endif
    c.wheel_duty_slew_per_s = 2.0f; /* S1/S3 실측 대기 */
    /* Encoder fault detection (plan 3.3), all S3/S4 실측 대기. The gap limit keeps the largest
       real step unambiguous: 0.02 s x 15 rad/s x 327680 / 2pi = 15645 <= 16383. */
    c.enc_gap_max_us = 20000;
    c.enc_phys_max_rad_s = 15.0f;
    c.enc_jump_rad_s = FW_MAX_WHEEL_RAD_S * 1.5f;
    c.enc_stuck_duty = 0.1f;
    c.enc_stuck_ms = 200;
    c.enc_stuck_counts = 50;
    c.enc_polarity_duty = 0.1f;
    c.enc_polarity_ms = 50;
    c.enc_polarity_rise_rad_s = 0.5f;
    c.enc_polarity_persist_ms = 1000;
    return c;
}

const drive_config *board_drive_config(void)
{
    static drive_config cfg;
    static bool made;
    if (!made) {
        cfg = make();
        made = true;
    }
    return &cfg;
}
