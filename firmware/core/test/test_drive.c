#include "fw/bench_pwm.h"
#include "fw/drive.h"
#include "fw_test.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692
#define BOOT_ID 0xC0FFEE01u

/* ---- synthetic configuration (not hardware values) ---------------------------------- */
static drive_config make_config(void)
{
    drive_config c;
    memset(&c, 0, sizeof c);
    c.period_us = 1000;
    c.command_timeout_ms = 200;
    c.stop_grace_ms = 100;
    c.driver_retry_ms = 50;
    c.driver_wake_us = 1000;
    c.spi_timeout_us = 3000;
    c.status_poll_ms = 10;
    c.overrun_limit = 3;
    c.adc_invalid_limit = 5;
    for (int i = 0; i < 2; i++) {
        c.wheel[i] = (wheel_enc_config){.counts_per_rev = 4096, .window = 10, .sign = 1};
    }
    c.wheel_pi = (pi_config){.kp = 0.2f, .ki = 1.0f, .kff = 0.05f, .out_max = 1.0f};
    c.steer_pi = (pi_config){.kp = 2.0f, .ki = 0.0f, .kff = 0.0f, .out_max = 0.8f};
    c.lift_pi = (pi_config){.kp = 5.0f, .ki = 0.0f, .kff = 0.0f, .out_max = 0.8f};
    c.steer = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                                .zero_offset_rad = 0.0f,
                                .sign = 1,
                                .sensor_per_joint = 1.0f,
                                .min_joint = -0.5f,
                                .max_joint = 0.5f,
                                .invalid_limit = 3};
    c.lift = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                               .zero_offset_rad = 0.0f,
                               .sign = 1,
                               .sensor_per_joint = 10.0f,
                               .min_joint = -0.01f,
                               .max_joint = 0.3f,
                               .invalid_limit = 3};
    c.steer_rate_max = 1.0f;
    c.lift_rate_max = 0.1f;
    c.max_wheel_rad_s = 10.0f;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c.duty_sign[ch] = 1;
        c.safe_brake[ch] = false;
        c.overcurrent_a[ch] = 5.0f;
    }
    c.drv_config3 = 0x40;
    c.drv_config4 = 0x04;
    c.ipropi_ohm = 1500.0f;
    c.ipropi_gain = 4750.0f;
    c.adc_vref_v = 3.3f;
    c.adc_full_scale = 65535;
    c.overcurrent_hold_ms = 20;
    /* v2: a normal build with an e-stop pin; slew and encoder limits far from these tests'
       motions unless a test tightens them (synthetic values). */
    c.estop_wired = true;
    c.wheel_duty_slew_per_s = 1000.0f;
    c.enc_gap_max_us = 20000;
    c.enc_phys_max_rad_s = 50.0f;
    c.enc_jump_rad_s = 100.0f;
    c.enc_stuck_duty = 0.5f;
    c.enc_stuck_ms = 200;
    c.enc_stuck_counts = 4;
    c.enc_polarity_duty = 0.3f;
    c.enc_polarity_ms = 50;
    c.enc_polarity_rise_rad_s = 1.0f;
    c.enc_polarity_persist_ms = 400;
    return c;
}

/* Every axis's commissioning limits (design 2026-11-25-bench-commission-limits 1.1). */
static void set_cm_limits(drive_config *c, uint16_t duty_e4, uint32_t time_ms)
{
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c->commission_duty_max_e4[ch] = duty_e4;
        c->commission_time_ms[ch] = time_ms;
    }
}

/* ---- fake world --------------------------------------------------------------------- */
static drive_config cfg;
static drive d;
static drive_inputs in;
static drive_outputs out;
static uint8_t regs[DRIVE_CH_COUNT][0x0E];
static bool vm_ok = true;       /* false: SPI answers 0x0000 (no VM / no device) */
static bool spi_mute = false;   /* true: no response at all */
static bool spi_refuse = false; /* true: the HAL refuses to start transactions */
static int clr_flt_seen;        /* CLR_FLT frames sent while every DRVOFF was high */
static int clr_flt_unsafe;      /* CLR_FLT frames sent with any DRVOFF low */
static bool nfault_after_clear[DRIVE_CH_COUNT]; /* nFAULT level a CLR_FLT leaves */
/* bench7: the board's output rule and report (fw/bench_pwm.h) in place of a HAL, when a test turns it on */
static bool pwm_sim;
static int32_t wheel_start_off[2]; /* bench8: a wheel's read starts this long after the tick time */
static bench_pwm_model pwm_m;

static uint16_t kth_frame(double sensor_rad)
{
    double turns = sensor_rad / TWO_PI;
    long raw = lround((turns - floor(turns)) * 4096.0) % 4096;
    uint16_t a = (uint16_t)raw;
    return (uint16_t)((a << 4) | crc4_kth7801(a, KTH_CRC_G704_BYTES));
}

static void set_steer(double rad)
{
    in.abs_frame[0] = kth_frame(rad);
}

static void set_lift(double m)
{
    in.abs_frame[1] = kth_frame(m * 10.0);
}

static uint32_t adc_for(double amps)
{
    double volts = amps / 4750.0 * 1500.0;
    return (uint32_t)lround(volts / 3.3 * 65535.0);
}

static uint16_t respond(uint8_t ch, uint16_t frame)
{
    if (!vm_ok) {
        return 0x0000;
    }
    uint8_t addr = (uint8_t)((frame >> 8) & 0x3Fu);
    bool read = (frame & 0x4000u) != 0;
    if (!read && addr < sizeof regs[0]) {
        regs[ch][addr] = (uint8_t)frame;
        if (addr == DRV8244_REG_COMMAND && (frame & 0x80u)) { /* CLR_FLT */
            bool all_off = true;
            for (int k = 0; k < DRIVE_CH_COUNT; k++) {
                all_off = all_off && out.drvoff[k];
            }
            all_off ? clr_flt_seen++ : clr_flt_unsafe++;
            in.nfault_low[ch] = nfault_after_clear[ch];
        }
    }
    uint8_t report = (read && addr < sizeof regs[0]) ? regs[ch][addr] : 0;
    return (uint16_t)(0xC000u | report);
}

/* One control cycle; messages are handled between begin and finish. */
static void tick_msgs(const link_msg *msgs, int count)
{
    in.now_us += cfg.period_us;
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0]); /* wheels idle unless a test moves them */
    in.abs_time_us[0] = in.abs_time_us[1] = in.now_us; /* acquired this tick */
    in.wheel_start_us[0] = in.now_us + (uint32_t)wheel_start_off[0]; /* bench8: each wheel's read time */
    in.wheel_start_us[1] = in.now_us + (uint32_t)wheel_start_off[1];
    if (pwm_sim) { /* an update between every two calls; the bundle began 50 us before this tick */
        in.abs_start_us[0] = in.abs_start_us[1] = in.now_us - 50u;
        in.abs_start_known[0] = in.abs_start_known[1] = true;
        bench_pwm_read(&pwm_m.t, true, in.now_us, &in);
    }
    drive_begin(&d, &in);
    for (int i = 0; i < count; i++) {
        drive_handle(&d, &msgs[i]);
    }
    drive_finish(&d, &out);
    if (pwm_sim) {
        bench_pwm_model_apply(&pwm_m, &out, true, in.now_us);
    }
    uint8_t ch;
    uint32_t tag;
    uint16_t frame;
    if (!spi_refuse && drive_spi_next(&d, &ch, &frame, &tag)) {
        drive_spi_started(&d);
        if (!spi_mute) {
            drive_spi_result(&d, ch, respond(ch, frame), tag);
        }
    }
}

static void tick(void)
{
    tick_msgs(NULL, 0);
}

static void ticks(int n)
{
    for (int i = 0; i < n; i++) {
        tick();
    }
}

static void reset_world(uint32_t t0)
{
    cfg = make_config();
    memset(&in, 0, sizeof in);
    memset(&out, 0, sizeof out);
    memset(regs, 0, sizeof regs);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        regs[ch][DRV8244_REG_CONFIG3] = 0x40;
        regs[ch][DRV8244_REG_CONFIG4] = 0x04;
        nfault_after_clear[ch] = false;
    }
    vm_ok = true;
    spi_mute = false;
    spi_refuse = false;
    pwm_sim = false;
    wheel_start_off[0] = wheel_start_off[1] = 0;
    bench_pwm_model_init(&pwm_m);
    clr_flt_seen = clr_flt_unsafe = 0;
    in.now_us = t0;
    in.abs_fresh[0] = in.abs_fresh[1] = true;
    in.adc_fresh = true;
    set_steer(0.0);
    set_lift(0.0);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        in.nfault_low[ch] = true; /* nFAULT is low after wake until CLR_FLT */
    }
}

static void boot(uint32_t t0)
{
    reset_world(t0);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
}

/* ---- message builders --------------------------------------------------------------- */
static uint64_t echo_now(void)
{
    return d.time_us; /* the STATE of the previous cycle */
}

static link_msg arm_msg(uint16_t seq, uint32_t boot_id, uint64_t echo)
{
    link_msg m = {.type = LINK_ARM, .seq = seq};
    m.u.arm = (link_arm){.boot_id = boot_id, .echo_mcu_time_us = echo};
    return m;
}

static link_msg drive_msg(uint16_t seq, float l, float r, float steer, uint16_t valid,
                          uint64_t echo)
{
    link_msg m = {.type = LINK_DRIVE, .seq = seq};
    m.u.drive = (link_drive){.left_wheel_rad_s = l,
                             .right_wheel_rad_s = r,
                             .steering_rad = steer,
                             .lift_m = 0.0f,
                             .valid_after_echo_ms = valid,
                             .flags = LINK_DRIVE_FLAG_LIFT_HOLD,
                             .arm_gen = d.arm_gen,
                             .boot_id = d.boot_id,
                             .echo_mcu_time_us = echo};
    return m;
}

static link_msg simple(uint8_t type, uint16_t seq)
{
    link_msg m = {.type = type, .seq = seq};
    m.u.clear.boot_id = d.boot_id;
    return m;
}

static void send(link_msg m)
{
    tick_msgs(&m, 1);
}

static void arm_and_zero(void)
{
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    send(drive_msg(2, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
}

static bool all_drvoff(void)
{
    return out.drvoff[0] && out.drvoff[1] && out.drvoff[2] && out.drvoff[3];
}

/* ---- tests -------------------------------------------------------------------------- */
static void boot_and_retry(void)
{
    reset_world(0);
    vm_ok = false;
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    ticks(5);
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    CHECK(all_drvoff());
    send(arm_msg(1, BOOT_ID, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_DRIVER_NOT_READY);
    CHECK(d.fault_flags & DRIVE_FAULT_DRIVER_NOT_READY);
    ticks(30);
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    vm_ok = true; /* battery switched on later: retried without a reset */
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(d.drivers_ready);
    CHECK_EQ_I(clr_flt_seen, 4);
    CHECK_EQ_I(clr_flt_unsafe, 0);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK_EQ_U(regs[ch][DRV8244_REG_COMMAND], 0x0A); /* locked at the end */
    }

    reset_world(0);
    spi_mute = true;
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    ticks(30);
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    spi_mute = false;
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
}

static void arming(void)
{
    boot(0);
    set_steer(0.1);
    tick();
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(d.arm_gen, 1);
    CHECK(!all_drvoff());
    /* ARM alone: hold the measured steering, wheels idle. */
    for (int i = 0; i < 100; i++) {
        tick();
    }
    CHECK_NEAR(d.steer_ref, 0.1, 1e-3);
    CHECK_NEAR(out.duty[DRIVE_CH_L], 0.0, 1e-6);
    CHECK_NEAR(out.duty[DRIVE_CH_M], 0.0, 1e-2);
    /* First DRIVE must stop both wheels. */
    send(drive_msg(5, 1.0f, 1.0f, 0.1f, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ZERO_FIRST);
    send(drive_msg(6, 0, 0, 0.1f, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(drive_msg(7, 1.0f, 1.0f, 0.1f, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK(out.duty[DRIVE_CH_L] > 0.0f);
}

static void arming_needs_valid_sensors(void)
{
    boot(0);
    in.abs_fresh[0] = false;
    ticks(3); /* invalid_limit */
    CHECK(d.fault_flags & DRIVE_FAULT_STEER_SENSOR_INVALID);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED); /* only a status bit while disarmed */
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    in.abs_fresh[0] = true;
    set_steer(0.7); /* out of range */
    tick();
    send(arm_msg(2, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    set_steer(0.0);
    in.adc_fresh = false;
    ticks(5);
    send(arm_msg(3, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    in.adc_fresh = true;
    tick();
    send(arm_msg(4, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}

static void arm_rejection_order(void)
{
    /* Wrong boot id always wins; otherwise the mode decides. */
    boot(0);
    send(arm_msg(1, d.boot_id + 1, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    arm_and_zero();
    send(arm_msg(3, d.boot_id + 1, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    send(arm_msg(4, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_DISARMED);
    ticks(160); /* deadline passes -> STOPPING */
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    send(arm_msg(5, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    in.estop_active = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    in.estop_active = false;
    tick();
    send(arm_msg(6, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_LATCHED);
    send(arm_msg(7, d.boot_id, d.time_us + 5000)); /* echo from the future */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
}

static void freshness_and_deadline(uint32_t t0)
{
    boot(t0);
    ticks(300); /* enough history for an echo 250 ms old */
    arm_and_zero();
    /* echo 250 ms old: echo + min(300, 200) already passed. */
    uint64_t old = d.time_us - 250000;
    send(drive_msg(10, 0, 0, 0, 300, old));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
    /* Other generation / boot. */
    link_msg m = drive_msg(11, 0, 0, 0, 100, echo_now());
    m.u.drive.arm_gen = (uint16_t)(d.arm_gen + 1);
    send(m);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ARM_GEN_MISMATCH);
    m = drive_msg(12, 0, 0, 0, 100, echo_now());
    m.u.drive.boot_id = d.boot_id ^ 1u;
    send(m);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    /* echo 30 ms old, 50 ms validity: deadline = echo + 50 ms = now + 20 ms. */
    uint64_t echo = d.time_us - 30000 + cfg.period_us; /* received during the next cycle */
    send(drive_msg(13, 0, 0, 0, 50, echo));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    uint64_t deadline = echo + 50000;
    while (d.time_us + cfg.period_us <= deadline) {
        tick();
        CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* the deadline instant is still valid */
    }
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT);
}

static void expiry_wins_over_a_late_drive(void)
{
    boot(0);
    arm_and_zero();
    uint64_t echo = echo_now();
    send(drive_msg(20, 0, 0, 0, 30, echo));
    ticks(29);
    /* Arrives in the cycle where the deadline has passed. */
    in.now_us += 1000;
    link_msg late = drive_msg(21, 0, 0, 0, 150, echo_now());
    tick_msgs(&late, 1);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
}

static void stopping_keeps_outputs_on(void)
{
    boot(0);
    arm_and_zero();
    send(drive_msg(30, 2.0f, 2.0f, 0, 150, echo_now()));
    /* Make the wheels turn so the stop has something to brake. */
    for (int i = 0; i < 160; i++) {
        in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 1);
        in.wheel_hw[1] = (uint16_t)(in.wheel_hw[1] + 1);
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    for (int i = 0; i < 50; i++) {
        in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 1);
        in.wheel_hw[1] = (uint16_t)(in.wheel_hw[1] + 1);
        tick();
        CHECK(!all_drvoff());
    }
    CHECK(out.duty[DRIVE_CH_L] < 0.0f); /* braking against the measured forward rate */
    ticks(60);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(all_drvoff());
}

static void seq_rules(void)
{
    boot(0);
    send(arm_msg(1, d.boot_id, echo_now()));
    send(drive_msg(100, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(drive_msg(100, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_SEQ);
    send(drive_msg(101, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(drive_msg(0xFFFF, 0, 0, 0, 150, echo_now()));
    send(drive_msg(0, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_SEQ); /* 0xFFFF is older than 101 */
    send(drive_msg(102, 0, 0, 0, 150, echo_now()));
    send(drive_msg(0x8000, 0, 0, 0, 150, echo_now()));
    send(drive_msg(0xFFFF, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(drive_msg(0, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE); /* wraps */
    send(simple(LINK_DISARM, 7));
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(drive_msg(1, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_ARMED);
}

static void replay_after_reboot(void)
{
    boot(0);
    uint32_t old_boot = d.boot_id;
    link_msg a = arm_msg(1, old_boot, echo_now());
    link_msg z = drive_msg(2, 0, 0, 0, 150, echo_now());
    link_msg go = drive_msg(3, 1.0f, 1.0f, 0, 150, echo_now());
    z.u.drive.arm_gen = go.u.drive.arm_gen = 1;
    /* MCU resets with a new boot id; the old bundle arrives late. */
    reset_world(0);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID + 7, DRIVE_RESET_POWER));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
    a.u.arm.echo_mcu_time_us = z.u.drive.echo_mcu_time_us = go.u.drive.echo_mcu_time_us =
        echo_now();
    send(a);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    send(z);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    send(go);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
}

static void estop_latch_and_clear(void)
{
    boot(0);
    cfg.safe_brake[DRIVE_CH_H] = true;
    arm_and_zero();
    in.estop_active = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    CHECK(out.drvoff[DRIVE_CH_L] && out.drvoff[DRIVE_CH_R] && out.drvoff[DRIVE_CH_M]);
    CHECK(!out.drvoff[DRIVE_CH_H]); /* brake: DRVOFF low, duty 0 */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK_NEAR(out.duty[ch], 0.0, 0.0);
    }
    send(simple(LINK_CLEAR_FAULTS, 9));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ESTOP_ACTIVE);
    in.estop_active = false;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    send(arm_msg(10, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_LATCHED);
    int before = clr_flt_seen;
    send(simple(LINK_CLEAR_FAULTS, 11));
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    CHECK(all_drvoff());
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_I(clr_flt_seen - before, 4);
    CHECK_EQ_I(clr_flt_unsafe, 0);
    send(arm_msg(12, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}

static void driver_fault_and_reinit(void)
{
    boot(0);
    cfg.safe_brake[DRIVE_CH_H] = true;
    arm_and_zero();
    in.nfault_low[DRIVE_CH_M] = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_DRIVER(DRIVE_CH_M));
    CHECK(out.drvoff[DRIVE_CH_M]);
    CHECK(!out.drvoff[DRIVE_CH_H]);
    /* nFAULT still low: CLEAR is accepted and decided by the re-initialisation. */
    nfault_after_clear[DRIVE_CH_M] = true;
    send(simple(LINK_CLEAR_FAULTS, 3));
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_DRIVER_NOT_READY);
    nfault_after_clear[DRIVE_CH_M] = false;
    send(simple(LINK_CLEAR_FAULTS, 4));
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);

    /* A status poll answered without 11 is a driver fault of that channel. */
    vm_ok = false;
    ticks(50);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & (DRIVE_FAULT_DRIVER(0) | DRIVE_FAULT_DRIVER(1) | DRIVE_FAULT_DRIVER(2) |
                           DRIVE_FAULT_DRIVER(3)));
}

static void overcurrent(void)
{
    boot(0);
    arm_and_zero();
    /* Keep commands fresh while the current is high. */
    in.adc_raw[DRIVE_CH_R] = adc_for(5.1);
    for (int i = 0; i < 19; i++) {
        send(drive_msg((uint16_t)(40 + i), 0, 0, 0, 150, echo_now()));
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* 19 ms < 20 ms hold */
    tick();
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_OVERCURRENT(DRIVE_CH_R));
    /* After a high-side OCP the IPROPI pin stays clamped high with the outputs off:
       CLEAR is not blocked by it, and no overcurrent is judged while not driving. */
    in.adc_raw[DRIVE_CH_R] = adc_for(10.0);
    send(simple(LINK_CLEAR_FAULTS, 70));
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_OVERCURRENT(DRIVE_CH_R)));
}

static void current_sense_and_sensor_faults(void)
{
    boot(0);
    arm_and_zero();
    in.adc_fresh = false;
    for (int i = 0; i < 5; i++) {
        send(drive_msg((uint16_t)(50 + i), 0, 0, 0, 150, echo_now()));
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_CURRENT_SENSE_INVALID);

    boot(0);
    arm_and_zero();
    in.abs_fresh[0] = false;
    ticks(3);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_STEER_SENSOR_INVALID);

    boot(0);
    arm_and_zero();
    set_steer(0.6);
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_STEER_OUT_OF_RANGE);
}

static void simultaneous_causes(void)
{
    boot(0);
    arm_and_zero();
    ticks(149); /* now == deadline (still valid); the next cycle is past it */
    in.estop_active = true;
    in.nfault_low[DRIVE_CH_L] = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    CHECK(d.fault_flags & DRIVE_FAULT_ESTOP_INPUT);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT);
    CHECK(d.fault_flags & DRIVE_FAULT_DRIVER(DRIVE_CH_L));
    CHECK(out.drvoff[DRIVE_CH_L]);
}

static void overruns(void)
{
    boot(0);
    arm_and_zero();
    in.cycle_overrun = true;
    send(drive_msg(60, 0, 0, 0, 150, echo_now()));
    send(drive_msg(61, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    send(drive_msg(62, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_CYCLE_OVERRUN);
}

static void config_validation(void)
{
    drive_config c = make_config();
    CHECK(drive_config_valid(&c));
    c.drv_config3 = 0x41;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.duty_sign[DRIVE_CH_M] = 0;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.steer.min_joint = -3.0f;
    c.steer.max_joint = 3.0f;
    c.steer.sensor_per_joint = 1.1f;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.command_timeout_ms = 65536;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.adc_invalid_limit = 0;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.period_us = 0;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.period_us = 999; /* contract 3.3: at most one KTH count per axis per ms */
    CHECK(!drive_config_valid(&c));
    /* The e-stop waiver only means something for an open normal build without a pin. */
    c = make_config();
    c.estop_wired = false;
    c.estop_waived = true;
    CHECK(drive_config_valid(&c));
    c.estop_wired = true;
    CHECK(!drive_config_valid(&c));
    c.estop_wired = false;
    c.outputs_locked = true;
    CHECK(!drive_config_valid(&c));
    c.outputs_locked = false;
    c.commission_build = true;
    set_cm_limits(&c, 1500, 2000);
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.ipropi_ohm = NAN;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    drive dd;
    drive_inputs first;
    memset(&first, 0, sizeof first);
    CHECK(!drive_init(&dd, &c, &first, 0, DRIVE_RESET_POWER));
}

static void control_directions_and_limits(void)
{
    boot(0);
    cfg.duty_sign[DRIVE_CH_L] = -1;
    arm_and_zero();
    send(drive_msg(80, 2.0f, 2.0f, 0.9f, 150, echo_now())); /* steering beyond 0.5 */
    CHECK(d.limited);
    CHECK_NEAR(d.steer_target, 0.5, 1e-6);
    CHECK(out.duty[DRIVE_CH_R] > 0.0f);
    CHECK(out.duty[DRIVE_CH_L] < 0.0f); /* same command, reversed channel */
    float before = d.steer_ref;
    tick();
    CHECK_NEAR(d.steer_ref - before, cfg.steer_rate_max * 0.001, 1e-6);
}

static void state_report(void)
{
    boot(0);
    in.adc_raw[DRIVE_CH_L] = adc_for(1.234);
    in.abs_fresh[1] = false;
    ticks(3);
    link_state s;
    memset(&s, 0xA5, sizeof s);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.boot_id, BOOT_ID);
    CHECK_EQ_U(s.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_I(s.current_ma[DRIVE_CH_L], 1234);
    CHECK(isnan(s.lift_m));
    CHECK(!isnan(s.steering_rad));
    CHECK(s.flags & DRIVE_STATE_DRIVERS_READY);
    CHECK(s.fault_flags & DRIVE_FAULT_LIFT_SENSOR_INVALID);
    in.adc_raw[DRIVE_CH_L] = adc_for(40.0);
    tick();
    drive_fill_state(&d, &s);
    CHECK_EQ_I(s.current_ma[DRIVE_CH_L], 32767); /* saturates */
}

/* v3 (contract 3.3): STATE carries the per-axis KTH counters, mod 2^16; only boot clears them. */
static void kth_counters_in_state(void)
{
    boot(0);
    link_state s0, s1;
    drive_fill_state(&d, &s0);
    CHECK_EQ_U(s0.kth_frames[0], (uint16_t)d.steer.frames);
    CHECK(s0.kth_frames[0] > 0); /* the boot ticks read both axes */
    in.abs_fresh[1] = false;
    ticks(5);
    drive_fill_state(&d, &s1);
    CHECK_EQ_U((uint16_t)(s1.kth_frames[0] - s0.kth_frames[0]), 5);
    CHECK_EQ_U((uint16_t)(s1.kth_frames[1] - s0.kth_frames[1]), 0);
    CHECK_EQ_U(s1.kth_crc_errors[0], s0.kth_crc_errors[0]);

    in.abs_frame[0] ^= 0x1u; /* angle 0: the two variants agree, so both count the error */
    tick();
    drive_fill_state(&d, &s0);
    CHECK_EQ_U((uint16_t)(s0.kth_crc_errors[0] - s1.kth_crc_errors[0]), 1);
    CHECK_EQ_U((uint16_t)(s0.kth_alt_crc_errors[0] - s1.kth_alt_crc_errors[0]), 1);
    set_steer(0.0);
    in.abs_fresh[1] = true;

    uint32_t frames = d.steer.frames, errs = d.steer.crc_errors;
    arm_and_zero();
    send(simple(LINK_DISARM, 3));
    send(simple(LINK_CLEAR_FAULTS, 4));
    CHECK_EQ_U(d.steer.frames, frames + 4); /* four more ticks, nothing reset */
    CHECK_EQ_U(d.steer.crc_errors, errs);

    d.steer.frames = 0x12345u; /* the wire carries the low 16 bits */
    d.lift.alt_crc_errors = 0x1FFFFu;
    drive_fill_state(&d, &s1);
    CHECK_EQ_U(s1.kth_frames[0], 0x2345);
    CHECK_EQ_U(s1.kth_alt_crc_errors[1], 0xFFFF);
}

static void clock_wrap_replay(void)
{
    /* Same boot, same arming: a DRIVE whose echo is 2^32 us + 10 ms old would look 10 ms
       old on a 32-bit clock. The 64-bit MCU time refuses it (Codex round 3). */
    boot(0);
    for (int i = 0; i < 3; i++) { /* jump the HAL clock by ~2^31 us three times */
        in.now_us += 0x7FFFFFFFu - cfg.period_us;
        tick();
    }
    CHECK(d.time_us > (1ull << 32));
    /* v2: a read gap that long loses wheel counts; clear it to reach the 64-bit check. */
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
    send(simple(LINK_CLEAR_FAULTS, 89));
    for (int i = 0; i < 200 && d.mode != DRIVE_MODE_DISARMED; i++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    arm_and_zero();
    uint64_t aliased = d.time_us - (1ull << 32) + 10000;
    send(drive_msg(90, 0, 0, 0, 150, aliased));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
}

static void spi_refusal_and_timeout(void)
{
    reset_world(0);
    spi_refuse = true;
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    ticks(100);
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT); /* refused starts time out and retry */
    spi_refuse = false;
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    /* Once ready, a status poll the HAL never starts is a driver fault. */
    spi_refuse = true;
    ticks(20);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    drive_config c = make_config();
    c.spi_timeout_us = 2 * c.period_us - 1;
    CHECK(!drive_config_valid(&c));
}

static void arm_alone_expires(void)
{
    boot(0);
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    ticks(198); /* echo + 200 ms is still valid */
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    ticks(2);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
}

static void fault_on_the_last_stopping_cycle_latches(void)
{
    /* Codex code review P1: a fault reaching its threshold in the cycle STOPPING ends must
       still latch, so the next ARM needs a CLEAR. */
    boot(0);
    arm_and_zero();
    ticks(149); /* deadline = now (still valid) */
    tick();     /* STOPPING starts */
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    ticks(97); /* two cycles before the grace ends */
    in.abs_fresh[0] = false;
    ticks(3); /* third failure lands on the cycle the grace ends */
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    in.abs_fresh[0] = true;
    tick();
    send(arm_msg(5, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_LATCHED);
}

/* One cycle without the automatic SPI handling of tick(). */
static void manual_cycle(const link_msg *m)
{
    in.now_us += cfg.period_us;
    drive_begin(&d, &in);
    if (m) {
        drive_handle(&d, m);
    }
    drive_finish(&d, &out);
}

static void stale_spi_response_after_reinit_is_drained(void)
{
    /* Codex code review P2: a status read of channel L that is still unanswered when a CLEAR
       restarts the initialisation must not be taken as the answer to the new L UNLOCK. */
    boot(0);
    spi_mute = true; /* status reads go out but are not answered (yet) */
    for (int i = 0; i < 40 && !(d.spi_inflight && d.spi_ch == DRIVE_CH_L); i++) {
        tick();
    }
    CHECK(d.spi_inflight && d.spi_ch == DRIVE_CH_L);
    uint32_t old_tag = d.spi_tag;
    in.nfault_low[DRIVE_CH_M] = true;
    manual_cycle(NULL);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    in.nfault_low[DRIVE_CH_M] = false;
    link_msg clear = simple(LINK_CLEAR_FAULTS, 3);
    manual_cycle(&clear);
    CHECK_EQ_U(d.mode, DRIVE_MODE_BOOT);
    manual_cycle(NULL); /* past the wake wait: the new L UNLOCK goes out */
    uint8_t ch;
    uint32_t tag;
    uint16_t frame;
    CHECK(drive_spi_next(&d, &ch, &frame, &tag));
    CHECK_EQ_U(frame, 0x0809);
    drive_spi_started(&d);
    drive_spi_result(&d, DRIVE_CH_L, 0xC000, old_tag); /* the old read answers: dropped */
    CHECK(d.spi_inflight);
    drive_spi_result(&d, DRIVE_CH_L, 0x0000, tag); /* the device's real (bad) UNLOCK answer */
    spi_mute = false;
    ticks(200);
    /* The bad UNLOCK answer fails this re-initialisation: the clear does not take. */
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_DRIVER_NOT_READY);
}

static void config_rejects_negative_gains_and_huge_zero(void)
{
    drive_config c = make_config();
    c.wheel_pi.kp = -1.0f;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.steer_pi.ki = -0.1f;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.lift_pi.kff = -0.1f;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.steer.zero_offset_rad = 1e9f; /* would stall a subtract-2pi loop */
    CHECK(!drive_config_valid(&c));
}

static void state_flags_follow_messages(void)
{
    /* Codex code review P2: STATE right after ARM / CLEAR reflects the new mode's bits. */
    boot(0);
    arm_and_zero();
    ticks(160); /* timeout -> STOPPING */
    ticks(100); /* -> DISARMED, COMMAND_TIMEOUT kept until the next ARM */
    CHECK(d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT);
    send(arm_msg(30, d.boot_id, echo_now()));
    link_state s;
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.mode, DRIVE_MODE_ARMED);
    CHECK(!(s.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT));

    in.estop_active = true;
    tick();
    in.estop_active = false;
    tick();
    send(simple(LINK_CLEAR_FAULTS, 31));
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.mode, DRIVE_MODE_BOOT);
    CHECK(!(s.fault_flags & DRIVE_FAULT_ESTOP_INPUT));
    CHECK(s.fault_flags & DRIVE_FAULT_DRIVER_NOT_READY);
}

static void refused_starts_do_not_cycle_spi_tags(void)
{
    /* Codex re-review P2: a stuck transaction makes the HAL refuse starts; queued-but-never-
       started frames must not use up tags, or the stuck one's tag comes round again. */
    boot(0);
    spi_mute = true;
    for (int i = 0; i < 40 && !(d.spi_inflight && d.spi_ch == DRIVE_CH_L); i++) {
        tick();
    }
    uint32_t old_tag = d.spi_tag;
    spi_refuse = true; /* the bus stays busy with the old read */
    ticks(800);        /* many polls queued, refused and timed out; FAULT on the way */
    CHECK_EQ_U(d.spi_tag, old_tag);
    link_msg clear = simple(LINK_CLEAR_FAULTS, 4);
    manual_cycle(&clear);
    manual_cycle(NULL);
    uint8_t ch;
    uint32_t tag;
    uint16_t frame;
    CHECK(drive_spi_next(&d, &ch, &frame, &tag));
    CHECK(tag != old_tag);
    drive_spi_started(&d);
    drive_spi_result(&d, DRIVE_CH_L, 0xC000, old_tag); /* the stuck read finally answers */
    CHECK(d.spi_inflight);
    drive_spi_result(&d, DRIVE_CH_L, 0x0000, tag); /* the real UNLOCK answer: bad */
    spi_refuse = false;
    spi_mute = false;
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
}

/* ---- mcu-link/2 (design 2026-11-11-mcu-link-v2-firmware-core) -------------------------- */
typedef void (*cfg_mod)(drive_config *c);

static double wheel_pos[2]; /* counts, vehicle-forward (sign +1 in the synthetic config) */

static void boot_with(cfg_mod mod)
{
    reset_world(0);
    if (mod) {
        mod(&cfg);
    }
    wheel_pos[0] = wheel_pos[1] = 0.0;
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
}

/* One cycle with the wheels turning at rate (rad/s); messages as in tick_msgs. */
static void tick_moving(double rate_l, double rate_r, const link_msg *msgs, int count)
{
    double rates[2] = {rate_l, rate_r};
    for (int i = 0; i < 2; i++) {
        wheel_pos[i] += rates[i] * (double)cfg.period_us * 1e-6 * 4096.0 / TWO_PI;
        in.wheel_hw[i] = (uint16_t)(long)llround(wheel_pos[i]);
    }
    tick_msgs(msgs, count);
}

static link_msg commission_msg(uint16_t seq, uint8_t axis, int16_t duty_e4, uint16_t valid,
                               uint64_t echo)
{
    link_msg m = {.type = LINK_COMMISSION, .seq = seq};
    m.u.commission = (link_commission){.axis = axis,
                                       .duty_e4 = duty_e4,
                                       .valid_after_echo_ms = valid,
                                       .arm_gen = d.arm_gen,
                                       .boot_id = d.boot_id,
                                       .echo_mcu_time_us = echo};
    return m;
}

static void mod_no_estop(drive_config *c)
{
    c->estop_wired = false;
}

static void mod_locked(drive_config *c)
{
    c->estop_wired = false;
    c->outputs_locked = true;
}

static void mod_hil(drive_config *c)
{
    mod_locked(c);
    c->hil_plant = true;
}

/* V1.0 e-stop waiver (plan 2026-11-18): open normal build, no pin, a committed user record. */
static void mod_waived(drive_config *c)
{
    c->estop_wired = false;
    c->estop_waived = true;
}

static void mod_commission(drive_config *c)
{
    c->estop_wired = false;
    c->commission_build = true;
    set_cm_limits(c, 1500, 2000);
    c->enc_stuck_duty = 0.05f;
    c->enc_polarity_duty = 0.05f;
}

static void v2_build_bits_and_estop_unwired(void)
{
    link_state s;
    boot_with(NULL);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u, 0);
    CHECK_EQ_U(s.reset_cause, DRIVE_RESET_POWER);

    boot_with(mod_no_estop); /* outputs open, no pin: ARM refused */
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ESTOP_UNWIRED);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u, DRIVE_STATE_ESTOP_UNWIRED);

    boot_with(mod_waived); /* outputs open, no pin, waived by the user's record: allowed */
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u, DRIVE_STATE_ESTOP_UNWIRED); /* the pin is still missing */

    boot_with(mod_locked); /* locked outputs: allowed */
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u, DRIVE_STATE_OUTPUTS_LOCKED | DRIVE_STATE_ESTOP_UNWIRED);

    boot_with(mod_hil);
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u,
               DRIVE_STATE_HIL_PLANT | DRIVE_STATE_OUTPUTS_LOCKED | DRIVE_STATE_ESTOP_UNWIRED);

    boot_with(mod_commission);
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.flags & 0xF0u, DRIVE_STATE_COMMISSION_BUILD | DRIVE_STATE_ESTOP_UNWIRED);

    /* The order: an active e-stop input is reported before the missing pin. */
    boot_with(mod_no_estop);
    in.estop_active = true;
    tick();
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_LATCHED);
}

static void v2_config_and_init(void)
{
    drive dd;
    drive_inputs first;
    memset(&first, 0, sizeof first);
    drive_config c = make_config();
    CHECK(drive_config_valid(&c));
    CHECK(!drive_init(&dd, &c, &first, BOOT_ID, DRIVE_RESET_OTHER + 1));
    CHECK(drive_init(&dd, &c, &first, BOOT_ID, DRIVE_RESET_OTHER));
    c.hil_plant = true; /* HIL must be locked */
    CHECK(!drive_config_valid(&c));
    c.outputs_locked = true;
    CHECK(drive_config_valid(&c));
    c.commission_build = true; /* and is not a commission build */
    set_cm_limits(&c, 1500, 2000);
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.commission_build = true;
    set_cm_limits(&c, 1500, 2000);
    CHECK(drive_config_valid(&c));
    c.commission_duty_max_e4[DRIVE_CH_H] = 0; /* every axis is checked */
    CHECK(!drive_config_valid(&c));
    c.commission_duty_max_e4[DRIVE_CH_H] = 10001;
    CHECK(!drive_config_valid(&c));
    c.commission_duty_max_e4[DRIVE_CH_H] = 10000;
    CHECK(drive_config_valid(&c));
    c.commission_time_ms[DRIVE_CH_R] = 0;
    CHECK(!drive_config_valid(&c));
    c.commission_time_ms[DRIVE_CH_R] = 0x7FFFFFFFu / 1000u; /* the validator's ms ceiling */
    CHECK(drive_config_valid(&c));
    c.commission_time_ms[DRIVE_CH_R] = 0x7FFFFFFFu / 1000u + 1u;
    CHECK(!drive_config_valid(&c));
    c = make_config(); /* a normal build ignores the zero limits it never uses */
    set_cm_limits(&c, 0, 0);
    CHECK(drive_config_valid(&c));
    /* gap x physical maximum must not fold in 16 bits: 0.5 s x 50 rad/s x 4096 / 2pi = 16297. */
    c = make_config();
    c.enc_gap_max_us = 500000;
    CHECK(drive_config_valid(&c));
    c.enc_gap_max_us = 510000;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.enc_gap_max_us = c.period_us; /* below two periods */
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.enc_polarity_persist_ms = 60; /* window 10 ms + rise 50 ms */
    CHECK(!drive_config_valid(&c));
    c.enc_polarity_persist_ms = 61;
    CHECK(drive_config_valid(&c));
    c = make_config();
    c.wheel_duty_slew_per_s = 0.0f;
    CHECK(!drive_config_valid(&c));
}

static void v2_wrong_build(void)
{
    boot_with(NULL);
    arm_and_zero();
    send(commission_msg(3, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
    link_msg bad = commission_msg(4, 9, 0, 150, echo_now()); /* before the format check */
    send(bad);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);

    boot_with(mod_commission);
    send(arm_msg(1, d.boot_id, echo_now()));
    send(drive_msg(2, 0, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
    link_msg nan_drive = drive_msg(3, NAN, 0, 0, 150, echo_now());
    send(nan_drive);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
}

static void v2_commission_checks_and_outputs(void)
{
    boot_with(mod_commission);
    in.abs_fresh[1] = false; /* no lift sensor: the wheels can still be commissioned */
    ticks(5);
    send(arm_msg(1, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    send(commission_msg(2, 4, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(3, 0, 1501, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(4, 0, -1501, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(5, 0, 0, 0, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(6, 0, 1000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ZERO_FIRST);
    send(commission_msg(7, 0, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(commission_msg(7, 0, 1000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_SEQ);
    send(commission_msg(8, 3, 500, 150, echo_now())); /* lift picked but not measurable */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* the unpicked lift does not fault */

    send(commission_msg(9, 0, 1000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_NEAR(out.duty[DRIVE_CH_L], 0.1, 1e-6);
    CHECK(!out.drvoff[DRIVE_CH_L]);
    CHECK(out.drvoff[DRIVE_CH_R] && out.drvoff[DRIVE_CH_M] && out.drvoff[DRIVE_CH_H]);
    CHECK_NEAR(out.duty[DRIVE_CH_R], 0.0, 0.0);

    /* Switching axis inside an output interval is refused (design 2026-11-25 1.1); after a
       zero-output cycle it is taken and L goes to its safe output. */
    send(commission_msg(10, 2, -500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_NEAR(out.duty[DRIVE_CH_L], 0.1, 1e-6);
    send(commission_msg(11, 0, 0, 150, echo_now()));
    CHECK(!d.cm_on);
    send(commission_msg(12, 2, -500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK(out.drvoff[DRIVE_CH_L]);
    CHECK_NEAR(d.wheel_duty[0], 0.0, 0.0);
    CHECK_NEAR(out.duty[DRIVE_CH_M], -0.05, 1e-6);

    /* Expiry: every duty 0 at once with the outputs on (electrical brake). */
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_NEAR(out.duty[DRIVE_CH_M], 0.0, 0.0);
    CHECK(!out.drvoff[DRIVE_CH_M]);
    ticks((int)cfg.stop_grace_ms + 1);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
}

static void v2_commission_duty_sign(void)
{
    boot_with(mod_commission);
    cfg.duty_sign[DRIVE_CH_R] = -1;
    send(arm_msg(1, d.boot_id, echo_now()));
    send(commission_msg(2, 1, 0, 150, echo_now()));
    send(commission_msg(3, 1, 1200, 150, echo_now()));
    CHECK_NEAR(out.duty[DRIVE_CH_R], -0.12, 1e-6);
    CHECK_NEAR(d.wheel_duty[1], 0.12, 1e-6); /* control coordinates */
}

/* Keeps axis L at duty_e4 by resending every 50 ms, the wheel turning so stuck stays quiet. */
static int commission_run(int ms, int16_t duty_e4, bool zero_in_same_cycle, uint16_t *seq)
{
    for (int t = 0; t < ms; t++) {
        if (t % 50 == 0) {
            link_msg m[2] = {commission_msg((*seq)++, 0, 0, 150, echo_now()),
                             commission_msg(0, 0, duty_e4, 150, echo_now())};
            m[1].seq = (*seq)++;
            if (zero_in_same_cycle) {
                tick_moving(2.0, 0.0, m, 2);
            } else {
                tick_moving(2.0, 0.0, &m[1], 1);
            }
        } else {
            tick_moving(2.0, 0.0, NULL, 0);
        }
        if (d.mode != DRIVE_MODE_ARMED) {
            return t;
        }
    }
    return -1;
}

static void v2_commission_time_limit(void)
{
    boot_with(mod_commission);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    int stopped = commission_run(3000, 1000, false, &seq);
    CHECK(stopped >= 1990 && stopped <= 2010);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    CHECK_NEAR(out.duty[DRIVE_CH_L], 0.0, 0.0);
    ticks((int)cfg.stop_grace_ms + 1);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED); /* not a fault */
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    send(arm_msg(seq++, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT));

    /* A zero slipped in before the non-zero duty in the same cycle does not reset the clock. */
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    stopped = commission_run(3000, 1000, true, &seq);
    CHECK(stopped >= 1990 && stopped <= 2010);

    /* One full cycle with every applied duty 0 restarts it. */
    ticks((int)cfg.stop_grace_ms + 1);
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    CHECK(commission_run(1500, 1000, false, &seq) < 0);
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    tick_moving(0.0, 0.0, NULL, 0);
    CHECK(commission_run(1500, 1000, false, &seq) < 0);
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}

static void v2_commission_encoder_reports_only(void)
{
    boot_with(mod_commission);
    send(arm_msg(1, d.boot_id, echo_now()));
    send(commission_msg(2, 0, 0, 150, echo_now()));
    uint16_t seq = 3;
    for (int t = 0; t < 400; t++) { /* counter frozen at duty 0.1 >= stuck duty 0.05 */
        if (t % 50 == 0) {
            send(commission_msg(seq++, 0, 1000, 150, echo_now()));
        } else {
            tick();
        }
    }
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(0));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    ticks(200 + (int)cfg.stop_grace_ms + 1);
    send(arm_msg(seq++, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(0)));
}

static void v2_gap_and_jump(void)
{
    /* The first cycle after init is not judged for a gap. */
    reset_world(0);
    wheel_pos[0] = wheel_pos[1] = 0.0;
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    in.now_us += 100000;
    tick();
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_GAP));
    CHECK(d.mode != DRIVE_MODE_FAULT);

    boot_with(NULL);
    uint32_t count0 = d.wheel[0].count;
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 50);
    in.now_us += 30000; /* > enc_gap_max_us 20 ms */
    tick();
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT); /* latched while disarmed */
    CHECK_EQ_U(d.wheel[0].count, count0); /* the step is dropped */
    CHECK_NEAR(d.wheel[0].rate_rad_s, 0.0, 0.0);
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 3); /* counting resumes from the new base */
    tick();
    CHECK_EQ_U(d.wheel[0].count, count0 + 3);

    /* Jump: 100 rad/s x 1 ms x 4096 / 2pi + 1 = 66.2 counts. */
    boot_with(NULL);
    in.wheel_hw[1] = (uint16_t)(in.wheel_hw[1] + 66);
    tick();
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(1)));
    in.wheel_hw[1] = (uint16_t)(in.wheel_hw[1] - 67);
    tick();
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(1));
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(0)));
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    send(simple(LINK_CLEAR_FAULTS, 9));
    for (int i = 0; i < 200 && d.mode != DRIVE_MODE_DISARMED; i++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(1)));

    /* A cycle right after a late one: the dt floor keeps 30 counts in 0.1 ms legal. */
    boot_with(NULL);
    in.now_us += 100 - cfg.period_us;                 /* tick() adds one period: 0.1 ms in all */
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 30); /* < 100 x 0.5 ms x 4096/2pi + 1 = 33.6 */
    tick();
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(0)));
}

static void arm_drive(double target)
{
    send(arm_msg(1, d.boot_id, echo_now()));
    send(drive_msg(2, 0, 0, 0, 150, echo_now()));
    send(drive_msg(3, (float)target, (float)target, 0, 150, echo_now()));
}

/* Keeps the DRIVE fresh with the wheels at rate; stops at the first fault. */
static int drive_run(int ms, double target, double rate, uint16_t *seq)
{
    for (int t = 0; t < ms; t++) {
        if (t % 50 == 0) {
            link_msg m = drive_msg((*seq)++, (float)target, (float)target, 0, 150, echo_now());
            tick_moving(rate, rate, &m, 1);
        } else {
            tick_moving(rate, rate, NULL, 0);
        }
        if (d.mode != DRIVE_MODE_ARMED) {
            return t;
        }
    }
    return -1;
}

static void v2_stuck(void)
{
    /* Frozen counter from the start, duty saturated. */
    boot_with(NULL);
    uint16_t seq = 4;
    arm_drive(10.0);
    int t = drive_run(1000, 10.0, 0.0, &seq);
    CHECK(t > 150 && t < 260);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(0));
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(1));

    /* Codex P1: running under high duty (heavy load), then the counter freezes. */
    boot_with(NULL);
    seq = 4;
    arm_drive(20.0);
    CHECK(drive_run(1000, 20.0, 5.0, &seq) < 0); /* progress keeps resetting the baseline */
    CHECK(fabsf(d.wheel_duty[0]) >= 0.5f);
    t = drive_run(1000, 20.0, 0.0, &seq);
    CHECK(t > 150 && t < 260);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(0));

    /* Below the stuck duty: a stalled wheel at low duty is not an encoder fault. */
    boot_with(NULL);
    cfg.wheel_pi.kp = 0.01f;
    cfg.wheel_pi.ki = 0.0f;
    cfg.wheel_pi.kff = 0.0f;
    seq = 4;
    arm_drive(10.0);
    CHECK(drive_run(1000, 10.0, 0.0, &seq) < 0);

    /* Locked outputs without HIL: wheels do not turn and that is normal. */
    boot_with(mod_locked);
    seq = 4;
    arm_drive(10.0);
    CHECK(drive_run(1000, 10.0, 0.0, &seq) < 0);
    /* HIL: the model turns on the computed duty, so a frozen model counter is a fault. */
    boot_with(mod_hil);
    seq = 4;
    arm_drive(10.0);
    CHECK(drive_run(1000, 10.0, 0.0, &seq) > 0);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_STUCK(0));
}

static void v2_polarity(void)
{
    /* A/B swapped: the wheel accelerates the wrong way under positive duty -> (a). */
    boot_with(NULL);
    uint16_t seq = 4;
    arm_drive(5.0);
    double rate = 0.0;
    int t;
    for (t = 0; t < 1000 && d.mode == DRIVE_MODE_ARMED; t++) {
        rate -= 0.05; /* -50 rad/s^2 */
        if (t % 50 == 0) {
            link_msg m = drive_msg(seq++, 5.0f, 5.0f, 0, 150, echo_now());
            tick_moving(rate, rate, &m, 1);
        } else {
            tick_moving(rate, rate, NULL, 0);
        }
    }
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_POLARITY(0));
    CHECK(t < 120); /* window 10 ms + a 1 rad/s rise */

    /* Reversed at constant speed: no rise, caught by persistence (b). */
    boot_with(NULL);
    seq = 4;
    arm_drive(5.0);
    t = drive_run(1000, 5.0, -3.0, &seq);
    CHECK(t >= 395 && t <= 420);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_POLARITY(0));

    /* Normal braking right after an acceleration, and a reversal both ways: no fault. */
    boot_with(NULL);
    seq = 4;
    arm_drive(5.0);
    rate = 0.0;
    double target = 5.0;
    for (t = 0; t < 1500; t++) {
        if (t == 300) {
            target = 0.0; /* brake */
        } else if (t == 600) {
            target = -5.0; /* reverse */
        } else if (t == 1000) {
            target = 5.0; /* and back */
        }
        double step = 0.05; /* +-50 rad/s^2 toward the target */
        rate += target > rate ? (target - rate < step ? target - rate : step)
                              : (rate - target < step ? target - rate : -step);
        if (t % 50 == 0) {
            link_msg m = drive_msg(seq++, (float)target, (float)target, 0, 150, echo_now());
            tick_moving(rate, rate, &m, 1);
        } else {
            tick_moving(rate, rate, NULL, 0);
        }
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(d.fault_flags & 0x00FE0000u, 0);
}

static void v2_slew(void)
{
    boot_with(NULL);
    cfg.wheel_duty_slew_per_s = 5.0f; /* 0.005 per 1 ms cycle */
    uint16_t seq = 4;
    arm_drive(10.0);
    float prev = d.wheel_duty[0];
    bool within = true;
    for (int t = 0; t < 100; t++) {
        tick_moving(0.5, 0.5, NULL, 0);
        within = within && fabsf(d.wheel_duty[0]) - fabsf(prev) <= 0.005f + 1e-6f;
        prev = d.wheel_duty[0];
        if (t % 50 == 0) {
            send(drive_msg(seq++, 10.0f, 10.0f, 0, 150, echo_now()));
            prev = d.wheel_duty[0];
        }
    }
    CHECK(within);
    CHECK(d.wheel_duty[0] > 0.3f && d.wheel_duty[0] < 0.6f); /* still ramping */
    CHECK_NEAR(d.wheel_pi[0].integral, 0.0, 0.0);            /* no wind-up behind the slew bound */

    /* Expiry from a high duty: STOPPING's first cycle may already brake (decrease is free). */
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK(d.wheel_duty[0] <= 0.0f);

    /* DISARM drops to the safe output at once. */
    boot_with(NULL);
    cfg.wheel_duty_slew_per_s = 5.0f;
    seq = 4;
    arm_drive(10.0);
    ticks(50);
    send(simple(LINK_DISARM, seq++));
    CHECK(all_drvoff());
    CHECK_NEAR(d.wheel_duty[0], 0.0, 0.0);
}

static void v2_state_carries_reset_cause(void)
{
    reset_world(0);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_IWDG));
    link_state s;
    drive_fill_state(&d, &s);
    CHECK_EQ_U(s.reset_cause, DRIVE_RESET_IWDG);
}

/* ---- Codex design round 2 ------------------------------------------------------------- */
static void v2r2_disarm_rearm_does_not_reset_commission_clock(void)
{
    boot_with(mod_commission);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    int t;
    for (t = 0; t < 3000 && d.mode == DRIVE_MODE_ARMED; t++) {
        if (t % 50 == 0) {
            link_msg m[4];
            int n = 0;
            if (t > 0 && t % 1500 == 0) { /* DISARM -> ARM -> 0 in the same cycle */
                m[n++] = simple(LINK_DISARM, seq++);
                m[n++] = arm_msg(seq++, d.boot_id, echo_now());
                m[n] = commission_msg(seq++, 0, 0, 150, echo_now());
                m[n++].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
            }
            m[n] = commission_msg(seq++, 0, 1000, 150, echo_now());
            if (n > 0) {
                m[n].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
            }
            n++;
            tick_moving(2.0, 0.0, m, n);
        } else {
            tick_moving(2.0, 0.0, NULL, 0);
        }
    }
    CHECK(t >= 1990 && t <= 2012);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
}

static void v2r2_first_cycle_gap_drops_the_step(void)
{
    reset_world(0);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    uint32_t c0 = d.wheel[0].count;
    in.now_us += 100000;                                 /* first cycle very late */
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 40000); /* would fold to -25536 */
    tick();
    CHECK_EQ_U(d.wheel[0].count, c0); /* dropped, no fault on the first cycle */
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_GAP));
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(0)));
    CHECK(d.mode != DRIVE_MODE_FAULT);
    /* A first cycle on time adds its step as usual. */
    reset_world(0);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 5);
    tick();
    CHECK_EQ_U(d.wheel[0].count, 5);
}

static void v2r2_polarity_baseline_waits_for_a_fresh_window(void)
{
    /* Configured 1 ms but running at 1.9 ms (no overrun): a 20-sample window spans 38 ms, so a
       baseline taken after 20 ms still holds the acceleration (Codex round 2). */
    boot_with(NULL);
    for (int i = 0; i < 2; i++) {
        cfg.wheel[i].window = 20;
    }
    cfg.enc_polarity_rise_rad_s = 0.05f;
    cfg.wheel_pi.ki = 0.0f; /* the duty turns negative the cycle the brake starts */
    cfg.wheel_pi.kff = 0.0f;
    uint16_t seq = 4;
    arm_drive(5.0);
    double rate = 0.0;
    for (int k = 0; k < 250 && d.mode == DRIVE_MODE_ARMED; k++) {
        double target = k < 50 ? 5.0 : 0.0;
        rate += k < 50 ? 0.04 : -0.001; /* up to 2 rad/s, then a slow brake */
        in.now_us += 1900 - cfg.period_us;
        for (int i = 0; i < 2; i++) {
            wheel_pos[i] += rate * 1.9e-3 * 4096.0 / TWO_PI;
            in.wheel_hw[i] = (uint16_t)(long)llround(wheel_pos[i]);
        }
        if (k % 25 == 0) {
            link_msg m = drive_msg(seq++, (float)target, (float)target, 0, 150, echo_now());
            tick_msgs(&m, 1);
        } else {
            tick();
        }
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_POLARITY(0)));
}

static void mod_commission_coast(drive_config *c)
{
    mod_commission(c);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c->safe_brake[ch] = false;
    }
}

static void v2r2_commission_stopping_brakes_every_axis(void)
{
    boot_with(mod_commission_coast);
    send(arm_msg(1, d.boot_id, echo_now()));
    send(commission_msg(2, 0, 0, 150, echo_now()));
    send(commission_msg(3, 0, 1000, 150, echo_now()));
    send(commission_msg(4, 0, 0, 150, echo_now()));   /* L driven, a zero cycle, then M */
    send(commission_msg(5, 2, 500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK(out.drvoff[DRIVE_CH_L]);                    /* unpicked in ARMED: its safe output */
    ticks(200);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK(!out.drvoff[ch]); /* every axis: duty 0 with the outputs on (brake) */
        CHECK_NEAR(out.duty[ch], 0.0, 0.0);
    }
}

/* ---- Codex PR review 1 ---------------------------------------------------------------- */
static void v2r3_rearm_bundle_after_the_limit_waits_the_stop(void)
{
    /* After the time limit fires, a DISARM -> ARM -> COMMISSION(0) -> COMMISSION(non-zero)
       bundle every cycle must not bring the output back before stop_grace_ms of zero output. */
    boot_with(mod_commission);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    CHECK(commission_run(3000, 1000, false, &seq) > 0);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    int zero_cycles = 0, first_nonzero = -1;
    for (int t = 0; t < 1000; t++) {
        link_msg m[4];
        m[0] = simple(LINK_DISARM, seq++);
        m[1] = arm_msg(seq++, d.boot_id, echo_now());
        m[2] = commission_msg(seq++, 0, 0, 150, echo_now());
        m[2].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        m[3] = commission_msg(seq++, 0, 1000, 150, echo_now());
        m[3].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        tick_moving(2.0, 0.0, m, 4);
        if (out.duty[DRIVE_CH_L] == 0.0f) {
            zero_cycles++;
        } else if (first_nonzero < 0) {
            first_nonzero = t;
        }
    }
    CHECK(first_nonzero >= (int)cfg.stop_grace_ms - 1);
    CHECK(zero_cycles >= (int)cfg.stop_grace_ms - 1);
}

static void v2r4_limit_and_expiry_in_the_same_cycle(void)
{
    /* The command deadline is placed exactly on the time limit, so both fire in one cycle;
       the re-arm bundle must still wait stop_grace_ms (Codex PR review 2). */
    boot_with(mod_commission);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 1000, 150, echo_now()));
    CHECK(d.cm_on);
    uint64_t limit_at = d.cm_on_since_us + (uint64_t)cfg.commission_time_ms[0] * 1000u;
    while (d.time_us + 150000u < limit_at) { /* keep it fresh until 150 ms before */
        if ((d.time_us / 1000u) % 50u == 0) {
            send(commission_msg(seq++, 0, 1000, 150, echo_now()));
        } else {
            tick();
        }
    }
    uint64_t echo = echo_now();
    link_msg last = commission_msg(seq++, 0, 1000, (uint16_t)((limit_at - echo) / 1000u), echo);
    send(last);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(d.deadline_us, limit_at);
    while (d.mode == DRIVE_MODE_ARMED) {
        tick();
    }
    CHECK(d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    int first_nonzero = -1;
    for (int t = 0; t < 400; t++) {
        link_msg m[4];
        m[0] = simple(LINK_DISARM, seq++);
        m[1] = arm_msg(seq++, d.boot_id, echo_now());
        m[2] = commission_msg(seq++, 0, 0, 150, echo_now());
        m[2].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        m[3] = commission_msg(seq++, 0, 1000, 150, echo_now());
        m[3].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        tick_moving(2.0, 0.0, m, 4);
        if (out.duty[DRIVE_CH_L] != 0.0f && first_nonzero < 0) {
            first_nonzero = t;
        }
    }
    CHECK(first_nonzero >= (int)cfg.stop_grace_ms - 1);
}

/* Runs commissioning up to the limit, then on the cycle that passes it injects a fault
   (0 = encoder gap, 1 = encoder jump); clears it and re-arms every cycle after. Returns
   whether output resumed before stop_grace_ms had passed since the limit (Codex PR review 3). */
static bool limit_with_fault_resumes_early(int kind)
{
    boot_with(mod_commission);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 1000, 150, echo_now()));
    uint64_t limit_at = d.cm_on_since_us + (uint64_t)cfg.commission_time_ms[0] * 1000u;
    while (d.time_us + 2000u < limit_at) {
        if ((d.time_us / 1000u) % 50u == 0) {
            send(commission_msg(seq++, 0, 1000, 150, echo_now()));
        } else {
            tick();
        }
    }
    if (kind == 0) {
        in.now_us += 25000; /* lands past the limit and past the gap limit together */
        in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 1);
    } else {
        in.now_us += 2000;
        in.wheel_hw[0] = (uint16_t)(in.wheel_hw[0] + 500);
    }
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    uint64_t t_limit = d.time_us;
    send(simple(LINK_CLEAR_FAULTS, seq++));
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT); /* kept through CLEAR_FAULTS */
    bool early = false;
    for (int t = 0; t < 400; t++) {
        link_msg m[3];
        m[0] = arm_msg(seq++, d.boot_id, echo_now());
        m[1] = commission_msg(seq++, 0, 0, 150, echo_now());
        m[1].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        m[2] = commission_msg(seq++, 0, 1000, 150, echo_now());
        m[2].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
        tick_moving(0.0, 0.0, m, 3);
        if (out.duty[DRIVE_CH_L] != 0.0f &&
            d.time_us < t_limit + (uint64_t)cfg.stop_grace_ms * 1000u) {
            early = true;
        }
    }
    return early;
}

static void v2r5_limit_hold_survives_a_fault_in_the_same_cycle(void)
{
    CHECK(!limit_with_fault_resumes_early(0));
    CHECK(!limit_with_fault_resumes_early(1));
}

/* Contract 3.4 as corrected 2026-11-12: a DISARM in the commissioning limit's STOPPING is
   taken at once (safe output per channel); only the re-arm hold cannot be shortened. Checked
   for a DISARM in the limit's own cycle and in the next one, with safe_brake both ways. */
static void limit_then_disarm(bool brake, bool same_cycle)
{
    boot_with(brake ? mod_commission : mod_commission_coast);
    if (brake) {
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            cfg.safe_brake[ch] = true;
        }
    }
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 1000, 150, echo_now()));
    uint64_t limit_at = d.cm_on_since_us + (uint64_t)cfg.commission_time_ms[0] * 1000u;
    while (d.time_us < limit_at) { /* the limit fires in the cycle after limit_at */
        if ((d.time_us / 1000u) % 50u == 0) {
            send(commission_msg(seq++, 0, 1000, 150, echo_now()));
        } else {
            tick();
        }
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    link_msg disarm = simple(LINK_DISARM, seq++);
    if (same_cycle) {
        tick_msgs(&disarm, 1); /* the limit fires at the head, the DISARM in the same cycle */
    } else {
        tick();
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        tick_msgs(&disarm, 1);
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK_NEAR(out.duty[ch], 0.0, 0.0);
        CHECK(out.drvoff[ch] == !brake); /* the configured safe output, no drive */
    }
    uint64_t hold_end = limit_at + 1000u + (uint64_t)cfg.stop_grace_ms * 1000u;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    while (d.time_us + 2000u < hold_end) {
        tick();
    }
    send(arm_msg(seq++, d.boot_id, echo_now())); /* handled one cycle before hold_end */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(d.time_us + 1000u, hold_end);
    send(arm_msg(seq++, d.boot_id, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT));
}

static void v2r6_disarm_in_the_limit_stop(void)
{
    limit_then_disarm(false, true);
    limit_then_disarm(false, false);
    limit_then_disarm(true, true);
    limit_then_disarm(true, false);
}


/* ---- bench commissioning limits (design 2026-11-25-bench-commission-limits 1.1) ----------- */
static void mod_commission_open_lr(drive_config *c)
{
    mod_commission(c);
    c->commission_duty_max_e4[DRIVE_CH_L] = c->commission_duty_max_e4[DRIVE_CH_R] = 10000;
    c->commission_time_ms[DRIVE_CH_L] = c->commission_time_ms[DRIVE_CH_R] = 5000;
}

static void s25_duty_limits_per_axis_in_integers(void)
{
    boot_with(mod_commission_open_lr);
    in.abs_fresh[1] = false;
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 10000, 150, echo_now())); /* L: up to 1.0 */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(commission_msg(seq++, 0, -10000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(commission_msg(seq++, 0, 10001, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(seq++, 0, (int16_t)-32768, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    tick();
    CHECK(!d.cm_on);
    send(commission_msg(seq++, 2, 1501, 150, echo_now())); /* M stays at 0.15 */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(seq++, 2, 1500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);

    /* The edge in integers: a limit of 7 takes 7 and refuses 8 (binary32 7 / 10000 x 10000 is
       6.9999995, which a float comparison would refuse). */
    boot_with(mod_commission);
    cfg.commission_duty_max_e4[DRIVE_CH_R] = 7;
    seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 1, 0, 150, echo_now()));
    send(commission_msg(seq++, 1, 7, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(commission_msg(seq++, 1, 8, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
}

static void s25_time_limit_is_the_interval_axis(void)
{
    boot_with(mod_commission_open_lr); /* L 5 s, M/H 2 s */
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    CHECK(commission_run(3000, 1000, false, &seq) < 0); /* past M's 2 s, still on */
    CHECK(!(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT));
    int stopped = commission_run(3000, 1000, false, &seq);
    CHECK(stopped >= 1990 && stopped <= 2010); /* L's own 5 s */
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
}

static void s25_disarm_alone_keeps_the_interval_axis(void)
{
    /* A DISARM resets cm_axis while cm_on still holds until the zero cycle: the limit check in
       that cycle reads the interval's own axis (Codex S P1-1). */
    boot_with(mod_commission_open_lr);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 3000, 150, echo_now()));
    CHECK(d.cm_on);
    CHECK_EQ_U(d.cm_on_axis, DRIVE_CH_L);
    link_msg m = simple(LINK_DISARM, seq++);
    tick_msgs(&m, 1);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(!d.cm_on);
    CHECK_EQ_U(d.cm_on_axis, 0xFFu);
    CHECK(!(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT));
}

static void s25_axis_change_inside_an_interval(void)
{
    boot_with(mod_commission_open_lr);
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 2000, 150, echo_now()));
    uint16_t last = d.last_rx_seq;
    uint64_t deadline = d.deadline_us;
    /* Same cycle DISARM -> ARM -> L(0) -> M(500): L continues the interval, M is refused. */
    link_msg b[4];
    b[0] = simple(LINK_DISARM, seq++);
    b[1] = arm_msg(seq++, d.boot_id, echo_now());
    b[2] = commission_msg(seq++, 0, 0, 150, echo_now());
    b[2].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
    b[3] = commission_msg(seq++, 2, 500, 150, echo_now());
    b[3].u.commission.arm_gen = (uint16_t)(d.arm_gen + 1u);
    tick_msgs(b, 4);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(d.last_rx_seq, (uint16_t)(seq - 2u)); /* the L(0) stays the last taken */
    CHECK_EQ_U(d.cm_axis, DRIVE_CH_L);
    (void)last;
    (void)deadline;

    /* A refusal leaves the current command, its deadline and the last accepted seq alone. */
    boot_with(mod_commission_open_lr);
    seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 0, 0, 150, echo_now()));
    send(commission_msg(seq++, 0, 2000, 150, echo_now()));
    last = d.last_rx_seq;
    deadline = d.deadline_us;
    float duty = d.cm_duty;
    send(commission_msg(seq++, 1, 2000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(d.last_rx_seq, last);
    CHECK(d.deadline_us == deadline);
    CHECK_NEAR(d.cm_duty, duty, 0.0);
    CHECK_EQ_U(d.cm_axis, DRIVE_CH_L);

    /* Reject order: the common checks first (a stale seq is 6), then the axis (11), then the
       picked axis's sensor (13). */
    in.abs_fresh[1] = false;
    ticks(5);
    send(commission_msg(seq++, 0, 2000, 150, echo_now())); /* keep L going */
    link_msg stale = commission_msg((uint16_t)(seq - 3u), 3, 500, 150, echo_now());
    send(stale);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_SEQ);
    send(commission_msg(seq++, 3, 500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
}

static void s25_each_axis_reads_its_own_time_limit(void)
{
    /* Different test limits per axis catch an index slip (Codex PR-1 P3): R 3 s, L 5 s, M 2 s. */
    boot_with(mod_commission_open_lr);
    cfg.commission_time_ms[DRIVE_CH_R] = 3000;
    uint16_t seq = 1;
    send(arm_msg(seq++, d.boot_id, echo_now()));
    send(commission_msg(seq++, 1, 0, 150, echo_now()));
    int t;
    for (t = 0; t < 4000 && d.mode == DRIVE_MODE_ARMED; t++) {
        if (t % 50 == 0) {
            link_msg m = commission_msg(seq++, 1, 1000, 150, echo_now());
            tick_moving(0.0, 2.0, &m, 1);
        } else {
            tick_moving(0.0, 2.0, NULL, 0);
        }
    }
    CHECK(t >= 2990 && t <= 3012); /* R's own 3 s, not L's 5 s */
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);

    /* M and H: the build's 2 s, then the re-arm hold. */
    for (uint8_t axis = DRIVE_CH_M; axis <= DRIVE_CH_H; axis++) {
        boot_with(mod_commission_open_lr);
        seq = 1;
        send(arm_msg(seq++, d.boot_id, echo_now()));
        send(commission_msg(seq++, axis, 0, 150, echo_now()));
        for (t = 0; t < 3000 && d.mode == DRIVE_MODE_ARMED; t++) {
            if (t % 50 == 0) {
                link_msg m = commission_msg(seq++, axis, 300, 150, echo_now());
                tick_msgs(&m, 1);
            } else {
                tick();
            }
        }
        CHECK(t >= 1990 && t <= 2012);
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        send(simple(LINK_DISARM, seq++));
        send(arm_msg(seq++, d.boot_id, echo_now()));
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING); /* the hold outlives the DISARM */
    }
}

/* ---- bench image (contract 8, bench console plan A1a) --------------------------------- */
static void mod_bench(drive_config *c)
{
    mod_commission(c);
    c->bench_build = true;
    pwm_sim = true; /* the bench board reports its outputs every tick (plan 4.3 decision 6) */
}

static link_bench_set bench_defaults(uint32_t run_id, uint8_t axis)
{
    link_bench_set b = {.boot_id = d.boot_id,
                        .echo_mcu_time_us = echo_now(),
                        .run_id = run_id,
                        .mode = DRIVE_BENCH_MODE_DUTY,
                        .axis = axis,
                        .source = DRIVE_BENCH_SOURCE_NONE,
                        .count_sign = 1,
                        .bypass = 0,
                        .stop_counts = 0};
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        b.duty_max_e4[ch] = 1500;
        b.time_ms[ch] = 2000;
    }
    return b;
}

static link_msg bench_set_msg(uint16_t seq, link_bench_set b)
{
    link_msg m = {.type = LINK_BENCH_SET, .seq = seq};
    m.u.bench_set = b;
    return m;
}

static link_msg bench_arm_msg(uint16_t seq, uint32_t run_id, uint64_t echo)
{
    link_msg m = {.type = LINK_BENCH_ARM, .seq = seq};
    m.u.bench_arm = (link_bench_arm){.boot_id = d.boot_id, .echo_mcu_time_us = echo, .run_id = run_id};
    return m;
}

static link_bench_state bench_state_now(void)
{
    link_bench_state s;
    drive_fill_bench_state(&d, &s);
    return s;
}

/* A connection change that began now and settled, applied by the next tick (as app.c does). */
static void link_change(void)
{
    drive_note_link_change(&d, in.now_us, true);
    tick();
}

/* SET + BENCH_ARM for one run on axis; true when armed. */
static bool bench_arm_run(uint32_t run_id, uint8_t axis, uint32_t bypass)
{
    link_bench_set b = bench_defaults(run_id, axis);
    b.bypass = bypass;
    send(bench_set_msg(10, b));
    if (d.reject_reason != DRIVE_REJECT_NONE) {
        return false;
    }
    send(bench_arm_msg(11, run_id, echo_now()));
    return d.mode == DRIVE_MODE_ARMED;
}

static void bench_config_validation(void)
{
    drive_config c = make_config();
    mod_bench(&c);
    CHECK(drive_config_valid(&c));
    c.commission_build = false; /* a bench image is a commissioning build */
    CHECK(!drive_config_valid(&c));
    c = make_config();
    mod_bench(&c);
    c.commission_single_axis = true; /* and never fw-single */
    CHECK(!drive_config_valid(&c));
    c = make_config();
    mod_bench(&c);
    c.outputs_locked = true;
    CHECK(!drive_config_valid(&c));
}

static void bench_set_refusals(void)
{
    boot_with(mod_commission); /* a commissioning build that is not the bench image */
    send(bench_set_msg(10, bench_defaults(1, DRIVE_CH_M)));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
    send(bench_arm_msg(11, 1, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);

    boot_with(mod_bench);
    link_bench_set b = bench_defaults(1, DRIVE_CH_M);
    b.boot_id = d.boot_id + 1u;
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    b = bench_defaults(1, DRIVE_CH_M);
    b.echo_mcu_time_us = d.time_us + 5000u; /* from the future */
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
    b = bench_defaults(1, DRIVE_CH_M);
    ticks(300); /* older than command_timeout_ms (200) */
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);

    struct {
        int field;
    } bad[13];
    for (int i = 0; i < 13; i++) {
        bad[i].field = i;
    }
    for (int i = 0; i < 13; i++) {
        b = bench_defaults(1, DRIVE_CH_M);
        switch (bad[i].field) {
        case 0: b.run_id = 0; break;
        case 1: b.mode = 2; break; /* hold: A1b */
        case 2: b.axis = 4; break;
        case 3: b.source = 2; break; /* mt: A1c */
        case 4: b.count_sign = 0; break;
        case 5: b.bypass = 1u << 4; break;
        case 6: b.stop_counts = 5; break; /* distance stop: A1b */
        case 7: b.duty_max_e4[2] = 0; break;
        case 8: b.duty_max_e4[2] = 10001; break;
        case 9: b.time_ms[1] = 0; break;
        case 11: b.axis = DRIVE_CH_L; b.source = 1; break; /* kth only on M and H */
        case 12: b.source = 1; b.stop_counts = INT32_MIN; break;
        default: b.time_ms[3] = 0x7FFFFFFFu / 1000u + 1u; break;
        }
        send(bench_set_msg(10, b));
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
        CHECK(!d.bset_valid);
    }
    CHECK(bench_arm_run(7, DRIVE_CH_M, 0));
    send(bench_set_msg(12, bench_defaults(8, DRIVE_CH_M))); /* armed: no SET */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_DISARMED);
}

static void bench_arm_needs_its_set(void)
{
    boot_with(mod_bench);
    send(arm_msg(1, d.boot_id, echo_now())); /* the bench image never takes a plain ARM */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_WRONG_BUILD);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(bench_arm_msg(2, 1, echo_now())); /* no SET */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(bench_set_msg(3, bench_defaults(5, DRIVE_CH_M)));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(bench_arm_msg(4, 6, echo_now())); /* another run's id */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    ticks((int)DRIVE_BENCH_SET_TTL_MS + 1); /* expired */
    send(bench_arm_msg(5, 5, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);

    send(bench_set_msg(6, bench_defaults(9, DRIVE_CH_M)));
    link_change(); /* a SET from the last connection cannot be consumed */
    send(bench_arm_msg(7, 9, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);

    CHECK(bench_arm_run(10, DRIVE_CH_M, 0));
    send(simple(LINK_DISARM, 12));
    send(bench_arm_msg(13, 10, echo_now())); /* consumed once */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(bench_set_msg(14, bench_defaults(11, DRIVE_CH_M)));
    send(simple(LINK_DISARM, 15)); /* a DISARM drops an unconsumed SET */
    send(bench_arm_msg(16, 11, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
}

static void bench_duty_run_ends_itself(void)
{
    boot_with(mod_bench);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.drive & 3u, DRIVE_BENCH_RUN_NONE);
    CHECK(bench_arm_run(42, DRIVE_CH_M, 0));
    s = bench_state_now();
    CHECK_EQ_U(s.run_id, 42);
    CHECK_EQ_U(s.drive, DRIVE_BENCH_RUN_PREP | (DRIVE_BENCH_MODE_DUTY << 4));
    CHECK_EQ_U(s.result_run_id, 0);
    send(commission_msg(20, DRIVE_CH_L, 0, 150, echo_now())); /* the arming's L(0): preparation */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(commission_msg(21, DRIVE_CH_L, 500, 150, echo_now())); /* not the run's axis */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    send(commission_msg(22, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(bench_state_now().drive & 3u, DRIVE_BENCH_RUN_PREP);
    send(commission_msg(23, DRIVE_CH_M, 1000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(bench_state_now().drive & 3u, DRIVE_BENCH_RUN_DRIVING);
    tick();
    CHECK(out.duty[DRIVE_CH_M] > 0.05f);
    send(commission_msg(24, DRIVE_CH_M, 0, 150, echo_now())); /* the end */
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(d.last_rx_seq, 24);
    s = bench_state_now();
    CHECK_EQ_U(s.drive & 3u, DRIVE_BENCH_RUN_ENDED);
    CHECK_EQ_U(s.result_run_id, 42);
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_END_ZERO);
    CHECK_EQ_U(s.result_axis, DRIVE_CH_M);
    CHECK((d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT) == 0);
    send(commission_msg(25, DRIVE_CH_M, 0, 150, echo_now())); /* refused and counted */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    send(commission_msg(26, DRIVE_CH_M, 900, 150, echo_now()));
    CHECK_EQ_U(bench_state_now().blocked, 2);
    ticks(150); /* past stop_grace_ms: DISARMED by itself, no COMMAND_TIMEOUT */
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK((d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT) == 0);
    CHECK(!d.run_active);
    s = bench_state_now();
    CHECK_EQ_U(s.result_run_id, 42); /* kept for the host */
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_END_ZERO);
    send(commission_msg(27, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_ARMED);
    CHECK_EQ_U(bench_state_now().blocked, 3);
    CHECK(bench_arm_run(43, DRIVE_CH_M, 0)); /* the next run consumed: the last result is gone */
    s = bench_state_now();
    CHECK_EQ_U(s.result_run_id, 0);
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_NONE);
    CHECK_EQ_U(s.blocked, 0);
}

/* Each first cause of a run that drove, and the first one kept. */
static void bench_run_end_causes(void)
{
    /* the command expiry */
    boot_with(mod_bench);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 50, echo_now()));
    ticks(60);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_COMMAND_TIMEOUT);
    ticks(150);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_COMMAND_TIMEOUT); /* first cause kept */

    /* the run's own time limit (shorter than the build's) */
    boot_with(mod_bench);
    link_bench_set b = bench_defaults(2, DRIVE_CH_M);
    b.time_ms[DRIVE_CH_M] = 30;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 2, echo_now()));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    for (int i = 0; i < 50 && d.mode == DRIVE_MODE_ARMED; i++) {
        send(commission_msg((uint16_t)(21 + i), DRIVE_CH_M, 800, 150, echo_now()));
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_TIME_LIMIT);

    /* a host DISARM while driving */
    boot_with(mod_bench);
    CHECK(bench_arm_run(3, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    send(simple(LINK_DISARM, 22));
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_DISARM);

    /* a driver fault before any result: FAULT keeps its outputs and latch, the result says fault */
    boot_with(mod_bench);
    CHECK(bench_arm_run(4, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    in.nfault_low[DRIVE_CH_H] = true;
    tick();
    in.nfault_low[DRIVE_CH_H] = false; /* one cycle only */
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(all_drvoff());
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.result_run_id, 4);
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_FAULT);
    send(bench_set_msg(12, bench_defaults(5, DRIVE_CH_M))); /* no new run before an explicit clear */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_DISARMED);
    send(bench_arm_msg(13, 5, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_LATCHED);

    /* a run that ends before it drove leaves no result */
    boot_with(mod_bench);
    CHECK(bench_arm_run(6, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_L, 0, 150, echo_now()));
    send(simple(LINK_DISARM, 21));
    s = bench_state_now();
    CHECK_EQ_U(s.result_run_id, 0);
    CHECK_EQ_U(s.drive & 3u, DRIVE_BENCH_RUN_ENDED);
}

static void bench_range_bypass_is_the_runs(void)
{
    boot_with(mod_bench);
    set_steer(0.7); /* outside the steering joint range (+-0.5) */
    ticks(5);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    send(simple(LINK_DISARM, 21));

    CHECK(bench_arm_run(2, DRIVE_CH_M, DRIVE_BENCH_STEER_RANGE));
    CHECK_EQ_U(bench_state_now().mcu_bypass, DRIVE_BENCH_STEER_RANGE);
    CHECK((d.fault_flags & (1u << 31)) == 0); /* v3 reserved bits stay 0 (plan A1) */
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    for (int i = 0; i < 20; i++) {
        send(commission_msg((uint16_t)(21 + i), DRIVE_CH_M, 800, 150, echo_now()));
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* no out-of-range fault while bypassed */
    in.abs_fresh[0] = false; /* KTH validity is not bypassed */
    for (int i = 0; i < 6 && d.mode == DRIVE_MODE_ARMED; i++) {
        send(commission_msg((uint16_t)(41 + i), DRIVE_CH_M, 800, 150, echo_now()));
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    in.abs_fresh[0] = true;
    ticks(5);
    send(simple(LINK_CLEAR_FAULTS, 50));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_U(bench_state_now().mcu_bypass, 0);

    CHECK(bench_arm_run(3, DRIVE_CH_M, 0)); /* the bypass was run 2's only */
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
}

static void bench_link_boundary(void)
{
    boot_with(mod_bench);
    CHECK_EQ_U(bench_state_now().conn_gen, 0);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    uint64_t old_echo = echo_now(); /* the last STATE of the old connection */
    tick();
    link_change(); /* an arming never outlives its connection */
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.conn_gen, 1);
    CHECK_EQ_U(s.result_run_id, 1);
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_LINK_BOUNDARY);
    send(commission_msg(22, DRIVE_CH_M, 800, 150, old_echo)); /* a late frame of the old host */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    link_bench_set b = bench_defaults(2, DRIVE_CH_M);
    b.echo_mcu_time_us = old_echo; /* echoes a STATE from before the change */
    send(bench_set_msg(23, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
    send(bench_set_msg(24, bench_defaults(2, DRIVE_CH_M)));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(bench_arm_msg(25, 2, old_echo));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
    send(bench_arm_msg(26, 2, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);

    /* a boundary while preparing: the arming ends, no result */
    send(simple(LINK_DISARM, 27));
    CHECK(bench_arm_run(3, DRIVE_CH_M, 0));
    link_change();
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().result_run_id, 0);

    /* other builds ignore it */
    boot_with(mod_commission);
    send(arm_msg(1, d.boot_id, echo_now()));
    link_change();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}


/* Codex A1a round 1: the bench image never cuts a STOPPING brake short with a DISARM. */
static void bench_disarm_keeps_the_brake(void)
{
    boot_with(mod_bench);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    send(commission_msg(22, DRIVE_CH_M, 0, 150, echo_now())); /* the end: STOPPING */
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    send(simple(LINK_DISARM, 23));
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING); /* the brake goes on */
    CHECK_EQ_U(d.last_rx_seq, 23);
    CHECK(!out.drvoff[DRIVE_CH_M]);
    ticks(50);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    ticks(60);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_END_ZERO);

    CHECK(bench_arm_run(2, DRIVE_CH_M, 0)); /* a connection change's STOPPING, then a late DISARM */
    send(commission_msg(30, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(31, DRIVE_CH_M, 800, 150, echo_now()));
    tick();
    link_change();
    send(simple(LINK_DISARM, 32));
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
}

/* Codex A1a round 1: an output interval keeps the run's limit after the run ends. */
static void bench_run_limit_outlives_the_run(void)
{
    for (int via_fault = 0; via_fault < 2; via_fault++) {
        boot_with(mod_bench); /* the build's M limit is 2000 ms */
        link_bench_set b = bench_defaults(1, DRIVE_CH_M);
        b.time_ms[DRIVE_CH_M] = 30000;
        send(bench_set_msg(10, b));
        send(bench_arm_msg(11, 1, echo_now()));
        send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
        for (int i = 0; i < 2500; i++) { /* 2.5 s of output: past the build's limit, inside the run's */
            send(commission_msg((uint16_t)(21 + i), DRIVE_CH_M, 800, 150, echo_now()));
        }
        CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
        if (via_fault) {
            in.nfault_low[DRIVE_CH_R] = true;
            tick();
            in.nfault_low[DRIVE_CH_R] = false;
            CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
        } else {
            send(simple(LINK_DISARM, 9000));
            CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
        }
        ticks(3);
        CHECK((d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT) == 0);
        CHECK_EQ_U(bench_state_now().stop_reason, via_fault ? DRIVE_BENCH_STOP_FAULT : DRIVE_BENCH_STOP_DISARM);
        if (!via_fault) {
            CHECK(bench_arm_run(2, DRIVE_CH_M, 0)); /* no re-arm hold either */
        }
    }
}

/* Codex A1a round 1: an unconsumed SET does not outlive an e-stop and its clear. */
static void bench_set_does_not_outlive_an_estop(void)
{
    boot_with(mod_bench);
    send(bench_set_msg(10, bench_defaults(3, DRIVE_CH_M)));
    CHECK(d.bset_valid);
    in.estop_active = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ESTOP);
    in.estop_active = false;
    tick();
    send(simple(LINK_CLEAR_FAULTS, 11));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(bench_arm_msg(12, 3, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
}

/* Codex A1a round 1: blocked counts only what the end refused (stopping, not armed). */
static void bench_blocked_counts_the_end_only(void)
{
    boot_with(mod_bench);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    send(commission_msg(22, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(23, DRIVE_CH_L, 500, 150, echo_now())); /* another axis: bad_value */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    link_msg m = commission_msg(24, DRIVE_CH_M, 0, 150, echo_now());
    m.u.commission.boot_id = d.boot_id + 1u; /* another boot */
    send(m);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BOOT_MISMATCH);
    CHECK_EQ_U(bench_state_now().blocked, 0);
    send(commission_msg(25, DRIVE_CH_M, 0, 150, echo_now()));
    CHECK_EQ_U(bench_state_now().blocked, 1);
}


/* Codex A1a round 2: the brake a connection change starts lasts the full stop_grace_ms of the tick's own time. */
static void bench_change_brake_is_full_length(void)
{
    boot_with(mod_bench);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    drive_note_link_change(&d, in.now_us, true);
    in.now_us += 15000; /* a long gap before the tick that applies it */
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    uint64_t start = d.time_us;
    while (d.mode == DRIVE_MODE_STOPPING) {
        tick();
    }
    CHECK(d.time_us - start >= 100000u); /* stop_grace_ms 100 from the applying tick */
}

/* Codex A1a round 2: the first cause between a change and the command expiry follows their real order. */
static void bench_change_versus_expiry_order(void)
{
    for (int change_first = 0; change_first < 2; change_first++) {
        boot_with(mod_bench);
        CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
        send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
        send(commission_msg(21, DRIVE_CH_M, 800, 40, echo_now())); /* deadline 40 ms after its echo */
        ticks(30);
        uint32_t t_change = change_first ? in.now_us + 5000u : in.now_us + 15000u; /* before / after the deadline */
        in.now_us += 19000; /* one tick past the deadline, under the encoder gap limit */
        drive_note_link_change(&d, t_change, true);
        tick_msgs(NULL, 0);
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        CHECK_EQ_U(bench_state_now().stop_reason,
                   change_first ? DRIVE_BENCH_STOP_LINK_BOUNDARY : DRIVE_BENCH_STOP_COMMAND_TIMEOUT);
        CHECK(((d.fault_flags & DRIVE_FAULT_COMMAND_TIMEOUT) != 0) == !change_first);
    }
}

/* Codex A1a round 3: an output time limit passed before the change began is the first cause. */
static void bench_change_versus_time_limit(void)
{
    boot_with(mod_bench);
    link_bench_set b = bench_defaults(1, DRIVE_CH_M);
    b.time_ms[DRIVE_CH_M] = 30;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 1, echo_now()));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    ticks(25); /* the interval opened ~25 ms ago; its limit is 30 ms */
    uint32_t t_change = in.now_us + 12000u; /* after the limit (5 ms from now), before the command deadline */
    in.now_us += 15000;
    drive_note_link_change(&d, t_change, true);
    tick_msgs(NULL, 0);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_TIME_LIMIT);
}

/* Codex A1a round 4: limit -> change -> deadline, all before the applying tick: the limit is still the first cause. */
static void bench_limit_change_deadline_order(void)
{
    boot_with(mod_bench);
    link_bench_set b = bench_defaults(1, DRIVE_CH_M);
    b.time_ms[DRIVE_CH_M] = 30;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 1, echo_now()));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 40, echo_now())); /* deadline <= 40 ms */
    ticks(25);
    uint32_t t_change = in.now_us + 8000u; /* after the limit (~5 ms from now), before the deadline (~15 ms) */
    in.now_us += 19000; /* the applying tick is past both */
    drive_note_link_change(&d, t_change, true);
    tick_msgs(NULL, 0);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_TIME_LIMIT);
}

/* Codex A1a round 2: another axis's request is not counted as blocked in DISARMED either. */
static void bench_blocked_other_axis_after_disarm(void)
{
    boot_with(mod_bench);
    CHECK(bench_arm_run(1, DRIVE_CH_M, 0));
    send(commission_msg(20, DRIVE_CH_M, 0, 150, echo_now()));
    send(commission_msg(21, DRIVE_CH_M, 800, 150, echo_now()));
    send(commission_msg(22, DRIVE_CH_M, 0, 150, echo_now()));
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(commission_msg(23, DRIVE_CH_L, 500, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_ARMED);
    CHECK_EQ_U(bench_state_now().blocked, 0);
    send(commission_msg(24, DRIVE_CH_M, 500, 150, echo_now()));
    CHECK_EQ_U(bench_state_now().blocked, 1);
}


/* ---- bench6: KTH count, distance stop, watch (plan 4.2 A1b-1) ------------------------------------------------ */

static void kraw(int i, uint16_t raw)
{
    in.abs_frame[i] = (uint16_t)((uint16_t)(raw << 4) | crc4_kth7801(raw, KTH_CRC_G704_BYTES));
}

/* A KTH run on M (index 0) with the range bypass, stopping at stop counts (0: measuring only). */
static bool kth_arm(uint32_t run_id, int32_t stop, int8_t count_sign)
{
    link_bench_set b = bench_defaults(run_id, DRIVE_CH_M);
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.stop_counts = stop;
    b.count_sign = count_sign;
    b.bypass = DRIVE_BENCH_STEER_RANGE;
    send(bench_set_msg(10, b));
    if (d.reject_reason != DRIVE_REJECT_NONE) {
        return false;
    }
    send(bench_arm_msg(11, run_id, echo_now()));
    return d.mode == DRIVE_MODE_ARMED;
}

static uint16_t cm_seq = 100;

static void cm(int16_t duty)
{
    send(commission_msg(cm_seq++, DRIVE_CH_M, duty, 150, echo_now()));
}

static void bench_distance_reached(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(60, 40, 1));
    cm(0);
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.count_state, DRIVE_BENCH_COUNT_VALID);
    int32_t ref = s.distance_ref;
    for (uint16_t raw = 1010; raw <= 1030; raw += 10) {
        kraw(0, raw);
        tick();
        CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    }
    kraw(0, 1040);
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
    CHECK_EQ_I(s.cut_position - ref, 40);
    CHECK_EQ_U(s.distance_confirm, DRIVE_BENCH_CONFIRM_NONE);
    CHECK_EQ_U(s.run_flags, DRIVE_BENCH_FLAG_RES_DISTANCE); /* bench7: the result names its kind */
    CHECK(s.end_evidence_age_ms <= 1u);
    cm(1000); /* refused after the cut, counted */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(bench_state_now().blocked, 1);
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
}

static void bench_distance_provisional_and_confirmation(void)
{
    for (int variant = 0; variant < 3; variant++) {
        boot_with(mod_bench);
        kraw(0, 30);
        ticks(3);
        CHECK(kth_arm(61, -25, 1));
        cm(0);
        cm(-1000);
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
        int32_t ref = bench_state_now().distance_ref;
        kraw(0, 20);
        tick();
        kraw(0, 0); /* crossing on a 0x0000 frame */
        tick();
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        link_bench_state s = bench_state_now();
        CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_PROVISIONAL);
        CHECK_EQ_U(s.distance_confirm, DRIVE_BENCH_CONFIRM_NONE);
        CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_TENTATIVE);
        if (variant == 0) {
            kraw(0, 4090); /* -26 from 20: -36 from the reference */
            tick();
            CHECK_EQ_U(bench_state_now().distance_confirm, DRIVE_BENCH_CONFIRM_REACHED);
        } else if (variant == 1) {
            kraw(0, 25); /* -5: the fake step undone */
            tick();
            s = bench_state_now();
            CHECK_EQ_U(s.distance_confirm, DRIVE_BENCH_CONFIRM_HELD);
            CHECK((s.run_flags & DRIVE_BENCH_FLAG_RES_LOSS) == 0);
        } else {
            kraw(0, 2000); /* a too-fast step: the count lost before a confirming frame */
            tick();
            kraw(0, 2010);
            ticks(2);
            s = bench_state_now();
            CHECK_EQ_U(s.distance_confirm, DRIVE_BENCH_CONFIRM_HELD);
            CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_LOSS);
        }
        (void)ref;
        kraw(0, 3000); /* settled once */
        ticks(2);
        CHECK(bench_state_now().distance_confirm != DRIVE_BENCH_CONFIRM_NONE);
    }
}

static void bench_distance_first_drive_needs_a_fresh_reference(void)
{
    boot_with(mod_bench);
    kraw(0, 256);
    ticks(3);
    CHECK(kth_arm(62, 100, 1));
    cm(0);
    kraw(0, 0); /* MISO low for a frame: a counted 0x0000 is no reference */
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    CHECK_EQ_U(bench_state_now().drive & 3u, DRIVE_BENCH_RUN_PREP);
    kraw(0, 256);
    in.abs_fresh[0] = false; /* no frame in the command's tick */
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    in.abs_fresh[0] = true;
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(bench_state_now().drive & 3u, DRIVE_BENCH_RUN_DRIVING);
}

static void bench_distance_loss_since_arm_refuses_the_drive(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(63, 100, 1));
    cm(0);
    kraw(0, 3000); /* a CRC-good step too fast for the bound: the count lost (the joint stays valid) */
    tick();
    CHECK_EQ_U(bench_state_now().count_state, DRIVE_BENCH_COUNT_LOST);
    kraw(0, 3010); /* a normal step from the detecting frame: recovered */
    ticks(2);
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(bench_state_now().count_state, DRIVE_BENCH_COUNT_VALID);
    CHECK(bench_state_now().run_flags & DRIVE_BENCH_FLAG_UNCLEAN);
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
}

static void bench_distance_direction_rules(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(64, 100, 1));
    cm(0);
    cm(-1000); /* -: the target + needs count_sign x duty = + */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    cm(1000);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    cm(-1000); /* one direction per distance run */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
}

static void bench_distance_wrong_way_stops_at_once(void)
{
    boot_with(mod_bench);
    kraw(0, 2000);
    ticks(3);
    CHECK(kth_arm(65, 1000, 1)); /* the operator's count_sign is wrong: the counts fall */
    cm(0);
    cm(1000);
    uint16_t raw = 2000;
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 40) {
        raw = (uint16_t)(raw - 10);
        kraw(0, raw);
        tick();
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK_EQ_I(n, 13); /* -130: the first trusted sample past -128 */
}

static void bench_distance_zero_run_and_its_recovery(void)
{
    boot_with(mod_bench);
    kraw(0, 100);
    ticks(3);
    CHECK(kth_arm(66, 2000, 1));
    cm(0);
    cm(1000);
    kraw(0, 0); /* stuck low near zero: counted, tentative */
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 20) {
        tick();
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT);
    CHECK(n >= 4 && n <= 6);
    ticks(150);
    link_bench_set b = bench_defaults(67, DRIVE_CH_M);
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.stop_counts = 100;
    b.bypass = DRIVE_BENCH_STEER_RANGE;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 67, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID); /* until a non-zero frame */
    kraw(0, 5);
    ticks(2);
    CHECK(kth_arm(68, 100, 1));
}

static void bench_distance_watch_stops_a_stalled_axis(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(69, 500, 1));
    cm(0);
    cm(1000);
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        if (n % 100 == 50) {
            cm(1000); /* keep the command fresh */
        } else {
            tick();
        }
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 298 && n <= 302);
}

static void bench_distance_slow_progress_keeps_going(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(70, 500, 1));
    cm(0);
    cm(1000);
    uint16_t raw = 1000;
    for (int n = 1; n <= 1000; n++) {
        if (n % 250 == 0) {
            raw = (uint16_t)(raw + 2); /* 2 counts every 0.25 s */
            kraw(0, raw);
        }
        if (n % 100 == 0) {
            cm(1000);
        } else {
            tick();
        }
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}

static void bench_measuring_run_flags(void)
{
    boot_with(mod_bench);
    kraw(0, 256);
    ticks(3);
    CHECK(kth_arm(71, 0, 1)); /* measuring only */
    cm(0);
    kraw(0, 0);
    cm(1000); /* starts on a tentative sample: no reference */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    kraw(0, 256);
    ticks(3);
    kraw(0, 0);
    tick();
    cm(0); /* ends on a tentative sample */
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_END_ZERO);
    CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_NO_REF);
    CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_TENTATIVE);
    int32_t end_counts = s.cut_position;
    kraw(0, 256);
    ticks(2); /* re-anchored after the end: the end position stays */
    CHECK_EQ_I(bench_state_now().cut_position, end_counts);
}

static void bench_evidence_age(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(72, 0, 1));
    CHECK_EQ_U(bench_state_now().evidence_age_ms, 0xFFFFu); /* none since the ARM */
    tick();
    CHECK(bench_state_now().evidence_age_ms <= 1u);
    in.abs_fresh[0] = false;
    ticks(2);
    CHECK(bench_state_now().evidence_age_ms >= 2u);
}

static void bench_h_axis_counts_its_own(void)
{
    boot_with(mod_bench);
    kraw(1, 1500);
    kraw(0, 700);
    ticks(3);
    link_bench_set b = bench_defaults(73, DRIVE_CH_H);
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.stop_counts = 30;
    b.bypass = DRIVE_BENCH_LIFT_RANGE;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 73, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    send(commission_msg(cm_seq++, DRIVE_CH_H, 0, 150, echo_now()));
    send(commission_msg(cm_seq++, DRIVE_CH_H, 1000, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    kraw(0, 900); /* M moves: not H's count */
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    kraw(1, 1530);
    tick();
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
}

static void bench_bench6_tests(void)
{
    bench_distance_reached();
    bench_distance_provisional_and_confirmation();
    bench_distance_first_drive_needs_a_fresh_reference();
    bench_distance_loss_since_arm_refuses_the_drive();
    bench_distance_direction_rules();
    bench_distance_wrong_way_stops_at_once();
    bench_distance_zero_run_and_its_recovery();
    bench_distance_watch_stops_a_stalled_axis();
    bench_distance_slow_progress_keeps_going();
    bench_measuring_run_flags();
    bench_evidence_age();
    bench_h_axis_counts_its_own();
}


/* ---- bench6 after the implementation review round 1 ---------------------------------------------------------- */

static void bench_watch_counts_from_the_last_progress(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(80, 500, 1));
    cm(0);
    cm(1000);
    for (int n = 1; n <= 50; n++) {
        if (n == 50) {
            kraw(0, 1002); /* +2: progress at this tick */
        }
        n % 40 == 0 ? cm(1000) : tick();
    }
    int since = 0;
    while (d.mode == DRIVE_MODE_ARMED && since < 400) {
        since++;
        since % 100 == 0 ? cm(1000) : tick();
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK_EQ_I(since, 300); /* 300 ms after the progress, never 299 */
}

static void bench_cut_and_a_command_in_the_same_tick(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(81, 40, 1));
    cm(0);
    cm(1000);
    int32_t ref = bench_state_now().distance_ref;
    kraw(0, 1040);
    cm(1000); /* the cut is judged before this tick's command */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
    CHECK_EQ_I(s.cut_position - ref, 40);
    CHECK_EQ_U(s.blocked, 1);
    CHECK(out.duty[DRIVE_CH_M] == 0.0f);
}

static void bench_cut_and_an_other_axis_fault_in_the_same_tick(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(82, 40, 1));
    cm(0);
    cm(1000);
    int32_t ref = bench_state_now().distance_ref;
    kraw(0, 1040);
    in.nfault_low[DRIVE_CH_R] = true; /* R's driver faults in the crossing tick */
    tick();
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_FAULT); /* the existing fault judgement first */
    CHECK_EQ_I(s.cut_position - ref, 40);              /* the end position from this tick's frame */
    CHECK_EQ_U(s.end_evidence_age_ms, 0);              /* the same frame: acquired in this tick */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK(out.duty[ch] == 0.0f);
    }
}

static void bench_cut_and_the_command_expiry_in_the_same_tick(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(83, 40, 1));
    cm(0);
    cm(1000);
    int32_t ref = bench_state_now().distance_ref;
    while (d.time_us + cfg.period_us <= d.deadline_us) {
        tick();
    }
    kraw(0, 1040); /* the crossing in the expiry tick */
    tick();
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_COMMAND_TIMEOUT);
    CHECK_EQ_I(s.cut_position - ref, 40);
}

static void bench_evidence_after_disarmed(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(84, 40, 1));
    cm(0);
    cm(1000);
    kraw(0, 1040);
    tick();
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    kraw(0, 1041);
    ticks(5);
    link_bench_state s = bench_state_now();
    CHECK(s.evidence_age_ms <= 1u); /* the run axis's evidence goes on until the next ARM */
    CHECK_EQ_U(s.run_flags & DRIVE_BENCH_FLAG_UNCLEAN, 0);
}

static void bench_distance_across_the_clock_wrap(void)
{
    reset_world(0xFFFFFFFFu - 400000u);
    mod_bench(&cfg);
    CHECK(drive_init(&d, &cfg, &in, BOOT_ID, DRIVE_RESET_POWER));
    for (int i = 0; i < 200 && d.mode == DRIVE_MODE_BOOT; i++) {
        tick();
    }
    kraw(0, 2000);
    while (in.now_us < 0xFFFFFFFFu - 5000u) {
        tick();
    }
    CHECK(kth_arm(85, 100, 1));
    cm(0);
    cm(1000);
    int32_t ref = bench_state_now().distance_ref;
    uint16_t raw = 2000;
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 30) {
        raw = (uint16_t)(raw + 10);
        kraw(0, raw);
        tick();
        n++;
    }
    CHECK(in.now_us < 0x00100000u); /* past the wrap */
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
    CHECK_EQ_I(s.cut_position - ref, 100);
    CHECK_EQ_I(n, 10);
}

/* ---- bench7: hold (plan 4.3 A1b-2) ------------------------------------------------------------------------- */

static link_bench_hold hcfg;

/* a hold run on M (KTH index 0); bypass: the steering range */
static bool hold_arm(uint32_t run_id, int8_t count_sign, uint32_t bypass)
{
    link_bench_set b = bench_defaults(run_id, DRIVE_CH_M);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.count_sign = count_sign;
    b.bypass = bypass;
    send(bench_set_msg(10, b));
    if (d.reject_reason != DRIVE_REJECT_NONE) {
        return false;
    }
    send(bench_arm_msg(11, run_id, echo_now()));
    hcfg = (link_bench_hold){.boot_id = d.boot_id,
                             .arm_gen = d.arm_gen,
                             .valid_after_echo_ms = 100,
                             .run_id = run_id,
                             .axis = DRIVE_CH_M,
                             .source = DRIVE_BENCH_SOURCE_KTH,
                             .target_kind = BENCH_HOLD_TARGET_HERE,
                             .cpr = 4096,
                             .kp = 0.5f,
                             .deadband_counts = 12,
                             .duty_max_e4 = 1500,
                             .min_duty_e4 = 1200,
                             .sign = count_sign};
    return d.mode == DRIVE_MODE_ARMED;
}

static link_msg hold_msg(void)
{
    link_msg m = {.type = LINK_BENCH_HOLD, .seq = cm_seq++}; /* one seq space with the COMMISSIONs */
    m.u.bench_hold = hcfg;
    m.u.bench_hold.echo_mcu_time_us = echo_now();
    m.u.bench_hold.arm_gen = d.arm_gen;
    return m;
}

static void hold(void)
{
    send(hold_msg());
}

/* SET + ARM + M(0) + the first HOLD at raw; true when driving */
static bool hold_start(uint32_t run_id, uint16_t raw, uint8_t kind, int32_t value)
{
    kraw(0, raw);
    pwm_sim = true; /* the board's report: progress needs it (Codex A1b-2a round 1 P1-2) */
    ticks(3);
    if (!hold_arm(run_id, 1, DRIVE_BENCH_STEER_RANGE)) {
        return false;
    }
    cm(0);
    hcfg.target_kind = kind;
    hcfg.target_value = value;
    hold();
    return d.reject_reason == DRIVE_REJECT_NONE && (bench_state_now().drive & 3u) == DRIVE_BENCH_RUN_DRIVING;
}

static void hold_set_rules(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    link_bench_set b = bench_defaults(80, DRIVE_CH_M);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    send(bench_set_msg(10, b)); /* no source */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.stop_counts = 100; /* no distance in a hold */
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    b.stop_counts = 0;
    b.axis = DRIVE_CH_L;
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    b.axis = DRIVE_CH_M;
    b.mode = 3;
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    CHECK(hold_arm(81, 1, DRIVE_BENCH_STEER_RANGE));
    CHECK_EQ_U((bench_state_now().drive >> 4) & 3u, DRIVE_BENCH_MODE_HOLD);
}

/* M run: L(0) refused (bad_value, ZERO_SEEN stays off), then the correct M HOLD answers zero_first; M(0), then the
   first HOLD drives and hands the run axis to the output checks (independent review P1-3, round 4 P2) */
static void hold_readiness_on_the_run_axis(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(hold_arm(82, 1, DRIVE_BENCH_STEER_RANGE));
    send(commission_msg(cm_seq++, DRIVE_CH_L, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    link_state st;
    drive_fill_state(&d, &st);
    CHECK((st.flags & DRIVE_STATE_ZERO_SEEN) == 0);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ZERO_FIRST);
    cm(0);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    cm(500); /* a hold run takes no non-zero COMMISSION */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(bench_state_now().drive & 3u, DRIVE_BENCH_RUN_DRIVING);
    CHECK_EQ_U(d.cm_axis, DRIVE_CH_M);
}

/* the first HOLD needs a non-zero frame counted in its tick; a wrong sign, axis or run is bad_value */
static void hold_first_needs_a_fresh_frame(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(hold_arm(83, 1, DRIVE_BENCH_STEER_RANGE));
    cm(0);
    in.abs_fresh[0] = false;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    in.abs_fresh[0] = true;
    hcfg.sign = -1;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hcfg.sign = 1;
    hcfg.run_id = 84;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hcfg.run_id = 83;
    hcfg.duty_max_e4 = 1501; /* above the run's M limit (1500) */
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hcfg.duty_max_e4 = 1500;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    /* a duty run's HOLD */
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    CHECK(kth_arm(85, 0, 1));
    cm(0);
    hcfg.run_id = 85;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
}

/* the targets, fixed once and kept in BENCH_STATE until the next ARM */
static void hold_targets_in_state(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(86, 1000, BENCH_HOLD_TARGET_REL, 40));
    link_bench_state s = bench_state_now();
    CHECK_EQ_I(s.target - s.cumulative, 40);
    boot_with(mod_bench);
    CHECK(hold_start(87, 4000, BENCH_HOLD_TARGET_ZERO, 0));
    s = bench_state_now();
    CHECK_EQ_I(s.target - s.cumulative, 96);
    boot_with(mod_bench);
    CHECK(hold_start(88, 4095, BENCH_HOLD_TARGET_ABS, 1));
    s = bench_state_now();
    CHECK_EQ_I(s.target - s.cumulative, 2);
}

/* toward the target at the minimum duty, 0 inside the deadband; a different HOLD is refused and the drive goes on */
static void hold_drives_and_settles(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(89, 1000, BENCH_HOLD_TARGET_REL, 100));
    CHECK(fabsf(out.duty[DRIVE_CH_M] - 0.12f) < 1e-6f); /* kp 0.5 x 0.153 rad = 0.077: raised to the minimum 0.12 */
    link_bench_hold keep = hcfg;
    hcfg.kp = 0.6f;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    hcfg = keep;
    for (uint16_t raw = 1010; raw <= 1080; raw += 10) {
        kraw(0, raw);
        hold();
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
        CHECK(out.duty[DRIVE_CH_M] > 0.0f);
    }
    kraw(0, 1092); /* 8 counts short: inside the deadband */
    hold();
    CHECK(out.duty[DRIVE_CH_M] == 0.0f);
    for (int k = 0; k < 2000; k++) { /* seconds at rest inside the deadband: no stop (lead's request) */
        if (k % 50 == 0) {
            hold();
        } else {
            tick();
        }
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(bench_state_now().deadband_entries, 1);
}

/* a stalled axis outside the deadband: no_progress at 0.3 s; the result names a hold run and its bypass */
static void hold_stall_stops(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(90, 1000, BENCH_HOLD_TARGET_REL, 100));
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        if (n % 50 == 0) {
            hold();
        } else {
            tick();
        }
        n++;
    }
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 298 && n <= 302);
    CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_HOLD);
    CHECK((s.run_flags & DRIVE_BENCH_FLAG_RES_DISTANCE) == 0);
    CHECK_EQ_U(s.result_bypass, DRIVE_BENCH_STEER_RANGE);
    hold(); /* the run's late HOLD after its end: blocked */
    CHECK_EQ_U(bench_state_now().blocked, 1);
    hcfg.axis = DRIVE_CH_H; /* another axis: not this run's */
    hold();
    CHECK_EQ_U(bench_state_now().blocked, 1);
}

/* the host's zero ends it (end_zero); target and result stay after DISARMED */
static void hold_end_zero(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(91, 1000, BENCH_HOLD_TARGET_REL, 5));
    ticks(5);
    cm(0);
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_END_ZERO);
    CHECK_EQ_I(s.target - s.cut_position, 5);
    CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_HOLD);
}

/* a tentative 0x0000 makes no output: 256 -> 0 -> 256 on a here hold (independent review P1-1) */
static void hold_tentative_zero_drives_nothing(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(92, 256, BENCH_HOLD_TARGET_HERE, 0));
    int nonzero = 0;
    for (int k = 0; k < 200; k++) {
        kraw(0, k % 7 == 3 ? 0 : 256);
        if (k % 50 == 0) {
            hold();
        } else {
            tick();
        }
        nonzero += out.duty[DRIVE_CH_M] != 0.0f;
    }
    CHECK_EQ_I(nonzero, 0);
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
}

/* here at raw 16, kp 1, ki = kd = min = 0, deadband 12, then 0 frames only: stopped within 0.3 s as 11 */
static void hold_stuck_zero_stops(void)
{
    boot_with(mod_bench);
    kraw(0, 16);
    ticks(3);
    CHECK(hold_arm(93, 1, DRIVE_BENCH_STEER_RANGE));
    cm(0);
    hcfg.kp = 1.0f;
    hcfg.min_duty_e4 = 0;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    kraw(0, 0);
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        if (n % 50 == 0) {
            hold();
        } else {
            tick();
        }
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_KTH_ZERO_SUSPECT);
    CHECK(n <= 302);
}

/* the hold axis's range is judged every tick: without the bypass a hold leaving the range faults */
static void hold_range_is_judged(void)
{
    for (int bypass = 0; bypass < 2; bypass++) {
        boot_with(mod_bench);
        kraw(0, 300); /* about 0.46 rad: inside the steering range (+-0.5) */
        ticks(3);
        CHECK(hold_arm(94, 1, bypass ? DRIVE_BENCH_STEER_RANGE : 0u));
        cm(0);
        hcfg.target_kind = BENCH_HOLD_TARGET_REL;
        hcfg.target_value = 100;
        hold();
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
        kraw(0, 340); /* about 0.52 rad: out of range */
        ticks(5);
        if (bypass) {
            CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
        } else {
            CHECK(d.mode != DRIVE_MODE_ARMED);
            CHECK(d.fault_flags & DRIVE_FAULT_STEER_OUT_OF_RANGE);
        }
    }
}

/* the echo must be of a STATE after the last connection change: re-armed on the new connection with its zero, a
   HOLD echoing a STATE before the boundary is stale_echo, one at the boundary is taken (Codex round 1) */
static void hold_echo_boundary(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    uint64_t old = echo_now();
    drive_note_link_change(&d, in.now_us, true);
    tick();
    uint64_t boundary = d.boundary_us;
    CHECK(old < boundary);
    CHECK(hold_arm(95, 1, DRIVE_BENCH_STEER_RANGE));
    cm(0);
    link_msg m = hold_msg();
    m.u.bench_hold.echo_mcu_time_us = old;
    send(m);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STALE_ECHO);
    m = hold_msg();
    m.u.bench_hold.echo_mcu_time_us = boundary;
    send(m);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
}

/* the HAL report: an armed interval closes only on a reported whole zero window, not on a requested 0 (decision 6) */
static void hold_interval_closes_on_the_report(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(96, 1000, BENCH_HOLD_TARGET_REL, 100));
    tick();
    CHECK(d.cm_on);
    uint64_t since = d.cm_on_since_us;
    pwm_sim = false; /* the reports below are set by hand */
    in.pwm_report = true;
    in.applied_window_us = 1000;
    in.abs_start_known[0] = true;
    in.applied_e4[DRIVE_CH_M] = 1200;
    in.applied_any[DRIVE_CH_M] = true;
    kraw(0, 1095); /* inside the deadband: the request goes to 0 */
    in.abs_start_us[0] = in.now_us + cfg.period_us;
    tick();
    CHECK(out.duty[DRIVE_CH_M] == 0.0f);
    CHECK(d.cm_on); /* the 0 is requested, not yet reported active */
    in.applied_e4[DRIVE_CH_M] = 0;
    in.abs_start_us[0] = in.now_us + cfg.period_us;
    tick(); /* 0 active now, but the window held the 1200 */
    CHECK(d.cm_on);
    CHECK_EQ_U(d.cm_on_since_us, since);
    /* a whole zero window closes the old budget at the tick's start and, in that same tick, a sample outside the
       deadband opens a new one (Codex A1b-2a round 2) */
    in.applied_any[DRIVE_CH_M] = false;
    kraw(0, 1000);
    in.abs_start_us[0] = in.now_us + cfg.period_us;
    tick();
    CHECK(out.duty[DRIVE_CH_M] != 0.0f);
    CHECK(d.cm_on);
    CHECK_EQ_U(d.cm_on_since_us, d.time_us);
    /* a tick without a report never closes it */
    uint64_t since2 = d.cm_on_since_us;
    kraw(0, 1095);
    in.applied_e4[DRIVE_CH_M] = 1200;
    in.applied_any[DRIVE_CH_M] = true;
    in.abs_start_us[0] = in.now_us + cfg.period_us;
    tick(); /* 0 requested */
    in.pwm_report = false;
    kraw(0, 1000);
    tick(); /* the report is missing: open still */
    CHECK(d.cm_on);
    CHECK_EQ_U(d.cm_on_since_us, since2);
}

/* the driven time is the HAL's read-to-read window, not the tick: 1020 us windows stop a stall at about 295 ticks
   (Codex A1b-2a round 2) */
static void hold_time_is_the_hal_window(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(102, 1000, BENCH_HOLD_TARGET_REL, 100));
    pwm_sim = false;
    in.pwm_report = true;
    in.applied_window_us = 1020;
    in.applied_e4[DRIVE_CH_M] = 1200;
    in.applied_any[DRIVE_CH_M] = true;
    in.abs_start_known[0] = true;
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        in.abs_start_us[0] = in.now_us + cfg.period_us - 50u;
        if (n % 50 == 0) {
            hold();
        } else {
            tick();
        }
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 292 && n <= 296);
    in.pwm_report = false;
}

/* a skipped HAL read: the fallback's tick is not counted again from the HAL window that follows (Codex round 3) */
static void hold_skipped_read_counts_once(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(103, 1000, BENCH_HOLD_TARGET_REL, 100));
    pwm_sim = false;
    in.applied_e4[DRIVE_CH_M] = 1200;
    in.applied_any[DRIVE_CH_M] = true;
    in.abs_start_known[0] = true;
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        bool skip = n == 150;
        in.pwm_report = !skip;
        in.applied_window_us = n == 151 ? 2000u : 1000u; /* the HAL's window across the skipped read */
        in.abs_start_us[0] = in.now_us + cfg.period_us - 50u;
        if (n % 50 == 0) {
            hold();
        } else {
            tick();
        }
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK_EQ_I(n, 300); /* counted twice, the skipped tick would stop it at 299 */
    in.pwm_report = false;
}

/* the run's time limit stops a hold whose output never reached a whole zero window: 2 ms on M, a 0 requested
   after it, the HAL still reporting the old 1200 (no update), then a request out of the deadband again -> STOPPING
   as 3 with commission_time_limit, before the new request (Codex round 3) */
static void hold_time_limit_stops(void)
{
    boot_with(mod_bench);
    kraw(0, 1000);
    ticks(3);
    link_bench_set b = bench_defaults(104, DRIVE_CH_M);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_KTH;
    b.bypass = DRIVE_BENCH_STEER_RANGE;
    b.time_ms[DRIVE_CH_M] = 2;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 104, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    hcfg = (link_bench_hold){.boot_id = d.boot_id, .arm_gen = d.arm_gen, .valid_after_echo_ms = 100, .run_id = 104,
                             .axis = DRIVE_CH_M, .source = DRIVE_BENCH_SOURCE_KTH,
                             .target_kind = BENCH_HOLD_TARGET_REL, .target_value = 100, .cpr = 4096, .kp = 0.5f,
                             .deadband_counts = 12, .duty_max_e4 = 1500, .min_duty_e4 = 1200, .sign = 1};
    cm(0);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    pwm_sim = false;
    in.pwm_report = true;
    in.applied_window_us = 1000;
    in.applied_e4[DRIVE_CH_M] = 1200;
    in.applied_any[DRIVE_CH_M] = true;
    in.abs_start_known[0] = true;
    in.abs_start_us[0] = in.now_us + cfg.period_us - 50u;
    tick(); /* 1 ms into the interval */
    kraw(0, 1095); /* 2 ms: inside, 0 requested, not over the limit yet, the 1200 still active */
    in.abs_start_us[0] = in.now_us + cfg.period_us - 50u;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(out.duty[DRIVE_CH_M] == 0.0f);
    kraw(0, 1000); /* 3 ms: out again with the 1200 reported -> the limit first */
    in.abs_start_us[0] = in.now_us + cfg.period_us - 50u;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_TIME_LIMIT);
    CHECK(d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    CHECK(out.duty[DRIVE_CH_M] == 0.0f);
    in.pwm_report = false;
}

static void mod_bench_m_reversed(drive_config *c)
{
    mod_bench(c);
    c->duty_sign[DRIVE_CH_M] = -1;
}

/* progress needs the HAL's report: slow real progress (2 counts every 100 ms) keeps a hold going with the board's
   report, and without any report the same run stops at 0.3 s -- the core never credits progress it cannot know
   (Codex A1b-2a round 1 P1-2) */
static void hold_progress_needs_the_report(void)
{
    for (int with = 0; with < 3; with++) {
        boot_with(with == 2 ? mod_bench_m_reversed : mod_bench); /* 2: M's duty_sign -1 (physical -> request) */
        CHECK(hold_start(99, 1000, BENCH_HOLD_TARGET_REL, 400));
        pwm_sim = with != 0;
        if (!with) {
            in.pwm_report = false;
        }
        uint16_t raw = 1000;
        int n = 0;
        while (d.mode == DRIVE_MODE_ARMED && n < 1000) {
            if (n % 100 == 99) {
                raw = (uint16_t)(raw + 2);
                kraw(0, raw);
            }
            if (n % 50 == 0) {
                hold();
            } else {
                tick();
            }
            n++;
        }
        if (with) {
            CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
        } else {
            CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
            CHECK(n <= 302);
        }
    }
}

/* the run axis's zero only: after a hold ends, another axis's 0 is not this run's blocked request (Codex round 1) */
static void hold_blocked_is_its_axis_only(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(100, 1000, BENCH_HOLD_TARGET_HERE, 0));
    cm(0);
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    send(commission_msg(cm_seq++, DRIVE_CH_L, 0, 150, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NOT_ARMED);
    CHECK_EQ_U(bench_state_now().blocked, 0);
    cm(0);
    CHECK_EQ_U(bench_state_now().blocked, 1);
}

/* a deadband entry consumed in the tick the command expires is counted (Codex round 1) */
static void hold_entry_on_the_end_tick(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(101, 1000, BENCH_HOLD_TARGET_REL, 13));
    while (d.mode == DRIVE_MODE_ARMED && d.time_us < d.deadline_us) {
        tick();
    }
    kraw(0, 1001); /* e 12: inside, consumed in the expiring tick */
    tick();
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_COMMAND_TIMEOUT);
    CHECK_EQ_U(s.deadband_entries, 1);
}

/* DISARM ends a hold as 4; another arming's HOLD (old arm_gen) is refused; run A's late HOLD leaves B's blocked */
static void hold_disarm_generation_and_attribution(void)
{
    boot_with(mod_bench);
    CHECK(hold_start(97, 1000, BENCH_HOLD_TARGET_HERE, 0));
    link_msg old = hold_msg();
    send(simple(LINK_DISARM, cm_seq++));
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_DISARM);
    ticks(3);
    CHECK(hold_start(98, 1000, BENCH_HOLD_TARGET_HERE, 0)); /* run B */
    old.seq = cm_seq++;
    old.u.bench_hold.echo_mcu_time_us = echo_now();
    send(old); /* A's HOLD: A's arm_gen and run id */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_ARM_GEN_MISMATCH);
    cm(0); /* B ends */
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_END_ZERO);
    old.seq = cm_seq++;
    old.u.bench_hold.echo_mcu_time_us = echo_now();
    send(old); /* A's late HOLD while B stops: not B's */
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(bench_state_now().blocked, 0);
    hold(); /* B's own late HOLD */
    CHECK_EQ_U(bench_state_now().blocked, 1);
}

static void bench_hold_tests(void)
{
    hold_disarm_generation_and_attribution();
    hold_set_rules();
    hold_readiness_on_the_run_axis();
    hold_first_needs_a_fresh_frame();
    hold_targets_in_state();
    hold_drives_and_settles();
    hold_stall_stops();
    hold_end_zero();
    hold_tentative_zero_drives_nothing();
    hold_stuck_zero_stops();
    hold_range_is_judged();
    hold_echo_boundary();
    hold_interval_closes_on_the_report();
    hold_progress_needs_the_report();
    hold_time_is_the_hal_window();
    hold_skipped_read_counts_once();
    hold_time_limit_stops();
    hold_blocked_is_its_axis_only();
    hold_entry_on_the_end_tick();
}

static void bench_review_round1_tests(void)
{
    bench_watch_counts_from_the_last_progress();
    bench_cut_and_a_command_in_the_same_tick();
    bench_cut_and_an_other_axis_fault_in_the_same_tick();
    bench_cut_and_the_command_expiry_in_the_same_tick();
    bench_evidence_after_disarmed();
    bench_distance_across_the_clock_wrap();
}

/* ---- bench8: the MT source (plan 4.5 A1c) ------------------------------------------------------------------ */

static void wstep(int i, int counts)
{
    in.wheel_hw[i] = (uint16_t)(in.wheel_hw[i] + counts);
}

static void cml(uint8_t axis, int16_t duty)
{
    send(commission_msg(cm_seq++, axis, duty, 150, echo_now()));
}

/* An MT run on a wheel axis (L or R): stop counts (0: measuring), the count sign, the axis's duty limit. */
static bool mt_arm(uint32_t run_id, uint8_t axis, int32_t stop, int8_t count_sign, uint16_t duty_max)
{
    link_bench_set b = bench_defaults(run_id, axis);
    b.source = DRIVE_BENCH_SOURCE_MT;
    b.stop_counts = stop;
    b.count_sign = count_sign;
    b.duty_max_e4[axis] = duty_max;
    send(bench_set_msg(10, b));
    if (d.reject_reason != DRIVE_REJECT_NONE) {
        return false;
    }
    send(bench_arm_msg(11, run_id, echo_now()));
    return d.mode == DRIVE_MODE_ARMED;
}

static void mt_pairs(void)
{
    boot_with(mod_bench);
    ticks(3);
    const uint8_t bad[4][2] = {{DRIVE_CH_L, DRIVE_BENCH_SOURCE_KTH},
                               {DRIVE_CH_M, DRIVE_BENCH_SOURCE_MT},
                               {DRIVE_CH_H, DRIVE_BENCH_SOURCE_MT},
                               {DRIVE_CH_R, DRIVE_BENCH_SOURCE_KTH}};
    for (int k = 0; k < 4; k++) {
        link_bench_set b = bench_defaults(70 + (uint32_t)k, bad[k][0]);
        b.source = bad[k][1];
        send(bench_set_msg(10, b));
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    }
    for (uint8_t ax = DRIVE_CH_L; ax <= DRIVE_CH_R; ax++) {
        link_bench_set b = bench_defaults(80 + ax, ax);
        b.source = DRIVE_BENCH_SOURCE_MT;
        b.stop_counts = 40;
        send(bench_set_msg(10, b));
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    }
}

/* L forward and R backward to a distance, across the 16-bit counter's wrap on R. */
static void mt_distance_reached(void)
{
    for (int side = 0; side < 2; side++) {
        boot_with(mod_bench);
        uint8_t ax = side == 0 ? DRIVE_CH_L : DRIVE_CH_R;
        int dir = side == 0 ? 1 : -1;
        in.wheel_hw[side] = side == 0 ? 1000 : 20; /* R crosses 0 -> 65535 going down */
        ticks(3);
        CHECK(mt_arm(90 + (uint32_t)side, ax, 40 * dir, 1, 1500));
        cml(ax, 0);
        cml(ax, (int16_t)(1000 * dir));
        CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
        int32_t ref = bench_state_now().distance_ref;
        int n = 0;
        while (d.mode == DRIVE_MODE_ARMED && n < 50) {
            wstep(side, 3 * dir);
            tick();
            n++;
        }
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        link_bench_state s = bench_state_now();
        CHECK_EQ_U(s.source, DRIVE_BENCH_SOURCE_MT);
        CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED);
        CHECK((s.cut_position - ref) * dir >= 40 && (s.cut_position - ref) * dir < 43);
        CHECK_EQ_U(s.run_flags, DRIVE_BENCH_FLAG_RES_DISTANCE); /* no tentative, no unclean */
        CHECK(s.end_evidence_age_ms <= 1u);                     /* qualified progress before the cut */
        CHECK_EQ_U(s.distance_confirm, DRIVE_BENCH_CONFIRM_NONE); /* no provisional cut for MT */
        CHECK_EQ_I(n, 14);
    }
}

static void mt_distance_wrong_way_and_frozen(void)
{
    boot_with(mod_bench);
    ticks(3);
    CHECK(mt_arm(95, DRIVE_CH_L, 400, 1, 1500));
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 1000);
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 100) {
        wstep(0, -4); /* counts down under a positive duty with count_sign +1 */
        tick();
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    CHECK(n >= 32 && n <= 34); /* 128 counts the wrong way */

    boot_with(mod_bench);
    ticks(3);
    CHECK(mt_arm(96, DRIVE_CH_L, 400, 1, 1500));
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 1000);
    uint64_t t0 = d.time_us;
    n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 400) {
        if (n % 100 == 0) {
            cml(DRIVE_CH_L, 1000); /* renewed: the command does not expire */
        }
        tick();
        n++;
    }
    CHECK_EQ_U(bench_state_now().stop_reason, DRIVE_BENCH_STOP_NO_PROGRESS);
    uint64_t ms = (d.time_us - t0) / 1000u; /* the renewals are ticks too */
    CHECK(ms >= 299 && ms <= 302); /* the 0.3 s watch */
    CHECK_EQ_U(bench_state_now().end_evidence_age_ms, 0xFFFFu); /* no progress, no evidence */
}

/* A failed read: the run wheel's gap faults its MT run; another run only reports it (lead decision 2026-11-28). */
static void mt_read_failure_scope(void)
{
    boot_with(mod_bench);
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    ticks(3);
    CHECK(mt_arm(97, DRIVE_CH_L, 0, 1, 1500)); /* measuring */
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 1000);
    for (int k = 0; k < 5; k++) {
        wstep(0, 3);
        tick();
    }
    in.wheel_fresh[0] = false;
    uint64_t t0 = d.time_us;
    int n = 0;
    while (d.mode == DRIVE_MODE_ARMED && n < 40) {
        if (n % 10 == 0) {
            cml(DRIVE_CH_L, 1000);
        }
        tick();
        n++;
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK_EQ_U((d.time_us - t0) / 1000u, 21u); /* the first tick past 20 ms since the last success, no new sample */
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
    link_bench_state s = bench_state_now();
    CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_FAULT);
    CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_UNCLEAN);

    /* the same failure under an M duty run without a source: no fault, the bit reported, the next ARM clears it */
    boot_with(mod_bench);
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    kraw(0, 1000);
    ticks(3);
    CHECK(bench_arm_run(98, DRIVE_CH_M, DRIVE_BENCH_STEER_RANGE));
    in.wheel_fresh[0] = false;
    for (int k = 0; k < 30; k++) {
        if (k % 10 == 0) {
            cm(0);
        }
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
    send(simple(LINK_DISARM, cm_seq++));
    ticks(150);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    /* still failing: an MT ARM of L is refused, M's runs arm */
    link_bench_set b = bench_defaults(99, DRIVE_CH_L);
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 99, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    in.wheel_fresh[0] = true; /* the first success re-bases L */
    tick();
    CHECK(mt_arm(100, DRIVE_CH_L, 0, 1, 1500));
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_GAP)); /* the ARM cleared the report */
}

/* One failed read, then the 2 ms step: judged on its real interval, no jump (plan 4.5 decision 4). */
static void mt_short_failure_recovers(void)
{
    boot_with(mod_bench);
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    ticks(3);
    CHECK(mt_arm(101, DRIVE_CH_L, 0, 1, 1500));
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 1000);
    for (int k = 0; k < 5; k++) {
        wstep(0, 50); /* 50 counts/ms; the 1 ms jump limit here is about 66 counts */
        tick();
    }
    wstep(0, 50);
    in.wheel_fresh[0] = false;
    tick();
    CHECK_EQ_U(bench_state_now().raw_angle, 1u); /* the last accepted sample is a tick old: at least 1 */
    wstep(0, 50);
    in.wheel_fresh[0] = true;
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* 100 counts over 2 ms: no jump */
    CHECK(!(d.fault_flags & (DRIVE_FAULT_ENCODER_JUMP(0) | DRIVE_FAULT_ENCODER_GAP)));
    CHECK_EQ_U(bench_state_now().raw_angle, 0u); /* accepted at this tick */
}

/* The polarity check of the MT run's wheel follows the declared count sign (independent round 3 P1). */
static void mt_polarity_follows_count_sign(void)
{
    boot_with(mod_bench);
    ticks(3);
    CHECK(mt_arm(102, DRIVE_CH_L, 0, -1, 5000)); /* measuring, counts fall under a positive duty */
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 4000);
    for (int k = 0; k < 600; k++) {
        if (k % 100 == 0) {
            cml(DRIVE_CH_L, 4000);
        }
        wstep(0, -5);
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(!(d.fault_flags & DRIVE_FAULT_ENCODER_POLARITY(0)));
}

/* An MT hold: here, settling, the CPR and the target kinds (plan 4.5 decision 6). */
static void mt_hold(void)
{
    boot_with(mod_bench);
    pwm_sim = true;
    ticks(3);
    link_bench_set b = bench_defaults(103, DRIVE_CH_L);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    send(bench_arm_msg(11, 103, echo_now()));
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    hcfg = (link_bench_hold){.boot_id = d.boot_id,
                             .valid_after_echo_ms = 100,
                             .run_id = 103,
                             .axis = DRIVE_CH_L,
                             .source = DRIVE_BENCH_SOURCE_MT,
                             .target_kind = BENCH_HOLD_TARGET_ZERO,
                             .cpr = 4096,
                             .kp = 0.5f,
                             .deadband_counts = 12,
                             .duty_max_e4 = 1500,
                             .min_duty_e4 = 1200,
                             .sign = 1};
    cml(DRIVE_CH_L, 0);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE); /* no zero target for MT */
    hcfg.target_kind = BENCH_HOLD_TARGET_ABS;
    hcfg.target_value = 100;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hcfg.target_kind = BENCH_HOLD_TARGET_REL;
    hcfg.target_value = 200;
    hcfg.cpr = 0;
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    hcfg.cpr = 8192; /* any motor shaft CPR */
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    link_bench_state s = bench_state_now();
    CHECK_EQ_I(s.target - s.cumulative, 200);
    /* the wheel follows the output: 1 count a tick per 0.1 of duty */
    int settled = 0;
    for (int k = 0; k < 400 && d.mode == DRIVE_MODE_ARMED; k++) {
        if (k % 50 == 0) {
            hold();
        }
        float u = out.duty[DRIVE_CH_L];
        wstep(0, (int)lroundf(u * 10.0f));
        tick();
        s = bench_state_now();
        settled = labs((long)s.target - (long)s.cumulative) <= 12 ? settled + 1 : 0;
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED);
    CHECK(settled > 100);
    CHECK(bench_state_now().deadband_entries >= 1);
    CHECK(bench_state_now().evidence_age_ms < 0xFFFFu); /* the moving approach left evidence */
}

/* A frozen encoder under an MT here: no evidence at all (plan 4.5 decision 8), then the watch. */
static void mt_hold_frozen_here_has_no_evidence(void)
{
    boot_with(mod_bench);
    pwm_sim = true;
    ticks(3);
    link_bench_set b = bench_defaults(104, DRIVE_CH_L);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 104, echo_now()));
    hcfg = (link_bench_hold){.boot_id = d.boot_id,
                             .valid_after_echo_ms = 100,
                             .run_id = 104,
                             .axis = DRIVE_CH_L,
                             .source = DRIVE_BENCH_SOURCE_MT,
                             .target_kind = BENCH_HOLD_TARGET_HERE,
                             .cpr = 4096,
                             .kp = 0.5f,
                             .deadband_counts = 12,
                             .duty_max_e4 = 1500,
                             .min_duty_e4 = 1200,
                             .sign = 1};
    cml(DRIVE_CH_L, 0);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    for (int k = 0; k < 200; k++) {
        if (k % 50 == 0) {
            hold();
        }
        tick();
    }
    CHECK_EQ_U(bench_state_now().evidence_age_ms, 0xFFFFu);
    cml(DRIVE_CH_L, 0); /* the end zero */
    CHECK_EQ_U(bench_state_now().end_evidence_age_ms, 0xFFFFu);
}

static void mod_bench_l_read_fails(drive_config *c)
{
    mod_bench(c);
    in.wheel_report = true;
    in.wheel_fresh[0] = false; /* L's reads fail from boot */
    in.wheel_fresh[1] = true;
}

/* L failing from boot: reported after 20 ms, M and R still arm, only L's MT ARM is refused (plan 4.5 decision 4). */
static void mt_boot_read_failure(void)
{
    boot_with(mod_bench_l_read_fails);
    ticks(30);
    CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
    CHECK_EQ_U(bench_state_now().count_state, DRIVE_BENCH_COUNT_NONE_YET); /* no run yet: nothing measured */
    kraw(0, 1000);
    ticks(2);
    CHECK(bench_arm_run(111, DRIVE_CH_M, DRIVE_BENCH_STEER_RANGE));
    send(simple(LINK_DISARM, cm_seq++));
    ticks(150);
    link_bench_set b = bench_defaults(112, DRIVE_CH_L);
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 112, echo_now()));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    CHECK(mt_arm(113, DRIVE_CH_R, 0, 1, 1500)); /* R's reads are fine */
}

/* A result fixed by an SPI answer before this tick's wheel samples is tentative, and a loss of the result's own wheel in
   that next tick is attributed to it; another wheel's is not (plan 4.5 decision 10). */
static void mt_spi_end_is_tentative_and_takes_its_wheels_loss(void)
{
    for (int jw = 0; jw < 2; jw++) {
        boot_with(mod_bench);
        ticks(3);
        CHECK(mt_arm(114 + (uint32_t)jw, DRIVE_CH_L, 0, 1, 1500)); /* measuring */
        cml(DRIVE_CH_L, 0);
        cml(DRIVE_CH_L, 1000);
        for (int k = 0; k < 3; k++) {
            wstep(0, 3);
            tick();
        }
        vm_ok = false; /* the next status poll's answer is invalid: a driver fault between ticks */
        int k = 0;
        while (d.mode != DRIVE_MODE_FAULT && k < 200) {
            if (k % 50 == 0) {
                cml(DRIVE_CH_L, 1000);
            }
            if (d.mode != DRIVE_MODE_FAULT) {
                wstep(0, 3);
                tick();
            }
            k++;
        }
        CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
        link_bench_state s = bench_state_now();
        CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_FAULT);
        CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_TENTATIVE); /* fixed before the next tick's wheel samples */
        CHECK(!(s.run_flags & DRIVE_BENCH_FLAG_RES_UNCLEAN));
        wstep(jw, 3000); /* the next tick: a jump on wheel jw */
        tick();
        s = bench_state_now();
        if (jw == 0) {
            CHECK(s.run_flags & DRIVE_BENCH_FLAG_RES_UNCLEAN); /* the result's own wheel */
        } else {
            CHECK(!(s.run_flags & DRIVE_BENCH_FLAG_RES_UNCLEAN)); /* another wheel's loss is not the result's */
        }
        CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(jw)); /* reported either way */
    }
}

/* An MT hold gives no output on a tick whose read failed (plan 4.5 decision 6), and drives again on the next. */
static void mt_hold_failed_read_gives_no_output(void)
{
    boot_with(mod_bench);
    pwm_sim = true;
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    ticks(3);
    link_bench_set b = bench_defaults(116, DRIVE_CH_L);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 116, echo_now()));
    hcfg = (link_bench_hold){.boot_id = d.boot_id,
                             .valid_after_echo_ms = 100,
                             .run_id = 116,
                             .axis = DRIVE_CH_L,
                             .source = DRIVE_BENCH_SOURCE_MT,
                             .target_kind = BENCH_HOLD_TARGET_REL,
                             .target_value = 500,
                             .cpr = 4096,
                             .kp = 0.5f,
                             .deadband_counts = 12,
                             .duty_max_e4 = 1500,
                             .min_duty_e4 = 1200,
                             .sign = 1};
    cml(DRIVE_CH_L, 0);
    hold();
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    ticks(3);
    CHECK(out.duty[DRIVE_CH_L] > 0.1f);
    in.wheel_fresh[0] = false;
    tick();
    CHECK(out.duty[DRIVE_CH_L] == 0.0f); /* the core's request on the failed tick */
    in.wheel_fresh[0] = true;
    tick();
    CHECK(out.duty[DRIVE_CH_L] > 0.1f);
}

/* A jump on the tick of an MT ARM: the wheel stays lost until the next success, so that ARM is refused (Codex A1c-1 r1
   P2-1); the next tick's success re-bases it. */
static void mt_arm_on_a_jump_tick_is_refused(void)
{
    boot_with(mod_bench);
    ticks(3);
    link_bench_set b = bench_defaults(120, DRIVE_CH_L);
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_NONE);
    wstep(0, 3000); /* a jump read in this tick's drive_begin, before the ARM is handled */
    link_msg arm = bench_arm_msg(11, 120, echo_now());
    tick_msgs(&arm, 1);
    CHECK_EQ_U(d.reject_reason, DRIVE_REJECT_SENSOR_INVALID);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_JUMP(0)); /* reported, not cleared by a refused ARM */
    tick(); /* the next success re-bases the wheel */
    CHECK(mt_arm(121, DRIVE_CH_L, 0, 1, 1500));
}

/* The failed-read gap is judged on the wheel's own read time, not the tick's (Codex A1c-1 r1 P2-2). */
static void mt_gap_on_the_read_time(void)
{
    boot_with(mod_bench);
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    ticks(3);
    CHECK(mt_arm(122, DRIVE_CH_L, 400, 1, 1500));
    cml(DRIVE_CH_L, 0);
    cml(DRIVE_CH_L, 1000);
    wstep(0, 3);
    tick(); /* the last success, read at the tick's time */
    in.wheel_fresh[0] = false;
    for (int k = 0; k < 19; k++) {
        tick();
    }
    CHECK_EQ_U(d.mode, DRIVE_MODE_ARMED); /* 19 ms of ticks */
    wheel_start_off[0] = 1300; /* the next tick is 20 ms after the last success, its L read 21.3 ms */
    tick();
    CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
    CHECK(d.fault_flags & DRIVE_FAULT_ENCODER_GAP);
}

/* The braking boundary (plan 4.5 decision 4): a loss on the tick the grace ends is still STOPPING -- a fault, the
   result kept; after DISARMED it only invalidates the rest observation. */
static void mt_loss_at_and_after_the_brake(void)
{
    for (int after = 0; after < 2; after++) {
        boot_with(mod_bench);
        ticks(3);
        CHECK(mt_arm(123 + (uint32_t)after, DRIVE_CH_L, 40, 1, 1500));
        cml(DRIVE_CH_L, 0);
        cml(DRIVE_CH_L, 1000);
        while (d.mode == DRIVE_MODE_ARMED) {
            wstep(0, 3);
            tick();
        }
        CHECK_EQ_U(d.mode, DRIVE_MODE_STOPPING);
        uint64_t until = d.stop_until_us;
        while (d.time_us + cfg.period_us < until) {
            tick();
        }
        if (after) {
            tick(); /* the grace ends: DISARMED */
            CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
        }
        wstep(0, 3000);
        tick(); /* after == 0: the tick that reaches stop_until, still STOPPING when the wheels are read */
        link_bench_state s = bench_state_now();
        CHECK_EQ_U(s.stop_reason, DRIVE_BENCH_STOP_DISTANCE_REACHED); /* the result is kept either way */
        if (after) {
            CHECK_EQ_U(d.mode, DRIVE_MODE_DISARMED);
            CHECK(s.run_flags & DRIVE_BENCH_FLAG_UNCLEAN); /* the live mark: the rest observation is void */
            CHECK(!(s.run_flags & DRIVE_BENCH_FLAG_RES_UNCLEAN));
        } else {
            CHECK_EQ_U(d.mode, DRIVE_MODE_FAULT);
        }
    }
}

/* The board's wheel slew (2.0 /s): a deadband entry's 0 is applied at once, and after a failed read the output rises
   again from 0 by the slew only (plan 4.5 decision 6). */
static void mod_bench_board_slew(drive_config *c)
{
    mod_bench(c);
    c->wheel_duty_slew_per_s = 2.0f;
}

static void mt_hold_on_the_board_slew(void)
{
    boot_with(mod_bench_board_slew);
    pwm_sim = true;
    in.wheel_report = true;
    in.wheel_fresh[0] = in.wheel_fresh[1] = true;
    ticks(3);
    link_bench_set b = bench_defaults(125, DRIVE_CH_L);
    b.mode = DRIVE_BENCH_MODE_HOLD;
    b.source = DRIVE_BENCH_SOURCE_MT;
    send(bench_set_msg(10, b));
    send(bench_arm_msg(11, 125, echo_now()));
    hcfg = (link_bench_hold){.boot_id = d.boot_id,
                             .valid_after_echo_ms = 100,
                             .run_id = 125,
                             .axis = DRIVE_CH_L,
                             .source = DRIVE_BENCH_SOURCE_MT,
                             .target_kind = BENCH_HOLD_TARGET_REL,
                             .target_value = 40,
                             .cpr = 4096,
                             .kp = 0.5f,
                             .deadband_counts = 12,
                             .duty_max_e4 = 1500,
                             .min_duty_e4 = 1200,
                             .sign = 1};
    cml(DRIVE_CH_L, 0);
    hold();
    for (int k = 0; k < 80; k++) {
        tick(); /* the output ramps up at 0.002 a tick */
    }
    CHECK(out.duty[DRIVE_CH_L] > 0.1f);
    in.wheel_fresh[0] = false;
    tick();
    CHECK(out.duty[DRIVE_CH_L] == 0.0f); /* the failed read's 0, at once */
    in.wheel_fresh[0] = true;
    tick();
    CHECK(out.duty[DRIVE_CH_L] > 0.0f && out.duty[DRIVE_CH_L] <= 0.0021f); /* back up by the slew only */
    float u = out.duty[DRIVE_CH_L];
    for (int k = 0; k < 100 && u > 0.0f; k++) {
        wstep(0, 1);
        tick();
        u = out.duty[DRIVE_CH_L];
        link_bench_state s = bench_state_now();
        if (labs((long)s.target - (long)s.cumulative) <= 12) {
            CHECK(u == 0.0f); /* the deadband's 0 at once, never slewed down */
            break;
        }
    }
}

static void bench_mt_tests(void)
{
    mt_arm_on_a_jump_tick_is_refused();
    mt_gap_on_the_read_time();
    mt_loss_at_and_after_the_brake();
    mt_hold_on_the_board_slew();
    mt_pairs();
    mt_distance_reached();
    mt_distance_wrong_way_and_frozen();
    mt_read_failure_scope();
    mt_short_failure_recovers();
    mt_polarity_follows_count_sign();
    mt_hold();
    mt_hold_frozen_here_has_no_evidence();
    mt_boot_read_failure();
    mt_spi_end_is_tentative_and_takes_its_wheels_loss();
    mt_hold_failed_read_gives_no_output();
}

int main(void)
{
    boot_and_retry();
    arming();
    arming_needs_valid_sensors();
    arm_rejection_order();
    freshness_and_deadline(0);
    freshness_and_deadline(0xFFFFFFFFu - 50000u); /* MCU clock wraps around the deadline */
    expiry_wins_over_a_late_drive();
    stopping_keeps_outputs_on();
    seq_rules();
    replay_after_reboot();
    estop_latch_and_clear();
    driver_fault_and_reinit();
    overcurrent();
    current_sense_and_sensor_faults();
    simultaneous_causes();
    overruns();
    config_validation();
    control_directions_and_limits();
    state_report();
    kth_counters_in_state();
    clock_wrap_replay();
    spi_refusal_and_timeout();
    arm_alone_expires();
    fault_on_the_last_stopping_cycle_latches();
    stale_spi_response_after_reinit_is_drained();
    config_rejects_negative_gains_and_huge_zero();
    state_flags_follow_messages();
    refused_starts_do_not_cycle_spi_tags();
    v2_build_bits_and_estop_unwired();
    v2_config_and_init();
    v2_wrong_build();
    v2_commission_checks_and_outputs();
    v2_commission_duty_sign();
    v2_commission_time_limit();
    v2_commission_encoder_reports_only();
    v2_gap_and_jump();
    v2_stuck();
    v2_polarity();
    v2_slew();
    v2_state_carries_reset_cause();
    v2r2_disarm_rearm_does_not_reset_commission_clock();
    v2r2_first_cycle_gap_drops_the_step();
    v2r2_polarity_baseline_waits_for_a_fresh_window();
    v2r2_commission_stopping_brakes_every_axis();
    v2r3_rearm_bundle_after_the_limit_waits_the_stop();
    v2r4_limit_and_expiry_in_the_same_cycle();
    v2r5_limit_hold_survives_a_fault_in_the_same_cycle();
    v2r6_disarm_in_the_limit_stop();
    s25_duty_limits_per_axis_in_integers();
    s25_time_limit_is_the_interval_axis();
    s25_disarm_alone_keeps_the_interval_axis();
    s25_axis_change_inside_an_interval();
    s25_each_axis_reads_its_own_time_limit();
    bench_config_validation();
    bench_set_refusals();
    bench_arm_needs_its_set();
    bench_duty_run_ends_itself();
    bench_run_end_causes();
    bench_range_bypass_is_the_runs();
    bench_link_boundary();
    bench_disarm_keeps_the_brake();
    bench_run_limit_outlives_the_run();
    bench_set_does_not_outlive_an_estop();
    bench_blocked_counts_the_end_only();
    bench_change_brake_is_full_length();
    bench_change_versus_expiry_order();
    bench_change_versus_time_limit();
    bench_limit_change_deadline_order();
    bench_blocked_other_axis_after_disarm();
    bench_bench6_tests();
    bench_review_round1_tests();
    bench_hold_tests();
    bench_mt_tests();
    return FW_TEST_RESULT();
}
