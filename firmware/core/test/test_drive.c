#include "fw/drive.h"
#include "fw_test.h"

#include <math.h>
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
    drive_begin(&d, &in);
    for (int i = 0; i < count; i++) {
        drive_handle(&d, &msgs[i]);
    }
    drive_finish(&d, &out);
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

static void mod_commission(drive_config *c)
{
    c->estop_wired = false;
    c->commission_build = true;
    c->commission_duty_max = 0.15f;
    c->commission_time_ms = 2000;
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
    c.commission_duty_max = 0.15f;
    c.commission_time_ms = 2000;
    CHECK(!drive_config_valid(&c));
    c = make_config();
    c.commission_build = true;
    c.commission_duty_max = 0.0f;
    c.commission_time_ms = 2000;
    CHECK(!drive_config_valid(&c));
    c.commission_duty_max = 1.5f;
    CHECK(!drive_config_valid(&c));
    c.commission_duty_max = 0.15f;
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

    /* Switching axis: L goes to its safe output in that cycle. */
    send(commission_msg(10, 2, -500, 150, echo_now()));
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
    return FW_TEST_RESULT();
}
