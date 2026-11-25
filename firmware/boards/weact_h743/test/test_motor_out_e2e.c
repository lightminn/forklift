/* The firmware core (fw_app) driving the real motor_out_tim1.c on the host register model, with
   the shared vehicle model (firmware/plant) as the inputs and the DRV8244 SPI (motor output design
   4, 4.2): each cut path ends in the channel's configured safe output, read back from the
   registers through RM0433 and DRV8244 table 8-4. The board's own configuration, with the outputs
   open (and, for the drive scenarios, an e-stop input wired). */
#include <math.h>
#include <string.h>

#include "board/board.h"
#include "board/board_config.h"
#include "board/board_hal.h"
#include "fw/app.h"
#include "fw/link.h"
#include "fw_test.h"
#include "plant.h"
#include "regmodel.h"

typedef struct {
    plant p;
    uint32_t now;
    drive_outputs out;
    uint8_t rx[1024];
    size_t rx_n, rx_off;
    bool estop, nfault_l, no_kth;
} world;

static world W;
static fw_app A;
static drive_config cfg;
static uint16_t seq;

static uint32_t hal_now(void *c)
{
    (void)c;
    return W.now;
}

static void hal_read(void *c, drive_inputs *in)
{
    (void)c;
    plant_inputs(&W.p, in);
    in->estop_active = W.estop;
    if (W.nfault_l) {
        in->nfault_low[DRIVE_CH_L] = true;
    }
    if (W.no_kth) { /* no KTH fitted: the SPI reads all zeros, CRC fails (2026-11-24 record) */
        in->abs_frame[0] = in->abs_frame[1] = 0xFFFFu;
    }
}

static void hal_write(void *c, const drive_outputs *out)
{
    (void)c;
    W.out = *out;
    motor_out_apply(out);
}

static size_t hal_link_read(void *c, uint8_t *buf, size_t cap)
{
    (void)c;
    size_t n = W.rx_n - W.rx_off < cap ? W.rx_n - W.rx_off : cap;
    memcpy(buf, W.rx + W.rx_off, n);
    W.rx_off += n;
    return n;
}

static bool hal_link_write(void *c, const uint8_t *f, size_t len)
{
    (void)c;
    (void)f;
    (void)len;
    return true;
}

static bool hal_spi_start(void *c, uint8_t ch, uint16_t tx, uint32_t tag)
{
    (void)c;
    return plant_spi_start(&W.p, ch, tx, tag);
}

static bool hal_spi_take(void *c, uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    (void)c;
    return plant_spi_take(&W.p, ch, rx, tag);
}

static void hal_kick(void *c)
{
    (void)c;
}

static const fw_hal HAL = {0,
                           hal_now,
                           hal_read,
                           hal_write,
                           hal_link_read,
                           hal_link_write,
                           hal_spi_start,
                           hal_spi_take,
                           hal_kick,
                           0};

static void tick(void)
{
    W.now += 1000u;
    fw_app_tick(&A);
    regmodel_uev(); /* the update events of the next millisecond */
    plant_step(&W.p, &W.out, 1e-3);
}

static void ticks(int n)
{
    for (int i = 0; i < n; i++) {
        tick();
    }
}

static void send(link_msg m)
{
    m.seq = ++seq;
    W.rx_n = W.rx_off = 0;
    W.rx_n = link_encode(&m, W.rx, sizeof W.rx);
    CHECK(W.rx_n > 0);
    tick();
}

static void boot(void (*mod)(drive_config *))
{
    memset(&W, 0, sizeof W);
    seq = 0;
    cfg = *board_drive_config();
    cfg.outputs_locked = false; /* an output build (design 2) */
    mod(&cfg);
    CHECK(drive_config_valid(&cfg));
    regmodel_reset();
    motor_out_safe_init();
    motor_out_wake_drivers();
    plant_init(&W.p, &cfg);
    W.now = 1000u;
    CHECK(fw_app_init(&A, &HAL, &cfg, 10, 0x1234u, DRIVE_RESET_POWER));
    for (int i = 0; i < 500 && A.d.mode != DRIVE_MODE_DISARMED; i++) {
        tick();
    }
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    for (int ch = 0; ch < 4; ch++) {
        regmodel_allow(ch, RM_ALLOW_FWD | RM_ALLOW_REV); /* the core picks the direction */
    }
}

static void mod_drive(drive_config *c)
{
    c->estop_wired = true; /* the drive scenarios have an e-stop input (Q3) */
}

static void mod_commission(drive_config *c)
{
    c->commission_build = true;
    /* This E2E compiles the normal board build and flips the bit at run time: set the low
       commissioning limits it tests explicitly (design 2026-11-25 5, Codex S P3). */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c->commission_duty_max_e4[ch] = 1500;
        c->commission_time_ms[ch] = 2000;
    }
}

static link_msg arm(void)
{
    link_msg m = {.type = LINK_ARM};
    m.u.arm = (link_arm){.boot_id = A.d.boot_id, .echo_mcu_time_us = A.d.time_us};
    return m;
}

static link_msg drive_cmd(float wheel, uint16_t valid_ms)
{
    link_msg m = {.type = LINK_DRIVE};
    m.u.drive = (link_drive){.left_wheel_rad_s = wheel,
                             .right_wheel_rad_s = wheel,
                             .steering_rad = 0.0f,
                             .lift_m = (float)W.p.lift_m,
                             .valid_after_echo_ms = valid_ms,
                             .flags = LINK_DRIVE_FLAG_LIFT_HOLD,
                             .arm_gen = A.d.arm_gen,
                             .boot_id = A.d.boot_id,
                             .echo_mcu_time_us = A.d.time_us};
    return m;
}

static link_msg simple(uint8_t type)
{
    link_msg m = {.type = type};
    m.u.clear.boot_id = A.d.boot_id;
    return m;
}

static bool driving(int ch)
{
    rm_state s = regmodel_channel(ch).state;
    return s == RM_FWD || s == RM_REV;
}

/* The channel's configured safe output (core design 4): cut (STANDBY) or electric brake. */
static bool safe(int ch)
{
    rm_state s = regmodel_channel(ch).state;
    return cfg.safe_brake[ch] ? (s == RM_BRAKE || s == RM_STANDBY) : s == RM_STANDBY;
}

/* Armed and both wheels driven by the core's speed loop. */
static void armed_and_rolling(void)
{
    boot(mod_drive);
    send(arm());
    send(drive_cmd(0.0f, 150));
    for (int i = 0; i < 100; i++) {
        send(drive_cmd(2.0f, 150));
    }
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_ARMED);
    CHECK(driving(DRIVE_CH_L) && driving(DRIVE_CH_R));
}

static void expiry_stops(void)
{
    armed_and_rolling();
    ticks(150 + (int)cfg.stop_grace_ms + 20); /* no more commands */
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    for (int ch = 0; ch < 4; ch++) {
        CHECK(safe(ch));
    }
    CHECK_EQ_U(regmodel_violations(), 0);
}

static void disarm_stops_in_its_cycle(void)
{
    armed_and_rolling();
    send(simple(LINK_DISARM));
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    CHECK(safe(DRIVE_CH_L) && safe(DRIVE_CH_R));
    CHECK_EQ_U(regmodel_violations(), 0);
}

static void nfault_cuts_the_channel(void)
{
    armed_and_rolling();
    W.nfault_l = true;
    send(drive_cmd(2.0f, 150));
    tick();
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_L).state, RM_STANDBY); /* its own driver fault: cut */
    CHECK(!driving(DRIVE_CH_R));
    CHECK_EQ_U(regmodel_violations(), 0);
}

static void estop_cuts_everything(void)
{
    armed_and_rolling();
    W.estop = true;
    send(drive_cmd(2.0f, 150));
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_ESTOP);
    for (int ch = 0; ch < 4; ch++) {
        CHECK(!driving(ch));
    }
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* Commissioning with the outputs open: the duty reaches the compare register as sent, a duty over
   the limit is refused with the previous one kept (Codex P3-7), and the time limit stops it. */
static link_msg commission(int16_t duty_e4)
{
    link_msg m = {.type = LINK_COMMISSION};
    m.u.commission = (link_commission){.axis = 0,
                                       .duty_e4 = duty_e4,
                                       .valid_after_echo_ms = 150,
                                       .arm_gen = A.d.arm_gen,
                                       .boot_id = A.d.boot_id,
                                       .echo_mcu_time_us = A.d.time_us};
    return m;
}

static void commission_limits(void)
{
    boot(mod_commission);
    send(arm());
    send(commission(0));
    for (int i = 0; i < 80; i++) { /* the wheel duty slew (2 /s) reaches 0.10 in 50 cycles */
        send(commission(1000));
    }
    tick();
    rm_channel c = regmodel_channel(DRIVE_CH_L);
    CHECK(c.state == RM_FWD && fabs(c.duty - 0.10) < 1e-6);
    send(commission(2000)); /* over 0.15: refused, 0.10 kept */
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_BAD_VALUE);
    tick();
    c = regmodel_channel(DRIVE_CH_L);
    CHECK(c.state == RM_FWD && fabs(c.duty - 0.10) < 1e-6);
    for (int i = 0; i < (int)cfg.commission_time_ms[DRIVE_CH_L] + 50 && A.d.mode == DRIVE_MODE_ARMED; i++) {
        send(commission(1000));
    }
    CHECK(A.d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    ticks((int)cfg.stop_grace_ms + 20);
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    CHECK(safe(DRIVE_CH_L));
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* ---- single-axis commissioning (partial-parts plan 2026-11-25 PR-B, fw-single) ---------- */

static void mod_single(drive_config *c)
{
    mod_commission(c);
    c->commission_single_axis = true;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c->safe_brake[ch] = true; /* the brake path is the one that keeps a DRVOFF low */
    }
}

static link_msg cm(int axis, int16_t duty_e4)
{
    link_msg m = commission(duty_e4);
    m.u.commission.axis = (uint8_t)axis;
    return m;
}

/* Several frames read in one tick (a DISARM-ARM-COMMISSION burst). */
static void send_burst(link_msg *ms, int n)
{
    W.rx_n = W.rx_off = 0;
    for (int i = 0; i < n; i++) {
        ms[i].seq = ++seq;
        size_t k = link_encode(&ms[i], W.rx + W.rx_n, sizeof W.rx - W.rx_n);
        CHECK(k > 0);
        W.rx_n += k;
    }
    tick();
}

static bool cut(int ch)
{
    return regmodel_channel(ch).state == RM_STANDBY;
}

static void only_cut_except(int keep)
{
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (ch != keep) {
            CHECK(cut(ch));
        }
    }
}

static void single_axis_fixes_the_axis_and_keeps_the_rest_cut(void)
{
    boot(mod_single);
    only_cut_except(-1); /* no axis yet: all four cut, though every channel would brake */
    send(arm());
    only_cut_except(-1);
    send(cm(DRIVE_CH_H, 0)); /* the first zero COMMISSION fixes H */
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_H).state, RM_BRAKE);
    only_cut_except(DRIVE_CH_H);
    send(cm(DRIVE_CH_M, 0)); /* another axis, zero or not: refused 11, H kept */
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(A.d.cm_axis, DRIVE_CH_H);
    send(cm(DRIVE_CH_H, 1000));
    tick();
    CHECK(driving(DRIVE_CH_H));
    send(cm(DRIVE_CH_L, 1000));
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_STOPPING);
    tick();
    CHECK(driving(DRIVE_CH_H));
    only_cut_except(DRIVE_CH_H);
    send(simple(LINK_DISARM)); /* H keeps its safe output, the others stay cut */
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_H).state, RM_BRAKE);
    only_cut_except(DRIVE_CH_H);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* H brakes; DISARM, ARM and M(0) in one tick: M is refused 11 until a tick has applied all four
   DRVOFF high, and in the pin log H's DRVOFF goes high before M's goes low. */
static void another_axis_waits_for_an_all_high_tick(void)
{
    boot(mod_single);
    send(arm());
    send(cm(DRIVE_CH_H, 0));
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_H).state, RM_BRAKE);
    regmodel_log_clear();
    link_msg burst[3] = {simple(LINK_DISARM), arm(), cm(DRIVE_CH_M, 0)};
    burst[2].u.commission.arm_gen = (uint16_t)(A.d.arm_gen + 1u); /* the new arming's */
    send_burst(burst, 3);
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_ARMED);
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_STOPPING);
    CHECK_EQ_U(A.d.cm_axis, 0xFFu); /* none yet */
    only_cut_except(-1); /* the tick applied all four high */
    send(cm(DRIVE_CH_M, 0));
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_M).state, RM_BRAKE);
    only_cut_except(DRIVE_CH_M);
    const rm_event *log;
    int n = regmodel_log(&log), h_high = -1, m_low = -1;
    for (int i = 0; i < n; i++) {
        if (log[i].kind == RM_EV_DRVOFF_HIGH && log[i].ch == DRIVE_CH_H && h_high < 0) {
            h_high = i;
        }
        if (log[i].kind == RM_EV_DRVOFF_LOW && log[i].ch == DRIVE_CH_M && m_low < 0) {
            m_low = i;
        }
    }
    CHECK(h_high >= 0 && m_low > h_high);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* The same axis re-armed needs no all-high tick (it is the one braking). */
static void the_same_axis_rearms_at_once(void)
{
    boot(mod_single);
    send(arm());
    send(cm(DRIVE_CH_H, 0));
    link_msg burst[3] = {simple(LINK_DISARM), arm(), cm(DRIVE_CH_H, 0)};
    burst[2].u.commission.arm_gen = (uint16_t)(A.d.arm_gen + 1u);
    send_burst(burst, 3);
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_NONE);
    CHECK_EQ_U(A.d.cm_axis, DRIVE_CH_H);
    only_cut_except(DRIVE_CH_H);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* An expiry's STOPPING brakes the fixed axis only; the time cap still holds on M; M drives with
   no KTH fitted (its bits in STATE, no FAULT). */
static void single_axis_stopping_time_cap_and_no_kth(void)
{
    boot(mod_single);
    W.no_kth = true;
    ticks(20);
    CHECK(A.d.fault_flags & DRIVE_FAULT_STEER_SENSOR_INVALID);
    send(arm());
    send(cm(DRIVE_CH_M, 0));
    CHECK_EQ_U(A.d.reject_reason, DRIVE_REJECT_NONE);
    send(cm(DRIVE_CH_M, 1000));
    tick();
    CHECK(driving(DRIVE_CH_M));
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_ARMED);
    ticks(160); /* no more commands: expiry, STOPPING */
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_STOPPING);
    CHECK_EQ_U(regmodel_channel(DRIVE_CH_M).state, RM_BRAKE);
    only_cut_except(DRIVE_CH_M);
    ticks((int)cfg.stop_grace_ms + 20);
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    only_cut_except(DRIVE_CH_M);
    /* the 2 s cap on M */
    send(arm());
    send(cm(DRIVE_CH_M, 0));
    for (int i = 0; i < (int)cfg.commission_time_ms[DRIVE_CH_M] + 50 && A.d.mode == DRIVE_MODE_ARMED; i++) {
        send(cm(DRIVE_CH_M, 1000));
    }
    CHECK(A.d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    only_cut_except(DRIVE_CH_M);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* A driver fault on the fixed axis latches the board; the others stay cut, the axis cuts. */
static void single_axis_fault_cuts(void)
{
    boot(mod_single);
    send(arm());
    send(cm(DRIVE_CH_L, 0));
    for (int i = 0; i < 60; i++) {
        send(cm(DRIVE_CH_L, 1000));
    }
    CHECK(driving(DRIVE_CH_L));
    W.nfault_l = true;
    send(cm(DRIVE_CH_L, 1000));
    tick();
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_FAULT);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK(cut(ch));
    }
    CHECK_EQ_U(regmodel_violations(), 0);
}

int main(void)
{
    expiry_stops();
    disarm_stops_in_its_cycle();
    nfault_cuts_the_channel();
    estop_cuts_everything();
    commission_limits();
    single_axis_fixes_the_axis_and_keeps_the_rest_cut();
    another_axis_waits_for_an_all_high_tick();
    the_same_axis_rearms_at_once();
    single_axis_stopping_time_cap_and_no_kth();
    single_axis_fault_cuts();
    return FW_TEST_RESULT();
}
