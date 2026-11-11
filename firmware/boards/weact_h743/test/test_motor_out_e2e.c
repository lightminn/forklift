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
    bool estop, nfault_l;
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
                           hal_kick};

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
    for (int i = 0; i < (int)cfg.commission_time_ms + 50 && A.d.mode == DRIVE_MODE_ARMED; i++) {
        send(commission(1000));
    }
    CHECK(A.d.fault_flags & DRIVE_FAULT_COMMISSION_TIME_LIMIT);
    ticks((int)cfg.stop_grace_ms + 20);
    CHECK_EQ_U(A.d.mode, DRIVE_MODE_DISARMED);
    CHECK(safe(DRIVE_CH_L));
    CHECK_EQ_U(regmodel_violations(), 0);
}

int main(void)
{
    expiry_stops();
    disarm_stops_in_its_cycle();
    nfault_cuts_the_channel();
    estop_cuts_everything();
    commission_limits();
    return FW_TEST_RESULT();
}
