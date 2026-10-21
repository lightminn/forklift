#include "fw/app.h"
#include "fw_test.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692
#define BOOT_ID 0x5EED0001u

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
        c.wheel[i] = (wheel_enc_config){.counts_per_rev = 4096, .window = 20, .sign = 1};
    }
    c.wheel_pi = (pi_config){.kp = 0.1f, .ki = 1.0f, .kff = 0.1f, .out_max = 1.0f};
    c.steer_pi = (pi_config){.kp = 2.0f, .out_max = 0.8f};
    c.lift_pi = (pi_config){.kp = 5.0f, .out_max = 0.8f};
    c.steer = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                                .sign = 1,
                                .sensor_per_joint = 1.0f,
                                .min_joint = -0.5f,
                                .max_joint = 0.5f,
                                .invalid_limit = 3};
    c.lift = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
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
        c.overcurrent_a[ch] = 5.0f;
    }
    c.drv_config3 = 0x40;
    c.drv_config4 = 0x04;
    c.ipropi_ohm = 1500.0f;
    c.ipropi_gain = 4750.0f;
    c.adc_vref_v = 3.3f;
    c.adc_full_scale = 65535;
    c.overcurrent_hold_ms = 20;
    return c;
}

/* ---- fake board ---------------------------------------------------------------------- */
typedef struct {
    uint32_t now;
    double wheel_rad_s[2], wheel_counts[2]; /* plant: w' = (K * duty - w) / TAU */
    drive_outputs last_out;
    bool outputs_written;
    uint8_t regs[DRIVE_CH_COUNT][0x0E];
    bool nfault_low[DRIVE_CH_COUNT];
    bool spi_busy, spi_done;
    uint8_t spi_ch;
    uint16_t spi_rx;
    int spi_refusals_left;
    int spi_started;
    int clr_flt_unsafe;
    uint16_t refused_frame;
    int refused_then_started;
    uint8_t to_mcu[4096];
    size_t to_mcu_len, to_mcu_pos;
    link_rx host_rx; /* the host side reading STATE */
    link_state last_state;
    int states;
    bool refuse_every_other;
    int write_calls;
    int kicks;
} board;

static board B;
static const double K = 10.0, TAU = 0.05;

static uint16_t kth_zero(void)
{
    return (uint16_t)(0 | crc4_kth7801(0, KTH_CRC_G704_BYTES));
}

static uint32_t hal_now(void *ctx)
{
    return ((board *)ctx)->now;
}

static void hal_read(void *ctx, drive_inputs *in)
{
    board *b = ctx;
    for (int i = 0; i < 2; i++) {
        in->wheel_hw[i] = (uint16_t)(int64_t)llround(b->wheel_counts[i]);
    }
    in->abs_frame[0] = in->abs_frame[1] = kth_zero();
    in->abs_fresh[0] = in->abs_fresh[1] = true;
    in->adc_fresh = true;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        in->nfault_low[ch] = b->nfault_low[ch];
    }
}

static void hal_write(void *ctx, const drive_outputs *out)
{
    board *b = ctx;
    b->last_out = *out;
    b->outputs_written = true;
}

static size_t hal_link_read(void *ctx, uint8_t *buf, size_t cap)
{
    board *b = ctx;
    size_t n = b->to_mcu_len - b->to_mcu_pos;
    n = n < cap ? n : cap;
    memcpy(buf, b->to_mcu + b->to_mcu_pos, n);
    b->to_mcu_pos += n;
    return n;
}

static bool hal_link_write(void *ctx, const uint8_t *frame, size_t len)
{
    board *b = ctx;
    b->write_calls++;
    if (b->refuse_every_other && (b->write_calls % 2) == 0) {
        return false;
    }
    link_msg m;
    for (size_t i = 0; i < len; i++) {
        if (link_rx_feed(&b->host_rx, frame[i], &m) && m.type == LINK_STATE) {
            b->last_state = m.u.state;
            b->states++;
        }
    }
    return true;
}

static bool hal_spi_start(void *ctx, uint8_t ch, uint16_t tx)
{
    board *b = ctx;
    if (b->spi_refusals_left > 0) {
        b->spi_refusals_left--;
        b->refused_frame = tx;
        return false;
    }
    if (b->refused_frame != 0 && tx == b->refused_frame) {
        b->refused_then_started++;
        b->refused_frame = 0;
    }
    uint8_t addr = (uint8_t)((tx >> 8) & 0x3Fu);
    bool read = (tx & 0x4000u) != 0;
    if (!read && addr < sizeof b->regs[0]) {
        b->regs[ch][addr] = (uint8_t)tx;
        if (addr == DRV8244_REG_COMMAND && (tx & 0x80u)) { /* CLR_FLT */
            if (!b->last_out.drvoff[ch]) {
                b->clr_flt_unsafe++;
            }
            b->nfault_low[ch] = false;
        }
    }
    b->spi_busy = true;
    b->spi_done = true;
    b->spi_ch = ch;
    b->spi_rx = (uint16_t)(0xC000u | ((read && addr < sizeof b->regs[0]) ? b->regs[ch][addr] : 0));
    b->spi_started++;
    return true;
}

static bool hal_spi_take(void *ctx, uint8_t *ch, uint16_t *rx)
{
    board *b = ctx;
    if (!b->spi_done) {
        return false;
    }
    b->spi_done = b->spi_busy = false;
    *ch = b->spi_ch;
    *rx = b->spi_rx;
    return true;
}

static void hal_kick(void *ctx)
{
    ((board *)ctx)->kicks++;
}

static const fw_hal HAL = {
    &B,           hal_now, hal_read, hal_write, hal_link_read, hal_link_write, hal_spi_start,
    hal_spi_take, hal_kick};

static drive_config CFG;
static fw_app app;

static void step_plant(double dt)
{
    for (int i = 0; i < 2; i++) {
        double duty = B.last_out.drvoff[i] ? 0.0 : B.last_out.duty[i];
        B.wheel_rad_s[i] += (K * duty - B.wheel_rad_s[i]) / TAU * dt;
        B.wheel_counts[i] += B.wheel_rad_s[i] * dt * 4096.0 / TWO_PI;
    }
}

static void run_ms(int ms)
{
    for (int i = 0; i < ms; i++) {
        B.now += 1000;
        step_plant(0.001);
        fw_app_tick(&app);
    }
}

static void host_send(link_msg m)
{
    uint8_t wire[LINK_WIRE_MAX];
    size_t n = link_encode(&m, wire, sizeof wire);
    CHECK(n > 0);
    memcpy(B.to_mcu + B.to_mcu_len, wire, n);
    B.to_mcu_len += n;
}

static void start(void)
{
    memset(&B, 0, sizeof B);
    B.now = 0xFFFF0000u; /* the HAL clock wraps during the run */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        B.nfault_low[ch] = true;
    }
    link_rx_init(&B.host_rx);
    CFG = make_config();
    CHECK(fw_app_init(&app, &HAL, &CFG, 10, BOOT_ID));
}

static uint16_t host_seq = 1;

static void host_arm(void)
{
    link_msg m = {.type = LINK_ARM, .seq = host_seq++};
    m.u.arm =
        (link_arm){.boot_id = B.last_state.boot_id, .echo_mcu_time_us = B.last_state.mcu_time_us};
    host_send(m);
}

static void host_drive(float rate)
{
    link_msg m = {.type = LINK_DRIVE, .seq = host_seq++};
    m.u.drive = (link_drive){.left_wheel_rad_s = rate,
                             .right_wheel_rad_s = rate,
                             .valid_after_echo_ms = 150,
                             .flags = LINK_DRIVE_FLAG_LIFT_HOLD,
                             .arm_gen = B.last_state.arm_gen,
                             .boot_id = B.last_state.boot_id,
                             .echo_mcu_time_us = B.last_state.mcu_time_us};
    host_send(m);
}

/* ---- tests -------------------------------------------------------------------------- */
static void boots_through_spi_with_drvoff_first(void)
{
    start();
    run_ms(100);
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_DISARMED);
    CHECK_EQ_I(B.clr_flt_unsafe, 0);
    CHECK(B.states > 0);
    CHECK_EQ_U(B.last_state.mode, DRIVE_MODE_DISARMED);
}

static void refused_spi_start_is_retried_once(void)
{
    start();
    run_ms(5);
    int started = B.spi_started;
    B.spi_refusals_left = 2;
    run_ms(3);
    CHECK_EQ_I(B.refused_then_started, 1); /* the same frame went out on the third try */
    CHECK_EQ_I(B.spi_started, started + 1);
    run_ms(100);
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_DISARMED);
}

static void closed_loop_speed(void)
{
    start();
    run_ms(100);
    host_arm();
    run_ms(12); /* the host waits for a STATE that shows the new arming */
    CHECK_EQ_U(B.last_state.mode, DRIVE_MODE_ARMED);
    host_drive(0.0f);
    run_ms(2);
    for (int k = 0; k < 20; k++) { /* 1 s, a fresh command every 50 ms */
        host_drive(2.0f);
        run_ms(50);
    }
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_ARMED);
    CHECK_NEAR(B.wheel_rad_s[0], 2.0, 0.2);
    CHECK_NEAR(B.last_state.wheel_rate_left_rad_s, 2.0, 0.2);
    /* Commands stop: STOPPING then outputs cut within 200 + 100 ms (+ one STATE period). */
    run_ms(320);
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_DISARMED);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        CHECK(B.last_out.drvoff[ch]);
    }
}

static void state_rate_and_whole_frame_drops(void)
{
    start();
    run_ms(100);
    int before = B.states;
    run_ms(1000);
    CHECK_EQ_I(B.states - before, 100);
    B.refuse_every_other = true;
    before = B.states;
    run_ms(1000);
    CHECK_EQ_I(B.states - before, 50);
    CHECK_EQ_U(B.host_rx.errors, 0); /* a refused frame never goes out half-written */
}

static void rx_errors_reported(void)
{
    start();
    run_ms(50);
    const uint8_t junk[] = {0x55, 0x66, 0x77, 0x00};
    memcpy(B.to_mcu + B.to_mcu_len, junk, sizeof junk);
    B.to_mcu_len += sizeof junk;
    run_ms(20);
    CHECK_EQ_U(B.last_state.rx_errors, 1);
}

static void overruns_and_watchdog(void)
{
    start();
    run_ms(100);
    int kicks = B.kicks;
    run_ms(10);
    CHECK_EQ_I(B.kicks - kicks, 10);
    B.now += 3000; /* one late cycle */
    run_ms(1);
    CHECK_EQ_U(app.cycle_overruns, 1);
    host_arm();
    run_ms(12);
    host_drive(0.0f);
    run_ms(1);
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_ARMED);
    for (int i = 0; i < 3; i++) {
        B.now += 3000;
        run_ms(1);
    }
    CHECK_EQ_U(app.d.mode, DRIVE_MODE_FAULT);
    CHECK(app.d.fault_flags & DRIVE_FAULT_CYCLE_OVERRUN);
}

int main(void)
{
    boots_through_spi_with_drvoff_first();
    refused_spi_start_is_retried_once();
    closed_loop_speed();
    state_rate_and_whole_frame_drops();
    rx_errors_reported();
    overruns_and_watchdog();
    return FW_TEST_RESULT();
}
