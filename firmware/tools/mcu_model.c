/* MCU model endpoint (sim-to-real plan P0-7): the firmware core (fw_app) on a fake board, in real
   time at 1 kHz, speaking mcu-link/1 over a tty (a pty slave in the tests). Host-only: it drives no
   hardware. Vehicle values come from the generated vehicle_config.h (tools/real_vehicle_header.py);
   every other drive_config value is synthetic, not a hardware value.

   usage: mcu_model --link PATH [--boot-id N] [--duration-ms N] [--fault-at-ms N] [--estop-at-ms N]
                    [--estop-release-ms N] [--silent-at-ms N]
   --fault-at-ms: nFAULT L goes low then and stays low until the core sends CLR_FLT.
   --silent-at-ms: from then the model neither reads nor writes the link (host silence detection
   only; it is not an IWDG and proves nothing about output cut-off). */
#define _DEFAULT_SOURCE /* cfmakeraw, clock_nanosleep */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "fw/app.h"
#include "vehicle_config.h"

#define TWO_PI 6.28318530717958647692
#define TX_CAP 4096u

typedef struct {
    int fd;
    uint64_t t0_ns, now_ns;
    double wheel_rad_s[2], wheel_counts[2];
    double steer_rad, lift_m;
    drive_outputs out;
    uint8_t regs[DRIVE_CH_COUNT][0x0E];
    bool nfault_low[DRIVE_CH_COUNT];
    bool spi_done;
    uint8_t spi_ch;
    uint32_t spi_tag;
    uint16_t spi_rx;
    uint8_t tx[TX_CAP];
    size_t tx_len;
    long fault_at_ms, estop_at_ms, estop_release_ms, silent_at_ms;
    bool fault_fired;
} board;

static board B;
static volatile sig_atomic_t stop_requested;

static void on_term(int sig)
{
    (void)sig;
    stop_requested = 1;
}

static uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
}

static long elapsed_ms(void)
{
    return (long)((B.now_ns - B.t0_ns) / 1000000u);
}

static bool silent(void)
{
    return B.silent_at_ms >= 0 && elapsed_ms() >= B.silent_at_ms;
}

static uint16_t kth_frame(double joint, double sensor_per_joint)
{
    double a = fmod(joint * sensor_per_joint, TWO_PI);
    if (a < 0) {
        a += TWO_PI;
    }
    uint16_t angle = (uint16_t)((uint32_t)lround(a * 4096.0 / TWO_PI) & 0x0FFFu);
    return (uint16_t)((angle << 4) | crc4_kth7801(angle, KTH_CRC_G704_BYTES));
}

/* ---- HAL ---------------------------------------------------------------------------- */
static uint32_t hal_now(void *ctx)
{
    (void)ctx;
    return (uint32_t)((B.now_ns - B.t0_ns) / 1000u);
}

static void hal_read(void *ctx, drive_inputs *in)
{
    (void)ctx;
    for (int i = 0; i < 2; i++) {
        in->wheel_hw[i] = (uint16_t)(int64_t)llround(B.wheel_counts[i]);
    }
    in->abs_frame[0] = kth_frame(B.steer_rad, 1.0);
    in->abs_frame[1] = kth_frame(B.lift_m, 10.0);
    in->abs_fresh[0] = in->abs_fresh[1] = true;
    in->adc_fresh = true;
    if (B.fault_at_ms >= 0 && !B.fault_fired && elapsed_ms() >= B.fault_at_ms) {
        B.fault_fired = true;
        B.nfault_low[DRIVE_CH_L] = true;
    }
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        in->nfault_low[ch] = B.nfault_low[ch];
    }
    long t = elapsed_ms();
    in->estop_active = B.estop_at_ms >= 0 && t >= B.estop_at_ms &&
                       (B.estop_release_ms < 0 || t < B.estop_release_ms);
}

static void hal_write(void *ctx, const drive_outputs *out)
{
    (void)ctx;
    B.out = *out;
}

static size_t hal_link_read(void *ctx, uint8_t *buf, size_t cap)
{
    (void)ctx;
    if (silent()) {
        return 0;
    }
    ssize_t n = read(B.fd, buf, cap);
    return n > 0 ? (size_t)n : 0;
}

static bool hal_link_write(void *ctx, const uint8_t *frame, size_t len)
{
    (void)ctx;
    if (silent()) {
        return true; /* swallowed: the host sees silence */
    }
    if (B.tx_len + len > TX_CAP) {
        return false; /* whole frame or nothing */
    }
    memcpy(B.tx + B.tx_len, frame, len);
    B.tx_len += len;
    return true;
}

static void flush_tx(void)
{
    while (B.tx_len > 0) {
        ssize_t n = write(B.fd, B.tx, B.tx_len);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return; /* EAGAIN or a closed peer: keep the bytes, try next tick */
        }
        memmove(B.tx, B.tx + n, B.tx_len - (size_t)n);
        B.tx_len -= (size_t)n;
    }
}

static bool hal_spi_start(void *ctx, uint8_t ch, uint16_t tx, uint32_t tag)
{
    (void)ctx;
    uint8_t addr = (uint8_t)((tx >> 8) & 0x3Fu);
    bool rd = (tx & 0x4000u) != 0;
    if (!rd && addr < sizeof B.regs[0]) {
        B.regs[ch][addr] = (uint8_t)tx;
        if (addr == DRV8244_REG_COMMAND && (tx & 0x80u)) { /* CLR_FLT */
            B.nfault_low[ch] = false;
        }
    }
    B.spi_done = true;
    B.spi_ch = ch;
    B.spi_tag = tag;
    B.spi_rx = (uint16_t)(0xC000u | ((rd && addr < sizeof B.regs[0]) ? B.regs[ch][addr] : 0));
    return true;
}

static bool hal_spi_take(void *ctx, uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    (void)ctx;
    if (!B.spi_done) {
        return false;
    }
    B.spi_done = false;
    *ch = B.spi_ch;
    *rx = B.spi_rx;
    *tag = B.spi_tag;
    return true;
}

static void hal_kick(void *ctx)
{
    (void)ctx;
}

static const fw_hal HAL = {
    &B,           hal_now, hal_read, hal_write, hal_link_read, hal_link_write, hal_spi_start,
    hal_spi_take, hal_kick};

/* ---- plant: first-order wheels, integrating steering and lift ------------------------- */
static const double WHEEL_TAU_S = 0.05;
static const double STEER_RAD_S_PER_DUTY = 1.0;
static const double LIFT_M_S_PER_DUTY = 0.1;

static void step_plant(double dt)
{
    const double k = (double)FW_MAX_WHEEL_RAD_S * 1.25; /* full duty reaches 1.25 x the limit */
    const double cpr[2] = {(double)FW_WHEEL_COUNTS_PER_REV_L, (double)FW_WHEEL_COUNTS_PER_REV_R};
    for (int i = 0; i < 2; i++) {
        double duty = B.out.drvoff[i] ? 0.0 : (double)B.out.duty[i];
        B.wheel_rad_s[i] += (k * duty - B.wheel_rad_s[i]) / WHEEL_TAU_S * dt;
        B.wheel_counts[i] += B.wheel_rad_s[i] * dt * cpr[i] / TWO_PI;
    }
    double ds = B.out.drvoff[DRIVE_CH_M] ? 0.0 : (double)B.out.duty[DRIVE_CH_M];
    double dl = B.out.drvoff[DRIVE_CH_H] ? 0.0 : (double)B.out.duty[DRIVE_CH_H];
    B.steer_rad += ds * STEER_RAD_S_PER_DUTY * dt;
    B.lift_m += dl * LIFT_M_S_PER_DUTY * dt;
}

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
    c.overrun_limit = 50; /* a loaded host must not fault the model on scheduling jitter */
    c.adc_invalid_limit = 5;
    c.wheel[0] =
        (wheel_enc_config){.counts_per_rev = FW_WHEEL_COUNTS_PER_REV_L, .window = 5, .sign = 1};
    c.wheel[1] =
        (wheel_enc_config){.counts_per_rev = FW_WHEEL_COUNTS_PER_REV_R, .window = 5, .sign = 1};
    c.wheel_pi = (pi_config){
        .kp = 0.05f, .ki = 0.5f, .kff = 1.0f / (FW_MAX_WHEEL_RAD_S * 1.25f), .out_max = 1.0f};
    c.steer_pi = (pi_config){.kp = 4.0f, .out_max = 0.8f};
    c.lift_pi = (pi_config){.kp = 20.0f, .out_max = 0.8f};
    c.steer = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                                .sign = 1,
                                .sensor_per_joint = 1.0f,
                                .min_joint = FW_STEER_MIN_RAD,
                                .max_joint = FW_STEER_MAX_RAD,
                                .invalid_limit = 3};
    c.lift = (abs_axis_config){.crc = KTH_CRC_G704_BYTES,
                               .sign = 1,
                               .sensor_per_joint = 10.0f,
                               .min_joint = -0.01f,
                               .max_joint = 0.3f,
                               .invalid_limit = 3};
    c.steer_rate_max = 1.0f;
    c.lift_rate_max = 0.1f;
    c.max_wheel_rad_s = FW_MAX_WHEEL_RAD_S;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        c.duty_sign[ch] = 1;
        c.overcurrent_a[ch] = 5.0f;
    }
    c.drv_config3 = 0x40;
    c.drv_config4 = 0x04;
    c.ipropi_ohm = 1000.0f;
    c.ipropi_gain = 4750.0f;
    c.adc_vref_v = 3.3f;
    c.adc_full_scale = 65535;
    c.overcurrent_hold_ms = 20;
    /* mcu-link/2: a normal build with an e-stop input (the model's --estop-at-ms); no slew in
       effect and encoder limits loose enough for the model's own motion. The read-gap limit is
       the longest the 16-bit step stays unambiguous at the model's top speed (design 5). */
    c.estop_wired = true;
    c.wheel_duty_slew_per_s = 1000.0f;
    c.enc_phys_max_rad_s = FW_MAX_WHEEL_RAD_S * 1.25f;
    double cpr = FW_WHEEL_COUNTS_PER_REV_L > FW_WHEEL_COUNTS_PER_REV_R ? FW_WHEEL_COUNTS_PER_REV_L
                                                                       : FW_WHEEL_COUNTS_PER_REV_R;
    c.enc_gap_max_us = (uint32_t)(16000.0 * TWO_PI / ((double)c.enc_phys_max_rad_s * cpr) * 1e6);
    c.enc_jump_rad_s = c.enc_phys_max_rad_s * 1.5f;
    c.enc_stuck_duty = 0.95f;
    c.enc_stuck_ms = 1000;
    c.enc_stuck_counts = 1;
    c.enc_polarity_duty = 0.95f;
    c.enc_polarity_ms = 200;
    c.enc_polarity_rise_rad_s = 5.0f;
    c.enc_polarity_persist_ms = 2000;
    return c;
}

static long arg_long(int argc, char **argv, const char *name, long dflt)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], name) == 0) {
            return strtol(argv[i + 1], NULL, 0);
        }
    }
    return dflt;
}

static const char *arg_str(int argc, char **argv, const char *name)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], name) == 0) {
            return argv[i + 1];
        }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    const char *link = arg_str(argc, argv, "--link");
    if (link == NULL) {
        fprintf(stderr, "usage: mcu_model --link PATH [options]\n");
        return 2;
    }
    memset(&B, 0, sizeof B);
    B.fd = open(link, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (B.fd < 0) {
        perror("open link");
        return 2;
    }
    struct termios tio;
    if (tcgetattr(B.fd, &tio) == 0) {
        cfmakeraw(&tio);
        tcsetattr(B.fd, TCSANOW, &tio);
    }
    B.fault_at_ms = arg_long(argc, argv, "--fault-at-ms", -1);
    B.estop_at_ms = arg_long(argc, argv, "--estop-at-ms", -1);
    B.estop_release_ms = arg_long(argc, argv, "--estop-release-ms", -1);
    B.silent_at_ms = arg_long(argc, argv, "--silent-at-ms", -1);
    long duration_ms = arg_long(argc, argv, "--duration-ms", -1);
    uint32_t boot_id = (uint32_t)arg_long(argc, argv, "--boot-id", 0x0B00700L);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        B.nfault_low[ch] = true; /* drivers wake with a fault latched until CLR_FLT */
    }
    signal(SIGTERM, on_term);
    signal(SIGINT, on_term);
    signal(SIGPIPE, SIG_IGN);

    static drive_config cfg;
    static fw_app app;
    cfg = make_config();
    B.t0_ns = B.now_ns = mono_ns();
    if (!fw_app_init(&app, &HAL, &cfg, 10, boot_id, DRIVE_RESET_POWER)) {
        fprintf(stderr, "fw_app_init refused the configuration\n");
        return 2;
    }
    uint64_t next = B.now_ns;
    while (!stop_requested && (duration_ms < 0 || elapsed_ms() < duration_ms)) {
        next += 1000000u;
        struct timespec ts = {.tv_sec = (time_t)(next / 1000000000u),
                              .tv_nsec = (long)(next % 1000000000u)};
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR &&
               !stop_requested) {
        }
        uint64_t prev = B.now_ns;
        B.now_ns = mono_ns();
        step_plant((double)(B.now_ns - prev) / 1e9);
        fw_app_tick(&app);
        if (!silent()) {
            flush_tx();
        }
    }
    close(B.fd);
    return 0;
}
