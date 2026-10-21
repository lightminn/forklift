#include "fw/drive.h"

#include <math.h>
#include <string.h>

#define US_PER_MS 1000u

static bool finite_pos(float x)
{
    return isfinite(x) && x > 0.0f;
}

static bool pi_valid(const pi_config *p)
{
    return isfinite(p->kp) && p->kp >= 0.0f && isfinite(p->ki) && p->ki >= 0.0f &&
           isfinite(p->kff) && p->kff >= 0.0f && finite_pos(p->out_max) && p->out_max <= 1.0f &&
           isfinite(p->deadband) && p->deadband >= 0.0f;
}

bool drive_config_valid(const drive_config *c)
{
    const uint32_t max_us = 0x7FFFFFFFu;
    const uint32_t max_ms = max_us / US_PER_MS;
    if (c->period_us == 0 || c->period_us > max_us / 2u || c->command_timeout_ms == 0 ||
        c->command_timeout_ms > 0xFFFFu || c->stop_grace_ms > max_ms || c->driver_retry_ms == 0 ||
        c->driver_retry_ms > max_ms || c->driver_wake_us > max_us ||
        c->spi_timeout_us < 2u * c->period_us || c->spi_timeout_us > max_us ||
        c->status_poll_ms == 0 || c->status_poll_ms > max_ms || c->overrun_limit == 0 ||
        c->adc_invalid_limit == 0 || c->overcurrent_hold_ms == 0 ||
        c->overcurrent_hold_ms > max_ms) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        const wheel_enc_config *w = &c->wheel[i];
        if (w->counts_per_rev == 0 || w->window == 0 || w->window > WHEEL_ENC_WINDOW_MAX ||
            (w->sign != 1 && w->sign != -1)) {
            return false;
        }
    }
    if (!pi_valid(&c->wheel_pi) || !pi_valid(&c->steer_pi) || !pi_valid(&c->lift_pi) ||
        !abs_axis_config_valid(&c->steer) || !abs_axis_config_valid(&c->lift) ||
        !finite_pos(c->steer_rate_max) || !finite_pos(c->lift_rate_max) ||
        !finite_pos(c->max_wheel_rad_s) || !drv8244_config3_valid(c->drv_config3) ||
        !finite_pos(c->ipropi_ohm) || !finite_pos(c->ipropi_gain) || !finite_pos(c->adc_vref_v) ||
        c->adc_full_scale == 0) {
        return false;
    }
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if ((c->duty_sign[ch] != 1 && c->duty_sign[ch] != -1) ||
            !finite_pos(c->overcurrent_a[ch])) {
            return false;
        }
    }
    return true;
}

/* ---- faults ------------------------------------------------------------------------- */

static void enter_fault(drive *d, uint32_t bits)
{
    d->latched |= bits;
    if (d->mode != DRIVE_MODE_ESTOP) {
        d->mode = DRIVE_MODE_FAULT;
    }
}

/* ---- driver initialisation and SPI (one transaction at a time) --------------------- */

static void queue_frame(drive *d, uint8_t ch, uint16_t frame)
{
    d->spi_pending = true;
    d->spi_ch = ch;
    d->spi_frame = frame;
    d->spi_since_us = d->time_us;
}

static void start_driver_init(drive *d, bool after_clear)
{
    d->drivers_ready = false;
    d->init_after_clear = after_clear;
    d->retry_waiting = false;
    d->init_ch = 0;
    d->init_sdo = 0;
    d->spi_pending = false;
    d->spi_inflight = false; /* its late answer carries an old tag and is dropped */
    drv8244_init_start(&d->ini, (uint32_t)d->time_us, d->cfg->driver_wake_us);
}

static void driver_init_failed(drive *d)
{
    if (d->init_after_clear) {
        /* The clear did not bring the drivers back: latch again (design 4). */
        enter_fault(d, DRIVE_FAULT_DRIVER_NOT_READY);
        d->init_after_clear = false;
    }
    d->spi_pending = false;
    d->spi_inflight = false;
    d->retry_waiting = true;
    d->retry_at_us = d->time_us + (uint64_t)d->cfg->driver_retry_ms * US_PER_MS;
}

static void step_driver_init(drive *d, const drive_inputs *in)
{
    for (;;) {
        if (d->drivers_ready || d->spi_inflight || d->spi_pending) {
            return;
        }
        if (d->retry_waiting) {
            if (d->time_us < d->retry_at_us) {
                return;
            }
            start_driver_init(d, false);
        }
        uint16_t frame;
        if (drv8244_init_next(&d->ini, (uint32_t)d->time_us, d->init_sdo, d->cfg->drv_config3,
                              d->cfg->drv_config4, &frame)) {
            queue_frame(d, d->init_ch, frame);
            return;
        }
        if (d->ini.failed) {
            driver_init_failed(d);
            return;
        }
        if (!d->ini.done) {
            return; /* waiting for tREADY */
        }
        if (d->init_ch + 1 < DRIVE_CH_COUNT) {
            d->init_ch++;
            d->init_sdo = 0;
            drv8244_init_start(&d->ini, (uint32_t)d->time_us, 0);
            continue;
        }
        break;
    }
    /* All four configured; CLR_FLT must have released every nFAULT. */
    uint32_t stuck = 0;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (in->nfault_low[ch]) {
            stuck |= DRIVE_FAULT_DRIVER(ch);
        }
    }
    if (stuck) {
        if (d->init_after_clear) {
            d->latched |= stuck;
        }
        driver_init_failed(d);
        return;
    }
    d->drivers_ready = true;
    d->init_after_clear = false;
    d->poll_ch = 0;
    d->next_poll_us = d->time_us + (uint64_t)d->cfg->status_poll_ms * US_PER_MS;
    if (d->mode == DRIVE_MODE_BOOT) {
        d->mode = DRIVE_MODE_DISARMED;
    }
}

static void step_status_poll(drive *d)
{
    if (!d->drivers_ready || d->spi_inflight || d->spi_pending || d->time_us < d->next_poll_us) {
        return;
    }
    queue_frame(d, d->poll_ch, drv8244_read_frame(DRV8244_REG_FAULT_SUMMARY));
    d->poll_ch = (uint8_t)((d->poll_ch + 1) % DRIVE_CH_COUNT);
    /* Each channel is read once per status_poll_ms. */
    d->next_poll_us = d->time_us + (uint64_t)d->cfg->status_poll_ms * US_PER_MS / DRIVE_CH_COUNT;
}

/* Lost-transaction detection: the app hands responses in at the head of the cycle, so a
   transaction (or a start the HAL keeps refusing) older than spi_timeout_us is lost. */
static void check_spi_timeout(drive *d)
{
    if (!(d->spi_pending || d->spi_inflight) ||
        d->time_us - d->spi_since_us < d->cfg->spi_timeout_us) {
        return;
    }
    uint8_t ch = d->spi_ch;
    if (!d->drivers_ready) {
        driver_init_failed(d);
    } else {
        d->spi_pending = false;
        d->spi_inflight = false;
        enter_fault(d, DRIVE_FAULT_DRIVER(ch));
    }
}

bool drive_spi_next(drive *d, uint8_t *ch, uint16_t *frame, uint32_t *tag)
{
    if (!d->spi_pending || d->spi_inflight) {
        return false;
    }
    *ch = d->spi_ch;
    *frame = d->spi_frame;
    *tag = d->spi_tag + 1u; /* the tag it gets if it starts */
    return true;
}

void drive_spi_started(drive *d)
{
    /* Tags advance only on transactions that really start: while an old one keeps the bus
       busy the HAL refuses new starts, so its tag cannot come round again. */
    d->spi_tag++;
    d->spi_pending = false;
    d->spi_inflight = true; /* spi_since_us keeps counting from when the frame was queued */
}

void drive_spi_result(drive *d, uint8_t ch, uint16_t sdo, uint32_t tag)
{
    if (!d->spi_inflight || tag != d->spi_tag || ch != d->spi_ch || ch >= DRIVE_CH_COUNT) {
        return; /* an answer to a transaction that is no longer current */
    }
    d->spi_inflight = false;
    bool valid = drv8244_response_valid(sdo);
    d->driver_status[ch] = valid ? drv8244_status_byte(sdo) : 0;
    if (!d->drivers_ready) {
        d->init_sdo = sdo;
    } else if (!valid) {
        enter_fault(d, DRIVE_FAULT_DRIVER(ch));
    }
}

/* ---- life cycle --------------------------------------------------------------------- */

static void reset_control(drive *d)
{
    for (int i = 0; i < 2; i++) {
        d->wheel_target[i] = 0.0f;
        pi_reset(&d->wheel_pi[i]);
    }
    pi_reset(&d->steer_pi);
    pi_reset(&d->lift_pi);
    d->steer_target = d->steer_ref = d->steer.position;
    d->lift_target = d->lift_ref = d->lift.position;
}

bool drive_init(drive *d, const drive_config *c, const drive_inputs *first, uint32_t boot_id)
{
    if (boot_id == 0 || !drive_config_valid(c)) {
        return false;
    }
    memset(d, 0, sizeof *d);
    d->cfg = c;
    d->boot_id = boot_id;
    d->last_now32 = first->now_us;
    d->mode = DRIVE_MODE_BOOT;
    for (int i = 0; i < 2; i++) {
        wheel_enc_init(&d->wheel[i], &c->wheel[i], first->wheel_hw[i], first->now_us);
    }
    abs_axis_init(&d->steer);
    abs_axis_init(&d->lift);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        d->out.drvoff[ch] = true;
    }
    start_driver_init(d, false);
    return true;
}

static bool driving(const drive *d)
{
    return d->mode == DRIVE_MODE_ARMED || d->mode == DRIVE_MODE_STOPPING;
}

static bool sensors_valid(const drive *d)
{
    return d->steer.valid && !d->steer.out_of_range && d->lift.valid && !d->lift.out_of_range &&
           d->adc_valid;
}

static void read_sensors(drive *d, const drive_inputs *in)
{
    const drive_config *c = d->cfg;
    for (int i = 0; i < 2; i++) {
        wheel_enc_update(&d->wheel[i], &c->wheel[i], in->wheel_hw[i], in->now_us);
    }
    abs_axis_update(&d->steer, &c->steer, in->abs_frame[0], in->abs_fresh[0]);
    abs_axis_update(&d->lift, &c->lift, in->abs_frame[1], in->abs_fresh[1]);
    if (in->adc_fresh) {
        d->adc_bad_streak = 0;
        d->adc_valid = true;
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            float volts = (float)in->adc_raw[ch] / (float)c->adc_full_scale * c->adc_vref_v;
            d->current_a[ch] = volts / c->ipropi_ohm * c->ipropi_gain;
        }
        return;
    }
    if (d->adc_bad_streak < UINT16_MAX) {
        d->adc_bad_streak++;
    }
    if (d->adc_bad_streak >= c->adc_invalid_limit) {
        d->adc_valid = false;
    }
}

static uint32_t sensor_bits(const drive *d)
{
    uint32_t bits = 0;
    if (!d->steer.valid) {
        bits |= DRIVE_FAULT_STEER_SENSOR_INVALID;
    } else if (d->steer.out_of_range) {
        bits |= DRIVE_FAULT_STEER_OUT_OF_RANGE;
    }
    if (!d->lift.valid) {
        bits |= DRIVE_FAULT_LIFT_SENSOR_INVALID;
    } else if (d->lift.out_of_range) {
        bits |= DRIVE_FAULT_LIFT_OUT_OF_RANGE;
    }
    if (!d->adc_valid) {
        bits |= DRIVE_FAULT_CURRENT_SENSE_INVALID;
    }
    return bits;
}

/* Judged only while the outputs are on: after a high-side overcurrent the DRV8244 keeps
   IPROPI clamped high with the outputs off (design 4). */
static void judge_driving_faults(drive *d, const drive_inputs *in)
{
    const drive_config *c = d->cfg;
    uint32_t bits = sensor_bits(d);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (d->adc_valid && fabsf(d->current_a[ch]) > c->overcurrent_a[ch]) {
            if (!d->over_active[ch]) {
                d->over_active[ch] = true;
                d->over_since_us[ch] = d->time_us;
            }
            if (d->time_us - d->over_since_us[ch] >= (uint64_t)c->overcurrent_hold_ms * US_PER_MS) {
                bits |= DRIVE_FAULT_OVERCURRENT(ch);
            }
        } else {
            d->over_active[ch] = false;
        }
    }
    d->overrun_streak = in->cycle_overrun ? (uint16_t)(d->overrun_streak + 1u) : 0u;
    if (d->overrun_streak >= c->overrun_limit) {
        bits |= DRIVE_FAULT_CYCLE_OVERRUN;
    }
    if (bits) {
        enter_fault(d, bits);
    }
}

/* Latched bits | live status bits; recomputed after every state change a message makes. */
static void refresh_flags(drive *d)
{
    uint32_t live = sensor_bits(d);
    if (d->estop_now) {
        live |= DRIVE_FAULT_ESTOP_INPUT;
    }
    if (!d->drivers_ready) {
        live |= DRIVE_FAULT_DRIVER_NOT_READY;
    }
    d->fault_flags = d->latched | live;
}

void drive_begin(drive *d, const drive_inputs *in)
{
    d->time_us += (uint32_t)(in->now_us - d->last_now32); /* widen the HAL clock */
    d->last_now32 = in->now_us;
    read_sensors(d, in);
    check_spi_timeout(d);

    /* Expiry first, so a DRIVE arriving after the deadline cannot extend it. */
    if (d->mode == DRIVE_MODE_ARMED && d->has_deadline && d->time_us > d->deadline_us) {
        d->mode = DRIVE_MODE_STOPPING;
        d->has_deadline = false;
        d->latched |= DRIVE_FAULT_COMMAND_TIMEOUT;
        d->wheel_target[0] = d->wheel_target[1] = 0.0f;
        d->steer_target = d->steer_ref = d->steer.position;
        d->lift_target = d->lift_ref = d->lift.position;
        d->stop_until_us = d->time_us + (uint64_t)d->cfg->stop_grace_ms * US_PER_MS;
    }
    /* Faults are judged while the outputs are still on, including the cycle the STOPPING
       grace ends, so they latch before the drop to DISARMED. */
    if (driving(d)) {
        judge_driving_faults(d, in);
    } else {
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            d->over_active[ch] = false;
        }
        d->overrun_streak = 0;
    }
    if (d->mode == DRIVE_MODE_STOPPING && d->time_us >= d->stop_until_us) {
        d->mode = DRIVE_MODE_DISARMED;
        reset_control(d);
    }
    if (d->drivers_ready) {
        uint32_t bits = 0;
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            if (in->nfault_low[ch]) {
                bits |= DRIVE_FAULT_DRIVER(ch);
            }
        }
        if (bits) {
            enter_fault(d, bits);
        }
    }
    d->estop_now = in->estop_active;
    if (in->estop_active) {
        d->latched |= DRIVE_FAULT_ESTOP_INPUT;
        d->mode = DRIVE_MODE_ESTOP;
    }

    step_driver_init(d, in);
    step_status_poll(d);
    refresh_flags(d);
}

/* ---- messages ----------------------------------------------------------------------- */

static bool echo_fresh(const drive *d, uint64_t echo, uint32_t window_ms)
{
    return echo <= d->time_us && d->time_us - echo <= (uint64_t)window_ms * US_PER_MS;
}

static uint8_t arm_check(const drive *d, const link_arm *a)
{
    if (a->boot_id != d->boot_id) {
        return DRIVE_REJECT_BOOT_MISMATCH;
    }
    if (!echo_fresh(d, a->echo_mcu_time_us, d->cfg->command_timeout_ms)) {
        return DRIVE_REJECT_STALE_ECHO;
    }
    switch (d->mode) {
    case DRIVE_MODE_BOOT:
        return DRIVE_REJECT_DRIVER_NOT_READY;
    case DRIVE_MODE_STOPPING:
        return DRIVE_REJECT_STOPPING;
    case DRIVE_MODE_ARMED:
        return DRIVE_REJECT_NOT_DISARMED;
    case DRIVE_MODE_FAULT:
    case DRIVE_MODE_ESTOP:
        return DRIVE_REJECT_LATCHED;
    default:
        break;
    }
    if (d->estop_now) {
        return DRIVE_REJECT_ESTOP_ACTIVE;
    }
    if (!sensors_valid(d)) {
        return DRIVE_REJECT_SENSOR_INVALID;
    }
    return DRIVE_REJECT_NONE;
}

static void handle_arm(drive *d, const link_msg *m)
{
    uint8_t why = arm_check(d, &m->u.arm);
    d->reject_reason = why;
    if (why != DRIVE_REJECT_NONE) {
        return;
    }
    d->arm_gen = d->arm_gen == 0xFFFFu ? 1u : (uint16_t)(d->arm_gen + 1u);
    d->mode = DRIVE_MODE_ARMED;
    d->latched &= ~DRIVE_FAULT_COMMAND_TIMEOUT;
    reset_control(d);
    d->zero_seen = false;
    d->have_drive_seq = false;
    d->limited = false;
    /* An ARM with no DRIVE behind it expires like a command. */
    d->has_deadline = true;
    d->deadline_us = m->u.arm.echo_mcu_time_us + (uint64_t)d->cfg->command_timeout_ms * US_PER_MS;
    d->last_rx_seq = m->seq;
}

static float clip(float x, float lo, float hi, bool *limited)
{
    if (x < lo) {
        *limited = true;
        return lo;
    }
    if (x > hi) {
        *limited = true;
        return hi;
    }
    return x;
}

static uint8_t drive_check(const drive *d, const link_msg *m, uint32_t window_ms)
{
    const link_drive *c = &m->u.drive;
    if (!isfinite(c->left_wheel_rad_s) || !isfinite(c->right_wheel_rad_s) ||
        !isfinite(c->steering_rad) || !isfinite(c->lift_m) || c->valid_after_echo_ms == 0 ||
        (c->flags & ~LINK_DRIVE_FLAG_LIFT_HOLD)) {
        return DRIVE_REJECT_BAD_VALUE;
    }
    if (c->boot_id != d->boot_id) {
        return DRIVE_REJECT_BOOT_MISMATCH;
    }
    if (d->mode != DRIVE_MODE_ARMED) {
        return d->mode == DRIVE_MODE_STOPPING ? DRIVE_REJECT_STOPPING : DRIVE_REJECT_NOT_ARMED;
    }
    if (c->arm_gen != d->arm_gen) {
        return DRIVE_REJECT_ARM_GEN_MISMATCH;
    }
    if (!echo_fresh(d, c->echo_mcu_time_us, window_ms)) {
        return DRIVE_REJECT_STALE_ECHO;
    }
    if (d->have_drive_seq && !link_seq_newer(m->seq, d->last_drive_seq)) {
        return DRIVE_REJECT_STALE_SEQ;
    }
    if (!d->zero_seen && (c->left_wheel_rad_s != 0.0f || c->right_wheel_rad_s != 0.0f)) {
        return DRIVE_REJECT_ZERO_FIRST;
    }
    return DRIVE_REJECT_NONE;
}

static void handle_drive(drive *d, const link_msg *m)
{
    const link_drive *c = &m->u.drive;
    const drive_config *cfg = d->cfg;
    uint32_t window_ms = c->valid_after_echo_ms < cfg->command_timeout_ms ? c->valid_after_echo_ms
                                                                          : cfg->command_timeout_ms;
    d->reject_reason = drive_check(d, m, window_ms);
    if (d->reject_reason != DRIVE_REJECT_NONE) {
        return;
    }
    bool limited = false;
    d->zero_seen = true;
    float w = cfg->max_wheel_rad_s;
    d->wheel_target[0] = clip(c->left_wheel_rad_s, -w, w, &limited);
    d->wheel_target[1] = clip(c->right_wheel_rad_s, -w, w, &limited);
    d->steer_target = clip(c->steering_rad, cfg->steer.min_joint, cfg->steer.max_joint, &limited);
    if (!(c->flags & LINK_DRIVE_FLAG_LIFT_HOLD)) {
        d->lift_target = clip(c->lift_m, cfg->lift.min_joint, cfg->lift.max_joint, &limited);
    }
    d->limited = limited;
    d->has_deadline = true;
    d->deadline_us = c->echo_mcu_time_us + (uint64_t)window_ms * US_PER_MS;
    d->have_drive_seq = true;
    d->last_drive_seq = m->seq;
    d->last_rx_seq = m->seq;
}

static void handle_clear(drive *d, const link_msg *m)
{
    if (d->mode != DRIVE_MODE_FAULT && d->mode != DRIVE_MODE_ESTOP) {
        return;
    }
    if (m->u.clear.boot_id != d->boot_id) {
        d->reject_reason = DRIVE_REJECT_BOOT_MISMATCH;
        return;
    }
    if (d->estop_now) {
        d->reject_reason = DRIVE_REJECT_ESTOP_ACTIVE;
        return;
    }
    if (!sensors_valid(d)) {
        d->reject_reason = DRIVE_REJECT_SENSOR_INVALID;
        return;
    }
    /* Driver faults and overcurrent are not conditions: the re-initialisation decides. */
    d->latched = 0;
    d->mode = DRIVE_MODE_BOOT;
    d->reject_reason = DRIVE_REJECT_NONE;
    d->last_rx_seq = m->seq;
    reset_control(d);
    start_driver_init(d, true);
}

void drive_handle(drive *d, const link_msg *m)
{
    switch (m->type) {
    case LINK_ARM:
        handle_arm(d, m);
        break;
    case LINK_DRIVE:
        handle_drive(d, m);
        break;
    case LINK_DISARM:
        if (driving(d)) {
            d->mode = DRIVE_MODE_DISARMED;
            d->has_deadline = false;
            reset_control(d);
        }
        d->last_rx_seq = m->seq;
        break;
    case LINK_CLEAR_FAULTS:
        handle_clear(d, m);
        break;
    default:
        break;
    }
    refresh_flags(d);
}

/* ---- control and outputs ------------------------------------------------------------ */

void drive_finish(drive *d, drive_outputs *out)
{
    const drive_config *c = d->cfg;
    float dt = (float)c->period_us * 1e-6f;
    drive_outputs o;
    memset(&o, 0, sizeof o);
    if (driving(d)) {
        float duty[DRIVE_CH_COUNT];
        for (int i = 0; i < 2; i++) {
            duty[i] = pi_step(&c->wheel_pi, &d->wheel_pi[i], d->wheel_target[i],
                              d->wheel[i].rate_rad_s, dt);
        }
        d->steer_ref = slew_step(d->steer_ref, d->steer_target, c->steer_rate_max, dt);
        d->lift_ref = slew_step(d->lift_ref, d->lift_target, c->lift_rate_max, dt);
        duty[DRIVE_CH_M] = pi_step(&c->steer_pi, &d->steer_pi, d->steer_ref, d->steer.position, dt);
        duty[DRIVE_CH_H] = pi_step(&c->lift_pi, &d->lift_pi, d->lift_ref, d->lift.position, dt);
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            o.duty[ch] = (float)c->duty_sign[ch] * duty[ch];
        }
    } else {
        /* Safe output per channel: cut unless configured to brake; always cut before the
           drivers are ready and on the channel's own driver fault. */
        for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
            bool own_fault = (d->fault_flags & DRIVE_FAULT_DRIVER(ch)) != 0;
            o.drvoff[ch] = !d->drivers_ready || own_fault || !c->safe_brake[ch];
        }
    }
    d->out = o;
    *out = o;
}

/* ---- STATE -------------------------------------------------------------------------- */

static int16_t sat16(float x)
{
    if (isnan(x)) {
        return 0;
    }
    if (x > 32767.0f) {
        return 32767;
    }
    if (x < -32768.0f) {
        return -32768;
    }
    return (int16_t)lroundf(x);
}

void drive_fill_state(const drive *d, link_state *s)
{
    s->mcu_time_us = d->time_us;
    s->boot_id = d->boot_id;
    s->arm_gen = d->arm_gen;
    s->last_rx_seq = d->last_rx_seq;
    s->mode = d->mode;
    s->reject_reason = d->reject_reason;
    s->fault_flags = d->fault_flags;
    s->wheel_count_left = (int32_t)d->wheel[0].count;
    s->wheel_count_right = (int32_t)d->wheel[1].count;
    s->wheel_rate_left_rad_s = d->wheel[0].rate_rad_s;
    s->wheel_rate_right_rad_s = d->wheel[1].rate_rad_s;
    s->steering_rad = d->steer.valid ? d->steer.position : NAN;
    s->lift_m = d->lift.valid ? d->lift.position : NAN;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        s->current_ma[ch] = sat16(d->current_a[ch] * 1000.0f);
        s->duty_e4[ch] = sat16(d->out.duty[ch] * 10000.0f);
        s->driver_status[ch] = d->driver_status[ch];
    }
    uint8_t flags = 0;
    if (d->limited) {
        flags |= DRIVE_STATE_LIMITED;
    }
    if (d->mode == DRIVE_MODE_ARMED && d->has_deadline && d->time_us <= d->deadline_us) {
        flags |= DRIVE_STATE_COMMAND_FRESH;
    }
    if (d->zero_seen) {
        flags |= DRIVE_STATE_ZERO_SEEN;
    }
    if (d->drivers_ready) {
        flags |= DRIVE_STATE_DRIVERS_READY;
    }
    s->flags = flags;
}
