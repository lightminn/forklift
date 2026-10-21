#include "fw/link.h"

#include <string.h>

#include "fw/cobs.h"

#define HEADER_LEN 4u
#define CRC_LEN 2u
#define ENCODED_MAX (LINK_BODY_MAX + 1u)

#define DRIVE_LEN 33u
#define ARM_LEN 12u
#define CLEAR_LEN 4u
#define STATE_LEN 71u

/* Little-endian writer/reader over a byte buffer; no struct copies (padding, endianness). */
typedef struct {
    uint8_t *p;
} wr;

static void put_u8(wr *w, uint8_t v)
{
    *w->p++ = v;
}

static void put_u16(wr *w, uint16_t v)
{
    put_u8(w, (uint8_t)v);
    put_u8(w, (uint8_t)(v >> 8));
}

static void put_u32(wr *w, uint32_t v)
{
    put_u16(w, (uint16_t)v);
    put_u16(w, (uint16_t)(v >> 16));
}

static void put_u64(wr *w, uint64_t v)
{
    put_u32(w, (uint32_t)v);
    put_u32(w, (uint32_t)(v >> 32));
}

static void put_f32(wr *w, float f)
{
    uint32_t v;
    memcpy(&v, &f, sizeof v);
    put_u32(w, v);
}

typedef struct {
    const uint8_t *p;
} rd;

static uint8_t get_u8(rd *r)
{
    return *r->p++;
}

static uint16_t get_u16(rd *r)
{
    uint16_t lo = get_u8(r);
    return (uint16_t)(lo | (uint16_t)(get_u8(r) << 8));
}

static uint32_t get_u32(rd *r)
{
    uint32_t lo = get_u16(r);
    return lo | ((uint32_t)get_u16(r) << 16);
}

static uint64_t get_u64(rd *r)
{
    uint64_t lo = get_u32(r);
    return lo | ((uint64_t)get_u32(r) << 32);
}

static float get_f32(rd *r)
{
    uint32_t v = get_u32(r);
    float f;
    memcpy(&f, &v, sizeof f);
    return f;
}

static size_t payload_len(uint8_t type)
{
    switch (type) {
    case LINK_DRIVE:
        return DRIVE_LEN;
    case LINK_ARM:
        return ARM_LEN;
    case LINK_DISARM:
        return 0;
    case LINK_CLEAR_FAULTS:
        return CLEAR_LEN;
    case LINK_STATE:
        return STATE_LEN;
    default:
        return SIZE_MAX;
    }
}

static void put_payload(wr *w, const link_msg *m)
{
    switch (m->type) {
    case LINK_DRIVE: {
        const link_drive *d = &m->u.drive;
        put_f32(w, d->left_wheel_rad_s);
        put_f32(w, d->right_wheel_rad_s);
        put_f32(w, d->steering_rad);
        put_f32(w, d->lift_m);
        put_u16(w, d->valid_after_echo_ms);
        put_u8(w, d->flags);
        put_u16(w, d->arm_gen);
        put_u32(w, d->boot_id);
        put_u64(w, d->echo_mcu_time_us);
        break;
    }
    case LINK_ARM:
        put_u32(w, m->u.arm.boot_id);
        put_u64(w, m->u.arm.echo_mcu_time_us);
        break;
    case LINK_CLEAR_FAULTS:
        put_u32(w, m->u.clear.boot_id);
        break;
    case LINK_STATE: {
        const link_state *s = &m->u.state;
        put_u64(w, s->mcu_time_us);
        put_u32(w, s->boot_id);
        put_u16(w, s->arm_gen);
        put_u16(w, s->last_rx_seq);
        put_u8(w, s->mode);
        put_u8(w, s->reject_reason);
        put_u32(w, s->fault_flags);
        put_u32(w, (uint32_t)s->wheel_count_left);
        put_u32(w, (uint32_t)s->wheel_count_right);
        put_f32(w, s->wheel_rate_left_rad_s);
        put_f32(w, s->wheel_rate_right_rad_s);
        put_f32(w, s->steering_rad);
        put_f32(w, s->lift_m);
        for (int i = 0; i < 4; i++) {
            put_u16(w, (uint16_t)s->current_ma[i]);
        }
        for (int i = 0; i < 4; i++) {
            put_u16(w, (uint16_t)s->duty_e4[i]);
        }
        for (int i = 0; i < 4; i++) {
            put_u8(w, s->driver_status[i]);
        }
        put_u16(w, s->rx_errors);
        put_u16(w, s->cycle_overruns);
        put_u8(w, s->flags);
        break;
    }
    default:
        break;
    }
}

static void get_payload(rd *r, link_msg *m)
{
    switch (m->type) {
    case LINK_DRIVE: {
        link_drive *d = &m->u.drive;
        d->left_wheel_rad_s = get_f32(r);
        d->right_wheel_rad_s = get_f32(r);
        d->steering_rad = get_f32(r);
        d->lift_m = get_f32(r);
        d->valid_after_echo_ms = get_u16(r);
        d->flags = get_u8(r);
        d->arm_gen = get_u16(r);
        d->boot_id = get_u32(r);
        d->echo_mcu_time_us = get_u64(r);
        break;
    }
    case LINK_ARM:
        m->u.arm.boot_id = get_u32(r);
        m->u.arm.echo_mcu_time_us = get_u64(r);
        break;
    case LINK_CLEAR_FAULTS:
        m->u.clear.boot_id = get_u32(r);
        break;
    case LINK_STATE: {
        link_state *s = &m->u.state;
        s->mcu_time_us = get_u64(r);
        s->boot_id = get_u32(r);
        s->arm_gen = get_u16(r);
        s->last_rx_seq = get_u16(r);
        s->mode = get_u8(r);
        s->reject_reason = get_u8(r);
        s->fault_flags = get_u32(r);
        s->wheel_count_left = (int32_t)get_u32(r);
        s->wheel_count_right = (int32_t)get_u32(r);
        s->wheel_rate_left_rad_s = get_f32(r);
        s->wheel_rate_right_rad_s = get_f32(r);
        s->steering_rad = get_f32(r);
        s->lift_m = get_f32(r);
        for (int i = 0; i < 4; i++) {
            s->current_ma[i] = (int16_t)get_u16(r);
        }
        for (int i = 0; i < 4; i++) {
            s->duty_e4[i] = (int16_t)get_u16(r);
        }
        for (int i = 0; i < 4; i++) {
            s->driver_status[i] = get_u8(r);
        }
        s->rx_errors = get_u16(r);
        s->cycle_overruns = get_u16(r);
        s->flags = get_u8(r);
        break;
    }
    default:
        break;
    }
}

size_t link_encode(const link_msg *msg, uint8_t *wire, size_t cap)
{
    size_t plen = payload_len(msg->type);
    if (plen == SIZE_MAX) {
        return 0;
    }
    uint8_t body[LINK_BODY_MAX];
    wr w = {body};
    put_u8(&w, LINK_VERSION);
    put_u8(&w, msg->type);
    put_u16(&w, msg->seq);
    put_payload(&w, msg);
    size_t len = HEADER_LEN + plen;
    put_u16(&w, crc16_ccitt_false(body, len));
    len += CRC_LEN;
    if (cap < 1) {
        return 0;
    }
    size_t n = cobs_encode(body, len, wire, cap - 1);
    if (n == 0) {
        return 0;
    }
    wire[n] = 0x00;
    return n + 1;
}

link_status link_decode_body(const uint8_t *body, size_t len, link_msg *out)
{
    if (len > LINK_BODY_MAX) {
        return LINK_OVERSIZE;
    }
    if (len < HEADER_LEN + CRC_LEN) {
        return LINK_BAD_LENGTH;
    }
    uint16_t crc = (uint16_t)(body[len - 2] | (body[len - 1] << 8));
    if (crc16_ccitt_false(body, len - CRC_LEN) != crc) {
        return LINK_BAD_CRC;
    }
    if (body[0] != LINK_VERSION) {
        return LINK_BAD_VERSION;
    }
    size_t plen = payload_len(body[1]);
    if (plen == SIZE_MAX) {
        return LINK_BAD_TYPE;
    }
    if (len != HEADER_LEN + plen + CRC_LEN) {
        return LINK_BAD_LENGTH;
    }
    rd r = {body + 1}; /* past the version byte */
    out->type = get_u8(&r);
    out->seq = get_u16(&r);
    get_payload(&r, out);
    return LINK_OK;
}

void link_rx_init(link_rx *rx)
{
    rx->len = 0;
    rx->overflow = false;
    rx->errors = 0;
}

static void count_error(link_rx *rx)
{
    if (rx->errors < UINT16_MAX) {
        rx->errors++;
    }
}

bool link_rx_feed(link_rx *rx, uint8_t byte, link_msg *out)
{
    if (byte != 0x00) {
        if (rx->len < ENCODED_MAX) {
            rx->buf[rx->len++] = byte;
        } else {
            rx->overflow = true;
        }
        return false;
    }
    size_t len = rx->len;
    bool overflow = rx->overflow;
    rx->len = 0;
    rx->overflow = false;
    if (len == 0 && !overflow) {
        return false; /* empty frame: not an error */
    }
    uint8_t body[LINK_BODY_MAX + 1];
    size_t n = overflow ? 0 : cobs_decode(rx->buf, len, body, sizeof body);
    if (n == 0 || link_decode_body(body, n, out) != LINK_OK) {
        count_error(rx);
        return false;
    }
    return true;
}

bool link_seq_newer(uint16_t candidate, uint16_t last)
{
    return (int16_t)(uint16_t)(candidate - last) > 0;
}
