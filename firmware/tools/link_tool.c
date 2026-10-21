/* Host-only mcu-link/1 encoder/decoder for the C <-> Python cross-check tests.
 *
 *   link_tool encode drive <seq> <left> <right> <steer> <lift> <valid_after_echo_ms> <flags>
 *                          <arm_gen> <boot_id> <echo>
 *   link_tool encode arm <seq> <boot_id> <echo>
 *   link_tool encode disarm <seq>
 *   link_tool encode clear <seq> <boot_id>
 *   link_tool encode state <seq> <28 values in contract 3.3 order, arrays flattened>
 *   link_tool decode            (hex wire per stdin line -> "<type> <seq> <values...>")
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fw/link.h"

static unsigned long long num(const char *s)
{
    return strtoull(s, NULL, 0);
}

static long long snum(const char *s)
{
    return strtoll(s, NULL, 0);
}

static float flt(const char *s)
{
    return strtof(s, NULL);
}

static int encode(int argc, char **argv)
{
    if (argc < 4) {
        return 2;
    }
    const char *kind = argv[2];
    link_msg m;
    memset(&m, 0, sizeof m);
    m.seq = (uint16_t)num(argv[3]);
    char **a = argv + 4;
    int n = argc - 4;
    if (strcmp(kind, "drive") == 0 && n == 9) {
        m.type = LINK_DRIVE;
        m.u.drive = (link_drive){flt(a[0]),           flt(a[1]),           flt(a[2]),
                                 flt(a[3]),           (uint16_t)num(a[4]), (uint8_t)num(a[5]),
                                 (uint16_t)num(a[6]), (uint32_t)num(a[7]), (uint64_t)num(a[8])};
    } else if (strcmp(kind, "arm") == 0 && n == 2) {
        m.type = LINK_ARM;
        m.u.arm = (link_arm){(uint32_t)num(a[0]), (uint64_t)num(a[1])};
    } else if (strcmp(kind, "disarm") == 0 && n == 0) {
        m.type = LINK_DISARM;
    } else if (strcmp(kind, "clear") == 0 && n == 1) {
        m.type = LINK_CLEAR_FAULTS;
        m.u.clear.boot_id = (uint32_t)num(a[0]);
    } else if (strcmp(kind, "state") == 0 && n == 28) {
        link_state *s = &m.u.state;
        m.type = LINK_STATE;
        s->mcu_time_us = (uint64_t)num(a[0]);
        s->boot_id = (uint32_t)num(a[1]);
        s->arm_gen = (uint16_t)num(a[2]);
        s->last_rx_seq = (uint16_t)num(a[3]);
        s->mode = (uint8_t)num(a[4]);
        s->reject_reason = (uint8_t)num(a[5]);
        s->fault_flags = (uint32_t)num(a[6]);
        s->wheel_count_left = (int32_t)snum(a[7]);
        s->wheel_count_right = (int32_t)snum(a[8]);
        s->wheel_rate_left_rad_s = flt(a[9]);
        s->wheel_rate_right_rad_s = flt(a[10]);
        s->steering_rad = flt(a[11]);
        s->lift_m = flt(a[12]);
        for (int i = 0; i < 4; i++) {
            s->current_ma[i] = (int16_t)snum(a[13 + i]);
            s->duty_e4[i] = (int16_t)snum(a[17 + i]);
            s->driver_status[i] = (uint8_t)num(a[21 + i]);
        }
        s->rx_errors = (uint16_t)num(a[25]);
        s->cycle_overruns = (uint16_t)num(a[26]);
        s->flags = (uint8_t)num(a[27]);
    } else {
        return 2;
    }
    uint8_t wire[LINK_WIRE_MAX];
    size_t len = link_encode(&m, wire, sizeof wire);
    if (len == 0) {
        return 1;
    }
    for (size_t i = 0; i < len; i++) {
        printf("%02x", wire[i]);
    }
    printf("\n");
    return 0;
}

static void print_msg(const link_msg *m)
{
    switch (m->type) {
    case LINK_DRIVE: {
        const link_drive *d = &m->u.drive;
        printf("drive %u %.9g %.9g %.9g %.9g %u %u %u %lu %llu\n", m->seq, d->left_wheel_rad_s,
               d->right_wheel_rad_s, d->steering_rad, d->lift_m, d->valid_after_echo_ms, d->flags,
               d->arm_gen, (unsigned long)d->boot_id, (unsigned long long)d->echo_mcu_time_us);
        break;
    }
    case LINK_ARM:
        printf("arm %u %lu %llu\n", m->seq, (unsigned long)m->u.arm.boot_id,
               (unsigned long long)m->u.arm.echo_mcu_time_us);
        break;
    case LINK_DISARM:
        printf("disarm %u\n", m->seq);
        break;
    case LINK_CLEAR_FAULTS:
        printf("clear %u %lu\n", m->seq, (unsigned long)m->u.clear.boot_id);
        break;
    case LINK_STATE: {
        const link_state *s = &m->u.state;
        printf("state %u %llu %lu %u %u %u %u %lu %ld %ld %.9g %.9g %.9g %.9g", m->seq,
               (unsigned long long)s->mcu_time_us, (unsigned long)s->boot_id, s->arm_gen,
               s->last_rx_seq, s->mode, s->reject_reason, (unsigned long)s->fault_flags,
               (long)s->wheel_count_left, (long)s->wheel_count_right, s->wheel_rate_left_rad_s,
               s->wheel_rate_right_rad_s, s->steering_rad, s->lift_m);
        for (int i = 0; i < 4; i++) {
            printf(" %d", s->current_ma[i]);
        }
        for (int i = 0; i < 4; i++) {
            printf(" %d", s->duty_e4[i]);
        }
        for (int i = 0; i < 4; i++) {
            printf(" %u", s->driver_status[i]);
        }
        printf(" %u %u %u\n", s->rx_errors, s->cycle_overruns, s->flags);
        break;
    }
    default:
        printf("error type\n");
    }
}

static int decode(void)
{
    char line[512];
    while (fgets(line, sizeof line, stdin)) {
        link_rx rx;
        link_rx_init(&rx);
        link_msg m;
        int got = 0;
        for (size_t i = 0; line[i] && line[i + 1] && line[i] != '\n'; i += 2) {
            char hex[3] = {line[i], line[i + 1], 0};
            if (link_rx_feed(&rx, (uint8_t)strtoul(hex, NULL, 16), &m)) {
                print_msg(&m);
                got = 1;
            }
        }
        if (!got) {
            printf("error %u\n", rx.errors);
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "encode") == 0) {
        return encode(argc, argv);
    }
    if (argc == 2 && strcmp(argv[1], "decode") == 0) {
        return decode();
    }
    fprintf(stderr, "usage: link_tool encode <kind> <seq> ... | link_tool decode\n");
    return 2;
}
