/* Bench image: one KTH7801 axis's travel counter (fw/bench_kth.h, bench console plan 4.2 A1b-1 decision 2). */
#include "fw/bench_kth.h"

#if defined(FW_CORE_BENCH)

#include <string.h>

#include "fw/abs_axis.h"

int32_t bench_kth_diff(uint32_t a, uint32_t b)
{
    uint32_t d = a - b; /* both unsigned: no signed overflow at 0x7fffffff/0x80000000 */
    return d <= 0x7fffffffu ? (int32_t)d : -(int32_t)(~d) - 1;
}

void bench_kth_reset(bench_kth *k)
{
    memset(k, 0, sizeof *k);
}

/* The shortest 12-bit step from prev to raw (-2048..2047). */
static int32_t shortest(uint16_t raw, uint16_t prev)
{
    int32_t d = (int32_t)raw - (int32_t)prev;
    if (d >= 2048) {
        d -= 4096;
    } else if (d < -2048) {
        d += 4096;
    }
    return d;
}

/* A step of |step| counts over dt_us is within the speed bound and the gap. */
static bool step_ok(int32_t step, uint64_t dt_us)
{
    uint64_t mag = (uint64_t)(step < 0 ? -(int64_t)step : step);
    return dt_us <= BENCH_KTH_GAP_US && mag * 1000u <= (uint64_t)BENCH_KTH_STEP_PER_MS * dt_us;
}

static uint8_t lose(bench_kth *k, bool base, uint16_t raw, uint64_t t)
{
    k->lost = true;
    k->losses++;
    k->base_valid = base;
    k->base_raw = raw;
    k->base_t = t;
    return BENCH_KTH_LOST;
}

static void trust(bench_kth *k, uint16_t raw, uint64_t t)
{
    k->trust_raw = raw;
    k->trust_cum = k->cum;
    k->trust_t = t;
    k->last_raw = raw;
    k->last_t = t;
    k->last_zero = false;
}

uint8_t bench_kth_tick(bench_kth *k, bool fresh, uint16_t frame, kth_crc_variant v, uint64_t t_acq, uint64_t now)
{
    uint16_t raw = 0;
    bool decoded = fresh && kth7801_decode(frame, v, &raw);
    if (decoded && now - t_acq > BENCH_KTH_GAP_US) {
        /* consumed too late: never counted, never a base (plan 4.2, independent review round 8) */
        return k->have && !k->lost ? lose(k, false, 0, 0) : 0u;
    }
    if (decoded) {
        if (!k->have) {
            if (raw == 0u) {
                return 0u; /* a 0x0000 is never the first sample */
            }
            k->have = true;
            trust(k, raw, t_acq);
            return BENCH_KTH_TRUSTED;
        }
        if (k->lost) {
            if (raw == 0u) {
                return 0u; /* no 0x0000 while lost: it can neither recover nor be a base */
            }
            if (k->base_valid) {
                int32_t step = shortest(raw, k->base_raw);
                if (step_ok(step, t_acq - k->base_t)) {
                    k->lost = false;
                    k->base_valid = false;
                    k->cum += (uint32_t)step;
                    trust(k, raw, t_acq);
                    return BENCH_KTH_TRUSTED | BENCH_KTH_RECOVERED;
                }
            }
            k->base_valid = true; /* the latest non-zero frame is the next comparison base */
            k->base_raw = raw;
            k->base_t = t_acq;
            return 0u;
        }
        if (raw == 0u) {
            if (k->last_raw > BENCH_KTH_ZERO_NEAR && k->last_raw < 4096u - BENCH_KTH_ZERO_NEAR) {
                decoded = false; /* not countable: as if no frame came (the gap judges it) */
            } else {
                int32_t step = shortest(0u, k->last_raw);
                if (!step_ok(step, t_acq - k->last_t)) {
                    return lose(k, false, 0, 0);
                }
                k->cum += (uint32_t)step; /* tentative until the next non-zero frame */
                k->last_raw = 0u;
                k->last_t = t_acq;
                k->last_zero = true;
                return BENCH_KTH_COUNTED_ZERO;
            }
        } else {
            /* re-anchored from the last trusted sample over the whole acquisition interval */
            int32_t step = shortest(raw, k->trust_raw);
            if (!step_ok(step, t_acq - k->trust_t)) {
                return lose(k, true, raw, t_acq); /* the detecting frame: the next comparison base */
            }
            k->cum = k->trust_cum + (uint32_t)step;
            trust(k, raw, t_acq);
            return BENCH_KTH_TRUSTED;
        }
    }
    if (k->have && !k->lost && now - k->last_t > BENCH_KTH_GAP_US) {
        return lose(k, false, 0, 0); /* no countable frame for too long: lost now */
    }
    return 0u;
}

#endif /* FW_CORE_BENCH */
