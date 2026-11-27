/* Bench image (bench console plan 4.2, A1b-1): one KTH7801 axis's travel counter.

   The 12-bit angle is unwrapped frame by frame into a 32-bit modular count. A 0x0000 frame passes the CRC and is
   also what a MISO stuck low reads, so it is counted only near the sensor's zero and only tentatively: the next
   non-zero frame re-anchors the count from the last trusted (non-zero) sample. Every interval is measured on the
   frames' acquisition times (the board's SPI completion time, widened to 64 bits), never on the tick that consumed
   them. Compiled only with FW_CORE_BENCH. */
#ifndef FW_BENCH_KTH_H
#define FW_BENCH_KTH_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/crc.h"

#if defined(FW_CORE_BENCH)

#define BENCH_KTH_STEP_PER_MS 512u /* counts a ms: the step bound (125 sensor turns/s) */
#define BENCH_KTH_GAP_US 3000u     /* no countable frame for longer, or a frame older than this: the count is lost */
#define BENCH_KTH_ZERO_NEAR 512u   /* a 0x0000 frame is countable only this close to the sensor's zero */

/* bench_kth_tick result bits */
#define BENCH_KTH_COUNTED_ZERO 0x01u /* a 0x0000 frame counted (tentative) */
#define BENCH_KTH_TRUSTED 0x02u      /* a non-zero frame counted: the count re-anchored on it */
#define BENCH_KTH_LOST 0x04u         /* the count was lost this tick */
#define BENCH_KTH_RECOVERED 0x08u    /* the first normal step after a loss (with TRUSTED) */

typedef struct {
    bool have;     /* a trusted sample exists: the count is no longer none_yet */
    bool lost;     /* lost until a normal non-zero step from the comparison base */
    bool base_valid;
    uint16_t base_raw; /* while lost: the latest non-zero frame, the base the next one is judged from */
    uint64_t base_t;
    uint16_t trust_raw; /* the last counted non-zero frame */
    uint32_t trust_cum;
    uint64_t trust_t;
    uint16_t last_raw; /* the last counted frame, zero or not */
    uint64_t last_t;
    bool last_zero; /* the last counted frame was 0x0000 (the count is tentative) */
    uint32_t cum;   /* modular: read differences with bench_kth_diff */
    uint32_t losses; /* loss events since boot (modular): a run compares it with its value at the ARM */
} bench_kth;

void bench_kth_reset(bench_kth *k);

/* One control tick. fresh: a frame was consumed this tick, acquired at t_acq (64-bit, never after now). Returns
   BENCH_KTH_* bits. */
uint8_t bench_kth_tick(bench_kth *k, bool fresh, uint16_t frame, kth_crc_variant v, uint64_t t_acq, uint64_t now);

/* a - b as a signed difference of two modular counts (both converted to unsigned before the subtraction). */
int32_t bench_kth_diff(uint32_t a, uint32_t b);

#endif /* FW_CORE_BENCH */
#endif
