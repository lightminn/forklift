/* DIAGNOSTIC IMAGE ONLY (branch diag/kth-raw, never merged): with FW_DIAG_KTH_RAW the axis keeps the newest
   fresh raw SPI word whatever its CRC, and a stale cycle leaves it as it was. */
#include <math.h>
#include <stdio.h>

#include "fw/abs_axis.h"

static int failed;
#define CHECK(x)                                                     \
    do {                                                             \
        if (!(x)) {                                                  \
            printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #x);       \
            failed = 1;                                              \
        }                                                            \
    } while (0)

int main(void)
{
    abs_axis_config c = {0};
    c.crc = KTH_CRC_G704_BYTES;
    c.sign = 1;
    c.sensor_per_joint = 1.0f;
    c.min_joint = -1.0f;
    c.max_joint = 1.0f;
    c.invalid_limit = 3;
    abs_axis a;
    abs_axis_init(&a);
    CHECK(a.last_raw == 0);
    abs_axis_update(&a, &c, 0xFFFFu, true); /* an all-ones word: CRC fails, the word is kept */
    CHECK(a.last_raw == 0xFFFFu && a.frames == 1 && a.crc_errors == 1);
    abs_axis_update(&a, &c, 0x1234u, false); /* not fresh: nothing changes */
    CHECK(a.last_raw == 0xFFFFu && a.frames == 1);
    abs_axis_update(&a, &c, 0x0000u, true);
    CHECK(a.last_raw == 0x0000u && a.frames == 2);
    if (!failed) {
        printf("OK test_kthraw\n");
    }
    return failed;
}
