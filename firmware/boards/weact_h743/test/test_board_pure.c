#include <string.h>

#include "board/board_config.h"
#include "board/mailbox.h"
#include "fw_test.h"

static void mailbox_rules(void)
{
    mailbox m;
    mailbox_init(&m);
    uint8_t ch;
    uint16_t v;
    uint32_t tag;
    CHECK(!mailbox_take(&m, &ch, &v, &tag));
    CHECK(mailbox_put(&m, 2, 0xC0AB, 77));
    CHECK(mailbox_full(&m));
    /* An unconsumed result is kept: a second put is refused, not overwritten. */
    CHECK(!mailbox_put(&m, 3, 0x1234, 78));
    CHECK(mailbox_take(&m, &ch, &v, &tag));
    CHECK_EQ_U(ch, 2);
    CHECK_EQ_U(v, 0xC0AB);
    CHECK_EQ_U(tag, 77);
    CHECK(!mailbox_full(&m));
    CHECK(!mailbox_take(&m, &ch, &v, &tag));
}

/* Interleaving: the consumer runs between the producer's field writes and its publication; it
   must see nothing, then the whole result. */
static void mailbox_interleaving(void)
{
    mailbox m;
    mailbox_init(&m);
    uint8_t ch;
    uint16_t v;
    uint32_t tag;
    m.channel = 1; /* producer stopped after the fields, before `ready` */
    m.value = 0x55;
    m.tag = 9;
    CHECK(!mailbox_take(&m, &ch, &v, &tag));
    m.ready = 1;
    CHECK(mailbox_take(&m, &ch, &v, &tag));
    CHECK(ch == 1 && v == 0x55 && tag == 9);
}

static void generations(void)
{
    uint32_t consumed = 0;
    CHECK(!generation_fresh(0, &consumed));
    CHECK(generation_fresh(1, &consumed));
    CHECK(!generation_fresh(1, &consumed)); /* the same result is fresh once */
    consumed = 0xFFFFFFFFu;
    CHECK(generation_fresh(0, &consumed)); /* wraps */
    CHECK_EQ_U(consumed, 0);
}

static abs_axis_config axis(float zero, float spj, float lo, float hi)
{
    abs_axis_config c;
    memset(&c, 0, sizeof c);
    c.crc = KTH_CRC_G704_BYTES;
    c.zero_offset_rad = zero;
    c.sign = 1;
    c.sensor_per_joint = spj;
    c.min_joint = lo;
    c.max_joint = hi;
    c.invalid_limit = 3;
    return c;
}

static void raw_zero(void)
{
    /* Joint zero at sensor angle 0: a stuck-low MISO reads as joint 0, inside -> refused. */
    abs_axis_config inside = axis(0.0f, 1.0f, -0.24f, 0.24f);
    CHECK(!kth_raw_zero_outside_range(&inside));
    /* Joint zero at half a turn: raw 0 maps to +-pi, far outside. */
    abs_axis_config outside = axis(3.14159265f, 1.0f, -0.24f, 0.24f);
    CHECK(kth_raw_zero_outside_range(&outside));
}

int main(void)
{
    mailbox_rules();
    mailbox_interleaving();
    generations();
    raw_zero();
    return FW_TEST_RESULT();
}
