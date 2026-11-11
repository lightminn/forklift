#include <string.h>

#include "board/board_config.h"
#include "board/mailbox.h"
#include "board/spi_cfg.h"
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

/* Codex B2a review: a consumer that missed one completion must not see the next frame as fresh
   twice. With the generation and the frame in one word the pair cannot be torn. */
static void sensor_slot(void)
{
    uint16_t consumed = 0, frame = 0;
    uint32_t slot = slot_pack(1, 0xAAAA); /* generation 1, never taken */
    slot = slot_pack(2, 0xBBBB);          /* the interrupt completes again before the tick */
    CHECK(slot_take(slot, &consumed, &frame));
    CHECK_EQ_U(frame, 0xBBBB);
    CHECK(!slot_take(slot, &consumed, &frame)); /* no new completion: not fresh again */
    CHECK_EQ_U(frame, 0xBBBB);
    /* Wraps over 16 bits. */
    consumed = 0xFFFF;
    CHECK(slot_take(slot_pack(0, 0x1234), &consumed, &frame));
    CHECK_EQ_U(consumed, 0);
    CHECK(!slot_take(slot_pack(0, 0x1234), &consumed, &frame));
}

static void spi_cfg1(void)
{
    uint32_t v = spi_cfg1_value(3u);
    CHECK_EQ_U((v >> 16) & 0x1Fu, 7u); /* CRCSIZE kept at its reset value */
    CHECK_EQ_U(v & 0x1Fu, 15u);        /* DSIZE 16-bit */
    CHECK_EQ_U((v >> 28) & 0x7u, 3u);  /* MBR /16 */
    CHECK_EQ_U(v & (1u << 22), 0u);    /* CRCEN off */
}

int main(void)
{
    spi_cfg1();
    sensor_slot();
    mailbox_rules();
    mailbox_interleaving();
    raw_zero();
    return FW_TEST_RESULT();
}
