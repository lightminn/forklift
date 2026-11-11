#include <string.h>

#include "board/adc_enc_cfg.h"
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

static void adc_values(void)
{
    CHECK_EQ_I(adc_boost_bits(REV_ID_Y), 1); /* 32 MHz analog clock: BOOST on */
    CHECK_EQ_I(adc_boost_bits(REV_ID_V), 2); /* 16 MHz: 12.5-25 MHz band */
    CHECK_EQ_I(adc_boost_bits(0x2001u), -1); /* unknown revision: ADC stays off */
    uint32_t j = adc_jsqr_value();
    CHECK_EQ_U(j & 3u, 3u);        /* JL: 4 conversions */
    CHECK_EQ_U((j >> 7) & 3u, 0u); /* JEXTEN: software trigger */
    CHECK_EQ_U((j >> 9) & 0x1Fu, 3u);
    CHECK_EQ_U((j >> 15) & 0x1Fu, 4u);
    CHECK_EQ_U((j >> 21) & 0x1Fu, 5u);
    CHECK_EQ_U((j >> 27) & 0x1Fu, 10u);
    CHECK_EQ_U(adc_pcsel_value(), (1u << 3) | (1u << 4) | (1u << 5) | (1u << 10));
    /* Software-triggered injected conversions need the queue disabled (JQDIS) when JEXTEN = 00. */
    CHECK(((j >> 7) & 3u) != 0u || (adc_cfgr_value() & ADC_CFGR_JQDIS_BIT) != 0u);
    CHECK_EQ_U((adc_cfgr_value() >> 2) & 7u, 0u); /* RES: 16-bit */
}

typedef struct {
    const uint16_t *seq;
    int n, i;
} fake_cnt;

static uint16_t fake_read(void *ctx)
{
    fake_cnt *f = ctx;
    return f->seq[f->i < f->n ? f->i++ : f->n - 1];
}

static void lptim_reads(void)
{
    uint16_t out = 0xDEAD;
    const uint16_t steady[] = {7, 7};
    fake_cnt a = {steady, 2, 0};
    CHECK(lptim_stable_read(fake_read, &a, 3, &out));
    CHECK_EQ_U(out, 7);
    const uint16_t moving[] = {7, 8, 8};
    fake_cnt b = {moving, 3, 0};
    CHECK(lptim_stable_read(fake_read, &b, 3, &out));
    CHECK_EQ_U(out, 8);
    const uint16_t never[] = {1, 2, 3};
    fake_cnt c = {never, 3, 0};
    out = 0xBEEF;
    CHECK(!lptim_stable_read(fake_read, &c, 3, &out));
    CHECK_EQ_U(out, 0xBEEF);
}

int main(void)
{
    adc_values();
    lptim_reads();
    spi_cfg1();
    sensor_slot();
    mailbox_rules();
    mailbox_interleaving();
    raw_zero();
    return FW_TEST_RESULT();
}
