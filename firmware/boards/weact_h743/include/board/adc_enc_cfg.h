/* Pure pieces of the ADC and encoder setup (board design 3.3, 4.3, 4.4); host-tested. */
#ifndef BOARD_ADC_ENC_CFG_H
#define BOARD_ADC_ENC_CFG_H

#include <stdbool.h>
#include <stdint.h>

#define REV_ID_Y 0x1003u
#define REV_ID_V 0x2003u

/* ADC clock: per_ck (HSI 64 MHz) through ADC12_COMMON PRESC /2 = 32 MHz at the ADC input. Rev V
   divides the kernel clock by 2 again inside the ADC (AN5312 1.7), so the analog clock is 32 MHz
   on rev Y and 16 MHz on rev V. */
#define ADC_INPUT_HZ 32000000u

/* ADC_CR.BOOST for the device revision, or -1 for a revision this firmware does not know (the
   ADC then stays off: current sense invalid, ARM refused -- the safe side). Rev Y: one bit,
   needed above 20 MHz. Rev V: two bits, 10 = 12.5-25 MHz. */
static inline int adc_boost_bits(uint32_t rev_id)
{
    if (rev_id == REV_ID_Y) {
        return ADC_INPUT_HZ > 20000000u ? 1 : 0;
    }
    if (rev_id == REV_ID_V) {
        uint32_t f = ADC_INPUT_HZ / 2u;
        return f <= 6250000u ? 0 : f <= 12500000u ? 1 : f <= 25000000u ? 2 : 3;
    }
    return -1;
}

/* IPROPI L/R/M/H: PA6 INP3, PC4 INP4, PB1 INP5, PC0 INP10 (schematic V1.0; DS12110). */
static const uint8_t ADC_IPROPI_CHANNEL[4] = {3, 4, 5, 10};

/* ADC_JSQR: four injected conversions in channel order, software trigger (JEXTEN = 00). */
static inline uint32_t adc_jsqr_value(void)
{
    return 3u /* JL = 4 conversions */ | ((uint32_t)ADC_IPROPI_CHANNEL[0] << 9) |
           ((uint32_t)ADC_IPROPI_CHANNEL[1] << 15) | ((uint32_t)ADC_IPROPI_CHANNEL[2] << 21) |
           ((uint32_t)ADC_IPROPI_CHANNEL[3] << 27);
}

/* ADC_CFGR: 16-bit, regular conversions unused, JQDIS = 1. With the injected queue enabled
   (JQDIS = 0) and JEXTEN = 00 both hardware and software triggers are disabled (RM0433 ADC_CFGR
   bit 31, 25.6.11) -- JADSTART would never convert (Codex B2b review). Writing JQDIS clears JSQR,
   so CFGR is written before JSQR. */
#define ADC_CFGR_JQDIS_BIT (1u << 31)
static inline uint32_t adc_cfgr_value(void)
{
    return ADC_CFGR_JQDIS_BIT;
}

/* ADC_PCSEL: the channels must be preselected on the H7. */
static inline uint32_t adc_pcsel_value(void)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v |= 1u << ADC_IPROPI_CHANNEL[i];
    }
    return v;
}

/* LPTIM_CNT must be read twice with the same value (RM0433 43.4). read() returns one raw read;
   up to `tries` reads; false (and *out unchanged) if no two consecutive reads agree. */
static inline bool lptim_stable_read(uint16_t (*read)(void *), void *ctx, int tries, uint16_t *out)
{
    uint16_t prev = read(ctx);
    for (int i = 1; i < tries; i++) {
        uint16_t v = read(ctx);
        if (v == prev) {
            *out = v;
            return true;
        }
        prev = v;
    }
    return false;
}

#endif
