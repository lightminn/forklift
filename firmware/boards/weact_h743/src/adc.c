/* IPROPI current sense on ADC1 injected conversions (board design 3.3, 4.4): PA6 INP3, PC4 INP4,
   PB1 INP5, PC0 INP10, 16-bit, software-started at the end of each tick and taken at the head of
   the next. A result is fresh once: JEOS is cleared (write 1) when it is consumed, and without a
   new JEOS the sample is not fresh, so the core's current_sense_invalid works (Codex design P2).

   SAFETY: on schematic V1.0 IPROPI reaches these pins through 1 kOhm / 10 nF only. The DRV8244
   clamps IPROPI at 4.5-5.5 V on an output fault, above PC4/PB1's 4.0 V absolute maximum. That is
   why the build keeps the motor outputs locked (plan 5 precondition, lead decision Q6): with
   DRVOFF high no load current flows and IPROPI stays near 0 V. */
#include "board/adc_enc_cfg.h"
#include "board/board.h"
#include "board/board_hal.h"
#include "board/gpio.h"
#include "stm32h7xx.h"

#define SMP_64_5 5u         /* sample time 64.5 cycles: 2-4 us per channel */
#define WAIT_CYCLES 400000u /* 1 ms at 400 MHz */

static bool g_on;

static void ch_analog(GPIO_TypeDef *port, uint32_t pin)
{
    port->MODER |= 3u << (2u * pin);
}

static uint32_t rev_id(void)
{
    return (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID) >> DBGMCU_IDCODE_REV_ID_Pos;
}

/* Calibrate and enable ADC1; false leaves it off (unknown revision or a timeout). */
static bool start(void)
{
    int boost = adc_boost_bits(rev_id());
    if (boost < 0) {
        return false;
    }
    RCC->D3CCIPR =
        (RCC->D3CCIPR & ~RCC_D3CCIPR_ADCSEL) | (2u << RCC_D3CCIPR_ADCSEL_Pos); /* per_ck */
    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    (void)RCC->AHB1ENR;
    ADC12_COMMON->CCR = (1u << ADC_CCR_PRESC_Pos); /* async kernel clock / 2; CKMODE = 00 */

    ADC1->CR = 0; /* leave deep power-down */
    ADC1->CR = ADC_CR_ADVREGEN;
    uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 8000u) { /* tADCVREG_STUP 10 us max at 400 MHz = 4000 cycles */
    }
    ADC1->CR = ADC_CR_ADVREGEN | ((uint32_t)boost << ADC_CR_BOOST_Pos);
    ADC1->CFGR = adc_cfgr_value(); /* 16-bit, JQDIS: software injected triggers (before JSQR) */
    iwdg_kick();
    ADC1->CR |= ADC_CR_ADCALLIN | ADC_CR_ADCAL; /* offset and linearity, single-ended */
    for (t0 = cycles_now(); ADC1->CR & ADC_CR_ADCAL;) {
        if (cycles_now() - t0 > 20u * WAIT_CYCLES) { /* 20 ms: inside the 40 ms IWDG budget */
            return false;
        }
    }
    iwdg_kick();
    ADC1->PCSEL = adc_pcsel_value();
    ADC1->SMPR1 = (SMP_64_5 << ADC_SMPR1_SMP3_Pos) | (SMP_64_5 << ADC_SMPR1_SMP4_Pos) |
                  (SMP_64_5 << ADC_SMPR1_SMP5_Pos);
    ADC1->SMPR2 = SMP_64_5 << ADC_SMPR2_SMP10_Pos;
    ADC1->JSQR = adc_jsqr_value();
    /* ADEN set within 4 ADC clocks of the calibration's end can be cleared again (Codex B2b
       review; the ST HAL re-sets it): wait > 4 clocks (250 ns at 16 MHz), then re-set ADEN while
       ADRDY has not come, within the timeout. */
    for (t0 = cycles_now(); cycles_now() - t0 < 400u;) { /* 1 us */
    }
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    for (t0 = cycles_now(); !(ADC1->ISR & ADC_ISR_ADRDY);) {
        if (cycles_now() - t0 > WAIT_CYCLES) {
            return false;
        }
        if (!(ADC1->CR & ADC_CR_ADEN)) {
            ADC1->CR |= ADC_CR_ADEN;
        }
    }
    return true;
}

void adc_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;
    ch_analog(GPIOA, 6);
    ch_analog(GPIOC, 4);
    ch_analog(GPIOB, 1);
    ch_analog(GPIOC, 0);
    g_on = start();
}

/* At the end of a tick: start the next conversion unless one is running. */
void adc_start(void)
{
    if (g_on && !(ADC1->CR & ADC_CR_JADSTART)) {
        ADC1->CR |= ADC_CR_JADSTART;
    }
}

/* At the head of a tick: the four results if a new sequence ended (fresh once). */
bool adc_take(uint32_t raw[4])
{
    if (!g_on || !(ADC1->ISR & ADC_ISR_JEOS)) {
        return false;
    }
    raw[0] = ADC1->JDR1;
    raw[1] = ADC1->JDR2;
    raw[2] = ADC1->JDR3;
    raw[3] = ADC1->JDR4;
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC; /* consumed */
    return true;
}
