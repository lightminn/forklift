/* Wheel encoders on LPTIM1 (L: PD12 IN1, PE1 IN2, AF1) and LPTIM2 (R: PB10 IN1, PD11 IN2, AF3)
   in encoder mode (board design 4.3; pinmap A2): internal clock (CKSEL = 0), prescaler /1
   (PRESC = 000), both edges (CKPOL = 10, x4), digital filter 8 kernel clocks (80 ns at the 100 MHz
   APB clock), ARR = 0xFFFF; the core widens the 16-bit count. Kernel clocks: rcc_pclk1 and
   rcc_pclk4 (reset selections), so the inputs may reach 25 MHz -- the encoders give ~104 kHz. */
#include "board/adc_enc_cfg.h"
#include "board/board.h"
#include "board/board_hal.h"
#include "board/gpio.h"
#include "stm32h7xx.h"

#define CKPOL_BOTH 2u
#define CKFLT_8 3u
#define READ_TRIES 4

static uint16_t g_last[2];

static void lptim_encoder(LPTIM_TypeDef *t)
{
    t->CR = 0;
    t->CFGR =
        LPTIM_CFGR_ENC | (CKPOL_BOTH << LPTIM_CFGR_CKPOL_Pos) | (CKFLT_8 << LPTIM_CFGR_CKFLT_Pos);
    t->CR = LPTIM_CR_ENABLE;
    t->ICR = LPTIM_ICR_ARROKCF;
    t->ARR = 0xFFFFu;
    if (!cycles_wait_flag(&t->ISR, LPTIM_ISR_ARROK, 400000u)) { /* 1 ms */
        led_fail_forever(10);
    }
    t->ICR = LPTIM_ICR_ARROKCF;
    t->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
}

void encoder_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
    RCC->APB1LENR |= RCC_APB1LENR_LPTIM1EN;
    RCC->APB4ENR |= RCC_APB4ENR_LPTIM2EN;
    (void)RCC->APB4ENR;
    gpio_af(GPIOD, 12, 1, GPIO_SPEED_LOW);
    gpio_af(GPIOE, 1, 1, GPIO_SPEED_LOW);
    gpio_af(GPIOB, 10, 3, GPIO_SPEED_LOW);
    gpio_af(GPIOD, 11, 3, GPIO_SPEED_LOW);
    lptim_encoder(LPTIM1);
    lptim_encoder(LPTIM2);
}

static uint16_t read_cnt(void *ctx)
{
    return (uint16_t)((LPTIM_TypeDef *)ctx)->CNT;
}

/* Two equal consecutive reads; if none agree within READ_TRIES the previous value is kept (a step
   counted one cycle later, never a torn value). */
void encoder_read(uint16_t hw[2])
{
    LPTIM_TypeDef *t[2] = {LPTIM1, LPTIM2};
    for (int i = 0; i < 2; i++) {
        uint16_t v;
        if (lptim_stable_read(read_cnt, t[i], READ_TRIES, &v)) {
            g_last[i] = v;
        }
        hw[i] = g_last[i];
    }
}
