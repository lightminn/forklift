/* Motor pins (main board schematic V1.0 page 2) and their safe levels, shared by
   motor_out_locked.c and motor_out_tim1.c -- the only files that write motor pins. Every register
   write goes through MO_WR: a plain store on the MCU; in the host register model it also runs the
   model after each write, so intermediate states are checked too (motor output design 4.2). */
#ifndef BOARD_MOTOR_PINS_H
#define BOARD_MOTOR_PINS_H

#include <stdint.h>

#include "stm32h7xx.h"

#if defined(FW_HOST_REGMODEL)
void regmodel_after_write(void);
#define MO_WR(reg, v) ((reg) = (v), regmodel_after_write())
#else
#define MO_WR(reg, v) ((reg) = (v))
#endif

#define MO_PIN(n) (1u << (n))

/* Channel L, R, M, H: DRVOFF on GPIOD, PH/IN2 on GPIOE, EN/IN1 on GPIOE (TIM1 CH1-4, AF1). */
static inline uint32_t mo_drvoff_pin(int ch)
{
    return ch == 0 ? 9u : ch == 1 ? 10u : ch == 2 ? 13u : 14u;
}

static inline uint32_t mo_ph_pin(int ch)
{
    return ch == 0 ? 2u : ch == 1 ? 6u : ch == 2 ? 4u : 5u;
}

static inline uint32_t mo_en_pin(int ch)
{
    return ch == 0 ? 9u : ch == 1 ? 11u : ch == 2 ? 13u : 14u;
}

#define MO_NSLEEP_PIN 0u /* GPIOB, shared by the four drivers (MTR_SLEEP) */

/* Safe levels: DRVOFF high (outputs off; also the DRV8244 pull-up), EN/IN1 and PH/IN2 low,
   nSLEEP low (asleep), DRV nCS PA0-PA3 high, the WeAct flashes' nCS PB6 and PD6 high (never
   used). */
#define MO_PA_HIGH (MO_PIN(0) | MO_PIN(1) | MO_PIN(2) | MO_PIN(3))
#define MO_PB_HIGH (MO_PIN(6))
#define MO_PB_LOW (MO_PIN(MO_NSLEEP_PIN))
#define MO_PD_DRVOFF (MO_PIN(9) | MO_PIN(10) | MO_PIN(13) | MO_PIN(14))
#define MO_PD_HIGH (MO_PIN(6) | MO_PD_DRVOFF)
#define MO_PE_EN (MO_PIN(9) | MO_PIN(11) | MO_PIN(13) | MO_PIN(14))
#define MO_PE_PH (MO_PIN(2) | MO_PIN(4) | MO_PIN(5) | MO_PIN(6))
#define MO_PE_LOW (MO_PE_EN | MO_PE_PH)

static inline void mo_safe_levels(void)
{
    MO_WR(GPIOA->BSRR, MO_PA_HIGH);
    MO_WR(GPIOB->BSRR, MO_PB_HIGH);
    MO_WR(GPIOD->BSRR, MO_PD_HIGH);
    MO_WR(GPIOE->BSRR, MO_PE_LOW << 16);
}

/* Output, push-pull, low speed, no pull: MODER = 01 for every pin in mask. */
static inline void mo_make_outputs(GPIO_TypeDef *port, uint32_t mask)
{
    for (uint32_t n = 0; n < 16u; n++) {
        if (mask & MO_PIN(n)) {
            MO_WR(port->OTYPER, port->OTYPER & ~MO_PIN(n));
            MO_WR(port->OSPEEDR, port->OSPEEDR & ~(3u << (2u * n)));
            MO_WR(port->PUPDR, port->PUPDR & ~(3u << (2u * n)));
            MO_WR(port->MODER, (port->MODER & ~(3u << (2u * n))) | (1u << (2u * n)));
        }
    }
}

/* Board design 3.2 step 2: levels first, so a pin turning into an output starts safe. */
static inline void mo_safe_init(void)
{
    MO_WR(RCC->AHB4ENR, RCC->AHB4ENR | RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN |
                            RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN);
    (void)RCC->AHB4ENR; /* the clock is on before the first port access */
    mo_safe_levels();
    MO_WR(GPIOB->BSRR, MO_PB_LOW << 16);
    mo_make_outputs(GPIOA, MO_PA_HIGH);
    mo_make_outputs(GPIOB, MO_PB_HIGH | MO_PB_LOW);
    mo_make_outputs(GPIOD, MO_PD_HIGH);
    mo_make_outputs(GPIOE, MO_PE_LOW);
}

#endif
