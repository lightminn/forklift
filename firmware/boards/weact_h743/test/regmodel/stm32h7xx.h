/* Host register model (motor output design 4.2): the registers motor_out_*.c touch, with the
   CMSIS names and bit positions of stm32h743xx.h (CMSIS device H7 v1.10.7), as plain memory.
   regmodel.c interprets them; it is never built for the MCU. */
#ifndef REGMODEL_STM32H7XX_H
#define REGMODEL_STM32H7XX_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR, CCR1,
        CCR2, CCR3, CCR4, BDTR;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t AHB4ENR, APB2ENR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t APB2FZ1, APB4FZ1;
} DBGMCU_TypeDef;

extern GPIO_TypeDef regmodel_gpio[4]; /* A, B, D, E */
extern TIM_TypeDef regmodel_tim1;
extern RCC_TypeDef regmodel_rcc;
extern DBGMCU_TypeDef regmodel_dbgmcu;

#define GPIOA (&regmodel_gpio[0])
#define GPIOB (&regmodel_gpio[1])
#define GPIOD (&regmodel_gpio[2])
#define GPIOE (&regmodel_gpio[3])
#define TIM1 (&regmodel_tim1)
#define RCC (&regmodel_rcc)
#define DBGMCU (&regmodel_dbgmcu)

#define RCC_AHB4ENR_GPIOAEN (1u << 0)
#define RCC_AHB4ENR_GPIOBEN (1u << 1)
#define RCC_AHB4ENR_GPIODEN (1u << 3)
#define RCC_AHB4ENR_GPIOEEN (1u << 4)
#define RCC_APB2ENR_TIM1EN (1u << 0)
#define DBGMCU_APB2FZ1_DBG_TIM1 (1u << 0)
#define DBGMCU_APB4FZ1_DBG_IWDG1 (1u << 18)

#define TIM_CR1_CEN (1u << 0)
#define TIM_CR1_ARPE (1u << 7)
#define TIM_EGR_UG (1u << 0)
#define TIM_SR_UIF (1u << 0)
#define TIM_CCMR1_OC1PE (1u << 3)
#define TIM_CCMR1_OC1M_Pos 4u
#define TIM_CCMR1_OC2PE (1u << 11)
#define TIM_CCMR1_OC2M_Pos 12u
#define TIM_CCMR2_OC3PE (1u << 3)
#define TIM_CCMR2_OC3M_Pos 4u
#define TIM_CCMR2_OC4PE (1u << 11)
#define TIM_CCMR2_OC4M_Pos 12u
#define TIM_CCER_CC1E (1u << 0)
#define TIM_CCER_CC2E (1u << 4)
#define TIM_CCER_CC3E (1u << 8)
#define TIM_CCER_CC4E (1u << 12)
#define TIM_BDTR_OSSI (1u << 10)
#define TIM_BDTR_OSSR (1u << 11)
#define TIM_BDTR_MOE (1u << 15)

#endif
