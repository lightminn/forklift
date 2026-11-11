/* TIM2: free-running 32-bit microsecond clock (the core widens it). TIM6: the 1 kHz control tick
   (board design 3.4, interrupt priority 2). */
#include "board/board.h"
#include "board/clock_config.h"
#include "stm32h7xx.h"

#define TICK_IRQ_PRIORITY 2u

static void (*tick_fn)(void);

void timebase_init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
    (void)RCC->APB1LENR;
    TIM2->CR1 = 0;
    TIM2->PSC = CLK_US_PRESCALER - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG; /* load PSC */
    TIM2->SR = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t timebase_now_us(void)
{
    return TIM2->CNT;
}

void tick_start(void (*on_tick)(void))
{
    tick_fn = on_tick;
    RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
    (void)RCC->APB1LENR;
    TIM6->CR1 = 0;
    TIM6->PSC = CLK_US_PRESCALER - 1u;       /* 1 MHz */
    TIM6->ARR = 1000000u / CLK_TICK_HZ - 1u; /* 1 ms */
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, TICK_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR = ~TIM_SR_UIF;
    if (tick_fn) {
        tick_fn();
    }
}
