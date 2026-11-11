/* Vector table and fault handlers. Every fault puts the motor pins back to their safe levels and
   stops without refreshing the IWDG, so the watchdog resets the board (board design 3.3, 3.5). */
#include "board/board.h"
#include "stm32h7xx.h"

#define IRQ_COUNT 150 /* the highest STM32H743 IRQn is 149 (WAKEUP_PIN_IRQn) */

extern uint32_t _estack;
void Reset_Handler(void);
void TIM6_DAC_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
#if defined(FW_LINK_USB)
void OTG_FS_IRQHandler(void);
#elif defined(FW_LINK_UART7)
void UART7_IRQHandler(void);
#else
#error "FW_LINK_USB or FW_LINK_UART7"
#endif

static void safe_stop(void)
{
    __disable_irq();
    motor_out_force_safe();
    for (;;) {
    }
}

/* Losing HSE while running raises CSS -> NMI: running on in a 6x slower clock would stretch every
   deadline, so stop and let the IWDG restart on the fallback clock. */
void NMI_Handler(void)
{
    safe_stop();
}

void HardFault_Handler(void)
{
    safe_stop();
}

void Default_Handler(void)
{
    safe_stop();
}

typedef void (*vector_fn)(void);

/* Every slot is filled: an interrupt nobody expected stops safely instead of jumping to 0 (Codex
   B2a review). The handlers in use override the default after the range (-Wno-override-init for
   this file only). */
__attribute__((section(".isr_vector"), used)) const vector_fn g_vectors[16 + IRQ_COUNT] = {
    [1 ... 16 + IRQ_COUNT - 1] = Default_Handler,
    [0] = (vector_fn)(uintptr_t)&_estack,
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = Default_Handler,  /* MemManage */
    [5] = Default_Handler,  /* BusFault */
    [6] = Default_Handler,  /* UsageFault */
    [11] = Default_Handler, /* SVCall */
    [12] = Default_Handler, /* DebugMon */
    [14] = Default_Handler, /* PendSV */
    [15] = Default_Handler, /* SysTick (not used) */
    [16 + SPI1_IRQn] = SPI1_IRQHandler,
    [16 + SPI2_IRQn] = SPI2_IRQHandler,
    [16 + TIM6_DAC_IRQn] = TIM6_DAC_IRQHandler,
#if defined(FW_LINK_USB)
    [16 + OTG_FS_IRQn] = OTG_FS_IRQHandler,
#else
    [16 + UART7_IRQn] = UART7_IRQHandler,
#endif
};

_Static_assert(TIM6_DAC_IRQn < IRQ_COUNT && WAKEUP_PIN_IRQn == IRQ_COUNT - 1, "vector table size");
