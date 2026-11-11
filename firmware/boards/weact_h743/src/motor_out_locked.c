/* Motor outputs of a locked build (board design 3.6): the pins only ever take their safe levels.
   This is the only board file that writes motor pins; TIM1 is poisoned for every board source. */
#include "board/board.h"
#include "board/board_hal.h"
#include "stm32h7xx.h"

#define PIN(n) (1u << (n))

/* Safe levels (meinboard schematic V1.0 page 2):
   DRVOFF L/R/M/H PD9/PD10/PD13/PD14 high (driver outputs off; also the DRV8244 pull-up),
   EN/IN1 L/R/M/H PE9/PE11/PE13/PE14 low, PH/IN2 L/R/M/H PE2/PE6/PE4/PE5 low,
   nSLEEP PB0 low (sleep; B2 raises it for the SPI check), DRV nCS PA0-PA3 high,
   the WeAct flashes' nCS PB6 (QSPI) and PD6 (SPI) high -- they are never used. */
#define PA_HIGH (PIN(0) | PIN(1) | PIN(2) | PIN(3))
#define PB_HIGH (PIN(6))
#define PB_LOW (PIN(0))
#define PD_HIGH (PIN(6) | PIN(9) | PIN(10) | PIN(13) | PIN(14))
#define PE_LOW (PIN(2) | PIN(4) | PIN(5) | PIN(6) | PIN(9) | PIN(11) | PIN(13) | PIN(14))

static void drive_levels(void)
{
    GPIOA->BSRR = PA_HIGH;
    GPIOB->BSRR = PB_HIGH;
    GPIOD->BSRR = PD_HIGH;
    GPIOE->BSRR = PE_LOW << 16;
}

/* Output, push-pull, low speed, no pull: MODER = 01 for every pin in mask. */
static void make_outputs(GPIO_TypeDef *port, uint32_t mask)
{
    for (uint32_t n = 0; n < 16u; n++) {
        if (mask & PIN(n)) {
            port->OTYPER &= ~PIN(n);
            port->OSPEEDR &= ~(3u << (2u * n));
            port->PUPDR &= ~(3u << (2u * n));
            port->MODER = (port->MODER & ~(3u << (2u * n))) | (1u << (2u * n));
        }
    }
}

void motor_out_safe_init(void)
{
    RCC->AHB4ENR |=
        RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
    (void)RCC->AHB4ENR; /* the clock is on before the first port access */
    drive_levels();     /* levels first, so a pin turning into an output starts safe */
    GPIOB->BSRR = PB_LOW << 16;
    make_outputs(GPIOA, PA_HIGH);
    make_outputs(GPIOB, PB_HIGH | PB_LOW);
    make_outputs(GPIOD, PD_HIGH);
    make_outputs(GPIOE, PE_LOW);
}

void motor_out_force_safe(void)
{
    drive_levels();
    GPIOB->BSRR = PB_LOW << 16; /* nSLEEP low: drivers asleep */
}

/* Locked: the requested duty and DRVOFF are ignored; every motor pin keeps its safe level. The
   duty the core computed still goes out in STATE (contract 3.3 outputs_locked). */
void motor_out_apply(const drive_outputs *out)
{
    (void)out;
    drive_levels();
#if defined(FW_BUILD_hil)
    GPIOB->BSRR = PB_LOW << 16; /* HIL: the drivers are never woken (board design 3.6, 9) */
#else
    GPIOB->BSRR = PIN(0); /* nSLEEP stays high once woken */
#endif
}

void motor_out_wake_drivers(void)
{
    GPIOB->BSRR = PIN(0); /* nSLEEP high; DRVOFF high keeps the outputs off */
}
