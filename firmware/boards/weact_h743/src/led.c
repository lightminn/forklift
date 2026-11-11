/* PE3: the WeAct blue LED (through a transistor) and the main board's BOARD_LED net. */
#include "board/board.h"
#include "stm32h7xx.h"

#define LED_PIN 3u

void led_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    (void)RCC->AHB4ENR;
    GPIOE->BSRR = 1u << (LED_PIN + 16u);
    GPIOE->MODER = (GPIOE->MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));
}

void led_set(bool on)
{
    GPIOE->BSRR = on ? (1u << LED_PIN) : (1u << (LED_PIN + 16u));
}

void led_toggle(void)
{
    GPIOE->ODR ^= 1u << LED_PIN;
}

static void spin(uint32_t cycles)
{
    uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < cycles) {
    }
}

void led_fail_forever(uint32_t code)
{
    __disable_irq();
    motor_out_force_safe();
    for (;;) { /* code short blinks, then a pause; the IWDG resets within ~50 ms anyway */
        for (uint32_t i = 0; i < code; i++) {
            led_set(true);
            spin(2000000u);
            led_set(false);
            spin(2000000u);
        }
        spin(8000000u);
    }
}
