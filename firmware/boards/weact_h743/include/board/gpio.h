/* Small GPIO helpers (registers only). Motor pins are written only by motor_out_*.c. */
#ifndef BOARD_GPIO_H
#define BOARD_GPIO_H

#include <stdint.h>

#include "stm32h7xx.h"

#define GPIO_MODE_IN 0u
#define GPIO_MODE_OUT 1u
#define GPIO_MODE_AF 2u
#define GPIO_PULL_NONE 0u
#define GPIO_PULL_UP 1u
#define GPIO_SPEED_LOW 0u
#define GPIO_SPEED_MEDIUM 1u
#define GPIO_SPEED_HIGH 2u

void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull);
/* Alternate function af, push-pull, given speed. */
void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af, uint32_t speed);

static inline void gpio_set(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = 1u << pin;
}

static inline void gpio_clear(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = 1u << (pin + 16u);
}

static inline uint32_t gpio_read(const GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1u;
}

#endif
