#include "board/gpio.h"

void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull)
{
    port->PUPDR = (port->PUPDR & ~(3u << (2u * pin))) | (pull << (2u * pin));
    port->MODER = (port->MODER & ~(3u << (2u * pin))) | (mode << (2u * pin));
}

void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af, uint32_t speed)
{
    uint32_t r = pin >> 3, s = (pin & 7u) * 4u;
    port->AFR[r] = (port->AFR[r] & ~(0xFu << s)) | (af << s);
    port->OSPEEDR = (port->OSPEEDR & ~(3u << (2u * pin))) | (speed << (2u * pin));
    port->OTYPER &= ~(1u << pin);
    gpio_mode(port, pin, GPIO_MODE_AF, GPIO_PULL_NONE);
}
