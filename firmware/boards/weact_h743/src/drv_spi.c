/* DRV8244 SPI on SPI1 (board design 4.2): PA5 SCK, PA7 MOSI, PB4 MISO (AF5), nCS PA0-PA3 for
   L/R/M/H. Mode 1, 16-bit, 100 MHz / 16 = 6.25 MHz (DRV8244 max 10 MHz). One transaction at a
   time; the EOT interrupt (priority 1) puts (channel, answer, tag) into a one-slot mailbox that the
   control tick takes. PB4 is shared with the WeAct SPI flash's output: the flash's nCS (PD6) is
   held high (motor_out_locked.c), and the DRV8244 SDO high level on PB4 is measured at S2 against
   the user's 3.6 V rule (user decision 2026-11-11) before the shared line is kept. */
#include "board/drv_spi.h"

#include "board/gpio.h"
#include "board/mailbox.h"
#include "spi_common.h"
#include "stm32h7xx.h"

#define SPI_IRQ_PRIORITY 1u

static mailbox g_box;
static volatile uint32_t g_busy; /* a transaction is in flight */
static volatile uint8_t g_ch;
static volatile uint32_t g_tag;

static const uint32_t ncs_pin[4] = {0, 1, 2, 3}; /* PA0..PA3 */

void drv_spi_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; /* kernel clock: PLL1Q (SPI123SEL reset value) */
    (void)RCC->APB2ENR;
    for (int i = 0; i < 4; i++) {
        gpio_set(GPIOA, ncs_pin[i]); /* deselected (already high from the safe pins) */
        gpio_mode(GPIOA, ncs_pin[i], GPIO_MODE_OUT, GPIO_PULL_NONE);
    }
    spi_setup(SPI1, SPI_MBR_DIV16, 0, 1); /* mode 1: CPOL 0, CPHA 1 */
    gpio_af(GPIOA, 5, 5, GPIO_SPEED_MEDIUM);
    gpio_af(GPIOA, 7, 5, GPIO_SPEED_MEDIUM);
    gpio_af(GPIOB, 4, 5, GPIO_SPEED_MEDIUM);
    mailbox_init(&g_box);
    g_busy = 0;
    NVIC_SetPriority(SPI1_IRQn, SPI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SPI1_IRQn);
}

bool drv_spi_begin(uint8_t ch, uint16_t tx, uint32_t tag)
{
    /* Refused while busy or while an answer waits to be taken (board design 3.4). */
    if (ch >= 4u || g_busy || mailbox_full(&g_box)) {
        return false;
    }
    g_busy = 1;
    g_ch = ch;
    g_tag = tag;
    gpio_clear(GPIOA, ncs_pin[ch]);
    spi_start(SPI1, tx);
    return true;
}

bool drv_spi_collect(uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    return mailbox_take(&g_box, ch, rx, tag);
}

void SPI1_IRQHandler(void)
{
    int ok;
    uint16_t rx = spi_finish(SPI1, &ok);
    gpio_set(GPIOA, ncs_pin[g_ch]); /* nCS rising edge latches the DRV8244 command */
    if (ok) {
        (void)mailbox_put(&g_box, g_ch, rx, g_tag);
    }
    /* A failed transaction gives no answer: the core's spi_timeout_us decides (design 4.2). */
    g_busy = 0;
}
