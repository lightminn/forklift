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

/* DIAGNOSTIC IMAGE ONLY (tag diag-spi-ssi-2): what the last SPI1 completion really saw, put into
   STATE driver_status by fw_diag_driver_status() below instead of the core's per-channel status.
   While channel L's init keeps failing, R/M/H are never addressed, so the four bytes are free:
     [0] last received word, high byte (valid or not)   [1] its low byte
     [2] SR at that IRQ: b0 RXP b1 EOT b2 TXTF b3 UDR b4 OVR b5 MODF b6 TXC; b7 = CFG2.MASTER now
     [3] (IRQ count mod 16) << 4 | (transaction starts mod 16) */
static volatile uint16_t g_diag_rx;
static volatile uint32_t g_diag_sr;
static volatile uint32_t g_diag_irqs;
static volatile uint32_t g_diag_starts;

void fw_diag_driver_status(uint8_t status[4]); /* overrides the core's weak no-op */
void fw_diag_driver_status(uint8_t status[4])
{
    uint32_t sr = g_diag_sr;
    uint8_t bits = (uint8_t)(((sr & SPI_SR_RXP) ? 0x01u : 0u) | ((sr & SPI_SR_EOT) ? 0x02u : 0u) |
                             ((sr & SPI_SR_TXTF) ? 0x04u : 0u) | ((sr & SPI_SR_UDR) ? 0x08u : 0u) |
                             ((sr & SPI_SR_OVR) ? 0x10u : 0u) | ((sr & SPI_SR_MODF) ? 0x20u : 0u) |
                             ((sr & SPI_SR_TXC) ? 0x40u : 0u) |
                             ((SPI1->CFG2 & SPI_CFG2_MASTER) ? 0x80u : 0u));
    status[0] = (uint8_t)(g_diag_rx >> 8);
    status[1] = (uint8_t)g_diag_rx;
    status[2] = bits;
    status[3] = (uint8_t)(((g_diag_irqs & 0xFu) << 4) | (g_diag_starts & 0xFu));
}

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
    g_diag_starts++;
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
    g_diag_sr = SPI1->SR; /* before spi_finish clears the flags */
    uint16_t rx = spi_finish(SPI1, &ok);
    g_diag_rx = rx;
    g_diag_irqs++;
    gpio_set(GPIOA, ncs_pin[g_ch]); /* nCS rising edge latches the DRV8244 command */
    if (ok) {
        (void)mailbox_put(&g_box, g_ch, rx, g_tag);
    }
    /* A failed transaction gives no answer: the core's spi_timeout_us decides (design 4.2). */
    g_busy = 0;
}
