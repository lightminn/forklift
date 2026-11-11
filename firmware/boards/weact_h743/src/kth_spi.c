/* KTH7801 absolute encoders on SPI2 (board design 4.1, 4.6): PB13 SCK, PC1 MOSI, PC2_C MISO
   (AF5), CS1 PD0 steering, CS2 PD1 lift; both through LVDS. Mode 3, 100 MHz / 32 = 3.125 MHz to
   start (KTH7801 max 10 MHz; raised after the S2 cable check). The control tick starts one bundle
   (steering, then lift) per cycle; the EOT interrupt stores each frame with a completion
   generation, waits >= 1 us with CS high, then starts the lift read. The MOSI word sent and the
   minimum CS-high time are KTH7801 datasheet values to confirm on first hardware (S2). */
#include "board/board.h"
#include "board/drv_spi.h"
#include "board/gpio.h"
#include "board/mailbox.h"
#include "spi_common.h"
#include "stm32h7xx.h"

#define SPI_IRQ_PRIORITY 1u
#define KTH_READ_WORD 0x0000u     /* S2 확인 */
#define CS_GAP_CYCLES (400u * 1u) /* 1 us at 400 MHz; > one 3.125 MHz SCK (320 ns) */

enum { STAGE_IDLE, STAGE_STEER, STAGE_LIFT };

static volatile uint32_t g_stage;
static volatile uint32_t g_slot[2];       /* generation | frame, one word (mailbox.h slot_pack) */
static uint16_t g_generation[2];          /* interrupt side */
static uint16_t g_consumed[2];            /* tick side */
static const uint32_t cs_pin[2] = {0, 1}; /* PD0, PD1 */

static void wait_cycles(uint32_t n)
{
    uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < n) {
    }
}

void kth_spi_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;
    (void)RCC->APB1LENR;
    /* PC2_C reaches the digital PC2 functions only with its analog switch closed (pinmap B2). */
    SYSCFG->PMCR &= ~SYSCFG_PMCR_PC2SO;
    for (int i = 0; i < 2; i++) {
        gpio_set(GPIOD, cs_pin[i]);
        gpio_mode(GPIOD, cs_pin[i], GPIO_MODE_OUT, GPIO_PULL_NONE);
    }
    spi_setup(SPI2, SPI_MBR_DIV32, 1, 1); /* mode 3 */
    gpio_af(GPIOB, 13, 5, GPIO_SPEED_MEDIUM);
    gpio_af(GPIOC, 1, 5, GPIO_SPEED_MEDIUM);
    gpio_af(GPIOC, 2, 5, GPIO_SPEED_MEDIUM);
    g_stage = STAGE_IDLE;
    NVIC_SetPriority(SPI2_IRQn, SPI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SPI2_IRQn);
}

void kth_spi_start_bundle(void)
{
    if (g_stage != STAGE_IDLE) {
        return; /* the previous bundle is still running: no fresh frame this cycle */
    }
    g_stage = STAGE_STEER;
    gpio_clear(GPIOD, cs_pin[0]);
    wait_cycles(CS_GAP_CYCLES);
    spi_start(SPI2, KTH_READ_WORD);
}

void kth_spi_snapshot(uint16_t frame[2], bool fresh[2])
{
    for (int i = 0; i < 2; i++) {
        fresh[i] = slot_take(g_slot[i], &g_consumed[i], &frame[i]); /* one 32-bit read */
    }
}

void SPI2_IRQHandler(void)
{
    int ok;
    uint16_t rx = spi_finish(SPI2, &ok);
    uint32_t axis = g_stage == STAGE_STEER ? 0u : 1u;
    gpio_set(GPIOD, cs_pin[axis]);
    if (ok) {
        g_generation[axis]++;
        g_slot[axis] = slot_pack(g_generation[axis], rx); /* one 32-bit write */
    }
    if (g_stage == STAGE_STEER) {
        g_stage = STAGE_LIFT;
        wait_cycles(CS_GAP_CYCLES); /* erratum gap and CS high time before the next read */
        gpio_clear(GPIOD, cs_pin[1]);
        wait_cycles(CS_GAP_CYCLES);
        spi_start(SPI2, KTH_READ_WORD);
    } else {
        g_stage = STAGE_IDLE;
    }
}
