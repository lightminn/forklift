/* The core's HAL table on the WeAct H743 (board design 4, core design 3): DRV8244 SPI, KTH frames,
   wheel encoders, IPROPI current sense, nFAULT, locked outputs, IWDG. The link comes in B3. */
#include "board/board_hal.h"
#include "board/board.h"
#include "board/drv_spi.h"
#include "board/gpio.h"
#include "stm32h7xx.h"

/* nFAULT L/R/M/H: PB11, PB12, PB14, PB15 (open drain, low = fault). */
static const uint32_t nfault_pin[4] = {11, 12, 14, 15};

static uint32_t hal_now(void *ctx)
{
    (void)ctx;
    return timebase_now_us();
}

static void hal_read(void *ctx, drive_inputs *in)
{
    (void)ctx;
    encoder_read(in->wheel_hw);
    kth_spi_snapshot(in->abs_frame, in->abs_fresh);
    in->adc_fresh = adc_take(in->adc_raw);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        in->nfault_low[ch] = gpio_read(GPIOB, nfault_pin[ch]) == 0u;
    }
    in->estop_active = false; /* no e-stop net on schematic V1.0 (estop_wired = false) */
}

static void hal_write(void *ctx, const drive_outputs *out)
{
    (void)ctx;
    motor_out_apply(out);
}

static size_t hal_link_read(void *ctx, uint8_t *buf, size_t cap)
{
    (void)ctx;
    (void)buf;
    (void)cap;
    return 0; /* B3 */
}

static bool hal_link_write(void *ctx, const uint8_t *frame, size_t len)
{
    (void)ctx;
    (void)frame;
    (void)len;
    return false; /* B3; the core counts the dropped STATE */
}

static bool hal_spi_start(void *ctx, uint8_t ch, uint16_t tx, uint32_t tag)
{
    (void)ctx;
    return drv_spi_begin(ch, tx, tag);
}

static bool hal_spi_take(void *ctx, uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    (void)ctx;
    return drv_spi_collect(ch, rx, tag);
}

static void hal_kick(void *ctx)
{
    (void)ctx;
    iwdg_kick();
}

static const fw_hal g_hal = {
    .ctx = 0,
    .now_us = hal_now,
    .read_inputs = hal_read,
    .write_outputs = hal_write,
    .link_read = hal_link_read,
    .link_write_frame = hal_link_write,
    .drv_spi_start = hal_spi_start,
    .drv_spi_take = hal_spi_take,
    .watchdog_kick = hal_kick,
};

const fw_hal *board_hal_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    (void)RCC->AHB4ENR;
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        gpio_mode(GPIOB, nfault_pin[ch], GPIO_MODE_IN, GPIO_PULL_UP);
    }
    drv_spi_init();
    kth_spi_init();
    encoder_init();
    adc_init();
    motor_out_wake_drivers();
    return &g_hal;
}
