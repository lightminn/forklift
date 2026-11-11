/* The core's HAL table in the HIL build (FW_BUILD=hil, board design 9): the shared vehicle model
   (firmware/plant) fills every input and answers the DRV8244 SPI. No motor-side peripheral is
   started: no DRV or KTH SPI, no LPTIM, no ADC; the motor pins keep their safe levels with nSLEEP
   low (motor_out_locked.c), so no wheel can turn whatever the core computes. The model runs on the
   duties the core computed (contract 3.3 outputs_locked), one 1 ms step per control cycle. */
#include "board/board.h"
#include "board/board_hal.h"
#include "board/link.h"
#include "plant.h"
#include "stm32h7xx.h"

static plant g_plant;
static drive_outputs g_out; /* the last outputs the core computed (all zero before the first) */

static uint32_t hal_now(void *ctx)
{
    (void)ctx;
    return timebase_now_us();
}

static void hal_read(void *ctx, drive_inputs *in)
{
    (void)ctx;
    plant_inputs(&g_plant, in);
    in->estop_active = false; /* no e-stop in the model (estop_wired = false) */
}

static void hal_write(void *ctx, const drive_outputs *out)
{
    (void)ctx;
    g_out = *out;
    motor_out_apply(out); /* pins stay safe */
}

static size_t hal_link_read(void *ctx, uint8_t *buf, size_t cap)
{
    (void)ctx;
    return link_read(buf, cap);
}

static bool hal_link_write(void *ctx, const uint8_t *frame, size_t len)
{
    (void)ctx;
    return link_write_frame(frame, len);
}

static bool hal_spi_start(void *ctx, uint8_t ch, uint16_t tx, uint32_t tag)
{
    (void)ctx;
    return plant_spi_start(&g_plant, ch, tx, tag);
}

static bool hal_spi_take(void *ctx, uint8_t *ch, uint16_t *rx, uint32_t *tag)
{
    (void)ctx;
    return plant_spi_take(&g_plant, ch, rx, tag);
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

const fw_hal *board_hal_init(const drive_config *cfg)
{
    plant_init(&g_plant, cfg);
    return &g_hal;
}

void board_hal_tick_end(void)
{
    plant_step(&g_plant, &g_out, 1e-3); /* CLK_TICK_HZ = 1 kHz */
}
