/* One control cycle over a board HAL table (design section 3). Every HAL call is
   non-blocking; a board provides the table once its pins and peripherals are fixed. */
#ifndef FW_APP_H
#define FW_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fw/drive.h"
#include "fw/link.h"

typedef struct {
    void *ctx;
    uint32_t (*now_us)(void *ctx);
    /* Fills the input snapshot except now_us and cycle_overrun (set by the app). */
    void (*read_inputs)(void *ctx, drive_inputs *in);
    void (*write_outputs)(void *ctx, const drive_outputs *out);
    size_t (*link_read)(void *ctx, uint8_t *buf, size_t cap);
    /* Queues the whole frame or nothing. */
    bool (*link_write_frame)(void *ctx, const uint8_t *frame, size_t len);
    /* Starts one DRV8244 transaction (false = cannot start now, e.g. the bus is busy). */
    bool (*drv_spi_start)(void *ctx, uint8_t ch, uint16_t tx, uint32_t tag);
    /* A completed transaction, if any, with the tag it was started with. */
    bool (*drv_spi_take)(void *ctx, uint8_t *ch, uint16_t *rx, uint32_t *tag);
    void (*watchdog_kick)(void *ctx);
} fw_hal;

typedef struct {
    const fw_hal *hal;
    drive d;
    link_rx rx;
    uint16_t tx_seq;
    uint64_t state_period_us, next_state_us;
    uint32_t last_tick_us;
    uint16_t cycle_overruns; /* saturating */
    uint32_t state_drops;    /* STATE frames the link could not take */
} fw_app;

/* false for an invalid drive config, boot_id 0 or state_period_ms 0. */
bool fw_app_init(fw_app *app, const fw_hal *hal, const drive_config *cfg, uint32_t state_period_ms,
                 uint32_t boot_id);

/* Order: SPI responses -> inputs -> drive_begin -> received messages -> drive_finish ->
   outputs -> next SPI frame -> STATE -> watchdog. */
void fw_app_tick(fw_app *app);

#endif
