#include "fw/app.h"

#include <string.h>

#define LINK_READ_CHUNK 64u
#define LINK_READ_CHUNKS_PER_TICK 4u

bool fw_app_init(fw_app *app, const fw_hal *hal, const drive_config *cfg, uint32_t state_period_ms,
                 uint32_t boot_id)
{
    if (state_period_ms == 0) {
        return false;
    }
    memset(app, 0, sizeof *app);
    app->hal = hal;
    drive_inputs first;
    memset(&first, 0, sizeof first);
    hal->read_inputs(hal->ctx, &first);
    first.now_us = hal->now_us(hal->ctx);
    if (!drive_init(&app->d, cfg, &first, boot_id)) {
        return false;
    }
    app->last_tick_us = first.now_us;
    link_rx_init(&app->rx);
    app->state_period_us = (uint64_t)state_period_ms * 1000u;
    app->next_state_us = app->state_period_us;
    return true;
}

static void send_state(fw_app *app)
{
    link_msg m;
    memset(&m, 0, sizeof m);
    m.type = LINK_STATE;
    m.seq = app->tx_seq;
    drive_fill_state(&app->d, &m.u.state);
    m.u.state.rx_errors = app->rx.errors;
    m.u.state.cycle_overruns = app->cycle_overruns;
    uint8_t wire[LINK_WIRE_MAX];
    size_t len = link_encode(&m, wire, sizeof wire);
    if (len > 0 && app->hal->link_write_frame(app->hal->ctx, wire, len)) {
        app->tx_seq++;
    } else {
        app->state_drops++;
    }
}

void fw_app_tick(fw_app *app)
{
    const fw_hal *hal = app->hal;
    uint32_t now = hal->now_us(hal->ctx);
    bool late = now - app->last_tick_us > 2u * app->d.cfg->period_us;
    app->last_tick_us = now;
    if (late && app->cycle_overruns < UINT16_MAX) {
        app->cycle_overruns++;
    }

    /* Responses first, so the core's lost-transaction check sees them (design 3). */
    uint8_t ch;
    uint32_t tag;
    uint16_t rx;
    while (hal->drv_spi_take(hal->ctx, &ch, &rx, &tag)) {
        drive_spi_result(&app->d, ch, rx, tag);
    }

    drive_inputs in;
    memset(&in, 0, sizeof in);
    hal->read_inputs(hal->ctx, &in);
    in.now_us = now;
    in.cycle_overrun = late;
    drive_begin(&app->d, &in);

    uint8_t buf[LINK_READ_CHUNK];
    for (unsigned k = 0; k < LINK_READ_CHUNKS_PER_TICK; k++) {
        size_t n = hal->link_read(hal->ctx, buf, sizeof buf);
        for (size_t i = 0; i < n; i++) {
            link_msg m;
            if (link_rx_feed(&app->rx, buf[i], &m)) {
                drive_handle(&app->d, &m);
            }
        }
        if (n < sizeof buf) {
            break;
        }
    }

    drive_outputs out;
    drive_finish(&app->d, &out);
    hal->write_outputs(hal->ctx, &out); /* DRVOFF is applied before any SPI frame starts */

    uint16_t frame;
    if (drive_spi_next(&app->d, &ch, &frame, &tag) &&
        hal->drv_spi_start(hal->ctx, ch, frame, tag)) {
        drive_spi_started(&app->d);
    }

    if (app->d.time_us >= app->next_state_us) {
        send_state(app);
        app->next_state_us += app->state_period_us;
        if (app->next_state_us <= app->d.time_us) { /* after a long stall, do not burst */
            app->next_state_us = app->d.time_us + app->state_period_us;
        }
    }
    hal->watchdog_kick(hal->ctx);
}
