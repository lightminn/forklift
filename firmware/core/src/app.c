#include "fw/app.h"

#include <string.h>

#define LINK_READ_CHUNK 64u
#define LINK_READ_CHUNKS_PER_TICK 4u

bool fw_app_init(fw_app *app, const fw_hal *hal, const drive_config *cfg, uint32_t state_period_ms,
                 uint32_t boot_id, uint8_t reset_cause)
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
    if (!drive_init(&app->d, cfg, &first, boot_id, reset_cause)) {
        return false;
    }
    app->last_tick_us = first.now_us;
    link_rx_init(&app->rx);
    app->state_period_us = (uint64_t)state_period_ms * 1000u;
    app->next_state_us = app->state_period_us;
#if defined(FW_CORE_BENCH)
    if (hal->link_conn != 0) {
        bool busy = false;
        uint32_t since = 0;
        app->conn_seen = hal->link_conn(hal->ctx, &busy, &since);
    }
#endif
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
#if defined(FW_CORE_BENCH)
    if (app->d.cfg->bench_build) {
        /* The pair goes as one write, so the link takes both or neither (contract 8). */
        uint8_t pair[2u * LINK_WIRE_MAX];
        size_t n1 = link_encode(&m, pair, LINK_WIRE_MAX);
        link_msg b;
        memset(&b, 0, sizeof b);
        b.type = LINK_BENCH_STATE;
        b.seq = (uint16_t)(app->tx_seq + 1u);
        drive_fill_bench_state(&app->d, &b.u.bench_state);
        b.u.bench_state.pair_seq = app->tx_seq;
        b.u.bench_state.dropped_pairs = app->bench_dropped;
        size_t n2 = n1 > 0 ? link_encode(&b, pair + n1, LINK_WIRE_MAX) : 0;
        if (n2 > 0 && app->hal->link_write_frame(app->hal->ctx, pair, n1 + n2)) {
            app->tx_seq = (uint16_t)(app->tx_seq + 2u);
        } else {
            app->state_drops++;
            if (app->bench_dropped < UINT16_MAX) {
                app->bench_dropped++;
            }
        }
        return;
    }
#endif
    uint8_t wire[LINK_WIRE_MAX];
    size_t len = link_encode(&m, wire, sizeof wire);
    if (len > 0 && app->hal->link_write_frame(app->hal->ctx, wire, len)) {
        app->tx_seq++;
    } else {
        app->state_drops++;
    }
}

#if defined(FW_CORE_BENCH)
/* The bench image's connection boundary (contract 8.4), before drive_begin: a change in progress is noted for
   the core (which ends the arming with this tick's time) and no byte is read; once it settles, whatever the ring
   still holds from the old connection and the half frame are dropped. false: read nothing this tick. */
static bool link_boundary(fw_app *app)
{
    const fw_hal *hal = app->hal;
    if (hal->link_conn == 0) {
        return true;
    }
    bool busy = false;
    uint32_t since = 0;
    uint32_t gen = hal->link_conn(hal->ctx, &busy, &since);
    bool changed = gen != app->conn_seen;
    if (busy || changed) {
        drive_note_link_change(&app->d, since, changed && !busy);
    }
    if (busy) {
        return false;
    }
    if (changed) {
        app->conn_seen = gen;
        uint8_t junk[LINK_READ_CHUNK];
        while (hal->link_read(hal->ctx, junk, sizeof junk) == sizeof junk) {
        }
        link_rx_init(&app->rx);
    }
    return true;
}
#endif

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

#if defined(FW_CORE_BENCH)
    /* Before drive_begin, which applies a noted change with this tick's time and before the expiry, comparing
       the change's start with the command deadline for the first cause (contract 8.4). */
    bool read_link = link_boundary(app);
#endif
    drive_inputs in;
    memset(&in, 0, sizeof in);
    in.now_us = now; /* before the read: the HAL stamps and filters the KTH frames by it (plan 4.2 A1b-1) */
    in.cycle_overrun = late;
    hal->read_inputs(hal->ctx, &in);
    drive_begin(&app->d, &in);

    uint8_t buf[LINK_READ_CHUNK];
    unsigned chunks = LINK_READ_CHUNKS_PER_TICK;
#if defined(FW_CORE_BENCH)
    if (!read_link) {
        chunks = 0;
    }
#endif
    for (unsigned k = 0; k < chunks; k++) {
        size_t n = hal->link_read(hal->ctx, buf, sizeof buf);
        for (size_t i = 0; i < n; i++) {
            link_msg m;
            if (link_rx_feed(&app->rx, buf[i], &m)) {
                uint8_t prior_mode = app->d.mode;
                int64_t prior_expiry = (app->d.has_deadline && app->d.mode == DRIVE_MODE_ARMED)
                                           ? (int64_t)app->d.deadline_us
                                           : -1; /* an active command only while armed */
                drive_handle(&app->d, &m);
                if (hal->frame_handled != 0) {
                    hal->frame_handled(hal->ctx, &m, prior_mode, prior_expiry, &app->d);
                }
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
