/* The mcu-link transport (board design 5): USB CDC by default, UART7 with FW_LINK=uart7. Both
   keep two rings: received bytes go transport -> control tick, frames go control tick ->
   transport. The control tick only ever touches the rings, so a slow or stuck transport cannot
   delay the cycle, the expiry or the watchdog. */
#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t rx_dropped;      /* bytes lost to a full receive ring (UART only) */
    volatile uint32_t tx_dropped_idle; /* bytes of frames dropped while no host had the port */
    volatile uint32_t uart_errors;     /* overrun, framing, noise */
    volatile uint32_t usb_started;     /* the USB stack came up */
    volatile uint32_t reopened;        /* DTR rising edges */
    volatile uint32_t usb33rdy;        /* PWR_CR3.USB33RDY after start (diagnostic only) */
} link_stats;

extern link_stats g_link_stats;

/* Main context, after the control tick has started: a transport that hangs while starting
   (the USB core reset waits without a bound) leaves the tick and the watchdog running. */
void link_init(void);
/* Main loop. */
void link_poll(void);
/* Control tick. */
size_t link_read(uint8_t *buf, size_t cap);
bool link_write_frame(const uint8_t *frame, size_t len);

#endif
