/* Pure pieces of the link (board design 5); host-tested. The transports (link_usb.c,
   link_uart7.c) call these with their device operations. */
#ifndef BOARD_LINK_CFG_H
#define BOARD_LINK_CFG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board/link_ring.h"

/* Board design 5: receive 512 B, send 1024 B (a 92 B mcu-link/3 STATE times 10 cycles, with room). */
#define LINK_RX_RING_SIZE 512u
#define LINK_TX_RING_SIZE 1024u
_Static_assert((LINK_RX_RING_SIZE & (LINK_RX_RING_SIZE - 1u)) == 0, "power of two");
_Static_assert((LINK_TX_RING_SIZE & (LINK_TX_RING_SIZE - 1u)) == 0, "power of two");

/* UART7 alternative: 460800 bps 8N1 (contract 5: 115200 is 69 % busy with STATE alone). */
#define LINK_UART_BAUD 460800u

/* USART BRR for oversampling by 16, rounded to the nearest divider. */
static inline uint32_t uart_brr(uint32_t kernel_hz, uint32_t baud)
{
    return (kernel_hz + baud / 2u) / baud;
}

/* Baud error of that divider in parts per million (signed). */
static inline int32_t uart_baud_error_ppm(uint32_t kernel_hz, uint32_t baud)
{
    int64_t actual = (int64_t)kernel_hz * 1000000 / uart_brr(kernel_hz, baud);
    return (int32_t)((actual - (int64_t)baud * 1000000) / (int64_t)baud);
}

/* USB serial number: the 96-bit device UID as 24 upper-case hex digits, word 0 first, each word
   most significant nibble first. out holds 25 bytes. */
static inline void usb_serial_hex(const uint32_t uid[3], char out[25])
{
    static const char hex[] = "0123456789ABCDEF";
    for (int w = 0; w < 3; w++) {
        for (int k = 0; k < 8; k++) {
            out[w * 8 + k] = hex[(uid[w] >> (28 - 4 * k)) & 0xFu];
        }
    }
    out[24] = '\0';
}

/* A USB string descriptor from ASCII: desc[0] = length and type, then UTF-16LE code units. A
   descriptor is at most 255 bytes, so at most 126 characters; longer strings (a long git
   description in the product string) are cut. cap_units counts desc[] elements. Returns the
   characters written. */
#define USB_STRING_MAX_CHARS 126u
static inline uint32_t usb_string_desc(const char *s, uint16_t *desc, uint32_t cap_units)
{
    uint32_t max = cap_units - 1u;
    if (max > USB_STRING_MAX_CHARS) {
        max = USB_STRING_MAX_CHARS;
    }
    uint32_t n = 0;
    while (n < max && s[n] != '\0') {
        desc[1 + n] = (uint16_t)(uint8_t)s[n];
        n++;
    }
    desc[0] = (uint16_t)((0x03u << 8) | (2u * n + 2u)); /* TUSB_DESC_STRING */
    return n;
}

/* The device side of a CDC port, as the send pump sees it. There is deliberately no "clear":
   with the DWC2 slave port (CFG_TUD_EDPT_DEDICATED_HWFIFO) an IN transfer is scheduled over the
   whole software FIFO and the interrupt reads it in place, so clearing it under an active
   transfer corrupts that transfer (Codex B3 review 2). */
typedef struct {
    void *ctx;
    bool (*connected)(void *ctx);                               /* host has the port open (DTR) */
    uint32_t (*room)(void *ctx);                                /* bytes the stack accepts now */
    uint32_t (*write)(void *ctx, const uint8_t *p, uint32_t n); /* returns bytes taken */
    void (*flush)(void *ctx);
} link_sink;

/* Main loop (the send ring's consumer).
   - While no host has the port open, frames are dropped as they come and *resync is set: what
     the stack already holds (at most its FIFO, CFG_TUD_CDC_TX_BUFSIZE) is the only old data a
     reopened port can see (board design 5.1).
   - With the port open and *resync set (set here, or by the transport on any connection
     life-cycle event), one 0x00 delimiter goes first: the old bytes may end inside a frame, and
     the delimiter closes them as one broken frame instead of letting them swallow the first
     fresh one. Empty frames are not errors on either decoder (core link_rx_feed,
     forklift_core.mcu_link.framing). No frame is sent before the delimiter is taken.
   - Then as much as the stack accepts moves on, frames possibly split across calls; the bytes
     stay in order.
   Returns the bytes dropped from the ring. */
static inline uint32_t link_tx_pump(link_ring *tx, const link_sink *s, bool *resync)
{
    if (!s->connected(s->ctx)) {
        *resync = true;
        return link_ring_drop_all(tx);
    }
    bool moved = false;
    if (*resync) {
        static const uint8_t delimiter = 0x00;
        if (s->room(s->ctx) == 0 || s->write(s->ctx, &delimiter, 1) != 1) {
            return 0;
        }
        *resync = false;
        moved = true;
    }
    for (;;) {
        const uint8_t *p;
        uint32_t n = link_ring_peek(tx, &p);
        uint32_t room = s->room(s->ctx);
        if (n > room) {
            n = room;
        }
        if (n == 0) {
            break;
        }
        uint32_t took = s->write(s->ctx, p, n);
        link_ring_consume(tx, took);
        moved = true;
        if (took < n) {
            break;
        }
    }
    if (moved) {
        s->flush(s->ctx);
    }
    return 0;
}

#endif
