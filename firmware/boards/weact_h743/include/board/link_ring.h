/* Single-producer single-consumer byte ring between the link transport and the control tick
   (board design 3.4, 5). head and tail are free-running 32-bit counts written whole by their one
   owner: the producer copies its bytes, then publishes head (release); the consumer reads head,
   copies, then gives the space back by moving tail (acquire). A frame is put whole or not at all,
   so a published head always sits on a frame boundary and dropping up to it never cuts a frame.
   Header-only and free of device headers so the host tests run it. */
#ifndef BOARD_LINK_RING_H
#define BOARD_LINK_RING_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__arm__)
#define RING_BARRIER() __asm__ volatile("dmb" ::: "memory")
#else
#define RING_BARRIER() __asm__ volatile("" ::: "memory")
#endif

typedef struct {
    volatile uint32_t head; /* bytes published (producer only) */
    volatile uint32_t tail; /* bytes consumed (consumer only) */
    uint8_t *buf;
    uint32_t mask; /* size - 1; size is a power of two */
} link_ring;

static inline void link_ring_init(link_ring *r, uint8_t *buf, uint32_t size)
{
    r->buf = buf;
    r->mask = size - 1u;
    r->head = 0;
    r->tail = 0;
}

/* Producer: the free space, seen from the producer side. */
static inline uint32_t link_ring_space(const link_ring *r)
{
    uint32_t tail = r->tail;
    RING_BARRIER(); /* the consumer finished reading the bytes before it moved tail */
    return r->mask + 1u - (r->head - tail);
}

static inline void link_ring_copy_in(link_ring *r, uint32_t at, const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        r->buf[(at + i) & r->mask] = p[i];
    }
}

/* Producer: the whole frame or nothing (core design 3: link_write_frame). */
static inline bool link_ring_put_frame(link_ring *r, const uint8_t *p, uint32_t n)
{
    if (n > link_ring_space(r)) {
        return false;
    }
    uint32_t head = r->head;
    link_ring_copy_in(r, head, p, n);
    RING_BARRIER();
    r->head = head + n;
    return true;
}

/* Producer: as many bytes as fit (a byte stream with no frame boundary to keep). */
static inline uint32_t link_ring_put_some(link_ring *r, const uint8_t *p, uint32_t n)
{
    uint32_t space = link_ring_space(r);
    if (n > space) {
        n = space;
    }
    uint32_t head = r->head;
    link_ring_copy_in(r, head, p, n);
    RING_BARRIER();
    r->head = head + n;
    return n;
}

/* Consumer: the readable bytes that sit contiguously at *p (0 when empty). */
static inline uint32_t link_ring_peek(const link_ring *r, const uint8_t **p)
{
    uint32_t head = r->head;
    RING_BARRIER(); /* the producer's bytes before its head */
    uint32_t tail = r->tail;
    uint32_t used = head - tail;
    uint32_t off = tail & r->mask;
    uint32_t contig = r->mask + 1u - off;
    *p = r->buf + off;
    return used < contig ? used : contig;
}

/* Consumer: give back n bytes that were read through link_ring_peek. */
static inline void link_ring_consume(link_ring *r, uint32_t n)
{
    RING_BARRIER(); /* finish reading before the producer may overwrite */
    r->tail = r->tail + n;
}

/* Consumer: copy out up to cap bytes. */
static inline uint32_t link_ring_get(link_ring *r, uint8_t *out, uint32_t cap)
{
    uint32_t got = 0;
    while (got < cap) {
        const uint8_t *p;
        uint32_t n = link_ring_peek(r, &p);
        if (n == 0) {
            break;
        }
        if (n > cap - got) {
            n = cap - got;
        }
        for (uint32_t i = 0; i < n; i++) {
            out[got + i] = p[i];
        }
        link_ring_consume(r, n);
        got += n;
    }
    return got;
}

/* Consumer: drop everything published so far. Frames are published whole, so this ends on a
   frame boundary; a frame the producer is still copying is not published yet and stays. */
static inline uint32_t link_ring_drop_all(link_ring *r)
{
    uint32_t head = r->head;
    RING_BARRIER();
    uint32_t dropped = head - r->tail;
    r->tail = head;
    return dropped;
}

#endif
