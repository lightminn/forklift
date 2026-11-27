/* One-slot mailbox from an interrupt to the control tick (board design 3.4): the producer fills
   every field, then publishes `ready` (release); the consumer sees `ready` (acquire), copies, then
   clears it. A new transaction may not start while a result is unconsumed. Header-only and free
   of device headers so the host tests run it. */
#ifndef BOARD_MAILBOX_H
#define BOARD_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__arm__)
#define MAILBOX_BARRIER() __asm__ volatile("dmb" ::: "memory")
#else
#define MAILBOX_BARRIER() __asm__ volatile("" ::: "memory")
#endif

typedef struct {
    volatile uint32_t ready; /* 0 empty, 1 full; 32-bit aligned, written whole */
    volatile uint32_t tag;
    volatile uint16_t value;
    volatile uint8_t channel;
} mailbox;

static inline void mailbox_init(mailbox *m)
{
    m->ready = 0;
}

/* Producer (interrupt). Returns false if the previous result was never taken (it is kept). */
static inline bool mailbox_put(mailbox *m, uint8_t channel, uint16_t value, uint32_t tag)
{
    if (m->ready) {
        return false;
    }
    m->channel = channel;
    m->value = value;
    m->tag = tag;
    MAILBOX_BARRIER();
    m->ready = 1;
    return true;
}

static inline bool mailbox_full(const mailbox *m)
{
    return m->ready != 0;
}

/* Consumer (control tick). */
static inline bool mailbox_take(mailbox *m, uint8_t *channel, uint16_t *value, uint32_t *tag)
{
    if (!m->ready) {
        return false;
    }
    MAILBOX_BARRIER();
    *channel = m->channel;
    *value = m->value;
    *tag = m->tag;
    MAILBOX_BARRIER();
    m->ready = 0;
    return true;
}

/* A sensor read with its acquisition time (bench console plan 4.2 A1b-1): a seqlock over the packed slot and the
   HAL time, so the tick never pairs a frame with another frame's time. The interrupt makes seq odd, writes both,
   then makes it even; the tick reads only an even, unchanged seq. */
typedef struct {
    volatile uint32_t seq;
    volatile uint32_t slot; /* slot_pack(generation, frame) */
    volatile uint32_t time; /* HAL microseconds at the read's completion */
} stamped_slot;

static inline void stamped_put(stamped_slot *s, uint32_t slot, uint32_t time)
{
    s->seq = s->seq + 1u;
    MAILBOX_BARRIER();
    s->slot = slot;
    s->time = time;
    MAILBOX_BARRIER();
    s->seq = s->seq + 1u;
}

/* False while the interrupt is writing (or wrote during the read): no frame this tick. */
static inline bool stamped_read(const stamped_slot *s, uint32_t *slot, uint32_t *time)
{
    uint32_t a = s->seq;
    if (a & 1u) {
        return false;
    }
    MAILBOX_BARRIER();
    *slot = s->slot;
    *time = s->time;
    MAILBOX_BARRIER();
    return s->seq == a;
}

/* A completed sensor read as one 32-bit word: generation (16 bits) | frame (16 bits). The
   interrupt writes the word once; the tick reads it once -- one aligned 32-bit access each, so the
   generation and its frame can never be torn apart (Codex B2a review). */
static inline uint32_t slot_pack(uint16_t generation, uint16_t frame)
{
    return ((uint32_t)generation << 16) | frame;
}

/* Fresh once per new generation (wrap-safe over 16 bits); *frame is the slot's frame either way. */
static inline bool slot_take(uint32_t slot, uint16_t *consumed, uint16_t *frame)
{
    uint16_t generation = (uint16_t)(slot >> 16);
    *frame = (uint16_t)slot;
    if ((int16_t)(uint16_t)(generation - *consumed) > 0) {
        *consumed = generation;
        return true;
    }
    return false;
}

/* The tick's take of a stamped read: fresh once per generation, never one acquired after the tick's time now (it
   stays unconsumed for the next tick: a higher-priority interrupt may publish between the tick's clock read and
   this one); *time is its acquisition time. */
static inline bool stamped_take(const stamped_slot *s, uint16_t *consumed, uint32_t now, uint16_t *frame,
                                uint32_t *time)
{
    uint32_t slot, t;
    if (!stamped_read(s, &slot, &t)) {
        return false;
    }
    if ((int32_t)(t - now) > 0) {
        return false;
    }
    *time = t;
    return slot_take(slot, consumed, frame);
}

#endif
