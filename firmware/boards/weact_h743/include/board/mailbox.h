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

#endif
