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

/* Completion generations (board design 4.5): the producer bumps `done` after storing a result;
   the consumer treats it as fresh once, wrap-safe. */
static inline bool generation_fresh(uint32_t done, uint32_t *consumed)
{
    if ((int32_t)(done - *consumed) > 0) {
        *consumed = done;
        return true;
    }
    return false;
}

#endif
