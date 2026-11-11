/* RCC_RSR -> STATE.reset_cause (board design 3.2). Pure, so it is host-tested; the bit
   positions are RM0433's and checked against the CMSIS header in the MCU build. */
#ifndef BOARD_RESET_CAUSE_H
#define BOARD_RESET_CAUSE_H

#include <stdint.h>

#define RSR_BORRSTF (1u << 21)
#define RSR_PINRSTF (1u << 22)
#define RSR_PORRSTF (1u << 23)
#define RSR_SFTRSTF (1u << 24)
#define RSR_IWDG1RSTF (1u << 26)

/* DRIVE_RESET_* from the flags: IWDG, then software, then power/brown-out, then the pin --
   the pin flag is also set by the other resets, so it decides only when alone. */
uint8_t reset_cause_from_rsr(uint32_t rsr);

#endif
