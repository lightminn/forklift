/* Force-included into every board C source of a locked build (board design 3.6, lead decision
   Q6): TIM1 drives the motor PWM, so in a locked build no source may name it. The CMSIS header
   is included first, its macros are undefined, then the names are poisoned: any later use fails
   to compile. Opening the outputs is a separate PR that does not compile this guard in. */
#ifndef BOARD_LOCK_GUARD_H
#define BOARD_LOCK_GUARD_H

#include "stm32h7xx.h"

#undef TIM1
#undef TIM1_BASE
#pragma GCC poison TIM1 TIM1_BASE

#endif
