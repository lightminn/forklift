#ifndef BOARD_BOARD_HAL_H
#define BOARD_BOARD_HAL_H

#include "fw/app.h"

/* Pins and peripherals for the core, then the HAL table (board design 4). */
const fw_hal *board_hal_init(void);

/* Locked build: whatever the core asks, the motor pins stay at their safe levels (board design
   3.6). Wakes the DRV8244s (nSLEEP high, outputs still off by DRVOFF) so their SPI can be checked
   at S2 under the user's VM procedure (decision 2026-11-11). */
void motor_out_apply(const drive_outputs *out);
void motor_out_wake_drivers(void);

#endif
