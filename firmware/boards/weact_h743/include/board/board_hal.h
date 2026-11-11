#ifndef BOARD_BOARD_HAL_H
#define BOARD_BOARD_HAL_H

#include "fw/app.h"

/* Pins and peripherals for the core, then the HAL table (board design 4); in the HIL build the
   vehicle model instead, set up from cfg (board design 9). */
const fw_hal *board_hal_init(const drive_config *cfg);

/* End of each control cycle, after fw_app_tick: start the next KTH bundle and ADC conversion,
   or (HIL) step the model by one cycle (board design 3.4). */
void board_hal_tick_end(void);

/* Locked build: whatever the core asks, the motor pins stay at their safe levels (board design
   3.6). Wakes the DRV8244s (nSLEEP high, outputs still off by DRVOFF) so their SPI can be checked
   at S2 under the user's VM procedure (decision 2026-11-11). */
void motor_out_apply(const drive_outputs *out);
void motor_out_wake_drivers(void);

/* Wheel encoders (LPTIM1/2) and IPROPI current sense (ADC1 injected), board design 4.3-4.4. */
void encoder_init(void);
void encoder_read(uint16_t hw[2]);
void adc_init(void);
void adc_start(void);
bool adc_take(uint32_t raw[4]);

#endif
