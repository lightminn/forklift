/* The vehicle model behind the firmware core's inputs (board design 9): first-order wheels, an
   integrating steering and lift, KTH7801 frames and DRV8244 SPI answers. One copy, two users:
   the host model endpoint (firmware/tools/mcu_model.c) and the MCU's HIL build
   (FW_BUILD=hil). Every value here is synthetic, none is a measured robot value. */
#ifndef FW_PLANT_H
#define FW_PLANT_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/drive.h"

#define PLANT_DRV_REGS 0x0Eu

typedef struct {
    /* Parameters (plant_init, from the drive configuration). */
    double wheel_full_rad_s; /* wheel speed a full duty settles at */
    double counts_per_rev[2];
    abs_axis_config steer_axis, lift_axis;
    /* State. */
    double wheel_rad_s[2], wheel_counts[2];
    double steer_rad, lift_m;
    int enc_flip; /* wheel whose counts run against its motion, -1 none */
    uint8_t regs[DRIVE_CH_COUNT][PLANT_DRV_REGS];
    bool nfault_low[DRIVE_CH_COUNT];
    bool spi_done;
    uint8_t spi_ch;
    uint32_t spi_tag;
    uint16_t spi_rx;
} plant;

/* Full duty reaches 1.25 x the configured wheel limit; time constants and rates below. */
#define PLANT_WHEEL_TAU_S 0.05
#define PLANT_STEER_RAD_S_PER_DUTY 1.0
#define PLANT_LIFT_M_S_PER_DUTY 0.1

/* Wheels at rest, steering and lift at zero held 1 % inside their configured ranges, every driver
   with a latched fault until CLR_FLT (as DRV8244 wakes). */
void plant_init(plant *p, const drive_config *c);

/* Advance by dt seconds under the outputs the core computed (DRVOFF high = no drive). */
void plant_step(plant *p, const drive_outputs *out, double dt);

/* The inputs a board would read now: encoder counters, KTH frames, ADC (zero current), nFAULT. */
void plant_inputs(const plant *p, drive_inputs *in);

/* A KTH7801 frame for a joint position under an axis configuration (the inverse of
   abs_axis_update). */
uint16_t plant_kth_frame(double joint, const abs_axis_config *axis);

/* DRV8244 SPI: writes land in the register model (CLR_FLT releases nFAULT), reads return
   0xC0 status with the register. One transaction in flight, answered on the next take. */
bool plant_spi_start(plant *p, uint8_t ch, uint16_t tx, uint32_t tag);
bool plant_spi_take(plant *p, uint8_t *ch, uint16_t *rx, uint32_t *tag);

#endif
