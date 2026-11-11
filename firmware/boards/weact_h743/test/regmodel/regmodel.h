/* Host register model of the motor pins and TIM1 (motor output design 4.2). */
#ifndef REGMODEL_H
#define REGMODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "board/motor_pins.h"

typedef enum { RM_SLEEP, RM_STANDBY, RM_BRAKE, RM_FWD, RM_REV, RM_INVALID } rm_state;

typedef struct {
    rm_state state;
    double duty; /* EN duty when FWD or REV */
} rm_channel;

typedef enum { RM_EV_DRVOFF_HIGH, RM_EV_DRVOFF_LOW, RM_EV_PH, RM_EV_CCR } rm_kind;

typedef struct {
    rm_kind kind;
    int ch;
    uint32_t value;
} rm_event;

#define RM_LOG_MAX 4096
#define RM_ALLOW_FWD 1u
#define RM_ALLOW_REV 2u

/* Registers at their reset values: GPIO analog, timer off. Clears the log and the allowances. */
void regmodel_reset(void);
/* Directions channel ch may drive in from now on (SLEEP, STANDBY, BRAKE are always allowed). A
   write that leaves any channel driving in a direction not allowed counts as a violation. */
void regmodel_allow(int ch, uint8_t dirs);
/* An update event: preloaded compare values become active and TIM1 SR.UIF is set. */
void regmodel_uev(void);
/* Inject one update event right after the k-th register write from now (k >= 1); 0 cancels. */
void regmodel_uev_after_write(int k);
rm_channel regmodel_channel(int ch);
int regmodel_violations(void);
int regmodel_log(const rm_event **out);
void regmodel_log_clear(void);

#endif
