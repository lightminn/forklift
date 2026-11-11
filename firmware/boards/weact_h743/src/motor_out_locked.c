/* Motor outputs of a locked build (board design 3.6): the pins only ever take their safe levels.
   One of the two board files that write motor pins (motor_pins.h); TIM1 is poisoned here. */
#include "board/board.h"
#include "board/board_hal.h"
#include "board/motor_pins.h"

void motor_out_safe_init(void)
{
    mo_safe_init();
}

void motor_out_force_safe(void)
{
    mo_safe_levels();
    MO_WR(GPIOB->BSRR, MO_PB_LOW << 16); /* nSLEEP low: drivers asleep */
}

/* Locked: the requested duty and DRVOFF are ignored; every motor pin keeps its safe level. The
   duty the core computed still goes out in STATE (contract 3.3 outputs_locked). */
void motor_out_apply(const drive_outputs *out)
{
    (void)out;
    mo_safe_levels();
#if defined(FW_BUILD_hil)
    MO_WR(GPIOB->BSRR,
          MO_PB_LOW << 16); /* HIL: the drivers are never woken (board design 3.6, 9) */
#else
    MO_WR(GPIOB->BSRR, MO_PB_LOW); /* nSLEEP stays high once woken */
#endif
}

void motor_out_wake_drivers(void)
{
    MO_WR(GPIOB->BSRR, MO_PB_LOW); /* nSLEEP high; DRVOFF high keeps the outputs off */
}
