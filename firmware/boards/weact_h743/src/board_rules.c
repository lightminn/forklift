#include "board/board_config.h"

bool kth_raw_zero_outside_range(const abs_axis_config *c)
{
    abs_axis a;
    abs_axis_init(&a);
    abs_axis_update(&a, c, 0x0000u, true); /* exactly what a stuck-low MISO delivers */
    return a.valid && a.out_of_range;
}

bool board_config_valid(const drive_config *c)
{
    return drive_config_valid(c) && kth_raw_zero_outside_range(&c->steer) &&
           kth_raw_zero_outside_range(&c->lift) && (c->outputs_locked || c->commission_build) &&
           !c->estop_wired;
}
