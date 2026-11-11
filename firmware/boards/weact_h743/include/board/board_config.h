/* The board's drive_config and its checks (board design 3.7). Pure: host-tested. */
#ifndef BOARD_BOARD_CONFIG_H
#define BOARD_BOARD_CONFIG_H

#include <stdbool.h>

#include "fw/abs_axis.h"
#include "fw/drive.h"

/* A KTH7801 frame 0x0000 (angle 0) passes the CRC (both variants), so a MISO stuck low reads as
   angle 0. The board requires that angle to map outside the joint range, so a stuck MISO is
   out_of_range (board design 3.7). Uses the core's own conversion. */
bool kth_raw_zero_outside_range(const abs_axis_config *c);

/* The configuration this build runs with (commission_build from FW_BUILD; outputs locked). */
const drive_config *board_drive_config(void);

/* drive_config_valid plus the board's own rules (raw-zero check on both axes, outputs locked,
   e-stop unwired while the schematic has no net). */
bool board_config_valid(const drive_config *c);

#endif
