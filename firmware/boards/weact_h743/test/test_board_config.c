/* The configuration the board actually runs (needs the vehicle header, so built only with
   FW_PYTHON). */
#include "board/board_config.h"
#include "fw_test.h"

int main(void)
{
    const drive_config *c = board_drive_config();
    CHECK(drive_config_valid(c));
    CHECK(board_config_valid(c));
    /* IPROPI on the ADC pins is unprotected on schematic V1.0: outputs stay locked by default
       (plan 5 precondition, lead decision Q6). */
    CHECK(c->outputs_locked);
#if defined(FW_BUILD_hil)
    CHECK(c->hil_plant && !c->commission_build); /* board design 9 */
#elif defined(FW_BUILD_commission)
    CHECK(c->commission_build && !c->hil_plant);
#else
    CHECK(!c->hil_plant && !c->commission_build);
#endif
    CHECK(!c->estop_wired); /* no e-stop net on schematic V1.0 */
    CHECK(kth_raw_zero_outside_range(&c->steer));
    CHECK(kth_raw_zero_outside_range(&c->lift));
    /* Raw zero inside the steering range is refused by the board rules (not by the core). */
    drive_config bad = *c;
    bad.steer.zero_offset_rad = 0.0f;
    CHECK(drive_config_valid(&bad));
    CHECK(!board_config_valid(&bad));
    /* An unlocked configuration is refused by this board build. */
    drive_config open = *c;
    open.outputs_locked = false;
    CHECK(!board_config_valid(&open));
    return FW_TEST_RESULT();
}
