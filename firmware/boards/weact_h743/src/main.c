/* Boot sequence (board design 3.2) and the control tick (3.4). From B2a the tick runs the core:
   fw_app_tick (which takes the DRV8244 answers, reads the inputs, runs the state machine, writes
   the locked outputs and kicks the IWDG only after a whole cycle), then starts the next KTH read.
 */
#include "board/board.h"
#include "board/board_config.h"
#include "board/board_hal.h"
#include "board/drv_spi.h"
#include "board/link.h"
#include "build_info.h"
#include "fw/app.h"
#include "stm32h7xx.h"

#define STATE_PERIOD_MS 10u /* plan 3.3: STATE at 100 Hz */

/* What this image is, kept in the ELF (board design 3.7). */
__attribute__((section(".build_info"), used)) const char g_build_info[] =
    FW_IDENTITY " link=" FW_LINK_KIND " cc=" FW_TOOLCHAIN;

static fw_app g_app;
static volatile clock_source g_clock_source;
static uint32_t g_ticks;

#if defined(FW_TEST_IWDG_STALL_S)
#if !FW_OUTPUTS_LOCKED
#error "the IWDG stall test build needs the outputs locked (lead condition 2026-11-12)"
#endif
_Static_assert(FW_TEST_IWDG_STALL_S >= 1 && FW_TEST_IWDG_STALL_S <= 60, "1..60 s");
#endif

static void on_tick(void)
{
#if defined(FW_TEST_IWDG_STALL_S)
    /* S5 test build (board design 9): N s after boot the tick stops for good -- no cycle, no IWDG
       refresh -- and the watchdog resets the board in 47.6-53.8 ms. The outputs are left as they
       are: whether the reset cuts them is what S5 measures. */
    if (g_ticks >= FW_TEST_IWDG_STALL_S * 1000u) {
        tick_stop();
        return;
    }
#endif
    fw_app_tick(&g_app);
    board_hal_tick_end();
    g_ticks++;
    if (g_ticks % 500u == 0u) {
        led_toggle(); /* 1 Hz heartbeat */
    }
}

int board_main(void)
{
    motor_out_safe_init(); /* step 2: before the clocks */
    bool iwdg_runs_in_debug = iwdg_unfreeze_in_debug();
    iwdg_start();
    uint32_t reset_cause = reset_cause_read_and_clear();
    cycles_init();
    led_init();
    if (!iwdg_runs_in_debug) {
        led_fail_forever(10); /* DBG_IWDG1 would not clear: a debug halt could stop the IWDG */
    }
    g_clock_source = clock_init();
    iwdg_kick();
    timebase_init();

    uint32_t boot_id;
    if (!rng_boot_id(&boot_id)) {
        led_fail_forever(7); /* no healthy RNG: boot failure, the IWDG retries */
    }
    const drive_config *cfg = board_drive_config();
    if (!board_config_valid(cfg)) {
        led_fail_forever(8);
    }
    const fw_hal *hal = board_hal_init(cfg);
    iwdg_kick();
    if (!fw_app_init(&g_app, hal, cfg, STATE_PERIOD_MS, boot_id, (uint8_t)reset_cause)) {
        led_fail_forever(9);
    }
    tick_start(on_tick); /* right after init: the first cycle's gap is not judged (core 5) */
    link_init();         /* after the tick: a transport that hangs starting leaves it running */
    for (;;) {
        link_poll();
        /* Anything the poll missed waits at most one tick: TIM6 wakes the loop every 1 ms. */
        __WFI();
    }
}
