/* B1 boot sequence (board design 3.2). The control tick does not run the core yet -- the
   peripheral HAL and the link come in B2/B3 -- it refreshes the IWDG and blinks the LED, which
   proves the clock tree, the time base and the watchdog on the bench. */
#include "board/board.h"
#include "build_info.h"
#include "stm32h7xx.h"

/* What this image is, kept in the ELF (board design 3.7). */
__attribute__((section(".build_info"), used)) const char g_build_info[] =
    "forklift-mcu " FW_BUILD_KIND " locked " FW_GIT_DESCRIBE " link=" FW_LINK_KIND
    " cc=" FW_TOOLCHAIN;

static volatile uint32_t g_boot_id;
static volatile uint32_t g_reset_cause;
static volatile clock_source g_clock_source;
static uint32_t g_ticks;

static void on_tick(void)
{
    g_ticks++;
    if (g_ticks % 500u == 0u) {
        led_toggle(); /* 1 Hz heartbeat */
    }
    iwdg_kick(); /* B2/B3: fw_app_tick kicks only after a whole cycle */
}

int board_main(void)
{
    motor_out_safe_init(); /* step 2: before the clocks */
    iwdg_start();
    g_reset_cause = reset_cause_read_and_clear();
    cycles_init();
    led_init();
    g_clock_source = clock_init();
    iwdg_kick();
    timebase_init();

    uint32_t id;
    if (!rng_boot_id(&id)) {
        led_fail_forever(7); /* no healthy RNG: boot failure, the IWDG retries */
    }
    g_boot_id = id;
    iwdg_kick();

    tick_start(on_tick);
    for (;;) {
        __WFI(); /* B3: tud_task() and the link rings */
    }
}
