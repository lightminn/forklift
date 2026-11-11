/* WeAct H743 board layer (board design 2026-11-12). B1: boot, clocks, time base, IWDG, safe
   pins, reset cause, boot id, LED. */
#ifndef BOARD_BOARD_H
#define BOARD_BOARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { CLOCK_FROM_HSE = 0, CLOCK_FROM_HSI = 1 } clock_source;

/* Motor pins to their safe levels: DRVOFF high, EN and IN2 low, nSLEEP low, DRV nCS high,
   board flash nCS high (board design 3.2 step 2). Registers only; callable from fault handlers. */
void motor_out_safe_init(void);
void motor_out_force_safe(void);

void iwdg_start(void);
void iwdg_kick(void);

/* Cycle counter for bounded waits before the time base runs. */
void cycles_init(void);
uint32_t cycles_now(void);
bool cycles_wait_flag(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_cycles);

/* VOS1, PLL1 at 400 MHz from HSE or (if it does not start) HSI, HSI48 for the RNG, CSS on. */
clock_source clock_init(void);

/* PLL3 Q at 48 MHz as the USB kernel clock (board design 3.3); false if PLL3 did not lock. */
bool clock_usb_init(void);

void timebase_init(void); /* TIM2 free-running 1 MHz */
uint32_t timebase_now_us(void);
void tick_start(void (*on_tick)(void)); /* TIM6 at CLK_TICK_HZ */
void tick_stop(void);                   /* IWDG stall test build only (board design 9) */

uint32_t reset_cause_read_and_clear(void);

/* A non-zero boot id from a healthy RNG; false on a clock/seed error or a timeout. */
bool rng_boot_id(uint32_t *out);

void led_init(void);
void led_set(bool on);
void led_toggle(void);
/* Blinks code forever without kicking the IWDG: the watchdog resets the board (board design 3.7).
 */
void led_fail_forever(uint32_t code);

#endif
