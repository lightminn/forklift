/* Boot id from the hardware RNG (board design 3.7, contract 2). An unhealthy RNG is a boot
   failure: no deterministic fallback, because a repeated id would let an old ARM/DRIVE pass. */
#include "board/board.h"
#include "board/clock_config.h"
#include "stm32h7xx.h"

#define RNG_WAIT_CYCLES (CLK_SYSCLK_HZ / 100u) /* 10 ms */

bool rng_boot_id(uint32_t *out)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN; /* kernel clock: HSI48 (RNGSEL reset value) */
    (void)RCC->AHB2ENR;
    RNG->CR = RNG_CR_RNGEN;
    for (int attempt = 0; attempt < 4; attempt++) {
        if (!cycles_wait_flag(&RNG->SR, RNG_SR_DRDY, RNG_WAIT_CYCLES)) {
            return false;
        }
        if (RNG->SR & (RNG_SR_CECS | RNG_SR_SECS | RNG_SR_CEIS | RNG_SR_SEIS)) {
            return false;
        }
        uint32_t v = RNG->DR;
        if (v != 0u) {
            *out = v;
            return true;
        }
    }
    return false;
}
