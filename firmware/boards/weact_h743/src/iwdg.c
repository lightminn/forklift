/* Independent watchdog (board design 3.5): LSI 29.76-33.60 kHz, /4, reload 399 -> 47.6-53.8 ms.
   Never frozen by the firmware; option bytes are never written. */
#include "board/board.h"
#include "stm32h7xx.h"

#define IWDG_KEY_START 0xCCCCu
#define IWDG_KEY_ACCESS 0x5555u
#define IWDG_KEY_RELOAD 0xAAAAu
#define IWDG_PR_DIV4 0u
#define IWDG_RELOAD 399u

void iwdg_start(void)
{
    IWDG1->KR = IWDG_KEY_START; /* starts LSI and the counter */
    IWDG1->KR = IWDG_KEY_ACCESS;
    IWDG1->PR = IWDG_PR_DIV4;
    IWDG1->RLR = IWDG_RELOAD;
    /* PR/RLR updates take a few LSI cycles; reloading early keeps the default (longer) timeout. */
    for (uint32_t spins = 0; (IWDG1->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) && spins < 100000u;
         spins++) {
    }
    IWDG1->KR = IWDG_KEY_RELOAD;
}

void iwdg_kick(void)
{
    IWDG1->KR = IWDG_KEY_RELOAD;
}
