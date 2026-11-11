#include "board/reset_cause.h"

#include "fw/drive.h"

uint8_t reset_cause_from_rsr(uint32_t rsr)
{
    if (rsr & RSR_IWDG1RSTF) {
        return DRIVE_RESET_IWDG;
    }
    if (rsr & RSR_SFTRSTF) {
        return DRIVE_RESET_SOFTWARE;
    }
    if (rsr & (RSR_PORRSTF | RSR_BORRSTF)) {
        return DRIVE_RESET_POWER;
    }
    if (rsr & RSR_PINRSTF) {
        return DRIVE_RESET_PIN;
    }
    return DRIVE_RESET_OTHER;
}

#ifdef __arm__
#include "stm32h7xx.h"

_Static_assert(RSR_BORRSTF == RCC_RSR_BORRSTF && RSR_PINRSTF == RCC_RSR_PINRSTF &&
                   RSR_PORRSTF == RCC_RSR_PORRSTF && RSR_SFTRSTF == RCC_RSR_SFTRSTF &&
                   RSR_IWDG1RSTF == RCC_RSR_IWDG1RSTF,
               "RCC_RSR bit positions");

uint32_t reset_cause_read_and_clear(void)
{
    uint32_t rsr = RCC->RSR;
    RCC->RSR = RCC_RSR_RMVF; /* the flags accumulate until removed */
    return reset_cause_from_rsr(rsr);
}
#endif
