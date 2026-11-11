/* VOS1 and the 400 MHz clock tree from HSE or, failing that, HSI (board design 3.3). Every wait
   is bounded by the cycle counter; the IWDG is refreshed between stages (design 3.5). */
#include "board/board.h"
#include "board/clock_config.h"
#include "stm32h7xx.h"

CLK_STATIC_ASSERTS;

/* At most 10 ms per wait on the 64 MHz reset clock (the cycle counter runs at SYSCLK). */
#define WAIT_CYCLES (CLK_HSI_HZ / 100u)

void cycles_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u; /* Cortex-M7 DWT lock */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cycles_now(void)
{
    return DWT->CYCCNT;
}

bool cycles_wait_flag(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_cycles)
{
    uint32_t t0 = cycles_now();
    while ((*reg & mask) != mask) {
        if (cycles_now() - t0 > timeout_cycles) {
            return false;
        }
    }
    return true;
}

static void stop(uint32_t code)
{
    led_fail_forever(code); /* no IWDG kick: the watchdog resets and boot retries */
}

static void vos1(void)
{
    PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS) | (3u << PWR_D3CR_VOS_Pos); /* VOS1 */
    (void)PWR->D3CR;
    if (!cycles_wait_flag(&PWR->D3CR, PWR_D3CR_VOSRDY, WAIT_CYCLES)) {
        stop(2);
    }
}

static void pll1_start(uint32_t src, uint32_t m, uint32_t n)
{
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1)) | src |
                     (m << RCC_PLLCKSELR_DIVM1_Pos);
    RCC->PLLCFGR =
        (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL1RGE | RCC_PLLCFGR_PLL1VCOSEL | RCC_PLLCFGR_PLL1FRACEN)) |
        (CLK_PLL_RANGE_4_8 << RCC_PLLCFGR_PLL1RGE_Pos) | RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN |
        RCC_PLLCFGR_DIVR1EN; /* wide VCO (VCOSEL = 0) */
    RCC->PLL1DIVR = ((n - 1u) << RCC_PLL1DIVR_N1_Pos) | ((CLK_PLL_P - 1u) << RCC_PLL1DIVR_P1_Pos) |
                    ((CLK_PLL_Q - 1u) << RCC_PLL1DIVR_Q1_Pos) |
                    ((CLK_PLL_R - 1u) << RCC_PLL1DIVR_R1_Pos);
    RCC->CR |= RCC_CR_PLL1ON;
}

/* PLL3 from the reference PLL1 runs on (PLLSRC is shared), Q output only (clock_config.h). Main
   context after the tick started: the wait does not hold the control cycle. */
bool clock_usb_init(void)
{
    bool hse = (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC) == RCC_PLLCKSELR_PLLSRC_HSE;
    uint32_t m = hse ? CLK_PLL_HSE_M : CLK_PLL_HSI_M;
    uint32_t n = hse ? CLK_PLL3_HSE_N : CLK_PLL3_HSI_N;
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~RCC_PLLCKSELR_DIVM3) | (m << RCC_PLLCKSELR_DIVM3_Pos);
    RCC->PLLCFGR =
        (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL3RGE | RCC_PLLCFGR_PLL3VCOSEL | RCC_PLLCFGR_PLL3FRACEN |
                          RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVR3EN)) |
        (CLK_PLL_RANGE_4_8 << RCC_PLLCFGR_PLL3RGE_Pos) | RCC_PLLCFGR_DIVQ3EN;
    RCC->PLL3DIVR = ((n - 1u) << RCC_PLL3DIVR_N3_Pos) | ((2u - 1u) << RCC_PLL3DIVR_P3_Pos) |
                    ((CLK_PLL3_Q - 1u) << RCC_PLL3DIVR_Q3_Pos) | ((2u - 1u) << RCC_PLL3DIVR_R3_Pos);
    RCC->CR |= RCC_CR_PLL3ON;
    if (!cycles_wait_flag(&RCC->CR, RCC_CR_PLL3RDY, CLK_SYSCLK_HZ / 100u)) {
        return false;
    }
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL) | (2u << RCC_D2CCIP2R_USBSEL_Pos);
    return true;
}

clock_source clock_init(void)
{
    vos1();
    iwdg_kick();

    clock_source source = CLOCK_FROM_HSE;
    RCC->CR |= RCC_CR_HSEON;
    if (cycles_wait_flag(&RCC->CR, RCC_CR_HSERDY, WAIT_CYCLES)) {
        pll1_start(RCC_PLLCKSELR_PLLSRC_HSE, CLK_PLL_HSE_M, CLK_PLL_HSE_N);
    } else {
        RCC->CR &= ~RCC_CR_HSEON;
        source = CLOCK_FROM_HSI;
        pll1_start(RCC_PLLCKSELR_PLLSRC_HSI, CLK_PLL_HSI_M, CLK_PLL_HSI_N);
    }
    if (!cycles_wait_flag(&RCC->CR, RCC_CR_PLL1RDY, WAIT_CYCLES)) {
        stop(3);
    }
    iwdg_kick();

    /* Flash wait states before the clock rises (RM0433 table 17: VOS1, AXI 200 MHz). */
    FLASH->ACR = (CLK_FLASH_LATENCY << FLASH_ACR_LATENCY_Pos) |
                 (CLK_FLASH_WRHIGHFREQ << FLASH_ACR_WRHIGHFREQ_Pos);
    if ((FLASH->ACR & (FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ)) !=
        ((CLK_FLASH_LATENCY << FLASH_ACR_LATENCY_Pos) |
         (CLK_FLASH_WRHIGHFREQ << FLASH_ACR_WRHIGHFREQ_Pos))) {
        stop(4);
    }
    RCC->D1CFGR = RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
    RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
    RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    if (!cycles_wait_flag(&RCC->CFGR, RCC_CFGR_SWS_PLL1, WAIT_CYCLES)) {
        stop(5);
    }
    iwdg_kick();

    /* HSI48 for the RNG (board design 3.3). The USB runs from PLL3 (clock_usb_init). */
    RCC->CR |= RCC_CR_HSI48ON;
    if (!cycles_wait_flag(&RCC->CR, RCC_CR_HSI48RDY, CLK_SYSCLK_HZ / 100u)) {
        stop(6);
    }
    if (source == CLOCK_FROM_HSE) {
        RCC->CR |= RCC_CR_CSSHSEON; /* losing HSE now raises NMI (vectors.c) */
    }
    SCB_EnableICache();
    return source;
}
