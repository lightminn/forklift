/* Clock tree constants (board design 3.3). Plain numbers so the host tests can check them too;
   clock.c programs the registers from these. Both PLL sources must give the same tree: TIM2's
   1 MHz and the SPI dividers are fixed against it. */
#ifndef BOARD_CLOCK_CONFIG_H
#define BOARD_CLOCK_CONFIG_H

#define CLK_HSE_HZ 25000000u /* WeAct X1 25 MHz crystal */
#define CLK_HSI_HZ 64000000u

/* PLL1 from HSE: 25 / 5 = 5 MHz (input range 4-8 MHz), x160 = 800 MHz VCO. */
#define CLK_PLL_HSE_M 5u
#define CLK_PLL_HSE_N 160u
/* PLL1 from HSI (fallback): 64 / 10 = 6.4 MHz (4-8 MHz), x125 = 800 MHz VCO. */
#define CLK_PLL_HSI_M 10u
#define CLK_PLL_HSI_N 125u
#define CLK_PLL_P 2u /* SYSCLK */
#define CLK_PLL_Q 8u /* SPI1/2/3 kernel */
#define CLK_PLL_R 2u
#define CLK_PLL_RANGE_4_8 2u /* RCC_PLLCFGR.PLL1RGE value for a 4-8 MHz reference */

#define CLK_VCO_HZ 800000000u
#define CLK_SYSCLK_HZ (CLK_VCO_HZ / CLK_PLL_P) /* 400 MHz, VOS1 maximum */
#define CLK_PLL1Q_HZ (CLK_VCO_HZ / CLK_PLL_Q)  /* 100 MHz */
#define CLK_HCLK_HZ (CLK_SYSCLK_HZ / 2u)       /* AXI/AHB 200 MHz */
#define CLK_PCLK_HZ (CLK_HCLK_HZ / 2u)         /* APB1-4 100 MHz */
#define CLK_TIMER_HZ (CLK_PCLK_HZ * 2u)        /* APB timers: x2 when the APB divider is not 1 */
#define CLK_FLASH_LATENCY 2u                   /* VOS1, AXI 200 MHz (RM0433 table 17) */
#define CLK_FLASH_WRHIGHFREQ 2u

/* USB kernel clock from PLL3 Q, 48 MHz from the same reference as PLL1 (PLLSRC is shared): the
   crystal keeps it within USB full speed's +-0.25 % (USB 2.0 7.1.11). HSI48 is not used for the
   USB: untrimmed it is 47.5-48.5 MHz at 30 C and -4.5/+3.5 % over temperature (DS12110 table 46),
   and trimming it from USB SOF through the CRS does not work on rev Y (ES0396 2.2.2). On the HSI
   fallback the USB clock carries the HSI's tolerance: enumeration is best effort there. */
#define CLK_PLL3_HSE_N 96u /* 5 MHz x 96 = 480 MHz VCO */
#define CLK_PLL3_HSI_N 75u /* 6.4 MHz x 75 = 480 MHz VCO */
#define CLK_PLL3_Q 10u     /* 48 MHz */
#define CLK_PLL3_VCO_HZ 480000000u
#define CLK_USB_HZ (CLK_PLL3_VCO_HZ / CLK_PLL3_Q)

#define CLK_US_PRESCALER (CLK_TIMER_HZ / 1000000u) /* TIM2 at 1 MHz */
#define CLK_TICK_HZ 1000u                          /* control period 1 ms (plan 3.3) */

#define CLK_STATIC_ASSERTS                                                                         \
    _Static_assert(CLK_HSE_HZ / CLK_PLL_HSE_M >= 4000000u &&                                       \
                       CLK_HSE_HZ / CLK_PLL_HSE_M <= 8000000u,                                     \
                   "HSE PLL reference outside 4-8 MHz");                                           \
    _Static_assert(CLK_HSI_HZ / CLK_PLL_HSI_M >= 4000000u &&                                       \
                       CLK_HSI_HZ / CLK_PLL_HSI_M <= 8000000u,                                     \
                   "HSI PLL reference outside 4-8 MHz");                                           \
    _Static_assert(CLK_HSE_HZ / CLK_PLL_HSE_M * CLK_PLL_HSE_N == CLK_VCO_HZ, "HSE VCO");           \
    _Static_assert(CLK_HSI_HZ / CLK_PLL_HSI_M * CLK_PLL_HSI_N == CLK_VCO_HZ, "HSI VCO");           \
    _Static_assert(CLK_VCO_HZ >= 192000000u && CLK_VCO_HZ <= 836000000u, "wide VCO range");        \
    _Static_assert(CLK_SYSCLK_HZ <= 400000000u, "VOS1 allows 400 MHz");                            \
    _Static_assert(CLK_HSE_HZ / CLK_PLL_HSE_M * CLK_PLL3_HSE_N == CLK_PLL3_VCO_HZ, "HSE PLL3");    \
    _Static_assert(CLK_HSI_HZ / CLK_PLL_HSI_M * CLK_PLL3_HSI_N == CLK_PLL3_VCO_HZ, "HSI PLL3");    \
    _Static_assert(CLK_PLL3_VCO_HZ >= 192000000u && CLK_PLL3_VCO_HZ <= 836000000u, "PLL3 VCO");    \
    _Static_assert(CLK_USB_HZ == 48000000u && CLK_USB_HZ * CLK_PLL3_Q == CLK_PLL3_VCO_HZ, "USB");  \
    _Static_assert(CLK_US_PRESCALER * 1000000u == CLK_TIMER_HZ, "TIM2 must count whole us")

#endif
