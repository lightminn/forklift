/* Motor outputs of an output build (motor output design, FW_MOTOR_OUTPUT=1): TIM1 PWM on EN/IN1,
   PH/IN2 and DRVOFF as GPIO, DRV8244 in PH/EN mode (SLVSG24C table 8-4). Configured only behind
   the IPROPI protection record gate (design 2); opening the outputs on the vehicle is the user's
   decision after that protection is confirmed (lead decision Q6) -- building this file is not.
   The only board file that names TIM1; every other board source has it poisoned. */
#include "board/board.h"
#include "board/board_hal.h"
#include "board/clock_config.h"
#include "board/motor_pins.h"

#define PWM_TOP 10000u /* ARR + 1: 200 MHz / 10000 = 20 kHz (DRV8244 up to 25 kHz) */
#define PWM_MODE1 6u   /* OCxM = 0110: active while CNT < CCR (RM0433 TIMx_CCMR1) */

_Static_assert(CLK_TIMER_HZ / PWM_TOP == 20000u, "TIM1 at 20 kHz");

/* Per channel: the PH level last written (+1 high, -1 low), the compare value last written, and
   whether a 0 written there still waits for an update event. "The previous call" does not prove
   one passed -- a pending TIM6 tick re-enters at once (Codex review 2) -- so TIM1's own UIF does:
   it is read at the start of a call and cleared at its end, after every write of the call. */
static int8_t g_ph[DRIVE_CH_COUNT] = {-1, -1, -1, -1};
static uint32_t g_ccr[DRIVE_CH_COUNT];
static bool g_zero_pending[DRIVE_CH_COUNT];
static bool g_cut[DRIVE_CH_COUNT] = {true, true, true, true}; /* DRVOFF high */

static volatile uint32_t *ccr_reg(int ch)
{
    return ch == 0 ? &TIM1->CCR1 : ch == 1 ? &TIM1->CCR2 : ch == 2 ? &TIM1->CCR3 : &TIM1->CCR4;
}

static void set_ccr(int ch, uint32_t v)
{
    if (v == 0u && g_ccr[ch] != 0u) {
        g_zero_pending[ch] = true;
    }
    MO_WR(*ccr_reg(ch), v);
    g_ccr[ch] = v;
}

static void drvoff(int ch, bool high)
{
    uint32_t pin = MO_PIN(mo_drvoff_pin(ch));
    MO_WR(GPIOD->BSRR, high ? pin : pin << 16);
    g_cut[ch] = high;
}

static void ph(int ch, int8_t dir)
{
    uint32_t pin = MO_PIN(mo_ph_pin(ch));
    MO_WR(GPIOE->BSRR, dir > 0 ? pin : pin << 16);
    g_ph[ch] = dir;
}

void motor_out_safe_init(void)
{
    mo_safe_init();
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) { /* what the safe levels just wrote */
        g_ph[ch] = -1;
        g_ccr[ch] = 0u;
        g_zero_pending[ch] = false;
        g_cut[ch] = true;
    }
}

/* Faults (NMI, HardFault, unexpected IRQ): registers only. DRVOFF high first, then the timer's
   outputs off and the compare values 0, then the drivers asleep. */
void motor_out_force_safe(void)
{
    MO_WR(GPIOD->BSRR, MO_PD_DRVOFF);
    MO_WR(TIM1->BDTR, TIM1->BDTR & ~TIM_BDTR_MOE);
    MO_WR(TIM1->CCR1, 0u);
    MO_WR(TIM1->CCR2, 0u);
    MO_WR(TIM1->CCR3, 0u);
    MO_WR(TIM1->CCR4, 0u);
    mo_safe_levels();
    MO_WR(GPIOB->BSRR, MO_PB_LOW << 16);
}

/* Wake the drivers (DRVOFF still high) and start TIM1 with every compare value 0, then hand the
   EN pins from GPIO low to the timer: at that moment the timer's outputs are already low. */
void motor_out_wake_drivers(void)
{
    MO_WR(GPIOB->BSRR, MO_PB_LOW); /* nSLEEP high */
    MO_WR(RCC->APB2ENR, RCC->APB2ENR | RCC_APB2ENR_TIM1EN);
    (void)RCC->APB2ENR;
    /* Debug halt: TIM1 frozen and its outputs disabled (RM0433 DBGMCU APB2FZ1). */
    MO_WR(DBGMCU->APB2FZ1, DBGMCU->APB2FZ1 | DBGMCU_APB2FZ1_DBG_TIM1);
    MO_WR(TIM1->CR1, TIM_CR1_ARPE); /* up-counting, edge aligned, counter stopped */
    MO_WR(TIM1->PSC, 0u);
    MO_WR(TIM1->ARR, PWM_TOP - 1u);
    MO_WR(TIM1->CCMR1, (PWM_MODE1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                           (PWM_MODE1 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE);
    MO_WR(TIM1->CCMR2, (PWM_MODE1 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                           (PWM_MODE1 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        set_ccr(ch, 0u);
    }
    MO_WR(TIM1->CR2, 0u); /* OISx = 0: idle (MOE off) outputs low */
    MO_WR(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    MO_WR(TIM1->EGR, TIM_EGR_UG); /* preloads to the active registers before the start */
    MO_WR(TIM1->BDTR, TIM_BDTR_OSSR | TIM_BDTR_OSSI | TIM_BDTR_MOE); /* AOE = 0, no dead time */
    MO_WR(TIM1->CR1, TIM_CR1_ARPE | TIM_CR1_CEN);
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        uint32_t n = mo_en_pin(ch), r = n >> 3, s = (n & 7u) * 4u;
        MO_WR(GPIOE->AFR[r], (GPIOE->AFR[r] & ~(0xFu << s)) | (1u << s)); /* AF1 = TIM1_CHx */
        MO_WR(GPIOE->MODER, (GPIOE->MODER & ~(3u << (2u * n))) | (2u << (2u * n)));
    }
}

/* Each cycle (design 3): a cut raises DRVOFF first; a drive lowers it last; a direction change
   writes compare 0 with PH unchanged and changes PH only in a later call that sees an update event
   after that 0 (UIF) -- then the new PH and compare value. duty 0 with DRVOFF low is the electric
   brake (EN low, OUT1 = OUT2 = H). */
void motor_out_apply(const drive_outputs *out)
{
    bool updated = (TIM1->SR & TIM_SR_UIF) != 0u; /* an update since the end of the last call */
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (updated) {
            g_zero_pending[ch] = false;
        }
    }
    for (int ch = 0; ch < DRIVE_CH_COUNT; ch++) {
        if (out->drvoff[ch] || (g_cut[ch] && g_zero_pending[ch])) {
            /* A cut, or a cut channel whose 0 is not active yet: lowering DRVOFF now would let the
               old compare value drive again until the next update event -- stay cut. */
            drvoff(ch, true);
            set_ccr(ch, 0u);
            continue;
        }
        float d = out->duty[ch];
        float mag = d < 0.0f ? -d : d;
        uint32_t ccr = mag >= 1.0f ? PWM_TOP : (uint32_t)(mag * (float)PWM_TOP + 0.5f);
        int8_t dir = d < 0.0f ? -1 : 1;
        if (ccr == 0u) {
            set_ccr(ch, 0u);
        } else if (dir != g_ph[ch] && (g_ccr[ch] != 0u || g_zero_pending[ch])) {
            set_ccr(ch, 0u); /* direction change: brake until the 0 is active */
        } else {
            if (dir != g_ph[ch]) {
                ph(ch, dir); /* the active compare value is 0: EN is low now */
            }
            set_ccr(ch, ccr);
        }
        drvoff(ch, false);
    }
    MO_WR(TIM1->SR, ~TIM_SR_UIF); /* rc_w0: clears UIF only */
}
