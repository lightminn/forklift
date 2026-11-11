/* Host register model of the motor pins and TIM1 (motor output design 4.2), interpreted through
   RM0433 (GPIO modes, TIM1 output control) and DRV8244 table 8-4 (PH/EN). motor_out_*.c write
   through MO_WR, which calls regmodel_after_write after every store: BSRR is folded into ODR, the
   preload/active compare values are tracked, every channel's state is computed and checked against
   what the test allows, and the write is logged. */
#include "regmodel.h"

#include <string.h>

GPIO_TypeDef regmodel_gpio[4];
TIM_TypeDef regmodel_tim1;
RCC_TypeDef regmodel_rcc;
DBGMCU_TypeDef regmodel_dbgmcu;

static uint32_t g_active[4]; /* active (shadow) compare values */
static uint8_t g_allow[4];   /* allowed drive directions per channel: bit0 FWD, bit1 REV */
static int g_violations;
static rm_event g_log[RM_LOG_MAX];
static int g_log_n;
static uint32_t g_prev_odr[4];
static uint32_t g_prev_ccr[4];
static int g_uev_in; /* writes left before an injected update event, 0 = none */

static volatile uint32_t *ccr(int ch)
{
    return ch == 0   ? &regmodel_tim1.CCR1
           : ch == 1 ? &regmodel_tim1.CCR2
           : ch == 2 ? &regmodel_tim1.CCR3
                     : &regmodel_tim1.CCR4;
}

static uint32_t ccmr_field(int ch)
{
    uint32_t r = ch < 2 ? regmodel_tim1.CCMR1 : regmodel_tim1.CCMR2;
    return (r >> ((ch & 1) ? 8u : 0u)) & 0xFFu;
}

static bool preload(int ch)
{
    return (ccmr_field(ch) & (1u << 3)) != 0u;
}

void regmodel_reset(void)
{
    memset(regmodel_gpio, 0, sizeof regmodel_gpio);
    /* RM0433 11.3.1: after reset most pins are analog (MODER 11). */
    for (int p = 0; p < 4; p++) {
        regmodel_gpio[p].MODER = 0xFFFFFFFFu;
    }
    memset(&regmodel_tim1, 0, sizeof regmodel_tim1);
    memset(&regmodel_rcc, 0, sizeof regmodel_rcc);
    memset(&regmodel_dbgmcu, 0, sizeof regmodel_dbgmcu);
    memset(g_active, 0, sizeof g_active);
    memset(g_prev_odr, 0, sizeof g_prev_odr);
    memset(g_prev_ccr, 0, sizeof g_prev_ccr);
    for (int ch = 0; ch < 4; ch++) {
        g_allow[ch] = 0;
    }
    g_violations = 0;
    g_log_n = 0;
    g_uev_in = 0;
}

void regmodel_uev_after_write(int k)
{
    g_uev_in = k;
}

void regmodel_allow(int ch, uint8_t dirs)
{
    g_allow[ch] = dirs;
}

void regmodel_uev(void)
{
    for (int ch = 0; ch < 4; ch++) {
        g_active[ch] = *ccr(ch);
    }
    if (regmodel_tim1.CR1 & TIM_CR1_CEN) {
        regmodel_tim1.SR |= TIM_SR_UIF;
    }
}

/* Pin level: 0 or 1 when driven; RM_FLOAT when floating (input or analog mode). */
static int gpio_mode(int port, uint32_t pin)
{
    return (int)((regmodel_gpio[port].MODER >> (2u * pin)) & 3u);
}

/* TIM1 channel output as a duty fraction (0..1), or -1 for an invalid configuration. */
static double tim_out(int ch)
{
    const TIM_TypeDef *t = &regmodel_tim1;
    if (!(regmodel_rcc.APB2ENR & RCC_APB2ENR_TIM1EN)) {
        return 0.0; /* timer in reset: its outputs inactive */
    }
    bool ccxe = (t->CCER >> (4u * (uint32_t)ch)) & 1u;
    bool ccxp = (t->CCER >> (4u * (uint32_t)ch + 1u)) & 1u;
    bool ois = (t->CR2 >> (8u + 2u * (uint32_t)ch)) & 1u;
    if (!(t->BDTR & TIM_BDTR_MOE)) {
        /* MOE = 0: OSSI = 1 drives the idle level OISx; OSSI = 0 releases the pin (pull-down). */
        return (t->BDTR & TIM_BDTR_OSSI) && ois ? 1.0 : 0.0;
    }
    if (!ccxe) {
        return 0.0; /* OSSR = 1: inactive level (polarity 0); OSSR = 0: released */
    }
    uint32_t mode = (ccmr_field(ch) >> 4) & 7u;
    if (mode != 6u) {
        return -1.0; /* only PWM mode 1 is allowed (design 3) */
    }
    double top = (double)t->ARR + 1.0;
    double ref;
    if (t->CR1 & TIM_CR1_CEN) {
        ref = g_active[ch] >= t->ARR + 1u ? 1.0 : (double)g_active[ch] / top;
    } else {
        ref = t->CNT < g_active[ch] ? 1.0 : 0.0; /* a stopped counter is not a cut */
    }
    return ccxp ? 1.0 - ref : ref;
}

static int level(int port, uint32_t pin, int floating)
{
    int m = gpio_mode(port, pin);
    if (m == 1) {
        return (int)((regmodel_gpio[port].ODR >> pin) & 1u);
    }
    return floating; /* input or analog: the DRV8244 internal pull decides */
}

rm_channel regmodel_channel(int ch)
{
    rm_channel c = {RM_SLEEP, 0.0};
    int nsleep = level(1, MO_NSLEEP_PIN, 0); /* internal pull-down */
    if (nsleep == 0) {
        return c;
    }
    int off = level(2, mo_drvoff_pin(ch), 1); /* internal pull-up */
    if (off) {
        c.state = RM_STANDBY;
        return c;
    }
    uint32_t en_pin = mo_en_pin(ch);
    double en;
    int m = gpio_mode(3, en_pin);
    if (m == 2) {
        uint32_t af = (regmodel_gpio[3].AFR[en_pin >> 3] >> ((en_pin & 7u) * 4u)) & 0xFu;
        en = af == 1u ? tim_out(ch) : 0.0;
    } else {
        en = (double)level(3, en_pin, 0);
    }
    if (en < 0.0) {
        c.state = RM_INVALID;
        return c;
    }
    if (en == 0.0) {
        c.state = RM_BRAKE;
        return c;
    }
    c.state = level(3, mo_ph_pin(ch), 0) ? RM_FWD : RM_REV;
    c.duty = en;
    return c;
}

static void log_event(rm_kind k, int ch, uint32_t v)
{
    if (g_log_n < RM_LOG_MAX) {
        g_log[g_log_n++] = (rm_event){k, ch, v};
    }
}

void regmodel_after_write(void)
{
    /* BSRR: set bits win over reset bits (RM0433 GPIO BSRR); the register reads 0 again. */
    for (int p = 0; p < 4; p++) {
        uint32_t b = regmodel_gpio[p].BSRR;
        if (b) {
            regmodel_gpio[p].ODR = (regmodel_gpio[p].ODR & ~(b >> 16)) | (b & 0xFFFFu);
            regmodel_gpio[p].BSRR = 0;
        }
    }
    if (regmodel_tim1.EGR & TIM_EGR_UG) {
        regmodel_uev();
        regmodel_tim1.EGR = 0;
    }
    for (int ch = 0; ch < 4; ch++) {
        if (!preload(ch)) {
            g_active[ch] = *ccr(ch);
        }
        uint32_t d = MO_PIN(mo_drvoff_pin(ch)), ph = MO_PIN(mo_ph_pin(ch));
        if ((regmodel_gpio[2].ODR ^ g_prev_odr[2]) & d) {
            log_event((regmodel_gpio[2].ODR & d) ? RM_EV_DRVOFF_HIGH : RM_EV_DRVOFF_LOW, ch, 0);
        }
        if ((regmodel_gpio[3].ODR ^ g_prev_odr[3]) & ph) {
            log_event(RM_EV_PH, ch, (regmodel_gpio[3].ODR & ph) ? 1u : 0u);
            /* Codex review 2: PH may change only while that channel's EN is low (or it is off). */
            rm_channel now = regmodel_channel(ch);
            if (now.state == RM_FWD || now.state == RM_REV) {
                g_violations++;
            }
        }
        if (*ccr(ch) != g_prev_ccr[ch]) {
            log_event(RM_EV_CCR, ch, *ccr(ch));
            g_prev_ccr[ch] = *ccr(ch);
        }
        rm_channel c = regmodel_channel(ch);
        if (c.state == RM_INVALID || (c.state == RM_FWD && !(g_allow[ch] & RM_ALLOW_FWD)) ||
            (c.state == RM_REV && !(g_allow[ch] & RM_ALLOW_REV))) {
            g_violations++;
        }
    }
    for (int p = 0; p < 4; p++) {
        g_prev_odr[p] = regmodel_gpio[p].ODR;
    }
    if (g_uev_in > 0 && --g_uev_in == 0) {
        regmodel_uev();
    }
}

int regmodel_violations(void)
{
    return g_violations;
}

int regmodel_log(const rm_event **out)
{
    *out = g_log;
    return g_log_n;
}

void regmodel_log_clear(void)
{
    g_log_n = 0;
}
