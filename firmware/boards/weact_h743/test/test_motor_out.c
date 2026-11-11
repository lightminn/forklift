/* motor_out_tim1.c on the host register model (motor output design 3, 4.2): every register write
   is interpreted through RM0433 and DRV8244 table 8-4, and checked against the directions the test
   allows. */
#include <math.h>
#include <string.h>

#include "board/board.h"
#include "board/board_hal.h"
#include "fw_test.h"
#include "regmodel.h"

static drive_outputs outs(float l, float r, float m, float h)
{
    drive_outputs o;
    memset(&o, 0, sizeof o);
    o.duty[0] = l;
    o.duty[1] = r;
    o.duty[2] = m;
    o.duty[3] = h;
    return o;
}

static void all_in(rm_state s)
{
    for (int ch = 0; ch < 4; ch++) {
        CHECK_EQ_U(regmodel_channel(ch).state, s);
    }
}

static void boot(void)
{
    regmodel_reset();
    all_in(RM_SLEEP); /* pins analog: the DRV8244 pull-down holds nSLEEP low */
    motor_out_safe_init();
    all_in(RM_SLEEP);
    motor_out_wake_drivers();
    all_in(RM_STANDBY); /* awake, DRVOFF high */
    regmodel_uev();
    all_in(RM_STANDBY);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* One control cycle: outputs, then the update events of the next millisecond. */
static void cycle(drive_outputs o)
{
    motor_out_apply(&o);
    regmodel_uev();
}

static void drive_and_brake(void)
{
    boot();
    regmodel_allow(0, RM_ALLOW_FWD);
    drive_outputs o = outs(0.5f, 0, 0, 0);
    motor_out_apply(&o);
    CHECK_EQ_U(regmodel_channel(0).state, RM_BRAKE); /* new compare value waits for the UEV */
    regmodel_uev();
    rm_channel c = regmodel_channel(0);
    CHECK_EQ_U(c.state, RM_FWD);
    CHECK(fabs(c.duty - 0.5) < 1e-9);
    CHECK_EQ_U(regmodel_channel(1).state, RM_BRAKE); /* duty 0, DRVOFF low: electric brake */
    cycle(outs(1.5f, 0, 0, 0));                      /* clamped to full */
    CHECK(fabs(regmodel_channel(0).duty - 1.0) < 1e-9);
    cycle(outs(0, 0, 0, 0));
    CHECK_EQ_U(regmodel_channel(0).state, RM_BRAKE);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* Design 3: a direction change takes two cycles, and PH never changes while EN can be high. Only
   the current direction is allowed in the first cycle and only the new one in the second: a PH
   flip under a live compare value would drive the wrong way in between and count. */
static void direction_change_in_two_cycles(void)
{
    boot();
    regmodel_allow(2, RM_ALLOW_FWD);
    cycle(outs(0, 0, 0.4f, 0));
    CHECK_EQ_U(regmodel_channel(2).state, RM_FWD);
    cycle(outs(0, 0, -0.3f, 0));
    CHECK_EQ_U(regmodel_channel(2).state, RM_BRAKE);
    regmodel_allow(2, RM_ALLOW_REV);
    cycle(outs(0, 0, -0.3f, 0));
    rm_channel c = regmodel_channel(2);
    CHECK_EQ_U(c.state, RM_REV);
    CHECK(fabs(c.duty - 0.3) < 1e-9);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* Codex review 2: the next call may come with no update event in between (a pending TIM6 tick
   re-entered at once): the direction change must wait for one, judged by TIM1's own UIF. */
static void direction_change_waits_for_an_update(void)
{
    boot();
    regmodel_allow(0, RM_ALLOW_FWD);
    cycle(outs(0.4f, 0, 0, 0));
    drive_outputs rev = outs(-0.3f, 0, 0, 0);
    motor_out_apply(&rev); /* compare 0 written, no update event follows */
    motor_out_apply(&rev);
    motor_out_apply(&rev);
    CHECK_EQ_U(regmodel_channel(0).state, RM_FWD); /* the old value is still active: no PH flip */
    regmodel_uev();
    regmodel_allow(0, RM_ALLOW_REV);
    motor_out_apply(&rev);
    CHECK_EQ_U(regmodel_channel(0).state, RM_BRAKE);
    regmodel_uev();
    CHECK_EQ_U(regmodel_channel(0).state, RM_REV);
    CHECK_EQ_U(regmodel_violations(), 0);
}

/* An update event may land between any two register writes of a call: PH still never changes
   under a live compare value (the model checks every PH write). */
static void update_event_at_every_write(void)
{
    for (int k = 1; k <= 40; k++) {
        boot();
        regmodel_allow(0, RM_ALLOW_FWD | RM_ALLOW_REV);
        cycle(outs(0.4f, 0, 0, 0));
        drive_outputs rev = outs(-0.3f, 0, 0, 0);
        regmodel_uev_after_write(k);
        motor_out_apply(&rev);
        regmodel_uev_after_write(k);
        motor_out_apply(&rev);
        regmodel_uev_after_write(0);
        regmodel_uev();
        motor_out_apply(&rev);
        regmodel_uev();
        CHECK_EQ_U(regmodel_channel(0).state, RM_REV);
        CHECK_EQ_U(regmodel_violations(), 0);
    }
}

/* Design 3: a cut raises DRVOFF before anything else on that channel, and the channel is off at
   that very write. */
static void cut_raises_drvoff_first(void)
{
    boot();
    regmodel_allow(1, RM_ALLOW_REV);
    cycle(outs(0, -0.6f, 0, 0));
    CHECK_EQ_U(regmodel_channel(1).state, RM_REV);
    regmodel_log_clear();
    drive_outputs o = outs(0, -0.6f, 0, 0);
    o.drvoff[1] = true;
    motor_out_apply(&o); /* REV stays allowed until the channel's own first write (the log) */
    CHECK_EQ_U(regmodel_channel(1).state, RM_STANDBY);
    const rm_event *ev;
    int n = regmodel_log(&ev);
    int first = -1;
    for (int i = 0; i < n; i++) {
        if (ev[i].ch == 1 && first < 0) {
            first = i;
        }
    }
    CHECK(first >= 0 && ev[first].kind == RM_EV_DRVOFF_HIGH);
    CHECK_EQ_U(regmodel_violations(), 0);
    /* Out of a cut with no update event since: lowering DRVOFF would let the old compare value
       drive again, so the channel stays cut until the 0 is active, then drives. */
    regmodel_allow(1, RM_ALLOW_FWD);
    drive_outputs fwd = outs(0, 0.2f, 0, 0);
    motor_out_apply(&fwd);
    CHECK_EQ_U(regmodel_channel(1).state, RM_STANDBY);
    regmodel_uev();
    cycle(fwd);
    CHECK_EQ_U(regmodel_channel(1).state, RM_FWD);
    CHECK_EQ_U(regmodel_violations(), 0);
}

static void fault_handler_cuts_everything(void)
{
    boot();
    for (int ch = 0; ch < 4; ch++) {
        regmodel_allow(ch, RM_ALLOW_FWD);
    }
    cycle(outs(0.3f, 0.3f, 0.3f, 0.3f));
    all_in(RM_FWD);
    for (int ch = 0; ch < 4; ch++) {
        regmodel_allow(ch, 0); /* nothing may drive from the handler's first write on */
    }
    regmodel_log_clear();
    motor_out_force_safe();
    all_in(RM_SLEEP);
    const rm_event *ev;
    CHECK(regmodel_log(&ev) > 0 && ev[0].kind == RM_EV_DRVOFF_HIGH);
    CHECK_EQ_U(regmodel_violations(), 0);
    regmodel_uev();
    all_in(RM_SLEEP);
}

/* The interpreter itself (Codex review 1): a stopped counter is not a cut, PWM mode 2 is refused,
   the output polarity is honoured. */
static void interpreter_follows_rm0433(void)
{
    boot();
    regmodel_allow(0, RM_ALLOW_FWD | RM_ALLOW_REV);
    cycle(outs(0.15f, 0, 0, 0));
    regmodel_tim1.CR1 &= ~TIM_CR1_CEN;
    regmodel_tim1.CNT = 0;
    rm_channel c = regmodel_channel(0);
    CHECK(c.state == RM_FWD && c.duty == 1.0); /* CNT 0 < CCR 1500: EN held high */
    regmodel_tim1.CR1 |= TIM_CR1_CEN;
    regmodel_tim1.CCER |= 1u << 1; /* CC1P */
    CHECK(fabs(regmodel_channel(0).duty - 0.85) < 1e-9);
    regmodel_tim1.CCER &= ~(1u << 1);
    regmodel_tim1.CCMR1 = (regmodel_tim1.CCMR1 & ~(7u << 4)) | (7u << 4); /* PWM mode 2 */
    CHECK_EQ_U(regmodel_channel(0).state, RM_INVALID);
}

int main(void)
{
    drive_and_brake();
    direction_change_in_two_cycles();
    direction_change_waits_for_an_update();
    update_event_at_every_write();
    cut_raises_drvoff_first();
    fault_handler_cuts_everything();
    interpreter_follows_rm0433();
    return FW_TEST_RESULT();
}
