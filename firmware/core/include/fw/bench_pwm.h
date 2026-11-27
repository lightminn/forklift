/* Bench image (bench console plan 4.3 A1b-2 decision 6): what the motor output HAL actually wrote and what may be
   active, reported to the next tick's drive_inputs.

   The board HAL (motor_out_tim1.c) keeps its register code and calls bench_pwm_wrote after every apply and
   bench_pwm_read at read_inputs; the host model and the tests run bench_pwm_model, the same register rules as a pure
   function, and a register-model test holds the two to the same values. These values describe the TIM1 PWM state
   only, never what the DRV8244 bridge delivered (plan 4.3 decision 6, meaning). Compiled only with FW_CORE_BENCH. */
#ifndef FW_BENCH_PWM_H
#define FW_BENCH_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/drive.h"

#if defined(FW_CORE_BENCH)

#define BENCH_PWM_TOP 10000u /* compare value of duty 1 (TIM1 ARR + 1) */

typedef struct {
    int16_t w[DRIVE_CH_COUNT];      /* the compare value the last apply wrote, physical sign (0: a 0 or a cut) */
    bool cut[DRIVE_CH_COUNT];       /* DRVOFF high after the last apply (immediate: no update event involved) */
    int16_t active[DRIVE_CH_COUNT]; /* reported active at the last read */
    bool wnz[DRIVE_CH_COUNT];       /* a non-zero compare value written since the last read */
    bool run_valid[DRIVE_CH_COUNT]; /* the written values form an unbroken run of one sign's non-zero */
    uint32_t run_since[DRIVE_CH_COUNT]; /* HAL clock after the run's first write */
    int8_t run_sign[DRIVE_CH_COUNT];
    uint32_t last_read;
    bool have_read;
} bench_pwm_track;

void bench_pwm_track_reset(bench_pwm_track *t);

/* After an apply's writes; now_after: the HAL clock once every write of the call is done. w: per channel the
   physical sign x compare value written, 0 for a 0 (a brake, a pending cut, a rounded 0) or a cut. */
void bench_pwm_wrote(bench_pwm_track *t, const int16_t w[DRIVE_CH_COUNT], const bool cut[DRIVE_CH_COUNT],
                     uint32_t now_after);

/* At read_inputs. updated: an update event since the last apply ended (then what it wrote is active); without one
   the report leans to the stopping side: a written 0 counts as not yet active, a written non-zero as possibly
   active. Fills in->pwm_report and the applied_* and run_* fields. */
void bench_pwm_read(bench_pwm_track *t, bool updated, uint32_t now, drive_inputs *in);

/* motor_out_apply's register rules as a pure model (PH, compare value, the pending 0 of a direction change, the
   pending cut), with its report. */
typedef struct {
    int8_t ph[DRIVE_CH_COUNT];
    uint32_t ccr[DRIVE_CH_COUNT];
    bool zero_pending[DRIVE_CH_COUNT];
    bool cut[DRIVE_CH_COUNT];
    bench_pwm_track t;
} bench_pwm_model;

void bench_pwm_model_init(bench_pwm_model *m);

/* One motor_out_apply. updated: an update event since the last apply ended (UIF at the start of the call). */
void bench_pwm_model_apply(bench_pwm_model *m, const drive_outputs *o, bool updated, uint32_t now_after);

#endif /* FW_CORE_BENCH */
#endif
