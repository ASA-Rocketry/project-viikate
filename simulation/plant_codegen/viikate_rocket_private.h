/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: viikate_rocket_private.h
 *
 * Code generated for Simulink model 'viikate_rocket'.
 *
 * Model version                  : 5.1
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Wed Nov 11 15:27:43 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef viikate_rocket_private_h_
#define viikate_rocket_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "viikate_rocket_types.h"
#include "viikate_rocket.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
#define rtmSetFirstInitCond(rtm, val)  ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
#define rtmIsFirstInitCond(rtm)        ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern double rt_atan2d_snf(double u0, double u1);
extern void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const double u0[3], const double u1
  [9], double y[3]);
extern double rt_urand_Upu32_Yd_f_pw_snf(uint32_t *u);
extern double rt_nrand_Upu32_Yd_f_pw_snf(uint32_t *u);
extern double look1_binlxpw(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);
extern int32_t plook_s32dd_bincp(double u, const double bp[], uint32_t maxIndex,
  double *fraction, int32_t *prevIndex);
extern int32_t binsearch_s32d_prevIdx(double u, const double bp[], uint32_t
  startIndex, uint32_t maxIndex);

/* private model entry point functions */
extern void viikate_rocket_derivatives(void);

#endif                                 /* viikate_rocket_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
