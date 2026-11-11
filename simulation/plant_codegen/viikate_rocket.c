/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: viikate_rocket.c
 *
 * Code generated for Simulink model 'viikate_rocket'.
 *
 * Model version                  : 5.1
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Wed Nov 11 01:28:46 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "viikate_rocket.h"
#include <math.h>
#include "viikate_rocket_private.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"
#define viikate_rocket_period          (0.001)

/* Exported block parameters */
double rp_gust_seed[2] = { 1.0, 2.0 } ;/* Variable: rp_gust_seed
                                        * Referenced by:
                                        *   '<S6>/Random Number1'
                                        *   '<S6>/Random Number2'
                                        */

double rp_gust_variance = 2.0;         /* Variable: rp_gust_variance
                                        * Referenced by:
                                        *   '<S6>/Random Number1'
                                        *   '<S6>/Random Number2'
                                        */
double rp_ignition_time_s = 3.0;       /* Variable: rp_ignition_time_s
                                        * Referenced by: '<Root>/ignition_time'
                                        */
double rp_mass_fuel_kg = 0.063;        /* Variable: rp_mass_fuel_kg
                                        * Referenced by:
                                        *   '<Root>/mass_rate'
                                        *   '<S5>/Constant2'
                                        *   '<S5>/Saturation'
                                        *   '<S9>/mass  '
                                        *   '<S79>/Constant'
                                        *   '<S79>/slope'
                                        */
double rp_mass_full_kg = 1.352;        /* Variable: rp_mass_full_kg
                                        * Referenced by:
                                        *   '<S5>/Constant1'
                                        *   '<S5>/Saturation'
                                        *   '<S9>/mass  '
                                        *   '<S79>/Constant'
                                        *   '<S79>/Constant1'
                                        *   '<S79>/slope'
                                        */
double rp_servo_rate_limit_radps = 10.471975511965978;/* Variable: rp_servo_rate_limit_radps
                                                       * Referenced by: '<Root>/servo_rate_limit'
                                                       */
double rp_servo_tau_s = 0.05;          /* Variable: rp_servo_tau_s
                                        * Referenced by: '<Root>/servo_gain'
                                        */
double rp_thrust_scale = 1.0;          /* Variable: rp_thrust_scale
                                        * Referenced by: '<S1>/thrust_scale'
                                        */
double rp_wind_model_mps[3] = { 0.0, 5.0, 5.0 } ;/* Variable: rp_wind_model_mps
                                                  * Referenced by: '<S6>/Integrator'
                                                  */

/* Continuous states */
X_viikate_rocket_T viikate_rocket_X;

/* Disabled State Vector */
XDis_viikate_rocket_T viikate_rocket_XDis;

/* Block signals and states (default storage) */
DW_viikate_rocket_T viikate_rocket_DW;

/* External inputs (root inport signals with default storage) */
ExtU_viikate_rocket_T viikate_rocket_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_viikate_rocket_T viikate_rocket_Y;

/* Real-time model */
static RT_MODEL_viikate_rocket_T viikate_rocket_M_;
RT_MODEL_viikate_rocket_T *const viikate_rocket_M = &viikate_rocket_M_;
static void rate_scheduler(void);
double look1_binlxpw(double u0, const double bp0[], const double table[],
                     uint32_t maxIndex)
{
  double frac;
  double yL_0d0;
  uint32_t iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_t bpIdx;
    uint32_t iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

int32_t plook_s32dd_bincp(double u, const double bp[], uint32_t maxIndex, double
  *fraction, int32_t *prevIndex)
{
  int32_t bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_s32d_prevIdx(u, bp, (uint32_t)*prevIndex, maxIndex);
    *fraction = (u - bp[(uint32_t)bpIndex]) / (bp[(uint32_t)bpIndex + 1U] - bp
      [(uint32_t)bpIndex]);
  } else {
    bpIndex = (int32_t)(maxIndex - 1U);
    *fraction = 1.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

int32_t binsearch_s32d_prevIdx(double u, const double bp[], uint32_t startIndex,
  uint32_t maxIndex)
{
  uint32_t bpIdx;
  uint32_t found;
  uint32_t iLeft;
  uint32_t iRght;

  /* Binary Search using Previous Index */
  bpIdx = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx - 1U;
      bpIdx = ((bpIdx + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIdx + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIdx + 1U;
      bpIdx = ((bpIdx + iRght) + 1U) >> 1U;
    }
  }

  return (int32_t)bpIdx;
}

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (viikate_rocket_M->Timing.TaskCounters.TID[2])++;
  if ((viikate_rocket_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.01s, 0.0s] */
    viikate_rocket_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  double t = rtsiGetT(si);
  double tnew = rtsiGetSolverStopTime(si);
  double h = rtsiGetStepSize(si);
  double *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  double *y = id->y;
  double *f0 = id->f[0];
  double *f1 = id->f[1];
  double *f2 = id->f[2];
  double *f3 = id->f[3];
  double temp;
  int i;
  int nXc = 21;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (unsigned int)nXc*sizeof(double));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  viikate_rocket_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  viikate_rocket_step();
  viikate_rocket_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  viikate_rocket_step();
  viikate_rocket_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  viikate_rocket_step();
  viikate_rocket_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_t tmp;
    int32_t tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const double u0[3], const double u1[9],
  double y[3])
{
  double A[9];
  double a21;
  double maxval;
  int32_t r1;
  int32_t r2;
  int32_t r3;
  memcpy(&A[0], &u1[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(u1[0]);
  a21 = fabs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (fabs(A[r3 + 3]) > fabs(A[r2 + 3])) {
    int32_t rtemp;
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[r1 + 3] * y[r1];
  y[r3] = u0[2] - A[r1 + 6] * y[r1];
  y[r2] /= A[r2 + 3];
  y[r3] -= A[r2 + 6] * y[r2];
  y[r3] /= A[r3 + 6];
  y[r2] -= A[r3 + 3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

double rt_urand_Upu32_Yd_f_pw_snf(uint32_t *u)
{
  uint32_t hi;
  uint32_t lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (double)*u * 4.656612875245797E-10;
}

double rt_nrand_Upu32_Yd_f_pw_snf(uint32_t *u)
{
  double si;
  double sr;
  double y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/* Model step function */
void viikate_rocket_step(void)
{
  if (rtmIsMajorTimeStep(viikate_rocket_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&viikate_rocket_M->solverInfo,
                          ((viikate_rocket_M->Timing.clockTick0+1)*
      viikate_rocket_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(viikate_rocket_M)) {
    viikate_rocket_M->Timing.t[0] = rtsiGetT(&viikate_rocket_M->solverInfo);
  }

  {
    double rtb_MatrixConcatenation[18];
    double rtb_ci[12];
    double rtb_ni[12];
    double rtb_Selector2[9];
    double rtb_VectorConcatenate_b[9];
    double rtb_ElementProduct[6];
    double rtb_M[3];
    double rtb_MatrixConcatenation_0[3];
    double rtb_Product2_ih[3];
    double rtb_Sum_fh[3];
    double rtb_Sum_k[3];
    double rtb_finForces[3];
    double rtb_hold_force[3];
    double rtb_ubvbwb[3];
    double tmp_0[3];
    double rtb_TmpSignalConversionAtPreLoo[2];
    double Integrator_CSTATE;
    double Saturation;
    double rtb_Gain2;
    double rtb_Liftmagnitude_idx_0;
    double rtb_Liftmagnitude_idx_1;
    double rtb_Liftmagnitude_idx_2;
    double rtb_Liftmagnitude_idx_3;
    double rtb_Sum_k_0;
    double rtb_V;
    double rtb_VectorConcatenate_o_tmp;
    double rtb_ci_0;
    double rtb_ci_1;
    double rtb_ci_idx_0;
    double rtb_ci_idx_1;
    double rtb_ci_idx_2;
    double rtb_ixj;
    double rtb_ixk;
    double rtb_ixk_f;
    double rtb_jxi;
    double rtb_jxi_n;
    double rtb_kxj;
    double rtb_ni_idx_1;
    double rtb_q0q1q2q3_idx_0;
    double rtb_q0q1q2q3_idx_1;
    double rtb_q0q1q2q3_idx_2;
    double rtb_q0q1q2q3_idx_3;
    double servo_lag_idx_0;
    double servo_lag_idx_1;
    double servo_lag_idx_2;
    double servo_lag_idx_3;
    int64_t tmp_1;
    int32_t i;
    int32_t mass_o2;
    int32_t rtb_MatrixConcatenation_tmp;
    int32_t rtb_PreLookUpIndexSearch_o1;
    int32_t tableOffset;
    int8_t rtAction;
    int8_t rtPrevAction;
    bool tmp;

    /* Integrator: '<S7>/q0 q1 q2 q3' incorporates:
     *  SignalConversion generated from: '<S7>/q0 q1 q2 q3'
     */
    if (viikate_rocket_DW.q0q1q2q3_DWORK1) {
      viikate_rocket_X.q0q1q2q3_CSTATE[0] = viikate_rocket_ConstB.q0;
      viikate_rocket_X.q0q1q2q3_CSTATE[1] = viikate_rocket_ConstB.q1;
      viikate_rocket_X.q0q1q2q3_CSTATE[2] = viikate_rocket_ConstB.q2;
      viikate_rocket_X.q0q1q2q3_CSTATE[3] = viikate_rocket_ConstB.q3;
    }

    rtb_q0q1q2q3_idx_0 = viikate_rocket_X.q0q1q2q3_CSTATE[0];
    rtb_q0q1q2q3_idx_1 = viikate_rocket_X.q0q1q2q3_CSTATE[1];
    rtb_q0q1q2q3_idx_2 = viikate_rocket_X.q0q1q2q3_CSTATE[2];
    rtb_q0q1q2q3_idx_3 = viikate_rocket_X.q0q1q2q3_CSTATE[3];

    /* Sqrt: '<S30>/sqrt' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     *  Product: '<S31>/Product'
     *  Product: '<S31>/Product1'
     *  Product: '<S31>/Product2'
     *  Product: '<S31>/Product3'
     *  Sum: '<S31>/Sum'
     */
    rtb_V = sqrt(((viikate_rocket_X.q0q1q2q3_CSTATE[0] *
                   viikate_rocket_X.q0q1q2q3_CSTATE[0] +
                   viikate_rocket_X.q0q1q2q3_CSTATE[1] *
                   viikate_rocket_X.q0q1q2q3_CSTATE[1]) +
                  viikate_rocket_X.q0q1q2q3_CSTATE[2] *
                  viikate_rocket_X.q0q1q2q3_CSTATE[2]) +
                 viikate_rocket_X.q0q1q2q3_CSTATE[3] *
                 viikate_rocket_X.q0q1q2q3_CSTATE[3]);

    /* Product: '<S29>/Product' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_jxi = viikate_rocket_X.q0q1q2q3_CSTATE[0] / rtb_V;

    /* Product: '<S29>/Product1' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_ixk_f = viikate_rocket_X.q0q1q2q3_CSTATE[1] / rtb_V;

    /* Product: '<S29>/Product2' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_jxi_n = viikate_rocket_X.q0q1q2q3_CSTATE[2] / rtb_V;

    /* Product: '<S29>/Product3' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_V = viikate_rocket_X.q0q1q2q3_CSTATE[3] / rtb_V;

    /* Product: '<S19>/Product3' incorporates:
     *  Product: '<S23>/Product3'
     */
    servo_lag_idx_0 = rtb_jxi * rtb_jxi;

    /* Product: '<S19>/Product2' incorporates:
     *  Product: '<S23>/Product2'
     */
    servo_lag_idx_1 = rtb_ixk_f * rtb_ixk_f;

    /* Product: '<S19>/Product1' incorporates:
     *  Product: '<S23>/Product1'
     *  Product: '<S27>/Product1'
     */
    servo_lag_idx_2 = rtb_jxi_n * rtb_jxi_n;

    /* Product: '<S19>/Product' incorporates:
     *  Product: '<S23>/Product'
     *  Product: '<S27>/Product'
     */
    servo_lag_idx_3 = rtb_V * rtb_V;

    /* Sum: '<S19>/Sum' incorporates:
     *  Product: '<S19>/Product'
     *  Product: '<S19>/Product1'
     *  Product: '<S19>/Product2'
     *  Product: '<S19>/Product3'
     */
    rtb_VectorConcatenate_b[0] = ((servo_lag_idx_0 + servo_lag_idx_1) -
      servo_lag_idx_2) - servo_lag_idx_3;

    /* Product: '<S22>/Product3' incorporates:
     *  Product: '<S20>/Product3'
     */
    Saturation = rtb_V * rtb_jxi;

    /* Product: '<S22>/Product2' incorporates:
     *  Product: '<S20>/Product2'
     */
    rtb_ixk = rtb_ixk_f * rtb_jxi_n;

    /* Gain: '<S22>/Gain' incorporates:
     *  Product: '<S22>/Product2'
     *  Product: '<S22>/Product3'
     *  Sum: '<S22>/Sum'
     */
    rtb_VectorConcatenate_b[1] = (rtb_ixk - Saturation) * 2.0;

    /* Product: '<S25>/Product2' incorporates:
     *  Product: '<S21>/Product2'
     */
    rtb_ixj = rtb_ixk_f * rtb_V;

    /* Product: '<S25>/Product1' incorporates:
     *  Product: '<S21>/Product1'
     */
    rtb_VectorConcatenate_o_tmp = rtb_jxi * rtb_jxi_n;

    /* Gain: '<S25>/Gain' incorporates:
     *  Product: '<S25>/Product1'
     *  Product: '<S25>/Product2'
     *  Sum: '<S25>/Sum'
     */
    rtb_VectorConcatenate_b[2] = (rtb_VectorConcatenate_o_tmp + rtb_ixj) * 2.0;

    /* Gain: '<S20>/Gain' incorporates:
     *  Sum: '<S20>/Sum'
     */
    rtb_VectorConcatenate_b[3] = (Saturation + rtb_ixk) * 2.0;

    /* Sum: '<S23>/Sum' incorporates:
     *  Sum: '<S27>/Sum'
     */
    servo_lag_idx_0 -= servo_lag_idx_1;
    rtb_VectorConcatenate_b[4] = (servo_lag_idx_0 + servo_lag_idx_2) -
      servo_lag_idx_3;

    /* Product: '<S26>/Product1' incorporates:
     *  Product: '<S24>/Product1'
     */
    servo_lag_idx_1 = rtb_jxi * rtb_ixk_f;

    /* Product: '<S26>/Product2' incorporates:
     *  Product: '<S24>/Product2'
     */
    Saturation = rtb_jxi_n * rtb_V;

    /* Gain: '<S26>/Gain' incorporates:
     *  Product: '<S26>/Product1'
     *  Product: '<S26>/Product2'
     *  Sum: '<S26>/Sum'
     */
    rtb_VectorConcatenate_b[5] = (Saturation - servo_lag_idx_1) * 2.0;

    /* Gain: '<S21>/Gain' incorporates:
     *  Sum: '<S21>/Sum'
     */
    rtb_VectorConcatenate_b[6] = (rtb_ixj - rtb_VectorConcatenate_o_tmp) * 2.0;

    /* Gain: '<S24>/Gain' incorporates:
     *  Sum: '<S24>/Sum'
     */
    rtb_VectorConcatenate_b[7] = (servo_lag_idx_1 + Saturation) * 2.0;

    /* Sum: '<S27>/Sum' */
    rtb_VectorConcatenate_b[8] = (servo_lag_idx_0 - servo_lag_idx_2) +
      servo_lag_idx_3;

    /* Clock: '<Root>/Clock' */
    rtb_jxi = viikate_rocket_M->Timing.t[0];

    /* Sum: '<Root>/time_since_ignition' incorporates:
     *  Constant: '<Root>/ignition_time'
     */
    rtb_ixk_f = rtb_jxi - rp_ignition_time_s;

    /* Saturate: '<Root>/t_ign' */
    if (rtb_ixk_f <= 0.0) {
      rtb_ixk_f = 0.0;
    }

    /* End of Saturate: '<Root>/t_ign' */

    /* Sum: '<S5>/Add' incorporates:
     *  Constant: '<S5>/Constant1'
     *  Constant: '<S5>/Constant2'
     *  Product: '<S5>/Product'
     */
    Saturation = -rp_mass_fuel_kg * rtb_ixk_f + rp_mass_full_kg;

    /* Saturate: '<S5>/Saturation' */
    servo_lag_idx_0 = rp_mass_full_kg - rp_mass_fuel_kg;
    if (Saturation > rp_mass_full_kg) {
      /* Saturate: '<S5>/Saturation' */
      Saturation = rp_mass_full_kg;
    } else if (Saturation < servo_lag_idx_0) {
      /* Saturate: '<S5>/Saturation' */
      Saturation = servo_lag_idx_0;
    }

    /* End of Saturate: '<S5>/Saturation' */

    /* MATLAB Function: '<S85>/MATLAB Function2' incorporates:
     *  Integrator: '<Root>/servo_lag'
     */
    rtb_ci[0] = cos(viikate_rocket_X.servo_lag_CSTATE[0]);
    rtb_ci[4] = sin(viikate_rocket_X.servo_lag_CSTATE[0]);
    rtb_ci[8] = 0.0;
    rtb_ci[1] = cos(viikate_rocket_X.servo_lag_CSTATE[1]);
    rtb_ci[5] = 0.0;
    rtb_ci[9] = sin(viikate_rocket_X.servo_lag_CSTATE[1]);
    rtb_ci[2] = cos(viikate_rocket_X.servo_lag_CSTATE[2]);
    rtb_ci[6] = -sin(viikate_rocket_X.servo_lag_CSTATE[2]);
    rtb_ci[10] = 0.0;
    rtb_ci[3] = cos(viikate_rocket_X.servo_lag_CSTATE[3]);
    rtb_ci[7] = 0.0;
    rtb_ci[11] = -sin(viikate_rocket_X.servo_lag_CSTATE[3]);

    /* Integrator: '<Root>/servo_lag' */
    servo_lag_idx_0 = viikate_rocket_X.servo_lag_CSTATE[0];

    /* MATLAB Function: '<S85>/MATLAB Function3' incorporates:
     *  Constant: '<S85>/Constant4'
     *  MATLAB Function: '<S85>/MATLAB Function2'
     */
    rtb_ni[0] = 0.0 - rtb_ci[4];
    rtb_ni[4] = rtb_ci[0];
    rtb_ni[8] = 0.0 * rtb_ci[4] - 0.0 * rtb_ci[0];

    /* Integrator: '<Root>/servo_lag' */
    servo_lag_idx_1 = viikate_rocket_X.servo_lag_CSTATE[1];

    /* MATLAB Function: '<S85>/MATLAB Function3' incorporates:
     *  Constant: '<S85>/Constant4'
     *  MATLAB Function: '<S85>/MATLAB Function2'
     */
    rtb_ni[1] = -rtb_ci[9];
    rtb_ni[5] = 0.0 * rtb_ci[1] - 0.0 * rtb_ci[9];
    rtb_ni[9] = 0.0 - (-rtb_ci[1]);

    /* Integrator: '<Root>/servo_lag' */
    servo_lag_idx_2 = viikate_rocket_X.servo_lag_CSTATE[2];

    /* MATLAB Function: '<S85>/MATLAB Function3' incorporates:
     *  Constant: '<S85>/Constant4'
     *  MATLAB Function: '<S85>/MATLAB Function2'
     */
    rtb_ni[2] = 0.0 - (-rtb_ci[6]);
    rtb_ni[6] = -rtb_ci[2];
    rtb_ni[10] = 0.0 * rtb_ci[6] - 0.0 * rtb_ci[2];

    /* Integrator: '<Root>/servo_lag' */
    servo_lag_idx_3 = viikate_rocket_X.servo_lag_CSTATE[3];

    /* MATLAB Function: '<S85>/MATLAB Function3' incorporates:
     *  Constant: '<S85>/Constant4'
     *  MATLAB Function: '<S85>/MATLAB Function2'
     */
    rtb_ni[3] = rtb_ci[11];
    rtb_ni[7] = 0.0 * rtb_ci[3] - 0.0 * rtb_ci[11];
    rtb_ni[11] = 0.0 - rtb_ci[3];

    /* Product: '<S1>/Product' incorporates:
     *  Sum: '<S74>/Sum'
     */
    rtb_ixj = 0.0;
    rtb_VectorConcatenate_o_tmp = 0.0;
    rtb_Sum_k_0 = 0.0;

    /* Integrator: '<S6>/Integrator' incorporates:
     *  Product: '<S6>/Matrix Multiply'
     */
    rtb_V = viikate_rocket_X.Integrator_CSTATE[1];
    rtb_ixk = viikate_rocket_X.Integrator_CSTATE[0];
    Integrator_CSTATE = viikate_rocket_X.Integrator_CSTATE[2];
    for (i = 0; i < 3; i++) {
      /* Product: '<S1>/Product' incorporates:
       *  Constant: '<S1>/Constant'
       *  Product: '<S1>/Product2'
       *  Reshape: '<S28>/Reshape (9) to [3x3] column-major'
       *  Sum: '<S74>/Sum'
       */
      rtb_ci_idx_2 = Saturation * viikate_rocket_ConstP.Constant_Value_c[i];
      rtb_ixj += rtb_VectorConcatenate_b[3 * i] * rtb_ci_idx_2;
      rtb_VectorConcatenate_o_tmp += rtb_VectorConcatenate_b[3 * i + 1] *
        rtb_ci_idx_2;
      rtb_Sum_k_0 += rtb_VectorConcatenate_b[3 * i + 2] * rtb_ci_idx_2;

      /* Integrator: '<S2>/ub,vb,wb' */
      rtb_jxi_n = viikate_rocket_X.ubvbwb_CSTATE[i];
      rtb_ubvbwb[i] = rtb_jxi_n;

      /* Sum: '<Root>/Add' incorporates:
       *  Gain: '<S6>/Gain'
       *  Integrator: '<S2>/ub,vb,wb'
       *  Integrator: '<S6>/Integrator'
       *  Product: '<S6>/Matrix Multiply'
       *  Reshape: '<S28>/Reshape (9) to [3x3] column-major'
       */
      rtb_hold_force[i] = ((rtb_VectorConcatenate_b[i + 3] * -rtb_V +
                            rtb_VectorConcatenate_b[i] * -rtb_ixk) +
                           rtb_VectorConcatenate_b[i + 6] * -Integrator_CSTATE)
        + rtb_jxi_n;
    }

    /* Switch: '<S3>/Switch2' incorporates:
     *  Abs: '<S3>/Abs'
     *  Constant: '<S3>/Constant3'
     */
    if (fabs(rtb_hold_force[0]) >= 0.1) {
      rtb_jxi_n = rtb_hold_force[0];
    } else {
      rtb_jxi_n = 0.1;
    }

    /* End of Switch: '<S3>/Switch2' */

    /* SignalConversion generated from: '<S85>/Matrix Multiply' */
    rtb_Product2_ih[0] = rtb_jxi_n;
    rtb_Product2_ih[1] = rtb_hold_force[1];
    rtb_Product2_ih[2] = rtb_hold_force[2];

    /* Product: '<S85>/Matrix Multiply' */
    Integrator_CSTATE = 0.0;
    rtb_ni_idx_1 = 0.0;
    rtb_kxj = 0.0;
    rtb_V = 0.0;

    /* Product: '<S85>/Matrix Multiply1' */
    rtb_ci_idx_0 = 0.0;
    rtb_ci_idx_1 = 0.0;
    rtb_ci_idx_2 = 0.0;
    rtb_ixk = 0.0;
    for (i = 0; i < 3; i++) {
      /* Product: '<S85>/Matrix Multiply' incorporates:
       *  Product: '<S85>/Matrix Multiply1'
       */
      rtb_Gain2 = rtb_Product2_ih[i];
      mass_o2 = i << 2;
      Integrator_CSTATE += rtb_ni[mass_o2] * rtb_Gain2;
      rtb_ni_idx_1 += rtb_ni[mass_o2 + 1] * rtb_Gain2;
      rtb_kxj += rtb_ni[mass_o2 + 2] * rtb_Gain2;
      rtb_V += rtb_ni[mass_o2 + 3] * rtb_Gain2;

      /* Product: '<S85>/Matrix Multiply1' incorporates:
       *  Product: '<S85>/Matrix Multiply'
       */
      rtb_ci_idx_0 += rtb_ci[mass_o2] * rtb_Gain2;
      rtb_ci_idx_1 += rtb_ci[mass_o2 + 1] * rtb_Gain2;
      rtb_ci_idx_2 += rtb_ci[mass_o2 + 2] * rtb_Gain2;
      rtb_ixk += rtb_ci[mass_o2 + 3] * rtb_Gain2;
    }

    /* Trigonometry: '<S85>/Atan2' */
    Integrator_CSTATE = rt_atan2d_snf(Integrator_CSTATE, rtb_ci_idx_0);

    /* Gain: '<S85>/Gain' */
    rtb_ci_idx_0 = 1.975 * Integrator_CSTATE;

    /* Trigonometry: '<S85>/Atan2' */
    Integrator_CSTATE = rt_atan2d_snf(rtb_ni_idx_1, rtb_ci_idx_1);

    /* Gain: '<S85>/Gain' */
    rtb_ni_idx_1 = 1.975 * Integrator_CSTATE;

    /* Trigonometry: '<S85>/Atan2' */
    Integrator_CSTATE = rt_atan2d_snf(rtb_kxj, rtb_ci_idx_2);

    /* Gain: '<S85>/Gain' */
    rtb_ci_idx_1 = 1.975 * Integrator_CSTATE;

    /* Trigonometry: '<S85>/Atan2' */
    Integrator_CSTATE = rt_atan2d_snf(rtb_V, rtb_ixk);

    /* Gain: '<S85>/Gain' */
    Integrator_CSTATE *= 1.975;

    /* Sqrt: '<S4>/Square Root' incorporates:
     *  DotProduct: '<S4>/Dot Product'
     *  SignalConversion generated from: '<S85>/Matrix Multiply'
     */
    rtb_V = sqrt((rtb_jxi_n * rtb_jxi_n + rtb_hold_force[1] * rtb_hold_force[1])
                 + rtb_hold_force[2] * rtb_hold_force[2]);

    /* Gain: '<S4>/gain' incorporates:
     *  Math: '<S4>/Square'
     */
    rtb_ixk = rtb_V * rtb_V * 0.6125;

    /* Gain: '<S85>/Gain1' */
    rtb_kxj = 0.0016875 * rtb_ixk;

    /* Product: '<S85>/Product1' incorporates:
     *  Gain: '<S85>/Gain'
     */
    rtb_Liftmagnitude_idx_0 = rtb_ci_idx_0 * rtb_kxj;
    rtb_Liftmagnitude_idx_1 = rtb_ni_idx_1 * rtb_kxj;
    rtb_Liftmagnitude_idx_2 = rtb_ci_idx_1 * rtb_kxj;
    rtb_Liftmagnitude_idx_3 = Integrator_CSTATE * rtb_kxj;
    for (mass_o2 = 0; mass_o2 < 3; mass_o2++) {
      /* MATLAB Function: '<S85>/MATLAB Function1' incorporates:
       *  Constant: '<S85>/Lift directions'
       */
      i = mass_o2 << 2;
      rtb_ci_idx_2 = viikate_rocket_ConstP.Liftdirections_Value[i] *
        rtb_Liftmagnitude_idx_0;
      rtb_ci[i] = rtb_ci_idx_2;
      rtb_Gain2 = viikate_rocket_ConstP.Liftdirections_Value[i + 1] *
        rtb_Liftmagnitude_idx_1;
      rtb_ci[i + 1] = rtb_Gain2;
      rtb_ci_0 = viikate_rocket_ConstP.Liftdirections_Value[i + 2] *
        rtb_Liftmagnitude_idx_2;
      rtb_ci[i + 2] = rtb_ci_0;
      rtb_ci_1 = viikate_rocket_ConstP.Liftdirections_Value[i + 3] *
        rtb_Liftmagnitude_idx_3;
      rtb_ci[i + 3] = rtb_ci_1;

      /* MATLAB Function: '<S85>/MATLAB Function4' */
      rtb_finForces[mass_o2] = ((rtb_Gain2 + rtb_ci_idx_2) + rtb_ci_0) +
        rtb_ci_1;
    }

    /* Gain: '<S4>/Gain2' */
    rtb_Gain2 = -0.0884 * rtb_ixk;

    /* Product: '<S84>/Divide' */
    rtb_ci_idx_2 = 1.0 / rtb_jxi_n;
    rtb_Liftmagnitude_idx_0 = rtb_ci_idx_2 * rtb_hold_force[1];
    rtb_ci_0 = rtb_ci_idx_2 * rtb_hold_force[2];

    /* Trigonometry: '<S84>/Atan' incorporates:
     *  Constant: '<S84>/Constant2'
     *  Product: '<S84>/Divide'
     */
    rtb_ci_idx_2 = atan(rtb_ci_idx_2 * 0.0);
    rtb_Liftmagnitude_idx_0 = atan(rtb_Liftmagnitude_idx_0);
    rtb_ci_0 = atan(rtb_ci_0);

    /* Product: '<S88>/Product3' incorporates:
     *  Constant: '<S88>/Constant3'
     *  DotProduct: '<S88>/Dot Product1'
     *  Gain: '<S88>/Gain8'
     *  Gain: '<S88>/Gain9'
     *  Sum: '<S88>/Add1'
     */
    rtb_Liftmagnitude_idx_1 = (((rtb_ci_idx_2 * rtb_ci_idx_2 +
      rtb_Liftmagnitude_idx_0 * rtb_Liftmagnitude_idx_0) + rtb_ci_0 * rtb_ci_0) *
      60.0 + 0.635) * (-0.00442 * rtb_ixk);

    /* Sum: '<S4>/Add3' incorporates:
     *  Product: '<S4>/Product'
     *  Product: '<S88>/Divide2'
     *  Product: '<S88>/Product4'
     *  SignalConversion generated from: '<S85>/Matrix Multiply'
     *  Trigonometry: '<S84>/Atan'
     */
    rtb_Product2_ih[0] = rtb_jxi_n / rtb_V * rtb_Liftmagnitude_idx_1 + rtb_Gain2
      * rtb_ci_idx_2;
    rtb_Product2_ih[1] = rtb_Product2_ih[1] / rtb_V * rtb_Liftmagnitude_idx_1 +
      rtb_Gain2 * rtb_Liftmagnitude_idx_0;
    rtb_Product2_ih[2] = rtb_Product2_ih[2] / rtb_V * rtb_Liftmagnitude_idx_1 +
      rtb_Gain2 * rtb_ci_0;

    /* DataTypeConversion: '<Root>/ignited' incorporates:
     *  Constant: '<Root>/ignition_time'
     *  RelationalOperator: '<Root>/ignited_cmp'
     */
    rtb_jxi = (rtb_jxi >= rp_ignition_time_s);

    /* Product: '<S1>/Product1' incorporates:
     *  Constant: '<S1>/burn_time'
     *  Constant: '<S1>/thrust_scale'
     *  DataTypeConversion: '<S1>/burning'
     *  Lookup_n-D: '<S1>/thrust_curve'
     *  Product: '<S78>/i x k'
     *  RelationalOperator: '<S1>/burning_cmp'
     */
    rtb_ixk_f = (double)(rtb_ixk_f < 1.0) * look1_binlxpw(rtb_ixk_f,
      viikate_rocket_ConstP.thrust_curve_bp01Data,
      viikate_rocket_ConstP.thrust_curve_tableData, 14U) * rtb_jxi *
      rp_thrust_scale;
    tmp = (rtmIsMajorTimeStep(viikate_rocket_M) &&
           viikate_rocket_M->Timing.TaskCounters.TID[1] == 0);
    if (tmp) {
      /* Logic: '<Root>/latch' incorporates:
       *  Gain: '<Root>/weight'
       *  RelationalOperator: '<Root>/thrust_exceeds_weight'
       *  UnitDelay: '<Root>/latch_memory'
       */
      viikate_rocket_DW.latch_memory_DSTATE = ((rtb_ixk_f > 9.819 * Saturation) ||
        viikate_rocket_DW.latch_memory_DSTATE);

      /* DataTypeConversion: '<Root>/released' incorporates:
       *  UnitDelay: '<Root>/latch_memory'
       */
      viikate_rocket_DW.released = viikate_rocket_DW.latch_memory_DSTATE;
    }

    /* Product: '<Root>/hold_force' incorporates:
     *  DotProduct: '<S85>/Dot Product'
     *  Gain: '<S85>/Gain'
     *  Gain: '<S85>/Gain2'
     *  MATLAB Function: '<S85>/MATLAB Function4'
     *  Product: '<S1>/Product'
     *  Product: '<S85>/Product'
     *  Sum: '<S1>/Add'
     *  Sum: '<S4>/Add2'
     *  Sum: '<S74>/Sum'
     */
    rtb_hold_force[0] = ((((((rtb_ci_idx_0 * rtb_ci_idx_0 + rtb_ni_idx_1 *
      rtb_ni_idx_1) + rtb_ci_idx_1 * rtb_ci_idx_1) + Integrator_CSTATE *
      Integrator_CSTATE) * -0.5 * rtb_kxj + (rtb_finForces[0] + rtb_Product2_ih
      [0])) + rtb_ixj) + rtb_ixk_f) * viikate_rocket_DW.released;
    rtb_hold_force[1] = ((((rtb_finForces[1] + rtb_Product2_ih[1]) + 0.0) +
                          rtb_VectorConcatenate_o_tmp) + 0.0) *
      viikate_rocket_DW.released;
    rtb_hold_force[2] = ((((rtb_finForces[2] + rtb_Product2_ih[2]) + 0.0) +
                          rtb_Sum_k_0) + 0.0) * viikate_rocket_DW.released;

    /* Integrator: '<S2>/p,q,r ' */
    rtb_Sum_fh[0] = viikate_rocket_X.pqr_CSTATE[0];
    rtb_Sum_fh[1] = viikate_rocket_X.pqr_CSTATE[1];
    rtb_Sum_fh[2] = viikate_rocket_X.pqr_CSTATE[2];
    if (tmp) {
      /* Outport: '<Root>/sf_body_mps2' incorporates:
       *  Product: '<Root>/hold_force'
       *  Product: '<Root>/specific_force'
       *  Product: '<S1>/Product'
       *  Sum: '<Root>/minus_gravity'
       *  Sum: '<S74>/Sum'
       */
      viikate_rocket_Y.sf_body_mps2[0] = (rtb_hold_force[0] - rtb_ixj) /
        Saturation;
      viikate_rocket_Y.sf_body_mps2[1] = (rtb_hold_force[1] -
        rtb_VectorConcatenate_o_tmp) / Saturation;
      viikate_rocket_Y.sf_body_mps2[2] = (rtb_hold_force[2] - rtb_Sum_k_0) /
        Saturation;

      /* Outport: '<Root>/omega_body_radps' incorporates:
       *  Integrator: '<S2>/p,q,r '
       */
      viikate_rocket_Y.omega_body_radps[0] = viikate_rocket_X.pqr_CSTATE[0];

      /* Outport: '<Root>/pos_model_m' incorporates:
       *  Integrator: '<S2>/xe,ye,ze'
       */
      viikate_rocket_Y.pos_model_m[0] = viikate_rocket_X.xeyeze_CSTATE[0];

      /* Outport: '<Root>/omega_body_radps' incorporates:
       *  Integrator: '<S2>/p,q,r '
       */
      viikate_rocket_Y.omega_body_radps[1] = viikate_rocket_X.pqr_CSTATE[1];

      /* Outport: '<Root>/pos_model_m' incorporates:
       *  Integrator: '<S2>/xe,ye,ze'
       */
      viikate_rocket_Y.pos_model_m[1] = viikate_rocket_X.xeyeze_CSTATE[1];

      /* Outport: '<Root>/omega_body_radps' incorporates:
       *  Integrator: '<S2>/p,q,r '
       */
      viikate_rocket_Y.omega_body_radps[2] = viikate_rocket_X.pqr_CSTATE[2];

      /* Outport: '<Root>/pos_model_m' incorporates:
       *  Integrator: '<S2>/xe,ye,ze'
       */
      viikate_rocket_Y.pos_model_m[2] = viikate_rocket_X.xeyeze_CSTATE[2];
    }

    /* Product: '<S14>/Product' incorporates:
     *  Integrator: '<S2>/ub,vb,wb'
     *  Math: '<S2>/Transpose'
     *  Reshape: '<S14>/Reshape2'
     *  Reshape: '<S28>/Reshape (9) to [3x3] column-major'
     */
    rtb_jxi_n = viikate_rocket_X.ubvbwb_CSTATE[1];
    rtb_ixk_f = viikate_rocket_X.ubvbwb_CSTATE[0];
    rtb_ixj = viikate_rocket_X.ubvbwb_CSTATE[2];
    for (i = 0; i < 3; i++) {
      viikate_rocket_DW.Reshape2[i] = (rtb_VectorConcatenate_b[3 * i + 1] *
        rtb_jxi_n + rtb_VectorConcatenate_b[3 * i] * rtb_ixk_f) +
        rtb_VectorConcatenate_b[3 * i + 2] * rtb_ixj;
    }

    /* End of Product: '<S14>/Product' */
    if (tmp) {
      /* Outport: '<Root>/vel_model_mps' incorporates:
       *  ZeroOrderHold: '<Root>/zoh_vel_model_mps'
       */
      viikate_rocket_Y.vel_model_mps[0] = viikate_rocket_DW.Reshape2[0];
      viikate_rocket_Y.vel_model_mps[1] = viikate_rocket_DW.Reshape2[1];
      viikate_rocket_Y.vel_model_mps[2] = viikate_rocket_DW.Reshape2[2];
    }

    /* Sqrt: '<S72>/sqrt' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     *  Product: '<S73>/Product'
     *  Product: '<S73>/Product1'
     *  Product: '<S73>/Product2'
     *  Product: '<S73>/Product3'
     *  Sum: '<S73>/Sum'
     */
    rtb_kxj = sqrt(((viikate_rocket_X.q0q1q2q3_CSTATE[0] *
                     viikate_rocket_X.q0q1q2q3_CSTATE[0] +
                     viikate_rocket_X.q0q1q2q3_CSTATE[1] *
                     viikate_rocket_X.q0q1q2q3_CSTATE[1]) +
                    viikate_rocket_X.q0q1q2q3_CSTATE[2] *
                    viikate_rocket_X.q0q1q2q3_CSTATE[2]) +
                   viikate_rocket_X.q0q1q2q3_CSTATE[3] *
                   viikate_rocket_X.q0q1q2q3_CSTATE[3]);

    /* Product: '<S71>/Product' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_ixj = viikate_rocket_X.q0q1q2q3_CSTATE[0] / rtb_kxj;

    /* Product: '<S71>/Product1' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_ixk_f = viikate_rocket_X.q0q1q2q3_CSTATE[1] / rtb_kxj;

    /* Product: '<S71>/Product2' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_jxi_n = viikate_rocket_X.q0q1q2q3_CSTATE[2] / rtb_kxj;

    /* Product: '<S71>/Product3' incorporates:
     *  Integrator: '<S7>/q0 q1 q2 q3'
     */
    rtb_kxj = viikate_rocket_X.q0q1q2q3_CSTATE[3] / rtb_kxj;

    /* Product: '<S65>/Product3' incorporates:
     *  Product: '<S61>/Product3'
     */
    rtb_ci_idx_0 = rtb_ixj * rtb_ixj;

    /* Product: '<S65>/Product2' incorporates:
     *  Product: '<S61>/Product2'
     */
    rtb_ci_idx_2 = rtb_ixk_f * rtb_ixk_f;

    /* Product: '<S65>/Product1' incorporates:
     *  Product: '<S61>/Product1'
     *  Product: '<S69>/Product1'
     */
    rtb_VectorConcatenate_o_tmp = rtb_jxi_n * rtb_jxi_n;

    /* Product: '<S65>/Product' incorporates:
     *  Product: '<S61>/Product'
     *  Product: '<S69>/Product'
     */
    rtb_Sum_k_0 = rtb_kxj * rtb_kxj;

    /* Sum: '<S65>/Sum' incorporates:
     *  Product: '<S65>/Product2'
     *  Product: '<S65>/Product3'
     *  Sum: '<S69>/Sum'
     */
    Integrator_CSTATE = rtb_ci_idx_0 - rtb_ci_idx_2;

    /* Gain: '<S63>/Gain' incorporates:
     *  Product: '<S63>/Product1'
     *  Product: '<S63>/Product2'
     *  Sum: '<S63>/Sum'
     */
    rtb_Selector2[6] = (rtb_ixk_f * rtb_kxj - rtb_ixj * rtb_jxi_n) * 2.0;

    /* If: '<S32>/If' incorporates:
     *  Gain: '<S36>/Gain1'
     *  Reshape: '<S70>/Reshape (9) to [3x3] column-major'
     *  Selector: '<S36>/Selector1'
     */
    rtPrevAction = viikate_rocket_DW.If_ActiveSubsystem;
    if (rtsiIsModeUpdateTimeStep(&viikate_rocket_M->solverInfo)) {
      rtAction = (int8_t)(!(-rtb_Selector2[6] >= 1.0) && !(-rtb_Selector2[6] <=
        -1.0));
      viikate_rocket_DW.If_ActiveSubsystem = rtAction;
    } else {
      rtAction = viikate_rocket_DW.If_ActiveSubsystem;
    }

    if (rtPrevAction != rtAction) {
      rtsiSetBlockStateForSolverChangedAtMajorStep(&viikate_rocket_M->solverInfo,
        true);
    }

    if (rtAction == 0) {
      /* Outputs for IfAction SubSystem: '<S32>/AxisRotZeroR3' incorporates:
       *  ActionPort: '<S35>/Action Port'
       */
      /* Fcn: '<S35>/Fcn1' incorporates:
       *  Gain: '<S36>/Gain3'
       *  Gain: '<S64>/Gain'
       *  Merge: '<S32>/Merge'
       *  Product: '<S64>/Product2'
       *  Product: '<S64>/Product3'
       *  Product: '<S65>/Product'
       *  Product: '<S65>/Product1'
       *  Sum: '<S64>/Sum'
       *  Sum: '<S65>/Sum'
       */
      viikate_rocket_DW.Merge[0] = rt_atan2d_snf(-((rtb_ixk_f * rtb_jxi_n -
        rtb_kxj * rtb_ixj) * 2.0), (Integrator_CSTATE +
        rtb_VectorConcatenate_o_tmp) - rtb_Sum_k_0);

      /* If: '<S42>/If' incorporates:
       *  Gain: '<S36>/Gain1'
       *  Reshape: '<S70>/Reshape (9) to [3x3] column-major'
       *  Selector: '<S36>/Selector1'
       */
      rtPrevAction = viikate_rocket_DW.If_ActiveSubsystem_o;
      if (rtsiIsModeUpdateTimeStep(&viikate_rocket_M->solverInfo)) {
        if (-rtb_Selector2[6] > 1.0) {
          rtAction = 0;
        } else if (-rtb_Selector2[6] < -1.0) {
          rtAction = 1;
        } else {
          rtAction = 2;
        }

        viikate_rocket_DW.If_ActiveSubsystem_o = rtAction;
      } else {
        rtAction = viikate_rocket_DW.If_ActiveSubsystem_o;
      }

      if (rtPrevAction != rtAction) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&viikate_rocket_M->solverInfo, true);
      }

      switch (rtAction) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S42>/If Action Subsystem' incorporates:
         *  ActionPort: '<S43>/Action Port'
         */
        if (rtmIsMajorTimeStep(viikate_rocket_M) &&
            viikate_rocket_M->Timing.TaskCounters.TID[1] == 0) {
          /* Merge: '<S42>/Merge' incorporates:
           *  Constant: '<S43>/Constant'
           */
          viikate_rocket_DW.Merge_k = 1.0;
        }

        /* End of Outputs for SubSystem: '<S42>/If Action Subsystem' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S42>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S44>/Action Port'
         */
        if (rtmIsMajorTimeStep(viikate_rocket_M) &&
            viikate_rocket_M->Timing.TaskCounters.TID[1] == 0) {
          /* Merge: '<S42>/Merge' incorporates:
           *  Constant: '<S44>/Constant'
           */
          viikate_rocket_DW.Merge_k = -1.0;
        }

        /* End of Outputs for SubSystem: '<S42>/If Action Subsystem1' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S42>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S45>/Action Port'
         */
        /* Merge: '<S42>/Merge' incorporates:
         *  Gain: '<S36>/Gain1'
         *  Reshape: '<S70>/Reshape (9) to [3x3] column-major'
         *  Selector: '<S36>/Selector1'
         *  SignalConversion generated from: '<S45>/In'
         */
        viikate_rocket_DW.Merge_k = -rtb_Selector2[6];

        /* End of Outputs for SubSystem: '<S42>/If Action Subsystem2' */
        break;
      }

      /* End of If: '<S42>/If' */

      /* Fcn: '<S35>/Fcn2' incorporates:
       *  Merge: '<S32>/Merge'
       */
      if (viikate_rocket_DW.Merge_k > 1.0) {
        rtb_ci_idx_2 = 1.0;
      } else if (viikate_rocket_DW.Merge_k < -1.0) {
        rtb_ci_idx_2 = -1.0;
      } else {
        rtb_ci_idx_2 = viikate_rocket_DW.Merge_k;
      }

      viikate_rocket_DW.Merge[1] = asin(rtb_ci_idx_2);

      /* End of Fcn: '<S35>/Fcn2' */

      /* Fcn: '<S35>/Fcn3' incorporates:
       *  Merge: '<S32>/Merge'
       */
      viikate_rocket_DW.Merge[2] = 0.0;

      /* End of Outputs for SubSystem: '<S32>/AxisRotZeroR3' */
    } else {
      /* Outputs for IfAction SubSystem: '<S32>/AxisRotDefault' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* Fcn: '<S34>/Fcn1' incorporates:
       *  Gain: '<S62>/Gain'
       *  Merge: '<S32>/Merge'
       *  Product: '<S62>/Product2'
       *  Product: '<S62>/Product3'
       *  Sum: '<S61>/Sum'
       *  Sum: '<S62>/Sum'
       */
      viikate_rocket_DW.Merge[0] = rt_atan2d_snf((rtb_kxj * rtb_ixj + rtb_ixk_f *
        rtb_jxi_n) * 2.0, ((rtb_ci_idx_0 + rtb_ci_idx_2) -
                           rtb_VectorConcatenate_o_tmp) - rtb_Sum_k_0);

      /* If: '<S38>/If' incorporates:
       *  Gain: '<S36>/Gain1'
       *  Reshape: '<S70>/Reshape (9) to [3x3] column-major'
       *  Selector: '<S36>/Selector1'
       */
      rtPrevAction = viikate_rocket_DW.If_ActiveSubsystem_d;
      if (rtsiIsModeUpdateTimeStep(&viikate_rocket_M->solverInfo)) {
        if (-rtb_Selector2[6] > 1.0) {
          rtAction = 0;
        } else if (-rtb_Selector2[6] < -1.0) {
          rtAction = 1;
        } else {
          rtAction = 2;
        }

        viikate_rocket_DW.If_ActiveSubsystem_d = rtAction;
      } else {
        rtAction = viikate_rocket_DW.If_ActiveSubsystem_d;
      }

      if (rtPrevAction != rtAction) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&viikate_rocket_M->solverInfo, true);
      }

      switch (rtAction) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S38>/If Action Subsystem' incorporates:
         *  ActionPort: '<S39>/Action Port'
         */
        if (rtmIsMajorTimeStep(viikate_rocket_M) &&
            viikate_rocket_M->Timing.TaskCounters.TID[1] == 0) {
          /* Merge: '<S38>/Merge' incorporates:
           *  Constant: '<S39>/Constant'
           */
          viikate_rocket_DW.Merge_o = 1.0;
        }

        /* End of Outputs for SubSystem: '<S38>/If Action Subsystem' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S38>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S40>/Action Port'
         */
        if (rtmIsMajorTimeStep(viikate_rocket_M) &&
            viikate_rocket_M->Timing.TaskCounters.TID[1] == 0) {
          /* Merge: '<S38>/Merge' incorporates:
           *  Constant: '<S40>/Constant'
           */
          viikate_rocket_DW.Merge_o = -1.0;
        }

        /* End of Outputs for SubSystem: '<S38>/If Action Subsystem1' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S38>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S41>/Action Port'
         */
        /* Merge: '<S38>/Merge' incorporates:
         *  Gain: '<S36>/Gain1'
         *  Reshape: '<S70>/Reshape (9) to [3x3] column-major'
         *  Selector: '<S36>/Selector1'
         *  SignalConversion generated from: '<S41>/In'
         */
        viikate_rocket_DW.Merge_o = -rtb_Selector2[6];

        /* End of Outputs for SubSystem: '<S38>/If Action Subsystem2' */
        break;
      }

      /* End of If: '<S38>/If' */

      /* Fcn: '<S34>/Fcn2' incorporates:
       *  Merge: '<S32>/Merge'
       */
      if (viikate_rocket_DW.Merge_o > 1.0) {
        rtb_ci_idx_2 = 1.0;
      } else if (viikate_rocket_DW.Merge_o < -1.0) {
        rtb_ci_idx_2 = -1.0;
      } else {
        rtb_ci_idx_2 = viikate_rocket_DW.Merge_o;
      }

      viikate_rocket_DW.Merge[1] = asin(rtb_ci_idx_2);

      /* End of Fcn: '<S34>/Fcn2' */

      /* Fcn: '<S34>/Fcn3' incorporates:
       *  Gain: '<S66>/Gain'
       *  Merge: '<S32>/Merge'
       *  Product: '<S66>/Product1'
       *  Product: '<S66>/Product2'
       *  Sum: '<S66>/Sum'
       *  Sum: '<S69>/Sum'
       */
      viikate_rocket_DW.Merge[2] = rt_atan2d_snf((rtb_ixj * rtb_ixk_f +
        rtb_jxi_n * rtb_kxj) * 2.0, (Integrator_CSTATE -
        rtb_VectorConcatenate_o_tmp) + rtb_Sum_k_0);

      /* End of Outputs for SubSystem: '<S32>/AxisRotDefault' */
    }

    /* End of If: '<S32>/If' */
    if (tmp) {
      /* Outport: '<Root>/euler_rad' incorporates:
       *  ZeroOrderHold: '<Root>/zoh_euler_rad'
       */
      viikate_rocket_Y.euler_rad[0] = viikate_rocket_DW.Merge[2];
      viikate_rocket_Y.euler_rad[1] = viikate_rocket_DW.Merge[1];
      viikate_rocket_Y.euler_rad[2] = viikate_rocket_DW.Merge[0];

      /* Outport: '<Root>/dcm_be' incorporates:
       *  Reshape: '<S28>/Reshape (9) to [3x3] column-major'
       */
      memcpy(&viikate_rocket_Y.dcm_be[0], &rtb_VectorConcatenate_b[0], 9U *
             sizeof(double));

      /* Outport: '<Root>/fin_angle_rad' incorporates:
       *  ZeroOrderHold: '<Root>/zoh_fin_angle_rad'
       */
      viikate_rocket_Y.fin_angle_rad[0] = servo_lag_idx_0;
      viikate_rocket_Y.fin_angle_rad[1] = servo_lag_idx_1;
      viikate_rocket_Y.fin_angle_rad[2] = servo_lag_idx_2;
      viikate_rocket_Y.fin_angle_rad[3] = servo_lag_idx_3;

      /* Outport: '<Root>/mass_kg' incorporates:
       *  ZeroOrderHold: '<Root>/zoh_mass_kg'
       */
      viikate_rocket_Y.mass_kg = Saturation;

      /* Outport: '<Root>/on_pad' incorporates:
       *  DataTypeConversion: '<Root>/on_pad_dbl'
       *  Logic: '<Root>/pad_flag'
       *  UnitDelay: '<Root>/latch_memory'
       */
      viikate_rocket_Y.on_pad = !viikate_rocket_DW.latch_memory_DSTATE;

      /* If: '<S37>/If1' */
      if (rtsiIsModeUpdateTimeStep(&viikate_rocket_M->solverInfo)) {
        viikate_rocket_DW.If1_ActiveSubsystem = -1;
      }

      /* End of If: '<S37>/If1' */
    }

    /* DotProduct: '<S18>/Dot Product' */
    rtb_ixk_f = ((rtb_q0q1q2q3_idx_0 * rtb_q0q1q2q3_idx_0 + rtb_q0q1q2q3_idx_1 *
                  rtb_q0q1q2q3_idx_1) + rtb_q0q1q2q3_idx_2 * rtb_q0q1q2q3_idx_2)
      + rtb_q0q1q2q3_idx_3 * rtb_q0q1q2q3_idx_3;

    /* SignalConversion generated from: '<S7>/q0 q1 q2 q3' incorporates:
     *  Constant: '<S18>/Constant'
     *  Fcn: '<S18>/q0dot'
     *  Fcn: '<S18>/q1dot'
     *  Fcn: '<S18>/q2dot'
     *  Fcn: '<S18>/q3dot'
     *  Sum: '<S18>/Sum'
     */
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[0] = ((rtb_Sum_fh[0] *
      rtb_q0q1q2q3_idx_1 + rtb_Sum_fh[1] * rtb_q0q1q2q3_idx_2) + rtb_Sum_fh[2] *
      rtb_q0q1q2q3_idx_3) * -0.5 + (1.0 - rtb_ixk_f) * rtb_q0q1q2q3_idx_0;
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[1] = ((rtb_q0q1q2q3_idx_0 *
      rtb_Sum_fh[0] + rtb_q0q1q2q3_idx_2 * rtb_Sum_fh[2]) - rtb_Sum_fh[1] *
      rtb_q0q1q2q3_idx_3) * 0.5 + (1.0 - rtb_ixk_f) * rtb_q0q1q2q3_idx_1;
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[2] = ((rtb_q0q1q2q3_idx_0 *
      rtb_Sum_fh[1] + rtb_Sum_fh[0] * rtb_q0q1q2q3_idx_3) - rtb_q0q1q2q3_idx_1 *
      rtb_Sum_fh[2]) * 0.5 + (1.0 - rtb_ixk_f) * rtb_q0q1q2q3_idx_2;
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[3] = ((rtb_q0q1q2q3_idx_0 *
      rtb_Sum_fh[2] + rtb_q0q1q2q3_idx_1 * rtb_Sum_fh[1]) - rtb_Sum_fh[0] *
      rtb_q0q1q2q3_idx_2) * 0.5 + (1.0 - rtb_ixk_f) * rtb_q0q1q2q3_idx_3;

    /* Integrator: '<S9>/mass  ' */
    /* Limited  Integrator  (w/ Saturation Port) */
    if (viikate_rocket_X.mass_CSTATE >= rp_mass_full_kg) {
      viikate_rocket_X.mass_CSTATE = rp_mass_full_kg;

      /* Integrator: '<S9>/mass  ' */
      mass_o2 = 1;
    } else if (viikate_rocket_X.mass_CSTATE <= rp_mass_full_kg - rp_mass_fuel_kg)
    {
      viikate_rocket_X.mass_CSTATE = rp_mass_full_kg - rp_mass_fuel_kg;

      /* Integrator: '<S9>/mass  ' */
      mass_o2 = -1;
    } else {
      /* Integrator: '<S9>/mass  ' */
      mass_o2 = 0;
    }

    /* SignalConversion generated from: '<S79>/PreLook-Up Index Search' incorporates:
     *  Constant: '<S79>/Constant'
     *  Constant: '<S79>/Constant1'
     */
    rtb_TmpSignalConversionAtPreLoo[0] = rp_mass_full_kg - rp_mass_fuel_kg;
    rtb_TmpSignalConversionAtPreLoo[1] = rp_mass_full_kg;

    /* PreLookup: '<S79>/PreLook-Up Index Search' incorporates:
     *  Integrator: '<S9>/mass  '
     */
    rtb_PreLookUpIndexSearch_o1 = plook_s32dd_bincp(viikate_rocket_X.mass_CSTATE,
      rtb_TmpSignalConversionAtPreLoo, 1U, &rtb_ixj,
      &viikate_rocket_DW.PreLookUpIndexSearch_DWORK1);

    /* LookupNDDirect: '<S81>/[0]' incorporates:
     *  PreLookup: '<S79>/PreLook-Up Index Search'
     *
     * About '<S81>/[0]':
     *  3-dimensional Direct Look-Up returning a 2-D Matrix,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    if (rtb_PreLookUpIndexSearch_o1 > 1) {
      i = 1;
    } else if (rtb_PreLookUpIndexSearch_o1 < 0) {
      i = 0;
    } else {
      i = rtb_PreLookUpIndexSearch_o1;
    }

    tableOffset = i * 9;

    /* Sum: '<S81>/Sum' incorporates:
     *  Constant: '<S81>/offset for upper index 1'
     */
    tmp_1 = rtb_PreLookUpIndexSearch_o1 + INT64_C(1);
    if (rtb_PreLookUpIndexSearch_o1 + INT64_C(1) > INT64_C(2147483647)) {
      tmp_1 = INT64_C(2147483647);
    }

    /* LookupNDDirect: '<S81>/[1]' incorporates:
     *  Sum: '<S81>/Sum'
     *
     * About '<S81>/[1]':
     *  3-dimensional Direct Look-Up returning a 2-D Matrix,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    if ((int32_t)tmp_1 > 1) {
      i = 1;
    } else if ((int32_t)tmp_1 < 0) {
      i = 0;
    } else {
      i = (int32_t)tmp_1;
    }

    /* Gain: '<Root>/mass_rate' */
    viikate_rocket_DW.mass_rate = -rp_mass_fuel_kg * rtb_jxi;

    /* LookupNDDirect: '<S81>/[0]'
     *
     * About '<S81>/[0]':
     *  3-dimensional Direct Look-Up returning a 2-D Matrix,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    memcpy(&rtb_VectorConcatenate_b[0],
           &viikate_rocket_ConstP.pooled8[tableOffset], 9U * sizeof(double));

    /* LookupNDDirect: '<S81>/[1]'
     *
     * About '<S81>/[1]':
     *  3-dimensional Direct Look-Up returning a 2-D Matrix,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    memcpy(&rtb_Selector2[0], &viikate_rocket_ConstP.pooled8[i * 9], 9U * sizeof
           (double));

    /* Product: '<S79>/Product' incorporates:
     *  Constant: '<S79>/slope'
     */
    rtb_ci_idx_2 = rp_mass_full_kg - (rp_mass_full_kg - rp_mass_fuel_kg);

    /* Product: '<S75>/Product' incorporates:
     *  Reshape: '<S76>/Reshape1'
     */
    rtb_q0q1q2q3_idx_0 = 0.0;
    rtb_q0q1q2q3_idx_1 = 0.0;
    rtb_q0q1q2q3_idx_2 = 0.0;
    for (i = 0; i < 3; i++) {
      /* Concatenate: '<S9>/Matrix Concatenation' incorporates:
       *  Constant: '<S79>/slope'
       *  Fcn: '<S81>/Fcn'
       *  Product: '<S79>/Product'
       *  Product: '<S81>/1-lambda_x'
       *  Product: '<S81>/lambda_x.'
       *  Sum: '<S81>/Sum3'
       */
      rtb_MatrixConcatenation[6 * i] = rtb_VectorConcatenate_b[3 * i] * (1.0 -
        rtb_ixj) + rtb_Selector2[3 * i] * rtb_ixj;
      rtb_MatrixConcatenation[6 * i + 3] = (viikate_rocket_ConstP.slope_If[3 * i]
        - viikate_rocket_ConstP.slope_Ie[3 * i]) / rtb_ci_idx_2 *
        viikate_rocket_DW.mass_rate;

      /* Sum: '<S81>/Sum3' incorporates:
       *  Constant: '<S79>/slope'
       *  Product: '<S79>/Product'
       *  Product: '<S81>/1-lambda_x'
       *  Product: '<S81>/lambda_x.'
       */
      tableOffset = 3 * i + 1;

      /* Concatenate: '<S9>/Matrix Concatenation' incorporates:
       *  Constant: '<S79>/slope'
       *  Fcn: '<S81>/Fcn'
       *  Product: '<S75>/Product'
       *  Product: '<S79>/Product'
       *  Product: '<S81>/1-lambda_x'
       *  Product: '<S81>/lambda_x.'
       *  Selector: '<S8>/Selector'
       *  Sum: '<S81>/Sum3'
       */
      rtb_PreLookUpIndexSearch_o1 = 6 * i + 1;
      rtb_MatrixConcatenation[rtb_PreLookUpIndexSearch_o1] = (1.0 - rtb_ixj) *
        rtb_VectorConcatenate_b[tableOffset] + rtb_Selector2[tableOffset] *
        rtb_ixj;
      rtb_MatrixConcatenation[6 * i + 4] =
        (viikate_rocket_ConstP.slope_If[tableOffset] -
         viikate_rocket_ConstP.slope_Ie[tableOffset]) / rtb_ci_idx_2 *
        viikate_rocket_DW.mass_rate;

      /* Sum: '<S81>/Sum3' incorporates:
       *  Constant: '<S79>/slope'
       *  Product: '<S79>/Product'
       *  Product: '<S81>/1-lambda_x'
       *  Product: '<S81>/lambda_x.'
       */
      tableOffset = 3 * i + 2;

      /* Concatenate: '<S9>/Matrix Concatenation' incorporates:
       *  Constant: '<S79>/slope'
       *  Fcn: '<S81>/Fcn'
       *  Product: '<S75>/Product'
       *  Product: '<S79>/Product'
       *  Product: '<S81>/1-lambda_x'
       *  Product: '<S81>/lambda_x.'
       *  Selector: '<S8>/Selector'
       *  Sum: '<S81>/Sum3'
       */
      rtb_MatrixConcatenation_tmp = 6 * i + 2;
      rtb_MatrixConcatenation[rtb_MatrixConcatenation_tmp] = (1.0 - rtb_ixj) *
        rtb_VectorConcatenate_b[tableOffset] + rtb_Selector2[tableOffset] *
        rtb_ixj;
      rtb_MatrixConcatenation[6 * i + 5] =
        (viikate_rocket_ConstP.slope_If[tableOffset] -
         viikate_rocket_ConstP.slope_Ie[tableOffset]) / rtb_ci_idx_2 *
        viikate_rocket_DW.mass_rate;

      /* Product: '<S75>/Product' incorporates:
       *  Concatenate: '<S9>/Matrix Concatenation'
       *  Reshape: '<S75>/Reshape1'
       *  Reshape: '<S76>/Reshape1'
       *  Selector: '<S8>/Selector'
       */
      rtb_q0q1q2q3_idx_3 = rtb_Sum_fh[i];
      rtb_q0q1q2q3_idx_0 += rtb_MatrixConcatenation[6 * i] * rtb_q0q1q2q3_idx_3;
      rtb_q0q1q2q3_idx_1 += rtb_MatrixConcatenation[rtb_PreLookUpIndexSearch_o1]
        * rtb_q0q1q2q3_idx_3;
      rtb_q0q1q2q3_idx_2 += rtb_MatrixConcatenation[rtb_MatrixConcatenation_tmp]
        * rtb_q0q1q2q3_idx_3;
    }

    /* MATLAB Function: '<S85>/MATLAB Function' incorporates:
     *  Constant: '<S85>/Constant'
     */
    rtb_q0q1q2q3_idx_3 = 0.0;
    rtb_jxi = 0.0;
    rtb_ixk_f = 0.0;
    for (i = 0; i < 4; i++) {
      rtb_jxi_n = viikate_rocket_ConstP.Constant_Value_e[i + 4];
      Saturation = viikate_rocket_ConstP.Constant_Value_e[i + 8];
      rtb_ci_idx_2 = rtb_ci[i + 8];
      rtb_Gain2 = rtb_ci[i + 4];
      rtb_ixj = viikate_rocket_ConstP.Constant_Value_e[i];
      rtb_ci_0 = rtb_ci[i];
      rtb_ixk_f += rtb_ixj * rtb_Gain2 - rtb_ci_0 * rtb_jxi_n;
      rtb_q0q1q2q3_idx_3 += rtb_jxi_n * rtb_ci_idx_2 - rtb_Gain2 * Saturation;
      rtb_jxi += rtb_ci_0 * Saturation - rtb_ixj * rtb_ci_idx_2;
    }

    rtb_M[2] = rtb_ixk_f;
    rtb_M[1] = rtb_jxi;
    rtb_M[0] = rtb_q0q1q2q3_idx_3;

    /* End of MATLAB Function: '<S85>/MATLAB Function' */

    /* Product: '<S86>/Element Product' incorporates:
     *  Constant: '<S4>/Constant1'
     */
    rtb_ElementProduct[0] = 0.0 * rtb_Product2_ih[2];
    rtb_q0q1q2q3_idx_3 = rtb_Product2_ih[0] * 0.0;
    rtb_ElementProduct[1] = rtb_q0q1q2q3_idx_3;
    rtb_ElementProduct[2] = -0.346 * rtb_Product2_ih[1];
    rtb_ElementProduct[3] = rtb_Product2_ih[1] * 0.0;
    rtb_ElementProduct[4] = -0.346 * rtb_Product2_ih[2];
    rtb_ElementProduct[5] = rtb_q0q1q2q3_idx_3;

    /* Product: '<S87>/Divide1' incorporates:
     *  Gain: '<S87>/Gain10'
     *  Gain: '<S87>/Gain11'
     *  Gain: '<S87>/Gain3'
     *  Gain: '<S87>/Gain4'
     *  Gain: '<S87>/Gain5'
     *  Gain: '<S87>/Gain6'
     *  Gain: '<S87>/Gain7'
     *  Product: '<S87>/Product2'
     */
    rtb_ci_idx_2 = 1.0 / (2.0 * rtb_V);
    tmp_0[0] = -0.3 * rtb_Sum_fh[0] * 8.663200000000002E-5 * rtb_ixk *
      rtb_ci_idx_2;
    tmp_0[1] = -40.0 * rtb_Sum_fh[1] * 0.007129018000000001 * rtb_ixk *
      rtb_ci_idx_2;
    tmp_0[2] = -40.0 * rtb_Sum_fh[2] * 0.007129018000000001 * rtb_ixk *
      rtb_ci_idx_2;

    /* Product: '<S76>/Product' incorporates:
     *  Concatenate: '<S9>/Matrix Concatenation'
     *  Reshape: '<S76>/Reshape1'
     *  Selector: '<S8>/Selector1'
     */
    rtb_q0q1q2q3_idx_3 = 0.0;
    rtb_jxi = 0.0;
    rtb_V = 0.0;
    for (i = 0; i < 3; i++) {
      rtb_ci_idx_2 = rtb_Sum_fh[i];
      rtb_q0q1q2q3_idx_3 += rtb_MatrixConcatenation[6 * i + 3] * rtb_ci_idx_2;
      rtb_jxi += rtb_MatrixConcatenation[6 * i + 4] * rtb_ci_idx_2;
      rtb_V += rtb_MatrixConcatenation[6 * i + 5] * rtb_ci_idx_2;
    }

    rtb_MatrixConcatenation_0[2] = rtb_V;
    rtb_MatrixConcatenation_0[1] = rtb_jxi;
    rtb_MatrixConcatenation_0[0] = rtb_q0q1q2q3_idx_3;

    /* Sum: '<S74>/Sum' incorporates:
     *  Product: '<S75>/Product'
     *  Product: '<S77>/i x j'
     *  Product: '<S77>/j x k'
     *  Product: '<S77>/k x i'
     *  Product: '<S78>/i x k'
     *  Product: '<S78>/j x i'
     *  Product: '<S78>/k x j'
     *  Reshape: '<S76>/Reshape1'
     */
    rtb_Sum_k[0] = rtb_Sum_fh[1] * rtb_q0q1q2q3_idx_2;
    rtb_Sum_k[1] = rtb_q0q1q2q3_idx_0 * rtb_Sum_fh[2];
    rtb_Sum_k[2] = rtb_Sum_fh[0] * rtb_q0q1q2q3_idx_1;
    rtb_finForces[0] = rtb_q0q1q2q3_idx_1 * rtb_Sum_fh[2];
    rtb_finForces[1] = rtb_Sum_fh[0] * rtb_q0q1q2q3_idx_2;
    rtb_finForces[2] = rtb_q0q1q2q3_idx_0 * rtb_Sum_fh[1];
    for (i = 0; i < 3; i++) {
      /* Product: '<S8>/Product2' incorporates:
       *  Product: '<Root>/hold_moment'
       *  Product: '<S76>/Product'
       *  Sum: '<S10>/Sum'
       *  Sum: '<S4>/Add'
       *  Sum: '<S74>/Sum'
       *  Sum: '<S86>/Sum'
       *  Sum: '<S8>/Sum2'
       */
      rtb_Product2_ih[i] = ((((rtb_ElementProduct[i] - rtb_ElementProduct[i + 3])
        + rtb_M[i]) + tmp_0[i]) * viikate_rocket_DW.released -
                            rtb_MatrixConcatenation_0[i]) - (rtb_Sum_k[i] -
        rtb_finForces[i]);

      /* Selector: '<S8>/Selector2' incorporates:
       *  Concatenate: '<S9>/Matrix Concatenation'
       */
      rtb_VectorConcatenate_b[3 * i] = rtb_MatrixConcatenation[6 * i];
      rtb_VectorConcatenate_b[3 * i + 1] = rtb_MatrixConcatenation[6 * i + 1];
      rtb_VectorConcatenate_b[3 * i + 2] = rtb_MatrixConcatenation[6 * i + 2];
    }

    /* Product: '<S8>/Product2' incorporates:
     *  Reshape: '<S8>/Reshape'
     */
    rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(rtb_Product2_ih, rtb_VectorConcatenate_b,
      viikate_rocket_DW.Reshape);

    /* Switch: '<S9>/Switch' incorporates:
     *  Product: '<S9>/Product'
     */
    if (mass_o2 != 0) {
      rtb_q0q1q2q3_idx_0 = 0.0;
    } else {
      rtb_q0q1q2q3_idx_0 = 0.0 * viikate_rocket_DW.mass_rate;
    }

    /* End of Switch: '<S9>/Switch' */

    /* Sum: '<S2>/Sum' incorporates:
     *  Integrator: '<S9>/mass  '
     *  Product: '<S2>/Product'
     *  Product: '<S82>/i x j'
     *  Product: '<S82>/j x k'
     *  Product: '<S82>/k x i'
     *  Product: '<S83>/i x k'
     *  Product: '<S83>/j x i'
     *  Product: '<S83>/k x j'
     *  Sum: '<S10>/Sum'
     *  Sum: '<S9>/Sum'
     */
    viikate_rocket_DW.Sum[0] = (rtb_hold_force[0] + rtb_q0q1q2q3_idx_0) /
      viikate_rocket_X.mass_CSTATE + (rtb_ubvbwb[1] * rtb_Sum_fh[2] -
      rtb_Sum_fh[1] * rtb_ubvbwb[2]);
    viikate_rocket_DW.Sum[1] = (rtb_hold_force[1] + rtb_q0q1q2q3_idx_0) /
      viikate_rocket_X.mass_CSTATE + (rtb_Sum_fh[0] * rtb_ubvbwb[2] -
      rtb_ubvbwb[0] * rtb_Sum_fh[2]);
    viikate_rocket_DW.Sum[2] = (rtb_hold_force[2] + rtb_q0q1q2q3_idx_0) /
      viikate_rocket_X.mass_CSTATE + (rtb_ubvbwb[0] * rtb_Sum_fh[1] -
      rtb_Sum_fh[0] * rtb_ubvbwb[1]);
    if (tmp) {
      /* RateLimiter: '<Root>/servo_rate_limit' incorporates:
       *  Inport: '<Root>/fin_cmd_rad'
       */
      rtb_ci_idx_2 = rp_servo_rate_limit_radps * viikate_rocket_period;
      rtb_q0q1q2q3_idx_3 = rp_servo_rate_limit_radps * viikate_rocket_period;
      rtb_q0q1q2q3_idx_0 = -rp_servo_rate_limit_radps * viikate_rocket_period;
      rtb_q0q1q2q3_idx_1 = -rp_servo_rate_limit_radps * viikate_rocket_period;
      rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[0] -
        viikate_rocket_DW.PrevY[0];
      if (rtb_q0q1q2q3_idx_2 > rtb_ci_idx_2) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[0] + rtb_q0q1q2q3_idx_3;
      } else if (rtb_q0q1q2q3_idx_2 < rtb_q0q1q2q3_idx_0) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[0] + rtb_q0q1q2q3_idx_1;
      } else {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[0];
      }

      /* RateLimiter: '<Root>/servo_rate_limit' */
      viikate_rocket_DW.servo_rate_limit[0] = rtb_q0q1q2q3_idx_2;

      /* RateLimiter: '<Root>/servo_rate_limit' incorporates:
       *  Inport: '<Root>/fin_cmd_rad'
       */
      viikate_rocket_DW.PrevY[0] = rtb_q0q1q2q3_idx_2;
      rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[1] -
        viikate_rocket_DW.PrevY[1];
      if (rtb_q0q1q2q3_idx_2 > rtb_ci_idx_2) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[1] + rtb_q0q1q2q3_idx_3;
      } else if (rtb_q0q1q2q3_idx_2 < rtb_q0q1q2q3_idx_0) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[1] + rtb_q0q1q2q3_idx_1;
      } else {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[1];
      }

      /* RateLimiter: '<Root>/servo_rate_limit' */
      viikate_rocket_DW.servo_rate_limit[1] = rtb_q0q1q2q3_idx_2;

      /* RateLimiter: '<Root>/servo_rate_limit' incorporates:
       *  Inport: '<Root>/fin_cmd_rad'
       */
      viikate_rocket_DW.PrevY[1] = rtb_q0q1q2q3_idx_2;
      rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[2] -
        viikate_rocket_DW.PrevY[2];
      if (rtb_q0q1q2q3_idx_2 > rtb_ci_idx_2) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[2] + rtb_q0q1q2q3_idx_3;
      } else if (rtb_q0q1q2q3_idx_2 < rtb_q0q1q2q3_idx_0) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[2] + rtb_q0q1q2q3_idx_1;
      } else {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[2];
      }

      /* RateLimiter: '<Root>/servo_rate_limit' */
      viikate_rocket_DW.servo_rate_limit[2] = rtb_q0q1q2q3_idx_2;

      /* RateLimiter: '<Root>/servo_rate_limit' incorporates:
       *  Inport: '<Root>/fin_cmd_rad'
       */
      viikate_rocket_DW.PrevY[2] = rtb_q0q1q2q3_idx_2;
      rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[3] -
        viikate_rocket_DW.PrevY[3];
      if (rtb_q0q1q2q3_idx_2 > rtb_ci_idx_2) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[3] + rtb_q0q1q2q3_idx_3;
      } else if (rtb_q0q1q2q3_idx_2 < rtb_q0q1q2q3_idx_0) {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_DW.PrevY[3] + rtb_q0q1q2q3_idx_1;
      } else {
        /* RateLimiter: '<Root>/servo_rate_limit' */
        rtb_q0q1q2q3_idx_2 = viikate_rocket_U.fin_cmd_rad[3];
      }

      /* RateLimiter: '<Root>/servo_rate_limit' */
      viikate_rocket_DW.servo_rate_limit[3] = rtb_q0q1q2q3_idx_2;

      /* RateLimiter: '<Root>/servo_rate_limit' */
      viikate_rocket_DW.PrevY[3] = rtb_q0q1q2q3_idx_2;
    }

    /* Gain: '<Root>/servo_gain' */
    rtb_ci_idx_2 = 1.0 / rp_servo_tau_s;

    /* Gain: '<Root>/servo_gain' incorporates:
     *  Sum: '<Root>/servo_error'
     */
    viikate_rocket_DW.servo_gain[0] = (viikate_rocket_DW.servo_rate_limit[0] -
      servo_lag_idx_0) * rtb_ci_idx_2;
    viikate_rocket_DW.servo_gain[1] = (viikate_rocket_DW.servo_rate_limit[1] -
      servo_lag_idx_1) * rtb_ci_idx_2;
    viikate_rocket_DW.servo_gain[2] = (viikate_rocket_DW.servo_rate_limit[2] -
      servo_lag_idx_2) * rtb_ci_idx_2;
    viikate_rocket_DW.servo_gain[3] = (viikate_rocket_DW.servo_rate_limit[3] -
      servo_lag_idx_3) * rtb_ci_idx_2;
    if (rtmIsMajorTimeStep(viikate_rocket_M) &&
        viikate_rocket_M->Timing.TaskCounters.TID[2] == 0) {
      /* RandomNumber: '<S6>/Random Number1' */
      viikate_rocket_DW.RandomNumber1 = viikate_rocket_DW.NextOutput;

      /* RandomNumber: '<S6>/Random Number2' */
      viikate_rocket_DW.RandomNumber2 = viikate_rocket_DW.NextOutput_a;
    }

    /* SignalConversion generated from: '<S6>/Integrator' incorporates:
     *  Constant: '<S6>/Constant3'
     */
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[0] = 0.0;
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[1] =
      viikate_rocket_DW.RandomNumber1;
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[2] =
      viikate_rocket_DW.RandomNumber2;
  }

  if (rtmIsMajorTimeStep(viikate_rocket_M)) {
    double NextOutput_tmp;

    /* Update for Integrator: '<S7>/q0 q1 q2 q3' */
    viikate_rocket_DW.q0q1q2q3_DWORK1 = false;
    if (rtmIsMajorTimeStep(viikate_rocket_M) &&
        viikate_rocket_M->Timing.TaskCounters.TID[2] == 0) {
      /* Update for RandomNumber: '<S6>/Random Number1' incorporates:
       *  RandomNumber: '<S6>/Random Number2'
       */
      NextOutput_tmp = sqrt(rp_gust_variance);
      viikate_rocket_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&viikate_rocket_DW.RandSeed) * NextOutput_tmp;

      /* Update for RandomNumber: '<S6>/Random Number2' */
      viikate_rocket_DW.NextOutput_a = rt_nrand_Upu32_Yd_f_pw_snf
        (&viikate_rocket_DW.RandSeed_e) * NextOutput_tmp;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(viikate_rocket_M)) {
    rt_ertODEUpdateContinuousStates(&viikate_rocket_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++viikate_rocket_M->Timing.clockTick0;
    viikate_rocket_M->Timing.t[0] = rtsiGetSolverStopTime
      (&viikate_rocket_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      viikate_rocket_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void viikate_rocket_derivatives(void)
{
  XDot_viikate_rocket_T *_rtXdot;
  bool lsat;
  bool usat;
  _rtXdot = ((XDot_viikate_rocket_T *) viikate_rocket_M->derivs);

  /* Derivatives for Integrator: '<S7>/q0 q1 q2 q3' */
  _rtXdot->q0q1q2q3_CSTATE[0] =
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[0];

  /* Derivatives for Integrator: '<Root>/servo_lag' */
  _rtXdot->servo_lag_CSTATE[0] = viikate_rocket_DW.servo_gain[0];

  /* Derivatives for Integrator: '<S7>/q0 q1 q2 q3' */
  _rtXdot->q0q1q2q3_CSTATE[1] =
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[1];

  /* Derivatives for Integrator: '<Root>/servo_lag' */
  _rtXdot->servo_lag_CSTATE[1] = viikate_rocket_DW.servo_gain[1];

  /* Derivatives for Integrator: '<S7>/q0 q1 q2 q3' */
  _rtXdot->q0q1q2q3_CSTATE[2] =
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[2];

  /* Derivatives for Integrator: '<Root>/servo_lag' */
  _rtXdot->servo_lag_CSTATE[2] = viikate_rocket_DW.servo_gain[2];

  /* Derivatives for Integrator: '<S7>/q0 q1 q2 q3' */
  _rtXdot->q0q1q2q3_CSTATE[3] =
    viikate_rocket_DW.TmpSignalConversionAtq0q1q2q3_g[3];

  /* Derivatives for Integrator: '<Root>/servo_lag' */
  _rtXdot->servo_lag_CSTATE[3] = viikate_rocket_DW.servo_gain[3];

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] =
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[0];

  /* Derivatives for Integrator: '<S2>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[0] = viikate_rocket_DW.Sum[0];

  /* Derivatives for Integrator: '<S2>/p,q,r ' */
  _rtXdot->pqr_CSTATE[0] = viikate_rocket_DW.Reshape[0];

  /* Derivatives for Integrator: '<S2>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[0] = viikate_rocket_DW.Reshape2[0];

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE[1] =
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[1];

  /* Derivatives for Integrator: '<S2>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[1] = viikate_rocket_DW.Sum[1];

  /* Derivatives for Integrator: '<S2>/p,q,r ' */
  _rtXdot->pqr_CSTATE[1] = viikate_rocket_DW.Reshape[1];

  /* Derivatives for Integrator: '<S2>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[1] = viikate_rocket_DW.Reshape2[1];

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE[2] =
    viikate_rocket_DW.TmpSignalConversionAtIntegrator[2];

  /* Derivatives for Integrator: '<S2>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[2] = viikate_rocket_DW.Sum[2];

  /* Derivatives for Integrator: '<S2>/p,q,r ' */
  _rtXdot->pqr_CSTATE[2] = viikate_rocket_DW.Reshape[2];

  /* Derivatives for Integrator: '<S2>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[2] = viikate_rocket_DW.Reshape2[2];

  /* Derivatives for Integrator: '<S9>/mass  ' */
  lsat = (viikate_rocket_X.mass_CSTATE <= rp_mass_full_kg - rp_mass_fuel_kg);
  usat = (viikate_rocket_X.mass_CSTATE >= rp_mass_full_kg);
  if ((!lsat && !usat) || (lsat && (viikate_rocket_DW.mass_rate > 0.0)) || (usat
       && (viikate_rocket_DW.mass_rate < 0.0))) {
    _rtXdot->mass_CSTATE = viikate_rocket_DW.mass_rate;
  } else {
    /* in saturation */
    _rtXdot->mass_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S9>/mass  ' */
}

/* Model initialize function */
void viikate_rocket_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&viikate_rocket_M->solverInfo,
                          &viikate_rocket_M->Timing.simTimeStep);
    rtsiSetTPtr(&viikate_rocket_M->solverInfo, &rtmGetTPtr(viikate_rocket_M));
    rtsiSetStepSizePtr(&viikate_rocket_M->solverInfo,
                       &viikate_rocket_M->Timing.stepSize0);
    rtsiSetdXPtr(&viikate_rocket_M->solverInfo, &viikate_rocket_M->derivs);
    rtsiSetContStatesPtr(&viikate_rocket_M->solverInfo, (double **)
                         &viikate_rocket_M->contStates);
    rtsiSetNumContStatesPtr(&viikate_rocket_M->solverInfo,
      &viikate_rocket_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&viikate_rocket_M->solverInfo,
      &viikate_rocket_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&viikate_rocket_M->solverInfo,
      &viikate_rocket_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&viikate_rocket_M->solverInfo,
      &viikate_rocket_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&viikate_rocket_M->solverInfo, (bool**)
      &viikate_rocket_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&viikate_rocket_M->solverInfo, (&rtmGetErrorStatus
      (viikate_rocket_M)));
    rtsiSetRTModelPtr(&viikate_rocket_M->solverInfo, viikate_rocket_M);
  }

  rtsiSetSimTimeStep(&viikate_rocket_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&viikate_rocket_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&viikate_rocket_M->solverInfo, false);
  viikate_rocket_M->intgData.y = viikate_rocket_M->odeY;
  viikate_rocket_M->intgData.f[0] = viikate_rocket_M->odeF[0];
  viikate_rocket_M->intgData.f[1] = viikate_rocket_M->odeF[1];
  viikate_rocket_M->intgData.f[2] = viikate_rocket_M->odeF[2];
  viikate_rocket_M->intgData.f[3] = viikate_rocket_M->odeF[3];
  viikate_rocket_M->contStates = ((X_viikate_rocket_T *) &viikate_rocket_X);
  viikate_rocket_M->contStateDisabled = ((XDis_viikate_rocket_T *)
    &viikate_rocket_XDis);
  viikate_rocket_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&viikate_rocket_M->solverInfo, (void *)
                    &viikate_rocket_M->intgData);
  rtsiSetSolverName(&viikate_rocket_M->solverInfo,"ode4");
  rtmSetTPtr(viikate_rocket_M, &viikate_rocket_M->Timing.tArray[0]);
  viikate_rocket_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(viikate_rocket_M, 1);

  {
    double NextOutput_tmp;
    double tmp;
    uint32_t r;
    uint32_t t;
    uint32_t tseed;

    /* Start for If: '<S32>/If' */
    viikate_rocket_DW.If_ActiveSubsystem = -1;

    /* Start for If: '<S37>/If1' */
    viikate_rocket_DW.If1_ActiveSubsystem = -1;

    /* InitializeConditions for Integrator: '<S7>/q0 q1 q2 q3' */
    if (rtmIsFirstInitCond(viikate_rocket_M)) {
      viikate_rocket_X.q0q1q2q3_CSTATE[0] = 0.0;
      viikate_rocket_X.q0q1q2q3_CSTATE[1] = 0.0;
      viikate_rocket_X.q0q1q2q3_CSTATE[2] = 0.0;
      viikate_rocket_X.q0q1q2q3_CSTATE[3] = 0.0;
    }

    viikate_rocket_DW.q0q1q2q3_DWORK1 = true;

    /* End of InitializeConditions for Integrator: '<S7>/q0 q1 q2 q3' */

    /* InitializeConditions for Integrator: '<Root>/servo_lag' */
    viikate_rocket_X.servo_lag_CSTATE[0] = 0.0;
    viikate_rocket_X.servo_lag_CSTATE[1] = 0.0;
    viikate_rocket_X.servo_lag_CSTATE[2] = 0.0;
    viikate_rocket_X.servo_lag_CSTATE[3] = 0.0;

    /* InitializeConditions for Integrator: '<S6>/Integrator' */
    viikate_rocket_X.Integrator_CSTATE[0] = rp_wind_model_mps[0];

    /* InitializeConditions for Integrator: '<S2>/ub,vb,wb' */
    viikate_rocket_X.ubvbwb_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/p,q,r ' */
    viikate_rocket_X.pqr_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/xe,ye,ze' */
    viikate_rocket_X.xeyeze_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S6>/Integrator' */
    viikate_rocket_X.Integrator_CSTATE[1] = rp_wind_model_mps[1];

    /* InitializeConditions for Integrator: '<S2>/ub,vb,wb' */
    viikate_rocket_X.ubvbwb_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/p,q,r ' */
    viikate_rocket_X.pqr_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/xe,ye,ze' */
    viikate_rocket_X.xeyeze_CSTATE[1] = 50.0;

    /* InitializeConditions for Integrator: '<S6>/Integrator' */
    viikate_rocket_X.Integrator_CSTATE[2] = rp_wind_model_mps[2];

    /* InitializeConditions for Integrator: '<S2>/ub,vb,wb' */
    viikate_rocket_X.ubvbwb_CSTATE[2] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/p,q,r ' */
    viikate_rocket_X.pqr_CSTATE[2] = 0.0;

    /* InitializeConditions for Integrator: '<S2>/xe,ye,ze' */
    viikate_rocket_X.xeyeze_CSTATE[2] = 50.0;

    /* InitializeConditions for Integrator: '<S9>/mass  ' */
    viikate_rocket_X.mass_CSTATE = rp_mass_full_kg;

    /* InitializeConditions for RandomNumber: '<S6>/Random Number1' incorporates:
     *  RandomNumber: '<S6>/Random Number2'
     */
    tmp = floor(rp_gust_seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_t)-(int32_t)(uint32_t)-tmp : (uint32_t)tmp;
    r = tseed >> 16U;
    t = tseed & 32768U;
    viikate_rocket_DW.RandSeed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (viikate_rocket_DW.RandSeed < 1U) {
      viikate_rocket_DW.RandSeed = 1144108930U;
    } else if (viikate_rocket_DW.RandSeed > 2147483646U) {
      viikate_rocket_DW.RandSeed = 2147483646U;
    }

    NextOutput_tmp = sqrt(rp_gust_variance);
    viikate_rocket_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&viikate_rocket_DW.RandSeed) * NextOutput_tmp;

    /* End of InitializeConditions for RandomNumber: '<S6>/Random Number1' */

    /* InitializeConditions for RandomNumber: '<S6>/Random Number2' */
    tmp = floor(rp_gust_seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_t)-(int32_t)(uint32_t)-tmp : (uint32_t)tmp;
    r = tseed >> 16U;
    t = tseed & 32768U;
    viikate_rocket_DW.RandSeed_e = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (viikate_rocket_DW.RandSeed_e < 1U) {
      viikate_rocket_DW.RandSeed_e = 1144108930U;
    } else if (viikate_rocket_DW.RandSeed_e > 2147483646U) {
      viikate_rocket_DW.RandSeed_e = 2147483646U;
    }

    viikate_rocket_DW.NextOutput_a = rt_nrand_Upu32_Yd_f_pw_snf
      (&viikate_rocket_DW.RandSeed_e) * NextOutput_tmp;

    /* SystemInitialize for IfAction SubSystem: '<S32>/AxisRotZeroR3' */
    /* Start for If: '<S42>/If' */
    viikate_rocket_DW.If_ActiveSubsystem_o = -1;

    /* End of SystemInitialize for SubSystem: '<S32>/AxisRotZeroR3' */

    /* SystemInitialize for IfAction SubSystem: '<S32>/AxisRotDefault' */
    /* Start for If: '<S38>/If' */
    viikate_rocket_DW.If_ActiveSubsystem_d = -1;

    /* End of SystemInitialize for SubSystem: '<S32>/AxisRotDefault' */
  }

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(viikate_rocket_M)) {
    rtmSetFirstInitCond(viikate_rocket_M, 0);
  }
}

/* Model terminate function */
void viikate_rocket_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
