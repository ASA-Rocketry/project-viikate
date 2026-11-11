/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: viikate_rocket_data.c
 *
 * Code generated for Simulink model 'viikate_rocket'.
 *
 * Model version                  : 5.3
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Wed Nov 11 15:32:51 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "viikate_rocket.h"

/* Invariant block signals (default storage) */
const ConstB_viikate_rocket_T viikate_rocket_ConstB = {
  { 0.0, 0.0, 0.0 },                   /* '<S17>/1//2' */

  { 0.0, 0.0, 0.0 },                   /* '<S17>/sincos' */

  { 1.0, 1.0, 1.0 },                   /* '<S17>/sincos' */
  1.0,                                 /* '<S17>/q0' */
  0.0,                                 /* '<S17>/q1' */
  0.0,                                 /* '<S17>/q2' */
  0.0                                  /* '<S17>/q3' */
};

/* Constant parameters (default storage) */
const ConstP_viikate_rocket_T viikate_rocket_ConstP = {
  /* Expression: gravity
   * Referenced by: '<S1>/Constant'
   */
  { -9.819, 0.0, 0.0 },

  /* Expression: N
   * Referenced by: '<S85>/Lift directions'
   */
  { 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0 },

  /* Pooled Parameter (Expression: matrix)
   * Referenced by:
   *   '<S81>/[0]'
   *   '<S81>/[1]'
   */
  { 0.0027, 0.0, 0.0, 0.0, 0.1655, 0.0, 0.0, 0.0, 0.1655, 0.0027, 0.0, 0.0, 0.0,
    0.177, 0.0, 0.0, 0.0, 0.177 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
