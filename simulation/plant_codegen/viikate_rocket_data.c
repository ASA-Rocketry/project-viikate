/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: viikate_rocket_data.c
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
  /* Variable: Ie
   * Referenced by: '<S79>/slope'
   */
  { 0.0027, 0.0, 0.0, 0.0, 0.1655, 0.0, 0.0, 0.0, 0.1655 },

  /* Variable: If
   * Referenced by: '<S79>/slope'
   */
  { 0.0027, 0.0, 0.0, 0.0, 0.177, 0.0, 0.0, 0.0, 0.177 },

  /* Expression: gravity
   * Referenced by: '<S1>/Constant'
   */
  { -9.819, 0.0, 0.0 },

  /* Expression: fin_positions
   * Referenced by: '<S85>/Constant'
   */
  { 0.4689, 0.4689, 0.4689, 0.4689, 0.0, -0.0596975, 0.0, 0.0596975, 0.0596975,
    0.0, -0.0596975, 0.0 },

  /* Expression: N
   * Referenced by: '<S85>/Lift directions'
   */
  { 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0 },

  /* Expression: thrust_n
   * Referenced by: '<S1>/thrust_curve'
   */
  { 89.654, 117.24, 119.654, 111.033, 124.826, 134.136, 149.308, 155.17, 138.274,
    133.791, 88.964, 44.482, 22.414, 2.069, 0.0 },

  /* Expression: thrust_t_s
   * Referenced by: '<S1>/thrust_curve'
   */
  { 0.008, 0.017, 0.027, 0.047, 0.115, 0.2, 0.399, 0.601, 0.8, 0.824, 0.878,
    0.914, 0.933, 0.974, 1.001 },

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
