/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: rt_nonfinite.h
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

#ifndef rt_nonfinite_h_
#define rt_nonfinite_h_
#include <stdbool.h>

extern double rtInf;
extern double rtMinusInf;
extern double rtNaN;
extern float rtInfF;
extern float rtMinusInfF;
extern float rtNaNF;
extern bool rtIsInf(double value);
extern bool rtIsInfF(float value);
extern bool rtIsNaN(double value);
extern bool rtIsNaNF(float value);

#endif                                 /* rt_nonfinite_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
