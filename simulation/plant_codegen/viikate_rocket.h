/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: viikate_rocket.h
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

#ifndef viikate_rocket_h_
#define viikate_rocket_h_
#ifndef viikate_rocket_COMMON_INCLUDES_
#define viikate_rocket_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* viikate_rocket_COMMON_INCLUDES_ */

#include "viikate_rocket_types.h"
#include "rtGetNaN.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  double Reshape2[3];                  /* '<S14>/Reshape2' */
  double Merge[3];                     /* '<S32>/Merge' */
  double TmpSignalConversionAtq0q1q2q3_g[4];/* '<S7>/qdot' */
  double Reshape[3];                   /* '<S8>/Reshape' */
  double Sum[3];                       /* '<S2>/Sum' */
  double servo_rate_limit[4];          /* '<Root>/servo_rate_limit' */
  double servo_gain[4];                /* '<Root>/servo_gain' */
  double TmpSignalConversionAtIntegrator[3];
  double Product2_DWORK4[9];           /* '<S8>/Product2' */
  double PrevY[4];                     /* '<Root>/servo_rate_limit' */
  double released;                     /* '<Root>/released' */
  double mass_rate;                    /* '<Root>/mass_rate' */
  double RandomNumber1;                /* '<S6>/Random Number1' */
  double RandomNumber2;                /* '<S6>/Random Number2' */
  double Merge_o;                      /* '<S38>/Merge' */
  double Merge_k;                      /* '<S42>/Merge' */
  double NextOutput;                   /* '<S6>/Random Number1' */
  double NextOutput_a;                 /* '<S6>/Random Number2' */
  int32_t PreLookUpIndexSearch_DWORK1; /* '<S79>/PreLook-Up Index Search' */
  uint32_t RandSeed;                   /* '<S6>/Random Number1' */
  uint32_t RandSeed_e;                 /* '<S6>/Random Number2' */
  int8_t If_ActiveSubsystem;           /* '<S32>/If' */
  int8_t If1_ActiveSubsystem;          /* '<S37>/If1' */
  int8_t If_ActiveSubsystem_d;         /* '<S38>/If' */
  int8_t If_ActiveSubsystem_o;         /* '<S42>/If' */
  bool latch_memory_DSTATE;            /* '<Root>/latch_memory' */
  bool q0q1q2q3_DWORK1;                /* '<S7>/q0 q1 q2 q3' */
} DW_viikate_rocket_T;

/* Continuous states (default storage) */
typedef struct {
  double q0q1q2q3_CSTATE[4];           /* '<S7>/q0 q1 q2 q3' */
  double servo_lag_CSTATE[4];          /* '<Root>/servo_lag' */
  double Integrator_CSTATE[3];         /* '<S6>/Integrator' */
  double ubvbwb_CSTATE[3];             /* '<S2>/ub,vb,wb' */
  double pqr_CSTATE[3];                /* '<S2>/p,q,r ' */
  double xeyeze_CSTATE[3];             /* '<S2>/xe,ye,ze' */
  double mass_CSTATE;                  /* '<S9>/mass  ' */
} X_viikate_rocket_T;

/* State derivatives (default storage) */
typedef struct {
  double q0q1q2q3_CSTATE[4];           /* '<S7>/q0 q1 q2 q3' */
  double servo_lag_CSTATE[4];          /* '<Root>/servo_lag' */
  double Integrator_CSTATE[3];         /* '<S6>/Integrator' */
  double ubvbwb_CSTATE[3];             /* '<S2>/ub,vb,wb' */
  double pqr_CSTATE[3];                /* '<S2>/p,q,r ' */
  double xeyeze_CSTATE[3];             /* '<S2>/xe,ye,ze' */
  double mass_CSTATE;                  /* '<S9>/mass  ' */
} XDot_viikate_rocket_T;

/* State disabled  */
typedef struct {
  bool q0q1q2q3_CSTATE[4];             /* '<S7>/q0 q1 q2 q3' */
  bool servo_lag_CSTATE[4];            /* '<Root>/servo_lag' */
  bool Integrator_CSTATE[3];           /* '<S6>/Integrator' */
  bool ubvbwb_CSTATE[3];               /* '<S2>/ub,vb,wb' */
  bool pqr_CSTATE[3];                  /* '<S2>/p,q,r ' */
  bool xeyeze_CSTATE[3];               /* '<S2>/xe,ye,ze' */
  bool mass_CSTATE;                    /* '<S9>/mass  ' */
} XDis_viikate_rocket_T;

/* Invariant block signals (default storage) */
typedef struct {
  const double u2[3];                  /* '<S17>/1//2' */
  const double sincos_o1[3];           /* '<S17>/sincos' */
  const double sincos_o2[3];           /* '<S17>/sincos' */
  const double q0;                     /* '<S17>/q0' */
  const double q1;                     /* '<S17>/q1' */
  const double q2;                     /* '<S17>/q2' */
  const double q3;                     /* '<S17>/q3' */
} ConstB_viikate_rocket_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  double *y;                           /* output */
  double *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Variable: Ie
   * Referenced by: '<S79>/slope'
   */
  double slope_Ie[9];

  /* Variable: If
   * Referenced by: '<S79>/slope'
   */
  double slope_If[9];

  /* Expression: gravity
   * Referenced by: '<S1>/Constant'
   */
  double Constant_Value_c[3];

  /* Expression: fin_positions
   * Referenced by: '<S85>/Constant'
   */
  double Constant_Value_e[12];

  /* Expression: N
   * Referenced by: '<S85>/Lift directions'
   */
  double Liftdirections_Value[12];

  /* Expression: thrust_n
   * Referenced by: '<S1>/thrust_curve'
   */
  double thrust_curve_tableData[15];

  /* Expression: thrust_t_s
   * Referenced by: '<S1>/thrust_curve'
   */
  double thrust_curve_bp01Data[15];

  /* Pooled Parameter (Expression: matrix)
   * Referenced by:
   *   '<S81>/[0]'
   *   '<S81>/[1]'
   */
  double pooled8[18];
} ConstP_viikate_rocket_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  double fin_cmd_rad[4];               /* '<Root>/fin_cmd_rad' */
} ExtU_viikate_rocket_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  double sf_body_mps2[3];              /* '<Root>/sf_body_mps2' */
  double omega_body_radps[3];          /* '<Root>/omega_body_radps' */
  double pos_model_m[3];               /* '<Root>/pos_model_m' */
  double vel_model_mps[3];             /* '<Root>/vel_model_mps' */
  double euler_rad[3];                 /* '<Root>/euler_rad' */
  double dcm_be[9];                    /* '<Root>/dcm_be' */
  double fin_angle_rad[4];             /* '<Root>/fin_angle_rad' */
  double mass_kg;                      /* '<Root>/mass_kg' */
  double on_pad;                       /* '<Root>/on_pad' */
} ExtY_viikate_rocket_T;

/* Real-time Model Data Structure */
struct tag_RTM_viikate_rocket_T {
  const char *errorStatus;
  RTWSolverInfo solverInfo;
  X_viikate_rocket_T *contStates;
  int *periodicContStateIndices;
  double *periodicContStateRanges;
  double *derivs;
  XDis_viikate_rocket_T *contStateDisabled;
  bool zCCacheNeedsReset;
  bool derivCacheNeedsReset;
  bool CTOutputIncnstWithState;
  double odeY[21];
  double odeF[4][21];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int numContStates;
    int numPeriodicContStates;
    int numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_t clockTick0;
    double stepSize0;
    uint32_t clockTick1;
    bool firstInitCondFlag;
    struct {
      uint8_t TID[3];
    } TaskCounters;

    double tStart;
    SimTimeStep simTimeStep;
    bool stopRequestedFlag;
    double *t;
    double tArray[3];
  } Timing;
};

/* Continuous states (default storage) */
extern X_viikate_rocket_T viikate_rocket_X;

/* Disabled states (default storage) */
extern XDis_viikate_rocket_T viikate_rocket_XDis;

/* Block signals and states (default storage) */
extern DW_viikate_rocket_T viikate_rocket_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_viikate_rocket_T viikate_rocket_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_viikate_rocket_T viikate_rocket_Y;
extern const ConstB_viikate_rocket_T viikate_rocket_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_viikate_rocket_T viikate_rocket_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern double rp_gust_seed[2];         /* Variable: rp_gust_seed
                                        * Referenced by:
                                        *   '<S6>/Random Number1'
                                        *   '<S6>/Random Number2'
                                        */
extern double rp_gust_variance;        /* Variable: rp_gust_variance
                                        * Referenced by:
                                        *   '<S6>/Random Number1'
                                        *   '<S6>/Random Number2'
                                        */
extern double rp_ignition_time_s;      /* Variable: rp_ignition_time_s
                                        * Referenced by: '<Root>/ignition_time'
                                        */
extern double rp_mass_fuel_kg;         /* Variable: rp_mass_fuel_kg
                                        * Referenced by:
                                        *   '<Root>/mass_rate'
                                        *   '<S5>/Constant2'
                                        *   '<S5>/Saturation'
                                        *   '<S9>/mass  '
                                        *   '<S79>/Constant'
                                        *   '<S79>/slope'
                                        */
extern double rp_mass_full_kg;         /* Variable: rp_mass_full_kg
                                        * Referenced by:
                                        *   '<S5>/Constant1'
                                        *   '<S5>/Saturation'
                                        *   '<S9>/mass  '
                                        *   '<S79>/Constant'
                                        *   '<S79>/Constant1'
                                        *   '<S79>/slope'
                                        */
extern double rp_servo_rate_limit_radps;/* Variable: rp_servo_rate_limit_radps
                                         * Referenced by: '<Root>/servo_rate_limit'
                                         */
extern double rp_servo_tau_s;          /* Variable: rp_servo_tau_s
                                        * Referenced by: '<Root>/servo_gain'
                                        */
extern double rp_thrust_scale;         /* Variable: rp_thrust_scale
                                        * Referenced by: '<S1>/thrust_scale'
                                        */
extern double rp_wind_model_mps[3];    /* Variable: rp_wind_model_mps
                                        * Referenced by: '<S6>/Integrator'
                                        */

/* Model entry point functions */
extern void viikate_rocket_initialize(void);
extern void viikate_rocket_step(void);
extern void viikate_rocket_terminate(void);

/* Real-time Model object */
extern RT_MODEL_viikate_rocket_T *const viikate_rocket_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S18>/High Gain Quaternion Normalization' : Eliminated nontunable gain of 1
 * Block '<S11>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S12>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S13>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<Root>/zoh_on_pad' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'viikate_rocket'
 * '<S1>'   : 'viikate_rocket/Forces'
 * '<S2>'   : 'viikate_rocket/Simple Variable Mass 6DOF'
 * '<S3>'   : 'viikate_rocket/Subsystem'
 * '<S4>'   : 'viikate_rocket/aerodynamics'
 * '<S5>'   : 'viikate_rocket/mass calculation'
 * '<S6>'   : 'viikate_rocket/wind'
 * '<S7>'   : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles'
 * '<S8>'   : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot'
 * '<S9>'   : 'viikate_rocket/Simple Variable Mass 6DOF/Determine Force,  Mass & Inertia'
 * '<S10>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Vbxw'
 * '<S11>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Velocity Conversion'
 * '<S12>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Velocity Conversion1'
 * '<S13>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Velocity Conversion2'
 * '<S14>'  : 'viikate_rocket/Simple Variable Mass 6DOF/transform to Inertial axes '
 * '<S15>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix'
 * '<S16>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles'
 * '<S17>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Rotation Angles to Quaternions'
 * '<S18>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/qdot'
 * '<S19>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A11'
 * '<S20>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A12'
 * '<S21>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A13'
 * '<S22>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A21'
 * '<S23>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A22'
 * '<S24>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A23'
 * '<S25>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A31'
 * '<S26>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A32'
 * '<S27>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A33'
 * '<S28>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S29>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S30>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S31>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S32>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang'
 * '<S33>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM'
 * '<S34>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotDefault'
 * '<S35>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotZeroR3'
 * '<S36>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Get DCM Values'
 * '<S37>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM'
 * '<S38>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotDefault/Protect asincos input'
 * '<S39>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotDefault/Protect asincos input/If Action Subsystem'
 * '<S40>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotDefault/Protect asincos input/If Action Subsystem1'
 * '<S41>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotDefault/Protect asincos input/If Action Subsystem2'
 * '<S42>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotZeroR3/Protect asincos input'
 * '<S43>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotZeroR3/Protect asincos input/If Action Subsystem'
 * '<S44>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotZeroR3/Protect asincos input/If Action Subsystem1'
 * '<S45>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/AxisRotZeroR3/Protect asincos input/If Action Subsystem2'
 * '<S46>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error'
 * '<S47>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S48>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/Else No Action'
 * '<S49>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/If Not Proper'
 * '<S50>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S51>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/isNotProper'
 * '<S52>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S53>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/Else If Not Orthogonal/None'
 * '<S54>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S55>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S56>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/If Not Proper/None'
 * '<S57>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S58>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S59>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S60>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/DCM2Ang/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 * '<S61>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A11'
 * '<S62>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A12'
 * '<S63>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A13'
 * '<S64>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A21'
 * '<S65>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A22'
 * '<S66>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A23'
 * '<S67>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A31'
 * '<S68>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A32'
 * '<S69>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/A33'
 * '<S70>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/Create 3x3 Matrix'
 * '<S71>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/Quaternion Normalize'
 * '<S72>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/Quaternion Normalize/Quaternion Modulus'
 * '<S73>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quat2DCM/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S74>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot/3x3 Cross Product'
 * '<S75>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot/I x w'
 * '<S76>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot/I x w1'
 * '<S77>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S78>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S79>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Determine Force,  Mass & Inertia/Estimate Inertia Tensor'
 * '<S80>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Determine Force,  Mass & Inertia/Estimate Inertia Tensor/Interpolate Inertia '
 * '<S81>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Determine Force,  Mass & Inertia/Estimate Inertia Tensor/Interpolate Inertia /Matrix interpolation'
 * '<S82>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Vbxw/Subsystem'
 * '<S83>'  : 'viikate_rocket/Simple Variable Mass 6DOF/Vbxw/Subsystem1'
 * '<S84>'  : 'viikate_rocket/aerodynamics/Angle of attack'
 * '<S85>'  : 'viikate_rocket/aerodynamics/Canard fin forces'
 * '<S86>'  : 'viikate_rocket/aerodynamics/Cross Product'
 * '<S87>'  : 'viikate_rocket/aerodynamics/Damping moments'
 * '<S88>'  : 'viikate_rocket/aerodynamics/Drag'
 * '<S89>'  : 'viikate_rocket/aerodynamics/Canard fin forces/MATLAB Function'
 * '<S90>'  : 'viikate_rocket/aerodynamics/Canard fin forces/MATLAB Function1'
 * '<S91>'  : 'viikate_rocket/aerodynamics/Canard fin forces/MATLAB Function2'
 * '<S92>'  : 'viikate_rocket/aerodynamics/Canard fin forces/MATLAB Function3'
 * '<S93>'  : 'viikate_rocket/aerodynamics/Canard fin forces/MATLAB Function4'
 */
#endif                                 /* viikate_rocket_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
