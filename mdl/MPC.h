/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MPC.h
 *
 * Code generated for Simulink model 'MPC'.
 *
 * Model version                  : 1.139
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Mon Apr 27 05:07:49 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MPC_h_
#define RTW_HEADER_MPC_h_
#ifndef MPC_COMMON_INCLUDES_
#define MPC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* MPC_COMMON_INCLUDES_ */

#include <stddef.h>
#include "MPC_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include <string.h>

/* Block signals (default storage) */
typedef struct {
  real32_T Su[144];
  real32_T H_qp[144];
  real32_T H_qp_tmp[144];
  real32_T Su_m[144];
  real32_T Su_c[144];
  real32_T RLinv[144];
  real32_T D[144];
  real32_T H[144];
  real32_T U[144];
  real32_T TL[144];
  real32_T Q[144];
  real32_T R[144];
  int16_T T[144];
  int32_T iC[24];
  real32_T a[24];
  real32_T cTol[24];
  real32_T Opt[24];
} B_MPC_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE[2];/* '<S4>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE[2];        /* '<S2>/Unit Delay' */
  boolean_T iA_prev[24];               /* '<S2>/MPC' */
} DW_MPC_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T Te_ref;                     /* '<Root>/Te_ref' */
  real32_T wm;                         /* '<Root>/wm' */
  real32_T Id_meas;                    /* '<Root>/Id_meas' */
  real32_T Iq_meas;                    /* '<Root>/Iq_meas' */
} ExtU_MPC_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Ud_out;                     /* '<Root>/Ud_out' */
  real32_T Uq_out;                     /* '<Root>/Uq_out' */
} ExtY_MPC_T;

/* Parameters (default storage) */
struct P_MPC_T_ {
  real_T J;                            /* Variable: J
                                        * Referenced by: '<S4>/MATLAB Function'
                                        */
  real32_T B;                          /* Variable: B
                                        * Referenced by: '<S4>/MATLAB Function'
                                        */
  real32_T LUT_Id[240];                /* Variable: LUT_Id
                                        * Referenced by: '<S3>/Interpolation Using Prelookup'
                                        */
  real32_T LUT_Iq[240];                /* Variable: LUT_Iq
                                        * Referenced by: '<S3>/Interpolation Using Prelookup1'
                                        */
  real32_T Ld;                         /* Variable: Ld
                                        * Referenced by: '<S2>/MPC'
                                        */
  real32_T Lq;                         /* Variable: Lq
                                        * Referenced by: '<S2>/MPC'
                                        */
  real32_T Rs;                         /* Variable: Rs
                                        * Referenced by: '<S2>/MPC'
                                        */
  real32_T Tsw;                        /* Variable: Tsw
                                        * Referenced by: '<S2>/MPC'
                                        */
  real32_T p;                          /* Variable: p
                                        * Referenced by: '<S1>/Gain'
                                        */
  real32_T psi_m;                      /* Variable: psi_m
                                        * Referenced by: '<S2>/MPC'
                                        */
  real32_T rpm_max;                    /* Variable: rpm_max
                                        * Referenced by: '<S3>/Saturation'
                                        */
  real32_T speed_rpm_vec[20];          /* Variable: speed_rpm_vec
                                        * Referenced by: '<S3>/Prelookup RPM'
                                        */
  real32_T UnitDelay_InitialCondition;
                               /* Computed Parameter: UnitDelay_InitialCondition
                                * Referenced by: '<S2>/Unit Delay'
                                */
  real32_T PrelookupTe_BreakpointsData[12];/* Expression: torque_vec'
                                            * Referenced by: '<S3>/Prelookup Te'
                                            */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<S3>/Saturation'
                                       */
  real32_T Switch_Threshold;           /* Computed Parameter: Switch_Threshold
                                        * Referenced by: '<S3>/Switch'
                                        */
  real32_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S4>/Discrete-Time Integrator'
                            */
  real32_T DiscreteTimeIntegrator_IC[2];
                                /* Computed Parameter: DiscreteTimeIntegrator_IC
                                 * Referenced by: '<S4>/Discrete-Time Integrator'
                                 */
  real32_T Gain1_Gain;                 /* Computed Parameter: Gain1_Gain
                                        * Referenced by: '<S1>/Gain1'
                                        */
};

/* Block parameters (default storage) */
extern P_MPC_T MPC_P;

/* Block signals (default storage) */
extern B_MPC_T MPC_B;

/* Block states (default storage) */
extern DW_MPC_T MPC_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_MPC_T MPC_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_MPC_T MPC_Y;

/* Model entry point functions */
extern void MPC_initialize(void);
extern void MPC_step(void);
extern void MPC_terminate(void);
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Data Type Conversion' : Eliminate redundant data type conversion
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
 * '<Root>' : 'MPC'
 * '<S1>'   : 'MPC/MPC'
 * '<S2>'   : 'MPC/MPC/Current controller'
 * '<S3>'   : 'MPC/MPC/Current reference generation'
 * '<S4>'   : 'MPC/MPC/Subsystem'
 * '<S5>'   : 'MPC/MPC/Te calculation'
 * '<S6>'   : 'MPC/MPC/Current controller/MPC'
 * '<S7>'   : 'MPC/MPC/Subsystem/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_MPC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
