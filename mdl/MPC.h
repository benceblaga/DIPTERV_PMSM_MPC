/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MPC.h
 *
 * Code generated for Simulink model 'MPC'.
 *
 * Model version                  : 1.151
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Tue May  5 19:08:27 2026
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

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE[2];/* '<S4>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE[2];        /* '<S2>/Unit Delay' */
  uint32_T PrelookupTe_DWORK1;         /* '<S3>/Prelookup Te' */
  uint32_T PrelookupRPM_DWORK1;        /* '<S3>/Prelookup RPM' */
  boolean_T iA_prev[4];                /* '<S2>/MPC' */
} DW_MPC_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: torque_vec'
   * Referenced by: '<S3>/Prelookup Te'
   */
  real32_T PrelookupTe_BreakpointsData[13];

  /* Expression: speed_rpm_vec
   * Referenced by: '<S3>/Prelookup RPM'
   */
  real32_T PrelookupRPM_BreakpointsData[21];

  /* Expression: LUT_Iq
   * Referenced by: '<S3>/Interpolation Using Prelookup1'
   */
  real32_T InterpolationUsingPrelookup1_Ta[273];

  /* Expression: LUT_Id
   * Referenced by: '<S3>/Interpolation Using Prelookup'
   */
  real32_T InterpolationUsingPrelookup_Tab[273];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S3>/Interpolation Using Prelookup'
   *   '<S3>/Interpolation Using Prelookup1'
   */
  uint32_T pooled3[2];
} ConstP_MPC_T;

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
  real32_T Te_meas_out;                /* '<Root>/Te_meas_out' */
  real32_T id_ref_cal_out;             /* '<Root>/id_ref_cal_out' */
  real32_T iq_ref_cal_out;             /* '<Root>/iq_ref_cal_out' */
  real32_T Tl_est_out;                 /* '<Root>/Tl_est_out' */
} ExtY_MPC_T;

/* Block states (default storage) */
extern DW_MPC_T MPC_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_MPC_T MPC_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_MPC_T MPC_Y;

/* Constant parameters (default storage) */
extern const ConstP_MPC_T MPC_ConstP;

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
