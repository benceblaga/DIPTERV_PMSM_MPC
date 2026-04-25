/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MPC_private.h
 *
 * Code generated for Simulink model 'MPC'.
 *
 * Model version                  : 1.134
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Sat Apr 18 05:01:08 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MPC_private_h_
#define RTW_HEADER_MPC_private_h_
#include "rtwtypes.h"
#include "MPC_types.h"
#include "MPC.h"

extern real32_T rt_roundf_snf(real32_T u);
extern real32_T rt_hypotf_snf(real32_T u0, real32_T u1);
extern uint32_T plook_u32ff_binx(real32_T u, const real32_T bp[], uint32_T
  maxIndex, real32_T *fraction);
extern real32_T intrp2d_fu32fl_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride);
extern uint32_T binsearch_u32f(real32_T u, const real32_T bp[], uint32_T
  startIndex, uint32_T maxIndex);
extern int16_T div_nde_s16_floor(int16_T numerator, int16_T denominator);

#endif                                 /* RTW_HEADER_MPC_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
