/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MPC.c
 *
 * Code generated for Simulink model 'MPC'.
 *
 * Model version                  : 1.156
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Wed May  6 14:42:29 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "MPC.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "mw_C28x_addsub_s32.h"
#include "MPC_private.h"

/* Block states (default storage) */
DW_MPC_T MPC_DW;

/* External inputs (root inport signals with default storage) */
ExtU_MPC_T MPC_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_MPC_T MPC_Y;

/* Forward declaration for local functions */
static void MPC_trisolve(const real32_T A[4], real32_T B[4]);
static void MPC_Unconstrained(const real32_T Hinv[4], const real32_T f[2],
  real32_T x[2]);
static real32_T MPC_norm(const real32_T x[2]);
static void MPC_abs(const real32_T x[2], real32_T y[2]);
static real32_T MPC_maximum(const real32_T x[2]);
static void MPC_abs_i(real32_T y[4]);
static void MPC_maximum2(const real32_T x[4], real32_T ex[4]);
static void MPC_xgemv(int16_T m, int16_T n, const real32_T A[4], int16_T ia0,
                      const real32_T x[4], int16_T ix0, real32_T y[2]);
static void MPC_xgerc(int16_T m, int16_T n, real32_T alpha1, int16_T ix0, const
                      real32_T y[2], real32_T A[4], int16_T ia0);
static real32_T MPC_KWIKfactor(const real32_T Ac[8], const int32_T iC[4],
  int32_T nA, const real32_T Linv[4], real32_T RLinv[4], real32_T D[4], real32_T
  H[4]);
static real32_T MPC_mtimes(const real32_T A[2], const real32_T B[2]);
static void MPC_DropConstraint(int32_T kDrop, boolean_T iA[4], int32_T *nA,
  int32_T iC[4]);
static void MPC_qpkwik(const real32_T Linv[4], const real32_T Hinv[4], const
  real32_T f[2], const real32_T Ac[8], boolean_T iA[4], real32_T x[2], real32_T
  lambda[4], int32_T *status);
uint32_T plook_u32ff_binxp(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0UL]) {
    bpIndex = 0UL;
    *fraction = (u - bp[0UL]) / (bp[1UL] - bp[0UL]);
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32f_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1UL] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1UL;
    *fraction = (u - bp[maxIndex - 1UL]) / (bp[maxIndex] - bp[maxIndex - 1UL]);
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real32_T intrp2d_fu32flm_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride[])
{
  real32_T yL_0d0;
  real32_T yL_0d1;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1UL] * stride[1UL] + bpIndex[0UL] * stride[0UL];
  yL_0d0 = table[offset_1d];
  yL_0d0 = (table[offset_1d + stride[0UL]] - yL_0d0) * frac[0UL] + yL_0d0;
  offset_1d = offset_1d + stride[1UL];
  yL_0d1 = table[offset_1d];
  return (((table[offset_1d + stride[0UL]] - yL_0d1) * frac[0UL] + yL_0d1) -
          yL_0d0) * frac[1UL] + yL_0d0;
}

uint32_T binsearch_u32f_prevIdx(real32_T u, const real32_T bp[], uint32_T
  startIndex, uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0UL;
  iRght = maxIndex;
  found = 0UL;
  while (found == 0UL) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1UL;
      bpIndex = ((bpIndex + iLeft) - 1UL) >> 1UL;
    } else if (u < bp[bpIndex + 1UL]) {
      found = 1UL;
    } else {
      iLeft = bpIndex + 1UL;
      bpIndex = ((bpIndex + iRght) + 1UL) >> 1UL;
    }
  }

  return bpIndex;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_trisolve(const real32_T A[4], real32_T B[4])
{
  real32_T s;
  int16_T colB;
  int16_T i;
  int16_T j;
  int16_T s_tmp;
  int16_T s_tmp_0;
  for (i = 0; i < 2; i++) {
    for (colB = 0; colB < 2; colB++) {
      s_tmp = i << 1U;
      s_tmp_0 = s_tmp + colB;
      s = B[s_tmp_0];
      for (j = 0; j < i; j++) {
        s -= A[s_tmp] * B[colB];
      }

      B[s_tmp_0] = s / A[s_tmp + i];
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_Unconstrained(const real32_T Hinv[4], const real32_T f[2],
  real32_T x[2])
{
  x[0] = f[0] * -Hinv[0] + f[1] * -Hinv[1];
  x[1] = f[0] * -Hinv[2] + f[1] * -Hinv[3];
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_norm(const real32_T x[2])
{
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * (real32_T)sqrt(y);
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_abs(const real32_T x[2], real32_T y[2])
{
  y[0] = fabsf(x[0]);
  y[1] = fabsf(x[1]);
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_maximum(const real32_T x[2])
{
  real32_T ex;
  if (x[0] < x[1]) {
    ex = x[1];
  } else if (rtIsNaNF(x[0])) {
    if (!rtIsNaNF(x[1])) {
      ex = x[1];
    } else {
      ex = (rtNaNF);
    }
  } else {
    ex = x[0];
  }

  return ex;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_abs_i(real32_T y[4])
{
  y[0] = 24.0F;
  y[1] = 24.0F;
  y[2] = 24.0F;
  y[3] = 24.0F;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_maximum2(const real32_T x[4], real32_T ex[4])
{
  if (x[0] >= 1.0F) {
    ex[0] = x[0];
  } else {
    ex[0] = 1.0F;
  }

  if (x[1] >= 1.0F) {
    ex[1] = x[1];
  } else {
    ex[1] = 1.0F;
  }

  if (x[2] >= 1.0F) {
    ex[2] = x[2];
  } else {
    ex[2] = 1.0F;
  }

  if (x[3] >= 1.0F) {
    ex[3] = x[3];
  } else {
    ex[3] = 1.0F;
  }
}

real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T a;
  real32_T b;
  real32_T y;
  a = fabsf(u0);
  b = fabsf(u1);
  if (a < b) {
    a /= b;
    y = (real32_T)sqrt(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = (real32_T)sqrt(b * b + 1.0F) * a;
  } else if (rtIsNaNF(b)) {
    y = (rtNaNF);
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_xgemv(int16_T m, int16_T n, const real32_T A[4], int16_T ia0,
                      const real32_T x[4], int16_T ix0, real32_T y[2])
{
  real32_T c;
  int16_T b;
  int16_T ia;
  int16_T iac;
  if (n != 0) {
    y[0] = 0.0F;
    for (iac = ia0; iac <= ia0; iac += 2) {
      c = 0.0F;
      b = (iac + m) - 1;
      for (ia = iac; ia <= b; ia++) {
        c += x[((ix0 + ia) - iac) - 1] * A[ia - 1];
      }

      ia = (iac - ia0) >> 1U;
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_xgerc(int16_T m, int16_T n, real32_T alpha1, int16_T ix0, const
                      real32_T y[2], real32_T A[4], int16_T ia0)
{
  real32_T temp;
  int16_T b;
  int16_T ijA;
  int16_T j;
  int16_T jA;
  if (!(alpha1 == 0.0F)) {
    jA = ia0;
    for (j = 0; j < n; j++) {
      temp = y[j];
      if (temp != 0.0F) {
        temp *= alpha1;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 2;
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_KWIKfactor(const real32_T Ac[8], const int32_T iC[4],
  int32_T nA, const real32_T Linv[4], real32_T RLinv[4], real32_T D[4], real32_T
  H[4])
{
  int32_T d_k;
  int32_T exitg1;
  int32_T i;
  real32_T A[4];
  real32_T Q[4];
  real32_T TL[4];
  real32_T tau[2];
  real32_T work[2];
  real32_T A_0;
  real32_T A_1;
  real32_T A_2;
  real32_T Linv_0;
  real32_T Linv_1;
  real32_T Status;
  real32_T beta1;
  int16_T b_lastv;
  int16_T c_lastc;
  int16_T i_i;
  int16_T ii;
  int16_T knt;
  Status = 1.0F;
  RLinv[0] = 0.0F;
  RLinv[1] = 0.0F;
  RLinv[2] = 0.0F;
  RLinv[3] = 0.0F;
  for (i = 1L; i <= nA; i++) {
    RLinv[(int16_T)i - 1] = 0.0F;
    i_i = ((int16_T)iC[(int16_T)i - 1] - 1) << 1U;
    A_0 = Ac[i_i];
    RLinv[(int16_T)i - 1] += A_0 * Linv[0];
    beta1 = Ac[i_i + 1];
    RLinv[(int16_T)i - 1] += beta1 * Linv[1];
    RLinv[(int16_T)i + 1] = 0.0F;
    RLinv[(int16_T)i + 1] += A_0 * Linv[2];
    RLinv[(int16_T)i + 1] += beta1 * Linv[3];
  }

  A[0] = RLinv[0];
  A[1] = RLinv[2];
  tau[0] = 0.0F;
  work[0] = 0.0F;
  A[2] = RLinv[1];
  A[3] = RLinv[3];
  tau[1] = 0.0F;
  work[1] = 0.0F;
  for (i_i = 0; i_i < 2; i_i++) {
    ii = (i_i << 1U) + i_i;
    if (i_i + 1 < 2) {
      A_0 = A[ii];
      c_lastc = ii + 2;
      tau[0] = 0.0F;
      beta1 = fabsf(A[ii + 1]);
      if (beta1 != 0.0F) {
        beta1 = rt_hypotf_snf(A_0, beta1);
        if (A_0 >= 0.0F) {
          beta1 = -beta1;
        }

        if (fabsf(beta1) < 9.86076132E-32F) {
          knt = 0;
          do {
            knt++;
            for (b_lastv = c_lastc; b_lastv <= ii + 2; b_lastv++) {
              A[b_lastv - 1] *= 1.01412048E+31F;
            }

            beta1 *= 1.01412048E+31F;
            A_0 *= 1.01412048E+31F;
          } while ((fabsf(beta1) < 9.86076132E-32F) && (knt < 20));

          beta1 = rt_hypotf_snf(A_0, fabsf(A[ii + 1]));
          if (A_0 >= 0.0F) {
            beta1 = -beta1;
          }

          tau[0] = (beta1 - A_0) / beta1;
          A_0 = 1.0F / (A_0 - beta1);
          for (b_lastv = c_lastc; b_lastv <= ii + 2; b_lastv++) {
            A[b_lastv - 1] *= A_0;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 9.86076132E-32F;
          }

          A_0 = beta1;
        } else {
          tau[0] = (beta1 - A_0) / beta1;
          A_0 = 1.0F / (A_0 - beta1);
          for (b_lastv = c_lastc; b_lastv <= ii + 2; b_lastv++) {
            A[b_lastv - 1] *= A_0;
          }

          A_0 = beta1;
        }
      }

      A[ii] = 1.0F;
      if (tau[0] != 0.0F) {
        b_lastv = 2;
        c_lastc = ii + 1;
        while ((b_lastv > 0) && (A[c_lastc] == 0.0F)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 1;
        knt = ii + 2;
        do {
          exitg1 = 0L;
          if (knt + 1 <= (ii + b_lastv) + 2) {
            if (A[knt] != 0.0F) {
              exitg1 = 1L;
            } else {
              knt++;
            }
          } else {
            c_lastc = 0;
            exitg1 = 1L;
          }
        } while (exitg1 == 0L);
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        MPC_xgemv(b_lastv, c_lastc, A, ii + 3, A, ii + 1, work);
        MPC_xgerc(b_lastv, c_lastc, -tau[0], ii + 1, work, A, ii + 3);
      }

      A[ii] = A_0;
    } else {
      tau[1] = 0.0F;
    }
  }

  for (i_i = 0; i_i < 2; i_i++) {
    for (ii = 0; ii <= i_i; ii++) {
      b_lastv = i_i << 1U;
      TL[ii + b_lastv] = A[b_lastv + ii];
    }

    if (i_i + 2 <= 2) {
      TL[(i_i << 1U) + 1] = 0.0F;
    }

    work[i_i] = 0.0F;
  }

  for (i_i = 1; i_i >= 0; i_i--) {
    ii = ((i_i << 1U) + i_i) + 2;
    if (i_i + 1 < 2) {
      A[ii - 2] = 1.0F;
      if (tau[i_i] != 0.0F) {
        b_lastv = 2;
        c_lastc = ii;
        while ((b_lastv > 0) && (A[c_lastc - 1] == 0.0F)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 1;
        knt = ii;
        do {
          exitg1 = 0L;
          if (knt + 1 <= ii + b_lastv) {
            if (A[knt] != 0.0F) {
              exitg1 = 1L;
            } else {
              knt++;
            }
          } else {
            c_lastc = 0;
            exitg1 = 1L;
          }
        } while (exitg1 == 0L);
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        MPC_xgemv(b_lastv, c_lastc, A, ii + 1, A, ii - 1, work);
        MPC_xgerc(b_lastv, c_lastc, -tau[i_i], ii - 1, work, A, ii + 1);
      }

      for (b_lastv = ii; b_lastv <= ii; b_lastv++) {
        A[b_lastv - 1] *= -tau[i_i];
      }
    }

    A[ii - 2] = 1.0F - tau[i_i];
    if (i_i - 1 >= 0) {
      A[ii - 3] = 0.0F;
    }
  }

  Q[0] = A[0];
  Q[1] = A[1];
  Q[2] = A[2];
  Q[3] = A[3];
  A[0] = Q[0];
  A[1] = Q[2];
  A[2] = Q[1];
  A[3] = Q[3];
  Q[0] = TL[0];
  Q[1] = TL[2];
  Q[2] = TL[1];
  Q[3] = TL[3];
  i = 1L;
  do {
    exitg1 = 0L;
    if (i <= nA) {
      if (fabsf(Q[((((int16_T)i - 1) << 1U) + (int16_T)i) - 1]) < 1.0E-12F) {
        Status = -2.0F;
        exitg1 = 1L;
      } else {
        i++;
      }
    } else {
      A_0 = A[2];
      beta1 = A[0];
      A_1 = A[3];
      A_2 = A[1];
      for (i_i = 0; i_i < 2; i_i++) {
        Linv_0 = Linv[i_i + 2];
        Linv_1 = Linv[i_i];
        b_lastv = i_i << 1U;
        TL[b_lastv] = Linv_0 * A_0 + beta1 * Linv_1;
        TL[b_lastv + 1] = Linv_0 * A_1 + A_2 * Linv_1;
      }

      RLinv[0] = 0.0F;
      RLinv[1] = 0.0F;
      RLinv[2] = 0.0F;
      RLinv[3] = 0.0F;
      for (i = nA; i > 0L; i--) {
        i_i = ((int16_T)i - 1) << 1U;
        ii = ((int16_T)i + i_i) - 1;
        RLinv[ii] = 1.0F;
        for (d_k = i; d_k <= nA; d_k++) {
          b_lastv = (i_i + (int16_T)d_k) - 1;
          RLinv[b_lastv] /= Q[ii];
        }

        if (i > 1L) {
          for (d_k = 2L; d_k <= nA; d_k++) {
            RLinv[1] -= Q[1] * RLinv[3];
          }
        }
      }

      for (i_i = 0; i_i + 1 < 3; i_i++) {
        H[i_i] = 0.0F;
        for (i = nA + 1L; i < 3L; i++) {
          H[i_i] -= TL[((i_i << 1U) + (int16_T)i) - 1] * TL[(int16_T)i - 1];
        }

        H[i_i << 1U] = H[i_i];
      }

      for (i_i = 1; i_i + 1 < 3; i_i++) {
        H[i_i + 2] = 0.0F;
        for (i = nA + 1L; i < 3L; i++) {
          H[i_i + 2] -= TL[((i_i << 1U) + (int16_T)i) - 1] * TL[(int16_T)i + 1];
        }

        H[(i_i << 1U) + 1] = H[i_i + 2];
      }

      for (i = 1L; i <= nA; i++) {
        D[(int16_T)i - 1] = 0.0F;
        for (d_k = i; (uint32_T)d_k <= (uint32_T)nA; d_k++) {
          D[(int16_T)i - 1] += RLinv[((((int16_T)i - 1) << 1U) + (int16_T)d_k) -
            1] * TL[(int16_T)d_k - 1];
        }

        D[(int16_T)i + 1] = 0.0F;
        for (d_k = i; (uint32_T)d_k <= (uint32_T)nA; d_k++) {
          D[(int16_T)i + 1] += RLinv[((((int16_T)i - 1) << 1U) + (int16_T)d_k) -
            1] * TL[(int16_T)d_k + 1];
        }
      }

      exitg1 = 1L;
    }
  } while (exitg1 == 0L);

  return Status;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_mtimes(const real32_T A[2], const real32_T B[2])
{
  return A[0] * B[0] + A[1] * B[1];
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_DropConstraint(int32_T kDrop, boolean_T iA[4], int32_T *nA,
  int32_T iC[4])
{
  int32_T b;
  int32_T i;
  if (kDrop > 0L) {
    iA[(int16_T)iC[(int16_T)kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      b = c28x_sub_s32_s32_s32_sat(*nA, 1L);
      for (i = kDrop; i <= b; i++) {
        iC[(int16_T)i - 1] = iC[(int16_T)(i + 1L) - 1];
      }
    }

    iC[(int16_T)*nA - 1] = 0L;
    *nA = c28x_sub_s32_s32_s32_sat(*nA, 1L);
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_qpkwik(const real32_T Linv[4], const real32_T Hinv[4], const
  real32_T f[2], const real32_T Ac[8], boolean_T iA[4], real32_T x[2], real32_T
  lambda[4], int32_T *status)
{
  int32_T iC[4];
  int32_T b_i;
  int32_T e_i;
  int32_T exitg1;
  int32_T exitg3;
  int32_T g_i;
  int32_T j_i;
  int32_T kDrop;
  int32_T nA;
  real32_T D[4];
  real32_T H[4];
  real32_T Opt[4];
  real32_T RLinv[4];
  real32_T Rhs[4];
  real32_T U[4];
  real32_T cTol[4];
  real32_T Ac_0[2];
  real32_T r[2];
  real32_T z[2];
  real32_T Xnorm0;
  real32_T cMin;
  real32_T cVal;
  real32_T rMin;
  real32_T t;
  real32_T zTa;
  int16_T Ac_tmp;
  int16_T f_i;
  int16_T kNext;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  x[0] = 0.0F;
  x[1] = 0.0F;
  lambda[0] = 0.0F;
  lambda[1] = 0.0F;
  lambda[2] = 0.0F;
  lambda[3] = 0.0F;
  *status = 1L;
  r[0] = 0.0F;
  r[1] = 0.0F;
  rMin = 0.0F;
  cTolComputed = false;
  cTol[0] = 1.0F;
  iC[0] = 0L;
  cTol[1] = 1.0F;
  iC[1] = 0L;
  cTol[2] = 1.0F;
  iC[2] = 0L;
  cTol[3] = 1.0F;
  iC[3] = 0L;
  nA = 0L;
  if (iA[0]) {
    nA = 1L;
    iC[0] = 1L;
  }

  if (iA[1]) {
    nA = (int16_T)nA + 1;
    iC[(int16_T)nA - 1] = 2L;
  }

  if (iA[2]) {
    nA = (int16_T)nA + 1;
    iC[(int16_T)nA - 1] = 3L;
  }

  if (iA[3]) {
    nA = (int16_T)nA + 1;
    iC[(int16_T)nA - 1] = 4L;
  }

  guard1 = false;
  if ((int16_T)nA > 0) {
    Opt[2] = 0.0F;
    Opt[3] = 0.0F;
    Rhs[0] = f[0];
    Rhs[2] = 0.0F;
    Rhs[1] = f[1];
    Rhs[3] = 0.0F;
    DualFeasible = false;
    ColdReset = false;
    do {
      exitg3 = 0L;
      if ((!DualFeasible) && (nA > 0L) && ((int16_T)*status <= 200)) {
        Xnorm0 = MPC_KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
        if (Xnorm0 < 0.0F) {
          if (ColdReset) {
            *status = -2L;
            exitg3 = 2L;
          } else {
            iA[0] = false;
            iC[0] = 0L;
            iA[1] = false;
            iC[1] = 0L;
            iA[2] = false;
            iC[2] = 0L;
            iA[3] = false;
            iC[3] = 0L;
            nA = 0L;
            ColdReset = true;
          }
        } else {
          for (kDrop = 1L; kDrop <= nA; kDrop++) {
            if (kDrop > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 2L;
            }

            Rhs[(int16_T)b_i - 1] = -24.0F;
            for (b_i = kDrop; b_i <= nA; b_i++) {
              kNext = ((int16_T)b_i - 1) << 1U;
              f_i = ((int16_T)kDrop + kNext) - 1;
              U[f_i] = 0.0F;
              for (e_i = 1L; e_i <= nA; e_i++) {
                U[f_i] += RLinv[((((int16_T)kDrop - 1) << 1U) + (int16_T)e_i) -
                  1] * RLinv[(kNext + (int16_T)e_i) - 1];
              }

              U[((int16_T)b_i + (((int16_T)kDrop - 1) << 1U)) - 1] = U[f_i];
            }
          }

          Opt[0] = Rhs[0] * H[0] + Rhs[1] * H[1];
          for (kDrop = 1L; (uint32_T)kDrop <= (uint32_T)nA; kDrop++) {
            if (kDrop > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 2L;
            }

            Opt[0] += D[(int16_T)kDrop - 1] * Rhs[(int16_T)b_i - 1];
          }

          Opt[1] = Rhs[0] * H[2] + Rhs[1] * H[3];
          for (kDrop = 1L; (uint32_T)kDrop <= (uint32_T)nA; kDrop++) {
            if (kDrop > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 2L;
            }

            Opt[1] += D[(int16_T)kDrop + 1] * Rhs[(int16_T)b_i - 1];
          }

          for (kDrop = 1L; (uint32_T)kDrop <= (uint32_T)nA; kDrop++) {
            if (kDrop > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 2L;
            }

            Opt[(int16_T)b_i - 1] = D[(int16_T)kDrop - 1] * Rhs[0] + D[(int16_T)
              kDrop + 1] * Rhs[1];
            if ((uint32_T)nA >= 1UL) {
              if (kDrop > 2147483645L) {
                g_i = MAX_int32_T;
                j_i = MAX_int32_T;
              } else {
                g_i = kDrop + 2L;
                j_i = kDrop + 2L;
              }
            }

            for (e_i = 1L; (uint32_T)e_i <= (uint32_T)nA; e_i++) {
              if (e_i > 2147483645L) {
                b_i = MAX_int32_T;
              } else {
                b_i = e_i + 2L;
              }

              Opt[(int16_T)g_i - 1] = U[((((int16_T)kDrop - 1) << 1U) + (int16_T)
                e_i) - 1] * Rhs[(int16_T)b_i - 1] + Opt[(int16_T)j_i - 1];
            }
          }

          Xnorm0 = -1.0E-12F;
          kDrop = 0L;
          for (e_i = 1L; (uint32_T)e_i <= (uint32_T)nA; e_i++) {
            if (e_i > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = e_i + 2L;
            }

            lambda[(int16_T)iC[(int16_T)e_i - 1] - 1] = Opt[(int16_T)b_i - 1];
            if (e_i > 2147483645L) {
              b_i = MAX_int32_T;
            } else {
              b_i = e_i + 2L;
            }

            if ((Opt[(int16_T)b_i - 1] < Xnorm0) && (e_i <= nA)) {
              kDrop = e_i;
              if (e_i > 2147483645L) {
                b_i = MAX_int32_T;
              } else {
                b_i = e_i + 2L;
              }

              Xnorm0 = Opt[(int16_T)b_i - 1];
            }
          }

          if (kDrop <= 0L) {
            DualFeasible = true;
            x[0] = Opt[0];
            x[1] = Opt[1];
          } else {
            *status = (int16_T)*status + 1;
            if ((int16_T)*status > 5) {
              iA[0] = false;
              iC[0] = 0L;
              iA[1] = false;
              iC[1] = 0L;
              iA[2] = false;
              iC[2] = 0L;
              iA[3] = false;
              iC[3] = 0L;
              nA = 0L;
              ColdReset = true;
            } else {
              lambda[(int16_T)iC[(int16_T)kDrop - 1] - 1] = 0.0F;
              MPC_DropConstraint(kDrop, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0L) {
          lambda[0] = 0.0F;
          lambda[1] = 0.0F;
          lambda[2] = 0.0F;
          lambda[3] = 0.0F;
          MPC_Unconstrained(Hinv, f, x);
        }

        exitg3 = 1L;
      }
    } while (exitg3 == 0L);

    if (exitg3 == 1L) {
      guard1 = true;
    }
  } else {
    MPC_Unconstrained(Hinv, f, x);
    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = MPC_norm(x);
    exitg2 = false;
    while ((!exitg2) && ((int16_T)*status <= 200)) {
      cMin = -1.0E-6F;
      kNext = -1;
      for (f_i = 0; f_i < 4; f_i++) {
        if (!cTolComputed) {
          Ac_tmp = f_i << 1U;
          Ac_0[0] = Ac[Ac_tmp] * x[0];
          Ac_0[1] = Ac[Ac_tmp + 1] * x[1];
          cVal = cTol[f_i];
          MPC_abs(Ac_0, z);
          t = MPC_maximum(z);
          if ((cVal >= t) || rtIsNaNF(t)) {
            cTol[f_i] = cVal;
          } else {
            cTol[f_i] = t;
          }
        }

        if (!iA[f_i]) {
          Ac_tmp = f_i << 1U;
          cVal = ((Ac[Ac_tmp + 1] * x[1] + Ac[Ac_tmp] * x[0]) - -24.0F) /
            cTol[f_i];
          if (cVal < cMin) {
            cMin = cVal;
            kNext = f_i;
          }
        }
      }

      cTolComputed = true;
      if (kNext + 1 <= 0) {
        exitg2 = true;
      } else if ((int16_T)*status == 200) {
        *status = 0L;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0L;
          if ((kNext + 1 > 0) && ((int16_T)*status <= 200)) {
            guard2 = false;
            if (nA == 0L) {
              f_i = kNext << 1U;
              cMin = Ac[f_i + 1];
              cVal = Ac[f_i];
              z[0] = cMin * Hinv[1] + cVal * Hinv[0];
              z[1] = cMin * Hinv[3] + cVal * Hinv[2];
              guard2 = true;
            } else {
              cMin = MPC_KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
              if (cMin <= 0.0F) {
                *status = -2L;
                exitg1 = 1L;
              } else {
                f_i = kNext << 1U;
                cMin = Ac[f_i + 1];
                cVal = Ac[f_i];
                z[0] = cMin * -H[1] + cVal * -H[0];
                z[1] = cMin * -H[3] + cVal * -H[2];
                for (g_i = 1L; g_i <= nA; g_i++) {
                  r[(int16_T)g_i - 1] = D[(int16_T)g_i + 1] * cMin + D[(int16_T)
                    g_i - 1] * cVal;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              kDrop = 0L;
              cMin = 0.0F;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0L) {
                g_i = 1L;
                exitg4 = false;
                while ((!exitg4) && ((uint32_T)g_i <= (uint32_T)nA)) {
                  if (r[(int16_T)g_i - 1] >= 1.0E-12F) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    g_i++;
                  }
                }
              }

              if ((nA != 0L) && (!ColdReset)) {
                for (g_i = 1L; g_i <= nA; g_i++) {
                  cVal = r[(int16_T)g_i - 1];
                  if (cVal > 1.0E-12F) {
                    cVal = lambda[(int16_T)iC[(int16_T)g_i - 1] - 1] / cVal;
                    if ((kDrop == 0L) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = g_i;
                    }
                  }
                }

                if (kDrop > 0L) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              Ac_tmp = kNext << 1U;
              cVal = Ac[Ac_tmp];
              Ac_0[0] = cVal;
              t = Ac[Ac_tmp + 1];
              Ac_0[1] = t;
              zTa = MPC_mtimes(z, Ac_0);
              if (zTa <= 0.0F) {
                cVal = 0.0F;
                ColdReset = true;
              } else {
                cVal = (-24.0F - (t * x[1] + cVal * x[0])) / zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1L;
                exitg1 = 1L;
              } else {
                if (ColdReset) {
                  t = cMin;
                } else if (DualFeasible) {
                  t = cVal;
                } else if (cMin < cVal) {
                  t = cMin;
                } else {
                  t = cVal;
                }

                for (g_i = 1L; g_i <= nA; g_i++) {
                  f_i = (int16_T)iC[(int16_T)g_i - 1] - 1;
                  lambda[f_i] -= r[(int16_T)g_i - 1] * t;
                  if (lambda[f_i] < 0.0F) {
                    lambda[f_i] = 0.0F;
                  }
                }

                lambda[kNext] += t;
                if (fabsf(t - cMin) < 1.1920929E-7F) {
                  MPC_DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!ColdReset) {
                  x[0] += t * z[0];
                  x[1] += t * z[1];
                  if (fabsf(t - cVal) < 1.1920929E-7F) {
                    if (nA == 2L) {
                      *status = -1L;
                      exitg1 = 1L;
                    } else {
                      nA = c28x_add_s32_s32_s32_sat(nA, 1L);
                      iC[(int16_T)nA - 1] = kNext + 1;
                      j_i = nA;
                      exitg4 = false;
                      while ((!exitg4) && (j_i > 1L)) {
                        b_i = iC[(int16_T)j_i - 1];
                        g_i = iC[(int16_T)(j_i - 1L) - 1];
                        if (b_i > g_i) {
                          exitg4 = true;
                        } else {
                          iC[(int16_T)j_i - 1] = g_i;
                          iC[(int16_T)(j_i - 1L) - 1] = b_i;
                          j_i--;
                        }
                      }

                      iA[kNext] = true;
                      kNext = -1;
                      *status = (int16_T)*status + 1;
                    }
                  } else {
                    *status = (int16_T)*status + 1;
                  }
                } else {
                  *status = (int16_T)*status + 1;
                }
              }
            }
          } else {
            cMin = MPC_norm(x);
            if (fabsf(cMin - Xnorm0) > 0.001F) {
              Xnorm0 = cMin;
              MPC_abs_i(RLinv);
              MPC_maximum2(RLinv, cTol);
              cTolComputed = false;
            }

            exitg1 = 2L;
          }
        } while (exitg1 == 0L);

        if (exitg1 == 1L) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Model step function */
void MPC_step(void)
{
  int32_T exitflag;
  real32_T H_qp[4];
  real32_T Linv[4];
  real32_T Su[4];
  real32_T T[4];
  real32_T a[4];
  real32_T g[4];
  real32_T IdIq_ref[2];
  real32_T frac[2];
  real32_T frac_0[2];
  real32_T rtb_Gain_0[2];
  real32_T tmp[2];
  real32_T Su_0;
  real32_T Su_1;
  real32_T e_wm;
  real32_T g_0;
  real32_T rtb_Saturation;
  real32_T u0;
  uint32_T bpIndex[2];
  uint32_T bpIndex_0[2];
  int16_T b;
  int16_T c_k;
  int16_T i;
  int16_T idxAjj;
  static const int16_T b_0[2] = { 0, 1 };

  static const real32_T g_1[4] = { 0.1F, 0.0F, 0.0F, 0.2F };

  static const int16_T e[4] = { 1, 0, 0, 1 };

  static const real32_T h[4] = { 0.01F, 0.0F, 0.0F, 0.01F };

  static const real32_T l[8] = { -1.0F, -0.0F, -0.0F, -1.0F, 1.0F, 0.0F, 0.0F,
    1.0F };

  boolean_T exitg1;

  /* Outputs for Atomic SubSystem: '<Root>/MPC' */
  /* MATLAB Function: '<S1>/Te calculation' incorporates:
   *  Inport: '<Root>/Id_meas'
   *  Inport: '<Root>/Iq_meas'
   */
  MPC_Y.Te_meas_out = (-0.00135187327F * MPC_U.Id_meas + 1.0F) * (0.194175F *
    MPC_U.Iq_meas);

  /* Outputs for Atomic SubSystem: '<S1>/Subsystem' */
  /* Sum: '<S1>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
   */
  MPC_Y.Tl_est_out = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
   *  Inport: '<Root>/wm'
   */
  e_wm = MPC_U.wm - MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
  rtb_Saturation = MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
   */
  u0 = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];
  g_0 = MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' incorporates:
   *  MATLAB Function: '<S4>/MATLAB Function'
   */
  Su_0 = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];
  MPC_DW.DiscreteTimeIntegrator_DSTATE[0] = (((MPC_Y.Te_meas_out - u0) - 0.005F *
    g_0) * 43859.6484F + 1780.70178F * e_wm) * 0.0001F + rtb_Saturation;
  MPC_DW.DiscreteTimeIntegrator_DSTATE[1] = -22.8000011F * e_wm * 0.0001F + Su_0;

  /* End of Outputs for SubSystem: '<S1>/Subsystem' */

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Te_ref'
   */
  rtb_Saturation = MPC_U.Te_ref + MPC_Y.Tl_est_out;

  /* Outputs for Atomic SubSystem: '<S1>/Current reference generation' */
  /* Saturate: '<S3>/Saturation1' */
  if (rtb_Saturation > 1.25F) {
    rtb_Saturation = 1.25F;
  } else if (rtb_Saturation < -1.25F) {
    rtb_Saturation = -1.25F;
  }

  /* End of Saturate: '<S3>/Saturation1' */

  /* PreLookup: '<S3>/Prelookup Te' incorporates:
   *  Abs: '<S3>/Abs'
   */
  MPC_Y.Te_indx_out = plook_u32ff_binxp(fabsf(rtb_Saturation),
    MPC_ConstP.PrelookupTe_BreakpointsData, 12UL, &MPC_Y.Te_frac_out,
    &MPC_DW.PrelookupTe_DWORK1);

  /* Abs: '<S3>/Abs1' incorporates:
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/wm'
   */
  u0 = fabsf(9.54929638F * MPC_U.wm);

  /* Saturate: '<S3>/Saturation' */
  if (u0 > 2674.0F) {
    u0 = 2674.0F;
  }

  /* PreLookup: '<S3>/Prelookup RPM' incorporates:
   *  Saturate: '<S3>/Saturation'
   */
  MPC_Y.Rpm_indx_out = plook_u32ff_binxp(u0,
    MPC_ConstP.PrelookupRPM_BreakpointsData, 20UL, &MPC_Y.Rpm_frac_out,
    &MPC_DW.PrelookupRPM_DWORK1);

  /* Interpolation_n-D: '<S3>/Interpolation Using Prelookup1' */
  frac[0L] = MPC_Y.Te_frac_out;
  if (MPC_Y.Te_frac_out < 0.0F) {
    frac[0L] = 0.0F;
  } else if (MPC_Y.Te_frac_out > 1.0F) {
    frac[0L] = 1.0F;
  }

  frac[1L] = MPC_Y.Rpm_frac_out;
  if (MPC_Y.Rpm_frac_out < 0.0F) {
    frac[1L] = 0.0F;
  } else if (MPC_Y.Rpm_frac_out > 1.0F) {
    frac[1L] = 1.0F;
  }

  bpIndex[0L] = MPC_Y.Te_indx_out;
  bpIndex[1L] = MPC_Y.Rpm_indx_out;

  /* Signum: '<S3>/Sign' */
  if (rtIsNaNF(rtb_Saturation)) {
    rtb_Saturation = (rtNaNF);
  } else if (rtb_Saturation < 0.0F) {
    rtb_Saturation = -1.0F;
  } else {
    rtb_Saturation = (rtb_Saturation > 0.0F);
  }

  /* Switch: '<S3>/Switch' incorporates:
   *  Signum: '<S3>/Sign'
   */
  if (rtb_Saturation >= 0.0F) {
    /* Switch: '<S3>/Switch' incorporates:
     *  Interpolation_n-D: '<S3>/Interpolation Using Prelookup1'
     */
    MPC_Y.iq_ref_cal_out = intrp2d_fu32flm_pw(bpIndex, frac,
      MPC_ConstP.InterpolationUsingPrelookup1_Ta, MPC_ConstP.pooled3);
  } else {
    /* Switch: '<S3>/Switch' incorporates:
     *  Interpolation_n-D: '<S3>/Interpolation Using Prelookup1'
     *  UnaryMinus: '<S3>/Unary Minus'
     */
    MPC_Y.iq_ref_cal_out = -intrp2d_fu32flm_pw(bpIndex, frac,
      MPC_ConstP.InterpolationUsingPrelookup1_Ta, MPC_ConstP.pooled3);
  }

  /* End of Switch: '<S3>/Switch' */

  /* Interpolation_n-D: '<S3>/Interpolation Using Prelookup' */
  frac_0[0L] = MPC_Y.Te_frac_out;
  if (MPC_Y.Te_frac_out < 0.0F) {
    frac_0[0L] = 0.0F;
  } else if (MPC_Y.Te_frac_out > 1.0F) {
    frac_0[0L] = 1.0F;
  }

  frac_0[1L] = MPC_Y.Rpm_frac_out;
  if (MPC_Y.Rpm_frac_out < 0.0F) {
    frac_0[1L] = 0.0F;
  } else if (MPC_Y.Rpm_frac_out > 1.0F) {
    frac_0[1L] = 1.0F;
  }

  bpIndex_0[0L] = MPC_Y.Te_indx_out;
  bpIndex_0[1L] = MPC_Y.Rpm_indx_out;

  /* Interpolation_n-D: '<S3>/Interpolation Using Prelookup' */
  MPC_Y.id_ref_cal_out = intrp2d_fu32flm_pw(bpIndex_0, frac_0,
    MPC_ConstP.InterpolationUsingPrelookup_Tab, MPC_ConstP.pooled3);

  /* End of Outputs for SubSystem: '<S1>/Current reference generation' */

  /* Gain: '<S1>/Gain' incorporates:
   *  Inport: '<Root>/wm'
   */
  rtb_Saturation = 4.0F * MPC_U.wm;

  /* Outputs for Atomic SubSystem: '<S1>/Current controller' */
  /* MATLAB Function: '<S2>/MPC' incorporates:
   *  Inport: '<Root>/Id_meas'
   *  Inport: '<Root>/Iq_meas'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  IdIq_ref[0] = MPC_Y.id_ref_cal_out;
  IdIq_ref[1] = MPC_Y.iq_ref_cal_out;
  H_qp[1] = 0.0F;
  H_qp[2] = 0.0F;
  H_qp[0] = 1.0F;
  H_qp[3] = 1.0F;
  a[0] = 0.0F;
  Su[0] = 0.0F;
  a[1] = 0.0F;
  Su[1] = 0.0F;
  a[2] = 0.0F;
  Su[2] = 0.0F;
  a[3] = 0.0F;
  Su[3] = 0.0F;
  tmp[0] = -0.042727273F;
  tmp[1] = rtb_Saturation * 0.00072F / 0.00055F * 0.0001F;
  rtb_Gain_0[0] = -rtb_Saturation * 0.00055F / 0.00072F * 0.0001F;
  rtb_Gain_0[1] = -0.0326388888F;
  rtb_Saturation = -rtb_Saturation * 0.016F / 0.00072F * 0.0001F;
  Linv[0] = 0.181818187F;
  Linv[1] = 0.0F;
  Linv[2] = 0.0F;
  Linv[3] = 0.138888896F;
  for (i = 0; i < 2; i++) {
    u0 = H_qp[i] + tmp[i];
    e_wm = H_qp[i + 2] + rtb_Gain_0[i];
    b = b_0[i];
    a[b] = 0.0F;
    Su[b] = 0.0F;
    a[b] += u0;
    Su[b] += Linv[i];
    a[b] += e_wm * 0.0F;
    a[b + 2] = 0.0F;
    Su[b + 2] = 0.0F;
    a[b + 2] += u0 * 0.0F;
    a[b + 2] += e_wm;
    Su[b + 2] += Linv[i + 2];
    frac[i] = (real32_T)e[(i << 1U) + 1] * rtb_Saturation;
  }

  frac_0[0] = 0.0F;
  frac_0[1] = 0.0F;
  rtb_Saturation = Su[2];
  u0 = Su[0];
  Su_0 = Su[3];
  Su_1 = Su[1];
  for (i = 0; i < 2; i++) {
    frac_0[b_0[i]] = MPC_DW.UnitDelay_DSTATE[i];
    e_wm = g_1[i + 2];
    g_0 = g_1[i];
    g[i] = e_wm * rtb_Saturation + g_0 * u0;
    g[i + 2] = e_wm * Su_0 + g_0 * Su_1;
  }

  e_wm = g[1];
  g_0 = g[0];
  Su_0 = g[3];
  Su_1 = g[2];
  for (i = 0; i < 2; i++) {
    H_qp[i] = h[i];
    rtb_Saturation = Su[i + 2];
    u0 = Su[i];
    Linv[i] = rtb_Saturation * e_wm + u0 * g_0;
    H_qp[i + 2] = h[i + 2];
    Linv[i + 2] = rtb_Saturation * Su_0 + u0 * Su_1;
  }

  for (i = 0; i < 2; i++) {
    b = i << 1U;
    rtb_Saturation = H_qp[b + 1];
    u0 = H_qp[b];
    T[b] = rtb_Saturation * 0.0F + u0;
    T[b + 1] = u0 * 0.0F + rtb_Saturation;
  }

  H_qp[0] = 2.0F * Linv[0] + 2.0F * T[0];
  H_qp[3] = 2.0F * Linv[3] + 2.0F * T[3];
  g[0] = (H_qp[0] + H_qp[0]) / 2.0F + 1.0E-6F;
  rtb_Saturation = ((2.0F * Linv[1] + 2.0F * T[1]) + (2.0F * Linv[2] + 2.0F * T
    [2])) / 2.0F;
  g[1] = rtb_Saturation;
  g[2] = rtb_Saturation;
  g[3] = (H_qp[3] + H_qp[3]) / 2.0F + 1.0E-6F;
  i = 0;
  b = 0;
  exitg1 = false;
  while ((!exitg1) && (b < 2)) {
    idxAjj = (b << 1U) + b;
    rtb_Saturation = 0.0F;
    if (b >= 1) {
      rtb_Saturation = g[1] * g[1];
    }

    rtb_Saturation = g[idxAjj] - rtb_Saturation;
    if (rtb_Saturation > 0.0F) {
      rtb_Saturation = (real32_T)sqrt(rtb_Saturation);
      g[idxAjj] = rtb_Saturation;
      if (b + 1 < 2) {
        rtb_Saturation = 1.0F / rtb_Saturation;
        for (c_k = idxAjj + 2; c_k <= idxAjj + 2; c_k++) {
          g[c_k - 1] *= rtb_Saturation;
        }
      }

      b++;
    } else {
      g[idxAjj] = rtb_Saturation;
      i = b + 1;
      exitg1 = true;
    }
  }

  if (i == 0) {
    i = 3;
  }

  if (i - 1 >= 2) {
    g[2] = 0.0F;
  }

  Linv[0] = 1.0F;
  H_qp[0] = g[0];
  Linv[2] = 0.0F;
  H_qp[1] = g[2];
  Linv[1] = 0.0F;
  H_qp[2] = g[1];
  Linv[3] = 1.0F;
  H_qp[3] = g[3];
  MPC_trisolve(H_qp, Linv);
  rtb_Saturation = Su[2];
  u0 = Su[0];
  Su_0 = Su[3];
  Su_1 = Su[1];
  for (i = 0; i < 2; i++) {
    b = i << 1U;
    e_wm = g_1[i + 2];
    g_0 = g_1[i];
    g[i] = e_wm * rtb_Saturation + g_0 * u0;
    g[i + 2] = e_wm * Su_0 + g_0 * Su_1;
    tmp[i] = ((a[b + 1] * MPC_U.Iq_meas + a[b] * MPC_U.Id_meas) + frac[i]) -
      IdIq_ref[i];
  }

  rtb_Saturation = tmp[0];
  u0 = tmp[1];
  for (i = 0; i < 2; i++) {
    b = i << 1U;
    H_qp[i] = h[i];
    tmp[i] = g[b + 1] * u0 + g[b] * rtb_Saturation;
    H_qp[i + 2] = h[i + 2];
  }

  rtb_Saturation = frac_0[0];
  u0 = frac_0[1];
  for (i = 0; i < 2; i++) {
    b = i << 1U;
    e_wm = Linv[i + 2];
    a[i] = e_wm * Linv[2] + Linv[i] * Linv[0];
    a[i + 2] = e_wm * Linv[3] + Linv[i] * Linv[1];
    frac[i] = 2.0F * tmp[i] - (H_qp[b + 1] * u0 + H_qp[b] * rtb_Saturation) *
      2.0F;
  }

  MPC_qpkwik(Linv, a, frac, l, MPC_DW.iA_prev, IdIq_ref, H_qp, &exitflag);
  if (exitflag != 1L) {
    IdIq_ref[0] = 0.0F;
    IdIq_ref[1] = 0.0F;
  }

  MPC_DW.UnitDelay_DSTATE[0] = IdIq_ref[0];
  MPC_DW.UnitDelay_DSTATE[1] = IdIq_ref[1];

  /* End of MATLAB Function: '<S2>/MPC' */
  /* End of Outputs for SubSystem: '<S1>/Current controller' */
  /* End of Outputs for SubSystem: '<Root>/MPC' */

  /* Outport: '<Root>/Ud_out' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  MPC_Y.Ud_out = MPC_DW.UnitDelay_DSTATE[0];

  /* Outport: '<Root>/Uq_out' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  MPC_Y.Uq_out = MPC_DW.UnitDelay_DSTATE[1];
}

/* Model initialize function */
void MPC_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* states (dwork) */
  (void) memset((void *)&MPC_DW, 0,
                sizeof(DW_MPC_T));

  /* external inputs */
  (void)memset(&MPC_U, 0, sizeof(ExtU_MPC_T));

  /* external outputs */
  (void)memset(&MPC_Y, 0, sizeof(ExtY_MPC_T));

  /* SystemInitialize for Atomic SubSystem: '<Root>/MPC' */
  /* SystemInitialize for Atomic SubSystem: '<S1>/Current controller' */
  /* SystemInitialize for MATLAB Function: '<S2>/MPC' */
  MPC_DW.iA_prev[0] = false;
  MPC_DW.iA_prev[1] = false;
  MPC_DW.iA_prev[2] = false;
  MPC_DW.iA_prev[3] = false;

  /* End of SystemInitialize for SubSystem: '<S1>/Current controller' */
  /* End of SystemInitialize for SubSystem: '<Root>/MPC' */
}

/* Model terminate function */
void MPC_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
