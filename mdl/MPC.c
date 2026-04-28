/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MPC.c
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

#include "MPC.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "MPC_private.h"
#include "mw_C28x_addsub_s32.h"

/* Block signals (default storage) */
B_MPC_T MPC_B;

/* Block states (default storage) */
DW_MPC_T MPC_DW;

/* External inputs (root inport signals with default storage) */
ExtU_MPC_T MPC_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_MPC_T MPC_Y;

/* Forward declaration for local functions */
static void MPC_trisolve(const real32_T A[144], real32_T B[144]);
static void MPC_Unconstrained(const real32_T Hinv[144], const real32_T f[12],
  real32_T x[12]);
static real32_T MPC_norm(const real32_T x[12]);
static real32_T MPC_maximum(const real32_T x[12]);
static real32_T MPC_xnrm2(int16_T n, const real32_T x[144], int16_T ix0);
static void MPC_xgemv(int16_T m, int16_T n, const real32_T A[144], int16_T ia0,
                      const real32_T x[144], int16_T ix0, real32_T y[12]);
static void MPC_xgerc(int16_T m, int16_T n, real32_T alpha1, int16_T ix0, const
                      real32_T y[12], real32_T A[144], int16_T ia0);
static real32_T MPC_KWIKfactor(const real32_T Ac[288], const int32_T iC[24],
  int32_T nA, const real32_T Linv[144], real32_T RLinv[144], real32_T D[144],
  real32_T H[144]);
static real32_T MPC_mtimes(const real32_T A[12], const real32_T B[12]);
static void MPC_DropConstraint(int32_T kDrop, boolean_T iA[24], int32_T *nA,
  int32_T iC[24]);
static void MPC_qpkwik(const real32_T Linv[144], const real32_T Hinv[144], const
  real32_T f[12], const real32_T Ac[288], boolean_T iA[24], real32_T x[12],
  real32_T lambda[24], int32_T *status);
uint32_T plook_u32ff_binx(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0UL]) {
    bpIndex = 0UL;
    *fraction = (u - bp[0UL]) / (bp[1UL] - bp[0UL]);
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32f(u, bp, maxIndex >> 1UL, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1UL] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1UL;
    *fraction = (u - bp[maxIndex - 1UL]) / (bp[maxIndex] - bp[maxIndex - 1UL]);
  }

  return bpIndex;
}

real32_T intrp2d_fu32fl_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride)
{
  real32_T yL_0d0;
  real32_T yL_0d1;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1UL] * stride + bpIndex[0UL];
  yL_0d0 = table[offset_1d];
  yL_0d0 += (table[offset_1d + 1UL] - yL_0d0) * frac[0UL];
  offset_1d += stride;
  yL_0d1 = table[offset_1d];
  return (((table[offset_1d + 1UL] - yL_0d1) * frac[0UL] + yL_0d1) - yL_0d0) *
    frac[1UL] + yL_0d0;
}

uint32_T binsearch_u32f(real32_T u, const real32_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIdx;
  uint32_T bpIndex;
  uint32_T iRght;

  /* Binary Search */
  bpIdx = startIndex;
  bpIndex = 0UL;
  iRght = maxIndex;
  while (iRght - bpIndex > 1UL) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1UL;
  }

  return bpIndex;
}

int16_T div_nde_s16_floor(int16_T numerator, int16_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_trisolve(const real32_T A[144], real32_T B[144])
{
  real32_T B_0;
  int16_T B_tmp;
  int16_T b_k;
  int16_T i;
  int16_T j;
  int16_T jBcol;
  int16_T kAcol;
  int16_T tmp;
  for (j = 0; j < 12; j++) {
    jBcol = 12 * j;
    for (b_k = 0; b_k < 12; b_k++) {
      kAcol = 12 * b_k;
      B_tmp = b_k + jBcol;
      B_0 = B[B_tmp];
      if (B_0 != 0.0F) {
        B[B_tmp] = B_0 / A[b_k + kAcol];
        for (i = b_k + 2; i < 13; i++) {
          tmp = (i + jBcol) - 1;
          B[tmp] -= A[(i + kAcol) - 1] * B[B_tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_Unconstrained(const real32_T Hinv[144], const real32_T f[12],
  real32_T x[12])
{
  real32_T Hinv_0;
  int16_T i;
  int16_T i_0;
  for (i = 0; i < 12; i++) {
    Hinv_0 = 0.0F;
    for (i_0 = 0; i_0 < 12; i_0++) {
      Hinv_0 += -Hinv[12 * i_0 + i] * f[i_0];
    }

    x[i] = Hinv_0;
  }
}

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if (fabsf(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_norm(const real32_T x[12])
{
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  int16_T k;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 12; k++) {
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt(y);
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_maximum(const real32_T x[12])
{
  real32_T ex;
  real32_T x_0;
  int16_T idx;
  int16_T k;
  boolean_T exitg1;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 13)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 13; k++) {
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_xnrm2(int16_T n, const real32_T x[144], int16_T ix0)
{
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  int16_T k;
  int16_T kend;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = fabsf(x[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabsf(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * (real32_T)sqrt(y);
    }
  }

  return y;
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
static void MPC_xgemv(int16_T m, int16_T n, const real32_T A[144], int16_T ia0,
                      const real32_T x[144], int16_T ix0, real32_T y[12])
{
  real32_T c;
  int16_T b;
  int16_T b_iy;
  int16_T d;
  int16_T ia;
  if (n != 0) {
    memset(&y[0], 0, (uint16_T)n * sizeof(real32_T));
    b = (n - 1) * 12 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 12) {
      c = 0.0F;
      d = (b_iy + m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * A[ia - 1];
      }

      ia = div_nde_s16_floor(b_iy - ia0, 12);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_xgerc(int16_T m, int16_T n, real32_T alpha1, int16_T ix0, const
                      real32_T y[12], real32_T A[144], int16_T ia0)
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

      jA += 12;
    }
  }
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_KWIKfactor(const real32_T Ac[288], const int32_T iC[24],
  int32_T nA, const real32_T Linv[144], real32_T RLinv[144], real32_T D[144],
  real32_T H[144])
{
  int32_T b_k;
  int32_T d_k;
  int32_T exitg1;
  int32_T i;
  real32_T tau[12];
  real32_T work[12];
  real32_T Status;
  real32_T TL;
  real32_T atmp;
  real32_T beta1;
  int16_T b_lastv;
  int16_T c_lastc;
  int16_T f_tmp;
  int16_T i_0;
  int16_T ii;
  int16_T knt;
  boolean_T exitg2;
  Status = 1.0F;
  memset(&RLinv[0], 0, 144U * sizeof(real32_T));
  for (i = 1L; i <= nA; i++) {
    c_lastc = (int16_T)iC[(int16_T)i - 1];
    for (i_0 = 0; i_0 < 12; i_0++) {
      b_lastv = ((int16_T)i - 1) * 12 + i_0;
      RLinv[b_lastv] = 0.0F;
      for (ii = 0; ii < 12; ii++) {
        RLinv[b_lastv] += Ac[(24 * ii + c_lastc) - 1] * Linv[12 * ii + i_0];
      }
    }
  }

  memcpy(&MPC_B.TL[0], &RLinv[0], 144U * sizeof(real32_T));
  for (i_0 = 0; i_0 < 12; i_0++) {
    tau[i_0] = 0.0F;
    work[i_0] = 0.0F;
  }

  for (i_0 = 0; i_0 < 12; i_0++) {
    ii = i_0 * 12 + i_0;
    if (i_0 + 1 < 12) {
      atmp = MPC_B.TL[ii];
      b_lastv = ii + 2;
      tau[i_0] = 0.0F;
      beta1 = MPC_xnrm2(11 - i_0, MPC_B.TL, ii + 2);
      if (beta1 != 0.0F) {
        TL = MPC_B.TL[ii];
        beta1 = rt_hypotf_snf(TL, beta1);
        if (TL >= 0.0F) {
          beta1 = -beta1;
        }

        if (fabsf(beta1) < 9.86076132E-32F) {
          knt = 0;
          f_tmp = (ii - i_0) + 12;
          do {
            knt++;
            for (c_lastc = b_lastv; c_lastc <= f_tmp; c_lastc++) {
              MPC_B.TL[c_lastc - 1] *= 1.01412048E+31F;
            }

            beta1 *= 1.01412048E+31F;
            atmp *= 1.01412048E+31F;
          } while ((fabsf(beta1) < 9.86076132E-32F) && (knt < 20));

          beta1 = rt_hypotf_snf(atmp, MPC_xnrm2(11 - i_0, MPC_B.TL, ii + 2));
          if (atmp >= 0.0F) {
            beta1 = -beta1;
          }

          tau[i_0] = (beta1 - atmp) / beta1;
          atmp = 1.0F / (atmp - beta1);
          for (c_lastc = b_lastv; c_lastc <= f_tmp; c_lastc++) {
            MPC_B.TL[c_lastc - 1] *= atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 9.86076132E-32F;
          }

          atmp = beta1;
        } else {
          tau[i_0] = (beta1 - TL) / beta1;
          atmp = 1.0F / (TL - beta1);
          knt = (ii - i_0) + 12;
          for (c_lastc = b_lastv; c_lastc <= knt; c_lastc++) {
            MPC_B.TL[c_lastc - 1] *= atmp;
          }

          atmp = beta1;
        }
      }

      MPC_B.TL[ii] = 1.0F;
      if (tau[i_0] != 0.0F) {
        b_lastv = 12 - i_0;
        c_lastc = (ii - i_0) + 11;
        while ((b_lastv > 0) && (MPC_B.TL[c_lastc] == 0.0F)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 11 - i_0;
        exitg2 = false;
        while ((!exitg2) && (c_lastc > 0)) {
          knt = ((c_lastc - 1) * 12 + ii) + 12;
          f_tmp = knt;
          do {
            exitg1 = 0L;
            if (f_tmp + 1 <= knt + b_lastv) {
              if (MPC_B.TL[f_tmp] != 0.0F) {
                exitg1 = 1L;
              } else {
                f_tmp++;
              }
            } else {
              c_lastc--;
              exitg1 = 2L;
            }
          } while (exitg1 == 0L);

          if (exitg1 == 1L) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        MPC_xgemv(b_lastv, c_lastc, MPC_B.TL, ii + 13, MPC_B.TL, ii + 1, work);
        MPC_xgerc(b_lastv, c_lastc, -tau[i_0], ii + 1, work, MPC_B.TL, ii + 13);
      }

      MPC_B.TL[ii] = atmp;
    } else {
      tau[11] = 0.0F;
    }
  }

  for (i_0 = 0; i_0 < 12; i_0++) {
    for (ii = 0; ii <= i_0; ii++) {
      MPC_B.R[ii + 12 * i_0] = MPC_B.TL[12 * i_0 + ii];
    }

    for (ii = i_0 + 2; ii < 13; ii++) {
      MPC_B.R[(ii + 12 * i_0) - 1] = 0.0F;
    }

    work[i_0] = 0.0F;
  }

  for (i_0 = 11; i_0 >= 0; i_0--) {
    ii = (i_0 * 12 + i_0) + 13;
    if (i_0 + 1 < 12) {
      MPC_B.TL[ii - 13] = 1.0F;
      if (tau[i_0] != 0.0F) {
        b_lastv = 12 - i_0;
        c_lastc = ii - i_0;
        while ((b_lastv > 0) && (MPC_B.TL[c_lastc - 2] == 0.0F)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 11 - i_0;
        exitg2 = false;
        while ((!exitg2) && (c_lastc > 0)) {
          knt = (c_lastc - 1) * 12 + ii;
          f_tmp = knt;
          do {
            exitg1 = 0L;
            if (f_tmp <= (knt + b_lastv) - 1) {
              if (MPC_B.TL[f_tmp - 1] != 0.0F) {
                exitg1 = 1L;
              } else {
                f_tmp++;
              }
            } else {
              c_lastc--;
              exitg1 = 2L;
            }
          } while (exitg1 == 0L);

          if (exitg1 == 1L) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        MPC_xgemv(b_lastv, c_lastc, MPC_B.TL, ii, MPC_B.TL, ii - 12, work);
        MPC_xgerc(b_lastv, c_lastc, -tau[i_0], ii - 12, work, MPC_B.TL, ii);
      }

      c_lastc = (ii - i_0) - 1;
      for (b_lastv = ii - 11; b_lastv <= c_lastc; b_lastv++) {
        MPC_B.TL[b_lastv - 1] *= -tau[i_0];
      }
    }

    MPC_B.TL[ii - 13] = 1.0F - tau[i_0];
    for (b_lastv = 0; b_lastv < i_0; b_lastv++) {
      MPC_B.TL[(ii - b_lastv) - 14] = 0.0F;
    }
  }

  memcpy(&MPC_B.Q[0], &MPC_B.TL[0], 144U * sizeof(real32_T));
  i = 1L;
  do {
    exitg1 = 0L;
    if (i <= nA) {
      if (fabsf(MPC_B.R[(((int16_T)i - 1) * 12 + (int16_T)i) - 1]) < 1.0E-12F) {
        Status = -2.0F;
        exitg1 = 1L;
      } else {
        i++;
      }
    } else {
      for (ii = 0; ii < 12; ii++) {
        for (b_lastv = 0; b_lastv < 12; b_lastv++) {
          beta1 = 0.0F;
          for (i_0 = 0; i_0 < 12; i_0++) {
            beta1 += Linv[12 * ii + i_0] * MPC_B.Q[12 * b_lastv + i_0];
          }

          MPC_B.TL[ii + 12 * b_lastv] = beta1;
        }
      }

      memset(&RLinv[0], 0, 144U * sizeof(real32_T));
      for (i = nA; i > 0L; i--) {
        i_0 = ((int16_T)i - 1) * 12;
        b_lastv = ((int16_T)i + i_0) - 1;
        RLinv[b_lastv] = 1.0F;
        for (d_k = i; d_k <= nA; d_k++) {
          ii = (((int16_T)d_k - 1) * 12 + (int16_T)i) - 1;
          RLinv[ii] /= MPC_B.R[b_lastv];
        }

        if (i > 1L) {
          for (d_k = 1L; d_k < i; d_k++) {
            for (b_k = i; b_k <= nA; b_k++) {
              b_lastv = ((int16_T)b_k - 1) * 12;
              ii = (b_lastv + (int16_T)d_k) - 1;
              RLinv[ii] -= MPC_B.R[(i_0 + (int16_T)d_k) - 1] * RLinv[(b_lastv +
                (int16_T)i) - 1];
            }
          }
        }
      }

      for (ii = 0; ii < 12; ii++) {
        for (c_lastc = ii; c_lastc + 1 < 13; c_lastc++) {
          i_0 = 12 * c_lastc + ii;
          H[i_0] = 0.0F;
          for (i = nA + 1L; i < 13L; i++) {
            b_lastv = ((int16_T)i - 1) * 12;
            H[i_0] -= MPC_B.TL[b_lastv + ii] * MPC_B.TL[b_lastv + c_lastc];
          }

          H[c_lastc + 12 * ii] = H[i_0];
        }
      }

      for (i = 1L; i <= nA; i++) {
        for (ii = 0; ii < 12; ii++) {
          i_0 = ((int16_T)i - 1) * 12 + ii;
          D[i_0] = 0.0F;
          for (d_k = i; (uint32_T)d_k <= (uint32_T)nA; d_k++) {
            b_lastv = ((int16_T)d_k - 1) * 12;
            D[i_0] += RLinv[(b_lastv + (int16_T)i) - 1] * MPC_B.TL[b_lastv + ii];
          }
        }
      }

      exitg1 = 1L;
    }
  } while (exitg1 == 0L);

  return Status;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static real32_T MPC_mtimes(const real32_T A[12], const real32_T B[12])
{
  real32_T C;
  int16_T k;
  C = 0.0F;
  for (k = 0; k < 12; k++) {
    C += A[k] * B[k];
  }

  return C;
}

/* Function for MATLAB Function: '<S2>/MPC' */
static void MPC_DropConstraint(int32_T kDrop, boolean_T iA[24], int32_T *nA,
  int32_T iC[24])
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
static void MPC_qpkwik(const real32_T Linv[144], const real32_T Hinv[144], const
  real32_T f[12], const real32_T Ac[288], boolean_T iA[24], real32_T x[12],
  real32_T lambda[24], int32_T *status)
{
  int32_T b_i;
  int32_T e_i;
  int32_T exitg1;
  int32_T exitg3;
  int32_T g_i;
  int32_T j_i;
  int32_T kDrop;
  int32_T nA;
  real32_T Rhs[24];
  real32_T Ac_0[12];
  real32_T r[12];
  real32_T z[12];
  real32_T Xnorm0;
  real32_T cMin;
  real32_T cVal;
  real32_T rMin;
  real32_T zTa;
  int16_T f_i;
  int16_T i;
  int16_T tmp;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  for (i = 0; i < 12; i++) {
    x[i] = 0.0F;
  }

  for (i = 0; i < 24; i++) {
    lambda[i] = 0.0F;
  }

  *status = 1L;
  for (i = 0; i < 12; i++) {
    r[i] = 0.0F;
  }

  rMin = 0.0F;
  cTolComputed = false;
  for (i = 0; i < 24; i++) {
    MPC_B.cTol[i] = 1.0F;
    MPC_B.iC[i] = 0L;
  }

  nA = 0L;
  for (i = 0; i < 24; i++) {
    if (iA[i]) {
      nA = (int16_T)nA + 1;
      MPC_B.iC[(int16_T)nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if ((int16_T)nA > 0) {
    for (i = 0; i < 24; i++) {
      MPC_B.Opt[i] = 0.0F;
    }

    for (i = 0; i < 12; i++) {
      Rhs[i] = f[i];
      Rhs[i + 12] = 0.0F;
    }

    DualFeasible = false;
    tmp = (int16_T)rt_roundf_snf(0.3F * (real32_T)nA);
    ColdReset = false;
    do {
      exitg3 = 0L;
      if ((!DualFeasible) && (nA > 0L) && ((int16_T)*status <= 200)) {
        Xnorm0 = MPC_KWIKfactor(Ac, MPC_B.iC, nA, Linv, MPC_B.RLinv, MPC_B.D,
          MPC_B.H);
        if (Xnorm0 < 0.0F) {
          if (ColdReset) {
            *status = -2L;
            exitg3 = 2L;
          } else {
            for (i = 0; i < 24; i++) {
              iA[i] = false;
              MPC_B.iC[i] = 0L;
            }

            nA = 0L;
            ColdReset = true;
          }
        } else {
          for (kDrop = 1L; kDrop <= nA; kDrop++) {
            if (kDrop > 2147483635L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 12L;
            }

            Rhs[(int16_T)b_i - 1] = -24.0F;
            for (b_i = kDrop; b_i <= nA; b_i++) {
              i = (((int16_T)kDrop - 1) * 12 + (int16_T)b_i) - 1;
              MPC_B.U[i] = 0.0F;
              for (e_i = 1L; e_i <= nA; e_i++) {
                f_i = ((int16_T)e_i - 1) * 12;
                MPC_B.U[i] += MPC_B.RLinv[(f_i + (int16_T)b_i) - 1] *
                  MPC_B.RLinv[(f_i + (int16_T)kDrop) - 1];
              }

              MPC_B.U[((int16_T)kDrop + 12 * ((int16_T)b_i - 1)) - 1] =
                MPC_B.U[i];
            }
          }

          for (f_i = 0; f_i < 12; f_i++) {
            Xnorm0 = 0.0F;
            for (i = 0; i < 12; i++) {
              Xnorm0 += MPC_B.H[12 * i + f_i] * Rhs[i];
            }

            MPC_B.Opt[f_i] = Xnorm0;
            for (kDrop = 1L; (uint32_T)kDrop <= (uint32_T)nA; kDrop++) {
              if (kDrop > 2147483635L) {
                b_i = MAX_int32_T;
              } else {
                b_i = kDrop + 12L;
              }

              MPC_B.Opt[f_i] += MPC_B.D[((int16_T)kDrop - 1) * 12 + f_i] * Rhs
                [(int16_T)b_i - 1];
            }
          }

          for (kDrop = 1L; (uint32_T)kDrop <= (uint32_T)nA; kDrop++) {
            Xnorm0 = 0.0F;
            for (i = 0; i < 12; i++) {
              Xnorm0 += MPC_B.D[((int16_T)kDrop - 1) * 12 + i] * Rhs[i];
            }

            if (kDrop > 2147483635L) {
              b_i = MAX_int32_T;
            } else {
              b_i = kDrop + 12L;
            }

            MPC_B.Opt[(int16_T)b_i - 1] = Xnorm0;
            if ((uint32_T)nA >= 1UL) {
              if (kDrop > 2147483635L) {
                g_i = MAX_int32_T;
                j_i = MAX_int32_T;
              } else {
                g_i = kDrop + 12L;
                j_i = kDrop + 12L;
              }
            }

            for (e_i = 1L; (uint32_T)e_i <= (uint32_T)nA; e_i++) {
              if (e_i > 2147483635L) {
                b_i = MAX_int32_T;
              } else {
                b_i = e_i + 12L;
              }

              MPC_B.Opt[(int16_T)g_i - 1] = MPC_B.U[(((int16_T)e_i - 1) * 12 +
                (int16_T)kDrop) - 1] * Rhs[(int16_T)b_i - 1] + MPC_B.Opt
                [(int16_T)j_i - 1];
            }
          }

          Xnorm0 = -1.0E-12F;
          kDrop = 0L;
          for (e_i = 1L; (uint32_T)e_i <= (uint32_T)nA; e_i++) {
            if (e_i > 2147483635L) {
              b_i = MAX_int32_T;
            } else {
              b_i = e_i + 12L;
            }

            lambda[(int16_T)MPC_B.iC[(int16_T)e_i - 1] - 1] = MPC_B.Opt[(int16_T)
              b_i - 1];
            if (e_i > 2147483635L) {
              b_i = MAX_int32_T;
            } else {
              b_i = e_i + 12L;
            }

            if ((MPC_B.Opt[(int16_T)b_i - 1] < Xnorm0) && (e_i <= nA)) {
              kDrop = e_i;
              if (e_i > 2147483635L) {
                b_i = MAX_int32_T;
              } else {
                b_i = e_i + 12L;
              }

              Xnorm0 = MPC_B.Opt[(int16_T)b_i - 1];
            }
          }

          if (kDrop <= 0L) {
            DualFeasible = true;
            for (i = 0; i < 12; i++) {
              x[i] = MPC_B.Opt[i];
            }
          } else {
            *status = (int16_T)*status + 1;
            if (tmp <= 5) {
              i = 5;
            } else {
              i = tmp;
            }

            if ((int16_T)*status > i) {
              for (i = 0; i < 24; i++) {
                iA[i] = false;
                MPC_B.iC[i] = 0L;
              }

              nA = 0L;
              ColdReset = true;
            } else {
              lambda[(int16_T)MPC_B.iC[(int16_T)kDrop - 1] - 1] = 0.0F;
              MPC_DropConstraint(kDrop, iA, &nA, MPC_B.iC);
            }
          }
        }
      } else {
        if (nA <= 0L) {
          for (i = 0; i < 24; i++) {
            lambda[i] = 0.0F;
          }

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
      tmp = -1;
      for (f_i = 0; f_i < 24; f_i++) {
        if (!cTolComputed) {
          for (i = 0; i < 12; i++) {
            z[i] = fabsf(Ac[24 * i + f_i] * x[i]);
          }

          cVal = MPC_B.cTol[f_i];
          zTa = MPC_maximum(z);
          if ((cVal >= zTa) || rtIsNaNF(zTa)) {
            MPC_B.cTol[f_i] = cVal;
          } else {
            MPC_B.cTol[f_i] = zTa;
          }
        }

        if (!iA[f_i]) {
          cVal = 0.0F;
          for (i = 0; i < 12; i++) {
            cVal += Ac[24 * i + f_i] * x[i];
          }

          cVal = (cVal - -24.0F) / MPC_B.cTol[f_i];
          if (cVal < cMin) {
            cMin = cVal;
            tmp = f_i;
          }
        }
      }

      cTolComputed = true;
      if (tmp + 1 <= 0) {
        exitg2 = true;
      } else if ((int16_T)*status == 200) {
        *status = 0L;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0L;
          if ((tmp + 1 > 0) && ((int16_T)*status <= 200)) {
            guard2 = false;
            if (nA == 0L) {
              for (i = 0; i < 12; i++) {
                cMin = 0.0F;
                for (f_i = 0; f_i < 12; f_i++) {
                  cMin += Hinv[12 * f_i + i] * Ac[24 * f_i + tmp];
                }

                z[i] = cMin;
              }

              guard2 = true;
            } else {
              cMin = MPC_KWIKfactor(Ac, MPC_B.iC, nA, Linv, MPC_B.RLinv, MPC_B.D,
                                    MPC_B.H);
              if (cMin <= 0.0F) {
                *status = -2L;
                exitg1 = 1L;
              } else {
                for (i = 0; i < 144; i++) {
                  MPC_B.U[i] = -MPC_B.H[i];
                }

                for (i = 0; i < 12; i++) {
                  cMin = 0.0F;
                  for (f_i = 0; f_i < 12; f_i++) {
                    cMin += MPC_B.U[12 * f_i + i] * Ac[24 * f_i + tmp];
                  }

                  z[i] = cMin;
                }

                for (g_i = 1L; g_i <= nA; g_i++) {
                  cVal = 0.0F;
                  for (i = 0; i < 12; i++) {
                    cVal += MPC_B.D[((int16_T)g_i - 1) * 12 + i] * Ac[24 * i +
                      tmp];
                  }

                  r[(int16_T)g_i - 1] = cVal;
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
                    cVal = lambda[(int16_T)MPC_B.iC[(int16_T)g_i - 1] - 1] /
                      cVal;
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

              for (i = 0; i < 12; i++) {
                Ac_0[i] = Ac[24 * i + tmp];
              }

              zTa = MPC_mtimes(z, Ac_0);
              if (zTa <= 0.0F) {
                cVal = 0.0F;
                ColdReset = true;
              } else {
                cVal = 0.0F;
                for (i = 0; i < 12; i++) {
                  cVal += Ac[24 * i + tmp] * x[i];
                }

                cVal = (-24.0F - cVal) / zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1L;
                exitg1 = 1L;
              } else {
                if (ColdReset) {
                  zTa = cMin;
                } else if (DualFeasible) {
                  zTa = cVal;
                } else if (cMin < cVal) {
                  zTa = cMin;
                } else {
                  zTa = cVal;
                }

                for (g_i = 1L; g_i <= nA; g_i++) {
                  i = (int16_T)MPC_B.iC[(int16_T)g_i - 1] - 1;
                  lambda[i] -= r[(int16_T)g_i - 1] * zTa;
                  if (lambda[i] < 0.0F) {
                    lambda[i] = 0.0F;
                  }
                }

                lambda[tmp] += zTa;
                if (fabsf(zTa - cMin) < 1.1920929E-7F) {
                  MPC_DropConstraint(kDrop, iA, &nA, MPC_B.iC);
                }

                if (!ColdReset) {
                  for (i = 0; i < 12; i++) {
                    x[i] += zTa * z[i];
                  }

                  if (fabsf(zTa - cVal) < 1.1920929E-7F) {
                    if (nA == 12L) {
                      *status = -1L;
                      exitg1 = 1L;
                    } else {
                      nA = c28x_add_s32_s32_s32_sat(nA, 1L);
                      MPC_B.iC[(int16_T)nA - 1] = tmp + 1;
                      j_i = nA;
                      exitg4 = false;
                      while ((!exitg4) && (j_i > 1L)) {
                        b_i = MPC_B.iC[(int16_T)j_i - 1];
                        g_i = MPC_B.iC[(int16_T)(j_i - 1L) - 1];
                        if (b_i > g_i) {
                          exitg4 = true;
                        } else {
                          MPC_B.iC[(int16_T)j_i - 1] = g_i;
                          MPC_B.iC[(int16_T)(j_i - 1L) - 1] = b_i;
                          j_i--;
                        }
                      }

                      iA[tmp] = true;
                      tmp = -1;
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
              for (i = 0; i < 24; i++) {
                MPC_B.cTol[i] = 24.0F;
              }

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
  real32_T IdIq_ref[12];
  real32_T Sh[12];
  real32_T T[12];
  real32_T a[12];
  real32_T b[12];
  real32_T A_d[4];
  real32_T A_d_pow_i[4];
  real32_T A_d_pow_i_0[4];
  real32_T A_d_pow_i_tmp[4];
  real32_T Su_tmp[4];
  real32_T Su_tmp_0[4];
  real32_T Su_tmp_1[4];
  real32_T frac[2];
  real32_T frac_0[2];
  real32_T tmp[2];
  real32_T A_d_0;
  real32_T A_d_1;
  real32_T A_d_pow_i_1;
  real32_T A_d_pow_i_tmp_0;
  real32_T A_d_pow_j;
  real32_T A_d_pow_j_0;
  real32_T A_d_pow_j_1;
  real32_T A_d_pow_j_2;
  real32_T B_d_idx_0;
  real32_T B_d_idx_2_tmp;
  real32_T B_d_idx_3;
  real32_T e_wm;
  real32_T rtb_DiscreteTimeIntegrator_idx_;
  real32_T rtb_Gain_idx_0_tmp;
  real32_T rtb_Gain_idx_1_tmp;
  real32_T rtb_PrelookupRPM_o2;
  real32_T rtb_PrelookupTe_o2;
  real32_T tmp_0;
  uint32_T bpIndex[2];
  uint32_T bpIndex_0[2];
  uint32_T rtb_PrelookupRPM_o1;
  uint32_T rtb_PrelookupTe_o1;
  int16_T A_d_tmp;
  int16_T Su_tmp_2;
  int16_T T_tmp;
  int16_T T_tmp_0;
  int16_T e;
  int16_T i;
  int16_T ia;
  int16_T idxAjj;
  static const real32_T y[144] = { 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F };

  static const real32_T b_y[144] = { 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F };

  static const real32_T h[144] = { 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0E-6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0E-6F };

  static const int16_T B[144] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const real32_T l[288] = { -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -1.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -1.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  boolean_T exitg1;

  /* Outputs for Atomic SubSystem: '<Root>/MPC' */
  /* Outputs for Atomic SubSystem: '<S1>/Subsystem' */
  /* DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_idx_ = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
   *  Inport: '<Root>/wm'
   */
  e_wm = MPC_U.wm - MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
  rtb_PrelookupTe_o2 = MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
   */
  rtb_PrelookupRPM_o2 = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];
  tmp_0 = MPC_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/Id_meas'
   *  Inport: '<Root>/Iq_meas'
   *  MATLAB Function: '<S1>/Te calculation'
   *  MATLAB Function: '<S4>/MATLAB Function'
   */
  rtb_Gain_idx_0_tmp = MPC_DW.DiscreteTimeIntegrator_DSTATE[1];
  MPC_DW.DiscreteTimeIntegrator_DSTATE[0] = ((((-0.00135187327F * MPC_U.Id_meas
    + 1.0F) * (0.194175F * MPC_U.Iq_meas) - rtb_PrelookupRPM_o2) - MPC_P.B *
    tmp_0) * (real32_T)(1.0 / MPC_P.J) + (2000.0F - MPC_P.B / (real32_T)MPC_P.J)
    * e_wm) * MPC_P.DiscreteTimeIntegrator_gainval + rtb_PrelookupTe_o2;
  MPC_DW.DiscreteTimeIntegrator_DSTATE[1] = (real32_T)-MPC_P.J * 1.0E+6F * e_wm *
    MPC_P.DiscreteTimeIntegrator_gainval + rtb_Gain_idx_0_tmp;

  /* End of Outputs for SubSystem: '<S1>/Subsystem' */

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Te_ref'
   */
  rtb_DiscreteTimeIntegrator_idx_ += MPC_U.Te_ref;

  /* Outputs for Atomic SubSystem: '<S1>/Current reference generation' */
  /* Abs: '<S3>/Abs' */
  rtb_PrelookupTe_o2 = fabsf(rtb_DiscreteTimeIntegrator_idx_);

  /* PreLookup: '<S3>/Prelookup Te' */
  rtb_PrelookupTe_o1 = plook_u32ff_binx(rtb_PrelookupTe_o2,
    MPC_P.PrelookupTe_BreakpointsData, 11UL, &rtb_PrelookupTe_o2);

  /* Abs: '<S3>/Abs1' incorporates:
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/wm'
   */
  rtb_PrelookupRPM_o2 = fabsf(MPC_P.Gain1_Gain * MPC_U.wm);

  /* Saturate: '<S3>/Saturation' */
  if (rtb_PrelookupRPM_o2 > MPC_P.rpm_max) {
    rtb_PrelookupRPM_o2 = MPC_P.rpm_max;
  } else if (rtb_PrelookupRPM_o2 < MPC_P.Saturation_LowerSat) {
    rtb_PrelookupRPM_o2 = MPC_P.Saturation_LowerSat;
  }

  /* End of Saturate: '<S3>/Saturation' */

  /* PreLookup: '<S3>/Prelookup RPM' */
  rtb_PrelookupRPM_o1 = plook_u32ff_binx(rtb_PrelookupRPM_o2,
    MPC_P.speed_rpm_vec, 19UL, &rtb_PrelookupRPM_o2);

  /* Interpolation_n-D: '<S3>/Interpolation Using Prelookup' */
  frac[0L] = rtb_PrelookupTe_o2;
  frac[1L] = rtb_PrelookupRPM_o2;
  bpIndex[0L] = rtb_PrelookupTe_o1;
  bpIndex[1L] = rtb_PrelookupRPM_o1;
  e_wm = intrp2d_fu32fl_pw(bpIndex, frac, MPC_P.LUT_Id, 12UL);

  /* Interpolation_n-D: '<S3>/Interpolation Using Prelookup1' */
  frac_0[0L] = rtb_PrelookupTe_o2;
  frac_0[1L] = rtb_PrelookupRPM_o2;
  bpIndex_0[0L] = rtb_PrelookupTe_o1;
  bpIndex_0[1L] = rtb_PrelookupRPM_o1;

  /* Signum: '<S3>/Sign' */
  if (rtIsNaNF(rtb_DiscreteTimeIntegrator_idx_)) {
    rtb_PrelookupTe_o2 = (rtNaNF);
  } else if (rtb_DiscreteTimeIntegrator_idx_ < 0.0F) {
    rtb_PrelookupTe_o2 = -1.0F;
  } else {
    rtb_PrelookupTe_o2 = (rtb_DiscreteTimeIntegrator_idx_ > 0.0F);
  }

  /* Switch: '<S3>/Switch' incorporates:
   *  Interpolation_n-D: '<S3>/Interpolation Using Prelookup1'
   *  Signum: '<S3>/Sign'
   *  UnaryMinus: '<S3>/Unary Minus'
   */
  if (rtb_PrelookupTe_o2 >= MPC_P.Switch_Threshold) {
    rtb_PrelookupTe_o2 = intrp2d_fu32fl_pw(bpIndex_0, frac_0, MPC_P.LUT_Iq, 12UL);
  } else {
    rtb_PrelookupTe_o2 = -intrp2d_fu32fl_pw(bpIndex_0, frac_0, MPC_P.LUT_Iq,
      12UL);
  }

  /* End of Switch: '<S3>/Switch' */
  /* End of Outputs for SubSystem: '<S1>/Current reference generation' */

  /* Gain: '<S1>/Gain' incorporates:
   *  Inport: '<Root>/wm'
   */
  rtb_DiscreteTimeIntegrator_idx_ = MPC_P.p * MPC_U.wm;

  /* Outputs for Atomic SubSystem: '<S1>/Current controller' */
  /* MATLAB Function: '<S2>/MPC' */
  rtb_PrelookupRPM_o2 = -MPC_P.Rs / MPC_P.Ld * MPC_P.Tsw;
  tmp_0 = rtb_DiscreteTimeIntegrator_idx_ * MPC_P.Lq / MPC_P.Ld * MPC_P.Tsw;
  rtb_Gain_idx_0_tmp = -rtb_DiscreteTimeIntegrator_idx_ * MPC_P.Ld / MPC_P.Lq *
    MPC_P.Tsw;
  rtb_Gain_idx_1_tmp = -MPC_P.Rs / MPC_P.Lq * MPC_P.Tsw;
  A_d[0] = rtb_PrelookupRPM_o2 + 1.0F;
  A_d[1] = rtb_Gain_idx_0_tmp;
  A_d[2] = tmp_0;
  A_d[3] = rtb_Gain_idx_1_tmp + 1.0F;
  B_d_idx_0 = 1.0F / MPC_P.Ld * MPC_P.Tsw;
  B_d_idx_2_tmp = 0.0F * MPC_P.Tsw;
  B_d_idx_3 = 1.0F / MPC_P.Lq * MPC_P.Tsw;
  rtb_DiscreteTimeIntegrator_idx_ = -rtb_DiscreteTimeIntegrator_idx_ *
    MPC_P.psi_m / MPC_P.Lq * MPC_P.Tsw;
  for (i = 0; i < 24; i++) {
    MPC_B.a[i] = 0.0F;
  }

  memset(&MPC_B.Su[0], 0, 144U * sizeof(real32_T));
  for (i = 0; i < 12; i++) {
    Sh[i] = 0.0F;
  }

  A_d_pow_i[0] = 1.0F;
  A_d_pow_i[1] = 0.0F;
  A_d_pow_i[2] = 0.0F;
  A_d_pow_i[3] = 1.0F;
  for (i = 0; i < 2; i++) {
    A_d_tmp = i << 1U;
    A_d_0 = A_d[A_d_tmp + 1];
    A_d_1 = A_d[A_d_tmp];
    A_d_pow_i_tmp_0 = A_d_0 * 0.0F + A_d_1;
    A_d_pow_i_tmp[A_d_tmp] = A_d_pow_i_tmp_0;
    A_d_0 += A_d_1 * 0.0F;
    A_d_pow_i_tmp[A_d_tmp + 1] = A_d_0;
    MPC_B.a[12 * i] = A_d_pow_i_tmp_0;
    A_d_pow_i_1 = A_d_pow_i[i];
    A_d_pow_i_tmp_0 = A_d_pow_i_1 * B_d_idx_2_tmp;
    A_d_1 = A_d_pow_i[i + 2];
    Su_tmp[i] = A_d_1 * B_d_idx_2_tmp + A_d_pow_i_1 * B_d_idx_0;
    MPC_B.a[12 * i + 1] = A_d_0;
    A_d_0 = A_d_1 * rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_i_tmp_0;
    frac[i] = A_d_0;
    Su_tmp[i + 2] = A_d_1 * B_d_idx_3 + A_d_pow_i_tmp_0;
    Sh[i] = A_d_0;
  }

  MPC_B.Su[26] = Su_tmp[0];
  MPC_B.Su[27] = Su_tmp[1];
  Sh[2] = (A_d_pow_i_tmp[0] * B_d_idx_2_tmp + A_d_pow_i_tmp[2] *
           rtb_DiscreteTimeIntegrator_idx_) + frac[0];
  MPC_B.Su[38] = Su_tmp[2];
  MPC_B.Su[39] = Su_tmp[3];
  Sh[3] = (A_d_pow_i_tmp[1] * B_d_idx_2_tmp + A_d_pow_i_tmp[3] *
           rtb_DiscreteTimeIntegrator_idx_) + frac[1];
  for (i = 0; i < 2; i++) {
    A_d_tmp = i << 1U;
    MPC_B.Su[12 * i] = Su_tmp[A_d_tmp];
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i + 2];
    A_d_0 = A_d_pow_i_tmp[i];
    A_d_pow_i_1 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 *
      rtb_Gain_idx_0_tmp;
    A_d_pow_i[i] = A_d_pow_i_1;
    MPC_B.Su[12 * i + 1] = Su_tmp[A_d_tmp + 1];
    A_d_1 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 * tmp_0;
    A_d_pow_i[i + 2] = A_d_1;
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_i_1 + A_d_1 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 2] = A_d_pow_i_tmp_0 * B_d_idx_2_tmp + A_d_0 * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_1 + A_d_pow_i_1 *
      tmp_0;
    MPC_B.Su[i + 14] = A_d_pow_i_tmp_0 * B_d_idx_3 + A_d_0 * B_d_idx_2_tmp;
  }

  MPC_B.a[2] = A_d_pow_i[0];
  MPC_B.a[3] = A_d_pow_i[1];
  MPC_B.a[14] = A_d_pow_i[2];
  MPC_B.a[15] = A_d_pow_i[3];
  A_d_pow_i[0] = A_d_pow_i_0[0];
  A_d_pow_i[1] = A_d_pow_i_0[1];
  A_d_pow_i[2] = A_d_pow_i_0[2];
  A_d_pow_i[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    idxAjj = i << 1U;
    MPC_B.a[12 * i + 4] = A_d_pow_i[idxAjj];
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i];
    A_d_1 = A_d_pow_i_tmp_0 * B_d_idx_2_tmp;
    A_d_tmp = (i + 4) * 12;
    MPC_B.Su[A_d_tmp + 4] = Su_tmp[idxAjj];
    A_d_0 = A_d_pow_i_tmp[i + 2];
    A_d_pow_j = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 *
      rtb_Gain_idx_0_tmp;
    Su_tmp_0[i] = A_d_0 * B_d_idx_2_tmp + A_d_pow_i_tmp_0 * B_d_idx_0;
    A_d[i] = A_d_pow_j;
    MPC_B.a[12 * i + 5] = A_d_pow_i[idxAjj + 1];
    A_d_pow_i_1 = A_d_0 * rtb_DiscreteTimeIntegrator_idx_ + A_d_1;
    tmp[i] = A_d_pow_i_1;
    MPC_B.Su[A_d_tmp + 5] = Su_tmp[idxAjj + 1];
    A_d_pow_j_0 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 * tmp_0;
    Su_tmp_0[i + 2] = A_d_0 * B_d_idx_3 + A_d_1;
    A_d[i + 2] = A_d_pow_j_0;
    Sh[i + 4] = (A_d_pow_j * B_d_idx_2_tmp + A_d_pow_j_0 *
                 rtb_DiscreteTimeIntegrator_idx_) + (frac[i] + A_d_pow_i_1);
  }

  for (i = 0; i < 2; i++) {
    A_d_tmp = i << 1U;
    Su_tmp_2 = (i + 2) * 12;
    MPC_B.Su[Su_tmp_2 + 4] = Su_tmp_0[A_d_tmp];
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i + 2];
    A_d_0 = A_d_pow_i_tmp[i];
    A_d_pow_j = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 *
      rtb_Gain_idx_0_tmp;
    Su_tmp_1[i] = A_d_pow_j;
    MPC_B.Su[Su_tmp_2 + 5] = Su_tmp_0[A_d_tmp + 1];
    A_d_pow_i_tmp_0 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 *
      tmp_0;
    Su_tmp_1[i + 2] = A_d_pow_i_tmp_0;
    A_d_pow_i_1 = A_d_pow_i[i + 2];
    A_d_1 = A_d_pow_i[i];
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_1 + A_d_pow_i_1 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 4] = A_d_pow_i_tmp_0 * B_d_idx_2_tmp + A_d_pow_j * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_1 + A_d_1 *
      tmp_0;
    MPC_B.Su[i + 16] = A_d_pow_i_tmp_0 * B_d_idx_3 + A_d_pow_j * B_d_idx_2_tmp;
  }

  A_d_pow_i[0] = A_d_pow_i_0[0];
  A_d_pow_i[1] = A_d_pow_i_0[1];
  A_d_pow_i[2] = A_d_pow_i_0[2];
  A_d_pow_i[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    idxAjj = i << 1U;
    MPC_B.a[12 * i + 6] = A_d_pow_i[idxAjj];
    A_d_pow_j = A_d[i + 2];
    A_d_pow_j_0 = A_d[i];
    A_d_tmp = (i + 6) * 12;
    MPC_B.Su[A_d_tmp + 6] = Su_tmp[idxAjj];
    Su_tmp_2 = (i + 4) * 12;
    MPC_B.Su[Su_tmp_2 + 6] = Su_tmp_0[idxAjj];
    MPC_B.a[12 * i + 7] = A_d_pow_i[idxAjj + 1];
    MPC_B.Su[A_d_tmp + 7] = Su_tmp[idxAjj + 1];
    MPC_B.Su[Su_tmp_2 + 7] = Su_tmp_0[idxAjj + 1];
    Sh[i + 6] = (((rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 * tmp_0)
                 * rtb_DiscreteTimeIntegrator_idx_ + ((rtb_PrelookupRPM_o2 +
      1.0F) * A_d_pow_j_0 + A_d_pow_j * rtb_Gain_idx_0_tmp) * B_d_idx_2_tmp) +
      ((A_d_pow_j * rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j_0 *
        B_d_idx_2_tmp) + (frac[i] + tmp[i]));
    A_d_pow_j = Su_tmp_1[i];
    A_d_pow_i_tmp_0 = Su_tmp_1[i + 2];
    A_d_0 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_i_tmp_0 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 30] = A_d_pow_i_tmp_0 * B_d_idx_2_tmp + A_d_pow_j * B_d_idx_0;
    A_d_1 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 + A_d_pow_j * tmp_0;
    MPC_B.Su[i + 42] = A_d_pow_i_tmp_0 * B_d_idx_3 + A_d_pow_j * B_d_idx_2_tmp;
    A_d_pow_i_tmp_0 = A_d_pow_i[i + 2];
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_i[i] +
      A_d_pow_i_tmp_0 * rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 6] = A_d_1 * B_d_idx_2_tmp + A_d_0 * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 +
      A_d_pow_i[i] * tmp_0;
    MPC_B.Su[i + 18] = A_d_1 * B_d_idx_3 + A_d_0 * B_d_idx_2_tmp;
  }

  A_d_pow_i[0] = A_d_pow_i_0[0];
  A_d_pow_i[1] = A_d_pow_i_0[1];
  A_d_pow_i[2] = A_d_pow_i_0[2];
  A_d_pow_i[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    idxAjj = i << 1U;
    MPC_B.a[12 * i + 8] = A_d_pow_i[idxAjj];
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i + 2];
    A_d_0 = A_d_pow_i_tmp[i];
    A_d_pow_j = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 *
      rtb_Gain_idx_0_tmp;
    MPC_B.a[12 * i + 9] = A_d_pow_i[idxAjj + 1];
    A_d_pow_j_0 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 * tmp_0;
    frac_0[i] = ((A_d_pow_i_tmp_0 * rtb_DiscreteTimeIntegrator_idx_ + A_d_0 *
                  B_d_idx_2_tmp) + frac[i]) + (A_d_pow_j_0 *
      rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j * B_d_idx_2_tmp);
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      rtb_Gain_idx_0_tmp;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      tmp_0;
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i + 2];
    A_d_pow_j_0 = A_d[i];
    A_d_tmp = i << 1U;
    Su_tmp_2 = (i + 8) * 12;
    MPC_B.Su[Su_tmp_2 + 8] = Su_tmp[A_d_tmp];
    MPC_B.Su[Su_tmp_2 + 9] = Su_tmp[A_d_tmp + 1];
    Sh[i + 8] = (((rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 * tmp_0)
                 * rtb_DiscreteTimeIntegrator_idx_ + ((rtb_PrelookupRPM_o2 +
      1.0F) * A_d_pow_j_0 + A_d_pow_j * rtb_Gain_idx_0_tmp) * B_d_idx_2_tmp) +
      ((A_d_pow_j * rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j_0 *
        B_d_idx_2_tmp) + frac_0[i]);
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i];
    A_d_0 = A_d_pow_i_tmp[i + 2];
    A_d_pow_j_0 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 *
      rtb_Gain_idx_0_tmp;
    A_d[i] = A_d_pow_j_0;
    MPC_B.Su[i + 80] = A_d_0 * B_d_idx_2_tmp + A_d_pow_i_tmp_0 * B_d_idx_0;
    A_d_pow_j = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 * tmp_0;
    A_d[i + 2] = A_d_pow_j;
    MPC_B.Su[i + 92] = A_d_0 * B_d_idx_3 + A_d_pow_i_tmp_0 * B_d_idx_2_tmp;
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 56] = A_d_pow_j * B_d_idx_2_tmp + A_d_pow_j_0 * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      tmp_0;
    MPC_B.Su[i + 68] = A_d_pow_j * B_d_idx_3 + A_d_pow_j_0 * B_d_idx_2_tmp;
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i];
    A_d_pow_j_0 = A_d[i + 2];
    A_d_pow_j_1 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 32] = A_d_pow_j_0 * B_d_idx_2_tmp + A_d_pow_j * B_d_idx_0;
    A_d_pow_j_2 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j_0 + A_d_pow_j * tmp_0;
    MPC_B.Su[i + 44] = A_d_pow_j_0 * B_d_idx_3 + A_d_pow_j * B_d_idx_2_tmp;
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i + 2];
    A_d_0 = A_d_pow_i_tmp[i];
    A_d_pow_j = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 *
      rtb_Gain_idx_0_tmp;
    A_d[i] = A_d_pow_j;
    A_d_pow_i_1 = A_d_pow_i[i + 2];
    A_d_1 = A_d_pow_i[i];
    MPC_B.a[i + 10] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_1 + A_d_pow_i_1 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 8] = A_d_pow_j_2 * B_d_idx_2_tmp + A_d_pow_j_1 * B_d_idx_0;
    A_d_pow_j_0 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 * tmp_0;
    A_d[i + 2] = A_d_pow_j_0;
    MPC_B.a[i + 22] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_i_1 + A_d_1 * tmp_0;
    MPC_B.Su[i + 20] = A_d_pow_j_2 * B_d_idx_3 + A_d_pow_j_1 * B_d_idx_2_tmp;
    frac_0[i] = ((A_d_pow_i_tmp_0 * rtb_DiscreteTimeIntegrator_idx_ + A_d_0 *
                  B_d_idx_2_tmp) + frac[i]) + (A_d_pow_j_0 *
      rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j * B_d_idx_2_tmp);
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      rtb_Gain_idx_0_tmp;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      tmp_0;
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i + 2];
    A_d_pow_j_0 = A_d[i];
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      rtb_Gain_idx_0_tmp;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      tmp_0;
    tmp[i] = (A_d_pow_j * rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j_0 *
              B_d_idx_2_tmp) + frac_0[i];
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i + 2];
    A_d_pow_j_0 = A_d[i];
    A_d_tmp = i << 1U;
    Su_tmp_2 = (i + 10) * 12;
    MPC_B.Su[Su_tmp_2 + 10] = Su_tmp[A_d_tmp];
    MPC_B.Su[Su_tmp_2 + 11] = Su_tmp[A_d_tmp + 1];
    Sh[i + 10] = (((rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 * tmp_0)
                  * rtb_DiscreteTimeIntegrator_idx_ + ((rtb_PrelookupRPM_o2 +
      1.0F) * A_d_pow_j_0 + A_d_pow_j * rtb_Gain_idx_0_tmp) * B_d_idx_2_tmp) +
      ((A_d_pow_j * rtb_DiscreteTimeIntegrator_idx_ + A_d_pow_j_0 *
        B_d_idx_2_tmp) + tmp[i]);
    A_d_pow_i_tmp_0 = A_d_pow_i_tmp[i];
    A_d_0 = A_d_pow_i_tmp[i + 2];
    A_d_pow_j_0 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_i_tmp_0 + A_d_0 *
      rtb_Gain_idx_0_tmp;
    A_d[i] = A_d_pow_j_0;
    MPC_B.Su[i + 106] = A_d_0 * B_d_idx_2_tmp + A_d_pow_i_tmp_0 * B_d_idx_0;
    A_d_pow_j = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_0 + A_d_pow_i_tmp_0 * tmp_0;
    A_d[i + 2] = A_d_pow_j;
    MPC_B.Su[i + 118] = A_d_0 * B_d_idx_3 + A_d_pow_i_tmp_0 * B_d_idx_2_tmp;
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 82] = A_d_pow_j * B_d_idx_2_tmp + A_d_pow_j_0 * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      tmp_0;
    MPC_B.Su[i + 94] = A_d_pow_j * B_d_idx_3 + A_d_pow_j_0 * B_d_idx_2_tmp;
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i];
    A_d_pow_j_0 = A_d[i + 2];
    A_d_pow_i_0[i] = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 58] = A_d_pow_j_0 * B_d_idx_2_tmp + A_d_pow_j * B_d_idx_0;
    A_d_pow_i_0[i + 2] = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j_0 + A_d_pow_j *
      tmp_0;
    MPC_B.Su[i + 70] = A_d_pow_j_0 * B_d_idx_3 + A_d_pow_j * B_d_idx_2_tmp;
  }

  A_d[0] = A_d_pow_i_0[0];
  A_d[1] = A_d_pow_i_0[1];
  A_d[2] = A_d_pow_i_0[2];
  A_d[3] = A_d_pow_i_0[3];
  for (i = 0; i < 2; i++) {
    A_d_pow_j = A_d[i];
    A_d_pow_j_0 = A_d[i + 2];
    A_d_pow_j_1 = (rtb_PrelookupRPM_o2 + 1.0F) * A_d_pow_j + A_d_pow_j_0 *
      rtb_Gain_idx_0_tmp;
    MPC_B.Su[i + 34] = A_d_pow_j_0 * B_d_idx_2_tmp + A_d_pow_j * B_d_idx_0;
    A_d_pow_j_2 = (rtb_Gain_idx_1_tmp + 1.0F) * A_d_pow_j_0 + A_d_pow_j * tmp_0;
    MPC_B.Su[i + 46] = A_d_pow_j_0 * B_d_idx_3 + A_d_pow_j * B_d_idx_2_tmp;
    MPC_B.Su[i + 10] = A_d_pow_j_2 * B_d_idx_2_tmp + A_d_pow_j_1 * B_d_idx_0;
    MPC_B.Su[i + 22] = A_d_pow_j_2 * B_d_idx_3 + A_d_pow_j_1 * B_d_idx_2_tmp;
  }

  for (i = 0; i < 6; i++) {
    A_d_tmp = (i + 1) << 1U;
    IdIq_ref[A_d_tmp - 2] = e_wm;
    IdIq_ref[A_d_tmp - 1] = rtb_PrelookupTe_o2;
  }

  memset(&MPC_B.T[0], 0, 144U * sizeof(int16_T));
  MPC_B.T[0] = 1;
  MPC_B.T[2] = -1;
  MPC_B.T[26] = 1;
  MPC_B.T[28] = -1;
  MPC_B.T[52] = 1;
  MPC_B.T[54] = -1;
  MPC_B.T[78] = 1;
  MPC_B.T[80] = -1;
  MPC_B.T[104] = 1;
  MPC_B.T[106] = -1;
  MPC_B.T[130] = 1;
  MPC_B.T[1] = 0;
  MPC_B.T[3] = 0;
  MPC_B.T[27] = 0;
  MPC_B.T[29] = 0;
  MPC_B.T[53] = 0;
  MPC_B.T[55] = 0;
  MPC_B.T[79] = 0;
  MPC_B.T[81] = 0;
  MPC_B.T[105] = 0;
  MPC_B.T[107] = 0;
  MPC_B.T[131] = 0;
  MPC_B.T[12] = 0;
  MPC_B.T[14] = 0;
  MPC_B.T[38] = 0;
  MPC_B.T[40] = 0;
  MPC_B.T[64] = 0;
  MPC_B.T[66] = 0;
  MPC_B.T[90] = 0;
  MPC_B.T[92] = 0;
  MPC_B.T[116] = 0;
  MPC_B.T[118] = 0;
  MPC_B.T[142] = 0;
  MPC_B.T[13] = 1;
  MPC_B.T[15] = -1;
  MPC_B.T[39] = 1;
  MPC_B.T[41] = -1;
  MPC_B.T[65] = 1;
  MPC_B.T[67] = -1;
  MPC_B.T[91] = 1;
  MPC_B.T[93] = -1;
  MPC_B.T[117] = 1;
  MPC_B.T[119] = -1;
  MPC_B.T[143] = 1;
  for (i = 0; i < 12; i++) {
    b[i] = 0.0F;
  }

  b[0] = MPC_DW.UnitDelay_DSTATE[0];
  b[1] = MPC_DW.UnitDelay_DSTATE[1];
  for (i = 0; i < 144; i++) {
    MPC_B.H_qp_tmp[i] = MPC_B.T[i];
  }

  for (i = 0; i < 12; i++) {
    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        rtb_DiscreteTimeIntegrator_idx_ += MPC_B.Su[12 * i + Su_tmp_2] * y[12 *
          idxAjj + Su_tmp_2];
      }

      MPC_B.Su_m[i + 12 * idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
    }

    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        rtb_DiscreteTimeIntegrator_idx_ += MPC_B.Su_m[12 * Su_tmp_2 + i] *
          MPC_B.Su[12 * idxAjj + Su_tmp_2];
      }

      MPC_B.Su_c[i + 12 * idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
    }
  }

  for (i = 0; i < 12; i++) {
    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        rtb_DiscreteTimeIntegrator_idx_ += MPC_B.H_qp_tmp[12 * i + Su_tmp_2] *
          b_y[12 * idxAjj + Su_tmp_2];
      }

      MPC_B.H_qp[i + 12 * idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
    }

    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        rtb_DiscreteTimeIntegrator_idx_ += MPC_B.H_qp[12 * Su_tmp_2 + i] *
          MPC_B.H_qp_tmp[12 * idxAjj + Su_tmp_2];
      }

      MPC_B.Su_m[i + 12 * idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
    }
  }

  for (i = 0; i < 144; i++) {
    MPC_B.H_qp[i] = 2.0F * MPC_B.Su_c[i] + 2.0F * MPC_B.Su_m[i];
  }

  for (i = 0; i < 12; i++) {
    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      Su_tmp_2 = 12 * i + idxAjj;
      MPC_B.H_qp_tmp[Su_tmp_2] = (MPC_B.H_qp[12 * idxAjj + i] +
        MPC_B.H_qp[Su_tmp_2]) / 2.0F + h[Su_tmp_2];
    }
  }

  i = 0;
  A_d_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_d_tmp < 12)) {
    idxAjj = A_d_tmp * 12 + A_d_tmp;
    rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
    if (A_d_tmp >= 1) {
      for (Su_tmp_2 = 0; Su_tmp_2 < A_d_tmp; Su_tmp_2++) {
        e_wm = MPC_B.H_qp_tmp[Su_tmp_2 * 12 + A_d_tmp];
        rtb_DiscreteTimeIntegrator_idx_ += e_wm * e_wm;
      }
    }

    rtb_DiscreteTimeIntegrator_idx_ = MPC_B.H_qp_tmp[idxAjj] -
      rtb_DiscreteTimeIntegrator_idx_;
    if (rtb_DiscreteTimeIntegrator_idx_ > 0.0F) {
      rtb_DiscreteTimeIntegrator_idx_ = (real32_T)sqrt
        (rtb_DiscreteTimeIntegrator_idx_);
      MPC_B.H_qp_tmp[idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
      if (A_d_tmp + 1 < 12) {
        if (A_d_tmp != 0) {
          T_tmp = ((A_d_tmp - 1) * 12 + A_d_tmp) + 2;
          for (T_tmp_0 = A_d_tmp + 2; T_tmp_0 <= T_tmp; T_tmp_0 += 12) {
            Su_tmp_2 = T_tmp_0 - A_d_tmp;
            e_wm = -MPC_B.H_qp_tmp[div_nde_s16_floor(Su_tmp_2 - 2, 12) * 12 +
              A_d_tmp];
            e = Su_tmp_2 + 10;
            for (ia = T_tmp_0; ia <= e; ia++) {
              Su_tmp_2 = ((idxAjj + ia) - T_tmp_0) + 1;
              MPC_B.H_qp_tmp[Su_tmp_2] += MPC_B.H_qp_tmp[ia - 1] * e_wm;
            }
          }
        }

        rtb_DiscreteTimeIntegrator_idx_ = 1.0F / rtb_DiscreteTimeIntegrator_idx_;
        Su_tmp_2 = (idxAjj - A_d_tmp) + 12;
        for (T_tmp = idxAjj + 2; T_tmp <= Su_tmp_2; T_tmp++) {
          MPC_B.H_qp_tmp[T_tmp - 1] *= rtb_DiscreteTimeIntegrator_idx_;
        }
      }

      A_d_tmp++;
    } else {
      MPC_B.H_qp_tmp[idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
      i = A_d_tmp + 1;
      exitg1 = true;
    }
  }

  if (i == 0) {
    i = 13;
  }

  i--;
  for (A_d_tmp = 2; A_d_tmp <= i; A_d_tmp++) {
    for (idxAjj = 0; idxAjj <= A_d_tmp - 2; idxAjj++) {
      MPC_B.H_qp_tmp[idxAjj + 12 * (A_d_tmp - 1)] = 0.0F;
    }
  }

  /* End of Outputs for SubSystem: '<S1>/Current controller' */
  /* End of Outputs for SubSystem: '<Root>/MPC' */
  for (i = 0; i < 144; i++) {
    /* Outputs for Atomic SubSystem: '<Root>/MPC' */
    /* Outputs for Atomic SubSystem: '<S1>/Current controller' */
    /* MATLAB Function: '<S2>/MPC' */
    MPC_B.H_qp[i] = B[i];

    /* End of Outputs for SubSystem: '<S1>/Current controller' */
    /* End of Outputs for SubSystem: '<Root>/MPC' */
  }

  /* Outputs for Atomic SubSystem: '<Root>/MPC' */
  /* Outputs for Atomic SubSystem: '<S1>/Current controller' */
  /* MATLAB Function: '<S2>/MPC' incorporates:
   *  Inport: '<Root>/Id_meas'
   *  Inport: '<Root>/Iq_meas'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  MPC_trisolve(MPC_B.H_qp_tmp, MPC_B.H_qp);
  for (i = 0; i < 12; i++) {
    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        rtb_DiscreteTimeIntegrator_idx_ += MPC_B.Su[12 * i + Su_tmp_2] * y[12 *
          idxAjj + Su_tmp_2];
      }

      MPC_B.Su_m[i + 12 * idxAjj] = rtb_DiscreteTimeIntegrator_idx_;
    }

    a[i] = ((MPC_B.a[i + 12] * MPC_U.Iq_meas + MPC_B.a[i] * MPC_U.Id_meas) +
            Sh[i]) - IdIq_ref[i];
  }

  for (i = 0; i < 12; i++) {
    Sh[i] = 0.0F;
    T[i] = 0.0F;
    for (idxAjj = 0; idxAjj < 12; idxAjj++) {
      A_d_tmp = 12 * idxAjj + i;
      Sh[i] += MPC_B.Su_m[A_d_tmp] * a[idxAjj];
      rtb_DiscreteTimeIntegrator_idx_ = 0.0F;
      e_wm = 0.0F;
      for (Su_tmp_2 = 0; Su_tmp_2 < 12; Su_tmp_2++) {
        T_tmp = 12 * i + Su_tmp_2;
        T_tmp_0 = 12 * idxAjj + Su_tmp_2;
        rtb_DiscreteTimeIntegrator_idx_ += (real32_T)MPC_B.T[T_tmp] *
          b_y[T_tmp_0];
        e_wm += MPC_B.H_qp[T_tmp] * MPC_B.H_qp[T_tmp_0];
      }

      MPC_B.Su[A_d_tmp] = e_wm;
      T[i] += rtb_DiscreteTimeIntegrator_idx_ * b[idxAjj];
    }

    IdIq_ref[i] = 2.0F * Sh[i] - 2.0F * T[i];
  }

  MPC_qpkwik(MPC_B.H_qp, MPC_B.Su, IdIq_ref, l, MPC_DW.iA_prev, Sh, MPC_B.a,
             &exitflag);
  if (exitflag != 1L) {
    for (i = 0; i < 12; i++) {
      Sh[i] = 0.0F;
    }
  }

  MPC_DW.UnitDelay_DSTATE[0] = Sh[0];
  MPC_DW.UnitDelay_DSTATE[1] = Sh[1];

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

  {
    int16_T i;

    /* SystemInitialize for Atomic SubSystem: '<Root>/MPC' */
    /* SystemInitialize for Atomic SubSystem: '<S1>/Subsystem' */
    /* InitializeConditions for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
    MPC_DW.DiscreteTimeIntegrator_DSTATE[0] = MPC_P.DiscreteTimeIntegrator_IC[0];

    /* End of SystemInitialize for SubSystem: '<S1>/Subsystem' */

    /* SystemInitialize for Atomic SubSystem: '<S1>/Current controller' */
    /* InitializeConditions for UnitDelay: '<S2>/Unit Delay' */
    MPC_DW.UnitDelay_DSTATE[0] = MPC_P.UnitDelay_InitialCondition;

    /* End of SystemInitialize for SubSystem: '<S1>/Current controller' */

    /* SystemInitialize for Atomic SubSystem: '<S1>/Subsystem' */
    /* InitializeConditions for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
    MPC_DW.DiscreteTimeIntegrator_DSTATE[1] = MPC_P.DiscreteTimeIntegrator_IC[1];

    /* End of SystemInitialize for SubSystem: '<S1>/Subsystem' */

    /* SystemInitialize for Atomic SubSystem: '<S1>/Current controller' */
    /* InitializeConditions for UnitDelay: '<S2>/Unit Delay' */
    MPC_DW.UnitDelay_DSTATE[1] = MPC_P.UnitDelay_InitialCondition;

    /* SystemInitialize for MATLAB Function: '<S2>/MPC' */
    for (i = 0; i < 24; i++) {
      MPC_DW.iA_prev[i] = false;
    }

    /* End of SystemInitialize for MATLAB Function: '<S2>/MPC' */
    /* End of SystemInitialize for SubSystem: '<S1>/Current controller' */
    /* End of SystemInitialize for SubSystem: '<Root>/MPC' */
  }
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
