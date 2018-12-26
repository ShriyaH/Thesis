/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * convt_r.c
 *
 * Code generation for function 'convt_r'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "convt_r.h"
#include "Q2DCM.h"
#include "cross_quat.h"
#include "conj_quat.h"
#include "convt_r_data.h"

/* Function Definitions */
void convt_r(const emlrtStack *sp, const real_T Y[42128], real_T r_A[10532],
             real_T r_B[7899])
{
  real_T C[23697];
  int32_T i;
  real_T b_Y[4];
  int32_T i0;
  real_T c_Y[4];
  real_T dv0[4];
  real_T dv1[4];
  real_T a[9];
  int32_T i1;
  int32_T i2;
  real_T b_a[3];
  memset(&r_A[0], 0, 10532U * sizeof(real_T));
  memset(&C[0], 0, 23697U * sizeof(real_T));
  i = 0;
  while (i < 2633) {
    for (i0 = 0; i0 < 4; i0++) {
      b_Y[i0] = Y[i + 2633 * i0];
    }

    for (i0 = 0; i0 < 4; i0++) {
      c_Y[i0] = Y[i + 2633 * (4 + i0)];
    }

    conj_quat(b_Y, dv0);
    cross_quat(dv0, c_Y, dv1);
    for (i0 = 0; i0 < 4; i0++) {
      r_A[i + 2633 * i0] = 2.0 * dv1[i0];
      b_Y[i0] = Y[i + 2633 * i0];
    }

    Q2DCM(b_Y, a);
    i0 = 3 * (1 + i) - 3;
    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        C[(i2 + i0) + 7899 * i1] = a[i2 + 3 * i1];
      }
    }

    i0 = 3 * (1 + i) - 3;
    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        a[i2 + 3 * i1] = C[(i2 + i0) + 7899 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_a[i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_a[i0] += a[i0 + 3 * i1] * r_A[i + 2633 * i1];
      }

      r_B[i + 2633 * i0] = b_a[i0];
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }
}

/* End of code generation (convt_r.c) */
