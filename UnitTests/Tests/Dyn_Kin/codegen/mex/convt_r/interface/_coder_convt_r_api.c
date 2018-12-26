/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_convt_r_api.c
 *
 * Code generation for function '_coder_convt_r_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "convt_r.h"
#include "_coder_convt_r_api.h"
#include "convt_r_data.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[42128];
static const mxArray *b_emlrt_marshallOut(const real_T u[7899]);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[42128];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Y, const
  char_T *identifier))[42128];
static const mxArray *emlrt_marshallOut(const real_T u[10532]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[42128]
{
  real_T (*y)[42128];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *b_emlrt_marshallOut(const real_T u[7899])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv2[2] = { 0, 0 };

  static const int32_T iv3[2] = { 2633, 3 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[42128]
{
  real_T (*ret)[42128];
  static const int32_T dims[2] = { 2633, 16 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[42128])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Y, const
  char_T *identifier))[42128]
{
  real_T (*y)[42128];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(Y), &thisId);
  emlrtDestroyArray(&Y);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[10532])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 2633, 4 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, iv1, 2);
  emlrtAssign(&y, m0);
  return y;
}

void convt_r_api(const mxArray * const prhs[1], const mxArray *plhs[2])
{
  real_T (*r_A)[10532];
  real_T (*r_B)[7899];
  real_T (*Y)[42128];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  r_A = (real_T (*)[10532])mxMalloc(sizeof(real_T [10532]));
  r_B = (real_T (*)[7899])mxMalloc(sizeof(real_T [7899]));

  /* Marshall function inputs */
  Y = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Y");

  /* Invoke the target function */
  convt_r(&st, *Y, *r_A, *r_B);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*r_A);
  plhs[1] = b_emlrt_marshallOut(*r_B);
}

/* End of code generation (_coder_convt_r_api.c) */
