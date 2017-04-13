/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_18.h
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#ifndef RTW_HEADER_rt_sys_SLUGS2_18_h_
#define RTW_HEADER_rt_sys_SLUGS2_18_h_
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S97>/Embedded MATLAB Function' */
typedef struct {
  real32_T y;                          /* '<S97>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_SL_T;

/* Block states (auto storage) for system '<S97>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S97>/Embedded MATLAB Function' */
  real_T b;                            /* '<S97>/Embedded MATLAB Function' */
  real32_T y_km1;                      /* '<S97>/Embedded MATLAB Function' */
  real32_T u_km1;                      /* '<S97>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S97>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_S_T;

extern void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_S_T *localDW);
extern void SLU_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T
  rtu_f, rtB_EmbeddedMATLABFunction_SL_T *localB,
  rtDW_EmbeddedMATLABFunction_S_T *localDW);

#endif                                 /* RTW_HEADER_rt_sys_SLUGS2_18_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
