/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_18.h
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.262
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Nov 23 10:01:40 2016
 */

#ifndef RTW_HEADER_rt_sys_AUAV_V3_TestSensors_18_h_
#define RTW_HEADER_rt_sys_AUAV_V3_TestSensors_18_h_
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S104>/Embedded MATLAB Function' */
typedef struct {
  real32_T y;                          /* '<S104>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_AU_T;

/* Block states (auto storage) for system '<S104>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S104>/Embedded MATLAB Function' */
  real_T b;                            /* '<S104>/Embedded MATLAB Function' */
  real32_T y_km1;                      /* '<S104>/Embedded MATLAB Function' */
  real32_T u_km1;                      /* '<S104>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S104>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_A_T;

extern void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW);
extern void AUA_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T
  rtu_f, rtB_EmbeddedMATLABFunction_AU_T *localB,
  rtDW_EmbeddedMATLABFunction_A_T *localDW);

#endif                                 /* RTW_HEADER_rt_sys_AUAV_V3_TestSensors_18_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
