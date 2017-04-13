/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_77.h
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
 */

#ifndef RTW_HEADER_rt_sys_AUAV_V3_TestSensors_77_h_
#define RTW_HEADER_rt_sys_AUAV_V3_TestSensors_77_h_
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S478>/Embedded MATLAB Function' */
typedef struct {
  real32_T y;                          /* '<S478>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_d_T;

extern void A_EmbeddedMATLABFunction_l(real32_T rtu_u,
  rtB_EmbeddedMATLABFunction_d_T *localB);

#endif                                 /* RTW_HEADER_rt_sys_AUAV_V3_TestSensors_77_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
