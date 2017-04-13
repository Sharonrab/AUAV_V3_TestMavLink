/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_86.h
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#ifndef RTW_HEADER_rt_sys_SLUGS2_86_h_
#define RTW_HEADER_rt_sys_SLUGS2_86_h_
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S545>/myMux Fun1' */
typedef struct {
  real32_T y[3];                       /* '<S545>/myMux Fun1' */
} rtB_myMuxFun1_SLUGS2_d_T;

extern void SLUGS2_myMuxFun1_e(real32_T rtu_u1, real32_T rtu_u2, real32_T rtu_u3,
  rtB_myMuxFun1_SLUGS2_d_T *localB);

#endif                                 /* RTW_HEADER_rt_sys_SLUGS2_86_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
