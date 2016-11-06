/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_86.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.241
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Nov 05 08:28:55 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_86.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S552>/myMux Fun1'
 *    '<S552>/myMux Fun2'
 *    '<S581>/myMux Fun'
 */
void AUAV_V3_TestSe_myMuxFun1_e(real32_T rtu_u1, real32_T rtu_u2, real32_T
  rtu_u3, rtB_myMuxFun1_AUAV_V3_TestS_d_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/myMux Fun1': '<S561>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S561>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
