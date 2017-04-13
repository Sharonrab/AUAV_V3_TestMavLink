/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_86.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_86.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Output and update for atomic system:
 *    '<S545>/myMux Fun1'
 *    '<S545>/myMux Fun2'
 *    '<S573>/myMux Fun'
 */
void SLUGS2_myMuxFun1_e(real32_T rtu_u1, real32_T rtu_u2, real32_T rtu_u3,
  rtB_myMuxFun1_SLUGS2_d_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/myMux Fun1': '<S554>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S554>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
