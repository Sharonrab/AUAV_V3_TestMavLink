/*--------------------------------------------------------------
 *   MPLAB Blockset v3.35 for Microchip dsPIC chip family.     *
 *   Generate .c and .h files from Simulink model              *
 *   and compile to .elf, .hex and .cof file that can be       *
 *   flashed into the microcontroller                          *
 *                                                             *
 *      The Microchip name PIC, dsPIC, and MPLAB are           *
 *      registered trademarks of Microchip Technology Inc.     *
 *      MATLAB, Simulink, and Real-Time Workshop are           *
 *      registered trademarks of The MathWorks, Inc.           *
 *                                                             *
 *  Blockset authors: L.Kerhuel, U.Kumar                       *
 *  Product Page:  http://www.microchip.com/SimulinkBlocks     *
 *          Forum: http://www.microchip.com/forums/f192.aspx   *
 *          Wiki:  http://microchip.wikidot.com/simulink:start *
 *--------------------------------------------------------------
 *
 * File: rt_sys_AUAV_V3_TestSensors_86.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_86.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S552>/myMux Fun1'
 *    '<S552>/myMux Fun2'
 *    '<S580>/myMux Fun'
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

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
