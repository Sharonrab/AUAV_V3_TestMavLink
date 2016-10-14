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
 * File: rt_sys_AUAV_V3_TestSensors_77.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_77.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S478>/Embedded MATLAB Function'
 *    '<S21>/Embedded MATLAB Function1'
 */
void A_EmbeddedMATLABFunction_l(real32_T rtu_u, rtB_EmbeddedMATLABFunction_d_T
  *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S491>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_u < 0.0F) {
    /* '<S491>:1:5' */
    /* '<S491>:1:6' */
    localB->y = rtu_u + 6.28318548F;
  } else {
    /* '<S491>:1:8' */
    localB->y = rtu_u;
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
