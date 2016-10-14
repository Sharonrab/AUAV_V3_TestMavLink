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
 * File: rt_sys_AUAV_V3_TestSensors_18.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_18.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Initial conditions for atomic system:
 *    '<S104>/Embedded MATLAB Function'
 *    '<S141>/Embedded MATLAB Function'
 *    '<S165>/Embedded MATLAB Function'
 *    '<S564>/Embedded MATLAB Function'
 *    '<S658>/Embedded MATLAB Function'
 *    '<S659>/Embedded MATLAB Function'
 */
void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S104>/Embedded MATLAB Function'
 *    '<S141>/Embedded MATLAB Function'
 *    '<S165>/Embedded MATLAB Function'
 *    '<S564>/Embedded MATLAB Function'
 *    '<S658>/Embedded MATLAB Function'
 *    '<S659>/Embedded MATLAB Function'
 */
void AUA_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_AU_T *localB, rtDW_EmbeddedMATLABFunction_A_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function': '<S112>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S112>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S112>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S112>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S112>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S112>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S112>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S112>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * (real32_T)localDW->a + (real32_T)
    localDW->b * localDW->y_km1;

  /* '<S112>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S112>:1:21' */
  localDW->u_km1 = rtu_u;
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
