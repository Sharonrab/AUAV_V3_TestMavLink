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
 * File: rt_sys_AUAV_V3_TestSensors_2.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_2.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for enable system:
 *    '<S28>/Enabled Subsystem'
 *    '<S615>/Enabled Subsystem'
 */
void AUAV_V3_T_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_AUAV_V3__T *localB)
{
  /* Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S38>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S38>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S28>/Enabled Subsystem' */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
