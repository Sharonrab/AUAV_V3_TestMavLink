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
 * File: AUAV3_AND_SLUGS_SENSOR_data.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV3_AND_SLUGS_SENSOR.
 *
 * Model version                        : 1.208
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Wed Apr 27 00:58:09 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Wed Apr 27 00:58:11 2016
 */

#include "AUAV3_AND_SLUGS_SENSOR.h"
#include "AUAV3_AND_SLUGS_SENSOR_private.h"

/* Constant parameters (auto storage) */
const ConstParam_AUAV3_AND_SLUGS_SE_T AUAV3_AND_SLUGS_SENSOR_ConstP = {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S155>/Constant'
   *   '<S155>/Constant1'
   *   '<S155>/Constant2'
   *   '<S155>/Constant3'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Computed Parameter: UEN2NEU_Gain
   * Referenced by: '<S27>/UEN 2 NEU'
   */
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F }
};

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
