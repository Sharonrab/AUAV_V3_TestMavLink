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
 * File: AUAV_V3_TestSensors_data.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Constant parameters (auto storage) */
const ConstParam_AUAV_V3_TestSensor_T AUAV_V3_TestSensors_ConstP = {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S584>/Constant1'
   *   '<S584>/Constant2'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S483>/UEN 2 NEU'
   *   '<S182>/UEN 2 NEU'
   *   '<S204>/UEN 2 NEU'
   *   '<S446>/UEN 2 NEU'
   *   '<S348>/UEN 2 NEU'
   *   '<S365>/UEN 2 NEU'
   *   '<S403>/UEN 2 NEU'
   *   '<S420>/UEN 2 NEU'
   */
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S73>/Switch3'
   *   '<S652>/dT'
   *   '<S74>/Switch3'
   *   '<S81>/Switch3'
   *   '<S82>/Switch3'
   *   '<S84>/Switch1'
   *   '<S180>/Integer Delay'
   *   '<S180>/Integer Delay1'
   *   '<S100>/Switch3'
   *   '<S101>/Switch3'
   *   '<S106>/Switch3'
   *   '<S107>/Switch3'
   *   '<S108>/On//Off'
   *   '<S110>/On//Off'
   *   '<S142>/On//Off'
   *   '<S143>/On//Off'
   *   '<S144>/On//Off'
   *   '<S145>/Schedule LPF'
   *   '<S146>/Switch3'
   *   '<S147>/Switch3'
   *   '<S148>/Switch3'
   *   '<S149>/Switch3'
   *   '<S181>/RTB1'
   *   '<S185>/RTB'
   *   '<S185>/RTB1'
   *   '<S187>/IC'
   *   '<S187>/Switch'
   *   '<S187>/Switch1'
   *   '<S188>/FromWP'
   *   '<S188>/ToWP'
   *   '<S188>/RTB'
   *   '<S188>/RTB1'
   *   '<S189>/IC2'
   *   '<S114>/Switch1'
   *   '<S119>/Switch3'
   *   '<S130>/Switch1'
   *   '<S136>/Switch3'
   *   '<S137>/Switch3'
   *   '<S154>/Switch1'
   *   '<S158>/Switch1'
   *   '<S162>/Switch1'
   *   '<S168>/Switch3'
   *   '<S178>/FixPt Constant'
   *   '<S201>/RTB1'
   *   '<S269>/Switch'
   *   '<S269>/Switch2'
   *   '<S274>/Switch1'
   *   '<S115>/Switch3'
   *   '<S116>/Switch3'
   *   '<S131>/Switch3'
   *   '<S132>/Switch3'
   *   '<S155>/Switch3'
   *   '<S156>/Switch3'
   *   '<S159>/Switch3'
   *   '<S160>/Switch3'
   *   '<S163>/Switch3'
   *   '<S164>/Switch3'
   *   '<S315>/Switch3'
   */
  0U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<Root>/HIL Manual Switch'
   *   '<S8>/COMP_ID'
   *   '<S652>/dA'
   *   '<S676>/COMP_ID'
   *   '<S679>/Switch'
   *   '<S679>/Switch1'
   *   '<S679>/Switch2'
   *   '<S679>/Switch3'
   *   '<S181>/FromWP'
   *   '<S181>/ToWP'
   *   '<S185>/FromWP'
   *   '<S187>/FromWP'
   *   '<S187>/Integer Delay'
   *   '<S178>/FixPt Unit Delay2'
   *   '<S199>/RTB1'
   *   '<S381>/Constant'
   */
  1U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S652>/dR'
   *   '<S664>/Index'
   *   '<S185>/ToWP'
   *   '<S187>/ToWP'
   *   '<S200>/RTB1'
   *   '<S381>/Constant1'
   */
  2U,

  /* Computed Parameter: dE_Value
   * Referenced by: '<S652>/dE'
   */
  3U
};

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
