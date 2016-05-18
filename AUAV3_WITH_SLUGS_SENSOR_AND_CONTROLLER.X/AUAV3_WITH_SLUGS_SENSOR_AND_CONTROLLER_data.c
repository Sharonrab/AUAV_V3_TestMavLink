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
 * File: AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_data.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.
 *
 * Model version                        : 1.275
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Tue May 17 18:16:41 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Tue May 17 18:16:42 2016
 */

#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h"

/* Constant parameters (auto storage) */
const ConstParam_AUAV3_WITH_SLUGS_S_T AUAV3_WITH_SLUGS_SENSOR__ConstP = {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S566>/Constant'
   *   '<S566>/Constant1'
   *   '<S566>/Constant2'
   *   '<S566>/Constant3'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S439>/UEN 2 NEU'
   *   '<S145>/UEN 2 NEU'
   *   '<S167>/UEN 2 NEU'
   *   '<S409>/UEN 2 NEU'
   *   '<S311>/UEN 2 NEU'
   *   '<S328>/UEN 2 NEU'
   *   '<S366>/UEN 2 NEU'
   *   '<S383>/UEN 2 NEU'
   */
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S4>/Switch'
   *   '<S7>/Constant1'
   *   '<S34>/Switch3'
   *   '<S631>/dT'
   *   '<S35>/Switch3'
   *   '<S42>/Switch3'
   *   '<S43>/Switch3'
   *   '<S45>/Switch1'
   *   '<S65>/Switch1'
   *   '<S143>/Integer Delay'
   *   '<S143>/Integer Delay1'
   *   '<S566>/Switch2'
   *   '<S61>/Switch3'
   *   '<S62>/Switch3'
   *   '<S68>/Switch3'
   *   '<S69>/Switch3'
   *   '<S70>/On//Off'
   *   '<S72>/On//Off'
   *   '<S105>/On//Off'
   *   '<S106>/On//Off'
   *   '<S107>/On//Off'
   *   '<S108>/Schedule LPF'
   *   '<S109>/Switch3'
   *   '<S110>/Switch3'
   *   '<S111>/Switch3'
   *   '<S112>/Switch3'
   *   '<S144>/RTB1'
   *   '<S148>/RTB'
   *   '<S148>/RTB1'
   *   '<S150>/IC'
   *   '<S150>/Switch'
   *   '<S150>/Switch1'
   *   '<S151>/FromWP'
   *   '<S151>/ToWP'
   *   '<S151>/RTB'
   *   '<S151>/RTB1'
   *   '<S152>/IC2'
   *   '<S76>/Switch1'
   *   '<S81>/Switch3'
   *   '<S92>/Switch1'
   *   '<S98>/Switch3'
   *   '<S99>/Switch3'
   *   '<S117>/Switch1'
   *   '<S121>/Switch1'
   *   '<S125>/Switch1'
   *   '<S131>/Switch3'
   *   '<S141>/FixPt Constant'
   *   '<S164>/RTB1'
   *   '<S232>/Switch'
   *   '<S232>/Switch2'
   *   '<S237>/Switch1'
   *   '<S77>/Switch3'
   *   '<S78>/Switch3'
   *   '<S93>/Switch3'
   *   '<S94>/Switch3'
   *   '<S118>/Switch3'
   *   '<S119>/Switch3'
   *   '<S122>/Switch3'
   *   '<S123>/Switch3'
   *   '<S126>/Switch3'
   *   '<S127>/Switch3'
   *   '<S278>/Switch3'
   */
  0U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S5>/COMP_ID'
   *   '<S564>/Constant'
   *   '<S631>/dA'
   *   '<S657>/Switch'
   *   '<S657>/Switch1'
   *   '<S657>/Switch2'
   *   '<S657>/Switch3'
   *   '<S144>/FromWP'
   *   '<S144>/ToWP'
   *   '<S148>/FromWP'
   *   '<S150>/FromWP'
   *   '<S150>/Integer Delay'
   *   '<S141>/FixPt Unit Delay2'
   *   '<S162>/RTB1'
   *   '<S344>/Constant'
   */
  1U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S7>/Constant7'
   *   '<S631>/dR'
   *   '<S643>/Index'
   *   '<S148>/ToWP'
   *   '<S150>/ToWP'
   *   '<S163>/RTB1'
   *   '<S344>/Constant1'
   */
  2U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S7>/Constant10'
   *   '<S631>/dE'
   */
  3U
};

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
