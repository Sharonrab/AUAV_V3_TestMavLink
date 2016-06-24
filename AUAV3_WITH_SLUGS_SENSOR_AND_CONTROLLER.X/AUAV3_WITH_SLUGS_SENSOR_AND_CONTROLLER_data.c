/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_data.c
 *
 * Code generated for Simulink model 'AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER'.
 *
 * Model version                  : 1.289
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Jun 23 23:55:56 2016
 */

#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h"

/* Constant parameters (auto storage) */
const ConstParam_AUAV3_WITH_SLUGS_S_T AUAV3_WITH_SLUGS_SENSOR__ConstP = {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S568>/Constant'
   *   '<S568>/Constant1'
   *   '<S568>/Constant2'
   *   '<S568>/Constant3'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S441>/UEN 2 NEU'
   *   '<S144>/UEN 2 NEU'
   *   '<S166>/UEN 2 NEU'
   *   '<S408>/UEN 2 NEU'
   *   '<S310>/UEN 2 NEU'
   *   '<S327>/UEN 2 NEU'
   *   '<S365>/UEN 2 NEU'
   *   '<S382>/UEN 2 NEU'
   */
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S35>/Switch3'
   *   '<S633>/dT'
   *   '<S36>/Switch3'
   *   '<S43>/Switch3'
   *   '<S44>/Switch3'
   *   '<S46>/Switch1'
   *   '<S142>/Integer Delay'
   *   '<S142>/Integer Delay1'
   *   '<S568>/Switch2'
   *   '<S62>/Switch3'
   *   '<S63>/Switch3'
   *   '<S68>/Switch3'
   *   '<S69>/Switch3'
   *   '<S70>/On//Off'
   *   '<S72>/On//Off'
   *   '<S104>/On//Off'
   *   '<S105>/On//Off'
   *   '<S106>/On//Off'
   *   '<S107>/Schedule LPF'
   *   '<S108>/Switch3'
   *   '<S109>/Switch3'
   *   '<S110>/Switch3'
   *   '<S111>/Switch3'
   *   '<S143>/RTB1'
   *   '<S147>/RTB'
   *   '<S147>/RTB1'
   *   '<S149>/IC'
   *   '<S149>/Switch'
   *   '<S149>/Switch1'
   *   '<S150>/FromWP'
   *   '<S150>/ToWP'
   *   '<S150>/RTB'
   *   '<S150>/RTB1'
   *   '<S151>/IC2'
   *   '<S76>/Switch1'
   *   '<S81>/Switch3'
   *   '<S92>/Switch1'
   *   '<S98>/Switch3'
   *   '<S99>/Switch3'
   *   '<S116>/Switch1'
   *   '<S120>/Switch1'
   *   '<S124>/Switch1'
   *   '<S130>/Switch3'
   *   '<S140>/FixPt Constant'
   *   '<S163>/RTB1'
   *   '<S231>/Switch'
   *   '<S231>/Switch2'
   *   '<S236>/Switch1'
   *   '<S77>/Switch3'
   *   '<S78>/Switch3'
   *   '<S93>/Switch3'
   *   '<S94>/Switch3'
   *   '<S117>/Switch3'
   *   '<S118>/Switch3'
   *   '<S121>/Switch3'
   *   '<S122>/Switch3'
   *   '<S125>/Switch3'
   *   '<S126>/Switch3'
   *   '<S277>/Switch3'
   */
  0U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S5>/COMP_ID'
   *   '<S566>/Constant'
   *   '<S633>/dA'
   *   '<S659>/Switch'
   *   '<S659>/Switch1'
   *   '<S659>/Switch2'
   *   '<S659>/Switch3'
   *   '<S143>/FromWP'
   *   '<S143>/ToWP'
   *   '<S147>/FromWP'
   *   '<S149>/FromWP'
   *   '<S149>/Integer Delay'
   *   '<S140>/FixPt Unit Delay2'
   *   '<S161>/RTB1'
   *   '<S343>/Constant'
   */
  1U,

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S633>/dR'
   *   '<S645>/Index'
   *   '<S147>/ToWP'
   *   '<S149>/ToWP'
   *   '<S162>/RTB1'
   *   '<S343>/Constant1'
   */
  2U,

  /* Computed Parameter: dE_Value
   * Referenced by: '<S633>/dE'
   */
  3U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
