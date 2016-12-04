/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: LEDs_Driver.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.262
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Nov 23 10:01:40 2016
 */

#include "LEDs_Driver.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void AUAV_V3_TestSe_LEDs_DriverTID3(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S466>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S466>/Digital Output Read/Output */

  /* Logic: '<S7>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S466>/Digital Output Read'
   */
  AUAV_V3_TestSensors_B.LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S466>/Digital Output Write' */
  LATBbits.LATB3 = AUAV_V3_TestSensors_B.LogicalOperator1;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void AUAV_V3_TestSe_LEDs_DriverTID4(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S465>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S465>/Digital Output Read/Output */

  /* Logic: '<S7>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S465>/Digital Output Read'
   */
  AUAV_V3_TestSensors_B.LogicalOperator = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S465>/Digital Output Write' */
  LATBbits.LATB2 = AUAV_V3_TestSensors_B.LogicalOperator;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void AUAV_V3_TestSe_LEDs_DriverTID7(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S467>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S467>/Digital Output Read/Output */

  /* Logic: '<S7>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S467>/Digital Output Read'
   */
  AUAV_V3_TestSensors_B.LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S467>/Digital Output Write' */
  LATBbits.LATB4 = AUAV_V3_TestSensors_B.LogicalOperator2;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void AUAV_V3_TestSe_LEDs_DriverTID9(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S468>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S468>/Digital Output Read/Output */

  /* Logic: '<S7>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S468>/Digital Output Read'
   */
  AUAV_V3_TestSensors_B.LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S468>/Digital Output Write' */
  LATBbits.LATB5 = AUAV_V3_TestSensors_B.LogicalOperator3;
#endif
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
