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
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "LEDs_Driver.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void SLUGS2_LEDs_DriverTID3(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S459>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S459>/Digital Output Read/Output */

  /* Logic: '<S5>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S459>/Digital Output Read'
   */
  SLUGS2_B.LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S459>/Digital Output Write' */
  LATBbits.LATB3 = SLUGS2_B.LogicalOperator1;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void SLUGS2_LEDs_DriverTID4(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S458>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S458>/Digital Output Read/Output */

  /* Logic: '<S5>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S458>/Digital Output Read'
   */
  SLUGS2_B.LogicalOperator = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S458>/Digital Output Write' */
  LATBbits.LATB2 = SLUGS2_B.LogicalOperator;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void SLUGS2_LEDs_DriverTID7(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S460>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S460>/Digital Output Read/Output */

  /* Logic: '<S5>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S460>/Digital Output Read'
   */
  SLUGS2_B.LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S460>/Digital Output Write' */
  LATBbits.LATB4 = SLUGS2_B.LogicalOperator2;
#endif
}

/* Outputs for atomic system: '<Root>/LEDs_Driver' */
void SLUGS2_LEDs_DriverTID9(void)
{
#ifndef WIN

  /* S-Function (MCHP_Digital_Output_Read): '<S461>/Digital Output Read' */
  /* MCHP_Digital_Output_Read Block: <S461>/Digital Output Read/Output */

  /* Logic: '<S5>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S461>/Digital Output Read'
   */
  SLUGS2_B.LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S461>/Digital Output Write' */
  LATBbits.LATB5 = SLUGS2_B.LogicalOperator3;
#endif
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
