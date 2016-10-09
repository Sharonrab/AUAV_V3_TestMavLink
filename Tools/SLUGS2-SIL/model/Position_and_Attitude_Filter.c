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
 * File: Position_and_Attitude_Filter.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#include "Position_and_Attitude_Filter.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S485>/myMux Fun1'
 *    '<S554>/myMux Fun1'
 */
void AUAV_V3_TestSens_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun1_AUAV_V3_TestSen_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1': '<S501>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S501>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S513>/Embedded MATLAB Function'
 *    '<S518>/Embedded MATLAB Function'
 */
void A_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_g_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function': '<S515>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S515>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Initial conditions for atomic system:
 *    '<S560>/Embedded MATLAB Function1'
 *    '<S560>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunct_c_Init(rtDW_EmbeddedMATLABFunction1__T *localDW)
{
  localDW->LastU_not_empty = false;
  localDW->LastU = 0.0F;
  localDW->rate = 0.0F;
  localDW->OldRate = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S560>/Embedded MATLAB Function1'
 *    '<S560>/Embedded MATLAB Function2'
 */
void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1': '<S576>:1' */
  if (!localDW->LastU_not_empty) {
    /* '<S576>:1:7' */
    localDW->LastU_not_empty = true;

    /* '<S576>:1:9' */
    localDW->TimeSinceLast = 0.01F;
  }

  if (rtu_NewGPS != 0) {
    /* '<S576>:1:15' */
    localDW->OldRate = localDW->rate;

    /* '<S576>:1:16' */
    localDW->rate = (rtu_u - localDW->LastU) / localDW->TimeSinceLast;

    /* '<S576>:1:17' */
    localDW->TimeSinceLast = 0.0F;

    /* '<S576>:1:18' */
    localDW->LastU = rtu_u;
  }

  /* '<S576>:1:20' */
  localB->y = (0.5F * localDW->rate + 0.5F * localDW->OldRate) *
    localDW->TimeSinceLast + rtu_u;

  /* '<S576>:1:21' */
  localDW->TimeSinceLast += 0.01F;
}

/* Initial conditions for atomic system: '<Root>/Position_and_Attitude_Filter' */
void Position_and_Attitude_Init(void)
{
  int16_T i;

  /* InitializeConditions for DiscreteIntegrator: '<S478>/Discrete-Time Integrator1' */
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 0.0F;

  /* InitializeConditions for MATLAB Function: '<S553>/Embedded MATLAB Function3' */
  AUAV_V3_TestSensors_DWork.lastGps_h_not_empty = false;
  AUAV_V3_TestSensors_DWork.lastGps_h = 0.0F;

  /* InitializeConditions for MATLAB Function: '<S564>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ph);
  for (i = 0; i < 15; i++) {
    /* InitializeConditions for Delay: '<S489>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[i] = 19.5F;

    /* InitializeConditions for Delay: '<S559>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[i] = 19.5F;
  }

  /* InitializeConditions for RateLimiter: '<S552>/Rate Limiter' */
  AUAV_V3_TestSensors_DWork.PrevY[0] = 0.0F;
  AUAV_V3_TestSensors_DWork.PrevY[1] = 0.0F;
  AUAV_V3_TestSensors_DWork.PrevY[2] = 0.0F;

  /* InitializeConditions for RateLimiter: '<S478>/Bias Rate Limiter' */
  AUAV_V3_TestSensors_DWork.PrevY_n[0] = 0.0F;
  AUAV_V3_TestSensors_DWork.PrevY_n[1] = 0.0F;
  AUAV_V3_TestSensors_DWork.PrevY_n[2] = 0.0F;
}

/* Start for atomic system: '<Root>/Position_and_Attitude_Filter' */
void Position_and_Attitud_Start(void)
{
  /* InitializeConditions for Enabled SubSystem: '<S552>/Subsystem' */

  /* InitializeConditions for MATLAB Function: '<S560>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_c_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_k);

  /* InitializeConditions for MATLAB Function: '<S560>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_c_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2);

  /* End of InitializeConditions for SubSystem: '<S552>/Subsystem' */
}

/* Output and update for atomic system: '<Root>/Position_and_Attitude_Filter' */
void Position_and_Attitude_Filt(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion2_g[3];
  real_T rtb_DiscreteZeroPole_f;
  real_T rtb_DiscreteZeroPole_c;
  real_T rtb_DiscreteZeroPole_ck;
  real_T rtb_DataTypeConversion2_e2[3];
  real_T rtb_DiscreteZeroPole_k;
  real_T rtb_DiscreteZeroPole_o;
  real_T rtb_DiscreteZeroPole_g;
  real_T rtb_DataTypeConversion2_b;
  real_T rtb_DataTypeConversion2_j;
  real_T rtb_DataTypeConversion2_gb;
  real_T rtb_DiscreteZeroPole_ki;
  real32_T rtb_RhhcosphicoslambXe;
  real32_T rtb_Product_pc;
  real32_T rtb_Product1_gz;
  real32_T rtb_Product2_noi;
  real32_T rtb_Product3_ap;
  real32_T rtb_Deg2R1;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_jxi;
  int32_T yIdx;
  int16_T colIdx;
  real32_T rtb_y_ox[3];
  real32_T rtb_ixk;
  real32_T rtb_kxj;
  real32_T rtb_ixj;
  real32_T rtb_kxi;
  real32_T rtb_Sum_bf;
  uint8_T rtb_DataTypeConversion2_h;
  real32_T rtb_Product1_e[3];
  real32_T rtb_Product1_kx[3];
  real32_T rtb_Add[3];
  real32_T rtb_VectorConcatenate[7];
  real32_T rtb_q_dot[4];
  real32_T tmp[9];
  real32_T tmp_0[3];
  real32_T rtb_TmpSignalConversionAtSFun_0[9];
  real32_T rtb_y_e;
  real32_T rtb_Deg2R_idx_0;
  real32_T rtb_Deg2R_idx_1;

  /* Sqrt: '<S521>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S478>/Discrete-Time Integrator1'
   *  Product: '<S522>/Product'
   *  Product: '<S522>/Product1'
   *  Product: '<S522>/Product2'
   *  Product: '<S522>/Product3'
   *  Sum: '<S522>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt
    (((AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] *
       AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] +
       AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] *
       AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1]) +
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] *
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2]) +
     AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] *
     AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3]);

  /* Product: '<S494>/Product' incorporates:
   *  DiscreteIntegrator: '<S478>/Discrete-Time Integrator1'
   */
  rtb_Product_pc = AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] /
    rtb_RhhcosphicoslambXe;

  /* Product: '<S494>/Product1' incorporates:
   *  DiscreteIntegrator: '<S478>/Discrete-Time Integrator1'
   */
  rtb_Product1_gz = AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] /
    rtb_RhhcosphicoslambXe;

  /* Product: '<S494>/Product2' incorporates:
   *  DiscreteIntegrator: '<S478>/Discrete-Time Integrator1'
   */
  rtb_Product2_noi = AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2]
    / rtb_RhhcosphicoslambXe;

  /* Product: '<S494>/Product3' incorporates:
   *  DiscreteIntegrator: '<S478>/Discrete-Time Integrator1'
   */
  rtb_Product3_ap = AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] /
    rtb_RhhcosphicoslambXe;

  /* Sqrt: '<S534>/sqrt' incorporates:
   *  Product: '<S535>/Product'
   *  Product: '<S535>/Product1'
   *  Product: '<S535>/Product2'
   *  Product: '<S535>/Product3'
   *  Sum: '<S535>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt(((rtb_Product_pc * rtb_Product_pc +
    rtb_Product1_gz * rtb_Product1_gz) + rtb_Product2_noi * rtb_Product2_noi) +
    rtb_Product3_ap * rtb_Product3_ap);

  /* Product: '<S533>/Product' */
  rtb_Deg2R1 = rtb_Product_pc / rtb_RhhcosphicoslambXe;

  /* Product: '<S533>/Product1' */
  rtb_RhhcosphisinlambYe = rtb_Product1_gz / rtb_RhhcosphicoslambXe;

  /* Product: '<S533>/Product2' */
  rtb_jxi = rtb_Product2_noi / rtb_RhhcosphicoslambXe;

  /* Product: '<S533>/Product3' */
  rtb_RhhcosphicoslambXe = rtb_Product3_ap / rtb_RhhcosphicoslambXe;

  /* Sum: '<S523>/Sum' incorporates:
   *  Product: '<S523>/Product'
   *  Product: '<S523>/Product1'
   *  Product: '<S523>/Product2'
   *  Product: '<S523>/Product3'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[0] = ((rtb_Deg2R1 * rtb_Deg2R1 +
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S526>/Gain' incorporates:
   *  Product: '<S526>/Product2'
   *  Product: '<S526>/Product3'
   *  Sum: '<S526>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[1] = (rtb_RhhcosphisinlambYe * rtb_jxi
    - rtb_RhhcosphicoslambXe * rtb_Deg2R1) * 2.0F;

  /* Gain: '<S529>/Gain' incorporates:
   *  Product: '<S529>/Product1'
   *  Product: '<S529>/Product2'
   *  Sum: '<S529>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[2] = (rtb_Deg2R1 * rtb_jxi +
    rtb_RhhcosphisinlambYe * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Gain: '<S524>/Gain' incorporates:
   *  Product: '<S524>/Product2'
   *  Product: '<S524>/Product3'
   *  Sum: '<S524>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[3] = (rtb_RhhcosphicoslambXe *
    rtb_Deg2R1 + rtb_RhhcosphisinlambYe * rtb_jxi) * 2.0F;

  /* Sum: '<S527>/Sum' incorporates:
   *  Product: '<S527>/Product'
   *  Product: '<S527>/Product1'
   *  Product: '<S527>/Product2'
   *  Product: '<S527>/Product3'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[4] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) + rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S530>/Gain' incorporates:
   *  Product: '<S530>/Product1'
   *  Product: '<S530>/Product2'
   *  Sum: '<S530>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[5] = (rtb_jxi * rtb_RhhcosphicoslambXe
    - rtb_Deg2R1 * rtb_RhhcosphisinlambYe) * 2.0F;

  /* Gain: '<S525>/Gain' incorporates:
   *  Product: '<S525>/Product1'
   *  Product: '<S525>/Product2'
   *  Sum: '<S525>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[6] = (rtb_RhhcosphisinlambYe *
    rtb_RhhcosphicoslambXe - rtb_Deg2R1 * rtb_jxi) * 2.0F;

  /* Gain: '<S528>/Gain' incorporates:
   *  Product: '<S528>/Product1'
   *  Product: '<S528>/Product2'
   *  Sum: '<S528>/Sum'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[7] = (rtb_Deg2R1 *
    rtb_RhhcosphisinlambYe + rtb_jxi * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Sum: '<S531>/Sum' incorporates:
   *  Product: '<S531>/Product'
   *  Product: '<S531>/Product1'
   *  Product: '<S531>/Product2'
   *  Product: '<S531>/Product3'
   */
  AUAV_V3_TestSensors_B.VectorConcatenate[8] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) +
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S478>/Submatrix1' */
  yIdx = 0L;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    AUAV_V3_TestSensors_B.Submatrix1[yIdx] =
      AUAV_V3_TestSensors_B.VectorConcatenate[(int32_T)(colIdx * 3)];
    yIdx++;
  }

  /* End of S-Function (sdspsubmtrx): '<S478>/Submatrix1' */

  /* MATLAB Function: '<S478>/myMux Fun2' incorporates:
   *  Constant: '<S478>/Constant1'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/myMux Fun2': '<S496>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S496>:1:5' */
  rtb_y_ox[0] = AUAV_V3_TestSensors_B.Submatrix1[0];
  rtb_y_ox[1] = AUAV_V3_TestSensors_B.Submatrix1[1];
  rtb_y_ox[2] = 0.0F;

  /* MATLAB Function: '<S518>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_o(rtb_y_ox,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_a);

  /* MATLAB Function: '<S517>/negprotect' */
  AUAV_V3_TestSen_negprotect
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_a.xDoty,
     &AUAV_V3_TestSensors_B.sf_negprotect_m);

  /* S-Function (MCHP_C_function_Call): '<S517>/[apUtils.c]' */
  AUAV_V3_TestSensors_B.apUtilsc = mySqrt(
    AUAV_V3_TestSensors_B.sf_negprotect_m.zpVal
    );

  /* Saturate: '<S493>/Zero Bound' */
  if (AUAV_V3_TestSensors_B.apUtilsc <= 0.001F) {
    rtb_RhhcosphicoslambXe = 0.001F;
  } else {
    rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_B.apUtilsc;
  }

  /* End of Saturate: '<S493>/Zero Bound' */

  /* Product: '<S493>/Divide' incorporates:
   *  MATLAB Function: '<S478>/myMux Fun2'
   */
  rtb_y_ox[0] = AUAV_V3_TestSensors_B.Submatrix1[0] / rtb_RhhcosphicoslambXe;
  rtb_y_e = AUAV_V3_TestSensors_B.Submatrix1[1] / rtb_RhhcosphicoslambXe;
  rtb_y_ox[2] = 0.0F / rtb_RhhcosphicoslambXe;

  /* S-Function (MCHP_C_function_Call): '<S14>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[0]
                );

  /* Gain: '<S547>/Deg2R' */
  rtb_Deg2R_idx_0 = 0.0174532924F *
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[2];
  rtb_Deg2R_idx_1 = 0.0174532924F *
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[1];

  /* Gain: '<S548>/Deg2R' */
  rtb_Sum_bf = 0.0174532924F *
    AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[0];

  /* Trigonometry: '<S548>/sin(phi)' */
  rtb_kxi = (real32_T)sin(rtb_Sum_bf);

  /* Sum: '<S548>/Sum1' incorporates:
   *  Constant: '<S548>/const'
   *  Product: '<S548>/Product1'
   *  Product: '<S548>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_kxi * rtb_kxi * 0.00669425726F;

  /* Fcn: '<S548>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S548>/f' */

  /* Product: '<S548>/Rh' incorporates:
   *  Constant: '<S548>/Re=equatorial radius'
   */
  rtb_ixj = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S548>/Sum2' */
  rtb_kxj = AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[2] + rtb_ixj;

  /* Trigonometry: '<S548>/cos(phi)' */
  rtb_Sum_bf = (real32_T)cos(rtb_Sum_bf);

  /* Gain: '<S548>/Deg2R1' */
  rtb_ixk = 0.0174532924F *
    AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[1];

  /* Product: '<S548>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S548>/cos(lamb)'
   */
  rtb_jxi = rtb_kxj * rtb_Sum_bf * (real32_T)cos(rtb_ixk);

  /* Product: '<S548>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S548>/sin(lamb)'
   */
  rtb_kxj = rtb_kxj * rtb_Sum_bf * (real32_T)sin(rtb_ixk);

  /* Product: '<S548>/Ze' incorporates:
   *  Product: '<S548>/Rh(1-e^2)'
   *  Sum: '<S548>/Sum4'
   */
  rtb_kxi *= 0.993305743F * rtb_ixj +
    AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[2];

  /* Gain: '<S539>/Deg2R' */
  rtb_Sum_bf = 0.0174532924F *
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[1];

  /* Trigonometry: '<S539>/sin(phi)' */
  rtb_ixj = (real32_T)sin(rtb_Sum_bf);

  /* Sum: '<S539>/Sum1' incorporates:
   *  Constant: '<S539>/const'
   *  Product: '<S539>/Product1'
   *  Product: '<S539>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_ixj * rtb_ixj * 0.00669425726F;

  /* Fcn: '<S539>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S539>/f' */

  /* Product: '<S539>/Rh' incorporates:
   *  Constant: '<S539>/Re=equatorial radius'
   */
  rtb_ixk = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S539>/Sum2' */
  rtb_RhhcosphisinlambYe =
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[0] + rtb_ixk;

  /* Trigonometry: '<S539>/cos(phi)' */
  rtb_Sum_bf = (real32_T)cos(rtb_Sum_bf);

  /* Gain: '<S539>/Deg2R1' */
  rtb_Deg2R1 = 0.0174532924F *
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[2];

  /* Product: '<S539>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S539>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_Sum_bf * (real32_T)cos
    (rtb_Deg2R1);

  /* Product: '<S539>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S539>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_Sum_bf * (real32_T)sin
    (rtb_Deg2R1);

  /* Product: '<S539>/Ze' incorporates:
   *  Product: '<S539>/Rh(1-e^2)'
   *  Sum: '<S539>/Sum4'
   */
  rtb_ixj *= 0.993305743F * rtb_ixk +
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorM_c[0];

  /* S-Function (MCHP_C_function_Call): '<S14>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  AUAV_V3_TestSensors_B.ChecksifFixTypeis3updateSensorM = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S481>/Enable'
   */
  if (AUAV_V3_TestSensors_B.ChecksifFixTypeis3updateSensorM > 0) {
    /* SignalConversion: '<S547>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S550>/11'
     *  Fcn: '<S550>/12'
     *  Fcn: '<S550>/13'
     *  Fcn: '<S550>/21'
     *  Fcn: '<S550>/22'
     *  Fcn: '<S550>/23'
     *  Fcn: '<S550>/31'
     *  Fcn: '<S550>/32'
     *  Fcn: '<S550>/33'
     */
    tmp[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    tmp[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos(rtb_Deg2R_idx_0);
    tmp[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    tmp[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin(rtb_Deg2R_idx_1);
    tmp[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    tmp[7] = 0.0F;
    tmp[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Sum: '<S483>/Sum1' */
    rtb_ixk = rtb_jxi - rtb_RhhcosphicoslambXe;
    rtb_Deg2R1 = rtb_kxj - rtb_RhhcosphisinlambYe;
    rtb_RhhcosphicoslambXe = rtb_kxi - rtb_ixj;

    /* Product: '<S547>/Product1' incorporates:
     *  Gain: '<S483>/UEN 2 NEU'
     */
    for (colIdx = 0; colIdx < 3; colIdx++) {
      tmp_0[colIdx] = tmp[colIdx + 6] * rtb_RhhcosphicoslambXe + (tmp[colIdx + 3]
        * rtb_Deg2R1 + tmp[colIdx] * rtb_ixk);
    }

    /* End of Product: '<S547>/Product1' */

    /* Inport: '<S481>/In1' */
    for (colIdx = 0; colIdx < 3; colIdx++) {
      /* Gain: '<S483>/UEN 2 NEU' */
      AUAV_V3_TestSensors_B.In1[colIdx] = 0.0F;
      AUAV_V3_TestSensors_B.In1[colIdx] +=
        AUAV_V3_TestSensors_ConstP.pooled64[colIdx] * tmp_0[0];
      AUAV_V3_TestSensors_B.In1[colIdx] +=
        AUAV_V3_TestSensors_ConstP.pooled64[colIdx + 3] * tmp_0[1];
      AUAV_V3_TestSensors_B.In1[colIdx] +=
        AUAV_V3_TestSensors_ConstP.pooled64[colIdx + 6] * tmp_0[2];
    }

    /* End of Inport: '<S481>/In1' */

    /* Inport: '<S481>/In2' */
    AUAV_V3_TestSensors_B.In2 =
      AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[3];

    /* Inport: '<S481>/In3' */
    AUAV_V3_TestSensors_B.In3 =
      AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[4];
  }

  /* End of Outputs for SubSystem: '<S14>/Enabled Subsystem' */

  /* Gain: '<S536>/Unit Conversion' */
  AUAV_V3_TestSensors_B.UnitConversion = 0.0174532924F *
    AUAV_V3_TestSensors_B.In2;

  /* S-Function (MCHP_C_function_Call): '<S537>/[apUtils.c]1' */
  AUAV_V3_TestSensors_B.apUtilsc1 = myCos(
    AUAV_V3_TestSensors_B.UnitConversion
    );

  /* S-Function (MCHP_C_function_Call): '<S537>/[apUtils.c]' */
  AUAV_V3_TestSensors_B.apUtilsc_a = mySin(
    AUAV_V3_TestSensors_B.UnitConversion
    );

  /* Product: '<S479>/Product1' */
  /* MATLAB Function 'Position_and_Attitude_Filter/COG.SOG2V/myMux Fun2': '<S538>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S538>:1:5' */
  rtb_Deg2R_idx_0 = AUAV_V3_TestSensors_B.In3 * AUAV_V3_TestSensors_B.apUtilsc1;

  /* Product: '<S479>/Product' */
  rtb_Deg2R_idx_1 = AUAV_V3_TestSensors_B.apUtilsc_a * AUAV_V3_TestSensors_B.In3;

  /* Abs: '<S541>/Abs1' incorporates:
   *  Sum: '<S544>/Diff'
   *  UnitDelay: '<S544>/UD'
   */
  rtb_ixk = (real32_T)fabs(AUAV_V3_TestSensors_B.In1[0] -
    AUAV_V3_TestSensors_DWork.UD_DSTATE);

  /* Abs: '<S542>/Abs1' incorporates:
   *  Sum: '<S545>/Diff'
   *  UnitDelay: '<S545>/UD'
   */
  rtb_Deg2R1 = (real32_T)fabs(AUAV_V3_TestSensors_B.In1[1] -
    AUAV_V3_TestSensors_DWork.UD_DSTATE_o);

  /* Abs: '<S543>/Abs1' incorporates:
   *  Sum: '<S546>/Diff'
   *  UnitDelay: '<S546>/UD'
   */
  rtb_RhhcosphicoslambXe = (real32_T)fabs(AUAV_V3_TestSensors_B.In1[2] -
    AUAV_V3_TestSensors_DWork.UD_DSTATE_o2);

  /* Saturate: '<S541>/Saturation1' */
  if (rtb_ixk > 1.0F) {
    rtb_ixk = 1.0F;
  }

  /* Saturate: '<S542>/Saturation1' */
  if (rtb_Deg2R1 > 1.0F) {
    rtb_Deg2R1 = 1.0F;
  }

  /* Saturate: '<S543>/Saturation1' */
  if (rtb_RhhcosphicoslambXe > 1.0F) {
    rtb_RhhcosphicoslambXe = 1.0F;
  }

  /* Sum: '<S482>/Sum' incorporates:
   *  Saturate: '<S541>/Saturation1'
   *  Saturate: '<S542>/Saturation1'
   *  Saturate: '<S543>/Saturation1'
   */
  rtb_ixk = (rtb_ixk + rtb_Deg2R1) + rtb_RhhcosphicoslambXe;

  /* Saturate: '<S482>/Saturation1' */
  if (rtb_ixk > 1.0F) {
    rtb_ixk = 1.0F;
  }

  /* DataTypeConversion: '<S482>/Data Type Conversion2' incorporates:
   *  Saturate: '<S482>/Saturation1'
   */
  rtb_RhhcosphicoslambXe = (real32_T)floor(rtb_ixk);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_DataTypeConversion2_h = 0U;
  } else {
    rtb_DataTypeConversion2_h = (uint8_T)(real32_T)fmod(rtb_RhhcosphicoslambXe,
      256.0F);
  }

  /* End of DataTypeConversion: '<S482>/Data Type Conversion2' */

  /* Outputs for Enabled SubSystem: '<S552>/Subsystem' incorporates:
   *  EnablePort: '<S560>/Enable'
   */
  /* RelationalOperator: '<S558>/Compare' incorporates:
   *  Constant: '<S558>/Constant'
   */
  if ((AUAV_V3_TestSensors_B.In3 > 7.0F) > 0) {
    if (!AUAV_V3_TestSensors_DWork.Subsystem_MODE) {
      AUAV_V3_TestSensors_DWork.Subsystem_MODE = true;
    }

    /* MATLAB Function: '<S560>/Embedded MATLAB Function1' */
    AU_EmbeddedMATLABFunction1(rtb_Deg2R_idx_1, rtb_DataTypeConversion2_h,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_k,
      &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_k);

    /* MATLAB Function: '<S560>/Embedded MATLAB Function2' */
    AU_EmbeddedMATLABFunction1(rtb_Deg2R_idx_0, rtb_DataTypeConversion2_h,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2,
      &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2);
  } else {
    if (AUAV_V3_TestSensors_DWork.Subsystem_MODE) {
      /* Disable for Outport: '<S560>/Vn_fil' */
      AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2.y = 0.0F;

      /* Disable for Outport: '<S560>/Ve_fil' */
      AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_k.y = 0.0F;
      AUAV_V3_TestSensors_DWork.Subsystem_MODE = false;
    }
  }

  /* End of RelationalOperator: '<S558>/Compare' */
  /* End of Outputs for SubSystem: '<S552>/Subsystem' */

  /* MATLAB Function: '<S553>/Embedded MATLAB Function3' incorporates:
   *  Gain: '<S14>/[1 1 -1]'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3': '<S563>:1' */
  /*  persistent lastH; */
  if (!AUAV_V3_TestSensors_DWork.lastGps_h_not_empty) {
    /* '<S563>:1:7' */
    AUAV_V3_TestSensors_DWork.lastGps_h_not_empty = true;

    /* '<S563>:1:9' */
    AUAV_V3_TestSensors_DWork.TimeSinceLast = 0.01F;

    /*      lastH           = single(-baseHeight); */
    /*      h_rate          = single(0); */
  }

  if (rtb_DataTypeConversion2_h != 0) {
    /* '<S563>:1:16' */
    rtb_RhhcosphicoslambXe = (-AUAV_V3_TestSensors_B.In1[2] -
      AUAV_V3_TestSensors_DWork.lastGps_h) /
      AUAV_V3_TestSensors_DWork.TimeSinceLast;

    /* '<S563>:1:17' */
    AUAV_V3_TestSensors_DWork.TimeSinceLast = 0.0F;
    if (!((rtb_RhhcosphicoslambXe > -10.6) && (rtb_RhhcosphicoslambXe < 17.67)))
    {
      /*          h = lastH; */
      /* '<S563>:1:23' */
      rtb_RhhcosphicoslambXe = 0.0F;
    } else {
      /* '<S563>:1:18' */
      /*          h = lastH + gps_h - lastGps_h; */
      /* '<S563>:1:20' */
    }

    /* '<S563>:1:25' */
    AUAV_V3_TestSensors_DWork.lastGps_h = -AUAV_V3_TestSensors_B.In1[2];
  } else {
    /*      h = lastH; */
    /* '<S563>:1:28' */
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* '<S563>:1:31' */
  AUAV_V3_TestSensors_DWork.TimeSinceLast += 0.01F;

  /* End of MATLAB Function: '<S553>/Embedded MATLAB Function3' */

  /* MATLAB Function: '<S564>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S564>/Constant'
   *  Constant: '<S564>/Constant1'
   *  Gain: '<S552>/Gain'
   *  Gain: '<S553>/Gain'
   *  Sum: '<S553>/Sum'
   */
  /*  lastH = h; */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
  /*  persistent lastGps_h; */
  /*  persistent TimeSinceLast; */
  /*  persistent rate; */
  /*  persistent lastH; */
  /*   */
  /*  if (isempty(lastGps_h)) */
  /*      lastGps_h       = single(0.0); */
  /*      TimeSinceLast   = single(apSampleTime); */
  /*      rate            = single(0); */
  /*      lastH           = single(-baseHeight); */
  /*      h_rate          = single(0); */
  /*  end */
  /*   */
  /*  if (NewGPS) */
  /*      rate = single((gps_h - lastGps_h)/TimeSinceLast); */
  /*      TimeSinceLast = single(0); */
  /*      if (rate > -10.6) && (rate < 17.67) */
  /*          h = lastH + gps_h - lastGps_h; */
  /*  %         h_rate = rate; */
  /*      else */
  /*          h = lastH; */
  /*  %         h_rate = single(0); */
  /*      end */
  /*      lastGps_h = gps_h; */
  /*  else */
  /*      h = lastH; */
  /*  %     h_rate = single(0); */
  /*  end */
  /*   */
  /*  TimeSinceLast = TimeSinceLast + apSampleTime; */
  /*  lastH = h; */
  AUA_EmbeddedMATLABFunction(0.5F * rtb_RhhcosphicoslambXe +
    -AUAV_V3_TestSensors_B.EnabledSubsystem_m.In1, 0.01, 0.31830988618379069,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ph,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ph);

  /* Gain: '<S557>/Gain' incorporates:
   *  Sum: '<S557>/Sum1'
   *  UnitDelay: '<S557>/Unit Delay'
   */
  rtb_jxi = (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ph.y -
             AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE) * 2.0F;

  /* MATLAB Function: '<S552>/myMux Fun2' */
  AUAV_V3_TestSe_myMuxFun1_e(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2.y,
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_k.y, rtb_jxi,
    &AUAV_V3_TestSensors_B.sf_myMuxFun2_a);

  /* Product: '<S478>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_Product1_e[colIdx] = AUAV_V3_TestSensors_B.VectorConcatenate[colIdx + 6]
      * AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[2] +
      (AUAV_V3_TestSensors_B.VectorConcatenate[colIdx + 3] *
       AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[1] +
       AUAV_V3_TestSensors_B.VectorConcatenate[colIdx] *
       AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S478>/Product1' */

  /* Product: '<S552>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_Product1_kx[colIdx] = AUAV_V3_TestSensors_B.VectorConcatenate[colIdx + 6]
      * AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[2] +
      (AUAV_V3_TestSensors_B.VectorConcatenate[colIdx + 3] *
       AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[1] +
       AUAV_V3_TestSensors_B.VectorConcatenate[colIdx] *
       AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S552>/Product1' */

  /* Sum: '<S552>/Sum' incorporates:
   *  Delay: '<S559>/Integer Delay1'
   *  Gain: '<S559>/Gain2'
   *  Sum: '<S559>/Sum5'
   */
  rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_B.DataTypeConversion3[0] -
    (rtb_Product1_kx[0] - AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[0]) *
    20.0F;
  rtb_Deg2R1 = AUAV_V3_TestSensors_B.DataTypeConversion3[1] - (rtb_Product1_kx[1]
    - AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[1]) * 20.0F;
  rtb_ixk = AUAV_V3_TestSensors_B.DataTypeConversion3[2] - (rtb_Product1_kx[2] -
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[2]) * 20.0F;

  /* RateLimiter: '<S552>/Rate Limiter' */
  rtb_Add[0] = rtb_RhhcosphicoslambXe - AUAV_V3_TestSensors_DWork.PrevY[0];
  rtb_Add[1] = rtb_Deg2R1 - AUAV_V3_TestSensors_DWork.PrevY[1];
  rtb_Add[2] = rtb_ixk - AUAV_V3_TestSensors_DWork.PrevY[2];
  rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe;
  if (rtb_Add[0] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[0] + 9.896E-9F;
  } else {
    if (rtb_Add[0] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[0] + -9.896E-9F;
    }
  }

  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe;
  rtb_RhhcosphisinlambYe = rtb_Deg2R1;
  if (rtb_Add[1] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[1] + 9.896E-9F;
  } else {
    if (rtb_Add[1] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[1] + -9.896E-9F;
    }
  }

  rtb_Deg2R1 = rtb_RhhcosphisinlambYe;
  rtb_RhhcosphisinlambYe = rtb_ixk;
  if (rtb_Add[2] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[2] + 9.896E-9F;
  } else {
    if (rtb_Add[2] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.PrevY[2] + -9.896E-9F;
    }
  }

  AUAV_V3_TestSensors_DWork.PrevY[0] = rtb_RhhcosphicoslambXe;
  AUAV_V3_TestSensors_DWork.PrevY[1] = rtb_Deg2R1;
  AUAV_V3_TestSensors_DWork.PrevY[2] = rtb_RhhcosphisinlambYe;

  /* End of RateLimiter: '<S552>/Rate Limiter' */

  /* DataTypeConversion: '<S554>/Data Type Conversion2' */
  rtb_DataTypeConversion2_g[0] = rtb_RhhcosphicoslambXe;
  rtb_DataTypeConversion2_g[1] = rtb_Deg2R1;
  rtb_DataTypeConversion2_g[2] = rtb_RhhcosphisinlambYe;

  /* DiscreteZeroPole: '<S566>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_f = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[0];
    rtb_DiscreteZeroPole_f += 3.9156825028515473E-10*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* DiscreteZeroPole: '<S567>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_c = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[1];
    rtb_DiscreteZeroPole_c += 3.9156825028515473E-10*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_j;
  }

  /* DiscreteZeroPole: '<S568>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ck = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[2];
    rtb_DiscreteZeroPole_ck += 3.9156825028515473E-10*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* MATLAB Function: '<S554>/myMux Fun1' */
  AUAV_V3_TestSens_myMuxFun1(rtb_DiscreteZeroPole_f, rtb_DiscreteZeroPole_c,
    rtb_DiscreteZeroPole_ck, &AUAV_V3_TestSensors_B.sf_myMuxFun1_d);

  /* DataTypeConversion: '<S554>/Data Type Conversion1' */
  AUAV_V3_TestSensors_B.DataTypeConversion1[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_d.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion1[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_d.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion1[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_d.y[2];

  /* Sum: '<S478>/Add' incorporates:
   *  Delay: '<S478>/Integer Delay'
   *  Delay: '<S489>/Integer Delay1'
   *  Gain: '<S489>/Gain2'
   *  Product: '<S504>/i x j'
   *  Product: '<S504>/j x k'
   *  Product: '<S504>/k x i'
   *  Product: '<S505>/i x k'
   *  Product: '<S505>/j x i'
   *  Product: '<S505>/k x j'
   *  Sum: '<S487>/Sum'
   *  Sum: '<S489>/Sum5'
   */
  rtb_Add[0] = (((AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[1] *
                  rtb_Product1_e[2] -
                  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[2] *
                  rtb_Product1_e[1]) + (rtb_Product1_e[0] -
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[0]) * 20.0F) -
                AUAV_V3_TestSensors_B.DataTypeConversion3[0]) -
    AUAV_V3_TestSensors_B.DataTypeConversion1[0];
  rtb_Add[1] = (((AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[2] *
                  rtb_Product1_e[0] -
                  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[0] *
                  rtb_Product1_e[2]) + (rtb_Product1_e[1] -
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[1]) * 20.0F) -
                AUAV_V3_TestSensors_B.DataTypeConversion3[1]) -
    AUAV_V3_TestSensors_B.DataTypeConversion1[1];
  rtb_Add[2] = (((AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[0] *
                  rtb_Product1_e[1] -
                  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[1] *
                  rtb_Product1_e[0]) + (rtb_Product1_e[2] -
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[2]) * 20.0F) -
                AUAV_V3_TestSensors_B.DataTypeConversion3[2]) -
    AUAV_V3_TestSensors_B.DataTypeConversion1[2];

  /* MATLAB Function: '<S513>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_o(rtb_Add,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_op);

  /* MATLAB Function: '<S512>/negprotect' */
  AUAV_V3_TestSen_negprotect
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_op.xDoty,
     &AUAV_V3_TestSensors_B.sf_negprotect_k);

  /* S-Function (MCHP_C_function_Call): '<S512>/[apUtils.c]' */
  AUAV_V3_TestSensors_B.apUtilsc_f = mySqrt(
    AUAV_V3_TestSensors_B.sf_negprotect_k.zpVal
    );

  /* Saturate: '<S492>/Zero Bound' */
  if (AUAV_V3_TestSensors_B.apUtilsc_f <= 0.001F) {
    rtb_RhhcosphicoslambXe = 0.001F;
  } else {
    rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_B.apUtilsc_f;
  }

  /* End of Saturate: '<S492>/Zero Bound' */

  /* Product: '<S492>/Divide' */
  rtb_Add[0] /= rtb_RhhcosphicoslambXe;
  rtb_Add[1] /= rtb_RhhcosphicoslambXe;
  rtb_RhhcosphisinlambYe = rtb_Add[2] / rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S478>/Submatrix' */
  yIdx = 0L;
  colIdx = 2;
  while (colIdx <= 2) {
    AUAV_V3_TestSensors_B.g_hat[yIdx] = AUAV_V3_TestSensors_B.VectorConcatenate
      [6L];
    AUAV_V3_TestSensors_B.g_hat[1L + yIdx] =
      AUAV_V3_TestSensors_B.VectorConcatenate[7L];
    AUAV_V3_TestSensors_B.g_hat[2L + yIdx] =
      AUAV_V3_TestSensors_B.VectorConcatenate[8L];
    yIdx += 3L;
    colIdx = 3;
  }

  /* End of S-Function (sdspsubmtrx): '<S478>/Submatrix' */

  /* Sum: '<S488>/Sum' incorporates:
   *  MATLAB Function: '<S479>/myMux Fun2'
   *  Product: '<S506>/i x j'
   *  Product: '<S506>/j x k'
   *  Product: '<S506>/k x i'
   *  Product: '<S507>/i x k'
   *  Product: '<S507>/j x i'
   *  Product: '<S507>/k x j'
   */
  rtb_Deg2R1 = rtb_y_e * 0.0F - rtb_y_ox[2] * AUAV_V3_TestSensors_B.apUtilsc_a;
  rtb_ixk = rtb_y_ox[2] * AUAV_V3_TestSensors_B.apUtilsc1 - rtb_y_ox[0] * 0.0F;
  rtb_RhhcosphicoslambXe = rtb_y_ox[0] * AUAV_V3_TestSensors_B.apUtilsc_a -
    rtb_y_e * AUAV_V3_TestSensors_B.apUtilsc1;

  /* Product: '<S478>/Product2' incorporates:
   *  Gain: '<S478>/Gain1'
   */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp_0[colIdx] = AUAV_V3_TestSensors_B.VectorConcatenate[colIdx + 6] *
      rtb_RhhcosphicoslambXe + (AUAV_V3_TestSensors_B.VectorConcatenate[colIdx +
      3] * rtb_ixk + AUAV_V3_TestSensors_B.VectorConcatenate[colIdx] *
      rtb_Deg2R1);
  }

  /* End of Product: '<S478>/Product2' */

  /* Product: '<S502>/i x j' */
  rtb_RhhcosphicoslambXe = rtb_Add[0];

  /* Product: '<S503>/i x k' */
  rtb_Deg2R1 = rtb_Add[0];

  /* Product: '<S503>/j x i' */
  rtb_ixk = rtb_Add[1];

  /* Sum: '<S478>/Sum4' incorporates:
   *  Gain: '<S478>/Gain1'
   *  Product: '<S502>/i x j'
   *  Product: '<S502>/j x k'
   *  Product: '<S502>/k x i'
   *  Product: '<S503>/i x k'
   *  Product: '<S503>/j x i'
   *  Product: '<S503>/k x j'
   *  Sum: '<S486>/Sum'
   */
  rtb_Add[0] = (rtb_Add[1] * AUAV_V3_TestSensors_B.g_hat[2] -
                rtb_RhhcosphisinlambYe * AUAV_V3_TestSensors_B.g_hat[1]) + 30.0F
    * tmp_0[0];
  rtb_Add[1] = (rtb_RhhcosphisinlambYe * AUAV_V3_TestSensors_B.g_hat[0] -
                rtb_Deg2R1 * AUAV_V3_TestSensors_B.g_hat[2]) + 30.0F * tmp_0[1];
  rtb_Add[2] = (rtb_RhhcosphicoslambXe * AUAV_V3_TestSensors_B.g_hat[1] -
                rtb_ixk * AUAV_V3_TestSensors_B.g_hat[0]) + 30.0F * tmp_0[2];

  /* DataTypeConversion: '<S485>/Data Type Conversion2' */
  rtb_DataTypeConversion2_e2[0] = rtb_Add[0];
  rtb_DataTypeConversion2_e2[1] = rtb_Add[1];
  rtb_DataTypeConversion2_e2[2] = rtb_Add[2];

  /* DiscreteZeroPole: '<S498>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_k = 0.005*rtb_DataTypeConversion2_e2[0];
    rtb_DiscreteZeroPole_k += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* DiscreteZeroPole: '<S499>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_o = 0.005*rtb_DataTypeConversion2_e2[1];
    rtb_DiscreteZeroPole_o += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* DiscreteZeroPole: '<S500>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_g = 0.005*rtb_DataTypeConversion2_e2[2];
    rtb_DiscreteZeroPole_g += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* MATLAB Function: '<S485>/myMux Fun1' */
  AUAV_V3_TestSens_myMuxFun1(rtb_DiscreteZeroPole_k, rtb_DiscreteZeroPole_o,
    rtb_DiscreteZeroPole_g, &AUAV_V3_TestSensors_B.sf_myMuxFun1_n);

  /* RateLimiter: '<S478>/Bias Rate Limiter' incorporates:
   *  DataTypeConversion: '<S485>/Data Type Conversion1'
   */
  AUAV_V3_TestSensors_B.BiasRateLimiter[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[0] -
    AUAV_V3_TestSensors_DWork.PrevY_n[0];
  AUAV_V3_TestSensors_B.BiasRateLimiter[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[1] -
    AUAV_V3_TestSensors_DWork.PrevY_n[1];
  AUAV_V3_TestSensors_B.BiasRateLimiter[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[2] -
    AUAV_V3_TestSensors_DWork.PrevY_n[2];
  if (AUAV_V3_TestSensors_B.BiasRateLimiter[0] > 2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[0] =
      AUAV_V3_TestSensors_DWork.PrevY_n[0] + 2.5572E-9F;
  } else if (AUAV_V3_TestSensors_B.BiasRateLimiter[0] < -2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[0] =
      AUAV_V3_TestSensors_DWork.PrevY_n[0] + -2.5572E-9F;
  } else {
    AUAV_V3_TestSensors_B.BiasRateLimiter[0] = (real32_T)
      AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[0];
  }

  if (AUAV_V3_TestSensors_B.BiasRateLimiter[1] > 2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[1] =
      AUAV_V3_TestSensors_DWork.PrevY_n[1] + 2.5572E-9F;
  } else if (AUAV_V3_TestSensors_B.BiasRateLimiter[1] < -2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[1] =
      AUAV_V3_TestSensors_DWork.PrevY_n[1] + -2.5572E-9F;
  } else {
    AUAV_V3_TestSensors_B.BiasRateLimiter[1] = (real32_T)
      AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[1];
  }

  if (AUAV_V3_TestSensors_B.BiasRateLimiter[2] > 2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[2] =
      AUAV_V3_TestSensors_DWork.PrevY_n[2] + 2.5572E-9F;
  } else if (AUAV_V3_TestSensors_B.BiasRateLimiter[2] < -2.5572E-9F) {
    AUAV_V3_TestSensors_B.BiasRateLimiter[2] =
      AUAV_V3_TestSensors_DWork.PrevY_n[2] + -2.5572E-9F;
  } else {
    AUAV_V3_TestSensors_B.BiasRateLimiter[2] = (real32_T)
      AUAV_V3_TestSensors_B.sf_myMuxFun1_n.y[2];
  }

  AUAV_V3_TestSensors_DWork.PrevY_n[0] = AUAV_V3_TestSensors_B.BiasRateLimiter[0];
  AUAV_V3_TestSensors_DWork.PrevY_n[1] = AUAV_V3_TestSensors_B.BiasRateLimiter[1];
  AUAV_V3_TestSensors_DWork.PrevY_n[2] = AUAV_V3_TestSensors_B.BiasRateLimiter[2];

  /* End of RateLimiter: '<S478>/Bias Rate Limiter' */

  /* Gain: '<S510>/Gain1' incorporates:
   *  Selector: '<S510>/Selector1'
   */
  rtb_VectorConcatenate[0] = AUAV_V3_TestSensors_B.VectorConcatenate[3];
  rtb_VectorConcatenate[1] = AUAV_V3_TestSensors_B.VectorConcatenate[0];
  rtb_VectorConcatenate[2] = -AUAV_V3_TestSensors_B.VectorConcatenate[6];

  /* Gain: '<S510>/Gain2' incorporates:
   *  Selector: '<S510>/Selector2'
   */
  rtb_VectorConcatenate[3] = AUAV_V3_TestSensors_B.VectorConcatenate[7];
  rtb_VectorConcatenate[4] = AUAV_V3_TestSensors_B.VectorConcatenate[8];

  /* Gain: '<S510>/Gain3' incorporates:
   *  Selector: '<S510>/Selector3'
   */
  rtb_VectorConcatenate[5] = -AUAV_V3_TestSensors_B.VectorConcatenate[1];
  rtb_VectorConcatenate[6] = AUAV_V3_TestSensors_B.VectorConcatenate[4];

  /* If: '<S490>/If' incorporates:
   *  Gain: '<S510>/Gain1'
   *  Selector: '<S510>/Selector1'
   */
  if ((-AUAV_V3_TestSensors_B.VectorConcatenate[6] >= 1.0F) ||
      (-AUAV_V3_TestSensors_B.VectorConcatenate[6] <= -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S490>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S509>/Action Port'
     */
    AUAV_V3_Test_AxisRotZeroR3(rtb_VectorConcatenate,
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[0],
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[1],
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[2]);

    /* End of Outputs for SubSystem: '<S490>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S490>/AxisRotDefault' incorporates:
     *  ActionPort: '<S508>/Action Port'
     */
    AUAV_V3_Tes_AxisRotDefault(rtb_VectorConcatenate,
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[0],
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[1],
      &AUAV_V3_TestSensors_B.DataTypeConversion_m[2]);

    /* End of Outputs for SubSystem: '<S490>/AxisRotDefault' */
  }

  /* End of If: '<S490>/If' */

  /* MATLAB Function: '<S478>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_l(AUAV_V3_TestSensors_B.DataTypeConversion_m[0],
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lt);

  /* Sum: '<S478>/Sum3' */
  AUAV_V3_TestSensors_B.GyroErr[0] = AUAV_V3_TestSensors_B.DataTypeConversion7[0]
    + AUAV_V3_TestSensors_B.BiasRateLimiter[0];
  AUAV_V3_TestSensors_B.GyroErr[1] = AUAV_V3_TestSensors_B.DataTypeConversion7[1]
    + AUAV_V3_TestSensors_B.BiasRateLimiter[1];
  AUAV_V3_TestSensors_B.GyroErr[2] = AUAV_V3_TestSensors_B.DataTypeConversion7[2]
    + AUAV_V3_TestSensors_B.BiasRateLimiter[2];

  /* Sum: '<S478>/Sum1' incorporates:
   *  Gain: '<S478>/Gain'
   */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[0] = 0.1F * rtb_Add[0] +
    AUAV_V3_TestSensors_B.GyroErr[0];
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[1] = 0.1F * rtb_Add[1] +
    AUAV_V3_TestSensors_B.GyroErr[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[2] = 0.1F * rtb_Add[2] +
    AUAV_V3_TestSensors_B.GyroErr[2];

  /* MATLAB Function: '<S478>/q dot calc' incorporates:
   *  SignalConversion: '<S497>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/q dot calc': '<S497>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S497>:1:5' */
  /* '<S497>:1:6' */
  /* '<S497>:1:7' */
  rtb_q_dot[1] = 0.0F;
  rtb_q_dot[2] = 0.0F;
  rtb_q_dot[3] = 0.0F;

  /* '<S497>:1:8' */
  rtb_q_dot[0] = (-0.5F * rtb_Product1_gz *
                  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[0] + -0.5F *
                  rtb_Product2_noi *
                  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[1]) + -0.5F *
    rtb_Product3_ap * AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[2];

  /* '<S497>:1:9' */
  rtb_TmpSignalConversionAtSFun_0[0] = rtb_Product_pc;
  rtb_TmpSignalConversionAtSFun_0[3] = -rtb_Product3_ap;
  rtb_TmpSignalConversionAtSFun_0[6] = rtb_Product2_noi;
  rtb_TmpSignalConversionAtSFun_0[1] = rtb_Product3_ap;
  rtb_TmpSignalConversionAtSFun_0[4] = rtb_Product_pc;
  rtb_TmpSignalConversionAtSFun_0[7] = -rtb_Product1_gz;
  rtb_TmpSignalConversionAtSFun_0[2] = -rtb_Product2_noi;
  rtb_TmpSignalConversionAtSFun_0[5] = rtb_Product1_gz;
  rtb_TmpSignalConversionAtSFun_0[8] = rtb_Product_pc;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp[3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx] * 0.5F;
    tmp[1 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 1] * 0.5F;
    tmp[2 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 2] * 0.5F;
  }

  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_q_dot[1 + colIdx] = 0.0F;
    rtb_q_dot[1 + colIdx] += tmp[colIdx] *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[0];
    rtb_q_dot[1 + colIdx] += tmp[colIdx + 3] *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[1];
    rtb_q_dot[1 + colIdx] += tmp[colIdx + 6] *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_j[2];
  }

  /* End of MATLAB Function: '<S478>/q dot calc' */

  /* DataTypeConversion: '<S570>/Data Type Conversion2' incorporates:
   *  Gain: '<S14>/[1 1 -1]'
   *  Gain: '<S555>/Gain'
   *  Sum: '<S555>/Sum'
   *  Sum: '<S555>/Sum1'
   *  UnitDelay: '<S555>/Unit Delay'
   */
  rtb_DataTypeConversion2_b = (AUAV_V3_TestSensors_B.In1[0] -
    AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_n) * 10.0F + rtb_Deg2R_idx_0;

  /* DiscreteZeroPole: '<S571>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_b;
    rtb_DiscreteZeroPole_ki += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_jl;
  }

  /* DataTypeConversion: '<S570>/Data Type Conversion1' */
  AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_n = (real32_T)
    rtb_DiscreteZeroPole_ki;

  /* DataTypeConversion: '<S572>/Data Type Conversion2' incorporates:
   *  Gain: '<S14>/[1 1 -1]'
   *  Gain: '<S556>/Gain'
   *  Sum: '<S556>/Sum'
   *  Sum: '<S556>/Sum1'
   *  UnitDelay: '<S556>/Unit Delay'
   */
  rtb_DataTypeConversion2_j = (AUAV_V3_TestSensors_B.In1[1] -
    AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_j) * 10.0F + rtb_Deg2R_idx_1;

  /* DiscreteZeroPole: '<S573>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_j;
    rtb_DiscreteZeroPole_ki += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* DataTypeConversion: '<S572>/Data Type Conversion1' */
  AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_j = (real32_T)
    rtb_DiscreteZeroPole_ki;

  /* DataTypeConversion: '<S574>/Data Type Conversion2' */
  rtb_DataTypeConversion2_gb = rtb_jxi;

  /* DiscreteZeroPole: '<S575>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_gb;
    rtb_DiscreteZeroPole_ki += 0.01*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_b;
  }

  /* DataTypeConversion: '<S574>/Data Type Conversion1' */
  AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE = (real32_T)rtb_DiscreteZeroPole_ki;

  /* MATLAB Function: '<S552>/myMux Fun1' incorporates:
   *  Gain: '<S552>/Gain1'
   */
  AUAV_V3_TestSe_myMuxFun1_e(AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_n,
    AUAV_V3_TestSensors_DWork.UnitDelay_DSTATE_j,
    -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ph.y,
    &AUAV_V3_TestSensors_B.sf_myMuxFun1_e);

  /* Update for DiscreteIntegrator: '<S478>/Discrete-Time Integrator1' */
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] += 0.01F *
    rtb_q_dot[0];
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] += 0.01F *
    rtb_q_dot[1];
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] += 0.01F *
    rtb_q_dot[2];
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] += 0.01F *
    rtb_q_dot[3];
  if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] >= 1.0F) {
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  } else {
    if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] <= -1.0F) {
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[0] = -1.0F;
    }
  }

  if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] >= 1.0F) {
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 1.0F;
  } else {
    if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] <= -1.0F) {
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[1] = -1.0F;
    }
  }

  if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] >= 1.0F) {
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 1.0F;
  } else {
    if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] <= -1.0F) {
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[2] = -1.0F;
    }
  }

  if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] >= 1.0F) {
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 1.0F;
  } else {
    if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] <= -1.0F) {
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator1_DSTATE[3] = -1.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S478>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S544>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE = AUAV_V3_TestSensors_B.In1[0];

  /* Update for UnitDelay: '<S545>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE_o = AUAV_V3_TestSensors_B.In1[1];

  /* Update for UnitDelay: '<S546>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE_o2 = AUAV_V3_TestSensors_B.In1[2];

  /* Update for Delay: '<S489>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[colIdx * 3] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3];
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[colIdx * 3 + 1] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3 + 1];
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[colIdx * 3 + 2] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3 + 2];
  }

  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[12] = rtb_Product1_e[0];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[13] = rtb_Product1_e[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE[14] = rtb_Product1_e[2];

  /* End of Update for Delay: '<S489>/Integer Delay1' */

  /* Update for Delay: '<S559>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[colIdx * 3] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3];
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[colIdx * 3 + 1] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3 + 1];
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[colIdx * 3 + 2] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3 + 2];
  }

  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[12] = rtb_Product1_kx[0];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[13] = rtb_Product1_kx[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_g[14] = rtb_Product1_kx[2];

  /* End of Update for Delay: '<S559>/Integer Delay1' */
  /* Update for DiscreteZeroPole: '<S566>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_o =
      rtb_DataTypeConversion2_g[0] + (-0.99960423832914991)*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* Update for DiscreteZeroPole: '<S567>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_j =
      rtb_DataTypeConversion2_g[1] + (-0.99960423832914991)*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_j;
  }

  /* Update for DiscreteZeroPole: '<S568>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_k =
      rtb_DataTypeConversion2_g[2] + (-0.99960423832914991)*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* Update for DiscreteZeroPole: '<S498>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_c =
      rtb_DataTypeConversion2_e2[0] + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* Update for DiscreteZeroPole: '<S499>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_n =
      rtb_DataTypeConversion2_e2[1] + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* Update for DiscreteZeroPole: '<S500>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_e =
      rtb_DataTypeConversion2_e2[2] + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* Update for DiscreteZeroPole: '<S571>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_jl =
      rtb_DataTypeConversion2_b + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_jl;
  }

  /* Update for DiscreteZeroPole: '<S573>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_a =
      rtb_DataTypeConversion2_j + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* Update for DiscreteZeroPole: '<S575>/Discrete Zero-Pole' */
  {
    AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_b =
      rtb_DataTypeConversion2_gb + 1.0*
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_b;
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
