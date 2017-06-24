/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Position_and_Attitude_Filter.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "Position_and_Attitude_Filter.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Output and update for atomic system:
 *    '<S478>/myMux Fun1'
 *    '<S547>/myMux Fun1'
 */
void SLUGS2_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
                      rtB_myMuxFun1_SLUGS2_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1': '<S494>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S494>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S506>/Embedded MATLAB Function'
 *    '<S511>/Embedded MATLAB Function'
 */
void S_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_g_T *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function': '<S508>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S508>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Initial conditions for atomic system:
 *    '<S553>/Embedded MATLAB Function1'
 *    '<S553>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunct_i_Init(rtDW_EmbeddedMATLABFunction1__T *localDW)
{
  localDW->LastU_not_empty = false;
  localDW->LastU = 0.0F;
  localDW->rate = 0.0F;
  localDW->OldRate = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S553>/Embedded MATLAB Function1'
 *    '<S553>/Embedded MATLAB Function2'
 */
void SL_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_S_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1': '<S569>:1' */
  if (!localDW->LastU_not_empty) {
    /* '<S569>:1:7' */
    localDW->LastU_not_empty = true;

    /* '<S569>:1:9' */
    localDW->TimeSinceLast = 0.01F;
  }

  if (rtu_NewGPS != 0) {
    /* '<S569>:1:15' */
    localDW->OldRate = localDW->rate;

    /* '<S569>:1:16' */
    localDW->rate = (rtu_u - localDW->LastU) / localDW->TimeSinceLast;

    /* '<S569>:1:17' */
    localDW->TimeSinceLast = 0.0F;

    /* '<S569>:1:18' */
    localDW->LastU = rtu_u;
  }

  /* '<S569>:1:20' */
  localB->y = (0.5F * localDW->rate + 0.5F * localDW->OldRate) *
    localDW->TimeSinceLast + rtu_u;

  /* '<S569>:1:21' */
  localDW->TimeSinceLast += 0.01F;
}

/* Initial conditions for atomic system: '<Root>/Position_and_Attitude_Filter' */
void Position_and_Attitude_Init(void)
{
  int16_T i;

  /* InitializeConditions for DiscreteIntegrator: '<S471>/Discrete-Time Integrator1' */
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 0.0F;

  /* InitializeConditions for MATLAB Function: '<S546>/Embedded MATLAB Function3' */
  SLUGS2_DWork.lastGps_h_not_empty = false;
  SLUGS2_DWork.lastGps_h = 0.0F;

  /* InitializeConditions for MATLAB Function: '<S557>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_ph);
  for (i = 0; i < 15; i++) {
    /* InitializeConditions for Delay: '<S482>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE[i] = 19.5F;

    /* InitializeConditions for Delay: '<S552>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[i] = 19.5F;
  }

  /* InitializeConditions for RateLimiter: '<S545>/Rate Limiter' */
  SLUGS2_DWork.PrevY[0] = 0.0F;
  SLUGS2_DWork.PrevY[1] = 0.0F;
  SLUGS2_DWork.PrevY[2] = 0.0F;

  /* InitializeConditions for RateLimiter: '<S471>/Bias Rate Limiter' */
  SLUGS2_DWork.PrevY_n[0] = 0.0F;
  SLUGS2_DWork.PrevY_n[1] = 0.0F;
  SLUGS2_DWork.PrevY_n[2] = 0.0F;
}

/* Start for atomic system: '<Root>/Position_and_Attitude_Filter' */
void Position_and_Attitud_Start(void)
{
  /* InitializeConditions for Enabled SubSystem: '<S545>/Subsystem' */

  /* InitializeConditions for MATLAB Function: '<S553>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_i_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction1_k);

  /* InitializeConditions for MATLAB Function: '<S553>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_i_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction2);

  /* End of InitializeConditions for SubSystem: '<S545>/Subsystem' */
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
  real32_T rtb_Product;
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

  /* Sqrt: '<S514>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S471>/Discrete-Time Integrator1'
   *  Product: '<S515>/Product'
   *  Product: '<S515>/Product1'
   *  Product: '<S515>/Product2'
   *  Product: '<S515>/Product3'
   *  Sum: '<S515>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt
    (((SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] *
       SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] +
       SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] *
       SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1]) +
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] *
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2]) +
     SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] *
     SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3]);

  /* Product: '<S487>/Product' incorporates:
   *  DiscreteIntegrator: '<S471>/Discrete-Time Integrator1'
   */
  rtb_Product = SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] /
    rtb_RhhcosphicoslambXe;

  /* Product: '<S487>/Product1' incorporates:
   *  DiscreteIntegrator: '<S471>/Discrete-Time Integrator1'
   */
  rtb_Product1_gz = SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] /
    rtb_RhhcosphicoslambXe;

  /* Product: '<S487>/Product2' incorporates:
   *  DiscreteIntegrator: '<S471>/Discrete-Time Integrator1'
   */
  rtb_Product2_noi = SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] /
    rtb_RhhcosphicoslambXe;

  /* Product: '<S487>/Product3' incorporates:
   *  DiscreteIntegrator: '<S471>/Discrete-Time Integrator1'
   */
  rtb_Product3_ap = SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] /
    rtb_RhhcosphicoslambXe;

  /* Sqrt: '<S527>/sqrt' incorporates:
   *  Product: '<S528>/Product'
   *  Product: '<S528>/Product1'
   *  Product: '<S528>/Product2'
   *  Product: '<S528>/Product3'
   *  Sum: '<S528>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt(((rtb_Product * rtb_Product +
    rtb_Product1_gz * rtb_Product1_gz) + rtb_Product2_noi * rtb_Product2_noi) +
    rtb_Product3_ap * rtb_Product3_ap);

  /* Product: '<S526>/Product' */
  rtb_Deg2R1 = rtb_Product / rtb_RhhcosphicoslambXe;

  /* Product: '<S526>/Product1' */
  rtb_RhhcosphisinlambYe = rtb_Product1_gz / rtb_RhhcosphicoslambXe;

  /* Product: '<S526>/Product2' */
  rtb_jxi = rtb_Product2_noi / rtb_RhhcosphicoslambXe;

  /* Product: '<S526>/Product3' */
  rtb_RhhcosphicoslambXe = rtb_Product3_ap / rtb_RhhcosphicoslambXe;

  /* Sum: '<S516>/Sum' incorporates:
   *  Product: '<S516>/Product'
   *  Product: '<S516>/Product1'
   *  Product: '<S516>/Product2'
   *  Product: '<S516>/Product3'
   */
  SLUGS2_B.VectorConcatenate[0] = ((rtb_Deg2R1 * rtb_Deg2R1 +
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S519>/Gain' incorporates:
   *  Product: '<S519>/Product2'
   *  Product: '<S519>/Product3'
   *  Sum: '<S519>/Sum'
   */
  SLUGS2_B.VectorConcatenate[1] = (rtb_RhhcosphisinlambYe * rtb_jxi -
    rtb_RhhcosphicoslambXe * rtb_Deg2R1) * 2.0F;

  /* Gain: '<S522>/Gain' incorporates:
   *  Product: '<S522>/Product1'
   *  Product: '<S522>/Product2'
   *  Sum: '<S522>/Sum'
   */
  SLUGS2_B.VectorConcatenate[2] = (rtb_Deg2R1 * rtb_jxi + rtb_RhhcosphisinlambYe
    * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Gain: '<S517>/Gain' incorporates:
   *  Product: '<S517>/Product2'
   *  Product: '<S517>/Product3'
   *  Sum: '<S517>/Sum'
   */
  SLUGS2_B.VectorConcatenate[3] = (rtb_RhhcosphicoslambXe * rtb_Deg2R1 +
    rtb_RhhcosphisinlambYe * rtb_jxi) * 2.0F;

  /* Sum: '<S520>/Sum' incorporates:
   *  Product: '<S520>/Product'
   *  Product: '<S520>/Product1'
   *  Product: '<S520>/Product2'
   *  Product: '<S520>/Product3'
   */
  SLUGS2_B.VectorConcatenate[4] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) + rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S523>/Gain' incorporates:
   *  Product: '<S523>/Product1'
   *  Product: '<S523>/Product2'
   *  Sum: '<S523>/Sum'
   */
  SLUGS2_B.VectorConcatenate[5] = (rtb_jxi * rtb_RhhcosphicoslambXe - rtb_Deg2R1
    * rtb_RhhcosphisinlambYe) * 2.0F;

  /* Gain: '<S518>/Gain' incorporates:
   *  Product: '<S518>/Product1'
   *  Product: '<S518>/Product2'
   *  Sum: '<S518>/Sum'
   */
  SLUGS2_B.VectorConcatenate[6] = (rtb_RhhcosphisinlambYe *
    rtb_RhhcosphicoslambXe - rtb_Deg2R1 * rtb_jxi) * 2.0F;

  /* Gain: '<S521>/Gain' incorporates:
   *  Product: '<S521>/Product1'
   *  Product: '<S521>/Product2'
   *  Sum: '<S521>/Sum'
   */
  SLUGS2_B.VectorConcatenate[7] = (rtb_Deg2R1 * rtb_RhhcosphisinlambYe + rtb_jxi
    * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Sum: '<S524>/Sum' incorporates:
   *  Product: '<S524>/Product'
   *  Product: '<S524>/Product1'
   *  Product: '<S524>/Product2'
   *  Product: '<S524>/Product3'
   */
  SLUGS2_B.VectorConcatenate[8] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) +
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S471>/Submatrix1' */
  yIdx = 0L;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    SLUGS2_B.Submatrix1[yIdx] = SLUGS2_B.VectorConcatenate[(int32_T)(colIdx * 3)];
    yIdx++;
  }

  /* End of S-Function (sdspsubmtrx): '<S471>/Submatrix1' */

  /* MATLAB Function: '<S471>/myMux Fun2' incorporates:
   *  Constant: '<S471>/Constant1'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/myMux Fun2': '<S489>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S489>:1:5' */
  rtb_y_ox[0] = SLUGS2_B.Submatrix1[0];
  rtb_y_ox[1] = SLUGS2_B.Submatrix1[1];
  rtb_y_ox[2] = 0.0F;

  /* MATLAB Function: '<S511>/Embedded MATLAB Function' */
  S_EmbeddedMATLABFunction_o(rtb_y_ox, &SLUGS2_B.sf_EmbeddedMATLABFunction_a);

  /* MATLAB Function: '<S510>/negprotect' */
  SLUGS2_negprotect(SLUGS2_B.sf_EmbeddedMATLABFunction_a.xDoty,
                    &SLUGS2_B.sf_negprotect_m);

  /* S-Function (MCHP_C_function_Call): '<S510>/[apUtils.c]' */
  SLUGS2_B.apUtilsc = mySqrt(
    SLUGS2_B.sf_negprotect_m.zpVal
    );

  /* Saturate: '<S486>/Zero Bound' */
  if (SLUGS2_B.apUtilsc <= 0.001F) {
    rtb_RhhcosphicoslambXe = 0.001F;
  } else {
    rtb_RhhcosphicoslambXe = SLUGS2_B.apUtilsc;
  }

  /* End of Saturate: '<S486>/Zero Bound' */

  /* Product: '<S486>/Divide' incorporates:
   *  MATLAB Function: '<S471>/myMux Fun2'
   */
  rtb_y_ox[0] = SLUGS2_B.Submatrix1[0] / rtb_RhhcosphicoslambXe;
  rtb_y_e = SLUGS2_B.Submatrix1[1] / rtb_RhhcosphicoslambXe;
  rtb_y_ox[2] = 0.0F / rtb_RhhcosphicoslambXe;

  /* S-Function (MCHP_C_function_Call): '<S12>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &SLUGS2_B.GettheGSLocationupdateSensorM_c[0]
                );

  /* Gain: '<S540>/Deg2R' */
  rtb_Deg2R_idx_0 = 0.0174532924F * SLUGS2_B.GettheGSLocationupdateSensorM_c[2];
  rtb_Deg2R_idx_1 = 0.0174532924F * SLUGS2_B.GettheGSLocationupdateSensorM_c[1];

  /* Gain: '<S541>/Deg2R' */
  rtb_Sum_bf = 0.0174532924F * SLUGS2_B.Switch1[0];

  /* Trigonometry: '<S541>/sin(phi)' */
  rtb_kxi = (real32_T)sin(rtb_Sum_bf);

  /* Sum: '<S541>/Sum1' incorporates:
   *  Constant: '<S541>/const'
   *  Product: '<S541>/Product1'
   *  Product: '<S541>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_kxi * rtb_kxi * 0.00669425726F;

  /* Fcn: '<S541>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S541>/f' */

  /* Product: '<S541>/Rh' incorporates:
   *  Constant: '<S541>/Re=equatorial radius'
   */
  rtb_ixj = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S541>/Sum2' */
  rtb_kxj = SLUGS2_B.Switch1[2] + rtb_ixj;

  /* Trigonometry: '<S541>/cos(phi)' */
  rtb_Sum_bf = (real32_T)cos(rtb_Sum_bf);

  /* Gain: '<S541>/Deg2R1' */
  rtb_ixk = 0.0174532924F * SLUGS2_B.Switch1[1];

  /* Product: '<S541>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S541>/cos(lamb)'
   */
  rtb_jxi = rtb_kxj * rtb_Sum_bf * (real32_T)cos(rtb_ixk);

  /* Product: '<S541>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S541>/sin(lamb)'
   */
  rtb_kxj = rtb_kxj * rtb_Sum_bf * (real32_T)sin(rtb_ixk);

  /* Product: '<S541>/Ze' incorporates:
   *  Product: '<S541>/Rh(1-e^2)'
   *  Sum: '<S541>/Sum4'
   */
  rtb_kxi *= 0.993305743F * rtb_ixj + SLUGS2_B.Switch1[2];

  /* Gain: '<S532>/Deg2R' */
  rtb_Sum_bf = 0.0174532924F * SLUGS2_B.GettheGSLocationupdateSensorM_c[1];

  /* Trigonometry: '<S532>/sin(phi)' */
  rtb_ixj = (real32_T)sin(rtb_Sum_bf);

  /* Sum: '<S532>/Sum1' incorporates:
   *  Constant: '<S532>/const'
   *  Product: '<S532>/Product1'
   *  Product: '<S532>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_ixj * rtb_ixj * 0.00669425726F;

  /* Fcn: '<S532>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S532>/f' */

  /* Product: '<S532>/Rh' incorporates:
   *  Constant: '<S532>/Re=equatorial radius'
   */
  rtb_ixk = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S532>/Sum2' */
  rtb_RhhcosphisinlambYe = SLUGS2_B.GettheGSLocationupdateSensorM_c[0] + rtb_ixk;

  /* Trigonometry: '<S532>/cos(phi)' */
  rtb_Sum_bf = (real32_T)cos(rtb_Sum_bf);

  /* Gain: '<S532>/Deg2R1' */
  rtb_Deg2R1 = 0.0174532924F * SLUGS2_B.GettheGSLocationupdateSensorM_c[2];

  /* Product: '<S532>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S532>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_Sum_bf * (real32_T)cos
    (rtb_Deg2R1);

  /* Product: '<S532>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S532>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_Sum_bf * (real32_T)sin
    (rtb_Deg2R1);

  /* Product: '<S532>/Ze' incorporates:
   *  Product: '<S532>/Rh(1-e^2)'
   *  Sum: '<S532>/Sum4'
   */
  rtb_ixj *= 0.993305743F * rtb_ixk + SLUGS2_B.GettheGSLocationupdateSensorM_c[0];

  /* S-Function (MCHP_C_function_Call): '<S12>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  SLUGS2_B.ChecksifFixTypeis3updateSenso_p = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S474>/Enable'
   */
#ifndef WIN
  if (SLUGS2_B.ChecksifFixTypeis3updateSenso_p > 0) {
#endif
    /* SignalConversion: '<S540>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S543>/11'
     *  Fcn: '<S543>/12'
     *  Fcn: '<S543>/13'
     *  Fcn: '<S543>/21'
     *  Fcn: '<S543>/22'
     *  Fcn: '<S543>/23'
     *  Fcn: '<S543>/31'
     *  Fcn: '<S543>/32'
     *  Fcn: '<S543>/33'
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

    /* Sum: '<S476>/Sum1' */
    rtb_ixk = rtb_jxi - rtb_RhhcosphicoslambXe;
    rtb_Deg2R1 = rtb_kxj - rtb_RhhcosphisinlambYe;
    rtb_RhhcosphicoslambXe = rtb_kxi - rtb_ixj;

    /* Product: '<S540>/Product1' incorporates:
     *  Gain: '<S476>/UEN 2 NEU'
     */
    for (colIdx = 0; colIdx < 3; colIdx++) {
      tmp_0[colIdx] = tmp[colIdx + 6] * rtb_RhhcosphicoslambXe + (tmp[colIdx + 3]
        * rtb_Deg2R1 + tmp[colIdx] * rtb_ixk);
    }

    /* End of Product: '<S540>/Product1' */

    /* Inport: '<S474>/In1' */
    for (colIdx = 0; colIdx < 3; colIdx++) {
      /* Gain: '<S476>/UEN 2 NEU' */
      SLUGS2_B.In1[colIdx] = 0.0F;
      SLUGS2_B.In1[colIdx] += SLUGS2_ConstP.pooled64[colIdx] * tmp_0[0];
      SLUGS2_B.In1[colIdx] += SLUGS2_ConstP.pooled64[colIdx + 3] * tmp_0[1];
      SLUGS2_B.In1[colIdx] += SLUGS2_ConstP.pooled64[colIdx + 6] * tmp_0[2];
    }

    /* End of Inport: '<S474>/In1' */

    /* Inport: '<S474>/In2' */
    SLUGS2_B.In2 = SLUGS2_B.Switch1[3];

    /* Inport: '<S474>/In3' */
    SLUGS2_B.In3 = SLUGS2_B.Switch1[4];
#ifndef WIN
  }
#endif

  /* End of Outputs for SubSystem: '<S12>/Enabled Subsystem' */

  /* Gain: '<S529>/Unit Conversion' */
  SLUGS2_B.UnitConversion = 0.0174532924F * SLUGS2_B.In2;

  /* S-Function (MCHP_C_function_Call): '<S530>/[apUtils.c]1' */
  SLUGS2_B.apUtilsc1 = myCos(
    SLUGS2_B.UnitConversion
    );

  /* S-Function (MCHP_C_function_Call): '<S530>/[apUtils.c]' */
  SLUGS2_B.apUtilsc_a = mySin(
    SLUGS2_B.UnitConversion
    );

  /* Product: '<S472>/Product1' */
  /* MATLAB Function 'Position_and_Attitude_Filter/COG.SOG2V/myMux Fun2': '<S531>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S531>:1:5' */
  rtb_Deg2R_idx_0 = SLUGS2_B.In3 * SLUGS2_B.apUtilsc1;

  /* Product: '<S472>/Product' */
  rtb_Deg2R_idx_1 = SLUGS2_B.apUtilsc_a * SLUGS2_B.In3;

  /* Abs: '<S534>/Abs1' incorporates:
   *  Sum: '<S537>/Diff'
   *  UnitDelay: '<S537>/UD'
   *
   * Block description for '<S537>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S537>/UD':
   *
   *  Store in Global RAM
   */
  rtb_ixk = (real32_T)fabs(SLUGS2_B.In1[0] - SLUGS2_DWork.UD_DSTATE);

  /* Abs: '<S535>/Abs1' incorporates:
   *  Sum: '<S538>/Diff'
   *  UnitDelay: '<S538>/UD'
   *
   * Block description for '<S538>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S538>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Deg2R1 = (real32_T)fabs(SLUGS2_B.In1[1] - SLUGS2_DWork.UD_DSTATE_o);

  /* Abs: '<S536>/Abs1' incorporates:
   *  Sum: '<S539>/Diff'
   *  UnitDelay: '<S539>/UD'
   *
   * Block description for '<S539>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S539>/UD':
   *
   *  Store in Global RAM
   */
  rtb_RhhcosphicoslambXe = (real32_T)fabs(SLUGS2_B.In1[2] -
    SLUGS2_DWork.UD_DSTATE_o2);

  /* Saturate: '<S534>/Saturation1' */
  if (rtb_ixk > 1.0F) {
    rtb_ixk = 1.0F;
  }

  /* Saturate: '<S535>/Saturation1' */
  if (rtb_Deg2R1 > 1.0F) {
    rtb_Deg2R1 = 1.0F;
  }

  /* Saturate: '<S536>/Saturation1' */
  if (rtb_RhhcosphicoslambXe > 1.0F) {
    rtb_RhhcosphicoslambXe = 1.0F;
  }

  /* Sum: '<S475>/Sum' incorporates:
   *  Saturate: '<S534>/Saturation1'
   *  Saturate: '<S535>/Saturation1'
   *  Saturate: '<S536>/Saturation1'
   */
  rtb_ixk = (rtb_ixk + rtb_Deg2R1) + rtb_RhhcosphicoslambXe;

  /* Saturate: '<S475>/Saturation1' */
  if (rtb_ixk > 1.0F) {
    rtb_ixk = 1.0F;
  }

  /* DataTypeConversion: '<S475>/Data Type Conversion2' incorporates:
   *  Saturate: '<S475>/Saturation1'
   */
  rtb_RhhcosphicoslambXe = (real32_T)floor(rtb_ixk);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_DataTypeConversion2_h = 0U;
  } else {
    rtb_DataTypeConversion2_h = (uint8_T)(real32_T)fmod(rtb_RhhcosphicoslambXe,
      256.0F);
  }

  /* End of DataTypeConversion: '<S475>/Data Type Conversion2' */

  /* Outputs for Enabled SubSystem: '<S545>/Subsystem' incorporates:
   *  EnablePort: '<S553>/Enable'
   */
  /* RelationalOperator: '<S551>/Compare' incorporates:
   *  Constant: '<S551>/Constant'
   */
  if ((SLUGS2_B.In3 > 7.0F) > 0) {
    if (!SLUGS2_DWork.Subsystem_MODE) {
      SLUGS2_DWork.Subsystem_MODE = true;
    }

    /* MATLAB Function: '<S553>/Embedded MATLAB Function1' */
    SL_EmbeddedMATLABFunction1(rtb_Deg2R_idx_1, rtb_DataTypeConversion2_h,
      &SLUGS2_B.sf_EmbeddedMATLABFunction1_k,
      &SLUGS2_DWork.sf_EmbeddedMATLABFunction1_k);

    /* MATLAB Function: '<S553>/Embedded MATLAB Function2' */
    SL_EmbeddedMATLABFunction1(rtb_Deg2R_idx_0, rtb_DataTypeConversion2_h,
      &SLUGS2_B.sf_EmbeddedMATLABFunction2,
      &SLUGS2_DWork.sf_EmbeddedMATLABFunction2);
  } else {
    if (SLUGS2_DWork.Subsystem_MODE) {
      /* Disable for Outport: '<S553>/Vn_fil' */
      SLUGS2_B.sf_EmbeddedMATLABFunction2.y = 0.0F;

      /* Disable for Outport: '<S553>/Ve_fil' */
      SLUGS2_B.sf_EmbeddedMATLABFunction1_k.y = 0.0F;
      SLUGS2_DWork.Subsystem_MODE = false;
    }
  }

  /* End of RelationalOperator: '<S551>/Compare' */
  /* End of Outputs for SubSystem: '<S545>/Subsystem' */

  /* MATLAB Function: '<S546>/Embedded MATLAB Function3' incorporates:
   *  Gain: '<S12>/[1 1 -1]'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3': '<S556>:1' */
  /*  persistent lastH; */
  if (!SLUGS2_DWork.lastGps_h_not_empty) {
    /* '<S556>:1:7' */
    SLUGS2_DWork.lastGps_h_not_empty = true;

    /* '<S556>:1:9' */
    SLUGS2_DWork.TimeSinceLast = 0.01F;

    /*      lastH           = single(-baseHeight); */
    /*      h_rate          = single(0); */
  }

  if (rtb_DataTypeConversion2_h != 0) {
    /* '<S556>:1:16' */
    rtb_RhhcosphicoslambXe = (-SLUGS2_B.In1[2] - SLUGS2_DWork.lastGps_h) /
      SLUGS2_DWork.TimeSinceLast;

    /* '<S556>:1:17' */
    SLUGS2_DWork.TimeSinceLast = 0.0F;
    if (!((rtb_RhhcosphicoslambXe > -10.6) && (rtb_RhhcosphicoslambXe < 17.67)))
    {
      /*          h = lastH; */
      /* '<S556>:1:23' */
      rtb_RhhcosphicoslambXe = 0.0F;
    } else {
      /* '<S556>:1:18' */
      /*          h = lastH + gps_h - lastGps_h; */
      /* '<S556>:1:20' */
    }

    /* '<S556>:1:25' */
    SLUGS2_DWork.lastGps_h = -SLUGS2_B.In1[2];
  } else {
    /*      h = lastH; */
    /* '<S556>:1:28' */
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* '<S556>:1:31' */
  SLUGS2_DWork.TimeSinceLast += 0.01F;

  /* End of MATLAB Function: '<S546>/Embedded MATLAB Function3' */

  /* MATLAB Function: '<S557>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S557>/Constant'
   *  Constant: '<S557>/Constant1'
   *  Gain: '<S545>/Gain'
   *  Gain: '<S546>/Gain'
   *  Sum: '<S546>/Sum'
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
  SLU_EmbeddedMATLABFunction(0.5F * rtb_RhhcosphicoslambXe +
    -SLUGS2_B.EnabledSubsystem_m.In1, 0.01, 0.31830988618379069,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_ph,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_ph);

  /* Gain: '<S550>/Gain' incorporates:
   *  Sum: '<S550>/Sum1'
   *  UnitDelay: '<S550>/Unit Delay'
   */
  rtb_jxi = (SLUGS2_B.sf_EmbeddedMATLABFunction_ph.y -
             SLUGS2_DWork.UnitDelay_DSTATE) * 2.0F;

  /* MATLAB Function: '<S545>/myMux Fun2' */
  SLUGS2_myMuxFun1_e(SLUGS2_B.sf_EmbeddedMATLABFunction2.y,
                     SLUGS2_B.sf_EmbeddedMATLABFunction1_k.y, rtb_jxi,
                     &SLUGS2_B.sf_myMuxFun2_a);

  /* Product: '<S471>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_Product1_e[colIdx] = SLUGS2_B.VectorConcatenate[colIdx + 6] *
      SLUGS2_B.sf_myMuxFun2_a.y[2] + (SLUGS2_B.VectorConcatenate[colIdx + 3] *
      SLUGS2_B.sf_myMuxFun2_a.y[1] + SLUGS2_B.VectorConcatenate[colIdx] *
      SLUGS2_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S471>/Product1' */

  /* Product: '<S545>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_Product1_kx[colIdx] = SLUGS2_B.VectorConcatenate[colIdx + 6] *
      SLUGS2_B.sf_myMuxFun2_a.y[2] + (SLUGS2_B.VectorConcatenate[colIdx + 3] *
      SLUGS2_B.sf_myMuxFun2_a.y[1] + SLUGS2_B.VectorConcatenate[colIdx] *
      SLUGS2_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S545>/Product1' */

  /* Sum: '<S545>/Sum' incorporates:
   *  Delay: '<S552>/Integer Delay1'
   *  Gain: '<S552>/Gain2'
   *  Sum: '<S552>/Sum5'
   */
  rtb_RhhcosphicoslambXe = SLUGS2_B.DataTypeConversion3[0] - (rtb_Product1_kx[0]
    - SLUGS2_DWork.IntegerDelay1_DSTATE_g[0]) * 20.0F;
  rtb_Deg2R1 = SLUGS2_B.DataTypeConversion3[1] - (rtb_Product1_kx[1] -
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[1]) * 20.0F;
  rtb_ixk = SLUGS2_B.DataTypeConversion3[2] - (rtb_Product1_kx[2] -
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[2]) * 20.0F;

  /* RateLimiter: '<S545>/Rate Limiter' */
  rtb_Add[0] = rtb_RhhcosphicoslambXe - SLUGS2_DWork.PrevY[0];
  rtb_Add[1] = rtb_Deg2R1 - SLUGS2_DWork.PrevY[1];
  rtb_Add[2] = rtb_ixk - SLUGS2_DWork.PrevY[2];
  rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe;
  if (rtb_Add[0] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[0] + 9.896E-9F;
  } else {
    if (rtb_Add[0] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[0] + -9.896E-9F;
    }
  }

  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe;
  rtb_RhhcosphisinlambYe = rtb_Deg2R1;
  if (rtb_Add[1] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[1] + 9.896E-9F;
  } else {
    if (rtb_Add[1] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[1] + -9.896E-9F;
    }
  }

  rtb_Deg2R1 = rtb_RhhcosphisinlambYe;
  rtb_RhhcosphisinlambYe = rtb_ixk;
  if (rtb_Add[2] > 9.896E-9F) {
    rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[2] + 9.896E-9F;
  } else {
    if (rtb_Add[2] < -9.896E-9F) {
      rtb_RhhcosphisinlambYe = SLUGS2_DWork.PrevY[2] + -9.896E-9F;
    }
  }

  SLUGS2_DWork.PrevY[0] = rtb_RhhcosphicoslambXe;
  SLUGS2_DWork.PrevY[1] = rtb_Deg2R1;
  SLUGS2_DWork.PrevY[2] = rtb_RhhcosphisinlambYe;

  /* End of RateLimiter: '<S545>/Rate Limiter' */

  /* DataTypeConversion: '<S547>/Data Type Conversion2' */
  rtb_DataTypeConversion2_g[0] = rtb_RhhcosphicoslambXe;
  rtb_DataTypeConversion2_g[1] = rtb_Deg2R1;
  rtb_DataTypeConversion2_g[2] = rtb_RhhcosphisinlambYe;

  /* DiscreteZeroPole: '<S559>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_f = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[0];
    rtb_DiscreteZeroPole_f += 3.9156825028515473E-10*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* DiscreteZeroPole: '<S560>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_c = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[1];
    rtb_DiscreteZeroPole_c += 3.9156825028515473E-10*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_j;
  }

  /* DiscreteZeroPole: '<S561>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ck = 9.8940417712526327E-7*rtb_DataTypeConversion2_g[2];
    rtb_DiscreteZeroPole_ck += 3.9156825028515473E-10*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* MATLAB Function: '<S547>/myMux Fun1' */
  SLUGS2_myMuxFun1(rtb_DiscreteZeroPole_f, rtb_DiscreteZeroPole_c,
                   rtb_DiscreteZeroPole_ck, &SLUGS2_B.sf_myMuxFun1_d);

  /* DataTypeConversion: '<S547>/Data Type Conversion1' */
  SLUGS2_B.DataTypeConversion1[0] = (real32_T)SLUGS2_B.sf_myMuxFun1_d.y[0];
  SLUGS2_B.DataTypeConversion1[1] = (real32_T)SLUGS2_B.sf_myMuxFun1_d.y[1];
  SLUGS2_B.DataTypeConversion1[2] = (real32_T)SLUGS2_B.sf_myMuxFun1_d.y[2];

  /* Sum: '<S471>/Add' incorporates:
   *  Delay: '<S471>/Integer Delay'
   *  Delay: '<S482>/Integer Delay1'
   *  Gain: '<S482>/Gain2'
   *  Product: '<S497>/i x j'
   *  Product: '<S497>/j x k'
   *  Product: '<S497>/k x i'
   *  Product: '<S498>/i x k'
   *  Product: '<S498>/j x i'
   *  Product: '<S498>/k x j'
   *  Sum: '<S480>/Sum'
   *  Sum: '<S482>/Sum5'
   */
  rtb_Add[0] = (((SLUGS2_DWork.IntegerDelay_DSTATE_j[1] * rtb_Product1_e[2] -
                  SLUGS2_DWork.IntegerDelay_DSTATE_j[2] * rtb_Product1_e[1]) +
                 (rtb_Product1_e[0] - SLUGS2_DWork.IntegerDelay1_DSTATE[0]) *
                 20.0F) - SLUGS2_B.DataTypeConversion3[0]) -
    SLUGS2_B.DataTypeConversion1[0];
  rtb_Add[1] = (((SLUGS2_DWork.IntegerDelay_DSTATE_j[2] * rtb_Product1_e[0] -
                  SLUGS2_DWork.IntegerDelay_DSTATE_j[0] * rtb_Product1_e[2]) +
                 (rtb_Product1_e[1] - SLUGS2_DWork.IntegerDelay1_DSTATE[1]) *
                 20.0F) - SLUGS2_B.DataTypeConversion3[1]) -
    SLUGS2_B.DataTypeConversion1[1];
  rtb_Add[2] = (((SLUGS2_DWork.IntegerDelay_DSTATE_j[0] * rtb_Product1_e[1] -
                  SLUGS2_DWork.IntegerDelay_DSTATE_j[1] * rtb_Product1_e[0]) +
                 (rtb_Product1_e[2] - SLUGS2_DWork.IntegerDelay1_DSTATE[2]) *
                 20.0F) - SLUGS2_B.DataTypeConversion3[2]) -
    SLUGS2_B.DataTypeConversion1[2];

  /* MATLAB Function: '<S506>/Embedded MATLAB Function' */
  S_EmbeddedMATLABFunction_o(rtb_Add, &SLUGS2_B.sf_EmbeddedMATLABFunction_op);

  /* MATLAB Function: '<S505>/negprotect' */
  SLUGS2_negprotect(SLUGS2_B.sf_EmbeddedMATLABFunction_op.xDoty,
                    &SLUGS2_B.sf_negprotect_k);

  /* S-Function (MCHP_C_function_Call): '<S505>/[apUtils.c]' */
  SLUGS2_B.apUtilsc_f = mySqrt(
    SLUGS2_B.sf_negprotect_k.zpVal
    );

  /* Saturate: '<S485>/Zero Bound' */
  if (SLUGS2_B.apUtilsc_f <= 0.001F) {
    rtb_RhhcosphicoslambXe = 0.001F;
  } else {
    rtb_RhhcosphicoslambXe = SLUGS2_B.apUtilsc_f;
  }

  /* End of Saturate: '<S485>/Zero Bound' */

  /* Product: '<S485>/Divide' */
  rtb_Add[0] /= rtb_RhhcosphicoslambXe;
  rtb_Add[1] /= rtb_RhhcosphicoslambXe;
  rtb_RhhcosphisinlambYe = rtb_Add[2] / rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S471>/Submatrix' */
  yIdx = 0L;
  colIdx = 2;
  while (colIdx <= 2) {
    SLUGS2_B.g_hat[yIdx] = SLUGS2_B.VectorConcatenate[6L];
    SLUGS2_B.g_hat[1L + yIdx] = SLUGS2_B.VectorConcatenate[7L];
    SLUGS2_B.g_hat[2L + yIdx] = SLUGS2_B.VectorConcatenate[8L];
    yIdx += 3L;
    colIdx = 3;
  }

  /* End of S-Function (sdspsubmtrx): '<S471>/Submatrix' */

  /* Sum: '<S481>/Sum' incorporates:
   *  MATLAB Function: '<S472>/myMux Fun2'
   *  Product: '<S499>/i x j'
   *  Product: '<S499>/j x k'
   *  Product: '<S499>/k x i'
   *  Product: '<S500>/i x k'
   *  Product: '<S500>/j x i'
   *  Product: '<S500>/k x j'
   */
  rtb_Deg2R1 = rtb_y_e * 0.0F - rtb_y_ox[2] * SLUGS2_B.apUtilsc_a;
  rtb_ixk = rtb_y_ox[2] * SLUGS2_B.apUtilsc1 - rtb_y_ox[0] * 0.0F;
  rtb_RhhcosphicoslambXe = rtb_y_ox[0] * SLUGS2_B.apUtilsc_a - rtb_y_e *
    SLUGS2_B.apUtilsc1;

  /* Product: '<S471>/Product2' incorporates:
   *  Gain: '<S471>/Gain1'
   */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp_0[colIdx] = SLUGS2_B.VectorConcatenate[colIdx + 6] *
      rtb_RhhcosphicoslambXe + (SLUGS2_B.VectorConcatenate[colIdx + 3] * rtb_ixk
      + SLUGS2_B.VectorConcatenate[colIdx] * rtb_Deg2R1);
  }

  /* End of Product: '<S471>/Product2' */

  /* Product: '<S495>/i x j' */
  rtb_RhhcosphicoslambXe = rtb_Add[0];

  /* Product: '<S496>/i x k' */
  rtb_Deg2R1 = rtb_Add[0];

  /* Product: '<S496>/j x i' */
  rtb_ixk = rtb_Add[1];

  /* Sum: '<S471>/Sum4' incorporates:
   *  Gain: '<S471>/Gain1'
   *  Product: '<S495>/i x j'
   *  Product: '<S495>/j x k'
   *  Product: '<S495>/k x i'
   *  Product: '<S496>/i x k'
   *  Product: '<S496>/j x i'
   *  Product: '<S496>/k x j'
   *  Sum: '<S479>/Sum'
   */
  rtb_Add[0] = (rtb_Add[1] * SLUGS2_B.g_hat[2] - rtb_RhhcosphisinlambYe *
                SLUGS2_B.g_hat[1]) + 30.0F * tmp_0[0];
  rtb_Add[1] = (rtb_RhhcosphisinlambYe * SLUGS2_B.g_hat[0] - rtb_Deg2R1 *
                SLUGS2_B.g_hat[2]) + 30.0F * tmp_0[1];
  rtb_Add[2] = (rtb_RhhcosphicoslambXe * SLUGS2_B.g_hat[1] - rtb_ixk *
                SLUGS2_B.g_hat[0]) + 30.0F * tmp_0[2];

  /* DataTypeConversion: '<S478>/Data Type Conversion2' */
  rtb_DataTypeConversion2_e2[0] = rtb_Add[0];
  rtb_DataTypeConversion2_e2[1] = rtb_Add[1];
  rtb_DataTypeConversion2_e2[2] = rtb_Add[2];

  /* DiscreteZeroPole: '<S491>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_k = 0.005*rtb_DataTypeConversion2_e2[0];
    rtb_DiscreteZeroPole_k += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* DiscreteZeroPole: '<S492>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_o = 0.005*rtb_DataTypeConversion2_e2[1];
    rtb_DiscreteZeroPole_o += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* DiscreteZeroPole: '<S493>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_g = 0.005*rtb_DataTypeConversion2_e2[2];
    rtb_DiscreteZeroPole_g += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* MATLAB Function: '<S478>/myMux Fun1' */
  SLUGS2_myMuxFun1(rtb_DiscreteZeroPole_k, rtb_DiscreteZeroPole_o,
                   rtb_DiscreteZeroPole_g, &SLUGS2_B.sf_myMuxFun1_n);

  /* RateLimiter: '<S471>/Bias Rate Limiter' incorporates:
   *  DataTypeConversion: '<S478>/Data Type Conversion1'
   */
  SLUGS2_B.BiasRateLimiter[0] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[0] -
    SLUGS2_DWork.PrevY_n[0];
  SLUGS2_B.BiasRateLimiter[1] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[1] -
    SLUGS2_DWork.PrevY_n[1];
  SLUGS2_B.BiasRateLimiter[2] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[2] -
    SLUGS2_DWork.PrevY_n[2];
  if (SLUGS2_B.BiasRateLimiter[0] > 2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[0] = SLUGS2_DWork.PrevY_n[0] + 2.5572E-9F;
  } else if (SLUGS2_B.BiasRateLimiter[0] < -2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[0] = SLUGS2_DWork.PrevY_n[0] + -2.5572E-9F;
  } else {
    SLUGS2_B.BiasRateLimiter[0] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[0];
  }

  if (SLUGS2_B.BiasRateLimiter[1] > 2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[1] = SLUGS2_DWork.PrevY_n[1] + 2.5572E-9F;
  } else if (SLUGS2_B.BiasRateLimiter[1] < -2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[1] = SLUGS2_DWork.PrevY_n[1] + -2.5572E-9F;
  } else {
    SLUGS2_B.BiasRateLimiter[1] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[1];
  }

  if (SLUGS2_B.BiasRateLimiter[2] > 2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[2] = SLUGS2_DWork.PrevY_n[2] + 2.5572E-9F;
  } else if (SLUGS2_B.BiasRateLimiter[2] < -2.5572E-9F) {
    SLUGS2_B.BiasRateLimiter[2] = SLUGS2_DWork.PrevY_n[2] + -2.5572E-9F;
  } else {
    SLUGS2_B.BiasRateLimiter[2] = (real32_T)SLUGS2_B.sf_myMuxFun1_n.y[2];
  }

  SLUGS2_DWork.PrevY_n[0] = SLUGS2_B.BiasRateLimiter[0];
  SLUGS2_DWork.PrevY_n[1] = SLUGS2_B.BiasRateLimiter[1];
  SLUGS2_DWork.PrevY_n[2] = SLUGS2_B.BiasRateLimiter[2];

  /* End of RateLimiter: '<S471>/Bias Rate Limiter' */

  /* Gain: '<S503>/Gain1' incorporates:
   *  Selector: '<S503>/Selector1'
   */
  rtb_VectorConcatenate[0] = SLUGS2_B.VectorConcatenate[3];
  rtb_VectorConcatenate[1] = SLUGS2_B.VectorConcatenate[0];
  rtb_VectorConcatenate[2] = -SLUGS2_B.VectorConcatenate[6];

  /* Gain: '<S503>/Gain2' incorporates:
   *  Selector: '<S503>/Selector2'
   */
  rtb_VectorConcatenate[3] = SLUGS2_B.VectorConcatenate[7];
  rtb_VectorConcatenate[4] = SLUGS2_B.VectorConcatenate[8];

  /* Gain: '<S503>/Gain3' incorporates:
   *  Selector: '<S503>/Selector3'
   */
  rtb_VectorConcatenate[5] = -SLUGS2_B.VectorConcatenate[1];
  rtb_VectorConcatenate[6] = SLUGS2_B.VectorConcatenate[4];

  /* If: '<S483>/If' incorporates:
   *  Gain: '<S503>/Gain1'
   *  Selector: '<S503>/Selector1'
   */
  if ((-SLUGS2_B.VectorConcatenate[6] >= 1.0F) || (-SLUGS2_B.VectorConcatenate[6]
       <= -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S483>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S502>/Action Port'
     */
    SLUGS2_AxisRotZeroR3(rtb_VectorConcatenate, &SLUGS2_B.DataTypeConversion_m[0],
                         &SLUGS2_B.DataTypeConversion_m[1],
                         &SLUGS2_B.DataTypeConversion_m[2]);

    /* End of Outputs for SubSystem: '<S483>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S483>/AxisRotDefault' incorporates:
     *  ActionPort: '<S501>/Action Port'
     */
    SLUGS2_AxisRotDefault(rtb_VectorConcatenate, &SLUGS2_B.DataTypeConversion_m
                          [0], &SLUGS2_B.DataTypeConversion_m[1],
                          &SLUGS2_B.DataTypeConversion_m[2]);

    /* End of Outputs for SubSystem: '<S483>/AxisRotDefault' */
  }

  /* End of If: '<S483>/If' */

  /* MATLAB Function: '<S471>/Embedded MATLAB Function' */
  S_EmbeddedMATLABFunction_l(SLUGS2_B.DataTypeConversion_m[0],
    &SLUGS2_B.sf_EmbeddedMATLABFunction_lt);

  /* Sum: '<S471>/Sum3' */
  SLUGS2_B.GyroErr[0] = SLUGS2_B.DataTypeConversion7[0] +
    SLUGS2_B.BiasRateLimiter[0];
  SLUGS2_B.GyroErr[1] = SLUGS2_B.DataTypeConversion7[1] +
    SLUGS2_B.BiasRateLimiter[1];
  SLUGS2_B.GyroErr[2] = SLUGS2_B.DataTypeConversion7[2] +
    SLUGS2_B.BiasRateLimiter[2];

  /* Sum: '<S471>/Sum1' incorporates:
   *  Gain: '<S471>/Gain'
   */
  SLUGS2_DWork.IntegerDelay_DSTATE_j[0] = 0.1F * rtb_Add[0] + SLUGS2_B.GyroErr[0];
  SLUGS2_DWork.IntegerDelay_DSTATE_j[1] = 0.1F * rtb_Add[1] + SLUGS2_B.GyroErr[1];
  SLUGS2_DWork.IntegerDelay_DSTATE_j[2] = 0.1F * rtb_Add[2] + SLUGS2_B.GyroErr[2];

  /* MATLAB Function: '<S471>/q dot calc' incorporates:
   *  SignalConversion: '<S490>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/q dot calc': '<S490>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S490>:1:5' */
  /* '<S490>:1:6' */
  /* '<S490>:1:7' */
  rtb_q_dot[1] = 0.0F;
  rtb_q_dot[2] = 0.0F;
  rtb_q_dot[3] = 0.0F;

  /* '<S490>:1:8' */
  rtb_q_dot[0] = (-0.5F * rtb_Product1_gz * SLUGS2_DWork.IntegerDelay_DSTATE_j[0]
                  + -0.5F * rtb_Product2_noi *
                  SLUGS2_DWork.IntegerDelay_DSTATE_j[1]) + -0.5F *
    rtb_Product3_ap * SLUGS2_DWork.IntegerDelay_DSTATE_j[2];

  /* '<S490>:1:9' */
  rtb_TmpSignalConversionAtSFun_0[0] = rtb_Product;
  rtb_TmpSignalConversionAtSFun_0[3] = -rtb_Product3_ap;
  rtb_TmpSignalConversionAtSFun_0[6] = rtb_Product2_noi;
  rtb_TmpSignalConversionAtSFun_0[1] = rtb_Product3_ap;
  rtb_TmpSignalConversionAtSFun_0[4] = rtb_Product;
  rtb_TmpSignalConversionAtSFun_0[7] = -rtb_Product1_gz;
  rtb_TmpSignalConversionAtSFun_0[2] = -rtb_Product2_noi;
  rtb_TmpSignalConversionAtSFun_0[5] = rtb_Product1_gz;
  rtb_TmpSignalConversionAtSFun_0[8] = rtb_Product;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp[3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx] * 0.5F;
    tmp[1 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 1] * 0.5F;
    tmp[2 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 2] * 0.5F;
  }

  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_q_dot[1 + colIdx] = 0.0F;
    rtb_q_dot[1 + colIdx] += tmp[colIdx] * SLUGS2_DWork.IntegerDelay_DSTATE_j[0];
    rtb_q_dot[1 + colIdx] += tmp[colIdx + 3] *
      SLUGS2_DWork.IntegerDelay_DSTATE_j[1];
    rtb_q_dot[1 + colIdx] += tmp[colIdx + 6] *
      SLUGS2_DWork.IntegerDelay_DSTATE_j[2];
  }

  /* End of MATLAB Function: '<S471>/q dot calc' */

  /* DataTypeConversion: '<S563>/Data Type Conversion2' incorporates:
   *  Gain: '<S12>/[1 1 -1]'
   *  Gain: '<S548>/Gain'
   *  Sum: '<S548>/Sum'
   *  Sum: '<S548>/Sum1'
   *  UnitDelay: '<S548>/Unit Delay'
   */
  rtb_DataTypeConversion2_b = (SLUGS2_B.In1[0] - SLUGS2_DWork.UnitDelay_DSTATE_n)
    * 10.0F + rtb_Deg2R_idx_0;

  /* DiscreteZeroPole: '<S564>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_b;
    rtb_DiscreteZeroPole_ki += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_jl;
  }

  /* DataTypeConversion: '<S563>/Data Type Conversion1' */
  SLUGS2_DWork.UnitDelay_DSTATE_n = (real32_T)rtb_DiscreteZeroPole_ki;

  /* DataTypeConversion: '<S565>/Data Type Conversion2' incorporates:
   *  Gain: '<S12>/[1 1 -1]'
   *  Gain: '<S549>/Gain'
   *  Sum: '<S549>/Sum'
   *  Sum: '<S549>/Sum1'
   *  UnitDelay: '<S549>/Unit Delay'
   */
  rtb_DataTypeConversion2_j = (SLUGS2_B.In1[1] - SLUGS2_DWork.UnitDelay_DSTATE_j)
    * 10.0F + rtb_Deg2R_idx_1;

  /* DiscreteZeroPole: '<S566>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_j;
    rtb_DiscreteZeroPole_ki += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* DataTypeConversion: '<S565>/Data Type Conversion1' */
  SLUGS2_DWork.UnitDelay_DSTATE_j = (real32_T)rtb_DiscreteZeroPole_ki;

  /* DataTypeConversion: '<S567>/Data Type Conversion2' */
  rtb_DataTypeConversion2_gb = rtb_jxi;

  /* DiscreteZeroPole: '<S568>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_ki = 0.005*rtb_DataTypeConversion2_gb;
    rtb_DiscreteZeroPole_ki += 0.01*SLUGS2_DWork.DiscreteZeroPole_DSTATE_b;
  }

  /* DataTypeConversion: '<S567>/Data Type Conversion1' */
  SLUGS2_DWork.UnitDelay_DSTATE = (real32_T)rtb_DiscreteZeroPole_ki;

  /* MATLAB Function: '<S545>/myMux Fun1' incorporates:
   *  Gain: '<S545>/Gain1'
   */
  SLUGS2_myMuxFun1_e(SLUGS2_DWork.UnitDelay_DSTATE_n,
                     SLUGS2_DWork.UnitDelay_DSTATE_j,
                     -SLUGS2_B.sf_EmbeddedMATLABFunction_ph.y,
                     &SLUGS2_B.sf_myMuxFun1_e);

  /* Update for DiscreteIntegrator: '<S471>/Discrete-Time Integrator1' */
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] += 0.01F * rtb_q_dot[0];
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] += 0.01F * rtb_q_dot[1];
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] += 0.01F * rtb_q_dot[2];
  SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] += 0.01F * rtb_q_dot[3];
  if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] >= 1.0F) {
    SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  } else {
    if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] <= -1.0F) {
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[0] = -1.0F;
    }
  }

  if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] >= 1.0F) {
    SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 1.0F;
  } else {
    if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] <= -1.0F) {
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[1] = -1.0F;
    }
  }

  if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] >= 1.0F) {
    SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 1.0F;
  } else {
    if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] <= -1.0F) {
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[2] = -1.0F;
    }
  }

  if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] >= 1.0F) {
    SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 1.0F;
  } else {
    if (SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] <= -1.0F) {
      SLUGS2_DWork.DiscreteTimeIntegrator1_DSTATE[3] = -1.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S471>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S537>/UD'
   *
   * Block description for '<S537>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE = SLUGS2_B.In1[0];

  /* Update for UnitDelay: '<S538>/UD'
   *
   * Block description for '<S538>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE_o = SLUGS2_B.In1[1];

  /* Update for UnitDelay: '<S539>/UD'
   *
   * Block description for '<S539>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE_o2 = SLUGS2_B.In1[2];

  /* Update for Delay: '<S482>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    SLUGS2_DWork.IntegerDelay1_DSTATE[colIdx * 3] =
      SLUGS2_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3];
    SLUGS2_DWork.IntegerDelay1_DSTATE[colIdx * 3 + 1] =
      SLUGS2_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3 + 1];
    SLUGS2_DWork.IntegerDelay1_DSTATE[colIdx * 3 + 2] =
      SLUGS2_DWork.IntegerDelay1_DSTATE[(colIdx + 1) * 3 + 2];
  }

  SLUGS2_DWork.IntegerDelay1_DSTATE[12] = rtb_Product1_e[0];
  SLUGS2_DWork.IntegerDelay1_DSTATE[13] = rtb_Product1_e[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE[14] = rtb_Product1_e[2];

  /* End of Update for Delay: '<S482>/Integer Delay1' */

  /* Update for Delay: '<S552>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[colIdx * 3] =
      SLUGS2_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3];
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[colIdx * 3 + 1] =
      SLUGS2_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3 + 1];
    SLUGS2_DWork.IntegerDelay1_DSTATE_g[colIdx * 3 + 2] =
      SLUGS2_DWork.IntegerDelay1_DSTATE_g[(colIdx + 1) * 3 + 2];
  }

  SLUGS2_DWork.IntegerDelay1_DSTATE_g[12] = rtb_Product1_kx[0];
  SLUGS2_DWork.IntegerDelay1_DSTATE_g[13] = rtb_Product1_kx[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_g[14] = rtb_Product1_kx[2];

  /* End of Update for Delay: '<S552>/Integer Delay1' */
  /* Update for DiscreteZeroPole: '<S559>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_o = rtb_DataTypeConversion2_g[0] +
      (-0.99960423832914991)*SLUGS2_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* Update for DiscreteZeroPole: '<S560>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_j = rtb_DataTypeConversion2_g[1] +
      (-0.99960423832914991)*SLUGS2_DWork.DiscreteZeroPole_DSTATE_j;
  }

  /* Update for DiscreteZeroPole: '<S561>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_k = rtb_DataTypeConversion2_g[2] +
      (-0.99960423832914991)*SLUGS2_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* Update for DiscreteZeroPole: '<S491>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_c = rtb_DataTypeConversion2_e2[0] + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* Update for DiscreteZeroPole: '<S492>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_n = rtb_DataTypeConversion2_e2[1] + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* Update for DiscreteZeroPole: '<S493>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_e = rtb_DataTypeConversion2_e2[2] + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* Update for DiscreteZeroPole: '<S564>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_jl = rtb_DataTypeConversion2_b + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_jl;
  }

  /* Update for DiscreteZeroPole: '<S566>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_a = rtb_DataTypeConversion2_j + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* Update for DiscreteZeroPole: '<S568>/Discrete Zero-Pole' */
  {
    SLUGS2_DWork.DiscreteZeroPole_DSTATE_b = rtb_DataTypeConversion2_gb + 1.0*
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_b;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
