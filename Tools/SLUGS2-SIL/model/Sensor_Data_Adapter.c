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
 * File: Sensor_Data_Adapter.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.223
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Wed Oct 12 11:23:18 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Wed Oct 12 11:23:23 2016
 */

#include "Sensor_Data_Adapter.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Initial conditions for atomic system:
 *    '<S591>/Embedded MATLAB Function'
 *    '<S591>/Embedded MATLAB Function1'
 *    '<S591>/Embedded MATLAB Function2'
 *    '<S592>/Embedded MATLAB Function'
 *    '<S592>/Embedded MATLAB Function1'
 *    '<S592>/Embedded MATLAB Function2'
 *    '<S593>/Embedded MATLAB Function'
 *    '<S593>/Embedded MATLAB Function1'
 *    '<S593>/Embedded MATLAB Function2'
 *    '<S624>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunct_m_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S591>/Embedded MATLAB Function'
 *    '<S591>/Embedded MATLAB Function1'
 *    '<S591>/Embedded MATLAB Function2'
 *    '<S592>/Embedded MATLAB Function'
 *    '<S592>/Embedded MATLAB Function1'
 *    '<S592>/Embedded MATLAB Function2'
 *    '<S593>/Embedded MATLAB Function'
 *    '<S593>/Embedded MATLAB Function1'
 *    '<S593>/Embedded MATLAB Function2'
 *    '<S624>/Embedded MATLAB Function'
 *    ...
 */
void A_EmbeddedMATLABFunction_f(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_gp_T *localB, rtDW_EmbeddedMATLABFunction_i_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S597>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S597>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S597>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S597>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S597>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S597>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S597>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S597>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S597>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S597>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S591>/myMux Fun'
 *    '<S592>/myMux Fun'
 *    '<S593>/myMux Fun'
 */
void AUAV_V3_TestSenso_myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_AUAV_V3_TestSens_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S600>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S600>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S586>/Embedded MATLAB Function1'
 *    '<S586>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunction1_i(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction1_e_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function1': '<S610>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S610>:1:5' */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/* Initial conditions for atomic system: '<Root>/Sensor_Data_Adapter' */
void A_Sensor_Data_Adapter_Init(void)
{
  /* InitializeConditions for MATLAB Function: '<S624>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_g);

  /* InitializeConditions for MATLAB Function: '<S626>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_lw);

  /* InitializeConditions for MATLAB Function: '<S627>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_cn);

  /* InitializeConditions for MATLAB Function: '<S625>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_np);

  /* InitializeConditions for MATLAB Function: '<S591>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_f);

  /* InitializeConditions for MATLAB Function: '<S591>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_i);

  /* InitializeConditions for MATLAB Function: '<S591>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_h);

  /* InitializeConditions for MATLAB Function: '<S592>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_kq);

  /* InitializeConditions for MATLAB Function: '<S592>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_f);

  /* InitializeConditions for MATLAB Function: '<S592>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_f);

  /* InitializeConditions for MATLAB Function: '<S593>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ft);

  /* InitializeConditions for MATLAB Function: '<S593>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_in);

  /* InitializeConditions for MATLAB Function: '<S593>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* InitializeConditions for MATLAB Function: '<S617>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV_V3_TestSensors_DWork.sf_EnablesDisablestheComputat_b);
}

/* Start for atomic system: '<Root>/Sensor_Data_Adapter' */
void Sensor_Data_Adapter_Start(void)
{
#ifndef WIN
  /* Start for S-Function (MCHP_MCU_LOAD): '<S583>/Calculus Time Step1' */
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */
#endif
}

/* Output and update for atomic system: '<Root>/Sensor_Data_Adapter' */
void AUAV_V_Sensor_Data_Adapter(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion_n;
  real_T rtb_DiscreteZeroPole_i;
  boolean_T rtb_LogicalOperator;
  int16_T rtb_Switch_d[13];
  real32_T rtb_Sum_o;
  uint8_T rtb_Compare_gl;
  real32_T rtb_u001maxDynPress;
  real32_T rtb_Sum_d;
  int16_T rtb_DataTypeConversion_ct;
  int16_T rtb_DataTypeConversion1_ax;
  int16_T rtb_DataTypeConversion2_c;
  real_T tmp;

  /* Outputs for Enabled SubSystem: '<S16>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S580>/Enable'
   */
  if (AUAV_V3_TestSensors_B.HILManualSwitch > 0.0) {
    /* S-Function (MCHP_C_function_Call): '<S580>/Data from HIL [hil.c]2' */
    hilRead(
            &AUAV_V3_TestSensors_B.DatafromHILhilc2[0]
            );

    /* S-Function (MCHP_C_function_Call): '<S580>/HIL Messages  Parser//Decoder [hil.c]1' */
    protDecodeHil(
                  &AUAV_V3_TestSensors_B.DatafromHILhilc2[0]
                  );

    /* S-Function (MCHP_C_function_Call): '<S580>/HIL Raw Readings [hil.c]1' */
    hil_getRawRead(
                   &AUAV_V3_TestSensors_B.HILRawReadingshilc1[0]
                   );
  }

  /* End of Outputs for SubSystem: '<S16>/Raw HIL  Readings' */

  /* Logic: '<S581>/Logical Operator' */
  rtb_LogicalOperator = !(AUAV_V3_TestSensors_B.HILManualSwitch != 0.0);

  /* Outputs for Enabled SubSystem: '<S581>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S586>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* DataTypeConversion: '<S586>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S586>/Get mlAirData1'
     */
    rtb_Sum_o = (real32_T)floor(mlAirData.press_abs);
    if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
      rtb_Sum_o = 0.0F;
    } else {
      rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
    }

    rtb_DataTypeConversion_ct = rtb_Sum_o < 0.0F ? -(int16_T)(uint16_T)
      -rtb_Sum_o : (int16_T)(uint16_T)rtb_Sum_o;

    /* End of DataTypeConversion: '<S586>/Data Type Conversion' */

    /* MATLAB Function: '<S586>/Embedded MATLAB Function1' incorporates:
     *  Constant: '<S586>/Constant1'
     */
    EmbeddedMATLABFunction1_i(AUAV_V3_TestSensors_ConstP.pooled16,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_ib);

    /* DataTypeConversion: '<S586>/Data Type Conversion1' */
    tmp = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_ib.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion1_ax = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S586>/Data Type Conversion1' */

    /* MATLAB Function: '<S586>/Embedded MATLAB Function2' incorporates:
     *  Constant: '<S586>/Constant2'
     */
    EmbeddedMATLABFunction1_i(AUAV_V3_TestSensors_ConstP.pooled16,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_p);

    /* DataTypeConversion: '<S586>/Data Type Conversion2' */
    tmp = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_p.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion2_c = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S586>/Data Type Conversion2' */

    /* MATLAB Function: '<S586>/myMux Fun' incorporates:
     *  DataStoreRead: '<S586>/Get mlAirData2'
     */
    /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S615>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S615>:1:5' */
    AUAV_V3_TestSensors_B.y_g[0] = rtb_DataTypeConversion_ct;
    AUAV_V3_TestSensors_B.y_g[1] = rtb_DataTypeConversion1_ax;
    AUAV_V3_TestSensors_B.y_g[2] = rtb_DataTypeConversion2_c;
    AUAV_V3_TestSensors_B.y_g[3] = mlAirData.temperature;

    /* S-Function (MCHP_C_function_Call): '<S586>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     &AUAV_V3_TestSensors_B.y_g[0]
                     );

    /* S-Function (MCHP_C_function_Call): '<S586>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                &AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[0]
                );

    /* MATLAB Function: '<S586>/myMux Fun4' incorporates:
     *  DataStoreRead: '<S586>/Get mlAirData2'
     */
    /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S616>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S616>:1:5' */
    AUAV_V3_TestSensors_B.y_l[0] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[0];
    AUAV_V3_TestSensors_B.y_l[1] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[1];
    AUAV_V3_TestSensors_B.y_l[2] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[2];
    AUAV_V3_TestSensors_B.y_l[3] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[3];
    AUAV_V3_TestSensors_B.y_l[4] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[4];
    AUAV_V3_TestSensors_B.y_l[5] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[5];
    AUAV_V3_TestSensors_B.y_l[6] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[6];
    AUAV_V3_TestSensors_B.y_l[7] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[7];
    AUAV_V3_TestSensors_B.y_l[8] =
      AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[8];
    AUAV_V3_TestSensors_B.y_l[9] = rtb_DataTypeConversion_ct;
    AUAV_V3_TestSensors_B.y_l[10] = rtb_DataTypeConversion1_ax;
    AUAV_V3_TestSensors_B.y_l[11] = rtb_DataTypeConversion2_c;
    AUAV_V3_TestSensors_B.y_l[12] = mlAirData.temperature;

    /* S-Function (MCHP_C_function_Call): '<S586>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    AUAV_V3_TestSensors_B.IstheGPSNovatelorUbloxgpsPortc1 = isGPSNovatel(
      );

    /* Outputs for Enabled SubSystem: '<S586>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S614>/Enable'
     */
    /* Logic: '<S586>/Logical Operator' */
    if (!(AUAV_V3_TestSensors_B.IstheGPSNovatelorUbloxgpsPortc1 != 0)) {
      /* S-Function (MCHP_C_function_Call): '<S614>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdat_a[
                          0]
                          );
    }

    /* End of Logic: '<S586>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S586>/if GPS is Ublox' */
  }

  /* End of Outputs for SubSystem: '<S581>/If no HIL then Read all the Sensors' */

  /* Switch: '<S581>/Switch' */
  for (rtb_DataTypeConversion_ct = 0; rtb_DataTypeConversion_ct < 13;
       rtb_DataTypeConversion_ct++) {
    if (AUAV_V3_TestSensors_B.HILManualSwitch != 0.0) {
      rtb_Switch_d[rtb_DataTypeConversion_ct] =
        AUAV_V3_TestSensors_B.HILRawReadingshilc1[rtb_DataTypeConversion_ct];
    } else {
      rtb_Switch_d[rtb_DataTypeConversion_ct] =
        AUAV_V3_TestSensors_B.y_l[rtb_DataTypeConversion_ct];
    }
  }

  /* End of Switch: '<S581>/Switch' */

  /* MATLAB Function: '<S624>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S624>/Constant'
   *  Constant: '<S624>/Constant1'
   *  DataTypeConversion: '<S588>/Data Type Conversion5'
   */
  A_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[12], 0.01, 0.02,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_g,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_g);

  /* Sum: '<S623>/Sum' incorporates:
   *  Constant: '<S623>/Bias'
   *  Product: '<S623>/Divide'
   */
  rtb_Sum_o = (real32_T)(1.5112853050231934 *
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_g.y) + -1605.28198F;

  /* RelationalOperator: '<S636>/Compare' incorporates:
   *  Constant: '<S636>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -130.0F);

  /* Outputs for Enabled SubSystem: '<S621>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S637>/Enable'
   */
  /* Logic: '<S621>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S637>/Sum2' incorporates:
     *  Constant: '<S637>/Mean Temperature for Calibration'
     *  Constant: '<S637>/gains'
     *  Product: '<S637>/Divide1'
     *  Sum: '<S637>/Sum1'
     */
    AUAV_V3_TestSensors_B.Merge = (real32_T)rtb_Switch_d[10] - (rtb_Sum_o -
      293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S621>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S621>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S621>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S638>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S638>/Add' incorporates:
     *  Constant: '<S638>/Constant'
     *  Constant: '<S638>/Mean Temperature for Calibration'
     *  Constant: '<S638>/gains'
     *  Product: '<S638>/Divide1'
     *  Sum: '<S638>/Sum1'
     *  Sum: '<S638>/Sum2'
     */
    AUAV_V3_TestSensors_B.Merge = ((real32_T)rtb_Switch_d[10] - (rtb_Sum_o -
      -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S621>/Lo Temp Compensation' */

  /* MATLAB Function: '<S626>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S626>/Constant'
   *  Constant: '<S626>/Constant1'
   *  DataTypeConversion: '<S588>/Data Type Conversion3'
   */
  A_EmbeddedMATLABFunction_f((real_T)AUAV_V3_TestSensors_B.Merge, 0.01, 4.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lw,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_lw);

  /* Sum: '<S619>/Sum' incorporates:
   *  Constant: '<S619>/Bias'
   *  Constant: '<S619>/Gains'
   *  DataTypeConversion: '<S588>/Data Type Conversion4'
   *  Product: '<S619>/Divide'
   */
  rtb_u001maxDynPress = 1.05137849F * (real32_T)
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lw.y + -1005.87872F;

  /* Saturate: '<S588>/[0.001  maxDynPress]' */
  if (rtb_u001maxDynPress > 3000.0F) {
    rtb_u001maxDynPress = 3000.0F;
  } else {
    if (rtb_u001maxDynPress < 0.001F) {
      rtb_u001maxDynPress = 0.001F;
    }
  }

  /* End of Saturate: '<S588>/[0.001  maxDynPress]' */

  /* RelationalOperator: '<S639>/Compare' incorporates:
   *  Constant: '<S639>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -50.0F);

  /* Outputs for Enabled SubSystem: '<S622>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S640>/Enable'
   */
  /* Logic: '<S622>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S640>/Add' incorporates:
     *  Constant: '<S640>/Constant'
     *  Constant: '<S640>/Mean Temperature for Calibration'
     *  Constant: '<S640>/gains'
     *  Product: '<S640>/Divide1'
     *  Sum: '<S640>/Sum1'
     *  Sum: '<S640>/Sum2'
     */
    AUAV_V3_TestSensors_B.Merge_j = ((real32_T)rtb_Switch_d[9] - (rtb_Sum_o -
      347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S622>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S622>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S622>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S641>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S641>/Sum2' incorporates:
     *  Constant: '<S641>/Mean Temperature for Calibration'
     *  Constant: '<S641>/gains'
     *  Product: '<S641>/Divide1'
     *  Sum: '<S641>/Sum1'
     */
    AUAV_V3_TestSensors_B.Merge_j = (real32_T)rtb_Switch_d[9] - (rtb_Sum_o -
      -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S622>/Lo Temp Compensation' */

  /* MATLAB Function: '<S627>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S627>/Constant'
   *  Constant: '<S627>/Constant1'
   *  DataTypeConversion: '<S588>/Data Type Conversion19'
   */
  A_EmbeddedMATLABFunction_f((real_T)AUAV_V3_TestSensors_B.Merge_j, 0.01, 4.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_cn,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_cn);

  /* Sum: '<S618>/Sum' incorporates:
   *  Constant: '<S618>/Bias'
   *  Constant: '<S618>/Gains'
   *  DataTypeConversion: '<S588>/Data Type Conversion1'
   *  Product: '<S618>/Divide'
   */
  rtb_Sum_d = 27.127F * (real32_T)
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_cn.y + 9444.44434F;

  /* MATLAB Function: '<S581>/myMux Fun' */
  AUAV_V3_TestSe_myMuxFun1_e(rtb_u001maxDynPress, rtb_Sum_d, rtb_Sum_o,
    &AUAV_V3_TestSensors_B.sf_myMuxFun);

  /* Outputs for Enabled SubSystem: '<S581>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S587>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* Inport: '<S587>/AirData' */
    AUAV_V3_TestSensors_B.AirData[0] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[0];
    AUAV_V3_TestSensors_B.AirData[1] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[1];
    AUAV_V3_TestSensors_B.AirData[2] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[2];

    /* S-Function (MCHP_C_function_Call): '<S587>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
    updateAirData(
                  &AUAV_V3_TestSensors_B.AirData[0]
                  );
  }

  /* End of Outputs for SubSystem: '<S581>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S581>/myMux Fun1' */
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/myMux Fun1': '<S590>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S590>:1:5' */
  AUAV_V3_TestSensors_B.y_o[0] = rtb_u001maxDynPress;
  AUAV_V3_TestSensors_B.y_o[1] = AUAV_V3_TestSensors_B.AirData[0];
  AUAV_V3_TestSensors_B.y_o[2] = 0.0F;
  AUAV_V3_TestSensors_B.y_o[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S581>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   &AUAV_V3_TestSensors_B.y_o[0]
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S585>/Calculus Time Step1 */
  AUAV_V3_TestSensors_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S585>/Calculus Time Step2 */
  {
//#ifndef WIN
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    AUAV_V3_TestSensors_B.CalculusTimeStep2 = tmp;
//#endif
  }

  /* DataTypeConversion: '<S585>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S585>/Data Type Conversion1'
   *  DataTypeConversion: '<S585>/Data Type Conversion2'
   *  Gain: '<S585>/Gain'
   *  Product: '<S585>/Divide'
   *  Rounding: '<S585>/Rounding Function'
   */
  tmp = floor((real_T)AUAV_V3_TestSensors_B.CalculusTimeStep1 / (real_T)
              AUAV_V3_TestSensors_B.CalculusTimeStep2 * 100.0);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 256.0);
  }

  AUAV_V3_TestSensors_B.DataTypeConversion12 = (uint8_T)(tmp < 0.0 ? (int16_T)
    (uint8_T)-(int8_T)(uint8_T)-tmp : (int16_T)(uint8_T)tmp);

  /* End of DataTypeConversion: '<S585>/Data Type Conversion12' */

  /* MATLAB Function: '<S625>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S625>/Constant'
   *  Constant: '<S625>/Constant1'
   *  DataTypeConversion: '<S588>/Data Type Conversion6'
   */
  A_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[11], 0.01, 0.02,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_np,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_np);

  /* DataTypeConversion: '<S588>/Data Type Conversion8' incorporates:
   *  Constant: '<S620>/Bias'
   *  Product: '<S620>/Divide'
   *  Sum: '<S620>/Sum'
   */
  rtb_Sum_o = (real32_T)floor((real32_T)(3.1760616302490234 *
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_np.y) + 911.698242F);
  if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
    rtb_Sum_o = 0.0F;
  } else {
    rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
  }

  AUAV_V3_TestSensors_B.DataTypeConversion8 = rtb_Sum_o < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_Sum_o : (uint16_T)rtb_Sum_o;

  /* End of DataTypeConversion: '<S588>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S581>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 AUAV_V3_TestSensors_B.DataTypeConversion12
                 , AUAV_V3_TestSensors_B.DataTypeConversion8
                 );

  /* S-Function (MCHP_C_function_Call): '<S583>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  getGpsUbloxData(
                  &AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[0]
                  );

  /* MATLAB Function: '<S591>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S584>/Gyro Gains'
   *  Constant: '<S591>/Constant'
   *  Constant: '<S591>/Constant1'
   *  DataTypeConversion: '<S584>/Data Type Conversion2'
   *  Gain: '<S594>/[ -1 1 -1]'
   *  Product: '<S584>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[0] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_f);

  /* MATLAB Function: '<S591>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S584>/Gyro Gains'
   *  Constant: '<S591>/Constant2'
   *  Constant: '<S591>/Constant3'
   *  DataTypeConversion: '<S584>/Data Type Conversion2'
   *  Product: '<S584>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[1] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_i,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_i);

  /* MATLAB Function: '<S591>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S584>/Gyro Gains'
   *  Constant: '<S591>/Constant4'
   *  Constant: '<S591>/Constant5'
   *  DataTypeConversion: '<S584>/Data Type Conversion2'
   *  Gain: '<S594>/[ -1 1 -1]'
   *  Product: '<S584>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[2] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_h,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_h);

  /* MATLAB Function: '<S591>/myMux Fun' */
  AUAV_V3_TestSenso_myMuxFun(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_f.y,
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_i.y,
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_h.y,
    &AUAV_V3_TestSensors_B.sf_myMuxFun_i);

  /* DataTypeConversion: '<S584>/Data Type Conversion7' */
  AUAV_V3_TestSensors_B.DataTypeConversion7[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion7[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion7[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[2];

  /* MATLAB Function: '<S592>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S584>/Gyro Gains1'
   *  Constant: '<S592>/Constant'
   *  Constant: '<S592>/Constant1'
   *  DataTypeConversion: '<S584>/Data Type Conversion1'
   *  Gain: '<S595>/[ -1 1 -1]'
   *  Product: '<S584>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[3] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_kq,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_kq);

  /* MATLAB Function: '<S592>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S584>/Gyro Gains1'
   *  Constant: '<S592>/Constant2'
   *  Constant: '<S592>/Constant3'
   *  DataTypeConversion: '<S584>/Data Type Conversion1'
   *  Product: '<S584>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[4] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_f);

  /* MATLAB Function: '<S592>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S584>/Gyro Gains1'
   *  Constant: '<S592>/Constant4'
   *  Constant: '<S592>/Constant5'
   *  DataTypeConversion: '<S584>/Data Type Conversion1'
   *  Gain: '<S595>/[ -1 1 -1]'
   *  Product: '<S584>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[5] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_f);

  /* MATLAB Function: '<S592>/myMux Fun' */
  AUAV_V3_TestSenso_myMuxFun
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_kq.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_f.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_f.y,
     &AUAV_V3_TestSensors_B.sf_myMuxFun_m);

  /* DataTypeConversion: '<S584>/Data Type Conversion3' */
  AUAV_V3_TestSensors_B.DataTypeConversion3[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion3[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion3[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[2];

  /* MATLAB Function: '<S593>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S584>/Gyro Gains2'
   *  Constant: '<S593>/Constant'
   *  Constant: '<S593>/Constant1'
   *  DataTypeConversion: '<S584>/Data Type Conversion5'
   *  Product: '<S584>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[6] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ft,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ft);

  /* MATLAB Function: '<S593>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S584>/Gyro Gains2'
   *  Constant: '<S593>/Constant2'
   *  Constant: '<S593>/Constant3'
   *  DataTypeConversion: '<S584>/Data Type Conversion5'
   *  Gain: '<S596>/[ 1 -1 -1]'
   *  Product: '<S584>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[8] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_in,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_in);

  /* MATLAB Function: '<S593>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S584>/Gyro Gains2'
   *  Constant: '<S593>/Constant4'
   *  Constant: '<S593>/Constant5'
   *  DataTypeConversion: '<S584>/Data Type Conversion5'
   *  Gain: '<S596>/[ 1 -1 -1]'
   *  Product: '<S584>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[7] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_fu,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* MATLAB Function: '<S593>/myMux Fun' */
  AUAV_V3_TestSenso_myMuxFun
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ft.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_in.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_fu.y,
     &AUAV_V3_TestSensors_B.sf_myMuxFun_a);

  /* DataTypeConversion: '<S584>/Data Type Conversion6' */
  AUAV_V3_TestSensors_B.DataTypeConversion6[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion6[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion6[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[2];

  /* MATLAB Function: '<S617>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b,
     &AUAV_V3_TestSensors_DWork.sf_EnablesDisablestheComputat_b);

  /* Outputs for Enabled SubSystem: '<S617>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S632>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* DataTypeConversion: '<S632>/Data Type Conversion' */
    rtb_DataTypeConversion_n = rtb_Sum_d;

    /* DiscreteZeroPole: '<S635>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_i = 0.014778325123152709*rtb_DataTypeConversion_n;
      rtb_DiscreteZeroPole_i += 0.029119852459414206*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S632>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S632>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_i > 120000.0F) {
      AUAV_V3_TestSensors_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_i < 80000.0F) {
      AUAV_V3_TestSensors_B.u0k120k = 80000.0F;
    } else {
      AUAV_V3_TestSensors_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_i;
    }

    /* End of Saturate: '<S632>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S635>/Discrete Zero-Pole' */
    {
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE =
        rtb_DataTypeConversion_n + 0.97044334975369462*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S617>/Initial Baro Bias' */

  /* Product: '<S628>/Divide' incorporates:
   *  Sum: '<S628>/Sum2'
   */
  rtb_Sum_o = (rtb_Sum_d - AUAV_V3_TestSensors_B.u0k120k) /
    AUAV_V3_TestSensors_B.u0k120k;

  /* S-Function (MCHP_C_function_Call): '<S631>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0]
                );

  /* Sum: '<S628>/Sum1' incorporates:
   *  Constant: '<S628>/Constant2'
   *  Constant: '<S628>/Constant3'
   *  Constant: '<S628>/Constant4'
   *  Constant: '<S628>/Constant5'
   *  Gain: '<S634>/Unit Conversion'
   *  Product: '<S628>/Divide1'
   *  Product: '<S628>/Divide2'
   *  Product: '<S628>/Divide3'
   *  Product: '<S628>/Divide4'
   *  Sum: '<S628>/Sum3'
   */
  rtb_Sum_o = ((rtb_Sum_o * rtb_Sum_o * 0.093502529F + rtb_Sum_o * -0.188893303F)
               + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0];

  /* Outputs for Enabled SubSystem: '<S617>/Zero Out Height' incorporates:
   *  EnablePort: '<S633>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* Sum: '<S633>/Sum' incorporates:
     *  Delay: '<S633>/Integer Delay'
     */
    AUAV_V3_TestSensors_B.Sum =
      AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0] -
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE;

    /* Update for Delay: '<S633>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE = rtb_Sum_o;
  }

  /* End of Outputs for SubSystem: '<S617>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S617>/Enabled Subsystem' */

  /* Logic: '<S617>/Logical Operator' incorporates:
   *  Sum: '<S617>/Sum1'
   */
  AUAV_V3_T_EnabledSubsystem
    (!(AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut != 0.0),
     AUAV_V3_TestSensors_B.Sum + rtb_Sum_o,
     &AUAV_V3_TestSensors_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S617>/Enabled Subsystem' */

  /* Switch: '<S581>/Switch1' */
  for (rtb_DataTypeConversion_ct = 0; rtb_DataTypeConversion_ct < 5;
       rtb_DataTypeConversion_ct++) {
    if (rtb_LogicalOperator) {
      AUAV_V3_TestSensors_B.Switch1[rtb_DataTypeConversion_ct] =
        AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdat_a[rtb_DataTypeConversion_ct];
    } else {
      AUAV_V3_TestSensors_B.Switch1[rtb_DataTypeConversion_ct] =
        AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[rtb_DataTypeConversion_ct];
    }
  }

  /* End of Switch: '<S581>/Switch1' */

  /* S-Function (MCHP_C_function_Call): '<S16>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  AUAV_V3_TestSensors_B.ChecksifFixTypeis3updateSensorM = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S16>/Initialize GS Location' incorporates:
   *  EnablePort: '<S579>/Enable'
   */
  /* Logic: '<S16>/Logical Operator' incorporates:
   *  DataStoreRead: '<S16>/Data Store Read'
   */
  if ((AUAV_V3_TestSensors_B.ChecksifFixTypeis3updateSensorM != 0) &&
      (AUAV_V3_TestSensors_DWork.GS_INIT_FLAG != 0.0)) {
    /* DataStoreWrite: '<S579>/Data Store Write' incorporates:
     *  Constant: '<S579>/Constant'
     */
    AUAV_V3_TestSensors_DWork.GS_INIT_FLAG = 0.0;

    /* DataStoreWrite: '<S579>/Data Store Write1' incorporates:
     *  DataStoreRead: '<S579>/Data Store Read1'
     *  DataTypeConversion: '<S579>/Data Type Conversion'
     *  DataTypeConversion: '<S579>/Data Type Conversion1'
     *  DataTypeConversion: '<S579>/Data Type Conversion2'
     */
    mlGSLocationFloat.lat = (real32_T)mlGpsData.lat;
    mlGSLocationFloat.lon = (real32_T)mlGpsData.lon;
    mlGSLocationFloat.alt = (real32_T)mlGpsData.alt;
  }

  /* End of Logic: '<S16>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S16>/Initialize GS Location' */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
