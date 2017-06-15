/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Sensor_Data_Adapter.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "Sensor_Data_Adapter.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Output and update for atomic system:
 *    '<S575>/Embedded MATLAB Function1'
 *    '<S575>/Embedded MATLAB Function2'
 *    '<S583>/Embedded MATLAB Function1'
 *    '<S583>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunction1_m(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction1_e_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/Embedded MATLAB Function1': '<S577>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S577>:1:5' */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/*
 * Output and update for atomic system:
 *    '<S575>/myMux Fun'
 *    '<S583>/myMux Fun'
 */
void SLUGS2_myMuxFun(int16_T rtu_u1, int16_T rtu_u2, int16_T rtu_u3, int16_T
                     rtu_u4, rtB_myMuxFun_SLUGS2_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/myMux Fun': '<S579>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S579>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
  localB->y[3] = rtu_u4;
}

/*
 * Initial conditions for atomic system:
 *    '<S588>/Embedded MATLAB Function'
 *    '<S588>/Embedded MATLAB Function1'
 *    '<S588>/Embedded MATLAB Function2'
 *    '<S589>/Embedded MATLAB Function'
 *    '<S589>/Embedded MATLAB Function1'
 *    '<S589>/Embedded MATLAB Function2'
 *    '<S590>/Embedded MATLAB Function'
 *    '<S590>/Embedded MATLAB Function1'
 *    '<S590>/Embedded MATLAB Function2'
 *    '<S621>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunct_d_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S588>/Embedded MATLAB Function'
 *    '<S588>/Embedded MATLAB Function1'
 *    '<S588>/Embedded MATLAB Function2'
 *    '<S589>/Embedded MATLAB Function'
 *    '<S589>/Embedded MATLAB Function1'
 *    '<S589>/Embedded MATLAB Function2'
 *    '<S590>/Embedded MATLAB Function'
 *    '<S590>/Embedded MATLAB Function1'
 *    '<S590>/Embedded MATLAB Function2'
 *    '<S621>/Embedded MATLAB Function'
 *    ...
 */
void S_EmbeddedMATLABFunction_f(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_gp_T *localB, rtDW_EmbeddedMATLABFunction_i_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S594>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S594>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S594>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S594>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S594>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S594>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S594>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S594>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S594>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S594>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S588>/myMux Fun'
 *    '<S589>/myMux Fun'
 *    '<S590>/myMux Fun'
 */
void SLUGS2_myMuxFun_i(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_SLUGS2_i_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S597>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S597>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/* Initial conditions for atomic system: '<Root>/Sensor_Data_Adapter' */
void S_Sensor_Data_Adapter_Init(void)
{
  /* InitializeConditions for MATLAB Function: '<S621>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_g);

  /* InitializeConditions for MATLAB Function: '<S623>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_lw);

  /* InitializeConditions for MATLAB Function: '<S624>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_cn);

  /* InitializeConditions for MATLAB Function: '<S622>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_np);

  /* InitializeConditions for MATLAB Function: '<S588>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_f);

  /* InitializeConditions for MATLAB Function: '<S588>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction1_i);

  /* InitializeConditions for MATLAB Function: '<S588>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction2_h);

  /* InitializeConditions for MATLAB Function: '<S589>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_kq);

  /* InitializeConditions for MATLAB Function: '<S589>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction1_f);

  /* InitializeConditions for MATLAB Function: '<S589>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction2_f);

  /* InitializeConditions for MATLAB Function: '<S590>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_ft);

  /* InitializeConditions for MATLAB Function: '<S590>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction1_in);

  /* InitializeConditions for MATLAB Function: '<S590>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_d_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* InitializeConditions for MATLAB Function: '<S614>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init(&SLUGS2_DWork.sf_EnablesDisablestheComputat_b);
}

/* Start for atomic system: '<Root>/Sensor_Data_Adapter' */
void Sensor_Data_Adapter_Start(void)
{
#ifndef WIN
  /* Start for S-Function (MCHP_MCU_LOAD): '<S590>/Calculus Time Step1' */
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */
#endif
  /* Start for Enabled SubSystem: '<S614>/Enabled Subsystem' */
  SLU_EnabledSubsystem_Start(&SLUGS2_B.EnabledSubsystem_m);

  /* End of Start for SubSystem: '<S614>/Enabled Subsystem' */
}

/* Output and update for atomic system: '<Root>/Sensor_Data_Adapter' */
void SLUGS2_Sensor_Data_Adapter(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion_n;
  real_T rtb_DiscreteZeroPole_i;
  boolean_T rtb_LogicalOperator_df;
  int16_T rtb_Switch_d[13];
  real32_T rtb_Sum_o;
  uint8_T rtb_Compare_gl;
  real32_T rtb_u001maxDynPress;
  real32_T rtb_Sum_d;
  int16_T rtb_DataTypeConversion1_ax;
  int16_T rtb_DataTypeConversion2_c;
  int16_T i;
  real_T tmp;
  real_T tmp_0;

  /* Outputs for Enabled SubSystem: '<S13>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S572>/Enable'
   */
  if (SLUGS2_B.DataStoreRead > 0.0) {
    /* Outputs for Enabled SubSystem: '<S572>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S574>/Enable'
     */
    /* DataStoreRead: '<S572>/Data Store Read' */
    if (SLUGS2_DWork.SIX_DOF_HIL_FLAG > 0.0) {
      /* S-Function (MCHP_C_function_Call): '<S574>/Data from HIL [hil.c]2' */
      hilRead(
              &SLUGS2_B.DatafromHILhilc2[0]
              );

      /* S-Function (MCHP_C_function_Call): '<S574>/HIL Messages  Parser//Decoder [hil.c]1' */
      protDecodeHil(
                    &SLUGS2_B.DatafromHILhilc2[0]
                    );

      /* S-Function (MCHP_C_function_Call): '<S574>/HIL Raw Readings [hil.c]1' */
      hil_getRawRead(
                     &SLUGS2_B.HILRawReadingshilc1_k[0]
                     );
    }

    /* End of DataStoreRead: '<S572>/Data Store Read' */
    /* End of Outputs for SubSystem: '<S572>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S572>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S575>/Enable'
     */
    /* DataStoreRead: '<S572>/Data Store Read1' */
    if (SLUGS2_DWork.X_PLANE_HIL_FLAG > 0.0) {
      /* S-Function (MCHP_C_function_Call): '<S575>/Data from HIL X-Plane' */
      HILSIM_set_gplane(
                        );

      /* S-Function (MCHP_C_function_Call): '<S575>/Data from HIL X-Plane2' */
      HILSIM_set_omegagyro(
                           );

      /* S-Function (MCHP_C_function_Call): '<S575>/HIL Raw Readings [hil.c]1' */
      hil_getRawRead(
                     &SLUGS2_B.HILRawReadingshilc1[0]
                     );

      /* MATLAB Function: '<S575>/Embedded MATLAB Function1' incorporates:
       *  Constant: '<S575>/Constant1'
       */
      EmbeddedMATLABFunction1_m(SLUGS2_ConstP.pooled16,
        &SLUGS2_B.sf_EmbeddedMATLABFunction1_m);

      /* DataTypeConversion: '<S575>/Data Type Conversion1' */
      tmp_0 = floor(SLUGS2_B.sf_EmbeddedMATLABFunction1_m.y);
      if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
        tmp_0 = 0.0;
      } else {
        tmp_0 = fmod(tmp_0, 65536.0);
      }

      /* MATLAB Function: '<S575>/Embedded MATLAB Function2' incorporates:
       *  Constant: '<S575>/Constant2'
       */
      EmbeddedMATLABFunction1_m(SLUGS2_ConstP.pooled16,
        &SLUGS2_B.sf_EmbeddedMATLABFunction2_i);

      /* DataTypeConversion: '<S575>/Data Type Conversion' incorporates:
       *  DataStoreRead: '<S575>/Get mlAirData1'
       */
      rtb_Sum_o = (real32_T)floor(mlAirData.press_abs);
      if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
        rtb_Sum_o = 0.0F;
      } else {
        rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
      }

      /* DataTypeConversion: '<S575>/Data Type Conversion2' */
      tmp = floor(SLUGS2_B.sf_EmbeddedMATLABFunction2_i.y);
      if (rtIsNaN(tmp) || rtIsInf(tmp)) {
        tmp = 0.0;
      } else {
        tmp = fmod(tmp, 65536.0);
      }

      /* MATLAB Function: '<S575>/myMux Fun' incorporates:
       *  DataStoreRead: '<S575>/Get mlAirData2'
       *  DataTypeConversion: '<S575>/Data Type Conversion'
       *  DataTypeConversion: '<S575>/Data Type Conversion1'
       *  DataTypeConversion: '<S575>/Data Type Conversion2'
       */
      SLUGS2_myMuxFun(rtb_Sum_o < 0.0F ? -(int16_T)(uint16_T)-rtb_Sum_o :
                      (int16_T)(uint16_T)rtb_Sum_o, tmp_0 < 0.0 ? -(int16_T)
                      (uint16_T)-tmp_0 : (int16_T)(uint16_T)tmp_0, tmp < 0.0 ?
                      -(int16_T)(uint16_T)-tmp : (int16_T)(uint16_T)tmp,
                      mlAirData.temperature, &SLUGS2_B.sf_myMuxFun_h);

      /* S-Function (MCHP_C_function_Call): '<S575>/Update State AP ADC Data [updateSensorMcuState.c]1' */
      updateRawADCData(
                       &SLUGS2_B.sf_myMuxFun_h.y[0]
                       );
    }

    /* End of Outputs for SubSystem: '<S572>/Enabled Subsystem1' */

    /* Switch: '<S572>/Switch' incorporates:
     *  DataStoreRead: '<S572>/Data Store Read1'
     */
    for (i = 0; i < 13; i++) {
      if (SLUGS2_DWork.X_PLANE_HIL_FLAG != 0.0) {
        SLUGS2_B.Switch_i[i] = SLUGS2_B.HILRawReadingshilc1[i];
      } else {
        SLUGS2_B.Switch_i[i] = SLUGS2_B.HILRawReadingshilc1_k[i];
      }
    }

    /* End of Switch: '<S572>/Switch' */
  }

  /* End of Outputs for SubSystem: '<S13>/Raw HIL  Readings' */

  /* Logic: '<S573>/Logical Operator' */
  rtb_LogicalOperator_df = !(SLUGS2_B.DataStoreRead != 0.0);

  /* Outputs for Enabled SubSystem: '<S573>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S583>/Enable'
   */
  if (rtb_LogicalOperator_df) {
    /* DataTypeConversion: '<S583>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S583>/Get mlAirData1'
     */
    rtb_Sum_o = (real32_T)floor(mlAirData.press_abs);
    if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
      rtb_Sum_o = 0.0F;
    } else {
      rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
    }

    i = rtb_Sum_o < 0.0F ? -(int16_T)(uint16_T)-rtb_Sum_o : (int16_T)(uint16_T)
      rtb_Sum_o;

    /* End of DataTypeConversion: '<S583>/Data Type Conversion' */

    /* MATLAB Function: '<S583>/Embedded MATLAB Function1' incorporates:
     *  Constant: '<S583>/Constant1'
     */
    EmbeddedMATLABFunction1_m(SLUGS2_ConstP.pooled16,
      &SLUGS2_B.sf_EmbeddedMATLABFunction1_ib);

    /* DataTypeConversion: '<S583>/Data Type Conversion1' */
    tmp_0 = floor(SLUGS2_B.sf_EmbeddedMATLABFunction1_ib.y);
    if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
      tmp_0 = 0.0;
    } else {
      tmp_0 = fmod(tmp_0, 65536.0);
    }

    rtb_DataTypeConversion1_ax = tmp_0 < 0.0 ? -(int16_T)(uint16_T)-tmp_0 :
      (int16_T)(uint16_T)tmp_0;

    /* End of DataTypeConversion: '<S583>/Data Type Conversion1' */

    /* MATLAB Function: '<S583>/Embedded MATLAB Function2' incorporates:
     *  Constant: '<S583>/Constant2'
     */
    EmbeddedMATLABFunction1_m(SLUGS2_ConstP.pooled16,
      &SLUGS2_B.sf_EmbeddedMATLABFunction2_p);

    /* DataTypeConversion: '<S583>/Data Type Conversion2' */
    tmp_0 = floor(SLUGS2_B.sf_EmbeddedMATLABFunction2_p.y);
    if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
      tmp_0 = 0.0;
    } else {
      tmp_0 = fmod(tmp_0, 65536.0);
    }

    rtb_DataTypeConversion2_c = tmp_0 < 0.0 ? -(int16_T)(uint16_T)-tmp_0 :
      (int16_T)(uint16_T)tmp_0;

    /* End of DataTypeConversion: '<S583>/Data Type Conversion2' */

    /* MATLAB Function: '<S583>/myMux Fun' incorporates:
     *  DataStoreRead: '<S583>/Get mlAirData2'
     */
    SLUGS2_myMuxFun(i, rtb_DataTypeConversion1_ax, rtb_DataTypeConversion2_c,
                    mlAirData.temperature, &SLUGS2_B.sf_myMuxFun_n);

    /* S-Function (MCHP_C_function_Call): '<S583>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     &SLUGS2_B.sf_myMuxFun_n.y[0]
                     );

    /* S-Function (MCHP_C_function_Call): '<S583>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                &SLUGS2_B.ReadtheCubeDataadisCube16405c1[0]
                );

    /* MATLAB Function: '<S583>/myMux Fun4' incorporates:
     *  DataStoreRead: '<S583>/Get mlAirData2'
     */
    /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S613>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S613>:1:5' */
    SLUGS2_B.y_l[0] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[0];
    SLUGS2_B.y_l[1] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[1];
    SLUGS2_B.y_l[2] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[2];
    SLUGS2_B.y_l[3] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[3];
    SLUGS2_B.y_l[4] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[4];
    SLUGS2_B.y_l[5] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[5];
    SLUGS2_B.y_l[6] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[6];
    SLUGS2_B.y_l[7] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[7];
    SLUGS2_B.y_l[8] = SLUGS2_B.ReadtheCubeDataadisCube16405c1[8];
    SLUGS2_B.y_l[9] = i;
    SLUGS2_B.y_l[10] = rtb_DataTypeConversion1_ax;
    SLUGS2_B.y_l[11] = rtb_DataTypeConversion2_c;
    SLUGS2_B.y_l[12] = mlAirData.temperature;

    /* S-Function (MCHP_C_function_Call): '<S583>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    SLUGS2_B.IstheGPSNovatelorUbloxgpsPortc1 = isGPSNovatel(
      );

    /* Outputs for Enabled SubSystem: '<S583>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S611>/Enable'
     */
    /* Logic: '<S583>/Logical Operator' */
    if (!(SLUGS2_B.IstheGPSNovatelorUbloxgpsPortc1 != 0)) {
      /* S-Function (MCHP_C_function_Call): '<S611>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &SLUGS2_B.ProducetheGPSMainDataandupdat_a[0]
                          );
    }

    /* End of Logic: '<S583>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S583>/if GPS is Ublox' */
  }

  /* End of Outputs for SubSystem: '<S573>/If no HIL then Read all the Sensors' */

  /* Switch: '<S573>/Switch' */
  for (i = 0; i < 13; i++) {
    if (SLUGS2_B.DataStoreRead != 0.0) {
      rtb_Switch_d[i] = SLUGS2_B.Switch_i[i];
    } else {
      rtb_Switch_d[i] = SLUGS2_B.y_l[i];
    }
  }

  /* End of Switch: '<S573>/Switch' */

  /* MATLAB Function: '<S621>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S621>/Constant'
   *  Constant: '<S621>/Constant1'
   *  DataTypeConversion: '<S585>/Data Type Conversion5'
   */
  S_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[12], 0.01, 0.02,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_g,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_g);

  /* Sum: '<S620>/Sum' incorporates:
   *  Constant: '<S620>/Bias'
   *  Product: '<S620>/Divide'
   */
  rtb_Sum_o = (real32_T)(1.5112853050231934 *
    SLUGS2_B.sf_EmbeddedMATLABFunction_g.y) + -1605.28198F;

  /* RelationalOperator: '<S633>/Compare' incorporates:
   *  Constant: '<S633>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -130.0F);

  /* Outputs for Enabled SubSystem: '<S618>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S634>/Enable'
   */
  /* Logic: '<S618>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S634>/Sum2' incorporates:
     *  Constant: '<S634>/Mean Temperature for Calibration'
     *  Constant: '<S634>/gains'
     *  Product: '<S634>/Divide1'
     *  Sum: '<S634>/Sum1'
     */
    SLUGS2_B.Merge = (real32_T)rtb_Switch_d[10] - (rtb_Sum_o - 293.053F) *
      -0.0950433F;
  }

  /* End of Logic: '<S618>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S618>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S618>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S635>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S635>/Add' incorporates:
     *  Constant: '<S635>/Constant'
     *  Constant: '<S635>/Mean Temperature for Calibration'
     *  Constant: '<S635>/gains'
     *  Product: '<S635>/Divide1'
     *  Sum: '<S635>/Sum1'
     *  Sum: '<S635>/Sum2'
     */
    SLUGS2_B.Merge = ((real32_T)rtb_Switch_d[10] - (rtb_Sum_o - -202.93F) *
                      -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S618>/Lo Temp Compensation' */

  /* MATLAB Function: '<S623>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S623>/Constant'
   *  Constant: '<S623>/Constant1'
   *  DataTypeConversion: '<S585>/Data Type Conversion3'
   */
  S_EmbeddedMATLABFunction_f((real_T)SLUGS2_B.Merge, 0.01, 4.0,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_lw,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_lw);

  /* Sum: '<S616>/Sum' incorporates:
   *  Constant: '<S616>/Bias'
   *  Constant: '<S616>/Gains'
   *  DataTypeConversion: '<S585>/Data Type Conversion4'
   *  Product: '<S616>/Divide'
   */
  rtb_u001maxDynPress = 1.05137849F * (real32_T)
    SLUGS2_B.sf_EmbeddedMATLABFunction_lw.y + -1005.87872F;

  /* Saturate: '<S585>/[0.001  maxDynPress]' */
  if (rtb_u001maxDynPress > 3000.0F) {
    rtb_u001maxDynPress = 3000.0F;
  } else {
    if (rtb_u001maxDynPress < 0.001F) {
      rtb_u001maxDynPress = 0.001F;
    }
  }

  /* End of Saturate: '<S585>/[0.001  maxDynPress]' */

  /* RelationalOperator: '<S636>/Compare' incorporates:
   *  Constant: '<S636>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -50.0F);

  /* Outputs for Enabled SubSystem: '<S619>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S637>/Enable'
   */
  /* Logic: '<S619>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S637>/Add' incorporates:
     *  Constant: '<S637>/Constant'
     *  Constant: '<S637>/Mean Temperature for Calibration'
     *  Constant: '<S637>/gains'
     *  Product: '<S637>/Divide1'
     *  Sum: '<S637>/Sum1'
     *  Sum: '<S637>/Sum2'
     */
    SLUGS2_B.Merge_j = ((real32_T)rtb_Switch_d[9] - (rtb_Sum_o - 347.23F) *
                        0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S619>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S619>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S619>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S638>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S638>/Sum2' incorporates:
     *  Constant: '<S638>/Mean Temperature for Calibration'
     *  Constant: '<S638>/gains'
     *  Product: '<S638>/Divide1'
     *  Sum: '<S638>/Sum1'
     */
    SLUGS2_B.Merge_j = (real32_T)rtb_Switch_d[9] - (rtb_Sum_o - -161.3F) *
      -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S619>/Lo Temp Compensation' */

  /* MATLAB Function: '<S624>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S624>/Constant'
   *  Constant: '<S624>/Constant1'
   *  DataTypeConversion: '<S585>/Data Type Conversion19'
   */
  S_EmbeddedMATLABFunction_f((real_T)SLUGS2_B.Merge_j, 0.01, 4.0,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_cn,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_cn);

  /* Sum: '<S615>/Sum' incorporates:
   *  Constant: '<S615>/Bias'
   *  Constant: '<S615>/Gains'
   *  DataTypeConversion: '<S585>/Data Type Conversion1'
   *  Product: '<S615>/Divide'
   */
  rtb_Sum_d = 27.127F * (real32_T)SLUGS2_B.sf_EmbeddedMATLABFunction_cn.y +
    9444.44434F;

  /* MATLAB Function: '<S573>/myMux Fun' */
  SLUGS2_myMuxFun1_e(rtb_u001maxDynPress, rtb_Sum_d, rtb_Sum_o,
                     &SLUGS2_B.sf_myMuxFun);

  /* Outputs for Enabled SubSystem: '<S573>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S584>/Enable'
   */
  /* Inport: '<S584>/AirData' */
  SLUGS2_B.AirData[0] = SLUGS2_B.sf_myMuxFun.y[0];
  SLUGS2_B.AirData[1] = SLUGS2_B.sf_myMuxFun.y[1];
  SLUGS2_B.AirData[2] = SLUGS2_B.sf_myMuxFun.y[2];

  /* S-Function (MCHP_C_function_Call): '<S584>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
  updateAirData(
                &SLUGS2_B.AirData[0]
                );

  /* End of Outputs for SubSystem: '<S573>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S573>/myMux Fun1' */
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/myMux Fun1': '<S587>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S587>:1:5' */
  SLUGS2_B.y_o[0] = rtb_u001maxDynPress;
  SLUGS2_B.y_o[1] = SLUGS2_B.AirData[0];
  SLUGS2_B.y_o[2] = 0.0F;
  SLUGS2_B.y_o[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S573>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   &SLUGS2_B.y_o[0]
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S582>/Calculus Time Step1 */
  SLUGS2_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S582>/Calculus Time Step2 */
  {
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    SLUGS2_B.CalculusTimeStep2 = tmp;
  }

  /* DataTypeConversion: '<S582>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S582>/Data Type Conversion1'
   *  DataTypeConversion: '<S582>/Data Type Conversion2'
   *  Gain: '<S582>/Gain'
   *  Product: '<S582>/Divide'
   *  Rounding: '<S582>/Rounding Function'
   */
  tmp_0 = floor((real_T)SLUGS2_B.CalculusTimeStep1 / (real_T)
                SLUGS2_B.CalculusTimeStep2 * 100.0);
  if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
    tmp_0 = 0.0;
  } else {
    tmp_0 = fmod(tmp_0, 256.0);
  }

  SLUGS2_B.DataTypeConversion12 = (uint8_T)(tmp_0 < 0.0 ? (int16_T)(uint8_T)
    -(int8_T)(uint8_T)-tmp_0 : (int16_T)(uint8_T)tmp_0);

  /* End of DataTypeConversion: '<S582>/Data Type Conversion12' */

  /* MATLAB Function: '<S622>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S622>/Constant'
   *  Constant: '<S622>/Constant1'
   *  DataTypeConversion: '<S585>/Data Type Conversion6'
   */
  S_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[11], 0.01, 0.02,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_np,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_np);

  /* DataTypeConversion: '<S585>/Data Type Conversion8' incorporates:
   *  Constant: '<S617>/Bias'
   *  Product: '<S617>/Divide'
   *  Sum: '<S617>/Sum'
   */
  rtb_Sum_o = (real32_T)floor((real32_T)(3.1760616302490234 *
    SLUGS2_B.sf_EmbeddedMATLABFunction_np.y) + 911.698242F);
  if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
    rtb_Sum_o = 0.0F;
  } else {
    rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
  }

  SLUGS2_B.DataTypeConversion8 = rtb_Sum_o < 0.0F ? (uint16_T)-(int16_T)
    (uint16_T)-rtb_Sum_o : (uint16_T)rtb_Sum_o;

  /* End of DataTypeConversion: '<S585>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S573>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 SLUGS2_B.DataTypeConversion12
                 , SLUGS2_B.DataTypeConversion8
                 );

  /* S-Function (MCHP_C_function_Call): '<S580>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  getGpsUbloxData(
                  &SLUGS2_B.ProducetheGPSMainDataandupdatet[0]
                  );

  /* MATLAB Function: '<S588>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S581>/Gyro Gains'
   *  Constant: '<S588>/Constant'
   *  Constant: '<S588>/Constant1'
   *  DataTypeConversion: '<S581>/Data Type Conversion2'
   *  Gain: '<S591>/[ -1 1 -1]'
   *  Product: '<S581>/Divide'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[0] *
    0.000266462477F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction_f,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_f);

  /* MATLAB Function: '<S588>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S581>/Gyro Gains'
   *  Constant: '<S588>/Constant2'
   *  Constant: '<S588>/Constant3'
   *  DataTypeConversion: '<S581>/Data Type Conversion2'
   *  Product: '<S581>/Divide'
   */
  S_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[1] *
    0.000266462477F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction1_i,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction1_i);

  /* MATLAB Function: '<S588>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S581>/Gyro Gains'
   *  Constant: '<S588>/Constant4'
   *  Constant: '<S588>/Constant5'
   *  DataTypeConversion: '<S581>/Data Type Conversion2'
   *  Gain: '<S591>/[ -1 1 -1]'
   *  Product: '<S581>/Divide'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[2] *
    0.000266462477F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction2_h,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction2_h);

  /* MATLAB Function: '<S588>/myMux Fun' */
  SLUGS2_myMuxFun_i(SLUGS2_B.sf_EmbeddedMATLABFunction_f.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction1_i.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction2_h.y,
                    &SLUGS2_B.sf_myMuxFun_i);

  /* DataTypeConversion: '<S581>/Data Type Conversion7' */
  SLUGS2_B.DataTypeConversion7[0] = (real32_T)SLUGS2_B.sf_myMuxFun_i.y[0];
  SLUGS2_B.DataTypeConversion7[1] = (real32_T)SLUGS2_B.sf_myMuxFun_i.y[1];
  SLUGS2_B.DataTypeConversion7[2] = (real32_T)SLUGS2_B.sf_myMuxFun_i.y[2];

  /* MATLAB Function: '<S589>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S581>/Gyro Gains1'
   *  Constant: '<S589>/Constant'
   *  Constant: '<S589>/Constant1'
   *  DataTypeConversion: '<S581>/Data Type Conversion1'
   *  Gain: '<S592>/[ -1 1 -1]'
   *  Product: '<S581>/Divide1'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[3] *
    0.000599060033F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction_kq,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_kq);

  /* MATLAB Function: '<S589>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S581>/Gyro Gains1'
   *  Constant: '<S589>/Constant2'
   *  Constant: '<S589>/Constant3'
   *  DataTypeConversion: '<S581>/Data Type Conversion1'
   *  Product: '<S581>/Divide1'
   */
  S_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[4] *
    0.000599060033F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction1_f,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction1_f);

  /* MATLAB Function: '<S589>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S581>/Gyro Gains1'
   *  Constant: '<S589>/Constant4'
   *  Constant: '<S589>/Constant5'
   *  DataTypeConversion: '<S581>/Data Type Conversion1'
   *  Gain: '<S592>/[ -1 1 -1]'
   *  Product: '<S581>/Divide1'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[5] *
    0.000599060033F), 0.01, 40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction2_f,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction2_f);

  /* MATLAB Function: '<S589>/myMux Fun' */
  SLUGS2_myMuxFun_i(SLUGS2_B.sf_EmbeddedMATLABFunction_kq.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction1_f.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction2_f.y,
                    &SLUGS2_B.sf_myMuxFun_m);

  /* DataTypeConversion: '<S581>/Data Type Conversion3' */
  SLUGS2_B.DataTypeConversion3[0] = (real32_T)SLUGS2_B.sf_myMuxFun_m.y[0];
  SLUGS2_B.DataTypeConversion3[1] = (real32_T)SLUGS2_B.sf_myMuxFun_m.y[1];
  SLUGS2_B.DataTypeConversion3[2] = (real32_T)SLUGS2_B.sf_myMuxFun_m.y[2];

  /* MATLAB Function: '<S590>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S581>/Gyro Gains2'
   *  Constant: '<S590>/Constant'
   *  Constant: '<S590>/Constant1'
   *  DataTypeConversion: '<S581>/Data Type Conversion5'
   *  Product: '<S581>/Divide2'
   */
  S_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[6] * 0.8F), 0.01,
    40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction_ft,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_ft);

  /* MATLAB Function: '<S590>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S581>/Gyro Gains2'
   *  Constant: '<S590>/Constant2'
   *  Constant: '<S590>/Constant3'
   *  DataTypeConversion: '<S581>/Data Type Conversion5'
   *  Gain: '<S593>/[ 1 -1 -1]'
   *  Product: '<S581>/Divide2'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[8] * 0.8F), 0.01,
    40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction1_in,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction1_in);

  /* MATLAB Function: '<S590>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S581>/Gyro Gains2'
   *  Constant: '<S590>/Constant4'
   *  Constant: '<S590>/Constant5'
   *  DataTypeConversion: '<S581>/Data Type Conversion5'
   *  Gain: '<S593>/[ 1 -1 -1]'
   *  Product: '<S581>/Divide2'
   */
  S_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[7] * 0.8F), 0.01,
    40.0, &SLUGS2_B.sf_EmbeddedMATLABFunction2_fu,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* MATLAB Function: '<S590>/myMux Fun' */
  SLUGS2_myMuxFun_i(SLUGS2_B.sf_EmbeddedMATLABFunction_ft.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction1_in.y,
                    SLUGS2_B.sf_EmbeddedMATLABFunction2_fu.y,
                    &SLUGS2_B.sf_myMuxFun_a);

  /* DataTypeConversion: '<S581>/Data Type Conversion6' */
  SLUGS2_B.DataTypeConversion6[0] = (real32_T)SLUGS2_B.sf_myMuxFun_a.y[0];
  SLUGS2_B.DataTypeConversion6[1] = (real32_T)SLUGS2_B.sf_myMuxFun_a.y[1];
  SLUGS2_B.DataTypeConversion6[2] = (real32_T)SLUGS2_B.sf_myMuxFun_a.y[2];

  /* MATLAB Function: '<S614>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat(&SLUGS2_B.sf_EnablesDisablestheComputat_b,
    &SLUGS2_DWork.sf_EnablesDisablestheComputat_b);

  /* Outputs for Enabled SubSystem: '<S614>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S629>/Enable'
   */
  if (SLUGS2_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* DataTypeConversion: '<S629>/Data Type Conversion' */
    rtb_DataTypeConversion_n = rtb_Sum_d;

    /* DiscreteZeroPole: '<S632>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_i = 0.014778325123152709*rtb_DataTypeConversion_n;
      rtb_DiscreteZeroPole_i += 0.029119852459414206*
        SLUGS2_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S629>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S629>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_i > 120000.0F) {
      SLUGS2_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_i < 80000.0F) {
      SLUGS2_B.u0k120k = 80000.0F;
    } else {
      SLUGS2_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_i;
    }

    /* End of Saturate: '<S629>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S632>/Discrete Zero-Pole' */
    {
      SLUGS2_DWork.DiscreteZeroPole_DSTATE = rtb_DataTypeConversion_n +
        0.97044334975369462*SLUGS2_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S614>/Initial Baro Bias' */

  /* Product: '<S625>/Divide' incorporates:
   *  Sum: '<S625>/Sum2'
   */
  rtb_Sum_o = (rtb_Sum_d - SLUGS2_B.u0k120k) / SLUGS2_B.u0k120k;

  /* S-Function (MCHP_C_function_Call): '<S628>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &SLUGS2_B.GettheGSLocationupdateSensorMCU[0]
                );

  /* Sum: '<S625>/Sum1' incorporates:
   *  Constant: '<S625>/Constant2'
   *  Constant: '<S625>/Constant3'
   *  Constant: '<S625>/Constant4'
   *  Constant: '<S625>/Constant5'
   *  Gain: '<S631>/Unit Conversion'
   *  Product: '<S625>/Divide1'
   *  Product: '<S625>/Divide2'
   *  Product: '<S625>/Divide3'
   *  Product: '<S625>/Divide4'
   *  Sum: '<S625>/Sum3'
   */
  rtb_Sum_o = ((rtb_Sum_o * rtb_Sum_o * 0.093502529F + rtb_Sum_o * -0.188893303F)
               + 2.18031291E-5F) * 145473.5F * 0.3048F +
    SLUGS2_B.GettheGSLocationupdateSensorMCU[0];

  /* Outputs for Enabled SubSystem: '<S614>/Zero Out Height' incorporates:
   *  EnablePort: '<S630>/Enable'
   */
  if (SLUGS2_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* Sum: '<S630>/Sum' incorporates:
     *  Delay: '<S630>/Integer Delay'
     */
    SLUGS2_B.Sum = SLUGS2_B.GettheGSLocationupdateSensorMCU[0] -
      SLUGS2_DWork.IntegerDelay_DSTATE;

    /* Update for Delay: '<S630>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE = rtb_Sum_o;
  }

  /* End of Outputs for SubSystem: '<S614>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S614>/Enabled Subsystem' */

  /* Logic: '<S614>/Logical Operator' incorporates:
   *  Sum: '<S614>/Sum1'
   */
  SLUGS2_EnabledSubsystem(!(SLUGS2_B.sf_EnablesDisablestheComputat_b.tOut != 0.0),
    SLUGS2_B.Sum + rtb_Sum_o, &SLUGS2_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S614>/Enabled Subsystem' */

  /* Switch: '<S573>/Switch1' */
  for (i = 0; i < 5; i++) {
    if (rtb_LogicalOperator_df) {
      SLUGS2_B.Switch1[i] = SLUGS2_B.ProducetheGPSMainDataandupdat_a[i];
    } else {
      SLUGS2_B.Switch1[i] = SLUGS2_B.ProducetheGPSMainDataandupdatet[i];
    }
  }

  /* End of Switch: '<S573>/Switch1' */

  /* S-Function (MCHP_C_function_Call): '<S13>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  SLUGS2_B.ChecksifFixTypeis3updateSensorM = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S13>/Initialize GS Location' incorporates:
   *  EnablePort: '<S571>/Enable'
   */
  /* Logic: '<S13>/Logical Operator' incorporates:
   *  DataStoreRead: '<S13>/Data Store Read'
   */
  if ((SLUGS2_B.ChecksifFixTypeis3updateSensorM != 0) &&
      (SLUGS2_DWork.GS_INIT_FLAG != 0.0)) {
    /* DataStoreWrite: '<S571>/Data Store Write' incorporates:
     *  Constant: '<S571>/Constant'
     */
    SLUGS2_DWork.GS_INIT_FLAG = 0.0;

    /* DataStoreWrite: '<S571>/Data Store Write1' incorporates:
     *  DataStoreRead: '<S571>/Data Store Read1'
     *  DataTypeConversion: '<S571>/Data Type Conversion'
     *  DataTypeConversion: '<S571>/Data Type Conversion1'
     *  DataTypeConversion: '<S571>/Data Type Conversion2'
     */
    mlGSLocationFloat.lat = (real32_T)mlGpsData.lat;
    mlGSLocationFloat.lon = (real32_T)mlGpsData.lon;
    mlGSLocationFloat.alt = (real32_T)mlGpsData.alt;
  }

  /* End of Logic: '<S13>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S13>/Initialize GS Location' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
