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
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.241
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Nov 05 08:28:55 2016
 */

#include "Sensor_Data_Adapter.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S583>/Embedded MATLAB Function1'
 *    '<S583>/Embedded MATLAB Function2'
 *    '<S591>/Embedded MATLAB Function1'
 *    '<S591>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunction1_m(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction1_e_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/Embedded MATLAB Function1': '<S585>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S585>:1:5' */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/*
 * Output and update for atomic system:
 *    '<S583>/myMux Fun'
 *    '<S591>/myMux Fun'
 */
void AUAV_V3_TestSenso_myMuxFun(int16_T rtu_u1, int16_T rtu_u2, int16_T rtu_u3,
  int16_T rtu_u4, rtB_myMuxFun_AUAV_V3_TestSens_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/myMux Fun': '<S587>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S587>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
  localB->y[3] = rtu_u4;
}

/*
 * Initial conditions for atomic system:
 *    '<S596>/Embedded MATLAB Function'
 *    '<S596>/Embedded MATLAB Function1'
 *    '<S596>/Embedded MATLAB Function2'
 *    '<S597>/Embedded MATLAB Function'
 *    '<S597>/Embedded MATLAB Function1'
 *    '<S597>/Embedded MATLAB Function2'
 *    '<S598>/Embedded MATLAB Function'
 *    '<S598>/Embedded MATLAB Function1'
 *    '<S598>/Embedded MATLAB Function2'
 *    '<S629>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunct_m_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S596>/Embedded MATLAB Function'
 *    '<S596>/Embedded MATLAB Function1'
 *    '<S596>/Embedded MATLAB Function2'
 *    '<S597>/Embedded MATLAB Function'
 *    '<S597>/Embedded MATLAB Function1'
 *    '<S597>/Embedded MATLAB Function2'
 *    '<S598>/Embedded MATLAB Function'
 *    '<S598>/Embedded MATLAB Function1'
 *    '<S598>/Embedded MATLAB Function2'
 *    '<S629>/Embedded MATLAB Function'
 *    ...
 */
void A_EmbeddedMATLABFunction_f(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_gp_T *localB, rtDW_EmbeddedMATLABFunction_i_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S602>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S602>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S602>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S602>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S602>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S602>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S602>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S602>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S602>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S602>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S596>/myMux Fun'
 *    '<S597>/myMux Fun'
 *    '<S598>/myMux Fun'
 */
void AUAV_V3_TestSen_myMuxFun_i(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_AUAV_V3_TestSe_i_T *localB)
{
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S605>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S605>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/* Initial conditions for atomic system: '<Root>/Sensor_Data_Adapter' */
void A_Sensor_Data_Adapter_Init(void)
{
  /* InitializeConditions for MATLAB Function: '<S629>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_g);

  /* InitializeConditions for MATLAB Function: '<S631>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_lw);

  /* InitializeConditions for MATLAB Function: '<S632>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_cn);

  /* InitializeConditions for MATLAB Function: '<S630>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_np);

  /* InitializeConditions for MATLAB Function: '<S596>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_f);

  /* InitializeConditions for MATLAB Function: '<S596>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_i);

  /* InitializeConditions for MATLAB Function: '<S596>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_h);

  /* InitializeConditions for MATLAB Function: '<S597>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_kq);

  /* InitializeConditions for MATLAB Function: '<S597>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_f);

  /* InitializeConditions for MATLAB Function: '<S597>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_f);

  /* InitializeConditions for MATLAB Function: '<S598>/Embedded MATLAB Function' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ft);

  /* InitializeConditions for MATLAB Function: '<S598>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_in);

  /* InitializeConditions for MATLAB Function: '<S598>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_m_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* InitializeConditions for MATLAB Function: '<S622>/Enables//Disables the Computation of  initial Baro Bias' */
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
  int16_T rtb_DataTypeConversion1_ax;
  int16_T rtb_DataTypeConversion2_c;
  int16_T i;
  real_T tmp;
  real_T tmp_0;

  /* Outputs for Enabled SubSystem: '<S16>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S580>/Enable'
   */
  if (AUAV_V3_TestSensors_B.HILManualSwitch > 0.0) {
    /* Outputs for Enabled SubSystem: '<S580>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S582>/Enable'
     */
    /* DataStoreRead: '<S580>/Data Store Read' */
    if (AUAV_V3_TestSensors_DWork.SIX_DOF_HIL_FLAG > 0.0) {
      /* S-Function (MCHP_C_function_Call): '<S582>/Data from HIL [hil.c]2' */
      hilRead(
              &AUAV_V3_TestSensors_B.DatafromHILhilc2[0]
              );

      /* S-Function (MCHP_C_function_Call): '<S582>/HIL Messages  Parser//Decoder [hil.c]1' */
      protDecodeHil(
                    &AUAV_V3_TestSensors_B.DatafromHILhilc2[0]
                    );

      /* S-Function (MCHP_C_function_Call): '<S582>/HIL Raw Readings [hil.c]1' */
      hil_getRawRead(
                     &AUAV_V3_TestSensors_B.HILRawReadingshilc1_k[0]
                     );
    }

    /* End of DataStoreRead: '<S580>/Data Store Read' */
    /* End of Outputs for SubSystem: '<S580>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S580>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S583>/Enable'
     */
    /* DataStoreRead: '<S580>/Data Store Read1' */
    if (AUAV_V3_TestSensors_DWork.X_PLANE_HIL_FLAG > 0.0) {
      /* S-Function (MCHP_C_function_Call): '<S583>/Data from HIL X-Plane' */
      HILSIM_set_gplane(
                        );

      /* S-Function (MCHP_C_function_Call): '<S583>/Data from HIL X-Plane2' */
      HILSIM_set_omegagyro(
                           );

      /* S-Function (MCHP_C_function_Call): '<S583>/HIL Raw Readings [hil.c]1' */
      hil_getRawRead(
                     &AUAV_V3_TestSensors_B.HILRawReadingshilc1[0]
                     );

      /* MATLAB Function: '<S583>/Embedded MATLAB Function1' incorporates:
       *  Constant: '<S583>/Constant1'
       */
      EmbeddedMATLABFunction1_m(AUAV_V3_TestSensors_ConstP.pooled16,
        &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_m);

      /* DataTypeConversion: '<S583>/Data Type Conversion1' */
      tmp_0 = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_m.y);
      if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
        tmp_0 = 0.0;
      } else {
        tmp_0 = fmod(tmp_0, 65536.0);
      }

      /* MATLAB Function: '<S583>/Embedded MATLAB Function2' incorporates:
       *  Constant: '<S583>/Constant2'
       */
      EmbeddedMATLABFunction1_m(AUAV_V3_TestSensors_ConstP.pooled16,
        &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_i);

      /* DataTypeConversion: '<S583>/Data Type Conversion' incorporates:
       *  DataStoreRead: '<S583>/Get mlAirData1'
       */
      rtb_Sum_o = (real32_T)floor(mlAirData.press_abs);
      if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
        rtb_Sum_o = 0.0F;
      } else {
        rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
      }

      /* DataTypeConversion: '<S583>/Data Type Conversion2' */
      tmp = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_i.y);
      if (rtIsNaN(tmp) || rtIsInf(tmp)) {
        tmp = 0.0;
      } else {
        tmp = fmod(tmp, 65536.0);
      }

      /* MATLAB Function: '<S583>/myMux Fun' incorporates:
       *  DataStoreRead: '<S583>/Get mlAirData2'
       *  DataTypeConversion: '<S583>/Data Type Conversion'
       *  DataTypeConversion: '<S583>/Data Type Conversion1'
       *  DataTypeConversion: '<S583>/Data Type Conversion2'
       */
      AUAV_V3_TestSenso_myMuxFun(rtb_Sum_o < 0.0F ? -(int16_T)(uint16_T)
        -rtb_Sum_o : (int16_T)(uint16_T)rtb_Sum_o, tmp_0 < 0.0 ? -(int16_T)
        (uint16_T)-tmp_0 : (int16_T)(uint16_T)tmp_0, tmp < 0.0 ? -(int16_T)
        (uint16_T)-tmp : (int16_T)(uint16_T)tmp, mlAirData.temperature,
        &AUAV_V3_TestSensors_B.sf_myMuxFun_h);

      /* S-Function (MCHP_C_function_Call): '<S583>/Update State AP ADC Data [updateSensorMcuState.c]1' */
      updateRawADCData(
                       &AUAV_V3_TestSensors_B.sf_myMuxFun_h.y[0]
                       );
    }

    /* End of Outputs for SubSystem: '<S580>/Enabled Subsystem1' */

    /* Switch: '<S580>/Switch' incorporates:
     *  DataStoreRead: '<S580>/Data Store Read1'
     */
    for (i = 0; i < 13; i++) {
      if (AUAV_V3_TestSensors_DWork.X_PLANE_HIL_FLAG != 0.0) {
        AUAV_V3_TestSensors_B.Switch_i[i] =
          AUAV_V3_TestSensors_B.HILRawReadingshilc1[i];
      } else {
        AUAV_V3_TestSensors_B.Switch_i[i] =
          AUAV_V3_TestSensors_B.HILRawReadingshilc1_k[i];
      }
    }

    /* End of Switch: '<S580>/Switch' */
  }

  /* End of Outputs for SubSystem: '<S16>/Raw HIL  Readings' */

  /* Logic: '<S581>/Logical Operator' */
  rtb_LogicalOperator = !(AUAV_V3_TestSensors_B.HILManualSwitch != 0.0);

  /* Outputs for Enabled SubSystem: '<S581>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S591>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* DataTypeConversion: '<S591>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S591>/Get mlAirData1'
     */
    rtb_Sum_o = (real32_T)floor(mlAirData.press_abs);
    if (rtIsNaNF(rtb_Sum_o) || rtIsInfF(rtb_Sum_o)) {
      rtb_Sum_o = 0.0F;
    } else {
      rtb_Sum_o = (real32_T)fmod(rtb_Sum_o, 65536.0F);
    }

    i = rtb_Sum_o < 0.0F ? -(int16_T)(uint16_T)-rtb_Sum_o : (int16_T)(uint16_T)
      rtb_Sum_o;

    /* End of DataTypeConversion: '<S591>/Data Type Conversion' */

    /* MATLAB Function: '<S591>/Embedded MATLAB Function1' incorporates:
     *  Constant: '<S591>/Constant1'
     */
    EmbeddedMATLABFunction1_m(AUAV_V3_TestSensors_ConstP.pooled16,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_ib);

    /* DataTypeConversion: '<S591>/Data Type Conversion1' */
    tmp_0 = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_ib.y);
    if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
      tmp_0 = 0.0;
    } else {
      tmp_0 = fmod(tmp_0, 65536.0);
    }

    rtb_DataTypeConversion1_ax = tmp_0 < 0.0 ? -(int16_T)(uint16_T)-tmp_0 :
      (int16_T)(uint16_T)tmp_0;

    /* End of DataTypeConversion: '<S591>/Data Type Conversion1' */

    /* MATLAB Function: '<S591>/Embedded MATLAB Function2' incorporates:
     *  Constant: '<S591>/Constant2'
     */
    EmbeddedMATLABFunction1_m(AUAV_V3_TestSensors_ConstP.pooled16,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_p);

    /* DataTypeConversion: '<S591>/Data Type Conversion2' */
    tmp_0 = floor(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_p.y);
    if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
      tmp_0 = 0.0;
    } else {
      tmp_0 = fmod(tmp_0, 65536.0);
    }

    rtb_DataTypeConversion2_c = tmp_0 < 0.0 ? -(int16_T)(uint16_T)-tmp_0 :
      (int16_T)(uint16_T)tmp_0;

    /* End of DataTypeConversion: '<S591>/Data Type Conversion2' */

    /* MATLAB Function: '<S591>/myMux Fun' incorporates:
     *  DataStoreRead: '<S591>/Get mlAirData2'
     */
    AUAV_V3_TestSenso_myMuxFun(i, rtb_DataTypeConversion1_ax,
      rtb_DataTypeConversion2_c, mlAirData.temperature,
      &AUAV_V3_TestSensors_B.sf_myMuxFun_n);

    /* S-Function (MCHP_C_function_Call): '<S591>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     &AUAV_V3_TestSensors_B.sf_myMuxFun_n.y[0]
                     );

    /* S-Function (MCHP_C_function_Call): '<S591>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                &AUAV_V3_TestSensors_B.ReadtheCubeDataadisCube16405c1[0]
                );

    /* MATLAB Function: '<S591>/myMux Fun4' incorporates:
     *  DataStoreRead: '<S591>/Get mlAirData2'
     */
    /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S621>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S621>:1:5' */
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
    AUAV_V3_TestSensors_B.y_l[9] = i;
    AUAV_V3_TestSensors_B.y_l[10] = rtb_DataTypeConversion1_ax;
    AUAV_V3_TestSensors_B.y_l[11] = rtb_DataTypeConversion2_c;
    AUAV_V3_TestSensors_B.y_l[12] = mlAirData.temperature;

    /* S-Function (MCHP_C_function_Call): '<S591>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    AUAV_V3_TestSensors_B.IstheGPSNovatelorUbloxgpsPortc1 = isGPSNovatel(
      );

    /* Outputs for Enabled SubSystem: '<S591>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S619>/Enable'
     */
    /* Logic: '<S591>/Logical Operator' */
    if (!(AUAV_V3_TestSensors_B.IstheGPSNovatelorUbloxgpsPortc1 != 0)) {
      /* S-Function (MCHP_C_function_Call): '<S619>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdat_a[
                          0]
                          );
    }

    /* End of Logic: '<S591>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S591>/if GPS is Ublox' */
  }

  /* End of Outputs for SubSystem: '<S581>/If no HIL then Read all the Sensors' */

  /* Switch: '<S581>/Switch' */
  for (i = 0; i < 13; i++) {
    if (AUAV_V3_TestSensors_B.HILManualSwitch != 0.0) {
      rtb_Switch_d[i] = AUAV_V3_TestSensors_B.Switch_i[i];
    } else {
      rtb_Switch_d[i] = AUAV_V3_TestSensors_B.y_l[i];
    }
  }

  /* End of Switch: '<S581>/Switch' */

  /* MATLAB Function: '<S629>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S629>/Constant'
   *  Constant: '<S629>/Constant1'
   *  DataTypeConversion: '<S593>/Data Type Conversion5'
   */
  A_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[12], 0.01, 0.02,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_g,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_g);

  /* Sum: '<S628>/Sum' incorporates:
   *  Constant: '<S628>/Bias'
   *  Product: '<S628>/Divide'
   */
  rtb_Sum_o = (real32_T)(1.5112853050231934 *
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_g.y) + -1605.28198F;

  /* RelationalOperator: '<S641>/Compare' incorporates:
   *  Constant: '<S641>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -130.0F);

  /* Outputs for Enabled SubSystem: '<S626>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S642>/Enable'
   */
  /* Logic: '<S626>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S642>/Sum2' incorporates:
     *  Constant: '<S642>/Mean Temperature for Calibration'
     *  Constant: '<S642>/gains'
     *  Product: '<S642>/Divide1'
     *  Sum: '<S642>/Sum1'
     */
    AUAV_V3_TestSensors_B.Merge = (real32_T)rtb_Switch_d[10] - (rtb_Sum_o -
      293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S626>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S626>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S626>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S643>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S643>/Add' incorporates:
     *  Constant: '<S643>/Constant'
     *  Constant: '<S643>/Mean Temperature for Calibration'
     *  Constant: '<S643>/gains'
     *  Product: '<S643>/Divide1'
     *  Sum: '<S643>/Sum1'
     *  Sum: '<S643>/Sum2'
     */
    AUAV_V3_TestSensors_B.Merge = ((real32_T)rtb_Switch_d[10] - (rtb_Sum_o -
      -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S626>/Lo Temp Compensation' */

  /* MATLAB Function: '<S631>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S631>/Constant'
   *  Constant: '<S631>/Constant1'
   *  DataTypeConversion: '<S593>/Data Type Conversion3'
   */
  A_EmbeddedMATLABFunction_f((real_T)AUAV_V3_TestSensors_B.Merge, 0.01, 4.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lw,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_lw);

  /* Sum: '<S624>/Sum' incorporates:
   *  Constant: '<S624>/Bias'
   *  Constant: '<S624>/Gains'
   *  DataTypeConversion: '<S593>/Data Type Conversion4'
   *  Product: '<S624>/Divide'
   */
  rtb_u001maxDynPress = 1.05137849F * (real32_T)
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lw.y + -1005.87872F;

  /* Saturate: '<S593>/[0.001  maxDynPress]' */
  if (rtb_u001maxDynPress > 3000.0F) {
    rtb_u001maxDynPress = 3000.0F;
  } else {
    if (rtb_u001maxDynPress < 0.001F) {
      rtb_u001maxDynPress = 0.001F;
    }
  }

  /* End of Saturate: '<S593>/[0.001  maxDynPress]' */

  /* RelationalOperator: '<S644>/Compare' incorporates:
   *  Constant: '<S644>/Constant'
   */
  rtb_Compare_gl = (uint8_T)(rtb_Sum_o < -50.0F);

  /* Outputs for Enabled SubSystem: '<S627>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S645>/Enable'
   */
  /* Logic: '<S627>/Logical Operator' */
  if (!(rtb_Compare_gl != 0)) {
    /* Sum: '<S645>/Add' incorporates:
     *  Constant: '<S645>/Constant'
     *  Constant: '<S645>/Mean Temperature for Calibration'
     *  Constant: '<S645>/gains'
     *  Product: '<S645>/Divide1'
     *  Sum: '<S645>/Sum1'
     *  Sum: '<S645>/Sum2'
     */
    AUAV_V3_TestSensors_B.Merge_j = ((real32_T)rtb_Switch_d[9] - (rtb_Sum_o -
      347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S627>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S627>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S627>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S646>/Enable'
   */
  if (rtb_Compare_gl > 0) {
    /* Sum: '<S646>/Sum2' incorporates:
     *  Constant: '<S646>/Mean Temperature for Calibration'
     *  Constant: '<S646>/gains'
     *  Product: '<S646>/Divide1'
     *  Sum: '<S646>/Sum1'
     */
    AUAV_V3_TestSensors_B.Merge_j = (real32_T)rtb_Switch_d[9] - (rtb_Sum_o -
      -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S627>/Lo Temp Compensation' */

  /* MATLAB Function: '<S632>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S632>/Constant'
   *  Constant: '<S632>/Constant1'
   *  DataTypeConversion: '<S593>/Data Type Conversion19'
   */
  A_EmbeddedMATLABFunction_f((real_T)AUAV_V3_TestSensors_B.Merge_j, 0.01, 4.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_cn,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_cn);

  /* Sum: '<S623>/Sum' incorporates:
   *  Constant: '<S623>/Bias'
   *  Constant: '<S623>/Gains'
   *  DataTypeConversion: '<S593>/Data Type Conversion1'
   *  Product: '<S623>/Divide'
   */
  rtb_Sum_d = 27.127F * (real32_T)
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_cn.y + 9444.44434F;

  /* MATLAB Function: '<S581>/myMux Fun' */
  AUAV_V3_TestSe_myMuxFun1_e(rtb_u001maxDynPress, rtb_Sum_d, rtb_Sum_o,
    &AUAV_V3_TestSensors_B.sf_myMuxFun);

  /* Outputs for Enabled SubSystem: '<S581>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S592>/Enable'
   */
  /* Inport: '<S592>/AirData' */
  AUAV_V3_TestSensors_B.AirData[0] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[0];
  AUAV_V3_TestSensors_B.AirData[1] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[1];
  AUAV_V3_TestSensors_B.AirData[2] = AUAV_V3_TestSensors_B.sf_myMuxFun.y[2];

  /* S-Function (MCHP_C_function_Call): '<S592>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
  updateAirData(
                &AUAV_V3_TestSensors_B.AirData[0]
                );

  /* End of Outputs for SubSystem: '<S581>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S581>/myMux Fun1' */
  /* MATLAB Function 'Sensor_Data_Adapter/Sensor Suite/myMux Fun1': '<S595>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S595>:1:5' */
  AUAV_V3_TestSensors_B.y_o[0] = rtb_u001maxDynPress;
  AUAV_V3_TestSensors_B.y_o[1] = AUAV_V3_TestSensors_B.AirData[0];
  AUAV_V3_TestSensors_B.y_o[2] = 0.0F;
  AUAV_V3_TestSensors_B.y_o[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S581>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   &AUAV_V3_TestSensors_B.y_o[0]
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S590>/Calculus Time Step1 */
  AUAV_V3_TestSensors_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S590>/Calculus Time Step2 */
  {
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    AUAV_V3_TestSensors_B.CalculusTimeStep2 = tmp;
  }

  /* DataTypeConversion: '<S590>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S590>/Data Type Conversion1'
   *  DataTypeConversion: '<S590>/Data Type Conversion2'
   *  Gain: '<S590>/Gain'
   *  Product: '<S590>/Divide'
   *  Rounding: '<S590>/Rounding Function'
   */
  tmp_0 = floor((real_T)AUAV_V3_TestSensors_B.CalculusTimeStep1 / (real_T)
                AUAV_V3_TestSensors_B.CalculusTimeStep2 * 100.0);
  if (rtIsNaN(tmp_0) || rtIsInf(tmp_0)) {
    tmp_0 = 0.0;
  } else {
    tmp_0 = fmod(tmp_0, 256.0);
  }

  AUAV_V3_TestSensors_B.DataTypeConversion12 = (uint8_T)(tmp_0 < 0.0 ? (int16_T)
    (uint8_T)-(int8_T)(uint8_T)-tmp_0 : (int16_T)(uint8_T)tmp_0);

  /* End of DataTypeConversion: '<S590>/Data Type Conversion12' */

  /* MATLAB Function: '<S630>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S630>/Constant'
   *  Constant: '<S630>/Constant1'
   *  DataTypeConversion: '<S593>/Data Type Conversion6'
   */
  A_EmbeddedMATLABFunction_f((real_T)rtb_Switch_d[11], 0.01, 0.02,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_np,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_np);

  /* DataTypeConversion: '<S593>/Data Type Conversion8' incorporates:
   *  Constant: '<S625>/Bias'
   *  Product: '<S625>/Divide'
   *  Sum: '<S625>/Sum'
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

  /* End of DataTypeConversion: '<S593>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S581>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 AUAV_V3_TestSensors_B.DataTypeConversion12
                 , AUAV_V3_TestSensors_B.DataTypeConversion8
                 );

  /* S-Function (MCHP_C_function_Call): '<S588>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  getGpsUbloxData(
                  &AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[0]
                  );

  /* MATLAB Function: '<S596>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S589>/Gyro Gains'
   *  Constant: '<S596>/Constant'
   *  Constant: '<S596>/Constant1'
   *  DataTypeConversion: '<S589>/Data Type Conversion2'
   *  Gain: '<S599>/[ -1 1 -1]'
   *  Product: '<S589>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[0] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_f);

  /* MATLAB Function: '<S596>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S589>/Gyro Gains'
   *  Constant: '<S596>/Constant2'
   *  Constant: '<S596>/Constant3'
   *  DataTypeConversion: '<S589>/Data Type Conversion2'
   *  Product: '<S589>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[1] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_i,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_i);

  /* MATLAB Function: '<S596>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S589>/Gyro Gains'
   *  Constant: '<S596>/Constant4'
   *  Constant: '<S596>/Constant5'
   *  DataTypeConversion: '<S589>/Data Type Conversion2'
   *  Gain: '<S599>/[ -1 1 -1]'
   *  Product: '<S589>/Divide'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[2] *
    0.000266462477F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_h,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_h);

  /* MATLAB Function: '<S596>/myMux Fun' */
  AUAV_V3_TestSen_myMuxFun_i(AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_f.y,
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_i.y,
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_h.y,
    &AUAV_V3_TestSensors_B.sf_myMuxFun_i);

  /* DataTypeConversion: '<S589>/Data Type Conversion7' */
  AUAV_V3_TestSensors_B.DataTypeConversion7[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion7[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion7[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_i.y[2];

  /* MATLAB Function: '<S597>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S589>/Gyro Gains1'
   *  Constant: '<S597>/Constant'
   *  Constant: '<S597>/Constant1'
   *  DataTypeConversion: '<S589>/Data Type Conversion1'
   *  Gain: '<S600>/[ -1 1 -1]'
   *  Product: '<S589>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[3] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_kq,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_kq);

  /* MATLAB Function: '<S597>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S589>/Gyro Gains1'
   *  Constant: '<S597>/Constant2'
   *  Constant: '<S597>/Constant3'
   *  DataTypeConversion: '<S589>/Data Type Conversion1'
   *  Product: '<S589>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[4] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_f);

  /* MATLAB Function: '<S597>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S589>/Gyro Gains1'
   *  Constant: '<S597>/Constant4'
   *  Constant: '<S597>/Constant5'
   *  DataTypeConversion: '<S589>/Data Type Conversion1'
   *  Gain: '<S600>/[ -1 1 -1]'
   *  Product: '<S589>/Divide1'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[5] *
    0.000599060033F), 0.01, 40.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_f,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_f);

  /* MATLAB Function: '<S597>/myMux Fun' */
  AUAV_V3_TestSen_myMuxFun_i
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_kq.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_f.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_f.y,
     &AUAV_V3_TestSensors_B.sf_myMuxFun_m);

  /* DataTypeConversion: '<S589>/Data Type Conversion3' */
  AUAV_V3_TestSensors_B.DataTypeConversion3[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion3[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion3[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_m.y[2];

  /* MATLAB Function: '<S598>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S589>/Gyro Gains2'
   *  Constant: '<S598>/Constant'
   *  Constant: '<S598>/Constant1'
   *  DataTypeConversion: '<S589>/Data Type Conversion5'
   *  Product: '<S589>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)((real32_T)rtb_Switch_d[6] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ft,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_ft);

  /* MATLAB Function: '<S598>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S589>/Gyro Gains2'
   *  Constant: '<S598>/Constant2'
   *  Constant: '<S598>/Constant3'
   *  DataTypeConversion: '<S589>/Data Type Conversion5'
   *  Gain: '<S601>/[ 1 -1 -1]'
   *  Product: '<S589>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[8] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_in,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction1_in);

  /* MATLAB Function: '<S598>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S589>/Gyro Gains2'
   *  Constant: '<S598>/Constant4'
   *  Constant: '<S598>/Constant5'
   *  DataTypeConversion: '<S589>/Data Type Conversion5'
   *  Gain: '<S601>/[ 1 -1 -1]'
   *  Product: '<S589>/Divide2'
   */
  A_EmbeddedMATLABFunction_f((real_T)-((real32_T)rtb_Switch_d[7] * 0.8F), 0.01,
    40.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_fu,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction2_fu);

  /* MATLAB Function: '<S598>/myMux Fun' */
  AUAV_V3_TestSen_myMuxFun_i
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ft.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1_in.y,
     AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction2_fu.y,
     &AUAV_V3_TestSensors_B.sf_myMuxFun_a);

  /* DataTypeConversion: '<S589>/Data Type Conversion6' */
  AUAV_V3_TestSensors_B.DataTypeConversion6[0] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[0];
  AUAV_V3_TestSensors_B.DataTypeConversion6[1] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[1];
  AUAV_V3_TestSensors_B.DataTypeConversion6[2] = (real32_T)
    AUAV_V3_TestSensors_B.sf_myMuxFun_a.y[2];

  /* MATLAB Function: '<S622>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b,
     &AUAV_V3_TestSensors_DWork.sf_EnablesDisablestheComputat_b);

  /* Outputs for Enabled SubSystem: '<S622>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S637>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* DataTypeConversion: '<S637>/Data Type Conversion' */
    rtb_DataTypeConversion_n = rtb_Sum_d;

    /* DiscreteZeroPole: '<S640>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_i = 0.014778325123152709*rtb_DataTypeConversion_n;
      rtb_DiscreteZeroPole_i += 0.029119852459414206*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S637>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S637>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_i > 120000.0F) {
      AUAV_V3_TestSensors_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_i < 80000.0F) {
      AUAV_V3_TestSensors_B.u0k120k = 80000.0F;
    } else {
      AUAV_V3_TestSensors_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_i;
    }

    /* End of Saturate: '<S637>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S640>/Discrete Zero-Pole' */
    {
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE =
        rtb_DataTypeConversion_n + 0.97044334975369462*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S622>/Initial Baro Bias' */

  /* Product: '<S633>/Divide' incorporates:
   *  Sum: '<S633>/Sum2'
   */
  rtb_Sum_o = (rtb_Sum_d - AUAV_V3_TestSensors_B.u0k120k) /
    AUAV_V3_TestSensors_B.u0k120k;

  /* S-Function (MCHP_C_function_Call): '<S636>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0]
                );

  /* Sum: '<S633>/Sum1' incorporates:
   *  Constant: '<S633>/Constant2'
   *  Constant: '<S633>/Constant3'
   *  Constant: '<S633>/Constant4'
   *  Constant: '<S633>/Constant5'
   *  Gain: '<S639>/Unit Conversion'
   *  Product: '<S633>/Divide1'
   *  Product: '<S633>/Divide2'
   *  Product: '<S633>/Divide3'
   *  Product: '<S633>/Divide4'
   *  Sum: '<S633>/Sum3'
   */
  rtb_Sum_o = ((rtb_Sum_o * rtb_Sum_o * 0.093502529F + rtb_Sum_o * -0.188893303F)
               + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0];

  /* Outputs for Enabled SubSystem: '<S622>/Zero Out Height' incorporates:
   *  EnablePort: '<S638>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut > 0.0) {
    /* Sum: '<S638>/Sum' incorporates:
     *  Delay: '<S638>/Integer Delay'
     */
    AUAV_V3_TestSensors_B.Sum =
      AUAV_V3_TestSensors_B.GettheGSLocationupdateSensorMCU[0] -
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE;

    /* Update for Delay: '<S638>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE = rtb_Sum_o;
  }

  /* End of Outputs for SubSystem: '<S622>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S622>/Enabled Subsystem' */

  /* Logic: '<S622>/Logical Operator' incorporates:
   *  Sum: '<S622>/Sum1'
   */
  AUAV_V3_T_EnabledSubsystem
    (!(AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputat_b.tOut != 0.0),
     AUAV_V3_TestSensors_B.Sum + rtb_Sum_o,
     &AUAV_V3_TestSensors_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S622>/Enabled Subsystem' */

  /* Switch: '<S581>/Switch1' */
  for (i = 0; i < 5; i++) {
    if (rtb_LogicalOperator) {
      AUAV_V3_TestSensors_B.Switch1[i] =
        AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdat_a[i];
    } else {
      AUAV_V3_TestSensors_B.Switch1[i] =
        AUAV_V3_TestSensors_B.ProducetheGPSMainDataandupdatet[i];
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

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
