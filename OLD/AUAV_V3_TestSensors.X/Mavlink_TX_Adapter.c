/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Mavlink_TX_Adapter.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
 */

#include "Mavlink_TX_Adapter.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID0(void)
{
  int16_T tmp;
  real32_T tmp_0;

  /* DataStoreWrite: '<S8>/Set VfrHud' incorporates:
   *  DataStoreRead: '<S8>/Get VfrHud1'
   *  DataStoreRead: '<S8>/Get VfrHud2'
   *  DataStoreRead: '<S8>/Get VfrHud4'
   *  DataTypeConversion: '<S8>/Data Type Conversion2'
   *  Gain: '<S8>/Gain'
   */
  mlVfr_hud.airspeed = mlNavigation.u_m;
  mlVfr_hud.groundspeed = (real32_T)(41943UL * mlGpsData.vel) * 2.38418579E-7F;
  mlVfr_hud.alt = mlLocalPositionData.z;
  mlVfr_hud.climb = mlLocalPositionData.vz;

  /* DataTypeConversion: '<S8>/Data Type Conversion1' incorporates:
   *  DataStoreRead: '<S8>/Get VfrHud3'
   *  Gain: '<S8>/Gain1'
   */
  tmp_0 = (real32_T)floor(57.2957802F * mlAttitudeSol.yaw);
  if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
    tmp_0 = 0.0F;
  } else {
    tmp_0 = (real32_T)fmod(tmp_0, 65536.0F);
  }

  /* DataStoreWrite: '<S8>/Set VfrHud' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion1'
   */
  mlVfr_hud.heading = tmp_0 < 0.0F ? -(int16_T)(uint16_T)-tmp_0 : (int16_T)
    (uint16_T)tmp_0;

  /* DataTypeConversion: '<S8>/Data Type Conversion5' incorporates:
   *  Constant: '<S8>/Constant1'
   *  DataStoreRead: '<S8>/Get VfrHud5'
   *  Gain: '<S8>/Gain2'
   *  Sum: '<S8>/Sum1'
   */
  tmp = (int16_T)fmod((int16_T)floor(((real_T)mlPwmCommands.servo1_raw - 189.0) *
    0.135), 65536.0);

  /* DataStoreWrite: '<S8>/Set VfrHud' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion5'
   */
  mlVfr_hud.throttle = tmp < 0 ? (uint16_T)-(int16_T)(uint16_T)-(real_T)tmp :
    (uint16_T)tmp;

  /* RateTransition: '<S8>/RawIMU Rate Transition' */
  if (!(AUAV_V3_TestSensors_DWork.RawIMURateTransition_semaphoreT != 0)) {
    AUAV_V3_TestSensors_DWork.RawIMURateTransition_Buffer0 = mlRawImuData;
  }

  /* End of RateTransition: '<S8>/RawIMU Rate Transition' */

  /* RateTransition: '<S8>/Attitude Rate Transition' */
  if (!(AUAV_V3_TestSensors_DWork.AttitudeRateTransition_semaphor != 0)) {
    AUAV_V3_TestSensors_DWork.AttitudeRateTransition_Buffer0 = mlAttitudeSol;
  }

  /* End of RateTransition: '<S8>/Attitude Rate Transition' */

  /* RateTransition: '<S8>/RC Rate Transition' */
  if (!(AUAV_V3_TestSensors_DWork.RCRateTransition_semaphoreTaken != 0)) {
    AUAV_V3_TestSensors_DWork.RCRateTransition_Buffer0 = mlPilotConsoleData;
  }

  /* End of RateTransition: '<S8>/RC Rate Transition' */
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID5(void)
{
  /* S-Function (MCHP_C_function_Call): '<S8>/ParamInterfaceResponse' */
  AUAV_V3_TestSensors_B.ParamInterfaceResponse = ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data5' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.ParamInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID6(void)
{
  /* S-Function (MCHP_C_function_Call): '<S8>/MissionInterfaceResponse' */
  AUAV_V3_TestSensors_B.MissionInterfaceResponse = MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data6' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.MissionInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID10(void)
{
  /* S-Function (MCHP_C_function_Call): '<S8>/PackHeartBeat' */
  AUAV_V3_TestSensors_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data1' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackHeartBeat
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID12(void)
{
  /* DataStoreRead: '<S8>/Get RawGpsInt' */
  AUAV_V3_TestSensors_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S8>/Get time1' */
  AUAV_V3_TestSensors_B.Gettime1 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackGpsRawInt' */
  AUAV_V3_TestSensors_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawGpsInt
    , AUAV_V3_TestSensors_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data2' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackGpsRawInt
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID13(void)
{
  /* RateTransition: '<S8>/RawIMU Rate Transition' */
  AUAV_V3_TestSensors_DWork.RawIMURateTransition_semaphoreT = 1;
  AUAV_V3_TestSensors_B.RawIMURateTransition =
    AUAV_V3_TestSensors_DWork.RawIMURateTransition_Buffer0;
  AUAV_V3_TestSensors_DWork.RawIMURateTransition_semaphoreT = 0;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawIMU' */
  AUAV_V3_TestSensors_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.RawIMURateTransition
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawIMU
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID14(void)
{
  /* DataStoreRead: '<S8>/Get mlAirData' */
  AUAV_V3_TestSensors_B.GetmlAirData = mlAirData;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackScaledPressure' */
  AUAV_V3_TestSensors_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetmlAirData
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackScaledPressure
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID15(void)
{
  /* S-Function (MCHP_C_function_Call): '<S8>/PackSysStatus' */
  AUAV_V3_TestSensors_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_rtZmavlink_sys_status_t
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackSysStatus
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID16(void)
{
  /* DataStoreRead: '<S8>/Get VfrHud' */
  AUAV_V3_TestSensors_B.GetVfrHud = mlVfr_hud;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackHUD' */
  AUAV_V3_TestSensors_B.PackHUD = PackVFR_HUD(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetVfrHud
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data8' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackHUD
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID17(void)
{
  /* RateTransition: '<S8>/RC Rate Transition' */
  AUAV_V3_TestSensors_DWork.RCRateTransition_semaphoreTaken = 1;
  AUAV_V3_TestSensors_B.RCRateTransition =
    AUAV_V3_TestSensors_DWork.RCRateTransition_Buffer0;
  AUAV_V3_TestSensors_DWork.RCRateTransition_semaphoreTaken = 0;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawRC' */
  AUAV_V3_TestSensors_B.PackRawRC = PackRawRC(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.RCRateTransition
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data7' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawRC
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID18(void)
{
  /* DataStoreRead: '<S8>/Get Raw Commands' */
  AUAV_V3_TestSensors_B.GetRawCommands_p = mlPwmCommands;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawServo' */
  AUAV_V3_TestSensors_B.PackRawServo_d = PackRawServo(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawCommands_p
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data9' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawServo_d
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID19(void)
{
  /* DataStoreRead: '<S8>/Get Attitude 1' */
  AUAV_V3_TestSensors_B.GetAttitude1 = mlNavigation;

  /* S-Function (MCHP_C_function_Call): '<S8>/Pack Navigation' */
  AUAV_V3_TestSensors_B.PackNavigation = PackRawNavigation(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetAttitude1
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data11' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackNavigation
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID20(void)
{
  /* RateTransition: '<S8>/Attitude Rate Transition' */
  AUAV_V3_TestSensors_DWork.AttitudeRateTransition_semaphor = 1;
  AUAV_V3_TestSensors_B.AttitudeRateTransition =
    AUAV_V3_TestSensors_DWork.AttitudeRateTransition_Buffer0;
  AUAV_V3_TestSensors_DWork.AttitudeRateTransition_semaphor = 0;

  /* S-Function (MCHP_C_function_Call): '<S8>/Pack Attitude' */
  AUAV_V3_TestSensors_B.PackAttitude = PackRawAttitude(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.AttitudeRateTransition
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data10' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackAttitude
                  );
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
