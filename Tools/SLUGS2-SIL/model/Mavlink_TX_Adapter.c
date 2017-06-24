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
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "Mavlink_TX_Adapter.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID0(void)
{
  int16_T tmp;
  real32_T tmp_0;

  /* DataStoreWrite: '<S6>/Set VfrHud' incorporates:
   *  DataStoreRead: '<S6>/Get VfrHud1'
   *  DataStoreRead: '<S6>/Get VfrHud2'
   *  DataStoreRead: '<S6>/Get VfrHud4'
   *  DataTypeConversion: '<S6>/Data Type Conversion2'
   *  Gain: '<S6>/Gain'
   */
  mlVfr_hud.airspeed = mlNavigation.u_m;
  mlVfr_hud.groundspeed = (real32_T)(41943UL * mlGpsData.vel) * 2.38418579E-7F;
  mlVfr_hud.alt = mlLocalPositionData.z;
  mlVfr_hud.climb = mlLocalPositionData.vz;

  /* DataTypeConversion: '<S6>/Data Type Conversion1' incorporates:
   *  DataStoreRead: '<S6>/Get VfrHud3'
   *  Gain: '<S6>/Gain1'
   */
  tmp_0 = (real32_T)floor(57.2957802F * mlAttitudeSol.yaw);
  if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
    tmp_0 = 0.0F;
  } else {
    tmp_0 = (real32_T)fmod(tmp_0, 65536.0F);
  }

  /* DataStoreWrite: '<S6>/Set VfrHud' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion1'
   */
  mlVfr_hud.heading = tmp_0 < 0.0F ? -(int16_T)(uint16_T)-tmp_0 : (int16_T)
    (uint16_T)tmp_0;

  /* DataTypeConversion: '<S6>/Data Type Conversion5' incorporates:
   *  Constant: '<S6>/Constant1'
   *  DataStoreRead: '<S6>/Get VfrHud5'
   *  Gain: '<S6>/Gain2'
   *  Sum: '<S6>/Sum1'
   */
  tmp = (int16_T)fmod((int16_T)floor(((real_T)mlPwmCommands.servo1_raw - 9386.0)
	  * 0.00013460761879122358 * 100), 65536.0);

  /* DataStoreWrite: '<S6>/Set VfrHud' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion5'
   */
  mlVfr_hud.throttle = tmp < 0 ? (uint16_T)-(int16_T)(uint16_T)-(real_T)tmp :
    (uint16_T)tmp;

  /* RateTransition: '<S6>/RawIMU Rate Transition' */
  if (!(SLUGS2_DWork.RawIMURateTransition_semaphoreT != 0)) {
    SLUGS2_DWork.RawIMURateTransition_Buffer0 = mlRawImuData;
  }

  /* End of RateTransition: '<S6>/RawIMU Rate Transition' */

  /* RateTransition: '<S6>/Attitude Rate Transition' */
  if (!(SLUGS2_DWork.AttitudeRateTransition_semaphor != 0)) {
    SLUGS2_DWork.AttitudeRateTransition_Buffer0 = mlAttitudeSol;
  }

  /* End of RateTransition: '<S6>/Attitude Rate Transition' */

  /* RateTransition: '<S6>/RC Rate Transition' */
  if (!(SLUGS2_DWork.RCRateTransition_semaphoreTaken != 0)) {
    SLUGS2_DWork.RCRateTransition_Buffer0 = mlPilotConsoleData;
  }

  /* End of RateTransition: '<S6>/RC Rate Transition' */
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID5(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/ParamInterfaceResponse' */
  SLUGS2_B.ParamInterfaceResponse = ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data5' */
  TxN_Data_OverU1(
                  SLUGS2_B.ParamInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID6(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/MissionInterfaceResponse' */
  SLUGS2_B.MissionInterfaceResponse = MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data6' */
  TxN_Data_OverU1(
                  SLUGS2_B.MissionInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID10(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/PackHeartBeat' */
  SLUGS2_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data1' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackHeartBeat
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID12(void)
{
  /* DataStoreRead: '<S6>/Get RawGpsInt' */
  SLUGS2_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S6>/Get time1' */
  SLUGS2_B.Gettime1 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackGpsRawInt' */
  SLUGS2_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetRawGpsInt
    , SLUGS2_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data2' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackGpsRawInt
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID13(void)
{
  /* RateTransition: '<S6>/RawIMU Rate Transition' */
  SLUGS2_DWork.RawIMURateTransition_semaphoreT = 1;
  SLUGS2_B.RawIMURateTransition = SLUGS2_DWork.RawIMURateTransition_Buffer0;
  SLUGS2_DWork.RawIMURateTransition_semaphoreT = 0;

  /* DataStoreRead: '<S6>/Get time' */
  SLUGS2_B.Gettime = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackRawIMU' */
  SLUGS2_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.RawIMURateTransition
    , SLUGS2_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackRawIMU
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID14(void)
{
  /* DataStoreRead: '<S6>/Get mlAirData' */
  SLUGS2_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S6>/Get time2' */
  SLUGS2_B.Gettime2 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackScaledPressure' */
  SLUGS2_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetmlAirData
    , SLUGS2_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data3' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackScaledPressure
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID15(void)
{
  /* DataStoreRead: '<S6>/Get mlSysStatus' */
  SLUGS2_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackSysStatus' */
  SLUGS2_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data4' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackSysStatus
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID16(void)
{
  /* DataStoreRead: '<S6>/Get VfrHud' */
  SLUGS2_B.GetVfrHud = mlVfr_hud;

  /* DataStoreRead: '<S6>/Get time4' */
  SLUGS2_B.Gettime4_e = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackHUD' */
  SLUGS2_B.PackHUD = PackVFR_HUD(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetVfrHud
    , SLUGS2_B.Gettime4_e
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data8' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackHUD
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID17(void)
{
  /* RateTransition: '<S6>/RC Rate Transition' */
  SLUGS2_DWork.RCRateTransition_semaphoreTaken = 1;
  SLUGS2_B.RCRateTransition = SLUGS2_DWork.RCRateTransition_Buffer0;
  SLUGS2_DWork.RCRateTransition_semaphoreTaken = 0;

  /* DataStoreRead: '<S6>/Get time3' */
  SLUGS2_B.Gettime3 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackRawRC' */
  SLUGS2_B.PackRawRC = PackRawRC(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.RCRateTransition
    , SLUGS2_B.Gettime3
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data7' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackRawRC
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID18(void)
{
  /* DataStoreRead: '<S6>/Get Raw Commands' */
  SLUGS2_B.GetRawCommands_p = mlPwmCommands;

  /* DataStoreRead: '<S6>/Get time5' */
  SLUGS2_B.Gettime5_a = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackRawServo' */
  SLUGS2_B.PackRawServo_d = PackRawServo(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetRawCommands_p
    , SLUGS2_B.Gettime5_a
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data9' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackRawServo_d
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID19(void)
{
  /* DataStoreRead: '<S6>/Get Attitude 1' */
  SLUGS2_B.GetAttitude1 = mlNavigation;

  /* DataStoreRead: '<S6>/Get time7' */
  SLUGS2_B.Gettime7 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/Pack Navigation' */
  SLUGS2_B.PackNavigation = PackRawNavigation(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.GetAttitude1
    , SLUGS2_B.Gettime7
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data11' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackNavigation
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void SLUGS2_Mavlink_TX_AdapterTID20(void)
{
  /* RateTransition: '<S6>/Attitude Rate Transition' */
  SLUGS2_DWork.AttitudeRateTransition_semaphor = 1;
  SLUGS2_B.AttitudeRateTransition = SLUGS2_DWork.AttitudeRateTransition_Buffer0;
  SLUGS2_DWork.AttitudeRateTransition_semaphor = 0;

  /* DataStoreRead: '<S6>/Get time6' */
  SLUGS2_B.Gettime6 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/Pack Attitude' */
  SLUGS2_B.PackAttitude = PackRawAttitude(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , SLUGS2_B.AttitudeRateTransition
    , SLUGS2_B.Gettime6
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data10' */
  TxN_Data_OverU1(
                  SLUGS2_B.PackAttitude
                  );
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
