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
 * File: Mavlink_TX_Adapter.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
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
  tmp = (int16_T)fmod((int16_T)floor(((real_T)mlPwmCommands.servo1_raw - 942.0) *
    0.13513513513513511), 65536.0);

  /* DataStoreWrite: '<S8>/Set VfrHud' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion5'
   */
  mlVfr_hud.throttle = tmp < 0 ? (uint16_T)-(int16_T)(uint16_T)-(real_T)tmp :
    (uint16_T)tmp;

  /* RateTransition: '<S8>/RawIMU Rate Transition' incorporates:
   *  RateTransition: '<S8>/Attitude Rate Transition'
   */
  if (AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_2 == 1) {
    AUAV_V3_TestSensors_B.RawIMURateTransition = mlRawImuData;
    AUAV_V3_TestSensors_B.AttitudeRateTransition = mlAttitudeSol;
  }

  /* End of RateTransition: '<S8>/RawIMU Rate Transition' */

  /* RateTransition: '<S8>/RC Rate Transition' */
  if (AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_3 == 1) {
    AUAV_V3_TestSensors_B.RCRateTransition = mlPilotConsoleData;
  }

  /* End of RateTransition: '<S8>/RC Rate Transition' */
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID2(void)
{
  /* DataStoreRead: '<S8>/Get time' */
  AUAV_V3_TestSensors_B.Gettime = AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawIMU' */
  AUAV_V3_TestSensors_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.RawIMURateTransition
    , AUAV_V3_TestSensors_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawIMU
                  );

  /* DataStoreRead: '<S8>/Get time6' */
  AUAV_V3_TestSensors_B.Gettime6 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/Pack Attitude' */
  AUAV_V3_TestSensors_B.PackAttitude = PackRawAttitude(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.AttitudeRateTransition
    , AUAV_V3_TestSensors_B.Gettime6
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data10' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackAttitude
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID3(void)
{
  /* DataStoreRead: '<S8>/Get time3' */
  AUAV_V3_TestSensors_B.Gettime3 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawRC' */
  AUAV_V3_TestSensors_B.PackRawRC = PackRawRC(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.RCRateTransition
    , AUAV_V3_TestSensors_B.Gettime3
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data7' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawRC
                  );

  /* DataStoreRead: '<S8>/Get VfrHud' */
  AUAV_V3_TestSensors_B.GetVfrHud = mlVfr_hud;

  /* DataStoreRead: '<S8>/Get time4' */
  AUAV_V3_TestSensors_B.Gettime4_e =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackHUD' */
  AUAV_V3_TestSensors_B.PackHUD = PackVFR_HUD(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetVfrHud
    , AUAV_V3_TestSensors_B.Gettime4_e
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data8' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackHUD
                  );

  /* DataStoreRead: '<S8>/Get Raw Commands' */
  AUAV_V3_TestSensors_B.GetRawCommands_p = mlPwmCommands;

  /* DataStoreRead: '<S8>/Get time5' */
  AUAV_V3_TestSensors_B.Gettime5_a =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackRawServo' */
  AUAV_V3_TestSensors_B.PackRawServo_d = PackRawServo(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawCommands_p
    , AUAV_V3_TestSensors_B.Gettime5_a
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data9' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawServo_d
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID4(void)
{
  /* DataStoreRead: '<S8>/Get mlAirData' */
  AUAV_V3_TestSensors_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S8>/Get time2' */
  AUAV_V3_TestSensors_B.Gettime2 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackScaledPressure' */
  AUAV_V3_TestSensors_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetmlAirData
    , AUAV_V3_TestSensors_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackScaledPressure
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID5(void)
{
  /* DataStoreRead: '<S8>/Get mlSysStatus' */
  AUAV_V3_TestSensors_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S8>/PackSysStatus' */
  AUAV_V3_TestSensors_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S8>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackSysStatus
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID6(void)
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
void AUAV_V3_Mavlink_TX_AdapterTID7(void)
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
void AUAV_V3_Mavlink_TX_AdapterTID9(void)
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
void AUAV_V3_Mavlink_TX_AdapterTID10(void)
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

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
