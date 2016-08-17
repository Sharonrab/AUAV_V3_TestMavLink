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
 * Model version                        : 1.138
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Mon Aug 15 17:25:56 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Mon Aug 15 17:25:58 2016
 */

#include "Mavlink_TX_Adapter.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"



/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID1(void)
{
  /* DataStoreRead: '<S6>/Get Raw IMU' */
  AUAV_V3_TestSensors_B.GetRawIMU = mlRawIMU;

  /* DataStoreRead: '<S6>/Get time' */
  AUAV_V3_TestSensors_B.Gettime = AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackRawIMU' */
  AUAV_V3_TestSensors_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawIMU
    , AUAV_V3_TestSensors_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawIMU
                  );

  /* DataStoreRead: '<S6>/Get Raw RC' */
  AUAV_V3_TestSensors_B.GetRawRC = mlRC_Commands;

  /* DataStoreRead: '<S6>/Get time3' */
  AUAV_V3_TestSensors_B.Gettime3 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackRawRC' */
  AUAV_V3_TestSensors_B.PackRawRC = PackRawRC(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawRC
    , AUAV_V3_TestSensors_B.Gettime3
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data7' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackRawRC
                  );

  /* DataStoreRead: '<S6>/Get VfrHud' */
  AUAV_V3_TestSensors_B.GetVfrHud = mlVfr_hud;

  /* DataStoreRead: '<S6>/Get time4' */
  AUAV_V3_TestSensors_B.Gettime4 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackHUD' */
  AUAV_V3_TestSensors_B.PackHUD = PackVFR_HUD(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetVfrHud
    , AUAV_V3_TestSensors_B.Gettime4
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data8' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackHUD
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID4(void)
{
  /* DataStoreRead: '<S6>/Get RawGpsInt' */
  AUAV_V3_TestSensors_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S6>/Get time1' */
  AUAV_V3_TestSensors_B.Gettime1 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackGpsRawInt' */
  AUAV_V3_TestSensors_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetRawGpsInt
    , AUAV_V3_TestSensors_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data2' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackGpsRawInt
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID5(void)
{
  /* DataStoreRead: '<S6>/Get mlAirData' */
  AUAV_V3_TestSensors_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S6>/Get time2' */
  AUAV_V3_TestSensors_B.Gettime2 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackScaledPressure' */
  AUAV_V3_TestSensors_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetmlAirData
    , AUAV_V3_TestSensors_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackScaledPressure
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID6(void)
{
  /* DataStoreRead: '<S6>/Get mlSysStatus' */
  AUAV_V3_TestSensors_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S6>/PackSysStatus' */
  AUAV_V3_TestSensors_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestSensors_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackSysStatus
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID7(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/ParamInterfaceResponse' */
  AUAV_V3_TestSensors_B.ParamInterfaceResponse = ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data5' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.ParamInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID8(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/MissionInterfaceResponse' */
  AUAV_V3_TestSensors_B.MissionInterfaceResponse = MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data6' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.MissionInterfaceResponse
                  );
}

/* Output and update for atomic system: '<Root>/Mavlink_TX_Adapter' */
void AUAV_V3_Mavlink_TX_AdapterTID10(void)
{
  /* S-Function (MCHP_C_function_Call): '<S6>/PackHeartBeat' */
  AUAV_V3_TestSensors_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S6>/TX_N_Data1' */
  TxN_Data_OverU1(
                  AUAV_V3_TestSensors_B.PackHeartBeat
                  );
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
