/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: SLUGS2.h
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.290
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Jun 24 13:21:56 2017
 */

#ifndef RTW_HEADER_SLUGS2_h_
#define RTW_HEADER_SLUGS2_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "Barometer_Driver.h"
#include "IMU_Mag_Driver.h"
#include "LEDs_Driver.h"
#include "Mavlink_TX_Adapter.h"
#include "Position_and_Attitude_Filter.h"
#include "Sensor_Data_Adapter.h"
#include "rt_sys_SLUGS2_18.h"
#include "rt_sys_SLUGS2_22.h"
#include "rt_sys_SLUGS2_75.h"
#include "rt_sys_SLUGS2_76.h"
#include "rt_sys_SLUGS2_77.h"
#define FCY                            7.0E+7

/* Include for pic 33E */
#include <p33Exxxx.h>
#include <libpic30.h>                  /* For possible use with C function Call block (delay_ms or delay_us functions might be used by few peripherals) */
#include <libq.h>                      /* For possible use with C function Call block */
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmCounterLimit
# define rtmCounterLimit(rtm, idx)     ((rtm)->Timing.TaskCounters.cLimit[(idx)])
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* user code (top of header file) */
#include "MavlinkComm.h"

extern void InitParameterInterface(void);

/* Block signals for system '<S167>/negprotect' */
typedef struct {
  real32_T zpVal;                      /* '<S167>/negprotect' */
} rtB_negprotect_SLUGS2_p_T;

/* Block signals for system '<S445>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S445>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_b_T;

/* Block signals for system '<S181>/Zero out Z1' */
typedef struct {
  real32_T P[3];                       /* '<S181>/Zero out Z1' */
} rtB_ZerooutZ1_SLUGS2_T;

/* Block signals for system '<S309>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S309>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_i_T;

/* Block signals for system '<S317>/Select N  Terms' */
typedef struct {
  real32_T N[3];                       /* '<S317>/Select N  Terms' */
} rtB_SelectNTerms_SLUGS2_T;

/* Block signals for system '<S205>/negprotect3' */
typedef struct {
  real32_T zpVal;                      /* '<S205>/negprotect3' */
} rtB_negprotect3_SLUGS2_T;

/* Block signals for system '<S643>/Buffer IC Channel' */
typedef struct {
  uint16_T history[7];                 /* '<S643>/Buffer IC Channel' */
} rtB_BufferICChannel_SLUGS2_T;

/* Block states (auto storage) for system '<S643>/Buffer IC Channel' */
typedef struct {
  uint16_T oldValues[7];               /* '<S643>/Buffer IC Channel' */
} rtDW_BufferICChannel_SLUGS2_T;

/* Block signals for system '<Root>/myMux Fun3' */
typedef struct {
  real32_T y[6];                       /* '<Root>/myMux Fun3' */
} rtB_myMuxFun3_SLUGS2_T;

/* Block signals (auto storage) */
typedef struct {
  mavlink_attitude_t AttitudeRateTransition;/* '<S6>/Attitude Rate Transition' */
  mavlink_gps_raw_int_t GetRawGpsInt;  /* '<S6>/Get RawGpsInt' */
  mavlink_slugs_navigation_t GetAttitude1;/* '<S6>/Get Attitude 1' */
  mavlink_sys_status_t GetmlSysStatus; /* '<S6>/Get mlSysStatus' */
  mavlink_rc_channels_raw_t RCRateTransition;/* '<S6>/RC Rate Transition' */
  mavlink_raw_imu_t RawIMURateTransition;/* '<S6>/RawIMU Rate Transition' */
  mavlink_servo_output_raw_t GetRawCommands;/* '<S671>/Get Raw Commands' */
  mavlink_servo_output_raw_t GetRawCommands_p;/* '<S6>/Get Raw Commands' */
  mavlink_vfr_hud_t GetVfrHud;         /* '<S6>/Get VfrHud' */
  mavlink_scaled_pressure_t GetmlAirData;/* '<S6>/Get mlAirData' */
  real_T DataStoreRead;                /* '<Root>/Data Store Read' */
  real32_T myCosaLibcupdated5116;      /* '<S68>/myCos() aLib.c [updated 5.1.16]' */
  real32_T CFunctionCall1;             /* '<S69>/C Function Call1' */
  real32_T Merge[4];                   /* '<S670>/Merge' */
  real32_T y[9];                       /* '<Root>/myMux Fun5' */
  real32_T y_k[6];                     /* '<Root>/myMux Fun2' */
  real32_T y_j[6];                     /* '<S17>/myMux Fun2' */
  real32_T DataTypeConversion;         /* '<S664>/Data Type Conversion' */
  real32_T DataTypeConversion_h;       /* '<S665>/Data Type Conversion' */
  real32_T DataTypeConversion_g;       /* '<S666>/Data Type Conversion' */
  real32_T DataTypeConversion_f;       /* '<S667>/Data Type Conversion' */
  real32_T Merge_m;                    /* '<S618>/Merge' */
  real32_T Merge_j;                    /* '<S619>/Merge' */
  real32_T ProducetheGPSMainDataandupdatet[5];/* '<S580>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  real32_T DataTypeConversion7[3];     /* '<S581>/Data Type Conversion7' */
  real32_T DataTypeConversion3[3];     /* '<S581>/Data Type Conversion3' */
  real32_T DataTypeConversion6[3];     /* '<S581>/Data Type Conversion6' */
  real32_T GettheGSLocationupdateSensorMCU[3];/* '<S628>/Get the GS Location [updateSensorMCUState.c]1' */
  real32_T Switch1[5];                 /* '<S573>/Switch1' */
  real32_T y_o[4];                     /* '<S573>/myMux Fun1' */
  real32_T Sum;                        /* '<S630>/Sum' */
  real32_T u0k120k;                    /* '<S629>/[80k - 120k]' */
  real32_T AirData[3];                 /* '<S584>/AirData' */
  real32_T ProducetheGPSMainDataandupdat_a[5];/* '<S611>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  real32_T VectorConcatenate[9];       /* '<S525>/Vector Concatenate' */
  real32_T Submatrix1[3];              /* '<S471>/Submatrix1' */
  real32_T apUtilsc;                   /* '<S510>/[apUtils.c]' */
  real32_T GettheGSLocationupdateSensorM_c[3];/* '<S12>/Get the GS Location [updateSensorMCUState.c]1' */
  real32_T UnitConversion;             /* '<S529>/Unit Conversion' */
  real32_T apUtilsc1;                  /* '<S530>/[apUtils.c]1' */
  real32_T apUtilsc_a;                 /* '<S530>/[apUtils.c]' */
  real32_T DataTypeConversion1[3];     /* '<S547>/Data Type Conversion1' */
  real32_T apUtilsc_f;                 /* '<S505>/[apUtils.c]' */
  real32_T g_hat[3];                   /* '<S471>/Submatrix' */
  real32_T BiasRateLimiter[3];         /* '<S471>/Bias Rate Limiter' */
  real32_T DataTypeConversion_m[3];    /* '<S471>/Data Type Conversion' */
  real32_T GyroErr[3];                 /* '<S471>/Sum3' */
  real32_T In1[3];                     /* '<S474>/In1' */
  real32_T In2;                        /* '<S474>/In2' */
  real32_T In3;                        /* '<S474>/In3' */
  real32_T mySqrtapUtilscupdated5116;  /* '<S257>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T Merge_o[3];                 /* '<S173>/Merge' */
  real32_T mySqrtapUtilscupdated5116_p;/* '<S276>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T Switch;                     /* '<S262>/Switch' */
  real32_T Switch2;                    /* '<S262>/Switch2' */
  real32_T myAtan2apUtilscupdated5116; /* '<S266>/myAtan2() apUtils.c [updated 5.1.16]' */
  real32_T Merge2;                     /* '<S173>/Merge2' */
  real32_T myAbsapUtilscupdated5116;   /* '<S265>/myAbs() apUtils.c [updated 5.1.16]' */
  real32_T Product;                    /* '<S264>/Product' */
  real32_T myAtanapUtilscupdated5116;  /* '<S282>/myAtan() apUtils.c [updated 5.1.16]' */
  real32_T Merge1;                     /* '<S173>/Merge1' */
  real32_T IC3;                        /* '<S182>/IC3' */
  real32_T y_p[3];                     /* '<S173>/myMux Fun2' */
  real32_T y_b[4];                     /* '<S173>/myMux Fun1' */
  real32_T NumericalUnity[3];          /* '<S454>/Numerical Unity' */
  real32_T DataTypeConversion_c[2];    /* '<S451>/Data Type Conversion' */
  real32_T DataTypeConversion1_l[3];   /* '<S451>/Data Type Conversion1' */
  real32_T mySqrtapUtilscupdated5116_a;/* '<S244>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T Sum2;                       /* '<S174>/Sum2' */
  real32_T myAbsapUtilscupdated5116_a; /* '<S196>/myAbs() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_m;/* '<S210>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_j;/* '<S218>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T u1;                         /* '<S205>/[-1 1]' */
  real32_T myAcosapUtilscupdated5116;  /* '<S215>/myAcos() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_e;/* '<S231>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T u1_h;                       /* '<S206>/[-1 1]' */
  real32_T myACosapUtilscupdated5116;  /* '<S228>/myACos() apUtils.c [updated 5.1.16]' */
  real32_T myAbsapUtilscupdated5116_c; /* '<S209>/myAbs() apUtils.c [updated 5.1.16]' */
  real32_T myAbsapUtilscupdated5116_b; /* '<S208>/myAbs() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_pu;/* '<S289>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_b;/* '<S430>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T myAbsapUtilscupdated5116_i; /* '<S303>/myAbs() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_d;/* '<S304>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T GetWPCoordnavsupportcupdated511[3];/* '<S376>/Get WP Coord navsupport.c [updated 5.1.16]' */
  real32_T GetWPCoordnavsupportcupdated5_o[3];/* '<S377>/Get WP Coord navsupport.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_a5;/* '<S383>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_i;/* '<S390>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T WP0L2IPT1[3];               /* '<S297>/Subtract' */
  real32_T GetWPCoordnavsupportcupdated5_d[3];/* '<S321>/Get WP Coord navsupport.c [updated 5.1.16]' */
  real32_T GetWPCoordnavsupportcupdated5_b[3];/* '<S322>/Get WP Coord navsupport.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_pr;/* '<S328>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_m4;/* '<S335>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T Product1;                   /* '<S296>/Product1' */
  real32_T mySqrtapUtilscupdated5116_g;/* '<S446>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T Reshape1[3];                /* '<S439>/Reshape1' */
  real32_T Add;                        /* '<S159>/Add' */
  real32_T myPowapUtilscupdated5116;   /* '<S165>/myPow() apUtils.c [updated 5.1.16]' */
  real32_T mySqrtapUtilscupdated5116_me;/* '<S167>/mySqrt() apUtils.c [updated 5.1.16]' */
  real32_T u060;                       /* '<S133>/[-60 60]' */
  real32_T myCosapUtilscupdated5116;   /* '<S144>/myCos() apUtils.c [updated 5.1.16]' */
  real32_T bankLimit;                  /* '<S128>/bank Limit' */
  real32_T myTanapUtilscupdated5116;   /* '<S131>/myTan() apUtils.c [updated 5.1.16]' */
  real32_T Divide;                     /* '<S104>/Divide' */
  real32_T myAtanapUtilscupdated5116_a;/* '<S126>/myAtan() apUtils.c [updated 5.1.16]' */
  real32_T bankLimit_e;                /* '<S110>/bank Limit' */
  real32_T myTanapUtilscupdated5116_o; /* '<S114>/myTan() apUtils.c [updated 5.1.16]' */
  real32_T Subtract;                   /* '<S111>/Subtract' */
  real32_T Divide_h;                   /* '<S113>/Divide' */
  real32_T myTanapUtilscupdated5116_i; /* '<S120>/myTan() apUtils.c [updated 5.1.16]' */
  real32_T c;                          /* '<S117>/Math Function' */
  real32_T c_b;                        /* '<S117>/1-c' */
  real32_T Merge_oo;                   /* '<S73>/Merge' */
  real32_T PsiDotLimit;                /* '<S62>/Psi Dot  Limit' */
  real32_T T;                          /* '<S88>/-T' */
  real32_T myExpapUtilscupdated5116;   /* '<S90>/myExp() apUtils.c [updated 5.1.16]' */
  real32_T NumericalUnity_n;           /* '<S91>/Numerical Unity' */
  real32_T c_h;                        /* '<S88>/1-c' */
  real32_T T_m;                        /* '<S78>/-T' */
  real32_T myExpapUtilscupdated5116_f; /* '<S80>/myExp() apUtils.c [updated 5.1.16]' */
  real32_T NumericalUnity_a;           /* '<S81>/Numerical Unity' */
  real32_T c_d;                        /* '<S78>/1-c' */
  real32_T Sum_k;                      /* '<S37>/Sum' */
  real32_T u0k120k_i;                  /* '<S36>/[80k - 120k]' */
  uint32_T Gettime4;                   /* '<Root>/Get time4' */
  uint32_T Gettime5;                   /* '<S671>/Get time5' */
  uint32_T Gettime;                    /* '<S6>/Get time' */
  uint32_T Gettime6;                   /* '<S6>/Get time6' */
  uint32_T Gettime7;                   /* '<S6>/Get time7' */
  uint32_T Gettime1;                   /* '<S6>/Get time1' */
  uint32_T Gettime2;                   /* '<S6>/Get time2' */
  uint32_T Gettime3;                   /* '<S6>/Get time3' */
  uint32_T Gettime4_e;                 /* '<S6>/Get time4' */
  uint32_T Gettime5_a;                 /* '<S6>/Get time5' */
  int16_T ReadtheCubeDataadisCube16405c1[10];/* '<S583>/Read the Cube Data [adisCube16405.c]1' */
  int16_T y_l[13];                     /* '<S583>/myMux Fun4' */
  int16_T Switch_i[13];                /* '<S572>/Switch' */
  int16_T HILRawReadingshilc1[13];     /* '<S575>/HIL Raw Readings [hil.c]1' */
  int16_T HILRawReadingshilc1_k[13];   /* '<S574>/HIL Raw Readings [hil.c]1' */
  int16_T RateTransition11;            /* '<S27>/Rate Transition11' */
  int16_T RateTransition12;            /* '<S27>/Rate Transition12' */
  int16_T RateTransition2;             /* '<S27>/Rate Transition2' */
  int16_T RateTransition3;             /* '<S27>/Rate Transition3' */
  int16_T RateTransition4;             /* '<S27>/Rate Transition4' */
  int16_T RateTransition8;             /* '<S27>/Rate Transition8' */
  int16_T RateTransition9;             /* '<S27>/Rate Transition9' */
  uint16_T InputCapture_o1;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o2;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o3;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o4;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o5;            /* '<S644>/Input Capture' */
  uint16_T ChoosetheMediannavSupportcupdat;/* '<S648>/Choose the Median [navSupport.c] [updated 5.1.16]' */
  uint16_T RateTransition[4];          /* '<S16>/Rate Transition' */
  uint16_T Gain[4];                    /* '<S16>/Gain' */
  uint16_T U3CH4;                      /* '<Root>/MCU Load' */
  uint16_T PackRawServo;               /* '<S672>/PackRawServo' */
  uint16_T PackRawServo_l;             /* '<S671>/PackRawServo' */
  uint16_T ChoosetheMediannavSupportcupd_a;/* '<S647>/Choose the Median navSupport.c [updated 4.28.16]' */
  uint16_T DataTypeConversion_hn;      /* '<S660>/Data Type Conversion' */
  uint16_T u;                          /* '<S647>/2' */
  uint16_T DataTypeConversion_e;       /* '<S661>/Data Type Conversion' */
  uint16_T u_h;                        /* '<S647>/3' */
  uint16_T DataTypeConversion_go;      /* '<S662>/Data Type Conversion' */
  uint16_T u_m;                        /* '<S647>/4' */
  uint16_T DataTypeConversion_k;       /* '<S663>/Data Type Conversion' */
  uint16_T CalculusTimeStep1;          /* '<S582>/Calculus Time Step1' */
  uint16_T CalculusTimeStep2;          /* '<S582>/Calculus Time Step2' */
  uint16_T DataTypeConversion8;        /* '<S585>/Data Type Conversion8' */
  uint16_T PackRawIMU;                 /* '<S6>/PackRawIMU' */
  uint16_T PackHeartBeat;              /* '<S6>/PackHeartBeat' */
  uint16_T PackAttitude;               /* '<S6>/Pack Attitude' */
  uint16_T PackNavigation;             /* '<S6>/Pack Navigation' */
  uint16_T PackGpsRawInt;              /* '<S6>/PackGpsRawInt' */
  uint16_T PackScaledPressure;         /* '<S6>/PackScaledPressure' */
  uint16_T PackSysStatus;              /* '<S6>/PackSysStatus' */
  uint16_T ParamInterfaceResponse;     /* '<S6>/ParamInterfaceResponse' */
  uint16_T MissionInterfaceResponse;   /* '<S6>/MissionInterfaceResponse' */
  uint16_T PackRawRC;                  /* '<S6>/PackRawRC' */
  uint16_T PackHUD;                    /* '<S6>/PackHUD' */
  uint16_T PackRawServo_d;             /* '<S6>/PackRawServo' */
  uint16_T U1CH8[3];                   /* '<S53>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (100 Hz)' */
  uint16_T U3CH4_e;                    /* '<S53>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (100 Hz)' */
  uint16_T BUSSPIReadMPU6050AxyzTGxyz100Hz[3];/* '<S53>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (100 Hz)' */
  uint16_T RateTransition5;            /* '<S27>/Rate Transition5' */
  uint16_T RateTransition6;            /* '<S27>/Rate Transition6' */
  uint16_T RateTransition7;            /* '<S27>/Rate Transition7' */
  uint16_T InputCapture_o6;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o7;            /* '<S644>/Input Capture' */
  uint16_T InputCapture_o8;            /* '<S644>/Input Capture' */
  uint8_T ManualorAutonavSupportcupdated4;/* '<S15>/Manual or Auto? [navSupport.c] [updated 4.28.16]' */
  uint8_T DataTypeConversion1_h;       /* '<S659>/Data Type Conversion1' */
  uint8_T CFunctionCall;               /* '<S659>/C Function Call' */
  uint8_T GetRTBOrdernavSupportcupdated42[2];/* '<Root>/Get RTB Order [navSupport.c] [updated 4.27.16]' */
  uint8_T InitializeControlMCU[4];     /* '<S675>/Initialize Control MCU' */
  uint8_T InitializeControlMCU_a[4];   /* '<S677>/Initialize Control MCU' */
  uint8_T DataTypeConversion12;        /* '<S582>/Data Type Conversion12' */
  uint8_T ChecksifFixTypeis3updateSensorM;/* '<S13>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  uint8_T IstheGPSNovatelorUbloxgpsPortc1;/* '<S583>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
  uint8_T DatafromHILhilc2[110];       /* '<S574>/Data from HIL [hil.c]2' */
  uint8_T ChecksifFixTypeis3updateSenso_p;/* '<S12>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  uint8_T Merge3;                      /* '<S173>/Merge3' */
  uint8_T Merge4;                      /* '<S173>/Merge4' */
  uint8_T IC;                          /* '<S180>/IC' */
  uint8_T WP0;                         /* '<S180>/computeCurrentWP' */
  uint8_T WP1;                         /* '<S180>/computeCurrentWP' */
  uint8_T BUSI2CReadHMC5883Magn100Hz1[6];/* '<S3>/BUS I2C Read HMC5883 Magn (100 Hz)1' */
  uint8_T BUSI2CInitializeBMP180ReadTConv[2];/* '<S1>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  uint8_T BUSI2CInitializeBMP180ReadPConv[2];/* '<S1>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  uint8_T BUSI2CInitializeBMP180readCalib[22];/* '<S27>/BUS I2C Initialize BMP180 read Calibration data @ 100Hz' */
  boolean_T UpdateEulerandPQRupdateSensorMc;/* '<S14>/Update Euler and PQR [updateSensorMcuState.c]1' */
  boolean_T UpdatetheTimeStampupdateSensorM;/* '<S14>/Update the Time Stamp [updateSensorMcuState.c]1' */
  boolean_T UpdatetheTimeStampupdateSenso_l;/* '<S14>/Update the Time Stamp [updateSensorMcuState.c]2' */
  boolean_T UpdatetheTimeStampupdateSenso_d;/* '<S14>/Update the Time Stamp [updateSensorMcuState.c]7' */
  boolean_T DatafromHILXPlane;         /* '<S575>/Data from HIL X-Plane' */
  boolean_T DatafromHILXPlane2;        /* '<S575>/Data from HIL X-Plane2' */
  boolean_T HILMessagesParserDecoderhilc1;/* '<S574>/HIL Messages  Parser//Decoder [hil.c]1' */
  boolean_T DigitalOutputRead_o2;      /* '<S458>/Digital Output Read' */
  boolean_T LogicalOperator;           /* '<S5>/Logical Operator' */
  boolean_T DigitalOutputRead_o2_h;    /* '<S459>/Digital Output Read' */
  boolean_T LogicalOperator1;          /* '<S5>/Logical Operator1' */
  boolean_T DigitalOutputRead_o2_d;    /* '<S460>/Digital Output Read' */
  boolean_T LogicalOperator2;          /* '<S5>/Logical Operator2' */
  boolean_T DigitalOutputRead_o2_hf;   /* '<S461>/Digital Output Read' */
  boolean_T LogicalOperator3;          /* '<S5>/Logical Operator3' */
  rtB_myMuxFun3_SLUGS2_T sf_myMuxFun4; /* '<Root>/myMux Fun4' */
  rtB_myMuxFun3_SLUGS2_T sf_myMuxFun3; /* '<Root>/myMux Fun3' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction1;/* '<S17>/Embedded MATLAB Function1' */
  rtB_BufferICChannel_SLUGS2_T sf_BufferFailsafeChannel;/* '<S648>/Buffer Failsafe Channel' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction_b;/* '<S654>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction;/* '<S653>/Embedded MATLAB Function' */
  rtB_BufferICChannel_SLUGS2_T sf_BufferICChannel3;/* '<S643>/Buffer IC Channel3' */
  rtB_BufferICChannel_SLUGS2_T sf_BufferICChannel2;/* '<S643>/Buffer IC Channel2' */
  rtB_BufferICChannel_SLUGS2_T sf_BufferICChannel1;/* '<S643>/Buffer IC Channel1' */
  rtB_BufferICChannel_SLUGS2_T sf_BufferICChannel;/* '<S643>/Buffer IC Channel' */
  rtB_myMuxFun1_SLUGS2_d_T sf_myMuxFun;/* '<S573>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_cn;/* '<S624>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_lw;/* '<S623>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_np;/* '<S622>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_g;/* '<S621>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputat_b;/* '<S614>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_SLUGS2_T EnabledSubsystem_m;/* '<S614>/Enabled Subsystem' */
  rtB_myMuxFun_SLUGS2_T sf_myMuxFun_n; /* '<S583>/myMux Fun' */
  rtB_EmbeddedMATLABFunction1_e_T sf_EmbeddedMATLABFunction2_p;/* '<S583>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction1_e_T sf_EmbeddedMATLABFunction1_ib;/* '<S583>/Embedded MATLAB Function1' */
  rtB_myMuxFun_SLUGS2_i_T sf_myMuxFun_a;/* '<S590>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction2_fu;/* '<S590>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction1_in;/* '<S590>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_ft;/* '<S590>/Embedded MATLAB Function' */
  rtB_myMuxFun_SLUGS2_i_T sf_myMuxFun_m;/* '<S589>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction2_f;/* '<S589>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction1_f;/* '<S589>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_kq;/* '<S589>/Embedded MATLAB Function' */
  rtB_myMuxFun_SLUGS2_i_T sf_myMuxFun_i;/* '<S588>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction2_h;/* '<S588>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction1_i;/* '<S588>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_gp_T sf_EmbeddedMATLABFunction_f;/* '<S588>/Embedded MATLAB Function' */
  rtB_myMuxFun_SLUGS2_T sf_myMuxFun_h; /* '<S575>/myMux Fun' */
  rtB_EmbeddedMATLABFunction1_e_T sf_EmbeddedMATLABFunction2_i;/* '<S575>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction1_e_T sf_EmbeddedMATLABFunction1_m;/* '<S575>/Embedded MATLAB Function1' */
  rtB_myMuxFun1_SLUGS2_d_T sf_myMuxFun2_a;/* '<S545>/myMux Fun2' */
  rtB_myMuxFun1_SLUGS2_d_T sf_myMuxFun1_e;/* '<S545>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction1_S_T sf_EmbeddedMATLABFunction2;/* '<S553>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction1_S_T sf_EmbeddedMATLABFunction1_k;/* '<S553>/Embedded MATLAB Function1' */
  rtB_myMuxFun1_SLUGS2_T sf_myMuxFun1_d;/* '<S547>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction_ph;/* '<S557>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_g_T sf_EmbeddedMATLABFunction_a;/* '<S511>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_T sf_negprotect_m;/* '<S510>/negprotect' */
  rtB_EmbeddedMATLABFunction_g_T sf_EmbeddedMATLABFunction_op;/* '<S506>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_T sf_negprotect_k;/* '<S505>/negprotect' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction_lt;/* '<S471>/Embedded MATLAB Function' */
  rtB_myMuxFun1_SLUGS2_T sf_myMuxFun1_n;/* '<S478>/myMux Fun1' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ2; /* '<S173>/Zero out Z2' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ1; /* '<S173>/Zero out Z1' */
  rtB_negprotect3_SLUGS2_T sf_negprotect_b;/* '<S263>/negprotect' */
  rtB_negprotect3_SLUGS2_T sf_negprotect_h;/* '<S262>/negprotect' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ1_i;/* '<S270>/Zero out Z1' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_o;/* '<S276>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_nv;/* '<S275>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_lq;/* '<S257>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_i1;/* '<S256>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_lu;/* '<S244>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_c;/* '<S243>/Embedded MATLAB Function' */
  rtB_negprotect3_SLUGS2_T sf_negprotect2;/* '<S193>/negprotect2' */
  rtB_negprotect3_SLUGS2_T sf_negprotect1;/* '<S193>/negprotect1' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_ne;/* '<S210>/negprotect' */
  rtB_negprotect3_SLUGS2_T sf_negprotect3_i;/* '<S206>/negprotect3' */
  rtB_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_m;/* '<S227>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_i;/* '<S231>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_eg;/* '<S230>/Embedded MATLAB Function' */
  rtB_negprotect3_SLUGS2_T sf_negprotect3;/* '<S205>/negprotect3' */
  rtB_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_p;/* '<S214>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_c;/* '<S218>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_bp;/* '<S217>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ2_h2;/* '<S191>/Zero out Z2' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_o4;/* '<S289>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_le;/* '<S288>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ2_j;/* '<S180>/Zero out Z2' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_lj;/* '<S430>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_d;/* '<S429>/Embedded MATLAB Function' */
  rtB_SelectNTerms_SLUGS2_T sf_SelectNTerms_f;/* '<S375>/Select N  Terms' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_j;/* '<S390>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_l;/* '<S389>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_nb;/* '<S383>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_e;/* '<S382>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ3; /* '<S296>/Zero out Z3' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ2_e;/* '<S296>/Zero out Z2' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ1_j;/* '<S296>/Zero out Z1' */
  rtB_SelectNTerms_SLUGS2_T sf_SelectNTerms;/* '<S317>/Select N  Terms' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_n;/* '<S335>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_b1;/* '<S334>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_bi;/* '<S328>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_ih;/* '<S327>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_p;/* '<S304>/negprotect' */
  rtB_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_o;/* '<S302>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_k;/* '<S309>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_SLUGS2_T sf_ZerooutZ1_a;/* '<S181>/Zero out Z1' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_la;/* '<S446>/negprotect' */
  rtB_EmbeddedMATLABFunction_b_T sf_EmbeddedMATLABFunction_bs;/* '<S445>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_p_T sf_negprotect_l;/* '<S167>/negprotect' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction_n;/* '<S158>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction_i;/* '<S134>/Embedded MATLAB Function' */
  rtB_negprotect_SLUGS2_T sf_negprotect;/* '<S110>/negprotect' */
  rtB_EmbeddedMATLABFunction_SL_T sf_EmbeddedMATLABFunction_b0;/* '<S97>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputatio;/* '<S24>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_SLUGS2_T EnabledSubsystem;/* '<S24>/Enabled Subsystem' */
} BlockIO_SLUGS2_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  mavlink_attitude_t AttitudeRateTransition_Buffer0;/* '<S6>/Attitude Rate Transition' */
  mavlink_rc_channels_raw_t RCRateTransition_Buffer0;/* '<S6>/RC Rate Transition' */
  mavlink_raw_imu_t RawIMURateTransition_Buffer0;/* '<S6>/RawIMU Rate Transition' */
  real_T DiscreteZeroPole_DSTATE;      /* '<S632>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_o;    /* '<S559>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_j;    /* '<S560>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_k;    /* '<S561>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_c;    /* '<S491>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_n;    /* '<S492>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_e;    /* '<S493>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_jl;   /* '<S564>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_a;    /* '<S566>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_b;    /* '<S568>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_nk;   /* '<S39>/Discrete Zero-Pole' */
  real_T GS_INIT_FLAG;                 /* '<Root>/Data Store Memory' */
  real_T SIX_DOF_HIL_FLAG;             /* '<Root>/Data Store Memory1' */
  real_T X_PLANE_HIL_FLAG;             /* '<Root>/Data Store Memory2' */
  real_T Add3_DWORK1;                  /* '<S136>/Add3' */
  real32_T IntegerDelay3_DSTATE;       /* '<S66>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_k;     /* '<S67>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE;        /* '<S630>/Integer Delay' */
  real32_T DiscreteTimeIntegrator1_DSTATE[4];/* '<S471>/Discrete-Time Integrator1' */
  real32_T IntegerDelay_DSTATE_j[3];   /* '<S471>/Integer Delay' */
  real32_T UD_DSTATE;                  /* '<S537>/UD' */
  real32_T UD_DSTATE_o;                /* '<S538>/UD' */
  real32_T UD_DSTATE_o2;               /* '<S539>/UD' */
  real32_T UnitDelay_DSTATE;           /* '<S550>/Unit Delay' */
  real32_T IntegerDelay1_DSTATE[15];   /* '<S482>/Integer Delay1' */
  real32_T IntegerDelay1_DSTATE_g[15]; /* '<S552>/Integer Delay1' */
  real32_T UnitDelay_DSTATE_n;         /* '<S548>/Unit Delay' */
  real32_T UnitDelay_DSTATE_j;         /* '<S549>/Unit Delay' */
  real32_T IntegerDelay1_DSTATE_a;     /* '<S191>/Integer Delay1' */
  real32_T IntegerDelay_DSTATE_h;      /* '<S174>/Integer Delay' */
  real32_T IntegerDelay3_DSTATE_p;     /* '<S308>/Integer Delay3' */
  real32_T IntegerDelay1_DSTATE_gn;    /* '<S296>/Integer Delay1' */
  real32_T DiscreteTimeIntegrator_DSTATE[3];/* '<S436>/Discrete-Time Integrator' */
  real32_T IntegerDelay3_DSTATE_g;     /* '<S161>/Integer Delay3' */
  real32_T FixPtUnitDelay1_DSTATE;     /* '<S171>/FixPt Unit Delay1' */
  real32_T IntegerDelay3_DSTATE_e;     /* '<S140>/Integer Delay3' */
  real32_T IntegerDelay2_DSTATE;       /* '<S151>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_a;     /* '<S153>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_a;      /* '<S151>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_h[2];  /* '<S151>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_av;    /* '<S152>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_c;     /* '<S142>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_j;     /* '<S139>/Integer Delay3' */
  real32_T NDelays_DSTATE[5];          /* '<S135>/NDelays' */
  real32_T IntegerDelay2_DSTATE_f;     /* '<S147>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_cl;    /* '<S148>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_l;      /* '<S147>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_hy[2]; /* '<S147>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_m;     /* '<S149>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_b;     /* '<S141>/Integer Delay3' */
  real32_T NDelays_DSTATE_c[5];        /* '<S137>/NDelays' */
  real32_T IntegerDelay2_DSTATE_b;     /* '<S155>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_o;     /* '<S156>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_e;      /* '<S155>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_k[2];  /* '<S155>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_ep;    /* '<S157>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_mw;    /* '<S129>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_bs;    /* '<S130>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_oy;    /* '<S100>/Integer Delay3' */
  real32_T NDelays_DSTATE_l[5];        /* '<S103>/NDelays' */
  real32_T IntegerDelay2_DSTATE_bj;    /* '<S123>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_n;     /* '<S124>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_o;      /* '<S123>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_l[2];  /* '<S123>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_f;     /* '<S125>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_k0;    /* '<S99>/Integer Delay3' */
  real32_T NDelays_DSTATE_p[5];        /* '<S101>/NDelays' */
  real32_T IntegerDelay2_DSTATE_j;     /* '<S107>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_l;     /* '<S108>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_b;      /* '<S107>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_l3[2]; /* '<S107>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_d;     /* '<S109>/Integer Delay3' */
  real32_T UD_DSTATE_f;                /* '<S119>/UD' */
  real32_T IntegerDelay_DSTATE_ed;     /* '<S111>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_n;     /* '<S111>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_ph;    /* '<S112>/Integer Delay3' */
  real32_T UD_DSTATE_ff;               /* '<S82>/UD' */
  real32_T IntegerDelay_DSTATE_an;     /* '<S72>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_i;     /* '<S72>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_fs;    /* '<S74>/Integer Delay3' */
  real32_T UD_DSTATE_j;                /* '<S92>/UD' */
  real32_T IntegerDelay_DSTATE_k;      /* '<S76>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_b;     /* '<S76>/Integer Delay1' */
  real32_T IntegerDelay_DSTATE_i;      /* '<S62>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_e;     /* '<S62>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_h;     /* '<S75>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_p;      /* '<S77>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_ey[2]; /* '<S77>/Integer Delay1' */
  real32_T IntegerDelay2_DSTATE_g;     /* '<S77>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_nz;    /* '<S93>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_hh;    /* '<S94>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_jh;     /* '<S37>/Integer Delay' */
  uint32_T Output_DSTATE;              /* '<S2>/Output' */
  uint32_T Delay11_DSTATE;             /* '<S28>/Delay11' */
  uint32_T Delay11_DSTATE_j;           /* '<S29>/Delay11' */
  real32_T PrevY[3];                   /* '<S545>/Rate Limiter' */
  real32_T PrevY_n[3];                 /* '<S471>/Bias Rate Limiter' */
  real32_T lastGps_h;                  /* '<S546>/Embedded MATLAB Function3' */
  real32_T TimeSinceLast;              /* '<S546>/Embedded MATLAB Function3' */
  real32_T Memory1_PreviousInput;      /* '<S136>/Memory1' */
  real32_T Memory1_PreviousInput_p;    /* '<S135>/Memory1' */
  real32_T Memory1_PreviousInput_o;    /* '<S137>/Memory1' */
  real32_T Memory1_PreviousInput_g;    /* '<S103>/Memory1' */
  real32_T Memory1_PreviousInput_j;    /* '<S101>/Memory1' */
  uint32_T time_since_boot_usec;       /* '<Root>/TimeSinceBoot_usec' */
  uint16_T Delay_DSTATE;               /* '<S1>/Delay' */
  uint8_T IntegerDelay_DSTATE_c;       /* '<S173>/Integer Delay' */
  uint8_T IntegerDelay1_DSTATE_o;      /* '<S173>/Integer Delay1' */
  uint8_T IntegerDelay_DSTATE_im;      /* '<S180>/Integer Delay' */
  uint8_T FixPtUnitDelay2_DSTATE;      /* '<S171>/FixPt Unit Delay2' */
  boolean_T Delay_DSTATE_e;            /* '<S457>/Delay' */
  int8_T RawIMURateTransition_semaphoreT;/* '<S6>/RawIMU Rate Transition' */
  int8_T AttitudeRateTransition_semaphor;/* '<S6>/Attitude Rate Transition' */
  int8_T RCRateTransition_semaphoreTaken;/* '<S6>/RC Rate Transition' */
  uint8_T fromWp;                      /* '<S180>/computeCurrentWP' */
  uint8_T toWp;                        /* '<S180>/computeCurrentWP' */
  uint8_T persistentDidReachIP;        /* '<S180>/Embedded MATLAB Function' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S436>/Discrete-Time Integrator' */
  boolean_T IC1_FirstOutputTime;       /* '<S646>/IC1' */
  boolean_T lastGps_h_not_empty;       /* '<S546>/Embedded MATLAB Function3' */
  boolean_T IC1_FirstOutputTime_m;     /* '<S182>/IC1' */
  boolean_T IC2_FirstOutputTime;       /* '<S182>/IC2' */
  boolean_T IC4_FirstOutputTime;       /* '<S182>/IC4' */
  boolean_T IC_FirstOutputTime;        /* '<S179>/IC' */
  boolean_T IC3_FirstOutputTime;       /* '<S182>/IC3' */
  boolean_T IC_FirstOutputTime_m;      /* '<S180>/IC' */
  boolean_T IC_FirstOutputTime_b;      /* '<S111>/IC' */
  boolean_T IC_FirstOutputTime_g;      /* '<S72>/IC' */
  boolean_T IC_FirstOutputTime_f;      /* '<S76>/IC' */
  boolean_T L1OutputFeedbackControllerWithP;/* '<S4>/L1 Output Feedback Controller With  Projection Operator' */
  boolean_T Subsystem_MODE;            /* '<S545>/Subsystem' */
  boolean_T SideslipCompensation_MODE; /* '<S95>/Sideslip Compensation' */
  rtDW_BufferICChannel_SLUGS2_T sf_BufferFailsafeChannel;/* '<S648>/Buffer Failsafe Channel' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction_b;/* '<S654>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction;/* '<S653>/Embedded MATLAB Function' */
  rtDW_BufferICChannel_SLUGS2_T sf_BufferICChannel3;/* '<S643>/Buffer IC Channel3' */
  rtDW_BufferICChannel_SLUGS2_T sf_BufferICChannel2;/* '<S643>/Buffer IC Channel2' */
  rtDW_BufferICChannel_SLUGS2_T sf_BufferICChannel1;/* '<S643>/Buffer IC Channel1' */
  rtDW_BufferICChannel_SLUGS2_T sf_BufferICChannel;/* '<S643>/Buffer IC Channel' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_cn;/* '<S624>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_lw;/* '<S623>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_np;/* '<S622>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_g;/* '<S621>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputat_b;/* '<S614>/Enables//Disables the Computation of  initial Baro Bias' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction2_fu;/* '<S590>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction1_in;/* '<S590>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_ft;/* '<S590>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction2_f;/* '<S589>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction1_f;/* '<S589>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_kq;/* '<S589>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction2_h;/* '<S588>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction1_i;/* '<S588>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_i_T sf_EmbeddedMATLABFunction_f;/* '<S588>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction1__T sf_EmbeddedMATLABFunction2;/* '<S553>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction1__T sf_EmbeddedMATLABFunction1_k;/* '<S553>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction_ph;/* '<S557>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction_n;/* '<S158>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction_i;/* '<S134>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_S_T sf_EmbeddedMATLABFunction_b0;/* '<S97>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputatio;/* '<S24>/Enables//Disables the Computation of  initial Baro Bias' */
} D_Work_SLUGS2_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S575>/Constant1'
   *   '<S575>/Constant2'
   *   '<S583>/Constant1'
   *   '<S583>/Constant2'
   */
  real_T pooled16[5];

  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S476>/UEN 2 NEU'
   *   '<S175>/UEN 2 NEU'
   *   '<S197>/UEN 2 NEU'
   *   '<S439>/UEN 2 NEU'
   *   '<S341>/UEN 2 NEU'
   *   '<S358>/UEN 2 NEU'
   *   '<S396>/UEN 2 NEU'
   *   '<S413>/UEN 2 NEU'
   */
  real32_T pooled65[9];
} ConstParam_SLUGS2_T;

/* Real-time Model Data Structure */
struct tag_RTM_SLUGS2_T {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    boolean_T firstInitCondFlag;
    struct {
      uint16_T TID[21];
      uint16_T cLimit[21];
    } TaskCounters;

    struct {
      uint16_T TID0_1;
    } RateInteraction;
  } Timing;
};

/* Block signals (auto storage) */
extern BlockIO_SLUGS2_T SLUGS2_B;

/* Block states (auto storage) */
extern D_Work_SLUGS2_T SLUGS2_DWork;

/* Constant parameters (auto storage) */
extern const ConstParam_SLUGS2_T SLUGS2_ConstP;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern pi_struct mlParamInterface;     /* '<Root>/mlParamINterface' */
extern mavlink_mission_item_values_t mlWpValues;/* '<Root>/mlWpValues' */
extern mavlink_attitude_t mlAttitudeData;/* '<Root>/mlAttitudeData' */
extern mavlink_attitude_t mlAttitudeSol;/* '<Root>/mlAttitudeData1' */
extern mavlink_gps_raw_int_t mlGpsData;/* '<Root>/mlGpsData' */
extern mavlink_local_position_ned_t mlLocalPositionData;/* '<Root>/mlLocalPositionData' */
extern mavlink_slugs_navigation_t mlNavigation;/* '<Root>/mlNavigation' */
extern mavlink_sys_status_t mlSysStatus;/* '<Root>/mlSysStatus' */
extern mavlink_rc_channels_raw_t mlPilotConsoleData;/* '<Root>/mlPilotConsoleData' */
extern mavlink_raw_imu_t mlRawImuData; /* '<Root>/mlRawImuData' */
extern mavlink_servo_output_raw_t mlPwmCommands;/* '<Root>/mlRawServo' */
extern mavlink_vfr_hud_t mlVfr_hud;    /* '<Root>/mlVfr_hud' */
extern mavlink_scaled_pressure_t mlAirData;/* '<Root>/mlAirData' */
extern mavlink_coordinate_float_t mlGSLocationFloat;/* '<Root>/mlGSLocationFloat' */
extern mavlink_heartbeat_t mlHeartbeatLocal;/* '<Root>/mlHeartbeatLocal' */
extern mavlink_isr_location_t mlISR;   /* '<Root>/mlISR' */
extern mavlink_mid_lvl_cmds_t mlMidLevelCommands;/* '<Root>/mlMidLevelCommands' */
extern mavlink_slugs_mobile_location_t mlMobileLocation;/* '<Root>/mlMobileLocation' */
extern mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
extern mavlink_volt_sensor_t mlVISensor;/* '<Root>/mlVISensor' */
extern uint16_T MPU_T;                 /* '<Root>/MPU_T' */

/* External data declarations for dependent source files */
extern const mavlink_scaled_pressure_t SLUGS2_rtZmavlink_scaled_pressure_t;/* mavlink_scaled_pressure_t ground */
extern const mavlink_attitude_t SLUGS2_rtZmavlink_attitude_t;/* mavlink_attitude_t ground */
extern const mavlink_coordinate_float_t SLUGS2_rtZmavlink_coordinate_float_t;/* mavlink_coordinate_float_t ground */
extern const mavlink_gps_raw_int_t SLUGS2_rtZmavlink_gps_raw_int_t;/* mavlink_gps_raw_int_t ground */
extern const mavlink_heartbeat_t SLUGS2_rtZmavlink_heartbeat_t;/* mavlink_heartbeat_t ground */
extern const mavlink_isr_location_t SLUGS2_rtZmavlink_isr_location_t;/* mavlink_isr_location_t ground */
extern const mavlink_local_position_ned_t SLUGS2_rtZmavlink_local_position_ned_t;/* mavlink_local_position_ned_t ground */
extern const mavlink_mid_lvl_cmds_t SLUGS2_rtZmavlink_mid_lvl_cmds_t;/* mavlink_mid_lvl_cmds_t ground */
extern const mavlink_slugs_mobile_location_t
  SLUGS2_rtZmavlink_slugs_mobile_location_t;/* mavlink_slugs_mobile_location_t ground */
extern const mavlink_slugs_navigation_t SLUGS2_rtZmavlink_slugs_navigation_t;/* mavlink_slugs_navigation_t ground */
extern const pi_struct SLUGS2_rtZpi_struct;/* pi_struct ground */
extern const mavlink_rc_channels_raw_t SLUGS2_rtZmavlink_rc_channels_raw_t;/* mavlink_rc_channels_raw_t ground */
extern const mavlink_raw_imu_t SLUGS2_rtZmavlink_raw_imu_t;/* mavlink_raw_imu_t ground */
extern const mavlink_raw_pressure_t SLUGS2_rtZmavlink_raw_pressure_t;/* mavlink_raw_pressure_t ground */
extern const mavlink_servo_output_raw_t SLUGS2_rtZmavlink_servo_output_raw_t;/* mavlink_servo_output_raw_t ground */
extern const mavlink_sys_status_t SLUGS2_rtZmavlink_sys_status_t;/* mavlink_sys_status_t ground */
extern const mavlink_volt_sensor_t SLUGS2_rtZmavlink_volt_sensor_t;/* mavlink_volt_sensor_t ground */
extern const mavlink_vfr_hud_t SLUGS2_rtZmavlink_vfr_hud_t;/* mavlink_vfr_hud_t ground */
extern const mavlink_mission_item_values_t
  SLUGS2_rtZmavlink_mission_item_values_t;/* mavlink_mission_item_values_t ground */

/* Model entry point functions */
extern void SLUGS2_initialize(void);
extern void SLUGS2_step0(void);
extern void SLUGS2_step1(void);
extern void SLUGS2_step2(void);
extern void SLUGS2_step3(void);
extern void SLUGS2_step4(void);
extern void SLUGS2_step5(void);
extern void SLUGS2_step6(void);
extern void SLUGS2_step7(void);
extern void SLUGS2_step8(void);
extern void SLUGS2_step9(void);
extern void SLUGS2_step10(void);
extern void SLUGS2_step11(void);
extern void SLUGS2_step12(void);
extern void SLUGS2_step13(void);
extern void SLUGS2_step14(void);
extern void SLUGS2_step15(void);
extern void SLUGS2_step16(void);
extern void SLUGS2_step17(void);
extern void SLUGS2_step18(void);
extern void SLUGS2_step19(void);
extern void SLUGS2_step20(void);

/* Real-time Model object */
extern RT_MODEL_SLUGS2_T *const SLUGS2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'SLUGS2'
 * '<S1>'   : 'SLUGS2/Barometer_Driver'
 * '<S2>'   : 'SLUGS2/Counter Free-Running'
 * '<S3>'   : 'SLUGS2/IMU_Mag_Driver'
 * '<S4>'   : 'SLUGS2/Inner Loop// Navigation'
 * '<S5>'   : 'SLUGS2/LEDs_Driver'
 * '<S6>'   : 'SLUGS2/Mavlink_TX_Adapter'
 * '<S7>'   : 'SLUGS2/Model Info'
 * '<S8>'   : 'SLUGS2/PID Gains [updated 4.28.16]'
 * '<S9>'   : 'SLUGS2/Passthrough Logic'
 * '<S10>'  : 'SLUGS2/Pilot'
 * '<S11>'  : 'SLUGS2/Pilot LED and HIL Control Pins'
 * '<S12>'  : 'SLUGS2/Position_and_Attitude_Filter'
 * '<S13>'  : 'SLUGS2/Sensor_Data_Adapter'
 * '<S14>'  : 'SLUGS2/To Control  MCU'
 * '<S15>'  : 'SLUGS2/Trim Vals [updated 4.28.16]'
 * '<S16>'  : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]'
 * '<S17>'  : 'SLUGS2/get Nav Vars [updated 4.28.16]'
 * '<S18>'  : 'SLUGS2/myMux Fun1'
 * '<S19>'  : 'SLUGS2/myMux Fun2'
 * '<S20>'  : 'SLUGS2/myMux Fun3'
 * '<S21>'  : 'SLUGS2/myMux Fun4'
 * '<S22>'  : 'SLUGS2/myMux Fun5'
 * '<S23>'  : 'SLUGS2/Barometer_Driver/BMP 180 Temperature Compensation See datasheet'
 * '<S24>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter'
 * '<S25>'  : 'SLUGS2/Barometer_Driver/Disable path through in HIL Subsystem'
 * '<S26>'  : 'SLUGS2/Barometer_Driver/Disable path through in HIL Subsystem1'
 * '<S27>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)'
 * '<S28>'  : 'SLUGS2/Barometer_Driver/LPF 1Hz'
 * '<S29>'  : 'SLUGS2/Barometer_Driver/LPF 1Hz1'
 * '<S30>'  : 'SLUGS2/Barometer_Driver/Pack'
 * '<S31>'  : 'SLUGS2/Barometer_Driver/Pack1'
 * '<S32>'  : 'SLUGS2/Barometer_Driver/Script Pressure to Altitude'
 * '<S33>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Compute Barometric Height'
 * '<S34>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Enabled Subsystem'
 * '<S35>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S36>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Initial Baro Bias'
 * '<S37>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Zero Out Height'
 * '<S38>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S39>'  : 'SLUGS2/Barometer_Driver/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S40>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack10'
 * '<S41>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack11'
 * '<S42>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack12'
 * '<S43>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack2'
 * '<S44>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack3'
 * '<S45>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack4'
 * '<S46>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack5'
 * '<S47>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack6'
 * '<S48>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack7'
 * '<S49>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack8'
 * '<S50>'  : 'SLUGS2/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack9'
 * '<S51>'  : 'SLUGS2/Counter Free-Running/Increment Real World'
 * '<S52>'  : 'SLUGS2/Counter Free-Running/Wrap To Zero'
 * '<S53>'  : 'SLUGS2/IMU_Mag_Driver/BUS SPI  Read MPU 6050'
 * '<S54>'  : 'SLUGS2/IMU_Mag_Driver/Disable path through in HIL Subsystem1'
 * '<S55>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem1'
 * '<S56>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem10'
 * '<S57>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem2'
 * '<S58>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem3'
 * '<S59>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem8'
 * '<S60>'  : 'SLUGS2/IMU_Mag_Driver/Subsystem9'
 * '<S61>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m'
 * '<S62>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator'
 * '<S63>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]'
 * '<S64>'  : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]'
 * '<S65>'  : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]'
 * '<S66>'  : 'SLUGS2/Inner Loop// Navigation/Protect NaNs'
 * '<S67>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m/Protect NaNs'
 * '<S68>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_cos'
 * '<S69>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_sin'
 * '<S70>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_cos/Environment Controller1'
 * '<S71>'  : 'SLUGS2/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_sin/Environment Controller'
 * '<S72>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law'
 * '<S73>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator'
 * '<S74>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Protect NaNs'
 * '<S75>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Protect NaNs2'
 * '<S76>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor'
 * '<S77>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator'
 * '<S78>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef'
 * '<S79>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Detect Change'
 * '<S80>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef/dsPIC_exp'
 * '<S81>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef/dsPIC_exp/Environment Controller'
 * '<S82>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Detect Change/Difference1'
 * '<S83>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Compare To Zero'
 * '<S84>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Compare To Zero1'
 * '<S85>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Enabled Subsystem'
 * '<S86>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Enabled Subsystem1'
 * '<S87>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/F(theta)'
 * '<S88>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef'
 * '<S89>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Detect Change'
 * '<S90>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef/dsPIC_exp'
 * '<S91>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef/dsPIC_exp/Environment Controller'
 * '<S92>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Detect Change/Difference1'
 * '<S93>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator/Protect NaNs'
 * '<S94>'  : 'SLUGS2/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator/Protect NaNs1'
 * '<S95>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel'
 * '<S96>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1'
 * '<S97>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/0.016 sec  tau'
 * '<S98>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Angle Conversion1'
 * '<S99>'  : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Protect NaNs'
 * '<S100>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Protect NaNs2'
 * '<S101>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Roll Control [PID]'
 * '<S102>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation'
 * '<S103>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Yaw Damper [PID]'
 * '<S104>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/psi_dot to Bank'
 * '<S105>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function'
 * '<S106>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Roll Control [PID]/Saturation Dynamic'
 * '<S107>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator'
 * '<S108>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S109>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S110>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot'
 * '<S111>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Low Pass'
 * '<S112>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Protect NaNs2'
 * '<S113>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/psi_dot to Bank'
 * '<S114>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/dsPIC_tan'
 * '<S115>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect'
 * '<S116>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/dsPIC_tan/Environment Controller'
 * '<S117>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Low Pass/Compute Coef'
 * '<S118>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Low Pass/Detect Change'
 * '<S119>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Low Pass/Detect Change/Difference1'
 * '<S120>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/psi_dot to Bank/dsPIC_atan'
 * '<S121>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/psi_dot to Bank/dsPIC_atan/Environment Controller'
 * '<S122>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Yaw Damper [PID]/Saturation Dynamic'
 * '<S123>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator'
 * '<S124>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S125>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S126>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/psi_dot to Bank/dsPIC_atan'
 * '<S127>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/psi_dot to Bank/dsPIC_atan/Environment Controller'
 * '<S128>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1/Bank to Psi Dot'
 * '<S129>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1/Bank to Psi Dot/Protect NaNs'
 * '<S130>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1/Bank to Psi Dot/Protect NaNs1'
 * '<S131>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1/Bank to Psi Dot/dsPIC_tan'
 * '<S132>' : 'SLUGS2/Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/psi_dot_c Select logic1/Bank to Psi Dot/dsPIC_tan/Environment Controller'
 * '<S133>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel'
 * '<S134>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/0.5 sec  tau for Turns'
 * '<S135>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Airspeed Hold [PID]'
 * '<S136>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Height Error to Pitch  Command [PI]'
 * '<S137>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitch To Elevator [PID]'
 * '<S138>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End'
 * '<S139>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Protect NaNs'
 * '<S140>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Protect NaNs1'
 * '<S141>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Protect NaNs2'
 * '<S142>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Protect NaNs4'
 * '<S143>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Rate Limiter Dynamic with External IC'
 * '<S144>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/dsPIC_cos'
 * '<S145>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/0.5 sec  tau for Turns/Embedded MATLAB Function'
 * '<S146>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Airspeed Hold [PID]/Saturation Dynamic'
 * '<S147>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator'
 * '<S148>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S149>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S150>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Height Error to Pitch  Command [PI]/Saturation Dynamic'
 * '<S151>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator'
 * '<S152>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs'
 * '<S153>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs '
 * '<S154>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitch To Elevator [PID]/Saturation Dynamic'
 * '<S155>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator'
 * '<S156>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S157>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S158>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/ 0.3 sec tau for high speeds'
 * '<S159>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Air Density from Height'
 * '<S160>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed'
 * '<S161>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Protect NaN'
 * '<S162>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/lowpass limit'
 * '<S163>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/ 0.3 sec tau for high speeds/Embedded MATLAB Function'
 * '<S164>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Air Density from Height/Length Conversion'
 * '<S165>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Air Density from Height/dsPIC Power'
 * '<S166>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Air Density from Height/dsPIC Power/Environment Controller'
 * '<S167>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT'
 * '<S168>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/Environment Controller'
 * '<S169>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/negprotect'
 * '<S170>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Rate Limiter Dynamic with External IC/Saturation Dynamic'
 * '<S171>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Rate Limiter Dynamic with External IC/Unit Delay External IC'
 * '<S172>' : 'SLUGS2/Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/dsPIC_cos/Environment Controller1'
 * '<S173>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation'
 * '<S174>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation'
 * '<S175>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP'
 * '<S176>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed'
 * '<S177>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+'
 * '<S178>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment'
 * '<S179>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Minimum Turn Radius'
 * '<S180>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation'
 * '<S181>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation'
 * '<S182>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable'
 * '<S183>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Zero out Z1'
 * '<S184>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Zero out Z2'
 * '<S185>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun1'
 * '<S186>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun2'
 * '<S187>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun3'
 * '<S188>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun4'
 * '<S189>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun5'
 * '<S190>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun6'
 * '<S191>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location'
 * '<S192>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection'
 * '<S193>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation'
 * '<S194>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/No intersection,  Navigate to ISR'
 * '<S195>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm'
 * '<S196>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/dsPIC_ABS'
 * '<S197>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP'
 * '<S198>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Zero out Z2'
 * '<S199>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S200>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1'
 * '<S201>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S202>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S203>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S204>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle'
 * '<S205>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1'
 * '<S206>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2'
 * '<S207>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function'
 * '<S208>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS'
 * '<S209>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1'
 * '<S210>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT'
 * '<S211>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect1'
 * '<S212>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect2'
 * '<S213>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm'
 * '<S214>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product'
 * '<S215>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos'
 * '<S216>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3'
 * '<S217>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product'
 * '<S218>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT'
 * '<S219>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S220>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S221>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S222>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S223>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S224>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Environment Controller'
 * '<S225>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos/Environment Controller'
 * '<S226>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm'
 * '<S227>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product'
 * '<S228>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos'
 * '<S229>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/negprotect3'
 * '<S230>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product'
 * '<S231>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT'
 * '<S232>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S233>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S234>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S235>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S236>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S237>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Environment Controller'
 * '<S238>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos/Environment Controller'
 * '<S239>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS/Environment Controller'
 * '<S240>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1/Environment Controller'
 * '<S241>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/Environment Controller'
 * '<S242>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/negprotect'
 * '<S243>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product'
 * '<S244>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT'
 * '<S245>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S246>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S247>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S248>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S249>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/dsPIC_ABS/Environment Controller'
 * '<S250>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP/ECEF2LTP (UEN)'
 * '<S251>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP/Geod2ECEF1'
 * '<S252>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S253>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S254>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S255>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm'
 * '<S256>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S257>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S258>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S259>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S260>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S261>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S262>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta'
 * '<S263>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/L2+'
 * '<S264>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Subsystem'
 * '<S265>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/dsPIC_ABS'
 * '<S266>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/dsPIC_atan2'
 * '<S267>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc '
 * '<S268>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/2D Dot Product '
 * '<S269>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Compare To Zero'
 * '<S270>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed'
 * '<S271>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get ZProd'
 * '<S272>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/negprotect'
 * '<S273>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm'
 * '<S274>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Zero out Z1'
 * '<S275>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S276>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S277>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S278>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S279>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S280>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S281>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/L2+/negprotect'
 * '<S282>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Subsystem/dsPIC_atan'
 * '<S283>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/Subsystem/dsPIC_atan/Environment Controller'
 * '<S284>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/dsPIC_ABS/Environment Controller'
 * '<S285>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/dsPIC_atan2/Environment Controller'
 * '<S286>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc /Compare To Zero'
 * '<S287>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm'
 * '<S288>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC Dot Product'
 * '<S289>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC_SQRT'
 * '<S290>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S291>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S292>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S293>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S294>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger'
 * '<S295>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Embedded MATLAB Function'
 * '<S296>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet'
 * '<S297>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable'
 * '<S298>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm'
 * '<S299>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Zero out Z2'
 * '<S300>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/computeCurrentWP'
 * '<S301>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection'
 * '<S302>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product'
 * '<S303>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS'
 * '<S304>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT'
 * '<S305>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max'
 * '<S306>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max1'
 * '<S307>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/min'
 * '<S308>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/Protect NaNs'
 * '<S309>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product'
 * '<S310>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S311>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Environment Controller'
 * '<S312>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S313>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Environment Controller'
 * '<S314>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS/Environment Controller'
 * '<S315>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/Environment Controller'
 * '<S316>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/negprotect'
 * '<S317>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet'
 * '<S318>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z1'
 * '<S319>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z2'
 * '<S320>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z3'
 * '<S321>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1'
 * '<S322>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2'
 * '<S323>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector'
 * '<S324>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector '
 * '<S325>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms'
 * '<S326>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S327>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S328>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S329>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S330>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S331>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S332>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S333>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S334>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S335>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S336>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S337>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S338>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S339>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S340>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Environment Controller1'
 * '<S341>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2'
 * '<S342>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM'
 * '<S343>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S344>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S345>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S346>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S347>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S348>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S349>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S350>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S351>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S352>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S353>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S354>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S355>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S356>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S357>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Environment Controller1'
 * '<S358>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2'
 * '<S359>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM'
 * '<S360>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S361>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1'
 * '<S362>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S363>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S364>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S365>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1'
 * '<S366>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z1'
 * '<S367>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z2'
 * '<S368>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z3'
 * '<S369>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S370>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S371>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S372>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S373>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S374>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem'
 * '<S375>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet'
 * '<S376>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index'
 * '<S377>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1'
 * '<S378>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector'
 * '<S379>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector '
 * '<S380>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Select N  Terms'
 * '<S381>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S382>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S383>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S384>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S385>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S386>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S387>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S388>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S389>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S390>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S391>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S392>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S393>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S394>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S395>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Environment Controller1'
 * '<S396>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2'
 * '<S397>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM'
 * '<S398>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S399>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1'
 * '<S400>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S401>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S402>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S403>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1'
 * '<S404>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z1'
 * '<S405>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z2'
 * '<S406>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z3'
 * '<S407>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S408>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S409>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S410>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S411>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S412>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Environment Controller1'
 * '<S413>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2'
 * '<S414>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM'
 * '<S415>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S416>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S417>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S418>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S419>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S420>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S421>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S422>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S423>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S424>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S425>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S426>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S427>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S428>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S429>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product'
 * '<S430>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT'
 * '<S431>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S432>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S433>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S434>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S435>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location'
 * '<S436>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Simple Predictor'
 * '<S437>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm'
 * '<S438>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Zero out Z1'
 * '<S439>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP'
 * '<S440>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S441>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1'
 * '<S442>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S443>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S444>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S445>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product'
 * '<S446>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT'
 * '<S447>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S448>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S449>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S450>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S451>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Grab Upon Enable'
 * '<S452>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Subsystem'
 * '<S453>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location'
 * '<S454>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Grab Upon Enable/Environment Controller'
 * '<S455>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1'
 * '<S456>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S457>' : 'SLUGS2/Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Recent Enable/Subsystem/Detect Rising Edge'
 * '<S458>' : 'SLUGS2/LEDs_Driver/LED 1 (Blue)'
 * '<S459>' : 'SLUGS2/LEDs_Driver/LED 2 (red)'
 * '<S460>' : 'SLUGS2/LEDs_Driver/LED 3 (Green)'
 * '<S461>' : 'SLUGS2/LEDs_Driver/LED 4 (Yellow)'
 * '<S462>' : 'SLUGS2/Passthrough Logic/SLUGS_MODE_MID_LEVEL'
 * '<S463>' : 'SLUGS2/Passthrough Logic/SLUGS_MODE_PASSTHROUGH'
 * '<S464>' : 'SLUGS2/Passthrough Logic/SLUGS_MODE_SELECTIVE_PASSTHROUGH'
 * '<S465>' : 'SLUGS2/Pilot/Increment Real World'
 * '<S466>' : 'SLUGS2/Pilot/Wrap To Zero'
 * '<S467>' : 'SLUGS2/Pilot LED and HIL Control Pins/HIL Attitude On//Off Switch'
 * '<S468>' : 'SLUGS2/Pilot LED and HIL Control Pins/HIL On//Off Switch'
 * '<S469>' : 'SLUGS2/Pilot LED and HIL Control Pins/VI Sensor D01'
 * '<S470>' : 'SLUGS2/Pilot LED and HIL Control Pins/VI Sensor D02'
 * '<S471>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG'
 * '<S472>' : 'SLUGS2/Position_and_Attitude_Filter/COG.SOG2V'
 * '<S473>' : 'SLUGS2/Position_and_Attitude_Filter/Compute GS Location'
 * '<S474>' : 'SLUGS2/Position_and_Attitude_Filter/Enabled Subsystem'
 * '<S475>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update'
 * '<S476>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1'
 * '<S477>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter'
 * '<S478>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter'
 * '<S479>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product'
 * '<S480>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product1'
 * '<S481>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product2'
 * '<S482>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Deriv '
 * '<S483>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles'
 * '<S484>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function'
 * '<S485>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1'
 * '<S486>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2'
 * '<S487>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternion Normalize'
 * '<S488>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix'
 * '<S489>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/myMux Fun2'
 * '<S490>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/q dot calc'
 * '<S491>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/Filter1'
 * '<S492>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/Filter2'
 * '<S493>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/Filter3'
 * '<S494>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1'
 * '<S495>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem'
 * '<S496>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem1'
 * '<S497>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem'
 * '<S498>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem1'
 * '<S499>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem'
 * '<S500>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem1'
 * '<S501>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotDefault'
 * '<S502>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotZeroR3'
 * '<S503>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/Get DCM Values'
 * '<S504>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm'
 * '<S505>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2'
 * '<S506>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3'
 * '<S507>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2/negprotect'
 * '<S508>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S509>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm'
 * '<S510>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2'
 * '<S511>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3'
 * '<S512>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2/negprotect'
 * '<S513>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S514>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus'
 * '<S515>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S516>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A11'
 * '<S517>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A12'
 * '<S518>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A13'
 * '<S519>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A21'
 * '<S520>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A22'
 * '<S521>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A23'
 * '<S522>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A31'
 * '<S523>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A32'
 * '<S524>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A33'
 * '<S525>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Create Transformation Matrix'
 * '<S526>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S527>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S528>' : 'SLUGS2/Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S529>' : 'SLUGS2/Position_and_Attitude_Filter/COG.SOG2V/Angle Conversion'
 * '<S530>' : 'SLUGS2/Position_and_Attitude_Filter/COG.SOG2V/Subsystem5'
 * '<S531>' : 'SLUGS2/Position_and_Attitude_Filter/COG.SOG2V/myMux Fun2'
 * '<S532>' : 'SLUGS2/Position_and_Attitude_Filter/Compute GS Location/Geod2ECEF1'
 * '<S533>' : 'SLUGS2/Position_and_Attitude_Filter/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S534>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change'
 * '<S535>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change1'
 * '<S536>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change2'
 * '<S537>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change/Difference1'
 * '<S538>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change1/Difference1'
 * '<S539>' : 'SLUGS2/Position_and_Attitude_Filter/GPS Update/Detect Change2/Difference1'
 * '<S540>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S541>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1/Geod2ECEF1'
 * '<S542>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S543>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S544>' : 'SLUGS2/Position_and_Attitude_Filter/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S545>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel'
 * '<S546>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend'
 * '<S547>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/3D Filter'
 * '<S548>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate'
 * '<S549>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate1'
 * '<S550>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompRateOnly'
 * '<S551>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/Compare To Constant'
 * '<S552>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/Deriv '
 * '<S553>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/Subsystem'
 * '<S554>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/myMux Fun1'
 * '<S555>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/myMux Fun2'
 * '<S556>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3'
 * '<S557>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4'
 * '<S558>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4/Embedded MATLAB Function'
 * '<S559>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/3D Filter/Filter1'
 * '<S560>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/3D Filter/Filter2'
 * '<S561>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/3D Filter/Filter3'
 * '<S562>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/3D Filter/myMux Fun1'
 * '<S563>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1'
 * '<S564>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1/Filter2'
 * '<S565>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1'
 * '<S566>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1/Filter2'
 * '<S567>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1'
 * '<S568>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1/Filter2'
 * '<S569>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1'
 * '<S570>' : 'SLUGS2/Position_and_Attitude_Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function2'
 * '<S571>' : 'SLUGS2/Sensor_Data_Adapter/Initialize GS Location'
 * '<S572>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings'
 * '<S573>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite'
 * '<S574>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem'
 * '<S575>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1'
 * '<S576>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/MATLAB Function'
 * '<S577>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/Embedded MATLAB Function1'
 * '<S578>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/Embedded MATLAB Function2'
 * '<S579>' : 'SLUGS2/Sensor_Data_Adapter/Raw HIL  Readings/Enabled Subsystem1/myMux Fun'
 * '<S580>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ GPS is Ublox'
 * '<S581>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond '
 * '<S582>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Compute CPU Load'
 * '<S583>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors'
 * '<S584>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then update Air Data'
 * '<S585>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration'
 * '<S586>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/myMux Fun'
 * '<S587>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/myMux Fun1'
 * '<S588>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition'
 * '<S589>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1'
 * '<S590>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2'
 * '<S591>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis'
 * '<S592>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis '
 * '<S593>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis  '
 * '<S594>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function'
 * '<S595>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function1'
 * '<S596>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function2'
 * '<S597>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun'
 * '<S598>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S599>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function1'
 * '<S600>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function2'
 * '<S601>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/myMux Fun'
 * '<S602>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S603>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function1'
 * '<S604>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function2'
 * '<S605>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/myMux Fun'
 * '<S606>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function'
 * '<S607>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function1'
 * '<S608>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function2'
 * '<S609>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function3'
 * '<S610>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Novatel'
 * '<S611>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Ublox'
 * '<S612>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun'
 * '<S613>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4'
 * '<S614>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter'
 * '<S615>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Barometer Unit Conversion'
 * '<S616>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Unit Conversion'
 * '<S617>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Power Unit Conversion'
 * '<S618>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1'
 * '<S619>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro'
 * '<S620>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Thermistor Unit Conversion'
 * '<S621>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1'
 * '<S622>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2'
 * '<S623>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3'
 * '<S624>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4'
 * '<S625>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height'
 * '<S626>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enabled Subsystem'
 * '<S627>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S628>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/GS Height'
 * '<S629>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias'
 * '<S630>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Zero Out Height'
 * '<S631>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S632>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S633>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Compare To Constant'
 * '<S634>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Hi Temp Compensation2'
 * '<S635>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Lo Temp Compensation'
 * '<S636>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Compare To Constant'
 * '<S637>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Hi Temp Compensation'
 * '<S638>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Lo Temp Compensation'
 * '<S639>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S640>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S641>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3/Embedded MATLAB Function'
 * '<S642>' : 'SLUGS2/Sensor_Data_Adapter/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4/Embedded MATLAB Function'
 * '<S643>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Bufffer IC'
 * '<S644>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input'
 * '<S645>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Convert to Microseconds '
 * '<S646>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Detect Transition High to Low'
 * '<S647>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C.'
 * '<S648>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Mean Filter the Transition'
 * '<S649>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel'
 * '<S650>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel1'
 * '<S651>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel2'
 * '<S652>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel3'
 * '<S653>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/2 sec  tau'
 * '<S654>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/2 sec  tau1'
 * '<S655>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun1'
 * '<S656>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun5'
 * '<S657>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/2 sec  tau/Embedded MATLAB Function'
 * '<S658>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Control Surface Input/2 sec  tau1/Embedded MATLAB Function'
 * '<S659>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Detect Transition High to Low/Subsystem'
 * '<S660>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Microseconds '
 * '<S661>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Microseconds 1'
 * '<S662>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Microseconds 2'
 * '<S663>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Microseconds 3'
 * '<S664>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Rad'
 * '<S665>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Rad1'
 * '<S666>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Rad2'
 * '<S667>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Grab I.C./Convert to Rad3'
 * '<S668>' : 'SLUGS2/Trim Vals [updated 4.28.16]/Mean Filter the Transition/Buffer Failsafe Channel'
 * '<S669>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Convert to Microseconds'
 * '<S670>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type'
 * '<S671>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Send_Cmd_6DOF_HIL'
 * '<S672>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Send_Cmd_X_Plane_HIL'
 * '<S673>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Manual'
 * '<S674>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Passthrough'
 * '<S675>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough'
 * '<S676>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds'
 * '<S677>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1'
 * '<S678>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM'
 * '<S679>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM1'
 * '<S680>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM2'
 * '<S681>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM3'
 * '<S682>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/myMux Fun1'
 * '<S683>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM'
 * '<S684>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM1'
 * '<S685>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM2'
 * '<S686>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM3'
 * '<S687>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/myMux Fun1'
 * '<S688>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Average'
 * '<S689>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Average1'
 * '<S690>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Average2'
 * '<S691>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Average3'
 * '<S692>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Convert to PWM'
 * '<S693>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Convert to PWM1'
 * '<S694>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Convert to PWM2'
 * '<S695>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/Convert to PWM3'
 * '<S696>' : 'SLUGS2/Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/myMux Fun1'
 * '<S697>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Angle Conversion'
 * '<S698>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Direction Cosine Matrix to Rotation Angles'
 * '<S699>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Embedded MATLAB Function1'
 * '<S700>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1'
 * '<S701>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2'
 * '<S702>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/myMux Fun1'
 * '<S703>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/myMux Fun2'
 * '<S704>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Direction Cosine Matrix to Rotation Angles/AxisRotDefault'
 * '<S705>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Direction Cosine Matrix to Rotation Angles/AxisRotZeroR3'
 * '<S706>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Direction Cosine Matrix to Rotation Angles/Get DCM Values'
 * '<S707>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A11'
 * '<S708>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A12'
 * '<S709>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A13'
 * '<S710>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A21'
 * '<S711>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A22'
 * '<S712>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A23'
 * '<S713>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A31'
 * '<S714>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A32'
 * '<S715>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/A33'
 * '<S716>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix1/Create Transformation Matrix'
 * '<S717>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A11'
 * '<S718>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A12'
 * '<S719>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A13'
 * '<S720>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A21'
 * '<S721>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A22'
 * '<S722>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A23'
 * '<S723>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A31'
 * '<S724>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A32'
 * '<S725>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/A33'
 * '<S726>' : 'SLUGS2/get Nav Vars [updated 4.28.16]/Euler Angles to  Direction Cosine Matrix2/Create Transformation Matrix'
 */
#endif                                 /* RTW_HEADER_SLUGS2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
