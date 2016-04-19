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
 * File: AUAV3_AND_SLUGS_SENSOR.h
 *
 * Real-Time Workshop code generated for Simulink model AUAV3_AND_SLUGS_SENSOR.
 *
 * Model version                        : 1.188
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Tue Apr 19 10:47:35 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Tue Apr 19 10:47:36 2016
 */

#ifndef RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_
#define RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_
#ifndef AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_
# define AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#endif                                 /* AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_ */

#include "AUAV3_AND_SLUGS_SENSOR_types.h"
#define FCY                            7.0E+7

/* Include for pic 33E */
#include <p33Exxxx.h>
#include <libpic30.h>                  /* For possible use with C function Call block (delay_ms or delay_us functions might be used by few peripherals) */
#include <libq.h>                      /* For possible use with C function Call block */

/* Macros for accessing real-time model data structure */
#ifndef rtmCounterLimit
# define rtmCounterLimit(rtm, idx)     ((rtm)->Timing.TaskCounters.cLimit[(idx)])
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((void*) 0)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((void) 0)
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* user code (top of header file) */
#include "gpsPort.h"
#include "MavlinkComm.h"
#include "..\mavlink\include\common\common.h"

typedef struct MCHP_SPI1_QueueStr{
  volatile uint16_T buffer[3];         /* Size is equal to the number or distinct sequence set +1 */
  uint16_T tail;                       /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint16_T head;              /* head is the index for the next value to be written into the Circular buffer */
} MCHP_SPI1_QueueStr;

/* Declare UART1 Tx Circular Buffer Structure */
#define Tx_BUFF_SIZE_Uart1             (128)

typedef struct MCHP_UART1_TxStr{
  volatile uint8_T buffer[Tx_BUFF_SIZE_Uart1];/* Size Rx_BUFF_SIZE_Uart1 is 128 */
  uint_T tail;                         /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint_T head;                /* head is the index for the next value to be written into the Circular buffer */
} MCHP_UART1_TxStr;

/* Declare UART4 Tx Circular Buffer Structure */
#define Tx_BUFF_SIZE_Uart4             (128)

typedef struct MCHP_UART4_TxStr{
  volatile uint8_T buffer[Tx_BUFF_SIZE_Uart4];/* Size Rx_BUFF_SIZE_Uart4 is 128 */
  uint_T tail;                         /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint_T head;                /* head is the index for the next value to be written into the Circular buffer */
} MCHP_UART4_TxStr;

typedef struct MCHP_I2C2_QueueStr{
  volatile uint16_T buffer[6];         /* Size is equal to the number or distinct sequence set +1 */
  uint16_T tail;                       /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint16_T head;              /* head is the index for the next value to be written into the Circular buffer */
} MCHP_I2C2_QueueStr;

/* Block signals for system '<S19>/Enabled Subsystem' */
typedef struct {
  real32_T In1;                        /* '<S27>/In1' */
} rtB_EnabledSubsystem_AUAV3_AN_T;

/* Block signals for system '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T tOut;                         /* '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
} rtB_EnablesDisablestheComputa_T;

/* Block states (auto storage) for system '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T aveCount;                     /* '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T tIni;                         /* '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
} rtDW_EnablesDisablestheComput_T;

/* Block signals for system '<S56>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S56>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_AU_T;

/* Block states (auto storage) for system '<S56>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S56>/Embedded MATLAB Function' */
  real_T b;                            /* '<S56>/Embedded MATLAB Function' */
  real_T y_km1;                        /* '<S56>/Embedded MATLAB Function' */
  real_T u_km1;                        /* '<S56>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S56>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_A_T;

/* Block signals for system '<S51>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S51>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_d_T;

/* Block signals (auto storage) */
typedef struct {
  mavlink_gps_raw_int_t GetRawGpsInt;  /* '<S4>/Get RawGpsInt' */
  mavlink_raw_imu_t GetRawIMU;         /* '<S4>/Get Raw IMU' */
  mavlink_scaled_pressure_t GetmlAirData;/* '<S4>/Get mlAirData' */
  real32_T Merge;                      /* '<S86>/Merge' */
  real32_T Merge_j;                    /* '<S87>/Merge' */
  real32_T GettheGSLocationupdateSensorMCU[3];/* '<S96>/Get the GS Location [updateSensorMCUState.c]1' */
  real32_T y[4];                       /* '<S48>/myMux Fun1' */
  real32_T Sum;                        /* '<S98>/Sum' */
  real32_T u0k120k;                    /* '<S97>/[80k - 120k]' */
  real32_T AirData[3];                 /* '<S52>/AirData' */
  real32_T ProducetheGPSMainDataandupdatet;/* '<S79>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  real32_T ProducetheGPSMainDataandupdat_d;/* '<S78>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]1' */
  real32_T Sum_c;                      /* '<S30>/Sum' */
  real32_T u0k120k_e;                  /* '<S29>/[80k - 120k]' */
  uint32_T Gettime1;                   /* '<S4>/Get time1' */
  int16_T ReadtheCubeDataadisCube16405c1[10];/* '<S51>/Read the Cube Data [adisCube16405.c]1' */
  int16_T y_h[13];                     /* '<S51>/myMux Fun4' */
  int16_T y_p[4];                      /* '<S51>/myMux Fun' */
  int16_T RateTransition11;            /* '<S20>/Rate Transition11' */
  int16_T RateTransition12;            /* '<S20>/Rate Transition12' */
  int16_T RateTransition2;             /* '<S20>/Rate Transition2' */
  int16_T RateTransition3;             /* '<S20>/Rate Transition3' */
  int16_T RateTransition4;             /* '<S20>/Rate Transition4' */
  int16_T RateTransition8;             /* '<S20>/Rate Transition8' */
  int16_T RateTransition9;             /* '<S20>/Rate Transition9' */
  uint16_T U3CH4;                      /* '<Root>/MCU Load' */
  uint16_T U1CH8[3];                   /* '<S7>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T U1CH4;                      /* '<S7>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T BUSSPIReadMPU6050AxyzTGxyz1kHz_[3];/* '<S7>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T dT;                         /* '<S1>/Input Capture' */
  uint16_T dA;                         /* '<S1>/Input Capture' */
  uint16_T dE;                         /* '<S1>/Input Capture' */
  uint16_T dR;                         /* '<S1>/Input Capture' */
  uint16_T dFailsafe;                  /* '<S1>/Input Capture' */
  uint16_T CalculusTimeStep1;          /* '<S50>/Calculus Time Step1' */
  uint16_T CalculusTimeStep2;          /* '<S50>/Calculus Time Step2' */
  uint16_T DataTypeConversion8;        /* '<S53>/Data Type Conversion8' */
  uint16_T InputCaptureRCReceiver1_o2; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o3; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o5; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o6; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T Saturation;                 /* '<S9>/Saturation' */
  uint16_T Saturation1;                /* '<S9>/Saturation1' */
  uint16_T PackRawIMU;                 /* '<S4>/PackRawIMU' */
  uint16_T PackHeartBeat;              /* '<S4>/PackHeartBeat' */
  uint16_T PackGpsRawInt;              /* '<S4>/PackGpsRawInt' */
  uint16_T PackScaledPressure;         /* '<S4>/PackScaledPressure' */
  uint16_T PackSysStatus;              /* '<S4>/PackSysStatus' */
  uint16_T ParamInterfaceResponse;     /* '<S4>/ParamInterfaceResponse' */
  uint16_T ParamInterfaceResponse1;    /* '<S4>/ParamInterfaceResponse1' */
  uint16_T RateTransition5;            /* '<S20>/Rate Transition5' */
  uint16_T RateTransition6;            /* '<S20>/Rate Transition6' */
  uint16_T RateTransition7;            /* '<S20>/Rate Transition7' */
  uint16_T cmdPITCHraw;                /* '<S9>/Input Capture RC Receiver1' */
  uint16_T cmdROLLraw;                 /* '<S9>/Input Capture RC Receiver1' */
  uint8_T BUSI2CInitializeBMP180ReadTConv[2];/* '<S6>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  uint8_T BUSI2CInitializeBMP180ReadPConv[2];/* '<S6>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  uint8_T BUSI2CReadHMC5883Magn50Hz[6];/* '<S7>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  uint8_T DataTypeConversion12;        /* '<S50>/Data Type Conversion12' */
  uint8_T ReadtheRawDatafromGPSgpsPortc1;/* '<S78>/Read the Raw Data from GPS [gpsPort.c]1' */
  uint8_T DatafromHILhilc2[13];        /* '<S47>/Data from HIL [hil.c]2' */
  uint8_T HILMessagesParserDecoderhilc1[13];/* '<S47>/HIL Messages  Parser//Decoder [hil.c]1' */
  uint8_T HILRawReadingshilc1[13];     /* '<S47>/HIL Raw Readings [hil.c]1' */
  uint8_T BUSI2CInitializeBMP180readCalib[22];/* '<S20>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
  boolean_T IstheGPSNovatelorUbloxgpsPortc1;/* '<S51>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_o;/* '<S92>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_l;/* '<S91>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_g;/* '<S90>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction;/* '<S89>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputat_e;/* '<S82>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_AUAV3_AN_T EnabledSubsystem_o;/* '<S82>/Enabled Subsystem' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction3;/* '<S51>/Embedded MATLAB Function3' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction2_o;/* '<S51>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction1_d;/* '<S51>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_d_T sf_EmbeddedMATLABFunction_kd;/* '<S51>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2_c;/* '<S58>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1_my;/* '<S58>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_hj;/* '<S58>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2_k;/* '<S57>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1_m;/* '<S57>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_h;/* '<S57>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2;/* '<S56>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1;/* '<S56>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_k;/* '<S56>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputatio;/* '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_AUAV3_AN_T EnabledSubsystem;/* '<S19>/Enabled Subsystem' */
} BlockIO_AUAV3_AND_SLUGS_SENSO_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  mavlink_raw_imu_t mlRawIMU;          /* '<Root>/mlRawIMU' */
  real_T DiscreteZeroPole_DSTATE;      /* '<S100>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_j;    /* '<S32>/Discrete Zero-Pole' */
  real32_T IntegerDelay_DSTATE;        /* '<S98>/Integer Delay' */
  real32_T IntegerDelay_DSTATE_k;      /* '<S30>/Integer Delay' */
  uint32_T Output_DSTATE;              /* '<S2>/Output' */
  uint32_T Delay11_DSTATE;             /* '<S21>/Delay11' */
  uint32_T Delay11_DSTATE_j;           /* '<S22>/Delay11' */
  uint32_T time_since_boot_usec;       /* '<Root>/TimeSinceBoot_usec' */
  uint16_T Delay_DSTATE;               /* '<S6>/Delay' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_o;/* '<S92>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_l;/* '<S91>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_g;/* '<S90>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction;/* '<S89>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputat_e;/* '<S82>/Enables//Disables the Computation of  initial Baro Bias' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2_c;/* '<S58>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1_my;/* '<S58>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_hj;/* '<S58>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2_k;/* '<S57>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1_m;/* '<S57>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_h;/* '<S57>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2;/* '<S56>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1;/* '<S56>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_k;/* '<S56>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputatio;/* '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
} D_Work_AUAV3_AND_SLUGS_SENSOR_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S51>/Constant'
   *   '<S51>/Constant1'
   *   '<S51>/Constant2'
   *   '<S51>/Constant3'
   */
  real_T pooled5[5];
} ConstParam_AUAV3_AND_SLUGS_SE_T;

/* Real-time Model Data Structure */
struct RT_MODEL_AUAV3_AND_SLUGS_SENS_T {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[11];
      uint8_T cLimit[11];
    } TaskCounters;
  } Timing;
};

/* Block signals (auto storage) */
extern BlockIO_AUAV3_AND_SLUGS_SENSO_T AUAV3_AND_SLUGS_SENSOR_B;

/* Block states (auto storage) */
extern D_Work_AUAV3_AND_SLUGS_SENSOR_T AUAV3_AND_SLUGS_SENSOR_DWork;

/* Constant parameters (auto storage) */
extern const ConstParam_AUAV3_AND_SLUGS_SE_T AUAV3_AND_SLUGS_SENSOR_ConstP;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern mavlink_gps_raw_int_t mlGpsData;/* '<Root>/mlGpsData' */
extern mavlink_sys_status_t mlSysStatus;/* '<Root>/mlSysStatus' */
extern mavlink_scaled_pressure_t mlAirData;/* '<Root>/mlAirData' */
extern mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
extern uint16_T MPU_T;                 /* '<Root>/MPU_T' */

/* External data declarations for dependent source files */
extern const mavlink_scaled_pressure_t
  AUAV3_AND_SLUGS_SENSOR_rtZmavlink_scaled_pressure_t;/* mavlink_scaled_pressure_t ground */
extern const mavlink_gps_raw_int_t
  AUAV3_AND_SLUGS_SENSOR_rtZmavlink_gps_raw_int_t;/* mavlink_gps_raw_int_t ground */
extern const mavlink_raw_pressure_t
  AUAV3_AND_SLUGS_SENSOR_rtZmavlink_raw_pressure_t;/* mavlink_raw_pressure_t ground */
extern const mavlink_sys_status_t AUAV3_AND_SLUGS_SENSOR_rtZmavlink_sys_status_t;/* mavlink_sys_status_t ground */

/* Model entry point functions */
extern void AUAV3_AND_SLUGS_SENSOR_initialize(boolean_T firstTime);
extern void AUAV3_AND_SLUGS_SENSOR_step(int_T tid);

/* Real-time Model object */
extern struct RT_MODEL_AUAV3_AND_SLUGS_SENS_T *const AUAV3_AND_SLUGS_SENSOR_M;

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
 * '<Root>' : 'AUAV3_AND_SLUGS_SENSOR'
 * '<S1>'   : 'AUAV3_AND_SLUGS_SENSOR/Control Surface Input'
 * '<S2>'   : 'AUAV3_AND_SLUGS_SENSOR/Counter Free-Running'
 * '<S3>'   : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO'
 * '<S4>'   : 'AUAV3_AND_SLUGS_SENSOR/MavlinkTX'
 * '<S5>'   : 'AUAV3_AND_SLUGS_SENSOR/Model Info'
 * '<S6>'   : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180'
 * '<S7>'   : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883'
 * '<S8>'   : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data'
 * '<S9>'   : 'AUAV3_AND_SLUGS_SENSOR/Servo In Out'
 * '<S10>'  : 'AUAV3_AND_SLUGS_SENSOR/Control Surface Input/Convert to Microseconds '
 * '<S11>'  : 'AUAV3_AND_SLUGS_SENSOR/Control Surface Input/myMux Fun5'
 * '<S12>'  : 'AUAV3_AND_SLUGS_SENSOR/Counter Free-Running/Increment Real World'
 * '<S13>'  : 'AUAV3_AND_SLUGS_SENSOR/Counter Free-Running/Wrap To Zero'
 * '<S14>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 1 (Blue)'
 * '<S15>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 2 (red)'
 * '<S16>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 3 (Green)'
 * '<S17>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 4 (Yellow)'
 * '<S18>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/BMP 180 Temperature Compensation See datasheet'
 * '<S19>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter'
 * '<S20>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)'
 * '<S21>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/LPF 1Hz'
 * '<S22>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/LPF 1Hz1'
 * '<S23>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Pack'
 * '<S24>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Pack1'
 * '<S25>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Script Pressure to Altitude'
 * '<S26>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Compute Barometric Height'
 * '<S27>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Enabled Subsystem'
 * '<S28>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S29>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Initial Baro Bias'
 * '<S30>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Zero Out Height'
 * '<S31>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S32>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S33>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack10'
 * '<S34>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack11'
 * '<S35>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack12'
 * '<S36>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack2'
 * '<S37>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack3'
 * '<S38>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack4'
 * '<S39>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack5'
 * '<S40>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack6'
 * '<S41>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack7'
 * '<S42>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack8'
 * '<S43>'  : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack9'
 * '<S44>'  : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem10'
 * '<S45>'  : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem8'
 * '<S46>'  : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem9'
 * '<S47>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Raw HIL  Readings'
 * '<S48>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite'
 * '<S49>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond '
 * '<S50>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Compute CPU Load'
 * '<S51>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors'
 * '<S52>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then update Air Data'
 * '<S53>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration'
 * '<S54>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/myMux Fun'
 * '<S55>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/myMux Fun1'
 * '<S56>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition'
 * '<S57>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1'
 * '<S58>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2'
 * '<S59>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis'
 * '<S60>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis '
 * '<S61>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis  '
 * '<S62>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function'
 * '<S63>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function1'
 * '<S64>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function2'
 * '<S65>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun'
 * '<S66>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S67>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function1'
 * '<S68>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function2'
 * '<S69>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/myMux Fun'
 * '<S70>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S71>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function1'
 * '<S72>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function2'
 * '<S73>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/myMux Fun'
 * '<S74>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function'
 * '<S75>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function1'
 * '<S76>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function2'
 * '<S77>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function3'
 * '<S78>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Novatel'
 * '<S79>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Ublox'
 * '<S80>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun'
 * '<S81>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4'
 * '<S82>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter'
 * '<S83>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Barometer Unit Conversion'
 * '<S84>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Unit Conversion'
 * '<S85>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Power Unit Conversion'
 * '<S86>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1'
 * '<S87>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro'
 * '<S88>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Thermistor Unit Conversion'
 * '<S89>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1'
 * '<S90>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2'
 * '<S91>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3'
 * '<S92>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4'
 * '<S93>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height'
 * '<S94>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enabled Subsystem'
 * '<S95>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S96>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/GS Height'
 * '<S97>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias'
 * '<S98>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Zero Out Height'
 * '<S99>'  : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S100>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S101>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Compare To Constant'
 * '<S102>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Hi Temp Compensation2'
 * '<S103>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Lo Temp Compensation'
 * '<S104>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Compare To Constant'
 * '<S105>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Hi Temp Compensation'
 * '<S106>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Lo Temp Compensation'
 * '<S107>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S108>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S109>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3/Embedded MATLAB Function'
 * '<S110>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4/Embedded MATLAB Function'
 * '<S111>' : 'AUAV3_AND_SLUGS_SENSOR/Servo In Out/Mixing'
 */
#endif                                 /* RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
