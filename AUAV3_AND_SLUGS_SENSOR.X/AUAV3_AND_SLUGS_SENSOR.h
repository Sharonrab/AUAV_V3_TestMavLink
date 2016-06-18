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
 * Model version                        : 1.223
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Fri Jun 17 19:05:27 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Fri Jun 17 19:05:28 2016
 */

#ifndef RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_
#define RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_
# define AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV3_AND_SLUGS_SENSOR_COMMON_INCLUDES_ */

#include "AUAV3_AND_SLUGS_SENSOR_types.h"

/* Shared type includes */
#include "multiword_types.h"
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
#include "gpsPort.h"
#include "MavlinkComm.h"

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

/* Block signals for system '<S123>/Enabled Subsystem' */
typedef struct {
  real32_T In1;                        /* '<S131>/In1' */
} rtB_EnabledSubsystem_AUAV3_AN_T;

/* Block signals for system '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T tOut;                         /* '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
} rtB_EnablesDisablestheComputa_T;

/* Block states (auto storage) for system '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T aveCount;                     /* '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T tIni;                         /* '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
} rtDW_EnablesDisablestheComput_T;

/* Block signals for system '<S160>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S160>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_AU_T;

/* Block states (auto storage) for system '<S160>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S160>/Embedded MATLAB Function' */
  real_T b;                            /* '<S160>/Embedded MATLAB Function' */
  real_T y_km1;                        /* '<S160>/Embedded MATLAB Function' */
  real_T u_km1;                        /* '<S160>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S160>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_A_T;

/* Block signals for system '<S155>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S155>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_n_T;

/* Block signals (auto storage) */
typedef struct {
  mavlink_gps_raw_int_t GetRawGpsInt;  /* '<S4>/Get RawGpsInt' */
  mavlink_sys_status_t GetmlSysStatus; /* '<S4>/Get mlSysStatus' */
  mavlink_raw_imu_t GetRawIMU;         /* '<S4>/Get Raw IMU' */
  mavlink_rc_channels_raw_t GetRawRC;  /* '<S4>/Get Raw RC' */
  mavlink_vfr_hud_t GetVfrHud;         /* '<S4>/Get VfrHud' */
  mavlink_scaled_pressure_t GetmlAirData;/* '<S4>/Get mlAirData' */
  real32_T Merge;                      /* '<S190>/Merge' */
  real32_T Merge_k;                    /* '<S191>/Merge' */
  real32_T GettheGSLocationupdateSensorMCU[3];/* '<S200>/Get the GS Location [updateSensorMCUState.c]1' */
  real32_T y[4];                       /* '<S152>/myMux Fun1' */
  real32_T Sum;                        /* '<S202>/Sum' */
  real32_T u0k120k;                    /* '<S201>/[80k - 120k]' */
  real32_T AirData[3];                 /* '<S156>/AirData' */
  real32_T ProducetheGPSMainDataandupdatet[5];/* '<S183>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
  real32_T ProducetheGPSMainDataandupdat_c[5];/* '<S182>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]2' */
  real32_T Sum_k;                      /* '<S134>/Sum' */
  real32_T u0k120k_i;                  /* '<S133>/[80k - 120k]' */
  uint32_T Gettime;                    /* '<S4>/Get time' */
  uint32_T Gettime1;                   /* '<S4>/Get time1' */
  uint32_T Gettime2;                   /* '<S4>/Get time2' */
  uint32_T Gettime3;                   /* '<S4>/Get time3' */
  uint32_T Gettime4;                   /* '<S4>/Get time4' */
  int16_T ReadtheCubeDataadisCube16405c1[10];/* '<S155>/Read the Cube Data [adisCube16405.c]1' */
  int16_T y_g[13];                     /* '<S155>/myMux Fun4' */
  int16_T y_b[4];                      /* '<S155>/myMux Fun' */
  int16_T RateTransition11;            /* '<S124>/Rate Transition11' */
  int16_T RateTransition12;            /* '<S124>/Rate Transition12' */
  int16_T RateTransition2;             /* '<S124>/Rate Transition2' */
  int16_T RateTransition3;             /* '<S124>/Rate Transition3' */
  int16_T RateTransition4;             /* '<S124>/Rate Transition4' */
  int16_T RateTransition8;             /* '<S124>/Rate Transition8' */
  int16_T RateTransition9;             /* '<S124>/Rate Transition9' */
  uint16_T U3CH4;                      /* '<Root>/MCU Load' */
  uint16_T Saturation;                 /* '<S10>/Saturation' */
  uint16_T Saturation1;                /* '<S10>/Saturation1' */
  uint16_T U1CH8[3];                   /* '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T U1CH4;                      /* '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T BUSSPIReadMPU6050AxyzTGxyz1kHz_[3];/* '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T dT;                         /* '<S1>/Input Capture' */
  uint16_T dA;                         /* '<S1>/Input Capture' */
  uint16_T dE;                         /* '<S1>/Input Capture' */
  uint16_T dR;                         /* '<S1>/Input Capture' */
  uint16_T dFailsafe;                  /* '<S1>/Input Capture' */
  uint16_T CalculusTimeStep1;          /* '<S154>/Calculus Time Step1' */
  uint16_T CalculusTimeStep2;          /* '<S154>/Calculus Time Step2' */
  uint16_T DataTypeConversion8;        /* '<S157>/Data Type Conversion8' */
  uint16_T PackRawIMU;                 /* '<S4>/PackRawIMU' */
  uint16_T PackHeartBeat;              /* '<S4>/PackHeartBeat' */
  uint16_T PackGpsRawInt;              /* '<S4>/PackGpsRawInt' */
  uint16_T PackScaledPressure;         /* '<S4>/PackScaledPressure' */
  uint16_T PackSysStatus;              /* '<S4>/PackSysStatus' */
  uint16_T ParamInterfaceResponse;     /* '<S4>/ParamInterfaceResponse' */
  uint16_T ParamInterfaceResponse1;    /* '<S4>/ParamInterfaceResponse1' */
  uint16_T PackRawRC;                  /* '<S4>/PackRawRC' */
  uint16_T PackRawRC1;                 /* '<S4>/PackRawRC1' */
  uint16_T RateTransition5;            /* '<S124>/Rate Transition5' */
  uint16_T RateTransition6;            /* '<S124>/Rate Transition6' */
  uint16_T RateTransition7;            /* '<S124>/Rate Transition7' */
  uint16_T cmdPITCHraw;                /* '<S10>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o2; /* '<S10>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o3; /* '<S10>/Input Capture RC Receiver1' */
  uint16_T cmdROLLraw;                 /* '<S10>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o5; /* '<S10>/Input Capture RC Receiver1' */
  uint8_T BUSI2CInitializeBMP180ReadTConv[2];/* '<S7>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  uint8_T BUSI2CInitializeBMP180ReadPConv[2];/* '<S7>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  uint8_T BUSI2CReadHMC5883Magn50Hz[6];/* '<S8>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  uint8_T DataTypeConversion12;        /* '<S154>/Data Type Conversion12' */
  uint8_T IstheGPSNovatelorUbloxgpsPortc1;/* '<S155>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
  uint8_T ReadtheRawDatafromGPSgpsPortc2;/* '<S182>/Read the Raw Data from GPS [gpsPort.c]2' */
  uint8_T DatafromHILhilc2[13];        /* '<S151>/Data from HIL [hil.c]2' */
  uint8_T HILMessagesParserDecoderhilc1[13];/* '<S151>/HIL Messages  Parser//Decoder [hil.c]1' */
  uint8_T HILRawReadingshilc1[13];     /* '<S151>/HIL Raw Readings [hil.c]1' */
  uint8_T BUSI2CInitializeBMP180readCalib[22];/* '<S124>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
  boolean_T DigitalOutputRead_o2;      /* '<S18>/Digital Output Read' */
  boolean_T DigitalOutputRead_o2_h;    /* '<S19>/Digital Output Read' */
  boolean_T DigitalOutputRead_o2_d;    /* '<S20>/Digital Output Read' */
  boolean_T DigitalOutputRead_o2_hf;   /* '<S21>/Digital Output Read' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_a;/* '<S196>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_o;/* '<S195>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_j;/* '<S194>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction;/* '<S193>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputat_m;/* '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_AUAV3_AN_T EnabledSubsystem_e;/* '<S186>/Enabled Subsystem' */
  rtB_EmbeddedMATLABFunction_n_T sf_EmbeddedMATLABFunction3;/* '<S155>/Embedded MATLAB Function3' */
  rtB_EmbeddedMATLABFunction_n_T sf_EmbeddedMATLABFunction2_db;/* '<S155>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_n_T sf_EmbeddedMATLABFunction1_m;/* '<S155>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_n_T sf_EmbeddedMATLABFunction_i;/* '<S155>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2_f;/* '<S162>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1_d;/* '<S162>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_d;/* '<S162>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2_d;/* '<S161>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1_h;/* '<S161>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_ph;/* '<S161>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction2;/* '<S160>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction1;/* '<S160>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_AU_T sf_EmbeddedMATLABFunction_p;/* '<S160>/Embedded MATLAB Function' */
  rtB_EnablesDisablestheComputa_T sf_EnablesDisablestheComputatio;/* '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
  rtB_EnabledSubsystem_AUAV3_AN_T EnabledSubsystem;/* '<S123>/Enabled Subsystem' */
} BlockIO_AUAV3_AND_SLUGS_SENSO_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  mavlink_raw_imu_t mlRawIMU;          /* '<Root>/mlRawIMU' */
  mavlink_rc_channels_raw_t mlRC_Commands;/* '<Root>/mlRawRC' */
  mavlink_servo_output_raw_t mlPwmCommands;/* '<Root>/mlRawServo' */
  mavlink_vfr_hud_t mlVfr_hud;         /* '<Root>/mlVfr_hud' */
  real_T DiscreteZeroPole_DSTATE;      /* '<S204>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_n;    /* '<S136>/Discrete Zero-Pole' */
  real32_T IntegerDelay_DSTATE;        /* '<S202>/Integer Delay' */
  real32_T IntegerDelay_DSTATE_j;      /* '<S134>/Integer Delay' */
  uint32_T Output_DSTATE;              /* '<S2>/Output' */
  uint32_T Delay11_DSTATE;             /* '<S125>/Delay11' */
  uint32_T Delay11_DSTATE_j;           /* '<S126>/Delay11' */
  uint32_T time_since_boot_usec;       /* '<Root>/TimeSinceBoot_usec' */
  uint16_T Delay_DSTATE;               /* '<S7>/Delay' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_a;/* '<S196>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_o;/* '<S195>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_j;/* '<S194>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction;/* '<S193>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputat_m;/* '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2_f;/* '<S162>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1_d;/* '<S162>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_d;/* '<S162>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2_d;/* '<S161>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1_h;/* '<S161>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_ph;/* '<S161>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction2;/* '<S160>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction1;/* '<S160>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_A_T sf_EmbeddedMATLABFunction_p;/* '<S160>/Embedded MATLAB Function' */
  rtDW_EnablesDisablestheComput_T sf_EnablesDisablestheComputatio;/* '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
} D_Work_AUAV3_AND_SLUGS_SENSOR_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: [1 1 1 1 1])
   * Referenced by:
   *   '<S155>/Constant'
   *   '<S155>/Constant1'
   *   '<S155>/Constant2'
   *   '<S155>/Constant3'
   */
  real_T pooled5[5];
} ConstParam_AUAV3_AND_SLUGS_SE_T;

/* Real-time Model Data Structure */
struct tag_RTM_AUAV3_AND_SLUGS_SENSO_T {
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
extern void AUAV3_AND_SLUGS_SENSOR_initialize(void);
extern void AUAV3_AND_SLUGS_SENSOR_step(int_T tid);

/* Real-time Model object */
extern RT_MODEL_AUAV3_AND_SLUGS_SENS_T *const AUAV3_AND_SLUGS_SENSOR_M;

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
 * '<S6>'   : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter'
 * '<S7>'   : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180'
 * '<S8>'   : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883'
 * '<S9>'   : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data'
 * '<S10>'  : 'AUAV3_AND_SLUGS_SENSOR/Servo In Out'
 * '<S11>'  : 'AUAV3_AND_SLUGS_SENSOR/myMux Fun2'
 * '<S12>'  : 'AUAV3_AND_SLUGS_SENSOR/myMux Fun3'
 * '<S13>'  : 'AUAV3_AND_SLUGS_SENSOR/myMux Fun4'
 * '<S14>'  : 'AUAV3_AND_SLUGS_SENSOR/Control Surface Input/Convert to Microseconds '
 * '<S15>'  : 'AUAV3_AND_SLUGS_SENSOR/Control Surface Input/myMux Fun5'
 * '<S16>'  : 'AUAV3_AND_SLUGS_SENSOR/Counter Free-Running/Increment Real World'
 * '<S17>'  : 'AUAV3_AND_SLUGS_SENSOR/Counter Free-Running/Wrap To Zero'
 * '<S18>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 1 (Blue)'
 * '<S19>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 2 (red)'
 * '<S20>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 3 (Green)'
 * '<S21>'  : 'AUAV3_AND_SLUGS_SENSOR/Flashing LEDs DISCO/LED 4 (Yellow)'
 * '<S22>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG'
 * '<S23>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/COG.SOG2V'
 * '<S24>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Compute GS Location'
 * '<S25>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Enabled Subsystem'
 * '<S26>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update'
 * '<S27>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1'
 * '<S28>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter'
 * '<S29>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter'
 * '<S30>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product'
 * '<S31>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1'
 * '<S32>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2'
 * '<S33>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Deriv '
 * '<S34>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles'
 * '<S35>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function'
 * '<S36>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1'
 * '<S37>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2'
 * '<S38>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize'
 * '<S39>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix'
 * '<S40>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/myMux Fun2'
 * '<S41>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/q dot calc'
 * '<S42>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter1'
 * '<S43>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter2'
 * '<S44>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter3'
 * '<S45>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1'
 * '<S46>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem'
 * '<S47>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem1'
 * '<S48>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem'
 * '<S49>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem1'
 * '<S50>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem'
 * '<S51>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem1'
 * '<S52>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotDefault'
 * '<S53>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotZeroR3'
 * '<S54>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/Get DCM Values'
 * '<S55>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm'
 * '<S56>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2'
 * '<S57>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3'
 * '<S58>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2/negprotect'
 * '<S59>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S60>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm'
 * '<S61>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2'
 * '<S62>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3'
 * '<S63>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2/negprotect'
 * '<S64>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S65>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus'
 * '<S66>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S67>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A11'
 * '<S68>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A12'
 * '<S69>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A13'
 * '<S70>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A21'
 * '<S71>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A22'
 * '<S72>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A23'
 * '<S73>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A31'
 * '<S74>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A32'
 * '<S75>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A33'
 * '<S76>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Create Transformation Matrix'
 * '<S77>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S78>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S79>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S80>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/COG.SOG2V/Angle Conversion'
 * '<S81>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/COG.SOG2V/Subsystem5'
 * '<S82>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/COG.SOG2V/myMux Fun2'
 * '<S83>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Compute GS Location/Geod2ECEF1'
 * '<S84>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S85>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change'
 * '<S86>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change1'
 * '<S87>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change2'
 * '<S88>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change/Difference1'
 * '<S89>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change1/Difference1'
 * '<S90>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/GPS Update/Detect Change2/Difference1'
 * '<S91>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S92>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1/Geod2ECEF1'
 * '<S93>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S94>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S95>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S96>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel'
 * '<S97>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend'
 * '<S98>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter'
 * '<S99>'  : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate'
 * '<S100>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1'
 * '<S101>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly'
 * '<S102>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/Compare To Constant'
 * '<S103>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/Deriv '
 * '<S104>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem'
 * '<S105>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun1'
 * '<S106>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun2'
 * '<S107>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3'
 * '<S108>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4'
 * '<S109>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4/Embedded MATLAB Function'
 * '<S110>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter1'
 * '<S111>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter2'
 * '<S112>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter3'
 * '<S113>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/myMux Fun1'
 * '<S114>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1'
 * '<S115>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1/Filter2'
 * '<S116>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1'
 * '<S117>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1/Filter2'
 * '<S118>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1'
 * '<S119>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1/Filter2'
 * '<S120>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1'
 * '<S121>' : 'AUAV3_AND_SLUGS_SENSOR/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function2'
 * '<S122>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/BMP 180 Temperature Compensation See datasheet'
 * '<S123>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter'
 * '<S124>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)'
 * '<S125>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/LPF 1Hz'
 * '<S126>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/LPF 1Hz1'
 * '<S127>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Pack'
 * '<S128>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Pack1'
 * '<S129>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Script Pressure to Altitude'
 * '<S130>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Compute Barometric Height'
 * '<S131>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Enabled Subsystem'
 * '<S132>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S133>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Initial Baro Bias'
 * '<S134>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Zero Out Height'
 * '<S135>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S136>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S137>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack10'
 * '<S138>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack11'
 * '<S139>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack12'
 * '<S140>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack2'
 * '<S141>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack3'
 * '<S142>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack4'
 * '<S143>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack5'
 * '<S144>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack6'
 * '<S145>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack7'
 * '<S146>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack8'
 * '<S147>' : 'AUAV3_AND_SLUGS_SENSOR/Read Barometer BMP180/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack9'
 * '<S148>' : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem10'
 * '<S149>' : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem8'
 * '<S150>' : 'AUAV3_AND_SLUGS_SENSOR/Read MPU600 and HMC5883/Subsystem9'
 * '<S151>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Raw HIL  Readings'
 * '<S152>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite'
 * '<S153>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond '
 * '<S154>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Compute CPU Load'
 * '<S155>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors'
 * '<S156>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then update Air Data'
 * '<S157>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration'
 * '<S158>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/myMux Fun'
 * '<S159>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/myMux Fun1'
 * '<S160>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition'
 * '<S161>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1'
 * '<S162>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2'
 * '<S163>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis'
 * '<S164>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis '
 * '<S165>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis  '
 * '<S166>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function'
 * '<S167>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function1'
 * '<S168>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function2'
 * '<S169>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun'
 * '<S170>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S171>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function1'
 * '<S172>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function2'
 * '<S173>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/myMux Fun'
 * '<S174>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S175>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function1'
 * '<S176>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function2'
 * '<S177>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/myMux Fun'
 * '<S178>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function'
 * '<S179>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function1'
 * '<S180>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function2'
 * '<S181>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function3'
 * '<S182>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Novatel'
 * '<S183>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Ublox'
 * '<S184>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun'
 * '<S185>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4'
 * '<S186>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter'
 * '<S187>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Barometer Unit Conversion'
 * '<S188>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Unit Conversion'
 * '<S189>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Power Unit Conversion'
 * '<S190>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1'
 * '<S191>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro'
 * '<S192>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Thermistor Unit Conversion'
 * '<S193>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1'
 * '<S194>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2'
 * '<S195>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3'
 * '<S196>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4'
 * '<S197>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height'
 * '<S198>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enabled Subsystem'
 * '<S199>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S200>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/GS Height'
 * '<S201>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias'
 * '<S202>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Zero Out Height'
 * '<S203>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S204>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S205>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Compare To Constant'
 * '<S206>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Hi Temp Compensation2'
 * '<S207>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Accel X1/Lo Temp Compensation'
 * '<S208>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Compare To Constant'
 * '<S209>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Hi Temp Compensation'
 * '<S210>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Lo Temp Compensation'
 * '<S211>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S212>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S213>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3/Embedded MATLAB Function'
 * '<S214>' : 'AUAV3_AND_SLUGS_SENSOR/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4/Embedded MATLAB Function'
 * '<S215>' : 'AUAV3_AND_SLUGS_SENSOR/Servo In Out/Mixing'
 */
#endif                                 /* RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
