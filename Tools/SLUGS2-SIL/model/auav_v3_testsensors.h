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
 * File: AUAV_V3_TestSensors.h
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.138
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Mon Aug 15 17:25:56 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Mon Aug 15 17:25:58 2016
 */

#ifndef RTW_HEADER_AUAV_V3_TestSensors_h_
#define RTW_HEADER_AUAV_V3_TestSensors_h_
#include <string.h>
#include <stddef.h>
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "Barometer_Driver.h"
#include "IMU_Mag_Driver.h"
#include "LEDs_Driver.h"
#include "Mavlink_TX_Adapter.h"
#include "PilotConsole.h"
#define FCY                            7.0E+7

/* Include for pic 33E */
#include <p33Exxxx.h>
//SLUGS2
//#ifndef SLUGS2
#include <libpic30.h>                  /* For possible use with C function Call block (delay_ms or delay_us functions might be used by few peripherals) */
#include <libq.h>                      /* For possible use with C function Call block */
//#endif
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

/* Block signals (auto storage) */
typedef struct {
  mavlink_gps_raw_int_t GetRawGpsInt;  /* '<S6>/Get RawGpsInt' */
  mavlink_sys_status_t GetmlSysStatus; /* '<S6>/Get mlSysStatus' */
  mavlink_raw_imu_t GetRawIMU;         /* '<S6>/Get Raw IMU' */
  mavlink_rc_channels_raw_t GetRawRC;  /* '<S6>/Get Raw RC' */
  mavlink_vfr_hud_t GetVfrHud;         /* '<S6>/Get VfrHud' */
  mavlink_scaled_pressure_t GetmlAirData;/* '<S6>/Get mlAirData' */
  real32_T Sum;                        /* '<S22>/Sum' */
  real32_T u0k120k;                    /* '<S21>/[80k - 120k]' */
  real32_T In1;                        /* '<S19>/In1' */
  uint32_T Gettime;                    /* '<S6>/Get time' */
  uint32_T Gettime1;                   /* '<S6>/Get time1' */
  uint32_T Gettime2;                   /* '<S6>/Get time2' */
  uint32_T Gettime3;                   /* '<S6>/Get time3' */
  uint32_T Gettime4;                   /* '<S6>/Get time4' */
  int16_T RateTransition11;            /* '<S12>/Rate Transition11' */
  int16_T RateTransition12;            /* '<S12>/Rate Transition12' */
  int16_T RateTransition2;             /* '<S12>/Rate Transition2' */
  int16_T RateTransition3;             /* '<S12>/Rate Transition3' */
  int16_T RateTransition4;             /* '<S12>/Rate Transition4' */
  int16_T RateTransition8;             /* '<S12>/Rate Transition8' */
  int16_T RateTransition9;             /* '<S12>/Rate Transition9' */
  uint16_T U3CH4;                      /* '<Root>/MCU Load' */
  uint16_T dT;                         /* '<S2>/Input Capture' */
  uint16_T dA;                         /* '<S2>/Input Capture' */
  uint16_T dE;                         /* '<S2>/Input Capture' */
  uint16_T dR;                         /* '<S2>/Input Capture' */
  uint16_T dFailsafe;                  /* '<S2>/Input Capture' */
  uint16_T InputCaptureRCReceiver1_o2; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o3; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T InputCaptureRCReceiver1_o5; /* '<S9>/Input Capture RC Receiver1' */
  uint16_T Saturation;                 /* '<S9>/Saturation' */
  uint16_T Saturation1;                /* '<S9>/Saturation1' */
  uint16_T PackRawIMU;                 /* '<S6>/PackRawIMU' */
  uint16_T PackHeartBeat;              /* '<S6>/PackHeartBeat' */
  uint16_T PackGpsRawInt;              /* '<S6>/PackGpsRawInt' */
  uint16_T PackScaledPressure;         /* '<S6>/PackScaledPressure' */
  uint16_T PackSysStatus;              /* '<S6>/PackSysStatus' */
  uint16_T ParamInterfaceResponse;     /* '<S6>/ParamInterfaceResponse' */
  uint16_T MissionInterfaceResponse;   /* '<S6>/MissionInterfaceResponse' */
  uint16_T PackRawRC;                  /* '<S6>/PackRawRC' */
  uint16_T PackHUD;                    /* '<S6>/PackHUD' */
  uint16_T U1CH8[3];                   /* '<S4>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T U1CH4;                      /* '<S4>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T BUSSPIReadMPU6050AxyzTGxyz1kHz_[3];/* '<S4>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  uint16_T RateTransition5;            /* '<S12>/Rate Transition5' */
  uint16_T RateTransition6;            /* '<S12>/Rate Transition6' */
  uint16_T RateTransition7;            /* '<S12>/Rate Transition7' */
  uint16_T cmdPITCHraw;                /* '<S9>/Input Capture RC Receiver1' */
  uint16_T cmdROLLraw;                 /* '<S9>/Input Capture RC Receiver1' */
  uint8_T BUSI2CReadHMC5883Magn50Hz[6];/* '<S4>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  uint8_T BUSI2CInitializeBMP180ReadTConv[2];/* '<S1>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  uint8_T BUSI2CInitializeBMP180ReadPConv[2];/* '<S1>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  uint8_T BUSI2CInitializeBMP180readCalib[22];/* '<S12>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
  boolean_T DigitalOutputRead_o2;      /* '<S44>/Digital Output Read' */
  boolean_T LogicalOperator;           /* '<S5>/Logical Operator' */
  boolean_T DigitalOutputRead_o2_h;    /* '<S45>/Digital Output Read' */
  boolean_T LogicalOperator1;          /* '<S5>/Logical Operator1' */
  boolean_T DigitalOutputRead_o2_d;    /* '<S46>/Digital Output Read' */
  boolean_T LogicalOperator2;          /* '<S5>/Logical Operator2' */
  boolean_T DigitalOutputRead_o2_hf;   /* '<S47>/Digital Output Read' */
  boolean_T LogicalOperator3;          /* '<S5>/Logical Operator3' */
} BlockIO_AUAV_V3_TestSensors_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteZeroPole_DSTATE;      /* '<S24>/Discrete Zero-Pole' */
  real_T aveCount;                     /* '<S11>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T tIni;                         /* '<S11>/Enables//Disables the Computation of  initial Baro Bias' */
  real32_T IntegerDelay_DSTATE;        /* '<S22>/Integer Delay' */
  uint32_T Output_DSTATE;              /* '<S3>/Output' */
  uint32_T Delay11_DSTATE;             /* '<S13>/Delay11' */
  uint32_T Delay11_DSTATE_j;           /* '<S14>/Delay11' */
  uint32_T time_since_boot_usec;       /* '<Root>/TimeSinceBoot_usec' */
  uint16_T Delay_DSTATE;               /* '<S1>/Delay' */
} D_Work_AUAV_V3_TestSensors_T;

/* Real-time Model Data Structure */
struct tag_RTM_AUAV_V3_TestSensors_T {
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
extern BlockIO_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_B;

/* Block states (auto storage) */
extern D_Work_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_DWork;

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
extern mavlink_raw_imu_t mlRawIMU;     /* '<Root>/mlRawIMU' */
extern mavlink_rc_channels_raw_t mlRC_Commands;/* '<Root>/mlRawRC' */
extern mavlink_servo_output_raw_t mlPwmCommands;/* '<Root>/mlRawServo' */
extern mavlink_vfr_hud_t mlVfr_hud;    /* '<Root>/mlVfr_hud' */
extern mavlink_scaled_pressure_t mlAirData;/* '<Root>/mlAirData' */
extern mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
extern uint16_T MPU_T;                 /* '<Root>/MPU_T' */

/* External data declarations for dependent source files */
extern const mavlink_scaled_pressure_t
  AUAV_V3_TestSensors_rtZmavlink_scaled_pressure_t;/* mavlink_scaled_pressure_t ground */
extern const mavlink_gps_raw_int_t AUAV_V3_TestSensors_rtZmavlink_gps_raw_int_t;/* mavlink_gps_raw_int_t ground */
extern const mavlink_raw_imu_t AUAV_V3_TestSensors_rtZmavlink_raw_imu_t;/* mavlink_raw_imu_t ground */
extern const mavlink_raw_pressure_t
  AUAV_V3_TestSensors_rtZmavlink_raw_pressure_t;/* mavlink_raw_pressure_t ground */
extern const mavlink_rc_channels_raw_t
  AUAV_V3_TestSensors_rtZmavlink_rc_channels_raw_t;/* mavlink_rc_channels_raw_t ground */
extern const mavlink_servo_output_raw_t
  AUAV_V3_TestSensors_rtZmavlink_servo_output_raw_t;/* mavlink_servo_output_raw_t ground */
extern const mavlink_sys_status_t AUAV_V3_TestSensors_rtZmavlink_sys_status_t;/* mavlink_sys_status_t ground */
extern const mavlink_vfr_hud_t AUAV_V3_TestSensors_rtZmavlink_vfr_hud_t;/* mavlink_vfr_hud_t ground */

/* Model entry point functions */
extern void AUAV_V3_TestSensors_initialize(void);
extern void AUAV_V3_TestSensors_step(int_T tid);

/* Real-time Model object */
extern RT_MODEL_AUAV_V3_TestSensors_T *const AUAV_V3_TestSensors_M;

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
 * '<Root>' : 'AUAV_V3_TestSensors'
 * '<S1>'   : 'AUAV_V3_TestSensors/Barometer_Driver'
 * '<S2>'   : 'AUAV_V3_TestSensors/Control Surface Input'
 * '<S3>'   : 'AUAV_V3_TestSensors/Counter Free-Running'
 * '<S4>'   : 'AUAV_V3_TestSensors/IMU_Mag_Driver'
 * '<S5>'   : 'AUAV_V3_TestSensors/LEDs_Driver'
 * '<S6>'   : 'AUAV_V3_TestSensors/Mavlink_TX_Adapter'
 * '<S7>'   : 'AUAV_V3_TestSensors/Model Info'
 * '<S8>'   : 'AUAV_V3_TestSensors/Servo In Out'
 * '<S9>'   : 'AUAV_V3_TestSensors/Servo_Driver'
 * '<S10>'  : 'AUAV_V3_TestSensors/Barometer_Driver/BMP 180 Temperature Compensation See datasheet'
 * '<S11>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter'
 * '<S12>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)'
 * '<S13>'  : 'AUAV_V3_TestSensors/Barometer_Driver/LPF 1Hz'
 * '<S14>'  : 'AUAV_V3_TestSensors/Barometer_Driver/LPF 1Hz1'
 * '<S15>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Pack'
 * '<S16>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Pack1'
 * '<S17>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Script Pressure to Altitude'
 * '<S18>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Compute Barometric Height'
 * '<S19>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Enabled Subsystem'
 * '<S20>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S21>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Initial Baro Bias'
 * '<S22>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Zero Out Height'
 * '<S23>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S24>'  : 'AUAV_V3_TestSensors/Barometer_Driver/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S25>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack10'
 * '<S26>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack11'
 * '<S27>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack12'
 * '<S28>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack2'
 * '<S29>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack3'
 * '<S30>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack4'
 * '<S31>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack5'
 * '<S32>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack6'
 * '<S33>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack7'
 * '<S34>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack8'
 * '<S35>'  : 'AUAV_V3_TestSensors/Barometer_Driver/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)/Pack9'
 * '<S36>'  : 'AUAV_V3_TestSensors/Control Surface Input/PilotConsole'
 * '<S37>'  : 'AUAV_V3_TestSensors/Control Surface Input/PilotConsole/Convert to Microseconds '
 * '<S38>'  : 'AUAV_V3_TestSensors/Control Surface Input/PilotConsole/myMux Fun5'
 * '<S39>'  : 'AUAV_V3_TestSensors/Counter Free-Running/Increment Real World'
 * '<S40>'  : 'AUAV_V3_TestSensors/Counter Free-Running/Wrap To Zero'
 * '<S41>'  : 'AUAV_V3_TestSensors/IMU_Mag_Driver/Subsystem10'
 * '<S42>'  : 'AUAV_V3_TestSensors/IMU_Mag_Driver/Subsystem8'
 * '<S43>'  : 'AUAV_V3_TestSensors/IMU_Mag_Driver/Subsystem9'
 * '<S44>'  : 'AUAV_V3_TestSensors/LEDs_Driver/LED 1 (Blue)'
 * '<S45>'  : 'AUAV_V3_TestSensors/LEDs_Driver/LED 2 (red)'
 * '<S46>'  : 'AUAV_V3_TestSensors/LEDs_Driver/LED 3 (Green)'
 * '<S47>'  : 'AUAV_V3_TestSensors/LEDs_Driver/LED 4 (Yellow)'
 * '<S48>'  : 'AUAV_V3_TestSensors/Servo In Out/Mixing'
 * '<S49>'  : 'AUAV_V3_TestSensors/Servo_Driver/Mixing'
 */
#endif                                 /* RTW_HEADER_AUAV_V3_TestSensors_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
