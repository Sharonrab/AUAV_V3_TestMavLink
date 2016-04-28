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
 * File: UnitTest.h
 *
 * Real-Time Workshop code generated for Simulink model UnitTest.
 *
 * Model version                        : 1.210
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Sat Apr 23 21:27:11 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Sat Apr 23 21:27:11 2016
 */

#ifndef RTW_HEADER_UnitTest_h_
#define RTW_HEADER_UnitTest_h_
#ifndef UnitTest_COMMON_INCLUDES_
# define UnitTest_COMMON_INCLUDES_
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* UnitTest_COMMON_INCLUDES_ */

#include "UnitTest_types.h"
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
#define UNIT_CODE

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

/* Block signals (auto storage) */
typedef struct {
  real32_T ProducetheGPSMainDataandupdatet[5];/* '<S3>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]1' */
  uint16_T U3CH4;                      /* '<Root>/MCU Load' */
  uint8_T ReadtheRawDatafromGPSgpsPortc1;/* '<S3>/Read the Raw Data from GPS [gpsPort.c]1' */
} BlockIO_UnitTest_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  mavlink_raw_imu_t mlRawIMU;          /* '<Root>/mlRawIMU' */
  uint32_T Output_DSTATE;              /* '<S1>/Output' */
  uint32_T time_since_boot_usec;       /* '<Root>/TimeSinceBoot_usec' */
} D_Work_UnitTest_T;

/* Real-time Model Data Structure */
struct RT_MODEL_UnitTest_T {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
      uint8_T cLimit[2];
    } TaskCounters;
  } Timing;
};

/* Block signals (auto storage) */
extern BlockIO_UnitTest_T UnitTest_B;

/* Block states (auto storage) */
extern D_Work_UnitTest_T UnitTest_DWork;

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
extern const mavlink_scaled_pressure_t UnitTest_rtZmavlink_scaled_pressure_t;/* mavlink_scaled_pressure_t ground */
extern const mavlink_gps_raw_int_t UnitTest_rtZmavlink_gps_raw_int_t;/* mavlink_gps_raw_int_t ground */
extern const mavlink_raw_pressure_t UnitTest_rtZmavlink_raw_pressure_t;/* mavlink_raw_pressure_t ground */
extern const mavlink_sys_status_t UnitTest_rtZmavlink_sys_status_t;/* mavlink_sys_status_t ground */

/* Model entry point functions */
extern void UnitTest_initialize(boolean_T firstTime);
extern void UnitTest_step(int_T tid);

/* Real-time Model object */
extern struct RT_MODEL_UnitTest_T *const UnitTest_M;

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
 * '<Root>' : 'UnitTest'
 * '<S1>'   : 'UnitTest/Counter Free-Running'
 * '<S2>'   : 'UnitTest/Model Info'
 * '<S3>'   : 'UnitTest/if GPS is Novatel'
 * '<S4>'   : 'UnitTest/Counter Free-Running/Increment Real World'
 * '<S5>'   : 'UnitTest/Counter Free-Running/Wrap To Zero'
 */
#endif                                 /* RTW_HEADER_UnitTest_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
