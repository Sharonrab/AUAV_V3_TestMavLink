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
 * File: AUAV3_AND_SLUGS_SENSOR_private.h
 *
 * Real-Time Workshop code generated for Simulink model AUAV3_AND_SLUGS_SENSOR.
 *
 * Model version                        : 1.208
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Wed Apr 27 00:58:09 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Wed Apr 27 00:58:11 2016
 */

#ifndef RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_private_h_
#define RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_private_h_
#include "rtwtypes.h"
#define CALL_EVENT                     (-1)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error "Code was generated for compiler with different sized uchar/char. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compiler's limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, which will disable the preprocessor word size checks."
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized ushort/short. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized uint/int. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error "Code was generated for compiler with different sized ulong/long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#else
#ifdef TMWTYPES_PREVIOUSLY_INCLUDED
#error This file requires rtwtypes.h to be included before tmwtypes.h
#else

/* Check for inclusion of an incorrect version of rtwtypes.h */
#ifndef RTWTYPES_ID_C08S16I16L32N16F1
#error This code was generated with a different "rtwtypes.h" than the file included
#endif                                 /* RTWTYPES_ID_C08S16I16L32N16F1 */
#endif                                 /* TMWTYPES_PREVIOUSLY_INCLUDED */
#endif                                 /* __RTWTYPES_H__ */

extern volatile uint16_T mcuFlagRecursion __attribute__ ((near));
extern volatile uint16_T MCHP_MCULoadResult[] __attribute__ ((near));
extern volatile uint16_T MCHP_MCULoadPreviousTimerValue[] __attribute__ ((near));
union
{
  struct {
    unsigned int task0 : 1;
    unsigned int task1 : 1;
    unsigned int task2 : 1;
    unsigned int task3 : 1;
    unsigned int task4 : 1;
    unsigned int task5 : 1;
    unsigned int task6 : 1;
    unsigned int task7 : 1;
    unsigned int task8 : 1;
    unsigned int task9 : 1;
    unsigned int task10 : 1;
  } b;

  unsigned int val;
} extern volatile MCHP_MCU_Overload;

extern uint16_T volatile MCHP_SPI1_State;
extern boolean_T volatile MCHP_SPI12_Request;

/* Declare SPI1 Queue Circular Buffer */
extern MCHP_SPI1_QueueStr MCHP_SPI1_Queue;

/* Declare UART1 Tx Circular Buffer Structure */
extern MCHP_UART1_TxStr MCHP_UART1_Tx;

/* Declare UART4 Tx Circular Buffer Structure */
extern MCHP_UART4_TxStr MCHP_UART4_Tx;
extern uint16_T volatile MCHP_I2C2_State;
extern unsigned int volatile MCHP_I2C22_Request;

/* Declare I2C2 Queue Circular Buffer */
extern MCHP_I2C2_QueueStr MCHP_I2C2_Queue;
extern volatile uint8_T I2C22_Buff8[2];
extern unsigned int volatile MCHP_I2C23_Request;
extern volatile uint8_T I2C23_Buff8[2];
extern unsigned int volatile MCHP_I2C21_Request;
extern volatile uint8_T I2C21_Buff8[22];
extern boolean_T volatile MCHP_SPI11_Request;
extern volatile uint16_T SPI11_Buff16[7];
extern unsigned int volatile MCHP_I2C24_Request;
extern volatile uint8_T I2C24_Buff8[6];
extern unsigned int volatile MCHP_I2C25_Request;
extern volatile uint16_T MCHP_ic1up;
extern volatile uint16_T MCHP_ic2up;
extern volatile uint16_T MCHP_ic3up;
extern volatile uint16_T MCHP_ic4up;
extern volatile uint16_T MCHP_ic5up;

/* C Function Call declare function as extern */
extern real32_T mySqrt(real32_T u1);

/* C Function Call declare function as extern */
extern void getGSLocation(real32_T* y1);

/* C Function Call declare function as extern */
extern void getGpsMainData(real32_T* data);

/* C Function Call declare function as extern */
extern void getGPSRawData(uint8_T* y1);

/* C Function Call declare function as extern */
extern void gpsParse(uint8_T* dataStream);

/* C Function Call declare function as extern */
extern void getGpsUbloxMainData(real32_T* y1);

/* C Function Call declare function as extern */
extern uint8_t isFixValid(void);

/* C Function Call declare function as extern */
extern real32_T myCos(real32_T u1);

/* C Function Call declare function as extern */
extern real32_T mySin(real32_T u1);

/* C Function Call declare function as extern */
extern void getGSLocation(real32_T* y1);

/* C Function Call declare function as extern */
extern void hilRead(uint8_T* y1);

/* C Function Call declare function as extern */
extern void protDecodeHil(uint8_T* u1);

/* C Function Call declare function as extern */
extern void hil_getRawRead(uint8_T* y1);

/* C Function Call declare function as extern */
extern real32_T mySqrt(real32_T u1);
extern volatile uint16_T MCHP_ic1up;
extern volatile uint16_T MCHP_ic2up;
extern volatile uint16_T MCHP_ic3up;
extern volatile uint16_T MCHP_ic4up;
extern volatile uint16_T MCHP_ic5up;
extern volatile uint16_T MCHP_ic6up;

/* C Function Call declare function as extern */
extern uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id,
  mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t PackHeartBeat(uint8_t system_id, uint8_t component_id);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t PackGpsRawInt(uint8_t system_id, uint8_t component_id,
  mavlink_gps_raw_int_t mlRawGpsDataInt ,uint32_t time_usec);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t PackScaledPressure(uint8_t system_id, uint8_t component_id,
  mavlink_scaled_pressure_t mlAirData ,uint32_t time_usec);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t PackSysStatus(uint8_t system_id, uint8_t component_id,
  mavlink_sys_status_t mlSysStatus);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t ParameterInterfaceResponse(uint8_t system_id, uint8_t
  component_id);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern uint16_t MissionInterfaceResponse(uint8_t system_id, uint8_t component_id);

/* C Function Call declare function as extern */
extern void TxN_Data_OverU1(uint16_t N);

/* C Function Call declare function as extern */
extern void gpsUbloxParse(void);
extern int16_T div_s16s32_round(int32_T numerator, int32_T denominator);
extern uint32_T div_u32_round(uint32_T numerator, uint32_T denominator);
extern uint32_T div_repeat_u32_round(uint32_T numerator, uint32_T denominator,
  uint16_T nRepeatSub);
extern void mul_wide_u32(uint32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern uint32_T mul_u32_u32_u32_sr15(uint32_T a, uint32_T b);
extern void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern uint32_T mul_u32_s32_s32_sat(int32_T a, int32_T b);
extern void mul_wide_su32(int32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern int32_T mul_s32_s32_u32_sr16(int32_T a, uint32_T b);
extern uint32_T mul_u32_u32_u32_sr11_round(uint32_T a, uint32_T b);
extern uint32_T mul_u32_s32_u32_sr15(int32_T a, uint32_T b);
extern int32_T mul_s32_s32_s32_sr28(int32_T a, int32_T b);
extern int32_T mul_s32_s32_s32_sr23(int32_T a, int32_T b);
extern uint32_T MultiWord2uLong(const uint32_T u[]);
extern void uMultiWordShr(const uint32_T u1[], int16_T n1, uint16_T n2, uint32_T
  y[], int16_T n);
extern void uMultiWordMul(const uint32_T u1[], int16_T n1, const uint32_T u2[],
  int16_T n2, uint32_T y[], int16_T n);
extern void AUAV3_AND_SLUGS__myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun1_AUAV3_AND_SLUGS_T *localB);
extern void AUAV3_AND_SLUGS_negprotect(real32_T rtu_val,
  rtB_negprotect_AUAV3_AND_SLUG_T *localB);
extern void AUA_EmbeddedMATLABFunction(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_AU_T *localB);
extern void EmbeddedMATLABFunct_a_Init(rtDW_EmbeddedMATLABFunction1__T *localDW);
extern void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW);
extern void AUAV3_AND_SLUG_myMuxFun1_p(real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, rtB_myMuxFun1_AUAV3_AND_SLU_c_T *localB);
extern void AUA_EnabledSubsystem_Start(rtB_EnabledSubsystem_AUAV3_AN_T *localB);
extern void AUAV3_AND_EnabledSubsystem(boolean_T rtu_0, real32_T rtu_1,
  rtB_EnabledSubsystem_AUAV3_AN_T *localB);
extern void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW);
extern void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW);
extern void AUAV3_A_ZeroOutHeight_Init(rtDW_ZeroOutHeight_AUAV3_AND__T *localDW);
extern void AUAV3__ZeroOutHeight_Start(rtDW_ZeroOutHeight_AUAV3_AND__T *localDW);
extern void AUAV3_ZeroOutHeight_Update(real_T rtu_0, real32_T rtu_ComputedHeight,
  rtDW_ZeroOutHeight_AUAV3_AND__T *localDW);
extern void AUAV3_AND_SL_ZeroOutHeight(real_T rtu_0, real32_T rtu_BaseHeight,
  rtB_ZeroOutHeight_AUAV3_AND_S_T *localB, rtDW_ZeroOutHeight_AUAV3_AND__T
  *localDW);
extern void EmbeddedMATLABFunct_m_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW);
extern void A_EmbeddedMATLABFunction_k(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_n_T *localB, rtDW_EmbeddedMATLABFunction_i_T
  *localDW);
extern void AUAV3_AND_SLUGS_S_myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun_AUAV3_AND_SLUGS__T *localB);
extern void EmbeddedMATLABFunction_kd(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_d_T *localB);
extern void AUAV3_AND_SLUGS_SENSOR_step0(void);
extern void AUAV3_AND_SLUGS_SENSOR_step1(void);
extern void AUAV3_AND_SLUGS_SENSOR_step2(void);
extern void AUAV3_AND_SLUGS_SENSOR_step3(void);
extern void AUAV3_AND_SLUGS_SENSOR_step4(void);
extern void AUAV3_AND_SLUGS_SENSOR_step5(void);
extern void AUAV3_AND_SLUGS_SENSOR_step6(void);
extern void AUAV3_AND_SLUGS_SENSOR_step7(void);
extern void AUAV3_AND_SLUGS_SENSOR_step8(void);
extern void AUAV3_AND_SLUGS_SENSOR_step9(void);
extern void AUAV3_AND_SLUGS_SENSOR_step10(void);

#endif                                 /* RTW_HEADER_AUAV3_AND_SLUGS_SENSOR_private_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
