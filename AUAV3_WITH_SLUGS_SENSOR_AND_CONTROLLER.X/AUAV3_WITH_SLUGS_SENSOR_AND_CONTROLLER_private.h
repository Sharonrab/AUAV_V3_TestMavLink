/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h
 *
 * Code generated for Simulink model 'AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER'.
 *
 * Model version                  : 1.288
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Jun 22 16:14:13 2016
 */

#ifndef RTW_HEADER_AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private_h_
#define RTW_HEADER_AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

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
extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);
extern volatile uint16_T MCHP_ic1up;
extern volatile uint16_T MCHP_ic2up;
extern volatile uint16_T MCHP_ic3up;
extern volatile uint16_T MCHP_ic4up;
extern volatile uint16_T MCHP_ic5up;
extern volatile uint16_T MCHP_ic6up;
extern volatile uint16_T MCHP_ic7up;

/* C Function Call declare function as extern */
extern uint16_T meanFilter5(uint8_T *u1);

/* C Function Call declare function as extern */
extern uint8_T isApManual(uint16_T u1);

/* C Function Call declare function as extern */
extern uint8_T justEnabled(uint8_T u1, uint8_T u2);

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
extern uint8_T isFixValid();

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
extern void getRTB(uint8_T* y1);

/* C Function Call declare function as extern */
extern void setDiagnosticFloat(float * flValues);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern void getWP(unsigned char idx, float* WPpos);

/* C Function Call declare function as extern */
extern void getWP(unsigned char idx, float* WPpos);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern void getWP(unsigned char idx, float* WPpos);

/* C Function Call declare function as extern */
extern void getWP(unsigned char idx, float* WPpos);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float myAbs(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float mySqrt(float x);

/* C Function Call declare function as extern */
extern float myAtan2(float num, float denom);

/* C Function Call declare function as extern */
extern void getXYZ(float* xyz);

/* C Function Call declare function as extern */
extern float myAbs(float x);

/* C Function Call declare function as extern */
extern float myAtan(float x);

/* C Function Call declare function as extern */
extern real32_T mySqrt(real32_T u1);

/* C Function Call declare function as extern */
extern real32_T mySqrt(real32_T u1);

/* C Function Call declare function as extern */
extern void getPassValues(uint8_T* pasVals);
extern volatile uint16_T MCHP_ic1up;
extern volatile uint16_T MCHP_ic2up;
extern volatile uint16_T MCHP_ic3up;
extern volatile uint16_T MCHP_ic4up;
extern volatile uint16_T MCHP_ic5up;
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

/* C Function Call declare function as extern */
extern void controlMCUInit();
extern void mul_wide_u32(uint32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern uint32_T mul_u32_u32_u32_sr15(uint32_T a, uint32_T b);
extern void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern uint32_T mul_u32_s32_s32_sat(int32_T a, int32_T b);
extern void mul_wide_su32(int32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern int32_T mul_s32_s32_u32_sr16(int32_T a, uint32_T b);
extern uint32_T div_u32_round(uint32_T numerator, uint32_T denominator);
extern uint32_T mul_u32_u32_u32_sr11_round(uint32_T a, uint32_T b);
extern uint32_T div_repeat_u32_round(uint32_T numerator, uint32_T denominator,
  uint16_T nRepeatSub);
extern uint32_T mul_u32_s32_u32_sr15(int32_T a, uint32_T b);
extern int32_T mul_s32_s32_s32_sr28(int32_T a, int32_T b);
extern int32_T mul_s32_s32_s32_sr23(int32_T a, int32_T b);
extern int16_T div_s16s32_round(int32_T numerator, int32_T denominator);
extern uint32_T MultiWord2uLong(const uint32_T u[]);
extern void uMultiWordShr(const uint32_T u1[], int16_T n1, uint16_T n2, uint32_T
  y[], int16_T n);
extern void uMultiWordMul(const uint32_T u1[], int16_T n1, const uint32_T u2[],
  int16_T n2, uint32_T y[], int16_T n);
extern void AUAV3_WITH_SLUGS_myMuxFun5(uint16_T rtu_u1, uint16_T rtu_u2,
  uint16_T rtu_u3, uint16_T rtu_u4, uint16_T rtu_u5,
  rtB_myMuxFun5_AUAV3_WITH_SLUG_T *localB);
extern void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW);
extern void AUA_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T
  rtu_f, rtB_EmbeddedMATLABFunction_AU_T *localB,
  rtDW_EmbeddedMATLABFunction_A_T *localDW);
extern void AUAV3_WITH_SLUG_negprotect(real32_T rtu_val,
  rtB_negprotect_AUAV3_WITH_SLU_T *localB);
extern void AUAV3_WITH_SL_negprotect_d(real32_T rtu_val,
  rtB_negprotect_AUAV3_WITH_S_h_T *localB);
extern void A_EmbeddedMATLABFunction_g(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_p_T *localB);
extern void AUAV3_WITH_SLUGS_ZerooutZ1(const real32_T rtu_Pin[3],
  rtB_ZerooutZ1_AUAV3_WITH_SLUG_T *localB);
extern void A_EmbeddedMATLABFunction_a(const real32_T rtu_x[3], const real32_T
  rtu_y[3], rtB_EmbeddedMATLABFunction_pk_T *localB);
extern void AUAV3_WITH_SL_SelectNTerms(const real32_T rtu_T[3],
  rtB_SelectNTerms_AUAV3_WITH_S_T *localB);
extern void AUAV3_WITH_SLU_negprotect3(real32_T rtu_val,
  rtB_negprotect3_AUAV3_WITH_SL_T *localB);
extern void AUAV3_WITH_SLUGS_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun1_AUAV3_WITH_SLUG_T *localB);
extern void AUAV3_WITH_S_AxisRotZeroR3(const real32_T rtu_In1[7], real32_T
  *rty_1, real32_T *rty_2, real32_T *rty_3);
extern void AUAV3_WITH__AxisRotDefault(const real32_T rtu_In1[7], real32_T
  *rty_1, real32_T *rty_2, real32_T *rty_3);
extern void A_EmbeddedMATLABFunction_d(real32_T rtu_u,
  rtB_EmbeddedMATLABFunction_pd_T *localB);
extern void A_EmbeddedMATLABFunction_h(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_d_T *localB);
extern void EmbeddedMATLABFunct_o_Init(rtDW_EmbeddedMATLABFunction1__T *localDW);
extern void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW);
extern void AUAV3_WITH_SLU_myMuxFun1_f(real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, rtB_myMuxFun1_AUAV3_WITH_SL_k_T *localB);
extern void AUA_EnabledSubsystem_Start(rtB_EnabledSubsystem_AUAV3_WI_T *localB);
extern void AUAV3_WIT_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_AUAV3_WI_T *localB);
extern void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW);
extern void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW);
extern void AUAV3_ZeroOutHeight_Update(real_T rtu_Enable, real32_T
  rtu_ComputedHeight, rtDW_ZeroOutHeight_AUAV3_WITH_T *localDW);
extern void AUAV3_WITH_S_ZeroOutHeight(real_T rtu_Enable, real32_T
  rtu_BaseHeight, rtB_ZeroOutHeight_AUAV3_WITH__T *localB,
  rtDW_ZeroOutHeight_AUAV3_WITH_T *localDW);
extern void EmbeddedMATLABFunct_i_Init(rtDW_EmbeddedMATLABFunction_f_T *localDW);
extern void A_EmbeddedMATLABFunction_n(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_m_T *localB, rtDW_EmbeddedMATLABFunction_f_T
  *localDW);
extern void AUAV3_WITH_SLUGS__myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun_AUAV3_WITH_SLUGS_T *localB);
extern void A_EmbeddedMATLABFunction_o(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_g_T *localB);
extern void AUAV3_BufferICChannel_Init(rtDW_BufferICChannel_AUAV3_WI_T *localDW);
extern void AUAV3_WITH_BufferICChannel(uint16_T rtu_latest,
  rtB_BufferICChannel_AUAV3_WIT_T *localB, rtDW_BufferICChannel_AUAV3_WI_T
  *localDW);
extern void AUAV3_WITH_SLU_myMuxFun1_c(uint16_T rtu_u1, uint16_T rtu_u2,
  uint16_T rtu_u3, uint16_T rtu_u4, uint16_T rty_y[4]);

#endif                                 /* RTW_HEADER_AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
