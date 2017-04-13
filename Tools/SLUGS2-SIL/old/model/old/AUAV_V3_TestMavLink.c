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
 * File: AUAV_V3_TestMavLink.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestMavLink.
 *
 * Model version                        : 1.7
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Fri May 06 13:10:25 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Fri May 06 13:10:26 2016
 */
#include "libUDB.h"

#ifdef _MSC_VER
#define __attribute__(x)
#define __pack_upper_byte
#define __eds__

#endif

#include "AUAV_V3_TestMavLink.h"
#include "AUAV_V3_TestMavLink_private.h"

/* user code (top of source file) */
#if (WIN != 1)
#include "gpsPort.h"
#endif
#include "MavlinkComm.h"

const mavlink_scaled_pressure_t AUAV_V3_TestMavLink_rtZmavlink_scaled_pressure_t
  = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_gps_raw_int_t AUAV_V3_TestMavLink_rtZmavlink_gps_raw_int_t = {
  0U,                                  /* time_usec */
  0,                                   /* lat */
  0,                                   /* lon */
  0,                                   /* alt */
  0U,                                  /* eph */
  0U,                                  /* epv */
  0U,                                  /* vel */
  0U,                                  /* cog */
  0U,                                  /* fix_type */
  0U                                   /* satellites_visible */
} ;                                    /* mavlink_gps_raw_int_t ground */

const mavlink_raw_pressure_t AUAV_V3_TestMavLink_rtZmavlink_raw_pressure_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_sys_status_t AUAV_V3_TestMavLink_rtZmavlink_sys_status_t = {
  0U,                                  /* onboard_control_sensors_present */
  0U,                                  /* onboard_control_sensors_enabled */
  0U,                                  /* onboard_control_sensors_health */
  0U,                                  /* load */
  0U,                                  /* voltage_battery */
  0,                                   /* current_battery */
  0U,                                  /* drop_rate_comm */
  0U,                                  /* errors_comm */
  0U,                                  /* errors_count1 */
  0U,                                  /* errors_count2 */
  0U,                                  /* errors_count3 */
  0U,                                  /* errors_count4 */
  0U                                   /* battery_remaining */
} ;                                    /* mavlink_sys_status_t ground */

/* Exported block states */
mavlink_gps_raw_int_t mlGpsData;       /* '<Root>/mlGpsData' */
mavlink_sys_status_t mlSysStatus;      /* '<Root>/mlSysStatus' */
mavlink_scaled_pressure_t mlAirData;   /* '<Root>/mlAirData' */
mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
uint16_T MPU_T;                        /* '<Root>/MPU_T' */

/* Block signals (auto storage) */
BlockIO_AUAV_V3_TestMavLink_T AUAV_V3_TestMavLink_B;

/* Block states (auto storage) */
D_Work_AUAV_V3_TestMavLink_T AUAV_V3_TestMavLink_DWork;

/* Real-time model */
RT_MODEL_AUAV_V3_TestMavLink_T AUAV_V3_TestMavLink_M_;
RT_MODEL_AUAV_V3_TestMavLink_T *const AUAV_V3_TestMavLink_M =
  &AUAV_V3_TestMavLink_M_;
void mul_wide_u32(uint32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T outBitsLo;
  uint32_T in0Lo;
  uint32_T in0Hi;
  uint32_T in1Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  in0Hi = in0 >> 16UL;
  in0Lo = in0 & 65535UL;
  in1Hi = in1 >> 16UL;
  in1Lo = in1 & 65535UL;
  productHiLo = in0Hi * in1Lo;
  productLoHi = in0Lo * in1Hi;
  in0Lo *= in1Lo;
  in1Lo = 0UL;
  outBitsLo = (productLoHi << 16UL) + in0Lo;
  if (outBitsLo < in0Lo) {
    in1Lo = 1UL;
  }

  in0Lo = outBitsLo;
  outBitsLo += productHiLo << 16UL;
  if (outBitsLo < in0Lo) {
    in1Lo++;
  }

  *ptrOutBitsHi = (((productLoHi >> 16UL) + (productHiLo >> 16UL)) + in0Hi *
                   in1Hi) + in1Lo;
  *ptrOutBitsLo = outBitsLo;
}

uint32_T mul_u32_u32_u32_sr15(uint32_T a, uint32_T b)
{
  uint32_T result;
  uint32_T u32_chi;
  mul_wide_u32(a, b, &u32_chi, &result);
  return u32_chi << 17UL | result >> 15UL;
}

void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T absIn0;
  uint32_T absIn1;
  uint32_T in0Lo;
  uint32_T in0Hi;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = (uint32_T)(in0 < 0L ? -in0 : in0);
  absIn1 = (uint32_T)(in1 < 0L ? -in1 : in1);
  in0Hi = absIn0 >> 16UL;
  in0Lo = absIn0 & 65535UL;
  in1Hi = absIn1 >> 16UL;
  absIn0 = absIn1 & 65535UL;
  productHiLo = in0Hi * absIn0;
  productLoHi = in0Lo * in1Hi;
  absIn0 *= in0Lo;
  absIn1 = 0UL;
  in0Lo = (productLoHi << 16UL) + absIn0;
  if (in0Lo < absIn0) {
    absIn1 = 1UL;
  }

  absIn0 = in0Lo;
  in0Lo += productHiLo << 16UL;
  if (in0Lo < absIn0) {
    absIn1++;
  }

  absIn0 = (((productLoHi >> 16UL) + (productHiLo >> 16UL)) + in0Hi * in1Hi) +
    absIn1;
  if (!((in0 == 0L) || ((in1 == 0L) || ((in0 > 0L) == (in1 > 0L))))) {
    absIn0 = ~absIn0;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0UL) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = in0Lo;
}

uint32_T mul_u32_s32_s32_sat(int32_T a, int32_T b)
{
  uint32_T result;
  uint32_T u32_chi;
  mul_wide_s32(a, b, &u32_chi, &result);
  if ((int32_T)u32_chi >= 0L) {
    if (u32_chi) {
      result = MAX_uint32_T;
    }
  } else {
    result = 0UL;
  }

  return result;
}

void mul_wide_su32(int32_T in0, uint32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                   *ptrOutBitsLo)
{
  uint32_T outBitsLo;
  uint32_T absIn0;
  uint32_T in0Hi;
  uint32_T in1Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = (uint32_T)(in0 < 0L ? -in0 : in0);
  in0Hi = absIn0 >> 16UL;
  absIn0 &= 65535UL;
  in1Hi = in1 >> 16UL;
  in1Lo = in1 & 65535UL;
  productHiLo = in0Hi * in1Lo;
  productLoHi = absIn0 * in1Hi;
  absIn0 *= in1Lo;
  in1Lo = 0UL;
  outBitsLo = (productLoHi << 16UL) + absIn0;
  if (outBitsLo < absIn0) {
    in1Lo = 1UL;
  }

  absIn0 = outBitsLo;
  outBitsLo += productHiLo << 16UL;
  if (outBitsLo < absIn0) {
    in1Lo++;
  }

  absIn0 = (((productLoHi >> 16UL) + (productHiLo >> 16UL)) + in0Hi * in1Hi) +
    in1Lo;
  if (!((in1 == 0UL) || (in0 >= 0L))) {
    absIn0 = ~absIn0;
    outBitsLo = ~outBitsLo;
    outBitsLo++;
    if (outBitsLo == 0UL) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = outBitsLo;
}

int32_T mul_s32_s32_u32_sr16(int32_T a, uint32_T b)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_su32(a, b, &u32_chi, &u32_clo);
  u32_clo = u32_chi << 16UL | u32_clo >> 16UL;
  return (int32_T)u32_clo;
}

uint32_T div_u32_round(uint32_T numerator, uint32_T denominator)
{
  uint32_T quotient;
  if (denominator == 0UL) {
    quotient = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    quotient = numerator / denominator;
    numerator %= denominator;
    if (numerator > 2147483647UL) {
      quotient++;
    } else {
      numerator <<= 1UL;
      if (numerator >= denominator) {
        quotient++;
      }
    }
  }

  return quotient;
}

uint32_T mul_u32_u32_u32_sr11_round(uint32_T a, uint32_T b)
{
  uint32_T result;
  uint32_T u32_chi;
  mul_wide_u32(a, b, &u32_chi, &result);
  return (u32_chi << 21UL | result >> 11UL) + ((result & 1024UL) != 0UL);
}

uint32_T div_repeat_u32_round(uint32_T numerator, uint32_T denominator, uint16_T
  nRepeatSub)
{
  uint32_T quotient;
  uint16_T iRepeatSub;
  boolean_T numeratorExtraBit;
  if (denominator == 0UL) {
    quotient = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    quotient = numerator / denominator;
    numerator %= denominator;
    for (iRepeatSub = 0U; iRepeatSub < nRepeatSub; iRepeatSub++) {
      numeratorExtraBit = (numerator >= 2147483648UL);
      numerator <<= 1UL;
      quotient <<= 1UL;
      if (numeratorExtraBit || (numerator >= denominator)) {
        quotient++;
        numerator -= denominator;
      }
    }

    numeratorExtraBit = (numerator >= 2147483648UL);
    numerator <<= 1UL;
    if (numeratorExtraBit || (numerator >= denominator)) {
      quotient++;
    }
  }

  return quotient;
}

uint32_T mul_u32_s32_u32_sr15(int32_T a, uint32_T b)
{
  uint32_T result;
  uint32_T u32_chi;
  mul_wide_su32(a, b, &u32_chi, &result);
  return u32_chi << 17UL | result >> 15UL;
}

int32_T mul_s32_s32_s32_sr28(int32_T a, int32_T b)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  u32_clo = u32_chi << 4UL | u32_clo >> 28UL;
  return (int32_T)u32_clo;
}

int32_T mul_s32_s32_s32_sr23(int32_T a, int32_T b)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  u32_clo = u32_chi << 9UL | u32_clo >> 23UL;
  return (int32_T)u32_clo;
}

int16_T div_s16s32_round(int32_T numerator, int32_T denominator)
{
  int16_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  if (denominator == 0L) {
    quotient = numerator >= 0L ? MAX_int16_T : MIN_int16_T;

    /* Divide by zero handler */
  } else {
    absNumerator = (uint32_T)(numerator >= 0L ? numerator : -numerator);
    absDenominator = (uint32_T)(denominator >= 0L ? denominator : -denominator);
    tempAbsQuotient = absNumerator / absDenominator;
    absNumerator %= absDenominator;
    absNumerator <<= 1UL;
    if (absNumerator >= absDenominator) {
      tempAbsQuotient++;
    }

    quotient = (numerator < 0L) != (denominator < 0L) ? (int16_T)-(int32_T)
      tempAbsQuotient : (int16_T)tempAbsQuotient;
  }

  return quotient;
}

uint32_T MultiWord2uLong(const uint32_T u[])
{
  return u[0];
}

void uMultiWordShr(const uint32_T u1[], int16_T n1, uint16_T n2, uint32_T y[],
                   int16_T n)
{
  int16_T nb;
  int16_T i;
  uint32_T yi;
  uint32_T u1i;
  int16_T nc;
  uint16_T nr;
  uint16_T nl;
  int16_T i1;
  nb = (int16_T)(n2 >> 5);
  i = 0;
  if (nb < n1) {
    nc = n + nb;
    if (nc > n1) {
      nc = n1;
    }

    nr = n2 - ((uint16_T)nb << 5);
    if (nr > 0U) {
      nl = 32U - nr;
      u1i = u1[nb];
      for (i1 = nb + 1; i1 < nc; i1++) {
        yi = u1i >> nr;
        u1i = u1[i1];
        y[i] = u1i << nl | yi;
        i++;
      }

      yi = u1i >> nr;
      if (nc < n1) {
        yi |= u1[nc] << nl;
      }

      y[i] = yi;
      i++;
    } else {
      for (i1 = nb; i1 < nc; i1++) {
        y[i] = u1[i1];
        i++;
      }
    }
  }

  while (i < n) {
    y[i] = 0UL;
    i++;
  }
}

void uMultiWordMul(const uint32_T u1[], int16_T n1, const uint32_T u2[], int16_T
                   n2, uint32_T y[], int16_T n)
{
  int16_T i;
  int16_T j;
  int16_T k;
  int16_T nj;
  uint32_T u1i;
  uint32_T yk;
  uint32_T a1;
  uint32_T a0;
  uint32_T b1;
  uint32_T w10;
  uint32_T w01;
  uint32_T cb;

  /* Initialize output to zero */
  for (k = 0; k < n; k++) {
    y[k] = 0UL;
  }

  for (i = 0; i < n1; i++) {
    cb = 0UL;
    u1i = u1[i];
    a1 = u1i >> 16U;
    a0 = u1i & 65535UL;
    k = n - i;
    nj = n2 <= k ? n2 : k;
    k = i;
    for (j = 0; j < nj; j++) {
      yk = y[k];
      u1i = u2[j];
      b1 = u1i >> 16U;
      u1i &= 65535UL;
      w10 = a1 * u1i;
      w01 = a0 * b1;
      yk += cb;
      cb = (uint32_T)(yk < cb);
      u1i *= a0;
      yk += u1i;
      cb += (yk < u1i);
      u1i = w10 << 16U;
      yk += u1i;
      cb += (yk < u1i);
      u1i = w01 << 16U;
      yk += u1i;
      cb += (yk < u1i);
      y[k] = yk;
      cb += w10 >> 16U;
      cb += w01 >> 16U;
      cb += a1 * b1;
      k++;
    }

    if (k < n) {
      y[k] = cb;
    }
  }
}

/* Model step function for TID0 */
void AUAV_V3_TestMavLink_step0(void)   /* Sample time: [0.0025s, 0.0s] */
{
  uint64m_T tmp;
  uint64m_T tmp_0;
  uint32_T tmp_1;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S1>/Output'
   */
  tmp_1 = 3276800000UL;
  uMultiWordMul(&tmp_1, 1, &AUAV_V3_TestMavLink_DWork.Output_DSTATE, 1,
                &tmp_0.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_0.chunks[0U], 2, 17U, &tmp.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV_V3_TestMavLink_DWork.time_since_boot_usec = MultiWord2uLong(&tmp.chunks
    [0U]);

  /* Switch: '<S9>/FixPt Switch' incorporates:
   *  Constant: '<S8>/FixPt Constant'
   *  Sum: '<S8>/FixPt Sum1'
   *  UnitDelay: '<S1>/Output'
   */
  AUAV_V3_TestMavLink_DWork.Output_DSTATE++;
}

/* Model step function for TID1 */
void AUAV_V3_TestMavLink_step1(void)   /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_DiscreteZeroPole;
  uint16_T rtb_BitwiseOperator_i;
  int16_T rtb_DataTypeConversion1_h;
  uint16_T rtb_DataTypeConversion3_m;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_x;
  int16_T rtb_DataTypeConversion2_i;
  uint32_T rtb_Sum6;
  uint32_T qY;

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  AUAV_V3_TestMavLink_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = AUAV_V3_TestMavLink_B.U3CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S5>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* number of I2C blocks : 5 ; Current: 2 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C22_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadTConv[0] = I2C22_Buff8[0];
    AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadTConv[1] = I2C22_Buff8[1];
    MCHP_I2C22_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 4;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C22_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C22_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C22_Request++;

  /* Sum: '<S17>/SumA21' incorporates:
   *  DataTypeConversion: '<S19>/Data Type Conversion1'
   *  DataTypeConversion: '<S19>/Data Type Conversion3'
   *  Delay: '<S17>/Delay11'
   *  Gain: '<S17>/a(2)(1)'
   *  Gain: '<S17>/s(1)'
   *  Gain: '<S19>/Gain'
   *  S-Function (sfix_bitop): '<S19>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadTConv[0] <<
            8 | AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadTConv[1]) *
    31949UL >> 5;
  qY = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV_V3_TestMavLink_DWork.Delay11_DSTATE);
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S17>/SumB21' incorporates:
   *  Delay: '<S17>/Delay11'
   *  Sum: '<S17>/SumA21'
   */
  if (qY > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY << 1;
  }

  if (AUAV_V3_TestMavLink_DWork.Delay11_DSTATE > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV_V3_TestMavLink_DWork.Delay11_DSTATE << 1;
  }

  rtb_Sum6 += rtb_B4;
  if (rtb_Sum6 < rtb_B4) {
    rtb_Sum6 = MAX_uint32_T;
  }

  rtb_BitwiseOperator_i = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S17>/SumB21' */

  /* RateTransition: '<S5>/Rate Transition1' */
  rtb_DataTypeConversion3_m = AUAV_V3_TestMavLink_DWork.RateTransition1_Buffer0;

  /* DataStoreWrite: '<S5>/Update RawPressure' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion7'
   *  DataTypeConversion: '<S5>/Data Type Conversion8'
   */
  mlRawPressureData.press_diff1 = (int16_T)rtb_BitwiseOperator_i;
  mlRawPressureData.press_diff2 = (int16_T)rtb_DataTypeConversion3_m;

  /* Outputs for Enabled SubSystem: '<S5>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  /* Delay: '<S5>/Delay' */
  if (AUAV_V3_TestMavLink_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S16>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
    /* number of I2C blocks : 5 ; Current: 1 ; MCHP_I2C_StartImplemented =  5*/
    {
      /* Enable I2C sequence */
      MCHP_I2C21_Request ++;
      MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 20;
      if (MCHP_I2C2_Queue.head >= 5)   /* There are 5 blocks I2C2, max idx for queue is 5 */
        MCHP_I2C2_Queue.head = 0;
      else
        MCHP_I2C2_Queue.head ++;
      if (MCHP_I2C2_State == 0)
        _MI2C2IF = 1;

      /* Wait for end of SPI sequence (handle within an interrupt) */
      while (MCHP_I2C21_Request != 0) ;/* Wait until end of SPI sequence. */
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[0] = I2C21_Buff8[0];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[1] = I2C21_Buff8[1];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[2] = I2C21_Buff8[2];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[3] = I2C21_Buff8[3];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[4] = I2C21_Buff8[4];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[5] = I2C21_Buff8[5];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[6] = I2C21_Buff8[6];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[7] = I2C21_Buff8[7];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[8] = I2C21_Buff8[8];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[9] = I2C21_Buff8[9];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[10] = I2C21_Buff8[10];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[11] = I2C21_Buff8[11];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[12] = I2C21_Buff8[12];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[13] = I2C21_Buff8[13];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[14] = I2C21_Buff8[14];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[15] = I2C21_Buff8[15];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[16] = I2C21_Buff8[16];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[17] = I2C21_Buff8[17];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[18] = I2C21_Buff8[18];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[19] = I2C21_Buff8[19];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[20] = I2C21_Buff8[20];
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[21] = I2C21_Buff8[21];
    }

    /* DataTypeConversion: '<S16>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S33>/Data Type Conversion1'
     *  DataTypeConversion: '<S33>/Data Type Conversion3'
     *  Gain: '<S33>/Gain'
     *  S-Function (sfix_bitop): '<S33>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition3 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S16>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S32>/Data Type Conversion1'
     *  DataTypeConversion: '<S32>/Data Type Conversion3'
     *  Gain: '<S32>/Gain'
     *  S-Function (sfix_bitop): '<S32>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S16>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S34>/Data Type Conversion1'
     *  DataTypeConversion: '<S34>/Data Type Conversion3'
     *  Gain: '<S34>/Gain'
     *  S-Function (sfix_bitop): '<S34>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S16>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S38>/Data Type Conversion1'
     *  DataTypeConversion: '<S38>/Data Type Conversion3'
     *  Gain: '<S38>/Gain'
     *  S-Function (sfix_bitop): '<S38>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S16>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S39>/Data Type Conversion1'
     *  DataTypeConversion: '<S39>/Data Type Conversion3'
     *  Gain: '<S39>/Gain'
     *  S-Function (sfix_bitop): '<S39>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S16>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S30>/Data Type Conversion1'
     *  DataTypeConversion: '<S30>/Data Type Conversion3'
     *  Gain: '<S30>/Gain'
     *  S-Function (sfix_bitop): '<S30>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S16>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S31>/Data Type Conversion1'
     *  DataTypeConversion: '<S31>/Data Type Conversion3'
     *  Gain: '<S31>/Gain'
     *  S-Function (sfix_bitop): '<S31>/Bitwise Operator'
     */
    AUAV_V3_TestMavLink_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S35>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S35>/Data Type Conversion1'
     *  DataTypeConversion: '<S35>/Data Type Conversion3'
     *  Gain: '<S35>/Gain'
     */
    AUAV_V3_TestMavLink_B.RateTransition5 = (uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S36>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S36>/Data Type Conversion1'
     *  DataTypeConversion: '<S36>/Data Type Conversion3'
     *  Gain: '<S36>/Gain'
     */
    AUAV_V3_TestMavLink_B.RateTransition6 = (uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S37>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S37>/Data Type Conversion1'
     *  DataTypeConversion: '<S37>/Data Type Conversion3'
     *  Gain: '<S37>/Gain'
     */
    AUAV_V3_TestMavLink_B.RateTransition7 = (uint16_T)
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S5>/Delay' */
  /* End of Outputs for SubSystem: '<S5>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S14>/Sum' */
  rtb_BitwiseOperator_i -= AUAV_V3_TestMavLink_B.RateTransition7;

  /* Product: '<S14>/Product' */
  rtb_Sum6 = (uint32_T)rtb_BitwiseOperator_i *
    AUAV_V3_TestMavLink_B.RateTransition6;
  rtb_BitwiseOperator_i = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) !=
    0U) + (rtb_Sum6 >> 15));

  /* Sum: '<S14>/Sum2' incorporates:
   *  Bias: '<S14>/Bias'
   *  Bias: '<S14>/Bias1'
   *  Product: '<S14>/Product1'
   *  Sum: '<S14>/Sum1'
   */
  rtb_DataTypeConversion1_h = (div_s16s32_round((int32_T)
    AUAV_V3_TestMavLink_B.RateTransition11 << 11, rtb_BitwiseOperator_i +
    AUAV_V3_TestMavLink_B.RateTransition12) + (int16_T)rtb_BitwiseOperator_i) -
    4000;

  /* MATLAB Function: '<S15>/Enables//Disables the Computation of  initial Baro Bias' */
  /* MATLAB Function 'Read Barometer BMP180/Baro Altimeter/Enables/Disables the Computation of  initial Baro Bias': '<S24>:1' */
  if (AUAV_V3_TestMavLink_DWork.aveCount < 2500.0) {
    /* '<S24>:1:11' */
    /* '<S24>:1:12' */
    AUAV_V3_TestMavLink_DWork.aveCount++;
  }

  if (AUAV_V3_TestMavLink_DWork.aveCount == 2500.0) {
    /* '<S24>:1:15' */
    /* '<S24>:1:16' */
    AUAV_V3_TestMavLink_DWork.tIni = 0.0;

    /* '<S24>:1:17' */
    AUAV_V3_TestMavLink_DWork.aveCount++;
  }

  /* Outputs for Enabled SubSystem: '<S15>/Zero Out Height' incorporates:
   *  EnablePort: '<S26>/Enable'
   */
  /* '<S24>:1:20' */
  /* if aveCount > 500 */
  /*     tIni = 0; */
  /* end */
  /*  if isempty(aveCount) */
  /*      aveCount =1; */
  /*      initBias = [0.0 0.0 0.0]'; */
  /*  end; */
  /*       */
  /*  if (aveCount<10) */
  /*      initBias = initBias+ Raw; */
  /*      bias = initBias/aveCount; */
  /*      aveCount = aveCount +1; */
  /*  end; */
  /*   */
  /*  if (aveCount == 10) */
  /*      initBias = initBias/(aveCount-1); */
  /*      aveCount = 100; */
  /*  end; */
  /*   */
  /*  if (aveCount == 100) */
  /*      bias = initBias; */
  /*  end; */
  if (AUAV_V3_TestMavLink_DWork.tIni > 0.0) {
    /* Sum: '<S26>/Sum' incorporates:
     *  Constant: '<S15>/Constant5'
     *  Delay: '<S26>/Integer Delay'
     */
    AUAV_V3_TestMavLink_B.Sum = 0.0F -
      AUAV_V3_TestMavLink_DWork.IntegerDelay_DSTATE;
  }

  /* End of Outputs for SubSystem: '<S15>/Zero Out Height' */

  /* Math: '<S14>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_DataTypeConversion1_h * rtb_DataTypeConversion1_h;

  /* Sum: '<S14>/Sum6' incorporates:
   *  Bias: '<S14>/Bias2'
   *  Product: '<S14>/Product2'
   *  Product: '<S14>/Product3'
   *  Sum: '<S14>/Sum3'
   *  Sum: '<S14>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_DataTypeConversion3_m - (((((int16_T)((int32_T)
    rtb_DataTypeConversion1_h * AUAV_V3_TestMavLink_B.RateTransition3 >> 11) +
    (int16_T)mul_s32_s32_s32_sr23(AUAV_V3_TestMavLink_B.RateTransition9,
    rtb_Bias5)) + ((int32_T)AUAV_V3_TestMavLink_B.RateTransition2 << 2)) + 2L) >>
    2);

  /* Product: '<S14>/Product6' incorporates:
   *  Bias: '<S14>/Bias3'
   *  Bias: '<S14>/Bias4'
   *  Gain: '<S14>/Gain1'
   *  Product: '<S14>/Product4'
   *  Product: '<S14>/Product5'
   *  Sum: '<S14>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15(((((int16_T)((int32_T)rtb_DataTypeConversion1_h *
    AUAV_V3_TestMavLink_B.RateTransition4 >> 13) + (int16_T)mul_s32_s32_s32_sr28
    (rtb_Bias5, AUAV_V3_TestMavLink_B.RateTransition8)) + 2) >> 2) + 32768L,
    AUAV_V3_TestMavLink_B.RateTransition5);

  /* Switch: '<S14>/Switch' incorporates:
   *  Gain: '<S14>/Gain15'
   *  Gain: '<S14>/Gain22'
   *  Product: '<S14>/Product7'
   *  Product: '<S14>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S14>/Switch' */

  /* Gain: '<S14>/Gain16' */
  rtb_Bias5 = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >>
    8));

  /* Sum: '<S14>/Sum8' incorporates:
   *  Bias: '<S14>/Bias5'
   *  Gain: '<S14>/Gain17'
   *  Gain: '<S14>/Gain19'
   *  Gain: '<S14>/Gain21'
   *  Math: '<S14>/Math Function2'
   *  Sum: '<S14>/Sum7'
   */
  rtb_Bias5 = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat
    (rtb_Bias5, rtb_Bias5)) + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >>
               4) + (int32_T)rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S15>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S25>/Enable'
   */
  /* MATLAB Function: '<S15>/Enables//Disables the Computation of  initial Baro Bias' */
  if (AUAV_V3_TestMavLink_DWork.tIni > 0.0) {
    /* DataTypeConversion: '<S25>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Bias5;

    /* DiscreteZeroPole: '<S28>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole += 0.029119852459414206*
        AUAV_V3_TestMavLink_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S25>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S25>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole > 120000.0F) {
      AUAV_V3_TestMavLink_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole < 80000.0F) {
      AUAV_V3_TestMavLink_B.u0k120k = 80000.0F;
    } else {
      AUAV_V3_TestMavLink_B.u0k120k = (real32_T)rtb_DiscreteZeroPole;
    }

    /* End of Saturate: '<S25>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S28>/Discrete Zero-Pole' */
    {
      AUAV_V3_TestMavLink_DWork.DiscreteZeroPole_DSTATE = rtb_DataTypeConversion
        + 0.97044334975369462*AUAV_V3_TestMavLink_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S15>/Initial Baro Bias' */

  /* Product: '<S22>/Divide' incorporates:
   *  Sum: '<S22>/Sum2'
   */
  rtb_x = ((real32_T)rtb_Bias5 - AUAV_V3_TestMavLink_B.u0k120k) /
    AUAV_V3_TestMavLink_B.u0k120k;

  /* Sum: '<S22>/Sum1' incorporates:
   *  Constant: '<S22>/Constant2'
   *  Constant: '<S22>/Constant3'
   *  Constant: '<S22>/Constant4'
   *  Constant: '<S22>/Constant5'
   *  Gain: '<S27>/Unit Conversion'
   *  Product: '<S22>/Divide1'
   *  Product: '<S22>/Divide2'
   *  Product: '<S22>/Divide3'
   *  Product: '<S22>/Divide4'
   *  Sum: '<S22>/Sum3'
   */
  rtb_x = ((rtb_x * rtb_x * 0.093502529F + rtb_x * -0.188893303F) +
           2.18031291E-5F) * 145473.5F * 0.3048F;

  /* Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S23>/Enable'
   */
  /* Logic: '<S15>/Logical Operator' incorporates:
   *  Inport: '<S23>/In1'
   *  MATLAB Function: '<S15>/Enables//Disables the Computation of  initial Baro Bias'
   *  Sum: '<S15>/Sum1'
   */
  if (!(AUAV_V3_TestMavLink_DWork.tIni != 0.0)) {
    AUAV_V3_TestMavLink_B.In1 = AUAV_V3_TestMavLink_B.Sum + rtb_x;
  }

  /* End of Logic: '<S15>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S15>/Enabled Subsystem' */

  /* DataStoreWrite: '<S5>/Update ScaledPressure' incorporates:
   *  Bias: '<S14>/Bias'
   *  Bias: '<S14>/Bias1'
   *  Gain: '<S5>/Gain21'
   */
  mlAirData.temperature = (int16_T)((rtb_DataTypeConversion1_h + 4008) * 5L >> 3);
  mlAirData.press_abs = AUAV_V3_TestMavLink_B.In1;

  /* S-Function (MCHP_BUS_SPI): '<S6>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  /* number of SPI blocks : 2 ; Current: 1 ; MCHP_SPI_StartImplemented =  2*/
  if (MCHP_SPI11_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestMavLink_B.U1CH8[0] = SPI11_Buff16[0];
    AUAV_V3_TestMavLink_B.U1CH8[1] = SPI11_Buff16[1];
    AUAV_V3_TestMavLink_B.U1CH8[2] = SPI11_Buff16[2];
    AUAV_V3_TestMavLink_B.U1CH4 = SPI11_Buff16[3];
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0] = SPI11_Buff16[4];
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1] = SPI11_Buff16[5];
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2] = SPI11_Buff16[6];
    MCHP_SPI11_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 8;
    if (MCHP_SPI1_Queue.head >= 2)     /* There are 2 blocks SPI1, max idx for queue is 2 */
      MCHP_SPI1_Queue.head = 0;
    else
      MCHP_SPI1_Queue.head ++;
    if (MCHP_SPI1_State == 0)
      _SPI1IF = 1;                     /* Force Interrupt */
  }

  /* DataStoreWrite: '<S6>/Update MPU_T' */
  MPU_T = AUAV_V3_TestMavLink_B.U1CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S6>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  /* number of I2C blocks : 5 ; Current: 4 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C24_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[0] = I2C24_Buff8[0];
    AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[1] = I2C24_Buff8[1];
    AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[2] = I2C24_Buff8[2];
    AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[3] = I2C24_Buff8[3];
    AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[4] = I2C24_Buff8[4];
    AUAV_V3_TestMavLink_B.BUSI2CRe7adHMC5883Magn50Hz[5] = I2C24_Buff8[5];
    MCHP_I2C24_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 71;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C24_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C24_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C24_Request++;

  /* DataTypeConversion: '<S41>/Data Type Conversion3' */
  rtb_DataTypeConversion3_m = AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[1];

  /* DataTypeConversion: '<S41>/Data Type Conversion2' incorporates:
   *  DataTypeConversion: '<S41>/Data Type Conversion1'
   *  S-Function (sfix_bitop): '<S41>/Bitwise Operator'
   */
  rtb_DataTypeConversion1_h = (int16_T)
    (AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[0] |
     rtb_DataTypeConversion3_m);

  /* DataTypeConversion: '<S42>/Data Type Conversion3' */
  rtb_DataTypeConversion3_m = AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[3];

  /* DataTypeConversion: '<S42>/Data Type Conversion2' incorporates:
   *  DataTypeConversion: '<S42>/Data Type Conversion1'
   *  S-Function (sfix_bitop): '<S42>/Bitwise Operator'
   */
  rtb_DataTypeConversion2_i = (int16_T)
    (AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[2] |
     rtb_DataTypeConversion3_m);

  /* DataTypeConversion: '<S40>/Data Type Conversion3' */
  rtb_DataTypeConversion3_m = AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[5];

  /* DataStoreWrite: '<S6>/Update Raw IMU DATA' incorporates:
   *  DataTypeConversion: '<S40>/Data Type Conversion1'
   *  DataTypeConversion: '<S40>/Data Type Conversion2'
   *  DataTypeConversion: '<S6>/Data Type Conversion'
   *  DataTypeConversion: '<S6>/Data Type Conversion1'
   *  DataTypeConversion: '<S6>/Data Type Conversion2'
   *  DataTypeConversion: '<S6>/Data Type Conversion3'
   *  DataTypeConversion: '<S6>/Data Type Conversion4'
   *  DataTypeConversion: '<S6>/Data Type Conversion5'
   *  S-Function (sfix_bitop): '<S40>/Bitwise Operator'
   */
  AUAV_V3_TestMavLink_DWork.mlRawIMU.xacc = (int16_T)
    AUAV_V3_TestMavLink_B.U1CH8[0];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.yacc = (int16_T)
    AUAV_V3_TestMavLink_B.U1CH8[1];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.zacc = (int16_T)
    AUAV_V3_TestMavLink_B.U1CH8[2];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.xgyro = (int16_T)
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.ygyro = (int16_T)
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.zgyro = (int16_T)
    AUAV_V3_TestMavLink_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2];
  AUAV_V3_TestMavLink_DWork.mlRawIMU.xmag = rtb_DataTypeConversion1_h;
  AUAV_V3_TestMavLink_DWork.mlRawIMU.ymag = rtb_DataTypeConversion2_i;
  AUAV_V3_TestMavLink_DWork.mlRawIMU.zmag = (int16_T)
    (AUAV_V3_TestMavLink_B.BUSI2CReadHMC5883Magn50Hz[4] |
     rtb_DataTypeConversion3_m);

  /* Update for Delay: '<S17>/Delay11' incorporates:
   *  Sum: '<S17>/SumA21'
   */
  AUAV_V3_TestMavLink_DWork.Delay11_DSTATE = qY;

  /* Update for Delay: '<S5>/Delay' incorporates:
   *  Constant: '<S5>/Constant'
   */
  AUAV_V3_TestMavLink_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S15>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S26>/Enable'
   */
  /* MATLAB Function: '<S15>/Enables//Disables the Computation of  initial Baro Bias' */
  if (AUAV_V3_TestMavLink_DWork.tIni > 0.0) {
    /* Update for Delay: '<S26>/Integer Delay' */
    AUAV_V3_TestMavLink_DWork.IntegerDelay_DSTATE = rtb_x;
  }

  /* End of Update for SubSystem: '<S15>/Zero Out Height' */
}

/* Model step function for TID2 */
void AUAV_V3_TestMavLink_step2(void)   /* Sample time: [0.01s, 0.005s] */
{
  uint32_T qY;
  uint32_T q0;
  uint32_T qY_0;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S5>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  /* number of I2C blocks : 5 ; Current: 3 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C23_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadPConv[0] = I2C23_Buff8[0];
    AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadPConv[1] = I2C23_Buff8[1];
    MCHP_I2C23_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 91;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C23_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C23_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C23_Request++;

  /* Sum: '<S18>/SumA21' incorporates:
   *  DataTypeConversion: '<S20>/Data Type Conversion1'
   *  DataTypeConversion: '<S20>/Data Type Conversion3'
   *  Delay: '<S18>/Delay11'
   *  Gain: '<S18>/a(2)(1)'
   *  Gain: '<S18>/s(1)'
   *  Gain: '<S20>/Gain'
   *  S-Function (sfix_bitop): '<S20>/Bitwise Operator'
   */
  q0 = ((uint16_T)AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadPConv[0] << 8 |
        AUAV_V3_TestMavLink_B.BUSI2CInitializeBMP180ReadPConv[1]) * 31949UL >> 5;
  qY_0 = q0 + mul_u32_u32_u32_sr15(30771UL,
    AUAV_V3_TestMavLink_DWork.Delay11_DSTATE_j);
  if (qY_0 < q0) {
    qY_0 = MAX_uint32_T;
  }

  /* Sum: '<S18>/SumB21' incorporates:
   *  Delay: '<S18>/Delay11'
   *  Sum: '<S18>/SumA21'
   */
  if (qY_0 > 2147483647UL) {
    q0 = MAX_uint32_T;
  } else {
    q0 = qY_0 << 1;
  }

  if (AUAV_V3_TestMavLink_DWork.Delay11_DSTATE_j > 2147483647UL) {
    qY = MAX_uint32_T;
  } else {
    qY = AUAV_V3_TestMavLink_DWork.Delay11_DSTATE_j << 1;
  }

  qY += q0;
  if (qY < q0) {
    qY = MAX_uint32_T;
  }

  AUAV_V3_TestMavLink_DWork.RateTransition1_Buffer0 = (uint16_T)(qY >> 16);

  /* End of Sum: '<S18>/SumB21' */

  /* Update for Delay: '<S18>/Delay11' incorporates:
   *  Sum: '<S18>/SumA21'
   */
  AUAV_V3_TestMavLink_DWork.Delay11_DSTATE_j = qY_0;
}

/* Model step function for TID3 */
void AUAV_V3_TestMavLink_step3(void)   /* Sample time: [0.02s, 0.0s] */
{
  int16_T rtb_Bias_g;
  int16_T rtb_Bias1;
  int32_T tmp;

  /* S-Function "dsPIC_PWM_IC" Block: <S7>/Input Capture RC Receiver1 */
  AUAV_V3_TestMavLink_B.cmdPITCHraw = MCHP_ic1up;
  AUAV_V3_TestMavLink_B.InputCaptureRCReceiver1_o2 = MCHP_ic2up;
  AUAV_V3_TestMavLink_B.InputCaptureRCReceiver1_o3 = MCHP_ic3up;
  AUAV_V3_TestMavLink_B.cmdROLLraw = MCHP_ic4up;
  AUAV_V3_TestMavLink_B.InputCaptureRCReceiver1_o5 = MCHP_ic5up;
  AUAV_V3_TestMavLink_B.InputCaptureRCReceiver1_o6 = MCHP_ic6up;

  /* Bias: '<S7>/Bias' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion4'
   */
  rtb_Bias_g = (int16_T)AUAV_V3_TestMavLink_B.cmdPITCHraw - 13125;

  /* Bias: '<S7>/Bias1' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion5'
   */
  rtb_Bias1 = (int16_T)AUAV_V3_TestMavLink_B.cmdROLLraw - 13125;

  /* Sum: '<S43>/Sum' */
  tmp = (int32_T)rtb_Bias1 - rtb_Bias_g;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S7>/Data Type Conversion2' incorporates:
   *  Bias: '<S7>/Bias2'
   *  Sum: '<S43>/Sum'
   */
  AUAV_V3_TestMavLink_B.Saturation = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S7>/Saturation' */
  if (AUAV_V3_TestMavLink_B.Saturation > 19249U) {
    /* DataTypeConversion: '<S7>/Data Type Conversion2' */
    AUAV_V3_TestMavLink_B.Saturation = 19249U;
  } else {
    if (AUAV_V3_TestMavLink_B.Saturation < 7000U) {
      /* DataTypeConversion: '<S7>/Data Type Conversion2' */
      AUAV_V3_TestMavLink_B.Saturation = 7000U;
    }
  }

  /* End of Saturate: '<S7>/Saturation' */

  /* Sum: '<S43>/Sum1' */
  tmp = (int32_T)rtb_Bias_g + rtb_Bias1;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S7>/Data Type Conversion3' incorporates:
   *  Bias: '<S7>/Bias3'
   *  Sum: '<S43>/Sum1'
   */
  AUAV_V3_TestMavLink_B.Saturation1 = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S7>/Saturation1' */
  if (AUAV_V3_TestMavLink_B.Saturation1 > 19249U) {
    /* DataTypeConversion: '<S7>/Data Type Conversion3' */
    AUAV_V3_TestMavLink_B.Saturation1 = 19249U;
  } else {
    if (AUAV_V3_TestMavLink_B.Saturation1 < 7000U) {
      /* DataTypeConversion: '<S7>/Data Type Conversion3' */
      AUAV_V3_TestMavLink_B.Saturation1 = 7000U;
    }
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* DataStoreRead: '<S3>/Get Raw IMU' */
  AUAV_V3_TestMavLink_B.GetRawIMU = AUAV_V3_TestMavLink_DWork.mlRawIMU;

  /* DataStoreRead: '<S3>/Get time' */
  AUAV_V3_TestMavLink_B.Gettime = AUAV_V3_TestMavLink_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S3>/PackRawIMU' */
  AUAV_V3_TestMavLink_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestMavLink_B.GetRawIMU
    , AUAV_V3_TestMavLink_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.PackRawIMU
                  );

  /* S-Function "dsPIC_PWM_OC" Block: <S7>/Output Compare - HW Drive Servo motor */
  OC1CON1 = 0x1008;                    /* Disable OC1 */
  OC1CON2bits.TRIGSTAT = 0;
  OC1RS = AUAV_V3_TestMavLink_B.Saturation;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC2CON1 = 0x1008;                    /* Disable OC2 */
  OC2CON2bits.TRIGSTAT = 0;
  OC2RS = AUAV_V3_TestMavLink_B.Saturation1;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC1CON2bits.TRIGSTAT = 1;
  OC1CON1 = 0x100C;                    /* Trig OC1 pulse */
  OC2CON2bits.TRIGSTAT = 1;
  OC2CON1 = 0x100C;                    /* Trig OC2 pulse */
}

/* Model step function for TID4 */
void AUAV_V3_TestMavLink_step4(void)   /* Sample time: [0.05s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator_h;

  /* S-Function (MCHP_Digital_Output_Read): '<S10>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S10>/Digital Output Read/Output */

  /* Logic: '<S2>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S10>/Digital Output Read'
   */
  rtb_LogicalOperator_h = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S10>/Digital Output Write' */
  LATBbits.LATB2 = rtb_LogicalOperator_h;
}

/* Model step function for TID5 */
void AUAV_V3_TestMavLink_step5(void)   /* Sample time: [0.1s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator1;

  /* S-Function (MCHP_Digital_Output_Read): '<S11>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S11>/Digital Output Read/Output */

  /* Logic: '<S2>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S11>/Digital Output Read'
   */
  rtb_LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S11>/Digital Output Write' */
  LATBbits.LATB3 = rtb_LogicalOperator1;
}

/* Model step function for TID6 */
void AUAV_V3_TestMavLink_step6(void)   /* Sample time: [0.2s, 0.0s] */
{
  /* DataStoreRead: '<S3>/Get RawGpsInt' */
  AUAV_V3_TestMavLink_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S3>/Get time1' */
  AUAV_V3_TestMavLink_B.Gettime1 =
    AUAV_V3_TestMavLink_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S3>/PackGpsRawInt' */
  AUAV_V3_TestMavLink_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestMavLink_B.GetRawGpsInt
    , AUAV_V3_TestMavLink_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data2' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.PackGpsRawInt
                  );

  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );

  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID7 */
void AUAV_V3_TestMavLink_step7(void)   /* Sample time: [0.2s, 0.02s] */
{
  /* DataStoreRead: '<S3>/Get mlAirData' */
  AUAV_V3_TestMavLink_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S3>/Get time2' */
  AUAV_V3_TestMavLink_B.Gettime2 =
    AUAV_V3_TestMavLink_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S3>/PackScaledPressure' */
  AUAV_V3_TestMavLink_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestMavLink_B.GetmlAirData
    , AUAV_V3_TestMavLink_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.PackScaledPressure
                  );
}

/* Model step function for TID8 */
void AUAV_V3_TestMavLink_step8(void)   /* Sample time: [0.2s, 0.04s] */
{
  /* DataStoreRead: '<S3>/Get mlSysStatus' */
  AUAV_V3_TestMavLink_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S3>/PackSysStatus' */
  AUAV_V3_TestMavLink_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV_V3_TestMavLink_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.PackSysStatus
                  );
}

/* Model step function for TID9 */
void AUAV_V3_TestMavLink_step9(void)   /* Sample time: [0.2s, 0.06s] */
{
  /* S-Function (MCHP_C_function_Call): '<S3>/ParamInterfaceResponse' */
  AUAV_V3_TestMavLink_B.ParamInterfaceResponse = ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data5' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.ParamInterfaceResponse
                  );
}

/* Model step function for TID10 */
void AUAV_V3_TestMavLink_step10(void)  /* Sample time: [0.2s, 0.08s] */
{
  /* S-Function (MCHP_C_function_Call): '<S3>/ParamInterfaceResponse1' */
  AUAV_V3_TestMavLink_B.ParamInterfaceResponse1 = MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data6' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.ParamInterfaceResponse1
                  );
}

/* Model step function for TID11 */
void AUAV_V3_TestMavLink_step11(void)  /* Sample time: [0.25s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator2;

  /* S-Function (MCHP_Digital_Output_Read): '<S12>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S12>/Digital Output Read/Output */

  /* Logic: '<S2>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S12>/Digital Output Read'
   */
  rtb_LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S12>/Digital Output Write' */
  LATBbits.LATB4 = rtb_LogicalOperator2;
}

/* Model step function for TID12 */
void AUAV_V3_TestMavLink_step12(void)  /* Sample time: [0.5s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator3;

  /* S-Function (MCHP_Digital_Output_Read): '<S13>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S13>/Digital Output Read/Output */

  /* Logic: '<S2>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S13>/Digital Output Read'
   */
  rtb_LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S13>/Digital Output Write' */
  LATBbits.LATB5 = rtb_LogicalOperator3;

  /* S-Function (MCHP_C_function_Call): '<S3>/PackHeartBeat' */
  AUAV_V3_TestMavLink_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S3>/TX_N_Data1' */
  TxN_Data_OverU1(
                  AUAV_V3_TestMavLink_B.PackHeartBeat
                  );

  /* S-Function (MCHP_BUS_I2C_MASTER): '<Root>/BUS I2C Initialize HMC5883 re-initialize at 0.5Hz' */
  /* number of I2C blocks : 5 ; Current: 5 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C25_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    MCHP_I2C25_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 107;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C25_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C25_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C25_Request++;
}

/* Model step wrapper function for compatibility with a static main program */
void AUAV_V3_TestMavLink_step(int_T tid)
{
  switch (tid) {
   case 0 :
    AUAV_V3_TestMavLink_step0();
    break;

   case 1 :
    AUAV_V3_TestMavLink_step1();
    break;

   case 2 :
    AUAV_V3_TestMavLink_step2();
    break;

   case 3 :
    AUAV_V3_TestMavLink_step3();
    break;

   case 4 :
    AUAV_V3_TestMavLink_step4();
    break;

   case 5 :
    AUAV_V3_TestMavLink_step5();
    break;

   case 6 :
    AUAV_V3_TestMavLink_step6();
    break;

   case 7 :
    AUAV_V3_TestMavLink_step7();
    break;

   case 8 :
    AUAV_V3_TestMavLink_step8();
    break;

   case 9 :
    AUAV_V3_TestMavLink_step9();
    break;

   case 10 :
    AUAV_V3_TestMavLink_step10();
    break;

   case 11 :
    AUAV_V3_TestMavLink_step11();
    break;

   case 12 :
    AUAV_V3_TestMavLink_step12();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void AUAV_V3_TestMavLink_initialize(void)
{
  /* Registration code */

  /* initialize sample time offsets */
  AUAV_V3_TestMavLink_M->Timing.TaskCounters.TID[2] = 2;/* Sample time: [0.01s, 0.005s] */

  /* initialize sample time offsets */
  AUAV_V3_TestMavLink_M->Timing.TaskCounters.TID[7] = 72;/* Sample time: [0.2s, 0.02s] */

  /* initialize sample time offsets */
  AUAV_V3_TestMavLink_M->Timing.TaskCounters.TID[8] = 64;/* Sample time: [0.2s, 0.04s] */

  /* initialize sample time offsets */
  AUAV_V3_TestMavLink_M->Timing.TaskCounters.TID[9] = 56;/* Sample time: [0.2s, 0.06s] */

  /* initialize sample time offsets */
  AUAV_V3_TestMavLink_M->Timing.TaskCounters.TID[10] = 48;/* Sample time: [0.2s, 0.08s] */

  /* block I/O */
  (void) memset(((void *) &AUAV_V3_TestMavLink_B), 0,
                sizeof(BlockIO_AUAV_V3_TestMavLink_T));

  /* states (dwork) */
  (void) memset((void *)&AUAV_V3_TestMavLink_DWork, 0,
                sizeof(D_Work_AUAV_V3_TestMavLink_T));

  /* exported global states */
  mlGpsData = AUAV_V3_TestMavLink_rtZmavlink_gps_raw_int_t;
  mlSysStatus = AUAV_V3_TestMavLink_rtZmavlink_sys_status_t;
  mlAirData = AUAV_V3_TestMavLink_rtZmavlink_scaled_pressure_t;
  mlRawPressureData = AUAV_V3_TestMavLink_rtZmavlink_raw_pressure_t;
  MPU_T = 0U;

  /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

  /* Start for S-Function (MCHP_MCU_LOAD): '<Root>/MCU Load' */
  TMR2 = 0;                            /* Initialize Timer 2 Value to 0.  Timer 2 is enabled only when the mcu is not idle */

  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S5>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0xA5;                      /* I2C clock = 399772  (400000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;

  /* Start for S-Function (MCHP_BUS_SPI): '<S6>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
  SPI1CON1 = 0x0474;
  SPI1CON2 = 0x01;
  SPI1STAT = 0x8014;
  _SPI1IP = 5;                         /* Set SPI Interrupt Priority */
  _SPI1IF = 0;                         /* Reset interrupt Flag */
  _SPI1IE = 1;                         /* Enable Interrupts */

  /* Start for S-Function (MCHP_IC): '<S7>/Input Capture RC Receiver1' */
  IC1CON1 = 0x1003;
  IC1CON2 = 0x00;
  IC2CON1 = 0x1003;
  IC2CON2 = 0x00;
  IC3CON1 = 0x1003;
  IC3CON2 = 0x00;
  IC4CON1 = 0x1003;
  IC4CON2 = 0x00;
  IC5CON1 = 0x1003;
  IC5CON2 = 0x00;
  IC6CON1 = 0x1003;
  IC6CON2 = 0x00;

  /* Set-up Input Capture Interruption */
  _IC1IF = 0;
  _IC1IP = 7;
  _IC1IE = 1;
  _IC2IF = 0;
  _IC2IP = 7;
  _IC2IE = 1;
  _IC3IF = 0;
  _IC3IP = 7;
  _IC3IE = 1;
  _IC4IF = 0;
  _IC4IP = 7;
  _IC4IE = 1;
  _IC5IF = 0;
  _IC5IP = 7;
  _IC5IE = 1;
  _IC6IF = 0;
  _IC6IP = 7;
  _IC6IE = 1;

  /* Start for S-Function (MCHP_OC_HW): '<S7>/Output Compare - HW Drive Servo motor' */
  /* OCxCON1[4108, 4108]   PulseTrig1*/
  OC1CON2 = 0x9F;
  OC1RS = 0x6689;
  OC1R = 1;
  OC2CON2 = 0x9F;
  OC2RS = 0x6689;
  OC2R = 1;

  /* Start for S-Function (MCHP_BUS_SPI): '<Root>/BUS SPI Initialize MPU 6000 Once at Startup' */
  /* SPI Initialisation sequence executed once */
  /* number of SPI blocks : 2 ; Current: 2 ; MCHP_SPI_StartImplemented =  2*/
  if (MCHP_SPI12_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    MCHP_SPI12_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 1;
    if (MCHP_SPI1_Queue.head >= 2)     /* There are 2 blocks SPI1, max idx for queue is 2 */
      MCHP_SPI1_Queue.head = 0;
    else
      MCHP_SPI1_Queue.head ++;
    if (MCHP_SPI1_State == 0)
      _SPI1IF = 1;                     /* Force Interrupt */
  }

  /* MCHP_UART_Config Block for UART 1: <Root>/UART Configuration UAV V3 UART 3/Initialize */
  /* Initialisation sequence for UART 1 */
  {
    const uint8_T InitSequence[11] = { 72, 101, 108, 108, 111, 32, 87, 111, 114,
      108, 100 };

    U1BRG = 0x9B5C;                    /* Baud rate: 110 (-0.00%) */
    U1MODE = 0x8000;
    U1STA = 0x2400;
    __delay32(1909091);                /* Wait for 1909091 cycles */

    {
      uint_T i1;
      for (i1 = 0; i1 < 11 ; i1++) {
        while (U1STAbits.UTXBF == 1) ; /* Wait for one empty space within buffer UART */
        U1TXREG = InitSequence[i1];
      }
    }

    while (U1STAbits.TRMT == 0) ;      /* Wait for all value to be sent */
    U1MODE = 0;                        /* Then switch off UART */
  }

  U1BRG = 0x4B;                        /* Baud rate: 57600 (-0.06%) */
  U1MODE = 0x8000;
  U1STA = 0x2400;

  /* Configure UART1 Tx Interruption */
  MCHP_UART1_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART1_Tx.tail = 0;
  _U1TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U1TXIF = 0;
  _U1TXIE = 1;                         /* Enable Interrupt */

  /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
  U4BRG = 0x71;                        /* Baud rate: 38400 (-0.06%) */
  U4MODE = 0x8000;
  U4STA = 0x2400;

  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U4TXIF = 0;
  _U4TXIE = 1;                         /* Enable Interrupt */

  /* InitializeConditions for Delay: '<S5>/Delay' */
  AUAV_V3_TestMavLink_DWork.Delay_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S15>/Enables//Disables the Computation of  initial Baro Bias' */
  AUAV_V3_TestMavLink_DWork.aveCount = 1.0;
  AUAV_V3_TestMavLink_DWork.tIni = 1.0;

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
