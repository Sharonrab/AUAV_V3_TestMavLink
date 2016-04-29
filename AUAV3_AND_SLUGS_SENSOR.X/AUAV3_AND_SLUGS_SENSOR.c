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
 * File: AUAV3_AND_SLUGS_SENSOR.c
 *
 * Real-Time Workshop code generated for Simulink model AUAV3_AND_SLUGS_SENSOR.
 *
 * Model version                        : 1.219
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Fri Apr 29 16:39:44 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Fri Apr 29 16:39:46 2016
 */

#include "AUAV3_AND_SLUGS_SENSOR.h"
#include "AUAV3_AND_SLUGS_SENSOR_private.h"

const mavlink_scaled_pressure_t
  AUAV3_AND_SLUGS_SENSOR_rtZmavlink_scaled_pressure_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_gps_raw_int_t AUAV3_AND_SLUGS_SENSOR_rtZmavlink_gps_raw_int_t = {
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

const mavlink_raw_pressure_t AUAV3_AND_SLUGS_SENSOR_rtZmavlink_raw_pressure_t =
  {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_sys_status_t AUAV3_AND_SLUGS_SENSOR_rtZmavlink_sys_status_t = {
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
BlockIO_AUAV3_AND_SLUGS_SENSO_T AUAV3_AND_SLUGS_SENSOR_B;

/* Block states (auto storage) */
D_Work_AUAV3_AND_SLUGS_SENSOR_T AUAV3_AND_SLUGS_SENSOR_DWork;

/* Real-time model */
RT_MODEL_AUAV3_AND_SLUGS_SENS_T AUAV3_AND_SLUGS_SENSOR_M_;
RT_MODEL_AUAV3_AND_SLUGS_SENS_T *const AUAV3_AND_SLUGS_SENSOR_M =
  &AUAV3_AND_SLUGS_SENSOR_M_;
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

/*
 * Output and update for enable system:
 *    '<S123>/Enabled Subsystem'
 *    '<S186>/Enabled Subsystem'
 */
void AUAV3_AND_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_AUAV3_AN_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S123>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S131>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S131>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S123>/Enabled Subsystem' */
}

/*
 * Initial conditions for atomic system:
 *    '<S123>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S186>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW)
{
  /* '<S132>:1:7' aveCount =1; */
  localDW->aveCount = 1.0;

  /* '<S132>:1:8' tIni = 1; */
  localDW->tIni = 1.0;
}

/*
 * Output and update for atomic system:
 *    '<S123>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S186>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW)
{
  /* MATLAB Function 'Read Barometer BMP180/Baro Altimeter/Enables/Disables the Computation of  initial Baro Bias': '<S132>:1' */
  /* '<S132>:1:6' if isempty(aveCount) */
  /* '<S132>:1:11' if aveCount < 2500 */
  if (localDW->aveCount < 2500.0) {
    /* '<S132>:1:12' aveCount = aveCount + 1; */
    localDW->aveCount++;
  }

  /* '<S132>:1:15' if aveCount == 2500 */
  if (localDW->aveCount == 2500.0) {
    /* '<S132>:1:16' tIni =0; */
    localDW->tIni = 0.0;

    /* '<S132>:1:17' aveCount = aveCount + 1; */
    localDW->aveCount++;
  }

  /* '<S132>:1:20' tOut = tIni; */
  localB->tOut = localDW->tIni;

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
}

/*
 * Initial conditions for atomic system:
 *    '<S160>/Embedded MATLAB Function'
 *    '<S160>/Embedded MATLAB Function1'
 *    '<S160>/Embedded MATLAB Function2'
 *    '<S161>/Embedded MATLAB Function'
 *    '<S161>/Embedded MATLAB Function1'
 *    '<S161>/Embedded MATLAB Function2'
 *    '<S162>/Embedded MATLAB Function'
 *    '<S162>/Embedded MATLAB Function1'
 *    '<S162>/Embedded MATLAB Function2'
 *    '<S193>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S160>/Embedded MATLAB Function'
 *    '<S160>/Embedded MATLAB Function1'
 *    '<S160>/Embedded MATLAB Function2'
 *    '<S161>/Embedded MATLAB Function'
 *    '<S161>/Embedded MATLAB Function1'
 *    '<S161>/Embedded MATLAB Function2'
 *    '<S162>/Embedded MATLAB Function'
 *    '<S162>/Embedded MATLAB Function1'
 *    '<S162>/Embedded MATLAB Function2'
 *    '<S193>/Embedded MATLAB Function'
 *    ...
 */
void AUA_EmbeddedMATLABFunction(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_AU_T *localB, rtDW_EmbeddedMATLABFunction_A_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S166>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S166>:1:9' if isempty(a) */
  if (!localDW->a_not_empty) {
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S166>:1:12' omega = 2*pi*f; */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S166>:1:13' a=T*omega/(2+T*omega); */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S166>:1:14' b=-(T*omega-2)/(T*omega+2); */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S166>:1:15' y_km1=u; */
    localDW->y_km1 = rtu_u;

    /* '<S166>:1:16' u_km1=u; */
    localDW->u_km1 = rtu_u;
  }

  /* '<S166>:1:19' y = a*(u+u_km1)+b*y_km1; */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S166>:1:20' y_km1=y; */
  localDW->y_km1 = localB->y;

  /* '<S166>:1:21' u_km1=u; */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S155>/Embedded MATLAB Function'
 *    '<S155>/Embedded MATLAB Function1'
 *    '<S155>/Embedded MATLAB Function2'
 *    '<S155>/Embedded MATLAB Function3'
 */
void A_EmbeddedMATLABFunction_i(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_n_T *localB)
{
  /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function': '<S178>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S178>:1:5' y = (u(1)+ u(2) + u(3) + u(4) + u(5))*0.20; */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/* Model step function for TID0 */
void AUAV3_AND_SLUGS_SENSOR_step0(void) /* Sample time: [0.0025s, 0.0s] */
{
  uint64m_T tmp;
  uint64m_T tmp_0;
  uint32_T tmp_1;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S2>/Output'
   */
  tmp_1 = 3276800000UL;
  uMultiWordMul(&tmp_1, 1, &AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE, 1,
                &tmp_0.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_0.chunks[0U], 2, 17U, &tmp.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec = MultiWord2uLong
    (&tmp.chunks[0U]);

  /* Switch: '<S17>/FixPt Switch' incorporates:
   *  Constant: '<S16>/FixPt Constant'
   *  Sum: '<S16>/FixPt Sum1'
   *  UnitDelay: '<S2>/Output'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE++;
}

/* Model step function for TID1 */
void AUAV3_AND_SLUGS_SENSOR_step1(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_DataTypeConversion_i;
  real_T rtb_DiscreteZeroPole;
  real_T rtb_DiscreteZeroPole_d;
  boolean_T rtb_EnableHILfromControlMCU1;
  uint16_T rtb_Sum1_n;
  int16_T rtb_DataTypeConversion1_h;
  uint16_T rtb_BitwiseOperator_i;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_DataTypeConversion2_h;
  real32_T rtb_Sum1;
  boolean_T rtb_LogicalOperator_o;
  int16_T rtb_Switch_j[13];
  uint8_T rtb_Compare;
  real32_T rtb_u001maxDynPress;
  real32_T rtb_Sum_e;
  int16_T rtb_DataTypeConversion1_ck;
  int16_T rtb_DataTypeConversion2_bc;
  int16_T rtb_DataTypeConversion3_j;
  uint32_T rtb_Sum6;
  real_T tmp;
  uint32_T qY;
  uint32_T qY_0;

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  AUAV3_AND_SLUGS_SENSOR_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = AUAV3_AND_SLUGS_SENSOR_B.U3CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S7>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* number of I2C blocks : 5 ; Current: 2 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C22_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[0] = I2C22_Buff8[0];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[1] = I2C22_Buff8[1];
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

  /* Sum: '<S125>/SumA21' incorporates:
   *  DataTypeConversion: '<S127>/Data Type Conversion1'
   *  DataTypeConversion: '<S127>/Data Type Conversion3'
   *  Delay: '<S125>/Delay11'
   *  Gain: '<S125>/a(2)(1)'
   *  Gain: '<S125>/s(1)'
   *  Gain: '<S127>/Gain'
   *  S-Function (sfix_bitop): '<S127>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[0]
            << 8 | AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[1]) *
    31949UL >> 5;
  qY_0 = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE);
  if (qY_0 < rtb_B4) {
    qY_0 = MAX_uint32_T;
  }

  /* Sum: '<S125>/SumB21' incorporates:
   *  Delay: '<S125>/Delay11'
   *  Sum: '<S125>/SumA21'
   */
  if (qY_0 > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY_0 << 1;
  }

  if (AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE << 1;
  }

  qY = rtb_B4 + rtb_Sum6;
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  rtb_Sum1_n = (uint16_T)(qY >> 16);

  /* End of Sum: '<S125>/SumB21' */

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S7>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  /* number of I2C blocks : 5 ; Current: 3 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C23_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[0] = I2C23_Buff8[0];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[1] = I2C23_Buff8[1];
    MCHP_I2C23_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 20;
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

  /* Sum: '<S126>/SumA21' incorporates:
   *  DataTypeConversion: '<S128>/Data Type Conversion1'
   *  DataTypeConversion: '<S128>/Data Type Conversion3'
   *  Delay: '<S126>/Delay11'
   *  Gain: '<S126>/a(2)(1)'
   *  Gain: '<S126>/s(1)'
   *  Gain: '<S128>/Gain'
   *  S-Function (sfix_bitop): '<S128>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[0]
            << 8 | AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[1]) *
    31949UL >> 5;
  qY = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j);
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S126>/SumB21' incorporates:
   *  Delay: '<S126>/Delay11'
   *  Sum: '<S126>/SumA21'
   */
  if (qY > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY << 1;
  }

  if (AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j << 1;
  }

  rtb_Sum6 += rtb_B4;
  if (rtb_Sum6 < rtb_B4) {
    rtb_Sum6 = MAX_uint32_T;
  }

  rtb_BitwiseOperator_i = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S126>/SumB21' */

  /* DataStoreWrite: '<S7>/Update RawPressure' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion7'
   *  DataTypeConversion: '<S7>/Data Type Conversion8'
   */
  mlRawPressureData.press_abs = (int16_T)rtb_Sum1_n;
  mlRawPressureData.temperature = (int16_T)rtb_BitwiseOperator_i;

  /* Outputs for Enabled SubSystem: '<S7>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S124>/Enable'
   */
  /* Delay: '<S7>/Delay' */
  if (AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S124>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
    /* number of I2C blocks : 5 ; Current: 1 ; MCHP_I2C_StartImplemented =  5*/
    {
      /* Enable I2C sequence */
      MCHP_I2C21_Request ++;
      MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 36;
      if (MCHP_I2C2_Queue.head >= 5)   /* There are 5 blocks I2C2, max idx for queue is 5 */
        MCHP_I2C2_Queue.head = 0;
      else
        MCHP_I2C2_Queue.head ++;
      if (MCHP_I2C2_State == 0)
        _MI2C2IF = 1;

      /* Wait for end of SPI sequence (handle within an interrupt) */
      while (MCHP_I2C21_Request != 0) ;/* Wait until end of SPI sequence. */
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[0] = I2C21_Buff8
        [0];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[1] = I2C21_Buff8
        [1];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[2] = I2C21_Buff8
        [2];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[3] = I2C21_Buff8
        [3];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[4] = I2C21_Buff8
        [4];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[5] = I2C21_Buff8
        [5];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[6] = I2C21_Buff8
        [6];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[7] = I2C21_Buff8
        [7];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[8] = I2C21_Buff8
        [8];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[9] = I2C21_Buff8
        [9];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[10] =
        I2C21_Buff8[10];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[11] =
        I2C21_Buff8[11];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[12] =
        I2C21_Buff8[12];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[13] =
        I2C21_Buff8[13];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[14] =
        I2C21_Buff8[14];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[15] =
        I2C21_Buff8[15];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[16] =
        I2C21_Buff8[16];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[17] =
        I2C21_Buff8[17];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[18] =
        I2C21_Buff8[18];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[19] =
        I2C21_Buff8[19];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[20] =
        I2C21_Buff8[20];
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[21] =
        I2C21_Buff8[21];
    }

    /* DataTypeConversion: '<S124>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S141>/Data Type Conversion1'
     *  DataTypeConversion: '<S141>/Data Type Conversion3'
     *  Gain: '<S141>/Gain'
     *  S-Function (sfix_bitop): '<S141>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition3 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S124>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S140>/Data Type Conversion1'
     *  DataTypeConversion: '<S140>/Data Type Conversion3'
     *  Gain: '<S140>/Gain'
     *  S-Function (sfix_bitop): '<S140>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S124>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S142>/Data Type Conversion1'
     *  DataTypeConversion: '<S142>/Data Type Conversion3'
     *  Gain: '<S142>/Gain'
     *  S-Function (sfix_bitop): '<S142>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S124>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S146>/Data Type Conversion1'
     *  DataTypeConversion: '<S146>/Data Type Conversion3'
     *  Gain: '<S146>/Gain'
     *  S-Function (sfix_bitop): '<S146>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S124>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S147>/Data Type Conversion1'
     *  DataTypeConversion: '<S147>/Data Type Conversion3'
     *  Gain: '<S147>/Gain'
     *  S-Function (sfix_bitop): '<S147>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S124>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S138>/Data Type Conversion1'
     *  DataTypeConversion: '<S138>/Data Type Conversion3'
     *  Gain: '<S138>/Gain'
     *  S-Function (sfix_bitop): '<S138>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S124>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S139>/Data Type Conversion1'
     *  DataTypeConversion: '<S139>/Data Type Conversion3'
     *  Gain: '<S139>/Gain'
     *  S-Function (sfix_bitop): '<S139>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S143>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S143>/Data Type Conversion1'
     *  DataTypeConversion: '<S143>/Data Type Conversion3'
     *  Gain: '<S143>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition5 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S144>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S144>/Data Type Conversion1'
     *  DataTypeConversion: '<S144>/Data Type Conversion3'
     *  Gain: '<S144>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition6 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S145>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S145>/Data Type Conversion1'
     *  DataTypeConversion: '<S145>/Data Type Conversion3'
     *  Gain: '<S145>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition7 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S7>/Delay' */
  /* End of Outputs for SubSystem: '<S7>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S122>/Sum' */
  rtb_Sum1_n -= AUAV3_AND_SLUGS_SENSOR_B.RateTransition7;

  /* Product: '<S122>/Product' */
  rtb_Sum6 = (uint32_T)rtb_Sum1_n * AUAV3_AND_SLUGS_SENSOR_B.RateTransition6;
  rtb_Sum1_n = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) != 0U) +
    (rtb_Sum6 >> 15));

  /* Sum: '<S122>/Sum2' incorporates:
   *  Bias: '<S122>/Bias'
   *  Bias: '<S122>/Bias1'
   *  Product: '<S122>/Product1'
   *  Sum: '<S122>/Sum1'
   */
  rtb_DataTypeConversion1_h = (div_s16s32_round((int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 << 11, rtb_Sum1_n +
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition12) + (int16_T)rtb_Sum1_n) - 4000;

  /* MATLAB Function: '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S123>/Zero Out Height' incorporates:
   *  EnablePort: '<S134>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Sum: '<S134>/Sum' incorporates:
     *  Constant: '<S123>/Constant5'
     *  Delay: '<S134>/Integer Delay'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Sum_k = 0.0F -
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE_j;
  }

  /* End of Outputs for SubSystem: '<S123>/Zero Out Height' */

  /* Math: '<S122>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_DataTypeConversion1_h * rtb_DataTypeConversion1_h;

  /* Sum: '<S122>/Sum6' incorporates:
   *  Bias: '<S122>/Bias2'
   *  Product: '<S122>/Product2'
   *  Product: '<S122>/Product3'
   *  Sum: '<S122>/Sum3'
   *  Sum: '<S122>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_BitwiseOperator_i - (((((int16_T)((int32_T)
    rtb_DataTypeConversion1_h * AUAV3_AND_SLUGS_SENSOR_B.RateTransition3 >> 11)
    + (int16_T)mul_s32_s32_s32_sr23(AUAV3_AND_SLUGS_SENSOR_B.RateTransition9,
    rtb_Bias5)) + ((int32_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition2 << 2)) + 2L)
    >> 2);

  /* Product: '<S122>/Product6' incorporates:
   *  Bias: '<S122>/Bias3'
   *  Bias: '<S122>/Bias4'
   *  Gain: '<S122>/Gain1'
   *  Product: '<S122>/Product4'
   *  Product: '<S122>/Product5'
   *  Sum: '<S122>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15(((((int16_T)((int32_T)rtb_DataTypeConversion1_h *
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 >> 13) + (int16_T)
    mul_s32_s32_s32_sr28(rtb_Bias5, AUAV3_AND_SLUGS_SENSOR_B.RateTransition8)) +
    2) >> 2) + 32768L, AUAV3_AND_SLUGS_SENSOR_B.RateTransition5);

  /* Switch: '<S122>/Switch' incorporates:
   *  Gain: '<S122>/Gain15'
   *  Gain: '<S122>/Gain22'
   *  Product: '<S122>/Product7'
   *  Product: '<S122>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S122>/Switch' */

  /* Gain: '<S122>/Gain16' */
  rtb_Bias5 = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >>
    8));

  /* Sum: '<S122>/Sum8' incorporates:
   *  Bias: '<S122>/Bias5'
   *  Gain: '<S122>/Gain17'
   *  Gain: '<S122>/Gain19'
   *  Gain: '<S122>/Gain21'
   *  Math: '<S122>/Math Function2'
   *  Sum: '<S122>/Sum7'
   */
  rtb_Bias5 = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat
    (rtb_Bias5, rtb_Bias5)) + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >>
               4) + (int32_T)rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S123>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S133>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* DataTypeConversion: '<S133>/Data Type Conversion' */
    rtb_DataTypeConversion_i = rtb_Bias5;

    /* DiscreteZeroPole: '<S136>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_d = 0.014778325123152709*rtb_DataTypeConversion_i;
      rtb_DiscreteZeroPole_d += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_n;
    }

    /* Saturate: '<S133>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S133>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_d > 120000.0F) {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k_i = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_d < 80000.0F) {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k_i = 80000.0F;
    } else {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k_i = (real32_T)rtb_DiscreteZeroPole_d;
    }

    /* End of Saturate: '<S133>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S136>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_n =
        rtb_DataTypeConversion_i + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_n;
    }
  }

  /* End of Outputs for SubSystem: '<S123>/Initial Baro Bias' */

  /* Product: '<S130>/Divide' incorporates:
   *  Sum: '<S130>/Sum2'
   */
  rtb_DataTypeConversion2_h = ((real32_T)rtb_Bias5 -
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k_i) / AUAV3_AND_SLUGS_SENSOR_B.u0k120k_i;

  /* Sum: '<S130>/Sum1' incorporates:
   *  Constant: '<S130>/Constant2'
   *  Constant: '<S130>/Constant3'
   *  Constant: '<S130>/Constant4'
   *  Constant: '<S130>/Constant5'
   *  Gain: '<S135>/Unit Conversion'
   *  Product: '<S130>/Divide1'
   *  Product: '<S130>/Divide2'
   *  Product: '<S130>/Divide3'
   *  Product: '<S130>/Divide4'
   *  Sum: '<S130>/Sum3'
   */
  rtb_Sum1 = ((rtb_DataTypeConversion2_h * rtb_DataTypeConversion2_h *
               0.093502529F + rtb_DataTypeConversion2_h * -0.188893303F) +
              2.18031291E-5F) * 145473.5F * 0.3048F;

  /* Outputs for Enabled SubSystem: '<S123>/Enabled Subsystem' */

  /* Logic: '<S123>/Logical Operator' incorporates:
   *  Sum: '<S123>/Sum1'
   */
  AUAV3_AND_EnabledSubsystem
    (!(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut != 0.0),
     AUAV3_AND_SLUGS_SENSOR_B.Sum_k + rtb_Sum1,
     &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem);

  /* End of Outputs for SubSystem: '<S123>/Enabled Subsystem' */

  /* DataStoreWrite: '<S7>/Update ScaledPressure' incorporates:
   *  Bias: '<S122>/Bias'
   *  Bias: '<S122>/Bias1'
   *  Gain: '<S7>/Gain21'
   */
  mlAirData.temperature = (int16_T)((rtb_DataTypeConversion1_h + 4008) * 5L >> 3);
  mlAirData.press_abs = AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem.In1;

  /* S-Function (MCHP_BUS_SPI): '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  /* number of SPI blocks : 2 ; Current: 1 ; MCHP_SPI_StartImplemented =  2*/
  if (MCHP_SPI11_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[0] = SPI11_Buff16[0];
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[1] = SPI11_Buff16[1];
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[2] = SPI11_Buff16[2];
    AUAV3_AND_SLUGS_SENSOR_B.U1CH4 = SPI11_Buff16[3];
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0] = SPI11_Buff16[4];
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1] = SPI11_Buff16[5];
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2] = SPI11_Buff16[6];
    MCHP_SPI11_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 8;
    if (MCHP_SPI1_Queue.head >= 2)     /* There are 2 blocks SPI1, max idx for queue is 2 */
      MCHP_SPI1_Queue.head = 0;
    else
      MCHP_SPI1_Queue.head ++;
    if (MCHP_SPI1_State == 0)
      _SPI1IF = 1;                     /* Force Interrupt */
  }

  /* DataStoreWrite: '<S8>/Update MPU_T' */
  MPU_T = AUAV3_AND_SLUGS_SENSOR_B.U1CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S8>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  /* number of I2C blocks : 5 ; Current: 4 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C24_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[0] = I2C24_Buff8[0];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[1] = I2C24_Buff8[1];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[2] = I2C24_Buff8[2];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[3] = I2C24_Buff8[3];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[4] = I2C24_Buff8[4];
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[5] = I2C24_Buff8[5];
    MCHP_I2C24_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 87;
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

  /* DataStoreWrite: '<S8>/Update Raw IMU DATA' incorporates:
   *  DataTypeConversion: '<S148>/Data Type Conversion3'
   *  DataTypeConversion: '<S149>/Data Type Conversion3'
   *  DataTypeConversion: '<S150>/Data Type Conversion3'
   *  DataTypeConversion: '<S8>/Data Type Conversion'
   *  DataTypeConversion: '<S8>/Data Type Conversion1'
   *  DataTypeConversion: '<S8>/Data Type Conversion2'
   *  DataTypeConversion: '<S8>/Data Type Conversion3'
   *  DataTypeConversion: '<S8>/Data Type Conversion4'
   *  DataTypeConversion: '<S8>/Data Type Conversion5'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xacc = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.yacc = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zacc = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.U1CH8[2];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xgyro = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.ygyro = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zgyro = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xmag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.ymag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[3];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zmag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[5];

  /* S-Function (MCHP_Digital_Input): '<S9>/Enable HIL from  Control MCU1' */

  /* MCHP_Digital_Input Block: <S9>/Enable HIL from  Control MCU1/Output */
  rtb_EnableHILfromControlMCU1 = PORTDbits.RD2;/* Read pin D2 */

  /* Outputs for Enabled SubSystem: '<S9>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S151>/Enable'
   */
  if (rtb_EnableHILfromControlMCU1) {
    /* S-Function (MCHP_C_function_Call): '<S151>/Data from HIL [hil.c]2' */
    hilRead(
            &AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2[0]
            );

    /* S-Function (MCHP_C_function_Call): '<S151>/HIL Messages  Parser//Decoder [hil.c]1' */
    protDecodeHil(
                  &AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2[0]
                  );

    /* S-Function (MCHP_C_function_Call): '<S151>/HIL Raw Readings [hil.c]1' */
    hil_getRawRead(
                   &AUAV3_AND_SLUGS_SENSOR_B.HILMessagesParserDecoderhilc1[0]
                   );
  }

  /* End of Outputs for SubSystem: '<S9>/Raw HIL  Readings' */

  /* Logic: '<S152>/Logical Operator' */
  rtb_LogicalOperator_o = !rtb_EnableHILfromControlMCU1;

  /* Outputs for Enabled SubSystem: '<S152>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S155>/Enable'
   */
  if (rtb_LogicalOperator_o) {
    /* MATLAB Function: '<S155>/Embedded MATLAB Function' incorporates:
     *  Constant: '<S155>/Constant'
     */
    A_EmbeddedMATLABFunction_i(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_i);

    /* DataTypeConversion: '<S155>/Data Type Conversion' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_i.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion1_h = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function1' incorporates:
     *  Constant: '<S155>/Constant1'
     */
    A_EmbeddedMATLABFunction_i(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_m);

    /* DataTypeConversion: '<S155>/Data Type Conversion1' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_m.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion1_ck = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion1' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function2' incorporates:
     *  Constant: '<S155>/Constant2'
     */
    A_EmbeddedMATLABFunction_i(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_db);

    /* DataTypeConversion: '<S155>/Data Type Conversion2' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_db.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion2_bc = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion2' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function3' incorporates:
     *  Constant: '<S155>/Constant3'
     */
    A_EmbeddedMATLABFunction_i(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3);

    /* DataTypeConversion: '<S155>/Data Type Conversion3' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion3_j = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion3' */

    /* MATLAB Function: '<S155>/myMux Fun' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S184>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S184>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
    AUAV3_AND_SLUGS_SENSOR_B.y_b[0] = rtb_DataTypeConversion1_h;
    AUAV3_AND_SLUGS_SENSOR_B.y_b[1] = rtb_DataTypeConversion1_ck;
    AUAV3_AND_SLUGS_SENSOR_B.y_b[2] = rtb_DataTypeConversion2_bc;
    AUAV3_AND_SLUGS_SENSOR_B.y_b[3] = rtb_DataTypeConversion3_j;

    /* S-Function (MCHP_C_function_Call): '<S155>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     &AUAV3_AND_SLUGS_SENSOR_B.y_b[0]
                     );

    /* S-Function (MCHP_C_function_Call): '<S155>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                &AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[0]
                );

    /* MATLAB Function: '<S155>/myMux Fun4' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S185>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S185>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1); u6(1); u7(1); u8(1); u9(1); u10(1); u11(1); u12(1); u13(1)]; */
    AUAV3_AND_SLUGS_SENSOR_B.y_g[0] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[0];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[1] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[1];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[2] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[2];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[3] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[3];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[4] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[4];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[5] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[5];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[6] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[6];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[7] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[7];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[8] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[8];
    AUAV3_AND_SLUGS_SENSOR_B.y_g[9] = rtb_DataTypeConversion1_h;
    AUAV3_AND_SLUGS_SENSOR_B.y_g[10] = rtb_DataTypeConversion1_ck;
    AUAV3_AND_SLUGS_SENSOR_B.y_g[11] = rtb_DataTypeConversion2_bc;
    AUAV3_AND_SLUGS_SENSOR_B.y_g[12] = rtb_DataTypeConversion3_j;

    /* S-Function (MCHP_C_function_Call): '<S155>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 = isGPSNovatel(
      );

    /* Outputs for Enabled SubSystem: '<S155>/if GPS is Novatel' incorporates:
     *  EnablePort: '<S182>/Enable'
     */
    if (AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 > 0) {
      /* S-Function (MCHP_C_function_Call): '<S182>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]2' */
      getGpsMainData(
                     &AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdat_c[0]
                     );

      /* S-Function (MCHP_C_function_Call): '<S182>/Read the Raw Data from GPS [gpsPort.c]2' */
      getGPSRawData(
                    &AUAV3_AND_SLUGS_SENSOR_B.ReadtheRawDatafromGPSgpsPortc2
                    );

      /* S-Function (MCHP_C_function_Call): '<S182>/Parse the GPS RAW Data [gps.c//novatel.c]2' */
      gpsParse(
               &AUAV3_AND_SLUGS_SENSOR_B.ReadtheRawDatafromGPSgpsPortc2
               );
    }

    /* End of Outputs for SubSystem: '<S155>/if GPS is Novatel' */

    /* Outputs for Enabled SubSystem: '<S155>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S183>/Enable'
     */
    /* Logic: '<S155>/Logical Operator' */
    if (!(AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 != 0)) {
      /* S-Function (MCHP_C_function_Call): '<S183>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdatet
                          [0]
                          );
    }

    /* End of Logic: '<S155>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S155>/if GPS is Ublox' */
  }

  /* End of Outputs for SubSystem: '<S152>/If no HIL then Read all the Sensors' */

  /* Switch: '<S152>/Switch' */
  for (rtb_DataTypeConversion1_h = 0; rtb_DataTypeConversion1_h < 13;
       rtb_DataTypeConversion1_h++) {
    if (rtb_EnableHILfromControlMCU1) {
      rtb_Switch_j[rtb_DataTypeConversion1_h] =
        AUAV3_AND_SLUGS_SENSOR_B.HILRawReadingshilc1[rtb_DataTypeConversion1_h];
    } else {
      rtb_Switch_j[rtb_DataTypeConversion1_h] =
        AUAV3_AND_SLUGS_SENSOR_B.y_g[rtb_DataTypeConversion1_h];
    }
  }

  /* End of Switch: '<S152>/Switch' */

  /* MATLAB Function: '<S193>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S193>/Constant'
   *  Constant: '<S193>/Constant1'
   *  DataTypeConversion: '<S157>/Data Type Conversion5'
   */
  AUA_EmbeddedMATLABFunction((real_T)rtb_Switch_j[12], 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction);

  /* Sum: '<S192>/Sum' incorporates:
   *  Constant: '<S192>/Bias'
   *  Product: '<S192>/Divide'
   */
  rtb_DataTypeConversion2_h = (real32_T)(1.5112853050231934 *
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction.y) + -1605.28198F;

  /* RelationalOperator: '<S205>/Compare' incorporates:
   *  Constant: '<S205>/Constant'
   */
  rtb_Compare = (uint8_T)(rtb_DataTypeConversion2_h < -130.0F);

  /* Outputs for Enabled SubSystem: '<S190>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S206>/Enable'
   */
  /* Logic: '<S190>/Logical Operator' */
  if (!(rtb_Compare != 0)) {
    /* Sum: '<S206>/Sum2' incorporates:
     *  Constant: '<S206>/Mean Temperature for Calibration'
     *  Constant: '<S206>/gains'
     *  Product: '<S206>/Divide1'
     *  Sum: '<S206>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = (real32_T)rtb_Switch_j[10] -
      (rtb_DataTypeConversion2_h - 293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S190>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S190>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S190>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S207>/Enable'
   */
  if (rtb_Compare > 0) {
    /* Sum: '<S207>/Add' incorporates:
     *  Constant: '<S207>/Constant'
     *  Constant: '<S207>/Mean Temperature for Calibration'
     *  Constant: '<S207>/gains'
     *  Product: '<S207>/Divide1'
     *  Sum: '<S207>/Sum1'
     *  Sum: '<S207>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = ((real32_T)rtb_Switch_j[10] -
      (rtb_DataTypeConversion2_h - -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S190>/Lo Temp Compensation' */

  /* MATLAB Function: '<S195>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S195>/Constant'
   *  Constant: '<S195>/Constant1'
   *  DataTypeConversion: '<S157>/Data Type Conversion3'
   */
  AUA_EmbeddedMATLABFunction((real_T)AUAV3_AND_SLUGS_SENSOR_B.Merge, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

  /* Sum: '<S188>/Sum' incorporates:
   *  Constant: '<S188>/Bias'
   *  Constant: '<S188>/Gains'
   *  DataTypeConversion: '<S157>/Data Type Conversion4'
   *  Product: '<S188>/Divide'
   */
  rtb_u001maxDynPress = 1.05137849F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o.y + -1005.87872F;

  /* Saturate: '<S157>/[0.001  maxDynPress]' */
  if (rtb_u001maxDynPress > 3000.0F) {
    rtb_u001maxDynPress = 3000.0F;
  } else {
    if (rtb_u001maxDynPress < 0.001F) {
      rtb_u001maxDynPress = 0.001F;
    }
  }

  /* End of Saturate: '<S157>/[0.001  maxDynPress]' */

  /* RelationalOperator: '<S208>/Compare' incorporates:
   *  Constant: '<S208>/Constant'
   */
  rtb_Compare = (uint8_T)(rtb_DataTypeConversion2_h < -50.0F);

  /* Outputs for Enabled SubSystem: '<S191>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S209>/Enable'
   */
  /* Logic: '<S191>/Logical Operator' */
  if (!(rtb_Compare != 0)) {
    /* Sum: '<S209>/Add' incorporates:
     *  Constant: '<S209>/Constant'
     *  Constant: '<S209>/Mean Temperature for Calibration'
     *  Constant: '<S209>/gains'
     *  Product: '<S209>/Divide1'
     *  Sum: '<S209>/Sum1'
     *  Sum: '<S209>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_k = ((real32_T)rtb_Switch_j[9] -
      (rtb_DataTypeConversion2_h - 347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S191>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S191>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S191>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S210>/Enable'
   */
  if (rtb_Compare > 0) {
    /* Sum: '<S210>/Sum2' incorporates:
     *  Constant: '<S210>/Mean Temperature for Calibration'
     *  Constant: '<S210>/gains'
     *  Product: '<S210>/Divide1'
     *  Sum: '<S210>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_k = (real32_T)rtb_Switch_j[9] -
      (rtb_DataTypeConversion2_h - -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S191>/Lo Temp Compensation' */

  /* MATLAB Function: '<S196>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S196>/Constant'
   *  Constant: '<S196>/Constant1'
   *  DataTypeConversion: '<S157>/Data Type Conversion19'
   */
  AUA_EmbeddedMATLABFunction((real_T)AUAV3_AND_SLUGS_SENSOR_B.Merge_k, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_a,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_a);

  /* Sum: '<S187>/Sum' incorporates:
   *  Constant: '<S187>/Bias'
   *  Constant: '<S187>/Gains'
   *  DataTypeConversion: '<S157>/Data Type Conversion1'
   *  Product: '<S187>/Divide'
   */
  rtb_Sum_e = 27.127F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_a.y + 9444.44434F;

  /* Outputs for Enabled SubSystem: '<S152>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S156>/Enable'
   */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun': '<S158>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S158>:1:5' y = [u1(1); u2(1); u3(1)]; */
  if (rtb_LogicalOperator_o) {
    /* Inport: '<S156>/AirData' incorporates:
     *  MATLAB Function: '<S152>/myMux Fun'
     */
    AUAV3_AND_SLUGS_SENSOR_B.AirData[0] = rtb_u001maxDynPress;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[1] = rtb_Sum_e;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[2] = rtb_DataTypeConversion2_h;

    /* S-Function (MCHP_C_function_Call): '<S156>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
    updateAirData(
                  &AUAV3_AND_SLUGS_SENSOR_B.AirData[0]
                  );
  }

  /* End of Outputs for SubSystem: '<S152>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S152>/myMux Fun1' */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun1': '<S159>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S159>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
  AUAV3_AND_SLUGS_SENSOR_B.y[0] = rtb_u001maxDynPress;
  AUAV3_AND_SLUGS_SENSOR_B.y[1] = AUAV3_AND_SLUGS_SENSOR_B.AirData[0];
  AUAV3_AND_SLUGS_SENSOR_B.y[2] = 0.0F;
  AUAV3_AND_SLUGS_SENSOR_B.y[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S152>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   &AUAV3_AND_SLUGS_SENSOR_B.y[0]
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S154>/Calculus Time Step1 */
  AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S154>/Calculus Time Step2 */
  {
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2 = tmp;
  }

  /* DataTypeConversion: '<S154>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S154>/Data Type Conversion1'
   *  DataTypeConversion: '<S154>/Data Type Conversion2'
   *  Gain: '<S154>/Gain'
   *  Product: '<S154>/Divide'
   *  Rounding: '<S154>/Rounding Function'
   */
  tmp = floor((real_T)AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1 / (real_T)
              AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2 * 100.0);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 256.0);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12 = (uint8_T)(tmp < 0.0 ? (int16_T)
    (uint8_T)-(int8_T)(uint8_T)-tmp : (int16_T)(uint8_T)tmp);

  /* End of DataTypeConversion: '<S154>/Data Type Conversion12' */

  /* MATLAB Function: '<S194>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S194>/Constant'
   *  Constant: '<S194>/Constant1'
   *  DataTypeConversion: '<S157>/Data Type Conversion6'
   */
  AUA_EmbeddedMATLABFunction((real_T)rtb_Switch_j[11], 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_j,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_j);

  /* DataTypeConversion: '<S157>/Data Type Conversion8' incorporates:
   *  Constant: '<S189>/Bias'
   *  Product: '<S189>/Divide'
   *  Sum: '<S189>/Sum'
   */
  rtb_DataTypeConversion2_h = (real32_T)floor((real32_T)(3.1760616302490234 *
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_j.y) + 911.698242F);
  if (rtIsNaNF(rtb_DataTypeConversion2_h) || rtIsInfF(rtb_DataTypeConversion2_h))
  {
    rtb_DataTypeConversion2_h = 0.0F;
  } else {
    rtb_DataTypeConversion2_h = (real32_T)fmod(rtb_DataTypeConversion2_h,
      65536.0F);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8 = rtb_DataTypeConversion2_h <
    0.0F ? (uint16_T)-(int16_T)(uint16_T)-rtb_DataTypeConversion2_h : (uint16_T)
    rtb_DataTypeConversion2_h;

  /* End of DataTypeConversion: '<S157>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S152>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12
                 , AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8
                 );

  /* MATLAB Function: '<S160>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S153>/Gyro Gains'
   *  Constant: '<S160>/Constant'
   *  Constant: '<S160>/Constant1'
   *  DataTypeConversion: '<S153>/Data Type Conversion2'
   *  Gain: '<S163>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[1] *
    0.000872664619F), 0.0025, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_p,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_p);

  /* MATLAB Function: '<S160>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S153>/Gyro Gains'
   *  Constant: '<S160>/Constant2'
   *  Constant: '<S160>/Constant3'
   *  DataTypeConversion: '<S153>/Data Type Conversion2'
   *  Gain: '<S163>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[0] *
    0.000872664619F), 0.0025, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S160>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S153>/Gyro Gains'
   *  Constant: '<S160>/Constant4'
   *  Constant: '<S160>/Constant5'
   *  DataTypeConversion: '<S153>/Data Type Conversion2'
   *  Gain: '<S163>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[2] *
    0.000872664619F), 0.0025, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

  /* MATLAB Function: '<S161>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S153>/Gyro Gains1'
   *  Constant: '<S161>/Constant'
   *  Constant: '<S161>/Constant1'
   *  DataTypeConversion: '<S153>/Data Type Conversion1'
   *  Product: '<S153>/Divide1'
   */
  AUA_EmbeddedMATLABFunction((real_T)((real32_T)rtb_Switch_j[4] * 0.0326839499F),
    0.0025, 40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_ph,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_ph);

  /* MATLAB Function: '<S161>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S153>/Gyro Gains1'
   *  Constant: '<S161>/Constant2'
   *  Constant: '<S161>/Constant3'
   *  DataTypeConversion: '<S153>/Data Type Conversion1'
   *  Product: '<S153>/Divide1'
   */
  AUA_EmbeddedMATLABFunction((real_T)((real32_T)rtb_Switch_j[3] * 0.0326839499F),
    0.0025, 40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_h,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_h);

  /* MATLAB Function: '<S161>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S153>/Gyro Gains1'
   *  Constant: '<S161>/Constant4'
   *  Constant: '<S161>/Constant5'
   *  DataTypeConversion: '<S153>/Data Type Conversion1'
   *  Product: '<S153>/Divide1'
   */
  AUA_EmbeddedMATLABFunction((real_T)((real32_T)rtb_Switch_j[5] * 0.0326839499F),
    0.0025, 40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_d,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_d);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S153>/Gyro Gains2'
   *  Constant: '<S162>/Constant'
   *  Constant: '<S162>/Constant1'
   *  DataTypeConversion: '<S153>/Data Type Conversion5'
   *  Gain: '<S165>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide2'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[7] * 0.5F), 0.0025,
    40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_d,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_d);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S153>/Gyro Gains2'
   *  Constant: '<S162>/Constant2'
   *  Constant: '<S162>/Constant3'
   *  DataTypeConversion: '<S153>/Data Type Conversion5'
   *  Gain: '<S165>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide2'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[6] * 0.5F), 0.0025,
    40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_d,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_d);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S153>/Gyro Gains2'
   *  Constant: '<S162>/Constant4'
   *  Constant: '<S162>/Constant5'
   *  DataTypeConversion: '<S153>/Data Type Conversion5'
   *  Gain: '<S165>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide2'
   */
  AUA_EmbeddedMATLABFunction((real_T)-((real32_T)rtb_Switch_j[8] * 0.5F), 0.0025,
    40.0, &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_f,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_f);

  /* MATLAB Function: '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_m,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_m);

  /* Outputs for Enabled SubSystem: '<S186>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S201>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_m.tOut > 0.0) {
    /* DataTypeConversion: '<S201>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Sum_e;

    /* DiscreteZeroPole: '<S204>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S201>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S201>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole > 120000.0F) {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole < 80000.0F) {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k = 80000.0F;
    } else {
      AUAV3_AND_SLUGS_SENSOR_B.u0k120k = (real32_T)rtb_DiscreteZeroPole;
    }

    /* End of Saturate: '<S201>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S204>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE =
        rtb_DataTypeConversion + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S186>/Initial Baro Bias' */

  /* Product: '<S197>/Divide' incorporates:
   *  Sum: '<S197>/Sum2'
   */
  rtb_DataTypeConversion2_h = (rtb_Sum_e - AUAV3_AND_SLUGS_SENSOR_B.u0k120k) /
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k;

  /* S-Function (MCHP_C_function_Call): '<S200>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0]
                );

  /* Sum: '<S197>/Sum1' incorporates:
   *  Constant: '<S197>/Constant2'
   *  Constant: '<S197>/Constant3'
   *  Constant: '<S197>/Constant4'
   *  Constant: '<S197>/Constant5'
   *  Gain: '<S203>/Unit Conversion'
   *  Product: '<S197>/Divide1'
   *  Product: '<S197>/Divide2'
   *  Product: '<S197>/Divide3'
   *  Product: '<S197>/Divide4'
   *  Sum: '<S197>/Sum3'
   */
  rtb_DataTypeConversion2_h = ((rtb_DataTypeConversion2_h *
    rtb_DataTypeConversion2_h * 0.093502529F + rtb_DataTypeConversion2_h *
    -0.188893303F) + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0];

  /* Outputs for Enabled SubSystem: '<S186>/Zero Out Height' incorporates:
   *  EnablePort: '<S202>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_m.tOut > 0.0) {
    /* Sum: '<S202>/Sum' incorporates:
     *  Delay: '<S202>/Integer Delay'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Sum =
      AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0] -
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE;

    /* Update for Delay: '<S202>/Integer Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE = rtb_DataTypeConversion2_h;
  }

  /* End of Outputs for SubSystem: '<S186>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S186>/Enabled Subsystem' */

  /* Logic: '<S186>/Logical Operator' incorporates:
   *  Sum: '<S186>/Sum1'
   */
  AUAV3_AND_EnabledSubsystem
    (!(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_m.tOut != 0.0),
     AUAV3_AND_SLUGS_SENSOR_B.Sum + rtb_DataTypeConversion2_h,
     &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_e);

  /* End of Outputs for SubSystem: '<S186>/Enabled Subsystem' */

  /* Update for Delay: '<S125>/Delay11' incorporates:
   *  Sum: '<S125>/SumA21'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE = qY_0;

  /* Update for Delay: '<S126>/Delay11' incorporates:
   *  Sum: '<S126>/SumA21'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j = qY;

  /* Update for Delay: '<S7>/Delay' incorporates:
   *  Constant: '<S7>/Constant'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S123>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S134>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Update for Delay: '<S134>/Integer Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE_j = rtb_Sum1;
  }

  /* End of Update for SubSystem: '<S123>/Zero Out Height' */
}

/* Model step function for TID2 */
void AUAV3_AND_SLUGS_SENSOR_step2(void) /* Sample time: [0.02s, 0.0s] */
{
  int16_T rtb_Bias_g;
  int16_T rtb_Bias1;
  int32_T tmp;

  /* S-Function "dsPIC_PWM_IC" Block: <S10>/Input Capture RC Receiver1 */
  AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw = MCHP_ic1up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o2 = MCHP_ic2up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o3 = MCHP_ic3up;
  AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw = MCHP_ic4up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o5 = MCHP_ic5up;

  /* DataStoreWrite: '<S10>/Update Raw RC Channels' */
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan1_raw =
    AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan2_raw =
    AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o2;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan3_raw =
    AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o3;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan4_raw =
    AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan5_raw =
    AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o5;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands.chan6_raw = 9999U;

  /* Bias: '<S10>/Bias' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion4'
   */
  rtb_Bias_g = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw - 13125;

  /* Bias: '<S10>/Bias1' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion5'
   */
  rtb_Bias1 = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw - 13125;

  /* Sum: '<S215>/Sum' */
  tmp = (int32_T)rtb_Bias1 - rtb_Bias_g;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S10>/Data Type Conversion2' incorporates:
   *  Bias: '<S10>/Bias2'
   *  Sum: '<S215>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S10>/Saturation' */
  if (AUAV3_AND_SLUGS_SENSOR_B.Saturation > 19249U) {
    /* DataTypeConversion: '<S10>/Data Type Conversion2' */
    AUAV3_AND_SLUGS_SENSOR_B.Saturation = 19249U;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_B.Saturation < 7000U) {
      /* DataTypeConversion: '<S10>/Data Type Conversion2' */
      AUAV3_AND_SLUGS_SENSOR_B.Saturation = 7000U;
    }
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Sum: '<S215>/Sum1' */
  tmp = (int32_T)rtb_Bias_g + rtb_Bias1;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S10>/Data Type Conversion3' incorporates:
   *  Bias: '<S10>/Bias3'
   *  Sum: '<S215>/Sum1'
   */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation1 = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S10>/Saturation1' */
  if (AUAV3_AND_SLUGS_SENSOR_B.Saturation1 > 19249U) {
    /* DataTypeConversion: '<S10>/Data Type Conversion3' */
    AUAV3_AND_SLUGS_SENSOR_B.Saturation1 = 19249U;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_B.Saturation1 < 7000U) {
      /* DataTypeConversion: '<S10>/Data Type Conversion3' */
      AUAV3_AND_SLUGS_SENSOR_B.Saturation1 = 7000U;
    }
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* S-Function "dsPIC_PWM_IC" Block: <S1>/Input Capture */
  AUAV3_AND_SLUGS_SENSOR_B.dT = MCHP_ic1up;
  AUAV3_AND_SLUGS_SENSOR_B.dA = MCHP_ic2up;
  AUAV3_AND_SLUGS_SENSOR_B.dE = MCHP_ic3up;
  AUAV3_AND_SLUGS_SENSOR_B.dR = MCHP_ic4up;
  AUAV3_AND_SLUGS_SENSOR_B.dFailsafe = MCHP_ic5up;

  /* DataStoreRead: '<S4>/Get Raw IMU' */
  /* MATLAB Function 'Control Surface Input/myMux Fun5': '<S15>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S15>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1)]; */
  AUAV3_AND_SLUGS_SENSOR_B.GetRawIMU = AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU;

  /* DataStoreRead: '<S4>/Get time' */
  AUAV3_AND_SLUGS_SENSOR_B.Gettime =
    AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackRawIMU' */
  AUAV3_AND_SLUGS_SENSOR_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetRawIMU
    , AUAV3_AND_SLUGS_SENSOR_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackRawIMU
                  );

  /* DataStoreRead: '<S4>/Get Raw RC' */
  AUAV3_AND_SLUGS_SENSOR_B.GetRawRC = AUAV3_AND_SLUGS_SENSOR_DWork.mlRC_Commands;

  /* DataStoreRead: '<S4>/Get time3' */
  AUAV3_AND_SLUGS_SENSOR_B.Gettime3 =
    AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackRawRC' */
  AUAV3_AND_SLUGS_SENSOR_B.PackRawRC = PackRawRC(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetRawRC
    , AUAV3_AND_SLUGS_SENSOR_B.Gettime3
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data7' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackRawRC
                  );

  /* S-Function "dsPIC_PWM_OC" Block: <S10>/Output Compare - HW Drive Servo motor */
  OC1CON1 = 0x1008;                    /* Disable OC1 */
  OC1CON2bits.TRIGSTAT = 0;
  OC1RS = AUAV3_AND_SLUGS_SENSOR_B.Saturation;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC2CON1 = 0x1008;                    /* Disable OC2 */
  OC2CON2bits.TRIGSTAT = 0;
  OC2RS = AUAV3_AND_SLUGS_SENSOR_B.Saturation1;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC1CON2bits.TRIGSTAT = 1;
  OC1CON1 = 0x100C;                    /* Trig OC1 pulse */
  OC2CON2bits.TRIGSTAT = 1;
  OC2CON1 = 0x100C;                    /* Trig OC2 pulse */
}

/* Model step function for TID3 */
void AUAV3_AND_SLUGS_SENSOR_step3(void) /* Sample time: [0.05s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator_h;

  /* S-Function (MCHP_Digital_Output_Read): '<S18>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S18>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S18>/Digital Output Read'
   */
  rtb_LogicalOperator_h = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S18>/Digital Output Write' */
  LATBbits.LATB2 = rtb_LogicalOperator_h;
}

/* Model step function for TID4 */
void AUAV3_AND_SLUGS_SENSOR_step4(void) /* Sample time: [0.1s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator1;

  /* S-Function (MCHP_Digital_Output_Read): '<S19>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S19>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S19>/Digital Output Read'
   */
  rtb_LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S19>/Digital Output Write' */
  LATBbits.LATB3 = rtb_LogicalOperator1;
}

/* Model step function for TID5 */
void AUAV3_AND_SLUGS_SENSOR_step5(void) /* Sample time: [0.2s, 0.0s] */
{
  /* DataStoreRead: '<S4>/Get RawGpsInt' */
  AUAV3_AND_SLUGS_SENSOR_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S4>/Get time1' */
  AUAV3_AND_SLUGS_SENSOR_B.Gettime1 =
    AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackGpsRawInt' */
  AUAV3_AND_SLUGS_SENSOR_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetRawGpsInt
    , AUAV3_AND_SLUGS_SENSOR_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data2' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackGpsRawInt
                  );

  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );

  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID6 */
void AUAV3_AND_SLUGS_SENSOR_step6(void) /* Sample time: [0.2s, 0.02s] */
{
  /* DataStoreRead: '<S4>/Get mlAirData' */
  AUAV3_AND_SLUGS_SENSOR_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S4>/Get time2' */
  AUAV3_AND_SLUGS_SENSOR_B.Gettime2 =
    AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackScaledPressure' */
  AUAV3_AND_SLUGS_SENSOR_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetmlAirData
    , AUAV3_AND_SLUGS_SENSOR_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackScaledPressure
                  );
}

/* Model step function for TID7 */
void AUAV3_AND_SLUGS_SENSOR_step7(void) /* Sample time: [0.2s, 0.04s] */
{
  /* DataStoreRead: '<S4>/Get mlSysStatus' */
  AUAV3_AND_SLUGS_SENSOR_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackSysStatus' */
  AUAV3_AND_SLUGS_SENSOR_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackSysStatus
                  );
}

/* Model step function for TID8 */
void AUAV3_AND_SLUGS_SENSOR_step8(void) /* Sample time: [0.2s, 0.06s] */
{
  /* S-Function (MCHP_C_function_Call): '<S4>/ParamInterfaceResponse' */
  AUAV3_AND_SLUGS_SENSOR_B.ParamInterfaceResponse = ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data5' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.ParamInterfaceResponse
                  );
}

/* Model step function for TID9 */
void AUAV3_AND_SLUGS_SENSOR_step9(void) /* Sample time: [0.2s, 0.08s] */
{
  /* S-Function (MCHP_C_function_Call): '<S4>/ParamInterfaceResponse1' */
  AUAV3_AND_SLUGS_SENSOR_B.ParamInterfaceResponse1 = MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data6' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.ParamInterfaceResponse1
                  );
}

/* Model step function for TID10 */
void AUAV3_AND_SLUGS_SENSOR_step10(void) /* Sample time: [0.25s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator2;

  /* S-Function (MCHP_Digital_Output_Read): '<S20>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S20>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S20>/Digital Output Read'
   */
  rtb_LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S20>/Digital Output Write' */
  LATBbits.LATB4 = rtb_LogicalOperator2;
}

/* Model step function for TID11 */
void AUAV3_AND_SLUGS_SENSOR_step11(void) /* Sample time: [0.5s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator3;

  /* S-Function (MCHP_Digital_Output_Read): '<S21>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S21>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S21>/Digital Output Read'
   */
  rtb_LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S21>/Digital Output Write' */
  LATBbits.LATB5 = rtb_LogicalOperator3;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackHeartBeat' */
  AUAV3_AND_SLUGS_SENSOR_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data1' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackHeartBeat
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
void AUAV3_AND_SLUGS_SENSOR_step(int_T tid)
{
  switch (tid) {
   case 0 :
    AUAV3_AND_SLUGS_SENSOR_step0();
    break;

   case 1 :
    AUAV3_AND_SLUGS_SENSOR_step1();
    break;

   case 2 :
    AUAV3_AND_SLUGS_SENSOR_step2();
    break;

   case 3 :
    AUAV3_AND_SLUGS_SENSOR_step3();
    break;

   case 4 :
    AUAV3_AND_SLUGS_SENSOR_step4();
    break;

   case 5 :
    AUAV3_AND_SLUGS_SENSOR_step5();
    break;

   case 6 :
    AUAV3_AND_SLUGS_SENSOR_step6();
    break;

   case 7 :
    AUAV3_AND_SLUGS_SENSOR_step7();
    break;

   case 8 :
    AUAV3_AND_SLUGS_SENSOR_step8();
    break;

   case 9 :
    AUAV3_AND_SLUGS_SENSOR_step9();
    break;

   case 10 :
    AUAV3_AND_SLUGS_SENSOR_step10();
    break;

   case 11 :
    AUAV3_AND_SLUGS_SENSOR_step11();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void AUAV3_AND_SLUGS_SENSOR_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[6] = 72;/* Sample time: [0.2s, 0.02s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[7] = 64;/* Sample time: [0.2s, 0.04s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[8] = 56;/* Sample time: [0.2s, 0.06s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[9] = 48;/* Sample time: [0.2s, 0.08s] */

  /* block I/O */
  (void) memset(((void *) &AUAV3_AND_SLUGS_SENSOR_B), 0,
                sizeof(BlockIO_AUAV3_AND_SLUGS_SENSO_T));

  /* states (dwork) */
  (void) memset((void *)&AUAV3_AND_SLUGS_SENSOR_DWork, 0,
                sizeof(D_Work_AUAV3_AND_SLUGS_SENSOR_T));

  /* exported global states */
  mlGpsData = AUAV3_AND_SLUGS_SENSOR_rtZmavlink_gps_raw_int_t;
  mlSysStatus = AUAV3_AND_SLUGS_SENSOR_rtZmavlink_sys_status_t;
  mlAirData = AUAV3_AND_SLUGS_SENSOR_rtZmavlink_scaled_pressure_t;
  mlRawPressureData = AUAV3_AND_SLUGS_SENSOR_rtZmavlink_raw_pressure_t;
  MPU_T = 0U;

  /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

  /* Start for S-Function (MCHP_MCU_LOAD): '<Root>/MCU Load' */
  TMR2 = 0;                            /* Initialize Timer 2 Value to 0.  Timer 2 is enabled only when the mcu is not idle */

  /* Start for S-Function (MCHP_IC): '<S10>/Input Capture RC Receiver1' */
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

  /* Start for S-Function (MCHP_OC_HW): '<S10>/Output Compare - HW Drive Servo motor' */
  /* OCxCON1[4108, 4108]   PulseTrig1*/
  OC1CON2 = 0x9F;
  OC1RS = 0x6689;
  OC1R = 1;
  OC2CON2 = 0x9F;
  OC2RS = 0x6689;
  OC2R = 1;

  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S7>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0xA5;                      /* I2C clock = 399772  (400000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;

  /* Start for S-Function (MCHP_BUS_SPI): '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
  SPI1CON1 = 0x0474;
  SPI1CON2 = 0x01;
  SPI1STAT = 0x8014;
  _SPI1IP = 5;                         /* Set SPI Interrupt Priority */
  _SPI1IF = 0;                         /* Reset interrupt Flag */
  _SPI1IE = 1;                         /* Enable Interrupts */

  /* MCHP_Digital_Output_Write Block: '<Root>/Drive SPI SSL High1' */
  LATBbits.LATB2 = true;

  /* Start for S-Function (MCHP_IC): '<S1>/Input Capture' */
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

  /* Set-up Input Capture Interruption */
  _IC1IF = 0;
  _IC1IP = 6;
  _IC1IE = 1;
  _IC2IF = 0;
  _IC2IP = 6;
  _IC2IE = 1;
  _IC3IF = 0;
  _IC3IP = 6;
  _IC3IE = 1;
  _IC4IF = 0;
  _IC4IP = 6;
  _IC4IE = 1;
  _IC5IF = 0;
  _IC5IP = 6;
  _IC5IE = 1;

  /* Start for S-Function (MCHP_MCU_LOAD): '<S154>/Calculus Time Step1' */
  TMR2 = 0;                            /* Initialize Timer 2 Value to 0.  Timer 2 is enabled only when the mcu is not idle */

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

  /* InitializeConditions for Delay: '<S7>/Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

  /* InitializeConditions for MATLAB Function: '<S193>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction);

  /* InitializeConditions for MATLAB Function: '<S195>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

  /* InitializeConditions for MATLAB Function: '<S196>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_a);

  /* InitializeConditions for MATLAB Function: '<S194>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_j);

  /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_p);

  /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

  /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

  /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_ph);

  /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_h);

  /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_d);

  /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_d);

  /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_d);

  /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_f);

  /* InitializeConditions for MATLAB Function: '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_m);

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
