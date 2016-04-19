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
 * Model version                        : 1.188
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Tue Apr 19 10:47:35 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Tue Apr 19 10:47:36 2016
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
int16_T div_s16s32_round(int32_T numerator, int32_T denominator)
{
  int16_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  uint16_T quotientNeedsNegation;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int16_T : MIN_int16_T;

    /* Divide by zero handler */
  } else {
    absNumerator = (uint32_T)(numerator >= 0 ? numerator : -numerator);
    absDenominator = (uint32_T)(denominator >= 0 ? denominator : -denominator);
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absDenominator == 0 ? MAX_uint32_T : absNumerator /
      absDenominator;
    absNumerator %= absDenominator;
    absNumerator <<= 1;
    if (absNumerator >= absDenominator) {
      tempAbsQuotient++;
    }

    quotient = quotientNeedsNegation ? (int16_T)-(int32_T)tempAbsQuotient :
      (int16_T)tempAbsQuotient;
  }

  return quotient;
}

uint32_T div_u32_round(uint32_T numerator, uint32_T denominator)
{
  uint32_T quotient;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    quotient = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    tempAbsQuotient = denominator == 0 ? MAX_uint32_T : numerator / denominator;
    numerator %= denominator;
    if (numerator > 2147483647UL) {
      tempAbsQuotient++;
    } else {
      numerator <<= 1;
      if (numerator >= denominator) {
        tempAbsQuotient++;
      }
    }

    quotient = tempAbsQuotient;
  }

  return quotient;
}

uint32_T div_repeat_u32_round(uint32_T numerator, uint32_T denominator, uint16_T
  nRepeatSub)
{
  uint32_T quotient;
  uint16_T iRepeatSub;
  uint8_T numeratorExtraBit;
  if (denominator == 0) {
    quotient = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    quotient = denominator == 0 ? MAX_uint32_T : numerator / denominator;
    numerator %= denominator;
    for (iRepeatSub = 0; iRepeatSub < nRepeatSub; iRepeatSub++) {
      numeratorExtraBit = (numerator >= 2147483648UL);
      numerator <<= 1;
      quotient <<= 1;
      if (numeratorExtraBit || (numerator >= denominator)) {
        quotient++;
        numerator -= denominator;
      }
    }

    numeratorExtraBit = (numerator >= 2147483648UL);
    numerator <<= 1;
    if (numeratorExtraBit || (numerator >= denominator)) {
      quotient++;
    }
  }

  return quotient;
}

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
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_u32(a, b, &u32_chi, &u32_clo);
  u32_clo = u32_chi << 17UL | u32_clo >> 15UL;
  return u32_clo;
}

void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T absIn;
  uint32_T absIn_0;
  uint32_T in0Lo;
  uint32_T in0Hi;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn = (uint32_T)(in0 < 0L ? -in0 : in0);
  absIn_0 = (uint32_T)(in1 < 0L ? -in1 : in1);
  in0Hi = absIn >> 16UL;
  in0Lo = absIn & 65535UL;
  in1Hi = absIn_0 >> 16UL;
  absIn = absIn_0 & 65535UL;
  productHiLo = in0Hi * absIn;
  productLoHi = in0Lo * in1Hi;
  absIn *= in0Lo;
  absIn_0 = 0UL;
  in0Lo = (productLoHi << 16UL) + absIn;
  if (in0Lo < absIn) {
    absIn_0 = 1UL;
  }

  absIn = in0Lo;
  in0Lo += productHiLo << 16UL;
  if (in0Lo < absIn) {
    absIn_0++;
  }

  absIn = (((productLoHi >> 16UL) + (productHiLo >> 16UL)) + in0Hi * in1Hi) +
    absIn_0;
  if (!((in0 == 0L) || ((in1 == 0L) || ((in0 > 0L) == (in1 > 0L))))) {
    absIn = ~absIn;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0UL) {
      absIn++;
    }
  }

  *ptrOutBitsHi = absIn;
  *ptrOutBitsLo = in0Lo;
}

uint32_T mul_u32_s32_s32_sat(int32_T a, int32_T b)
{
  uint32_T result;
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if ((int32_T)u32_chi >= 0L) {
    if (u32_chi) {
      result = MAX_uint32_T;
    } else {
      result = u32_clo;
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
  uint32_T absIn;
  uint32_T in0Hi;
  uint32_T in1Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn = (uint32_T)(in0 < 0L ? -in0 : in0);
  in0Hi = absIn >> 16UL;
  absIn &= 65535UL;
  in1Hi = in1 >> 16UL;
  in1Lo = in1 & 65535UL;
  productHiLo = in0Hi * in1Lo;
  productLoHi = absIn * in1Hi;
  absIn *= in1Lo;
  in1Lo = 0UL;
  outBitsLo = (productLoHi << 16UL) + absIn;
  if (outBitsLo < absIn) {
    in1Lo = 1UL;
  }

  absIn = outBitsLo;
  outBitsLo += productHiLo << 16UL;
  if (outBitsLo < absIn) {
    in1Lo++;
  }

  absIn = (((productLoHi >> 16UL) + (productHiLo >> 16UL)) + in0Hi * in1Hi) +
    in1Lo;
  if (!((in1 == 0UL) || (in0 >= 0L))) {
    absIn = ~absIn;
    outBitsLo = ~outBitsLo;
    outBitsLo++;
    if (outBitsLo == 0UL) {
      absIn++;
    }
  }

  *ptrOutBitsHi = absIn;
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

uint32_T mul_u32_u32_u32_sr11_round(uint32_T a, uint32_T b)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  boolean_T roundup;
  mul_wide_u32(a, b, &u32_chi, &u32_clo);
  roundup = ((u32_clo & 1024UL) != 0UL);
  u32_clo = (u32_chi << 21UL | u32_clo >> 11UL) + (uint32_T)roundup;
  return u32_clo;
}

uint32_T mul_u32_s32_u32_sr15(int32_T a, uint32_T b)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_su32(a, b, &u32_chi, &u32_clo);
  u32_clo = u32_chi << 17UL | u32_clo >> 15UL;
  return u32_clo;
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
  int16_T loop_ub;
  int16_T loop_ub_0;
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
      loop_ub = nc - 1;
      for (i1 = nb + 1; i1 <= loop_ub; i1++) {
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
      loop_ub_0 = nc - 1;
      for (i1 = nb; i1 <= loop_ub_0; i1++) {
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
  uint32_T a;
  uint32_T a_0;
  uint32_T b;
  uint32_T w;
  uint32_T w_0;
  uint32_T cb;
  int16_T loop_ub;
  int16_T loop_ub_0;
  int16_T loop_ub_1;

  /* Initialize output to zero */
  loop_ub = n - 1;
  for (k = 0; k <= loop_ub; k++) {
    y[k] = 0UL;
  }

  loop_ub_1 = n1 - 1;
  for (i = 0; i <= loop_ub_1; i++) {
    cb = 0UL;
    u1i = u1[i];
    a = u1i >> 16U;
    a_0 = u1i & 65535UL;
    k = n - i;
    nj = n2 <= k ? n2 : k;
    k = i;
    loop_ub_0 = nj - 1;
    for (j = 0; j <= loop_ub_0; j++) {
      yk = y[k];
      u1i = u2[j];
      b = u1i >> 16U;
      u1i &= 65535UL;
      w = a * u1i;
      w_0 = a_0 * b;
      yk += cb;
      cb = yk < cb ? 1UL : 0UL;
      u1i *= a_0;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      u1i = w << 16U;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      u1i = w_0 << 16U;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      y[k] = yk;
      cb += w >> 16U;
      cb += w_0 >> 16U;
      cb += a * b;
      k++;
    }

    if (k < n) {
      y[k] = cb;
    }
  }
}

/*
 * Start for enable system:
 *    '<S19>/Enabled Subsystem'
 *    '<S82>/Enabled Subsystem'
 */
void AUA_EnabledSubsystem_Start(rtB_EnabledSubsystem_AUAV3_AN_T *localB)
{
  /* VirtualOutportStart for Outport: '<S27>/Out1' */
  localB->In1 = 143.543F;
}

/*
 * Output and update for enable system:
 *    '<S19>/Enabled Subsystem'
 *    '<S82>/Enabled Subsystem'
 */
void AUAV3_AND_EnabledSubsystem(boolean_T rtu_0, real32_T rtu_1,
  rtB_EnabledSubsystem_AUAV3_AN_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S19>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S27>/Enable'
   */
  if (rtu_0) {
    /* Inport: '<S27>/In1' */
    localB->In1 = rtu_1;
  }

  /* End of Outputs for SubSystem: '<S19>/Enabled Subsystem' */
}

/*
 * Initial conditions for atomic system:
 *    '<S19>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S82>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW)
{
  localDW->aveCount = 1.0;
  localDW->tIni = 1.0;
}

/*
 * Output and update for atomic system:
 *    '<S19>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S82>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW)
{
  /* MATLAB Function 'Read Barometer BMP180/Baro Altimeter/Enables/Disables the Computation of  initial Baro Bias': '<S28>:1' */
  if (localDW->aveCount < 2500.0) {
    /* '<S28>:1:11' */
    /* '<S28>:1:12' */
    localDW->aveCount = localDW->aveCount + 1.0;
  }

  if (localDW->aveCount == 2500.0) {
    /* '<S28>:1:15' */
    /* '<S28>:1:16' */
    localDW->tIni = 0.0;

    /* '<S28>:1:17' */
    localDW->aveCount = localDW->aveCount + 1.0;
  }

  /* '<S28>:1:20' */
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
 *    '<S56>/Embedded MATLAB Function'
 *    '<S56>/Embedded MATLAB Function1'
 *    '<S56>/Embedded MATLAB Function2'
 *    '<S57>/Embedded MATLAB Function'
 *    '<S57>/Embedded MATLAB Function1'
 *    '<S57>/Embedded MATLAB Function2'
 *    '<S58>/Embedded MATLAB Function'
 *    '<S58>/Embedded MATLAB Function1'
 *    '<S58>/Embedded MATLAB Function2'
 *    '<S89>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW)
{
  localDW->a_not_empty = FALSE;
}

/*
 * Output and update for atomic system:
 *    '<S56>/Embedded MATLAB Function'
 *    '<S56>/Embedded MATLAB Function1'
 *    '<S56>/Embedded MATLAB Function2'
 *    '<S57>/Embedded MATLAB Function'
 *    '<S57>/Embedded MATLAB Function1'
 *    '<S57>/Embedded MATLAB Function2'
 *    '<S58>/Embedded MATLAB Function'
 *    '<S58>/Embedded MATLAB Function1'
 *    '<S58>/Embedded MATLAB Function2'
 *    '<S89>/Embedded MATLAB Function'
 *    ...
 */
void AUA_EmbeddedMATLABFunction(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_AU_T *localB, rtDW_EmbeddedMATLABFunction_A_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S62>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S62>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S62>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S62>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = TRUE;

    /* '<S62>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S62>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S62>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S62>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S62>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S62>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S51>/Embedded MATLAB Function'
 *    '<S51>/Embedded MATLAB Function1'
 *    '<S51>/Embedded MATLAB Function2'
 *    '<S51>/Embedded MATLAB Function3'
 */
void A_EmbeddedMATLABFunction_k(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_d_T *localB)
{
  /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function': '<S74>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S74>:1:5' */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/* Model step function for TID0 */
void AUAV3_AND_SLUGS_SENSOR_step0(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion5;
  real_T rtb_DataTypeConversion3;
  real_T rtb_DataTypeConversion19;
  real_T rtb_DataTypeConversion6;
  real_T rtb_DataTypeConversion2[3];
  real_T rtb_DataTypeConversion1[3];
  real_T rtb_DataTypeConversion5_l[3];
  real_T rtb_DataTypeConversion;
  real_T rtb_DataTypeConversion_d;
  real_T rtb_DiscreteZeroPole;
  real_T rtb_DiscreteZeroPole_k;
  real32_T rtb_Sum1;
  real32_T rtb_Sum1_f;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_EnableHILfromControlMCU1;
  boolean_T rtb_LogicalOperator_f;
  uint32_T rtb_FixPtSum1;
  uint16_T rtb_Sum1_a2;
  int16_T rtb_DataTypeConversion1_a;
  uint16_T rtb_BitwiseOperator_e;
  int16_T rtb_Gain21_h;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_DataTypeConversion2_b;
  int16_T rtb_DataTypeConversion2_o;
  int16_T rtb_DataTypeConversion3_m;
  int16_T rtb_DataTypeConversion4;
  int16_T rtb_DataTypeConversion5_b;
  boolean_T rtb_LogicalOperator_o;
  int16_T rtb_Switch_o[13];
  real32_T rtb_Sum;
  uint8_T rtb_Compare;
  real32_T rtb_Sum_e;
  uint32_T rtb_Sum6;
  boolean_T rtb_LogicalOperator_g;
  real32_T Sum;
  real_T tmp;
  uint32_T qY;
  uint32_T qY_0;
  uint64m_T tmp_0;
  uint64m_T tmp_1;

  /* UnitDelay: '<S2>/Output' */
  rtb_FixPtSum1 = AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE + 1UL;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S2>/Output'
   */
  rtb_Sum6 = 3276800000UL;
  uMultiWordMul(&rtb_Sum6, 1, &AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE, 1,
                &tmp_1.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_1.chunks[0U], 2, 15U, &tmp_0.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec = MultiWord2uLong
    (&tmp_0.chunks[0U]);

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  AUAV3_AND_SLUGS_SENSOR_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = AUAV3_AND_SLUGS_SENSOR_B.U3CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S6>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
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

  /* Sum: '<S21>/SumA21' incorporates:
   *  DataTypeConversion: '<S23>/Data Type Conversion1'
   *  DataTypeConversion: '<S23>/Data Type Conversion3'
   *  Delay: '<S21>/Delay11'
   *  Gain: '<S21>/a(2)(1)'
   *  Gain: '<S21>/s(1)'
   *  Gain: '<S23>/Gain'
   *  S-Function (sfix_bitop): '<S23>/Bitwise Operator'
   */
  rtb_B4 = (uint32_T)((uint16_T)
                      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[0]
                      << 8 | (uint16_T)
                      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadTConv[1])
    * 31949UL >> 5;
  qY_0 = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE);
  if (qY_0 < rtb_B4) {
    qY_0 = MAX_uint32_T;
  }

  /* Sum: '<S21>/SumB21' incorporates:
   *  Delay: '<S21>/Delay11'
   *  Sum: '<S21>/SumA21'
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

  rtb_Sum1_a2 = (uint16_T)(qY >> 16);

  /* End of Sum: '<S21>/SumB21' */

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S6>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
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

  /* Sum: '<S22>/SumA21' incorporates:
   *  DataTypeConversion: '<S24>/Data Type Conversion1'
   *  DataTypeConversion: '<S24>/Data Type Conversion3'
   *  Delay: '<S22>/Delay11'
   *  Gain: '<S22>/a(2)(1)'
   *  Gain: '<S22>/s(1)'
   *  Gain: '<S24>/Gain'
   *  S-Function (sfix_bitop): '<S24>/Bitwise Operator'
   */
  rtb_B4 = (uint32_T)((uint16_T)
                      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[0]
                      << 8 | (uint16_T)
                      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180ReadPConv[1])
    * 31949UL >> 5;
  qY = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j);
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S22>/SumB21' incorporates:
   *  Delay: '<S22>/Delay11'
   *  Sum: '<S22>/SumA21'
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

  rtb_BitwiseOperator_e = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S22>/SumB21' */

  /* DataStoreWrite: '<S6>/Update RawPressure' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion7'
   *  DataTypeConversion: '<S6>/Data Type Conversion8'
   */
  mlRawPressureData.press_abs = (int16_T)rtb_Sum1_a2;
  mlRawPressureData.temperature = (int16_T)rtb_BitwiseOperator_e;

  /* Outputs for Enabled SubSystem: '<S6>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S20>/Enable'
   */
  /* Delay: '<S6>/Delay' */
  if (AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S20>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
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

    /* DataTypeConversion: '<S20>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S37>/Data Type Conversion1'
     *  DataTypeConversion: '<S37>/Data Type Conversion3'
     *  Gain: '<S37>/Gain'
     *  S-Function (sfix_bitop): '<S37>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition3 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S20>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S36>/Data Type Conversion1'
     *  DataTypeConversion: '<S36>/Data Type Conversion3'
     *  Gain: '<S36>/Gain'
     *  S-Function (sfix_bitop): '<S36>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S20>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S38>/Data Type Conversion1'
     *  DataTypeConversion: '<S38>/Data Type Conversion3'
     *  Gain: '<S38>/Gain'
     *  S-Function (sfix_bitop): '<S38>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S20>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S42>/Data Type Conversion1'
     *  DataTypeConversion: '<S42>/Data Type Conversion3'
     *  Gain: '<S42>/Gain'
     *  S-Function (sfix_bitop): '<S42>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S20>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S43>/Data Type Conversion1'
     *  DataTypeConversion: '<S43>/Data Type Conversion3'
     *  Gain: '<S43>/Gain'
     *  S-Function (sfix_bitop): '<S43>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S20>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S34>/Data Type Conversion1'
     *  DataTypeConversion: '<S34>/Data Type Conversion3'
     *  Gain: '<S34>/Gain'
     *  S-Function (sfix_bitop): '<S34>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S20>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S35>/Data Type Conversion1'
     *  DataTypeConversion: '<S35>/Data Type Conversion3'
     *  Gain: '<S35>/Gain'
     *  S-Function (sfix_bitop): '<S35>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S39>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S39>/Data Type Conversion1'
     *  DataTypeConversion: '<S39>/Data Type Conversion3'
     *  Gain: '<S39>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition5 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S40>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S40>/Data Type Conversion1'
     *  DataTypeConversion: '<S40>/Data Type Conversion3'
     *  Gain: '<S40>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition6 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S41>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S41>/Data Type Conversion1'
     *  DataTypeConversion: '<S41>/Data Type Conversion3'
     *  Gain: '<S41>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition7 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S6>/Delay' */
  /* End of Outputs for SubSystem: '<S6>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S18>/Sum' */
  rtb_Sum1_a2 -= AUAV3_AND_SLUGS_SENSOR_B.RateTransition7;

  /* Product: '<S18>/Product' */
  rtb_Sum6 = (uint32_T)rtb_Sum1_a2 * (uint32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition6;
  rtb_Sum1_a2 = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) != 0U) +
    (rtb_Sum6 >> 15));

  /* Sum: '<S18>/Sum2' incorporates:
   *  Product: '<S18>/Product1'
   *  Sum: '<S18>/Sum1'
   */
  rtb_DataTypeConversion1_a = div_s16s32_round((int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 << 11, (int32_T)(rtb_Sum1_a2 +
    (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition12)) + (int16_T)rtb_Sum1_a2;

  /* Gain: '<S6>/Gain21' incorporates:
   *  Bias: '<S18>/Bias'
   */
  rtb_Gain21_h = (int16_T)((int32_T)(rtb_DataTypeConversion1_a + 8) * 5L >> 3);

  /* MATLAB Function: '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S19>/Zero Out Height' incorporates:
   *  EnablePort: '<S30>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Sum: '<S30>/Sum' incorporates:
     *  Constant: '<S19>/Constant5'
     *  Delay: '<S30>/Integer Delay'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Sum_c = 143.543F -
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE_k;
  }

  /* End of Outputs for SubSystem: '<S19>/Zero Out Height' */

  /* Bias: '<S18>/Bias1' */
  rtb_DataTypeConversion1_a -= 4000;

  /* Math: '<S18>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_DataTypeConversion1_a * (int32_T)
    rtb_DataTypeConversion1_a;

  /* Sum: '<S18>/Sum6' incorporates:
   *  Bias: '<S18>/Bias2'
   *  Product: '<S18>/Product2'
   *  Product: '<S18>/Product3'
   *  Sum: '<S18>/Sum3'
   *  Sum: '<S18>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_BitwiseOperator_e - (uint32_T)((((int32_T)((int16_T)
    ((int32_T)rtb_DataTypeConversion1_a * (int32_T)
     AUAV3_AND_SLUGS_SENSOR_B.RateTransition3 >> 11) + (int16_T)
    mul_s32_s32_s32_sr23((int32_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition9,
    rtb_Bias5)) + ((int32_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition2 << 2)) + 2L)
    >> 2);

  /* Product: '<S18>/Product6' incorporates:
   *  Bias: '<S18>/Bias3'
   *  Bias: '<S18>/Bias4'
   *  Gain: '<S18>/Gain1'
   *  Product: '<S18>/Product4'
   *  Product: '<S18>/Product5'
   *  Sum: '<S18>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15((int32_T)((((int16_T)((int32_T)
    rtb_DataTypeConversion1_a * (int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 >> 13) + (int16_T)
    mul_s32_s32_s32_sr28(rtb_Bias5, (int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition8)) + 2) >> 2) + 32768L, (uint32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition5);

  /* Switch: '<S18>/Switch' incorporates:
   *  Gain: '<S18>/Gain15'
   *  Gain: '<S18>/Gain22'
   *  Product: '<S18>/Product7'
   *  Product: '<S18>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S18>/Switch' */

  /* Gain: '<S18>/Gain16' */
  rtb_Bias5 = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >>
    8));

  /* Sum: '<S18>/Sum8' incorporates:
   *  Bias: '<S18>/Bias5'
   *  Gain: '<S18>/Gain17'
   *  Gain: '<S18>/Gain19'
   *  Gain: '<S18>/Gain21'
   *  Math: '<S18>/Math Function2'
   *  Sum: '<S18>/Sum7'
   */
  rtb_Bias5 = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat
    (rtb_Bias5, rtb_Bias5)) + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >>
               4) + (int32_T)rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S19>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S29>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* DataTypeConversion: '<S29>/Data Type Conversion' */
    rtb_DataTypeConversion_d = (real_T)rtb_Bias5;

    /* DiscreteZeroPole: '<S32>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_k = 0.014778325123152709*rtb_DataTypeConversion_d;
      rtb_DiscreteZeroPole_k += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j;
    }

    /* Saturate: '<S29>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S29>/Data Type Conversion1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e = (real32_T)rtb_DiscreteZeroPole_k >=
      120000.0F ? 120000.0F : (real32_T)rtb_DiscreteZeroPole_k <= 80000.0F ?
      80000.0F : (real32_T)rtb_DiscreteZeroPole_k;

    /* Update for DiscreteZeroPole: '<S32>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j =
        rtb_DataTypeConversion_d + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j;
    }
  }

  /* End of Outputs for SubSystem: '<S19>/Initial Baro Bias' */

  /* Product: '<S26>/Divide' incorporates:
   *  Sum: '<S26>/Sum2'
   */
  rtb_DataTypeConversion2_b = ((real32_T)rtb_Bias5 -
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e) / AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e;

  /* Sum: '<S26>/Sum1' incorporates:
   *  Constant: '<S19>/Constant5'
   *  Constant: '<S26>/Constant2'
   *  Constant: '<S26>/Constant3'
   *  Constant: '<S26>/Constant4'
   *  Constant: '<S26>/Constant5'
   *  Gain: '<S31>/Unit Conversion'
   *  Product: '<S26>/Divide1'
   *  Product: '<S26>/Divide2'
   *  Product: '<S26>/Divide3'
   *  Product: '<S26>/Divide4'
   *  Sum: '<S26>/Sum3'
   */
  Sum = ((rtb_DataTypeConversion2_b * rtb_DataTypeConversion2_b * 0.093502529F +
          rtb_DataTypeConversion2_b * -0.188893303F) + 2.18031291E-5F) *
    145473.5F * 0.3048F + 143.543F;

  /* Sum: '<S19>/Sum1' */
  rtb_Sum1 = AUAV3_AND_SLUGS_SENSOR_B.Sum_c + Sum;

  /* Logic: '<S19>/Logical Operator' */
  rtb_LogicalOperator =
    !(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut != 0.0);

  /* Outputs for Enabled SubSystem: '<S19>/Enabled Subsystem' */
  AUAV3_AND_EnabledSubsystem(rtb_LogicalOperator, rtb_Sum1,
    &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem);

  /* End of Outputs for SubSystem: '<S19>/Enabled Subsystem' */

  /* DataStoreWrite: '<S6>/Update ScaledPressure' */
  mlAirData.temperature = rtb_Gain21_h;
  mlAirData.press_abs = AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem.In1;

  /* S-Function (MCHP_BUS_SPI): '<S7>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
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

  /* DataStoreWrite: '<S7>/Update MPU_T' */
  MPU_T = AUAV3_AND_SLUGS_SENSOR_B.U1CH4;

  /* DataTypeConversion: '<S7>/Data Type Conversion' */
  rtb_Gain21_h = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[0];

  /* DataTypeConversion: '<S7>/Data Type Conversion1' */
  rtb_DataTypeConversion1_a = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[1];

  /* DataTypeConversion: '<S7>/Data Type Conversion2' */
  rtb_DataTypeConversion2_o = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[2];

  /* DataTypeConversion: '<S7>/Data Type Conversion3' */
  rtb_DataTypeConversion3_m = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0];

  /* DataTypeConversion: '<S7>/Data Type Conversion4' */
  rtb_DataTypeConversion4 = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1];

  /* DataTypeConversion: '<S7>/Data Type Conversion5' */
  rtb_DataTypeConversion5_b = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2];

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S7>/BUS I2C Read HMC5883 Magn (50 Hz)' */
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

  /* DataStoreWrite: '<S7>/Update Raw IMU DATA' incorporates:
   *  DataTypeConversion: '<S44>/Data Type Conversion3'
   *  DataTypeConversion: '<S45>/Data Type Conversion3'
   *  DataTypeConversion: '<S46>/Data Type Conversion3'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xacc = rtb_Gain21_h;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.yacc = rtb_DataTypeConversion1_a;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zacc = rtb_DataTypeConversion2_o;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xgyro = rtb_DataTypeConversion3_m;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.ygyro = rtb_DataTypeConversion4;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zgyro = rtb_DataTypeConversion5_b;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xmag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.ymag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[3];
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.zmag =
    AUAV3_AND_SLUGS_SENSOR_B.BUSI2CReadHMC5883Magn50Hz[5];

  /* S-Function (MCHP_Digital_Input): '<S8>/Enable HIL from  Control MCU1' */

  /* MCHP_Digital_Input Block: <S8>/Enable HIL from  Control MCU1/Output */
  rtb_EnableHILfromControlMCU1 = PORTDbits.RD2;/* Read pin D2 */

  /* Outputs for Enabled SubSystem: '<S8>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S47>/Enable'
   */
  if (rtb_EnableHILfromControlMCU1) {
    /* S-Function (MCHP_C_function_Call): '<S47>/Data from HIL [hil.c]2' */
    hilRead(
            AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2
            );

    /* S-Function (MCHP_C_function_Call): '<S47>/HIL Messages  Parser//Decoder [hil.c]1' */
    protDecodeHil(
                  AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2
                  );

    /* S-Function (MCHP_C_function_Call): '<S47>/HIL Raw Readings [hil.c]1' */
    hil_getRawRead(
                   AUAV3_AND_SLUGS_SENSOR_B.HILMessagesParserDecoderhilc1
                   );
  }

  /* End of Outputs for SubSystem: '<S8>/Raw HIL  Readings' */

  /* Logic: '<S48>/Logical Operator' */
  rtb_LogicalOperator_o = !rtb_EnableHILfromControlMCU1;

  /* Outputs for Enabled SubSystem: '<S48>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S51>/Enable'
   */
  if (rtb_LogicalOperator_o) {
    /* MATLAB Function: '<S51>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_k(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_kd);

    /* DataTypeConversion: '<S51>/Data Type Conversion' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_kd.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_Gain21_h = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)(uint16_T)tmp;

    /* End of DataTypeConversion: '<S51>/Data Type Conversion' */

    /* MATLAB Function: '<S51>/Embedded MATLAB Function1' */
    A_EmbeddedMATLABFunction_k(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_d);

    /* DataTypeConversion: '<S51>/Data Type Conversion1' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_d.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion2_o = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S51>/Data Type Conversion1' */

    /* MATLAB Function: '<S51>/Embedded MATLAB Function2' */
    A_EmbeddedMATLABFunction_k(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_o);

    /* DataTypeConversion: '<S51>/Data Type Conversion2' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_o.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion3_m = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S51>/Data Type Conversion2' */

    /* MATLAB Function: '<S51>/Embedded MATLAB Function3' */
    A_EmbeddedMATLABFunction_k(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled5,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3);

    /* DataTypeConversion: '<S51>/Data Type Conversion3' */
    tmp = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3.y);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 65536.0);
    }

    rtb_DataTypeConversion4 = tmp < 0.0 ? -(int16_T)(uint16_T)-tmp : (int16_T)
      (uint16_T)tmp;

    /* End of DataTypeConversion: '<S51>/Data Type Conversion3' */

    /* MATLAB Function: '<S51>/myMux Fun' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S80>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S80>:1:5' */
    AUAV3_AND_SLUGS_SENSOR_B.y_p[0] = rtb_Gain21_h;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[1] = rtb_DataTypeConversion2_o;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[2] = rtb_DataTypeConversion3_m;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[3] = rtb_DataTypeConversion4;

    /* S-Function (MCHP_C_function_Call): '<S51>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     AUAV3_AND_SLUGS_SENSOR_B.y_p
                     );

    /* S-Function (MCHP_C_function_Call): '<S51>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1
                );

    /* MATLAB Function: '<S51>/myMux Fun4' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S81>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S81>:1:5' */
    AUAV3_AND_SLUGS_SENSOR_B.y_h[0] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[0];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[1] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[1];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[2] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[2];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[3] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[3];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[4] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[4];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[5] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[5];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[6] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[6];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[7] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[7];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[8] =
      AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1[8];
    AUAV3_AND_SLUGS_SENSOR_B.y_h[9] = rtb_Gain21_h;
    AUAV3_AND_SLUGS_SENSOR_B.y_h[10] = rtb_DataTypeConversion2_o;
    AUAV3_AND_SLUGS_SENSOR_B.y_h[11] = rtb_DataTypeConversion3_m;
    AUAV3_AND_SLUGS_SENSOR_B.y_h[12] = rtb_DataTypeConversion4;

    /* S-Function (MCHP_C_function_Call): '<S51>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    isGPSNovatel(
                 );

    /* Logic: '<S51>/Logical Operator' */
    rtb_LogicalOperator_g =
      !AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1;

    /* Outputs for Enabled SubSystem: '<S51>/if GPS is Novatel' incorporates:
     *  EnablePort: '<S78>/Enable'
     */
    if (AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1) {
      /* S-Function (MCHP_C_function_Call): '<S78>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]1' */
      getGpsMainData(
                     &AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdat_d
                     );

      /* S-Function (MCHP_C_function_Call): '<S78>/Read the Raw Data from GPS [gpsPort.c]1' */
      getGPSRawData(
                    &AUAV3_AND_SLUGS_SENSOR_B.ReadtheRawDatafromGPSgpsPortc1
                    );

      /* S-Function (MCHP_C_function_Call): '<S78>/Parse the GPS RAW Data [gps.c//novatel.c]1' */
      gpsParse(
               &AUAV3_AND_SLUGS_SENSOR_B.ReadtheRawDatafromGPSgpsPortc1
               );
    }

    /* End of Outputs for SubSystem: '<S51>/if GPS is Novatel' */

    /* Outputs for Enabled SubSystem: '<S51>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S79>/Enable'
     */
    if (rtb_LogicalOperator_g) {
      /* S-Function (MCHP_C_function_Call): '<S79>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdatet
                          );
    }

    /* End of Outputs for SubSystem: '<S51>/if GPS is Ublox' */
  }

  /* End of Outputs for SubSystem: '<S48>/If no HIL then Read all the Sensors' */

  /* Switch: '<S48>/Switch' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 13; rtb_Gain21_h++) {
    if (rtb_EnableHILfromControlMCU1) {
      rtb_Switch_o[rtb_Gain21_h] =
        AUAV3_AND_SLUGS_SENSOR_B.HILRawReadingshilc1[rtb_Gain21_h];
    } else {
      rtb_Switch_o[rtb_Gain21_h] = AUAV3_AND_SLUGS_SENSOR_B.y_h[rtb_Gain21_h];
    }
  }

  /* End of Switch: '<S48>/Switch' */

  /* DataTypeConversion: '<S53>/Data Type Conversion5' */
  rtb_DataTypeConversion5 = (real_T)rtb_Switch_o[12];

  /* MATLAB Function: '<S89>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion5, 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction);

  /* Sum: '<S88>/Sum' incorporates:
   *  Constant: '<S88>/Bias'
   *  Constant: '<S88>/Gains'
   *  Product: '<S88>/Divide'
   */
  rtb_Sum = (real32_T)(1.5112853050231934 *
                       AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction.y) +
    -1605.28198F;

  /* RelationalOperator: '<S101>/Compare' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_Compare = (rtb_Sum < -130.0F);

  /* Outputs for Enabled SubSystem: '<S86>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S102>/Enable'
   */
  /* Logic: '<S86>/Logical Operator' */
  if (!(rtb_Compare != 0)) {
    /* Sum: '<S102>/Sum2' incorporates:
     *  Constant: '<S102>/Mean Temperature for Calibration'
     *  Constant: '<S102>/gains'
     *  Product: '<S102>/Divide1'
     *  Sum: '<S102>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = (real32_T)rtb_Switch_o[10] - (rtb_Sum -
      293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S86>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S86>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S86>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S103>/Enable'
   */
  if (rtb_Compare > 0) {
    /* Sum: '<S103>/Add' incorporates:
     *  Constant: '<S103>/Constant'
     *  Constant: '<S103>/Mean Temperature for Calibration'
     *  Constant: '<S103>/gains'
     *  Product: '<S103>/Divide1'
     *  Sum: '<S103>/Sum1'
     *  Sum: '<S103>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = ((real32_T)rtb_Switch_o[10] - (rtb_Sum -
      -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S86>/Lo Temp Compensation' */

  /* DataTypeConversion: '<S53>/Data Type Conversion3' */
  rtb_DataTypeConversion3 = AUAV3_AND_SLUGS_SENSOR_B.Merge;

  /* MATLAB Function: '<S91>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion3, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_l,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_l);

  /* Sum: '<S84>/Sum' incorporates:
   *  Constant: '<S84>/Bias'
   *  Constant: '<S84>/Gains'
   *  DataTypeConversion: '<S53>/Data Type Conversion4'
   *  Product: '<S84>/Divide'
   */
  rtb_DataTypeConversion2_b = 1.05137849F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_l.y + -1005.87872F;

  /* Saturate: '<S53>/[0.001  maxDynPress]' */
  rtb_DataTypeConversion2_b = rtb_DataTypeConversion2_b >= 3000.0F ? 3000.0F :
    rtb_DataTypeConversion2_b <= 0.001F ? 0.001F : rtb_DataTypeConversion2_b;

  /* RelationalOperator: '<S104>/Compare' incorporates:
   *  Constant: '<S104>/Constant'
   */
  rtb_Compare = (rtb_Sum < -50.0F);

  /* Outputs for Enabled SubSystem: '<S87>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S105>/Enable'
   */
  /* Logic: '<S87>/Logical Operator' */
  if (!(rtb_Compare != 0)) {
    /* Sum: '<S105>/Add' incorporates:
     *  Constant: '<S105>/Constant'
     *  Constant: '<S105>/Mean Temperature for Calibration'
     *  Constant: '<S105>/gains'
     *  Product: '<S105>/Divide1'
     *  Sum: '<S105>/Sum1'
     *  Sum: '<S105>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_j = ((real32_T)rtb_Switch_o[9] - (rtb_Sum -
      347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S87>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S87>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S87>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S106>/Enable'
   */
  if (rtb_Compare > 0) {
    /* Sum: '<S106>/Sum2' incorporates:
     *  Constant: '<S106>/Mean Temperature for Calibration'
     *  Constant: '<S106>/gains'
     *  Product: '<S106>/Divide1'
     *  Sum: '<S106>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_j = (real32_T)rtb_Switch_o[9] - (rtb_Sum -
      -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S87>/Lo Temp Compensation' */

  /* DataTypeConversion: '<S53>/Data Type Conversion19' */
  rtb_DataTypeConversion19 = AUAV3_AND_SLUGS_SENSOR_B.Merge_j;

  /* MATLAB Function: '<S92>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion19, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

  /* Sum: '<S83>/Sum' incorporates:
   *  Constant: '<S83>/Bias'
   *  Constant: '<S83>/Gains'
   *  DataTypeConversion: '<S53>/Data Type Conversion1'
   *  Product: '<S83>/Divide'
   */
  rtb_Sum_e = 27.127F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o.y + 9444.44434F;

  /* Outputs for Enabled SubSystem: '<S48>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S52>/Enable'
   */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun': '<S54>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S54>:1:5' */
  if (rtb_LogicalOperator_o) {
    /* Inport: '<S52>/AirData' incorporates:
     *  MATLAB Function: '<S48>/myMux Fun'
     */
    AUAV3_AND_SLUGS_SENSOR_B.AirData[0] = rtb_DataTypeConversion2_b;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[1] = rtb_Sum_e;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[2] = rtb_Sum;

    /* S-Function (MCHP_C_function_Call): '<S52>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
    updateAirData(
                  AUAV3_AND_SLUGS_SENSOR_B.AirData
                  );
  }

  /* End of Outputs for SubSystem: '<S48>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S48>/myMux Fun1' */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun1': '<S55>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S55>:1:5' */
  AUAV3_AND_SLUGS_SENSOR_B.y[0] = rtb_DataTypeConversion2_b;
  AUAV3_AND_SLUGS_SENSOR_B.y[1] = AUAV3_AND_SLUGS_SENSOR_B.AirData[0];
  AUAV3_AND_SLUGS_SENSOR_B.y[2] = 0.0F;
  AUAV3_AND_SLUGS_SENSOR_B.y[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S48>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   AUAV3_AND_SLUGS_SENSOR_B.y
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S50>/Calculus Time Step1 */
  AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* DataTypeConversion: '<S50>/Data Type Conversion2' */
  rtb_DataTypeConversion2_b = (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1;

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S50>/Calculus Time Step2 */
  {
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2 = tmp;
  }

  /* DataTypeConversion: '<S50>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S50>/Data Type Conversion1'
   *  Gain: '<S50>/Gain'
   *  Product: '<S50>/Divide'
   *  Rounding: '<S50>/Rounding Function'
   */
  tmp = floor((real_T)rtb_DataTypeConversion2_b / (real_T)
              AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2 * 100.0);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 256.0);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12 = (uint8_T)(tmp < 0.0 ? (uint8_T)
    (int16_T)(int8_T)-(int8_T)(uint8_T)-tmp : (uint8_T)tmp);

  /* End of DataTypeConversion: '<S50>/Data Type Conversion12' */

  /* DataTypeConversion: '<S53>/Data Type Conversion6' */
  rtb_DataTypeConversion6 = (real_T)rtb_Switch_o[11];

  /* MATLAB Function: '<S90>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion6, 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_g,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_g);

  /* DataTypeConversion: '<S53>/Data Type Conversion8' incorporates:
   *  Constant: '<S85>/Bias'
   *  Constant: '<S85>/Gains'
   *  Product: '<S85>/Divide'
   *  Sum: '<S85>/Sum'
   */
  rtb_DataTypeConversion2_b = (real32_T)floor((real32_T)(3.1760616302490234 *
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_g.y) + 911.698242F);
  if (rtIsNaNF(rtb_DataTypeConversion2_b) || rtIsInfF(rtb_DataTypeConversion2_b))
  {
    rtb_DataTypeConversion2_b = 0.0F;
  } else {
    rtb_DataTypeConversion2_b = (real32_T)fmod(rtb_DataTypeConversion2_b,
      65536.0F);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8 = rtb_DataTypeConversion2_b <
    0.0F ? (uint16_T)-(int16_T)(uint16_T)-rtb_DataTypeConversion2_b : (uint16_T)
    rtb_DataTypeConversion2_b;

  /* End of DataTypeConversion: '<S53>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S48>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12
                 , AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8
                 );

  /* DataTypeConversion: '<S49>/Data Type Conversion2' incorporates:
   *  Constant: '<S49>/Gyro Gains'
   *  Gain: '<S59>/[ -1 -1 -1]'
   *  Product: '<S49>/Divide'
   */
  rtb_DataTypeConversion2[0] = -((real32_T)rtb_Switch_o[1] * 0.000872664619F);
  rtb_DataTypeConversion2[1] = -((real32_T)rtb_Switch_o[0] * 0.000872664619F);
  rtb_DataTypeConversion2[2] = -((real32_T)rtb_Switch_o[2] * 0.000872664619F);

  /* MATLAB Function: '<S56>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion2[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_k,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_k);

  /* MATLAB Function: '<S56>/Embedded MATLAB Function1' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion2[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S56>/Embedded MATLAB Function2' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion2[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

  /* MATLAB Function: '<S56>/myMux Fun' */
  /* DataTypeConversion: '<S49>/Data Type Conversion1' incorporates:
   *  Constant: '<S49>/Gyro Gains1'
   *  Product: '<S49>/Divide1'
   */
  rtb_DataTypeConversion1[0] = (real32_T)rtb_Switch_o[4] * 0.0326839499F;
  rtb_DataTypeConversion1[1] = (real32_T)rtb_Switch_o[3] * 0.0326839499F;
  rtb_DataTypeConversion1[2] = (real32_T)rtb_Switch_o[5] * 0.0326839499F;

  /* MATLAB Function: '<S57>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion1[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_h,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_h);

  /* MATLAB Function: '<S57>/Embedded MATLAB Function1' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion1[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_m,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_m);

  /* MATLAB Function: '<S57>/Embedded MATLAB Function2' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion1[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_k,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_k);

  /* MATLAB Function: '<S57>/myMux Fun' */
  /* DataTypeConversion: '<S49>/Data Type Conversion5' incorporates:
   *  Constant: '<S49>/Gyro Gains2'
   *  Gain: '<S61>/[ -1 -1 -1]'
   *  Product: '<S49>/Divide2'
   */
  rtb_DataTypeConversion5_l[0] = -((real32_T)rtb_Switch_o[7] * 0.5F);
  rtb_DataTypeConversion5_l[1] = -((real32_T)rtb_Switch_o[6] * 0.5F);
  rtb_DataTypeConversion5_l[2] = -((real32_T)rtb_Switch_o[8] * 0.5F);

  /* MATLAB Function: '<S58>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion5_l[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_hj,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_hj);

  /* MATLAB Function: '<S58>/Embedded MATLAB Function1' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion5_l[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_my,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_my);

  /* MATLAB Function: '<S58>/Embedded MATLAB Function2' */
  AUA_EmbeddedMATLABFunction(rtb_DataTypeConversion5_l[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_c,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_c);

  /* MATLAB Function: '<S82>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_e);

  /* Outputs for Enabled SubSystem: '<S82>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S97>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut > 0.0) {
    /* DataTypeConversion: '<S97>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Sum_e;

    /* DiscreteZeroPole: '<S100>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
    }

    /* Saturate: '<S97>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S97>/Data Type Conversion1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k = (real32_T)rtb_DiscreteZeroPole >=
      120000.0F ? 120000.0F : (real32_T)rtb_DiscreteZeroPole <= 80000.0F ?
      80000.0F : (real32_T)rtb_DiscreteZeroPole;

    /* Update for DiscreteZeroPole: '<S100>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE =
        rtb_DataTypeConversion + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
    }
  }

  /* End of Outputs for SubSystem: '<S82>/Initial Baro Bias' */

  /* Product: '<S93>/Divide' incorporates:
   *  Sum: '<S93>/Sum2'
   */
  rtb_DataTypeConversion2_b = (rtb_Sum_e - AUAV3_AND_SLUGS_SENSOR_B.u0k120k) /
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k;

  /* S-Function (MCHP_C_function_Call): '<S96>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU
                );

  /* Sum: '<S93>/Sum1' incorporates:
   *  Constant: '<S93>/Constant2'
   *  Constant: '<S93>/Constant3'
   *  Constant: '<S93>/Constant4'
   *  Constant: '<S93>/Constant5'
   *  Gain: '<S99>/Unit Conversion'
   *  Product: '<S93>/Divide1'
   *  Product: '<S93>/Divide2'
   *  Product: '<S93>/Divide3'
   *  Product: '<S93>/Divide4'
   *  Sum: '<S93>/Sum3'
   */
  rtb_DataTypeConversion2_b = ((rtb_DataTypeConversion2_b *
    rtb_DataTypeConversion2_b * 0.093502529F + rtb_DataTypeConversion2_b *
    -0.188893303F) + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0];

  /* Outputs for Enabled SubSystem: '<S82>/Zero Out Height' incorporates:
   *  EnablePort: '<S98>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut > 0.0) {
    /* Sum: '<S98>/Sum' incorporates:
     *  Delay: '<S98>/Integer Delay'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Sum =
      AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0] -
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE;

    /* Update for Delay: '<S98>/Integer Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE = rtb_DataTypeConversion2_b;
  }

  /* End of Outputs for SubSystem: '<S82>/Zero Out Height' */

  /* Sum: '<S82>/Sum1' */
  rtb_Sum1_f = AUAV3_AND_SLUGS_SENSOR_B.Sum + rtb_DataTypeConversion2_b;

  /* Logic: '<S82>/Logical Operator' */
  rtb_LogicalOperator_f =
    !(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut != 0.0);

  /* Outputs for Enabled SubSystem: '<S82>/Enabled Subsystem' */
  AUAV3_AND_EnabledSubsystem(rtb_LogicalOperator_f, rtb_Sum1_f,
    &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_o);

  /* End of Outputs for SubSystem: '<S82>/Enabled Subsystem' */

  /* Update for UnitDelay: '<S2>/Output' incorporates:
   *  Switch: '<S13>/FixPt Switch'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE = rtb_FixPtSum1;

  /* Update for Delay: '<S21>/Delay11' incorporates:
   *  Sum: '<S21>/SumA21'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE = qY_0;

  /* Update for Delay: '<S22>/Delay11' incorporates:
   *  Sum: '<S22>/SumA21'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j = qY;

  /* Update for Delay: '<S6>/Delay' incorporates:
   *  Constant: '<S6>/Constant'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S19>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S30>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Update for Delay: '<S30>/Integer Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE_k = Sum;
  }

  /* End of Update for SubSystem: '<S19>/Zero Out Height' */
}

/* Model step function for TID1 */
void AUAV3_AND_SLUGS_SENSOR_step1(void) /* Sample time: [0.02s, 0.0s] */
{
  int16_T rtb_Bias_i;
  int16_T rtb_Bias1;
  int32_T tmp;
  uint16_T u;

  /* S-Function "dsPIC_PWM_IC" Block: <S1>/Input Capture */
  AUAV3_AND_SLUGS_SENSOR_B.dT = MCHP_ic1up;
  AUAV3_AND_SLUGS_SENSOR_B.dA = MCHP_ic2up;
  AUAV3_AND_SLUGS_SENSOR_B.dE = MCHP_ic3up;
  AUAV3_AND_SLUGS_SENSOR_B.dR = MCHP_ic4up;
  AUAV3_AND_SLUGS_SENSOR_B.dFailsafe = MCHP_ic5up;

  /* MATLAB Function 'Control Surface Input/myMux Fun5': '<S11>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S11>:1:5' */
  /* S-Function "dsPIC_PWM_IC" Block: <S9>/Input Capture RC Receiver1 */
  AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw = MCHP_ic1up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o2 = MCHP_ic2up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o3 = MCHP_ic3up;
  AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw = MCHP_ic4up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o5 = MCHP_ic5up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o6 = MCHP_ic6up;

  /* Bias: '<S9>/Bias' incorporates:
   *  DataTypeConversion: '<S9>/Data Type Conversion4'
   */
  rtb_Bias_i = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw - 13125;

  /* Bias: '<S9>/Bias1' incorporates:
   *  DataTypeConversion: '<S9>/Data Type Conversion5'
   */
  rtb_Bias1 = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw - 13125;

  /* Sum: '<S111>/Sum' */
  tmp = (int32_T)rtb_Bias1 - (int32_T)rtb_Bias_i;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S9>/Data Type Conversion2' incorporates:
   *  Bias: '<S9>/Bias2'
   *  Sum: '<S111>/Sum'
   */
  u = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S9>/Saturation' */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation = u >= 19249U ? 19249U : u <= 7000U ?
    7000U : u;

  /* Sum: '<S111>/Sum1' */
  tmp = (int32_T)rtb_Bias_i + (int32_T)rtb_Bias1;
  if (tmp > 32767L) {
    tmp = 32767L;
  } else {
    if (tmp < -32768L) {
      tmp = -32768L;
    }
  }

  /* DataTypeConversion: '<S9>/Data Type Conversion3' incorporates:
   *  Bias: '<S9>/Bias3'
   *  Sum: '<S111>/Sum1'
   */
  u = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S9>/Saturation1' */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation1 = u >= 19249U ? 19249U : u <= 7000U ?
    7000U : u;

  /* DataStoreRead: '<S4>/Get Raw IMU' */
  AUAV3_AND_SLUGS_SENSOR_B.GetRawIMU = AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackRawIMU' */
  AUAV3_AND_SLUGS_SENSOR_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetRawIMU
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackRawIMU
                  );

  /* S-Function "dsPIC_PWM_OC" Block: <S9>/Output Compare - HW Drive Servo motor */
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

/* Model step function for TID2 */
void AUAV3_AND_SLUGS_SENSOR_step2(void) /* Sample time: [0.05s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator_a;

  /* S-Function (MCHP_Digital_Output_Read): '<S14>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S14>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S14>/Digital Output Read'
   */
  rtb_LogicalOperator_a = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S14>/Digital Output Write' */
  LATBbits.LATB2 = rtb_LogicalOperator_a;
}

/* Model step function for TID3 */
void AUAV3_AND_SLUGS_SENSOR_step3(void) /* Sample time: [0.1s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator1;

  /* S-Function (MCHP_Digital_Output_Read): '<S15>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S15>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S15>/Digital Output Read'
   */
  rtb_LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S15>/Digital Output Write' */
  LATBbits.LATB3 = rtb_LogicalOperator1;
}

/* Model step function for TID4 */
void AUAV3_AND_SLUGS_SENSOR_step4(void) /* Sample time: [0.2s, 0.0s] */
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

/* Model step function for TID5 */
void AUAV3_AND_SLUGS_SENSOR_step5(void) /* Sample time: [0.2s, 0.02s] */
{
  /* DataStoreRead: '<S4>/Get mlAirData' */
  AUAV3_AND_SLUGS_SENSOR_B.GetmlAirData = mlAirData;

  /* S-Function (MCHP_C_function_Call): '<S4>/PackScaledPressure' */
  AUAV3_AND_SLUGS_SENSOR_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_B.GetmlAirData
    , ((uint32_T)0UL)
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackScaledPressure
                  );
}

/* Model step function for TID6 */
void AUAV3_AND_SLUGS_SENSOR_step6(void) /* Sample time: [0.2s, 0.04s] */
{
  /* S-Function (MCHP_C_function_Call): '<S4>/PackSysStatus' */
  AUAV3_AND_SLUGS_SENSOR_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_AND_SLUGS_SENSOR_rtZmavlink_sys_status_t
    );

  /* S-Function (MCHP_C_function_Call): '<S4>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV3_AND_SLUGS_SENSOR_B.PackSysStatus
                  );
}

/* Model step function for TID7 */
void AUAV3_AND_SLUGS_SENSOR_step7(void) /* Sample time: [0.2s, 0.06s] */
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

/* Model step function for TID8 */
void AUAV3_AND_SLUGS_SENSOR_step8(void) /* Sample time: [0.2s, 0.08s] */
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

/* Model step function for TID9 */
void AUAV3_AND_SLUGS_SENSOR_step9(void) /* Sample time: [0.25s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator2;

  /* S-Function (MCHP_Digital_Output_Read): '<S16>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S16>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S16>/Digital Output Read'
   */
  rtb_LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S16>/Digital Output Write' */
  LATBbits.LATB4 = rtb_LogicalOperator2;
}

/* Model step function for TID10 */
void AUAV3_AND_SLUGS_SENSOR_step10(void) /* Sample time: [0.5s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator3;

  /* S-Function (MCHP_Digital_Output_Read): '<S17>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S17>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S17>/Digital Output Read'
   */
  rtb_LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S17>/Digital Output Write' */
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

   default :
    break;
  }
}

/* Model initialize function */
void AUAV3_AND_SLUGS_SENSOR_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[5] = 18;/* Sample time: [0.2s, 0.02s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[6] = 16;/* Sample time: [0.2s, 0.04s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[7] = 14;/* Sample time: [0.2s, 0.06s] */

  /* initialize sample time offsets */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.TaskCounters.TID[8] = 12;/* Sample time: [0.2s, 0.08s] */

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
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */

  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S6>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */

  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0xA5;                      /* I2C clock = 399772  (400000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;

  /* Start for Enabled SubSystem: '<S19>/Zero Out Height' */
  /* InitializeConditions for Delay: '<S30>/Integer Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE_k = 0.0F;

  /* End of Start for SubSystem: '<S19>/Zero Out Height' */

  /* Start for Enabled SubSystem: '<S19>/Enabled Subsystem' */
  AUA_EnabledSubsystem_Start(&AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem);

  /* End of Start for SubSystem: '<S19>/Enabled Subsystem' */

  /* Start for S-Function (MCHP_BUS_SPI): '<S7>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */

  /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
  SPI1CON1 = 0x0474;
  SPI1CON2 = 0x01;
  SPI1STAT = 0x8014;
  _SPI1IP = 5;                         /* Set SPI Interrupt Priority */
  _SPI1IF = 0;                         /* Reset interrupt Flag */
  _SPI1IE = 1;                         /* Enable Interrupts */

  /* MCHP_Digital_Output_Write Block: '<Root>/Drive SPI SSL High1' */
  LATBbits.LATB2 = TRUE;

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

  /* Start for S-Function (MCHP_MCU_LOAD): '<S50>/Calculus Time Step1' */
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */

  /* Start for Enabled SubSystem: '<S82>/Zero Out Height' */
  /* InitializeConditions for Delay: '<S98>/Integer Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE = 0.0F;

  /* End of Start for SubSystem: '<S82>/Zero Out Height' */

  /* Start for Enabled SubSystem: '<S82>/Enabled Subsystem' */
  AUA_EnabledSubsystem_Start(&AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_o);

  /* End of Start for SubSystem: '<S82>/Enabled Subsystem' */

  /* Start for S-Function (MCHP_IC): '<S9>/Input Capture RC Receiver1' */
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

  /* Start for S-Function (MCHP_OC_HW): '<S9>/Output Compare - HW Drive Servo motor' */

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

  U1BRG = 0x9B5C;                      /* Baud rate: 110 (-0.00%) */
  U1MODE = 0x8000;
  U1STA = 0x2400;

  /* Configure UART1 Tx Interruption */
  MCHP_UART1_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART1_Tx.tail = 0;
  _U1TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U1TXIF = 0;
  _U1TXIE = 1;                         /* Enable Interrupt */

  /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
  U4BRG = 0x9B5C;                      /* Baud rate: 110 (-0.00%) */
  U4MODE = 0x8000;
  U4STA = 0x2400;

  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U4TXIF = 0;
  _U4TXIE = 1;                         /* Enable Interrupt */

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();

  /* InitializeConditions for Delay: '<S21>/Delay11' */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE = 0UL;

  /* InitializeConditions for Delay: '<S22>/Delay11' */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j = 0UL;

  /* InitializeConditions for Delay: '<S6>/Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S19>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

  /* InitializeConditions for MATLAB Function: '<S89>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction);

  /* InitializeConditions for MATLAB Function: '<S91>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_l);

  /* InitializeConditions for MATLAB Function: '<S92>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

  /* InitializeConditions for MATLAB Function: '<S90>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_g);

  /* InitializeConditions for MATLAB Function: '<S56>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_k);

  /* InitializeConditions for MATLAB Function: '<S56>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

  /* InitializeConditions for MATLAB Function: '<S56>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

  /* InitializeConditions for MATLAB Function: '<S57>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_h);

  /* InitializeConditions for MATLAB Function: '<S57>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_m);

  /* InitializeConditions for MATLAB Function: '<S57>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_k);

  /* InitializeConditions for MATLAB Function: '<S58>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_hj);

  /* InitializeConditions for MATLAB Function: '<S58>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_my);

  /* InitializeConditions for MATLAB Function: '<S58>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunctio_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_c);

  /* InitializeConditions for MATLAB Function: '<S82>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_e);
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
