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
 * Model version                        : 1.208
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Wed Apr 27 00:58:09 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Wed Apr 27 00:58:11 2016
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
 * Output and update for atomic system:
 *    '<S29>/myMux Fun1'
 *    '<S98>/myMux Fun1'
 */
void AUAV3_AND_SLUGS__myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun1_AUAV3_AND_SLUGS_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1': '<S45>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S45>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S56>/negprotect'
 *    '<S61>/negprotect'
 */
void AUAV3_AND_SLUGS_negprotect(real32_T rtu_val,
  rtB_negprotect_AUAV3_AND_SLUG_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2/negprotect': '<S58>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S58>:1:5' if val >= single(0.001) */
  if (rtu_val >= 0.001F) {
    /* '<S58>:1:6' zpVal = val; */
    localB->zpVal = rtu_val;
  } else {
    /* '<S58>:1:7' else */
    /* '<S58>:1:8' zpVal = single(0.001); */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S57>/Embedded MATLAB Function'
 *    '<S62>/Embedded MATLAB Function'
 */
void AUA_EmbeddedMATLABFunction(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_AU_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function': '<S59>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S59>:1:5' xDoty = x(1)*y(1) + x(2)*y(2) + x(3)*y(3); */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Initial conditions for atomic system:
 *    '<S104>/Embedded MATLAB Function1'
 *    '<S104>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunct_a_Init(rtDW_EmbeddedMATLABFunction1__T *localDW)
{
  localDW->LastU_not_empty = FALSE;
}

/*
 * Output and update for atomic system:
 *    '<S104>/Embedded MATLAB Function1'
 *    '<S104>/Embedded MATLAB Function2'
 */
void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW)
{
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1': '<S120>:1' */
  /* '<S120>:1:7' if (isempty(LastU)) */
  if (!localDW->LastU_not_empty) {
    /* '<S120>:1:8' LastU = single(0.); */
    localDW->LastU = 0.0F;
    localDW->LastU_not_empty = TRUE;

    /* '<S120>:1:9' TimeSinceLast = single(apSampleTime); */
    localDW->TimeSinceLast = 0.01F;

    /* '<S120>:1:10' rate = single(0); */
    localDW->rate = 0.0F;

    /* '<S120>:1:11' OldRate = single(0); */
    localDW->OldRate = 0.0F;
  }

  /* '<S120>:1:14' if (NewGPS) */
  if (rtu_NewGPS != 0) {
    /* '<S120>:1:15' OldRate = rate; */
    localDW->OldRate = localDW->rate;

    /* '<S120>:1:16' rate = single((u - LastU)/TimeSinceLast); */
    localDW->rate = (rtu_u - localDW->LastU) / localDW->TimeSinceLast;

    /* '<S120>:1:17' TimeSinceLast = single(0); */
    localDW->TimeSinceLast = 0.0F;

    /* '<S120>:1:18' LastU = u; */
    localDW->LastU = rtu_u;
  }

  /* '<S120>:1:20' y = u + ( .5*rate + .5*OldRate )*TimeSinceLast; */
  localB->y = (0.5F * localDW->rate + 0.5F * localDW->OldRate) *
    localDW->TimeSinceLast + rtu_u;

  /* '<S120>:1:21' TimeSinceLast = TimeSinceLast + apSampleTime; */
  localDW->TimeSinceLast = localDW->TimeSinceLast + 0.01F;
}

/*
 * Output and update for atomic system:
 *    '<S96>/myMux Fun1'
 *    '<S96>/myMux Fun2'
 */
void AUAV3_AND_SLUG_myMuxFun1_p(real32_T rtu_u1, real32_T rtu_u2, real32_T
  rtu_u3, rtB_myMuxFun1_AUAV3_AND_SLU_c_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun1': '<S105>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S105>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Start for enable system:
 *    '<S123>/Enabled Subsystem'
 *    '<S186>/Enabled Subsystem'
 */
void AUA_EnabledSubsystem_Start(rtB_EnabledSubsystem_AUAV3_AN_T *localB)
{
  /* VirtualOutportStart for Outport: '<S131>/Out1' */
  localB->In1 = 143.543F;
}

/*
 * Output and update for enable system:
 *    '<S123>/Enabled Subsystem'
 *    '<S186>/Enabled Subsystem'
 */
void AUAV3_AND_EnabledSubsystem(boolean_T rtu_0, real32_T rtu_1,
  rtB_EnabledSubsystem_AUAV3_AN_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S123>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S131>/Enable'
   */
  if (rtu_0) {
    /* Inport: '<S131>/In1' */
    localB->In1 = rtu_1;
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
  localDW->aveCount = 1.0;
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
    localDW->aveCount = localDW->aveCount + 1.0;
  }

  /* '<S132>:1:15' if aveCount == 2500 */
  if (localDW->aveCount == 2500.0) {
    /* '<S132>:1:16' tIni =0; */
    localDW->tIni = 0.0;

    /* '<S132>:1:17' aveCount = aveCount + 1; */
    localDW->aveCount = localDW->aveCount + 1.0;
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
 * Initial conditions for enable system:
 *    '<S123>/Zero Out Height'
 *    '<S186>/Zero Out Height'
 */
void AUAV3_A_ZeroOutHeight_Init(rtDW_ZeroOutHeight_AUAV3_AND__T *localDW)
{
  /* InitializeConditions for Delay: '<S134>/Integer Delay' */
  localDW->IntegerDelay_DSTATE = 0.0F;
}

/*
 * Start for enable system:
 *    '<S123>/Zero Out Height'
 *    '<S186>/Zero Out Height'
 */
void AUAV3__ZeroOutHeight_Start(rtDW_ZeroOutHeight_AUAV3_AND__T *localDW)
{
  AUAV3_A_ZeroOutHeight_Init(localDW);
}

/*
 * Outputs for enable system:
 *    '<S123>/Zero Out Height'
 *    '<S186>/Zero Out Height'
 */
void AUAV3_AND_SL_ZeroOutHeight(real_T rtu_0, real32_T rtu_BaseHeight,
  rtB_ZeroOutHeight_AUAV3_AND_S_T *localB, rtDW_ZeroOutHeight_AUAV3_AND__T
  *localDW)
{
  /* Outputs for Enabled SubSystem: '<S123>/Zero Out Height' incorporates:
   *  EnablePort: '<S134>/Enable'
   */
  if (rtu_0 > 0.0) {
    /* Sum: '<S134>/Sum' incorporates:
     *  Delay: '<S134>/Integer Delay'
     */
    localB->Sum = rtu_BaseHeight - localDW->IntegerDelay_DSTATE;
  }

  /* End of Outputs for SubSystem: '<S123>/Zero Out Height' */
}

/*
 * Update for enable system:
 *    '<S123>/Zero Out Height'
 *    '<S186>/Zero Out Height'
 */
void AUAV3_ZeroOutHeight_Update(real_T rtu_0, real32_T rtu_ComputedHeight,
  rtDW_ZeroOutHeight_AUAV3_AND__T *localDW)
{
  /* Update for Enabled SubSystem: '<S123>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S134>/Enable'
   */
  if (rtu_0 > 0.0) {
    /* Update for Delay: '<S134>/Integer Delay' */
    localDW->IntegerDelay_DSTATE = rtu_ComputedHeight;
  }

  /* End of Update for SubSystem: '<S123>/Zero Out Height' */
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
void EmbeddedMATLABFunct_m_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW)
{
  localDW->a_not_empty = FALSE;
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
void A_EmbeddedMATLABFunction_k(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_n_T *localB, rtDW_EmbeddedMATLABFunction_i_T
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
    localDW->a_not_empty = TRUE;

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
 *    '<S160>/myMux Fun'
 *    '<S161>/myMux Fun'
 *    '<S162>/myMux Fun'
 */
void AUAV3_AND_SLUGS_S_myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_AUAV3_AND_SLUGS__T *localB)
{
  /* MATLAB Function 'Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S169>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S169>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S155>/Embedded MATLAB Function'
 *    '<S155>/Embedded MATLAB Function1'
 *    '<S155>/Embedded MATLAB Function2'
 *    '<S155>/Embedded MATLAB Function3'
 */
void EmbeddedMATLABFunction_kd(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_d_T *localB)
{
  /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function': '<S178>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S178>:1:5' y = (u(1)+ u(2) + u(3) + u(4) + u(5))*0.20; */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/* Model step function for TID0 */
void AUAV3_AND_SLUGS_SENSOR_step0(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion5;
  real_T rtb_DataTypeConversion19;
  real_T rtb_DataTypeConversion1[3];
  real_T rtb_DataTypeConversion2[3];
  real_T rtb_DiscreteZeroPole;
  real_T rtb_DiscreteZeroPole_i;
  real_T rtb_DiscreteZeroPole_d;
  real_T rtb_DataTypeConversion2_b[3];
  real_T rtb_DiscreteZeroPole_n;
  real_T rtb_DiscreteZeroPole_g;
  real_T rtb_DiscreteZeroPole_p;
  real_T rtb_DataTypeConversion2_d[3];
  real_T rtb_DataTypeConversion2_k;
  real_T rtb_DataTypeConversion2_p;
  real_T rtb_DataTypeConversion2_bt;
  real_T rtb_DataTypeConversion3;
  real_T rtb_DataTypeConversion6;
  real_T rtb_DataTypeConversion5_l[3];
  real_T rtb_DataTypeConversion;
  real_T rtb_DataTypeConversion_d;
  real_T rtb_DiscreteZeroPole_j;
  real_T rtb_DiscreteZeroPole_k;
  real_T rtb_RoundingFunction;
  real32_T rtb_Sum1;
  real32_T rtb_Product1_b;
  real32_T rtb_Product_e;
  real32_T rtb_Sum1_f;
  real32_T rtb_Gain_k;
  real32_T rtb_DataTypeConversion1_a[3];
  real32_T rtb_Add[3];
  real32_T rtb_BiasRateLimiter[3];
  real32_T rtb_DataTypeConversion1_d;
  real32_T rtb_DataTypeConversion1_dq;
  real32_T rtb_Gain1_o;
  real32_T rtb_y_b[3];
  uint8_T rtb_DataTypeConversion2_ob;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_EnableHILfromControlMCU1;
  boolean_T rtb_LogicalOperator_f;
  uint32_T rtb_FixPtSum1;
  uint16_T rtb_Sum1_a2;
  int16_T rtb_DataTypeConversion1_a0;
  uint16_T rtb_BitwiseOperator_e;
  int16_T rtb_Gain21_h;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_RhhcosphicoslambXe;
  int16_T rtb_DataTypeConversion2_o;
  int16_T rtb_DataTypeConversion3_m;
  int16_T rtb_DataTypeConversion4;
  int16_T rtb_DataTypeConversion5_b;
  real32_T rtb_Product_p;
  real32_T rtb_Product1_j;
  real32_T rtb_Product2_c;
  real32_T rtb_Product3_d;
  real32_T rtb_Deg2R1;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_jxi;
  real32_T rtb_ixk;
  real32_T rtb_kxj;
  real32_T rtb_ixj;
  real32_T rtb_DataTypeConversion1_o;
  real32_T rtb_Sum_bt;
  boolean_T rtb_LogicalOperator_o;
  int16_T rtb_Switch_o[13];
  real32_T rtb_Sum_jv;
  uint8_T rtb_Compare_b;
  real32_T rtb_Sum_e1;
  uint32_T rtb_Sum6;
  boolean_T rtb_LogicalOperator_g;
  real32_T rtb_UEN2NEU[3];
  real32_T rtb_q_dot[4];
  real32_T Product[3];
  real32_T Product1_o[3];
  real32_T tmp[9];
  real32_T tmp_0[3];
  real32_T rtb_TmpSignalConversionAtSFun_0[9];
  real32_T rtb_Deg2R_idx;
  real32_T rtb_Deg2R_idx_0;
  real32_T rtb_RateLimiter_idx;
  real32_T rtb_RateLimiter_idx_0;
  real32_T rtb_RateLimiter_idx_1;
  real32_T rtb_Divide2_l_idx;
  real32_T rtb_Divide2_l_idx_0;
  real32_T rtb_Divide2_l_idx_1;
  real_T tmp_1;
  uint32_T qY;
  uint32_T qY_0;
  uint64m_T tmp_2;
  uint64m_T tmp_3;

  /* UnitDelay: '<S2>/Output' */
  rtb_FixPtSum1 = AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE + 1UL;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S2>/Output'
   */
  rtb_Sum6 = 3276800000UL;
  uMultiWordMul(&rtb_Sum6, 1, &AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE, 1,
                &tmp_3.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_3.chunks[0U], 2, 15U, &tmp_2.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.time_since_boot_usec = MultiWord2uLong
    (&tmp_2.chunks[0U]);

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

  rtb_Sum1_a2 = (uint16_T)(qY >> 16);

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

  rtb_BitwiseOperator_e = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S126>/SumB21' */

  /* DataStoreWrite: '<S7>/Update RawPressure' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion7'
   *  DataTypeConversion: '<S7>/Data Type Conversion8'
   */
  mlRawPressureData.press_abs = (int16_T)rtb_Sum1_a2;
  mlRawPressureData.temperature = (int16_T)rtb_BitwiseOperator_e;

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
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S124>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S140>/Data Type Conversion1'
     *  DataTypeConversion: '<S140>/Data Type Conversion3'
     *  Gain: '<S140>/Gain'
     *  S-Function (sfix_bitop): '<S140>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S124>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S142>/Data Type Conversion1'
     *  DataTypeConversion: '<S142>/Data Type Conversion3'
     *  Gain: '<S142>/Gain'
     *  S-Function (sfix_bitop): '<S142>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S124>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S146>/Data Type Conversion1'
     *  DataTypeConversion: '<S146>/Data Type Conversion3'
     *  Gain: '<S146>/Gain'
     *  S-Function (sfix_bitop): '<S146>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S124>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S147>/Data Type Conversion1'
     *  DataTypeConversion: '<S147>/Data Type Conversion3'
     *  Gain: '<S147>/Gain'
     *  S-Function (sfix_bitop): '<S147>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S124>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S138>/Data Type Conversion1'
     *  DataTypeConversion: '<S138>/Data Type Conversion3'
     *  Gain: '<S138>/Gain'
     *  S-Function (sfix_bitop): '<S138>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S124>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S139>/Data Type Conversion1'
     *  DataTypeConversion: '<S139>/Data Type Conversion3'
     *  Gain: '<S139>/Gain'
     *  S-Function (sfix_bitop): '<S139>/Bitwise Operator'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S143>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S143>/Data Type Conversion1'
     *  DataTypeConversion: '<S143>/Data Type Conversion3'
     *  Gain: '<S143>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition5 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S144>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S144>/Data Type Conversion1'
     *  DataTypeConversion: '<S144>/Data Type Conversion3'
     *  Gain: '<S144>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition6 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S145>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S145>/Data Type Conversion1'
     *  DataTypeConversion: '<S145>/Data Type Conversion3'
     *  Gain: '<S145>/Gain'
     */
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition7 = (uint16_T)
      AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S7>/Delay' */
  /* End of Outputs for SubSystem: '<S7>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S122>/Sum' */
  rtb_Sum1_a2 -= AUAV3_AND_SLUGS_SENSOR_B.RateTransition7;

  /* Product: '<S122>/Product' */
  rtb_Sum6 = (uint32_T)rtb_Sum1_a2 * (uint32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition6;
  rtb_Sum1_a2 = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) != 0U) +
    (rtb_Sum6 >> 15));

  /* Sum: '<S122>/Sum2' incorporates:
   *  Product: '<S122>/Product1'
   *  Sum: '<S122>/Sum1'
   */
  rtb_DataTypeConversion1_a0 = div_s16s32_round((int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition11 << 11, (int32_T)(rtb_Sum1_a2 +
    (uint16_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition12)) + (int16_T)rtb_Sum1_a2;

  /* Gain: '<S7>/Gain21' incorporates:
   *  Bias: '<S122>/Bias'
   */
  rtb_Gain21_h = (int16_T)((int32_T)(rtb_DataTypeConversion1_a0 + 8) * 5L >> 3);

  /* MATLAB Function: '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S123>/Zero Out Height' */
  AUAV3_AND_SL_ZeroOutHeight
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut, 143.543F,
     &AUAV3_AND_SLUGS_SENSOR_B.ZeroOutHeight,
     &AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight);

  /* End of Outputs for SubSystem: '<S123>/Zero Out Height' */

  /* Bias: '<S122>/Bias1' */
  rtb_DataTypeConversion1_a0 -= 4000;

  /* Math: '<S122>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_DataTypeConversion1_a0 * (int32_T)
    rtb_DataTypeConversion1_a0;

  /* Sum: '<S122>/Sum6' incorporates:
   *  Bias: '<S122>/Bias2'
   *  Product: '<S122>/Product2'
   *  Product: '<S122>/Product3'
   *  Sum: '<S122>/Sum3'
   *  Sum: '<S122>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_BitwiseOperator_e - (uint32_T)((((int32_T)((int16_T)
    ((int32_T)rtb_DataTypeConversion1_a0 * (int32_T)
     AUAV3_AND_SLUGS_SENSOR_B.RateTransition3 >> 11) + (int16_T)
    mul_s32_s32_s32_sr23((int32_T)AUAV3_AND_SLUGS_SENSOR_B.RateTransition9,
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
  rtb_B4 = mul_u32_s32_u32_sr15((int32_T)((((int16_T)((int32_T)
    rtb_DataTypeConversion1_a0 * (int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition4 >> 13) + (int16_T)
    mul_s32_s32_s32_sr28(rtb_Bias5, (int32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition8)) + 2) >> 2) + 32768L, (uint32_T)
    AUAV3_AND_SLUGS_SENSOR_B.RateTransition5);

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
    rtb_DataTypeConversion_d = (real_T)rtb_Bias5;

    /* DiscreteZeroPole: '<S136>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_k = 0.014778325123152709*rtb_DataTypeConversion_d;
      rtb_DiscreteZeroPole_k += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j;
    }

    /* Saturate: '<S133>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S133>/Data Type Conversion1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e = (real32_T)rtb_DiscreteZeroPole_k >=
      120000.0F ? 120000.0F : (real32_T)rtb_DiscreteZeroPole_k <= 80000.0F ?
      80000.0F : (real32_T)rtb_DiscreteZeroPole_k;

    /* Update for DiscreteZeroPole: '<S136>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j =
        rtb_DataTypeConversion_d + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_j;
    }
  }

  /* End of Outputs for SubSystem: '<S123>/Initial Baro Bias' */

  /* Product: '<S130>/Divide' incorporates:
   *  Sum: '<S130>/Sum2'
   */
  rtb_RhhcosphicoslambXe = ((real32_T)rtb_Bias5 -
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e) / AUAV3_AND_SLUGS_SENSOR_B.u0k120k_e;

  /* Sum: '<S130>/Sum1' incorporates:
   *  Constant: '<S123>/Constant5'
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
  AUAV3_AND_SLUGS_SENSOR_B.Sum1 = ((rtb_RhhcosphicoslambXe *
    rtb_RhhcosphicoslambXe * 0.093502529F + rtb_RhhcosphicoslambXe *
    -0.188893303F) + 2.18031291E-5F) * 145473.5F * 0.3048F + 143.543F;

  /* Sum: '<S123>/Sum1' */
  rtb_Sum1 = AUAV3_AND_SLUGS_SENSOR_B.ZeroOutHeight.Sum +
    AUAV3_AND_SLUGS_SENSOR_B.Sum1;

  /* Logic: '<S123>/Logical Operator' */
  rtb_LogicalOperator =
    !(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut != 0.0);

  /* Outputs for Enabled SubSystem: '<S123>/Enabled Subsystem' */
  AUAV3_AND_EnabledSubsystem(rtb_LogicalOperator, rtb_Sum1,
    &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem);

  /* End of Outputs for SubSystem: '<S123>/Enabled Subsystem' */

  /* DataStoreWrite: '<S7>/Update ScaledPressure' */
  mlAirData.temperature = rtb_Gain21_h;
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

  /* DataTypeConversion: '<S8>/Data Type Conversion' */
  rtb_Gain21_h = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[0];

  /* DataTypeConversion: '<S8>/Data Type Conversion1' */
  rtb_DataTypeConversion1_a0 = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[1];

  /* DataTypeConversion: '<S8>/Data Type Conversion2' */
  rtb_DataTypeConversion2_o = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.U1CH8[2];

  /* DataTypeConversion: '<S8>/Data Type Conversion3' */
  rtb_DataTypeConversion3_m = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0];

  /* DataTypeConversion: '<S8>/Data Type Conversion4' */
  rtb_DataTypeConversion4 = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1];

  /* DataTypeConversion: '<S8>/Data Type Conversion5' */
  rtb_DataTypeConversion5_b = (int16_T)
    AUAV3_AND_SLUGS_SENSOR_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2];

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
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.xacc = rtb_Gain21_h;
  AUAV3_AND_SLUGS_SENSOR_DWork.mlRawIMU.yacc = rtb_DataTypeConversion1_a0;
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

  /* Sqrt: '<S65>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator1'
   *  Product: '<S66>/Product'
   *  Product: '<S66>/Product1'
   *  Product: '<S66>/Product2'
   *  Product: '<S66>/Product3'
   *  Sum: '<S66>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt
    (((AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] *
       AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] +
       AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] *
       AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1]) +
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] *
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2]) +
     AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] *
     AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3]);

  /* Product: '<S38>/Product' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator1'
   */
  rtb_Product_p = AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0]
    / rtb_RhhcosphicoslambXe;

  /* Product: '<S38>/Product1' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator1'
   */
  rtb_Product1_j = AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1]
    / rtb_RhhcosphicoslambXe;

  /* Product: '<S38>/Product2' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator1'
   */
  rtb_Product2_c = AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2]
    / rtb_RhhcosphicoslambXe;

  /* Product: '<S38>/Product3' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator1'
   */
  rtb_Product3_d = AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3]
    / rtb_RhhcosphicoslambXe;

  /* Sqrt: '<S78>/sqrt' incorporates:
   *  Product: '<S79>/Product'
   *  Product: '<S79>/Product1'
   *  Product: '<S79>/Product2'
   *  Product: '<S79>/Product3'
   *  Sum: '<S79>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)sqrt(((rtb_Product_p * rtb_Product_p +
    rtb_Product1_j * rtb_Product1_j) + rtb_Product2_c * rtb_Product2_c) +
    rtb_Product3_d * rtb_Product3_d);

  /* Product: '<S77>/Product' */
  rtb_Deg2R1 = rtb_Product_p / rtb_RhhcosphicoslambXe;

  /* Product: '<S77>/Product1' */
  rtb_RhhcosphisinlambYe = rtb_Product1_j / rtb_RhhcosphicoslambXe;

  /* Product: '<S77>/Product2' */
  rtb_jxi = rtb_Product2_c / rtb_RhhcosphicoslambXe;

  /* Product: '<S77>/Product3' */
  rtb_RhhcosphicoslambXe = rtb_Product3_d / rtb_RhhcosphicoslambXe;

  /* Sum: '<S67>/Sum' incorporates:
   *  Product: '<S67>/Product'
   *  Product: '<S67>/Product1'
   *  Product: '<S67>/Product2'
   *  Product: '<S67>/Product3'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[0] = ((rtb_Deg2R1 * rtb_Deg2R1 +
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S70>/Gain' incorporates:
   *  Product: '<S70>/Product2'
   *  Product: '<S70>/Product3'
   *  Sum: '<S70>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[1] = (rtb_RhhcosphisinlambYe *
    rtb_jxi - rtb_RhhcosphicoslambXe * rtb_Deg2R1) * 2.0F;

  /* Gain: '<S73>/Gain' incorporates:
   *  Product: '<S73>/Product1'
   *  Product: '<S73>/Product2'
   *  Sum: '<S73>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[2] = (rtb_Deg2R1 * rtb_jxi +
    rtb_RhhcosphisinlambYe * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Gain: '<S68>/Gain' incorporates:
   *  Product: '<S68>/Product2'
   *  Product: '<S68>/Product3'
   *  Sum: '<S68>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[3] = (rtb_RhhcosphicoslambXe *
    rtb_Deg2R1 + rtb_RhhcosphisinlambYe * rtb_jxi) * 2.0F;

  /* Sum: '<S71>/Sum' incorporates:
   *  Product: '<S71>/Product'
   *  Product: '<S71>/Product1'
   *  Product: '<S71>/Product2'
   *  Product: '<S71>/Product3'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[4] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) + rtb_jxi * rtb_jxi) -
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* Gain: '<S74>/Gain' incorporates:
   *  Product: '<S74>/Product1'
   *  Product: '<S74>/Product2'
   *  Sum: '<S74>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[5] = (rtb_jxi *
    rtb_RhhcosphicoslambXe - rtb_Deg2R1 * rtb_RhhcosphisinlambYe) * 2.0F;

  /* Gain: '<S69>/Gain' incorporates:
   *  Product: '<S69>/Product1'
   *  Product: '<S69>/Product2'
   *  Sum: '<S69>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[6] = (rtb_RhhcosphisinlambYe *
    rtb_RhhcosphicoslambXe - rtb_Deg2R1 * rtb_jxi) * 2.0F;

  /* Gain: '<S72>/Gain' incorporates:
   *  Product: '<S72>/Product1'
   *  Product: '<S72>/Product2'
   *  Sum: '<S72>/Sum'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[7] = (rtb_Deg2R1 *
    rtb_RhhcosphisinlambYe + rtb_jxi * rtb_RhhcosphicoslambXe) * 2.0F;

  /* Sum: '<S75>/Sum' incorporates:
   *  Product: '<S75>/Product'
   *  Product: '<S75>/Product1'
   *  Product: '<S75>/Product2'
   *  Product: '<S75>/Product3'
   */
  AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[8] = ((rtb_Deg2R1 * rtb_Deg2R1 -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphisinlambYe) - rtb_jxi * rtb_jxi) +
    rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S22>/Submatrix1' */
  rtb_Bias5 = 0L;
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    AUAV3_AND_SLUGS_SENSOR_B.Submatrix1[rtb_Bias5] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[(int32_T)(rtb_Gain21_h * 3)];
    rtb_Bias5++;
  }

  /* End of S-Function (sdspsubmtrx): '<S22>/Submatrix1' */

  /* MATLAB Function: '<S22>/myMux Fun2' incorporates:
   *  Constant: '<S22>/Constant1'
   */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/myMux Fun2': '<S40>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S40>:1:5' y = [u1(1); u2(1); u3(1)]; */
  rtb_y_b[0] = AUAV3_AND_SLUGS_SENSOR_B.Submatrix1[0];
  rtb_y_b[1] = AUAV3_AND_SLUGS_SENSOR_B.Submatrix1[1];
  rtb_y_b[2] = 0.0F;

  /* MATLAB Function: '<S62>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_y_b,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction);

  /* MATLAB Function: '<S61>/negprotect' */
  AUAV3_AND_SLUGS_negprotect
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction.xDoty,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_negprotect);

  /* S-Function (MCHP_C_function_Call): '<S61>/[apUtils.c]' */
  AUAV3_AND_SLUGS_SENSOR_B.apUtilsc = mySqrt(
    AUAV3_AND_SLUGS_SENSOR_B.sf_negprotect.zpVal
    );

  /* Saturate: '<S37>/Zero Bound' */
  rtb_RhhcosphicoslambXe = AUAV3_AND_SLUGS_SENSOR_B.apUtilsc >= 0.001F ?
    AUAV3_AND_SLUGS_SENSOR_B.apUtilsc : 0.001F;

  /* Product: '<S37>/Divide' */
  rtb_RateLimiter_idx_1 = rtb_y_b[0] / rtb_RhhcosphicoslambXe;
  rtb_RateLimiter_idx = rtb_y_b[1] / rtb_RhhcosphicoslambXe;
  rtb_RateLimiter_idx_0 = rtb_y_b[2] / rtb_RhhcosphicoslambXe;

  /* S-Function (MCHP_C_function_Call): '<S6>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU
                );

  /* Gain: '<S91>/Deg2R' */
  rtb_Deg2R_idx = 0.0174532924F *
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[2];
  rtb_Deg2R_idx_0 = 0.0174532924F *
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[1];

  /* S-Function (MCHP_Digital_Input): '<S9>/Enable HIL from  Control MCU1' */

  /* MCHP_Digital_Input Block: <S9>/Enable HIL from  Control MCU1/Output */
  rtb_EnableHILfromControlMCU1 = PORTDbits.RD2;/* Read pin D2 */

  /* Logic: '<S152>/Logical Operator' */
  rtb_LogicalOperator_o = !rtb_EnableHILfromControlMCU1;

  /* Outputs for Enabled SubSystem: '<S152>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S155>/Enable'
   */
  if (rtb_LogicalOperator_o) {
    /* MATLAB Function: '<S155>/Embedded MATLAB Function' */
    EmbeddedMATLABFunction_kd(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled7,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_kd);

    /* DataTypeConversion: '<S155>/Data Type Conversion' */
    tmp_1 = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_kd.y);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_Gain21_h = tmp_1 < 0.0 ? -(int16_T)(uint16_T)-tmp_1 : (int16_T)(uint16_T)
      tmp_1;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunction_kd(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled7,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_d);

    /* DataTypeConversion: '<S155>/Data Type Conversion1' */
    tmp_1 = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_d.y);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion2_o = tmp_1 < 0.0 ? -(int16_T)(uint16_T)-tmp_1 :
      (int16_T)(uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion1' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunction_kd(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled7,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_o);

    /* DataTypeConversion: '<S155>/Data Type Conversion2' */
    tmp_1 = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_o.y);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion3_m = tmp_1 < 0.0 ? -(int16_T)(uint16_T)-tmp_1 :
      (int16_T)(uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion2' */

    /* MATLAB Function: '<S155>/Embedded MATLAB Function3' */
    EmbeddedMATLABFunction_kd(AUAV3_AND_SLUGS_SENSOR_ConstP.pooled7,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3_c);

    /* DataTypeConversion: '<S155>/Data Type Conversion3' */
    tmp_1 = floor(AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction3_c.y);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion4 = tmp_1 < 0.0 ? -(int16_T)(uint16_T)-tmp_1 :
      (int16_T)(uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S155>/Data Type Conversion3' */

    /* MATLAB Function: '<S155>/myMux Fun' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S184>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S184>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
    AUAV3_AND_SLUGS_SENSOR_B.y_p[0] = rtb_Gain21_h;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[1] = rtb_DataTypeConversion2_o;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[2] = rtb_DataTypeConversion3_m;
    AUAV3_AND_SLUGS_SENSOR_B.y_p[3] = rtb_DataTypeConversion4;

    /* S-Function (MCHP_C_function_Call): '<S155>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     AUAV3_AND_SLUGS_SENSOR_B.y_p
                     );

    /* S-Function (MCHP_C_function_Call): '<S155>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                AUAV3_AND_SLUGS_SENSOR_B.ReadtheCubeDataadisCube16405c1
                );

    /* MATLAB Function: '<S155>/myMux Fun4' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S185>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S185>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1); u6(1); u7(1); u8(1); u9(1); u10(1); u11(1); u12(1); u13(1)]; */
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

    /* S-Function (MCHP_C_function_Call): '<S155>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 = isGPSNovatel(
      );

    /* Logic: '<S155>/Logical Operator' */
    rtb_LogicalOperator_g =
      !(AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 != 0);

    /* Outputs for Enabled SubSystem: '<S155>/if GPS is Novatel' incorporates:
     *  EnablePort: '<S182>/Enable'
     */
    if (AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 > 0) {
      /* S-Function (MCHP_C_function_Call): '<S182>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]2' */
      getGpsMainData(
                     AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdat_i
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
    if (rtb_LogicalOperator_g) {
      /* S-Function (MCHP_C_function_Call): '<S183>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdatet
                          );
    }

    /* End of Outputs for SubSystem: '<S155>/if GPS is Ublox' */

    /* Switch: '<S155>/Switch2' */
    for (rtb_Gain21_h = 0; rtb_Gain21_h < 5; rtb_Gain21_h++) {
      if (AUAV3_AND_SLUGS_SENSOR_B.IstheGPSNovatelorUbloxgpsPortc1 > 0) {
        AUAV3_AND_SLUGS_SENSOR_B.Switch2[rtb_Gain21_h] =
          AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdat_i[rtb_Gain21_h];
      } else {
        AUAV3_AND_SLUGS_SENSOR_B.Switch2[rtb_Gain21_h] =
          AUAV3_AND_SLUGS_SENSOR_B.ProducetheGPSMainDataandupdatet[rtb_Gain21_h];
      }
    }

    /* End of Switch: '<S155>/Switch2' */
  }

  /* End of Outputs for SubSystem: '<S152>/If no HIL then Read all the Sensors' */

  /* Gain: '<S92>/Deg2R' */
  rtb_Sum_bt = 0.0174532924F * AUAV3_AND_SLUGS_SENSOR_B.Switch2[0];

  /* Trigonometry: '<S92>/sin(phi)' */
  rtb_DataTypeConversion1_o = (real32_T)sin(rtb_Sum_bt);

  /* Sum: '<S92>/Sum1' incorporates:
   *  Constant: '<S92>/const'
   *  Product: '<S92>/Product1'
   *  Product: '<S92>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_DataTypeConversion1_o *
    rtb_DataTypeConversion1_o * 0.00669425726F;

  /* Fcn: '<S92>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S92>/f' */

  /* Product: '<S92>/Rh' incorporates:
   *  Constant: '<S92>/Re=equatorial radius'
   */
  rtb_ixj = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S92>/Sum2' */
  rtb_kxj = AUAV3_AND_SLUGS_SENSOR_B.Switch2[2] + rtb_ixj;

  /* Trigonometry: '<S92>/cos(phi)' */
  rtb_Sum_bt = (real32_T)cos(rtb_Sum_bt);

  /* Gain: '<S92>/Deg2R1' */
  rtb_ixk = 0.0174532924F * AUAV3_AND_SLUGS_SENSOR_B.Switch2[1];

  /* Product: '<S92>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S92>/cos(lamb)'
   */
  rtb_jxi = rtb_kxj * rtb_Sum_bt * (real32_T)cos(rtb_ixk);

  /* Product: '<S92>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S92>/sin(lamb)'
   */
  rtb_kxj = rtb_kxj * rtb_Sum_bt * (real32_T)sin(rtb_ixk);

  /* Product: '<S92>/Ze' incorporates:
   *  Product: '<S92>/Rh(1-e^2)'
   *  Sum: '<S92>/Sum4'
   */
  rtb_DataTypeConversion1_o *= 0.993305743F * rtb_ixj +
    AUAV3_AND_SLUGS_SENSOR_B.Switch2[2];

  /* Gain: '<S83>/Deg2R' */
  rtb_Sum_bt = 0.0174532924F *
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[1];

  /* Trigonometry: '<S83>/sin(phi)' */
  rtb_ixj = (real32_T)sin(rtb_Sum_bt);

  /* Sum: '<S83>/Sum1' incorporates:
   *  Constant: '<S83>/const'
   *  Product: '<S83>/Product1'
   *  Product: '<S83>/sin(phi)^2'
   */
  rtb_RhhcosphicoslambXe = 1.0F - rtb_ixj * rtb_ixj * 0.00669425726F;

  /* Fcn: '<S83>/f' */
  if (rtb_RhhcosphicoslambXe < 0.0F) {
    rtb_RhhcosphicoslambXe = -(real32_T)sqrt(-rtb_RhhcosphicoslambXe);
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)sqrt(rtb_RhhcosphicoslambXe);
  }

  /* End of Fcn: '<S83>/f' */

  /* Product: '<S83>/Rh' incorporates:
   *  Constant: '<S83>/Re=equatorial radius'
   */
  rtb_ixk = 6.378137E+6F / rtb_RhhcosphicoslambXe;

  /* Sum: '<S83>/Sum2' */
  rtb_RhhcosphisinlambYe =
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0] + rtb_ixk;

  /* Trigonometry: '<S83>/cos(phi)' */
  rtb_Sum_bt = (real32_T)cos(rtb_Sum_bt);

  /* Gain: '<S83>/Deg2R1' */
  rtb_Deg2R1 = 0.0174532924F *
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[2];

  /* Product: '<S83>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S83>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_Sum_bt * (real32_T)cos
    (rtb_Deg2R1);

  /* Product: '<S83>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S83>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_Sum_bt * (real32_T)sin
    (rtb_Deg2R1);

  /* Product: '<S83>/Ze' incorporates:
   *  Product: '<S83>/Rh(1-e^2)'
   *  Sum: '<S83>/Sum4'
   */
  rtb_ixj *= 0.993305743F * rtb_ixk +
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorMCU[0];

  /* SignalConversion: '<S91>/TmpSignal ConversionAtProduct1Inport1' incorporates:
   *  Fcn: '<S94>/11'
   *  Fcn: '<S94>/12'
   *  Fcn: '<S94>/13'
   *  Fcn: '<S94>/21'
   *  Fcn: '<S94>/22'
   *  Fcn: '<S94>/31'
   *  Fcn: '<S94>/32'
   *  Fcn: '<S94>/33'
   */
  tmp[0L] = (real32_T)cos(rtb_Deg2R_idx) * (real32_T)cos(rtb_Deg2R_idx_0);
  tmp[1L] = -(real32_T)sin(rtb_Deg2R_idx);
  tmp[2L] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx);
  tmp[3L] = (real32_T)sin(rtb_Deg2R_idx) * (real32_T)cos(rtb_Deg2R_idx_0);
  tmp[4L] = (real32_T)cos(rtb_Deg2R_idx);
  tmp[5L] = -(real32_T)sin(rtb_Deg2R_idx) * (real32_T)sin(rtb_Deg2R_idx_0);
  tmp[6L] = (real32_T)sin(rtb_Deg2R_idx_0);
  tmp[7L] = 0.0F;
  tmp[8L] = (real32_T)cos(rtb_Deg2R_idx_0);

  /* Sum: '<S27>/Sum1' */
  rtb_ixk = rtb_jxi - rtb_RhhcosphicoslambXe;
  rtb_Deg2R1 = rtb_kxj - rtb_RhhcosphisinlambYe;
  rtb_RhhcosphicoslambXe = rtb_DataTypeConversion1_o - rtb_ixj;

  /* Product: '<S91>/Product1' incorporates:
   *  Gain: '<S27>/UEN 2 NEU'
   */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    tmp_0[rtb_Gain21_h] = tmp[rtb_Gain21_h + 6] * rtb_RhhcosphicoslambXe +
      (tmp[rtb_Gain21_h + 3] * rtb_Deg2R1 + tmp[rtb_Gain21_h] * rtb_ixk);
  }

  /* End of Product: '<S91>/Product1' */

  /* Gain: '<S27>/UEN 2 NEU' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    rtb_UEN2NEU[rtb_Gain21_h] =
      AUAV3_AND_SLUGS_SENSOR_ConstP.UEN2NEU_Gain[rtb_Gain21_h + 6] * tmp_0[2] +
      (AUAV3_AND_SLUGS_SENSOR_ConstP.UEN2NEU_Gain[rtb_Gain21_h + 3] * tmp_0[1] +
       AUAV3_AND_SLUGS_SENSOR_ConstP.UEN2NEU_Gain[rtb_Gain21_h] * tmp_0[0]);
  }

  /* S-Function (MCHP_C_function_Call): '<S6>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  AUAV3_AND_SLUGS_SENSOR_B.ChecksifFixTypeis3updateSensorM = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S25>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.ChecksifFixTypeis3updateSensorM > 0) {
    /* Inport: '<S25>/In1' */
    AUAV3_AND_SLUGS_SENSOR_B.In1[0] = rtb_UEN2NEU[0];
    AUAV3_AND_SLUGS_SENSOR_B.In1[1] = rtb_UEN2NEU[1];
    AUAV3_AND_SLUGS_SENSOR_B.In1[2] = rtb_UEN2NEU[2];

    /* Inport: '<S25>/In2' */
    AUAV3_AND_SLUGS_SENSOR_B.In2 = AUAV3_AND_SLUGS_SENSOR_B.Switch2[3];

    /* Inport: '<S25>/In3' */
    AUAV3_AND_SLUGS_SENSOR_B.In3 = AUAV3_AND_SLUGS_SENSOR_B.Switch2[4];
  }

  /* End of Outputs for SubSystem: '<S6>/Enabled Subsystem' */

  /* Gain: '<S80>/Unit Conversion' */
  AUAV3_AND_SLUGS_SENSOR_B.UnitConversion = 0.0174532924F *
    AUAV3_AND_SLUGS_SENSOR_B.In2;

  /* S-Function (MCHP_C_function_Call): '<S81>/[apUtils.c]1' */
  AUAV3_AND_SLUGS_SENSOR_B.apUtilsc1 = myCos(
    AUAV3_AND_SLUGS_SENSOR_B.UnitConversion
    );

  /* S-Function (MCHP_C_function_Call): '<S81>/[apUtils.c]' */
  AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_e = mySin(
    AUAV3_AND_SLUGS_SENSOR_B.UnitConversion
    );

  /* MATLAB Function: '<S23>/myMux Fun2' incorporates:
   *  Constant: '<S23>/Constant'
   */
  /* MATLAB Function 'Position and Attitude Filter/COG.SOG2V/myMux Fun2': '<S82>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S82>:1:5' y = [u1(1); u2(1); u3(1)]; */
  rtb_UEN2NEU[0] = AUAV3_AND_SLUGS_SENSOR_B.apUtilsc1;
  rtb_UEN2NEU[1] = AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_e;
  rtb_UEN2NEU[2] = 0.0F;

  /* Sum: '<S32>/Sum' incorporates:
   *  Product: '<S50>/i x j'
   *  Product: '<S50>/j x k'
   *  Product: '<S50>/k x i'
   *  Product: '<S51>/i x k'
   *  Product: '<S51>/j x i'
   *  Product: '<S51>/k x j'
   */
  rtb_RhhcosphicoslambXe = rtb_RateLimiter_idx * 0.0F - rtb_RateLimiter_idx_0 *
    rtb_UEN2NEU[1];
  rtb_Deg2R1 = rtb_RateLimiter_idx_0 * rtb_UEN2NEU[0] - rtb_RateLimiter_idx_1 *
    0.0F;
  rtb_RateLimiter_idx_0 = rtb_RateLimiter_idx_1 * rtb_UEN2NEU[1] -
    rtb_RateLimiter_idx * rtb_UEN2NEU[0];

  /* Product: '<S22>/Product2' incorporates:
   *  Gain: '<S22>/Gain1'
   */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    tmp_0[rtb_Gain21_h] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 6] *
      rtb_RateLimiter_idx_0 +
      (AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 3] * rtb_Deg2R1
       + AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h] *
       rtb_RhhcosphicoslambXe);
  }

  /* End of Product: '<S22>/Product2' */

  /* Delay: '<S22>/Integer Delay' */
  rtb_Divide2_l_idx_1 = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[0];
  rtb_Divide2_l_idx = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[1];
  rtb_Divide2_l_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[2];

  /* Product: '<S23>/Product1' */
  rtb_Product1_b = AUAV3_AND_SLUGS_SENSOR_B.In3 *
    AUAV3_AND_SLUGS_SENSOR_B.apUtilsc1;

  /* Product: '<S23>/Product' */
  rtb_Product_e = AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_e *
    AUAV3_AND_SLUGS_SENSOR_B.In3;

  /* Abs: '<S85>/Abs1' incorporates:
   *  Sum: '<S88>/Diff'
   *  UnitDelay: '<S88>/UD'
   *
   * Block description for '<S88>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S88>/UD':
   *
   *  Store in Global RAM
   */
  rtb_ixk = (real32_T)fabs(AUAV3_AND_SLUGS_SENSOR_B.In1[0] -
    AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE);

  /* Abs: '<S86>/Abs1' incorporates:
   *  Sum: '<S89>/Diff'
   *  UnitDelay: '<S89>/UD'
   *
   * Block description for '<S89>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S89>/UD':
   *
   *  Store in Global RAM
   */
  rtb_RhhcosphicoslambXe = (real32_T)fabs(AUAV3_AND_SLUGS_SENSOR_B.In1[1] -
    AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE_i);

  /* Abs: '<S87>/Abs1' incorporates:
   *  Sum: '<S90>/Diff'
   *  UnitDelay: '<S90>/UD'
   *
   * Block description for '<S90>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S90>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Deg2R1 = (real32_T)fabs(AUAV3_AND_SLUGS_SENSOR_B.In1[2] -
    AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE_h);

  /* Sum: '<S26>/Sum' incorporates:
   *  Saturate: '<S85>/Saturation1'
   *  Saturate: '<S86>/Saturation1'
   *  Saturate: '<S87>/Saturation1'
   */
  rtb_ixk = ((rtb_ixk >= 1.0F ? 1.0F : rtb_ixk <= 0.0F ? 0.0F : rtb_ixk) +
             (rtb_RhhcosphicoslambXe >= 1.0F ? 1.0F : rtb_RhhcosphicoslambXe <=
              0.0F ? 0.0F : rtb_RhhcosphicoslambXe)) + (rtb_Deg2R1 >= 1.0F ?
    1.0F : rtb_Deg2R1 <= 0.0F ? 0.0F : rtb_Deg2R1);

  /* DataTypeConversion: '<S26>/Data Type Conversion2' incorporates:
   *  Saturate: '<S26>/Saturation1'
   */
  rtb_RhhcosphicoslambXe = (real32_T)floor(rtb_ixk >= 1.0F ? 1.0F : rtb_ixk <=
    0.0F ? 0.0F : rtb_ixk);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 256.0F);
  }

  rtb_DataTypeConversion2_ob = (uint8_T)rtb_RhhcosphicoslambXe;

  /* End of DataTypeConversion: '<S26>/Data Type Conversion2' */

  /* Outputs for Enabled SubSystem: '<S96>/Subsystem' incorporates:
   *  EnablePort: '<S104>/Enable'
   */
  /* RelationalOperator: '<S102>/Compare' incorporates:
   *  Constant: '<S102>/Constant'
   */
  if ((AUAV3_AND_SLUGS_SENSOR_B.In3 > 7.0F) > 0) {
    if (!AUAV3_AND_SLUGS_SENSOR_DWork.Subsystem_MODE) {
      AUAV3_AND_SLUGS_SENSOR_DWork.Subsystem_MODE = TRUE;
    }

    /* MATLAB Function: '<S104>/Embedded MATLAB Function1' */
    AU_EmbeddedMATLABFunction1(rtb_Product_e, rtb_DataTypeConversion2_ob,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_o,
      &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_o);

    /* MATLAB Function: '<S104>/Embedded MATLAB Function2' */
    AU_EmbeddedMATLABFunction1(rtb_Product1_b, rtb_DataTypeConversion2_ob,
      &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_n,
      &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_n);
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_DWork.Subsystem_MODE) {
      /* Disable for Outport: '<S104>/Vn_fil' */
      AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_n.y = 0.0F;

      /* Disable for Outport: '<S104>/Ve_fil' */
      AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_o.y = 0.0F;
      AUAV3_AND_SLUGS_SENSOR_DWork.Subsystem_MODE = FALSE;
    }
  }

  /* End of RelationalOperator: '<S102>/Compare' */
  /* End of Outputs for SubSystem: '<S96>/Subsystem' */

  /* Gain: '<S6>/[1 1 -1]' */
  rtb_ixj = AUAV3_AND_SLUGS_SENSOR_B.In1[0];
  rtb_DataTypeConversion1_o = AUAV3_AND_SLUGS_SENSOR_B.In1[1];

  /* MATLAB Function: '<S97>/Embedded MATLAB Function3' incorporates:
   *  Gain: '<S6>/[1 1 -1]'
   */
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3': '<S107>:1' */
  /*  persistent lastH; */
  /* '<S107>:1:7' if (isempty(lastGps_h)) */
  if (!AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h_not_empty) {
    /* '<S107>:1:8' lastGps_h       = single(0.0); */
    AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h_not_empty = TRUE;

    /* '<S107>:1:9' TimeSinceLast   = single(apSampleTime); */
    AUAV3_AND_SLUGS_SENSOR_DWork.TimeSinceLast = 0.01F;

    /* '<S107>:1:10' rate            = single(0); */
    /*      lastH           = single(-baseHeight); */
    /*      h_rate          = single(0); */
  }

  /* '<S107>:1:15' if (NewGPS) */
  if (rtb_DataTypeConversion2_ob != 0) {
    /* '<S107>:1:16' rate = single((gps_h - lastGps_h)/TimeSinceLast); */
    rtb_RhhcosphicoslambXe = (-AUAV3_AND_SLUGS_SENSOR_B.In1[2] -
      AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h) /
      AUAV3_AND_SLUGS_SENSOR_DWork.TimeSinceLast;

    /* '<S107>:1:17' TimeSinceLast = single(0); */
    AUAV3_AND_SLUGS_SENSOR_DWork.TimeSinceLast = 0.0F;

    /* '<S107>:1:18' if (rate > -10.6) && (rate < 17.67) */
    if (!(((real_T)rtb_RhhcosphicoslambXe > -10.6) && ((real_T)
          rtb_RhhcosphicoslambXe < 17.67))) {
      /* '<S107>:1:21' else */
      /*          h = lastH; */
      /* '<S107>:1:23' h_rate = single(0); */
      rtb_RhhcosphicoslambXe = 0.0F;
    } else {
      /*          h = lastH + gps_h - lastGps_h; */
      /* '<S107>:1:20' h_rate = rate; */
    }

    /* '<S107>:1:25' lastGps_h = gps_h; */
    AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h = -AUAV3_AND_SLUGS_SENSOR_B.In1[2];
  } else {
    /* '<S107>:1:26' else */
    /*      h = lastH; */
    /* '<S107>:1:28' h_rate = single(0); */
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* '<S107>:1:31' TimeSinceLast = TimeSinceLast + apSampleTime; */
  AUAV3_AND_SLUGS_SENSOR_DWork.TimeSinceLast =
    AUAV3_AND_SLUGS_SENSOR_DWork.TimeSinceLast + 0.01F;

  /* End of MATLAB Function: '<S97>/Embedded MATLAB Function3' */
  /*  lastH = h; */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
  /*  persistent lastGps_h; */
  /*  persistent TimeSinceLast; */
  /*  persistent rate; */
  /*  persistent lastH; */
  /*   */
  /*  if (isempty(lastGps_h)) */
  /*      lastGps_h       = single(0.0); */
  /*      TimeSinceLast   = single(apSampleTime); */
  /*      rate            = single(0); */
  /*      lastH           = single(-baseHeight); */
  /*      h_rate          = single(0); */
  /*  end */
  /*   */
  /*  if (NewGPS) */
  /*      rate = single((gps_h - lastGps_h)/TimeSinceLast); */
  /*      TimeSinceLast = single(0); */
  /*      if (rate > -10.6) && (rate < 17.67) */
  /*          h = lastH + gps_h - lastGps_h; */
  /*  %         h_rate = rate; */
  /*      else */
  /*          h = lastH; */
  /*  %         h_rate = single(0); */
  /*      end */
  /*      lastGps_h = gps_h; */
  /*  else */
  /*      h = lastH; */
  /*  %     h_rate = single(0); */
  /*  end */
  /*   */
  /*  TimeSinceLast = TimeSinceLast + apSampleTime; */
  /*  lastH = h; */

  /* S-Function (MCHP_C_function_Call): '<S200>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorM_j
                );

  /* MATLAB Function: '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e,
     &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_e);

  /* Outputs for Enabled SubSystem: '<S186>/Zero Out Height' */
  AUAV3_AND_SL_ZeroOutHeight
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut,
     AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorM_j[0],
     &AUAV3_AND_SLUGS_SENSOR_B.ZeroOutHeight_d,
     &AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight_d);

  /* End of Outputs for SubSystem: '<S186>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S9>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S151>/Enable'
   */
  if (rtb_EnableHILfromControlMCU1) {
    /* S-Function (MCHP_C_function_Call): '<S151>/Data from HIL [hil.c]2' */
    hilRead(
            AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2
            );

    /* S-Function (MCHP_C_function_Call): '<S151>/HIL Messages  Parser//Decoder [hil.c]1' */
    protDecodeHil(
                  AUAV3_AND_SLUGS_SENSOR_B.DatafromHILhilc2
                  );

    /* S-Function (MCHP_C_function_Call): '<S151>/HIL Raw Readings [hil.c]1' */
    hil_getRawRead(
                   AUAV3_AND_SLUGS_SENSOR_B.HILMessagesParserDecoderhilc1
                   );
  }

  /* End of Outputs for SubSystem: '<S9>/Raw HIL  Readings' */

  /* Switch: '<S152>/Switch' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 13; rtb_Gain21_h++) {
    if (rtb_EnableHILfromControlMCU1) {
      rtb_Switch_o[rtb_Gain21_h] =
        AUAV3_AND_SLUGS_SENSOR_B.HILRawReadingshilc1[rtb_Gain21_h];
    } else {
      rtb_Switch_o[rtb_Gain21_h] = AUAV3_AND_SLUGS_SENSOR_B.y_h[rtb_Gain21_h];
    }
  }

  /* End of Switch: '<S152>/Switch' */

  /* DataTypeConversion: '<S157>/Data Type Conversion5' */
  rtb_DataTypeConversion5 = (real_T)rtb_Switch_o[12];

  /* MATLAB Function: '<S193>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion5, 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_a,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_a);

  /* Sum: '<S192>/Sum' incorporates:
   *  Constant: '<S192>/Bias'
   *  Constant: '<S192>/Gains'
   *  Product: '<S192>/Divide'
   */
  rtb_Sum_jv = (real32_T)(1.5112853050231934 *
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_a.y) + -1605.28198F;

  /* RelationalOperator: '<S208>/Compare' incorporates:
   *  Constant: '<S208>/Constant'
   */
  rtb_Compare_b = (rtb_Sum_jv < -50.0F);

  /* Outputs for Enabled SubSystem: '<S191>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S209>/Enable'
   */
  /* Logic: '<S191>/Logical Operator' */
  if (!(rtb_Compare_b != 0)) {
    /* Sum: '<S209>/Add' incorporates:
     *  Constant: '<S209>/Constant'
     *  Constant: '<S209>/Mean Temperature for Calibration'
     *  Constant: '<S209>/gains'
     *  Product: '<S209>/Divide1'
     *  Sum: '<S209>/Sum1'
     *  Sum: '<S209>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = ((real32_T)rtb_Switch_o[9] - (rtb_Sum_jv -
      347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S191>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S191>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S191>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S210>/Enable'
   */
  if (rtb_Compare_b > 0) {
    /* Sum: '<S210>/Sum2' incorporates:
     *  Constant: '<S210>/Mean Temperature for Calibration'
     *  Constant: '<S210>/gains'
     *  Product: '<S210>/Divide1'
     *  Sum: '<S210>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge = (real32_T)rtb_Switch_o[9] - (rtb_Sum_jv -
      -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S191>/Lo Temp Compensation' */

  /* DataTypeConversion: '<S157>/Data Type Conversion19' */
  rtb_DataTypeConversion19 = AUAV3_AND_SLUGS_SENSOR_B.Merge;

  /* MATLAB Function: '<S196>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion19, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

  /* Sum: '<S187>/Sum' incorporates:
   *  Constant: '<S187>/Bias'
   *  Constant: '<S187>/Gains'
   *  DataTypeConversion: '<S157>/Data Type Conversion1'
   *  Product: '<S187>/Divide'
   */
  rtb_Sum_e1 = 27.127F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_o.y + 9444.44434F;

  /* Outputs for Enabled SubSystem: '<S186>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S201>/Enable'
   */
  if (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut > 0.0) {
    /* DataTypeConversion: '<S201>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Sum_e1;

    /* DiscreteZeroPole: '<S204>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_j = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole_j += 0.029119852459414206*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_m;
    }

    /* Saturate: '<S201>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S201>/Data Type Conversion1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_j >=
      120000.0F ? 120000.0F : (real32_T)rtb_DiscreteZeroPole_j <= 80000.0F ?
      80000.0F : (real32_T)rtb_DiscreteZeroPole_j;

    /* Update for DiscreteZeroPole: '<S204>/Discrete Zero-Pole' */
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_m =
        rtb_DataTypeConversion + 0.97044334975369462*
        AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_m;
    }
  }

  /* End of Outputs for SubSystem: '<S186>/Initial Baro Bias' */

  /* Product: '<S197>/Divide' incorporates:
   *  Sum: '<S197>/Sum2'
   */
  rtb_Sum_bt = (rtb_Sum_e1 - AUAV3_AND_SLUGS_SENSOR_B.u0k120k) /
    AUAV3_AND_SLUGS_SENSOR_B.u0k120k;

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
  AUAV3_AND_SLUGS_SENSOR_B.Sum1_b = ((rtb_Sum_bt * rtb_Sum_bt * 0.093502529F +
    rtb_Sum_bt * -0.188893303F) + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV3_AND_SLUGS_SENSOR_B.GettheGSLocationupdateSensorM_j[0];

  /* Sum: '<S186>/Sum1' */
  rtb_Sum1_f = AUAV3_AND_SLUGS_SENSOR_B.ZeroOutHeight_d.Sum +
    AUAV3_AND_SLUGS_SENSOR_B.Sum1_b;

  /* Logic: '<S186>/Logical Operator' */
  rtb_LogicalOperator_f =
    !(AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut != 0.0);

  /* Outputs for Enabled SubSystem: '<S186>/Enabled Subsystem' */
  AUAV3_AND_EnabledSubsystem(rtb_LogicalOperator_f, rtb_Sum1_f,
    &AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_o);

  /* End of Outputs for SubSystem: '<S186>/Enabled Subsystem' */

  /* Sum: '<S97>/Sum' incorporates:
   *  Gain: '<S96>/Gain'
   *  Gain: '<S97>/Gain'
   */
  rtb_RhhcosphicoslambXe = 0.5F * rtb_RhhcosphicoslambXe +
    -AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_o.In1;

  /* MATLAB Function: '<S108>/Embedded MATLAB Function' */
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4/Embedded MATLAB Function': '<S109>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S109>:1:9' if isempty(a) */
  if (!AUAV3_AND_SLUGS_SENSOR_DWork.a_not_empty) {
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S109>:1:12' omega = 2*pi*f; */
    /* '<S109>:1:13' a=T*omega/(2+T*omega); */
    AUAV3_AND_SLUGS_SENSOR_DWork.a = 0.0099009900990099011;
    AUAV3_AND_SLUGS_SENSOR_DWork.a_not_empty = TRUE;

    /* '<S109>:1:14' b=-(T*omega-2)/(T*omega+2); */
    AUAV3_AND_SLUGS_SENSOR_DWork.b = 0.98019801980198018;

    /* '<S109>:1:15' y_km1=u; */
    AUAV3_AND_SLUGS_SENSOR_DWork.y_km1 = rtb_RhhcosphicoslambXe;

    /* '<S109>:1:16' u_km1=u; */
    AUAV3_AND_SLUGS_SENSOR_DWork.u_km1 = rtb_RhhcosphicoslambXe;
  }

  /* '<S109>:1:19' y = a*(u+u_km1)+b*y_km1; */
  rtb_kxj = (rtb_RhhcosphicoslambXe + AUAV3_AND_SLUGS_SENSOR_DWork.u_km1) *
    (real32_T)AUAV3_AND_SLUGS_SENSOR_DWork.a + (real32_T)
    AUAV3_AND_SLUGS_SENSOR_DWork.b * AUAV3_AND_SLUGS_SENSOR_DWork.y_km1;

  /* '<S109>:1:20' y_km1=y; */
  AUAV3_AND_SLUGS_SENSOR_DWork.y_km1 = rtb_kxj;

  /* '<S109>:1:21' u_km1=u; */
  AUAV3_AND_SLUGS_SENSOR_DWork.u_km1 = rtb_RhhcosphicoslambXe;

  /* End of MATLAB Function: '<S108>/Embedded MATLAB Function' */

  /* Gain: '<S101>/Gain' incorporates:
   *  Sum: '<S101>/Sum1'
   *  UnitDelay: '<S101>/Unit Delay'
   */
  rtb_Gain_k = (rtb_kxj - AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE) * 2.0F;

  /* MATLAB Function: '<S96>/myMux Fun2' */
  AUAV3_AND_SLUG_myMuxFun1_p
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_n.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_o.y, rtb_Gain_k,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a);

  /* Product: '<S22>/Product1' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    Product[rtb_Gain21_h] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 6] *
      AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[2] +
      (AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 3] *
       AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[1] +
       AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h] *
       AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S22>/Product1' */

  /* Gain: '<S33>/Gain2' incorporates:
   *  Delay: '<S33>/Integer Delay1'
   *  Sum: '<S33>/Sum5'
   */
  rtb_UEN2NEU[0] = (Product[0] -
                    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[0]) *
    20.0F;
  rtb_UEN2NEU[1] = (Product[1] -
                    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[1]) *
    20.0F;
  rtb_UEN2NEU[2] = (Product[2] -
                    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[2]) *
    20.0F;

  /* DataTypeConversion: '<S153>/Data Type Conversion1' incorporates:
   *  Constant: '<S153>/Gyro Gains1'
   *  Product: '<S153>/Divide1'
   */
  rtb_DataTypeConversion1[0] = (real32_T)rtb_Switch_o[4] * 0.0326839499F;
  rtb_DataTypeConversion1[1] = (real32_T)rtb_Switch_o[3] * 0.0326839499F;
  rtb_DataTypeConversion1[2] = (real32_T)rtb_Switch_o[5] * 0.0326839499F;

  /* MATLAB Function: '<S161>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion1[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_h,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_h);

  /* MATLAB Function: '<S161>/Embedded MATLAB Function1' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion1[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S161>/Embedded MATLAB Function2' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion1[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

  /* MATLAB Function: '<S161>/myMux Fun' */
  AUAV3_AND_SLUGS_S_myMuxFun
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_h.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2.y,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun);

  /* DataTypeConversion: '<S153>/Data Type Conversion3' */
  rtb_RhhcosphicoslambXe = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[0];
  rtb_Deg2R1 = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[1];
  rtb_ixk = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[2];

  /* Product: '<S96>/Product1' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    Product1_o[rtb_Gain21_h] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 6] *
      AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[2] +
      (AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h + 3] *
       AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[1] +
       AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[rtb_Gain21_h] *
       AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun2_a.y[0]);
  }

  /* End of Product: '<S96>/Product1' */

  /* Sum: '<S96>/Sum' incorporates:
   *  DataTypeConversion: '<S153>/Data Type Conversion3'
   *  Delay: '<S103>/Integer Delay1'
   *  Gain: '<S103>/Gain2'
   *  Sum: '<S103>/Sum5'
   */
  rtb_RateLimiter_idx_1 = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[0] -
    (Product1_o[0] - AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[0]) *
    20.0F;
  rtb_RateLimiter_idx = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[1] -
    (Product1_o[1] - AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[1]) *
    20.0F;
  rtb_RateLimiter_idx_0 = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun.y[2] -
    (Product1_o[2] - AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[2]) *
    20.0F;

  /* RateLimiter: '<S96>/Rate Limiter' */
  rtb_RhhcosphisinlambYe = rtb_RateLimiter_idx_1 -
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[0];
  rtb_jxi = rtb_RateLimiter_idx - AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[1];
  rtb_Deg2R_idx = rtb_RateLimiter_idx_0 - AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[2];
  rtb_Deg2R_idx_0 = rtb_RateLimiter_idx_1;
  if (rtb_RhhcosphisinlambYe > 9.896E-9F) {
    rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[0] + 9.896E-9F;
  } else {
    if (rtb_RhhcosphisinlambYe < -9.896E-9F) {
      rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[0] + -9.896E-9F;
    }
  }

  rtb_RateLimiter_idx_1 = rtb_Deg2R_idx_0;

  /* RateLimiter: '<S96>/Rate Limiter' */
  rtb_Deg2R_idx_0 = rtb_RateLimiter_idx;
  if (rtb_jxi > 9.896E-9F) {
    rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[1] + 9.896E-9F;
  } else {
    if (rtb_jxi < -9.896E-9F) {
      rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[1] + -9.896E-9F;
    }
  }

  rtb_RateLimiter_idx = rtb_Deg2R_idx_0;

  /* RateLimiter: '<S96>/Rate Limiter' */
  rtb_Deg2R_idx_0 = rtb_RateLimiter_idx_0;
  if (rtb_Deg2R_idx > 9.896E-9F) {
    rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[2] + 9.896E-9F;
  } else {
    if (rtb_Deg2R_idx < -9.896E-9F) {
      rtb_Deg2R_idx_0 = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[2] + -9.896E-9F;
    }
  }

  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[0] = rtb_RateLimiter_idx_1;
  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[1] = rtb_RateLimiter_idx;
  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[2] = rtb_Deg2R_idx_0;

  /* DataTypeConversion: '<S98>/Data Type Conversion2' */
  rtb_DataTypeConversion2[0] = rtb_RateLimiter_idx_1;
  rtb_DataTypeConversion2[1] = rtb_RateLimiter_idx;
  rtb_DataTypeConversion2[2] = rtb_Deg2R_idx_0;

  /* DiscreteZeroPole: '<S110>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole = 9.8940417712526327E-7*rtb_DataTypeConversion2[0];
    rtb_DiscreteZeroPole += 3.9156825028515473E-10*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
  }

  /* DiscreteZeroPole: '<S111>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_i = 9.8940417712526327E-7*rtb_DataTypeConversion2[1];
    rtb_DiscreteZeroPole_i += 3.9156825028515473E-10*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* DiscreteZeroPole: '<S112>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_d = 9.8940417712526327E-7*rtb_DataTypeConversion2[2];
    rtb_DiscreteZeroPole_d += 3.9156825028515473E-10*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* MATLAB Function: '<S98>/myMux Fun1' */
  AUAV3_AND_SLUGS__myMuxFun1(rtb_DiscreteZeroPole, rtb_DiscreteZeroPole_i,
    rtb_DiscreteZeroPole_d, &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1);

  /* DataTypeConversion: '<S98>/Data Type Conversion1' */
  rtb_DataTypeConversion1_a[0] = (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1.y[0];
  rtb_DataTypeConversion1_a[1] = (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1.y[1];
  rtb_DataTypeConversion1_a[2] = (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1.y[2];

  /* Sum: '<S22>/Add' incorporates:
   *  Product: '<S48>/i x j'
   *  Product: '<S48>/j x k'
   *  Product: '<S48>/k x i'
   *  Product: '<S49>/i x k'
   *  Product: '<S49>/j x i'
   *  Product: '<S49>/k x j'
   *  Sum: '<S31>/Sum'
   */
  rtb_Add[0] = (((rtb_Divide2_l_idx * Product[2] - rtb_Divide2_l_idx_0 *
                  Product[1]) + rtb_UEN2NEU[0]) - rtb_RhhcosphicoslambXe) -
    rtb_DataTypeConversion1_a[0];
  rtb_Add[1] = (((rtb_Divide2_l_idx_0 * Product[0] - rtb_Divide2_l_idx_1 *
                  Product[2]) + rtb_UEN2NEU[1]) - rtb_Deg2R1) -
    rtb_DataTypeConversion1_a[1];
  rtb_Add[2] = (((rtb_Divide2_l_idx_1 * Product[1] - rtb_Divide2_l_idx *
                  Product[0]) + rtb_UEN2NEU[2]) - rtb_ixk) -
    rtb_DataTypeConversion1_a[2];

  /* MATLAB Function: '<S57>/Embedded MATLAB Function' */
  AUA_EmbeddedMATLABFunction(rtb_Add,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_e);

  /* MATLAB Function: '<S56>/negprotect' */
  AUAV3_AND_SLUGS_negprotect
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_e.xDoty,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_negprotect_m);

  /* S-Function (MCHP_C_function_Call): '<S56>/[apUtils.c]' */
  AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_n = mySqrt(
    AUAV3_AND_SLUGS_SENSOR_B.sf_negprotect_m.zpVal
    );

  /* Saturate: '<S36>/Zero Bound' */
  rtb_RhhcosphicoslambXe = AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_n >= 0.001F ?
    AUAV3_AND_SLUGS_SENSOR_B.apUtilsc_n : 0.001F;

  /* Product: '<S36>/Divide' */
  rtb_Divide2_l_idx_1 = rtb_Add[0] / rtb_RhhcosphicoslambXe;
  rtb_Divide2_l_idx = rtb_Add[1] / rtb_RhhcosphicoslambXe;
  rtb_Divide2_l_idx_0 = rtb_Add[2] / rtb_RhhcosphicoslambXe;

  /* S-Function (sdspsubmtrx): '<S22>/Submatrix' */
  rtb_Bias5 = 0L;
  rtb_Gain21_h = 2;
  while (rtb_Gain21_h <= 2) {
    AUAV3_AND_SLUGS_SENSOR_B.g_hat[rtb_Bias5] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[6];
    AUAV3_AND_SLUGS_SENSOR_B.g_hat[1L + rtb_Bias5] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[7];
    AUAV3_AND_SLUGS_SENSOR_B.g_hat[2L + rtb_Bias5] =
      AUAV3_AND_SLUGS_SENSOR_B.VectorConcatenate[8];
    rtb_Bias5 += 3L;
    rtb_Gain21_h = 3;
  }

  /* End of S-Function (sdspsubmtrx): '<S22>/Submatrix' */

  /* Sum: '<S30>/Sum' incorporates:
   *  Product: '<S46>/i x j'
   *  Product: '<S47>/i x k'
   *  Product: '<S47>/j x i'
   */
  rtb_RhhcosphicoslambXe = rtb_Divide2_l_idx_1 * AUAV3_AND_SLUGS_SENSOR_B.g_hat
    [1];
  rtb_Deg2R1 = rtb_Divide2_l_idx_1 * AUAV3_AND_SLUGS_SENSOR_B.g_hat[2];
  rtb_ixk = rtb_Divide2_l_idx * AUAV3_AND_SLUGS_SENSOR_B.g_hat[0];

  /* Sum: '<S22>/Sum4' incorporates:
   *  Gain: '<S22>/Gain1'
   *  Product: '<S46>/j x k'
   *  Product: '<S46>/k x i'
   *  Product: '<S47>/k x j'
   *  Sum: '<S30>/Sum'
   */
  rtb_Divide2_l_idx_1 = (rtb_Divide2_l_idx * AUAV3_AND_SLUGS_SENSOR_B.g_hat[2] -
    rtb_Divide2_l_idx_0 * AUAV3_AND_SLUGS_SENSOR_B.g_hat[1]) + 30.0F * tmp_0[0];
  rtb_Divide2_l_idx = (rtb_Divide2_l_idx_0 * AUAV3_AND_SLUGS_SENSOR_B.g_hat[0] -
                       rtb_Deg2R1) + 30.0F * tmp_0[1];
  rtb_Divide2_l_idx_0 = 30.0F * tmp_0[2] + (rtb_RhhcosphicoslambXe - rtb_ixk);

  /* DataTypeConversion: '<S29>/Data Type Conversion2' */
  rtb_DataTypeConversion2_b[0] = rtb_Divide2_l_idx_1;
  rtb_DataTypeConversion2_b[1] = rtb_Divide2_l_idx;
  rtb_DataTypeConversion2_b[2] = rtb_Divide2_l_idx_0;

  /* DiscreteZeroPole: '<S42>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_n = 0.005*rtb_DataTypeConversion2_b[0];
    rtb_DiscreteZeroPole_n += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_p;
  }

  /* DiscreteZeroPole: '<S43>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_g = 0.005*rtb_DataTypeConversion2_b[1];
    rtb_DiscreteZeroPole_g += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o2;
  }

  /* DiscreteZeroPole: '<S44>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_p = 0.005*rtb_DataTypeConversion2_b[2];
    rtb_DiscreteZeroPole_p += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_d;
  }

  /* MATLAB Function: '<S29>/myMux Fun1' */
  AUAV3_AND_SLUGS__myMuxFun1(rtb_DiscreteZeroPole_n, rtb_DiscreteZeroPole_g,
    rtb_DiscreteZeroPole_p, &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j);

  /* RateLimiter: '<S22>/Bias Rate Limiter' incorporates:
   *  DataTypeConversion: '<S29>/Data Type Conversion1'
   */
  rtb_BiasRateLimiter[0] = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[0]
    - AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[0];
  rtb_BiasRateLimiter[1] = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[1]
    - AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[1];
  rtb_BiasRateLimiter[2] = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[2]
    - AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[2];
  if (rtb_BiasRateLimiter[0] > 2.5572E-9F) {
    rtb_BiasRateLimiter[0] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[0] +
      2.5572E-9F;
  } else if (rtb_BiasRateLimiter[0] < -2.5572E-9F) {
    rtb_BiasRateLimiter[0] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[0] +
      -2.5572E-9F;
  } else {
    rtb_BiasRateLimiter[0] = (real32_T)
      AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[0];
  }

  if (rtb_BiasRateLimiter[1] > 2.5572E-9F) {
    rtb_BiasRateLimiter[1] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[1] +
      2.5572E-9F;
  } else if (rtb_BiasRateLimiter[1] < -2.5572E-9F) {
    rtb_BiasRateLimiter[1] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[1] +
      -2.5572E-9F;
  } else {
    rtb_BiasRateLimiter[1] = (real32_T)
      AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[1];
  }

  if (rtb_BiasRateLimiter[2] > 2.5572E-9F) {
    rtb_BiasRateLimiter[2] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[2] +
      2.5572E-9F;
  } else if (rtb_BiasRateLimiter[2] < -2.5572E-9F) {
    rtb_BiasRateLimiter[2] = AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[2] +
      -2.5572E-9F;
  } else {
    rtb_BiasRateLimiter[2] = (real32_T)
      AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_j.y[2];
  }

  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[0] = rtb_BiasRateLimiter[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[1] = rtb_BiasRateLimiter[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[2] = rtb_BiasRateLimiter[2];

  /* End of RateLimiter: '<S22>/Bias Rate Limiter' */

  /* Gain: '<S22>/Gain' */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S35>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S35>:1:5' if ( u < 0 ) */
  rtb_UEN2NEU[0] = 0.1F * rtb_Divide2_l_idx_1;
  rtb_UEN2NEU[1] = 0.1F * rtb_Divide2_l_idx;
  rtb_UEN2NEU[2] = 0.1F * rtb_Divide2_l_idx_0;

  /* DataTypeConversion: '<S153>/Data Type Conversion2' incorporates:
   *  Constant: '<S153>/Gyro Gains'
   *  Gain: '<S163>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide'
   */
  rtb_DataTypeConversion2_d[0] = -((real32_T)rtb_Switch_o[1] * 0.000872664619F);
  rtb_DataTypeConversion2_d[1] = -((real32_T)rtb_Switch_o[0] * 0.000872664619F);
  rtb_DataTypeConversion2_d[2] = -((real32_T)rtb_Switch_o[2] * 0.000872664619F);

  /* MATLAB Function: '<S160>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion2_d[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_k,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_k);

  /* MATLAB Function: '<S160>/Embedded MATLAB Function1' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion2_d[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_m,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_m);

  /* MATLAB Function: '<S160>/Embedded MATLAB Function2' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion2_d[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_p,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_p);

  /* MATLAB Function: '<S160>/myMux Fun' */
  AUAV3_AND_SLUGS_S_myMuxFun
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_k.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_m.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_p.y,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun_g);

  /* Sum: '<S22>/Sum1' incorporates:
   *  DataTypeConversion: '<S153>/Data Type Conversion7'
   *  Sum: '<S22>/Sum3'
   */
  rtb_UEN2NEU[0] += (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun_g.y[0] +
    rtb_BiasRateLimiter[0];
  rtb_UEN2NEU[1] += (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun_g.y[1] +
    rtb_BiasRateLimiter[1];
  rtb_UEN2NEU[2] += (real32_T)AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun_g.y[2] +
    rtb_BiasRateLimiter[2];

  /* MATLAB Function: '<S22>/q dot calc' incorporates:
   *  SignalConversion: '<S41>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/q dot calc': '<S41>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S41>:1:5' s = q(1,1); */
  /* '<S41>:1:6' v = q(2:4,1); */
  /* '<S41>:1:7' q_dot = single(zeros(4,1)); */
  rtb_q_dot[0] = 0.0F;
  rtb_q_dot[1] = 0.0F;
  rtb_q_dot[2] = 0.0F;
  rtb_q_dot[3] = 0.0F;

  /* '<S41>:1:8' q_dot(1,1) = -.5*v'*om_hat; */
  rtb_q_dot[0] = (-0.5F * rtb_Product1_j * rtb_UEN2NEU[0] + -0.5F *
                  rtb_Product2_c * rtb_UEN2NEU[1]) + -0.5F * rtb_Product3_d *
    rtb_UEN2NEU[2];

  /* '<S41>:1:9' q_dot(2:4,1) =  .5*[ s    -v(3) v(2); ... */
  /* '<S41>:1:10'                         v(3)  s    -v(1); ... */
  /* '<S41>:1:11'                        -v(2) v(1) s]*om_hat; */
  rtb_TmpSignalConversionAtSFun_0[0] = rtb_Product_p;
  rtb_TmpSignalConversionAtSFun_0[3] = -rtb_Product3_d;
  rtb_TmpSignalConversionAtSFun_0[6] = rtb_Product2_c;
  rtb_TmpSignalConversionAtSFun_0[1] = rtb_Product3_d;
  rtb_TmpSignalConversionAtSFun_0[4] = rtb_Product_p;
  rtb_TmpSignalConversionAtSFun_0[7] = -rtb_Product1_j;
  rtb_TmpSignalConversionAtSFun_0[2] = -rtb_Product2_c;
  rtb_TmpSignalConversionAtSFun_0[5] = rtb_Product1_j;
  rtb_TmpSignalConversionAtSFun_0[8] = rtb_Product_p;
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    tmp[3 * rtb_Gain21_h] = rtb_TmpSignalConversionAtSFun_0[3 * rtb_Gain21_h] *
      0.5F;
    tmp[1 + 3 * rtb_Gain21_h] = rtb_TmpSignalConversionAtSFun_0[3 * rtb_Gain21_h
      + 1] * 0.5F;
    tmp[2 + 3 * rtb_Gain21_h] = rtb_TmpSignalConversionAtSFun_0[3 * rtb_Gain21_h
      + 2] * 0.5F;
  }

  for (rtb_Gain21_h = 0; rtb_Gain21_h < 3; rtb_Gain21_h++) {
    rtb_q_dot[(int32_T)(1 + rtb_Gain21_h)] = 0.0F;
    rtb_q_dot[(int32_T)(1 + rtb_Gain21_h)] += tmp[rtb_Gain21_h] * rtb_UEN2NEU[0];
    rtb_q_dot[(int32_T)(1 + rtb_Gain21_h)] += tmp[rtb_Gain21_h + 3] *
      rtb_UEN2NEU[1];
    rtb_q_dot[(int32_T)(1 + rtb_Gain21_h)] += tmp[rtb_Gain21_h + 6] *
      rtb_UEN2NEU[2];
  }

  /* End of MATLAB Function: '<S22>/q dot calc' */

  /* DataTypeConversion: '<S114>/Data Type Conversion2' incorporates:
   *  Gain: '<S99>/Gain'
   *  Sum: '<S99>/Sum'
   *  Sum: '<S99>/Sum1'
   *  UnitDelay: '<S99>/Unit Delay'
   */
  rtb_DataTypeConversion2_k = (rtb_ixj -
    AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE_c) * 10.0F + rtb_Product1_b;

  /* DiscreteZeroPole: '<S115>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = 0.005*rtb_DataTypeConversion2_k;
    rtb_RoundingFunction += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_f;
  }

  /* DataTypeConversion: '<S114>/Data Type Conversion1' */
  rtb_DataTypeConversion1_d = (real32_T)rtb_RoundingFunction;

  /* DataTypeConversion: '<S116>/Data Type Conversion2' incorporates:
   *  Gain: '<S100>/Gain'
   *  Sum: '<S100>/Sum'
   *  Sum: '<S100>/Sum1'
   *  UnitDelay: '<S100>/Unit Delay'
   */
  rtb_DataTypeConversion2_p = (rtb_DataTypeConversion1_o -
    AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE_a) * 10.0F + rtb_Product_e;

  /* DiscreteZeroPole: '<S117>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = 0.005*rtb_DataTypeConversion2_p;
    rtb_RoundingFunction += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_fl;
  }

  /* DataTypeConversion: '<S116>/Data Type Conversion1' */
  rtb_DataTypeConversion1_dq = (real32_T)rtb_RoundingFunction;

  /* DataTypeConversion: '<S118>/Data Type Conversion2' */
  rtb_DataTypeConversion2_bt = rtb_Gain_k;

  /* DiscreteZeroPole: '<S119>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = 0.005*rtb_DataTypeConversion2_bt;
    rtb_RoundingFunction += 0.01*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_db;
  }

  /* DataTypeConversion: '<S118>/Data Type Conversion1' */
  rtb_Deg2R1 = (real32_T)rtb_RoundingFunction;

  /* Gain: '<S96>/Gain1' */
  rtb_Gain1_o = -rtb_kxj;

  /* MATLAB Function: '<S96>/myMux Fun1' */
  AUAV3_AND_SLUG_myMuxFun1_p(rtb_DataTypeConversion1_d,
    rtb_DataTypeConversion1_dq, rtb_Gain1_o,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun1_p);

  /* RelationalOperator: '<S205>/Compare' incorporates:
   *  Constant: '<S205>/Constant'
   */
  rtb_Compare_b = (rtb_Sum_jv < -130.0F);

  /* Outputs for Enabled SubSystem: '<S190>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S206>/Enable'
   */
  /* Logic: '<S190>/Logical Operator' */
  if (!(rtb_Compare_b != 0)) {
    /* Sum: '<S206>/Sum2' incorporates:
     *  Constant: '<S206>/Mean Temperature for Calibration'
     *  Constant: '<S206>/gains'
     *  Product: '<S206>/Divide1'
     *  Sum: '<S206>/Sum1'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_p = (real32_T)rtb_Switch_o[10] - (rtb_Sum_jv
      - 293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S190>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S190>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S190>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S207>/Enable'
   */
  if (rtb_Compare_b > 0) {
    /* Sum: '<S207>/Add' incorporates:
     *  Constant: '<S207>/Constant'
     *  Constant: '<S207>/Mean Temperature for Calibration'
     *  Constant: '<S207>/gains'
     *  Product: '<S207>/Divide1'
     *  Sum: '<S207>/Sum1'
     *  Sum: '<S207>/Sum2'
     */
    AUAV3_AND_SLUGS_SENSOR_B.Merge_p = ((real32_T)rtb_Switch_o[10] - (rtb_Sum_jv
      - -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S190>/Lo Temp Compensation' */

  /* DataTypeConversion: '<S157>/Data Type Conversion3' */
  rtb_DataTypeConversion3 = AUAV3_AND_SLUGS_SENSOR_B.Merge_p;

  /* MATLAB Function: '<S195>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion3, 0.01, 4.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_l,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_l);

  /* Sum: '<S188>/Sum' incorporates:
   *  Constant: '<S188>/Bias'
   *  Constant: '<S188>/Gains'
   *  DataTypeConversion: '<S157>/Data Type Conversion4'
   *  Product: '<S188>/Divide'
   */
  rtb_ixk = 1.05137849F * (real32_T)
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_l.y + -1005.87872F;

  /* Saturate: '<S157>/[0.001  maxDynPress]' */
  rtb_RhhcosphicoslambXe = rtb_ixk >= 3000.0F ? 3000.0F : rtb_ixk <= 0.001F ?
    0.001F : rtb_ixk;

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
    AUAV3_AND_SLUGS_SENSOR_B.AirData[0] = rtb_RhhcosphicoslambXe;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[1] = rtb_Sum_e1;
    AUAV3_AND_SLUGS_SENSOR_B.AirData[2] = rtb_Sum_jv;

    /* S-Function (MCHP_C_function_Call): '<S156>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
    updateAirData(
                  AUAV3_AND_SLUGS_SENSOR_B.AirData
                  );
  }

  /* End of Outputs for SubSystem: '<S152>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S152>/myMux Fun1' */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun1': '<S159>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S159>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
  AUAV3_AND_SLUGS_SENSOR_B.y[0] = rtb_RhhcosphicoslambXe;
  AUAV3_AND_SLUGS_SENSOR_B.y[1] = AUAV3_AND_SLUGS_SENSOR_B.AirData[0];
  AUAV3_AND_SLUGS_SENSOR_B.y[2] = 0.0F;
  AUAV3_AND_SLUGS_SENSOR_B.y[3] = 0.0F;

  /* S-Function (MCHP_C_function_Call): '<S152>/Sensor DSC Diag [updateSensorMcuState.c]1' */
  updateSensorDiag(
                   AUAV3_AND_SLUGS_SENSOR_B.y
                   );

  /* S-Function "MCHP_MCU_LOAD" Block: <S154>/Calculus Time Step1 */
  AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1 = MCHP_MCULoadResult[0];

  /* DataTypeConversion: '<S154>/Data Type Conversion2' */
  rtb_Sum_bt = (real32_T)AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep1;

  /* S-Function "MCHP_MCU_OVERLOAD" Block: <S154>/Calculus Time Step2 */
  {
    uint16_T register tmp = MCHP_MCU_Overload.val;
    MCHP_MCU_Overload.val ^= tmp;      /* Multi Tasking: potential simultaneous access ==> using xor to protect from potential miss */
    AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2 = tmp;
  }

  /* Product: '<S154>/Divide' incorporates:
   *  DataTypeConversion: '<S154>/Data Type Conversion1'
   */
  rtb_RoundingFunction = (real_T)rtb_Sum_bt / (real_T)
    AUAV3_AND_SLUGS_SENSOR_B.CalculusTimeStep2;

  /* Rounding: '<S154>/Rounding Function' incorporates:
   *  Gain: '<S154>/Gain'
   */
  rtb_RoundingFunction = floor(100.0 * rtb_RoundingFunction);

  /* DataTypeConversion: '<S154>/Data Type Conversion12' */
  tmp_1 = floor(rtb_RoundingFunction);
  if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
    tmp_1 = 0.0;
  } else {
    tmp_1 = fmod(tmp_1, 256.0);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12 = (uint8_T)(tmp_1 < 0.0 ?
    (uint8_T)(int16_T)(int8_T)-(int8_T)(uint8_T)-tmp_1 : (uint8_T)tmp_1);

  /* End of DataTypeConversion: '<S154>/Data Type Conversion12' */

  /* DataTypeConversion: '<S157>/Data Type Conversion6' */
  rtb_DataTypeConversion6 = (real_T)rtb_Switch_o[11];

  /* MATLAB Function: '<S194>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion6, 0.01, 0.02,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_g,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_g);

  /* DataTypeConversion: '<S157>/Data Type Conversion8' incorporates:
   *  Constant: '<S189>/Bias'
   *  Constant: '<S189>/Gains'
   *  Product: '<S189>/Divide'
   *  Sum: '<S189>/Sum'
   */
  rtb_RhhcosphicoslambXe = (real32_T)floor((real32_T)(3.1760616302490234 *
    AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_g.y) + 911.698242F);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8 = rtb_RhhcosphicoslambXe < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)
    rtb_RhhcosphicoslambXe;

  /* End of DataTypeConversion: '<S157>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S152>/Update the Load and Power Data [updateSensorMcuState.c]1' */
  updateLoadData(
                 AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion12
                 , AUAV3_AND_SLUGS_SENSOR_B.DataTypeConversion8
                 );

  /* DataTypeConversion: '<S153>/Data Type Conversion5' incorporates:
   *  Constant: '<S153>/Gyro Gains2'
   *  Gain: '<S165>/[ -1 -1 -1]'
   *  Product: '<S153>/Divide2'
   */
  rtb_DataTypeConversion5_l[0] = -((real32_T)rtb_Switch_o[7] * 0.5F);
  rtb_DataTypeConversion5_l[1] = -((real32_T)rtb_Switch_o[6] * 0.5F);
  rtb_DataTypeConversion5_l[2] = -((real32_T)rtb_Switch_o[8] * 0.5F);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion5_l[0], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_hj,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_hj);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function1' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion5_l[1], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_my,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_my);

  /* MATLAB Function: '<S162>/Embedded MATLAB Function2' */
  A_EmbeddedMATLABFunction_k(rtb_DataTypeConversion5_l[2], 0.01, 40.0,
    &AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_c,
    &AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_c);

  /* MATLAB Function: '<S162>/myMux Fun' */
  AUAV3_AND_SLUGS_S_myMuxFun
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction_hj.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction1_my.y,
     AUAV3_AND_SLUGS_SENSOR_B.sf_EmbeddedMATLABFunction2_c.y,
     &AUAV3_AND_SLUGS_SENSOR_B.sf_myMuxFun_d);

  /* MATLAB Function: '<Root>/myMux Fun3' */
  /* MATLAB Function 'myMux Fun2': '<S11>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S11>:1:5' y = [u1(1:3); u2(1:3)]; */

  /* Update for UnitDelay: '<S2>/Output' incorporates:
   *  Switch: '<S17>/FixPt Switch'
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.Output_DSTATE = rtb_FixPtSum1;

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

  /* Update for Enabled SubSystem: '<S123>/Zero Out Height' */
  AUAV3_ZeroOutHeight_Update
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputatio.tOut,
     AUAV3_AND_SLUGS_SENSOR_B.Sum1, &AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight);

  /* End of Update for SubSystem: '<S123>/Zero Out Height' */

  /* Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator1' */
  AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 0.01F *
    rtb_q_dot[0] + AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 0.01F *
    rtb_q_dot[1] + AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 0.01F *
    rtb_q_dot[2] + AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2];
  AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 0.01F *
    rtb_q_dot[3] + AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3];
  if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] >= 1.0F) {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] <= -1.0F)
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] = -1.0F;
    }
  }

  if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] >= 1.0F) {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 1.0F;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] <= -1.0F)
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] = -1.0F;
    }
  }

  if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] >= 1.0F) {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 1.0F;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] <= -1.0F)
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] = -1.0F;
    }
  }

  if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] >= 1.0F) {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 1.0F;
  } else {
    if (AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] <= -1.0F)
    {
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] = -1.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator1' */
  /* Update for Delay: '<S22>/Integer Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[0] = rtb_UEN2NEU[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[1] = rtb_UEN2NEU[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[2] = rtb_UEN2NEU[2];

  /* Update for UnitDelay: '<S88>/UD'
   *
   * Block description for '<S88>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE = AUAV3_AND_SLUGS_SENSOR_B.In1[0];

  /* Update for UnitDelay: '<S89>/UD'
   *
   * Block description for '<S89>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE_i = AUAV3_AND_SLUGS_SENSOR_B.In1[1];

  /* Update for UnitDelay: '<S90>/UD'
   *
   * Block description for '<S90>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_AND_SLUGS_SENSOR_DWork.UD_DSTATE_h = AUAV3_AND_SLUGS_SENSOR_B.In1[2];

  /* Update for Enabled SubSystem: '<S186>/Zero Out Height' */
  AUAV3_ZeroOutHeight_Update
    (AUAV3_AND_SLUGS_SENSOR_B.sf_EnablesDisablestheComputat_e.tOut,
     AUAV3_AND_SLUGS_SENSOR_B.Sum1_b,
     &AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight_d);

  /* End of Update for SubSystem: '<S186>/Zero Out Height' */

  /* Update for UnitDelay: '<S101>/Unit Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE = rtb_Deg2R1;

  /* Update for Delay: '<S33>/Integer Delay1' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 4; rtb_Gain21_h++) {
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[(uint16_T)rtb_Gain21_h *
      3U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[((uint16_T)
      rtb_Gain21_h + 1U) * 3U];
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[(uint16_T)rtb_Gain21_h *
      3U + 1U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[((uint16_T)
      rtb_Gain21_h + 1U) * 3U + 1U];
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[(uint16_T)rtb_Gain21_h *
      3U + 2U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[((uint16_T)
      rtb_Gain21_h + 1U) * 3U + 2U];
  }

  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[12] = Product[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[13] = Product[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[14] = Product[2];

  /* End of Update for Delay: '<S33>/Integer Delay1' */

  /* Update for Delay: '<S103>/Integer Delay1' */
  for (rtb_Gain21_h = 0; rtb_Gain21_h < 4; rtb_Gain21_h++) {
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[(uint16_T)rtb_Gain21_h *
      3U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[((uint16_T)
      rtb_Gain21_h + 1U) * 3U];
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[(uint16_T)rtb_Gain21_h *
      3U + 1U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[((uint16_T)
      rtb_Gain21_h + 1U) * 3U + 1U];
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[(uint16_T)rtb_Gain21_h *
      3U + 2U] = AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[((uint16_T)
      rtb_Gain21_h + 1U) * 3U + 2U];
  }

  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[12] = Product1_o[0];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[13] = Product1_o[1];
  AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[14] = Product1_o[2];

  /* End of Update for Delay: '<S103>/Integer Delay1' */
  /* Update for DiscreteZeroPole: '<S110>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE =
      rtb_DataTypeConversion2[0] + (-0.99960423832914991)*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE;
  }

  /* Update for DiscreteZeroPole: '<S111>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o =
      rtb_DataTypeConversion2[1] + (-0.99960423832914991)*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o;
  }

  /* Update for DiscreteZeroPole: '<S112>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_k =
      rtb_DataTypeConversion2[2] + (-0.99960423832914991)*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* Update for DiscreteZeroPole: '<S42>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_p =
      rtb_DataTypeConversion2_b[0] + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_p;
  }

  /* Update for DiscreteZeroPole: '<S43>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o2 =
      rtb_DataTypeConversion2_b[1] + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_o2;
  }

  /* Update for DiscreteZeroPole: '<S44>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_d =
      rtb_DataTypeConversion2_b[2] + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_d;
  }

  /* Update for UnitDelay: '<S99>/Unit Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE_c = rtb_DataTypeConversion1_d;

  /* Update for DiscreteZeroPole: '<S115>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_f =
      rtb_DataTypeConversion2_k + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_f;
  }

  /* Update for UnitDelay: '<S100>/Unit Delay' */
  AUAV3_AND_SLUGS_SENSOR_DWork.UnitDelay_DSTATE_a = rtb_DataTypeConversion1_dq;

  /* Update for DiscreteZeroPole: '<S117>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_fl =
      rtb_DataTypeConversion2_p + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_fl;
  }

  /* Update for DiscreteZeroPole: '<S119>/Discrete Zero-Pole' */
  {
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_db =
      rtb_DataTypeConversion2_bt + 1.0*
      AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteZeroPole_DSTATE_db;
  }

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  AUAV3_AND_SLUGS_SENSOR_M->Timing.clockTick0++;
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

  /* MATLAB Function 'Control Surface Input/myMux Fun5': '<S15>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S15>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1)]; */
  /* S-Function "dsPIC_PWM_IC" Block: <S10>/Input Capture RC Receiver1 */
  AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw = MCHP_ic1up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o2 = MCHP_ic2up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o3 = MCHP_ic3up;
  AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw = MCHP_ic4up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o5 = MCHP_ic5up;
  AUAV3_AND_SLUGS_SENSOR_B.InputCaptureRCReceiver1_o6 = MCHP_ic6up;

  /* Bias: '<S10>/Bias' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion4'
   */
  rtb_Bias_i = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdPITCHraw - 13125;

  /* Bias: '<S10>/Bias1' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion5'
   */
  rtb_Bias1 = (int16_T)AUAV3_AND_SLUGS_SENSOR_B.cmdROLLraw - 13125;

  /* Sum: '<S215>/Sum' */
  tmp = (int32_T)rtb_Bias1 - (int32_T)rtb_Bias_i;
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
  u = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S10>/Saturation' */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation = u >= 19249U ? 19249U : u <= 7000U ?
    7000U : u;

  /* Sum: '<S215>/Sum1' */
  tmp = (int32_T)rtb_Bias_i + (int32_T)rtb_Bias1;
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
  u = (uint16_T)((int16_T)tmp + 13125);

  /* Saturate: '<S10>/Saturation1' */
  AUAV3_AND_SLUGS_SENSOR_B.Saturation1 = u >= 19249U ? 19249U : u <= 7000U ?
    7000U : u;

  /* DataStoreRead: '<S4>/Get Raw IMU' */
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

/* Model step function for TID2 */
void AUAV3_AND_SLUGS_SENSOR_step2(void) /* Sample time: [0.05s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator_a;

  /* S-Function (MCHP_Digital_Output_Read): '<S18>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S18>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S18>/Digital Output Read'
   */
  rtb_LogicalOperator_a = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S18>/Digital Output Write' */
  LATBbits.LATB2 = rtb_LogicalOperator_a;
}

/* Model step function for TID3 */
void AUAV3_AND_SLUGS_SENSOR_step3(void) /* Sample time: [0.1s, 0.0s] */
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

/* Model step function for TID6 */
void AUAV3_AND_SLUGS_SENSOR_step6(void) /* Sample time: [0.2s, 0.04s] */
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

  /* S-Function (MCHP_Digital_Output_Read): '<S20>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S20>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S20>/Digital Output Read'
   */
  rtb_LogicalOperator2 = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S20>/Digital Output Write' */
  LATBbits.LATB4 = rtb_LogicalOperator2;
}

/* Model step function for TID10 */
void AUAV3_AND_SLUGS_SENSOR_step10(void) /* Sample time: [0.5s, 0.0s] */
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

  /* initialize real-time model */
  (void) memset((void *)AUAV3_AND_SLUGS_SENSOR_M, 0,
                sizeof(RT_MODEL_AUAV3_AND_SLUGS_SENS_T));
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[0] = 1;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[1] = 2;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[2] = 5;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[3] = 10;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[4] = 20;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[5] = 20;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[6] = 20;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[7] = 20;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[8] = 20;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[9] = 25;
  (AUAV3_AND_SLUGS_SENSOR_M)->Timing.TaskCounters.cLimit[10] = 50;

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

  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S7>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */

  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0xA5;                      /* I2C clock = 399772  (400000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;

  /* Start for Enabled SubSystem: '<S123>/Zero Out Height' */
  AUAV3__ZeroOutHeight_Start(&AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight);

  /* End of Start for SubSystem: '<S123>/Zero Out Height' */

  /* Start for Enabled SubSystem: '<S123>/Enabled Subsystem' */
  AUA_EnabledSubsystem_Start(&AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem);

  /* End of Start for SubSystem: '<S123>/Enabled Subsystem' */

  /* Start for S-Function (MCHP_BUS_SPI): '<S8>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */

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

  /* Start for Enabled SubSystem: '<S96>/Subsystem' */

  /* InitializeConditions for MATLAB Function: '<S104>/Embedded MATLAB Function1' */
  EmbeddedMATLABFunct_a_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_o);

  /* InitializeConditions for MATLAB Function: '<S104>/Embedded MATLAB Function2' */
  EmbeddedMATLABFunct_a_Init
    (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_n);

  /* End of Start for SubSystem: '<S96>/Subsystem' */

  /* Start for Enabled SubSystem: '<S186>/Zero Out Height' */
  AUAV3__ZeroOutHeight_Start(&AUAV3_AND_SLUGS_SENSOR_DWork.ZeroOutHeight_d);

  /* End of Start for SubSystem: '<S186>/Zero Out Height' */

  /* Start for Enabled SubSystem: '<S186>/Enabled Subsystem' */
  AUA_EnabledSubsystem_Start(&AUAV3_AND_SLUGS_SENSOR_B.EnabledSubsystem_o);

  /* End of Start for SubSystem: '<S186>/Enabled Subsystem' */

  /* Start for S-Function (MCHP_MCU_LOAD): '<S154>/Calculus Time Step1' */
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */

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

  /* Start for S-Function (MCHP_OC_HW): '<S10>/Output Compare - HW Drive Servo motor' */

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

  {
    int16_T i;

    /* user code (Initialize function Body) */
    uartBufferInit();
    uartMavlinkBufferInit ();
    InitParameterInterface();

    /* InitializeConditions for Delay: '<S125>/Delay11' */
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE = 0UL;

    /* InitializeConditions for Delay: '<S126>/Delay11' */
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay11_DSTATE_j = 0UL;

    /* InitializeConditions for Delay: '<S7>/Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.Delay_DSTATE = 1U;

    /* InitializeConditions for MATLAB Function: '<S123>/Enables//Disables the Computation of  initial Baro Bias' */
    EnablesDisablestheCom_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputatio);

    /* InitializeConditions for DiscreteIntegrator: '<S22>/Discrete-Time Integrator1' */
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 0.0F;

    /* InitializeConditions for Delay: '<S22>/Integer Delay' */
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[0] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[1] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay_DSTATE[2] = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S97>/Embedded MATLAB Function3' */
    AUAV3_AND_SLUGS_SENSOR_DWork.lastGps_h_not_empty = FALSE;

    /* InitializeConditions for MATLAB Function: '<S186>/Enables//Disables the Computation of  initial Baro Bias' */
    EnablesDisablestheCom_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EnablesDisablestheComputat_e);

    /* InitializeConditions for MATLAB Function: '<S193>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_a);

    /* InitializeConditions for MATLAB Function: '<S196>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_o);

    /* InitializeConditions for MATLAB Function: '<S108>/Embedded MATLAB Function' */
    AUAV3_AND_SLUGS_SENSOR_DWork.a_not_empty = FALSE;

    /* InitializeConditions for Delay: '<S33>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE[i] = 19.5F;
    }

    /* End of InitializeConditions for Delay: '<S33>/Integer Delay1' */

    /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_h);

    /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1);

    /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2);

    /* InitializeConditions for Delay: '<S103>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      AUAV3_AND_SLUGS_SENSOR_DWork.IntegerDelay1_DSTATE_k[i] = 19.5F;
    }

    /* End of InitializeConditions for Delay: '<S103>/Integer Delay1' */

    /* InitializeConditions for RateLimiter: '<S96>/Rate Limiter' */
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[0] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[1] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY[2] = 0.0F;

    /* InitializeConditions for RateLimiter: '<S22>/Bias Rate Limiter' */
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[0] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[1] = 0.0F;
    AUAV3_AND_SLUGS_SENSOR_DWork.PrevY_c[2] = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_k);

    /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_m);

    /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_p);

    /* InitializeConditions for MATLAB Function: '<S195>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_l);

    /* InitializeConditions for MATLAB Function: '<S194>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_g);

    /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction_hj);

    /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction1_my);

    /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_m_Init
      (&AUAV3_AND_SLUGS_SENSOR_DWork.sf_EmbeddedMATLABFunction2_c);
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
