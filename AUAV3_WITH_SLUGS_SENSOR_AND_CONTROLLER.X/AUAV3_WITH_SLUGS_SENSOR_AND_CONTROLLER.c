/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.c
 *
 * Code generated for Simulink model 'AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER'.
 *
 * Model version                  : 1.289
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Jun 23 23:55:56 2016
 */

#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h"

const mavlink_scaled_pressure_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_scaled_pressure_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_attitude_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_attitude_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* roll */
  0.0F,                                /* pitch */
  0.0F,                                /* yaw */
  0.0F,                                /* rollspeed */
  0.0F,                                /* pitchspeed */
  0.0F                                 /* yawspeed */
} ;                                    /* mavlink_attitude_t ground */

const mavlink_coordinate_float_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_coordinate_float_t = {
  0U,                                  /* lat */
  0U,                                  /* lon */
  0U                                   /* alt */
} ;                                    /* mavlink_coordinate_float_t ground */

const mavlink_gps_raw_int_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_gps_raw_int_t = {
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

const mavlink_heartbeat_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_heartbeat_t = {
  0U,                                  /* custom_mode */
  0U,                                  /* type */
  0U,                                  /* autopilot */
  0U,                                  /* base_mode */
  0U,                                  /* system_status */
  0U                                   /* mavlink_version */
} ;                                    /* mavlink_heartbeat_t ground */

const mavlink_isr_location_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_isr_location_t = {
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0.0F,                                /* height */
  0U,                                  /* option1 */
  0U,                                  /* option2 */
  0U                                   /* option3 */
} ;                                    /* mavlink_isr_location_t ground */

const mavlink_mid_lvl_cmds_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_mid_lvl_cmds_t = {
  0.0F,                                /* hCommand */
  0.0F,                                /* uCommand */
  0.0F,                                /* rCommand */
  0U                                   /* target */
} ;                                    /* mavlink_mid_lvl_cmds_t ground */

const mavlink_slugs_mobile_location_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_slugs_mobile_location_t = {
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0U                                   /* target */
} ;                                    /* mavlink_slugs_mobile_location_t ground */

const mavlink_slugs_navigation_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_slugs_navigation_t = {
  0.0F,                                /* u_m */
  0.0F,                                /* phi_c */
  0.0F,                                /* theta_c */
  0.0F,                                /* psiDot_c */
  0.0F,                                /* ay_body */
  0.0F,                                /* totalDist */
  0.0F,                                /* dist2Go */
  0U,                                  /* h_c */
  0U,                                  /* fromWP */
  0U                                   /* toWP */
} ;                                    /* mavlink_slugs_navigation_t ground */

const pi_struct AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZpi_struct = {
  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    /* param */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U }
  /* param_name */
} ;                                    /* pi_struct ground */

const mavlink_raw_pressure_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_raw_pressure_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_servo_output_raw_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_servo_output_raw_t = {
  0U,                                  /* time_usec */
  0U,                                  /* servo1_raw */
  0U,                                  /* servo2_raw */
  0U,                                  /* servo3_raw */
  0U,                                  /* servo4_raw */
  0U,                                  /* servo5_raw */
  0U,                                  /* servo6_raw */
  0U,                                  /* servo7_raw */
  0U,                                  /* servo8_raw */
  0U                                   /* port */
} ;                                    /* mavlink_servo_output_raw_t ground */

const mavlink_sys_status_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_sys_status_t = {
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

const mavlink_volt_sensor_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_volt_sensor_t = {
  0U,                                  /* voltage */
  0U,                                  /* reading2 */
  0U                                   /* r2Type */
} ;                                    /* mavlink_volt_sensor_t ground */

const mavlink_mission_item_values_t
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_mission_item_values_t = {
  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* lat */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* lon */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* alt */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* type */

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    /* orbit */
  0U                                   /* wpCount */
} ;                                    /* mavlink_mission_item_values_t ground */

/* Exported block states */
pi_struct mlParamInterface;            /* '<Root>/mlParamINterface' */
mavlink_mission_item_values_t mlWpValues;/* '<Root>/mlWpValues' */
mavlink_attitude_t mlAttitude;         /* '<Root>/mlAttitude' */
mavlink_gps_raw_int_t mlGpsData;       /* '<Root>/mlGpsData' */
mavlink_slugs_navigation_t mlNavigation;/* '<Root>/mlNavigation' */
mavlink_sys_status_t mlSysStatus;      /* '<Root>/mlSysStatus' */
mavlink_servo_output_raw_t mlServoOutputRaw;/* '<Root>/mlServoOutputRaw' */
mavlink_scaled_pressure_t mlAirData;   /* '<Root>/mlAirData' */
mavlink_coordinate_float_t mlGSLocationFloat;/* '<Root>/mlGSLocationFloat' */
mavlink_heartbeat_t mlHeartbeatLocal;  /* '<Root>/mlHeartbeatLocal' */
mavlink_isr_location_t mlISR;          /* '<Root>/mlISR' */
mavlink_mid_lvl_cmds_t mlMidLevelCommands;/* '<Root>/mlMidLevelCommands' */
mavlink_slugs_mobile_location_t mlMobileLocation;/* '<Root>/mlMobileLocation' */
mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
mavlink_volt_sensor_t mlVISensor;      /* '<Root>/mlVISensor' */
uint16_T MPU_T;                        /* '<Root>/MPU_T' */

/* Block signals (auto storage) */
BlockIO_AUAV3_WITH_SLUGS_SENS_T AUAV3_WITH_SLUGS_SENSOR_AND_C_B;

/* Block states (auto storage) */
D_Work_AUAV3_WITH_SLUGS_SENSO_T AUAV3_WITH_SLUGS_SENSOR_A_DWork;

/* Real-time model */
RT_MODEL_AUAV3_WITH_SLUGS_SEN_T AUAV3_WITH_SLUGS_SENSOR_AND__M_;
RT_MODEL_AUAV3_WITH_SLUGS_SEN_T *const AUAV3_WITH_SLUGS_SENSOR_AND__M =
  &AUAV3_WITH_SLUGS_SENSOR_AND__M_;
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
 * Output and update for atomic system:
 *    '<S1>/myMux Fun5'
 *    '<S630>/myMux Fun5'
 */
void AUAV3_WITH_SLUGS_myMuxFun5(uint16_T rtu_u1, uint16_T rtu_u2, uint16_T
  rtu_u3, uint16_T rtu_u4, uint16_T rtu_u5, rtB_myMuxFun5_AUAV3_WITH_SLUG_T
  *localB)
{
  /* MATLAB Function 'Control Surface Input/myMux Fun5': '<S23>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S23>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
  localB->y[3] = rtu_u4;
  localB->y[4] = rtu_u5;
}

/*
 * Initial conditions for atomic system:
 *    '<S66>/Embedded MATLAB Function'
 *    '<S103>/Embedded MATLAB Function'
 *    '<S127>/Embedded MATLAB Function'
 *    '<S522>/Embedded MATLAB Function'
 *    '<S639>/Embedded MATLAB Function'
 *    '<S640>/Embedded MATLAB Function'
 */
void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_A_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S66>/Embedded MATLAB Function'
 *    '<S103>/Embedded MATLAB Function'
 *    '<S127>/Embedded MATLAB Function'
 *    '<S522>/Embedded MATLAB Function'
 *    '<S639>/Embedded MATLAB Function'
 *    '<S640>/Embedded MATLAB Function'
 */
void AUA_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_AU_T *localB, rtDW_EmbeddedMATLABFunction_A_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Inner Loop/ Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function': '<S74>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S74>:1:9' if isempty(a) */
  if (!localDW->a_not_empty) {
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S74>:1:12' omega = 2*pi*f; */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S74>:1:13' a=T*omega/(2+T*omega); */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S74>:1:14' b=-(T*omega-2)/(T*omega+2); */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S74>:1:15' y_km1=u; */
    localDW->y_km1 = rtu_u;

    /* '<S74>:1:16' u_km1=u; */
    localDW->u_km1 = rtu_u;
  }

  /* '<S74>:1:19' y = a*(u+u_km1)+b*y_km1; */
  localB->y = (rtu_u + localDW->u_km1) * (real32_T)localDW->a + (real32_T)
    localDW->b * localDW->y_km1;

  /* '<S74>:1:20' y_km1=y; */
  localDW->y_km1 = localB->y;

  /* '<S74>:1:21' u_km1=u; */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S79>/negprotect'
 *    '<S470>/negprotect'
 *    '<S475>/negprotect'
 */
void AUAV3_WITH_SLUG_negprotect(real32_T rtu_val,
  rtB_negprotect_AUAV3_WITH_SLU_T *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect': '<S84>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S84>:1:5' if val >= single(0.001) */
  if (rtu_val >= 0.001F) {
    /* '<S84>:1:6' zpVal = val; */
    localB->zpVal = rtu_val;
  } else {
    /* '<S84>:1:7' else */
    /* '<S84>:1:8' zpVal = single(0.001); */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S136>/negprotect'
 *    '<S415>/negprotect'
 *    '<S273>/negprotect'
 *    '<S297>/negprotect'
 *    '<S304>/negprotect'
 *    '<S352>/negprotect'
 *    '<S359>/negprotect'
 *    '<S399>/negprotect'
 *    '<S258>/negprotect'
 *    '<S187>/negprotect'
 *    ...
 */
void AUAV3_WITH_SL_negprotect_d(real32_T rtu_val,
  rtB_negprotect_AUAV3_WITH_S_h_T *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/negprotect': '<S138>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S138>:1:5' if val >= single(0.001) */
  if (rtu_val >= 0.001F) {
    /* '<S138>:1:6' zpVal = val; */
    localB->zpVal = rtu_val;
  } else {
    /* '<S138>:1:7' else */
    /* '<S138>:1:8' zpVal = single(0.001); */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S414>/Embedded MATLAB Function'
 *    '<S296>/Embedded MATLAB Function'
 *    '<S303>/Embedded MATLAB Function'
 *    '<S351>/Embedded MATLAB Function'
 *    '<S358>/Embedded MATLAB Function'
 *    '<S398>/Embedded MATLAB Function'
 *    '<S257>/Embedded MATLAB Function'
 *    '<S186>/Embedded MATLAB Function'
 *    '<S199>/Embedded MATLAB Function'
 *    '<S212>/Embedded MATLAB Function'
 *    ...
 */
void A_EmbeddedMATLABFunction_g(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_p_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S416>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S416>:1:5' xDoty = x(1)*y(1) + x(2)*y(2) + x(3)*y(3); */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Output and update for atomic system:
 *    '<S150>/Zero out Z1'
 *    '<S265>/Zero out Z1'
 *    '<S265>/Zero out Z2'
 *    '<S265>/Zero out Z3'
 *    '<S149>/Zero out Z2'
 *    '<S160>/Zero out Z2'
 *    '<S239>/Zero out Z1'
 *    '<S142>/Zero out Z1'
 *    '<S142>/Zero out Z2'
 */
void AUAV3_WITH_SLUGS_ZerooutZ1(const real32_T rtu_Pin[3],
  rtB_ZerooutZ1_AUAV3_WITH_SLUG_T *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB/Follow Mobile Navigation/Zero out Z1': '<S407>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S407>:1:5' P = [Pin(1); Pin(2); 0]; */
  localB->P[0] = rtu_Pin[0];
  localB->P[1] = rtu_Pin[1];
  localB->P[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S278>/Embedded MATLAB Function'
 *    '<S271>/Embedded MATLAB Function'
 *    '<S183>/Embedded MATLAB Function'
 *    '<S196>/Embedded MATLAB Function'
 */
void A_EmbeddedMATLABFunction_a(const real32_T rtu_x[3], const real32_T rtu_y[3],
  rtB_EmbeddedMATLABFunction_pk_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S279>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S279>:1:5' xDoty = x(1)*y(1) + x(2)*y(2) + x(3)*y(3); */
  localB->xDoty = (rtu_x[0] * rtu_y[0] + rtu_x[1] * rtu_y[1]) + rtu_x[2] *
    rtu_y[2];
}

/*
 * Output and update for atomic system:
 *    '<S286>/Select N  Terms'
 *    '<S344>/Select N  Terms'
 */
void AUAV3_WITH_SL_SelectNTerms(const real32_T rtu_T[3],
  rtB_SelectNTerms_AUAV3_WITH_S_T *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms': '<S294>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S294>:1:5' N = [-T(2); T(1); 0]; */
  localB->N[0] = -rtu_T[1];
  localB->N[1] = rtu_T[0];
  localB->N[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S174>/negprotect3'
 *    '<S175>/negprotect3'
 *    '<S162>/negprotect1'
 *    '<S162>/negprotect2'
 *    '<S231>/negprotect'
 *    '<S232>/negprotect'
 */
void AUAV3_WITH_SLU_negprotect3(real32_T rtu_val,
  rtB_negprotect3_AUAV3_WITH_SL_T *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3': '<S185>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S185>:1:5' if val >= single(0.0001) */
  if (rtu_val >= 0.0001F) {
    /* '<S185>:1:6' zpVal = val; */
    localB->zpVal = rtu_val;
  } else {
    /* '<S185>:1:7' else */
    /* '<S185>:1:8' zpVal = single(0.0001); */
    localB->zpVal = 0.0001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S443>/myMux Fun1'
 *    '<S512>/myMux Fun1'
 */
void AUAV3_WITH_SLUGS_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun1_AUAV3_WITH_SLUG_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Attitude Complimentary Filter COG/3D Filter/myMux Fun1': '<S459>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S459>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int16_T u0_0;
  int16_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2(u0_0, u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/*
 * Output and update for action system:
 *    '<S448>/AxisRotZeroR3'
 *    '<S672>/AxisRotZeroR3'
 */
void AUAV3_WITH_S_AxisRotZeroR3(const real32_T rtu_In1[7], real32_T *rty_1,
  real32_T *rty_2, real32_T *rty_3)
{
  real32_T rtu_In1_0;

  /* Fcn: '<S467>/Fcn1' */
  *rty_1 = rt_atan2f_snf(rtu_In1[5], rtu_In1[6]);

  /* Fcn: '<S467>/Fcn2' */
  if (rtu_In1[2] > 1.0F) {
    rtu_In1_0 = 1.0F;
  } else if (rtu_In1[2] < -1.0F) {
    rtu_In1_0 = -1.0F;
  } else {
    rtu_In1_0 = rtu_In1[2];
  }

  *rty_2 = (real32_T)asin(rtu_In1_0);

  /* End of Fcn: '<S467>/Fcn2' */

  /* Fcn: '<S467>/Fcn3' */
  *rty_3 = 0.0F;
}

/*
 * Output and update for action system:
 *    '<S448>/AxisRotDefault'
 *    '<S672>/AxisRotDefault'
 */
void AUAV3_WITH__AxisRotDefault(const real32_T rtu_In1[7], real32_T *rty_1,
  real32_T *rty_2, real32_T *rty_3)
{
  real32_T rtu_In1_0;

  /* Fcn: '<S466>/Fcn1' */
  *rty_1 = rt_atan2f_snf(rtu_In1[0], rtu_In1[1]);

  /* Fcn: '<S466>/Fcn2' */
  if (rtu_In1[2] > 1.0F) {
    rtu_In1_0 = 1.0F;
  } else if (rtu_In1[2] < -1.0F) {
    rtu_In1_0 = -1.0F;
  } else {
    rtu_In1_0 = rtu_In1[2];
  }

  *rty_2 = (real32_T)asin(rtu_In1_0);

  /* End of Fcn: '<S466>/Fcn2' */

  /* Fcn: '<S466>/Fcn3' */
  *rty_3 = rt_atan2f_snf(rtu_In1[3], rtu_In1[4]);
}

/*
 * Output and update for atomic system:
 *    '<S436>/Embedded MATLAB Function'
 *    '<S18>/Embedded MATLAB Function1'
 */
void A_EmbeddedMATLABFunction_d(real32_T rtu_u, rtB_EmbeddedMATLABFunction_pd_T *
  localB)
{
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S449>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S449>:1:5' if ( u < 0 ) */
  if (rtu_u < 0.0F) {
    /* '<S449>:1:6' y = u + 2*pi; */
    localB->y = rtu_u + 6.28318548F;
  } else {
    /* '<S449>:1:7' else */
    /* '<S449>:1:8' y = u; */
    localB->y = rtu_u;
  }
}

/*
 * Output and update for atomic system:
 *    '<S471>/Embedded MATLAB Function'
 *    '<S476>/Embedded MATLAB Function'
 */
void A_EmbeddedMATLABFunction_h(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_d_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function': '<S473>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S473>:1:5' xDoty = x(1)*y(1) + x(2)*y(2) + x(3)*y(3); */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Initial conditions for atomic system:
 *    '<S518>/Embedded MATLAB Function1'
 *    '<S518>/Embedded MATLAB Function2'
 */
void EmbeddedMATLABFunct_o_Init(rtDW_EmbeddedMATLABFunction1__T *localDW)
{
  localDW->LastU_not_empty = false;
  localDW->LastU = 0.0F;
  localDW->rate = 0.0F;
  localDW->OldRate = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S518>/Embedded MATLAB Function1'
 *    '<S518>/Embedded MATLAB Function2'
 */
void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW)
{
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Position Filter [updated 5.13.16]/BlendPosVel/Subsystem/Embedded MATLAB Function1': '<S534>:1' */
  /* '<S534>:1:7' if (isempty(LastU)) */
  if (!localDW->LastU_not_empty) {
    /* '<S534>:1:8' LastU = single(0.); */
    localDW->LastU_not_empty = true;

    /* '<S534>:1:9' TimeSinceLast = single(apSampleTime); */
    localDW->TimeSinceLast = 0.01F;

    /* '<S534>:1:10' rate = single(0); */
    /* '<S534>:1:11' OldRate = single(0); */
  }

  /* '<S534>:1:14' if (NewGPS) */
  if (rtu_NewGPS != 0) {
    /* '<S534>:1:15' OldRate = rate; */
    localDW->OldRate = localDW->rate;

    /* '<S534>:1:16' rate = single((u - LastU)/TimeSinceLast); */
    localDW->rate = (rtu_u - localDW->LastU) / localDW->TimeSinceLast;

    /* '<S534>:1:17' TimeSinceLast = single(0); */
    localDW->TimeSinceLast = 0.0F;

    /* '<S534>:1:18' LastU = u; */
    localDW->LastU = rtu_u;
  }

  /* '<S534>:1:20' y = u + ( .5*rate + .5*OldRate )*TimeSinceLast; */
  localB->y = (0.5F * localDW->rate + 0.5F * localDW->OldRate) *
    localDW->TimeSinceLast + rtu_u;

  /* '<S534>:1:21' TimeSinceLast = TimeSinceLast + apSampleTime; */
  localDW->TimeSinceLast += 0.01F;
}

/*
 * Output and update for atomic system:
 *    '<S510>/myMux Fun1'
 *    '<S510>/myMux Fun2'
 *    '<S566>/myMux Fun'
 */
void AUAV3_WITH_SLU_myMuxFun1_f(real32_T rtu_u1, real32_T rtu_u2, real32_T
  rtu_u3, rtB_myMuxFun1_AUAV3_WITH_SL_k_T *localB)
{
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Position Filter [updated 5.13.16]/BlendPosVel/myMux Fun1': '<S519>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S519>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Start for enable system:
 *    '<S537>/Enabled Subsystem'
 *    '<S599>/Enabled Subsystem'
 */
void AUA_EnabledSubsystem_Start(rtB_EnabledSubsystem_AUAV3_WI_T *localB)
{
  /* VirtualOutportStart for Outport: '<S545>/Out1' */
  localB->In1 = 143.543F;
}

/*
 * Output and update for enable system:
 *    '<S537>/Enabled Subsystem'
 *    '<S599>/Enabled Subsystem'
 */
void AUAV3_WIT_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_AUAV3_WI_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S537>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S545>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S545>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S537>/Enabled Subsystem' */
}

/*
 * Initial conditions for atomic system:
 *    '<S537>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S599>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW)
{
  /* '<S546>:1:7' aveCount =1; */
  localDW->aveCount = 1.0;

  /* '<S546>:1:8' tIni = 1; */
  localDW->tIni = 1.0;
}

/*
 * Output and update for atomic system:
 *    '<S537>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S599>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW)
{
  /* MATLAB Function 'Read Barometer BMP180/Baro Altimeter/Enables/Disables the Computation of  initial Baro Bias': '<S546>:1' */
  /* '<S546>:1:6' if isempty(aveCount) */
  /* '<S546>:1:11' if aveCount < 2500 */
  if (localDW->aveCount < 2500.0) {
    /* '<S546>:1:12' aveCount = aveCount + 1; */
    localDW->aveCount++;
  }

  /* '<S546>:1:15' if aveCount == 2500 */
  if (localDW->aveCount == 2500.0) {
    /* '<S546>:1:16' tIni =0; */
    localDW->tIni = 0.0;

    /* '<S546>:1:17' aveCount = aveCount + 1; */
    localDW->aveCount++;
  }

  /* '<S546>:1:20' tOut = tIni; */
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
 * Outputs for enable system:
 *    '<S537>/Zero Out Height'
 *    '<S599>/Zero Out Height'
 */
void AUAV3_WITH_S_ZeroOutHeight(real_T rtu_Enable, real32_T rtu_BaseHeight,
  rtB_ZeroOutHeight_AUAV3_WITH__T *localB, rtDW_ZeroOutHeight_AUAV3_WITH_T
  *localDW)
{
  /* Outputs for Enabled SubSystem: '<S537>/Zero Out Height' incorporates:
   *  EnablePort: '<S548>/Enable'
   */
  if (rtu_Enable > 0.0) {
    /* Sum: '<S548>/Sum' incorporates:
     *  Delay: '<S548>/Integer Delay'
     */
    localB->Sum = rtu_BaseHeight - localDW->IntegerDelay_DSTATE;
  }

  /* End of Outputs for SubSystem: '<S537>/Zero Out Height' */
}

/*
 * Update for enable system:
 *    '<S537>/Zero Out Height'
 *    '<S599>/Zero Out Height'
 */
void AUAV3_ZeroOutHeight_Update(real_T rtu_Enable, real32_T rtu_ComputedHeight,
  rtDW_ZeroOutHeight_AUAV3_WITH_T *localDW)
{
  /* Update for Enabled SubSystem: '<S537>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S548>/Enable'
   */
  if (rtu_Enable > 0.0) {
    /* Update for Delay: '<S548>/Integer Delay' */
    localDW->IntegerDelay_DSTATE = rtu_ComputedHeight;
  }

  /* End of Update for SubSystem: '<S537>/Zero Out Height' */
}

/*
 * Initial conditions for atomic system:
 *    '<S573>/Embedded MATLAB Function'
 *    '<S573>/Embedded MATLAB Function1'
 *    '<S573>/Embedded MATLAB Function2'
 *    '<S574>/Embedded MATLAB Function'
 *    '<S574>/Embedded MATLAB Function1'
 *    '<S574>/Embedded MATLAB Function2'
 *    '<S575>/Embedded MATLAB Function'
 *    '<S575>/Embedded MATLAB Function1'
 *    '<S575>/Embedded MATLAB Function2'
 *    '<S606>/Embedded MATLAB Function'
 *    ...
 */
void EmbeddedMATLABFunct_i_Init(rtDW_EmbeddedMATLABFunction_f_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S573>/Embedded MATLAB Function'
 *    '<S573>/Embedded MATLAB Function1'
 *    '<S573>/Embedded MATLAB Function2'
 *    '<S574>/Embedded MATLAB Function'
 *    '<S574>/Embedded MATLAB Function1'
 *    '<S574>/Embedded MATLAB Function2'
 *    '<S575>/Embedded MATLAB Function'
 *    '<S575>/Embedded MATLAB Function1'
 *    '<S575>/Embedded MATLAB Function2'
 *    '<S606>/Embedded MATLAB Function'
 *    ...
 */
void A_EmbeddedMATLABFunction_n(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_m_T *localB, rtDW_EmbeddedMATLABFunction_f_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S579>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S579>:1:9' if isempty(a) */
  if (!localDW->a_not_empty) {
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S579>:1:12' omega = 2*pi*f; */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S579>:1:13' a=T*omega/(2+T*omega); */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S579>:1:14' b=-(T*omega-2)/(T*omega+2); */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S579>:1:15' y_km1=u; */
    localDW->y_km1 = rtu_u;

    /* '<S579>:1:16' u_km1=u; */
    localDW->u_km1 = rtu_u;
  }

  /* '<S579>:1:19' y = a*(u+u_km1)+b*y_km1; */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S579>:1:20' y_km1=y; */
  localDW->y_km1 = localB->y;

  /* '<S579>:1:21' u_km1=u; */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S573>/myMux Fun'
 *    '<S574>/myMux Fun'
 *    '<S575>/myMux Fun'
 */
void AUAV3_WITH_SLUGS__myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_AUAV3_WITH_SLUGS_T *localB)
{
  /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S582>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S582>:1:5' y = [u1(1); u2(1); u3(1)]; */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S568>/Embedded MATLAB Function'
 *    '<S568>/Embedded MATLAB Function1'
 *    '<S568>/Embedded MATLAB Function2'
 *    '<S568>/Embedded MATLAB Function3'
 */
void A_EmbeddedMATLABFunction_o(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_g_T *localB)
{
  /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function': '<S591>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S591>:1:5' y = (u(1)+ u(2) + u(3) + u(4) + u(5))*0.20; */
  localB->y = ((((rtu_u[0] + rtu_u[1]) + rtu_u[2]) + rtu_u[3]) + rtu_u[4]) * 0.2;
}

/*
 * Initial conditions for atomic system:
 *    '<S629>/Buffer IC Channel'
 *    '<S629>/Buffer IC Channel1'
 *    '<S629>/Buffer IC Channel2'
 *    '<S629>/Buffer IC Channel3'
 *    '<S634>/Buffer Failsafe Channel'
 */
void AUAV3_BufferICChannel_Init(rtDW_BufferICChannel_AUAV3_WI_T *localDW)
{
  int16_T i;

  /* '<S635>:1:8' oldValues = uint16([0 0 0 0 0 0 0]); */
  for (i = 0; i < 7; i++) {
    localDW->oldValues[i] = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S629>/Buffer IC Channel'
 *    '<S629>/Buffer IC Channel1'
 *    '<S629>/Buffer IC Channel2'
 *    '<S629>/Buffer IC Channel3'
 *    '<S634>/Buffer Failsafe Channel'
 */
void AUAV3_WITH_BufferICChannel(uint16_T rtu_latest,
  rtB_BufferICChannel_AUAV3_WIT_T *localB, rtDW_BufferICChannel_AUAV3_WI_T
  *localDW)
{
  int16_T i;

  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel': '<S635>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S635>:1:7' if isempty(oldValues) */
  /* '<S635>:1:11' history = uint16([0 0 0 0 0 0 0]); */
  for (i = 0; i < 7; i++) {
    localB->history[i] = 0U;
  }

  /* '<S635>:1:13' for i=7:-1:2 */
  for (i = 0; i < 6; i++) {
    /* '<S635>:1:14' oldValues(i) = oldValues(i-1); */
    localDW->oldValues[6 - i] = localDW->oldValues[5 - i];

    /* '<S635>:1:15' history(i) = oldValues(i-1); */
    localB->history[6 - i] = localDW->oldValues[5 - i];
  }

  /* '<S635>:1:18' oldValues(1) = latest; */
  localDW->oldValues[0] = rtu_latest;

  /* '<S635>:1:19' history(1) = latest; */
  localB->history[0] = rtu_latest;
}

/*
 * Output and update for atomic system:
 *    '<S660>/myMux Fun1'
 *    '<S659>/myMux Fun1'
 */
void AUAV3_WITH_SLU_myMuxFun1_c(uint16_T rtu_u1, uint16_T rtu_u2, uint16_T
  rtu_u3, uint16_T rtu_u4, uint16_T rty_y[4])
{
  /* MATLAB Function 'Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/myMux Fun1': '<S670>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S670>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
  rty_y[0] = rtu_u1;
  rty_y[1] = rtu_u2;
  rty_y[2] = rtu_u3;
  rty_y[3] = rtu_u4;
}

/* Model step function for TID0 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step0(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion2;
  real_T rtb_DataTypeConversion2_p;
  real_T rtb_DataTypeConversion2_d[3];
  real_T rtb_DiscreteZeroPole;
  real_T rtb_DiscreteZeroPole_n;
  real_T rtb_DiscreteZeroPole_m;
  real_T rtb_DataTypeConversion2_dp[3];
  real_T rtb_DiscreteZeroPole_c;
  real_T rtb_DiscreteZeroPole_g;
  real_T rtb_DiscreteZeroPole_a;
  real_T rtb_DataTypeConversion2_f;
  real_T rtb_DataTypeConversion;
  real_T rtb_DataTypeConversion_i;
  real_T rtb_DiscreteZeroPole_e;
  real_T rtb_DiscreteZeroPole_d;
  real_T rtb_DiscreteZeroPole_k;
  boolean_T rtb_EnableHILfromControlMCU1;
  boolean_T rtb_DataTypeConversion1_k;
  uint8_T rtb_DataTypeConversion_gm;
  boolean_T rtb_IC1_l;
  real32_T rtb_RhhcosphicoslambXe_cj;
  real32_T rtb_Deg2R1_bc;
  real32_T rtb_RhhcosphisinlambYe_h;
  real32_T rtb_jxi;
  real32_T rtb_kxj;
  real32_T rtb_ixj;
  real32_T rtb_kxi;
  real32_T rtb_Sum_b;
  boolean_T rtb_LogicalOperator_bu;
  real32_T rtb_Sum1;
  real32_T rtb_u11[3];
  uint8_T rtb_DataTypeConversion2_j;
  int16_T rtb_Switch_a[13];
  real32_T rtb_Sum;
  uint8_T rtb_Compare_n;
  real32_T rtb_Sum_cm;
  real32_T rtb_Gain_px;
  real32_T rtb_VectorConcatenate_g[9];
  real32_T rtb_Product_hz;
  real32_T rtb_Product1_bh;
  real32_T rtb_Product2_ka;
  real32_T rtb_Product3_e;
  real32_T rtb_VectorConcatenate[7];
  int32_T yIdx;
  real32_T rtb_Product1_a3[3];
  real32_T rtb_Add_of[3];
  real32_T rtb_VectorConcatenate_k[9];
  uint32_T rtb_B4;
  uint16_T rtb_u2deg;
  int16_T rtb_DataTypeConversion1_a;
  int16_T rtb_DataTypeConversion2_b;
  int16_T rtb_DataTypeConversion3_f;
  boolean_T rtb_IC1;
  uint8_T rtb_IC2;
  real32_T rtb_BankLimitCommand;
  real32_T rtb_Switch3;
  real32_T rtb_Add1_o;
  real32_T rtb_Projection;
  real32_T rtb_Switch3_bx;
  real32_T rtb_Subtract_jz;
  real32_T rtb_Subtract_j5;
  real32_T rtb_Switch1_h;
  real32_T rtb_q_dot[4];
  real32_T rtb_Merge_p[3];
  real32_T rtb_Switch3_fv;
  uint16_T rtb_Switch1_p;
  uint16_T rtb_Switch2;
  uint16_T rtb_Switch3_bc;
  real32_T rtb_Sum1_ao[3];
  real32_T rtb_Product5[3];
  real32_T rtb_P32[3];
  uint32_T rtb_Sum6;
  real32_T rtb_MathFunction[9];
  real32_T rtb_Merge[3];
  int16_T i;
  real32_T tmp[3];
  real32_T tmp_0[9];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[9];
  real32_T rtb_MathFunction_0[3];
  real32_T rtb_y_c_0[9];
  real32_T rtb_DataTypeConversion7;
  real32_T rtb_DataTypeConversion1_jq_idx_;
  real32_T rtb_DataTypeConversion1_jq_id_0;
  real32_T rtb_Deg2R_idx_1;
  real32_T rtb_Deg2R_idx_0;
  real32_T rtb_DataTypeConversion7_idx_0;
  real32_T rtb_DataTypeConversion7_idx_1;
  uint32_T qY;
  uint32_T qY_0;
  real_T tmp_4;
  real_T tmp_5;
  real_T tmp_6;
  real_T tmp_7;
  uint64m_T tmp_8;
  uint64m_T tmp_9;

  /* DataStoreWrite: '<Root>/Update SysStatus Load' incorporates:
   *  Constant: '<Root>/Constant6'
   */
  mlSysStatus.load = 1U;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S2>/Output'
   */
  rtb_Sum6 = 3276800000UL;
  uMultiWordMul(&rtb_Sum6, 1, &AUAV3_WITH_SLUGS_SENSOR_A_DWork.Output_DSTATE, 1,
                &tmp_9.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_9.chunks[0U], 2, 15U, &tmp_8.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.time_since_boot_usec = MultiWord2uLong
    (&tmp_8.chunks[0U]);

  /* DataTypeConversion: '<S656>/Data Type Conversion1' incorporates:
   *  DataStoreRead: '<S17>/mlHeartbeatLocal'
   */
  rtb_DataTypeConversion_gm = (uint8_T)mlHeartbeatLocal.custom_mode;

  /* S-Function "dsPIC_PWM_IC" Block: <S630>/Input Capture */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdROLLraw = MCHP_ic1up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o2 = MCHP_ic2up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o3 = MCHP_ic3up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o4 = MCHP_ic4up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o5 = MCHP_ic5up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdPITCHraw = MCHP_ic6up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o7 = MCHP_ic7up;

  /* MATLAB Function: '<S630>/myMux Fun5' */
  AUAV3_WITH_SLUGS_myMuxFun5(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o2,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o3,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o4,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o5,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCapture_o7,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5);

  /* MATLAB Function: '<S634>/Buffer Failsafe Channel' incorporates:
   *  DataTypeConversion: '<S631>/Data Type Conversion'
   *  Gain: '<S631>/Convert to  Microseconds'
   */
  AUAV3_WITH_BufferICChannel((uint16_T)(52429UL *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[4] >> 18),
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferFailsafeChannel,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferFailsafeChannel);

  /* DataTypeConversion: '<S634>/Data Type Conversion1' */
  for (i = 0; i < 7; i++) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1_n[i] = (uint8_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferFailsafeChannel.history[i];
  }

  /* End of DataTypeConversion: '<S634>/Data Type Conversion1' */

  /* S-Function (MCHP_C_function_Call): '<S634>/Choose the Median [navSupport.c] [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupdat = meanFilter5(
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1_n[0]
    );

  /* S-Function (MCHP_C_function_Call): '<S16>/Manual or Auto? [navSupport.c] [updated 4.28.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 = isApManual(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupdat
    );

  /* MATLAB Function: '<S629>/Buffer IC Channel' */
  AUAV3_WITH_BufferICChannel(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[0],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel);

  /* MATLAB Function: '<S629>/Buffer IC Channel1' */
  AUAV3_WITH_BufferICChannel(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[1],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel1,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel1);

  /* MATLAB Function: '<S629>/Buffer IC Channel2' */
  AUAV3_WITH_BufferICChannel(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[3],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel2,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel2);

  /* MATLAB Function: '<S629>/Buffer IC Channel3' */
  AUAV3_WITH_BufferICChannel(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[2],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel3,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel3);

  /* Logic: '<S632>/Logical Operator' */
  rtb_IC1_l = !(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4
                != 0);

  /* InitialCondition: '<S632>/IC1' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime = false;
    rtb_IC1_l = false;
  }

  /* End of InitialCondition: '<S632>/IC1' */

  /* DataTypeConversion: '<S645>/Data Type Conversion1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1_g = rtb_IC1_l;

  /* S-Function (MCHP_C_function_Call): '<S645>/C Function Call' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.CFunctionCall = justEnabled(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1_g
    , ((uint8_T)2U)
    );

  /* Outputs for Enabled SubSystem: '<S16>/Grab I.C.' incorporates:
   *  EnablePort: '<S633>/Enable'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.CFunctionCall > 0) {
    /* S-Function (MCHP_C_function_Call): '<S633>/Choose the Median navSupport.c [updated 4.28.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupd_d =
      meanFilter5(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel.history[0]
                  );

    /* Saturate: '<S633>/[0.55 0.68]' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupd_d > 8085U)
    {
      rtb_u2deg = 8085U;
    } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupd_d <
               7753U) {
      rtb_u2deg = 7753U;
    } else {
      rtb_u2deg =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChoosetheMediannavSupportcupd_d;
    }

    /* End of Saturate: '<S633>/[0.55 0.68]' */

    /* DataTypeConversion: '<S646>/Data Type Conversion' incorporates:
     *  Gain: '<S646>/Convert to  Microseconds'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_n = (uint16_T)(52429UL *
      rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S633>/update dT Trim updateControlMcu.c [updated 4.28.16]' */
    updatePWMTrim(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_n
                  , &AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled89
                  );

    /* DataTypeConversion: '<S650>/Data Type Conversion' incorporates:
     *  Constant: '<S650>/Constant1'
     *  Constant: '<S650>/Constant2'
     *  Product: '<S650>/Divide'
     *  Sum: '<S650>/Add'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion = (real32_T)((real_T)
      rtb_u2deg * 0.000392464678178964 + -2.4929356357927808);

    /* S-Function (MCHP_C_function_Call): '<S633>/2' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u = meanFilter5(
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel1.history[0]
      );

    /* Saturate: '<S633>/[-2  2] deg' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u > 7693U) {
      rtb_u2deg = 7693U;
    } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u < 7040U) {
      rtb_u2deg = 7040U;
    } else {
      rtb_u2deg = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u;
    }

    /* End of Saturate: '<S633>/[-2  2] deg' */

    /* DataTypeConversion: '<S647>/Data Type Conversion' incorporates:
     *  Gain: '<S647>/Convert to  Microseconds'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_h = (uint16_T)(52429UL *
      rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S633>/update dA Trim updateControlMcu.c' */
    updatePWMTrim(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_h
                  , &AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled90
                  );

    /* DataTypeConversion: '<S651>/Data Type Conversion' incorporates:
     *  Constant: '<S651>/Constant1'
     *  Constant: '<S651>/Constant2'
     *  Product: '<S651>/Divide'
     *  Sum: '<S651>/Add'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d = (real32_T)((real_T)
      rtb_u2deg * -0.00010684205998466119 + 0.787069841887004);

    /* S-Function (MCHP_C_function_Call): '<S633>/3' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_n = meanFilter5(
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel2.history[0]
      );

    /* Saturate: '<S633>/[-2  2] deg ' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_n > 7653U) {
      rtb_u2deg = 7653U;
    } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_n < 7247U) {
      rtb_u2deg = 7247U;
    } else {
      rtb_u2deg = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_n;
    }

    /* End of Saturate: '<S633>/[-2  2] deg ' */

    /* DataTypeConversion: '<S648>/Data Type Conversion' incorporates:
     *  Gain: '<S648>/Convert to  Microseconds'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_l = (uint16_T)(52429UL *
      rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S633>/update dR Trim updateControlMcu.c' */
    updatePWMTrim(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_l
                  , &AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled91
                  );

    /* DataTypeConversion: '<S652>/Data Type Conversion' incorporates:
     *  Constant: '<S652>/Constant1'
     *  Constant: '<S652>/Constant2'
     *  Product: '<S652>/Divide'
     *  Sum: '<S652>/Add'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j = (real32_T)((real_T)
      rtb_u2deg * -0.00017229941427366325 + 1.2836306363387913);

    /* S-Function (MCHP_C_function_Call): '<S633>/4' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_l = meanFilter5(
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_BufferICChannel3.history[0]
      );

    /* Saturate: '<S633>/[-2  2] deg  ' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_l > 7955U) {
      rtb_u2deg = 7955U;
    } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_l < 7112U) {
      rtb_u2deg = 7112U;
    } else {
      rtb_u2deg = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u_l;
    }

    /* End of Saturate: '<S633>/[-2  2] deg  ' */

    /* DataTypeConversion: '<S649>/Data Type Conversion' incorporates:
     *  Gain: '<S649>/Convert to  Microseconds'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_dm = (uint16_T)(52429UL *
      rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S633>/update dE Trim updateControlMcu.c' */
    updatePWMTrim(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_dm
                  , &AUAV3_WITH_SLUGS_SENSOR__ConstP.dE_Value
                  );

    /* DataTypeConversion: '<S653>/Data Type Conversion' incorporates:
     *  Constant: '<S653>/Constant1'
     *  Constant: '<S653>/Constant2'
     *  Product: '<S653>/Divide'
     *  Sum: '<S653>/Add'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k = (real32_T)((real_T)
      rtb_u2deg * 8.2860394028366822E-5 + -0.62421496834703016);
  }

  /* End of Outputs for SubSystem: '<S16>/Grab I.C.' */

  /* Logic: '<S8>/Logical Operator' incorporates:
   *  Constant: '<S427>/Constant'
   *  Constant: '<S428>/Constant'
   *  Constant: '<S429>/Constant'
   *  DataStoreRead: '<Root>/isPassthrough'
   *  RelationalOperator: '<S427>/Compare'
   *  RelationalOperator: '<S428>/Compare'
   *  RelationalOperator: '<S429>/Compare'
   */
  rtb_IC1_l = (boolean_T)((mlHeartbeatLocal.custom_mode == 2UL) ^
    (mlHeartbeatLocal.custom_mode == 4UL)) ^ (mlHeartbeatLocal.custom_mode ==
    8UL);

  /* S-Function (MCHP_C_function_Call): '<S11>/Get the GS Location [updateSensorMCUState.c]' */
  getGSLocation(
                &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[
                0]
                );

  /* Gain: '<S505>/Deg2R' */
  rtb_Deg2R_idx_0 = 0.0174532924F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[2];
  rtb_Deg2R_idx_1 = 0.0174532924F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[1];

  /* S-Function (MCHP_Digital_Input): '<S14>/Enable HIL from  Control MCU1' */

  /* MCHP_Digital_Input Block: <S14>/Enable HIL from  Control MCU1/Output */
  rtb_EnableHILfromControlMCU1 = PORTDbits.RD2;/* Read pin D2 */

  /* Logic: '<S566>/Logical Operator' */
  rtb_LogicalOperator_bu = !rtb_EnableHILfromControlMCU1;

  /* Outputs for Enabled SubSystem: '<S566>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S568>/Enable'
   */
  if (rtb_LogicalOperator_bu) {
    /* MATLAB Function: '<S568>/Embedded MATLAB Function' incorporates:
     *  Constant: '<S568>/Constant'
     */
    A_EmbeddedMATLABFunction_o(AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled9,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_og);

    /* DataTypeConversion: '<S568>/Data Type Conversion' */
    tmp_7 = floor(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_og.y);
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    i = tmp_7 < 0.0 ? -(int16_T)(uint16_T)-tmp_7 : (int16_T)(uint16_T)tmp_7;

    /* End of DataTypeConversion: '<S568>/Data Type Conversion' */

    /* MATLAB Function: '<S568>/Embedded MATLAB Function1' incorporates:
     *  Constant: '<S568>/Constant1'
     */
    A_EmbeddedMATLABFunction_o(AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled9,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_g3);

    /* DataTypeConversion: '<S568>/Data Type Conversion1' */
    tmp_7 = floor
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_g3.y);
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    rtb_DataTypeConversion1_a = tmp_7 < 0.0 ? -(int16_T)(uint16_T)-tmp_7 :
      (int16_T)(uint16_T)tmp_7;

    /* End of DataTypeConversion: '<S568>/Data Type Conversion1' */

    /* MATLAB Function: '<S568>/Embedded MATLAB Function2' incorporates:
     *  Constant: '<S568>/Constant2'
     */
    A_EmbeddedMATLABFunction_o(AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled9,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_ix);

    /* DataTypeConversion: '<S568>/Data Type Conversion2' */
    tmp_7 = floor
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_ix.y);
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    rtb_DataTypeConversion2_b = tmp_7 < 0.0 ? -(int16_T)(uint16_T)-tmp_7 :
      (int16_T)(uint16_T)tmp_7;

    /* End of DataTypeConversion: '<S568>/Data Type Conversion2' */

    /* MATLAB Function: '<S568>/Embedded MATLAB Function3' incorporates:
     *  Constant: '<S568>/Constant3'
     */
    A_EmbeddedMATLABFunction_o(AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled9,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction3_n);

    /* DataTypeConversion: '<S568>/Data Type Conversion3' */
    tmp_7 = floor(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction3_n.y);
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    rtb_DataTypeConversion3_f = tmp_7 < 0.0 ? -(int16_T)(uint16_T)-tmp_7 :
      (int16_T)(uint16_T)tmp_7;

    /* End of DataTypeConversion: '<S568>/Data Type Conversion3' */

    /* MATLAB Function: '<S568>/myMux Fun' */
    /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S597>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S597>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_j[0] = i;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_j[1] = rtb_DataTypeConversion1_a;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_j[2] = rtb_DataTypeConversion2_b;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_j[3] = rtb_DataTypeConversion3_f;

    /* S-Function (MCHP_C_function_Call): '<S568>/Update State AP ADC Data [updateSensorMcuState.c]1' */
    updateRawADCData(
                     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_j[0]
                     );

    /* S-Function (MCHP_C_function_Call): '<S568>/Read the Cube Data [adisCube16405.c]1' */
    getCubeData(
                &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1
                [0]
                );

    /* MATLAB Function: '<S568>/myMux Fun4' */
    /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S598>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S598>:1:5' y = [u1(1); u2(1); u3(1); u4(1); u5(1); u6(1); u7(1); u8(1); u9(1); u10(1); u11(1); u12(1); u13(1)]; */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[2] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[2];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[3] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[3];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[4] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[4];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[5] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[5];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[6] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[6];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[7] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[7];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[8] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheCubeDataadisCube16405c1[8];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[9] = i;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[10] = rtb_DataTypeConversion1_a;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[11] = rtb_DataTypeConversion2_b;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[12] = rtb_DataTypeConversion3_f;

    /* S-Function (MCHP_C_function_Call): '<S568>/Is the GPS Novatel or Ublox? [gpsPort.c]1' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IstheGPSNovatelorUbloxgpsPortc1 =
      isGPSNovatel(
                   );

    /* Outputs for Enabled SubSystem: '<S568>/if GPS is Novatel' incorporates:
     *  EnablePort: '<S595>/Enable'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IstheGPSNovatelorUbloxgpsPortc1 > 0) {
      /* S-Function (MCHP_C_function_Call): '<S595>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]2' */
      getGpsMainData(
                     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ProducetheGPSMainDataandupdat_b
                     [0]
                     );

      /* S-Function (MCHP_C_function_Call): '<S595>/Read the Raw Data from GPS [gpsPort.c]2' */
      getGPSRawData(
                    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheRawDatafromGPSgpsPortc2
                    );

      /* S-Function (MCHP_C_function_Call): '<S595>/Parse the GPS RAW Data [gps.c//novatel.c]2' */
      gpsParse(
               &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ReadtheRawDatafromGPSgpsPortc2
               );
    }

    /* End of Outputs for SubSystem: '<S568>/if GPS is Novatel' */

    /* Outputs for Enabled SubSystem: '<S568>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S596>/Enable'
     */
    /* Logic: '<S568>/Logical Operator' */
    if (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IstheGPSNovatelorUbloxgpsPortc1 != 0))
    {
      /* S-Function (MCHP_C_function_Call): '<S596>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]1' */
      getGpsUbloxMainData(
                          &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ProducetheGPSMainDataandupdatet
                          [0]
                          );
    }

    /* End of Logic: '<S568>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S568>/if GPS is Ublox' */

    /* Switch: '<S568>/Switch2' */
    for (i = 0; i < 5; i++) {
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IstheGPSNovatelorUbloxgpsPortc1 > 0) {
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[i] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ProducetheGPSMainDataandupdat_b[i];
      } else {
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[i] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ProducetheGPSMainDataandupdatet[i];
      }
    }

    /* End of Switch: '<S568>/Switch2' */
  }

  /* End of Outputs for SubSystem: '<S566>/If no HIL then Read all the Sensors' */

  /* Gain: '<S506>/Deg2R' */
  rtb_Sum_b = 0.0174532924F * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[0];

  /* Trigonometry: '<S506>/sin(phi)' */
  rtb_kxi = (real32_T)sin(rtb_Sum_b);

  /* Sum: '<S506>/Sum1' incorporates:
   *  Constant: '<S506>/const'
   *  Product: '<S506>/Product1'
   *  Product: '<S506>/sin(phi)^2'
   */
  rtb_Sum1 = 1.0F - rtb_kxi * rtb_kxi * 0.00669425726F;

  /* Fcn: '<S506>/f' */
  if (rtb_Sum1 < 0.0F) {
    rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
  } else {
    rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
  }

  /* End of Fcn: '<S506>/f' */

  /* Product: '<S506>/Rh' incorporates:
   *  Constant: '<S506>/Re=equatorial radius'
   */
  rtb_ixj = 6.378137E+6F / rtb_Sum1;

  /* Sum: '<S506>/Sum2' */
  rtb_kxj = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[2] + rtb_ixj;

  /* Trigonometry: '<S506>/cos(phi)' */
  rtb_Sum_b = (real32_T)cos(rtb_Sum_b);

  /* Gain: '<S506>/Deg2R1' */
  rtb_Sum1 = 0.0174532924F * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[1];

  /* Product: '<S506>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S506>/cos(lamb)'
   */
  rtb_jxi = rtb_kxj * rtb_Sum_b * (real32_T)cos(rtb_Sum1);

  /* Product: '<S506>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S506>/sin(lamb)'
   */
  rtb_kxj = rtb_kxj * rtb_Sum_b * (real32_T)sin(rtb_Sum1);

  /* Product: '<S506>/Ze' incorporates:
   *  Product: '<S506>/Rh(1-e^2)'
   *  Sum: '<S506>/Sum4'
   */
  rtb_kxi *= 0.993305743F * rtb_ixj + AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[2];

  /* Gain: '<S497>/Deg2R' */
  rtb_Sum_b = 0.0174532924F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[1];

  /* Trigonometry: '<S497>/sin(phi)' */
  rtb_ixj = (real32_T)sin(rtb_Sum_b);

  /* Sum: '<S497>/Sum1' incorporates:
   *  Constant: '<S497>/const'
   *  Product: '<S497>/Product1'
   *  Product: '<S497>/sin(phi)^2'
   */
  rtb_Sum1 = 1.0F - rtb_ixj * rtb_ixj * 0.00669425726F;

  /* Fcn: '<S497>/f' */
  if (rtb_Sum1 < 0.0F) {
    rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
  } else {
    rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
  }

  /* End of Fcn: '<S497>/f' */

  /* Product: '<S497>/Rh' incorporates:
   *  Constant: '<S497>/Re=equatorial radius'
   */
  rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

  /* Sum: '<S497>/Sum2' */
  rtb_RhhcosphisinlambYe_h =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[0] +
    rtb_Sum1;

  /* Trigonometry: '<S497>/cos(phi)' */
  rtb_Sum_b = (real32_T)cos(rtb_Sum_b);

  /* Gain: '<S497>/Deg2R1' */
  rtb_Deg2R1_bc = 0.0174532924F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[2];

  /* Product: '<S497>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S497>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe_cj = rtb_RhhcosphisinlambYe_h * rtb_Sum_b * (real32_T)
    cos(rtb_Deg2R1_bc);

  /* Product: '<S497>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S497>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe_h = rtb_RhhcosphisinlambYe_h * rtb_Sum_b * (real32_T)
    sin(rtb_Deg2R1_bc);

  /* Product: '<S497>/Ze' incorporates:
   *  Product: '<S497>/Rh(1-e^2)'
   *  Sum: '<S497>/Sum4'
   */
  rtb_ixj *= 0.993305743F * rtb_Sum1 +
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorMCU[0];

  /* Sum: '<S441>/Sum1' */
  rtb_DataTypeConversion1_jq_idx_ = rtb_jxi - rtb_RhhcosphicoslambXe_cj;
  rtb_DataTypeConversion1_jq_id_0 = rtb_kxj - rtb_RhhcosphisinlambYe_h;
  rtb_kxj = rtb_kxi - rtb_ixj;

  /* S-Function (MCHP_C_function_Call): '<S11>/Checks if FixType is 3 [updateSensorMCUState.c]1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChecksifFixTypeis3updateSensorM = isFixValid(
    );

  /* Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S439>/Enable'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ChecksifFixTypeis3updateSensorM > 0) {
    /* SignalConversion: '<S505>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S508>/11'
     *  Fcn: '<S508>/12'
     *  Fcn: '<S508>/13'
     *  Fcn: '<S508>/21'
     *  Fcn: '<S508>/22'
     *  Fcn: '<S508>/23'
     *  Fcn: '<S508>/31'
     *  Fcn: '<S508>/32'
     *  Fcn: '<S508>/33'
     */
    rtb_MathFunction[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos
      (rtb_Deg2R_idx_1);
    rtb_MathFunction[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    rtb_MathFunction[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos
      (rtb_Deg2R_idx_0);
    rtb_MathFunction[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos
      (rtb_Deg2R_idx_1);
    rtb_MathFunction[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    rtb_MathFunction[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin
      (rtb_Deg2R_idx_1);
    rtb_MathFunction[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    rtb_MathFunction[7] = 0.0F;
    rtb_MathFunction[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Product: '<S505>/Product1' incorporates:
     *  Gain: '<S441>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = rtb_MathFunction[i + 6] * rtb_kxj + (rtb_MathFunction[i + 3] *
        rtb_DataTypeConversion1_jq_id_0 + rtb_MathFunction[i] *
        rtb_DataTypeConversion1_jq_idx_);
    }

    /* End of Product: '<S505>/Product1' */

    /* Inport: '<S439>/In1' */
    for (i = 0; i < 3; i++) {
      /* Gain: '<S441>/UEN 2 NEU' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[i] = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2];
    }

    /* End of Inport: '<S439>/In1' */

    /* Inport: '<S439>/In2' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In2 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[3];

    /* Inport: '<S439>/In3' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In3 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2[4];
  }

  /* End of Outputs for SubSystem: '<S11>/Enabled Subsystem' */

  /* Gain: '<S494>/Unit Conversion' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.UnitConversion = 0.0174532924F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In2;

  /* S-Function (MCHP_C_function_Call): '<S495>/[apUtils.c]1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc1 = myCos(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.UnitConversion
    );

  /* Product: '<S437>/Product1' */
  rtb_RhhcosphicoslambXe_cj = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In3 *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc1;

  /* DataTypeConversion: '<S528>/Data Type Conversion2' incorporates:
   *  Gain: '<S11>/[1 1 -1]'
   *  Gain: '<S513>/Gain'
   *  Sum: '<S513>/Sum'
   *  Sum: '<S513>/Sum1'
   */
  rtb_DataTypeConversion2 = (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[0] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE) * 10.0F +
    rtb_RhhcosphicoslambXe_cj;

  /* DiscreteZeroPole: '<S529>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_k = 0.005*rtb_DataTypeConversion2;
    rtb_DiscreteZeroPole_k += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE;
  }

  /* DataTypeConversion: '<S528>/Data Type Conversion1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE = (real32_T)
    rtb_DiscreteZeroPole_k;

  /* S-Function (MCHP_C_function_Call): '<S495>/[apUtils.c]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc = mySin(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.UnitConversion
    );

  /* Product: '<S437>/Product' */
  rtb_kxj = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In3;

  /* DataTypeConversion: '<S530>/Data Type Conversion2' incorporates:
   *  Gain: '<S11>/[1 1 -1]'
   *  Gain: '<S514>/Gain'
   *  Sum: '<S514>/Sum'
   *  Sum: '<S514>/Sum1'
   */
  rtb_DataTypeConversion2_p = (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[1] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE_j) * 10.0F + rtb_kxj;

  /* DiscreteZeroPole: '<S531>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_k = 0.005*rtb_DataTypeConversion2_p;
    rtb_DiscreteZeroPole_k += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* DataTypeConversion: '<S530>/Data Type Conversion1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE_j = (real32_T)
    rtb_DiscreteZeroPole_k;

  /* Abs: '<S501>/Abs1' incorporates:
   *  Sum: '<S504>/Diff'
   *
   * Block description for '<S504>/Diff':
   *
   *  Add in CPU
   */
  rtb_jxi = (real32_T)fabs(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[2] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_o);

  /* Abs: '<S500>/Abs1' incorporates:
   *  Sum: '<S503>/Diff'
   *
   * Block description for '<S503>/Diff':
   *
   *  Add in CPU
   */
  rtb_Sum1 = (real32_T)fabs(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[1] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_m);

  /* Abs: '<S499>/Abs1' incorporates:
   *  Sum: '<S502>/Diff'
   *
   * Block description for '<S502>/Diff':
   *
   *  Add in CPU
   */
  rtb_Deg2R1_bc = (real32_T)fabs(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[0] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE);

  /* Saturate: '<S499>/Saturation1' */
  if (rtb_Deg2R1_bc > 1.0F) {
    rtb_Deg2R1_bc = 1.0F;
  }

  /* Saturate: '<S500>/Saturation1' */
  if (rtb_Sum1 > 1.0F) {
    rtb_Sum1 = 1.0F;
  }

  /* Saturate: '<S501>/Saturation1' */
  if (rtb_jxi > 1.0F) {
    rtb_jxi = 1.0F;
  }

  /* Sum: '<S440>/Sum' incorporates:
   *  Saturate: '<S499>/Saturation1'
   *  Saturate: '<S500>/Saturation1'
   *  Saturate: '<S501>/Saturation1'
   */
  rtb_jxi += rtb_Deg2R1_bc + rtb_Sum1;

  /* Saturate: '<S440>/Saturation1' */
  if (rtb_jxi > 1.0F) {
    rtb_jxi = 1.0F;
  }

  /* DataTypeConversion: '<S440>/Data Type Conversion2' incorporates:
   *  Saturate: '<S440>/Saturation1'
   */
  rtb_jxi = (real32_T)floor(rtb_jxi);
  if (rtIsNaNF(rtb_jxi) || rtIsInfF(rtb_jxi)) {
    rtb_DataTypeConversion2_j = 0U;
  } else {
    rtb_DataTypeConversion2_j = (uint8_T)(real32_T)fmod(rtb_jxi, 256.0F);
  }

  /* End of DataTypeConversion: '<S440>/Data Type Conversion2' */

  /* MATLAB Function: '<S511>/Embedded MATLAB Function3' incorporates:
   *  Gain: '<S11>/[1 1 -1]'
   */
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Position Filter [updated 5.13.16]/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3': '<S521>:1' */
  /*  persistent lastH; */
  /* '<S521>:1:7' if (isempty(lastGps_h)) */
  if (!AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h_not_empty) {
    /* '<S521>:1:8' lastGps_h       = single(0.0); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h_not_empty = true;

    /* '<S521>:1:9' TimeSinceLast   = single(apSampleTime); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.TimeSinceLast = 0.01F;

    /* '<S521>:1:10' rate            = single(0); */
    /*      lastH           = single(-baseHeight); */
    /*      h_rate          = single(0); */
  }

  /* '<S521>:1:15' if (NewGPS) */
  if (rtb_DataTypeConversion2_j != 0) {
    /* '<S521>:1:16' rate = single((gps_h - lastGps_h)/TimeSinceLast); */
    rtb_Sum1 = (-AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[2] -
                AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h) /
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.TimeSinceLast;

    /* '<S521>:1:17' TimeSinceLast = single(0); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.TimeSinceLast = 0.0F;

    /* '<S521>:1:18' if (rate > -10.6) && (rate < 17.67) */
    if (!((rtb_Sum1 > -10.6) && (rtb_Sum1 < 17.67))) {
      /* '<S521>:1:21' else */
      /*          h = lastH; */
      /* '<S521>:1:23' h_rate = single(0); */
      rtb_Sum1 = 0.0F;
    } else {
      /*          h = lastH + gps_h - lastGps_h; */
      /* '<S521>:1:20' h_rate = rate; */
    }

    /* '<S521>:1:25' lastGps_h = gps_h; */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h =
      -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[2];
  } else {
    /* '<S521>:1:26' else */
    /*      h = lastH; */
    /* '<S521>:1:28' h_rate = single(0); */
    rtb_Sum1 = 0.0F;
  }

  /* '<S521>:1:31' TimeSinceLast = TimeSinceLast + apSampleTime; */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.TimeSinceLast += 0.01F;

  /* End of MATLAB Function: '<S511>/Embedded MATLAB Function3' */
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

  /* S-Function (MCHP_C_function_Call): '<S613>/Get the GS Location [updateSensorMCUState.c]1' */
  getGSLocation(
                &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorM_d[
                0]
                );

  /* MATLAB Function: '<S599>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputatio,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S599>/Zero Out Height' */
  AUAV3_WITH_S_ZeroOutHeight
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputatio.tOut,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorM_d[0],
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ZeroOutHeight,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.ZeroOutHeight);

  /* End of Outputs for SubSystem: '<S599>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S14>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S565>/Enable'
   */
  if (rtb_EnableHILfromControlMCU1) {
    /* S-Function (MCHP_C_function_Call): '<S565>/Data from HIL [hil.c]2' */
    hilRead(
            &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DatafromHILhilc2[0]
            );

    /* S-Function (MCHP_C_function_Call): '<S565>/HIL Messages  Parser//Decoder [hil.c]1' */
    protDecodeHil(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DatafromHILhilc2[0]
                  );

    /* S-Function (MCHP_C_function_Call): '<S565>/HIL Raw Readings [hil.c]1' */
    hil_getRawRead(
                   &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.HILMessagesParserDecoderhilc1
                   [0]
                   );
  }

  /* End of Outputs for SubSystem: '<S14>/Raw HIL  Readings' */

  /* Switch: '<S566>/Switch' */
  for (i = 0; i < 13; i++) {
    if (rtb_EnableHILfromControlMCU1) {
      rtb_Switch_a[i] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.HILRawReadingshilc1[i];
    } else {
      rtb_Switch_a[i] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_d[i];
    }
  }

  /* End of Switch: '<S566>/Switch' */

  /* MATLAB Function: '<S606>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S606>/Constant'
   *  Constant: '<S606>/Constant1'
   *  DataTypeConversion: '<S570>/Data Type Conversion5'
   */
  A_EmbeddedMATLABFunction_n((real_T)rtb_Switch_a[12], 0.01, 0.02,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction);

  /* Sum: '<S605>/Sum' incorporates:
   *  Constant: '<S605>/Bias'
   *  Product: '<S605>/Divide'
   */
  rtb_Sum = (real32_T)(1.5112853050231934 *
                       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction.y)
    + -1605.28198F;

  /* RelationalOperator: '<S621>/Compare' incorporates:
   *  Constant: '<S621>/Constant'
   */
  rtb_Compare_n = (uint8_T)(rtb_Sum < -50.0F);

  /* Outputs for Enabled SubSystem: '<S604>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S622>/Enable'
   */
  /* Logic: '<S604>/Logical Operator' */
  if (!(rtb_Compare_n != 0)) {
    /* Sum: '<S622>/Add' incorporates:
     *  Constant: '<S622>/Constant'
     *  Constant: '<S622>/Mean Temperature for Calibration'
     *  Constant: '<S622>/gains'
     *  Product: '<S622>/Divide1'
     *  Sum: '<S622>/Sum1'
     *  Sum: '<S622>/Sum2'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge = ((real32_T)rtb_Switch_a[9] -
      (rtb_Sum - 347.23F) * 0.0207608F) + -6.0F;
  }

  /* End of Logic: '<S604>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S604>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S604>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S623>/Enable'
   */
  if (rtb_Compare_n > 0) {
    /* Sum: '<S623>/Sum2' incorporates:
     *  Constant: '<S623>/Mean Temperature for Calibration'
     *  Constant: '<S623>/gains'
     *  Product: '<S623>/Divide1'
     *  Sum: '<S623>/Sum1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge = (real32_T)rtb_Switch_a[9] - (rtb_Sum
      - -161.3F) * -0.0102663F;
  }

  /* End of Outputs for SubSystem: '<S604>/Lo Temp Compensation' */

  /* MATLAB Function: '<S609>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S609>/Constant'
   *  Constant: '<S609>/Constant1'
   *  DataTypeConversion: '<S570>/Data Type Conversion19'
   */
  A_EmbeddedMATLABFunction_n((real_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge, 0.01,
    4.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_f,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_f);

  /* Sum: '<S600>/Sum' incorporates:
   *  Constant: '<S600>/Bias'
   *  Constant: '<S600>/Gains'
   *  DataTypeConversion: '<S570>/Data Type Conversion1'
   *  Product: '<S600>/Divide'
   */
  rtb_Sum_cm = 27.127F * (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_f.y + 9444.44434F;

  /* Outputs for Enabled SubSystem: '<S599>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S614>/Enable'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputatio.tOut > 0.0)
  {
    /* DataTypeConversion: '<S614>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Sum_cm;

    /* DiscreteZeroPole: '<S617>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_e = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole_e += 0.029119852459414206*
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_e;
    }

    /* Saturate: '<S614>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S614>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_e > 120000.0F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_e < 80000.0F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k = 80000.0F;
    } else {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_e;
    }

    /* End of Saturate: '<S614>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S617>/Discrete Zero-Pole' */
    {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_e =
        rtb_DataTypeConversion + 0.97044334975369462*
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_e;
    }
  }

  /* End of Outputs for SubSystem: '<S599>/Initial Baro Bias' */

  /* Product: '<S610>/Divide' incorporates:
   *  Sum: '<S610>/Sum2'
   */
  rtb_Sum_b = (rtb_Sum_cm - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k) /
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k;

  /* Sum: '<S610>/Sum1' incorporates:
   *  Constant: '<S610>/Constant2'
   *  Constant: '<S610>/Constant3'
   *  Constant: '<S610>/Constant4'
   *  Constant: '<S610>/Constant5'
   *  Gain: '<S616>/Unit Conversion'
   *  Product: '<S610>/Divide1'
   *  Product: '<S610>/Divide2'
   *  Product: '<S610>/Divide3'
   *  Product: '<S610>/Divide4'
   *  Sum: '<S610>/Sum3'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1 = ((rtb_Sum_b * rtb_Sum_b * 0.093502529F
    + rtb_Sum_b * -0.188893303F) + 2.18031291E-5F) * 145473.5F * 0.3048F +
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GettheGSLocationupdateSensorM_d[0];

  /* Outputs for Enabled SubSystem: '<S599>/Enabled Subsystem' */

  /* Logic: '<S599>/Logical Operator' incorporates:
   *  Sum: '<S599>/Sum1'
   */
  AUAV3_WIT_EnabledSubsystem
    (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputatio.tOut !=
       0.0), AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ZeroOutHeight.Sum +
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S599>/Enabled Subsystem' */

  /* MATLAB Function: '<S522>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S522>/Constant'
   *  Constant: '<S522>/Constant1'
   *  Gain: '<S510>/Gain'
   *  Gain: '<S511>/Gain'
   *  Sum: '<S511>/Sum'
   */
  AUA_EmbeddedMATLABFunction(0.5F * rtb_Sum1 +
    -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_m.In1, 0.01,
    0.31830988618379069,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fw,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fw);

  /* MATLAB Function: '<S510>/myMux Fun1' incorporates:
   *  Gain: '<S510>/Gain1'
   */
  AUAV3_WITH_SLU_myMuxFun1_f(AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE,
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE_j,
    -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fw.y,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1);

  /* Outputs for Enabled SubSystem: '<S510>/Subsystem' incorporates:
   *  EnablePort: '<S518>/Enable'
   */
  /* RelationalOperator: '<S516>/Compare' incorporates:
   *  Constant: '<S516>/Constant'
   */
  if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In3 > 7.0F) > 0) {
    if (!AUAV3_WITH_SLUGS_SENSOR_A_DWork.Subsystem_MODE) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.Subsystem_MODE = true;
    }

    /* MATLAB Function: '<S518>/Embedded MATLAB Function1' */
    AU_EmbeddedMATLABFunction1(rtb_kxj, rtb_DataTypeConversion2_j,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_m,
      &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_m);

    /* MATLAB Function: '<S518>/Embedded MATLAB Function2' */
    AU_EmbeddedMATLABFunction1(rtb_RhhcosphicoslambXe_cj,
      rtb_DataTypeConversion2_j,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_c,
      &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_c);
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Subsystem_MODE) {
      /* Disable for Outport: '<S518>/Vn_fil' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_c.y = 0.0F;

      /* Disable for Outport: '<S518>/Ve_fil' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_m.y = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.Subsystem_MODE = false;
    }
  }

  /* End of RelationalOperator: '<S516>/Compare' */
  /* End of Outputs for SubSystem: '<S510>/Subsystem' */

  /* Gain: '<S515>/Gain' incorporates:
   *  Sum: '<S515>/Sum1'
   */
  rtb_Gain_px = (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fw.y
                 - AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE_jx) * 2.0F;

  /* MATLAB Function: '<S510>/myMux Fun2' */
  AUAV3_WITH_SLU_myMuxFun1_f
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_c.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_m.y, rtb_Gain_px,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2);

  /* S-Function (MCHP_C_function_Call): '<Root>/Get RTB Order [navSupport.c] [updated 4.27.16]' */
  getRTB(
         &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRTBOrdernavSupportcupdated42[0]
         );

  /* Outputs for Atomic SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */
  /* DataTypeConversion: '<S34>/Data Type Conversion' incorporates:
   *  DataStoreRead: '<S34>/Get Nav Mode'
   */
  rtb_DataTypeConversion2_j = (uint8_T)mlHeartbeatLocal.custom_mode;

  /* InitialCondition: '<S151>/IC1' incorporates:
   *  DataStoreRead: '<Root>/isWpFly?'
   *  Logic: '<S142>/Logical Operator'
   *  Logic: '<S142>/Logical Operator1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime_e) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime_e = false;
    rtb_IC1 = false;
  } else {
    rtb_IC1 = ((mlHeartbeatLocal.custom_mode != 0UL) &&
               (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4
                  != 0)));
  }

  /* End of InitialCondition: '<S151>/IC1' */

  /* DataTypeConversion: '<S421>/Data Type Conversion2' incorporates:
   *  Delay: '<S426>/Delay'
   *  Logic: '<S426>/Logical Operator'
   *  RelationalOperator: '<S426>/Relational Operator'
   */
  rtb_Compare_n = (uint8_T)((AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay_DSTATE_p !=
    rtb_IC1) && rtb_IC1);

  /* InitialCondition: '<S151>/IC2' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC2_FirstOutputTime) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC2_FirstOutputTime = false;
    rtb_IC2 = 0U;
  } else {
    rtb_IC2 = rtb_Compare_n;
  }

  /* End of InitialCondition: '<S151>/IC2' */

  /* Outputs for Enabled SubSystem: '<S151>/Grab Upon Enable' incorporates:
   *  EnablePort: '<S420>/Enable'
   */
  if (rtb_IC2 > 0) {
    /* SignalConversion: '<S423>/Numerical Unity' incorporates:
     *  DataStoreRead: '<S420>/Get GS Location'
     *  DataTypeConversion: '<S420>/Data Type Conversion2'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[0] = (real32_T)
      mlGSLocationFloat.lat;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[1] = (real32_T)
      mlGSLocationFloat.lon;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[2] = (real32_T)
      mlGSLocationFloat.alt;

    /* Gain: '<S424>/Deg2R' */
    rtb_RhhcosphisinlambYe_h = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[1];

    /* Trigonometry: '<S424>/sin(phi)' */
    rtb_kxj = (real32_T)sin(rtb_RhhcosphisinlambYe_h);

    /* Sum: '<S424>/Sum1' incorporates:
     *  Constant: '<S424>/const'
     *  Product: '<S424>/Product1'
     *  Product: '<S424>/sin(phi)^2'
     */
    rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

    /* Fcn: '<S424>/f' */
    if (rtb_Sum1 < 0.0F) {
      rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
    } else {
      rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
    }

    /* End of Fcn: '<S424>/f' */

    /* Product: '<S424>/Rh' incorporates:
     *  Constant: '<S424>/Re=equatorial radius'
     */
    rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

    /* Sum: '<S424>/Sum2' */
    rtb_Deg2R1_bc = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[0] + rtb_Sum1;

    /* Trigonometry: '<S424>/cos(phi)' */
    rtb_RhhcosphisinlambYe_h = (real32_T)cos(rtb_RhhcosphisinlambYe_h);

    /* Gain: '<S424>/Deg2R1' */
    rtb_RhhcosphicoslambXe_cj = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[2];

    /* Product: '<S424>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S424>/cos(lamb)'
     */
    rtb_jxi = rtb_Deg2R1_bc * rtb_RhhcosphisinlambYe_h * (real32_T)cos
      (rtb_RhhcosphicoslambXe_cj);

    /* Product: '<S424>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S424>/sin(lamb)'
     */
    rtb_Deg2R1_bc = rtb_Deg2R1_bc * rtb_RhhcosphisinlambYe_h * (real32_T)sin
      (rtb_RhhcosphicoslambXe_cj);

    /* Product: '<S424>/Ze' incorporates:
     *  Product: '<S424>/Rh(1-e^2)'
     *  Sum: '<S424>/Sum4'
     */
    rtb_kxj *= 0.993305743F * rtb_Sum1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[0];

    /* DataTypeConversion: '<S420>/Data Type Conversion' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_di[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[2];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_di[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[1];

    /* DataTypeConversion: '<S420>/Data Type Conversion1' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0] = rtb_jxi;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1] = rtb_Deg2R1_bc;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2] = rtb_kxj;
  }

  /* End of Outputs for SubSystem: '<S151>/Grab Upon Enable' */

  /* MATLAB Function: '<S142>/myMux Fun2' */
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun2': '<S155>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S155>:1:5' y = [u1(1); u2(1); u3(1)]; */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_a[0] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_a[1] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_a[2] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

  /* S-Function (MCHP_C_function_Call): '<S142>/Diagnoistics Set navsupport.c [updated 5.1.16]1' */
  setDiagnosticFloat(
                     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_a[0]
                     );

  /* InitialCondition: '<S151>/IC4' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC4_FirstOutputTime) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC4_FirstOutputTime = false;
    rtb_Sum_b = 36.9885063F;
    rtb_ixj = -122.055305F;
  } else {
    rtb_Sum_b = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_di[0];
    rtb_ixj = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_di[1];
  }

  /* End of InitialCondition: '<S151>/IC4' */

  /* MATLAB Function: '<S142>/Zero out Z2' */
  AUAV3_WITH_SLUGS_ZerooutZ1(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1.y,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2);

  /* MATLAB Function: '<S142>/Zero out Z1' */
  AUAV3_WITH_SLUGS_ZerooutZ1(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1);

  /* MATLAB Function: '<S225>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_g(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_g);

  /* MATLAB Function: '<S226>/negprotect' */
  AUAV3_WITH_SL_negprotect_d
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_g.xDoty,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p);

  /* S-Function (MCHP_C_function_Call): '<S226>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116 = mySqrt(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p.zpVal
    );

  /* InitialCondition: '<S148>/IC' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime = false;
    rtb_Sum1 = 1.5F;
  } else {
    rtb_Sum1 = mlParamInterface.param[25];
  }

  /* End of InitialCondition: '<S148>/IC' */

  /* Saturate: '<S148>/[1.5 10]' */
  if (rtb_Sum1 > 10.0F) {
    rtb_Sum1 = 10.0F;
  } else {
    if (rtb_Sum1 < 1.5F) {
      rtb_Sum1 = 1.5F;
    }
  }

  /* Product: '<S148>/Product3' incorporates:
   *  Constant: '<S148>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S148>/Product1'
   *  Product: '<S148>/Product2'
   *  Saturate: '<S148>/[1.5 10]'
   */
  rtb_kxi = mlMidLevelCommands.hCommand * mlMidLevelCommands.hCommand * rtb_Sum1
    / 4.57681F;

  /* If: '<S142>/Determine Overall Nav by the Nav Mode' incorporates:
   *  Constant: '<S143>/RTB1'
   *  Constant: '<S147>/RTB'
   *  Constant: '<S147>/RTB1'
   *  Constant: '<S150>/RTB'
   *  Constant: '<S150>/RTB1'
   *  Constant: '<S343>/Constant'
   *  Constant: '<S343>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Gain: '<S168>/Deg2R'
   *  Gain: '<S409>/Deg2R'
   *  Inport: '<S147>/MidLvl h_c'
   *  Inport: '<S150>/MidLvl h_c'
   *  Product: '<S292>/Divide'
   *  Product: '<S347>/Divide'
   *  Sum: '<S286>/Add'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRTBOrdernavSupportcupdated42[0] == 1) {
    /* Outputs for IfAction SubSystem: '<S142>/RTB//Follow Mobile Navigation' incorporates:
     *  ActionPort: '<S150>/Action Port'
     */
    /* Outputs for Enabled SubSystem: '<S150>/Compute Mobile Location' incorporates:
     *  EnablePort: '<S404>/Enable'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRTBOrdernavSupportcupdated42[1] > 0)
    {
      rtb_Sum_b *= 0.0174532924F;

      /* Gain: '<S409>/Deg2R' */
      rtb_ixj *= 0.0174532924F;

      /* Gain: '<S410>/Deg2R' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_jxi = 0.0174532924F * mlMobileLocation.latitude;

      /* Trigonometry: '<S410>/sin(phi)' */
      rtb_RhhcosphisinlambYe_h = (real32_T)sin(rtb_jxi);

      /* Sum: '<S410>/Sum1' incorporates:
       *  Constant: '<S410>/const'
       *  Product: '<S410>/Product1'
       *  Product: '<S410>/sin(phi)^2'
       */
      rtb_Sum1 = 1.0F - rtb_RhhcosphisinlambYe_h * rtb_RhhcosphisinlambYe_h *
        0.00669425726F;

      /* Fcn: '<S410>/f' */
      if (rtb_Sum1 < 0.0F) {
        rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
      } else {
        rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
      }

      /* End of Fcn: '<S410>/f' */

      /* Product: '<S410>/Rh' incorporates:
       *  Constant: '<S410>/Re=equatorial radius'
       */
      rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

      /* Trigonometry: '<S410>/cos(phi)' */
      rtb_jxi = (real32_T)cos(rtb_jxi);

      /* Gain: '<S410>/Deg2R1' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_Deg2R1_bc = 0.0174532924F * mlMobileLocation.longitude;

      /* Product: '<S410>/Ze' incorporates:
       *  Product: '<S410>/Rh(1-e^2)'
       */
      rtb_RhhcosphisinlambYe_h *= 0.993305743F * rtb_Sum1;

      /* SignalConversion: '<S409>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S412>/11'
       *  Fcn: '<S412>/12'
       *  Fcn: '<S412>/13'
       *  Fcn: '<S412>/21'
       *  Fcn: '<S412>/22'
       *  Fcn: '<S412>/23'
       *  Fcn: '<S412>/31'
       *  Fcn: '<S412>/32'
       *  Fcn: '<S412>/33'
       */
      tmp_0[0] = (real32_T)cos(rtb_Sum_b) * (real32_T)cos(rtb_ixj);
      tmp_0[1] = -(real32_T)sin(rtb_Sum_b);
      tmp_0[2] = -(real32_T)sin(rtb_ixj) * (real32_T)cos(rtb_Sum_b);
      tmp_0[3] = (real32_T)sin(rtb_Sum_b) * (real32_T)cos(rtb_ixj);
      tmp_0[4] = (real32_T)cos(rtb_Sum_b);
      tmp_0[5] = -(real32_T)sin(rtb_Sum_b) * (real32_T)sin(rtb_ixj);
      tmp_0[6] = (real32_T)sin(rtb_ixj);
      tmp_0[7] = 0.0F;
      tmp_0[8] = (real32_T)cos(rtb_ixj);

      /* Sum: '<S408>/Sum1' incorporates:
       *  Product: '<S409>/Product1'
       *  Product: '<S410>/(Rh+h)cos(phi)*cos(lamb)=Xe'
       *  Product: '<S410>/(Rh+h)cos(phi)*sin(lamb)=Ye'
       *  Sum: '<S410>/Sum2'
       *  Trigonometry: '<S410>/cos(lamb)'
       *  Trigonometry: '<S410>/sin(lamb)'
       */
      rtb_RhhcosphicoslambXe_cj = rtb_Sum1 * rtb_jxi * (real32_T)cos
        (rtb_Deg2R1_bc) - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
      rtb_Deg2R1_bc = rtb_Sum1 * rtb_jxi * (real32_T)sin(rtb_Deg2R1_bc) -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
      rtb_Sum1 = rtb_RhhcosphisinlambYe_h -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

      /* Product: '<S409>/Product1' incorporates:
       *  Gain: '<S408>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp[i] = tmp_0[i + 6] * rtb_Sum1 + (tmp_0[i + 3] * rtb_Deg2R1_bc +
          tmp_0[i] * rtb_RhhcosphicoslambXe_cj);
      }

      /* Reshape: '<S408>/Reshape1' incorporates:
       *  Gain: '<S408>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Reshape1[i] = 0.0F;
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Reshape1[i] +=
          AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Reshape1[i] +=
          AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Reshape1[i] +=
          AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2];
      }

      /* End of Reshape: '<S408>/Reshape1' */
    }

    /* End of Outputs for SubSystem: '<S150>/Compute Mobile Location' */

    /* MATLAB Function: '<S150>/Zero out Z1' incorporates:
     *  Gain: '<S409>/Deg2R'
     */
    AUAV3_WITH_SLUGS_ZerooutZ1(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Reshape1,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_g);

    /* Sum: '<S150>/Subtract' */
    rtb_u11[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_g.P[0] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
    rtb_u11[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_g.P[1] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
    rtb_u11[2] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_g.P[2] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

    /* DiscreteIntegrator: '<S405>/Discrete-Time Integrator' */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_IC_LOADI != 0) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[0] =
        rtb_u11[0];
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[1] =
        rtb_u11[1];
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[2] =
        rtb_u11[2];
    }

    /* Sum: '<S405>/Sum' incorporates:
     *  DiscreteIntegrator: '<S405>/Discrete-Time Integrator'
     */
    rtb_Sum1_ao[0] = rtb_u11[0] -
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[0];
    rtb_Sum1_ao[1] = rtb_u11[1] -
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[1];
    rtb_Sum1_ao[2] = rtb_u11[2] -
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* Sum: '<S405>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S405>/Discrete-Time Integrator'
     *  Gain: '<S405>/Gain1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = 3.0F * rtb_Sum1_ao[0] +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = 3.0F * rtb_Sum1_ao[1] +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] = 3.0F * rtb_Sum1_ao[2] +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* MATLAB Function: '<S414>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_go);

    /* MATLAB Function: '<S415>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_go.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_j);

    /* S-Function (MCHP_C_function_Call): '<S415>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_pl = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_j.zpVal
      );

    /* SignalConversion: '<S418>/Numerical Unity' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_pl;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge1 = mlMidLevelCommands.uCommand;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 0U;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 = 0U;

    /* Update for DiscreteIntegrator: '<S405>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S150>/RTB'
     *  Constant: '<S150>/RTB1'
     *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
     *  Inport: '<S150>/MidLvl h_c'
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_IC_LOADI = 0U;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[0] += 0.01F *
      rtb_Sum1_ao[0];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[1] += 0.01F *
      rtb_Sum1_ao[1];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_DSTATE[2] += 0.01F *
      rtb_Sum1_ao[2];

    /* End of Outputs for SubSystem: '<S142>/RTB//Follow Mobile Navigation' */
  } else if (rtb_DataTypeConversion2_j == 3) {
    /* Outputs for IfAction SubSystem: '<S142>/Normal WP  Navigation' incorporates:
     *  ActionPort: '<S149>/Action Port'
     */
    /* Product: '<S149>/Product' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    rtb_kxi = mlParamInterface.param[18] *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116;

    /* Outputs for Enabled SubSystem: '<S149>/On WP Enable' incorporates:
     *  EnablePort: '<S266>/Enable'
     */
    /* Gain: '<S367>/Deg2R' */
    rtb_Deg2R_idx_0 = 0.0174532924F * rtb_Sum_b;
    rtb_Deg2R_idx_1 = 0.0174532924F * rtb_ixj;

    /* S-Function (MCHP_C_function_Call): '<S345>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)1U)
          , &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated511[0]
          );

    /* Gain: '<S368>/Deg2R' incorporates:
     *  Constant: '<S343>/Constant'
     */
    rtb_RhhcosphisinlambYe_h = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated511[0];

    /* Trigonometry: '<S368>/sin(phi)' */
    rtb_kxj = (real32_T)sin(rtb_RhhcosphisinlambYe_h);

    /* Sum: '<S368>/Sum1' incorporates:
     *  Constant: '<S368>/const'
     *  Product: '<S368>/Product1'
     *  Product: '<S368>/sin(phi)^2'
     */
    rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

    /* Fcn: '<S368>/f' */
    if (rtb_Sum1 < 0.0F) {
      rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
    } else {
      rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
    }

    /* End of Fcn: '<S368>/f' */

    /* Product: '<S368>/Rh' incorporates:
     *  Constant: '<S368>/Re=equatorial radius'
     */
    rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

    /* Sum: '<S368>/Sum2' */
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated511[2]
      + rtb_Sum1;

    /* Trigonometry: '<S368>/cos(phi)' */
    rtb_RhhcosphisinlambYe_h = (real32_T)cos(rtb_RhhcosphisinlambYe_h);

    /* Gain: '<S368>/Deg2R1' */
    rtb_Deg2R1_bc = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated511[1];

    /* Product: '<S368>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S368>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe_cj = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)
      cos(rtb_Deg2R1_bc);

    /* Product: '<S368>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S368>/sin(lamb)'
     */
    rtb_jxi = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)sin(rtb_Deg2R1_bc);

    /* Product: '<S368>/Ze' incorporates:
     *  Product: '<S368>/Rh(1-e^2)'
     *  Sum: '<S368>/Sum4'
     */
    rtb_kxj *= 0.993305743F * rtb_Sum1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated511[2];

    /* SignalConversion: '<S367>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S370>/11'
     *  Fcn: '<S370>/12'
     *  Fcn: '<S370>/13'
     *  Fcn: '<S370>/21'
     *  Fcn: '<S370>/22'
     *  Fcn: '<S370>/23'
     *  Fcn: '<S370>/31'
     *  Fcn: '<S370>/32'
     *  Fcn: '<S370>/33'
     */
    tmp_0[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_0[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    tmp_0[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_0[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_0[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_0[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_0[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_0[7] = 0.0F;
    tmp_0[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Sum: '<S365>/Sum1' incorporates:
     *  Product: '<S367>/Product1'
     */
    rtb_RhhcosphicoslambXe_cj -=
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
    rtb_Deg2R1_bc = rtb_jxi -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
    rtb_Sum1 = rtb_kxj - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

    /* Product: '<S367>/Product1' incorporates:
     *  Gain: '<S365>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = tmp_0[i + 6] * rtb_Sum1 + (tmp_0[i + 3] * rtb_Deg2R1_bc + tmp_0[i]
        * rtb_RhhcosphicoslambXe_cj);
    }

    /* Gain: '<S365>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[i] = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[i] +=
        AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2];
    }

    /* Gain: '<S384>/Deg2R' */
    rtb_Deg2R_idx_0 = 0.0174532924F * rtb_Sum_b;
    rtb_Deg2R_idx_1 = 0.0174532924F * rtb_ixj;

    /* S-Function (MCHP_C_function_Call): '<S346>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)2U)
          , &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_n[0]
          );

    /* Gain: '<S385>/Deg2R' incorporates:
     *  Constant: '<S343>/Constant1'
     */
    rtb_RhhcosphisinlambYe_h = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_n[0];

    /* Trigonometry: '<S385>/sin(phi)' */
    rtb_kxj = (real32_T)sin(rtb_RhhcosphisinlambYe_h);

    /* Sum: '<S385>/Sum1' incorporates:
     *  Constant: '<S385>/const'
     *  Product: '<S385>/Product1'
     *  Product: '<S385>/sin(phi)^2'
     */
    rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

    /* Fcn: '<S385>/f' */
    if (rtb_Sum1 < 0.0F) {
      rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
    } else {
      rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
    }

    /* End of Fcn: '<S385>/f' */

    /* Product: '<S385>/Rh' incorporates:
     *  Constant: '<S385>/Re=equatorial radius'
     */
    rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

    /* Sum: '<S385>/Sum2' */
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_n[2]
      + rtb_Sum1;

    /* Trigonometry: '<S385>/cos(phi)' */
    rtb_RhhcosphisinlambYe_h = (real32_T)cos(rtb_RhhcosphisinlambYe_h);

    /* Gain: '<S385>/Deg2R1' */
    rtb_Deg2R1_bc = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_n[1];

    /* Product: '<S385>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S385>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe_cj = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)
      cos(rtb_Deg2R1_bc);

    /* Product: '<S385>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S385>/sin(lamb)'
     */
    rtb_jxi = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)sin(rtb_Deg2R1_bc);

    /* Product: '<S385>/Ze' incorporates:
     *  Product: '<S385>/Rh(1-e^2)'
     *  Sum: '<S385>/Sum4'
     */
    rtb_kxj *= 0.993305743F * rtb_Sum1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_n[2];

    /* SignalConversion: '<S384>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S387>/11'
     *  Fcn: '<S387>/12'
     *  Fcn: '<S387>/13'
     *  Fcn: '<S387>/21'
     *  Fcn: '<S387>/22'
     *  Fcn: '<S387>/23'
     *  Fcn: '<S387>/31'
     *  Fcn: '<S387>/32'
     *  Fcn: '<S387>/33'
     */
    tmp_1[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_1[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    tmp_1[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_1[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_1[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_1[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_1[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_1[7] = 0.0F;
    tmp_1[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Sum: '<S382>/Sum1' incorporates:
     *  Product: '<S384>/Product1'
     */
    rtb_RhhcosphicoslambXe_cj -=
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
    rtb_Deg2R1_bc = rtb_jxi -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
    rtb_Sum1 = rtb_kxj - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

    /* Product: '<S384>/Product1' incorporates:
     *  Gain: '<S382>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = tmp_1[i + 6] * rtb_Sum1 + (tmp_1[i + 3] * rtb_Deg2R1_bc + tmp_1[i]
        * rtb_RhhcosphicoslambXe_cj);
    }

    /* Gain: '<S382>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_MathFunction_0[i] = AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] *
        tmp[2] + (AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1] +
                  AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0]);
    }

    /* Sum: '<S344>/Add' incorporates:
     *  Gain: '<S382>/UEN 2 NEU'
     */
    rtb_u11[0] = rtb_MathFunction_0[0] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[0];
    rtb_u11[1] = rtb_MathFunction_0[1] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[1];
    rtb_u11[2] = rtb_MathFunction_0[2] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[2];

    /* MATLAB Function: '<S351>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_h5);

    /* MATLAB Function: '<S352>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_h5.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_b);

    /* S-Function (MCHP_C_function_Call): '<S352>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_c = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_b.zpVal
      );

    /* Saturate: '<S347>/Zero Bound' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_c <= 0.001F) {
      rtb_Sum1 = 0.001F;
    } else {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_c;
    }

    /* End of Saturate: '<S347>/Zero Bound' */
    rtb_u11[0] /= rtb_Sum1;
    rtb_u11[1] /= rtb_Sum1;

    /* Product: '<S347>/Divide' */
    rtb_Sum1 = rtb_u11[2] / rtb_Sum1;
    rtb_u11[2] = rtb_Sum1;

    /* MATLAB Function: '<S344>/Select N  Terms' incorporates:
     *  Product: '<S347>/Divide'
     */
    AUAV3_WITH_SL_SelectNTerms(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms_g);

    /* MATLAB Function: '<S358>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms_g.N,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_gn);

    /* MATLAB Function: '<S359>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_gn.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_dc);

    /* S-Function (MCHP_C_function_Call): '<S359>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_e = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_dc.zpVal
      );

    /* Sum: '<S266>/Subtract' incorporates:
     *  Constant: '<S266>/Constant5'
     *  Product: '<S266>/Product'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[0] -= rtb_kxi * rtb_u11[0] * 2.0F;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[1] -= rtb_kxi * rtb_u11[1] * 2.0F;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1[2] -= rtb_kxi * rtb_Sum1 * 2.0F;

    /* End of Outputs for SubSystem: '<S149>/On WP Enable' */

    /* MATLAB Function: '<S149>/Zero out Z2' */
    AUAV3_WITH_SLUGS_ZerooutZ1(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0L2IPT1,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_o);

    /* Sum: '<S149>/Add' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_o.P[0] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_o.P[1] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_o.P[2] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S398>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_l);

    /* MATLAB Function: '<S399>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_l.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_px);

    /* S-Function (MCHP_C_function_Call): '<S399>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_of = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_px.zpVal
      );

    /* MATLAB Function: '<S149>/Embedded MATLAB Function' incorporates:
     *  Gain: '<S149>/Gain'
     *  RelationalOperator: '<S149>/Relational Operator'
     */
    /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Embedded MATLAB Function': '<S264>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    /* '<S264>:1:10' if isempty(persistentDidReachIP) */
    /*  this is just turned on for one sample  */
    /*  inmediatley upon enabling the navigation. */
    /*  Reset the flag the the IP was reached */
    /* '<S264>:1:18' if wpEnable */
    if (rtb_Compare_n != 0) {
      /* '<S264>:1:19' persistentDidReachIP = uint8(0); */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.persistentDidReachIP = 0U;
    }

    /*  Once the IP is reached the the persistent variable */
    /*  preserves the values until reset */
    /* '<S264>:1:24' if didReachIP */
    if (0.4F * rtb_kxi >
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_of) {
      /* '<S264>:1:25' persistentDidReachIP = uint8(1); */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.persistentDidReachIP = 1U;
    }

    /* InitialCondition: '<S149>/IC' incorporates:
     *  MATLAB Function: '<S149>/Embedded MATLAB Function'
     */
    /* '<S264>:1:28' reached = persistentDidReachIP; */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j = false;
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC = 0U;
    } else {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.persistentDidReachIP;
    }

    /* End of InitialCondition: '<S149>/IC' */

    /* MATLAB Function: '<S149>/computeCurrentWP' incorporates:
     *  DataStoreRead: '<Root>/Get Max WP'
     *  Delay: '<S149>/Integer Delay'
     */
    /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/computeCurrentWP': '<S269>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    /* '<S269>:1:10' if isempty(fromWp) */
    /* '<S269>:1:15' if WPSwitch */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bu != 0) {
      /* '<S269>:1:16' fromWp = toWp; */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.fromWp =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp;

      /* '<S269>:1:17' toWp = toWp +1; */
      rtb_u2deg = AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp + 1U;
      if (rtb_u2deg > 255U) {
        rtb_u2deg = 255U;
      }

      AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp = (uint8_T)rtb_u2deg;

      /* '<S269>:1:18' if (toWp > maxWp) */
      if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp > mlWpValues.wpCount) {
        /* '<S269>:1:19' toWp = uint8(1); */
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp = 1U;
      }
    }

    /*  this is jturned on long as we have not reached IP  */
    /* '<S269>:1:25' if ~IPReached */
    if (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC != 0)) {
      /* '<S269>:1:26' fromWp = uint8(1); */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.fromWp = 1U;

      /* '<S269>:1:27' toWp = uint8(2); */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp = 2U;
    }

    /* '<S269>:1:32' WP0 = fromWp; */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.fromWp;

    /* '<S269>:1:33' WP1 = toWp; */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp;

    /* End of MATLAB Function: '<S149>/computeCurrentWP' */

    /* Outputs for Enabled SubSystem: '<S149>/Get Frenet' incorporates:
     *  EnablePort: '<S265>/Enable'
     */
    /* Gain: '<S312>/Deg2R' */
    rtb_Deg2R_idx_0 = 0.0174532924F * rtb_Sum_b;
    rtb_Deg2R_idx_1 = 0.0174532924F * rtb_ixj;

    /* S-Function (MCHP_C_function_Call): '<S290>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0
          , &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_c[0]
          );

    /* Gain: '<S313>/Deg2R' */
    rtb_RhhcosphisinlambYe_h = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_c[0];

    /* Trigonometry: '<S313>/sin(phi)' */
    rtb_kxj = (real32_T)sin(rtb_RhhcosphisinlambYe_h);

    /* Sum: '<S313>/Sum1' incorporates:
     *  Constant: '<S313>/const'
     *  Product: '<S313>/Product1'
     *  Product: '<S313>/sin(phi)^2'
     */
    rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

    /* Fcn: '<S313>/f' */
    if (rtb_Sum1 < 0.0F) {
      rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
    } else {
      rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
    }

    /* End of Fcn: '<S313>/f' */

    /* Product: '<S313>/Rh' incorporates:
     *  Constant: '<S313>/Re=equatorial radius'
     */
    rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

    /* Sum: '<S313>/Sum2' */
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_c[2]
      + rtb_Sum1;

    /* Trigonometry: '<S313>/cos(phi)' */
    rtb_RhhcosphisinlambYe_h = (real32_T)cos(rtb_RhhcosphisinlambYe_h);

    /* Gain: '<S313>/Deg2R1' */
    rtb_Deg2R1_bc = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_c[1];

    /* Product: '<S313>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S313>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe_cj = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)
      cos(rtb_Deg2R1_bc);

    /* Product: '<S313>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S313>/sin(lamb)'
     */
    rtb_jxi = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)sin(rtb_Deg2R1_bc);

    /* Product: '<S313>/Ze' incorporates:
     *  Product: '<S313>/Rh(1-e^2)'
     *  Sum: '<S313>/Sum4'
     */
    rtb_kxj *= 0.993305743F * rtb_Sum1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_c[2];

    /* SignalConversion: '<S312>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S315>/11'
     *  Fcn: '<S315>/12'
     *  Fcn: '<S315>/13'
     *  Fcn: '<S315>/21'
     *  Fcn: '<S315>/22'
     *  Fcn: '<S315>/23'
     *  Fcn: '<S315>/31'
     *  Fcn: '<S315>/32'
     *  Fcn: '<S315>/33'
     */
    tmp_2[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_2[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    tmp_2[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_2[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_2[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_2[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_2[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_2[7] = 0.0F;
    tmp_2[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Sum: '<S310>/Sum1' incorporates:
     *  Product: '<S312>/Product1'
     */
    rtb_RhhcosphicoslambXe_cj -=
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
    rtb_Deg2R1_bc = rtb_jxi -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
    rtb_Sum1 = rtb_kxj - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

    /* Product: '<S312>/Product1' incorporates:
     *  Gain: '<S310>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = tmp_2[i + 6] * rtb_Sum1 + (tmp_2[i + 3] * rtb_Deg2R1_bc + tmp_2[i]
        * rtb_RhhcosphicoslambXe_cj);
    }

    /* Gain: '<S310>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_u11[i] = AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2] +
        (AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1] +
         AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0]);
    }

    /* Gain: '<S329>/Deg2R' */
    rtb_Deg2R_idx_0 = 0.0174532924F * rtb_Sum_b;
    rtb_Deg2R_idx_1 = 0.0174532924F * rtb_ixj;

    /* S-Function (MCHP_C_function_Call): '<S291>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP1
          , &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_p[0]
          );

    /* Gain: '<S330>/Deg2R' */
    rtb_RhhcosphisinlambYe_h = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_p[0];

    /* Trigonometry: '<S330>/sin(phi)' */
    rtb_kxj = (real32_T)sin(rtb_RhhcosphisinlambYe_h);

    /* Sum: '<S330>/Sum1' incorporates:
     *  Constant: '<S330>/const'
     *  Product: '<S330>/Product1'
     *  Product: '<S330>/sin(phi)^2'
     */
    rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

    /* Fcn: '<S330>/f' */
    if (rtb_Sum1 < 0.0F) {
      rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
    } else {
      rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
    }

    /* End of Fcn: '<S330>/f' */

    /* Product: '<S330>/Rh' incorporates:
     *  Constant: '<S330>/Re=equatorial radius'
     */
    rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

    /* Sum: '<S330>/Sum2' */
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_p[2]
      + rtb_Sum1;

    /* Trigonometry: '<S330>/cos(phi)' */
    rtb_RhhcosphisinlambYe_h = (real32_T)cos(rtb_RhhcosphisinlambYe_h);

    /* Gain: '<S330>/Deg2R1' */
    rtb_Deg2R1_bc = 0.0174532924F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_p[1];

    /* Product: '<S330>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S330>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe_cj = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)
      cos(rtb_Deg2R1_bc);

    /* Product: '<S330>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S330>/sin(lamb)'
     */
    rtb_jxi = rtb_jxi * rtb_RhhcosphisinlambYe_h * (real32_T)sin(rtb_Deg2R1_bc);

    /* Product: '<S330>/Ze' incorporates:
     *  Product: '<S330>/Rh(1-e^2)'
     *  Sum: '<S330>/Sum4'
     */
    rtb_kxj *= 0.993305743F * rtb_Sum1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetWPCoordnavsupportcupdated5_p[2];

    /* SignalConversion: '<S329>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S332>/11'
     *  Fcn: '<S332>/12'
     *  Fcn: '<S332>/13'
     *  Fcn: '<S332>/21'
     *  Fcn: '<S332>/22'
     *  Fcn: '<S332>/23'
     *  Fcn: '<S332>/31'
     *  Fcn: '<S332>/32'
     *  Fcn: '<S332>/33'
     */
    tmp_3[0] = (real32_T)cos(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_3[1] = -(real32_T)sin(rtb_Deg2R_idx_0);
    tmp_3[2] = -(real32_T)sin(rtb_Deg2R_idx_1) * (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_3[3] = (real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx_1);
    tmp_3[4] = (real32_T)cos(rtb_Deg2R_idx_0);
    tmp_3[5] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_3[6] = (real32_T)sin(rtb_Deg2R_idx_1);
    tmp_3[7] = 0.0F;
    tmp_3[8] = (real32_T)cos(rtb_Deg2R_idx_1);

    /* Sum: '<S327>/Sum1' incorporates:
     *  Product: '<S329>/Product1'
     */
    rtb_RhhcosphicoslambXe_cj -=
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
    rtb_Deg2R1_bc = rtb_jxi -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
    rtb_Sum1 = rtb_kxj - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

    /* Product: '<S329>/Product1' incorporates:
     *  Gain: '<S327>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = tmp_3[i + 6] * rtb_Sum1 + (tmp_3[i + 3] * rtb_Deg2R1_bc + tmp_3[i]
        * rtb_RhhcosphicoslambXe_cj);
    }

    /* Gain: '<S327>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Sum1_ao[i] = AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2]
        + (AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1] +
           AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0]);
    }

    rtb_u11[0] = rtb_Sum1_ao[0] - rtb_u11[0];
    rtb_u11[1] = rtb_Sum1_ao[1] - rtb_u11[1];
    rtb_u11[2] = rtb_Sum1_ao[2] - rtb_u11[2];

    /* MATLAB Function: '<S296>/Embedded MATLAB Function' incorporates:
     *  Sum: '<S286>/Add'
     */
    A_EmbeddedMATLABFunction_g(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_l2);

    /* MATLAB Function: '<S297>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_l2.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_o);

    /* S-Function (MCHP_C_function_Call): '<S297>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_kw = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_o.zpVal
      );

    /* Saturate: '<S292>/Zero Bound' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_kw <= 0.001F)
    {
      rtb_Sum1 = 0.001F;
    } else {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_kw;
    }

    /* End of Saturate: '<S292>/Zero Bound' */
    rtb_u11[0] /= rtb_Sum1;
    rtb_u11[1] /= rtb_Sum1;
    rtb_u11[2] /= rtb_Sum1;

    /* MATLAB Function: '<S286>/Select N  Terms' incorporates:
     *  Product: '<S292>/Divide'
     */
    AUAV3_WITH_SL_SelectNTerms(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms);

    /* MATLAB Function: '<S303>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms.N,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fl);

    /* MATLAB Function: '<S304>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fl.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_i);

    /* S-Function (MCHP_C_function_Call): '<S304>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_b = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_i.zpVal
      );

    /* Saturate: '<S293>/Zero Bound' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_b <= 0.001F) {
      rtb_Sum1 = 0.001F;
    } else {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_b;
    }

    /* End of Saturate: '<S293>/Zero Bound' */

    /* Product: '<S293>/Divide' */
    rtb_Product5[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms.N[0] /
      rtb_Sum1;
    rtb_Product5[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms.N[1] /
      rtb_Sum1;
    rtb_Product5[2] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_SelectNTerms.N[2] /
      rtb_Sum1;

    /* Delay: '<S265>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge1 =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_di;

    /* MATLAB Function: '<S265>/Zero out Z1' */
    AUAV3_WITH_SLUGS_ZerooutZ1(rtb_Product5,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_p);

    /* MATLAB Function: '<S265>/Zero out Z2' */
    AUAV3_WITH_SLUGS_ZerooutZ1(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_c);

    /* MATLAB Function: '<S265>/Zero out Z3' */
    AUAV3_WITH_SLUGS_ZerooutZ1(rtb_Sum1_ao,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3);

    /* Product: '<S265>/Product1' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Product1 = mlParamInterface.param[19] *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116;

    /* Update for Delay: '<S265>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_di = rtb_Sum1_ao[2];

    /* End of Outputs for SubSystem: '<S149>/Get Frenet' */

    /* Sum: '<S263>/Subtract' */
    rtb_u11[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[0];
    rtb_u11[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[1];
    rtb_u11[2] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[2];

    /* MATLAB Function: '<S271>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_a(rtb_u11,
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_p.P,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_o);

    /* S-Function (MCHP_C_function_Call): '<S272>/myAbs() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_gb = myAbs(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_o.xDoty
      );

    /* Sum: '<S270>/Subtract' */
    rtb_Sum1_ao[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[0] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
    rtb_Sum1_ao[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[1] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
    rtb_Sum1_ao[2] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ3.P[2] -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S278>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_a(rtb_Sum1_ao,
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_c.P,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_a);

    /* Switch: '<S277>/Switch3' incorporates:
     *  RelationalOperator: '<S277>/Relational Operator2'
     */
    if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_a.xDoty ==
         AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_a.xDoty) > 0)
    {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_d =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_a.xDoty;
    }

    /* End of Switch: '<S277>/Switch3' */

    /* Product: '<S263>/Divide' incorporates:
     *  Constant: '<S263>/Constant4'
     */
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_gb /
      0.577350259F;

    /* Switch: '<S276>/Switch' incorporates:
     *  Product: '<S263>/Divide1'
     *  RelationalOperator: '<S276>/Relational Operator'
     */
    if (rtb_kxi < rtb_Sum1) {
      rtb_Sum1 = rtb_kxi;
    }

    /* End of Switch: '<S276>/Switch' */

    /* MATLAB Function: '<S273>/negprotect' incorporates:
     *  Product: '<S263>/Product'
     *  Product: '<S263>/Product1'
     *  Sum: '<S263>/Add'
     */
    AUAV3_WITH_SL_negprotect_d(rtb_kxi * rtb_kxi -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_gb *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_gb,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_l);

    /* S-Function (MCHP_C_function_Call): '<S273>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_k = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_l.zpVal
      );

    /* Switch: '<S149>/Switch' incorporates:
     *  Product: '<S263>/Product2'
     *  Sum: '<S263>/Add3'
     *  Switch: '<S149>/Switch1'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC > 0) {
      /* Switch: '<S275>/Switch' incorporates:
       *  RelationalOperator: '<S263>/Relational Operator'
       *  RelationalOperator: '<S275>/Relational Operator'
       *  Switch: '<S263>/Switch1'
       */
      if ((!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_gb >
             rtb_kxi)) && (!(rtb_Sum1 >
                             AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_k)))
      {
        rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_k;
      }

      /* End of Switch: '<S275>/Switch' */

      /* Sum: '<S263>/Add1' */
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_d -
        rtb_Sum1;

      /* Switch: '<S274>/Switch' incorporates:
       *  Constant: '<S263>/Constant1'
       *  RelationalOperator: '<S274>/Relational Operator'
       */
      if (!(rtb_Sum1 > 0.0F)) {
        rtb_Sum1 = 0.0F;
      }

      /* End of Switch: '<S274>/Switch' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = (0.0F - rtb_Sum1 *
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_c.P[0]) - rtb_u11[0];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = (0.0F - rtb_Sum1 *
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_c.P[1]) - rtb_u11[1];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] = (0.0F - rtb_Sum1 *
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_c.P[2]) - rtb_u11[2];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2 =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_d;
    } else {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2 =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_of;
    }

    /* End of Switch: '<S149>/Switch' */

    /* Update for Delay: '<S149>/Integer Delay' incorporates:
     *  RelationalOperator: '<S263>/Switch Distance Less than?'
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bu = (uint8_T)
      (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_d <
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Product1);

    /* End of Outputs for SubSystem: '<S142>/Normal WP  Navigation' */
  } else if (rtb_DataTypeConversion2_j == 10) {
    /* Outputs for IfAction SubSystem: '<S142>/Line Segment' incorporates:
     *  ActionPort: '<S147>/Action Port'
     */
    /* Gain: '<S147>/Gain' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] =
      -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] =
      -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] =
      -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

    /* Gain: '<S147>/Gain1' */
    rtb_u11[0] = -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
    rtb_u11[1] = -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
    rtb_u11[2] = -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S257>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_g(rtb_u11,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_om);

    /* MATLAB Function: '<S258>/negprotect' */
    AUAV3_WITH_SL_negprotect_d
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_om.xDoty,
       &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_a);

    /* S-Function (MCHP_C_function_Call): '<S258>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_mg = mySqrt(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_a.zpVal
      );

    /* SignalConversion: '<S261>/Numerical Unity' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_mg;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge1 = mlMidLevelCommands.uCommand;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 0U;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 = 0U;

    /* End of Outputs for SubSystem: '<S142>/Line Segment' */
  } else {
    if (rtb_DataTypeConversion2_j == 9) {
      /* Outputs for IfAction SubSystem: '<S142>/Circle Navigation' incorporates:
       *  ActionPort: '<S143>/Action Port'
       */
      rtb_Sum_b *= 0.0174532924F;

      /* Gain: '<S168>/Deg2R' */
      rtb_ixj *= 0.0174532924F;

      /* Gain: '<S169>/Deg2R' incorporates:
       *  DataStoreRead: '<S34>/Get ISR Location'
       */
      rtb_jxi = 0.0174532924F * mlISR.latitude;

      /* Trigonometry: '<S169>/sin(phi)' */
      rtb_kxj = (real32_T)sin(rtb_jxi);

      /* Sum: '<S169>/Sum1' incorporates:
       *  Constant: '<S169>/const'
       *  Product: '<S169>/Product1'
       *  Product: '<S169>/sin(phi)^2'
       */
      rtb_Sum1 = 1.0F - rtb_kxj * rtb_kxj * 0.00669425726F;

      /* Fcn: '<S169>/f' */
      if (rtb_Sum1 < 0.0F) {
        rtb_Sum1 = -(real32_T)sqrt(-rtb_Sum1);
      } else {
        rtb_Sum1 = (real32_T)sqrt(rtb_Sum1);
      }

      /* End of Fcn: '<S169>/f' */

      /* Product: '<S169>/Rh' incorporates:
       *  Constant: '<S169>/Re=equatorial radius'
       */
      rtb_Sum1 = 6.378137E+6F / rtb_Sum1;

      /* Sum: '<S169>/Sum2' incorporates:
       *  DataStoreRead: '<S34>/Get ISR Location'
       */
      rtb_RhhcosphisinlambYe_h = mlISR.height + rtb_Sum1;

      /* Trigonometry: '<S169>/cos(phi)' */
      rtb_jxi = (real32_T)cos(rtb_jxi);

      /* Gain: '<S169>/Deg2R1' incorporates:
       *  DataStoreRead: '<S34>/Get ISR Location'
       */
      rtb_Deg2R1_bc = 0.0174532924F * mlISR.longitude;

      /* Product: '<S169>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
       *  Trigonometry: '<S169>/cos(lamb)'
       */
      rtb_RhhcosphicoslambXe_cj = rtb_RhhcosphisinlambYe_h * rtb_jxi * (real32_T)
        cos(rtb_Deg2R1_bc);

      /* Product: '<S169>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
       *  Trigonometry: '<S169>/sin(lamb)'
       */
      rtb_RhhcosphisinlambYe_h = rtb_RhhcosphisinlambYe_h * rtb_jxi * (real32_T)
        sin(rtb_Deg2R1_bc);

      /* Product: '<S169>/Ze' incorporates:
       *  DataStoreRead: '<S34>/Get ISR Location'
       *  Product: '<S169>/Rh(1-e^2)'
       *  Sum: '<S169>/Sum4'
       */
      rtb_kxj *= 0.993305743F * rtb_Sum1 + mlISR.height;

      /* SignalConversion: '<S168>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S171>/11'
       *  Fcn: '<S171>/12'
       *  Fcn: '<S171>/13'
       *  Fcn: '<S171>/21'
       *  Fcn: '<S171>/22'
       *  Fcn: '<S171>/23'
       *  Fcn: '<S171>/31'
       *  Fcn: '<S171>/32'
       *  Fcn: '<S171>/33'
       */
      tmp_0[0] = (real32_T)cos(rtb_Sum_b) * (real32_T)cos(rtb_ixj);
      tmp_0[1] = -(real32_T)sin(rtb_Sum_b);
      tmp_0[2] = -(real32_T)sin(rtb_ixj) * (real32_T)cos(rtb_Sum_b);
      tmp_0[3] = (real32_T)sin(rtb_Sum_b) * (real32_T)cos(rtb_ixj);
      tmp_0[4] = (real32_T)cos(rtb_Sum_b);
      tmp_0[5] = -(real32_T)sin(rtb_Sum_b) * (real32_T)sin(rtb_ixj);
      tmp_0[6] = (real32_T)sin(rtb_ixj);
      tmp_0[7] = 0.0F;
      tmp_0[8] = (real32_T)cos(rtb_ixj);

      /* Sum: '<S166>/Sum1' incorporates:
       *  Product: '<S168>/Product1'
       */
      rtb_RhhcosphicoslambXe_cj -=
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[0];
      rtb_Sum1 = rtb_RhhcosphisinlambYe_h -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[1];
      rtb_Deg2R1_bc = rtb_kxj -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion1[2];

      /* Product: '<S168>/Product1' incorporates:
       *  Gain: '<S166>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp[i] = tmp_0[i + 6] * rtb_Deg2R1_bc + (tmp_0[i + 3] * rtb_Sum1 +
          tmp_0[i] * rtb_RhhcosphicoslambXe_cj);
      }

      /* Gain: '<S166>/UEN 2 NEU' */
      for (i = 0; i < 3; i++) {
        rtb_u11[i] = AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 6] * tmp[2] +
          (AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i + 3] * tmp[1] +
           AUAV3_WITH_SLUGS_SENSOR__ConstP.pooled55[i] * tmp[0]);
      }

      /* Delay: '<S160>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge1 =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_d;

      /* MATLAB Function: '<S160>/Zero out Z2' */
      AUAV3_WITH_SLUGS_ZerooutZ1(rtb_u11,
        &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih);

      /* Sum: '<S143>/Sum1' */
      rtb_Sum1_ao[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0] -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[0];
      rtb_Sum1_ao[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1] -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[1];
      rtb_Sum1_ao[2] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2] -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[2];

      /* MATLAB Function: '<S212>/Embedded MATLAB Function' */
      A_EmbeddedMATLABFunction_g(rtb_Sum1_ao,
        &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_oc);

      /* MATLAB Function: '<S213>/negprotect' */
      AUAV3_WITH_SL_negprotect_d
        (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_oc.xDoty,
         &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_bj);

      /* S-Function (MCHP_C_function_Call): '<S213>/mySqrt() apUtils.c [updated 5.1.16]' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m = mySqrt(
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_bj.zpVal
        );

      /* Product: '<S143>/Product' incorporates:
       *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
       */
      rtb_Deg2R1_bc = mlParamInterface.param[18] *
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116;

      /* Sum: '<S143>/Sum2' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum2 = rtb_kxi - rtb_Deg2R1_bc;

      /* S-Function (MCHP_C_function_Call): '<S165>/myAbs() apUtils.c [updated 5.1.16]' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_n = myAbs(
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum2
        );

      /* If: '<S143>/If' incorporates:
       *  Constant: '<S161>/RTB1'
       *  Constant: '<S162>/RTB1'
       *  Constant: '<S163>/RTB1'
       *  Product: '<S162>/Product6'
       *  Product: '<S162>/Product7'
       *  Sum: '<S143>/Sum'
       *  Sum: '<S162>/Subtract3'
       *  Sum: '<S162>/Subtract6'
       */
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m >
          rtb_Deg2R1_bc + rtb_kxi) {
        /* Outputs for IfAction SubSystem: '<S143>/No intersection,  Navigate to ISR' incorporates:
         *  ActionPort: '<S163>/Action Port'
         */
        /* Sum: '<S163>/Subtract' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[0] -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[1] -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[2] -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 0U;

        /* End of Outputs for SubSystem: '<S143>/No intersection,  Navigate to ISR' */
      } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m <
                 AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_n) {
        /* Outputs for IfAction SubSystem: '<S143>/Inside the Circle,  Keep Straight until  intersection' incorporates:
         *  ActionPort: '<S161>/Action Port'
         */
        /* Sum: '<S161>/Subtract' incorporates:
         *  MATLAB Function: '<S161>/Compute Head of Circle'
         */
        /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle': '<S173>:1' */
        /* % Compute the top coordinate of the circle */
        /*  using the circle's parametric equations: */
        /*   x = a + r cos (t) */
        /*   y = b + r sin (t) */
        /*  */
        /*  @ t =0 */
        /* '<S173>:1:9' topCoord = zeros(3,1); */
        /* '<S173>:1:11' topCoord(1) = ISR(1)+ R; */
        /* '<S173>:1:12' topCoord(2) = ISR(2); */
        /* '<S173>:1:13' topCoord(3) = ISR(3); */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = (real32_T)((real_T)
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[0] + rtb_kxi) -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0]);
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = (real32_T)((real_T)
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[1] -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1]);
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] = (real32_T)((real_T)
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[2] -
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2]);
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 1U;

        /* End of Outputs for SubSystem: '<S143>/Inside the Circle,  Keep Straight until  intersection' */
      } else {
        /* Outputs for IfAction SubSystem: '<S143>/Intersection. Circular Navigation' incorporates:
         *  ActionPort: '<S162>/Action Port'
         */
        /* Product: '<S162>/Product' */
        rtb_Sum1 = rtb_kxi * rtb_kxi;

        /* MATLAB Function: '<S162>/negprotect2' incorporates:
         *  Sum: '<S162>/Subtract1'
         */
        AUAV3_WITH_SLU_negprotect3
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m +
           AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect2);

        /* Product: '<S162>/Product3' incorporates:
         *  Product: '<S162>/Product1'
         *  Product: '<S162>/Product2'
         *  Sum: '<S162>/Subtract'
         */
        rtb_Deg2R1_bc = ((rtb_Sum1 - rtb_Deg2R1_bc * rtb_Deg2R1_bc) +
                         AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m
                         * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m)
          / AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect2.zpVal;

        /* MATLAB Function: '<S179>/negprotect' incorporates:
         *  Product: '<S162>/Product4'
         *  Sum: '<S162>/Subtract2'
         */
        AUAV3_WITH_SL_negprotect_d(rtb_Sum1 - rtb_Deg2R1_bc * rtb_Deg2R1_bc,
          &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_iw);

        /* S-Function (MCHP_C_function_Call): '<S179>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_f = mySqrt(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_iw.zpVal
          );

        /* MATLAB Function: '<S162>/negprotect1' */
        AUAV3_WITH_SLU_negprotect3
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_m,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect1);
        rtb_Sum1_ao[0] /= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect1.zpVal;

        /* Product: '<S162>/Product6' */
        rtb_Sum1 = rtb_Sum1_ao[1] /
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect1.zpVal;

        /* Product: '<S162>/Product5' */
        rtb_Product5[0] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_f *
          rtb_Sum1_ao[0];
        rtb_Product5[1] =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_f * rtb_Sum1;
        rtb_Sum1_ao[0] = rtb_Sum1_ao[0] * rtb_Deg2R1_bc +
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[0];

        /* Sum: '<S162>/Subtract3' incorporates:
         *  Product: '<S162>/Product7'
         */
        rtb_Sum1 = rtb_Sum1 * rtb_Deg2R1_bc +
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2_ih.P[1];

        /* MATLAB Function: '<S162>/Embedded MATLAB Function' */
        /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function': '<S176>:1' */
        /*  This block supports an embeddable subset of the MATLAB language. */
        /*  See the help menu for details.  */
        /* '<S176>:1:5' P31 = single([0 0 0]'); */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = 0.0F;
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = 0.0F;
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] = 0.0F;

        /* '<S176>:1:6' P32 = single([0 0 0]'); */
        /* '<S176>:1:8' P31(1) = P2(1) + hFact(2); */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = rtb_Sum1_ao[0] +
          rtb_Product5[1];

        /* '<S176>:1:9' P32(1) = P2(1) - hFact(2); */
        rtb_P32[0] = rtb_Sum1_ao[0] - rtb_Product5[1];

        /* '<S176>:1:11' P31(2) = P2(2) - hFact(1); */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = rtb_Sum1 - rtb_Product5[0];

        /* '<S176>:1:12' P32(2) = P2(2) + hFact(1); */
        rtb_P32[1] = rtb_Sum1 + rtb_Product5[0];
        rtb_P32[0] -= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
        rtb_P32[1] -= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
        rtb_P32[2] = 0.0F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S183>/Embedded MATLAB Function' incorporates:
         *  Sum: '<S162>/Subtract6'
         */
        A_EmbeddedMATLABFunction_a
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P, rtb_P32,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_e);

        /* MATLAB Function: '<S186>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_g(rtb_P32,
          &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_m);

        /* MATLAB Function: '<S187>/negprotect' */
        AUAV3_WITH_SL_negprotect_d
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_m.xDoty,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_mp);

        /* S-Function (MCHP_C_function_Call): '<S187>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_p = mySqrt(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_mp.zpVal
          );

        /* MATLAB Function: '<S174>/negprotect3' incorporates:
         *  Product: '<S174>/Product9'
         */
        AUAV3_WITH_SLU_negprotect3
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_p *
           AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect3);

        /* Product: '<S174>/Product8' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1 =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_e.xDoty /
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect3.zpVal;

        /* Saturate: '<S174>/[-1 1]' */
        if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1 > 1.0F) {
          /* Product: '<S174>/Product8' */
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1 = 1.0F;
        } else {
          if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1 < -1.0F) {
            /* Product: '<S174>/Product8' */
            AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1 = -1.0F;
          }
        }

        /* End of Saturate: '<S174>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S184>/myAcos() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAcosapUtilscupdated5116 = myAcos(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1
          );

        /* Sum: '<S162>/Subtract5' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] -=
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[0];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] -=
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[1];
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] -=
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S196>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_a
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P,
           AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_iq);

        /* MATLAB Function: '<S199>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_g(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c,
          &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ao);

        /* MATLAB Function: '<S200>/negprotect' */
        AUAV3_WITH_SL_negprotect_d
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ao.xDoty,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_g);

        /* S-Function (MCHP_C_function_Call): '<S200>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_o = mySqrt(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_g.zpVal
          );

        /* MATLAB Function: '<S175>/negprotect3' incorporates:
         *  Product: '<S175>/Product9'
         */
        AUAV3_WITH_SLU_negprotect3
          (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_o *
           AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116,
           &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect3_h);

        /* Product: '<S175>/Product8' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o =
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_iq.xDoty /
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect3_h.zpVal;

        /* Saturate: '<S175>/[-1 1]' */
        if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o > 1.0F) {
          /* Product: '<S175>/Product8' */
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o = 1.0F;
        } else {
          if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o < -1.0F) {
            /* Product: '<S175>/Product8' */
            AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o = -1.0F;
          }
        }

        /* End of Saturate: '<S175>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S197>/myACos() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myACosapUtilscupdated5116 = myAcos(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u1_o
          );

        /* S-Function (MCHP_C_function_Call): '<S178>/myAbs() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_g = myAbs(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myACosapUtilscupdated5116
          );

        /* S-Function (MCHP_C_function_Call): '<S177>/myAbs() apUtils.c [updated 5.1.16]' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_e = myAbs(
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAcosapUtilscupdated5116
          );

        /* Switch: '<S162>/Switch1' incorporates:
         *  RelationalOperator: '<S162>/Relational Operator'
         *  Sum: '<S162>/Subtract6'
         */
        if (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_g <=
              AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116_e)) {
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] = rtb_P32[0];
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] = rtb_P32[1];
          AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[2] = 0.0F -
            AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ2.P[2];
        }

        /* End of Switch: '<S162>/Switch1' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 2U;

        /* End of Outputs for SubSystem: '<S143>/Intersection. Circular Navigation' */
      }

      /* End of If: '<S143>/If' */

      /* Delay: '<S143>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2 =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a;
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 = 0U;

      /* Update for Delay: '<S160>/Integer Delay1' incorporates:
       *  Constant: '<S143>/RTB1'
       */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_d = rtb_u11[2];

      /* Update for Delay: '<S143>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a = rtb_kxi;

      /* End of Outputs for SubSystem: '<S142>/Circle Navigation' */
    }
  }

  /* End of If: '<S142>/Determine Overall Nav by the Nav Mode' */

  /* MATLAB Function: '<S239>/Zero out Z1' */
  AUAV3_WITH_SLUGS_ZerooutZ1(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_d);

  /* MATLAB Function: '<S244>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_g(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1_d.P,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_c);

  /* MATLAB Function: '<S245>/negprotect' */
  AUAV3_WITH_SL_negprotect_d
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_c.xDoty,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_e);

  /* S-Function (MCHP_C_function_Call): '<S245>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_h = mySqrt(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_e.zpVal
    );

  /* MATLAB Function: '<S231>/negprotect' incorporates:
   *  Product: '<S231>/Product1'
   */
  AUAV3_WITH_SLU_negprotect3
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_h *
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5);

  /* DeadZone: '<S231>/Dead Zone' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal > 0.1F) {
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal - 0.1F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal >= -0.1F) {
    rtb_jxi = 0.0F;
  } else {
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal - -0.1F;
  }

  /* End of DeadZone: '<S231>/Dead Zone' */

  /* Switch: '<S231>/Switch' incorporates:
   *  Constant: '<S231>/cos(pi//2)'
   *  Constant: '<S231>/sin(pi//2)'
   *  Constant: '<S238>/Constant'
   *  Product: '<S231>/Divide1'
   *  Product: '<S231>/Divide2'
   *  Product: '<S237>/Product'
   *  Product: '<S237>/Product1'
   *  Product: '<S240>/Product'
   *  Product: '<S240>/Product1'
   *  RelationalOperator: '<S238>/Compare'
   *  Sum: '<S237>/Subtract'
   *  Sum: '<S240>/Subtract'
   *  Switch: '<S231>/Switch2'
   */
  if ((rtb_jxi == 0.0F) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch = 1.0F;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2_p = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch =
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P[0] -
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P[1]) /
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2_p =
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[0] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P[0] +
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_c[1] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_ZerooutZ1.P[1]) * (1.0F /
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_p5.zpVal);
  }

  /* End of Switch: '<S231>/Switch' */

  /* S-Function (MCHP_C_function_Call): '<S235>/myAtan2() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtan2apUtilscupdated5116 = myAtan2(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch2_p
    );

  /* Delay: '<S142>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_al;

  /* Delay: '<S142>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_k;

  /* MATLAB Function: '<S142>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S142>/Data Type Conversion5'
   *  DataTypeConversion: '<S142>/Data Type Conversion6'
   *  Gain: '<S146>/Rad2Deg'
   */
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun1': '<S154>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S154>:1:4' y = [u1(1); u2(1); u3(1); u4(1)]; */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_m[0] = 57.2957802F *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtan2apUtilscupdated5116;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_m[1] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge2;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_m[2] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_m[3] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4;

  /* S-Function (MCHP_C_function_Call): '<S142>/getXYZ() navsupport.c [updated 5.1.16]' */
  getXYZ(
         &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_m[0]
         );

  /* MATLAB Function: '<S232>/negprotect' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
   */
  AUAV3_WITH_SLU_negprotect3(mlParamInterface.param[18],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_m);

  /* S-Function (MCHP_C_function_Call): '<S234>/myAbs() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116 = myAbs(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtan2apUtilscupdated5116
    );

  /* Switch: '<S146>/Switch' incorporates:
   *  Constant: '<S146>/Constant10'
   *  Constant: '<S255>/Constant'
   *  Gain: '<S232>/Gain'
   *  Product: '<S232>/Divide'
   *  RelationalOperator: '<S146>/Relational Operator'
   *  RelationalOperator: '<S255>/Compare'
   *  Switch: '<S236>/Switch1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAbsapUtilscupdated5116 <= 1.57079637F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116 /
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_m.zpVal * 2.0F;
  } else if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Switch < 0.0F) > 0) {
    /* Switch: '<S236>/Switch1' incorporates:
     *  Constant: '<S236>/Constant9'
     */
    rtb_Sum1 = -4.57681F;
  } else {
    /* Switch: '<S236>/Switch1' incorporates:
     *  Constant: '<S236>/Constant1'
     */
    rtb_Sum1 = 4.57681F;
  }

  /* End of Switch: '<S146>/Switch' */

  /* Product: '<S233>/Product' incorporates:
   *  Constant: '<S233>/Constant'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Product = rtb_Sum1 / 9.815F;

  /* S-Function (MCHP_C_function_Call): '<S251>/myAtan() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtanapUtilscupdated5116 = myAtan(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Product
    );

  /* Saturate: '<S233>/Bank  Limit Command' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtanapUtilscupdated5116 > 0.436332315F)
  {
    rtb_BankLimitCommand = 0.436332315F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtanapUtilscupdated5116 <
             -0.436332315F) {
    rtb_BankLimitCommand = -0.436332315F;
  } else {
    rtb_BankLimitCommand =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myAtanapUtilscupdated5116;
  }

  /* End of Saturate: '<S233>/Bank  Limit Command' */

  /* InitialCondition: '<S151>/IC3' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC3_FirstOutputTime) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC3_FirstOutputTime = false;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC3 = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC3 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity[0];
  }

  /* End of InitialCondition: '<S151>/IC3' */

  /* Update for Delay: '<S426>/Delay' */
  /* MATLAB Function 'Inner Loop/ Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun3': '<S156>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S156>:1:5' y = [u1(1); u1(2); u2(1)]; */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay_DSTATE_p = rtb_IC1;

  /* Update for Delay: '<S142>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_al =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0;

  /* Update for Delay: '<S142>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_k =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP1;

  /* End of Outputs for SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */

  /* Switch: '<S35>/Switch3' incorporates:
   *  Delay: '<S35>/Integer Delay3'
   *  RelationalOperator: '<S35>/Relational Operator2'
   */
  if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1.y[2] ==
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1.y[2]) > 0) {
    rtb_Switch3 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1.y[2];
  } else {
    rtb_Switch3 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE;
  }

  /* End of Switch: '<S35>/Switch3' */

  /* Gain: '<S671>/Unit Conversion' incorporates:
   *  DataStoreRead: '<S18>/PAR_CONFIG_ROLL_R PAR_CONFIG_PITCH_R PAR_CONFIG_YAW_R'
   */
  rtb_Add_of[0] = 0.0174532924F * mlParamInterface.param[15L];
  rtb_Add_of[1] = 0.0174532924F * mlParamInterface.param[16L];
  rtb_Add_of[2] = 0.0174532924F * mlParamInterface.param[17L];

  /* Trigonometry: '<S674>/SinCos' */
  rtb_DataTypeConversion1_jq_idx_ = (real32_T)cos(rtb_Add_of[0]);
  rtb_Add_of[0] = (real32_T)sin(rtb_Add_of[0]);
  rtb_DataTypeConversion1_jq_id_0 = (real32_T)cos(rtb_Add_of[1]);
  rtb_Add_of[1] = (real32_T)sin(rtb_Add_of[1]);
  rtb_Sum1 = (real32_T)cos(rtb_Add_of[2]);
  rtb_jxi = (real32_T)sin(rtb_Add_of[2]);

  /* Product: '<S681>/u(5)*u(6)' incorporates:
   *  Trigonometry: '<S674>/SinCos'
   */
  rtb_VectorConcatenate_g[0] = rtb_DataTypeConversion1_jq_id_0 * rtb_Sum1;

  /* Sum: '<S684>/Sum' incorporates:
   *  Product: '<S684>/u(3)*u(4)'
   *  Product: '<S684>/u(6)*u(1)*u(2)'
   *  Trigonometry: '<S674>/SinCos'
   */
  rtb_VectorConcatenate_g[1] = rtb_Sum1 * rtb_Add_of[0] * rtb_Add_of[1] -
    rtb_jxi * rtb_DataTypeConversion1_jq_idx_;

  /* Sum: '<S687>/Sum' incorporates:
   *  Product: '<S687>/u(1)*u(3)'
   *  Product: '<S687>/u(2)*u(4)*u(6)'
   *  Trigonometry: '<S674>/SinCos'
   */
  rtb_VectorConcatenate_g[2] = rtb_Add_of[1] * rtb_DataTypeConversion1_jq_idx_ *
    rtb_Sum1 + rtb_Add_of[0] * rtb_jxi;

  /* Product: '<S682>/u(3)*u(5)' */
  rtb_VectorConcatenate_g[3] = rtb_jxi * rtb_DataTypeConversion1_jq_id_0;

  /* Sum: '<S685>/Sum' incorporates:
   *  Product: '<S685>/u(1)*u(2)*u(3)'
   *  Product: '<S685>/u(4)*u(6)'
   *  Trigonometry: '<S674>/SinCos'
   */
  rtb_VectorConcatenate_g[4] = rtb_Add_of[0] * rtb_Add_of[1] * rtb_jxi +
    rtb_DataTypeConversion1_jq_idx_ * rtb_Sum1;

  /* Sum: '<S688>/Sum' incorporates:
   *  Product: '<S688>/u(1)*u(6)'
   *  Product: '<S688>/u(2)*u(3)*u(4)'
   *  Trigonometry: '<S674>/SinCos'
   */
  rtb_VectorConcatenate_g[5] = rtb_Add_of[1] * rtb_jxi *
    rtb_DataTypeConversion1_jq_idx_ - rtb_Sum1 * rtb_Add_of[0];

  /* Gain: '<S683>/Gain2' */
  rtb_VectorConcatenate_g[6] = -rtb_Add_of[1];

  /* Product: '<S686>/u(1)*u(3)' */
  rtb_VectorConcatenate_g[7] = rtb_Add_of[0] * rtb_DataTypeConversion1_jq_id_0;

  /* Product: '<S689>/u(4)*u(5)' */
  rtb_VectorConcatenate_g[8] = rtb_DataTypeConversion1_jq_idx_ *
    rtb_DataTypeConversion1_jq_id_0;

  /* Math: '<S18>/Math Function' */
  for (i = 0; i < 3; i++) {
    rtb_MathFunction[3 * i] = rtb_VectorConcatenate_g[i];
    rtb_MathFunction[1 + 3 * i] = rtb_VectorConcatenate_g[i + 3];
    rtb_MathFunction[2 + 3 * i] = rtb_VectorConcatenate_g[i + 6];
  }

  /* End of Math: '<S18>/Math Function' */

  /* Sqrt: '<S479>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator1'
   *  Product: '<S480>/Product'
   *  Product: '<S480>/Product1'
   *  Product: '<S480>/Product2'
   *  Product: '<S480>/Product3'
   *  Sum: '<S480>/Sum'
   */
  rtb_Sum_b = (real32_T)sqrt
    (((AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] *
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] +
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] *
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1]) +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2]) +
     AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] *
     AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3]);

  /* Product: '<S452>/Product' incorporates:
   *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator1'
   */
  rtb_Product_hz =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] /
    rtb_Sum_b;

  /* Product: '<S452>/Product1' incorporates:
   *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator1'
   */
  rtb_Product1_bh =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] /
    rtb_Sum_b;

  /* Product: '<S452>/Product2' incorporates:
   *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator1'
   */
  rtb_Product2_ka =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] /
    rtb_Sum_b;

  /* Product: '<S452>/Product3' incorporates:
   *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator1'
   */
  rtb_Product3_e =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] /
    rtb_Sum_b;

  /* Sqrt: '<S492>/sqrt' incorporates:
   *  Product: '<S493>/Product'
   *  Product: '<S493>/Product1'
   *  Product: '<S493>/Product2'
   *  Product: '<S493>/Product3'
   *  Sum: '<S493>/Sum'
   */
  rtb_Sum_b = (real32_T)sqrt(((rtb_Product_hz * rtb_Product_hz + rtb_Product1_bh
    * rtb_Product1_bh) + rtb_Product2_ka * rtb_Product2_ka) + rtb_Product3_e *
    rtb_Product3_e);

  /* Product: '<S491>/Product' */
  rtb_kxi = rtb_Product_hz / rtb_Sum_b;

  /* Product: '<S491>/Product1' */
  rtb_ixj = rtb_Product1_bh / rtb_Sum_b;

  /* Product: '<S491>/Product2' */
  rtb_kxj = rtb_Product2_ka / rtb_Sum_b;

  /* Product: '<S491>/Product3' */
  rtb_Sum_b = rtb_Product3_e / rtb_Sum_b;

  /* Sum: '<S481>/Sum' incorporates:
   *  Product: '<S481>/Product'
   *  Product: '<S481>/Product1'
   *  Product: '<S481>/Product2'
   *  Product: '<S481>/Product3'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[0] = ((rtb_kxi * rtb_kxi +
    rtb_ixj * rtb_ixj) - rtb_kxj * rtb_kxj) - rtb_Sum_b * rtb_Sum_b;

  /* Gain: '<S484>/Gain' incorporates:
   *  Product: '<S484>/Product2'
   *  Product: '<S484>/Product3'
   *  Sum: '<S484>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[1] = (rtb_ixj * rtb_kxj -
    rtb_Sum_b * rtb_kxi) * 2.0F;

  /* Gain: '<S487>/Gain' incorporates:
   *  Product: '<S487>/Product1'
   *  Product: '<S487>/Product2'
   *  Sum: '<S487>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[2] = (rtb_kxi * rtb_kxj +
    rtb_ixj * rtb_Sum_b) * 2.0F;

  /* Gain: '<S482>/Gain' incorporates:
   *  Product: '<S482>/Product2'
   *  Product: '<S482>/Product3'
   *  Sum: '<S482>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[3] = (rtb_Sum_b * rtb_kxi +
    rtb_ixj * rtb_kxj) * 2.0F;

  /* Sum: '<S485>/Sum' incorporates:
   *  Product: '<S485>/Product'
   *  Product: '<S485>/Product1'
   *  Product: '<S485>/Product2'
   *  Product: '<S485>/Product3'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[4] = ((rtb_kxi * rtb_kxi -
    rtb_ixj * rtb_ixj) + rtb_kxj * rtb_kxj) - rtb_Sum_b * rtb_Sum_b;

  /* Gain: '<S488>/Gain' incorporates:
   *  Product: '<S488>/Product1'
   *  Product: '<S488>/Product2'
   *  Sum: '<S488>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[5] = (rtb_kxj * rtb_Sum_b -
    rtb_kxi * rtb_ixj) * 2.0F;

  /* Gain: '<S483>/Gain' incorporates:
   *  Product: '<S483>/Product1'
   *  Product: '<S483>/Product2'
   *  Sum: '<S483>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[6] = (rtb_ixj * rtb_Sum_b -
    rtb_kxi * rtb_kxj) * 2.0F;

  /* Gain: '<S486>/Gain' incorporates:
   *  Product: '<S486>/Product1'
   *  Product: '<S486>/Product2'
   *  Sum: '<S486>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[7] = (rtb_kxi * rtb_ixj +
    rtb_kxj * rtb_Sum_b) * 2.0F;

  /* Sum: '<S489>/Sum' incorporates:
   *  Product: '<S489>/Product'
   *  Product: '<S489>/Product1'
   *  Product: '<S489>/Product2'
   *  Product: '<S489>/Product3'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[8] = ((rtb_kxi * rtb_kxi -
    rtb_ixj * rtb_ixj) - rtb_kxj * rtb_kxj) + rtb_Sum_b * rtb_Sum_b;

  /* Gain: '<S468>/Gain1' incorporates:
   *  Selector: '<S468>/Selector1'
   */
  rtb_VectorConcatenate[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[3];
  rtb_VectorConcatenate[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[0];
  rtb_VectorConcatenate[2] = -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate
    [6];

  /* Gain: '<S468>/Gain2' incorporates:
   *  Selector: '<S468>/Selector2'
   */
  rtb_VectorConcatenate[3] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[7];
  rtb_VectorConcatenate[4] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[8];

  /* Gain: '<S468>/Gain3' incorporates:
   *  Selector: '<S468>/Selector3'
   */
  rtb_VectorConcatenate[5] = -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate
    [1];
  rtb_VectorConcatenate[6] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[4];

  /* If: '<S448>/If' incorporates:
   *  Gain: '<S468>/Gain1'
   *  Selector: '<S468>/Selector1'
   */
  if ((-AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[6] >= 1.0F) ||
      (-AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[6] <= -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S448>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S467>/Action Port'
     */
    AUAV3_WITH_S_AxisRotZeroR3(rtb_VectorConcatenate, &rtb_Merge[0], &rtb_Merge
      [1], &rtb_Merge[2]);

    /* End of Outputs for SubSystem: '<S448>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S448>/AxisRotDefault' incorporates:
     *  ActionPort: '<S466>/Action Port'
     */
    AUAV3_WITH__AxisRotDefault(rtb_VectorConcatenate, &rtb_Merge[0], &rtb_Merge
      [1], &rtb_Merge[2]);

    /* End of Outputs for SubSystem: '<S448>/AxisRotDefault' */
  }

  /* End of If: '<S448>/If' */

  /* MATLAB Function: '<S436>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_d(rtb_Merge[0],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_d);

  /* MATLAB Function: '<S573>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S567>/Gyro Gains'
   *  Constant: '<S573>/Constant'
   *  Constant: '<S573>/Constant1'
   *  DataTypeConversion: '<S567>/Data Type Conversion2'
   *  Gain: '<S576>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[1] *
    0.000872664619F), 0.01, 40.0,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n);

  /* MATLAB Function: '<S573>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S567>/Gyro Gains'
   *  Constant: '<S573>/Constant2'
   *  Constant: '<S573>/Constant3'
   *  DataTypeConversion: '<S567>/Data Type Conversion2'
   *  Gain: '<S576>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[0] *
    0.000872664619F), 0.01, 40.0,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S573>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S567>/Gyro Gains'
   *  Constant: '<S573>/Constant4'
   *  Constant: '<S573>/Constant5'
   *  DataTypeConversion: '<S567>/Data Type Conversion2'
   *  Gain: '<S576>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[2] *
    0.000872664619F), 0.01, 40.0,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2);

  /* MATLAB Function: '<S573>/myMux Fun' */
  AUAV3_WITH_SLUGS__myMuxFun
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2.y,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun);

  /* DataTypeConversion: '<S567>/Data Type Conversion7' */
  rtb_DataTypeConversion7_idx_0 = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun.y[0];
  rtb_DataTypeConversion7_idx_1 = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun.y[1];
  rtb_RhhcosphisinlambYe_h = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun.y[2];

  /* S-Function (sdspsubmtrx): '<S436>/Submatrix1' */
  yIdx = 0L;
  for (i = 0; i < 3; i++) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Submatrix1[yIdx] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[(int32_T)(i * 3)];
    yIdx++;
  }

  /* End of S-Function (sdspsubmtrx): '<S436>/Submatrix1' */

  /* MATLAB Function: '<S436>/myMux Fun2' incorporates:
   *  Constant: '<S436>/Constant1'
   */
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Attitude Complimentary Filter COG/myMux Fun2': '<S454>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S454>:1:5' y = [u1(1); u2(1); u3(1)]; */
  rtb_Add_of[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Submatrix1[0];
  rtb_Add_of[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Submatrix1[1];
  rtb_Add_of[2] = 0.0F;

  /* MATLAB Function: '<S476>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_h(rtb_Add_of,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_i);

  /* MATLAB Function: '<S475>/negprotect' */
  AUAV3_WITH_SLUG_negprotect
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_i.xDoty,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect);

  /* S-Function (MCHP_C_function_Call): '<S475>/[apUtils.c]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_d = mySqrt(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect.zpVal
    );

  /* Saturate: '<S451>/Zero Bound' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_d <= 0.001F) {
    rtb_Sum1 = 0.001F;
  } else {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_d;
  }

  /* End of Saturate: '<S451>/Zero Bound' */

  /* Product: '<S451>/Divide' incorporates:
   *  MATLAB Function: '<S436>/myMux Fun2'
   */
  rtb_Add_of[0] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Submatrix1[0] / rtb_Sum1;
  rtb_Add_of[1] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Submatrix1[1] / rtb_Sum1;
  rtb_jxi = 0.0F / rtb_Sum1;

  /* Product: '<S464>/i x j' */
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/COG.SOG2V/myMux Fun2': '<S496>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S496>:1:5' y = [u1(1); u2(1); u3(1)]; */
  rtb_Sum1 = rtb_Add_of[0];

  /* Product: '<S465>/i x k' */
  rtb_Deg2R1_bc = rtb_Add_of[0];

  /* Product: '<S465>/j x i' */
  rtb_RhhcosphicoslambXe_cj = rtb_Add_of[1];

  /* Sum: '<S446>/Sum' incorporates:
   *  MATLAB Function: '<S437>/myMux Fun2'
   *  Product: '<S464>/i x j'
   *  Product: '<S464>/j x k'
   *  Product: '<S464>/k x i'
   *  Product: '<S465>/i x k'
   *  Product: '<S465>/j x i'
   *  Product: '<S465>/k x j'
   */
  rtb_Add_of[0] = rtb_Add_of[1] * 0.0F - rtb_jxi *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc;
  rtb_Add_of[1] = rtb_jxi * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc1 -
    rtb_Deg2R1_bc * 0.0F;
  rtb_Add_of[2] = rtb_Sum1 * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc -
    rtb_RhhcosphicoslambXe_cj * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc1;

  /* Product: '<S436>/Product2' incorporates:
   *  Gain: '<S436>/Gain1'
   */
  for (i = 0; i < 3; i++) {
    tmp[i] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 6] *
      rtb_Add_of[2] + (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 3] *
                       rtb_Add_of[1] +
                       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i] *
                       rtb_Add_of[0]);
  }

  /* End of Product: '<S436>/Product2' */

  /* Product: '<S436>/Product1' */
  for (i = 0; i < 3; i++) {
    rtb_P32[i] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 6] *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[2] +
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 3] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[1] +
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[0]);
  }

  /* End of Product: '<S436>/Product1' */

  /* MATLAB Function: '<S574>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S567>/Gyro Gains1'
   *  Constant: '<S574>/Constant'
   *  Constant: '<S574>/Constant1'
   *  DataTypeConversion: '<S567>/Data Type Conversion1'
   *  Product: '<S567>/Divide1'
   */
  A_EmbeddedMATLABFunction_n((real_T)((real32_T)rtb_Switch_a[4] * 0.0326839499F),
    0.01, 40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_dv,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_dv);

  /* MATLAB Function: '<S574>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S567>/Gyro Gains1'
   *  Constant: '<S574>/Constant2'
   *  Constant: '<S574>/Constant3'
   *  DataTypeConversion: '<S567>/Data Type Conversion1'
   *  Product: '<S567>/Divide1'
   */
  A_EmbeddedMATLABFunction_n((real_T)((real32_T)rtb_Switch_a[3] * 0.0326839499F),
    0.01, 40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_b,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_b);

  /* MATLAB Function: '<S574>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S567>/Gyro Gains1'
   *  Constant: '<S574>/Constant4'
   *  Constant: '<S574>/Constant5'
   *  DataTypeConversion: '<S567>/Data Type Conversion1'
   *  Product: '<S567>/Divide1'
   */
  A_EmbeddedMATLABFunction_n((real_T)((real32_T)rtb_Switch_a[5] * 0.0326839499F),
    0.01, 40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_i,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_i);

  /* MATLAB Function: '<S574>/myMux Fun' */
  AUAV3_WITH_SLUGS__myMuxFun
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_dv.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_b.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_i.y,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h);

  /* Product: '<S510>/Product1' */
  for (i = 0; i < 3; i++) {
    rtb_Product1_a3[i] = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 6]
      * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[2] +
      (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i + 3] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[1] +
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[i] *
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun2.y[0]);
  }

  /* End of Product: '<S510>/Product1' */

  /* Sum: '<S510>/Sum' incorporates:
   *  DataTypeConversion: '<S567>/Data Type Conversion3'
   *  Delay: '<S517>/Integer Delay1'
   *  Gain: '<S517>/Gain2'
   *  Sum: '<S517>/Sum5'
   */
  rtb_DataTypeConversion1_jq_idx_ = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[0] - (rtb_Product1_a3[0] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[0]) * 20.0F;
  rtb_DataTypeConversion1_jq_id_0 = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[1] - (rtb_Product1_a3[1] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[1]) * 20.0F;
  rtb_kxj = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[2] -
    (rtb_Product1_a3[2] -
     AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[2]) * 20.0F;

  /* RateLimiter: '<S510>/Rate Limiter' */
  rtb_Sum1_ao[0] = rtb_DataTypeConversion1_jq_idx_ -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[0];
  rtb_Sum1_ao[1] = rtb_DataTypeConversion1_jq_id_0 -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[1];
  rtb_Sum1_ao[2] = rtb_kxj - AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[2];
  rtb_Sum1 = rtb_DataTypeConversion1_jq_idx_;
  if (rtb_Sum1_ao[0] > 9.896E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[0] + 9.896E-9F;
  } else {
    if (rtb_Sum1_ao[0] < -9.896E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[0] + -9.896E-9F;
    }
  }

  rtb_DataTypeConversion1_jq_idx_ = rtb_Sum1;
  rtb_Sum1 = rtb_DataTypeConversion1_jq_id_0;
  if (rtb_Sum1_ao[1] > 9.896E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[1] + 9.896E-9F;
  } else {
    if (rtb_Sum1_ao[1] < -9.896E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[1] + -9.896E-9F;
    }
  }

  rtb_DataTypeConversion1_jq_id_0 = rtb_Sum1;
  rtb_Sum1 = rtb_kxj;
  if (rtb_Sum1_ao[2] > 9.896E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[2] + 9.896E-9F;
  } else {
    if (rtb_Sum1_ao[2] < -9.896E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[2] + -9.896E-9F;
    }
  }

  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[0] = rtb_DataTypeConversion1_jq_idx_;
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[1] = rtb_DataTypeConversion1_jq_id_0;
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[2] = rtb_Sum1;

  /* End of RateLimiter: '<S510>/Rate Limiter' */

  /* DataTypeConversion: '<S512>/Data Type Conversion2' */
  rtb_DataTypeConversion2_d[0] = rtb_DataTypeConversion1_jq_idx_;
  rtb_DataTypeConversion2_d[1] = rtb_DataTypeConversion1_jq_id_0;
  rtb_DataTypeConversion2_d[2] = rtb_Sum1;

  /* DiscreteZeroPole: '<S524>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole = 9.8940417712526327E-7*rtb_DataTypeConversion2_d[0];
    rtb_DiscreteZeroPole += 3.9156825028515473E-10*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l;
  }

  /* DiscreteZeroPole: '<S525>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_n = 9.8940417712526327E-7*rtb_DataTypeConversion2_d[1];
    rtb_DiscreteZeroPole_n += 3.9156825028515473E-10*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_i;
  }

  /* DiscreteZeroPole: '<S526>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_m = 9.8940417712526327E-7*rtb_DataTypeConversion2_d[2];
    rtb_DiscreteZeroPole_m += 3.9156825028515473E-10*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* MATLAB Function: '<S512>/myMux Fun1' */
  AUAV3_WITH_SLUGS_myMuxFun1(rtb_DiscreteZeroPole, rtb_DiscreteZeroPole_n,
    rtb_DiscreteZeroPole_m, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n);

  /* DataTypeConversion: '<S512>/Data Type Conversion1' */
  rtb_DataTypeConversion1_jq_idx_ = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[0];
  rtb_DataTypeConversion1_jq_id_0 = (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[1];
  rtb_kxj = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[2];

  /* Sum: '<S436>/Add' incorporates:
   *  DataTypeConversion: '<S512>/Data Type Conversion1'
   *  DataTypeConversion: '<S567>/Data Type Conversion3'
   *  Delay: '<S436>/Integer Delay'
   *  Delay: '<S447>/Integer Delay1'
   *  Gain: '<S447>/Gain2'
   *  Product: '<S462>/i x j'
   *  Product: '<S462>/j x k'
   *  Product: '<S462>/k x i'
   *  Product: '<S463>/i x k'
   *  Product: '<S463>/j x i'
   *  Product: '<S463>/k x j'
   *  Sum: '<S445>/Sum'
   *  Sum: '<S447>/Sum5'
   */
  rtb_Add_of[0] = (((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[1] *
                     rtb_P32[2] -
                     AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[2] *
                     rtb_P32[1]) + (rtb_P32[0] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[0]) * 20.0F) -
                   (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[0])
    - (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[0];
  rtb_Add_of[1] = (((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[2] *
                     rtb_P32[0] -
                     AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[0] *
                     rtb_P32[2]) + (rtb_P32[1] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[1]) * 20.0F) -
                   (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[1])
    - (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[1];
  rtb_Add_of[2] = (((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[0] *
                     rtb_P32[1] -
                     AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[1] *
                     rtb_P32[0]) + (rtb_P32[2] -
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[2]) * 20.0F) -
                   (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[2])
    - (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_n.y[2];

  /* MATLAB Function: '<S471>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_h(rtb_Add_of,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_h);

  /* MATLAB Function: '<S470>/negprotect' */
  AUAV3_WITH_SLUG_negprotect
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_h.xDoty,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_d);

  /* S-Function (MCHP_C_function_Call): '<S470>/[apUtils.c]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_p = mySqrt(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_d.zpVal
    );

  /* Saturate: '<S450>/Zero Bound' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_p <= 0.001F) {
    rtb_Sum1 = 0.001F;
  } else {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.apUtilsc_p;
  }

  /* End of Saturate: '<S450>/Zero Bound' */

  /* Product: '<S450>/Divide' */
  rtb_Add_of[0] /= rtb_Sum1;
  rtb_Add_of[1] /= rtb_Sum1;
  rtb_jxi = rtb_Add_of[2] / rtb_Sum1;

  /* S-Function (sdspsubmtrx): '<S436>/Submatrix' */
  yIdx = 0L;
  i = 2;
  while (i <= 2) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[yIdx] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[6L];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[1L + yIdx] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[7L];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[2L + yIdx] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.VectorConcatenate[8L];
    yIdx += 3L;
    i = 3;
  }

  /* End of S-Function (sdspsubmtrx): '<S436>/Submatrix' */

  /* Product: '<S460>/i x j' */
  rtb_Sum1 = rtb_Add_of[0];

  /* Product: '<S461>/i x k' */
  rtb_Deg2R1_bc = rtb_Add_of[0];

  /* Product: '<S461>/j x i' */
  rtb_RhhcosphicoslambXe_cj = rtb_Add_of[1];

  /* Sum: '<S436>/Sum4' incorporates:
   *  Gain: '<S436>/Gain1'
   *  Product: '<S460>/i x j'
   *  Product: '<S460>/j x k'
   *  Product: '<S460>/k x i'
   *  Product: '<S461>/i x k'
   *  Product: '<S461>/j x i'
   *  Product: '<S461>/k x j'
   *  Sum: '<S444>/Sum'
   */
  rtb_Add_of[0] = (rtb_Add_of[1] * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[2] -
                   rtb_jxi * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[1]) + 30.0F *
    tmp[0];
  rtb_Add_of[1] = (rtb_jxi * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[0] -
                   rtb_Deg2R1_bc * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[2]) +
    30.0F * tmp[1];
  rtb_Add_of[2] = (rtb_Sum1 * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[1] -
                   rtb_RhhcosphicoslambXe_cj *
                   AUAV3_WITH_SLUGS_SENSOR_AND_C_B.g_hat[0]) + 30.0F * tmp[2];

  /* DataTypeConversion: '<S443>/Data Type Conversion2' */
  rtb_DataTypeConversion2_dp[0] = rtb_Add_of[0];
  rtb_DataTypeConversion2_dp[1] = rtb_Add_of[1];
  rtb_DataTypeConversion2_dp[2] = rtb_Add_of[2];

  /* DiscreteZeroPole: '<S456>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_c = 0.005*rtb_DataTypeConversion2_dp[0];
    rtb_DiscreteZeroPole_c += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_d;
  }

  /* DiscreteZeroPole: '<S457>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_g = 0.005*rtb_DataTypeConversion2_dp[1];
    rtb_DiscreteZeroPole_g += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nu;
  }

  /* DiscreteZeroPole: '<S458>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_a = 0.005*rtb_DataTypeConversion2_dp[2];
    rtb_DiscreteZeroPole_a += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* MATLAB Function: '<S443>/myMux Fun1' */
  AUAV3_WITH_SLUGS_myMuxFun1(rtb_DiscreteZeroPole_c, rtb_DiscreteZeroPole_g,
    rtb_DiscreteZeroPole_a, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h);

  /* RateLimiter: '<S436>/Bias Rate Limiter' incorporates:
   *  DataTypeConversion: '<S443>/Data Type Conversion1'
   */
  rtb_Sum1_ao[0] = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[0]
    - AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[0];
  rtb_Sum1_ao[1] = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[1]
    - AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[1];
  rtb_Sum1_ao[2] = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[2]
    - AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[2];

  /* DataTypeConversion: '<S443>/Data Type Conversion1' */
  rtb_Sum1 = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[0];

  /* RateLimiter: '<S436>/Bias Rate Limiter' */
  if (rtb_Sum1_ao[0] > 2.5572E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[0] + 2.5572E-9F;
  } else {
    if (rtb_Sum1_ao[0] < -2.5572E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[0] + -2.5572E-9F;
    }
  }

  rtb_u11[0] = rtb_Sum1;

  /* DataTypeConversion: '<S443>/Data Type Conversion1' */
  rtb_Sum1 = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[1];

  /* RateLimiter: '<S436>/Bias Rate Limiter' */
  if (rtb_Sum1_ao[1] > 2.5572E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[1] + 2.5572E-9F;
  } else {
    if (rtb_Sum1_ao[1] < -2.5572E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[1] + -2.5572E-9F;
    }
  }

  rtb_u11[1] = rtb_Sum1;

  /* DataTypeConversion: '<S443>/Data Type Conversion1' */
  rtb_Sum1 = (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun1_h.y[2];

  /* RateLimiter: '<S436>/Bias Rate Limiter' */
  if (rtb_Sum1_ao[2] > 2.5572E-9F) {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[2] + 2.5572E-9F;
  } else {
    if (rtb_Sum1_ao[2] < -2.5572E-9F) {
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[2] + -2.5572E-9F;
    }
  }

  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[0] = rtb_u11[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[1] = rtb_u11[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[2] = rtb_Sum1;

  /* Sum: '<S436>/Sum3' */
  rtb_DataTypeConversion7_idx_0 += rtb_u11[0];
  rtb_DataTypeConversion7_idx_1 += rtb_u11[1];
  rtb_DataTypeConversion7 = rtb_RhhcosphisinlambYe_h + rtb_Sum1;

  /* Trigonometry: '<S675>/SinCos' incorporates:
   *  MATLAB Function: '<Root>/myMux Fun3'
   *  SignalConversion: '<S21>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'myMux Fun3': '<S21>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S21>:1:5' y = [u1(1:3); u2(1:3)]; */
  rtb_u11[0] = (real32_T)sin(rtb_Merge[2]);
  rtb_Sum1 = (real32_T)cos(rtb_Merge[2]);
  rtb_u11[1] = (real32_T)sin(rtb_Merge[1]);
  rtb_Deg2R1_bc = (real32_T)cos(rtb_Merge[1]);
  rtb_u11[2] = (real32_T)sin
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_d.y);
  rtb_RhhcosphicoslambXe_cj = (real32_T)cos
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_d.y);

  /* Product: '<S691>/u(5)*u(6)' */
  rtb_VectorConcatenate_k[0] = rtb_Deg2R1_bc * rtb_RhhcosphicoslambXe_cj;

  /* Sum: '<S694>/Sum' incorporates:
   *  Product: '<S694>/u(3)*u(4)'
   *  Product: '<S694>/u(6)*u(1)*u(2)'
   */
  rtb_VectorConcatenate_k[1] = rtb_RhhcosphicoslambXe_cj * rtb_u11[0] * rtb_u11
    [1] - rtb_u11[2] * rtb_Sum1;

  /* Sum: '<S697>/Sum' incorporates:
   *  Product: '<S697>/u(1)*u(3)'
   *  Product: '<S697>/u(2)*u(4)*u(6)'
   */
  rtb_VectorConcatenate_k[2] = rtb_u11[1] * rtb_Sum1 * rtb_RhhcosphicoslambXe_cj
    + rtb_u11[0] * rtb_u11[2];

  /* Product: '<S692>/u(3)*u(5)' */
  rtb_VectorConcatenate_k[3] = rtb_u11[2] * rtb_Deg2R1_bc;

  /* Sum: '<S695>/Sum' incorporates:
   *  Product: '<S695>/u(1)*u(2)*u(3)'
   *  Product: '<S695>/u(4)*u(6)'
   */
  rtb_VectorConcatenate_k[4] = rtb_u11[0] * rtb_u11[1] * rtb_u11[2] + rtb_Sum1 *
    rtb_RhhcosphicoslambXe_cj;

  /* Sum: '<S698>/Sum' incorporates:
   *  Product: '<S698>/u(1)*u(6)'
   *  Product: '<S698>/u(2)*u(3)*u(4)'
   */
  rtb_VectorConcatenate_k[5] = rtb_u11[1] * rtb_u11[2] * rtb_Sum1 -
    rtb_RhhcosphicoslambXe_cj * rtb_u11[0];

  /* Gain: '<S693>/Gain2' */
  rtb_VectorConcatenate_k[6] = -rtb_u11[1];

  /* Product: '<S696>/u(1)*u(3)' */
  rtb_VectorConcatenate_k[7] = rtb_u11[0] * rtb_Deg2R1_bc;

  /* Product: '<S699>/u(4)*u(5)' */
  rtb_VectorConcatenate_k[8] = rtb_Sum1 * rtb_Deg2R1_bc;

  /* Product: '<S18>/Product' */
  for (i = 0; i < 3; i++) {
    for (rtb_DataTypeConversion1_a = 0; rtb_DataTypeConversion1_a < 3;
         rtb_DataTypeConversion1_a++) {
      rtb_VectorConcatenate_g[i + 3 * rtb_DataTypeConversion1_a] = 0.0F;
      rtb_VectorConcatenate_g[i + 3 * rtb_DataTypeConversion1_a] +=
        rtb_VectorConcatenate_k[3 * rtb_DataTypeConversion1_a] *
        rtb_MathFunction[i];
      rtb_VectorConcatenate_g[i + 3 * rtb_DataTypeConversion1_a] +=
        rtb_VectorConcatenate_k[3 * rtb_DataTypeConversion1_a + 1] *
        rtb_MathFunction[i + 3];
      rtb_VectorConcatenate_g[i + 3 * rtb_DataTypeConversion1_a] +=
        rtb_VectorConcatenate_k[3 * rtb_DataTypeConversion1_a + 2] *
        rtb_MathFunction[i + 6];
    }
  }

  /* End of Product: '<S18>/Product' */

  /* Gain: '<S680>/Gain1' incorporates:
   *  Selector: '<S680>/Selector1'
   */
  rtb_VectorConcatenate[0] = rtb_VectorConcatenate_g[3];
  rtb_VectorConcatenate[1] = rtb_VectorConcatenate_g[0];
  rtb_VectorConcatenate[2] = -rtb_VectorConcatenate_g[6];

  /* Gain: '<S680>/Gain2' incorporates:
   *  Selector: '<S680>/Selector2'
   */
  rtb_VectorConcatenate[3] = rtb_VectorConcatenate_g[7];
  rtb_VectorConcatenate[4] = rtb_VectorConcatenate_g[8];

  /* Gain: '<S680>/Gain3' incorporates:
   *  Selector: '<S680>/Selector3'
   */
  rtb_VectorConcatenate[5] = -rtb_VectorConcatenate_g[1];
  rtb_VectorConcatenate[6] = rtb_VectorConcatenate_g[4];

  /* If: '<S672>/If' incorporates:
   *  Gain: '<S680>/Gain1'
   *  Selector: '<S680>/Selector1'
   */
  if ((-rtb_VectorConcatenate_g[6] >= 1.0F) || (-rtb_VectorConcatenate_g[6] <=
       -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S672>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S679>/Action Port'
     */
    AUAV3_WITH_S_AxisRotZeroR3(rtb_VectorConcatenate, &rtb_Merge_p[0],
      &rtb_Merge_p[1], &rtb_Merge_p[2]);

    /* End of Outputs for SubSystem: '<S672>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S672>/AxisRotDefault' incorporates:
     *  ActionPort: '<S678>/Action Port'
     */
    AUAV3_WITH__AxisRotDefault(rtb_VectorConcatenate, &rtb_Merge_p[0],
      &rtb_Merge_p[1], &rtb_Merge_p[2]);

    /* End of Outputs for SubSystem: '<S672>/AxisRotDefault' */
  }

  /* End of If: '<S672>/If' */

  /* MATLAB Function: '<S18>/Embedded MATLAB Function1' */
  A_EmbeddedMATLABFunction_d(rtb_Merge_p[0],
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_be);

  /* Product: '<S18>/Product1' incorporates:
   *  MATLAB Function: '<Root>/myMux Fun3'
   */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun1': '<S676>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S676>:1:5' y = [u1(1); u2(1); u3(1)]; */
  for (i = 0; i < 3; i++) {
    rtb_u11[i] = rtb_MathFunction[i + 6] * rtb_DataTypeConversion7 +
      (rtb_MathFunction[i + 3] * rtb_DataTypeConversion7_idx_1 +
       rtb_MathFunction[i] * rtb_DataTypeConversion7_idx_0);
  }

  /* End of Product: '<S18>/Product1' */

  /* MATLAB Function: '<S18>/myMux Fun2' incorporates:
   *  MATLAB Function: '<S18>/myMux Fun1'
   */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun2': '<S677>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S677>:1:5' y = [u1(1:3); u2(1:3)]; */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0] = rtb_Merge_p[2];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[1] = rtb_Merge_p[1];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[2] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_be.y;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[3] = rtb_u11[0];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[4] = rtb_u11[1];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[5] = rtb_u11[2];

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S128>/[0 1000]' */
  if (rtb_Switch3 > 1000.0F) {
    rtb_Sum1 = 1000.0F;
  } else if (rtb_Switch3 < 0.0F) {
    rtb_Sum1 = 0.0F;
  } else {
    rtb_Sum1 = rtb_Switch3;
  }

  /* Sum: '<S128>/Add' incorporates:
   *  Constant: '<S128>/Constant'
   *  Constant: '<S128>/Constant from Model'
   *  Gain: '<S133>/Unit Conversion'
   *  Product: '<S128>/Divide'
   *  Saturate: '<S128>/[0 1000]'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Add = 1.0F - 3.28084F * rtb_Sum1 / 145442.0F;

  /* S-Function (MCHP_C_function_Call): '<S134>/myTan() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116 = myPow(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Add
    , 4.25587606F
    );

  /* Product: '<S128>/Divide1' incorporates:
   *  Constant: '<S128>/Rho_0 (Kg//m^3)'
   */
  rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116 * 1.225F;

  /* Saturate: '<S129>/[ 0.01 50000]' */
  if (rtb_jxi > 50000.0F) {
    rtb_jxi = 50000.0F;
  } else {
    if (rtb_jxi < 0.01F) {
      rtb_jxi = 0.01F;
    }
  }

  /* MATLAB Function: '<S136>/negprotect' incorporates:
   *  Constant: '<S129>/a'
   *  DataStoreRead: '<Root>/Get mlAirData'
   *  Product: '<S129>/Divide2'
   *  Saturate: '<S129>/[ 0.01 50000]'
   */
  AUAV3_WITH_SL_negprotect_d(2.0F * mlAirData.press_diff / rtb_jxi,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_dv);

  /* S-Function (MCHP_C_function_Call): '<S136>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_l = mySqrt(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_dv.zpVal
    );

  /* Switch: '<S130>/Switch3' incorporates:
   *  RelationalOperator: '<S130>/Relational Operator2'
   */
  if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_l ==
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_l) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_m =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.mySqrtapUtilscupdated5116_l;
  }

  /* End of Switch: '<S130>/Switch3' */

  /* MATLAB Function: '<S127>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S127>/Constant'
   *  Constant: '<S127>/Constant1'
   */
  AUA_EmbeddedMATLABFunction
    (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_m, 0.01, 1.0,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_hb,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_hb);

  /* Switch: '<S107>/Schedule LPF' incorporates:
   *  Constant: '<S131>/Constant'
   *  RelationalOperator: '<S131>/Compare'
   */
  if ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_m <= 5.0F) > 0) {
    rtb_RhhcosphisinlambYe_h =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_m;
  } else {
    rtb_RhhcosphisinlambYe_h =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_hb.y;
  }

  /* End of Switch: '<S107>/Schedule LPF' */

  /* Product: '<S112>/delta rise limit' incorporates:
   *  Constant: '<S102>/Constant3'
   *  SampleTimeMath: '<S112>/sample time'
   *
   * About '<S112>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Deg2R1_bc = 0.02F;

  /* Switch: '<S140>/Init' incorporates:
   *  UnitDelay: '<S140>/FixPt Unit Delay1'
   *  UnitDelay: '<S140>/FixPt Unit Delay2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay2_DSTATE != 0) {
    rtb_RhhcosphicoslambXe_cj = 100.0F;
  } else {
    rtb_RhhcosphicoslambXe_cj =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay1_DSTATE;
  }

  /* End of Switch: '<S140>/Init' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Switch: '<S4>/Switch' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  if (rtb_IC1_l) {
    rtb_Sum1 = mlMidLevelCommands.uCommand;
  } else {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge1;
  }

  /* End of Switch: '<S4>/Switch' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Sum: '<S112>/Difference Inputs1'
   *
   * Block description for '<S112>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Sum1 -= rtb_RhhcosphicoslambXe_cj;

  /* Switch: '<S139>/Switch2' incorporates:
   *  RelationalOperator: '<S139>/LowerRelop1'
   */
  if (!(rtb_Sum1 > 0.02F)) {
    /* Switch: '<S139>/Switch' incorporates:
     *  RelationalOperator: '<S139>/UpperRelop'
     */
    if (rtb_Sum1 < -0.03F) {
      rtb_Deg2R1_bc = -0.03F;
    } else {
      rtb_Deg2R1_bc = rtb_Sum1;
    }

    /* End of Switch: '<S139>/Switch' */
  }

  /* End of Switch: '<S139>/Switch2' */

  /* Sum: '<S112>/Difference Inputs2'
   *
   * Block description for '<S112>/Difference Inputs2':
   *
   *  Add in CPU
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay1_DSTATE = rtb_Deg2R1_bc +
    rtb_RhhcosphicoslambXe_cj;

  /* Sum: '<S102>/Add2' */
  rtb_RhhcosphicoslambXe_cj =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay1_DSTATE - rtb_Switch3;

  /* Switch: '<S109>/Switch3' incorporates:
   *  RelationalOperator: '<S109>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_o =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S109>/Switch3' */

  /* Product: '<S105>/Product1' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   */
  rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_o *
    mlParamInterface.param[9];

  /* Sum: '<S105>/Sum2' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Gain: '<S105>/Gain'
   *  Memory: '<S105>/Memory1'
   *  Product: '<S105>/Product4'
   */
  rtb_RhhcosphicoslambXe_cj = (0.01F *
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_o *
    mlParamInterface.param[10] +
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput) + rtb_Sum1;

  /* Switch: '<S105>/AntiWindup' incorporates:
   *  Constant: '<S105>/Constant5'
   *  Logic: '<S105>/Logical Operator'
   *  RelationalOperator: '<S105>/Relational Operator'
   *  RelationalOperator: '<S105>/Relational Operator1'
   */
  if ((rtb_RhhcosphicoslambXe_cj > -0.261799395F) && (rtb_RhhcosphicoslambXe_cj <
       0.261799395F)) {
    rtb_RhhcosphicoslambXe_cj =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_o;
  } else {
    rtb_RhhcosphicoslambXe_cj = 0.0F;
  }

  /* End of Switch: '<S105>/AntiWindup' */

  /* Switch: '<S122>/Switch3' incorporates:
   *  RelationalOperator: '<S122>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_me =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S122>/Switch3' */

  /* Switch: '<S120>/Switch1' incorporates:
   *  Constant: '<S120>/Constant'
   *  Constant: '<S120>/Constant1'
   *  Constant: '<S120>/Constant2'
   *  Constant: '<S120>/Constant3'
   *  Constant: '<S120>/Constant5'
   *  Delay: '<S120>/Integer Delay'
   *  Delay: '<S120>/Integer Delay1'
   *  Delay: '<S120>/Integer Delay2'
   *  Product: '<S120>/Product'
   *  Product: '<S120>/Product1'
   *  Product: '<S120>/Product2'
   *  Product: '<S120>/Product3'
   *  Sum: '<S120>/Subtract'
   *  Sum: '<S120>/Subtract1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_RhhcosphicoslambXe_cj = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe_cj =
      ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_me * 0.333333343F +
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_k * 1.33333337F) +
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_g[0] * 0.333333343F)
      * 0.005F + AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE;
  }

  /* End of Switch: '<S120>/Switch1' */

  /* Switch: '<S121>/Switch3' incorporates:
   *  RelationalOperator: '<S121>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_e0 =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S121>/Switch3' */

  /* Switch: '<S105>/On//Off' incorporates:
   *  Constant: '<S105>/Constant1'
   *  DataStoreRead: '<S7>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Product: '<S105>/Product'
   *  Sum: '<S105>/Add2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_e0 *
      mlParamInterface.param[10] + rtb_Sum1;
  }

  /* End of Switch: '<S105>/On//Off' */

  /* Switch: '<S119>/Switch2' incorporates:
   *  RelationalOperator: '<S119>/LowerRelop1'
   *  RelationalOperator: '<S119>/UpperRelop'
   *  Switch: '<S119>/Switch'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput > 0.261799395F) {
    /* Saturate: '<S102>/Theta_c Limit' */
    rtb_RhhcosphicoslambXe_cj = 0.261799395F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput <
             -0.261799395F) {
    /* Switch: '<S119>/Switch' incorporates:
     *  Saturate: '<S102>/Theta_c Limit'
     */
    rtb_RhhcosphicoslambXe_cj = -0.261799395F;
  } else {
    /* Saturate: '<S102>/Theta_c Limit' incorporates:
     *  Switch: '<S119>/Switch'
     */
    rtb_RhhcosphicoslambXe_cj =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput;
  }

  /* End of Switch: '<S119>/Switch2' */

  /* DataStoreWrite: '<S33>/mlNavigation' */
  mlNavigation.u_m = rtb_RhhcosphisinlambYe_h;
  mlNavigation.theta_c = rtb_RhhcosphicoslambXe_cj;

  /* DataTypeConversion: '<S102>/Data Type Conversion' */
  rtb_jxi = (real32_T)floor
    (AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay1_DSTATE);
  if (rtIsNaNF(rtb_jxi) || rtIsInfF(rtb_jxi)) {
    rtb_jxi = 0.0F;
  } else {
    rtb_jxi = (real32_T)fmod(rtb_jxi, 65536.0F);
  }

  /* DataStoreWrite: '<S33>/mlNavigation' incorporates:
   *  DataTypeConversion: '<S102>/Data Type Conversion'
   */
  mlNavigation.h_c = rtb_jxi < 0.0F ? (uint16_T)-(int16_T)(uint16_T)-rtb_jxi :
    (uint16_T)rtb_jxi;

  /* Switch: '<S111>/Switch3' incorporates:
   *  RelationalOperator: '<S111>/Relational Operator2'
   */
  if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0] ==
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0]) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_c =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0];
  }

  /* End of Switch: '<S111>/Switch3' */

  /* MATLAB Function: '<S103>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S103>/Constant'
   *  Constant: '<S103>/Constant1'
   */
  AUA_EmbeddedMATLABFunction
    (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_c, 0.01, 0.32,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n4,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n4);

  /* Sum: '<S102>/Add' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  rtb_Deg2R1_bc = mlMidLevelCommands.hCommand - rtb_RhhcosphisinlambYe_h;

  /* Sum: '<S104>/Add3' incorporates:
   *  Constant: '<S104>/SaturationLimit'
   */
  rtb_Add1_o = 0.95F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion;

  /* Switch: '<S108>/Switch3' incorporates:
   *  RelationalOperator: '<S108>/Relational Operator2'
   */
  if ((rtb_Deg2R1_bc == rtb_Deg2R1_bc) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b = rtb_Deg2R1_bc;
  }

  /* End of Switch: '<S108>/Switch3' */

  /* Sum: '<S104>/Add1' incorporates:
   *  Constant: '<S104>/delayTime'
   *  DataStoreRead: '<S7>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Delay: '<S104>/NDelays'
   *  Product: '<S104>/Product1'
   *  Product: '<S104>/Product2'
   *  Product: '<S104>/Product3'
   *  Sum: '<S104>/Sum'
   */
  rtb_Sum1 = (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b -
              AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[0]) / 0.05F *
    mlParamInterface.param[2] +
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b *
    mlParamInterface.param[0];

  /* Sum: '<S104>/Sum2' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Gain: '<S104>/Gain'
   *  Memory: '<S104>/Memory1'
   *  Product: '<S104>/Product4'
   */
  rtb_ixj = (0.01F * AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b *
             mlParamInterface.param[1] +
             AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a) + rtb_Sum1;

  /* Switch: '<S104>/AntiWindup' incorporates:
   *  Constant: '<S104>/Constant5'
   *  Constant: '<S104>/SaturationLimit'
   *  Constant: '<S104>/SaturationLimit1'
   *  Logic: '<S104>/Logical Operator'
   *  RelationalOperator: '<S104>/Relational Operator'
   *  RelationalOperator: '<S104>/Relational Operator1'
   *  Sum: '<S104>/Add3'
   *  Sum: '<S104>/Add4'
   */
  if ((rtb_ixj > 0.0F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion) &&
      (rtb_ixj < 0.95F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion)) {
    rtb_ixj = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b;
  } else {
    rtb_ixj = 0.0F;
  }

  /* End of Switch: '<S104>/AntiWindup' */

  /* Switch: '<S117>/Switch3' incorporates:
   *  RelationalOperator: '<S117>/Relational Operator2'
   */
  if ((rtb_ixj == rtb_ixj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k = rtb_ixj;
  }

  /* End of Switch: '<S117>/Switch3' */

  /* Switch: '<S116>/Switch1' incorporates:
   *  Constant: '<S116>/Constant'
   *  Constant: '<S116>/Constant1'
   *  Constant: '<S116>/Constant2'
   *  Constant: '<S116>/Constant3'
   *  Constant: '<S116>/Constant5'
   *  Delay: '<S116>/Integer Delay'
   *  Delay: '<S116>/Integer Delay1'
   *  Delay: '<S116>/Integer Delay2'
   *  Product: '<S116>/Product'
   *  Product: '<S116>/Product1'
   *  Product: '<S116>/Product2'
   *  Product: '<S116>/Product3'
   *  Sum: '<S116>/Subtract'
   *  Sum: '<S116>/Subtract1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_ixj = 0.0F;
  } else {
    rtb_ixj = ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k *
                0.333333343F +
                AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p *
                1.33333337F) +
               AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_f[0] *
               0.333333343F) * 0.005F +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_j;
  }

  /* End of Switch: '<S116>/Switch1' */

  /* Switch: '<S118>/Switch3' incorporates:
   *  RelationalOperator: '<S118>/Relational Operator2'
   */
  if ((rtb_ixj == rtb_ixj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l = rtb_ixj;
  }

  /* End of Switch: '<S118>/Switch3' */

  /* Switch: '<S104>/On//Off' incorporates:
   *  Constant: '<S104>/Constant1'
   *  DataStoreRead: '<S7>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Product: '<S104>/Product'
   *  Sum: '<S104>/Add2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l *
      mlParamInterface.param[1] + rtb_Sum1;
  }

  /* End of Switch: '<S104>/On//Off' */

  /* Switch: '<S115>/Switch2' incorporates:
   *  Constant: '<S104>/SaturationLimit'
   *  RelationalOperator: '<S115>/LowerRelop1'
   *  Sum: '<S104>/Add3'
   */
  if (!(AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a > 0.95F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion)) {
    /* Switch: '<S115>/Switch' incorporates:
     *  Constant: '<S104>/SaturationLimit1'
     *  RelationalOperator: '<S115>/UpperRelop'
     *  Sum: '<S104>/Add4'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a < 0.0F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion) {
      rtb_Add1_o = 0.0F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion;
    } else {
      rtb_Add1_o = AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_a;
    }

    /* End of Switch: '<S115>/Switch' */
  }

  /* End of Switch: '<S115>/Switch2' */

  /* Sum: '<S102>/Add1' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_PITC_DT_FF'
   *  Product: '<S102>/Product2'
   */
  rtb_Add1_o = (rtb_Add1_o + AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion)
    + mlParamInterface.param[15] * AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[1];

  /* Sum: '<S102>/Add3' */
  rtb_RhhcosphicoslambXe_cj -= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[1];

  /* Sum: '<S106>/Add3' incorporates:
   *  Constant: '<S106>/SaturationLimit'
   */
  rtb_ixj = 0.401425719F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k;

  /* Switch: '<S110>/Switch3' incorporates:
   *  RelationalOperator: '<S110>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S110>/Switch3' */

  /* Sum: '<S106>/Add1' incorporates:
   *  Constant: '<S106>/delayTime'
   *  DataStoreRead: '<S7>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Delay: '<S106>/NDelays'
   *  Product: '<S106>/Product1'
   *  Product: '<S106>/Product2'
   *  Product: '<S106>/Product3'
   *  Sum: '<S106>/Sum'
   */
  rtb_Sum1 = (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx -
              AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[0]) / 0.05F *
    mlParamInterface.param[5] +
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx *
    mlParamInterface.param[3];

  /* Sum: '<S106>/Sum2' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Gain: '<S106>/Gain'
   *  Memory: '<S106>/Memory1'
   *  Product: '<S106>/Product4'
   */
  rtb_RhhcosphicoslambXe_cj = (0.01F *
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx *
    mlParamInterface.param[4] +
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac) + rtb_Sum1;

  /* Switch: '<S106>/AntiWindup' incorporates:
   *  Constant: '<S106>/Constant5'
   *  Constant: '<S106>/SaturationLimit'
   *  Constant: '<S106>/SaturationLimit1'
   *  Logic: '<S106>/Logical Operator'
   *  RelationalOperator: '<S106>/Relational Operator'
   *  RelationalOperator: '<S106>/Relational Operator1'
   *  Sum: '<S106>/Add3'
   *  Sum: '<S106>/Add4'
   */
  if ((rtb_RhhcosphicoslambXe_cj > -0.401425719F -
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k) &&
      (rtb_RhhcosphicoslambXe_cj < 0.401425719F -
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k)) {
    rtb_RhhcosphicoslambXe_cj =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx;
  } else {
    rtb_RhhcosphicoslambXe_cj = 0.0F;
  }

  /* End of Switch: '<S106>/AntiWindup' */

  /* Switch: '<S125>/Switch3' incorporates:
   *  RelationalOperator: '<S125>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l1 =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S125>/Switch3' */

  /* Switch: '<S124>/Switch1' incorporates:
   *  Constant: '<S124>/Constant'
   *  Constant: '<S124>/Constant1'
   *  Constant: '<S124>/Constant2'
   *  Constant: '<S124>/Constant3'
   *  Constant: '<S124>/Constant5'
   *  Delay: '<S124>/Integer Delay'
   *  Delay: '<S124>/Integer Delay1'
   *  Delay: '<S124>/Integer Delay2'
   *  Product: '<S124>/Product'
   *  Product: '<S124>/Product1'
   *  Product: '<S124>/Product2'
   *  Product: '<S124>/Product3'
   *  Sum: '<S124>/Subtract'
   *  Sum: '<S124>/Subtract1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_RhhcosphicoslambXe_cj = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe_cj =
      ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l1 * 0.333333343F +
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_c * 1.33333337F) +
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_m[0] * 0.333333343F)
      * 0.005F + AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_d;
  }

  /* End of Switch: '<S124>/Switch1' */

  /* Switch: '<S126>/Switch3' incorporates:
   *  RelationalOperator: '<S126>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k0 =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S126>/Switch3' */

  /* Switch: '<S106>/On//Off' incorporates:
   *  Constant: '<S106>/Constant1'
   *  DataStoreRead: '<S7>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Product: '<S106>/Product'
   *  Sum: '<S106>/Add2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k0 *
      mlParamInterface.param[4] + rtb_Sum1;
  }

  /* End of Switch: '<S106>/On//Off' */

  /* Switch: '<S123>/Switch2' incorporates:
   *  Constant: '<S106>/SaturationLimit'
   *  RelationalOperator: '<S123>/LowerRelop1'
   *  Sum: '<S106>/Add3'
   */
  if (!(AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac > 0.401425719F
        - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k)) {
    /* Switch: '<S123>/Switch' incorporates:
     *  Constant: '<S106>/SaturationLimit1'
     *  RelationalOperator: '<S123>/UpperRelop'
     *  Sum: '<S106>/Add4'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac < -0.401425719F
        - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k) {
      rtb_ixj = -0.401425719F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k;
    } else {
      rtb_ixj = AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_ac;
    }

    /* End of Switch: '<S123>/Switch' */
  }

  /* End of Switch: '<S123>/Switch2' */

  /* Saturate: '<S102>/[-60 60]' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n4.y > 60.0F) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u060 = 60.0F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n4.y <
             -60.0F) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u060 = -60.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u060 =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n4.y;
  }

  /* End of Saturate: '<S102>/[-60 60]' */

  /* S-Function (MCHP_C_function_Call): '<S113>/myCos() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myCosapUtilscupdated5116 = myCos(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u060
    );

  /* Sum: '<S102>/Add4' incorporates:
   *  Constant: '<S102>/Constant2'
   *  Constant: '<S102>/Constant4'
   *  DataStoreRead: '<S7>/PAR_PID_HEI_ERR_FF'
   *  Product: '<S102>/Product'
   *  Product: '<S102>/Product1'
   *  Sum: '<S102>/Add5'
   */
  rtb_ixj = (1.0F / AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myCosapUtilscupdated5116 -
             1.0F) * mlParamInterface.param[11] + (rtb_ixj +
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_k);

  /* Update for UnitDelay: '<S140>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S140>/FixPt Constant'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay2_DSTATE = 0U;

  /* Update for Delay: '<S120>/Integer Delay2' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_e0;

  /* Update for Delay: '<S120>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_k =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_me;

  /* Update for Delay: '<S120>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_g[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_g[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_g[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_me;

  /* Update for Delay: '<S104>/NDelays' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[2] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[3];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[3] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[4];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE[4] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_b;

  /* Update for Delay: '<S116>/Integer Delay2' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_j =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l;

  /* Update for Delay: '<S116>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k;

  /* Update for Delay: '<S116>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_f[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_f[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_f[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k;

  /* Update for Delay: '<S106>/NDelays' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[2] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[3];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[3] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[4];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_i[4] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bx;

  /* Update for Delay: '<S124>/Integer Delay2' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_d =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_k0;

  /* Update for Delay: '<S124>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_c =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l1;

  /* Update for Delay: '<S124>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_m[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_m[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_m[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_l1;

  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* S-Function (MCHP_C_function_Call): '<S37>/myCos() aLib.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myCosaLibcupdated5116 = myCos(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0]
    );

  /* Product: '<S30>/Product' */
  rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[5] *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myCosaLibcupdated5116;

  /* S-Function (MCHP_C_function_Call): '<S38>/C Function Call1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.CFunctionCall1 = mySin(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0]
    );

  /* Sum: '<S30>/Subtract' incorporates:
   *  Product: '<S30>/Product1'
   */
  rtb_Sum_b = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[4] *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.CFunctionCall1 + rtb_Sum1;

  /* Switch: '<S36>/Switch3' incorporates:
   *  Delay: '<S36>/Integer Delay3'
   *  RelationalOperator: '<S36>/Relational Operator2'
   */
  if ((rtb_Sum_b == rtb_Sum_b) > 0) {
    rtb_kxi = rtb_Sum_b;
  } else {
    rtb_kxi = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_e;
  }

  /* End of Switch: '<S36>/Switch3' */

  /* DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF' incorporates:
   *  Inport: '<S54>/In1'
   *  Logic: '<S42>/Logical Operator'
   */
  rtb_Deg2R_idx_0 = mlParamInterface.param[21];
  rtb_Deg2R_idx_1 = mlParamInterface.param[22];

  /* Outputs for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  EnablePort: '<S31>/Enable'
   */
  if (mlParamInterface.param[24L] > 0.0F) {
    if (!AUAV3_WITH_SLUGS_SENSOR_A_DWork.L1OutputFeedbackControllerWithP) {
      /* InitializeConditions for UnitDelay: '<S51>/UD'
       *
       * Block description for '<S51>/UD':
       *
       *  Store in Global RAM
       */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_f = 0.0F;

      /* InitializeConditions for Delay: '<S41>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a1 = 0.0F;

      /* InitializeConditions for Delay: '<S41>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_l = 0.0F;

      /* InitializeConditions for Delay: '<S43>/Integer Delay3' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_h = 0.0F;

      /* InitializeConditions for UnitDelay: '<S61>/UD'
       *
       * Block description for '<S61>/UD':
       *
       *  Store in Global RAM
       */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S45>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_f = 0.0F;

      /* InitializeConditions for Delay: '<S45>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_a = 0.0F;

      /* InitializeConditions for Delay: '<S31>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bt = 0.0F;

      /* InitializeConditions for Delay: '<S31>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja = 0.0F;

      /* InitializeConditions for Merge: '<S42>/Merge' */
      if (rtmIsFirstInitCond(AUAV3_WITH_SLUGS_SENSOR_AND__M)) {
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_b = 0.0F;
      }

      /* End of InitializeConditions for Merge: '<S42>/Merge' */

      /* InitializeConditions for Delay: '<S44>/Integer Delay3' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_nf = 0.0F;

      /* InitializeConditions for Delay: '<S46>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_o = 0.0F;

      /* InitializeConditions for Delay: '<S46>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[0] = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[1] = 0.0F;

      /* InitializeConditions for Delay: '<S46>/Integer Delay2' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_l = 0.0F;

      /* InitializeConditions for Delay: '<S62>/Integer Delay3' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_mn = 0.0F;

      /* InitializeConditions for Delay: '<S63>/Integer Delay3' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_aq = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.L1OutputFeedbackControllerWithP = true;
    }

    /* InitialCondition: '<S41>/IC' */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_l) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_l = false;
      rtb_Sum1 = 1.0F;
    } else {
      /* Abs: '<S48>/Abs1' incorporates:
       *  Sum: '<S51>/Diff'
       *  UnitDelay: '<S51>/UD'
       *
       * Block description for '<S51>/Diff':
       *
       *  Add in CPU
       *
       * Block description for '<S51>/UD':
       *
       *  Store in Global RAM
       */
      rtb_Sum1 = (real32_T)fabs(mlParamInterface.param[21] -
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_f);

      /* Saturate: '<S48>/Saturation1' */
      if (rtb_Sum1 > 1.0F) {
        rtb_Sum1 = 1.0F;
      }

      /* End of Saturate: '<S48>/Saturation1' */
    }

    /* End of InitialCondition: '<S41>/IC' */

    /* Outputs for Enabled SubSystem: '<S41>/Compute Coef' incorporates:
     *  EnablePort: '<S47>/Enable'
     */
    if (rtb_Sum1 > 0.0F) {
      /* Gain: '<S47>/-T' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.T_j = -0.01F * mlParamInterface.param[21];

      /* S-Function (MCHP_C_function_Call): '<S49>/myExp() apUtils.c [updated 5.1.16]' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myExpapUtilscupdated5116_f = myExp(
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.T_j
        );

      /* SignalConversion: '<S50>/Numerical Unity' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_f =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myExpapUtilscupdated5116_f;

      /* Sum: '<S47>/1-c' incorporates:
       *  Constant: '<S47>/Constant'
       */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_i = (real32_T)(1.0 -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_f);
    }

    /* End of Outputs for SubSystem: '<S41>/Compute Coef' */

    /* Sum: '<S41>/Subtract' incorporates:
     *  Delay: '<S41>/Integer Delay'
     *  Delay: '<S41>/Integer Delay1'
     *  Product: '<S41>/Divide'
     *  Product: '<S41>/Divide1'
     */
    rtb_Subtract_j5 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_i *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a1 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_f *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_l;

    /* Switch: '<S43>/Switch3' incorporates:
     *  Delay: '<S43>/Integer Delay3'
     *  RelationalOperator: '<S43>/Relational Operator2'
     */
    if ((rtb_kxi == rtb_kxi) > 0) {
      rtb_Switch3_bx = rtb_kxi;
    } else {
      rtb_Switch3_bx = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_h;
    }

    /* End of Switch: '<S43>/Switch3' */

    /* InitialCondition: '<S45>/IC' */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j4) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j4 = false;
      rtb_Sum1 = 1.0F;
    } else {
      /* Abs: '<S58>/Abs1' incorporates:
       *  Sum: '<S61>/Diff'
       *  UnitDelay: '<S61>/UD'
       *
       * Block description for '<S61>/Diff':
       *
       *  Add in CPU
       *
       * Block description for '<S61>/UD':
       *
       *  Store in Global RAM
       */
      rtb_Sum1 = (real32_T)fabs(mlParamInterface.param[22] -
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_i);

      /* Saturate: '<S58>/Saturation1' */
      if (rtb_Sum1 > 1.0F) {
        rtb_Sum1 = 1.0F;
      }

      /* End of Saturate: '<S58>/Saturation1' */
    }

    /* End of InitialCondition: '<S45>/IC' */

    /* Outputs for Enabled SubSystem: '<S45>/Compute Coef' incorporates:
     *  EnablePort: '<S57>/Enable'
     */
    if (rtb_Sum1 > 0.0F) {
      /* Gain: '<S57>/-T' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.T = -0.01F * mlParamInterface.param[22];

      /* S-Function (MCHP_C_function_Call): '<S59>/myExp() apUtils.c [updated 5.1.16]' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myExpapUtilscupdated5116 = myExp(
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.T
        );

      /* SignalConversion: '<S60>/Numerical Unity' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_c =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myExpapUtilscupdated5116;

      /* Sum: '<S57>/1-c' incorporates:
       *  Constant: '<S57>/Constant'
       */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_b = (real32_T)(1.0 -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_c);
    }

    /* End of Outputs for SubSystem: '<S45>/Compute Coef' */

    /* Sum: '<S45>/Subtract' incorporates:
     *  Delay: '<S45>/Integer Delay'
     *  Delay: '<S45>/Integer Delay1'
     *  Product: '<S45>/Divide'
     *  Product: '<S45>/Divide1'
     */
    rtb_Subtract_jz = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_b *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_f +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.NumericalUnity_c *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_a;

    /* Gain: '<S31>/Gain' incorporates:
     *  Sum: '<S31>/Sum3'
     */
    rtb_Sum1 = -(rtb_Subtract_jz - rtb_Switch3_bx);

    /* Product: '<S56>/Divide4' incorporates:
     *  Constant: '<S56>/Constant'
     *  Delay: '<S31>/Integer Delay1'
     */
    rtb_Deg2R1_bc = 0.25F *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja * 2.0F;

    /* Product: '<S56>/Divide3' incorporates:
     *  Delay: '<S31>/Integer Delay1'
     *  Product: '<S56>/Divide'
     *  Sum: '<S56>/Subtract'
     */
    rtb_RhhcosphicoslambXe_cj =
      (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja *
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja - 4.0F) / 4.0F;

    /* Logic: '<S42>/Logical Operator2' incorporates:
     *  Constant: '<S52>/Constant'
     *  Constant: '<S53>/Constant'
     *  Product: '<S42>/Divide2'
     *  RelationalOperator: '<S52>/Compare'
     *  RelationalOperator: '<S53>/Compare'
     */
    rtb_DataTypeConversion2_j = (uint8_T)((rtb_Deg2R1_bc * rtb_Sum1 > 0.0F) &&
      (rtb_RhhcosphicoslambXe_cj >= 0.0F));

    /* Outputs for Enabled SubSystem: '<S42>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S54>/Enable'
     */
    if (!(rtb_DataTypeConversion2_j != 0) > 0) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_b = rtb_Sum1;
    }

    /* End of Outputs for SubSystem: '<S42>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S42>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S55>/Enable'
     */
    if (rtb_DataTypeConversion2_j > 0) {
      /* Signum: '<S55>/Sign' */
      if (rtb_Deg2R1_bc < 0.0F) {
        rtb_Deg2R1_bc = -1.0F;
      } else if (rtb_Deg2R1_bc > 0.0F) {
        rtb_Deg2R1_bc = 1.0F;
      } else {
        if (rtb_Deg2R1_bc == 0.0F) {
          rtb_Deg2R1_bc = 0.0F;
        }
      }

      /* Product: '<S55>/Divide2' incorporates:
       *  Constant: '<S55>/Constant'
       *  Product: '<S55>/Divide1'
       *  Signum: '<S55>/Sign'
       *  Sum: '<S55>/Subtract'
       */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_b = (1.0F -
        rtb_RhhcosphicoslambXe_cj * rtb_Deg2R1_bc) * rtb_Sum1;
    }

    /* End of Outputs for SubSystem: '<S42>/Enabled Subsystem1' */

    /* Product: '<S31>/Projection' incorporates:
     *  Inport: '<S54>/In1'
     *  Logic: '<S42>/Logical Operator'
     */
    rtb_Projection = mlParamInterface.param[23] *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_b;

    /* Sum: '<S31>/Sum4' incorporates:
     *  Delay: '<S31>/Integer Delay'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bt - rtb_Subtract_j5;

    /* Saturate: '<S31>/Psi Dot  Limit' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit > 1.0F) {
      /* Sum: '<S31>/Sum4' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit = 1.0F;
    } else {
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit < -1.0F) {
        /* Sum: '<S31>/Sum4' */
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit = -1.0F;
      }
    }

    /* End of Saturate: '<S31>/Psi Dot  Limit' */

    /* Sum: '<S31>/Sum2' incorporates:
     *  Delay: '<S31>/Integer Delay1'
     */
    rtb_Switch1_h = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja;

    /* Switch: '<S44>/Switch3' incorporates:
     *  Delay: '<S44>/Integer Delay3'
     *  RelationalOperator: '<S44>/Relational Operator2'
     */
    if ((rtb_Switch1_h == rtb_Switch1_h) > 0) {
      rtb_Switch3_fv = rtb_Switch1_h;
    } else {
      rtb_Switch3_fv = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_nf;
    }

    /* End of Switch: '<S44>/Switch3' */

    /* Switch: '<S62>/Switch3' incorporates:
     *  Delay: '<S62>/Integer Delay3'
     *  RelationalOperator: '<S62>/Relational Operator2'
     */
    if (!((rtb_Projection == rtb_Projection) > 0)) {
      rtb_Projection = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_mn;
    }

    /* End of Switch: '<S62>/Switch3' */

    /* Switch: '<S46>/Switch1' incorporates:
     *  Constant: '<S46>/Constant5'
     *  Logic: '<S31>/Logical Operator'
     */
    if (!(mlParamInterface.param[24] != 0.0F) > 0) {
      rtb_Switch1_h = 0.0F;
    } else {
      /* Sum: '<S46>/Subtract1' incorporates:
       *  Constant: '<S46>/Constant'
       *  Constant: '<S46>/Constant1'
       *  Constant: '<S46>/Constant2'
       *  Constant: '<S46>/Constant3'
       *  Delay: '<S46>/Integer Delay'
       *  Delay: '<S46>/Integer Delay1'
       *  Delay: '<S46>/Integer Delay2'
       *  Product: '<S46>/Product'
       *  Product: '<S46>/Product1'
       *  Product: '<S46>/Product2'
       *  Product: '<S46>/Product3'
       *  Sum: '<S46>/Subtract'
       */
      rtb_Switch1_h = ((rtb_Projection * 0.333333343F +
                        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_o *
                        1.33333337F) +
                       AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[0]
                       * 0.333333343F) * 0.005F +
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_l;

      /* Saturate: '<S46>/[min max]' */
      if (rtb_Switch1_h > 2.0F) {
        rtb_Switch1_h = 2.0F;
      } else {
        if (rtb_Switch1_h < -2.0F) {
          rtb_Switch1_h = -2.0F;
        }
      }

      /* End of Saturate: '<S46>/[min max]' */
    }

    /* End of Switch: '<S46>/Switch1' */

    /* Switch: '<S63>/Switch3' incorporates:
     *  Delay: '<S63>/Integer Delay3'
     *  RelationalOperator: '<S63>/Relational Operator2'
     */
    if (!((rtb_Switch1_h == rtb_Switch1_h) > 0)) {
      rtb_Switch1_h = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_aq;
    }

    /* End of Switch: '<S63>/Switch3' */
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.L1OutputFeedbackControllerWithP) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.L1OutputFeedbackControllerWithP = false;
    }
  }

  /* End of Outputs for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */

  /* Product: '<S18>/Product2' incorporates:
   *  DataTypeConversion: '<S567>/Data Type Conversion3'
   *  Sum: '<S18>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_MathFunction_0[i] = rtb_MathFunction[i + 6] * (real32_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[2] + (rtb_MathFunction[i +
      3] * (real32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[1] +
      rtb_MathFunction[i] * (real32_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_h.y[0]);
  }

  /* End of Product: '<S18>/Product2' */

  /* Product: '<S18>/Product3' incorporates:
   *  Sum: '<S18>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_Product5[i] = rtb_MathFunction[i + 6] * rtb_kxj + (rtb_MathFunction[i +
      3] * rtb_DataTypeConversion1_jq_id_0 + rtb_MathFunction[i] *
      rtb_DataTypeConversion1_jq_idx_);
  }

  /* End of Product: '<S18>/Product3' */

  /* Sum: '<S18>/Sum' */
  rtb_Sum1_ao[1] = rtb_MathFunction_0[1] + rtb_Product5[1];

  /* Outputs for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Switch: '<S98>/Switch3' incorporates:
   *  RelationalOperator: '<S98>/Relational Operator2'
   */
  if ((rtb_BankLimitCommand == rtb_BankLimitCommand) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_dj =
      rtb_BankLimitCommand;
  }

  /* End of Switch: '<S98>/Switch3' */

  /* Saturate: '<S97>/bank Limit' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_dj > 0.436332315F) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit = 0.436332315F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_dj <
             -0.436332315F) {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit = -0.436332315F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_dj;
  }

  /* End of Saturate: '<S97>/bank Limit' */

  /* S-Function (MCHP_C_function_Call): '<S100>/myTan() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_a = myTan(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit
    );

  /* Switch: '<S99>/Switch3' incorporates:
   *  RelationalOperator: '<S99>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe_h == rtb_RhhcosphisinlambYe_h) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bn =
      rtb_RhhcosphisinlambYe_h;
  }

  /* End of Switch: '<S99>/Switch3' */

  /* Switch: '<S65>/Switch1' incorporates:
   *  Constant: '<S97>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S97>/Divide'
   *  Product: '<S97>/Divide1'
   *  Saturate: '<S97>/[0 40]'
   */
  if (rtb_IC1_l) {
    rtb_kxj = mlMidLevelCommands.rCommand;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bn > 40.0F) {
      /* Saturate: '<S97>/[0 40]' */
      rtb_Sum1 = 40.0F;
    } else if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bn < 0.0F) {
      /* Saturate: '<S97>/[0 40]' */
      rtb_Sum1 = 0.0F;
    } else {
      /* Saturate: '<S97>/[0 40]' */
      rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_bn;
    }

    rtb_kxj = 1.0F / rtb_Sum1 * 9.80665F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_a;
  }

  /* End of Switch: '<S65>/Switch1' */

  /* Switch: '<S65>/Switch' incorporates:
   *  DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF'
   */
  if (mlParamInterface.param[24] > 0.3F) {
    rtb_Deg2R1_bc = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PsiDotLimit;
  } else {
    rtb_Deg2R1_bc = rtb_kxj;
  }

  /* End of Switch: '<S65>/Switch' */

  /* Saturate: '<S64>/Psi Dot  Limit' */
  if (rtb_Deg2R1_bc > 1.0F) {
    rtb_Deg2R1_bc = 1.0F;
  } else {
    if (rtb_Deg2R1_bc < -1.0F) {
      rtb_Deg2R1_bc = -1.0F;
    }
  }

  /* End of Saturate: '<S64>/Psi Dot  Limit' */

  /* Outputs for Enabled SubSystem: '<S64>/Sideslip Compensation' incorporates:
   *  EnablePort: '<S71>/Enable'
   */
  /* DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON' */
  if (mlParamInterface.param[20L] > 0.0F) {
    if (!AUAV3_WITH_SLUGS_SENSOR_A_DWork.SideslipCompensation_MODE) {
      /* InitializeConditions for UnitDelay: '<S88>/UD'
       *
       * Block description for '<S88>/UD':
       *
       *  Store in Global RAM
       */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_b = 0.0F;

      /* InitializeConditions for Delay: '<S80>/Integer Delay' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p2 = 0.0F;

      /* InitializeConditions for Delay: '<S80>/Integer Delay1' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j = 0.0F;

      /* InitializeConditions for Delay: '<S81>/Integer Delay3' */
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_a = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.SideslipCompensation_MODE = true;
    }

    /* MATLAB Function: '<S79>/negprotect' */
    AUAV3_WITH_SLUG_negprotect(rtb_RhhcosphisinlambYe_h,
      &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_k);

    /* Saturate: '<S79>/bank Limit' */
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0] > 0.436332315F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit_l = 0.436332315F;
    } else if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0] < -0.436332315F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit_l = -0.436332315F;
    } else {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit_l =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0];
    }

    /* End of Saturate: '<S79>/bank Limit' */

    /* S-Function (MCHP_C_function_Call): '<S83>/myTan() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_f = myTan(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.bankLimit_l
      );

    /* InitialCondition: '<S80>/IC' */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_f) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_f = false;
      rtb_Sum1 = 1.0F;
    } else {
      /* Abs: '<S87>/Abs1' incorporates:
       *  Constant: '<S71>/Constant'
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
      rtb_Sum1 = (real32_T)fabs(0.3F -
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_b);

      /* Saturate: '<S87>/Saturation1' */
      if (rtb_Sum1 > 1.0F) {
        rtb_Sum1 = 1.0F;
      }

      /* End of Saturate: '<S87>/Saturation1' */
    }

    /* End of InitialCondition: '<S80>/IC' */

    /* Outputs for Enabled SubSystem: '<S80>/Compute Coef' incorporates:
     *  EnablePort: '<S86>/Enable'
     */
    if (rtb_Sum1 > 0.0F) {
      /* Math: '<S86>/Math Function' incorporates:
       *  Constant: '<S71>/Constant'
       *  Gain: '<S86>/-T'
       *
       * About '<S86>/Math Function':
       *  Operator: exp
       */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c = 0.997004509F;

      /* Sum: '<S86>/1-c' incorporates:
       *  Constant: '<S86>/Constant'
       */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_d = (real32_T)(1.0 -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c);
    }

    /* End of Outputs for SubSystem: '<S80>/Compute Coef' */

    /* Sum: '<S80>/Subtract' incorporates:
     *  Delay: '<S80>/Integer Delay'
     *  Delay: '<S80>/Integer Delay1'
     *  Product: '<S80>/Divide'
     *  Product: '<S80>/Divide1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Subtract =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c_d *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p2 +
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.c *
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j;

    /* Product: '<S82>/Divide' incorporates:
     *  Constant: '<S79>/Constant1'
     *  Constant: '<S82>/Constant1'
     *  Product: '<S79>/Divide'
     *  Product: '<S79>/Divide1'
     *  Sum: '<S71>/Subtract'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Divide_d = (1.0F /
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_negprotect_k.zpVal * 9.80665F *
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_f -
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[5]) * rtb_RhhcosphisinlambYe_h /
      9.80665F;

    /* S-Function (MCHP_C_function_Call): '<S89>/myTan() apUtils.c [updated 5.1.16]' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_h = myTan(
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Divide_d
      );

    /* Switch: '<S81>/Switch3' incorporates:
     *  RelationalOperator: '<S81>/Relational Operator2'
     */
    if ((AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_h ==
         AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_h) > 0) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_a =
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_h;
    }

    /* End of Switch: '<S81>/Switch3' */

    /* Update for UnitDelay: '<S88>/UD' incorporates:
     *  Constant: '<S71>/Constant'
     *
     * Block description for '<S88>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_b = 0.3F;

    /* Update for Delay: '<S80>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p2 =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_a;

    /* Update for Delay: '<S80>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Subtract;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.SideslipCompensation_MODE) {
      /* Disable for Outport: '<S71>/bankComp' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Subtract = 0.0F;
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.SideslipCompensation_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S64>/Sideslip Compensation' */

  /* Product: '<S73>/Divide' incorporates:
   *  Constant: '<S73>/Constant1'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Divide = rtb_Deg2R1_bc *
    rtb_RhhcosphisinlambYe_h / 9.80665F;

  /* S-Function (MCHP_C_function_Call): '<S95>/myTan() apUtils.c [updated 5.1.16]' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_i = myTan(
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Divide
    );

  /* Sum: '<S64>/Add2' */
  rtb_RhhcosphicoslambXe_cj = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Subtract +
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.myTanapUtilscupdated5116_i;

  /* Saturate: '<S64>/Bank  Limit Command' */
  if (rtb_RhhcosphicoslambXe_cj > 0.436332315F) {
    rtb_RhhcosphicoslambXe_cj = 0.436332315F;
  } else {
    if (rtb_RhhcosphicoslambXe_cj < -0.436332315F) {
      rtb_RhhcosphicoslambXe_cj = -0.436332315F;
    }
  }

  /* End of Saturate: '<S64>/Bank  Limit Command' */

  /* Switch: '<S69>/Switch3' incorporates:
   *  RelationalOperator: '<S69>/Relational Operator2'
   */
  if ((rtb_Sum1_ao[1] == rtb_Sum1_ao[1]) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_f = rtb_Sum1_ao[1];
  }

  /* End of Switch: '<S69>/Switch3' */

  /* Saturate: '<S64>/[-20 20]' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_f > 20.0F) {
    rtb_Sum1 = 20.0F;
  } else if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_f < -20.0F) {
    rtb_Sum1 = -20.0F;
  } else {
    rtb_Sum1 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_f;
  }

  /* MATLAB Function: '<S66>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S66>/Constant'
   *  Constant: '<S66>/Constant1'
   *  Saturate: '<S64>/[-20 20]'
   */
  AUA_EmbeddedMATLABFunction(rtb_Sum1, 0.01, 10.0,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n0);

  /* DataStoreWrite: '<S32>/mlNavigation' incorporates:
   *  Gain: '<S64>/Neg Feedback '
   */
  mlNavigation.psiDot_c = rtb_Deg2R1_bc;
  mlNavigation.phi_c = rtb_RhhcosphicoslambXe_cj;
  mlNavigation.ay_body =
    -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y;

  /* Sum: '<S72>/Add3' incorporates:
   *  Constant: '<S72>/SaturationLimit'
   */
  rtb_jxi = 0.17453292F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j;

  /* Sum: '<S72>/Add1' incorporates:
   *  Constant: '<S72>/delayTime'
   *  DataStoreRead: '<S7>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Delay: '<S72>/NDelays'
   *  Gain: '<S64>/Neg Feedback '
   *  Product: '<S72>/Product1'
   *  Product: '<S72>/Product2'
   *  Product: '<S72>/Product3'
   *  Sum: '<S72>/Sum'
   */
  rtb_Sum1 = (-AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y -
              AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[0]) / 0.05F *
    mlParamInterface.param[14] +
    -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y *
    mlParamInterface.param[12];

  /* Sum: '<S72>/Sum2' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Gain: '<S64>/Neg Feedback '
   *  Gain: '<S72>/Gain'
   *  Memory: '<S72>/Memory1'
   *  Product: '<S72>/Product4'
   */
  rtb_Deg2R1_bc = (0.01F *
                   -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y
                   * mlParamInterface.param[13] +
                   AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b) +
    rtb_Sum1;

  /* Switch: '<S72>/AntiWindup' incorporates:
   *  Constant: '<S72>/Constant5'
   *  Constant: '<S72>/SaturationLimit'
   *  Constant: '<S72>/SaturationLimit1'
   *  Gain: '<S64>/Neg Feedback '
   *  Logic: '<S72>/Logical Operator'
   *  RelationalOperator: '<S72>/Relational Operator'
   *  RelationalOperator: '<S72>/Relational Operator1'
   *  Sum: '<S72>/Add3'
   *  Sum: '<S72>/Add4'
   */
  if ((rtb_Deg2R1_bc > -0.17453292F -
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j) && (rtb_Deg2R1_bc <
       0.17453292F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j)) {
    rtb_Deg2R1_bc =
      -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y;
  } else {
    rtb_Deg2R1_bc = 0.0F;
  }

  /* End of Switch: '<S72>/AntiWindup' */

  /* Switch: '<S93>/Switch3' incorporates:
   *  RelationalOperator: '<S93>/Relational Operator2'
   */
  if ((rtb_Deg2R1_bc == rtb_Deg2R1_bc) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_n = rtb_Deg2R1_bc;
  }

  /* End of Switch: '<S93>/Switch3' */

  /* Switch: '<S92>/Switch1' incorporates:
   *  Constant: '<S92>/Constant'
   *  Constant: '<S92>/Constant1'
   *  Constant: '<S92>/Constant2'
   *  Constant: '<S92>/Constant3'
   *  Constant: '<S92>/Constant5'
   *  Delay: '<S92>/Integer Delay'
   *  Delay: '<S92>/Integer Delay1'
   *  Delay: '<S92>/Integer Delay2'
   *  Product: '<S92>/Product'
   *  Product: '<S92>/Product1'
   *  Product: '<S92>/Product2'
   *  Product: '<S92>/Product3'
   *  Sum: '<S92>/Subtract'
   *  Sum: '<S92>/Subtract1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_RhhcosphisinlambYe_h = 0.0F;
  } else {
    rtb_RhhcosphisinlambYe_h =
      ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_n * 0.333333343F +
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_cf * 1.33333337F) +
       AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_gh[0] * 0.333333343F)
      * 0.005F + AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_f;
  }

  /* End of Switch: '<S92>/Switch1' */

  /* Switch: '<S94>/Switch3' incorporates:
   *  RelationalOperator: '<S94>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe_h == rtb_RhhcosphisinlambYe_h) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_j =
      rtb_RhhcosphisinlambYe_h;
  }

  /* End of Switch: '<S94>/Switch3' */

  /* Switch: '<S72>/On//Off' incorporates:
   *  Constant: '<S72>/Constant1'
   *  DataStoreRead: '<S7>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Product: '<S72>/Product'
   *  Sum: '<S72>/Add2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_j *
      mlParamInterface.param[13] + rtb_Sum1;
  }

  /* End of Switch: '<S72>/On//Off' */

  /* Switch: '<S91>/Switch2' incorporates:
   *  Constant: '<S72>/SaturationLimit'
   *  RelationalOperator: '<S91>/LowerRelop1'
   *  Sum: '<S72>/Add3'
   */
  if (!(AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b > 0.17453292F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j)) {
    /* Switch: '<S91>/Switch' incorporates:
     *  Constant: '<S72>/SaturationLimit1'
     *  RelationalOperator: '<S91>/UpperRelop'
     *  Sum: '<S72>/Add4'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b < -0.17453292F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j) {
      rtb_jxi = -0.17453292F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j;
    } else {
      rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_b;
    }

    /* End of Switch: '<S91>/Switch' */
  }

  /* End of Switch: '<S91>/Switch2' */

  /* Sum: '<S64>/Add' */
  rtb_Deg2R1_bc = rtb_jxi + AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_j;

  /* Sum: '<S64>/Add1' */
  rtb_RhhcosphicoslambXe_cj -= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y[0];

  /* Sum: '<S70>/Add3' incorporates:
   *  Constant: '<S70>/SaturationLimit'
   */
  rtb_RhhcosphisinlambYe_h = 0.383972436F -
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d;

  /* Switch: '<S68>/Switch3' incorporates:
   *  RelationalOperator: '<S68>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe_cj == rtb_RhhcosphicoslambXe_cj) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p =
      rtb_RhhcosphicoslambXe_cj;
  }

  /* End of Switch: '<S68>/Switch3' */

  /* Sum: '<S70>/Add1' incorporates:
   *  Constant: '<S70>/delayTime'
   *  DataStoreRead: '<S7>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Delay: '<S70>/NDelays'
   *  Product: '<S70>/Product1'
   *  Product: '<S70>/Product2'
   *  Product: '<S70>/Product3'
   *  Sum: '<S70>/Sum'
   */
  rtb_Sum1 = (AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p -
              AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[0]) / 0.05F *
    mlParamInterface.param[8] +
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p *
    mlParamInterface.param[6];

  /* Sum: '<S70>/Sum2' incorporates:
   *  DataStoreRead: '<S7>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Gain: '<S70>/Gain'
   *  Memory: '<S70>/Memory1'
   *  Product: '<S70>/Product4'
   */
  rtb_jxi = (0.01F * AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p *
             mlParamInterface.param[7] +
             AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l) + rtb_Sum1;

  /* Switch: '<S70>/AntiWindup' incorporates:
   *  Constant: '<S70>/Constant5'
   *  Constant: '<S70>/SaturationLimit'
   *  Constant: '<S70>/SaturationLimit1'
   *  Logic: '<S70>/Logical Operator'
   *  RelationalOperator: '<S70>/Relational Operator'
   *  RelationalOperator: '<S70>/Relational Operator1'
   *  Sum: '<S70>/Add3'
   *  Sum: '<S70>/Add4'
   */
  if ((rtb_jxi > -0.383972436F -
       AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d) && (rtb_jxi <
       0.383972436F - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d)) {
    rtb_jxi = AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p;
  } else {
    rtb_jxi = 0.0F;
  }

  /* End of Switch: '<S70>/AntiWindup' */

  /* Switch: '<S77>/Switch3' incorporates:
   *  RelationalOperator: '<S77>/Relational Operator2'
   */
  if ((rtb_jxi == rtb_jxi) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_ny = rtb_jxi;
  }

  /* End of Switch: '<S77>/Switch3' */

  /* Switch: '<S76>/Switch1' incorporates:
   *  Constant: '<S76>/Constant'
   *  Constant: '<S76>/Constant1'
   *  Constant: '<S76>/Constant2'
   *  Constant: '<S76>/Constant3'
   *  Constant: '<S76>/Constant5'
   *  Delay: '<S76>/Integer Delay'
   *  Delay: '<S76>/Integer Delay1'
   *  Delay: '<S76>/Integer Delay2'
   *  Product: '<S76>/Product'
   *  Product: '<S76>/Product1'
   *  Product: '<S76>/Product2'
   *  Product: '<S76>/Product3'
   *  Sum: '<S76>/Subtract'
   *  Sum: '<S76>/Subtract1'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_jxi = 0.0F;
  } else {
    rtb_jxi = ((AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_ny *
                0.333333343F +
                AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_b *
                1.33333337F) +
               AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_o[0] *
               0.333333343F) * 0.005F +
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_dm;
  }

  /* End of Switch: '<S76>/Switch1' */

  /* Switch: '<S78>/Switch3' incorporates:
   *  RelationalOperator: '<S78>/Relational Operator2'
   */
  if ((rtb_jxi == rtb_jxi) > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_jy = rtb_jxi;
  }

  /* End of Switch: '<S78>/Switch3' */

  /* Switch: '<S70>/On//Off' incorporates:
   *  Constant: '<S70>/Constant1'
   *  DataStoreRead: '<S7>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Product: '<S70>/Product'
   *  Sum: '<S70>/Add2'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l = 0.0F;
  } else {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_jy *
      mlParamInterface.param[7] + rtb_Sum1;
  }

  /* End of Switch: '<S70>/On//Off' */

  /* Switch: '<S75>/Switch2' incorporates:
   *  Constant: '<S70>/SaturationLimit'
   *  RelationalOperator: '<S75>/LowerRelop1'
   *  Sum: '<S70>/Add3'
   */
  if (!(AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l > 0.383972436F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d)) {
    /* Switch: '<S75>/Switch' incorporates:
     *  Constant: '<S70>/SaturationLimit1'
     *  RelationalOperator: '<S75>/UpperRelop'
     *  Sum: '<S70>/Add4'
     */
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l < -0.383972436F
        - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d) {
      rtb_RhhcosphisinlambYe_h = -0.383972436F -
        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d;
    } else {
      rtb_RhhcosphisinlambYe_h =
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.Memory1_PreviousInput_l;
    }

    /* End of Switch: '<S75>/Switch' */
  }

  /* End of Switch: '<S75>/Switch2' */

  /* Sum: '<S64>/Add3' */
  rtb_RhhcosphisinlambYe_h +=
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion_d;

  /* Saturate: '<S64>/Aileron Limit' */
  if (rtb_RhhcosphisinlambYe_h > 0.383972436F) {
    rtb_Sum1 = 0.383972436F;
  } else if (rtb_RhhcosphisinlambYe_h < -0.383972436F) {
    rtb_Sum1 = -0.383972436F;
  } else {
    rtb_Sum1 = rtb_RhhcosphisinlambYe_h;
  }

  /* End of Saturate: '<S64>/Aileron Limit' */

  /* Switch: '<S64>/Switch2' incorporates:
   *  Constant: '<S64>/Constant'
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   *  Product: '<S64>/Product'
   */
  if (mlParamInterface.param[26L] > 0.3F) {
    rtb_jxi = mlParamInterface.param[27] * rtb_Sum1;
  } else {
    rtb_jxi = 0.0F;
  }

  /* End of Switch: '<S64>/Switch2' */

  /* Sum: '<S64>/Add4' */
  rtb_RhhcosphisinlambYe_h = rtb_jxi + rtb_Deg2R1_bc;

  /* Update for Delay: '<S72>/NDelays' incorporates:
   *  Gain: '<S64>/Neg Feedback '
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[2] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[3];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[3] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[4];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_l[4] =
    -AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_n0.y;

  /* Update for Delay: '<S92>/Integer Delay2' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_f =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_j;

  /* Update for Delay: '<S92>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_cf =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S92>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_gh[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_gh[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_gh[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S70>/NDelays' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[2] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[3];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[3] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[4];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.NDelays_DSTATE_h[4] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_p;

  /* Update for Delay: '<S76>/Integer Delay2' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_dm =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_jy;

  /* Update for Delay: '<S76>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_b =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_ny;

  /* Update for Delay: '<S76>/Integer Delay1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_o[0] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_o[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_o[1] =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_ny;

  /* End of Outputs for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S102>/Throttle  Limit' */
  /* MATLAB Function 'myMux Fun1': '<S19>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S19>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
  if (rtb_Add1_o > 0.95F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_Add1_o = 0.95F;
  } else {
    if (rtb_Add1_o < 0.0F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_Add1_o = 0.0F;
    }
  }

  /* End of Saturate: '<S102>/Throttle  Limit' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S64>/Rudder Limit' */
  if (rtb_RhhcosphisinlambYe_h > 0.17453292F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_RhhcosphisinlambYe_h = 0.17453292F;
  } else {
    if (rtb_RhhcosphisinlambYe_h < -0.17453292F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_RhhcosphisinlambYe_h = -0.17453292F;
    }
  }

  /* End of Saturate: '<S64>/Rudder Limit' */
  /* End of Outputs for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S102>/Elevator  Limit' */
  if (rtb_ixj > 0.401425719F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_ixj = 0.401425719F;
  } else {
    if (rtb_ixj < -0.401425719F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_ixj = -0.401425719F;
    }
  }

  /* End of Saturate: '<S102>/Elevator  Limit' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* If: '<S656>/If' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ManualorAutonavSupportcupdated4 == 1) {
    /* Outputs for IfAction SubSystem: '<S656>/If  Control Type Is Manual' incorporates:
     *  ActionPort: '<S657>/Action Port'
     */
    /* Gain: '<S657>/Gain4' incorporates:
     *  SignalConversion: '<S657>/TmpSignal ConversionAtGain4Inport1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[0] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[1] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[2] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[3] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[3] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[2] >> 1;

    /* End of Outputs for SubSystem: '<S656>/If  Control Type Is Manual' */
  } else if ((rtb_DataTypeConversion_gm == 3) || (rtb_DataTypeConversion_gm == 4)
             || (rtb_DataTypeConversion_gm == 9) || (rtb_DataTypeConversion_gm ==
              10)) {
    /* Outputs for IfAction SubSystem: '<S656>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' incorporates:
     *  ActionPort: '<S660>/Action Port'
     */
    /* DataTypeConversion: '<S667>/Data Type Conversion' incorporates:
     *  Constant: '<S667>/Constant1'
     *  Constant: '<S667>/Constant2'
     *  Product: '<S667>/Divide'
     *  Sum: '<S667>/Add'
     */
    tmp_7 = floor(rtb_Add1_o * 1274.0 + 3176.0);

    /* DataTypeConversion: '<S668>/Data Type Conversion' incorporates:
     *  Constant: '<S668>/Constant1'
     *  Constant: '<S668>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S668>/Divide'
     *  Sum: '<S668>/Add'
     */
    tmp_4 = floor(rtb_Sum1 * -4679.1553269017213 + 3683.3333333333344);

    /* DataTypeConversion: '<S669>/Data Type Conversion' incorporates:
     *  Constant: '<S669>/Constant1'
     *  Constant: '<S669>/Constant2'
     *  Product: '<S669>/Divide'
     *  Sum: '<S669>/Add'
     */
    tmp_5 = floor(rtb_RhhcosphisinlambYe_h * -2900.5988378497932 +
                  3725.0000000000009);

    /* DataTypeConversion: '<S666>/Data Type Conversion' incorporates:
     *  Constant: '<S666>/Constant1'
     *  Constant: '<S666>/Constant2'
     *  Product: '<S666>/Divide'
     *  Sum: '<S666>/Add'
     */
    tmp_6 = floor(rtb_ixj * 6016.0568488736444 + 3766.6666666666679);

    /* DataTypeConversion: '<S667>/Data Type Conversion' */
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    /* DataTypeConversion: '<S668>/Data Type Conversion' */
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      tmp_4 = 0.0;
    } else {
      tmp_4 = fmod(tmp_4, 65536.0);
    }

    /* DataTypeConversion: '<S669>/Data Type Conversion' */
    if (rtIsNaN(tmp_5) || rtIsInf(tmp_5)) {
      tmp_5 = 0.0;
    } else {
      tmp_5 = fmod(tmp_5, 65536.0);
    }

    /* DataTypeConversion: '<S666>/Data Type Conversion' */
    if (rtIsNaN(tmp_6) || rtIsInf(tmp_6)) {
      tmp_6 = 0.0;
    } else {
      tmp_6 = fmod(tmp_6, 65536.0);
    }

    /* MATLAB Function: '<S660>/myMux Fun1' incorporates:
     *  DataTypeConversion: '<S666>/Data Type Conversion'
     *  DataTypeConversion: '<S667>/Data Type Conversion'
     *  DataTypeConversion: '<S668>/Data Type Conversion'
     *  DataTypeConversion: '<S669>/Data Type Conversion'
     */
    AUAV3_WITH_SLU_myMuxFun1_c((uint16_T)tmp_7, (uint16_T)tmp_4, (uint16_T)tmp_5,
      (uint16_T)tmp_6, AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e);

    /* End of Outputs for SubSystem: '<S656>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' */
  } else if (rtb_DataTypeConversion_gm == 2) {
    /* Outputs for IfAction SubSystem: '<S656>/If  Control Type Is Passthrough' incorporates:
     *  ActionPort: '<S658>/Action Port'
     */
    /* Gain: '<S658>/Gain4' incorporates:
     *  SignalConversion: '<S658>/TmpSignal ConversionAtGain4Inport1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[0] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[1] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[2] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[3] >> 1;
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[3] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[2] >> 1;

    /* End of Outputs for SubSystem: '<S656>/If  Control Type Is Passthrough' */
  } else {
    if (rtb_DataTypeConversion_gm == 8) {
      /* Switch: '<S659>/Switch' incorporates:
       *  Gain: '<S659>/Gain'
       */
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InitializeControlMCU[3] >= 1) {
        rtb_u2deg = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[0] >> 1;
      } else {
        /* DataTypeConversion: '<S662>/Data Type Conversion' incorporates:
         *  Constant: '<S662>/Constant1'
         *  Constant: '<S662>/Constant2'
         *  Product: '<S662>/Divide'
         *  Sum: '<S662>/Add'
         */
        tmp_7 = floor(rtb_Add1_o * 1274.0 + 3176.0);
        if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
          rtb_u2deg = 0U;
        } else {
          rtb_u2deg = (uint16_T)fmod(tmp_7, 65536.0);
        }

        /* End of DataTypeConversion: '<S662>/Data Type Conversion' */
      }

      /* End of Switch: '<S659>/Switch' */

      /* Switch: '<S659>/Switch1' incorporates:
       *  Gain: '<S659>/Gain1'
       */
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InitializeControlMCU[2] >= 1) {
        rtb_Switch1_p = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[1] >> 1;
      } else {
        /* DataTypeConversion: '<S663>/Data Type Conversion' incorporates:
         *  Constant: '<S663>/Constant1'
         *  Constant: '<S663>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S663>/Divide'
         *  Sum: '<S663>/Add'
         */
        tmp_7 = floor(rtb_Sum1 * -4679.1553269017213 + 3683.3333333333344);
        if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
          rtb_Switch1_p = 0U;
        } else {
          rtb_Switch1_p = (uint16_T)fmod(tmp_7, 65536.0);
        }

        /* End of DataTypeConversion: '<S663>/Data Type Conversion' */
      }

      /* End of Switch: '<S659>/Switch1' */

      /* Switch: '<S659>/Switch2' incorporates:
       *  Gain: '<S659>/Gain2'
       */
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InitializeControlMCU[1] >= 1) {
        rtb_Switch2 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[3] >> 1;
      } else {
        /* DataTypeConversion: '<S664>/Data Type Conversion' incorporates:
         *  Constant: '<S664>/Constant1'
         *  Constant: '<S664>/Constant2'
         *  Product: '<S664>/Divide'
         *  Sum: '<S664>/Add'
         */
        tmp_7 = floor(rtb_RhhcosphisinlambYe_h * -2900.5988378497932 +
                      3725.0000000000009);
        if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
          rtb_Switch2 = 0U;
        } else {
          rtb_Switch2 = (uint16_T)fmod(tmp_7, 65536.0);
        }

        /* End of DataTypeConversion: '<S664>/Data Type Conversion' */
      }

      /* End of Switch: '<S659>/Switch2' */

      /* Switch: '<S659>/Switch3' incorporates:
       *  Gain: '<S659>/Gain3'
       */
      if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InitializeControlMCU[0] >= 1) {
        rtb_Switch3_bc = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5.y[2] >> 1;
      } else {
        /* DataTypeConversion: '<S661>/Data Type Conversion' incorporates:
         *  Constant: '<S661>/Constant1'
         *  Constant: '<S661>/Constant2'
         *  Product: '<S661>/Divide'
         *  Sum: '<S661>/Add'
         */
        tmp_7 = floor(rtb_ixj * 6016.0568488736444 + 3766.6666666666679);
        if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
          rtb_Switch3_bc = 0U;
        } else {
          rtb_Switch3_bc = (uint16_T)fmod(tmp_7, 65536.0);
        }

        /* End of DataTypeConversion: '<S661>/Data Type Conversion' */
      }

      /* End of Switch: '<S659>/Switch3' */

      /* MATLAB Function: '<S659>/myMux Fun1' */
      AUAV3_WITH_SLU_myMuxFun1_c(rtb_u2deg, rtb_Switch1_p, rtb_Switch2,
        rtb_Switch3_bc, AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e);

      /* End of Outputs for SubSystem: '<S656>/If  Control Type Is Selective Passthrough' */
    }
  }

  /* End of If: '<S656>/If' */

  /* DataStoreWrite: '<S17>/mlNavigation' incorporates:
   *  DataTypeConversion: '<S655>/Data Type Conversion'
   *  Gain: '<S655>/Convert to  Microseconds'
   */
  mlServoOutputRaw.servo1_raw = (uint16_T)(52429UL *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[0] >> 17);
  mlServoOutputRaw.servo2_raw = (uint16_T)(52429UL *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[1] >> 17);
  mlServoOutputRaw.servo3_raw = (uint16_T)(52429UL *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[2] >> 17);
  mlServoOutputRaw.servo4_raw = (uint16_T)(52429UL *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[3] >> 17);

  /* DataStoreWrite: '<S17>/mlAttitudeEuler' incorporates:
   *  MATLAB Function: '<S18>/myMux Fun1'
   */
  mlAttitude.roll = rtb_Merge_p[2];
  mlAttitude.pitch = rtb_Merge_p[1];
  mlAttitude.yaw =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_be.y;

  /* DataStoreWrite: '<S17>/mlAttitudePQR' */
  mlAttitude.rollspeed = rtb_u11[0];
  mlAttitude.pitchspeed = rtb_u11[1];
  mlAttitude.yawspeed = rtb_u11[2];

  /* MATLAB Function: '<S639>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S639>/Constant'
   *  Constant: '<S639>/Constant1'
   *  DataTypeConversion: '<S630>/Data Type Conversion2'
   */
  AUA_EmbeddedMATLABFunction((real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdPITCHraw, 0.01, 0.1,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fz,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fz);

  /* DataTypeConversion: '<S630>/Data Type Conversion1' */
  rtb_jxi = (real32_T)floor
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fz.y);
  if (rtIsNaNF(rtb_jxi) || rtIsInfF(rtb_jxi)) {
    rtb_jxi = 0.0F;
  } else {
    rtb_jxi = (real32_T)fmod(rtb_jxi, 65536.0F);
  }

  /* MATLAB Function: '<S640>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S640>/Constant'
   *  Constant: '<S640>/Constant1'
   *  DataTypeConversion: '<S630>/Data Type Conversion3'
   */
  AUA_EmbeddedMATLABFunction((real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdROLLraw, 0.01, 0.1,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fx,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fx);

  /* DataTypeConversion: '<S630>/Data Type Conversion4' */
  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun1': '<S641>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S641>:1:5' y = [u1(1); u2(1)]; */
  rtb_Sum1 = (real32_T)floor
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_fx.y);
  if (rtIsNaNF(rtb_Sum1) || rtIsInfF(rtb_Sum1)) {
    rtb_Sum1 = 0.0F;
  } else {
    rtb_Sum1 = (real32_T)fmod(rtb_Sum1, 65536.0F);
  }

  /* DataStoreWrite: '<S17>/mlAttitude' incorporates:
   *  DataTypeConversion: '<S630>/Data Type Conversion1'
   *  DataTypeConversion: '<S630>/Data Type Conversion4'
   */
  mlVISensor.voltage = rtb_jxi < 0.0F ? (uint16_T)-(int16_T)(uint16_T)-rtb_jxi :
    (uint16_T)rtb_jxi;
  mlVISensor.reading2 = rtb_Sum1 < 0.0F ? (uint16_T)-(int16_T)(uint16_T)
    -rtb_Sum1 : (uint16_T)rtb_Sum1;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S12>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* number of I2C blocks : 5 ; Current: 2 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C22_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadTConv[0] =
      I2C22_Buff8[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadTConv[1] =
      I2C22_Buff8[1];
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

  /* Sum: '<S539>/SumA21' incorporates:
   *  DataTypeConversion: '<S541>/Data Type Conversion1'
   *  DataTypeConversion: '<S541>/Data Type Conversion3'
   *  Delay: '<S539>/Delay11'
   *  Gain: '<S539>/a(2)(1)'
   *  Gain: '<S539>/s(1)'
   *  Gain: '<S541>/Gain'
   *  S-Function (sfix_bitop): '<S541>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)
            AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadTConv[0] <<
            8 | AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadTConv
            [1]) * 31949UL >> 5;
  qY_0 = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE);
  if (qY_0 < rtb_B4) {
    qY_0 = MAX_uint32_T;
  }

  /* Sum: '<S539>/SumB21' incorporates:
   *  Delay: '<S539>/Delay11'
   *  Sum: '<S539>/SumA21'
   */
  if (qY_0 > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY_0 << 1;
  }

  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE << 1;
  }

  qY = rtb_B4 + rtb_Sum6;
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  rtb_u2deg = (uint16_T)(qY >> 16);

  /* End of Sum: '<S539>/SumB21' */

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S12>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  /* number of I2C blocks : 5 ; Current: 3 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C23_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadPConv[0] =
      I2C23_Buff8[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadPConv[1] =
      I2C23_Buff8[1];
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

  /* Sum: '<S540>/SumA21' incorporates:
   *  DataTypeConversion: '<S542>/Data Type Conversion1'
   *  DataTypeConversion: '<S542>/Data Type Conversion3'
   *  Delay: '<S540>/Delay11'
   *  Gain: '<S540>/a(2)(1)'
   *  Gain: '<S540>/s(1)'
   *  Gain: '<S542>/Gain'
   *  S-Function (sfix_bitop): '<S542>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)
            AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadPConv[0] <<
            8 | AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180ReadPConv
            [1]) * 31949UL >> 5;
  qY = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE_j);
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S540>/SumB21' incorporates:
   *  Delay: '<S540>/Delay11'
   *  Sum: '<S540>/SumA21'
   */
  if (qY > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY << 1;
  }

  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE_j > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE_j << 1;
  }

  rtb_Sum6 += rtb_B4;
  if (rtb_Sum6 < rtb_B4) {
    rtb_Sum6 = MAX_uint32_T;
  }

  rtb_Switch1_p = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S540>/SumB21' */

  /* DataStoreWrite: '<S12>/Update RawPressure' incorporates:
   *  DataTypeConversion: '<S12>/Data Type Conversion7'
   *  DataTypeConversion: '<S12>/Data Type Conversion8'
   */
  mlRawPressureData.press_abs = (int16_T)rtb_u2deg;
  mlRawPressureData.temperature = (int16_T)rtb_Switch1_p;

  /* Outputs for Enabled SubSystem: '<S12>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S538>/Enable'
   */
  /* Delay: '<S12>/Delay' */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S538>/BUS I2C Initialize BMP180 read Calibration data @ 1Hz' */
    /* number of I2C blocks : 5 ; Current: 1 ; MCHP_I2C_StartImplemented =  5*/
    if (MCHP_I2C21_Request == 0)       /* Last I2C sequence from this block is finished (not in the queue ?) */
    {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[0] =
        I2C21_Buff8[0];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[1] =
        I2C21_Buff8[1];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[2] =
        I2C21_Buff8[2];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[3] =
        I2C21_Buff8[3];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[4] =
        I2C21_Buff8[4];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[5] =
        I2C21_Buff8[5];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[6] =
        I2C21_Buff8[6];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[7] =
        I2C21_Buff8[7];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[8] =
        I2C21_Buff8[8];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[9] =
        I2C21_Buff8[9];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[10] =
        I2C21_Buff8[10];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[11] =
        I2C21_Buff8[11];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[12] =
        I2C21_Buff8[12];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[13] =
        I2C21_Buff8[13];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[14] =
        I2C21_Buff8[14];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[15] =
        I2C21_Buff8[15];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[16] =
        I2C21_Buff8[16];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[17] =
        I2C21_Buff8[17];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[18] =
        I2C21_Buff8[18];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[19] =
        I2C21_Buff8[19];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[20] =
        I2C21_Buff8[20];
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[21] =
        I2C21_Buff8[21];
      MCHP_I2C21_Request ++;
      MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 36;
      if (MCHP_I2C2_Queue.head >= 5)   /* There are 5 blocks I2C2, max idx for queue is 5 */
        MCHP_I2C2_Queue.head = 0;
      else
        MCHP_I2C2_Queue.head ++;
      if (MCHP_I2C2_State == 0)
        _MI2C2IF = 1;                  /* Force Interrupt */
    } else if (MCHP_I2C21_Request > 3) {
      I2C2CONbits.I2CEN = 0;           /* Switch Off I2C*/
      TRISAbits.TRISA3 = 0;            /* Set I2C Port as Port Output */
      TRISAbits.TRISA2 = 0;
      MCHP_I2C21_Request = 0;          /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
      MCHP_I2C2_State = 1;             /* try to Reset I2C BUS */
      _MI2C2IF = 1;                    /* Force Interrupt */
    } else
      MCHP_I2C21_Request++;

    /* DataTypeConversion: '<S538>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S555>/Data Type Conversion1'
     *  DataTypeConversion: '<S555>/Data Type Conversion3'
     *  Gain: '<S555>/Gain'
     *  S-Function (sfix_bitop): '<S555>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition3 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S538>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S554>/Data Type Conversion1'
     *  DataTypeConversion: '<S554>/Data Type Conversion3'
     *  Gain: '<S554>/Gain'
     *  S-Function (sfix_bitop): '<S554>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S538>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S556>/Data Type Conversion1'
     *  DataTypeConversion: '<S556>/Data Type Conversion3'
     *  Gain: '<S556>/Gain'
     *  S-Function (sfix_bitop): '<S556>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S538>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S560>/Data Type Conversion1'
     *  DataTypeConversion: '<S560>/Data Type Conversion3'
     *  Gain: '<S560>/Gain'
     *  S-Function (sfix_bitop): '<S560>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S538>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S561>/Data Type Conversion1'
     *  DataTypeConversion: '<S561>/Data Type Conversion3'
     *  Gain: '<S561>/Gain'
     *  S-Function (sfix_bitop): '<S561>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S538>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S552>/Data Type Conversion1'
     *  DataTypeConversion: '<S552>/Data Type Conversion3'
     *  Gain: '<S552>/Gain'
     *  S-Function (sfix_bitop): '<S552>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S538>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S553>/Data Type Conversion1'
     *  DataTypeConversion: '<S553>/Data Type Conversion3'
     *  Gain: '<S553>/Gain'
     *  S-Function (sfix_bitop): '<S553>/Bitwise Operator'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S557>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S557>/Data Type Conversion1'
     *  DataTypeConversion: '<S557>/Data Type Conversion3'
     *  Gain: '<S557>/Gain'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition5 = (uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S558>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S558>/Data Type Conversion1'
     *  DataTypeConversion: '<S558>/Data Type Conversion3'
     *  Gain: '<S558>/Gain'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition6 = (uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S559>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S559>/Data Type Conversion1'
     *  DataTypeConversion: '<S559>/Data Type Conversion3'
     *  Gain: '<S559>/Gain'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition7 = (uint16_T)
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S12>/Delay' */
  /* End of Outputs for SubSystem: '<S12>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S536>/Sum' */
  rtb_u2deg -= AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition7;

  /* Product: '<S536>/Product' */
  rtb_Sum6 = (uint32_T)rtb_u2deg *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition6;
  rtb_u2deg = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) != 0U) +
    (rtb_Sum6 >> 15));

  /* Sum: '<S536>/Sum2' incorporates:
   *  Bias: '<S536>/Bias'
   *  Bias: '<S536>/Bias1'
   *  Product: '<S536>/Product1'
   *  Sum: '<S536>/Sum1'
   */
  i = (div_s16s32_round((int32_T)
                        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition11 << 11,
                        rtb_u2deg +
                        AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition12) +
       (int16_T)rtb_u2deg) - 4000;

  /* MATLAB Function: '<S537>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputat_h,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EnablesDisablestheComputat_h);

  /* Outputs for Enabled SubSystem: '<S537>/Zero Out Height' */

  /* Constant: '<S537>/Constant5' */
  AUAV3_WITH_S_ZeroOutHeight
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputat_h.tOut,
     143.543F, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ZeroOutHeight_f,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.ZeroOutHeight_f);

  /* End of Outputs for SubSystem: '<S537>/Zero Out Height' */

  /* Math: '<S536>/Math Function' */
  yIdx = (int32_T)i * i;

  /* Sum: '<S536>/Sum6' incorporates:
   *  Bias: '<S536>/Bias2'
   *  Product: '<S536>/Product2'
   *  Product: '<S536>/Product3'
   *  Sum: '<S536>/Sum3'
   *  Sum: '<S536>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_Switch1_p - (((((int16_T)((int32_T)i *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition3 >> 11) + (int16_T)
    mul_s32_s32_s32_sr23(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition9, yIdx))
    + ((int32_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition2 << 2)) + 2L) >>
    2);

  /* Product: '<S536>/Product6' incorporates:
   *  Bias: '<S536>/Bias3'
   *  Bias: '<S536>/Bias4'
   *  Gain: '<S536>/Gain1'
   *  Product: '<S536>/Product4'
   *  Product: '<S536>/Product5'
   *  Sum: '<S536>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15(((((int16_T)((int32_T)i *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition4 >> 13) + (int16_T)
    mul_s32_s32_s32_sr28(yIdx, AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition8))
    + 2) >> 2) + 32768L, AUAV3_WITH_SLUGS_SENSOR_AND_C_B.RateTransition5);

  /* Switch: '<S536>/Switch' incorporates:
   *  Gain: '<S536>/Gain15'
   *  Gain: '<S536>/Gain22'
   *  Product: '<S536>/Product7'
   *  Product: '<S536>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S536>/Switch' */

  /* Gain: '<S536>/Gain16' */
  yIdx = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >> 8));

  /* Sum: '<S536>/Sum8' incorporates:
   *  Bias: '<S536>/Bias5'
   *  Gain: '<S536>/Gain17'
   *  Gain: '<S536>/Gain19'
   *  Gain: '<S536>/Gain21'
   *  Math: '<S536>/Math Function2'
   *  Sum: '<S536>/Sum7'
   */
  yIdx = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat(yIdx, yIdx))
            + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >> 4) + (int32_T)
    rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S537>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S547>/Enable'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputat_h.tOut > 0.0)
  {
    /* DataTypeConversion: '<S547>/Data Type Conversion' */
    rtb_DataTypeConversion_i = yIdx;

    /* DiscreteZeroPole: '<S550>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_d = 0.014778325123152709*rtb_DataTypeConversion_i;
      rtb_DiscreteZeroPole_d += 0.029119852459414206*
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nk;
    }

    /* Saturate: '<S547>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S547>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole_d > 120000.0F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k_i = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole_d < 80000.0F) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k_i = 80000.0F;
    } else {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k_i = (real32_T)
        rtb_DiscreteZeroPole_d;
    }

    /* End of Saturate: '<S547>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S550>/Discrete Zero-Pole' */
    {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nk =
        rtb_DataTypeConversion_i + 0.97044334975369462*
        AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nk;
    }
  }

  /* End of Outputs for SubSystem: '<S537>/Initial Baro Bias' */

  /* Product: '<S544>/Divide' incorporates:
   *  Sum: '<S544>/Sum2'
   */
  rtb_Sum_b = ((real32_T)yIdx - AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k_i) /
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.u0k120k_i;

  /* Sum: '<S544>/Sum1' incorporates:
   *  Constant: '<S537>/Constant5'
   *  Constant: '<S544>/Constant2'
   *  Constant: '<S544>/Constant3'
   *  Constant: '<S544>/Constant4'
   *  Constant: '<S544>/Constant5'
   *  Gain: '<S549>/Unit Conversion'
   *  Product: '<S544>/Divide1'
   *  Product: '<S544>/Divide2'
   *  Product: '<S544>/Divide3'
   *  Product: '<S544>/Divide4'
   *  Sum: '<S544>/Sum3'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1_b = ((rtb_Sum_b * rtb_Sum_b *
    0.093502529F + rtb_Sum_b * -0.188893303F) + 2.18031291E-5F) * 145473.5F *
    0.3048F + 143.543F;

  /* Outputs for Enabled SubSystem: '<S537>/Enabled Subsystem' */

  /* Logic: '<S537>/Logical Operator' incorporates:
   *  Sum: '<S537>/Sum1'
   */
  AUAV3_WIT_EnabledSubsystem
    (!(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputat_h.tOut !=
       0.0), AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ZeroOutHeight_f.Sum +
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1_b,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_h);

  /* End of Outputs for SubSystem: '<S537>/Enabled Subsystem' */

  /* DataStoreWrite: '<S12>/Update ScaledPressure' incorporates:
   *  Bias: '<S536>/Bias'
   *  Bias: '<S536>/Bias1'
   *  Gain: '<S12>/Gain21'
   */
  mlAirData.temperature = (int16_T)((i + 4008) * 5L >> 3);
  mlAirData.press_abs = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_h.In1;

  /* S-Function (MCHP_BUS_SPI): '<S13>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
  /* number of SPI blocks : 2 ; Current: 1 ; MCHP_SPI_StartImplemented =  2*/
  if (MCHP_SPI11_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[0] = SPI11_Buff16[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[1] = SPI11_Buff16[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[2] = SPI11_Buff16[2];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH4 = SPI11_Buff16[3];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0] =
      SPI11_Buff16[4];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1] =
      SPI11_Buff16[5];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2] =
      SPI11_Buff16[6];
    MCHP_SPI11_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 8;
    if (MCHP_SPI1_Queue.head >= 2)     /* There are 2 blocks SPI1, max idx for queue is 2 */
      MCHP_SPI1_Queue.head = 0;
    else
      MCHP_SPI1_Queue.head ++;
    if (MCHP_SPI1_State == 0)
      _SPI1IF = 1;                     /* Force Interrupt */
  }

  /* DataStoreWrite: '<S13>/Update MPU_T' */
  MPU_T = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH4;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S13>/BUS I2C Read HMC5883 Magn (50 Hz)' */
  /* number of I2C blocks : 5 ; Current: 4 ; MCHP_I2C_StartImplemented =  5*/
  if (MCHP_I2C24_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[0] = I2C24_Buff8[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[1] = I2C24_Buff8[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[2] = I2C24_Buff8[2];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[3] = I2C24_Buff8[3];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[4] = I2C24_Buff8[4];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[5] = I2C24_Buff8[5];
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

  /* DataStoreWrite: '<S13>/Update Raw IMU DATA' incorporates:
   *  DataTypeConversion: '<S13>/Data Type Conversion'
   *  DataTypeConversion: '<S13>/Data Type Conversion1'
   *  DataTypeConversion: '<S13>/Data Type Conversion2'
   *  DataTypeConversion: '<S13>/Data Type Conversion3'
   *  DataTypeConversion: '<S13>/Data Type Conversion4'
   *  DataTypeConversion: '<S13>/Data Type Conversion5'
   *  DataTypeConversion: '<S562>/Data Type Conversion3'
   *  DataTypeConversion: '<S563>/Data Type Conversion3'
   *  DataTypeConversion: '<S564>/Data Type Conversion3'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.xacc = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.yacc = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.zacc = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.U1CH8[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.xgyro = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.ygyro = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.zgyro = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSSPIReadMPU6050AxyzTGxyz1kHz_[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.xmag =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.ymag =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[3];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU.zmag =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.BUSI2CReadHMC5883Magn50Hz[5];

  /* S-Function "dsPIC_PWM_IC" Block: <S1>/Input Capture */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dT = MCHP_ic1up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dA = MCHP_ic2up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dE = MCHP_ic3up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dR = MCHP_ic4up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dFailsafe = MCHP_ic5up;

  /* MATLAB Function: '<S1>/myMux Fun5' */
  AUAV3_WITH_SLUGS_myMuxFun5(AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dT,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dA, AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dE,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dR,
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.dFailsafe,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun5_h);

  /* Switch: '<S25>/FixPt Switch' incorporates:
   *  Constant: '<S24>/FixPt Constant'
   *  Sum: '<S24>/FixPt Sum1'
   *  UnitDelay: '<S2>/Output'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.Output_DSTATE++;

  /* DataTypeConversion: '<S10>/Data Type Conversion1' incorporates:
   *  DataTypeConversion: '<S14>/Data Type Conversion'
   */
  rtb_DataTypeConversion1_k = rtb_EnableHILfromControlMCU1;

  /* S-Function (MCHP_Digital_Output_Write): '<S432>/Digital Output Write' */
  LATBbits.LATB6 = rtb_DataTypeConversion1_k;

  /* S-Function (MCHP_Digital_Output_Write): '<S433>/Digital Output Write' */
  LATEbits.LATE9 = rtb_DataTypeConversion1_k;

  /* Sum: '<S436>/Sum1' incorporates:
   *  Gain: '<S436>/Gain'
   */
  rtb_DataTypeConversion7_idx_0 += 0.1F * rtb_Add_of[0];
  rtb_DataTypeConversion7_idx_1 += 0.1F * rtb_Add_of[1];
  rtb_DataTypeConversion7 += 0.1F * rtb_Add_of[2];

  /* MATLAB Function: '<S436>/q dot calc' incorporates:
   *  SignalConversion: '<S455>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Position and Attitude Filter [updated 5.13.16]/Attitude Complimentary Filter COG/q dot calc': '<S455>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S455>:1:5' s = q(1,1); */
  /* '<S455>:1:6' v = q(2:4,1); */
  /* '<S455>:1:7' q_dot = single(zeros(4,1)); */
  rtb_q_dot[1] = 0.0F;
  rtb_q_dot[2] = 0.0F;
  rtb_q_dot[3] = 0.0F;

  /* '<S455>:1:8' q_dot(1,1) = -.5*v'*om_hat; */
  rtb_q_dot[0] = (-0.5F * rtb_Product1_bh * rtb_DataTypeConversion7_idx_0 +
                  -0.5F * rtb_Product2_ka * rtb_DataTypeConversion7_idx_1) +
    -0.5F * rtb_Product3_e * rtb_DataTypeConversion7;

  /* '<S455>:1:9' q_dot(2:4,1) =  .5*[ s    -v(3) v(2); ... */
  /* '<S455>:1:10'                         v(3)  s    -v(1); ... */
  /* '<S455>:1:11'                        -v(2) v(1) s]*om_hat; */
  rtb_y_c_0[0] = rtb_Product_hz;
  rtb_y_c_0[3] = -rtb_Product3_e;
  rtb_y_c_0[6] = rtb_Product2_ka;
  rtb_y_c_0[1] = rtb_Product3_e;
  rtb_y_c_0[4] = rtb_Product_hz;
  rtb_y_c_0[7] = -rtb_Product1_bh;
  rtb_y_c_0[2] = -rtb_Product2_ka;
  rtb_y_c_0[5] = rtb_Product1_bh;
  rtb_y_c_0[8] = rtb_Product_hz;
  for (i = 0; i < 3; i++) {
    rtb_MathFunction[3 * i] = rtb_y_c_0[3 * i] * 0.5F;
    rtb_MathFunction[1 + 3 * i] = rtb_y_c_0[3 * i + 1] * 0.5F;
    rtb_MathFunction[2 + 3 * i] = rtb_y_c_0[3 * i + 2] * 0.5F;
  }

  for (i = 0; i < 3; i++) {
    rtb_q_dot[1 + i] = 0.0F;
    rtb_q_dot[1 + i] += rtb_MathFunction[i] * rtb_DataTypeConversion7_idx_0;
    rtb_q_dot[1 + i] += rtb_MathFunction[i + 3] * rtb_DataTypeConversion7_idx_1;
    rtb_q_dot[1 + i] += rtb_MathFunction[i + 6] * rtb_DataTypeConversion7;
  }

  /* End of MATLAB Function: '<S436>/q dot calc' */

  /* DataTypeConversion: '<S532>/Data Type Conversion2' */
  rtb_DataTypeConversion2_f = rtb_Gain_px;

  /* DiscreteZeroPole: '<S533>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_k = 0.005*rtb_DataTypeConversion2_f;
    rtb_DiscreteZeroPole_k += 0.01*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l1;
  }

  /* DataTypeConversion: '<S532>/Data Type Conversion1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UnitDelay_DSTATE_jx = (real32_T)
    rtb_DiscreteZeroPole_k;

  /* RelationalOperator: '<S618>/Compare' incorporates:
   *  Constant: '<S618>/Constant'
   */
  rtb_DataTypeConversion2_j = (uint8_T)(rtb_Sum < -130.0F);

  /* Outputs for Enabled SubSystem: '<S603>/Hi Temp Compensation2' incorporates:
   *  EnablePort: '<S619>/Enable'
   */
  /* Logic: '<S603>/Logical Operator' */
  if (!(rtb_DataTypeConversion2_j != 0)) {
    /* Sum: '<S619>/Sum2' incorporates:
     *  Constant: '<S619>/Mean Temperature for Calibration'
     *  Constant: '<S619>/gains'
     *  Product: '<S619>/Divide1'
     *  Sum: '<S619>/Sum1'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_h = (real32_T)rtb_Switch_a[10] -
      (rtb_Sum - 293.053F) * -0.0950433F;
  }

  /* End of Logic: '<S603>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S603>/Hi Temp Compensation2' */

  /* Outputs for Enabled SubSystem: '<S603>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S620>/Enable'
   */
  if (rtb_DataTypeConversion2_j > 0) {
    /* Sum: '<S620>/Add' incorporates:
     *  Constant: '<S620>/Constant'
     *  Constant: '<S620>/Mean Temperature for Calibration'
     *  Constant: '<S620>/gains'
     *  Product: '<S620>/Divide1'
     *  Sum: '<S620>/Sum1'
     *  Sum: '<S620>/Sum2'
     */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_h = ((real32_T)rtb_Switch_a[10] -
      (rtb_Sum - -202.93F) * -0.0552923F) + -41.0F;
  }

  /* End of Outputs for SubSystem: '<S603>/Lo Temp Compensation' */

  /* MATLAB Function: '<S608>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S608>/Constant'
   *  Constant: '<S608>/Constant1'
   *  DataTypeConversion: '<S570>/Data Type Conversion3'
   */
  A_EmbeddedMATLABFunction_n((real_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_h,
    0.01, 4.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ht,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_ht);

  /* Sum: '<S601>/Sum' incorporates:
   *  Constant: '<S601>/Bias'
   *  Constant: '<S601>/Gains'
   *  DataTypeConversion: '<S570>/Data Type Conversion4'
   *  Product: '<S601>/Divide'
   */
  rtb_Sum1 = 1.05137849F * (real32_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ht.y +
    -1005.87872F;

  /* Saturate: '<S570>/[0.001  maxDynPress]' */
  if (rtb_Sum1 > 3000.0F) {
    rtb_Sum1 = 3000.0F;
  } else {
    if (rtb_Sum1 < 0.001F) {
      rtb_Sum1 = 0.001F;
    }
  }

  /* End of Saturate: '<S570>/[0.001  maxDynPress]' */

  /* MATLAB Function: '<S566>/myMux Fun' */
  AUAV3_WITH_SLU_myMuxFun1_f(rtb_Sum1, rtb_Sum_cm, rtb_Sum,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_n);

  /* Outputs for Enabled SubSystem: '<S566>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S569>/Enable'
   */
  if (rtb_LogicalOperator_bu) {
    /* Inport: '<S569>/AirData' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.AirData[0] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_n.y[0];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.AirData[1] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_n.y[1];
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.AirData[2] =
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_n.y[2];

    /* S-Function (MCHP_C_function_Call): '<S569>/Update the Air Calibrated Data [updateSensorMcuState.c]1' */
    updateAirData(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.AirData[0]
                  );
  }

  /* End of Outputs for SubSystem: '<S566>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S566>/myMux Fun1' */
  /* MATLAB Function 'Sensor Data [updated 4.28.16]/Sensor Suite/myMux Fun1': '<S572>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S572>:1:5' y = [u1(1); u2(1); u3(1); u4(1)]; */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_o[0] = rtb_Sum1;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_o[1] =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.AirData[0];
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_o[2] = 1.0F;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_o[3] = 1.0F;

  /* S-Function (MCHP_C_function_Call): '<S566>/Sensor DSC Diag [updateSensorMcuState.c]' */
  updateSensorDiag(
                   &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.y_o[0]
                   );

  /* MATLAB Function: '<S607>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S607>/Constant'
   *  Constant: '<S607>/Constant1'
   *  DataTypeConversion: '<S570>/Data Type Conversion6'
   */
  A_EmbeddedMATLABFunction_n((real_T)rtb_Switch_a[11], 0.01, 0.02,
    &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_p,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_p);

  /* DataTypeConversion: '<S570>/Data Type Conversion8' incorporates:
   *  Constant: '<S602>/Bias'
   *  Product: '<S602>/Divide'
   *  Sum: '<S602>/Sum'
   */
  rtb_jxi = (real32_T)floor((real32_T)(3.1760616302490234 *
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_p.y) + 911.698242F);
  if (rtIsNaNF(rtb_jxi) || rtIsInfF(rtb_jxi)) {
    rtb_jxi = 0.0F;
  } else {
    rtb_jxi = (real32_T)fmod(rtb_jxi, 65536.0F);
  }

  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion8 = rtb_jxi < 0.0F ?
    (uint16_T)-(int16_T)(uint16_T)-rtb_jxi : (uint16_T)rtb_jxi;

  /* End of DataTypeConversion: '<S570>/Data Type Conversion8' */

  /* S-Function (MCHP_C_function_Call): '<S566>/Update the Load and Power Data [updateSensorMcuState.c]' */
  updateLoadData(
                 ((uint8_T)1U)
                 , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.DataTypeConversion8
                 );

  /* MATLAB Function: '<S575>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S567>/Gyro Gains2'
   *  Constant: '<S575>/Constant'
   *  Constant: '<S575>/Constant1'
   *  DataTypeConversion: '<S567>/Data Type Conversion5'
   *  Gain: '<S578>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide2'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[7] * 0.5F), 0.01,
    40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ih,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_ih);

  /* MATLAB Function: '<S575>/Embedded MATLAB Function1' incorporates:
   *  Constant: '<S567>/Gyro Gains2'
   *  Constant: '<S575>/Constant2'
   *  Constant: '<S575>/Constant3'
   *  DataTypeConversion: '<S567>/Data Type Conversion5'
   *  Gain: '<S578>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide2'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[6] * 0.5F), 0.01,
    40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_g,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_g);

  /* MATLAB Function: '<S575>/Embedded MATLAB Function2' incorporates:
   *  Constant: '<S567>/Gyro Gains2'
   *  Constant: '<S575>/Constant4'
   *  Constant: '<S575>/Constant5'
   *  DataTypeConversion: '<S567>/Data Type Conversion5'
   *  Gain: '<S578>/[ -1 -1 -1]'
   *  Product: '<S567>/Divide2'
   */
  A_EmbeddedMATLABFunction_n((real_T)-((real32_T)rtb_Switch_a[8] * 0.5F), 0.01,
    40.0, &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_n,
    &AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_n);

  /* MATLAB Function: '<S575>/myMux Fun' */
  AUAV3_WITH_SLUGS__myMuxFun
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction_ih.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction1_g.y,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EmbeddedMATLABFunction2_n.y,
     &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_myMuxFun_j);/* MATLAB Function 'myMux Fun2': '<S20>:1' */

  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S20>:1:5' y = [u1(1:3); u2(1:3); u3(1:3)]; */
  /* S-Function "dsPIC_PWM_IC" Block: <S15>/Input Capture RC Receiver1 */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdPITCHraw = MCHP_ic1up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCaptureRCReceiver1_o2 = MCHP_ic2up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCaptureRCReceiver1_o3 = MCHP_ic3up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdROLLraw = MCHP_ic4up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCaptureRCReceiver1_o5 = MCHP_ic5up;
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InputCaptureRCReceiver1_o6 = MCHP_ic6up;

  /* Bias: '<S15>/Bias' incorporates:
   *  DataTypeConversion: '<S15>/Data Type Conversion4'
   */
  i = (int16_T)AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdPITCHraw - 13125;

  /* Bias: '<S15>/Bias1' incorporates:
   *  DataTypeConversion: '<S15>/Data Type Conversion5'
   */
  rtb_DataTypeConversion1_a = (int16_T)
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.cmdROLLraw - 13125;

  /* Sum: '<S628>/Sum' */
  yIdx = (int32_T)rtb_DataTypeConversion1_a - i;
  if (yIdx > 32767L) {
    yIdx = 32767L;
  } else {
    if (yIdx < -32768L) {
      yIdx = -32768L;
    }
  }

  /* DataTypeConversion: '<S15>/Data Type Conversion2' incorporates:
   *  Bias: '<S15>/Bias2'
   *  Sum: '<S628>/Sum'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation = (uint16_T)((int16_T)yIdx + 13125);

  /* Saturate: '<S15>/Saturation' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation > 19249U) {
    /* DataTypeConversion: '<S15>/Data Type Conversion2' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation = 19249U;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation < 7000U) {
      /* DataTypeConversion: '<S15>/Data Type Conversion2' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation = 7000U;
    }
  }

  /* End of Saturate: '<S15>/Saturation' */

  /* Sum: '<S628>/Sum1' */
  yIdx = (int32_T)i + rtb_DataTypeConversion1_a;
  if (yIdx > 32767L) {
    yIdx = 32767L;
  } else {
    if (yIdx < -32768L) {
      yIdx = -32768L;
    }
  }

  /* DataTypeConversion: '<S15>/Data Type Conversion3' incorporates:
   *  Bias: '<S15>/Bias3'
   *  Sum: '<S628>/Sum1'
   */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1 = (uint16_T)((int16_T)yIdx + 13125);

  /* Saturate: '<S15>/Saturation1' */
  if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1 > 19249U) {
    /* DataTypeConversion: '<S15>/Data Type Conversion3' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1 = 19249U;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1 < 7000U) {
      /* DataTypeConversion: '<S15>/Data Type Conversion3' */
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1 = 7000U;
    }
  }

  /* End of Saturate: '<S15>/Saturation1' */

  /* Update for DiscreteZeroPole: '<S529>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE =
      rtb_DataTypeConversion2 + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE;
  }

  /* Update for DiscreteZeroPole: '<S531>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_a =
      rtb_DataTypeConversion2_p + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* Update for UnitDelay: '<S502>/UD'
   *
   * Block description for '<S502>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[0];

  /* Update for UnitDelay: '<S503>/UD'
   *
   * Block description for '<S503>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_m =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[1];

  /* Update for UnitDelay: '<S504>/UD'
   *
   * Block description for '<S504>/UD':
   *
   *  Store in Global RAM
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_o =
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.In1[2];

  /* Update for Enabled SubSystem: '<S599>/Zero Out Height' */
  AUAV3_ZeroOutHeight_Update
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputatio.tOut,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.ZeroOutHeight);

  /* End of Update for SubSystem: '<S599>/Zero Out Height' */

  /* Update for Delay: '<S35>/Integer Delay3' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE = rtb_Switch3;

  /* Update for DiscreteIntegrator: '<S436>/Discrete-Time Integrator1' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] += 0.01F *
    rtb_q_dot[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] += 0.01F *
    rtb_q_dot[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] += 0.01F *
    rtb_q_dot[2];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] += 0.01F *
    rtb_q_dot[3];
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] >= 1.0F)
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] <=
        -1.0F) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] = -1.0F;
    }
  }

  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] >= 1.0F)
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 1.0F;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] <=
        -1.0F) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] = -1.0F;
    }
  }

  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] >= 1.0F)
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 1.0F;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] <=
        -1.0F) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] = -1.0F;
    }
  }

  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] >= 1.0F)
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 1.0F;
  } else {
    if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] <=
        -1.0F) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] = -1.0F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S436>/Discrete-Time Integrator1' */

  /* Update for Delay: '<S436>/Integer Delay' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[0] =
    rtb_DataTypeConversion7_idx_0;
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[1] =
    rtb_DataTypeConversion7_idx_1;
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE[2] =
    rtb_DataTypeConversion7;

  /* Update for Delay: '<S447>/Integer Delay1' */
  for (i = 0; i < 4; i++) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[i * 3] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[(i + 1) * 3];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[i * 3 + 1] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[(i + 1) * 3 + 1];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[i * 3 + 2] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[(i + 1) * 3 + 2];
  }

  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[12] = rtb_P32[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[13] = rtb_P32[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[14] = rtb_P32[2];

  /* End of Update for Delay: '<S447>/Integer Delay1' */

  /* Update for Delay: '<S517>/Integer Delay1' */
  for (i = 0; i < 4; i++) {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[i * 3] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[(i + 1) * 3];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[i * 3 + 1] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[(i + 1) * 3 + 1];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[i * 3 + 2] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[(i + 1) * 3 + 2];
  }

  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[12] = rtb_Product1_a3[0];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[13] = rtb_Product1_a3[1];
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[14] = rtb_Product1_a3[2];

  /* End of Update for Delay: '<S517>/Integer Delay1' */
  /* Update for DiscreteZeroPole: '<S524>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l =
      rtb_DataTypeConversion2_d[0] + (-0.99960423832914991)*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l;
  }

  /* Update for DiscreteZeroPole: '<S525>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_i =
      rtb_DataTypeConversion2_d[1] + (-0.99960423832914991)*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_i;
  }

  /* Update for DiscreteZeroPole: '<S526>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_n =
      rtb_DataTypeConversion2_d[2] + (-0.99960423832914991)*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* Update for DiscreteZeroPole: '<S456>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_d =
      rtb_DataTypeConversion2_dp[0] + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_d;
  }

  /* Update for DiscreteZeroPole: '<S457>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nu =
      rtb_DataTypeConversion2_dp[1] + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_nu;
  }

  /* Update for DiscreteZeroPole: '<S458>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_c =
      rtb_DataTypeConversion2_dp[2] + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_c;
  }

  /* Update for Delay: '<S36>/Integer Delay3' */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_e = rtb_kxi;

  /* Update for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  Update for EnablePort: '<S31>/Enable'
   */
  if (AUAV3_WITH_SLUGS_SENSOR_A_DWork.L1OutputFeedbackControllerWithP) {
    /* Update for UnitDelay: '<S51>/UD'
     *
     * Block description for '<S51>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_f = rtb_Deg2R_idx_0;

    /* Update for Delay: '<S41>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a1 = rtb_Switch1_h;

    /* Update for Delay: '<S41>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_l = rtb_Subtract_j5;

    /* Update for Delay: '<S43>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_h = rtb_Switch3_bx;

    /* Update for UnitDelay: '<S61>/UD'
     *
     * Block description for '<S61>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_i = rtb_Deg2R_idx_1;

    /* Update for Delay: '<S45>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_f = rtb_Switch3_fv;

    /* Update for Delay: '<S45>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_a = rtb_Subtract_jz;

    /* Update for Delay: '<S31>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bt = rtb_kxj;

    /* Update for Delay: '<S31>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja = rtb_Switch1_h;

    /* Update for Delay: '<S44>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_nf = rtb_Switch3_fv;

    /* Update for Delay: '<S46>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_o = rtb_Projection;

    /* Update for Delay: '<S46>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[0] =
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[1];
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[1] = rtb_Projection;

    /* Update for Delay: '<S46>/Integer Delay2' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_l = rtb_Switch1_h;

    /* Update for Delay: '<S62>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_mn = rtb_Projection;

    /* Update for Delay: '<S63>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_aq = rtb_Switch1_h;
  }

  /* End of Update for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
  /* Update for S-Function (MCHP_PWM_HighSpeed): '<S17>/PWM High Speed' */
  PDC1 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[2];/* Period for Channel 1 */
  PDC2 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[3];/* Period for Channel 2 */
  PDC3 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[1];/* Period for Channel 3 */
  PDC4 = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_e[0];/* Period for Channel 4 */

  /* Update for Delay: '<S539>/Delay11' incorporates:
   *  Sum: '<S539>/SumA21'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE = qY_0;

  /* Update for Delay: '<S540>/Delay11' incorporates:
   *  Sum: '<S540>/SumA21'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay11_DSTATE_j = qY;

  /* Update for Delay: '<S12>/Delay' incorporates:
   *  Constant: '<S12>/Constant'
   */
  AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S537>/Zero Out Height' */
  AUAV3_ZeroOutHeight_Update
    (AUAV3_WITH_SLUGS_SENSOR_AND_C_B.sf_EnablesDisablestheComputat_h.tOut,
     AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Sum1_b,
     &AUAV3_WITH_SLUGS_SENSOR_A_DWork.ZeroOutHeight_f);

  /* End of Update for SubSystem: '<S537>/Zero Out Height' */

  /* Update for DiscreteZeroPole: '<S533>/Discrete Zero-Pole' */
  {
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l1 =
      rtb_DataTypeConversion2_f + 1.0*
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteZeroPole_DSTATE_l1;
  }

  /* S-Function "dsPIC_PWM_OC" Block: <S15>/Output Compare - HW Drive Servo motor */
  OC1CON1 = 0x1008;                    /* Disable OC1 */
  OC1CON2bits.TRIGSTAT = 0;
  OC1RS = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC2CON1 = 0x1008;                    /* Disable OC2 */
  OC2CON2bits.TRIGSTAT = 0;
  OC2RS = AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Saturation1;/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC1CON2bits.TRIGSTAT = 1;
  OC1CON1 = 0x100C;                    /* Trig OC1 pulse */
  OC2CON2bits.TRIGSTAT = 1;
  OC2CON1 = 0x100C;                    /* Trig OC2 pulse */

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  AUAV3_WITH_SLUGS_SENSOR_AND__M->Timing.clockTick0++;
}

/* Model step function for TID1 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step1(void) /* Sample time: [0.02s, 0.0s] */
{
  /* DataStoreRead: '<S5>/Get Raw IMU' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRawIMU =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.mlRawIMU;

  /* DataStoreRead: '<S5>/Get time' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S5>/PackRawIMU' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackRawIMU = PackRawIMU(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRawIMU
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackRawIMU
                  );
}

/* Model step function for TID2 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step2(void) /* Sample time: [0.05s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator_h1;

  /* S-Function (MCHP_Digital_Output_Read): '<S26>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S26>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S26>/Digital Output Read'
   */
  rtb_LogicalOperator_h1 = !LATBbits.LATB2;

  /* S-Function (MCHP_Digital_Output_Write): '<S26>/Digital Output Write' */
  LATBbits.LATB2 = rtb_LogicalOperator_h1;
}

/* Model step function for TID3 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step3(void) /* Sample time: [0.1s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator1;

  /* S-Function (MCHP_Digital_Output_Read): '<S27>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S27>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator1' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S27>/Digital Output Read'
   */
  rtb_LogicalOperator1 = !LATBbits.LATB3;

  /* S-Function (MCHP_Digital_Output_Write): '<S27>/Digital Output Write' */
  LATBbits.LATB3 = rtb_LogicalOperator1;
}

/* Model step function for TID4 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step4(void) /* Sample time: [0.2s, 0.0s] */
{
  /* DataStoreRead: '<S5>/Get RawGpsInt' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRawGpsInt = mlGpsData;

  /* DataStoreRead: '<S5>/Get time1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime1 =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S5>/PackGpsRawInt' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackGpsRawInt = PackGpsRawInt(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetRawGpsInt
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime1
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data2' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackGpsRawInt
                  );

  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );

  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID5 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step5(void) /* Sample time: [0.2s, 0.02s] */
{
  /* DataStoreRead: '<S5>/Get mlAirData' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetmlAirData = mlAirData;

  /* DataStoreRead: '<S5>/Get time2' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime2 =
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S5>/PackScaledPressure' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackScaledPressure = PackScaledPressure(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetmlAirData
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Gettime2
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data3' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackScaledPressure
                  );
}

/* Model step function for TID6 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step6(void) /* Sample time: [0.2s, 0.04s] */
{
  /* DataStoreRead: '<S5>/Get mlSysStatus' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetmlSysStatus = mlSysStatus;

  /* S-Function (MCHP_C_function_Call): '<S5>/PackSysStatus' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackSysStatus = PackSysStatus(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    , AUAV3_WITH_SLUGS_SENSOR_AND_C_B.GetmlSysStatus
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data4' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackSysStatus
                  );
}

/* Model step function for TID7 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step7(void) /* Sample time: [0.2s, 0.06s] */
{
  /* S-Function (MCHP_C_function_Call): '<S5>/ParamInterfaceResponse' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ParamInterfaceResponse =
    ParameterInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data5' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ParamInterfaceResponse
                  );
}

/* Model step function for TID8 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step8(void) /* Sample time: [0.2s, 0.08s] */
{
  /* S-Function (MCHP_C_function_Call): '<S5>/ParamInterfaceResponse1' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ParamInterfaceResponse1 =
    MissionInterfaceResponse(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data6' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.ParamInterfaceResponse1
                  );
}

/* Model step function for TID9 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step9(void) /* Sample time: [0.25s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator2_m;

  /* S-Function (MCHP_Digital_Output_Read): '<S28>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S28>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S28>/Digital Output Read'
   */
  rtb_LogicalOperator2_m = !LATBbits.LATB4;

  /* S-Function (MCHP_Digital_Output_Write): '<S28>/Digital Output Write' */
  LATBbits.LATB4 = rtb_LogicalOperator2_m;
}

/* Model step function for TID10 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step10(void) /* Sample time: [0.5s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator3;

  /* S-Function (MCHP_Digital_Output_Read): '<S29>/Digital Output Read' */

  /* MCHP_Digital_Output_Read Block: <S29>/Digital Output Read/Output */

  /* Logic: '<S3>/Logical Operator3' incorporates:
   *  S-Function (MCHP_Digital_Output_Read): '<S29>/Digital Output Read'
   */
  rtb_LogicalOperator3 = !LATBbits.LATB5;

  /* S-Function (MCHP_Digital_Output_Write): '<S29>/Digital Output Write' */
  LATBbits.LATB5 = rtb_LogicalOperator3;

  /* S-Function (MCHP_C_function_Call): '<S5>/PackHeartBeat' */
  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackHeartBeat = PackHeartBeat(
    ((uint8_T)101U)
    , ((uint8_T)1U)
    );

  /* S-Function (MCHP_C_function_Call): '<S5>/TX_N_Data1' */
  TxN_Data_OverU1(
                  AUAV3_WITH_SLUGS_SENSOR_AND_C_B.PackHeartBeat
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

/* Model step function for TID11 */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step11(void) /* Sample time: [1.0s, 0.0s] */
{
  /* (no output/update code required) */
}

/* Model initialize function */
void AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)AUAV3_WITH_SLUGS_SENSOR_AND__M, 0,
                sizeof(RT_MODEL_AUAV3_WITH_SLUGS_SEN_T));
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[0] = 1;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[1] = 2;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[2] = 5;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[3] = 10;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[4] = 20;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[5] = 20;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[6] = 20;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[7] = 20;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[8] = 20;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[9] = 25;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[10] = 50;
  (AUAV3_WITH_SLUGS_SENSOR_AND__M)->Timing.TaskCounters.cLimit[11] = 100;
  rtmSetFirstInitCond(AUAV3_WITH_SLUGS_SENSOR_AND__M, 1);

  /* initialize sample time offsets */
  AUAV3_WITH_SLUGS_SENSOR_AND__M->Timing.TaskCounters.TID[5] = 18;/* Sample time: [0.2s, 0.02s] */

  /* initialize sample time offsets */
  AUAV3_WITH_SLUGS_SENSOR_AND__M->Timing.TaskCounters.TID[6] = 16;/* Sample time: [0.2s, 0.04s] */

  /* initialize sample time offsets */
  AUAV3_WITH_SLUGS_SENSOR_AND__M->Timing.TaskCounters.TID[7] = 14;/* Sample time: [0.2s, 0.06s] */

  /* initialize sample time offsets */
  AUAV3_WITH_SLUGS_SENSOR_AND__M->Timing.TaskCounters.TID[8] = 12;/* Sample time: [0.2s, 0.08s] */

  /* block I/O */
  (void) memset(((void *) &AUAV3_WITH_SLUGS_SENSOR_AND_C_B), 0,
                sizeof(BlockIO_AUAV3_WITH_SLUGS_SENS_T));

  /* states (dwork) */
  (void) memset((void *)&AUAV3_WITH_SLUGS_SENSOR_A_DWork, 0,
                sizeof(D_Work_AUAV3_WITH_SLUGS_SENSO_T));

  /* exported global states */
  mlParamInterface = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZpi_struct;
  mlWpValues =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_mission_item_values_t;
  mlAttitude = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_attitude_t;
  mlGpsData = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_gps_raw_int_t;
  mlNavigation =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_slugs_navigation_t;
  mlSysStatus = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_sys_status_t;
  mlServoOutputRaw =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_servo_output_raw_t;
  mlAirData =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_scaled_pressure_t;
  mlGSLocationFloat =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_coordinate_float_t;
  mlHeartbeatLocal =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_heartbeat_t;
  mlISR = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_isr_location_t;
  mlMidLevelCommands =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_mid_lvl_cmds_t;
  mlMobileLocation =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_slugs_mobile_location_t;
  mlRawPressureData =
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_raw_pressure_t;
  mlVISensor = AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_rtZmavlink_volt_sensor_t;
  MPU_T = 0U;

  {
    int16_T i;

    /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

    /* Start for S-Function (MCHP_IC): '<S630>/Input Capture' */
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
    IC7CON1 = 0x1003;
    IC7CON2 = 0x00;

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
    _IC6IF = 0;
    _IC6IP = 6;
    _IC6IE = 1;
    _IC7IF = 0;
    _IC7IP = 6;
    _IC7IE = 1;

    /* Start for InitialCondition: '<S632>/IC1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime = true;

    /* Start for Enabled SubSystem: '<S599>/Enabled Subsystem' */
    AUA_EnabledSubsystem_Start
      (&AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_m);

    /* End of Start for SubSystem: '<S599>/Enabled Subsystem' */

    /* InitializeConditions for Enabled SubSystem: '<S510>/Subsystem' */

    /* InitializeConditions for MATLAB Function: '<S518>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_o_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_m);

    /* InitializeConditions for MATLAB Function: '<S518>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_o_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_c);

    /* End of InitializeConditions for SubSystem: '<S510>/Subsystem' */

    /* Start for Atomic SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */
    /* Start for InitialCondition: '<S151>/IC1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC1_FirstOutputTime_e = true;

    /* Start for InitialCondition: '<S151>/IC2' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC2_FirstOutputTime = true;

    /* Start for InitialCondition: '<S151>/IC4' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC4_FirstOutputTime = true;

    /* Start for InitialCondition: '<S148>/IC' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime = true;

    /* InitializeConditions for IfAction SubSystem: '<S142>/RTB//Follow Mobile Navigation' */
    /* InitializeConditions for DiscreteIntegrator: '<S405>/Discrete-Time Integrator' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;

    /* End of InitializeConditions for SubSystem: '<S142>/RTB//Follow Mobile Navigation' */

    /* Start for IfAction SubSystem: '<S142>/Normal WP  Navigation' */

    /* Start for InitialCondition: '<S149>/IC' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC = 0U;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j = true;

    /* InitializeConditions for Enabled SubSystem: '<S149>/Get Frenet' */
    /* InitializeConditions for Delay: '<S265>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_di = 1.0F;

    /* End of InitializeConditions for SubSystem: '<S149>/Get Frenet' */
    /* InitializeConditions for IfAction SubSystem: '<S142>/Normal WP  Navigation' */
    /* InitializeConditions for Delay: '<S149>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bu = 1U;

    /* InitializeConditions for MATLAB Function: '<S149>/Embedded MATLAB Function' */
    /* '<S264>:1:11' persistentDidReachIP =  uint8(0); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.persistentDidReachIP = 0U;

    /* InitializeConditions for MATLAB Function: '<S149>/computeCurrentWP' */
    /* '<S269>:1:11' fromWp =  uint8(1); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.fromWp = 1U;

    /* '<S269>:1:12' toWp = uint8(2); */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.toWp = 2U;

    /* End of InitializeConditions for SubSystem: '<S142>/Normal WP  Navigation' */

    /* Start for IfAction SubSystem: '<S142>/Normal WP  Navigation' */
    /* VirtualOutportStart for Outport: '<S149>/FromWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP0 = 1U;

    /* VirtualOutportStart for Outport: '<S149>/ToWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.WP1 = 2U;

    /* End of Start for SubSystem: '<S142>/Normal WP  Navigation' */
    /* VirtualOutportStart for Outport: '<S147>/FromWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 1U;

    /* VirtualOutportStart for Outport: '<S147>/ToWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 = 2U;

    /* End of Start for SubSystem: '<S142>/Line Segment' */

    /* VirtualOutportStart for Outport: '<S143>/FromWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge3 = 1U;

    /* VirtualOutportStart for Outport: '<S143>/ToWP' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge4 = 1U;

    /* End of Start for SubSystem: '<S142>/Circle Navigation' */
    /* Start for InitialCondition: '<S151>/IC3' */
    AUAV3_WITH_SLUGS_SENSOR_AND_C_B.IC3 = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC3_FirstOutputTime = true;

    /* End of Start for SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */

    /* Start for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
    /* Start for InitialCondition: '<S41>/IC' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_l = true;

    /* Start for InitialCondition: '<S45>/IC' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_j4 = true;

    /* End of Start for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */

    /* InitializeConditions for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
    /* InitializeConditions for UnitDelay: '<S51>/UD'
     *
     * Block description for '<S51>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_f = 0.0F;

    /* InitializeConditions for Delay: '<S41>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_a1 = 0.0F;

    /* InitializeConditions for Delay: '<S41>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_l = 0.0F;

    /* InitializeConditions for Delay: '<S43>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_h = 0.0F;

    /* InitializeConditions for UnitDelay: '<S61>/UD'
     *
     * Block description for '<S61>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_i = 0.0F;

    /* InitializeConditions for Delay: '<S45>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_f = 0.0F;

    /* InitializeConditions for Delay: '<S45>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_a = 0.0F;

    /* InitializeConditions for Delay: '<S31>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_bt = 0.0F;

    /* InitializeConditions for Delay: '<S31>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_ja = 0.0F;

    /* InitializeConditions for Merge: '<S42>/Merge' */
    if (rtmIsFirstInitCond(AUAV3_WITH_SLUGS_SENSOR_AND__M)) {
      AUAV3_WITH_SLUGS_SENSOR_AND_C_B.Merge_b = 0.0F;
    }

    /* End of InitializeConditions for Merge: '<S42>/Merge' */

    /* InitializeConditions for Delay: '<S44>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_nf = 0.0F;

    /* InitializeConditions for Delay: '<S46>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_o = 0.0F;

    /* InitializeConditions for Delay: '<S46>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[0] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j5[1] = 0.0F;

    /* InitializeConditions for Delay: '<S46>/Integer Delay2' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay2_DSTATE_l = 0.0F;

    /* InitializeConditions for Delay: '<S62>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_mn = 0.0F;

    /* InitializeConditions for Delay: '<S63>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_aq = 0.0F;

    /* End of InitializeConditions for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
    /* Start for InitialCondition: '<S80>/IC' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IC_FirstOutputTime_f = true;

    /* InitializeConditions for Enabled SubSystem: '<S64>/Sideslip Compensation' */
    /* InitializeConditions for UnitDelay: '<S88>/UD'
     *
     * Block description for '<S88>/UD':
     *
     *  Store in Global RAM
     */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.UD_DSTATE_b = 0.0F;

    /* InitializeConditions for Delay: '<S80>/Integer Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay_DSTATE_p2 = 0.0F;

    /* InitializeConditions for Delay: '<S80>/Integer Delay1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_j = 0.0F;

    /* InitializeConditions for Delay: '<S81>/Integer Delay3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay3_DSTATE_a = 0.0F;

    /* End of InitializeConditions for SubSystem: '<S64>/Sideslip Compensation' */

    /* Start for IfAction SubSystem: '<S656>/If  Control Type Is Selective Passthrough' */

    /* Start for S-Function (MCHP_C_function_Call): '<S659>/Initialize Control MCU' */
    getPassValues(
                  &AUAV3_WITH_SLUGS_SENSOR_AND_C_B.InitializeControlMCU[0]
                  );

    /* End of Start for SubSystem: '<S656>/If  Control Type Is Selective Passthrough' */

    /* Start for S-Function (MCHP_PWM_HighSpeed): '<S17>/PWM High Speed' */
    /* Set PWM Period on Primary Time Base */
    PHASE1 = 0x445B;
    PHASE2 = 0x445B;
    PHASE3 = 0x445B;
    PHASE4 = 0x445B;
    PDC4 = 0x00;
    IOCON1 = 0xC000;
    IOCON2 = 0xC000;
    IOCON3 = 0xC000;
    IOCON4 = 0xC000;
    IOCON5 = 0x00;
    IOCON6 = 0x00;
    FCLCON1 = 0x03;
    FCLCON2 = 0x03;
    FCLCON3 = 0x03;
    FCLCON4 = 0x03;
    PTPER = 34999;
    SEVTCMP = 3499;
    PWMCON1 = 0x0284;
    PWMCON2 = 0x0284;
    PWMCON3 = 0x0284;
    PWMCON4 = 0x0284;
    PTCON2 = 0x02;

    /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S12>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
    /* Set-up I2C 2 peripheral */
    I2C2BRG = 0xA5;                    /* I2C clock = 399772  (400000 with  0.0 \% error) */
    I2C2CON = 0x8300;
    LATAbits.LATA3 = 0;                /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
    LATAbits.LATA2 = 0;
    _MI2C2IP = 6;                      /* Set I2C Master Interrupt Priority */
    _MI2C2IF = 0;
    _MI2C2IE = 1;

    /* Start for Enabled SubSystem: '<S537>/Enabled Subsystem' */
    AUA_EnabledSubsystem_Start
      (&AUAV3_WITH_SLUGS_SENSOR_AND_C_B.EnabledSubsystem_h);

    /* End of Start for SubSystem: '<S537>/Enabled Subsystem' */

    /* Start for S-Function (MCHP_BUS_SPI): '<S13>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (1 kHz)' */
    /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
    SPI1CON1 = 0x0474;
    SPI1CON2 = 0x01;
    SPI1STAT = 0x8014;
    _SPI1IP = 5;                       /* Set SPI Interrupt Priority */
    _SPI1IF = 0;                       /* Reset interrupt Flag */
    _SPI1IE = 1;                       /* Enable Interrupts */

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

    /* MCHP_Digital_Output_Write Block: '<S434>/Digital Output Write' */
    LATGbits.LATG8 = false;

    /* MCHP_Digital_Output_Write Block: '<S435>/Digital Output Write' */
    LATGbits.LATG9 = false;

    /* Start for S-Function (MCHP_IC): '<S15>/Input Capture RC Receiver1' */
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

    /* Start for S-Function (MCHP_OC_HW): '<S15>/Output Compare - HW Drive Servo motor' */
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
    if (MCHP_SPI12_Request == 0)       /* Last SPI sequence from this block is finished (not in the queue ?) */
    {
      MCHP_SPI12_Request = 1;
      MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 1;
      if (MCHP_SPI1_Queue.head >= 2)   /* There are 2 blocks SPI1, max idx for queue is 2 */
        MCHP_SPI1_Queue.head = 0;
      else
        MCHP_SPI1_Queue.head ++;
      if (MCHP_SPI1_State == 0)
        _SPI1IF = 1;                   /* Force Interrupt */
    }

    /* Start for S-Function (MCHP_C_function_Call): '<Root>/Initialize Control MCU' */
    controlMCUInit(
                   );

    /* MCHP_UART_Config Block for UART 1: <Root>/UART Configuration UAV V3 UART 3/Initialize */
    /* Initialisation sequence for UART 1 */
    {
      const uint8_T InitSequence[11] = { 72, 101, 108, 108, 111, 32, 87, 111,
        114, 108, 100 };

      U1BRG = 0x9B5C;                  /* Baud rate: 110 (-0.00%) */
      U1MODE = 0x8000;
      U1STA = 0x2400;
      __delay32(1909091);              /* Wait for 1909091 cycles */

      {
        uint_T i1;
        for (i1 = 0; i1 < 11 ; i1++) {
          while (U1STAbits.UTXBF == 1) ;/* Wait for one empty space within buffer UART */
          U1TXREG = InitSequence[i1];
        }
      }

      while (U1STAbits.TRMT == 0) ;    /* Wait for all value to be sent */
      U1MODE = 0;                      /* Then switch off UART */
    }

    U1BRG = 0x9B5C;                    /* Baud rate: 110 (-0.00%) */
    U1MODE = 0x8000;
    U1STA = 0x2400;

    /* Configure UART1 Tx Interruption */
    MCHP_UART1_Tx.head = 0;            /* Initialise Circular Buffers */
    MCHP_UART1_Tx.tail = 0;
    _U1TXIP = 1;                       /*  Tx Interrupt priority set to 1 */
    _U1TXIF = 0;                       /*  */
    _U1TXIE = 1;                       /* Enable Interrupt */

    /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
    U4BRG = 0x9B5C;                    /* Baud rate: 110 (-0.00%) */
    U4MODE = 0x8000;
    U4STA = 0x2400;

    /* Configure UART4 Tx Interruption */
    MCHP_UART4_Tx.head = 0;            /* Initialise Circular Buffers */
    MCHP_UART4_Tx.tail = 0;
    _U4TXIP = 1;                       /*  Tx Interrupt priority set to 1 */
    _U4TXIF = 0;                       /*  */
    _U4TXIE = 1;                       /* Enable Interrupt */

    /* InitializeConditions for MATLAB Function: '<S634>/Buffer Failsafe Channel' */
    AUAV3_BufferICChannel_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferFailsafeChannel);

    /* InitializeConditions for MATLAB Function: '<S629>/Buffer IC Channel' */
    AUAV3_BufferICChannel_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel);

    /* InitializeConditions for MATLAB Function: '<S629>/Buffer IC Channel1' */
    AUAV3_BufferICChannel_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel1);

    /* InitializeConditions for MATLAB Function: '<S629>/Buffer IC Channel2' */
    AUAV3_BufferICChannel_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel2);

    /* InitializeConditions for MATLAB Function: '<S629>/Buffer IC Channel3' */
    AUAV3_BufferICChannel_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_BufferICChannel3);

    /* InitializeConditions for MATLAB Function: '<S511>/Embedded MATLAB Function3' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h_not_empty = false;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.lastGps_h = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S599>/Enables//Disables the Computation of  initial Baro Bias' */
    EnablesDisablestheCom_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EnablesDisablestheComputatio);

    /* InitializeConditions for MATLAB Function: '<S606>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction);

    /* InitializeConditions for MATLAB Function: '<S609>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_f);

    /* InitializeConditions for MATLAB Function: '<S522>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fw);

    /* InitializeConditions for DiscreteIntegrator: '<S436>/Discrete-Time Integrator1' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[0] = 1.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.DiscreteTimeIntegrator1_DSTATE[3] = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S573>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n);

    /* InitializeConditions for MATLAB Function: '<S573>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1);

    /* InitializeConditions for MATLAB Function: '<S573>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2);

    /* InitializeConditions for Delay: '<S447>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE[i] = 19.5F;
    }

    /* End of InitializeConditions for Delay: '<S447>/Integer Delay1' */

    /* InitializeConditions for MATLAB Function: '<S574>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_dv);

    /* InitializeConditions for MATLAB Function: '<S574>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_b);

    /* InitializeConditions for MATLAB Function: '<S574>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_i);

    /* InitializeConditions for Delay: '<S517>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      AUAV3_WITH_SLUGS_SENSOR_A_DWork.IntegerDelay1_DSTATE_h[i] = 19.5F;
    }

    /* End of InitializeConditions for Delay: '<S517>/Integer Delay1' */

    /* InitializeConditions for RateLimiter: '<S510>/Rate Limiter' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[0] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[1] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY[2] = 0.0F;

    /* InitializeConditions for RateLimiter: '<S436>/Bias Rate Limiter' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[0] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[1] = 0.0F;
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.PrevY_h[2] = 0.0F;

    /* InitializeConditions for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
    /* InitializeConditions for MATLAB Function: '<S127>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_hb);

    /* InitializeConditions for UnitDelay: '<S140>/FixPt Unit Delay2' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.FixPtUnitDelay2_DSTATE = 1U;

    /* InitializeConditions for MATLAB Function: '<S103>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n4);

    /* End of InitializeConditions for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

    /* InitializeConditions for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

    /* InitializeConditions for MATLAB Function: '<S66>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_n0);

    /* End of InitializeConditions for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

    /* InitializeConditions for MATLAB Function: '<S639>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fz);

    /* InitializeConditions for MATLAB Function: '<S640>/Embedded MATLAB Function' */
    EmbeddedMATLABFunctio_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_fx);

    /* InitializeConditions for Delay: '<S12>/Delay' */
    AUAV3_WITH_SLUGS_SENSOR_A_DWork.Delay_DSTATE = 1U;

    /* InitializeConditions for MATLAB Function: '<S537>/Enables//Disables the Computation of  initial Baro Bias' */
    EnablesDisablestheCom_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EnablesDisablestheComputat_h);

    /* InitializeConditions for MATLAB Function: '<S608>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_ht);

    /* InitializeConditions for MATLAB Function: '<S607>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_p);

    /* InitializeConditions for MATLAB Function: '<S575>/Embedded MATLAB Function' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction_ih);

    /* InitializeConditions for MATLAB Function: '<S575>/Embedded MATLAB Function1' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction1_g);

    /* InitializeConditions for MATLAB Function: '<S575>/Embedded MATLAB Function2' */
    EmbeddedMATLABFunct_i_Init
      (&AUAV3_WITH_SLUGS_SENSOR_A_DWork.sf_EmbeddedMATLABFunction2_n);
  }

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(AUAV3_WITH_SLUGS_SENSOR_AND__M)) {
    rtmSetFirstInitCond(AUAV3_WITH_SLUGS_SENSOR_AND__M, 0);
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
