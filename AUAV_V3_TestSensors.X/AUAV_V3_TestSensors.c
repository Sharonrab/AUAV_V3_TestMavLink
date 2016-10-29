/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV_V3_TestSensors.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.158
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Sep 01 18:16:13 2016
 */

#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* user code (top of source file) */
#include "gpsPort.h"
#include "MavlinkComm.h"

const mavlink_scaled_pressure_t AUAV_V3_TestSensors_rtZmavlink_scaled_pressure_t
  = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_attitude_t AUAV_V3_TestSensors_rtZmavlink_attitude_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* roll */
  0.0F,                                /* pitch */
  0.0F,                                /* yaw */
  0.0F,                                /* rollspeed */
  0.0F,                                /* pitchspeed */
  0.0F                                 /* yawspeed */
} ;                                    /* mavlink_attitude_t ground */

const mavlink_coordinate_float_t
  AUAV_V3_TestSensors_rtZmavlink_coordinate_float_t = {
  0U,                                  /* lat */
  0U,                                  /* lon */
  0U                                   /* alt */
} ;                                    /* mavlink_coordinate_float_t ground */

const mavlink_gps_raw_int_t AUAV_V3_TestSensors_rtZmavlink_gps_raw_int_t = {
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

const mavlink_heartbeat_t AUAV_V3_TestSensors_rtZmavlink_heartbeat_t = {
  0U,                                  /* custom_mode */
  0U,                                  /* type */
  0U,                                  /* autopilot */
  0U,                                  /* base_mode */
  0U,                                  /* system_status */
  0U                                   /* mavlink_version */
} ;                                    /* mavlink_heartbeat_t ground */

const mavlink_isr_location_t AUAV_V3_TestSensors_rtZmavlink_isr_location_t = {
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0.0F,                                /* height */
  0U,                                  /* option1 */
  0U,                                  /* option2 */
  0U                                   /* option3 */
} ;                                    /* mavlink_isr_location_t ground */

const mavlink_mid_lvl_cmds_t AUAV_V3_TestSensors_rtZmavlink_mid_lvl_cmds_t = {
  0.0F,                                /* hCommand */
  0.0F,                                /* uCommand */
  0.0F,                                /* rCommand */
  0U                                   /* target */
} ;                                    /* mavlink_mid_lvl_cmds_t ground */

const mavlink_slugs_mobile_location_t
  AUAV_V3_TestSensors_rtZmavlink_slugs_mobile_location_t = {
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0U                                   /* target */
} ;                                    /* mavlink_slugs_mobile_location_t ground */

const mavlink_slugs_navigation_t
  AUAV_V3_TestSensors_rtZmavlink_slugs_navigation_t = {
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

const pi_struct AUAV_V3_TestSensors_rtZpi_struct = {
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

const mavlink_raw_imu_t AUAV_V3_TestSensors_rtZmavlink_raw_imu_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* xacc */
  0,                                   /* yacc */
  0,                                   /* zacc */
  0,                                   /* xgyro */
  0,                                   /* ygyro */
  0,                                   /* zgyro */
  0,                                   /* xmag */
  0,                                   /* ymag */
  0                                    /* zmag */
} ;                                    /* mavlink_raw_imu_t ground */

const mavlink_raw_pressure_t AUAV_V3_TestSensors_rtZmavlink_raw_pressure_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_rc_channels_raw_t AUAV_V3_TestSensors_rtZmavlink_rc_channels_raw_t
  = {
  0U,                                  /* time_usec */
  0U,                                  /* chan1_raw */
  0U,                                  /* chan2_raw */
  0U,                                  /* chan3_raw */
  0U,                                  /* chan4_raw */
  0U,                                  /* chan5_raw */
  0U,                                  /* chan6_raw */
  0U,                                  /* chan7_raw */
  0U,                                  /* chan8_raw */
  0U,                                  /* port */
  0U                                   /* rssi */
} ;                                    /* mavlink_rc_channels_raw_t ground */

const mavlink_servo_output_raw_t
  AUAV_V3_TestSensors_rtZmavlink_servo_output_raw_t = {
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

const mavlink_sys_status_t AUAV_V3_TestSensors_rtZmavlink_sys_status_t = {
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

const mavlink_volt_sensor_t AUAV_V3_TestSensors_rtZmavlink_volt_sensor_t = {
  0U,                                  /* voltage */
  0U,                                  /* reading2 */
  0U                                   /* r2Type */
} ;                                    /* mavlink_volt_sensor_t ground */

const mavlink_vfr_hud_t AUAV_V3_TestSensors_rtZmavlink_vfr_hud_t = {
  0.0F,                                /* airspeed */
  0.0F,                                /* groundspeed */
  0.0F,                                /* alt */
  0.0F,                                /* climb */
  0,                                   /* heading */
  0U                                   /* throttle */
} ;                                    /* mavlink_vfr_hud_t ground */

const mavlink_mission_item_values_t
  AUAV_V3_TestSensors_rtZmavlink_mission_item_values_t = {
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
mavlink_raw_imu_t mlRawIMU;            /* '<Root>/mlRawIMU' */
mavlink_rc_channels_raw_t mlRC_Commands;/* '<Root>/mlRawRC' */
mavlink_servo_output_raw_t mlPwmCommands;/* '<Root>/mlRawServo' */
mavlink_vfr_hud_t mlVfr_hud;           /* '<Root>/mlVfr_hud' */
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
BlockIO_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_B;

/* Block states (auto storage) */
D_Work_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_DWork;

/* Real-time model */
RT_MODEL_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_M_;
RT_MODEL_AUAV_V3_TestSensors_T *const AUAV_V3_TestSensors_M =
  &AUAV_V3_TestSensors_M_;
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
void AUAV_V3_TestSensors_step0(void)   /* Sample time: [0.01s, 0.0s] */
{
  uint64m_T tmp;
  uint64m_T tmp_0;
  uint32_T tmp_1;

  /* Update the flag to indicate when data transfers from
   *  Sample time: [0.01s, 0.0s] to Sample time: [0.02s, 0.0s]  */
  (AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1)++;
  if ((AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1) > 1) {
    AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1 = 0;
  }

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S3>/Output'
   */
  tmp_1 = 3276800000UL;
  uMultiWordMul(&tmp_1, 1, &AUAV_V3_TestSensors_DWork.Output_DSTATE, 1,
                &tmp_0.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_0.chunks[0U], 2, 15U, &tmp.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV_V3_TestSensors_DWork.time_since_boot_usec = MultiWord2uLong(&tmp.chunks
    [0U]);

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  AUAV_V3_TestSensors_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = AUAV_V3_TestSensors_B.U3CH4;

  /* Switch: '<S54>/FixPt Switch' incorporates:
   *  Constant: '<S53>/FixPt Constant'
   *  Sum: '<S53>/FixPt Sum1'
   *  UnitDelay: '<S3>/Output'
   */
  AUAV_V3_TestSensors_DWork.Output_DSTATE++;

  /* Outputs for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  AUAV_V_Sensor_Data_Adapter();

  /* End of Outputs for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Outputs for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Filt();

  /* End of Outputs for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* Outputs for Atomic SubSystem: '<Root>/Barometer_Driver' */

  /* MATLAB Function 'myMux Fun2': '<S21>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S21>:1:5' */
  AUAV_V3_T_Barometer_Driver();

  /* End of Outputs for SubSystem: '<Root>/Barometer_Driver' */

  /* Outputs for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  AUAV_V3_Tes_IMU_Mag_DriverTID0();

  /* End of Outputs for SubSystem: '<Root>/IMU_Mag_Driver' */

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  AUAV_V3_TestSensors_M->Timing.clockTick0++;
}

/* Model step function for TID1 */
void AUAV_V3_TestSensors_step1(void)   /* Sample time: [0.02s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  AUAV_V3_Tes_IMU_Mag_DriverTID1();

  /* End of Outputs for SubSystem: '<Root>/IMU_Mag_Driver' */

  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID1();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */

  /* Update for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  AUAV_IMU_Mag_Driver_UpdateTID1();

  /* End of Update for SubSystem: '<Root>/IMU_Mag_Driver' */
}

/* Model step function for TID2 */
void AUAV_V3_TestSensors_step2(void)   /* Sample time: [0.05s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID2();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID3 */
void AUAV_V3_TestSensors_step3(void)   /* Sample time: [0.1s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID3();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID4 */
void AUAV_V3_TestSensors_step4(void)   /* Sample time: [0.2s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID4();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */

  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );

  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID5 */
void AUAV_V3_TestSensors_step5(void)   /* Sample time: [0.2s, 0.02s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID5();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID6 */
void AUAV_V3_TestSensors_step6(void)   /* Sample time: [0.2s, 0.04s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID6();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID7 */
void AUAV_V3_TestSensors_step7(void)   /* Sample time: [0.2s, 0.06s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID7();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID8 */
void AUAV_V3_TestSensors_step8(void)   /* Sample time: [0.2s, 0.08s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID8();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID9 */
void AUAV_V3_TestSensors_step9(void)   /* Sample time: [0.25s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID9();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID10 */
void AUAV_V3_TestSensors_step10(void)  /* Sample time: [0.5s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID10();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */

  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID10();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */

  /* S-Function (MCHP_BUS_I2C_MASTER): '<Root>/BUS I2C Initialize HMC5883 re-initialize at 0.5Hz1' */
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
void AUAV_V3_TestSensors_step11(void)  /* Sample time: [1.0s, 0.0s] */
{
  /* (no output/update code required) */
}

/* Model initialize function */
void AUAV_V3_TestSensors_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)AUAV_V3_TestSensors_M, 0,
                sizeof(RT_MODEL_AUAV_V3_TestSensors_T));
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[0] = 1;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[1] = 2;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[2] = 5;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[3] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[4] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[5] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[6] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[7] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[8] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[9] = 25;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[10] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[11] = 100;

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[5] = 18;/* Sample time: [0.2s, 0.02s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[6] = 16;/* Sample time: [0.2s, 0.04s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[7] = 14;/* Sample time: [0.2s, 0.06s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[8] = 12;/* Sample time: [0.2s, 0.08s] */

  /* block I/O */
  (void) memset(((void *) &AUAV_V3_TestSensors_B), 0,
                sizeof(BlockIO_AUAV_V3_TestSensors_T));

  /* states (dwork) */
  (void) memset((void *)&AUAV_V3_TestSensors_DWork, 0,
                sizeof(D_Work_AUAV_V3_TestSensors_T));

  /* exported global states */
  mlParamInterface = AUAV_V3_TestSensors_rtZpi_struct;
  mlWpValues = AUAV_V3_TestSensors_rtZmavlink_mission_item_values_t;
  mlAttitude = AUAV_V3_TestSensors_rtZmavlink_attitude_t;
  mlGpsData = AUAV_V3_TestSensors_rtZmavlink_gps_raw_int_t;
  mlNavigation = AUAV_V3_TestSensors_rtZmavlink_slugs_navigation_t;
  mlSysStatus = AUAV_V3_TestSensors_rtZmavlink_sys_status_t;
  mlRawIMU = AUAV_V3_TestSensors_rtZmavlink_raw_imu_t;
  mlRC_Commands = AUAV_V3_TestSensors_rtZmavlink_rc_channels_raw_t;
  mlPwmCommands = AUAV_V3_TestSensors_rtZmavlink_servo_output_raw_t;
  mlVfr_hud = AUAV_V3_TestSensors_rtZmavlink_vfr_hud_t;
  mlAirData = AUAV_V3_TestSensors_rtZmavlink_scaled_pressure_t;
  mlGSLocationFloat = AUAV_V3_TestSensors_rtZmavlink_coordinate_float_t;
  mlHeartbeatLocal = AUAV_V3_TestSensors_rtZmavlink_heartbeat_t;
  mlISR = AUAV_V3_TestSensors_rtZmavlink_isr_location_t;
  mlMidLevelCommands = AUAV_V3_TestSensors_rtZmavlink_mid_lvl_cmds_t;
  mlMobileLocation = AUAV_V3_TestSensors_rtZmavlink_slugs_mobile_location_t;
  mlRawPressureData = AUAV_V3_TestSensors_rtZmavlink_raw_pressure_t;
  mlVISensor = AUAV_V3_TestSensors_rtZmavlink_volt_sensor_t;
  MPU_T = 0U;

  /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

  /* Start for S-Function (MCHP_MCU_LOAD): '<Root>/MCU Load' */
  TMR2 = 0;                            /* Initialize Timer 2 Value to 0.  Timer 2 is enabled only when the mcu is not idle */

  /* Start for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  Sensor_Data_Adapter_Start();

  /* End of Start for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Start for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitud_Start();

  /* End of Start for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* MCHP_Digital_Output_Write Block: '<S468>/Digital Output Write' */
  LATBbits.LATB6 = false;

  /* MCHP_Digital_Output_Write Block: '<S469>/Digital Output Write' */
  LATEbits.LATE9 = false;

  /* MCHP_Digital_Output_Write Block: '<S470>/Digital Output Write' */
  LATGbits.LATG8 = false;

  /* MCHP_Digital_Output_Write Block: '<S471>/Digital Output Write' */
  LATGbits.LATG9 = false;

  /* Start for Atomic SubSystem: '<Root>/Barometer_Driver' */
  AUA_Barometer_Driver_Start();

  /* End of Start for SubSystem: '<Root>/Barometer_Driver' */

  /* Start for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  AUAV__IMU_Mag_Driver_Start();

  /* End of Start for SubSystem: '<Root>/IMU_Mag_Driver' */

  /* Start for S-Function (MCHP_BUS_SPI): '<Root>/BUS SPI Initialize MPU 6000 Once at Startup Gyro:+-500°//s (65535=>1000) Accelero: +-2G (65535=>4G)' */
  /* SPI Initialisation sequence executed once */
  /* number of SPI blocks : 2 ; Current: 2 ; MCHP_SPI_StartImplemented =  2*/
  if (MCHP_SPI12_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    MCHP_SPI12_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 3;
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
  _U1TXIP = 1;                         /*  Tx Interrupt priority set to 1 */
  _U1TXIF = 0;                         /*  */
  _U1TXIE = 1;                         /* Enable Interrupt */

  /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
  U4BRG = 0x71;                        /* Baud rate: 38400 (-0.06%) */
  U4MODE = 0x8000;
  U4STA = 0x2400;

  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 1;                         /*  Tx Interrupt priority set to 1 */
  _U4TXIF = 0;                         /*  */
  _U4TXIE = 1;                         /* Enable Interrupt */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  A_Sensor_Data_Adapter_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Barometer_Driver' */
  AUAV_Barometer_Driver_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Barometer_Driver' */

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
