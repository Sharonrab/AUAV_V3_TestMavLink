/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: SLUGS2.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.291
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Jun 24 14:00:22 2017
 */

#include "SLUGS2.h"
#include "SLUGS2_private.h"

/* user code (top of source file) */
#include "gpsPort.h"
#include "MavlinkComm.h"

const mavlink_scaled_pressure_t SLUGS2_rtZmavlink_scaled_pressure_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_attitude_t SLUGS2_rtZmavlink_attitude_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* roll */
  0.0F,                                /* pitch */
  0.0F,                                /* yaw */
  0.0F,                                /* rollspeed */
  0.0F,                                /* pitchspeed */
  0.0F                                 /* yawspeed */
} ;                                    /* mavlink_attitude_t ground */

const mavlink_coordinate_float_t SLUGS2_rtZmavlink_coordinate_float_t = {
  0.0F,                                /* lat */
  0.0F,                                /* lon */
  0.0F                                 /* alt */
} ;                                    /* mavlink_coordinate_float_t ground */

const mavlink_gps_raw_int_t SLUGS2_rtZmavlink_gps_raw_int_t = {
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

const mavlink_heartbeat_t SLUGS2_rtZmavlink_heartbeat_t = {
  0U,                                  /* custom_mode */
  0U,                                  /* type */
  0U,                                  /* autopilot */
  0U,                                  /* base_mode */
  0U,                                  /* system_status */
  0U                                   /* mavlink_version */
} ;                                    /* mavlink_heartbeat_t ground */

const mavlink_isr_location_t SLUGS2_rtZmavlink_isr_location_t = {
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0.0F,                                /* height */
  0U,                                  /* option1 */
  0U,                                  /* option2 */
  0U                                   /* option3 */
} ;                                    /* mavlink_isr_location_t ground */

const mavlink_local_position_ned_t SLUGS2_rtZmavlink_local_position_ned_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* x */
  0.0F,                                /* y */
  0.0F,                                /* z */
  0.0F,                                /* vx */
  0.0F,                                /* vy */
  0.0F                                 /* vz */
} ;                                    /* mavlink_local_position_ned_t ground */

const mavlink_mid_lvl_cmds_t SLUGS2_rtZmavlink_mid_lvl_cmds_t = {
  0.0F,                                /* hCommand */
  0.0F,                                /* uCommand */
  0.0F,                                /* rCommand */
  0U                                   /* target */
} ;                                    /* mavlink_mid_lvl_cmds_t ground */

const mavlink_slugs_mobile_location_t SLUGS2_rtZmavlink_slugs_mobile_location_t =
{
  0.0F,                                /* latitude */
  0.0F,                                /* longitude */
  0U                                   /* target */
} ;                                    /* mavlink_slugs_mobile_location_t ground */

const mavlink_slugs_navigation_t SLUGS2_rtZmavlink_slugs_navigation_t = {
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

const pi_struct SLUGS2_rtZpi_struct = {
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

const mavlink_rc_channels_raw_t SLUGS2_rtZmavlink_rc_channels_raw_t = {
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

const mavlink_raw_imu_t SLUGS2_rtZmavlink_raw_imu_t = {
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

const mavlink_raw_pressure_t SLUGS2_rtZmavlink_raw_pressure_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_servo_output_raw_t SLUGS2_rtZmavlink_servo_output_raw_t = {
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

const mavlink_sys_status_t SLUGS2_rtZmavlink_sys_status_t = {
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

const mavlink_volt_sensor_t SLUGS2_rtZmavlink_volt_sensor_t = {
  0U,                                  /* voltage */
  0U,                                  /* reading2 */
  0U                                   /* r2Type */
} ;                                    /* mavlink_volt_sensor_t ground */

const mavlink_vfr_hud_t SLUGS2_rtZmavlink_vfr_hud_t = {
  0.0F,                                /* airspeed */
  0.0F,                                /* groundspeed */
  0.0F,                                /* alt */
  0.0F,                                /* climb */
  0,                                   /* heading */
  0U                                   /* throttle */
} ;                                    /* mavlink_vfr_hud_t ground */

const mavlink_mission_item_values_t SLUGS2_rtZmavlink_mission_item_values_t = {
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
mavlink_attitude_t mlAttitudeData;     /* '<Root>/mlAttitudeData' */
mavlink_attitude_t mlAttitudeSol;      /* '<Root>/mlAttitudeData1' */
mavlink_gps_raw_int_t mlGpsData;       /* '<Root>/mlGpsData' */
mavlink_local_position_ned_t mlLocalPositionData;/* '<Root>/mlLocalPositionData' */
mavlink_slugs_navigation_t mlNavigation;/* '<Root>/mlNavigation' */
mavlink_sys_status_t mlSysStatus;      /* '<Root>/mlSysStatus' */
mavlink_rc_channels_raw_t mlPilotConsoleData;/* '<Root>/mlPilotConsoleData' */
mavlink_raw_imu_t mlRawImuData;        /* '<Root>/mlRawImuData' */
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
BlockIO_SLUGS2_T SLUGS2_B;

/* Block states (auto storage) */
D_Work_SLUGS2_T SLUGS2_DWork;

/* Real-time model */
RT_MODEL_SLUGS2_T SLUGS2_M_;
RT_MODEL_SLUGS2_T *const SLUGS2_M = &SLUGS2_M_;
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
 *    '<S167>/negprotect'
 *    '<S446>/negprotect'
 *    '<S304>/negprotect'
 *    '<S328>/negprotect'
 *    '<S335>/negprotect'
 *    '<S383>/negprotect'
 *    '<S390>/negprotect'
 *    '<S430>/negprotect'
 *    '<S289>/negprotect'
 *    '<S218>/negprotect'
 *    ...
 */
void SLUGS2_negprotect_l(real32_T rtu_val, rtB_negprotect_SLUGS2_p_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/negprotect': '<S169>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S169>:1:5' */
    /* '<S169>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S169>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S445>/Embedded MATLAB Function'
 *    '<S327>/Embedded MATLAB Function'
 *    '<S334>/Embedded MATLAB Function'
 *    '<S382>/Embedded MATLAB Function'
 *    '<S389>/Embedded MATLAB Function'
 *    '<S429>/Embedded MATLAB Function'
 *    '<S288>/Embedded MATLAB Function'
 *    '<S217>/Embedded MATLAB Function'
 *    '<S230>/Embedded MATLAB Function'
 *    '<S243>/Embedded MATLAB Function'
 *    ...
 */
void S_EmbeddedMATLABFunction_b(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_b_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S447>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S447>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Output and update for atomic system:
 *    '<S181>/Zero out Z1'
 *    '<S296>/Zero out Z1'
 *    '<S296>/Zero out Z2'
 *    '<S296>/Zero out Z3'
 *    '<S180>/Zero out Z2'
 *    '<S191>/Zero out Z2'
 *    '<S270>/Zero out Z1'
 *    '<S173>/Zero out Z1'
 *    '<S173>/Zero out Z2'
 */
void SLUGS2_ZerooutZ1(const real32_T rtu_Pin[3], rtB_ZerooutZ1_SLUGS2_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Zero out Z1': '<S438>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S438>:1:5' */
  localB->P[0] = rtu_Pin[0];
  localB->P[1] = rtu_Pin[1];
  localB->P[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S309>/Embedded MATLAB Function'
 *    '<S302>/Embedded MATLAB Function'
 *    '<S214>/Embedded MATLAB Function'
 *    '<S227>/Embedded MATLAB Function'
 */
void S_EmbeddedMATLABFunction_k(const real32_T rtu_x[3], const real32_T rtu_y[3],
  rtB_EmbeddedMATLABFunction_i_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S310>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S310>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_y[0] + rtu_x[1] * rtu_y[1]) + rtu_x[2] *
    rtu_y[2];
}

/*
 * Output and update for atomic system:
 *    '<S317>/Select N  Terms'
 *    '<S375>/Select N  Terms'
 */
void SLUGS2_SelectNTerms(const real32_T rtu_T[3], rtB_SelectNTerms_SLUGS2_T
  *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms': '<S325>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S325>:1:5' */
  localB->N[0] = -rtu_T[1];
  localB->N[1] = rtu_T[0];
  localB->N[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S205>/negprotect3'
 *    '<S206>/negprotect3'
 *    '<S193>/negprotect1'
 *    '<S193>/negprotect2'
 *    '<S262>/negprotect'
 *    '<S263>/negprotect'
 */
void SLUGS2_negprotect3(real32_T rtu_val, rtB_negprotect3_SLUGS2_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3': '<S216>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.0001F) {
    /* '<S216>:1:5' */
    /* '<S216>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S216>:1:8' */
    localB->zpVal = 0.0001F;
  }
}

/*
 * Initial conditions for atomic system:
 *    '<S643>/Buffer IC Channel'
 *    '<S643>/Buffer IC Channel1'
 *    '<S643>/Buffer IC Channel2'
 *    '<S643>/Buffer IC Channel3'
 *    '<S648>/Buffer Failsafe Channel'
 */
void SLUGS_BufferICChannel_Init(rtDW_BufferICChannel_SLUGS2_T *localDW)
{
  int16_T i;
  for (i = 0; i < 7; i++) {
    localDW->oldValues[i] = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S643>/Buffer IC Channel'
 *    '<S643>/Buffer IC Channel1'
 *    '<S643>/Buffer IC Channel2'
 *    '<S643>/Buffer IC Channel3'
 *    '<S648>/Buffer Failsafe Channel'
 */
void SLUGS2_BufferICChannel(uint16_T rtu_latest, rtB_BufferICChannel_SLUGS2_T
  *localB, rtDW_BufferICChannel_SLUGS2_T *localDW)
{
  int16_T i;

  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel': '<S649>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S649>:1:11' */
  for (i = 0; i < 7; i++) {
    localB->history[i] = 0U;
  }

  /* '<S649>:1:13' */
  for (i = 0; i < 6; i++) {
    /* '<S649>:1:13' */
    /* '<S649>:1:14' */
    localDW->oldValues[6 - i] = localDW->oldValues[5 - i];

    /* '<S649>:1:15' */
    localB->history[6 - i] = localDW->oldValues[5 - i];

    /* '<S649>:1:13' */
  }

  /* '<S649>:1:18' */
  localDW->oldValues[0] = rtu_latest;

  /* '<S649>:1:19' */
  localB->history[0] = rtu_latest;
}

/*
 * Output and update for atomic system:
 *    '<S677>/myMux Fun1'
 *    '<S675>/myMux Fun1'
 */
void SLUGS2_myMuxFun1_h(real32_T rtu_u1, real32_T rtu_u2, real32_T rtu_u3,
  real32_T rtu_u4, real32_T rty_y[4])
{
  /* MATLAB Function 'Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1/myMux Fun1': '<S696>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S696>:1:5' */
  rty_y[0] = rtu_u1;
  rty_y[1] = rtu_u2;
  rty_y[2] = rtu_u3;
  rty_y[3] = rtu_u4;
}

/*
 * Output and update for atomic system:
 *    '<Root>/myMux Fun3'
 *    '<Root>/myMux Fun4'
 */
void SLUGS2_myMuxFun3(const real32_T rtu_u1[3], const real32_T rtu_u2[3],
                      rtB_myMuxFun3_SLUGS2_T *localB)
{
  /* MATLAB Function 'myMux Fun3': '<S20>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S20>:1:5' */
  localB->y[0] = rtu_u1[0];
  localB->y[1] = rtu_u1[1];
  localB->y[2] = rtu_u1[2];
  localB->y[3] = rtu_u2[0];
  localB->y[4] = rtu_u2[1];
  localB->y[5] = rtu_u2[2];
}

/* Model step function for TID0 */
void SLUGS2_step0(void)                /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  boolean_T rtb_ConfigUART4RxPin;
  real32_T rtb_Product3_j0[3];
  real32_T rtb_VectorConcatenate[9];
  real32_T rtb_Product2_c[3];
  real32_T rtb_VectorConcatenate_m[9];
  real32_T rtb_VectorConcatenate_h[7];
  uint8_T rtb_DataTypeConversion1_hq;
  boolean_T rtb_IC1_a;
  uint16_T rtb_u2deg;
  uint8_T rtb_Compare_j;
  boolean_T rtb_IC1;
  uint8_T rtb_DataTypeConversion2_fi;
  uint8_T rtb_IC2;
  real32_T rtb_Product3_bo;
  real32_T rtb_Switch3;
  real32_T rtb_Projection;
  real32_T rtb_Switch3_gd;
  real32_T rtb_Subtract_a;
  real32_T rtb_Subtract_od;
  real32_T rtb_Switch1_a;
  uint16_T rtb_DataTypeConversion_lk;
  uint16_T rtb_DataTypeConversion_ee;
  uint16_T rtb_DataTypeConversion_g0;
  real32_T rtb_Merge_m[3];
  real32_T rtb_Ze_b;
  real32_T rtb_cosphi;
  real32_T rtb_Sum1_mj;
  real32_T rtb_Deg2R1;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_RhhcosphicoslambXe;
  real32_T rtb_Product;
  real32_T rtb_Switch3_kh;
  real32_T rtb_P32[3];
  real32_T rtb_MathFunction[9];
  int16_T i;
  int16_T i_0;
  real32_T tmp[9];
  real32_T tmp_0[3];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[9];
  real32_T rtb_IC4_idx_1;
  real32_T rtb_IC4_idx_0;
  real32_T rtb_Deg2R_h_idx_0;
  real32_T rtb_y_jl_idx_1;
  real32_T rtb_y_jl_idx_0;
  real_T tmp_4;
  uint64m_T tmp_5;
  uint64m_T tmp_6;
  uint32_T tmp_7;

  /* Update the flag to indicate when data transfers from
   *  Sample time: [0.01s, 0.0s] to Sample time: [0.02s, 0.0s]  */
  (SLUGS2_M->Timing.RateInteraction.TID0_1)++;
  if ((SLUGS2_M->Timing.RateInteraction.TID0_1) > 1) {
    SLUGS2_M->Timing.RateInteraction.TID0_1 = 0;
  }

  /* Gain: '<S697>/Unit Conversion' incorporates:
   *  DataStoreRead: '<S17>/PAR_CONFIG_ROLL_R PAR_CONFIG_PITCH_R PAR_CONFIG_YAW_R'
   */
  rtb_Product3_j0[0] = 0.0174532924F * mlParamInterface.param[15L];
  rtb_Product3_j0[1] = 0.0174532924F * mlParamInterface.param[16L];
  rtb_Product3_j0[2] = 0.0174532924F * mlParamInterface.param[17L];

  /* Trigonometry: '<S700>/SinCos' */
  rtb_Product2_c[0] = (real32_T)cos(rtb_Product3_j0[0]);
  rtb_Product3_j0[0] = (real32_T)sin(rtb_Product3_j0[0]);
  rtb_Product2_c[1] = (real32_T)cos(rtb_Product3_j0[1]);
  rtb_Product3_j0[1] = (real32_T)sin(rtb_Product3_j0[1]);
  rtb_RhhcosphisinlambYe = (real32_T)cos(rtb_Product3_j0[2]);
  rtb_Sum1_mj = (real32_T)sin(rtb_Product3_j0[2]);

  /* Product: '<S707>/u(5)*u(6)' incorporates:
   *  Trigonometry: '<S700>/SinCos'
   */
  rtb_VectorConcatenate[0] = rtb_Product2_c[1] * rtb_RhhcosphisinlambYe;

  /* Sum: '<S710>/Sum' incorporates:
   *  Product: '<S710>/u(3)*u(4)'
   *  Product: '<S710>/u(6)*u(1)*u(2)'
   *  Trigonometry: '<S700>/SinCos'
   */
  rtb_VectorConcatenate[1] = rtb_RhhcosphisinlambYe * rtb_Product3_j0[0] *
    rtb_Product3_j0[1] - rtb_Sum1_mj * rtb_Product2_c[0];

  /* Sum: '<S713>/Sum' incorporates:
   *  Product: '<S713>/u(1)*u(3)'
   *  Product: '<S713>/u(2)*u(4)*u(6)'
   *  Trigonometry: '<S700>/SinCos'
   */
  rtb_VectorConcatenate[2] = rtb_Product3_j0[1] * rtb_Product2_c[0] *
    rtb_RhhcosphisinlambYe + rtb_Product3_j0[0] * rtb_Sum1_mj;

  /* Product: '<S708>/u(3)*u(5)' */
  rtb_VectorConcatenate[3] = rtb_Sum1_mj * rtb_Product2_c[1];

  /* Sum: '<S711>/Sum' incorporates:
   *  Product: '<S711>/u(1)*u(2)*u(3)'
   *  Product: '<S711>/u(4)*u(6)'
   *  Trigonometry: '<S700>/SinCos'
   */
  rtb_VectorConcatenate[4] = rtb_Product3_j0[0] * rtb_Product3_j0[1] *
    rtb_Sum1_mj + rtb_Product2_c[0] * rtb_RhhcosphisinlambYe;

  /* Sum: '<S714>/Sum' incorporates:
   *  Product: '<S714>/u(1)*u(6)'
   *  Product: '<S714>/u(2)*u(3)*u(4)'
   *  Trigonometry: '<S700>/SinCos'
   */
  rtb_VectorConcatenate[5] = rtb_Product3_j0[1] * rtb_Sum1_mj * rtb_Product2_c[0]
    - rtb_RhhcosphisinlambYe * rtb_Product3_j0[0];

  /* Gain: '<S709>/Gain2' */
  rtb_VectorConcatenate[6] = -rtb_Product3_j0[1];

  /* Product: '<S712>/u(1)*u(3)' */
  rtb_VectorConcatenate[7] = rtb_Product3_j0[0] * rtb_Product2_c[1];

  /* Product: '<S715>/u(4)*u(5)' */
  rtb_VectorConcatenate[8] = rtb_Product2_c[0] * rtb_Product2_c[1];

  /* Math: '<S17>/Math Function' */
  for (i = 0; i < 3; i++) {
    rtb_MathFunction[3 * i] = rtb_VectorConcatenate[i];
    rtb_MathFunction[1 + 3 * i] = rtb_VectorConcatenate[i + 3];
    rtb_MathFunction[2 + 3 * i] = rtb_VectorConcatenate[i + 6];
  }

  /* End of Math: '<S17>/Math Function' */

  /* DataStoreRead: '<Root>/Data Store Read' */
  SLUGS2_B.DataStoreRead = SLUGS2_DWork.X_PLANE_HIL_FLAG;

  /* Outputs for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  SLUGS2_Sensor_Data_Adapter();

  /* End of Outputs for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Outputs for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Filt();

  /* End of Outputs for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* MATLAB Function: '<Root>/myMux Fun2' incorporates:
   *  SignalConversion: '<S19>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'myMux Fun2': '<S19>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S19>:1:5' */
  SLUGS2_B.y_k[0] = SLUGS2_B.DataTypeConversion_m[2];
  SLUGS2_B.y_k[1] = SLUGS2_B.DataTypeConversion_m[1];
  SLUGS2_B.y_k[2] = SLUGS2_B.sf_EmbeddedMATLABFunction_lt.y;
  SLUGS2_B.y_k[3] = SLUGS2_B.GyroErr[0];
  SLUGS2_B.y_k[4] = SLUGS2_B.GyroErr[1];
  SLUGS2_B.y_k[5] = SLUGS2_B.GyroErr[2];

  /* Trigonometry: '<S701>/SinCos' */
  rtb_Product2_c[0] = (real32_T)sin(SLUGS2_B.y_k[0]);
  rtb_Product3_j0[0] = (real32_T)cos(SLUGS2_B.y_k[0]);
  rtb_Product2_c[1] = (real32_T)sin(SLUGS2_B.y_k[1]);
  rtb_Product3_j0[1] = (real32_T)cos(SLUGS2_B.y_k[1]);
  rtb_Product2_c[2] = (real32_T)sin(SLUGS2_B.y_k[2]);
  rtb_Product3_j0[2] = (real32_T)cos(SLUGS2_B.y_k[2]);

  /* Product: '<S717>/u(5)*u(6)' */
  rtb_VectorConcatenate_m[0] = rtb_Product3_j0[1] * rtb_Product3_j0[2];

  /* Sum: '<S720>/Sum' incorporates:
   *  Product: '<S720>/u(3)*u(4)'
   *  Product: '<S720>/u(6)*u(1)*u(2)'
   */
  rtb_VectorConcatenate_m[1] = rtb_Product3_j0[2] * rtb_Product2_c[0] *
    rtb_Product2_c[1] - rtb_Product2_c[2] * rtb_Product3_j0[0];

  /* Sum: '<S723>/Sum' incorporates:
   *  Product: '<S723>/u(1)*u(3)'
   *  Product: '<S723>/u(2)*u(4)*u(6)'
   */
  rtb_VectorConcatenate_m[2] = rtb_Product2_c[1] * rtb_Product3_j0[0] *
    rtb_Product3_j0[2] + rtb_Product2_c[0] * rtb_Product2_c[2];

  /* Product: '<S718>/u(3)*u(5)' */
  rtb_VectorConcatenate_m[3] = rtb_Product2_c[2] * rtb_Product3_j0[1];

  /* Sum: '<S721>/Sum' incorporates:
   *  Product: '<S721>/u(1)*u(2)*u(3)'
   *  Product: '<S721>/u(4)*u(6)'
   */
  rtb_VectorConcatenate_m[4] = rtb_Product2_c[0] * rtb_Product2_c[1] *
    rtb_Product2_c[2] + rtb_Product3_j0[0] * rtb_Product3_j0[2];

  /* Sum: '<S724>/Sum' incorporates:
   *  Product: '<S724>/u(1)*u(6)'
   *  Product: '<S724>/u(2)*u(3)*u(4)'
   */
  rtb_VectorConcatenate_m[5] = rtb_Product2_c[1] * rtb_Product2_c[2] *
    rtb_Product3_j0[0] - rtb_Product3_j0[2] * rtb_Product2_c[0];

  /* Gain: '<S719>/Gain2' */
  rtb_VectorConcatenate_m[6] = -rtb_Product2_c[1];

  /* Product: '<S722>/u(1)*u(3)' */
  rtb_VectorConcatenate_m[7] = rtb_Product2_c[0] * rtb_Product3_j0[1];

  /* Product: '<S725>/u(4)*u(5)' */
  rtb_VectorConcatenate_m[8] = rtb_Product3_j0[0] * rtb_Product3_j0[1];

  /* Product: '<S17>/Product' */
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_VectorConcatenate[i + 3 * i_0] = 0.0F;
      rtb_VectorConcatenate[i + 3 * i_0] += rtb_VectorConcatenate_m[3 * i_0] *
        rtb_MathFunction[i];
      rtb_VectorConcatenate[i + 3 * i_0] += rtb_VectorConcatenate_m[3 * i_0 + 1]
        * rtb_MathFunction[i + 3];
      rtb_VectorConcatenate[i + 3 * i_0] += rtb_VectorConcatenate_m[3 * i_0 + 2]
        * rtb_MathFunction[i + 6];
    }
  }

  /* End of Product: '<S17>/Product' */

  /* Gain: '<S706>/Gain1' incorporates:
   *  Selector: '<S706>/Selector1'
   */
  rtb_VectorConcatenate_h[0] = rtb_VectorConcatenate[3];
  rtb_VectorConcatenate_h[1] = rtb_VectorConcatenate[0];
  rtb_VectorConcatenate_h[2] = -rtb_VectorConcatenate[6];

  /* Gain: '<S706>/Gain2' incorporates:
   *  Selector: '<S706>/Selector2'
   */
  rtb_VectorConcatenate_h[3] = rtb_VectorConcatenate[7];
  rtb_VectorConcatenate_h[4] = rtb_VectorConcatenate[8];

  /* Gain: '<S706>/Gain3' incorporates:
   *  Selector: '<S706>/Selector3'
   */
  rtb_VectorConcatenate_h[5] = -rtb_VectorConcatenate[1];
  rtb_VectorConcatenate_h[6] = rtb_VectorConcatenate[4];

  /* If: '<S698>/If' incorporates:
   *  Gain: '<S706>/Gain1'
   *  Selector: '<S706>/Selector1'
   */
  if ((-rtb_VectorConcatenate[6] >= 1.0F) || (-rtb_VectorConcatenate[6] <= -1.0F))
  {
    /* Outputs for IfAction SubSystem: '<S698>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S705>/Action Port'
     */
    SLUGS2_AxisRotZeroR3(rtb_VectorConcatenate_h, &rtb_Merge_m[0], &rtb_Merge_m
                         [1], &rtb_Merge_m[2]);

    /* End of Outputs for SubSystem: '<S698>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S698>/AxisRotDefault' incorporates:
     *  ActionPort: '<S704>/Action Port'
     */
    SLUGS2_AxisRotDefault(rtb_VectorConcatenate_h, &rtb_Merge_m[0],
                          &rtb_Merge_m[1], &rtb_Merge_m[2]);

    /* End of Outputs for SubSystem: '<S698>/AxisRotDefault' */
  }

  /* End of If: '<S698>/If' */

  /* MATLAB Function: '<S17>/Embedded MATLAB Function1' */
  S_EmbeddedMATLABFunction_l(rtb_Merge_m[0],
    &SLUGS2_B.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S17>/myMux Fun1' */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun1': '<S702>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S702>:1:5' */
  rtb_y_jl_idx_0 = rtb_Merge_m[2];
  rtb_y_jl_idx_1 = rtb_Merge_m[1];

  /* DataStoreWrite: '<S16>/mlAttitudeEuler' incorporates:
   *  DataStoreRead: '<S16>/Get time3'
   *  MATLAB Function: '<S17>/myMux Fun1'
   */
  mlAttitudeSol.roll = rtb_Merge_m[2];
  mlAttitudeSol.pitch = rtb_Merge_m[1];
  mlAttitudeSol.yaw = SLUGS2_B.sf_EmbeddedMATLABFunction1.y;
  mlAttitudeSol.time_boot_ms = SLUGS2_DWork.time_since_boot_usec;

  /* DataTypeConversion: '<S670>/Data Type Conversion1' incorporates:
   *  DataStoreRead: '<S16>/mlHeartbeatLocal'
   */
  rtb_DataTypeConversion1_hq = (uint8_T)mlHeartbeatLocal.custom_mode;
#ifndef WIN

  /* S-Function "dsPIC_PWM_IC" Block: <S644>/Input Capture */
  SLUGS2_B.InputCapture_o1 = MCHP_ic1up;
  SLUGS2_B.InputCapture_o2 = MCHP_ic2up;
  SLUGS2_B.InputCapture_o3 = MCHP_ic3up;
  SLUGS2_B.InputCapture_o4 = MCHP_ic4up;
  SLUGS2_B.InputCapture_o5 = MCHP_ic5up;
  SLUGS2_B.InputCapture_o6 = MCHP_ic6up;
  SLUGS2_B.InputCapture_o7 = MCHP_ic7up;
  SLUGS2_B.InputCapture_o8 = MCHP_ic8up;
#else
  SLUGS2_B.InputCapture_o1 = mlPilotConsoleData.chan1_raw;
  SLUGS2_B.InputCapture_o2 = mlPilotConsoleData.chan2_raw;
  SLUGS2_B.InputCapture_o3 = mlPilotConsoleData.chan3_raw;
  SLUGS2_B.InputCapture_o4 = mlPilotConsoleData.chan4_raw;
  SLUGS2_B.InputCapture_o5 = mlPilotConsoleData.chan5_raw;
  SLUGS2_B.InputCapture_o6 = mlPilotConsoleData.chan6_raw;
  SLUGS2_B.InputCapture_o7 = mlPilotConsoleData.chan7_raw;
  SLUGS2_B.InputCapture_o8 = mlPilotConsoleData.chan8_raw;
#endif
  /* MATLAB Function: '<S648>/Buffer Failsafe Channel' incorporates:
   *  DataTypeConversion: '<S645>/Data Type Conversion'
   *  Gain: '<S645>/Convert to  Microseconds'
   *  MATLAB Function: '<S644>/myMux Fun5'
   */
  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun5': '<S656>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S656>:1:5' */
  SLUGS2_BufferICChannel((uint16_T)(52429UL * SLUGS2_B.InputCapture_o5 >> 18),
    &SLUGS2_B.sf_BufferFailsafeChannel, &SLUGS2_DWork.sf_BufferFailsafeChannel);

  /* S-Function (MCHP_C_function_Call): '<S648>/Choose the Median [navSupport.c] [updated 5.1.16]' */
  SLUGS2_B.ChoosetheMediannavSupportcupdat = meanFilter5(
    &SLUGS2_B.sf_BufferFailsafeChannel.history[0]
    );

  /* S-Function (MCHP_C_function_Call): '<S15>/Manual or Auto? [navSupport.c] [updated 4.28.16]' */
  SLUGS2_B.ManualorAutonavSupportcupdated4 = isApManual(
    SLUGS2_B.ChoosetheMediannavSupportcupdat
    );

  /* MATLAB Function: '<S643>/Buffer IC Channel' incorporates:
   *  MATLAB Function: '<S644>/myMux Fun5'
   */
  SLUGS2_BufferICChannel(SLUGS2_B.InputCapture_o3, &SLUGS2_B.sf_BufferICChannel,
    &SLUGS2_DWork.sf_BufferICChannel);

  /* MATLAB Function: '<S643>/Buffer IC Channel1' incorporates:
   *  MATLAB Function: '<S644>/myMux Fun5'
   */
  SLUGS2_BufferICChannel(SLUGS2_B.InputCapture_o1, &SLUGS2_B.sf_BufferICChannel1,
    &SLUGS2_DWork.sf_BufferICChannel1);

  /* MATLAB Function: '<S643>/Buffer IC Channel2' incorporates:
   *  MATLAB Function: '<S644>/myMux Fun5'
   */
  SLUGS2_BufferICChannel(SLUGS2_B.InputCapture_o4, &SLUGS2_B.sf_BufferICChannel2,
    &SLUGS2_DWork.sf_BufferICChannel2);

  /* MATLAB Function: '<S643>/Buffer IC Channel3' incorporates:
   *  MATLAB Function: '<S644>/myMux Fun5'
   */
  SLUGS2_BufferICChannel(SLUGS2_B.InputCapture_o2, &SLUGS2_B.sf_BufferICChannel3,
    &SLUGS2_DWork.sf_BufferICChannel3);

  /* Logic: '<S646>/Logical Operator' */
  rtb_IC1_a = !(SLUGS2_B.ManualorAutonavSupportcupdated4 != 0);

  /* InitialCondition: '<S646>/IC1' */
  if (SLUGS2_DWork.IC1_FirstOutputTime) {
    SLUGS2_DWork.IC1_FirstOutputTime = false;
    rtb_IC1_a = false;
  }

  /* End of InitialCondition: '<S646>/IC1' */

  /* DataTypeConversion: '<S659>/Data Type Conversion1' */
  SLUGS2_B.DataTypeConversion1_h = rtb_IC1_a;

  /* S-Function (MCHP_C_function_Call): '<S659>/C Function Call' */
  SLUGS2_B.CFunctionCall = justEnabled(
    SLUGS2_B.DataTypeConversion1_h
    , ((uint8_T)2U)
    );

  /* Outputs for Enabled SubSystem: '<S15>/Grab I.C.' incorporates:
   *  EnablePort: '<S647>/Enable'
   */
  if (SLUGS2_B.CFunctionCall > 0) {
    /* S-Function (MCHP_C_function_Call): '<S647>/Choose the Median navSupport.c [updated 4.28.16]' */
    SLUGS2_B.ChoosetheMediannavSupportcupd_a = meanFilter5(
      &SLUGS2_B.sf_BufferICChannel.history[0]
      );

    /* Saturate: '<S647>/[0.55 0.68]' */
    if (SLUGS2_B.ChoosetheMediannavSupportcupd_a > 14163U) {
      rtb_u2deg = 14163U;
    } else if (SLUGS2_B.ChoosetheMediannavSupportcupd_a < 13219U) {
      rtb_u2deg = 13219U;
    } else {
      rtb_u2deg = SLUGS2_B.ChoosetheMediannavSupportcupd_a;
    }

    /* End of Saturate: '<S647>/[0.55 0.68]' */

    /* DataTypeConversion: '<S660>/Data Type Conversion' incorporates:
     *  Gain: '<S660>/Convert to  Microseconds'
     */
    SLUGS2_B.DataTypeConversion_hn = (uint16_T)(52429UL * rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S647>/update dT Trim updateControlMcu.c [updated 4.28.16]' */
    updatePWMTrim(
                  SLUGS2_B.DataTypeConversion_hn
                  , ((uint8_T)0U)
                  );

    /* DataTypeConversion: '<S664>/Data Type Conversion' incorporates:
     *  Constant: '<S664>/Constant1'
     *  Constant: '<S664>/Constant2'
     *  Product: '<S664>/Divide'
     *  Sum: '<S664>/Add'
     */
    SLUGS2_B.DataTypeConversion = (real32_T)((real_T)rtb_u2deg *
      0.00013768415255404102 + -1.2699986231584739);

    /* S-Function (MCHP_C_function_Call): '<S647>/2' */
    SLUGS2_B.u = meanFilter5(
      &SLUGS2_B.sf_BufferICChannel1.history[0]
      );

    /* Saturate: '<S647>/[-2  2] deg' */
    if (SLUGS2_B.u > 13338U) {
      rtb_u2deg = 13338U;
    } else if (SLUGS2_B.u < 12386U) {
      rtb_u2deg = 12386U;
    } else {
      rtb_u2deg = SLUGS2_B.u;
    }

    /* End of Saturate: '<S647>/[-2  2] deg' */

    /* DataTypeConversion: '<S661>/Data Type Conversion' incorporates:
     *  Gain: '<S661>/Convert to  Microseconds'
     */
    SLUGS2_B.DataTypeConversion_e = (uint16_T)(52429UL * rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S647>/update dA Trim updateControlMcu.c' */
    updatePWMTrim(
                  SLUGS2_B.DataTypeConversion_e
                  , ((uint8_T)1U)
                  );

    /* DataTypeConversion: '<S665>/Data Type Conversion' incorporates:
     *  Constant: '<S665>/Constant1'
     *  Constant: '<S665>/Constant2'
     *  Product: '<S665>/Divide'
     *  Sum: '<S665>/Add'
     */
    SLUGS2_B.DataTypeConversion_h = (real32_T)((real_T)rtb_u2deg *
      -7.3291671030313021E-5 + 0.94267747279188585);

    /* S-Function (MCHP_C_function_Call): '<S647>/3' */
    SLUGS2_B.u_h = meanFilter5(
      &SLUGS2_B.sf_BufferICChannel2.history[0]
      );

    /* Saturate: '<S647>/[-2  2] deg ' */
    if (SLUGS2_B.u_h > 13227U) {
      rtb_u2deg = 13227U;
    } else if (SLUGS2_B.u_h < 12514U) {
      rtb_u2deg = 12514U;
    } else {
      rtb_u2deg = SLUGS2_B.u_h;
    }

    /* End of Saturate: '<S647>/[-2  2] deg ' */

    /* DataTypeConversion: '<S662>/Data Type Conversion' incorporates:
     *  Gain: '<S662>/Convert to  Microseconds'
     */
    SLUGS2_B.DataTypeConversion_go = (uint16_T)(52429UL * rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S647>/update dR Trim updateControlMcu.c' */
    updatePWMTrim(
                  SLUGS2_B.DataTypeConversion_go
                  , ((uint8_T)2U)
                  );

    /* DataTypeConversion: '<S666>/Data Type Conversion' incorporates:
     *  Constant: '<S666>/Constant1'
     *  Constant: '<S666>/Constant2'
     *  Product: '<S666>/Divide'
     *  Sum: '<S666>/Add'
     */
    SLUGS2_B.DataTypeConversion_g = (real32_T)((real_T)rtb_u2deg *
      -9.7941229953457077E-5 + 1.2605362765776436);

    /* S-Function (MCHP_C_function_Call): '<S647>/4' */
    SLUGS2_B.u_m = meanFilter5(
      &SLUGS2_B.sf_BufferICChannel3.history[0]
      );

    /* Saturate: '<S647>/[-2  2] deg  ' */
    if (SLUGS2_B.u_m > 13548U) {
      rtb_u2deg = 13548U;
    } else if (SLUGS2_B.u_m < 12135U) {
      rtb_u2deg = 12135U;
    } else {
      rtb_u2deg = SLUGS2_B.u_m;
    }

    /* End of Saturate: '<S647>/[-2  2] deg  ' */

    /* DataTypeConversion: '<S663>/Data Type Conversion' incorporates:
     *  Gain: '<S663>/Convert to  Microseconds'
     */
    SLUGS2_B.DataTypeConversion_k = (uint16_T)(52429UL * rtb_u2deg >> 18);

    /* S-Function (MCHP_C_function_Call): '<S647>/update dE Trim updateControlMcu.c' */
    updatePWMTrim(
                  SLUGS2_B.DataTypeConversion_k
                  , ((uint8_T)3U)
                  );

    /* DataTypeConversion: '<S667>/Data Type Conversion' incorporates:
     *  Constant: '<S667>/Constant1'
     *  Constant: '<S667>/Constant2'
     *  Product: '<S667>/Divide'
     *  Sum: '<S667>/Add'
     */
    SLUGS2_B.DataTypeConversion_f = (real32_T)((real_T)rtb_u2deg *
      4.9407763350001607E-5 + -0.63447802768627071);
  }

  /* End of Outputs for SubSystem: '<S15>/Grab I.C.' */

  /* Logic: '<S9>/Logical Operator' incorporates:
   *  Constant: '<S462>/Constant'
   *  Constant: '<S463>/Constant'
   *  Constant: '<S464>/Constant'
   *  DataStoreRead: '<Root>/isPassthrough'
   *  RelationalOperator: '<S462>/Compare'
   *  RelationalOperator: '<S463>/Compare'
   *  RelationalOperator: '<S464>/Compare'
   */
  rtb_IC1_a = (boolean_T)((mlHeartbeatLocal.custom_mode == 2UL) ^
    (mlHeartbeatLocal.custom_mode == 4UL)) ^ (mlHeartbeatLocal.custom_mode ==
    8UL);

  /* S-Function (MCHP_C_function_Call): '<Root>/Get RTB Order [navSupport.c] [updated 4.27.16]' */
  getRTB(
         &SLUGS2_B.GetRTBOrdernavSupportcupdated42[0]
         );

  /* Outputs for Atomic SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */
  /* DataTypeConversion: '<S65>/Data Type Conversion' incorporates:
   *  DataStoreRead: '<S65>/Get Nav Mode'
   */
  rtb_Compare_j = (uint8_T)mlHeartbeatLocal.custom_mode;

  /* InitialCondition: '<S182>/IC1' incorporates:
   *  DataStoreRead: '<Root>/isWpFly?'
   *  Logic: '<S173>/Logical Operator'
   *  Logic: '<S173>/Logical Operator1'
   */
  if (SLUGS2_DWork.IC1_FirstOutputTime_m) {
    SLUGS2_DWork.IC1_FirstOutputTime_m = false;
    rtb_IC1 = false;
  } else {
    rtb_IC1 = ((mlHeartbeatLocal.custom_mode != 0UL) &&
               (!(SLUGS2_B.ManualorAutonavSupportcupdated4 != 0)));
  }

  /* End of InitialCondition: '<S182>/IC1' */

  /* DataTypeConversion: '<S452>/Data Type Conversion2' incorporates:
   *  Delay: '<S457>/Delay'
   *  Logic: '<S457>/Logical Operator'
   *  RelationalOperator: '<S457>/Relational Operator'
   */
  rtb_DataTypeConversion2_fi = (uint8_T)((SLUGS2_DWork.Delay_DSTATE_e != rtb_IC1)
    && rtb_IC1);

  /* InitialCondition: '<S182>/IC2' */
  if (SLUGS2_DWork.IC2_FirstOutputTime) {
    SLUGS2_DWork.IC2_FirstOutputTime = false;
    rtb_IC2 = 0U;
  } else {
    rtb_IC2 = rtb_DataTypeConversion2_fi;
  }

  /* End of InitialCondition: '<S182>/IC2' */

  /* Outputs for Enabled SubSystem: '<S182>/Grab Upon Enable' incorporates:
   *  EnablePort: '<S451>/Enable'
   */
  if (rtb_IC2 > 0) {
    /* SignalConversion: '<S454>/Numerical Unity' incorporates:
     *  DataStoreRead: '<S451>/Get GS Location'
     *  Gain: '<S451>/Gain'
     *  Gain: '<S451>/Gain1'
     *  Gain: '<S451>/Gain2'
     */
    SLUGS2_B.NumericalUnity[0] = 0.001F * mlGSLocationFloat.alt;
    SLUGS2_B.NumericalUnity[1] = 1.0E-7F * mlGSLocationFloat.lat;
    SLUGS2_B.NumericalUnity[2] = 1.0E-7F * mlGSLocationFloat.lon;

    /* Gain: '<S455>/Deg2R' */
    rtb_cosphi = 0.0174532924F * SLUGS2_B.NumericalUnity[1];

    /* Trigonometry: '<S455>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S455>/Sum1' incorporates:
     *  Constant: '<S455>/const'
     *  Product: '<S455>/Product1'
     *  Product: '<S455>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S455>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S455>/f' */

    /* Product: '<S455>/Rh' incorporates:
     *  Constant: '<S455>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S455>/Sum2' */
    rtb_RhhcosphisinlambYe = SLUGS2_B.NumericalUnity[0] + rtb_Sum1_mj;

    /* Trigonometry: '<S455>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S455>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F * SLUGS2_B.NumericalUnity[2];

    /* Product: '<S455>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S455>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S455>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S455>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S455>/Ze' incorporates:
     *  Product: '<S455>/Rh(1-e^2)'
     *  Sum: '<S455>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj + SLUGS2_B.NumericalUnity[0];

    /* DataTypeConversion: '<S451>/Data Type Conversion' */
    SLUGS2_B.DataTypeConversion_c[0] = SLUGS2_B.NumericalUnity[2];
    SLUGS2_B.DataTypeConversion_c[1] = SLUGS2_B.NumericalUnity[1];

    /* DataTypeConversion: '<S451>/Data Type Conversion1' */
    SLUGS2_B.DataTypeConversion1_l[0] = rtb_RhhcosphicoslambXe;
    SLUGS2_B.DataTypeConversion1_l[1] = rtb_RhhcosphisinlambYe;
    SLUGS2_B.DataTypeConversion1_l[2] = rtb_Ze_b;
  }

  /* End of Outputs for SubSystem: '<S182>/Grab Upon Enable' */

  /* MATLAB Function: '<S173>/myMux Fun2' */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun2': '<S186>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S186>:1:5' */
  SLUGS2_B.y_p[0] = SLUGS2_B.DataTypeConversion1_l[0];
  SLUGS2_B.y_p[1] = SLUGS2_B.DataTypeConversion1_l[1];
  SLUGS2_B.y_p[2] = SLUGS2_B.DataTypeConversion1_l[2];

  /* S-Function (MCHP_C_function_Call): '<S173>/Diagnoistics Set navsupport.c [updated 5.1.16]1' */
  setDiagnosticFloat(
                     &SLUGS2_B.y_p[0]
                     );

  /* InitialCondition: '<S182>/IC4' */
  if (SLUGS2_DWork.IC4_FirstOutputTime) {
    SLUGS2_DWork.IC4_FirstOutputTime = false;
    rtb_IC4_idx_0 = -122.050926F;
    rtb_IC4_idx_1 = 36.9885025F;
  } else {
    rtb_IC4_idx_0 = SLUGS2_B.DataTypeConversion_c[0];
    rtb_IC4_idx_1 = SLUGS2_B.DataTypeConversion_c[1];
  }

  /* End of InitialCondition: '<S182>/IC4' */

  /* MATLAB Function: '<S173>/Zero out Z2' */
  SLUGS2_ZerooutZ1(SLUGS2_B.sf_myMuxFun1_e.y, &SLUGS2_B.sf_ZerooutZ2);

  /* MATLAB Function: '<S173>/Zero out Z1' */
  SLUGS2_ZerooutZ1(SLUGS2_B.sf_myMuxFun2_a.y, &SLUGS2_B.sf_ZerooutZ1);

  /* MATLAB Function: '<S256>/Embedded MATLAB Function' */
  S_EmbeddedMATLABFunction_b(SLUGS2_B.sf_ZerooutZ1.P,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_i1);

  /* MATLAB Function: '<S257>/negprotect' */
  SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_i1.xDoty,
                      &SLUGS2_B.sf_negprotect_lq);

  /* S-Function (MCHP_C_function_Call): '<S257>/mySqrt() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.mySqrtapUtilscupdated5116 = mySqrt(
    SLUGS2_B.sf_negprotect_lq.zpVal
    );

  /* InitialCondition: '<S179>/IC' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   */
  if (SLUGS2_DWork.IC_FirstOutputTime) {
    SLUGS2_DWork.IC_FirstOutputTime = false;
    rtb_RhhcosphisinlambYe = 1.5F;
  } else {
    rtb_RhhcosphisinlambYe = mlParamInterface.param[26];
  }

  /* End of InitialCondition: '<S179>/IC' */

  /* Saturate: '<S179>/[1.5 10]' */
  if (rtb_RhhcosphisinlambYe > 10.0F) {
    rtb_RhhcosphisinlambYe = 10.0F;
  } else {
    if (rtb_RhhcosphisinlambYe < 1.5F) {
      rtb_RhhcosphisinlambYe = 1.5F;
    }
  }

  /* Product: '<S179>/Product3' incorporates:
   *  Constant: '<S179>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S179>/Product1'
   *  Product: '<S179>/Product2'
   *  Saturate: '<S179>/[1.5 10]'
   */
  rtb_Product3_bo = mlMidLevelCommands.uCommand * mlMidLevelCommands.uCommand *
    rtb_RhhcosphisinlambYe / 4.57681F;

  /* If: '<S173>/Determine Overall Nav by the Nav Mode' incorporates:
   *  Constant: '<S174>/RTB1'
   *  Constant: '<S178>/RTB'
   *  Constant: '<S178>/RTB1'
   *  Constant: '<S181>/RTB'
   *  Constant: '<S181>/RTB1'
   *  Constant: '<S374>/Constant'
   *  Constant: '<S374>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Gain: '<S199>/Deg2R'
   *  Gain: '<S440>/Deg2R'
   *  Inport: '<S178>/MidLvl h_c'
   *  Inport: '<S181>/MidLvl h_c'
   *  Product: '<S323>/Divide'
   *  Product: '<S378>/Divide'
   *  Sum: '<S317>/Add'
   */
  if (SLUGS2_B.GetRTBOrdernavSupportcupdated42[0] == 1) {
    /* Outputs for IfAction SubSystem: '<S173>/RTB//Follow Mobile Navigation' incorporates:
     *  ActionPort: '<S181>/Action Port'
     */
    /* Outputs for Enabled SubSystem: '<S181>/Compute Mobile Location' incorporates:
     *  EnablePort: '<S435>/Enable'
     */
    if (SLUGS2_B.GetRTBOrdernavSupportcupdated42[1] > 0) {
      rtb_IC4_idx_0 *= 0.0174532924F;

      /* Gain: '<S440>/Deg2R' */
      rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_1;

      /* Gain: '<S441>/Deg2R' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_RhhcosphicoslambXe = 0.0174532924F * mlMobileLocation.latitude;

      /* Trigonometry: '<S441>/sin(phi)' */
      rtb_cosphi = (real32_T)sin(rtb_RhhcosphicoslambXe);

      /* Sum: '<S441>/Sum1' incorporates:
       *  Constant: '<S441>/const'
       *  Product: '<S441>/Product1'
       *  Product: '<S441>/sin(phi)^2'
       */
      rtb_Sum1_mj = 1.0F - rtb_cosphi * rtb_cosphi * 0.00669425726F;

      /* Fcn: '<S441>/f' */
      if (rtb_Sum1_mj < 0.0F) {
        rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
      } else {
        rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
      }

      /* End of Fcn: '<S441>/f' */

      /* Product: '<S441>/Rh' incorporates:
       *  Constant: '<S441>/Re=equatorial radius'
       */
      rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

      /* Trigonometry: '<S441>/cos(phi)' */
      rtb_RhhcosphicoslambXe = (real32_T)cos(rtb_RhhcosphicoslambXe);

      /* Gain: '<S441>/Deg2R1' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_RhhcosphisinlambYe = 0.0174532924F * mlMobileLocation.longitude;

      /* Product: '<S441>/Ze' incorporates:
       *  Product: '<S441>/Rh(1-e^2)'
       */
      rtb_cosphi *= 0.993305743F * rtb_Sum1_mj;

      /* SignalConversion: '<S440>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S443>/11'
       *  Fcn: '<S443>/12'
       *  Fcn: '<S443>/13'
       *  Fcn: '<S443>/21'
       *  Fcn: '<S443>/22'
       *  Fcn: '<S443>/23'
       *  Fcn: '<S443>/31'
       *  Fcn: '<S443>/32'
       *  Fcn: '<S443>/33'
       */
      tmp[0] = (real32_T)cos(rtb_IC4_idx_0) * (real32_T)cos(rtb_Deg2R_h_idx_0);
      tmp[1] = -(real32_T)sin(rtb_IC4_idx_0);
      tmp[2] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_IC4_idx_0);
      tmp[3] = (real32_T)sin(rtb_IC4_idx_0) * (real32_T)cos(rtb_Deg2R_h_idx_0);
      tmp[4] = (real32_T)cos(rtb_IC4_idx_0);
      tmp[5] = -(real32_T)sin(rtb_IC4_idx_0) * (real32_T)sin(rtb_Deg2R_h_idx_0);
      tmp[6] = (real32_T)sin(rtb_Deg2R_h_idx_0);
      tmp[7] = 0.0F;
      tmp[8] = (real32_T)cos(rtb_Deg2R_h_idx_0);

      /* Sum: '<S439>/Sum1' incorporates:
       *  Product: '<S440>/Product1'
       *  Product: '<S441>/(Rh+h)cos(phi)*cos(lamb)=Xe'
       *  Product: '<S441>/(Rh+h)cos(phi)*sin(lamb)=Ye'
       *  Sum: '<S441>/Sum2'
       *  Trigonometry: '<S441>/cos(lamb)'
       *  Trigonometry: '<S441>/sin(lamb)'
       */
      rtb_Deg2R1 = rtb_Sum1_mj * rtb_RhhcosphicoslambXe * (real32_T)cos
        (rtb_RhhcosphisinlambYe) - SLUGS2_B.DataTypeConversion1_l[0];
      rtb_RhhcosphisinlambYe = rtb_Sum1_mj * rtb_RhhcosphicoslambXe * (real32_T)
        sin(rtb_RhhcosphisinlambYe) - SLUGS2_B.DataTypeConversion1_l[1];
      rtb_Sum1_mj = rtb_cosphi - SLUGS2_B.DataTypeConversion1_l[2];

      /* Product: '<S440>/Product1' incorporates:
       *  Gain: '<S439>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp_0[i] = tmp[i + 6] * rtb_Sum1_mj + (tmp[i + 3] *
          rtb_RhhcosphisinlambYe + tmp[i] * rtb_Deg2R1);
      }

      /* Reshape: '<S439>/Reshape1' incorporates:
       *  Gain: '<S439>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        SLUGS2_B.Reshape1[i] = 0.0F;
        SLUGS2_B.Reshape1[i] += SLUGS2_ConstP.pooled65[i] * tmp_0[0];
        SLUGS2_B.Reshape1[i] += SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1];
        SLUGS2_B.Reshape1[i] += SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2];
      }

      /* End of Reshape: '<S439>/Reshape1' */
    }

    /* End of Outputs for SubSystem: '<S181>/Compute Mobile Location' */

    /* MATLAB Function: '<S181>/Zero out Z1' incorporates:
     *  Gain: '<S440>/Deg2R'
     */
    SLUGS2_ZerooutZ1(SLUGS2_B.Reshape1, &SLUGS2_B.sf_ZerooutZ1_a);

    /* Sum: '<S181>/Subtract' */
    rtb_Product3_j0[0] = SLUGS2_B.sf_ZerooutZ1_a.P[0] - SLUGS2_B.sf_ZerooutZ2.P
      [0];
    rtb_Product3_j0[1] = SLUGS2_B.sf_ZerooutZ1_a.P[1] - SLUGS2_B.sf_ZerooutZ2.P
      [1];
    rtb_Product3_j0[2] = SLUGS2_B.sf_ZerooutZ1_a.P[2] - SLUGS2_B.sf_ZerooutZ2.P
      [2];

    /* DiscreteIntegrator: '<S436>/Discrete-Time Integrator' */
    if (SLUGS2_DWork.DiscreteTimeIntegrator_IC_LOADI != 0) {
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[0] = rtb_Product3_j0[0];
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[1] = rtb_Product3_j0[1];
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[2] = rtb_Product3_j0[2];
    }

    /* Sum: '<S436>/Sum' incorporates:
     *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator'
     */
    rtb_Product2_c[0] = rtb_Product3_j0[0] -
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[0];
    rtb_Product2_c[1] = rtb_Product3_j0[1] -
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[1];
    rtb_Product2_c[2] = rtb_Product3_j0[2] -
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* Sum: '<S436>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S436>/Discrete-Time Integrator'
     *  Gain: '<S436>/Gain1'
     */
    SLUGS2_B.Merge_o[0] = 3.0F * rtb_Product2_c[0] +
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[0];
    SLUGS2_B.Merge_o[1] = 3.0F * rtb_Product2_c[1] +
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[1];
    SLUGS2_B.Merge_o[2] = 3.0F * rtb_Product2_c[2] +
      SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* MATLAB Function: '<S445>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_bs);

    /* MATLAB Function: '<S446>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_bs.xDoty,
                        &SLUGS2_B.sf_negprotect_la);

    /* S-Function (MCHP_C_function_Call): '<S446>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_g = mySqrt(
      SLUGS2_B.sf_negprotect_la.zpVal
      );

    /* SignalConversion: '<S449>/Numerical Unity' */
    SLUGS2_B.Merge2 = SLUGS2_B.mySqrtapUtilscupdated5116_g;
    SLUGS2_B.Merge1 = mlMidLevelCommands.hCommand;
    SLUGS2_B.Merge3 = 0U;
    SLUGS2_B.Merge4 = 0U;

    /* Update for DiscreteIntegrator: '<S436>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S181>/RTB'
     *  Constant: '<S181>/RTB1'
     *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
     *  Inport: '<S181>/MidLvl h_c'
     */
    SLUGS2_DWork.DiscreteTimeIntegrator_IC_LOADI = 0U;
    SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[0] += 0.01F * rtb_Product2_c[0];
    SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[1] += 0.01F * rtb_Product2_c[1];
    SLUGS2_DWork.DiscreteTimeIntegrator_DSTATE[2] += 0.01F * rtb_Product2_c[2];

    /* End of Outputs for SubSystem: '<S173>/RTB//Follow Mobile Navigation' */
  } else if (rtb_Compare_j == 3) {
    /* Outputs for IfAction SubSystem: '<S173>/Normal WP  Navigation' incorporates:
     *  ActionPort: '<S180>/Action Port'
     */
    /* Product: '<S180>/Product' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    rtb_Product = mlParamInterface.param[19] *
      SLUGS2_B.mySqrtapUtilscupdated5116;

    /* Outputs for Enabled SubSystem: '<S180>/On WP Enable' incorporates:
     *  EnablePort: '<S297>/Enable'
     */
    /* Gain: '<S398>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S376>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)1U)
          , &SLUGS2_B.GetWPCoordnavsupportcupdated511[0]
          );

    /* Gain: '<S399>/Deg2R' incorporates:
     *  Constant: '<S374>/Constant'
     */
    rtb_cosphi = 0.0174532924F * SLUGS2_B.GetWPCoordnavsupportcupdated511[0];

    /* Trigonometry: '<S399>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S399>/Sum1' incorporates:
     *  Constant: '<S399>/const'
     *  Product: '<S399>/Product1'
     *  Product: '<S399>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S399>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S399>/f' */

    /* Product: '<S399>/Rh' incorporates:
     *  Constant: '<S399>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S399>/Sum2' */
    rtb_RhhcosphicoslambXe = SLUGS2_B.GetWPCoordnavsupportcupdated511[2] +
      rtb_Sum1_mj;

    /* Trigonometry: '<S399>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S399>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      SLUGS2_B.GetWPCoordnavsupportcupdated511[1];

    /* Product: '<S399>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S399>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S399>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S399>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S399>/Ze' incorporates:
     *  Product: '<S399>/Rh(1-e^2)'
     *  Sum: '<S399>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      SLUGS2_B.GetWPCoordnavsupportcupdated511[2];

    /* SignalConversion: '<S398>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S401>/11'
     *  Fcn: '<S401>/12'
     *  Fcn: '<S401>/13'
     *  Fcn: '<S401>/21'
     *  Fcn: '<S401>/22'
     *  Fcn: '<S401>/23'
     *  Fcn: '<S401>/31'
     *  Fcn: '<S401>/32'
     *  Fcn: '<S401>/33'
     */
    tmp[0] = (real32_T)cos(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp[1] = -(real32_T)sin(rtb_Deg2R_h_idx_0);
    tmp[2] = -(real32_T)sin(rtb_Product3_bo) * (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp[3] = (real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp[4] = (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp[5] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)sin(rtb_Product3_bo);
    tmp[6] = (real32_T)sin(rtb_Product3_bo);
    tmp[7] = 0.0F;
    tmp[8] = (real32_T)cos(rtb_Product3_bo);

    /* Sum: '<S396>/Sum1' incorporates:
     *  Product: '<S398>/Product1'
     */
    rtb_Deg2R1 -= SLUGS2_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      SLUGS2_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - SLUGS2_B.DataTypeConversion1_l[2];

    /* Product: '<S398>/Product1' incorporates:
     *  Gain: '<S396>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp[i + 6] * rtb_Sum1_mj + (tmp[i + 3] * rtb_RhhcosphisinlambYe
        + tmp[i] * rtb_Deg2R1);
    }

    /* Gain: '<S396>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      SLUGS2_B.WP0L2IPT1[i] = 0.0F;
      SLUGS2_B.WP0L2IPT1[i] += SLUGS2_ConstP.pooled65[i] * tmp_0[0];
      SLUGS2_B.WP0L2IPT1[i] += SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1];
      SLUGS2_B.WP0L2IPT1[i] += SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2];
    }

    /* Gain: '<S415>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S377>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)2U)
          , &SLUGS2_B.GetWPCoordnavsupportcupdated5_o[0]
          );

    /* Gain: '<S416>/Deg2R' incorporates:
     *  Constant: '<S374>/Constant1'
     */
    rtb_cosphi = 0.0174532924F * SLUGS2_B.GetWPCoordnavsupportcupdated5_o[0];

    /* Trigonometry: '<S416>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S416>/Sum1' incorporates:
     *  Constant: '<S416>/const'
     *  Product: '<S416>/Product1'
     *  Product: '<S416>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S416>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S416>/f' */

    /* Product: '<S416>/Rh' incorporates:
     *  Constant: '<S416>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S416>/Sum2' */
    rtb_RhhcosphicoslambXe = SLUGS2_B.GetWPCoordnavsupportcupdated5_o[2] +
      rtb_Sum1_mj;

    /* Trigonometry: '<S416>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S416>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      SLUGS2_B.GetWPCoordnavsupportcupdated5_o[1];

    /* Product: '<S416>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S416>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S416>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S416>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S416>/Ze' incorporates:
     *  Product: '<S416>/Rh(1-e^2)'
     *  Sum: '<S416>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      SLUGS2_B.GetWPCoordnavsupportcupdated5_o[2];

    /* SignalConversion: '<S415>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S418>/11'
     *  Fcn: '<S418>/12'
     *  Fcn: '<S418>/13'
     *  Fcn: '<S418>/21'
     *  Fcn: '<S418>/22'
     *  Fcn: '<S418>/23'
     *  Fcn: '<S418>/31'
     *  Fcn: '<S418>/32'
     *  Fcn: '<S418>/33'
     */
    tmp_1[0] = (real32_T)cos(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_1[1] = -(real32_T)sin(rtb_Deg2R_h_idx_0);
    tmp_1[2] = -(real32_T)sin(rtb_Product3_bo) * (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_1[3] = (real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_1[4] = (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_1[5] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)sin(rtb_Product3_bo);
    tmp_1[6] = (real32_T)sin(rtb_Product3_bo);
    tmp_1[7] = 0.0F;
    tmp_1[8] = (real32_T)cos(rtb_Product3_bo);

    /* Sum: '<S413>/Sum1' incorporates:
     *  Product: '<S415>/Product1'
     */
    rtb_Deg2R1 -= SLUGS2_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      SLUGS2_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - SLUGS2_B.DataTypeConversion1_l[2];

    /* Product: '<S415>/Product1' incorporates:
     *  Gain: '<S413>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_1[i + 6] * rtb_Sum1_mj + (tmp_1[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_1[i] * rtb_Deg2R1);
    }

    /* Gain: '<S413>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_P32[i] = SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2] +
        (SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1] + SLUGS2_ConstP.pooled65[i] *
         tmp_0[0]);
    }

    /* Sum: '<S375>/Add' incorporates:
     *  Gain: '<S413>/UEN 2 NEU'
     */
    rtb_Product3_j0[0] = rtb_P32[0] - SLUGS2_B.WP0L2IPT1[0];
    rtb_Product3_j0[1] = rtb_P32[1] - SLUGS2_B.WP0L2IPT1[1];
    rtb_Product3_j0[2] = rtb_P32[2] - SLUGS2_B.WP0L2IPT1[2];

    /* MATLAB Function: '<S382>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_e);

    /* MATLAB Function: '<S383>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_e.xDoty,
                        &SLUGS2_B.sf_negprotect_nb);

    /* S-Function (MCHP_C_function_Call): '<S383>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_a5 = mySqrt(
      SLUGS2_B.sf_negprotect_nb.zpVal
      );

    /* Saturate: '<S378>/Zero Bound' */
    if (SLUGS2_B.mySqrtapUtilscupdated5116_a5 <= 0.001F) {
      rtb_Sum1_mj = 0.001F;
    } else {
      rtb_Sum1_mj = SLUGS2_B.mySqrtapUtilscupdated5116_a5;
    }

    /* End of Saturate: '<S378>/Zero Bound' */
    rtb_Product3_j0[0] /= rtb_Sum1_mj;
    rtb_Product3_j0[1] /= rtb_Sum1_mj;

    /* Product: '<S378>/Divide' */
    rtb_Sum1_mj = rtb_Product3_j0[2] / rtb_Sum1_mj;
    rtb_Product3_j0[2] = rtb_Sum1_mj;

    /* MATLAB Function: '<S375>/Select N  Terms' incorporates:
     *  Product: '<S378>/Divide'
     */
    SLUGS2_SelectNTerms(rtb_Product3_j0, &SLUGS2_B.sf_SelectNTerms_f);

    /* MATLAB Function: '<S389>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(SLUGS2_B.sf_SelectNTerms_f.N,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_l);

    /* MATLAB Function: '<S390>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_l.xDoty,
                        &SLUGS2_B.sf_negprotect_j);

    /* S-Function (MCHP_C_function_Call): '<S390>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_i = mySqrt(
      SLUGS2_B.sf_negprotect_j.zpVal
      );

    /* Sum: '<S297>/Subtract' incorporates:
     *  Constant: '<S297>/Constant5'
     *  Product: '<S297>/Product'
     */
    SLUGS2_B.WP0L2IPT1[0] -= rtb_Product * rtb_Product3_j0[0] * 2.0F;
    SLUGS2_B.WP0L2IPT1[1] -= rtb_Product * rtb_Product3_j0[1] * 2.0F;
    SLUGS2_B.WP0L2IPT1[2] -= rtb_Product * rtb_Sum1_mj * 2.0F;

    /* End of Outputs for SubSystem: '<S180>/On WP Enable' */

    /* MATLAB Function: '<S180>/Zero out Z2' */
    SLUGS2_ZerooutZ1(SLUGS2_B.WP0L2IPT1, &SLUGS2_B.sf_ZerooutZ2_j);

    /* Sum: '<S180>/Add' */
    SLUGS2_B.Merge_o[0] = SLUGS2_B.sf_ZerooutZ2_j.P[0] -
      SLUGS2_B.sf_ZerooutZ2.P[0];
    SLUGS2_B.Merge_o[1] = SLUGS2_B.sf_ZerooutZ2_j.P[1] -
      SLUGS2_B.sf_ZerooutZ2.P[1];
    SLUGS2_B.Merge_o[2] = SLUGS2_B.sf_ZerooutZ2_j.P[2] -
      SLUGS2_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S429>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(SLUGS2_B.Merge_o,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_d);

    /* MATLAB Function: '<S430>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_d.xDoty,
                        &SLUGS2_B.sf_negprotect_lj);

    /* S-Function (MCHP_C_function_Call): '<S430>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_b = mySqrt(
      SLUGS2_B.sf_negprotect_lj.zpVal
      );

    /* MATLAB Function: '<S180>/Embedded MATLAB Function' incorporates:
     *  Gain: '<S180>/Gain'
     *  RelationalOperator: '<S180>/Relational Operator'
     */
    /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Embedded MATLAB Function': '<S295>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    /*  this is just turned on for one sample  */
    /*  inmediatley upon enabling the navigation. */
    /*  Reset the flag the the IP was reached */
    if (rtb_DataTypeConversion2_fi != 0) {
      /* '<S295>:1:19' */
      SLUGS2_DWork.persistentDidReachIP = 0U;
    }

    /*  Once the IP is reached the the persistent variable */
    /*  preserves the values until reset */
    if (0.4F * rtb_Product > SLUGS2_B.mySqrtapUtilscupdated5116_b) {
      /* '<S295>:1:24' */
      /* '<S295>:1:25' */
      SLUGS2_DWork.persistentDidReachIP = 1U;
    }

    /* InitialCondition: '<S180>/IC' incorporates:
     *  MATLAB Function: '<S180>/Embedded MATLAB Function'
     */
    /* '<S295>:1:28' */
    if (SLUGS2_DWork.IC_FirstOutputTime_m) {
      SLUGS2_DWork.IC_FirstOutputTime_m = false;
      SLUGS2_B.IC = 0U;
    } else {
      SLUGS2_B.IC = SLUGS2_DWork.persistentDidReachIP;
    }

    /* End of InitialCondition: '<S180>/IC' */

    /* MATLAB Function: '<S180>/computeCurrentWP' incorporates:
     *  DataStoreRead: '<Root>/Get Max WP'
     *  Delay: '<S180>/Integer Delay'
     */
    /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/computeCurrentWP': '<S300>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    if (SLUGS2_DWork.IntegerDelay_DSTATE_im != 0) {
      /* '<S300>:1:16' */
      SLUGS2_DWork.fromWp = SLUGS2_DWork.toWp;

      /* '<S300>:1:17' */
      rtb_u2deg = SLUGS2_DWork.toWp + 1U;
      if (rtb_u2deg > 255U) {
        rtb_u2deg = 255U;
      }

      SLUGS2_DWork.toWp = (uint8_T)rtb_u2deg;
      if (SLUGS2_DWork.toWp > mlWpValues.wpCount) {
        /* '<S300>:1:18' */
        /* '<S300>:1:19' */
        SLUGS2_DWork.toWp = 1U;
      }
    }

    /*  this is jturned on long as we have not reached IP  */
    if (!(SLUGS2_B.IC != 0)) {
      /* '<S300>:1:25' */
      /* '<S300>:1:26' */
      SLUGS2_DWork.fromWp = 1U;

      /* '<S300>:1:27' */
      SLUGS2_DWork.toWp = 2U;
    }

    /* '<S300>:1:32' */
    SLUGS2_B.WP0 = SLUGS2_DWork.fromWp;

    /* '<S300>:1:33' */
    SLUGS2_B.WP1 = SLUGS2_DWork.toWp;

    /* End of MATLAB Function: '<S180>/computeCurrentWP' */

    /* Outputs for Enabled SubSystem: '<S180>/Get Frenet' incorporates:
     *  EnablePort: '<S296>/Enable'
     */
    /* Gain: '<S343>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S321>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          SLUGS2_B.WP0
          , &SLUGS2_B.GetWPCoordnavsupportcupdated5_d[0]
          );

    /* Gain: '<S344>/Deg2R' */
    rtb_cosphi = 0.0174532924F * SLUGS2_B.GetWPCoordnavsupportcupdated5_d[0];

    /* Trigonometry: '<S344>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S344>/Sum1' incorporates:
     *  Constant: '<S344>/const'
     *  Product: '<S344>/Product1'
     *  Product: '<S344>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S344>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S344>/f' */

    /* Product: '<S344>/Rh' incorporates:
     *  Constant: '<S344>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S344>/Sum2' */
    rtb_RhhcosphicoslambXe = SLUGS2_B.GetWPCoordnavsupportcupdated5_d[2] +
      rtb_Sum1_mj;

    /* Trigonometry: '<S344>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S344>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      SLUGS2_B.GetWPCoordnavsupportcupdated5_d[1];

    /* Product: '<S344>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S344>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S344>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S344>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S344>/Ze' incorporates:
     *  Product: '<S344>/Rh(1-e^2)'
     *  Sum: '<S344>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      SLUGS2_B.GetWPCoordnavsupportcupdated5_d[2];

    /* SignalConversion: '<S343>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S346>/11'
     *  Fcn: '<S346>/12'
     *  Fcn: '<S346>/13'
     *  Fcn: '<S346>/21'
     *  Fcn: '<S346>/22'
     *  Fcn: '<S346>/23'
     *  Fcn: '<S346>/31'
     *  Fcn: '<S346>/32'
     *  Fcn: '<S346>/33'
     */
    tmp_2[0] = (real32_T)cos(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_2[1] = -(real32_T)sin(rtb_Deg2R_h_idx_0);
    tmp_2[2] = -(real32_T)sin(rtb_Product3_bo) * (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_2[3] = (real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_2[4] = (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_2[5] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)sin(rtb_Product3_bo);
    tmp_2[6] = (real32_T)sin(rtb_Product3_bo);
    tmp_2[7] = 0.0F;
    tmp_2[8] = (real32_T)cos(rtb_Product3_bo);

    /* Sum: '<S341>/Sum1' incorporates:
     *  Product: '<S343>/Product1'
     */
    rtb_Deg2R1 -= SLUGS2_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      SLUGS2_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - SLUGS2_B.DataTypeConversion1_l[2];

    /* Product: '<S343>/Product1' incorporates:
     *  Gain: '<S341>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_2[i + 6] * rtb_Sum1_mj + (tmp_2[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_2[i] * rtb_Deg2R1);
    }

    /* Gain: '<S341>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product3_j0[i] = SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2] +
        (SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1] + SLUGS2_ConstP.pooled65[i] *
         tmp_0[0]);
    }

    /* Gain: '<S360>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S322>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          SLUGS2_B.WP1
          , &SLUGS2_B.GetWPCoordnavsupportcupdated5_b[0]
          );

    /* Gain: '<S361>/Deg2R' */
    rtb_cosphi = 0.0174532924F * SLUGS2_B.GetWPCoordnavsupportcupdated5_b[0];

    /* Trigonometry: '<S361>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S361>/Sum1' incorporates:
     *  Constant: '<S361>/const'
     *  Product: '<S361>/Product1'
     *  Product: '<S361>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S361>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S361>/f' */

    /* Product: '<S361>/Rh' incorporates:
     *  Constant: '<S361>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S361>/Sum2' */
    rtb_RhhcosphicoslambXe = SLUGS2_B.GetWPCoordnavsupportcupdated5_b[2] +
      rtb_Sum1_mj;

    /* Trigonometry: '<S361>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S361>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      SLUGS2_B.GetWPCoordnavsupportcupdated5_b[1];

    /* Product: '<S361>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S361>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S361>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S361>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S361>/Ze' incorporates:
     *  Product: '<S361>/Rh(1-e^2)'
     *  Sum: '<S361>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      SLUGS2_B.GetWPCoordnavsupportcupdated5_b[2];

    /* SignalConversion: '<S360>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S363>/11'
     *  Fcn: '<S363>/12'
     *  Fcn: '<S363>/13'
     *  Fcn: '<S363>/21'
     *  Fcn: '<S363>/22'
     *  Fcn: '<S363>/23'
     *  Fcn: '<S363>/31'
     *  Fcn: '<S363>/32'
     *  Fcn: '<S363>/33'
     */
    tmp_3[0] = (real32_T)cos(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_3[1] = -(real32_T)sin(rtb_Deg2R_h_idx_0);
    tmp_3[2] = -(real32_T)sin(rtb_Product3_bo) * (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_3[3] = (real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_3[4] = (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_3[5] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)sin(rtb_Product3_bo);
    tmp_3[6] = (real32_T)sin(rtb_Product3_bo);
    tmp_3[7] = 0.0F;
    tmp_3[8] = (real32_T)cos(rtb_Product3_bo);

    /* Sum: '<S358>/Sum1' incorporates:
     *  Product: '<S360>/Product1'
     */
    rtb_Deg2R1 -= SLUGS2_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      SLUGS2_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - SLUGS2_B.DataTypeConversion1_l[2];

    /* Product: '<S360>/Product1' incorporates:
     *  Gain: '<S358>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_3[i + 6] * rtb_Sum1_mj + (tmp_3[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_3[i] * rtb_Deg2R1);
    }

    /* Gain: '<S358>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product2_c[i] = SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2] +
        (SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1] + SLUGS2_ConstP.pooled65[i] *
         tmp_0[0]);
    }

    rtb_Product3_j0[0] = rtb_Product2_c[0] - rtb_Product3_j0[0];
    rtb_Product3_j0[1] = rtb_Product2_c[1] - rtb_Product3_j0[1];
    rtb_Product3_j0[2] = rtb_Product2_c[2] - rtb_Product3_j0[2];

    /* MATLAB Function: '<S327>/Embedded MATLAB Function' incorporates:
     *  Sum: '<S317>/Add'
     */
    S_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_ih);

    /* MATLAB Function: '<S328>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_ih.xDoty,
                        &SLUGS2_B.sf_negprotect_bi);

    /* S-Function (MCHP_C_function_Call): '<S328>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_pr = mySqrt(
      SLUGS2_B.sf_negprotect_bi.zpVal
      );

    /* Saturate: '<S323>/Zero Bound' */
    if (SLUGS2_B.mySqrtapUtilscupdated5116_pr <= 0.001F) {
      rtb_RhhcosphisinlambYe = 0.001F;
    } else {
      rtb_RhhcosphisinlambYe = SLUGS2_B.mySqrtapUtilscupdated5116_pr;
    }

    /* End of Saturate: '<S323>/Zero Bound' */
    rtb_Product3_j0[0] /= rtb_RhhcosphisinlambYe;
    rtb_Product3_j0[1] /= rtb_RhhcosphisinlambYe;
    rtb_Product3_j0[2] /= rtb_RhhcosphisinlambYe;

    /* MATLAB Function: '<S317>/Select N  Terms' incorporates:
     *  Product: '<S323>/Divide'
     */
    SLUGS2_SelectNTerms(rtb_Product3_j0, &SLUGS2_B.sf_SelectNTerms);

    /* MATLAB Function: '<S334>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(SLUGS2_B.sf_SelectNTerms.N,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_b1);

    /* MATLAB Function: '<S335>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_b1.xDoty,
                        &SLUGS2_B.sf_negprotect_n);

    /* S-Function (MCHP_C_function_Call): '<S335>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_m4 = mySqrt(
      SLUGS2_B.sf_negprotect_n.zpVal
      );

    /* Saturate: '<S324>/Zero Bound' */
    if (SLUGS2_B.mySqrtapUtilscupdated5116_m4 <= 0.001F) {
      rtb_RhhcosphisinlambYe = 0.001F;
    } else {
      rtb_RhhcosphisinlambYe = SLUGS2_B.mySqrtapUtilscupdated5116_m4;
    }

    /* End of Saturate: '<S324>/Zero Bound' */

    /* Product: '<S324>/Divide' */
    rtb_Merge_m[0] = SLUGS2_B.sf_SelectNTerms.N[0] / rtb_RhhcosphisinlambYe;
    rtb_Merge_m[1] = SLUGS2_B.sf_SelectNTerms.N[1] / rtb_RhhcosphisinlambYe;
    rtb_Merge_m[2] = SLUGS2_B.sf_SelectNTerms.N[2] / rtb_RhhcosphisinlambYe;

    /* Delay: '<S296>/Integer Delay1'
     *
     * Block description for '<S296>/Integer Delay1':
     *  TODO: issue with the generated code
     */
    SLUGS2_B.Merge1 = SLUGS2_DWork.IntegerDelay1_DSTATE_gn;

    /* MATLAB Function: '<S296>/Zero out Z1' */
    SLUGS2_ZerooutZ1(rtb_Merge_m, &SLUGS2_B.sf_ZerooutZ1_j);

    /* MATLAB Function: '<S296>/Zero out Z2' */
    SLUGS2_ZerooutZ1(rtb_Product3_j0, &SLUGS2_B.sf_ZerooutZ2_e);

    /* MATLAB Function: '<S296>/Zero out Z3' */
    SLUGS2_ZerooutZ1(rtb_Product2_c, &SLUGS2_B.sf_ZerooutZ3);

    /* Product: '<S296>/Product1' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    SLUGS2_B.Product1 = mlParamInterface.param[20] *
      SLUGS2_B.mySqrtapUtilscupdated5116;

    /* Update for Delay: '<S296>/Integer Delay1'
     *
     * Block description for '<S296>/Integer Delay1':
     *  TODO: issue with the generated code
     */
    SLUGS2_DWork.IntegerDelay1_DSTATE_gn = rtb_Product2_c[2];

    /* End of Outputs for SubSystem: '<S180>/Get Frenet' */

    /* Sum: '<S294>/Subtract' */
    rtb_Product3_j0[0] = SLUGS2_B.sf_ZerooutZ2.P[0] - SLUGS2_B.sf_ZerooutZ3.P[0];
    rtb_Product3_j0[1] = SLUGS2_B.sf_ZerooutZ2.P[1] - SLUGS2_B.sf_ZerooutZ3.P[1];
    rtb_Product3_j0[2] = SLUGS2_B.sf_ZerooutZ2.P[2] - SLUGS2_B.sf_ZerooutZ3.P[2];

    /* MATLAB Function: '<S302>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_k(rtb_Product3_j0, SLUGS2_B.sf_ZerooutZ1_j.P,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_o);

    /* S-Function (MCHP_C_function_Call): '<S303>/myAbs() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.myAbsapUtilscupdated5116_i = myAbs(
      SLUGS2_B.sf_EmbeddedMATLABFunction_o.xDoty
      );

    /* Sum: '<S301>/Subtract' */
    rtb_Product2_c[0] = SLUGS2_B.sf_ZerooutZ3.P[0] - SLUGS2_B.sf_ZerooutZ2.P[0];
    rtb_Product2_c[1] = SLUGS2_B.sf_ZerooutZ3.P[1] - SLUGS2_B.sf_ZerooutZ2.P[1];
    rtb_Product2_c[2] = SLUGS2_B.sf_ZerooutZ3.P[2] - SLUGS2_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S309>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_k(rtb_Product2_c, SLUGS2_B.sf_ZerooutZ2_e.P,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_k);

    /* Switch: '<S308>/Switch3' incorporates:
     *  RelationalOperator: '<S308>/Relational Operator2'
     */
    if ((SLUGS2_B.sf_EmbeddedMATLABFunction_k.xDoty ==
         SLUGS2_B.sf_EmbeddedMATLABFunction_k.xDoty) > 0) {
      SLUGS2_DWork.IntegerDelay3_DSTATE_p =
        SLUGS2_B.sf_EmbeddedMATLABFunction_k.xDoty;
    }

    /* End of Switch: '<S308>/Switch3' */

    /* Product: '<S294>/Divide' incorporates:
     *  Constant: '<S294>/Constant4'
     */
    rtb_Sum1_mj = SLUGS2_B.myAbsapUtilscupdated5116_i / 0.577350259F;

    /* Switch: '<S307>/Switch' incorporates:
     *  Product: '<S294>/Divide1'
     *  RelationalOperator: '<S307>/Relational Operator'
     */
    if (rtb_Product < rtb_Sum1_mj) {
      rtb_Sum1_mj = rtb_Product;
    }

    /* End of Switch: '<S307>/Switch' */

    /* MATLAB Function: '<S304>/negprotect' incorporates:
     *  Product: '<S294>/Product'
     *  Product: '<S294>/Product1'
     *  Sum: '<S294>/Add'
     */
    SLUGS2_negprotect_l(rtb_Product * rtb_Product -
                        SLUGS2_B.myAbsapUtilscupdated5116_i *
                        SLUGS2_B.myAbsapUtilscupdated5116_i,
                        &SLUGS2_B.sf_negprotect_p);

    /* S-Function (MCHP_C_function_Call): '<S304>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_d = mySqrt(
      SLUGS2_B.sf_negprotect_p.zpVal
      );

    /* Switch: '<S180>/Switch' incorporates:
     *  Product: '<S294>/Product2'
     *  Sum: '<S294>/Add3'
     *  Switch: '<S180>/Switch1'
     */
    if (SLUGS2_B.IC > 0) {
      /* Switch: '<S306>/Switch' incorporates:
       *  RelationalOperator: '<S294>/Relational Operator'
       *  RelationalOperator: '<S306>/Relational Operator'
       *  Switch: '<S294>/Switch1'
       */
      if ((!(SLUGS2_B.myAbsapUtilscupdated5116_i > rtb_Product)) &&
          (!(rtb_Sum1_mj > SLUGS2_B.mySqrtapUtilscupdated5116_d))) {
        rtb_Sum1_mj = SLUGS2_B.mySqrtapUtilscupdated5116_d;
      }

      /* End of Switch: '<S306>/Switch' */

      /* Sum: '<S294>/Add1' */
      rtb_RhhcosphisinlambYe = SLUGS2_DWork.IntegerDelay3_DSTATE_p - rtb_Sum1_mj;

      /* Switch: '<S305>/Switch' incorporates:
       *  Constant: '<S294>/Constant1'
       *  RelationalOperator: '<S305>/Relational Operator'
       */
      if (!(rtb_RhhcosphisinlambYe > 0.0F)) {
        rtb_RhhcosphisinlambYe = 0.0F;
      }

      /* End of Switch: '<S305>/Switch' */
      SLUGS2_B.Merge_o[0] = (0.0F - rtb_RhhcosphisinlambYe *
        SLUGS2_B.sf_ZerooutZ2_e.P[0]) - rtb_Product3_j0[0];
      SLUGS2_B.Merge_o[1] = (0.0F - rtb_RhhcosphisinlambYe *
        SLUGS2_B.sf_ZerooutZ2_e.P[1]) - rtb_Product3_j0[1];
      SLUGS2_B.Merge_o[2] = (0.0F - rtb_RhhcosphisinlambYe *
        SLUGS2_B.sf_ZerooutZ2_e.P[2]) - rtb_Product3_j0[2];
      SLUGS2_B.Merge2 = SLUGS2_DWork.IntegerDelay3_DSTATE_p;
    } else {
      SLUGS2_B.Merge2 = SLUGS2_B.mySqrtapUtilscupdated5116_b;
    }

    /* End of Switch: '<S180>/Switch' */

    /* Update for Delay: '<S180>/Integer Delay' incorporates:
     *  RelationalOperator: '<S294>/Switch Distance Less than?'
     */
    SLUGS2_DWork.IntegerDelay_DSTATE_im = (uint8_T)
      (SLUGS2_DWork.IntegerDelay3_DSTATE_p < SLUGS2_B.Product1);

    /* End of Outputs for SubSystem: '<S173>/Normal WP  Navigation' */
  } else if (rtb_Compare_j == 10) {
    /* Outputs for IfAction SubSystem: '<S173>/Line Segment' incorporates:
     *  ActionPort: '<S178>/Action Port'
     */
    /* Gain: '<S178>/Gain' */
    SLUGS2_B.Merge_o[0] = -SLUGS2_B.sf_ZerooutZ2.P[0];
    SLUGS2_B.Merge_o[1] = -SLUGS2_B.sf_ZerooutZ2.P[1];
    SLUGS2_B.Merge_o[2] = -SLUGS2_B.sf_ZerooutZ2.P[2];

    /* Gain: '<S178>/Gain1' */
    rtb_Product3_j0[0] = -SLUGS2_B.sf_ZerooutZ2.P[0];
    rtb_Product3_j0[1] = -SLUGS2_B.sf_ZerooutZ2.P[1];
    rtb_Product3_j0[2] = -SLUGS2_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S288>/Embedded MATLAB Function' */
    S_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &SLUGS2_B.sf_EmbeddedMATLABFunction_le);

    /* MATLAB Function: '<S289>/negprotect' */
    SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_le.xDoty,
                        &SLUGS2_B.sf_negprotect_o4);

    /* S-Function (MCHP_C_function_Call): '<S289>/mySqrt() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.mySqrtapUtilscupdated5116_pu = mySqrt(
      SLUGS2_B.sf_negprotect_o4.zpVal
      );

    /* SignalConversion: '<S292>/Numerical Unity' */
    SLUGS2_B.Merge2 = SLUGS2_B.mySqrtapUtilscupdated5116_pu;
    SLUGS2_B.Merge1 = mlMidLevelCommands.hCommand;
    SLUGS2_B.Merge3 = 0U;
    SLUGS2_B.Merge4 = 0U;

    /* End of Outputs for SubSystem: '<S173>/Line Segment' */
  } else {
    if (rtb_Compare_j == 9) {
      /* Outputs for IfAction SubSystem: '<S173>/Circle Navigation' incorporates:
       *  ActionPort: '<S174>/Action Port'
       */
      rtb_IC4_idx_0 *= 0.0174532924F;

      /* Gain: '<S199>/Deg2R' */
      rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_1;

      /* Gain: '<S200>/Deg2R' incorporates:
       *  DataStoreRead: '<S65>/Get ISR Location'
       */
      rtb_RhhcosphicoslambXe = 0.0174532924F * mlISR.latitude;

      /* Trigonometry: '<S200>/sin(phi)' */
      rtb_Ze_b = (real32_T)sin(rtb_RhhcosphicoslambXe);

      /* Sum: '<S200>/Sum1' incorporates:
       *  Constant: '<S200>/const'
       *  Product: '<S200>/Product1'
       *  Product: '<S200>/sin(phi)^2'
       */
      rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

      /* Fcn: '<S200>/f' */
      if (rtb_Sum1_mj < 0.0F) {
        rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
      } else {
        rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
      }

      /* End of Fcn: '<S200>/f' */

      /* Product: '<S200>/Rh' incorporates:
       *  Constant: '<S200>/Re=equatorial radius'
       */
      rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

      /* Sum: '<S200>/Sum2' incorporates:
       *  DataStoreRead: '<S65>/Get ISR Location'
       */
      rtb_cosphi = mlISR.height + rtb_Sum1_mj;

      /* Trigonometry: '<S200>/cos(phi)' */
      rtb_RhhcosphicoslambXe = (real32_T)cos(rtb_RhhcosphicoslambXe);

      /* Gain: '<S200>/Deg2R1' incorporates:
       *  DataStoreRead: '<S65>/Get ISR Location'
       */
      rtb_RhhcosphisinlambYe = 0.0174532924F * mlISR.longitude;

      /* Product: '<S200>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
       *  Trigonometry: '<S200>/cos(lamb)'
       */
      rtb_Deg2R1 = rtb_cosphi * rtb_RhhcosphicoslambXe * (real32_T)cos
        (rtb_RhhcosphisinlambYe);

      /* Product: '<S200>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
       *  Trigonometry: '<S200>/sin(lamb)'
       */
      rtb_cosphi = rtb_cosphi * rtb_RhhcosphicoslambXe * (real32_T)sin
        (rtb_RhhcosphisinlambYe);

      /* Product: '<S200>/Ze' incorporates:
       *  DataStoreRead: '<S65>/Get ISR Location'
       *  Product: '<S200>/Rh(1-e^2)'
       *  Sum: '<S200>/Sum4'
       */
      rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj + mlISR.height;

      /* SignalConversion: '<S199>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S202>/11'
       *  Fcn: '<S202>/12'
       *  Fcn: '<S202>/13'
       *  Fcn: '<S202>/21'
       *  Fcn: '<S202>/22'
       *  Fcn: '<S202>/23'
       *  Fcn: '<S202>/31'
       *  Fcn: '<S202>/32'
       *  Fcn: '<S202>/33'
       */
      tmp[0] = (real32_T)cos(rtb_IC4_idx_0) * (real32_T)cos(rtb_Deg2R_h_idx_0);
      tmp[1] = -(real32_T)sin(rtb_IC4_idx_0);
      tmp[2] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_IC4_idx_0);
      tmp[3] = (real32_T)sin(rtb_IC4_idx_0) * (real32_T)cos(rtb_Deg2R_h_idx_0);
      tmp[4] = (real32_T)cos(rtb_IC4_idx_0);
      tmp[5] = -(real32_T)sin(rtb_IC4_idx_0) * (real32_T)sin(rtb_Deg2R_h_idx_0);
      tmp[6] = (real32_T)sin(rtb_Deg2R_h_idx_0);
      tmp[7] = 0.0F;
      tmp[8] = (real32_T)cos(rtb_Deg2R_h_idx_0);

      /* Sum: '<S197>/Sum1' incorporates:
       *  Product: '<S199>/Product1'
       */
      rtb_Deg2R1 -= SLUGS2_B.DataTypeConversion1_l[0];
      rtb_Sum1_mj = rtb_cosphi - SLUGS2_B.DataTypeConversion1_l[1];
      rtb_RhhcosphisinlambYe = rtb_Ze_b - SLUGS2_B.DataTypeConversion1_l[2];

      /* Product: '<S199>/Product1' incorporates:
       *  Gain: '<S197>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp_0[i] = tmp[i + 6] * rtb_RhhcosphisinlambYe + (tmp[i + 3] *
          rtb_Sum1_mj + tmp[i] * rtb_Deg2R1);
      }

      /* Gain: '<S197>/UEN 2 NEU' */
      for (i = 0; i < 3; i++) {
        rtb_Product3_j0[i] = SLUGS2_ConstP.pooled65[i + 6] * tmp_0[2] +
          (SLUGS2_ConstP.pooled65[i + 3] * tmp_0[1] + SLUGS2_ConstP.pooled65[i] *
           tmp_0[0]);
      }

      /* Delay: '<S191>/Integer Delay1'
       *
       * Block description for '<S191>/Integer Delay1':
       *  TODO: check that it pull the right entry
       */
      SLUGS2_B.Merge1 = SLUGS2_DWork.IntegerDelay1_DSTATE_a;

      /* MATLAB Function: '<S191>/Zero out Z2' */
      SLUGS2_ZerooutZ1(rtb_Product3_j0, &SLUGS2_B.sf_ZerooutZ2_h2);

      /* Sum: '<S174>/Sum1' */
      rtb_Product2_c[0] = SLUGS2_B.sf_ZerooutZ2.P[0] -
        SLUGS2_B.sf_ZerooutZ2_h2.P[0];
      rtb_Product2_c[1] = SLUGS2_B.sf_ZerooutZ2.P[1] -
        SLUGS2_B.sf_ZerooutZ2_h2.P[1];
      rtb_Product2_c[2] = SLUGS2_B.sf_ZerooutZ2.P[2] -
        SLUGS2_B.sf_ZerooutZ2_h2.P[2];

      /* MATLAB Function: '<S243>/Embedded MATLAB Function' */
      S_EmbeddedMATLABFunction_b(rtb_Product2_c,
        &SLUGS2_B.sf_EmbeddedMATLABFunction_c);

      /* MATLAB Function: '<S244>/negprotect' */
      SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_c.xDoty,
                          &SLUGS2_B.sf_negprotect_lu);

      /* S-Function (MCHP_C_function_Call): '<S244>/mySqrt() apUtils.c [updated 5.1.16]' */
      SLUGS2_B.mySqrtapUtilscupdated5116_a = mySqrt(
        SLUGS2_B.sf_negprotect_lu.zpVal
        );

      /* Product: '<S174>/Product' incorporates:
       *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
       */
      rtb_RhhcosphisinlambYe = mlParamInterface.param[19] *
        SLUGS2_B.mySqrtapUtilscupdated5116;

      /* Sum: '<S174>/Sum2' */
      SLUGS2_B.Sum2 = rtb_Product3_bo - rtb_RhhcosphisinlambYe;

      /* S-Function (MCHP_C_function_Call): '<S196>/myAbs() apUtils.c [updated 5.1.16]' */
      SLUGS2_B.myAbsapUtilscupdated5116_a = myAbs(
        SLUGS2_B.Sum2
        );

      /* If: '<S174>/If' incorporates:
       *  Constant: '<S192>/RTB1'
       *  Constant: '<S193>/RTB1'
       *  Constant: '<S194>/RTB1'
       *  Product: '<S193>/Product6'
       *  Product: '<S193>/Product7'
       *  Sum: '<S174>/Sum'
       *  Sum: '<S193>/Subtract3'
       *  Sum: '<S193>/Subtract6'
       */
      if (SLUGS2_B.mySqrtapUtilscupdated5116_a > rtb_RhhcosphisinlambYe +
          rtb_Product3_bo) {
        /* Outputs for IfAction SubSystem: '<S174>/No intersection,  Navigate to ISR' incorporates:
         *  ActionPort: '<S194>/Action Port'
         */
        /* Sum: '<S194>/Subtract' */
        SLUGS2_B.Merge_o[0] = SLUGS2_B.sf_ZerooutZ2_h2.P[0] -
          SLUGS2_B.sf_ZerooutZ2.P[0];
        SLUGS2_B.Merge_o[1] = SLUGS2_B.sf_ZerooutZ2_h2.P[1] -
          SLUGS2_B.sf_ZerooutZ2.P[1];
        SLUGS2_B.Merge_o[2] = SLUGS2_B.sf_ZerooutZ2_h2.P[2] -
          SLUGS2_B.sf_ZerooutZ2.P[2];
        SLUGS2_B.Merge3 = 0U;

        /* End of Outputs for SubSystem: '<S174>/No intersection,  Navigate to ISR' */
      } else if (SLUGS2_B.mySqrtapUtilscupdated5116_a <
                 SLUGS2_B.myAbsapUtilscupdated5116_a) {
        /* Outputs for IfAction SubSystem: '<S174>/Inside the Circle,  Keep Straight until  intersection' incorporates:
         *  ActionPort: '<S192>/Action Port'
         */
        /* Sum: '<S192>/Subtract' incorporates:
         *  MATLAB Function: '<S192>/Compute Head of Circle'
         */
        /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle': '<S204>:1' */
        /* % Compute the top coordinate of the circle */
        /*  using the circle's parametric equations: */
        /*   x = a + r cos (t) */
        /*   y = b + r sin (t) */
        /*  */
        /*  @ t =0 */
        /* '<S204>:1:9' */
        /* '<S204>:1:11' */
        /* '<S204>:1:12' */
        /* '<S204>:1:13' */
        SLUGS2_B.Merge_o[0] = (real32_T)((real_T)(SLUGS2_B.sf_ZerooutZ2_h2.P[0]
          + rtb_Product3_bo) - SLUGS2_B.sf_ZerooutZ2.P[0]);
        SLUGS2_B.Merge_o[1] = (real32_T)((real_T)SLUGS2_B.sf_ZerooutZ2_h2.P[1] -
          SLUGS2_B.sf_ZerooutZ2.P[1]);
        SLUGS2_B.Merge_o[2] = (real32_T)((real_T)SLUGS2_B.sf_ZerooutZ2_h2.P[2] -
          SLUGS2_B.sf_ZerooutZ2.P[2]);
        SLUGS2_B.Merge3 = 1U;

        /* End of Outputs for SubSystem: '<S174>/Inside the Circle,  Keep Straight until  intersection' */
      } else {
        /* Outputs for IfAction SubSystem: '<S174>/Intersection. Circular Navigation' incorporates:
         *  ActionPort: '<S193>/Action Port'
         */
        /* Product: '<S193>/Product' */
        rtb_Sum1_mj = rtb_Product3_bo * rtb_Product3_bo;

        /* MATLAB Function: '<S193>/negprotect2' incorporates:
         *  Sum: '<S193>/Subtract1'
         */
        SLUGS2_negprotect3(SLUGS2_B.mySqrtapUtilscupdated5116_a +
                           SLUGS2_B.mySqrtapUtilscupdated5116_a,
                           &SLUGS2_B.sf_negprotect2);

        /* Product: '<S193>/Product3' incorporates:
         *  Product: '<S193>/Product1'
         *  Product: '<S193>/Product2'
         *  Sum: '<S193>/Subtract'
         */
        rtb_RhhcosphisinlambYe = ((rtb_Sum1_mj - rtb_RhhcosphisinlambYe *
          rtb_RhhcosphisinlambYe) + SLUGS2_B.mySqrtapUtilscupdated5116_a *
          SLUGS2_B.mySqrtapUtilscupdated5116_a) / SLUGS2_B.sf_negprotect2.zpVal;

        /* MATLAB Function: '<S210>/negprotect' incorporates:
         *  Product: '<S193>/Product4'
         *  Sum: '<S193>/Subtract2'
         */
        SLUGS2_negprotect_l(rtb_Sum1_mj - rtb_RhhcosphisinlambYe *
                            rtb_RhhcosphisinlambYe, &SLUGS2_B.sf_negprotect_ne);

        /* S-Function (MCHP_C_function_Call): '<S210>/mySqrt() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.mySqrtapUtilscupdated5116_m = mySqrt(
          SLUGS2_B.sf_negprotect_ne.zpVal
          );

        /* MATLAB Function: '<S193>/negprotect1' */
        SLUGS2_negprotect3(SLUGS2_B.mySqrtapUtilscupdated5116_a,
                           &SLUGS2_B.sf_negprotect1);
        rtb_Product2_c[0] /= SLUGS2_B.sf_negprotect1.zpVal;
        rtb_Product2_c[1] /= SLUGS2_B.sf_negprotect1.zpVal;

        /* Product: '<S193>/Product5' incorporates:
         *  Product: '<S193>/Product6'
         */
        rtb_Merge_m[0] = SLUGS2_B.mySqrtapUtilscupdated5116_m * rtb_Product2_c[0];
        rtb_Merge_m[1] = SLUGS2_B.mySqrtapUtilscupdated5116_m * rtb_Product2_c[1];
        rtb_Product2_c[0] = rtb_Product2_c[0] * rtb_RhhcosphisinlambYe +
          SLUGS2_B.sf_ZerooutZ2_h2.P[0];
        rtb_Product2_c[1] = rtb_Product2_c[1] * rtb_RhhcosphisinlambYe +
          SLUGS2_B.sf_ZerooutZ2_h2.P[1];

        /* MATLAB Function: '<S193>/Embedded MATLAB Function' incorporates:
         *  Product: '<S193>/Product7'
         *  Sum: '<S193>/Subtract3'
         */
        /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function': '<S207>:1' */
        /*  This block supports an embeddable subset of the MATLAB language. */
        /*  See the help menu for details.  */
        /* '<S207>:1:5' */
        SLUGS2_B.Merge_o[0] = 0.0F;
        SLUGS2_B.Merge_o[1] = 0.0F;
        SLUGS2_B.Merge_o[2] = 0.0F;

        /* '<S207>:1:6' */
        /* '<S207>:1:8' */
        SLUGS2_B.Merge_o[0] = rtb_Product2_c[0] + rtb_Merge_m[1];

        /* '<S207>:1:9' */
        rtb_P32[0] = rtb_Product2_c[0] - rtb_Merge_m[1];

        /* '<S207>:1:11' */
        SLUGS2_B.Merge_o[1] = rtb_Product2_c[1] - rtb_Merge_m[0];

        /* '<S207>:1:12' */
        rtb_P32[1] = rtb_Product2_c[1] + rtb_Merge_m[0];
        rtb_P32[0] -= SLUGS2_B.sf_ZerooutZ2.P[0];
        rtb_P32[1] -= SLUGS2_B.sf_ZerooutZ2.P[1];
        rtb_P32[2] = 0.0F - SLUGS2_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S214>/Embedded MATLAB Function' incorporates:
         *  Sum: '<S193>/Subtract6'
         */
        S_EmbeddedMATLABFunction_k(SLUGS2_B.sf_ZerooutZ1.P, rtb_P32,
          &SLUGS2_B.sf_EmbeddedMATLABFunction_p);

        /* MATLAB Function: '<S217>/Embedded MATLAB Function' */
        S_EmbeddedMATLABFunction_b(rtb_P32,
          &SLUGS2_B.sf_EmbeddedMATLABFunction_bp);

        /* MATLAB Function: '<S218>/negprotect' */
        SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_bp.xDoty,
                            &SLUGS2_B.sf_negprotect_c);

        /* S-Function (MCHP_C_function_Call): '<S218>/mySqrt() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.mySqrtapUtilscupdated5116_j = mySqrt(
          SLUGS2_B.sf_negprotect_c.zpVal
          );

        /* MATLAB Function: '<S205>/negprotect3' incorporates:
         *  Product: '<S205>/Product9'
         */
        SLUGS2_negprotect3(SLUGS2_B.mySqrtapUtilscupdated5116_j *
                           SLUGS2_B.mySqrtapUtilscupdated5116,
                           &SLUGS2_B.sf_negprotect3);

        /* Product: '<S205>/Product8' */
        SLUGS2_B.u1 = SLUGS2_B.sf_EmbeddedMATLABFunction_p.xDoty /
          SLUGS2_B.sf_negprotect3.zpVal;

        /* Saturate: '<S205>/[-1 1]' */
        if (SLUGS2_B.u1 > 1.0F) {
          /* Product: '<S205>/Product8' */
          SLUGS2_B.u1 = 1.0F;
        } else {
          if (SLUGS2_B.u1 < -1.0F) {
            /* Product: '<S205>/Product8' */
            SLUGS2_B.u1 = -1.0F;
          }
        }

        /* End of Saturate: '<S205>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S215>/myAcos() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.myAcosapUtilscupdated5116 = myAcos(
          SLUGS2_B.u1
          );

        /* Sum: '<S193>/Subtract5' */
        SLUGS2_B.Merge_o[0] -= SLUGS2_B.sf_ZerooutZ2.P[0];
        SLUGS2_B.Merge_o[1] -= SLUGS2_B.sf_ZerooutZ2.P[1];
        SLUGS2_B.Merge_o[2] -= SLUGS2_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S227>/Embedded MATLAB Function' */
        S_EmbeddedMATLABFunction_k(SLUGS2_B.sf_ZerooutZ1.P, SLUGS2_B.Merge_o,
          &SLUGS2_B.sf_EmbeddedMATLABFunction_m);

        /* MATLAB Function: '<S230>/Embedded MATLAB Function' */
        S_EmbeddedMATLABFunction_b(SLUGS2_B.Merge_o,
          &SLUGS2_B.sf_EmbeddedMATLABFunction_eg);

        /* MATLAB Function: '<S231>/negprotect' */
        SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_eg.xDoty,
                            &SLUGS2_B.sf_negprotect_i);

        /* S-Function (MCHP_C_function_Call): '<S231>/mySqrt() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.mySqrtapUtilscupdated5116_e = mySqrt(
          SLUGS2_B.sf_negprotect_i.zpVal
          );

        /* MATLAB Function: '<S206>/negprotect3' incorporates:
         *  Product: '<S206>/Product9'
         */
        SLUGS2_negprotect3(SLUGS2_B.mySqrtapUtilscupdated5116_e *
                           SLUGS2_B.mySqrtapUtilscupdated5116,
                           &SLUGS2_B.sf_negprotect3_i);

        /* Product: '<S206>/Product8' */
        SLUGS2_B.u1_h = SLUGS2_B.sf_EmbeddedMATLABFunction_m.xDoty /
          SLUGS2_B.sf_negprotect3_i.zpVal;

        /* Saturate: '<S206>/[-1 1]' */
        if (SLUGS2_B.u1_h > 1.0F) {
          /* Product: '<S206>/Product8' */
          SLUGS2_B.u1_h = 1.0F;
        } else {
          if (SLUGS2_B.u1_h < -1.0F) {
            /* Product: '<S206>/Product8' */
            SLUGS2_B.u1_h = -1.0F;
          }
        }

        /* End of Saturate: '<S206>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S228>/myACos() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.myACosapUtilscupdated5116 = myAcos(
          SLUGS2_B.u1_h
          );

        /* S-Function (MCHP_C_function_Call): '<S209>/myAbs() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.myAbsapUtilscupdated5116_c = myAbs(
          SLUGS2_B.myACosapUtilscupdated5116
          );

        /* S-Function (MCHP_C_function_Call): '<S208>/myAbs() apUtils.c [updated 5.1.16]' */
        SLUGS2_B.myAbsapUtilscupdated5116_b = myAbs(
          SLUGS2_B.myAcosapUtilscupdated5116
          );

        /* Switch: '<S193>/Switch1' incorporates:
         *  RelationalOperator: '<S193>/Relational Operator'
         *  Sum: '<S193>/Subtract6'
         */
        if (!(SLUGS2_B.myAbsapUtilscupdated5116_c <=
              SLUGS2_B.myAbsapUtilscupdated5116_b)) {
          SLUGS2_B.Merge_o[0] = rtb_P32[0];
          SLUGS2_B.Merge_o[1] = rtb_P32[1];
          SLUGS2_B.Merge_o[2] = 0.0F - SLUGS2_B.sf_ZerooutZ2.P[2];
        }

        /* End of Switch: '<S193>/Switch1' */
        SLUGS2_B.Merge3 = 2U;

        /* End of Outputs for SubSystem: '<S174>/Intersection. Circular Navigation' */
      }

      /* End of If: '<S174>/If' */

      /* Delay: '<S174>/Integer Delay' */
      SLUGS2_B.Merge2 = SLUGS2_DWork.IntegerDelay_DSTATE_h;
      SLUGS2_B.Merge4 = 0U;

      /* Update for Delay: '<S191>/Integer Delay1' incorporates:
       *  Constant: '<S174>/RTB1'
       *
       * Block description for '<S191>/Integer Delay1':
       *  TODO: check that it pull the right entry
       */
      SLUGS2_DWork.IntegerDelay1_DSTATE_a = rtb_Product3_j0[2];

      /* Update for Delay: '<S174>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_h = rtb_Product3_bo;

      /* End of Outputs for SubSystem: '<S173>/Circle Navigation' */
    }
  }

  /* End of If: '<S173>/Determine Overall Nav by the Nav Mode' */

  /* MATLAB Function: '<S270>/Zero out Z1' */
  SLUGS2_ZerooutZ1(SLUGS2_B.Merge_o, &SLUGS2_B.sf_ZerooutZ1_i);

  /* MATLAB Function: '<S275>/Embedded MATLAB Function' */
  S_EmbeddedMATLABFunction_b(SLUGS2_B.sf_ZerooutZ1_i.P,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_nv);

  /* MATLAB Function: '<S276>/negprotect' */
  SLUGS2_negprotect_l(SLUGS2_B.sf_EmbeddedMATLABFunction_nv.xDoty,
                      &SLUGS2_B.sf_negprotect_o);

  /* S-Function (MCHP_C_function_Call): '<S276>/mySqrt() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.mySqrtapUtilscupdated5116_p = mySqrt(
    SLUGS2_B.sf_negprotect_o.zpVal
    );

  /* MATLAB Function: '<S262>/negprotect' incorporates:
   *  Product: '<S262>/Product1'
   */
  SLUGS2_negprotect3(SLUGS2_B.mySqrtapUtilscupdated5116_p *
                     SLUGS2_B.mySqrtapUtilscupdated5116,
                     &SLUGS2_B.sf_negprotect_h);

  /* DeadZone: '<S262>/Dead Zone' */
  if (SLUGS2_B.sf_negprotect_h.zpVal > 0.1F) {
    rtb_RhhcosphicoslambXe = SLUGS2_B.sf_negprotect_h.zpVal - 0.1F;
  } else if (SLUGS2_B.sf_negprotect_h.zpVal >= -0.1F) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = SLUGS2_B.sf_negprotect_h.zpVal - -0.1F;
  }

  /* End of DeadZone: '<S262>/Dead Zone' */

  /* Switch: '<S262>/Switch' incorporates:
   *  Constant: '<S262>/cos(pi//2)'
   *  Constant: '<S262>/sin(pi//2)'
   *  Constant: '<S269>/Constant'
   *  Product: '<S262>/Divide1'
   *  Product: '<S262>/Divide2'
   *  Product: '<S268>/Product'
   *  Product: '<S268>/Product1'
   *  Product: '<S271>/Product'
   *  Product: '<S271>/Product1'
   *  RelationalOperator: '<S269>/Compare'
   *  Sum: '<S268>/Subtract'
   *  Sum: '<S271>/Subtract'
   *  Switch: '<S262>/Switch2'
   */
  if ((rtb_RhhcosphicoslambXe == 0.0F) > 0) {
    SLUGS2_B.Switch = 1.0F;
    SLUGS2_B.Switch2 = 0.0F;
  } else {
    SLUGS2_B.Switch = (SLUGS2_B.Merge_o[1] * SLUGS2_B.sf_ZerooutZ1.P[0] -
                       SLUGS2_B.Merge_o[0] * SLUGS2_B.sf_ZerooutZ1.P[1]) /
      SLUGS2_B.sf_negprotect_h.zpVal;
    SLUGS2_B.Switch2 = (SLUGS2_B.Merge_o[0] * SLUGS2_B.sf_ZerooutZ1.P[0] +
                        SLUGS2_B.Merge_o[1] * SLUGS2_B.sf_ZerooutZ1.P[1]) *
      (1.0F / SLUGS2_B.sf_negprotect_h.zpVal);
  }

  /* End of Switch: '<S262>/Switch' */

  /* S-Function (MCHP_C_function_Call): '<S266>/myAtan2() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myAtan2apUtilscupdated5116 = myAtan2(
    SLUGS2_B.Switch
    , SLUGS2_B.Switch2
    );

  /* Delay: '<S173>/Integer Delay' */
  SLUGS2_B.Merge3 = SLUGS2_DWork.IntegerDelay_DSTATE_c;

  /* Delay: '<S173>/Integer Delay1' */
  SLUGS2_B.Merge4 = SLUGS2_DWork.IntegerDelay1_DSTATE_o;

  /* MATLAB Function: '<S173>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S173>/Data Type Conversion5'
   *  DataTypeConversion: '<S173>/Data Type Conversion6'
   *  Gain: '<S177>/Rad2Deg'
   */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun1': '<S185>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S185>:1:4' */
  SLUGS2_B.y_b[0] = 57.2957802F * SLUGS2_B.myAtan2apUtilscupdated5116;
  SLUGS2_B.y_b[1] = SLUGS2_B.Merge2;
  SLUGS2_B.y_b[2] = SLUGS2_B.Merge3;
  SLUGS2_B.y_b[3] = SLUGS2_B.Merge4;

  /* S-Function (MCHP_C_function_Call): '<S173>/getXYZ() navsupport.c [updated 5.1.16]' */
  setNavNav(
            &SLUGS2_B.y_b[0]
            );

  /* MATLAB Function: '<S263>/negprotect' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
   */
  SLUGS2_negprotect3(mlParamInterface.param[19], &SLUGS2_B.sf_negprotect_b);

  /* S-Function (MCHP_C_function_Call): '<S265>/myAbs() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myAbsapUtilscupdated5116 = myAbs(
    SLUGS2_B.myAtan2apUtilscupdated5116
    );

  /* Switch: '<S177>/Switch' incorporates:
   *  Constant: '<S177>/Constant10'
   *  Constant: '<S286>/Constant'
   *  Gain: '<S263>/Gain'
   *  Product: '<S263>/Divide'
   *  RelationalOperator: '<S177>/Relational Operator'
   *  RelationalOperator: '<S286>/Compare'
   *  Switch: '<S267>/Switch1'
   */
  if (SLUGS2_B.myAbsapUtilscupdated5116 <= 1.57079637F) {
    rtb_Sum1_mj = SLUGS2_B.Switch * SLUGS2_B.mySqrtapUtilscupdated5116 /
      SLUGS2_B.sf_negprotect_b.zpVal * 2.0F;
  } else if ((SLUGS2_B.Switch < 0.0F) > 0) {
    /* Switch: '<S267>/Switch1' incorporates:
     *  Constant: '<S267>/Constant9'
     */
    rtb_Sum1_mj = -4.57681F;
  } else {
    /* Switch: '<S267>/Switch1' incorporates:
     *  Constant: '<S267>/Constant1'
     */
    rtb_Sum1_mj = 4.57681F;
  }

  /* End of Switch: '<S177>/Switch' */

  /* Product: '<S264>/Product' incorporates:
   *  Constant: '<S264>/Constant'
   */
  SLUGS2_B.Product = rtb_Sum1_mj / 9.815F;

  /* S-Function (MCHP_C_function_Call): '<S282>/myAtan() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myAtanapUtilscupdated5116 = myAtan(
    SLUGS2_B.Product
    );

  /* Saturate: '<S264>/Bank  Limit Command' */
  if (SLUGS2_B.myAtanapUtilscupdated5116 > 0.436332315F) {
    rtb_cosphi = 0.436332315F;
  } else if (SLUGS2_B.myAtanapUtilscupdated5116 < -0.436332315F) {
    rtb_cosphi = -0.436332315F;
  } else {
    rtb_cosphi = SLUGS2_B.myAtanapUtilscupdated5116;
  }

  /* End of Saturate: '<S264>/Bank  Limit Command' */

  /* InitialCondition: '<S182>/IC3' */
  if (SLUGS2_DWork.IC3_FirstOutputTime) {
    SLUGS2_DWork.IC3_FirstOutputTime = false;
    SLUGS2_B.IC3 = 166.3F;
  } else {
    SLUGS2_B.IC3 = SLUGS2_B.NumericalUnity[0];
  }

  /* End of InitialCondition: '<S182>/IC3' */

  /* Update for Delay: '<S457>/Delay' */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun3': '<S187>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S187>:1:5' */
  SLUGS2_DWork.Delay_DSTATE_e = rtb_IC1;

  /* Update for Delay: '<S173>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_c = SLUGS2_B.WP0;

  /* Update for Delay: '<S173>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_o = SLUGS2_B.WP1;

  /* End of Outputs for SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */

  /* Switch: '<S66>/Switch3' incorporates:
   *  Delay: '<S66>/Integer Delay3'
   *  RelationalOperator: '<S66>/Relational Operator2'
   */
  if ((SLUGS2_B.sf_myMuxFun1_e.y[2] == SLUGS2_B.sf_myMuxFun1_e.y[2]) > 0) {
    rtb_Switch3 = SLUGS2_B.sf_myMuxFun1_e.y[2];
  } else {
    rtb_Switch3 = SLUGS2_DWork.IntegerDelay3_DSTATE;
  }

  /* End of Switch: '<S66>/Switch3' */

  /* Product: '<S17>/Product1' */
  for (i = 0; i < 3; i++) {
    rtb_Product2_c[i] = rtb_MathFunction[i + 6] * SLUGS2_B.y_k[5] +
      (rtb_MathFunction[i + 3] * SLUGS2_B.y_k[4] + rtb_MathFunction[i] *
       SLUGS2_B.y_k[3]);
  }

  /* End of Product: '<S17>/Product1' */

  /* MATLAB Function: '<S17>/myMux Fun2' incorporates:
   *  MATLAB Function: '<S17>/myMux Fun1'
   */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun2': '<S703>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S703>:1:5' */
  SLUGS2_B.y_j[0] = rtb_y_jl_idx_0;
  SLUGS2_B.y_j[1] = rtb_y_jl_idx_1;
  SLUGS2_B.y_j[2] = SLUGS2_B.sf_EmbeddedMATLABFunction1.y;
  SLUGS2_B.y_j[3] = rtb_Product2_c[0];
  SLUGS2_B.y_j[4] = rtb_Product2_c[1];
  SLUGS2_B.y_j[5] = rtb_Product2_c[2];

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S159>/[0 1000]' */
  if (rtb_Switch3 > 1000.0F) {
    rtb_Sum1_mj = 1000.0F;
  } else if (rtb_Switch3 < 0.0F) {
    rtb_Sum1_mj = 0.0F;
  } else {
    rtb_Sum1_mj = rtb_Switch3;
  }

  /* Sum: '<S159>/Add' incorporates:
   *  Constant: '<S159>/Constant'
   *  Constant: '<S159>/Constant from Model'
   *  Gain: '<S164>/Unit Conversion'
   *  Product: '<S159>/Divide'
   *  Saturate: '<S159>/[0 1000]'
   */
  SLUGS2_B.Add = 1.0F - 3.28084F * rtb_Sum1_mj / 145442.0F;

  /* S-Function (MCHP_C_function_Call): '<S165>/myPow() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myPowapUtilscupdated5116 = myPow(
    SLUGS2_B.Add
    , 4.25587606F
    );

  /* Product: '<S159>/Divide1' incorporates:
   *  Constant: '<S159>/Rho_0 (Kg//m^3)'
   */
  rtb_Sum1_mj = SLUGS2_B.myPowapUtilscupdated5116 * 1.225F;

  /* Saturate: '<S160>/[ 0.01 50000]' */
  if (rtb_Sum1_mj > 50000.0F) {
    rtb_Sum1_mj = 50000.0F;
  } else {
    if (rtb_Sum1_mj < 0.01F) {
      rtb_Sum1_mj = 0.01F;
    }
  }

  /* MATLAB Function: '<S167>/negprotect' incorporates:
   *  Constant: '<S160>/a'
   *  DataStoreRead: '<Root>/Get mlAirData'
   *  Gain: '<Root>/Gain1'
   *  Product: '<S160>/Divide2'
   *  Saturate: '<S160>/[ 0.01 50000]'
   */
  SLUGS2_negprotect_l(2.0F * (100.0F * mlAirData.press_diff) / rtb_Sum1_mj,
                      &SLUGS2_B.sf_negprotect_l);

  /* S-Function (MCHP_C_function_Call): '<S167>/mySqrt() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.mySqrtapUtilscupdated5116_me = mySqrt(
    SLUGS2_B.sf_negprotect_l.zpVal
    );

  /* Switch: '<S161>/Switch3' incorporates:
   *  RelationalOperator: '<S161>/Relational Operator2'
   */
  if ((SLUGS2_B.mySqrtapUtilscupdated5116_me ==
       SLUGS2_B.mySqrtapUtilscupdated5116_me) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_g = SLUGS2_B.mySqrtapUtilscupdated5116_me;
  }

  /* End of Switch: '<S161>/Switch3' */

  /* MATLAB Function: '<S158>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S158>/Constant'
   *  Constant: '<S158>/Constant1'
   */
  SLU_EmbeddedMATLABFunction(SLUGS2_DWork.IntegerDelay3_DSTATE_g, 0.01, 1.0,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_n,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_n);

  /* Switch: '<S138>/Schedule LPF' incorporates:
   *  Constant: '<S162>/Constant'
   *  RelationalOperator: '<S162>/Compare'
   */
  if ((SLUGS2_DWork.IntegerDelay3_DSTATE_g <= 5.0F) > 0) {
    rtb_Ze_b = SLUGS2_DWork.IntegerDelay3_DSTATE_g;
  } else {
    rtb_Ze_b = SLUGS2_B.sf_EmbeddedMATLABFunction_n.y;
  }

  /* End of Switch: '<S138>/Schedule LPF' */

  /* Product: '<S143>/delta rise limit' incorporates:
   *  Constant: '<S133>/Constant3'
   *  SampleTimeMath: '<S143>/sample time'
   *
   * About '<S143>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_RhhcosphisinlambYe = 0.02F;

  /* Switch: '<S171>/Init' incorporates:
   *  UnitDelay: '<S171>/FixPt Unit Delay1'
   *  UnitDelay: '<S171>/FixPt Unit Delay2'
   */
  if (SLUGS2_DWork.FixPtUnitDelay2_DSTATE != 0) {
    rtb_Deg2R1 = 166.3F;
  } else {
    rtb_Deg2R1 = SLUGS2_DWork.FixPtUnitDelay1_DSTATE;
  }

  /* End of Switch: '<S171>/Init' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Switch: '<S4>/Switch' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  if (rtb_IC1_a) {
    rtb_Sum1_mj = mlMidLevelCommands.hCommand;
  } else {
	  rtb_Sum1_mj = SLUGS2_B.Merge1;
  }

  /* End of Switch: '<S4>/Switch' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Sum: '<S143>/Difference Inputs1'
   *
   * Block description for '<S143>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Sum1_mj -= rtb_Deg2R1;

  /* Switch: '<S170>/Switch2' incorporates:
   *  RelationalOperator: '<S170>/LowerRelop1'
   */
  if (!(rtb_Sum1_mj > 0.02F)) {
    /* Switch: '<S170>/Switch' incorporates:
     *  RelationalOperator: '<S170>/UpperRelop'
     */
    if (rtb_Sum1_mj < -0.03F) {
      rtb_RhhcosphisinlambYe = -0.03F;
    } else {
      rtb_RhhcosphisinlambYe = rtb_Sum1_mj;
    }

    /* End of Switch: '<S170>/Switch' */
  }

  /* End of Switch: '<S170>/Switch2' */

  /* Sum: '<S143>/Difference Inputs2'
   *
   * Block description for '<S143>/Difference Inputs2':
   *
   *  Add in CPU
   */
  SLUGS2_DWork.FixPtUnitDelay1_DSTATE = rtb_RhhcosphisinlambYe + rtb_Deg2R1;

  /* Sum: '<S133>/Add2' */
  rtb_Deg2R1 = SLUGS2_DWork.FixPtUnitDelay1_DSTATE - rtb_Switch3;

  /* Switch: '<S140>/Switch3' incorporates:
   *  RelationalOperator: '<S140>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_e = rtb_Deg2R1;
  }

  /* End of Switch: '<S140>/Switch3' */

  /* Product: '<S136>/Product1' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   */
  rtb_Sum1_mj = SLUGS2_DWork.IntegerDelay3_DSTATE_e * mlParamInterface.param[9];

  /* Sum: '<S136>/Sum2' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Gain: '<S136>/Gain'
   *  Memory: '<S136>/Memory1'
   *  Product: '<S136>/Product4'
   */
  rtb_Deg2R1 = (0.01F * SLUGS2_DWork.IntegerDelay3_DSTATE_e *
                mlParamInterface.param[10] + SLUGS2_DWork.Memory1_PreviousInput)
    + rtb_Sum1_mj;

  /* Switch: '<S136>/AntiWindup' incorporates:
   *  Constant: '<S136>/Constant5'
   *  Logic: '<S136>/Logical Operator'
   *  RelationalOperator: '<S136>/Relational Operator'
   *  RelationalOperator: '<S136>/Relational Operator1'
   */
  if ((rtb_Deg2R1 > -0.261799395F) && (rtb_Deg2R1 < 0.261799395F)) {
    rtb_Deg2R1 = SLUGS2_DWork.IntegerDelay3_DSTATE_e;
  } else {
    rtb_Deg2R1 = 0.0F;
  }

  /* End of Switch: '<S136>/AntiWindup' */

  /* Switch: '<S153>/Switch3' incorporates:
   *  RelationalOperator: '<S153>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_a = rtb_Deg2R1;
  }

  /* End of Switch: '<S153>/Switch3' */

  /* Switch: '<S151>/Switch1' incorporates:
   *  Constant: '<S151>/Constant'
   *  Constant: '<S151>/Constant1'
   *  Constant: '<S151>/Constant2'
   *  Constant: '<S151>/Constant3'
   *  Constant: '<S151>/Constant5'
   *  Delay: '<S151>/Integer Delay'
   *  Delay: '<S151>/Integer Delay1'
   *  Delay: '<S151>/Integer Delay2'
   *  Product: '<S151>/Product'
   *  Product: '<S151>/Product1'
   *  Product: '<S151>/Product2'
   *  Product: '<S151>/Product3'
   *  Sum: '<S151>/Subtract'
   *  Sum: '<S151>/Subtract1'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = ((SLUGS2_DWork.IntegerDelay3_DSTATE_a * 0.333333343F +
                   SLUGS2_DWork.IntegerDelay_DSTATE_a * 1.33333337F) +
                  SLUGS2_DWork.IntegerDelay1_DSTATE_h[0] * 0.333333343F) *
      0.005F + SLUGS2_DWork.IntegerDelay2_DSTATE;
  }

  /* End of Switch: '<S151>/Switch1' */

  /* Switch: '<S152>/Switch3' incorporates:
   *  RelationalOperator: '<S152>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_av = rtb_Deg2R1;
  }

  /* End of Switch: '<S152>/Switch3' */

  /* Switch: '<S136>/On//Off' incorporates:
   *  Constant: '<S136>/Constant1'
   *  DataStoreRead: '<S8>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Product: '<S136>/Product'
   *  Sum: '<S136>/Add2'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    SLUGS2_DWork.Memory1_PreviousInput = 0.0F;
  } else {
    SLUGS2_DWork.Memory1_PreviousInput = SLUGS2_DWork.IntegerDelay3_DSTATE_av *
      mlParamInterface.param[10] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S136>/On//Off' */

  /* Switch: '<S150>/Switch2' incorporates:
   *  RelationalOperator: '<S150>/LowerRelop1'
   *  RelationalOperator: '<S150>/UpperRelop'
   *  Switch: '<S150>/Switch'
   */
  if (SLUGS2_DWork.Memory1_PreviousInput > 0.261799395F) {
    /* Saturate: '<S133>/Theta_c Limit' */
    rtb_Deg2R1 = 0.261799395F;
  } else if (SLUGS2_DWork.Memory1_PreviousInput < -0.261799395F) {
    /* Switch: '<S150>/Switch' incorporates:
     *  Saturate: '<S133>/Theta_c Limit'
     */
    rtb_Deg2R1 = -0.261799395F;
  } else {
    /* Saturate: '<S133>/Theta_c Limit' incorporates:
     *  Switch: '<S150>/Switch'
     */
    rtb_Deg2R1 = SLUGS2_DWork.Memory1_PreviousInput;
  }

  /* End of Switch: '<S150>/Switch2' */

  /* DataStoreWrite: '<S64>/mlNavigation' */
  mlNavigation.u_m = rtb_Ze_b;
  mlNavigation.theta_c = rtb_Deg2R1;

  /* DataTypeConversion: '<S133>/Data Type Conversion' */
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_DWork.FixPtUnitDelay1_DSTATE);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  /* DataStoreWrite: '<S64>/mlNavigation' incorporates:
   *  DataTypeConversion: '<S133>/Data Type Conversion'
   */
//#ifdef WIN
//  mlNavigation.h_c = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)(int16_T)
//    (uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;
//#else
mlNavigation.h_c = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)-(int16_T)
    (uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;
//#endif

  /* Switch: '<S142>/Switch3' incorporates:
   *  RelationalOperator: '<S142>/Relational Operator2'
   */
  if ((SLUGS2_B.y_j[0] == SLUGS2_B.y_j[0]) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_c = SLUGS2_B.y_j[0];
  }

  /* End of Switch: '<S142>/Switch3' */

  /* MATLAB Function: '<S134>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S134>/Constant'
   *  Constant: '<S134>/Constant1'
   */
  SLU_EmbeddedMATLABFunction(SLUGS2_DWork.IntegerDelay3_DSTATE_c, 0.01, 0.32,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_i,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_i);

  /* Sum: '<S133>/Add' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  rtb_RhhcosphisinlambYe = mlMidLevelCommands.uCommand - rtb_Ze_b;

  /* Sum: '<S135>/Add3' incorporates:
   *  Constant: '<S135>/SaturationLimit'
   */
  rtb_Deg2R_h_idx_0 = 0.95F - SLUGS2_B.DataTypeConversion;

  /* Switch: '<S139>/Switch3' incorporates:
   *  RelationalOperator: '<S139>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe == rtb_RhhcosphisinlambYe) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_j = rtb_RhhcosphisinlambYe;
  }

  /* End of Switch: '<S139>/Switch3' */

  /* Sum: '<S135>/Add1' incorporates:
   *  Constant: '<S135>/delayTime'
   *  DataStoreRead: '<S8>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Delay: '<S135>/NDelays'
   *  Product: '<S135>/Product1'
   *  Product: '<S135>/Product2'
   *  Product: '<S135>/Product3'
   *  Sum: '<S135>/Sum'
   */
  rtb_Sum1_mj = (SLUGS2_DWork.IntegerDelay3_DSTATE_j -
                 SLUGS2_DWork.NDelays_DSTATE[0]) / 0.05F *
    mlParamInterface.param[2] + SLUGS2_DWork.IntegerDelay3_DSTATE_j *
    mlParamInterface.param[0];

  /* Sum: '<S135>/Sum2' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Gain: '<S135>/Gain'
   *  Memory: '<S135>/Memory1'
   *  Product: '<S135>/Product4'
   */
  rtb_IC4_idx_1 = (0.01F * SLUGS2_DWork.IntegerDelay3_DSTATE_j *
                   mlParamInterface.param[1] +
                   SLUGS2_DWork.Memory1_PreviousInput_p) + rtb_Sum1_mj;

  /* Switch: '<S135>/AntiWindup' incorporates:
   *  Constant: '<S135>/Constant5'
   *  Constant: '<S135>/SaturationLimit'
   *  Constant: '<S135>/SaturationLimit1'
   *  Logic: '<S135>/Logical Operator'
   *  RelationalOperator: '<S135>/Relational Operator'
   *  RelationalOperator: '<S135>/Relational Operator1'
   *  Sum: '<S135>/Add3'
   *  Sum: '<S135>/Add4'
   */
  if ((rtb_IC4_idx_1 > 0.0F - SLUGS2_B.DataTypeConversion) && (rtb_IC4_idx_1 <
       0.95F - SLUGS2_B.DataTypeConversion)) {
    rtb_IC4_idx_1 = SLUGS2_DWork.IntegerDelay3_DSTATE_j;
  } else {
    rtb_IC4_idx_1 = 0.0F;
  }

  /* End of Switch: '<S135>/AntiWindup' */

  /* Switch: '<S148>/Switch3' incorporates:
   *  RelationalOperator: '<S148>/Relational Operator2'
   */
  if ((rtb_IC4_idx_1 == rtb_IC4_idx_1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_cl = rtb_IC4_idx_1;
  }

  /* End of Switch: '<S148>/Switch3' */

  /* Switch: '<S147>/Switch1' incorporates:
   *  Constant: '<S147>/Constant'
   *  Constant: '<S147>/Constant1'
   *  Constant: '<S147>/Constant2'
   *  Constant: '<S147>/Constant3'
   *  Constant: '<S147>/Constant5'
   *  Delay: '<S147>/Integer Delay'
   *  Delay: '<S147>/Integer Delay1'
   *  Delay: '<S147>/Integer Delay2'
   *  Product: '<S147>/Product'
   *  Product: '<S147>/Product1'
   *  Product: '<S147>/Product2'
   *  Product: '<S147>/Product3'
   *  Sum: '<S147>/Subtract'
   *  Sum: '<S147>/Subtract1'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_IC4_idx_1 = 0.0F;
  } else {
    rtb_IC4_idx_1 = ((SLUGS2_DWork.IntegerDelay3_DSTATE_cl * 0.333333343F +
                      SLUGS2_DWork.IntegerDelay_DSTATE_l * 1.33333337F) +
                     SLUGS2_DWork.IntegerDelay1_DSTATE_hy[0] * 0.333333343F) *
      0.005F + SLUGS2_DWork.IntegerDelay2_DSTATE_f;
  }

  /* End of Switch: '<S147>/Switch1' */

  /* Switch: '<S149>/Switch3' incorporates:
   *  RelationalOperator: '<S149>/Relational Operator2'
   */
  if ((rtb_IC4_idx_1 == rtb_IC4_idx_1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_m = rtb_IC4_idx_1;
  }

  /* End of Switch: '<S149>/Switch3' */

  /* Switch: '<S135>/On//Off' incorporates:
   *  Constant: '<S135>/Constant1'
   *  DataStoreRead: '<S8>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Product: '<S135>/Product'
   *  Sum: '<S135>/Add2'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    SLUGS2_DWork.Memory1_PreviousInput_p = 0.0F;
  } else {
    SLUGS2_DWork.Memory1_PreviousInput_p = SLUGS2_DWork.IntegerDelay3_DSTATE_m *
      mlParamInterface.param[1] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S135>/On//Off' */

  /* Switch: '<S146>/Switch2' incorporates:
   *  Constant: '<S135>/SaturationLimit'
   *  RelationalOperator: '<S146>/LowerRelop1'
   *  Sum: '<S135>/Add3'
   */
  if (!(SLUGS2_DWork.Memory1_PreviousInput_p > 0.95F -
        SLUGS2_B.DataTypeConversion)) {
    /* Switch: '<S146>/Switch' incorporates:
     *  Constant: '<S135>/SaturationLimit1'
     *  RelationalOperator: '<S146>/UpperRelop'
     *  Sum: '<S135>/Add4'
     */
    if (SLUGS2_DWork.Memory1_PreviousInput_p < 0.0F -
        SLUGS2_B.DataTypeConversion) {
      rtb_Deg2R_h_idx_0 = 0.0F - SLUGS2_B.DataTypeConversion;
    } else {
      rtb_Deg2R_h_idx_0 = SLUGS2_DWork.Memory1_PreviousInput_p;
    }

    /* End of Switch: '<S146>/Switch' */
  }

  /* End of Switch: '<S146>/Switch2' */

  /* Sum: '<S133>/Add1' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_PITC_DT_FF'
   *  Product: '<S133>/Product2'
   */
  rtb_Deg2R_h_idx_0 = (rtb_Deg2R_h_idx_0 + SLUGS2_B.DataTypeConversion) +
    mlParamInterface.param[15] * SLUGS2_B.y_j[1];

  /* Sum: '<S133>/Add3' */
  rtb_Deg2R1 -= SLUGS2_B.y_j[1];

  /* Sum: '<S137>/Add3' incorporates:
   *  Constant: '<S137>/SaturationLimit'
   */
  rtb_IC4_idx_1 = 0.401425719F - SLUGS2_B.DataTypeConversion_f;

  /* Switch: '<S141>/Switch3' incorporates:
   *  RelationalOperator: '<S141>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_b = rtb_Deg2R1;
  }

  /* End of Switch: '<S141>/Switch3' */

  /* Sum: '<S137>/Add1' incorporates:
   *  Constant: '<S137>/delayTime'
   *  DataStoreRead: '<S8>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Delay: '<S137>/NDelays'
   *  Product: '<S137>/Product1'
   *  Product: '<S137>/Product2'
   *  Product: '<S137>/Product3'
   *  Sum: '<S137>/Sum'
   */
  rtb_Sum1_mj = (SLUGS2_DWork.IntegerDelay3_DSTATE_b -
                 SLUGS2_DWork.NDelays_DSTATE_c[0]) / 0.05F *
    mlParamInterface.param[5] + SLUGS2_DWork.IntegerDelay3_DSTATE_b *
    mlParamInterface.param[3];

  /* Sum: '<S137>/Sum2' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Gain: '<S137>/Gain'
   *  Memory: '<S137>/Memory1'
   *  Product: '<S137>/Product4'
   */
  rtb_Deg2R1 = (0.01F * SLUGS2_DWork.IntegerDelay3_DSTATE_b *
                mlParamInterface.param[4] + SLUGS2_DWork.Memory1_PreviousInput_o)
    + rtb_Sum1_mj;

  /* Switch: '<S137>/AntiWindup' incorporates:
   *  Constant: '<S137>/Constant5'
   *  Constant: '<S137>/SaturationLimit'
   *  Constant: '<S137>/SaturationLimit1'
   *  Logic: '<S137>/Logical Operator'
   *  RelationalOperator: '<S137>/Relational Operator'
   *  RelationalOperator: '<S137>/Relational Operator1'
   *  Sum: '<S137>/Add3'
   *  Sum: '<S137>/Add4'
   */
  if ((rtb_Deg2R1 > -0.401425719F - SLUGS2_B.DataTypeConversion_f) &&
      (rtb_Deg2R1 < 0.401425719F - SLUGS2_B.DataTypeConversion_f)) {
    rtb_Deg2R1 = SLUGS2_DWork.IntegerDelay3_DSTATE_b;
  } else {
    rtb_Deg2R1 = 0.0F;
  }

  /* End of Switch: '<S137>/AntiWindup' */

  /* Switch: '<S156>/Switch3' incorporates:
   *  RelationalOperator: '<S156>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_o = rtb_Deg2R1;
  }

  /* End of Switch: '<S156>/Switch3' */

  /* Switch: '<S155>/Switch1' incorporates:
   *  Constant: '<S155>/Constant'
   *  Constant: '<S155>/Constant1'
   *  Constant: '<S155>/Constant2'
   *  Constant: '<S155>/Constant3'
   *  Constant: '<S155>/Constant5'
   *  Delay: '<S155>/Integer Delay'
   *  Delay: '<S155>/Integer Delay1'
   *  Delay: '<S155>/Integer Delay2'
   *  Product: '<S155>/Product'
   *  Product: '<S155>/Product1'
   *  Product: '<S155>/Product2'
   *  Product: '<S155>/Product3'
   *  Sum: '<S155>/Subtract'
   *  Sum: '<S155>/Subtract1'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = ((SLUGS2_DWork.IntegerDelay3_DSTATE_o * 0.333333343F +
                   SLUGS2_DWork.IntegerDelay_DSTATE_e * 1.33333337F) +
                  SLUGS2_DWork.IntegerDelay1_DSTATE_k[0] * 0.333333343F) *
      0.005F + SLUGS2_DWork.IntegerDelay2_DSTATE_b;
  }

  /* End of Switch: '<S155>/Switch1' */

  /* Switch: '<S157>/Switch3' incorporates:
   *  RelationalOperator: '<S157>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_ep = rtb_Deg2R1;
  }

  /* End of Switch: '<S157>/Switch3' */

  /* Switch: '<S137>/On//Off' incorporates:
   *  Constant: '<S137>/Constant1'
   *  DataStoreRead: '<S8>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Product: '<S137>/Product'
   *  Sum: '<S137>/Add2'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    SLUGS2_DWork.Memory1_PreviousInput_o = 0.0F;
  } else {
    SLUGS2_DWork.Memory1_PreviousInput_o = SLUGS2_DWork.IntegerDelay3_DSTATE_ep *
      mlParamInterface.param[4] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S137>/On//Off' */

  /* Switch: '<S154>/Switch2' incorporates:
   *  Constant: '<S137>/SaturationLimit'
   *  RelationalOperator: '<S154>/LowerRelop1'
   *  Sum: '<S137>/Add3'
   */
  if (!(SLUGS2_DWork.Memory1_PreviousInput_o > 0.401425719F -
        SLUGS2_B.DataTypeConversion_f)) {
    /* Switch: '<S154>/Switch' incorporates:
     *  Constant: '<S137>/SaturationLimit1'
     *  RelationalOperator: '<S154>/UpperRelop'
     *  Sum: '<S137>/Add4'
     */
    if (SLUGS2_DWork.Memory1_PreviousInput_o < -0.401425719F -
        SLUGS2_B.DataTypeConversion_f) {
      rtb_IC4_idx_1 = -0.401425719F - SLUGS2_B.DataTypeConversion_f;
    } else {
      rtb_IC4_idx_1 = SLUGS2_DWork.Memory1_PreviousInput_o;
    }

    /* End of Switch: '<S154>/Switch' */
  }

  /* End of Switch: '<S154>/Switch2' */

  /* Saturate: '<S133>/[-60 60]' */
  if (SLUGS2_B.sf_EmbeddedMATLABFunction_i.y > 60.0F) {
    SLUGS2_B.u060 = 60.0F;
  } else if (SLUGS2_B.sf_EmbeddedMATLABFunction_i.y < -60.0F) {
    SLUGS2_B.u060 = -60.0F;
  } else {
    SLUGS2_B.u060 = SLUGS2_B.sf_EmbeddedMATLABFunction_i.y;
  }

  /* End of Saturate: '<S133>/[-60 60]' */

  /* S-Function (MCHP_C_function_Call): '<S144>/myCos() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myCosapUtilscupdated5116 = myCos(
    SLUGS2_B.u060
    );

  /* Sum: '<S133>/Add4' incorporates:
   *  Constant: '<S133>/Constant2'
   *  Constant: '<S133>/Constant4'
   *  DataStoreRead: '<S8>/PAR_PID_HEI_ERR_FF'
   *  Product: '<S133>/Product'
   *  Product: '<S133>/Product1'
   *  Sum: '<S133>/Add5'
   */
  rtb_IC4_idx_1 = (1.0F / SLUGS2_B.myCosapUtilscupdated5116 - 1.0F) *
    mlParamInterface.param[11] + (rtb_IC4_idx_1 + SLUGS2_B.DataTypeConversion_f);

  /* Update for UnitDelay: '<S171>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S171>/FixPt Constant'
   */
  SLUGS2_DWork.FixPtUnitDelay2_DSTATE = 0U;

  /* Update for Delay: '<S151>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE = SLUGS2_DWork.IntegerDelay3_DSTATE_av;

  /* Update for Delay: '<S151>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_a = SLUGS2_DWork.IntegerDelay3_DSTATE_a;

  /* Update for Delay: '<S151>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_h[0] = SLUGS2_DWork.IntegerDelay1_DSTATE_h[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_h[1] = SLUGS2_DWork.IntegerDelay3_DSTATE_a;

  /* Update for Delay: '<S135>/NDelays' */
  SLUGS2_DWork.NDelays_DSTATE[0] = SLUGS2_DWork.NDelays_DSTATE[1];
  SLUGS2_DWork.NDelays_DSTATE[1] = SLUGS2_DWork.NDelays_DSTATE[2];
  SLUGS2_DWork.NDelays_DSTATE[2] = SLUGS2_DWork.NDelays_DSTATE[3];
  SLUGS2_DWork.NDelays_DSTATE[3] = SLUGS2_DWork.NDelays_DSTATE[4];
  SLUGS2_DWork.NDelays_DSTATE[4] = SLUGS2_DWork.IntegerDelay3_DSTATE_j;

  /* Update for Delay: '<S147>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE_f = SLUGS2_DWork.IntegerDelay3_DSTATE_m;

  /* Update for Delay: '<S147>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_l = SLUGS2_DWork.IntegerDelay3_DSTATE_cl;

  /* Update for Delay: '<S147>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_hy[0] =
    SLUGS2_DWork.IntegerDelay1_DSTATE_hy[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_hy[1] = SLUGS2_DWork.IntegerDelay3_DSTATE_cl;

  /* Update for Delay: '<S137>/NDelays' */
  SLUGS2_DWork.NDelays_DSTATE_c[0] = SLUGS2_DWork.NDelays_DSTATE_c[1];
  SLUGS2_DWork.NDelays_DSTATE_c[1] = SLUGS2_DWork.NDelays_DSTATE_c[2];
  SLUGS2_DWork.NDelays_DSTATE_c[2] = SLUGS2_DWork.NDelays_DSTATE_c[3];
  SLUGS2_DWork.NDelays_DSTATE_c[3] = SLUGS2_DWork.NDelays_DSTATE_c[4];
  SLUGS2_DWork.NDelays_DSTATE_c[4] = SLUGS2_DWork.IntegerDelay3_DSTATE_b;

  /* Update for Delay: '<S155>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE_b = SLUGS2_DWork.IntegerDelay3_DSTATE_ep;

  /* Update for Delay: '<S155>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_e = SLUGS2_DWork.IntegerDelay3_DSTATE_o;

  /* Update for Delay: '<S155>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_k[0] = SLUGS2_DWork.IntegerDelay1_DSTATE_k[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_k[1] = SLUGS2_DWork.IntegerDelay3_DSTATE_o;

  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* S-Function (MCHP_C_function_Call): '<S68>/myCos() aLib.c [updated 5.1.16]' */
  SLUGS2_B.myCosaLibcupdated5116 = myCos(
    SLUGS2_B.y_j[0]
    );

  /* Product: '<S61>/Product' */
  rtb_Sum1_mj = SLUGS2_B.y_j[5] * SLUGS2_B.myCosaLibcupdated5116;

  /* S-Function (MCHP_C_function_Call): '<S69>/C Function Call1' */
  SLUGS2_B.CFunctionCall1 = mySin(
    SLUGS2_B.y_j[0]
    );

  /* Sum: '<S61>/Subtract' incorporates:
   *  Product: '<S61>/Product1'
   */
  rtb_Product = SLUGS2_B.y_j[4] * SLUGS2_B.CFunctionCall1 + rtb_Sum1_mj;

  /* Switch: '<S67>/Switch3' incorporates:
   *  Delay: '<S67>/Integer Delay3'
   *  RelationalOperator: '<S67>/Relational Operator2'
   */
  if (!((rtb_Product == rtb_Product) > 0)) {
    rtb_Product = SLUGS2_DWork.IntegerDelay3_DSTATE_k;
  }

  /* End of Switch: '<S67>/Switch3' */

  /* DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF' incorporates:
   *  Inport: '<S85>/In1'
   *  Logic: '<S73>/Logical Operator'
   */
  rtb_IC4_idx_0 = mlParamInterface.param[22];
  rtb_y_jl_idx_0 = mlParamInterface.param[23];

  /* Outputs for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  EnablePort: '<S62>/Enable'
   */
  if (mlParamInterface.param[25L] > 0.0F) {
    if (!SLUGS2_DWork.L1OutputFeedbackControllerWithP) {
      /* InitializeConditions for UnitDelay: '<S82>/UD'
       *
       * Block description for '<S82>/UD':
       *
       *  Store in Global RAM
       */
      SLUGS2_DWork.UD_DSTATE_ff = 0.0F;

      /* InitializeConditions for Delay: '<S72>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_an = 0.0F;

      /* InitializeConditions for Delay: '<S72>/Integer Delay1' */
      SLUGS2_DWork.IntegerDelay1_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S74>/Integer Delay3' */
      SLUGS2_DWork.IntegerDelay3_DSTATE_fs = 0.0F;

      /* InitializeConditions for UnitDelay: '<S92>/UD'
       *
       * Block description for '<S92>/UD':
       *
       *  Store in Global RAM
       */
      SLUGS2_DWork.UD_DSTATE_j = 0.0F;

      /* InitializeConditions for Delay: '<S76>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_k = 0.0F;

      /* InitializeConditions for Delay: '<S76>/Integer Delay1' */
      SLUGS2_DWork.IntegerDelay1_DSTATE_b = 0.0F;

      /* InitializeConditions for Delay: '<S62>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S62>/Integer Delay1' */
      SLUGS2_DWork.IntegerDelay1_DSTATE_e = 0.0F;

      /* InitializeConditions for Merge: '<S73>/Merge' */
      if (rtmIsFirstInitCond(SLUGS2_M)) {
        SLUGS2_B.Merge_oo = 0.0F;
      }

      /* End of InitializeConditions for Merge: '<S73>/Merge' */

      /* InitializeConditions for Delay: '<S75>/Integer Delay3' */
      SLUGS2_DWork.IntegerDelay3_DSTATE_h = 0.0F;

      /* InitializeConditions for Delay: '<S77>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_p = 0.0F;

      /* InitializeConditions for Delay: '<S77>/Integer Delay1' */
      SLUGS2_DWork.IntegerDelay1_DSTATE_ey[0] = 0.0F;
      SLUGS2_DWork.IntegerDelay1_DSTATE_ey[1] = 0.0F;

      /* InitializeConditions for Delay: '<S77>/Integer Delay2' */
      SLUGS2_DWork.IntegerDelay2_DSTATE_g = 0.0F;

      /* InitializeConditions for Delay: '<S93>/Integer Delay3' */
      SLUGS2_DWork.IntegerDelay3_DSTATE_nz = 0.0F;

      /* InitializeConditions for Delay: '<S94>/Integer Delay3' */
      SLUGS2_DWork.IntegerDelay3_DSTATE_hh = 0.0F;
      SLUGS2_DWork.L1OutputFeedbackControllerWithP = true;
    }

    /* InitialCondition: '<S72>/IC' */
    if (SLUGS2_DWork.IC_FirstOutputTime_g) {
      SLUGS2_DWork.IC_FirstOutputTime_g = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S79>/Abs1' incorporates:
       *  Sum: '<S82>/Diff'
       *  UnitDelay: '<S82>/UD'
       *
       * Block description for '<S82>/Diff':
       *
       *  Add in CPU
       *
       * Block description for '<S82>/UD':
       *
       *  Store in Global RAM
       */
      rtb_Sum1_mj = (real32_T)fabs(mlParamInterface.param[22] -
        SLUGS2_DWork.UD_DSTATE_ff);

      /* Saturate: '<S79>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S79>/Saturation1' */
    }

    /* End of InitialCondition: '<S72>/IC' */

    /* Outputs for Enabled SubSystem: '<S72>/Compute Coef' incorporates:
     *  EnablePort: '<S78>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Gain: '<S78>/-T' */
      SLUGS2_B.T_m = -0.01F * mlParamInterface.param[22];

      /* S-Function (MCHP_C_function_Call): '<S80>/myExp() apUtils.c [updated 5.1.16]' */
      SLUGS2_B.myExpapUtilscupdated5116_f = myExp(
        SLUGS2_B.T_m
        );

      /* SignalConversion: '<S81>/Numerical Unity' */
      SLUGS2_B.NumericalUnity_a = SLUGS2_B.myExpapUtilscupdated5116_f;

      /* Sum: '<S78>/1-c' incorporates:
       *  Constant: '<S78>/Constant'
       */
      SLUGS2_B.c_d = (real32_T)(1.0 - SLUGS2_B.NumericalUnity_a);
    }

    /* End of Outputs for SubSystem: '<S72>/Compute Coef' */

    /* Sum: '<S72>/Subtract' incorporates:
     *  Delay: '<S72>/Integer Delay'
     *  Delay: '<S72>/Integer Delay1'
     *  Product: '<S72>/Divide'
     *  Product: '<S72>/Divide1'
     */
    rtb_Subtract_od = SLUGS2_B.c_d * SLUGS2_DWork.IntegerDelay_DSTATE_an +
      SLUGS2_B.NumericalUnity_a * SLUGS2_DWork.IntegerDelay1_DSTATE_i;

    /* Switch: '<S74>/Switch3' incorporates:
     *  Delay: '<S74>/Integer Delay3'
     *  RelationalOperator: '<S74>/Relational Operator2'
     */
    if ((rtb_Product == rtb_Product) > 0) {
      rtb_Switch3_gd = rtb_Product;
    } else {
      rtb_Switch3_gd = SLUGS2_DWork.IntegerDelay3_DSTATE_fs;
    }

    /* End of Switch: '<S74>/Switch3' */

    /* InitialCondition: '<S76>/IC' */
    if (SLUGS2_DWork.IC_FirstOutputTime_f) {
      SLUGS2_DWork.IC_FirstOutputTime_f = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S89>/Abs1' incorporates:
       *  Sum: '<S92>/Diff'
       *  UnitDelay: '<S92>/UD'
       *
       * Block description for '<S92>/Diff':
       *
       *  Add in CPU
       *
       * Block description for '<S92>/UD':
       *
       *  Store in Global RAM
       */
      rtb_Sum1_mj = (real32_T)fabs(mlParamInterface.param[23] -
        SLUGS2_DWork.UD_DSTATE_j);

      /* Saturate: '<S89>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S89>/Saturation1' */
    }

    /* End of InitialCondition: '<S76>/IC' */

    /* Outputs for Enabled SubSystem: '<S76>/Compute Coef' incorporates:
     *  EnablePort: '<S88>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Gain: '<S88>/-T' */
      SLUGS2_B.T = -0.01F * mlParamInterface.param[23];

      /* S-Function (MCHP_C_function_Call): '<S90>/myExp() apUtils.c [updated 5.1.16]' */
      SLUGS2_B.myExpapUtilscupdated5116 = myExp(
        SLUGS2_B.T
        );

      /* SignalConversion: '<S91>/Numerical Unity' */
      SLUGS2_B.NumericalUnity_n = SLUGS2_B.myExpapUtilscupdated5116;

      /* Sum: '<S88>/1-c' incorporates:
       *  Constant: '<S88>/Constant'
       */
      SLUGS2_B.c_h = (real32_T)(1.0 - SLUGS2_B.NumericalUnity_n);
    }

    /* End of Outputs for SubSystem: '<S76>/Compute Coef' */

    /* Sum: '<S76>/Subtract' incorporates:
     *  Delay: '<S76>/Integer Delay'
     *  Delay: '<S76>/Integer Delay1'
     *  Product: '<S76>/Divide'
     *  Product: '<S76>/Divide1'
     */
    rtb_Subtract_a = SLUGS2_B.c_h * SLUGS2_DWork.IntegerDelay_DSTATE_k +
      SLUGS2_B.NumericalUnity_n * SLUGS2_DWork.IntegerDelay1_DSTATE_b;

    /* Gain: '<S62>/Gain' incorporates:
     *  Sum: '<S62>/Sum3'
     */
    rtb_Sum1_mj = -(rtb_Subtract_a - rtb_Switch3_gd);

    /* Product: '<S87>/Divide4' incorporates:
     *  Constant: '<S87>/Constant'
     *  Delay: '<S62>/Integer Delay1'
     */
    rtb_RhhcosphisinlambYe = 0.25F * SLUGS2_DWork.IntegerDelay1_DSTATE_e * 2.0F;

    /* Product: '<S87>/Divide3' incorporates:
     *  Delay: '<S62>/Integer Delay1'
     *  Product: '<S87>/Divide'
     *  Sum: '<S87>/Subtract'
     */
    rtb_Deg2R1 = (SLUGS2_DWork.IntegerDelay1_DSTATE_e *
                  SLUGS2_DWork.IntegerDelay1_DSTATE_e - 4.0F) / 4.0F;

    /* Logic: '<S73>/Logical Operator2' incorporates:
     *  Constant: '<S83>/Constant'
     *  Constant: '<S84>/Constant'
     *  Product: '<S73>/Divide2'
     *  RelationalOperator: '<S83>/Compare'
     *  RelationalOperator: '<S84>/Compare'
     */
    rtb_Compare_j = (uint8_T)((rtb_RhhcosphisinlambYe * rtb_Sum1_mj > 0.0F) &&
      (rtb_Deg2R1 >= 0.0F));

    /* Outputs for Enabled SubSystem: '<S73>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S85>/Enable'
     */
    if (!(rtb_Compare_j != 0) > 0) {
      SLUGS2_B.Merge_oo = rtb_Sum1_mj;
    }

    /* End of Outputs for SubSystem: '<S73>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S73>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S86>/Enable'
     */
    if (rtb_Compare_j > 0) {
      /* Signum: '<S86>/Sign' */
      if (rtb_RhhcosphisinlambYe < 0.0F) {
        rtb_RhhcosphisinlambYe = -1.0F;
      } else if (rtb_RhhcosphisinlambYe > 0.0F) {
        rtb_RhhcosphisinlambYe = 1.0F;
      } else {
        if (rtb_RhhcosphisinlambYe == 0.0F) {
          rtb_RhhcosphisinlambYe = 0.0F;
        }
      }

      /* Product: '<S86>/Divide2' incorporates:
       *  Constant: '<S86>/Constant'
       *  Product: '<S86>/Divide1'
       *  Signum: '<S86>/Sign'
       *  Sum: '<S86>/Subtract'
       */
      SLUGS2_B.Merge_oo = (1.0F - rtb_Deg2R1 * rtb_RhhcosphisinlambYe) *
        rtb_Sum1_mj;
    }

    /* End of Outputs for SubSystem: '<S73>/Enabled Subsystem1' */

    /* Product: '<S62>/Projection' incorporates:
     *  Inport: '<S85>/In1'
     *  Logic: '<S73>/Logical Operator'
     */
    rtb_Projection = mlParamInterface.param[24] * SLUGS2_B.Merge_oo;

    /* Sum: '<S62>/Sum4' incorporates:
     *  Delay: '<S62>/Integer Delay'
     */
    SLUGS2_B.PsiDotLimit = SLUGS2_DWork.IntegerDelay_DSTATE_i - rtb_Subtract_od;

    /* Saturate: '<S62>/Psi Dot  Limit' */
    if (SLUGS2_B.PsiDotLimit > 1.0F) {
      /* Sum: '<S62>/Sum4' */
      SLUGS2_B.PsiDotLimit = 1.0F;
    } else {
      if (SLUGS2_B.PsiDotLimit < -1.0F) {
        /* Sum: '<S62>/Sum4' */
        SLUGS2_B.PsiDotLimit = -1.0F;
      }
    }

    /* End of Saturate: '<S62>/Psi Dot  Limit' */

    /* Sum: '<S62>/Sum2' incorporates:
     *  Delay: '<S62>/Integer Delay1'
     */
    rtb_Switch1_a = SLUGS2_B.PsiDotLimit + SLUGS2_DWork.IntegerDelay1_DSTATE_e;

    /* Switch: '<S75>/Switch3' incorporates:
     *  Delay: '<S75>/Integer Delay3'
     *  RelationalOperator: '<S75>/Relational Operator2'
     */
    if ((rtb_Switch1_a == rtb_Switch1_a) > 0) {
      rtb_Switch3_kh = rtb_Switch1_a;
    } else {
      rtb_Switch3_kh = SLUGS2_DWork.IntegerDelay3_DSTATE_h;
    }

    /* End of Switch: '<S75>/Switch3' */

    /* Switch: '<S93>/Switch3' incorporates:
     *  Delay: '<S93>/Integer Delay3'
     *  RelationalOperator: '<S93>/Relational Operator2'
     */
    if (!((rtb_Projection == rtb_Projection) > 0)) {
      rtb_Projection = SLUGS2_DWork.IntegerDelay3_DSTATE_nz;
    }

    /* End of Switch: '<S93>/Switch3' */

    /* Switch: '<S77>/Switch1' incorporates:
     *  Constant: '<S77>/Constant5'
     *  Logic: '<S62>/Logical Operator'
     */
    if (!(mlParamInterface.param[25] != 0.0F) > 0) {
      rtb_Switch1_a = 0.0F;
    } else {
      /* Sum: '<S77>/Subtract1' incorporates:
       *  Constant: '<S77>/Constant'
       *  Constant: '<S77>/Constant1'
       *  Constant: '<S77>/Constant2'
       *  Constant: '<S77>/Constant3'
       *  Delay: '<S77>/Integer Delay'
       *  Delay: '<S77>/Integer Delay1'
       *  Delay: '<S77>/Integer Delay2'
       *  Product: '<S77>/Product'
       *  Product: '<S77>/Product1'
       *  Product: '<S77>/Product2'
       *  Product: '<S77>/Product3'
       *  Sum: '<S77>/Subtract'
       */
      rtb_Switch1_a = ((rtb_Projection * 0.333333343F +
                        SLUGS2_DWork.IntegerDelay_DSTATE_p * 1.33333337F) +
                       SLUGS2_DWork.IntegerDelay1_DSTATE_ey[0] * 0.333333343F) *
        0.005F + SLUGS2_DWork.IntegerDelay2_DSTATE_g;

      /* Saturate: '<S77>/[min max]' */
      if (rtb_Switch1_a > 2.0F) {
        rtb_Switch1_a = 2.0F;
      } else {
        if (rtb_Switch1_a < -2.0F) {
          rtb_Switch1_a = -2.0F;
        }
      }

      /* End of Saturate: '<S77>/[min max]' */
    }

    /* End of Switch: '<S77>/Switch1' */

    /* Switch: '<S94>/Switch3' incorporates:
     *  Delay: '<S94>/Integer Delay3'
     *  RelationalOperator: '<S94>/Relational Operator2'
     */
    if (!((rtb_Switch1_a == rtb_Switch1_a) > 0)) {
      rtb_Switch1_a = SLUGS2_DWork.IntegerDelay3_DSTATE_hh;
    }

    /* End of Switch: '<S94>/Switch3' */
  } else {
    if (SLUGS2_DWork.L1OutputFeedbackControllerWithP) {
      SLUGS2_DWork.L1OutputFeedbackControllerWithP = false;
    }
  }

  /* End of Outputs for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */

  /* Product: '<S17>/Product2' incorporates:
   *  Sum: '<S17>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_P32[i] = rtb_MathFunction[i + 6] * SLUGS2_B.DataTypeConversion3[2] +
      (rtb_MathFunction[i + 3] * SLUGS2_B.DataTypeConversion3[1] +
       rtb_MathFunction[i] * SLUGS2_B.DataTypeConversion3[0]);
  }

  /* End of Product: '<S17>/Product2' */

  /* Product: '<S17>/Product3' incorporates:
   *  Sum: '<S17>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_Merge_m[i] = rtb_MathFunction[i + 6] * SLUGS2_B.DataTypeConversion1[2] +
      (rtb_MathFunction[i + 3] * SLUGS2_B.DataTypeConversion1[1] +
       rtb_MathFunction[i] * SLUGS2_B.DataTypeConversion1[0]);
  }

  /* End of Product: '<S17>/Product3' */

  /* Sum: '<S17>/Sum' */
  rtb_Product3_j0[1] = rtb_P32[1] + rtb_Merge_m[1];

  /* Outputs for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Switch: '<S129>/Switch3' incorporates:
   *  RelationalOperator: '<S129>/Relational Operator2'
   */
  if ((rtb_cosphi == rtb_cosphi) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_mw = rtb_cosphi;
  }

  /* End of Switch: '<S129>/Switch3' */

  /* Saturate: '<S128>/bank Limit' */
  if (SLUGS2_DWork.IntegerDelay3_DSTATE_mw > 0.436332315F) {
    SLUGS2_B.bankLimit = 0.436332315F;
  } else if (SLUGS2_DWork.IntegerDelay3_DSTATE_mw < -0.436332315F) {
    SLUGS2_B.bankLimit = -0.436332315F;
  } else {
    SLUGS2_B.bankLimit = SLUGS2_DWork.IntegerDelay3_DSTATE_mw;
  }

  /* End of Saturate: '<S128>/bank Limit' */

  /* S-Function (MCHP_C_function_Call): '<S131>/myTan() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myTanapUtilscupdated5116 = myTan(
    SLUGS2_B.bankLimit
    );

  /* Switch: '<S130>/Switch3' incorporates:
   *  RelationalOperator: '<S130>/Relational Operator2'
   */
  if ((rtb_Ze_b == rtb_Ze_b) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_bs = rtb_Ze_b;
  }

  /* End of Switch: '<S130>/Switch3' */

  /* Switch: '<S96>/Switch1' incorporates:
   *  Constant: '<S128>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S128>/Divide'
   *  Product: '<S128>/Divide1'
   *  Saturate: '<S128>/[0 40]'
   */
  if (rtb_IC1_a) {
    rtb_Product3_bo = mlMidLevelCommands.rCommand;
  } else {
    if (SLUGS2_DWork.IntegerDelay3_DSTATE_bs > 40.0F) {
      /* Saturate: '<S128>/[0 40]' */
      rtb_Sum1_mj = 40.0F;
    } else if (SLUGS2_DWork.IntegerDelay3_DSTATE_bs < 0.0F) {
      /* Saturate: '<S128>/[0 40]' */
      rtb_Sum1_mj = 0.0F;
    } else {
      /* Saturate: '<S128>/[0 40]' */
      rtb_Sum1_mj = SLUGS2_DWork.IntegerDelay3_DSTATE_bs;
    }

    rtb_Product3_bo = 1.0F / rtb_Sum1_mj * 9.80665F *
      SLUGS2_B.myTanapUtilscupdated5116;
  }

  /* End of Switch: '<S96>/Switch1' */

  /* Switch: '<S96>/Switch' incorporates:
   *  DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF'
   */
  if (mlParamInterface.param[25] > 0.3F) {
    rtb_RhhcosphisinlambYe = SLUGS2_B.PsiDotLimit;
  } else {
    rtb_RhhcosphisinlambYe = rtb_Product3_bo;
  }

  /* End of Switch: '<S96>/Switch' */

  /* Saturate: '<S95>/Psi Dot  Limit' */
  if (rtb_RhhcosphisinlambYe > 1.0F) {
    rtb_RhhcosphisinlambYe = 1.0F;
  } else {
    if (rtb_RhhcosphisinlambYe < -1.0F) {
      rtb_RhhcosphisinlambYe = -1.0F;
    }
  }

  /* End of Saturate: '<S95>/Psi Dot  Limit' */

  /* Outputs for Enabled SubSystem: '<S95>/Sideslip Compensation' incorporates:
   *  EnablePort: '<S102>/Enable'
   */
  /* DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON' */
  if (mlParamInterface.param[21L] > 0.0F) {
    if (!SLUGS2_DWork.SideslipCompensation_MODE) {
      /* InitializeConditions for UnitDelay: '<S119>/UD'
       *
       * Block description for '<S119>/UD':
       *
       *  Store in Global RAM
       */
      SLUGS2_DWork.UD_DSTATE_f = 0.0F;

      /* InitializeConditions for Delay: '<S111>/Integer Delay' */
      SLUGS2_DWork.IntegerDelay_DSTATE_ed = 0.0F;

      /* InitializeConditions for Delay: '<S111>/Integer Delay1' */
      SLUGS2_DWork.IntegerDelay1_DSTATE_n = 0.0F;

      /* InitializeConditions for Delay: '<S112>/Integer Delay3' */
      SLUGS2_DWork.IntegerDelay3_DSTATE_ph = 0.0F;
      SLUGS2_DWork.SideslipCompensation_MODE = true;
    }

    /* MATLAB Function: '<S110>/negprotect' */
    SLUGS2_negprotect(rtb_Ze_b, &SLUGS2_B.sf_negprotect);

    /* Saturate: '<S110>/bank Limit' */
    if (SLUGS2_B.y_j[0] > 0.436332315F) {
      SLUGS2_B.bankLimit_e = 0.436332315F;
    } else if (SLUGS2_B.y_j[0] < -0.436332315F) {
      SLUGS2_B.bankLimit_e = -0.436332315F;
    } else {
      SLUGS2_B.bankLimit_e = SLUGS2_B.y_j[0];
    }

    /* End of Saturate: '<S110>/bank Limit' */

    /* S-Function (MCHP_C_function_Call): '<S114>/myTan() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.myTanapUtilscupdated5116_o = myTan(
      SLUGS2_B.bankLimit_e
      );

    /* InitialCondition: '<S111>/IC' */
    if (SLUGS2_DWork.IC_FirstOutputTime_b) {
      SLUGS2_DWork.IC_FirstOutputTime_b = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S118>/Abs1' incorporates:
       *  Constant: '<S102>/Constant'
       *  Sum: '<S119>/Diff'
       *  UnitDelay: '<S119>/UD'
       *
       * Block description for '<S119>/Diff':
       *
       *  Add in CPU
       *
       * Block description for '<S119>/UD':
       *
       *  Store in Global RAM
       */
      rtb_Sum1_mj = (real32_T)fabs(0.3F - SLUGS2_DWork.UD_DSTATE_f);

      /* Saturate: '<S118>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S118>/Saturation1' */
    }

    /* End of InitialCondition: '<S111>/IC' */

    /* Outputs for Enabled SubSystem: '<S111>/Compute Coef' incorporates:
     *  EnablePort: '<S117>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Math: '<S117>/Math Function' incorporates:
       *  Constant: '<S102>/Constant'
       *  Gain: '<S117>/-T'
       *
       * About '<S117>/Math Function':
       *  Operator: exp
       */
      SLUGS2_B.c = 0.997004509F;

      /* Sum: '<S117>/1-c' incorporates:
       *  Constant: '<S117>/Constant'
       */
      SLUGS2_B.c_b = (real32_T)(1.0 - SLUGS2_B.c);
    }

    /* End of Outputs for SubSystem: '<S111>/Compute Coef' */

    /* Sum: '<S111>/Subtract' incorporates:
     *  Delay: '<S111>/Integer Delay'
     *  Delay: '<S111>/Integer Delay1'
     *  Product: '<S111>/Divide'
     *  Product: '<S111>/Divide1'
     */
    SLUGS2_B.Subtract = SLUGS2_B.c_b * SLUGS2_DWork.IntegerDelay_DSTATE_ed +
      SLUGS2_B.c * SLUGS2_DWork.IntegerDelay1_DSTATE_n;

    /* Product: '<S113>/Divide' incorporates:
     *  Constant: '<S110>/Constant1'
     *  Constant: '<S113>/Constant1'
     *  Product: '<S110>/Divide'
     *  Product: '<S110>/Divide1'
     *  Sum: '<S102>/Subtract'
     */
    SLUGS2_B.Divide_h = (1.0F / SLUGS2_B.sf_negprotect.zpVal * 9.80665F *
                         SLUGS2_B.myTanapUtilscupdated5116_o - SLUGS2_B.y_j[5]) *
      rtb_Ze_b / 9.80665F;

    /* S-Function (MCHP_C_function_Call): '<S120>/myTan() apUtils.c [updated 5.1.16]' */
    SLUGS2_B.myTanapUtilscupdated5116_i = myTan(
      SLUGS2_B.Divide_h
      );

    /* Switch: '<S112>/Switch3' incorporates:
     *  RelationalOperator: '<S112>/Relational Operator2'
     */
    if ((SLUGS2_B.myTanapUtilscupdated5116_i ==
         SLUGS2_B.myTanapUtilscupdated5116_i) > 0) {
      SLUGS2_DWork.IntegerDelay3_DSTATE_ph = SLUGS2_B.myTanapUtilscupdated5116_i;
    }

    /* End of Switch: '<S112>/Switch3' */

    /* Update for UnitDelay: '<S119>/UD' incorporates:
     *  Constant: '<S102>/Constant'
     *
     * Block description for '<S119>/UD':
     *
     *  Store in Global RAM
     */
    SLUGS2_DWork.UD_DSTATE_f = 0.3F;

    /* Update for Delay: '<S111>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_ed = SLUGS2_DWork.IntegerDelay3_DSTATE_ph;

    /* Update for Delay: '<S111>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_n = SLUGS2_B.Subtract;
  } else {
    if (SLUGS2_DWork.SideslipCompensation_MODE) {
      /* Disable for Outport: '<S102>/bankComp' */
      SLUGS2_B.Subtract = 0.0F;
      SLUGS2_DWork.SideslipCompensation_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S95>/Sideslip Compensation' */

  /* Product: '<S104>/Divide' incorporates:
   *  Constant: '<S104>/Constant1'
   */
  SLUGS2_B.Divide = rtb_RhhcosphisinlambYe * rtb_Ze_b / 9.80665F;

  /* S-Function (MCHP_C_function_Call): '<S126>/myAtan() apUtils.c [updated 5.1.16]' */
  SLUGS2_B.myAtanapUtilscupdated5116_a = myAtan(
    SLUGS2_B.Divide
    );

  /* Sum: '<S95>/Add2' */
  rtb_Deg2R1 = SLUGS2_B.Subtract + SLUGS2_B.myAtanapUtilscupdated5116_a;

  /* Saturate: '<S95>/Bank  Limit Command' */
  if (rtb_Deg2R1 > 0.436332315F) {
    rtb_Deg2R1 = 0.436332315F;
  } else {
    if (rtb_Deg2R1 < -0.436332315F) {
      rtb_Deg2R1 = -0.436332315F;
    }
  }

  /* End of Saturate: '<S95>/Bank  Limit Command' */

  /* Switch: '<S100>/Switch3' incorporates:
   *  RelationalOperator: '<S100>/Relational Operator2'
   */
  if ((rtb_Product3_j0[1] == rtb_Product3_j0[1]) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_oy = rtb_Product3_j0[1];
  }

  /* End of Switch: '<S100>/Switch3' */

  /* Saturate: '<S95>/[-20 20]' */
  if (SLUGS2_DWork.IntegerDelay3_DSTATE_oy > 20.0F) {
    rtb_Sum1_mj = 20.0F;
  } else if (SLUGS2_DWork.IntegerDelay3_DSTATE_oy < -20.0F) {
    rtb_Sum1_mj = -20.0F;
  } else {
    rtb_Sum1_mj = SLUGS2_DWork.IntegerDelay3_DSTATE_oy;
  }

  /* MATLAB Function: '<S97>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S97>/Constant'
   *  Constant: '<S97>/Constant1'
   *  Saturate: '<S95>/[-20 20]'
   */
  SLU_EmbeddedMATLABFunction(rtb_Sum1_mj, 0.01, 10.0,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_b0,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_b0);

  /* DataStoreWrite: '<S63>/mlNavigation' incorporates:
   *  Gain: '<S95>/Neg Feedback '
   */
  mlNavigation.psiDot_c = rtb_RhhcosphisinlambYe;
  mlNavigation.phi_c = rtb_Deg2R1;
  mlNavigation.ay_body = -SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y;

  /* Sum: '<S103>/Add3' incorporates:
   *  Constant: '<S103>/SaturationLimit'
   */
  rtb_RhhcosphicoslambXe = 0.17453292F - SLUGS2_B.DataTypeConversion_g;

  /* Sum: '<S103>/Add1' incorporates:
   *  Constant: '<S103>/delayTime'
   *  DataStoreRead: '<S8>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Delay: '<S103>/NDelays'
   *  Gain: '<S95>/Neg Feedback '
   *  Product: '<S103>/Product1'
   *  Product: '<S103>/Product2'
   *  Product: '<S103>/Product3'
   *  Sum: '<S103>/Sum'
   */
  rtb_Sum1_mj = (-SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y -
                 SLUGS2_DWork.NDelays_DSTATE_l[0]) / 0.05F *
    mlParamInterface.param[14] + -SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y *
    mlParamInterface.param[12];

  /* Sum: '<S103>/Sum2' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Gain: '<S103>/Gain'
   *  Gain: '<S95>/Neg Feedback '
   *  Memory: '<S103>/Memory1'
   *  Product: '<S103>/Product4'
   */
  rtb_RhhcosphisinlambYe = (0.01F * -SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y *
    mlParamInterface.param[13] + SLUGS2_DWork.Memory1_PreviousInput_g) +
    rtb_Sum1_mj;

  /* Switch: '<S103>/AntiWindup' incorporates:
   *  Constant: '<S103>/Constant5'
   *  Constant: '<S103>/SaturationLimit'
   *  Constant: '<S103>/SaturationLimit1'
   *  Gain: '<S95>/Neg Feedback '
   *  Logic: '<S103>/Logical Operator'
   *  RelationalOperator: '<S103>/Relational Operator'
   *  RelationalOperator: '<S103>/Relational Operator1'
   *  Sum: '<S103>/Add3'
   *  Sum: '<S103>/Add4'
   */
  if ((rtb_RhhcosphisinlambYe > -0.17453292F - SLUGS2_B.DataTypeConversion_g) &&
      (rtb_RhhcosphisinlambYe < 0.17453292F - SLUGS2_B.DataTypeConversion_g)) {
    rtb_RhhcosphisinlambYe = -SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y;
  } else {
    rtb_RhhcosphisinlambYe = 0.0F;
  }

  /* End of Switch: '<S103>/AntiWindup' */

  /* Switch: '<S124>/Switch3' incorporates:
   *  RelationalOperator: '<S124>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe == rtb_RhhcosphisinlambYe) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_n = rtb_RhhcosphisinlambYe;
  }

  /* End of Switch: '<S124>/Switch3' */

  /* Switch: '<S123>/Switch1' incorporates:
   *  Constant: '<S123>/Constant'
   *  Constant: '<S123>/Constant1'
   *  Constant: '<S123>/Constant2'
   *  Constant: '<S123>/Constant3'
   *  Constant: '<S123>/Constant5'
   *  Delay: '<S123>/Integer Delay'
   *  Delay: '<S123>/Integer Delay1'
   *  Delay: '<S123>/Integer Delay2'
   *  Product: '<S123>/Product'
   *  Product: '<S123>/Product1'
   *  Product: '<S123>/Product2'
   *  Product: '<S123>/Product3'
   *  Sum: '<S123>/Subtract'
   *  Sum: '<S123>/Subtract1'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_cosphi = 0.0F;
  } else {
    rtb_cosphi = ((SLUGS2_DWork.IntegerDelay3_DSTATE_n * 0.333333343F +
                   SLUGS2_DWork.IntegerDelay_DSTATE_o * 1.33333337F) +
                  SLUGS2_DWork.IntegerDelay1_DSTATE_l[0] * 0.333333343F) *
      0.005F + SLUGS2_DWork.IntegerDelay2_DSTATE_bj;
  }

  /* End of Switch: '<S123>/Switch1' */

  /* Switch: '<S125>/Switch3' incorporates:
   *  RelationalOperator: '<S125>/Relational Operator2'
   */
  if ((rtb_cosphi == rtb_cosphi) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_f = rtb_cosphi;
  }

  /* End of Switch: '<S125>/Switch3' */

  /* Switch: '<S103>/On//Off' incorporates:
   *  Constant: '<S103>/Constant1'
   *  DataStoreRead: '<S8>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Product: '<S103>/Product'
   *  Sum: '<S103>/Add2'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    SLUGS2_DWork.Memory1_PreviousInput_g = 0.0F;
  } else {
    SLUGS2_DWork.Memory1_PreviousInput_g = SLUGS2_DWork.IntegerDelay3_DSTATE_f *
      mlParamInterface.param[13] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S103>/On//Off' */

  /* Switch: '<S122>/Switch2' incorporates:
   *  Constant: '<S103>/SaturationLimit'
   *  RelationalOperator: '<S122>/LowerRelop1'
   *  Sum: '<S103>/Add3'
   */
  if (!(SLUGS2_DWork.Memory1_PreviousInput_g > 0.17453292F -
        SLUGS2_B.DataTypeConversion_g)) {
    /* Switch: '<S122>/Switch' incorporates:
     *  Constant: '<S103>/SaturationLimit1'
     *  RelationalOperator: '<S122>/UpperRelop'
     *  Sum: '<S103>/Add4'
     */
    if (SLUGS2_DWork.Memory1_PreviousInput_g < -0.17453292F -
        SLUGS2_B.DataTypeConversion_g) {
      rtb_RhhcosphicoslambXe = -0.17453292F - SLUGS2_B.DataTypeConversion_g;
    } else {
      rtb_RhhcosphicoslambXe = SLUGS2_DWork.Memory1_PreviousInput_g;
    }

    /* End of Switch: '<S122>/Switch' */
  }

  /* End of Switch: '<S122>/Switch2' */

  /* Sum: '<S95>/Add' */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe +
    SLUGS2_B.DataTypeConversion_g;

  /* Sum: '<S95>/Add1' */
  rtb_Deg2R1 -= SLUGS2_B.y_j[0];

  /* Sum: '<S101>/Add3' incorporates:
   *  Constant: '<S101>/SaturationLimit'
   */
  rtb_cosphi = 0.383972436F - SLUGS2_B.DataTypeConversion_h;

  /* Switch: '<S99>/Switch3' incorporates:
   *  RelationalOperator: '<S99>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_k0 = rtb_Deg2R1;
  }

  /* End of Switch: '<S99>/Switch3' */

  /* Sum: '<S101>/Add1' incorporates:
   *  Constant: '<S101>/delayTime'
   *  DataStoreRead: '<S8>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Delay: '<S101>/NDelays'
   *  Product: '<S101>/Product1'
   *  Product: '<S101>/Product2'
   *  Product: '<S101>/Product3'
   *  Sum: '<S101>/Sum'
   */
  rtb_Sum1_mj = (SLUGS2_DWork.IntegerDelay3_DSTATE_k0 -
                 SLUGS2_DWork.NDelays_DSTATE_p[0]) / 0.05F *
    mlParamInterface.param[8] + SLUGS2_DWork.IntegerDelay3_DSTATE_k0 *
    mlParamInterface.param[6];

  /* Sum: '<S101>/Sum2' incorporates:
   *  DataStoreRead: '<S8>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Gain: '<S101>/Gain'
   *  Memory: '<S101>/Memory1'
   *  Product: '<S101>/Product4'
   */
  rtb_RhhcosphicoslambXe = (0.01F * SLUGS2_DWork.IntegerDelay3_DSTATE_k0 *
    mlParamInterface.param[7] + SLUGS2_DWork.Memory1_PreviousInput_j) +
    rtb_Sum1_mj;

  /* Switch: '<S101>/AntiWindup' incorporates:
   *  Constant: '<S101>/Constant5'
   *  Constant: '<S101>/SaturationLimit'
   *  Constant: '<S101>/SaturationLimit1'
   *  Logic: '<S101>/Logical Operator'
   *  RelationalOperator: '<S101>/Relational Operator'
   *  RelationalOperator: '<S101>/Relational Operator1'
   *  Sum: '<S101>/Add3'
   *  Sum: '<S101>/Add4'
   */
  if ((rtb_RhhcosphicoslambXe > -0.383972436F - SLUGS2_B.DataTypeConversion_h) &&
      (rtb_RhhcosphicoslambXe < 0.383972436F - SLUGS2_B.DataTypeConversion_h)) {
    rtb_RhhcosphicoslambXe = SLUGS2_DWork.IntegerDelay3_DSTATE_k0;
  } else {
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* End of Switch: '<S101>/AntiWindup' */

  /* Switch: '<S108>/Switch3' incorporates:
   *  RelationalOperator: '<S108>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe == rtb_RhhcosphicoslambXe) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_l = rtb_RhhcosphicoslambXe;
  }

  /* End of Switch: '<S108>/Switch3' */

  /* Switch: '<S107>/Switch1' incorporates:
   *  Constant: '<S107>/Constant'
   *  Constant: '<S107>/Constant1'
   *  Constant: '<S107>/Constant2'
   *  Constant: '<S107>/Constant3'
   *  Constant: '<S107>/Constant5'
   *  Delay: '<S107>/Integer Delay'
   *  Delay: '<S107>/Integer Delay1'
   *  Delay: '<S107>/Integer Delay2'
   *  Product: '<S107>/Product'
   *  Product: '<S107>/Product1'
   *  Product: '<S107>/Product2'
   *  Product: '<S107>/Product3'
   *  Sum: '<S107>/Subtract'
   *  Sum: '<S107>/Subtract1'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = ((SLUGS2_DWork.IntegerDelay3_DSTATE_l *
      0.333333343F + SLUGS2_DWork.IntegerDelay_DSTATE_b * 1.33333337F) +
      SLUGS2_DWork.IntegerDelay1_DSTATE_l3[0] * 0.333333343F) * 0.005F +
      SLUGS2_DWork.IntegerDelay2_DSTATE_j;
  }

  /* End of Switch: '<S107>/Switch1' */

  /* Switch: '<S109>/Switch3' incorporates:
   *  RelationalOperator: '<S109>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe == rtb_RhhcosphicoslambXe) > 0) {
    SLUGS2_DWork.IntegerDelay3_DSTATE_d = rtb_RhhcosphicoslambXe;
  }

  /* End of Switch: '<S109>/Switch3' */

  /* Switch: '<S101>/On//Off' incorporates:
   *  Constant: '<S101>/Constant1'
   *  DataStoreRead: '<S8>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Product: '<S101>/Product'
   *  Sum: '<S101>/Add2'
   */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 > 0) {
    SLUGS2_DWork.Memory1_PreviousInput_j = 0.0F;
  } else {
    SLUGS2_DWork.Memory1_PreviousInput_j = SLUGS2_DWork.IntegerDelay3_DSTATE_d *
      mlParamInterface.param[7] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S101>/On//Off' */

  /* Switch: '<S106>/Switch2' incorporates:
   *  Constant: '<S101>/SaturationLimit'
   *  RelationalOperator: '<S106>/LowerRelop1'
   *  Sum: '<S101>/Add3'
   */
  if (!(SLUGS2_DWork.Memory1_PreviousInput_j > 0.383972436F -
        SLUGS2_B.DataTypeConversion_h)) {
    /* Switch: '<S106>/Switch' incorporates:
     *  Constant: '<S101>/SaturationLimit1'
     *  RelationalOperator: '<S106>/UpperRelop'
     *  Sum: '<S101>/Add4'
     */
    if (SLUGS2_DWork.Memory1_PreviousInput_j < -0.383972436F -
        SLUGS2_B.DataTypeConversion_h) {
      rtb_cosphi = -0.383972436F - SLUGS2_B.DataTypeConversion_h;
    } else {
      rtb_cosphi = SLUGS2_DWork.Memory1_PreviousInput_j;
    }

    /* End of Switch: '<S106>/Switch' */
  }

  /* End of Switch: '<S106>/Switch2' */

  /* Sum: '<S95>/Add3' */
  rtb_cosphi += SLUGS2_B.DataTypeConversion_h;

  /* Saturate: '<S95>/Aileron Limit' */
  if (rtb_cosphi > 0.383972436F) {
    rtb_Sum1_mj = 0.383972436F;
  } else if (rtb_cosphi < -0.383972436F) {
    rtb_Sum1_mj = -0.383972436F;
  } else {
    rtb_Sum1_mj = rtb_cosphi;
  }

  /* End of Saturate: '<S95>/Aileron Limit' */

  /* Switch: '<S95>/Switch2' incorporates:
   *  Constant: '<S95>/Constant'
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   *  Product: '<S95>/Product'
   */
  if (mlParamInterface.param[27L] > 0.3F) {
    rtb_RhhcosphicoslambXe = mlParamInterface.param[28] * rtb_Sum1_mj;
  } else {
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* End of Switch: '<S95>/Switch2' */

  /* Sum: '<S95>/Add4' */
  rtb_cosphi = rtb_RhhcosphicoslambXe + rtb_RhhcosphisinlambYe;

  /* Update for Delay: '<S103>/NDelays' incorporates:
   *  Gain: '<S95>/Neg Feedback '
   */
  SLUGS2_DWork.NDelays_DSTATE_l[0] = SLUGS2_DWork.NDelays_DSTATE_l[1];
  SLUGS2_DWork.NDelays_DSTATE_l[1] = SLUGS2_DWork.NDelays_DSTATE_l[2];
  SLUGS2_DWork.NDelays_DSTATE_l[2] = SLUGS2_DWork.NDelays_DSTATE_l[3];
  SLUGS2_DWork.NDelays_DSTATE_l[3] = SLUGS2_DWork.NDelays_DSTATE_l[4];
  SLUGS2_DWork.NDelays_DSTATE_l[4] = -SLUGS2_B.sf_EmbeddedMATLABFunction_b0.y;

  /* Update for Delay: '<S123>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE_bj = SLUGS2_DWork.IntegerDelay3_DSTATE_f;

  /* Update for Delay: '<S123>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_o = SLUGS2_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S123>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_l[0] = SLUGS2_DWork.IntegerDelay1_DSTATE_l[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_l[1] = SLUGS2_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S101>/NDelays' */
  SLUGS2_DWork.NDelays_DSTATE_p[0] = SLUGS2_DWork.NDelays_DSTATE_p[1];
  SLUGS2_DWork.NDelays_DSTATE_p[1] = SLUGS2_DWork.NDelays_DSTATE_p[2];
  SLUGS2_DWork.NDelays_DSTATE_p[2] = SLUGS2_DWork.NDelays_DSTATE_p[3];
  SLUGS2_DWork.NDelays_DSTATE_p[3] = SLUGS2_DWork.NDelays_DSTATE_p[4];
  SLUGS2_DWork.NDelays_DSTATE_p[4] = SLUGS2_DWork.IntegerDelay3_DSTATE_k0;

  /* Update for Delay: '<S107>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE_j = SLUGS2_DWork.IntegerDelay3_DSTATE_d;

  /* Update for Delay: '<S107>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_b = SLUGS2_DWork.IntegerDelay3_DSTATE_l;

  /* Update for Delay: '<S107>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_l3[0] =
    SLUGS2_DWork.IntegerDelay1_DSTATE_l3[1];
  SLUGS2_DWork.IntegerDelay1_DSTATE_l3[1] = SLUGS2_DWork.IntegerDelay3_DSTATE_l;

  /* End of Outputs for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S133>/Throttle  Limit' */
  /* MATLAB Function 'myMux Fun1': '<S18>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S18>:1:5' */
  if (rtb_Deg2R_h_idx_0 > 0.95F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_Deg2R_h_idx_0 = 0.95F;
  } else {
    if (rtb_Deg2R_h_idx_0 < 0.0F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_Deg2R_h_idx_0 = 0.0F;
    }
  }

  /* End of Saturate: '<S133>/Throttle  Limit' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S95>/Rudder Limit' */
  if (rtb_cosphi > 0.17453292F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_cosphi = 0.17453292F;
  } else {
    if (rtb_cosphi < -0.17453292F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_cosphi = -0.17453292F;
    }
  }

  /* End of Saturate: '<S95>/Rudder Limit' */
  /* End of Outputs for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S133>/Elevator  Limit' */
  if (rtb_IC4_idx_1 > 0.401425719F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_IC4_idx_1 = 0.401425719F;
  } else {
    if (rtb_IC4_idx_1 < -0.401425719F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_IC4_idx_1 = -0.401425719F;
    }
  }

  /* End of Saturate: '<S133>/Elevator  Limit' */
  /* End of Outputs for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* If: '<S670>/If' */
  if (SLUGS2_B.ManualorAutonavSupportcupdated4 == 1) {
    /* Outputs for IfAction SubSystem: '<S670>/If  Control Type Is Manual' incorporates:
     *  ActionPort: '<S673>/Action Port'
     */
    /* Gain: '<S673>/Gain4' incorporates:
     *  MATLAB Function: '<S644>/myMux Fun5'
     */
    SLUGS2_B.Merge[0] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o3;
    SLUGS2_B.Merge[1] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o1;
    SLUGS2_B.Merge[2] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o4;
    SLUGS2_B.Merge[3] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o2;

    /* End of Outputs for SubSystem: '<S670>/If  Control Type Is Manual' */
  } else if ((rtb_DataTypeConversion1_hq == 3) || (rtb_DataTypeConversion1_hq ==
              4) || (rtb_DataTypeConversion1_hq == 9) ||
             (rtb_DataTypeConversion1_hq == 10)) {
    /* Outputs for IfAction SubSystem: '<S670>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1' incorporates:
     *  ActionPort: '<S677>/Action Port'
     */
    /* DataTypeConversion: '<S692>/Data Type Conversion' incorporates:
     *  Constant: '<S692>/Constant1'
     *  Constant: '<S692>/Constant2'
     *  Product: '<S692>/Divide'
     *  Sum: '<S692>/Add'
     */
    tmp_4 = floor(rtb_IC4_idx_1 * 10119.867056498166 + 6420.8333333333348);
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      rtb_u2deg = 0U;
    } else {
      rtb_u2deg = (uint16_T)fmod(tmp_4, 65536.0);
    }

    /* End of DataTypeConversion: '<S692>/Data Type Conversion' */

    /* DataTypeConversion: '<S695>/Data Type Conversion' incorporates:
     *  Constant: '<S695>/Constant1'
     *  Constant: '<S695>/Constant2'
     *  Product: '<S695>/Divide'
     *  Sum: '<S695>/Add'
     */
    tmp_4 = floor(rtb_cosphi * -5105.0539546156351 + 6435.1666666666679);
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      rtb_DataTypeConversion_lk = 0U;
    } else {
      rtb_DataTypeConversion_lk = (uint16_T)fmod(tmp_4, 65536.0);
    }

    /* End of DataTypeConversion: '<S695>/Data Type Conversion' */

    /* DataTypeConversion: '<S694>/Data Type Conversion' incorporates:
     *  Constant: '<S694>/Constant1'
     *  Constant: '<S694>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S694>/Divide'
     *  Sum: '<S694>/Add'
     */
    tmp_4 = floor(rtb_Sum1_mj * -6822.0174806909972 + 6431.0000000000018);
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      rtb_DataTypeConversion_ee = 0U;
    } else {
      rtb_DataTypeConversion_ee = (uint16_T)fmod(tmp_4, 65536.0);
    }

    /* End of DataTypeConversion: '<S694>/Data Type Conversion' */

    /* DataTypeConversion: '<S693>/Data Type Conversion' incorporates:
     *  Constant: '<S693>/Constant1'
     *  Constant: '<S693>/Constant2'
     *  Product: '<S693>/Divide'
     *  Sum: '<S693>/Add'
     */
    tmp_4 = floor(rtb_Deg2R_h_idx_0 * 3631.5 + 4612.0);
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      rtb_DataTypeConversion_g0 = 0U;
    } else {
      rtb_DataTypeConversion_g0 = (uint16_T)fmod(tmp_4, 65536.0);
    }

    /* End of DataTypeConversion: '<S693>/Data Type Conversion' */

    /* S-Function (MCHP_C_function_Call): '<S677>/Initialize Control MCU' */
    getPassValues(
                  &SLUGS2_B.InitializeControlMCU_a[0]
                  );

    /* Switch: '<S677>/Switch' incorporates:
     *  Gain: '<S677>/Gain'
     *  Gain: '<S691>/Gain4'
     *  MATLAB Function: '<S644>/myMux Fun5'
     *  Sum: '<S691>/Add'
     */
    if (SLUGS2_B.InitializeControlMCU_a[3] >= 1) {
      rtb_RhhcosphicoslambXe = (0.5F * (real32_T)SLUGS2_B.InputCapture_o3 +
        (real32_T)rtb_DataTypeConversion_g0) * 0.5F;
    } else {
      rtb_RhhcosphicoslambXe = rtb_DataTypeConversion_g0;
    }

    /* End of Switch: '<S677>/Switch' */

    /* Switch: '<S677>/Switch1' incorporates:
     *  Gain: '<S677>/Gain1'
     *  Gain: '<S690>/Gain4'
     *  MATLAB Function: '<S644>/myMux Fun5'
     *  Sum: '<S690>/Add'
     */
    if (SLUGS2_B.InitializeControlMCU_a[2] >= 1) {
      rtb_Sum1_mj = (0.5F * (real32_T)SLUGS2_B.InputCapture_o1 + (real32_T)
                     rtb_DataTypeConversion_ee) * 0.5F;
    } else {
      rtb_Sum1_mj = rtb_DataTypeConversion_ee;
    }

    /* End of Switch: '<S677>/Switch1' */

    /* Switch: '<S677>/Switch2' incorporates:
     *  Gain: '<S677>/Gain2'
     *  Gain: '<S689>/Gain4'
     *  MATLAB Function: '<S644>/myMux Fun5'
     *  Sum: '<S689>/Add'
     */
    if (SLUGS2_B.InitializeControlMCU_a[1] >= 1) {
      rtb_RhhcosphisinlambYe = (0.5F * (real32_T)SLUGS2_B.InputCapture_o4 +
        (real32_T)rtb_DataTypeConversion_lk) * 0.5F;
    } else {
      rtb_RhhcosphisinlambYe = rtb_DataTypeConversion_lk;
    }

    /* End of Switch: '<S677>/Switch2' */

    /* Switch: '<S677>/Switch3' incorporates:
     *  Gain: '<S677>/Gain3'
     *  Gain: '<S688>/Gain4'
     *  MATLAB Function: '<S644>/myMux Fun5'
     *  Sum: '<S688>/Add'
     */
    if (SLUGS2_B.InitializeControlMCU_a[0] >= 1) {
      rtb_Deg2R1 = (0.5F * (real32_T)SLUGS2_B.InputCapture_o2 + (real32_T)
                    rtb_u2deg) * 0.5F;
    } else {
      rtb_Deg2R1 = rtb_u2deg;
    }

    /* End of Switch: '<S677>/Switch3' */

    /* MATLAB Function: '<S677>/myMux Fun1' */
    SLUGS2_myMuxFun1_h(rtb_RhhcosphicoslambXe, rtb_Sum1_mj,
                       rtb_RhhcosphisinlambYe, rtb_Deg2R1, SLUGS2_B.Merge);

    /* End of Outputs for SubSystem: '<S670>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds and we want to add Selective Passthrough1' */
  } else if (rtb_DataTypeConversion1_hq == 2) {
    /* Outputs for IfAction SubSystem: '<S670>/If  Control Type Is Passthrough' incorporates:
     *  ActionPort: '<S674>/Action Port'
     */
    /* Gain: '<S674>/Gain4' incorporates:
     *  MATLAB Function: '<S644>/myMux Fun5'
     */
    SLUGS2_B.Merge[0] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o3;
    SLUGS2_B.Merge[1] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o1;
    SLUGS2_B.Merge[2] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o4;
    SLUGS2_B.Merge[3] = 0.5F * (real32_T)SLUGS2_B.InputCapture_o2;

    /* End of Outputs for SubSystem: '<S670>/If  Control Type Is Passthrough' */
  } else {
    if (rtb_DataTypeConversion1_hq == 8) {
      /* Outputs for IfAction SubSystem: '<S670>/If  Control Type Is Selective Passthrough' incorporates:
       *  ActionPort: '<S675>/Action Port'
       */

      /* S-Function (MCHP_C_function_Call): '<S675>/Initialize Control MCU' */
      getPassValues(
                    &SLUGS2_B.InitializeControlMCU[0]
                    );

      /* Switch: '<S675>/Switch' incorporates:
       *  Gain: '<S675>/Gain'
       *  MATLAB Function: '<S644>/myMux Fun5'
       */
      if (SLUGS2_B.InitializeControlMCU[3] >= 1) {
        rtb_RhhcosphicoslambXe = 0.5F * (real32_T)SLUGS2_B.InputCapture_o3;
      } else {
        /* DataTypeConversion: '<S679>/Data Type Conversion' incorporates:
         *  Constant: '<S679>/Constant1'
         *  Constant: '<S679>/Constant2'
         *  Product: '<S679>/Divide'
         *  Sum: '<S679>/Add'
         */
        tmp_4 = floor(rtb_Deg2R_h_idx_0 * 3631.5 + 4612.0);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_RhhcosphicoslambXe = 0.0F;
        } else {
          rtb_RhhcosphicoslambXe = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S679>/Data Type Conversion' */
      }

      /* End of Switch: '<S675>/Switch' */

      /* Switch: '<S675>/Switch1' incorporates:
       *  Gain: '<S675>/Gain1'
       *  MATLAB Function: '<S644>/myMux Fun5'
       */
      if (SLUGS2_B.InitializeControlMCU[2] >= 1) {
        rtb_Deg2R1 = 0.5F * (real32_T)SLUGS2_B.InputCapture_o1;
      } else {
        /* DataTypeConversion: '<S680>/Data Type Conversion' incorporates:
         *  Constant: '<S680>/Constant1'
         *  Constant: '<S680>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S680>/Divide'
         *  Sum: '<S680>/Add'
         */
        tmp_4 = floor(rtb_Sum1_mj * -6822.0174806909972 + 6431.0000000000018);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_Deg2R1 = 0.0F;
        } else {
          rtb_Deg2R1 = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S680>/Data Type Conversion' */
      }

      /* End of Switch: '<S675>/Switch1' */

      /* Switch: '<S675>/Switch2' incorporates:
       *  Gain: '<S675>/Gain2'
       *  MATLAB Function: '<S644>/myMux Fun5'
       */
      if (SLUGS2_B.InitializeControlMCU[1] >= 1) {
        rtb_RhhcosphisinlambYe = 0.5F * (real32_T)SLUGS2_B.InputCapture_o4;
      } else {
        /* DataTypeConversion: '<S681>/Data Type Conversion' incorporates:
         *  Constant: '<S681>/Constant1'
         *  Constant: '<S681>/Constant2'
         *  Product: '<S681>/Divide'
         *  Sum: '<S681>/Add'
         */
        tmp_4 = floor(rtb_cosphi * -5105.0539546156351 + 6435.1666666666679);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_RhhcosphisinlambYe = 0.0F;
        } else {
          rtb_RhhcosphisinlambYe = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S681>/Data Type Conversion' */
      }

      /* End of Switch: '<S675>/Switch2' */

      /* Switch: '<S675>/Switch3' incorporates:
       *  Gain: '<S675>/Gain3'
       *  MATLAB Function: '<S644>/myMux Fun5'
       */
      if (SLUGS2_B.InitializeControlMCU[0] >= 1) {
        rtb_Sum1_mj = 0.5F * (real32_T)SLUGS2_B.InputCapture_o2;
      } else {
        /* DataTypeConversion: '<S678>/Data Type Conversion' incorporates:
         *  Constant: '<S678>/Constant1'
         *  Constant: '<S678>/Constant2'
         *  Product: '<S678>/Divide'
         *  Sum: '<S678>/Add'
         */
        tmp_4 = floor(rtb_IC4_idx_1 * 10119.867056498166 + 6420.8333333333348);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_Sum1_mj = 0.0F;
        } else {
          rtb_Sum1_mj = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S678>/Data Type Conversion' */
      }

      /* End of Switch: '<S675>/Switch3' */

      /* MATLAB Function: '<S675>/myMux Fun1' */
      SLUGS2_myMuxFun1_h(rtb_RhhcosphicoslambXe, rtb_Deg2R1,
                         rtb_RhhcosphisinlambYe, rtb_Sum1_mj, SLUGS2_B.Merge);

      /* End of Outputs for SubSystem: '<S670>/If  Control Type Is Selective Passthrough' */
    }
  }

  /* End of If: '<S670>/If' */

  /* DataTypeConversion: '<S670>/Data Type Conversion' */
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_B.Merge[0]);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  rtb_u2deg = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)-(int16_T)(uint16_T)
    -rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_B.Merge[1]);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  rtb_DataTypeConversion_lk = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)
    rtb_RhhcosphicoslambXe;
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_B.Merge[2]);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  rtb_DataTypeConversion_ee = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)
    rtb_RhhcosphicoslambXe;
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_B.Merge[3]);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  rtb_DataTypeConversion_g0 = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)
    rtb_RhhcosphicoslambXe;

  /* End of DataTypeConversion: '<S670>/Data Type Conversion' */

  /* DataStoreWrite: '<S16>/mlPwmCommands' incorporates:
   *  DataStoreRead: '<S16>/Get time3'
   *  DataTypeConversion: '<S669>/Data Type Conversion'
   *  Gain: '<S669>/Convert to  Microseconds'
   */
  mlPwmCommands.servo1_raw = (uint16_T)(((uint32_T)rtb_u2deg << 15) >> 14);
  mlPwmCommands.servo2_raw = (uint16_T)(((uint32_T)rtb_DataTypeConversion_lk <<
    15) >> 14);
  mlPwmCommands.servo3_raw = (uint16_T)(((uint32_T)rtb_DataTypeConversion_ee <<
    15) >> 14);
  mlPwmCommands.servo4_raw = (uint16_T)(((uint32_T)rtb_DataTypeConversion_g0 <<
    15) >> 14);
  mlPwmCommands.time_usec = SLUGS2_DWork.time_since_boot_usec;

  /* DataStoreWrite: '<S16>/mlAttitudePQR' */
  mlAttitudeSol.rollspeed = rtb_Product2_c[0];
  mlAttitudeSol.pitchspeed = rtb_Product2_c[1];
  mlAttitudeSol.yawspeed = rtb_Product2_c[2];

  /* MATLAB Function: '<S653>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S653>/Constant'
   *  Constant: '<S653>/Constant1'
   *  DataTypeConversion: '<S644>/Data Type Conversion2'
   */
  SLU_EmbeddedMATLABFunction((real32_T)SLUGS2_B.InputCapture_o6, 0.01, 0.1,
    &SLUGS2_B.sf_EmbeddedMATLABFunction, &SLUGS2_DWork.sf_EmbeddedMATLABFunction);

  /* DataTypeConversion: '<S644>/Data Type Conversion1' */
  rtb_RhhcosphicoslambXe = (real32_T)floor(SLUGS2_B.sf_EmbeddedMATLABFunction.y);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  /* MATLAB Function: '<S654>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S654>/Constant'
   *  Constant: '<S654>/Constant1'
   *  DataTypeConversion: '<S644>/Data Type Conversion3'
   */
  SLU_EmbeddedMATLABFunction((real32_T)SLUGS2_B.InputCapture_o7, 0.01, 0.1,
    &SLUGS2_B.sf_EmbeddedMATLABFunction_b,
    &SLUGS2_DWork.sf_EmbeddedMATLABFunction_b);

  /* DataTypeConversion: '<S644>/Data Type Conversion4' */
  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun1': '<S655>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S655>:1:5' */
  rtb_Sum1_mj = (real32_T)floor(SLUGS2_B.sf_EmbeddedMATLABFunction_b.y);
  if (rtIsNaNF(rtb_Sum1_mj) || rtIsInfF(rtb_Sum1_mj)) {
    rtb_Sum1_mj = 0.0F;
  } else {
    rtb_Sum1_mj = (real32_T)fmod(rtb_Sum1_mj, 65536.0F);
  }

  /* DataStoreWrite: '<S16>/mlAttitude' incorporates:
   *  DataTypeConversion: '<S644>/Data Type Conversion1'
   *  DataTypeConversion: '<S644>/Data Type Conversion4'
   */
  mlVISensor.voltage = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)-(int16_T)
    (uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;
  mlVISensor.reading2 = rtb_Sum1_mj < 0.0F ? (uint16_T)-(int16_T)(uint16_T)
    -rtb_Sum1_mj : (uint16_T)rtb_Sum1_mj;

  /* RateTransition: '<S16>/Rate Transition' */
  if (SLUGS2_M->Timing.RateInteraction.TID0_1 == 1) {
    SLUGS2_B.RateTransition[0] = rtb_DataTypeConversion_lk;
    SLUGS2_B.RateTransition[1] = rtb_DataTypeConversion_g0;
    SLUGS2_B.RateTransition[2] = rtb_u2deg;
    SLUGS2_B.RateTransition[3] = rtb_DataTypeConversion_ee;
  }

  /* End of RateTransition: '<S16>/Rate Transition' */

  /* Outputs for Enabled SubSystem: '<S16>/Send_Cmd_6DOF_HIL' incorporates:
   *  EnablePort: '<S671>/Enable'
   */
  /* DataStoreRead: '<S16>/Data Store Read' incorporates:
   *  Constant: '<S671>/COMP_ID'
   *  Constant: '<S671>/SYS_ID'
   */
  if (SLUGS2_DWork.SIX_DOF_HIL_FLAG > 0.0) {
    /* DataStoreRead: '<S671>/Get Raw Commands' */
    SLUGS2_B.GetRawCommands = mlPwmCommands;

    /* DataStoreRead: '<S671>/Get time5' */
    SLUGS2_B.Gettime5 = SLUGS2_DWork.time_since_boot_usec;

    /* S-Function (MCHP_C_function_Call): '<S671>/PackRawServo' */
    SLUGS2_B.PackRawServo_l = HIL_PackRawServo(
      ((uint8_T)101U)
      , ((uint8_T)1U)
      , SLUGS2_B.GetRawCommands
      , SLUGS2_B.Gettime5
      );

    /* S-Function (MCHP_C_function_Call): '<S671>/TX_N_Data9' */
    TxN_Data_OverU4(
                    SLUGS2_B.PackRawServo_l
                    );
  }

  /* End of DataStoreRead: '<S16>/Data Store Read' */
  /* End of Outputs for SubSystem: '<S16>/Send_Cmd_6DOF_HIL' */

  /* Outputs for Enabled SubSystem: '<S16>/Send_Cmd_X_Plane_HIL' incorporates:
   *  EnablePort: '<S672>/Enable'
   */
  /* DataStoreRead: '<S16>/Data Store Read1' */
  if (SLUGS2_DWork.X_PLANE_HIL_FLAG > 0.0) {
    /* S-Function (MCHP_C_function_Call): '<S672>/PackRawServo' */
    SLUGS2_B.PackRawServo = send_HILSIM_outputs(
      );

    /* S-Function (MCHP_C_function_Call): '<S672>/TX_N_Data9' */
    TxN_Data_OverU4(
                    SLUGS2_B.PackRawServo
                    );
  }

  /* End of DataStoreRead: '<S16>/Data Store Read1' */
  /* End of Outputs for SubSystem: '<S16>/Send_Cmd_X_Plane_HIL' */

  /* DataStoreWrite: '<S644>/Update Control Surface DATA' */
  mlPilotConsoleData.chan1_raw = SLUGS2_B.InputCapture_o1;
  mlPilotConsoleData.chan2_raw = SLUGS2_B.InputCapture_o2;
  mlPilotConsoleData.chan3_raw = SLUGS2_B.InputCapture_o3;
  mlPilotConsoleData.chan4_raw = SLUGS2_B.InputCapture_o4;
  mlPilotConsoleData.chan5_raw = SLUGS2_B.InputCapture_o5;
  mlPilotConsoleData.chan6_raw = SLUGS2_B.InputCapture_o6;
  mlPilotConsoleData.chan7_raw = SLUGS2_B.InputCapture_o7;
  mlPilotConsoleData.chan8_raw = SLUGS2_B.InputCapture_o8;
  mlPilotConsoleData.port = 8U;
  mlPilotConsoleData.rssi = 20U;

  /* DataStoreWrite: '<Root>/Data Store Write' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  SLUGS2_DWork.SIX_DOF_HIL_FLAG = 0.0;

  /* DataStoreWrite: '<Root>/Data Store Write1' incorporates:
   *  Constant: '<Root>/Constant5'
   */
  SLUGS2_DWork.X_PLANE_HIL_FLAG = 1.0;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S2>/Output'
   */
  tmp_7 = 3276800000UL;
  uMultiWordMul(&tmp_7, 1, &SLUGS2_DWork.Output_DSTATE, 1, &tmp_6.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_6.chunks[0U], 2, 15U, &tmp_5.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  SLUGS2_DWork.time_since_boot_usec = MultiWord2uLong(&tmp_5.chunks[0U]);

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  SLUGS2_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = SLUGS2_B.U3CH4;
#ifndef WIN
  /* Outputs for Atomic SubSystem: '<Root>/Barometer_Driver' */
  SLUGS2_Barometer_Driver();

  /* End of Outputs for SubSystem: '<Root>/Barometer_Driver' */

  /* Outputs for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  SLUGS2_IMU_Mag_Driver();

  /* End of Outputs for SubSystem: '<Root>/IMU_Mag_Driver' */
#endif
  /* MATLAB Function: '<Root>/myMux Fun4' */
  SLUGS2_myMuxFun3(SLUGS2_B.BiasRateLimiter, SLUGS2_B.DataTypeConversion1,
                   &SLUGS2_B.sf_myMuxFun4);

  /* MATLAB Function: '<Root>/myMux Fun3' */
  SLUGS2_myMuxFun3(SLUGS2_B.sf_myMuxFun1_e.y, SLUGS2_B.sf_myMuxFun2_a.y,
                   &SLUGS2_B.sf_myMuxFun3);

  /* MATLAB Function: '<Root>/myMux Fun5' */
  /* MATLAB Function 'myMux Fun5': '<S22>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S22>:1:5' */
  SLUGS2_B.y[0] = SLUGS2_B.DataTypeConversion3[0];
  SLUGS2_B.y[1] = SLUGS2_B.DataTypeConversion3[1];
  SLUGS2_B.y[2] = SLUGS2_B.DataTypeConversion3[2];
  SLUGS2_B.y[3] = SLUGS2_B.DataTypeConversion6[0];
  SLUGS2_B.y[4] = SLUGS2_B.DataTypeConversion6[1];
  SLUGS2_B.y[5] = SLUGS2_B.DataTypeConversion6[2];
  SLUGS2_B.y[6] = SLUGS2_B.DataTypeConversion7[0];
  SLUGS2_B.y[7] = SLUGS2_B.DataTypeConversion7[1];
  SLUGS2_B.y[8] = SLUGS2_B.DataTypeConversion7[2];

  /* DataStoreRead: '<Root>/Get time4' */
  SLUGS2_B.Gettime4 = SLUGS2_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S14>/Update Euler and PQR [updateSensorMcuState.c]1' */
  updateEuler(
              &SLUGS2_B.y_k[0]
              );

  /* S-Function (MCHP_C_function_Call): '<S14>/Update the Time Stamp [updateSensorMcuState.c]1' */
  updateTimeStamp(
                  SLUGS2_B.Gettime4
                  );

  /* S-Function (MCHP_C_function_Call): '<S14>/Update the Time Stamp [updateSensorMcuState.c]2' */
  updateSensorData(
                   &SLUGS2_B.y[0]
                   );

  /* S-Function (MCHP_C_function_Call): '<S14>/Update the Time Stamp [updateSensorMcuState.c]7' */
  updatePosition(
                 &SLUGS2_B.sf_myMuxFun3.y[0]
                 );

  /* S-Function (MCHP_C_function_Call): '<S14>/Update the Time Stamp [updateSensorMcuState.c]6' */
  updateBias(
             &SLUGS2_B.sf_myMuxFun4.y[0]
             );

  /* S-Function (MCHP_Digital_Input): '<Root>/Config UART4 Rx Pin' */
#ifndef WIN
  /* MCHP_Digital_Input Block: <Root>/Config UART4 Rx Pin/Output */
  rtb_ConfigUART4RxPin = PORTEbits.RE6;/* Read pin E6 */
#endif

  /* Switch: '<S52>/FixPt Switch' incorporates:
   *  Constant: '<S51>/FixPt Constant'
   *  Sum: '<S51>/FixPt Sum1'
   *  UnitDelay: '<S2>/Output'
   */
  SLUGS2_DWork.Output_DSTATE++;

  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID0();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */

  /* Update for Delay: '<S66>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE = rtb_Switch3;

  /* Update for Delay: '<S67>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_k = rtb_Product;

  /* Update for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  Update for EnablePort: '<S62>/Enable'
   */
  if (SLUGS2_DWork.L1OutputFeedbackControllerWithP) {
    /* Update for UnitDelay: '<S82>/UD'
     *
     * Block description for '<S82>/UD':
     *
     *  Store in Global RAM
     */
    SLUGS2_DWork.UD_DSTATE_ff = rtb_IC4_idx_0;

    /* Update for Delay: '<S72>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_an = rtb_Switch1_a;

    /* Update for Delay: '<S72>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_i = rtb_Subtract_od;

    /* Update for Delay: '<S74>/Integer Delay3' */
    SLUGS2_DWork.IntegerDelay3_DSTATE_fs = rtb_Switch3_gd;

    /* Update for UnitDelay: '<S92>/UD'
     *
     * Block description for '<S92>/UD':
     *
     *  Store in Global RAM
     */
    SLUGS2_DWork.UD_DSTATE_j = rtb_y_jl_idx_0;

    /* Update for Delay: '<S76>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_k = rtb_Switch3_kh;

    /* Update for Delay: '<S76>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_b = rtb_Subtract_a;

    /* Update for Delay: '<S62>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_i = rtb_Product3_bo;

    /* Update for Delay: '<S62>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_e = rtb_Switch1_a;

    /* Update for Delay: '<S75>/Integer Delay3' */
    SLUGS2_DWork.IntegerDelay3_DSTATE_h = rtb_Switch3_kh;

    /* Update for Delay: '<S77>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_p = rtb_Projection;

    /* Update for Delay: '<S77>/Integer Delay1' */
    SLUGS2_DWork.IntegerDelay1_DSTATE_ey[0] =
      SLUGS2_DWork.IntegerDelay1_DSTATE_ey[1];
    SLUGS2_DWork.IntegerDelay1_DSTATE_ey[1] = rtb_Projection;

    /* Update for Delay: '<S77>/Integer Delay2' */
    SLUGS2_DWork.IntegerDelay2_DSTATE_g = rtb_Switch1_a;

    /* Update for Delay: '<S93>/Integer Delay3' */
    SLUGS2_DWork.IntegerDelay3_DSTATE_nz = rtb_Projection;

    /* Update for Delay: '<S94>/Integer Delay3' */
    SLUGS2_DWork.IntegerDelay3_DSTATE_hh = rtb_Switch1_a;
  }

  /* End of Update for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  SLUGS2_M->Timing.clockTick0++;
}

/* Model step function for TID1 */
void SLUGS2_step1(void)                /* Sample time: [0.02s, 0.0s] */
{
  /* Gain: '<S16>/Gain' */
  SLUGS2_B.Gain[0] = SLUGS2_B.RateTransition[0] << 1;
  SLUGS2_B.Gain[1] = SLUGS2_B.RateTransition[1] << 1;
  SLUGS2_B.Gain[2] = SLUGS2_B.RateTransition[2] << 1;
  SLUGS2_B.Gain[3] = SLUGS2_B.RateTransition[3] << 1;
#ifndef WIN

  /* S-Function "dsPIC_PWM_OC" Block: <S16>/Output Compare - HW Drive Servo motor */
  OC1CON1 = 0x1008;                    /* Disable OC1 */
  OC1CON2bits.TRIGSTAT = 0;
  OC1RS = SLUGS2_B.Gain[0];            /* Pulse in Set-Reset mode (mimic up & down mode) */
  OC2CON1 = 0x1008;                    /* Disable OC2 */
  OC2CON2bits.TRIGSTAT = 0;
  OC2RS = SLUGS2_B.Gain[1];            /* Pulse in Set-Reset mode (mimic up & down mode) */
  OC3CON1 = 0x1008;                    /* Disable OC3 */
  OC3CON2bits.TRIGSTAT = 0;
  OC3RS = SLUGS2_B.Gain[2];            /* Pulse in Set-Reset mode (mimic up & down mode) */
  OC4CON1 = 0x1008;                    /* Disable OC4 */
  OC4CON2bits.TRIGSTAT = 0;
  OC4RS = SLUGS2_B.Gain[3];            /* Pulse in Set-Reset mode (mimic up & down mode) */
  OC1CON2bits.TRIGSTAT = 1;
  OC1CON1 = 0x100C;                    /* Trig OC1 pulse */
  OC2CON2bits.TRIGSTAT = 1;
  OC2CON1 = 0x100C;                    /* Trig OC2 pulse */
  OC3CON2bits.TRIGSTAT = 1;
  OC3CON1 = 0x100C;                    /* Trig OC3 pulse */
  OC4CON2bits.TRIGSTAT = 1;
  OC4CON1 = 0x100C;                    /* Trig OC4 pulse */
#endif
}

/* Model step function for TID2 */
void SLUGS2_step2(void)                /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID3 */
void SLUGS2_step3(void)                /* Sample time: [0.1s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  SLUGS2_LEDs_DriverTID3();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID4 */
void SLUGS2_step4(void)                /* Sample time: [0.1s, 0.05s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  SLUGS2_LEDs_DriverTID4();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID5 */
void SLUGS2_step5(void)                /* Sample time: [0.1s, 0.06s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID5();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID6 */
void SLUGS2_step6(void)                /* Sample time: [0.1s, 0.08s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID6();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID7 */
void SLUGS2_step7(void)                /* Sample time: [0.2s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  SLUGS2_LEDs_DriverTID7();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID8 */
void SLUGS2_step8(void)                /* Sample time: [0.5s, 0.0s] */
{
#ifndef WIN

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
#endif
}

/* Model step function for TID9 */
void SLUGS2_step9(void)                /* Sample time: [0.5s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  SLUGS2_LEDs_DriverTID9();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID10 */
void SLUGS2_step10(void)               /* Sample time: [0.5s, 0.1s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID10();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID11 */
void SLUGS2_step11(void)               /* Sample time: [0.5s, 0.2s] */
{
  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );
}

/* Model step function for TID12 */
void SLUGS2_step12(void)               /* Sample time: [0.5s, 0.3s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID12();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID13 */
void SLUGS2_step13(void)               /* Sample time: [1.0s, 0.1s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID13();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID14 */
void SLUGS2_step14(void)               /* Sample time: [1.0s, 0.4s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID14();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID15 */
void SLUGS2_step15(void)               /* Sample time: [1.0s, 0.5s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID15();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID16 */
void SLUGS2_step16(void)               /* Sample time: [1.0s, 0.7s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID16();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID17 */
void SLUGS2_step17(void)               /* Sample time: [2.0s, 0.6s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID17();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID18 */
void SLUGS2_step18(void)               /* Sample time: [2.0s, 0.7s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID18();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID19 */
void SLUGS2_step19(void)               /* Sample time: [2.0s, 0.8s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID19();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID20 */
void SLUGS2_step20(void)               /* Sample time: [2.0s, 0.9s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  SLUGS2_Mavlink_TX_AdapterTID20();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model initialize function */
void SLUGS2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)SLUGS2_M, 0,
                sizeof(RT_MODEL_SLUGS2_T));
  (SLUGS2_M)->Timing.TaskCounters.cLimit[0] = 1;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[1] = 2;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[2] = 10;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[3] = 10;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[4] = 10;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[5] = 10;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[6] = 10;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[7] = 20;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[8] = 50;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[9] = 50;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[10] = 50;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[11] = 50;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[12] = 50;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[13] = 100;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[14] = 100;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[15] = 100;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[16] = 100;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[17] = 200;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[18] = 200;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[19] = 200;
  (SLUGS2_M)->Timing.TaskCounters.cLimit[20] = 200;
  rtmSetFirstInitCond(SLUGS2_M, 1);

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[3] = 9;/* Sample time: [0.1s, 0.01s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[4] = 5;/* Sample time: [0.1s, 0.05s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[5] = 4;/* Sample time: [0.1s, 0.06s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[6] = 2;/* Sample time: [0.1s, 0.08s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[7] = 19;/* Sample time: [0.2s, 0.01s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[9] = 49;/* Sample time: [0.5s, 0.01s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[10] = 40;/* Sample time: [0.5s, 0.1s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[11] = 30;/* Sample time: [0.5s, 0.2s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[12] = 20;/* Sample time: [0.5s, 0.3s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[13] = 90;/* Sample time: [1.0s, 0.1s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[14] = 60;/* Sample time: [1.0s, 0.4s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[15] = 50;/* Sample time: [1.0s, 0.5s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[16] = 30;/* Sample time: [1.0s, 0.7s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[17] = 140;/* Sample time: [2.0s, 0.6s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[18] = 130;/* Sample time: [2.0s, 0.7s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[19] = 120;/* Sample time: [2.0s, 0.8s] */

  /* initialize sample time offsets */
  SLUGS2_M->Timing.TaskCounters.TID[20] = 110;/* Sample time: [2.0s, 0.9s] */

  /* block I/O */
  (void) memset(((void *) &SLUGS2_B), 0,
                sizeof(BlockIO_SLUGS2_T));

  /* states (dwork) */
  (void) memset((void *)&SLUGS2_DWork, 0,
                sizeof(D_Work_SLUGS2_T));

  /* exported global states */
  mlParamInterface = SLUGS2_rtZpi_struct;
  mlWpValues = SLUGS2_rtZmavlink_mission_item_values_t;
  mlAttitudeData = SLUGS2_rtZmavlink_attitude_t;
  mlAttitudeSol = SLUGS2_rtZmavlink_attitude_t;
  mlGpsData = SLUGS2_rtZmavlink_gps_raw_int_t;
  mlLocalPositionData = SLUGS2_rtZmavlink_local_position_ned_t;
  mlNavigation = SLUGS2_rtZmavlink_slugs_navigation_t;
  mlSysStatus = SLUGS2_rtZmavlink_sys_status_t;
  mlPilotConsoleData = SLUGS2_rtZmavlink_rc_channels_raw_t;
  mlRawImuData = SLUGS2_rtZmavlink_raw_imu_t;
  mlPwmCommands = SLUGS2_rtZmavlink_servo_output_raw_t;
  mlVfr_hud = SLUGS2_rtZmavlink_vfr_hud_t;
  mlAirData = SLUGS2_rtZmavlink_scaled_pressure_t;
  mlGSLocationFloat = SLUGS2_rtZmavlink_coordinate_float_t;
  mlHeartbeatLocal = SLUGS2_rtZmavlink_heartbeat_t;
  mlISR = SLUGS2_rtZmavlink_isr_location_t;
  mlMidLevelCommands = SLUGS2_rtZmavlink_mid_lvl_cmds_t;
  mlMobileLocation = SLUGS2_rtZmavlink_slugs_mobile_location_t;
  mlRawPressureData = SLUGS2_rtZmavlink_raw_pressure_t;
  mlVISensor = SLUGS2_rtZmavlink_volt_sensor_t;
  MPU_T = 0U;

  /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

  /* Start for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  Sensor_Data_Adapter_Start();

  /* End of Start for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Start for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitud_Start();

  /* End of Start for SubSystem: '<Root>/Position_and_Attitude_Filter' */
#ifndef WIN

  /* Start for S-Function (MCHP_IC): '<S644>/Input Capture' */
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
  IC8CON1 = 0x1003;
  IC8CON2 = 0x00;

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
  _IC8IF = 0;
  _IC8IP = 6;
  _IC8IE = 1;
#endif
  /* Start for InitialCondition: '<S646>/IC1' */
  SLUGS2_DWork.IC1_FirstOutputTime = true;

  /* Start for Atomic SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */
  /* Start for InitialCondition: '<S182>/IC1' */
  SLUGS2_DWork.IC1_FirstOutputTime_m = true;

  /* Start for InitialCondition: '<S182>/IC2' */
  SLUGS2_DWork.IC2_FirstOutputTime = true;

  /* Start for InitialCondition: '<S182>/IC4' */
  SLUGS2_DWork.IC4_FirstOutputTime = true;

  /* Start for InitialCondition: '<S179>/IC' */
  SLUGS2_DWork.IC_FirstOutputTime = true;

  /* InitializeConditions for IfAction SubSystem: '<S173>/RTB//Follow Mobile Navigation' */
  /* InitializeConditions for DiscreteIntegrator: '<S436>/Discrete-Time Integrator' */
  SLUGS2_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;

  /* End of InitializeConditions for SubSystem: '<S173>/RTB//Follow Mobile Navigation' */

  /* Start for IfAction SubSystem: '<S173>/Normal WP  Navigation' */

  /* Start for InitialCondition: '<S180>/IC' */
  SLUGS2_B.IC = 0U;
  SLUGS2_DWork.IC_FirstOutputTime_m = true;

  /* InitializeConditions for Enabled SubSystem: '<S180>/Get Frenet' */
  /* InitializeConditions for Delay: '<S296>/Integer Delay1'
   *
   * Block description for '<S296>/Integer Delay1':
   *  TODO: issue with the generated code
   */
  SLUGS2_DWork.IntegerDelay1_DSTATE_gn = 166.3F;

  /* End of InitializeConditions for SubSystem: '<S180>/Get Frenet' */
  /* InitializeConditions for IfAction SubSystem: '<S173>/Normal WP  Navigation' */
  /* InitializeConditions for Delay: '<S180>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_im = 1U;

  /* InitializeConditions for MATLAB Function: '<S180>/Embedded MATLAB Function' */
  SLUGS2_DWork.persistentDidReachIP = 0U;

  /* InitializeConditions for MATLAB Function: '<S180>/computeCurrentWP' */
  SLUGS2_DWork.fromWp = 1U;
  SLUGS2_DWork.toWp = 2U;

  /* End of InitializeConditions for SubSystem: '<S173>/Normal WP  Navigation' */

  /* Start for IfAction SubSystem: '<S173>/Normal WP  Navigation' */
  /* VirtualOutportStart for Outport: '<S180>/h_c' */
  SLUGS2_B.Merge1 = 166.3F;

  /* VirtualOutportStart for Outport: '<S180>/FromWP' */
  SLUGS2_B.WP0 = 1U;

  /* VirtualOutportStart for Outport: '<S180>/ToWP' */
  SLUGS2_B.WP1 = 2U;

  /* End of Start for SubSystem: '<S173>/Normal WP  Navigation' */
  /* VirtualOutportStart for Outport: '<S178>/FromWP' */
  SLUGS2_B.Merge3 = 1U;

  /* VirtualOutportStart for Outport: '<S178>/ToWP' */
  SLUGS2_B.Merge4 = 2U;

  /* End of Start for SubSystem: '<S173>/Line Segment' */

  /* InitializeConditions for IfAction SubSystem: '<S173>/Circle Navigation' */
  /* InitializeConditions for Delay: '<S191>/Integer Delay1'
   *
   * Block description for '<S191>/Integer Delay1':
   *  TODO: check that it pull the right entry
   */
  SLUGS2_DWork.IntegerDelay1_DSTATE_a = 166.3F;

  /* End of InitializeConditions for SubSystem: '<S173>/Circle Navigation' */

  /* Start for IfAction SubSystem: '<S173>/Circle Navigation' */
  /* VirtualOutportStart for Outport: '<S174>/FromWP' */
  SLUGS2_B.Merge3 = 1U;

  /* VirtualOutportStart for Outport: '<S174>/ToWP' */
  SLUGS2_B.Merge4 = 1U;

  /* End of Start for SubSystem: '<S173>/Circle Navigation' */
  /* Start for InitialCondition: '<S182>/IC3' */
  SLUGS2_B.IC3 = 166.3F;
  SLUGS2_DWork.IC3_FirstOutputTime = true;

  /* End of Start for SubSystem: '<S4>/Navigation Encaps [updated 4.28.16]' */

  /* Start for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for InitialCondition: '<S72>/IC' */
  SLUGS2_DWork.IC_FirstOutputTime_g = true;

  /* Start for InitialCondition: '<S76>/IC' */
  SLUGS2_DWork.IC_FirstOutputTime_f = true;

  /* End of Start for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */

  /* InitializeConditions for Enabled SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
  /* InitializeConditions for UnitDelay: '<S82>/UD'
   *
   * Block description for '<S82>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE_ff = 0.0F;

  /* InitializeConditions for Delay: '<S72>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_an = 0.0F;

  /* InitializeConditions for Delay: '<S72>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_i = 0.0F;

  /* InitializeConditions for Delay: '<S74>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_fs = 0.0F;

  /* InitializeConditions for UnitDelay: '<S92>/UD'
   *
   * Block description for '<S92>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE_j = 0.0F;

  /* InitializeConditions for Delay: '<S76>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_k = 0.0F;

  /* InitializeConditions for Delay: '<S76>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_b = 0.0F;

  /* InitializeConditions for Delay: '<S62>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_i = 0.0F;

  /* InitializeConditions for Delay: '<S62>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_e = 0.0F;

  /* InitializeConditions for Merge: '<S73>/Merge' */
  if (rtmIsFirstInitCond(SLUGS2_M)) {
    SLUGS2_B.Merge_oo = 0.0F;
  }

  /* End of InitializeConditions for Merge: '<S73>/Merge' */

  /* InitializeConditions for Delay: '<S75>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_h = 0.0F;

  /* InitializeConditions for Delay: '<S77>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_p = 0.0F;

  /* InitializeConditions for Delay: '<S77>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_ey[0] = 0.0F;
  SLUGS2_DWork.IntegerDelay1_DSTATE_ey[1] = 0.0F;

  /* InitializeConditions for Delay: '<S77>/Integer Delay2' */
  SLUGS2_DWork.IntegerDelay2_DSTATE_g = 0.0F;

  /* InitializeConditions for Delay: '<S93>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_nz = 0.0F;

  /* InitializeConditions for Delay: '<S94>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_hh = 0.0F;

  /* End of InitializeConditions for SubSystem: '<S4>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for InitialCondition: '<S111>/IC' */
  SLUGS2_DWork.IC_FirstOutputTime_b = true;

  /* InitializeConditions for Enabled SubSystem: '<S95>/Sideslip Compensation' */
  /* InitializeConditions for UnitDelay: '<S119>/UD'
   *
   * Block description for '<S119>/UD':
   *
   *  Store in Global RAM
   */
  SLUGS2_DWork.UD_DSTATE_f = 0.0F;

  /* InitializeConditions for Delay: '<S111>/Integer Delay' */
  SLUGS2_DWork.IntegerDelay_DSTATE_ed = 0.0F;

  /* InitializeConditions for Delay: '<S111>/Integer Delay1' */
  SLUGS2_DWork.IntegerDelay1_DSTATE_n = 0.0F;

  /* InitializeConditions for Delay: '<S112>/Integer Delay3' */
  SLUGS2_DWork.IntegerDelay3_DSTATE_ph = 0.0F;

  /* End of InitializeConditions for SubSystem: '<S95>/Sideslip Compensation' */

#ifndef WIN
  /* Start for S-Function (MCHP_OC_HW): '<S16>/Output Compare - HW Drive Servo motor' */
  /* OCxCON1[4108, 4108, 4108, 4108]   PulseTrig1*/
  OC1CON2 = 0x9F;
  OC1RS = 0x6689;
  OC1R = 1;
  OC2CON2 = 0x9F;
  OC2RS = 0x6689;
  OC2R = 1;
  OC3CON2 = 0x9F;
  OC3RS = 0x6689;
  OC3R = 1;
  OC4CON2 = 0x9F;
  OC4RS = 0x6689;
  OC4R = 1;

  /* Start for S-Function (MCHP_MCU_LOAD): '<Root>/MCU Load' */
  TMR3 = 0;                            /* Initialize Timer 3 Value to 0.  Timer 3 is enabled only when the mcu is not idle */

  /* Start for Atomic SubSystem: '<Root>/Barometer_Driver' */
  SLU_Barometer_Driver_Start();

  /* End of Start for SubSystem: '<Root>/Barometer_Driver' */

  /* Start for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  SLUGS_IMU_Mag_Driver_Start();

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
#endif

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  SLUGS2_DWork.GS_INIT_FLAG = 1.0;
#ifndef WIN
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
  _U1TXIP = 5;                         /*  Tx Interrupt priority set to 5 */
  _U1TXIF = 0;                         /*  */
  _U1TXIE = 1;                         /* Enable Interrupt */

  /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
  /* Initialisation sequence for UART 4 */
  {
    const uint8_T InitSequence[11] = { 72, 101, 108, 108, 111, 32, 87, 111, 114,
      108, 100 };

    U4BRG = 0x9B5C;                    /* Baud rate: 110 (-0.00%) */
    U4MODE = 0x8000;
    U4STA = 0x2400;
    __delay32(1909091);                /* Wait for 1909091 cycles */

    {
      uint_T i1;
      for (i1 = 0; i1 < 11 ; i1++) {
        while (U4STAbits.UTXBF == 1) ; /* Wait for one empty space within buffer UART */
        U4TXREG = InitSequence[i1];
      }
    }

    while (U4STAbits.TRMT == 0) ;      /* Wait for all value to be sent */
    U4MODE = 0;                        /* Then switch off UART */
  }
#ifdef SLUGS2 //for different GPS reciever
  U4BRG = 0x71;                        /* Baud rate: 38400 (-0.06%) */
  U4MODE = 0x8000;
#else 
  
  U4BRG = 0x0E3D;                      /* Baud rate: 4800 (-0.00%) */
  U4MODE = 0x8008;
  U4STA = 0x2400;
#endif
  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 5;                         /*  Tx Interrupt priority set to 5 */
  _U4TXIF = 0;                         /*  */
  _U4TXIE = 1;                         /* Enable Interrupt */
#endif

  /* InitializeConditions for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  S_Sensor_Data_Adapter_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* InitializeConditions for MATLAB Function: '<S648>/Buffer Failsafe Channel' */
  SLUGS_BufferICChannel_Init(&SLUGS2_DWork.sf_BufferFailsafeChannel);

  /* InitializeConditions for MATLAB Function: '<S643>/Buffer IC Channel' */
  SLUGS_BufferICChannel_Init(&SLUGS2_DWork.sf_BufferICChannel);

  /* InitializeConditions for MATLAB Function: '<S643>/Buffer IC Channel1' */
  SLUGS_BufferICChannel_Init(&SLUGS2_DWork.sf_BufferICChannel1);

  /* InitializeConditions for MATLAB Function: '<S643>/Buffer IC Channel2' */
  SLUGS_BufferICChannel_Init(&SLUGS2_DWork.sf_BufferICChannel2);

  /* InitializeConditions for MATLAB Function: '<S643>/Buffer IC Channel3' */
  SLUGS_BufferICChannel_Init(&SLUGS2_DWork.sf_BufferICChannel3);

  /* InitializeConditions for Atomic SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* InitializeConditions for MATLAB Function: '<S158>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_n);

  /* InitializeConditions for UnitDelay: '<S171>/FixPt Unit Delay2' */
  SLUGS2_DWork.FixPtUnitDelay2_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S134>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_i);

  /* End of InitializeConditions for SubSystem: '<S4>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for Atomic SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for MATLAB Function: '<S97>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_b0);

  /* End of InitializeConditions for SubSystem: '<S4>/Lateral Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for MATLAB Function: '<S653>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction);

  /* InitializeConditions for MATLAB Function: '<S654>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init(&SLUGS2_DWork.sf_EmbeddedMATLABFunction_b);
#ifndef WIN
  /* InitializeConditions for Atomic SubSystem: '<Root>/Barometer_Driver' */
  SLUG_Barometer_Driver_Init();
#endif
  /* End of InitializeConditions for SubSystem: '<Root>/Barometer_Driver' */

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(SLUGS2_M)) {
    rtmSetFirstInitCond(SLUGS2_M, 0);
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
