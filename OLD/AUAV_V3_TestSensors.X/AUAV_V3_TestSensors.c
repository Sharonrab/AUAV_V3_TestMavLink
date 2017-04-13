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
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
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
  0.0F,                                /* lat */
  0.0F,                                /* lon */
  0.0F                                 /* alt */
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

const mavlink_local_position_ned_t
  AUAV_V3_TestSensors_rtZmavlink_local_position_ned_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* x */
  0.0F,                                /* y */
  0.0F,                                /* z */
  0.0F,                                /* vx */
  0.0F,                                /* vy */
  0.0F                                 /* vz */
} ;                                    /* mavlink_local_position_ned_t ground */

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

/*
 * Output and update for atomic system:
 *    '<S174>/negprotect'
 *    '<S453>/negprotect'
 *    '<S311>/negprotect'
 *    '<S335>/negprotect'
 *    '<S342>/negprotect'
 *    '<S390>/negprotect'
 *    '<S397>/negprotect'
 *    '<S437>/negprotect'
 *    '<S296>/negprotect'
 *    '<S225>/negprotect'
 *    ...
 */
void AUAV_V3_TestS_negprotect_l(real32_T rtu_val,
  rtB_negprotect_AUAV_V3_Test_p_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Longitudinal Channel Encaps [updated 4.28.16]/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/negprotect': '<S176>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S176>:1:5' */
    /* '<S176>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S176>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S452>/Embedded MATLAB Function'
 *    '<S334>/Embedded MATLAB Function'
 *    '<S341>/Embedded MATLAB Function'
 *    '<S389>/Embedded MATLAB Function'
 *    '<S396>/Embedded MATLAB Function'
 *    '<S436>/Embedded MATLAB Function'
 *    '<S295>/Embedded MATLAB Function'
 *    '<S224>/Embedded MATLAB Function'
 *    '<S237>/Embedded MATLAB Function'
 *    '<S250>/Embedded MATLAB Function'
 *    ...
 */
void A_EmbeddedMATLABFunction_b(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_b_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S454>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S454>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Output and update for atomic system:
 *    '<S188>/Zero out Z1'
 *    '<S303>/Zero out Z1'
 *    '<S303>/Zero out Z2'
 *    '<S303>/Zero out Z3'
 *    '<S187>/Zero out Z2'
 *    '<S198>/Zero out Z2'
 *    '<S277>/Zero out Z1'
 *    '<S180>/Zero out Z1'
 *    '<S180>/Zero out Z2'
 */
void AUAV_V3_TestSens_ZerooutZ1(const real32_T rtu_Pin[3],
  rtB_ZerooutZ1_AUAV_V3_TestSen_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/RTB//Follow Mobile Navigation/Zero out Z1': '<S445>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S445>:1:5' */
  localB->P[0] = rtu_Pin[0];
  localB->P[1] = rtu_Pin[1];
  localB->P[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S316>/Embedded MATLAB Function'
 *    '<S309>/Embedded MATLAB Function'
 *    '<S221>/Embedded MATLAB Function'
 *    '<S234>/Embedded MATLAB Function'
 */
void A_EmbeddedMATLABFunction_k(const real32_T rtu_x[3], const real32_T rtu_y[3],
  rtB_EmbeddedMATLABFunction_i_T *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S317>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S317>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_y[0] + rtu_x[1] * rtu_y[1]) + rtu_x[2] *
    rtu_y[2];
}

/*
 * Output and update for atomic system:
 *    '<S324>/Select N  Terms'
 *    '<S382>/Select N  Terms'
 */
void AUAV_V3_TestS_SelectNTerms(const real32_T rtu_T[3],
  rtB_SelectNTerms_AUAV_V3_Test_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms': '<S332>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S332>:1:5' */
  localB->N[0] = -rtu_T[1];
  localB->N[1] = rtu_T[0];
  localB->N[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S212>/negprotect3'
 *    '<S213>/negprotect3'
 *    '<S200>/negprotect1'
 *    '<S200>/negprotect2'
 *    '<S269>/negprotect'
 *    '<S270>/negprotect'
 */
void AUAV_V3_TestSe_negprotect3(real32_T rtu_val,
  rtB_negprotect3_AUAV_V3_TestS_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3': '<S223>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.0001F) {
    /* '<S223>:1:5' */
    /* '<S223>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S223>:1:8' */
    localB->zpVal = 0.0001F;
  }
}

/*
 * Initial conditions for atomic system:
 *    '<S655>/Buffer IC Channel'
 *    '<S655>/Buffer IC Channel1'
 *    '<S655>/Buffer IC Channel2'
 *    '<S655>/Buffer IC Channel3'
 *    '<S660>/Buffer Failsafe Channel'
 */
void AUAV__BufferICChannel_Init(rtDW_BufferICChannel_AUAV_V3__T *localDW)
{
  int16_T i;
  for (i = 0; i < 7; i++) {
    localDW->oldValues[i] = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S655>/Buffer IC Channel'
 *    '<S655>/Buffer IC Channel1'
 *    '<S655>/Buffer IC Channel2'
 *    '<S655>/Buffer IC Channel3'
 *    '<S660>/Buffer Failsafe Channel'
 */
void AUAV_V3_Te_BufferICChannel(uint16_T rtu_latest,
  rtB_BufferICChannel_AUAV_V3_T_T *localB, rtDW_BufferICChannel_AUAV_V3__T
  *localDW)
{
  int16_T i;

  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Bufffer IC/Buffer IC Channel': '<S661>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S661>:1:11' */
  for (i = 0; i < 7; i++) {
    localB->history[i] = 0U;
  }

  /* '<S661>:1:13' */
  for (i = 0; i < 6; i++) {
    /* '<S661>:1:13' */
    /* '<S661>:1:14' */
    localDW->oldValues[6 - i] = localDW->oldValues[5 - i];

    /* '<S661>:1:15' */
    localB->history[6 - i] = localDW->oldValues[5 - i];

    /* '<S661>:1:13' */
  }

  /* '<S661>:1:18' */
  localDW->oldValues[0] = rtu_latest;

  /* '<S661>:1:19' */
  localB->history[0] = rtu_latest;
}

/*
 * Output and update for atomic system:
 *    '<S688>/myMux Fun1'
 *    '<S687>/myMux Fun1'
 */
void AUAV_V3_TestSe_myMuxFun1_d(uint16_T rtu_u1, uint16_T rtu_u2, uint16_T
  rtu_u3, uint16_T rtu_u4, uint16_T rty_y[4])
{
  /* MATLAB Function 'Update PWM Commands and Send Telemetry [updated 4.27.16]/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/myMux Fun1': '<S698>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S698>:1:5' */
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
void AUAV_V3_TestSens_myMuxFun3(const real32_T rtu_u1[3], const real32_T rtu_u2
  [3], rtB_myMuxFun3_AUAV_V3_TestSen_T *localB)
{
  /* MATLAB Function 'myMux Fun3': '<S24>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S24>:1:5' */
  localB->y[0] = rtu_u1[0];
  localB->y[1] = rtu_u1[1];
  localB->y[2] = rtu_u1[2];
  localB->y[3] = rtu_u2[0];
  localB->y[4] = rtu_u2[1];
  localB->y[5] = rtu_u2[2];
}

/* Model step function for TID0 */
void AUAV_V3_TestSensors_step0(void)   /* Sample time: [0.01s, 0.0s] */
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
  real32_T rtb_Merge_m[3];
  real32_T rtb_Ze_b;
  real32_T rtb_cosphi;
  real32_T rtb_Sum1_mj;
  real32_T rtb_Deg2R1;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_RhhcosphicoslambXe;
  real32_T rtb_Product;
  real32_T rtb_Switch3_kh;
  uint16_T rtb_Switch1_m;
  uint16_T rtb_Switch2;
  uint16_T rtb_Switch3_fq;
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
  real_T tmp_5;
  real_T tmp_6;
  real_T tmp_7;
  uint64m_T tmp_8;
  uint64m_T tmp_9;
  uint32_T tmp_a;

  /* Update the flag to indicate when data transfers from
   *  Sample time: [0.01s, 0.0s] to Sample time: [0.02s, 0.0s]  */
  (AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1)++;
  if ((AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1) > 1) {
    AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1 = 0;
  }

  /* Gain: '<S699>/Unit Conversion' incorporates:
   *  DataStoreRead: '<S21>/PAR_CONFIG_ROLL_R PAR_CONFIG_PITCH_R PAR_CONFIG_YAW_R'
   */
  rtb_Product3_j0[0] = 0.0174532924F * mlParamInterface.param[15L];
  rtb_Product3_j0[1] = 0.0174532924F * mlParamInterface.param[16L];
  rtb_Product3_j0[2] = 0.0174532924F * mlParamInterface.param[17L];

  /* Trigonometry: '<S702>/SinCos' */
  rtb_Product2_c[0] = (real32_T)cos(rtb_Product3_j0[0]);
  rtb_Product3_j0[0] = (real32_T)sin(rtb_Product3_j0[0]);
  rtb_Product2_c[1] = (real32_T)cos(rtb_Product3_j0[1]);
  rtb_Product3_j0[1] = (real32_T)sin(rtb_Product3_j0[1]);
  rtb_RhhcosphisinlambYe = (real32_T)cos(rtb_Product3_j0[2]);
  rtb_Sum1_mj = (real32_T)sin(rtb_Product3_j0[2]);

  /* Product: '<S709>/u(5)*u(6)' incorporates:
   *  Trigonometry: '<S702>/SinCos'
   */
  rtb_VectorConcatenate[0] = rtb_Product2_c[1] * rtb_RhhcosphisinlambYe;

  /* Sum: '<S712>/Sum' incorporates:
   *  Product: '<S712>/u(3)*u(4)'
   *  Product: '<S712>/u(6)*u(1)*u(2)'
   *  Trigonometry: '<S702>/SinCos'
   */
  rtb_VectorConcatenate[1] = rtb_RhhcosphisinlambYe * rtb_Product3_j0[0] *
    rtb_Product3_j0[1] - rtb_Sum1_mj * rtb_Product2_c[0];

  /* Sum: '<S715>/Sum' incorporates:
   *  Product: '<S715>/u(1)*u(3)'
   *  Product: '<S715>/u(2)*u(4)*u(6)'
   *  Trigonometry: '<S702>/SinCos'
   */
  rtb_VectorConcatenate[2] = rtb_Product3_j0[1] * rtb_Product2_c[0] *
    rtb_RhhcosphisinlambYe + rtb_Product3_j0[0] * rtb_Sum1_mj;

  /* Product: '<S710>/u(3)*u(5)' */
  rtb_VectorConcatenate[3] = rtb_Sum1_mj * rtb_Product2_c[1];

  /* Sum: '<S713>/Sum' incorporates:
   *  Product: '<S713>/u(1)*u(2)*u(3)'
   *  Product: '<S713>/u(4)*u(6)'
   *  Trigonometry: '<S702>/SinCos'
   */
  rtb_VectorConcatenate[4] = rtb_Product3_j0[0] * rtb_Product3_j0[1] *
    rtb_Sum1_mj + rtb_Product2_c[0] * rtb_RhhcosphisinlambYe;

  /* Sum: '<S716>/Sum' incorporates:
   *  Product: '<S716>/u(1)*u(6)'
   *  Product: '<S716>/u(2)*u(3)*u(4)'
   *  Trigonometry: '<S702>/SinCos'
   */
  rtb_VectorConcatenate[5] = rtb_Product3_j0[1] * rtb_Sum1_mj * rtb_Product2_c[0]
    - rtb_RhhcosphisinlambYe * rtb_Product3_j0[0];

  /* Gain: '<S711>/Gain2' */
  rtb_VectorConcatenate[6] = -rtb_Product3_j0[1];

  /* Product: '<S714>/u(1)*u(3)' */
  rtb_VectorConcatenate[7] = rtb_Product3_j0[0] * rtb_Product2_c[1];

  /* Product: '<S717>/u(4)*u(5)' */
  rtb_VectorConcatenate[8] = rtb_Product2_c[0] * rtb_Product2_c[1];

  /* Math: '<S21>/Math Function' */
  for (i = 0; i < 3; i++) {
    rtb_MathFunction[3 * i] = rtb_VectorConcatenate[i];
    rtb_MathFunction[1 + 3 * i] = rtb_VectorConcatenate[i + 3];
    rtb_MathFunction[2 + 3 * i] = rtb_VectorConcatenate[i + 6];
  }

  /* End of Math: '<S21>/Math Function' */

  /* DataStoreRead: '<Root>/Data Store Read' */
  AUAV_V3_TestSensors_B.DataStoreRead =
    AUAV_V3_TestSensors_DWork.X_PLANE_HIL_FLAG;

  /* Outputs for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  AUAV_V_Sensor_Data_Adapter();

  /* End of Outputs for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Outputs for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Filt();

  /* End of Outputs for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* MATLAB Function: '<Root>/myMux Fun2' incorporates:
   *  SignalConversion: '<S23>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'myMux Fun2': '<S23>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S23>:1:5' */
  AUAV_V3_TestSensors_B.y_k[0] = AUAV_V3_TestSensors_B.DataTypeConversion_m[2];
  AUAV_V3_TestSensors_B.y_k[1] = AUAV_V3_TestSensors_B.DataTypeConversion_m[1];
  AUAV_V3_TestSensors_B.y_k[2] =
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_lt.y;
  AUAV_V3_TestSensors_B.y_k[3] = AUAV_V3_TestSensors_B.GyroErr[0];
  AUAV_V3_TestSensors_B.y_k[4] = AUAV_V3_TestSensors_B.GyroErr[1];
  AUAV_V3_TestSensors_B.y_k[5] = AUAV_V3_TestSensors_B.GyroErr[2];

  /* Trigonometry: '<S703>/SinCos' */
  rtb_Product2_c[0] = (real32_T)sin(AUAV_V3_TestSensors_B.y_k[0]);
  rtb_Product3_j0[0] = (real32_T)cos(AUAV_V3_TestSensors_B.y_k[0]);
  rtb_Product2_c[1] = (real32_T)sin(AUAV_V3_TestSensors_B.y_k[1]);
  rtb_Product3_j0[1] = (real32_T)cos(AUAV_V3_TestSensors_B.y_k[1]);
  rtb_Product2_c[2] = (real32_T)sin(AUAV_V3_TestSensors_B.y_k[2]);
  rtb_Product3_j0[2] = (real32_T)cos(AUAV_V3_TestSensors_B.y_k[2]);

  /* Product: '<S719>/u(5)*u(6)' */
  rtb_VectorConcatenate_m[0] = rtb_Product3_j0[1] * rtb_Product3_j0[2];

  /* Sum: '<S722>/Sum' incorporates:
   *  Product: '<S722>/u(3)*u(4)'
   *  Product: '<S722>/u(6)*u(1)*u(2)'
   */
  rtb_VectorConcatenate_m[1] = rtb_Product3_j0[2] * rtb_Product2_c[0] *
    rtb_Product2_c[1] - rtb_Product2_c[2] * rtb_Product3_j0[0];

  /* Sum: '<S725>/Sum' incorporates:
   *  Product: '<S725>/u(1)*u(3)'
   *  Product: '<S725>/u(2)*u(4)*u(6)'
   */
  rtb_VectorConcatenate_m[2] = rtb_Product2_c[1] * rtb_Product3_j0[0] *
    rtb_Product3_j0[2] + rtb_Product2_c[0] * rtb_Product2_c[2];

  /* Product: '<S720>/u(3)*u(5)' */
  rtb_VectorConcatenate_m[3] = rtb_Product2_c[2] * rtb_Product3_j0[1];

  /* Sum: '<S723>/Sum' incorporates:
   *  Product: '<S723>/u(1)*u(2)*u(3)'
   *  Product: '<S723>/u(4)*u(6)'
   */
  rtb_VectorConcatenate_m[4] = rtb_Product2_c[0] * rtb_Product2_c[1] *
    rtb_Product2_c[2] + rtb_Product3_j0[0] * rtb_Product3_j0[2];

  /* Sum: '<S726>/Sum' incorporates:
   *  Product: '<S726>/u(1)*u(6)'
   *  Product: '<S726>/u(2)*u(3)*u(4)'
   */
  rtb_VectorConcatenate_m[5] = rtb_Product2_c[1] * rtb_Product2_c[2] *
    rtb_Product3_j0[0] - rtb_Product3_j0[2] * rtb_Product2_c[0];

  /* Gain: '<S721>/Gain2' */
  rtb_VectorConcatenate_m[6] = -rtb_Product2_c[1];

  /* Product: '<S724>/u(1)*u(3)' */
  rtb_VectorConcatenate_m[7] = rtb_Product2_c[0] * rtb_Product3_j0[1];

  /* Product: '<S727>/u(4)*u(5)' */
  rtb_VectorConcatenate_m[8] = rtb_Product3_j0[0] * rtb_Product3_j0[1];

  /* Product: '<S21>/Product' */
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

  /* End of Product: '<S21>/Product' */

  /* Gain: '<S708>/Gain1' incorporates:
   *  Selector: '<S708>/Selector1'
   */
  rtb_VectorConcatenate_h[0] = rtb_VectorConcatenate[3];
  rtb_VectorConcatenate_h[1] = rtb_VectorConcatenate[0];
  rtb_VectorConcatenate_h[2] = -rtb_VectorConcatenate[6];

  /* Gain: '<S708>/Gain2' incorporates:
   *  Selector: '<S708>/Selector2'
   */
  rtb_VectorConcatenate_h[3] = rtb_VectorConcatenate[7];
  rtb_VectorConcatenate_h[4] = rtb_VectorConcatenate[8];

  /* Gain: '<S708>/Gain3' incorporates:
   *  Selector: '<S708>/Selector3'
   */
  rtb_VectorConcatenate_h[5] = -rtb_VectorConcatenate[1];
  rtb_VectorConcatenate_h[6] = rtb_VectorConcatenate[4];

  /* If: '<S700>/If' incorporates:
   *  Gain: '<S708>/Gain1'
   *  Selector: '<S708>/Selector1'
   */
  if ((-rtb_VectorConcatenate[6] >= 1.0F) || (-rtb_VectorConcatenate[6] <= -1.0F))
  {
    /* Outputs for IfAction SubSystem: '<S700>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S707>/Action Port'
     */
    AUAV_V3_Test_AxisRotZeroR3(rtb_VectorConcatenate_h, &rtb_Merge_m[0],
      &rtb_Merge_m[1], &rtb_Merge_m[2]);

    /* End of Outputs for SubSystem: '<S700>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S700>/AxisRotDefault' incorporates:
     *  ActionPort: '<S706>/Action Port'
     */
    AUAV_V3_Tes_AxisRotDefault(rtb_VectorConcatenate_h, &rtb_Merge_m[0],
      &rtb_Merge_m[1], &rtb_Merge_m[2]);

    /* End of Outputs for SubSystem: '<S700>/AxisRotDefault' */
  }

  /* End of If: '<S700>/If' */

  /* MATLAB Function: '<S21>/Embedded MATLAB Function1' */
  A_EmbeddedMATLABFunction_l(rtb_Merge_m[0],
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S21>/myMux Fun1' */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun1': '<S704>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S704>:1:5' */
  rtb_y_jl_idx_0 = rtb_Merge_m[2];
  rtb_y_jl_idx_1 = rtb_Merge_m[1];

  /* DataStoreWrite: '<S20>/mlAttitudeEuler' incorporates:
   *  DataStoreRead: '<S20>/Get time3'
   *  MATLAB Function: '<S21>/myMux Fun1'
   */
  mlAttitudeSol.roll = rtb_Merge_m[2];
  mlAttitudeSol.pitch = rtb_Merge_m[1];
  mlAttitudeSol.yaw = AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1.y;
  mlAttitudeSol.time_boot_ms = AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* DataTypeConversion: '<S682>/Data Type Conversion1' incorporates:
   *  DataStoreRead: '<S20>/mlHeartbeatLocal'
   */
  rtb_DataTypeConversion1_hq = (uint8_T)mlHeartbeatLocal.custom_mode;

  /* S-Function "dsPIC_PWM_IC" Block: <S656>/Input Capture */
  AUAV_V3_TestSensors_B.InputCapture_o1 = MCHP_ic1up;
  AUAV_V3_TestSensors_B.InputCapture_o2 = MCHP_ic2up;
  AUAV_V3_TestSensors_B.InputCapture_o3 = MCHP_ic3up;
  AUAV_V3_TestSensors_B.InputCapture_o4 = MCHP_ic4up;
  AUAV_V3_TestSensors_B.InputCapture_o5 = MCHP_ic5up;
  AUAV_V3_TestSensors_B.InputCapture_o6 = MCHP_ic6up;
  AUAV_V3_TestSensors_B.InputCapture_o7 = MCHP_ic7up;
  AUAV_V3_TestSensors_B.InputCapture_o8 = MCHP_ic8up;

  /* MATLAB Function: '<S660>/Buffer Failsafe Channel' incorporates:
   *  DataTypeConversion: '<S657>/Data Type Conversion'
   *  Gain: '<S657>/Convert to  Microseconds'
   *  MATLAB Function: '<S656>/myMux Fun5'
   */
  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun5': '<S668>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S668>:1:5' */
  AUAV_V3_Te_BufferICChannel((uint16_T)(52429UL *
    AUAV_V3_TestSensors_B.InputCapture_o5 >> 18),
    &AUAV_V3_TestSensors_B.sf_BufferFailsafeChannel,
    &AUAV_V3_TestSensors_DWork.sf_BufferFailsafeChannel);

  /* S-Function (MCHP_C_function_Call): '<S660>/Choose the Median [navSupport.c] [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupdat = meanFilter5(
    &AUAV_V3_TestSensors_B.sf_BufferFailsafeChannel.history[0]
    );

  /* S-Function (MCHP_C_function_Call): '<S19>/Manual or Auto? [navSupport.c] [updated 4.28.16]' */
  AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 = isApManual(
    AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupdat
    );

  /* MATLAB Function: '<S655>/Buffer IC Channel' incorporates:
   *  MATLAB Function: '<S656>/myMux Fun5'
   */
  AUAV_V3_Te_BufferICChannel(AUAV_V3_TestSensors_B.InputCapture_o3,
    &AUAV_V3_TestSensors_B.sf_BufferICChannel,
    &AUAV_V3_TestSensors_DWork.sf_BufferICChannel);

  /* MATLAB Function: '<S655>/Buffer IC Channel1' incorporates:
   *  MATLAB Function: '<S656>/myMux Fun5'
   */
  AUAV_V3_Te_BufferICChannel(AUAV_V3_TestSensors_B.InputCapture_o1,
    &AUAV_V3_TestSensors_B.sf_BufferICChannel1,
    &AUAV_V3_TestSensors_DWork.sf_BufferICChannel1);

  /* MATLAB Function: '<S655>/Buffer IC Channel2' incorporates:
   *  MATLAB Function: '<S656>/myMux Fun5'
   */
  AUAV_V3_Te_BufferICChannel(AUAV_V3_TestSensors_B.InputCapture_o4,
    &AUAV_V3_TestSensors_B.sf_BufferICChannel2,
    &AUAV_V3_TestSensors_DWork.sf_BufferICChannel2);

  /* MATLAB Function: '<S655>/Buffer IC Channel3' incorporates:
   *  MATLAB Function: '<S656>/myMux Fun5'
   */
  AUAV_V3_Te_BufferICChannel(AUAV_V3_TestSensors_B.InputCapture_o2,
    &AUAV_V3_TestSensors_B.sf_BufferICChannel3,
    &AUAV_V3_TestSensors_DWork.sf_BufferICChannel3);

  /* Logic: '<S658>/Logical Operator' */
  rtb_IC1_a = !(AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 != 0);

  /* InitialCondition: '<S658>/IC1' */
  if (AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime) {
    AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime = false;
    rtb_IC1_a = false;
  }

  /* End of InitialCondition: '<S658>/IC1' */

  /* DataTypeConversion: '<S671>/Data Type Conversion1' */
  AUAV_V3_TestSensors_B.DataTypeConversion1_h = rtb_IC1_a;

  /* S-Function (MCHP_C_function_Call): '<S671>/C Function Call' */
  AUAV_V3_TestSensors_B.CFunctionCall = justEnabled(
    AUAV_V3_TestSensors_B.DataTypeConversion1_h
    , ((uint8_T)2U)
    );

  /* Outputs for Enabled SubSystem: '<S19>/Grab I.C.' incorporates:
   *  EnablePort: '<S659>/Enable'
   */
  if (AUAV_V3_TestSensors_B.CFunctionCall > 0) {
    /* S-Function (MCHP_C_function_Call): '<S659>/Choose the Median navSupport.c [updated 4.28.16]' */
    AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupd_a = meanFilter5(
      &AUAV_V3_TestSensors_B.sf_BufferICChannel.history[0]
      );

    /* Saturate: '<S659>/[0.55 0.68]' */
    if (AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupd_a > 14000U) {
      rtb_u2deg = 14000U;
    } else if (AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupd_a < 9386U) {
      rtb_u2deg = 9386U;
    } else {
      rtb_u2deg = AUAV_V3_TestSensors_B.ChoosetheMediannavSupportcupd_a;
    }

    /* End of Saturate: '<S659>/[0.55 0.68]' */

    /* DataTypeConversion: '<S672>/Data Type Conversion' incorporates:
     *  Gain: '<S672>/Convert to  Microseconds'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_hn = (uint16_T)(52429UL * rtb_u2deg
      >> 18);

    /* S-Function (MCHP_C_function_Call): '<S659>/update dT Trim updateControlMcu.c [updated 4.28.16]' */
    updatePWMTrim(
                  AUAV_V3_TestSensors_B.DataTypeConversion_hn
                  , ((uint8_T)0U)
                  );

    /* DataTypeConversion: '<S676>/Data Type Conversion' incorporates:
     *  Constant: '<S676>/Constant1'
     *  Constant: '<S676>/Constant2'
     *  Product: '<S676>/Divide'
     *  Sum: '<S676>/Add'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion = (real32_T)((real_T)rtb_u2deg *
      0.00013460761879122363 + -1.2634271099744254);

    /* S-Function (MCHP_C_function_Call): '<S659>/2' */
    AUAV_V3_TestSensors_B.u = meanFilter5(
      &AUAV_V3_TestSensors_B.sf_BufferICChannel1.history[0]
      );

    /* Saturate: '<S659>/[-2  2] deg' */
    if (AUAV_V3_TestSensors_B.u > 14000U) {
      rtb_u2deg = 14000U;
    } else if (AUAV_V3_TestSensors_B.u < 11000U) {
      rtb_u2deg = 11000U;
    } else {
      rtb_u2deg = AUAV_V3_TestSensors_B.u;
    }

    /* End of Saturate: '<S659>/[-2  2] deg' */

    /* DataTypeConversion: '<S673>/Data Type Conversion' incorporates:
     *  Gain: '<S673>/Convert to  Microseconds'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_e = (uint16_T)(52429UL * rtb_u2deg >>
      18);

    /* S-Function (MCHP_C_function_Call): '<S659>/update dA Trim updateControlMcu.c' */
    updatePWMTrim(
                  AUAV_V3_TestSensors_B.DataTypeConversion_e
                  , ((uint8_T)1U)
                  );

    /* DataTypeConversion: '<S677>/Data Type Conversion' incorporates:
     *  Constant: '<S677>/Constant1'
     *  Constant: '<S677>/Constant2'
     *  Product: '<S677>/Divide'
     *  Sum: '<S677>/Add'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_h = (real32_T)((real_T)rtb_u2deg *
      -7.1481057518468276E-5 + 0.94019034954041281);

    /* S-Function (MCHP_C_function_Call): '<S659>/3' */
    AUAV_V3_TestSensors_B.u_h = meanFilter5(
      &AUAV_V3_TestSensors_B.sf_BufferICChannel2.history[0]
      );

    /* Saturate: '<S659>/[-2  2] deg ' */
    if (AUAV_V3_TestSensors_B.u_h > 14000U) {
      rtb_u2deg = 14000U;
    } else if (AUAV_V3_TestSensors_B.u_h < 11000U) {
      rtb_u2deg = 11000U;
    } else {
      rtb_u2deg = AUAV_V3_TestSensors_B.u_h;
    }

    /* End of Saturate: '<S659>/[-2  2] deg ' */

    /* DataTypeConversion: '<S674>/Data Type Conversion' incorporates:
     *  Gain: '<S674>/Convert to  Microseconds'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_go = (uint16_T)(52429UL * rtb_u2deg
      >> 18);

    /* S-Function (MCHP_C_function_Call): '<S659>/update dR Trim updateControlMcu.c' */
    updatePWMTrim(
                  AUAV_V3_TestSensors_B.DataTypeConversion_go
                  , ((uint8_T)2U)
                  );

    /* DataTypeConversion: '<S678>/Data Type Conversion' incorporates:
     *  Constant: '<S678>/Constant1'
     *  Constant: '<S678>/Constant2'
     *  Product: '<S678>/Divide'
     *  Sum: '<S678>/Add'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_g = (real32_T)((real_T)rtb_u2deg *
      -9.5308076691291043E-5 + 1.2535871327205506);

    /* S-Function (MCHP_C_function_Call): '<S659>/4' */
    AUAV_V3_TestSensors_B.u_m = meanFilter5(
      &AUAV_V3_TestSensors_B.sf_BufferICChannel3.history[0]
      );

    /* Saturate: '<S659>/[-2  2] deg  ' */
    if (AUAV_V3_TestSensors_B.u_m > 14000U) {
      rtb_u2deg = 14000U;
    } else if (AUAV_V3_TestSensors_B.u_m < 11000U) {
      rtb_u2deg = 11000U;
    } else {
      rtb_u2deg = AUAV_V3_TestSensors_B.u_m;
    }

    /* End of Saturate: '<S659>/[-2  2] deg  ' */

    /* DataTypeConversion: '<S675>/Data Type Conversion' incorporates:
     *  Gain: '<S675>/Convert to  Microseconds'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_k = (uint16_T)(52429UL * rtb_u2deg >>
      18);

    /* S-Function (MCHP_C_function_Call): '<S659>/update dE Trim updateControlMcu.c' */
    updatePWMTrim(
                  AUAV_V3_TestSensors_B.DataTypeConversion_k
                  , ((uint8_T)3U)
                  );

    /* DataTypeConversion: '<S679>/Data Type Conversion' incorporates:
     *  Constant: '<S679>/Constant1'
     *  Constant: '<S679>/Constant2'
     *  Product: '<S679>/Divide'
     *  Sum: '<S679>/Add'
     */
    AUAV_V3_TestSensors_B.DataTypeConversion_f = (real32_T)((real_T)rtb_u2deg *
      4.7654038345645522E-5 + -0.62679356636027528);
  }

  /* End of Outputs for SubSystem: '<S19>/Grab I.C.' */

  /* Logic: '<S11>/Logical Operator' incorporates:
   *  Constant: '<S469>/Constant'
   *  Constant: '<S470>/Constant'
   *  Constant: '<S471>/Constant'
   *  DataStoreRead: '<Root>/isPassthrough'
   *  RelationalOperator: '<S469>/Compare'
   *  RelationalOperator: '<S470>/Compare'
   *  RelationalOperator: '<S471>/Compare'
   */
  rtb_IC1_a = (boolean_T)((mlHeartbeatLocal.custom_mode == 2UL) ^
    (mlHeartbeatLocal.custom_mode == 4UL)) ^ (mlHeartbeatLocal.custom_mode ==
    8UL);

  /* S-Function (MCHP_C_function_Call): '<Root>/Get RTB Order [navSupport.c] [updated 4.27.16]' */
  getRTB(
         &AUAV_V3_TestSensors_B.GetRTBOrdernavSupportcupdated42[0]
         );

  /* Outputs for Atomic SubSystem: '<S6>/Navigation Encaps [updated 4.28.16]' */
  /* DataTypeConversion: '<S72>/Data Type Conversion' incorporates:
   *  DataStoreRead: '<S72>/Get Nav Mode'
   */
  rtb_Compare_j = (uint8_T)mlHeartbeatLocal.custom_mode;

  /* InitialCondition: '<S189>/IC1' incorporates:
   *  DataStoreRead: '<Root>/isWpFly?'
   *  Logic: '<S180>/Logical Operator'
   *  Logic: '<S180>/Logical Operator1'
   */
  if (AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime_m) {
    AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime_m = false;
    rtb_IC1 = false;
  } else {
    rtb_IC1 = ((mlHeartbeatLocal.custom_mode != 0UL) &&
               (!(AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 != 0)));
  }

  /* End of InitialCondition: '<S189>/IC1' */

  /* DataTypeConversion: '<S459>/Data Type Conversion2' incorporates:
   *  Delay: '<S464>/Delay'
   *  Logic: '<S464>/Logical Operator'
   *  RelationalOperator: '<S464>/Relational Operator'
   */
  rtb_DataTypeConversion2_fi = (uint8_T)
    ((AUAV_V3_TestSensors_DWork.Delay_DSTATE_e != rtb_IC1) && rtb_IC1);

  /* InitialCondition: '<S189>/IC2' */
  if (AUAV_V3_TestSensors_DWork.IC2_FirstOutputTime) {
    AUAV_V3_TestSensors_DWork.IC2_FirstOutputTime = false;
    rtb_IC2 = 0U;
  } else {
    rtb_IC2 = rtb_DataTypeConversion2_fi;
  }

  /* End of InitialCondition: '<S189>/IC2' */

  /* Outputs for Enabled SubSystem: '<S189>/Grab Upon Enable' incorporates:
   *  EnablePort: '<S458>/Enable'
   */
  if (rtb_IC2 > 0) {
    /* SignalConversion: '<S461>/Numerical Unity' incorporates:
     *  DataStoreRead: '<S458>/Get GS Location'
     *  Gain: '<S458>/Gain'
     *  Gain: '<S458>/Gain1'
     *  Gain: '<S458>/Gain2'
     */
    AUAV_V3_TestSensors_B.NumericalUnity[0] = 0.001F * mlGSLocationFloat.alt;
    AUAV_V3_TestSensors_B.NumericalUnity[1] = 1.0E-7F * mlGSLocationFloat.lat;
    AUAV_V3_TestSensors_B.NumericalUnity[2] = 1.0E-7F * mlGSLocationFloat.lon;

    /* Gain: '<S462>/Deg2R' */
    rtb_cosphi = 0.0174532924F * AUAV_V3_TestSensors_B.NumericalUnity[1];

    /* Trigonometry: '<S462>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S462>/Sum1' incorporates:
     *  Constant: '<S462>/const'
     *  Product: '<S462>/Product1'
     *  Product: '<S462>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S462>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S462>/f' */

    /* Product: '<S462>/Rh' incorporates:
     *  Constant: '<S462>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S462>/Sum2' */
    rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_B.NumericalUnity[0] +
      rtb_Sum1_mj;

    /* Trigonometry: '<S462>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S462>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F * AUAV_V3_TestSensors_B.NumericalUnity[2];

    /* Product: '<S462>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S462>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S462>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S462>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S462>/Ze' incorporates:
     *  Product: '<S462>/Rh(1-e^2)'
     *  Sum: '<S462>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      AUAV_V3_TestSensors_B.NumericalUnity[0];

    /* DataTypeConversion: '<S458>/Data Type Conversion' */
    AUAV_V3_TestSensors_B.DataTypeConversion_c[0] =
      AUAV_V3_TestSensors_B.NumericalUnity[2];
    AUAV_V3_TestSensors_B.DataTypeConversion_c[1] =
      AUAV_V3_TestSensors_B.NumericalUnity[1];

    /* DataTypeConversion: '<S458>/Data Type Conversion1' */
    AUAV_V3_TestSensors_B.DataTypeConversion1_l[0] = rtb_RhhcosphicoslambXe;
    AUAV_V3_TestSensors_B.DataTypeConversion1_l[1] = rtb_RhhcosphisinlambYe;
    AUAV_V3_TestSensors_B.DataTypeConversion1_l[2] = rtb_Ze_b;
  }

  /* End of Outputs for SubSystem: '<S189>/Grab Upon Enable' */

  /* MATLAB Function: '<S180>/myMux Fun2' */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun2': '<S193>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S193>:1:5' */
  AUAV_V3_TestSensors_B.y_p[0] = AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
  AUAV_V3_TestSensors_B.y_p[1] = AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
  AUAV_V3_TestSensors_B.y_p[2] = AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

  /* S-Function (MCHP_C_function_Call): '<S180>/Diagnoistics Set navsupport.c [updated 5.1.16]1' */
  setDiagnosticFloat(
                     &AUAV_V3_TestSensors_B.y_p[0]
                     );

  /* InitialCondition: '<S189>/IC4' */
  if (AUAV_V3_TestSensors_DWork.IC4_FirstOutputTime) {
    AUAV_V3_TestSensors_DWork.IC4_FirstOutputTime = false;
    rtb_IC4_idx_0 = 36.9885063F;
    rtb_IC4_idx_1 = -122.055305F;
  } else {
    rtb_IC4_idx_0 = AUAV_V3_TestSensors_B.DataTypeConversion_c[0];
    rtb_IC4_idx_1 = AUAV_V3_TestSensors_B.DataTypeConversion_c[1];
  }

  /* End of InitialCondition: '<S189>/IC4' */

  /* MATLAB Function: '<S180>/Zero out Z2' */
  AUAV_V3_TestSens_ZerooutZ1(AUAV_V3_TestSensors_B.sf_myMuxFun1_e.y,
    &AUAV_V3_TestSensors_B.sf_ZerooutZ2);

  /* MATLAB Function: '<S180>/Zero out Z1' */
  AUAV_V3_TestSens_ZerooutZ1(AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y,
    &AUAV_V3_TestSensors_B.sf_ZerooutZ1);

  /* MATLAB Function: '<S263>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.sf_ZerooutZ1.P,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i1);

  /* MATLAB Function: '<S264>/negprotect' */
  AUAV_V3_TestS_negprotect_l
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i1.xDoty,
     &AUAV_V3_TestSensors_B.sf_negprotect_lq);

  /* S-Function (MCHP_C_function_Call): '<S264>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116 = mySqrt(
    AUAV_V3_TestSensors_B.sf_negprotect_lq.zpVal
    );

  /* InitialCondition: '<S186>/IC' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   */
  if (AUAV_V3_TestSensors_DWork.IC_FirstOutputTime) {
    AUAV_V3_TestSensors_DWork.IC_FirstOutputTime = false;
    rtb_RhhcosphisinlambYe = 1.5F;
  } else {
    rtb_RhhcosphisinlambYe = mlParamInterface.param[26];
  }

  /* End of InitialCondition: '<S186>/IC' */

  /* Saturate: '<S186>/[1.5 10]' */
  if (rtb_RhhcosphisinlambYe > 10.0F) {
    rtb_RhhcosphisinlambYe = 10.0F;
  } else {
    if (rtb_RhhcosphisinlambYe < 1.5F) {
      rtb_RhhcosphisinlambYe = 1.5F;
    }
  }

  /* Product: '<S186>/Product3' incorporates:
   *  Constant: '<S186>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S186>/Product1'
   *  Product: '<S186>/Product2'
   *  Saturate: '<S186>/[1.5 10]'
   */
  rtb_Product3_bo = mlMidLevelCommands.uCommand * mlMidLevelCommands.uCommand *
    rtb_RhhcosphisinlambYe / 4.57681F;

  /* If: '<S180>/Determine Overall Nav by the Nav Mode' incorporates:
   *  Constant: '<S181>/RTB1'
   *  Constant: '<S185>/RTB'
   *  Constant: '<S185>/RTB1'
   *  Constant: '<S188>/RTB'
   *  Constant: '<S188>/RTB1'
   *  Constant: '<S381>/Constant'
   *  Constant: '<S381>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Gain: '<S206>/Deg2R'
   *  Gain: '<S447>/Deg2R'
   *  Inport: '<S185>/MidLvl h_c'
   *  Inport: '<S188>/MidLvl h_c'
   *  Product: '<S330>/Divide'
   *  Product: '<S385>/Divide'
   *  Sum: '<S324>/Add'
   */
  if (AUAV_V3_TestSensors_B.GetRTBOrdernavSupportcupdated42[0] == 1) {
    /* Outputs for IfAction SubSystem: '<S180>/RTB//Follow Mobile Navigation' incorporates:
     *  ActionPort: '<S188>/Action Port'
     */
    /* Outputs for Enabled SubSystem: '<S188>/Compute Mobile Location' incorporates:
     *  EnablePort: '<S442>/Enable'
     */
    if (AUAV_V3_TestSensors_B.GetRTBOrdernavSupportcupdated42[1] > 0) {
      rtb_IC4_idx_0 *= 0.0174532924F;

      /* Gain: '<S447>/Deg2R' */
      rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_1;

      /* Gain: '<S448>/Deg2R' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_RhhcosphicoslambXe = 0.0174532924F * mlMobileLocation.latitude;

      /* Trigonometry: '<S448>/sin(phi)' */
      rtb_cosphi = (real32_T)sin(rtb_RhhcosphicoslambXe);

      /* Sum: '<S448>/Sum1' incorporates:
       *  Constant: '<S448>/const'
       *  Product: '<S448>/Product1'
       *  Product: '<S448>/sin(phi)^2'
       */
      rtb_Sum1_mj = 1.0F - rtb_cosphi * rtb_cosphi * 0.00669425726F;

      /* Fcn: '<S448>/f' */
      if (rtb_Sum1_mj < 0.0F) {
        rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
      } else {
        rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
      }

      /* End of Fcn: '<S448>/f' */

      /* Product: '<S448>/Rh' incorporates:
       *  Constant: '<S448>/Re=equatorial radius'
       */
      rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

      /* Trigonometry: '<S448>/cos(phi)' */
      rtb_RhhcosphicoslambXe = (real32_T)cos(rtb_RhhcosphicoslambXe);

      /* Gain: '<S448>/Deg2R1' incorporates:
       *  DataStoreRead: '<Root>/Get Mobile Location'
       */
      rtb_RhhcosphisinlambYe = 0.0174532924F * mlMobileLocation.longitude;

      /* Product: '<S448>/Ze' incorporates:
       *  Product: '<S448>/Rh(1-e^2)'
       */
      rtb_cosphi *= 0.993305743F * rtb_Sum1_mj;

      /* SignalConversion: '<S447>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S450>/11'
       *  Fcn: '<S450>/12'
       *  Fcn: '<S450>/13'
       *  Fcn: '<S450>/21'
       *  Fcn: '<S450>/22'
       *  Fcn: '<S450>/23'
       *  Fcn: '<S450>/31'
       *  Fcn: '<S450>/32'
       *  Fcn: '<S450>/33'
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

      /* Sum: '<S446>/Sum1' incorporates:
       *  Product: '<S447>/Product1'
       *  Product: '<S448>/(Rh+h)cos(phi)*cos(lamb)=Xe'
       *  Product: '<S448>/(Rh+h)cos(phi)*sin(lamb)=Ye'
       *  Sum: '<S448>/Sum2'
       *  Trigonometry: '<S448>/cos(lamb)'
       *  Trigonometry: '<S448>/sin(lamb)'
       */
      rtb_Deg2R1 = rtb_Sum1_mj * rtb_RhhcosphicoslambXe * (real32_T)cos
        (rtb_RhhcosphisinlambYe) - AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
      rtb_RhhcosphisinlambYe = rtb_Sum1_mj * rtb_RhhcosphicoslambXe * (real32_T)
        sin(rtb_RhhcosphisinlambYe) -
        AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
      rtb_Sum1_mj = rtb_cosphi - AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

      /* Product: '<S447>/Product1' incorporates:
       *  Gain: '<S446>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp_0[i] = tmp[i + 6] * rtb_Sum1_mj + (tmp[i + 3] *
          rtb_RhhcosphisinlambYe + tmp[i] * rtb_Deg2R1);
      }

      /* Reshape: '<S446>/Reshape1' incorporates:
       *  Gain: '<S446>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        AUAV_V3_TestSensors_B.Reshape1[i] = 0.0F;
        AUAV_V3_TestSensors_B.Reshape1[i] +=
          AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0];
        AUAV_V3_TestSensors_B.Reshape1[i] +=
          AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1];
        AUAV_V3_TestSensors_B.Reshape1[i] +=
          AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0[2];
      }

      /* End of Reshape: '<S446>/Reshape1' */
    }

    /* End of Outputs for SubSystem: '<S188>/Compute Mobile Location' */

    /* MATLAB Function: '<S188>/Zero out Z1' incorporates:
     *  Gain: '<S447>/Deg2R'
     */
    AUAV_V3_TestSens_ZerooutZ1(AUAV_V3_TestSensors_B.Reshape1,
      &AUAV_V3_TestSensors_B.sf_ZerooutZ1_a);

    /* Sum: '<S188>/Subtract' */
    rtb_Product3_j0[0] = AUAV_V3_TestSensors_B.sf_ZerooutZ1_a.P[0] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
    rtb_Product3_j0[1] = AUAV_V3_TestSensors_B.sf_ZerooutZ1_a.P[1] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
    rtb_Product3_j0[2] = AUAV_V3_TestSensors_B.sf_ZerooutZ1_a.P[2] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

    /* DiscreteIntegrator: '<S443>/Discrete-Time Integrator' */
    if (AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_IC_LOADI != 0) {
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[0] =
        rtb_Product3_j0[0];
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[1] =
        rtb_Product3_j0[1];
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[2] =
        rtb_Product3_j0[2];
    }

    /* Sum: '<S443>/Sum' incorporates:
     *  DiscreteIntegrator: '<S443>/Discrete-Time Integrator'
     */
    rtb_Product2_c[0] = rtb_Product3_j0[0] -
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[0];
    rtb_Product2_c[1] = rtb_Product3_j0[1] -
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[1];
    rtb_Product2_c[2] = rtb_Product3_j0[2] -
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* Sum: '<S443>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S443>/Discrete-Time Integrator'
     *  Gain: '<S443>/Gain1'
     */
    AUAV_V3_TestSensors_B.Merge_o[0] = 3.0F * rtb_Product2_c[0] +
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[0];
    AUAV_V3_TestSensors_B.Merge_o[1] = 3.0F * rtb_Product2_c[1] +
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[1];
    AUAV_V3_TestSensors_B.Merge_o[2] = 3.0F * rtb_Product2_c[2] +
      AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* MATLAB Function: '<S452>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_bs);

    /* MATLAB Function: '<S453>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_bs.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_la);

    /* S-Function (MCHP_C_function_Call): '<S453>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_g = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_la.zpVal
      );

    /* SignalConversion: '<S456>/Numerical Unity' */
    AUAV_V3_TestSensors_B.Merge2 =
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_g;
    AUAV_V3_TestSensors_B.Merge1 = mlMidLevelCommands.hCommand;
    AUAV_V3_TestSensors_B.Merge3 = 0U;
    AUAV_V3_TestSensors_B.Merge4 = 0U;

    /* Update for DiscreteIntegrator: '<S443>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S188>/RTB'
     *  Constant: '<S188>/RTB1'
     *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
     *  Inport: '<S188>/MidLvl h_c'
     */
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_IC_LOADI = 0U;
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[0] += 0.01F *
      rtb_Product2_c[0];
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[1] += 0.01F *
      rtb_Product2_c[1];
    AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_DSTATE[2] += 0.01F *
      rtb_Product2_c[2];

    /* End of Outputs for SubSystem: '<S180>/RTB//Follow Mobile Navigation' */
  } else if (rtb_Compare_j == 3) {
    /* Outputs for IfAction SubSystem: '<S180>/Normal WP  Navigation' incorporates:
     *  ActionPort: '<S187>/Action Port'
     */
    /* Product: '<S187>/Product' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    rtb_Product = mlParamInterface.param[19] *
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116;

    /* Outputs for Enabled SubSystem: '<S187>/On WP Enable' incorporates:
     *  EnablePort: '<S304>/Enable'
     */
    /* Gain: '<S405>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S383>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)1U)
          , &AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated511[0]
          );

    /* Gain: '<S406>/Deg2R' incorporates:
     *  Constant: '<S381>/Constant'
     */
    rtb_cosphi = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated511[0];

    /* Trigonometry: '<S406>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S406>/Sum1' incorporates:
     *  Constant: '<S406>/const'
     *  Product: '<S406>/Product1'
     *  Product: '<S406>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S406>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S406>/f' */

    /* Product: '<S406>/Rh' incorporates:
     *  Constant: '<S406>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S406>/Sum2' */
    rtb_RhhcosphicoslambXe =
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated511[2] + rtb_Sum1_mj;

    /* Trigonometry: '<S406>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S406>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated511[1];

    /* Product: '<S406>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S406>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S406>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S406>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S406>/Ze' incorporates:
     *  Product: '<S406>/Rh(1-e^2)'
     *  Sum: '<S406>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated511[2];

    /* SignalConversion: '<S405>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S408>/11'
     *  Fcn: '<S408>/12'
     *  Fcn: '<S408>/13'
     *  Fcn: '<S408>/21'
     *  Fcn: '<S408>/22'
     *  Fcn: '<S408>/23'
     *  Fcn: '<S408>/31'
     *  Fcn: '<S408>/32'
     *  Fcn: '<S408>/33'
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

    /* Sum: '<S403>/Sum1' incorporates:
     *  Product: '<S405>/Product1'
     */
    rtb_Deg2R1 -= AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

    /* Product: '<S405>/Product1' incorporates:
     *  Gain: '<S403>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp[i + 6] * rtb_Sum1_mj + (tmp[i + 3] * rtb_RhhcosphisinlambYe
        + tmp[i] * rtb_Deg2R1);
    }

    /* Gain: '<S403>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      AUAV_V3_TestSensors_B.WP0L2IPT1[i] = 0.0F;
      AUAV_V3_TestSensors_B.WP0L2IPT1[i] +=
        AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0];
      AUAV_V3_TestSensors_B.WP0L2IPT1[i] +=
        AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1];
      AUAV_V3_TestSensors_B.WP0L2IPT1[i] +=
        AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0[2];
    }

    /* Gain: '<S422>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S384>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          ((uint8_T)2U)
          , &AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_o[0]
          );

    /* Gain: '<S423>/Deg2R' incorporates:
     *  Constant: '<S381>/Constant1'
     */
    rtb_cosphi = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_o[0];

    /* Trigonometry: '<S423>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S423>/Sum1' incorporates:
     *  Constant: '<S423>/const'
     *  Product: '<S423>/Product1'
     *  Product: '<S423>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S423>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S423>/f' */

    /* Product: '<S423>/Rh' incorporates:
     *  Constant: '<S423>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S423>/Sum2' */
    rtb_RhhcosphicoslambXe =
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_o[2] + rtb_Sum1_mj;

    /* Trigonometry: '<S423>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S423>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_o[1];

    /* Product: '<S423>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S423>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S423>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S423>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S423>/Ze' incorporates:
     *  Product: '<S423>/Rh(1-e^2)'
     *  Sum: '<S423>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_o[2];

    /* SignalConversion: '<S422>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S425>/11'
     *  Fcn: '<S425>/12'
     *  Fcn: '<S425>/13'
     *  Fcn: '<S425>/21'
     *  Fcn: '<S425>/22'
     *  Fcn: '<S425>/23'
     *  Fcn: '<S425>/31'
     *  Fcn: '<S425>/32'
     *  Fcn: '<S425>/33'
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

    /* Sum: '<S420>/Sum1' incorporates:
     *  Product: '<S422>/Product1'
     */
    rtb_Deg2R1 -= AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

    /* Product: '<S422>/Product1' incorporates:
     *  Gain: '<S420>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_1[i + 6] * rtb_Sum1_mj + (tmp_1[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_1[i] * rtb_Deg2R1);
    }

    /* Gain: '<S420>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_P32[i] = AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0[2] +
        (AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1] +
         AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0]);
    }

    /* Sum: '<S382>/Add' incorporates:
     *  Gain: '<S420>/UEN 2 NEU'
     */
    rtb_Product3_j0[0] = rtb_P32[0] - AUAV_V3_TestSensors_B.WP0L2IPT1[0];
    rtb_Product3_j0[1] = rtb_P32[1] - AUAV_V3_TestSensors_B.WP0L2IPT1[1];
    rtb_Product3_j0[2] = rtb_P32[2] - AUAV_V3_TestSensors_B.WP0L2IPT1[2];

    /* MATLAB Function: '<S389>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_e);

    /* MATLAB Function: '<S390>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_e.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_nb);

    /* S-Function (MCHP_C_function_Call): '<S390>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a5 = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_nb.zpVal
      );

    /* Saturate: '<S385>/Zero Bound' */
    if (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a5 <= 0.001F) {
      rtb_Sum1_mj = 0.001F;
    } else {
      rtb_Sum1_mj = AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a5;
    }

    /* End of Saturate: '<S385>/Zero Bound' */
    rtb_Product3_j0[0] /= rtb_Sum1_mj;
    rtb_Product3_j0[1] /= rtb_Sum1_mj;

    /* Product: '<S385>/Divide' */
    rtb_Sum1_mj = rtb_Product3_j0[2] / rtb_Sum1_mj;
    rtb_Product3_j0[2] = rtb_Sum1_mj;

    /* MATLAB Function: '<S382>/Select N  Terms' incorporates:
     *  Product: '<S385>/Divide'
     */
    AUAV_V3_TestS_SelectNTerms(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_SelectNTerms_f);

    /* MATLAB Function: '<S396>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.sf_SelectNTerms_f.N,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_l);

    /* MATLAB Function: '<S397>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_l.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_j);

    /* S-Function (MCHP_C_function_Call): '<S397>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_i = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_j.zpVal
      );

    /* Sum: '<S304>/Subtract' incorporates:
     *  Constant: '<S304>/Constant5'
     *  Product: '<S304>/Product'
     */
    AUAV_V3_TestSensors_B.WP0L2IPT1[0] -= rtb_Product * rtb_Product3_j0[0] *
      2.0F;
    AUAV_V3_TestSensors_B.WP0L2IPT1[1] -= rtb_Product * rtb_Product3_j0[1] *
      2.0F;
    AUAV_V3_TestSensors_B.WP0L2IPT1[2] -= rtb_Product * rtb_Sum1_mj * 2.0F;

    /* End of Outputs for SubSystem: '<S187>/On WP Enable' */

    /* MATLAB Function: '<S187>/Zero out Z2' */
    AUAV_V3_TestSens_ZerooutZ1(AUAV_V3_TestSensors_B.WP0L2IPT1,
      &AUAV_V3_TestSensors_B.sf_ZerooutZ2_j);

    /* Sum: '<S187>/Add' */
    AUAV_V3_TestSensors_B.Merge_o[0] = AUAV_V3_TestSensors_B.sf_ZerooutZ2_j.P[0]
      - AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
    AUAV_V3_TestSensors_B.Merge_o[1] = AUAV_V3_TestSensors_B.sf_ZerooutZ2_j.P[1]
      - AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
    AUAV_V3_TestSensors_B.Merge_o[2] = AUAV_V3_TestSensors_B.sf_ZerooutZ2_j.P[2]
      - AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S436>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.Merge_o,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_d);

    /* MATLAB Function: '<S437>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_d.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_lj);

    /* S-Function (MCHP_C_function_Call): '<S437>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_b = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_lj.zpVal
      );

    /* MATLAB Function: '<S187>/Embedded MATLAB Function' incorporates:
     *  Gain: '<S187>/Gain'
     *  RelationalOperator: '<S187>/Relational Operator'
     */
    /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/Embedded MATLAB Function': '<S302>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    /*  this is just turned on for one sample  */
    /*  inmediatley upon enabling the navigation. */
    /*  Reset the flag the the IP was reached */
    if (rtb_DataTypeConversion2_fi != 0) {
      /* '<S302>:1:19' */
      AUAV_V3_TestSensors_DWork.persistentDidReachIP = 0U;
    }

    /*  Once the IP is reached the the persistent variable */
    /*  preserves the values until reset */
    if (0.4F * rtb_Product > AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_b)
    {
      /* '<S302>:1:24' */
      /* '<S302>:1:25' */
      AUAV_V3_TestSensors_DWork.persistentDidReachIP = 1U;
    }

    /* InitialCondition: '<S187>/IC' incorporates:
     *  MATLAB Function: '<S187>/Embedded MATLAB Function'
     */
    /* '<S302>:1:28' */
    if (AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_m) {
      AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_m = false;
      AUAV_V3_TestSensors_B.IC = 0U;
    } else {
      AUAV_V3_TestSensors_B.IC = AUAV_V3_TestSensors_DWork.persistentDidReachIP;
    }

    /* End of InitialCondition: '<S187>/IC' */

    /* MATLAB Function: '<S187>/computeCurrentWP' incorporates:
     *  DataStoreRead: '<Root>/Get Max WP'
     *  Delay: '<S187>/Integer Delay'
     */
    /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Normal WP  Navigation/computeCurrentWP': '<S307>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    if (AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_im != 0) {
      /* '<S307>:1:16' */
      AUAV_V3_TestSensors_DWork.fromWp = AUAV_V3_TestSensors_DWork.toWp;

      /* '<S307>:1:17' */
      rtb_u2deg = AUAV_V3_TestSensors_DWork.toWp + 1U;
      if (rtb_u2deg > 255U) {
        rtb_u2deg = 255U;
      }

      AUAV_V3_TestSensors_DWork.toWp = (uint8_T)rtb_u2deg;
      if (AUAV_V3_TestSensors_DWork.toWp > mlWpValues.wpCount) {
        /* '<S307>:1:18' */
        /* '<S307>:1:19' */
        AUAV_V3_TestSensors_DWork.toWp = 1U;
      }
    }

    /*  this is jturned on long as we have not reached IP  */
    if (!(AUAV_V3_TestSensors_B.IC != 0)) {
      /* '<S307>:1:25' */
      /* '<S307>:1:26' */
      AUAV_V3_TestSensors_DWork.fromWp = 1U;

      /* '<S307>:1:27' */
      AUAV_V3_TestSensors_DWork.toWp = 2U;
    }

    /* '<S307>:1:32' */
    AUAV_V3_TestSensors_B.WP0 = AUAV_V3_TestSensors_DWork.fromWp;

    /* '<S307>:1:33' */
    AUAV_V3_TestSensors_B.WP1 = AUAV_V3_TestSensors_DWork.toWp;

    /* End of MATLAB Function: '<S187>/computeCurrentWP' */

    /* Outputs for Enabled SubSystem: '<S187>/Get Frenet' incorporates:
     *  EnablePort: '<S303>/Enable'
     */
    /* Gain: '<S350>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S328>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          AUAV_V3_TestSensors_B.WP0
          , &AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_d[0]
          );

    /* Gain: '<S351>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_d[0];

    /* Trigonometry: '<S351>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S351>/Sum1' incorporates:
     *  Constant: '<S351>/const'
     *  Product: '<S351>/Product1'
     *  Product: '<S351>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S351>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S351>/f' */

    /* Product: '<S351>/Rh' incorporates:
     *  Constant: '<S351>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S351>/Sum2' */
    rtb_RhhcosphicoslambXe =
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_d[2] + rtb_Sum1_mj;

    /* Trigonometry: '<S351>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S351>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_d[1];

    /* Product: '<S351>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S351>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S351>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S351>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S351>/Ze' incorporates:
     *  Product: '<S351>/Rh(1-e^2)'
     *  Sum: '<S351>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_d[2];

    /* SignalConversion: '<S350>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S353>/11'
     *  Fcn: '<S353>/12'
     *  Fcn: '<S353>/13'
     *  Fcn: '<S353>/21'
     *  Fcn: '<S353>/22'
     *  Fcn: '<S353>/23'
     *  Fcn: '<S353>/31'
     *  Fcn: '<S353>/32'
     *  Fcn: '<S353>/33'
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

    /* Sum: '<S348>/Sum1' incorporates:
     *  Product: '<S350>/Product1'
     */
    rtb_Deg2R1 -= AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

    /* Product: '<S350>/Product1' incorporates:
     *  Gain: '<S348>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_2[i + 6] * rtb_Sum1_mj + (tmp_2[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_2[i] * rtb_Deg2R1);
    }

    /* Gain: '<S348>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product3_j0[i] = AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0[2]
        + (AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1] +
           AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0]);
    }

    /* Gain: '<S367>/Deg2R' */
    rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_0;
    rtb_Product3_bo = 0.0174532924F * rtb_IC4_idx_1;

    /* S-Function (MCHP_C_function_Call): '<S329>/Get WP Coord navsupport.c [updated 5.1.16]' */
    getWP(
          AUAV_V3_TestSensors_B.WP1
          , &AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_b[0]
          );

    /* Gain: '<S368>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_b[0];

    /* Trigonometry: '<S368>/sin(phi)' */
    rtb_Ze_b = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S368>/Sum1' incorporates:
     *  Constant: '<S368>/const'
     *  Product: '<S368>/Product1'
     *  Product: '<S368>/sin(phi)^2'
     */
    rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

    /* Fcn: '<S368>/f' */
    if (rtb_Sum1_mj < 0.0F) {
      rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
    } else {
      rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
    }

    /* End of Fcn: '<S368>/f' */

    /* Product: '<S368>/Rh' incorporates:
     *  Constant: '<S368>/Re=equatorial radius'
     */
    rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

    /* Sum: '<S368>/Sum2' */
    rtb_RhhcosphicoslambXe =
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_b[2] + rtb_Sum1_mj;

    /* Trigonometry: '<S368>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S368>/Deg2R1' */
    rtb_RhhcosphisinlambYe = 0.0174532924F *
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_b[1];

    /* Product: '<S368>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S368>/cos(lamb)'
     */
    rtb_Deg2R1 = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)cos
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S368>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S368>/sin(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphicoslambXe * rtb_cosphi * (real32_T)sin
      (rtb_RhhcosphisinlambYe);

    /* Product: '<S368>/Ze' incorporates:
     *  Product: '<S368>/Rh(1-e^2)'
     *  Sum: '<S368>/Sum4'
     */
    rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj +
      AUAV_V3_TestSensors_B.GetWPCoordnavsupportcupdated5_b[2];

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
    tmp_3[0] = (real32_T)cos(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_3[1] = -(real32_T)sin(rtb_Deg2R_h_idx_0);
    tmp_3[2] = -(real32_T)sin(rtb_Product3_bo) * (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_3[3] = (real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)cos(rtb_Product3_bo);
    tmp_3[4] = (real32_T)cos(rtb_Deg2R_h_idx_0);
    tmp_3[5] = -(real32_T)sin(rtb_Deg2R_h_idx_0) * (real32_T)sin(rtb_Product3_bo);
    tmp_3[6] = (real32_T)sin(rtb_Product3_bo);
    tmp_3[7] = 0.0F;
    tmp_3[8] = (real32_T)cos(rtb_Product3_bo);

    /* Sum: '<S365>/Sum1' incorporates:
     *  Product: '<S367>/Product1'
     */
    rtb_Deg2R1 -= AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
    rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe -
      AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
    rtb_Sum1_mj = rtb_Ze_b - AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

    /* Product: '<S367>/Product1' incorporates:
     *  Gain: '<S365>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = tmp_3[i + 6] * rtb_Sum1_mj + (tmp_3[i + 3] *
        rtb_RhhcosphisinlambYe + tmp_3[i] * rtb_Deg2R1);
    }

    /* Gain: '<S365>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product2_c[i] = AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0[2]
        + (AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1] +
           AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0]);
    }

    rtb_Product3_j0[0] = rtb_Product2_c[0] - rtb_Product3_j0[0];
    rtb_Product3_j0[1] = rtb_Product2_c[1] - rtb_Product3_j0[1];
    rtb_Product3_j0[2] = rtb_Product2_c[2] - rtb_Product3_j0[2];

    /* MATLAB Function: '<S334>/Embedded MATLAB Function' incorporates:
     *  Sum: '<S324>/Add'
     */
    A_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ih);

    /* MATLAB Function: '<S335>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_ih.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_bi);

    /* S-Function (MCHP_C_function_Call): '<S335>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_pr = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_bi.zpVal
      );

    /* Saturate: '<S330>/Zero Bound' */
    if (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_pr <= 0.001F) {
      rtb_RhhcosphisinlambYe = 0.001F;
    } else {
      rtb_RhhcosphisinlambYe =
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_pr;
    }

    /* End of Saturate: '<S330>/Zero Bound' */
    rtb_Product3_j0[0] /= rtb_RhhcosphisinlambYe;
    rtb_Product3_j0[1] /= rtb_RhhcosphisinlambYe;
    rtb_Product3_j0[2] /= rtb_RhhcosphisinlambYe;

    /* MATLAB Function: '<S324>/Select N  Terms' incorporates:
     *  Product: '<S330>/Divide'
     */
    AUAV_V3_TestS_SelectNTerms(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_SelectNTerms);

    /* MATLAB Function: '<S341>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.sf_SelectNTerms.N,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b1);

    /* MATLAB Function: '<S342>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b1.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_n);

    /* S-Function (MCHP_C_function_Call): '<S342>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m4 = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_n.zpVal
      );

    /* Saturate: '<S331>/Zero Bound' */
    if (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m4 <= 0.001F) {
      rtb_RhhcosphisinlambYe = 0.001F;
    } else {
      rtb_RhhcosphisinlambYe =
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m4;
    }

    /* End of Saturate: '<S331>/Zero Bound' */

    /* Product: '<S331>/Divide' */
    rtb_Merge_m[0] = AUAV_V3_TestSensors_B.sf_SelectNTerms.N[0] /
      rtb_RhhcosphisinlambYe;
    rtb_Merge_m[1] = AUAV_V3_TestSensors_B.sf_SelectNTerms.N[1] /
      rtb_RhhcosphisinlambYe;
    rtb_Merge_m[2] = AUAV_V3_TestSensors_B.sf_SelectNTerms.N[2] /
      rtb_RhhcosphisinlambYe;

    /* Delay: '<S303>/Integer Delay1' */
    AUAV_V3_TestSensors_B.Merge1 =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_gn;

    /* MATLAB Function: '<S303>/Zero out Z1' */
    AUAV_V3_TestSens_ZerooutZ1(rtb_Merge_m,
      &AUAV_V3_TestSensors_B.sf_ZerooutZ1_j);

    /* MATLAB Function: '<S303>/Zero out Z2' */
    AUAV_V3_TestSens_ZerooutZ1(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_ZerooutZ2_e);

    /* MATLAB Function: '<S303>/Zero out Z3' */
    AUAV_V3_TestSens_ZerooutZ1(rtb_Product2_c,
      &AUAV_V3_TestSensors_B.sf_ZerooutZ3);

    /* Product: '<S303>/Product1' incorporates:
     *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
     */
    AUAV_V3_TestSensors_B.Product1 = mlParamInterface.param[20] *
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116;

    /* Update for Delay: '<S303>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_gn = rtb_Product2_c[2];

    /* End of Outputs for SubSystem: '<S187>/Get Frenet' */

    /* Sum: '<S301>/Subtract' */
    rtb_Product3_j0[0] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[0];
    rtb_Product3_j0[1] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[1];
    rtb_Product3_j0[2] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[2];

    /* MATLAB Function: '<S309>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_k(rtb_Product3_j0,
      AUAV_V3_TestSensors_B.sf_ZerooutZ1_j.P,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_o);

    /* S-Function (MCHP_C_function_Call): '<S310>/myAbs() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_i = myAbs(
      AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_o.xDoty
      );

    /* Sum: '<S308>/Subtract' */
    rtb_Product2_c[0] = AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[0] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
    rtb_Product2_c[1] = AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[1] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
    rtb_Product2_c[2] = AUAV_V3_TestSensors_B.sf_ZerooutZ3.P[2] -
      AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S316>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_k(rtb_Product2_c,
      AUAV_V3_TestSensors_B.sf_ZerooutZ2_e.P,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_k);

    /* Switch: '<S315>/Switch3' incorporates:
     *  RelationalOperator: '<S315>/Relational Operator2'
     */
    if ((AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_k.xDoty ==
         AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_k.xDoty) > 0) {
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_p =
        AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_k.xDoty;
    }

    /* End of Switch: '<S315>/Switch3' */

    /* Product: '<S301>/Divide' incorporates:
     *  Constant: '<S301>/Constant4'
     */
    rtb_Sum1_mj = AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_i /
      0.577350259F;

    /* Switch: '<S314>/Switch' incorporates:
     *  Product: '<S301>/Divide1'
     *  RelationalOperator: '<S314>/Relational Operator'
     */
    if (rtb_Product < rtb_Sum1_mj) {
      rtb_Sum1_mj = rtb_Product;
    }

    /* End of Switch: '<S314>/Switch' */

    /* MATLAB Function: '<S311>/negprotect' incorporates:
     *  Product: '<S301>/Product'
     *  Product: '<S301>/Product1'
     *  Sum: '<S301>/Add'
     */
    AUAV_V3_TestS_negprotect_l(rtb_Product * rtb_Product -
      AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_i *
      AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_i,
      &AUAV_V3_TestSensors_B.sf_negprotect_p);

    /* S-Function (MCHP_C_function_Call): '<S311>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_d = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_p.zpVal
      );

    /* Switch: '<S187>/Switch' incorporates:
     *  Product: '<S301>/Product2'
     *  Sum: '<S301>/Add3'
     *  Switch: '<S187>/Switch1'
     */
    if (AUAV_V3_TestSensors_B.IC > 0) {
      /* Switch: '<S313>/Switch' incorporates:
       *  RelationalOperator: '<S301>/Relational Operator'
       *  RelationalOperator: '<S313>/Relational Operator'
       *  Switch: '<S301>/Switch1'
       */
      if ((!(AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_i > rtb_Product)) &&
          (!(rtb_Sum1_mj > AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_d)))
      {
        rtb_Sum1_mj = AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_d;
      }

      /* End of Switch: '<S313>/Switch' */

      /* Sum: '<S301>/Add1' */
      rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_p
        - rtb_Sum1_mj;

      /* Switch: '<S312>/Switch' incorporates:
       *  Constant: '<S301>/Constant1'
       *  RelationalOperator: '<S312>/Relational Operator'
       */
      if (!(rtb_RhhcosphisinlambYe > 0.0F)) {
        rtb_RhhcosphisinlambYe = 0.0F;
      }

      /* End of Switch: '<S312>/Switch' */
      AUAV_V3_TestSensors_B.Merge_o[0] = (0.0F - rtb_RhhcosphisinlambYe *
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_e.P[0]) - rtb_Product3_j0[0];
      AUAV_V3_TestSensors_B.Merge_o[1] = (0.0F - rtb_RhhcosphisinlambYe *
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_e.P[1]) - rtb_Product3_j0[1];
      AUAV_V3_TestSensors_B.Merge_o[2] = (0.0F - rtb_RhhcosphisinlambYe *
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_e.P[2]) - rtb_Product3_j0[2];
      AUAV_V3_TestSensors_B.Merge2 =
        AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_p;
    } else {
      AUAV_V3_TestSensors_B.Merge2 =
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_b;
    }

    /* End of Switch: '<S187>/Switch' */

    /* Update for Delay: '<S187>/Integer Delay' incorporates:
     *  RelationalOperator: '<S301>/Switch Distance Less than?'
     */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_im = (uint8_T)
      (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_p <
       AUAV_V3_TestSensors_B.Product1);

    /* End of Outputs for SubSystem: '<S180>/Normal WP  Navigation' */
  } else if (rtb_Compare_j == 10) {
    /* Outputs for IfAction SubSystem: '<S180>/Line Segment' incorporates:
     *  ActionPort: '<S185>/Action Port'
     */
    /* Gain: '<S185>/Gain' */
    AUAV_V3_TestSensors_B.Merge_o[0] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
    AUAV_V3_TestSensors_B.Merge_o[1] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
    AUAV_V3_TestSensors_B.Merge_o[2] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

    /* Gain: '<S185>/Gain1' */
    rtb_Product3_j0[0] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
    rtb_Product3_j0[1] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
    rtb_Product3_j0[2] = -AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S295>/Embedded MATLAB Function' */
    A_EmbeddedMATLABFunction_b(rtb_Product3_j0,
      &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_le);

    /* MATLAB Function: '<S296>/negprotect' */
    AUAV_V3_TestS_negprotect_l
      (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_le.xDoty,
       &AUAV_V3_TestSensors_B.sf_negprotect_o4);

    /* S-Function (MCHP_C_function_Call): '<S296>/mySqrt() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_pu = mySqrt(
      AUAV_V3_TestSensors_B.sf_negprotect_o4.zpVal
      );

    /* SignalConversion: '<S299>/Numerical Unity' */
    AUAV_V3_TestSensors_B.Merge2 =
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_pu;
    AUAV_V3_TestSensors_B.Merge1 = mlMidLevelCommands.hCommand;
    AUAV_V3_TestSensors_B.Merge3 = 0U;
    AUAV_V3_TestSensors_B.Merge4 = 0U;

    /* End of Outputs for SubSystem: '<S180>/Line Segment' */
  } else {
    if (rtb_Compare_j == 9) {
      /* Outputs for IfAction SubSystem: '<S180>/Circle Navigation' incorporates:
       *  ActionPort: '<S181>/Action Port'
       */
      rtb_IC4_idx_0 *= 0.0174532924F;

      /* Gain: '<S206>/Deg2R' */
      rtb_Deg2R_h_idx_0 = 0.0174532924F * rtb_IC4_idx_1;

      /* Gain: '<S207>/Deg2R' incorporates:
       *  DataStoreRead: '<S72>/Get ISR Location'
       */
      rtb_RhhcosphicoslambXe = 0.0174532924F * mlISR.latitude;

      /* Trigonometry: '<S207>/sin(phi)' */
      rtb_Ze_b = (real32_T)sin(rtb_RhhcosphicoslambXe);

      /* Sum: '<S207>/Sum1' incorporates:
       *  Constant: '<S207>/const'
       *  Product: '<S207>/Product1'
       *  Product: '<S207>/sin(phi)^2'
       */
      rtb_Sum1_mj = 1.0F - rtb_Ze_b * rtb_Ze_b * 0.00669425726F;

      /* Fcn: '<S207>/f' */
      if (rtb_Sum1_mj < 0.0F) {
        rtb_Sum1_mj = -(real32_T)sqrt(-rtb_Sum1_mj);
      } else {
        rtb_Sum1_mj = (real32_T)sqrt(rtb_Sum1_mj);
      }

      /* End of Fcn: '<S207>/f' */

      /* Product: '<S207>/Rh' incorporates:
       *  Constant: '<S207>/Re=equatorial radius'
       */
      rtb_Sum1_mj = 6.378137E+6F / rtb_Sum1_mj;

      /* Sum: '<S207>/Sum2' incorporates:
       *  DataStoreRead: '<S72>/Get ISR Location'
       */
      rtb_cosphi = mlISR.height + rtb_Sum1_mj;

      /* Trigonometry: '<S207>/cos(phi)' */
      rtb_RhhcosphicoslambXe = (real32_T)cos(rtb_RhhcosphicoslambXe);

      /* Gain: '<S207>/Deg2R1' incorporates:
       *  DataStoreRead: '<S72>/Get ISR Location'
       */
      rtb_RhhcosphisinlambYe = 0.0174532924F * mlISR.longitude;

      /* Product: '<S207>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
       *  Trigonometry: '<S207>/cos(lamb)'
       */
      rtb_Deg2R1 = rtb_cosphi * rtb_RhhcosphicoslambXe * (real32_T)cos
        (rtb_RhhcosphisinlambYe);

      /* Product: '<S207>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
       *  Trigonometry: '<S207>/sin(lamb)'
       */
      rtb_cosphi = rtb_cosphi * rtb_RhhcosphicoslambXe * (real32_T)sin
        (rtb_RhhcosphisinlambYe);

      /* Product: '<S207>/Ze' incorporates:
       *  DataStoreRead: '<S72>/Get ISR Location'
       *  Product: '<S207>/Rh(1-e^2)'
       *  Sum: '<S207>/Sum4'
       */
      rtb_Ze_b *= 0.993305743F * rtb_Sum1_mj + mlISR.height;

      /* SignalConversion: '<S206>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S209>/11'
       *  Fcn: '<S209>/12'
       *  Fcn: '<S209>/13'
       *  Fcn: '<S209>/21'
       *  Fcn: '<S209>/22'
       *  Fcn: '<S209>/23'
       *  Fcn: '<S209>/31'
       *  Fcn: '<S209>/32'
       *  Fcn: '<S209>/33'
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

      /* Sum: '<S204>/Sum1' incorporates:
       *  Product: '<S206>/Product1'
       */
      rtb_Deg2R1 -= AUAV_V3_TestSensors_B.DataTypeConversion1_l[0];
      rtb_Sum1_mj = rtb_cosphi - AUAV_V3_TestSensors_B.DataTypeConversion1_l[1];
      rtb_RhhcosphisinlambYe = rtb_Ze_b -
        AUAV_V3_TestSensors_B.DataTypeConversion1_l[2];

      /* Product: '<S206>/Product1' incorporates:
       *  Gain: '<S204>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp_0[i] = tmp[i + 6] * rtb_RhhcosphisinlambYe + (tmp[i + 3] *
          rtb_Sum1_mj + tmp[i] * rtb_Deg2R1);
      }

      /* Gain: '<S204>/UEN 2 NEU' */
      for (i = 0; i < 3; i++) {
        rtb_Product3_j0[i] = AUAV_V3_TestSensors_ConstP.pooled63[i + 6] * tmp_0
          [2] + (AUAV_V3_TestSensors_ConstP.pooled63[i + 3] * tmp_0[1] +
                 AUAV_V3_TestSensors_ConstP.pooled63[i] * tmp_0[0]);
      }

      /* Delay: '<S198>/Integer Delay1' */
      AUAV_V3_TestSensors_B.Merge1 =
        AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_a;

      /* MATLAB Function: '<S198>/Zero out Z2' */
      AUAV_V3_TestSens_ZerooutZ1(rtb_Product3_j0,
        &AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2);

      /* Sum: '<S181>/Sum1' */
      rtb_Product2_c[0] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0] -
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[0];
      rtb_Product2_c[1] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1] -
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[1];
      rtb_Product2_c[2] = AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2] -
        AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[2];

      /* MATLAB Function: '<S250>/Embedded MATLAB Function' */
      A_EmbeddedMATLABFunction_b(rtb_Product2_c,
        &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_c);

      /* MATLAB Function: '<S251>/negprotect' */
      AUAV_V3_TestS_negprotect_l
        (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_c.xDoty,
         &AUAV_V3_TestSensors_B.sf_negprotect_lu);

      /* S-Function (MCHP_C_function_Call): '<S251>/mySqrt() apUtils.c [updated 5.1.16]' */
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a = mySqrt(
        AUAV_V3_TestSensors_B.sf_negprotect_lu.zpVal
        );

      /* Product: '<S181>/Product' incorporates:
       *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
       */
      rtb_RhhcosphisinlambYe = mlParamInterface.param[19] *
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116;

      /* Sum: '<S181>/Sum2' */
      AUAV_V3_TestSensors_B.Sum2 = rtb_Product3_bo - rtb_RhhcosphisinlambYe;

      /* S-Function (MCHP_C_function_Call): '<S203>/myAbs() apUtils.c [updated 5.1.16]' */
      AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_a = myAbs(
        AUAV_V3_TestSensors_B.Sum2
        );

      /* If: '<S181>/If' incorporates:
       *  Constant: '<S199>/RTB1'
       *  Constant: '<S200>/RTB1'
       *  Constant: '<S201>/RTB1'
       *  Product: '<S200>/Product6'
       *  Product: '<S200>/Product7'
       *  Sum: '<S181>/Sum'
       *  Sum: '<S200>/Subtract3'
       *  Sum: '<S200>/Subtract6'
       */
      if (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a >
          rtb_RhhcosphisinlambYe + rtb_Product3_bo) {
        /* Outputs for IfAction SubSystem: '<S181>/No intersection,  Navigate to ISR' incorporates:
         *  ActionPort: '<S201>/Action Port'
         */
        /* Sum: '<S201>/Subtract' */
        AUAV_V3_TestSensors_B.Merge_o[0] =
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[0] -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
        AUAV_V3_TestSensors_B.Merge_o[1] =
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[1] -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
        AUAV_V3_TestSensors_B.Merge_o[2] =
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[2] -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];
        AUAV_V3_TestSensors_B.Merge3 = 0U;

        /* End of Outputs for SubSystem: '<S181>/No intersection,  Navigate to ISR' */
      } else if (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a <
                 AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_a) {
        /* Outputs for IfAction SubSystem: '<S181>/Inside the Circle,  Keep Straight until  intersection' incorporates:
         *  ActionPort: '<S199>/Action Port'
         */
        /* Sum: '<S199>/Subtract' incorporates:
         *  MATLAB Function: '<S199>/Compute Head of Circle'
         */
        /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle': '<S211>:1' */
        /* % Compute the top coordinate of the circle */
        /*  using the circle's parametric equations: */
        /*   x = a + r cos (t) */
        /*   y = b + r sin (t) */
        /*  */
        /*  @ t =0 */
        /* '<S211>:1:9' */
        /* '<S211>:1:11' */
        /* '<S211>:1:12' */
        /* '<S211>:1:13' */
        AUAV_V3_TestSensors_B.Merge_o[0] = (real32_T)((real_T)
          (AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[0] + rtb_Product3_bo) -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0]);
        AUAV_V3_TestSensors_B.Merge_o[1] = (real32_T)((real_T)
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[1] -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1]);
        AUAV_V3_TestSensors_B.Merge_o[2] = (real32_T)((real_T)
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[2] -
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2]);
        AUAV_V3_TestSensors_B.Merge3 = 1U;

        /* End of Outputs for SubSystem: '<S181>/Inside the Circle,  Keep Straight until  intersection' */
      } else {
        /* Outputs for IfAction SubSystem: '<S181>/Intersection. Circular Navigation' incorporates:
         *  ActionPort: '<S200>/Action Port'
         */
        /* Product: '<S200>/Product' */
        rtb_Sum1_mj = rtb_Product3_bo * rtb_Product3_bo;

        /* MATLAB Function: '<S200>/negprotect2' incorporates:
         *  Sum: '<S200>/Subtract1'
         */
        AUAV_V3_TestSe_negprotect3
          (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a +
           AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a,
           &AUAV_V3_TestSensors_B.sf_negprotect2);

        /* Product: '<S200>/Product3' incorporates:
         *  Product: '<S200>/Product1'
         *  Product: '<S200>/Product2'
         *  Sum: '<S200>/Subtract'
         */
        rtb_RhhcosphisinlambYe = ((rtb_Sum1_mj - rtb_RhhcosphisinlambYe *
          rtb_RhhcosphisinlambYe) +
          AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a *
          AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a) /
          AUAV_V3_TestSensors_B.sf_negprotect2.zpVal;

        /* MATLAB Function: '<S217>/negprotect' incorporates:
         *  Product: '<S200>/Product4'
         *  Sum: '<S200>/Subtract2'
         */
        AUAV_V3_TestS_negprotect_l(rtb_Sum1_mj - rtb_RhhcosphisinlambYe *
          rtb_RhhcosphisinlambYe, &AUAV_V3_TestSensors_B.sf_negprotect_ne);

        /* S-Function (MCHP_C_function_Call): '<S217>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m = mySqrt(
          AUAV_V3_TestSensors_B.sf_negprotect_ne.zpVal
          );

        /* MATLAB Function: '<S200>/negprotect1' */
        AUAV_V3_TestSe_negprotect3
          (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_a,
           &AUAV_V3_TestSensors_B.sf_negprotect1);
        rtb_Product2_c[0] /= AUAV_V3_TestSensors_B.sf_negprotect1.zpVal;
        rtb_Product2_c[1] /= AUAV_V3_TestSensors_B.sf_negprotect1.zpVal;

        /* Product: '<S200>/Product5' incorporates:
         *  Product: '<S200>/Product6'
         */
        rtb_Merge_m[0] = AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m *
          rtb_Product2_c[0];
        rtb_Merge_m[1] = AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_m *
          rtb_Product2_c[1];
        rtb_Product2_c[0] = rtb_Product2_c[0] * rtb_RhhcosphisinlambYe +
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[0];
        rtb_Product2_c[1] = rtb_Product2_c[1] * rtb_RhhcosphisinlambYe +
          AUAV_V3_TestSensors_B.sf_ZerooutZ2_h2.P[1];

        /* MATLAB Function: '<S200>/Embedded MATLAB Function' incorporates:
         *  Product: '<S200>/Product7'
         *  Sum: '<S200>/Subtract3'
         */
        /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function': '<S214>:1' */
        /*  This block supports an embeddable subset of the MATLAB language. */
        /*  See the help menu for details.  */
        /* '<S214>:1:5' */
        AUAV_V3_TestSensors_B.Merge_o[0] = 0.0F;
        AUAV_V3_TestSensors_B.Merge_o[1] = 0.0F;
        AUAV_V3_TestSensors_B.Merge_o[2] = 0.0F;

        /* '<S214>:1:6' */
        /* '<S214>:1:8' */
        AUAV_V3_TestSensors_B.Merge_o[0] = rtb_Product2_c[0] + rtb_Merge_m[1];

        /* '<S214>:1:9' */
        rtb_P32[0] = rtb_Product2_c[0] - rtb_Merge_m[1];

        /* '<S214>:1:11' */
        AUAV_V3_TestSensors_B.Merge_o[1] = rtb_Product2_c[1] - rtb_Merge_m[0];

        /* '<S214>:1:12' */
        rtb_P32[1] = rtb_Product2_c[1] + rtb_Merge_m[0];
        rtb_P32[0] -= AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
        rtb_P32[1] -= AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
        rtb_P32[2] = 0.0F - AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S221>/Embedded MATLAB Function' incorporates:
         *  Sum: '<S200>/Subtract6'
         */
        A_EmbeddedMATLABFunction_k(AUAV_V3_TestSensors_B.sf_ZerooutZ1.P, rtb_P32,
          &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_p);

        /* MATLAB Function: '<S224>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_b(rtb_P32,
          &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_bp);

        /* MATLAB Function: '<S225>/negprotect' */
        AUAV_V3_TestS_negprotect_l
          (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_bp.xDoty,
           &AUAV_V3_TestSensors_B.sf_negprotect_c);

        /* S-Function (MCHP_C_function_Call): '<S225>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_j = mySqrt(
          AUAV_V3_TestSensors_B.sf_negprotect_c.zpVal
          );

        /* MATLAB Function: '<S212>/negprotect3' incorporates:
         *  Product: '<S212>/Product9'
         */
        AUAV_V3_TestSe_negprotect3
          (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_j *
           AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116,
           &AUAV_V3_TestSensors_B.sf_negprotect3);

        /* Product: '<S212>/Product8' */
        AUAV_V3_TestSensors_B.u1 =
          AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_p.xDoty /
          AUAV_V3_TestSensors_B.sf_negprotect3.zpVal;

        /* Saturate: '<S212>/[-1 1]' */
        if (AUAV_V3_TestSensors_B.u1 > 1.0F) {
          /* Product: '<S212>/Product8' */
          AUAV_V3_TestSensors_B.u1 = 1.0F;
        } else {
          if (AUAV_V3_TestSensors_B.u1 < -1.0F) {
            /* Product: '<S212>/Product8' */
            AUAV_V3_TestSensors_B.u1 = -1.0F;
          }
        }

        /* End of Saturate: '<S212>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S222>/myAcos() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.myAcosapUtilscupdated5116 = myAcos(
          AUAV_V3_TestSensors_B.u1
          );

        /* Sum: '<S200>/Subtract5' */
        AUAV_V3_TestSensors_B.Merge_o[0] -=
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[0];
        AUAV_V3_TestSensors_B.Merge_o[1] -=
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[1];
        AUAV_V3_TestSensors_B.Merge_o[2] -=
          AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S234>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_k(AUAV_V3_TestSensors_B.sf_ZerooutZ1.P,
          AUAV_V3_TestSensors_B.Merge_o,
          &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_m);

        /* MATLAB Function: '<S237>/Embedded MATLAB Function' */
        A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.Merge_o,
          &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_eg);

        /* MATLAB Function: '<S238>/negprotect' */
        AUAV_V3_TestS_negprotect_l
          (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_eg.xDoty,
           &AUAV_V3_TestSensors_B.sf_negprotect_i);

        /* S-Function (MCHP_C_function_Call): '<S238>/mySqrt() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_e = mySqrt(
          AUAV_V3_TestSensors_B.sf_negprotect_i.zpVal
          );

        /* MATLAB Function: '<S213>/negprotect3' incorporates:
         *  Product: '<S213>/Product9'
         */
        AUAV_V3_TestSe_negprotect3
          (AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_e *
           AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116,
           &AUAV_V3_TestSensors_B.sf_negprotect3_i);

        /* Product: '<S213>/Product8' */
        AUAV_V3_TestSensors_B.u1_h =
          AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_m.xDoty /
          AUAV_V3_TestSensors_B.sf_negprotect3_i.zpVal;

        /* Saturate: '<S213>/[-1 1]' */
        if (AUAV_V3_TestSensors_B.u1_h > 1.0F) {
          /* Product: '<S213>/Product8' */
          AUAV_V3_TestSensors_B.u1_h = 1.0F;
        } else {
          if (AUAV_V3_TestSensors_B.u1_h < -1.0F) {
            /* Product: '<S213>/Product8' */
            AUAV_V3_TestSensors_B.u1_h = -1.0F;
          }
        }

        /* End of Saturate: '<S213>/[-1 1]' */

        /* S-Function (MCHP_C_function_Call): '<S235>/myACos() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.myACosapUtilscupdated5116 = myAcos(
          AUAV_V3_TestSensors_B.u1_h
          );

        /* S-Function (MCHP_C_function_Call): '<S216>/myAbs() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_c = myAbs(
          AUAV_V3_TestSensors_B.myACosapUtilscupdated5116
          );

        /* S-Function (MCHP_C_function_Call): '<S215>/myAbs() apUtils.c [updated 5.1.16]' */
        AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_b = myAbs(
          AUAV_V3_TestSensors_B.myAcosapUtilscupdated5116
          );

        /* Switch: '<S200>/Switch1' incorporates:
         *  RelationalOperator: '<S200>/Relational Operator'
         *  Sum: '<S200>/Subtract6'
         */
        if (!(AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_c <=
              AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116_b)) {
          AUAV_V3_TestSensors_B.Merge_o[0] = rtb_P32[0];
          AUAV_V3_TestSensors_B.Merge_o[1] = rtb_P32[1];
          AUAV_V3_TestSensors_B.Merge_o[2] = 0.0F -
            AUAV_V3_TestSensors_B.sf_ZerooutZ2.P[2];
        }

        /* End of Switch: '<S200>/Switch1' */
        AUAV_V3_TestSensors_B.Merge3 = 2U;

        /* End of Outputs for SubSystem: '<S181>/Intersection. Circular Navigation' */
      }

      /* End of If: '<S181>/If' */

      /* Delay: '<S181>/Integer Delay' */
      AUAV_V3_TestSensors_B.Merge2 =
        AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_h;
      AUAV_V3_TestSensors_B.Merge4 = 0U;

      /* Update for Delay: '<S198>/Integer Delay1' incorporates:
       *  Constant: '<S181>/RTB1'
       */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_a = rtb_Product3_j0[2];

      /* Update for Delay: '<S181>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_h = rtb_Product3_bo;

      /* End of Outputs for SubSystem: '<S180>/Circle Navigation' */
    }
  }

  /* End of If: '<S180>/Determine Overall Nav by the Nav Mode' */

  /* MATLAB Function: '<S277>/Zero out Z1' */
  AUAV_V3_TestSens_ZerooutZ1(AUAV_V3_TestSensors_B.Merge_o,
    &AUAV_V3_TestSensors_B.sf_ZerooutZ1_i);

  /* MATLAB Function: '<S282>/Embedded MATLAB Function' */
  A_EmbeddedMATLABFunction_b(AUAV_V3_TestSensors_B.sf_ZerooutZ1_i.P,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_nv);

  /* MATLAB Function: '<S283>/negprotect' */
  AUAV_V3_TestS_negprotect_l
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_nv.xDoty,
     &AUAV_V3_TestSensors_B.sf_negprotect_o);

  /* S-Function (MCHP_C_function_Call): '<S283>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_p = mySqrt(
    AUAV_V3_TestSensors_B.sf_negprotect_o.zpVal
    );

  /* MATLAB Function: '<S269>/negprotect' incorporates:
   *  Product: '<S269>/Product1'
   */
  AUAV_V3_TestSe_negprotect3(AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_p *
    AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116,
    &AUAV_V3_TestSensors_B.sf_negprotect_h);

  /* DeadZone: '<S269>/Dead Zone' */
  if (AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal > 0.1F) {
    rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal - 0.1F;
  } else if (AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal >= -0.1F) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal - -0.1F;
  }

  /* End of DeadZone: '<S269>/Dead Zone' */

  /* Switch: '<S269>/Switch' incorporates:
   *  Constant: '<S269>/cos(pi//2)'
   *  Constant: '<S269>/sin(pi//2)'
   *  Constant: '<S276>/Constant'
   *  Product: '<S269>/Divide1'
   *  Product: '<S269>/Divide2'
   *  Product: '<S275>/Product'
   *  Product: '<S275>/Product1'
   *  Product: '<S278>/Product'
   *  Product: '<S278>/Product1'
   *  RelationalOperator: '<S276>/Compare'
   *  Sum: '<S275>/Subtract'
   *  Sum: '<S278>/Subtract'
   *  Switch: '<S269>/Switch2'
   */
  if ((rtb_RhhcosphicoslambXe == 0.0F) > 0) {
    AUAV_V3_TestSensors_B.Switch = 1.0F;
    AUAV_V3_TestSensors_B.Switch2 = 0.0F;
  } else {
    AUAV_V3_TestSensors_B.Switch = (AUAV_V3_TestSensors_B.Merge_o[1] *
      AUAV_V3_TestSensors_B.sf_ZerooutZ1.P[0] - AUAV_V3_TestSensors_B.Merge_o[0]
      * AUAV_V3_TestSensors_B.sf_ZerooutZ1.P[1]) /
      AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal;
    AUAV_V3_TestSensors_B.Switch2 = (AUAV_V3_TestSensors_B.Merge_o[0] *
      AUAV_V3_TestSensors_B.sf_ZerooutZ1.P[0] + AUAV_V3_TestSensors_B.Merge_o[1]
      * AUAV_V3_TestSensors_B.sf_ZerooutZ1.P[1]) * (1.0F /
      AUAV_V3_TestSensors_B.sf_negprotect_h.zpVal);
  }

  /* End of Switch: '<S269>/Switch' */

  /* S-Function (MCHP_C_function_Call): '<S273>/myAtan2() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myAtan2apUtilscupdated5116 = myAtan2(
    AUAV_V3_TestSensors_B.Switch
    , AUAV_V3_TestSensors_B.Switch2
    );

  /* Delay: '<S180>/Integer Delay' */
  AUAV_V3_TestSensors_B.Merge3 = AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_c;

  /* Delay: '<S180>/Integer Delay1' */
  AUAV_V3_TestSensors_B.Merge4 =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_o;

  /* MATLAB Function: '<S180>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S180>/Data Type Conversion5'
   *  DataTypeConversion: '<S180>/Data Type Conversion6'
   *  Gain: '<S184>/Rad2Deg'
   */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun1': '<S192>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S192>:1:4' */
  AUAV_V3_TestSensors_B.y_b[0] = 57.2957802F *
    AUAV_V3_TestSensors_B.myAtan2apUtilscupdated5116;
  AUAV_V3_TestSensors_B.y_b[1] = AUAV_V3_TestSensors_B.Merge2;
  AUAV_V3_TestSensors_B.y_b[2] = AUAV_V3_TestSensors_B.Merge3;
  AUAV_V3_TestSensors_B.y_b[3] = AUAV_V3_TestSensors_B.Merge4;

  /* S-Function (MCHP_C_function_Call): '<S180>/getXYZ() navsupport.c [updated 5.1.16]' */
  setNavNav(
            &AUAV_V3_TestSensors_B.y_b[0]
            );

  /* MATLAB Function: '<S270>/negprotect' incorporates:
   *  DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON'
   */
  AUAV_V3_TestSe_negprotect3(mlParamInterface.param[19],
    &AUAV_V3_TestSensors_B.sf_negprotect_b);

  /* S-Function (MCHP_C_function_Call): '<S272>/myAbs() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116 = myAbs(
    AUAV_V3_TestSensors_B.myAtan2apUtilscupdated5116
    );

  /* Switch: '<S184>/Switch' incorporates:
   *  Constant: '<S184>/Constant10'
   *  Constant: '<S293>/Constant'
   *  Gain: '<S270>/Gain'
   *  Product: '<S270>/Divide'
   *  RelationalOperator: '<S184>/Relational Operator'
   *  RelationalOperator: '<S293>/Compare'
   *  Switch: '<S274>/Switch1'
   */
  if (AUAV_V3_TestSensors_B.myAbsapUtilscupdated5116 <= 1.57079637F) {
    rtb_Sum1_mj = AUAV_V3_TestSensors_B.Switch *
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116 /
      AUAV_V3_TestSensors_B.sf_negprotect_b.zpVal * 2.0F;
  } else if ((AUAV_V3_TestSensors_B.Switch < 0.0F) > 0) {
    /* Switch: '<S274>/Switch1' incorporates:
     *  Constant: '<S274>/Constant9'
     */
    rtb_Sum1_mj = -4.57681F;
  } else {
    /* Switch: '<S274>/Switch1' incorporates:
     *  Constant: '<S274>/Constant1'
     */
    rtb_Sum1_mj = 4.57681F;
  }

  /* End of Switch: '<S184>/Switch' */

  /* Product: '<S271>/Product' incorporates:
   *  Constant: '<S271>/Constant'
   */
  AUAV_V3_TestSensors_B.Product = rtb_Sum1_mj / 9.815F;

  /* S-Function (MCHP_C_function_Call): '<S289>/myAtan() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116 = myAtan(
    AUAV_V3_TestSensors_B.Product
    );

  /* Saturate: '<S271>/Bank  Limit Command' */
  if (AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116 > 0.436332315F) {
    rtb_cosphi = 0.436332315F;
  } else if (AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116 < -0.436332315F) {
    rtb_cosphi = -0.436332315F;
  } else {
    rtb_cosphi = AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116;
  }

  /* End of Saturate: '<S271>/Bank  Limit Command' */

  /* InitialCondition: '<S189>/IC3' */
  if (AUAV_V3_TestSensors_DWork.IC3_FirstOutputTime) {
    AUAV_V3_TestSensors_DWork.IC3_FirstOutputTime = false;
    AUAV_V3_TestSensors_B.IC3 = 0.0F;
  } else {
    AUAV_V3_TestSensors_B.IC3 = AUAV_V3_TestSensors_B.NumericalUnity[0];
  }

  /* End of InitialCondition: '<S189>/IC3' */

  /* Update for Delay: '<S464>/Delay' */
  /* MATLAB Function 'Inner Loop// Navigation/Navigation Encaps [updated 4.28.16]/Navigation/myMux Fun3': '<S194>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S194>:1:5' */
  AUAV_V3_TestSensors_DWork.Delay_DSTATE_e = rtb_IC1;

  /* Update for Delay: '<S180>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_c = AUAV_V3_TestSensors_B.WP0;

  /* Update for Delay: '<S180>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_o = AUAV_V3_TestSensors_B.WP1;

  /* End of Outputs for SubSystem: '<S6>/Navigation Encaps [updated 4.28.16]' */

  /* Switch: '<S73>/Switch3' incorporates:
   *  Delay: '<S73>/Integer Delay3'
   *  RelationalOperator: '<S73>/Relational Operator2'
   */
  if ((AUAV_V3_TestSensors_B.sf_myMuxFun1_e.y[2] ==
       AUAV_V3_TestSensors_B.sf_myMuxFun1_e.y[2]) > 0) {
    rtb_Switch3 = AUAV_V3_TestSensors_B.sf_myMuxFun1_e.y[2];
  } else {
    rtb_Switch3 = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE;
  }

  /* End of Switch: '<S73>/Switch3' */

  /* Product: '<S21>/Product1' */
  for (i = 0; i < 3; i++) {
    rtb_Product2_c[i] = rtb_MathFunction[i + 6] * AUAV_V3_TestSensors_B.y_k[5] +
      (rtb_MathFunction[i + 3] * AUAV_V3_TestSensors_B.y_k[4] +
       rtb_MathFunction[i] * AUAV_V3_TestSensors_B.y_k[3]);
  }

  /* End of Product: '<S21>/Product1' */

  /* MATLAB Function: '<S21>/myMux Fun2' incorporates:
   *  MATLAB Function: '<S21>/myMux Fun1'
   */
  /* MATLAB Function 'get Nav Vars [updated 4.28.16]/myMux Fun2': '<S705>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S705>:1:5' */
  AUAV_V3_TestSensors_B.y_j[0] = rtb_y_jl_idx_0;
  AUAV_V3_TestSensors_B.y_j[1] = rtb_y_jl_idx_1;
  AUAV_V3_TestSensors_B.y_j[2] =
    AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction1.y;
  AUAV_V3_TestSensors_B.y_j[3] = rtb_Product2_c[0];
  AUAV_V3_TestSensors_B.y_j[4] = rtb_Product2_c[1];
  AUAV_V3_TestSensors_B.y_j[5] = rtb_Product2_c[2];

  /* Outputs for Atomic SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S166>/[0 1000]' */
  if (rtb_Switch3 > 1000.0F) {
    rtb_Sum1_mj = 1000.0F;
  } else if (rtb_Switch3 < 0.0F) {
    rtb_Sum1_mj = 0.0F;
  } else {
    rtb_Sum1_mj = rtb_Switch3;
  }

  /* Sum: '<S166>/Add' incorporates:
   *  Constant: '<S166>/Constant'
   *  Constant: '<S166>/Constant from Model'
   *  Gain: '<S171>/Unit Conversion'
   *  Product: '<S166>/Divide'
   *  Saturate: '<S166>/[0 1000]'
   */
  AUAV_V3_TestSensors_B.Add = 1.0F - 3.28084F * rtb_Sum1_mj / 145442.0F;

  /* S-Function (MCHP_C_function_Call): '<S172>/myPow() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myPowapUtilscupdated5116 = myPow(
    AUAV_V3_TestSensors_B.Add
    , 4.25587606F
    );

  /* Product: '<S166>/Divide1' incorporates:
   *  Constant: '<S166>/Rho_0 (Kg//m^3)'
   */
  rtb_Sum1_mj = AUAV_V3_TestSensors_B.myPowapUtilscupdated5116 * 1.225F;

  /* Saturate: '<S167>/[ 0.01 50000]' */
  if (rtb_Sum1_mj > 50000.0F) {
    rtb_Sum1_mj = 50000.0F;
  } else {
    if (rtb_Sum1_mj < 0.01F) {
      rtb_Sum1_mj = 0.01F;
    }
  }

  /* MATLAB Function: '<S174>/negprotect' incorporates:
   *  Constant: '<S167>/a'
   *  DataStoreRead: '<Root>/Get mlAirData'
   *  Gain: '<Root>/Gain1'
   *  Product: '<S167>/Divide2'
   *  Saturate: '<S167>/[ 0.01 50000]'
   */
  AUAV_V3_TestS_negprotect_l(2.0F * (100.0F * mlAirData.press_diff) /
    rtb_Sum1_mj, &AUAV_V3_TestSensors_B.sf_negprotect_l);

  /* S-Function (MCHP_C_function_Call): '<S174>/mySqrt() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_me = mySqrt(
    AUAV_V3_TestSensors_B.sf_negprotect_l.zpVal
    );

  /* Switch: '<S168>/Switch3' incorporates:
   *  RelationalOperator: '<S168>/Relational Operator2'
   */
  if ((AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_me ==
       AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_me) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_g =
      AUAV_V3_TestSensors_B.mySqrtapUtilscupdated5116_me;
  }

  /* End of Switch: '<S168>/Switch3' */

  /* MATLAB Function: '<S165>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S165>/Constant'
   *  Constant: '<S165>/Constant1'
   */
  AUA_EmbeddedMATLABFunction(AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_g,
    0.01, 1.0, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_n,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_n);

  /* Switch: '<S145>/Schedule LPF' incorporates:
   *  Constant: '<S169>/Constant'
   *  RelationalOperator: '<S169>/Compare'
   */
  if ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_g <= 5.0F) > 0) {
    rtb_Product3_bo = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_g;
  } else {
    rtb_Product3_bo = AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_n.y;
  }

  /* End of Switch: '<S145>/Schedule LPF' */

  /* Product: '<S150>/delta rise limit' incorporates:
   *  Constant: '<S140>/Constant3'
   *  SampleTimeMath: '<S150>/sample time'
   *
   * About '<S150>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_RhhcosphisinlambYe = 0.02F;

  /* Switch: '<S178>/Init' incorporates:
   *  UnitDelay: '<S178>/FixPt Unit Delay1'
   *  UnitDelay: '<S178>/FixPt Unit Delay2'
   */
  if (AUAV_V3_TestSensors_DWork.FixPtUnitDelay2_DSTATE != 0) {
    rtb_Deg2R1 = 100.0F;
  } else {
    rtb_Deg2R1 = AUAV_V3_TestSensors_DWork.FixPtUnitDelay1_DSTATE;
  }

  /* End of Switch: '<S178>/Init' */
  /* End of Outputs for SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Switch: '<S6>/Switch' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  if (rtb_IC1_a) {
    rtb_Sum1_mj = mlMidLevelCommands.hCommand;
  } else {
    rtb_Sum1_mj = AUAV_V3_TestSensors_B.Merge1;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Outputs for Atomic SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Sum: '<S150>/Difference Inputs1' */
  rtb_Sum1_mj -= rtb_Deg2R1;

  /* Switch: '<S177>/Switch2' incorporates:
   *  RelationalOperator: '<S177>/LowerRelop1'
   */
  if (!(rtb_Sum1_mj > 0.02F)) {
    /* Switch: '<S177>/Switch' incorporates:
     *  RelationalOperator: '<S177>/UpperRelop'
     */
    if (rtb_Sum1_mj < -0.03F) {
      rtb_RhhcosphisinlambYe = -0.03F;
    } else {
      rtb_RhhcosphisinlambYe = rtb_Sum1_mj;
    }

    /* End of Switch: '<S177>/Switch' */
  }

  /* End of Switch: '<S177>/Switch2' */

  /* Sum: '<S150>/Difference Inputs2' */
  AUAV_V3_TestSensors_DWork.FixPtUnitDelay1_DSTATE = rtb_RhhcosphisinlambYe +
    rtb_Deg2R1;

  /* Sum: '<S140>/Add2' */
  rtb_Deg2R1 = AUAV_V3_TestSensors_DWork.FixPtUnitDelay1_DSTATE - rtb_Switch3;

  /* Switch: '<S147>/Switch3' incorporates:
   *  RelationalOperator: '<S147>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_e = rtb_Deg2R1;
  }

  /* End of Switch: '<S147>/Switch3' */

  /* Product: '<S143>/Product1' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   */
  rtb_Sum1_mj = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_e *
    mlParamInterface.param[9];

  /* Sum: '<S143>/Sum2' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Gain: '<S143>/Gain'
   *  Memory: '<S143>/Memory1'
   *  Product: '<S143>/Product4'
   */
  rtb_Deg2R1 = (0.01F * AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_e *
                mlParamInterface.param[10] +
                AUAV_V3_TestSensors_DWork.Memory1_PreviousInput) + rtb_Sum1_mj;

  /* Switch: '<S143>/AntiWindup' incorporates:
   *  Constant: '<S143>/Constant5'
   *  Logic: '<S143>/Logical Operator'
   *  RelationalOperator: '<S143>/Relational Operator'
   *  RelationalOperator: '<S143>/Relational Operator1'
   */
  if ((rtb_Deg2R1 > -0.261799395F) && (rtb_Deg2R1 < 0.261799395F)) {
    rtb_Deg2R1 = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_e;
  } else {
    rtb_Deg2R1 = 0.0F;
  }

  /* End of Switch: '<S143>/AntiWindup' */

  /* Switch: '<S160>/Switch3' incorporates:
   *  RelationalOperator: '<S160>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_a = rtb_Deg2R1;
  }

  /* End of Switch: '<S160>/Switch3' */

  /* Switch: '<S158>/Switch1' incorporates:
   *  Constant: '<S158>/Constant'
   *  Constant: '<S158>/Constant1'
   *  Constant: '<S158>/Constant2'
   *  Constant: '<S158>/Constant3'
   *  Constant: '<S158>/Constant5'
   *  Delay: '<S158>/Integer Delay'
   *  Delay: '<S158>/Integer Delay1'
   *  Delay: '<S158>/Integer Delay2'
   *  Product: '<S158>/Product'
   *  Product: '<S158>/Product1'
   *  Product: '<S158>/Product2'
   *  Product: '<S158>/Product3'
   *  Sum: '<S158>/Subtract'
   *  Sum: '<S158>/Subtract1'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_a *
                   0.333333343F +
                   AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_a * 1.33333337F)
                  + AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_h[0] *
                  0.333333343F) * 0.005F +
      AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE;
  }

  /* End of Switch: '<S158>/Switch1' */

  /* Switch: '<S159>/Switch3' incorporates:
   *  RelationalOperator: '<S159>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_av = rtb_Deg2R1;
  }

  /* End of Switch: '<S159>/Switch3' */

  /* Switch: '<S143>/On//Off' incorporates:
   *  Constant: '<S143>/Constant1'
   *  DataStoreRead: '<S10>/PAR_PID_HE_TO_PI_P PAR_PID_HE_TO_PI_I'
   *  Product: '<S143>/Product'
   *  Sum: '<S143>/Add2'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput = 0.0F;
  } else {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_av *
      mlParamInterface.param[10] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S143>/On//Off' */

  /* Switch: '<S157>/Switch2' incorporates:
   *  RelationalOperator: '<S157>/LowerRelop1'
   *  RelationalOperator: '<S157>/UpperRelop'
   *  Switch: '<S157>/Switch'
   */
  if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput > 0.261799395F) {
    /* Saturate: '<S140>/Theta_c Limit' */
    rtb_Deg2R1 = 0.261799395F;
  } else if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput < -0.261799395F) {
    /* Switch: '<S157>/Switch' incorporates:
     *  Saturate: '<S140>/Theta_c Limit'
     */
    rtb_Deg2R1 = -0.261799395F;
  } else {
    /* Saturate: '<S140>/Theta_c Limit' incorporates:
     *  Switch: '<S157>/Switch'
     */
    rtb_Deg2R1 = AUAV_V3_TestSensors_DWork.Memory1_PreviousInput;
  }

  /* End of Switch: '<S157>/Switch2' */

  /* DataStoreWrite: '<S71>/mlNavigation' */
  mlNavigation.u_m = rtb_Product3_bo;
  mlNavigation.theta_c = rtb_Deg2R1;

  /* DataTypeConversion: '<S140>/Data Type Conversion' */
  rtb_RhhcosphicoslambXe = (real32_T)floor
    (AUAV_V3_TestSensors_DWork.FixPtUnitDelay1_DSTATE);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  /* DataStoreWrite: '<S71>/mlNavigation' incorporates:
   *  DataTypeConversion: '<S140>/Data Type Conversion'
   */
  mlNavigation.h_c = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)-(int16_T)
    (uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;

  /* Switch: '<S149>/Switch3' incorporates:
   *  RelationalOperator: '<S149>/Relational Operator2'
   */
  if ((AUAV_V3_TestSensors_B.y_j[0] == AUAV_V3_TestSensors_B.y_j[0]) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_c =
      AUAV_V3_TestSensors_B.y_j[0];
  }

  /* End of Switch: '<S149>/Switch3' */

  /* MATLAB Function: '<S141>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S141>/Constant'
   *  Constant: '<S141>/Constant1'
   */
  AUA_EmbeddedMATLABFunction(AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_c,
    0.01, 0.32, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_i);

  /* Sum: '<S140>/Add' incorporates:
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   */
  rtb_RhhcosphisinlambYe = mlMidLevelCommands.uCommand - rtb_Product3_bo;

  /* Sum: '<S142>/Add3' incorporates:
   *  Constant: '<S142>/SaturationLimit'
   */
  rtb_y_jl_idx_0 = 0.95F - AUAV_V3_TestSensors_B.DataTypeConversion;

  /* Switch: '<S146>/Switch3' incorporates:
   *  RelationalOperator: '<S146>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe == rtb_RhhcosphisinlambYe) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j = rtb_RhhcosphisinlambYe;
  }

  /* End of Switch: '<S146>/Switch3' */

  /* Sum: '<S142>/Add1' incorporates:
   *  Constant: '<S142>/delayTime'
   *  DataStoreRead: '<S10>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Delay: '<S142>/NDelays'
   *  Product: '<S142>/Product1'
   *  Product: '<S142>/Product2'
   *  Product: '<S142>/Product3'
   *  Sum: '<S142>/Sum'
   */
  rtb_Sum1_mj = (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j -
                 AUAV_V3_TestSensors_DWork.NDelays_DSTATE[0]) / 0.05F *
    mlParamInterface.param[2] + AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j
    * mlParamInterface.param[0];

  /* Sum: '<S142>/Sum2' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Gain: '<S142>/Gain'
   *  Memory: '<S142>/Memory1'
   *  Product: '<S142>/Product4'
   */
  rtb_IC4_idx_0 = (0.01F * AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j *
                   mlParamInterface.param[1] +
                   AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p) +
    rtb_Sum1_mj;

  /* Switch: '<S142>/AntiWindup' incorporates:
   *  Constant: '<S142>/Constant5'
   *  Constant: '<S142>/SaturationLimit'
   *  Constant: '<S142>/SaturationLimit1'
   *  Logic: '<S142>/Logical Operator'
   *  RelationalOperator: '<S142>/Relational Operator'
   *  RelationalOperator: '<S142>/Relational Operator1'
   *  Sum: '<S142>/Add3'
   *  Sum: '<S142>/Add4'
   */
  if ((rtb_IC4_idx_0 > 0.0F - AUAV_V3_TestSensors_B.DataTypeConversion) &&
      (rtb_IC4_idx_0 < 0.95F - AUAV_V3_TestSensors_B.DataTypeConversion)) {
    rtb_IC4_idx_0 = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j;
  } else {
    rtb_IC4_idx_0 = 0.0F;
  }

  /* End of Switch: '<S142>/AntiWindup' */

  /* Switch: '<S155>/Switch3' incorporates:
   *  RelationalOperator: '<S155>/Relational Operator2'
   */
  if ((rtb_IC4_idx_0 == rtb_IC4_idx_0) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_cl = rtb_IC4_idx_0;
  }

  /* End of Switch: '<S155>/Switch3' */

  /* Switch: '<S154>/Switch1' incorporates:
   *  Constant: '<S154>/Constant'
   *  Constant: '<S154>/Constant1'
   *  Constant: '<S154>/Constant2'
   *  Constant: '<S154>/Constant3'
   *  Constant: '<S154>/Constant5'
   *  Delay: '<S154>/Integer Delay'
   *  Delay: '<S154>/Integer Delay1'
   *  Delay: '<S154>/Integer Delay2'
   *  Product: '<S154>/Product'
   *  Product: '<S154>/Product1'
   *  Product: '<S154>/Product2'
   *  Product: '<S154>/Product3'
   *  Sum: '<S154>/Subtract'
   *  Sum: '<S154>/Subtract1'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_IC4_idx_0 = 0.0F;
  } else {
    rtb_IC4_idx_0 = ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_cl *
                      0.333333343F +
                      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_l *
                      1.33333337F) +
                     AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_hy[0] *
                     0.333333343F) * 0.005F +
      AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_f;
  }

  /* End of Switch: '<S154>/Switch1' */

  /* Switch: '<S156>/Switch3' incorporates:
   *  RelationalOperator: '<S156>/Relational Operator2'
   */
  if ((rtb_IC4_idx_0 == rtb_IC4_idx_0) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_m = rtb_IC4_idx_0;
  }

  /* End of Switch: '<S156>/Switch3' */

  /* Switch: '<S142>/On//Off' incorporates:
   *  Constant: '<S142>/Constant1'
   *  DataStoreRead: '<S10>/PAR_PID_AIRSPEED_P PAR_PID_AIRSPEED_I PAR_PID_AIRSPEED_D'
   *  Product: '<S142>/Product'
   *  Sum: '<S142>/Add2'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p = 0.0F;
  } else {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_m * mlParamInterface.param
      [1] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S142>/On//Off' */

  /* Switch: '<S153>/Switch2' incorporates:
   *  Constant: '<S142>/SaturationLimit'
   *  RelationalOperator: '<S153>/LowerRelop1'
   *  Sum: '<S142>/Add3'
   */
  if (!(AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p > 0.95F -
        AUAV_V3_TestSensors_B.DataTypeConversion)) {
    /* Switch: '<S153>/Switch' incorporates:
     *  Constant: '<S142>/SaturationLimit1'
     *  RelationalOperator: '<S153>/UpperRelop'
     *  Sum: '<S142>/Add4'
     */
    if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p < 0.0F -
        AUAV_V3_TestSensors_B.DataTypeConversion) {
      rtb_y_jl_idx_0 = 0.0F - AUAV_V3_TestSensors_B.DataTypeConversion;
    } else {
      rtb_y_jl_idx_0 = AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_p;
    }

    /* End of Switch: '<S153>/Switch' */
  }

  /* End of Switch: '<S153>/Switch2' */

  /* Sum: '<S140>/Add1' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_PITC_DT_FF'
   *  Product: '<S140>/Product2'
   */
  rtb_y_jl_idx_0 = (rtb_y_jl_idx_0 + AUAV_V3_TestSensors_B.DataTypeConversion) +
    mlParamInterface.param[15] * AUAV_V3_TestSensors_B.y_j[1];

  /* Sum: '<S140>/Add3' */
  rtb_Deg2R1 -= AUAV_V3_TestSensors_B.y_j[1];

  /* Sum: '<S144>/Add3' incorporates:
   *  Constant: '<S144>/SaturationLimit'
   */
  rtb_IC4_idx_0 = 0.401425719F - AUAV_V3_TestSensors_B.DataTypeConversion_f;

  /* Switch: '<S148>/Switch3' incorporates:
   *  RelationalOperator: '<S148>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b = rtb_Deg2R1;
  }

  /* End of Switch: '<S148>/Switch3' */

  /* Sum: '<S144>/Add1' incorporates:
   *  Constant: '<S144>/delayTime'
   *  DataStoreRead: '<S10>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Delay: '<S144>/NDelays'
   *  Product: '<S144>/Product1'
   *  Product: '<S144>/Product2'
   *  Product: '<S144>/Product3'
   *  Sum: '<S144>/Sum'
   */
  rtb_Sum1_mj = (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b -
                 AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[0]) / 0.05F *
    mlParamInterface.param[5] + AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b
    * mlParamInterface.param[3];

  /* Sum: '<S144>/Sum2' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Gain: '<S144>/Gain'
   *  Memory: '<S144>/Memory1'
   *  Product: '<S144>/Product4'
   */
  rtb_Deg2R1 = (0.01F * AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b *
                mlParamInterface.param[4] +
                AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o) + rtb_Sum1_mj;

  /* Switch: '<S144>/AntiWindup' incorporates:
   *  Constant: '<S144>/Constant5'
   *  Constant: '<S144>/SaturationLimit'
   *  Constant: '<S144>/SaturationLimit1'
   *  Logic: '<S144>/Logical Operator'
   *  RelationalOperator: '<S144>/Relational Operator'
   *  RelationalOperator: '<S144>/Relational Operator1'
   *  Sum: '<S144>/Add3'
   *  Sum: '<S144>/Add4'
   */
  if ((rtb_Deg2R1 > -0.401425719F - AUAV_V3_TestSensors_B.DataTypeConversion_f) &&
      (rtb_Deg2R1 < 0.401425719F - AUAV_V3_TestSensors_B.DataTypeConversion_f))
  {
    rtb_Deg2R1 = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b;
  } else {
    rtb_Deg2R1 = 0.0F;
  }

  /* End of Switch: '<S144>/AntiWindup' */

  /* Switch: '<S163>/Switch3' incorporates:
   *  RelationalOperator: '<S163>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_o = rtb_Deg2R1;
  }

  /* End of Switch: '<S163>/Switch3' */

  /* Switch: '<S162>/Switch1' incorporates:
   *  Constant: '<S162>/Constant'
   *  Constant: '<S162>/Constant1'
   *  Constant: '<S162>/Constant2'
   *  Constant: '<S162>/Constant3'
   *  Constant: '<S162>/Constant5'
   *  Delay: '<S162>/Integer Delay'
   *  Delay: '<S162>/Integer Delay1'
   *  Delay: '<S162>/Integer Delay2'
   *  Product: '<S162>/Product'
   *  Product: '<S162>/Product1'
   *  Product: '<S162>/Product2'
   *  Product: '<S162>/Product3'
   *  Sum: '<S162>/Subtract'
   *  Sum: '<S162>/Subtract1'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_o *
                   0.333333343F +
                   AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_e * 1.33333337F)
                  + AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_k[0] *
                  0.333333343F) * 0.005F +
      AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_b;
  }

  /* End of Switch: '<S162>/Switch1' */

  /* Switch: '<S164>/Switch3' incorporates:
   *  RelationalOperator: '<S164>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ep = rtb_Deg2R1;
  }

  /* End of Switch: '<S164>/Switch3' */

  /* Switch: '<S144>/On//Off' incorporates:
   *  Constant: '<S144>/Constant1'
   *  DataStoreRead: '<S10>/PAR_PID_PITCH_FO_P PAR_PID_PITCH_FO_I PAR_PID_PITCH_FO_D'
   *  Product: '<S144>/Product'
   *  Sum: '<S144>/Add2'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o = 0.0F;
  } else {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ep *
      mlParamInterface.param[4] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S144>/On//Off' */

  /* Switch: '<S161>/Switch2' incorporates:
   *  Constant: '<S144>/SaturationLimit'
   *  RelationalOperator: '<S161>/LowerRelop1'
   *  Sum: '<S144>/Add3'
   */
  if (!(AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o > 0.401425719F -
        AUAV_V3_TestSensors_B.DataTypeConversion_f)) {
    /* Switch: '<S161>/Switch' incorporates:
     *  Constant: '<S144>/SaturationLimit1'
     *  RelationalOperator: '<S161>/UpperRelop'
     *  Sum: '<S144>/Add4'
     */
    if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o < -0.401425719F -
        AUAV_V3_TestSensors_B.DataTypeConversion_f) {
      rtb_IC4_idx_0 = -0.401425719F - AUAV_V3_TestSensors_B.DataTypeConversion_f;
    } else {
      rtb_IC4_idx_0 = AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_o;
    }

    /* End of Switch: '<S161>/Switch' */
  }

  /* End of Switch: '<S161>/Switch2' */

  /* Saturate: '<S140>/[-60 60]' */
  if (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i.y > 60.0F) {
    AUAV_V3_TestSensors_B.u060 = 60.0F;
  } else if (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i.y < -60.0F) {
    AUAV_V3_TestSensors_B.u060 = -60.0F;
  } else {
    AUAV_V3_TestSensors_B.u060 =
      AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_i.y;
  }

  /* End of Saturate: '<S140>/[-60 60]' */

  /* S-Function (MCHP_C_function_Call): '<S151>/myCos() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myCosapUtilscupdated5116 = myCos(
    AUAV_V3_TestSensors_B.u060
    );

  /* Sum: '<S140>/Add4' incorporates:
   *  Constant: '<S140>/Constant2'
   *  Constant: '<S140>/Constant4'
   *  DataStoreRead: '<S10>/PAR_PID_HEI_ERR_FF'
   *  Product: '<S140>/Product'
   *  Product: '<S140>/Product1'
   *  Sum: '<S140>/Add5'
   */
  rtb_IC4_idx_0 = (1.0F / AUAV_V3_TestSensors_B.myCosapUtilscupdated5116 - 1.0F)
    * mlParamInterface.param[11] + (rtb_IC4_idx_0 +
    AUAV_V3_TestSensors_B.DataTypeConversion_f);

  /* Update for UnitDelay: '<S178>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S178>/FixPt Constant'
   */
  AUAV_V3_TestSensors_DWork.FixPtUnitDelay2_DSTATE = 0U;

  /* Update for Delay: '<S158>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_av;

  /* Update for Delay: '<S158>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_a =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_a;

  /* Update for Delay: '<S158>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_h[0] =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_h[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_h[1] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_a;

  /* Update for Delay: '<S142>/NDelays' */
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE[0] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE[1];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE[1] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE[2];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE[2] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE[3];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE[3] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE[4];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE[4] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_j;

  /* Update for Delay: '<S154>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_f =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_m;

  /* Update for Delay: '<S154>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_l =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_cl;

  /* Update for Delay: '<S154>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_hy[0] =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_hy[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_hy[1] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_cl;

  /* Update for Delay: '<S144>/NDelays' */
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[0] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[1];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[1] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[2];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[2] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[3];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[3] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[4];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_c[4] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_b;

  /* Update for Delay: '<S162>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_b =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ep;

  /* Update for Delay: '<S162>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_e =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_o;

  /* Update for Delay: '<S162>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_k[0] =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_k[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_k[1] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_o;

  /* End of Outputs for SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* S-Function (MCHP_C_function_Call): '<S75>/myCos() aLib.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myCosaLibcupdated5116 = myCos(
    AUAV_V3_TestSensors_B.y_j[0]
    );

  /* Product: '<S68>/Product' */
  rtb_Sum1_mj = AUAV_V3_TestSensors_B.y_j[5] *
    AUAV_V3_TestSensors_B.myCosaLibcupdated5116;

  /* S-Function (MCHP_C_function_Call): '<S76>/C Function Call1' */
  AUAV_V3_TestSensors_B.CFunctionCall1 = mySin(
    AUAV_V3_TestSensors_B.y_j[0]
    );

  /* Sum: '<S68>/Subtract' incorporates:
   *  Product: '<S68>/Product1'
   */
  rtb_Deg2R_h_idx_0 = AUAV_V3_TestSensors_B.y_j[4] *
    AUAV_V3_TestSensors_B.CFunctionCall1 + rtb_Sum1_mj;

  /* Switch: '<S74>/Switch3' incorporates:
   *  Delay: '<S74>/Integer Delay3'
   *  RelationalOperator: '<S74>/Relational Operator2'
   */
  if (!((rtb_Deg2R_h_idx_0 == rtb_Deg2R_h_idx_0) > 0)) {
    rtb_Deg2R_h_idx_0 = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k;
  }

  /* End of Switch: '<S74>/Switch3' */

  /* DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF' incorporates:
   *  Inport: '<S92>/In1'
   *  Logic: '<S80>/Logical Operator'
   */
  rtb_IC4_idx_1 = mlParamInterface.param[22];
  rtb_Product = mlParamInterface.param[23];

  /* Outputs for Enabled SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  EnablePort: '<S69>/Enable'
   */
  if (mlParamInterface.param[25L] > 0.0F) {
    if (!AUAV_V3_TestSensors_DWork.L1OutputFeedbackControllerWithP) {
      /* InitializeConditions for UnitDelay: '<S89>/UD' */
      AUAV_V3_TestSensors_DWork.UD_DSTATE_ff = 0.0F;

      /* InitializeConditions for Delay: '<S79>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_an = 0.0F;

      /* InitializeConditions for Delay: '<S79>/Integer Delay1' */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S81>/Integer Delay3' */
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_fs = 0.0F;

      /* InitializeConditions for UnitDelay: '<S99>/UD' */
      AUAV_V3_TestSensors_DWork.UD_DSTATE_j = 0.0F;

      /* InitializeConditions for Delay: '<S83>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_k = 0.0F;

      /* InitializeConditions for Delay: '<S83>/Integer Delay1' */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_b = 0.0F;

      /* InitializeConditions for Delay: '<S69>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S69>/Integer Delay1' */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e = 0.0F;

      /* InitializeConditions for Merge: '<S80>/Merge' */
      if (rtmIsFirstInitCond(AUAV_V3_TestSensors_M)) {
        AUAV_V3_TestSensors_B.Merge_oo = 0.0F;
      }

      /* End of InitializeConditions for Merge: '<S80>/Merge' */

      /* InitializeConditions for Delay: '<S82>/Integer Delay3' */
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_h = 0.0F;

      /* InitializeConditions for Delay: '<S84>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_p = 0.0F;

      /* InitializeConditions for Delay: '<S84>/Integer Delay1' */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[0] = 0.0F;
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[1] = 0.0F;

      /* InitializeConditions for Delay: '<S84>/Integer Delay2' */
      AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_g = 0.0F;

      /* InitializeConditions for Delay: '<S100>/Integer Delay3' */
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_nz = 0.0F;

      /* InitializeConditions for Delay: '<S101>/Integer Delay3' */
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_hh = 0.0F;
      AUAV_V3_TestSensors_DWork.L1OutputFeedbackControllerWithP = true;
    }

    /* InitialCondition: '<S79>/IC' */
    if (AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_g) {
      AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_g = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S86>/Abs1' incorporates:
       *  Sum: '<S89>/Diff'
       *  UnitDelay: '<S89>/UD'
       */
      rtb_Sum1_mj = (real32_T)fabs(mlParamInterface.param[22] -
        AUAV_V3_TestSensors_DWork.UD_DSTATE_ff);

      /* Saturate: '<S86>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S86>/Saturation1' */
    }

    /* End of InitialCondition: '<S79>/IC' */

    /* Outputs for Enabled SubSystem: '<S79>/Compute Coef' incorporates:
     *  EnablePort: '<S85>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Gain: '<S85>/-T' */
      AUAV_V3_TestSensors_B.T_m = -0.01F * mlParamInterface.param[22];

      /* S-Function (MCHP_C_function_Call): '<S87>/myExp() apUtils.c [updated 5.1.16]' */
      AUAV_V3_TestSensors_B.myExpapUtilscupdated5116_f = myExp(
        AUAV_V3_TestSensors_B.T_m
        );

      /* SignalConversion: '<S88>/Numerical Unity' */
      AUAV_V3_TestSensors_B.NumericalUnity_a =
        AUAV_V3_TestSensors_B.myExpapUtilscupdated5116_f;

      /* Sum: '<S85>/1-c' incorporates:
       *  Constant: '<S85>/Constant'
       */
      AUAV_V3_TestSensors_B.c_d = (real32_T)(1.0 -
        AUAV_V3_TestSensors_B.NumericalUnity_a);
    }

    /* End of Outputs for SubSystem: '<S79>/Compute Coef' */

    /* Sum: '<S79>/Subtract' incorporates:
     *  Delay: '<S79>/Integer Delay'
     *  Delay: '<S79>/Integer Delay1'
     *  Product: '<S79>/Divide'
     *  Product: '<S79>/Divide1'
     */
    rtb_Subtract_od = AUAV_V3_TestSensors_B.c_d *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_an +
      AUAV_V3_TestSensors_B.NumericalUnity_a *
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_i;

    /* Switch: '<S81>/Switch3' incorporates:
     *  Delay: '<S81>/Integer Delay3'
     *  RelationalOperator: '<S81>/Relational Operator2'
     */
    if ((rtb_Deg2R_h_idx_0 == rtb_Deg2R_h_idx_0) > 0) {
      rtb_Switch3_gd = rtb_Deg2R_h_idx_0;
    } else {
      rtb_Switch3_gd = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_fs;
    }

    /* End of Switch: '<S81>/Switch3' */

    /* InitialCondition: '<S83>/IC' */
    if (AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_f) {
      AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_f = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S96>/Abs1' incorporates:
       *  Sum: '<S99>/Diff'
       *  UnitDelay: '<S99>/UD'
       */
      rtb_Sum1_mj = (real32_T)fabs(mlParamInterface.param[23] -
        AUAV_V3_TestSensors_DWork.UD_DSTATE_j);

      /* Saturate: '<S96>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S96>/Saturation1' */
    }

    /* End of InitialCondition: '<S83>/IC' */

    /* Outputs for Enabled SubSystem: '<S83>/Compute Coef' incorporates:
     *  EnablePort: '<S95>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Gain: '<S95>/-T' */
      AUAV_V3_TestSensors_B.T = -0.01F * mlParamInterface.param[23];

      /* S-Function (MCHP_C_function_Call): '<S97>/myExp() apUtils.c [updated 5.1.16]' */
      AUAV_V3_TestSensors_B.myExpapUtilscupdated5116 = myExp(
        AUAV_V3_TestSensors_B.T
        );

      /* SignalConversion: '<S98>/Numerical Unity' */
      AUAV_V3_TestSensors_B.NumericalUnity_n =
        AUAV_V3_TestSensors_B.myExpapUtilscupdated5116;

      /* Sum: '<S95>/1-c' incorporates:
       *  Constant: '<S95>/Constant'
       */
      AUAV_V3_TestSensors_B.c_h = (real32_T)(1.0 -
        AUAV_V3_TestSensors_B.NumericalUnity_n);
    }

    /* End of Outputs for SubSystem: '<S83>/Compute Coef' */

    /* Sum: '<S83>/Subtract' incorporates:
     *  Delay: '<S83>/Integer Delay'
     *  Delay: '<S83>/Integer Delay1'
     *  Product: '<S83>/Divide'
     *  Product: '<S83>/Divide1'
     */
    rtb_Subtract_a = AUAV_V3_TestSensors_B.c_h *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_k +
      AUAV_V3_TestSensors_B.NumericalUnity_n *
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_b;

    /* Gain: '<S69>/Gain' incorporates:
     *  Sum: '<S69>/Sum3'
     */
    rtb_Sum1_mj = -(rtb_Subtract_a - rtb_Switch3_gd);

    /* Product: '<S94>/Divide4' incorporates:
     *  Constant: '<S94>/Constant'
     *  Delay: '<S69>/Integer Delay1'
     */
    rtb_RhhcosphisinlambYe = 0.25F *
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e * 2.0F;

    /* Product: '<S94>/Divide3' incorporates:
     *  Delay: '<S69>/Integer Delay1'
     *  Product: '<S94>/Divide'
     *  Sum: '<S94>/Subtract'
     */
    rtb_Deg2R1 = (AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e *
                  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e - 4.0F) /
      4.0F;

    /* Logic: '<S80>/Logical Operator2' incorporates:
     *  Constant: '<S90>/Constant'
     *  Constant: '<S91>/Constant'
     *  Product: '<S80>/Divide2'
     *  RelationalOperator: '<S90>/Compare'
     *  RelationalOperator: '<S91>/Compare'
     */
    rtb_Compare_j = (uint8_T)((rtb_RhhcosphisinlambYe * rtb_Sum1_mj > 0.0F) &&
      (rtb_Deg2R1 >= 0.0F));

    /* Outputs for Enabled SubSystem: '<S80>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S92>/Enable'
     */
    if (!(rtb_Compare_j != 0) > 0) {
      AUAV_V3_TestSensors_B.Merge_oo = rtb_Sum1_mj;
    }

    /* End of Outputs for SubSystem: '<S80>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S80>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S93>/Enable'
     */
    if (rtb_Compare_j > 0) {
      /* Signum: '<S93>/Sign' */
      if (rtb_RhhcosphisinlambYe < 0.0F) {
        rtb_RhhcosphisinlambYe = -1.0F;
      } else if (rtb_RhhcosphisinlambYe > 0.0F) {
        rtb_RhhcosphisinlambYe = 1.0F;
      } else {
        if (rtb_RhhcosphisinlambYe == 0.0F) {
          rtb_RhhcosphisinlambYe = 0.0F;
        }
      }

      /* Product: '<S93>/Divide2' incorporates:
       *  Constant: '<S93>/Constant'
       *  Product: '<S93>/Divide1'
       *  Signum: '<S93>/Sign'
       *  Sum: '<S93>/Subtract'
       */
      AUAV_V3_TestSensors_B.Merge_oo = (1.0F - rtb_Deg2R1 *
        rtb_RhhcosphisinlambYe) * rtb_Sum1_mj;
    }

    /* End of Outputs for SubSystem: '<S80>/Enabled Subsystem1' */

    /* Product: '<S69>/Projection' incorporates:
     *  Inport: '<S92>/In1'
     *  Logic: '<S80>/Logical Operator'
     */
    rtb_Projection = mlParamInterface.param[24] * AUAV_V3_TestSensors_B.Merge_oo;

    /* Sum: '<S69>/Sum4' incorporates:
     *  Delay: '<S69>/Integer Delay'
     */
    AUAV_V3_TestSensors_B.PsiDotLimit =
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_i - rtb_Subtract_od;

    /* Saturate: '<S69>/Psi Dot  Limit' */
    if (AUAV_V3_TestSensors_B.PsiDotLimit > 1.0F) {
      /* Sum: '<S69>/Sum4' */
      AUAV_V3_TestSensors_B.PsiDotLimit = 1.0F;
    } else {
      if (AUAV_V3_TestSensors_B.PsiDotLimit < -1.0F) {
        /* Sum: '<S69>/Sum4' */
        AUAV_V3_TestSensors_B.PsiDotLimit = -1.0F;
      }
    }

    /* End of Saturate: '<S69>/Psi Dot  Limit' */

    /* Sum: '<S69>/Sum2' incorporates:
     *  Delay: '<S69>/Integer Delay1'
     */
    rtb_Switch1_a = AUAV_V3_TestSensors_B.PsiDotLimit +
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e;

    /* Switch: '<S82>/Switch3' incorporates:
     *  Delay: '<S82>/Integer Delay3'
     *  RelationalOperator: '<S82>/Relational Operator2'
     */
    if ((rtb_Switch1_a == rtb_Switch1_a) > 0) {
      rtb_Switch3_kh = rtb_Switch1_a;
    } else {
      rtb_Switch3_kh = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_h;
    }

    /* End of Switch: '<S82>/Switch3' */

    /* Switch: '<S100>/Switch3' incorporates:
     *  Delay: '<S100>/Integer Delay3'
     *  RelationalOperator: '<S100>/Relational Operator2'
     */
    if (!((rtb_Projection == rtb_Projection) > 0)) {
      rtb_Projection = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_nz;
    }

    /* End of Switch: '<S100>/Switch3' */

    /* Switch: '<S84>/Switch1' incorporates:
     *  Constant: '<S84>/Constant5'
     *  Logic: '<S69>/Logical Operator'
     */
    if (!(mlParamInterface.param[25] != 0.0F) > 0) {
      rtb_Switch1_a = 0.0F;
    } else {
      /* Sum: '<S84>/Subtract1' incorporates:
       *  Constant: '<S84>/Constant'
       *  Constant: '<S84>/Constant1'
       *  Constant: '<S84>/Constant2'
       *  Constant: '<S84>/Constant3'
       *  Delay: '<S84>/Integer Delay'
       *  Delay: '<S84>/Integer Delay1'
       *  Delay: '<S84>/Integer Delay2'
       *  Product: '<S84>/Product'
       *  Product: '<S84>/Product1'
       *  Product: '<S84>/Product2'
       *  Product: '<S84>/Product3'
       *  Sum: '<S84>/Subtract'
       */
      rtb_Switch1_a = ((rtb_Projection * 0.333333343F +
                        AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_p *
                        1.33333337F) +
                       AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[0] *
                       0.333333343F) * 0.005F +
        AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_g;

      /* Saturate: '<S84>/[min max]' */
      if (rtb_Switch1_a > 2.0F) {
        rtb_Switch1_a = 2.0F;
      } else {
        if (rtb_Switch1_a < -2.0F) {
          rtb_Switch1_a = -2.0F;
        }
      }

      /* End of Saturate: '<S84>/[min max]' */
    }

    /* End of Switch: '<S84>/Switch1' */

    /* Switch: '<S101>/Switch3' incorporates:
     *  Delay: '<S101>/Integer Delay3'
     *  RelationalOperator: '<S101>/Relational Operator2'
     */
    if (!((rtb_Switch1_a == rtb_Switch1_a) > 0)) {
      rtb_Switch1_a = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_hh;
    }

    /* End of Switch: '<S101>/Switch3' */
  } else {
    if (AUAV_V3_TestSensors_DWork.L1OutputFeedbackControllerWithP) {
      AUAV_V3_TestSensors_DWork.L1OutputFeedbackControllerWithP = false;
    }
  }

  /* End of Outputs for SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */

  /* Product: '<S21>/Product2' incorporates:
   *  Sum: '<S21>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_P32[i] = rtb_MathFunction[i + 6] *
      AUAV_V3_TestSensors_B.DataTypeConversion3[2] + (rtb_MathFunction[i + 3] *
      AUAV_V3_TestSensors_B.DataTypeConversion3[1] + rtb_MathFunction[i] *
      AUAV_V3_TestSensors_B.DataTypeConversion3[0]);
  }

  /* End of Product: '<S21>/Product2' */

  /* Product: '<S21>/Product3' incorporates:
   *  Sum: '<S21>/Sum'
   */
  for (i = 0; i < 3; i++) {
    rtb_Merge_m[i] = rtb_MathFunction[i + 6] *
      AUAV_V3_TestSensors_B.DataTypeConversion1[2] + (rtb_MathFunction[i + 3] *
      AUAV_V3_TestSensors_B.DataTypeConversion1[1] + rtb_MathFunction[i] *
      AUAV_V3_TestSensors_B.DataTypeConversion1[0]);
  }

  /* End of Product: '<S21>/Product3' */

  /* Sum: '<S21>/Sum' */
  rtb_Product3_j0[1] = rtb_P32[1] + rtb_Merge_m[1];

  /* Outputs for Atomic SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Switch: '<S136>/Switch3' incorporates:
   *  RelationalOperator: '<S136>/Relational Operator2'
   */
  if ((rtb_cosphi == rtb_cosphi) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_mw = rtb_cosphi;
  }

  /* End of Switch: '<S136>/Switch3' */

  /* Saturate: '<S135>/bank Limit' */
  if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_mw > 0.436332315F) {
    AUAV_V3_TestSensors_B.bankLimit = 0.436332315F;
  } else if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_mw < -0.436332315F)
  {
    AUAV_V3_TestSensors_B.bankLimit = -0.436332315F;
  } else {
    AUAV_V3_TestSensors_B.bankLimit =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_mw;
  }

  /* End of Saturate: '<S135>/bank Limit' */

  /* S-Function (MCHP_C_function_Call): '<S138>/myTan() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myTanapUtilscupdated5116 = myTan(
    AUAV_V3_TestSensors_B.bankLimit
    );

  /* Switch: '<S137>/Switch3' incorporates:
   *  RelationalOperator: '<S137>/Relational Operator2'
   */
  if ((rtb_Product3_bo == rtb_Product3_bo) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_bs = rtb_Product3_bo;
  }

  /* End of Switch: '<S137>/Switch3' */

  /* Switch: '<S103>/Switch1' incorporates:
   *  Constant: '<S135>/Constant1'
   *  DataStoreRead: '<Root>/Get mlMidLevelCommands'
   *  Product: '<S135>/Divide'
   *  Product: '<S135>/Divide1'
   *  Saturate: '<S135>/[0 40]'
   */
  if (rtb_IC1_a) {
    rtb_Ze_b = mlMidLevelCommands.rCommand;
  } else {
    if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_bs > 40.0F) {
      /* Saturate: '<S135>/[0 40]' */
      rtb_Sum1_mj = 40.0F;
    } else if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_bs < 0.0F) {
      /* Saturate: '<S135>/[0 40]' */
      rtb_Sum1_mj = 0.0F;
    } else {
      /* Saturate: '<S135>/[0 40]' */
      rtb_Sum1_mj = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_bs;
    }

    rtb_Ze_b = 1.0F / rtb_Sum1_mj * 9.80665F *
      AUAV_V3_TestSensors_B.myTanapUtilscupdated5116;
  }

  /* End of Switch: '<S103>/Switch1' */

  /* Switch: '<S103>/Switch' incorporates:
   *  DataStoreRead: '<Root>/PAR_L1_OMEGA PAR_L1_M PAR_L1_GAMMA PAR_L1_ON_OFF'
   */
  if (mlParamInterface.param[25] > 0.3F) {
    rtb_RhhcosphisinlambYe = AUAV_V3_TestSensors_B.PsiDotLimit;
  } else {
    rtb_RhhcosphisinlambYe = rtb_Ze_b;
  }

  /* End of Switch: '<S103>/Switch' */

  /* Saturate: '<S102>/Psi Dot  Limit' */
  if (rtb_RhhcosphisinlambYe > 1.0F) {
    rtb_RhhcosphisinlambYe = 1.0F;
  } else {
    if (rtb_RhhcosphisinlambYe < -1.0F) {
      rtb_RhhcosphisinlambYe = -1.0F;
    }
  }

  /* End of Saturate: '<S102>/Psi Dot  Limit' */

  /* Outputs for Enabled SubSystem: '<S102>/Sideslip Compensation' incorporates:
   *  EnablePort: '<S109>/Enable'
   */
  /* DataStoreRead: '<Root>/PAR_NAV_L2_BASE PAR_NAV_PRETURN_K PAR_NAV_SSCOMP_ON' */
  if (mlParamInterface.param[21L] > 0.0F) {
    if (!AUAV_V3_TestSensors_DWork.SideslipCompensation_MODE) {
      /* InitializeConditions for UnitDelay: '<S126>/UD' */
      AUAV_V3_TestSensors_DWork.UD_DSTATE_f = 0.0F;

      /* InitializeConditions for Delay: '<S118>/Integer Delay' */
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_ed = 0.0F;

      /* InitializeConditions for Delay: '<S118>/Integer Delay1' */
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_n = 0.0F;

      /* InitializeConditions for Delay: '<S119>/Integer Delay3' */
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ph = 0.0F;
      AUAV_V3_TestSensors_DWork.SideslipCompensation_MODE = true;
    }

    /* MATLAB Function: '<S117>/negprotect' */
    AUAV_V3_TestSen_negprotect(rtb_Product3_bo,
      &AUAV_V3_TestSensors_B.sf_negprotect);

    /* Saturate: '<S117>/bank Limit' */
    if (AUAV_V3_TestSensors_B.y_j[0] > 0.436332315F) {
      AUAV_V3_TestSensors_B.bankLimit_e = 0.436332315F;
    } else if (AUAV_V3_TestSensors_B.y_j[0] < -0.436332315F) {
      AUAV_V3_TestSensors_B.bankLimit_e = -0.436332315F;
    } else {
      AUAV_V3_TestSensors_B.bankLimit_e = AUAV_V3_TestSensors_B.y_j[0];
    }

    /* End of Saturate: '<S117>/bank Limit' */

    /* S-Function (MCHP_C_function_Call): '<S121>/myTan() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_o = myTan(
      AUAV_V3_TestSensors_B.bankLimit_e
      );

    /* InitialCondition: '<S118>/IC' */
    if (AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_b) {
      AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_b = false;
      rtb_Sum1_mj = 1.0F;
    } else {
      /* Abs: '<S125>/Abs1' incorporates:
       *  Constant: '<S109>/Constant'
       *  Sum: '<S126>/Diff'
       *  UnitDelay: '<S126>/UD'
       */
      rtb_Sum1_mj = (real32_T)fabs(0.3F - AUAV_V3_TestSensors_DWork.UD_DSTATE_f);

      /* Saturate: '<S125>/Saturation1' */
      if (rtb_Sum1_mj > 1.0F) {
        rtb_Sum1_mj = 1.0F;
      }

      /* End of Saturate: '<S125>/Saturation1' */
    }

    /* End of InitialCondition: '<S118>/IC' */

    /* Outputs for Enabled SubSystem: '<S118>/Compute Coef' incorporates:
     *  EnablePort: '<S124>/Enable'
     */
    if (rtb_Sum1_mj > 0.0F) {
      /* Math: '<S124>/Math Function' incorporates:
       *  Constant: '<S109>/Constant'
       *  Gain: '<S124>/-T'
       *
       * About '<S124>/Math Function':
       *  Operator: exp
       */
      AUAV_V3_TestSensors_B.c = 0.997004509F;

      /* Sum: '<S124>/1-c' incorporates:
       *  Constant: '<S124>/Constant'
       */
      AUAV_V3_TestSensors_B.c_b = (real32_T)(1.0 - AUAV_V3_TestSensors_B.c);
    }

    /* End of Outputs for SubSystem: '<S118>/Compute Coef' */

    /* Sum: '<S118>/Subtract' incorporates:
     *  Delay: '<S118>/Integer Delay'
     *  Delay: '<S118>/Integer Delay1'
     *  Product: '<S118>/Divide'
     *  Product: '<S118>/Divide1'
     */
    AUAV_V3_TestSensors_B.Subtract = AUAV_V3_TestSensors_B.c_b *
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_ed + AUAV_V3_TestSensors_B.c
      * AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_n;

    /* Product: '<S120>/Divide' incorporates:
     *  Constant: '<S117>/Constant1'
     *  Constant: '<S120>/Constant1'
     *  Product: '<S117>/Divide'
     *  Product: '<S117>/Divide1'
     *  Sum: '<S109>/Subtract'
     */
    AUAV_V3_TestSensors_B.Divide_h = (1.0F /
      AUAV_V3_TestSensors_B.sf_negprotect.zpVal * 9.80665F *
      AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_o -
      AUAV_V3_TestSensors_B.y_j[5]) * rtb_Product3_bo / 9.80665F;

    /* S-Function (MCHP_C_function_Call): '<S127>/myTan() apUtils.c [updated 5.1.16]' */
    AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_i = myTan(
      AUAV_V3_TestSensors_B.Divide_h
      );

    /* Switch: '<S119>/Switch3' incorporates:
     *  RelationalOperator: '<S119>/Relational Operator2'
     */
    if ((AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_i ==
         AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_i) > 0) {
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ph =
        AUAV_V3_TestSensors_B.myTanapUtilscupdated5116_i;
    }

    /* End of Switch: '<S119>/Switch3' */

    /* Update for UnitDelay: '<S126>/UD' incorporates:
     *  Constant: '<S109>/Constant'
     */
    AUAV_V3_TestSensors_DWork.UD_DSTATE_f = 0.3F;

    /* Update for Delay: '<S118>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_ed =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ph;

    /* Update for Delay: '<S118>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_n =
      AUAV_V3_TestSensors_B.Subtract;
  } else {
    if (AUAV_V3_TestSensors_DWork.SideslipCompensation_MODE) {
      /* Disable for Outport: '<S109>/bankComp' */
      AUAV_V3_TestSensors_B.Subtract = 0.0F;
      AUAV_V3_TestSensors_DWork.SideslipCompensation_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S102>/Sideslip Compensation' */

  /* Product: '<S111>/Divide' incorporates:
   *  Constant: '<S111>/Constant1'
   */
  AUAV_V3_TestSensors_B.Divide = rtb_RhhcosphisinlambYe * rtb_Product3_bo /
    9.80665F;

  /* S-Function (MCHP_C_function_Call): '<S133>/myAtan() apUtils.c [updated 5.1.16]' */
  AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116_a = myAtan(
    AUAV_V3_TestSensors_B.Divide
    );

  /* Sum: '<S102>/Add2' */
  rtb_Deg2R1 = AUAV_V3_TestSensors_B.Subtract +
    AUAV_V3_TestSensors_B.myAtanapUtilscupdated5116_a;

  /* Saturate: '<S102>/Bank  Limit Command' */
  if (rtb_Deg2R1 > 0.436332315F) {
    rtb_Deg2R1 = 0.436332315F;
  } else {
    if (rtb_Deg2R1 < -0.436332315F) {
      rtb_Deg2R1 = -0.436332315F;
    }
  }

  /* End of Saturate: '<S102>/Bank  Limit Command' */

  /* Switch: '<S107>/Switch3' incorporates:
   *  RelationalOperator: '<S107>/Relational Operator2'
   */
  if ((rtb_Product3_j0[1] == rtb_Product3_j0[1]) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_oy = rtb_Product3_j0[1];
  }

  /* End of Switch: '<S107>/Switch3' */

  /* Saturate: '<S102>/[-20 20]' */
  if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_oy > 20.0F) {
    rtb_Sum1_mj = 20.0F;
  } else if (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_oy < -20.0F) {
    rtb_Sum1_mj = -20.0F;
  } else {
    rtb_Sum1_mj = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_oy;
  }

  /* MATLAB Function: '<S104>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S104>/Constant'
   *  Constant: '<S104>/Constant1'
   *  Saturate: '<S102>/[-20 20]'
   */
  AUA_EmbeddedMATLABFunction(rtb_Sum1_mj, 0.01, 10.0,
    &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_b0);

  /* DataStoreWrite: '<S70>/mlNavigation' incorporates:
   *  Gain: '<S102>/Neg Feedback '
   */
  mlNavigation.psiDot_c = rtb_RhhcosphisinlambYe;
  mlNavigation.phi_c = rtb_Deg2R1;
  mlNavigation.ay_body = -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y;

  /* Sum: '<S110>/Add3' incorporates:
   *  Constant: '<S110>/SaturationLimit'
   */
  rtb_RhhcosphicoslambXe = 0.17453292F -
    AUAV_V3_TestSensors_B.DataTypeConversion_g;

  /* Sum: '<S110>/Add1' incorporates:
   *  Constant: '<S110>/delayTime'
   *  DataStoreRead: '<S10>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Delay: '<S110>/NDelays'
   *  Gain: '<S102>/Neg Feedback '
   *  Product: '<S110>/Product1'
   *  Product: '<S110>/Product2'
   *  Product: '<S110>/Product3'
   *  Sum: '<S110>/Sum'
   */
  rtb_Sum1_mj = (-AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y -
                 AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[0]) / 0.05F *
    mlParamInterface.param[14] +
    -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y *
    mlParamInterface.param[12];

  /* Sum: '<S110>/Sum2' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Gain: '<S102>/Neg Feedback '
   *  Gain: '<S110>/Gain'
   *  Memory: '<S110>/Memory1'
   *  Product: '<S110>/Product4'
   */
  rtb_RhhcosphisinlambYe = (0.01F *
    -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y *
    mlParamInterface.param[13] +
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g) + rtb_Sum1_mj;

  /* Switch: '<S110>/AntiWindup' incorporates:
   *  Constant: '<S110>/Constant5'
   *  Constant: '<S110>/SaturationLimit'
   *  Constant: '<S110>/SaturationLimit1'
   *  Gain: '<S102>/Neg Feedback '
   *  Logic: '<S110>/Logical Operator'
   *  RelationalOperator: '<S110>/Relational Operator'
   *  RelationalOperator: '<S110>/Relational Operator1'
   *  Sum: '<S110>/Add3'
   *  Sum: '<S110>/Add4'
   */
  if ((rtb_RhhcosphisinlambYe > -0.17453292F -
       AUAV_V3_TestSensors_B.DataTypeConversion_g) && (rtb_RhhcosphisinlambYe <
       0.17453292F - AUAV_V3_TestSensors_B.DataTypeConversion_g)) {
    rtb_RhhcosphisinlambYe =
      -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y;
  } else {
    rtb_RhhcosphisinlambYe = 0.0F;
  }

  /* End of Switch: '<S110>/AntiWindup' */

  /* Switch: '<S131>/Switch3' incorporates:
   *  RelationalOperator: '<S131>/Relational Operator2'
   */
  if ((rtb_RhhcosphisinlambYe == rtb_RhhcosphisinlambYe) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_n = rtb_RhhcosphisinlambYe;
  }

  /* End of Switch: '<S131>/Switch3' */

  /* Switch: '<S130>/Switch1' incorporates:
   *  Constant: '<S130>/Constant'
   *  Constant: '<S130>/Constant1'
   *  Constant: '<S130>/Constant2'
   *  Constant: '<S130>/Constant3'
   *  Constant: '<S130>/Constant5'
   *  Delay: '<S130>/Integer Delay'
   *  Delay: '<S130>/Integer Delay1'
   *  Delay: '<S130>/Integer Delay2'
   *  Product: '<S130>/Product'
   *  Product: '<S130>/Product1'
   *  Product: '<S130>/Product2'
   *  Product: '<S130>/Product3'
   *  Sum: '<S130>/Subtract'
   *  Sum: '<S130>/Subtract1'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_cosphi = 0.0F;
  } else {
    rtb_cosphi = ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_n *
                   0.333333343F +
                   AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_o * 1.33333337F)
                  + AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l[0] *
                  0.333333343F) * 0.005F +
      AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_bj;
  }

  /* End of Switch: '<S130>/Switch1' */

  /* Switch: '<S132>/Switch3' incorporates:
   *  RelationalOperator: '<S132>/Relational Operator2'
   */
  if ((rtb_cosphi == rtb_cosphi) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_f = rtb_cosphi;
  }

  /* End of Switch: '<S132>/Switch3' */

  /* Switch: '<S110>/On//Off' incorporates:
   *  Constant: '<S110>/Constant1'
   *  DataStoreRead: '<S10>/PAR_PID_YAW_DAMP_P PAR_PID_YAW_DAMP_I PAR_PID_YAW_DAMP_D'
   *  Product: '<S110>/Product'
   *  Sum: '<S110>/Add2'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g = 0.0F;
  } else {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_f * mlParamInterface.param
      [13] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S110>/On//Off' */

  /* Switch: '<S129>/Switch2' incorporates:
   *  Constant: '<S110>/SaturationLimit'
   *  RelationalOperator: '<S129>/LowerRelop1'
   *  Sum: '<S110>/Add3'
   */
  if (!(AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g > 0.17453292F -
        AUAV_V3_TestSensors_B.DataTypeConversion_g)) {
    /* Switch: '<S129>/Switch' incorporates:
     *  Constant: '<S110>/SaturationLimit1'
     *  RelationalOperator: '<S129>/UpperRelop'
     *  Sum: '<S110>/Add4'
     */
    if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g < -0.17453292F -
        AUAV_V3_TestSensors_B.DataTypeConversion_g) {
      rtb_RhhcosphicoslambXe = -0.17453292F -
        AUAV_V3_TestSensors_B.DataTypeConversion_g;
    } else {
      rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_g;
    }

    /* End of Switch: '<S129>/Switch' */
  }

  /* End of Switch: '<S129>/Switch2' */

  /* Sum: '<S102>/Add' */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphicoslambXe +
    AUAV_V3_TestSensors_B.DataTypeConversion_g;

  /* Sum: '<S102>/Add1' */
  rtb_Deg2R1 -= AUAV_V3_TestSensors_B.y_j[0];

  /* Sum: '<S108>/Add3' incorporates:
   *  Constant: '<S108>/SaturationLimit'
   */
  rtb_cosphi = 0.383972436F - AUAV_V3_TestSensors_B.DataTypeConversion_h;

  /* Switch: '<S106>/Switch3' incorporates:
   *  RelationalOperator: '<S106>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0 = rtb_Deg2R1;
  }

  /* End of Switch: '<S106>/Switch3' */

  /* Sum: '<S108>/Add1' incorporates:
   *  Constant: '<S108>/delayTime'
   *  DataStoreRead: '<S10>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Delay: '<S108>/NDelays'
   *  Product: '<S108>/Product1'
   *  Product: '<S108>/Product2'
   *  Product: '<S108>/Product3'
   *  Sum: '<S108>/Sum'
   */
  rtb_Sum1_mj = (AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0 -
                 AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[0]) / 0.05F *
    mlParamInterface.param[8] +
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0 * mlParamInterface.param[6];

  /* Sum: '<S108>/Sum2' incorporates:
   *  DataStoreRead: '<S10>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Gain: '<S108>/Gain'
   *  Memory: '<S108>/Memory1'
   *  Product: '<S108>/Product4'
   */
  rtb_RhhcosphicoslambXe = (0.01F *
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0 * mlParamInterface.param[7]
    + AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j) + rtb_Sum1_mj;

  /* Switch: '<S108>/AntiWindup' incorporates:
   *  Constant: '<S108>/Constant5'
   *  Constant: '<S108>/SaturationLimit'
   *  Constant: '<S108>/SaturationLimit1'
   *  Logic: '<S108>/Logical Operator'
   *  RelationalOperator: '<S108>/Relational Operator'
   *  RelationalOperator: '<S108>/Relational Operator1'
   *  Sum: '<S108>/Add3'
   *  Sum: '<S108>/Add4'
   */
  if ((rtb_RhhcosphicoslambXe > -0.383972436F -
       AUAV_V3_TestSensors_B.DataTypeConversion_h) && (rtb_RhhcosphicoslambXe <
       0.383972436F - AUAV_V3_TestSensors_B.DataTypeConversion_h)) {
    rtb_RhhcosphicoslambXe = AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0;
  } else {
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* End of Switch: '<S108>/AntiWindup' */

  /* Switch: '<S115>/Switch3' incorporates:
   *  RelationalOperator: '<S115>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe == rtb_RhhcosphicoslambXe) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_l = rtb_RhhcosphicoslambXe;
  }

  /* End of Switch: '<S115>/Switch3' */

  /* Switch: '<S114>/Switch1' incorporates:
   *  Constant: '<S114>/Constant'
   *  Constant: '<S114>/Constant1'
   *  Constant: '<S114>/Constant2'
   *  Constant: '<S114>/Constant3'
   *  Constant: '<S114>/Constant5'
   *  Delay: '<S114>/Integer Delay'
   *  Delay: '<S114>/Integer Delay1'
   *  Delay: '<S114>/Integer Delay2'
   *  Product: '<S114>/Product'
   *  Product: '<S114>/Product1'
   *  Product: '<S114>/Product2'
   *  Product: '<S114>/Product3'
   *  Sum: '<S114>/Subtract'
   *  Sum: '<S114>/Subtract1'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = ((AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_l *
      0.333333343F + AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_b *
      1.33333337F) + AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l3[0] *
      0.333333343F) * 0.005F + AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_j;
  }

  /* End of Switch: '<S114>/Switch1' */

  /* Switch: '<S116>/Switch3' incorporates:
   *  RelationalOperator: '<S116>/Relational Operator2'
   */
  if ((rtb_RhhcosphicoslambXe == rtb_RhhcosphicoslambXe) > 0) {
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_d = rtb_RhhcosphicoslambXe;
  }

  /* End of Switch: '<S116>/Switch3' */

  /* Switch: '<S108>/On//Off' incorporates:
   *  Constant: '<S108>/Constant1'
   *  DataStoreRead: '<S10>/PAR_PID_ROLL_P PAR_PID_ROLL_I PAR_PID_ROLL_D'
   *  Product: '<S108>/Product'
   *  Sum: '<S108>/Add2'
   */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 > 0) {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j = 0.0F;
  } else {
    AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j =
      AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_d * mlParamInterface.param
      [7] + rtb_Sum1_mj;
  }

  /* End of Switch: '<S108>/On//Off' */

  /* Switch: '<S113>/Switch2' incorporates:
   *  Constant: '<S108>/SaturationLimit'
   *  RelationalOperator: '<S113>/LowerRelop1'
   *  Sum: '<S108>/Add3'
   */
  if (!(AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j > 0.383972436F -
        AUAV_V3_TestSensors_B.DataTypeConversion_h)) {
    /* Switch: '<S113>/Switch' incorporates:
     *  Constant: '<S108>/SaturationLimit1'
     *  RelationalOperator: '<S113>/UpperRelop'
     *  Sum: '<S108>/Add4'
     */
    if (AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j < -0.383972436F -
        AUAV_V3_TestSensors_B.DataTypeConversion_h) {
      rtb_cosphi = -0.383972436F - AUAV_V3_TestSensors_B.DataTypeConversion_h;
    } else {
      rtb_cosphi = AUAV_V3_TestSensors_DWork.Memory1_PreviousInput_j;
    }

    /* End of Switch: '<S113>/Switch' */
  }

  /* End of Switch: '<S113>/Switch2' */

  /* Sum: '<S102>/Add3' */
  rtb_cosphi += AUAV_V3_TestSensors_B.DataTypeConversion_h;

  /* Saturate: '<S102>/Aileron Limit' */
  if (rtb_cosphi > 0.383972436F) {
    rtb_Sum1_mj = 0.383972436F;
  } else if (rtb_cosphi < -0.383972436F) {
    rtb_Sum1_mj = -0.383972436F;
  } else {
    rtb_Sum1_mj = rtb_cosphi;
  }

  /* End of Saturate: '<S102>/Aileron Limit' */

  /* Switch: '<S102>/Switch2' incorporates:
   *  Constant: '<S102>/Constant'
   *  DataStoreRead: '<Root>/PAR_NAV_ISR_FAC PAR_PID_RMIX_ON PAR_PID_RMIX_P'
   *  Product: '<S102>/Product'
   */
  if (mlParamInterface.param[27L] > 0.3F) {
    rtb_RhhcosphicoslambXe = mlParamInterface.param[28] * rtb_Sum1_mj;
  } else {
    rtb_RhhcosphicoslambXe = 0.0F;
  }

  /* End of Switch: '<S102>/Switch2' */

  /* Sum: '<S102>/Add4' */
  rtb_cosphi = rtb_RhhcosphicoslambXe + rtb_RhhcosphisinlambYe;

  /* Update for Delay: '<S110>/NDelays' incorporates:
   *  Gain: '<S102>/Neg Feedback '
   */
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[0] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[1];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[1] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[2];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[2] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[3];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[3] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[4];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_l[4] =
    -AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b0.y;

  /* Update for Delay: '<S130>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_bj =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_f;

  /* Update for Delay: '<S130>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_o =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S130>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l[0] =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l[1] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_n;

  /* Update for Delay: '<S108>/NDelays' */
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[0] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[1];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[1] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[2];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[2] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[3];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[3] =
    AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[4];
  AUAV_V3_TestSensors_DWork.NDelays_DSTATE_p[4] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k0;

  /* Update for Delay: '<S114>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_j =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_d;

  /* Update for Delay: '<S114>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_b =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_l;

  /* Update for Delay: '<S114>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l3[0] =
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l3[1];
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_l3[1] =
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_l;

  /* End of Outputs for SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S140>/Throttle  Limit' */
  /* MATLAB Function 'myMux Fun1': '<S22>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S22>:1:5' */
  if (rtb_y_jl_idx_0 > 0.95F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_y_jl_idx_0 = 0.95F;
  } else {
    if (rtb_y_jl_idx_0 < 0.0F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_y_jl_idx_0 = 0.0F;
    }
  }

  /* End of Saturate: '<S140>/Throttle  Limit' */
  /* End of Outputs for SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S102>/Rudder Limit' */
  if (rtb_cosphi > 0.17453292F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_cosphi = 0.17453292F;
  } else {
    if (rtb_cosphi < -0.17453292F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_cosphi = -0.17453292F;
    }
  }

  /* End of Saturate: '<S102>/Rudder Limit' */
  /* End of Outputs for SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */

  /* Outputs for Atomic SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* Saturate: '<S140>/Elevator  Limit' */
  if (rtb_IC4_idx_0 > 0.401425719F) {
    /* MATLAB Function: '<Root>/myMux Fun1' */
    rtb_IC4_idx_0 = 0.401425719F;
  } else {
    if (rtb_IC4_idx_0 < -0.401425719F) {
      /* MATLAB Function: '<Root>/myMux Fun1' */
      rtb_IC4_idx_0 = -0.401425719F;
    }
  }

  /* End of Saturate: '<S140>/Elevator  Limit' */
  /* End of Outputs for SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* If: '<S682>/If' */
  if (AUAV_V3_TestSensors_B.ManualorAutonavSupportcupdated4 == 1) {
    /* Outputs for IfAction SubSystem: '<S682>/If  Control Type Is Manual' incorporates:
     *  ActionPort: '<S685>/Action Port'
     */
    /* Gain: '<S685>/Gain4' incorporates:
     *  MATLAB Function: '<S656>/myMux Fun5'
     */
    AUAV_V3_TestSensors_B.Merge_oc[0] = AUAV_V3_TestSensors_B.InputCapture_o3 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[1] = AUAV_V3_TestSensors_B.InputCapture_o1 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[2] = AUAV_V3_TestSensors_B.InputCapture_o4 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[3] = AUAV_V3_TestSensors_B.InputCapture_o2 >>
      1;

    /* End of Outputs for SubSystem: '<S682>/If  Control Type Is Manual' */
  } else if ((rtb_DataTypeConversion1_hq == 3) || (rtb_DataTypeConversion1_hq ==
              4) || (rtb_DataTypeConversion1_hq == 9) ||
             (rtb_DataTypeConversion1_hq == 10)) {
    /* Outputs for IfAction SubSystem: '<S682>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' incorporates:
     *  ActionPort: '<S688>/Action Port'
     */
    /* DataTypeConversion: '<S695>/Data Type Conversion' incorporates:
     *  Constant: '<S695>/Constant1'
     *  Constant: '<S695>/Constant2'
     *  Product: '<S695>/Divide'
     *  Sum: '<S695>/Add'
     */
    tmp_4 = floor(rtb_y_jl_idx_0 * 3714.5 + 4693.0);

    /* DataTypeConversion: '<S696>/Data Type Conversion' incorporates:
     *  Constant: '<S696>/Constant1'
     *  Constant: '<S696>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S696>/Divide'
     *  Sum: '<S696>/Add'
     */
    tmp_5 = floor(rtb_Sum1_mj * -6994.8597488887972 + 6576.5000000000009);

    /* DataTypeConversion: '<S697>/Data Type Conversion' incorporates:
     *  Constant: '<S697>/Constant1'
     *  Constant: '<S697>/Constant2'
     *  Product: '<S697>/Divide'
     *  Sum: '<S697>/Add'
     */
    tmp_6 = floor(rtb_cosphi * -5246.1448116666006 + 6576.5);

    /* DataTypeConversion: '<S694>/Data Type Conversion' incorporates:
     *  Constant: '<S694>/Constant1'
     *  Constant: '<S694>/Constant2'
     *  Product: '<S694>/Divide'
     *  Sum: '<S694>/Add'
     */
    tmp_7 = floor(rtb_IC4_idx_0 * 10492.289623333201 + 6576.5);

    /* DataTypeConversion: '<S695>/Data Type Conversion' */
    if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
      tmp_4 = 0.0;
    } else {
      tmp_4 = fmod(tmp_4, 65536.0);
    }

    /* DataTypeConversion: '<S696>/Data Type Conversion' */
    if (rtIsNaN(tmp_5) || rtIsInf(tmp_5)) {
      tmp_5 = 0.0;
    } else {
      tmp_5 = fmod(tmp_5, 65536.0);
    }

    /* DataTypeConversion: '<S697>/Data Type Conversion' */
    if (rtIsNaN(tmp_6) || rtIsInf(tmp_6)) {
      tmp_6 = 0.0;
    } else {
      tmp_6 = fmod(tmp_6, 65536.0);
    }

    /* DataTypeConversion: '<S694>/Data Type Conversion' */
    if (rtIsNaN(tmp_7) || rtIsInf(tmp_7)) {
      tmp_7 = 0.0;
    } else {
      tmp_7 = fmod(tmp_7, 65536.0);
    }

    /* MATLAB Function: '<S688>/myMux Fun1' incorporates:
     *  DataTypeConversion: '<S694>/Data Type Conversion'
     *  DataTypeConversion: '<S695>/Data Type Conversion'
     *  DataTypeConversion: '<S696>/Data Type Conversion'
     *  DataTypeConversion: '<S697>/Data Type Conversion'
     */
    AUAV_V3_TestSe_myMuxFun1_d((uint16_T)tmp_4, (uint16_T)tmp_5, (uint16_T)tmp_6,
      (uint16_T)tmp_7, AUAV_V3_TestSensors_B.Merge_oc);

    /* End of Outputs for SubSystem: '<S682>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' */
  } else if (rtb_DataTypeConversion1_hq == 2) {
    /* Outputs for IfAction SubSystem: '<S682>/If  Control Type Is Passthrough' incorporates:
     *  ActionPort: '<S686>/Action Port'
     */
    /* Gain: '<S686>/Gain4' incorporates:
     *  MATLAB Function: '<S656>/myMux Fun5'
     */
    AUAV_V3_TestSensors_B.Merge_oc[0] = AUAV_V3_TestSensors_B.InputCapture_o3 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[1] = AUAV_V3_TestSensors_B.InputCapture_o1 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[2] = AUAV_V3_TestSensors_B.InputCapture_o4 >>
      1;
    AUAV_V3_TestSensors_B.Merge_oc[3] = AUAV_V3_TestSensors_B.InputCapture_o2 >>
      1;

    /* End of Outputs for SubSystem: '<S682>/If  Control Type Is Passthrough' */
  } else {
    if (rtb_DataTypeConversion1_hq == 8) {
      /* Outputs for IfAction SubSystem: '<S682>/If  Control Type Is Selective Passthrough' incorporates:
       *  ActionPort: '<S687>/Action Port'
       */

      /* S-Function (MCHP_C_function_Call): '<S687>/Initialize Control MCU' */
      getPassValues(
                    &AUAV_V3_TestSensors_B.InitializeControlMCU[0]
                    );

      /* Switch: '<S687>/Switch' incorporates:
       *  Gain: '<S687>/Gain'
       *  MATLAB Function: '<S656>/myMux Fun5'
       */
      if (AUAV_V3_TestSensors_B.InitializeControlMCU[3] >= 1) {
        rtb_u2deg = AUAV_V3_TestSensors_B.InputCapture_o3 >> 1;
      } else {
        /* DataTypeConversion: '<S690>/Data Type Conversion' incorporates:
         *  Constant: '<S690>/Constant1'
         *  Constant: '<S690>/Constant2'
         *  Product: '<S690>/Divide'
         *  Sum: '<S690>/Add'
         */
        tmp_4 = floor(rtb_y_jl_idx_0 * 3714.5 + 4693.0);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_u2deg = 0U;
        } else {
          rtb_u2deg = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S690>/Data Type Conversion' */
      }

      /* End of Switch: '<S687>/Switch' */

      /* Switch: '<S687>/Switch1' incorporates:
       *  Gain: '<S687>/Gain1'
       *  MATLAB Function: '<S656>/myMux Fun5'
       */
      if (AUAV_V3_TestSensors_B.InitializeControlMCU[2] >= 1) {
        rtb_Switch1_m = AUAV_V3_TestSensors_B.InputCapture_o1 >> 1;
      } else {
        /* DataTypeConversion: '<S691>/Data Type Conversion' incorporates:
         *  Constant: '<S691>/Constant1'
         *  Constant: '<S691>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S691>/Divide'
         *  Sum: '<S691>/Add'
         */
        tmp_4 = floor(rtb_Sum1_mj * -6994.8597488887972 + 6576.5000000000009);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_Switch1_m = 0U;
        } else {
          rtb_Switch1_m = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S691>/Data Type Conversion' */
      }

      /* End of Switch: '<S687>/Switch1' */

      /* Switch: '<S687>/Switch2' incorporates:
       *  Gain: '<S687>/Gain2'
       *  MATLAB Function: '<S656>/myMux Fun5'
       */
      if (AUAV_V3_TestSensors_B.InitializeControlMCU[1] >= 1) {
        rtb_Switch2 = AUAV_V3_TestSensors_B.InputCapture_o4 >> 1;
      } else {
        /* DataTypeConversion: '<S692>/Data Type Conversion' incorporates:
         *  Constant: '<S692>/Constant1'
         *  Constant: '<S692>/Constant2'
         *  Product: '<S692>/Divide'
         *  Sum: '<S692>/Add'
         */
        tmp_4 = floor(rtb_cosphi * -5246.1448116666006 + 6576.5);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_Switch2 = 0U;
        } else {
          rtb_Switch2 = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S692>/Data Type Conversion' */
      }

      /* End of Switch: '<S687>/Switch2' */

      /* Switch: '<S687>/Switch3' incorporates:
       *  Gain: '<S687>/Gain3'
       *  MATLAB Function: '<S656>/myMux Fun5'
       */
      if (AUAV_V3_TestSensors_B.InitializeControlMCU[0] >= 1) {
        rtb_Switch3_fq = AUAV_V3_TestSensors_B.InputCapture_o2 >> 1;
      } else {
        /* DataTypeConversion: '<S689>/Data Type Conversion' incorporates:
         *  Constant: '<S689>/Constant1'
         *  Constant: '<S689>/Constant2'
         *  Product: '<S689>/Divide'
         *  Sum: '<S689>/Add'
         */
        tmp_4 = floor(rtb_IC4_idx_0 * 10492.289623333201 + 6576.5);
        if (rtIsNaN(tmp_4) || rtIsInf(tmp_4)) {
          rtb_Switch3_fq = 0U;
        } else {
          rtb_Switch3_fq = (uint16_T)fmod(tmp_4, 65536.0);
        }

        /* End of DataTypeConversion: '<S689>/Data Type Conversion' */
      }

      /* End of Switch: '<S687>/Switch3' */

      /* MATLAB Function: '<S687>/myMux Fun1' */
      AUAV_V3_TestSe_myMuxFun1_d(rtb_u2deg, rtb_Switch1_m, rtb_Switch2,
        rtb_Switch3_fq, AUAV_V3_TestSensors_B.Merge_oc);

      /* End of Outputs for SubSystem: '<S682>/If  Control Type Is Selective Passthrough' */
    }
  }

  /* End of If: '<S682>/If' */

  /* DataStoreWrite: '<S20>/mlPwmCommands' incorporates:
   *  DataStoreRead: '<S20>/Get time3'
   *  DataTypeConversion: '<S681>/Data Type Conversion'
   *  Gain: '<S681>/Convert to  Microseconds'
   */
  mlPwmCommands.servo1_raw = (uint16_T)(52429UL *
    AUAV_V3_TestSensors_B.Merge_oc[0] >> 18);
  mlPwmCommands.servo2_raw = (uint16_T)(52429UL *
    AUAV_V3_TestSensors_B.Merge_oc[1] >> 18);
  mlPwmCommands.servo3_raw = (uint16_T)(52429UL *
    AUAV_V3_TestSensors_B.Merge_oc[2] >> 18);
  mlPwmCommands.servo4_raw = (uint16_T)(52429UL *
    AUAV_V3_TestSensors_B.Merge_oc[3] >> 18);
  mlPwmCommands.time_usec = AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* DataStoreWrite: '<S20>/mlAttitudePQR' */
  mlAttitudeSol.rollspeed = rtb_Product2_c[0];
  mlAttitudeSol.pitchspeed = rtb_Product2_c[1];
  mlAttitudeSol.yawspeed = rtb_Product2_c[2];

  /* MATLAB Function: '<S665>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S665>/Constant'
   *  Constant: '<S665>/Constant1'
   *  DataTypeConversion: '<S656>/Data Type Conversion2'
   */
  AUA_EmbeddedMATLABFunction((real32_T)AUAV_V3_TestSensors_B.InputCapture_o6,
    0.01, 0.1, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction);

  /* DataTypeConversion: '<S656>/Data Type Conversion1' */
  rtb_RhhcosphicoslambXe = (real32_T)floor
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction.y);
  if (rtIsNaNF(rtb_RhhcosphicoslambXe) || rtIsInfF(rtb_RhhcosphicoslambXe)) {
    rtb_RhhcosphicoslambXe = 0.0F;
  } else {
    rtb_RhhcosphicoslambXe = (real32_T)fmod(rtb_RhhcosphicoslambXe, 65536.0F);
  }

  /* MATLAB Function: '<S666>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S666>/Constant'
   *  Constant: '<S666>/Constant1'
   *  DataTypeConversion: '<S656>/Data Type Conversion3'
   */
  AUA_EmbeddedMATLABFunction((real32_T)AUAV_V3_TestSensors_B.InputCapture_o7,
    0.01, 0.1, &AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b,
    &AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_b);

  /* DataTypeConversion: '<S656>/Data Type Conversion4' */
  /* MATLAB Function 'Trim Vals [updated 4.28.16]/Control Surface Input/myMux Fun1': '<S667>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S667>:1:5' */
  rtb_Sum1_mj = (real32_T)floor
    (AUAV_V3_TestSensors_B.sf_EmbeddedMATLABFunction_b.y);
  if (rtIsNaNF(rtb_Sum1_mj) || rtIsInfF(rtb_Sum1_mj)) {
    rtb_Sum1_mj = 0.0F;
  } else {
    rtb_Sum1_mj = (real32_T)fmod(rtb_Sum1_mj, 65536.0F);
  }

  /* DataStoreWrite: '<S20>/mlAttitude' incorporates:
   *  DataTypeConversion: '<S656>/Data Type Conversion1'
   *  DataTypeConversion: '<S656>/Data Type Conversion4'
   */
  mlVISensor.voltage = rtb_RhhcosphicoslambXe < 0.0F ? (uint16_T)-(int16_T)
    (uint16_T)-rtb_RhhcosphicoslambXe : (uint16_T)rtb_RhhcosphicoslambXe;
  mlVISensor.reading2 = rtb_Sum1_mj < 0.0F ? (uint16_T)-(int16_T)(uint16_T)
    -rtb_Sum1_mj : (uint16_T)rtb_Sum1_mj;

  /* RateTransition: '<S20>/Rate Transition' */
  if (AUAV_V3_TestSensors_M->Timing.RateInteraction.TID0_1 == 1) {
    AUAV_V3_TestSensors_B.RateTransition[0] = AUAV_V3_TestSensors_B.Merge_oc[1];
    AUAV_V3_TestSensors_B.RateTransition[1] = AUAV_V3_TestSensors_B.Merge_oc[3];
    AUAV_V3_TestSensors_B.RateTransition[2] = AUAV_V3_TestSensors_B.Merge_oc[0];
    AUAV_V3_TestSensors_B.RateTransition[3] = AUAV_V3_TestSensors_B.Merge_oc[2];
  }

  /* End of RateTransition: '<S20>/Rate Transition' */

  /* Outputs for Enabled SubSystem: '<S20>/Send_Cmd_6DOF_HIL' incorporates:
   *  EnablePort: '<S683>/Enable'
   */
  /* DataStoreRead: '<S20>/Data Store Read' incorporates:
   *  Constant: '<S683>/COMP_ID'
   *  Constant: '<S683>/SYS_ID'
   */
  if (AUAV_V3_TestSensors_DWork.SIX_DOF_HIL_FLAG > 0.0) {
    /* DataStoreRead: '<S683>/Get Raw Commands' */
    AUAV_V3_TestSensors_B.GetRawCommands = mlPwmCommands;

    /* S-Function (MCHP_C_function_Call): '<S683>/PackRawServo' */
    AUAV_V3_TestSensors_B.PackRawServo_l = HIL_PackRawServo(
      ((uint8_T)101U)
      , ((uint8_T)1U)
      , AUAV_V3_TestSensors_B.GetRawCommands
      , ((uint32_T)0UL)
      );

    /* S-Function (MCHP_C_function_Call): '<S683>/TX_N_Data9' */
    TxN_Data_OverU4(
                    AUAV_V3_TestSensors_B.PackRawServo_l
                    );
  }

  /* End of DataStoreRead: '<S20>/Data Store Read' */
  /* End of Outputs for SubSystem: '<S20>/Send_Cmd_6DOF_HIL' */

  /* Outputs for Enabled SubSystem: '<S20>/Send_Cmd_X_Plane_HIL' incorporates:
   *  EnablePort: '<S684>/Enable'
   */
  /* DataStoreRead: '<S20>/Data Store Read1' */
  if (AUAV_V3_TestSensors_DWork.X_PLANE_HIL_FLAG > 0.0) {
    /* S-Function (MCHP_C_function_Call): '<S684>/PackRawServo' */
    AUAV_V3_TestSensors_B.PackRawServo = send_HILSIM_outputs(
      );

    /* S-Function (MCHP_C_function_Call): '<S684>/TX_N_Data9' */
    TxN_Data_OverU4(
                    AUAV_V3_TestSensors_B.PackRawServo
                    );
  }

  /* End of DataStoreRead: '<S20>/Data Store Read1' */
  /* End of Outputs for SubSystem: '<S20>/Send_Cmd_X_Plane_HIL' */

  /* DataStoreWrite: '<S656>/Update Control Surface DATA' */
  mlPilotConsoleData.chan1_raw = AUAV_V3_TestSensors_B.InputCapture_o1;
  mlPilotConsoleData.chan2_raw = AUAV_V3_TestSensors_B.InputCapture_o2;
  mlPilotConsoleData.chan3_raw = AUAV_V3_TestSensors_B.InputCapture_o3;
  mlPilotConsoleData.chan4_raw = AUAV_V3_TestSensors_B.InputCapture_o4;
  mlPilotConsoleData.chan5_raw = AUAV_V3_TestSensors_B.InputCapture_o5;
  mlPilotConsoleData.chan6_raw = AUAV_V3_TestSensors_B.InputCapture_o6;
  mlPilotConsoleData.chan7_raw = AUAV_V3_TestSensors_B.InputCapture_o7;
  mlPilotConsoleData.chan8_raw = AUAV_V3_TestSensors_B.InputCapture_o8;

  /* DataStoreWrite: '<Root>/Data Store Write' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  AUAV_V3_TestSensors_DWork.SIX_DOF_HIL_FLAG = 0.0;

  /* DataStoreWrite: '<Root>/Data Store Write1' incorporates:
   *  Constant: '<Root>/Constant5'
   */
  AUAV_V3_TestSensors_DWork.X_PLANE_HIL_FLAG = 0.0;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S3>/Output'
   */
  tmp_a = 3276800000UL;
  uMultiWordMul(&tmp_a, 1, &AUAV_V3_TestSensors_DWork.Output_DSTATE, 1,
                &tmp_9.chunks[0U], 2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_9.chunks[0U], 2, 15U, &tmp_8.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  AUAV_V3_TestSensors_DWork.time_since_boot_usec = MultiWord2uLong
    (&tmp_8.chunks[0U]);

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  AUAV_V3_TestSensors_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = AUAV_V3_TestSensors_B.U3CH4;

  /* Outputs for Atomic SubSystem: '<Root>/Barometer_Driver' */
  AUAV_V3_T_Barometer_Driver();

  /* End of Outputs for SubSystem: '<Root>/Barometer_Driver' */

  /* Outputs for Atomic SubSystem: '<Root>/IMU_Mag_Driver' */
  AUAV_V3_Tes_IMU_Mag_Driver();

  /* End of Outputs for SubSystem: '<Root>/IMU_Mag_Driver' */

  /* MATLAB Function: '<Root>/myMux Fun4' */
  AUAV_V3_TestSens_myMuxFun3(AUAV_V3_TestSensors_B.BiasRateLimiter,
    AUAV_V3_TestSensors_B.DataTypeConversion1,
    &AUAV_V3_TestSensors_B.sf_myMuxFun4);

  /* MATLAB Function: '<Root>/myMux Fun3' */
  AUAV_V3_TestSens_myMuxFun3(AUAV_V3_TestSensors_B.sf_myMuxFun1_e.y,
    AUAV_V3_TestSensors_B.sf_myMuxFun2_a.y, &AUAV_V3_TestSensors_B.sf_myMuxFun3);

  /* MATLAB Function: '<Root>/myMux Fun5' */
  /* MATLAB Function 'myMux Fun5': '<S26>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S26>:1:5' */
  AUAV_V3_TestSensors_B.y[0] = AUAV_V3_TestSensors_B.DataTypeConversion3[0];
  AUAV_V3_TestSensors_B.y[1] = AUAV_V3_TestSensors_B.DataTypeConversion3[1];
  AUAV_V3_TestSensors_B.y[2] = AUAV_V3_TestSensors_B.DataTypeConversion3[2];
  AUAV_V3_TestSensors_B.y[3] = AUAV_V3_TestSensors_B.DataTypeConversion6[0];
  AUAV_V3_TestSensors_B.y[4] = AUAV_V3_TestSensors_B.DataTypeConversion6[1];
  AUAV_V3_TestSensors_B.y[5] = AUAV_V3_TestSensors_B.DataTypeConversion6[2];
  AUAV_V3_TestSensors_B.y[6] = AUAV_V3_TestSensors_B.DataTypeConversion7[0];
  AUAV_V3_TestSensors_B.y[7] = AUAV_V3_TestSensors_B.DataTypeConversion7[1];
  AUAV_V3_TestSensors_B.y[8] = AUAV_V3_TestSensors_B.DataTypeConversion7[2];

  /* DataStoreRead: '<Root>/Get time4' */
  AUAV_V3_TestSensors_B.Gettime4 =
    AUAV_V3_TestSensors_DWork.time_since_boot_usec;

  /* S-Function (MCHP_C_function_Call): '<S18>/Update Euler and PQR [updateSensorMcuState.c]1' */
  updateEuler(
              &AUAV_V3_TestSensors_B.y_k[0]
              );

  /* S-Function (MCHP_C_function_Call): '<S18>/Update the Time Stamp [updateSensorMcuState.c]1' */
  updateTimeStamp(
                  AUAV_V3_TestSensors_B.Gettime4
                  );

  /* S-Function (MCHP_C_function_Call): '<S18>/Update the Time Stamp [updateSensorMcuState.c]2' */
  updateSensorData(
                   &AUAV_V3_TestSensors_B.y[0]
                   );

  /* S-Function (MCHP_C_function_Call): '<S18>/Update the Time Stamp [updateSensorMcuState.c]7' */
  updatePosition(
                 &AUAV_V3_TestSensors_B.sf_myMuxFun3.y[0]
                 );

  /* S-Function (MCHP_C_function_Call): '<S18>/Update the Time Stamp [updateSensorMcuState.c]6' */
  updateBias(
             &AUAV_V3_TestSensors_B.sf_myMuxFun4.y[0]
             );

  /* S-Function (MCHP_Digital_Input): '<Root>/Config UART4 Rx Pin' */

  /* MCHP_Digital_Input Block: <Root>/Config UART4 Rx Pin/Output */
  rtb_ConfigUART4RxPin = PORTEbits.RE6;/* Read pin E6 */

  /* Switch: '<S59>/FixPt Switch' incorporates:
   *  Constant: '<S58>/FixPt Constant'
   *  Sum: '<S58>/FixPt Sum1'
   *  UnitDelay: '<S3>/Output'
   */
  AUAV_V3_TestSensors_DWork.Output_DSTATE++;

  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID0();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */

  /* Update for Delay: '<S73>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE = rtb_Switch3;

  /* Update for Delay: '<S74>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_k = rtb_Deg2R_h_idx_0;

  /* Update for Enabled SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  Update for EnablePort: '<S69>/Enable'
   */
  if (AUAV_V3_TestSensors_DWork.L1OutputFeedbackControllerWithP) {
    /* Update for UnitDelay: '<S89>/UD' */
    AUAV_V3_TestSensors_DWork.UD_DSTATE_ff = rtb_IC4_idx_1;

    /* Update for Delay: '<S79>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_an = rtb_Switch1_a;

    /* Update for Delay: '<S79>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_i = rtb_Subtract_od;

    /* Update for Delay: '<S81>/Integer Delay3' */
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_fs = rtb_Switch3_gd;

    /* Update for UnitDelay: '<S99>/UD' */
    AUAV_V3_TestSensors_DWork.UD_DSTATE_j = rtb_Product;

    /* Update for Delay: '<S83>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_k = rtb_Switch3_kh;

    /* Update for Delay: '<S83>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_b = rtb_Subtract_a;

    /* Update for Delay: '<S69>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_i = rtb_Ze_b;

    /* Update for Delay: '<S69>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e = rtb_Switch1_a;

    /* Update for Delay: '<S82>/Integer Delay3' */
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_h = rtb_Switch3_kh;

    /* Update for Delay: '<S84>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_p = rtb_Projection;

    /* Update for Delay: '<S84>/Integer Delay1' */
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[0] =
      AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[1];
    AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[1] = rtb_Projection;

    /* Update for Delay: '<S84>/Integer Delay2' */
    AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_g = rtb_Switch1_a;

    /* Update for Delay: '<S100>/Integer Delay3' */
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_nz = rtb_Projection;

    /* Update for Delay: '<S101>/Integer Delay3' */
    AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_hh = rtb_Switch1_a;
  }

  /* End of Update for SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */

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
  /* Gain: '<S20>/Gain' */
  AUAV_V3_TestSensors_B.Gain[0] = AUAV_V3_TestSensors_B.RateTransition[0] << 1;
  AUAV_V3_TestSensors_B.Gain[1] = AUAV_V3_TestSensors_B.RateTransition[1] << 1;
  AUAV_V3_TestSensors_B.Gain[2] = AUAV_V3_TestSensors_B.RateTransition[2] << 1;
  AUAV_V3_TestSensors_B.Gain[3] = AUAV_V3_TestSensors_B.RateTransition[3] << 1;

  /* S-Function "dsPIC_PWM_OC" Block: <S20>/Output Compare - HW Drive Servo motor */
  OC1CON1 = 0x1008;                    /* Disable OC1 */
  OC1CON2bits.TRIGSTAT = 0;
  OC1RS = AUAV_V3_TestSensors_B.Gain[0];/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC2CON1 = 0x1008;                    /* Disable OC2 */
  OC2CON2bits.TRIGSTAT = 0;
  OC2RS = AUAV_V3_TestSensors_B.Gain[1];/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC3CON1 = 0x1008;                    /* Disable OC3 */
  OC3CON2bits.TRIGSTAT = 0;
  OC3RS = AUAV_V3_TestSensors_B.Gain[2];/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC4CON1 = 0x1008;                    /* Disable OC4 */
  OC4CON2bits.TRIGSTAT = 0;
  OC4RS = AUAV_V3_TestSensors_B.Gain[3];/* Pulse in Set-Reset mode (mimic up & down mode) */
  OC1CON2bits.TRIGSTAT = 1;
  OC1CON1 = 0x100C;                    /* Trig OC1 pulse */
  OC2CON2bits.TRIGSTAT = 1;
  OC2CON1 = 0x100C;                    /* Trig OC2 pulse */
  OC3CON2bits.TRIGSTAT = 1;
  OC3CON1 = 0x100C;                    /* Trig OC3 pulse */
  OC4CON2bits.TRIGSTAT = 1;
  OC4CON1 = 0x100C;                    /* Trig OC4 pulse */
}

/* Model step function for TID2 */
void AUAV_V3_TestSensors_step2(void)   /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (MCHP_C_function_Call): '<Root>/ protDecodeMavlink' */
  protDecodeMavlink(
                    );
}

/* Model step function for TID3 */
void AUAV_V3_TestSensors_step3(void)   /* Sample time: [0.1s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID3();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID4 */
void AUAV_V3_TestSensors_step4(void)   /* Sample time: [0.1s, 0.05s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID4();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID5 */
void AUAV_V3_TestSensors_step5(void)   /* Sample time: [0.1s, 0.06s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID5();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID6 */
void AUAV_V3_TestSensors_step6(void)   /* Sample time: [0.1s, 0.08s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID6();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID7 */
void AUAV_V3_TestSensors_step7(void)   /* Sample time: [0.2s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID7();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID8 */
void AUAV_V3_TestSensors_step8(void)   /* Sample time: [0.5s, 0.0s] */
{
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

/* Model step function for TID9 */
void AUAV_V3_TestSensors_step9(void)   /* Sample time: [0.5s, 0.01s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/LEDs_Driver' */
  AUAV_V3_TestSe_LEDs_DriverTID9();

  /* End of Outputs for SubSystem: '<Root>/LEDs_Driver' */
}

/* Model step function for TID10 */
void AUAV_V3_TestSensors_step10(void)  /* Sample time: [0.5s, 0.1s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID10();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID11 */
void AUAV_V3_TestSensors_step11(void)  /* Sample time: [0.5s, 0.2s] */
{
  /* S-Function (MCHP_C_function_Call): '<Root>/ gpsUbloxParse' */
  gpsUbloxParse(
                );
}

/* Model step function for TID12 */
void AUAV_V3_TestSensors_step12(void)  /* Sample time: [0.5s, 0.3s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID12();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID13 */
void AUAV_V3_TestSensors_step13(void)  /* Sample time: [1.0s, 0.1s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID13();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID14 */
void AUAV_V3_TestSensors_step14(void)  /* Sample time: [1.0s, 0.4s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID14();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID15 */
void AUAV_V3_TestSensors_step15(void)  /* Sample time: [1.0s, 0.5s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID15();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID16 */
void AUAV_V3_TestSensors_step16(void)  /* Sample time: [1.0s, 0.7s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID16();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID17 */
void AUAV_V3_TestSensors_step17(void)  /* Sample time: [2.0s, 0.6s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID17();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID18 */
void AUAV_V3_TestSensors_step18(void)  /* Sample time: [2.0s, 0.7s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID18();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID19 */
void AUAV_V3_TestSensors_step19(void)  /* Sample time: [2.0s, 0.8s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID19();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
}

/* Model step function for TID20 */
void AUAV_V3_TestSensors_step20(void)  /* Sample time: [2.0s, 0.9s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/Mavlink_TX_Adapter' */
  AUAV_V3_Mavlink_TX_AdapterTID20();

  /* End of Outputs for SubSystem: '<Root>/Mavlink_TX_Adapter' */
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
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[2] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[3] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[4] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[5] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[6] = 10;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[7] = 20;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[8] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[9] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[10] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[11] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[12] = 50;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[13] = 100;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[14] = 100;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[15] = 100;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[16] = 100;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[17] = 200;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[18] = 200;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[19] = 200;
  (AUAV_V3_TestSensors_M)->Timing.TaskCounters.cLimit[20] = 200;
  rtmSetFirstInitCond(AUAV_V3_TestSensors_M, 1);

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[3] = 9;/* Sample time: [0.1s, 0.01s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[4] = 5;/* Sample time: [0.1s, 0.05s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[5] = 4;/* Sample time: [0.1s, 0.06s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[6] = 2;/* Sample time: [0.1s, 0.08s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[7] = 19;/* Sample time: [0.2s, 0.01s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[9] = 49;/* Sample time: [0.5s, 0.01s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[10] = 40;/* Sample time: [0.5s, 0.1s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[11] = 30;/* Sample time: [0.5s, 0.2s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[12] = 20;/* Sample time: [0.5s, 0.3s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[13] = 90;/* Sample time: [1.0s, 0.1s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[14] = 60;/* Sample time: [1.0s, 0.4s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[15] = 50;/* Sample time: [1.0s, 0.5s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[16] = 30;/* Sample time: [1.0s, 0.7s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[17] = 140;/* Sample time: [2.0s, 0.6s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[18] = 130;/* Sample time: [2.0s, 0.7s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[19] = 120;/* Sample time: [2.0s, 0.8s] */

  /* initialize sample time offsets */
  AUAV_V3_TestSensors_M->Timing.TaskCounters.TID[20] = 110;/* Sample time: [2.0s, 0.9s] */

  /* block I/O */
  (void) memset(((void *) &AUAV_V3_TestSensors_B), 0,
                sizeof(BlockIO_AUAV_V3_TestSensors_T));

  /* states (dwork) */
  (void) memset((void *)&AUAV_V3_TestSensors_DWork, 0,
                sizeof(D_Work_AUAV_V3_TestSensors_T));

  /* exported global states */
  mlParamInterface = AUAV_V3_TestSensors_rtZpi_struct;
  mlWpValues = AUAV_V3_TestSensors_rtZmavlink_mission_item_values_t;
  mlAttitudeData = AUAV_V3_TestSensors_rtZmavlink_attitude_t;
  mlAttitudeSol = AUAV_V3_TestSensors_rtZmavlink_attitude_t;
  mlGpsData = AUAV_V3_TestSensors_rtZmavlink_gps_raw_int_t;
  mlLocalPositionData = AUAV_V3_TestSensors_rtZmavlink_local_position_ned_t;
  mlNavigation = AUAV_V3_TestSensors_rtZmavlink_slugs_navigation_t;
  mlSysStatus = AUAV_V3_TestSensors_rtZmavlink_sys_status_t;
  mlPilotConsoleData = AUAV_V3_TestSensors_rtZmavlink_rc_channels_raw_t;
  mlRawImuData = AUAV_V3_TestSensors_rtZmavlink_raw_imu_t;
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

  /* Start for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  Sensor_Data_Adapter_Start();

  /* End of Start for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* Start for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitud_Start();

  /* End of Start for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* Start for S-Function (MCHP_IC): '<S656>/Input Capture' */
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

  /* Start for InitialCondition: '<S658>/IC1' */
  AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime = true;

  /* Start for Atomic SubSystem: '<S6>/Navigation Encaps [updated 4.28.16]' */
  /* Start for InitialCondition: '<S189>/IC1' */
  AUAV_V3_TestSensors_DWork.IC1_FirstOutputTime_m = true;

  /* Start for InitialCondition: '<S189>/IC2' */
  AUAV_V3_TestSensors_DWork.IC2_FirstOutputTime = true;

  /* Start for InitialCondition: '<S189>/IC4' */
  AUAV_V3_TestSensors_DWork.IC4_FirstOutputTime = true;

  /* Start for InitialCondition: '<S186>/IC' */
  AUAV_V3_TestSensors_DWork.IC_FirstOutputTime = true;

  /* InitializeConditions for IfAction SubSystem: '<S180>/RTB//Follow Mobile Navigation' */
  /* InitializeConditions for DiscreteIntegrator: '<S443>/Discrete-Time Integrator' */
  AUAV_V3_TestSensors_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;

  /* End of InitializeConditions for SubSystem: '<S180>/RTB//Follow Mobile Navigation' */

  /* Start for IfAction SubSystem: '<S180>/Normal WP  Navigation' */

  /* Start for InitialCondition: '<S187>/IC' */
  AUAV_V3_TestSensors_B.IC = 0U;
  AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_m = true;

  /* InitializeConditions for Enabled SubSystem: '<S187>/Get Frenet' */
  /* InitializeConditions for Delay: '<S303>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_gn = 1.0F;

  /* End of InitializeConditions for SubSystem: '<S187>/Get Frenet' */
  /* InitializeConditions for IfAction SubSystem: '<S180>/Normal WP  Navigation' */
  /* InitializeConditions for Delay: '<S187>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_im = 1U;

  /* InitializeConditions for MATLAB Function: '<S187>/Embedded MATLAB Function' */
  AUAV_V3_TestSensors_DWork.persistentDidReachIP = 0U;

  /* InitializeConditions for MATLAB Function: '<S187>/computeCurrentWP' */
  AUAV_V3_TestSensors_DWork.fromWp = 1U;
  AUAV_V3_TestSensors_DWork.toWp = 2U;

  /* End of InitializeConditions for SubSystem: '<S180>/Normal WP  Navigation' */

  /* Start for IfAction SubSystem: '<S180>/Normal WP  Navigation' */
  /* VirtualOutportStart for Outport: '<S187>/FromWP' */
  AUAV_V3_TestSensors_B.WP0 = 1U;

  /* VirtualOutportStart for Outport: '<S187>/ToWP' */
  AUAV_V3_TestSensors_B.WP1 = 2U;

  /* End of Start for SubSystem: '<S180>/Normal WP  Navigation' */
  /* VirtualOutportStart for Outport: '<S185>/FromWP' */
  AUAV_V3_TestSensors_B.Merge3 = 1U;

  /* VirtualOutportStart for Outport: '<S185>/ToWP' */
  AUAV_V3_TestSensors_B.Merge4 = 2U;

  /* End of Start for SubSystem: '<S180>/Line Segment' */

  /* VirtualOutportStart for Outport: '<S181>/FromWP' */
  AUAV_V3_TestSensors_B.Merge3 = 1U;

  /* VirtualOutportStart for Outport: '<S181>/ToWP' */
  AUAV_V3_TestSensors_B.Merge4 = 1U;

  /* End of Start for SubSystem: '<S180>/Circle Navigation' */
  /* Start for InitialCondition: '<S189>/IC3' */
  AUAV_V3_TestSensors_B.IC3 = 0.0F;
  AUAV_V3_TestSensors_DWork.IC3_FirstOutputTime = true;

  /* End of Start for SubSystem: '<S6>/Navigation Encaps [updated 4.28.16]' */

  /* Start for Enabled SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for InitialCondition: '<S79>/IC' */
  AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_g = true;

  /* Start for InitialCondition: '<S83>/IC' */
  AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_f = true;

  /* End of Start for SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */

  /* InitializeConditions for Enabled SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */
  /* InitializeConditions for UnitDelay: '<S89>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE_ff = 0.0F;

  /* InitializeConditions for Delay: '<S79>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_an = 0.0F;

  /* InitializeConditions for Delay: '<S79>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_i = 0.0F;

  /* InitializeConditions for Delay: '<S81>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_fs = 0.0F;

  /* InitializeConditions for UnitDelay: '<S99>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE_j = 0.0F;

  /* InitializeConditions for Delay: '<S83>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_k = 0.0F;

  /* InitializeConditions for Delay: '<S83>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_b = 0.0F;

  /* InitializeConditions for Delay: '<S69>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_i = 0.0F;

  /* InitializeConditions for Delay: '<S69>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_e = 0.0F;

  /* InitializeConditions for Merge: '<S80>/Merge' */
  if (rtmIsFirstInitCond(AUAV_V3_TestSensors_M)) {
    AUAV_V3_TestSensors_B.Merge_oo = 0.0F;
  }

  /* End of InitializeConditions for Merge: '<S80>/Merge' */

  /* InitializeConditions for Delay: '<S82>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_h = 0.0F;

  /* InitializeConditions for Delay: '<S84>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_p = 0.0F;

  /* InitializeConditions for Delay: '<S84>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[0] = 0.0F;
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_ey[1] = 0.0F;

  /* InitializeConditions for Delay: '<S84>/Integer Delay2' */
  AUAV_V3_TestSensors_DWork.IntegerDelay2_DSTATE_g = 0.0F;

  /* InitializeConditions for Delay: '<S100>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_nz = 0.0F;

  /* InitializeConditions for Delay: '<S101>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_hh = 0.0F;

  /* End of InitializeConditions for SubSystem: '<S6>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for InitialCondition: '<S118>/IC' */
  AUAV_V3_TestSensors_DWork.IC_FirstOutputTime_b = true;

  /* InitializeConditions for Enabled SubSystem: '<S102>/Sideslip Compensation' */
  /* InitializeConditions for UnitDelay: '<S126>/UD' */
  AUAV_V3_TestSensors_DWork.UD_DSTATE_f = 0.0F;

  /* InitializeConditions for Delay: '<S118>/Integer Delay' */
  AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_ed = 0.0F;

  /* InitializeConditions for Delay: '<S118>/Integer Delay1' */
  AUAV_V3_TestSensors_DWork.IntegerDelay1_DSTATE_n = 0.0F;

  /* InitializeConditions for Delay: '<S119>/Integer Delay3' */
  AUAV_V3_TestSensors_DWork.IntegerDelay3_DSTATE_ph = 0.0F;

  /* End of InitializeConditions for SubSystem: '<S102>/Sideslip Compensation' */

  /* Start for S-Function (MCHP_OC_HW): '<S20>/Output Compare - HW Drive Servo motor' */
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

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  AUAV_V3_TestSensors_DWork.GS_INIT_FLAG = 1.0;

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

  U4BRG = 0x71;                        /* Baud rate: 38400 (-0.06%) */
  U4MODE = 0x8000;
  U4STA = 0x2400;

  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 5;                         /*  Tx Interrupt priority set to 5 */
  _U4TXIF = 0;                         /*  */
  _U4TXIE = 1;                         /* Enable Interrupt */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Sensor_Data_Adapter' */
  A_Sensor_Data_Adapter_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Sensor_Data_Adapter' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/Position_and_Attitude_Filter' */
  Position_and_Attitude_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Position_and_Attitude_Filter' */

  /* InitializeConditions for MATLAB Function: '<S660>/Buffer Failsafe Channel' */
  AUAV__BufferICChannel_Init(&AUAV_V3_TestSensors_DWork.sf_BufferFailsafeChannel);

  /* InitializeConditions for MATLAB Function: '<S655>/Buffer IC Channel' */
  AUAV__BufferICChannel_Init(&AUAV_V3_TestSensors_DWork.sf_BufferICChannel);

  /* InitializeConditions for MATLAB Function: '<S655>/Buffer IC Channel1' */
  AUAV__BufferICChannel_Init(&AUAV_V3_TestSensors_DWork.sf_BufferICChannel1);

  /* InitializeConditions for MATLAB Function: '<S655>/Buffer IC Channel2' */
  AUAV__BufferICChannel_Init(&AUAV_V3_TestSensors_DWork.sf_BufferICChannel2);

  /* InitializeConditions for MATLAB Function: '<S655>/Buffer IC Channel3' */
  AUAV__BufferICChannel_Init(&AUAV_V3_TestSensors_DWork.sf_BufferICChannel3);

  /* InitializeConditions for Atomic SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */
  /* InitializeConditions for MATLAB Function: '<S165>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_n);

  /* InitializeConditions for UnitDelay: '<S178>/FixPt Unit Delay2' */
  AUAV_V3_TestSensors_DWork.FixPtUnitDelay2_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S141>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_i);

  /* End of InitializeConditions for SubSystem: '<S6>/Longitudinal Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for Atomic SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for MATLAB Function: '<S104>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_b0);

  /* End of InitializeConditions for SubSystem: '<S6>/Lateral Channel Encaps [updated 4.28.16]' */

  /* InitializeConditions for MATLAB Function: '<S665>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction);

  /* InitializeConditions for MATLAB Function: '<S666>/Embedded MATLAB Function' */
  EmbeddedMATLABFunctio_Init
    (&AUAV_V3_TestSensors_DWork.sf_EmbeddedMATLABFunction_b);

  /* InitializeConditions for Atomic SubSystem: '<Root>/Barometer_Driver' */
  AUAV_Barometer_Driver_Init();

  /* End of InitializeConditions for SubSystem: '<Root>/Barometer_Driver' */

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(AUAV_V3_TestSensors_M)) {
    rtmSetFirstInitCond(AUAV_V3_TestSensors_M, 0);
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
