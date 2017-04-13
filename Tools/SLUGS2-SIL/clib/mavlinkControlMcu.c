#include "MavlinkComm.h"
#include "mavlinkControlMcu.h"
#include "SLUGS2.h"

//mavlink_raw_imu_t mlRawImuData;
//mavlink_gps_raw_int_t mlGpsData;
//mavlink_cpu_load_t mlCpuLoadData;
//mavlink_scaled_pressure_t mlAirData;
//mavlink_sensor_bias_t mlSensorBiasData;
//mavlink_diagnostic_t mlDiagnosticData;
//mavlink_raw_pressure_t mlRawPressureData;
//mavlink_attitude_t mlAttitudeData;
//mavlink_local_position_ned_t mlLocalPositionData;
//mavlink_rc_channels_raw_t mlPilotConsoleData;
//mavlink_scaled_imu_t mlFilteredData;
//mavlink_boot_t mlBoot;
//mavlink_gps_date_time_t mlGpsDateTime;

//mavlink_heartbeat_t mlHeartbeat;
//mavlink_heartbeat_t mlHeartbeatLocal;
//mavlink_mid_lvl_cmds_t mlMidLevelCommands;
mavlink_set_mode_t mlApMode;
//mavlink_servo_output_raw_t mlPwmCommands;
//mavlink_mission_item_values_t mlWpValues; //defined in mavlinkControlMcu.h
//mavlink_mission_item_t mlSingleWp;
//mavlink_slugs_navigation_t mlNavigation;
mavlink_data_log_t mlDataLog;
mavlink_ctrl_srfc_pt_t mlPassthrough;
mavlink_attitude_t mlAttitudeRotated;
//mavlink_command_ack_t mlCommandAck;
//mavlink_pending_requests_t mlPending;
mavlink_rtb_t mlRTB;
//mavlink_statustext_t mlStatustext;
mavlink_ping_t mlPing;
// mavlink_slugs_action_t mlAction; 
mavlink_command_long_t mlCommand;
//mavlink_mission_request_t mlWpRequest;
//mavlink_mission_ack_t mlWpAck;
//mavlink_mission_count_t mlWpCount;
mavlink_sys_status_t mlSystemStatus;
mavlink_slugs_camera_order_t mlCameraOrder;
//mavlink_slugs_rtb_t mlRTB; // moved to custom structure for migration to command_long
//mavlink_slugs_mobile_location_t mlMobileLocation;
//mavlink_isr_location_t mlISR;
mavlink_turn_light_t mlLights;
mavlink_slugs_configuration_camera_t mlCameraConfig;
mavlink_ptz_status_t mlPtzStatus;
//mavlink_volt_sensor_t mlVISensor;
#if USE_NMEA
//mavlink_status_gps_t mlGpsStatus;
#else 
mavlink_novatel_diag_t mlNovatelStatus;
#endif
//mavlink_sensor_diag_t mlSensorDiag;

//Referencing the global declared in "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_types.h"
//struct pi_struct mlParamInterface;
//pi_struct mlParamInterface;

//struct gps_float_struct mlGpsFloat; // float


// via SPI
//mavlink_set_gps_global_origin_t mlGSLocation;

uint32_t lastNavigationMode;


/**********************************************************************
 * Function: mavlinkInit
 * @return none
 * @remark Initializes the control MCU's mavlink message structures.
 ***********************************************************************/
// void mavlinkInit(void)
// {

    // // clear all the variables
    // memset(&mlGpsData, 0, sizeof (mavlink_gps_raw_int_t));
    // memset(&mlCpuLoadData, 0, sizeof (mavlink_cpu_load_t));
    // memset(&mlAirData, 0, sizeof (mavlink_scaled_pressure_t));
    // memset(&mlSensorBiasData, 0, sizeof (mavlink_sensor_bias_t));
    // memset(&mlDiagnosticData, 0, sizeof (mavlink_diagnostic_t));
    // memset(&mlPilotConsoleData, 0, sizeof (mavlink_rc_channels_raw_t));
    // memset(&mlRawImuData, 0, sizeof (mavlink_raw_imu_t));
    // memset(&mlRawPressureData, 0, sizeof (mavlink_raw_pressure_t));
    // memset(&mlAttitudeData, 0, sizeof (mavlink_attitude_t));
    // memset(&mlLocalPositionData, 0, sizeof (mavlink_local_position_ned_t));
    // memset(&mlFilteredData, 0, sizeof (mavlink_scaled_imu_t));
    // memset(&mlBoot, 0, sizeof (mavlink_boot_t));
    // memset(&mlGpsDateTime, 0, sizeof (mavlink_gps_date_time_t));
    // memset(&mlHeartbeat, 0, sizeof (mavlink_heartbeat_t));
    // memset(&mlHeartbeatLocal, 0, sizeof (mavlink_heartbeat_t));


    // memset(&mlMidLevelCommands, 0, sizeof (mavlink_mid_lvl_cmds_t));
    // memset(&mlPwmCommands, 0, sizeof (mavlink_servo_output_raw_t));
    // memset(&mlWpValues, 0, sizeof (mavlink_mission_item_values_t));
    // memset(&mlSingleWp, 0, sizeof (mavlink_mission_item_t));
    // memset(&mlNavigation, 0, sizeof (mavlink_slugs_navigation_t));
    // memset(&mlDataLog, 0, sizeof (mavlink_data_log_t));
    // memset(&mlPassthrough, 0, sizeof (mavlink_ctrl_srfc_pt_t));
    // memset(&mlAttitudeRotated, 0, sizeof (mavlink_attitude_t));
    // memset(&mlCommandAck, 0, sizeof (mavlink_command_ack_t));
    // memset(&mlPending, 0, sizeof (mavlink_pending_requests_t));
    // memset(&mlPing, 0, sizeof (mavlink_ping_t));
    // //memset(&mlAction, 0, sizeof (mavlink_slugs_action_t));
    // memset(&mlWpRequest, 0, sizeof (mavlink_mission_request_t));
    // memset(&mlWpAck, 0, sizeof (mavlink_mission_ack_t));
    // memset(&mlWpCount, 0, sizeof (mavlink_mission_count_t));
    // memset(&mlSystemStatus, 0, sizeof (mavlink_sys_status_t));

    // memset(&mlParamInterface, 0, sizeof (struct pi_struct));
    // //memset(&mlGpsFloat, 0, sizeof (struct gps_float_struct));
    // memset(&mlGSLocation, 0, sizeof (mavlink_set_gps_global_origin_t));
    // memset(&mlCameraOrder, 0, sizeof(mavlink_slugs_camera_order_t));
    // memset(&mlRTB, 0, sizeof(mavlink_rtb_t));
    // memset(&mlMobileLocation, 0, sizeof(mavlink_slugs_mobile_location_t));
    // memset(&mlISR, 0, sizeof(mavlink_isr_location_t));
    // memset(&mlLights, 0, sizeof(mavlink_turn_light_t));
    // memset(&mlCameraConfig, 0, sizeof(mavlink_slugs_configuration_camera_t));
    // memset(&mlPtzStatus, 0, sizeof(mavlink_ptz_status_t));
    // memset(&mlVISensor, 0, sizeof(mavlink_volt_sensor_t));
// #if USE_NMEA
    // memset(&mlGpsStatus, 0, sizeof(mavlink_status_gps_t));
// #else 
    // memset(&mlNovatelStatus, 0, sizeof(mavlink_novatel_diag_t));
// #endif
    // memset(&mlSensorDiag, 0, sizeof(mavlink_sensor_diag_t));
    
    // //mlCommandAck.command = SLUGS_ACTION_NONE; // Note: ensure flags are added instead

    // mlPending.wpTransaction = 0;
    // mlPending.wpProtState = WP_PROT_IDLE;
    // mlPending.wpTimeOut = 0;
    
    // mlPending.heartbeatAge = 0;

    // mlHeartbeatLocal.mavlink_version = MAVLINK_VERSION;

    // // Initialize the system Status
    // mlHeartbeatLocal.base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    // mlHeartbeatLocal.custom_mode = SLUGS_MODE_WAYPOINT;
    // mlHeartbeatLocal.system_status = MAV_STATE_ACTIVE;
    // //lastNavigationMode = mlHeartbeatLocal.custom_mode;
    // //mlSystemStatus.mode = MAV_MODE_MANUAL;
    // //mlSystemStatus.nav_mode = MAV_NAV_WAYPOINT;
    // //mlSystemStatus.status = MAV_STATE_ACTIVE;
    // mlSystemStatus.load = 500;
    // mlSystemStatus.voltage_battery = 0;
    
    // mlCameraOrder.zoom = 1;
    
    // mlBoot.version = 1;

    // // Populate default mid-level commands
    // mlMidLevelCommands.hCommand = 120.0f; // altitude (m)
    // mlMidLevelCommands.uCommand = 16.0f; // airspeed (m/s)
    // mlMidLevelCommands.rCommand = 0.0f; // turn rate (radians/s)


    // populateParameterInterface();

    // SPI

// }

// TODO: Commented out for compilation, need to potentially initialize these PARAM values in the model.
void populateParameterInterface(void)
{
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_P], "PID_AIRSPD_P");
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_I], "PID_AIRSPD_I");
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_D], "PID_AIRSPD_D");

    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_P], "PID_PIT_FO_P");
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_I], "PID_PIT_FO_I");
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_D], "PID_PIT_FO_D");

    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_P], "PID_ROLL_CO_P");
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_I], "PID_ROLL_CO_I");
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_D], "PID_ROLL_CO_D");

    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_P], "PID_HE2PITC_P");
    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_I], "PID_HE2PITC_I");

    strcpy(mlParamInterface.param_name[PAR_PID_HEI_ERR_FF], "PID_HERR_FF");

    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_P], "PID_YAW_DA_P");
    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_I], "PID_YAW_DA_I");
    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_D], "PID_YAW_DA_D");

    strcpy(mlParamInterface.param_name[PAR_PID_PITC_DT_FF], "PID_PIT_DT_FF");

    strcpy(mlParamInterface.param_name[PAR_CONFIG_ROLL_R ], "CONFIG_ROLL_R");
    strcpy(mlParamInterface.param_name[PAR_CONFIG_PITCH_R], "CONFIG_PITCH_R");
    strcpy(mlParamInterface.param_name[PAR_CONFIG_YAW_R ], "CONFIG_YAW_R");

    strcpy(mlParamInterface.param_name[PAR_NAV_L2_BASE ], "NAV_L2_BASE");
    strcpy(mlParamInterface.param_name[PAR_NAV_PRETURN_K], "NAV_PRETURN_K");
    strcpy(mlParamInterface.param_name[PAR_NAV_SSCOMP_ON], "NAV_SSCOMP_ON");

    strcpy(mlParamInterface.param_name[PAR_L1_OMEGA ], "L1_OMEGA");
    strcpy(mlParamInterface.param_name[PAR_L1_M ], "L1_M")  ;
    strcpy(mlParamInterface.param_name[PAR_L1_GAMMA ], "L1_GAMMA");
    strcpy(mlParamInterface.param_name[PAR_L1_ON_OFF], "L1_ON_OFF");

    strcpy(mlParamInterface.param_name[PAR_NAV_ISR_FAC ], "NAV_ISR_FACT");
    strcpy(mlParamInterface.param_name[PAR_PID_RMIX_ON ], "PID_RMIX_ON");
    strcpy(mlParamInterface.param_name[PAR_PID_RMIX_P], "PID_RMIX_P");
    
    strcpy(mlParamInterface.param_name[PAR_CAM_X], "CAM_LOC_X");
    strcpy(mlParamInterface.param_name[PAR_CAM_Z], "CAM_LOC_Z");

    
    //strcpy(mlParamInterface.param_name[PAR_RATE_TELEMETRY], "RATE_TELEMETRY");
     


}

