#ifndef _MAVLINKCONTROLMCU_H_
#define _MAVLINKCONTROLMCU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "MavlinkComm.h"
#include "mavlink.h"
#include "apDefinitions.h"

#include "circBuffer.h"

    // ======== PARAMETER INTERFACE DATA ========

    // NOTE: The EEPROM can hold a Maximum of 79 floats
    //       DO NOT EXCEED THAT NUMBER!

    // If you add a new parameter remember to update the 
    // apConfiguration/parameterEnums.m file to reflect that

    // NOTE: EEPROM storage space was effectively doubled for the dsPIC33F by
    // modifications made to the EEPROM emulation library, DEE.c, made in 2011.

//    enum SLUGS_PARAM_INTERFACE_IDX {
//        PAR_PID_AIRSPEED_P = 0,
//        PAR_PID_AIRSPEED_I = 1,
//        PAR_PID_AIRSPEED_D = 2,
//
//        PAR_PID_PITCH_FO_P = 3,
//        PAR_PID_PITCH_FO_I = 4,
//        PAR_PID_PITCH_FO_D = 5,
//
//        PAR_PID_ROLL_CON_P = 6,
//        PAR_PID_ROLL_CON_I = 7,
//        PAR_PID_ROLL_CON_D = 8,
//
//        PAR_PID_HE_TO_PI_P = 9,
//        PAR_PID_HE_TO_PI_I = 10,
//
//        PAR_PID_HEI_ERR_FF = 11,
//
//        PAR_PID_YAW_DAMP_P = 12,
//        PAR_PID_YAW_DAMP_I = 13,
//        PAR_PID_YAW_DAMP_D = 14,
//
//        PAR_PID_PITC_DT_FF = 15,
//
//        PAR_CONFIG_ROLL_R = 16,
//        PAR_CONFIG_PITCH_R = 17,
//        PAR_CONFIG_YAW_R = 18,
//
//        PAR_NAV_L2_BASE = 19,
//        PAR_NAV_PRETURN_K = 20,
//        PAR_NAV_SSCOMP_ON = 21,
//
//        PAR_L1_OMEGA = 22,
//        PAR_L1_M = 23,
//        PAR_L1_GAMMA = 24,
//        PAR_L1_ON_OFF = 25,
//
//        PAR_NAV_ISR_FAC = 26,
//        PAR_PID_RMIX_ON = 27,
//        PAR_PID_RMIX_P = 28,
//
//        PAR_CAM_X = 29,
//        PAR_CAM_Z = 30,
//
//        /*
//        PAR_RATE_TELEMETRY = 31, // attitude sent at this rate, others 1/10th
//        */
//
//        PAR_PARAM_COUNT // Always at the end, do not assign value
//    };


#define SLUGS_PARAM_NAME_LENGTH	16

//    struct pi_struct {
//        float param[PAR_PARAM_COUNT];
//        char param_name[PAR_PARAM_COUNT][SLUGS_PARAM_NAME_LENGTH];
//    };

    struct gps_float_struct {
        float lat;
        float lon;
        float alt;
        float eph;
        float epv;
        float vel;
        float cog;
    };

//    typedef struct mavlink_mission_item_values_t {
//        float lat[MAX_NUM_WPS];
//        float lon[MAX_NUM_WPS];
//        float alt[MAX_NUM_WPS];
//        uint8_t type[MAX_NUM_WPS];
//        uint16_t orbit[MAX_NUM_WPS];
//        uint8_t wpCount;
//    } mavlink_mission_item_values_t;
//
    // See MAV_CMD_RETURN_TO_BASE for details
    typedef struct mavlink_rtb_t {
        uint8_t rtb;
        uint8_t track_mobile;
    } mavlink_rtb_t;

    // See MAV_CMD_TURN_LIGHT for details
    typedef struct mavlink_turn_light_t {
        uint8_t state;
        uint8_t type;
    } mavlink_turn_light_t;

//    typedef struct mavlink_pending_requests_t {
//        // requests
//        uint8_t ping;
//        uint8_t midLvlCmds;
//        uint8_t pt;
//        uint8_t isrLoc;
//
//        // Acknowledgement counters
//        uint8_t commandAck;
//        uint8_t wpAck;
//
//        // Stuff to send
//        uint8_t statustext;
//        uint8_t command;
//
//        // slugs Acknowledge
//        // uint8_t slugsAction; // Note: migrate slugs actions into commands, acks, and text
//
//        // WP Protocol states
//        uint8_t wpTransaction;
//        uint8_t wpProtState;
//        uint8_t wpCurrentWpInTransaction;
//        uint8_t wpTimeOut;
//        uint8_t wpTotalWps;
//        uint8_t wpSendCurrent; // send current mission item
//
//        // Info
//        uint8_t pidIdx;
//        uint8_t wpsIdx;
//
//        //uint8_t 	requestCount;
//
//        // Parameter Interface
//        uint8_t piTransaction;
//        uint8_t piProtState;
//        uint8_t piCurrentParamInTransaction;
//        uint8_t piBackToList;
//        uint8_t piQueue[5];
//        int8_t piQIdx;
//
//        // spi
//        uint8_t spiToSensor[MAXSEND];
//        uint8_t spiCurrentIndex;
//        uint8_t spiTotalData;
//        uint8_t spiSendGSLocation;
//
//        // Heartbeat status
//        uint16_t heartbeatAge;
//
//    } mavlink_pending_requests_t;


    // Comments Nomenclature
    // // In init method
    //
    // .. Already in Decoder (thus we can receive messages of this type)
    // -- Does not need Decoder (messages are only outgoing)
    //	
    // == Scheduled for regular transmision
    // ** It does not need to be scheduled
    //	
    // ^^ Temporary value does not require decode nor encode


    extern mavlink_raw_imu_t mlRawImuData; // 	..	==
    extern mavlink_gps_raw_int_t mlGpsData; // 	..	==
    extern mavlink_cpu_load_t mlCpuLoadData; // 	..	== 
    extern mavlink_scaled_pressure_t mlAirData; // 	..	== 
    extern mavlink_sensor_bias_t mlSensorBiasData; // 	..	== 	
    extern mavlink_diagnostic_t mlDiagnosticData; // 	..	== 
    extern mavlink_raw_pressure_t mlRawPressureData; // 	..	== 
    extern mavlink_attitude_t mlAttitudeData; // 	..	== 
    extern mavlink_local_position_ned_t mlLocalPositionData; // 	.. 	==
    extern mavlink_rc_channels_raw_t mlPilotConsoleData; // 	..	==	
    extern mavlink_scaled_imu_t mlFilteredData; // 	..
    extern mavlink_boot_t mlBoot; //	..	==
    extern mavlink_gps_date_time_t mlGpsDateTime; // 	..

    extern mavlink_heartbeat_t mlHeartbeat; // 	..	==
    extern mavlink_heartbeat_t mlHeartbeatLocal; // 	..	==
    extern mavlink_mid_lvl_cmds_t mlMidLevelCommands; // 	..	**
    extern mavlink_servo_output_raw_t mlPwmCommands; // 	--	==
    extern mavlink_mission_item_values_t mlWpValues; // 	..	**	defined in mavlinkControlMcu.h
    extern mavlink_mission_item_t mlSingleWp; // 	^^	^^
    extern mavlink_slugs_navigation_t mlNavigation; // 	--  ==		
    extern mavlink_data_log_t mlDataLog; // 	--	==
    extern mavlink_ctrl_srfc_pt_t mlPassthrough; // 	..	**
    extern mavlink_attitude_t mlAttitudeRotated; // 	--	==
    extern mavlink_command_long_t mlCommand;
    extern mavlink_command_ack_t mlCommandAck; // 	-- 	==
    extern mavlink_pending_requests_t mlPending; //	--	**
    extern mavlink_ping_t mlPing; // 	..	**
    //extern mavlink_slugs_action_t mlAction; // 	..	**
    extern mavlink_mission_request_t mlWpRequest; // 	..	**
    extern mavlink_mission_ack_t mlWpAck; // 	..	**
    extern mavlink_mission_count_t mlWpCount; // 	..	**
    extern mavlink_sys_status_t mlSystemStatus; //	--	==
    //extern struct pi_struct mlParamInterface;
    extern pi_struct mlParamInterface;
    extern mavlink_slugs_camera_order_t mlCameraOrder;
    extern mavlink_rtb_t mlRTB;
    extern mavlink_slugs_mobile_location_t mlMobileLocation;
    extern mavlink_isr_location_t mlISR;
    extern mavlink_turn_light_t mlLights;
    extern mavlink_statustext_t mlStatustext;
    extern mavlink_slugs_configuration_camera_t mlCameraConfig;
    extern mavlink_ptz_status_t mlPtzStatus;
    extern mavlink_volt_sensor_t mlVISensor;
#if USE_NMEA
    extern mavlink_status_gps_t mlGpsStatus;
#else 
    extern mavlink_novatel_diag_t mlNovatelStatus;
#endif
    extern mavlink_sensor_diag_t mlSensorDiag;
    
    // SPI
    extern mavlink_set_gps_global_origin_t mlGSLocation;

    //extern gps_float_struct mlGpsFloat;

    extern uint32_t lastNavigationMode;


    //
    void mavlinkInit(void);
    void populateParameterInterface(void);


#ifdef __cplusplus
}
#endif

#endif /* _MAVLINKCONTROLMCU_H_ */
