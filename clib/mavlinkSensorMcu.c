
#include <.\rtwtypes.h>
#include "mavlinkSensorMcu.h"
#include "apDefinitions.h"


// Declare the global structures that will hold the state of the Sensor MCU

BOOL sendCommandAcknowledgement; // used for IPC of command acks to control MCU


//mavlink_raw_imu_t mlRawImuData;
//mavlink_gps_raw_int_t mlGpsData;
mavlink_cpu_load_t mlCpuLoadData;
//mavlink_scaled_pressure_t mlAirData;
mavlink_sensor_bias_t mlSensorBiasData;
mavlink_diagnostic_t mlDiagnosticData;
//mavlink_raw_pressure_t mlRawPressureData;
//mavlink_attitude_t mlAttitudeData;
//mavlink_local_position_ned_t mlLocalPositionData;
//mavlink_rc_channels_raw_t mlPilotConsoleData;
mavlink_scaled_imu_t mlFilteredData;
mavlink_boot_t mlBoot;
mavlink_gps_date_time_t mlGpsDateTime;
//mavlink_coordinate_float_t mlGSLocationFloat;
#if USE_NMEA
mavlink_status_gps_t mlGpsStatus;
#endif
mavlink_novatel_diag_t mlNovatelStatus;
mavlink_sensor_diag_t mlSensorDiag;


// via SPI
//mavlink_set_gps_global_origin_t mlGSLocation;
mavlink_command_ack_t mlCommandAck;
extern mavlink_servo_output_raw_t mlPwmCommands;

// Debug
mavlink_statustext_t mlStatustext;

/**********************************************************************
 * Function: mavlinkInit
 * @return none
 * @remark Initializes the sensor MCU's mavlink message structures.
 ***********************************************************************/
void mavlinkInit(void) {

    // clear all the variables
    memset(&mlGpsData, 0, sizeof (mavlink_gps_raw_int_t));
    memset(&mlCpuLoadData, 0, sizeof (mavlink_cpu_load_t));
    memset(&mlAirData, 0, sizeof (mavlink_scaled_pressure_t));
    memset(&mlSensorBiasData, 0, sizeof (mavlink_sensor_bias_t));
    memset(&mlDiagnosticData, 0, sizeof (mavlink_diagnostic_t));
    memset(&mlPilotConsoleData, 0, sizeof (mavlink_rc_channels_raw_t));
    memset(&mlRawImuData, 0, sizeof (mavlink_raw_imu_t));
    memset(&mlRawPressureData, 0, sizeof (mavlink_raw_pressure_t));
    memset(&mlAttitudeData, 0, sizeof (mavlink_attitude_t));
    memset(&mlLocalPositionData, 0, sizeof (mavlink_local_position_ned_t));
    memset(&mlFilteredData, 0, sizeof (mavlink_scaled_imu_t));
    memset(&mlBoot, 0, sizeof (mavlink_boot_t));
    memset(&mlGpsDateTime, 0, sizeof (mavlink_gps_date_time_t));
    memset(&mlGSLocationFloat, 0, sizeof (mavlink_coordinate_float_t));
#if USE_NMEA
    memset(&mlGpsStatus, 0, sizeof(mavlink_status_gps_t));
#else 
    memset(&mlNovatelStatus, 0, sizeof(mavlink_novatel_diag_t));
#endif
    memset(&mlSensorDiag, 0, sizeof(mavlink_sensor_diag_t));

    //spi	
    memset(&mlGSLocation, 0, sizeof (mavlink_set_gps_global_origin_t));
    memset(&mlCommandAck, 0, sizeof (mavlink_command_ack_t));
    memset(&mlPwmCommands, 0, sizeof (mavlink_servo_output_raw_t));

    // Debug
    memset(&mlStatustext, 0, sizeof(mavlink_statustext_t));

    // Flags
    sendCommandAcknowledgement = FALSE;

}
