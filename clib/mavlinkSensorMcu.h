#ifndef _MAVLINKSENSORMCU_H_
#define _MAVLINKSENSORMCU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "apDefinitions.h"
#include "mavlink.h"


    typedef struct _mavlink_coordinate_float_t {
        float lat;
        float lon;
        float alt;
    } mavlink_coordinate_float_t;

    extern mavlink_gps_raw_int_t mlGpsData; //
    extern mavlink_cpu_load_t mlCpuLoadData; //
    extern mavlink_scaled_pressure_t mlAirData; //
    extern mavlink_sensor_bias_t mlSensorBiasData; //
    extern mavlink_diagnostic_t mlDiagnosticData; //
    extern mavlink_rc_channels_raw_t mlPilotConsoleData; //
    extern mavlink_raw_imu_t mlRawImuData; //
    extern mavlink_raw_pressure_t mlRawPressureData; //
    extern mavlink_attitude_t mlAttitudeData; //
    extern mavlink_local_position_ned_t mlLocalPositionData; //
    extern mavlink_scaled_imu_t mlFilteredData; //
    extern mavlink_boot_t mlBoot; //
    extern mavlink_gps_date_time_t mlGpsDateTime;
    extern mavlink_coordinate_float_t mlGSLocationFloat;
#if USE_NMEA
    extern mavlink_status_gps_t mlGpsStatus;
#endif
    extern mavlink_novatel_diag_t mlNovatelStatus;
    extern mavlink_sensor_diag_t mlSensorDiag;
    // via SPI
    extern mavlink_set_gps_global_origin_t mlGSLocation;
    extern mavlink_command_ack_t mlCommandAck;
    extern mavlink_servo_output_raw_t mlPwmCommands;

    // Debug
    extern mavlink_statustext_t mlStatustext;

    // Others
    extern BOOL sendCommandAcknowledgement;


    void mavlinkInit(void);


#ifdef __cplusplus
}
#endif

#endif /* _MAVLINKSENSORMCU_H_ */
