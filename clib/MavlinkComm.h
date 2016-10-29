#ifndef  _MAVLINK_COMM_H_
#define  _MAVLINK_COMM_H_
#include "circBuffer.h"
#include <rtwtypes.h>
#include "inttypes.h"
#include "mavlink.h"


extern CBRef uartMavlinkInBuffer;
extern struct CircBuffer comMavlinkBuffer;
void uartMavlinkBufferInit (void);


#define TRUE ((char)1)
#define FALSE ((char)0)

#define SUCCESS ((char)0)
#define FAILURE ((char)-1)

typedef char BOOL;
#define TRUE ((char)1)
#define FALSE ((char)0)

#define SUCCESS ((char)0)
#define FAILURE ((char)-1)

#define SYSTEMID	101
#define COMPID		1
#define GS_SYSTEMID	127
#define GS_COMPID	0

#define MAXINLEN        200
#define MAXSEND         109
#define MAX_NUM_WPS     17
#define PARAM_NAME_LENGTH	16

#define PROTOCOL_TIMEOUT_TICKS 20    ///< maximum time to wait for pending messages until timeout

  enum WP_PROTOCOL {
    WP_PROT_IDLE,
    WP_PROT_LIST_REQUESTED,
    WP_PROT_NUM_SENT,
    WP_PROT_TX_WP,
    WP_PROT_RX_WP,
    WP_PROT_SENDING_WP_IDLE,
    WP_PROT_GETTING_WP_IDLE
  };

  enum PARAM_INTERFACE {
    PI_IDLE,
    PI_SEND_ALL_PARAM,
    PI_SEND_ONE_PARAM
  };

  enum PARAM_INTERFACE_IDX {
    PAR_PID_AIRSPEED_P = 0,
    PAR_PID_AIRSPEED_I = 1,
    PAR_PID_AIRSPEED_D = 2,

    PAR_PID_PITCH_FO_P = 3,
    PAR_PID_PITCH_FO_I = 4,
    PAR_PID_PITCH_FO_D = 5,

    PAR_PID_ROLL_CON_P = 6,
    PAR_PID_ROLL_CON_I = 7,
    PAR_PID_ROLL_CON_D = 8,

    PAR_PID_HE_TO_PI_P = 9,
    PAR_PID_HE_TO_PI_I = 10,

    PAR_PID_HEI_ERR_FF = 11,

    PAR_PID_YAW_DAMP_P = 12,
    PAR_PID_YAW_DAMP_I = 13,
    PAR_PID_YAW_DAMP_D = 14,

    PAR_PID_PITC_DT_FF = 15,

        PAR_CONFIG_ROLL_R = 16,
        PAR_CONFIG_PITCH_R = 17,
        PAR_CONFIG_YAW_R = 18,

        PAR_NAV_L2_BASE = 19,
        PAR_NAV_PRETURN_K = 20,
        PAR_NAV_SSCOMP_ON = 21,

        PAR_L1_OMEGA = 22,
        PAR_L1_M = 23,
        PAR_L1_GAMMA = 24,
        PAR_L1_ON_OFF = 25,

        PAR_NAV_ISR_FAC = 26,
        PAR_PID_RMIX_ON = 27,
        PAR_PID_RMIX_P = 28,

        PAR_CAM_X = 29,
        PAR_CAM_Z = 30,

        /*
        PAR_RATE_TELEMETRY = 31, // attitude sent at this rate, others 1/10th
        */
    PAR_PARAM_COUNT // Always at the end, do not assign value
  };
  
  typedef struct pi_struct {
    float param[PAR_PARAM_COUNT];
    char param_name[PAR_PARAM_COUNT][PARAM_NAME_LENGTH];
  } pi_struct;

   typedef struct mavlink_pending_requests_t {
        // requests
        uint8_t ping;
        uint8_t midLvlCmds;
        uint8_t pt;
        uint8_t isrLoc;

        // Acknowledgement counters
        uint8_t commandAck;
        uint8_t wpAck;

        // Stuff to send
        uint8_t statustext;
        uint8_t command;

        // slugs Acknowledge
        // uint8_t slugsAction; // Note: migrate slugs actions into commands, acks, and text

        // WP Protocol states
        uint8_t wpTransaction;
        uint8_t wpProtState;
        uint8_t wpCurrentWpInTransaction;
        uint8_t wpTimeOut;
        uint8_t wpTotalWps;
        uint8_t wpSendCurrent; // send current mission item

        // Info
        uint8_t pidIdx;
        uint8_t wpsIdx;

        //uint8_t 	requestCount;

        // Parameter Interface
        uint8_t piTransaction;
        uint8_t piProtState;
        uint8_t piCurrentParamInTransaction;
        uint8_t piBackToList;
        uint8_t piQueue[5];
        int8_t piQIdx;

        // spi
        uint8_t spiToSensor[MAXSEND];
        uint8_t spiCurrentIndex;
        uint8_t spiTotalData;
        uint8_t spiSendGSLocation;

        // Heartbeat status
        uint16_t heartbeatAge;

    } mavlink_pending_requests_t;

    typedef struct mavlink_mission_item_values_t {
        float lat[MAX_NUM_WPS];
        float lon[MAX_NUM_WPS];
        float alt[MAX_NUM_WPS];
        uint8_t type[MAX_NUM_WPS];
        uint16_t orbit[MAX_NUM_WPS];
        uint8_t wpCount;
    } mavlink_mission_item_values_t;

	uint16_t PackHeartBeat(uint8_t system_id, uint8_t component_id);
    uint16_t PackRawServo(uint8_t system_id, uint8_t component_id, mavlink_servo_output_raw_t mlPwmCommands ,uint32_t time_usec);

    uint16_t HIL_PackRawServo(uint8_t system_id, uint8_t component_id, mavlink_servo_output_raw_t mlPwmCommands ,uint32_t time_usec);
        
	uint16_t PackRawRC(uint8_t system_id, uint8_t component_id, mavlink_rc_channels_raw_t mlRC_Commands, uint32_t time_usec);
	uint16_t PackVFR_HUD(uint8_t system_id, uint8_t component_id, mavlink_vfr_hud_t mlVfr_hud, uint32_t time_usec);


    uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id, mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec);
    uint16_t PackRawAttitude(uint8_t system_id, uint8_t component_id, mavlink_attitude_t mlAttitudeData ,uint32_t time_usec);
    uint16_t PackPosXYZ_Sol(uint8_t system_id, uint8_t component_id, mavlink_local_position_ned_t mlLocalPositionSol ,uint32_t time_usec);

char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) ;
uint16_t PackGpsRawInt(uint8_t system_id, uint8_t component_id, mavlink_gps_raw_int_t mlRawGpsDataInt, uint32_t time_usec);
uint16_t PackScaledPressure(uint8_t system_id, uint8_t component_id, mavlink_scaled_pressure_t mlAirData, uint32_t time_usec);
uint16_t PackSysStatus(uint8_t system_id, uint8_t component_id, mavlink_sys_status_t mlSysStatus);


//uint8_t GetCharAtIdx(int2_t idx);
#endif