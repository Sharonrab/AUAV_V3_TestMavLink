


//#include "defines.h"
//#include "behaviour.h"
//#include "servoPrepare.h"
//#include "../libDCM/gpsParseCommon.h"
//#include "config.h"
//#include "flightplan-waypoints.h"
#include "libUDB.h"

#include <setjmp.h>

#include "mavlink.h"
/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif


//#if (SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) 
//void parameter_table_init(void);
//#endif

static jmp_buf buf;
const mavlink_scaled_pressure_t AUAV_V3_TestMavLink_rtZmavlink_scaled_pressure_t
= {
	0U,                                  /* time_boot_ms */
	0.0F,                                /* press_abs */
	0.0F,                                /* press_diff */
	0                                    /* temperature */
};                                    /* mavlink_scaled_pressure_t ground */

const mavlink_gps_raw_int_t AUAV_V3_TestMavLink_rtZmavlink_gps_raw_int_t = {
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
};                                    /* mavlink_gps_raw_int_t ground */

const mavlink_raw_pressure_t AUAV_V3_TestMavLink_rtZmavlink_raw_pressure_t = {
	0U,                                  /* time_boot_ms */
	0,                                   /* press_abs */
	0,                                   /* press_diff1 */
	0,                                   /* press_diff2 */
	0                                    /* temperature */
};                                    /* mavlink_raw_pressure_t ground */

const mavlink_sys_status_t AUAV_V3_TestMavLink_rtZmavlink_sys_status_t = {
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
};                                    /* mavlink_sys_status_t ground */

/* Exported block states */

mavlink_gps_raw_int_t mlGpsData;       /* '<Root>/mlGpsData' */
mavlink_sys_status_t mlSysStatus;      /* '<Root>/mlSysStatus' */
mavlink_raw_imu_t mlRawIMU;            /* '<Root>/mlRawIMU' */
mavlink_rc_channels_raw_t mlRC_Commands;/* '<Root>/mlRawRC' */
mavlink_servo_output_raw_t mlPwmCommands;/* '<Root>/mlRawServo' */
mavlink_vfr_hud_t mlVfr_hud;           /* '<Root>/mlVfr_hud' */
mavlink_scaled_pressure_t mlAirData;   /* '<Root>/mlAirData' */
mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
uint16_T MPU_T;


/* Declare UART1 Tx Circular Buffer Structure */
MCHP_UART1_TxStr MCHP_UART1_Tx;
/* Block signals (auto storage) */
BlockIO_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_B;
/* Block states (auto storage) */
D_Work_AUAV_V3_TestSensors_T AUAV_V3_TestSensors_DWork;


int mp_argc;
char **mp_argv;
int16_t angleOfAttack;
int16_t tiltError[3];
int16_t rotationRateError[3];
int16_t desiredRotationRateRadians[3];
int main(int argc, char** argv)
{
	// keep these values available for later
	mp_argc = argc;
	mp_argv = argv;

	//gps_init();     // this sets function pointers so i'm calling it early for now
	udb_init();
	//dcm_init();
	//init_config();  // this will need to be moved up in order to support runtime hardware options
	//init_waypoints();
	//init_servoPrepare();
	//init_states();
	//init_behavior();
	//init_serial();

	if (setjmp(buf))
	{
		// a processor exception occurred and we're resuming execution here 
		DPRINT("longjmp'd\r\n");
	}

	/* exported global states */
	mlGpsData = AUAV_V3_TestMavLink_rtZmavlink_gps_raw_int_t;
	mlSysStatus = AUAV_V3_TestMavLink_rtZmavlink_sys_status_t;
	mlAirData = AUAV_V3_TestMavLink_rtZmavlink_scaled_pressure_t;
	mlRawPressureData = AUAV_V3_TestMavLink_rtZmavlink_raw_pressure_t;

	//parameter_table_init();
	/* user code (Initialize function Body) */
	uartBufferInit();
	uartMavlinkBufferInit();
	InitParameterInterface();

	while (1)
	{

		udb_run();
	}
	return 0;
}
