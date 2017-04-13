


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
#include "SLUGS2.h"
#include "SLUGS2_private.h"

#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif


//#if (SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) 
//void parameter_table_init(void);
//#endif

/* Microchip Global Variables */
//volatile uint16_T MCHP_MCULoadResult[1] ;
//volatile uint16_T MCHP_MCULoadPreviousTimerValue[1] ;
//volatile uint16_T mcuFlagRecursion  = 0;

	/*union
	{
		struct {
			unsigned int task0 : 1;
			unsigned int task1 : 1;
			unsigned int task2 : 1;
			unsigned int task3 : 1;
			unsigned int task4 : 1;
			unsigned int task5 : 1;
			unsigned int task6 : 1;
			unsigned int task7 : 1;
			unsigned int task8 : 1;
			unsigned int task9 : 1;
			unsigned int task10 : 1;
		} b;

		unsigned int val;
	
} MCHP_MCU_Overload;*/

static jmp_buf buf;




/* Declare UART1 Tx Circular Buffer Structure */
MCHP_UART1_TxStr MCHP_UART1_Tx;

float unitTest;
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
	unitTest = myTan(0.1f);
	//gps_init();     // this sets function pointers so i'm calling it early for now
	udb_init();
	//dcm_init();
	//init_config();  // this will need to be moved up in order to support runtime hardware options
	//init_waypoints();
	init_servoPrepare();
	//init_states();
	//init_behavior();
	//init_serial();

	if (setjmp(buf))
	{
		// a processor exception occurred and we're resuming execution here 
		DPRINT("longjmp'd\r\n");
	}

	

	//parameter_table_init();
	/* user code (Initialize function Body) */
	//uartBufferInit();
	//uartMavlinkBufferInit();
	//InitParameterInterface();

	while (1)
	{

		udb_run();
	}
	return 0;
}
