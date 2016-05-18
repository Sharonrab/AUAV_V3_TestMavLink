
#include "updateControlMcuState.h"
#include "apUtils.h"

void updatePWM(unsigned short * PWMData){
	// dT dA dR dE Failsafe
	mlPwmCommands.servo1_raw = PWMData[0];
	mlPwmCommands.servo2_raw = PWMData[1];
	mlPwmCommands.servo3_raw = PWMData[2];
	mlPwmCommands.servo4_raw = PWMData[3];
        mlPwmCommands.time_usec = mlRawImuData.time_usec; // Is this okay?
}

void updatePWMTrim(unsigned short PWMData, unsigned char channel ){
	switch (channel){
		case 0:
			mlPwmCommands.servo5_raw = PWMData;
		break; 
		case 1:
			mlPwmCommands.servo6_raw = PWMData;
		break; 
		case 2:
			mlPwmCommands.servo7_raw = PWMData;
		break; 
		case 3:
			mlPwmCommands.servo8_raw = PWMData;
		break; 								
	}
}

void updateLoad (uint8_t mcuLoad){
	mlCpuLoadData.ctrlLoad =  mcuLoad;
	mlSystemStatus.load = mcuLoad*10;
}

void updateEuler(float* newEuler){
	mlAttitudeRotated.roll	= newEuler[0];
	mlAttitudeRotated.pitch = newEuler[1];
	mlAttitudeRotated.yaw   = newEuler[2];
	//mlAttitudeRotated.usec  = mlAttitudeData.usec;
        mlAttitudeRotated.time_boot_ms = mlAttitudeData.time_boot_ms;
		
	// mlAttitudeRotated.roll	= mlAttitudeData.roll;
	// mlAttitudeRotated.pitch = mlAttitudeData.pitch;
	// mlAttitudeRotated.yaw   = mlAttitudeData.yaw;
	// mlAttitudeRotated.usec  = mlAttitudeData.usec;
}


void updatePQR(float* newPQR){
	mlAttitudeRotated.rollspeed 	= newPQR[0];
	mlAttitudeRotated.pitchspeed 	= newPQR[1];
	mlAttitudeRotated.yawspeed 		= newPQR[2];
	
	// mlAttitudeRotated.rollspeed 	= mlAttitudeData.rollspeed;
	// mlAttitudeRotated.pitchspeed 	= mlAttitudeData.pitchspeed;
	// mlAttitudeRotated.yawspeed 		= mlAttitudeData.yawspeed;
}

// void updatePilotCommands (unsigned short*  pilCom){
// 	pilCom[0] = mlPilotConsoleData.dt;
// 	pilCom[1] = mlPilotConsoleData.dla;
// 	pilCom[2] = mlPilotConsoleData.dra;
// 	pilCom[3] = mlPilotConsoleData.dr;
// 	pilCom[4] = mlPilotConsoleData.de;
// }

unsigned char getHilOnOff (void){
    return hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED);
}

// void getGSLocation(float* altLatLon) {

    // altLatLon[0] = mlWpValues.alt[MAX_NUM_WPS-1]  * 0.001;
    // altLatLon[1] = mlWpValues.lat[MAX_NUM_WPS-1]  * 0.0000001;
    // altLatLon[2] = mlWpValues.lon[MAX_NUM_WPS-1]  * 0.0000001;
// }

uint8_t getLightsOnOff (void){
    return mlLights.state;
}

uint8_t getLightsDayNight (void){
    return mlLights.type;
}
    
// 00 - I
// 01 - Sr Pr
// 10 - Sr Pp
// 11 - RPM

void updateVISensor (uint16_t* data){
    mlVISensor.voltage = data[0];
    mlVISensor.reading2 = data[1];
}

void updatePTZ (int16_t* ptz){
    mlPtzStatus.pan = ptz[0];
    mlPtzStatus.tilt   = ptz[1];
    mlPtzStatus.zoom = (uint8_t) ptz[2];
}