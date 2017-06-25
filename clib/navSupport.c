/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */

#include "navSupport.h"
#include "apUtils.h"

/*

MAVLink's supported modes are:

enum MAV_NAV
        { 
    MAV_NAV_GROUNDED = 0,
    MAV_NAV_LIFTOFF,
    MAV_NAV_HOLD,
    MAV_NAV_WAYPOINT,
    MAV_NAV_VECTOR,
    MAV_NAV_RETURNING,
    MAV_NAV_LANDING,
    MAV_NAV_LOST,
    MAV_NAV_LOITER
};

These are only relevant when SLUGS mode is in MAV_MODE_GUIDED.
SLUGS uses the nav modes as follows:

MAV_NAV_VECTOR: Autonomous mode but using mid-level commands, the end user must set a command for airspeed, altitude and turnrate

MAV_NAV_WAYPOINT: Fully autonomous mode using waypoint navigation. The end user configures the waypoints and the commanded airspeed.

MAV_NAV_HOLD: Passthrough mode. The Pilot commands are passed through the autopilot and sent as if it were autopilot generated

MAV_NAV_LOITER: Selective passtrough. Some pilot commands (selectively chosen form the ground station) are passed and others are used from the autopilot
                                                        as if in GUIDED mode.

 */

void getMidLevelCommands(float* commands) {
    commands[0] = mlMidLevelCommands.uCommand;
    commands[1] = mlMidLevelCommands.hCommand;
    commands[2] = mlMidLevelCommands.rCommand;
}

unsigned char isApManual(uint16_t failsafe) {
    // Disable and toggle manual/guided mode depending on failsafe
    if (failsafe < (uint16_t)2000U) {
		mlHeartbeatLocal.custom_mode = SLUGS_MODE_MID_LEVEL;// SLUGS_MODE_WAYPOINT;
        mlHeartbeatLocal.base_mode &= ~MAV_MODE_FLAG_AUTO_ENABLED;
		//mlHeartbeatLocal.base_mode &= ~MAV_MODE_FLAG_STABILIZE_ENABLED;

        mlHeartbeatLocal.base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
	else if (failsafe < (uint16_t)3000U) {
		mlHeartbeatLocal.custom_mode = SLUGS_MODE_SELECTIVE_PASSTHROUGH;// SLUGS_MODE_MID_LEVEL;// SLUGS_MODE_SELECTIVE_PASSTHROUGH;
		mlHeartbeatLocal.base_mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		//mlHeartbeatLocal.base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;

		mlHeartbeatLocal.base_mode |= MAV_MODE_FLAG_AUTO_ENABLED;

		
	}
    else {
		mlHeartbeatLocal.custom_mode = SLUGS_MODE_WAYPOINT;
        mlHeartbeatLocal.base_mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		//mlHeartbeatLocal.base_mode &= ~MAV_MODE_FLAG_STABILIZE_ENABLED;

        mlHeartbeatLocal.base_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
    }

    return hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
}

float getParamIdx(unsigned char idx) {

    return mlParamInterface.param[idx];
}

void getRangeOfParams(uint8_t startIdx, uint8_t endIdx, float* parameters) {
    uint8_t i;

    for (i = startIdx; i <= endIdx; i++) {
        parameters[i - startIdx] = mlParamInterface.param[i];
    }
}

float getDynamic(void) {
    // hectopascals to pascals
    return mlAirData.press_diff;
}

void getAttitude(float* attitude) {

    // Return the values to the control algorithm
    attitude[0] = mlAttitudeData.roll;
    attitude[1] = mlAttitudeData.pitch;
    attitude[2] = mlAttitudeData.yaw;
    attitude[3] = mlAttitudeData.rollspeed;
    attitude[4] = mlAttitudeData.pitchspeed;
    attitude[5] = mlAttitudeData.yawspeed;

}

void getXYZ(float* xyz) {
    xyz[0] = mlLocalPositionData.x;
    xyz[1] = mlLocalPositionData.y;
    xyz[2] = mlLocalPositionData.z;
}

void getVned(float* xyz) {
    xyz[0] = mlLocalPositionData.vx;
    xyz[1] = mlLocalPositionData.vy;
    xyz[2] = mlLocalPositionData.vz;
}

uint8_t getMaxWp(void) {
    return mlWpValues.wpCount;
}

unsigned char isWpFly(void) {
    return ((mlHeartbeatLocal.custom_mode == SLUGS_MODE_WAYPOINT)
            || (mlHeartbeatLocal.custom_mode == SLUGS_MODE_ISR)
            || (mlHeartbeatLocal.custom_mode == SLUGS_MODE_LINE_PATROL));
    /*
    return ((mlSystemStatus.nav_mode == MAV_NAV_WAYPOINT) ||
                (mlSystemStatus.nav_mode == MAV_NAV_ISR) ||
                (mlSystemStatus.nav_mode == MAV_NAV_LINE_PATROL)
              );
     * */
}

unsigned char isPassthrough(void) {
    return ((mlHeartbeatLocal.custom_mode == SLUGS_MODE_PASSTHROUGH)
            || (mlHeartbeatLocal.custom_mode == SLUGS_MODE_SELECTIVE_PASSTHROUGH)
            || (mlHeartbeatLocal.custom_mode == SLUGS_MODE_MID_LEVEL));
    /*
    return ( (mlSystemStatus.nav_mode == MAV_NAV_PASSTHROUGH) || 
                 (mlSystemStatus.nav_mode == MAV_NAV_SEL_PT) || 
                 (mlSystemStatus.nav_mode == MAV_NAV_MID_LEVEL));
     * */
}

void setDiagnosticFloat(float * flValues) {
    mlDiagnosticData.diagFl1 = flValues[0];
    mlDiagnosticData.diagFl2 = flValues[1];
    mlDiagnosticData.diagFl3 = flValues[2];
}

void setDiagnosticShort(int16_t* shValues) {
    mlDiagnosticData.diagSh1 = shValues[0];
    mlDiagnosticData.diagSh2 = shValues[1];
    mlDiagnosticData.diagSh3 = shValues[2];
}

void getWP(unsigned char idx, float* WPpos) {
    WPpos[0] = mlWpValues.lat[idx - 1];
    WPpos[1] = mlWpValues.lon[idx - 1];
    WPpos[2] = mlWpValues.alt[idx - 1];
}

void setLogFloat1(float * flValues) {
    mlDataLog.fl_1 = flValues[0];
    mlDataLog.fl_2 = flValues[1];
    mlDataLog.fl_3 = flValues[2];
}

void setLogFloat2(float * flValues) {
    mlDataLog.fl_4 = flValues[0];
    mlDataLog.fl_5 = flValues[1];
    mlDataLog.fl_6 = flValues[2];
}

unsigned char getApControlType(void) {
    return mlHeartbeatLocal.custom_mode;
}

unsigned char getPassValues(uint8_t* pasVals) {
    pasVals[0] = (uint8_t) (mlPassthrough.bitfieldPt & 128);
    pasVals[1] = (uint8_t) (mlPassthrough.bitfieldPt & 64);
    pasVals[2] = (uint8_t) (mlPassthrough.bitfieldPt & 16);
    pasVals[3] = (uint8_t) (mlPassthrough.bitfieldPt & 8);
    return 1;
}

void setCurrentCommands(float airSpeed) {
    mlMidLevelCommands.uCommand = airSpeed;
    mlMidLevelCommands.hCommand = mlLocalPositionData.z;
    mlMidLevelCommands.rCommand = 0.0;

}

// NOTE should this be double buffered or converted each time? (dagoodma)
void getCurrentGPSPos(float* latLonAlt) {
    latLonAlt[0] = INT32_1E7_TO_FLOAT(mlGpsData.lat);
    latLonAlt[1] = INT32_1E7_TO_FLOAT(mlGpsData.lon);
    latLonAlt[2] = INT32_1E3_TO_FLOAT(mlGpsData.alt);
}

void setNavLong(float* values) {
    mlNavigation.u_m = values[0];
    mlNavigation.theta_c = values[1];
    mlNavigation.h_c = (uint16_t)( floorf(values[2]*10.0));
}

void setNavLat(float* values) {
    mlNavigation.psiDot_c = values[0];
    mlNavigation.phi_c = values[1];
    mlNavigation.ay_body = values[2];
}

void setNavNav(float* values) {
    mlNavigation.totalDist = values[0];
    mlNavigation.dist2Go = values[1];
    mlNavigation.fromWP = (uint8_t) values[2];

    if (mlNavigation.toWP != (uint8_t)values[3]) mlPending.wpSendCurrent = TRUE;
    mlNavigation.toWP = (uint8_t) values[3];
}

void getAccels(float * accels) {
    accels[0] = (float) (mlFilteredData.xacc * MG_TO_MPS);
    accels[1] = (float) (mlFilteredData.yacc * MG_TO_MPS);
    accels[2] = (float) (mlFilteredData.zacc * MG_TO_MPS);
}

void getAccBias(float * bias) {
    bias[0] = mlSensorBiasData.axBias;
    bias[1] = mlSensorBiasData.ayBias;
    bias[2] = mlSensorBiasData.azBias;
}

void getRTB(uint8_t* rtb) {
    rtb[0] = (mlRTB.rtb || (mlPending.heartbeatAge > HEARTBEAT_LIMIT));
    rtb[1] = mlRTB.track_mobile; 

    // No RTB in HIL simulation
    if (hasMode(mlHeartbeatLocal.base_mode,MAV_MODE_FLAG_HIL_ENABLED)) {
        return;
    }
    
    if (rtb[0]) {
        if (mlHeartbeatLocal.custom_mode != SLUGS_MODE_RETURNING) {
            lastNavigationMode = mlHeartbeatLocal.custom_mode;

            mlPending.statustext++;

            mlStatustext.severity = MAV_SEVERITY_INFO;
            strncpy(mlStatustext.text, "Lost groundstation heartbeat. Returning to base.", 49);
        }

        mlHeartbeatLocal.custom_mode = SLUGS_MODE_RETURNING; //MAV_STATE_RETURNING;
    } else {
        //if (mlHeartbeatLocal.custom_mode == SLUGS_MODE_RETURNING)
        //    mlHeartbeatLocal.custom_mode = lastNavigationMode;
    }
}

void getMobileLocation (float* loc){
    loc[0] = mlMobileLocation.latitude;
    loc[1] = mlMobileLocation.longitude;
}

uint8_t getNavMode (void){
    return mlHeartbeatLocal.custom_mode;
}

void getISRLocation (float* loc){
    loc[0] =  mlISR.latitude;
    loc[1] =  mlISR.longitude;
    loc[2] =  mlISR.height;
}


uint8_t getISRCameraOption1(void){
    return mlISR.option1;
}
unsigned short meanFilter5(unsigned short * values) {
    quickSort(values, 7);
    return values[3];
}

uint8_t justEnabled(uint8_t enableValue, uint8_t index){
    /*
     * The indexes used are as follows:
     * 
     * 0    Just switched to WP
     * 1    DidReachIP
     * 2    Trim Conditions
     */
    
    static uint8_t enableValues [NUM_JUST_ENAB] ;
    static uint8_t firstRun = 1;
    uint8_t retVal;
      
    if (firstRun){
        memset(enableValues, 0, NUM_JUST_ENAB);
        firstRun = 0;
    }
    
    retVal = (enableValues[index] == 0 && enableValue == 1);
    
    enableValues[index] = enableValue;
    
    return retVal;
}

//  quickSort
//
//  This public-domain C implementation by Darel Rex Finley.
//
//  * Returns YES if sort was successful, or NO if the nested
//    pivots went too deep, in which case your array will have
//    been re-ordered, but probably not sorted correctly.
//
//  * This function assumes it is called with valid parameters.
//
//  * Example calls:
//    quickSort(&myArray[0],5); // sorts elements 0, 1, 2, 3, and 4
//    quickSort(&myArray[3],5); // sorts elements 3, 4, 5, 6, and 7
//
// 	source: http://alienryderflex.com/quicksort/

unsigned char quickSort(unsigned short *arr, char elements) {

#define  MAX_LEVELS  8

    short i = 0, L, R;
    unsigned short piv, beg[MAX_LEVELS], end[MAX_LEVELS];

    beg[0] = 0;
    end[0] = elements;
    while (i >= 0) {
        L = beg[i];
        R = end[i] - 1;
        if (L < R) {
            piv = arr[L];
            if (i == MAX_LEVELS - 1) return 0;
            while (L < R) {
                while (arr[R] >= piv && L < R) {
                    R--;
                }
                if (L < R) {
                    arr[L++] = arr[R];
                }
                while (arr[L] <= piv && L < R) {
                    L++;
                }
                if (L < R) {
                    arr[R--] = arr[L];
                }
            }
            arr[L] = piv;
            beg[i + 1] = L + 1;
            end[i + 1] = end[i];
            end[i++] = L;
        } else {
            i--;
        }
    }
    return 1;
}
