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

#include "AUAV_V3_TestSensors.h"

#include "updateSensorMcuState.h"

extern float myPow(float x, float toTheN);

void updateRawADCData(int16_t* adcData) {
	float T, p, r, h,v;

    //mlRawPressureData.time_usec = mlAttitudeData.time_boot_ms; // set by updateTimestamp()
    mlRawPressureData.press_abs = (int16_t) adcData[0]; // Baro
    mlRawPressureData.press_diff1 = (int16_t) adcData[1]; // Pito
    mlRawPressureData.press_diff2 = (int16_t) adcData[2]; // Power
    mlRawPressureData.temperature = (int16_t) adcData[3]; // Temp
	//SLUGS2
	h = (float)((float)(mlGpsData.alt) / 1000.0f);
	T = (float)(15.04f - .00649f * h);
	p = (float)(101.29f *myPow((T + 273.1f) / 288.08f, 5.256f) * 1000.0f); //pascal
	r = (float)(p / (.2869f * (T + 273.1f))) /1000.0f;
	v = (float)((float)(mlGpsData.vel) / 100.0f);

	mlRawPressureData.press_abs = (p - 9444.4) / 27.1270; //convert to static pressure in Pascal / baroScale
	mlRawPressureData.press_diff1 = (1 / 2 * r * myPow(v, 2) + 1005.9) / 1.0514;//pitotScale
	mlRawPressureData.temperature = (T + 1605.3) / 1.5113;
	
	mlAirData.press_diff = (float)((r * myPow(v, 2)) / 2.0f / 100.0f);//hectopascal (1 hPa = 100 Pa)
	mlAirData.press_abs = (float)((p) / 100.0f);//hectopascal (1 hPa = 100 Pa)
	mlAirData.temperature = (int16_t)(T * 100.0f);//0.01 degrees celsius
}

void updateAirData(float* airData) {
	//SLUGS2
	float T, p, r, h,v;

	h = (float)((float)(mlGpsData.alt) / 1000.0f);
	T = (float)(15.04f - .00649f * h);
	p = (float)(101.29f *myPow((T + 273.1f) / 288.08f, 5.256f) * 1000.0f); //pascal
	r = (float)(p / (.2869f * (T + 273.1f))) /1000.0f;
	v = (float)((float)(mlGpsData.vel) / 100.0f);

	

    //mlAirData.time_boot_ms = mlAttitudeData.time_boot_ms; // set by updateTimestamp()
    mlAirData.press_diff = airData[0]; //dynamic
    mlAirData.press_abs = airData[1]; //static
    mlAirData.temperature = (int16_t) (airData[2]*10.0); // temp 0.01

	//SLUGS2
	mlAirData.press_diff = (float)((r * myPow(v, 2)) / 2.0f / 100.0f);//hectopascal (1 hPa = 100 Pa)
	mlAirData.press_abs = (float)((p) / 100.0f);//hectopascal (1 hPa = 100 Pa)
	mlAirData.temperature = (int16_t)(T * 100.0f);//0.01 degrees celsius
}



void updateLoadData(uint8_t load, uint16_t mvPower) {
    mlCpuLoadData.sensLoad = load;
    mlCpuLoadData.batVolt = mvPower;
}

void updateAttitude(float * attitudeData) {

    mlAttitudeData.roll = attitudeData[0];
    mlAttitudeData.pitch = attitudeData[1];
    mlAttitudeData.yaw = attitudeData[2];
    mlAttitudeData.rollspeed = attitudeData[3];
    mlAttitudeData.pitchspeed = attitudeData[4];
    mlAttitudeData.yawspeed = attitudeData[5];

}

void getGSLocation(float* altLatLon) {

    altLatLon[0] = mlGSLocationFloat.alt * 0.001;
    altLatLon[1] = mlGSLocationFloat.lat * 0.0000001;
    altLatLon[2] = mlGSLocationFloat.lon * 0.0000001;
}

/* Simulink sends 10's of milliseconds since boot. */
void updateTimeStamp(uint32_t timeSt) {

    // Todo fix units later
    mlAirData.time_boot_ms = timeSt;
    mlAttitudeData.time_boot_ms = timeSt;
    mlRawPressureData.time_usec = timeSt;

    /* Doesn't work:
    // Messages with time stamps in milliseconds
    uint32_t timeMs = timeSt*10;
    mlAirData.time_boot_ms = (uint64_t) timeMs;
    mlAttitudeData.time_boot_ms = timeMs;
    mlFilteredData.time_boot_ms = timeMs;
    mlLocalPositionData.time_boot_ms = timeMs;
    mlPilotConsoleData.time_boot_ms = timeMs;

    // Messages with time stamps in microseconds
    uint64_t timeUsec = timeSt*10000;
    mlGpsData.time_usec = timeUsec;
    mlRawImuData.time_usec = timeUsec;
    mlRawPressureData.time_usec = timeUsec;
    //mlPwmCommands.time_usec = timeSt; // this one is handled by control dsc
     * */
}

void updatePosition(float * posData) {

    mlLocalPositionData.x = posData[0];
    mlLocalPositionData.y = posData[1];
    mlLocalPositionData.z = posData[2];
    mlLocalPositionData.vx = posData[3];
    mlLocalPositionData.vy = posData[4];
    mlLocalPositionData.vz = posData[5];
}

void updatePilotConsole(uint16_t * pilData) {
    // dT dA dR dE dFail

    mlPilotConsoleData.chan1_raw = pilData[0];
    mlPilotConsoleData.chan2_raw = pilData[1];
    mlPilotConsoleData.chan3_raw = pilData[2];
    mlPilotConsoleData.chan4_raw = pilData[3];
    mlPilotConsoleData.chan5_raw = pilData[4];
}

void updateDiagnosticFl(float* diagFl) {
    mlDiagnosticData.diagFl1 = diagFl[0];
    mlDiagnosticData.diagFl2 = diagFl[1];
    mlDiagnosticData.diagFl3 = diagFl[2];
}

void updateDiagnosticSh(int16_t* diagSh) {
    mlDiagnosticData.diagSh1 = diagSh[0];
    mlDiagnosticData.diagSh2 = diagSh[1];
    mlDiagnosticData.diagSh3 = diagSh[2];
}

void updateBias(float * biasData) {
    mlSensorBiasData.axBias = biasData[0];
    mlSensorBiasData.ayBias = biasData[1];
    mlSensorBiasData.azBias = biasData[2];
    mlSensorBiasData.gxBias = biasData[3];
    mlSensorBiasData.gyBias = biasData[4];
    mlSensorBiasData.gzBias = biasData[5];
}

void updateSensorData(float* sens) {
    mlFilteredData.xacc = (int16_t) (sens[0] * MPS_TO_MG);
    mlFilteredData.yacc = (int16_t) (sens[1] * MPS_TO_MG);
    mlFilteredData.zacc = (int16_t) (sens[2] * MPS_TO_MG);
    mlFilteredData.xmag = (int16_t) (sens[3]); // assuming milligaus
    mlFilteredData.ymag = (int16_t) (sens[4]);
    mlFilteredData.zmag = (int16_t) (sens[5]);
    mlFilteredData.xgyro = (int16_t) (sens[6]*1000.0);
    mlFilteredData.ygyro = (int16_t) (sens[7]*1000.0);
    mlFilteredData.zgyro = (int16_t) (sens[8]*1000.0);
}

uint8_t isFixValid (void){
    return (mlGpsData.fix_type == GPS_FIX_3D);
}

void updateSensorDiag (float* diag){
    mlSensorDiag.float1 = diag[0];
    mlSensorDiag.float2 = diag[1];
    mlSensorDiag.int1 = (int16_t) diag[2];
    mlSensorDiag.char1 = (int8_t) diag[3];
}

