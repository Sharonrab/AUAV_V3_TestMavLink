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

#ifndef _UPDATESENSORMCUSTATE_H_
#define _UPDATESENSORMCUSTATE_H_

#include "apDefinitions.h"
#include "mavlinkSensorMcu.h"

#define MPS_TO_MG 	101.8849



void updateRawADCData (int16_t * adcData);
void updateAirData (float* airData);
void updateLoadData (uint8_t load, uint16_t mvPower);
void updateAttitude (float * attitudeData);
void updatePosition (float * posData);
void updateTimeStamp (uint32_t timeSt);
void updatePilotConsole (uint16_t* pilData);
void updateDiagnosticFl (float* diagFl);
void updateDiagnosticSh (int16_t* diagSh);
void updateBias (float * biasData); 
void updateSensorData (float* sens);

void getGSLocation (float* altLatLon);
void updateSensorDiag (float* diag);

uint8_t isFixValid (void);

#endif /* _UPDATESENSORMCUSTATE_H_ */
