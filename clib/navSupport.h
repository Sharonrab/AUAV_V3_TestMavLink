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

#ifndef _NAVSUPPORT_H_
#define _NAVSUPPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "apDefinitions.h"
#include "mavlinkControlMcu.h"
#include "math.h"

#define MG_TO_MPS	0.009815
#define NUM_JUST_ENAB          3

    void getMidLevelCommands(float* commands);
    unsigned char isApManual(uint16_t failsafe);
    float getParamIdx(uint8_t idx);
    void getRangeOfParams(uint8_t startIdx, uint8_t endIdx, float* parameters);
    float getDynamic(void);
    void getAttitude(float* attitude);
    void getXYZ(float* xyz);
    uint8_t getMaxWp(void);
    unsigned char isWpFly(void);
    unsigned char isPassthrough(void);
    void setDiagnosticFloat(float * flValues);
    void setDiagnosticShort(int16_t* shValues);
    void getWP(unsigned char idx, float* WPpos);
    unsigned char getApControlType(void);
    unsigned char getPassValues(unsigned char * pasVals);
    void setCurrentCommands(float airSpeed);
    unsigned char quickSort(unsigned short *arr, char elements);
    unsigned short meanFilter5(unsigned short * values);
    void setNavLong(float* values);
    void setNavLat(float* values);
    void setNavNav(float* values);
    void setLogFloat1(float * flValues);
    void setLogFloat2(float * flValues);
    void getVned(float* xyz);
    void getCurrentGPSPos(float* latLonAlt);
    void getRTB(uint8_t* rtb);
    void getMobileLocation(float* loc);
    uint8_t getNavMode (void);
    void getISRLocation (float* loc);
    uint8_t getISRCameraOption1(void);
    uint8_t justEnabled(uint8_t enableValue, uint8_t index);


#ifdef __cplusplus
}
#endif

#endif /* _NAVSUPPORT_H_ */
