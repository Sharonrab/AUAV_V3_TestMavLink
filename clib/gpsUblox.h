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

#ifndef _GPS_UBLOX_H_
#define _GPS_UBLOX_H_


#ifdef __cplusplus
       extern "C"{
#endif

#include <stdlib.h>
#include <string.h>
    // GPS Circular Buffers
    // ====================
#define MSIZE			180

#define TOKEN_SIZE	15

// GPS Checksum Messages
// =====================
#define GGACS			86
#define RMCCS			75
    // GPS Header IDs
    // ==============
#define GGAID			1
#define RMCID			2
#define VTGID			3
#define UNKID			254

// GPS Fix Types
enum GPS_FIX {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3
};

           // Scaling functions for GPS data message
#define INT32_1E7_TO_FLOAT(x)  ((float)x * 0.0000001f)
#define FLOAT_TO_INT32_1E7(x) ((int32_t)(x * 10000000.0f))

// Used for eph, epv, vel, and cog
#define UINT16_1E2_TO_FLOAT(x)  ((float)x * 0.01f)
#define FLOAT_TO_UINT16_1E2(x) ((uint16_t)(x * 100.0f))
//
#define INT32_1E3_TO_FLOAT(x)  ((float)x * 0.001f)
#define FLOAT_TO_INT32_1E3(x) ((int32_t)(x * 1000.0f))

#define KTS2MPS 		0.514444444

#define DOLLAR	36
#define STAR	42
#define CR	13
#define LF      10
#define AT	64

char hex2char (char halfhex);
unsigned char gpsUbloxSeparate (unsigned char* outStream);
void gpsUbloxParse (void);
void getGpsUbloxMainData (float* data);
float degMinToDeg (unsigned char degrees, float minutes);
char gpSmbl (char symbl);
void parseRMC (unsigned char* stream);
void parseGGA (unsigned char* stream);
unsigned char getChecksum (unsigned char* sentence, unsigned char size);


#ifdef __cplusplus
       }
#endif
       
#endif /* _GPS_UBLOX_H_ */
