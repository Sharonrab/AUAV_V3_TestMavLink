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

#include "apUtils.h"
	//#define Nop()    {__asm__ volatile ("nop");}
#define Nop() {;}
/*
// =====================================
//			Trig and Math Functions
// =====================================
*/

float myAtan2(float num, float denom){
     #ifdef _IN_PC_
        return atan2(num,denom);
     #else
		return atan2f(num,denom);
     #endif
}


float myPow(float x, float toTheN){
     #ifdef _IN_PC_
        return pow(x,toTheN);
     #else
		return powf(x,toTheN);
     #endif
}

float mySqrt(float x){
     #ifdef _IN_PC_
        return sqrt(x);
     #else
		return sqrtf(x);
     #endif
}

float myAbs(float x){
     #ifdef _IN_PC_
        return fabs(x);
     #else
		return fabsf(x);
     #endif
}


float myAtan(float x){
     #ifdef _IN_PC_
        return atan(x);
     #else
		return atanf(x);
     #endif
}

float myAsin(float x){
     #ifdef _IN_PC_
        return asin(x);
     #else
		return asinf(x);
     #endif
}

float mySin(float x){
     #ifdef _IN_PC_
        return sin(x);
     #else
		return sinf(x);
     #endif
}

float myCos(float x){
     #ifdef _IN_PC_
        return cos(x);
     #else
		return cosf(x);
     #endif
}

float myTan(float x){
     #ifdef _IN_PC_
        return tan(x);
     #else
		return tanf(x);
     #endif
}

float myAcos(float x){
     #ifdef _IN_PC_
        return acos(x);
     #else
		return acosf(x);
     #endif
	
}

float myExp(float x){
     #ifdef _IN_PC_
        return exp(x);
     #else
		return expf(x);
     #endif
}

//uint8_t isFinite(float s) {
//  // By IEEE 754 rule, 2*Inf equals Inf
//  return ((s == s) && ((s == 0) || (s != 2*s)));
//}

void hugeDelay(void) {
    int i, j;
    for (i = 0; i < 750; i += 1) {
        for (j = 0; j < 32700; j += 1) {
            Nop();
        }
    }
}

void smallDelay(void) {
    int i;
    for (i = 0; i < 5000; i += 1) {
        Nop();
    }
}


void tinyDelay(void) {
    int i;
    for (i = 0; i < 50; i += 1) {
        Nop();
    }
}


	LOCAL_BOOL hasMode(uint8_t field, uint8_t flag) {
		return (field & flag) > 0;
	}

#ifndef __cplusplus

	// ================================
	//    Debug Functions
	// ================================
//	void printToUart2(const char *fmt, ...) {
//		va_list ap;
//		char buf[300];
//
//		va_start(ap, fmt);
//		vsprintf(buf, fmt, ap);
//		va_end(ap);
//#ifndef WIN
//
//		//putsUART2((unsigned int*)buf);
//		//while (BusyUART2());
//
//#endif
//
//	}

#endif