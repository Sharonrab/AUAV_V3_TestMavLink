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

#ifndef _ADISCUBE16405_H_
#define _ADISCUBE16405_H_
#ifdef __cplusplus
    extern "C"{
#endif
   
#ifndef WIN
//#include <p33fxxxx.h>
#include <p33EP512MU810.h>
#include <spi.h>
#endif    

#include "apDefinitions.h"
#include "apUtils.h"
#include "mavlinkSensorMcu.h"


#if (USE_CUBE_16405 == 1)


#define selectCube()		LATGbits.LATG9 = 0
#define deselectCube()		LATGbits.LATG9 = 1
#define cubeDataReady()		PORTCbits.RC3

#define BITMASK_14		(unsigned short)0x3FFF
#define BITMASK_12		(unsigned short)0x0FFF

#define BITTEST_14		(unsigned short)0x2000
#define BITTEST_12		(unsigned short)0x0800

#define BITEXTEND_14	(unsigned short)0xC000
#define BITEXTEND_12	(unsigned short)0xF000


// Max readings is MAX_CUBE_READ + 1
#define MAX_CUBE_READ		3

// Register Addresses
// ==================
// Note that FOR READING register addresses are in the MSB, assigning dont cares
// to the LSB
#define R_ADISPWR   	(unsigned short)0x0200		// ADIS Power Supply 
#define R_GYROX     	(unsigned short)0x0400		// X-CHANNEL GYRO 
#define R_GYROY     	(unsigned short)0x0600		// Y-CHANNEL GYRO 
#define R_GYROZ     	(unsigned short)0x0800		// Z-CHANNEL GYRO 
#define R_ACCELX    	(unsigned short)0X0A00		// X-CHANNEL ACCEL
#define R_ACCELY    	(unsigned short)0x0C00		// Y-CHANNEL ACCEL 
#define R_ACCELZ    	(unsigned short)0x0E00		// Z-CHANNEL ACCEL
#define R_MAGNX    		(unsigned short)0x1000		// X-CHANNEL MAGNE
#define R_MAGNY    		(unsigned short)0x1200		// Y-CHANNEL MAGNE
#define R_MAGNZ    		(unsigned short)0x1400		// Z-CHANNEL MAGNE
#define R_TEMP				(unsigned short)0x1600		// SENSOR TEMP
#define R_STATUS    	(unsigned short)0x3C00		// STATUS REGISTER

// Note that FOR WRITING register addresses are in the MSB and the actual data
// to write is in the LSB

// Startup Config Values
// =====================
/*
Calibration
We will use the precission bias null calibration. That requires the unit to be static for 30 seconds.
Address = 0x3E			//GLOB_CMD
Value   = 0x10
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 1][1 1 1 0][0 0 0 1][0 0 0 0] X 
=> 0xBE10

we will also use the acceleration bias compensation and use the
DIO pin in the cube to let us know when data is ready

Address = 0x34		//MSC_CTRL
Value   = 0x86
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 1][0 1 0 0][1 0 0 0] [0 1 1 0]
=> 0xB486

*/

#define W_COMMAND_FULLCAL	(unsigned short)0xBE10		//Start full calibration (GLOB_CMD)
#define W_COMMAND_QUICKCAL	(unsigned short)0xBE01		//Start Quick calibration (GLOB_CMD)
#define W_MSC_CTRL		(unsigned short)0xB486		// Set DIO1 as data ready pin (MSC_CTRL)

/*
Definition of the Internal Sample Rate

Ts = Tb(Ns+1) , where
	 Ts is Sample Time (1/Fs)
	 Tb is time base, it is either 0.61035e-3 S or 18.921e-3 S
	 Ns is the multiplier, which is configured in register SMPL_PRD
	 
So desired Ts = 0.00125 (800Hz) then
	 0.00125 = 0.61035e-3 * (3.096 + 1)
	 => Choose multiplier as 1 => Ts = 1.22e-3 S => Fs= 819.2 Hz

So the register configuration is as follows:
Address = 0x36
Value   = 0x01
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 1][0 1 1 0][0 0 0 0][0 0 0 1]
=> 0xB601
*/
#define W_SMPL_PRD  	(unsigned short)0xB601		// INTERNAL SAMPLE RATE (SMPL_PRD)

/*
Digital Filtering and Dynamic Range

The dynamic range will be set to 300 deg/s and the taps at the minimum 4.
Since this configuration spans two registers it will be done in two steps,
by the data sheet recomendation, sensitivity first, then taps.

Address	= 0x39
Value	= 0x04
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 1][1 0 0 1][0 0 0 0][0 1 0 0]
=> 0xB904

The taps will be set at the minimum of 4  (therefore the value is 2 since
they are powers of 2)
Address	= 0x38
Value	= 0x04
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 1][1 0 0 0][0 0 0 0][0 1 0 0]   // [0 0 0 0][0 1 0 0][0 0 0 0][0 1 0 0]
=> 0xB804 //0x0404

*/
#define W_SENS_AVG_H  	(unsigned short)0xB904	//	// DYNAMIC RANGE
#define W_SENS_AVG_L  	(unsigned short)0xB804		// DIGITAL FILTERING

#define W_MSC_CTRL_SELFTEST (unsigned short)0xB504
/*
Sleeping

No sleeping on the sensor
*/

#define W_SLP_CNT		(unsigned short)0xBA00		// DONT SLEEP

/*
Accelerometer zero offset
This is used to zero out any offset in the accelerometer reading
Values are in twos complement

X accel -> currently no offset
Address	= 0x20
Value	= 0xFB (-5)
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 1 0][1 0 1 0][1 1 1 1][1 0 1 1]
=> 0xA0FB

Y accel -> showing a -43 count offset
Address	= 0x22
Value	= 0x21 (+33)
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 0 1][1 1 0 0][0 0 1 0][0 0 0 1]
=> 0xA221

Z accel -> currently a +4 offset ( -396.5 counts = 1000 mg)
Address	= 0x24
Value	= 0xF6 (-10)
bits [6:7] of Address must be [1 0] since this will be a write
=> [1 0 0 1][1 1 1 0][1 1 1 1][0 1 1 0]
=> 0xA4F5

*/
#define W_XACC_OFFSET_LO	(unsigned short)0xA0EC // -20
#define W_XACC_OFFSET_HI	(unsigned short)0xA11F
#define W_YACC_OFFSET_LO	(unsigned short)0xA210 // 16 
#define W_YACC_OFFSET_HI	(unsigned short)0xA300
#define W_ZACC_OFFSET_LO	(unsigned short)0xA4FB // -5
#define W_ZACC_OFFSET_HI	(unsigned short)0xA51F

typedef struct tCubeBuffer {
  int16_t  ax[4];
  int16_t  ay[4];
  int16_t  az[4];
  int16_t  gx[4];
  int16_t  gy[4];
  int16_t  gz[4];
  int16_t  mx[4];
  int16_t  my[4];
  int16_t  mz[4];
  uint8_t sampleCount;
}tCubeBuffer;

void cubeInit (void);
unsigned short write2Cube (unsigned short data2Send);
void getCubeData (short * cubeData);
void updateCubeData (void);
int16_t averageData (int16_t* theData, uint8_t count);

short convert12BitToShort (short wordData);
short convert14BitToShort (short wordData);
	
unsigned char isCube16405 (void);

#endif // USE_CUBE_16405

#ifdef __cplusplus
    }
#endif
#endif /* _ADISCUBE16405_H_ */
