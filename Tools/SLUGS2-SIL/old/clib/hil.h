#ifndef _HIL_H_
#define _HIL_H_

#include "mavlinkSensorMcu.h"
#include "apDefinitions.h"
#include "circBuffer.h"
//#include "ipcScheduler.h"

void hilRead (unsigned char* hilChunk);
void protDecodeHil (uint8_t* dataIn);

void hil_getRawRead (short * rawData);
void hil_getVned (float* vned);
void hil_getXYZ (float* xyz);
void hil_getEuler (float* euler);
void hil_getRates (float* pqr);
unsigned int hil_getTs (void);

extern mavlink_raw_imu_t mlRawImuData; // 	..	==
#endif /* _HIL_H_ */
