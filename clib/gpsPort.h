#ifndef _GPS_PORT_H_
#define _GPS_PORT_H_


#ifdef __cplusplus
       extern "C"{
#endif
           
#define DOUBLE_BAUD_RATE  1
#define USE_SBAS 0
       	
#include "circBuffer.h"
#include "apUtils.h"
#include "mavlinkSensorMcu.h"
#include "apDefinitions.h"

#include <p33fxxxx.h>
#include <uart.h>

extern CBRef uartBuffer;
void uartBufferInit (void);
void gpsSentenceConfig (void);
void gpsFreqConfig (void);
void getGPSRawData(unsigned char* gpsBuffer);
uint8_t isGPSNovatel (void);


#ifdef __cplusplus
       }
#endif
       
#endif /* _GPS_PORT_H_ */
