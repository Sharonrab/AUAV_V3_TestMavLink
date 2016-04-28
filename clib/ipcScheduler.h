// ==============================================================
// ipcScheduler.h
// This is code implements a fully DMA driven UART writer to
// be used in the UCSC Autopilot project. It makes use of the 
// circular buffer data structure circBuffer.c. It has been 
// written to be implemented in Simulink. It configures UART 2
// at a predefined baud rate, then initializes a circular buffer,
// configures the DMA and starts the service. 
// The main function logData writes data to UART2 in the predefined
// comm protocol and returns data in the comm protocol to be sent
// via SPI to the second MCU. 
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 26 2008 @ 21:15
// =========================================================

#ifndef _IPCSCHEDULER_H_
#define _IPCSCHEDULER_H_

#include "mavlink.h"
#include "apDefinitions.h"
#include "circBuffer.h"
#include "apUtils.h"
#include "mavlinkSensorMcu.h"
#include <string.h>
#include <p33fxxxx.h>
#include <uart.h>

// Expose this buffer so the HIL can read it when it needs to
extern CBRef uartBufferIn;
 
void copyBufferToDMA (unsigned char size);
void scheduleData (unsigned char hilOn, unsigned char* data4SPI);
void schedulerInit (void);
void send2DebugPort (unsigned char* protData, unsigned char hilOn);

#endif /* _IPCSCHEDULER_H_ */

