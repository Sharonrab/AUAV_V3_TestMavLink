#ifndef FCBEX1000_H_INCLUDED
#define FCBEX1000_H_INCLUDED

#include "apDefinitions.h"
#include "apUtils.h"

#include "mavlinkControlMcu.h"

#include <p33fxxxx.h>
#include <uart.h>

#include "inttypes.h"

#define CANAL_PAN   0x01
#define CANAL_TILT  0x02

#define ZERO_PAN     978
#define ZERO_TILT     1266
#define ZERO_ROLL   977

#define PANTILT_INCREMENT       7

#define MAX_PAN     1630
#define MIN_PAN       271
#define MAX_PAN_AIDE   240
#define MIN_PAN_AIDE   -240

#define MAX_TILT     1266
#define MIN_TILT       698
#define MAX_TILT_AIDE   240
#define MIN_TILT_AIDE   -240

#define MAX_ROLL     1095
#define MIN_ROLL      859

// For Pan and Tilt
// 1 ms is 625 OCxR
// 2 ms is 1250 OCxR

void configuracionInicial(void);
void uartCameraInit(void);
void controlZoom(uint8_t order);

void panTiltInit(void) ;
uint16_t getPanValue(uint8_t goHome) ;
uint16_t getTiltValue(uint8_t goHome) ;

void updateRoll (uint16_t newRoll);
void updatePan (uint16_t newPan) ;
void updateTilt (uint16_t newTilt);

uint8_t getGoHome (void);

void sendToCamera (uint8_t* mensaje, uint8_t size);
void controlICR(uint8_t icr);
void controlBrightness(uint8_t bright);
void controlStabilization(uint8_t stabil);
void controlSmoothing(uint8_t smooth);
void setCameraConfig(void);

int16_t getTiltISRAide(uint8_t goHome);
int16_t getPanISRAide(uint8_t goHome);

/*
void controlCamara(void);
void controlZoom(void);
void construyeZoom(unsigned char* hexa, int numero);
void controlBrillo(int order);
void controlApertura(int order);
void controlIris(int order);
void controlICRAuto(int order);
void controlICR(int order);
void controlBacklight(int order);
 */


#endif // FCBEX1000_H_INCLUDED
