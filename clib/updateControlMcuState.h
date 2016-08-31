#ifndef _UPDATECONTROLMCUSTATE_H_
#define _UPDATECONTROLMCUSTATE_H_


#ifdef __cplusplus
       extern "C"{
#endif
      
#include "mavlinkControlMcu.h" 
#include "apDefinitions.h"

// void updatePilotCommands (unsigned short* pilCom);
void updatePWM (unsigned short * PWMData);
void updatePWMTrim (unsigned short PWMData, unsigned char channel );
void updateLoad (uint8_t mcuLoad);
void updateEuler (float* newEuler);
void updatePQR (float* newPQR);
void getGSLocation(float* altLatLon) ;       
uint8_t getLightsOnOff (void);
uint8_t getLightsDayNight (void);
void updateVISensor (uint16_t* data);
void updatePTZ (int16_t* ptz);

#ifdef __cplusplus
       }
#endif
       
       
#endif /* _UPDATECONTROLMCUSTATE_H_ */
