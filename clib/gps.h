#ifndef _GPS_H_
#define _GPS_H_


#ifdef __cplusplus
       extern "C"{
#endif

//#define _TESTING_ 1

#if _TESTING_

     #include "mavlink.h"

    extern mavlink_gps_raw_t mlGpsData;
    extern mavlink_gps_date_time_t mlGpsDateTime;

#else

    #include "mavlinkSensorMcu.h"
    #include "gpsPort.h"

#endif

#include "apDefinitions.h"
#include "apUtils.h"

#include <stdlib.h>
#include <string.h>


#define TOKEN_SIZE	15

// GPS Checksum Messages
// =====================
#define GGACS			86
#define RMCCS			75
#define GNRMCCS                                       85
#define VTGCS			82
#define GNVTGCS			76



char hex2char (char halfhex);
void gpsInit (void);
unsigned char gpsSeparate (unsigned char* inStream, unsigned char* outStream);
void gpsParse(unsigned char* dataStream);
void getGpsMainData (float* data);
float degMinToDeg (unsigned char degrees, float minutes);
char gpSmbl (char symbl);
void parseRMC (unsigned char* stream);
void parseGGA (unsigned char* stream);
void parseVTG(unsigned char* stream) ;
unsigned char getChecksum (unsigned char* sentence, unsigned char size);


#ifdef __cplusplus
       }
#endif

#endif /* _GPS_H_ */
