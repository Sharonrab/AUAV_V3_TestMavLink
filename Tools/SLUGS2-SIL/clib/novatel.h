

#ifndef NOVATEL_H
#define	NOVATEL_H

#ifdef	__cplusplus
extern "C" {
#endif

#define _TESTING_ 0

#if _TESTING_
#include "mavlink.h"

    extern mavlink_gps_raw_int_t mlGpsData;
    extern mavlink_gps_date_time_t mlGpsDateTime;
    extern mavlink_status_gps_t mlGpsStatus;
    extern mavlink_novatel_diag_t mlNovatelStatus;

    typedef union {
        unsigned char chData[8];
        double doData;
    } tDoubleToChar;

    typedef union {
        unsigned char chData[4];
        unsigned int uiData;
    } tUnsignedIntToChar;

#else

#include "mavlinkSensorMcu.h"
#include "gpsPort.h"
#include "gpsUblox.h"
    typedef union {
        unsigned char chData[8];
        long double doData;
    } tDoubleToChar;

    typedef union {
        unsigned char chData[4];
        unsigned long uiData;
    } tUnsignedIntToChar;
#endif

#include "apDefinitions.h"
#include "apUtils.h"

#include <stdlib.h>
#include <string.h>
#define CRC32_POLYNOMIAL	0xEDB88320L

#define HEAD1   170
#define HEAD2   68
#define HEAD3   18
#define HEAD_LEN 28
#define BEST_POS_ID 42
#define BEST_VEL_ID 99
#define TIME_ID 101

    typedef enum _DECODER_STATUS {
        DECO_START,
        DECO_RECEIVING_HEADER2,
        DECO_RECEIVING_HEADER3,
        DECO_RECEIVING_HEADER,
        DECO_RECEIVING_PAYLOAD,
        DECO_COMPUTING_CRC,
        DECO_DECODING,
    } DECODER_STATUS;

    void gpsInit(void);
    void gpsParse(uint8_t* dataStream);
    void updateStatus(uint8_t* completeMessage);
    void updatePositionData(uint8_t* completeMessage);
    void updateVelocityData(uint8_t* completeMessage);
    void updateTimeData(uint8_t* completeMessage);
    void getGpsMainData(float* data) ;

    uint32_t CalculateBlockCRC32(uint32_t ulCount, uint8_t *ucBuffer);
    uint32_t CRC32Value(int32_t i);

    //TODO: charsToUint16 and charsToFloat are duplicated functions. The same function is
    // already implemented in conversions.c GET RID OF THESE!
    uint16_t charsToUint16(uint8_t* data);
    uint32_t charsToUint32(uint8_t* data);
    float charsToFloat(uint8_t* data);

#if _TESTING_
    double charsToDouble(uint8_t* data);
#else
    long double charsToDouble(uint8_t* data);
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* NOVATEL_H */

