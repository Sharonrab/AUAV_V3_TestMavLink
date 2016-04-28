#include "novatel.h"
#include "mavlinkControlMcu.h"

#if _TESTING_
mavlink_gps_raw_t mlGpsData;
mavlink_gps_date_time_t mlGpsDateTime;
mavlink_status_gps_t mlGpsStatus;
mavlink_novatel_diag_t mlNovatelStatus;
#endif

void gpsInit(void) {

#if _TESTING_
    memset(&mlGpsData, 0, sizeof (mavlink_gps_raw_t));
    memset(&mlGpsDateTime, 0, sizeof (mavlink_gps_date_time_t));
    memset(&mlGpsStatus, 0, sizeof (mavlink_status_gps_t));
    memset(&mlNovatelStatus, 0, sizeof (mavlink_novatel_diag_t));

#else
    
    uartBufferInit();
#endif
}

void gpsParse(uint8_t* dataStream) {

    DECODER_STATUS static currentStatus = DECO_START;

    uint16_t static payloadLen = 0;
    uint16_t static bytesRead = 0;
    uint8_t static completeMessage [MSIZE];
    uint16_t static totalBytes = 0;
    uint32_t static currentChecksum = 0;

    uint8_t ii = 1;
    uint8_t bytesReceived = dataStream[0];
    tUnsignedShortToChar shTemp;

    while (bytesReceived) {
        switch (currentStatus) {

            case DECO_START:
                payloadLen = 0;
                bytesRead = 0;
                totalBytes = 0;
                currentChecksum = 0;

                while (dataStream[ii] != HEAD1) {
                    ii++;
                    bytesReceived--;

                    if (bytesReceived == 0) {
                        return;
                    }
                }
                completeMessage[bytesRead++] = HEAD1;
                ii++;
                bytesReceived--;
                currentStatus = DECO_RECEIVING_HEADER2;
                break;

            case DECO_RECEIVING_HEADER2:
                if (dataStream[ii] != HEAD2) {
                    currentStatus = DECO_START;

                } else {
                    ii++;
                    bytesReceived--;
                    completeMessage[bytesRead++] = HEAD2;
                    currentStatus = DECO_RECEIVING_HEADER3;
                }

                break;

            case DECO_RECEIVING_HEADER3:
                if (dataStream[ii] != HEAD3) {
                    currentStatus = DECO_START;
                    bytesRead = 0;
                } else {
                    ii++;
                    bytesReceived--;
                    completeMessage[bytesRead++] = HEAD3;
                    currentStatus = DECO_RECEIVING_HEADER;
                }
                break;

            case DECO_RECEIVING_HEADER:
                while ((bytesReceived > 0) && (bytesRead < HEAD_LEN)) {
                    completeMessage[bytesRead++] = dataStream[ii++];
                    bytesReceived--;
                }

                if (bytesRead == HEAD_LEN) {
                    currentStatus = DECO_RECEIVING_PAYLOAD;

                    payloadLen = charsToUint16(&(completeMessage[8]));

                    totalBytes = payloadLen + HEAD_LEN + 4; // this includes the 4 CRC bytes
                }
                break;


            case DECO_RECEIVING_PAYLOAD:
                while ((bytesReceived > 0) && (bytesRead < totalBytes)) {
                    completeMessage[bytesRead++] = dataStream[ii++];
                    bytesReceived--;
                }

                if (bytesRead == totalBytes) {
                    currentStatus = DECO_COMPUTING_CRC;

                    currentChecksum = charsToUint32(&(completeMessage[bytesRead - 4]));
                }
                break;

            case DECO_COMPUTING_CRC:
                if (CalculateBlockCRC32(bytesRead - 4, completeMessage) == currentChecksum) {
                    currentStatus = DECO_DECODING;
                } else {
                    mlNovatelStatus.csFails++;
                    currentStatus = DECO_START;
                }
                break;

            case DECO_DECODING:
                shTemp.chData[0] = completeMessage[4];
                shTemp.chData[1] = completeMessage[5];

                updateStatus(completeMessage);

                switch (shTemp.usData) {
                    case BEST_POS_ID:
                        updatePositionData(completeMessage);
                        break;

                    case BEST_VEL_ID:
                        updateVelocityData(completeMessage);
                        break;

                    case TIME_ID:
                        updateTimeData(completeMessage);
                        break;

                    default:

                        break;
                }

                currentStatus = DECO_START;
                break;
        } // switch

    }// while bytes received

}

void updateStatus(uint8_t* completeMessage) {
    mlGpsDateTime.percentUsed = completeMessage[12];
    mlNovatelStatus.timeStatus = completeMessage[13];
    mlNovatelStatus.receiverStatus = (uint32_t) completeMessage[20];
};

void updatePositionData(uint8_t* completeMessage) {
    t64IntToChar temp;
    temp.liData = mlGpsData.time_usec;
    temp.shData[0]++;
    mlGpsData.time_usec = temp.liData;

    mlNovatelStatus.solStatus = (uint8_t) charsToUint32(&(completeMessage[28]));
    mlNovatelStatus.posType = (uint8_t) charsToUint32(&(completeMessage[32]));

    mlGpsData.fix_type = ((mlNovatelStatus.posType>15) && (mlNovatelStatus.solStatus == 0))?
        GPS_FIX_3D : GPS_FIX_NONE;
    
    mlGpsData.lat = FLOAT_TO_INT32_1E7((float) charsToDouble(&(completeMessage[36])));
    mlGpsData.lon = FLOAT_TO_INT32_1E7((float) charsToDouble(&(completeMessage[44])));
    mlGpsData.alt = FLOAT_TO_INT32_1E3((float) charsToDouble(&(completeMessage[52])));

    mlNovatelStatus.posSolAge = charsToFloat(&(completeMessage[88]));

    mlGpsDateTime.visSat = completeMessage[92];
    mlGpsDateTime.useSat = completeMessage[93];
    mlGpsDateTime.GppGl = completeMessage[94];

    mlGpsDateTime.sigUsedMask = completeMessage[99];

};

void updateVelocityData(uint8_t* completeMessage) {
    t64IntToChar temp;
    temp.liData = mlGpsData.time_usec;
    temp.shData[1]++;
    mlGpsData.time_usec = temp.liData;

    mlNovatelStatus.velType = (uint8_t) charsToUint32(&(completeMessage[32]));
    mlGpsData.vel = FLOAT_TO_UINT16_1E2((float) charsToDouble(&(completeMessage[44])));
    mlGpsData.cog = FLOAT_TO_UINT16_1E2((float) charsToDouble(&(completeMessage[52])));
};

void updateTimeData(uint8_t* completeMessage) {
     t64IntToChar temp;
    temp.liData = mlGpsData.time_usec;
    temp.shData[2]++;
    mlGpsData.time_usec = temp.liData;
    
    mlGpsDateTime.clockStat = (uint8_t) charsToUint32(&(completeMessage[28]));
    mlGpsDateTime.year = (uint8_t) (charsToUint32(&(completeMessage[56])) - 2000);
    mlGpsDateTime.month = completeMessage[60];
    mlGpsDateTime.day = completeMessage[61];
    mlGpsDateTime.hour = completeMessage[62];
    mlGpsDateTime.min = completeMessage[63];
    mlGpsDateTime.sec = (uint8_t) (charsToUint32(&(completeMessage[64])) / 1000);
};

void getGpsMainData(float* data) {
    data[0] = INT32_1E7_TO_FLOAT(mlGpsData.lat);
    data[1] = INT32_1E7_TO_FLOAT(mlGpsData.lon);
    data[2] = INT32_1E3_TO_FLOAT(mlGpsData.alt);
    data[3] = UINT16_1E2_TO_FLOAT(mlGpsData.cog);
    data[4] = UINT16_1E2_TO_FLOAT(mlGpsData.vel);
}

// Functions provided by NOVATEL

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
 -------------------------------------------------------------------------- */
uint32_t CRC32Value(int32_t i) {
    int32_t j;
    uint32_t ulCRC;
    ulCRC = i;
    for (j = 8; j > 0; j--) {
        if (ulCRC & 1) {
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        } else {
            ulCRC >>= 1;
        } // if
    } // for
    return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
uint32_t CalculateBlockCRC32(
    uint32_t ulCount, /* Number of bytes in the data block */
    uint8_t *ucBuffer) { /* Data block */

    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    while (ulCount-- != 0) {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int32_t) ulCRC ^ *ucBuffer++) & 0xff);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return ( ulCRC);
}

uint16_t charsToUint16(uint8_t* data) {
    tUnsignedShortToChar shTemp;

    shTemp.chData[0] = data[0];
    shTemp.chData[1] = data[1];

    return shTemp.usData;
}

uint32_t charsToUint32(uint8_t* data) {
    tUnsignedIntToChar inTemp;

    inTemp.chData[0] = data[0];
    inTemp.chData[1] = data[1];
    inTemp.chData[2] = data[2];
    inTemp.chData[3] = data[3];

    return inTemp.uiData;
}

float charsToFloat(uint8_t* data) {
    tFloatToChar flTemp;

    flTemp.chData[0] = data[0];
    flTemp.chData[1] = data[1];
    flTemp.chData[2] = data[2];
    flTemp.chData[3] = data[3];

    return flTemp.flData;
}

#if _TESTING_

double charsToDouble(uint8_t* data) {
#else

long double charsToDouble(uint8_t* data) {
#endif
    tDoubleToChar doTemp;

    doTemp.chData[0] = data[0];
    doTemp.chData[1] = data[1];
    doTemp.chData[2] = data[2];
    doTemp.chData[3] = data[3];
    doTemp.chData[4] = data[4];
    doTemp.chData[5] = data[5];
    doTemp.chData[6] = data[6];
    doTemp.chData[7] = data[7];

    return doTemp.doData;

}
