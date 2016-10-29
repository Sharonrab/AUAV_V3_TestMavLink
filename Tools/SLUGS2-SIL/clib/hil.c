#include "hil.h"
extern CBRef uartBuffer;

void hilRead(unsigned char* hilChunk) {

    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next hilRead
     //unsigned char tmpLen = getLength(uartBufferIn), i = 0;
        unsigned char tmpLen = getLength(uartBuffer), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    hilChunk[0] = (tmpLen > MAXSEND - 1) ? MAXSEND - 1 : tmpLen;

    // read the data 
    for (i = 1; i <= hilChunk[0]; i += 1) {
//        hilChunk[i] = readFront(uartBufferIn);
        hilChunk[i] = readFront(uartBuffer);

    } 
}

void hil_getRawRead(short * rawData) {
    rawData[0] = mlRawImuData.xgyro;
    rawData[1] = mlRawImuData.ygyro;
    rawData[2] = mlRawImuData.zgyro;
    rawData[3] = mlRawImuData.xacc;
    rawData[4] = mlRawImuData.yacc;
    rawData[5] = mlRawImuData.zacc;
    rawData[6] = mlRawImuData.xmag;
    rawData[7] = mlRawImuData.ymag;
    rawData[8] = mlRawImuData.zmag;
    rawData[9] = mlRawPressureData.press_abs;
    rawData[10] = mlRawPressureData.press_diff1;
    rawData[11] = mlCpuLoadData.batVolt;
    rawData[12] = mlRawPressureData.temperature;
}

void hil_getVned(float* vned) {
    vned[0] = mlLocalPositionData.vx;
    vned[1] = mlLocalPositionData.vy;
    vned[2] = mlLocalPositionData.vz;
}

void hil_getEuler(float* euler) {
    euler[0] = mlAttitudeData.roll;
    euler[1] = mlAttitudeData.pitch;
    euler[2] = mlAttitudeData.yaw;
}

void hil_getRates(float* pqr) {
    pqr[0] = mlAttitudeData.rollspeed;
    pqr[1] = mlAttitudeData.pitchspeed;
    pqr[2] = mlAttitudeData.yawspeed;
}

void hil_getXYZ(float* xyz) {
    xyz[0] = mlLocalPositionData.x;
    xyz[1] = mlLocalPositionData.y;
    xyz[2] = mlLocalPositionData.z;
}

unsigned int hil_getTs(void) {
    return mlAttitudeData.time_boot_ms;
}

void protDecodeHil(uint8_t* dataIn) {

    uint8_t i/*, indx, writeSuccess*/;
    //uint16_t indexOffset;
    //tFloatToChar tempFloat;
    //uint32_t temp;


    mavlink_message_t msg;
    mavlink_status_t status;

    for (i = 1; i <= dataIn[0]; i++) {
        // Try to get a new message
        if (mavlink_parse_char(0, dataIn[i], &msg, &status)) {

            // Handle message
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    mavlink_msg_gps_raw_int_decode(&msg, &mlGpsData);
                    break;

                case MAVLINK_MSG_ID_CPU_LOAD:
                    mavlink_msg_cpu_load_decode(&msg, &mlCpuLoadData);
                    break;

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msg, &mlLocalPositionData);
                    break;

                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    mavlink_msg_scaled_pressure_decode(&msg, &mlAirData);
                    break;

                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msg, &mlAttitudeData);
                    break;

                case MAVLINK_MSG_ID_RAW_IMU:
                    mavlink_msg_raw_imu_decode(&msg, &mlRawImuData);
                    break;

                case MAVLINK_MSG_ID_RAW_PRESSURE:
                    mavlink_msg_raw_pressure_decode(&msg, &mlRawPressureData);
                    break;

                case MAVLINK_MSG_ID_GPS_DATE_TIME:
                    mavlink_msg_gps_date_time_decode(&msg, &mlGpsDateTime);
                    break;

            } // switch	
        } // if
    }// for  
}

