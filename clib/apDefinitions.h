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

/* ==================================================
This is the definition file for the the UCSC AP code
it creates all the common defines, unions, enumerations
and data types.

Code by: Mariano I. Lizarraga
First Revision: Aug 18 2008 @ 17:42
====================================================*/
#ifndef _APDEFINITIONS_H_
#define _APDEFINITIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

    // =========== Global Definitions ==========

    // Boolean types
    // ===================
typedef char BOOL;
#define TRUE ((char)1)
#define FALSE ((char)0)

#define SUCCESS ((char)0)
#define FAILURE ((char)-1)

    // Circular Buffer Size
    // ===================
#define BSIZE			512

    // GPS Circular Buffers
    // ====================
#define MSIZE			180

    // UAV System ID
    // =============
#define SLUGS_SYSTEMID		101
#define SLUGS_COMPID		1

#define GS_SYSTEMID		127
#define GS_COMPID		0

    // Cube Model Used
    // ===============
#define USE_CUBE_16405	 1
    
    // GPS Model Used
#define USE_NOVATEL_GPS  0

    // GPS Header IDs
    // ==============
#define GGAID			1
#define RMCID			2
#define VTGID			3
#define UNKID			254

#define USE_NMEA 		1

    // DMA Maximum Char Sending
    // ========================
#define MAXSEND					109
#define MAXSPI                   200

    // Maximun Number of WPs and PIDs
#define MAX_NUM_WPS		17

    // Age limit of heartbeat to consider that the UAV has lost comm with GC
#define HEARTBEAT_LIMIT 6000  // 30 seconds

    // Define log raw data at 100 hz. Comment out to have
    // XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
    //#define LOGRAW100	1

    // Define diagnostic data at 100 hz. Comment out to have
    // XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
    //#define DIAG100		1

    // ERROR MESSAGES
    // ==============

    // GPS Fix Types
//    enum GPS_FIX {
//        GPS_FIX_NONE = 0,
//        GPS_FIX_2D = 2,
//        GPS_FIX_3D = 3
//    };

    // PID EEPROM Error Messages

    enum EEPROM_STATUS {
        EEP_MEMORY_OK,
        EEP_WRITE_FAIL = 10,
        EEP_PAGE_EXP = 20,
        EEP_MEMORY_CORR = 30
    };

    enum SLUGS_ACTION {
        SLUGS_ACTION_NONE,
        SLUGS_ACTION_SUCCESS,
        SLUGS_ACTION_FAIL,
        SLUGS_ACTION_EEPROM,
        SLUGS_ACTION_MODE_CHANGE,
        SLUGS_ACTION_MODE_REPORT,
        SLUGS_ACTION_PT_CHANGE,
        SLUGS_ACTION_PT_REPORT,
        SLUGS_ACTION_PID_CHANGE,
        SLUGS_ACTION_PID_REPORT,
        SLUGS_ACTION_WP_CHANGE,
        SLUGS_ACTION_WP_REPORT,
        SLUGS_ACTION_MLC_CHANGE,
        SLUGS_ACTION_MLC_REPORT,
        SLUGS_ACTION_SPI_TO_SENSOR,
        SLUGS_ACTION_SAVE_MOBILE_LOCATION,
        SLUGS_ACTION_CAMERA_ORDER,
        SLUGS_ACTION_RTB_ON_NO_MOBILE,
        SLUGS_ACTION_RTB_ON_MOBILE,
        SLUGS_ACTION_RTB_OFF,
        SLUGS_ACTION_ISR_LOCATION,
        SLUGS_ACTION_LIGHTS,
        SLUGS_ACTION_CAMERA_CONFIG,
    };

    enum WP_PROTOCOL {
        WP_PROT_IDLE,
        WP_PROT_LIST_REQUESTED,
        WP_PROT_NUM_SENT,
        WP_PROT_TX_WP,
        WP_PROT_RX_WP,
        WP_PROT_SENDING_WP_IDLE,
        WP_PROT_GETTING_WP_IDLE
    };

    enum PARAM_INTERFACE {
        PI_IDLE,
        PI_SEND_ALL_PARAM,
        PI_SEND_ONE_PARAM
    };


    // Standard characters used in the parsing of messages
    // ===================================================
#define DOLLAR	36
#define STAR	42
#define CR	13
#define LF                   10
#define AT		64


    // Telemetry rates
    // ===============
//#define MAVLINK_TELEMETRY_RATE      50.0f // comment this out to disable parameter


    // Standard Units
    // ==============
#define KTS2MPS 		0.514444444
#define PI          3.141592653589793

    // Periphereal Configurations
    // ==========================
#define APFCY			40000000


#define GPSBAUDF		9600//19200//38400
#define GPSBAUDI		9600//38400
#define SYSCLK APFCY
    //UxBRG = ((sysclk/baudrate)/16)-1
#define UCSCAP_UBRGF 	((SYSCLK/GPSBAUDF)/16)-1 //64 for 38400

#define UCSCAP_UBRGI 	((SYSCLK/GPSBAUDI)/16)-1 //64 for 38400

#define LOGBAUD		115200
#define LOG_UBRG		21

#define CAMERABAUD              9600
#define CAMERA_UBRG            259


    // ifdef switches for debugging and conditional inclusion
    // ======================================================
#define __IN_DSPIC__ 	1 // switch for use in PC

#if __IN_DSPIC__
#ifdef DEBUG
#undef DEBUG
#endif
#else
#define DEBUG 1
#endif

    // Uncomment if there is no magentometer
#define NO_MAGNETO

    // Uncomment to allow full gyro calibration
    //#define DO_FULL_CAL


    // ============= Unions Used for Data Transmission ====
    //Type definitions for standard unions used in sending
    // and receiving data

    typedef union {
        unsigned char chData[2];
        unsigned short usData;
    } tUnsignedShortToChar;

    typedef union {
        unsigned char chData[2];
        short shData;
    } tShortToChar;

    typedef union {
        unsigned char chData[4];
        int inData;
    } tIntToChar;

    typedef union {
        unsigned char chData[4];
        float flData;
        unsigned short shData[2];
    } tFloatToChar;

    typedef union {
        unsigned char chData[8];
        uint16_t shData[4];
        uint64_t liData;
    } t64IntToChar;

#ifdef __cplusplus
}
#endif

#endif /* _APDEFINITIONS_H_ */
