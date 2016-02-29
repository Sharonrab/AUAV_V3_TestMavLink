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

// ==============================================================
// gps.c
// This is code implements a fully interrupt driven UART reader to
// be used in the UCSC Autopilot project. It makes use of the 
// circular buffer data structure circBuffer.c. It has been 
// written to be implemented in Simulink. It configures UART 1
// at a predefined baud rate, then initializes a circular buffer,
// configures the interrupt and starts the service. 
// The main function gpsRead returns an array where byte 0 indicates
// how many new bytes were read, and byte m indicates how many remain
// in the buffer.
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 21 2008 @ 21:15
// ==============================================================
#include "AUAV_V3_TestMavLink.h"
#include "gpsUblox.h"
#include "circBuffer.h"

// this function converts one hex ascii character to decimal
// used for the checksum comparison
// Kindly contributed by: Bryant Mairs
extern CBRef uartBuffer;
extern mavlink_gps_raw_int_t mlGpsData;

char hex2char(char halfhex) {
    char rv;

    // Test for numeric characters
    if ((rv = halfhex - '0') <= 9 && rv >= 0) {
        return rv;
    }        // Otherwise check for upper-case A-F
    else if ((rv = halfhex - 'A') <= 5 && rv >= 0) {
        return rv + 10;
    }        // Finally check for lower-case a-f
    else if ((rv = halfhex - 'a') <= 5 && rv >= 0) {
        return rv + 10;
    }

    // Otherwise return -1 as an error
    return -1;
}

float degMinToDeg(unsigned char degrees, float minutes) {
    return ((float) degrees + minutes / 60.0);
}


// this function reads the serial stream as it comes in
// then assembles full messages, verifies the checksum and
// if valid sends it out to the parser

unsigned char gpsUbloxSeparate(unsigned char* outStream) {
    // Static variables CAREFUL
    static unsigned char outBuf [MSIZE] = {0};
    static unsigned char previousComplete = 1;
    static unsigned char indexLast = 0;

    // local variables
    unsigned char i = 0;
    unsigned char tmpChksum = 0;
    unsigned char chsmStr_0, chsmStr_1;
    unsigned char isValid = 0;
    unsigned char chksumHeader = 0;
    unsigned char tmpIndex = 0;
    unsigned char tmpLen = getLength(uartBuffer);

    // If the previous message was complete, then
    // go over the buffer and advance until you find a dollar sign
    if (previousComplete) {
        while (tmpLen > 0 && peak(uartBuffer) != '$') {
            readFront(uartBuffer);
            tmpLen--;
        }
    }

    // read until you find a CR or run out of bytes to read
    while (tmpLen > 0 && peak(uartBuffer) != 13) { //CR = 13
        outBuf[indexLast] = readFront(uartBuffer);
        indexLast++;
        tmpLen--;
    }

    // if we found a carriage return, then the message is complete
    if (peak(uartBuffer) == 13) {
        // validate the checksum
        chsmStr_0 = hex2char(outBuf[indexLast - 2]);
        chsmStr_1 = hex2char(outBuf[indexLast - 1]);
        // convert the hex checksum to decimal
        tmpChksum = (unsigned char) (chsmStr_0 * 16 + chsmStr_1);

        // verify the validity
        isValid = (tmpChksum == getChecksum(outBuf, indexLast));

        // turn on the flag of complete stream
        previousComplete = 1;
        // set the outStream valid flag
        outStream[MSIZE - 1] = isValid;

        // save the index value for copying purposes
        tmpIndex = indexLast;

        // reset the array index
        indexLast = 0;
    } else { // if we did not find a CR, i.e. we ran out of bytes
        // turn off the previousComplete flag
        previousComplete = 0;
    }

    // If we found a complete sentence and it is valid
    if (previousComplete && isValid) {
        // copy the data to the outstream
        for (i = 1; i <= tmpIndex; ++i) {
            outStream[i] = outBuf[i - 1];
        }

        // finally compute the type of message
        chksumHeader = getChecksum(outBuf, 6);

        // based on the obtained header checksum set the type
        switch (chksumHeader) {
            case GGACS:
                outStream[0] = GGAID;
                break;
            case RMCCS:
                outStream[0] = RMCID;
                break;
            default:
                outStream[0] = UNKID;
                break;
        }
    }
    return tmpLen;
}

void gpsUbloxParse(void) {

    unsigned char inStream[MSIZE];
    unsigned char bufferLen = 11;

    memset(inStream, 0, MSIZE);

    while (bufferLen > 10) {
        bufferLen = gpsUbloxSeparate(inStream);

        // if the sentence is valid
        if (inStream[MSIZE - 1] == 1) {
            // parse the data according to the header
            switch (inStream[0]) {
                case RMCID:
                    parseRMC(inStream);
                    break;
                case GGAID:
                    parseGGA(inStream);
                    break;
            }
        }

        memset(inStream, 0, MSIZE);
    }

}

void getGpsUbloxMainData(float* data) {
    gpsUbloxParse();
    data[0] = INT32_1E7_TO_FLOAT(mlGpsData.lat);
    data[1] = INT32_1E7_TO_FLOAT(mlGpsData.lon);
    data[2] = INT32_1E3_TO_FLOAT(mlGpsData.alt);
    data[3] = UINT16_1E2_TO_FLOAT(mlGpsData.cog);
    data[4] = UINT16_1E2_TO_FLOAT(mlGpsData.vel);
}

char gpSmbl(char symbl) {
    switch (symbl) {
        case 'N': return 1; /*North*/
        case 'E': return 1; /*East*/
        case 'S': return -1; /*South*/
        case 'W': return -1; /*West*/
        case 'A': return 3; /*Valid or Autonomous*/
        case 'D': return 1; /*Differential*/
        case 'V': return 0; /*Non-Valid*/
        case 'M': return 100; /*Meters*/
        default: return 0; /*Non-Valid*/
    }
}

// a return value of 1 means the string is done. No more tokens
// This function is stateful, call it once with the String and then with NULL
// similar to strtok but this will support succesive tokens like
// "find,,,the,,commas"

unsigned char myTokenizer(char* stringToTokenize, char token, char * returnToken) {
    static char * pch;
    static char * prevPch;
    static char * lastByte;

    // Make sure the return token is "empty"
    memset(returnToken, 0, TOKEN_SIZE);

    // get the pointer to next token if it exists 
    // and the stringToTokenize is null
    // Bahavior similar to strtok
    if (stringToTokenize == NULL) {
        pch = strchr(prevPch, token);
    } else {
        pch = strchr(stringToTokenize, token);
        prevPch = stringToTokenize;
        lastByte = stringToTokenize + strlen(stringToTokenize);
    }

    if (pch != NULL) {
        memcpy(returnToken, prevPch, pch - prevPch);
        prevPch = pch + 1;
    } else {
        memcpy(returnToken, prevPch, lastByte - prevPch);
    }

    return pch == NULL;
}

void parseRMC(unsigned char* stream) {
    // declare the local vars
    char token[TOKEN_SIZE];
    char tmp [3] = {0, 0, '\0'}, tmp3[4] = {0, 0, 0, '\0'};
    unsigned char chTmp = 0;


    // initialize tokenizer, let go first token which holds the msg type
    // token = strtok(stream, ",");
    myTokenizer((char *)stream, ',', token);

    // 1.- hhmmss.ssss
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
//        tmp[0] = token[0];
//        tmp[1] = token[1];
//        mlGpsDateTime.hour = (unsigned char) atoi(tmp);
//        tmp[0] = token[2];
//        tmp[1] = token[3];
//        mlGpsDateTime.min = (unsigned char) atoi(tmp);
//        tmp[0] = token[4];
//        tmp[1] = token[5];
//        mlGpsDateTime.sec = (unsigned char) atoi(tmp);
    }

    // 2.- Status of position Fix
    myTokenizer(NULL, ',', token);
    if (strlen(token) == 1) {
        mlGpsData.fix_type = gpSmbl((char) token[0]);
    }

    // 3.- Latitude
    // ddmm.mmmmmm
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        // get the first two values
        tmp[0] = token[0];
        tmp[1] = token[1];
        // get the degrees
        chTmp = (unsigned char) atoi(tmp);
        // make the degrees zero for minutes conversion
        token[0] = '0';
        token[1] = '0';
        // get the float
        mlGpsData.lat = FLOAT_TO_INT32_1E7(degMinToDeg(chTmp, atof(token)));

        // 4.- Latitude Sector
        myTokenizer(NULL, ',', token);
        if (strlen(token) == 1) {
            // set the sign of the float value
            mlGpsData.lat *= gpSmbl((char) token[0]);
        }
    }

    // 5.- Longitude
    // dddmm.mmmmmm
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        // get the first two values
        tmp3[0] = token[0];
        tmp3[1] = token[1];
        tmp3[2] = token[2];
        // get the degrees
        chTmp = (unsigned char) atoi(tmp3);
        // make the degrees zero for minutes conversion
        token[0] = '0';
        token[1] = '0';
        token [2] = '0';
        // get the float
        mlGpsData.lon = FLOAT_TO_INT32_1E7(degMinToDeg(chTmp, atof(token)));

        // 6.- Longitude Sector
        myTokenizer(NULL, ',', token);
        if (strlen(token) == 1) {
            // set the sign of the float value
            mlGpsData.lon *= gpSmbl((char) token[0]);
        }
    }

    // 7.- SOG in knots but gets stored in m/s CAUTION
    // xx.xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.vel = FLOAT_TO_UINT16_1E2((unsigned short) (atof(token) * KTS2MPS));
    }

    // 8.- COG in degrees
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.cog = FLOAT_TO_UINT16_1E2((unsigned short) atof(token));
    }

    // 9.- UTC Date
    // ddmmyy
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
        // get day
//        tmp[0] = token[0];
//        tmp[1] = token[1];
//        mlGpsDateTime.day = (unsigned char) atoi(tmp);
//        // get month
//        tmp[0] = token[2];
//        tmp[1] = token[3];
//        mlGpsDateTime.month = (unsigned char) atoi(tmp);
//        // get year
//        tmp[0] = token[4];
//        tmp[1] = token[5];
//        mlGpsDateTime.year = (unsigned char) atoi(tmp);
    }
}

void parseGGA(unsigned char* stream) {
    // declare the local vars
    char token[TOKEN_SIZE];
    char tmp [3] = {0, 0, '\0'}, tmp3[4] = {0, 0, 0, '\0'};
    unsigned char chTmp = 0;

    // initialize tokenizer, let go first token which holds the msg type
    myTokenizer((char *)stream, ',', token);

    // 1.- hhmmss.ssss
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
//        tmp[0] = token[0];
//        tmp[1] = token[1];
//        mlGpsDateTime.hour = (unsigned char) atoi(tmp);
//        tmp[0] = token[2];
//        tmp[1] = token[3];
//        mlGpsDateTime.min = (unsigned char) atoi(tmp);
//        tmp[0] = token[4];
//        tmp[1] = token[5];
//        mlGpsDateTime.sec = (unsigned char) atoi(tmp);
    }
    // 2.- Latitude
    // ddmm.mmmmmm
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        // get the first two values
        tmp[0] = token[0];
        tmp[1] = token[1];
        // get the degrees
        chTmp = (unsigned char) atoi(tmp);
        // make the degrees zero for minutes conversion
        token[0] = '0';
        token[1] = '0';
        // get the float
        mlGpsData.lat = FLOAT_TO_INT32_1E7(degMinToDeg(chTmp, atof(token)));
        // 3.- Latitude Sector
        myTokenizer(NULL, ',', token);
        if (strlen(token) == 1) {
            // set the sign of the float value
            mlGpsData.lat *= gpSmbl((char) token[0]);
        }
    }

    // 4.- Longitude
    // ddmm.mmmmmm
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        // get the first two values
        tmp3[0] = token[0];
        tmp3[1] = token[1];
        tmp3[2] = token[2];
        // get the degrees
        chTmp = (unsigned char) atoi(tmp3);
        // make the degrees zero for minutes conversion
        token[0] = '0';
        token[1] = '0';
        token [2] = '0';
        // get the float
        mlGpsData.lon = FLOAT_TO_INT32_1E7(degMinToDeg(chTmp, atof(token)));

        // 5.- Longitude Sector
        myTokenizer(NULL, ',', token);

        if (strlen(token) > 0) {
            // set the sign of the float value
            mlGpsData.lon *= gpSmbl((char) token[0]);
        }
    }

    // 6.- Quality Indicator
    myTokenizer(NULL, ',', token);
    if (strlen(token) == 1) {
        chTmp = (unsigned char) atoi(token);
        mlGpsData.fix_type =  (chTmp>1 && chTmp < 6)? GPS_FIX_3D : GPS_FIX_NONE ;
    }

    // 7.- Sats used in solution
    // xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        //mlGpsDateTime.visSat = (unsigned char) atoi(token);
        mlGpsData.satellites_visible = (unsigned char) atoi(token);
    }

    // 8.- HDOP given from 0 to 99.99 but stored from 0 - 990 
    //in integers, i.e HDOP = HDOP_stored/100 CAUTION
    // xx.xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.eph = FLOAT_TO_UINT16_1E2((unsigned short) (atof(token) * 10.0));
    }

    // 9.- Altitude above mean sea level given in meters
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.alt = FLOAT_TO_INT32_1E3(atof(token));
    }

}

// GPS checksum code based on 
// http://www.codeproject.com/KB/mobile/WritingGPSApplications2.aspx
// original code in C# written by Jon Person, author of "GPS.NET" (www.gpsdotnet.com)

unsigned char getChecksum(unsigned char* sentence, unsigned char size) {

    // Loop through all chars to get a checksum
    unsigned char checkSum = 0;
    unsigned char i;
    for (i = 0; i < size; i++) {
        if (sentence[i] == DOLLAR) {
            // Ignore the dollar sign
        } else if (sentence[i] == STAR) {
            // Stop processing before the asterisk
            break;
        } else {
            // Is this the first value for the checksum?
            if (i == 0) {
                // Yes. Set the checksum to the value
                checkSum = sentence[i];
            } else {
                // No. XOR the checksum with this character's value
                checkSum ^= sentence[i];
            }
        }
    }
    // Return the checksum 
    return checkSum;
}
