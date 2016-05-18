// ==============================================================
// gps.c
// This is code implements a fully interrupt driven UART reader to
// be used in the UCSC Autopilot project. It makes use of the
// circular buffer data structure circBuffer.c. It has been
// written to be implemented in Simulink. It configures UART 1
// at a predefined baud rate, then initializes a circular buffer,
// configures the interrupt and starts the service.
//
// Code by: Mariano I. Lizarraga
// First Revision: Aug 21 2008 @ 21:15
// Last Revision: Feb 25 2012 @ 12:38
// ==============================================================

#include "gps.h"
#include "mavlinkSensorMcu.h"


#if _TESTING_
mavlink_gps_raw_t mlGpsData;
mavlink_gps_date_time_t mlGpsDateTime;
#endif

void gpsInit(void) {

#if _TESTING_
    memset(&mlGpsData, 0, sizeof (mavlink_gps_raw_t));
    memset(&mlGpsDateTime, 0, sizeof (mavlink_gps_date_time_t));
#else
    uartInit();
#endif
}

// this function converts one hex ascii character to decimal
// used for the checksum comparison
// Kindly contributed by: Bryant Mairs

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
// if valid sends it out to the parser with the following structure
// inStream = [[Length] Data .....[currenIndex] ]
// outStream = [[MsgType] Data ... [isValid]]

unsigned char gpsSeparate(unsigned char* inStream, unsigned char* outStream) {
    // Static variables CAREFUL
    static unsigned char outBuf [MSIZE];
    static unsigned char previousComplete = 1;
    static unsigned char indexLast = 0;

    // local variables
    unsigned char i = 0;
    unsigned char tmpChksum = 0;
    unsigned char chsmStr_0, chsmStr_1;
    unsigned char isValid = 0;
    unsigned char chksumHeader = 0;
    unsigned char tmpIndex = 0;
    unsigned char* tmpLen = &inStream[0];
    unsigned char* inStreamIdx = &inStream[MSIZE - 1];
    
    // Exit immidiatley if called with a zero lenght stream
    if (*tmpLen == 0 ){
        return 0;
    }

    // If the previous message was complete, then
    // go over the buffer and advance until you find a dollar sign
    if (previousComplete) {
        while (*tmpLen > 0 && inStream[*inStreamIdx] != DOLLAR) {
           inStream[ (*inStreamIdx)++] = 0;
           (*tmpLen)--;
        }
    }

    // read until you find a CR or run out of bytes to read
#if _TESTING_
    while (*tmpLen > 0 && inStream[*inStreamIdx] != LF) {
#else
    while (*tmpLen > 0 && inStream[*inStreamIdx] != CR) {
#endif
        outBuf[indexLast++] = inStream[(*inStreamIdx)];
        inStream[(*inStreamIdx)++] = 0;
        (*tmpLen)--;
    }

    // if we found a carriage return, then the message is complete
#if _TESTING_
    if (inStream[*inStreamIdx] == LF) {
#else
    if (inStream[*inStreamIdx] == CR) {
#endif        
        // validate the checksum
        chsmStr_0 = hex2char(outBuf[indexLast - 2]);
        chsmStr_1 = hex2char(outBuf[indexLast - 1]);
        // convert the hex checksum to decimal
        tmpChksum = (unsigned char) (chsmStr_0 * 16 + chsmStr_1);

        // verify the validity
        isValid = (tmpChksum == getChecksum(outBuf, indexLast));

#if _TESTING_
        if (!isValid) {
            printf("=========== Checksum Failed! ============");
            exit(0);
        }
#else
        if (!isValid) {
            outBuf[indexLast] = '\0';
            mlGpsStatus.csFails++;
        }
#endif

        // turn on the flag of complete stream
        previousComplete = 1;
        // set the outStream valid flag
        outStream[MSIZE - 1] = isValid;

        // save the index value for copying purposes
        tmpIndex = indexLast;

        // reset the array index
        indexLast = 0;
        
        //  Reset the CR
        inStream[*inStreamIdx] = 0;
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
            case GNRMCCS:
            case RMCCS:
                outStream[0] = RMCID;
                break;
            case GNVTGCS:
            case VTGCS:
                outStream[0] = VTGID;
                break;
            default:
                outStream[0] = UNKID;
                break;
        }
        
        
    }
    return *tmpLen;
}

void gpsParse(unsigned char* dataStream) {

    unsigned char inStream[MSIZE];
    short bufferLen = dataStream[0];

    memset(inStream, 0, MSIZE);

    while (bufferLen > 0) {
        bufferLen = gpsSeparate(dataStream, inStream);

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
                case VTGID:
                    parseVTG(inStream);
                    break;

            }
        }
        memset(inStream, 0, MSIZE);
    }

}

void getGpsMainData(float* data) {
    data[0] = mlGpsData.lat;
    data[1] = mlGpsData.lon;
    data[2] = mlGpsData.alt;
    data[3] = mlGpsData.hdg;
    data[4] = mlGpsData.v;
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
    myTokenizer(stream, ',', token);

    // 1.- hhmmss.ssss
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
        tmp[0] = token[0];
        tmp[1] = token[1];
        mlGpsDateTime.hour = (unsigned char) atoi(tmp);
        tmp[0] = token[2];
        tmp[1] = token[3];
        mlGpsDateTime.min = (unsigned char) atoi(tmp);
        tmp[0] = token[4];
        tmp[1] = token[5];
        mlGpsDateTime.sec = (unsigned char) atoi(tmp);
    }

    // 2.- Status of position Fix
    myTokenizer(NULL, ',', token);
    if (strlen(token) == 1) {
       mlGpsStatus.posStatus = (uint8_t) token[0];
        
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
        mlGpsData.lat = degMinToDeg(chTmp, atof(token));

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
        mlGpsData.lon = degMinToDeg(chTmp, atof(token));

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
        mlGpsData.v = (atof(token) * KTS2MPS);
    }

    // 8.- COG in degrees
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.hdg = atof(token);
    }

    // 9.- UTC Date
    // ddmmyy
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
        // get day
        tmp[0] = token[0];
        tmp[1] = token[1];
        mlGpsDateTime.day = (unsigned char) atoi(tmp);
        // get month
        tmp[0] = token[2];
        tmp[1] = token[3];
        mlGpsDateTime.month = (unsigned char) atoi(tmp);
        // get year
        tmp[0] = token[4];
        tmp[1] = token[5];
        mlGpsDateTime.year = (unsigned char) atoi(tmp);
    }
    
    // 11.- Magnetic Variation
    //x.x
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsStatus.magVar = atof(token);
    }
    
    // 12.- Variation Direction
    myTokenizer(NULL, ',', token);
    if (strlen(token) == 1) {
       mlGpsStatus.magDir = (int8_t) (token[0] == 'E'? -1 : 1);
    }
    
    // 13.- Mode Indicator
    myTokenizer(NULL, '*', token);
    if (strlen(token) == 1) {
       mlGpsStatus.modeInd = (uint8_t) token[0];
    }
}

void parseGGA(unsigned char* stream) {
    // declare the local vars
    char token[TOKEN_SIZE];
    char tmp [3] = {0, 0, '\0'}, tmp3[4] = {0, 0, 0, '\0'};
    unsigned char chTmp = 0;

    // initialize tokenizer, let go first token which holds the msg type
    myTokenizer(stream, ',', token);

    // 1.- hhmmss.ssss
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 5) {
        tmp[0] = token[0];
        tmp[1] = token[1];
        mlGpsDateTime.hour = (unsigned char) atoi(tmp);
        tmp[0] = token[2];
        tmp[1] = token[3];
        mlGpsDateTime.min = (unsigned char) atoi(tmp);
        tmp[0] = token[4];
        tmp[1] = token[5];
        mlGpsDateTime.sec = (unsigned char) atoi(tmp);
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
        mlGpsData.lat = degMinToDeg(chTmp, atof(token));
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
        mlGpsData.lon = degMinToDeg(chTmp, atof(token));

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
        mlGpsData.fix_type = (char) atoi(token);
        mlGpsStatus.gpsQuality = (uint8_t) atoi(token);
    }

    // 7.- Sats used in solution
    // xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsDateTime.visSat = (unsigned char) atoi(token);
    }

    // 8.- HDOP given from 0 to 99.99 but stored from 0 - 990
    //in integers, i.e HDOP = HDOP_stored/100 CAUTION
    // xx.xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.eph = (unsigned short) (atof(token) * 10.0);
    }

    // 9.- Altitude above mean sea level given in meters
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.alt = atof(token);
    }

}

void parseVTG(unsigned char* stream) {
    // declare the local vars
    char token[TOKEN_SIZE];
    unsigned char chTmp = 0;

    // initialize tokenizer, let go first token which holds the msg type
    myTokenizer(stream, ',', token);

    // 1.- Track true
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.hdg = atof(token);
    }

    // 2.- Let go the token of track indicator
    myTokenizer(NULL, ',', token);

    // 3.- Magnetic Track
    // xxx.xxx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.epv = atof(token);
    }

    // 4.- Let go the token of track indicator
    myTokenizer(NULL, ',', token);

    // 5.- SOG in knots but gets stored in m/s CAUTION
    // xx.xx
    myTokenizer(NULL, ',', token);
    if (strlen(token) > 0) {
        mlGpsData.v = (atof(token) * KTS2MPS);
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
        if ((sentence[i] == DOLLAR) || (sentence[i] == LF)) {
            // Ignore the dollar sign or the LF
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

