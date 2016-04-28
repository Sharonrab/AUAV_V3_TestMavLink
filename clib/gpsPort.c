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
// Last Revision: Feb 25 2012 @ 12:38
// ==============================================================

#include "gpsPort.h"

struct CircBuffer com1Buffer;
CBRef uartBuffer;


// UART and Buffer initialization

void uartBufferInit(void) {
    // initialize the circular buffer
    uartBuffer = (struct CircBuffer*) &com1Buffer;
    newCircBuffer(uartBuffer);
}

void gpsFreqConfig(void) {
#if USE_NOVATEL_GPS

#if USE_NMEA
    unsigned char enableGGA [] = "log com2 gpggalong ontime 0.2 0 hold\r\n\0"; //Habilitar GGA a 10 hz
    unsigned char enableRMC [] = "log com2 gprmc ontime 0.2 0.1 hold\r\n\0"; //Habilitar RMC a 10 h
    unsigned char enableVTG [] = "log com2 gpvtg ontime 0.25 0.2 hold\r\n\0"; //Habilitar RMC a 10 h

    putsUART1((unsigned int *) enableGGA);
    while (BusyUART1());
    putsUART2((unsigned int *) enableGGA);
    while (BusyUART2());
    hugeDelay();


    putsUART1((unsigned int *) enableRMC);
    while (BusyUART1());
    putsUART2((unsigned int *) enableRMC);
    while (BusyUART2());
    hugeDelay();


    putsUART1((unsigned int *) enableVTG);
    while (BusyUART1());
    putsUART2((unsigned int *) enableVTG);
    while (BusyUART2());
#else
    unsigned char enableBESTPOS [] = "log com2 bestposb ontime 0.2 0 hold\r\n\0"; //Position at 5 hz
    unsigned char enableBESTVEL [] = "log com2 bestvelb ontime 0.2 0.1 hold\r\n\0"; //Velocity at 5 hz
    unsigned char enableTIME [] = "log com2 timeb ontime 0.5 0.25 hold\r\n\0"; //Time at 2 hz
    
    putsUART1((unsigned int *) enableTIME);
    while (BusyUART1());
    hugeDelay();
    putsUART1((unsigned int *) enableTIME);
    while (BusyUART1());
    hugeDelay();
    
    // Send to debug port
    putsUART2((unsigned int *) enableTIME);
    while (BusyUART2());


    putsUART1((unsigned int *) enableBESTPOS);
    while (BusyUART1());
    hugeDelay();
    putsUART1((unsigned int *) enableBESTPOS);
    while (BusyUART1());
    hugeDelay();

    // Send to debug port
    putsUART2((unsigned int *) enableBESTPOS);
    while (BusyUART2());
   
 
    putsUART1((unsigned int *) enableBESTVEL);
    while (BusyUART1());
    hugeDelay();
    putsUART1((unsigned int *) enableBESTVEL);
    while (BusyUART1());
    hugeDelay();
    
    putsUART2((unsigned int *) enableBESTVEL);
    while (BusyUART2());
    
#endif

#else
    unsigned char chFreq [] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96}; //Configurar a 4 Hz
    uint8_t i;
    unsigned char chNav [] = {0xB5, 0x62, 0x06, 0x03, 0x1C, 0x00, 0x05, 0x03, 0x10, 0x18, 0x14, 0x05, 0x00, 0x3C,
        0x3C, 0x14, 0xE8, 0x03, 0x00, 0x00, 0x0F, 0x17, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00,
        0x2C, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x9F, 0x0A};

    unsigned char chNav5 [] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x10, 0x27, 0x00, 0x00, 0x05, 0x0F, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x25, 0x35};

    for (i = 0; i < sizeof (chFreq); i++) {
        WriteUART1((unsigned int) chFreq[i]);
        while (BusyUART1());
    }
    smallDelay();

    for (i = 0; i < sizeof (chNav); i++) {
        WriteUART1((unsigned int) chFreq[i]);
        while (BusyUART1());
    }
    smallDelay();

    for (i = 0; i < sizeof (chNav5); i++) {
        WriteUART1((unsigned int) chFreq[i]);
        while (BusyUART1());
    }
    hugeDelay();
    
#endif
}

// Interrupt service routine for U1 GPS port

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {

    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U1STAbits.URXDA == 1) {
        writeBack(uartBuffer, (unsigned char) U1RXREG);
    }

    // If there was an overun error clear it and continue
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS0bits.U1RXIF = 0;
}

void getGPSRawData(unsigned char* gpsBuffer) {
    unsigned char i;
    unsigned char tmpLen = (unsigned char) getLength(uartBuffer);

    memset(gpsBuffer, 0, MSIZE);

    gpsBuffer[0] = tmpLen > (MSIZE - 2) ? MSIZE - 2 : tmpLen;

#if USE_NMEA
    mlGpsStatus.msgsType += gpsBuffer[0];
#endif
    
    for (i = 1; i <= gpsBuffer[0]; i++) {
        gpsBuffer[i] = readFront(uartBuffer);
        /*
                // Uncomment to show what is being red by the GPS port
         #if !_TESTING_
                WriteUART2((unsigned int)gpsBuffer[i] );
                while(BusyUART2());
         #endif
         */
    }

    gpsBuffer[MSIZE - 1] = 1;
}

uint8_t isGPSNovatel (void){
    return (uint8_t) USE_NOVATEL_GPS;
}
