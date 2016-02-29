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
#include <string.h>
#include "gpsPort.h"

struct CircBuffer com4Buffer;
CBRef uartBuffer;
    // GPS Circular Buffers
    // ====================
#define MSIZE			180

// UART and Buffer initialization

void uartBufferInit(void) {

     /* Configure Pins as Analog or Digital */
    ANSELE = 0xFF83;

    /* Configure Remappables Pins */
    RPINR28 = 0x56;
    /* Configure UART4 Rx Interruption */
    _U4RXIP = 1;                         /* Rx Interrupt priority set to 1 */
    _U4RXIF = 0;
    _U4RXIE = 1;                         /* Enable Interrupt */
    uartBuffer = (struct CircBuffer*) &com4Buffer;
    newCircBuffer(uartBuffer);
}

// Interrupt service routine for U4 GPS port
void __attribute__((__interrupt__, no_auto_psv)) _U4RXInterrupt(void) {
      _U4RXIF = 0;
    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U4STAbits.URXDA == 1) {
        writeBack(uartBuffer, (unsigned char) U4RXREG);
    }

    // If there was an overun error clear it and continue
    if (U4STAbits.OERR == 1) {
        _U4RXIF = 0;
        U4STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS5bits.U4RXIF = 0;
}




