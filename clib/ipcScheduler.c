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
// ipcScheduler.c
// This is code implements a fully DMA driven UART writer to
// be used in the UCSC Autopilot project. It makes use of the 
// circular buffer data structure circBuffer.c. It has been 
// written to be implemented in Simulink. It configures UART 2
// at a predefined baud rate, then initializes a circular buffer,
// configures the DMA and starts the service. 
// The main function logData writes data to UART2 in the predefined
// comm protocol and returns data in the comm protocol to be sent
// via SPI to the second MCU. 
// 
// This file also contains Sensor MCU support functions
//
// Code by: Mariano I. Lizarraga
// First Revision: Aug 26 2008 @ 21:15
// =========================================================

#include "ipcScheduler.h"


struct CircBuffer com2Buffer;
CBRef logBuffer;

struct CircBuffer com2BufferIn;
CBRef uartBufferIn;

unsigned int BufferA[MAXSEND] __attribute__((space(dma))) = {0};

void schedulerInit(void) {
    // initialize the circular buffers
    logBuffer = (struct CircBuffer*) &com2Buffer;
    newCircBuffer(logBuffer);

    uartBufferIn = (struct CircBuffer*) &com2BufferIn;
    newCircBuffer(uartBufferIn);

    // DMA0REQ Register
    // ================
    DMA0REQ = 0x001F;

    // DMA0PAD Register
    // ================
    DMA0PAD = (volatile unsigned int) &U2TXREG;

    // DMA0CON Register
    // ================
    DMA0CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA0CONbits.MODE = 1; // One-shot, No Ping-Pong Mode	
    DMA0CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA0CONbits.SIZE = 0; // Word Data Transfer

    // DMA0CNT Register
    // ==============
    DMA0CNT = MAXSEND - 1;

    // DMA0STA Register
    // ================
    DMA0STA = __builtin_dmaoffset(BufferA);

    // Enable DMA0 TX interrupts
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

    // Configure and open the port;
    // U2MODE Register
    // ==============
    U2MODEbits.ABAUD = 0; // Disable autobaud
    U2MODEbits.PDSEL = 0; // No parity 8 bit
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2MODEbits.BRGH = 0; // Low speed mode

    // U2STA Register
    // ==============
    U2STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U2STAbits.UTXISEL1 = 0; // for the DMA
    U2STAbits.URXISEL = 0; // RX interrupt with every char
    U2STAbits.OERR = 0; // clear overun error

    // U2BRG Register
    // ==============
    U2BRG = LOG_UBRG; // Set the baud rate for data logger

    // Initialize the Interrupt  
    // ========================
    IPC7bits.U2RXIP = 5; // Interrupt priority 5  
    IFS1bits.U2RXIF = 0; // Clear the interrupt flag
    IEC1bits.U2RXIE = 1; // Enable interrupts

    // Enable the port;
    U2MODEbits.UARTEN = 1; // Enable the port	
    U2STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U2EIE = 0;
}

void copyBufferToDMA(unsigned char size) {
    unsigned char i;
    for (i = 0; i < size; i += 1) {
        BufferA[i] = (unsigned int) readFront(logBuffer);
    }
}

/*
    logData is the implementation of the outgoing Comunications 		
    protocol it is geared to prepare the data for logging and
    for the SPI inter processor communication. It uses a 
    static variable to monitor at which samples the outgoing messages 
    are queued in the circular buffer for DMA transmision.
 */

/*
    TODO: This function needs to be merged with prepareTelemetry
    in groundStationDriver.c and moved to something like
    apUtils.c
	
 */

void scheduleData(unsigned char hilOn, unsigned char* dataOut) {

    static BOOL sentHILMessage = FALSE;

    // Generic message container used to pack the messages
    mavlink_message_t msg;

    // Buffer to send data to HIL Sim
    uint8_t hilBuf[MAXSEND];

    // Cycles from 1 to 10 to decide which 
    // message's turn is to be sent
    static uint8_t samplePeriod = 1;

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;

    memset(&msg, 0, sizeof (mavlink_message_t));

    switch (samplePeriod) {
        case 1: //servo commands in HIL, sensor diag
            // if in HIL mode then send PWM commands 
            if (PORTDbits.RD2 == 1) {//AM DBG - proper form should be (hilOn == 1)
                mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlPwmCommands);

                hilBuf[0] = mavlink_msg_to_send_buffer(hilBuf + 1, &msg);

                send2DebugPort(hilBuf, 1);

                memset(&msg, 0, sizeof (mavlink_message_t));

                // Send HIL statustext message
                if (!sentHILMessage) {
                    strncpy(mlStatustext.text,"Sensor DSC in HIL Mode.",35);
                    mlStatustext.severity = MAV_SEVERITY_INFO;

                    mavlink_msg_statustext_encode(SLUGS_SYSTEMID,
                        SLUGS_COMPID, &msg, &mlStatustext);
                    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
                    sentHILMessage = TRUE;

                    memset(&msg, 0, sizeof (mavlink_message_t));
                }
                

            }
            else if (sentHILMessage) { sentHILMessage = FALSE; }

            mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorDiag);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;

        case 2: // LOAD, GPS
            mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlCpuLoadData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
            memset(&msg, 0, sizeof (mavlink_message_t));

            // Pack the GPS message
            mavlink_msg_gps_raw_int_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            break;
        case 3: // raw IMU	
            mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawImuData);

            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
            break;

        case 4: // Reboot (if required)		
            // it there has been a reboot
            if (mlBoot.version == 1) {
                // Copy the message to the send buffer
                mavlink_msg_boot_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    1);
                mlBoot.version = 0;
            }

            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;

        case 5: // Bias		
            mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorBiasData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;
        case 6: // GPS Status
#if USE_NMEA
            //mlGpsStatus.msgsType++;
            mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsStatus);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
#else
            mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNovatelStatus);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
#endif

            break;

        case 7: // Pilot Console Data
            mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPilotConsoleData);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;

        case 8: // Sensor Data in meaningful units
            mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlFilteredData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;

        case 9: // Raw Pressure
            mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawPressureData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;

        case 10:
            // Send response to control MCU messages
            if (sendCommandAcknowledgement) {

                mavlink_msg_command_ack_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlCommandAck);
                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
                sendCommandAcknowledgement = FALSE;
            }

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsDateTime);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break;
        default:
            *dataOut = 0;

            break;
    }

    memset(&msg, 0, sizeof (mavlink_message_t));
    // Air Data, Gets included every time
    mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAirData);
    // Copy the message to the send buffer
    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    // clear the message
    memset(&msg, 0, sizeof (mavlink_message_t));


    // Attitude data. Gets included every sample time
    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAttitudeData);

    // Copy the message to the send buffer	
    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    memset(&msg, 0, sizeof (mavlink_message_t));

    // XYZ Position. Gets included every sample time
    mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlLocalPositionData);
    // Copy the message to the send buffer
    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    samplePeriod = (samplePeriod >= 10) ? 1 : samplePeriod + 1;

    //send2DebugPort(dataOut,1);

}

void send2DebugPort(unsigned char* protData, unsigned char hilOn) {
    unsigned int bufLen, i;

    // add the data to the circular buffer
    for (i = 1; i <= protData[0]; i += 1) {
        writeBack(logBuffer, protData[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(logBuffer);



    // if the interrupt catched up with the circularBuffer
    //  then turn on the DMA
    if (!(DMA0CONbits.CHEN) && bufLen > 0) {
        // Configure the bytes to send
        DMA0CNT = bufLen <= (MAXSEND - 1) ? bufLen - 1 : MAXSEND - 1;
        // copy the buffer to the DMA channel outgoing buffer	
        copyBufferToDMA((unsigned char) DMA0CNT + 1);
        // Enable the DMA
        DMA0CONbits.CHEN = 1;
        // Init the transmission
        DMA0REQbits.FORCE = 1;
    }
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {

    // Clear the DMA0 Interrupt Flag;
    IFS0bits.DMA0IF = 0;

}

// Interrupt service routine for U2 HIL protocol port

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {

    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U2STAbits.URXDA == 1) {
        writeBack(uartBufferIn, (unsigned char) U2RXREG);
    }

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
}






