
#include <math.h>
#include "mavlinkCommsControlMcu.h"

/*

MAVLink supports the following UAV Modes:

enum MAV_MODE
{
    MAV_MODE_UNINIT = 0,     ///< System is in undefined state
    MAV_MODE_LOCKED = 1,     ///< Motors are blocked, system is safe
    MAV_MODE_MANUAL = 2,     ///< System is allowed to be active, under manual (RC) control
    MAV_MODE_GUIDED = 3,     ///< System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_AUTO =   4,     ///< System is allowed to be active, under autonomous control and navigation
    MAV_MODE_TEST1 =  5,     ///< Generic test mode, for custom use
    MAV_MODE_TEST2 =  6,     ///< Generic test mode, for custom use
    MAV_MODE_TEST3 =  7,     ///< Generic test mode, for custom use
    MAV_MODE_READY =  8,     ///< System is ready, motors are unblocked, but controllers are inactive
    MAV_MODE_RC_TRAINING = 9 ///< System is blocked, only RC valued are read and reported back
};

SLUGS uses two of these, MANUAL and GUIDED. The former, renders the navigation
modes (see navSupport.c) irrelevant which is when the safety pilot controls
the aircraft from the ground. In the other (GUIDED) it is the autopilot
who is in control.

MAVLink also supports a set of states:

enum MAV_STATE
{
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATING,
    MAV_STATE_STANDBY,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_EMERGENCY,
    MAV_STATE_HILSIM,
    MAV_STATE_PILOT,
    MAV_STATE_POWEROFF
};

SLUGS uses two of these as follows:

MAV_STATE_ACTIVE: The autopilot is reading data from the real sensors onboard

MAV_STATE_HILSIM: The autopilot is in HIL mode and thus reading data from the
                                    HIL sim serial port in the sensor MCU.

 */

void sendTelemetryMavlink(unsigned char* dataOut);


struct CircBuffer com2BufferIn;
CBRef uartBufferIn;


struct CircBuffer com1BufferOut;
CBRef uartBufferOut;

unsigned int BufferB[MAXSEND] __attribute__((space(dma))) = {0};

char sw_debug;
char sw_temp[50];
char sw_intTemp;
float fl_temp1, fl_temp2;

// UART, DMA and Buffer initialization

void uart2Init(void) {
    sw_debug = 0;

    // initialize the circular buffers
    uartBufferIn = (struct CircBuffer*) &com2BufferIn;
    newCircBuffer(uartBufferIn);

    uartBufferOut = (struct CircBuffer*) &com1BufferOut;
    newCircBuffer(uartBufferOut);


    // DMA1REQ Register
    // ================
    DMA1REQ = 0x001F;

    // DMA1PAD Register
    // ================
    DMA1PAD = (volatile unsigned int) &U2TXREG;

    // DMA1CON Register
    // ================
    DMA1CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA1CONbits.MODE = 1; // One-shot, No Ping-Pong Mode	
    DMA1CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA1CONbits.SIZE = 0; // Word Data Transfer

    // DMA1CNT Register
    // ==============
    DMA1CNT = MAXSEND - 1;

    // DMA1STA Register
    // ================
    DMA1STA = __builtin_dmaoffset(BufferB);

    // Enable DMA1 TX interrupts
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IPC3bits.DMA1IP = 6; // interrupt priority to 6
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    // Configure and open the port;
    // U2MODE Register
    // ==============
    U2MODEbits.UARTEN = 0; // Disable the port		
    U2MODEbits.USIDL = 0; // Stop on idle
    U2MODEbits.IREN = 0; // No IR decoder
    U2MODEbits.RTSMD = 0; // Ready to send mode (irrelevant)
    U2MODEbits.UEN = 0; // Only RX and TX
    U2MODEbits.WAKE = 1; // Enable at startup
    U2MODEbits.LPBACK = 0; // Disable loopback
    U2MODEbits.ABAUD = 0; // Disable autobaud
    U2MODEbits.URXINV = 0; // Normal operation (high is idle)
    U2MODEbits.PDSEL = 0; // No parity 8 bit
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2MODEbits.BRGH = 0; // Low speed mode

    // U1STA Register
    // ==============
    U2STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U2STAbits.UTXISEL1 = 0; // for the DMA	
    U2STAbits.URXISEL = 0; // RX interrupt when a char is in
    U2STAbits.OERR = 0; // clear overun error

    // U1BRG Register
    // ==============
    U2BRG = LOG_UBRG; // Set the baud rate for 115,200

    // Initialize the Interrupt  
    // ========================
    IPC7bits.U2RXIP = 7; // Interrupt priority 7  
    IFS1bits.U2RXIF = 0; // Clear the interrupt flag
    IEC1bits.U2RXIE = 1; // Enable interrupts

    // Enable the port;
    U2MODEbits.UARTEN = 1; // Enable the port	
    U2STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U2EIE = 1;

}

void gsRead(unsigned char* gsChunk) {
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uartBufferIn), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    gsChunk[0] = (tmpLen > MAXSPI) ? MAXSPI : tmpLen;

    // read the data 
    for (i = 1; i <= gsChunk[0]; i += 1) {
        gsChunk[i] = readFront(uartBufferIn);
    }

    gsChunk[MAXSPI + 1] = 1;
}

/* Called at 50 Hz by Simulink, and called sendTelemtry dependent on rate parameter. */
#ifdef MAVLINK_TELEMETRY_RATE
void prepareTelemetryMavlink(unsigned char* dataOut) {

    static uint32_t ticks = 0;
    float telemetryRate = mlParamInterface.param[PAR_RATE_TELEMETRY];
    uint32_t ticksBeforeSend = (telemetryRate >= MAVLINK_TELEMETRY_RATE || telemetryRate < 1.0f)?
            0
            :
            (uint32_t)(MAVLINK_TELEMETRY_RATE - telemetryRate + 1);

    // Send telemetry if we've waited enough ticks or if desired rate is invalid

    if (ticks >= ticksBeforeSend) {
        sendTelemetryMavlink(dataOut);
        ticks = 0;
    }
    else {
        ticks++;
    }
}
#else
void prepareTelemetryMavlink(unsigned char* dataOut) {
    sendTelemetryMavlink(dataOut);
}
#endif

/* Called by Simulink at 50 Hz. */
void sendTelemetryMavlink(unsigned char* dataOut) {

    // Generic message container used to pack the messages
    mavlink_message_t msg;

    // Generic buffer used to hold data to be streamed via serial port
    //uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Cycles from 1 to 10 to decide which 
    // message's turn is to be sent
    static uint8_t sampleTelemetry = 1;

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;//, paramDelay = 0;

    // String used to send text messages to QGC console
    char vr_message[50];

    memset(&msg, 0, sizeof (mavlink_message_t));

    switch (sampleTelemetry) {
        case 1: // GPS, Heartbeat and Passthrough if necessary
            // Pack the GPS message
            mavlink_msg_gps_raw_int_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                mlGpsData.time_usec,
                mlGpsData.fix_type,
                mlGpsData.lat,
                mlGpsData.lon,
                mlGpsData.alt,
                mlGpsData.eph,
                0.0, // ephv
                mlGpsData.vel,
                mlGpsData.cog,
                0 // sattelites_visible
                );

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg to pack a new variable
            memset(&msg, 0, sizeof (mavlink_message_t));

            // Pack the Heartbeat message
            mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                MAV_TYPE_FIXED_WING,
                MAV_AUTOPILOT_SLUGS,
                mlHeartbeatLocal.base_mode,
                mlHeartbeatLocal.custom_mode,
                mlHeartbeatLocal.system_status//,
                //mlHeartbeatLocal.mavlink_version
                );

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // If passthrough was requested (never used)
            if (mlPending.pt == TRUE) {
                // clear the message
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_ctrl_srfc_pt_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    GS_SYSTEMID,
                    mlPassthrough.bitfieldPt);
                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                // Clear the flag
                mlPending.pt = FALSE;
            }

            break; //case 1
            
        case 2: // GPS Date Time, diagnostic, air data, 
            mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsDateTime);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_diagnostic_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDiagnosticData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlAirData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 2

        case 3: // data log, ping, vfr_hud
            mavlink_msg_data_log_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDataLog);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (mlPending.ping == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_ping_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    mlPing.seq,
                    SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    mlRawImuData.time_usec);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
            }


            memset(&msg, 0, sizeof (mavlink_message_t));
            // Hud data for primary flight display
            mavlink_msg_vfr_hud_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                mlNavigation.u_m,               // air speed (m/s)
                (float)mlGpsData.vel * 0.01f,   // ground speed (m/s)
                //(float)(mlAttitudeData.yaw + M_PI)*(180.0f/M_PI), // heading from 0 to 360 (deg)
                0.0f,
                0,                              // throttle from 0 to 100 (percent)
                mlLocalPositionData.z,          // altitude (m)
                mlLocalPositionData.vz         // climb rate (m/s)
                );

            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            break; // case 3

        case 4: // navigation, cpu load, sensor diag

            mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNavigation);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlCpuLoadData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

             // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorDiag);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 4

        case 5: // Raw IMU, Parameter Interface
            mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawImuData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (!mlPending.piTransaction) break;

            // memset(vr_message,0,sizeof(vr_message));
            // 	sprintf(vr_message, "P = %d, S=%d ", mlPending.piCurrentParamInTransaction, mlPending.piProtState);	
            // 	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
            // 				
            switch (mlPending.piProtState) {

                case PI_SEND_ALL_PARAM:
                    if (mlPending.piCurrentParamInTransaction < PAR_PARAM_COUNT) {

                        mavlink_msg_param_value_pack(SLUGS_SYSTEMID,
                            SLUGS_COMPID,
                            &msg,
                            mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                            mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                            MAV_PARAM_TYPE_REAL32, // TODO: make sure this is correct later on
                            PAR_PARAM_COUNT,
                            mlPending.piCurrentParamInTransaction);

                        mlPending.piCurrentParamInTransaction++;

                        // Copy the message to the send buffer
                        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                    } else {
                        mlPending.piProtState = PI_IDLE;
                        mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
                        mlPending.piTransaction = 0;
                        mlPending.piBackToList = 0;
                        mlPending.piQIdx = -1;
                    }
                    break;

                case PI_SEND_ONE_PARAM:

                    // memset(vr_message,0,sizeof(vr_message));
                    // 			 		sprintf(vr_message, "b2l = %d, ix = %d, rtx = %d", mlPending.piBackToList, mlPending.piQIdx, mlPending.piQueue[mlPending.piQIdx]);	
                    // 			 		bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
                    // 						

                    if (!mlPending.piBackToList) { // if this is just a single pm read, i.e. not a retransmission
                        mavlink_msg_param_value_pack(SLUGS_SYSTEMID,
                            SLUGS_COMPID,
                            &msg,
                            mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                            mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                            MAV_PARAM_TYPE_REAL32,
                            PAR_PARAM_COUNT,
                            mlPending.piCurrentParamInTransaction);

                        // Copy the message to the send buffer
                        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


                        mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
                        mlPending.piProtState = PI_IDLE;
                        mlPending.piTransaction = 0;
                    } else { // you will need to go back to send all

                        if (mlPending.piQIdx < 0) { // if you've sent all the requests, then go back to list
                            mlPending.piProtState = PI_SEND_ALL_PARAM;
                            mlPending.piBackToList = 0;

                        } else { // send the requests
                            mavlink_msg_param_value_pack(SLUGS_SYSTEMID,
                                SLUGS_COMPID,
                                &msg,
                                mlParamInterface.param_name[mlPending.piQueue[mlPending.piQIdx]],
                                mlParamInterface.param[mlPending.piQueue[mlPending.piQIdx]],
                                MAV_PARAM_TYPE_REAL32,
                                PAR_PARAM_COUNT,
                                mlPending.piQueue[mlPending.piQIdx]);


                            // Copy the message to the send buffer
                            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                            // decrement the queue index
                            mlPending.piQIdx--;
                        } // QIdx < 0			      
                    }// !backToList

                    break;

            }// switch PI state machine


            break; // case 5

        case 6: // Local Position, System Status, GPS Status

            mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlLocalPositionData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sys_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSystemStatus);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            memset(&msg, 0, sizeof (mavlink_message_t));
#if USE_NMEA
            mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsStatus);
#else
            mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNovatelStatus);
#endif
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 6

        case 7: // PWM Commands, Biases, slugs action

            mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPwmCommands);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // if in HIL Mode, report PWM commands to Sensor MCU so they
            // are sent to the 6DOF model in Simulink
            if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                addMessageToSpiOut(&msg);
            }

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorBiasData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            //
            if (mlPending.command != FALSE) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_command_long_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlCommand);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.command = 0;
            }

            break; // case 7

        case 8:// Wp Protocol state machine, raw Pressure
            // This block goes via SPI to the sensor DSC so it does not count
            // towards the Bandwith budget
            if (mlPending.spiSendGSLocation) {
                mavlink_msg_set_gps_global_origin_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    //GS_COMPID,
                    GS_SYSTEMID,
                    mlWpValues.lat[MAX_NUM_WPS - 1],
                    mlWpValues.lon[MAX_NUM_WPS - 1],
                    mlWpValues.alt[MAX_NUM_WPS - 1]);

                addMessageToSpiOut(&msg);

                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "Sensor DSC Reboot.");
                bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);

                mlPending.spiSendGSLocation = 0;
            }

            // end of SPI block

            if (sw_debug == 1) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "hla = %2.4f, hlo =%2.4f  hh = %2.2f", (double)mlWpValues.lat[MAX_NUM_WPS - 1], (double)mlWpValues.lon[MAX_NUM_WPS - 1], (double)mlWpValues.alt[MAX_NUM_WPS - 1]);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
                sw_debug = 0;
            }
            // 
            if (sw_debug == 4) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%d %d %d %d %d %d %d %d %d %d", sw_temp[0], sw_temp[1], sw_temp[2], sw_temp[3], sw_temp[4], sw_temp[5], sw_temp[6], sw_temp[7], sw_temp[8], sw_temp[9]);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
                sw_debug = 0;

            }
            // 						
            // 			if (sw_debug == 3){
            // 				memset(vr_message,0,sizeof(vr_message));
            // 				sprintf(vr_message, "Mode = %d", mlSystemStatus.mode);	
            // 				bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);	
            // 				sw_debug = 0;		
            // 			}


            if (mlPending.wpProtState == WP_PROT_TX_WP) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%d: y =%2.2f x =%2.2f z =%2.2f o =%d  t =%d", mlPending.wpCurrentWpInTransaction, (double)mlWpValues.lat[mlPending.wpCurrentWpInTransaction],
                    (double)mlWpValues.lon[mlPending.wpCurrentWpInTransaction],
                    (double)mlWpValues.alt[mlPending.wpCurrentWpInTransaction],
                    mlWpValues.orbit[mlPending.wpCurrentWpInTransaction],
                    mlWpValues.type[mlPending.wpCurrentWpInTransaction]);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
            }


            if (mlPending.wpProtState == WP_PROT_GETTING_WP_IDLE) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "com = %d, tb =%2.2f ta = %2.2f", sw_intTemp, (double)fl_temp1, (double)fl_temp2);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);

            }

            // Current mission item (1 off indexing issue in qgc vs et)
            if (mlPending.wpSendCurrent) {
                memset(&msg, 0, sizeof (mavlink_message_t));
                mavlink_msg_mission_current_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg, ((uint16_t)mlNavigation.toWP) - 1);
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
                mlPending.wpSendCurrent = FALSE;
            }


            // Raw pressure
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawPressureData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (!mlPending.wpTransaction) break;

            switch (mlPending.wpProtState) {

                case WP_PROT_LIST_REQUESTED:
                    // clear the msg
                    memset(&msg, 0, sizeof (mavlink_message_t));

                    //mlPending.statustext++;
                    //mlStatustext.severity = MAV_SEVERITY_INFO;
                    //strncpy(mlStatustext.text, "Got mission list request. Sending count...", 49);

                    mavlink_msg_mission_count_pack(SLUGS_SYSTEMID,
                        MAV_COMP_ID_MISSIONPLANNER,
                        &msg,
                        GS_SYSTEMID,
                        GS_COMPID,
                        mlWpValues.wpCount);

                    // Copy the message to the send buffer
                    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                    // Change the state machine state
                    mlPending.wpProtState = WP_PROT_NUM_SENT;

                    // Reset the timeout
                    mlPending.wpTimeOut = 0;
                    break;

                case WP_PROT_GETTING_WP_IDLE:
                    if (mlPending.wpCurrentWpInTransaction < mlPending.wpTotalWps) {

                        // clear the msg
                        memset(&msg, 0, sizeof (mavlink_message_t));

                        mavlink_msg_mission_request_pack(SLUGS_SYSTEMID,
                            MAV_COMP_ID_MISSIONPLANNER,
                            &msg,
                            GS_SYSTEMID,
                            GS_COMPID,
                            mlPending.wpCurrentWpInTransaction++);

                        // Copy the message to the send buffer
                        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                        // Change the state machine state
                        mlPending.wpProtState = WP_PROT_RX_WP;

                    } else {
                        // clear the msg
                        memset(&msg, 0, sizeof (mavlink_message_t));

                        mavlink_msg_mission_ack_pack(SLUGS_SYSTEMID,
                            MAV_COMP_ID_MISSIONPLANNER,
                            &msg,
                            GS_SYSTEMID,
                            GS_COMPID,
                            MAV_MISSION_ACCEPTED); // 0 is success

                        // Copy the message to the send buffer
                        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                        // Update the waypoint count
                        mlWpValues.wpCount = mlPending.wpTotalWps;

                        // End the transaction
                        mlPending.wpTransaction = 0;
                        mlPending.wpProtState = WP_PROT_IDLE;
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTotalWps = 0;
                        mlPending.wpSendCurrent = TRUE; // send current waypoint index

                        // put zeros in the rest of the waypoints;
                        clearWaypointsFrom(mlWpValues.wpCount);

                    }

                    // Reset the timeout
                    mlPending.wpTimeOut = 0;
                    break;

                case WP_PROT_TX_WP:
                    memset(&msg, 0, sizeof (mavlink_message_t));

                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    strncpy(mlStatustext.text, "Sending waypoint.", 25);


                    // Send WP
                    mavlink_msg_mission_item_pack(SLUGS_SYSTEMID,
                        MAV_COMP_ID_MISSIONPLANNER,
                        &msg,
                        GS_SYSTEMID,
                        GS_COMPID,
                        mlPending.wpCurrentWpInTransaction,
                        MAV_FRAME_GLOBAL,
                        mlWpValues.type[mlPending.wpCurrentWpInTransaction],
                        0, // not current
                        1, // autocontinue
                        0.0, // Param 1 not used
                        0.0, // Param 2 not used
                        (float) mlWpValues.orbit[mlPending.wpCurrentWpInTransaction],
                        0.0, // Param 4 not used
                        mlWpValues.lat[mlPending.wpCurrentWpInTransaction],
                        mlWpValues.lon[mlPending.wpCurrentWpInTransaction],
                        mlWpValues.alt[mlPending.wpCurrentWpInTransaction]); // always autocontinue

                    // Copy the message to the send buffer
                    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                    // Switch the state waiting for the next request
                    // Change the state machine state
                    mlPending.wpProtState = WP_PROT_SENDING_WP_IDLE;

                    // Reset the timeout
                    mlPending.wpTimeOut = 0;
                    break;

            } // switch wpProtState

            mlPending.wpTimeOut++;

            // if Timed out reset the state machine and send an error
            if (mlPending.wpTimeOut > PROTOCOL_TIMEOUT_TICKS) {
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_mission_ack_pack(SLUGS_SYSTEMID,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    1); // 1 is failure

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                // reset the state machine
                mlPending.wpTransaction = 0;
                mlPending.wpProtState = WP_PROT_IDLE;
                mlPending.wpCurrentWpInTransaction = 0;
                mlPending.wpTimeOut = 0;
                mlPending.wpTotalWps = 0;
            }

            break; // case 8

        case 9: // Action Ack, Pilot Console, Mid Level Commands, boot

            mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPilotConsoleData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (mlPending.commandAck) {
                mavlink_msg_command_ack_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlCommandAck);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.commandAck = FALSE;
            }


            // if there is a pending request for the Mid Level Commands
            if (mlPending.midLvlCmds == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_mid_lvl_cmds_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    GS_SYSTEMID,
                    mlMidLevelCommands.hCommand,
                    mlMidLevelCommands.uCommand,
                    mlMidLevelCommands.rCommand);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                // clear the flag
                mlPending.midLvlCmds = 0;
            }

            if (mlPending.isrLoc == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_isr_location_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlISR);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.isrLoc = 0;
            }

            // if the boot message is set then transmit it urgently
            if (mlBoot.version == 1) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%s DSC Reboot.", "Control");
                bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);
                mlBoot.version = 0;
            }

            break; // case 9

        case 10: // Filtered data, PTZ data, VI Sensor
            mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlFilteredData);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_ptz_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPtzStatus);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_volt_sensor_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlVISensor);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 10

    } // Switch

    // Send additional statustext messages
    if (mlPending.statustext > 0) {
        memset(&msg, 0, sizeof (mavlink_message_t));


        mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        mlStatustext.severity,
        mlStatustext.text);

        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.statustext = 0;
    }



    memset(&msg, 0, sizeof (mavlink_message_t));

    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAttitudeRotated);
    // Copy the message to the send buffer	
    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    sampleTelemetry = (sampleTelemetry >= 10) ? 1 : sampleTelemetry + 1;

}

void protDecodeMavlink(uint8_t* dataIn) {

    uint8_t i, indx, writeSuccess, commChannel = dataIn[MAXSPI + 1];
    uint32_t temp;
    mavlink_param_set_t set;
    //mavlink_set_nav_mode_t mode;



    //static int16_t packet_drops = 0;
    mavlink_message_t msg;
    mavlink_status_t status;

    // increment the age of heartbeat
    mlPending.heartbeatAge++;

    for (i = 1; i <= dataIn[0]; i++) {

        // Try to get a new message
        if (mavlink_parse_char(commChannel, dataIn[i], &msg, &status)) {

            // Handle message
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&msg, &mlHeartbeat);
                    // Reset the heartbeat
                    mlPending.heartbeatAge = 0;
                    break;

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    mavlink_msg_gps_raw_int_decode(&msg, &mlGpsData);
                    //updateGpsPosition(); // scales raw ints to floats
                    break;

                case MAVLINK_MSG_ID_CPU_LOAD:
                    mavlink_msg_cpu_load_decode(&msg, &mlCpuLoadData);
                    // Copy battery voltage to system status message
                    mlSystemStatus.voltage_battery = mlCpuLoadData.batVolt;
                    break;

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msg, &mlLocalPositionData);
                    break;

                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    mavlink_msg_scaled_pressure_decode(&msg, &mlAirData);
                    break;

                case MAVLINK_MSG_ID_BOOT:
                    temp = mavlink_msg_boot_get_version(&msg);

                    if (temp) {
                        mlPending.spiSendGSLocation = 1;
                    }
                    /*
                    if (temp != 0 && mlBoot.version != 0) {
                        mlBoot.version += temp;
                    } else if (mlBoot.version == 0) {
                        mavlink_msg_boot_decode(&msg, &mlBoot);
                    }
                     */

                    break;

                case MAVLINK_MSG_ID_SENSOR_BIAS:
                    mavlink_msg_sensor_bias_decode(&msg, &mlSensorBiasData);
                    break;

                case MAVLINK_MSG_ID_DIAGNOSTIC:
                    //mavlink_msg_diagnostic_decode(&msg, &mlDiagnosticData);
                    break;

                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    mavlink_msg_rc_channels_raw_decode(&msg, &mlPilotConsoleData);
                    break;

                case MAVLINK_MSG_ID_SCALED_IMU:
                    mavlink_msg_scaled_imu_decode(&msg, &mlFilteredData);
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

#if USE_NMEA
                case MAVLINK_MSG_ID_STATUS_GPS:
                    mavlink_msg_status_gps_decode(&msg, &mlGpsStatus);
                    break;
#else
                case MAVLINK_MSG_ID_NOVATEL_DIAG:
                    mavlink_msg_novatel_diag_decode(&msg, &mlNovatelStatus);
                    break;
#endif

                case MAVLINK_MSG_ID_SENSOR_DIAG:
                    mavlink_msg_sensor_diag_decode(&msg, &mlSensorDiag );
                    break;
                    //  End of Sensor MCU exclusive Messages
                    // =====================================

                case MAVLINK_MSG_ID_SET_MODE:
                {
                    mavlink_set_mode_t mlSetMode;
                    mavlink_msg_set_mode_decode(&msg, &mlSetMode );
                    // Can set custom_mode (previously nav_mode) and or base_mode
                    if (mlSetMode.base_mode != 0) {

                        if (!hasMode(mlHeartbeatLocal.base_mode,MAV_MODE_FLAG_HIL_ENABLED)
                            && hasMode(mlSetMode.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                            // Turned HIL on
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Turning on HIL mode.", 49);
                        }
                        else if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)
                            && !hasMode(mlSetMode.base_mode,MAV_MODE_FLAG_HIL_ENABLED)) {
                            // Turned HIL off
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Turning off HIL mode.", 49);
                        }
                        mlHeartbeatLocal.base_mode = mlSetMode.base_mode;

                    }
                    if (mlSetMode.custom_mode != SLUGS_MODE_NONE) {
                        /*
                        BOOL badMode = FALSE;
                        // Note: some modes are unused. consider pruning
                        switch (mlSetMode.custom_mode) {
                            case SLUGS_MODE_LIFTOFF:
                                break;
                            case SLUGS_MODE_PASSTHROUGH:
                                mlPending.pt = TRUE; // want passthrough mode
                                break;
                            case SLUGS_MODE_WAYPOINT:
                                break;
                            case SLUGS_MODE_MID_LEVEL:
                                break;
                            case SLUGS_MODE_RETURNING:
                                break;
                            case SLUGS_MODE_LANDING:
                                break;
                            case SLUGS_MODE_LOST:
                                break;
                            case SLUGS_MODE_SELECTIVE_PASSTHROUGH:
                                break;
                            case SLUGS_MODE_ISR:
                                break;
                            case SLUGS_MODE_LINE_PATROL:
                                break;
                            default:
                                badMode = TRUE;
                                // Send an error message
                                mlPending.statustext++;

                                mlStatustext.severity = MAV_SEVERITY_ERROR;
                                sprintf(mlStatustext.text, "Unknown slugs mode sent: %X.", mlSetMode.custom_mode)
                                strncpy(mlStatustext.text,"Failed to clear waypoints from EEPROM.", 49);

                        }
                        // Set new slugs mode if valid
                        if (!badMode)
                        */
                        //lastNavigationMode = mlHeartbeatLocal.custom_mode;
                        mlHeartbeatLocal.custom_mode = mlSetMode.custom_mode;
                    }

                    break;
                }
                case MAVLINK_MSG_ID_MID_LVL_CMDS:
                {
                    mavlink_msg_mid_lvl_cmds_decode(&msg, &mlMidLevelCommands);

                    // Report the Change

                    mlPending.statustext++;

                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    strncpy(mlStatustext.text, "Mid level flight parameters received.", 49);
                    /*
                    mlAction.actionId = SLUGS_ACTION_MLC_CHANGE;
                    mlAction.actionVal = SLUGS_ACTION_SUCCESS;
                     */

                    mlPending.midLvlCmds = 1;

                    break;
                }
                // Slugs camera orders not used, but should be moved to command_long
                /*
                case MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER:
                    mavlink_msg_slugs_camera_order_decode(&msg, &mlCameraOrder);

                    // Report the Change
                    mlPending.commandAck++;
                    mlCommandAck.command = MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER;
                    mlCommandAck.result = MAV_RESULT_ACCEPTED;

                    break;

                case MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA:
                    mavlink_msg_slugs_configuration_camera_decode(&msg, &mlCameraConfig);

                    // Report the Change
                    mlPending.commandAck++;
                    mlCommandAck.command = MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA;
                    mlCommandAck.result = MAV_RESULT_ACCEPTED;

                    break;
                */
                case MAVLINK_MSG_ID_MISSION_COUNT:

                    if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)) {


                        mavlink_msg_mission_count_decode(&msg, &mlWpCount);

                        // Start the transaction
                        mlPending.wpTransaction = 1;

                        // change the state	
                        mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                        // reset the rest of the state machine
                        mlPending.wpTotalWps = mlWpCount.count;
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:

                    // if there is no transaction going on
                    if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)) {
                        // Start the transaction
                        mlPending.wpTransaction = 1;

                        // change the state
                        mlPending.wpProtState = WP_PROT_LIST_REQUESTED;



                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;
                    }
                    break;

                case MAVLINK_MSG_ID_MISSION_REQUEST:
                    mavlink_msg_mission_request_decode(&msg, &mlWpRequest);

                    if (mlPending.wpTransaction && (mlWpRequest.seq < mlWpValues.wpCount)) {
                        // change the state
                        mlPending.wpProtState = WP_PROT_TX_WP;

                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = mlWpRequest.seq;
                        mlPending.wpTimeOut = 0;
                    } else {
                        // TODO: put here a report for a single WP, i.e. not inside a transaction
                    }
                    break;

                case MAVLINK_MSG_ID_MISSION_ACK:
                    mavlink_msg_mission_ack_decode(&msg, &mlWpAck);

                    if (mlPending.wpTransaction) {
                        // End the transaction
                        mlPending.wpTransaction = 0;

                        // change the state
                        mlPending.wpProtState = WP_PROT_IDLE;

                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;

                        // send current waypoint index
                        mlPending.wpSendCurrent = TRUE;
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_ITEM:
                    writeSuccess = SUCCESS;
                    mavlink_msg_mission_item_decode(&msg, &mlSingleWp);

                    if (mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_RX_WP)) {
                        mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                    }

                    indx = (uint8_t) mlSingleWp.seq;

                    mlWpValues.lat[indx] = mlSingleWp.x;
                    mlWpValues.lon[indx] = mlSingleWp.y;
                    mlWpValues.alt[indx] = mlSingleWp.z;

                    mlWpValues.type[indx] = mlSingleWp.command;

                    mlWpValues.orbit[indx] = (uint16_t) mlSingleWp.param3;

                    // Record the data in EEPROM
                    writeSuccess = storeWaypointInEeprom(&mlSingleWp);

                    // Set the flag of Aknowledge for the AKN Message
                    // if the write was not successful
                    if (writeSuccess != SUCCESS) {
                        mlPending.wpAck++;

                        //mlWpAck.target_system =
                        mlWpAck.target_component = MAV_COMP_ID_MISSIONPLANNER;
                        mlWpAck.type = MAV_MISSION_ERROR;
                        /*
                        mlAction.actionId = SLUGS_ACTION_EEPROM;
                        mlAction.actionVal = SLUGS_ACTION_FAIL;
                        */
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:

                    writeSuccess = SUCCESS;

                    // clear the WP values in memory;
                    memset(&mlWpValues, 0, sizeof (mavlink_mission_item_values_t));

                    writeSuccess = clearWaypointsFrom(0);

                    // Set the flag of Aknowledge fail
                    // if the write was unsuccessful
                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to clear waypoints from EEPROM.", 49);

                    }
                    /*
                        mlCommandAck.command = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
                        mlCommandAck.result = MAV_RESULT_FAILED;
                        
                        mlAction.actionId = SLUGS_ACTION_EEPROM;
                        mlAction.actionVal = SLUGS_ACTION_FAIL;
                         
                    }
                     * */

                    // Update the waypoint count
                    mlWpValues.wpCount = 0;

                    // Set the state machine ready to send the WP akn
                    mlPending.wpCurrentWpInTransaction = 0;
                    mlPending.wpTotalWps = 0;
                    mlPending.wpTransaction = 1;
                    mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                    break;

                case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                    writeSuccess = SUCCESS;

                    memset(&mlSingleWp, 0, sizeof (mavlink_mission_item_t));

                    mavlink_msg_set_gps_global_origin_decode(&msg, &mlGSLocation);


                    addMessageToSpiOut(&msg);   

                    mlSingleWp.x = (float) (mlGSLocation.latitude);
                    mlSingleWp.y = (float) (mlGSLocation.longitude);
                    mlSingleWp.z = (float) (mlGSLocation.altitude);

                    indx = (uint8_t) MAX_NUM_WPS - 1;

                    mlWpValues.lat[indx] = mlSingleWp.x;
                    mlWpValues.lon[indx] = mlSingleWp.y;
                    mlWpValues.alt[indx] = mlSingleWp.z;
                    mlWpValues.type[indx] = MAV_CMD_NAV_LAND;
                    mlWpValues.orbit[indx] = 0;

                    // Record the data in EEPROM
                    writeSuccess = storeWaypointInEeprom(&mlSingleWp);

                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to write origin to EEPROM.", 49);
                    }
                    else {

                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_INFO;
                        strncpy(mlStatustext.text, "Control DSC GPS origin set.", 49);
                    }

                    // Set the flag of Aknowledge for the AKN Message
                    // if the write was not successful
                    /*
                    if (writeSuccess != SUCCESS) {
                        
                        mlAction.actionId = SLUGS_ACTION_EEPROM;
                        mlAction.actionVal = SLUGS_ACTION_FAIL;
                         
                    }
                    */
                    break;

                case MAVLINK_MSG_ID_CTRL_SRFC_PT:
                    mavlink_msg_ctrl_srfc_pt_decode(&msg, &mlPassthrough);

                    // Report the Change // NOTE: look into this
                    /*
                    mlPending.slugsAction++;
                    mlAction.actionId = SLUGS_ACTION_PT_CHANGE;
                    mlAction.actionVal = SLUGS_ACTION_SUCCESS;
                    */

                    mlPending.statustext++;

                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text, "Control surface passthrough: 0x%X", mlPassthrough.bitfieldPt);
                    break;

                case MAVLINK_MSG_ID_PING:
                    mavlink_msg_ping_decode(&msg, &mlPing);

                    mlPending.ping = 1;
                    break;


                case MAVLINK_MSG_ID_STATUSTEXT:
                    // Received status message from the sensor MCU -- forward to GS
                    mavlink_msg_statustext_decode(&msg, &mlStatustext);
                    mlPending.statustext++;
                    break;


                case MAVLINK_MSG_ID_COMMAND_ACK:
                    // Received a command ack from the sensor MCU
                    mavlink_msg_command_ack_decode(&msg, &mlCommandAck);

                    switch (mlCommandAck.command) {
                        case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                            // since this case has no specific action required on this end
                            // then just increment slugs action and report it
                            //mlPending.slugsAction++;
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Sensor DSC GPS origin set.", 49);
                            break;
                    }

                    break;

                // Replaced by COMMAND_ACK above
                // Action stuff was here

                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_msg_command_long_decode(&msg, &mlCommand);

                    switch (mlCommand.command) {
                        // TODO: Only handle this in PREFLIGHT mode
                        case MAV_CMD_PREFLIGHT_STORAGE:
                            writeSuccess = FAILURE;
                            // Parameter storage
                            if (mlCommand.param1 == 0.0f) { // read
                                memset(&(mlParamInterface.param[0]), 0, sizeof (float) *PAR_PARAM_COUNT);

                                // Comment out MAVLINK_TELEMETRY_RATE in apDefinitions.h to disable this feature
#ifdef MAVLINK_TELEMETRY_RATE
    // Default parameter
    mlParamInterface.param[PAR_RATE_TELEMETRY] = MAVLINK_TELEMETRY_RATE; // (Hz)
#endif

                                writeSuccess = readParamsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write

                                writeSuccess = storeAllParamsInEeprom();
                            }
                            // Waypoint storage (only handle either params or waypoints)
                            // TODO look into implementing this (again?)
                            else if (mlCommand.param2 == 0.0f) { // read
                            }
                            else if (mlCommand.param2 == 1.0f) { // write
                            }

                            mlPending.commandAck = TRUE;

                            mlCommandAck.command = MAV_CMD_PREFLIGHT_STORAGE;
                            mlCommandAck.result = (writeSuccess == SUCCESS)?
                                MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

                            break;
                        // Store or read mid-level commands to/from eeprom
                        case MAV_CMD_MIDLEVEL_STORAGE:
                            writeSuccess = FAILURE;
                            if (mlCommand.param1 == 0.0f) { // read
                                mlMidLevelCommands.hCommand = 0.0f;
                                mlMidLevelCommands.rCommand = 0.0f;
                                mlMidLevelCommands.uCommand = 0.0f;
                                writeSuccess = readMidLevelCommandsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write
                                writeSuccess = storeMidLevelCommandsInEeprom();
                                mlPending.statustext++;

                                mlStatustext.severity = MAV_SEVERITY_INFO;
                                strncpy(mlStatustext.text, "Wrote mid-level commands to EEPROM.", 49);
                            }

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_MIDLEVEL_STORAGE;
                            mlCommandAck.result = (writeSuccess == SUCCESS)?
                                MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

                            break;
                        // HIL Stuff Moved to set_mode
                        case MAV_CMD_RETURN_TO_BASE:
                            mlRTB.rtb = TRUE;
                            mlRTB.track_mobile = mlCommand.param1;
                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_RETURN_TO_BASE;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
                            break;
                        case MAV_CMD_TURN_LIGHT:
                            mlLights.state = mlCommand.param2;
                            mlLights.type = mlCommand.param1;

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_TURN_LIGHT;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
                            break;
                        case MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            mlPending.midLvlCmds = 1;

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_GET_MID_LEVEL_COMMANDS;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
                            break;

                    } // switch COMMAND_LONG

                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    mlPending.piTransaction = 1;
                    mlPending.piProtState = PI_SEND_ALL_PARAM;
                    mlPending.piCurrentParamInTransaction = 0;
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    // If it was in the middle of a list transmission or there is already a param enqueued
                    mlPending.piTransaction = 1;
                    switch (mlPending.piProtState) {
                        case PI_IDLE:
                            mlPending.piBackToList = 0; // no need to go back
                            mlPending.piQIdx = -1; // no Index
                            mlPending.piCurrentParamInTransaction = mavlink_msg_param_request_read_get_param_index(&msg); // assign directly
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;

                        case PI_SEND_ALL_PARAM:
                            mlPending.piBackToList = 1; // mark to go back
                            mlPending.piQIdx++; // done like this because when empty index = -1
                            mlPending.piQueue[mlPending.piQIdx] = mavlink_msg_param_request_read_get_param_index(&msg); // put in in queue
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;

                        case PI_SEND_ONE_PARAM:
                            if (mlPending.piBackToList) {
                                mlPending.piQIdx++; // done like this because when empty index = -1
                                mlPending.piQueue[mlPending.piQIdx] = mavlink_msg_param_request_read_get_param_index(&msg); // put in in queue				  									  		
                            }
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;
                    }
                    break;

                case MAVLINK_MSG_ID_PARAM_SET:
                    mavlink_msg_param_set_decode(&msg, &set);

                    if ((uint8_t) set.target_system == (uint8_t) SLUGS_SYSTEMID &&
                        (uint8_t) set.target_component == (uint8_t) SLUGS_COMPID) {


                        char* key = (char*) set.param_id;
                        uint8_t i, j;
                        uint8_t match;
                        for (i = 0; i < PAR_PARAM_COUNT; i++) {
                            match = 1;
                            for (j = 0; j < SLUGS_PARAM_NAME_LENGTH; j++) {
                                // Compare
                                if (((char) (mlParamInterface.param_name[i][j]))
                                    != (char) (key[j])) {
                                    match = 0;
                                } // if

                                // End matching if null termination is reached
                                if (((char) mlParamInterface.param_name[i][j]) == '\0') {
                                    break;
                                } // if
                            }// for j

                            // Check if matched
                            if (match) {
                                //sw_debug = 1;
                                // Only write and emit changes if there is actually a difference
                                // AND only write if new value is NOT "not-a-number"
                                // AND is NOT infinity

                                if (isFinite(set.param_value)) {

                                    mlParamInterface.param[i] = set.param_value;

                                    // Report back new value
                                    mlPending.piBackToList = 0; // no need to go back
                                    mlPending.piQIdx = -1; // no Index
                                    mlPending.piCurrentParamInTransaction = i; // assign directly
                                    mlPending.piProtState = PI_SEND_ONE_PARAM;
                                    mlPending.piTransaction = 1;

                                } // if different and not nan and not inf
                            } // if match
                        }// for i
                    } // if addressed to this
                    break;

                    // case MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER:
                    // 	mavlink_msg_slugs_camera_order_decode(&msg, &mlCameraOrder); 
                    // break;
                // mav_mode moved into custom_mode of set_mode 
                /*
                case MAVLINK_MSG_ID_SET_NAV_MODE:
                    mavlink_msg_set_nav_mode_decode(&msg, &mode);

                    mlSystemStatus.nav_mode = mode.nav_mode;
                    break;
                */
                // moved to MAV_CMD_RETURN_TO_BASE
                    /*
                case MAVLINK_MSG_ID_SLUGS_RTB:
                    mavlink_msg_slugs_rtb_decode(&msg, &mlRTB);

                    // Report the Change
                    //mlPending.
                    //mlPending.slugsAction++;

                    if (!mlRTB.rtb) {
                        mlAction.actionId = SLUGS_ACTION_RTB_OFF;
                    } else if (!mlRTB.track_mobile) {
                        mlAction.actionId = SLUGS_ACTION_RTB_ON_NO_MOBILE;
                    } else {
                        mlAction.actionId = SLUGS_ACTION_RTB_ON_MOBILE;
                    }
                    mlAction.actionVal = SLUGS_ACTION_SUCCESS;
                     
                    break;
                    */
                case MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION:
                    mavlink_msg_slugs_mobile_location_decode(&msg, &mlMobileLocation);
                    break;


                case MAVLINK_MSG_ID_ISR_LOCATION:
                    mavlink_msg_isr_location_decode(&msg, &mlISR);

                    // Report the Change
                    // Actions removed. TODO: determine reporting for ISR_LOC set
                    /*
                    mlPending.slugsAction++;
                    mlAction.actionId = SLUGS_ACTION_ISR_LOCATION;
                    mlAction.actionVal = SLUGS_ACTION_SUCCESS;
                    */

                    mlPending.statustext++;

                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    strncpy(mlStatustext.text, "ISR position recieved.", 49);

                    mlPending.isrLoc = 1;
                    break;
                // Moved to MAV_CMD_TURN_LIGHT
                    /*
                case MAVLINK_MSG_ID_TURN_LIGHT:
                    mavlink_msg_turn_light_decode(&msg, &mlLights);


                    mlPending.slugsAction++;
                    mlAction.actionId = SLUGS_ACTION_LIGHTS;
                    mlAction.actionVal = SLUGS_ACTION_SUCCESS;

                    break;
                    */
            } // switch	
        } // if

        // Update global packet drops counter
        if (commChannel == 1) {
            mlSystemStatus.errors_comm += status.packet_rx_drop_count;
        }


    }// for  
}

void copyBufferToDMA1(unsigned char size) {
    unsigned char i;
    for (i = 0; i < size; i += 1) {
        BufferB[i] = (unsigned int) readFront(uartBufferOut);
    }
}

void send2GS(unsigned char* protData) {
    unsigned int bufLen, i;

    // add the data to the circular buffer
    for (i = 1; i <= protData[0]; i += 1) {
        writeBack(uartBufferOut, protData[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(uartBufferOut);


    // if the interrupt catched up with the circularBuffer
    // and new data was added then turn on the DMA 
    if (!(DMA1CONbits.CHEN) && (bufLen > 0)) {
        // Configure the bytes to send
        DMA1CNT = bufLen <= (MAXSEND - 1) ? bufLen - 1 : MAXSEND - 1;
        // copy the buffer to the DMA channel outgoing buffer
        copyBufferToDMA1((unsigned char) DMA1CNT + 1);
        // Enable the DMA
        DMA1CONbits.CHEN = 1;
        // Init the transmission
        DMA1REQbits.FORCE = 1;
    }

}

// Interrupts

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS0bits.DMA1IF = 0;
}

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

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // If there was an overun error clear it and continue
    if (IFS4bits.U2EIF == 1) {
        IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
    }
}

char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) {
    mavlink_message_t msg;
    unsigned char bytes2Send = 0; // size in bytes of the mavlink packed message (return value)

    mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        severity,
        dbgMessage);

    bytes2Send = mavlink_msg_to_send_buffer((bytesToAdd + positionStart), &msg);

    return bytes2Send;
}

// TODO: This probably needs to move to another file since, strictly speaking it has nothing
//				to do with Mavlink comms.

uint8_t clearWaypointsFrom(uint8_t startingWp) {

    uint8_t writeSuccess = 0;
    uint8_t indx, indexOffset;
    tFloatToChar tempFloat;

    // erase the flash values in EEPROM emulation
    for (indx = startingWp; indx < MAX_NUM_WPS - 1; indx++) {
        // Compute the adecuate index offset
        indexOffset = indx * 8;

        // Clear the data from the EEPROM
        tempFloat.flData = 0.0;
        writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset);
        writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 1);

        tempFloat.flData = 0.0;
        writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset + 2);
        writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 3);

        tempFloat.flData = 0.0;
        writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET + indexOffset + 4);
        writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET + indexOffset + 5);


        writeSuccess += DataEEWrite((unsigned short) 0, WPS_OFFSET + indexOffset + 6);

        writeSuccess += DataEEWrite((unsigned short) 0, WPS_OFFSET + indexOffset + 7);
    }

    return writeSuccess;
}

void addMessageToSpiOut(mavlink_message_t* msg) {
    // mlPending.spiCurrentIndex = 0;

    mlPending.spiTotalData += mavlink_msg_to_send_buffer(&(mlPending.spiToSensor[mlPending.spiTotalData]), msg);

}
