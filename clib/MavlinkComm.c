
#include "MavlinkComm.h"
#include "circBuffer.h"

#include <stdio.h>
<<<<<<< HEAD
#ifdef UNIT_TEST
#include "AUAV_V3_TestMavLink.h"
//#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"

#elif defined  SENSORS_UNIT_TEST
#include "AUAV_V3_TestSensors.h"
#else
#include "AUAV_V3_TestSensors.h"

#endif
=======
#include "AUAV_V3_TestMavlink.h"
>>>>>>> master

uint8_t UartOutBuff[MAVLINK_MAX_PACKET_LEN];
uint8_t Uart4OutBuff[MAVLINK_MAX_PACKET_LEN];

struct CircBuffer comMavlinkBuffer;
CBRef uartMavlinkInBuffer;
uint8_T DatafromGSmavlink[MAXINLEN+2];
mavlink_pending_requests_t mlPending;
mavlink_heartbeat_t mlHeartbeat;
mavlink_mission_count_t mlWpCount;
mavlink_mission_request_t mlWpRequest;
//mavlink_mission_item_values_t mlWpValues; //defined in MavlinkComm.h
mavlink_mission_item_t mlSingleWp;
mavlink_set_gps_global_origin_t mlGSLocation;
mavlink_mission_ack_t mlWpAck;

//pi_struct mlParamInterface;

void uartMavlinkBufferInit (void){
#if (WIN != 1)
  _U1RXIP = 5;                         /* Rx Interrupt priority set to 1 */
  _U1RXIF = 0;
  _U1RXIE = 1;                         /* Enable Interrupt */
  /* Configure Remappables Pins */
   RPINR18 = 0x62;
#endif
  uartMavlinkInBuffer = (struct CircBuffer*) &comMavlinkBuffer;
  newCircBuffer(uartMavlinkInBuffer);
}
#if (WIN != 1)
void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)
{
  //
        _U1RXIF = 0;
    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U1STAbits.URXDA == 1) {
        writeBack(uartMavlinkInBuffer, (unsigned char) U1RXREG);
    }

    // If there was an overun error clear it and continue
    if (U1STAbits.OERR == 1) {
        _U1RXIF = 0;
        U1STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS0bits.U1RXIF = 0;

}
#endif
uint8_t isFinite(float s) {
  // By IEEE 754 rule, 2*Inf equals Inf
  return ((s == s) && ((s == 0) || (s != 2*s)));
}

void InitParameterInterface(void)
{
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_P], "PID_AIRSPD_P");
    mlParamInterface.param[PAR_PID_AIRSPEED_P] = 0.949999988079;
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_I], "PID_AIRSPD_I");
    mlParamInterface.param[PAR_PID_AIRSPEED_I] = 0.00700000021607;
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_D], "PID_AIRSPD_D");
    mlParamInterface.param[PAR_PID_AIRSPEED_D] = 0.20000000298;

    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_P], "PID_PIT_FO_P");
        mlParamInterface.param[PAR_PID_PITCH_FO_P] = -0.20000000298;
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_I], "PID_PIT_FO_I");
        mlParamInterface.param[PAR_PID_PITCH_FO_I] = -0.00499999988824;   
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_D], "PID_PIT_FO_D");
        mlParamInterface.param[PAR_PID_PITCH_FO_D] = -0.0599999986589;

    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_P], "PID_ROLL_CO_P");
            mlParamInterface.param[PAR_PID_ROLL_CON_P] = 0.649999976158;
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_I], "PID_ROLL_CO_I");
            mlParamInterface.param[PAR_PID_ROLL_CON_I] = 0.00999999977648;
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_D], "PID_ROLL_CO_D");
            mlParamInterface.param[PAR_PID_ROLL_CON_D] = 0.0599999986589;


    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_P], "PID_HE2PITC_P");
                mlParamInterface.param[PAR_PID_HE_TO_PI_P] = 0.0450000017881;

    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_I], "PID_HE2PITC_I");
                mlParamInterface.param[PAR_PID_HE_TO_PI_I] = 0;

    strcpy(mlParamInterface.param_name[PAR_PID_HEI_ERR_FF], "PID_HERR_FF");
                mlParamInterface.param[PAR_PID_HEI_ERR_FF] = -0.10000000149;
    strcpy(mlParamInterface.param_name[PAR_PID_PITC_DT_FF], "PID_PITC_DT_FF");
                mlParamInterface.param[PAR_PID_PITC_DT_FF] = 0.34999999404;

    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_P], "PID_YAW_DA_P");
                    mlParamInterface.param[PAR_PID_YAW_DAMP_P] = 0.00999999977648;

    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_I], "PID_YAW_DA_I");
                    mlParamInterface.param[PAR_PID_YAW_DAMP_I] = 0;

    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_D], "PID_YAW_DA_D");
                    mlParamInterface.param[PAR_PID_YAW_DAMP_D] = 0;

 strcpy(mlParamInterface.param_name[PAR_NAV_L2_BASE], "NAV_L2_BASE");
                    mlParamInterface.param[PAR_NAV_L2_BASE] =  5;
strcpy(mlParamInterface.param_name[PAR_NAV_PRETURN_K], "NAV_PRETURN_K");
                    mlParamInterface.param[PAR_NAV_PRETURN_K] = 5;	
strcpy(mlParamInterface.param_name[PAR_NAV_SSCOMP_ON], "NAV_SSCOMP_ON");
                    mlParamInterface.param[PAR_NAV_SSCOMP_ON] = 0;

 strcpy(mlParamInterface.param_name[PAR_L1_OMEGA], "L1_OMEGA");
                    mlParamInterface.param[PAR_L1_OMEGA] = 0;                   
strcpy(mlParamInterface.param_name[PAR_L1_M], "L1_M");
                    mlParamInterface.param[PAR_L1_M] = 0; 
strcpy(mlParamInterface.param_name[PAR_L1_GAMMA], "L1_GAMMA");
                    mlParamInterface.param[PAR_L1_GAMMA] = 0;
strcpy(mlParamInterface.param_name[PAR_L1_ON_OFF], "L1_ON_OFF");
                    mlParamInterface.param[PAR_L1_ON_OFF] = 0;
strcpy(mlParamInterface.param_name[PAR_NAV_ISR_FAC], "NAV_ISR_FAC");
                    mlParamInterface.param[PAR_NAV_ISR_FAC] = 5;
strcpy(mlParamInterface.param_name[PAR_PID_RMIX_ON], "PID_RMIX_ON");
                    mlParamInterface.param[PAR_PID_RMIX_ON] = 0;
strcpy(mlParamInterface.param_name[PAR_PID_RMIX_P], "PID_RMIX_P");
                    mlParamInterface.param[PAR_PID_RMIX_P] = 0; 
                    
strcpy(mlParamInterface.param_name[PAR_CONFIG_ROLL_R], "CONFIG_ROLL_R");
    mlParamInterface.param[PAR_CONFIG_ROLL_R] = 0;
    strcpy(mlParamInterface.param_name[PAR_CONFIG_PITCH_R], "CONFIG_PITCH_R");
    mlParamInterface.param[PAR_CONFIG_PITCH_R] = 0;
    strcpy(mlParamInterface.param_name[PAR_CONFIG_YAW_R], "CONFIG_YAW_R");
    mlParamInterface.param[PAR_CONFIG_YAW_R] = 0;                    
    // Populate default mid-level commands
    mlMidLevelCommands.hCommand = 120.0f; // altitude (m)
    mlMidLevelCommands.uCommand = 16.0f; // airspeed (m/s)
    mlMidLevelCommands.rCommand = 0.0f; // turn rate (radians/s)

}

void protDecodeMavlink(void) {

    uint8_t indx, writeSuccess, commChannel = 1;
    mavlink_param_set_t set;
    mavlink_message_t msg;
    mavlink_status_t status;
   // uint8_t* dataIn;
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uartMavlinkInBuffer), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    //DatafromGSmavlink[0] = (tmpLen > MAXINLEN) ? MAXINLEN : tmpLen;

    // read the data
    //for (i = 1; i <= DatafromGSmavlink[0]; i += 1) {
      //mavlink_parse_char(commChannel, readFront(uartBufferInMavlink), &msg, &status);
      //DatafromGSmavlink[i] = readFront(uartMavlinkInBuffer);
    //}
    //dataIn = DatafromGSmavlink;
    // increment the age of heartbeat
    mlPending.heartbeatAge++;

    for (i = 0; i <= tmpLen; i++) {
        // Try to get a new message
       if (mavlink_parse_char(commChannel, readFront(uartMavlinkInBuffer), &msg, &status)) {
                    // Handle message
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&msg, &mlHeartbeat);
                    // Reset the heartbeat
                    mlPending.heartbeatAge = 0;
                    break;
//AM DBG
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
/*
                    // Record the data in EEPROM
                    writeSuccess = storeWaypointInEeprom(&mlSingleWp);

                    // Set the flag of Aknowledge for the AKN Message
                    // if the write was not successful
                    if (writeSuccess != SUCCESS) {
                        mlPending.wpAck++;

                        mlWpAck.target_component = MAV_COMP_ID_MISSIONPLANNER;
                        mlWpAck.type = MAV_MISSION_ERROR;

                    }
*/
                    break;

                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:

                    writeSuccess = SUCCESS;

                    // clear the WP values in memory;
                    memset(&mlWpValues, 0, sizeof (mavlink_mission_item_values_t));
/*
                    writeSuccess = clearWaypointsFrom(0);

                    // Set the flag of Aknowledge fail
                    // if the write was unsuccessful
                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to clear waypoints from EEPROM.", 49);

                    }
  */

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
                    /*
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
                     */
                    break;

//AM DBG
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

                    if ((uint8_t) set.target_system == (uint8_t) SYSTEMID &&
                        (uint8_t) set.target_component == (uint8_t) COMPID) {


                        char* key = (char*) set.param_id;
                        uint8_t i, j;
                        uint8_t match;
                        for (i = 0; i < PAR_PARAM_COUNT; i++) {
                            match = 1;
                            for (j = 0; j < PARAM_NAME_LENGTH; j++) {
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
              default:
                break;
            }
       }
    }
}
uint16_t MissionInterfaceResponse(uint8_t system_id, uint8_t component_id){
  mavlink_message_t msg;
  uint16_t bytes2Send = 0;
  uint8_t CopyMsgToBuff = 0;
//  char vr_message[50];
//  if (mlPending.wpProtState == WP_PROT_TX_WP) { //SLUGS Nav DBG info
//    memset(vr_message, 0, sizeof (vr_message));
//    sprintf(vr_message, "%d: y =%2.2f x =%2.2f z =%2.2f o =%d  t =%d", mlPending.wpCurrentWpInTransaction, (double)mlWpValues.lat[mlPending.wpCurrentWpInTransaction],
//        (double)mlWpValues.lon[mlPending.wpCurrentWpInTransaction],
//        (double)mlWpValues.alt[mlPending.wpCurrentWpInTransaction],
//        mlWpValues.orbit[mlPending.wpCurrentWpInTransaction],
//        mlWpValues.type[mlPending.wpCurrentWpInTransaction]);
//    bytes2Send += sendQGCDebugMessage(vr_message, 0, UartOutBuff, bytes2Send + 1);
//  }
//  if (mlPending.wpProtState == WP_PROT_GETTING_WP_IDLE) { //SLUGS Nav DBG info
//      memset(vr_message, 0, sizeof (vr_message));
//      sprintf(vr_message, "com = %d, tb =%2.2f ta = %2.2f", sw_intTemp, (double)fl_temp1, (double)fl_temp2);
//      bytes2Send += sendQGCDebugMessage(vr_message, 0, UartOutBuff, bytes2Send + 1);
//
//  }

  // Current mission item (1 off indexing issue in qgc vs et)
//  if (mlPending.wpSendCurrent) { --DBG TODO AM implement with nav implamntation
//      memset(&msg, 0, sizeof (mavlink_message_t));
//      mavlink_msg_mission_current_pack(system_id,
//          component_id,
//          &msg, ((uint16_t)mlNavigation.toWP) - 1);
//      bytes2Send += mavlink_msg_to_send_buffer((UartOutBuff + 1 + bytes2Send), &msg);
//      mlPending.wpSendCurrent = FALSE;
//  }

  // clear the msg
  memset(&msg, 0, sizeof (mavlink_message_t));
  //if (mlPending.wpTransaction){ //DBG
    CopyMsgToBuff = 1; //Set send msg flag - if nothing to send the switch statement will reset this
    switch (mlPending.wpProtState) { 
        case WP_PROT_LIST_REQUESTED:

            //mlPending.statustext++;
            //mlStatustext.severity = MAV_SEVERITY_INFO;
            //strncpy(mlStatustext.text, "Got mission list request. Sending count...", 49);

            mavlink_msg_mission_count_pack(system_id,
                MAV_COMP_ID_MISSIONPLANNER,
                &msg,
                GS_SYSTEMID,
                GS_COMPID,
                mlWpValues.wpCount);

            // Change the state machine state
            mlPending.wpProtState = WP_PROT_NUM_SENT;

            // Reset the timeout
            mlPending.wpTimeOut = 0;
            bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
            break;

        case WP_PROT_GETTING_WP_IDLE:
            if (mlPending.wpCurrentWpInTransaction < mlPending.wpTotalWps) {

                mavlink_msg_mission_request_pack(system_id,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    mlPending.wpCurrentWpInTransaction++);

                // Change the state machine state
                mlPending.wpProtState = WP_PROT_RX_WP;

            } else {
                mavlink_msg_mission_ack_pack(system_id,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    MAV_MISSION_ACCEPTED); // 0 is success

                // Update the waypoint count
                mlWpValues.wpCount = mlPending.wpTotalWps;

                // End the transaction
                mlPending.wpTransaction = 0;
                mlPending.wpProtState = WP_PROT_IDLE;
                mlPending.wpCurrentWpInTransaction = 0;
                mlPending.wpTotalWps = 0;
                mlPending.wpSendCurrent = TRUE; // send current waypoint index

                // put zeros in the rest of the waypoints;
                //clearWaypointsFrom(mlWpValues.wpCount); //this clears the EEPROM WP storage

            }
            bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
            // Reset the timeout            
            mlPending.wpTimeOut = 0;
            break;

        case WP_PROT_TX_WP:
            //mlPending.statustext++;
            //mlStatustext.severity = MAV_SEVERITY_INFO;
            //strncpy(mlStatustext.text, "Sending waypoint.", 25);


            // Send WP
            mavlink_msg_mission_item_pack(system_id,
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

            // Switch the state waiting for the next request
            // Change the state machine state
            mlPending.wpProtState = WP_PROT_SENDING_WP_IDLE;
            bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
            // Reset the timeout
            mlPending.wpTimeOut = 0;
            break;
      default:
         CopyMsgToBuff = 0;
         break;
    } // switch wpProtState
//    if (CopyMsgToBuff) // Copy the message to the send buffer
//      bytes2Send += mavlink_msg_to_send_buffer((UartOutBuff + 1 + bytes2Send), &msg);

  //}//if DBG
  mlPending.wpTimeOut++;

  // if Timed out reset the state machine and send an error
  if (mlPending.wpTimeOut > PROTOCOL_TIMEOUT_TICKS) {
      memset(&msg, 0, sizeof (mavlink_message_t));

      mavlink_msg_mission_ack_pack(system_id,
          MAV_COMP_ID_MISSIONPLANNER,
          &msg,
          GS_SYSTEMID,
          GS_COMPID,
          1); // 1 is failure

      // Copy the message to the send buffer
      bytes2Send += mavlink_msg_to_send_buffer((UartOutBuff + 1 + bytes2Send), &msg);

      // reset the state machine
      mlPending.wpTransaction = 0;
      mlPending.wpProtState = WP_PROT_IDLE;
      mlPending.wpCurrentWpInTransaction = 0;
      mlPending.wpTimeOut = 0;
      mlPending.wpTotalWps = 0;
  }
  return(bytes2Send);
}

uint16_t ParameterInterfaceResponse(uint8_t system_id, uint8_t component_id){
  mavlink_message_t msg;
  uint16_t bytes2Send = 0;
  if (mlPending.piTransaction){

    switch (mlPending.piProtState) {
      case PI_SEND_ALL_PARAM:
          if (mlPending.piCurrentParamInTransaction < PAR_PARAM_COUNT) {

              mavlink_msg_param_value_pack(system_id,
                  component_id,
                  &msg,
                  mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                  mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                  MAV_PARAM_TYPE_REAL32, // TODO: make sure this is correct later on
                  PAR_PARAM_COUNT,
                  mlPending.piCurrentParamInTransaction);

              mlPending.piCurrentParamInTransaction++;

              // Copy the message to the send buffer
              bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);

          } else {
              mlPending.piProtState = PI_IDLE;
              mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
              mlPending.piTransaction = 0;
              mlPending.piBackToList = 0;
              mlPending.piQIdx = -1;
          }
          break;

      case PI_SEND_ONE_PARAM:


          if (!mlPending.piBackToList) { // if this is just a single pm read, i.e. not a retransmission
              mavlink_msg_param_value_pack(system_id,
                  component_id,
                  &msg,
                  mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                  mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                  MAV_PARAM_TYPE_REAL32,
                  PAR_PARAM_COUNT,
                  mlPending.piCurrentParamInTransaction);

              // Copy the message to the send buffer
              bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);


              mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
              mlPending.piProtState = PI_IDLE;
              mlPending.piTransaction = 0;
          } else { // you will need to go back to send all

              if (mlPending.piQIdx < 0) { // if you've sent all the requests, then go back to list
                  mlPending.piProtState = PI_SEND_ALL_PARAM;
                  mlPending.piBackToList = 0;

              } else { // send the requests
                  mavlink_msg_param_value_pack(system_id,
                      component_id,
                      &msg,
                      mlParamInterface.param_name[mlPending.piQueue[mlPending.piQIdx]],
                      mlParamInterface.param[mlPending.piQueue[mlPending.piQIdx]],
                      MAV_PARAM_TYPE_REAL32,
                      PAR_PARAM_COUNT,
                      mlPending.piQueue[mlPending.piQIdx]);

                  // Copy the message to the send buffer
                  bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);

                  // decrement the queue index
                  mlPending.piQIdx--;
              } // QIdx < 0
          }// !backToList
    }//switch
  }//if (mlPending.piTransaction)
  return(bytes2Send);
}
uint16_t PackHeartBeat(uint8_t system_id, uint8_t component_id){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  mavlink_system.type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = mlHeartbeatLocal.base_mode; ///< Booting up
  uint32_t custom_mode = mlHeartbeatLocal.custom_mode;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = mlHeartbeatLocal.system_status; ///< System ready for flight
  mavlink_message_t msg;
  uint16_t bytes2Send = 0;
  //////////////////////////////////////////////////////////////////////////
  memset(&msg, 0, sizeof (mavlink_message_t));
  // Pack the Heartbeat message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  /*
  mavlink_msg_heartbeat_pack(system_id,
                  component_id,
                  &msg,
                  MAV_TYPE_FIXED_WING,//MAV_TYPE_GENERIC,
                  MAV_AUTOPILOT_SLUGS,//MAV_AUTOPILOT_GENERIC,
                  MAV_MODE_PREFLIGHT,//mlHeartbeatLocal.base_mode,
                  MAV_MODE_PREFLIGHT,//mlHeartbeatLocal.custom_mode,
                  MAV_STATE_UNINIT//mlHeartbeatLocal.system_status
                  );
*/
  // Copy the message to the send buffer
  bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
  return(bytes2Send);
}

uint16_t PackTextMsg(uint8_t system_id, uint8_t component_id, unsigned char buf){
  mavlink_message_t msg;
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  char vr_message[50];
  memset(vr_message, 0, sizeof (vr_message));
  sprintf(vr_message, "buffer %d",buf);
  mavlink_msg_statustext_pack(mavlink_system.sysid,
  mavlink_system.compid,
  &msg,
  0,
  vr_message);
  return(mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}


uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id, mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_raw_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlRawIMUData.xacc , mlRawIMUData.yacc , mlRawIMUData.zacc , mlRawIMUData.xgyro , mlRawIMUData.ygyro , mlRawIMUData.zgyro , mlRawIMUData.xmag , mlRawIMUData.ymag , mlRawIMUData.zmag );
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackRawAttitude(uint8_t system_id, uint8_t component_id, mavlink_attitude_t mlAttitudeData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlAttitudeData.roll, mlAttitudeData.pitch, mlAttitudeData.yaw, mlAttitudeData.rollspeed, mlAttitudeData.pitchspeed, mlAttitudeData.yawspeed );
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackPosXYZ_Sol(uint8_t system_id, uint8_t component_id, mavlink_local_position_ned_t mlLocalPositionSol ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec, mlLocalPositionSol.x, mlLocalPositionSol.y, mlLocalPositionSol.z, mlLocalPositionSol.vx, mlLocalPositionSol.vy, mlLocalPositionSol.vz);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackGpsRawInt(uint8_t system_id, uint8_t component_id, mavlink_gps_raw_int_t mlRawGpsDataInt ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec ,mlRawGpsDataInt.fix_type, mlRawGpsDataInt.lat,
          mlRawGpsDataInt.lon, mlRawGpsDataInt.alt, mlRawGpsDataInt.eph, mlRawGpsDataInt.epv, mlRawGpsDataInt.vel,
          mlRawGpsDataInt.cog, mlRawGpsDataInt.satellites_visible);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}


uint16_t PackScaledPressure(uint8_t system_id, uint8_t component_id, mavlink_scaled_pressure_t mlAirData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_scaled_pressure_pack(system_id, component_id, &msg,
						       time_usec/1000, mlAirData.press_abs, mlAirData.press_diff, mlAirData.temperature);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackSysStatus(uint8_t system_id, uint8_t component_id, mavlink_sys_status_t mlSysStatus){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_sys_status_pack(system_id, component_id, &msg,
						       mlSysStatus.onboard_control_sensors_present,mlSysStatus.onboard_control_sensors_enabled, mlSysStatus.onboard_control_sensors_health,
                   mlSysStatus.load, mlSysStatus.voltage_battery, mlSysStatus.current_battery,
                   mlSysStatus.battery_remaining, mlSysStatus.drop_rate_comm, mlSysStatus.errors_comm,
                   mlSysStatus.errors_count1, mlSysStatus.errors_count2, mlSysStatus.errors_count3, mlSysStatus.errors_count4);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}
uint16_t PackRawServo(uint8_t system_id, uint8_t component_id, mavlink_servo_output_raw_t mlPwmCommands ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlPwmCommands.port, mlPwmCommands.servo1_raw, mlPwmCommands.servo2_raw, mlPwmCommands.servo3_raw, mlPwmCommands.servo4_raw, mlPwmCommands.servo5_raw, mlPwmCommands.servo6_raw, mlPwmCommands.servo7_raw, mlPwmCommands.servo8_raw );
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t HIL_PackRawServo(uint8_t system_id, uint8_t component_id, mavlink_servo_output_raw_t mlPwmCommands ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlPwmCommands.port, mlPwmCommands.servo1_raw, mlPwmCommands.servo2_raw, mlPwmCommands.servo3_raw, mlPwmCommands.servo4_raw, mlPwmCommands.servo5_raw, mlPwmCommands.servo6_raw, mlPwmCommands.servo7_raw, mlPwmCommands.servo8_raw );
  return( mavlink_msg_to_send_buffer(Uart4OutBuff, &msg));
}

uint16_t PackRawRC(uint8_t system_id, uint8_t component_id, mavlink_rc_channels_raw_t mlRC_Commands ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlRC_Commands.port, mlRC_Commands.chan1_raw, mlRC_Commands.chan2_raw, mlRC_Commands.chan3_raw, mlRC_Commands.chan4_raw, mlRC_Commands.chan5_raw, mlRC_Commands.chan6_raw, mlRC_Commands.chan7_raw, mlRC_Commands.chan8_raw,mlRC_Commands.rssi );
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackVFR_HUD(uint8_t system_id, uint8_t component_id, mavlink_vfr_hud_t mlVfr_hud ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_vfr_hud_pack(mavlink_system.sysid, mavlink_system.compid, &msg , mlVfr_hud.airspeed, mlVfr_hud.groundspeed, mlVfr_hud.heading, mlVfr_hud.throttle, mlVfr_hud.alt, mlVfr_hud.climb );
                                                                                
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) {
    mavlink_message_t msg;
    unsigned char bytes2Send = 0; // size in bytes of the mavlink packed message (return value)

    mavlink_msg_statustext_pack(101,
        1,
        &msg,
        severity,
        dbgMessage);

    bytes2Send = mavlink_msg_to_send_buffer(UartOutBuff, &msg);

    return bytes2Send;
}
  

//uint8_t GetCharAtBuffIdx(int32_t idx){
//    return(UartOutBuff[idx]);
//}
/* Declare UART1 Tx Circular Buffer Structure */

extern MCHP_UART1_TxStr MCHP_UART1_Tx;

void TxN_Data_OverU1(uint16_t N){
#if (WIN != 1)//SLUGS2 SIL
  uint16_T i;
  for (i = 0U; i < N; i++) {
    uint16_T Tmp;
    Tmp = ~(MCHP_UART1_Tx.tail - MCHP_UART1_Tx.head);
    Tmp = Tmp & (Tx_BUFF_SIZE_Uart1 - 1);/* Modulo Buffer Size */
    if (Tmp != 0) {
      MCHP_UART1_Tx.buffer[MCHP_UART1_Tx.tail] = UartOutBuff[i];
      MCHP_UART1_Tx.tail = (MCHP_UART1_Tx.tail + 1) & (Tx_BUFF_SIZE_Uart1 - 1);
      Tmp--;
    }
  }
  _U1TXIF = U1STAbits.TRMT;
#else
	mavlink_serial_send(MAVLINK_COMM_0, &UartOutBuff[0], (uint16_t)N);

#endif
}
extern MCHP_UART4_TxStr MCHP_UART4_Tx;

void TxN_Data_OverU4(uint16_t N){
#if (WIN != 1)//SLUGS2 SIL
  uint16_T i;
  for (i = 0U; i < N; i++) {
    uint16_T Tmp;
    Tmp = ~(MCHP_UART4_Tx.tail - MCHP_UART4_Tx.head);
    Tmp = Tmp & (Tx_BUFF_SIZE_Uart4 - 1);/* Modulo Buffer Size */
    if (Tmp != 0) {
      MCHP_UART4_Tx.buffer[MCHP_UART4_Tx.tail] = Uart4OutBuff[i];
      MCHP_UART4_Tx.tail = (MCHP_UART4_Tx.tail + 1) & (Tx_BUFF_SIZE_Uart4 - 1);
      Tmp--;
    }
  }
  _U4TXIF = U4STAbits.TRMT;
#else
	mavlink_serial_send(MAVLINK_COMM_0, &Uart4OutBuff[0], (uint16_t)N);

#endif
}