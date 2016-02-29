
void protDecodeMavlink(void) {

    uint8_t indx, writeSuccess, commChannel = 1;
    mavlink_param_set_t set;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t* dataIn;
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uartBufferInMavlink), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    DatafromGSmavlink[0] = (tmpLen > MAXINLEN) ? MAXINLEN : tmpLen;

    // read the data
    for (i = 1; i <= DatafromGSmavlink[0]; i += 1) {
      DatafromGSmavlink[i] = readFront(uartBufferInMavlink);
    }
    dataIn = DatafromGSmavlink;

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
                    if (mlSetMode.custom_mode != 0 ) { //!= SLUGS_MODE_NONE
                      mlHeartbeatLocal.custom_mode = mlSetMode.custom_mode;
                    }
                    break;
                }

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

                case MAVLINK_MSG_ID_PING:
                    mavlink_msg_ping_decode(&msg, &mlPing);

                    mlPending.ping = 1;
                    break;


                case MAVLINK_MSG_ID_STATUSTEXT:
                    // Received status message from the sensor MCU -- forward to GS
                    mavlink_msg_statustext_decode(&msg, &mlStatustext);
                    mlPending.statustext++;
                    break;

                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_msg_command_long_decode(&msg, &mlCommand);

                    switch (mlCommand.command) {
                        // TODO: Only handle this in PREFLIGHT mode
                        case MAV_CMD_PREFLIGHT_STORAGE:
                            writeSuccess = FAILURE;
                            // Parameter storage
                            if (mlCommand.param1 == 0.0f) { // read
                                memset(&(mlParamInterface.param[0]), 0, sizeof (float) *PAR_PARAM_COUNT);
                                //writeSuccess = readParamsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write

                                //writeSuccess = storeAllParamsInEeprom();
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
            }// switch
        } // if

        // Update global packet drops counter
        if (commChannel == 1) {
            mlSystemStatus.errors_comm += status.packet_rx_drop_count;
        }


    }// for
}
