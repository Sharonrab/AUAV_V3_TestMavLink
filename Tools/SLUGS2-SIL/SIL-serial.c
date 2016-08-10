//
//  SIL-serial.c
//  MatrixPilot-SIL
//
//  Created by Ben Levitt on 2/10/13.
//  Copyright (c) 2013 MatrixPilot. All rights reserved.
//

#if (WIN == 1 || NIX == 1)

#include <stdio.h>
#include "SIL-udb.h"
//#include "../../libUDB/libUDB.h"
#include "libUDB.h"

#include "UDBSocket.h"

#include "circBuffer.h"

#include "../mavLink/include/mavlink_types.h"

UDBSocket gpsSocket;
UDBSocket telemetrySocket;

int32_t gpsRate = 0;
int32_t serialRate = 0;


#define BUFLEN 512

struct CircBuffer com4Buffer;
CBRef uartBuffer;

const uint8_t* gps_out_buffer = 0;
int16_t gps_out_buffer_length = 0;
int16_t gps_out_index = 0;

#define	SERIAL_BUFFER_SIZE  MAVLINK_MAX_PACKET_LEN

int16_t sb_index = 0;
int16_t end_index = 0;
char serial_interrupt_stopped = 1;
uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
//////////////////////////////////////////////////////////
// GPS and Telemetry
//////////////////////////////////////////////////////////

void udb_gps_set_rate(int32_t rate)
{
	gpsRate = rate;
}


boolean udb_gps_check_rate(int32_t rate)
{
	return (rate == gpsRate);
}

// Call this function to initiate sending data to the GPS
void udb_gps_start_sending_data(void)
{
	uint8_t buffer[BUFLEN];
	int16_t bytesWritten;
	int16_t c;
	int16_t pos=0;

	if (!gpsSocket) return;

	while (pos < BUFLEN && (c = udb_gps_callback_get_byte_to_send()) != -1) {
		buffer[pos++] = c;
	}
	
	bytesWritten = UDBSocket_write(gpsSocket, (uint8_t*)buffer, pos);
	
	if (bytesWritten < 0) {
		UDBSocket_close(gpsSocket);
		gpsSocket = NULL;
	}
}


void udb_serial_set_rate(int32_t rate)
{
	serialRate = rate;
}


boolean udb_serial_check_rate(int32_t rate)
{
	return (serialRate == rate);
}


// Call this function to initiate sending a data to the serial port
void udb_serial_start_sending_data(void)
{
	uint8_t buffer[BUFLEN];
	int16_t bytesWritten;
	int16_t c;
	int16_t pos = 0;

	if (!telemetrySocket) return;

	while (pos < BUFLEN && (c = udb_serial_callback_get_byte_to_send()) != -1) {
		buffer[pos++] = c;
	}
	bytesWritten = UDBSocket_write(telemetrySocket, (uint8_t*)buffer, pos);
	if (bytesWritten == -1) {
		UDBSocket_close(telemetrySocket);
		telemetrySocket = NULL;
	}
}
void uartBufferInit(void) 
{
	
	uartBuffer = (struct CircBuffer*) &com4Buffer;
	newCircBuffer(uartBuffer);
}
// Got a character from the GPS
void udb_gps_callback_received_byte(uint8_t rxchar)
{
	//bin_out(rxchar);      // binary out to the debugging USART
	//(*msg_parse)(rxchar);   // parse the input byte
	writeBack(uartBuffer, (unsigned char)rxchar);

}
// MAIN MATRIXPILOT MAVLINK CODE FOR RECEIVING COMMANDS FROM THE GROUND CONTROL STATION
//

mavlink_message_t msg[2];
uint8_t mavlink_message_index = 0;
mavlink_status_t r_mavlink_status;

void udb_serial_callback_received_byte(uint8_t rxchar)
{
	//	DPRINT("%u \r\n", rxchar);

	//if (mavlink_parse_char(0, rxchar, &msg[mavlink_message_index], &r_mavlink_status))
	//{
	//	// Check that handling of previous message has completed before calling again
	//	if (handling_of_message_completed == true)
	//	{
	//		// Switch between incoming message buffers
	//		if (mavlink_message_index == 0) mavlink_message_index = 1;
	//		else mavlink_message_index = 0;
	//		handling_of_message_completed = false;
	//		trigger_event(mavlink_process_message_handle);
	//	}
	//}
}

void gpsoutbin(int16_t length, const uint8_t msg[]) // output a binary message to the GPS
{
	gps_out_buffer = 0; // clear the buffer pointer first, for safety, in case we're interrupted
	gps_out_index = 0;
	gps_out_buffer_length = length;
	gps_out_buffer = (uint8_t*)msg;

	udb_gps_start_sending_data();
}

int16_t udb_gps_callback_get_byte_to_send(void)
{
	if (gps_out_buffer != 0 && gps_out_index < gps_out_buffer_length)
	{
		// We have a byte to send
		return (uint8_t)(gps_out_buffer[gps_out_index++]);
	}
	else
	{
		// No byte to send, so clear the link to the buffer
		gps_out_buffer = 0;
	}
	return -1;
}

int16_t udb_serial_callback_get_byte_to_send(void)
{
	if (sb_index < end_index && sb_index < SERIAL_BUFFER_SIZE) // ensure never end up racing thru memory.
	{
		uint8_t txchar = serial_buffer[sb_index++];
		return txchar;
	}
	else
	{
		serial_interrupt_stopped = 1;
	}
	return -1;
}

#endif // (WIN == 1 || NIX == 1)
