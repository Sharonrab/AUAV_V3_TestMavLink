// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


//#include "libDCM_internal.h"
//#include "gpsParseCommon.h"
//#include "../libUDB/magnetometer.h"
//#include "rmat.h"

//#include "udbTypes.h"
//#include "libUDB.h"
#include "mavlink.h"
/* Include model header file for global data */
#ifdef UNIT_TEST
#include "AUAV_V3_TestSensors.h"
#else
#include "SLUGS2.h"
#endif
#ifdef WIN
#include <stdint.h>
#endif
extern uint8_t Uart4OutBuff[MAVLINK_MAX_PACKET_LEN];

// UDB Types
struct bb { uint8_t B0; uint8_t B1; };
struct bbbb { uint8_t B0; uint8_t B1; uint8_t B2; uint8_t B3; };
struct ww { int16_t W0; int16_t W1; };
struct wwww { int16_t W0; int16_t W1; int16_t W2; int16_t W3; };
struct LL { int32_t L0; int32_t L1; };

union intbb { int16_t BB;  struct bb _; };
union uintbb { uint16_t BB; struct bb _; };
union longbbbb { int32_t WW;  struct ww _; struct bbbb __; };
union longww { int32_t WW;  struct ww _; };
union longlongLL { int64_t LL;  struct LL _; struct wwww __; };

union longbbbb lat_origin, lon_origin, alt_origin;
union longbbbb lat_gps, lon_gps, alt_sl_gps;        // latitude, longitude, altitude
union intbb week_no;
union intbb sog_gps;                                // speed over ground
union uintbb cog_gps;                               // course over ground
union intbb climb_gps;                              // climb
union intbb hilsim_airspeed;
union longbbbb tow;
//union longbbbb xpg, ypg, zpg;                     // gps x, y, z position
//union intbb    xvg, yvg, zvg;                     // gps x, y, z velocity
//uint8_t mode1, mode2;                             // gps mode1, mode2
uint8_t hdop;                                       // horizontal dilution of precision
uint8_t svs;                                        // number of satellites
int16_t cos_lat = 0;
int16_t gps_data_age;
const uint8_t* gps_out_buffer = 0;
int16_t gps_out_buffer_length = 0;
int16_t gps_out_index = 0;

// GPS parser modules variables
union longbbbb lat_gps_, lon_gps_;
union longbbbb alt_sl_gps_;
union longbbbb tow_;
//union intbb sog_gps_, cog_gps_, climb_gps_;
//union intbb nav_valid_, nav_type_, week_no_;
union intbb hdop_;

int8_t actual_dir;
uint16_t ground_velocity_magnitudeXY = 0;
int16_t forward_acceleration = 0;
uint16_t air_speed_magnitudeXY = 0;
uint16_t air_speed_3DGPS = 0;
int8_t calculated_heading;           // takes into account wind velocity

#define SCALEACCEL      2.64

#define GRAVITY                 ((int32_t)(5280.0/SCALEACCEL))  // gravity in AtoD/2 units
typedef int16_t fractional;

fractional gplane[] = { 0, 0, GRAVITY };
int16_t aero_force[] = { 0 , 0 , -GRAVITY };
//static int8_t cog_previous = 64;
//static int16_t sog_previous = 0;
//static int16_t climb_rate_previous = 0;
//static int16_t location_previous[] = { 0, 0, 0 };
//static uint16_t velocity_previous = 0;

extern void(*msg_parse)(uint8_t gpschar);
typedef void(*background_callback)(void);

static void udb_background_callback_triggered(void);
void udb_background_trigger(background_callback callback);

void gpsoutline(const char* message);
void gpsoutbin(int16_t length, const uint8_t* msg);

union longbbbb date_gps_, time_gps_;


// fractional dirOverGndHGPS[];         //  horizontal velocity over ground, as measured by GPS (Vz = 0 )
// fractional dirOverGndHrmat[];        //  horizontal direction over ground, as indicated by Rmatrix
 extern mavlink_gps_raw_int_t mlGpsData;
 extern mavlink_servo_output_raw_t mlPwmCommands;
//#if (GPS_TYPE == GPS_UBX_2HZ || GPS_TYPE == GPS_UBX_4HZ || GPS_TYPE == GPS_ALL)

// Parse the GPS messages, using the binary interface.
// The parser uses a state machine implemented via a pointer to a function.
// Binary values received from the GPS are directed to program variables via a table
// of pointers to the variable locations.
// Unions of structures are used to be able to access the variables as long, ints, or bytes.

static union intbb payloadlength;
static union intbb checksum;
static uint16_t msg_class;
static uint16_t msg_id;
static uint16_t ack_class; // set but never used - RobD
static uint16_t ack_id; // set but never used - RobD
static uint16_t ack_type; // set but never used - RobD
static uint8_t CK_A;
static uint8_t CK_B;

static void msg_B3(uint8_t inchar);
static void msg_SYNC1(uint8_t inchar);
static void msg_SYNC2(uint8_t inchar);
static void msg_CLASS(uint8_t inchar);
static void msg_ID(uint8_t inchar);
static void msg_PL1(uint8_t inchar);
static void msg_POSLLH(uint8_t inchar);
static void msg_DOP(uint8_t inchar);
static void msg_SOL(uint8_t inchar);
static void msg_VELNED(uint8_t inchar);
static void msg_CS1(uint8_t inchar);

//#if (HILSIM == 1)
	static void msg_BODYRATES(uint8_t inchar);
//#endif

static void msg_MSGU(uint8_t inchar);
static void msg_ACK_CLASS(uint8_t inchar);
static void msg_ACK_ID(uint8_t inchar);

//void bin_out(char outchar);

const char bin_mode_withnmea[] = "$PUBX,41,1,0003,0003,19200,0*21\r\n"; // turn on UBX + NMEA, 19200 baud
const char bin_mode_nonmea[] = "$PUBX,41,1,0003,0001,19200,0*23\r\n";   // turn on UBX only, 19200 baud
const char disable_GSV[] = "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"; //Disable the $GPGSV NMEA message
const char disable_VTG[] = "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"; //Disable the $GPVTG NMEA message
const char disable_GLL[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"; //Disable the $GPGLL NMEA message
const char disable_GSA[] = "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"; //Disable the $GPGSA NMEA message

//#if (GPS_TYPE == GPS_UBX_4HZ)
const uint8_t set_rate[] = {
	0xB5, 0x62, // Header
	0x06, 0x08, // ID
	0x06, 0x00, // Payload Length
	0xFA, 0x00, // measRate
	0x01, 0x00, // navRate
	0x01, 0x00, // timeRef
	0x10, 0x96  // Checksum
};
//#else
//const uint8_t set_rate[] = {
//	0xB5, 0x62, // Header
//	0x06, 0x08, // ID
//	0x06, 0x00, // Payload Length
//	0xF4, 0x01, // measRate 2Hz
//	0x01, 0x00, // navRate
//	0x01, 0x00, // timeRef
//	0x0B, 0x77  // Checksum
//};
//#endif

const uint8_t enable_UBX_only[] = {
	0xB5, 0x62, // Header
	0x06, 0x00, // ID
	0x14, 0x00, // Payload length
	0x01,       // Port ID
	0x00,       // res0
	0x00, 0x00, // res1
	0xD0, 0x08, 0x00, 0x00, // mode
	0x00, 0x4B, 0x00, 0x00, // baudrate
	0x03, 0x00, // inProtoMask
	0x01, 0x00, // outProtoMask
	0x00, 0x00, // Flags - reserved, set to 0
	0x00, 0x00, // Pad - reserved, set to 0
	0x42, 0x2B  // checksum
};

const uint8_t enable_UBX_NMEA[] = {
	0xB5, 0x62, // Header
	0x06, 0x00, // ID
	0x14, 0x00, // Payload length
	0x01,       // Port ID
	0x00,       // res0
	0x00, 0x00, // res1
	0xD0, 0x08, 0x00, 0x00, // mode
	0x00, 0x4B, 0x00, 0x00, // baudrate
	0x03, 0x00, // inProtoMask
	0x03, 0x00, // outProtoMask
	0x00, 0x00, // Flags - reserved, set to 0
	0x00, 0x00, // Pad - reserved, set to 0
	0x44, 0x37  // checksum
};

const uint8_t enable_NAV_SOL[] = {
	0xB5, 0x62, // Header
	0x06, 0x01, // ID
	0x08, 0x00, // Payload length
	0x01,       // NAV message class
	0x06,       // SOL message ID
	0x00,       // Rate on I2C
	0x01,       // Rate on UART 1
	0x00,       // Rate on UART 2
	0x00,       // Rate on USB
	0x00,       // Rate on SPI
	0x00,       // Rate on ???
	0x17, 0xDA  // Checksum
};

const uint8_t enable_NAV_POSLLH[] = {
	0xB5, 0x62, // Header
	0x06, 0x01, // ID
	0x08, 0x00, // Payload length
	0x01,       // NAV message class
	0x02,       // POSLLH message ID
	0x00,       // Rate on I2C
	0x01,       // Rate on UART 1
	0x00,       // Rate on UART 2
	0x00,       // Rate on USB
	0x00,       // Rate on SPI
	0x00,       // Rate on ???
	0x13, 0xBE  // Checksum
};

const uint8_t enable_NAV_VELNED[] = {
	0xB5, 0x62, // Header
	0x06, 0x01, // ID
	0x08, 0x00, // Payload length
	0x01,       // NAV message class
	0x12,       // VELNED message ID
	0x00,       // Rate on I2C
	0x01,       // Rate on UART 1
	0x00,       // Rate on UART 2
	0x00,       // Rate on USB
	0x00,       // Rate on SPI
	0x00,       // Rate on ???
	0x23, 0x2E  // Checksum
};

//#if (GPS_TYPE == GPS_UBX_4HZ)
const uint8_t enable_NAV_DOP[] = {
	0xB5, 0x62, // Header
	0x06, 0x01, // ID
	0x08, 0x00, // Payload length
	0x01,       // NAV message class
	0x04,       // DOP message ID
	0x00,       // Rate on I2C
	0x04,       // Rate on UART 1
	0x00,       // Rate on UART 2
	0x00,       // Rate on USB
	0x00,       // Rate on SPI
	0x00,       // Rate on ???
	0x18, 0xDB  // Checksum
};
//#else
//const uint8_t enable_NAV_DOP[] = {
//	0xB5, 0x62, // Header
//	0x06, 0x01, // ID
//	0x08, 0x00, // Payload length
//	0x01,       // NAV message class
//	0x04,       // DOP message ID
//	0x00,       // Rate on I2C
//	0x02,       // Rate on UART 1
//	0x00,       // Rate on UART 2
//	0x00,       // Rate on USB
//	0x00,       // Rate on SPI
//	0x00,       // Rate on ???
//	0x16, 0xD1  // Checksum
//};
//#endif

const uint8_t enable_SBAS[] = {
	0xB5, 0x62, // Header
	0x06, 0x16, // ID
	0x08, 0x00, // Payload length
	0x01,       // Enable SBAS
	0x03,       //
	0x01,       //
	0x00,       //
	0x00,       //
	0x00,       //
	0x00,       //
	0x00,       //
	0x29, 0xAD  // Checksum
};

const uint8_t config_NAV5[] = {
	0xB5, 0x62, // Header
	0x06, 0x24, // ID
	0x24, 0x00, // Payload length
	0xFF, 0xFF, //
	0x04,       // Dynamic Model Number
	0x02,       //
	0x00, 0x00, //
	0x00, 0x00, //
	0x10, 0x27, //
	0x00, 0x00, //
	0x05, 0x00, //
	0xFA, 0x00, //
	0xFA, 0x00, //
	0x64, 0x00, //
	0x2C, 0x01, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x00, 0x00, //
	0x13, 0x77  // Checksum
};

const uint16_t set_rate_length = 14;
const uint16_t enable_NAV_SOL_length = 16;
const uint16_t enable_NAV_POSLLH_length = 16;
const uint16_t enable_NAV_VELNED_length = 16;
const uint16_t enable_NAV_DOP_length = 16;
const uint16_t enable_UBX_only_length = 28;
const uint16_t enable_SBAS_length = 16;
const uint16_t config_NAV5_length = 44;

void (*msg_parse)(uint8_t inchar) = &msg_B3;

static uint8_t un;
//static union longbbbb xpg_, ypg_, zpg_;
//static union longbbbb xvg_, yvg_, zvg_;
//static uint8_t mode1_, mode2_;
static uint8_t svs_, nav_valid_;
//static union longbbbb lat_gps_, lon_gps_, alt_sl_gps_;
static union longbbbb sog_gps_, cog_gps_, climb_gps_;
//static union longbbbb tow_;
static union longbbbb as_sim_;
//static union intbb hdop_;
static union intbb week_no_;

uint8_t svsmin = 24;
uint8_t svsmax = 0;
static int16_t store_index = 0;
static int16_t nmea_passthru_countdown = 0; // used by nmea_passthru to count how many more bytes are passed through
static uint8_t nmea_passthrough_char = 0;
//static int16_t frame_errors = 0;

//#if (HILSIM == 1)
static union intbb g_a_x_sim_, g_a_y_sim_, g_a_z_sim_;
static union intbb g_a_x_sim,  g_a_y_sim,  g_a_z_sim;
static union intbb p_sim_,     q_sim_,     r_sim_;
static union intbb p_sim,      q_sim,      r_sim;
void commit_bodyrate_data(void);
//#endif

//#if (HILSIM == 1 && MAG_YAW_DRIFT == 1)
//extern uint8_t magreg[6];
//#endif

uint8_t* const msg_SOL_parse[] = {
	&tow_.__.B0, &tow_.__.B1, &tow_.__.B2, &tow_.__.B3, // iTOW
	&un, &un, &un, &un,                                 // fTOW
	&week_no_._.B0, &week_no_._.B1,                     // week
	&nav_valid_,                                        // gpsFix
	&un,                                                // flags
	&un, &un, &un, &un,                                 // ecefX
	&un, &un, &un, &un,                                 // ecefY
	&un, &un, &un, &un,                                 // ecefZ
//#if (HILSIM == 1 && MAG_YAW_DRIFT == 1)
//	&magreg[1], &magreg[0], &magreg[3], &magreg[2],     // simulate the magnetometer with HILSIM, and use these slots
//	                                                    // note: mag registers come out high:low from magnetometer
//#else
	&un, &un, &un, &un,                                 // pAcc
//#endif
	&un, &un, &un, &un,                                 // ecefVX
	&un, &un, &un, &un,                                 // ecefVY
	&un, &un, &un, &un,                                 // ecefVZ

//#if (HILSIM == 1 && MAG_YAW_DRIFT == 1)
//	&magreg[5], &magreg[4], &un, &un,                   // simulate the magnetometer with HILSIM, and use these slots
//	                                                    // note: mag registers come out high:low from magnetometer
//#else
	&un, &un, &un, &un,                                 // sAcc
//#endif
	&un, &un,                                           // pDOP
	&un,                                                // res1
	&svs_,                                              // numSV
	&un, &un, &un, &un,                                 // res2
};

uint8_t* const msg_DOP_parse[] = {
	&un, &un, &un, &un,                                 // iTOW
	&un, &un,                                           // gDOP
	&un, &un,                                           // pDOP
	&un, &un,                                           // tDOP
	&un, &un,                                           // vDOP
	&hdop_._.B0, &hdop_._.B1,                           // hDOP
	&un, &un,                                           // nDOP
	&un, &un,                                           // eDOP
};

uint8_t* const msg_POSLLH_parse[] = {
	&un, &un, &un, &un,                                 // iTOW
	&lon_gps_.__.B0, &lon_gps_.__.B1,
	&lon_gps_.__.B2, &lon_gps_.__.B3,                   // lon
	&lat_gps_.__.B0, &lat_gps_.__.B1,
	&lat_gps_.__.B2, &lat_gps_.__.B3,                   // lat
	&un, &un, &un, &un,                                 // height
	&alt_sl_gps_.__.B0, &alt_sl_gps_.__.B1,
	&alt_sl_gps_.__.B2, &alt_sl_gps_.__.B3,             // hMSL
	&un, &un, &un, &un,                                 // hAcc
	&un, &un, &un, &un,                                 // vAcc
};

uint8_t* const msg_VELNED_parse[] = {
	&un, &un, &un, &un,                                 // iTOW
	&un, &un, &un, &un,                                 // velN
	&un, &un, &un, &un,                                 // velE
	&climb_gps_.__.B0, &climb_gps_.__.B1,
	&climb_gps_.__.B2, &climb_gps_.__.B3,               // velD
	&as_sim_.__.B0, &as_sim_.__.B1,
	&as_sim_.__.B2, &as_sim_.__.B3,                     // air speed
	&sog_gps_.__.B0, &sog_gps_.__.B1,
	&sog_gps_.__.B2, &sog_gps_.__.B3,                   // gSpeed
	&cog_gps_.__.B0, &cog_gps_.__.B1,
	&cog_gps_.__.B2, &cog_gps_.__.B3,                   // heading
	&un, &un, &un, &un,                                 // sAcc
	&un, &un, &un, &un,                                 // cAcc
};

//#if (HILSIM == 1)
// These are the data being delivered from the hardware-in-the-loop simulator
uint8_t* const msg_BODYRATES_parse[] = {
	&p_sim_._.B0, &p_sim_._.B1,         // roll rate
	&q_sim_._.B0, &q_sim_._.B1,         // pitch rate
	&r_sim_._.B0, &r_sim_._.B1,         // yaw rate
	&g_a_x_sim_._.B0, &g_a_x_sim_._.B1, // x accel reading (grav - accel, body frame)
	&g_a_y_sim_._.B0, &g_a_y_sim_._.B1, // y accel reading (grav - accel, body frame)
	&g_a_z_sim_._.B0, &g_a_z_sim_._.B1, // z accel reading (grav - accel, body frame)
};
//#endif // HILSIM

void gps_startup_sequence(int16_t gpscount)
{
//	if (gpscount == 980)
//	{
//#if (HILSIM == 1)
//		udb_gps_set_rate(HILSIM_BAUD);
//#else
//		udb_gps_set_rate(9600);
//#endif
//	}
//	else if (dcm_flags._.nmea_passthrough && gpscount == 200)
//		gpsoutline(disable_GSV);
//	else if (dcm_flags._.nmea_passthrough && gpscount == 190)
//		gpsoutline(disable_GSA);
//	else if (dcm_flags._.nmea_passthrough && gpscount == 180)
//		gpsoutline(disable_GLL);
//	else if (dcm_flags._.nmea_passthrough && gpscount == 170)
//		gpsoutline(disable_VTG);
//	else if (dcm_flags._.nmea_passthrough && gpscount == 160)
//		// set the UBX to use binary and nmea
//		gpsoutline(bin_mode_withnmea);
//	else if (!dcm_flags._.nmea_passthrough && gpscount == 160)
//		// set the UBX to use binary mode
//		gpsoutline(bin_mode_nonmea);
//#if (HILSIM != 1)
//	else if (gpscount == 150)
//		udb_gps_set_rate(19200);
//#endif
//	else if (gpscount == 140)
//		gpsoutbin(set_rate_length, set_rate);
//	else if (gpscount == 130)
//		// command GPS to select which messages are sent, using UBX interface
//		gpsoutbin(enable_NAV_SOL_length, enable_NAV_SOL);
//	else if (gpscount == 120)
//		gpsoutbin(enable_NAV_POSLLH_length, enable_NAV_POSLLH);
//	else if (gpscount == 110)
//		gpsoutbin(enable_NAV_VELNED_length, enable_NAV_VELNED);
//	else if (gpscount == 100)
//		gpsoutbin(enable_NAV_DOP_length, enable_NAV_DOP);
//	else if (dcm_flags._.nmea_passthrough && gpscount == 90)
//		gpsoutbin(enable_UBX_only_length, enable_UBX_NMEA);
//	else if (!dcm_flags._.nmea_passthrough && gpscount == 90)
//		gpsoutbin(enable_UBX_only_length, enable_UBX_only);
//	else if (gpscount == 80)
//		gpsoutbin(enable_SBAS_length, enable_SBAS);
//	else if (gpscount == 70)
//		gpsoutbin(config_NAV5_length, config_NAV5);
}
typedef uint8_t boolean;
//#endif
//#define true                    1
//#define false                   0

boolean gps_nav_valid(void)
{
	return (nav_valid_ == 3);
}

/*
int16_t hex_count = 0;
const char convert[] = "0123456789ABCDEF";
const char endchar = 0xB5;

// Used for debugging purposes, converts to HEX and outputs to the debugging USART
// Only the first 5 bytes following a B3 are displayed.
void hex_out(char outchar)
{
	if (hex_count > 0) 
	{
		U1TXREG = convert[((outchar>>4) & 0x0F)];
		U1TXREG = convert[(outchar & 0x0F)];
		U1TXREG = ' ';
		hex_count--;
	}
	if (outchar == endchar)
	{
		hex_count = 5;
		U1TXREG = '\r';
		U1TXREG = '\n';
	}
}
 */

// The parsing routines follow. Each routine is named for the state in which the routine is applied.
// States correspond to the portions of the binary messages.
// For example, msg_B3 is the routine that is applied to the byte received after a B3 is received.
// If an A0 is received, the state machine transitions to the A0 state.

void nmea_passthru(uint8_t gpschar)
{
	nmea_passthrough_char = gpschar;
	gpsoutbin(1, &nmea_passthrough_char);

	nmea_passthru_countdown--;
/* removed in favor of the line ending mechanism, see below. While this is compliant with
   (published) standards, the issue appears to center around the end of line.

	if (gpschar == '*')
	{ // * indicates the start of the checksum, 2 characters remain in the message
		nmea_passthru_countdown = 2;
	}
 */
	if (gpschar == 0x0A)
	{ // end of line appears to always be 0x0D, 0x0A (\r\n)
		msg_parse = &msg_B3; // back to the inital state
	}
	else if (nmea_passthru_countdown == 0)
	{
		msg_parse = &msg_B3; // back to the inital state
	}
}

static void msg_B3(uint8_t gpschar)
{
	if (gpschar == 0xB5)
	{
		//bin_out(0x01);
		msg_parse = &msg_SYNC1;
	}
	//else if (dcm_flags._.nmea_passthrough && gpschar == '$' && udb_gps_check_rate(19200))
	//{
	//	nmea_passthru_countdown = 128; // this limits the number of characters we will passthrough. (Most lines are 60-80 chars long.)
	//	msg_parse = &nmea_passthru;
	//	nmea_passthru (gpschar);
	//}
	else
	{
		// error condition
	}
}

static void msg_SYNC1(uint8_t gpschar)
{
	if (gpschar == 0x62)
	{
		//bin_out(0x02);
		store_index = 0;
		msg_parse = &msg_SYNC2;
	}
	else
	{
		msg_parse = &msg_B3;        // error condition
	}
}

static void msg_SYNC2(uint8_t gpschar)
{
	//bin_out(0x03);
	msg_class = gpschar;
	CK_A = 0;
	CK_B = 0;
	CK_A += gpschar;
	CK_B += CK_A;
	msg_parse = &msg_CLASS;
}

static void msg_CLASS(uint8_t gpschar)
{
	//bin_out(0x04);
	msg_id = gpschar;
	CK_A += gpschar;
	CK_B += CK_A;
	msg_parse = &msg_ID;
}

static void msg_ID(uint8_t gpschar)
{
	//bin_out(0x05);
	payloadlength._.B0 = gpschar;   // UBX stored payload length in little endian order
	CK_A += gpschar;
	CK_B += CK_A;
	msg_parse = &msg_PL1;
}
#define NUM_POINTERS_IN(x)      (sizeof(x)/sizeof(char*))

static void msg_PL1(uint8_t gpschar)
{
	//bin_out(0x06);
	payloadlength._.B1 = gpschar;   // UBX stored payload length in little endian order
	CK_A += gpschar;
	CK_B += CK_A;

	switch (msg_class) {
		case 0x01 : {
			switch (msg_id) {
				case 0x02 : { // NAV-POSLLH message
					if (payloadlength.BB  == NUM_POINTERS_IN(msg_POSLLH_parse))
					{
						msg_parse = &msg_POSLLH;
					}
					else
					{
						msg_parse = &msg_B3;    // error condition
					}
					break;
				}
				case 0x04 : { // NAV-DOP message
					if (payloadlength.BB  == NUM_POINTERS_IN(msg_DOP_parse))
					{
						msg_parse = &msg_DOP;
					}
					else
					{
						msg_parse = &msg_B3;    // error condition
					}
					break;
				}
				case 0x06 : { // NAV-SOL message
					if (payloadlength.BB  == NUM_POINTERS_IN(msg_SOL_parse))
					{
						msg_parse = &msg_SOL;
					}
					else
					{
						msg_parse = &msg_B3;    // error condition
					}
					break;
				}
				case 0x12 : { // NAV-VELNED message
					if (payloadlength.BB  == NUM_POINTERS_IN(msg_VELNED_parse))
					{
						msg_parse = &msg_VELNED;
					}
					else
					{
						msg_parse = &msg_B3;    // error condition
					}
					msg_parse = &msg_VELNED;    // TODO: this does not look right (wipes out error setting above) - RobD
					break;
				}
//#if (HILSIM == 1)
				case 0xAB : { // NAV-BODYRATES message - THIS IS NOT AN OFFICIAL UBX MESSAGE
					// WE ARE FAKING THIS FOR HIL SIMULATION
					if (payloadlength.BB  == NUM_POINTERS_IN(msg_BODYRATES_parse))
					{
						msg_parse = &msg_BODYRATES;
					}
					else
					{
						msg_parse = &msg_B3;    // error condition
					}
					break;
				}
//#endif
				default : {     // some other NAV class message
					msg_parse = &msg_MSGU;
					break;
				}
			}
			break;
		}
		case 0x05 : {
			switch (msg_id) {
				case 0x00 : {   // NACK message
					ack_type = 0;
					msg_parse = &msg_ACK_CLASS;
					break;
				}
				case 0x01 : {   // ACK message
					ack_type = 1;
					msg_parse = &msg_ACK_CLASS;
					break;
				}
				default : {     // There are no other messages in this class, so this is an error
					msg_parse = &msg_B3;
					break;
				}
			}
			break;
		}
		default : {             // a non NAV class message
			msg_parse = &msg_MSGU;
			break;
		}
	}
}

static void msg_POSLLH(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		*msg_POSLLH_parse[store_index++] = gpschar;
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		//gpsoutchar2(0x08);
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}

static void msg_DOP(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		*msg_DOP_parse[store_index++] = gpschar;
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		//gpsoutchar2(0x09);
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}

static void msg_SOL(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		*msg_SOL_parse[store_index++] = gpschar;
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		//gpsoutchar2(0x0A);
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}

static void msg_VELNED(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		*msg_VELNED_parse[store_index++] = gpschar;
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		//gpsoutchar2(0x0B);
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}

//#if (HILSIM == 1)
static void msg_BODYRATES(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		*msg_BODYRATES_parse[store_index++] = gpschar;
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}
//#endif // HILSIM

static void msg_ACK_CLASS(uint8_t gpschar)
{
	//bin_out(0xAA);
	ack_class = gpschar;
	CK_A += gpschar;
	CK_B += CK_A;
	msg_parse = &msg_ACK_ID;
}

static void msg_ACK_ID(uint8_t gpschar)
{
	//bin_out(0xBB);
	ack_id = gpschar;
	CK_A += gpschar;
	CK_B += CK_A;
	msg_parse = &msg_CS1;
}

static void msg_MSGU(uint8_t gpschar)
{
	if (payloadlength.BB > 0)
	{
		CK_A += gpschar;
		CK_B += CK_A;
		payloadlength.BB--;
	}
	else
	{
		// If the payload length is zero, we have received the entire payload, or the payload length
		// was zero to start with. either way, the byte we just received is the first checksum byte.
		//gpsoutchar2(0x08);
		checksum._.B1 = gpschar;
		msg_parse = &msg_CS1;
	}
}
// If GPS data has not been received for this many state machine cycles, consider the GPS lock to be lost.
#define GPS_DATA_MAX_AGE    9
void udb_background_trigger(background_callback callback)
{
	if (callback) callback();
}

static void msg_CS1(uint8_t gpschar)
{
	checksum._.B0 = gpschar;
	if ((checksum._.B1 == CK_A) && (checksum._.B0 == CK_B))
	{
		if (msg_id == 0x12)
		{
			// correct checksum for VELNED message
			//gps_parse_common(); // parsing is complete, schedule navigation
			udb_background_trigger(&udb_background_callback_triggered);
		}
//#if (HILSIM == 1)
		else if (msg_id == 0xAB)
		{
			// If we got the correct checksum for bodyrates, commit that data immediately
			commit_bodyrate_data();
		}
//#endif
	}
	else
	{
		gps_data_age = GPS_DATA_MAX_AGE+1;  // if the checksum is wrong then the data from this packet is invalid. 
		                                    // setting this ensures the nav routine does not try to use this data.
	}
	msg_parse = &msg_B3;
}

void gps_update_basic_data(void)
{
	week_no         = week_no_;
	svs             = svs_;
}

void gps_commit_data(void)
{
	//bin_out(0xFF);
	week_no         = week_no_;
	tow             = tow_;
	mlGpsData.time_usec = tow.WW;
	lat_gps         = lat_gps_;
	mlGpsData.lat = lat_gps.WW;
	lon_gps         = lon_gps_;
	mlGpsData.lon = lon_gps_.WW;
	alt_sl_gps.WW   = alt_sl_gps_.WW;          // SIRF provides altMSL in cm, UBX provides it in mm
	mlGpsData.alt = alt_sl_gps.WW;
	sog_gps.BB      = sog_gps_._.W0;                // SIRF uses 2 byte SOG, UBX provides 4 bytes
	mlGpsData.vel = sog_gps.BB;
//#if (HILSIM == 1)
	hilsim_airspeed.BB       = as_sim_._.W0;                 // provided by HILSIM, simulated airspeed
	
//#endif
	cog_gps.BB      = (uint16_t)(cog_gps_.WW / 1000);// SIRF uses 2 byte COG, 10^-2 deg, UBX provides 4 bytes, 10^-5 deg
	mlGpsData.cog = cog_gps.BB;
	climb_gps.BB    = - climb_gps_._.W0;            // SIRF uses 2 byte climb rate, UBX provides 4 bytes
	hdop            = (uint8_t)(hdop_.BB / 20);     // SIRF scales HDOP by 5, UBX by 10^-2
	mlGpsData.eph = hdop;
	mlGpsData.epv = 65535;
	// SIRF provides position in m, UBX provides cm
//	xpg.WW          = xpg_.WW / 100;
//	ypg.WW          = ypg_.WW / 100;
//	zpg.WW          = zpg_.WW / 100;
	// SIRF provides 2 byte velocity in m scaled by 8,
	// UBX provides 4 bytes in cm
//	xvg.BB          = (int16_t)(xvg_.WW / 100 * 8);
//	yvg.BB          = (int16_t)(yvg_.WW / 100 * 8);
//	zvg.BB          = (int16_t)(zvg_.WW / 100 * 8);
//	mode1           = mode1_;
//	mode2           = mode2_;
	svs             = svs_;
	mlGpsData.satellites_visible = svs;
	mlGpsData.fix_type = nav_valid_;

//#if (HILSIM == 1 && MAG_YAW_DRIFT == 1)
//	HILSIM_MagData(udb_magnetometer_callback); // run the magnetometer computations
//#endif // HILSIM
}

//#if (HILSIM == 1)
void commit_bodyrate_data(void)
{
	g_a_x_sim = g_a_x_sim_;
	g_a_y_sim = g_a_y_sim_;
	g_a_z_sim = g_a_z_sim_;
	p_sim = p_sim_;
	q_sim = q_sim_;
	r_sim = r_sim_;
	



}
#define RMAX                    16384//0b0100000000000000       // 1.0 in 2.14 fractional format

void HILSIM_saturate( uint16_t size , int16_t vector[3] )
{
	// hardware 16 bit signed integer gyro and accelerometer data and offsets
	// are divided by 2 prior to subtracting offsets from values.
	// This is done to prevent overflow.
	// However, it limits range to approximately +- RMAX.
	// Data coming in from Xplane is in 16 bit signed integer format with range +-2*RMAX,
	// so it needs to be passed through a saturation computation that limits to +-RMAX.
	uint16_t index ;
	for ( index = 0 ; index < size ; index ++ )
	{
		if ( vector[index ] > RMAX )
		{
			vector[index] = RMAX ;
		}
		if ( vector[index] < -RMAX )
		{
			vector[index] = -RMAX ;
		}
	}	
}
// gravity, as measured in plane coordinate system


void HILSIM_set_gplane(void)
{
	gplane[0] = g_a_y_sim.BB;
	gplane[1] = g_a_x_sim.BB;
	gplane[2] = g_a_z_sim.BB;
	HILSIM_saturate( 3, gplane ) ;
	aero_force[0] = - gplane[0] ;
	aero_force[1] = - gplane[1] ;
	aero_force[2] = - gplane[2] ;
	//from X-Plane interface adapter:
	// Accelerations are in m/s^2
	// Divide by 9.8 to get g's
	// Multiply by 5280 (constant from UDB code)
	// Divide by SCALEACCEL (2.64 for red board)
	// 1 / 9.8 * 5280 / 2.64 = 204.8
	mlRawImuData.xacc = gplane[1]  / 204.8 * 16384 / 9.815;//[1] is 'drg_axil' from  X-PLane
	mlRawImuData.yacc = -gplane[0] / 204.8 * 16384 / 9.815;//[0] is 'drg_side' from  X-PLane
	mlRawImuData.zacc = gplane[2]  / 204.8 * 16384 / 9.815;//[2] is 'drg_nrml' from  X-PLane

}
#define RADPERSEC ((int64_t)5632.0/SCALEGYRO)
// one radian per second, in AtoD/2 units
#ifdef WIN
extern fractional omegagyro[];

#else
fractional omegagyro[] = { 0, 0, 0 };

#endif

void HILSIM_set_omegagyro(void)
{
	omegagyro[0] = p_sim.BB;
	omegagyro[1] = q_sim.BB;
	omegagyro[2] = r_sim.BB;
	HILSIM_saturate( 3, omegagyro ) ;
	//I converted back to deg/sec since we have convertor inside the model to rad/sec
	//also converting to the sensor scale 
	// Angular rate
	// multiply by 5632 (constant from UDB code)
	// Divide by SCALEGYRO(3.0 for red board)
	// 1 * 5632 / 3.0 = 1877.33
	mlRawImuData.xgyro = -omegagyro[0] / 1877.33 * 65.5 / PI * 180 ;
	mlRawImuData.ygyro = -omegagyro[1] / 1877.33 * 65.5 / PI * 180 ;// I put the negative sign since HILSIM interface to X-PLane convert from NED to UDB
	mlRawImuData.zgyro = -omegagyro[2] / 1877.33 * 65.5 / PI * 180 ;
	

}
//#endif // HILSIM

void init_gps_ubx(void)
{
}

//#endif // (GPS_TYPE == GPS_UBX_2HZ || GPS_TYPE == GPS_UBX_4HZ || GPS_TYPE == GPS_ALL)

void gpsoutbin(int16_t length, const uint8_t msg[]) // output a binary message to the GPS
{
    uint8_t i;
	gps_out_buffer = 0; // clear the buffer pointer first, for safety, in case we're interrupted
	gps_out_index = 0;
	gps_out_buffer_length = length;
	gps_out_buffer = (uint8_t*)msg;

#ifdef WIN
    	udb_gps_start_sending_data();
#else
         for (i = 0U; i < length; i++) {
             Uart4OutBuff[i] = msg[i];
	}
        
#endif
}

void gpsoutline(const char *message) // output one NMEA line to the GPS
{
	gpsoutbin(strlen(message), (const uint8_t*)message);
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

// Got a character from the GPS
void udb_gps_callback_received_byte(uint8_t rxchar)
{
	//bin_out(rxchar);      // binary out to the debugging USART
	(*msg_parse)(rxchar);   // parse the input byte
}


boolean gps_nav_capable_check_set(void)
{
	boolean tmp ;
	if (gps_data_age < GPS_DATA_MAX_AGE) gps_data_age++;
	tmp = (gps_data_age < GPS_DATA_MAX_AGE);
	//	return (gps_data_age < GPS_DATA_MAX_AGE);
	return (tmp);// dcm_flags._.nav_capable;
}



// Received a full set of GPS messages
//void gps_parse_common(void)
//{
//	udb_background_trigger(&udb_background_callback_triggered);
//}

static void udb_background_callback_triggered(void)
{
//	union longbbbb accum;
//	union longww accum_velocity;
//	union longww longaccum;
//	int8_t cog_circular;
//	int8_t cog_delta;
//	int16_t sog_delta;
//	int16_t climb_rate_delta;
//#ifdef USE_EXTENDED_NAV
//	int32_t location[3];
//#else
//	int16_t location[3];
//	union longbbbb accum_nav;
//#endif // USE_EXTENDED_NAV
	/*int16_t location_deltaZ;
	struct relative2D location_deltaXY;
	struct relative2D velocity_thru_air;
	int16_t velocity_thru_airz;*/

#ifdef SLUGS2
	fractional rmat[] = { RMAX, 0, 0, 0, RMAX, 0, 0, 0, RMAX };
#endif
	// compute horizontal projection of air velocity,
	// taking into account the angle of attack.
	/*longaccum.WW = (__builtin_mulss(rmat[2], angleOfAttack)) << 2;
	dirOverGndHrmat[0] = rmat[1] + longaccum._.W1;
	longaccum.WW = (__builtin_mulss(rmat[5], angleOfAttack)) << 2;
	dirOverGndHrmat[1] = rmat[4] + longaccum._.W1;
	dirOverGndHrmat[2] = 0;*/

	if (gps_nav_valid())
	{
		gps_commit_data();


		gps_data_age = 0;
//#ifndef SLUGS2
//		dcm_callback_gps_location_updated();
//#endif

//#ifdef USE_EXTENDED_NAV
//		location[1] = ((lat_gps.WW - lat_origin.WW) / 90); // in meters, range is about 20 miles
//		location[0] = long_scale((lon_gps.WW - lon_origin.WW) / 90, cos_lat);
//		location[2] = (alt_sl_gps.WW - alt_origin.WW) / 100; // height in meters
//#else // USE_EXTENDED_NAV
//		accum_nav.WW = ((lat_gps.WW - lat_origin.WW) / 90); // in meters, range is about 20 miles
//		location[1] = accum_nav._.W0;
//		accum_nav.WW = long_scale((lon_gps.WW - lon_origin.WW) / 90, cos_lat);
//
//		location[0] = accum_nav._.W0;
//#ifdef USE_PRESSURE_ALT
//		#warning "using pressure altitude instead of GPS altitude"
//			// division by 100 implies alt_origin is in centimeters; not documented elsewhere
//			// longword result = (longword/10 - longword)/100 : range
//			accum_nav.WW = ((get_barometer_altitude() / 10) - alt_origin.WW) / 100; // height in meters
//#else
//		accum_nav.WW = (alt_sl_gps.WW - alt_origin.WW) / 100; // height in meters
//#endif // USE_PRESSURE_ALT
//		location[2] = accum_nav._.W0;
//#endif // USE_EXTENDED_NAV
//
//		// convert GPS course of 360 degrees to a binary model with 256
//		accum.WW = __builtin_muluu(COURSEDEG_2_BYTECIR, cog_gps.BB) + 0x00008000;
//		// re-orientate from compass (clockwise) to maths (anti-clockwise) with 0 degrees in East
//		cog_circular = -accum.__.B2 + 64;
//
		// compensate for GPS reporting latency.
		// The dynamic model of the EM406 and uBlox is not well known.
		// However, it seems likely much of it is simply reporting latency.
		// This section of the code compensates for reporting latency.
		// markw: what is the latency? It doesn't appear numerically or as a comment
		// in the following code. Since this method is called at the GPS reporting rate
		// it must be assumed to be one reporting interval?

//		if (dcm_flags._.gps_history_valid)
//		{
//			cog_delta = cog_circular - cog_previous;
//			sog_delta = sog_gps.BB - sog_previous;
//			climb_rate_delta = climb_gps.BB - climb_rate_previous;
//
//			location_deltaXY.x = location[0] - location_previous[0];
//			location_deltaXY.y = location[1] - location_previous[1];
//			location_deltaZ = location[2] - location_previous[2];
//		}
//		else
//		{
//			cog_delta = sog_delta = climb_rate_delta = 0;
//			location_deltaXY.x = location_deltaXY.y = location_deltaZ = 0;
//		}
//		dcm_flags._.gps_history_valid = 1;
//		actual_dir = cog_circular + cog_delta;
//		cog_previous = cog_circular;
//
//		// Note that all these velocities are in centimeters / second
//
//		ground_velocity_magnitudeXY = sog_gps.BB + sog_delta;
//		sog_previous = sog_gps.BB;
//
//		GPSvelocity.z = climb_gps.BB + climb_rate_delta;
//		climb_rate_previous = climb_gps.BB;
//		accum_velocity.WW = (__builtin_mulss(cosine(actual_dir), ground_velocity_magnitudeXY) << 2) + 0x00008000;
//		GPSvelocity.x = accum_velocity._.W1;
//		accum_velocity.WW = (__builtin_mulss(sine(actual_dir), ground_velocity_magnitudeXY) << 2) + 0x00008000;
//
//		GPSvelocity.y = accum_velocity._.W1;
//		rotate_2D(&location_deltaXY, cog_delta); // this is a key step to account for rotation effects!!
//
//		GPSlocation.x = location[0] + location_deltaXY.x;
//		GPSlocation.y = location[1] + location_deltaXY.y;
//		GPSlocation.z = location[2] + location_deltaZ;
//
//		location_previous[0] = location[0];
//		location_previous[1] = location[1];
//		location_previous[2] = location[2];
//
//#ifndef SLUGS2
//		velocity_thru_air.y = GPSvelocity.y - estimatedWind[1];
//		velocity_thru_air.x = GPSvelocity.x - estimatedWind[0];
//		velocity_thru_airz = GPSvelocity.z - estimatedWind[2];
//#else
//		velocity_thru_air.y = GPSvelocity.y;
//		velocity_thru_air.x = GPSvelocity.x;
//		velocity_thru_airz = GPSvelocity.z;
//#endif

//#if (HILSIM == 1)
//		air_speed_3DGPS = hilsim_airspeed.BB; // use Xplane as a pitot
//#else
//		air_speed_3DGPS = vector3_mag(velocity_thru_air.x, velocity_thru_air.y, velocity_thru_airz);
//#endif
//
//		calculated_heading = rect_to_polar(&velocity_thru_air);
//
//		// veclocity_thru_air.x becomes XY air speed as a by product of CORDIC routine in rect_to_polar()
//		air_speed_magnitudeXY = velocity_thru_air.x; // in cm / sec
//
//#if (GPS_RATE == 4)
//		forward_acceleration = (air_speed_3DGPS - velocity_previous) << 2; // Ublox enters code 4 times per second
//#elif (GPS_RATE == 2)
//		forward_acceleration = (air_speed_3DGPS - velocity_previous) << 1; // Ublox enters code 2 times per second
//#else
//		forward_acceleration = (air_speed_3DGPS - velocity_previous);      // EM406 standard GPS enters code once per second
//#endif
//
//		velocity_previous = air_speed_3DGPS;
//#ifndef SLUGS2
//
//		estimateWind();
//		estAltitude();
//		estYawDrift();
//#endif
//		dcm_flags._.yaw_req = 1;       // request yaw drift correction
//		dcm_flags._.reckon_req = 1;    // request dead reckoning correction
//		dcm_flags._.rollpitch_req = 1;
//#if (DEADRECKONING == 0)
//		process_flightplan();
//#endif
	}
	else
	{
		gps_data_age = GPS_DATA_MAX_AGE + 1;
		//dirOverGndHGPS[0] = dirOverGndHrmat[0];
		//dirOverGndHGPS[1] = dirOverGndHrmat[1];
		//dirOverGndHGPS[2] = 0;
		//dcm_flags._.yaw_req = 1;            // request yaw drift correction
		//dcm_flags._.gps_history_valid = 0;  // gps history has to be restarted
		gps_update_basic_data();            // update svs
	}
}
#define CHANNEL_1               1
#define CHANNEL_2               2
#define CHANNEL_3               3
#define CHANNEL_4               4
#define CHANNEL_5               5
#define THROTTLE_OUTPUT_CHANNEL             CHANNEL_3
#define AILERON_OUTPUT_CHANNEL              CHANNEL_1
#define ELEVATOR_OUTPUT_CHANNEL             CHANNEL_2
#define RUDDER_OUTPUT_CHANNEL               CHANNEL_4
#define NUM_OUTPUTS                         4
#define HILSIM_NUM_SERVOS 8
#define MAX_OUTPUTS     8
uint8_t SIMservoOutputs[] = {
	0xFF, 0xEE, //sync
	0x03, 0x04, //S1
	0x05, 0x06, //S2
	0x07, 0x08, //S3
	0x09, 0x0A, //S4
	0x0B, 0x0C, //S5
	0x0D, 0x0E, //S6
	0x0F, 0x10, //S7
	0x11, 0x12, //S8
	0x13, 0x14  //checksum
};
uint16_t send_HILSIM_outputs(void)
{
	// Setup outputs for HILSIM
	int16_t i;
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	union intbb TempBB;
    
    int16_t udb_pwOut[MAX_OUTPUTS]; // pulse widths for servo outputs

	udb_pwOut[AILERON_OUTPUT_CHANNEL] = mlPwmCommands.servo2_raw *10 ;//
	udb_pwOut[THROTTLE_OUTPUT_CHANNEL] = mlPwmCommands.servo1_raw * 10;//
	udb_pwOut[RUDDER_OUTPUT_CHANNEL] = mlPwmCommands.servo3_raw * 10;//
	udb_pwOut[ELEVATOR_OUTPUT_CHANNEL] = mlPwmCommands.servo4_raw * 10 ;//
	for (i = 1; i <= NUM_OUTPUTS; i++)
	{
		
		TempBB.BB = udb_pwOut[i];
		SIMservoOutputs[2 * i] = TempBB._.B1;
		SIMservoOutputs[(2 * i) + 1] = TempBB._.B0;
	}

	for (i = 2; i < HILSIM_NUM_SERVOS * 2 + 2; i++)
	{
		CK_A += SIMservoOutputs[i];
		CK_B += CK_A;
	}
	SIMservoOutputs[i] = CK_A;
	SIMservoOutputs[i + 1] = CK_B;

	// Send HILSIM outputs
	gpsoutbin(HILSIM_NUM_SERVOS * 2 + 4, SIMservoOutputs);

    return HILSIM_NUM_SERVOS * 2 + 4;
}