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


#ifndef AUAV3_DEFINES_H
#define AUAV3_DEFINES_H

#include <stdint.h>


//#if SILSIM
#if (SILSIM == 1)
#define NUM_POINTERS_IN(x)      (sizeof(x)/sizeof(char*))
#else
#define NUM_POINTERS_IN(x)      (sizeof(x)>>1)
#endif

// Build for the specific board type
#define RED_BOARD               1   // red board with vertical LISY gyros (deprecated)
#define GREEN_BOARD             2   // green board with Analog Devices 75 degree/second gyros (deprecated)
#define UDB3_BOARD              3   // red board with daughter boards 500 degree/second Invensense gyros (deprecated)
#define RUSTYS_BOARD            4   // Red board with Rusty's IXZ-500_RAD2a patch board (deprecated)
#define UDB4_BOARD              5   // board with dsPIC33 and integrally mounted 500 degree/second Invensense gyros
#define CAN_INTERFACE           6
#define AUAV2_BOARD             7   // Nick Arsov's AUAV2 with dsPIC33 and MPU6000
#define UDB5_BOARD              8   // board with dsPIC33 and MPU6000
#define AUAV3_BOARD             9   // Nick Arsov's AUAV3 with dsPIC33EP and MPU6000
#define AUAV4_BOARD             10  // AUAV4 with PIC32MX

#if (SILSIM == 0)

// Include the necessary files for the current board type

#include "ConfigAUAV3.h"

#endif // SILSIM

#if (HILSIM == 1)
#include "ConfigHILSIM.h"
#endif

#if (USE_PPM_INPUT == 1 )
#undef MAX_INPUTS
#define MAX_INPUTS              8
#undef MAX_OUTPUTS
#define MAX_OUTPUTS             9
#endif


// define the board rotations here.
// This include must go just after the board type has been declared
// Do not move this
// Orientation of the board
#define ORIENTATION_FORWARDS    0
#define ORIENTATION_BACKWARDS   1
#define ORIENTATION_INVERTED    2
#define ORIENTATION_FLIPPED     3
#define ORIENTATION_ROLLCW      4
#define ORIENTATION_ROLLCW180   5
#define ORIENTATION_YAWCW       6
#define ORIENTATION_YAWCCW      7

#include "boardRotation_defines.h"


// Clock configurations
#define CLOCK_CONFIG            3   // legacy definition for telemetry output


// Dead reckoning
// DEADRECKONING 0 selects the GPS to perform navigation, at the GPS update rate.
// DEADRECKONING 1 selects the dead reckoning computations to perform navigation, at 40 Hz.
#ifndef DEADRECKONING           // define only if not already defined in options.h
#define DEADRECKONING           1
#endif

// Wind Estimation and Navigation
// Set this to 1 to use automatic wind estimation and navigation. 
// Wind estimation is done using a mathematical model developed by William Premerlani.
// Every time the plane performs a significant turn, the plane estimates the wind.
// This facility only requires a working GPS and the UAV DevBoard. 
#ifndef WIND_ESTIMATION         // define only if not already defined in options.h
#define WIND_ESTIMATION         1
#endif

// Enforce that if DEADRECKONING is on, WIND_ESTIMATION must be on as well.
// Using dead reckoning in high winds without wind estimation will cause large
// errors in the dead reckoning.
#if (DEADRECKONING == 1 && WIND_ESTIMATION == 0)
#undef WIND_ESTIMATION
#define WIND_ESTIMATION         1
#endif


// Types
//#ifndef SIL_WINDOWS_INCS
typedef uint8_t boolean;
//#endif
#define true                    1
#define false                   0

struct ADchannel {
	int16_t input;  // raw input
	int16_t value;  // average of the sum of inputs between report outs
	int16_t offset; // baseline at power up 
	int32_t sum;    // used as an integrator
}; // variables for processing an AD channel


struct udb_flag_bits {
	uint16_t unused : 12;
	uint16_t a2d_read : 1;
	uint16_t radio_on : 1;
	uint16_t sonar_updated : 1;
	uint16_t sonar_print_telemetry : 1;
};


// LED states
#define LED_ON                  0
#define LED_OFF                 1



// Channel numbers on the board, mapped to positions in the pulse width arrays.
#define CHANNEL_UNUSED          0   // udb_pwIn[0], udb_pwOut[0], etc. are not used, but allow lazy code everywhere else  :)
#define CHANNEL_1               1
#define CHANNEL_2               2
#define CHANNEL_3               3
#define CHANNEL_4               4
#define CHANNEL_5               5
#define CHANNEL_6               6
#define CHANNEL_7               7
#define CHANNEL_8               8
#define CHANNEL_9               9
#define CHANNEL_10              10
#define CHANNEL_11              11
#define CHANNEL_12              12
#define CHANNEL_13              13
#define CHANNEL_14              14
#define CHANNEL_15              15
#define CHANNEL_16              16


// Constants
#define RMAX                    16384//0b0100000000000000       // 1.0 in 2.14 fractional format
#define GRAVITY                 ((int32_t)(5280.0/SCALEACCEL))  // gravity in AtoD/2 units

#define SERVOCENTER             3000
#define SERVORANGE              ((int16_t)(SERVOSAT*1000))
#define SERVOMAX                (SERVOCENTER + SERVORANGE)
#define SERVOMIN                (SERVOCENTER - SERVORANGE)

extern int16_t magMessage;
extern int16_t vref_adj;

#define NETWORK_INTERFACE_NONE                  0
#define NETWORK_INTERFACE_WIFI_MRF24WG          1
#define NETWORK_INTERFACE_ETHERNET_ENC624J600   2
#define NETWORK_INTERFACE_ETHERNET_ENC28J60     3

//from option.h
// NUM_INPUTS:
// For classic boards: Set to 1-5 (or 1-8 when using PPM input)
//   1-4 enables only the first 1-4 of the 4 standard input channels
//   5 also enables E8 as the 5th input channel
// For UDB4 boards: Set to 1-8
#define NUM_INPUTS                          5

// Channel numbers for each input.
// Use as is, or edit to match your setup.
//   - If you're set up to use Rudder Navigation (like MatrixNav), then you may want to swap
//     the aileron and rudder channels so that rudder is CHANNEL_1, and aileron is 5.
#define THROTTLE_INPUT_CHANNEL              CHANNEL_3
#define AILERON_INPUT_CHANNEL               CHANNEL_1
#define ELEVATOR_INPUT_CHANNEL              CHANNEL_2
#define RUDDER_INPUT_CHANNEL                CHANNEL_5
#define MODE_SWITCH_INPUT_CHANNEL           CHANNEL_4
#define CAMERA_PITCH_INPUT_CHANNEL          CHANNEL_UNUSED
#define CAMERA_YAW_INPUT_CHANNEL            CHANNEL_UNUSED
#define CAMERA_MODE_INPUT_CHANNEL           CHANNEL_UNUSED
#define OSD_MODE_SWITCH_INPUT_CHANNEL       CHANNEL_UNUSED
#define PASSTHROUGH_A_INPUT_CHANNEL         CHANNEL_UNUSED
#define PASSTHROUGH_B_INPUT_CHANNEL         CHANNEL_UNUSED
#define PASSTHROUGH_C_INPUT_CHANNEL         CHANNEL_UNUSED
#define PASSTHROUGH_D_INPUT_CHANNEL         CHANNEL_UNUSED

// NUM_OUTPUTS:
//   NOTE: If USE_PPM_INPUT is enabled above, up to 9 outputs are available.)
// For UDB4/5 boards: Set to 3-8 (or up to 10 using pins RA4 and RA1.)
// For AUAV3 boards:  Set to 3-8 (or up to 11 using pins RE1, RA6 and RA7.)
//                               (this needs developing, so contact the list)
#define NUM_OUTPUTS                         4

// Channel numbers for each output
// Use as is, or edit to match your setup.
//   - Only assign each channel to one output purpose
//   - If you don't want to use an output channel, set it to CHANNEL_UNUSED
//   - If you're set up to use Rudder Navigation (like MatrixNav), then you may want to swap
//     the aileron and runner channels so that rudder is CHANNEL_1, and aileron is 5.
//
// NOTE: If your board is powered from your ESC through the throttle cable, make sure to
// connect THROTTLE_OUTPUT_CHANNEL to one of the built-in Outputs (1, 2, or 3) to make
// sure your board gets power.
//
#define THROTTLE_OUTPUT_CHANNEL             CHANNEL_3
#define AILERON_OUTPUT_CHANNEL              CHANNEL_1
#define ELEVATOR_OUTPUT_CHANNEL             CHANNEL_2
#define RUDDER_OUTPUT_CHANNEL               CHANNEL_4
#define AILERON_SECONDARY_OUTPUT_CHANNEL    CHANNEL_UNUSED
#define CAMERA_PITCH_OUTPUT_CHANNEL         CHANNEL_UNUSED
#define CAMERA_YAW_OUTPUT_CHANNEL           CHANNEL_UNUSED
#define TRIGGER_OUTPUT_CHANNEL              CHANNEL_UNUSED
#define PASSTHROUGH_A_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define PASSTHROUGH_B_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define PASSTHROUGH_C_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define PASSTHROUGH_D_OUTPUT_CHANNEL        CHANNEL_UNUSED

////////////////////////////////////////////////////////////////////////////////
// Mode Switch is ideally controlled by a 3-position switch on your transmitter.
// Often the Flap channel will be controlled by a 3-position switch.
// These are the thresholds for the cutoffs between low and middle, and between middle and high.
// Normal signals should fall within about 2000 - 4000.
#define MODE_SWITCH_THRESHOLD_LOW           2600
#define MODE_SWITCH_THRESHOLD_HIGH          3400

////////////////////////////////////////////////////////////////////////////////
// The Failsafe Channel is the RX channel that is monitored for loss of signal
// Make sure this is set to a channel you actually have plugged into the UAV Dev Board!
//
// For a receiver that remembers a failsafe value for when it loses the transmitter signal,
// like the Spektrum AR6100, you can program the receiver's failsafe value to a value below
// the normal low value for that channel.  Then set the FAILSAFE_INPUT_MIN value to a value
// between the receiver's programmed failsafe value and the transmitter's normal lowest
// value for that channel.  This way the firmware can detect the difference between a normal
// signal, and a lost transmitter.
//
// FAILSAFE_INPUT_MIN and _MAX define the range within which we consider the radio on.
// Normal signals should fall within about 2000 - 4000.
#define FAILSAFE_INPUT_CHANNEL              THROTTLE_INPUT_CHANNEL
#define FAILSAFE_INPUT_MIN                  1500
#define FAILSAFE_INPUT_MAX                  4500

// SERVOSAT limits servo throw by controlling pulse width saturation.
// set it to 1.0 if you want full servo throw, otherwise set it to the portion that you want
#define SERVOSAT                            1.0

//from udbtypes.h

// UDB Types
struct bb   { uint8_t B0; uint8_t B1; };
struct bbbb { uint8_t B0; uint8_t B1; uint8_t B2; uint8_t B3; };
struct ww   { int16_t W0; int16_t W1; };
struct wwww { int16_t W0; int16_t W1; int16_t W2; int16_t W3; };
struct LL   { int32_t L0; int32_t L1; };

union intbb      { int16_t BB;  struct bb _; };
union uintbb     { uint16_t BB; struct bb _; };
union longbbbb   { int32_t WW;  struct ww _; struct bbbb __; };
union longww     { int32_t WW;  struct ww _; };
union longlongLL { int64_t LL;  struct LL _; struct wwww __; };

//from libUDB.h

#define inline __inline
#define SILSIM                              1
#undef  HILSIM
#define HILSIM                              1

#define DPRINT(args, ...)
////////////////////////////////////////////////////////////////////////////////
// Run Background Tasks

// Implement this callback to perform periodic background tasks (high priority).
// It is called at 40 Hertz and must return quickly. (No printf!)
void udb_heartbeat_40hz_callback(void);

// Implement this callback to prepare the pwOut values.
// It is called at HEARTBEAT_HZ at a low priority.
void udb_heartbeat_callback(void);

typedef void(*background_callback)(void);

// Trigger the background_callback() functions from a low priority ISR.
void udb_background_trigger(background_callback callback);
void udb_background_trigger_pulse(background_callback callback);

// Return the current CPU load as an integer percentage value from 0-100.
uint8_t udb_cpu_load(void);
//inline void cpu_load_calc(void);

////////////////////////////////////////////////////////////////////////////////
// Radio Inputs / Servo Outputs

// These are the values of the radio input channels.  Each channel will be a
// value between approximately 2000 and 4000, with 3000 being the center.
// Treat udb_pwIn values as readonly.
extern int16_t udb_pwIn[];                  // pulse widths of radio inputs

// These are the recorded trim values of the radio input channels.
// These values are recorded when you call the udb_servo_record_trims()
// function.
// Each channel will be a value between approximately 2000 and 4000.
// Treat udb_pwTrim values as readonly.
extern int16_t udb_pwTrim[];                // initial pulse widths for trimming

// These are the servo channel values that will be sent out to the servos.
// Set these values in your implementation of the udb_heartbeat_callback()
// Each channel should be set to a value between 2000 and 4000.
extern int16_t udb_pwOut[];                 // pulse widths for servo outputs

// This read-only value holds flags that tell you, among other things,
// whether the receiver is currently receiving values from the transmitter.
extern union udb_fbts_byte { struct udb_flag_bits _; int8_t B; } udb_flags;

// This takes a servo out value, and clips it to be within
// 3000-1000*SERVOSAT and 3000+1000*SERVOSAT (2000-4000 by default).
int16_t udb_servo_pulsesat(int32_t pw);

// Call this funtion once at some point soon after
// the UDB has booted up and the radio is on.
void udb_servo_record_trims(void);

// Called immediately whenever the radio_on flag is set to 0
void udb_callback_radio_did_turn_off(void);     // Callback

// Call this function to set the digital output to 0 or 1.
// This can be used to do things like triggering cameras, turning on
// lights, etc.
void udb_set_action_state(uint8_t/*boolean*/ newValue);

	  //from hearthbeat.h
// number of heartbeats per second
#define HEARTBEAT_HZ 40

// number of servo updates per second
#define SERVO_HZ 40

// frequency of PID loop (HEARTBEAT_HZ / PID_HZ must be an integer)
#define PID_HZ 40

#endif // AUAV3_DEFINES_H
