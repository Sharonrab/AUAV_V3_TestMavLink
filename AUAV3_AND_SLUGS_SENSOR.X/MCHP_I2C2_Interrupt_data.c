/* Declaration of Global Variable for I2C 2 Peripheral */

#include "AUAV3_AND_SLUGS_SENSOR.h"
#include "AUAV3_AND_SLUGS_SENSOR_private.h"

volatile uint16_T MCHP_I2C2_State = 0;
unsigned int volatile MCHP_I2C22_Request = 0;

/* Declare I2C2 Queue Circular Buffer */
MCHP_I2C2_QueueStr MCHP_I2C2_Queue;
volatile uint8_T I2C22_Buff8[2];
unsigned int volatile MCHP_I2C23_Request = 0;
volatile uint8_T I2C23_Buff8[2];
unsigned int volatile MCHP_I2C21_Request = 0;
volatile uint8_T I2C21_Buff8[22];
unsigned int volatile MCHP_I2C24_Request = 0;
volatile uint8_T I2C24_Buff8[6];
unsigned int volatile MCHP_I2C25_Request = 0;
