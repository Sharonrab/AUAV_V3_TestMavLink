/* Declaration of Global Variable for SPI 1 Peripheral */

#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h"

/* SPI - Interrupt: <Root>/BUS SPI Initialize MPU 6000 Once at Startup */
volatile uint16_T MCHP_SPI1_State = 0;
boolean_T volatile MCHP_SPI12_Request = 0;

/* Declare SPI1 Queue Circular Buffer */
MCHP_SPI1_QueueStr MCHP_SPI1_Queue;
boolean_T volatile MCHP_SPI11_Request = 0;
volatile uint16_T SPI11_Buff16[7];
