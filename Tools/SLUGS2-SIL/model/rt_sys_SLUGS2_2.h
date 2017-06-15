/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_2.h
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#ifndef RTW_HEADER_rt_sys_SLUGS2_2_h_
#define RTW_HEADER_rt_sys_SLUGS2_2_h_
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

typedef struct MCHP_I2C2_QueueStr{
  volatile uint16_T buffer[6];         /* Size is equal to the number or distinct sequence set +1 */
  uint16_T tail;                       /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint16_T head;              /* head is the index for the next value to be written into the Circular buffer */
} MCHP_I2C2_QueueStr;

typedef struct MCHP_SPI1_QueueStr{
  volatile uint16_T buffer[3];         /* Size is equal to the number or distinct sequence set +1 */
  uint16_T tail;                       /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint16_T head;              /* head is the index for the next value to be written into the Circular buffer */
} MCHP_SPI1_QueueStr;

/* Declare UART1 Tx Circular Buffer Structure */
#define Tx_BUFF_SIZE_Uart1             (1024)

typedef struct MCHP_UART1_TxStr{
  volatile uint8_T buffer[Tx_BUFF_SIZE_Uart1];/* Size Rx_BUFF_SIZE_Uart1 is 1024 */
  uint_T tail;                         /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint_T head;                /* head is the index for the next value to be written into the Circular buffer */
} MCHP_UART1_TxStr;

/* Declare UART4 Tx Circular Buffer Structure */
#define Tx_BUFF_SIZE_Uart4             (512)

typedef struct MCHP_UART4_TxStr{
  volatile uint8_T buffer[Tx_BUFF_SIZE_Uart4];/* Size Rx_BUFF_SIZE_Uart4 is 512 */
  uint_T tail;                         /* tail is the index for the next value to be read from the Circular buffer */
  volatile uint_T head;                /* head is the index for the next value to be written into the Circular buffer */
} MCHP_UART4_TxStr;

/* Block signals for system '<S24>/Enabled Subsystem' */
typedef struct {
  real32_T In1;                        /* '<S34>/In1' */
} rtB_EnabledSubsystem_SLUGS2_T;

extern void SLU_EnabledSubsystem_Start(rtB_EnabledSubsystem_SLUGS2_T *localB);
extern void SLUGS2_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_SLUGS2_T *localB);

#endif                                 /* RTW_HEADER_rt_sys_SLUGS2_2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
