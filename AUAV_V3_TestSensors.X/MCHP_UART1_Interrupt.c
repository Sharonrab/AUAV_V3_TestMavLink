#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* UART Config - Interrupt: <Root>/UART Configuration UAV V3 UART 3 */

/* Declare UART1 Tx Circular Buffer Structure */
MCHP_UART1_TxStr MCHP_UART1_Tx;
void __attribute__((__interrupt__,__auto_psv__)) _U1TXInterrupt(void)
{
  _U1TXIF = 0;                         /*  */
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T2CONbits.TON = 1;
  while ((U1STAbits.UTXBF == 0) && (MCHP_UART1_Tx.tail != MCHP_UART1_Tx.head) )/* while UxTXREG buffer is not full */
  {
    U1TXREG = MCHP_UART1_Tx.buffer[MCHP_UART1_Tx.head];
    MCHP_UART1_Tx.head = (MCHP_UART1_Tx.head + 1) & (Tx_BUFF_SIZE_Uart1-1);
  }

  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}
