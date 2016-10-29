#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* UART Config - Interrupt: <Root>/UART Configuration UAV V3 UART 4 GPS */

/* Declare UART4 Tx Circular Buffer Structure */
MCHP_UART4_TxStr MCHP_UART4_Tx;
void __attribute__((__interrupt__,__auto_psv__)) _U4TXInterrupt(void)
{
  _U4TXIF = 0;                         /*  */
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T2CONbits.TON = 1;
  while ((U4STAbits.UTXBF == 0) && (MCHP_UART4_Tx.tail != MCHP_UART4_Tx.head) )/* while UxTXREG buffer is not full */
  {
    U4TXREG = MCHP_UART4_Tx.buffer[MCHP_UART4_Tx.head];
    MCHP_UART4_Tx.head = (MCHP_UART4_Tx.head + 1) & (Tx_BUFF_SIZE_Uart4-1);
  }

  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}
