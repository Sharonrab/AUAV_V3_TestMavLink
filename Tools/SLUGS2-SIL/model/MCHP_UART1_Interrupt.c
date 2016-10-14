#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Declare UART1 Tx Circular Buffer Structure */
MCHP_UART1_TxStr MCHP_UART1_Tx;
void __attribute__((__interrupt__,__auto_psv__)) _U1TXInterrupt(void)
{
  _U1TXIF = 0;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T3CONbits.TON = 1;
  while ((U1STAbits.UTXBF == 0) && (MCHP_UART1_Tx.tail != MCHP_UART1_Tx.head) )/* while UxTXREG buffer is not full */
  {
    U1TXREG = MCHP_UART1_Tx.buffer[MCHP_UART1_Tx.head];
    MCHP_UART1_Tx.head = (MCHP_UART1_Tx.head + 1) & (Tx_BUFF_SIZE_Uart1-1);
  }

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }
}
