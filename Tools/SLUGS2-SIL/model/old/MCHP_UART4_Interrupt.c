#include "AUAV_V3_TestMavLink.h"
#include "AUAV_V3_TestMavLink_private.h"

/* Declare UART4 Tx Circular Buffer Structure */
MCHP_UART4_TxStr MCHP_UART4_Tx;
void __attribute__((__interrupt__,__auto_psv__)) _U4TXInterrupt(void)
{
  _U4TXIF = 0;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while ((U4STAbits.UTXBF == 0) && (MCHP_UART4_Tx.tail != MCHP_UART4_Tx.head) )/* while UxTXREG buffer is not full */
  {
    U4TXREG = MCHP_UART4_Tx.buffer[MCHP_UART4_Tx.head];
    MCHP_UART4_Tx.head = (MCHP_UART4_Tx.head + 1) & (Tx_BUFF_SIZE_Uart4-1);
  }

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}
