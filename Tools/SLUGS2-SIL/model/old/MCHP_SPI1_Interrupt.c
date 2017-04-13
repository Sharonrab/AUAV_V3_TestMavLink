#include "SLUGS2.h"
#include "SLUGS2_private.h"

/* Implement SPI 1 Interrupts */
void __attribute__((__interrupt__,__shadow__,__auto_psv__)) _SPI1Interrupt(void)
  /* SPI1 ISR */
{
  boolean_T Continue;
  _SPI1IF = 0;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T3CONbits.TON = 1;

  /* Declaration of Variables */
  uint16_T tmp;
  do {
    Continue = 0;                      /* By default, execute one sequence only */
    switch (MCHP_SPI1_State)
    {
     case 0:                           /* Idle */
      /* Check if the next sequence is already in the queue. If so, start it */
      Continue = (MCHP_SPI1_Queue.head != MCHP_SPI1_Queue.tail);/* One next sequence queued, start it now */
      if (Continue) {
        MCHP_SPI1_State = MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.tail];
        if (MCHP_SPI1_Queue.tail >= 2) /* There are 2 SPI1 blocks, max idx for queue is 2 */
          MCHP_SPI1_Queue.tail = 0;
        else
          MCHP_SPI1_Queue.tail++;
      }
      break;

     case 1:                           /* Start a new SPI Sequence */
      /* Toggle pin */
      LATEbits.LATE4 = 0;              /* switch RE4 to 0 */

      /* Write only Sequence */
      SPI1BUF = 0xBA00;

      /* Read only Sequence */
      SPI1BUF = 0x0000;
      SPI1BUF = 0x0000;
      SPI1BUF = 0x0000;

      /* Read only Sequence */
      SPI1BUF = 0x0000;

      /* Read only Sequence */
      SPI1BUF = 0x0000;
      SPI1BUF = 0x0000;
      SPI1BUF = 0x0000;
      MCHP_SPI1_State++;
      break;

     case 2:
      /* Toggle pin */
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */
      tmp = SPI1BUF;
      SPI11_Buff16[0] = SPI1BUF;
      SPI11_Buff16[1] = SPI1BUF;
      SPI11_Buff16[2] = SPI1BUF;
      SPI11_Buff16[3] = SPI1BUF;
      SPI11_Buff16[4] = SPI1BUF;
      SPI11_Buff16[5] = SPI1BUF;
      SPI11_Buff16[6] = SPI1BUF;
      MCHP_SPI11_Request = 0;          /* This SPI sequence could be Re-Enabled */
      MCHP_SPI1_State = 0;             /* End of SPI Sequence. SPI Interrupt should be started for new sequence */
      Continue = (MCHP_SPI1_Queue.head != MCHP_SPI1_Queue.tail);/* One next sequence queued, start it now */
      break;

     case 3:                           /* Start a new SPI Sequence */
      /* Toggle pin */
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */

      /* Delay */
      __delay32(700000);               /* Delay  */

      /* Toggle pin */
      LATEbits.LATE4 = 0;              /* switch RE4 to 0 */

      /* Write only Sequence */
      SPI1BUF = 0x6A80;

      /* Toggle pin */
      MCHP_SPI1_State++;
      break;

     case 4:
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */

      /* Delay */
      __delay32(4200000);              /* Delay  */

      /* Toggle pin */
      LATEbits.LATE4 = 0;              /* switch RE4 to 0 */

      /* Write only Sequence */
      tmp = SPI1BUF;
      SPI1BUF = 0x6B01;

      /* Toggle pin */
      MCHP_SPI1_State++;
      break;

     case 5:
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */

      /* Delay */
      __delay32(700000);               /* Delay  */

      /* Toggle pin */
      LATEbits.LATE4 = 0;              /* switch RE4 to 0 */

      /* Write only Sequence */
      tmp = SPI1BUF;
      SPI1BUF = 0x1B08;

      /* Toggle pin */
      MCHP_SPI1_State++;
      break;

     case 6:
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */

      /* Delay */
      __delay32(700000);               /* Delay  */

      /* Toggle pin */
      LATEbits.LATE4 = 0;              /* switch RE4 to 0 */

      /* Write only Sequence */
      tmp = SPI1BUF;
      SPI1BUF = 0x6A10;

      /* Toggle pin */
      MCHP_SPI1_State++;
      break;

     case 7:
      LATEbits.LATE4 = 1;              /* switch RE4 to 1 */
      tmp = SPI1BUF;
      MCHP_SPI12_Request = 0;          /* This SPI sequence could be Re-Enabled */
      MCHP_SPI1_State = 0;             /* End of SPI Sequence. SPI Interrupt should be started for new sequence */
      Continue = (MCHP_SPI1_Queue.head != MCHP_SPI1_Queue.tail);/* One next sequence queued, start it now */
      break;

     default:                          /* Sequence finished */
      MCHP_SPI1_State = 0;             /* Should never happend */
      break;
    }                                  /* End of switch case sequence*/
  } while (Continue == 1);             /* end do-While */

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }
}                                      /* Enf of interrupt */
