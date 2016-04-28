#include "UnitTest.h"
#include "UnitTest_private.h"

/* Implement SPI 1 Interrupts */
void __attribute__((__interrupt__,__shadow__,__auto_psv__)) _SPI1Interrupt(void)
  /* SPI1 ISR */
{
  _SPI1IF = 0;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;

  /* Declaration of Variables */
  uint16_T tmp;
  switch (MCHP_SPI1_State)
  {
   case 0:                             /* Idle */
    break;

   case 1:                             /* Start a new SPI Sequence */
    /* Toggle pin */
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(700000);                 /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    SPI1BUF = 0x6B80;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 2:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(4200000);                /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    tmp = SPI1BUF;
    SPI1BUF = 0x6B01;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 3:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(700000);                 /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    tmp = SPI1BUF;
    SPI1BUF = 0x6A10;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 4:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(700000);                 /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    tmp = SPI1BUF;
    SPI1BUF = 0x1904;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 5:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(700000);                 /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    tmp = SPI1BUF;
    SPI1BUF = 0x1A03;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 6:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */

    /* Delay */
    __delay32(700000);                 /* Delay  */

    /* Toggle pin */
    LATEbits.LATE4 = 0;                /* switch RE4 to 0 */

    /* Write only Sequence */
    tmp = SPI1BUF;
    SPI1BUF = 0x1B08;

    /* Toggle pin */
    MCHP_SPI1_State++;
    break;

   case 7:
    LATEbits.LATE4 = 1;                /* switch RE4 to 1 */
    tmp = SPI1BUF;
    MCHP_SPI1_State = 0;               /* End of SPI Sequence. SPI is available for a new sequence */
    break;

   default:                            /* Sequence finished */
    MCHP_SPI1_State = 0;               /* Should never happend */
    break;
  }                                    /* End of switch case sequence*/

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}                                      /* Enf of interrupt */
