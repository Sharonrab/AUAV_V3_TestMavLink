#include "UnitTest.h"
#include "UnitTest_private.h"

/* Implement I2C 2 Interrupts */
void __attribute__((__interrupt__,__auto_psv__)) _MI2C2Interrupt(void) /* MI2C2 ISR */
{
  _MI2C2IF = 0;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;

  /* Declaration of Variables */
  switch (MCHP_I2C2_State)
  {
   case 0:                             /* Idle */
    break;

   case 1:                             /* Problem: try Reset of I2C BUS : Repeated Start sequence and Stop */
    TRISAbits.TRISA3 = 1;              /* Set I2C Port as Port Input (better for Fj chips)*/
    TRISAbits.TRISA2 = 1;
    I2C2CONbits.I2CEN = 1;             /* Re-Enable I2C*/
    I2C2STAT &= 65526;                 /* Reset I2C State */
    I2C2CONbits.RSEN = 1;              /* I2C: Restart */
    MCHP_I2C2_State++;
    break;

   case 2:                             /* Problem: try Reset of I2C BUS : Stop sequence*/
    I2C2STAT &= 65526;                 /* Reset I2C Receive Overflow and I2C Collision during Write or Master operation  */
    I2C2CONbits.PEN = 1;               /* I2C: STOP */
    MCHP_I2C2_State++;
    break;

   case 3:
    I2C2STAT &= 65526;                 /* Reset I2C Receive Overflow and I2C Collision during Write or Master operation  */
    MCHP_I2C2_State = 0;
    break;

   case 4:                             /* Start a new I2C Sequence */
    I2C2CONbits.SEN = 1;               /* I2C: START  */
    MCHP_I2C2_State++;
    break;

   case 5:
    I2C2TRN = 0x3C;                    /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
    MCHP_I2C2_State++;
    break;

   case 6:
    I2C2TRN = 0x00;                    /* I2C : Write data */
    MCHP_I2C2_State++;
    break;

   case 7:
    I2C2TRN = 0x78;                    /* I2C : Write data */
    MCHP_I2C2_State++;
    break;

   case 8:
    I2C2TRN = 0x00;                    /* I2C : Write data */
    MCHP_I2C2_State++;
    break;

   case 9:
    I2C2TRN = 0x00;                    /* I2C : Write data */
    MCHP_I2C2_State++;
    break;

   case 10:
    I2C2CONbits.PEN = 1;               /* I2C: STOP */
    MCHP_I2C2_State++;
    break;

   case 11:
    MCHP_I2C2_Request = 0;             /* Reset counter for error */
    MCHP_I2C2_State = 0;               /* End of I2C Sequence. I2C is available for a new sequence */
    break;

   default:                            /* Sequence finished */
    MCHP_I2C2_State = 0;               /* Should never happend */
    break;
  }                                    /* End of switch case sequence*/

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}                                      /* Enf of interrupt */
