#include "AUAV3_AND_SLUGS_SENSOR.h"
#include "AUAV3_AND_SLUGS_SENSOR_private.h"

volatile uint16_T MCHP_ic1up;
volatile uint16_T MCHP_ic2up;
volatile uint16_T MCHP_ic3up;
volatile uint16_T MCHP_ic4up;
volatile uint16_T MCHP_ic5up;

/* Implement Input Capture Interrupts if required */
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
  static uint16_T IC1BUF_Old;
  uint16_T IC1BUF_New;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while (IC1CON1bits.ICBNE == 1)
    IC1BUF_New = IC1BUF;               /* take the last value */
  if (IC1CON1 & 1)                     /* This is a rising edge */
  {
    IC1CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC1CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic1up = IC1BUF_New - IC1BUF_Old;/* Compute Up time */
  }

  IC1BUF_Old = IC1BUF_New;
  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }

  _IC1IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC2Interrupt(void)
{
  static uint16_T IC2BUF_Old;
  uint16_T IC2BUF_New;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while (IC2CON1bits.ICBNE == 1)
    IC2BUF_New = IC2BUF;               /* take the last value */
  if (IC2CON1 & 1)                     /* This is a rising edge */
  {
    IC2CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC2CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic2up = IC2BUF_New - IC2BUF_Old;/* Compute Up time */
  }

  IC2BUF_Old = IC2BUF_New;
  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }

  _IC2IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC3Interrupt(void)
{
  static uint16_T IC3BUF_Old;
  uint16_T IC3BUF_New;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while (IC3CON1bits.ICBNE == 1)
    IC3BUF_New = IC3BUF;               /* take the last value */
  if (IC3CON1 & 1)                     /* This is a rising edge */
  {
    IC3CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC3CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic3up = IC3BUF_New - IC3BUF_Old;/* Compute Up time */
  }

  IC3BUF_Old = IC3BUF_New;
  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }

  _IC3IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC4Interrupt(void)
{
  static uint16_T IC4BUF_Old;
  uint16_T IC4BUF_New;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while (IC4CON1bits.ICBNE == 1)
    IC4BUF_New = IC4BUF;               /* take the last value */
  if (IC4CON1 & 1)                     /* This is a rising edge */
  {
    IC4CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC4CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic4up = IC4BUF_New - IC4BUF_Old;/* Compute Up time */
  }

  IC4BUF_Old = IC4BUF_New;
  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }

  _IC4IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC5Interrupt(void)
{
  static uint16_T IC5BUF_Old;
  uint16_T IC5BUF_New;
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;
  while (IC5CON1bits.ICBNE == 1)
    IC5BUF_New = IC5BUF;               /* take the last value */
  if (IC5CON1 & 1)                     /* This is a rising edge */
  {
    IC5CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC5CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic5up = IC5BUF_New - IC5BUF_Old;/* Compute Up time */
  }

  IC5BUF_Old = IC5BUF_New;
  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }

  _IC5IF = 0;
}
