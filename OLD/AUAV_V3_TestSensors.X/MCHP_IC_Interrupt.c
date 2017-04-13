#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Input Capture - Data-Interrupt: <S656>/Input Capture */
volatile uint16_T MCHP_ic1up;
volatile uint16_T MCHP_ic2up;
volatile uint16_T MCHP_ic3up;
volatile uint16_T MCHP_ic4up;
volatile uint16_T MCHP_ic5up;
volatile uint16_T MCHP_ic6up;
volatile uint16_T MCHP_ic7up;
volatile uint16_T MCHP_ic8up;

/* Implement Input Capture Interrupts if required */
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
  static uint16_T IC1BUF_Old;
  uint16_T IC1BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
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
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC1IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC2Interrupt(void)
{
  static uint16_T IC2BUF_Old;
  uint16_T IC2BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
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
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC2IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC3Interrupt(void)
{
  static uint16_T IC3BUF_Old;
  uint16_T IC3BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
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
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC3IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC4Interrupt(void)
{
  static uint16_T IC4BUF_Old;
  uint16_T IC4BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
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
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC4IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC5Interrupt(void)
{
  static uint16_T IC5BUF_Old;
  uint16_T IC5BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
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
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC5IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC6Interrupt(void)
{
  static uint16_T IC6BUF_Old;
  uint16_T IC6BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
  while (IC6CON1bits.ICBNE == 1)
    IC6BUF_New = IC6BUF;               /* take the last value */
  if (IC6CON1 & 1)                     /* This is a rising edge */
  {
    IC6CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC6CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic6up = IC6BUF_New - IC6BUF_Old;/* Compute Up time */
  }

  IC6BUF_Old = IC6BUF_New;
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC6IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC7Interrupt(void)
{
  static uint16_T IC7BUF_Old;
  uint16_T IC7BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
  while (IC7CON1bits.ICBNE == 1)
    IC7BUF_New = IC7BUF;               /* take the last value */
  if (IC7CON1 & 1)                     /* This is a rising edge */
  {
    IC7CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC7CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic7up = IC7BUF_New - IC7BUF_Old;/* Compute Up time */
  }

  IC7BUF_Old = IC7BUF_New;
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC7IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC8Interrupt(void)
{
  static uint16_T IC8BUF_Old;
  uint16_T IC8BUF_New;
  __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic increment of mcuFlagRecursion */
  T3CONbits.TON = 1;
  while (IC8CON1bits.ICBNE == 1)
    IC8BUF_New = IC8BUF;               /* take the last value */
  if (IC8CON1 & 1)                     /* This is a rising edge */
  {
    IC8CON1 &= 0xFFFE;                 /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC8CON1 |= 1;                      /* detect next rising edge */
    MCHP_ic8up = IC8BUF_New - IC8BUF_Old;/* Compute Up time */
  }

  IC8BUF_Old = IC8BUF_New;
  __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (mcuFlagRecursion) );/* ensure atomic decrement of mcuFlagRecursion */
  if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }

  _IC8IF = 0;
}
