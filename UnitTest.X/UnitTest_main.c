/*--------------------------------------------------------------
 *   MPLAB Blockset v3.35 for Microchip dsPIC chip family.     *
 *   Generate .c and .h files from Simulink model              *
 *   and compile to .elf, .hex and .cof file that can be       *
 *   flashed into the microcontroller                          *
 *                                                             *
 *      The Microchip name PIC, dsPIC, and MPLAB are           *
 *      registered trademarks of Microchip Technology Inc.     *
 *      MATLAB, Simulink, and Real-Time Workshop are           *
 *      registered trademarks of The MathWorks, Inc.           *
 *                                                             *
 *  Blockset authors: L.Kerhuel, U.Kumar                       *
 *  Product Page:  http://www.microchip.com/SimulinkBlocks     *
 *          Forum: http://www.microchip.com/forums/f192.aspx   *
 *          Wiki:  http://microchip.wikidot.com/simulink:start *
 *--------------------------------------------------------------
 *
 * File: UnitTest_main.c
 *
 * Real-Time Workshop code generated for Simulink model UnitTest.
 *
 * Model version                        : 1.210
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Sat Apr 23 21:27:11 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Sat Apr 23 21:27:11 2016
 */

#define MCHP_isMainFile
#include "UnitTest.h"

/* Microchip Global Variables */
volatile uint16_T MCHP_MCULoadResult[1] __attribute__ ((near));
volatile uint16_T MCHP_MCULoadPreviousTimerValue[1] __attribute__ ((near));
volatile uint16_T mcuFlagRecursion __attribute__ ((near)) = 0;
volatile uint16_T BusyFlagRecursion __attribute__ ((near)) = 0;/* Set Fuses Options */
_FGS( GSSK_OFF ) ;
_FOSCSEL( FNOSC_PRI ) ;
_FOSC( POSCMD_XT & OSCIOFNC_ON & FCKSM_CSECME ) ;
_FWDT( FWDTEN_OFF ) ;
_FPOR( BOREN_OFF & ALTI2C2_ON ) ;
_FICD( ICS_PGD3 & JTAGEN_OFF ) ;
_FAS( APLK_OFF ) ;

/* Scheduler */
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void)
{
  asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  T2CONbits.TON = 1;

  {
    extern volatile uint16_T BusyFlagRecursion __attribute__ ((near));
    struct {
      unsigned int Flags0 : 1;
      unsigned int Flags1 : 1;
    } static volatile Overrun;

    struct {
      unsigned int Flags0 : 1;
      unsigned int Flags1 : 1;
    } static volatile event;

    static int_T taskCounter[2] = { 0, 0 };

    _T1IF = 0;                         /* Re-enable interrupt */

    /* Set busy flag */
    asm("INC _BusyFlagRecursion");     /* ensure atomic operation for BusyFlagRecursion++; */
    LATAbits.LATA6 = 1;

    /* Check subrate overrun, set rates that need to run this time step*/
    if (taskCounter[1] == 0) {         /* task dropped on overload */
      event.Flags1 = 1U;
    }

    /* Update task internal counters */
    taskCounter[1]++;
    if (taskCounter[1] == 50) {
      taskCounter[1]= 0;
    }

    /* Step the model for base rate */
    /* Start profiling task 0 */
    {
      uint16_T Tmp_TMR;
      Tmp_TMR = TMR2;
      MCHP_MCULoadResult[0] = Tmp_TMR - MCHP_MCULoadPreviousTimerValue[0];
      MCHP_MCULoadPreviousTimerValue[0] = Tmp_TMR;
    }

    UnitTest_step(0);

    /* Get model outputs here */
    _T1IF = 0;

    /* Stop profiling task 0 */
    if (_T1IF) {
      asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Will re-enter into the interrupt */
    }

    /* Re-Enable Interrupt */
    _IPL0 = 1;                         /* Set IPL to 1 (interrupt priority is 2) */
    _IPL1 = 0;

    /* Step the model for any subrate */
    /* Handle Task 1 */
    if (Overrun.Flags1) {
      asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags1) {
      Overrun.Flags1 = 1;
      do {
        /* Start profiling task 1 */
        event.Flags1 = 0U;
        UnitTest_step(1);

        /* Get model outputs here */
        ;                              /* Execute task tid 1 */

        /* Stop profiling task 1 */
      } while (event.Flags1);

      Overrun.Flags1 = 0U;
    }

    _IPL1 = 1;
    _IPL0 = 0;

    /* Release busy flag */
    asm("DEC _BusyFlagRecursion");     /* ensure atomic operation for BusyFlagRecursion--; */
    if (BusyFlagRecursion == 0)
      LATAbits.LATA6 = 0;
  }

  asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  if (mcuFlagRecursion == 0) {
    T2CONbits.TON = 0;
  }
}

int main()
{
  /* Initialize model */
  /* Start Clock Switching */
  if ((OSCCONbits.COSC & 1) && !(OSCCONbits.COSC & 4))/* check not already in PLL mode (0?1) */
  {
    __builtin_write_OSCCONH(0);        /* Clock Switch to non PLL mode */
    __builtin_write_OSCCONL(0x01);     /* Start clock switching */
    asm("nop");
    asm("nop");
    while (OSCCONbits.COSC != 0) ;
  }

  PLLFBD = 0xF3;                       /* configure Oscillator PLL : PLLDIV */
  CLKDIV = 0x05;                       /* configure Oscillator PLL : FRCDIV ; PLLPOST ; PLLPRE */
  __builtin_write_OSCCONH(3);          /* Clock Switch to desired configuration */
  __builtin_write_OSCCONL(0x01);       /* Start clock switching */

  /* Configure Pins as Analog or Digital */
  ANSELA = 0xFFBF;
  ANSELE = 0xFFE3;

  /* Configure Remappables Pins */
  RPINR20 = 0x53;
  RPOR0 = 0x1D;
  RPOR5 = 0x05;
  RPOR15 = 0x0600;

  /* Configure Digitals I/O directions */
  TRISA = 0xFFBF;
  TRISD = 0xFFFE;
  TRISE = 0xFFEB;
  TRISG = 0x7FFF;

  /* Finish clock switching procedure */
  while (OSCCONbits.COSC != 3) ;       /* Wait for Oscillator Stabilisation */
  while (OSCCONbits.LOCK != 1) ;       /* Wait for PLL Stabilisation */

  /* Initialize model */
  UnitTest_initialize(1);

  /* Configure Timers */
  /* Configure Timers */
  /* Configuration for TIMER 1 */
  _T1IP = 2;
  _T1IF = 0;
  _T1IE = 1;                           /* Enable Timer interrupt. */
  PR1 = 0x2AB8;                        /* Period */

  /* Timer 1 will be enabled after the configuration function. */
  /* Configuration for TIMER 2 */
  PR2 = 0xFFFF;                        /* Period */
  T2CON = 0x20;

  /* Enable Time-step */
  TMR1 = 0x2AB7;
  T1CON = 0x8020;                      /* Timer 1 is the source trigger for the model Time-step */

  /* Main Loop */
  for (;;) ;
}                                      /* end of main() */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
