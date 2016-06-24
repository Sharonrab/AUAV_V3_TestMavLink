/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_main.c
 *
 * Code generated for Simulink model 'AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER'.
 *
 * Model version                  : 1.289
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Fri Jun 24 00:20:50 2016
 */

#define MCHP_isMainFile
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"

/* Microchip Global Variables */
volatile uint_T BusyFlagRecursion __attribute__ ((near)) = 0;/* Set Fuses Options */
_FGS( GSSK_OFF ) ;
_FOSCSEL( FNOSC_PRI ) ;
_FOSC( POSCMD_XT & OSCIOFNC_ON & FCKSM_CSECME ) ;
_FWDT( FWDTEN_OFF ) ;
_FPOR( BOREN_OFF & ALTI2C2_ON ) ;
_FICD( ICS_PGD3 & JTAGEN_OFF ) ;
_FAS( APLK_OFF ) ;

/* Scheduler */
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void)
{
  {
    extern volatile uint_T BusyFlagRecursion __attribute__ ((near)) ;
    struct {
      unsigned int Flags0 : 1;
      unsigned int Flags1 : 1;
      unsigned int Flags2 : 1;
      unsigned int Flags3 : 1;
      unsigned int Flags4 : 1;
      unsigned int Flags5 : 1;
      unsigned int Flags6 : 1;
      unsigned int Flags7 : 1;
      unsigned int Flags8 : 1;
      unsigned int Flags9 : 1;
      unsigned int Flags10 : 1;
      unsigned int Flags11 : 1;
    } static volatile Overrun;

    struct {
      unsigned int Flags0 : 1;
      unsigned int Flags1 : 1;
      unsigned int Flags2 : 1;
      unsigned int Flags3 : 1;
      unsigned int Flags4 : 1;
      unsigned int Flags5 : 1;
      unsigned int Flags6 : 1;
      unsigned int Flags7 : 1;
      unsigned int Flags8 : 1;
      unsigned int Flags9 : 1;
      unsigned int Flags10 : 1;
      unsigned int Flags11 : 1;
    } static volatile event;

    static int_T taskCounter[12] = { 0, 0, 0, 0, 0, 18, 16, 14, 12, 0, 0, 0 };

    _T2IF = 0;                         /* Re-enable interrupt */

    /* Set busy flag */
    __asm__ volatile ("INC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic increment of BusyFlagRecursion */
    LATAbits.LATA6 = 1;

    /* Check subrate overrun, set rates that need to run this time step*/
    if (taskCounter[1] == 0) {         /* task dropped on overload */
      event.Flags1 = 1U;
    }

    if (taskCounter[2] == 0) {         /* task dropped on overload */
      event.Flags2 = 1U;
    }

    if (taskCounter[3] == 0) {         /* task dropped on overload */
      event.Flags3 = 1U;
    }

    if (taskCounter[4] == 0) {         /* task dropped on overload */
      event.Flags4 = 1U;
    }

    if (taskCounter[5] == 0) {         /* task dropped on overload */
      event.Flags5 = 1U;
    }

    if (taskCounter[6] == 0) {         /* task dropped on overload */
      event.Flags6 = 1U;
    }

    if (taskCounter[7] == 0) {         /* task dropped on overload */
      event.Flags7 = 1U;
    }

    if (taskCounter[8] == 0) {         /* task dropped on overload */
      event.Flags8 = 1U;
    }

    if (taskCounter[9] == 0) {         /* task dropped on overload */
      event.Flags9 = 1U;
    }

    if (taskCounter[10] == 0) {        /* task dropped on overload */
      event.Flags10 = 1U;
    }

    if (taskCounter[11] == 0) {        /* task dropped on overload */
      event.Flags11 = 1U;
    }

    /* Update task internal counters */
    taskCounter[1]++;
    if (taskCounter[1] == 2) {
      taskCounter[1]= 0;
    }

    taskCounter[2]++;
    if (taskCounter[2] == 5) {
      taskCounter[2]= 0;
    }

    taskCounter[3]++;
    if (taskCounter[3] == 10) {
      taskCounter[3]= 0;
    }

    taskCounter[4]++;
    if (taskCounter[4] == 20) {
      taskCounter[4]= 0;
    }

    taskCounter[5]++;
    if (taskCounter[5] == 20) {
      taskCounter[5]= 0;
    }

    taskCounter[6]++;
    if (taskCounter[6] == 20) {
      taskCounter[6]= 0;
    }

    taskCounter[7]++;
    if (taskCounter[7] == 20) {
      taskCounter[7]= 0;
    }

    taskCounter[8]++;
    if (taskCounter[8] == 20) {
      taskCounter[8]= 0;
    }

    taskCounter[9]++;
    if (taskCounter[9] == 25) {
      taskCounter[9]= 0;
    }

    taskCounter[10]++;
    if (taskCounter[10] == 50) {
      taskCounter[10]= 0;
    }

    taskCounter[11]++;
    if (taskCounter[11] == 100) {
      taskCounter[11]= 0;
    }

    /* Step the model for base rate */
    /* Start profiling task 0 */
    AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step0();

    /* Get model outputs here */
    _T2IF = 0;                         /* Re-enable interrupt */

    /* Stop profiling task 0 */
    if (_T2IF ) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Will re-enter into the interrupt */
    }

    /* Re-Enable Interrupt. IPL value is 2 at this point */
    _IPL0 = 1;                         /* Set IPL to 1 (interrupt priority is 2) */
    _IPL1 = 0;

    /* Step the model for any subrate */
    /* Handle Task 1 */
    if (Overrun.Flags1) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags1) {
      Overrun.Flags1 = 1;
      do {
        /* Start profiling task 1 */
        event.Flags1 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step1();

        /* Get model outputs here */
        ;                              /* Execute task tid 1 */

        /* Stop profiling task 1 */
      } while (event.Flags1);

      Overrun.Flags1 = 0U;
    }

    /* Handle Task 2 */
    if (Overrun.Flags2) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags2) {
      Overrun.Flags2 = 1;
      do {
        /* Start profiling task 2 */
        event.Flags2 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step2();

        /* Get model outputs here */
        ;                              /* Execute task tid 2 */

        /* Stop profiling task 2 */
      } while (event.Flags2);

      Overrun.Flags2 = 0U;
    }

    /* Handle Task 3 */
    if (Overrun.Flags3) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags3) {
      Overrun.Flags3 = 1;
      do {
        /* Start profiling task 3 */
        event.Flags3 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step3();

        /* Get model outputs here */
        ;                              /* Execute task tid 3 */

        /* Stop profiling task 3 */
      } while (event.Flags3);

      Overrun.Flags3 = 0U;
    }

    /* Handle Task 4 */
    if (Overrun.Flags4) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags4) {
      Overrun.Flags4 = 1;
      do {
        /* Start profiling task 4 */
        event.Flags4 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step4();

        /* Get model outputs here */
        ;                              /* Execute task tid 4 */

        /* Stop profiling task 4 */
      } while (event.Flags4);

      Overrun.Flags4 = 0U;
    }

    /* Handle Task 5 */
    if (Overrun.Flags5) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags5) {
      Overrun.Flags5 = 1;
      do {
        /* Start profiling task 5 */
        event.Flags5 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step5();

        /* Get model outputs here */
        ;                              /* Execute task tid 5 */

        /* Stop profiling task 5 */
      } while (event.Flags5);

      Overrun.Flags5 = 0U;
    }

    /* Handle Task 6 */
    if (Overrun.Flags6) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags6) {
      Overrun.Flags6 = 1;
      do {
        /* Start profiling task 6 */
        event.Flags6 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step6();

        /* Get model outputs here */
        ;                              /* Execute task tid 6 */

        /* Stop profiling task 6 */
      } while (event.Flags6);

      Overrun.Flags6 = 0U;
    }

    /* Handle Task 7 */
    if (Overrun.Flags7) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags7) {
      Overrun.Flags7 = 1;
      do {
        /* Start profiling task 7 */
        event.Flags7 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step7();

        /* Get model outputs here */
        ;                              /* Execute task tid 7 */

        /* Stop profiling task 7 */
      } while (event.Flags7);

      Overrun.Flags7 = 0U;
    }

    /* Handle Task 8 */
    if (Overrun.Flags8) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags8) {
      Overrun.Flags8 = 1;
      do {
        /* Start profiling task 8 */
        event.Flags8 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step8();

        /* Get model outputs here */
        ;                              /* Execute task tid 8 */

        /* Stop profiling task 8 */
      } while (event.Flags8);

      Overrun.Flags8 = 0U;
    }

    /* Handle Task 9 */
    if (Overrun.Flags9) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags9) {
      Overrun.Flags9 = 1;
      do {
        /* Start profiling task 9 */
        event.Flags9 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step9();

        /* Get model outputs here */
        ;                              /* Execute task tid 9 */

        /* Stop profiling task 9 */
      } while (event.Flags9);

      Overrun.Flags9 = 0U;
    }

    /* Handle Task 10 */
    if (Overrun.Flags10) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags10) {
      Overrun.Flags10 = 1;
      do {
        /* Start profiling task 10 */
        event.Flags10 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step10();

        /* Get model outputs here */
        ;                              /* Execute task tid 10 */

        /* Stop profiling task 10 */
      } while (event.Flags10);

      Overrun.Flags10 = 0U;
    }

    /* Handle Task 11 */
    if (Overrun.Flags11) {
      __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags11) {
      Overrun.Flags11 = 1;
      do {
        /* Start profiling task 11 */
        event.Flags11 = 0U;
        AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_step11();

        /* Get model outputs here */
        ;                              /* Execute task tid 11 */

        /* Stop profiling task 11 */
      } while (event.Flags11);

      Overrun.Flags11 = 0U;
    }

    /* Disable Interrupt. IPL value is 1 at this point */
    _IPL1 = 1;                         /* Set IPL to 2 (interrupt priority was 1) */
    _IPL0 = 0;

    /* Release busy flag */
    __asm__ volatile ("DEC %[MyVar]" : [MyVar] "+m" (BusyFlagRecursion) );/* ensure atomic decrement of BusyFlagRecursion */
    if (BusyFlagRecursion == 0)
      LATAbits.LATA6 = 0;
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
    __asm__ volatile("nop\n"
                     "nop");
    while (OSCCONbits.COSC != 0) ;
  }

  PLLFBD = 0xF3;                       /* configure Oscillator PLL : PLLDIV */
  CLKDIV = 0x05;                       /* configure Oscillator PLL : FRCDIV ; PLLPOST ; PLLPRE */
  __builtin_write_OSCCONH(3);          /* Clock Switch to desired configuration */
  __builtin_write_OSCCONL(0x01);       /* Start clock switching */

  /* Configure Pins as Analog or Digital */
  ANSELA = 0x0680;
  ANSELB = 0xFF83;
  ANSELC = 0x6016;
  ANSELE = 0x01C2;
  ANSELG = 0xC0;

  /* Configure Remappables Pins */
  RPINR7 = 0x4840;
  RPINR8 = 0x1F4B;
  RPINR9 = 0x4D10;
  RPINR10 = 0x33;
  RPINR20 = 0x53;
  RPOR4 = 0x1100;
  RPOR5 = 0x05;
  RPOR6 = 0x1D;
  RPOR8 = 0x0100;
  RPOR12 = 0x1000;
  RPOR15 = 0x0600;

  /* Configure Digitals I/O directions */
  TRISA = 0xFFBF;
  TRISB = 0xFF83;
  TRISE = 0xFDCA;
  TRISF = 0xFFF7;
  TRISG = 0x7CFE;

  /* Finish clock switching procedure */
  while (OSCCONbits.COSC != 3) ;       /* Wait for Oscillator Stabilisation */
  while (OSCCONbits.LOCK != 1) ;       /* Wait for PLL Stabilisation */

  /* Initialize model */
  AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_initialize();

  /* Configure Timers */
  /* --- TIMER 1 --- */
  PR1 = 0xFFFF;                        /* Period */
  T1CON = 0x8010;

  /* --- TIMER 2 --- This timer is enabled at end of configuration functions. */
  _T2IP = 2;                           /* Set timer Interrupt Priority */
  _T2IF = 0;                           /* Reset pending Interrupt */
  _T2IE = 1;                           /* Enable Timer Interrupt. */
  PR2 = 0x2AB8;                        /* Period */

  /* Enable Time-step */
  PTCON = 0x8000;                      /* Enable PWM peripheral */
  TMR2 = 0x2AB7;
  T2CON = 0x8020;                      /* Timer 2 is the source trigger for the model Time-step */

  /* Main Loop */
  for (;;) ;
}                                      /* end of main() */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
