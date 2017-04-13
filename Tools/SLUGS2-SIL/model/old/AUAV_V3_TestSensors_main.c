/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: AUAV_V3_TestSensors_main.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.262
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Nov 23 10:01:40 2016
 */

#define MCHP_isMainFile
#include "AUAV_V3_TestSensors.h"

/* Microchip Global Variables */
volatile uint16_T MCHP_MCULoadResult[1] ;
volatile uint16_T MCHP_MCULoadPreviousTimerValue[1] ;
volatile uint16_T mcuFlagRecursion  = 0;
union{
  struct {
    unsigned int task0 : 1;
    unsigned int task1 : 1;
    unsigned int task2 : 1;
    unsigned int task3 : 1;
    unsigned int task4 : 1;
    unsigned int task5 : 1;
    unsigned int task6 : 1;
    unsigned int task7 : 1;
    unsigned int task8 : 1;
    unsigned int task9 : 1;
    unsigned int task10 : 1;
    unsigned int task11 : 1;
    unsigned int task12 : 1;
    unsigned int task13 : 1;
    unsigned int task14 : 1;
    unsigned int task15 : 1;
  } b;

  unsigned int val;
} volatile MCHP_MCU_Overload;

volatile uint_T BusyFlagRecursion  = 0;/* Set Fuses Options */
//_FGS( GSSK_OFF ) ;
//_FOSCSEL( FNOSC_PRI ) ;
//_FOSC( POSCMD_XT & OSCIOFNC_ON & FCKSM_CSECME ) ;
//_FWDT( FWDTEN_OFF ) ;
//_FPOR( BOREN_OFF & ALTI2C2_ON ) ;
//_FICD( ICS_PGD3 & JTAGEN_OFF ) ;
//_FAS( APLK_OFF ) ;

/* Scheduler */
void  _T2Interrupt(void)
{
	mcuFlagRecursion++;
  //asm("INC _mcuFlagRecursion");        /* ensure atomic	mcuFlagRecursion++; */
  //T3CONbits.TON = 1;

  {
    extern volatile uint_T BusyFlagRecursion ;
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
      unsigned int Flags12 : 1;
      unsigned int Flags13 : 1;
      unsigned int Flags14 : 1;
      unsigned int Flags15 : 1;
      unsigned int Flags16 : 1;
      unsigned int Flags17 : 1;
      unsigned int Flags18 : 1;
      unsigned int Flags19 : 1;
      unsigned int Flags20 : 1;
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
      unsigned int Flags12 : 1;
      unsigned int Flags13 : 1;
      unsigned int Flags14 : 1;
      unsigned int Flags15 : 1;
      unsigned int Flags16 : 1;
      unsigned int Flags17 : 1;
      unsigned int Flags18 : 1;
      unsigned int Flags19 : 1;
      unsigned int Flags20 : 1;
    } static volatile event;

    static int_T taskCounter[21] = { 0, 0, 0, 9, 5, 4, 2, 19, 0, 49, 40, 30, 20,
      90, 60, 50, 30, 140, 130, 120, 110 };
    //_T2IF = 0;                         /* Re-enable interrupt */

    /* Set busy flag */
	BusyFlagRecursion++;
    //asm("INC _BusyFlagRecursion");     /* ensure atomic operation for BusyFlagRecursion++; */
    //LATAbits.LATA6 = 1;

    /* Check subrate overrun, set rates that need to run this time step*/
    if (taskCounter[1] == 0) {         /* task dropped on overload */
      event.Flags1 = 1U;
      if (Overrun.Flags1) {
        MCHP_MCU_Overload.b.task1 = 1U;/* Set overload bit for tid 1 */
      }
    }

    if (taskCounter[2] == 0) {         /* task dropped on overload */
      event.Flags2 = 1U;
      if (Overrun.Flags2) {
        MCHP_MCU_Overload.b.task2 = 1U;/* Set overload bit for tid 2 */
      }
    }

    if (taskCounter[3] == 0) {         /* task dropped on overload */
      event.Flags3 = 1U;
      if (Overrun.Flags3) {
        MCHP_MCU_Overload.b.task3 = 1U;/* Set overload bit for tid 3 */
      }
    }

    if (taskCounter[4] == 0) {         /* task dropped on overload */
      event.Flags4 = 1U;
      if (Overrun.Flags4) {
        MCHP_MCU_Overload.b.task4 = 1U;/* Set overload bit for tid 4 */
      }
    }

    if (taskCounter[5] == 0) {         /* task dropped on overload */
      event.Flags5 = 1U;
      if (Overrun.Flags5) {
        MCHP_MCU_Overload.b.task5 = 1U;/* Set overload bit for tid 5 */
      }
    }

    if (taskCounter[6] == 0) {         /* task dropped on overload */
      event.Flags6 = 1U;
      if (Overrun.Flags6) {
        MCHP_MCU_Overload.b.task6 = 1U;/* Set overload bit for tid 6 */
      }
    }

    if (taskCounter[7] == 0) {         /* task dropped on overload */
      event.Flags7 = 1U;
      if (Overrun.Flags7) {
        MCHP_MCU_Overload.b.task7 = 1U;/* Set overload bit for tid 7 */
      }
    }

    if (taskCounter[8] == 0) {         /* task dropped on overload */
      event.Flags8 = 1U;
      if (Overrun.Flags8) {
        MCHP_MCU_Overload.b.task8 = 1U;/* Set overload bit for tid 8 */
      }
    }

    if (taskCounter[9] == 0) {         /* task dropped on overload */
      event.Flags9 = 1U;
      if (Overrun.Flags9) {
        MCHP_MCU_Overload.b.task9 = 1U;/* Set overload bit for tid 9 */
      }
    }

    if (taskCounter[10] == 0) {        /* task dropped on overload */
      event.Flags10 = 1U;
      if (Overrun.Flags10) {
        MCHP_MCU_Overload.b.task10 = 1U;/* Set overload bit for tid 10 */
      }
    }

    if (taskCounter[11] == 0) {        /* task dropped on overload */
      event.Flags11 = 1U;
      if (Overrun.Flags11) {
        MCHP_MCU_Overload.b.task11 = 1U;/* Set overload bit for tid 11 */
      }
    }

    if (taskCounter[12] == 0) {        /* task dropped on overload */
      event.Flags12 = 1U;
      if (Overrun.Flags12) {
        MCHP_MCU_Overload.b.task12 = 1U;/* Set overload bit for tid 12 */
      }
    }

    if (taskCounter[13] == 0) {        /* task dropped on overload */
      event.Flags13 = 1U;
      if (Overrun.Flags13) {
        MCHP_MCU_Overload.b.task13 = 1U;/* Set overload bit for tid 13 */
      }
    }

    if (taskCounter[14] == 0) {        /* task dropped on overload */
      event.Flags14 = 1U;
      if (Overrun.Flags14) {
        MCHP_MCU_Overload.b.task14 = 1U;/* Set overload bit for tid 14 */
      }
    }

    if (taskCounter[15] == 0) {        /* task dropped on overload */
      event.Flags15 = 1U;
      if (Overrun.Flags15) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    if (taskCounter[16] == 0) {        /* task dropped on overload */
      event.Flags16 = 1U;
      if (Overrun.Flags16) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    if (taskCounter[17] == 0) {        /* task dropped on overload */
      event.Flags17 = 1U;
      if (Overrun.Flags17) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    if (taskCounter[18] == 0) {        /* task dropped on overload */
      event.Flags18 = 1U;
      if (Overrun.Flags18) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    if (taskCounter[19] == 0) {        /* task dropped on overload */
      event.Flags19 = 1U;
      if (Overrun.Flags19) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    if (taskCounter[20] == 0) {        /* task dropped on overload */
      event.Flags20 = 1U;
      if (Overrun.Flags20) {
        MCHP_MCU_Overload.b.task15 = 1U;/* Set overload bit for tid 15 */
      }
    }

    /* Update task internal counters */
    taskCounter[1]++;
    if (taskCounter[1] == 2) {
      taskCounter[1]= 0;
    }

    taskCounter[2]++;
    if (taskCounter[2] == 10) {
      taskCounter[2]= 0;
    }

    taskCounter[3]++;
    if (taskCounter[3] == 10) {
      taskCounter[3]= 0;
    }

    taskCounter[4]++;
    if (taskCounter[4] == 10) {
      taskCounter[4]= 0;
    }

    taskCounter[5]++;
    if (taskCounter[5] == 10) {
      taskCounter[5]= 0;
    }

    taskCounter[6]++;
    if (taskCounter[6] == 10) {
      taskCounter[6]= 0;
    }

    taskCounter[7]++;
    if (taskCounter[7] == 20) {
      taskCounter[7]= 0;
    }

    taskCounter[8]++;
    if (taskCounter[8] == 50) {
      taskCounter[8]= 0;
    }

    taskCounter[9]++;
    if (taskCounter[9] == 50) {
      taskCounter[9]= 0;
    }

    taskCounter[10]++;
    if (taskCounter[10] == 50) {
      taskCounter[10]= 0;
    }

    taskCounter[11]++;
    if (taskCounter[11] == 50) {
      taskCounter[11]= 0;
    }

    taskCounter[12]++;
    if (taskCounter[12] == 50) {
      taskCounter[12]= 0;
    }

    taskCounter[13]++;
    if (taskCounter[13] == 100) {
      taskCounter[13]= 0;
    }

    taskCounter[14]++;
    if (taskCounter[14] == 100) {
      taskCounter[14]= 0;
    }

    taskCounter[15]++;
    if (taskCounter[15] == 100) {
      taskCounter[15]= 0;
    }

    taskCounter[16]++;
    if (taskCounter[16] == 100) {
      taskCounter[16]= 0;
    }

    taskCounter[17]++;
    if (taskCounter[17] == 200) {
      taskCounter[17]= 0;
    }

    taskCounter[18]++;
    if (taskCounter[18] == 200) {
      taskCounter[18]= 0;
    }

    taskCounter[19]++;
    if (taskCounter[19] == 200) {
      taskCounter[19]= 0;
    }

    taskCounter[20]++;
    if (taskCounter[20] == 200) {
      taskCounter[20]= 0;
    }

    /* Step the model for base rate */
    /* Start profiling task 0 */
    {
      uint16_T Tmp_TMR;
	  Tmp_TMR = AUAV_V3_TestSensors_DWork.time_since_boot_usec;// TMR3;
      MCHP_MCULoadResult[0] = Tmp_TMR - MCHP_MCULoadPreviousTimerValue[0];
      MCHP_MCULoadPreviousTimerValue[0] = Tmp_TMR;
    }

    AUAV_V3_TestSensors_step0();

    /* Get model outputs here */
    //_T2IF = 0;

    /* Stop profiling task 0 */
    //if (_T2IF) {
    //  asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
    //  asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
    //  MCHP_MCU_Overload.b.task0 = 1U;  /* Set overload bit for tid 0 */
    //  return;                          /* Will re-enter into the interrupt */
    //}

    /* Re-Enable Interrupt */
    //_IPL0 = 1;                         /* Set IPL to 1 (interrupt priority is 2) */
    //_IPL1 = 0;

    /* Step the model for any subrate */
    /* Handle Task 1 */
    if (Overrun.Flags1) {
		BusyFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
		mcuFlagRecursion--;
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags1) {
      Overrun.Flags1 = 1;
      do {
        /* Start profiling task 1 */
        event.Flags1 = 0U;
        AUAV_V3_TestSensors_step1();

        /* Get model outputs here */
        ;                              /* Execute task tid 1 */

        /* Stop profiling task 1 */
      } while (event.Flags1);

      Overrun.Flags1 = 0U;
    }

    /* Handle Task 2 */
    if (Overrun.Flags2) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;

      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags2) {
      Overrun.Flags2 = 1;
      do {
        /* Start profiling task 2 */
        event.Flags2 = 0U;
        AUAV_V3_TestSensors_step2();

        /* Get model outputs here */
        ;                              /* Execute task tid 2 */

        /* Stop profiling task 2 */
      } while (event.Flags2);

      Overrun.Flags2 = 0U;
    }

    /* Handle Task 3 */
    if (Overrun.Flags3) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags3) {
      Overrun.Flags3 = 1;
      do {
        /* Start profiling task 3 */
        event.Flags3 = 0U;
        AUAV_V3_TestSensors_step3();

        /* Get model outputs here */
        ;                              /* Execute task tid 3 */

        /* Stop profiling task 3 */
      } while (event.Flags3);

      Overrun.Flags3 = 0U;
    }

    /* Handle Task 4 */
    if (Overrun.Flags4) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags4) {
      Overrun.Flags4 = 1;
      do {
        /* Start profiling task 4 */
        event.Flags4 = 0U;
        AUAV_V3_TestSensors_step4();

        /* Get model outputs here */
        ;                              /* Execute task tid 4 */

        /* Stop profiling task 4 */
      } while (event.Flags4);

      Overrun.Flags4 = 0U;
    }

    /* Handle Task 5 */
    if (Overrun.Flags5) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags5) {
      Overrun.Flags5 = 1;
      do {
        /* Start profiling task 5 */
        event.Flags5 = 0U;
        AUAV_V3_TestSensors_step5();

        /* Get model outputs here */
        ;                              /* Execute task tid 5 */

        /* Stop profiling task 5 */
      } while (event.Flags5);

      Overrun.Flags5 = 0U;
    }

    /* Handle Task 6 */
    if (Overrun.Flags6) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags6) {
      Overrun.Flags6 = 1;
      do {
        /* Start profiling task 6 */
        event.Flags6 = 0U;
        AUAV_V3_TestSensors_step6();

        /* Get model outputs here */
        ;                              /* Execute task tid 6 */

        /* Stop profiling task 6 */
      } while (event.Flags6);

      Overrun.Flags6 = 0U;
    }

    /* Handle Task 7 */
    if (Overrun.Flags7) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags7) {
      Overrun.Flags7 = 1;
      do {
        /* Start profiling task 7 */
        event.Flags7 = 0U;
        AUAV_V3_TestSensors_step7();

        /* Get model outputs here */
        ;                              /* Execute task tid 7 */

        /* Stop profiling task 7 */
      } while (event.Flags7);

      Overrun.Flags7 = 0U;
    }

    /* Handle Task 8 */
    if (Overrun.Flags8) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags8) {
      Overrun.Flags8 = 1;
      do {
        /* Start profiling task 8 */
        event.Flags8 = 0U;
        AUAV_V3_TestSensors_step8();

        /* Get model outputs here */
        ;                              /* Execute task tid 8 */

        /* Stop profiling task 8 */
      } while (event.Flags8);

      Overrun.Flags8 = 0U;
    }

    /* Handle Task 9 */
    if (Overrun.Flags9) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags9) {
      Overrun.Flags9 = 1;
      do {
        /* Start profiling task 9 */
        event.Flags9 = 0U;
        AUAV_V3_TestSensors_step9();

        /* Get model outputs here */
        ;                              /* Execute task tid 9 */

        /* Stop profiling task 9 */
      } while (event.Flags9);

      Overrun.Flags9 = 0U;
    }

    /* Handle Task 10 */
    if (Overrun.Flags10) {
		BusyFlagRecursion--;
		mcuFlagRecursion--;
      //asm("DEC _BusyFlagRecursion");   /* ensure atomic operation for BusyFlagRecursion--; */
      //asm("DEC _mcuFlagRecursion");    /* ensure atomic	mcuFlagRecursion--; The value cannot reach 0 here, do not stop the timer */
      return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags10) {
      Overrun.Flags10 = 1;
      do {
        /* Start profiling task 10 */
        event.Flags10 = 0U;
        AUAV_V3_TestSensors_step10();

        /* Get model outputs here */
        ;                              /* Execute task tid 10 */

        /* Stop profiling task 10 */
      } while (event.Flags10);

      Overrun.Flags10 = 0U;
    }

    /* Handle Task 11 */
    if (Overrun.Flags11) {
     BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags11) {
      Overrun.Flags11 = 1;
      do {
        /* Start profiling task 11 */
        event.Flags11 = 0U;
        AUAV_V3_TestSensors_step11();

        /* Get model outputs here */
        ;                              /* Execute task tid 11 */

        /* Stop profiling task 11 */
      } while (event.Flags11);

      Overrun.Flags11 = 0U;
    }

    /* Handle Task 12 */
    if (Overrun.Flags12) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags12) {
      Overrun.Flags12 = 1;
      do {
        /* Start profiling task 12 */
        event.Flags12 = 0U;
        AUAV_V3_TestSensors_step12();

        /* Get model outputs here */
        ;                              /* Execute task tid 12 */

        /* Stop profiling task 12 */
      } while (event.Flags12);

      Overrun.Flags12 = 0U;
    }

    /* Handle Task 13 */
    if (Overrun.Flags13) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags13) {
      Overrun.Flags13 = 1;
      do {
        /* Start profiling task 13 */
        event.Flags13 = 0U;
        AUAV_V3_TestSensors_step13();

        /* Get model outputs here */
        ;                              /* Execute task tid 13 */

        /* Stop profiling task 13 */
      } while (event.Flags13);

      Overrun.Flags13 = 0U;
    }

    /* Handle Task 14 */
    if (Overrun.Flags14) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		 return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags14) {
      Overrun.Flags14 = 1;
      do {
        /* Start profiling task 14 */
        event.Flags14 = 0U;
        AUAV_V3_TestSensors_step14();

        /* Get model outputs here */
        ;                              /* Execute task tid 14 */

        /* Stop profiling task 14 */
      } while (event.Flags14);

      Overrun.Flags14 = 0U;
    }

    /* Handle Task 15 */
    if (Overrun.Flags15) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		 return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags15) {
      Overrun.Flags15 = 1;
      do {
        /* Start profiling task 15 */
        event.Flags15 = 0U;
        AUAV_V3_TestSensors_step15();

        /* Get model outputs here */
        ;                              /* Execute task tid 15 */

        /* Stop profiling task 15 */
      } while (event.Flags15);

      Overrun.Flags15 = 0U;
    }

    /* Handle Task 16 */
    if (Overrun.Flags16) {
     BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags16) {
      Overrun.Flags16 = 1;
      do {
        /* Start profiling task 16 */
        event.Flags16 = 0U;
        AUAV_V3_TestSensors_step16();

        /* Get model outputs here */
        ;                              /* Execute task tid 16 */

        /* Stop profiling task 16 */
      } while (event.Flags16);

      Overrun.Flags16 = 0U;
    }

    /* Handle Task 17 */
    if (Overrun.Flags17) {
     BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags17) {
      Overrun.Flags17 = 1;
      do {
        /* Start profiling task 17 */
        event.Flags17 = 0U;
        AUAV_V3_TestSensors_step17();

        /* Get model outputs here */
        ;                              /* Execute task tid 17 */

        /* Stop profiling task 17 */
      } while (event.Flags17);

      Overrun.Flags17 = 0U;
    }

    /* Handle Task 18 */
    if (Overrun.Flags18) {
     BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags18) {
      Overrun.Flags18 = 1;
      do {
        /* Start profiling task 18 */
        event.Flags18 = 0U;
        AUAV_V3_TestSensors_step18();

        /* Get model outputs here */
        ;                              /* Execute task tid 18 */

        /* Stop profiling task 18 */
      } while (event.Flags18);

      Overrun.Flags18 = 0U;
    }

    /* Handle Task 19 */
    if (Overrun.Flags19) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		 return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags19) {
      Overrun.Flags19 = 1;
      do {
        /* Start profiling task 19 */
        event.Flags19 = 0U;
        AUAV_V3_TestSensors_step19();

        /* Get model outputs here */
        ;                              /* Execute task tid 19 */

        /* Stop profiling task 19 */
      } while (event.Flags19);

      Overrun.Flags19 = 0U;
    }

    /* Handle Task 20 */
    if (Overrun.Flags20) {
      BusyFlagRecursion--;
		mcuFlagRecursion--;
		return;                          /* Priority to higher rate steps interrupted */
    }

    if (event.Flags20) {
      Overrun.Flags20 = 1;
      do {
        /* Start profiling task 20 */
        event.Flags20 = 0U;
        AUAV_V3_TestSensors_step20();

        /* Get model outputs here */
        ;                              /* Execute task tid 20 */

        /* Stop profiling task 20 */
      } while (event.Flags20);

      Overrun.Flags20 = 0U;
    }

    /* Disable Interrupt. IPL value is 1 at this point */
    //_IPL1 = 1;                         /* Set IPL to 2 (interrupt priority was 1) */
    //_IPL0 = 0;

    /* Release busy flag */
    //asm("DEC _BusyFlagRecursion");     /* ensure atomic operation for BusyFlagRecursion--; */
   /* if (BusyFlagRecursion == 0)
      LATAbits.LATA6 = 0;*/
  }

  //asm("DEC _mcuFlagRecursion");        /* ensure --mcuFlagRecursion is atomic */
  /*if (mcuFlagRecursion == 0) {
    T3CONbits.TON = 0;
  }*/
}

//int main()
//{
//  /* Initialize model */
//  /* Start Clock Switching */
//  if ((OSCCONbits.COSC & 1) && !(OSCCONbits.COSC & 4))/* check not already in PLL mode (0?1) */
//  {
//    __builtin_write_OSCCONH(0);        /* Clock Switch to non PLL mode */
//    __builtin_write_OSCCONL(0x01);     /* Start clock switching */
//    asm("nop");
//    asm("nop");
//    while (OSCCONbits.COSC != 0) ;
//  }
//
//  PLLFBD = 0xF3;                       /* configure Oscillator PLL : PLLDIV */
//  CLKDIV = 0x05;                       /* configure Oscillator PLL : FRCDIV ; PLLPOST ; PLLPRE */
//  __builtin_write_OSCCONH(3);          /* Clock Switch to desired configuration */
//  __builtin_write_OSCCONL(0x01);       /* Start clock switching */
//
//  /* Configure Pins as Analog or Digital */
//  ANSELA = 0xFFBF;
//  ANSELB = 0xFFC3;
//  ANSELD = 0xFF7F;
//  ANSELE = 0xFF82;
//
//  /* Configure Remappables Pins */
//  RPINR7 = 0x4840;
//  RPINR8 = 0x1F4B;
//  RPINR9 = 0x1E15;
//  RPINR10 = 0x1014;
//  RPINR20 = 0x53;
//  RPOR3 = 0x1300;
//  RPOR4 = 0x1100;
//  RPOR5 = 0x05;
//  RPOR6 = 0x1D;
//  RPOR8 = 0x0100;
//  RPOR12 = 0x1000;
//  RPOR14 = 0x1200;
//  RPOR15 = 0x0600;
//
//  /* Configure Digitals I/O directions */
//  TRISA = 0xFFBF;
//  TRISB = 0xFFC3;
//  TRISD = 0xFF7F;
//  TRISE = 0xFFCA;
//  TRISF = 0xFFF7;
//  TRISG = 0x5FFE;
//
//  /* Finish clock switching procedure */
//  while (OSCCONbits.COSC != 3) ;       /* Wait for Oscillator Stabilisation */
//  while (OSCCONbits.LOCK != 1) ;       /* Wait for PLL Stabilisation */
//
//  /* Initialize model */
//  AUAV_V3_TestSensors_initialize();
//
//  /* Configure Timers */
//  /* Configure Timers */
//  /* Configuration for TIMER 1 */
//  PR1 = 0xFFFF;                        /* Period */
//  T1CON = 0x8010;
//
//  /* Configuration for TIMER 2 */
//  _T2IP = 2;
//  _T2IF = 0;
//  _T2IE = 1;                           /* Enable Timer interrupt. */
//  PR2 = 0x2AB8;                        /* Period */
//
//  /* Timer 2 will be enabled after the configuration function. */
//  /* Configuration for TIMER 3 */
//  PR3 = 0xFFFF;                        /* Period */
//  T3CON = 0x20;
//
//  /* Enable Time-step */
//  TMR2 = 0x2AB7;
//  T2CON = 0x8020;                      /* Timer 2 is the source trigger for the model Time-step */
//
//  /* Main Loop */
//  for (;;) ;
//}                                      /* end of main() */
/*
 * File trailer for generated code.
 *
 * [EOF]
 */
