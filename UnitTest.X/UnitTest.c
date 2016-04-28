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
 * File: UnitTest.c
 *
 * Real-Time Workshop code generated for Simulink model UnitTest.
 *
 * Model version                        : 1.210
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Sat Apr 23 21:27:11 2016
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Sat Apr 23 21:27:11 2016
 */

#include "UnitTest.h"
#include "UnitTest_private.h"

const mavlink_scaled_pressure_t UnitTest_rtZmavlink_scaled_pressure_t = {
  0U,                                  /* time_boot_ms */
  0.0F,                                /* press_abs */
  0.0F,                                /* press_diff */
  0                                    /* temperature */
} ;                                    /* mavlink_scaled_pressure_t ground */

const mavlink_gps_raw_int_t UnitTest_rtZmavlink_gps_raw_int_t = {
  0U,                                  /* time_usec */
  0,                                   /* lat */
  0,                                   /* lon */
  0,                                   /* alt */
  0U,                                  /* eph */
  0U,                                  /* epv */
  0U,                                  /* vel */
  0U,                                  /* cog */
  0U,                                  /* fix_type */
  0U                                   /* satellites_visible */
} ;                                    /* mavlink_gps_raw_int_t ground */

const mavlink_raw_pressure_t UnitTest_rtZmavlink_raw_pressure_t = {
  0U,                                  /* time_boot_ms */
  0,                                   /* press_abs */
  0,                                   /* press_diff1 */
  0,                                   /* press_diff2 */
  0                                    /* temperature */
} ;                                    /* mavlink_raw_pressure_t ground */

const mavlink_sys_status_t UnitTest_rtZmavlink_sys_status_t = {
  0U,                                  /* onboard_control_sensors_present */
  0U,                                  /* onboard_control_sensors_enabled */
  0U,                                  /* onboard_control_sensors_health */
  0U,                                  /* load */
  0U,                                  /* voltage_battery */
  0,                                   /* current_battery */
  0U,                                  /* drop_rate_comm */
  0U,                                  /* errors_comm */
  0U,                                  /* errors_count1 */
  0U,                                  /* errors_count2 */
  0U,                                  /* errors_count3 */
  0U,                                  /* errors_count4 */
  0U                                   /* battery_remaining */
} ;                                    /* mavlink_sys_status_t ground */

/* Exported block states */
mavlink_gps_raw_int_t mlGpsData;       /* '<Root>/mlGpsData' */
mavlink_sys_status_t mlSysStatus;      /* '<Root>/mlSysStatus' */
mavlink_scaled_pressure_t mlAirData;   /* '<Root>/mlAirData' */
mavlink_raw_pressure_t mlRawPressureData;/* '<Root>/mlRawPressureData' */
uint16_T MPU_T;                        /* '<Root>/MPU_T' */

/* Block signals (auto storage) */
BlockIO_UnitTest_T UnitTest_B;

/* Block states (auto storage) */
D_Work_UnitTest_T UnitTest_DWork;

/* Real-time model */
RT_MODEL_UnitTest_T UnitTest_M_;
RT_MODEL_UnitTest_T *const UnitTest_M = &UnitTest_M_;
uint32_T MultiWord2uLong(const uint32_T u[])
{
  return u[0];
}

void uMultiWordShr(const uint32_T u1[], int16_T n1, uint16_T n2, uint32_T y[],
                   int16_T n)
{
  int16_T nb;
  int16_T i;
  uint32_T yi;
  uint32_T u1i;
  int16_T nc;
  uint16_T nr;
  uint16_T nl;
  int16_T i1;
  int16_T loop_ub;
  int16_T loop_ub_0;
  nb = (int16_T)(n2 >> 5);
  i = 0;
  if (nb < n1) {
    nc = n + nb;
    if (nc > n1) {
      nc = n1;
    }

    nr = n2 - ((uint16_T)nb << 5);
    if (nr > 0U) {
      nl = 32U - nr;
      u1i = u1[nb];
      loop_ub = nc - 1;
      for (i1 = nb + 1; i1 <= loop_ub; i1++) {
        yi = u1i >> nr;
        u1i = u1[i1];
        y[i] = u1i << nl | yi;
        i++;
      }

      yi = u1i >> nr;
      if (nc < n1) {
        yi |= u1[nc] << nl;
      }

      y[i] = yi;
      i++;
    } else {
      loop_ub_0 = nc - 1;
      for (i1 = nb; i1 <= loop_ub_0; i1++) {
        y[i] = u1[i1];
        i++;
      }
    }
  }

  while (i < n) {
    y[i] = 0UL;
    i++;
  }
}

void uMultiWordMul(const uint32_T u1[], int16_T n1, const uint32_T u2[], int16_T
                   n2, uint32_T y[], int16_T n)
{
  int16_T i;
  int16_T j;
  int16_T k;
  int16_T nj;
  uint32_T u1i;
  uint32_T yk;
  uint32_T a;
  uint32_T a_0;
  uint32_T b;
  uint32_T w;
  uint32_T w_0;
  uint32_T cb;
  int16_T loop_ub;
  int16_T loop_ub_0;
  int16_T loop_ub_1;

  /* Initialize output to zero */
  loop_ub = n - 1;
  for (k = 0; k <= loop_ub; k++) {
    y[k] = 0UL;
  }

  loop_ub_1 = n1 - 1;
  for (i = 0; i <= loop_ub_1; i++) {
    cb = 0UL;
    u1i = u1[i];
    a = u1i >> 16U;
    a_0 = u1i & 65535UL;
    k = n - i;
    nj = n2 <= k ? n2 : k;
    k = i;
    loop_ub_0 = nj - 1;
    for (j = 0; j <= loop_ub_0; j++) {
      yk = y[k];
      u1i = u2[j];
      b = u1i >> 16U;
      u1i &= 65535UL;
      w = a * u1i;
      w_0 = a_0 * b;
      yk += cb;
      cb = yk < cb ? 1UL : 0UL;
      u1i *= a_0;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      u1i = w << 16U;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      u1i = w_0 << 16U;
      yk += u1i;
      cb += yk < u1i ? 1UL : 0UL;
      y[k] = yk;
      cb += w >> 16U;
      cb += w_0 >> 16U;
      cb += a * b;
      k++;
    }

    if (k < n) {
      y[k] = cb;
    }
  }
}

/* Model step function for TID0 */
void UnitTest_step0(void)              /* Sample time: [0.01s, 0.0s] */
{
  uint32_T rtb_FixPtSum1;
  uint64m_T tmp;
  uint64m_T tmp_0;
  uint32_T tmp_1;

  /* UnitDelay: '<S1>/Output' */
  rtb_FixPtSum1 = UnitTest_DWork.Output_DSTATE + 1UL;

  /* Gain: '<Root>/Gain' incorporates:
   *  UnitDelay: '<S1>/Output'
   */
  tmp_1 = 3276800000UL;
  uMultiWordMul(&tmp_1, 1, &UnitTest_DWork.Output_DSTATE, 1, &tmp_0.chunks[0U],
                2);

  /* DataTypeConversion: '<Root>/Data Type Conversion6' */
  uMultiWordShr(&tmp_0.chunks[0U], 2, 15U, &tmp.chunks[0U], 2);

  /* DataStoreWrite: '<Root>/Update time' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion6'
   */
  UnitTest_DWork.time_since_boot_usec = MultiWord2uLong(&tmp.chunks[0U]);

  /* S-Function "MCHP_MCU_LOAD" Block: <Root>/MCU Load */
  UnitTest_B.U3CH4 = MCHP_MCULoadResult[0];

  /* DataStoreWrite: '<Root>/Update SysStatus Load' */
  mlSysStatus.load = UnitTest_B.U3CH4;

  /* Outputs for Enabled SubSystem: '<Root>/if GPS is Novatel' incorporates:
   *  EnablePort: '<S3>/Enable'
   */

  /* S-Function (MCHP_C_function_Call): '<S3>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]1' */
  getGpsMainData(
                 UnitTest_B.ProducetheGPSMainDataandupdatet
                 );

  /* S-Function (MCHP_C_function_Call): '<S3>/Read the Raw Data from GPS [gpsPort.c]1' */
  getGPSRawData(
                &UnitTest_B.ReadtheRawDatafromGPSgpsPortc1
                );

  /* S-Function (MCHP_C_function_Call): '<S3>/Parse the GPS RAW Data [gps.c//novatel.c]1' */
  gpsParse(
           &UnitTest_B.ReadtheRawDatafromGPSgpsPortc1
           );

  /* End of Outputs for SubSystem: '<Root>/if GPS is Novatel' */

  /* Update for UnitDelay: '<S1>/Output' incorporates:
   *  Switch: '<S5>/FixPt Switch'
   */
  UnitTest_DWork.Output_DSTATE = rtb_FixPtSum1;
}

/* Model step function for TID1 */
void UnitTest_step1(void)              /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (MCHP_BUS_I2C_MASTER): '<Root>/BUS I2C Initialize HMC5883 re-initialize at 0.5Hz' */
  /* number of I2C blocks : 1 ; Current: 1 ; MCHP_I2C_StartImplemented =  1*/
  if (MCHP_I2C2_State == 0)            /* Free for next sequence ?*/
  {
    MCHP_I2C2_Request ++;
    MCHP_I2C2_State = 4;
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else if (MCHP_I2C2_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C2_Request = 0;
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C2_Request++;
}

void UnitTest_step(int_T tid)
{
  switch (tid) {
   case 0 :
    UnitTest_step0();
    break;

   case 1 :
    UnitTest_step1();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void UnitTest_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* block I/O */
  (void) memset(((void *) &UnitTest_B), 0,
                sizeof(BlockIO_UnitTest_T));

  /* states (dwork) */
  (void) memset((void *)&UnitTest_DWork, 0,
                sizeof(D_Work_UnitTest_T));

  /* exported global states */
  mlGpsData = UnitTest_rtZmavlink_gps_raw_int_t;
  mlSysStatus = UnitTest_rtZmavlink_sys_status_t;
  mlAirData = UnitTest_rtZmavlink_scaled_pressure_t;
  mlRawPressureData = UnitTest_rtZmavlink_raw_pressure_t;
  MPU_T = 0U;

  /* S-Function "Microchip MASTER" initialization Block: <Root>/Microchip Master AUAV V3 Board Busy Flag on D2 (RA6) */

  /* Start for S-Function (MCHP_MCU_LOAD): '<Root>/MCU Load' */
  TMR2 = 0;                            /* Initialize Timer 2 Value to 0.  Timer 2 is enabled only when the mcu is not idle */

  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<Root>/BUS I2C Initialize HMC5883 re-initialize at 0.5Hz' */

  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0xA5;                      /* I2C clock = 399772  (400000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;

  /* Start for S-Function (MCHP_BUS_SPI): '<Root>/BUS SPI Initialize MPU 6000 Once at Startup' */

  /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
  SPI1CON1 = 0x0474;
  SPI1CON2 = 0x01;
  SPI1STAT = 0x8014;
  _SPI1IP = 5;                         /* Set SPI Interrupt Priority */
  _SPI1IF = 0;                         /* Reset interrupt Flag */
  _SPI1IE = 1;                         /* Enable Interrupts */

  /* SPI Initialisation sequence executed once */
  /* number of SPI blocks : 1 ; Current: 1 ; MCHP_SPI_StartImplemented =  1*/
  if (MCHP_SPI1_State == 0)            /* Free for next sequence ?*/
  {
    MCHP_SPI1_State = 1;
    _SPI1IF = 1;                       /* Force Interrupt */
  }

  /* MCHP_UART_Config Block for UART 1: <Root>/UART Configuration UAV V3 UART 3/Initialize */
  /* Initialisation sequence for UART 1 */
  {
    const uint8_T InitSequence[11] = { 72, 101, 108, 108, 111, 32, 87, 111, 114,
      108, 100 };

    U1BRG = 0x9B5C;                    /* Baud rate: 110 (-0.00%) */
    U1MODE = 0x8000;
    U1STA = 0x2400;
    __delay32(1909091);                /* Wait for 1909091 cycles */

    {
      uint_T i1;
      for (i1 = 0; i1 < 11 ; i1++) {
        while (U1STAbits.UTXBF == 1) ; /* Wait for one empty space within buffer UART */
        U1TXREG = InitSequence[i1];
      }
    }

    while (U1STAbits.TRMT == 0) ;      /* Wait for all value to be sent */
    U1MODE = 0;                        /* Then switch off UART */
  }

  U1BRG = 0x9B5C;                      /* Baud rate: 110 (-0.00%) */
  U1MODE = 0x8000;
  U1STA = 0x2400;

  /* Configure UART1 Tx Interruption */
  MCHP_UART1_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART1_Tx.tail = 0;
  _U1TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U1TXIF = 0;
  _U1TXIE = 1;                         /* Enable Interrupt */

  /* MCHP_UART_Config Block for UART 4: <Root>/UART Configuration UAV V3 UART 4 GPS/Initialize */
  U4BRG = 0x9B5C;                      /* Baud rate: 110 (-0.00%) */
  U4MODE = 0x8000;
  U4STA = 0x2400;

  /* Configure UART4 Tx Interruption */
  MCHP_UART4_Tx.head = 0;              /* Initialise Circular Buffers */
  MCHP_UART4_Tx.tail = 0;
  _U4TXIP = 1;                         /* Tx Interrupt priority set to 1 */
  _U4TXIF = 0;
  _U4TXIE = 1;                         /* Enable Interrupt */

  /* user code (Initialize function Body) */
  uartBufferInit();
  uartMavlinkBufferInit ();
  InitParameterInterface();
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
