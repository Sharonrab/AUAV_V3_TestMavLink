/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Barometer_Driver.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "Barometer_Driver.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/* Initial conditions for atomic system: '<Root>/Barometer_Driver' */
void SLUG_Barometer_Driver_Init(void)
{
  /* InitializeConditions for Delay: '<S1>/Delay' */
  SLUGS2_DWork.Delay_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S24>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init(&SLUGS2_DWork.sf_EnablesDisablestheComputatio);
}

/* Start for atomic system: '<Root>/Barometer_Driver' */
void SLU_Barometer_Driver_Start(void)
{
  /* Start for S-Function (MCHP_BUS_I2C_MASTER): '<S1>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* Set-up I2C 2 peripheral */
  I2C2BRG = 0x01FF;                    /* I2C clock = 134331  (100000 with  0.0 \% error) */
  I2C2CON = 0x8300;
  LATAbits.LATA3 = 0;                  /* Might help to reset I2C bus when stuck (Disabling I2C peripheral force SDA & SCL to 0) */
  LATAbits.LATA2 = 0;
  _MI2C2IP = 6;                        /* Set I2C Master Interrupt Priority */
  _MI2C2IF = 0;
  _MI2C2IE = 1;
}

/* Output and update for atomic system: '<Root>/Barometer_Driver' */
void SLUGS2_Barometer_Driver(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_DiscreteZeroPole;
  uint16_T rtb_Product;
  int16_T rtb_B6;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_x;
  uint32_T rtb_Sum6;
  uint32_T qY;
  uint32_T qY_0;
  uint32_T q0;
  uint32_T qY_1;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S1>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* number of I2C blocks : 5 ; Current: 2 ; MCHP_I2C_StartImplemented =  3*/
  if (MCHP_I2C22_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    SLUGS2_B.BUSI2CInitializeBMP180ReadTConv[0] = I2C22_Buff8[0];
    SLUGS2_B.BUSI2CInitializeBMP180ReadTConv[1] = I2C22_Buff8[1];
    MCHP_I2C22_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 4;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C22_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C22_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C22_Request++;

  /* Sum: '<S28>/SumA21' incorporates:
   *  DataTypeConversion: '<S30>/Data Type Conversion1'
   *  DataTypeConversion: '<S30>/Data Type Conversion3'
   *  Delay: '<S28>/Delay11'
   *  Gain: '<S28>/a(2)(1)'
   *  Gain: '<S28>/s(1)'
   *  Gain: '<S30>/Gain'
   *  S-Function (sfix_bitop): '<S30>/Bitwise Operator'
   */
  q0 = ((uint16_T)SLUGS2_B.BUSI2CInitializeBMP180ReadTConv[0] << 8 |
        SLUGS2_B.BUSI2CInitializeBMP180ReadTConv[1]) * 31949UL >> 5;
  qY_1 = q0 + mul_u32_u32_u32_sr15(30771UL, SLUGS2_DWork.Delay11_DSTATE);
  if (qY_1 < q0) {
    qY_1 = MAX_uint32_T;
  }

  /* Sum: '<S28>/SumB21' incorporates:
   *  Delay: '<S28>/Delay11'
   *  Sum: '<S28>/SumA21'
   */
  if (qY_1 > 2147483647UL) {
    q0 = MAX_uint32_T;
  } else {
    q0 = qY_1 << 1;
  }

  if (SLUGS2_DWork.Delay11_DSTATE > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = SLUGS2_DWork.Delay11_DSTATE << 1;
  }

  qY_0 = q0 + rtb_B4;
  if (qY_0 < q0) {
    qY_0 = MAX_uint32_T;
  }

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S1>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  /* number of I2C blocks : 5 ; Current: 3 ; MCHP_I2C_StartImplemented =  3*/
  if (MCHP_I2C23_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    SLUGS2_B.BUSI2CInitializeBMP180ReadPConv[0] = I2C23_Buff8[0];
    SLUGS2_B.BUSI2CInitializeBMP180ReadPConv[1] = I2C23_Buff8[1];
    MCHP_I2C23_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 20;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C23_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C23_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C23_Request++;

  /* Sum: '<S29>/SumA21' incorporates:
   *  DataTypeConversion: '<S31>/Data Type Conversion1'
   *  DataTypeConversion: '<S31>/Data Type Conversion3'
   *  Delay: '<S29>/Delay11'
   *  Gain: '<S29>/a(2)(1)'
   *  Gain: '<S29>/s(1)'
   *  Gain: '<S31>/Gain'
   *  S-Function (sfix_bitop): '<S31>/Bitwise Operator'
   */
  q0 = ((uint16_T)SLUGS2_B.BUSI2CInitializeBMP180ReadPConv[0] << 8 |
        SLUGS2_B.BUSI2CInitializeBMP180ReadPConv[1]) * 31949UL >> 5;
  qY = q0 + mul_u32_u32_u32_sr15(30771UL, SLUGS2_DWork.Delay11_DSTATE_j);
  if (qY < q0) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S29>/SumB21' incorporates:
   *  Delay: '<S29>/Delay11'
   *  Sum: '<S29>/SumA21'
   */
  if (qY > 2147483647UL) {
    q0 = MAX_uint32_T;
  } else {
    q0 = qY << 1;
  }

  if (SLUGS2_DWork.Delay11_DSTATE_j > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = SLUGS2_DWork.Delay11_DSTATE_j << 1;
  }

  rtb_Sum6 = q0 + rtb_B4;
  if (rtb_Sum6 < q0) {
    rtb_Sum6 = MAX_uint32_T;
  }

  /* Outputs for Enabled SubSystem: '<S1>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S27>/Enable'
   */
  /* Delay: '<S1>/Delay' */
  if (SLUGS2_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S27>/BUS I2C Initialize BMP180 read Calibration data @ 100Hz' */
    /* number of I2C blocks : 5 ; Current: 1 ; MCHP_I2C_StartImplemented =  3*/
    {
      /* Enable I2C sequence */
      MCHP_I2C21_Request ++;
      MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 36;
      if (MCHP_I2C2_Queue.head >= 5)   /* There are 5 blocks I2C2, max idx for queue is 5 */
        MCHP_I2C2_Queue.head = 0;
      else
        MCHP_I2C2_Queue.head ++;
      if (MCHP_I2C2_State == 0)
        _MI2C2IF = 1;

      /* Wait for end of SPI sequence (handle within an interrupt) */
      while (MCHP_I2C21_Request != 0) ;/* Wait until end of SPI sequence. */
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[0] = I2C21_Buff8[0];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[1] = I2C21_Buff8[1];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[2] = I2C21_Buff8[2];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[3] = I2C21_Buff8[3];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[4] = I2C21_Buff8[4];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[5] = I2C21_Buff8[5];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[6] = I2C21_Buff8[6];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[7] = I2C21_Buff8[7];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[8] = I2C21_Buff8[8];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[9] = I2C21_Buff8[9];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[10] = I2C21_Buff8[10];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[11] = I2C21_Buff8[11];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[12] = I2C21_Buff8[12];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[13] = I2C21_Buff8[13];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[14] = I2C21_Buff8[14];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[15] = I2C21_Buff8[15];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[16] = I2C21_Buff8[16];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[17] = I2C21_Buff8[17];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[18] = I2C21_Buff8[18];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[19] = I2C21_Buff8[19];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[20] = I2C21_Buff8[20];
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[21] = I2C21_Buff8[21];
    }

    /* DataTypeConversion: '<S27>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S44>/Data Type Conversion1'
     *  DataTypeConversion: '<S44>/Data Type Conversion3'
     *  Gain: '<S44>/Gain'
     *  S-Function (sfix_bitop): '<S44>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition3 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S27>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S43>/Data Type Conversion1'
     *  DataTypeConversion: '<S43>/Data Type Conversion3'
     *  Gain: '<S43>/Gain'
     *  S-Function (sfix_bitop): '<S43>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition2 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S27>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S45>/Data Type Conversion1'
     *  DataTypeConversion: '<S45>/Data Type Conversion3'
     *  Gain: '<S45>/Gain'
     *  S-Function (sfix_bitop): '<S45>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition4 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S27>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S49>/Data Type Conversion1'
     *  DataTypeConversion: '<S49>/Data Type Conversion3'
     *  Gain: '<S49>/Gain'
     *  S-Function (sfix_bitop): '<S49>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition8 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S27>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S50>/Data Type Conversion1'
     *  DataTypeConversion: '<S50>/Data Type Conversion3'
     *  Gain: '<S50>/Gain'
     *  S-Function (sfix_bitop): '<S50>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition9 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S27>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S41>/Data Type Conversion1'
     *  DataTypeConversion: '<S41>/Data Type Conversion3'
     *  Gain: '<S41>/Gain'
     *  S-Function (sfix_bitop): '<S41>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition11 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S27>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S42>/Data Type Conversion1'
     *  DataTypeConversion: '<S42>/Data Type Conversion3'
     *  Gain: '<S42>/Gain'
     *  S-Function (sfix_bitop): '<S42>/Bitwise Operator'
     */
    SLUGS2_B.RateTransition12 = (int16_T)((uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S46>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S46>/Data Type Conversion1'
     *  DataTypeConversion: '<S46>/Data Type Conversion3'
     *  Gain: '<S46>/Gain'
     */
    SLUGS2_B.RateTransition5 = (uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S47>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S47>/Data Type Conversion1'
     *  DataTypeConversion: '<S47>/Data Type Conversion3'
     *  Gain: '<S47>/Gain'
     */
    SLUGS2_B.RateTransition6 = (uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S48>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S48>/Data Type Conversion1'
     *  DataTypeConversion: '<S48>/Data Type Conversion3'
     *  Gain: '<S48>/Gain'
     */
    SLUGS2_B.RateTransition7 = (uint16_T)
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      SLUGS2_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S1>/Delay' */
  /* End of Outputs for SubSystem: '<S1>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Product: '<S23>/Product' incorporates:
   *  Sum: '<S23>/Sum'
   *  Sum: '<S28>/SumB21'
   */
  rtb_B4 = (uint32_T)((uint16_T)(qY_0 >> 16) - SLUGS2_B.RateTransition7) *
    SLUGS2_B.RateTransition6;
  rtb_Product = (uint16_T)(((uint16_T)((int16_T)rtb_B4 & 16384) != 0U) + (rtb_B4
    >> 15));

  /* Sum: '<S23>/Sum2' incorporates:
   *  Bias: '<S23>/Bias'
   *  Bias: '<S23>/Bias1'
   *  Product: '<S23>/Product1'
   *  Sum: '<S23>/Sum1'
   */
  rtb_B6 = (div_s16s32_round((int32_T)SLUGS2_B.RateTransition11 << 11,
             rtb_Product + SLUGS2_B.RateTransition12) + (int16_T)rtb_Product) -
    4000;

  /* MATLAB Function: '<S24>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat(&SLUGS2_B.sf_EnablesDisablestheComputatio,
    &SLUGS2_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S24>/Zero Out Height' incorporates:
   *  EnablePort: '<S37>/Enable'
   */
  if (SLUGS2_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Sum: '<S37>/Sum' incorporates:
     *  Constant: '<S24>/Constant5'
     *  Delay: '<S37>/Integer Delay'
     */
    SLUGS2_B.Sum_k = 0.0F - SLUGS2_DWork.IntegerDelay_DSTATE_jh;
  }

  /* End of Outputs for SubSystem: '<S24>/Zero Out Height' */

  /* Math: '<S23>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_B6 * rtb_B6;

  /* Sum: '<S23>/Sum6' incorporates:
   *  Bias: '<S23>/Bias2'
   *  Product: '<S23>/Product2'
   *  Product: '<S23>/Product3'
   *  Sum: '<S23>/Sum3'
   *  Sum: '<S23>/Sum4'
   *  Sum: '<S29>/SumB21'
   */
  rtb_Sum6 = (uint32_T)(uint16_T)(rtb_Sum6 >> 16) - (((((int16_T)((int32_T)
    rtb_B6 * SLUGS2_B.RateTransition3 >> 11) + (int16_T)mul_s32_s32_s32_sr23
    (SLUGS2_B.RateTransition9, rtb_Bias5)) + ((int32_T)SLUGS2_B.RateTransition2 <<
    2)) + 2L) >> 2);

  /* Product: '<S23>/Product6' incorporates:
   *  Bias: '<S23>/Bias3'
   *  Bias: '<S23>/Bias4'
   *  Gain: '<S23>/Gain1'
   *  Product: '<S23>/Product4'
   *  Product: '<S23>/Product5'
   *  Sum: '<S23>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15(((((int16_T)((int32_T)rtb_B6 *
    SLUGS2_B.RateTransition4 >> 13) + (int16_T)mul_s32_s32_s32_sr28(rtb_Bias5,
    SLUGS2_B.RateTransition8)) + 2) >> 2) + 32768L, SLUGS2_B.RateTransition5);

  /* Switch: '<S23>/Switch' incorporates:
   *  Gain: '<S23>/Gain15'
   *  Gain: '<S23>/Gain22'
   *  Product: '<S23>/Product7'
   *  Product: '<S23>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S23>/Switch' */

  /* Gain: '<S23>/Gain16' */
  rtb_Bias5 = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >>
    8));

  /* Sum: '<S23>/Sum8' incorporates:
   *  Bias: '<S23>/Bias5'
   *  Gain: '<S23>/Gain17'
   *  Gain: '<S23>/Gain19'
   *  Gain: '<S23>/Gain21'
   *  Math: '<S23>/Math Function2'
   *  Sum: '<S23>/Sum7'
   */
  rtb_Bias5 = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat
    (rtb_Bias5, rtb_Bias5)) + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >>
               4) + (int32_T)rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S24>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S36>/Enable'
   */
  if (SLUGS2_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* DataTypeConversion: '<S36>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Bias5;

    /* DiscreteZeroPole: '<S39>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole += 0.029119852459414206*
        SLUGS2_DWork.DiscreteZeroPole_DSTATE_nk;
    }

    /* Saturate: '<S36>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S36>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole > 120000.0F) {
      SLUGS2_B.u0k120k_i = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole < 80000.0F) {
      SLUGS2_B.u0k120k_i = 80000.0F;
    } else {
      SLUGS2_B.u0k120k_i = (real32_T)rtb_DiscreteZeroPole;
    }

    /* End of Saturate: '<S36>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S39>/Discrete Zero-Pole' */
    {
      SLUGS2_DWork.DiscreteZeroPole_DSTATE_nk = rtb_DataTypeConversion +
        0.97044334975369462*SLUGS2_DWork.DiscreteZeroPole_DSTATE_nk;
    }
  }

  /* End of Outputs for SubSystem: '<S24>/Initial Baro Bias' */

  /* Product: '<S33>/Divide' incorporates:
   *  Sum: '<S33>/Sum2'
   */
  rtb_x = ((real32_T)rtb_Bias5 - SLUGS2_B.u0k120k_i) / SLUGS2_B.u0k120k_i;

  /* Sum: '<S33>/Sum1' incorporates:
   *  Constant: '<S33>/Constant2'
   *  Constant: '<S33>/Constant3'
   *  Constant: '<S33>/Constant4'
   *  Constant: '<S33>/Constant5'
   *  Gain: '<S38>/Unit Conversion'
   *  Product: '<S33>/Divide1'
   *  Product: '<S33>/Divide2'
   *  Product: '<S33>/Divide3'
   *  Product: '<S33>/Divide4'
   *  Sum: '<S33>/Sum3'
   */
  rtb_x = ((rtb_x * rtb_x * 0.093502529F + rtb_x * -0.188893303F) +
           2.18031291E-5F) * 145473.5F * 0.3048F;

  /* Outputs for Enabled SubSystem: '<S24>/Enabled Subsystem' */

  /* Logic: '<S24>/Logical Operator' incorporates:
   *  Sum: '<S24>/Sum1'
   */
  SLUGS2_EnabledSubsystem(!(SLUGS2_B.sf_EnablesDisablestheComputatio.tOut != 0.0),
    SLUGS2_B.Sum_k + rtb_x, &SLUGS2_B.EnabledSubsystem);

  /* End of Outputs for SubSystem: '<S24>/Enabled Subsystem' */

  /* Update for Delay: '<S28>/Delay11' incorporates:
   *  Sum: '<S28>/SumA21'
   */
  SLUGS2_DWork.Delay11_DSTATE = qY_1;

  /* Update for Delay: '<S29>/Delay11' incorporates:
   *  Sum: '<S29>/SumA21'
   */
  SLUGS2_DWork.Delay11_DSTATE_j = qY;

  /* Update for Delay: '<S1>/Delay' incorporates:
   *  Constant: '<S1>/Constant'
   */
  SLUGS2_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S24>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S37>/Enable'
   */
  if (SLUGS2_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Update for Delay: '<S37>/Integer Delay' */
    SLUGS2_DWork.IntegerDelay_DSTATE_jh = rtb_x;
  }

  /* End of Update for SubSystem: '<S24>/Zero Out Height' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
