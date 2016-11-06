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
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.241
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Nov 05 08:28:55 2016
 */

#include "Barometer_Driver.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Initial conditions for atomic system: '<Root>/Barometer_Driver' */
void AUAV_Barometer_Driver_Init(void)
{
  /* InitializeConditions for Delay: '<S1>/Delay' */
  AUAV_V3_TestSensors_DWork.Delay_DSTATE = 1U;

  /* InitializeConditions for MATLAB Function: '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheCom_Init
    (&AUAV_V3_TestSensors_DWork.sf_EnablesDisablestheComputatio);
}

/* Start for atomic system: '<Root>/Barometer_Driver' */
void AUA_Barometer_Driver_Start(void)
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
void AUAV_V3_T_Barometer_Driver(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_DiscreteZeroPole;
  uint16_T rtb_Sum1_n;
  uint16_T rtb_SumB21;
  boolean_T rtb_LogicalOperator;
  int16_T rtb_B6;
  int32_T rtb_Bias5;
  uint32_T rtb_B4;
  real32_T rtb_x;
  uint32_T rtb_Sum6;
  uint32_T qY;
  uint32_T qY_0;

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S1>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz' */
  /* number of I2C blocks : 5 ; Current: 2 ; MCHP_I2C_StartImplemented =  3*/
  if (MCHP_I2C22_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadTConv[0] = I2C22_Buff8[0];
    AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadTConv[1] = I2C22_Buff8[1];
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

  /* Sum: '<S32>/SumA21' incorporates:
   *  DataTypeConversion: '<S34>/Data Type Conversion1'
   *  DataTypeConversion: '<S34>/Data Type Conversion3'
   *  Delay: '<S32>/Delay11'
   *  Gain: '<S32>/a(2)(1)'
   *  Gain: '<S32>/s(1)'
   *  Gain: '<S34>/Gain'
   *  S-Function (sfix_bitop): '<S34>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadTConv[0] <<
            8 | AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadTConv[1]) *
    31949UL >> 5;
  qY_0 = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV_V3_TestSensors_DWork.Delay11_DSTATE);
  if (qY_0 < rtb_B4) {
    qY_0 = MAX_uint32_T;
  }

  /* Sum: '<S32>/SumB21' incorporates:
   *  Delay: '<S32>/Delay11'
   *  Sum: '<S32>/SumA21'
   */
  if (qY_0 > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY_0 << 1;
  }

  if (AUAV_V3_TestSensors_DWork.Delay11_DSTATE > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV_V3_TestSensors_DWork.Delay11_DSTATE << 1;
  }

  qY = rtb_B4 + rtb_Sum6;
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  rtb_Sum1_n = (uint16_T)(qY >> 16);

  /* End of Sum: '<S32>/SumB21' */

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S1>/BUS I2C Initialize BMP180 Read P, Convert T° @100Hz' */
  /* number of I2C blocks : 5 ; Current: 3 ; MCHP_I2C_StartImplemented =  3*/
  if (MCHP_I2C23_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadPConv[0] = I2C23_Buff8[0];
    AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadPConv[1] = I2C23_Buff8[1];
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

  /* Sum: '<S33>/SumA21' incorporates:
   *  DataTypeConversion: '<S35>/Data Type Conversion1'
   *  DataTypeConversion: '<S35>/Data Type Conversion3'
   *  Delay: '<S33>/Delay11'
   *  Gain: '<S33>/a(2)(1)'
   *  Gain: '<S33>/s(1)'
   *  Gain: '<S35>/Gain'
   *  S-Function (sfix_bitop): '<S35>/Bitwise Operator'
   */
  rtb_B4 = ((uint16_T)AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadPConv[0] <<
            8 | AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180ReadPConv[1]) *
    31949UL >> 5;
  qY = rtb_B4 + mul_u32_u32_u32_sr15(30771UL,
    AUAV_V3_TestSensors_DWork.Delay11_DSTATE_j);
  if (qY < rtb_B4) {
    qY = MAX_uint32_T;
  }

  /* Sum: '<S33>/SumB21' incorporates:
   *  Delay: '<S33>/Delay11'
   *  Sum: '<S33>/SumA21'
   */
  if (qY > 2147483647UL) {
    rtb_B4 = MAX_uint32_T;
  } else {
    rtb_B4 = qY << 1;
  }

  if (AUAV_V3_TestSensors_DWork.Delay11_DSTATE_j > 2147483647UL) {
    rtb_Sum6 = MAX_uint32_T;
  } else {
    rtb_Sum6 = AUAV_V3_TestSensors_DWork.Delay11_DSTATE_j << 1;
  }

  rtb_Sum6 += rtb_B4;
  if (rtb_Sum6 < rtb_B4) {
    rtb_Sum6 = MAX_uint32_T;
  }

  rtb_SumB21 = (uint16_T)(rtb_Sum6 >> 16);

  /* End of Sum: '<S33>/SumB21' */

  /* Logic: '<S1>/Logical Operator' */
  rtb_LogicalOperator = !(AUAV_V3_TestSensors_B.HILManualSwitch != 0.0);

  /* Outputs for Enabled SubSystem: '<S1>/Disable path through in HIL Subsystem' incorporates:
   *  EnablePort: '<S29>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* DataStoreWrite: '<S29>/Update RawPressure' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion7'
     *  DataTypeConversion: '<S1>/Data Type Conversion8'
     */
    mlRawPressureData.press_abs = (int16_T)rtb_Sum1_n;
    mlRawPressureData.temperature = (int16_T)rtb_SumB21;
  }

  /* End of Outputs for SubSystem: '<S1>/Disable path through in HIL Subsystem' */

  /* Outputs for Enabled SubSystem: '<S1>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' incorporates:
   *  EnablePort: '<S31>/Enable'
   */
  /* Delay: '<S1>/Delay' */
  if (AUAV_V3_TestSensors_DWork.Delay_DSTATE > 0U) {
    /* S-Function (MCHP_BUS_I2C_MASTER): '<S31>/BUS I2C Initialize BMP180 read Calibration data @ 100Hz' */
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
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[0] = I2C21_Buff8[0];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[1] = I2C21_Buff8[1];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[2] = I2C21_Buff8[2];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[3] = I2C21_Buff8[3];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[4] = I2C21_Buff8[4];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[5] = I2C21_Buff8[5];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[6] = I2C21_Buff8[6];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[7] = I2C21_Buff8[7];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[8] = I2C21_Buff8[8];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[9] = I2C21_Buff8[9];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[10] = I2C21_Buff8[10];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[11] = I2C21_Buff8[11];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[12] = I2C21_Buff8[12];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[13] = I2C21_Buff8[13];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[14] = I2C21_Buff8[14];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[15] = I2C21_Buff8[15];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[16] = I2C21_Buff8[16];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[17] = I2C21_Buff8[17];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[18] = I2C21_Buff8[18];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[19] = I2C21_Buff8[19];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[20] = I2C21_Buff8[20];
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[21] = I2C21_Buff8[21];
    }

    /* DataTypeConversion: '<S31>/Data Type Conversion1' incorporates:
     *  DataTypeConversion: '<S48>/Data Type Conversion1'
     *  DataTypeConversion: '<S48>/Data Type Conversion3'
     *  Gain: '<S48>/Gain'
     *  S-Function (sfix_bitop): '<S48>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition3 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[2] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[3]);

    /* DataTypeConversion: '<S31>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S47>/Data Type Conversion1'
     *  DataTypeConversion: '<S47>/Data Type Conversion3'
     *  Gain: '<S47>/Gain'
     *  S-Function (sfix_bitop): '<S47>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition2 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[0] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[1]);

    /* DataTypeConversion: '<S31>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S49>/Data Type Conversion1'
     *  DataTypeConversion: '<S49>/Data Type Conversion3'
     *  Gain: '<S49>/Gain'
     *  S-Function (sfix_bitop): '<S49>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition4 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[4] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[5]);

    /* DataTypeConversion: '<S31>/Data Type Conversion4' incorporates:
     *  DataTypeConversion: '<S53>/Data Type Conversion1'
     *  DataTypeConversion: '<S53>/Data Type Conversion3'
     *  Gain: '<S53>/Gain'
     *  S-Function (sfix_bitop): '<S53>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition8 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[12] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[13]);

    /* DataTypeConversion: '<S31>/Data Type Conversion5' incorporates:
     *  DataTypeConversion: '<S54>/Data Type Conversion1'
     *  DataTypeConversion: '<S54>/Data Type Conversion3'
     *  Gain: '<S54>/Gain'
     *  S-Function (sfix_bitop): '<S54>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition9 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[14] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[15]);

    /* DataTypeConversion: '<S31>/Data Type Conversion7' incorporates:
     *  DataTypeConversion: '<S45>/Data Type Conversion1'
     *  DataTypeConversion: '<S45>/Data Type Conversion3'
     *  Gain: '<S45>/Gain'
     *  S-Function (sfix_bitop): '<S45>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition11 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[18] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[19]);

    /* DataTypeConversion: '<S31>/Data Type Conversion8' incorporates:
     *  DataTypeConversion: '<S46>/Data Type Conversion1'
     *  DataTypeConversion: '<S46>/Data Type Conversion3'
     *  Gain: '<S46>/Gain'
     *  S-Function (sfix_bitop): '<S46>/Bitwise Operator'
     */
    AUAV_V3_TestSensors_B.RateTransition12 = (int16_T)((uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[20] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[21]);

    /* S-Function (sfix_bitop): '<S50>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S50>/Data Type Conversion1'
     *  DataTypeConversion: '<S50>/Data Type Conversion3'
     *  Gain: '<S50>/Gain'
     */
    AUAV_V3_TestSensors_B.RateTransition5 = (uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[6] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[7];

    /* S-Function (sfix_bitop): '<S51>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S51>/Data Type Conversion1'
     *  DataTypeConversion: '<S51>/Data Type Conversion3'
     *  Gain: '<S51>/Gain'
     */
    AUAV_V3_TestSensors_B.RateTransition6 = (uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[8] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[9];

    /* S-Function (sfix_bitop): '<S52>/Bitwise Operator' incorporates:
     *  DataTypeConversion: '<S52>/Data Type Conversion1'
     *  DataTypeConversion: '<S52>/Data Type Conversion3'
     *  Gain: '<S52>/Gain'
     */
    AUAV_V3_TestSensors_B.RateTransition7 = (uint16_T)
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[10] << 8 |
      AUAV_V3_TestSensors_B.BUSI2CInitializeBMP180readCalib[11];
  }

  /* End of Delay: '<S1>/Delay' */
  /* End of Outputs for SubSystem: '<S1>/I2C Initialisation Read EEPROM Calibration data Once (Blocking Function)' */

  /* Sum: '<S27>/Sum' */
  rtb_Sum1_n -= AUAV_V3_TestSensors_B.RateTransition7;

  /* Product: '<S27>/Product' */
  rtb_Sum6 = (uint32_T)rtb_Sum1_n * AUAV_V3_TestSensors_B.RateTransition6;
  rtb_Sum1_n = (uint16_T)(((uint16_T)((int16_T)rtb_Sum6 & 16384) != 0U) +
    (rtb_Sum6 >> 15));

  /* Sum: '<S27>/Sum2' incorporates:
   *  Bias: '<S27>/Bias'
   *  Bias: '<S27>/Bias1'
   *  Product: '<S27>/Product1'
   *  Sum: '<S27>/Sum1'
   */
  rtb_B6 = (div_s16s32_round((int32_T)AUAV_V3_TestSensors_B.RateTransition11 <<
             11, rtb_Sum1_n + AUAV_V3_TestSensors_B.RateTransition12) + (int16_T)
            rtb_Sum1_n) - 4000;

  /* MATLAB Function: '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
  EnablesDisablestheComputat
    (&AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputatio,
     &AUAV_V3_TestSensors_DWork.sf_EnablesDisablestheComputatio);

  /* Outputs for Enabled SubSystem: '<S28>/Zero Out Height' incorporates:
   *  EnablePort: '<S41>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Sum: '<S41>/Sum' incorporates:
     *  Constant: '<S28>/Constant5'
     *  Delay: '<S41>/Integer Delay'
     */
    AUAV_V3_TestSensors_B.Sum_k = 0.0F -
      AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_jh;
  }

  /* End of Outputs for SubSystem: '<S28>/Zero Out Height' */

  /* Math: '<S27>/Math Function' */
  rtb_Bias5 = (int32_T)rtb_B6 * rtb_B6;

  /* Sum: '<S27>/Sum6' incorporates:
   *  Bias: '<S27>/Bias2'
   *  Product: '<S27>/Product2'
   *  Product: '<S27>/Product3'
   *  Sum: '<S27>/Sum3'
   *  Sum: '<S27>/Sum4'
   */
  rtb_Sum6 = (uint32_T)rtb_SumB21 - (((((int16_T)((int32_T)rtb_B6 *
    AUAV_V3_TestSensors_B.RateTransition3 >> 11) + (int16_T)mul_s32_s32_s32_sr23
    (AUAV_V3_TestSensors_B.RateTransition9, rtb_Bias5)) + ((int32_T)
    AUAV_V3_TestSensors_B.RateTransition2 << 2)) + 2L) >> 2);

  /* Product: '<S27>/Product6' incorporates:
   *  Bias: '<S27>/Bias3'
   *  Bias: '<S27>/Bias4'
   *  Gain: '<S27>/Gain1'
   *  Product: '<S27>/Product4'
   *  Product: '<S27>/Product5'
   *  Sum: '<S27>/Sum9'
   */
  rtb_B4 = mul_u32_s32_u32_sr15(((((int16_T)((int32_T)rtb_B6 *
    AUAV_V3_TestSensors_B.RateTransition4 >> 13) + (int16_T)mul_s32_s32_s32_sr28
    (rtb_Bias5, AUAV_V3_TestSensors_B.RateTransition8)) + 2) >> 2) + 32768L,
    AUAV_V3_TestSensors_B.RateTransition5);

  /* Switch: '<S27>/Switch' incorporates:
   *  Gain: '<S27>/Gain15'
   *  Gain: '<S27>/Gain22'
   *  Product: '<S27>/Product7'
   *  Product: '<S27>/Product8'
   */
  if (rtb_Sum6 > 2147483647UL) {
    rtb_Sum6 = mul_u32_u32_u32_sr11_round(div_repeat_u32_round(rtb_Sum6, rtb_B4,
      15U), 3125UL) << 1;
  } else {
    rtb_Sum6 = div_u32_round(mul_u32_u32_u32_sr11_round(rtb_Sum6 << 16, 3125UL),
      rtb_B4);
  }

  /* End of Switch: '<S27>/Switch' */

  /* Gain: '<S27>/Gain16' */
  rtb_Bias5 = (int32_T)(((uint16_T)((int16_T)rtb_Sum6 & 128) != 0U) + (rtb_Sum6 >>
    8));

  /* Sum: '<S27>/Sum8' incorporates:
   *  Bias: '<S27>/Bias5'
   *  Gain: '<S27>/Gain17'
   *  Gain: '<S27>/Gain19'
   *  Gain: '<S27>/Gain21'
   *  Math: '<S27>/Math Function2'
   *  Sum: '<S27>/Sum7'
   */
  rtb_Bias5 = ((((int32_T)mul_u32_u32_u32_sr15(1519UL, mul_u32_s32_s32_sat
    (rtb_Bias5, rtb_Bias5)) + mul_s32_s32_u32_sr16(-7357L, rtb_Sum6)) + 3791L) >>
               4) + (int32_T)rtb_Sum6;

  /* Outputs for Enabled SubSystem: '<S28>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S40>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* DataTypeConversion: '<S40>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_Bias5;

    /* DiscreteZeroPole: '<S43>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole = 0.014778325123152709*rtb_DataTypeConversion;
      rtb_DiscreteZeroPole += 0.029119852459414206*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_nk;
    }

    /* Saturate: '<S40>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S40>/Data Type Conversion1'
     */
    if ((real32_T)rtb_DiscreteZeroPole > 120000.0F) {
      AUAV_V3_TestSensors_B.u0k120k_i = 120000.0F;
    } else if ((real32_T)rtb_DiscreteZeroPole < 80000.0F) {
      AUAV_V3_TestSensors_B.u0k120k_i = 80000.0F;
    } else {
      AUAV_V3_TestSensors_B.u0k120k_i = (real32_T)rtb_DiscreteZeroPole;
    }

    /* End of Saturate: '<S40>/[80k - 120k]' */
    /* Update for DiscreteZeroPole: '<S43>/Discrete Zero-Pole' */
    {
      AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_nk =
        rtb_DataTypeConversion + 0.97044334975369462*
        AUAV_V3_TestSensors_DWork.DiscreteZeroPole_DSTATE_nk;
    }
  }

  /* End of Outputs for SubSystem: '<S28>/Initial Baro Bias' */

  /* Product: '<S37>/Divide' incorporates:
   *  Sum: '<S37>/Sum2'
   */
  rtb_x = ((real32_T)rtb_Bias5 - AUAV_V3_TestSensors_B.u0k120k_i) /
    AUAV_V3_TestSensors_B.u0k120k_i;

  /* Sum: '<S37>/Sum1' incorporates:
   *  Constant: '<S37>/Constant2'
   *  Constant: '<S37>/Constant3'
   *  Constant: '<S37>/Constant4'
   *  Constant: '<S37>/Constant5'
   *  Gain: '<S42>/Unit Conversion'
   *  Product: '<S37>/Divide1'
   *  Product: '<S37>/Divide2'
   *  Product: '<S37>/Divide3'
   *  Product: '<S37>/Divide4'
   *  Sum: '<S37>/Sum3'
   */
  rtb_x = ((rtb_x * rtb_x * 0.093502529F + rtb_x * -0.188893303F) +
           2.18031291E-5F) * 145473.5F * 0.3048F;

  /* Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' */

  /* Logic: '<S28>/Logical Operator' incorporates:
   *  Sum: '<S28>/Sum1'
   */
  AUAV_V3_T_EnabledSubsystem
    (!(AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputatio.tOut != 0.0),
     AUAV_V3_TestSensors_B.Sum_k + rtb_x,
     &AUAV_V3_TestSensors_B.EnabledSubsystem);

  /* End of Outputs for SubSystem: '<S28>/Enabled Subsystem' */

  /* Outputs for Enabled SubSystem: '<S1>/Disable path through in HIL Subsystem1' incorporates:
   *  EnablePort: '<S30>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* DataStoreWrite: '<S30>/Update ScaledPressure' incorporates:
     *  Bias: '<S27>/Bias'
     *  Bias: '<S27>/Bias1'
     *  Gain: '<S1>/Gain21'
     */
    mlAirData.temperature = (int16_T)((rtb_B6 + 4008) * 5L >> 3);
    mlAirData.press_abs = AUAV_V3_TestSensors_B.EnabledSubsystem.In1;
  }

  /* End of Outputs for SubSystem: '<S1>/Disable path through in HIL Subsystem1' */

  /* Update for Delay: '<S32>/Delay11' incorporates:
   *  Sum: '<S32>/SumA21'
   */
  AUAV_V3_TestSensors_DWork.Delay11_DSTATE = qY_0;

  /* Update for Delay: '<S33>/Delay11' incorporates:
   *  Sum: '<S33>/SumA21'
   */
  AUAV_V3_TestSensors_DWork.Delay11_DSTATE_j = qY;

  /* Update for Delay: '<S1>/Delay' incorporates:
   *  Constant: '<S1>/Constant'
   */
  AUAV_V3_TestSensors_DWork.Delay_DSTATE = 0U;

  /* Update for Enabled SubSystem: '<S28>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S41>/Enable'
   */
  if (AUAV_V3_TestSensors_B.sf_EnablesDisablestheComputatio.tOut > 0.0) {
    /* Update for Delay: '<S41>/Integer Delay' */
    AUAV_V3_TestSensors_DWork.IntegerDelay_DSTATE_jh = rtb_x;
  }

  /* End of Update for SubSystem: '<S28>/Zero Out Height' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
