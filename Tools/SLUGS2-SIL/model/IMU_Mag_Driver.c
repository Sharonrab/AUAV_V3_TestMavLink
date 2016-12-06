/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: IMU_Mag_Driver.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.264
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Dec 03 23:00:15 2016
 */

#include "IMU_Mag_Driver.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/* Start for atomic system: '<Root>/IMU_Mag_Driver' */
void AUAV__IMU_Mag_Driver_Start(void)
{
  /* Start for S-Function (MCHP_BUS_SPI): '<S60>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (100 Hz)' */
  /* Set-up SPI 1 peripheral with Fsck = 364583  (364583 with  0.0 \% error)  */
  SPI1CON1 = 0x0474;
  SPI1CON2 = 0x01;
  SPI1STAT = 0x8014;
  _SPI1IP = 5;                         /* Set SPI Interrupt Priority */
  _SPI1IF = 0;                         /* Reset interrupt Flag */
  _SPI1IE = 1;                         /* Enable Interrupts */
}

/* Output and update for atomic system: '<Root>/IMU_Mag_Driver' */
void AUAV_V3_Tes_IMU_Mag_Driver(void)
{
  /* S-Function (MCHP_BUS_SPI): '<S60>/BUS SPI Read MPU 6050 Axyz, T°, Gxyz (100 Hz)' */
  /* number of SPI blocks : 2 ; Current: 1 ; MCHP_SPI_StartImplemented =  1*/
  if (MCHP_SPI11_Request == 0)         /* Last SPI sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestSensors_B.U1CH8[0] = SPI11_Buff16[0];
    AUAV_V3_TestSensors_B.U1CH8[1] = SPI11_Buff16[1];
    AUAV_V3_TestSensors_B.U1CH8[2] = SPI11_Buff16[2];
    AUAV_V3_TestSensors_B.U3CH4_e = SPI11_Buff16[3];
    AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[0] = SPI11_Buff16[4];
    AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[1] = SPI11_Buff16[5];
    AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[2] = SPI11_Buff16[6];
    MCHP_SPI11_Request = 1;
    MCHP_SPI1_Queue.buffer[MCHP_SPI1_Queue.head] = 1;
    if (MCHP_SPI1_Queue.head >= 2)     /* There are 2 blocks SPI1, max idx for queue is 2 */
      MCHP_SPI1_Queue.head = 0;
    else
      MCHP_SPI1_Queue.head ++;
    if (MCHP_SPI1_State == 0)
      _SPI1IF = 1;                     /* Force Interrupt */
  }

  /* S-Function (MCHP_BUS_I2C_MASTER): '<S5>/BUS I2C Read HMC5883 Magn (100 Hz)1' */
  /* number of I2C blocks : 5 ; Current: 4 ; MCHP_I2C_StartImplemented =  4*/
  if (MCHP_I2C24_Request == 0)         /* Last I2C sequence from this block is finished (not in the queue ?) */
  {
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[0] = I2C24_Buff8[0];
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[1] = I2C24_Buff8[1];
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[2] = I2C24_Buff8[2];
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[3] = I2C24_Buff8[3];
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[4] = I2C24_Buff8[4];
    AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[5] = I2C24_Buff8[5];
    MCHP_I2C24_Request ++;
    MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.head] = 87;
    if (MCHP_I2C2_Queue.head >= 5)     /* There are 5 blocks I2C2, max idx for queue is 5 */
      MCHP_I2C2_Queue.head = 0;
    else
      MCHP_I2C2_Queue.head ++;
    if (MCHP_I2C2_State == 0)
      _MI2C2IF = 1;                    /* Force Interrupt */
  } else if (MCHP_I2C24_Request > 3) {
    I2C2CONbits.I2CEN = 0;             /* Switch Off I2C*/
    TRISAbits.TRISA3 = 0;              /* Set I2C Port as Port Output */
    TRISAbits.TRISA2 = 0;
    MCHP_I2C24_Request = 0;            /* This block might still be in the queue provided another I2C block stuck the bus, Next block execution will however add another Request */
    MCHP_I2C2_State = 1;               /* try to Reset I2C BUS */
    _MI2C2IF = 1;                      /* Force Interrupt */
  } else
    MCHP_I2C24_Request++;

  /* Outputs for Enabled SubSystem: '<S5>/Disable path through in HIL Subsystem1' incorporates:
   *  EnablePort: '<S61>/Enable'
   */
  /* Logic: '<S5>/Logical Operator1' */
  if (!(AUAV_V3_TestSensors_B.DataStoreRead != 0.0)) {
    /* DataStoreWrite: '<S61>/Update Raw IMU DATA1' incorporates:
     *  Bias: '<S60>/Bias11'
     *  Bias: '<S60>/Bias6'
     *  Bias: '<S60>/Bias7'
     *  Bias: '<S60>/Bias8'
     *  Bias: '<S60>/Bias9'
     *  DataTypeConversion: '<S60>/Data Type Conversion2'
     *  DataTypeConversion: '<S60>/Data Type Conversion3'
     *  DataTypeConversion: '<S62>/Data Type Conversion3'
     *  DataTypeConversion: '<S64>/Data Type Conversion3'
     *  DataTypeConversion: '<S65>/Data Type Conversion3'
     */
    mlRawImuData.xacc = (int16_T)AUAV_V3_TestSensors_B.U1CH8[0] - 50;
    mlRawImuData.yacc = (int16_T)AUAV_V3_TestSensors_B.U1CH8[1] - 350;
    mlRawImuData.zacc = (int16_T)AUAV_V3_TestSensors_B.U1CH8[2] + 180;
    mlRawImuData.xgyro = (int16_T)
      AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[0] + 96;
    mlRawImuData.ygyro = (int16_T)
      AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[1] - 15;
    mlRawImuData.zgyro = (int16_T)
      AUAV_V3_TestSensors_B.BUSSPIReadMPU6050AxyzTGxyz100Hz[2] + 4;
    mlRawImuData.xmag = AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[1];
    mlRawImuData.ymag = AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[3];
    mlRawImuData.zmag = AUAV_V3_TestSensors_B.BUSI2CReadHMC5883Magn100Hz1[5];

    /* DataStoreWrite: '<S61>/Update MPU_T1' */
    MPU_T = AUAV_V3_TestSensors_B.U3CH4_e;
  }

  /* End of Logic: '<S5>/Logical Operator1' */
  /* End of Outputs for SubSystem: '<S5>/Disable path through in HIL Subsystem1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
