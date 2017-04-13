/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_3.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.262
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Nov 23 10:01:40 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_3.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Initial conditions for atomic system:
 *    '<S28>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S622>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW)
{
  localDW->aveCount = 1.0;
  localDW->tIni = 1.0;
}

/*
 * Output and update for atomic system:
 *    '<S28>/Enables//Disables the Computation of  initial Baro Bias'
 *    '<S622>/Enables//Disables the Computation of  initial Baro Bias'
 */
void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW)
{
  /* MATLAB Function 'Barometer_Driver/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias': '<S39>:1' */
  if (localDW->aveCount < 2500.0) {
    /* '<S39>:1:11' */
    /* '<S39>:1:12' */
    localDW->aveCount++;
  }

  if (localDW->aveCount == 2500.0) {
    /* '<S39>:1:15' */
    /* '<S39>:1:16' */
    localDW->tIni = 0.0;

    /* '<S39>:1:17' */
    localDW->aveCount++;
  }

  /* '<S39>:1:20' */
  localB->tOut = localDW->tIni;

  /* if aveCount > 500 */
  /*     tIni = 0; */
  /* end */
  /*  if isempty(aveCount) */
  /*      aveCount =1; */
  /*      initBias = [0.0 0.0 0.0]'; */
  /*  end; */
  /*       */
  /*  if (aveCount<10) */
  /*      initBias = initBias+ Raw; */
  /*      bias = initBias/aveCount; */
  /*      aveCount = aveCount +1; */
  /*  end; */
  /*   */
  /*  if (aveCount == 10) */
  /*      initBias = initBias/(aveCount-1); */
  /*      aveCount = 100; */
  /*  end; */
  /*   */
  /*  if (aveCount == 100) */
  /*      bias = initBias; */
  /*  end; */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
