/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_76.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
 */

#include "rt_sys_AUAV_V3_TestSensors_76.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for action system:
 *    '<S490>/AxisRotDefault'
 *    '<S700>/AxisRotDefault'
 */
void AUAV_V3_Tes_AxisRotDefault(const real32_T rtu_In1[7], real32_T *rty_1,
  real32_T *rty_2, real32_T *rty_3)
{
  real32_T rtu_In1_0;

  /* Fcn: '<S508>/Fcn1' */
  *rty_1 = rt_atan2f_snf(rtu_In1[0], rtu_In1[1]);

  /* Fcn: '<S508>/Fcn2' */
  if (rtu_In1[2] > 1.0F) {
    rtu_In1_0 = 1.0F;
  } else if (rtu_In1[2] < -1.0F) {
    rtu_In1_0 = -1.0F;
  } else {
    rtu_In1_0 = rtu_In1[2];
  }

  *rty_2 = (real32_T)asin(rtu_In1_0);

  /* End of Fcn: '<S508>/Fcn2' */

  /* Fcn: '<S508>/Fcn3' */
  *rty_3 = rt_atan2f_snf(rtu_In1[3], rtu_In1[4]);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
