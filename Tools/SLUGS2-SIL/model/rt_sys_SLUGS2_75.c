/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_75.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_75.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int16_T u0_0;
  int16_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2(u0_0, u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/*
 * Output and update for action system:
 *    '<S483>/AxisRotZeroR3'
 *    '<S688>/AxisRotZeroR3'
 */
void SLUGS2_AxisRotZeroR3(const real32_T rtu_In1[7], real32_T *rty_1, real32_T
  *rty_2, real32_T *rty_3)
{
  real32_T rtu_In1_0;

  /* Fcn: '<S502>/Fcn1' */
  *rty_1 = rt_atan2f_snf(rtu_In1[5], rtu_In1[6]);

  /* Fcn: '<S502>/Fcn2' */
  if (rtu_In1[2] > 1.0F) {
    rtu_In1_0 = 1.0F;
  } else if (rtu_In1[2] < -1.0F) {
    rtu_In1_0 = -1.0F;
  } else {
    rtu_In1_0 = rtu_In1[2];
  }

  *rty_2 = (real32_T)asin(rtu_In1_0);

  /* End of Fcn: '<S502>/Fcn2' */

  /* Fcn: '<S502>/Fcn3' */
  *rty_3 = 0.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
