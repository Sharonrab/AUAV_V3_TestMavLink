/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_77.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
 */

#include "rt_sys_AUAV_V3_TestSensors_77.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S478>/Embedded MATLAB Function'
 *    '<S21>/Embedded MATLAB Function1'
 */
void A_EmbeddedMATLABFunction_l(real32_T rtu_u, rtB_EmbeddedMATLABFunction_d_T
  *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S491>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_u < 0.0F) {
    /* '<S491>:1:5' */
    /* '<S491>:1:6' */
    localB->y = rtu_u + 6.28318548F;
  } else {
    /* '<S491>:1:8' */
    localB->y = rtu_u;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
