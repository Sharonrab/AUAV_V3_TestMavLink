/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_22.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.262
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Nov 23 10:01:40 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_22.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for atomic system:
 *    '<S117>/negprotect'
 *    '<S512>/negprotect'
 *    '<S517>/negprotect'
 */
void AUAV_V3_TestSen_negprotect(real32_T rtu_val,
  rtB_negprotect_AUAV_V3_TestSe_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect': '<S122>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S122>:1:5' */
    /* '<S122>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S122>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
