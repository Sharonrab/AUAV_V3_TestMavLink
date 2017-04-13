/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_22.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_22.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Output and update for atomic system:
 *    '<S110>/negprotect'
 *    '<S505>/negprotect'
 *    '<S510>/negprotect'
 */
void SLUGS2_negprotect(real32_T rtu_val, rtB_negprotect_SLUGS2_T *localB)
{
  /* MATLAB Function 'Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect': '<S115>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S115>:1:5' */
    /* '<S115>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S115>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
