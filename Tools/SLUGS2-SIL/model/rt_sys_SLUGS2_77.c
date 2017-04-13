/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_77.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_77.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Output and update for atomic system:
 *    '<S471>/Embedded MATLAB Function'
 *    '<S17>/Embedded MATLAB Function1'
 */
void S_EmbeddedMATLABFunction_l(real32_T rtu_u, rtB_EmbeddedMATLABFunction_d_T
  *localB)
{
  /* MATLAB Function 'Position_and_Attitude_Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S484>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_u < 0.0F) {
    /* '<S484>:1:5' */
    /* '<S484>:1:6' */
    localB->y = rtu_u + 6.28318548F;
  } else {
    /* '<S484>:1:8' */
    localB->y = rtu_u;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
