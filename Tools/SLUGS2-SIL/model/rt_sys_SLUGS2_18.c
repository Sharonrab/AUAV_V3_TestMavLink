/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_18.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_18.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Initial conditions for atomic system:
 *    '<S97>/Embedded MATLAB Function'
 *    '<S134>/Embedded MATLAB Function'
 *    '<S158>/Embedded MATLAB Function'
 *    '<S557>/Embedded MATLAB Function'
 *    '<S653>/Embedded MATLAB Function'
 *    '<S654>/Embedded MATLAB Function'
 */
void EmbeddedMATLABFunctio_Init(rtDW_EmbeddedMATLABFunction_S_T *localDW)
{
  localDW->a_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S97>/Embedded MATLAB Function'
 *    '<S134>/Embedded MATLAB Function'
 *    '<S158>/Embedded MATLAB Function'
 *    '<S557>/Embedded MATLAB Function'
 *    '<S653>/Embedded MATLAB Function'
 *    '<S654>/Embedded MATLAB Function'
 */
void SLU_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_SL_T *localB, rtDW_EmbeddedMATLABFunction_S_T
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Inner Loop// Navigation/Lateral Channel Encaps [updated 4.28.16]/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function': '<S105>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S105>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S105>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S105>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = true;

    /* '<S105>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S105>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S105>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S105>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * (real32_T)localDW->a + (real32_T)
    localDW->b * localDW->y_km1;

  /* '<S105>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S105>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
