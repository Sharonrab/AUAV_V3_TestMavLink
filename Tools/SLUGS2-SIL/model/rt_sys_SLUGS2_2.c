/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_SLUGS2_2.c
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#include "rt_sys_SLUGS2_2.h"

/* Include model header file for global data */
#include "SLUGS2.h"
#include "SLUGS2_private.h"

/*
 * Start for enable system:
 *    '<S24>/Enabled Subsystem'
 *    '<S614>/Enabled Subsystem'
 */
void SLU_EnabledSubsystem_Start(rtB_EnabledSubsystem_SLUGS2_T *localB)
{
  /* VirtualOutportStart for Outport: '<S34>/Out1' */
  localB->In1 = 166.3F;
}

/*
 * Output and update for enable system:
 *    '<S24>/Enabled Subsystem'
 *    '<S614>/Enabled Subsystem'
 */
void SLUGS2_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_SLUGS2_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S24>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S34>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S34>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S24>/Enabled Subsystem' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
