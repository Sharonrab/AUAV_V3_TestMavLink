/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: rt_sys_AUAV_V3_TestSensors_2.c
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.241
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Nov 05 08:28:55 2016
 */

#include "rt_sys_AUAV_V3_TestSensors_2.h"

/* Include model header file for global data */
#include "AUAV_V3_TestSensors.h"
#include "AUAV_V3_TestSensors_private.h"

/*
 * Output and update for enable system:
 *    '<S28>/Enabled Subsystem'
 *    '<S622>/Enabled Subsystem'
 */
void AUAV_V3_T_EnabledSubsystem(boolean_T rtu_Enable, real32_T rtu_In1,
  rtB_EnabledSubsystem_AUAV_V3__T *localB)
{
  /* Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S38>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S38>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S28>/Enabled Subsystem' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
