/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Barometer_Driver.h
 *
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#ifndef RTW_HEADER_Barometer_Driver_h_
#define RTW_HEADER_Barometer_Driver_h_
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "rt_sys_SLUGS2_2.h"
#include "rt_sys_SLUGS2_3.h"

extern void SLUG_Barometer_Driver_Init(void);
extern void SLU_Barometer_Driver_Start(void);
extern void SLUGS2_Barometer_Driver(void);

#endif                                 /* RTW_HEADER_Barometer_Driver_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
