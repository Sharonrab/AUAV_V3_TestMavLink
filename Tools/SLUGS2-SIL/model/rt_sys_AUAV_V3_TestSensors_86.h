/*--------------------------------------------------------------
 *   MPLAB Blockset v3.35 for Microchip dsPIC chip family.     *
 *   Generate .c and .h files from Simulink model              *
 *   and compile to .elf, .hex and .cof file that can be       *
 *   flashed into the microcontroller                          *
 *                                                             *
 *      The Microchip name PIC, dsPIC, and MPLAB are           *
 *      registered trademarks of Microchip Technology Inc.     *
 *      MATLAB, Simulink, and Real-Time Workshop are           *
 *      registered trademarks of The MathWorks, Inc.           *
 *                                                             *
 *  Blockset authors: L.Kerhuel, U.Kumar                       *
 *  Product Page:  http://www.microchip.com/SimulinkBlocks     *
 *          Forum: http://www.microchip.com/forums/f192.aspx   *
 *          Wiki:  http://microchip.wikidot.com/simulink:start *
 *--------------------------------------------------------------
 *
 * File: rt_sys_AUAV_V3_TestSensors_86.h
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#ifndef RTW_HEADER_rt_sys_AUAV_V3_TestSensors_86_h_
#define RTW_HEADER_rt_sys_AUAV_V3_TestSensors_86_h_
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S552>/myMux Fun1' */
typedef struct {
  real32_T y[3];                       /* '<S552>/myMux Fun1' */
} rtB_myMuxFun1_AUAV_V3_TestS_d_T;

extern void AUAV_V3_TestSe_myMuxFun1_e(real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, rtB_myMuxFun1_AUAV_V3_TestS_d_T *localB);

#endif                                 /* RTW_HEADER_rt_sys_AUAV_V3_TestSensors_86_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */