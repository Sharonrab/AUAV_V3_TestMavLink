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
 * File: rt_sys_AUAV_V3_TestSensors_3.h
 *
 * Real-Time Workshop code generated for Simulink model AUAV_V3_TestSensors.
 *
 * Model version                        : 1.221
 * Real-Time Workshop file version      : 8.8 (R2015a) 09-Feb-2015
 * Real-Time Workshop file generated on : Sun Oct 09 00:06:23 2016
 * TLC version                          : 8.8 (Jan 20 2015)
 * C source code generated on           : Sun Oct 09 00:06:26 2016
 */

#ifndef RTW_HEADER_rt_sys_AUAV_V3_TestSensors_3_h_
#define RTW_HEADER_rt_sys_AUAV_V3_TestSensors_3_h_
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Block signals for system '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T tOut;                         /* '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
} rtB_EnablesDisablestheComputa_T;

/* Block states (auto storage) for system '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
typedef struct {
  real_T aveCount;                     /* '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T tIni;                         /* '<S28>/Enables//Disables the Computation of  initial Baro Bias' */
} rtDW_EnablesDisablestheComput_T;

extern void EnablesDisablestheCom_Init(rtDW_EnablesDisablestheComput_T *localDW);
extern void EnablesDisablestheComputat(rtB_EnablesDisablestheComputa_T *localB,
  rtDW_EnablesDisablestheComput_T *localDW);

#endif                                 /* RTW_HEADER_rt_sys_AUAV_V3_TestSensors_3_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
