/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Position_and_Attitude_Filter.h
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.269
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Mar 30 17:57:32 2017
 */

#ifndef RTW_HEADER_Position_and_Attitude_Filter_h_
#define RTW_HEADER_Position_and_Attitude_Filter_h_
#include <math.h>
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "rt_sys_AUAV_V3_TestSensors_18.h"
#include "rt_sys_AUAV_V3_TestSensors_22.h"
#include "rt_sys_AUAV_V3_TestSensors_75.h"
#include "rt_sys_AUAV_V3_TestSensors_76.h"
#include "rt_sys_AUAV_V3_TestSensors_77.h"
#include "rt_sys_AUAV_V3_TestSensors_86.h"
#include "rt_nonfinite.h"

/* Block signals for system '<S485>/myMux Fun1' */
typedef struct {
  real_T y[3];                         /* '<S485>/myMux Fun1' */
} rtB_myMuxFun1_AUAV_V3_TestSen_T;

/* Block signals for system '<S513>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S513>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_g_T;

/* Block signals for system '<S560>/Embedded MATLAB Function1' */
typedef struct {
  real32_T y;                          /* '<S560>/Embedded MATLAB Function1' */
} rtB_EmbeddedMATLABFunction1_A_T;

/* Block states (auto storage) for system '<S560>/Embedded MATLAB Function1' */
typedef struct {
  real32_T LastU;                      /* '<S560>/Embedded MATLAB Function1' */
  real32_T TimeSinceLast;              /* '<S560>/Embedded MATLAB Function1' */
  real32_T rate;                       /* '<S560>/Embedded MATLAB Function1' */
  real32_T OldRate;                    /* '<S560>/Embedded MATLAB Function1' */
  boolean_T LastU_not_empty;           /* '<S560>/Embedded MATLAB Function1' */
} rtDW_EmbeddedMATLABFunction1__T;

extern void AUAV_V3_TestSens_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun1_AUAV_V3_TestSen_T *localB);
extern void A_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_g_T *localB);
extern void EmbeddedMATLABFunct_c_Init(rtDW_EmbeddedMATLABFunction1__T *localDW);
extern void AU_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_A_T *localB, rtDW_EmbeddedMATLABFunction1__T
  *localDW);
extern void Position_and_Attitude_Init(void);
extern void Position_and_Attitud_Start(void);
extern void Position_and_Attitude_Filt(void);

#endif                                 /* RTW_HEADER_Position_and_Attitude_Filter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
