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
 * Code generated for Simulink model 'SLUGS2'.
 *
 * Model version                  : 1.271
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Tue Apr 11 15:47:53 2017
 */

#ifndef RTW_HEADER_Position_and_Attitude_Filter_h_
#define RTW_HEADER_Position_and_Attitude_Filter_h_
#include <math.h>
#ifndef SLUGS2_COMMON_INCLUDES_
# define SLUGS2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SLUGS2_COMMON_INCLUDES_ */

#include "SLUGS2_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "rt_sys_SLUGS2_18.h"
#include "rt_sys_SLUGS2_22.h"
#include "rt_sys_SLUGS2_75.h"
#include "rt_sys_SLUGS2_76.h"
#include "rt_sys_SLUGS2_77.h"
#include "rt_sys_SLUGS2_86.h"
#include "rt_nonfinite.h"

/* Block signals for system '<S478>/myMux Fun1' */
typedef struct {
  real_T y[3];                         /* '<S478>/myMux Fun1' */
} rtB_myMuxFun1_SLUGS2_T;

/* Block signals for system '<S506>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S506>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_g_T;

/* Block signals for system '<S553>/Embedded MATLAB Function1' */
typedef struct {
  real32_T y;                          /* '<S553>/Embedded MATLAB Function1' */
} rtB_EmbeddedMATLABFunction1_S_T;

/* Block states (auto storage) for system '<S553>/Embedded MATLAB Function1' */
typedef struct {
  real32_T LastU;                      /* '<S553>/Embedded MATLAB Function1' */
  real32_T TimeSinceLast;              /* '<S553>/Embedded MATLAB Function1' */
  real32_T rate;                       /* '<S553>/Embedded MATLAB Function1' */
  real32_T OldRate;                    /* '<S553>/Embedded MATLAB Function1' */
  boolean_T LastU_not_empty;           /* '<S553>/Embedded MATLAB Function1' */
} rtDW_EmbeddedMATLABFunction1__T;

extern void SLUGS2_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun1_SLUGS2_T *localB);
extern void S_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_g_T *localB);
extern void EmbeddedMATLABFunct_i_Init(rtDW_EmbeddedMATLABFunction1__T *localDW);
extern void SL_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_S_T *localB, rtDW_EmbeddedMATLABFunction1__T
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
