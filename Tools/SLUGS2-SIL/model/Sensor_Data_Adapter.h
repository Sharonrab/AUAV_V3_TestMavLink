/*
 * -------------------------------------------------------------------
 * MPLAB 16-Bit Device Blocks for Simulink v3.38.
 *
 *   Product Page:  http://www.microchip.com/SimulinkBlocks
 *           Forum: http://www.microchip.com/forums/f192.aspx
 *           Wiki:  http://microchip.wikidot.com/simulink:start
 * -------------------------------------------------------------------
 * File: Sensor_Data_Adapter.h
 *
 * Code generated for Simulink model 'AUAV_V3_TestSensors'.
 *
 * Model version                  : 1.241
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Sat Nov 05 08:28:55 2016
 */

#ifndef RTW_HEADER_Sensor_Data_Adapter_h_
#define RTW_HEADER_Sensor_Data_Adapter_h_
#include <math.h>
#ifndef AUAV_V3_TestSensors_COMMON_INCLUDES_
# define AUAV_V3_TestSensors_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* AUAV_V3_TestSensors_COMMON_INCLUDES_ */

#include "AUAV_V3_TestSensors_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Child system includes */
#include "rt_sys_AUAV_V3_TestSensors_2.h"
#include "rt_sys_AUAV_V3_TestSensors_3.h"
#include "rt_sys_AUAV_V3_TestSensors_86.h"
#include "rt_nonfinite.h"

/* Block signals for system '<S583>/Embedded MATLAB Function1' */
typedef struct {
  real_T y;                            /* '<S583>/Embedded MATLAB Function1' */
} rtB_EmbeddedMATLABFunction1_e_T;

/* Block signals for system '<S583>/myMux Fun' */
typedef struct {
  int16_T y[4];                        /* '<S583>/myMux Fun' */
} rtB_myMuxFun_AUAV_V3_TestSens_T;

/* Block signals for system '<S596>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S596>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_gp_T;

/* Block states (auto storage) for system '<S596>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S596>/Embedded MATLAB Function' */
  real_T b;                            /* '<S596>/Embedded MATLAB Function' */
  real_T y_km1;                        /* '<S596>/Embedded MATLAB Function' */
  real_T u_km1;                        /* '<S596>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S596>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_i_T;

/* Block signals for system '<S596>/myMux Fun' */
typedef struct {
  real_T y[3];                         /* '<S596>/myMux Fun' */
} rtB_myMuxFun_AUAV_V3_TestSe_i_T;

extern void EmbeddedMATLABFunction1_m(const real_T rtu_u[5],
  rtB_EmbeddedMATLABFunction1_e_T *localB);
extern void AUAV_V3_TestSenso_myMuxFun(int16_T rtu_u1, int16_T rtu_u2, int16_T
  rtu_u3, int16_T rtu_u4, rtB_myMuxFun_AUAV_V3_TestSens_T *localB);
extern void EmbeddedMATLABFunct_m_Init(rtDW_EmbeddedMATLABFunction_i_T *localDW);
extern void A_EmbeddedMATLABFunction_f(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_gp_T *localB, rtDW_EmbeddedMATLABFunction_i_T
  *localDW);
extern void AUAV_V3_TestSen_myMuxFun_i(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun_AUAV_V3_TestSe_i_T *localB);
extern void A_Sensor_Data_Adapter_Init(void);
extern void Sensor_Data_Adapter_Start(void);
extern void AUAV_V_Sensor_Data_Adapter(void);

#endif                                 /* RTW_HEADER_Sensor_Data_Adapter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
