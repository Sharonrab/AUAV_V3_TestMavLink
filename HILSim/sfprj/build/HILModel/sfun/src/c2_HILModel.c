/* Include files */
#include "HILModel_sfun.h"
#include "c2_HILModel.h"
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "HILModel_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c2_IN_NO_ACTIVE_CHILD           (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc2_HILModelInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c2_HILModel(void);
static void initialize_params_c2_HILModel(void);
static void enable_c2_HILModel(void);
static void disable_c2_HILModel(void);
static void finalize_c2_HILModel(void);
static void sf_c2_HILModel(void);
static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u);
static real_T *c2_cog(void);
static real_T *c2_Vn(void);
static real_T *c2_Ve(void);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c2_HILModel(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c2_is_active_c2_HILModel = 0U;
}

static void initialize_params_c2_HILModel(void)
{
}

static void enable_c2_HILModel(void)
{
}

static void disable_c2_HILModel(void)
{
}

static void finalize_c2_HILModel(void)
{
}

static void sf_c2_HILModel(void)
{
  uint8_T c2_previousEvent;
  real_T c2_b_Vn;
  real_T c2_b_Ve;
  real_T c2_b_cog;
  real_T c2_Y;
  real_T c2_X;
  real_T c2_yk;
  real_T c2_xk;
  real_T c2_y;
  real_T c2_x;
  real_T c2_b_x;
  boolean_T c2_b;
  real_T c2_c_x;
  boolean_T c2_b_b;
  real_T c2_d_x;
  boolean_T c2_c_b;
  real_T c2_e_x;
  boolean_T c2_d_b;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_b_xk;
  real_T c2_h_x;
  boolean_T c2_e_b;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_c_xk;
  real_T c2_k_x;
  boolean_T c2_f_b;
  real_T c2_l_x;
  real_T c2_m_x;
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_DATA_RANGE_CHECK(*c2_cog(), 1U);
  _SFD_DATA_RANGE_CHECK(*c2_Vn(), 2U);
  _SFD_DATA_RANGE_CHECK(*c2_Ve(), 0U);
  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  c2_b_Vn = *c2_Vn();
  c2_b_Ve = *c2_Ve();
  sf_debug_symbol_scope_push(3U, 0U);
  sf_debug_symbol_scope_add("cog", &c2_b_cog, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("Ve", &c2_b_Ve, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("Vn", &c2_b_Vn, c2_sf_marshall);
  CV_EML_FCN(0, 0);
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  _SFD_EML_CALL(STATE_DURING_DURING_ACTION_TAG,0,5);
  c2_Y = c2_b_Ve;
  c2_X = c2_b_Vn;
  c2_b_cog = 0.0;
  c2_yk = c2_Y;
  c2_xk = c2_X;
  c2_y = c2_yk;
  c2_x = c2_xk;
  c2_b_x = c2_x;
  c2_b = rtIsNaN(c2_b_x);
  if(c2_b) {
    goto label_1;
  } else {
    c2_c_x = c2_y;
    c2_b_b = rtIsNaN(c2_c_x);
    if(c2_b_b) {
      goto label_1;
    } else {
      c2_d_x = c2_y;
      c2_c_b = rtIsInf(c2_d_x);
      if(c2_c_b) {
        c2_e_x = c2_x;
        c2_d_b = rtIsInf(c2_e_x);
        if(c2_d_b) {
          c2_f_x = c2_x;
          c2_g_x = c2_f_x;
          c2_b_xk = c2_g_x;
          c2_h_x = c2_b_xk;
          c2_e_b = rtIsNaN(c2_h_x);
          if(c2_e_b) {
            c2_g_x = rtNaN;
          } else if(c2_b_xk > 0.0) {
            c2_g_x = 1.0;
          } else if(c2_b_xk < 0.0) {
            c2_g_x = -1.0;
          }
          c2_i_x = c2_y;
          c2_j_x = c2_i_x;
          c2_c_xk = c2_j_x;
          c2_k_x = c2_c_xk;
          c2_f_b = rtIsNaN(c2_k_x);
          if(c2_f_b) {
            c2_j_x = rtNaN;
          } else if(c2_c_xk > 0.0) {
            c2_j_x = 1.0;
          } else if(c2_c_xk < 0.0) {
            c2_j_x = -1.0;
          }
          c2_l_x = atan2(c2_j_x, c2_g_x);
          c2_b_cog = c2_l_x;
          goto label_2;
        }
      }
    }
  }
  if(c2_x == 0.0) {
    if(c2_y > 0.0) {
      c2_b_cog = 1.5707963267948966E+000;
    } else if(c2_y < 0.0) {
      c2_b_cog = -1.5707963267948966E+000;
    } else {
      c2_b_cog = 0.0;
    }
  } else {
    c2_m_x = atan2(c2_y, c2_x);
    c2_b_cog = c2_m_x;
  }
  goto label_2;
  label_1:;
  c2_b_cog = rtNaN;
  label_2:;
  _SFD_EML_CALL(STATE_DURING_DURING_ACTION_TAG,0,6);
  if(CV_EML_IF(0, 0, c2_b_cog < 0.0)) {
    _SFD_EML_CALL(STATE_DURING_DURING_ACTION_TAG,0,7);
    c2_b_cog = c2_b_cog + 6.2831853071795862E+000;
  }
  _SFD_EML_CALL(STATE_DURING_DURING_ACTION_TAG,0,-7);
  sf_debug_symbol_scope_pop();
  *c2_cog() = c2_b_cog;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_HILModelMachineNumber_,
   chartInstance.chartNumber, chartInstance.instanceNumber);
}

static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *(real_T *)c2_u;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create(&c2_b_u, "y", 0, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *(real_T *)c2_u;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create(&c2_b_u, "y", 0, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *(real_T *)c2_u;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create(&c2_b_u, "y", 0, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_HILModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create(NULL, "nameCaptureInfo", 0,
    0U, 1U, 2, 0, 1));
  return c2_nameCaptureInfo;
}

static real_T *c2_cog(void)
{
  return (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
}

static real_T *c2_Vn(void)
{
  return (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
}

static real_T *c2_Ve(void)
{
  return (real_T *)ssGetInputPortSignal(chartInstance.S, 1);
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c2_HILModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4148415380U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1004006559U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2294556768U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2651083685U);
}

mxArray *sf_c2_HILModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] =
  {"checksum","inputs","parameters","outputs"};
  mxArray *mxAutoinheritanceInfo =
  mxCreateStructMatrix(1,1,4,autoinheritanceFields);
  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2494184099U);
    pr[1] = (double)(3062960115U);
    pr[2] = (double)(1048967506U);
    pr[3] = (double)(3957619554U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }
  {
    const char *dataFields[] = {"size","type","complexity"};
    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);
    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }
    {
      const char *typeFields[] = {"base","aliasId","fixpt"};
      mxArray *mxType = mxCreateStructMatrix(1,1,3,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"aliasId",mxCreateDoubleScalar(0));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }
    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }
    {
      const char *typeFields[] = {"base","aliasId","fixpt"};
      mxArray *mxType = mxCreateStructMatrix(1,1,3,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"aliasId",mxCreateDoubleScalar(0));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }
    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }
  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,mxREAL));
  }
  {
    const char *dataFields[] = {"size","type","complexity"};
    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);
    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }
    {
      const char *typeFields[] = {"base","aliasId","fixpt"};
      mxArray *mxType = mxCreateStructMatrix(1,1,3,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"aliasId",mxCreateDoubleScalar(0));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }
    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }
  return(mxAutoinheritanceInfo);
}

static void chart_debug_initialization(SimStruct *S, unsigned int
 fullDebuggerInitialization)
{
  if(ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
    /* do this only if simulation is starting */
    if(!sim_mode_is_rtw_gen(S)) {
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_HILModelMachineNumber_,
         2,
         1,
         1,
         3,
         0,
         0,
         0,
         0,
         &(chartInstance.chartNumber),
         &(chartInstance.instanceNumber),
         ssGetPath(S),
         (void *)S);
        if(chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting(_HILModelMachineNumber_,chartInstance.chartNumber,1);
          sf_debug_set_chart_event_thresholds(_HILModelMachineNumber_,
           chartInstance.chartNumber,
           0,
           0,
           0);

          _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"cog",0,c2_c_sf_marshall);
          _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Vn",0,c2_sf_marshall);
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Ve",0,c2_b_sf_marshall);
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }
        _SFD_CV_INIT_CHART(0,0,0,0);
        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of EML Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,191);
        _SFD_CV_INIT_EML_IF(0,0,152,162,-1,189);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if(chartAlreadyPresent==0)
        {
          _SFD_TRANS_COV_MAPS(0,
           0,NULL,NULL,
           0,NULL,NULL,
           1,NULL,NULL,
           0,NULL,NULL);
        }
        _SFD_SET_DATA_VALUE_PTR(1U, c2_cog());
        _SFD_SET_DATA_VALUE_PTR(2U, c2_Vn());
        _SFD_SET_DATA_VALUE_PTR(0U, c2_Ve());
      }
    }
  } else {
    sf_debug_reset_current_state_configuration(_HILModelMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
  }
}

static void sf_opaque_initialize_c2_HILModel(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c2_HILModel();
  initialize_c2_HILModel();
}

static void sf_opaque_enable_c2_HILModel(void *chartInstanceVar)
{
  enable_c2_HILModel();
}

static void sf_opaque_disable_c2_HILModel(void *chartInstanceVar)
{
  disable_c2_HILModel();
}

static void sf_opaque_gateway_c2_HILModel(void *chartInstanceVar)
{
  sf_c2_HILModel();
}

static void sf_opaque_terminate_c2_HILModel(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }
  finalize_c2_HILModel();
}

static void mdlProcessParameters_c2_HILModel(SimStruct *S)
{
  int i;
  for(i=0;i<ssGetNumRunTimeParams(S);i++) {
    if(ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }
  initialize_params_c2_HILModel();
}

static void mdlSetWorkWidths_c2_HILModel(SimStruct *S)
{
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("HILModel",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("HILModel",2,"gatewayCannotBeInlinedMultipleTimes"));
    if(chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"HILModel",2,2);
      sf_mark_chart_reusable_outputs(S,"HILModel",2,1);
    }
    if (!sf_is_chart_instance_optimized_out("HILModel",2)) {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("HILModel",2);
      dtId = ssRegisterDataType(S, chartInstanceTypedefName);
      if (dtId == INVALID_DTYPE_ID ) return;
      /* Register the size of the udt */
      if (!ssSetDataTypeSize(S, dtId, 8)) return;
      if(!ssSetNumDWork(S,1)) return;
      ssSetDWorkDataType(S, 0, dtId);
      ssSetDWorkWidth(S, 0, 1);
      ssSetDWorkName(S, 0, "ChartInstance"); /*optional name, less than 16 chars*/
      sf_set_rtw_identifier(S);
    }
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(2060483662U));
  ssSetChecksum1(S,(3476886716U));
  ssSetChecksum2(S,(591953927U));
  ssSetChecksum3(S,(908934155U));

  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_HILModel(SimStruct *S)
{
  if(sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "HILModel", 2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_HILModel(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_opaque_gateway_c2_HILModel;
  chartInstance.chartInfo.initializeChart = sf_opaque_initialize_c2_HILModel;
  chartInstance.chartInfo.terminateChart = sf_opaque_terminate_c2_HILModel;
  chartInstance.chartInfo.enableChart = sf_opaque_enable_c2_HILModel;
  chartInstance.chartInfo.disableChart = sf_opaque_disable_c2_HILModel;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c2_HILModel;
  chartInstance.chartInfo.mdlStart = mdlStart_c2_HILModel;
  chartInstance.chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_HILModel;
  chartInstance.chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.storeCurrentConfiguration = NULL;
  chartInstance.S = S;
  ssSetUserData(S,(void *)(&(chartInstance.chartInfo))); /* register the chart instance with simstruct */

  if(!sim_mode_is_rtw_gen(S)) {
    init_dsm_address_info();
  }
  chart_debug_initialization(S,1);
}

void c2_HILModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_HILModel(S);
    break;
   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_HILModel(S);
    break;
   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_HILModel(S);
    break;
   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
     "Error calling c2_HILModel_method_dispatcher.\n"
     "Can't handle method %d.\n", method);
    break;
  }
}

