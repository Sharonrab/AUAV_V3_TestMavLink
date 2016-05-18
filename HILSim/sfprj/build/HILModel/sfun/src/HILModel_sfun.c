/* Include files */
#include "HILModel_sfun.h"
#include "c2_HILModel.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint8_T _sfEvent_;
uint32_T _HILModelMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void HILModel_initializer(void)
{
  _sfEvent_ = CALL_EVENT;
}

void HILModel_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_HILModel_method_dispatcher(SimStruct *simstructPtr, unsigned int
 chartFileNumber, int_T method, void *data)
{
  if(chartFileNumber==2) {
    c2_HILModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }
  return 0;
}
unsigned int sf_HILModel_process_check_sum_call( int nlhs, mxArray * plhs[], int
 nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"sf_get_check_sum")) return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if(nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if(!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(341317804U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2438347111U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3299975871U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1159196242U);
    }else if(!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    }else if(!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2436477407U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3132148161U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(200691780U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1303230451U);
    }else if(nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch(chartFileNumber) {
       case 2:
        {
          extern void sf_c2_HILModel_get_check_sum(mxArray *plhs[]);
          sf_c2_HILModel_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    }else if(!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2553529877U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1250385535U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3747036769U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(84901116U);
    }else {
      return 0;
    }
  } else{
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2751931402U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(362458542U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3591482983U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(89029608U);
  }
  return 1;
#else
  return 0;
#endif
}

unsigned int sf_HILModel_autoinheritance_info( int nlhs, mxArray * plhs[], int
 nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[32];
  if (nrhs<2 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"get_autoinheritance_info")) return 0;
  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch(chartFileNumber) {
     case 2:
      {
        extern mxArray *sf_c2_HILModel_get_autoinheritance_info(void);
        plhs[0] = sf_c2_HILModel_get_autoinheritance_info();
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }
  return 1;
#else
  return 0;
#endif
}
unsigned int sf_HILModel_get_eml_resolved_functions_info( int nlhs, mxArray *
 plhs[], int nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[32];
  if (nrhs<2 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"get_eml_resolved_functions_info")) return 0;
  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch(chartFileNumber) {
     case 2:
      {
        extern const mxArray
        *sf_c2_HILModel_get_eml_resolved_functions_info(void);
        plhs[0] = (mxArray *)sf_c2_HILModel_get_eml_resolved_functions_info();
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }
  return 1;
#else
  return 0;
#endif
}
void HILModel_debug_initialize(void)
{
  _HILModelMachineNumber_ =
  sf_debug_initialize_machine("HILModel","sfun",0,1,0,0,0);
  sf_debug_set_machine_event_thresholds(_HILModelMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_HILModelMachineNumber_,0);
}

void HILModel_register_exported_symbols(SimStruct* S)
{
}
