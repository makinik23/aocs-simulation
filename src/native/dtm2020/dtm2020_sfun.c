#define S_FUNCTION_NAME dtm2020_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "dtm2020_bridge.h"

enum { COEFFICIENT_FILE_PARAMETER = 0, PARAMETER_COUNT = 1 };
enum { DTM_INPUT_WIDTH = 9, DTM_OUTPUT_WIDTH = 10 };

static char error_message[256];

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, PARAMETER_COUNT);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetSFcnParamTunable(S, COEFFICIENT_FILE_PARAMETER, 0);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, DTM_INPUT_WIDTH);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, DTM_OUTPUT_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 0);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    const mxArray *parameter = ssGetSFcnParam(S, COEFFICIENT_FILE_PARAMETER);
    char *coefficient_file;
    int status;

    if (!mxIsChar(parameter)) {
        ssSetErrorStatus(S, "DTM2020 coefficient file parameter must be a character vector.");
        return;
    }

    coefficient_file = mxArrayToString(parameter);
    if (coefficient_file == NULL) {
        ssSetErrorStatus(S, "Unable to decode DTM2020 coefficient file path.");
        return;
    }
    dtm2020_initialize(coefficient_file, (int)mxGetNumberOfElements(parameter), &status);
    mxFree(coefficient_file);
    if (status != 0) {
        snprintf(error_message, sizeof(error_message),
                 "DTM2020 initialization failed with status %d.", status);
        ssSetErrorStatus(S, error_message);
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *input = (const real_T *)ssGetInputPortSignal(S, 0);
    real_T *output = ssGetOutputPortRealSignal(S, 0);
    double species_g_cm3[6];
    int status;
    (void)tid;

    dtm2020_evaluate(input[0], input[1], input[2], input[3], input[4],
                     input[5], input[6], input[7], input[8],
                     &output[0], &output[1], &output[2], &output[3],
                     species_g_cm3, &status);
    if (status != 0) {
        snprintf(error_message, sizeof(error_message),
                 "DTM2020 evaluation failed with status %d.", status);
        ssSetErrorStatus(S, error_message);
        return;
    }

    for (int index = 0; index < 6; ++index) {
        output[4 + index] = species_g_cm3[index];
    }
}

static void mdlTerminate(SimStruct *S)
{
    (void)S;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
