#include "mex.h"
#include "dtm2020_bridge.h"

static int model_initialized = 0;

static void require_real_double(const mxArray *value, mwSize elements,
                                const char *name)
{
    if (!mxIsDouble(value) || mxIsComplex(value) ||
            mxGetNumberOfElements(value) != elements) {
        mexErrMsgIdAndTxt("AOCS:DTM2020:InvalidInput",
                          "%s must be a real double array with %llu element(s).",
                          name, (unsigned long long)elements);
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char *coefficient_file;
    const double *f;
    const double *fbar;
    const double *akp;
    double rho_g_cm3;
    double uncertainty_percent;
    double temperature_local_K;
    double temperature_exospheric_K;
    double species_g_cm3[6];
    int status;

    if (nrhs != 9 || nlhs > 5) {
        mexErrMsgIdAndTxt("AOCS:DTM2020:InvalidCall",
            "Expected [rho,sigma,T,Tinf,d] = dtm2020_mex(file,day,f,fbar,akp,alt,hl,lat,lon).");
    }
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("AOCS:DTM2020:InvalidInput",
                          "Coefficient file must be a character vector.");
    }

    require_real_double(prhs[1], 1, "day");
    require_real_double(prhs[2], 2, "f");
    require_real_double(prhs[3], 2, "fbar");
    require_real_double(prhs[4], 4, "akp");
    require_real_double(prhs[5], 1, "altitude_km");
    require_real_double(prhs[6], 1, "local_solar_time_rad");
    require_real_double(prhs[7], 1, "latitude_rad");
    require_real_double(prhs[8], 1, "longitude_rad");

    coefficient_file = mxArrayToString(prhs[0]);
    if (coefficient_file == NULL) {
        mexErrMsgIdAndTxt("AOCS:DTM2020:InvalidInput",
                          "Unable to decode coefficient file path.");
    }

    if (!model_initialized) {
        dtm2020_initialize(coefficient_file, (int)mxGetNumberOfElements(prhs[0]),
                           &status);
        if (status != 0) {
            mxFree(coefficient_file);
            mexErrMsgIdAndTxt("AOCS:DTM2020:InitializationFailed",
                              "DTM2020 initialization failed with status %d.", status);
        }
        model_initialized = 1;
    }
    mxFree(coefficient_file);

    f = mxGetDoubles(prhs[2]);
    fbar = mxGetDoubles(prhs[3]);
    akp = mxGetDoubles(prhs[4]);
    dtm2020_evaluate(mxGetScalar(prhs[1]), f[0], fbar[0], akp[0], akp[2],
                     mxGetScalar(prhs[5]), mxGetScalar(prhs[6]),
                     mxGetScalar(prhs[7]), mxGetScalar(prhs[8]),
                     &rho_g_cm3, &uncertainty_percent, &temperature_local_K,
                     &temperature_exospheric_K, species_g_cm3, &status);
    if (status != 0) {
        mexErrMsgIdAndTxt("AOCS:DTM2020:EvaluationFailed",
                          "DTM2020 evaluation failed with status %d.", status);
    }

    if (nlhs > 0) plhs[0] = mxCreateDoubleScalar(rho_g_cm3);
    if (nlhs > 1) plhs[1] = mxCreateDoubleScalar(uncertainty_percent);
    if (nlhs > 2) plhs[2] = mxCreateDoubleScalar(temperature_local_K);
    if (nlhs > 3) plhs[3] = mxCreateDoubleScalar(temperature_exospheric_K);
    if (nlhs > 4) {
        plhs[4] = mxCreateDoubleMatrix(6, 1, mxREAL);
        for (mwSize index = 0; index < 6; ++index) {
            mxGetDoubles(plhs[4])[index] = species_g_cm3[index];
        }
    }
}
