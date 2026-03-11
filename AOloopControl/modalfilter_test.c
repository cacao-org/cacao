#include "ImageStreamIO/ImageStruct.h"
/**
 * @file    modalfilter_test.c
 * @brief   simulate sequence for modal filter test
 *
 * Creates a multi-dim stream sequence to feed to mfilt
 * Noise is added at each step, and attenuation is applied
 * (mutlfactor) to keep the sequence from diverging
 *
 * Connects to:
 * [mvalWFS] aolX_mfiltt_mvalWFS (can be sym link to aolX_modevalWFS)
 * [mvalDM]  aolX_mfiltt_mvalDM  (can by sym link to aolX_modevalDM)
 *
 * Internal arrays
 * [mvalIN]                : input disturbance
 * [mvalDMd]               : time-delayed input DM correction
 * [mvalC]                 : Corrected: mvalIN-mvalDMd
 *
 * outmval [user-set name] : output
 *
 * Main steps:
 * - Compute mvalIN
 * - Delay the input correction: mvalDM  (DMdelay)> mvalDMd
 * - Apply correction: mvalIN+mvalDMd -> mvalC
 * - Apply time delay, wite ouput: mvalC (WFSdelay)-> outmval
 */

#include <math.h>

#include "CLIcore/CLIcore.h"

// for random noise
#include "statistic/statistic.h"



static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "mfilttest",
    .cmdkey      = "mfilttest",
    .description = "test input for modal filter"
};

#define SNAMEPREFIX "tseqPF"

static uint64_t AOloopindex = 0;
static char mvalDM[
    FUNCTION_PARAMETER_STRMAXLEN];
static char mvalWFS[
    FUNCTION_PARAMETER_STRMAXLEN];
static float minPrate = 0;
static float maxPrate = 0;
static float noiseamp = 0;
static float multfact = 0;
static float WFSlatency = 0;
static float DMlatency = 0;

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, \
      FPTYPE_UINT64, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "AO loop index") \
    X(".mvalDM", mvalDM, \
      FPTYPE_STREAMNAME, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input mode values - DM control") \
    X(".mvalWFS", mvalWFS, \
      FPTYPE_STREAMNAME, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output mode values to WFS") \
    X(".minPrate", &minPrate, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "min phase rate [rad/tstep]") \
    X(".maxPrate", &maxPrate, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "max phase rate [rad/tstep]") \
    X(".noiseamp", &noiseamp, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "noise amplitude") \
    X(".multfact", &multfact, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "multiplicative factor") \
    X(".WFSlatency", &WFSlatency, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "WFS latency [frame]") \
    X(".DMlatency", &DMlatency, \
      FPTYPE_FLOAT32, 1, \
      FPFLAG_DEFAULT_INPUT, "DM latency [frame]")

static FPS_CLI_BINDING my_bindings[] = {
    FPS_PARAMS(FPS_X_BINDING)
};

static const int nb_bindings = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};


#ifdef FPS_STANDALONE
CLICMDDATA CLIcmddata = {
#else
static CLICMDDATA CLIcmddata = {
#endif
    "",
    "",
    CLICMD_FIELDS_DEFAULTS
};

static CMDSETTINGS default_cmdsettings = {0};

static __attribute__((constructor))
void init_cmdsettings(void)
{
    strncpy(CLIcmddata.key,
            FPS_app_info.cmdkey,
            sizeof(CLIcmddata.key) - 1);
    strncpy(CLIcmddata.description,
            FPS_app_info.description,
            sizeof(CLIcmddata.description) - 1);
    if (CLIcmddata.cmdsettings == NULL) {
        CLIcmddata.cmdsettings =
            &default_cmdsettings;
    }
}

static errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        long fpi;
        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".mvalDM");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".mvalWFS");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".minPrate");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".multfact");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".maxPrate");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".noiseamp");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".DMlatency");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".WFSlatency");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;
    }
    return RETURN_SUCCESS;
}

static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    // Connect to mvalDM
    // connect to input mode values array and get number of modes
    //
    IMGID imgmvalDM = imgid_make_from_name(mvalDM);
    resolveIMGID(&imgmvalDM, ERRMODE_ABORT, data.core.image, data.core.NB_MAX_IMAGE);
    printf("%u modes\n", imgmvalDM.md->size[0]);
    uint32_t NBmode = imgmvalDM.md->size[0];

    // Connect to mvalWFS
    //
    IMGID imgmvalWFS = imgid_make_from_name(mvalWFS);
    resolveIMGID(&imgmvalWFS, ERRMODE_ABORT, data.core.image, data.core.NB_MAX_IMAGE);



    // connect / create mvalC
    //IMGID imgmvalC = stream_connect_create_2Df32(mvalCname, *NBmode, 1);

    // connect / create mvalout
    //IMGID imgmvalout = stream_connect_create_2Df32(mvaloutname, *NBmode, 1);







    // create input buffer holding recent input values to apply delays
    //
    uint32_t NBdelaystep = 50;

    // mvalDM buffer
    uint32_t mvalDMbuff_tindex = 0;
    IMGID imgmvalDMbuff = imgid_make_from_name_2D("mvalDMbuff", NBmode, NBdelaystep);
    createimagefromIMGID(&imgmvalDMbuff);

    // mvalOUT buffer
    uint32_t mvalCbuff_tindex = 0;
    IMGID imgmvalCbuff = imgid_make_from_name_2D("mvalCbuff", NBmode, NBdelaystep);
    createimagefromIMGID(&imgmvalCbuff);



    list_image_ID();




    float *mvalIN = (float*) malloc(sizeof(float)*NBmode);
    float *mvalINpha = (float*) malloc(sizeof(float)*NBmode);

    // time-delayed DM correction
    float *mvalDMd = (float*) malloc(sizeof(float)*NBmode);

    // Corrected input
    float *mvalC = (float*) malloc(sizeof(float)*NBmode);

    // Time-delayed corrected input
    float *mvalCd = (float*) malloc(sizeof(float)*NBmode);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START


    // Write input disturbance
    //
    for(uint32_t mi=0; mi < NBmode; mi++)
    {
        float phastep = (minPrate) + (1.0*mi / NBmode) * ((maxPrate) - (minPrate));
        mvalINpha[mi] += phastep;

        mvalIN[mi] = cos(mvalINpha[mi]);

        // add noise
        mvalIN[mi] += (noiseamp) * (1.0-2.0*ran1());
        // mult
        mvalIN[mi] *= (multfact);
    }

    //printf("mi0:  %8.6f\n", mvalIN[0]);


    {
        // Grab new input mvalDM
        char *ptr = (char *) imgmvalDMbuff.im->array.F;
        ptr += SIZEOF_DATATYPE_FLOAT*NBmode*mvalDMbuff_tindex;
        memcpy( ptr, imgmvalDM.im->array.F, sizeof(float)*NBmode);

        int latint = floor(DMlatency);  // integer part
        float latfrac = (DMlatency) - latint;  // fractional part
        int index0 = mvalDMbuff_tindex - latint;
        if(index0 < 0)
        {
            index0 += NBdelaystep;
        }
        int index1 = index0 - 1;
        if(index1 < 0)
        {
            index1 += NBdelaystep;
        }

        for(uint32_t mi=0; mi < NBmode; mi++)
        {
            mvalDMd[mi] = (1.0-latfrac) * imgmvalDMbuff.im->array.F[index0*NBmode+mi];
            mvalDMd[mi] += latfrac * imgmvalDMbuff.im->array.F[index1*NBmode+mi];
        }

        {
            uint32_t mi = 5;
            printf("latency DM index  [%3d %3d  %+9.6f  %+9.6f -> %+9.6f   %+9.6f\n",
                   index0, index1,
                   imgmvalDMbuff.im->array.F[index0*NBmode+mi],
                   imgmvalDMbuff.im->array.F[index1*NBmode+mi],
                   mvalDMd[mi],
                   imgmvalDM.im->array.F[mi]);
        }

    }
    // apply time-delayed DM correction
    for(uint32_t mi=0; mi < NBmode; mi++)
    {
        mvalC[mi] = mvalIN[mi] + mvalDMd[mi];
    }
    // update DM buffer index
    mvalDMbuff_tindex ++;
    if(mvalDMbuff_tindex == NBdelaystep)
    {
        mvalDMbuff_tindex = 0;
    }



    {
        // Grab new input mvalC
        char *ptr = (char *) imgmvalCbuff.im->array.F;
        ptr += SIZEOF_DATATYPE_FLOAT*NBmode*mvalCbuff_tindex;
        memcpy( ptr, mvalC, sizeof(float)*NBmode);

        int latint = floor(WFSlatency);  // integer part
        float latfrac = (WFSlatency) - latint;  // fractional part
        int index0 = mvalCbuff_tindex - latint;
        if(index0 < 0)
        {
            index0 += NBdelaystep;
        }
        int index1 = mvalCbuff_tindex - latint;
        if(index1 < 0)
        {
            index1 += NBdelaystep;
        }

        for(uint32_t mi=0; mi < NBmode; mi++)
        {
            mvalCd[mi] = (1.0-latfrac) * imgmvalCbuff.im->array.F[index0*NBmode+mi];
            mvalCd[mi] += latfrac * imgmvalCbuff.im->array.F[index1*NBmode+mi];
        }
    }
    // update C buffer index
    mvalCbuff_tindex ++;
    if(mvalCbuff_tindex == NBdelaystep)
    {
        mvalCbuff_tindex = 0;
    }


    memcpy(imgmvalWFS.im->array.F, mvalCd, sizeof(float)*NBmode);
    processinfo_update_output_stream(processinfo, imgmvalWFS.im, NULL);

    /*
        {
            int mimax = NBmode;
            if(mimax > 5)
            {
                mimax = 5;
            }
            for(int mi=0; mi < mimax; mi++)
            {
                printf("%10.6f  ", mvalIN[mi]);
            }
            printf("\n");
        }
    */


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalC);
    free(mvalCd);
    free(mvalDMd);
    free(mvalIN);
    free(mvalINpha);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__modalfilter_test()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(
    FPS_app_info,
    FPS_PARAMS,
    compute_function,
    customCONFsetup,
    customCONFcheck)
#endif
