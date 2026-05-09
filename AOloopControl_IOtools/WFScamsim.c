/**
 * @file    WFScamsim.c
 * @brief   camera simulation for WFS
 *
 */
#include "ImageStreamIO/ImageStruct.h"

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "statistic/statistic.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "WFScamsim",
    .cmdkey      = "WFScamsim",
    .description = "simulate WFS camera",
    .description_long =
        "Simulate a wavefront sensor camera by applying photon noise, readout noise, and detector response to an ideal WFS image."
};

// Local variables
static char wfssignal_in[FUNCTION_PARAMETER_STRMAXLEN];
static char wfsim_out[FUNCTION_PARAMETER_STRMAXLEN];
static uint64_t compdarkadd;
static char wfsdark[FUNCTION_PARAMETER_STRMAXLEN];
static float fluxtotal;
static float camgain;
static uint64_t compphnoise;
static float camRON;

#define FPS_PARAMS(X) \
    X(".wfssignal", wfssignal_in, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "Wavefront sensor input signal") \
    X(".wfscamim", wfsim_out, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "Wavefront sensor ouput image") \
    X(".compdarkadd", &compdarkadd, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "subtract dark") \
    X(".camdark", wfsdark, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "camera dark frame") \
    X(".fluxtotal", &fluxtotal, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_OUTPUT, "total output flux [phe-], <0 if no scaling") \
    X(".camgain", &camgain, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_OUTPUT, "camera gain [e- / ADU]") \
    X(".compphnoise", &compphnoise, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "compute photon noise") \
    X(".camRON", &camRON, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_OUTPUT, "camera readout noise [e-] (neg = 0)")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        long fpi_compdarkadd = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".compdarkadd");
        long fpi_fluxtotal   = functionparameter_GetParamIndex(data.core.fpsptr, ".fluxtotal");
        long fpi_camgain     = functionparameter_GetParamIndex(data.core.fpsptr, ".camgain");
        long fpi_compphnoise = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".compphnoise");
        long fpi_camRON      = functionparameter_GetParamIndex(data.core.fpsptr, ".camRON");

        if(fpi_compdarkadd > -1)
            data.core.fpsptr
                ->parray[fpi_compdarkadd]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_fluxtotal > -1)
            data.core.fpsptr
                ->parray[fpi_fluxtotal]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_camgain > -1)
            data.core.fpsptr
                ->parray[fpi_camgain]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_compphnoise > -1)
            data.core.fpsptr
                ->parray[fpi_compphnoise]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_camRON > -1)
            data.core.fpsptr
                ->parray[fpi_camRON]
                .fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    IMGID wfssignalimg = imgid_make_from_name(wfssignal_in);
    resolveIMGID(
        &wfssignalimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (wfssignalimg.ID == -1) return RETURN_FAILURE;

    uint32_t sizexWFS = wfssignalimg.md->size[0];
    uint32_t sizeyWFS = wfssignalimg.md->size[1];

    uint64_t sizeWFS = (uint64_t) sizexWFS;
    sizeWFS *= sizeyWFS;

    IMGID wfsdarkimg = imgid_make_from_name(wfsdark);

    resolveIMGID(
        &wfsdarkimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);

    IMGID imcamtmpimg = imgid_make_from_name_2D("imcamtmp", sizexWFS, sizeyWFS);
    createimagefromIMGID(&imcamtmpimg);

    // Create output
    //
    IMGID wfsoutimg;
    {
        wfsoutimg =
            stream_connect_create_2D(wfsim_out,
                sizexWFS,
                sizeyWFS,
                _DATATYPE_UINT16);
    }


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        // scale flux
        // ensure there is no negative value
        //
        if(fluxtotal < 0.0)
        {
            // do not scale
            memcpy(imcamtmpimg.im->array.F, wfssignalimg.im->array.F,
                   sizeof(float)* sizexWFS);
        }
        else
        {
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                if(wfssignalimg.im->array.F[ii] > 0.0)
                {
                    imcamtmpimg.im->array.F[ii] = fluxtotal * wfssignalimg.im->array.F[ii];
                }
                else
                {
                    imcamtmpimg.im->array.F[ii] = 0.0;
                }
            }
        }

        // add photon noise
        //
        if(fluxtotal >= 0.0)
        {
            long fpi_compphnoise = 
                functionparameter_GetParamIndex(
                    data.core.fpsptr, ".compphnoise");
            if(fpi_compphnoise > -1 && (data.core.fpsptr->parray[fpi_compphnoise].fpflag & FPFLAG_ONOFF))
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] = poisson(imcamtmpimg.im->array.F[ii]);
                }
            }
        }

        // add readout noise
        //
        if(fluxtotal >= 0.0)
        {
            if(camRON > 0.0)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] += camRON * gauss();
                }
            }
        }

        // convert to ADU
        //
        if(fluxtotal >= 0.0)
        {
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                imcamtmpimg.im->array.F[ii] /= camgain;
            }
        }


        // add dark
        //
        if(fluxtotal >= 0.0)
        {
            if(wfsdarkimg.ID != -1)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] += wfsdarkimg.im->array.F[ii];
                }
            }
        }

        // write to output camera
        //
        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {

            float tmpval = imcamtmpimg.im->array.F[ii];
            if(tmpval < 0.0)
            {
                wfsoutimg.im->array.UI16[ii] = 0;
            }
            else if(tmpval > 65535)
            {
                wfsoutimg.im->array.UI16[ii] = 65535;
            }
            else
            {
                wfsoutimg.im->array.UI16[ii] = (uint16_t) tmpval;
            }
        }

        DEBUG_TRACEPOINT(" ");

        struct timespec ts;
        if(clock_gettime(CLOCK_ISIO, &ts) == -1)
        {
            perror("clock_gettime");
            exit(EXIT_FAILURE);
        }
        wfsoutimg.im->md->atime = ts;

        processinfo_update_output_stream(processinfo, wfsoutimg.im, NULL);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&wfssignalimg);
    imgid_free(&wfsdarkimg);
    imgid_free(&imcamtmpimg);
    imgid_free(&wfsoutimg);

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
CLIADDCMD_AOloopControl_IOtools__WFScamsim()
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
