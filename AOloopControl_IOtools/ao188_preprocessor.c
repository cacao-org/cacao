/**
 * @file    ao188_preprocessor.c
 * @brief   Convert ao188 APD data into curvature + SH data
 *
 * Templated upon the RTS19 code.
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

static long NUM_APD_HOWFS = 188;
static long NUM_APD_LOWFS = 16;
static float CURVATURE_REGULARIZATION = 1.0;
static float one_sided_curv_integrator_gain = 0.01; // TODO FPS

// Local variables pointers
static char *apd_mat_name;
static long  fpi_wfsinsname;

static char *wfsoutsname;
static long fpi_wfsoutsname;

static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".wfsin",
        "Wavefront sensor input",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &apd_mat_name,
        &fpi_wfsinsname
    },
    {
        CLIARG_IMG,
        ".wfsout",
        "Wavefront sensor output",
        "wfsout",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsoutsname,
        &fpi_wfsoutsname
    }
};





// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

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

static CLICMDDATA CLIcmddata =
{
    "ao188preproc", "AO188 APD Preprocessor", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}


/*
Compute the curvature from the 188x2 APD curve-signed buffer.
NGL, assuming size is gonna be 188...
*/
static errno_t two_sided_curvature_compute(
    float *curvature,
    int16_t *apd_twosided,
    long size)
{
    float fp, fm;
    for(int k = 0; k < size; ++k)
    {
        fp = (float)apd_twosided[k];
        fm = (float)apd_twosided[k + size];
        curvature[k] = (fp - fm) / (fp + fm + CURVATURE_REGULARIZATION);
    }
    return RETURN_SUCCESS;
}

/*
One sided curvature updated each and every time
curv_sign = 0 or 1
*/
static errno_t one_sided_curvature_compute(
    float *curvature,
    int16_t *apd_onesided,
    float *apd_reference, // TODO must be updated.
    long size,
    int curv_sign)
{
    if(curv_sign == 0)
    {
        for(int k = 0; k < size; ++k)
        {
            curvature[k] = (apd_onesided[k] - apd_reference[k]) / apd_reference[k];
        }
    }
    else
    {
        for(int k = 0; k < size; ++k)
        {
            curvature[k] = (- apd_onesided[k] - apd_reference[k]) / apd_reference[k];
        }
    }
    return RETURN_SUCCESS;
}

static errno_t apd_integrator_update(
    float *apd_integrator,
    int16_t *apd_onesided,
    float integ_gain,
    long size
)
{
    for(int k = 0; k < size; ++k)
    {
        apd_integrator[k] *= 1.0 -
                             integ_gain; // TODO confcheck that gain < 1.0, maybe enfore much lower.
        apd_integrator[k] += integ_gain * apd_onesided[k];
    }
    return RETURN_SUCCESS;
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID apd_mat_in = stream_connect(apd_mat_name);
    float apd_integrator[NUM_APD_HOWFS];
    memset(apd_integrator, 0, NUM_APD_HOWFS * sizeof(float));

    int kw_idx_CURVSGN = 0; // TODO

    // Create curvature outputs
    uint32_t size_curvature = NUM_APD_HOWFS;
    IMGID curv_1k_doublesided = stream_connect_create_2D("curv_1kdouble",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);
    IMGID curv_2k_doublesided = stream_connect_create_2D("curv_2kdouble",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);
    IMGID curv_2k_singlesided = stream_connect_create_2D("curv_2ksingle",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);


    // TODO Does procinfo need to be marked that apd_mat_in is the triggersname?

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        int curv_sign = apd_mat_in.im->kw[kw_idx_CURVSGN].value.numl;

        // TODO these 3 can be re-ordered to optimize latency for the mode being used.
        // TODO pass keywords through. Or don't?

        curv_2k_doublesided.im->md->write = 1;
        two_sided_curvature_compute(curv_2k_doublesided.im->array.F, apd_mat_in.im->array.SI16, NUM_APD_HOWFS);
        processinfo_update_output_stream(processinfo, curv_2k_doublesided.ID);

        // Post outputs
        if(curv_sign == 1)
        {
            curv_1k_doublesided.im->md->write = 1;
            memcpy(curv_1k_doublesided.im->array.F, curv_2k_doublesided.im->array.F,
                   NUM_APD_HOWFS);
            processinfo_update_output_stream(processinfo, curv_1k_doublesided.ID);
        }

        curv_2k_singlesided.im->md->write = 1;
        // Get the latest side of the APD 216x2 buffer. WARNING: Size may be 217 if the curvature tag is embedded!
        // apd_mat_in.size[0] = 216 or 217 =/= NUM_APD_HOWFS.
        uint16_t *apd_ptr = apd_mat_in.im->array.SI16 + curv_sign * apd_mat_in.size[0];

        apd_integrator_update(apd_integrator, apd_ptr, NUM_APD_HOWFS, one_sided_curv_integrator_gain);
        one_sided_curvature_compute(curv_2k_singlesided.im->array.F, apd_ptr, apd_integrator, NUM_APD_HOWFS, curv_sign);

        processinfo_update_output_stream(processinfo, curv_2k_singlesided.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    // We should probably clean close the SHMs?
    // Why doesn't anything in milk ever call ImageStreamIO_closeIm?



    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__AO188Preproc()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
