/**
 * @file    acquireWFS.c
 * @brief   acquire and preprocess WFS image
 *
 */



#include "CommandLineInterface/CLIcore.h"


// Local variables pointers
static uint32_t *loop;
long fpi_loop;

static uint32_t *semindex;
long fpi_semindex;


static float *fluxtotal;
long fpi_fluxtotal;

static float *GPUalpha;
long fpi_GPUalpha;

static float *GPUbeta;
long fpi_GPUbeta;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32, ".loop", "loop index", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &loop, &fpi_loop
    },
    {
        CLIARG_UINT32, ".semindex", "input semaphore index", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &semindex, &fpi_semindex
    },
    {
        CLIARG_FLOAT32, ".out.fluxtotal", "total flux", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &fluxtotal, &fpi_fluxtotal
    },
    {
        CLIARG_FLOAT32, ".out.GPUalpha", "GPU alpha coefficient", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &GPUalpha, &fpi_GPUalpha
    },
    {
        CLIARG_FLOAT32, ".out.GPUbeta", "GPU beta coefficient", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &GPUbeta, &fpi_GPUbeta
    }
};






// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
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
    "acquireWFS",
    "acquire WFS image",
    CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();



    INSERT_STD_PROCINFO_COMPUTEFUNC_START


    printf(">>>>>>>>>>>>>>>>>>>> acquire WFS iteration done\n");

    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__acquireWFS()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}





