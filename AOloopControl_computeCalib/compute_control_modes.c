/**
 * @file compute_control_modes.c
 * @brief Compute AO control modes in both input (WFS) and output (DM) space
 *
 */

#include "CommandLineInterface/CLIcore.h"


// Local variables pointers



static float  *svdlim0;
static double *svdlim1;

static int32_t  *i32;
static int64_t  *i64;
static uint32_t *ui32;
static uint64_t *ui64;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_FLOAT32, ".svdlim0", "SVD limit 0", "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &svdlim0, NULL
    },
    {
        CLIARG_FLOAT64, ".svdlim1", "SVD limit 1", "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &svdlim1, NULL
    },
    {
        CLIARG_INT32, ".int32", "int 32", "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &i32, NULL
    },
    {
        CLIARG_INT64, ".int64", "int 64", "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &i64, NULL
    },
    {
        CLIARG_UINT32, ".uint32", "uint 32", "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &ui32, NULL
    },
    {
        CLIARG_UINT64, ".uint64", "uint 64", "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &ui64, NULL
    }
};



// Optional custom configuration setup
// Runs once at conf startup
//
// To use this function, set :
// CLIcmddata.FPS_customCONFsetup = customCONFsetup
// when registering function
// (see end of this file)
//
static errno_t customCONFsetup()
{

    return RETURN_SUCCESS;
}




// Optional custom configuration checks
// Runs at every configuration check loop iteration
//
// To use this function, set :
// CLIcmddata.FPS_customCONFcheck = customCONFcheck
// when registering function
// (see end of this file)
//
static errno_t customCONFcheck()
{
    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}



static CLICMDDATA CLIcmddata =
{
    "compctrlmodes",
    "compute AO control modes in WFS and DM space",
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

    /*IMGID inimg = makeIMGID(inimname);
    resolveIMGID(&inimg, ERRMODE_ABORT);

    IMGID outimg = makeIMGID(outimname);
    resolveIMGID(&outimg, ERRMODE_ABORT);
    */
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // custom initialization
    if(CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
    {
        // procinfo is accessible here
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART

    printf("svdlim0 = %f\n", (float) *svdlim0);
    printf("svdlim1 = %f\n", (float) *svdlim1);

    printf("i32     = %d\n", (int) *i32);
    printf("i64     = %d\n", (int) *i64);

    printf("ui32    = %d\n", (int) *ui32);
    printf("ui64    = %d\n", (int) *ui64);

    //streamprocess(inimg, outimg);
    //processinfo_update_output_stream(processinfo, outimg.ID);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_cacao_computeCalib__compute_control_modes()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}


