/**
 * @file compute_control_modes.c
 * @brief Compute AO control modes in both input (WFS) and output (DM) space
 *
 */

#include "CommandLineInterface/CLIcore.h"


// Local variables pointers



static float *svdlim;

static CLICMDARGDEF farg[] =
{
    {
        CLIARG_FLOAT32, ".svdlim", "SVD limit", "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &svdlim, NULL
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

    printf("svdlim = %f\n", *svdlim);
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


