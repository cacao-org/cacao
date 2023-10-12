/**
 * @file    .c
 * @brief
 *
 *
 *
 */


#include "CommandLineInterface/CLIcore.h"

static long NUM_APD_HOWFS = 188;
static long NUM_APD_LOWFS = 16;

// Local variables pointers
static char *input_shm_name; // THE FPS PARAMETER VARIABLE
static long  fpi_inputshmname;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".wfsin",
        "Wavefront sensor input",
        "wfsin",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &input_shm_name,
        &fpi_inputshmname
    },
    // TODO time-downsample factor for LOWFS.
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

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // JON YOU GET TO INIT HERE

    // This is the while(True) {
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // JON YOU GET TO WORK HERE

        // Done and post downstream.
        processinfo_update_output_stream(processinfo, curv_2k_singlesided.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END // } // while (true)

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__PLtracerenorm()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
