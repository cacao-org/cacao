/**
 * @file    pl_column_sum.c
 * @brief   sum the columns of input shm
 *
 *
 *
 */

#include <math.h>
#include "CommandLineInterface/CLIcore.h"

// Local variables pointers
static char *input_shm_name; // input shared memory
static long  fpi_inputshmname;

static char *output_shm_name; // output shared memory
static long  fpi_outputshmname;

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
    {
        CLIARG_STR,
        ".wfsout",
        "Wavefront sensor output",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &output_shm_name,
        &fpi_outputshmname
    },
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
    "column_sum", "sum columns", CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();
    // JON YOU GET TO INIT HERE

    IMGID wfsin = mkIMGID_from_name(input_shm_name);
    resolveIMGID(&wfsin, ERRMODE_ABORT);

    uint32_t sizeoutx = wfsin.size[0];
    uint32_t sizeouty = wfsin.size[1];

    // Create output
    IMGID wfsout;
    wfsout =
        stream_connect_create_2D(output_shm_name, sizeoutx,1, _DATATYPE_FLOAT);

    // This is the while(True) {
    //INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        // JON YOU GET TO WORK HERE
        uint32_t i;
        for (i = 0; i < sizeoutx; i++){
            float tot = 0.0;
            uint32_t j;
            for (j = 0; j < sizeouty; j++) {
                uint64_t pixindex =  j*sizeoutx + i;//i * sizeouty + j;
                tot += wfsin.im->array.F[pixindex];
            }
            wfsout.im->array.F[i] = tot;
        }
        
        // Done and post downstream.
        processinfo_update_output_stream(processinfo, wfsout.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END // } // while (true)

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__PLcolumnsum()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
