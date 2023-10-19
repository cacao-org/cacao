/**
 * @file    extract_spectra.c
 * @brief   extract spectra
 *
 *
 *
 */

#include <math.h>
#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_tools/COREMOD_tools.h"

// Local variables pointers
static char *input_shm_name; // input shared memory
static long  fpi_inputshmname;

static char *specmask_shm_name; // mask shared memory
static long  fpi_specmaskshmname;

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
        CLIARG_IMG,
        ".wfsmask",
        "Wavefront sensor spectral extraction mask",
        "wfsspecmask",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &specmask_shm_name,
        &fpi_specmaskshmname
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
    "extract_spectra", "extract spectra", CLICMD_FIELDS_DEFAULTS
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

    IMGID wfsmask = mkIMGID_from_name(specmask_shm_name);
    resolveIMGID(&wfsmask, ERRMODE_ABORT);
    uint32_t masksizeoutz = wfsmask.size[2];

    // Create output
    IMGID wfsout;
    wfsout =
        stream_connect_create_2D(output_shm_name,sizeoutx,3,_DATATYPE_FLOAT);

    // This is the while(True) {
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // JON YOU GET TO WORK HERE

        for (uint32_t k = 0; k < masksizeoutz; k++){
            for (uint32_t i = 0; i < sizeoutx; i++){
                float tot = 0.0;
                for (uint32_t j = 0; j < sizeouty; j++) {
                    uint64_t pixindex = k * sizeoutx * sizeouty +  i * sizeouty + j; //
                    tot += wfsin.im->array.UI16[pixindex] * wfsmask.im->array.F[pixindex];
                }
                wfsout.im->array.F[k * 3 + i] = tot;
            }
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
errno_t CLIADDCMD_AOloopControl_IOtools__extractspectra()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
