/**
 * @file    norm_spectra.c
 * @brief   normalize spectra
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

static char *output_shm_name; // output shared memory
static long  fpi_outputshmname;

static int64_t *compWFSnormalize; // toggle for normalization
static long     fpi_compWFSnormalize;

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
        CLIARG_ONOFF,
        ".comp.WFSnormalize",
        "normalize spectral traces",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSnormalize,
        &fpi_compWFSnormalize
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
    "norm_spectra", "normalize spectra", CLICMD_FIELDS_DEFAULTS
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

    // Create output, same size as input
    IMGID wfsout;
    wfsout =
        stream_connect_create_2D(output_shm_name,sizeoutx,sizeouty,_DATATYPE_FLOAT);

    // This is the while(True) {
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // JON YOU GET TO WORK HERE
        if(data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF) {
            int j;
            for (j = 0; j < sizeouty; j++){
                double tot = 0.0;
                int i;
                for (i = 0; i < sizeoutx; i++) {
                    tot += wfsin.im->array.F[j*sizeoutx + i];
                }
                double normval;
                if (tot <= 0){
                    normval = 0;
                }
                else {
                    normval = 1/tot;
                }
                for (i = 0; i < sizeoutx; i++) {
                    wfsout.im->array.F[j*sizeoutx + i] = wfsin.im->array.F[j*sizeoutx + i]*normval;
                }
            }
        }
        else {
            memcpy(wfsout.im->array.F,
            wfsin.im->array.F,
            sizeof(float) * sizeout);
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
errno_t CLIADDCMD_AOloopControl_IOtools__normspectra()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
