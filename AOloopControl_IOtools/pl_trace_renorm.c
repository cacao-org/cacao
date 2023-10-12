/**
 * @file    pl_trace_renorm.c
 * @brief   acquire and normalize spectra of 3-port PL
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

static int64_t *compWFSnormalize; // spec norm toggle (lifted this from acquireWFSim.c heh)
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
        CLIARG_STR,
        ".wfsout",
        "Wavefront sensor output",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &output_shm_name,
        &fpi_outputshmname
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSnormalize",
        "normalize WFS frames -> imWFS1",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSnormalize,
        &fpi_compWFSnormalize
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
    uint32_t sizeout = sizeoutx * sizeouty;

    // Create output
    // trying to copy shape of wfsin. Assume a structure 3 rows x N columns, N goes along wavelength
    IMGID wfsout;
    wfsout =
        stream_connect_create_2D(output_shm_name, sizeoutx, sizeouty, _DATATYPE_FLOAT);

    // This is the while(True) {
    //INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // JON YOU GET TO WORK HERE
        if(data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF) {
            int j;
            for (j = 0; j < sizeouty; j++){
                double tot = 0.0;
                int i;
                for (i = 0; i < sizeoutx; i++) {
                    tot += wfsin.im->array.F[i*sizeouty+j];
                }
                for (i = 0; i < sizeoutx; i++) {
                    wfsout.im->array.F[i*sizeouty+j] /= tot;
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
errno_t CLIADDCMD_AOloopControl_IOtools__PLtracerenorm()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
