/**
 * @file    mlat.c
 * @brief   measure hardware latency
 *
 * Measure latency between DM and WFS
 *
 *
 */

#include "CommandLineInterface/CLIcore.h"



// Local variables pointers
static char *dmstream;
long fpi_dmstream;

static char *wfsstream;
long fpi_wfsstream;


static float *frameratewait;
long fpi_frameratewait;

static float *OPDamp;
long fpi_OPDamp;

static uint32_t *NBiter;
long fpi_NBiter;

static uint32_t *wfsNBframemax;
long fpi_wfsNBframemax;

static float *wfsdt;
long fpi_wfsdt;

static float *twaitus;
long fpi_twaitus;

static float *refdtoffset;
long fpi_refdtoffset;

static float *dtoffset;
long fpi_dtoffset;

static float *framerateHz;
long fpi_framerateHz;

static float *latencyfr;
long fpi_latencyfr;





static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STREAM, ".dmstream", "DM stream", "null",
        CLIARG_VISIBLE_DEFAULT, (void **) &dmstream, &fpi_dmstream
    },
    {
        CLIARG_STREAM, ".wfsstream", "WFS stream", "null",
        CLIARG_VISIBLE_DEFAULT, (void **) &wfsstream, &fpi_wfsstream
    },
    {
        CLIARG_FLOAT32, ".OPDamp", "poke amplitude [um]", "0.1",
        CLIARG_VISIBLE_DEFAULT, (void **) &OPDamp, &fpi_OPDamp
    },
    {
        CLIARG_FLOAT32, ".frameratewait", "time period for frame rate measurement", "5",
        CLIARG_HIDDEN_DEFAULT, (void **) &frameratewait, &fpi_frameratewait
    },
    {
        CLIARG_UINT32, ".NBiter", "Number of iteration", "100",
        CLIARG_HIDDEN_DEFAULT, (void **) &NBiter, &fpi_NBiter
    },
    {
        CLIARG_UINT32, ".wfsNBframemax", "Number frames in measurement sequence", "50",
        CLIARG_HIDDEN_DEFAULT, (void **) &wfsNBframemax, &fpi_wfsNBframemax
    },
    {
        CLIARG_FLOAT32, ".status.wfsdt", "WFS frame interval", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &wfsdt, &fpi_wfsdt
    },
    {
        CLIARG_FLOAT32, ".status.twaitus", "initial wait [us]", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &twaitus, &fpi_twaitus
    },
    {
        CLIARG_FLOAT32, ".status.refdtoffset", "baseline time offset to poke", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &refdtoffset, &fpi_refdtoffset
    },
    {
        CLIARG_FLOAT32, ".status.dtoffset", "actual time offset to poke", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &dtoffset, &fpi_dtoffset
    },
    {
        CLIARG_FLOAT32, ".out.framerateHz", "WFS frame rate [Hz]", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &framerateHz, &fpi_framerateHz
    },
    {
        CLIARG_FLOAT32, ".out.latencyfr", "WFS frame rate [Hz]", "0",
        CLIARG_OUTPUT_DEFAULT, (void **) &latencyfr, &fpi_latencyfr
    }
};




// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_dmstream].fpflag |= FPFLAG_STREAM_RUN_REQUIRED|FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_wfsstream].fpflag |= FPFLAG_STREAM_RUN_REQUIRED|FPFLAG_CHECKSTREAM;
    }



    return RETURN_SUCCESS;
}


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
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
    "mlat",
    "measure latency between DM and WFS",
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

    //uint32_t dmxsize;
    //uint32_t dmysize;

    // connect to DM
    IMGID imgdm = makeIMGID(wfsstream);
    resolveIMGID(&imgdm, ERRMODE_ABORT);
    printf("DM size : %u %u\n", imgdm.md->size[0], imgdm.md->size[1]);

    // connect to WFS
    IMGID imgwfs = makeIMGID(wfsstream);
    resolveIMGID(&imgwfs, ERRMODE_ABORT);
    printf("WFS size : %u %u\n", imgwfs.md->size[0], imgwfs.md->size[1]);


    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    // coarse estimate of frame rate

    {
        double tdouble_start;
        double tdouble_end;
        long wfscntstart;
        long wfscntend;
        struct timespec tnow;

        long stringmaxlen = 200;
        char msgstring[stringmaxlen];
        snprintf(msgstring, stringmaxlen, "Measuring approx frame rate over %.1f sec", *frameratewait);
        processinfo_WriteMessage(processinfo, msgstring);

        clock_gettime(CLOCK_REALTIME, &tnow);
        tdouble_start = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
        wfscntstart = imgwfs.md->cnt0;
        usleep( (long)  (1000000 * (*frameratewait)) );
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdouble_end = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
        wfscntend = imgwfs.md->cnt0;
        *wfsdt = (tdouble_end - tdouble_start) / (wfscntend - wfscntstart);

        printf("wfs dt = %f sec\n", *wfsdt);

        if(wfscntend - wfscntstart < 5)
        {
            snprintf(msgstring, stringmaxlen,
                     "Number of frames %ld too small -> cannot proceed", wfscntend - wfscntstart);
            processinfo_error(processinfo, msgstring);
//        free(latencyarray);
//        free(latencysteparray);
            DEBUG_TRACE_FEXIT();
            return RETURN_FAILURE;
        }

        snprintf(msgstring, stringmaxlen, "frame period wfsdt = %f sec  ( %f Hz )\n",
                 *wfsdt, 1.0 / *wfsdt);
        processinfo_WriteMessage(processinfo, msgstring);

        // This is approximate, will be measured more precisely later on
        *framerateHz = 1.0 / (*wfsdt);
    }






    printf("mlat iteration done\n");

    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_perfTest__mlat()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}




