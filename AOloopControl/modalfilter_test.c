/**
 * @file    modalfilter_test.c
 * @brief   simulate sequence for modal filter test
 *
 * Creates a multi-dim stream sequence to feed to mfilt
 * Noise is added at each step, and attenuation is applied
 * (mutlfactor) to keep the sequence from diverging
 *
 * Connects to:
 * [mvalWFS] aolX_mfiltt_mvalWFS (can be sym link to aolX_modevalWFS)
 * [mvalDM]  aolX_mfiltt_mvalDM  (can by sym link to aolX_modevalDM)
 *
 * Internal arrays
 * [mvalIN]                : input disturbance
 * [mvalDMd]               : time-delayed input DM correction
 * [mvalC]                 : Corrected: mvalIN-mvalDMd
 *
 * outmval [user-set name] : output
 *
 * Main steps:
 * - Compute mvalIN
 * - Delay the input correction: mvalDM  (DMdelay)> mvalDMd
 * - Apply correction: mvalIN+mvalDMd -> mvalC
 * - Apply time delay, wite ouput: mvalC (WFSdelay)-> outmval
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

// for random noise
#include "statistic/statistic.h"



#define SNAMEPREFIX "tseqPF"


static uint64_t *AOloopindex;

static char *mvalDM;
static long  fpi_mvalDM;

static char *mvalWFS;
static long  fpi_mvalWFS;



static float *minPrate;
static long   fpi_minPrate;

static float *maxPrate;
static long   fpi_maxPrate;



static float *noiseamp;
static long   fpi_noiseamp;

static float *multfact;
static long   fpi_multfact;


static float *WFSlatency;
static long   fpi_WFSlatency;

static float *DMlatency;
static long   fpi_DMlatency;







static CLICMDARGDEF farg[] =
{
    {
        // AO loop index. Used for naming streams aolX_
        CLIARG_UINT64,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {
        CLIARG_STREAM,
        ".mvalDM",
        "input mode values - DM control",
        "aol0_mfiltt_mvalDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &mvalDM,
        &fpi_mvalDM
    },
    {
        CLIARG_STREAM,
        ".mvalWFS",
        "output mode values to WFS",
        "aol0_mfiltt_mvalWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &mvalWFS,
        &fpi_mvalWFS
    },
    {
        CLIARG_FLOAT32,
        ".minPrate",
        "min phase rate [rad/tstep]",
        "0.1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &minPrate,
        &fpi_minPrate
    },
    {
        CLIARG_FLOAT32,
        ".maxPrate",
        "max phase rate [rad/tstep]",
        "2.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maxPrate,
        &fpi_maxPrate
    },
    {
        // Random noise amplitude
        // injected at each iteration
        // drives mval to random walk
        CLIARG_FLOAT32,
        ".noiseamp",
        "noise amplitude",
        "0.1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &noiseamp,
        &fpi_noiseamp
    },
    {
        // Mult factor
        // multiplied to ouput at each iteration
        // drives back toward zero
        CLIARG_FLOAT32,
        ".multfact",
        "multiplicative factor",
        "0.99",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &multfact,
        &fpi_multfact
    },
    {
        // WFS latency\ [frame]
        CLIARG_FLOAT32,
        ".WFSlatency",
        "WFS latency [frame]",
        "2.7",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFSlatency,
        &fpi_WFSlatency
    },
    {
        // DM latency\ [frame]
        CLIARG_FLOAT32,
        ".DMlatency",
        "DM latency [frame]",
        "0.8",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMlatency,
        &fpi_DMlatency
    }
};


// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_mvalDM].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_mvalWFS].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;


        data.fpsptr->parray[fpi_minPrate].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_multfact].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_maxPrate].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_noiseamp].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_DMlatency].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSlatency].fpflag |= FPFLAG_WRITERUN;
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
    "mfilttest", "test input for modal filter", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    // Connect to mvalDM
    // connect to input mode values array and get number of modes
    //
    IMGID imgmvalDM = mkIMGID_from_name(mvalDM);
    resolveIMGID(&imgmvalDM, ERRMODE_ABORT);
    printf("%u modes\n", imgmvalDM.md->size[0]);
    uint32_t NBmode = imgmvalDM.md->size[0];

    // Connect to mvalWFS
    //
    IMGID imgmvalWFS = mkIMGID_from_name(mvalWFS);
    resolveIMGID(&imgmvalWFS, ERRMODE_ABORT);



    // connect / create mvalC
    //IMGID imgmvalC = stream_connect_create_2Df32(mvalCname, *NBmode, 1);

    // connect / create mvalout
    //IMGID imgmvalout = stream_connect_create_2Df32(mvaloutname, *NBmode, 1);







    // create input buffer holding recent input values to apply delays
    //
    uint32_t NBdelaystep = 50;

    // mvalDM buffer
    uint32_t mvalDMbuff_tindex = 0;
    IMGID imgmvalDMbuff = makeIMGID_2D("mvalDMbuff", NBmode, NBdelaystep);
    createimagefromIMGID(&imgmvalDMbuff);

    // mvalOUT buffer
    uint32_t mvalCbuff_tindex = 0;
    IMGID imgmvalCbuff = makeIMGID_2D("mvalCbuff", NBmode, NBdelaystep);
    createimagefromIMGID(&imgmvalCbuff);



    list_image_ID();




    float *mvalIN = (float*) malloc(sizeof(float)*NBmode);
    float *mvalINpha = (float*) malloc(sizeof(float)*NBmode);

    // time-delayed DM correction
    float *mvalDMd = (float*) malloc(sizeof(float)*NBmode);

    // Corrected input
    float *mvalC = (float*) malloc(sizeof(float)*NBmode);

    // Time-delayed corrected input
    float *mvalCd = (float*) malloc(sizeof(float)*NBmode);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START


    // Write input disturbance
    //
    for(uint32_t mi=0; mi < NBmode; mi++)
    {
        float phastep = (*minPrate) + (1.0*mi / NBmode) * ((*maxPrate) - (*minPrate));
        mvalINpha[mi] += phastep;

        mvalIN[mi] = cos(mvalINpha[mi]);

        // add noise
        mvalIN[mi] += (*noiseamp) * (1.0-2.0*ran1());
        // mult
        mvalIN[mi] *= (*multfact);
    }

    //printf("mi0:  %8.6f\n", mvalIN[0]);


    {
        // Grab new input mvalDM
        char *ptr = (char *) imgmvalDMbuff.im->array.F;
        ptr += SIZEOF_DATATYPE_FLOAT*NBmode*mvalDMbuff_tindex;
        memcpy( ptr, imgmvalDM.im->array.F, sizeof(float)*NBmode);

        int latint = floor(*DMlatency);  // integer part
        float latfrac = (*DMlatency) - latint;  // fractional part
        int index0 = mvalDMbuff_tindex - latint;
        if(index0 < 0)
        {
            index0 += NBdelaystep;
        }
        int index1 = index0 - 1;
        if(index1 < 0)
        {
            index1 += NBdelaystep;
        }

        for(uint32_t mi=0; mi < NBmode; mi++)
        {
            mvalDMd[mi] = (1.0-latfrac) * imgmvalDMbuff.im->array.F[index0*NBmode+mi];
            mvalDMd[mi] += latfrac * imgmvalDMbuff.im->array.F[index1*NBmode+mi];
        }

        {
            uint32_t mi = 5;
            printf("latency DM index  [%3d %3d  %+9.6f  %+9.6f -> %+9.6f   %+9.6f\n",
                   index0, index1,
                   imgmvalDMbuff.im->array.F[index0*NBmode+mi],
                   imgmvalDMbuff.im->array.F[index1*NBmode+mi],
                   mvalDMd[mi],
                   imgmvalDM.im->array.F[mi]);
        }

    }
    // apply time-delayed DM correction
    for(uint32_t mi=0; mi < NBmode; mi++)
    {
        mvalC[mi] = mvalIN[mi] + mvalDMd[mi];
    }
    // update DM buffer index
    mvalDMbuff_tindex ++;
    if(mvalDMbuff_tindex == NBdelaystep)
    {
        mvalDMbuff_tindex = 0;
    }



    {
        // Grab new input mvalC
        char *ptr = (char *) imgmvalCbuff.im->array.F;
        ptr += SIZEOF_DATATYPE_FLOAT*NBmode*mvalCbuff_tindex;
        memcpy( ptr, mvalC, sizeof(float)*NBmode);

        int latint = floor(*WFSlatency);  // integer part
        float latfrac = (*WFSlatency) - latint;  // fractional part
        int index0 = mvalCbuff_tindex - latint;
        if(index0 < 0)
        {
            index0 += NBdelaystep;
        }
        int index1 = mvalCbuff_tindex - latint;
        if(index1 < 0)
        {
            index1 += NBdelaystep;
        }

        for(uint32_t mi=0; mi < NBmode; mi++)
        {
            mvalCd[mi] = (1.0-latfrac) * imgmvalCbuff.im->array.F[index0*NBmode+mi];
            mvalCd[mi] += latfrac * imgmvalCbuff.im->array.F[index1*NBmode+mi];
        }
    }
    // update C buffer index
    mvalCbuff_tindex ++;
    if(mvalCbuff_tindex == NBdelaystep)
    {
        mvalCbuff_tindex = 0;
    }


    memcpy(imgmvalWFS.im->array.F, mvalCd, sizeof(float)*NBmode);
    processinfo_update_output_stream(processinfo, imgmvalWFS.ID);

    /*
        {
            int mimax = NBmode;
            if(mimax > 5)
            {
                mimax = 5;
            }
            for(int mi=0; mi < mimax; mi++)
            {
                printf("%10.6f  ", mvalIN[mi]);
            }
            printf("\n");
        }
    */


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalC);
    free(mvalCd);
    free(mvalDMd);
    free(mvalIN);
    free(mvalINpha);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__modalfilter_test()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
