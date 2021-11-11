/**
 * @file    modalfilter.c
 * @brief   Apply modal filtering
 *
 *
 *
 */


#include <math.h>

#include "CommandLineInterface/CLIcore.h"


// Local variables pointers
static char *inWFSstream;
long fpi_inWFSstream;

static char *modevalDM;
long fpi_modevalDM;

static float *loopgain;
long fpi_loopgain;

static float *loopmult;
long fpi_loopmult;

static float *vlimit;
long fpi_vlimit;

static float *galpha;
long fpi_galpha;

static uint32_t *mimax;
long fpi_mimax;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STREAM, ".inWFSstream", "input WFS stream", "aol0_modeval",
        CLIARG_VISIBLE_DEFAULT, (void **) &inWFSstream, &fpi_inWFSstream
    },
    {
        CLIARG_STREAM, ".modevalDM", "DM mode coefficient values", "aol0_modevalDM",
        CLIARG_VISIBLE_DEFAULT, (void **) &modevalDM, &fpi_modevalDM
    },
    {
        CLIARG_FLOAT32, ".loopgain", "loop gain", "0.01",
        CLIARG_HIDDEN_DEFAULT, (void **) &loopgain, &fpi_loopgain
    },
    {
        CLIARG_FLOAT32, ".loopmult", "loop mult", "0.95",
        CLIARG_HIDDEN_DEFAULT, (void **) &loopmult, &fpi_loopmult
    },
    {
        CLIARG_FLOAT32, ".vlimit", "value limit", "0.2",
        CLIARG_HIDDEN_DEFAULT, (void **) &vlimit, &fpi_vlimit
    },
    {
        CLIARG_FLOAT32, ".galpha", "loop gain alpha (1=flat)", "0.5",
        CLIARG_HIDDEN_DEFAULT, (void **) &galpha, &fpi_galpha
    },
    {
        CLIARG_UINT32, ".mimax", "maximum mode index", "100",
        CLIARG_HIDDEN_DEFAULT, (void **) &mimax, &fpi_mimax
    }
};


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_vlimit].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_galpha].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_mimax].fpflag |= FPFLAG_WRITERUN;
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
    "modalfilter",
    "modal filtering",
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

    // connect to WFS modes
    IMGID imgin = makeIMGID(inWFSstream);
    resolveIMGID(&imgin, ERRMODE_ABORT);
    printf("%u modes\n", imgin.md->size[0]);
    uint32_t NBmode = imgin.md->size[0];

    float *mvalout = (float *) malloc(sizeof(float) * NBmode);

    // create output mode coeffs
    imageID IDmodevalDM;
    {
        uint32_t naxes[2];
        naxes[0] = imgin.md->size[0];
        naxes[1] = 1;

        create_image_ID( modevalDM, 2, naxes, imgin.datatype, 1, 0, 0, &IDmodevalDM);
    }

    for(uint32_t mi=0; mi<NBmode; mi++)
    {
        data.image[IDmodevalDM].array.F[mi] = 0.0;
        mvalout[mi] = 0.0;
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    for(uint32_t mi=0; mi<NBmode; mi++)
    {
        float x = 1.0*mi / NBmode;

        float gain = (*loopgain);
        gain *= pow( (*galpha), x);

        if(mi > (*mimax))
        {
            gain = 0.0;
        }
        else
        {
            gain *= 1.0; //TODO figure out why necessary
        }

        float mult = (*loopmult);
        float limitval = (*vlimit);

        mvalout[mi] = (1.0-gain)*mvalout[mi] - gain * imgin.im->array.F[mi];
        mvalout[mi] *= mult;

        if(mvalout[mi] > limitval)
        {
            mvalout[mi] = limitval;
        }

        if(mvalout[mi] < -limitval)
        {
            mvalout[mi] = -limitval;
        }
    }

    memcpy(data.image[IDmodevalDM].array.F, mvalout, sizeof(float) * NBmode);
    processinfo_update_output_stream(processinfo, IDmodevalDM);


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalout);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl__modalfilter()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}


