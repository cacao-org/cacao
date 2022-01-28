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
static int AOloopindex = 0;

static char *inmval;
long         fpi_inmval;

static char *outmval;
long         fpi_outmval;

static float *loopgain;
long          fpi_loopgain;

static float *loopmult;
long          fpi_loopmult;

static float *vlimit;
long          fpi_vlimit;

static float *galpha;
long          fpi_galpha;

static uint32_t *mimax;
long             fpi_mimax;

static uint32_t *avets;
long             fpi_avets;

static uint32_t *aftgain;
long             fpi_aftgain;

static CLICMDARGDEF farg[] = {{CLIARG_STREAM,
                               ".inmval",
                               "input mode values from WFS",
                               "aol0_modevalWFS",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &inmval,
                               &fpi_inmval},
                              {CLIARG_STREAM,
                               ".outmval",
                               "output mode values to DM",
                               "aol0_modevalDM",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &outmval,
                               &fpi_outmval},
                              {CLIARG_FLOAT32,
                               ".loopgain",
                               "loop gain",
                               "0.01",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &loopgain,
                               &fpi_loopgain},
                              {CLIARG_FLOAT32,
                               ".loopmult",
                               "loop mult",
                               "0.95",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &loopmult,
                               &fpi_loopmult},
                              {CLIARG_FLOAT32,
                               ".vlimit",
                               "value limit",
                               "0.2",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &vlimit,
                               &fpi_vlimit},
                              {CLIARG_FLOAT32,
                               ".galpha",
                               "loop gain alpha (1=flat)",
                               "0.5",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &galpha,
                               &fpi_galpha},
                              {CLIARG_UINT32,
                               ".mimax",
                               "maximum mode index",
                               "100",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &mimax,
                               &fpi_mimax},
                              {CLIARG_FLOAT32,
                               ".avets",
                               "averaging timescale [nb fr]",
                               "10000.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &avets,
                               &fpi_avets},
                              {CLIARG_FLOAT32,
                               ".aftgain",
                               "afterburner gain",
                               "0.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &aftgain,
                               &fpi_aftgain}};

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_vlimit].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_galpha].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_mimax].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_avets].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_aftgain].fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if (data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata = {
    "modalfilter", "modal filtering", CLICMD_FIELDS_DEFAULTS};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to input mode values array and get number of modes
    //
    IMGID imgin = makeIMGID(inmval);
    resolveIMGID(&imgin, ERRMODE_ABORT);
    printf("%u modes\n", imgin.md->size[0]);
    uint32_t NBmode = imgin.md->size[0];

    float *mvalout = (float *) malloc(sizeof(float) * NBmode);

    float *avemval = (float *) malloc(sizeof(float) * NBmode);

    // create output mode coeffs
    imageID IDoutmval;
    imageID IDmodegainfact;
    {
        uint32_t naxes[2];
        naxes[0] = imgin.md->size[0];
        naxes[1] = 1;

        char modegfname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(modegfname, "aol%d_modevalgain", AOloopindex);

        printf("Create %s size %ld %ld\n", outmval, naxes[0], naxes[1]);

        create_image_ID(outmval, 2, naxes, imgin.datatype, 1, 0, 0, &IDoutmval);

        create_image_ID(modegfname,
                        2,
                        naxes,
                        imgin.datatype,
                        1,
                        0,
                        0,
                        &IDmodegainfact);
    }

    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        data.image[IDoutmval].array.F[mi]      = 0.0;
        mvalout[mi]                            = 0.0;
        avemval[mi]                            = 0.0;
        data.image[IDmodegainfact].array.F[mi] = 1.0;
    }

    float avegain = 1.0 / (*avets);
    printf("avegain = %f\n", avegain);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    avegain = 1.0 / (*avets);
    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        float x = 1.0 * mi / NBmode;

        float gain = (*loopgain);
        gain *= pow((*galpha), x);
        gain *= data.image[IDmodegainfact].array.F[mi];

        if (mi > (*mimax))
        {
            gain = 0.0;
        }
        /*        else
      {
      gain *= 1.0e-4; //TODO figure out why necessary
      }
      */
        float mult     = (*loopmult);
        float limitval = (*vlimit);

        /*        if((mi==0) || (mi==1))
      {
      gain *= 4.0;
      }
      */

        avemval[mi] = (1.0 - avegain) * avemval[mi] +
                      avegain * (imgin.im->array.F[mi] * gain);

        // update long term average if input mode values
        mvalout[mi] = (1.0 - gain) * mvalout[mi] -
                      gain * (imgin.im->array.F[mi] - (*aftgain) * avemval[mi]);
        mvalout[mi] *= mult;

        if (mi > (*mimax))
        {
            mvalout[mi] = 0.0;
        }

        if (mvalout[mi] > limitval)
        {
            mvalout[mi] = limitval;
        }

        if (mvalout[mi] < -limitval)
        {
            mvalout[mi] = -limitval;
        }

        if (avemval[mi] > limitval)
        {
            avemval[mi] = limitval;
        }

        if (avemval[mi] < -limitval)
        {
            avemval[mi] = -limitval;
        }
    }

    memcpy(data.image[IDoutmval].array.F, mvalout, sizeof(float) * NBmode);
    processinfo_update_output_stream(processinfo, IDoutmval);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalout);
    free(avemval);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

    // Register function in CLI
    errno_t
    CLIADDCMD_AOloopControl__modalfilter()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
