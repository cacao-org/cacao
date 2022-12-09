#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "statistic/statistic.h"

// poke mode values
//
static float *pokemval = NULL;
static float *pokemfreq = NULL;
static float *pokempha = NULL;


// Local variables pointers

static char *outsname;

static char *modecsname;

static float *pokeampl;
static long      fpi_pokeampl = -1;


static float *pokefreq;
static long      fpi_pokefreq = -1;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".outsname",
        "output stream",
        "dm00disp10",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outsname,
        NULL
    },
    {
        CLIARG_IMG,
        ".mode cube",
        "modes to be poked",
        "modec",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &modecsname,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".pokeampl",
        "poke amplitude",
        "0.1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &pokeampl,
        &fpi_pokeampl
    },
    {
        CLIARG_FLOAT32,
        ".pokefreq",
        "poke frequency",
        "0.001",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &pokefreq,
        &fpi_pokefreq
    }
};



static errno_t customCONFsetup()
{

    return RETURN_SUCCESS;
}


static errno_t customCONFcheck()
{
    if(data.fpsptr != NULL)
    {
        // nothing to check
    }

    return RETURN_SUCCESS;
}


static CLICMDDATA CLIcmddata =
{
    "pokerndmodes",
    "poke modes with random amplitudes",
    CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}





static errno_t pokerndmodes(IMGID outimg, IMGID modecimg)
{

    static int NBmode = 0;
    static uint64_t iter = 0;


    if(pokemval == NULL)
    {
        printf("Initializing\n");
        NBmode = modecimg.md->size[2];
        printf("%d modes\n", NBmode);
        pokemval = (float *) malloc(sizeof(float) * NBmode);
        pokemfreq = (float *) malloc(sizeof(float) * NBmode);
        pokempha = (float *) malloc(sizeof(float) * NBmode);

        for(int m = 0; m < NBmode; m++)
        {
            pokemval[m] = (*pokeampl) * (1.0 - 2.0 * ran1());
            pokemfreq[m] = (*pokefreq) * (0.5 + 0.5 * ran1());
            pokempha[m] = 2.0 * M_PI * ran1();
        }
    }
    /*    else
        {
            for(int m = 0; m < NBmode; m++)
            {
                pokemval[m] += (*pokeampl) * (1.0 - 2.0 * ran1());
                pokemval[m] *= *pokemult;
            }
        }*/



    for(int m = 0; m < NBmode; m++)
    {
        pokempha[m] += pokemfreq[m] * ran1();
        pokemfreq[m] += (*pokefreq) * 0.01 * (1.0 - 2.0 * ran1());

        if(pokemfreq[m] < 0.5 * (*pokefreq))
        {
            pokemfreq[m] = 0.5 * (*pokefreq);
        }

        if(pokemfreq[m] > (*pokefreq))
        {
            pokemfreq[m] = (*pokefreq);
        }

        while(pokempha[m] > 2.0 * M_PI)
        {
            pokempha[m] -= 2.0 * M_PI;
        }

        pokemval[m] = (*pokeampl) * sin(pokempha[m]);
    }

    for(uint64_t ii = 0; ii < outimg.md->size[0]*outimg.md->size[1]; ii++)
    {
        outimg.im->array.F[ii] = 0.0;
    }
    for(int m = 0; m < NBmode; m++)
    {
        for(uint64_t ii = 0; ii < outimg.md->size[0]*outimg.md->size[1]; ii++)
        {
            outimg.im->array.F[ii] += pokemval[m] * modecimg.im->array.F[m *
                                      outimg.md->size[0] * outimg.md->size[1] + ii];
        }

    }

    iter++;

    return RETURN_SUCCESS;
}





static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID outimg = mkIMGID_from_name(outsname);
    resolveIMGID(&outimg, ERRMODE_ABORT);

    IMGID modecimg = mkIMGID_from_name(modecsname);
    resolveIMGID(&modecimg, ERRMODE_ABORT);

    printf(" COMPUTE Flags = %ld\n", CLIcmddata.cmdsettings->flags);
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // custom initialization
    printf(" COMPUTE Flags = %ld\n", CLIcmddata.cmdsettings->flags);
    if(CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
    {
        // procinfo is accessible here
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART

    pokerndmodes(outimg, modecimg);
    processinfo_update_output_stream(processinfo, outimg.ID);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_DM__pokerndmodes()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
