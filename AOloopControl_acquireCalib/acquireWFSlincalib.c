/**
 * @file acquireWFSlincalib.c
 * @brief Acquire linear WFS response
 *
 */

#include "CommandLineInterface/CLIcore.h"


// Local variables pointers
static uint32_t *AOloopindex;
static float    *pokeampl;

// timing params
static uint32_t *delayfr;
static uint32_t *delayRM1us;
static uint32_t *NBave;
static uint32_t *NBexcl;
static uint32_t *NBcycle;
static uint32_t *NBinnerCycle;

static uint32_t *AOinitMode;
static uint64_t *MaskMode;

static float    *maskDMp0;
static float    *maskDMc0;
static float    *maskDMp1;
static float    *maskDMc1;

static float    *DMproxrad;

static float    *maskWFSp0;
static float    *maskWFSc0;
static float    *maskWFSp1;
static float    *maskWFSc1;

static char     *fn_pokeC;
static char     *fn_RMDMmask;

static uint64_t *normalize;
static uint64_t *autotiming;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32, ".AOloopindex", "loop index", "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex, NULL
    },
    {
        CLIARG_FLOAT32, ".ampl", "RM poke amplitude", "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &pokeampl, NULL
    },
    {
        CLIARG_UINT32, ".timing.delayfr", "frame delay, whole part", "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayfr, NULL
    },
    {
        CLIARG_UINT32, ".timing.delayRM1us", "Sub-frame delay [us]", "100",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayRM1us, NULL
    },
    {
        CLIARG_UINT32, ".timing.NBave", "Number of frames averaged for a single poke measurement", "5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBave, NULL
    },
    {
        CLIARG_UINT32, ".timing.NBexcl", "Number of frames excluded", "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBexcl, NULL
    },
    {
        CLIARG_UINT32, ".timing.NBcycle", "Number of measurement cycles to be repeated", "10",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBcycle, NULL
    },
    {
        CLIARG_UINT32, ".timing.NBinnerCycle", "Number of inner cycles (how many consecutive times should a single +/- poke be repeated)", "10",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBinnerCycle, NULL
    },
    {
        CLIARG_UINT32, ".AOinitMode", "AO initialization mode", "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &AOinitMode, NULL
    },
    {
        CLIARG_ONOFF, ".MaskMode", "Mask mode, DM and WFS", "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &MaskMode, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMp0", "DM mask, point0 percentile point", "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMp0, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMc0", "DM mask, point0 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMc0, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMp1", "DM mask, point1 percentile point", "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMp1, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMc1", "DM mask, point1 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMc1, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.proxrad", "DM actuator proximity radius", "2.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMproxrad, NULL
    },
    {
        CLIARG_FLOAT32, ".WFSmask.RMp0", "WFS mask, point0 percentile point", "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSp0, NULL
    },
    {
        CLIARG_FLOAT32, ".WFSmask.RMc0", "WFS mask, point0 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSc0, NULL
    },
    {
        CLIARG_FLOAT32, ".WFSmask.RMp1", "WFS mask, point1 percentile point", "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSp1, NULL
    },
    {
        CLIARG_FLOAT32, ".WFSmask.RMc1", "WFS mask, point1 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSc1, NULL
    },
    {
        CLIARG_FITSFILENAME, ".fn_pokeC", "Poke sequence cube", "nullfitsfn",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fn_pokeC, NULL
    },
    {
        CLIARG_FITSFILENAME, ".fn_RMDMmask", "RM active DM actuators mask", "nullfitsfn",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fn_RMDMmask, NULL
    },
    {
        CLIARG_ONOFF, ".normalize", "Normalize WFS frames", "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &normalize, NULL
    },
    {
        CLIARG_ONOFF, ".Hpoke", "Normalize WFS frames", "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &normalize, NULL
    },
    {
        CLIARG_ONOFF, ".autoTiming", "Auto Timing", "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autotiming, NULL
    }
};



static CLICMDDATA CLIcmddata =
{
    "acqWFSlincal",
    "acquire linear WFS calibration",
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


    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART



    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_milk_AOloopControl_acquireCalib__acquireWFSlincalib()
{

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}


