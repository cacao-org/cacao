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

static float    *maskRMp0;
static float    *maskRMc0;
static float    *maskRMp1;
static float    *maskRMc1;

static float    *DMproxrad;



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
        (void **) &maskRMp0, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMc0", "DM mask, point0 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskRMc0, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMp1", "DM mask, point1 percentile point", "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskRMp1, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.RMc1", "DM mask, point1 coefficient", "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskRMc1, NULL
    },
    {
        CLIARG_FLOAT32, ".DMmask.proxrad", "DM actuator proximity radius", "2.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMproxrad, NULL
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


