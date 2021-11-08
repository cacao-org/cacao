/**
 * @file    AOloopControl_DM_comb.c
 * @brief   DM control
 *
 * Combine DM channels
 *
 *
 *
 */

#include "CommandLineInterface/CLIcore.h"



// Local variables pointers
static uint32_t *DMindex;

static uint32_t *DMxsize;
static uint32_t *DMysize;

static uint32_t *NBchannel;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32, ".DMindex", "Deformable mirror index", "5",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMindex
    },
    {
        CLIARG_UINT32, ".DMxsize", "x size", "20",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMxsize
    },
    {
        CLIARG_UINT32, ".DMysize", "y size", "20",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMysize
    },
    {
        CLIARG_UINT32, ".NBchannel", "number of DM channels", "12",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBchannel
    }
};


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    printf(">>>>>>>>>>>>>>>>>>> customCONFsetup\n");
//    *DMysize = 45;

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    printf("customCONFcheck >>>>>>>>>>>>>>>>>\n");
    *DMysize = 45;

    return RETURN_SUCCESS;
}



static CLICMDDATA CLIcmddata =
{
    "DMcomb",
    "Deformable mirror combine channels",
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


    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    printf("This is DM comb, index = %ld\n", (long) *DMindex);


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_DM__comb()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}

