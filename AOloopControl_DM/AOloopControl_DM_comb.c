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
long fpi_DMindex;

static uint32_t *DMxsize;
long fpi_DMxsize;

static uint32_t *DMysize;
long fpi_DMysize;

static uint32_t *NBchannel;

static uint32_t *DMmode;

static uint32_t *AveMode;

static int64_t  *dm2dm_mode;
long fpi_dm2dm_mode;

static char     *dm2dm_DMmodes;
long fpi_dm2dm_DMmodes;

static char     *dm2dm_outdisp;
long fpi_dm2dm_outdisp;

static int64_t  *wfsrefmode;
long fpi_wfsrefmode;

static char     *wfsref_WFSRespMat;
long fpi_wfsref_WFSRespMat;

static char     *wfsref_out;
long fpi_wfsref_out;

static int64_t  *voltmode;
static long fpi_voltmode = -1;

static uint32_t *volttype;

static float    *stroke100; // stroke [um] for 100V

static char     *voltname;
long fpi_voltname;

static float    *DClevel;

static float    *maxvolt;

static uint64_t *loopcnt;



static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32, ".DMindex", "Deformable mirror index", "5",
        CLIARG_VISIBLE_DEFAULT, (void **) &DMindex, &fpi_DMindex
    },
    {
        CLIARG_UINT32, ".DMxsize", "x size", "20",
        CLIARG_VISIBLE_DEFAULT, (void **) &DMxsize, &fpi_DMxsize
    },
    {
        CLIARG_UINT32, ".DMysize", "y size", "20",
        CLIARG_VISIBLE_DEFAULT, (void **) &DMysize, &fpi_DMysize
    },
    {
        CLIARG_UINT32, ".NBchannel", "number of DM channels", "12",
        CLIARG_HIDDEN_DEFAULT, (void **) &NBchannel, NULL
    },
    {
        CLIARG_UINT32, ".DMmode", "0:SquareGrid, 1:Generic", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &DMmode, NULL
    },
    {
        CLIARG_UINT32, ".AveMode", "Averaging mode", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &AveMode, NULL
    },
    {
        CLIARG_ONOFF, ".option.dm2dm_mode", "DM to DM offset mode", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &dm2dm_mode, &fpi_dm2dm_mode
    },
    {
        CLIARG_STREAM, ".option.dm2dm_DMmodes", "Output stream DM to DM", "null",
        CLIARG_HIDDEN_DEFAULT, (void **) &dm2dm_DMmodes, &fpi_dm2dm_DMmodes
    },
    {
        CLIARG_STREAM, ".option.dm2dm_outdisp", "data stream to which output DM is written", "null",
        CLIARG_HIDDEN_DEFAULT, (void **) &dm2dm_outdisp, &fpi_dm2dm_outdisp
    },
    {
        CLIARG_ONOFF, ".option.wfsrefmode", "WFS ref mode", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &wfsrefmode, &fpi_wfsrefmode
    },
    {
        CLIARG_STREAM, ".option.wfsref_WFSRespMat", "Output WFS resp matrix", "null",
        CLIARG_HIDDEN_DEFAULT, (void **) &wfsref_WFSRespMat, &fpi_wfsref_WFSRespMat
    },
    {
        CLIARG_STREAM, ".option.wfsref_out", "Output WFS", "null",
        CLIARG_HIDDEN_DEFAULT, (void **) &wfsref_out, &fpi_wfsref_out
    },
    {
        CLIARG_ONOFF, ".option.voltmode", "Volt mode", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &voltmode, &fpi_voltmode
    },
    {
        CLIARG_UINT32, ".option.volttype", "volt type", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &volttype, NULL
    },
    {
        CLIARG_FLOAT32, ".option.stroke100", "Stroke for 100 V [um]", "1.0",
        CLIARG_HIDDEN_DEFAULT, (void **) &stroke100, NULL
    },
    {
        CLIARG_STREAM, ".option.voltname", "Stream name for volt output", "dmvolt",
        CLIARG_HIDDEN_DEFAULT, (void **) &voltname, &fpi_voltname
    },
    {
        CLIARG_FLOAT32, ".option.DClevel", "DC level [um]", "0.5",
        CLIARG_HIDDEN_DEFAULT, (void **) &DClevel, NULL
    },
    {
        CLIARG_FLOAT32, ".option.maxvolt", "Maximum voltage", "100.0",
        CLIARG_HIDDEN_DEFAULT, (void **) &maxvolt, NULL
    },
    {
        CLIARG_UINT64, ".status.loopcnt", "Loop counter", "0",
        CLIARG_HIDDEN_DEFAULT, (void **) &loopcnt, NULL
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

        data.fpsptr->parray[fpi_DMindex].fpflag = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
        data.fpsptr->parray[fpi_DMindex].fpflag &= ~FPFLAG_WRITECONF;
        data.fpsptr->parray[fpi_DMindex].fpflag &= ~FPFLAG_WRITERUN;


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
        if(data.fpsptr->parray[fpi_dm2dm_mode].fpflag & FPFLAG_ONOFF)     // ON state
        {
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag |= FPFLAG_VISIBLE;
        }
        else     // OFF state
        {
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag &= ~FPFLAG_VISIBLE;
        }


        if(data.fpsptr->parray[fpi_wfsrefmode].fpflag & FPFLAG_ONOFF)    // ON state
        {
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_wfsref_out].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_out].fpflag |= FPFLAG_VISIBLE;
        }
        else     // OFF state
        {
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_wfsref_out].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_out].fpflag &= ~FPFLAG_VISIBLE;
        }


        if(data.fpsptr->parray[fpi_voltmode].fpflag & FPFLAG_ONOFF)    // ON state
        {
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_STREAM_RUN_REQUIRED;
        }
        else     // OFF state
        {
            data.fpsptr->parray[fpi_voltname].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_voltname].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_voltname].fpflag &= ~FPFLAG_STREAM_RUN_REQUIRED;
        }
    }

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

