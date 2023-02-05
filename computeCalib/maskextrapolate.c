/**
 * @file maskextrapolate.c
 *
 */


#include "CommandLineInterface/CLIcore.h"




// Input modes to be masked/extrapolated
//
static char *inmodeC;
static long  fpi_inmodeC;

// mask image
//
static char *maskim;
static long  fpi_maskim;


// extended mask image
//
static char *extmaskim;
static long  fpi_extmaskim;


static char *outmodeC;
static long  fpi_outmodeC;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".inmodeC",
        "input modes",
        "inmodeC",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inmodeC,
        &fpi_inmodeC
    },
    {
        CLIARG_IMG,
        ".maskim",
        "input mask",
        "maskim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &maskim,
        &fpi_maskim
    },
    {
        CLIARG_IMG,
        ".extmaskim",
        "extended input mask",
        "extmaskim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &extmaskim,
        &fpi_extmaskim
    },
    {
        CLIARG_IMG,
        ".outmodeC",
        "output modes",
        "outmodeC",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outmodeC,
        &fpi_outmodeC
    }
};




// Optional custom configuration setup. comptbuff
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

    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "maskextrapolate", "mask and extrapolate modes", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imginmodeC = mkIMGID_from_name(inmodeC);
    resolveIMGID(&imginmodeC, ERRMODE_ABORT);
    uint32_t xsize = imginmodeC.md->size[0];
    uint32_t ysize = imginmodeC.md->size[1];
    uint64_t xysize = xsize;
    xysize *= ysize;
    uint32_t NBmodes = imginmodeC.md->size[2];
    printf("%u modes\n", NBmodes);

    IMGID imgmask = mkIMGID_from_name(maskim);
    resolveIMGID(&imgmask, ERRMODE_ABORT);

    IMGID imgextmask = mkIMGID_from_name(extmaskim);
    resolveIMGID(&imgextmask, ERRMODE_ABORT);



    IMGID imgoutmoudeC = makeIMGID_3D(outmodeC, xsize, ysize, NBmodes);
    createimagefromIMGID(&imgoutmoudeC);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        for(uint32_t mi=0; mi<NBmodes; mi++)
        {
            for(uint64_t ii=0; ii<xysize; ii++)
            {
                imgoutmoudeC.im->array.F[xysize*mi + ii] = imginmodeC.im->array.F[xysize*mi + ii] * imgmask.im->array.F[ii];
            }
        }
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__maskextrapolate()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
