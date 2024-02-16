/**
 * @file generateRMWFS.c
 *
 */


#include "CommandLineInterface/CLIcore.h"




// zonal WFS response
//
static char *zrespWFS;
static long  fpi_zrespWFS;


static char *DMmodesC;
static long  fpi_DMmodesC;


static char *outWFSmodesC;
static long  fpi_outWFSmodesC;

static CLICMDARGDEF farg[] =
{
    {
        // zonal RM WFS
        CLIARG_IMG,
        ".zrespWFS",
        "input zonal response matrix",
        "zrespM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &zrespWFS,
        &fpi_zrespWFS
    },
    {
        CLIARG_IMG,
        ".DMmodesC",
        "input DM modes",
        "DMmodesC",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMmodesC,
        &fpi_DMmodesC
    },
    {
        CLIARG_STR,
        ".outWFSmodesC",
        "output WFS modes",
        "modesWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outWFSmodesC,
        &fpi_outWFSmodesC
    }
};




// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_zrespWFS].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_DMmodesC].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
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
    "generateRMWFS", "generate RM WFS modes", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imgzRM = mkIMGID_from_name(zrespWFS);
    resolveIMGID(&imgzRM, ERRMODE_ABORT);
    uint32_t wfsxsize = imgzRM.md->size[0];
    uint32_t wfsysize = imgzRM.md->size[1];
    uint64_t wfssize = wfsxsize;
    wfssize *= wfsysize;
    printf("wfssize = %lu\n", wfssize);


    IMGID imDMmodesC = mkIMGID_from_name(DMmodesC);
    resolveIMGID(&imDMmodesC, ERRMODE_ABORT);
    uint32_t dmxsize = imDMmodesC.md->size[0];
    uint32_t dmysize = imDMmodesC.md->size[1];
    uint64_t dmsize = dmxsize;
    dmsize *= dmysize;
    printf("dmsize = %lu\n", dmsize);

    uint32_t NBmodes = imDMmodesC.md->size[2];
    printf("%u modes\n", NBmodes);


    IMGID imgoutWFSc = makeIMGID_3D(outWFSmodesC, wfsxsize, wfsysize, NBmodes);
    createimagefromIMGID(&imgoutWFSc);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        for(uint32_t mode=0; mode < NBmodes; mode++)
        {
            printf(".");
            fflush(stdout);

            for(uint64_t iidm=0; iidm < dmsize; iidm++)
            {
                for(uint64_t iiwfs=0; iiwfs < wfssize; iiwfs++)
                {
                    imgoutWFSc.im->array.F[wfssize * mode + iiwfs] +=
                    imDMmodesC.im->array.F[dmsize*mode + iidm] * imgzRM.im->array.F[wfssize * iidm + iiwfs];
                }
            }
        }
        printf("\n");
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__generateRMWFS()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
