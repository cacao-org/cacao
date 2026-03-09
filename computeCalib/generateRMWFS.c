/**
 * @file generateRMWFS.c
 *
 */


#include "CommandLineInterface/CLIcore.h"




static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "generateRMWFS",
    .cmdkey      = "generateRMWFS",
    .description = "generate RM WFS modes"
};

// zonal WFS response
//
static char *zrespWFS;
static char *DMmodesC;
static char *outWFSmodesC;


#define FPS_PARAMS(X) \
    X(".zrespWFS", &zrespWFS, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input zonal response matrix") \
    X(".DMmodesC", &DMmodesC, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input DM modes") \
    X(".outWFSmodesC", &outWFSmodesC, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output WFS modes")

static FPS_CLI_BINDING my_bindings[] = {
    FPS_PARAMS(FPS_X_BINDING)
};

static const int nb_bindings = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

#ifdef FPS_STANDALONE
CLICMDDATA CLIcmddata = {
#else
static CLICMDDATA CLIcmddata = {
#endif
    "",
    "",
    CLICMD_FIELDS_DEFAULTS
};

static CMDSETTINGS default_cmdsettings = {0};

static __attribute__((constructor))
void init_cmdsettings(void)
{
    strncpy(CLIcmddata.key,
            FPS_app_info.cmdkey,
            sizeof(CLIcmddata.key) - 1);
    strncpy(CLIcmddata.description,
            FPS_app_info.description,
            sizeof(CLIcmddata.description) - 1);
    if (CLIcmddata.cmdsettings == NULL) {
        CLIcmddata.cmdsettings =
            &default_cmdsettings;
    }
}


// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".zrespWFS")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMmodesC")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}



// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.core.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}





// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imgzRM = imgid_make_from_name(zrespWFS);
    resolveIMGID(&imgzRM, ERRMODE_ABORT, data.core.image, data.core.NB_MAX_IMAGE);
    uint32_t wfsxsize = imgzRM.md->size[0];
    uint32_t wfsysize = imgzRM.md->size[1];
    uint64_t wfssize = wfsxsize;
    wfssize *= wfsysize;
    printf("wfssize = %lu\n", wfssize);


    IMGID imDMmodesC = imgid_make_from_name(DMmodesC);
    resolveIMGID(&imDMmodesC, ERRMODE_ABORT, data.core.image, data.core.NB_MAX_IMAGE);
    uint32_t dmxsize = imDMmodesC.md->size[0];
    uint32_t dmysize = imDMmodesC.md->size[1];
    uint64_t dmsize = dmxsize;
    dmsize *= dmysize;
    printf("dmsize = %lu\n", dmsize);

    uint32_t NBmodes = imDMmodesC.md->size[2];
    printf("%u modes\n", NBmodes);


    IMGID imgoutWFSc = imgid_make_from_name_3D(outWFSmodesC, wfsxsize, wfsysize, NBmodes);
    createimagefromIMGID(&imgoutWFSc);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        for(uint32_t mode = 0; mode < NBmodes; mode++)
        {
            printf(".");
            fflush(stdout);

            for(uint64_t iidm = 0; iidm < dmsize; iidm++)
            {
                for(uint64_t iiwfs = 0; iiwfs < wfssize; iiwfs++)
                {
                    imgoutWFSc.im->array.F[wfssize * mode + iiwfs] +=
                    imDMmodesC.im->array.F[dmsize * mode + iidm] * imgzRM.im->array.F[wfssize * iidm
                            + iiwfs];
                }
            }
        }
        printf("\n");
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__generateRMWFS()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(
    FPS_app_info,
    FPS_PARAMS,
    compute_function,

    customCONFcheck)
#endif
