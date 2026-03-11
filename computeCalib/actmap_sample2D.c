/**
 * @file actmap_sample2D.c
 * @brief Actmap sample2d module
 */

/**
 * @file CLIADDCMD_AOloopControl_computeCalib__sample2D
 *
 */


#include "CLIcore/CLIcore.h"




static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "sample2DWF",
    .cmdkey      = "sample2DWF",
    .description = "sample 2D WF to act pos"
};

static char inWF2D[FUNCTION_PARAMETER_STRMAXLEN];
static char map2D[FUNCTION_PARAMETER_STRMAXLEN];
static char outWF1D[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X) \
    X(".inwf2D", inWF2D, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input 2D wavefront") \
    X(".mapfile", map2D, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "mapping file, can be read from mapcoord2D.txt") \
    X(".outWF1D", outWF1D, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output WF 1D")

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
        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".inwf2D")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        //data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".mapfile")].fpflag |=
        //    FPFLAG_STREAM_RUN_REQUIRED;
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



static IMGID load_actmapcoord2D(
    char *fname,
    char *outim
)
{
    FILE *fp = NULL;

    fp = fopen(fname, "r");
    if(fp == NULL)
    {
        printf("ERROR: cannot load file %s\n", fname);
        exit(0);
    }


    // count number of lines
    long actindex;
    float xcoord;
    float ycoord;
    long NBact = 0; // counter
    while(fscanf(fp, "%ld %f %f\n", &actindex, &xcoord, &ycoord) == 3)
    {
        NBact++;
    }
    fclose(fp);



    IMGID imgout = imgid_make_from_name_2D(outim, NBact, 2);
    createimagefromIMGID(&imgout);


    fp = fopen(fname, "r");
    if(fp == NULL)
    {
        printf("ERROR: cannot load file %s\n", fname);
        exit(0);
    }
    for(uint32_t act = 0; act < NBact; act++)
    {

        int ret = fscanf(fp, "%ld %f %f\n", &actindex, &xcoord, &ycoord);
        if(ret != 3)
        {
            printf("ERROR reading file %s\n", fname);
            exit(0);
        }

        imgout.im->array.F[act * 2] = xcoord;
        imgout.im->array.F[act * 2 + 1] = ycoord;
    }
    fclose(fp);


    return imgout;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imgWF2D = imgid_make_from_name(inWF2D);
    resolveIMGID(&imgWF2D, ERRMODE_ABORT, data.core.image, data.core.NB_MAX_IMAGE);
    uint32_t wfxsize = imgWF2D.md->size[0];
    uint32_t wfysize = imgWF2D.md->size[1];
    uint64_t wfsize = wfxsize;
    wfsize *= wfysize;
    printf("wfsize = %lu\n", wfsize);


    IMGID imgmap2D = imgid_make_from_name(map2D);
    resolveIMGID(&imgmap2D, ERRMODE_WARN, data.core.image, data.core.NB_MAX_IMAGE);
    if(imgmap2D.ID == -1)
    {
        imgmap2D = load_actmapcoord2D("mapcoord2D.txt", map2D);
    }
    uint32_t mapsize = imgmap2D.md->size[0];
    // mapsizedim should be 2: x and y coord
    // uint32_t mapsizedim = imgmap2D.md->size[1];
    printf("mapsize = %u\n", mapsize);




    uint32_t NBslice = imgWF2D.md->size[2];
    printf("%u slice\n", NBslice);


    IMGID imgoutWF1D = imgid_make_from_name_3D(outWF1D, mapsize, 1, NBslice);
    createimagefromIMGID(&imgoutWF1D);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    float xcentf = 0.5 * wfxsize;
    float ycentf = 0.5 * wfysize;
    float radf   = 0.25 * (wfxsize + wfysize);


    // Build mapping
    //
    long *iiact = (long *) malloc(sizeof(long) * mapsize);
    long *jjact = (long *) malloc(sizeof(long) * mapsize);

    for(uint32_t act = 0; act < mapsize; act++)
    {
        // actuator coordinates
        // relative to beam center, radius = 1
        //
        float xact = imgmap2D.im->array.F[act * 2];
        float yact = imgmap2D.im->array.F[act * 2 + 1];

        iiact[act] = (long)(xcentf + radf * xact);
        jjact[act] = (long)(ycentf + radf * yact);

        if(iiact[act] < 0)
        {
            iiact[act] = 0;
        }
        if(iiact[act] > wfxsize - 1)
        {
            iiact[act] = wfxsize - 1;
        }

        if(jjact[act] < 0)
        {
            jjact[act] = 0;
        }
        if(jjact[act] > wfysize - 1)
        {
            jjact[act] = wfysize - 1;
        }
    }





    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        for(uint32_t slice = 0; slice < NBslice; slice++)
        {
            for(uint32_t act = 0; act < mapsize; act++)
            {
                imgoutWF1D.im->array.F[slice * mapsize + act] =
                imgWF2D.im->array.F[slice * wfsize + jjact[act] * wfxsize + iiact[act]];
            }
        }
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    free(iiact);
    free(jjact);

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
CLIADDCMD_AOloopControl_computeCalib__sample2D()
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
