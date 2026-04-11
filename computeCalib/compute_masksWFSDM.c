/**
 * @file compute_masksWFSDM.c
 * @brief Compute maskswfsdm module
 */

/**
 * @file compute_straight_CM.c
 *
 */


#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "COREMOD_arith/COREMOD_arith.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "info/info.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "compmasksWFSDM",
    .cmdkey      = "compmasksWFSDM",
    .description = "compute WFS and DM masks"
};

static char zrespWFS[FUNCTION_PARAMETER_STRMAXLEN];
static uint32_t dmxsize;
static uint32_t dmysize;

static float dmmaskperc0;
static float dmmaskcoeff0;
static float dmmaskperc1;
static float dmmaskcoeff1;

static float wfsmaskperc0;
static float wfsmaskcoeff0;
static float wfsmaskperc1;
static float wfsmaskcoeff1;

#define FPS_PARAMS(X) \
    X(".zrespM", zrespWFS, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input zonal WFS RM") \
    X(".dmxsize", &dmxsize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "DM x size") \
    X(".dmysize", &dmysize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "DM y size") \
    X(".dmmask.perc0", &dmmaskperc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask percentile 0") \
    X(".dmmask.coeff0", &dmmaskcoeff0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask coefficient 0") \
    X(".dmmask.perc1", &dmmaskperc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask percentile 1") \
    X(".dmmask.coeff1", &dmmaskcoeff1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask coefficient 1") \
    X(".wfsmask.perc0", &wfsmaskperc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask percentile 0") \
    X(".wfsmask.coeff0", &wfsmaskcoeff0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask coefficient 0") \
    X(".wfsmask.perc1", &wfsmaskperc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask percentile 1") \
    X(".wfsmask.coeff1", &wfsmaskcoeff1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask coefficient 1")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup() __attribute__((unused))
{
    if(data.core.fpsptr != NULL)
    {

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
static errno_t help_function() __attribute__((unused))
{


    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    list_image_ID();


    imageID IDzrm = image_ID(zrespWFS, data.core.image, data.core.NB_MAX_IMAGE);
    printf("IDzrm = %ld\n", IDzrm);
    uint32_t sizexWFS = data.core.image[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = data.core.image[IDzrm].md[0].size[1];
    uint64_t sizeWFS = sizexWFS;
    sizeWFS *= sizeyWFS;

    uint32_t NBpoke   = data.core.image[IDzrm].md[0].size[2];


    imageID IDWFSmap;
    create_2Dimage_ID("wfsmap", sizexWFS, sizeyWFS, &IDWFSmap);

    imageID IDDMmap;
    create_2Dimage_ID("dmmap", dmxsize, dmysize, &IDDMmap);

    imageID IDWFSmask;
    create_2Dimage_ID("wfsmask", sizexWFS, sizeyWFS, &IDWFSmask);

    imageID IDDMmask;
    create_2Dimage_ID("dmmask", dmxsize, dmysize, &IDDMmask);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        printf("Preparing DM map ... ");
        fflush(stdout);
        for(uint32_t poke = 0; poke < NBpoke; poke++)
        {
            double rms = 0.0;
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                double tmpv = data.core.image[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            data.core.image[IDDMmap].array.F[poke] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing WFS map ... ");
        fflush(stdout);
        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            double rms = 0.0;
            for(uint32_t poke = 0; poke < NBpoke; poke++)
            {
                double tmpv = data.core.image[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            data.core.image[IDWFSmap].array.F[ii] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing DM mask ... ");
        fflush(stdout);

        // pre-filtering
        // gauss_filter(DMmap_name, "dmmapg", 5.0, 8);
        // IDDMmap1 = image_ID("dmmapg", data.core.image, data.core.NB_MAX_IMAGE);

        // (map/map1)*pow(map,0.25)

        // DMmask: select pixels
        double lim0 = dmmaskcoeff0 * img_percentile("dmmap", dmmaskperc0);

        imageID IDtmp;
        create_2Dimage_ID("_tmpdmmap", dmxsize, dmysize, &IDtmp);
        for(uint64_t ii = 0; ii < dmxsize * dmysize; ii++)
        {
            data.core.image[IDtmp].array.F[ii] = data.core.image[IDDMmap].array.F[ii] - lim0;
        }
        double lim = dmmaskcoeff1 * img_percentile("_tmpdmmap", dmmaskperc1);

        for(uint32_t poke = 0; poke < NBpoke; poke++)
        {
            if(data.core.image[IDtmp].array.F[poke] < lim)
            {
                data.core.image[IDDMmask].array.F[poke] = 0.0;
            }
            else
            {
                data.core.image[IDDMmask].array.F[poke] = 1.0;
            }
        }
        delete_image_ID("_tmpdmmap", DELETE_IMAGE_ERRMODE_WARNING);
        printf("done\n");
        fflush(stdout);

        // WFSmask : select pixels
        printf("Preparing WFS mask ... ");
        fflush(stdout);

        lim0 = wfsmaskcoeff0 * img_percentile("wfsmap", wfsmaskperc0);
        create_2Dimage_ID("_tmpwfsmap", sizexWFS, sizeyWFS, &IDtmp);
        for(uint64_t ii = 0; ii < sizexWFS *sizeyWFS; ii++)
        {
            data.core.image[IDtmp].array.F[ii] = data.core.image[IDWFSmap].array.F[ii] - lim0;
        }
        lim = wfsmaskcoeff1 * img_percentile("_tmpwfsmap", wfsmaskperc1);

        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            if(data.core.image[IDWFSmap].array.F[ii] < lim)
            {
                data.core.image[IDWFSmask].array.F[ii] = 0.0;
            }
            else
            {
                data.core.image[IDWFSmask].array.F[ii] = 1.0;
            }
        }
        delete_image_ID("_tmpwfsmap", DELETE_IMAGE_ERRMODE_WARNING);
        printf("done\n");
        fflush(stdout);


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
CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM()
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
