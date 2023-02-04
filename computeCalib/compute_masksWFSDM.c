/**
 * @file compute_straight_CM.c
 *
 */


#include "CommandLineInterface/CLIcore.h"


#include "COREMOD_arith/COREMOD_arith.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "info/info.h"

static char *zrespWFS;
static long  fpi_zrespWFS;

static uint32_t *dmxsize;
static long fpi_dmxsize;

static uint32_t *dmysize;
static long fpi_dmysize;



static float *dmmaskperc0;
static long fpi_dmmaskperc0;

static float *dmmaskcoeff0;
static long fpi_dmmaskcoeff0;

static float *dmmaskperc1;
static long fpi_dmmaskperc1;

static float *dmmaskcoeff1;
static long fpi_dmmaskcoeff1;




static float *wfsmaskperc0;
static long fpi_wfsmaskperc0;

static float *wfsmaskcoeff0;
static long fpi_wfsmaskcoeff0;

static float *wfsmaskperc1;
static long fpi_wfsmaskperc1;

static float *wfsmaskcoeff1;
static long fpi_wfsmaskcoeff1;





static CLICMDARGDEF farg[] =
{
    {
        // input zonal WFS RM
        CLIARG_STR,
        ".zrespM",
        "input zonal WFS RM",
        "zrespWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &zrespWFS,
        &fpi_zrespWFS
    },
    {
        // DM x size
        CLIARG_UINT32,
        ".dmxsize",
        "DM x size",
        "50",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmxsize,
        &fpi_dmxsize
    },
    {
        // DM y size
        CLIARG_UINT32,
        ".dmysize",
        "DM y size",
        "50",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmysize,
        &fpi_dmysize
    },
    {
        // DM mask - percentile 0
        CLIARG_FLOAT32,
        ".dmmask.perc0",
        "DM mask percentile 0",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dmmaskperc0,
        &fpi_dmmaskperc0
    },
    {
        // DM mask - coefficient 0
        CLIARG_FLOAT32,
        ".dmmask.coeff0",
        "DM mask coefficient 0",
        "0.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dmmaskcoeff0,
        &fpi_dmmaskcoeff0
    },
    {
        // DM mask - percentile 0
        CLIARG_FLOAT32,
        ".dmmask.perc1",
        "DM mask percentile 1",
        "0.8",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dmmaskperc1,
        &fpi_dmmaskperc1
    },
    {
        // DM mask - coefficient 1
        CLIARG_FLOAT32,
        ".dmmask.coeff1",
        "DM mask coefficient 1",
        "0.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dmmaskcoeff1,
        &fpi_dmmaskcoeff1
    },
    {
        // WFS mask - percentile 0
        CLIARG_FLOAT32,
        ".wfsmask.perc0",
        "WFS mask percentile 0",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsmaskperc0,
        &fpi_wfsmaskperc0
    },
    {
        // WFS mask - coefficient 0
        CLIARG_FLOAT32,
        ".wfsmask.coeff0",
        "WFS mask coefficient 0",
        "0.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsmaskcoeff0,
        &fpi_wfsmaskcoeff0
    },
    {
        // WFS mask - percentile 0
        CLIARG_FLOAT32,
        ".wfsmask.perc1",
        "WFS mask percentile 1",
        "0.8",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsmaskperc1,
        &fpi_wfsmaskperc1
    },
    {
        // WFS mask - coefficient 1
        CLIARG_FLOAT32,
        ".wfsmask.coeff1",
        "WFS mask coefficient 1",
        "0.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsmaskcoeff1,
        &fpi_wfsmaskcoeff1
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
    "compmasksWFSDM", "compute WFS and DM masks", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    list_image_ID();


    imageID IDzrm = image_ID(zrespWFS);
    printf("IDzrm = %ld\n", IDzrm);
    uint32_t sizexWFS = data.image[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = data.image[IDzrm].md[0].size[1];
    uint64_t sizeWFS = sizexWFS;
    sizeWFS *= sizeyWFS;

    uint32_t NBpoke   = data.image[IDzrm].md[0].size[2];


    imageID IDWFSmap;
    create_2Dimage_ID("wfsmap", sizexWFS, sizeyWFS, &IDWFSmap);

    imageID IDDMmap;
    create_2Dimage_ID("dmmap", *dmxsize, *dmysize, &IDDMmap);

    imageID IDWFSmask;
    create_2Dimage_ID("wfsmask", sizexWFS, sizeyWFS, &IDWFSmask);

    imageID IDDMmask;
    create_2Dimage_ID("dmmask", *dmxsize, *dmysize, &IDDMmask);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        printf("Preparing DM map ... ");
        fflush(stdout);
        for(int poke = 0; poke < NBpoke; poke++)
        {
            double rms = 0.0;
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                double tmpv = data.image[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            data.image[IDDMmap].array.F[poke] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing WFS map ... ");
        fflush(stdout);
        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            double rms = 0.0;
            for(int poke = 0; poke < NBpoke; poke++)
            {
                double tmpv = data.image[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            data.image[IDWFSmap].array.F[ii] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing DM mask ... ");
        fflush(stdout);

        // pre-filtering
        // gauss_filter(DMmap_name, "dmmapg", 5.0, 8);
        // IDDMmap1 = image_ID("dmmapg");

        // (map/map1)*pow(map,0.25)

        // DMmask: select pixels
        double lim0 = (*dmmaskcoeff0) * img_percentile("dmmap", (*dmmaskperc0));

        imageID IDtmp;
        create_2Dimage_ID("_tmpdmmap", (*dmxsize), (*dmysize), &IDtmp);
        for(uint64_t ii = 0; ii < (*dmxsize) * (*dmysize); ii++)
        {
            data.image[IDtmp].array.F[ii] = data.image[IDDMmap].array.F[ii] - lim0;
        }
        double lim = (*dmmaskcoeff1) * img_percentile("_tmpdmmap", (*dmmaskperc1));

        for(int poke = 0; poke < NBpoke; poke++)
        {
            if(data.image[IDtmp].array.F[poke] < lim)
            {
                data.image[IDDMmask].array.F[poke] = 0.0;
            }
            else
            {
                data.image[IDDMmask].array.F[poke] = 1.0;
            }
        }
        delete_image_ID("_tmpdmmap", DELETE_IMAGE_ERRMODE_WARNING);
        printf("done\n");
        fflush(stdout);

        // WFSmask : select pixels
        printf("Preparing WFS mask ... ");
        fflush(stdout);

        lim0 = (*wfsmaskcoeff0) * img_percentile("wfsmap", (*wfsmaskperc0));
        create_2Dimage_ID("_tmpwfsmap", sizexWFS, sizeyWFS, &IDtmp);
        for(uint64_t ii = 0; ii < sizexWFS * sizeyWFS; ii++)
        {
            data.image[IDtmp].array.F[ii] = data.image[IDWFSmap].array.F[ii] - lim0;
        }
        lim = (*wfsmaskcoeff1) * img_percentile("_tmpwfsmap", (*wfsmaskperc1));

        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            if(data.image[IDWFSmap].array.F[ii] < lim)
            {
                data.image[IDWFSmask].array.F[ii] = 0.0;
            }
            else
            {
                data.image[IDWFSmask].array.F[ii] = 1.0;
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




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
