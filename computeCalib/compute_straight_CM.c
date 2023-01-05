/**
 * @file compute_straight_CM.c
 *
 */


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>

#include "linopt_imtools/compute_SVDpseudoInverse.h"
P
#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif




static uint64_t *AOloopindex;

static char *RMmodesDM;
static long  fpi_RMmodesDM;

static char *RMmodesWFS;
static long  fpi_RMmodesWFS;

static char *CMmodesDM;
static long  fpi_CMmodesDM;

static char *CMmodesWFS;
static long  fpi_CMmodesWFS;

static char *controlM;
static long  fpi_controlM;

static float *svdlim;
static long   fpi_svdlim;

static int32_t *GPUdevice;
static long     fpi_GPUdevice;



static CLICMDARGDEF farg[] =
{
    {
        // AO loop index - used for automatic naming of streams aolX_
        CLIARG_UINT64,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {
        // input RM : DM modes
        CLIARG_FILENAME,
        ".RMmodesDM",
        "input response matrix DM modes",
        "RMmodesDM.fits",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesDM,
        &fpi_RMmodesDM
    },
    {
        // input RM : WFS modes
        CLIARG_FILENAME,
        ".RMmodesWFS",
        "input response matrix WFS modes",
        "RMmodesWFS.fits",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesWFS,
        &fpi_RMmodesWFS
    },
    {
        // output CM : DM modes
        CLIARG_FILENAME,
        ".CMmodesDM",
        "output control matrix DM modes",
        "CMmodesDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &CMmodesDM,
        &fpi_CMmodesDM
    },
    {
        // output CM : WFS modes
        CLIARG_FILENAME,
        ".CMmodesWFS",
        "output control matrix WFS modes",
        "CMmodesWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &CMmodesWFS,
        &fpi_CMmodesWFS
    },
    {
        // Singular Value Decomposition limit
        CLIARG_FLOAT32,
        ".svdlim",
        "SVD limit",
        "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &svdlim,
        &fpi_svdlim
    },
    {
        // using GPU (-1 : no GPU, otherwise GPU device)
        CLIARG_INT32,
        ".GPUdevive",
        "GPU device",
        "-1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &GPUdevice,
        &fpi_GPUdevice
    }
};




// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_RMmodesDM].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[fpi_RMmodesWFS].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;
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
    "compsCM", "compute straight control matrix", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();



    load_fits(RMmodesDM, "RMmodesDM", LOADFITS_ERRMODE_WARNING, NULL);
    load_fits(RMmodesDM, "RMmodesWFS", LOADFITS_ERRMODE_WARNING, NULL);


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {


#ifdef HAVE_CUDA
        if(*GPUdevice >= 0)
        {
            CUDACOMP_magma_compute_SVDpseudoInverse("RMmodesWFS",
                                                    "controlM",
                                                    *svdlim,
                                                    100000,
                                                    "VTmat",
                                                    0,
                                                    0,
                                                    64,
                                                    *GPUdevice, // GPU device
                                                    NULL);
        }
        else
        {
#endif
            linopt_compute_SVDpseudoInverse("RMmodesWFS",
                                            "controlM",
                                            *svdlim,
                                            10000,
                                            "VTmat",
                                            NULL);
#ifdef HAVE_CUDA
        }
#endif


        list_image_ID();

        // save_fits("VTmat", "./mkmodestmp/VTmat.fits");
        //delete_image_ID("VTmat", DELETE_IMAGE_ERRMODE_WARNING);

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__compsCM()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
