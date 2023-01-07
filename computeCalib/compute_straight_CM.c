/**
 * @file compute_straight_CM.c
 *
 */


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

/*
#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
*/

#include <cblas.h>
#include <lapacke.h>


#include "linopt_imtools/compute_SVDpseudoInverse.h"

#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif




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


    list_image_ID();


    imageID ID;

    load_fits(RMmodesDM, "RMmodesDM", LOADFITS_ERRMODE_WARNING, &ID);
    IMGID imgRMDM = makesetIMGID("RMmodesDM", ID);

    load_fits(RMmodesWFS, "RMmodesWFS", LOADFITS_ERRMODE_WARNING, &ID);
    IMGID imgRMWFS = makesetIMGID("RMmodesWFS", ID);



    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {








#ifdef HAVE_CUDA
printf("USING CUDA\n");
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
printf("USING CPU\n");
            linopt_compute_SVDpseudoInverse("RMmodesWFS",
                                            "controlM",
                                            *svdlim,
                                            10000,
                                            "VTmat",
                                            NULL);
#ifdef HAVE_CUDA
        }
#endif





        ID = image_ID("VTmat");
        IMGID imgVT = makesetIMGID("VTmat", ID);

        printf("Number of modes    : %d\n", imgRMDM.md->size[2]);
        printf("Number of DM act   : %d x %d\n", imgRMDM.md->size[0], imgRMDM.md->size[1]);
        printf("Number of WFS pix  : %d x %d\n", imgRMWFS.md->size[0], imgRMWFS.md->size[1]);

        int nbmode = imgRMDM.md->size[2];
        int nbact = imgRMDM.md->size[0] * imgRMDM.md->size[1];
        int nbwfspix = imgRMWFS.md->size[0] * imgRMWFS.md->size[1];



         EXECUTE_SYSTEM_COMMAND("mkdir -p mkmodestmp");




        {
        // create ATA
        IMGID imgATA = makeIMGID_2D("ATA", nbmode, nbmode);
        createimagefromIMGID(&imgATA);
        cblas_sgemm(CblasColMajor, CblasTrans, CblasNoTrans,
                nbmode, nbmode, nbwfspix, 1.0, imgRMWFS.im->array.F, nbwfspix, imgRMWFS.im->array.F, nbwfspix, 0.0, imgATA.im->array.F, nbmode);
        save_fits("ATA", "./mkmodestmp/ATA.fits");

        float *d = (float*) malloc(sizeof(float)*nbmode);
        float *e = (float*) malloc(sizeof(float)*nbmode);
        float *t = (float*) malloc(sizeof(float)*nbmode);

        LAPACKE_ssytrd(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, d, e, t);

        // Assemble Q matrix
        LAPACKE_sorgtr(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, t );


       // create eigenvectors array
        IMGID imgevec = makeIMGID_2D("eigenvec", nbmode, nbmode);
        createimagefromIMGID(&imgevec);


        if(0)
        {
        int evfound;

        // create eigenvalues array
        IMGID imgeval = makeIMGID_2D("eigenval", nbmode, 1);
        createimagefromIMGID(&imgeval);

        int * isuppz = (int*) malloc(sizeof(int)*2*nbmode);
        lapack_logical tryrac = 0;
        LAPACKE_sstemr(LAPACK_COL_MAJOR, 'V', 'I', nbmode, d, e, 0.0, 0.0, nbmode-10, nbmode, &evfound, imgeval.im->array.F, imgevec.im->array.F, nbmode, nbmode, isuppz, &tryrac);

        printf("Found %d eigenvalues\n", evfound);
        }

        memcpy(imgevec.im->array.F, imgATA.im->array.F, sizeof(float)*nbmode*nbmode);
        LAPACKE_ssteqr(LAPACK_COL_MAJOR, 'V', nbmode, d, e, imgevec.im->array.F, nbmode);


        free(d);
        free(e);
        free(t);

        save_fits("eigenvec", "./mkmodestmp/eigenvec.fits");
        }





        // create CM WFS
        IMGID imgCMWFS = makeIMGID_3D("CMmodesWFS", imgRMWFS.md->size[0], imgRMWFS.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMWFS);

        // Compute WFS modes
        // Multiply RMmodesWFS by Vmat
        //

       cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans,
                nbwfspix, nbmode, nbmode, 1.0, imgRMWFS.im->array.F, nbwfspix, imgVT.im->array.F, nbmode, 0.0, imgCMWFS.im->array.F, nbwfspix);



        // create CM DM
        IMGID imgCMDM = makeIMGID_3D("CMmodesDM", imgRMDM.md->size[0], imgRMDM.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMDM);

        // Compute DM modes
        // Multiply RMmodesDM by Vmat
        //

       cblas_sgemm (CblasColMajor, CblasNoTrans, CblasTrans,
                nbact, nbmode, nbmode, 1.0, imgRMDM.im->array.F, nbact, imgVT.im->array.F, nbmode, 0.0, imgCMDM.im->array.F, nbact);



       

        {
            // measure norm of moces in DM and WFS space
        //
        FILE *fp = fopen("mkmodestmp/mode_norm.txt", "w");
        for(int mi=0; mi<nbmode; mi++)
        {
            char *ptr;

            ptr = (void*) imgCMDM.im->array.F;
            ptr += sizeof(float)*mi*nbact;
            float n2cmDM = cblas_snrm2(nbact, (float*) ptr, 1);

            ptr = (void*) imgCMWFS.im->array.F;
            ptr += sizeof(float)*mi*nbwfspix;
            float n2cmWFS = cblas_snrm2(nbwfspix, (float*) ptr, 1);

            fprintf(fp, "%4d    %20g    %20g\n", mi, n2cmDM, n2cmWFS);
        }
        fclose(fp);
        }



        save_fits("VTmat", "./mkmodestmp/VTmat.fits");
        save_fits("CMmodesWFS", "./mkmodestmp/CMmodesWFS.fits");
        save_fits("CMmodesDM", "./mkmodestmp/CMmodesDM.fits");
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
