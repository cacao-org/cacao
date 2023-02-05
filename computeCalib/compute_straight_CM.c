/**
 * @file compute_straight_CM.c
 *
 */


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

#include "CommandLineInterface/timeutils.h"

/*
#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
*/


// Use MKL if available
// Otherwise use openBLAS
//
#ifdef HAVE_MKL
#include "mkl.h"
#define BLASLIB "IntelMKL"
#else
#ifdef HAVE_OPENBLAS
#include <cblas.h>
#include <lapacke.h>
#define BLASLIB "OpenBLAS"
#endif
#endif



#include "linopt_imtools/compute_SVDpseudoInverse.h"

#ifdef HAVE_CUDA
#include <cublas_v2.h>
#endif

/*
#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif
*/

// CPU mode: Use MKL if available
// Otherwise use openBLAS
//
#ifdef HAVE_MKL
#include "mkl.h"
#define BLASLIB "IntelMKL"
#else
#ifdef HAVE_OPENBLAS
#include <cblas.h>
#include <lapacke.h>
#define BLASLIB "OpenBLAS"
#endif
#endif




static char *RMmodesDMfname;
static long  fpi_RMmodesDMfname;

static char *RMmodesWFSfname;
static long  fpi_RMmodesWFSfname;

static char *CMmodesDMfname;
static long  fpi_CMmodesDMfname;

static char *CMmodesWFSfname;
static long  fpi_CMmodesWFSfname;

//static char *controlM;
//static long  fpi_controlM;

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
        (void **) &RMmodesDMfname,
        &fpi_RMmodesDMfname
    },
    {
        // input RM : WFS modes
        CLIARG_FILENAME,
        ".RMmodesWFS",
        "input response matrix WFS modes",
        "RMmodesWFS.fits",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesWFSfname,
        &fpi_RMmodesWFSfname
    },
    {
        // output CM : DM modes
        CLIARG_FILENAME,
        ".CMmodesDM",
        "output control matrix DM modes",
        "CMmodesDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &CMmodesDMfname,
        &fpi_CMmodesDMfname
    },
    {
        // output CM : WFS modes
        CLIARG_FILENAME,
        ".CMmodesWFS",
        "output control matrix WFS modes",
        "CMmodesWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &CMmodesWFSfname,
        &fpi_CMmodesWFSfname
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
        // using GPU (99 : no GPU, otherwise GPU device)
        CLIARG_INT32,
        ".GPUdevice",
        "GPU device, 99 for CPU",
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
        data.fpsptr->parray[fpi_RMmodesDMfname].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[fpi_RMmodesWFSfname].fpflag |=
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

    load_fits(RMmodesDMfname, "RMmodesDM", LOADFITS_ERRMODE_WARNING, &ID);
    IMGID imgRMDM = makesetIMGID("RMmodesDM", ID);

    load_fits(RMmodesWFSfname, "RMmodesWFS", LOADFITS_ERRMODE_WARNING, &ID);
    IMGID imgRMWFS = makesetIMGID("RMmodesWFS", ID);


    struct timespec t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {



#ifdef HAVE_OPENBLAS
        printf("OpenBLASS  YES\n");
#else
        printf("OpenBLASS  NO\n");
#endif

#ifdef HAVE_MKL
        printf("MKL        YES\n");
#else
        printf("MKL        NO\n");
#endif


#ifdef HAVE_CUDA
        printf("CUDA       YES\n");
#else
        printf("CUDA       NO\n");
#endif






        /*
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

                    clock_gettime(CLOCK_REALTIME, &t0);
                    linopt_compute_SVDpseudoInverse("RMmodesWFS",
                                                    "controlM",
                                                    *svdlim,
                                                    10000,
                                                    "VTmat",
                                                    NULL);
                    clock_gettime(CLOCK_REALTIME, &t1);
        #ifdef HAVE_CUDA
                }
        #endif
        */




        ID = image_ID("VTmat");
        //IMGID imgVT = makesetIMGID("VTmat", ID);

        printf("Number of modes    : %d\n", imgRMDM.md->size[2]);
        printf("Number of DM act   : %d x %d\n", imgRMDM.md->size[0], imgRMDM.md->size[1]);
        printf("Number of WFS pix  : %d x %d\n", imgRMWFS.md->size[0], imgRMWFS.md->size[1]);

        int nbmode = imgRMDM.md->size[2];
        int nbact = imgRMDM.md->size[0] * imgRMDM.md->size[1];
        int nbwfspix = imgRMWFS.md->size[0] * imgRMWFS.md->size[1];



        EXECUTE_SYSTEM_COMMAND("mkdir -p mkmodestmp");




        // create eigenvectors array
        IMGID imgevec = makeIMGID_2D("eigenvec", nbmode, nbmode);
        createimagefromIMGID(&imgevec);

        // create eigenvalues array
        IMGID imgeval = makeIMGID_2D("eigenval", nbmode, 1);
        createimagefromIMGID(&imgeval);


        clock_gettime(CLOCK_REALTIME, &t0);


        {
            // create ATA
            IMGID imgATA = makeIMGID_2D("ATA", nbmode, nbmode);
            createimagefromIMGID(&imgATA);


#ifdef HAVE_CUDA
            {
                const float alf = 1;
                const float bet = 0;
                const float *alpha = &alf;
                const float *beta = &bet;

                float *d_RMWFS;
                cudaMalloc((void **)&d_RMWFS, imgRMWFS.md->nelement * sizeof(float));
                cudaMemcpy(d_RMWFS, imgRMWFS.im->array.F, imgRMWFS.md->nelement * sizeof(float), cudaMemcpyHostToDevice);

                float *d_ATA;
                cudaMalloc((void **)&d_ATA, imgATA.md->nelement * sizeof(float));
                //cudaMemcpy(d_RMWFS,imgRMWFS.im->array.F, imgRMWFS.md->nelement * sizeof(float), cudaMemcpyHostToDevice);

                // Create a handle for CUBLAS
                cublasHandle_t handle;
                cublasCreate(&handle);

                // Do the actual multiplication
                cublasSgemm(handle, CUBLAS_OP_T, CUBLAS_OP_N,
                            nbmode, nbmode, nbwfspix, alpha, d_RMWFS, nbwfspix, d_RMWFS, nbwfspix, beta, d_ATA, nbmode);

                // Destroy the handle
                cublasDestroy(handle);

                cudaMemcpy(imgATA.im->array.F, d_ATA, imgATA.md->nelement * sizeof(float), cudaMemcpyDeviceToHost);

                cudaFree(d_RMWFS);
                cudaFree(d_ATA);
            }
#elif
            cblas_sgemm(CblasColMajor, CblasTrans, CblasNoTrans,
                        nbmode, nbmode, nbwfspix, 1.0, imgRMWFS.im->array.F, nbwfspix, imgRMWFS.im->array.F, nbwfspix, 0.0, imgATA.im->array.F, nbmode);
#endif




            clock_gettime(CLOCK_REALTIME, &t1);
            //save_fits("ATA", "./mkmodestmp/ATA.fits");

            float *d = (float*) malloc(sizeof(float)*nbmode);
            float *e = (float*) malloc(sizeof(float)*nbmode);
            float *t = (float*) malloc(sizeof(float)*nbmode);

            LAPACKE_ssytrd(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, d, e, t);

            clock_gettime(CLOCK_REALTIME, &t2);

            // Assemble Q matrix
            LAPACKE_sorgtr(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, t );


            clock_gettime(CLOCK_REALTIME, &t3);


            if(0)
            {
                // compute some of the eigenvalues and eigenvectors
                //
                lapack_int evfound;


                lapack_int * isuppz = (lapack_int*) malloc(sizeof(lapack_int)*2*nbmode);
                lapack_logical tryrac = 0;
                LAPACKE_sstemr(LAPACK_COL_MAJOR, 'V', 'I', nbmode, d, e, 0.0, 0.0, nbmode-10,
                               nbmode, &evfound, imgeval.im->array.F, imgevec.im->array.F, nbmode, nbmode, isuppz, &tryrac);

                printf("Found %d eigenvalues\n", (int) evfound);
            }
            else
            {
                // compute all eigenvalues and eivenvectors
                //
                memcpy(imgevec.im->array.F, imgATA.im->array.F, sizeof(float)*nbmode*nbmode);
                LAPACKE_ssteqr(LAPACK_COL_MAJOR, 'V', nbmode, d, e, imgevec.im->array.F, nbmode);
                memcpy(imgeval.im->array.F, d, sizeof(float)*nbmode);
            }
            clock_gettime(CLOCK_REALTIME, &t4);

            free(d);
            free(e);
            free(t);

            //save_fits("eigenvec", "./mkmodestmp/eigenvec.fits");
        }





        // create CM WFS
        IMGID imgCMWFSall = makeIMGID_3D("CMmodesWFSall", imgRMWFS.md->size[0], imgRMWFS.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMWFSall);

        clock_gettime(CLOCK_REALTIME, &t5);

        // Compute WFS modes
        // Multiply RMmodesWFS by Vmat
        //
        cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans,
                     nbwfspix, nbmode, nbmode, 1.0, imgRMWFS.im->array.F, nbwfspix, imgevec.im->array.F, nbmode, 0.0, imgCMWFSall.im->array.F, nbwfspix);


        clock_gettime(CLOCK_REALTIME, &t6);

        // create CM DM
        IMGID imgCMDMall = makeIMGID_3D("CMmodesDMall", imgRMDM.md->size[0], imgRMDM.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMDMall);

        // Compute DM modes
        // Multiply RMmodesDM by Vmat
        //

        cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans,
                     nbact, nbmode, nbmode, 1.0, imgRMDM.im->array.F, nbact, imgevec.im->array.F, nbmode, 0.0, imgCMDMall.im->array.F, nbact);

        clock_gettime(CLOCK_REALTIME, &t7);


        // norm2 of WFS and DM modes
        float * n2cmWFS = (float *) malloc( sizeof(float) * nbmode);
        float * n2cmDM  = (float *) malloc( sizeof(float) * nbmode);

        {
            // measure norm of moces in DM and WFS space
            //
            FILE *fp = fopen("mkmodestmp/mode_norm.txt", "w");
            for(int mi=0; mi<nbmode; mi++)
            {
                char *ptr;

                ptr = (void*) imgCMDMall.im->array.F;
                ptr += sizeof(float)*mi*nbact;
                n2cmDM[mi] = cblas_snrm2(nbact, (float*) ptr, 1);

                ptr = (void*) imgCMWFSall.im->array.F;
                ptr += sizeof(float)*mi*nbwfspix;
                n2cmWFS[mi] = cblas_snrm2(nbwfspix, (float*) ptr, 1);

                fprintf(fp, "%4d    %20g    %20g   %20g\n", mi, n2cmDM[mi], n2cmWFS[mi], imgeval.im->array.F[mi]);
            }
            fclose(fp);
        }

        clock_gettime(CLOCK_REALTIME, &t8);


        // select modes
        float evalmax = imgeval.im->array.F[nbmode-1];
        int ecnt = 0;
        float evlim = *svdlim * *svdlim;
        {
            int mi = 0;
            while ( imgeval.im->array.F[mi] < evalmax * evlim )
            {
                mi ++;
            }
            ecnt = nbmode - mi;
            printf("Selected %d modes\n", ecnt);
        }

        // create CMWFS and CMDM
        // contains strongest (highest singular values) modes from CMWFSall
        //

        IMGID imgCMWFS = makeIMGID_3D("CMmodesWFS", imgRMWFS.md->size[0], imgRMWFS.md->size[1], ecnt);
        createimagefromIMGID(&imgCMWFS);

        IMGID imgCMDM = makeIMGID_3D("CMmodesDM", imgRMDM.md->size[0], imgRMDM.md->size[1], ecnt);
        createimagefromIMGID(&imgCMDM);


        clock_gettime(CLOCK_REALTIME, &t9);


        for(int CMmode=0; CMmode < ecnt; CMmode ++)
        {
            // index in WFSall and CMall cubes
            //
            int mi = nbmode-1-CMmode;

            // copy and normalize by norm2 WFS

            for(int ii=0; ii<nbwfspix; ii++)
            {
                imgCMWFS.im->array.F[CMmode*nbwfspix + ii] = imgCMWFSall.im->array.F[mi*nbwfspix + ii] / n2cmWFS[mi];
            }

            for(int ii=0; ii<nbact; ii++)
            {
                imgCMDM.im->array.F[CMmode*nbact + ii] = imgCMDMall.im->array.F[mi*nbact + ii] / n2cmWFS[mi];
            }

        }



        free(n2cmDM);
        free(n2cmWFS);


        save_fits("VTmat", "./mkmodestmp/VTmat.fits");
        save_fits("CMmodesWFS", "./mkmodestmp/CMmodesWFS.fits");
        save_fits("CMmodesDM", "./mkmodestmp/CMmodesDM.fits");


        save_fits("CMmodesDM", CMmodesDMfname);
        save_fits("CMmodesWFS", CMmodesWFSfname);

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    struct timespec tdiff;


    tdiff = timespec_diff(t0, t1);
    double t01d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t1, t2);
    double t12d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t2, t3);
    double t23d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t3, t4);
    double t34d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t4, t5);
    double t45d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t5, t6);
    double t56d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t6, t7);
    double t67d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t7, t8);
    double t78d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff = timespec_diff(t8, t9);
    double t89d  = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;




//    printf("GSL         %5.3f s\n", t01d);
    printf("total       %5.3f s\n", t01d+t12d+t23d+t34d+t45d+t56d+t67d+t78d+t89d);
    printf("   0-1      %5.3f s\n", t01d);
    printf("   1-2      %5.3f s\n", t12d);
    printf("   2-3      %5.3f s\n", t23d);
    printf("   3-4      %5.3f s\n", t34d);
    printf("   4-5      %5.3f s\n", t45d);
    printf("   5-6      %5.3f s\n", t56d);
    printf("   6-7      %5.3f s\n", t67d);
    printf("   7-8      %5.3f s\n", t78d);
    printf("   8-9      %5.3f s\n", t89d);


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
