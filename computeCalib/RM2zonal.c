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
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cusolverDn.h>
#include <device_types.h>
#include <pthread.h>
#endif



// CPU mode: Use MKL if available
// Otherwise use openBLAS
//
#ifdef HAVE_MKL
#include "mkl.h"
#include "mkl_lapacke.h"
#define BLASLIB "IntelMKL"
#else
#ifdef HAVE_OPENBLAS
#include <cblas.h>
#include <lapacke.h>
#define BLASLIB "OpenBLAS"
#endif
#endif




static char *RMmodesDM;
static long  fpi_RMmodesDM;

static char *RMmodesWFS;
static long  fpi_RMmodesWFS;

static char *RMmodesDMz;
static long  fpi_RMmodesDMz;

static char *RMmodesWFSz;
static long  fpi_RMmodesWFSz;


static float *svdlim;
static long   fpi_svdlim;

static int32_t *GPUdevice;
static long     fpi_GPUdevice;



static CLICMDARGDEF farg[] =
{
    {
        // input RM : DM modes
        CLIARG_IMG,
        ".RMmodesDM",
        "input response matrix DM modes",
        "RMmodesDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesDM,
        &fpi_RMmodesDM
    },
    {
        // input RM : WFS modes
        CLIARG_IMG,
        ".RMmodesWFS",
        "input response matrix WFS modes",
        "RMmodesWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesWFS,
        &fpi_RMmodesWFS
    },
    {
        // output zonal RM : DM modes
        CLIARG_STR,
        ".RMmodesDMz",
        "output zonal response matrix DM modes",
        "RMmodesDMz",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesDMz,
        &fpi_RMmodesDMz
    },
    {
        // output zonal RM : WFS modes
        CLIARG_STR,
        ".RMmodesWFSz",
        "output zonal response matrix WFS modes",
        "RMmodesWFSz",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &RMmodesWFSz,
        &fpi_RMmodesWFSz
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
    "RM2zonal", "convert arbitrary response matrix to zonal", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    IMGID imgRMDM = mkIMGID_from_name(RMmodesDM);
    resolveIMGID(&imgRMDM, ERRMODE_ABORT);

    IMGID imgRMWFS = mkIMGID_from_name(RMmodesWFS);
    resolveIMGID(&imgRMWFS, ERRMODE_ABORT);

    struct timespec t0, t1, t2, t3, t4, t5;


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




        //ID = image_ID("VTmat");
        //IMGID imgVT = makesetIMGID("VTmat", ID);

        int nbmode;
        int nbact;

        if(imgRMDM.md->naxis == 3)
        {
            printf("Number of modes    : %d\n", imgRMDM.md->size[2]);
            nbmode = imgRMDM.md->size[2];

            printf("Number of DM act   : %d x %d\n", imgRMDM.md->size[0],
                   imgRMDM.md->size[1]);
            nbact = imgRMDM.md->size[0] * imgRMDM.md->size[1];
        }
        else
        {
            printf("Number of modes    : %d\n", imgRMDM.md->size[1]);
            nbmode = imgRMDM.md->size[1];

            printf("Number of DM act   : %d\n", imgRMDM.md->size[0]);
            nbact = imgRMDM.md->size[0];
        }

        printf("Number of WFS pix  : %d x %d\n", imgRMWFS.md->size[0], imgRMWFS.md->size[1]);
        int nbwfspix = imgRMWFS.md->size[0] * imgRMWFS.md->size[1];






        EXECUTE_SYSTEM_COMMAND("mkdir -p mkmodestmp");

        printf("=============================\n");
        printf("GPU device = %d\n", (int)(*GPUdevice));
        printf("SVD limit  = %f\n", *svdlim);









        enum matrix_shape{longaxis_act, longaxis_mode} mshape;
        uint32_t Mdim = 0;
        uint32_t Ndim = 0;

        if(nbmode < nbact)
        {
            // this is the default
            // notations follow this case
            //
            printf("CASE NBMODE < NBACT \n");
            mshape = longaxis_act;
            Mdim = nbact;
            Ndim = nbmode;

        }
        else
        {
            printf("CASE NBMODE > NBACT \n");
            mshape = longaxis_mode;
            Mdim = nbmode;
            Ndim = nbact;
        }

        // from now on, Mdim > Ndim


        // create eigenvectors array
        IMGID imgmV = makeIMGID_2D("mV", Ndim, Ndim);
        createimagefromIMGID(&imgmV);

        // create eigenvalues array
        IMGID imgeval = makeIMGID_2D("eigenval", Ndim, 1);
        createimagefromIMGID(&imgeval);


        clock_gettime(CLOCK_MILK, &t0);


        {
            // create ATA
            // note that this is AAT if nbmode > nbact
            //
            IMGID imgATA = makeIMGID_2D("ATA", Ndim, Ndim);
            createimagefromIMGID(&imgATA);

            {
                int SGEMMcomputed = 0;
                if((*GPUdevice >= 0) && (*GPUdevice <= 99))
                {
#ifdef HAVE_CUDA
                    printf("Running SGEMM 1 on GPU device %d\n", *GPUdevice);
                    fflush(stdout);

                    const float alf = 1;
                    const float bet = 0;
                    const float *alpha = &alf;
                    const float *beta = &bet;

                    float *d_RMDM;
                    cudaMalloc((void **)&d_RMDM, imgRMDM.md->nelement * sizeof(float));
                    cudaMemcpy(d_RMDM, imgRMDM.im->array.F, imgRMDM.md->nelement * sizeof(float),
                               cudaMemcpyHostToDevice);

                    float *d_ATA;
                    cudaMalloc((void **)&d_ATA, imgATA.md->nelement * sizeof(float));

                    // Create a handle for CUBLAS
                    cublasHandle_t handle;
                    cublasCreate(&handle);

                    // Do the actual multiplication
                    cublasOperation_t OP0 = CUBLAS_OP_T;
                    cublasOperation_t OP1 = CUBLAS_OP_N;
                    if(mshape == longaxis_mode)
                    {
                        OP0 = CUBLAS_OP_N;
                        OP1 = CUBLAS_OP_T;
                    }

                    cublasSgemm(handle, OP0, OP1,
                                Ndim, Ndim, Mdim, alpha, d_RMDM, nbact, d_RMDM, nbact, beta, d_ATA, Ndim);

                    cublasDestroy(handle);

                    cudaMemcpy(imgATA.im->array.F, d_ATA, imgATA.md->nelement * sizeof(float),
                               cudaMemcpyDeviceToHost);

                    cudaFree(d_RMDM);
                    cudaFree(d_ATA);

                    SGEMMcomputed = 1;
#endif
                }
                if(SGEMMcomputed == 0)
                {
                    printf("Running SGEMM 1 on CPU\n");
                    fflush(stdout);


                    CBLAS_TRANSPOSE OP0 = CblasTrans;
                    CBLAS_TRANSPOSE OP1 = CblasNoTrans;
                    if(mshape == longaxis_mode)
                    {
                        OP0 = CblasNoTrans;
                        OP1 = CblasTrans;
                    }

                    cblas_sgemm(CblasColMajor, OP0, OP1,
                                Ndim, Ndim, Mdim, 1.0, imgRMDM.im->array.F, nbact, imgRMDM.im->array.F, nbact,
                                0.0, imgATA.im->array.F, Ndim);
                }
            }



            clock_gettime(CLOCK_MILK, &t1);


            //save_fits("ATA", "mATA.fits");



            float *d = (float *) malloc(sizeof(float) * Ndim);
            float *e = (float *) malloc(sizeof(float) * Ndim);
            float *t = (float *) malloc(sizeof(float) * Ndim);


#ifdef HAVE_MKL
            mkl_set_interface_layer(MKL_INTERFACE_ILP64);
#endif

            LAPACKE_ssytrd(LAPACK_COL_MAJOR, 'U', Ndim, (float *) imgATA.im->array.F, Ndim, d, e, t);

            clock_gettime(CLOCK_MILK, &t2);

            // Assemble Q matrix
            LAPACKE_sorgtr(LAPACK_COL_MAJOR, 'U', Ndim, imgATA.im->array.F, Ndim, t);


            clock_gettime(CLOCK_MILK, &t3);


            // compute all eigenvalues and eivenvectors -> imgV
            //
            memcpy(imgmV.im->array.F, imgATA.im->array.F, sizeof(float)*Ndim * Ndim);
            LAPACKE_ssteqr(LAPACK_COL_MAJOR, 'V', Ndim, d, e, imgmV.im->array.F, Ndim);
            memcpy(imgeval.im->array.F, d, sizeof(float)*Ndim);

            clock_gettime(CLOCK_MILK, &t4);

            free(d);
            free(e);
            free(t);

            delete_image(&imgATA, DELETE_IMAGE_ERRMODE_EXIT);
            // this is matV
            //save_fits("mV", "mV.fits");
        }





        // create mU (only non-zero part allocated)
        //
        IMGID imgmU = makeIMGID_2D("mU", Mdim, Ndim);
        createimagefromIMGID(&imgmU);

        clock_gettime(CLOCK_MILK, &t5);

        // Compute mU (only non-zero part allocated)
        // Multiply RMmodesDM by Vmat
        //

        {
            int SGEMMcomputed = 0;
            if((*GPUdevice >= 0) && (*GPUdevice <= 99))
            {
#ifdef HAVE_CUDA
                printf("Running SGEMM 2 on GPU device %d\n", *GPUdevice);
                fflush(stdout);

                const float alf = 1;
                const float bet = 0;
                const float *alpha = &alf;
                const float *beta = &bet;

                float *d_RMDM;
                cudaMalloc((void **)&d_RMDM, imgRMDM.md->nelement * sizeof(float));
                cudaMemcpy(d_RMDM, imgRMDM.im->array.F, imgRMDM.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_mV;
                cudaMalloc((void **)&d_mV, imgmV.md->nelement * sizeof(float));
                cudaMemcpy(d_mV, imgmV.im->array.F, imgmV.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_mU;
                cudaMalloc((void **)&d_mU, imgmU.md->nelement * sizeof(float));

                cublasHandle_t handle;
                cublasCreate(&handle);

                cublasOperation_t OP0 = CUBLAS_OP_N;
                if(mshape == longaxis_mode)
                {
                    OP0 = CUBLAS_OP_T;
                }
                cublasSgemm(handle, OP0, CUBLAS_OP_N,
                            Mdim, Ndim, Ndim, alpha, d_RMDM, nbact, d_mV, Ndim, beta, d_mU, Mdim);

                cublasDestroy(handle);

                cudaMemcpy(imgmU.im->array.F, d_mU, imgmU.md->nelement * sizeof(float),
                           cudaMemcpyDeviceToHost);

                cudaFree(d_RMDM);
                cudaFree(d_mV);
                cudaFree(d_mU);

                SGEMMcomputed = 1;
#endif
            }

            if(SGEMMcomputed == 0)
            {
                printf("Running SGEMM 2 on CPU\n");
                fflush(stdout);

                CBLAS_TRANSPOSE OP0 = CblasNoTrans;
                if(mshape == longaxis_mode)
                {
                    OP0 = CblasTrans;
                }

                cblas_sgemm(CblasColMajor, OP0, CblasNoTrans,
                            Mdim, Ndim, Ndim, 1.0, imgRMDM.im->array.F, nbact, imgmV.im->array.F, Ndim, 0.0,
                            imgmU.im->array.F, Mdim);

            }
        }





        //IMGID imgmAinv = makeIMGID_2D("mAinv", Ndim, Mdim);
        IMGID imgmAinv = makeIMGID_2D("mAinv", nbmode, nbact);
        createimagefromIMGID(&imgmAinv);

        float evalmax = imgeval.im->array.F[Ndim - 1];
        uint32_t modecnt = 0;


        if(mshape == longaxis_act)
        {
            // Compute pseudo inverse
            // multiply V (=mV) and UT (=Transpose(mU))
            //
            // transpose U -> UT (UT truncated to number of eivenvals)
            // multiply lines of UT by inv(eigenval)
            //
            IMGID imgmUT = makeIMGID_2D("mUT", Ndim, Mdim);
            createimagefromIMGID(&imgmUT);

            for(uint32_t ii = 0; ii < Ndim; ii++)
            {
                float mcolcoeff = 0.0;
                float evalnorm = imgeval.im->array.F[ii] / evalmax;


                if(evalnorm > *svdlim)
                {
                    mcolcoeff = 1.0 / imgeval.im->array.F[ii];
                    modecnt ++;
                }
                //printf("mode %4u  %12f %12f   %12f\n", ii, imgeval.im->array.F[ii], evalnorm, mcolcoeff);


                for(uint32_t jj = 0; jj < Mdim; jj++)
                {
                    imgmUT.im->array.F[jj * Ndim + ii] = mcolcoeff * imgmU.im->array.F[ii * Mdim +
                                                         jj];
                }
            }


            cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                        Ndim, Mdim, Ndim, 1.0, imgmV.im->array.F, Ndim, imgmUT.im->array.F, Ndim, 0.0,
                        imgmAinv.im->array.F, Ndim);

            delete_image(&imgmUT, DELETE_IMAGE_ERRMODE_EXIT);
        }
        else
        {
            // Compute pseudo inverse
            // multiply V (=mU) and UT (=Transpose(mV))
            //
            IMGID imgmUi = makeIMGID_2D("mUi", Mdim, Ndim);
            createimagefromIMGID(&imgmUi);

            for(uint32_t jj = 0; jj < Ndim; jj++)
            {
                float mcolcoeff = 0.0;
                float evalnorm = imgeval.im->array.F[jj] / evalmax;


                if(evalnorm > *svdlim)
                {
                    mcolcoeff = 1.0 / imgeval.im->array.F[jj];
                    modecnt ++;
                }
                //printf("mode %4u  %12f %12f   %12f\n", jj, imgeval.im->array.F[jj], evalnorm, mcolcoeff);


                for(uint32_t ii = 0; ii < Mdim; ii++)
                {
                    imgmUi.im->array.F[jj * Mdim + ii] = mcolcoeff * imgmU.im->array.F[jj * Mdim +
                                                         ii];
                }
            }

            cblas_sgemm(CblasColMajor, CblasNoTrans, CblasTrans,
                        Mdim, Ndim, Ndim, 1.0, imgmUi.im->array.F, Mdim, imgmV.im->array.F, Ndim, 0.0,
                        imgmAinv.im->array.F, Mdim);

            delete_image(&imgmUi, DELETE_IMAGE_ERRMODE_EXIT);
        }

        printf("Kept %u / %u modes\n", modecnt, nbmode);

        delete_image(&imgmV, DELETE_IMAGE_ERRMODE_EXIT);
        delete_image(&imgmU, DELETE_IMAGE_ERRMODE_EXIT);
        delete_image(&imgeval, DELETE_IMAGE_ERRMODE_EXIT);


        /*
        printf("==============TESTING =============\n");
        fflush(stdout);

        // Test Ainv x A

        IMGID imgmAinvA = makeIMGID_2D("mAinvA", nbmode, nbmode);
        createimagefromIMGID(&imgmAinvA);


        cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans,
                     nbmode, nbmode, nbact, 1.0, imgmAinv.im->array.F, nbmode, imgRMDM.im->array.F, nbact, 0.0, imgmAinvA.im->array.F, nbmode);

        // Test A x Ainv

        IMGID imgmAAinv = makeIMGID_2D("mAAinv", nbact, nbact);
        createimagefromIMGID(&imgmAAinv);

        cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans,
                     nbact, nbact, nbmode, 1.0, imgRMDM.im->array.F, nbact, imgmAinv.im->array.F, nbmode, 0.0, imgmAAinv.im->array.F, nbact);

        */







        // multiply RMwfs x Ainv -> RMzwfs

        IMGID imgRMWFSz = makeIMGID_3D(RMmodesWFSz,  imgRMWFS.md->size[0], imgRMWFS.md->size[1], nbact);
        createimagefromIMGID(&imgRMWFSz);

        cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                    nbwfspix, nbact, nbmode, 1.0, imgRMWFS.im->array.F, nbwfspix, imgmAinv.im->array.F, nbmode, 0.0, imgRMWFSz.im->array.F, nbwfspix);



        // multiply RMdm x Ainv -> RMzdm

        IMGID imgRMDMz = makeIMGID_2D(RMmodesDMz,  imgRMDM.md->size[0] * imgRMDM.md->size[1], nbact);
        createimagefromIMGID(&imgRMDMz);

        cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                    nbact, nbact, nbmode, 1.0, imgRMDM.im->array.F, nbact, imgmAinv.im->array.F, nbmode, 0.0, imgRMDMz.im->array.F, nbact);




        /*
        // TESTING RECOVERY OF ORIGINAL MODES

        // RMzwfs x RMDM -> RMwfsm

        IMGID imgRMWFSm = makeIMGID_3D("RMwfsm",  imgRMWFS.md->size[0], imgRMWFS.md->size[1], nbmode);
        createimagefromIMGID(&imgRMWFSm);

        cblas_sgemm (CblasColMajor, CblasNoTrans, CblasNoTrans,
                     nbwfspix, nbmode, nbact, 1.0, imgRMWFSz.im->array.F, nbwfspix, imgRMDM.im->array.F, nbact, 0.0, imgRMWFSm.im->array.F, nbwfspix);

        */

        delete_image(&imgmAinv, DELETE_IMAGE_ERRMODE_EXIT);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END



    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__RM2zonal()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
