/**
 * @file compute_straight_CM.c
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

#include "timeutils.h"

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




static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "compsCM",
    .cmdkey      = "compsCM",
    .description = "compute straight control matrix"
};

static char *RMmodesDMfname;
static char *RMmodesWFSfname;
static char *DMmaskfname;
static char *WFSmaskfname;
static char *CMmodesDMfname;
static char *CMmodesWFSfname;

static float *svdlim;
static int32_t *GPUdevice;

#define FPS_PARAMS(X) \
    X(".RMmodesDM", &RMmodesDMfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input RM : DM modes") \
    X(".RMmodesWFS", &RMmodesWFSfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "input RM : WFS modes") \
    X(".dmmask", &DMmaskfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "DM mask for normalization") \
    X(".wfsmask", &WFSmaskfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "WFS mask for normalization") \
    X(".CMmodesDM", &CMmodesDMfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output CM : DM modes") \
    X(".CMmodesWFS", &CMmodesWFSfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output CM : WFS modes") \
    X(".svdlim", &svdlim, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "SVD limit") \
    X(".GPUdevice", &GPUdevice, FPTYPE_INT32, 1, FPFLAG_DEFAULT_INPUT, "using GPU (99 : no GPU, otherwise GPU device)")

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
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[functionparameter_GetParamIndex(data.fpsptr, ".RMmodesDM")].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[functionparameter_GetParamIndex(data.fpsptr, ".RMmodesWFS")].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[functionparameter_GetParamIndex(data.fpsptr, ".dmmask")].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[functionparameter_GetParamIndex(data.fpsptr, ".wfsmask")].fpflag |=
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






// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    list_image_ID();


    // input
    //

    IMGID imgRMDM;
    {
        imageID ID;
        load_fits(RMmodesDMfname, "RMmodesDM", LOADFITS_ERRMODE_WARNING, &ID);
        imgRMDM = makesetIMGID("RMmodesDM", ID);
    }


    IMGID imgRMWFS;
    {
        imageID ID;
        load_fits(RMmodesWFSfname, "RMmodesWFS", LOADFITS_ERRMODE_WARNING, &ID);
        imgRMWFS = makesetIMGID("RMmodesWFS", ID);
    }


    // masks are used for normalization of output
    // WFSmask multiplied by input WFS modes to exclude "bad" sensors
    //
    IMGID imgDMmask;
    {
        imageID ID;
        load_fits(DMmaskfname, "DMmask", LOADFITS_ERRMODE_WARNING, &ID);
        imgDMmask = makesetIMGID("DMmask", ID);
    }


    IMGID imgWFSmask;
    {
        imageID ID;
        load_fits(WFSmaskfname, "WFSmask", LOADFITS_ERRMODE_WARNING, &ID);
        imgWFSmask = makesetIMGID("WFSmask", ID);
    }






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



        printf("Number of modes    : %d\n", imgRMDM.md->size[2]);
        printf("Number of DM act   : %d x %d\n", imgRMDM.md->size[0], imgRMDM.md->size[1]);
        printf("Number of WFS pix  : %d x %d\n", imgRMWFS.md->size[0], imgRMWFS.md->size[1]);

        int nbmode = imgRMDM.md->size[2];
        int nbact = imgRMDM.md->size[0] * imgRMDM.md->size[1];
        int nbwfspix = imgRMWFS.md->size[0] * imgRMWFS.md->size[1];


        // multiply RMmodesWFS by WFSmask
        printf("Masking RM WFS by WFSmask\n");
        for(int m = 0; m < nbmode; m++)
        {
            for(int ii = 0; ii < nbwfspix; ii++)
            {
                imgRMWFS.im->array.F[m * nbwfspix + ii] *= imgWFSmask.im->array.F[ii];
            }
        }


        EXECUTE_SYSTEM_COMMAND("mkdir -p mkmodestmp");

        printf("=============================\n");
        printf("GPU device = %d\n", (int)(*GPUdevice));
        printf("SVD limit  = %f\n", *svdlim);


        // create eigenvectors array
        IMGID imgevec = imgid_make_from_name_2D("eigenvec", nbmode, nbmode);
        createimagefromIMGID(&imgevec);

        // create eigenvalues array
        IMGID imgeval = imgid_make_from_name_2D("eigenval", nbmode, 1);
        createimagefromIMGID(&imgeval);


        clock_gettime(CLOCK_MILK, &t0);


        {
            processinfo_WriteMessage(processinfo, "Create ATA");
            // create ATA
            IMGID imgATA = imgid_make_from_name_2D("ATA", nbmode, nbmode);
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

                    float *d_RMWFS;
                    cudaMalloc((void **)&d_RMWFS, imgRMWFS.md->nelement * sizeof(float));
                    cudaMemcpy(d_RMWFS, imgRMWFS.im->array.F, imgRMWFS.md->nelement * sizeof(float),
                               cudaMemcpyHostToDevice);

                    float *d_ATA;
                    cudaMalloc((void **)&d_ATA, imgATA.md->nelement * sizeof(float));

                    cublasHandle_t handle;
                    cublasCreate(&handle);

                    // Do the actual multiplication
                    cublasSgemm(handle, CUBLAS_OP_T, CUBLAS_OP_N,
                                nbmode, nbmode, nbwfspix, alpha, d_RMWFS, nbwfspix, d_RMWFS, nbwfspix, beta,
                                d_ATA, nbmode);

                    cublasDestroy(handle);

                    cudaMemcpy(imgATA.im->array.F, d_ATA, imgATA.md->nelement * sizeof(float),
                               cudaMemcpyDeviceToHost);

                    cudaFree(d_RMWFS);
                    cudaFree(d_ATA);

                    SGEMMcomputed = 1;
#endif
                }
                if(SGEMMcomputed == 0)
                {
                    printf("Running SGEMM 1 on CPU\n");
                    fflush(stdout);

                    cblas_sgemm(CblasColMajor, CblasTrans, CblasNoTrans,
                                nbmode, nbmode, nbwfspix, 1.0, imgRMWFS.im->array.F, nbwfspix,
                                imgRMWFS.im->array.F, nbwfspix, 0.0, imgATA.im->array.F, nbmode);
                }
            }



            clock_gettime(CLOCK_MILK, &t1);
            //save_fits("ATA", "compstrCM-ATA.fits");


            //nbmode = 100;
            //float *a = (float*) malloc(sizeof(float)*nbmode*nbmode);
            float *d = (float *) malloc(sizeof(float) * nbmode);
            float *e = (float *) malloc(sizeof(float) * nbmode);
            float *t = (float *) malloc(sizeof(float) * nbmode);


#ifdef HAVE_MKL
            mkl_set_interface_layer(MKL_INTERFACE_LP64);
#endif

            LAPACKE_ssytrd(LAPACK_COL_MAJOR, 'U', nbmode, (float *) imgATA.im->array.F, nbmode, d, e, t);

            clock_gettime(CLOCK_MILK, &t2);

            // Assemble Q matrix
            LAPACKE_sorgtr(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, t);


            clock_gettime(CLOCK_MILK, &t3);


            processinfo_WriteMessage(processinfo, "comp eigenv");

            memcpy(imgevec.im->array.F, imgATA.im->array.F, sizeof(float)*nbmode * nbmode);
            LAPACKE_ssteqr(LAPACK_COL_MAJOR, 'V', nbmode, d, e, imgevec.im->array.F, nbmode);
            memcpy(imgeval.im->array.F, d, sizeof(float)*nbmode);

            clock_gettime(CLOCK_MILK, &t4);

            free(d);
            free(e);
            free(t);

            //save_fits("eigenvec", "./mkmodestmp/eigenvec.fits");
        }






        // create CM WFS

        processinfo_WriteMessage(processinfo, "create CM WFS");

        IMGID imgCMWFSall = imgid_make_from_name_3D("CMmodesWFSall",
                                         imgRMWFS.md->size[0],
                                         imgRMWFS.md->size[1],
                                         imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMWFSall);

        clock_gettime(CLOCK_MILK, &t5);




        // Compute WFS modes
        // Multiply RMmodesWFS by Vmat
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

                float *d_RMWFS;
                cudaMalloc((void **)&d_RMWFS, imgRMWFS.md->nelement * sizeof(float));
                cudaMemcpy(d_RMWFS, imgRMWFS.im->array.F, imgRMWFS.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_evec;
                cudaMalloc((void **)&d_evec, imgevec.md->nelement * sizeof(float));
                cudaMemcpy(d_evec, imgevec.im->array.F, imgevec.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_CMWFSall;
                cudaMalloc((void **)&d_CMWFSall, imgCMWFSall.md->nelement * sizeof(float));
                //cudaMemcpy(d_RMWFS,imgRMWFS.im->array.F, imgRMWFS.md->nelement * sizeof(float), cudaMemcpyHostToDevice);

                // Create a handle for CUBLAS
                cublasHandle_t handle;
                cublasCreate(&handle);

                // Do the actual multiplication
                cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N,
                            nbwfspix, nbmode, nbmode, alpha, d_RMWFS, nbwfspix, d_evec, nbmode, beta,
                            d_CMWFSall, nbwfspix);

                // Destroy the handle
                cublasDestroy(handle);

                cudaMemcpy(imgCMWFSall.im->array.F, d_CMWFSall,
                           imgCMWFSall.md->nelement * sizeof(float), cudaMemcpyDeviceToHost);

                cudaFree(d_RMWFS);
                cudaFree(d_evec);
                cudaFree(d_CMWFSall);

                SGEMMcomputed = 1;
#endif
            }

            if(SGEMMcomputed == 0)
            {

                printf("Running SGEMM 2 on CPU\n");
                fflush(stdout);

                cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                            nbwfspix, nbmode, nbmode, 1.0, imgRMWFS.im->array.F, nbwfspix,
                            imgevec.im->array.F, nbmode, 0.0, imgCMWFSall.im->array.F, nbwfspix);

            }
        }




        clock_gettime(CLOCK_MILK, &t6);

        // create CM DM
        processinfo_WriteMessage(processinfo, "create CM DM");
        IMGID imgCMDMall = imgid_make_from_name_3D("CMmodesDMall", imgRMDM.md->size[0], imgRMDM.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMDMall);




        // Compute DM modes
        // Multiply RMmodesDM by Vmat
        //
        {
            int SGEMMcomputed = 0;
            if((*GPUdevice >= 0) && (*GPUdevice <= 99))
            {
#ifdef HAVE_CUDA
                printf("Running SGEMM 3 on GPU device %d\n", *GPUdevice);
                fflush(stdout);

                const float alf = 1;
                const float bet = 0;
                const float *alpha = &alf;
                const float *beta = &bet;

                float *d_RMDM;
                cudaMalloc((void **)&d_RMDM, imgRMDM.md->nelement * sizeof(float));
                cudaMemcpy(d_RMDM, imgRMDM.im->array.F, imgRMDM.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_evec;
                cudaMalloc((void **)&d_evec, imgevec.md->nelement * sizeof(float));
                cudaMemcpy(d_evec, imgevec.im->array.F, imgevec.md->nelement * sizeof(float),
                           cudaMemcpyHostToDevice);

                float *d_CMDMall;
                cudaMalloc((void **)&d_CMDMall, imgCMDMall.md->nelement * sizeof(float));

                // Create a handle for CUBLAS
                cublasHandle_t handle;
                cublasCreate(&handle);

                // Do the actual multiplication
                cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N,
                            nbact, nbmode, nbmode, alpha, d_RMDM, nbact, d_evec, nbmode, beta, d_CMDMall,
                            nbact);

                // Destroy the handle
                cublasDestroy(handle);

                cudaMemcpy(imgCMDMall.im->array.F, d_CMDMall,
                           imgCMDMall.md->nelement * sizeof(float), cudaMemcpyDeviceToHost);

                cudaFree(d_RMDM);
                cudaFree(d_evec);
                cudaFree(d_CMDMall);

                SGEMMcomputed = 1;
#endif
            }

            if(SGEMMcomputed == 0)
            {
                printf("Running SGEMM 3 on CPU\n");
                fflush(stdout);

                cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                            nbact, nbmode, nbmode, 1.0, imgRMDM.im->array.F, nbact, imgevec.im->array.F,
                            nbmode, 0.0, imgCMDMall.im->array.F, nbact);
            }
        }



        clock_gettime(CLOCK_MILK, &t7);


        processinfo_WriteMessage(processinfo, "mormalize modes");
        // norm2 of WFS and DM modes
        float *n2cmWFS = (float *) malloc(sizeof(float) * nbmode);
        float *n2cmDM  = (float *) malloc(sizeof(float) * nbmode);

        {
            // measure norm of modes in DM and WFS space
            //
            FILE *fp = fopen("mkmodestmp/mode_norm.txt", "w");
            for(int mi = 0; mi < nbmode; mi++)
            {
                //char *ptr;

                {
                    double DMnorm = 0.0;
                    double DMnormcnt = 0.0;
                    uint64_t iioffset = mi * imgCMDMall.md->size[0] * imgCMDMall.md->size[1];
                    for(uint64_t ii = 0; ii < imgCMDMall.md->size[0]*imgCMDMall.md->size[1]; ii++)
                    {
                        double val = imgCMDMall.im->array.F[iioffset + ii];
                        double valm = imgDMmask.im->array.F[ii];
                        DMnorm += val * val * valm;
                        DMnormcnt += valm;
                    }
                    n2cmDM[mi] = sqrt(DMnorm / DMnormcnt);
                }

                {
                    double WFSnorm = 0.0;
                    double WFSnormcnt = 0.0;
                    uint64_t iioffset = mi * imgCMWFSall.md->size[0] * imgCMWFSall.md->size[1];
                    for(uint64_t ii = 0; ii < imgCMWFSall.md->size[0]*imgCMWFSall.md->size[1]; ii++)
                    {
                        double val = imgCMWFSall.im->array.F[iioffset + ii];
                        double valm = imgWFSmask.im->array.F[ii];
                        WFSnorm += val * val * valm;
                        WFSnormcnt += valm;
                    }
                    n2cmWFS[mi] = sqrt(WFSnorm / WFSnormcnt);
                }



                //ptr = (void *) imgCMDMall.im->array.F;
                //ptr += sizeof(float) * mi * nbact;
                //n2cmDM[mi] = cblas_snrm2(nbact, (float *) ptr, 1);

                //ptr = (void *) imgCMWFSall.im->array.F;
                //ptr += sizeof(float) * mi * nbwfspix;
                //n2cmWFS[mi] = cblas_snrm2(nbwfspix, (float *) ptr, 1);

                fprintf(fp, "%4d    %20g    %20g   %20g\n", mi, n2cmDM[mi], n2cmWFS[mi],
                        imgeval.im->array.F[mi]);
            }
            fclose(fp);
        }

        clock_gettime(CLOCK_MILK, &t8);


        // select modes
        float evalmax = imgeval.im->array.F[nbmode - 1];
        int ecnt = 0;
        float evlim = *svdlim * *svdlim;
        {
            int mi = 0;
            while(imgeval.im->array.F[mi] < evalmax * evlim)
            {
                mi ++;
            }
            ecnt = nbmode - mi;
            printf("Selected %d modes\n", ecnt);
        }

        // create CMWFS and CMDM
        // contains strongest (highest singular values) modes from CMWFSall
        //
        processinfo_WriteMessage(processinfo, "create CMWFS and CMDM");


        IMGID imgCMWFS = imgid_make_from_name_3D("CMmodesWFS",
                                      imgRMWFS.md->size[0],
                                      imgRMWFS.md->size[1],
                                      ecnt);
        createimagefromIMGID(&imgCMWFS);

        IMGID imgCMDM = imgid_make_from_name_3D("CMmodesDM",
                                     imgRMDM.md->size[0],
                                     imgRMDM.md->size[1],
                                     ecnt);
        createimagefromIMGID(&imgCMDM);


        clock_gettime(CLOCK_MILK, &t9);



        //
        // Modes are normalized to RMS=1 in DM space
        //
        for(int CMmode = 0; CMmode < ecnt; CMmode ++)
        {
            // index in WFSall and CMall cubes
            //
            int mi = nbmode - 1 - CMmode;

            // copy and normalize by norm2 DM

            for(int ii = 0; ii < nbwfspix; ii++)
            {
                imgCMWFS.im->array.F[CMmode * nbwfspix + ii] =
                imgCMWFSall.im->array.F[mi * nbwfspix + ii] / n2cmDM[mi];
            }

            for(int ii = 0; ii < nbact; ii++)
            {
                imgCMDM.im->array.F[CMmode * nbact + ii] =
                    imgCMDMall.im->array.F[mi * nbact + ii] / n2cmDM[mi];
            }

        }



        {
            // measure norm of modes in DM and WFS space
            //
            FILE *fp = fopen("mkmodestmp/mode_norm_1.txt", "w");
            for(int CMmode = 0; CMmode < ecnt; CMmode++)
            {

                {
                    double WFSnorm = 0.0;
                    double WFSnormcnt = 0.0;

                    for(int ii = 0; ii < nbwfspix; ii++)
                    {
                        double val = imgCMWFS.im->array.F[CMmode * nbwfspix + ii];
                        double valm = imgWFSmask.im->array.F[ii];
                        WFSnorm += val * val * valm;
                        WFSnormcnt += valm;
                    }
                    n2cmWFS[CMmode] = sqrt(WFSnorm / WFSnormcnt);
                }


                {
                    double DMnorm = 0.0;
                    double DMnormcnt = 0.0;

                    for(int ii = 0; ii < nbact; ii++)
                    {
                        double val = imgCMDM.im->array.F[CMmode * nbact + ii];
                        double valm = imgDMmask.im->array.F[ii];
                        DMnorm += val * val * valm;
                        DMnormcnt += valm;
                    }
                    n2cmDM[CMmode] = sqrt(DMnorm / DMnormcnt);
                }


                fprintf(fp, "%4d    %20g    %20g \n", CMmode, n2cmDM[CMmode], n2cmWFS[CMmode]);

            }
            fclose(fp);
        }





        free(n2cmDM);
        free(n2cmWFS);

        printf("SAVING FILES TO DISK \n");
        save_fits("VTmat", "./mkmodestmp/VTmat.fits");
        save_fits("CMmodesWFS", "./mkmodestmp/CMmodesWFS.fits");
        save_fits("CMmodesDM", "./mkmodestmp/CMmodesDM.fits");

        printf("writing CMmodesDM  to file %s\n", CMmodesDMfname);
        printf("writing CMmodesWFS to file %s\n", CMmodesWFSfname);

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
    printf("total       %5.3f s\n",
           t01d + t12d + t23d + t34d + t45d + t56d + t67d + t78d + t89d);
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
CLIADDCMD_AOloopControl_computeCalib__compsCM()
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
