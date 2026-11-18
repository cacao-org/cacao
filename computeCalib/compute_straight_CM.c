// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file compute_straight_CM.c
 * @brief Compute straight cm module
 */

// MILK_CMAKE_MANDATE_LAPACKE
// MILK_CMAKE_REQUEST_BLAS
// MILK_CMAKE_REQUEST_CUDA

#include <limits.h>
#include <math.h>
#include <omp.h>
#include <stdlib.h>
#include <unistd.h>

#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"

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
#include "milk_blas_lapacke.h"

#include "linopt_imtools/compute_SVDpseudoInverse.h"

#ifdef HAVE_CUDA
#    include <cublas_v2.h>
#    include <cuda_runtime.h>
#    include <cuda_runtime_api.h>
#    include <cusolverDn.h>
#    include <device_types.h>
#    include <pthread.h>
#endif

#include "computeCalib.h"


static char RMmodesDMfname[FUNCTION_PARAMETER_STRMAXLEN];
static char RMmodesWFSfname[FUNCTION_PARAMETER_STRMAXLEN];
static char DMmaskfname[FUNCTION_PARAMETER_STRMAXLEN];
static char WFSmaskfname[FUNCTION_PARAMETER_STRMAXLEN];
static char CMmodesDMfname[FUNCTION_PARAMETER_STRMAXLEN];
static char CMmodesWFSfname[FUNCTION_PARAMETER_STRMAXLEN];

static float   svdlim;
static int32_t GPUdevice;

#define FPS_PARAMS(X)                                                                              \
    X(".RMmodesDM", RMmodesDMfname, FPTYPE_FILENAME, 1,                                            \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_FILE_RUN_REQUIRED), "input RM : DM modes") \
    X(".RMmodesWFS", RMmodesWFSfname, FPTYPE_FILENAME, 1,                                          \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_FILE_RUN_REQUIRED),                        \
      "input RM : WFS modes")                                                                      \
    X(".dmmask", DMmaskfname, FPTYPE_FILENAME, 1,                                                  \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_FILE_RUN_REQUIRED),                        \
      "DM mask for normalization")                                                                 \
    X(".wfsmask", WFSmaskfname, FPTYPE_FILENAME, 1,                                                \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_FILE_RUN_REQUIRED),                        \
      "WFS mask for normalization")                                                                \
    X(".CMmodesDM", CMmodesDMfname, FPTYPE_FILENAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "output CM : DM modes")                                                                      \
    X(".CMmodesWFS", CMmodesWFSfname, FPTYPE_FILENAME, 1,                                          \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output CM : WFS modes")                          \
    X(".svdlim", &svdlim, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),            \
      "SVD limit")                                                                                 \
    X(".GPUdevice", &GPUdevice, FPTYPE_INT32, 1, FPFLAG_DEFAULT_INPUT,                             \
      "using GPU (99 : no GPU, otherwise GPU device)")

MILK_PARAMS_DEFINE(compute_straight_CM, FPS_PARAMS);


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    return RETURN_SUCCESS;
}


static MILK_HOT errno_t compute_straight_CM_compute(const MILK_RUNNABLE *self)
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


    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
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

        int nbmode   = imgRMDM.md->size[2];
        int nbact    = imgRMDM.md->size[0] * imgRMDM.md->size[1];
        int nbwfspix = imgRMWFS.md->size[0] * imgRMWFS.md->size[1];


        // multiply RMmodesWFS by WFSmask
        printf("Masking RM WFS by WFSmask\n");
#pragma omp parallel for schedule(static)
        for (int m = 0; m < nbmode; m++)
        {
            float       *dst = &imgRMWFS.im->array.F[(size_t) m * nbwfspix];
            const float *msk = imgWFSmask.im->array.F;
            for (int ii = 0; ii < nbwfspix; ii++)
            {
                dst[ii] *= msk[ii];
            }
        }


        EXECUTE_SYSTEM_COMMAND("mkdir -p mkmodestmp");

        printf("=============================\n");
        printf("GPU device = %d\n", (int) (GPUdevice));
        printf("SVD limit  = %f\n", svdlim);


        // create eigenvectors array
        IMGID imgevec = imgid_make_from_name_2D("eigenvec", nbmode, nbmode);
        createimagefromIMGID(&imgevec);

        // create eigenvalues array
        IMGID imgeval = imgid_make_from_name_2D("eigenval", nbmode, 1);
        createimagefromIMGID(&imgeval);


        clock_gettime(CLOCK_MILK, &t0);

        IMGID imgCMWFSall = imgid_make_from_name_3D("CMmodesWFSall", imgRMWFS.md->size[0],
                                                    imgRMWFS.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMWFSall);

        IMGID imgCMDMall = imgid_make_from_name_3D("CMmodesDMall", imgRMDM.md->size[0],
                                                   imgRMDM.md->size[1], imgRMDM.md->size[2]);
        createimagefromIMGID(&imgCMDMall);

        int gpu_pipeline_done = 0;
#ifdef HAVE_CUDA
        if ((GPUdevice >= 0) && (GPUdevice < 99))
        {
            int ndev = 0;
            if (cudaGetDeviceCount(&ndev) == cudaSuccess && GPUdevice < ndev)
            {
                cudaSetDevice(GPUdevice);
                printf("Running GPU cuBLAS + cuSOLVER Ssyevd pipeline on device %d\n", GPUdevice);
                fflush(stdout);

                const float alf  = 1.0f;
                const float bet  = 0.0f;
                float      *d_RMWFS    = NULL;
                float      *d_ATA      = NULL;
                float      *d_eval     = NULL;
                float      *d_work     = NULL;
                int        *d_info     = NULL;
                float      *d_CMWFSall = NULL;
                float      *d_RMDM     = NULL;
                float      *d_CMDMall  = NULL;

                cublasHandle_t     cublasH   = NULL;
                cusolverDnHandle_t cusolverH = NULL;

                size_t rmwfs_bytes = imgRMWFS.md->nelement * sizeof(float);
                size_t rmdm_bytes  = imgRMDM.md->nelement * sizeof(float);
                size_t ata_bytes   = (size_t) nbmode * (size_t) nbmode * sizeof(float);
                size_t eval_bytes  = (size_t) nbmode * sizeof(float);

                if (cublasCreate(&cublasH) == CUBLAS_STATUS_SUCCESS &&
                    cusolverDnCreate(&cusolverH) == CUSOLVER_STATUS_SUCCESS &&
                    cudaMalloc((void **) &d_RMWFS, rmwfs_bytes) == cudaSuccess &&
                    cudaMalloc((void **) &d_ATA, ata_bytes) == cudaSuccess &&
                    cudaMalloc((void **) &d_eval, eval_bytes) == cudaSuccess &&
                    cudaMalloc((void **) &d_info, sizeof(int)) == cudaSuccess)
                {
                    processinfo_WriteMessage(processinfo, "Create ATA (GPU)");
                    cudaMemcpy(d_RMWFS, imgRMWFS.im->array.F, rmwfs_bytes, cudaMemcpyHostToDevice);
                    cublasSgemm(cublasH, CUBLAS_OP_T, CUBLAS_OP_N, nbmode, nbmode, nbwfspix, &alf,
                                d_RMWFS, nbwfspix, d_RMWFS, nbwfspix, &bet, d_ATA, nbmode);

                    clock_gettime(CLOCK_MILK, &t1);
                    t2 = t1;
                    t3 = t1;

                    processinfo_WriteMessage(processinfo, "comp eigenv (GPU)");
                    int lwork = 0;
                    if (cusolverDnSsyevd_bufferSize(cusolverH, CUSOLVER_EIG_MODE_VECTOR,
                                                    CUBLAS_FILL_MODE_UPPER, nbmode, d_ATA, nbmode,
                                                    d_eval, &lwork) == CUSOLVER_STATUS_SUCCESS &&
                        cudaMalloc((void **) &d_work, (size_t) (lwork > 0 ? lwork : 1) *
                                                          sizeof(float)) == cudaSuccess)
                    {
                        int h_info = -1;
                        if (cusolverDnSsyevd(cusolverH, CUSOLVER_EIG_MODE_VECTOR,
                                             CUBLAS_FILL_MODE_UPPER, nbmode, d_ATA, nbmode, d_eval,
                                             d_work, lwork, d_info) == CUSOLVER_STATUS_SUCCESS)
                        {
                            cudaMemcpy(&h_info, d_info, sizeof(int), cudaMemcpyDeviceToHost);
                        }
                        if (h_info == 0)
                        {
                            cudaMemcpy(imgeval.im->array.F, d_eval, eval_bytes,
                                       cudaMemcpyDeviceToHost);
                            cudaMemcpy(imgevec.im->array.F, d_ATA, ata_bytes,
                                       cudaMemcpyDeviceToHost);
                            clock_gettime(CLOCK_MILK, &t4);

                            processinfo_WriteMessage(processinfo, "create CM WFS (GPU)");
                            clock_gettime(CLOCK_MILK, &t5);
                            if (cudaMalloc((void **) &d_CMWFSall, rmwfs_bytes) == cudaSuccess)
                            {
                                cublasSgemm(cublasH, CUBLAS_OP_N, CUBLAS_OP_N, nbwfspix, nbmode,
                                            nbmode, &alf, d_RMWFS, nbwfspix, d_ATA, nbmode, &bet,
                                            d_CMWFSall, nbwfspix);
                                cudaMemcpy(imgCMWFSall.im->array.F, d_CMWFSall, rmwfs_bytes,
                                           cudaMemcpyDeviceToHost);
                                cudaFree(d_CMWFSall);
                                d_CMWFSall = NULL;
                                cudaFree(d_RMWFS);
                                d_RMWFS = NULL;
                                clock_gettime(CLOCK_MILK, &t6);

                                processinfo_WriteMessage(processinfo, "create CM DM (GPU)");
                                if (cudaMalloc((void **) &d_RMDM, rmdm_bytes) == cudaSuccess &&
                                    cudaMalloc((void **) &d_CMDMall, rmdm_bytes) == cudaSuccess)
                                {
                                    cudaMemcpy(d_RMDM, imgRMDM.im->array.F, rmdm_bytes,
                                               cudaMemcpyHostToDevice);
                                    cublasSgemm(cublasH, CUBLAS_OP_N, CUBLAS_OP_N, nbact, nbmode,
                                                nbmode, &alf, d_RMDM, nbact, d_ATA, nbmode, &bet,
                                                d_CMDMall, nbact);
                                    cudaMemcpy(imgCMDMall.im->array.F, d_CMDMall, rmdm_bytes,
                                               cudaMemcpyDeviceToHost);
                                    gpu_pipeline_done = 1;
                                }
                            }
                        }
                    }
                }

                if (d_CMDMall)
                    cudaFree(d_CMDMall);
                if (d_RMDM)
                    cudaFree(d_RMDM);
                if (d_CMWFSall)
                    cudaFree(d_CMWFSall);
                if (d_work)
                    cudaFree(d_work);
                if (d_info)
                    cudaFree(d_info);
                if (d_eval)
                    cudaFree(d_eval);
                if (d_ATA)
                    cudaFree(d_ATA);
                if (d_RMWFS)
                    cudaFree(d_RMWFS);
                if (cusolverH)
                    cusolverDnDestroy(cusolverH);
                if (cublasH)
                    cublasDestroy(cublasH);
            }
        }
#endif

        if (!gpu_pipeline_done)
        {
            processinfo_WriteMessage(processinfo, "Create ATA (CPU)");
            IMGID imgATA = imgid_make_from_name_2D("ATA", nbmode, nbmode);
            createimagefromIMGID(&imgATA);

#ifdef HAVE_BLAS
            printf("Running SGEMM 1 + divide-and-conquer ssyevd on CPU\n");
            fflush(stdout);
            cblas_sgemm(CblasColMajor, CblasTrans, CblasNoTrans, nbmode, nbmode, nbwfspix, 1.0f,
                        imgRMWFS.im->array.F, nbwfspix, imgRMWFS.im->array.F, nbwfspix, 0.0f,
                        imgATA.im->array.F, nbmode);
#else
            PRINT_ERROR("FATAL -- need either of CUDA or BLAS for this computation\n");
            exit(1);
#endif
            clock_gettime(CLOCK_MILK, &t1);
            t2 = t1;
            t3 = t1;

#ifdef HAVE_MKL
            mkl_set_interface_layer(MKL_INTERFACE_LP64);
#endif
            processinfo_WriteMessage(processinfo, "comp eigenv");
            memcpy(imgevec.im->array.F, imgATA.im->array.F,
                   sizeof(float) * (size_t) nbmode * (size_t) nbmode);
            if (LAPACKE_ssyevd(LAPACK_COL_MAJOR, 'V', 'U', nbmode, imgevec.im->array.F, nbmode,
                               imgeval.im->array.F) != 0)
            {
                float *d = (float *) malloc(sizeof(float) * nbmode);
                float *e = (float *) malloc(sizeof(float) * nbmode);
                float *t = (float *) malloc(sizeof(float) * nbmode);
                LAPACKE_ssytrd(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, d, e, t);
                LAPACKE_sorgtr(LAPACK_COL_MAJOR, 'U', nbmode, imgATA.im->array.F, nbmode, t);
                memcpy(imgevec.im->array.F, imgATA.im->array.F,
                       sizeof(float) * (size_t) nbmode * (size_t) nbmode);
                LAPACKE_ssteqr(LAPACK_COL_MAJOR, 'V', nbmode, d, e, imgevec.im->array.F, nbmode);
                memcpy(imgeval.im->array.F, d, sizeof(float) * nbmode);
                free(d);
                free(e);
                free(t);
            }
            clock_gettime(CLOCK_MILK, &t4);

            processinfo_WriteMessage(processinfo, "create CM WFS");
            clock_gettime(CLOCK_MILK, &t5);
#ifdef HAVE_BLAS
            cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, nbwfspix, nbmode, nbmode, 1.0f,
                        imgRMWFS.im->array.F, nbwfspix, imgevec.im->array.F, nbmode, 0.0f,
                        imgCMWFSall.im->array.F, nbwfspix);
#endif
            clock_gettime(CLOCK_MILK, &t6);

            processinfo_WriteMessage(processinfo, "create CM DM");
#ifdef HAVE_BLAS
            cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, nbact, nbmode, nbmode, 1.0f,
                        imgRMDM.im->array.F, nbact, imgevec.im->array.F, nbmode, 0.0f,
                        imgCMDMall.im->array.F, nbact);
#endif
        }


        clock_gettime(CLOCK_MILK, &t7);


        processinfo_WriteMessage(processinfo, "mormalize modes");
        // norm2 of WFS and DM modes
        float *n2cmWFS = (float *) malloc(sizeof(float) * nbmode);
        float *n2cmDM  = (float *) malloc(sizeof(float) * nbmode);

        {
            double DMnormcnt  = 0.0;
            double WFSnormcnt = 0.0;
            for (int ii = 0; ii < nbact; ii++)
            {
                DMnormcnt += (double) imgDMmask.im->array.F[ii];
            }
            for (int ii = 0; ii < nbwfspix; ii++)
            {
                WFSnormcnt += (double) imgWFSmask.im->array.F[ii];
            }

#pragma omp parallel for schedule(static)
            for (int mi = 0; mi < nbmode; mi++)
            {
                double       DMnorm  = 0.0;
                const float *dm_src  = &imgCMDMall.im->array.F[(size_t) mi * nbact];
                const float *dm_msk  = imgDMmask.im->array.F;
                for (int ii = 0; ii < nbact; ii++)
                {
                    double val = (double) dm_src[ii];
                    DMnorm += val * val * (double) dm_msk[ii];
                }
                n2cmDM[mi] = (float) sqrt(DMnorm / DMnormcnt);

                double       WFSnorm = 0.0;
                const float *wfs_src = &imgCMWFSall.im->array.F[(size_t) mi * nbwfspix];
                const float *wfs_msk = imgWFSmask.im->array.F;
                for (int ii = 0; ii < nbwfspix; ii++)
                {
                    double val = (double) wfs_src[ii];
                    WFSnorm += val * val * (double) wfs_msk[ii];
                }
                n2cmWFS[mi] = (float) sqrt(WFSnorm / WFSnormcnt);
            }

            FILE *fp = fopen("mkmodestmp/mode_norm.txt", "w");
            if (fp != NULL)
            {
                for (int mi = 0; mi < nbmode; mi++)
                {
                    fprintf(fp, "%4d    %20g    %20g   %20g\n", mi, n2cmDM[mi], n2cmWFS[mi],
                            imgeval.im->array.F[mi]);
                }
                fclose(fp);
            }
        }

        clock_gettime(CLOCK_MILK, &t8);


        // select modes
        float evalmax = imgeval.im->array.F[nbmode - 1];
        int   ecnt    = 0;
        float evlim   = svdlim * svdlim;
        {
            int mi = 0;
            while (mi < nbmode && imgeval.im->array.F[mi] < evalmax * evlim)
            {
                mi++;
            }
            ecnt = nbmode - mi;
            printf("Selected %d modes\n", ecnt);
        }

        // create CMWFS and CMDM
        // contains strongest (highest singular values) modes from CMWFSall
        //
        processinfo_WriteMessage(processinfo, "create CMWFS and CMDM");


        IMGID imgCMWFS =
            imgid_make_from_name_3D("CMmodesWFS", imgRMWFS.md->size[0], imgRMWFS.md->size[1], ecnt);
        createimagefromIMGID(&imgCMWFS);

        IMGID imgCMDM =
            imgid_make_from_name_3D("CMmodesDM", imgRMDM.md->size[0], imgRMDM.md->size[1], ecnt);
        createimagefromIMGID(&imgCMDM);


        clock_gettime(CLOCK_MILK, &t9);


        //
        // Modes are normalized to RMS=1 in DM space
        //
#pragma omp parallel for schedule(static)
        for (int CMmode = 0; CMmode < ecnt; CMmode++)
        {
            int   mi       = nbmode - 1 - CMmode;
            float inv_norm = 1.0f / n2cmDM[mi];

            float       *dst_wfs = &imgCMWFS.im->array.F[(size_t) CMmode * nbwfspix];
            const float *src_wfs = &imgCMWFSall.im->array.F[(size_t) mi * nbwfspix];
            for (int ii = 0; ii < nbwfspix; ii++)
            {
                dst_wfs[ii] = src_wfs[ii] * inv_norm;
            }

            float       *dst_dm = &imgCMDM.im->array.F[(size_t) CMmode * nbact];
            const float *src_dm = &imgCMDMall.im->array.F[(size_t) mi * nbact];
            for (int ii = 0; ii < nbact; ii++)
            {
                dst_dm[ii] = src_dm[ii] * inv_norm;
            }
        }

        {
            double DMnormcnt  = 0.0;
            double WFSnormcnt = 0.0;
            for (int ii = 0; ii < nbact; ii++)
            {
                DMnormcnt += (double) imgDMmask.im->array.F[ii];
            }
            for (int ii = 0; ii < nbwfspix; ii++)
            {
                WFSnormcnt += (double) imgWFSmask.im->array.F[ii];
            }

#pragma omp parallel for schedule(static)
            for (int CMmode = 0; CMmode < ecnt; CMmode++)
            {
                double       WFSnorm = 0.0;
                const float *wfs_src = &imgCMWFS.im->array.F[(size_t) CMmode * nbwfspix];
                const float *wfs_msk = imgWFSmask.im->array.F;
                for (int ii = 0; ii < nbwfspix; ii++)
                {
                    double val = (double) wfs_src[ii];
                    WFSnorm += val * val * (double) wfs_msk[ii];
                }
                n2cmWFS[CMmode] = (float) sqrt(WFSnorm / WFSnormcnt);

                double       DMnorm = 0.0;
                const float *dm_src = &imgCMDM.im->array.F[(size_t) CMmode * nbact];
                const float *dm_msk = imgDMmask.im->array.F;
                for (int ii = 0; ii < nbact; ii++)
                {
                    double val = (double) dm_src[ii];
                    DMnorm += val * val * (double) dm_msk[ii];
                }
                n2cmDM[CMmode] = (float) sqrt(DMnorm / DMnormcnt);
            }

            FILE *fp = fopen("mkmodestmp/mode_norm_1.txt", "w");
            if (fp != NULL)
            {
                for (int CMmode = 0; CMmode < ecnt; CMmode++)
                {
                    fprintf(fp, "%4d    %20g    %20g \n", CMmode, n2cmDM[CMmode], n2cmWFS[CMmode]);
                }
                fclose(fp);
            }
        }


        free(n2cmDM);
        free(n2cmWFS);

        printf("SAVING FILES TO DISK \n");
        save_fits("VTmat", "./mkmodestmp/VTmat.fits");

        printf("writing CMmodesDM  to file %s\n", CMmodesDMfname);
        printf("writing CMmodesWFS to file %s\n", CMmodesWFSfname);

        save_fits("CMmodesDM", CMmodesDMfname);
        save_fits("CMmodesWFS", CMmodesWFSfname);

        {
            char abs_buf[PATH_MAX];
            const char *dm_p  = (CMmodesDMfname[0] == '!') ? (CMmodesDMfname + 1) : CMmodesDMfname;
            const char *wfs_p = (CMmodesWFSfname[0] == '!') ? (CMmodesWFSfname + 1) : CMmodesWFSfname;
            unlink("./mkmodestmp/CMmodesDM.fits");
            if (dm_p[0] == '\0' || realpath(dm_p, abs_buf) == NULL ||
                symlink(abs_buf, "./mkmodestmp/CMmodesDM.fits") != 0)
            {
                save_fits("CMmodesDM", "./mkmodestmp/CMmodesDM.fits");
            }
            unlink("./mkmodestmp/CMmodesWFS.fits");
            if (wfs_p[0] == '\0' || realpath(wfs_p, abs_buf) == NULL ||
                symlink(abs_buf, "./mkmodestmp/CMmodesWFS.fits") != 0)
            {
                save_fits("CMmodesWFS", "./mkmodestmp/CMmodesWFS.fits");
            }
        }
    }
    MILK_PROCINFO_END(self)


    struct timespec tdiff;


    tdiff       = timespec_diff(t0, t1);
    double t01d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t1, t2);
    double t12d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t2, t3);
    double t23d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t3, t4);
    double t34d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t4, t5);
    double t45d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t5, t6);
    double t56d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t6, t7);
    double t67d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t7, t8);
    double t78d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

    tdiff       = timespec_diff(t8, t9);
    double t89d = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;


    //    printf("GSL         %5.3f s\n", t01d);
    printf("total       %5.3f s\n", t01d + t12d + t23d + t34d + t45d + t56d + t67d + t78d + t89d);
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


const MILK_RUNNABLE runnable_computeCalib_compute_straight_CM = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "compsCM",
                 .cmdkey           = "compsCM",
                 .description      = "compute straight control matrix",
                 .description_long = "Compute a control matrix directly from a response matrix "
                                     "using straight pseudo-inversion without modal "
                                     "decomposition." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(compute_straight_CM),
    .compute = compute_straight_CM_compute,
};
EXPORT_FPSEXEC(cacao, "cacaocc-compstrcm", runnable_computeCalib_compute_straight_CM);
