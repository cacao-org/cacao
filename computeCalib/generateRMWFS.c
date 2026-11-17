// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file generateRMWFS.c
 * @brief Generatermwfs module
 */

// MILK_CMAKE_REQUEST_BLAS
// MILK_CMAKE_REQUEST_CUDA

#include <stdlib.h>
#include <string.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "milk_blas_lapacke.h"

#ifdef HAVE_CUDA
#    include <cublas_v2.h>
#    include <cuda_runtime.h>
#endif

#include "computeCalib.h"


// zonal WFS response
//
static char zrespWFS[FUNCTION_PARAMETER_STRMAXLEN];
static char DMmodesC[FUNCTION_PARAMETER_STRMAXLEN];
static char outWFSmodesC[FUNCTION_PARAMETER_STRMAXLEN];


#define FPS_PARAMS(X)                                                                             \
    X(".zrespWFS", zrespWFS, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),     \
      "input zonal response matrix")                                                              \
    X(".DMmodesC", DMmodesC, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),     \
      "input DM modes")                                                                           \
    X(".outWFSmodesC", outWFSmodesC, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "output WFS modes")

MILK_PARAMS_DEFINE(generateRMWFS, FPS_PARAMS);


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
        milk_data.fpsptr->parray[functionparameter_GetParamIndex(milk_data.fpsptr, ".zrespWFS")]
            .fpflag |= FPFLAG_STREAM_RUN_REQUIRED;

        milk_data.fpsptr->parray[functionparameter_GetParamIndex(milk_data.fpsptr, ".DMmodesC")]
            .fpflag |= FPFLAG_STREAM_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    return RETURN_SUCCESS;
}


static MILK_HOT errno_t generateRMWFS_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    IMGID imgzRM = imgid_make_from_name(zrespWFS);
    resolveIMGID(&imgzRM, ERRMODE_WARN, dcimg, dcnimg);
    if (imgzRM.ID == -1)
    {
        return RETURN_FAILURE;
    }
    uint32_t wfsxsize = imgzRM.md->size[0];
    uint32_t wfsysize = imgzRM.md->size[1];
    uint64_t wfssize  = wfsxsize;
    wfssize *= wfsysize;
    printf("wfssize = %lu\n", wfssize);


    IMGID imDMmodesC = imgid_make_from_name(DMmodesC);
    resolveIMGID(&imDMmodesC, ERRMODE_WARN, dcimg, dcnimg);
    if (imDMmodesC.ID == -1)
    {
        return RETURN_FAILURE;
    }
    uint32_t dmxsize = imDMmodesC.md->size[0];
    uint32_t dmysize = imDMmodesC.md->size[1];
    uint64_t dmsize  = dmxsize;
    dmsize *= dmysize;
    printf("dmsize = %lu\n", dmsize);

    uint32_t NBmodes = imDMmodesC.md->size[2];
    printf("%u modes\n", NBmodes);


    IMGID imgoutWFSc = imgid_make_from_name_3D(outWFSmodesC, wfsxsize, wfsysize, NBmodes);
    createimagefromIMGID(&imgoutWFSc);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        imgoutWFSc.md->write = 1;
        int         sgemm_done = 0;
        const float alpha      = 1.0f;
        const float beta       = 0.0f;

#ifdef HAVE_CUDA
        {
            double flops   = 2.0 * (double) wfssize * (double) dmsize * (double) NBmodes;
            int    use_gpu = (getenv("CACAO_FORCE_CUDA") != NULL);
#    ifndef HAVE_BLAS
            use_gpu = 1;
#    else
            if (flops >= 4.0e12)
            {
                use_gpu = 1;
            }
#    endif
            int dev_cnt = 0;
            if (use_gpu && cudaGetDeviceCount(&dev_cnt) == cudaSuccess && dev_cnt > 0)
            {
                float         *d_zrm = NULL;
                float         *d_dm  = NULL;
                float         *d_out = NULL;
                size_t         sz_z  = sizeof(float) * wfssize * dmsize;
                size_t         sz_d  = sizeof(float) * dmsize * (size_t) NBmodes;
                size_t         sz_o  = sizeof(float) * wfssize * (size_t) NBmodes;
                cublasHandle_t handle = NULL;

                if (cudaMalloc((void **) &d_zrm, sz_z) == cudaSuccess &&
                    cudaMalloc((void **) &d_dm, sz_d) == cudaSuccess &&
                    cudaMalloc((void **) &d_out, sz_o) == cudaSuccess &&
                    cublasCreate(&handle) == CUBLAS_STATUS_SUCCESS)
                {
                    if (cudaMemcpy(d_zrm, imgzRM.im->array.F, sz_z, cudaMemcpyHostToDevice) ==
                            cudaSuccess &&
                        cudaMemcpy(d_dm, imDMmodesC.im->array.F, sz_d,
                                   cudaMemcpyHostToDevice) == cudaSuccess &&
                        cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, (int) wfssize,
                                    (int) NBmodes, (int) dmsize, &alpha, d_zrm, (int) wfssize,
                                    d_dm, (int) dmsize, &beta, d_out, (int) wfssize) ==
                            CUBLAS_STATUS_SUCCESS &&
                        cudaMemcpy(imgoutWFSc.im->array.F, d_out, sz_o,
                                   cudaMemcpyDeviceToHost) == cudaSuccess)
                    {
                        sgemm_done = 1;
                    }
                }
                if (handle != NULL)
                {
                    cublasDestroy(handle);
                }
                cudaFree(d_zrm);
                cudaFree(d_dm);
                cudaFree(d_out);
            }
        }
#endif

        if (!sgemm_done)
        {
#ifdef HAVE_BLAS
            cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, (int) wfssize, (int) NBmodes,
                        (int) dmsize, 1.0f, imgzRM.im->array.F, (int) wfssize,
                        imDMmodesC.im->array.F, (int) dmsize, 0.0f, imgoutWFSc.im->array.F,
                        (int) wfssize);
            sgemm_done = 1;
#endif
        }

        if (!sgemm_done)
        {
            memset(imgoutWFSc.im->array.F, 0, sizeof(float) * wfssize * (size_t) NBmodes);
            for (uint32_t mode = 0; mode < NBmodes; mode++)
            {
                for (uint64_t iidm = 0; iidm < dmsize; iidm++)
                {
                    float w = imDMmodesC.im->array.F[dmsize * mode + iidm];
                    for (uint64_t iiwfs = 0; iiwfs < wfssize; iiwfs++)
                    {
                        imgoutWFSc.im->array.F[wfssize * mode + iiwfs] +=
                            w * imgzRM.im->array.F[wfssize * iidm + iiwfs];
                    }
                }
            }
        }

        processinfo_update_output_stream(processinfo, imgoutWFSc.im, NULL);
    }
    MILK_PROCINFO_END(self)


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_computeCalib_generateRMWFS = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "generateRMWFS",
                 .cmdkey           = "generateRMWFS",
                 .description      = "generate RM WFS modes",
                 .description_long = "Generate synthetic response matrix modes for WFS testing. "
                                     "Creates ideal poke responses for simulation and "
                                     "validation." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(generateRMWFS),
    .compute   = generateRMWFS_compute,
    .confcheck = customCONFcheck,
};
EXPORT_FPSEXEC(cacao, "cacaocc-genrmwfs", runnable_computeCalib_generateRMWFS);
