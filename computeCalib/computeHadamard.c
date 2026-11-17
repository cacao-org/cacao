// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib_Hadamard.c
 * @brief   Compute Hadamard modes
 *
 */

// MILK_CMAKE_REQUEST_BLAS
// MILK_CMAKE_REQUEST_CUDA

#define _GNU_SOURCE

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "milk_runnable.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "milk_blas_lapacke.h"

#ifdef HAVE_CUDA
#    include <cublas_v2.h>
#    include <cuda_runtime.h>
#endif

#include "computeCalib.h"

#define MAX_MBLOCK 20

#ifdef _OPENMP
#    include <omp.h>
#    define OMP_NELEMENT_LIMIT 1000000
#endif


static char inmask[FUNCTION_PARAMETER_STRMAXLEN];
static char outHcube[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X)                                                                     \
    X(".inmask", inmask, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),     \
      "pixel mask (0 and 1 vals)")                                                        \
    X(".outHcube", outHcube, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "output Hadamard cube")

MILK_PARAMS_DEFINE(computeHadamard, FPS_PARAMS);


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    return RETURN_SUCCESS;
}


// output:
// Hadamard modes (outname)
// Hadamard matrix ("Hmat.fits")
// pixel indexes ("Hpixindex.fits", float, to be converted to long)
imageID AOloopControl_computeCalib_mkHadamardModes(const char *DMmask_name, const char *outname)
{
    imageID IDout;
    long    cnt;

    uint32_t Hsize;
    uint32_t n2max;

    long     *indexarray;
    long      index;
    imageID   IDmat;
    int      *Hmat;
    imageID   IDindex;

    imageID IDmask = image_ID(DMmask_name, dcimg, dcnimg);
    if (IDmask == -1)
    {
        IDmask = read_sharedmem_image(DMmask_name, dcimg, dcnimg);
    }
    if (IDmask == -1)
    {
        PRINT_ERROR("Cannot resolve mask %s", DMmask_name);
        return -1;
    }

    uint32_t xsize  = dcimg[IDmask].md[0].size[0];
    uint32_t ysize  = dcimg[IDmask].md[0].size[1];
    uint64_t xysize = xsize * ysize;

    create_2Dimage_ID("Hpixindex", xsize, ysize, &IDindex);

    cnt = 0;
    for (uint64_t ii = 0; ii < xysize; ii++)
    {
        if (dcimg[IDmask].array.F[ii] > 0.5)
        {
            cnt++;
        }
    }

    Hsize = 1;
    n2max = 0;
    while (Hsize < cnt)
    {
        Hsize *= 2;
        n2max++;
    }
    n2max++;

    printf("Hsize n2max = %u  %u\n", Hsize, n2max);
    fflush(stdout);

    for (uint64_t ii = 0; ii < xysize; ii++)
    {
        dcimg[IDindex].array.F[ii] = -10.0;
    }

    index = 0;

    indexarray = (long *) malloc(sizeof(long) * Hsize);
    if (indexarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    for (uint32_t k = 0; k < Hsize; k++)
    {
        indexarray[k] = -1;
    }
    for (uint64_t ii = 0; ii < xysize; ii++)
    {
        if ((dcimg[IDmask].array.F[ii] > 0.5) && (index < Hsize))
        {
            indexarray[index] = ii;
            // printf("(%ld %ld)  ", index, ii);

            dcimg[IDindex].array.F[ii] = 1.0 * index;

            index++;
        }
    }
    // save_fits("Hpixindex", "./conf/Hpixindex.fits");

    Hmat = (int *) malloc(sizeof(int) * Hsize * Hsize);
    if (Hmat == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    // n = 0

    uint32_t ii           = 0;
    uint32_t jj           = 0;
    Hmat[jj * Hsize + ii] = 1;
    uint32_t n2           = 1;
    for (uint32_t n = 1; n < n2max; n++)
    {
        for (uint32_t ii = 0; ii < n2; ii++)
        {
            for (uint32_t jj = 0; jj < n2; jj++)
            {
                Hmat[jj * Hsize + (ii + n2)]        = Hmat[jj * Hsize + ii];
                Hmat[(jj + n2) * Hsize + (ii + n2)] = -Hmat[jj * Hsize + ii];
                Hmat[(jj + n2) * Hsize + ii]        = Hmat[jj * Hsize + ii];
            }
        }
        n2 *= 2;
    }

    DEBUG_TRACEPOINT("n2 = %u", n2);

    create_2Dimage_ID("Hmat", Hsize, Hsize, &IDmat);

    for (uint32_t ii = 0; ii < Hsize; ii++)
    {
        for (uint32_t jj = 0; jj < Hsize; jj++)
        {
            dcimg[IDmat].array.F[jj * Hsize + ii] = Hmat[jj * Hsize + ii];
        }
    }

    //    save_fits("Htest", "./conf/Hmat.fits");

    DEBUG_TRACEPOINT("image %s size %u %u %u", outname, xsize, ysize, Hsize);
    create_3Dimage_ID(outname, xsize, ysize, Hsize, &IDout);
    list_image_ID();

    for (uint32_t k = 0; k < Hsize; k++)
    {
        for (uint32_t index = 0; index < Hsize; index++)
        {
            long ii = indexarray[index];

            if (ii >= 0)
            {
                DEBUG_TRACEPOINT("%u %u %ld", k, index, indexarray[index]);
                dcimg[IDout].array.F[k * xysize + ii] = Hmat[k * Hsize + index];
            }
        }
    }

    free(Hmat);

    free(indexarray);

    DEBUG_TRACEPOINT("exit function");

    return IDout;
}


static int is_sylvester_hadamard(const float *H, long N)
{
    if (H == NULL || N <= 0 || (N & (N - 1)) != 0 || fabsf(H[0] - 1.0f) > 1e-5f)
    {
        return 0;
    }
    for (long len = 1; len < N; len <<= 1)
    {
        for (long r = 0; r < len; r++)
        {
            const float *row0 = &H[r * N];
            const float *row1 = &H[(r + len) * N];
            for (long c = 0; c < len; c++)
            {
                float v = row0[c];
                if (fabsf(row0[c + len] - v) > 1e-5f || fabsf(row1[c] - v) > 1e-5f ||
                    fabsf(row1[c + len] + v) > 1e-5f)
                {
                    return 0;
                }
            }
        }
    }
    return 1;
}

static int fwht_decode_cube(
    const float *in, long sizewfs, long NBframes, float alpha, float *decbuf)
{
    const long tile_w = 64;
    long       ntiles = (sizewfs + tile_w - 1) / tile_w;
    int        oom    = 0;

#ifdef _OPENMP
#    pragma omp parallel reduction(| : oom)
#endif
    {
        float *tile = (float *) malloc(sizeof(float) * (size_t) NBframes * (size_t) tile_w);
        if (tile == NULL)
        {
            oom = 1;
        }
        else
        {
#ifdef _OPENMP
#            pragma omp for schedule(static)
#endif
            for (long t = 0; t < ntiles; t++)
            {
                long i0 = t * tile_w;
                long bw = sizewfs - i0;
                if (bw > tile_w)
                {
                    bw = tile_w;
                }
                for (long c = 0; c < NBframes; c++)
                {
                    memcpy(&tile[(size_t) c * (size_t) bw],
                           &in[(size_t) c * (size_t) sizewfs + (size_t) i0],
                           sizeof(float) * (size_t) bw);
                }
                for (long len = 1; len < NBframes; len <<= 1)
                {
                    long step = len << 1;
                    for (long start = 0; start < NBframes; start += step)
                    {
                        for (long j = 0; j < len; j++)
                        {
                            float *u = &tile[(size_t) (start + j) * (size_t) bw];
                            float *v = &tile[(size_t) (start + len + j) * (size_t) bw];
#ifdef _OPENMP
#                            pragma omp simd
#endif
                            for (long p = 0; p < bw; p++)
                            {
                                float a = u[p];
                                float b = v[p];
                                u[p]    = a + b;
                                v[p]    = a - b;
                            }
                        }
                    }
                }
                for (long c = 0; c < NBframes; c++)
                {
                    const float *src = &tile[(size_t) c * (size_t) bw];
                    float       *dst = &decbuf[(size_t) c * (size_t) sizewfs + (size_t) i0];
#ifdef _OPENMP
#                    pragma omp simd
#endif
                    for (long p = 0; p < bw; p++)
                    {
                        dst[p] = src[p] * alpha;
                    }
                }
            }
            free(tile);
        }
    }
    return oom ? -1 : 0;
}

imageID AOloopControl_computeCalib_Hadamard_decodeRM(const char *inname,
                                                     const char *Hmatname,
                                                     const char *indexname,
                                                     const char *outname)
{
    imageID  IDin, IDhad, IDout, IDindex;
    long     NBframes, sizexwfs, sizeywfs, sizewfs;
    long     kk, kk1, ii;
    uint32_t zsizeout;

    IDin = image_ID(inname, dcimg, dcnimg);
    if (IDin == -1)
    {
        IDin = read_sharedmem_image(inname, dcimg, dcnimg);
    }
    if (IDin == -1)
    {
        PRINT_ERROR("Cannot resolve input RM %s", inname);
        return -1;
    }
    sizexwfs = dcimg[IDin].md[0].size[0];
    sizeywfs = dcimg[IDin].md[0].size[1];
    sizewfs  = sizexwfs * sizeywfs;
    NBframes = dcimg[IDin].md[0].size[2];

    IDindex = image_ID(indexname, dcimg, dcnimg);
    if (IDindex == -1)
    {
        IDindex = read_sharedmem_image(indexname, dcimg, dcnimg);
    }
    if (IDindex == -1)
    {
        PRINT_ERROR("Cannot resolve index image %s", indexname);
        return -1;
    }

    IDhad = image_ID(Hmatname, dcimg, dcnimg);
    if (IDhad == -1)
    {
        IDhad = read_sharedmem_image(Hmatname, dcimg, dcnimg);
    }
    if (IDhad == -1)
    {
        PRINT_ERROR("Cannot resolve Hadamard matrix %s", Hmatname);
        return -1;
    }
    if ((dcimg[IDhad].md[0].size[0] != NBframes) || (dcimg[IDhad].md[0].size[1] != NBframes))
    {
        printf("ERROR: size of Hadamard matrix [%ld x %ld] does not match "
               "available number of frames [%ld]\n",
               (long) dcimg[IDhad].md[0].size[0], (long) dcimg[IDhad].md[0].size[1], NBframes);
        exit(0);
    }

    zsizeout = dcimg[IDindex].md[0].size[0] * dcimg[IDindex].md[0].size[1];
    create_3Dimage_ID(outname, sizexwfs, sizeywfs, zsizeout, &IDout);

    size_t n_in   = (size_t) sizewfs * (size_t) NBframes;
    float *decbuf = (float *) malloc(sizeof(float) * n_in);
    if (decbuf == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        return -1;
    }

    int   sgemm_done = 0;
    float alpha      = 1.0f / (float) NBframes;

    if (is_sylvester_hadamard(dcimg[IDhad].array.F, NBframes) &&
        fwht_decode_cube(dcimg[IDin].array.F, sizewfs, NBframes, alpha, decbuf) == 0)
    {
        sgemm_done = 1;
    }

#ifdef HAVE_CUDA
    if (!sgemm_done)
    {
        double flops   = 2.0 * (double) sizewfs * (double) NBframes * (double) NBframes;
        int    use_gpu = (getenv("CACAO_FORCE_CUDA") != NULL);
#    ifndef HAVE_BLAS
        use_gpu = 1;
#    else
        if (flops >= 4.0e12)
        {
            use_gpu = 1;
        }
#    endif
        if (use_gpu)
        {
            float beta    = 0.0f;
            int   dev_cnt = 0;
            if (cudaGetDeviceCount(&dev_cnt) == cudaSuccess && dev_cnt > 0)
            {
                float         *d_in   = NULL;
                float         *d_had  = NULL;
                float         *d_dec  = NULL;
                size_t         sz_in  = sizeof(float) * n_in;
                size_t         sz_had = sizeof(float) * (size_t) NBframes * (size_t) NBframes;
                cublasHandle_t handle = NULL;

                if (cudaMalloc((void **) &d_in, sz_in) == cudaSuccess &&
                    cudaMalloc((void **) &d_had, sz_had) == cudaSuccess &&
                    cudaMalloc((void **) &d_dec, sz_in) == cudaSuccess &&
                    cublasCreate(&handle) == CUBLAS_STATUS_SUCCESS)
                {
                    if (cudaMemcpy(d_in, dcimg[IDin].array.F, sz_in, cudaMemcpyHostToDevice) ==
                            cudaSuccess &&
                        cudaMemcpy(d_had, dcimg[IDhad].array.F, sz_had,
                                   cudaMemcpyHostToDevice) == cudaSuccess &&
                        cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, (int) sizewfs,
                                    (int) NBframes, (int) NBframes, &alpha, d_in, (int) sizewfs,
                                    d_had, (int) NBframes, &beta, d_dec, (int) sizewfs) ==
                            CUBLAS_STATUS_SUCCESS &&
                        cudaMemcpy(decbuf, d_dec, sz_in, cudaMemcpyDeviceToHost) == cudaSuccess)
                    {
                        sgemm_done = 1;
                    }
                }
                if (handle != NULL)
                {
                    cublasDestroy(handle);
                }
                cudaFree(d_in);
                cudaFree(d_had);
                cudaFree(d_dec);
            }
        }
    }
#endif

    if (!sgemm_done)
    {
#ifdef HAVE_BLAS
        cblas_sgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, (int) sizewfs, (int) NBframes,
                    (int) NBframes, alpha, dcimg[IDin].array.F, (int) sizewfs,
                    dcimg[IDhad].array.F, (int) NBframes, 0.0f, decbuf, (int) sizewfs);
        sgemm_done = 1;
#endif
    }

    if (!sgemm_done)
    {
        memset(decbuf, 0, sizeof(float) * n_in);
#ifdef _OPENMP
#    pragma omp parallel for private(kk1, ii)
#endif
        for (long c = 0; c < NBframes; c++)
        {
            float *dst = &decbuf[(size_t) c * (size_t) sizewfs];
            for (kk1 = 0; kk1 < NBframes; kk1++)
            {
                float        hval = dcimg[IDhad].array.F[c * NBframes + kk1] * alpha;
                const float *src  = &dcimg[IDin].array.F[(size_t) kk1 * (size_t) sizewfs];
                for (ii = 0; ii < sizewfs; ii++)
                {
                    dst[ii] += src[ii] * hval;
                }
            }
        }
    }

#ifdef _OPENMP
#    pragma omp parallel for schedule(static)
#endif
    for (kk = 0; kk < (long) zsizeout; kk++)
    {
        float *dst = &dcimg[IDout].array.F[(size_t) kk * (size_t) sizewfs];
        long   kk0 = (long) floorf(dcimg[IDindex].array.F[kk] + 0.5f);
        if (kk0 >= 0 && kk0 < NBframes)
        {
            memcpy(dst, &decbuf[(size_t) kk0 * (size_t) sizewfs],
                   sizeof(float) * (size_t) sizewfs);
        }
        else
        {
            memset(dst, 0, sizeof(float) * (size_t) sizewfs);
        }
    }

    free(decbuf);

    return (IDout);
}


static MILK_HOT errno_t computeHadamard_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();


    printf("INPUT  : %s\n", inmask);
    printf("OUTPUT : %s\n", outHcube);

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        AOloopControl_computeCalib_mkHadamardModes(inmask, outHcube);
    }
    MILK_PROCINFO_END(self)

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_computeCalib_computeHadamard = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "mkHadamard",
                 .cmdkey      = "mkHadamard",
                 .description = "make Hadamard modes",
                 .description_long =
                     "Generate Hadamard mode patterns for efficient response matrix acquisition. "
                     "Uses balanced Hadamard matrices to minimize measurement noise." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(computeHadamard),
    .compute   = computeHadamard_compute,
    .confcheck = customCONFcheck,
};
EXPORT_FPSEXEC(cacao, "cacaocc-comphadamard", runnable_computeCalib_computeHadamard);
