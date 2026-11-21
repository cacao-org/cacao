// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalog.c
 * @brief   Online single-stage optical-gain estimator via closed-loop DM pokes (exogenous IV)
 *
 * Subscribes asynchronously to double-buffered modal telemetry produced by cacao-fpsexec-mfilt:
 *   - aol<N>_modevalWFS_buff   (closed-loop WFS modal residuals w[k])
 *   - aol<N>_modevalDM_buff    (controller modal commands u_ctrl[k])
 *   - aol<N>_modevalprobe_buff (exogenous zero-mean probe commands p[k])
 *
 * Uses the lab-calibrated WFS-rate 3D dynamic impulse kernel H_eff[l] (from cacaoac-factordynrm,
 * supporting 3D MIMO, 2D diagonal, uniform broadcast, or 2-tap fractional delay fallback) to form:
 *   y_dm[k] = (H_eff * (u_ctrl + p))[k]   (known physical DM response)
 *   z[k]    = (H_eff * p)[k]              (strictly exogenous matched-filtered instrument)
 *
 * Applying an identical optional first temporal difference Delta x[k] = x[k] - x[k-1] removes
 * low-frequency atmospheric turbulence variance by 25-100x with exact zero bias, yielding the
 * controller-invariant Instrumental-Variables optical gain estimator:
 *
 *   g_hat[m] = sum_k (Dz_m[k] * Dw_m[k]) / sum_k (Dz_m[k] * Dy_dm,m[k])
 *
 * Followed by block-sum autocorrelation-robust standard errors, inverse-variance Gaussian radial
 * spatial-frequency smoothing (keeping leading n_free TT/petal modes individually free), safety
 * clamping, EMA tracking, and optional live update of aol<N>_mWFSfact[m] = 1 / g_smooth[m].
 */

// MILK_CMAKE_REQUEST_BLAS

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_BLAS
#    include "milk_blas_lapacke.h"
#endif

#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl.h"

#define MOG_MAX_TAPS 32

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint64_t p_loopindex                              = 0;
static char     p_inwfs[FUNCTION_PARAMETER_STRMAXLEN]    = "aol0_modevalWFS_buff";
static char     p_indm[FUNCTION_PARAMETER_STRMAXLEN]     = "aol0_modevalDM_buff";
static char     p_inprobe[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalprobe_buff";
static char     p_kernel[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static float    p_latency_fr                             = 2.0f;
static uint32_t p_nbwin                                  = 2048;
static int32_t  p_use_diff                               = 1;
static uint32_t p_n_free                                 = 2;
static float    p_smooth_bw                              = 1.5f;
static float    p_snr_min                                = 3.0f;
static float    p_g_min                                  = 0.20f;
static float    p_g_max                                  = 2.50f;
static float    p_ema_alpha                              = 0.35f;
static int32_t  p_apply                                  = 0;

static uint32_t o_n_samples     = 0;
static uint32_t o_n_valid_modes = 0;
static uint32_t o_kern_mode     = 0;
static uint64_t o_applied_cnt   = 0;
static float    o_og_med        = 1.0f;
static float    o_og_lo_med     = 1.0f;
static float    o_og_hi_med     = 1.0f;
static float    o_err_med       = 0.0f;
static float    o_snr_med       = 0.0f;
static float    o_probe_rms     = 0.0f;

#define FPS_PARAMS_MODALOG(X)                                                                      \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                            \
    X(".inwfs", p_inwfs, FPTYPE_STREAMNAME, 1,                                                     \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input WFS double buffer")         \
    X(".indm", p_indm, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,              \
      "input DM controller double buffer")                                                         \
    X(".inprobe", p_inprobe, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "input exogenous OG probe double buffer")                                                    \
    X(".kernel", p_kernel, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "optional H_mimo/H_diag FITS kernel (empty = 2-tap)")                                        \
    X(".latency_fr", &p_latency_fr, FPTYPE_FLOAT32, 0,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fallback 2-tap latency [fr]")    \
    X(".NBwin", &p_nbwin, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "sliding estimation window [frames]")                                                        \
    X(".use_diff", &p_use_diff, FPTYPE_ONOFF, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "first-diff variance reduction")  \
    X(".n_free", &p_n_free, FPTYPE_UINT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "leading unsmoothed TT/petals")   \
    X(".smooth_bw", &p_smooth_bw, FPTYPE_FLOAT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "radial smooth bw (0 = off)")     \
    X(".snr_min", &p_snr_min, FPTYPE_FLOAT32, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min IV SNR gate")                \
    X(".g_min", &p_g_min, FPTYPE_FLOAT32, 0,                                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "lower optical-gain clamp")       \
    X(".g_max", &p_g_max, FPTYPE_FLOAT32, 0,                                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "upper optical-gain clamp")       \
    X(".ema_alpha", &p_ema_alpha, FPTYPE_FLOAT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EMA update weight (0..1]")       \
    X(".apply", &p_apply, FPTYPE_ONOFF, 0,                                                         \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "write 1/g_opt to mWFSfact")      \
    X(".out.n_samples", &o_n_samples, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "active frames in window")                                                                   \
    X(".out.n_valid_modes", &o_n_valid_modes, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "probed modes passing SNR gate")                                                             \
    X(".out.kern_mode", &o_kern_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "0=2-tap 1=diag FIR 2=MIMO FIR")                                                             \
    X(".out.applied_cnt", &o_applied_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "live mWFSfact updates written")                                                             \
    X(".out.og_med", &o_og_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "median smoothed optical gain")                                                              \
    X(".out.og_lo_med", &o_og_lo_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "median optical gain (modes 0..19)")                                                         \
    X(".out.og_hi_med", &o_og_hi_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "median optical gain (modes 20+)")                                                           \
    X(".out.err_med", &o_err_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "median IV 1-sigma error")                                                                   \
    X(".out.snr_med", &o_snr_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "median IV SNR across probed modes")                                                         \
    X(".out.probe_rms", &o_probe_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "total RMS of active DM probes")

MILK_PARAMS_DEFINE(modalog, FPS_PARAMS_MODALOG);

/* ================================================================
 * 2.  HELPERS (KERNEL LOADER, MEDIAN, CONVOLUTION)
 * ============================================================= */

typedef struct
{
    uint32_t mode; /* 0=2tap, 1=diag, 2=MIMO */
    uint32_t ntaps;
    uint32_t nmode;
    float   *h_diag; /* [ntaps * nmode] */
    float   *h_mimo; /* [ntaps * nmode * nmode] */
    uint8_t  tap_nz[MOG_MAX_TAPS];
} MOG_KERN;

static void mog_kern_free(MOG_KERN *k)
{
    free(k->h_diag);
    free(k->h_mimo);
    memset(k, 0, sizeof(*k));
}

static int mog_load_kernel(MOG_KERN *k, uint32_t nbmode, const char *raw_path)
{
    mog_kern_free(k);
    if (raw_path == NULL || raw_path[0] == '\0' || strcmp(raw_path, "none") == 0)
    {
        return 0;
    }

#ifdef USE_CFITSIO
    const char *path   = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    fitsfile   *fptr   = NULL;
    int         status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        fprintf(stderr, "modalog: cannot open kernel '%s'\n", path);
        return -1;
    }
    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || (naxis != 2 && naxis != 3))
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    fits_get_img_size(fptr, naxis, naxes, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    LONGLONG nelem = (LONGLONG) naxes[0] * (LONGLONG) naxes[1] *
                     ((naxis == 3) ? (LONGLONG) naxes[2] : 1LL);
    float *raw = (float *) malloc(sizeof(float) * (size_t) nelem);
    if (raw == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, nelem, NULL, raw, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(raw);
        return -1;
    }

    if (naxis == 2)
    {
        uint32_t f_nin = (uint32_t) naxes[0];
        uint32_t f_L   = (uint32_t) naxes[1];
        if (f_nin == 0 || f_L < 2 || f_L > MOG_MAX_TAPS)
        {
            free(raw);
            return -1;
        }
        uint32_t n_dyn = (f_nin == 1) ? nbmode : ((f_nin < nbmode) ? f_nin : nbmode);
        float   *hd    = (float *) calloc((size_t) f_L * n_dyn, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            float mv = 0.0f;
            for (uint32_t m = 0; m < n_dyn; m++)
            {
                float v                  = (f_nin == 1) ? raw[l] : raw[(size_t) l * f_nin + m];
                hd[(size_t) l * n_dyn + m] = v;
                float av                 = fabsf(v);
                if (av > mv)
                {
                    mv = av;
                }
            }
            k->tap_nz[l] = (mv > 1.0e-7f) ? 1u : 0u;
        }
        free(raw);
        k->mode   = 1;
        k->ntaps  = f_L;
        k->nmode  = n_dyn;
        k->h_diag = hd;
        return 0;
    }

    uint32_t f_nin = (uint32_t) naxes[0];
    uint32_t f_ns  = (uint32_t) naxes[1];
    uint32_t f_L   = (uint32_t) naxes[2];
    if (f_nin == 0 || f_nin != f_ns || f_L < 2 || f_L > MOG_MAX_TAPS)
    {
        free(raw);
        return -1;
    }
    if (f_nin == 1)
    {
        float *hd = (float *) calloc((size_t) f_L * nbmode, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            float v = raw[l];
            for (uint32_t m = 0; m < nbmode; m++)
            {
                hd[(size_t) l * nbmode + m] = v;
            }
            k->tap_nz[l] = (fabsf(v) > 1.0e-7f) ? 1u : 0u;
        }
        free(raw);
        k->mode   = 1;
        k->ntaps  = f_L;
        k->nmode  = nbmode;
        k->h_diag = hd;
        return 0;
    }

    uint32_t n_dyn = (f_nin < nbmode) ? f_nin : nbmode;
    float   *hm    = (float *) calloc((size_t) f_L * n_dyn * n_dyn, sizeof(float));
    if (hm == NULL)
    {
        free(raw);
        return -1;
    }
    for (uint32_t l = 0; l < f_L; l++)
    {
        float mv = 0.0f;
        for (uint32_t s = 0; s < n_dyn; s++)
        {
            const float *src = &raw[((size_t) l * f_nin + s) * f_nin];
            float       *dst = &hm[((size_t) l * n_dyn + s) * n_dyn];
            for (uint32_t q = 0; q < n_dyn; q++)
            {
                float v  = src[q];
                dst[q]   = v;
                float av = fabsf(v);
                if (av > mv)
                {
                    mv = av;
                }
            }
        }
        k->tap_nz[l] = (mv > 1.0e-7f) ? 1u : 0u;
    }
    free(raw);
    k->mode   = 2;
    k->ntaps  = f_L;
    k->nmode  = n_dyn;
    k->h_mimo = hm;
    return 0;
#else
    (void) nbmode;
    return -1;
#endif
}

static int mog_cmp_f64(const void *a, const void *b)
{
    double da = *(const double *) a;
    double db = *(const double *) b;
    return (da > db) - (da < db);
}

static float mog_median_f64(double *vals, uint32_t n)
{
    if (n == 0)
    {
        return 0.0f;
    }
    qsort(vals, n, sizeof(double), mog_cmp_f64);
    if ((n & 1u) != 0u)
    {
        return (float) vals[n / 2];
    }
    return (float) (0.5 * (vals[n / 2 - 1] + vals[n / 2]));
}

/**
 * @brief Convolve contiguous [n_fr x nbmode] input signal X with causal kernel H_eff (l = 1..L-1)
 *        (and 2-tap fractional delay fallback for modes >= n_dyn) into Y [n_fr x nbmode].
 */
static void mog_apply_kernel(const MOG_KERN *k,
                             float           lat_fr,
                             const float    *X,
                             float          *Y,
                             uint32_t        n_fr,
                             uint32_t        nbmode,
                             uint32_t        t0)
{
    memset(Y, 0, sizeof(float) * (size_t) n_fr * nbmode);
    uint32_t n_dyn = (k->mode > 0) ? k->nmode : 0u;
    uint32_t L     = (k->mode > 0) ? k->ntaps : 0u;

    if (k->mode == 1 && n_dyn > 0)
    {
        for (uint32_t l = 1; l < L; l++)
        {
            if (!k->tap_nz[l])
            {
                continue;
            }
            const float *h_l = &k->h_diag[(size_t) l * n_dyn];
            for (uint32_t t = t0; t < n_fr; t++)
            {
                const float *x_l = &X[(size_t) (t - l) * nbmode];
                float       *y_t = &Y[(size_t) t * nbmode];
                for (uint32_t m = 0; m < n_dyn; m++)
                {
                    y_t[m] += h_l[m] * x_l[m];
                }
            }
        }
    }
    else if (k->mode == 2 && n_dyn > 0)
    {
        uint32_t nrows = (n_fr > t0) ? (n_fr - t0) : 0u;
        for (uint32_t l = 1; l < L && nrows > 0; l++)
        {
            if (!k->tap_nz[l])
            {
                continue;
            }
            const float *h_l = &k->h_mimo[(size_t) l * n_dyn * n_dyn];
#ifdef HAVE_BLAS
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) nrows, (int) n_dyn,
                        (int) n_dyn, 1.0f, &X[(size_t) (t0 - l) * nbmode], (int) nbmode, h_l,
                        (int) n_dyn, 1.0f, &Y[(size_t) t0 * nbmode], (int) nbmode);
#else
            for (uint32_t t = t0; t < n_fr; t++)
            {
                const float *x_l = &X[(size_t) (t - l) * nbmode];
                float       *y_t = &Y[(size_t) t * nbmode];
                for (uint32_t s = 0; s < n_dyn; s++)
                {
                    const float *hr  = &h_l[(size_t) s * n_dyn];
                    float        acc = 0.0f;
                    for (uint32_t q = 0; q < n_dyn; q++)
                    {
                        acc += hr[q] * x_l[q];
                    }
                    y_t[s] += acc;
                }
            }
#endif
        }
    }

    // 2-tap fractional latency fallback for modes m = n_dyn .. nbmode - 1
    if (n_dyn < nbmode)
    {
        float lf_tot = (lat_fr < 1.0f) ? 1.0f : ((lat_fr > 28.0f) ? 28.0f : lat_fr);
        uint32_t li  = (uint32_t) lf_tot;
        float    lf  = lf_tot - (float) li;
        float    w0  = 1.0f - lf;
        for (uint32_t t = t0; t < n_fr; t++)
        {
            const float *x0  = &X[(size_t) (t - li) * nbmode];
            const float *x1  = &X[(size_t) (t - li - 1) * nbmode];
            float       *y_t = &Y[(size_t) t * nbmode];
            for (uint32_t m = n_dyn; m < nbmode; m++)
            {
                y_t[m] = w0 * x0[m] + lf * x1[m];
            }
        }
    }
}

/* ================================================================
 * 3.  MAIN COMPUTE UNIT
 * ============================================================= */

static MILK_HOT errno_t modalog_compute(const MILK_RUNNABLE *self)
{
    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

    IMGID imgwfs = imgid_make_from_fpskey(p_inwfs, dcfpsname, ".inwfs");
    resolveIMGID(&imgwfs, ERRMODE_WARN, dcimg, dcnimg);
    if (imgwfs.ID == -1)
    {
        return RETURN_FAILURE;
    }

    IMGID imgdm = imgid_make_from_fpskey(p_indm, dcfpsname, ".indm");
    resolveIMGID(&imgdm, ERRMODE_WARN, dcimg, dcnimg);
    if (imgdm.ID == -1)
    {
        return RETURN_FAILURE;
    }

    IMGID imgprb = imgid_make_from_fpskey(p_inprobe, dcfpsname, ".inprobe");
    resolveIMGID(&imgprb, ERRMODE_WARN, dcimg, dcnimg);
    if (imgprb.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode    = imgwfs.md[0].size[0];
    uint32_t slice_len = (imgwfs.md[0].naxis >= 2) ? imgwfs.md[0].size[1] : 1u;
    uint32_t n_slices  = (imgwfs.md[0].naxis >= 3) ? imgwfs.md[0].size[2] : 1u;
    uint32_t nbwin     = (p_nbwin >= slice_len) ? p_nbwin : slice_len;
    if (nbwin < 64)
    {
        nbwin = 64;
    }

    MOG_KERN kern;
    memset(&kern, 0, sizeof(kern));
    if (p_kernel[0] != '\0' && strcmp(p_kernel, "none") != 0)
    {
        if (mog_load_kernel(&kern, NBmode, p_kernel) != 0)
        {
            fprintf(stderr, "modalog: failed loading '%s', falling back to 2-tap\n", p_kernel);
            mog_kern_free(&kern);
        }
    }
    o_kern_mode = kern.mode;

    // Create output SHM streams and connect to aol<N>_mWFSfact
    char  sname[STRINGMAXLEN_STREAMNAME];
    WRITE_IMAGENAME(sname, "aol%lu_og_raw", p_loopindex);
    IMGID img_raw = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_og_err", p_loopindex);
    IMGID img_err = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_og_smooth", p_loopindex);
    IMGID img_smt = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_og_snr", p_loopindex);
    IMGID img_snr = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mWFSfact", p_loopindex);
    IMGID img_mwfs = stream_connect_create_2Df32(sname, NBmode, 1);

    // Initialize smoothed OG state to 1.0
    float *g_ema = (float *) malloc(sizeof(float) * NBmode);
    for (uint32_t m = 0; m < NBmode; m++)
    {
        g_ema[m]                   = 1.0f;
        img_smt.im->array.F[m]     = 1.0f;
    }
    ImageStreamIO_UpdateIm(img_smt.im);

    // Sliding ring buffers + contiguous chronological workspaces
    size_t ring_el   = (size_t) nbwin * NBmode;
    float *wfs_ring  = (float *) calloc(ring_el, sizeof(float));
    float *uapp_ring = (float *) calloc(ring_el, sizeof(float));
    float *prb_ring  = (float *) calloc(ring_el, sizeof(float));
    float *W_lin     = (float *) calloc(ring_el, sizeof(float));
    float *U_lin     = (float *) calloc(ring_el, sizeof(float));
    float *P_lin     = (float *) calloc(ring_el, sizeof(float));
    float *Y_dm      = (float *) calloc(ring_el, sizeof(float));
    float *Z_prb     = (float *) calloc(ring_el, sizeof(float));

    double  *g_raw   = (double *) calloc(NBmode, sizeof(double));
    double  *g_se    = (double *) calloc(NBmode, sizeof(double));
    double  *g_snr   = (double *) calloc(NBmode, sizeof(double));
    double  *g_sm    = (double *) calloc(NBmode, sizeof(double));
    uint8_t *m_valid = (uint8_t *) calloc(NBmode, sizeof(uint8_t));
    double  *med_buf = (double *) calloc(NBmode, sizeof(double));

    if (!g_ema || !wfs_ring || !uapp_ring || !prb_ring || !W_lin || !U_lin || !P_lin || !Y_dm ||
        !Z_prb || !g_raw || !g_se || !g_snr || !g_sm || !m_valid || !med_buf)
    {
        mog_kern_free(&kern);
        free(g_ema);
        free(wfs_ring);
        free(uapp_ring);
        free(prb_ring);
        free(W_lin);
        free(U_lin);
        free(P_lin);
        free(Y_dm);
        free(Z_prb);
        free(g_raw);
        free(g_se);
        free(g_snr);
        free(g_sm);
        free(m_valid);
        free(med_buf);
        return RETURN_FAILURE;
    }

    uint32_t ring_head = 0;
    uint32_t ring_fill = 0;
    int      ema_init  = 0;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        o_kern_mode = kern.mode;
        uint32_t done_sl = (n_slices > 1u) ? (uint32_t) (imgwfs.md[0].cnt1 % n_slices) : 0u;
        size_t   sl_off  = (size_t) done_sl * slice_len * NBmode;
        const float *w_src = &imgwfs.im->array.F[sl_off];
        const float *d_src = &imgdm.im->array.F[sl_off];
        const float *p_src = &imgprb.im->array.F[sl_off];

        // Append newly arrived slice into sliding ring buffer (u_applied = u_ctrl + p)
        for (uint32_t t = 0; t < slice_len; t++)
        {
            size_t       s_off = (size_t) t * NBmode;
            size_t       r_off = (size_t) ring_head * NBmode;
            const float *ws    = &w_src[s_off];
            const float *ds    = &d_src[s_off];
            const float *ps    = &p_src[s_off];
            float       *wd    = &wfs_ring[r_off];
            float       *ud    = &uapp_ring[r_off];
            float       *pd    = &prb_ring[r_off];
            for (uint32_t m = 0; m < NBmode; m++)
            {
                wd[m] = ws[m];
                ud[m] = ds[m] + ps[m];
                pd[m] = ps[m];
            }
            ring_head = (ring_head + 1u) % nbwin;
            if (ring_fill < nbwin)
            {
                ring_fill++;
            }
        }
        o_n_samples = ring_fill;

        // Unwrap ring chronologically into [0 .. ring_fill - 1]
        uint32_t start = (ring_fill < nbwin) ? 0u : ring_head;
        uint32_t n1    = nbwin - start;
        if (n1 > ring_fill)
        {
            n1 = ring_fill;
        }
        uint32_t n2 = ring_fill - n1;
        memcpy(W_lin, &wfs_ring[(size_t) start * NBmode], sizeof(float) * (size_t) n1 * NBmode);
        memcpy(U_lin, &uapp_ring[(size_t) start * NBmode], sizeof(float) * (size_t) n1 * NBmode);
        memcpy(P_lin, &prb_ring[(size_t) start * NBmode], sizeof(float) * (size_t) n1 * NBmode);
        if (n2 > 0)
        {
            memcpy(&W_lin[(size_t) n1 * NBmode], wfs_ring, sizeof(float) * (size_t) n2 * NBmode);
            memcpy(&U_lin[(size_t) n1 * NBmode], uapp_ring, sizeof(float) * (size_t) n2 * NBmode);
            memcpy(&P_lin[(size_t) n1 * NBmode], prb_ring, sizeof(float) * (size_t) n2 * NBmode);
        }

        uint32_t max_lag = (kern.mode > 0) ? kern.ntaps : ((uint32_t) p_latency_fr + 2u);
        if (max_lag < 2u)
        {
            max_lag = 2u;
        }
        uint32_t t0 = max_lag + (p_use_diff ? 1u : 0u);
        if (ring_fill > t0 + 16u)
        {

        // Convolve U_lin and P_lin through H_eff[l]
        mog_apply_kernel(&kern, p_latency_fr, U_lin, Y_dm, ring_fill, NBmode, max_lag);
        mog_apply_kernel(&kern, p_latency_fr, P_lin, Z_prb, ring_fill, NBmode, max_lag);

        // Compute total probe RMS across window
        double prb_ss = 0.0;
        for (uint32_t t = t0; t < ring_fill; t++)
        {
            const float *pt = &P_lin[(size_t) t * NBmode];
            for (uint32_t m = 0; m < NBmode; m++)
            {
                prb_ss += (double) pt[m] * (double) pt[m];
            }
        }
        o_probe_rms = (float) sqrt(prb_ss / (double) (ring_fill - t0));

        // Per-mode unbiased IV estimator + block-sum autocorrelation-robust SE
        uint32_t n_eval    = ring_fill - t0;
        uint32_t blk_len   = (max_lag > 8u) ? max_lag : 8u;
        uint32_t n_blk     = (n_eval >= 2u * blk_len) ? (n_eval / blk_len) : 1u;
        float    g_lo_clmp = (p_g_min > 0.05f) ? p_g_min : 0.05f;
        float    g_hi_clmp = (p_g_max > g_lo_clmp) ? p_g_max : (g_lo_clmp + 1.0f);
        double   snr_gate  = (p_snr_min > 0.0f) ? (double) p_snr_min : 3.0;
        int      use_diff  = (p_use_diff != 0);

        uint32_t n_valid = 0;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            double s_zw = 0.0;
            double s_zy = 0.0;
            double s_zz = 0.0;
            for (uint32_t t = t0; t < ring_fill; t++)
            {
                size_t idx  = (size_t) t * NBmode + m;
                size_t pidx = (size_t) (t - 1u) * NBmode + m;
                double z    = use_diff ? ((double) Z_prb[idx] - (double) Z_prb[pidx])
                                       : (double) Z_prb[idx];
                double w    = use_diff ? ((double) W_lin[idx] - (double) W_lin[pidx])
                                       : (double) W_lin[idx];
                double y    = use_diff ? ((double) Y_dm[idx] - (double) Y_dm[pidx])
                                       : (double) Y_dm[idx];
                s_zw       += z * w;
                s_zy       += z * y;
                s_zz       += z * z;
            }

            g_raw[m]   = 1.0;
            g_se[m]    = 0.0;
            g_snr[m]   = 0.0;
            m_valid[m] = 0u;

            if (s_zz <= 1.0e-20 || fabs(s_zy) <= 1.0e-20)
            {
                continue;
            }
            double gh = s_zw / s_zy;
            if (!isfinite(gh))
            {
                continue;
            }

            // Autocorrelation-robust variance via non-overlapping temporal blocks
            double s_blk2 = 0.0;
            for (uint32_t b = 0; b < n_blk; b++)
            {
                uint32_t bt0 = t0 + b * blk_len;
                uint32_t bt1 = (b + 1u == n_blk) ? ring_fill : (bt0 + blk_len);
                double   sb  = 0.0;
                for (uint32_t t = bt0; t < bt1; t++)
                {
                    size_t idx  = (size_t) t * NBmode + m;
                    size_t pidx = (size_t) (t - 1u) * NBmode + m;
                    double z    = use_diff ? ((double) Z_prb[idx] - (double) Z_prb[pidx])
                                           : (double) Z_prb[idx];
                    double w    = use_diff ? ((double) W_lin[idx] - (double) W_lin[pidx])
                                           : (double) W_lin[idx];
                    double y    = use_diff ? ((double) Y_dm[idx] - (double) Y_dm[pidx])
                                           : (double) Y_dm[idx];
                    sb         += z * (w - gh * y);
                }
                s_blk2 += sb * sb;
            }
            double se  = sqrt(s_blk2) / fabs(s_zy);
            double snr = (se > 1.0e-12) ? (fabs(gh) / se) : 1.0e6;

            g_raw[m] = gh;
            g_se[m]  = se;
            g_snr[m] = snr;
            if (snr >= snr_gate && gh >= 0.5 * (double) g_lo_clmp &&
                gh <= 2.0 * (double) g_hi_clmp)
            {
                m_valid[m] = 1u;
                n_valid++;
            }
        }
        o_n_valid_modes = n_valid;

        // Smooth optical gain curve over radial coordinate f_m = sqrt(m + 1)
        // Leading n_free modes (TT/petals) remain individually unsmoothed
        uint32_t n_free = (p_n_free < NBmode) ? p_n_free : NBmode;
        double   bw     = (double) p_smooth_bw;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            double g_est = (double) g_ema[m];
            if (m < n_free || bw <= 1.0e-6)
            {
                if (m_valid[m])
                {
                    g_est = g_raw[m];
                }
            }
            else if (n_valid > 0)
            {
                // 1st-order local-linear weighted regression in radial coordinate f_m = sqrt(m+1)
                // (eliminates Nadaraya-Watson boundary bias on sloped PyWFS OG profiles)
                double fm   = sqrt((double) (m + 1u));
                double inv2 = 0.5 / (bw * bw);
                double s0   = 0.0;
                double s1   = 0.0;
                double s2   = 0.0;
                double t0   = 0.0;
                double t1   = 0.0;
                for (uint32_t j = n_free; j < NBmode; j++)
                {
                    if (!m_valid[j])
                    {
                        continue;
                    }
                    double fj  = sqrt((double) (j + 1u));
                    double df  = fj - fm;
                    double kwt = exp(-df * df * inv2);
                    double se2 = g_se[j] * g_se[j] + 1.0e-6;
                    double wt  = kwt / se2;
                    s0        += wt;
                    s1        += wt * df;
                    s2        += wt * df * df;
                    t0        += wt * g_raw[j];
                    t1        += wt * df * g_raw[j];
                }
                double det = s0 * s2 - s1 * s1;
                if (det > 1.0e-12 * s0 * s2 && s0 > 1.0e-18)
                {
                    g_est = (s2 * t0 - s1 * t1) / det;
                }
                else if (s0 > 1.0e-18)
                {
                    g_est = t0 / s0;
                }
                else if (m_valid[m])
                {
                    g_est = g_raw[m];
                }
            }

            if (g_est < (double) g_lo_clmp)
            {
                g_est = (double) g_lo_clmp;
            }
            if (g_est > (double) g_hi_clmp)
            {
                g_est = (double) g_hi_clmp;
            }
            g_sm[m] = g_est;
        }

        // EMA tracking across updates (instant on first valid estimate)
        float alpha = (p_ema_alpha > 0.0f && p_ema_alpha <= 1.0f) ? p_ema_alpha : 0.35f;
        if (n_valid > 0)
        {
            float eff_a = ema_init ? alpha : 1.0f;
            for (uint32_t m = 0; m < NBmode; m++)
            {
                g_ema[m] = (1.0f - eff_a) * g_ema[m] + eff_a * (float) g_sm[m];
            }
            ema_init = 1;
        }

        // Publish per-mode SHM outputs
        img_raw.md[0].write = 1;
        img_err.md[0].write = 1;
        img_smt.md[0].write = 1;
        img_snr.md[0].write = 1;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            img_raw.im->array.F[m] = (float) g_raw[m];
            img_err.im->array.F[m] = (float) g_se[m];
            img_smt.im->array.F[m] = g_ema[m];
            img_snr.im->array.F[m] = (float) g_snr[m];
        }
        processinfo_update_output_stream(processinfo, img_raw.im, NULL);
        processinfo_update_output_stream(processinfo, img_err.im, NULL);
        processinfo_update_output_stream(processinfo, img_smt.im, NULL);
        processinfo_update_output_stream(processinfo, img_snr.im, NULL);

        // Optionally write live per-mode inverse optical gain to aol<N>_mWFSfact
        if (p_apply == 1 && n_valid > 0)
        {
            img_mwfs.md[0].write = 1;
            for (uint32_t m = 0; m < NBmode; m++)
            {
                float g = g_ema[m];
                if (g < g_lo_clmp)
                {
                    g = g_lo_clmp;
                }
                if (g > g_hi_clmp)
                {
                    g = g_hi_clmp;
                }
                img_mwfs.im->array.F[m] = 1.0f / g;
            }
            processinfo_update_output_stream(processinfo, img_mwfs.im, NULL);
            o_applied_cnt++;
        }

        // Summary medians for FPS dashboard
        uint32_t cnt = 0;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            med_buf[cnt++] = (double) g_ema[m];
        }
        o_og_med = mog_median_f64(med_buf, cnt);

        uint32_t lo_n = (NBmode < 20u) ? NBmode : 20u;
        for (uint32_t m = 0; m < lo_n; m++)
        {
            med_buf[m] = (double) g_ema[m];
        }
        o_og_lo_med = mog_median_f64(med_buf, lo_n);

        if (NBmode > 20u)
        {
            uint32_t hi_n = NBmode - 20u;
            for (uint32_t m = 0; m < hi_n; m++)
            {
                med_buf[m] = (double) g_ema[20u + m];
            }
            o_og_hi_med = mog_median_f64(med_buf, hi_n);
        }
        else
        {
            o_og_hi_med = o_og_lo_med;
        }

        cnt = 0;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            if (m_valid[m])
            {
                med_buf[cnt++] = g_se[m];
            }
        }
        o_err_med = mog_median_f64(med_buf, cnt);

        cnt = 0;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            if (m_valid[m])
            {
                med_buf[cnt++] = g_snr[m];
            }
        }
        o_snr_med = mog_median_f64(med_buf, cnt);

        if (processinfo != NULL)
        {
            processinfo_WriteMessage_fmt(
                processinfo, "og=%.3f (lo=%.3f hi=%.3f) err=%.3f snr=%.1f valid=%u apply=%d",
                o_og_med, o_og_lo_med, o_og_hi_med, o_err_med, o_snr_med, n_valid, p_apply);
        }
        }
    }
    MILK_PROCINFO_END(self)

    mog_kern_free(&kern);
    free(g_ema);
    free(wfs_ring);
    free(uapp_ring);
    free(prb_ring);
    free(W_lin);
    free(U_lin);
    free(P_lin);
    free(Y_dm);
    free(Z_prb);
    free(g_raw);
    free(g_se);
    free(g_snr);
    free(g_sm);
    free(m_valid);
    free(med_buf);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_modalog = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "modalog",
                 .cmdkey      = "modalog",
                 .description = "Online single-stage optical-gain estimator via closed-loop probes",
                 .description_long =
                     "Unbiased closed-loop optical-gain estimator subscribing to double-buffered "
                     "WFS, DM, and exogenous probe streams. Uses lab-calibrated 3D dynamic "
                     "impulse kernels H_eff[l] in a differenced Instrumental-Variables estimator "
                     "with radial spatial-frequency smoothing and optional live aolX_mWFSfact "
                     "updates." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(modalog),
    .compute = modalog_compute,
};
EXPORT_FPSEXEC(cacao, "modalog", runnable_AOloopControl_modalog);
