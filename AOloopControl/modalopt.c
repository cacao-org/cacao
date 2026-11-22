// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalopt.c
 * @brief   Online per-mode leaky-integrator (gain, leak) autotuner via Parseval Multi-Res PSD
 *
 * Subscribes asynchronously to double-buffered pseudo-open-loop telemetry produced by
 * cacao-fpsexec-mfilt (aol<N>_modevalOL_buff) and optional closed-loop WFS residuals
 * (aol<N>_modevalWFS_buff), optical gain (aol<N>_og_smooth / aol<N>_mWFSfact), and clipping
 * telemetry (aol<N>_mlimitcntfrac).
 *
 * Algorithm:
 *   1. Parseval-exact unclamped 2-band Multi-Resolution PSD:
 *        - Band 1 (sub-Hz & DC): decimated ring buffer (L_dec=256) + explicit DC line mu^2
 *          with exact sinc^2 boxcar decimation inversion so white noise is flat across splice.
 *        - Band 2 (high-f): Hann-windowed Welch PSD on incoming buffer slices.
 *      Because int(|ETF|^2 - |NTF|^2) df = fs/2 = const for any stable causal controller with
 *      delay >= 1, minimizing J_m(g,a) = sum_k P_OL(f_k) * (|ETF|^2 + lam_dm * |CTF|^2) is
 *      algebraically equivalent to exact time-domain causal replay with zero noise-floor
 *      subtraction and zero max(S-Sn,0) rectification bias on faint stars.
 *   2. Exact discrete plant H(z) (2-tap fractional delay or loaded dynpsol H_diag/H_mimo FIR)
 *      with Argument-Principle Nyquist stability and modulus-margin (max|ETF| <= ms_max) bounds.
 *   3. Decoupled update rate & faint-star adaptive memory:
 *        - Fast wake-up cadence on every mfilt buffer flip (~0.25-0.5 s).
 *        - Cumulative 1/k warmup -> per-mode SNR-adaptive EWMA memory [tau_fast..tau_faint].
 *        - Dual regime-change detector (low-order total OL power AND HF [0.30,0.44]fs noise floor).
 *        - Optional variance-aware James-Stein radial Empirical-Bayes cost pooling along sqrt(m+1).
 *   4. Closed-loop safety guards:
 *        - Lag-d & lag-1 ringing detector on WFS slice: immediate multiplicative gain backoff,
 *          freezing PSD accumulation on ringing slices to prevent corrupting sufficient statistics.
 *        - Optional clipping-fraction anti-windup leak nudge and asymmetric EMA publishing.
 */

// MILK_CMAKE_REQUEST_BLAS
// MILK_CMAKE_REQUEST_CFITSIO

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef HAVE_BLAS
#    include "milk_blas_lapacke.h"
#endif

#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#define MOPT_MAX_TAPS    32
#define MOPT_L_DEC       256
#define MOPT_NSEG_HI     256
#define MOPT_N_G_GRID    18
#define MOPT_N_A_GRID    10
#define MOPT_MAX_CAND    (MOPT_N_G_GRID * MOPT_N_A_GRID)
#define MOPT_DENSE_NF    257

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint64_t p_loopindex                             = 0;
static char     p_inol[FUNCTION_PARAMETER_STRMAXLEN]    = "aol0_modevalOL_buff";
static char     p_inwfs[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_modevalWFS_buff";
static char     p_inog[FUNCTION_PARAMETER_STRMAXLEN]    = "";
static char     p_inmwfs[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_inlim[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static char     p_kernel[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static float    p_fs_hz                                 = 1000.0f;
static float    p_latency_fr                            = 2.5f;
static float    p_og_scalar                             = 1.0f;
static float    p_g_min                                 = 0.005f;
static float    p_g_max                                 = 0.80f;
static float    p_alpha_min                             = 0.90f;
static float    p_alpha_max                             = 0.9995f;
static float    p_ms_max                                = 2.25f;
static float    p_lam_dm                                = 1.0e-4f;
static float    p_tau_fast                              = 3.0f;
static float    p_tau_faint                             = 15.0f;
static float    p_snr_mid                               = 0.8f;
static int32_t  p_eb_enable                             = 1;
static float    p_eb_bw                                 = 1.6f;
static uint32_t p_n_free                                = 8;
static float    p_chg_ratio                             = 2.2f;
static int32_t  p_ring_enable                           = 1;
static float    p_ring_thresh                           = 0.12f;
static float    p_clip_thresh                           = 0.05f;
static float    p_ema_up                                = 0.30f;
static float    p_ema_down                              = 0.60f;
static int32_t  p_reset_stats                           = 0;
static int32_t  p_apply_gain                            = 0;
static int32_t  p_apply_mult                            = 0;
static float    p_loopgain_ref                          = 0.0f;
static float    p_loopmult_ref                          = 0.0f;

/* Output telemetry scalars */
static uint64_t o_n_updates   = 0;
static uint64_t o_applied_cnt = 0;
static uint64_t o_chg_cnt     = 0;
static uint64_t o_ring_cnt    = 0;
static uint32_t o_n_modes     = 0;
static uint32_t o_n_cand      = 0;
static uint32_t o_kern_mode   = 0;
static float    o_g_med       = 0.0f;
static float    o_g_lo_med    = 0.0f;
static float    o_g_hi_med    = 0.0f;
static float    o_a_med       = 0.98f;
static float    o_a_lo_med    = 0.98f;
static float    o_a_hi_med    = 0.98f;
static float    o_snr_med     = 0.0f;
static float    o_tau_med_s   = 0.0f;
static float    o_ol_rms      = 0.0f;
static float    o_pred_rms    = 0.0f;
static float    o_var_red_pct = 0.0f;
static float    o_solve_ms    = 0.0f;

#define FPS_PARAMS_MODALOPT(X)                                                                    \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".inol", p_inol, FPTYPE_STREAMNAME, 1,                                                      \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL double buffer")       \
    X(".inwfs", p_inwfs, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "optional WFS double buffer (ringing guard)")                                               \
    X(".inog", p_inog, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "optional modalog smoothed OG stream")                                                      \
    X(".inmwfs", p_inmwfs, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,         \
      "optional mWFSfact stream (fallback OG=1/f)")                                               \
    X(".inlimfrac", p_inlim, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "optional mlimitcntfrac clipping stream")                                                   \
    X(".kernel", p_kernel, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "optional dynpsol H_diag/H_mimo FITS kernel")                                               \
    X(".fs_hz", &p_fs_hz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "loop frame rate [Hz]")                                                                     \
    X(".latency_fr", &p_latency_fr, FPTYPE_FLOAT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "total loop latency [frames]")   \
    X(".og_scalar", &p_og_scalar, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fallback scalar optical gain")  \
    X(".g_min", &p_g_min, FPTYPE_FLOAT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min modal gain floor")          \
    X(".g_max", &p_g_max, FPTYPE_FLOAT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max raw modal gain clamp")      \
    X(".alpha_min", &p_alpha_min, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min leak multiplier bound")     \
    X(".alpha_max", &p_alpha_max, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max leak multiplier (<1)")      \
    X(".ms_max", &p_ms_max, FPTYPE_FLOAT32, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max |ETF| modulus margin cap")  \
    X(".lam_dm", &p_lam_dm, FPTYPE_FLOAT32, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "DM-effort regularizer")         \
    X(".tau_fast", &p_tau_fast, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "high-SNR memory time [s]")      \
    X(".tau_faint", &p_tau_faint, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "faint low-SNR memory time [s]") \
    X(".snr_mid", &p_snr_mid, FPTYPE_FLOAT32, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "SNR midpoint for memory scale") \
    X(".eb_enable", &p_eb_enable, FPTYPE_ONOFF, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "James-Stein radial EB pooling") \
    X(".eb_bw", &p_eb_bw, FPTYPE_FLOAT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "radial EB bandwidth sqrt(m+1)") \
    X(".n_free", &p_n_free, FPTYPE_UINT32, 0,                                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "leading unpooled modes")        \
    X(".chg_ratio", &p_chg_ratio, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "dual change-det jump ratio")    \
    X(".ring_enable", &p_ring_enable, FPTYPE_ONOFF, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fast WFS lag-d ringing guard")  \
    X(".ring_thresh", &p_ring_thresh, FPTYPE_FLOAT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "ringing -rho(d) threshold")     \
    X(".clip_thresh", &p_clip_thresh, FPTYPE_FLOAT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "clipping frac leak trigger")    \
    X(".ema_up", &p_ema_up, FPTYPE_FLOAT32, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EMA gain ramp-up weight")       \
    X(".ema_down", &p_ema_down, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EMA gain backoff weight")       \
    X(".reset_stats", &p_reset_stats, FPTYPE_ONOFF, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "one-shot reset PSD statistics") \
    X(".apply_gain", &p_apply_gain, FPTYPE_ONOFF, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "write live aolX_mgainfact")     \
    X(".apply_mult", &p_apply_mult, FPTYPE_ONOFF, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "write live aolX_mmultfact")     \
    X(".loopgain_ref", &p_loopgain_ref, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mfilt loopgain (0=auto)")       \
    X(".loopmult_ref", &p_loopmult_ref, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mfilt loopmult (0=auto)")       \
    X(".out.n_updates", &o_n_updates, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "buffer updates processed")                                                                 \
    X(".out.applied_cnt", &o_applied_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "live mgainfact/mmultfact writes")                                                          \
    X(".out.chg_cnt", &o_chg_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "regime-change flush triggers")                                                             \
    X(".out.ring_cnt", &o_ring_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "cumulative mode ringing triggers")                                                         \
    X(".out.n_modes", &o_n_modes, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "number of modes optimized")                                                                \
    X(".out.n_cand", &o_n_cand, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "valid stable (g,alpha) grid points")                                                       \
    X(".out.kern_mode", &o_kern_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "0=2-tap 1=uniform FIR 2=per-mode FIR")                                                     \
    X(".out.g_med", &o_g_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "median raw modal gain (all)")                                                              \
    X(".out.g_lo_med", &o_g_lo_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "median raw modal gain (modes 0..19)")                                                      \
    X(".out.g_hi_med", &o_g_hi_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "median raw modal gain (modes 20+)")                                                        \
    X(".out.a_med", &o_a_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "median modal leak mult (all)")                                                             \
    X(".out.a_lo_med", &o_a_lo_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "median modal leak mult (modes 0..19)")                                                     \
    X(".out.a_hi_med", &o_a_hi_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "median modal leak mult (modes 20+)")                                                       \
    X(".out.snr_med", &o_snr_med, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "median broadband modal SNR")                                                               \
    X(".out.tau_med_s", &o_tau_med_s, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "median adaptive memory time [s]")                                                          \
    X(".out.ol_rms", &o_ol_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "quadrature PSOL RMS [mode units]")                                                         \
    X(".out.pred_rms", &o_pred_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "predicted optimal residual RMS")                                                           \
    X(".out.var_red_pct", &o_var_red_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "predicted variance reduction vs OL [%]")                                                   \
    X(".out.solve_ms", &o_solve_ms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "last update wall-clock compute [ms]")

MILK_PARAMS_DEFINE(cacao_modalopt, FPS_PARAMS_MODALOPT);

/* ================================================================
 * 2.  HELPERS (FFT, MEDIAN, KERNEL LOADER, PLANT STABILITY GRID)
 * ============================================================= */

static void mopt_fft1d_c2c(float *re_im, uint32_t n)
{
    /* In-place radix-2 Cooley-Tukey forward FFT (n power of 2 >= 2) */
    uint32_t j = 0;
    for (uint32_t i = 1; i < n; i++)
    {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
        {
            j ^= bit;
        }
        j ^= bit;
        if (i < j)
        {
            float tr          = re_im[2u * i];
            float ti          = re_im[2u * i + 1u];
            re_im[2u * i]      = re_im[2u * j];
            re_im[2u * i + 1u] = re_im[2u * j + 1u];
            re_im[2u * j]      = tr;
            re_im[2u * j + 1u] = ti;
        }
    }
    for (uint32_t len = 2; len <= n; len <<= 1)
    {
        double ang  = -2.0 * M_PI / (double) len;
        float  wlr  = (float) cos(ang);
        float  wli  = (float) sin(ang);
        uint32_t h  = len >> 1;
        for (uint32_t i = 0; i < n; i += len)
        {
            float wr = 1.0f;
            float wi = 0.0f;
            for (uint32_t k = 0; k < h; k++)
            {
                uint32_t i0 = 2u * (i + k);
                uint32_t i1 = 2u * (i + k + h);
                float    ur = re_im[i0];
                float    ui = re_im[i0 + 1u];
                float    vr = re_im[i1] * wr - re_im[i1 + 1u] * wi;
                float    vi = re_im[i1] * wi + re_im[i1 + 1u] * wr;
                re_im[i0]      = ur + vr;
                re_im[i0 + 1u] = ui + vi;
                re_im[i1]      = ur - vr;
                re_im[i1 + 1u] = ui - vi;
                float nwr      = wr * wlr - wi * wli;
                wi             = wr * wli + wi * wlr;
                wr             = nwr;
            }
        }
    }
}

static int mopt_cmp_f64(const void *a, const void *b)
{
    double da = *(const double *) a;
    double db = *(const double *) b;
    return (da > db) - (da < db);
}

static float mopt_median_f64(double *vals, uint32_t n)
{
    if (n == 0)
    {
        return 0.0f;
    }
    qsort(vals, n, sizeof(double), mopt_cmp_f64);
    if ((n & 1u) != 0u)
    {
        return (float) vals[n / 2u];
    }
    return (float) (0.5 * (vals[n / 2u - 1u] + vals[n / 2u]));
}

static uint32_t mopt_slice_frames(const IMAGE *im)
{
    if (im->md[0].naxis == 3 && im->md[0].size[2] == 2 && im->md[0].size[1] > 1)
    {
        return im->md[0].size[1];
    }
    if (im->md[0].naxis == 3)
    {
        return im->md[0].size[2];
    }
    if (im->md[0].naxis == 2)
    {
        return im->md[0].size[1];
    }
    return 1u;
}

static const float *mopt_slice_ptr(const IMAGE *im, uint32_t nbmode, uint32_t *out_nslice)
{
    if (im->md[0].naxis == 3 && im->md[0].size[2] == 2 && im->md[0].size[1] > 1)
    {
        uint32_t ns = im->md[0].size[1];
        uint32_t sl = (uint32_t) (im->md[0].cnt1 & 1ULL);
        *out_nslice = ns;
        return &im->array.F[(size_t) sl * ns * nbmode];
    }
    *out_nslice = mopt_slice_frames(im);
    return im->array.F;
}

typedef struct
{
    uint32_t mode;  /* 0=2tap, 1=uniform FIR, 2=per-mode diag FIR */
    uint32_t ntaps;
    uint32_t nmode;
    float   *h_diag; /* [ntaps * nmode] (when mode==1, nmode==1) */
} MOPT_KERN;

static void mopt_kern_free(MOPT_KERN *k)
{
    free(k->h_diag);
    memset(k, 0, sizeof(*k));
}

static int mopt_load_kernel(MOPT_KERN *k, uint32_t nbmode, const char *raw_path)
{
    mopt_kern_free(k);
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
        fprintf(stderr, "modalopt: cannot open kernel '%s'\n", path);
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
        if (f_nin == 0 || f_L < 2 || f_L > MOPT_MAX_TAPS)
        {
            free(raw);
            return -1;
        }
        if (f_nin == 1)
        {
            float *hd = (float *) calloc(f_L, sizeof(float));
            if (hd == NULL)
            {
                free(raw);
                return -1;
            }
            for (uint32_t l = 0; l < f_L; l++)
            {
                hd[l] = raw[l];
            }
            free(raw);
            k->mode   = 1;
            k->ntaps  = f_L;
            k->nmode  = 1;
            k->h_diag = hd;
            return 0;
        }
        uint32_t n_dyn = (f_nin < nbmode) ? f_nin : nbmode;
        float   *hd    = (float *) calloc((size_t) f_L * n_dyn, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            for (uint32_t m = 0; m < n_dyn; m++)
            {
                hd[(size_t) l * n_dyn + m] = raw[(size_t) l * f_nin + m];
            }
        }
        free(raw);
        k->mode   = 2;
        k->ntaps  = f_L;
        k->nmode  = n_dyn;
        k->h_diag = hd;
        return 0;
    }

    /* 3D MIMO kernel [N_in, N_in, L]: extract per-mode diagonal impulse h_{m,m}[l] */
    uint32_t f_nin = (uint32_t) naxes[0];
    uint32_t f_ns  = (uint32_t) naxes[1];
    uint32_t f_L   = (uint32_t) naxes[2];
    if (f_nin == 0 || f_nin != f_ns || f_L < 2 || f_L > MOPT_MAX_TAPS)
    {
        free(raw);
        return -1;
    }
    if (f_nin == 1)
    {
        float *hd = (float *) calloc(f_L, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            hd[l] = raw[l];
        }
        free(raw);
        k->mode   = 1;
        k->ntaps  = f_L;
        k->nmode  = 1;
        k->h_diag = hd;
        return 0;
    }

    uint32_t n_dyn = (f_nin < nbmode) ? f_nin : nbmode;
    float   *hd    = (float *) calloc((size_t) f_L * n_dyn, sizeof(float));
    if (hd == NULL)
    {
        free(raw);
        return -1;
    }
    for (uint32_t l = 0; l < f_L; l++)
    {
        for (uint32_t m = 0; m < n_dyn; m++)
        {
            hd[(size_t) l * n_dyn + m] = raw[((size_t) l * f_nin + m) * f_nin + m];
        }
    }
    free(raw);
    k->mode   = 2;
    k->ntaps  = f_L;
    k->nmode  = n_dyn;
    k->h_diag = hd;
    return 0;
#else
    (void) nbmode;
    return -1;
#endif
}

/* Evaluate plant H(e^{iw}) for either a causal FIR vector h[1..L-1] or 2-tap fractional delay */
static void mopt_eval_plant_resp(const float *h_taps,
                                 uint32_t     ntaps,
                                 double       lat_fr,
                                 const double *w_rad,
                                 uint32_t     nf,
                                 double      *hr_out,
                                 double      *hi_out)
{
    if (h_taps != NULL && ntaps >= 2)
    {
        for (uint32_t k = 0; k < nf; k++)
        {
            double w  = w_rad[k];
            double re = 0.0;
            double im = 0.0;
            for (uint32_t l = 1; l < ntaps; l++)
            {
                double hl = (double) h_taps[l];
                if (fabs(hl) < 1.0e-12)
                {
                    continue;
                }
                double ang = w * (double) l;
                re        += hl * cos(ang);
                im        -= hl * sin(ang);
            }
            hr_out[k] = re;
            hi_out[k] = im;
        }
        return;
    }

    double lat = (lat_fr < 1.0) ? 1.0 : ((lat_fr > 24.0) ? 24.0 : lat_fr);
    int    di  = (int) floor(lat);
    double df  = lat - (double) di;
    double w0  = 1.0 - df;
    for (uint32_t k = 0; k < nf; k++)
    {
        double w  = w_rad[k];
        double a0 = w * (double) di;
        double a1 = w * (double) (di + 1);
        hr_out[k] = w0 * cos(a0) + df * cos(a1);
        hi_out[k] = -(w0 * sin(a0) + df * sin(a1));
    }
}

/* Exact Nyquist Argument-Principle stability & modulus-margin check on dense [0..pi] grid */
static int mopt_is_stable_pair(double        g_eff,
                               double        alpha,
                               double        ms_max,
                               const double *hd_r,
                               const double *hd_i,
                               const double *cos_wd,
                               const double *sin_wd)
{
    if (alpha < 0.0 || alpha >= 0.99995 || g_eff < 0.0)
    {
        return 0;
    }
    if (g_eff == 0.0)
    {
        return 1;
    }

    double prev_ang = 0.0;
    double wind     = 0.0;
    for (uint32_t k = 0; k < MOPT_DENSE_NF; k++)
    {
        double dr = 1.0 - alpha * cos_wd[k];
        double di = alpha * sin_wd[k];
        double d2 = dr * dr + di * di;
        if (d2 < 1.0e-18)
        {
            return 0;
        }
        double nr  = g_eff * hd_r[k];
        double ni  = g_eff * hd_i[k];
        /* 1 + L = (D + N) / D */
        double clr = dr + nr;
        double cli = di + ni;
        double cl2 = clr * clr + cli * cli;
        if (cl2 < 1.0e-16)
        {
            return 0;
        }
        double etf_mag = sqrt(d2 / cl2);
        if (etf_mag > ms_max)
        {
            return 0;
        }
        /* Complex ratio 1 + L = (clr + i*cli) / (dr + i*di) */
        double rr  = (clr * dr + cli * di) / d2;
        double ri  = (cli * dr - clr * di) / d2;
        double ang = atan2(ri, rr);
        if (k == 0)
        {
            if (rr <= 0.0)
            {
                return 0;
            }
            prev_ang = ang;
        }
        else
        {
            double da = ang - prev_ang;
            if (da > M_PI)
            {
                da -= 2.0 * M_PI;
            }
            else if (da < -M_PI)
            {
                da += 2.0 * M_PI;
            }
            wind    += da;
            prev_ang = ang;
        }
    }
    /* Zero unstable closed-loop poles iff net upper-half winding of (1 + L) is 0 */
    if (fabs(wind) > 0.35 * M_PI)
    {
        return 0;
    }
    return 1;
}

typedef struct
{
    uint32_t ncand;
    uint32_t nf;
    float    g_eff[MOPT_MAX_CAND];
    float    alpha[MOPT_MAX_CAND];
    float   *etf2; /* [ncand * nf] row-major */
    float   *ctf2; /* [ncand * nf] row-major */
} MOPT_GRID;

static const double MOPT_G_BASE[MOPT_N_G_GRID] = {
    0.0,  0.015, 0.03, 0.05, 0.08, 0.12, 0.16, 0.21, 0.26,
    0.32, 0.38,  0.44, 0.50, 0.56, 0.62, 0.68, 0.74, 0.82
};
static const double MOPT_A_BASE[MOPT_N_A_GRID] = {
    0.75, 0.82, 0.88, 0.92, 0.95, 0.97, 0.985, 0.992, 0.9965, 0.9995
};

static uint32_t mopt_build_grid(MOPT_GRID    *gr,
                                const float  *h_taps,
                                uint32_t      ntaps,
                                double        lat_fr,
                                double        alpha_min,
                                double        alpha_max,
                                double        ms_max,
                                const double *w_comb,
                                uint32_t      nf,
                                const double *w_dense,
                                const double *cos_wd,
                                const double *sin_wd)
{
    double hd_r[MOPT_DENSE_NF];
    double hd_i[MOPT_DENSE_NF];
    mopt_eval_plant_resp(h_taps, ntaps, lat_fr, w_dense, MOPT_DENSE_NF, hd_r, hd_i);

    double *hc_r  = (double *) malloc(sizeof(double) * nf);
    double *hc_i  = (double *) malloc(sizeof(double) * nf);
    double *cos_c = (double *) malloc(sizeof(double) * nf);
    double *sin_c = (double *) malloc(sizeof(double) * nf);
    if (!hc_r || !hc_i || !cos_c || !sin_c)
    {
        free(hc_r);
        free(hc_i);
        free(cos_c);
        free(sin_c);
        gr->ncand = 0;
        return 0;
    }
    mopt_eval_plant_resp(h_taps, ntaps, lat_fr, w_comb, nf, hc_r, hc_i);
    for (uint32_t k = 0; k < nf; k++)
    {
        cos_c[k] = cos(w_comb[k]);
        sin_c[k] = sin(w_comb[k]);
    }

    /* Build clamped unique alpha candidates spanning [alpha_min, alpha_max] */
    double   a_list[MOPT_N_A_GRID + 2];
    uint32_t na   = 0;
    double   amin = (alpha_min < 0.50) ? 0.50 : ((alpha_min > 0.999) ? 0.999 : alpha_min);
    double   amax = (alpha_max < amin) ? amin : ((alpha_max > 0.9999) ? 0.9999 : alpha_max);
    a_list[na++]  = amin;
    for (uint32_t ia = 0; ia < MOPT_N_A_GRID; ia++)
    {
        double av = MOPT_A_BASE[ia];
        if (av > amin + 1.0e-4 && av < amax - 1.0e-4)
        {
            a_list[na++] = av;
        }
    }
    if (amax > amin + 1.0e-4)
    {
        a_list[na++] = amax;
    }

    uint32_t nc = 0;
    for (uint32_t ia = 0; ia < na; ia++)
    {
        double av = a_list[ia];
        for (uint32_t ig = 0; ig < MOPT_N_G_GRID; ig++)
        {
            double gv = MOPT_G_BASE[ig];
            if (nc >= MOPT_MAX_CAND)
            {
                break;
            }
            if (!mopt_is_stable_pair(gv, av, ms_max, hd_r, hd_i, cos_wd, sin_wd))
            {
                continue;
            }
            gr->g_eff[nc] = (float) gv;
            gr->alpha[nc] = (float) av;
            float *erow   = &gr->etf2[(size_t) nc * nf];
            float *crow   = &gr->ctf2[(size_t) nc * nf];
            for (uint32_t k = 0; k < nf; k++)
            {
                double dr  = 1.0 - av * cos_c[k];
                double di  = av * sin_c[k];
                double d2  = dr * dr + di * di;
                double clr = dr + gv * hc_r[k];
                double cli = di + gv * hc_i[k];
                double cl2 = clr * clr + cli * cli;
                erow[k]    = (float) ((cl2 > 1.0e-15) ? (d2 / cl2) : 1.0);
                crow[k]    = (float) ((cl2 > 1.0e-15) ? ((gv * gv) / cl2) : 0.0);
            }
            nc++;
        }
    }

    gr->ncand = nc;
    gr->nf    = nf;
    free(hc_r);
    free(hc_i);
    free(cos_c);
    free(sin_c);
    return nc;
}

/* Auto-detect scalar loopgain/loopmult from mfilt's live ratio mgain/mgainfact */
static float mopt_detect_scalar_ref(const IMAGE *im_eff,
                                    const IMAGE *im_fact,
                                    uint32_t     nbmode,
                                    double      *scratch)
{
    if (im_eff == NULL || im_fact == NULL || im_eff->md[0].cnt0 == 0)
    {
        return 1.0f;
    }
    uint32_t cnt = 0;
    for (uint32_t m = 0; m < nbmode; m++)
    {
        float f = im_fact->array.F[m];
        float e = im_eff->array.F[m];
        if (isfinite(f) && isfinite(e) && f > 1.0e-3f && e > 1.0e-5f)
        {
            double r = (double) e / (double) f;
            if (r > 1.0e-4 && r < 10.0)
            {
                scratch[cnt++] = r;
            }
        }
    }
    if (cnt == 0)
    {
        return 1.0f;
    }
    return mopt_median_f64(scratch, cnt);
}

/* ================================================================
 * 3.  MAIN COMPUTE UNIT
 * ============================================================= */

static MILK_HOT errno_t modalopt_compute(const MILK_RUNNABLE *self)
{
    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

    o_n_updates   = 0;
    o_applied_cnt = 0;
    o_chg_cnt     = 0;
    o_ring_cnt    = 0;

    IMGID imgol = imgid_make_from_fpskey(p_inol, dcfpsname, ".inol");
    resolveIMGID(&imgol, ERRMODE_WARN, dcimg, dcnimg);
    if (imgol.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode    = imgol.md[0].size[0];
    uint32_t slice_len = mopt_slice_frames(imgol.im);
    if (NBmode == 0 || slice_len < 16u)
    {
        return RETURN_FAILURE;
    }
    o_n_modes = NBmode;

    /* Optional companion streams */
    int   has_wfs = 0;
    IMGID imgwfs  = imgid_make_from_fpskey(p_inwfs, dcfpsname, ".inwfs");
    if (p_inwfs[0] != '\0' && strcmp(p_inwfs, "none") != 0)
    {
        resolveIMGID(&imgwfs, ERRMODE_NULL, dcimg, dcnimg);
        if (imgwfs.ID != -1 && imgwfs.md[0].size[0] == NBmode)
        {
            has_wfs = 1;
        }
    }

    int   has_og = 0;
    IMGID imgog  = imgid_make_from_fpskey(p_inog, dcfpsname, ".inog");
    if (p_inog[0] != '\0' && strcmp(p_inog, "none") != 0)
    {
        resolveIMGID(&imgog, ERRMODE_NULL, dcimg, dcnimg);
        if (imgog.ID != -1 && imgog.md[0].size[0] == NBmode)
        {
            has_og = 1;
        }
    }

    int   has_mwfs = 0;
    IMGID imgmwfs  = imgid_make_from_fpskey(p_inmwfs, dcfpsname, ".inmwfs");
    if (p_inmwfs[0] != '\0' && strcmp(p_inmwfs, "none") != 0)
    {
        resolveIMGID(&imgmwfs, ERRMODE_NULL, dcimg, dcnimg);
        if (imgmwfs.ID != -1 && imgmwfs.md[0].size[0] == NBmode)
        {
            has_mwfs = 1;
        }
    }

    int   has_lim = 0;
    IMGID imglim  = imgid_make_from_fpskey(p_inlim, dcfpsname, ".inlimfrac");
    if (p_inlim[0] != '\0' && strcmp(p_inlim, "none") != 0)
    {
        resolveIMGID(&imglim, ERRMODE_NULL, dcimg, dcnimg);
        if (imglim.ID != -1 && imglim.md[0].size[0] == NBmode)
        {
            has_lim = 1;
        }
    }

    /* Optional dynpsol FIR kernel */
    MOPT_KERN kern;
    memset(&kern, 0, sizeof(kern));
    if (p_kernel[0] != '\0' && strcmp(p_kernel, "none") != 0)
    {
        if (mopt_load_kernel(&kern, NBmode, p_kernel) != 0)
        {
            fprintf(stderr, "modalopt: failed loading kernel '%s', using 2-tap delay\n", p_kernel);
            mopt_kern_free(&kern);
        }
    }
    o_kern_mode = kern.mode;

    /* Connect diagnostic output streams + mfilt gain/mult streams */
    char  sname[STRINGMAXLEN_STREAMNAME];
    WRITE_IMAGENAME(sname, "aol%lu_mopt_gain", p_loopindex);
    IMGID img_out_g = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mopt_mult", p_loopindex);
    IMGID img_out_a = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mopt_snr", p_loopindex);
    IMGID img_out_snr = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mopt_taumem", p_loopindex);
    IMGID img_out_tau = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mopt_ring", p_loopindex);
    IMGID img_out_rng = stream_connect_create_2Df32(sname, NBmode, 1);

    WRITE_IMAGENAME(sname, "aol%lu_mgain", p_loopindex);
    IMGID img_mgain = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mgainfact", p_loopindex);
    IMGID img_mgainf = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mmult", p_loopindex);
    IMGID img_mmult = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mmultfact", p_loopindex);
    IMGID img_mmultf = stream_connect_create_2Df32(sname, NBmode, 1);

    /* Configure 2-band Parseval frequency grid geometry */
    double fs_cur = (p_fs_hz > 10.0f) ? (double) p_fs_hz : 1000.0;
    /* Choose power-of-2 decimation so fs_dec ~ 50..125 Hz and divides slice_len cleanly */
    uint32_t decim = 16u;
    if (fs_cur < 200.0)
    {
        decim = 2u;
    }
    else if (fs_cur < 450.0)
    {
        decim = 4u;
    }
    else if (fs_cur < 900.0)
    {
        decim = 8u;
    }
    while (decim > 1u && slice_len < decim * 4u)
    {
        decim >>= 1u;
    }
    uint32_t nseg_hi = MOPT_NSEG_HI;
    while (nseg_hi > slice_len && nseg_hi > 32u)
    {
        nseg_hi >>= 1u;
    }
    uint32_t nf_hi_all = nseg_hi / 2u + 1u;
    double   fs_dec    = fs_cur / (double) decim;
    double   df_lo     = fs_dec / (double) MOPT_L_DEC;
    double   df_hi     = fs_cur / (double) nseg_hi;
    double   f_split   = (3.0 * df_hi > 12.0) ? (3.0 * df_hi) : 12.0;
    if (f_split > 0.35 * fs_dec)
    {
        f_split = 0.35 * fs_dec;
    }
    uint32_t k_lo_end = (uint32_t) ceil(f_split / df_lo);
    if (k_lo_end < 2u)
    {
        k_lo_end = 2u;
    }
    if (k_lo_end > MOPT_L_DEC / 2u - 1u)
    {
        k_lo_end = MOPT_L_DEC / 2u - 1u;
    }
    double   f_lo_cut   = ((double) k_lo_end - 0.5) * df_lo;
    uint32_t k_hi_start = (uint32_t) floor(f_lo_cut / df_hi + 0.5);
    if (k_hi_start < 1u)
    {
        k_hi_start = 1u;
    }
    if (k_hi_start >= nf_hi_all - 2u)
    {
        k_hi_start = nf_hi_all - 2u;
    }
    double splice_w0 = (((double) k_hi_start + 0.5) * df_hi - f_lo_cut) / df_hi;
    if (splice_w0 < 0.15)
    {
        splice_w0 = 0.15;
    }
    else if (splice_w0 > 1.5)
    {
        splice_w0 = 1.5;
    }

    uint32_t n_hi_keep = nf_hi_all - k_hi_start;
    uint32_t nf_comb   = k_lo_end + n_hi_keep; /* bin 0 = DC, 1..k_lo_end-1 = lo, rest = hi */

    double *w_comb       = (double *) malloc(sizeof(double) * nf_comb);
    double *sinc2_inv    = (double *) malloc(sizeof(double) * k_lo_end);
    double *w_dense      = (double *) malloc(sizeof(double) * MOPT_DENSE_NF);
    double *cos_wd       = (double *) malloc(sizeof(double) * MOPT_DENSE_NF);
    double *sin_wd       = (double *) malloc(sizeof(double) * MOPT_DENSE_NF);
    float  *hann_hi      = (float *) malloc(sizeof(float) * nseg_hi);
    float  *hann_dec     = (float *) malloc(sizeof(float) * MOPT_L_DEC);

    MOPT_GRID grid;
    memset(&grid, 0, sizeof(grid));
    grid.etf2 = (float *) calloc((size_t) MOPT_MAX_CAND * nf_comb, sizeof(float));
    grid.ctf2 = (float *) calloc((size_t) MOPT_MAX_CAND * nf_comb, sizeof(float));

    /* Per-mode workspaces & persistent sufficient statistics */
    float *tmp_ol = (float *) calloc((size_t) slice_len * NBmode, sizeof(float));
    float *tmp_wfs =
        has_wfs ? (float *) calloc((size_t) slice_len * NBmode, sizeof(float)) : NULL;
    float  *dec_ring  = (float *) calloc((size_t) MOPT_L_DEC * NBmode, sizeof(float));
    float  *pow_sl    = (float *) calloc((size_t) nf_comb * NBmode, sizeof(float));
    float  *pow_ema   = (float *) calloc((size_t) nf_comb * NBmode, sizeof(float));
    float  *cost_mat  = (float *) calloc((size_t) MOPT_MAX_CAND * NBmode, sizeof(float));
    float  *cost_ctf  = (float *) calloc((size_t) MOPT_MAX_CAND * NBmode, sizeof(float));
    float  *cost_eb   = (float *) calloc((size_t) MOPT_MAX_CAND * NBmode, sizeof(float));
    float  *nvar_sl   = (float *) calloc(NBmode, sizeof(float));
    float  *nvar_ema  = (float *) calloc(NBmode, sizeof(float));
    float  *ol_ms_sl  = (float *) calloc(NBmode, sizeof(float));
    float  *n_eff_m   = (float *) calloc(NBmode, sizeof(float));
    float  *mode_snr  = (float *) calloc(NBmode, sizeof(float));
    float  *tau_mem_m = (float *) calloc(NBmode, sizeof(float));
    float  *g_pub     = (float *) calloc(NBmode, sizeof(float));
    float  *a_pub     = (float *) calloc(NBmode, sizeof(float));
    float  *g_tgt     = (float *) calloc(NBmode, sizeof(float));
    float  *a_tgt     = (float *) calloc(NBmode, sizeof(float));
    float  *og_vec    = (float *) calloc(NBmode, sizeof(float));
    float  *ring_scl  = (float *) calloc(NBmode, sizeof(float));
    uint8_t *ring_flg = (uint8_t *) calloc(NBmode, sizeof(uint8_t));
    uint8_t *ring_cln = (uint8_t *) calloc(NBmode, sizeof(uint8_t));
    double  *med_buf  = (double *) calloc(NBmode, sizeof(double));
    float   *dc_wfs   = (float *) calloc(NBmode, sizeof(float));
    float   *r_coord  = (float *) calloc(NBmode, sizeof(float));

    if (!w_comb || !sinc2_inv || !w_dense || !cos_wd || !sin_wd || !hann_hi || !hann_dec ||
        !grid.etf2 || !grid.ctf2 || !tmp_ol || (has_wfs && !tmp_wfs) || !dec_ring || !pow_sl ||
        !pow_ema || !cost_mat || !cost_ctf || !cost_eb || !nvar_sl || !nvar_ema || !ol_ms_sl ||
        !n_eff_m || !mode_snr || !tau_mem_m || !g_pub || !a_pub || !g_tgt || !a_tgt || !og_vec ||
        !ring_scl || !ring_flg || !ring_cln || !med_buf || !dc_wfs || !r_coord)
    {
        mopt_kern_free(&kern);
        free(w_comb);
        free(sinc2_inv);
        free(w_dense);
        free(cos_wd);
        free(sin_wd);
        free(hann_hi);
        free(hann_dec);
        free(grid.etf2);
        free(grid.ctf2);
        free(tmp_ol);
        free(tmp_wfs);
        free(dec_ring);
        free(pow_sl);
        free(pow_ema);
        free(cost_mat);
        free(cost_ctf);
        free(cost_eb);
        free(nvar_sl);
        free(nvar_ema);
        free(ol_ms_sl);
        free(n_eff_m);
        free(mode_snr);
        free(tau_mem_m);
        free(g_pub);
        free(a_pub);
        free(g_tgt);
        free(a_tgt);
        free(og_vec);
        free(ring_scl);
        free(ring_flg);
        free(ring_cln);
        free(med_buf);
        free(dc_wfs);
        free(r_coord);
        return RETURN_FAILURE;
    }

    /* Precompute frequency grids, Hann windows, and boxcar-decimation sinc^2 inversion */
    w_comb[0]    = 0.0;
    sinc2_inv[0] = 1.0;
    for (uint32_t k = 1; k < k_lo_end; k++)
    {
        double f_k   = (double) k * df_lo;
        w_comb[k]    = 2.0 * M_PI * f_k / fs_cur;
        double x     = M_PI * f_k / fs_dec;
        double s     = (x > 1.0e-8) ? (sin(x) / x) : 1.0;
        sinc2_inv[k] = 1.0 / (s * s);
    }
    for (uint32_t j = 0; j < n_hi_keep; j++)
    {
        double f_k          = (double) (k_hi_start + j) * df_hi;
        w_comb[k_lo_end + j] = 2.0 * M_PI * f_k / fs_cur;
    }
    for (uint32_t k = 0; k < MOPT_DENSE_NF; k++)
    {
        double w  = M_PI * (double) k / (double) (MOPT_DENSE_NF - 1u);
        w_dense[k] = w;
        cos_wd[k]  = cos(w);
        sin_wd[k]  = sin(w);
    }
    double hann_hi_s2 = 0.0;
    for (uint32_t i = 0; i < nseg_hi; i++)
    {
        double w    = 0.5 * (1.0 - cos(2.0 * M_PI * (double) i / (double) nseg_hi));
        hann_hi[i]  = (float) w;
        hann_hi_s2 += w * w;
    }
    double hann_dec_s2 = 0.0;
    for (uint32_t i = 0; i < MOPT_L_DEC; i++)
    {
        double w     = 0.5 * (1.0 - cos(2.0 * M_PI * (double) i / (double) MOPT_L_DEC));
        hann_dec[i]  = (float) w;
        hann_dec_s2 += w * w;
    }

    /* High-frequency white-noise reference band [0.30, 0.44] fs_hz */
    uint32_t k_nf0 = (uint32_t) ceil(0.30 * (double) nseg_hi);
    uint32_t k_nf1 = (uint32_t) floor(0.44 * (double) nseg_hi);
    if (k_nf0 < 1u)
    {
        k_nf0 = 1u;
    }
    if (k_nf1 >= nf_hi_all - 1u)
    {
        k_nf1 = nf_hi_all - 2u;
    }
    if (k_nf1 <= k_nf0)
    {
        k_nf0 = nf_hi_all / 2u;
        k_nf1 = nf_hi_all - 1u;
    }
    double nf_bw_frac = (double) (k_nf1 - k_nf0 + 1u) / (double) (nseg_hi / 2u);

    /* Initialize per-mode state */
    for (uint32_t m = 0; m < NBmode; m++)
    {
        float g0 = img_mgain.im->array.F[m];
        float a0 = img_mmult.im->array.F[m];
        g_pub[m]     = (isfinite(g0) && g0 > 1.0e-4f && g0 < 1.5f) ? g0 : 0.20f;
        a_pub[m]     = (isfinite(a0) && a0 >= 0.5f && a0 < 1.0f) ? a0 : 0.98f;
        ring_scl[m]  = 1.0f;
        r_coord[m]   = sqrtf((float) (m + 1u));
        tau_mem_m[m] = p_tau_fast;
    }

    /* Cache parameters that trigger plant-grid rebuild */
    float    prev_lat   = -1.0f;
    float    prev_amin  = -1.0f;
    float    prev_amax  = -1.0f;
    float    prev_msmax = -1.0f;
    uint32_t dec_head   = 0;
    uint32_t dec_fill   = 0;
    int      ema_inited = 0;
    double   ref_tot    = 0.0;
    double   ref_noise  = 0.0;
    int      ref_inited = 0;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        struct timespec ts0, ts1;
        clock_gettime(CLOCK_MONOTONIC, &ts0);

        /* Copy newly completed slice from double buffer */
        uint32_t     ns_ol  = slice_len;
        const float *src_ol = mopt_slice_ptr(imgol.im, NBmode, &ns_ol);
        if (ns_ol > slice_len)
        {
            ns_ol = slice_len;
        }
        memcpy(tmp_ol, src_ol, sizeof(float) * (size_t) ns_ol * NBmode);

        uint32_t ns_wfs = 0;
        if (has_wfs)
        {
            const float *src_wfs = mopt_slice_ptr(imgwfs.im, NBmode, &ns_wfs);
            if (ns_wfs > slice_len)
            {
                ns_wfs = slice_len;
            }
            memcpy(tmp_wfs, src_wfs, sizeof(float) * (size_t) ns_wfs * NBmode);
        }

        /* One-shot stats reset if requested via FPS */
        if (p_reset_stats != 0)
        {
            ema_inited = 0;
            ref_inited = 0;
            dec_fill   = 0;
            dec_head   = 0;
            memset(n_eff_m, 0, sizeof(float) * NBmode);
            for (uint32_t m = 0; m < NBmode; m++)
            {
                ring_scl[m] = 1.0f;
                ring_cln[m] = 0u;
            }
            p_reset_stats = 0;
        }

        /* Rebuild plant stability/transfer grid when latency/bounds change */
        float cur_lat   = (p_latency_fr >= 1.0f && p_latency_fr <= 24.0f) ? p_latency_fr : 2.5f;
        float cur_amin  = (p_alpha_min >= 0.50f && p_alpha_min <= 0.999f) ? p_alpha_min : 0.90f;
        float cur_amax  = (p_alpha_max >= cur_amin && p_alpha_max <= 0.9999f)
                              ? p_alpha_max
                              : 0.9995f;
        float cur_msmax = (p_ms_max >= 1.30f && p_ms_max <= 6.0f) ? p_ms_max : 2.25f;
        if (cur_lat != prev_lat || cur_amin != prev_amin || cur_amax != prev_amax ||
            cur_msmax != prev_msmax || grid.ncand == 0)
        {
            const float *h_uni = (kern.mode == 1) ? kern.h_diag : NULL;
            uint32_t     nt_u  = (kern.mode == 1) ? kern.ntaps : 0u;
            o_n_cand           = mopt_build_grid(&grid, h_uni, nt_u, (double) cur_lat,
                                                 (double) cur_amin, (double) cur_amax,
                                                 (double) cur_msmax, w_comb, nf_comb,
                                                 w_dense, cos_wd, sin_wd);
            prev_lat           = cur_lat;
            prev_amin          = cur_amin;
            prev_amax          = cur_amax;
            prev_msmax         = cur_msmax;
        }
        if (grid.ncand == 0)
        {
            continue;
        }

        /* 1. Fast closed-loop ringing & DC-droop check on incoming WFS slice */
        uint32_t d_lag = (uint32_t) ceil((double) cur_lat - 1.0e-4);
        if (d_lag < 1u)
        {
            d_lag = 1u;
        }
        uint32_t ring_now_cnt = 0;
        memset(ring_flg, 0, sizeof(uint8_t) * NBmode);
        memset(dc_wfs, 0, sizeof(float) * NBmode);

        if (has_wfs && p_ring_enable != 0 && ns_wfs > d_lag + 16u)
        {
            double r_cfg = (p_ring_thresh > 0.02f) ? (double) p_ring_thresh : 0.12;
            double r_sig = 2.8 / sqrt((double) ns_wfs);
            double r_thr = (r_cfg > r_sig) ? r_cfg : r_sig;
#pragma omp parallel for schedule(static) reduction(+ : ring_now_cnt)
            for (uint32_t m = 0; m < NBmode; m++)
            {
                double mw = 0.0;
                for (uint32_t t = 0; t < ns_wfs; t++)
                {
                    mw += (double) tmp_wfs[(size_t) t * NBmode + m];
                }
                mw /= (double) ns_wfs;
                dc_wfs[m] = (float) fabs(mw);

                double vw = 0.0;
                double c1 = 0.0;
                double cd = 0.0;
                for (uint32_t t = 0; t < ns_wfs; t++)
                {
                    double v0 = (double) tmp_wfs[(size_t) t * NBmode + m] - mw;
                    vw       += v0 * v0;
                    if (t >= 1u)
                    {
                        double v1 = (double) tmp_wfs[(size_t) (t - 1u) * NBmode + m] - mw;
                        c1       += v0 * v1;
                    }
                    if (t >= d_lag)
                    {
                        double vd = (double) tmp_wfs[(size_t) (t - d_lag) * NBmode + m] - mw;
                        cd       += v0 * vd;
                    }
                }
                double rho1 = (vw > 1.0e-24) ? (c1 / vw) : 0.0;
                double rhod = (vw > 1.0e-24) ? (cd / vw) : 0.0;
                if (rhod < -r_thr || (rho1 < -(r_thr + 0.10) && rhod < -0.35 * r_thr))
                {
                    ring_flg[m] = 1u;
                    ring_cln[m] = 0u;
                    ring_scl[m] *= 0.65f;
                    if (ring_scl[m] < 0.20f)
                    {
                        ring_scl[m] = 0.20f;
                    }
                    g_pub[m] *= 0.65f;
                    ring_now_cnt++;
                }
                else if (rhod > -0.02)
                {
                    if (ring_cln[m] < 255u)
                    {
                        ring_cln[m]++;
                    }
                    /* Only relax guard after >= 4 consecutive clean slices */
                    if (ring_cln[m] >= 4u && ring_scl[m] < 1.0f)
                    {
                        ring_scl[m] *= 1.03f;
                        if (ring_scl[m] > 1.0f)
                        {
                            ring_scl[m] = 1.0f;
                        }
                    }
                }
            }
            o_ring_cnt += ring_now_cnt;
        }

        /* 2. Append decimated block averages to low-rate ring buffer */
        uint32_t n_dec_add = ns_ol / decim;
        for (uint32_t b = 0; b < n_dec_add; b++)
        {
            float *dst = &dec_ring[(size_t) dec_head * NBmode];
            memset(dst, 0, sizeof(float) * NBmode);
            for (uint32_t j = 0; j < decim; j++)
            {
                const float *src = &tmp_ol[(size_t) (b * decim + j) * NBmode];
                for (uint32_t m = 0; m < NBmode; m++)
                {
                    dst[m] += src[m];
                }
            }
            float inv_d = 1.0f / (float) decim;
            for (uint32_t m = 0; m < NBmode; m++)
            {
                dst[m] *= inv_d;
            }
            dec_head = (dec_head + 1u) % MOPT_L_DEC;
            if (dec_fill < MOPT_L_DEC)
            {
                dec_fill++;
            }
        }

        /* 3. Compute 2-band Parseval slice power spectrum [nf_comb x NBmode] & HF noise floor */
        uint32_t step_hi = nseg_hi / 2u;
        uint32_t nwin_hi = (ns_ol >= nseg_hi) ? (1u + (ns_ol - nseg_hi) / step_hi) : 1u;
        double   sc_hi   = 1.0 / ((double) nseg_hi * hann_hi_s2 * (double) nwin_hi);

        /* Power-of-2 length for low-f decimated FFT during warmup */
        uint32_t n_dec_fft = 1u;
        while ((n_dec_fft << 1u) <= dec_fill && n_dec_fft < MOPT_L_DEC)
        {
            n_dec_fft <<= 1u;
        }

#pragma omp parallel
        {
            float *fft_hi  = (float *) malloc(sizeof(float) * 2u * nseg_hi);
            float *fft_lo  = (float *) malloc(sizeof(float) * 2u * MOPT_L_DEC);
            double *p_hi_t = (double *) malloc(sizeof(double) * nf_hi_all);

            if (fft_hi && fft_lo && p_hi_t)
            {
#pragma omp for schedule(static)
                for (uint32_t m = 0; m < NBmode; m++)
                {
                    /* Slice mean & total mean-square (DC^2 + AC) */
                    double mu_sl = 0.0;
                    double ms_sl = 0.0;
                    for (uint32_t t = 0; t < ns_ol; t++)
                    {
                        double v = (double) tmp_ol[(size_t) t * NBmode + m];
                        mu_sl   += v;
                        ms_sl   += v * v;
                    }
                    mu_sl /= (double) ns_ol;
                    ol_ms_sl[m] = (float) (ms_sl / (double) ns_ol);

                    /* High-f Hann Welch PSD on slice */
                    memset(p_hi_t, 0, sizeof(double) * nf_hi_all);
                    for (uint32_t w = 0; w < nwin_hi; w++)
                    {
                        uint32_t off = w * step_hi;
                        for (uint32_t i = 0; i < nseg_hi; i++)
                        {
                            uint32_t idx = (off + i < ns_ol) ? (off + i) : (ns_ol - 1u);
                            double   v   = (double) tmp_ol[(size_t) idx * NBmode + m] - mu_sl;
                            fft_hi[2u * i]      = (float) (v * (double) hann_hi[i]);
                            fft_hi[2u * i + 1u] = 0.0f;
                        }
                        mopt_fft1d_c2c(fft_hi, nseg_hi);
                        for (uint32_t k = 0; k < nf_hi_all; k++)
                        {
                            double re  = (double) fft_hi[2u * k];
                            double im  = (double) fft_hi[2u * k + 1u];
                            double pwr = (re * re + im * im) * sc_hi;
                            if (k > 0u && k < nf_hi_all - 1u)
                            {
                                pwr *= 2.0;
                            }
                            p_hi_t[k] += pwr;
                        }
                    }

                    /* Estimate HF [0.30, 0.44] fs noise variance (only for SNR & change det) */
                    double hf_sum = 0.0;
                    for (uint32_t k = k_nf0; k <= k_nf1; k++)
                    {
                        hf_sum += p_hi_t[k];
                    }
                    nvar_sl[m] = (float) (hf_sum / nf_bw_frac);

                    /* Store high-f bins (with exact splice weight on first bin) */
                    for (uint32_t j = 0; j < n_hi_keep; j++)
                    {
                        double pv = p_hi_t[k_hi_start + j];
                        if (j == 0u)
                        {
                            pv *= splice_w0;
                        }
                        pow_sl[(size_t) (k_lo_end + j) * NBmode + m] = (float) pv;
                    }

                    /* Band 1: Low-frequency spectrum from decimated ring buffer */
                    for (uint32_t k = 0; k < k_lo_end; k++)
                    {
                        pow_sl[(size_t) k * NBmode + m] = 0.0f;
                    }
                    if (n_dec_fft >= 16u)
                    {
                        /* Read most recent n_dec_fft samples chronologically */
                        uint32_t start = (dec_head + MOPT_L_DEC - n_dec_fft) % MOPT_L_DEC;
                        double   mu_d  = 0.0;
                        for (uint32_t i = 0; i < n_dec_fft; i++)
                        {
                            uint32_t slot = (start + i) % MOPT_L_DEC;
                            mu_d         += (double) dec_ring[(size_t) slot * NBmode + m];
                        }
                        mu_d /= (double) n_dec_fft;
                        pow_sl[m] = (float) (mu_d * mu_d); /* explicit DC line */

                        double w_s2 = 0.0;
                        for (uint32_t i = 0; i < n_dec_fft; i++)
                        {
                            uint32_t slot = (start + i) % MOPT_L_DEC;
                            double   w    = (n_dec_fft == MOPT_L_DEC)
                                                ? (double) hann_dec[i]
                                                : (0.5 * (1.0 - cos(2.0 * M_PI * (double) i /
                                                                    (double) n_dec_fft)));
                            w_s2 += w * w;
                            double v = (double) dec_ring[(size_t) slot * NBmode + m] - mu_d;
                            fft_lo[2u * i]      = (float) (v * w);
                            fft_lo[2u * i + 1u] = 0.0f;
                        }
                        mopt_fft1d_c2c(fft_lo, n_dec_fft);
                        double   sc_lo  = 2.0 / ((double) n_dec_fft * w_s2);
                        double   df_cur = fs_dec / (double) n_dec_fft;
                        uint32_t khalf  = n_dec_fft / 2u;
                        for (uint32_t kb = 1; kb < khalf; kb++)
                        {
                            double fk = (double) kb * df_cur;
                            if (fk >= f_lo_cut)
                            {
                                break;
                            }
                            double re = (double) fft_lo[2u * kb];
                            double im = (double) fft_lo[2u * kb + 1u];
                            double inv_s2;
                            if (n_dec_fft == MOPT_L_DEC && kb < k_lo_end)
                            {
                                inv_s2 = sinc2_inv[kb];
                            }
                            else
                            {
                                double x  = M_PI * fk / fs_dec;
                                double sx = (x > 1.0e-8) ? (sin(x) / x) : 1.0;
                                inv_s2    = 1.0 / (sx * sx);
                            }
                            double   pwr = (re * re + im * im) * sc_lo * inv_s2;
                            uint32_t kt = (n_dec_fft == MOPT_L_DEC)
                                              ? kb
                                              : (uint32_t) lround(fk / df_lo);
                            if (kt < 1u)
                            {
                                kt = 1u;
                            }
                            if (kt >= k_lo_end)
                            {
                                kt = k_lo_end - 1u;
                            }
                            pow_sl[(size_t) kt * NBmode + m] += (float) pwr;
                        }
                    }
                    else
                    {
                        /* Very first slice: map slice DC + low bins directly */
                        pow_sl[m] = (float) (mu_sl * mu_sl);
                        for (uint32_t kb = 1; kb < k_hi_start; kb++)
                        {
                            double   fk = (double) kb * df_hi;
                            uint32_t kt = (uint32_t) lround(fk / df_lo);
                            if (kt < 1u)
                            {
                                kt = 1u;
                            }
                            if (kt >= k_lo_end)
                            {
                                kt = k_lo_end - 1u;
                            }
                            pow_sl[(size_t) kt * NBmode + m] += (float) p_hi_t[kb];
                        }
                    }
                }
            }
            free(fft_hi);
            free(fft_lo);
            free(p_hi_t);
        }

        /* 4. Dual regime-change detector: BOTH total OL power (DC^2+AC) AND HF noise floor! */
        uint32_t m_ref = (NBmode < 64u) ? NBmode : 64u;
        double   sl_tot = 0.0;
        for (uint32_t m = 0; m < m_ref; m++)
        {
            sl_tot += (double) ol_ms_sl[m];
        }
        sl_tot /= (double) m_ref;

        uint32_t m_n0  = (NBmode > 64u) ? 64u : (NBmode / 2u);
        uint32_t n_cnt = 0;
        for (uint32_t m = m_n0; m < NBmode; m++)
        {
            med_buf[n_cnt++] = (double) nvar_sl[m];
        }
        double sl_noise = (double) mopt_median_f64(med_buf, n_cnt);

        int    reset_now = 0;
        double chg_thr   = (double) p_chg_ratio;
        if (!ref_inited)
        {
            ref_tot    = sl_tot;
            ref_noise  = sl_noise;
            ref_inited = 1;
        }
        else if (chg_thr > 1.05 && ring_now_cnt < NBmode / 4u)
        {
            double rt = (sl_tot > ref_tot) ? (sl_tot / (ref_tot + 1.0e-24))
                                           : (ref_tot / (sl_tot + 1.0e-24));
            double rn = (sl_noise > ref_noise) ? (sl_noise / (ref_noise + 1.0e-24))
                                               : (ref_noise / (sl_noise + 1.0e-24));
            if (rt > chg_thr || rn > chg_thr)
            {
                reset_now = 1;
                o_chg_cnt++;
                /* Flush stale low-f decimated ring, keeping only current slice */
                dec_fill = (n_dec_add < MOPT_L_DEC) ? n_dec_add : MOPT_L_DEC;
            }
            double w_r = reset_now ? 0.50 : 0.15;
            ref_tot    = (1.0 - w_r) * ref_tot + w_r * sl_tot;
            ref_noise  = (1.0 - w_r) * ref_noise + w_r * sl_noise;
        }

        /* 5. Update per-mode PSD sufficient statistics with 1/k cold-start & SNR-adaptive memory */
        double tf_fast  = (p_tau_fast >= 0.25f) ? (double) p_tau_fast : 3.0;
        double tf_faint = (p_tau_faint >= tf_fast) ? (double) p_tau_faint : tf_fast;
        double snr_mid  = (p_snr_mid > 1.0e-3f) ? (double) p_snr_mid : 0.8;

        if (!ema_inited || reset_now)
        {
            memcpy(pow_ema, pow_sl, sizeof(float) * (size_t) nf_comb * NBmode);
            memcpy(nvar_ema, nvar_sl, sizeof(float) * NBmode);
            for (uint32_t m = 0; m < NBmode; m++)
            {
                n_eff_m[m] = (float) ns_ol;
            }
            ema_inited = 1;
        }
        else
        {
#pragma omp parallel for schedule(static)
            for (uint32_t m = 0; m < NBmode; m++)
            {
                /* Freeze PSD update on actively ringing modes so corrupted PSOL is skipped */
                if (ring_flg[m])
                {
                    continue;
                }
                double tot_cur = 0.0;
                for (uint32_t k = 0; k < nf_comb; k++)
                {
                    tot_cur += (double) pow_ema[(size_t) k * NBmode + m];
                }
                double nv_cur = (double) nvar_ema[m];
                double snr_c  = (tot_cur > nv_cur)
                                    ? ((tot_cur - nv_cur) / (nv_cur + 1.0e-24))
                                    : 0.0;
                double tau_m  = tf_fast + (tf_faint - tf_fast) / (1.0 + snr_c / snr_mid);
                tau_mem_m[m]  = (float) tau_m;

                double n_cap = tau_m * fs_cur;
                if (n_cap < (double) ns_ol)
                {
                    n_cap = (double) ns_ol;
                }
                double ne_new = (double) n_eff_m[m] + (double) ns_ol;
                if (ne_new > n_cap)
                {
                    ne_new = n_cap;
                }
                double lam_hi = (double) ns_ol / ne_new;
                double lam_lo = lam_hi * 1.5;
                if (lam_lo > 0.80)
                {
                    lam_lo = 0.80;
                }
                n_eff_m[m]  = (float) ne_new;
                nvar_ema[m] = (float) ((1.0 - lam_hi) * nv_cur + lam_hi * (double) nvar_sl[m]);
                for (uint32_t k = 0; k < k_lo_end; k++)
                {
                    size_t idx   = (size_t) k * NBmode + m;
                    pow_ema[idx] = (float) ((1.0 - lam_lo) * (double) pow_ema[idx] +
                                            lam_lo * (double) pow_sl[idx]);
                }
                for (uint32_t k = k_lo_end; k < nf_comb; k++)
                {
                    size_t idx   = (size_t) k * NBmode + m;
                    pow_ema[idx] = (float) ((1.0 - lam_hi) * (double) pow_ema[idx] +
                                            lam_hi * (double) pow_sl[idx]);
                }
            }
        }

        /* 6. Parseval-exact unclamped 2D (g_eff, alpha) cost matrix via single BLAS SGEMM */
        uint32_t nc     = grid.ncand;
        float    lam_dm = (p_lam_dm >= 0.0f) ? p_lam_dm : 0.0f;

#ifdef HAVE_BLAS
        cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) nc, (int) NBmode,
                    (int) nf_comb, 1.0f, grid.etf2, (int) nf_comb, pow_ema, (int) NBmode, 0.0f,
                    cost_mat, (int) NBmode);
        if (lam_dm > 0.0f)
        {
            cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) nc, (int) NBmode,
                        (int) nf_comb, lam_dm, grid.ctf2, (int) nf_comb, pow_ema, (int) NBmode,
                        1.0f, cost_mat, (int) NBmode);
        }
#else
#    pragma omp parallel for schedule(static)
        for (uint32_t c = 0; c < nc; c++)
        {
            const float *erow = &grid.etf2[(size_t) c * nf_comb];
            const float *crow = &grid.ctf2[(size_t) c * nf_comb];
            float       *out  = &cost_mat[(size_t) c * NBmode];
            memset(out, 0, sizeof(float) * NBmode);
            for (uint32_t k = 0; k < nf_comb; k++)
            {
                float        wt = erow[k] + lam_dm * crow[k];
                const float *pk = &pow_ema[(size_t) k * NBmode];
                for (uint32_t m = 0; m < NBmode; m++)
                {
                    out[m] += wt * pk[m];
                }
            }
        }
#endif

        /* If per-mode dynpsol FIR kernel is loaded (mode==2), refine cost_mat on m < kern.nmode */
        if (kern.mode == 2 && kern.nmode > 0)
        {
            uint32_t n_dyn = kern.nmode;
#pragma omp parallel
            {
                float  *ht    = (float *) malloc(sizeof(float) * kern.ntaps);
                double *hcr   = (double *) malloc(sizeof(double) * nf_comb);
                double *hci   = (double *) malloc(sizeof(double) * nf_comb);
                double *hdr   = (double *) malloc(sizeof(double) * MOPT_DENSE_NF);
                double *hdi   = (double *) malloc(sizeof(double) * MOPT_DENSE_NF);
                double *cos_c = (double *) malloc(sizeof(double) * nf_comb);
                double *sin_c = (double *) malloc(sizeof(double) * nf_comb);
                if (ht && hcr && hci && hdr && hdi && cos_c && sin_c)
                {
                    for (uint32_t k = 0; k < nf_comb; k++)
                    {
                        cos_c[k] = cos(w_comb[k]);
                        sin_c[k] = sin(w_comb[k]);
                    }
#pragma omp for schedule(static)
                    for (uint32_t m = 0; m < n_dyn; m++)
                    {
                        for (uint32_t l = 0; l < kern.ntaps; l++)
                        {
                            ht[l] = kern.h_diag[(size_t) l * n_dyn + m];
                        }
                        mopt_eval_plant_resp(ht, kern.ntaps, cur_lat, w_dense, MOPT_DENSE_NF, hdr,
                                             hdi);
                        mopt_eval_plant_resp(ht, kern.ntaps, cur_lat, w_comb, nf_comb, hcr, hci);
                        for (uint32_t c = 0; c < nc; c++)
                        {
                            double gv = (double) grid.g_eff[c];
                            double av = (double) grid.alpha[c];
                            if (!mopt_is_stable_pair(gv, av, (double) cur_msmax, hdr, hdi, cos_wd,
                                                     sin_wd))
                            {
                                cost_mat[(size_t) c * NBmode + m] = 1.0e30f;
                                continue;
                            }
                            double acc = 0.0;
                            for (uint32_t k = 0; k < nf_comb; k++)
                            {
                                double dr  = 1.0 - av * cos_c[k];
                                double di  = av * sin_c[k];
                                double d2  = dr * dr + di * di;
                                double clr = dr + gv * hcr[k];
                                double cli = di + gv * hci[k];
                                double cl2 = clr * clr + cli * cli;
                                double wt  = (cl2 > 1.0e-15)
                                                 ? ((d2 + (double) lam_dm * gv * gv) / cl2)
                                                 : 1.0;
                                acc += wt * (double) pow_ema[(size_t) k * NBmode + m];
                            }
                            cost_mat[(size_t) c * NBmode + m] = (float) acc;
                        }
                    }
                }
                free(ht);
                free(hcr);
                free(hci);
                free(hdr);
                free(hdi);
                free(cos_c);
                free(sin_c);
            }
        }

        /* Normalize cost matrix by open-loop mean-square & compute modal SNR */
        double sum_ol_v = 0.0;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            double tot_m = 0.0;
            for (uint32_t k = 0; k < nf_comb; k++)
            {
                tot_m += (double) pow_ema[(size_t) k * NBmode + m];
            }
            sum_ol_v += tot_m;
            double nv = (double) nvar_ema[m];
            double sn = (tot_m > nv) ? ((tot_m - nv) / (nv + 1.0e-24)) : 0.0;
            mode_snr[m] = (float) sn;
            double inv_ms = (tot_m > 1.0e-24) ? (1.0 / tot_m) : 0.0;
            for (uint32_t c = 0; c < nc; c++)
            {
                cost_eb[(size_t) c * NBmode + m] =
                    (float) ((double) cost_mat[(size_t) c * NBmode + m] * inv_ms);
            }
        }

        /* 7. Optional variance-aware James-Stein radial EB pooling (cost_eb -> cost_ctf) */
        const float *cost_use = cost_eb;
        uint32_t     n_free   = (p_n_free < NBmode) ? p_n_free : NBmode;
        if (p_eb_enable != 0 && p_eb_bw > 1.0e-3f && NBmode > n_free + 4u)
        {
            memcpy(cost_ctf, cost_eb, sizeof(float) * (size_t) nc * NBmode);
            double inv2bw2 = 0.5 / ((double) p_eb_bw * (double) p_eb_bw);

#pragma omp parallel for schedule(static)
            for (uint32_t m = n_free; m < NBmode; m++)
            {
                /* Fast gate: high-SNR modes need zero radial pooling */
                double sn = (double) mode_snr[m];
                double sn_gate = 1.0 / (1.0 + (sn / 0.8) * (sn / 0.8));
                if (sn_gate < 0.03)
                {
                    continue;
                }
                double rm = (double) r_coord[m];
                /* Restrict radial neighborhood to +-3.5*bw in sqrt(m+1) for O(M*K) speed */
                double r_lo = rm - 3.5 * (double) p_eb_bw;
                double r_hi = rm + 3.5 * (double) p_eb_bw;
                uint32_t j0 = (r_lo > 1.0) ? (uint32_t) (r_lo * r_lo) : n_free;
                if (j0 < n_free)
                {
                    j0 = n_free;
                }
                uint32_t j1 = (uint32_t) ceil(r_hi * r_hi) + 1u;
                if (j1 > NBmode)
                {
                    j1 = NBmode;
                }

                double w_sum_full = 0.0;
                double w_sum_loo  = 0.0;
                double y_pr[MOPT_MAX_CAND];
                double y_lo[MOPT_MAX_CAND];
                memset(y_pr, 0, sizeof(double) * nc);
                memset(y_lo, 0, sizeof(double) * nc);

                for (uint32_t j = j0; j < j1; j++)
                {
                    double dr = (double) r_coord[j] - rm;
                    double wt = exp(-dr * dr * inv2bw2);
                    w_sum_full += wt;
                    if (j != m)
                    {
                        w_sum_loo += wt;
                    }
                    for (uint32_t c = 0; c < nc; c++)
                    {
                        double v = (double) cost_eb[(size_t) c * NBmode + j];
                        y_pr[c] += wt * v;
                        if (j != m)
                        {
                            y_lo[c] += wt * v;
                        }
                    }
                }
                if (w_sum_full <= 1.0e-12 || w_sum_loo <= 1.0e-12)
                {
                    continue;
                }
                double diff2 = 0.0;
                for (uint32_t c = 0; c < nc; c++)
                {
                    y_pr[c] /= w_sum_full;
                    y_lo[c] /= w_sum_loo;
                    double d = (double) cost_eb[(size_t) c * NBmode + m] - y_lo[c];
                    diff2   += d * d;
                }
                diff2 /= (double) nc;
                double ne     = (n_eff_m[m] > 64.0f) ? (double) n_eff_m[m] : 64.0;
                double v_samp = (2.5 / ne) * (1.0 / (1.0 + 2.0 * sn));
                double w_js   = (diff2 > v_samp) ? (v_samp / diff2) : 1.0;
                w_js         *= sn_gate;
                if (w_js > 0.92)
                {
                    w_js = 0.92;
                }
                for (uint32_t c = 0; c < nc; c++)
                {
                    double ym = (double) cost_eb[(size_t) c * NBmode + m];
                    cost_ctf[(size_t) c * NBmode + m] =
                        (float) ((1.0 - w_js) * ym + w_js * y_pr[c]);
                }
            }
            cost_use = cost_ctf;
        }

        /* 8. Read per-mode optical gain & pick optimal (g_tgt, a_tgt) */
        float og_fb = (p_og_scalar > 0.10f && p_og_scalar < 5.0f) ? p_og_scalar : 1.0f;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            float og = og_fb;
            if (has_og)
            {
                float v = imgog.im->array.F[m];
                if (isfinite(v) && v > 0.15f && v < 3.0f)
                {
                    og = v;
                }
            }
            else if (has_mwfs)
            {
                float f = imgmwfs.im->array.F[m];
                if (isfinite(f) && f > 0.33f && f < 6.67f)
                {
                    og = 1.0f / f;
                }
            }
            og_vec[m] = og;
        }

        float  g_min_c  = (p_g_min >= 0.0f) ? p_g_min : 0.0f;
        float  g_max_c  = (p_g_max > g_min_c) ? p_g_max : (g_min_c + 0.5f);
        double sum_opt_v = 0.0;

        for (uint32_t m = 0; m < NBmode; m++)
        {
            uint32_t best_c = 0;
            float    best_v = cost_use[m];
            for (uint32_t c = 1; c < nc; c++)
            {
                float v = cost_use[(size_t) c * NBmode + m];
                if (v < best_v)
                {
                    best_v = v;
                    best_c = c;
                }
            }
            float raw_cost = cost_mat[(size_t) best_c * NBmode + m];
            if (isfinite(raw_cost) && raw_cost > 0.0f && raw_cost < 1.0e20f)
            {
                sum_opt_v += (double) raw_cost;
            }
            float g_raw = (grid.g_eff[best_c] / og_vec[m]) * ring_scl[m];
            if (g_raw < g_min_c)
            {
                g_raw = g_min_c;
            }
            if (g_raw > g_max_c)
            {
                g_raw = g_max_c;
            }
            float a_raw = grid.alpha[best_c];

            /* Anti-windup clipping guard: nudge leak stronger when mode clips heavily */
            if (has_lim && p_clip_thresh > 0.0f)
            {
                float cf = imglim.im->array.F[m];
                if (isfinite(cf) && cf > p_clip_thresh)
                {
                    float a_nudge = a_raw - 0.02f;
                    a_raw         = (a_nudge > cur_amin) ? a_nudge : cur_amin;
                }
            }

            g_tgt[m] = g_raw;
            a_tgt[m] = a_raw;
        }

        /* 9. Asymmetric SNR-adaptive EMA smoothing on published (g_pub, a_pub) */
        float b_up_base = (p_ema_up > 0.01f && p_ema_up <= 1.0f) ? p_ema_up : 0.30f;
        float b_dn_base = (p_ema_down > 0.01f && p_ema_down <= 1.0f) ? p_ema_down : 0.60f;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            float sn = mode_snr[m];
            float bu;
            float bd;
            float ba;
            if (reset_now || o_n_updates < 2u)
            {
                bu = 0.80f;
                bd = 0.90f;
                ba = 0.85f;
            }
            else
            {
                float sn_f = sn / (sn + 1.5f);
                bu         = b_up_base * (0.50f + 1.00f * sn_f);
                bd         = b_dn_base * (0.75f + 0.50f * sn_f);
                if (bu > 0.80f)
                {
                    bu = 0.80f;
                }
                if (bd > 0.90f)
                {
                    bd = 0.90f;
                }
                /* Fast upward alpha jump if a mode shows strong DC droop on WFS */
                ba = (a_tgt[m] > a_pub[m] && has_wfs &&
                      dc_wfs[m] * dc_wfs[m] > 0.15f * ol_ms_sl[m])
                         ? 0.80f
                         : bu;
            }
            float bg = (g_tgt[m] < g_pub[m]) ? bd : bu;
            if (!ring_flg[m])
            {
                g_pub[m] = (1.0f - bg) * g_pub[m] + bg * g_tgt[m];
            }
            a_pub[m] = (1.0f - ba) * a_pub[m] + ba * a_tgt[m];
            if (a_pub[m] < cur_amin)
            {
                a_pub[m] = cur_amin;
            }
            if (a_pub[m] > cur_amax)
            {
                a_pub[m] = cur_amax;
            }
        }

        /* 10. Publish shadow diagnostic SHM streams */
        img_out_g.md[0].write   = 1;
        img_out_a.md[0].write   = 1;
        img_out_snr.md[0].write = 1;
        img_out_tau.md[0].write = 1;
        img_out_rng.md[0].write = 1;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            img_out_g.im->array.F[m]   = g_pub[m];
            img_out_a.im->array.F[m]   = a_pub[m];
            img_out_snr.im->array.F[m] = mode_snr[m];
            img_out_tau.im->array.F[m] = tau_mem_m[m];
            img_out_rng.im->array.F[m] = ring_scl[m];
        }
        processinfo_update_output_stream(processinfo, img_out_g.im, NULL);
        processinfo_update_output_stream(processinfo, img_out_a.im, NULL);
        processinfo_update_output_stream(processinfo, img_out_snr.im, NULL);
        processinfo_update_output_stream(processinfo, img_out_tau.im, NULL);
        processinfo_update_output_stream(processinfo, img_out_rng.im, NULL);

        /* 11. Optionally apply live to mfilt's aol<N>_mgainfact / aol<N>_mmultfact */
        int did_apply = 0;
        if (p_apply_gain != 0)
        {
            float lg = (p_loopgain_ref > 1.0e-5f)
                           ? p_loopgain_ref
                           : mopt_detect_scalar_ref(img_mgain.im, img_mgainf.im, NBmode, med_buf);
            if (lg < 1.0e-5f)
            {
                lg = 1.0f;
            }
            img_mgainf.md[0].write = 1;
            for (uint32_t m = 0; m < NBmode; m++)
            {
                img_mgainf.im->array.F[m] = g_pub[m] / lg;
            }
            processinfo_update_output_stream(processinfo, img_mgainf.im, NULL);
            did_apply = 1;
        }
        if (p_apply_mult != 0)
        {
            float lm = (p_loopmult_ref > 1.0e-5f)
                           ? p_loopmult_ref
                           : mopt_detect_scalar_ref(img_mmult.im, img_mmultf.im, NBmode, med_buf);
            if (lm < 1.0e-5f)
            {
                lm = 1.0f;
            }
            float max_f = 0.9999f / lm;
            img_mmultf.md[0].write = 1;
            for (uint32_t m = 0; m < NBmode; m++)
            {
                float fv = a_pub[m] / lm;
                if (fv > max_f)
                {
                    fv = max_f;
                }
                if (fv < 0.0f)
                {
                    fv = 0.0f;
                }
                img_mmultf.im->array.F[m] = fv;
            }
            processinfo_update_output_stream(processinfo, img_mmultf.im, NULL);
            did_apply = 1;
        }
        if (did_apply)
        {
            o_applied_cnt++;
        }

        /* 12. Update FPS dashboard scalars */
        o_n_updates++;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            med_buf[m] = (double) g_pub[m];
        }
        o_g_med = mopt_median_f64(med_buf, NBmode);
        uint32_t lo_n = (NBmode < 20u) ? NBmode : 20u;
        for (uint32_t m = 0; m < lo_n; m++)
        {
            med_buf[m] = (double) g_pub[m];
        }
        o_g_lo_med = mopt_median_f64(med_buf, lo_n);
        if (NBmode > 20u)
        {
            for (uint32_t m = 20u; m < NBmode; m++)
            {
                med_buf[m - 20u] = (double) g_pub[m];
            }
            o_g_hi_med = mopt_median_f64(med_buf, NBmode - 20u);
        }
        else
        {
            o_g_hi_med = o_g_lo_med;
        }

        for (uint32_t m = 0; m < NBmode; m++)
        {
            med_buf[m] = (double) a_pub[m];
        }
        o_a_med = mopt_median_f64(med_buf, NBmode);
        for (uint32_t m = 0; m < lo_n; m++)
        {
            med_buf[m] = (double) a_pub[m];
        }
        o_a_lo_med = mopt_median_f64(med_buf, lo_n);
        if (NBmode > 20u)
        {
            for (uint32_t m = 20u; m < NBmode; m++)
            {
                med_buf[m - 20u] = (double) a_pub[m];
            }
            o_a_hi_med = mopt_median_f64(med_buf, NBmode - 20u);
        }
        else
        {
            o_a_hi_med = o_a_lo_med;
        }

        for (uint32_t m = 0; m < NBmode; m++)
        {
            med_buf[m] = (double) mode_snr[m];
        }
        o_snr_med = mopt_median_f64(med_buf, NBmode);
        for (uint32_t m = 0; m < NBmode; m++)
        {
            med_buf[m] = (double) tau_mem_m[m];
        }
        o_tau_med_s   = mopt_median_f64(med_buf, NBmode);
        o_ol_rms      = (float) sqrt(sum_ol_v);
        o_pred_rms    = (float) sqrt(sum_opt_v);
        o_var_red_pct = (sum_ol_v > 1.0e-24)
                            ? (float) (100.0 * (sum_ol_v - sum_opt_v) / sum_ol_v)
                            : 0.0f;

        clock_gettime(CLOCK_MONOTONIC, &ts1);
        o_solve_ms = (float) (((double) (ts1.tv_sec - ts0.tv_sec) * 1.0e3) +
                              ((double) (ts1.tv_nsec - ts0.tv_nsec) * 1.0e-6));

        if (dcfpsptr != NULL)
        {
            fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
        }
        if (processinfo != NULL)
        {
            processinfo_WriteMessage_fmt(
                processinfo, "g=%.3f(%.2f/%.2f) a=%.4f(%.3f/%.3f) tau=%.1fs red=%.1f%% %.1fms",
                o_g_med, o_g_lo_med, o_g_hi_med, o_a_med, o_a_lo_med, o_a_hi_med, o_tau_med_s,
                o_var_red_pct, o_solve_ms);
        }
    }
    MILK_PROCINFO_END(self)

    mopt_kern_free(&kern);
    free(w_comb);
    free(sinc2_inv);
    free(w_dense);
    free(cos_wd);
    free(sin_wd);
    free(hann_hi);
    free(hann_dec);
    free(grid.etf2);
    free(grid.ctf2);
    free(tmp_ol);
    free(tmp_wfs);
    free(dec_ring);
    free(pow_sl);
    free(pow_ema);
    free(cost_mat);
    free(cost_ctf);
    free(cost_eb);
    free(nvar_sl);
    free(nvar_ema);
    free(ol_ms_sl);
    free(n_eff_m);
    free(mode_snr);
    free(tau_mem_m);
    free(g_pub);
    free(a_pub);
    free(g_tgt);
    free(a_tgt);
    free(og_vec);
    free(ring_scl);
    free(ring_flg);
    free(ring_cln);
    free(med_buf);
    free(dc_wfs);
    free(r_coord);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_modalopt = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "modalopt",
                 .cmdkey      = "modalopt",
                 .description = "Online per-mode leaky-integrator gain and leak autotuner",
                 .description_long =
                     "Asynchronously optimizes per-mode leaky-integrator gains and leaks from "
                     "mfilt PSOL double buffers using Parseval-exact unclamped 2-band "
                     "multi-resolution PSDs, exact discrete plant stability margins, SNR-adaptive "
                     "faint-star memory, dual change detection, James-Stein radial EB pooling, "
                     "and closed-loop lag-d ringing protection." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacao_modalopt),
    .compute = modalopt_compute,
};
EXPORT_FPSEXEC(cacao, "modalopt", runnable_AOloopControl_modalopt);
