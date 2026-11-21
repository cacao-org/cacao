// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    factor_dyn_resp.c
 * @brief   Factor raw 3D dynamic response matrix into static RM M_cal and WFS-rate kernel H_eff[l]
 *
 * Consumes a raw exposure-integrated 3D step response resp3D.fits [N_out, N_in, N_t] produced by
 * cacaoac-measdynresp, factors out the static spatial response M_cal via regularized SVD
 * pseudoinverse, and extracts the normalized WFS-frame-rate causal impulse kernel H_eff[l]:
 *
 *   H_eff[0]   = S_wfs[0]
 *   H_eff[l]   = S_wfs[l] - S_wfs[l - 1]   (l = 1 .. L - 2)
 *   H_eff[L-1] = I - S_wfs[L - 2]          (enforces exact unity DC gain: sum_l H_eff[l] = I)
 *
 * Outputs (FITS float32 with timing/diagnostic keywords):
 *   - outmimo   : [N_in (poked), N_in (sensed), L]  MIMO/block-MIMO WFS-rate impulse kernel
 *   - outdiag   : [N_in, L]                         per-mode normalized diagonal impulse kernel
 *   - outstatic : [N_out, N_in]                     static spatial response matrix M_cal
 *   - outstep   : [N_in, N_in, N_t_post]            sub-frame normalized modal step response
 */

// MILK_CMAKE_MANDATE_LAPACKE
// MILK_CMAKE_REQUEST_BLAS

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef _OPENMP
#    include <omp.h>
#endif

#include "milk_blas_lapacke.h"
#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl_acquireCalib.h"

#define FDR_MAX_TAPS 32

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     p_resp3d[FUNCTION_PARAMETER_STRMAXLEN]    = "resp3D.fits";
static char     p_staticrm[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_outmimo[FUNCTION_PARAMETER_STRMAXLEN]   = "H_mimo.fits";
static char     p_outdiag[FUNCTION_PARAMETER_STRMAXLEN]   = "H_diag.fits";
static char     p_outstatic[FUNCTION_PARAMETER_STRMAXLEN] = "";
static char     p_outstep[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static float    p_svdlim                                  = 1.0e-4f;
static uint32_t p_ntaps                                   = 12;
static float    p_phase_fr                                = -1.0f;
static uint32_t p_tail_fr                                 = 2;
static uint32_t p_blocksize                               = 0;
static float    p_offdiag_thr                             = 0.0f;

static uint32_t o_n_in               = 0;
static uint32_t o_n_out              = 0;
static uint32_t o_n_taps             = 0;
static uint32_t o_nbsubfr            = 0;
static uint32_t o_n_repaired         = 0;
static float    o_dt_us              = 0.0f;
static float    o_wfsfreq_hz         = 0.0f;
static float    o_lat_centroid_fr    = 0.0f;
static float    o_peak_overshoot_pct = 0.0f;
static float    o_peak_offdiag_pct   = 0.0f;
static float    o_tail_err_pct       = 0.0f;
static float    o_dc_err_max         = 0.0f;
static float    o_psol_2tap_pct      = 0.0f;
static float    o_psol_diag_pct      = 0.0f;
static float    o_psol_mimo_pct      = 0.0f;

#define FPS_PARAMS_FACTORDYNRM(X)                                                                 \
    X(".resp3d", p_resp3d, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "input raw 3D response cube [N_out, N_in, N_t]")                                            \
    X(".outmimo", p_outmimo, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "output 3D WFS-rate MIMO kernel [N_in, N_in, L]")                                           \
    X(".outdiag", p_outdiag, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "optional 2D diagonal kernel [N_in, L]")                                                    \
    X(".staticrm", p_staticrm, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "optional 2D static RM [N_out, N_in] (empty = auto)")                                       \
    X(".outstatic", p_outstatic, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "optional output 2D static RM M_cal [N_out, N_in]")                                         \
    X(".outstep", p_outstep, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "optional output sub-frame modal step cube")                                                \
    X(".svdlim", &p_svdlim, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "relative SVD truncation threshold for M_cal+")                                             \
    X(".n_taps", &p_ntaps, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "WFS-rate impulse kernel length L (2..32)")                                                 \
    X(".phase_fr", &p_phase_fr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "compute/publish delay offset [WFS fr, <0 = auto 0.5/NBsubfr]")                             \
    X(".tail_fr", &p_tail_fr, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "settled tail frames averaged when staticrm absent")                                        \
    X(".blocksize", &p_blocksize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "optional block-diagonal truncation B (0 = full)")                                          \
    X(".offdiag_thr", &p_offdiag_thr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "optional off-diagonal magnitude prune threshold")                                          \
    X(".out.n_in", &o_n_in, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT, "number of input modes")     \
    X(".out.n_out", &o_n_out, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT, "number of WFS outputs")   \
    X(".out.n_taps", &o_n_taps, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT, "effective WFS taps L")  \
    X(".out.nbsubfr", &o_nbsubfr, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT, "sub-frames/frame")    \
    X(".out.n_repaired", &o_n_repaired, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "dropped (t, mode) bins temporally interpolated")                                           \
    X(".out.dt_us", &o_dt_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT, "sub-frame bin dt [us]")  \
    X(".out.wfsfreq_hz", &o_wfsfreq_hz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "WFS frame rate [Hz]")                                                                      \
    X(".out.lat_centroid_fr", &o_lat_centroid_fr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,       \
      "mean impulse centroid latency [WFS frames]")                                               \
    X(".out.peak_overshoot_pct", &o_peak_overshoot_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT, \
      "mean peak diagonal step overshoot [%]")                                                    \
    X(".out.peak_offdiag_pct", &o_peak_offdiag_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,     \
      "peak transient off-diagonal coupling [%]")                                                 \
    X(".out.tail_err_pct", &o_tail_err_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "pre-closure tail distance ||S[L-2]-I||_F [%]")                                             \
    X(".out.dc_err_max", &o_dc_err_max, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "max |sum_l H_eff[l] - I|")                                                                 \
    X(".out.psol_2tap_pct", &o_psol_2tap_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "benchmark PSOL RMS: 2-tap scalar delay [%]")                                               \
    X(".out.psol_diag_pct", &o_psol_diag_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "benchmark PSOL RMS: diagonal L-tap FIR [%]")                                               \
    X(".out.psol_mimo_pct", &o_psol_mimo_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "benchmark PSOL RMS: MIMO L-tap FIR [%]")

MILK_PARAMS_DEFINE(cacaoac_factordynrm, FPS_PARAMS_FACTORDYNRM);

/* ================================================================
 * 2.  FITS I/O HELPERS
 * ============================================================= */

typedef struct
{
    double dt_sec;
    double wfsfreq;
    long   q0_bin;
    long   nbsubfr;
    long   nbrespfr;
    int    has_dt;
    int    has_wfsfreq;
    int    has_q0;
    int    has_nsub;
    int    has_nresp;
} FDR_HDR;

static void fdr_mkdir_parent(const char *path)
{
    char buf[1024];
    snprintf(buf, sizeof(buf), "%s", path);
    char *slash = strrchr(buf, '/');
    if (slash == NULL || slash == buf)
    {
        return;
    }
    *slash = '\0';
    for (char *p = buf + 1; *p != '\0'; p++)
    {
        if (*p == '/')
        {
            *p = '\0';
            (void) mkdir(buf, 0775);
            *p = '/';
        }
    }
    (void) mkdir(buf, 0775);
}

static int fdr_load_fits_f64(const char *raw_path,
                             double    **out_data,
                             int        *out_naxis,
                             long        out_naxes[3],
                             FDR_HDR    *hdr_opt)
{
    if (raw_path == NULL || raw_path[0] == '\0')
    {
        return -1;
    }
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;

#ifdef USE_CFITSIO
    fitsfile *fptr   = NULL;
    int       status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        return -1;
    }
    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || naxis < 1 || naxis > 3)
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
    if (hdr_opt != NULL)
    {
        memset(hdr_opt, 0, sizeof(*hdr_opt));
        int kst = 0;
        if (fits_read_key(fptr, TDOUBLE, "DT_SEC", &hdr_opt->dt_sec, NULL, &kst) == 0)
        {
            hdr_opt->has_dt = 1;
        }
        kst = 0;
        if (fits_read_key(fptr, TDOUBLE, "WFSFREQ", &hdr_opt->wfsfreq, NULL, &kst) == 0)
        {
            hdr_opt->has_wfsfreq = 1;
        }
        kst = 0;
        if (fits_read_key(fptr, TLONG, "Q0_BIN", &hdr_opt->q0_bin, NULL, &kst) == 0)
        {
            hdr_opt->has_q0 = 1;
        }
        kst = 0;
        if (fits_read_key(fptr, TLONG, "NBSUBFR", &hdr_opt->nbsubfr, NULL, &kst) == 0)
        {
            hdr_opt->has_nsub = 1;
        }
        kst = 0;
        if (fits_read_key(fptr, TLONG, "NBRESPFR", &hdr_opt->nbrespfr, NULL, &kst) == 0)
        {
            hdr_opt->has_nresp = 1;
        }
    }

    LONGLONG nelem = 1;
    for (int i = 0; i < naxis; i++)
    {
        out_naxes[i] = naxes[i];
        nelem       *= (LONGLONG) naxes[i];
    }
    for (int i = naxis; i < 3; i++)
    {
        out_naxes[i] = 1;
    }
    *out_naxis = naxis;

    double *buf = (double *) malloc(sizeof(double) * (size_t) nelem);
    if (buf == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TDOUBLE, 1, nelem, NULL, buf, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(buf);
        return -1;
    }
    *out_data = buf;
    return 0;
#else
    (void) out_data;
    (void) out_naxis;
    (void) out_naxes;
    (void) hdr_opt;
    fprintf(stderr, "factordynrm: built without CFITSIO support\n");
    return -1;
#endif
}

static int fdr_write_fits_f32(const char  *raw_path,
                              const float *data,
                              int          naxis,
                              const long   naxes[3],
                              int          hmode,
                              long         n_mode,
                              long         n_taps,
                              double       phase_fr,
                              double       lat_cent,
                              double       wfsfreq,
                              double       dt_sec,
                              long         nbsubfr,
                              double       oversht,
                              double       offdiag,
                              double       tailerr,
                              double       p2tap,
                              double       pdiag,
                              double       pmimo)
{
    if (raw_path == NULL || raw_path[0] == '\0')
    {
        return 0;
    }
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    fdr_mkdir_parent(path);

#ifdef USE_CFITSIO
    char bang[1040];
    snprintf(bang, sizeof(bang), "!%s", path);
    fitsfile *fptr   = NULL;
    int       status = 0;
    if (fits_create_file(&fptr, bang, &status) != 0)
    {
        return -1;
    }
    long ax[3] = { naxes[0], naxes[1], naxes[2] };
    if (fits_create_img(fptr, FLOAT_IMG, naxis, ax, &status) != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    LONGLONG nelem = 1;
    for (int i = 0; i < naxis; i++)
    {
        nelem *= (LONGLONG) ax[i];
    }
    fits_write_img(fptr, TFLOAT, 1, nelem, (void *) data, &status);

    long hm = (long) hmode;
    fits_update_key(fptr, TLONG, "HMODE", &hm, "0=MIMO 1=DIAG 2=STATIC 3=STEP", &status);
    fits_update_key(fptr, TLONG, "NMODE", &n_mode, "Input/modal dimension N_in", &status);
    fits_update_key(fptr, TLONG, "NTAPS", &n_taps, "WFS-rate causal impulse taps L", &status);
    fits_update_key(fptr, TDOUBLE, "PHASEFR", &phase_fr, "Sampling phase offset [WFS fr]", &status);
    fits_update_key(fptr, TDOUBLE, "LATCENT", &lat_cent, "Mean impulse centroid [WFS fr]", &status);
    fits_update_key(fptr, TDOUBLE, "WFSFREQ", &wfsfreq, "WFS frame rate [Hz]", &status);
    fits_update_key(fptr, TDOUBLE, "DT_SEC", &dt_sec, "Sub-frame bin width [s]", &status);
    fits_update_key(fptr, TLONG, "NBSUBFR", &nbsubfr, "Sub-frame bins per WFS frame", &status);
    fits_update_key(fptr, TDOUBLE, "OVERSHT", &oversht, "Mean peak step overshoot [%]", &status);
    fits_update_key(fptr, TDOUBLE, "OFFDIAG", &offdiag, "Peak transient off-diag [%]", &status);
    fits_update_key(fptr, TDOUBLE, "TAILERR", &tailerr, "Pre-closure tail err [%]", &status);
    fits_update_key(fptr, TDOUBLE, "P2TAP", &p2tap, "Bench PSOL 2-tap RMS [%]", &status);
    fits_update_key(fptr, TDOUBLE, "PDIAG", &pdiag, "Bench PSOL diag FIR RMS [%]", &status);
    fits_update_key(fptr, TDOUBLE, "PMIMO", &pmimo, "Bench PSOL MIMO FIR RMS [%]", &status);

    fits_close_file(fptr, &status);
    return (status == 0) ? 0 : -1;
#else
    (void) data;
    (void) naxis;
    (void) naxes;
    (void) hmode;
    (void) n_mode;
    (void) n_taps;
    (void) phase_fr;
    (void) lat_cent;
    (void) wfsfreq;
    (void) dt_sec;
    (void) nbsubfr;
    (void) oversht;
    (void) offdiag;
    (void) tailerr;
    (void) p2tap;
    (void) pdiag;
    (void) pmimo;
    return -1;
#endif
}

/* ================================================================
 * 3.  DETERMINISTIC RNG & KOLMOGOROV PSOL BENCHMARK
 * ============================================================= */

static double fdr_randn(uint64_t *st)
{
    double u1 = 0.0;
    double u2 = 0.0;
    for (int i = 0; i < 2; i++)
    {
        uint64_t x = *st;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        *st = x;
        double u = (double) ((x * 0x2545F4914F6CDD1DULL) >> 11) * (1.0 / 9007199254740992.0);
        if (i == 0)
        {
            u1 = (u < 1.0e-15) ? 1.0e-15 : u;
        }
        else
        {
            u2 = u;
        }
    }
    return sqrt(-2.0 * log(u1)) * cos(6.28318530717958647692 * u2);
}

/**
 * @brief Score 2-tap scalar delay, diagonal FIR, and MIMO FIR on a synthetic 1 kHz-like
 *        Kolmogorov AR(1) command stream against the unpruned full-length WFS-rate truth H_truth.
 */
static void fdr_bench_psol(const double *H_truth,
                           uint32_t      L_truth,
                           const double *H_mimo,
                           const double *H_diag,
                           uint32_t      L_kern,
                           uint32_t      n_in,
                           float        *out_2tap,
                           float        *out_diag,
                           float        *out_mimo)
{
    uint32_t n_sim  = 1600;
    uint32_t t_eval = (L_truth > L_kern ? L_truth : L_kern) + 4;
    if (t_eval + 128 >= n_sim || n_in == 0)
    {
        *out_2tap = 0.0f;
        *out_diag = 0.0f;
        *out_mimo = 0.0f;
        return;
    }

    double *u_cmd = (double *) calloc((size_t) n_sim * n_in, sizeof(double));
    double *y_tru = (double *) calloc((size_t) n_sim * n_in, sizeof(double));
    double *y_mim = (double *) calloc((size_t) n_sim * n_in, sizeof(double));
    double *y_dia = (double *) calloc((size_t) n_sim * n_in, sizeof(double));
    if (!u_cmd || !y_tru || !y_mim || !y_dia)
    {
        free(u_cmd);
        free(y_tru);
        free(y_mim);
        free(y_dia);
        return;
    }

    uint64_t rng = 0x51A7C0DE2026ULL;
    for (uint32_t m = 0; m < n_in; m++)
    {
        double rad = sqrt((double) (m + 1));
        double rho = 0.985 - 0.020 * rad;
        if (rho < 0.80)
        {
            rho = 0.80;
        }
        double amp = pow(rad + 1.0, -1.1) * sqrt(1.0 - rho * rho);
        for (uint32_t t = 1; t < n_sim; t++)
        {
            u_cmd[(size_t) t * n_in + m] =
                rho * u_cmd[(size_t) (t - 1) * n_in + m] + amp * fdr_randn(&rng);
        }
    }

    // Convolve truth, MIMO, and diag kernels: y[t] += u[t - l] * H[l]^T
    for (uint32_t l = 0; l < L_truth; l++)
    {
        uint32_t nrows = n_sim - l;
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) nrows, (int) n_in, (int) n_in,
                    1.0, u_cmd, (int) n_in, &H_truth[(size_t) l * n_in * n_in], (int) n_in, 1.0,
                    &y_tru[(size_t) l * n_in], (int) n_in);
    }
    for (uint32_t l = 0; l < L_kern; l++)
    {
        uint32_t nrows = n_sim - l;
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) nrows, (int) n_in, (int) n_in,
                    1.0, u_cmd, (int) n_in, &H_mimo[(size_t) l * n_in * n_in], (int) n_in, 1.0,
                    &y_mim[(size_t) l * n_in], (int) n_in);
        const double *hd = &H_diag[(size_t) l * n_in];
        for (uint32_t t = l; t < n_sim; t++)
        {
            const double *u_r = &u_cmd[(size_t) (t - l) * n_in];
            double       *y_r = &y_dia[(size_t) t * n_in];
            for (uint32_t m = 0; m < n_in; m++)
            {
                y_r[m] += hd[m] * u_r[m];
            }
        }
    }

    double s_tru = 0.0;
    double s_dia = 0.0;
    double s_mim = 0.0;
    for (uint32_t t = t_eval; t < n_sim; t++)
    {
        for (uint32_t m = 0; m < n_in; m++)
        {
            size_t idx = (size_t) t * n_in + m;
            double yt  = y_tru[idx];
            double ed  = yt - y_dia[idx];
            double em  = yt - y_mim[idx];
            s_tru     += yt * yt;
            s_dia     += ed * ed;
            s_mim     += em * em;
        }
    }

    // Best-fit 2-tap scalar latency observer (CACAO baseline)
    double best_2tap = 1.0e30;
    for (int step = 0; step <= 160; step++)
    {
        double   lat_c = 0.0 + 0.025 * (double) step;
        uint32_t li    = (uint32_t) lat_c;
        double   lf    = lat_c - (double) li;
        if (li + 1 >= t_eval)
        {
            break;
        }
        double s_err = 0.0;
        for (uint32_t t = t_eval; t < n_sim; t++)
        {
            const double *u0 = &u_cmd[(size_t) (t - li) * n_in];
            const double *u1 = &u_cmd[(size_t) (t - li - 1) * n_in];
            const double *yt = &y_tru[(size_t) t * n_in];
            for (uint32_t m = 0; m < n_in; m++)
            {
                double est = (1.0 - lf) * u0[m] + lf * u1[m];
                double e   = yt[m] - est;
                s_err     += e * e;
            }
        }
        if (s_err < best_2tap)
        {
            best_2tap = s_err;
        }
    }

    double denom = (s_tru > 1.0e-30) ? s_tru : 1.0e-30;
    *out_2tap    = (float) (100.0 * sqrt(best_2tap / denom));
    *out_diag    = (float) (100.0 * sqrt(s_dia / denom));
    *out_mimo    = (float) (100.0 * sqrt(s_mim / denom));

    free(u_cmd);
    free(y_tru);
    free(y_mim);
    free(y_dia);
}

/* ================================================================
 * 4.  MAIN COMPUTE UNIT
 * ============================================================= */

/**
 * @brief Build sibling FITS path by replacing trailing ".fits" with @p suffix (e.g. "_cnt.fits").
 */
static int fdr_sibling_fits(const char *raw_path, const char *suffix, char *out, size_t out_sz)
{
    const char *rp = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    size_t      rl = strlen(rp);
    size_t      sl = strlen(suffix);
    if (rl <= 5 || strcmp(rp + rl - 5, ".fits") != 0 || rl - 5 + sl + 1 > out_sz)
    {
        return -1;
    }
    memcpy(out, rp, rl - 5);
    memcpy(out + rl - 5, suffix, sl + 1);
    return 0;
}

/**
 * @brief Repair dropped (t, mode) bins (cnt == 0 or late exact zero) by temporal interpolation.
 */
static uint32_t fdr_repair_dropped_bins(double     *r3_raw,
                                        uint32_t    n_out,
                                        uint32_t    n_in,
                                        uint32_t    n_t,
                                        uint32_t    q0_bin,
                                        uint32_t    nbsubfr,
                                        const char *resp3d_path)
{
    uint8_t *valid = (uint8_t *) malloc((size_t) n_t * n_in);
    if (valid == NULL)
    {
        return 0;
    }
    memset(valid, 1, (size_t) n_t * n_in);

    // 1. Prefer authoritative <resp3d>_cnt.fits when present
    char    cnt_path[1024];
    double *cnt_data = NULL;
    int     na_c     = 0;
    long    ax_c[3]  = { 1, 1, 1 };
    int     have_cnt = 0;
    if (fdr_sibling_fits(resp3d_path, "_cnt.fits", cnt_path, sizeof(cnt_path)) == 0 &&
        fdr_load_fits_f64(cnt_path, &cnt_data, &na_c, ax_c, NULL) == 0)
    {
        if (na_c == 2 && (uint32_t) ax_c[0] == n_in && (uint32_t) ax_c[1] == n_t)
        {
            have_cnt = 1;
            for (uint32_t t = 0; t < n_t; t++)
            {
                for (uint32_t m = 0; m < n_in; m++)
                {
                    if (cnt_data[(size_t) t * n_in + m] < 0.5)
                    {
                        valid[(size_t) t * n_in + m] = 0;
                    }
                }
            }
        }
        free(cnt_data);
    }

    // 2. Fallback heuristic if _cnt.fits absent: detect exact-zero post-rise slices
    if (!have_cnt)
    {
        uint32_t t_chk0 = q0_bin + 3 * nbsubfr;
        for (uint32_t t = t_chk0; t < n_t; t++)
        {
            for (uint32_t m = 0; m < n_in; m++)
            {
                const double *v   = &r3_raw[((size_t) t * n_in + m) * n_out];
                double        nrm = 0.0;
                for (uint32_t p = 0; p < n_out; p++)
                {
                    nrm += v[p] * v[p];
                }
                if (nrm <= 1.0e-30)
                {
                    valid[(size_t) t * n_in + m] = 0;
                }
            }
        }
    }

    uint32_t n_repaired = 0;
    for (uint32_t m = 0; m < n_in; m++)
    {
        for (uint32_t t = 0; t < n_t; t++)
        {
            if (valid[(size_t) t * n_in + m])
            {
                continue;
            }
            int tp = (int) t - 1;
            while (tp >= 0 && !valid[(size_t) tp * n_in + m])
            {
                tp--;
            }
            int tn = (int) t + 1;
            while (tn < (int) n_t && !valid[(size_t) tn * n_in + m])
            {
                tn++;
            }
            double *dst = &r3_raw[((size_t) t * n_in + m) * n_out];
            if (tp >= 0 && tn < (int) n_t)
            {
                double        a  = (double) ((int) t - tp) / (double) (tn - tp);
                const double *vp = &r3_raw[((size_t) tp * n_in + m) * n_out];
                const double *vn = &r3_raw[((size_t) tn * n_in + m) * n_out];
                for (uint32_t p = 0; p < n_out; p++)
                {
                    dst[p] = (1.0 - a) * vp[p] + a * vn[p];
                }
                n_repaired++;
            }
            else if (tp >= 0)
            {
                const double *vp = &r3_raw[((size_t) tp * n_in + m) * n_out];
                memcpy(dst, vp, sizeof(double) * n_out);
                n_repaired++;
            }
            else if (tn < (int) n_t)
            {
                const double *vn = &r3_raw[((size_t) tn * n_in + m) * n_out];
                memcpy(dst, vn, sizeof(double) * n_out);
                n_repaired++;
            }
        }
    }

    free(valid);
    return n_repaired;
}

static errno_t factordynrm_compute(const MILK_RUNNABLE *self)
{
    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

    MILK_PROCINFO_INIT(self);

    double *r3_raw  = NULL;
    double *m_stat  = NULL;
    double *m_copy  = NULL;
    double *svd_s   = NULL;
    double *svd_u   = NULL;
    double *svd_vt  = NULL;
    double *c_pinv  = NULL;
    double *s_sub   = NULL;
    double *S_wfs   = NULL;
    double *H_truth = NULL;
    double *H_mimo  = NULL;
    double *H_diag  = NULL;
    errno_t ret     = RETURN_FAILURE;

    MILK_PROCINFO_LOOPSTART(self);
    {
        int     naxis_r3    = 0;
        long    naxes_r3[3] = { 1, 1, 1 };
        FDR_HDR hdr;
        memset(&hdr, 0, sizeof(hdr));

        if (fdr_load_fits_f64(p_resp3d, &r3_raw, &naxis_r3, naxes_r3, &hdr) != 0 || naxis_r3 != 3)
        {
            fprintf(stderr, "factordynrm: failed to read 3D resp3d '%s'\n", p_resp3d);
            goto cleanup;
        }

        uint32_t n_out   = (uint32_t) naxes_r3[0];
        uint32_t n_in    = (uint32_t) naxes_r3[1];
        uint32_t n_t     = (uint32_t) naxes_r3[2];
        uint32_t nbsubfr = (hdr.has_nsub && hdr.nbsubfr >= 1) ? (uint32_t) hdr.nbsubfr : 1;
        uint32_t q0_bin  = (hdr.has_q0 && hdr.q0_bin >= 0 && hdr.q0_bin < (long) n_t)
                               ? (uint32_t) hdr.q0_bin
                               : 0;
        double   dt_sec  = (hdr.has_dt && hdr.dt_sec > 0.0) ? hdr.dt_sec : 1.0e-3;
        double   wfsfreq = (hdr.has_wfsfreq && hdr.wfsfreq > 0.0)
                               ? hdr.wfsfreq
                               : (1.0 / (dt_sec * (double) nbsubfr));

        // Repair any dropped (t, mode) bins before static/dynamic factoring
        uint32_t n_repaired =
            fdr_repair_dropped_bins(r3_raw, n_out, n_in, n_t, q0_bin, nbsubfr, p_resp3d);

        // Load static RM: explicit .staticrm, auto <resp3d>_static.fits, or settled tail
        int have_stat = 0;
        if (p_staticrm[0] != '\0' && strcmp(p_staticrm, "none") != 0)
        {
            int  na_s    = 0;
            long ax_s[3] = { 1, 1, 1 };
            if (fdr_load_fits_f64(p_staticrm, &m_stat, &na_s, ax_s, NULL) != 0 || na_s != 2 ||
                (uint32_t) ax_s[0] != n_out || (uint32_t) ax_s[1] != n_in)
            {
                fprintf(stderr, "factordynrm: invalid staticrm '%s' (expected %u x %u)\n",
                        p_staticrm, n_out, n_in);
                goto cleanup;
            }
            have_stat = 1;
        }
        if (!have_stat)
        {
            char auto_stat[1024];
            int  na_s    = 0;
            long ax_s[3] = { 1, 1, 1 };
            if (fdr_sibling_fits(p_resp3d, "_static.fits", auto_stat, sizeof(auto_stat)) == 0 &&
                fdr_load_fits_f64(auto_stat, &m_stat, &na_s, ax_s, NULL) == 0)
            {
                if (na_s == 2 && (uint32_t) ax_s[0] == n_out && (uint32_t) ax_s[1] == n_in)
                {
                    have_stat = 1;
                }
                else
                {
                    free(m_stat);
                    m_stat = NULL;
                }
            }
        }
        if (!have_stat)
        {
            uint32_t tail_fr   = (p_tail_fr >= 1) ? p_tail_fr : 2;
            uint32_t tail_bins = tail_fr * nbsubfr;
            if (tail_bins >= n_t - q0_bin)
            {
                tail_bins = (n_t > q0_bin + 1) ? ((n_t - q0_bin) / 4 + 1) : 1;
            }
            uint32_t t_tail0 = n_t - tail_bins;
            m_stat           = (double *) calloc((size_t) n_in * n_out, sizeof(double));
            if (m_stat == NULL)
            {
                goto cleanup;
            }
            for (uint32_t t = t_tail0; t < n_t; t++)
            {
                const double *sl = &r3_raw[(size_t) t * n_in * n_out];
                for (size_t idx = 0; idx < (size_t) n_in * n_out; idx++)
                {
                    m_stat[idx] += sl[idx];
                }
            }
            double inv_tb = 1.0 / (double) tail_bins;
            for (size_t idx = 0; idx < (size_t) n_in * n_out; idx++)
            {
                m_stat[idx] *= inv_tb;
            }
        }

        // Regularized SVD pseudoinverse C_pinv [n_in x n_out] of M_stat [n_in x n_out]
        uint32_t k_svd = (n_in < n_out) ? n_in : n_out;
        m_copy         = (double *) malloc(sizeof(double) * (size_t) n_in * n_out);
        svd_s          = (double *) calloc(k_svd, sizeof(double));
        svd_u          = (double *) malloc(sizeof(double) * (size_t) n_in * k_svd);
        svd_vt         = (double *) malloc(sizeof(double) * (size_t) k_svd * n_out);
        c_pinv         = (double *) calloc((size_t) n_in * n_out, sizeof(double));
        s_sub          = (double *) calloc((size_t) n_t * n_in * n_in, sizeof(double));

        if (!m_copy || !svd_s || !svd_u || !svd_vt || !c_pinv || !s_sub)
        {
            goto cleanup;
        }

        memcpy(m_copy, m_stat, sizeof(double) * (size_t) n_in * n_out);
        lapack_int info = LAPACKE_dgesdd(LAPACK_ROW_MAJOR, 'S', (lapack_int) n_in,
                                         (lapack_int) n_out, m_copy, (lapack_int) n_out, svd_s,
                                         svd_u, (lapack_int) k_svd, svd_vt, (lapack_int) n_out);
        if (info != 0 || svd_s[0] <= 1.0e-30)
        {
            fprintf(stderr, "factordynrm: SVD failed (info=%d, s0=%.3e)\n", (int) info, svd_s[0]);
            goto cleanup;
        }

        double thr = ((p_svdlim > 0.0f) ? (double) p_svdlim : 1.0e-4) * svd_s[0];
        for (uint32_t r = 0; r < k_svd; r++)
        {
            double scale = (svd_s[r] > thr) ? (1.0 / svd_s[r]) : 0.0;
            for (uint32_t i = 0; i < n_in; i++)
            {
                svd_u[(size_t) i * k_svd + r] *= scale;
            }
        }
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) n_in, (int) n_out, (int) k_svd,
                    1.0, svd_u, (int) k_svd, svd_vt, (int) n_out, 0.0, c_pinv, (int) n_out);

        // Project every time slice t: S_sub[t] = C_pinv * R_raw[t]^T  [n_in x n_in]
        for (uint32_t t = 0; t < n_t; t++)
        {
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) n_in, (int) n_in,
                        (int) n_out, 1.0, c_pinv, (int) n_out, &r3_raw[(size_t) t * n_in * n_out],
                        (int) n_out, 0.0, &s_sub[(size_t) t * n_in * n_in], (int) n_in);
        }

        // Sub-frame diagnostics (peak overshoot & peak transient off-diagonal coupling)
        double sum_ov = 0.0;
        for (uint32_t m = 0; m < n_in; m++)
        {
            double pk = 0.0;
            for (uint32_t t = q0_bin; t < n_t; t++)
            {
                double v = s_sub[(size_t) t * n_in * n_in + (size_t) m * n_in + m];
                if (v > pk)
                {
                    pk = v;
                }
            }
            sum_ov += (pk - 1.0) * 100.0;
        }
        double mean_ov = sum_ov / (double) n_in;

        double   diag_dc = 0.0;
        uint32_t n_dc    = (2 * nbsubfr < n_t) ? (2 * nbsubfr) : 1;
        for (uint32_t t = n_t - n_dc; t < n_t; t++)
        {
            double dn2 = 0.0;
            for (uint32_t m = 0; m < n_in; m++)
            {
                double v = s_sub[(size_t) t * n_in * n_in + (size_t) m * n_in + m];
                dn2     += v * v;
            }
            diag_dc += sqrt(dn2);
        }
        diag_dc /= (double) n_dc;

        double peak_off = 0.0;
        for (uint32_t t = q0_bin; t < n_t; t++)
        {
            double        dn2 = 0.0;
            double        on2 = 0.0;
            const double *st  = &s_sub[(size_t) t * n_in * n_in];
            for (uint32_t i = 0; i < n_in; i++)
            {
                for (uint32_t j = 0; j < n_in; j++)
                {
                    double v = st[(size_t) i * n_in + j];
                    if (i == j)
                    {
                        dn2 += v * v;
                    }
                    else
                    {
                        on2 += v * v;
                    }
                }
            }
            double dn = sqrt(dn2);
            if (dn >= 0.25 * diag_dc && dn > 1.0e-12)
            {
                double frac = 100.0 * sqrt(on2) / dn;
                if (frac > peak_off)
                {
                    peak_off = frac;
                }
            }
        }

        // Sample WFS-rate step response at elapsed time tau_l = (l - phase_fr) * T_wfs
        // measdynresp bin q has center tau(q) = (q - q0_bin + 0.5) * dt_bin
        // => continuous bin index fbin(l) = q0_bin + (l - phase_fr) * nbsubfr - 0.5
        // When phase_fr < 0 (auto), phase_fr = 0.5 / nbsubfr selects exact bin q0 + l*nbsubfr - 1
        // (matching measdynresp phase_idx=0 immediate post-WFS RTC publish).
        // For l = 0, tau_0 <= 0 => strictly causal S_wfs[0] = 0 (H_eff[0] = 0).
        double phase_fr =
            (p_phase_fr < 0.0f) ? (0.5 / (double) nbsubfr) : (double) p_phase_fr;
        if (phase_fr >= 1.0)
        {
            phase_fr = 0.9999;
        }

        uint32_t avail_fr = 1;
        while ((double) q0_bin + ((double) avail_fr - phase_fr) * (double) nbsubfr - 0.5 <
               (double) (n_t - 1))
        {
            avail_fr++;
        }
        if (avail_fr < 3)
        {
            avail_fr = 3;
        }
        uint32_t L_truth = avail_fr;
        uint32_t L_kern  = (p_ntaps >= 2) ? p_ntaps : 12;
        if (L_kern > FDR_MAX_TAPS)
        {
            L_kern = FDR_MAX_TAPS;
        }
        if (L_kern > L_truth)
        {
            L_kern = L_truth;
        }

        size_t nn = (size_t) n_in * n_in;
        S_wfs     = (double *) calloc((size_t) L_truth * nn, sizeof(double));
        H_truth   = (double *) calloc((size_t) L_truth * nn, sizeof(double));
        H_mimo    = (double *) calloc((size_t) L_kern * nn, sizeof(double));
        H_diag    = (double *) calloc((size_t) L_kern * n_in, sizeof(double));
        if (!S_wfs || !H_truth || !H_mimo || !H_diag)
        {
            goto cleanup;
        }

        // S_wfs[0] remains 0 (strict causality); sample l = 1 .. L_truth - 1
        for (uint32_t l = 1; l < L_truth; l++)
        {
            double fbin = (double) q0_bin + ((double) l - phase_fr) * (double) nbsubfr - 0.5;
            if (fbin <= (double) q0_bin - 0.5)
            {
                continue;
            }
            if (fbin < 0.0)
            {
                fbin = 0.0;
            }
            if (fbin > (double) (n_t - 1))
            {
                fbin = (double) (n_t - 1);
            }
            uint32_t b0 = (uint32_t) floor(fbin);
            if (b0 >= n_t - 1)
            {
                b0 = n_t - 2;
            }
            double        frac = fbin - (double) b0;
            const double *s0   = &s_sub[(size_t) b0 * nn];
            const double *s1   = &s_sub[(size_t) (b0 + 1) * nn];
            double       *dst  = &S_wfs[(size_t) l * nn];
            for (size_t idx = 0; idx < nn; idx++)
            {
                dst[idx] = (1.0 - frac) * s0[idx] + frac * s1[idx];
            }
        }

        // Pre-closure tail error at S_wfs[L_kern - 2]
        double        tail_sq = 0.0;
        const double *s_pen   = &S_wfs[(size_t) (L_kern - 2) * nn];
        for (uint32_t i = 0; i < n_in; i++)
        {
            for (uint32_t j = 0; j < n_in; j++)
            {
                double target = (i == j) ? 1.0 : 0.0;
                double d      = s_pen[(size_t) i * n_in + j] - target;
                tail_sq      += d * d;
            }
        }
        double tail_err_pct = 100.0 * sqrt(tail_sq / (double) n_in);

        // Full-length truth impulse kernel H_truth (exact DC closure at L_truth - 1)
        memcpy(H_truth, S_wfs, sizeof(double) * nn);
        for (uint32_t l = 1; l + 1 < L_truth; l++)
        {
            for (size_t idx = 0; idx < nn; idx++)
            {
                H_truth[(size_t) l * nn + idx] =
                    S_wfs[(size_t) l * nn + idx] - S_wfs[(size_t) (l - 1) * nn + idx];
            }
        }
        for (uint32_t i = 0; i < n_in; i++)
        {
            for (uint32_t j = 0; j < n_in; j++)
            {
                double target = (i == j) ? 1.0 : 0.0;
                size_t idx    = (size_t) i * n_in + j;
                H_truth[(size_t) (L_truth - 1) * nn + idx] =
                    target - S_wfs[(size_t) (L_truth - 2) * nn + idx];
            }
        }

        // Truncated L_kern MIMO impulse kernel with exact DC closure at tap L_kern - 1
        memcpy(H_mimo, S_wfs, sizeof(double) * nn);
        for (uint32_t l = 1; l + 1 < L_kern; l++)
        {
            for (size_t idx = 0; idx < nn; idx++)
            {
                H_mimo[(size_t) l * nn + idx] =
                    S_wfs[(size_t) l * nn + idx] - S_wfs[(size_t) (l - 1) * nn + idx];
            }
        }
        for (uint32_t i = 0; i < n_in; i++)
        {
            for (uint32_t j = 0; j < n_in; j++)
            {
                double target = (i == j) ? 1.0 : 0.0;
                size_t idx    = (size_t) i * n_in + j;
                H_mimo[(size_t) (L_kern - 1) * nn + idx] =
                    target - S_wfs[(size_t) (L_kern - 2) * nn + idx];
            }
        }

        // Optional block-diagonal truncation and/or off-diagonal pruning, preserving sum H = I
        if (p_blocksize > 0 || p_offdiag_thr > 0.0f)
        {
            uint32_t blk = p_blocksize;
            double   oth = (p_offdiag_thr > 0.0f) ? (double) p_offdiag_thr : 0.0;
            for (uint32_t i = 0; i < n_in; i++)
            {
                for (uint32_t j = 0; j < n_in; j++)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    int drop_pair = (blk > 0 && (i / blk) != (j / blk));
                    if (drop_pair)
                    {
                        for (uint32_t l = 0; l < L_kern; l++)
                        {
                            H_mimo[(size_t) l * nn + (size_t) i * n_in + j] = 0.0;
                        }
                    }
                    else if (oth > 0.0)
                    {
                        double sum_od = 0.0;
                        for (uint32_t l = 0; l + 1 < L_kern; l++)
                        {
                            double *vp = &H_mimo[(size_t) l * nn + (size_t) i * n_in + j];
                            if (fabs(*vp) < oth)
                            {
                                *vp = 0.0;
                            }
                            sum_od += *vp;
                        }
                        H_mimo[(size_t) (L_kern - 1) * nn + (size_t) i * n_in + j] = -sum_od;
                    }
                }
            }
        }

        // Extract normalized diagonal kernel H_diag[l, m] with sum_l H_diag[l, m] = 1.0
        double sum_lat = 0.0;
        for (uint32_t m = 0; m < n_in; m++)
        {
            double sd = 0.0;
            for (uint32_t l = 0; l < L_kern; l++)
            {
                double v                      = H_mimo[(size_t) l * nn + (size_t) m * n_in + m];
                H_diag[(size_t) l * n_in + m] = v;
                sd                           += v;
            }
            if (fabs(sd) > 1.0e-12)
            {
                double inv_sd = 1.0 / sd;
                for (uint32_t l = 0; l < L_kern; l++)
                {
                    H_diag[(size_t) l * n_in + m] *= inv_sd;
                }
            }
            double cm = 0.0;
            for (uint32_t l = 0; l < L_kern; l++)
            {
                cm += (double) l * H_diag[(size_t) l * n_in + m];
            }
            sum_lat += cm;
        }
        double lat_cent = sum_lat / (double) n_in;

        // Verify exact DC gain error max |sum_l H_mimo[l] - I|
        double dc_err_max = 0.0;
        for (uint32_t i = 0; i < n_in; i++)
        {
            for (uint32_t j = 0; j < n_in; j++)
            {
                double s = 0.0;
                for (uint32_t l = 0; l < L_kern; l++)
                {
                    s += H_mimo[(size_t) l * nn + (size_t) i * n_in + j];
                }
                double e = fabs(s - ((i == j) ? 1.0 : 0.0));
                if (e > dc_err_max)
                {
                    dc_err_max = e;
                }
            }
        }

        // Synthetic Kolmogorov AR(1) PSOL benchmark (2-tap vs diag FIR vs MIMO FIR)
        float p2tap = 0.0f, pdiag = 0.0f, pmimo = 0.0f;
        fdr_bench_psol(H_truth, L_truth, H_mimo, H_diag, L_kern, n_in, &p2tap, &pdiag, &pmimo);

        // Populate FPS outputs and sync back to FPS SHM
        o_n_in               = n_in;
        o_n_out              = n_out;
        o_n_taps             = L_kern;
        o_nbsubfr            = nbsubfr;
        o_n_repaired         = n_repaired;
        o_dt_us              = (float) (dt_sec * 1.0e6);
        o_wfsfreq_hz         = (float) wfsfreq;
        o_lat_centroid_fr    = (float) lat_cent;
        o_peak_overshoot_pct = (float) mean_ov;
        o_peak_offdiag_pct   = (float) peak_off;
        o_tail_err_pct       = (float) tail_err_pct;
        o_dc_err_max         = (float) dc_err_max;
        o_psol_2tap_pct      = p2tap;
        o_psol_diag_pct      = pdiag;
        o_psol_mimo_pct      = pmimo;

        if (dcfpsptr != NULL)
        {
            fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
        }

        // Convert and write FITS float32 outputs
        int    wr_rc   = 0;
        float *f32_buf = (float *) malloc(sizeof(float) * ((size_t) L_kern * nn));
        if (f32_buf == NULL)
        {
            wr_rc = -1;
        }
        else
        {
            for (size_t idx = 0; idx < (size_t) L_kern * nn; idx++)
            {
                f32_buf[idx] = (float) H_mimo[idx];
            }
            long ax_mimo[3] = { (long) n_in, (long) n_in, (long) L_kern };
            if (fdr_write_fits_f32(p_outmimo, f32_buf, 3, ax_mimo, 0, (long) n_in, (long) L_kern,
                                   phase_fr, lat_cent, wfsfreq, dt_sec, (long) nbsubfr, mean_ov,
                                   peak_off, tail_err_pct, p2tap, pdiag, pmimo) != 0)
            {
                wr_rc = -1;
            }
            free(f32_buf);
        }

        if (wr_rc == 0 && p_outdiag[0] != '\0')
        {
            float *fd = (float *) malloc(sizeof(float) * (size_t) L_kern * n_in);
            if (fd != NULL)
            {
                for (size_t idx = 0; idx < (size_t) L_kern * n_in; idx++)
                {
                    fd[idx] = (float) H_diag[idx];
                }
                long ax_d[3] = { (long) n_in, (long) L_kern, 1 };
                if (fdr_write_fits_f32(p_outdiag, fd, 2, ax_d, 1, (long) n_in, (long) L_kern,
                                       phase_fr, lat_cent, wfsfreq, dt_sec, (long) nbsubfr, mean_ov,
                                       peak_off, tail_err_pct, p2tap, pdiag, pmimo) != 0)
                {
                    wr_rc = -1;
                }
                free(fd);
            }
            else
            {
                wr_rc = -1;
            }
        }

        if (wr_rc == 0 && p_outstatic[0] != '\0')
        {
            float *fs = (float *) malloc(sizeof(float) * (size_t) n_in * n_out);
            if (fs != NULL)
            {
                for (size_t idx = 0; idx < (size_t) n_in * n_out; idx++)
                {
                    fs[idx] = (float) m_stat[idx];
                }
                long ax_s[3] = { (long) n_out, (long) n_in, 1 };
                if (fdr_write_fits_f32(p_outstatic, fs, 2, ax_s, 2, (long) n_in, (long) L_kern,
                                       phase_fr, lat_cent, wfsfreq, dt_sec, (long) nbsubfr, mean_ov,
                                       peak_off, tail_err_pct, p2tap, pdiag, pmimo) != 0)
                {
                    wr_rc = -1;
                }
                free(fs);
            }
            else
            {
                wr_rc = -1;
            }
        }

        if (wr_rc == 0 && p_outstep[0] != '\0')
        {
            uint32_t n_post = n_t - q0_bin;
            float   *fst    = (float *) malloc(sizeof(float) * (size_t) n_post * nn);
            if (fst != NULL)
            {
                for (size_t idx = 0; idx < (size_t) n_post * nn; idx++)
                {
                    fst[idx] = (float) s_sub[(size_t) q0_bin * nn + idx];
                }
                long ax_st[3] = { (long) n_in, (long) n_in, (long) n_post };
                if (fdr_write_fits_f32(p_outstep, fst, 3, ax_st, 3, (long) n_in, (long) L_kern,
                                       phase_fr, lat_cent, wfsfreq, dt_sec, (long) nbsubfr, mean_ov,
                                       peak_off, tail_err_pct, p2tap, pdiag, pmimo) != 0)
                {
                    wr_rc = -1;
                }
                free(fst);
            }
            else
            {
                wr_rc = -1;
            }
        }

        if (processinfo != NULL)
        {
            processinfo_WriteMessage_fmt(
                processinfo,
                "N=%u L=%u rep=%u lat=%.2ffr ov=%.1f%% off=%.1f%% psol=%.2f/%.2f/%.2f%%", n_in,
                L_kern, n_repaired, lat_cent, mean_ov, peak_off, p2tap, pdiag, pmimo);
        }

        if (wr_rc == 0)
        {
            ret = RETURN_SUCCESS;
        }

    cleanup:
        free(r3_raw);
        free(m_stat);
        free(m_copy);
        free(svd_s);
        free(svd_u);
        free(svd_vt);
        free(c_pinv);
        free(s_sub);
        free(S_wfs);
        free(H_truth);
        free(H_mimo);
        free(H_diag);
        break;
    }
    MILK_PROCINFO_END(self);
    return ret;
}

const MILK_RUNNABLE runnable_AOloopControl_acquireCalib_factordynrm = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "cacaoac-factordynrm",
                 .cmdkey      = "factordynrm",
                 .description = "factor raw 3D RM into static M_cal and WFS-rate H_eff[l]",
                 .description_long =
                     "Factor a raw exposure-integrated 3D response cube from cacaoac-measdynresp "
                     "into static spatial response M_cal and normalized WFS-rate MIMO/diagonal "
                     "causal impulse kernels H_eff[l] for Dynamic PSOL, online OG, and 2WFS1DM." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaoac_factordynrm),
    .compute = factordynrm_compute,
};
EXPORT_FPSEXEC(cacao, "cacaoac-factordynrm", runnable_AOloopControl_acquireCalib_factordynrm);
