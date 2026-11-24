// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    compute_dm_preshaper.c
 * @brief   Synthesize high-rate look-ahead DM command pre-shaper kernel from raw 3D RM
 *
 * Consumes a raw exposure-integrated 3D response matrix resp3D.fits [N_out, N_in, N_t]
 * acquired by cacaoac-measdynresp together with its input mode cube inmodes.fits
 * [dmx, dmy, N_in], projects the dynamics into the N_in-dimensional DM input subspace,
 * and solves a direct constrained time-domain Tikhonov least-squares model-matching
 * problem for the look-ahead buffer kernel C[0..M_buf-1]:
 *
 *   u_shaped[n] = u_raw[n] + sum_{k=0}^{M_buf-1} C[k] * (u_raw[n - k] - u_raw[n - k - 1])
 *
 * Parameterizing directly by C[0..M_buf-1] with C[M_buf] = 0 enforces exact unity DC
 * gain (sum K[m] = I) by construction and avoids dividing by WFS camera exposure sinc
 * nulls by matching S_raw * K directly against (b_Texp * S_target_phys).
 *
 * Outputs a single self-contained multi-extension FITS (MEF) file:
 *   - Primary HDU (HDU 0): C look-ahead kernel (float32)
 *       KMODE = 0 (MIMO modal/subspace):      [N_in, N_in, M_buf]
 *       KMODE = 1 (SISO decoupled modal):     [N_in, N_in, M_buf] (diagonal slices)
 *       KMODE = 2 (SISO uniform broadcast):   [1, 1, M_buf]
 *       KMODE = 3 (SISO per-actuator):        [dmx, dmy, M_buf]
 *       KMODE = 4 (MIMO per-actuator stencil):[N_nb, dmx*dmy, M_buf] (gather weights)
 *   - Extension P_FWD  (KMODE 0, 1): forward DM mode basis [dmx, dmy, N_in] (float32)
 *   - Extension P_PINV (KMODE 0, 1): dual projection basis [dmx, dmy, N_in] (float32)
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
#include <unistd.h>
#ifdef _OPENMP
#    include <omp.h>
#endif

#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"
#include "milk_blas_lapacke.h"

#include "computeCalib.h"

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     p_resp3d[FUNCTION_PARAMETER_STRMAXLEN]    = "resp3D.fits";
static char     p_inmodes[FUNCTION_PARAMETER_STRMAXLEN]   = "inmodes.fits";
static char     p_staticrm[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_outkernel[FUNCTION_PARAMETER_STRMAXLEN] = "dm_preshaper.fits";
static char     p_outpred[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static uint32_t p_mode                                    = 0;
static float    p_svdlim                                  = 1.0e-3f;
static uint32_t p_ntaps                                   = 24;
static float    p_tau_target_us                           = 100.0f;
static float    p_delay_us                                = -1.0f;
static float    p_texp_us                                 = -1.0f;
static float    p_dt_us                                   = 0.0f;
static float    p_lambda_ctrl                             = 0.02f;
static float    p_lambda_diff                             = 0.02f;
static float    p_max_gain                                = 2.5f;
static int64_t  p_deconv_exp                              = 1;
static float    p_lambda_deconv                           = 2.8e-2f;
static float    p_deconv_tmax_ms                          = 0.0f;
static int64_t  p_energy_weight                           = 1;
static uint32_t p_stencil_rad                             = 2;
static int64_t  p_fill_uncal                              = 0;

static float o_err_before      = 0.0f;
static float o_err_after       = 0.0f;
static float o_err_reduction_x = 0.0f;
static float o_ov_before_pct   = 0.0f;
static float o_ov_after_pct    = 0.0f;
static float o_off_reduction_x = 0.0f;
static float o_max_step_gain   = 0.0f;

#define FPS_PARAMS_COMPDMPRESHAPER(X)                                                              \
    X(".resp3d", p_resp3d, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "input raw 3D response cube [N_out, N_in, N_t]")                                             \
    X(".inmodes", p_inmodes, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "input DM poke modes 3D FITS [dmx, dmy, N_in]")                                              \
    X(".staticrm", p_staticrm, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "optional static 2D RM [N_out, N_in] (empty = auto)")                                        \
    X(".outkernel", p_outkernel, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "output self-contained MEF pre-shaper kernel FITS")                                          \
    X(".outpred", p_outpred, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "optional predicted shaped modal step cube [N_in, N_in, L]")                                 \
    X(".mode", &p_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                 \
      "0: MIMO modal | 1: SISO modal | 2: SISO uniform | 3: SISO per-act | 4: MIMO per-act stencil") \
    X(".svdlim", &p_svdlim, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,            \
      "relative SVD threshold for static & mode pseudo-inverses")                                  \
    X(".n_taps", &p_ntaps, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,              \
      "total FIR taps M (M_buf = M - 1 look-ahead slots; 0 = auto)")                               \
    X(".tau_target_us", &p_tau_target_us, FPTYPE_FLOAT32, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                                                     \
      "target critically-damped physical rise time constant [us]")                                 \
    X(".delay_us", &p_delay_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "target causal transport delay [us] (< 0 = auto-detect)")                                    \
    X(".texp_us", &p_texp_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "WFS exposure boxcar width [us] (< 0 = auto from FITS, 0 = instant)")                        \
    X(".dt_us", &p_dt_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,              \
      "override 3D RM time-bin width [us] (<= 0 = read DT_SEC)")                                   \
    X(".lambda_ctrl", &p_lambda_ctrl, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "Tikhonov control-effort penalty on ||C||^2")                                                \
    X(".lambda_diff", &p_lambda_diff, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "temporal smoothness & zero-tail penalty on ||dC||^2")                                       \
    X(".max_gain", &p_max_gain, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "safety cap on max|I + C[k]| step gain (<= 0 = uncapped)")                                   \
    X(".deconv_exp", &p_deconv_exp, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "two-sided finite-support Toeplitz WFS-exposure deconvolution before inversion")             \
    X(".lambda_deconv", &p_lambda_deconv, FPTYPE_FLOAT32, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                                                     \
      "dimensionless Toeplitz exposure deconvolution weight (scaled by diag(B^T B) = 1/n_box)")    \
    X(".deconv_tmax_ms", &p_deconv_tmax_ms, FPTYPE_FLOAT32, 0,                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                                                     \
      "settling horizon [ms] for two-sided deconvolution (<= 0 = auto)")                           \
    X(".energy_weight", &p_energy_weight, FPTYPE_ONOFF, 0,                                         \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                                                     \
      "weight MIMO residuals & penalties by physical static WFS & actuator L2 energy")             \
    X(".stencil_rad", &p_stencil_rad, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "local neighborhood half-width R for KMODE=4 ((2R+1)x(2R+1) per-actuator stencil, 1..5)")    \
    X(".fill_uncal", &p_fill_uncal, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "KMODE=3/4: smoothly interpolate uncalibrated actuators from calibrated neighbors")          \
    X(".out.err_before", &o_err_before, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "unshaped tracking error norm vs ideal target")                                              \
    X(".out.err_after", &o_err_after, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "predicted pre-shaped tracking error norm vs ideal target")                                  \
    X(".out.err_reduction_x", &o_err_reduction_x, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,        \
      "tracking error reduction factor")                                                           \
    X(".out.ov_before_pct", &o_ov_before_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,            \
      "mean unshaped diagonal overshoot [%]")                                                      \
    X(".out.ov_after_pct", &o_ov_after_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,              \
      "mean predicted pre-shaped diagonal overshoot [%]")                                          \
    X(".out.off_reduction_x", &o_off_reduction_x, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,        \
      "peak off-diagonal cross-talk reduction factor")                                             \
    X(".out.max_step_gain", &o_max_step_gain, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,            \
      "peak shaped step command gain max|I + C[k]|")

MILK_PARAMS_DEFINE(cacaocc_compdmpreshaper, FPS_PARAMS_COMPDMPRESHAPER);

/* ================================================================
 * 2.  FITS I/O & PATH HELPERS
 * ============================================================= */

typedef struct
{
    double dt_sec;
    double wfsdt_sec;
    long   q0_bin;
    long   nbsubfr;
    long   has_dt;
    long   has_q0;
    long   has_nsub;
} PS_RESP_HDR;

static void ps_ensure_parent_dir(const char *path)
{
    const char *slash = strrchr(path, '/');
    if (slash == NULL || slash == path)
    {
        return;
    }
    char   buf[STRINGMAXLEN_FULLFILENAME];
    size_t len = (size_t) (slash - path);
    if (len >= sizeof(buf))
    {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, path, len);
    buf[len] = '\0';

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

static int ps_load_fits_f64(const char  *raw_path,
                            double     **out_data,
                            int         *out_naxis,
                            long         out_naxes[3],
                            PS_RESP_HDR *hdr_opt)
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
        (void) fits_read_key(fptr, TDOUBLE, "WFSDT", &hdr_opt->wfsdt_sec, NULL, &kst);
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
    fprintf(stderr, "compdmpreshaper: built without CFITSIO support\n");
    return -1;
#endif
}

typedef struct
{
    uint32_t kmode;
    double   dt_sec;
    uint32_t dmx;
    uint32_t dmy;
    uint32_t n_in;
    uint32_t m_buf;
    uint32_t stencil_rad;
    uint32_t stencil_nb;
    double   tau_tar_us;
    double   delay_us;
    double   texp_us;
    double   lam_ctrl;
    double   lam_diff;
    double   lam_deconv;
    int      deconv_exp;
    int      energy_weight;
    double   max_gain;
    double   err_before;
    double   err_after;
    double   ov_before_pct;
    double   ov_after_pct;
    double   off_red_x;
} PS_OUT_META;

static int ps_write_mef_kernel(const char        *raw_path,
                               const float       *c_data,
                               const long         c_naxes[3],
                               const float       *p_fwd,
                               const float       *p_pinv,
                               const long         p_naxes[3],
                               const PS_OUT_META *meta)
{
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    ps_ensure_parent_dir(path);

#ifdef USE_CFITSIO
    char tmp_path[STRINGMAXLEN_FULLFILENAME];
    char cfits_path[STRINGMAXLEN_FULLFILENAME];
    snprintf(tmp_path, sizeof(tmp_path), "%s.%d.tmp", path, (int) getpid());
    snprintf(cfits_path, sizeof(cfits_path), "!%s", tmp_path);

    fitsfile *fptr   = NULL;
    int       status = 0;
    if (fits_create_file(&fptr, cfits_path, &status) != 0)
    {
        fprintf(stderr, "compdmpreshaper: fits_create_file '%s' failed (%d)\n", tmp_path, status);
        return -1;
    }

    long     ax_c[3] = { c_naxes[0], c_naxes[1], c_naxes[2] };
    LONGLONG n_c     = (LONGLONG) ax_c[0] * ax_c[1] * ax_c[2];
    fits_create_img(fptr, FLOAT_IMG, 3, ax_c, &status);

    long   lval;
    double dval;

    lval = (long) meta->kmode;
    fits_update_key(fptr, TLONG, "KMODE", &lval,
                    "0=MIMO, 1=SISO_MOD, 2=SISO_UNI, 3=SISO_ACT, 4=MIMO_ACT", &status);
    dval = meta->dt_sec;
    fits_update_key(fptr, TDOUBLE, "DT_SEC", &dval, "Kernel tick period [s]", &status);
    lval = (long) meta->dmx;
    fits_update_key(fptr, TLONG, "DMX", &lval, "DM X size", &status);
    lval = (long) meta->dmy;
    fits_update_key(fptr, TLONG, "DMY", &lval, "DM Y size", &status);
    lval = (long) meta->n_in;
    fits_update_key(fptr, TLONG, "NBIN", &lval, "Number of calibrated DM modes/actuators", &status);
    lval = (long) meta->m_buf;
    fits_update_key(fptr, TLONG, "MBUF", &lval, "Number of look-ahead buffer slots (M - 1)",
                    &status);
    if (meta->kmode == 4u)
    {
        lval = (long) meta->stencil_rad;
        fits_update_key(fptr, TLONG, "STNCLRAD", &lval,
                        "Per-actuator local MIMO stencil radius R ((2R+1)x(2R+1))", &status);
        lval = (long) meta->stencil_nb;
        fits_update_key(fptr, TLONG, "STNCLNB", &lval,
                        "Number of neighbors per actuator stencil ((2R+1)^2)", &status);
    }
    dval = meta->tau_tar_us;
    fits_update_key(fptr, TDOUBLE, "TAUTARUS", &dval, "Target physical rise constant [us]",
                    &status);
    dval = meta->delay_us;
    fits_update_key(fptr, TDOUBLE, "DELAYUS", &dval, "Target causal transport delay [us]", &status);
    dval = meta->texp_us;
    fits_update_key(fptr, TDOUBLE, "TEXPUS", &dval, "WFS exposure boxcar width [us]", &status);
    dval = meta->lam_ctrl;
    fits_update_key(fptr, TDOUBLE, "LAMCTRL", &dval, "Control-effort regularization lambda",
                    &status);
    dval = meta->lam_diff;
    fits_update_key(fptr, TDOUBLE, "LAMDIFF", &dval, "Smoothness regularization lambda", &status);
    lval = (long) meta->deconv_exp;
    fits_update_key(fptr, TLONG, "DECONV", &lval, "1=two-sided Toeplitz exposure deconv", &status);
    dval = meta->lam_deconv;
    fits_update_key(fptr, TDOUBLE, "LAMDEC", &dval, "Exposure deconvolution regularization",
                    &status);
    lval = (long) meta->energy_weight;
    fits_update_key(fptr, TLONG, "ENRGWGT", &lval, "1=physical WFS/actuator energy weighting",
                    &status);
    dval = meta->max_gain;
    fits_update_key(fptr, TDOUBLE, "MAXGAIN", &dval, "Peak shaped step command gain", &status);
    dval = meta->err_before;
    fits_update_key(fptr, TDOUBLE, "ERRPRE", &dval, "Unshaped RMS tracking error", &status);
    dval = meta->err_after;
    fits_update_key(fptr, TDOUBLE, "ERRPOST", &dval, "Predicted pre-shaped tracking error",
                    &status);
    dval = meta->ov_before_pct;
    fits_update_key(fptr, TDOUBLE, "OVPRE", &dval, "Mean unshaped overshoot [%]", &status);
    dval = meta->ov_after_pct;
    fits_update_key(fptr, TDOUBLE, "OVPOST", &dval, "Mean predicted pre-shaped overshoot [%]",
                    &status);
    dval = meta->off_red_x;
    fits_update_key(fptr, TDOUBLE, "OFFREDX", &dval, "Peak off-diagonal reduction factor", &status);

    fits_write_img(fptr, TFLOAT, 1, n_c, (void *) c_data, &status);

    /* Append forward and dual projection bases as named image extensions for modal/MIMO modes */
    if (p_fwd != NULL && p_pinv != NULL)
    {
        long     ax_p[3] = { p_naxes[0], p_naxes[1], p_naxes[2] };
        LONGLONG n_p     = (LONGLONG) ax_p[0] * ax_p[1] * ax_p[2];

        fits_create_img(fptr, FLOAT_IMG, 3, ax_p, &status);
        fits_update_key(fptr, TSTRING, "EXTNAME", (void *) "P_FWD", "Forward DM mode basis",
                        &status);
        fits_write_img(fptr, TFLOAT, 1, n_p, (void *) p_fwd, &status);

        fits_create_img(fptr, FLOAT_IMG, 3, ax_p, &status);
        fits_update_key(fptr, TSTRING, "EXTNAME", (void *) "P_PINV",
                        "Dual projection basis (P_PINV^T P_FWD = I)", &status);
        fits_write_img(fptr, TFLOAT, 1, n_p, (void *) p_pinv, &status);
    }

    fits_close_file(fptr, &status);
    if (status != 0)
    {
        remove(tmp_path);
        fprintf(stderr, "compdmpreshaper: FITS write failed for '%s' (%d)\n", path, status);
        return -1;
    }
    if (rename(tmp_path, path) != 0)
    {
        remove(tmp_path);
        fprintf(stderr, "compdmpreshaper: rename '%s' -> '%s' failed: %s\n", tmp_path, path,
                strerror(errno));
        return -1;
    }
    return 0;
#else
    (void) c_data;
    (void) c_naxes;
    (void) p_fwd;
    (void) p_pinv;
    (void) p_naxes;
    (void) meta;
    return -1;
#endif
}

static int ps_write_simple_fits_f32(const char  *raw_path,
                                    const float *data,
                                    const long   naxes[3],
                                    double       dt_sec)
{
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    ps_ensure_parent_dir(path);

#ifdef USE_CFITSIO
    char tmp_path[STRINGMAXLEN_FULLFILENAME];
    char cfits_path[STRINGMAXLEN_FULLFILENAME];
    snprintf(tmp_path, sizeof(tmp_path), "%s.%d.tmp", path, (int) getpid());
    snprintf(cfits_path, sizeof(cfits_path), "!%s", tmp_path);

    fitsfile *fptr   = NULL;
    int       status = 0;
    if (fits_create_file(&fptr, cfits_path, &status) != 0)
    {
        return -1;
    }
    long     ax[3] = { naxes[0], naxes[1], naxes[2] };
    LONGLONG n_el  = (LONGLONG) ax[0] * ax[1] * ax[2];
    fits_create_img(fptr, FLOAT_IMG, 3, ax, &status);
    fits_update_key(fptr, TDOUBLE, "DT_SEC", &dt_sec, "Time-bin width [s]", &status);
    fits_write_img(fptr, TFLOAT, 1, n_el, (void *) data, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        remove(tmp_path);
        return -1;
    }
    if (rename(tmp_path, path) != 0)
    {
        remove(tmp_path);
        return -1;
    }
    return 0;
#else
    (void) data;
    (void) naxes;
    (void) dt_sec;
    return -1;
#endif
}

/* ================================================================
 * 3.  LINEAR ALGEBRA HELPERS (SVD PSEUDO-INVERSE & CHOLESKY)
 * ============================================================= */

/**
 * Compute row-major Moore-Penrose pseudo-inverse helper:
 * Given M [rows x cols] (row-major), compute P [rows x cols] such that:
 *   - if rows <= cols: P @ M^T = I_rows  (left inverse of M^T)
 *   - if rows >  cols: M^T @ P = I_cols
 * Equivalently, P = (M^+) ^ T = U * diag(1/s) * V^T.
 */
static int ps_svd_pinv_transpose(const double *mat_in,
                                 size_t        rows,
                                 size_t        cols,
                                 double        svdlim,
                                 double       *pinv_t_out)
{
    const size_t k = (rows < cols) ? rows : cols;
    double      *a = (double *) malloc(sizeof(double) * rows * cols);
    double      *s = (double *) malloc(sizeof(double) * k);
    double      *u = (double *) malloc(sizeof(double) * rows * k);
    double     *vt = (double *) malloc(sizeof(double) * k * cols);
    if (!a || !s || !u || !vt)
    {
        free(vt);
        free(u);
        free(s);
        free(a);
        return -1;
    }
    memcpy(a, mat_in, sizeof(double) * rows * cols);

    lapack_int info = LAPACKE_dgesdd(LAPACK_ROW_MAJOR, 'S', (lapack_int) rows, (lapack_int) cols, a,
                                     (lapack_int) cols, s, u, (lapack_int) k, vt,
                                     (lapack_int) cols);
    if (info != 0)
    {
        fprintf(stderr, "compdmpreshaper: LAPACKE_dgesdd failed (info=%d)\n", (int) info);
        free(vt);
        free(u);
        free(s);
        free(a);
        return -1;
    }

    const double s_cut = (s[0] > 0.0) ? (svdlim * s[0]) : 0.0;
    for (size_t j = 0; j < k; j++)
    {
        const double inv_s = (s[j] > s_cut) ? (1.0 / s[j]) : 0.0;
        for (size_t i = 0; i < rows; i++)
        {
            u[i * k + j] *= inv_s;
        }
    }

    /* pinv_t_out [rows x cols] = (U * S^-1) [rows x k] @ VT [k x cols] */
#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) rows, (int) cols, (int) k, 1.0, u,
                (int) k, vt, (int) cols, 0.0, pinv_t_out, (int) cols);
#else
#    ifdef _OPENMP
#        pragma omp parallel for schedule(static)
#    endif
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t c = 0; c < cols; c++)
        {
            double sum = 0.0;
            for (size_t j = 0; j < k; j++)
            {
                sum += u[i * k + j] * vt[j * cols + c];
            }
            pinv_t_out[i * cols + c] = sum;
        }
    }
#endif

    free(vt);
    free(u);
    free(s);
    free(a);
    return 0;
}

static double ps_crossing_time(const double *y, size_t len, double dt_sec, double thr)
{
    for (size_t n = 0; n < len; n++)
    {
        if (y[n] >= thr)
        {
            if (n == 0)
            {
                return 0.5 * dt_sec;
            }
            double dy = y[n] - y[n - 1];
            double f  = (dy > 1.0e-12) ? ((thr - y[n - 1]) / dy) : 0.0;
            return ((double) (n - 1) + f + 0.5) * dt_sec;
        }
    }
    return (double) len * dt_sec;
}

static void ps_build_target_step(double *s_tar,
                                 size_t   len,
                                 double   dt_sec,
                                 double   delay_sec,
                                 double   tau_tar_sec,
                                 double   texp_sec)
{
    const size_t sub    = 16;
    const size_t n_fine = len * sub;
    const double dt_f   = dt_sec / (double) sub;

    double *cs = (double *) malloc(sizeof(double) * (n_fine + 1));
    if (cs == NULL)
    {
        for (size_t n = 0; n < len; n++)
        {
            double t = ((double) n + 0.5) * dt_sec;
            s_tar[n] = (t >= delay_sec) ? 1.0 : 0.0;
        }
        return;
    }

    cs[0] = 0.0;
    for (size_t i = 0; i < n_fine; i++)
    {
        double t   = ((double) i + 0.5) * dt_f;
        double val = 0.0;
        if (t >= delay_sec)
        {
            if (tau_tar_sec <= 1.0e-7)
            {
                val = 1.0;
            }
            else
            {
                double x = (t - delay_sec) / tau_tar_sec;
                val      = 1.0 - exp(-x);
            }
        }
        cs[i + 1] = cs[i] + val;
    }

    size_t n_box = (texp_sec > 0.5 * dt_f) ? (size_t) llround(texp_sec / dt_f) : 1;
    if (n_box < 1)
    {
        n_box = 1;
    }

    for (size_t n = 0; n < len; n++)
    {
        size_t idx = n * sub + (sub / 2);
        size_t j0  = (idx + 1 > n_box) ? (idx + 1 - n_box) : 0;
        s_tar[n]   = (cs[idx + 1] - cs[j0]) / (double) n_box;
    }

    free(cs);
}

/**
 * Two-sided finite-support Toeplitz exposure deconvolution for a [len, n_in, n_in] step cube.
 *
 * Uses both physical boundary conditions (s_phys[n] = 0 for n < n_on and s_phys[n] = I for
 * n >= n_end) so the tall rectangular boxcar operator B [len x l_act] has full column rank
 * with zero null space even at the periodic camera sinc nulls f = k / T_exp.
 *
 * Return: 0 on success, -1 if deconvolution cannot be applied (caller falls back to raw s_obs).
 */
static int ps_deconv_twosided_toeplitz(const double *s_obs,
                                       size_t        len,
                                       size_t        n_in,
                                       size_t        n_box,
                                       double        dt_sec,
                                       double        lam_dec,
                                       double        tmax_ms,
                                       double       *s_phys_out,
                                       size_t       *out_n_on)
{
    if (n_box < 2 || len <= 2 * n_box + 4)
    {
        return -1;
    }

    size_t n_on = 0;
    for (size_t n = 0; n < len; n++)
    {
        double tr = 0.0;
        for (size_t i = 0; i < n_in; i++)
        {
            tr += s_obs[(n * n_in + i) * n_in + i];
        }
        if (tr / (double) n_in >= 0.015)
        {
            n_on = n;
            break;
        }
    }

    const size_t max_end = len - (n_box - 1);
    const size_t min_end = n_on + 2 * n_box;
    if (min_end >= max_end)
    {
        return -1;
    }

    size_t n_end = (tmax_ms > 0.0) ? (size_t) llround((tmax_ms * 1.0e-3) / dt_sec)
                                   : (size_t) llround(0.018 / dt_sec);
    if (n_end > max_end)
    {
        n_end = max_end;
    }
    if (n_end < min_end)
    {
        n_end = min_end;
    }
    const size_t l_act = n_end - n_on;
    const size_t n_col = n_in * n_in;

    double *b_mat  = (double *) calloc(len * l_act, sizeof(double));
    double *c_tail = (double *) calloc(len, sizeof(double));
    double *w_env  = (double *) malloc(sizeof(double) * l_act);
    double *a_dec  = (double *) calloc(l_act * l_act, sizeof(double));
    double *y_rhs  = (double *) malloc(sizeof(double) * len * n_col);
    double *b_rhs  = (double *) calloc(l_act * n_col, sizeof(double));
    if (!b_mat || !c_tail || !w_env || !a_dec || !y_rhs || !b_rhs)
    {
        free(b_rhs);
        free(y_rhs);
        free(a_dec);
        free(w_env);
        free(c_tail);
        free(b_mat);
        return -1;
    }

    const double inv_box = 1.0 / (double) n_box;
    for (size_t n = 0; n < len; n++)
    {
        for (size_t j = 0; j < n_box; j++)
        {
            if (n < j)
            {
                continue;
            }
            size_t idx = n - j;
            if (idx >= n_on && idx < n_end)
            {
                b_mat[n * l_act + (idx - n_on)] += inv_box;
            }
            else if (idx >= n_end)
            {
                c_tail[n] += inv_box;
            }
        }
    }

    for (size_t k = 0; k < l_act; k++)
    {
        double ex = 1.5 * (((double) k + 0.5) * dt_sec / 0.008);
        if (ex > 4.0)
        {
            ex = 4.0;
        }
        w_env[k] = exp(ex);
    }

    for (size_t n = 0; n < len; n++)
    {
        const double *s_sl = &s_obs[n * n_col];
        double       *y_sl = &y_rhs[n * n_col];
        const double  ct   = c_tail[n];
        for (size_t r = 0; r < n_in; r++)
        {
            for (size_t c = 0; c < n_in; c++)
            {
                size_t m = r * n_in + c;
                y_sl[m]  = s_sl[m] - ((r == c) ? ct : 0.0);
            }
        }
    }

#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
    cblas_dsyrk(CblasRowMajor, CblasUpper, CblasTrans, (int) l_act, (int) len, 1.0, b_mat,
                (int) l_act, 0.0, a_dec, (int) l_act);
    cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, (int) l_act, (int) n_col, (int) len, 1.0,
                b_mat, (int) l_act, y_rhs, (int) n_col, 0.0, b_rhs, (int) n_col);
#else
    for (size_t i = 0; i < l_act; i++)
    {
        for (size_t j = i; j < l_act; j++)
        {
            double sum = 0.0;
            for (size_t n = 0; n < len; n++)
            {
                sum += b_mat[n * l_act + i] * b_mat[n * l_act + j];
            }
            a_dec[i * l_act + j] = sum;
        }
        for (size_t m = 0; m < n_col; m++)
        {
            double sum = 0.0;
            for (size_t n = 0; n < len; n++)
            {
                sum += b_mat[n * l_act + i] * y_rhs[n * n_col + m];
            }
            b_rhs[i * n_col + m] = sum;
        }
    }
#endif

    /* Scale dimensionless lambda_deconv by diag(B^T B) = 1/n_box for NBsubfr invariance */
    const double lam_eff = lam_dec * inv_box;
    const double lam_env = 0.15 * lam_eff;
    const double lam_sm  = 0.85 * lam_eff;
    for (size_t i = 0; i < l_act; i++)
    {
        for (size_t j = i + 1; j < l_act; j++)
        {
            a_dec[j * l_act + i] = a_dec[i * l_act + j];
        }
        a_dec[i * l_act + i] += lam_env * w_env[i] + 2.0 * lam_sm;
        if (i + 1 < l_act)
        {
            a_dec[i * l_act + (i + 1)] -= lam_sm;
            a_dec[(i + 1) * l_act + i] -= lam_sm;
        }
    }

    for (size_t k = 0; k < l_act; k++)
    {
        const double add_env = lam_env * w_env[k];
        for (size_t i = 0; i < n_in; i++)
        {
            b_rhs[k * n_col + i * n_in + i] += add_env;
        }
    }
    for (size_t i = 0; i < n_in; i++)
    {
        b_rhs[(l_act - 1) * n_col + i * n_in + i] += lam_sm;
    }

    lapack_int info =
        LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'U', (lapack_int) l_act, a_dec, (lapack_int) l_act);
    if (info == 0)
    {
        info = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', (lapack_int) l_act, (lapack_int) n_col, a_dec,
                              (lapack_int) l_act, b_rhs, (lapack_int) n_col);
    }

    if (info == 0)
    {
        memset(s_phys_out, 0, sizeof(double) * len * n_col);
        for (size_t k = 0; k < l_act; k++)
        {
            memcpy(&s_phys_out[(n_on + k) * n_col], &b_rhs[k * n_col], sizeof(double) * n_col);
        }
        for (size_t n = n_end; n < len; n++)
        {
            double *sl = &s_phys_out[n * n_col];
            for (size_t i = 0; i < n_in; i++)
            {
                sl[i * n_in + i] = 1.0;
            }
        }
        if (out_n_on != NULL)
        {
            *out_n_on = n_on;
        }
    }

    free(b_rhs);
    free(y_rhs);
    free(a_dec);
    free(w_env);
    free(c_tail);
    free(b_mat);
    return (info == 0) ? 0 : -1;
}

/* ================================================================
 * 4.  RUNNABLE ENTRY POINT
 * ============================================================= */

static MILK_HOT errno_t compdmpreshaper_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

    if (p_resp3d[0] == '\0' || p_inmodes[0] == '\0' || p_outkernel[0] == '\0')
    {
        fprintf(stderr, "compdmpreshaper: .resp3d, .inmodes, and .outkernel are required\n");
        return RETURN_FAILURE;
    }

    MILK_PROCINFO_INIT(self);

    double *r3_raw    = NULL;
    double *inm_raw   = NULL;
    double *stat_raw  = NULL;
    double *c_static  = NULL;
    double *p_pinv_d  = NULL;
    double *s_post    = NULL;
    double *s_work    = NULL;
    double *h_raw     = NULL;
    double *s_tar     = NULL;
    double *w_out     = NULL;
    double *w_act     = NULL;
    double *c_kern    = NULL;
    double *s_shaped  = NULL;
    float  *c_out_f32 = NULL;
    float  *pf_f32    = NULL;
    float  *pp_f32    = NULL;
    int    *mode_ax   = NULL;
    int    *mode_ay   = NULL;
    double *mode_amp  = NULL;
    int    *act2mode  = NULL;
    errno_t ret       = RETURN_FAILURE;

    MILK_PROCINFO_LOOPSTART(self);
    {

    /* 1. Load 3D RM [N_out, N_in, N_t] and input DM modes [dmx, dmy, N_in] */
    int         r3_naxis    = 0;
    long        r3_naxes[3] = { 1, 1, 1 };
    PS_RESP_HDR rhdr;
    if (ps_load_fits_f64(p_resp3d, &r3_raw, &r3_naxis, r3_naxes, &rhdr) != 0 || r3_naxis != 3)
    {
        fprintf(stderr, "compdmpreshaper: failed to load 3D resp3d '%s'\n", p_resp3d);
        goto cleanup;
    }
    const size_t n_out = (size_t) r3_naxes[0];
    const size_t n_in  = (size_t) r3_naxes[1];
    const size_t n_t   = (size_t) r3_naxes[2];

    int  inm_naxis    = 0;
    long inm_naxes[3] = { 1, 1, 1 };
    if (ps_load_fits_f64(p_inmodes, &inm_raw, &inm_naxis, inm_naxes, NULL) != 0)
    {
        fprintf(stderr, "compdmpreshaper: failed to load inmodes '%s'\n", p_inmodes);
        goto cleanup;
    }
    const size_t dmx    = (size_t) inm_naxes[0];
    const size_t dmy    = (inm_naxis >= 2) ? (size_t) inm_naxes[1] : 1;
    const size_t n_in_m = (inm_naxis >= 3) ? (size_t) inm_naxes[2] : 1;
    const size_t n_dm   = dmx * dmy;
    if (n_in_m != n_in)
    {
        fprintf(stderr, "compdmpreshaper: mode count mismatch resp3d=%zu vs inmodes=%zu\n", n_in,
                n_in_m);
        goto cleanup;
    }

    const size_t q0_bin  = (rhdr.has_q0 && rhdr.q0_bin >= 0 && (size_t) rhdr.q0_bin < n_t)
                               ? (size_t) rhdr.q0_bin
                               : 0;
    const size_t nbsubfr = (rhdr.has_nsub && rhdr.nbsubfr >= 1) ? (size_t) rhdr.nbsubfr : 1;
    const double dt_sec  = (p_dt_us > 0.0f) ? ((double) p_dt_us * 1.0e-6)
                                            : ((rhdr.has_dt && rhdr.dt_sec > 0.0) ? rhdr.dt_sec
                                                                                  : 2.5e-4);
    const size_t len     = n_t - q0_bin;
    if (len < 4)
    {
        fprintf(stderr, "compdmpreshaper: insufficient post-poke samples (L=%zu)\n", len);
        goto cleanup;
    }

    /* 2. Load static RM [N_out, N_in] (explicit, auto <resp3d>_static.fits, or settled tail) */
    int  st_naxis    = 0;
    long st_naxes[3] = { 1, 1, 1 };
    int  have_stat   = 0;
    if (p_staticrm[0] != '\0' && strcmp(p_staticrm, "none") != 0)
    {
        if (ps_load_fits_f64(p_staticrm, &stat_raw, &st_naxis, st_naxes, NULL) == 0 &&
            (size_t) st_naxes[0] == n_out && (size_t) st_naxes[1] == n_in)
        {
            have_stat = 1;
        }
    }
    if (!have_stat)
    {
        char auto_stat[STRINGMAXLEN_FULLFILENAME];
        const char *rp = (p_resp3d[0] == '!') ? (p_resp3d + 1) : p_resp3d;
        size_t      rl = strlen(rp);
        if (rl > 5 && strcmp(rp + rl - 5, ".fits") == 0 && rl + 8 < sizeof(auto_stat))
        {
            memcpy(auto_stat, rp, rl - 5);
            strcpy(auto_stat + rl - 5, "_static.fits");
            if (ps_load_fits_f64(auto_stat, &stat_raw, &st_naxis, st_naxes, NULL) == 0 &&
                (size_t) st_naxes[0] == n_out && (size_t) st_naxes[1] == n_in)
            {
                have_stat = 1;
            }
        }
    }
    if (!have_stat)
    {
        stat_raw = (double *) calloc(n_in * n_out, sizeof(double));
        if (stat_raw == NULL)
        {
            goto cleanup;
        }
        size_t n_tail = 2 * nbsubfr;
        if (n_tail > len / 2)
        {
            n_tail = (len / 2 > 0) ? (len / 2) : 1;
        }
        const double inv_nt = 1.0 / (double) n_tail;
        for (size_t t = n_t - n_tail; t < n_t; t++)
        {
            const double *sl = &r3_raw[t * n_in * n_out];
            for (size_t i = 0; i < n_in * n_out; i++)
            {
                stat_raw[i] += sl[i] * inv_nt;
            }
        }
    }

    /* 3. Compute static pseudo-inverse C_static [n_in x n_out] and dual P_pinv [n_in x n_dm] */
    c_static = (double *) malloc(sizeof(double) * n_in * n_out);
    p_pinv_d = (double *) malloc(sizeof(double) * n_in * n_dm);
    if (!c_static || !p_pinv_d)
    {
        goto cleanup;
    }

    const double svdlim = (p_svdlim > 0.0f) ? (double) p_svdlim : 1.0e-3;
    if (p_mode == 2u || p_mode == 3u)
    {
        /* Matched-filter WFS projection c_static[m] = stat_raw[m] / ||stat_raw[m]||^2:
         * avoids oblique static-facesheet overlap bias between adjacent zonal actuators
         * when solving 1D diagonal SISO pre-shapers (identical to pinv for orthogonal modes). */
        for (size_t m = 0; m < n_in; m++)
        {
            const double *row = &stat_raw[m * n_out];
            double       *dst = &c_static[m * n_out];
            double        n2  = 0.0;
            for (size_t p = 0; p < n_out; p++)
            {
                n2 += row[p] * row[p];
            }
            const double inv = (n2 > 1.0e-24) ? (1.0 / n2) : 0.0;
            for (size_t p = 0; p < n_out; p++)
            {
                dst[p] = row[p] * inv;
            }
        }
    }
    else
    {
        if (ps_svd_pinv_transpose(stat_raw, n_in, n_out, svdlim, c_static) != 0)
        {
            goto cleanup;
        }
    }
    if (p_mode <= 1u)
    {
        if (ps_svd_pinv_transpose(inm_raw, n_in, n_dm, svdlim, p_pinv_d) != 0)
        {
            goto cleanup;
        }
    }

    /* 4. Project post-poke slices: S_post[n, s, q] = sum_p c_static[s, p] * r3[q0+n, q, p] */
    s_post = (double *) calloc(len * n_in * n_in, sizeof(double));
    s_work = (double *) calloc(len * n_in * n_in, sizeof(double));
    h_raw  = (double *) calloc(len * n_in * n_in, sizeof(double));
    s_tar  = (double *) calloc(len, sizeof(double));
    w_out  = (double *) malloc(sizeof(double) * n_in);
    w_act  = (double *) malloc(sizeof(double) * n_in);
    if (!s_post || !s_work || !h_raw || !s_tar || !w_out || !w_act)
    {
        goto cleanup;
    }

    for (size_t n = 0; n < len; n++)
    {
        const double *r_sl = &r3_raw[(q0_bin + n) * n_in * n_out]; /* [n_in x n_out] */
        double       *s_sl = &s_post[n * n_in * n_in];             /* [n_in x n_in]  */
#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) n_in, (int) n_in, (int) n_out,
                    1.0, c_static, (int) n_out, r_sl, (int) n_out, 0.0, s_sl, (int) n_in);
#else
#    ifdef _OPENMP
#        pragma omp parallel for schedule(static)
#    endif
        for (size_t s = 0; s < n_in; s++)
        {
            for (size_t q = 0; q < n_in; q++)
            {
                double sum = 0.0;
                for (size_t p = 0; p < n_out; p++)
                {
                    sum += c_static[s * n_out + p] * r_sl[q * n_out + p];
                }
                s_sl[s * n_in + q] = sum;
            }
        }
#endif
    }

    /* 4b. Compute physical-space output (WFS static norm) & input (actuator L2 norm) weights */
    if (p_energy_weight != 0)
    {
        double mean_wo = 0.0;
        double mean_wa = 0.0;
        for (size_t m = 0; m < n_in; m++)
        {
            double s2 = 0.0;
            for (size_t p = 0; p < n_out; p++)
            {
                double v = stat_raw[m * n_out + p];
                s2      += v * v;
            }
            w_out[m] = sqrt(s2);
            mean_wo += w_out[m];

            double a2 = 0.0;
            for (size_t d = 0; d < n_dm; d++)
            {
                double v = inm_raw[m * n_dm + d];
                a2      += v * v;
            }
            w_act[m] = sqrt(a2);
            mean_wa += w_act[m];
        }
        mean_wo = (mean_wo > 1.0e-18) ? (mean_wo / (double) n_in) : 1.0;
        mean_wa = (mean_wa > 1.0e-18) ? (mean_wa / (double) n_in) : 1.0;
        for (size_t m = 0; m < n_in; m++)
        {
            w_out[m] /= mean_wo;
            w_act[m] /= mean_wa;
            if (w_out[m] < 1.0e-3)
            {
                w_out[m] = 1.0e-3;
            }
            if (w_act[m] < 1.0e-3)
            {
                w_act[m] = 1.0e-3;
            }
        }
    }
    else
    {
        for (size_t m = 0; m < n_in; m++)
        {
            w_out[m] = 1.0;
            w_act[m] = 1.0;
        }
    }

    /* 5. Optional two-sided finite-support Toeplitz exposure deconvolution (removes sinc nulls) */
    const double texp_sec = (p_texp_us < 0.0f) ? ((double) nbsubfr * dt_sec)
                                               : ((double) p_texp_us * 1.0e-6);
    const double tau_tar_sec =
        (p_tau_target_us > 0.0f) ? ((double) p_tau_target_us * 1.0e-6) : 0.0;
    const double lam_dec = (p_lambda_deconv > 1.0e-9f) ? (double) p_lambda_deconv : 1.0e-9;

    int    used_deconv = 0;
    size_t n_on_deconv = 0;
    size_t n_box_dec   = (texp_sec > 0.5 * dt_sec) ? (size_t) llround(texp_sec / dt_sec) : 1;
    if (p_deconv_exp != 0 && n_box_dec >= 2)
    {
        if (ps_deconv_twosided_toeplitz(s_post, len, n_in, n_box_dec, dt_sec, lam_dec,
                                        (double) p_deconv_tmax_ms, s_work, &n_on_deconv) == 0)
        {
            used_deconv = 1;
        }
    }
    if (!used_deconv)
    {
        memcpy(s_work, s_post, sizeof(double) * len * n_in * n_in);
    }

    for (size_t n = 0; n < len; n++)
    {
        const double *cur  = &s_work[n * n_in * n_in];
        const double *prev = (n > 0) ? &s_work[(n - 1) * n_in * n_in] : NULL;
        double       *dst  = &h_raw[n * n_in * n_in];
        for (size_t i = 0; i < n_in * n_in; i++)
        {
            dst[i] = cur[i] - ((prev != NULL) ? prev[i] : 0.0);
        }
    }

    /* 5b. Determine causal onset delay and build target step s_tar[0..len-1] */
    const double texp_tar_sec = used_deconv ? 0.0 : texp_sec;
    double       delay_sec    = (double) p_delay_us * 1.0e-6;
    if (p_delay_us < 0.0f)
    {
        if (used_deconv)
        {
            delay_sec = (double) n_on_deconv * dt_sec;
        }
        else
        {
            double *diag_mean = (double *) malloc(sizeof(double) * len);
            double *s_zero    = (double *) malloc(sizeof(double) * len);
            if (diag_mean != NULL && s_zero != NULL)
            {
                for (size_t n = 0; n < len; n++)
                {
                    double tr = 0.0;
                    for (size_t i = 0; i < n_in; i++)
                    {
                        tr += s_work[(n * n_in + i) * n_in + i];
                    }
                    diag_mean[n] = tr / (double) n_in;
                }
                size_t idx_on = 0;
                for (size_t n = 0; n < len; n++)
                {
                    if (diag_mean[n] >= 0.02)
                    {
                        idx_on = n;
                        break;
                    }
                }
                double d_floor = (idx_on > 0) ? (((double) idx_on - 0.5) * dt_sec) : 0.0;
                ps_build_target_step(s_zero, len, dt_sec, 0.0, tau_tar_sec, texp_tar_sec);
                double t50_raw = ps_crossing_time(diag_mean, len, dt_sec, 0.50);
                double t50_tar = ps_crossing_time(s_zero, len, dt_sec, 0.50);
                double d50     = (t50_raw > t50_tar) ? (t50_raw - t50_tar) : 0.0;
                delay_sec      = (d50 > d_floor) ? d50 : d_floor;
            }
            else
            {
                delay_sec = 0.0;
            }
            free(s_zero);
            free(diag_mean);
        }
    }

    ps_build_target_step(s_tar, len, dt_sec, delay_sec, tau_tar_sec, texp_tar_sec);

    /* 6. Solve constrained normal equations directly for look-ahead kernel C[0..m_buf-1] */
    size_t n_taps = p_ntaps;
    if (n_taps == 0)
    {
        n_taps = (len / 2 < 32) ? (len / 2) : 32;
    }
    if (n_taps < 3)
    {
        n_taps = 3;
    }
    if (n_taps >= len)
    {
        n_taps = len - 1;
    }
    const size_t   m_buf    = n_taps - 1;
    const uint32_t kmode    = (p_mode <= 4u) ? p_mode : 0u;
    const double   lam_ctrl = (p_lambda_ctrl > 1.0e-9f) ? (double) p_lambda_ctrl : 1.0e-9;
    const double   lam_diff = (p_lambda_diff >= 0.0f) ? (double) p_lambda_diff : 0.0;
    const uint32_t st_rad   = (kmode == 4u)
                                  ? ((p_stencil_rad >= 1u && p_stencil_rad <= 5u) ? p_stencil_rad
                                                                                  : 2u)
                                  : 0u;
    const uint32_t st_side  = (kmode == 4u) ? (2u * st_rad + 1u) : 0u;
    const uint32_t st_nb    = (kmode == 4u) ? (st_side * st_side) : 0u;

    /* For per-actuator modes (KMODE 3, 4), map each calibrated input mode to its peak (ax, ay) */
    if (kmode == 3u || kmode == 4u)
    {
        mode_ax  = (int *) calloc(n_in, sizeof(int));
        mode_ay  = (int *) calloc(n_in, sizeof(int));
        mode_amp = (double *) calloc(n_in, sizeof(double));
        act2mode = (int *) malloc(sizeof(int) * n_dm);
        if (!mode_ax || !mode_ay || !mode_amp || !act2mode)
        {
            goto cleanup;
        }
        for (size_t i = 0; i < n_dm; i++)
        {
            act2mode[i] = -1;
        }
        for (size_t m = 0; m < n_in; m++)
        {
            const double *mp   = &inm_raw[m * n_dm];
            size_t        best = 0;
            double        babs = fabs(mp[0]);
            for (size_t i = 1; i < n_dm; i++)
            {
                double a = fabs(mp[i]);
                if (a > babs)
                {
                    babs = a;
                    best = i;
                }
            }
            mode_ax[m]  = (int) (best % dmx);
            mode_ay[m]  = (int) (best / dmx);
            mode_amp[m] = (babs > 1.0e-12) ? mp[best] : 1.0;
            if (act2mode[best] >= 0)
            {
                fprintf(stderr,
                        "compdmpreshaper: warning: duplicate actuator (%d,%d) for modes %d & %zu\n",
                        mode_ax[m], mode_ay[m], act2mode[best], m);
            }
            act2mode[best] = (int) m;
        }
    }

    c_kern = (double *) calloc(m_buf * n_in * n_in, sizeof(double));
    if (c_kern == NULL)
    {
        goto cleanup;
    }

    if (kmode == 0u)
    {
        /* Full MIMO coupled spatiotemporal synthesis: A [n_sys x n_sys], B [n_sys x n_in] */
        const size_t n_sys = m_buf * n_in;
        double      *h_mat = (double *) calloc(len * n_in * n_sys, sizeof(double));
        double      *rhs   = (double *) calloc(len * n_in * n_in, sizeof(double));
        double      *a_sys = (double *) calloc(n_sys * n_sys, sizeof(double));
        double      *b_sys = (double *) calloc(n_sys * n_in, sizeof(double));
        if (!h_mat || !rhs || !a_sys || !b_sys)
        {
            free(b_sys);
            free(a_sys);
            free(rhs);
            free(h_mat);
            goto cleanup;
        }

        for (size_t n = 0; n < len; n++)
        {
            for (size_t k = 0; k < m_buf; k++)
            {
                if (n < k)
                {
                    continue;
                }
                const double *h_bl = &h_raw[(n - k) * n_in * n_in];
                for (size_t r = 0; r < n_in; r++)
                {
                    const double  wr  = w_out[r];
                    const double *src = &h_bl[r * n_in];
                    double       *dst = &h_mat[(n * n_in + r) * n_sys + k * n_in];
                    for (size_t c = 0; c < n_in; c++)
                    {
                        dst[c] = wr * src[c];
                    }
                }
            }
            const double *s_bl = &s_work[n * n_in * n_in];
            for (size_t r = 0; r < n_in; r++)
            {
                const double wr    = w_out[r];
                double      *r_row = &rhs[(n * n_in + r) * n_in];
                for (size_t c = 0; c < n_in; c++)
                {
                    double target = (r == c) ? s_tar[n] : 0.0;
                    r_row[c]      = wr * (target - s_bl[r * n_in + c]);
                }
            }
        }

        const size_t n_rows = len * n_in;
#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
        cblas_dsyrk(CblasRowMajor, CblasUpper, CblasTrans, (int) n_sys, (int) n_rows, 1.0, h_mat,
                    (int) n_sys, 0.0, a_sys, (int) n_sys);
        cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, (int) n_sys, (int) n_in, (int) n_rows,
                    1.0, h_mat, (int) n_sys, rhs, (int) n_in, 0.0, b_sys, (int) n_in);
#else
#    ifdef _OPENMP
#        pragma omp parallel for schedule(static)
#    endif
        for (size_t i = 0; i < n_sys; i++)
        {
            for (size_t j = i; j < n_sys; j++)
            {
                double sum = 0.0;
                for (size_t r = 0; r < n_rows; r++)
                {
                    sum += h_mat[r * n_sys + i] * h_mat[r * n_sys + j];
                }
                a_sys[i * n_sys + j] = sum;
            }
            for (size_t c = 0; c < n_in; c++)
            {
                double sum = 0.0;
                for (size_t r = 0; r < n_rows; r++)
                {
                    sum += h_mat[r * n_sys + i] * rhs[r * n_in + c];
                }
                b_sys[i * n_in + c] = sum;
            }
        }
#endif
        /* Symmetrize and add physical-energy-weighted Tikhonov + smoothness (C[-1]=0, C[m_buf]=0) */
        for (size_t i = 0; i < n_sys; i++)
        {
            for (size_t j = i + 1; j < n_sys; j++)
            {
                a_sys[j * n_sys + i] = a_sys[i * n_sys + j];
            }
            const size_t m   = i % n_in;
            const double wa2 = w_act[m] * w_act[m];
            a_sys[i * n_sys + i] += (lam_ctrl + 2.0 * lam_diff) * wa2;
        }
        if (lam_diff > 0.0)
        {
            for (size_t k = 0; k + 1 < m_buf; k++)
            {
                for (size_t m = 0; m < n_in; m++)
                {
                    const double wa2        = w_act[m] * w_act[m];
                    size_t       i1         = k * n_in + m;
                    size_t       i2         = (k + 1) * n_in + m;
                    a_sys[i1 * n_sys + i2] -= lam_diff * wa2;
                    a_sys[i2 * n_sys + i1] -= lam_diff * wa2;
                }
            }
        }

        lapack_int info = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'U', (lapack_int) n_sys, a_sys,
                                         (lapack_int) n_sys);
        if (info == 0)
        {
            info = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', (lapack_int) n_sys, (lapack_int) n_in,
                                  a_sys, (lapack_int) n_sys, b_sys, (lapack_int) n_in);
        }
        if (info != 0)
        {
            fprintf(stderr, "compdmpreshaper: Cholesky solve failed (info=%d)\n", (int) info);
            free(b_sys);
            free(a_sys);
            free(rhs);
            free(h_mat);
            goto cleanup;
        }

        memcpy(c_kern, b_sys, sizeof(double) * n_sys * n_in);
        free(b_sys);
        free(a_sys);
        free(rhs);
        free(h_mat);
    }
    else if (kmode == 4u)
    {
        /* Per-actuator spatially-varying local-stencil MIMO:
         * For each calibrated input actuator m at (ax_m, ay_m), solve an independent
         * local (2R+1)x(2R+1) MIMO Cholesky system driving only calibrated actuators
         * q in N(m) to match target s_tar on m and suppress cross-actuator waves across
         * the observation patch O(m). */
        const int    rad_i  = (int) st_rad;
        const int    obs_r  = rad_i + 3;
        size_t      *loc_q  = (size_t *) malloc(sizeof(size_t) * st_nb);
        size_t      *obs_id = (size_t *) malloc(sizeof(size_t) * n_in);
        const size_t max_sys = m_buf * st_nb;
        const size_t max_obs = (n_in <= 256u) ? n_in : (size_t) ((2 * obs_r + 1) * (2 * obs_r + 1));
        const size_t max_rows = len * ((max_obs < n_in) ? max_obs : n_in);
        double      *h_loc  = (double *) malloc(sizeof(double) * max_rows * max_sys);
        double      *r_loc  = (double *) malloc(sizeof(double) * max_rows);
        double      *a_loc  = (double *) malloc(sizeof(double) * max_sys * max_sys);
        double      *b_loc  = (double *) malloc(sizeof(double) * max_sys);
        if (!loc_q || !obs_id || !h_loc || !r_loc || !a_loc || !b_loc)
        {
            free(b_loc);
            free(a_loc);
            free(r_loc);
            free(h_loc);
            free(obs_id);
            free(loc_q);
            goto cleanup;
        }

        int solve_ok = 1;
        for (size_t m = 0; m < n_in; m++)
        {
            const int ax_m = mode_ax[m];
            const int ay_m = mode_ay[m];

            /* Gather calibrated control neighbors q in (2R+1)x(2R+1) stencil around (ax_m, ay_m) */
            size_t n_loc = 0;
            for (int dy = -rad_i; dy <= rad_i; dy++)
            {
                int qy = ay_m + dy;
                if (qy < 0 || qy >= (int) dmy)
                {
                    continue;
                }
                for (int dx = -rad_i; dx <= rad_i; dx++)
                {
                    int qx = ax_m + dx;
                    if (qx < 0 || qx >= (int) dmx)
                    {
                        continue;
                    }
                    int qm = act2mode[(size_t) qy * dmx + (size_t) qx];
                    if (qm >= 0)
                    {
                        loc_q[n_loc++] = (size_t) qm;
                    }
                }
            }
            if (n_loc == 0)
            {
                loc_q[n_loc++] = m;
            }

            /* Gather observation actuators r in O(m) (all calibrated actuators when n_in <= 256) */
            size_t n_obs = 0;
            if (n_in <= 256u)
            {
                for (size_t r = 0; r < n_in; r++)
                {
                    obs_id[n_obs++] = r;
                }
            }
            else
            {
                for (int dy = -obs_r; dy <= obs_r; dy++)
                {
                    int ry = ay_m + dy;
                    if (ry < 0 || ry >= (int) dmy)
                    {
                        continue;
                    }
                    for (int dx = -obs_r; dx <= obs_r; dx++)
                    {
                        int rx = ax_m + dx;
                        if (rx < 0 || rx >= (int) dmx)
                        {
                            continue;
                        }
                        int rm = act2mode[(size_t) ry * dmx + (size_t) rx];
                        if (rm >= 0)
                        {
                            obs_id[n_obs++] = (size_t) rm;
                        }
                    }
                }
            }

            const size_t n_sys_loc = m_buf * n_loc;
            const size_t n_rows    = len * n_obs;
            memset(h_loc, 0, sizeof(double) * n_rows * n_sys_loc);

            for (size_t n = 0; n < len; n++)
            {
                for (size_t ir = 0; ir < n_obs; ir++)
                {
                    const size_t r   = obs_id[ir];
                    const double wr  = w_out[r];
                    const size_t row = n * n_obs + ir;
                    const double tar = (r == m) ? s_tar[n] : 0.0;
                    r_loc[row]       = wr * (tar - s_work[(n * n_in + r) * n_in + m]);

                    const size_t k_lim = (n + 1 < m_buf) ? (n + 1) : m_buf;
                    for (size_t k = 0; k < k_lim; k++)
                    {
                        const double *h_row = &h_raw[((n - k) * n_in + r) * n_in];
                        double       *dst   = &h_loc[row * n_sys_loc + k * n_loc];
                        for (size_t iq = 0; iq < n_loc; iq++)
                        {
                            dst[iq] = wr * h_row[loc_q[iq]];
                        }
                    }
                }
            }

#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
            cblas_dsyrk(CblasRowMajor, CblasUpper, CblasTrans, (int) n_sys_loc, (int) n_rows, 1.0,
                        h_loc, (int) n_sys_loc, 0.0, a_loc, (int) n_sys_loc);
            cblas_dgemv(CblasRowMajor, CblasTrans, (int) n_rows, (int) n_sys_loc, 1.0, h_loc,
                        (int) n_sys_loc, r_loc, 1, 0.0, b_loc, 1);
#else
            for (size_t i = 0; i < n_sys_loc; i++)
            {
                for (size_t j = i; j < n_sys_loc; j++)
                {
                    double sum = 0.0;
                    for (size_t row = 0; row < n_rows; row++)
                    {
                        sum += h_loc[row * n_sys_loc + i] * h_loc[row * n_sys_loc + j];
                    }
                    a_loc[i * n_sys_loc + j] = sum;
                }
                double bsum = 0.0;
                for (size_t row = 0; row < n_rows; row++)
                {
                    bsum += h_loc[row * n_sys_loc + i] * r_loc[row];
                }
                b_loc[i] = bsum;
            }
#endif
            for (size_t i = 0; i < n_sys_loc; i++)
            {
                for (size_t j = i + 1; j < n_sys_loc; j++)
                {
                    a_loc[j * n_sys_loc + i] = a_loc[i * n_sys_loc + j];
                }
                const size_t q   = loc_q[i % n_loc];
                const double wa2 = w_act[q] * w_act[q];
                a_loc[i * n_sys_loc + i] += (lam_ctrl + 2.0 * lam_diff) * wa2;
            }
            if (lam_diff > 0.0)
            {
                for (size_t k = 0; k + 1 < m_buf; k++)
                {
                    for (size_t iq = 0; iq < n_loc; iq++)
                    {
                        const size_t q   = loc_q[iq];
                        const double wa2 = w_act[q] * w_act[q];
                        const size_t i1  = k * n_loc + iq;
                        const size_t i2  = (k + 1) * n_loc + iq;
                        a_loc[i1 * n_sys_loc + i2] -= lam_diff * wa2;
                        a_loc[i2 * n_sys_loc + i1] -= lam_diff * wa2;
                    }
                }
            }

            lapack_int info = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'U', (lapack_int) n_sys_loc, a_loc,
                                             (lapack_int) n_sys_loc);
            if (info == 0)
            {
                info = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', (lapack_int) n_sys_loc, 1, a_loc,
                                      (lapack_int) n_sys_loc, b_loc, 1);
            }
            if (info != 0)
            {
                fprintf(stderr,
                        "compdmpreshaper: KMODE=4 local Cholesky failed (act m=%zu, info=%d)\n", m,
                        (int) info);
                solve_ok = 0;
                break;
            }

            for (size_t k = 0; k < m_buf; k++)
            {
                for (size_t iq = 0; iq < n_loc; iq++)
                {
                    const size_t q = loc_q[iq];
                    c_kern[(k * n_in + q) * n_in + m] = b_loc[k * n_loc + iq];
                }
            }
        }

        free(b_loc);
        free(a_loc);
        free(r_loc);
        free(h_loc);
        free(obs_id);
        free(loc_q);
        if (!solve_ok)
        {
            goto cleanup;
        }
    }
    else
    {
        /* Mode 1 (per-mode decoupled SISO), Mode 2 (uniform SISO), or Mode 3 (per-actuator SISO) */
        const size_t n_solves = (kmode == 2u) ? 1u : n_in;
        double      *a_siso   = (double *) malloc(sizeof(double) * m_buf * m_buf);
        double      *b_siso   = (double *) malloc(sizeof(double) * m_buf);
        double      *s_1d     = (double *) malloc(sizeof(double) * len);
        double      *h_1d     = (double *) malloc(sizeof(double) * len);
        if (!a_siso || !b_siso || !s_1d || !h_1d)
        {
            free(h_1d);
            free(s_1d);
            free(b_siso);
            free(a_siso);
            goto cleanup;
        }

        for (size_t m = 0; m < n_solves; m++)
        {
            for (size_t n = 0; n < len; n++)
            {
                if (kmode == 1u || kmode == 3u)
                {
                    s_1d[n] = s_work[(n * n_in + m) * n_in + m];
                }
                else
                {
                    double tr = 0.0;
                    for (size_t i = 0; i < n_in; i++)
                    {
                        tr += s_work[(n * n_in + i) * n_in + i];
                    }
                    s_1d[n] = tr / (double) n_in;
                }
                h_1d[n] = s_1d[n] - ((n > 0) ? s_1d[n - 1] : 0.0);
            }

            memset(a_siso, 0, sizeof(double) * m_buf * m_buf);
            memset(b_siso, 0, sizeof(double) * m_buf);
            for (size_t i = 0; i < m_buf; i++)
            {
                for (size_t j = i; j < m_buf; j++)
                {
                    double sum = 0.0;
                    for (size_t n = j; n < len; n++)
                    {
                        sum += h_1d[n - i] * h_1d[n - j];
                    }
                    a_siso[i * m_buf + j] = sum;
                    a_siso[j * m_buf + i] = sum;
                }
                a_siso[i * m_buf + i] += lam_ctrl + 2.0 * lam_diff;
                if (i + 1 < m_buf)
                {
                    a_siso[i * m_buf + (i + 1)] -= lam_diff;
                    a_siso[(i + 1) * m_buf + i] -= lam_diff;
                }
                double bsum = 0.0;
                for (size_t n = i; n < len; n++)
                {
                    bsum += h_1d[n - i] * (s_tar[n] - s_1d[n]);
                }
                b_siso[i] = bsum;
            }

            lapack_int info = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'U', (lapack_int) m_buf, a_siso,
                                             (lapack_int) m_buf);
            if (info == 0)
            {
                info = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', (lapack_int) m_buf, 1, a_siso,
                                      (lapack_int) m_buf, b_siso, 1);
            }
            if (info != 0)
            {
                fprintf(stderr, "compdmpreshaper: SISO Cholesky failed (m=%zu, info=%d)\n", m,
                        (int) info);
                free(h_1d);
                free(s_1d);
                free(b_siso);
                free(a_siso);
                goto cleanup;
            }

            if (kmode == 1u || kmode == 3u)
            {
                for (size_t k = 0; k < m_buf; k++)
                {
                    c_kern[(k * n_in + m) * n_in + m] = b_siso[k];
                }
            }
            else
            {
                for (size_t k = 0; k < m_buf; k++)
                {
                    for (size_t i = 0; i < n_in; i++)
                    {
                        c_kern[(k * n_in + i) * n_in + i] = b_siso[k];
                    }
                }
            }
        }
        free(h_1d);
        free(s_1d);
        free(b_siso);
        free(a_siso);
    }

    /* 7. Apply optional safety cap on max|I + C[k]| while strictly preserving unity DC gain */
    double max_step_gain = 1.0;
    for (size_t k = 0; k < m_buf; k++)
    {
        for (size_t r = 0; r < n_in; r++)
        {
            for (size_t c = 0; c < n_in; c++)
            {
                double val = ((r == c) ? 1.0 : 0.0) + c_kern[(k * n_in + r) * n_in + c];
                if (fabs(val) > max_step_gain)
                {
                    max_step_gain = fabs(val);
                }
            }
        }
    }
    if (p_max_gain > 1.01f && max_step_gain > (double) p_max_gain)
    {
        double scale = ((double) p_max_gain - 1.0) / (max_step_gain - 1.0);
        for (size_t i = 0; i < m_buf * n_in * n_in; i++)
        {
            c_kern[i] *= scale;
        }
        max_step_gain = (double) p_max_gain;
    }

    /* 8. Compute predicted pre-shaped response S_shaped = S_work + H_raw * C and diagnostics */
    s_shaped = (double *) malloc(sizeof(double) * len * n_in * n_in);
    if (s_shaped == NULL)
    {
        goto cleanup;
    }
    memcpy(s_shaped, s_work, sizeof(double) * len * n_in * n_in);

    for (size_t n = 0; n < len; n++)
    {
        double *dst   = &s_shaped[n * n_in * n_in];
        size_t  k_max = (n + 1 < m_buf) ? (n + 1) : m_buf;
        for (size_t k = 0; k < k_max; k++)
        {
            const double *h_bl = &h_raw[(n - k) * n_in * n_in];
            const double *c_bl = &c_kern[k * n_in * n_in];
#if defined(HAVE_MKL) || defined(HAVE_OPENBLAS)
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) n_in, (int) n_in,
                        (int) n_in, 1.0, h_bl, (int) n_in, c_bl, (int) n_in, 1.0, dst, (int) n_in);
#else
            for (size_t r = 0; r < n_in; r++)
            {
                for (size_t c = 0; c < n_in; c++)
                {
                    double sum = 0.0;
                    for (size_t q = 0; q < n_in; q++)
                    {
                        sum += h_bl[r * n_in + q] * c_bl[q * n_in + c];
                    }
                    dst[r * n_in + c] += sum;
                }
            }
#endif
        }
    }

    double err_pre_sq  = 0.0;
    double err_post_sq = 0.0;
    double off_pre_pk  = 0.0;
    double off_post_pk = 0.0;
    for (size_t n = 0; n < len; n++)
    {
        double off_b = 0.0;
        double off_a = 0.0;
        for (size_t r = 0; r < n_in; r++)
        {
            for (size_t c = 0; c < n_in; c++)
            {
                double tar = (r == c) ? s_tar[n] : 0.0;
                double vb  = s_work[(n * n_in + r) * n_in + c];
                double va  = s_shaped[(n * n_in + r) * n_in + c];
                err_pre_sq  += (vb - tar) * (vb - tar);
                err_post_sq += (va - tar) * (va - tar);
                if (r != c)
                {
                    off_b += vb * vb;
                    off_a += va * va;
                }
            }
        }
        if (sqrt(off_b) > off_pre_pk)
        {
            off_pre_pk = sqrt(off_b);
        }
        if (sqrt(off_a) > off_post_pk)
        {
            off_post_pk = sqrt(off_a);
        }
    }

    double ov_pre_sum  = 0.0;
    double ov_post_sum = 0.0;
    for (size_t i = 0; i < n_in; i++)
    {
        double pk_b = 0.0;
        double pk_a = 0.0;
        for (size_t n = 0; n < len; n++)
        {
            double vb = s_work[(n * n_in + i) * n_in + i];
            double va = s_shaped[(n * n_in + i) * n_in + i];
            if (vb > pk_b)
            {
                pk_b = vb;
            }
            if (va > pk_a)
            {
                pk_a = va;
            }
        }
        ov_pre_sum  += (pk_b - 1.0) * 100.0;
        ov_post_sum += (pk_a - 1.0) * 100.0;
    }

    const double err_pre   = sqrt(err_pre_sq);
    const double err_post  = sqrt(err_post_sq);
    const double err_red_x = err_pre / ((err_post > 1.0e-12) ? err_post : 1.0e-12);
    const double ov_pre    = ov_pre_sum / (double) n_in;
    const double ov_post   = ov_post_sum / (double) n_in;
    const double off_red_x = (off_pre_pk > 1.0e-9)
                                 ? (off_pre_pk / ((off_post_pk > 1.0e-12) ? off_post_pk : 1.0e-12))
                                 : 1.0;

    /* 9. Write self-contained MEF kernel FITS file */
    PS_OUT_META meta;
    meta.kmode         = kmode;
    meta.dt_sec        = dt_sec;
    meta.dmx           = (uint32_t) dmx;
    meta.dmy           = (uint32_t) dmy;
    meta.n_in          = (uint32_t) n_in;
    meta.m_buf         = (uint32_t) m_buf;
    meta.stencil_rad   = st_rad;
    meta.stencil_nb    = st_nb;
    meta.tau_tar_us    = tau_tar_sec * 1.0e6;
    meta.delay_us      = delay_sec * 1.0e6;
    meta.texp_us       = texp_sec * 1.0e6;
    meta.lam_ctrl      = lam_ctrl;
    meta.lam_diff      = lam_diff;
    meta.lam_deconv    = lam_dec;
    meta.deconv_exp    = used_deconv;
    meta.energy_weight = (p_energy_weight != 0) ? 1 : 0;
    meta.max_gain      = max_step_gain;
    meta.err_before    = err_pre;
    meta.err_after     = err_post;
    meta.ov_before_pct = ov_pre;
    meta.ov_after_pct  = ov_post;
    meta.off_red_x     = off_red_x;

    long p_naxes[3] = { (long) dmx, (long) dmy, (long) n_in };
    if (kmode == 2u)
    {
        c_out_f32 = (float *) malloc(sizeof(float) * m_buf);
        if (!c_out_f32)
        {
            goto cleanup;
        }
        for (size_t k = 0; k < m_buf; k++)
        {
            c_out_f32[k] = (float) c_kern[(k * n_in + 0) * n_in + 0];
        }
        long c_naxes[3] = { 1, 1, (long) m_buf };
        if (ps_write_mef_kernel(p_outkernel, c_out_f32, c_naxes, NULL, NULL, p_naxes, &meta) != 0)
        {
            goto cleanup;
        }
    }
    else if (kmode == 3u)
    {
        /* Per-actuator SISO: Primary HDU [dmx, dmy, M_buf], no P_FWD/P_PINV */
        c_out_f32 = (float *) calloc(m_buf * n_dm, sizeof(float));
        if (!c_out_f32)
        {
            goto cleanup;
        }
        for (size_t m = 0; m < n_in; m++)
        {
            const size_t i_dm = (size_t) mode_ay[m] * dmx + (size_t) mode_ax[m];
            for (size_t k = 0; k < m_buf; k++)
            {
                c_out_f32[k * n_dm + i_dm] = (float) c_kern[(k * n_in + m) * n_in + m];
            }
        }
        if (p_fill_uncal != 0 && n_in > 0)
        {
            for (size_t iy = 0; iy < dmy; iy++)
            {
                for (size_t ix = 0; ix < dmx; ix++)
                {
                    const size_t i_dm = iy * dmx + ix;
                    if (act2mode[i_dm] >= 0)
                    {
                        continue;
                    }
                    double wsum = 0.0;
                    for (size_t m = 0; m < n_in; m++)
                    {
                        double dx = (double) ix - (double) mode_ax[m];
                        double dy = (double) iy - (double) mode_ay[m];
                        double w  = 1.0 / (dx * dx + dy * dy + 0.5);
                        wsum     += w;
                        for (size_t k = 0; k < m_buf; k++)
                        {
                            c_out_f32[k * n_dm + i_dm] +=
                                (float) (w * c_kern[(k * n_in + m) * n_in + m]);
                        }
                    }
                    if (wsum > 1.0e-12)
                    {
                        const float inv_w = (float) (1.0 / wsum);
                        for (size_t k = 0; k < m_buf; k++)
                        {
                            c_out_f32[k * n_dm + i_dm] *= inv_w;
                        }
                    }
                }
            }
        }
        long c_naxes[3] = { (long) dmx, (long) dmy, (long) m_buf };
        if (ps_write_mef_kernel(p_outkernel, c_out_f32, c_naxes, NULL, NULL, p_naxes, &meta) != 0)
        {
            goto cleanup;
        }
    }
    else if (kmode == 4u)
    {
        /* Per-actuator spatially-varying local-stencil MIMO (gather layout):
         * C_gather[k, i_out, (dy + R)*S + (dx + R)] = (a_q / a_m) * c_kern[k, q=i_out, m=j_in]
         * where dx = ax_in - ax_out, dy = ay_in - ay_out in [-R, +R].
         * Primary HDU [N_nb, dmx*dmy, M_buf], no P_FWD/P_PINV. */
        const int rad_i = (int) st_rad;
        const int side  = (int) st_side;
        c_out_f32       = (float *) calloc(m_buf * n_dm * st_nb, sizeof(float));
        if (!c_out_f32)
        {
            goto cleanup;
        }
        for (size_t m = 0; m < n_in; m++)
        {
            const int    jx  = mode_ax[m];
            const int    jy  = mode_ay[m];
            const double am  = (fabs(mode_amp[m]) > 1.0e-12) ? mode_amp[m] : 1.0;
            for (int oy = -rad_i; oy <= rad_i; oy++)
            {
                const int iy = jy + oy;
                if (iy < 0 || iy >= (int) dmy)
                {
                    continue;
                }
                for (int ox = -rad_i; ox <= rad_i; ox++)
                {
                    const int ix = jx + ox;
                    if (ix < 0 || ix >= (int) dmx)
                    {
                        continue;
                    }
                    const size_t i_dm = (size_t) iy * dmx + (size_t) ix;
                    const int    qm   = act2mode[i_dm];
                    if (qm < 0)
                    {
                        continue;
                    }
                    const int    dx    = jx - ix; /* = -ox in [-R, +R] */
                    const int    dy    = jy - iy; /* = -oy in [-R, +R] */
                    const size_t p_idx = (size_t) ((dy + rad_i) * side + (dx + rad_i));
                    const double scale = mode_amp[qm] / am;
                    for (size_t k = 0; k < m_buf; k++)
                    {
                        c_out_f32[(k * n_dm + i_dm) * st_nb + p_idx] =
                            (float) (scale * c_kern[(k * n_in + (size_t) qm) * n_in + m]);
                    }
                }
            }
        }
        if (p_fill_uncal != 0 && n_in > 0)
        {
            /* Fill uncalibrated actuators by copying the stencil of their nearest calibrated neighbor */
            for (int iy = 0; iy < (int) dmy; iy++)
            {
                for (int ix = 0; ix < (int) dmx; ix++)
                {
                    const size_t i_dm = (size_t) iy * dmx + (size_t) ix;
                    if (act2mode[i_dm] >= 0)
                    {
                        continue;
                    }
                    size_t best_m = 0;
                    double best_d = 1.0e30;
                    for (size_t m = 0; m < n_in; m++)
                    {
                        double ddx = (double) ix - (double) mode_ax[m];
                        double ddy = (double) iy - (double) mode_ay[m];
                        double d2  = ddx * ddx + ddy * ddy;
                        if (d2 < best_d)
                        {
                            best_d = d2;
                            best_m = m;
                        }
                    }
                    const size_t src_dm =
                        (size_t) mode_ay[best_m] * dmx + (size_t) mode_ax[best_m];
                    for (size_t k = 0; k < m_buf; k++)
                    {
                        const float *src = &c_out_f32[(k * n_dm + src_dm) * st_nb];
                        float       *dst = &c_out_f32[(k * n_dm + i_dm) * st_nb];
                        for (int dy = -rad_i; dy <= rad_i; dy++)
                        {
                            int jy = iy + dy;
                            for (int dx = -rad_i; dx <= rad_i; dx++)
                            {
                                int    jx = ix + dx;
                                size_t p  = (size_t) ((dy + rad_i) * side + (dx + rad_i));
                                if (jx >= 0 && jx < (int) dmx && jy >= 0 && jy < (int) dmy)
                                {
                                    dst[p] = src[p];
                                }
                            }
                        }
                    }
                }
            }
        }
        long c_naxes[3] = { (long) st_nb, (long) n_dm, (long) m_buf };
        if (ps_write_mef_kernel(p_outkernel, c_out_f32, c_naxes, NULL, NULL, p_naxes, &meta) != 0)
        {
            goto cleanup;
        }
    }
    else
    {
        c_out_f32 = (float *) malloc(sizeof(float) * m_buf * n_in * n_in);
        pf_f32    = (float *) malloc(sizeof(float) * n_in * n_dm);
        pp_f32    = (float *) malloc(sizeof(float) * n_in * n_dm);
        if (!c_out_f32 || !pf_f32 || !pp_f32)
        {
            goto cleanup;
        }
        for (size_t i = 0; i < m_buf * n_in * n_in; i++)
        {
            c_out_f32[i] = (float) c_kern[i];
        }
        for (size_t i = 0; i < n_in * n_dm; i++)
        {
            pf_f32[i] = (float) inm_raw[i];
            pp_f32[i] = (float) p_pinv_d[i];
        }
        long c_naxes[3] = { (long) n_in, (long) n_in, (long) m_buf };
        if (ps_write_mef_kernel(p_outkernel, c_out_f32, c_naxes, pf_f32, pp_f32, p_naxes, &meta) !=
            0)
        {
            goto cleanup;
        }
    }

    if (p_outpred[0] != '\0' && strcmp(p_outpred, "none") != 0)
    {
        float *pred_f32 = (float *) malloc(sizeof(float) * len * n_in * n_in);
        if (pred_f32 != NULL)
        {
            for (size_t i = 0; i < len * n_in * n_in; i++)
            {
                pred_f32[i] = (float) s_shaped[i];
            }
            long pred_ax[3] = { (long) n_in, (long) n_in, (long) len };
            (void) ps_write_simple_fits_f32(p_outpred, pred_f32, pred_ax, dt_sec);
            free(pred_f32);
        }
    }

    o_err_before      = (float) err_pre;
    o_err_after       = (float) err_post;
    o_err_reduction_x = (float) err_red_x;
    o_ov_before_pct   = (float) ov_pre;
    o_ov_after_pct    = (float) ov_post;
    o_off_reduction_x = (float) off_red_x;
    o_max_step_gain   = (float) max_step_gain;

    if (dcfpsptr != NULL)
    {
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    if (processinfo != NULL)
    {
        processinfo_WriteMessage_fmt(processinfo,
                                     "KMODE=%u M_buf=%zu dt=%.1fus err=%.2fx ov=%.1f->%.1f%% "
                                     "off=%.2fx gain=%.2fx",
                                     kmode, m_buf, dt_sec * 1.0e6, err_red_x, ov_pre, ov_post,
                                     off_red_x, max_step_gain);
    }

    ret = RETURN_SUCCESS;

cleanup:
    free(act2mode);
    free(mode_amp);
    free(mode_ay);
    free(mode_ax);
    free(pp_f32);
    free(pf_f32);
    free(c_out_f32);
    free(s_shaped);
    free(c_kern);
    free(w_act);
    free(w_out);
    free(s_tar);
    free(h_raw);
    free(s_work);
    free(s_post);
    free(p_pinv_d);
    free(c_static);
    free(stat_raw);
    free(inm_raw);
    free(r3_raw);
    break;
    }
    MILK_PROCINFO_END(self);
    DEBUG_TRACE_FEXIT();
    return ret;
}

const MILK_RUNNABLE runnable_computeCalib_compute_dm_preshaper = {
    .kind = MILK_RUNNABLE_FPS,
    .app = {
        .fps_name         = "cacaocc-compdmpreshaper",
        .cmdkey           = "compdmpreshaper",
        .description      = "Synthesize high-rate look-ahead DM pre-shaper kernel from 3D RM",
        .description_long = "Computes a causal, unity-DC-gain look-ahead DM command pre-shaper "
                            "kernel from a raw 3D response matrix (cacaoac-measdynresp) using "
                            "regularized time-domain model matching without WFS sinc-null "
                            "inversion instability.",
    },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaocc_compdmpreshaper),
    .compute = compdmpreshaper_compute,
};

EXPORT_FPSEXEC(cacao, "cacaocc-compdmpreshaper", runnable_computeCalib_compute_dm_preshaper);
