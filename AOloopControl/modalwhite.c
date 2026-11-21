// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalwhite.c
 * @brief   Controller-agnostic closed-loop residual whiteness & predictability monitor
 *
 * Subscribes asynchronously to double-buffered modal WFS telemetry (aol<N>_modevalWFS_buff)
 * and optional DM telemetry (aol<N>_modevalDM_buff), accumulating a sliding window of NBwin
 * frames and evaluating:
 *   1. Exact MA(d-1) Bartlett-corrected Ljung-Box whiteness Q_tilde_m for lags k >= ceil(tau)
 *   2. Exact Isserlis/Bartlett cross-mode whiteness Q_xmode on Block 0 and cross-block B0xB1
 *   3. Chronological 2-fold split-half out-of-sample recoverable WFE (Delta WFE_recov, PRF_m)
 *   4. WFS-vs-DM lag >= d cross-correlation r_{wu}(d), Q_{wu}, and W|U recoverable WFE
 *   5. Signed rho(d) ringing / lag / ZPO-ramp diagnostic alert bitmask
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
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl.h"

#define MWHITE_MAX_LAG   64
#define MWHITE_MAX_ORDER 16
#define MWHITE_MAX_BLOCK 256

static uint64_t p_loopindex                            = 0;
static char     p_inwfs[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalWFS_buff";
static char     p_indm[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_modevalDM_buff";
static float    p_latency_fr                           = 2.5f;
static uint32_t p_nbwin                                = 2048;
static uint32_t p_klags                                = 16;
static uint32_t p_precov                               = 8;
static uint32_t p_pctrl                                = 8;
static uint32_t p_blocksize                            = 128;
static float    p_reglambda                            = 0.01f;

static uint32_t o_n_samples     = 0;
static uint32_t o_d_ceil        = 3;
static float    o_wfs_rms_nm    = 0.0f;
static float    o_recov_wfe_nm  = 0.0f;
static float    o_recov_var_pct = 0.0f;
static float    o_b0_recov_nm   = 0.0f;
static float    o_Q_vw_all      = 0.0f;
static float    o_Q_vw_b0       = 0.0f;
static float    o_Q_lo20        = 0.0f;
static float    o_Q_beyond_b0   = 0.0f;
static float    o_Q_xmode_b0    = 0.0f;
static float    o_Q_xblk_b01    = 0.0f;
static float    o_rho_d_b0      = 0.0f;
static float    o_cliff_b0      = 0.0f;
static float    o_rwu_d_lo20    = 0.0f;
static float    o_Q_wu_lo20     = 0.0f;
static float    o_wu_recov_nm   = 0.0f;
static uint32_t o_alert_flags   = 0;

#define FPS_PARAMS_MODALWHITE(X)                                                                  \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".inWFSbuff", p_inwfs, FPTYPE_STREAMNAME, 1,                                                \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input WFS modal buffer")         \
    X(".inDMbuff", p_indm, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,         \
      "optional input DM modal buffer")                                                           \
    X(".latency_fr", &p_latency_fr, FPTYPE_FLOAT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop latency [frames]")         \
    X(".NBwin", &p_nbwin, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,              \
      "sliding analysis window [frames]")                                                         \
    X(".K_lags", &p_klags, FPTYPE_UINT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "Bartlett lag count K")          \
    X(".P_recov", &p_precov, FPTYPE_UINT32, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "OOS AR order (1..16)")          \
    X(".P_ctrl", &p_pctrl, FPTYPE_UINT32, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "controller lookback P")         \
    X(".blocksize", &p_blocksize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "Block-0 mode count for Q_xmode")                                                           \
    X(".reglambda", &p_reglambda, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "OOS ridge relative lambda")     \
    X(".out.n_samples", &o_n_samples, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "active frames in sliding window")                                                          \
    X(".out.d_ceil", &o_d_ceil, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "effective lag cutoff ceil(latency_fr)")                                                    \
    X(".out.wfs_rms_nm", &o_wfs_rms_nm, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "quadrature modal WFS RMS [nm]")                                                            \
    X(".out.recov_wfe_nm", &o_recov_wfe_nm, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "OOS lag>=d recoverable WFE [nm]")                                                          \
    X(".out.recov_var_pct", &o_recov_var_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "OOS lag>=d recoverable variance [%]")                                                      \
    X(".out.b0_recov_wfe_nm", &o_b0_recov_nm, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "Block-0 OOS recoverable WFE [nm]")                                                         \
    X(".out.Q_vw_all", &o_Q_vw_all, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "variance-weighted Bartlett Q (all)")                                                       \
    X(".out.Q_vw_b0", &o_Q_vw_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "variance-weighted Bartlett Q (Block 0)")                                                   \
    X(".out.Q_lo20", &o_Q_lo20, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "variance-weighted Bartlett Q (modes 0..19)")                                               \
    X(".out.Q_beyond_b0", &o_Q_beyond_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "Zone-2 Bartlett Q beyond floor(tau)+P_ctrl")                                               \
    X(".out.Q_xmode_b0", &o_Q_xmode_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "Block-0 off-diag Bartlett Q_xmode")                                                        \
    X(".out.Q_xblk_b01", &o_Q_xblk_b01, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "Cross-block B0xB1 Bartlett Q_xmode")                                                       \
    X(".out.rho_d_b0", &o_rho_d_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "signed Block-0 variance-weighted rho(d)")                                                  \
    X(".out.cliff_ratio_b0", &o_cliff_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "MA(d-1) cliff |rho(d-1)| / |rho(d)| (B0)")                                                 \
    X(".out.rwu_d_lo20", &o_rwu_d_lo20, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "signed WFS-DM r_wu(d) on modes 0..19")                                                     \
    X(".out.Q_wu_lo20", &o_Q_wu_lo20, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "Bartlett WFS-DM Q_wu on modes 0..19")                                                      \
    X(".out.wu_recov_wfe_nm", &o_wu_recov_nm, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "OOS WFS-from-DM recoverable WFE [nm]")                                                     \
    X(".out.alert_flags", &o_alert_flags, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "bitmask: 1=lag 2=ringing 4=xmode 8=ZPO/DM")

MILK_PARAMS_DEFINE(cacao_modalwhite, FPS_PARAMS_MODALWHITE);

static int mwhite_solve_spd(uint32_t P, const double *A, const double *b, double *x)
{
    double L[MWHITE_MAX_ORDER * MWHITE_MAX_ORDER];
    double y[MWHITE_MAX_ORDER];

    for (uint32_t i = 0; i < P; i++)
    {
        for (uint32_t j = 0; j <= i; j++)
        {
            double s = A[i * P + j];
            for (uint32_t k = 0; k < j; k++)
            {
                s -= L[i * P + k] * L[j * P + k];
            }
            if (i == j)
            {
                if (s <= 1.0e-30)
                {
                    return 0;
                }
                L[i * P + j] = sqrt(s);
            }
            else
            {
                L[i * P + j] = s / L[j * P + j];
            }
        }
    }
    for (uint32_t i = 0; i < P; i++)
    {
        double s = b[i];
        for (uint32_t k = 0; k < i; k++)
        {
            s -= L[i * P + k] * y[k];
        }
        y[i] = s / L[i * P + i];
    }
    for (int i = (int) P - 1; i >= 0; i--)
    {
        double s = y[i];
        for (uint32_t k = (uint32_t) i + 1; k < P; k++)
        {
            s -= L[k * P + (uint32_t) i] * x[k];
        }
        x[i] = s / L[(uint32_t) i * P + (uint32_t) i];
    }
    return 1;
}

static void mwhite_oos_fold(const float *y_tr,
                            const float *x_tr,
                            uint32_t     n_tr,
                            const float *y_te,
                            const float *x_te,
                            uint32_t     n_te,
                            uint32_t     d,
                            uint32_t     P,
                            double       lam_rel,
                            double      *out_vy,
                            double      *out_ve)
{
    uint32_t t0 = d + P - 1;
    if (n_tr <= t0 + P + 2 || n_te <= t0 + 2)
    {
        *out_vy = 0.0;
        *out_ve = 0.0;
        return;
    }
    uint32_t nr = n_tr - t0;
    uint32_t ne = n_te - t0;

    double my_tr = 0.0;
    double mx_tr = 0.0;
    for (uint32_t t = t0; t < n_tr; t++)
    {
        my_tr += (double) y_tr[t];
    }
    my_tr /= (double) nr;
    for (uint32_t t = 0; t < n_tr; t++)
    {
        mx_tr += (double) x_tr[t];
    }
    mx_tr /= (double) n_tr;

    double my_te = 0.0;
    double mx_te = 0.0;
    for (uint32_t t = t0; t < n_te; t++)
    {
        my_te += (double) y_te[t];
    }
    my_te /= (double) ne;
    for (uint32_t t = 0; t < n_te; t++)
    {
        mx_te += (double) x_te[t];
    }
    mx_te /= (double) n_te;

    double G[MWHITE_MAX_ORDER * MWHITE_MAX_ORDER];
    double rhs[MWHITE_MAX_ORDER];
    double w[MWHITE_MAX_ORDER];
    memset(G, 0, sizeof(double) * P * P);
    memset(rhs, 0, sizeof(double) * P);

    for (uint32_t t = t0; t < n_tr; t++)
    {
        double yt = (double) y_tr[t] - my_tr;
        double xv[MWHITE_MAX_ORDER];
        for (uint32_t k = 0; k < P; k++)
        {
            xv[k] = (double) x_tr[t - (d + k)] - mx_tr;
        }
        for (uint32_t i = 0; i < P; i++)
        {
            rhs[i] += xv[i] * yt;
            for (uint32_t j = 0; j <= i; j++)
            {
                G[i * P + j] += xv[i] * xv[j];
            }
        }
    }

    double tr = 0.0;
    for (uint32_t i = 0; i < P; i++)
    {
        tr += G[i * P + i];
        for (uint32_t j = 0; j < i; j++)
        {
            G[j * P + i] = G[i * P + j];
        }
    }
    tr /= (double) P;

    memset(w, 0, sizeof(double) * P);
    double reg = lam_rel * ((tr > 1.0e-24) ? tr : 1.0e-24);
    for (uint32_t i = 0; i < P; i++)
    {
        G[i * P + i] += reg;
    }
    if (!mwhite_solve_spd(P, G, rhs, w))
    {
        memset(w, 0, sizeof(double) * P);
    }

    double sy2 = 0.0;
    double se2 = 0.0;
    for (uint32_t t = t0; t < n_te; t++)
    {
        double yt   = (double) y_te[t] - my_te;
        double pred = 0.0;
        for (uint32_t k = 0; k < P; k++)
        {
            pred += w[k] * ((double) x_te[t - (d + k)] - mx_te);
        }
        double err = yt - pred;
        sy2       += yt * yt;
        se2       += err * err;
    }
    *out_vy = sy2 / (double) ne;
    *out_ve = se2 / (double) ne;
}

static uint32_t mwhite_slice_frames(const IMAGE *im)
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
    return 1;
}

static const float *mwhite_slice_ptr(const IMAGE *im, uint32_t NBmode, uint32_t *out_nslice)
{
    if (im->md[0].naxis == 3 && im->md[0].size[2] == 2 && im->md[0].size[1] > 1)
    {
        uint32_t ns = im->md[0].size[1];
        uint32_t sl = (uint32_t) (im->md[0].cnt1 & 1ULL);
        *out_nslice = ns;
        return &im->array.F[(size_t) sl * ns * NBmode];
    }
    *out_nslice = mwhite_slice_frames(im);
    return im->array.F;
}

static float mwhite_xmode_block(const float  *Z_tgt,
                                uint32_t      bt,
                                const float  *Z_src,
                                uint32_t      bs,
                                const double *rho_tgt,
                                const double *rho_src,
                                uint32_t      stride_rho,
                                uint32_t      N,
                                uint32_t      d,
                                uint32_t      k_start,
                                uint32_t      k_count,
                                int           exclude_diag,
                                float        *C_buf)
{
    if (bt == 0 || bs == 0 || k_count == 0 || N <= k_start + k_count)
    {
        return 0.0f;
    }
    if (exclude_diag && bt == bs && bt < 2)
    {
        return 0.0f;
    }

    double sum_q = 0.0;
    for (uint32_t ki = 0; ki < k_count; ki++)
    {
        uint32_t k     = k_start + ki;
        uint32_t len   = N - k;
        float    alpha = 1.0f / (float) N;

#ifdef HAVE_BLAS
        cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) bt, (int) bs, (int) len, alpha,
                    Z_tgt + k, (int) N, Z_src, (int) N, 0.0f, C_buf, (int) bs);
#else
#    pragma omp parallel for schedule(static)
        for (uint32_t i = 0; i < bt; i++)
        {
            const float *zi = Z_tgt + (size_t) i * N + k;
            for (uint32_t j = 0; j < bs; j++)
            {
                const float *zj  = Z_src + (size_t) j * N;
                double       acc = 0.0;
                for (uint32_t t = 0; t < len; t++)
                {
                    acc += (double) zi[t] * (double) zj[t];
                }
                C_buf[(size_t) i * bs + j] = (float) (acc / (double) N);
            }
        }
#endif

        double lb_w   = ((double) N * (double) (N + 2)) / (double) (N - k);
        double lag_s  = 0.0;
        size_t npairs = 0;
        for (uint32_t i = 0; i < bt; i++)
        {
            const double *ri = &rho_tgt[(size_t) i * stride_rho];
            for (uint32_t j = 0; j < bs; j++)
            {
                if (exclude_diag && i == j)
                {
                    continue;
                }
                const double *rj  = &rho_src[(size_t) j * stride_rho];
                double        vij = 1.0;
                for (uint32_t l = 1; l < d; l++)
                {
                    vij += 2.0 * ri[l] * rj[l];
                }
                if (vij < 0.25)
                {
                    vij = 0.25;
                }
                double c = (double) C_buf[(size_t) i * bs + j];
                lag_s   += lb_w * (c * c) / vij;
                npairs++;
            }
        }
        if (npairs > 0)
        {
            sum_q += lag_s / (double) npairs;
        }
    }
    return (float) (sum_q / (double) k_count);
}

static MILK_HOT errno_t modalwhite_compute(const MILK_RUNNABLE *self)
{
    IMGID imgwfs = imgid_make_from_fpskey(p_inwfs, dcfpsname, ".inWFSbuff");
    resolveIMGID(&imgwfs, ERRMODE_WARN, dcimg, dcnimg);
    if (imgwfs.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode = imgwfs.md[0].size[0];
    if (NBmode == 0)
    {
        return RETURN_FAILURE;
    }

    int   has_dm = 0;
    IMGID imgdm  = imgid_make_from_fpskey(p_indm, dcfpsname, ".inDMbuff");
    if (p_indm[0] != '\0')
    {
        resolveIMGID(&imgdm, ERRMODE_NULL, dcimg, dcnimg);
        if (imgdm.ID != -1 && imgdm.md[0].size[0] == NBmode)
        {
            has_dm = 1;
        }
    }

    uint32_t cap_win = (p_nbwin >= 64) ? p_nbwin : 2048;
    if (cap_win > 32768)
    {
        cap_win = 32768;
    }
    uint32_t B = (p_blocksize >= 2) ? p_blocksize : 128;
    if (B > MWHITE_MAX_BLOCK)
    {
        B = MWHITE_MAX_BLOCK;
    }
    if (B > NBmode)
    {
        B = NBmode;
    }
    uint32_t b0 = B;
    uint32_t b1 = (NBmode > b0) ? ((NBmode - b0 < B) ? (NBmode - b0) : B) : 0;

    uint32_t max_slice = mwhite_slice_frames(imgwfs.im);
    if (has_dm)
    {
        uint32_t dm_sl = mwhite_slice_frames(imgdm.im);
        if (dm_sl > max_slice)
        {
            max_slice = dm_sl;
        }
    }

    // Create output SHM streams
    char  sname[STRINGMAXLEN_IMGNAME];
    IMGID imgQ, imgRhod, imgPrf, imgRecov, imgRwud, imgQwu, imgAcf;
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_Q", p_loopindex);
    imgQ = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_rhod", p_loopindex);
    imgRhod = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_prf", p_loopindex);
    imgPrf = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_recov", p_loopindex);
    imgRecov = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_rwud", p_loopindex);
    imgRwud = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_Qwu", p_loopindex);
    imgQwu = stream_connect_create_2Df32(sname, NBmode, 1);
    WRITE_IMAGENAME(sname, "aol%lu_mwhite_acf", p_loopindex);
    imgAcf = stream_connect_create_2Df32(sname, MWHITE_MAX_LAG, NBmode);

    float  *wfs_hist = (float *) calloc((size_t) cap_win * NBmode, sizeof(float));
    float  *dm_hist  = has_dm ? (float *) calloc((size_t) cap_win * NBmode, sizeof(float)) : NULL;
    float  *tmp_wfs  = (float *) calloc((size_t) max_slice * NBmode, sizeof(float));
    float  *tmp_dm   = has_dm ? (float *) calloc((size_t) max_slice * NBmode, sizeof(float)) : NULL;
    float  *W_mn     = (float *) malloc(sizeof(float) * (size_t) NBmode * cap_win);
    float  *U_mn     = has_dm ? (float *) malloc(sizeof(float) * (size_t) NBmode * cap_win) : NULL;
    float  *Z_b0     = (float *) malloc(sizeof(float) * (size_t) b0 * cap_win);
    float  *Z_b1     = (b1 > 0) ? (float *) malloc(sizeof(float) * (size_t) b1 * cap_win) : NULL;
    float  *C_buf    = (float *) malloc(sizeof(float) * (size_t) b0 * b0);
    double *var_w    = (double *) calloc(NBmode, sizeof(double));
    double *var_u    = (double *) calloc(NBmode, sizeof(double));
    double *rho_w    = (double *) calloc((size_t) NBmode * MWHITE_MAX_LAG, sizeof(double));
    double *rho_u    = (double *) calloc((size_t) NBmode * MWHITE_MAX_LAG, sizeof(double));
    double *r_wu     = (double *) calloc((size_t) NBmode * MWHITE_MAX_LAG, sizeof(double));
    double *q_mode   = (double *) calloc(NBmode, sizeof(double));
    double *q_bey    = (double *) calloc(NBmode, sizeof(double));
    double *q_wu     = (double *) calloc(NBmode, sizeof(double));
    double *prf_m    = (double *) calloc(NBmode, sizeof(double));
    double *recov_v  = (double *) calloc(NBmode, sizeof(double));
    double *wurec_v  = (double *) calloc(NBmode, sizeof(double));

    if (!wfs_hist || !tmp_wfs || !W_mn || !Z_b0 || !C_buf || !var_w || !var_u || !rho_w ||
        !rho_u || !r_wu || !q_mode || !q_bey || !q_wu || !prf_m || !recov_v || !wurec_v ||
        (has_dm && (!dm_hist || !tmp_dm || !U_mn)) || (b1 > 0 && !Z_b1))
    {
        free(wfs_hist);
        free(dm_hist);
        free(tmp_wfs);
        free(tmp_dm);
        free(W_mn);
        free(U_mn);
        free(Z_b0);
        free(Z_b1);
        free(C_buf);
        free(var_w);
        free(var_u);
        free(rho_w);
        free(rho_u);
        free(r_wu);
        free(q_mode);
        free(q_bey);
        free(q_wu);
        free(prf_m);
        free(recov_v);
        free(wurec_v);
        return RETURN_FAILURE;
    }

    uint32_t n_valid = 0;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        uint32_t     ns_wfs  = 1;
        const float *src_wfs = mwhite_slice_ptr(imgwfs.im, NBmode, &ns_wfs);
        if (ns_wfs > max_slice)
        {
            ns_wfs = max_slice;
        }
        memcpy(tmp_wfs, src_wfs, sizeof(float) * (size_t) NBmode * ns_wfs);

        uint32_t n_add = ns_wfs;
        if (has_dm)
        {
            uint32_t     ns_dm  = 1;
            const float *src_dm = mwhite_slice_ptr(imgdm.im, NBmode, &ns_dm);
            if (ns_dm > max_slice)
            {
                ns_dm = max_slice;
            }
            if (ns_dm < n_add)
            {
                n_add = ns_dm;
            }
            memcpy(tmp_dm, src_dm, sizeof(float) * (size_t) NBmode * n_add);
        }

        if (n_add >= cap_win)
        {
            uint32_t off = n_add - cap_win;
            memcpy(wfs_hist, &tmp_wfs[(size_t) off * NBmode],
                   sizeof(float) * (size_t) cap_win * NBmode);
            if (has_dm)
            {
                memcpy(dm_hist, &tmp_dm[(size_t) off * NBmode],
                       sizeof(float) * (size_t) cap_win * NBmode);
            }
            n_valid = cap_win;
        }
        else
        {
            if (n_valid + n_add > cap_win)
            {
                uint32_t drop = (n_valid + n_add) - cap_win;
                uint32_t keep = n_valid - drop;
                memmove(wfs_hist, &wfs_hist[(size_t) drop * NBmode],
                        sizeof(float) * (size_t) keep * NBmode);
                if (has_dm)
                {
                    memmove(dm_hist, &dm_hist[(size_t) drop * NBmode],
                            sizeof(float) * (size_t) keep * NBmode);
                }
                n_valid = keep;
            }
            memcpy(&wfs_hist[(size_t) n_valid * NBmode], tmp_wfs,
                   sizeof(float) * (size_t) n_add * NBmode);
            if (has_dm)
            {
                memcpy(&dm_hist[(size_t) n_valid * NBmode], tmp_dm,
                       sizeof(float) * (size_t) n_add * NBmode);
            }
            n_valid += n_add;
        }

        uint32_t N   = n_valid;
        o_n_samples  = N;
        float lat_fr = (p_latency_fr >= 0.05f && p_latency_fr <= 24.0f) ? p_latency_fr : 2.5f;
        uint32_t d   = (uint32_t) ceil((double) lat_fr - 1.0e-4);
        if (d < 1)
        {
            d = 1;
        }
        o_d_ceil = d;

        uint32_t K = (p_klags >= 1) ? p_klags : 16;
        if (d + K > MWHITE_MAX_LAG)
        {
            K = MWHITE_MAX_LAG - d;
        }
        uint32_t P_rec = (p_precov >= 1 && p_precov <= MWHITE_MAX_ORDER) ? p_precov : 8;
        uint32_t k_bey = (uint32_t) floor((double) lat_fr) + ((p_pctrl >= 1) ? p_pctrl : 8);
        uint32_t K_bey = 8;
        if (k_bey + K_bey > MWHITE_MAX_LAG)
        {
            K_bey = (k_bey < MWHITE_MAX_LAG) ? (MWHITE_MAX_LAG - k_bey) : 0;
        }
        uint32_t K_acf = d + K;
        if (k_bey + K_bey > K_acf)
        {
            K_acf = k_bey + K_bey;
        }
        if (N < 4 * K_acf + 16 || N < 2 * (d + P_rec + 4))
        {
            continue;
        }

        double lam_rel = (p_reglambda > 1.0e-9f) ? (double) p_reglambda : 0.01;
        uint32_t mid   = N / 2;

        // Transpose to mode-major [NBmode x N], demean, compute ACF, XCF, Bartlett Q, OOS recov
#pragma omp parallel for schedule(static)
        for (uint32_t m = 0; m < NBmode; m++)
        {
            float *wm = &W_mn[(size_t) m * N];
            double mw = 0.0;
            for (uint32_t t = 0; t < N; t++)
            {
                float v = wfs_hist[(size_t) t * NBmode + m];
                wm[t]   = v;
                mw     += (double) v;
            }
            mw /= (double) N;
            double sw2 = 0.0;
            for (uint32_t t = 0; t < N; t++)
            {
                double dv = (double) wm[t] - mw;
                wm[t]     = (float) dv;
                sw2      += dv * dv;
            }
            double vw = sw2 / (double) N;
            var_w[m]  = vw;

            double *rw = &rho_w[(size_t) m * MWHITE_MAX_LAG];
            memset(rw, 0, sizeof(double) * MWHITE_MAX_LAG);
            if (vw > 1.0e-24)
            {
                rw[0]         = 1.0;
                double inv_nv = 1.0 / ((double) N * vw);
                for (uint32_t k = 1; k < K_acf; k++)
                {
                    double acc = 0.0;
                    for (uint32_t t = k; t < N; t++)
                    {
                        acc += (double) wm[t] * (double) wm[t - k];
                    }
                    rw[k] = acc * inv_nv;
                }
            }

            double vm = 1.0;
            for (uint32_t j = 1; j < d; j++)
            {
                vm += 2.0 * rw[j] * rw[j];
            }
            if (vm < 1.0e-12)
            {
                vm = 1.0e-12;
            }
            double qraw = 0.0;
            for (uint32_t k = d; k < d + K; k++)
            {
                double lbw = ((double) N * (double) (N + 2)) / (double) (N - k);
                qraw      += lbw * rw[k] * rw[k];
            }
            q_mode[m] = qraw / ((double) K * vm);

            double qraw_b = 0.0;
            if (K_bey > 0)
            {
                for (uint32_t k = k_bey; k < k_bey + K_bey; k++)
                {
                    double lbw = ((double) N * (double) (N + 2)) / (double) (N - k);
                    qraw_b    += lbw * rw[k] * rw[k];
                }
                q_bey[m] = qraw_b / ((double) K_bey * vm);
            }
            else
            {
                q_bey[m] = 0.0;
            }

            // 2-fold chronological split-half OOS diagonal AR(P_rec) at lag >= d
            double vy1 = 0.0, ve1 = 0.0, vy2 = 0.0, ve2 = 0.0;
            mwhite_oos_fold(wm, wm, mid, wm + mid, wm + mid, N - mid, d, P_rec, lam_rel, &vy1,
                            &ve1);
            mwhite_oos_fold(wm + mid, wm + mid, N - mid, wm, wm, mid, d, P_rec, lam_rel, &vy2,
                            &ve2);
            double vy  = 0.5 * (vy1 + vy2);
            double ve  = 0.5 * (ve1 + ve2);
            double prf = (vy > 1.0e-24) ? (1.0 - ve / vy) : 0.0;
            prf_m[m]   = prf;
            recov_v[m] = (prf > 0.0) ? (prf * vy) : 0.0;

            // Optional DM buffer: ACF_u, XCF r_wu, Bartlett Q_wu, and W|U OOS recovery
            double *ru  = &rho_u[(size_t) m * MWHITE_MAX_LAG];
            double *rwu = &r_wu[(size_t) m * MWHITE_MAX_LAG];
            memset(ru, 0, sizeof(double) * MWHITE_MAX_LAG);
            memset(rwu, 0, sizeof(double) * MWHITE_MAX_LAG);
            q_wu[m]    = 0.0;
            wurec_v[m] = 0.0;
            var_u[m]   = 0.0;

            if (has_dm)
            {
                float *um = &U_mn[(size_t) m * N];
                double mu = 0.0;
                for (uint32_t t = 0; t < N; t++)
                {
                    float v = dm_hist[(size_t) t * NBmode + m];
                    um[t]   = v;
                    mu     += (double) v;
                }
                mu /= (double) N;
                double su2 = 0.0;
                for (uint32_t t = 0; t < N; t++)
                {
                    double dv = (double) um[t] - mu;
                    um[t]     = (float) dv;
                    su2      += dv * dv;
                }
                double vu = su2 / (double) N;
                var_u[m]  = vu;
                if (vu > 1.0e-24)
                {
                    ru[0]         = 1.0;
                    double inv_nu = 1.0 / ((double) N * vu);
                    for (uint32_t k = 1; k < d; k++)
                    {
                        double acc = 0.0;
                        for (uint32_t t = k; t < N; t++)
                        {
                            acc += (double) um[t] * (double) um[t - k];
                        }
                        ru[k] = acc * inv_nu;
                    }
                }
                double denom = sqrt(vw * vu);
                if (denom > 1.0e-24)
                {
                    double inv_nd = 1.0 / ((double) N * denom);
                    for (uint32_t k = d; k < d + K; k++)
                    {
                        double acc = 0.0;
                        for (uint32_t t = k; t < N; t++)
                        {
                            acc += (double) wm[t] * (double) um[t - k];
                        }
                        rwu[k] = acc * inv_nd;
                    }
                    double vwu = 1.0;
                    for (uint32_t j = 1; j < d; j++)
                    {
                        vwu += 2.0 * rw[j] * ru[j];
                    }
                    if (vwu < 0.25)
                    {
                        vwu = 0.25;
                    }
                    double qw = 0.0;
                    for (uint32_t k = d; k < d + K; k++)
                    {
                        double wk = (double) N / (double) (N - k);
                        qw       += wk * (double) N * rwu[k] * rwu[k];
                    }
                    q_wu[m] = qw / ((double) K * vwu);
                }

                double wvy1 = 0.0, wve1 = 0.0, wvy2 = 0.0, wve2 = 0.0;
                mwhite_oos_fold(wm, um, mid, wm + mid, um + mid, N - mid, d, P_rec, lam_rel, &wvy1,
                                &wve1);
                mwhite_oos_fold(wm + mid, um + mid, N - mid, wm, um, mid, d, P_rec, lam_rel, &wvy2,
                                &wve2);
                double wvy  = 0.5 * (wvy1 + wvy2);
                double wve  = 0.5 * (wve1 + wve2);
                double wprf = (wvy > 1.0e-24) ? (1.0 - wve / wvy) : 0.0;
                wurec_v[m]  = (wprf > 0.0) ? (wprf * wvy) : 0.0;
            }
        }

        // Standardize Block 0 (and Block 1) for exact Isserlis/Bartlett Q_xmode
        for (uint32_t i = 0; i < b0; i++)
        {
            double inv_s = (var_w[i] > 1.0e-24) ? (1.0 / sqrt(var_w[i])) : 0.0;
            const float *src = &W_mn[(size_t) i * N];
            float       *dst = &Z_b0[(size_t) i * N];
            for (uint32_t t = 0; t < N; t++)
            {
                dst[t] = (float) ((double) src[t] * inv_s);
            }
        }
        uint32_t K_xm = (K < 8) ? K : 8;
        float    qx_b0 = mwhite_xmode_block(Z_b0, b0, Z_b0, b0, rho_w, rho_w, MWHITE_MAX_LAG, N, d,
                                            d, K_xm, 1, C_buf);
        float    qx_01 = 0.0f;
        if (b1 > 0)
        {
            for (uint32_t j = 0; j < b1; j++)
            {
                uint32_t     m     = b0 + j;
                double       inv_s = (var_w[m] > 1.0e-24) ? (1.0 / sqrt(var_w[m])) : 0.0;
                const float *src   = &W_mn[(size_t) m * N];
                float       *dst   = &Z_b1[(size_t) j * N];
                for (uint32_t t = 0; t < N; t++)
                {
                    dst[t] = (float) ((double) src[t] * inv_s);
                }
            }
            qx_01 = mwhite_xmode_block(Z_b0, b0, Z_b1, b1, rho_w,
                                       &rho_w[(size_t) b0 * MWHITE_MAX_LAG], MWHITE_MAX_LAG, N, d,
                                       d, K_xm, 0, C_buf);
        }

        // Aggregate band scalars (mode values in um -> report WFE in nm)
        uint32_t lo20       = (NBmode < 20) ? NBmode : 20;
        double   sum_v_all  = 0.0;
        double   sum_v_b0   = 0.0;
        double   sum_v_lo20 = 0.0;
        double   sum_r_all  = 0.0;
        double   sum_r_b0   = 0.0;
        double   sum_wr_all = 0.0;
        double   sQ_all     = 0.0;
        double   sQ_b0      = 0.0;
        double   sQ_lo20    = 0.0;
        double   sQbey_b0   = 0.0;
        double   srhod_b0   = 0.0;
        double   sabs_dm1   = 0.0;
        double   sabs_d     = 0.0;
        double   srwu_lo20  = 0.0;
        double   sQwu_lo20  = 0.0;

        for (uint32_t m = 0; m < NBmode; m++)
        {
            double vw = var_w[m];
            sum_v_all  += vw;
            sum_r_all  += recov_v[m];
            sum_wr_all += wurec_v[m];
            sQ_all     += vw * q_mode[m];
            if (m < b0)
            {
                const double *rw = &rho_w[(size_t) m * MWHITE_MAX_LAG];
                sum_v_b0  += vw;
                sum_r_b0  += recov_v[m];
                sQ_b0     += vw * q_mode[m];
                sQbey_b0  += vw * q_bey[m];
                srhod_b0  += vw * rw[d];
                sabs_dm1  += vw * ((d > 1) ? fabs(rw[d - 1]) : 1.0);
                sabs_d    += vw * fabs(rw[d]);
            }
            if (m < lo20)
            {
                const double *rwu = &r_wu[(size_t) m * MWHITE_MAX_LAG];
                sum_v_lo20 += vw;
                sQ_lo20    += vw * q_mode[m];
                srwu_lo20  += vw * rwu[d];
                sQwu_lo20  += vw * q_wu[m];
            }
        }

        o_wfs_rms_nm    = (float) (1000.0 * sqrt(sum_v_all));
        o_recov_wfe_nm  = (float) (1000.0 * sqrt(sum_r_all));
        o_recov_var_pct = (sum_v_all > 1.0e-24) ? (float) (100.0 * sum_r_all / sum_v_all) : 0.0f;
        o_b0_recov_nm   = (float) (1000.0 * sqrt(sum_r_b0));
        o_Q_vw_all      = (sum_v_all > 1.0e-24) ? (float) (sQ_all / sum_v_all) : 0.0f;
        o_Q_vw_b0       = (sum_v_b0 > 1.0e-24) ? (float) (sQ_b0 / sum_v_b0) : 0.0f;
        o_Q_lo20        = (sum_v_lo20 > 1.0e-24) ? (float) (sQ_lo20 / sum_v_lo20) : 0.0f;
        o_Q_beyond_b0   = (sum_v_b0 > 1.0e-24) ? (float) (sQbey_b0 / sum_v_b0) : 0.0f;
        o_Q_xmode_b0    = qx_b0;
        o_Q_xblk_b01    = qx_01;
        o_rho_d_b0      = (sum_v_b0 > 1.0e-24) ? (float) (srhod_b0 / sum_v_b0) : 0.0f;
        o_cliff_b0      = (sabs_d > 1.0e-12) ? (float) (sabs_dm1 / sabs_d) : 0.0f;
        o_rwu_d_lo20 =
            (has_dm && sum_v_lo20 > 1.0e-24) ? (float) (srwu_lo20 / sum_v_lo20) : 0.0f;
        o_Q_wu_lo20 =
            (has_dm && sum_v_lo20 > 1.0e-24) ? (float) (sQwu_lo20 / sum_v_lo20) : 0.0f;
        o_wu_recov_nm   = has_dm ? (float) (1000.0 * sqrt(sum_wr_all)) : 0.0f;

        uint32_t flags = 0;
        if (o_Q_vw_b0 > 10.0f && o_rho_d_b0 > 0.08f)
        {
            flags |= 0x1u;
        }
        if (o_rho_d_b0 < -0.08f)
        {
            flags |= 0x2u;
        }
        if (o_Q_xmode_b0 > 2.5f)
        {
            flags |= 0x4u;
        }
        if (has_dm && o_Q_wu_lo20 > 6.0f && fabsf(o_rwu_d_lo20) > 0.15f)
        {
            flags |= 0x8u;
        }
        o_alert_flags = flags;

        // Publish per-mode SHM streams
        imgQ.md[0].write     = 1;
        imgRhod.md[0].write  = 1;
        imgPrf.md[0].write   = 1;
        imgRecov.md[0].write = 1;
        imgRwud.md[0].write  = 1;
        imgQwu.md[0].write   = 1;
        imgAcf.md[0].write   = 1;
        for (uint32_t m = 0; m < NBmode; m++)
        {
            const double *rw              = &rho_w[(size_t) m * MWHITE_MAX_LAG];
            const double *rwu             = &r_wu[(size_t) m * MWHITE_MAX_LAG];
            imgQ.im->array.F[m]           = (float) q_mode[m];
            imgRhod.im->array.F[m]        = (float) rw[d];
            imgPrf.im->array.F[m]         = (float) prf_m[m];
            imgRecov.im->array.F[m]       = (float) sqrt(recov_v[m]);
            imgRwud.im->array.F[m]        = has_dm ? (float) rwu[d] : 0.0f;
            imgQwu.im->array.F[m]         = has_dm ? (float) q_wu[m] : 0.0f;
            float *acf_row                = &imgAcf.im->array.F[(size_t) m * MWHITE_MAX_LAG];
            for (uint32_t k = 0; k < MWHITE_MAX_LAG; k++)
            {
                acf_row[k] = (float) rw[k];
            }
        }
        processinfo_update_output_stream(processinfo, imgQ.im, NULL);
        processinfo_update_output_stream(processinfo, imgRhod.im, NULL);
        processinfo_update_output_stream(processinfo, imgPrf.im, NULL);
        processinfo_update_output_stream(processinfo, imgRecov.im, NULL);
        processinfo_update_output_stream(processinfo, imgRwud.im, NULL);
        processinfo_update_output_stream(processinfo, imgQwu.im, NULL);
        processinfo_update_output_stream(processinfo, imgAcf.im, NULL);
    }
    MILK_PROCINFO_END(self)

    free(wfs_hist);
    free(dm_hist);
    free(tmp_wfs);
    free(tmp_dm);
    free(W_mn);
    free(U_mn);
    free(Z_b0);
    free(Z_b1);
    free(C_buf);
    free(var_w);
    free(var_u);
    free(rho_w);
    free(rho_u);
    free(r_wu);
    free(q_mode);
    free(q_bey);
    free(q_wu);
    free(prf_m);
    free(recov_v);
    free(wurec_v);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_modalwhite = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "modalwhite",
                 .cmdkey      = "modalwhite",
                 .description = "closed-loop residual whiteness and predictability monitor",
                 .description_long =
                     "Monitor closed-loop modal WFS and DM buffers for MA(d-1) lag>=ceil(tau) "
                     "Bartlett whiteness, cross-mode wind coupling, OOS recoverable WFE, and "
                     "WFS-DM leakage/ZPO ramp diagnostics." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacao_modalwhite),
    .compute = modalwhite_compute,
};
EXPORT_FPSEXEC(cacao, "modalwhite", runnable_AOloopControl_modalwhite);
