// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl_mkEOFPF.c
 * @brief   Online block cross-mode EOF (VAR(P)) predictive filter builder from PSOL telemetry
 *
 * Implements the Guyon & Males (2017) Empirical Orthogonal Functions (EOF) multi-mode transition
 * matrix predictor grouped into contiguous mode blocks of size B, with automatic diagonal AR(P)
 * progressive bootstrapping during cold-start accumulation to prevent petal slips on segmented
 * or V-spider pupils before sufficient samples (N >= minsamples) are buffered.
 */

// MILK_CMAKE_MANDATE_LAPACKE
// MILK_CMAKE_REQUEST_BLAS

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "milk_blas_lapacke.h"
#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"
#include "AOloopControl_PredictiveControl/eoflin_phys.h"

#define EOFPF_MAX_ORDER 64
#define EOFPF_MAX_BLOCK 512

static uint64_t p_loopindex                                = 0;
static char     p_inbuff[FUNCTION_PARAMETER_STRMAXLEN]     = "aol0_modevalOL_buff";
static char     p_outPF[FUNCTION_PARAMETER_STRMAXLEN]      = "aol0_eofPFmat";
static uint32_t p_pforder                                  = 8;
static uint32_t p_diagorder                                = 16;
static uint32_t p_blocksize                                = 128;
static float    p_pflatency                                = 2.4f;
static uint32_t p_nbtrain                                  = 16384;
static uint32_t p_minsamples                               = 12000;
static uint32_t p_diagminsamples                           = 256;
static uint32_t p_updateperiod                             = 4;
static uint32_t p_nbmodemax                                = 4096;
static float    p_reglambda                                = 1.0e-4f;
static float    p_diagreglambda                            = 5.0e-3f;
static float    p_maxnorm2                                 = 2.0f;
static float    p_diagmaxnorm2                             = 1.35f;
static float    p_maxdcgain                                = 0.995f;
static float    p_loopgain                                 = 0.50f;
static int32_t  p_zoh_ridge                                = 1;
static float    p_gram_halflife                            = 0.0f;
static char     p_dm_kernel[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_wl_layers[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_dmmodes[FUNCTION_PARAMETER_STRMAXLEN]    = "";
static float    p_pup_diam                                 = 8.0f;
static float    p_dm_dx                                    = 0.0f;
static float    p_frate_hz                                 = 1000.0f;
static float    p_l0_m                                     = 25.0f;
static float    p_n_prior                                  = 1024.0f;
static float    p_wl_switch_decay                          = 0.25f;
static uint32_t p_regime_kmax                              = 0;
static float    p_regime_kappa                             = 0.10f;
static float    p_regime_bw                                = 3.0f;
static float    p_regime_rlim                              = 3.0f;
static char     p_regime_in[FUNCTION_PARAMETER_STRMAXLEN]  = "";

static uint64_t o_n_updates     = 0;
static uint64_t o_n_eof_updates = 0;
static uint32_t o_eof_active    = 0;
static uint32_t o_n_samples     = 0;
static float    o_eff_samples   = 0.0f;
static uint32_t o_phys_active   = 0;
static uint32_t o_fx_active     = 0;
static uint32_t o_regime_k      = 0;
static uint64_t o_wl_switches   = 0;
static float    o_rms_ol        = 0.0f;
static float    o_rms_zoh       = 0.0f;
static float    o_rms_pred      = 0.0f;
static float    o_var_red_pct   = 0.0f;
static float    o_offdiag_frac  = 0.0f;
static float    o_last_lam      = 0.0f;
static float    o_rms_zoh_oos   = 0.0f;
static float    o_rms_pred_oos  = 0.0f;
static float    o_var_red_oos   = 0.0f;
static float    o_overfit_ratio = 0.0f;
static float    o_oos_rho_d_b0  = 0.0f;

#define FPS_PARAMS_MKEOFPF(X)                                                                     \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".inbuff", p_inbuff, FPTYPE_STREAMNAME, 1,                                                  \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL telemetry buffer")    \
    X(".outPF", p_outPF, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,   \
      "output block-EOF matrix [(PForder*B) x NBmode]")                                           \
    X(".PForder", &p_pforder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "EOF lookback tap order (1..64)")                                                           \
    X(".diagorder", &p_diagorder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "bootstrap diag AR order (1..64)")                                                          \
    X(".blocksize", &p_blocksize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "coupled mode block size B (1..512)")                                                       \
    X(".PFlatency", &p_pflatency, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "prediction horizon [frames]")   \
    X(".NBtrain", &p_nbtrain, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "sliding PSOL training window [frames]")                                                    \
    X(".minsamples", &p_minsamples, FPTYPE_UINT32, 0,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min frames for cross-mode EOF") \
    X(".diagminsamples", &p_diagminsamples, FPTYPE_UINT32, 0,                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min frames for diag bootstrap") \
    X(".updateperiod", &p_updateperiod, FPTYPE_UINT32, 0,                                         \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EOF solve stride [buffers]")    \
    X(".NBmodeMax", &p_nbmodemax, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max modes predicted")           \
    X(".reglambda", &p_reglambda, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EOF relative Tikhonov lambda")  \
    X(".diagreglambda", &p_diagreglambda, FPTYPE_FLOAT32, 0,                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "bootstrap diag AR lambda")      \
    X(".maxnorm2", &p_maxnorm2, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "EOF p95 row ||F_m||_2^2 cap")   \
    X(".diagmaxnorm2", &p_diagmaxnorm2, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "bootstrap diag ||w||_2^2 cap")  \
    X(".maxdcgain", &p_maxdcgain, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max self-DC gain (<1)")         \
    X(".loopgain", &p_loopgain, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "filter EMA update gain")        \
    X(".zoh_ridge", &p_zoh_ridge, FPTYPE_ONOFF, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "center ridge toward ZOH identity map (vs zero)")                                           \
    X(".gram_halflife", &p_gram_halflife, FPTYPE_FLOAT32, 0,                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "extended-memory Gram half-life [frames] (0=sliding NBtrain)")                              \
    X(".dm_kernel", p_dm_kernel, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "optional Pass-2 H_diag/H_mimo FITS for Filtered-X EOF")                                    \
    X(".wl_layers", p_wl_layers, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "optional mlt-windlayers wl_layers SHM stream for physics prior")                           \
    X(".dmmodes", p_dmmodes, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "optional 3D DM modes FITS cube [nx,ny,NBmode] for physics prior")                          \
    X(".pup_diam", &p_pup_diam, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "pupil diameter across DM aperture [m]")                                                    \
    X(".dm_dx", &p_dm_dx, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "DM actuator pitch on pupil [m] (0=pup_diam/nx)")                                           \
    X(".frate_hz", &p_frate_hz, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "physical loop frame rate [Hz]") \
    X(".l0_m", &p_l0_m, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "von Karman outer scale L0 [m]")                                                            \
    X(".n_prior", &p_n_prior, FPTYPE_FLOAT32, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "windlayers physics-Gram prior sample weight (0=off)")                                      \
    X(".wl_switch_decay", &p_wl_switch_decay, FPTYPE_FLOAT32, 0,                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "empirical Gram discount on wind switch (0..1)")                                            \
    X(".regime_kmax", &p_regime_kmax, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "max regime Gram library anchors (0=off, 2..16=on)")                                        \
    X(".regime_kappa", &p_regime_kappa, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "regime weight floor kappa for dissimilar anchors")                                         \
    X(".regime_bw", &p_regime_bw, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "regime Gaussian kernel bandwidth h")                                                       \
    X(".regime_rlim", &p_regime_rlim, FPTYPE_FLOAT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "regime anchor spawn radius")                                                               \
    X(".regime_in", p_regime_in, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "optional 1D external regime descriptor SHM stream")                                        \
    X(".out.n_updates", &o_n_updates, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "total published filter updates")                                                           \
    X(".out.n_eof_updates", &o_n_eof_updates, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,            \
      "completed cross-mode EOF updates")                                                         \
    X(".out.eof_active", &o_eof_active, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "state: 0=init, 1=diag-bootstrap, 2=EOF")                                                   \
    X(".out.n_samples", &o_n_samples, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "active PSOL training samples in ring")                                                     \
    X(".out.eff_samples", &o_eff_samples, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "effective Gram sample weight")                                                             \
    X(".out.phys_active", &o_phys_active, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "1 when windlayers physics-Gram prior is active")                                           \
    X(".out.fx_active", &o_fx_active, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "1 when Filtered-X Pass-2 dm_kernel is active")                                             \
    X(".out.regime_k", &o_regime_k, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "active regime Gram anchors")                                                               \
    X(".out.wl_switches", &o_wl_switches, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "detected wind-layer switch events")                                                        \
    X(".out.rms_ol", &o_rms_ol, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "quadrature open-loop modal RMS")                                                           \
    X(".out.rms_zoh", &o_rms_zoh, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "quadrature pure-lag modal RMS")                                                            \
    X(".out.rms_pred", &o_rms_pred, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "quadrature EOF predicted modal RMS")                                                       \
    X(".out.var_red_pct", &o_var_red_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "in-window variance reduction vs ZOH [%]")                                                  \
    X(".out.offdiag_frac", &o_offdiag_frac, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "off-diagonal cross-mode L2 energy fraction")                                               \
    X(".out.last_lam", &o_last_lam, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "effective relative lambda on block 0")                                                     \
    X(".out.rms_zoh_oos", &o_rms_zoh_oos, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "out-of-sample pure-lag ZOH modal RMS")                                                     \
    X(".out.rms_pred_oos", &o_rms_pred_oos, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "out-of-sample published filter modal RMS")                                                 \
    X(".out.var_red_oos_pct", &o_var_red_oos, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "out-of-sample variance reduction vs ZOH [%]")                                              \
    X(".out.overfit_ratio", &o_overfit_ratio, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "OOS/in-sample predicted RMS ratio")                                                        \
    X(".out.oos_rho_d_b0", &o_oos_rho_d_b0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "Block-0 lag-d autocorr of OOS pred residual")

MILK_PARAMS_DEFINE(cacaopc_mkEOFPF, FPS_PARAMS_MKEOFPF);

static int cmp_double_asc(const void *a, const void *b)
{
    double da = *(const double *) a;
    double db = *(const double *) b;
    return (da > db) - (da < db);
}

/**
 * @brief Solve small SPD system A * x = b (size P <= EOFPF_MAX_ORDER) via Cholesky.
 */
static int solve_spd_diag(uint32_t P, const double *A, const double *b, double *x)
{
    double L[EOFPF_MAX_ORDER * EOFPF_MAX_ORDER];
    double y[EOFPF_MAX_ORDER];

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

/**
 * @brief Progressive cold-start bootstrap: fit per-mode diagonal AR(P_diag) into block-EOF layout.
 *        Taps 0..P_eof-1 are placed on the block diagonal (k*B + local_m); extra taps
 *        P_eof..P_diag-1 are placed in the trailing diag_tail columns at offset P_eof*B.
 */
static void solve_diag_bootstrap(const float *hist_x,
                                 const float *hist_y,
                                 int          use_fx,
                                 uint32_t     fx_start,
                                 uint32_t     NBmode,
                                 uint32_t     eff_modes,
                                 uint32_t     n_valid,
                                 uint32_t     P_eof,
                                 uint32_t     P_diag,
                                 uint32_t     B,
                                 uint32_t     stride,
                                 uint32_t     lat_i,
                                 double       lat_a,
                                 float       *w_stage,
                                 double      *out_var_ol,
                                 double      *out_var_zoh,
                                 double      *out_var_pred)
{
    uint32_t P      = P_diag;
    uint32_t pb_len = P_eof * B;
    uint32_t t0_idx = use_fx ? (fx_start + P - 1u) : (P - 1u);
    uint32_t t1_idx = use_fx ? n_valid : (n_valid - lat_i - 1u);
    uint32_t nrows  = (t1_idx > t0_idx) ? (t1_idx - t0_idx) : 0;
    if (nrows < 16)
    {
        return;
    }

    double reg_rel   = (p_diagreglambda > 1.0e-8f) ? (double) p_diagreglambda : 5.0e-3;
    double max_norm2 = (p_diagmaxnorm2 > 0.1f) ? (double) p_diagmaxnorm2 : 1.35;
    double max_dc    = 0.985;

    double sum_var_ol   = 0.0;
    double sum_var_zoh  = 0.0;
    double sum_var_pred = 0.0;

    memset(w_stage, 0, sizeof(float) * (size_t) NBmode * stride);

#pragma omp parallel for schedule(static) reduction(+ : sum_var_ol, sum_var_zoh, sum_var_pred)
    for (uint32_t m = 0; m < eff_modes; m++)
    {
        double G[EOFPF_MAX_ORDER * EOFPF_MAX_ORDER];
        double rhs[EOFPF_MAX_ORDER];
        double rhs_r[EOFPF_MAX_ORDER];
        double A[EOFPF_MAX_ORDER * EOFPF_MAX_ORDER];
        double w_sol[EOFPF_MAX_ORDER];

        memset(G, 0, sizeof(double) * P * P);
        memset(rhs, 0, sizeof(double) * P);

        double s_y    = 0.0;
        double s_y2   = 0.0;
        double s_zoh2 = 0.0;

        for (uint32_t r = 0; r < nrows; r++)
        {
            uint32_t tk = t0_idx + r;
            double   yk;
            if (use_fx)
            {
                yk = (double) hist_y[(size_t) tk * NBmode + m];
            }
            else
            {
                double y0 = (double) hist_y[(size_t) (tk + lat_i) * NBmode + m];
                double y1 = (double) hist_y[(size_t) (tk + lat_i + 1u) * NBmode + m];
                yk        = (1.0 - lat_a) * y0 + lat_a * y1;
            }
            double x0   = (double) hist_x[(size_t) tk * NBmode + m];
            double dz   = yk - x0;
            s_y    += yk;
            s_y2   += yk * yk;
            s_zoh2 += dz * dz;

            for (uint32_t i = 0; i < P; i++)
            {
                double xi = (double) hist_x[(size_t) (tk - i) * NBmode + m];
                rhs[i] += xi * yk;
                for (uint32_t j = 0; j <= i; j++)
                {
                    G[i * P + j] += xi * (double) hist_x[(size_t) (tk - j) * NBmode + m];
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

        memset(w_sol, 0, sizeof(double) * P);
        if (tr > 1.0e-26)
        {
            double lam = reg_rel * tr;
            int    ok  = 0;
            for (int attempt = 0; attempt < 12; attempt++)
            {
                memcpy(A, G, sizeof(double) * P * P);
                memcpy(rhs_r, rhs, sizeof(double) * P);
                for (uint32_t i = 0; i < P; i++)
                {
                    A[i * P + i] += lam;
                }
                if (p_zoh_ridge)
                {
                    rhs_r[0] += lam;
                }
                if (!solve_spd_diag(P, A, rhs_r, w_sol))
                {
                    lam *= 4.0;
                    continue;
                }
                double n2     = 0.0;
                double dc_sum = 0.0;
                for (uint32_t k = 0; k < P; k++)
                {
                    n2     += w_sol[k] * w_sol[k];
                    dc_sum += w_sol[k];
                }
                ok = 1;
                if (n2 <= max_norm2 && fabs(dc_sum) <= 1.01)
                {
                    break;
                }
                lam *= 3.0;
            }
            if (!ok)
            {
                memset(w_sol, 0, sizeof(double) * P);
            }
        }

        double dc_sum = 0.0;
        for (uint32_t k = 0; k < P; k++)
        {
            dc_sum += w_sol[k];
        }
        if (fabs(dc_sum) > max_dc)
        {
            double sc = max_dc / fabs(dc_sum);
            for (uint32_t k = 0; k < P; k++)
            {
                w_sol[k] *= sc;
            }
        }

        double wr  = 0.0;
        double wGw = 0.0;
        for (uint32_t i = 0; i < P; i++)
        {
            wr += w_sol[i] * rhs[i];
            double gi = 0.0;
            for (uint32_t j = 0; j < P; j++)
            {
                gi += G[i * P + j] * w_sol[j];
            }
            wGw += w_sol[i] * gi;
        }
        double s_pred2 = s_y2 - 2.0 * wr + wGw;
        if (s_pred2 < 0.0)
        {
            s_pred2 = 0.0;
        }
        double inv_n = 1.0 / (double) nrows;
        double my    = s_y * inv_n;
        double v_ol  = s_y2 * inv_n - my * my;
        if (v_ol < 0.0)
        {
            v_ol = 0.0;
        }
        sum_var_ol   += v_ol;
        sum_var_zoh  += s_zoh2 * inv_n;
        sum_var_pred += s_pred2 * inv_n;

        uint32_t local_m = m % B;
        float   *row_out = &w_stage[(size_t) m * stride];
        uint32_t p_main  = (P_eof < P) ? P_eof : P;
        for (uint32_t k = 0; k < p_main; k++)
        {
            row_out[k * B + local_m] = (float) w_sol[k];
        }
        for (uint32_t k = p_main; k < P; k++)
        {
            row_out[pb_len + (k - p_main)] = (float) w_sol[k];
        }
    }

    *out_var_ol   = sum_var_ol;
    *out_var_zoh  = sum_var_zoh;
    *out_var_pred = sum_var_pred;
}

static void compute_chunk_descriptor(const float            *hist_fm,
                                     uint32_t                NBmode,
                                     uint32_t                eff_modes,
                                     uint32_t                n_valid,
                                     uint32_t                n_added,
                                     const EOFPF_PHYS_PRIOR *phys,
                                     const IMGID            *img_reg,
                                     double                  desc_out[EOFPF_REG_DIM])
{
    memset(desc_out, 0, sizeof(double) * EOFPF_REG_DIM);
    if (img_reg != NULL && img_reg->ID != -1 && img_reg->im != NULL)
    {
        uint32_t n = img_reg->md[0].size[0];
        if (n > EOFPF_REG_DIM)
        {
            n = EOFPF_REG_DIM;
        }
        for (uint32_t i = 0; i < n; i++)
        {
            double v    = (double) img_reg->im->array.F[i];
            desc_out[i] = isfinite(v) ? v : 0.0;
        }
        return;
    }

    uint32_t start = (n_valid > n_added) ? (n_valid - n_added) : 0u;
    uint32_t n     = (n_valid > start) ? (n_valid - start) : 0u;
    if (n < 2u || eff_modes == 0u)
    {
        return;
    }
    uint32_t m_b0  = (eff_modes < 32u) ? eff_modes : 32u;
    double   e_lo  = 0.0;
    double   e_mid = 0.0;
    double   e_hi  = 0.0;
    double   c0    = 0.0;
    double   c1    = 0.0;
    uint32_t m1    = eff_modes / 3u;
    uint32_t m2    = (2u * eff_modes) / 3u;
    if (m1 < 1u)
    {
        m1 = 1u;
    }
    if (m2 <= m1)
    {
        m2 = eff_modes;
    }
    for (uint32_t t = start; t < n_valid; t++)
    {
        const float *row = &hist_fm[(size_t) t * NBmode];
        for (uint32_t m = 0; m < eff_modes; m++)
        {
            double v2 = (double) row[m] * (double) row[m];
            if (m < m1)
            {
                e_lo += v2;
            }
            else if (m < m2)
            {
                e_mid += v2;
            }
            else
            {
                e_hi += v2;
            }
        }
        if (t > start)
        {
            const float *prv = &hist_fm[(size_t) (t - 1u) * NBmode];
            for (uint32_t m = 0; m < m_b0; m++)
            {
                c0 += (double) row[m] * (double) row[m];
                c1 += (double) row[m] * (double) prv[m];
            }
        }
    }
    double l_lo = log(sqrt(e_lo / (double) (n * m1)) + 1.0e-6);

    if (phys != NULL && phys->has_layers)
    {
        desc_out[0] = phys->vbar_x;
        desc_out[1] = phys->vbar_y;
        desc_out[2] = phys->v_std;
        desc_out[3] = l_lo;
        return;
    }

    double ac1   = (c0 > 1.0e-24) ? (c1 / c0) : 0.0;
    double l_mid = (m2 > m1) ? log(sqrt(e_mid / (double) (n * (m2 - m1))) + 1.0e-6) : l_lo;
    double l_hi  = (eff_modes > m2) ? log(sqrt(e_hi / (double) (n * (eff_modes - m2))) + 1.0e-6)
                                    : l_mid;
    desc_out[0]  = 10.0 * ac1;
    desc_out[1]  = l_lo;
    desc_out[2]  = l_mid;
    desc_out[3]  = l_hi;
}

static MILK_HOT errno_t mkEOFPF_compute(const MILK_RUNNABLE *self)
{
    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

    IMGID imgin = imgid_make_from_fpskey(p_inbuff, dcfpsname, ".inbuff");
    resolveIMGID(&imgin, ERRMODE_WARN, dcimg, dcnimg);
    if (imgin.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode = imgin.md[0].size[0];
    if (NBmode == 0)
    {
        return RETURN_FAILURE;
    }

    uint32_t P = p_pforder;
    if (P < 1)
    {
        P = 1;
    }
    if (P > EOFPF_MAX_ORDER)
    {
        P = EOFPF_MAX_ORDER;
    }

    uint32_t P_diag = p_diagorder;
    if (P_diag < P)
    {
        P_diag = P;
    }
    if (P_diag > EOFPF_MAX_ORDER)
    {
        P_diag = EOFPF_MAX_ORDER;
    }
    uint32_t diag_tail = P_diag - P;

    uint32_t B = p_blocksize;
    if (B < 1)
    {
        B = 1;
    }
    if (B > EOFPF_MAX_BLOCK)
    {
        B = EOFPF_MAX_BLOCK;
    }
    if (B > NBmode)
    {
        B = NBmode;
    }

    uint32_t cap_train = (p_nbtrain >= 128) ? p_nbtrain : 16384;
    if (cap_train > 65536)
    {
        cap_train = 65536;
    }

    uint32_t eff_modes_init = (p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;
    uint32_t stride         = P * B + diag_tail;
    IMGID    imgoutPF       = stream_connect_create_2Df32(p_outPF, stride, NBmode);

    uint32_t max_slice_frames = 1;
    if (imgin.md[0].naxis == 3 && imgin.md[0].size[2] == 2 && imgin.md[0].size[1] > 1)
    {
        max_slice_frames = imgin.md[0].size[1];
    }
    else if (imgin.md[0].naxis == 3)
    {
        max_slice_frames = imgin.md[0].size[2];
    }
    else if (imgin.md[0].naxis == 2)
    {
        max_slice_frames = imgin.md[0].size[1];
    }

    /* Optional Phase-5 modules: Pass-2 DM kernel, windlayers prior, and Gram library. */
    EOFPF_DM_KERN dm_kern;
    memset(&dm_kern, 0, sizeof(dm_kern));
    if (p_dm_kernel[0] != '\0')
    {
        (void) eofpf_dm_kern_load(p_dm_kernel, NBmode, &dm_kern);
    }

    EOFPF_PHYS_PRIOR phys;
    memset(&phys, 0, sizeof(phys));
    if (p_dmmodes[0] != '\0' && p_wl_layers[0] != '\0')
    {
        (void) eofpf_phys_init(&phys, p_dmmodes, eff_modes_init, B, P, p_pup_diam, p_dm_dx, p_l0_m);
    }

    IMGID img_wl = { .ID = -1 };
    if (p_wl_layers[0] != '\0')
    {
        img_wl = imgid_make_from_name(p_wl_layers);
        resolveIMGID(&img_wl, ERRMODE_NULL, dcimg, dcnimg);
        if (img_wl.ID == -1)
        {
            read_sharedmem_image(img_wl.name, dcimg, dcnimg);
            resolveIMGID(&img_wl, ERRMODE_NULL, dcimg, dcnimg);
        }
    }
    IMGID img_reg = { .ID = -1 };
    if (p_regime_in[0] != '\0')
    {
        img_reg = imgid_make_from_name(p_regime_in);
        resolveIMGID(&img_reg, ERRMODE_NULL, dcimg, dcnimg);
        if (img_reg.ID == -1)
        {
            read_sharedmem_image(img_reg.name, dcimg, dcnimg);
            resolveIMGID(&img_reg, ERRMODE_NULL, dcimg, dcnimg);
        }
    }

    int              use_gram = (p_gram_halflife > 0.0f) || (p_regime_kmax > 0u);
    EOFPF_GRAM_STATE gstate;
    memset(&gstate, 0, sizeof(gstate));
    if (use_gram)
    {
        uint32_t kmax = (p_regime_kmax > 1u) ? p_regime_kmax : 1u;
        if (eofpf_gram_init(&gstate, eff_modes_init, B, P, kmax) != 0)
        {
            use_gram = 0;
        }
    }

    // Frame-major history [cap_train x NBmode] + output staging buffer [NBmode x (P*B)]
    float *hist_fm   = (float *) calloc((size_t) cap_train * NBmode, sizeof(float));
    float *hist_fx   = (dm_kern.mode > 0)
                           ? (float *) calloc((size_t) cap_train * NBmode, sizeof(float))
                           : NULL;
    float *u_oos_buf = (dm_kern.mode > 0)
                           ? (float *) calloc((size_t) (max_slice_frames + EOFPF_MAX_DM_TAPS + 4u) *
                                                  NBmode,
                                              sizeof(float))
                           : NULL;
    float *y_oos_buf = (dm_kern.mode > 0)
                           ? (float *) calloc((size_t) (max_slice_frames + EOFPF_MAX_DM_TAPS + 4u) *
                                                  NBmode,
                                              sizeof(float))
                           : NULL;
    float *w_stage   = (float *) calloc((size_t) NBmode * stride, sizeof(float));
    float *tmp_in    = (float *) calloc((size_t) max_slice_frames * NBmode, sizeof(float));

    // Reusable double-precision BLAS/LAPACKE workspace for one block (max size D = P * B)
    size_t  D_max = (size_t) P * B;
    double *X_blk = (double *) malloc(sizeof(double) * (size_t) cap_train * D_max);
    double *Y_blk = (double *) malloc(sizeof(double) * (size_t) cap_train * B);
    double *G_blk = (double *) malloc(sizeof(double) * D_max * D_max);
    double *A_blk = (double *) malloc(sizeof(double) * D_max * D_max);
    double *R_blk = (double *) malloc(sizeof(double) * D_max * B);
    double *W_blk = (double *) malloc(sizeof(double) * D_max * B);
    double *GW    = (double *) malloc(sizeof(double) * D_max * B);

    if (!hist_fm || !w_stage || !tmp_in || !X_blk || !Y_blk || !G_blk || !A_blk || !R_blk ||
        !W_blk || !GW || (dm_kern.mode > 0 && (!hist_fx || !u_oos_buf || !y_oos_buf)))
    {
        eofpf_gram_free(&gstate);
        eofpf_phys_free(&phys);
        eofpf_dm_kern_free(&dm_kern);
        free(hist_fm);
        free(hist_fx);
        free(u_oos_buf);
        free(y_oos_buf);
        free(w_stage);
        free(tmp_in);
        free(X_blk);
        free(Y_blk);
        free(G_blk);
        free(A_blk);
        free(R_blk);
        free(W_blk);
        free(GW);
        return RETURN_FAILURE;
    }

    uint32_t n_valid       = 0;
    uint32_t bufs_since_ef = 0;
    uint64_t total_g_rows  = 0;
    uint64_t prev_wl_cnt   = 0;
    int      wl_synced     = 0;
    o_n_updates            = 0;
    o_n_eof_updates        = 0;
    o_eof_active           = 0;
    o_n_samples            = 0;
    o_eff_samples          = 0.0f;
    o_phys_active          = 0;
    o_fx_active            = (dm_kern.mode > 0) ? 1u : 0u;
    o_regime_k             = 0;
    o_wl_switches          = 0;
    o_rms_ol               = 0.0f;
    o_rms_zoh              = 0.0f;
    o_rms_pred             = 0.0f;
    o_var_red_pct          = 0.0f;
    o_offdiag_frac         = 0.0f;
    o_last_lam             = 0.0f;
    o_rms_zoh_oos          = 0.0f;
    o_rms_pred_oos         = 0.0f;
    o_var_red_oos          = 0.0f;
    o_overfit_ratio        = 0.0f;
    o_oos_rho_d_b0         = 0.0f;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        o_fx_active = (dm_kern.mode > 0) ? 1u : 0u;

        // 1. Snapshot incoming double-buffered PSOL telemetry slice
        uint32_t     nslice  = 1;
        const float *src_ptr = imgin.im->array.F;
        if (imgin.md[0].naxis == 3 && imgin.md[0].size[2] == 2 && imgin.md[0].size[1] > 1)
        {
            nslice      = imgin.md[0].size[1];
            uint32_t sl = (uint32_t) (imgin.md[0].cnt1 & 1ULL);
            src_ptr     = &imgin.im->array.F[(size_t) sl * nslice * NBmode];
        }
        else if (imgin.md[0].naxis == 3)
        {
            nslice = imgin.md[0].size[2];
        }
        else if (imgin.md[0].naxis == 2)
        {
            nslice = imgin.md[0].size[1];
        }
        if (nslice > max_slice_frames)
        {
            nslice = max_slice_frames;
        }
        memcpy(tmp_in, src_ptr, sizeof(float) * (size_t) NBmode * nslice);

        // Skip leading dead/zero frames before loop closure
        uint32_t first_nz = nslice;
        for (uint32_t t = 0; t < nslice; t++)
        {
            const float *row = &tmp_in[(size_t) t * NBmode];
            float        e2  = 0.0f;
            uint32_t     chk = (NBmode < 16) ? NBmode : 16;
            for (uint32_t m = 0; m < chk; m++)
            {
                e2 += row[m] * row[m];
            }
            if (e2 > 1.0e-24f)
            {
                first_nz = t;
                break;
            }
        }

        uint32_t n_added   = 0;
        uint32_t fx_t0     = (dm_kern.mode > 0 && dm_kern.n_taps > 1u) ? (dm_kern.n_taps - 1u) : 0u;
        if (first_nz < nslice)
        {
            uint32_t n_add = nslice - first_nz;
            if (n_add >= cap_train)
            {
                first_nz += (n_add - cap_train);
                n_add     = cap_train;
                n_valid   = 0;
            }
            else if (n_valid + n_add > cap_train)
            {
                uint32_t drop = (n_valid + n_add) - cap_train;
                uint32_t keep = n_valid - drop;
                memmove(hist_fm, &hist_fm[(size_t) drop * NBmode],
                        sizeof(float) * (size_t) keep * NBmode);
                if (hist_fx != NULL)
                {
                    memmove(hist_fx, &hist_fx[(size_t) drop * NBmode],
                            sizeof(float) * (size_t) keep * NBmode);
                }
                n_valid = keep;
            }
            uint32_t old_valid = n_valid;
            memcpy(&hist_fm[(size_t) n_valid * NBmode], &tmp_in[(size_t) first_nz * NBmode],
                   sizeof(float) * (size_t) n_add * NBmode);
            n_valid    += n_add;
            n_added     = n_add;
            o_n_samples = n_valid;
            if (hist_fx != NULL)
            {
                uint32_t c_start = (old_valid > fx_t0) ? old_valid : fx_t0;
                if (n_valid > c_start)
                {
                    eofpf_dm_kern_convolve(&dm_kern, hist_fm, n_valid, NBmode, c_start, n_valid,
                                           p_pflatency, hist_fx);
                }
            }
        }

        float    lat_fr    = (p_pflatency >= 0.05f && p_pflatency <= 24.0f) ? p_pflatency : 2.4f;
        uint32_t lat_i     = (uint32_t) lat_fr;
        double   lat_a     = (double) lat_fr - (double) lat_i;
        uint32_t eff_modes = (p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;
        if (use_gram && eff_modes > gstate.eff_modes)
        {
            eff_modes = gstate.eff_modes;
        }
        uint32_t upd_per = (p_updateperiod >= 1) ? p_updateperiod : 1;

        /* 1b. Poll live wl_layers SHM for windlayers physics-Gram prior & wind switches. */
        if (img_wl.ID == -1 && p_wl_layers[0] != '\0')
        {
            img_wl = imgid_make_from_name(p_wl_layers);
            resolveIMGID(&img_wl, ERRMODE_NULL, dcimg, dcnimg);
            if (img_wl.ID == -1)
            {
                read_sharedmem_image(img_wl.name, dcimg, dcnimg);
                resolveIMGID(&img_wl, ERRMODE_NULL, dcimg, dcnimg);
            }
        }
        int wind_switched_now = 0;
        if (img_wl.ID != -1 && img_wl.im != NULL)
        {
            uint64_t wl_c = img_wl.md[0].cnt0;
            if (!wl_synced || wl_c != prev_wl_cnt)
            {
                int      switched = 0;
                uint32_t w_nc     = img_wl.md[0].size[0];
                uint32_t w_nr     = (img_wl.md[0].naxis >= 2) ? img_wl.md[0].size[1] : 1u;
                float    lat_use  = (dm_kern.mode > 0) ? dm_kern.centroid : lat_fr;
                if (phys.ready)
                {
                    (void) eofpf_phys_update_layers(&phys, img_wl.im->array.F, w_nc, w_nr, lat_use,
                                                    p_frate_hz, &switched);
                }
                wl_synced   = 1;
                prev_wl_cnt = wl_c;
                if (switched)
                {
                    wind_switched_now = 1;
                    o_wl_switches++;
                    double disc = (double) p_wl_switch_decay;
                    if (disc > 0.0 && disc < 1.0)
                    {
                        if (use_gram)
                        {
                            eofpf_gram_discount_all(&gstate, disc);
                        }
                        else if (n_valid > P + lat_i + fx_t0 + 64u)
                        {
                            uint32_t keep = P + lat_i + fx_t0 + 64u;
                            uint32_t drop = n_valid - keep;
                            memmove(hist_fm, &hist_fm[(size_t) drop * NBmode],
                                    sizeof(float) * (size_t) keep * NBmode);
                            if (hist_fx != NULL)
                            {
                                memmove(hist_fx, &hist_fx[(size_t) drop * NBmode],
                                        sizeof(float) * (size_t) keep * NBmode);
                            }
                            n_valid     = keep;
                            o_n_samples = n_valid;
                        }
                    }
                }
            }
        }
        o_phys_active = (phys.has_layers && p_n_prior > 0.0f) ? 1u : 0u;

        double desc_now[EOFPF_REG_DIM];
        compute_chunk_descriptor(hist_fm, NBmode, eff_modes, n_valid, n_added, &phys, &img_reg,
                                 desc_now);

        uint32_t diag_req = (p_diagminsamples >= P_diag + lat_i + fx_t0 + 16u)
                                ? p_diagminsamples
                                : (P_diag + lat_i + fx_t0 + 16u);
        uint32_t eof_req  = (p_minsamples >= P + lat_i + fx_t0 + 64u)
                                ? p_minsamples
                                : (P + lat_i + fx_t0 + 64u);

        const float *hist_x = (dm_kern.mode > 0) ? hist_fx : hist_fm;
        int          use_fx = (dm_kern.mode > 0) ? 1 : 0;

        // Strictly out-of-sample evaluation of the EXISTING published filter on newly arrived
        // frames (runs on every fresh slice before any refit or updateperiod skip).
        if (o_n_updates > 0 && n_added > 1 && n_valid > P_diag + lat_i + fx_t0 + 1u)
        {
            uint32_t new_start = n_valid - n_added;
            uint32_t P_need    = (o_eof_active == 1 || diag_tail > 0) ? P_diag : P;
            uint32_t t_start   = 0;
            uint32_t t_end     = 0;
            if (use_fx)
            {
                t_start = (new_start > fx_t0) ? new_start : fx_t0;
                if (t_start < P_need - 1u + fx_t0)
                {
                    t_start = P_need - 1u + fx_t0;
                }
                t_end = n_valid;
            }
            else
            {
                t_start = (new_start > lat_i) ? (new_start - lat_i) : 0u;
                if (t_start < P_need - 1u)
                {
                    t_start = P_need - 1u;
                }
                t_end = n_valid - lat_i - 1u;
            }
            uint32_t n_oos = (t_end > t_start) ? (t_end - t_start) : 0u;
            if (n_oos >= 8u && use_fx)
            {
                /* Exact delivered OOS evaluation under Pass-2 dm_kernel:
                 *   u_pred[t]  = W * [phi[t], ..., phi[t-P+1]]
                 *   y_deliv[t] = (H_eff * u_pred)[t]
                 *   err[t]     = phi[t] - y_deliv[t]  vs  ZOH err = phi[t] - (H_eff * phi)[t] */
                uint32_t u_t0 = (t_start >= fx_t0) ? (t_start - fx_t0) : 0u;
                if (u_t0 < P_need - 1u)
                {
                    u_t0 = P_need - 1u;
                }
                uint32_t n_u = (t_end > u_t0) ? (t_end - u_t0) : 0u;
                uint32_t max_u = max_slice_frames + EOFPF_MAX_DM_TAPS + 4u;
                if (n_u > max_u)
                {
                    u_t0 += (n_u - max_u);
                    n_u   = max_u;
                }
                if (n_u > fx_t0 + 8u)
                {
                    const float *cur_pf  = imgoutPF.im->array.F;
                    uint32_t     pb_len  = P * B;
                    uint32_t     nblocks = (eff_modes + B - 1u) / B;
                    memset(u_oos_buf, 0, sizeof(float) * (size_t) n_u * NBmode);
                    for (uint32_t blk = 0; blk < nblocks; blk++)
                    {
                        uint32_t s0 = blk * B;
                        uint32_t b  = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
                        uint32_t D  = P * b;
                        for (uint32_t r = 0; r < n_u; r++)
                        {
                            uint32_t tk   = u_t0 + r;
                            double  *xrow = &X_blk[(size_t) r * D];
                            for (uint32_t k = 0; k < P; k++)
                            {
                                const float *hk = &hist_fm[(size_t) (tk - k) * NBmode + s0];
                                double      *xd = &xrow[(size_t) k * b];
                                for (uint32_t i = 0; i < b; i++)
                                {
                                    xd[i] = (double) hk[i];
                                }
                            }
                        }
                        for (uint32_t j = 0; j < b; j++)
                        {
                            const float *row_pf = &cur_pf[(size_t) (s0 + j) * stride];
                            for (uint32_t k = 0; k < P; k++)
                            {
                                const float *lag_pf = &row_pf[(size_t) k * B];
                                for (uint32_t i = 0; i < b; i++)
                                {
                                    W_blk[(size_t) (k * b + i) * b + j] = (double) lag_pf[i];
                                }
                            }
                        }
                        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) n_u, (int) b,
                                    (int) D, 1.0, X_blk, (int) D, W_blk, (int) b, 0.0, Y_blk,
                                    (int) b);
                        for (uint32_t r = 0; r < n_u; r++)
                        {
                            uint32_t tk = u_t0 + r;
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double up = Y_blk[(size_t) r * b + j];
                                if (diag_tail > 0u)
                                {
                                    const float *tail =
                                        &cur_pf[(size_t) (s0 + j) * stride + pb_len];
                                    for (uint32_t kt = 0; kt < diag_tail; kt++)
                                    {
                                        up += (double) tail[kt] *
                                              (double) hist_fm[(size_t) (tk - (P + kt)) * NBmode +
                                                               (s0 + j)];
                                    }
                                }
                                u_oos_buf[(size_t) r * NBmode + (s0 + j)] = (float) up;
                            }
                        }
                    }
                    eofpf_dm_kern_convolve(&dm_kern, u_oos_buf, n_u, NBmode, fx_t0, n_u, lat_fr,
                                           y_oos_buf);
                    uint32_t ev_n     = n_u - fx_t0;
                    double   oos_zoh2 = 0.0;
                    double   oos_prd2 = 0.0;
                    for (uint32_t r = fx_t0; r < n_u; r++)
                    {
                        uint32_t     tk  = u_t0 + r;
                        const float *phi = &hist_fm[(size_t) tk * NBmode];
                        const float *zoh = &hist_fx[(size_t) tk * NBmode];
                        const float *del = &y_oos_buf[(size_t) r * NBmode];
                        for (uint32_t m = 0; m < eff_modes; m++)
                        {
                            double dz  = (double) phi[m] - (double) zoh[m];
                            double err = (double) phi[m] - (double) del[m];
                            oos_zoh2  += dz * dz;
                            oos_prd2  += err * err;
                        }
                    }
                    double inv_ev   = 1.0 / (double) ev_n;
                    double vz       = oos_zoh2 * inv_ev;
                    double vp       = oos_prd2 * inv_ev;
                    o_rms_zoh_oos   = (float) sqrt(vz);
                    o_rms_pred_oos  = (float) sqrt(vp);
                    o_var_red_oos   = (vz > 1.0e-24) ? (float) ((1.0 - vp / vz) * 100.0) : 0.0f;
                    o_overfit_ratio = (o_rms_pred > 1.0e-12f) ? (o_rms_pred_oos / o_rms_pred)
                                                              : 1.0f;
                }
            }
            else if (n_oos >= 8u)
            {
                const float *cur_pf   = imgoutPF.im->array.F;
                uint32_t     pb_len   = P * B;
                double       oos_zoh2 = 0.0;
                double       oos_prd2 = 0.0;
                uint32_t     d_ceil   = (uint32_t) ceil((double) lat_fr - 1.0e-4);
                if (d_ceil < 1)
                {
                    d_ceil = 1;
                }
                double b0_rho_num = 0.0;
                double b0_rho_den = 0.0;

                if (o_eof_active == 2)
                {
                    uint32_t nblocks = (eff_modes + B - 1) / B;
                    for (uint32_t blk = 0; blk < nblocks; blk++)
                    {
                        uint32_t s0 = blk * B;
                        uint32_t b  = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
                        uint32_t D  = P * b;

                        // Assemble X_blk [n_oos x D] and unpack published W_blk [D x b]
#pragma omp parallel for schedule(static)
                        for (uint32_t r = 0; r < n_oos; r++)
                        {
                            uint32_t tk   = t_start + r;
                            double  *xrow = &X_blk[(size_t) r * D];
                            for (uint32_t k = 0; k < P; k++)
                            {
                                const float *hk = &hist_fm[(size_t) (tk - k) * NBmode + s0];
                                double      *xd = &xrow[(size_t) k * b];
                                for (uint32_t i = 0; i < b; i++)
                                {
                                    xd[i] = (double) hk[i];
                                }
                            }
                        }
                        for (uint32_t j = 0; j < b; j++)
                        {
                            const float *row_pf = &cur_pf[(size_t) (s0 + j) * stride];
                            for (uint32_t k = 0; k < P; k++)
                            {
                                const float *lag_pf = &row_pf[(size_t) k * B];
                                for (uint32_t i = 0; i < b; i++)
                                {
                                    W_blk[(size_t) (k * b + i) * b + j] = (double) lag_pf[i];
                                }
                            }
                        }
                        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, (int) n_oos,
                                    (int) b, (int) D, 1.0, X_blk, (int) D, W_blk, (int) b, 0.0,
                                    Y_blk, (int) b);

                        double blk_zoh = 0.0;
                        double blk_prd = 0.0;
#pragma omp parallel for schedule(static) reduction(+ : blk_zoh, blk_prd)
                        for (uint32_t r = 0; r < n_oos; r++)
                        {
                            uint32_t     tk  = t_start + r;
                            const float *h0  = &hist_fm[(size_t) tk * NBmode + s0];
                            const float *hf0 = &hist_fm[(size_t) (tk + lat_i) * NBmode + s0];
                            const float *hf1 = &hist_fm[(size_t) (tk + lat_i + 1) * NBmode + s0];
                            double      *pr  = &Y_blk[(size_t) r * b];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double pred = pr[j];
                                if (diag_tail > 0)
                                {
                                    const float *tail =
                                        &cur_pf[(size_t) (s0 + j) * stride + pb_len];
                                    for (uint32_t kt = 0; kt < diag_tail; kt++)
                                    {
                                        pred += (double) tail[kt] *
                                                (double) hist_fm[(size_t) (tk - (P + kt)) * NBmode +
                                                                 (s0 + j)];
                                    }
                                }
                                double yv  = (1.0 - lat_a) * (double) hf0[j] +
                                             lat_a * (double) hf1[j];
                                double dz  = yv - (double) h0[j];
                                double err = yv - pred;
                                pr[j]      = err;
                                blk_zoh   += dz * dz;
                                blk_prd   += err * err;
                            }
                        }
                        oos_zoh2 += blk_zoh;
                        oos_prd2 += blk_prd;

                        if (blk == 0 && n_oos > d_ceil + 4)
                        {
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double mu = 0.0;
                                for (uint32_t r = 0; r < n_oos; r++)
                                {
                                    mu += Y_blk[(size_t) r * b + j];
                                }
                                mu /= (double) n_oos;
                                double v0 = 0.0;
                                double vd = 0.0;
                                for (uint32_t r = 0; r < n_oos; r++)
                                {
                                    double er = Y_blk[(size_t) r * b + j] - mu;
                                    v0       += er * er;
                                    if (r >= d_ceil)
                                    {
                                        double ep = Y_blk[(size_t) (r - d_ceil) * b + j] - mu;
                                        vd       += er * ep;
                                    }
                                }
                                b0_rho_num += vd;
                                b0_rho_den += v0;
                            }
                        }
                    }
                }
                else
                {
                    uint32_t b0_lim = (B < eff_modes) ? B : eff_modes;
#pragma omp parallel for schedule(static) reduction(+ : oos_zoh2, oos_prd2, b0_rho_num, b0_rho_den)
                    for (uint32_t m = 0; m < eff_modes; m++)
                    {
                        uint32_t     local_m = m % B;
                        const float *row_pf  = &cur_pf[(size_t) m * stride];
                        double       w_loc[EOFPF_MAX_ORDER];
                        for (uint32_t k = 0; k < P; k++)
                        {
                            w_loc[k] = (double) row_pf[(size_t) k * B + local_m];
                        }
                        for (uint32_t k = P; k < P_diag; k++)
                        {
                            w_loc[k] = (double) row_pf[pb_len + (k - P)];
                        }

                        double mu = 0.0;
                        double e_prev[32];
                        uint32_t ring_sz = (d_ceil < 32) ? 32 : 0;
                        double   v0      = 0.0;
                        double   vd      = 0.0;
                        for (uint32_t r = 0; r < n_oos; r++)
                        {
                            uint32_t tk = t_start + r;
                            double y0 = (double) hist_fm[(size_t) (tk + lat_i) * NBmode + m];
                            double y1 = (double) hist_fm[(size_t) (tk + lat_i + 1) * NBmode + m];
                            double yv = (1.0 - lat_a) * y0 + lat_a * y1;
                            double h0   = (double) hist_fm[(size_t) tk * NBmode + m];
                            double pred = 0.0;
                            for (uint32_t k = 0; k < P_diag; k++)
                            {
                                pred += w_loc[k] * (double) hist_fm[(size_t) (tk - k) * NBmode + m];
                            }
                            double dz  = yv - h0;
                            double err = yv - pred;
                            oos_zoh2  += dz * dz;
                            oos_prd2  += err * err;
                            if (m < b0_lim && ring_sz > 0)
                            {
                                mu                       += err;
                                v0                       += err * err;
                                e_prev[r & 31u]           = err;
                                if (r >= d_ceil)
                                {
                                    vd += err * e_prev[(r - d_ceil) & 31u];
                                }
                            }
                        }
                        if (m < b0_lim && ring_sz > 0 && n_oos > d_ceil + 4)
                        {
                            double mean_e = mu / (double) n_oos;
                            double c0     = v0 - (double) n_oos * mean_e * mean_e;
                            double cd     = vd - (double) (n_oos - d_ceil) * mean_e * mean_e;
                            if (c0 > 0.0)
                            {
                                b0_rho_num += cd;
                                b0_rho_den += c0;
                            }
                        }
                    }
                }

                double inv_noos  = 1.0 / (double) n_oos;
                double v_zoh_oos = oos_zoh2 * inv_noos;
                double v_prd_oos = oos_prd2 * inv_noos;
                o_rms_zoh_oos    = (float) sqrt(v_zoh_oos);
                o_rms_pred_oos   = (float) sqrt(v_prd_oos);
                o_var_red_oos    = (v_zoh_oos > 1.0e-24)
                                       ? (float) ((1.0 - v_prd_oos / v_zoh_oos) * 100.0)
                                       : 0.0f;
                o_overfit_ratio  = (o_rms_pred > 1.0e-12f) ? (o_rms_pred_oos / o_rms_pred) : 1.0f;
                o_oos_rho_d_b0   = (b0_rho_den > 1.0e-24) ? (float) (b0_rho_num / b0_rho_den)
                                                          : 0.0f;
            }
        }

        /* 2. Incremental extended-memory / regime-weighted chunk-Gram accumulation. */
        if (use_gram && n_added > 0u)
        {
            uint32_t new_start = n_valid - n_added;
            uint32_t c_t0      = use_fx ? (fx_t0 + P - 1u) : (P - 1u);
            uint32_t c_t1      = use_fx ? n_valid : ((n_valid > lat_i + 1u) ? (n_valid - lat_i - 1u)
                                                                            : 0u);
            uint32_t inc_t0 = new_start;
            if (!use_fx)
            {
                inc_t0 = (new_start > lat_i + 1u) ? (new_start - lat_i - 1u) : 0u;
            }
            if (inc_t0 < c_t0)
            {
                inc_t0 = c_t0;
            }
            uint32_t n_chunk = (c_t1 > inc_t0) ? (c_t1 - inc_t0) : 0u;
            if (n_chunk >= 4u)
            {
                uint32_t anc = eofpf_gram_select_anchor(&gstate, desc_now, (double) p_regime_rlim);
                double   dec = (p_gram_halflife > 1.0f)
                                   ? pow(0.5, (double) n_chunk / (double) p_gram_halflife)
                                   : 1.0;
                if (gstate.k_used > 1u && dec < 0.999999)
                {
                    eofpf_gram_discount_all(&gstate, dec);
                }
                double   anc_dec = (gstate.k_used > 1u) ? 1.0 : dec;
                double   ch_y2   = 0.0;
                double   ch_zoh2 = 0.0;
                uint32_t nblocks = (eff_modes + B - 1u) / B;

                for (uint32_t blk = 0; blk < nblocks; blk++)
                {
                    uint32_t s0   = blk * B;
                    uint32_t b    = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
                    uint32_t D    = P * b;
                    double   by2  = 0.0;
                    double   bz2  = 0.0;
#pragma omp parallel for schedule(static) reduction(+ : by2, bz2)
                    for (uint32_t r = 0; r < n_chunk; r++)
                    {
                        uint32_t     tk   = inc_t0 + r;
                        const float *h0   = &hist_x[(size_t) tk * NBmode + s0];
                        double      *yrow = &Y_blk[(size_t) r * b];
                        if (use_fx)
                        {
                            const float *hy = &hist_fm[(size_t) tk * NBmode + s0];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double yv = (double) hy[j];
                                double dz = yv - (double) h0[j];
                                yrow[j]   = yv;
                                by2      += yv * yv;
                                bz2      += dz * dz;
                            }
                        }
                        else
                        {
                            const float *hf0 = &hist_fm[(size_t) (tk + lat_i) * NBmode + s0];
                            const float *hf1 = &hist_fm[(size_t) (tk + lat_i + 1u) * NBmode + s0];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double yv = (1.0 - lat_a) * (double) hf0[j] +
                                            lat_a * (double) hf1[j];
                                double dz = yv - (double) h0[j];
                                yrow[j]   = yv;
                                by2      += yv * yv;
                                bz2      += dz * dz;
                            }
                        }
                        double *xrow = &X_blk[(size_t) r * D];
                        for (uint32_t k = 0; k < P; k++)
                        {
                            const float *hk = &hist_x[(size_t) (tk - k) * NBmode + s0];
                            double      *xd = &xrow[(size_t) k * b];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                xd[j] = (double) hk[j];
                            }
                        }
                    }
                    ch_y2   += by2;
                    ch_zoh2 += bz2;

                    cblas_dsyrk(CblasRowMajor, CblasLower, CblasTrans, (int) D, (int) n_chunk, 1.0,
                                X_blk, (int) D, 0.0, G_blk, (int) D);
                    for (uint32_t i = 0; i < D; i++)
                    {
                        for (uint32_t j = 0; j < i; j++)
                        {
                            G_blk[(size_t) j * D + i] = G_blk[(size_t) i * D + j];
                        }
                    }
                    cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, (int) D, (int) b,
                                (int) n_chunk, 1.0, X_blk, (int) D, Y_blk, (int) b, 0.0, R_blk,
                                (int) b);
                    eofpf_gram_accum_block(&gstate, anc, blk, G_blk, R_blk, anc_dec);
                }

                gstate.w_eff[anc]    = anc_dec * gstate.w_eff[anc] + (double) n_chunk;
                gstate.y2_sum[anc]   = anc_dec * gstate.y2_sum[anc] + ch_y2;
                gstate.zoh2_sum[anc] = anc_dec * gstate.zoh2_sum[anc] + ch_zoh2;
                total_g_rows        += n_chunk;
                o_regime_k           = gstate.k_used;
            }
        }

        uint64_t avail_samples = use_gram ? total_g_rows : (uint64_t) n_valid;
        uint32_t phys_min_req  = P + lat_i + fx_t0 + 64u;
        uint32_t eof_need      = o_phys_active ? phys_min_req : eof_req;
        int      do_eof        = (avail_samples >= (uint64_t) eof_need) &&
                                 (n_valid > P + lat_i + fx_t0 + 16u) && (B > 1u);
        int      do_diag       = (!do_eof) && (n_valid >= diag_req) &&
                                 (n_valid > P_diag + lat_i + fx_t0 + 4u);

        if (do_eof)
        {
            uint32_t eff_upd = (o_n_eof_updates < 3 && upd_per > 2) ? 2 : upd_per;
            bufs_since_ef++;
            if (!wind_switched_now && o_n_eof_updates > 0 && bufs_since_ef < eff_upd)
            {
                continue;
            }
            bufs_since_ef = 0;

            uint32_t t0_idx = use_fx ? (fx_t0 + P - 1u) : (P - 1u);
            uint32_t t1_idx = use_fx ? n_valid : (n_valid - lat_i - 1u);
            uint32_t nrows  = (t1_idx > t0_idx) ? (t1_idx - t0_idx) : 0u;
            if (!use_gram && nrows < 32u)
            {
                continue;
            }

            double reg_init  = (p_reglambda > 1.0e-9f) ? (double) p_reglambda : 1.0e-4;
            double max_norm2 = (p_maxnorm2 > 0.1f) ? (double) p_maxnorm2 : 2.0;
            double max_dc    = (p_maxdcgain > 0.1f && p_maxdcgain < 0.9999f)
                                   ? (double) p_maxdcgain
                                   : 0.995;

            double sum_var_ol   = 0.0;
            double sum_var_zoh  = 0.0;
            double sum_var_pred = 0.0;
            double sum_e_tot    = 0.0;
            double sum_e_off    = 0.0;
            double w_eff_b0     = (double) nrows;

            memset(w_stage, 0, sizeof(float) * (size_t) NBmode * stride);

            uint32_t nblocks = (eff_modes + B - 1u) / B;
            for (uint32_t blk = 0; blk < nblocks; blk++)
            {
                uint32_t s0 = blk * B;
                uint32_t b  = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
                uint32_t D  = P * b;

                double b_y2   = 0.0;
                double b_zoh2 = 0.0;
                double b_sum_y[EOFPF_MAX_BLOCK];
                memset(b_sum_y, 0, sizeof(double) * b);
                double w_emp = (double) nrows;

                if (use_gram)
                {
                    eofpf_gram_blend_block(&gstate, blk, desc_now, (double) p_regime_kappa,
                                           (double) p_regime_bw, G_blk, R_blk, &w_emp);
                    if (blk == 0u)
                    {
                        w_eff_b0 = w_emp;
                        double w_sum_y2 = 0.0;
                        double w_sum_z2 = 0.0;
                        double inv2bw = 0.0;
                        if (p_regime_bw > 1.0e-6f)
                        {
                            inv2bw = 0.5 / ((double) p_regime_bw * (double) p_regime_bw);
                        }
                        for (uint32_t k = 0; k < gstate.k_used; k++)
                        {
                            double wk = 1.0;
                            if (gstate.k_used > 1u && inv2bw > 0.0)
                            {
                                double d2 = 0.0;
                                for (int qi = 0; qi < EOFPF_REG_DIM; qi++)
                                {
                                    double dq = desc_now[qi] - gstate.desc[k][qi];
                                    d2       += dq * dq;
                                }
                                wk = (double) p_regime_kappa +
                                     (1.0 - (double) p_regime_kappa) * exp(-d2 * inv2bw);
                            }
                            w_sum_y2 += wk * gstate.y2_sum[k];
                            w_sum_z2 += wk * gstate.zoh2_sum[k];
                        }
                        if (w_emp > 1.0)
                        {
                            sum_var_ol  = w_sum_y2 / w_emp;
                            sum_var_zoh = w_sum_z2 / w_emp;
                        }
                    }
                    for (uint32_t j = 0; j < b; j++)
                    {
                        b_y2 += G_blk[(size_t) j * D + j];
                    }
                }
                else
                {
                    // Assemble X_blk [nrows x D] and Y_blk [nrows x b] (row-major, parallelized)
#pragma omp parallel for schedule(static) reduction(+ : b_y2, b_zoh2)
                    for (uint32_t r = 0; r < nrows; r++)
                    {
                        uint32_t     tk   = t0_idx + r;
                        const float *h0   = &hist_x[(size_t) tk * NBmode + s0];
                        double      *yrow = &Y_blk[(size_t) r * b];
                        if (use_fx)
                        {
                            const float *hy = &hist_fm[(size_t) tk * NBmode + s0];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double yv = (double) hy[j];
                                double dz = yv - (double) h0[j];
                                yrow[j]   = yv;
                                b_y2     += yv * yv;
                                b_zoh2   += dz * dz;
                            }
                        }
                        else
                        {
                            const float *hf0 = &hist_fm[(size_t) (tk + lat_i) * NBmode + s0];
                            const float *hf1 = &hist_fm[(size_t) (tk + lat_i + 1u) * NBmode + s0];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                double yv = (1.0 - lat_a) * (double) hf0[j] +
                                            lat_a * (double) hf1[j];
                                double dz = yv - (double) h0[j];
                                yrow[j]   = yv;
                                b_y2     += yv * yv;
                                b_zoh2   += dz * dz;
                            }
                        }
                        double *xrow = &X_blk[(size_t) r * D];
                        for (uint32_t k = 0; k < P; k++)
                        {
                            const float *hk = &hist_x[(size_t) (tk - k) * NBmode + s0];
                            double      *xd = &xrow[(size_t) k * b];
                            for (uint32_t j = 0; j < b; j++)
                            {
                                xd[j] = (double) hk[j];
                            }
                        }
                    }
                    for (uint32_t r = 0; r < nrows; r++)
                    {
                        const double *yrow = &Y_blk[(size_t) r * b];
                        for (uint32_t j = 0; j < b; j++)
                        {
                            b_sum_y[j] += yrow[j];
                        }
                    }

                    // G_blk = X^T X (lower triangle) and R_blk = X^T Y
                    cblas_dsyrk(CblasRowMajor, CblasLower, CblasTrans, (int) D, (int) nrows, 1.0,
                                X_blk, (int) D, 0.0, G_blk, (int) D);
                    cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, (int) D, (int) b,
                                (int) nrows, 1.0, X_blk, (int) D, Y_blk, (int) b, 0.0, R_blk,
                                (int) b);
                    for (uint32_t i = 0; i < D; i++)
                    {
                        for (uint32_t j = 0; j < i; j++)
                        {
                            G_blk[(size_t) j * D + i] = G_blk[(size_t) i * D + j];
                        }
                    }
                }

                /* Blend windlayers Bayesian physics-Gram prior when active. */
                double prior_y2 = 0.0;
                double w_prior  = 0.0;
                if (o_phys_active && blk < phys.nblocks)
                {
                    eofpf_phys_blend_block(&phys, blk, G_blk, R_blk, w_emp, (double) p_n_prior,
                                           G_blk, R_blk, &prior_y2);
                    w_prior = (double) p_n_prior;
                    if (blk == 0u)
                    {
                        w_eff_b0 = w_emp + w_prior;
                    }
                }

                // Compute mean diagonal trace of final (possibly blended) G_blk
                double tr = 0.0;
                for (uint32_t i = 0; i < D; i++)
                {
                    tr += G_blk[(size_t) i * D + i];
                }
                tr /= (double) D;

                double lam_rel_used = reg_init;
                int    solved       = 0;
                double n2_col[EOFPF_MAX_BLOCK];
                double n2_sorted[EOFPF_MAX_BLOCK];

                if (tr > 1.0e-26)
                {
                    double lam_rel = reg_init;
                    for (int attempt = 0; attempt < 12; attempt++)
                    {
                        memcpy(A_blk, G_blk, sizeof(double) * (size_t) D * D);
                        double lam_abs = lam_rel * tr;
                        for (uint32_t i = 0; i < D; i++)
                        {
                            A_blk[(size_t) i * D + i] += lam_abs;
                        }
                        memcpy(W_blk, R_blk, sizeof(double) * (size_t) D * b);
                        if (p_zoh_ridge)
                        {
                            for (uint32_t j = 0; j < b; j++)
                            {
                                W_blk[(size_t) j * b + j] += lam_abs;
                            }
                        }

                        lapack_int info =
                            LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'L', (lapack_int) D, A_blk,
                                           (lapack_int) D);
                        if (info != 0)
                        {
                            lam_rel *= 4.0;
                            continue;
                        }
                        info = LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'L', (lapack_int) D,
                                              (lapack_int) b, A_blk, (lapack_int) D, W_blk,
                                              (lapack_int) b);
                        if (info != 0)
                        {
                            lam_rel *= 4.0;
                            continue;
                        }

                        for (uint32_t j = 0; j < b; j++)
                        {
                            double n2 = 0.0;
                            for (uint32_t d = 0; d < D; d++)
                            {
                                double wv = W_blk[(size_t) d * b + j];
                                n2 += wv * wv;
                            }
                            n2_col[j]    = n2;
                            n2_sorted[j] = n2;
                        }
                        qsort(n2_sorted, b, sizeof(double), cmp_double_asc);
                        uint32_t idx95 = (b > 1) ? (uint32_t) (0.95 * (double) (b - 1)) : 0;
                        double   p95   = n2_sorted[idx95];
                        double   mx    = n2_sorted[b - 1];

                        solved       = 1;
                        lam_rel_used = lam_rel;
                        if (p95 <= max_norm2 && mx <= 1.75 * max_norm2)
                        {
                            break;
                        }
                        lam_rel *= 2.5;
                    }
                }
                if (blk == 0)
                {
                    o_last_lam = (float) lam_rel_used;
                }
                if (!solved)
                {
                    memset(W_blk, 0, sizeof(double) * (size_t) D * b);
                }

                // Per-mode outlier norm cap + gentle self-DC cap
                double hard_n2 = 1.75 * max_norm2;
                for (uint32_t j = 0; j < b; j++)
                {
                    double n2 = 0.0;
                    for (uint32_t d = 0; d < D; d++)
                    {
                        double wv = W_blk[(size_t) d * b + j];
                        n2 += wv * wv;
                    }
                    if (n2 > hard_n2)
                    {
                        double sc = sqrt(hard_n2 / n2);
                        for (uint32_t d = 0; d < D; d++)
                        {
                            W_blk[(size_t) d * b + j] *= sc;
                        }
                    }
                    double dc_self = 0.0;
                    for (uint32_t k = 0; k < P; k++)
                    {
                        dc_self += W_blk[(size_t) (k * b + j) * b + j];
                    }
                    if (fabs(dc_self) > max_dc)
                    {
                        double sc = max_dc / fabs(dc_self);
                        for (uint32_t d = 0; d < D; d++)
                        {
                            W_blk[(size_t) d * b + j] *= sc;
                        }
                    }
                }

                // Exact training-window residual: ||Y||_F^2 - 2 <W, R> + <W, G W>
                cblas_dsymm(CblasRowMajor, CblasLeft, CblasLower, (int) D, (int) b, 1.0, G_blk,
                            (int) D, W_blk, (int) b, 0.0, GW, (int) b);
                double wr  = 0.0;
                double wGw = 0.0;
                for (size_t idx = 0; idx < (size_t) D * b; idx++)
                {
                    wr  += W_blk[idx] * R_blk[idx];
                    wGw += W_blk[idx] * GW[idx];
                }
                double b_pred2 = (b_y2 + prior_y2) - 2.0 * wr + wGw;
                if (b_pred2 < 0.0)
                {
                    b_pred2 = 0.0;
                }

                double w_den      = use_gram ? ((w_emp > 1.0) ? w_emp : 1.0) : (double) nrows;
                double inv_n      = 1.0 / w_den;
                double inv_n_post = 1.0 / (w_den + w_prior);
                if (!use_gram)
                {
                    double b_mean = 0.0;
                    for (uint32_t j = 0; j < b; j++)
                    {
                        double mj = b_sum_y[j] * inv_n;
                        b_mean   += mj * mj;
                    }
                    double b_vol = b_y2 * inv_n - b_mean;
                    if (b_vol < 0.0)
                    {
                        b_vol = 0.0;
                    }
                    sum_var_ol   += b_vol;
                    sum_var_zoh  += b_zoh2 * inv_n;
                }
                sum_var_pred += b_pred2 * inv_n_post;

                // Pack block matrix into row-major [NBmode x (P*B)] staging array
                for (uint32_t j = 0; j < b; j++)
                {
                    float *row_out = &w_stage[(size_t) (s0 + j) * stride];
                    for (uint32_t k = 0; k < P; k++)
                    {
                        float *lag_out = &row_out[(size_t) k * B];
                        for (uint32_t i = 0; i < b; i++)
                        {
                            double wv  = W_blk[(size_t) (k * b + i) * b + j];
                            lag_out[i] = (float) wv;
                            sum_e_tot += wv * wv;
                            if (i != j)
                            {
                                sum_e_off += wv * wv;
                            }
                        }
                    }
                }
            }

            // Smooth cross-fade from diag AR(P_diag) bootstrap into cross-mode EOF
            // (immediate full publish on first update or detected wind switch).
            float alpha = (o_n_updates == 0 || wind_switched_now) ? 1.0f : p_loopgain;
            if (alpha <= 0.0f || alpha > 1.0f)
            {
                alpha = 0.5f;
            }
            float  beta   = 1.0f - alpha;
            size_t n_elem = (size_t) NBmode * stride;

            imgoutPF.md[0].write = 1;
            float *dst           = imgoutPF.im->array.F;
            if (alpha >= 0.9999f)
            {
                memcpy(dst, w_stage, sizeof(float) * n_elem);
            }
            else
            {
#pragma omp parallel for schedule(static)
                for (size_t idx = 0; idx < n_elem; idx++)
                {
                    dst[idx] = beta * dst[idx] + alpha * w_stage[idx];
                }
            }
            processinfo_update_output_stream(processinfo, imgoutPF.im, NULL);

            o_n_updates++;
            o_n_eof_updates++;
            o_eof_active   = 2;
            o_eff_samples  = (float) w_eff_b0;
            o_rms_ol       = (float) sqrt(sum_var_ol);
            o_rms_zoh      = (float) sqrt(sum_var_zoh);
            o_rms_pred     = (float) sqrt(sum_var_pred);
            o_var_red_pct  = (sum_var_zoh > 1.0e-24)
                                 ? (float) ((1.0 - sum_var_pred / sum_var_zoh) * 100.0)
                                 : 0.0f;
            o_offdiag_frac = (sum_e_tot > 1.0e-24) ? (float) (sum_e_off / sum_e_tot) : 0.0f;
        }
        else if (do_diag)
        {
            double sum_var_ol   = 0.0;
            double sum_var_zoh  = 0.0;
            double sum_var_pred = 0.0;
            solve_diag_bootstrap(hist_x, hist_fm, use_fx, fx_t0, NBmode, eff_modes, n_valid, P,
                                 P_diag, B, stride, lat_i, lat_a, w_stage, &sum_var_ol,
                                 &sum_var_zoh, &sum_var_pred);

            float alpha = (o_n_updates == 0) ? 1.0f : 0.30f;
            float beta  = 1.0f - alpha;
            size_t n_elem = (size_t) NBmode * stride;

            imgoutPF.md[0].write = 1;
            float *dst           = imgoutPF.im->array.F;
            if (alpha >= 0.9999f)
            {
                memcpy(dst, w_stage, sizeof(float) * n_elem);
            }
            else
            {
#pragma omp parallel for schedule(static)
                for (size_t idx = 0; idx < n_elem; idx++)
                {
                    dst[idx] = beta * dst[idx] + alpha * w_stage[idx];
                }
            }
            processinfo_update_output_stream(processinfo, imgoutPF.im, NULL);

            o_n_updates++;
            o_eof_active   = 1;
            o_eff_samples  = (float) n_valid;
            o_rms_ol       = (float) sqrt(sum_var_ol);
            o_rms_zoh      = (float) sqrt(sum_var_zoh);
            o_rms_pred     = (float) sqrt(sum_var_pred);
            o_var_red_pct  = (sum_var_zoh > 1.0e-24)
                                 ? (float) ((1.0 - sum_var_pred / sum_var_zoh) * 100.0)
                                 : 0.0f;
            o_offdiag_frac = 0.0f;
        }
    }
    MILK_PROCINFO_END(self)

    if (img_wl.ID != -1)
    {
        imgid_free(&img_wl);
    }
    if (img_reg.ID != -1)
    {
        imgid_free(&img_reg);
    }
    eofpf_gram_free(&gstate);
    eofpf_phys_free(&phys);
    eofpf_dm_kern_free(&dm_kern);
    free(GW);
    free(W_blk);
    free(R_blk);
    free(A_blk);
    free(G_blk);
    free(Y_blk);
    free(X_blk);
    free(tmp_in);
    free(w_stage);
    free(y_oos_buf);
    free(u_oos_buf);
    free(hist_fx);
    free(hist_fm);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_mkEOFPF = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "mkEOFPF",
                 .cmdkey      = "mkEOFPF",
                 .description = "build online block cross-mode EOF predictive filter",
                 .description_long =
                     "Accumulate pseudo-open-loop (PSOL) modal telemetry and solve regularized "
                     "block cross-mode Empirical Orthogonal Functions (VAR(P)) transition "
                     "matrices with progressive diagonal AR cold-start bootstrapping." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaopc_mkEOFPF),
    .compute = mkEOFPF_compute,
};
EXPORT_FPSEXEC(cacao, "cacaopc-mkEOFPF", runnable_AOloopControl_PredictiveControl_mkEOFPF);
