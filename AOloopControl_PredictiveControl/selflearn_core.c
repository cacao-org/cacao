// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    selflearn_core.c
 * @brief   Online GRIC state-space clustering + 2-level hierarchical factorized RLS engine
 *          with closed-loop stability guarantees.
 *
 * Key closed-loop stability invariants:
 *   1. Zero-DC increment regressor:
 *        phi_{t,m} = [dOL_m(t), dOL_m(t-1), dOL_m(t-2), dOL_m(t-3),
 *                     w_m(t), dOL_0(t)..dOL_{q-1}(t)]
 *      All dOL taps vanish identically at DC, so the closed-loop DC gain is strictly pinned
 *      to `dc_leak < 1.0` (standalone PF) or inherits `in_basePF`'s DC leak (cascaded PF).
 *      No unconstrained intercept can ever integrate into the DM rail.
 *   2. Explicit physical stability bounds on learned weights:
 *      - AR increment filter L2 norm^2 capped at `max_ar_norm2` (prevents Nyquist ringing).
 *      - Physical WFS optical-gain weight `w_phys` bounded to `[-0.35, +max_wfs_gain]`
 *        (prevents loop-gain overshoot even when optical gain varies by 3-5x).
 *   3. 2-level hierarchical GRIC (libgric) + soft LWPR blending + causal OOS Stein gates:
 *      - Level 0 tracks global linear dynamics & optical gain with 100% of frames.
 *      - Level 1 learns only state-dependent deviations per GRIC cluster, shrunk toward 0
 *        and projected by the causal OOS Stein gate `alpha_l1 in [0, 1]`.
 */

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "AOloopControl_PredictiveControl/selflearn_core.h"

struct SL_ENGINE
{
    SL_CONFIG cfg;
    uint32_t  M;
    uint32_t  key_dim;
    uint32_t  p_feat;   /* SL_AR_DIFF_TAPS + 1 + n_nl + q_sh + n_wsh */
    uint32_t  q_sh;
    uint32_t  n_nl;     /* 0 (nl_mode=0) or SL_NL_WFS_TAPS (nl_mode>=1) */
    uint32_t  n_wsh;    /* 0 (nl_mode<2) or q_sh (nl_mode>=2) */
    double    last_l_lo; /* Latest rolling log low-order WFS energy from extract_key */

    /* Online causal standardization stats */
    uint64_t stat_n;
    double  *w_mean;
    double  *w_m2;
    double  *w_sd;
    double  *dol_mean;
    double  *dol_m2;
    double  *dol_sd;
    double  *dm_mean;
    double  *dm_m2;
    double  *dm_sd;
    double   key_mean[SL_MAX_KEY_DIM];
    double   key_m2[SL_MAX_KEY_DIM];
    double   key_sd[SL_MAX_KEY_DIM];

    /* Rolling energy / velocity buffers for key extraction */
    double   e_lo_ring[SL_ENERGY_WIN];
    double   e_hi_ring[SL_ENERGY_WIN];
    double   e_tot_ring[SL_ENERGY_WIN];
    double   e_vel_ring[SL_VEL_WIN];
    uint32_t e_pos;
    uint32_t vel_pos;
    double   e_lo_sum;
    double   e_hi_sum;
    double   e_tot_sum;
    double   e_vel_sum;

    /* Lagged PSOL & dOL history buffers for zero-DC AR(4) increment features */
    float *ol_prev;
    float *dol_hist; /* [SL_AR_DIFF_TAPS * M] circular history */
    uint32_t dol_head;
    uint32_t dol_count;

    /* GRIC streaming cluster engine + cached anchors */
#ifdef HAVE_LIBGRIC
    gric_cluster_t *gric_ctx;
#endif
    uint32_t k_used;
    double   anchors[SL_MAX_CLUSTERS * SL_MAX_KEY_DIM];

    /* Level-0 global exponentially-weighted normal equations (G0 + lam0 I) w0 = b0 */
    double *G0;
    double *b0;
    double *w0;
    double  n0_eff;

    /* Level-1 per-cluster exponentially-weighted normal equations (G1 + lam1 I) w1 = b1 */
    double *G1;
    double *b1;
    double *w1;
    double  n1_eff[SL_MAX_CLUSTERS];

    /* Causal OOS Stein gates & running OOS MSE accumulators */
    double c0_ey;
    double c0_yy;
    double c1_ey;
    double c1_yy;
    double oos_e_base2;
    double oos_e_l0_2;
    double oos_e_l1_2;

    /* Strictly causal delayed-target ring buffer */
    uint32_t ring_head;
    uint64_t ring_count;
    float   *ring_phi;
    float   *ring_y0_fit;
    float   *ring_y0_act;
    float   *ring_y0_raw;
    float   *ring_dy1_raw;
    float   *ring_base;
    float   *ring_ol;
    float   *ring_wfs;
    float   *ring_tweak;
    int32_t  ring_cid[SL_RING_SIZE];
    uint8_t  ring_valid[SL_RING_SIZE];

    /* Scratch buffers */
    float  *cur_phi;
    float  *cur_dol;
    float  *cur_y0_fit;
    float  *cur_y0_act;
    float  *cur_y0_raw;
    float  *cur_dy1_raw;
    float  *cur_base;
    double *blend_w;

    SL_DIAG diag;
};

/* Bit-level finiteness checks remain valid even when compiled under -Ofast. */
static inline int sl_isfinite_d(double x)
{
    uint64_t u;
    memcpy(&u, &x, sizeof(u));
    return ((u >> 52) & 0x7FFULL) != 0x7FFULL;
}

static inline int sl_isfinite_f(float x)
{
    uint32_t u;
    memcpy(&u, &x, sizeof(u));
    return ((u >> 23) & 0xFFu) != 0xFFu;
}

static uint32_t resolve_key_dim(uint32_t key_mode, uint32_t ext_dim)
{
    switch (key_mode)
    {
        case SL_KEY_TT2_E2:
            return 4;
        case SL_KEY_E1_LOGE:
            return 1;
        case SL_KEY_WFS_LO4:
            return 4;
        case SL_KEY_TT2_DM2_E2:
            return 6;
        case SL_KEY_REGIME_E2_VEL:
            return 3;
        case SL_KEY_EXTERNAL:
            return (ext_dim >= 1 && ext_dim <= SL_MAX_KEY_DIM) ? ext_dim : 4;
        default:
            return 4;
    }
}

static void sl_create_gric(SL_ENGINE *eng)
{
#ifdef HAVE_LIBGRIC
    if (eng->gric_ctx != NULL)
    {
        gric_cluster_destroy(eng->gric_ctx);
        eng->gric_ctx = NULL;
    }
    gric_cluster_config_t gcfg;
    gric_cluster_config_default(&gcfg);
    gcfg.rlim             = eng->cfg.rlim;
    gcfg.maxnbclust       = (int) eng->cfg.k_max;
    gcfg.maxnbfr          = 65536;
    gcfg.tm_mixing_coeff  = eng->cfg.tm_mix;
    gcfg.use_double       = 1;
    gcfg.maxcl_strategy   = 0;
    gcfg.ncpu             = 1;
    eng->gric_ctx         = gric_cluster_create(&gcfg, (size_t) eng->key_dim);
    eng->diag.gric_active = (eng->gric_ctx != NULL) ? 1u : 0u;
#else
    eng->diag.gric_active = 0u;
#endif
}

SL_ENGINE *sl_engine_create(const SL_CONFIG *cfg_in)
{
    if (cfg_in == NULL || cfg_in->n_modes == 0)
    {
        return NULL;
    }

    SL_ENGINE *eng = (SL_ENGINE *) calloc(1, sizeof(SL_ENGINE));
    if (eng == NULL)
    {
        return NULL;
    }

    eng->cfg = *cfg_in;
    if (eng->cfg.k_max < 1u) eng->cfg.k_max = 1u;
    if (eng->cfg.k_max > SL_MAX_CLUSTERS) eng->cfg.k_max = SL_MAX_CLUSTERS;
    if (eng->cfg.q_shared > SL_MAX_QSHARED) eng->cfg.q_shared = SL_MAX_QSHARED;
    if (eng->cfg.rlim < 0.05) eng->cfg.rlim = 0.05;
    if (eng->cfg.gate_mult < 0.5) eng->cfg.gate_mult = 1.0;
    if (eng->cfg.blend_bw < 0.1) eng->cfg.blend_bw = 1.0;
    if (eng->cfg.latency_fr < 1.0) eng->cfg.latency_fr = 1.0;
    if (eng->cfg.latency_fr > (double) (SL_RING_SIZE - 4))
    {
        eng->cfg.latency_fr = (double) (SL_RING_SIZE - 4);
    }
    if (eng->cfg.dc_leak <= 0.5 || eng->cfg.dc_leak >= 1.0)
    {
        eng->cfg.dc_leak = 0.985;
    }
    if (eng->cfg.aux_leak < 0.0 || eng->cfg.aux_leak >= 0.98)
    {
        eng->cfg.aux_leak = 0.75;
    }
    if (eng->cfg.max_wfs_gain < 0.0)
    {
        eng->cfg.max_wfs_gain = 0.5;
    }
    if (eng->cfg.max_ar_norm2 <= 0.0)
    {
        eng->cfg.max_ar_norm2 = 2.0;
    }
    if (eng->cfg.lam0 < 1.0e-3) eng->cfg.lam0 = 20.0;
    if (eng->cfg.lam1 < 1.0e-3) eng->cfg.lam1 = 200.0;
    if (eng->cfg.gamma0 < 0.9 || eng->cfg.gamma0 > 1.0) eng->cfg.gamma0 = 0.998;
    if (eng->cfg.gamma1 < 0.9 || eng->cfg.gamma1 > 1.0) eng->cfg.gamma1 = 0.995;
    if (eng->cfg.gate_eta < 0.9 || eng->cfg.gate_eta >= 1.0) eng->cfg.gate_eta = 0.995;
    if (eng->cfg.outlier_sig < 3.0) eng->cfg.outlier_sig = 12.0;

    eng->M       = eng->cfg.n_modes;
    eng->q_sh    = (eng->cfg.q_shared <= eng->M) ? eng->cfg.q_shared : eng->M;
    eng->n_nl    = (eng->cfg.nl_mode >= 1u) ? SL_NL_WFS_TAPS : 0u;
    eng->n_wsh   = (eng->cfg.nl_mode >= 2u) ? eng->q_sh : 0u;
    eng->p_feat  = SL_AR_DIFF_TAPS + 1u + eng->n_nl + eng->q_sh + eng->n_wsh;
    eng->key_dim = resolve_key_dim(eng->cfg.key_mode, eng->cfg.ext_key_dim);

    uint32_t M    = eng->M;
    uint32_t p    = eng->p_feat;
    uint32_t Kmax = eng->cfg.k_max;
    size_t   mp   = (size_t) M * p;
    size_t   mpp  = (size_t) M * p * p;

    eng->w_mean       = (double *) calloc(M, sizeof(double));
    eng->w_m2         = (double *) calloc(M, sizeof(double));
    eng->w_sd         = (double *) calloc(M, sizeof(double));
    eng->dol_mean     = (double *) calloc(M, sizeof(double));
    eng->dol_m2       = (double *) calloc(M, sizeof(double));
    eng->dol_sd       = (double *) calloc(M, sizeof(double));
    eng->dm_mean      = (double *) calloc(M, sizeof(double));
    eng->dm_m2        = (double *) calloc(M, sizeof(double));
    eng->dm_sd        = (double *) calloc(M, sizeof(double));
    eng->ol_prev      = (float *) calloc(M, sizeof(float));
    eng->dol_hist     = (float *) calloc((size_t) SL_AR_DIFF_TAPS * M, sizeof(float));
    eng->G0           = (double *) calloc(mpp, sizeof(double));
    eng->b0           = (double *) calloc(mp, sizeof(double));
    eng->w0           = (double *) calloc(mp, sizeof(double));
    eng->G1           = (double *) calloc((size_t) Kmax * mpp, sizeof(double));
    eng->b1           = (double *) calloc((size_t) Kmax * mp, sizeof(double));
    eng->w1           = (double *) calloc((size_t) Kmax * mp, sizeof(double));
    eng->ring_phi     = (float *) calloc((size_t) SL_RING_SIZE * mp, sizeof(float));
    eng->ring_y0_fit  = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_y0_act  = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_y0_raw  = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_dy1_raw = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_base    = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_ol      = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_wfs     = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->ring_tweak   = (float *) calloc((size_t) SL_RING_SIZE * M, sizeof(float));
    eng->cur_phi      = (float *) calloc(mp, sizeof(float));
    eng->cur_dol      = (float *) calloc(M, sizeof(float));
    eng->cur_y0_fit   = (float *) calloc(M, sizeof(float));
    eng->cur_y0_act   = (float *) calloc(M, sizeof(float));
    eng->cur_y0_raw   = (float *) calloc(M, sizeof(float));
    eng->cur_dy1_raw  = (float *) calloc(M, sizeof(float));
    eng->cur_base     = (float *) calloc(M, sizeof(float));
    eng->blend_w      = (double *) calloc(mp, sizeof(double));

    if (!eng->w_mean || !eng->w_m2 || !eng->w_sd || !eng->dol_mean || !eng->dol_m2 ||
        !eng->dol_sd || !eng->dm_mean || !eng->dm_m2 || !eng->dm_sd || !eng->ol_prev ||
        !eng->dol_hist || !eng->G0 || !eng->b0 || !eng->w0 || !eng->G1 || !eng->b1 || !eng->w1 ||
        !eng->ring_phi || !eng->ring_y0_fit || !eng->ring_y0_act || !eng->ring_y0_raw ||
        !eng->ring_dy1_raw || !eng->ring_base || !eng->ring_ol || !eng->ring_wfs ||
        !eng->ring_tweak || !eng->cur_phi || !eng->cur_dol || !eng->cur_y0_fit ||
        !eng->cur_y0_act || !eng->cur_y0_raw || !eng->cur_dy1_raw || !eng->cur_base ||
        !eng->blend_w)
    {
        sl_engine_destroy(eng);
        return NULL;
    }

    sl_engine_reset(eng);
    return eng;
}

void sl_engine_destroy(SL_ENGINE *eng)
{
    if (eng == NULL)
    {
        return;
    }
#ifdef HAVE_LIBGRIC
    if (eng->gric_ctx != NULL)
    {
        gric_cluster_destroy(eng->gric_ctx);
        eng->gric_ctx = NULL;
    }
#endif
    free(eng->w_mean);
    free(eng->w_m2);
    free(eng->w_sd);
    free(eng->dol_mean);
    free(eng->dol_m2);
    free(eng->dol_sd);
    free(eng->dm_mean);
    free(eng->dm_m2);
    free(eng->dm_sd);
    free(eng->ol_prev);
    free(eng->dol_hist);
    free(eng->G0);
    free(eng->b0);
    free(eng->w0);
    free(eng->G1);
    free(eng->b1);
    free(eng->w1);
    free(eng->ring_phi);
    free(eng->ring_y0_fit);
    free(eng->ring_y0_act);
    free(eng->ring_y0_raw);
    free(eng->ring_dy1_raw);
    free(eng->ring_base);
    free(eng->ring_ol);
    free(eng->ring_wfs);
    free(eng->ring_tweak);
    free(eng->cur_phi);
    free(eng->cur_dol);
    free(eng->cur_y0_fit);
    free(eng->cur_y0_act);
    free(eng->cur_y0_raw);
    free(eng->cur_dy1_raw);
    free(eng->cur_base);
    free(eng->blend_w);
    free(eng);
}

void sl_engine_reset(SL_ENGINE *eng)
{
    if (eng == NULL)
    {
        return;
    }
    uint32_t M    = eng->M;
    uint32_t p    = eng->p_feat;
    uint32_t Kmax = eng->cfg.k_max;
    size_t   mp   = (size_t) M * p;
    size_t   mpp  = (size_t) M * p * p;

    eng->stat_n = 0;
    for (uint32_t m = 0; m < M; m++)
    {
        eng->w_mean[m]   = 0.0;
        eng->w_m2[m]     = 0.0;
        eng->w_sd[m]     = 1.0e-3;
        eng->dol_mean[m] = 0.0;
        eng->dol_m2[m]   = 0.0;
        eng->dol_sd[m]   = 1.0e-3;
        eng->dm_mean[m]  = 0.0;
        eng->dm_m2[m]    = 0.0;
        eng->dm_sd[m]    = 1.0e-2;
    }
    for (uint32_t i = 0; i < SL_MAX_KEY_DIM; i++)
    {
        eng->key_mean[i] = 0.0;
        eng->key_m2[i]   = 0.0;
        eng->key_sd[i]   = 1.0;
    }

    memset(eng->e_lo_ring, 0, sizeof(eng->e_lo_ring));
    memset(eng->e_hi_ring, 0, sizeof(eng->e_hi_ring));
    memset(eng->e_tot_ring, 0, sizeof(eng->e_tot_ring));
    memset(eng->e_vel_ring, 0, sizeof(eng->e_vel_ring));
    eng->e_pos     = 0;
    eng->vel_pos   = 0;
    eng->e_lo_sum  = 0.0;
    eng->e_hi_sum  = 0.0;
    eng->e_tot_sum = 0.0;
    eng->e_vel_sum = 0.0;

    memset(eng->ol_prev, 0, sizeof(float) * M);
    memset(eng->dol_hist, 0, sizeof(float) * (size_t) SL_AR_DIFF_TAPS * M);
    eng->dol_head  = 0;
    eng->dol_count = 0;

    eng->k_used = 0;
    memset(eng->anchors, 0, sizeof(eng->anchors));
    sl_create_gric(eng);

    memset(eng->G0, 0, sizeof(double) * mpp);
    memset(eng->b0, 0, sizeof(double) * mp);
    memset(eng->w0, 0, sizeof(double) * mp);
    eng->n0_eff = 0.0;

    memset(eng->G1, 0, sizeof(double) * (size_t) Kmax * mpp);
    memset(eng->b1, 0, sizeof(double) * (size_t) Kmax * mp);
    memset(eng->w1, 0, sizeof(double) * (size_t) Kmax * mp);
    for (uint32_t k = 0; k < Kmax; k++)
    {
        eng->n1_eff[k] = 0.0;
    }

    eng->c0_ey       = 0.0;
    eng->c0_yy       = 0.0;
    eng->c1_ey       = 0.0;
    eng->c1_yy       = 0.0;
    eng->oos_e_base2 = 0.0;
    eng->oos_e_l0_2  = 0.0;
    eng->oos_e_l1_2  = 0.0;

    eng->ring_head  = 0;
    eng->ring_count = 0;
    memset(eng->ring_valid, 0, sizeof(eng->ring_valid));
    memset(eng->ring_cid, 0, sizeof(eng->ring_cid));

    uint32_t g_act = eng->diag.gric_active;
    memset(&eng->diag, 0, sizeof(eng->diag));
    eng->diag.gric_active = g_act;
    eng->diag.last_cid    = -1;
}

/* Exact exponentially-weighted ridge normal equations: (G + lam * I) w = b.
 * G stays strictly symmetric positive semi-definite forever (never loses PD or drifts). */
static inline void rls_gram_update_solve(double      *Gm,
                                         double      *bm,
                                         double      *wm,
                                         const float *xm,
                                         double       target_val,
                                         uint32_t     p,
                                         double       gamma,
                                         double       lam)
{
    for (uint32_t i = 0; i < p; i++)
    {
        double  xi  = (double) xm[i];
        double *row = Gm + (size_t) i * p;
        bm[i]       = gamma * bm[i] + xi * target_val;
        for (uint32_t j = 0; j <= i; j++)
        {
            double v               = gamma * row[j] + xi * (double) xm[j];
            row[j]                 = v;
            Gm[(size_t) j * p + i] = v;
        }
    }

    double L[SL_MAX_PFEAT * SL_MAX_PFEAT];
    double y[SL_MAX_PFEAT];
    for (uint32_t i = 0; i < p; i++)
    {
        for (uint32_t j = 0; j <= i; j++)
        {
            double s = Gm[(size_t) i * p + j] + ((i == j) ? lam : 0.0);
            for (uint32_t k = 0; k < j; k++)
            {
                s -= L[i * p + k] * L[j * p + k];
            }
            if (i == j)
            {
                if (!sl_isfinite_d(s) || s <= 1.0e-18)
                {
                    return;
                }
                L[i * p + i] = sqrt(s);
            }
            else
            {
                L[i * p + j] = s / L[j * p + j];
            }
        }
    }
    for (uint32_t i = 0; i < p; i++)
    {
        double s = bm[i];
        for (uint32_t k = 0; k < i; k++)
        {
            s -= L[i * p + k] * y[k];
        }
        y[i] = s / L[i * p + i];
    }
    for (int32_t i = (int32_t) p - 1; i >= 0; i--)
    {
        double s = y[i];
        for (uint32_t k = (uint32_t) i + 1u; k < p; k++)
        {
            s -= L[k * p + (uint32_t) i] * wm[k];
        }
        double wi = s / L[(uint32_t) i * p + (uint32_t) i];
        wm[i]     = sl_isfinite_d(wi) ? wi : 0.0;
    }
}

/* Project per-mode physical weights onto closed-loop stability ball:
 *   - AR increment weights (indices 0..SL_AR_DIFF_TAPS-1) + shared dOL: L2 norm^2 <= max_ar_norm2
 *   - WFS residual weight (index SL_AR_DIFF_TAPS): physical gain in [-0.35, +max_wfs_gain]
 *     (or symmetric [-max_wfs_gain, +max_wfs_gain] in auxDM mode). */
static inline void project_weights_stable(double  *wm,
                                          uint32_t p,
                                          uint32_t n_nl,
                                          uint32_t q_sh,
                                          uint32_t n_wsh,
                                          double   dol_sd,
                                          double   w_sd,
                                          double   max_ar_norm2,
                                          double   max_wfs_gain,
                                          uint32_t out_mode)
{
    uint32_t idx_w   = SL_AR_DIFF_TAPS;
    uint32_t idx_nl  = idx_w + 1u;
    uint32_t idx_dol = idx_nl + n_nl;
    uint32_t idx_wsh = idx_dol + q_sh;
    double   inv_dol = 1.0 / fmax(dol_sd, 1.0e-6);
    double   ar_n2   = 0.0;
    for (uint32_t i = 0; i < SL_AR_DIFF_TAPS; i++)
    {
        double phys = wm[i] * inv_dol;
        ar_n2      += phys * phys;
    }
    for (uint32_t i = idx_dol; i < idx_dol + q_sh && i < p; i++)
    {
        double phys = wm[i] * inv_dol;
        ar_n2      += phys * phys;
    }
    if (ar_n2 > max_ar_norm2 && ar_n2 > 1.0e-18)
    {
        double sc = sqrt(max_ar_norm2 / ar_n2);
        for (uint32_t i = 0; i < SL_AR_DIFF_TAPS; i++)
        {
            wm[i] *= sc;
        }
        for (uint32_t i = idx_dol; i < idx_dol + q_sh && i < p; i++)
        {
            wm[i] *= sc;
        }
    }

    /* Cap linear WFS residual weight in physical DM/WFS units */
    double w_sd_safe = fmax(w_sd, 1.0e-6);
    double w_phys_lo = (out_mode == 0u) ? -0.35 : -max_wfs_gain;
    double w_phys_hi = max_wfs_gain;
    double w_lim_lo  = w_phys_lo * w_sd_safe;
    double w_lim_hi  = w_phys_hi * w_sd_safe;
    if (wm[idx_w] < w_lim_lo)
    {
        wm[idx_w] = w_lim_lo;
    }
    else if (wm[idx_w] > w_lim_hi)
    {
        wm[idx_w] = w_lim_hi;
    }

    /* Cap nonlinear odd-in-wfs optical-gain/saturation weights inside stability ball */
    if (n_nl > 0u)
    {
        double nl_lo  = -0.60 * max_wfs_gain * w_sd_safe;
        double nl_hi  = 0.80 * max_wfs_gain * w_sd_safe;
        double nl_sum = 0.0;
        for (uint32_t i = idx_nl; i < idx_nl + n_nl && i < p; i++)
        {
            if (wm[i] < nl_lo) wm[i] = nl_lo;
            else if (wm[i] > nl_hi) wm[i] = nl_hi;
            nl_sum += fabs(wm[i]);
        }
        double w_pos = (wm[idx_w] > 0.0) ? wm[idx_w] : 0.0;
        double cap_l1 = 1.60 * max_wfs_gain * w_sd_safe;
        if (w_pos + nl_sum > cap_l1 && nl_sum > 1.0e-18)
        {
            double budget = fmax(0.0, cap_l1 - w_pos);
            double sc     = budget / nl_sum;
            for (uint32_t i = idx_nl; i < idx_nl + n_nl && i < p; i++)
            {
                wm[i] *= sc;
            }
        }
    }

    /* Cap shared low-order WFS cross-talk L2 norm */
    if (n_wsh > 0u)
    {
        double inv_w = 1.0 / w_sd_safe;
        double xt_n2 = 0.0;
        for (uint32_t i = idx_wsh; i < idx_wsh + n_wsh && i < p; i++)
        {
            double phys = wm[i] * inv_w;
            xt_n2      += phys * phys;
        }
        double max_xt2 = 0.25 * max_wfs_gain * max_wfs_gain;
        if (xt_n2 > max_xt2 && xt_n2 > 1.0e-18)
        {
            double sc = sqrt(max_xt2 / xt_n2);
            for (uint32_t i = idx_wsh; i < idx_wsh + n_wsh && i < p; i++)
            {
                wm[i] *= sc;
            }
        }
    }
}

static void update_running_stats(SL_ENGINE   *eng,
                                 const float *wfs,
                                 const float *dm,
                                 const float *dol)
{
    uint32_t M = eng->M;
    eng->stat_n++;
    if (eng->stat_n <= (uint64_t) eng->cfg.key_warmup || eng->cfg.key_warmup == 0u)
    {
        double inv_n = 1.0 / (double) eng->stat_n;
        for (uint32_t m = 0; m < M; m++)
        {
            double wv = (double) wfs[m];
            double dw = wv - eng->w_mean[m];
            eng->w_mean[m] += dw * inv_n;
            eng->w_m2[m]   += dw * (wv - eng->w_mean[m]);
            if (eng->stat_n >= 8)
            {
                double v     = eng->w_m2[m] / (double) (eng->stat_n - 1);
                eng->w_sd[m] = sqrt(v > 1.0e-10 ? v : 1.0e-10);
            }

            double dv = (double) dol[m];
            double dd = dv - eng->dol_mean[m];
            eng->dol_mean[m] += dd * inv_n;
            eng->dol_m2[m]   += dd * (dv - eng->dol_mean[m]);
            if (eng->stat_n >= 8)
            {
                double v       = eng->dol_m2[m] / (double) (eng->stat_n - 1);
                eng->dol_sd[m] = sqrt(v > 1.0e-10 ? v : 1.0e-10);
            }

            if (dm != NULL)
            {
                double mv  = (double) dm[m];
                double ddm = mv - eng->dm_mean[m];
                eng->dm_mean[m] += ddm * inv_n;
                eng->dm_m2[m]   += ddm * (mv - eng->dm_mean[m]);
                if (eng->stat_n >= 8)
                {
                    double v      = eng->dm_m2[m] / (double) (eng->stat_n - 1);
                    eng->dm_sd[m] = sqrt(v > 1.0e-10 ? v : 1.0e-10);
                }
            }
        }
    }
}

static void extract_key(SL_ENGINE   *eng,
                        const float *wfs,
                        const float *dm,
                        const float *dol,
                        const float *ext_key,
                        double      *z_out)
{
    uint32_t M    = eng->M;
    uint32_t d_k  = eng->key_dim;
    uint32_t n_lo = (M >= 10u) ? 10u : ((M >= 2u) ? (M / 2u) : 1u);
    uint32_t n_hi = (M > n_lo) ? (M - n_lo) : 1u;

    double e_lo_cur  = 0.0;
    double e_hi_cur  = 0.0;
    double e_vel_cur = 0.0;
    for (uint32_t m = 0; m < M; m++)
    {
        double wz = ((double) wfs[m] - eng->w_mean[m]) / eng->w_sd[m];
        double w2 = wz * wz;
        if (m < n_lo)
        {
            e_lo_cur += w2;
        }
        else
        {
            e_hi_cur += w2;
        }
        if (m < 48u)
        {
            double dz  = ((double) dol[m] - eng->dol_mean[m]) / eng->dol_sd[m];
            e_vel_cur += dz * dz;
        }
    }
    e_lo_cur /= (double) n_lo;
    e_hi_cur  = (M > n_lo) ? (e_hi_cur / (double) n_hi) : e_lo_cur;
    double e_tot_cur =
        (e_lo_cur * (double) n_lo + e_hi_cur * (double) ((M > n_lo) ? n_hi : 0)) / (double) M;
    e_vel_cur /= (double) ((M < 48u) ? M : 48u);

    uint32_t ep = eng->e_pos;
    eng->e_lo_sum      += e_lo_cur - eng->e_lo_ring[ep];
    eng->e_hi_sum      += e_hi_cur - eng->e_hi_ring[ep];
    eng->e_tot_sum     += e_tot_cur - eng->e_tot_ring[ep];
    eng->e_lo_ring[ep]  = e_lo_cur;
    eng->e_hi_ring[ep]  = e_hi_cur;
    eng->e_tot_ring[ep] = e_tot_cur;
    eng->e_pos          = (ep + 1u) % SL_ENERGY_WIN;

    uint32_t vp = eng->vel_pos;
    eng->e_vel_sum     += e_vel_cur - eng->e_vel_ring[vp];
    eng->e_vel_ring[vp] = e_vel_cur;
    eng->vel_pos        = (vp + 1u) % SL_VEL_WIN;

    double ew_n  = (eng->stat_n < SL_ENERGY_WIN) ? (double) eng->stat_n : (double) SL_ENERGY_WIN;
    double vw_n  = (eng->stat_n < SL_VEL_WIN) ? (double) eng->stat_n : (double) SL_VEL_WIN;
    double l_lo  = log(fmax(eng->e_lo_sum / ew_n, 1.0e-8));
    double l_hi  = log(fmax(eng->e_hi_sum / ew_n, 1.0e-8));
    double l_tot = log(fmax(eng->e_tot_sum / ew_n, 1.0e-8));
    double l_vel = log(fmax(eng->e_vel_sum / vw_n, 1.0e-8));
    eng->last_l_lo = l_lo;

    double raw[SL_MAX_KEY_DIM];
    memset(raw, 0, sizeof(raw));

    switch (eng->cfg.key_mode)
    {
        case SL_KEY_TT2_E2:
            raw[0] = (M > 0) ? (((double) wfs[0] - eng->w_mean[0]) / eng->w_sd[0]) : 0.0;
            raw[1] = (M > 1) ? (((double) wfs[1] - eng->w_mean[1]) / eng->w_sd[1]) : 0.0;
            raw[2] = l_lo;
            raw[3] = l_hi;
            break;
        case SL_KEY_E1_LOGE:
            raw[0] = l_tot;
            break;
        case SL_KEY_WFS_LO4:
            for (uint32_t j = 0; j < 4u && j < M; j++)
            {
                raw[j] = ((double) wfs[j] - eng->w_mean[j]) / eng->w_sd[j];
            }
            break;
        case SL_KEY_TT2_DM2_E2:
            raw[0] = (M > 0) ? (((double) wfs[0] - eng->w_mean[0]) / eng->w_sd[0]) : 0.0;
            raw[1] = (M > 1) ? (((double) wfs[1] - eng->w_mean[1]) / eng->w_sd[1]) : 0.0;
            raw[2] = (dm && M > 0) ? (((double) dm[0] - eng->dm_mean[0]) / eng->dm_sd[0]) : 0.0;
            raw[3] = (dm && M > 1) ? (((double) dm[1] - eng->dm_mean[1]) / eng->dm_sd[1]) : 0.0;
            raw[4] = l_lo;
            raw[5] = l_hi;
            break;
        case SL_KEY_REGIME_E2_VEL:
            raw[0] = l_lo;
            raw[1] = l_hi;
            raw[2] = l_vel;
            break;
        case SL_KEY_EXTERNAL:
            if (ext_key != NULL)
            {
                for (uint32_t j = 0; j < d_k; j++)
                {
                    raw[j] = sl_isfinite_f(ext_key[j]) ? (double) ext_key[j] : 0.0;
                }
            }
            break;
        default:
            break;
    }

    if (eng->stat_n <= (uint64_t) eng->cfg.key_warmup || eng->cfg.key_warmup == 0u)
    {
        double inv_n = 1.0 / (double) eng->stat_n;
        for (uint32_t j = 0; j < d_k; j++)
        {
            double dj = raw[j] - eng->key_mean[j];
            eng->key_mean[j] += dj * inv_n;
            eng->key_m2[j]   += dj * (raw[j] - eng->key_mean[j]);
            if (eng->stat_n >= 16)
            {
                double v       = eng->key_m2[j] / (double) (eng->stat_n - 1);
                eng->key_sd[j] = sqrt(v > 1.0e-6 ? v : 1.0e-6);
            }
        }
    }

    for (uint32_t j = 0; j < d_k; j++)
    {
        double zv = (raw[j] - eng->key_mean[j]) / eng->key_sd[j];
        if (!sl_isfinite_d(zv)) zv = 0.0;
        if (zv > 10.0) zv = 10.0;
        else if (zv < -10.0) zv = -10.0;
        z_out[j] = zv;
    }
}

static int32_t assign_cluster(SL_ENGINE *eng, const double *z, int learn_on, double *dist_out)
{
    uint32_t d_k  = eng->key_dim;
    int32_t  cid  = 0;
    double   dist = 0.0;

#ifdef HAVE_LIBGRIC
    if (eng->gric_ctx != NULL)
    {
        gric_cluster_set_query_mode(eng->gric_ctx, learn_on ? 0 : 1);
        int64_t       gid = -1;
        gric_status_t st  = gric_cluster_feed_frame(eng->gric_ctx, z, &gid);
        if (st == GRIC_SUCCESS && gid >= 0 && gid < (int64_t) eng->cfg.k_max)
        {
            cid  = (int32_t) gid;
            dist = gric_cluster_get_last_dist(eng->gric_ctx);
            if (dist < 0.0 || !sl_isfinite_d(dist))
            {
                dist = 0.0;
            }
            int64_t nk = gric_cluster_get_num_clusters(eng->gric_ctx);
            if (nk > (int64_t) eng->cfg.k_max)
            {
                nk = (int64_t) eng->cfg.k_max;
            }
            if (nk > (int64_t) eng->k_used)
            {
                int64_t got = gric_cluster_get_anchors(eng->gric_ctx, eng->anchors, NULL,
                                                       (size_t) eng->cfg.k_max);
                if (got > 0)
                {
                    eng->k_used = (uint32_t) got;
                }
            }
            *dist_out = dist;
            return cid;
        }
    }
#endif

    if (eng->k_used == 0u)
    {
        memcpy(eng->anchors, z, sizeof(double) * d_k);
        eng->k_used = 1u;
        *dist_out   = 0.0;
        return 0;
    }
    uint32_t best_k  = 0u;
    double   best_d2 = 1.0e30;
    for (uint32_t k = 0; k < eng->k_used; k++)
    {
        const double *ak = eng->anchors + (size_t) k * d_k;
        double        d2 = 0.0;
        for (uint32_t j = 0; j < d_k; j++)
        {
            double dj = z[j] - ak[j];
            d2       += dj * dj;
        }
        if (d2 < best_d2)
        {
            best_d2 = d2;
            best_k  = k;
        }
    }
    double r2 = eng->cfg.rlim * eng->cfg.rlim;
    if (learn_on && best_d2 > r2 && eng->k_used < eng->cfg.k_max)
    {
        uint32_t nk = eng->k_used++;
        memcpy(eng->anchors + (size_t) nk * d_k, z, sizeof(double) * d_k);
        *dist_out = 0.0;
        return (int32_t) nk;
    }
    *dist_out = sqrt(best_d2);
    return (int32_t) best_k;
}

void sl_engine_step(SL_ENGINE   *eng,
                    const float *wfs,
                    const float *dm,
                    const float *ol,
                    const float *base_pf,
                    const float *ext_key,
                    int          learn_on,
                    int          shadow_on,
                    int          enable_l0,
                    int          enable_l1,
                    float        apply_gain,
                    float       *out_tweak,
                    float       *out_total,
                    SL_DIAG     *diag)
{
    if (eng == NULL || wfs == NULL || out_tweak == NULL || out_total == NULL)
    {
        return;
    }

    uint32_t     M      = eng->M;
    uint32_t     p      = eng->p_feat;
    uint32_t     q      = eng->q_sh;
    size_t       mp     = (size_t) M * p;
    size_t       mpp    = (size_t) M * p * p;
    const float *ol_cur = (ol != NULL) ? ol : wfs;

    /* Compute single-step PSOL increment dOL[t] = ol[t] - ol[t-1] */
    if (eng->dol_count > 0u)
    {
        for (uint32_t m = 0; m < M; m++)
        {
            float d        = ol_cur[m] - eng->ol_prev[m];
            eng->cur_dol[m] = sl_isfinite_f(d) ? d : 0.0f;
        }
    }
    else
    {
        memset(eng->cur_dol, 0, sizeof(float) * M);
    }

    /* Push cur_dol into 4-tap circular history */
    uint32_t dh = eng->dol_head;
    memcpy(eng->dol_hist + (size_t) dh * M, eng->cur_dol, sizeof(float) * M);

    /* 1. Update causal normalization stats & extract low-D key */
    if (learn_on || eng->stat_n < 16u)
    {
        update_running_stats(eng, wfs, dm, eng->cur_dol);
    }

    double z_cur[SL_MAX_KEY_DIM];
    extract_key(eng, wfs, dm, eng->cur_dol, ext_key, z_cur);

    /* 2. Build zero-DC per-mode regressor:
     *    phi[0..3]         = dOL_m(t..t-3) / dol_sd[m]
     *    phi[4]            = w_m(t) / w_sd[m]
     *    if nl_mode >= 1:
     *      phi[5]          = w_z * min(|w_z|, 3) / 3          (odd PyWFS S-curve saturation)
     *      phi[6]          = w_z * tanh(0.5 * log(E_lo))      (Strehl/energy OG modulation)
     *      phi[7]          = w_z * tanh(0.5 * (r_TT - 1))     (TT off-apex OG modulation)
     *    phi[idx_dol..+q]  = dOL_j(t) / dol_sd[j]             (shared low-order velocity)
     *    if nl_mode >= 2:
     *      phi[idx_wsh..+q]= w_j(t) / w_sd[j]                 (shared low-order WFS crosstalk) */
    double cap = eng->cfg.outlier_sig;
    float  dol_lo_z[SL_MAX_QSHARED];
    float  wfs_lo_z[SL_MAX_QSHARED];
    for (uint32_t j = 0; j < q; j++)
    {
        double dz = (double) eng->cur_dol[j] / eng->dol_sd[j];
        if (!sl_isfinite_d(dz)) dz = 0.0;
        if (dz > cap) dz = cap;
        else if (dz < -cap) dz = -cap;
        dol_lo_z[j] = (float) dz;

        double wzj = (double) wfs[j] / eng->w_sd[j];
        if (!sl_isfinite_d(wzj)) wzj = 0.0;
        if (wzj > cap) wzj = cap;
        else if (wzj < -cap) wzj = -cap;
        wfs_lo_z[j] = (float) wzj;
    }

    double og_mod = 0.0;
    double tt_mod = 0.0;
    if (eng->n_nl > 0u)
    {
        og_mod = tanh(0.5 * eng->last_l_lo);
        double w0z = (M > 0u) ? ((double) wfs[0] / eng->w_sd[0]) : 0.0;
        double w1z = (M > 1u) ? ((double) wfs[1] / eng->w_sd[1]) : 0.0;
        if (!sl_isfinite_d(w0z)) w0z = 0.0;
        if (!sl_isfinite_d(w1z)) w1z = 0.0;
        double r_tt = sqrt(w0z * w0z + w1z * w1z);
        tt_mod      = tanh(0.5 * (r_tt - 1.0));
    }

    uint32_t tap_slot[SL_AR_DIFF_TAPS];
    for (uint32_t k = 0; k < SL_AR_DIFF_TAPS; k++)
    {
        tap_slot[k] = (dh + SL_AR_DIFF_TAPS - k) % SL_AR_DIFF_TAPS;
    }

    uint32_t idx_nl  = SL_AR_DIFF_TAPS + 1u;
    uint32_t idx_dol = idx_nl + eng->n_nl;
    uint32_t idx_wsh = idx_dol + q;

    for (uint32_t m = 0; m < M; m++)
    {
        float *pm      = eng->cur_phi + (size_t) m * p;
        double inv_dsd = 1.0 / eng->dol_sd[m];
        for (uint32_t k = 0; k < SL_AR_DIFF_TAPS; k++)
        {
            double dv = (eng->dol_count >= k)
                            ? ((double) eng->dol_hist[(size_t) tap_slot[k] * M + m] * inv_dsd)
                            : 0.0;
            if (!sl_isfinite_d(dv)) dv = 0.0;
            if (dv > cap) dv = cap;
            else if (dv < -cap) dv = -cap;
            pm[k] = (float) dv;
        }
        double wz = (double) wfs[m] / eng->w_sd[m];
        if (!sl_isfinite_d(wz)) wz = 0.0;
        if (wz > cap) wz = cap;
        else if (wz < -cap) wz = -cap;
        pm[SL_AR_DIFF_TAPS] = (float) wz;
        if (eng->n_nl > 0u)
        {
            double sat_fac = fmin(fabs(wz), 3.0) / 3.0;
            pm[idx_nl + 0u] = (float) (wz * sat_fac);
            pm[idx_nl + 1u] = (float) (wz * og_mod);
            pm[idx_nl + 2u] = (float) (wz * tt_mod);
        }
        for (uint32_t j = 0; j < q; j++)
        {
            pm[idx_dol + j] = dol_lo_z[j];
        }
        for (uint32_t j = 0; j < eng->n_wsh; j++)
        {
            pm[idx_wsh + j] = wfs_lo_z[j];
        }
    }

    /* 3. Strictly causal delayed update for past query frame s = t - (lat_i + 1) */
    uint32_t lat_i = (uint32_t) eng->cfg.latency_fr;
    double   lat_f = eng->cfg.latency_fr - (double) lat_i;
    uint32_t lag_s = lat_i + 1u;

    if (learn_on && eng->ring_count >= (uint64_t) lag_s && eng->stat_n >= 16u)
    {
        uint32_t slot_s   = (eng->ring_head + SL_RING_SIZE - lag_s) & (SL_RING_SIZE - 1u);
        uint32_t slot_tm1 = (eng->ring_head + SL_RING_SIZE - 1u) & (SL_RING_SIZE - 1u);

        if (eng->ring_valid[slot_s] && eng->ring_valid[slot_tm1])
        {
            const float *phi_s      = eng->ring_phi + (size_t) slot_s * mp;
            const float *base_s     = eng->ring_base + (size_t) slot_s * M;
            const float *y0_fit_s   = eng->ring_y0_fit + (size_t) slot_s * M;
            const float *y0_act_s   = eng->ring_y0_act + (size_t) slot_s * M;
            const float *y0_raw_s   = eng->ring_y0_raw + (size_t) slot_s * M;
            const float *dy1_s      = eng->ring_dy1_raw + (size_t) slot_s * M;
            const float *twk_s      = eng->ring_tweak + (size_t) slot_s * M;
            const float *ol_tm1     = eng->ring_ol + (size_t) slot_tm1 * M;
            const float *wfs_tm1    = eng->ring_wfs + (size_t) slot_tm1 * M;
            int32_t      cid_s      = eng->ring_cid[slot_s];
            double       dc_sc      = (base_pf != NULL) ? 1.0 : eng->cfg.dc_leak;
            int          l1_ready   = (!enable_l0 || eng->n0_eff >= 40.0);

            double sum_y2    = 0.0;
            double sum_e0_2  = 0.0;
            double sum_e1_2  = 0.0;
            double dot_0_ey  = 0.0;
            double dot_0_yy  = 0.0;
            double dot_1_ey  = 0.0;
            double dot_1_yy  = 0.0;
            double max_z_err = 0.0;
            int    finite_ok = 1;

            double a1_cur = (eng->c1_yy > 1.0e-18)
                                ? fmin(1.0, fmax(0.0, eng->c1_ey / (eng->c1_yy + 1.0e-18)))
                                : 0.0;

            for (uint32_t m = 0; m < M; m++)
            {
                double yt;
                if (eng->cfg.out_mode == 0u)
                {
                    double o_obs = (1.0 - lat_f) * (double) ol_tm1[m] + lat_f * (double) ol_cur[m];
                    yt           = dc_sc * o_obs - (double) base_s[m];
                }
                else
                {
                    double w_obs = (1.0 - lat_f) * (double) wfs_tm1[m] + lat_f * (double) wfs[m];
                    yt           = eng->cfg.aux_leak * (double) twk_s[m] -
                                   eng->cfg.wfs_to_dm * w_obs;
                }
                if (!sl_isfinite_d(yt))
                {
                    finite_ok = 0;
                    break;
                }
                double ref_sd = (eng->cfg.out_mode == 0u)
                                    ? (eng->dol_sd[m] * eng->cfg.latency_fr)
                                    : eng->w_sd[m];
                double zerr   = fabs(yt) / fmax(ref_sd, 1.0e-4);
                if (zerr > max_z_err)
                {
                    max_z_err = zerr;
                }

                double y0r = (double) y0_raw_s[m];
                double y0a = enable_l0 ? (double) y0_act_s[m] : 0.0;
                double e0  = yt - y0a;
                double d1r = l1_ready ? (double) dy1_s[m] : 0.0;
                double e1  = e0 - a1_cur * d1r;

                sum_y2   += yt * yt;
                sum_e0_2 += e0 * e0;
                sum_e1_2 += e1 * e1;
                dot_0_ey += yt * y0r;
                dot_0_yy += y0r * y0r;
                if (l1_ready)
                {
                    dot_1_ey += e0 * d1r;
                    dot_1_yy += d1r * d1r;
                }
            }

            if (!finite_ok || max_z_err > eng->cfg.outlier_sig * 8.0)
            {
                eng->diag.outliers_skipped++;
            }
            else
            {
                double eta = eng->cfg.gate_eta;
                eng->c0_ey       = eta * eng->c0_ey + dot_0_ey;
                eng->c0_yy       = eta * eng->c0_yy + dot_0_yy;
                if (eng->c0_ey < -0.25 * eng->c0_yy)
                {
                    eng->c0_ey = -0.25 * eng->c0_yy;
                }
                if (l1_ready)
                {
                    eng->c1_ey = eta * eng->c1_ey + dot_1_ey;
                    eng->c1_yy = eta * eng->c1_yy + dot_1_yy;
                    if (eng->c1_ey < -0.25 * eng->c1_yy)
                    {
                        eng->c1_ey = -0.25 * eng->c1_yy;
                    }
                }
                eng->oos_e_base2 = eta * eng->oos_e_base2 + sum_y2;
                eng->oos_e_l0_2  = eta * eng->oos_e_l0_2 + sum_e0_2;
                eng->oos_e_l1_2  = eta * eng->oos_e_l1_2 + sum_e1_2;

                double g0   = eng->cfg.gamma0;
                double lam0 = eng->cfg.lam0;
                double g1   = eng->cfg.gamma1;
                double lam1 = eng->cfg.lam1;

                int     do_l1_upd = (l1_ready && cid_s >= 0 && (uint32_t) cid_s < eng->k_used);
                double *G1_ck     = do_l1_upd ? (eng->G1 + (size_t) cid_s * mpp) : NULL;
                double *b1_ck     = do_l1_upd ? (eng->b1 + (size_t) cid_s * mp) : NULL;
                double *w1_ck     = do_l1_upd ? (eng->w1 + (size_t) cid_s * mp) : NULL;

                uint32_t m;
#pragma omp parallel for schedule(static) if (M >= 256)
                for (m = 0; m < M; m++)
                {
                    double yt;
                    if (eng->cfg.out_mode == 0u)
                    {
                        double o_obs =
                            (1.0 - lat_f) * (double) ol_tm1[m] + lat_f * (double) ol_cur[m];
                        yt = dc_sc * o_obs - (double) base_s[m];
                    }
                    else
                    {
                        double w_obs =
                            (1.0 - lat_f) * (double) wfs_tm1[m] + lat_f * (double) wfs[m];
                        yt = eng->cfg.aux_leak * (double) twk_s[m] -
                             eng->cfg.wfs_to_dm * w_obs;
                    }
                    const float *xm  = phi_s + (size_t) m * p;
                    double      *w0m = eng->w0 + (size_t) m * p;

                    /* Exact SPD exponentially-weighted ridge normal equations on Level 0 */
                    rls_gram_update_solve(eng->G0 + (size_t) m * p * p, eng->b0 + (size_t) m * p,
                                          w0m, xm, yt, p, g0, lam0);

                    if (do_l1_upd)
                    {
                        /* Train Level 1 strictly on the unbiased unconstrained L0 residual so
                         * stability clamping or a0/sh0 attenuation never leaks global linear
                         * gain into cluster weights. */
                        double  e0_train = yt - (enable_l0 ? (double) y0_fit_s[m] : 0.0);
                        double *w1m      = w1_ck + (size_t) m * p;
                        rls_gram_update_solve(G1_ck + (size_t) m * p * p, b1_ck + (size_t) m * p,
                                              w1m, xm, e0_train, p, g1, lam1);
                    }
                }
                eng->n0_eff = g0 * eng->n0_eff + 1.0;
                if (do_l1_upd)
                {
                    eng->n1_eff[cid_s] = g1 * eng->n1_eff[cid_s] + 1.0;
                }
                eng->diag.updates++;
            }
        }
    }

    /* 4. Assign current key to GRIC cluster */
    int     can_spawn = learn_on && (eng->stat_n >= 16u);
    double  dist      = 0.0;
    int32_t cid       = assign_cluster(eng, z_cur, can_spawn, &dist);

    /* 5. Assemble Level-1 GRIC cluster weight blend (soft Gaussian LWPR or hard nearest) */
    double a0       = (eng->c0_yy > 1.0e-18)
                          ? fmin(1.0, fmax(0.0, eng->c0_ey / (eng->c0_yy + 1.0e-18)))
                          : 0.0;
    double a1       = (eng->c1_yy > 1.0e-18)
                          ? fmin(1.0, fmax(0.0, eng->c1_ey / (eng->c1_yy + 1.0e-18)))
                          : 0.0;
    double sh0      = eng->n0_eff / (eng->n0_eff + 20.0);
    double gate_lim = eng->cfg.rlim * eng->cfg.gate_mult;
    int    has_l1   = 0;

    if (eng->k_used > 0u && dist <= gate_lim)
    {
        double nw = (eng->cfg.n_warmup > 0u) ? (double) eng->cfg.n_warmup : 1.0;
        if (eng->cfg.blend_soft && eng->k_used > 1u)
        {
            double   kw[SL_MAX_CLUSTERS];
            double   wsum    = 0.0;
            double   sig     = eng->cfg.rlim * eng->cfg.blend_bw;
            double   inv_2s2 = 0.5 / (sig * sig);
            uint32_t d_k     = eng->key_dim;

            for (uint32_t k = 0; k < eng->k_used; k++)
            {
                if (eng->n1_eff[k] < 1.0)
                {
                    kw[k] = 0.0;
                    continue;
                }
                const double *ak = eng->anchors + (size_t) k * d_k;
                double        d2 = 0.0;
                for (uint32_t j = 0; j < d_k; j++)
                {
                    double dj = z_cur[j] - ak[j];
                    d2       += dj * dj;
                }
                kw[k] = exp(-d2 * inv_2s2);
                wsum += kw[k];
            }
            if (wsum > 1.0e-12)
            {
                double inv_ws = 1.0 / wsum;
                memset(eng->blend_w, 0, sizeof(double) * mp);
                for (uint32_t k = 0; k < eng->k_used; k++)
                {
                    if (kw[k] <= 1.0e-9)
                    {
                        continue;
                    }
                    double        shk = eng->n1_eff[k] / (eng->n1_eff[k] + nw);
                    double        wt  = (kw[k] * inv_ws) * shk;
                    const double *wk  = eng->w1 + (size_t) k * mp;
                    for (size_t i = 0; i < mp; i++)
                    {
                        eng->blend_w[i] += wt * wk[i];
                    }
                }
                has_l1 = 1;
            }
        }
        else if (cid >= 0 && (uint32_t) cid < eng->k_used && eng->n1_eff[cid] >= 1.0)
        {
            double        shk = eng->n1_eff[cid] / (eng->n1_eff[cid] + nw);
            const double *wk  = eng->w1 + (size_t) cid * mp;
            for (size_t i = 0; i < mp; i++)
            {
                eng->blend_w[i] = shk * wk[i];
            }
            has_l1 = 1;
        }
    }

    /* 6. Combined stability-ball projection & bounded command synthesis:
     *    Both Level-0 alone and combined Level-0 + Level-1 are projected onto the EXACT SAME
     *    closed-loop stability ball (max_ar_norm2, max_wfs_gain), so Level 1 can never
     *    circumvent stability limits or add extra global gain. */
    double gain = (apply_gain < 0.0f) ? 0.0 : ((apply_gain > 1.0f) ? 1.0 : (double) apply_gain);
    if (shadow_on)
    {
        gain = 0.0;
    }
    double max_twk = (eng->cfg.max_tweak > 0.0) ? eng->cfg.max_tweak : 1.0e6;
    double twk_ss  = 0.0;
    double l1_ss   = 0.0;

    for (uint32_t m = 0; m < M; m++)
    {
        const float  *xm  = eng->cur_phi + (size_t) m * p;
        const double *w0m = eng->w0 + (size_t) m * p;
        const double *w1m = has_l1 ? (eng->blend_w + (size_t) m * p) : NULL;

        double w0_raw_m[SL_MAX_PFEAT];
        double w0_act_m[SL_MAX_PFEAT];
        double w_tot_m[SL_MAX_PFEAT];
        double w_eff_m[SL_MAX_PFEAT];

        double y0_fit = 0.0;
        for (uint32_t j = 0; j < p; j++)
        {
            y0_fit      += w0m[j] * (double) xm[j];
            w0_raw_m[j]  = sh0 * w0m[j];
        }
        project_weights_stable(w0_raw_m, p, eng->n_nl, eng->q_sh, eng->n_wsh,
                               eng->dol_sd[m], eng->w_sd[m], eng->cfg.max_ar_norm2,
                               eng->cfg.max_wfs_gain, eng->cfg.out_mode);

        double y0_proj_raw = 0.0;
        for (uint32_t j = 0; j < p; j++)
        {
            y0_proj_raw += w0_raw_m[j] * (double) xm[j];
            w0_act_m[j]  = enable_l0 ? (a0 * w0_raw_m[j]) : 0.0;
        }
        double y0_act = enable_l0 ? (a0 * y0_proj_raw) : 0.0;

        double dy1_raw = 0.0;
        if (w1m != NULL)
        {
            for (uint32_t j = 0; j < p; j++)
            {
                w_tot_m[j] = w0_act_m[j] + w1m[j];
            }
            project_weights_stable(w_tot_m, p, eng->n_nl, eng->q_sh, eng->n_wsh,
                                   eng->dol_sd[m], eng->w_sd[m], eng->cfg.max_ar_norm2,
                                   eng->cfg.max_wfs_gain, eng->cfg.out_mode);
            double y_tot_raw = 0.0;
            for (uint32_t j = 0; j < p; j++)
            {
                y_tot_raw += w_tot_m[j] * (double) xm[j];
            }
            dy1_raw = y_tot_raw - y0_act;
        }

        for (uint32_t j = 0; j < p; j++)
        {
            w_eff_m[j] = w0_act_m[j] + ((enable_l1 && w1m != NULL) ? (a1 * w1m[j]) : 0.0);
        }
        project_weights_stable(w_eff_m, p, eng->n_nl, eng->q_sh, eng->n_wsh,
                               eng->dol_sd[m], eng->w_sd[m], eng->cfg.max_ar_norm2,
                               eng->cfg.max_wfs_gain, eng->cfg.out_mode);
        double y_eff = 0.0;
        for (uint32_t j = 0; j < p; j++)
        {
            y_eff += w_eff_m[j] * (double) xm[j];
        }
        double c1 = y_eff - y0_act;

        eng->cur_y0_fit[m]  = (float) y0_fit;
        eng->cur_y0_raw[m]  = (float) y0_proj_raw;
        eng->cur_y0_act[m]  = (float) y0_act;
        eng->cur_dy1_raw[m] = (float) dy1_raw;

        float bv = 0.0f;
        if (eng->cfg.out_mode == 0u)
        {
            bv = (base_pf != NULL) ? base_pf[m] : (float) (eng->cfg.dc_leak * (double) ol_cur[m]);
            if (!sl_isfinite_f(bv))
            {
                bv = 0.0f;
            }
        }
        eng->cur_base[m] = bv;

        double dt = gain * y_eff;
        if (!sl_isfinite_d(dt)) dt = 0.0;
        if (!sl_isfinite_d(c1)) c1 = 0.0;
        if (dt > max_twk) dt = max_twk;
        else if (dt < -max_twk) dt = -max_twk;

        out_tweak[m] = (float) dt;
        out_total[m] = (eng->cfg.out_mode == 0u) ? (bv + (float) dt) : (float) dt;
        twk_ss      += dt * dt;
        l1_ss       += c1 * c1;
    }

    /* 7. Record frame into strictly causal delayed-update ring buffer */
    uint32_t h = eng->ring_head;
    memcpy(eng->ring_phi + (size_t) h * mp, eng->cur_phi, sizeof(float) * mp);
    memcpy(eng->ring_y0_fit + (size_t) h * M, eng->cur_y0_fit, sizeof(float) * M);
    memcpy(eng->ring_y0_act + (size_t) h * M, eng->cur_y0_act, sizeof(float) * M);
    memcpy(eng->ring_y0_raw + (size_t) h * M, eng->cur_y0_raw, sizeof(float) * M);
    memcpy(eng->ring_dy1_raw + (size_t) h * M, eng->cur_dy1_raw, sizeof(float) * M);
    memcpy(eng->ring_base + (size_t) h * M, eng->cur_base, sizeof(float) * M);
    memcpy(eng->ring_ol + (size_t) h * M, ol_cur, sizeof(float) * M);
    memcpy(eng->ring_wfs + (size_t) h * M, wfs, sizeof(float) * M);
    memcpy(eng->ring_tweak + (size_t) h * M, out_tweak, sizeof(float) * M);
    eng->ring_cid[h]   = cid;
    eng->ring_valid[h] = 1u;
    eng->ring_head     = (h + 1u) & (SL_RING_SIZE - 1u);
    eng->ring_count++;

    memcpy(eng->ol_prev, ol_cur, sizeof(float) * M);
    eng->dol_head = (eng->dol_head + 1u) % SL_AR_DIFF_TAPS;
    if (eng->dol_count < SL_AR_DIFF_TAPS)
    {
        eng->dol_count++;
    }

    /* 8. Update diagnostics */
    eng->diag.frames++;
    eng->diag.n_clusters = eng->k_used;
    eng->diag.last_cid   = cid;
    eng->diag.last_dist  = (float) dist;
    eng->diag.alpha_l0   = (float) a0;
    eng->diag.alpha_l1   = (float) a1;
    eng->diag.tweak_rms  = (float) sqrt(twk_ss / (double) M);
    eng->diag.l1_rms     = (float) sqrt(l1_ss / (double) M);

    if (eng->oos_e_base2 > 1.0e-18)
    {
        eng->diag.oos_vr_l0_pct =
            (float) (100.0 * (1.0 - eng->oos_e_l0_2 / eng->oos_e_base2));
        eng->diag.oos_vr_tot_pct =
            (float) (100.0 * (1.0 - eng->oos_e_l1_2 / eng->oos_e_base2));
    }
    if (eng->oos_e_l0_2 > 1.0e-18)
    {
        eng->diag.oos_vr_l1_pct =
            (float) (100.0 * (1.0 - eng->oos_e_l1_2 / eng->oos_e_l0_2));
    }

    if (diag != NULL)
    {
        *diag = eng->diag;
    }
}

int sl_engine_save_gric(const SL_ENGINE *eng, const char *out_dir)
{
    if (eng == NULL || out_dir == NULL || out_dir[0] == '\0')
    {
        return -1;
    }
#ifdef HAVE_LIBGRIC
    if (eng->gric_ctx != NULL)
    {
        return (gric_cluster_save_results(eng->gric_ctx, out_dir) == GRIC_SUCCESS) ? 0 : -1;
    }
#endif
    return -1;
}

uint32_t sl_engine_get_cluster_summary(const SL_ENGINE *eng,
                                       float           *out_counts,
                                       uint32_t         max_k)
{
    if (eng == NULL)
    {
        return 0u;
    }
    uint32_t n = (eng->k_used < max_k) ? eng->k_used : max_k;
    if (out_counts != NULL)
    {
        for (uint32_t k = 0; k < n; k++)
        {
            out_counts[k] = (float) eng->n1_eff[k];
        }
    }
    return eng->k_used;
}
