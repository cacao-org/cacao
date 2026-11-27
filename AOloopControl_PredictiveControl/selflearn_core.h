// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    selflearn_core.h
 * @brief   Online GRIC state-space clustering + 2-level hierarchical factorized RLS core
 *          with closed-loop stability guarantees (DC leak, zero-DC increment basis, bounded
 *          WFS optical-gain weight, AR filter L2 cap, and causal OOS Stein gating).
 */

#ifndef CACAO_AOLOOPCONTROL_PC_SELFLEARN_CORE_H
#define CACAO_AOLOOPCONTROL_PC_SELFLEARN_CORE_H

#include <stdint.h>

#ifdef HAVE_LIBGRIC
#    include <gric/gric.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define SL_MAX_KEY_DIM   16
#define SL_MAX_CLUSTERS  64
#define SL_MAX_QSHARED   8
#define SL_AR_DIFF_TAPS  4 /* [dOL(t), dOL(t-1), dOL(t-2), dOL(t-3)] */
#define SL_NL_WFS_TAPS   3 /* [w*|w|, w*tanh(0.5*logE_lo), w*tanh(0.5*(r_TT-1))] */
#define SL_MAX_PFEAT     (SL_AR_DIFF_TAPS + 1 + SL_NL_WFS_TAPS + 2 * SL_MAX_QSHARED)
#define SL_RING_SIZE     64 /* Power of 2; covers causal lag up to 60 frames */
#define SL_ENERGY_WIN    8
#define SL_VEL_WIN       64

typedef enum
{
    SL_KEY_TT2_E2        = 0, /* 4D: [w_0, w_1, log E_lo, log E_hi] */
    SL_KEY_E1_LOGE       = 1, /* 1D: [log E_tot] */
    SL_KEY_WFS_LO4       = 2, /* 4D: [w_0, w_1, w_2, w_3] */
    SL_KEY_TT2_DM2_E2    = 3, /* 6D: [w_0, w_1, dm_0, dm_1, log E_lo, log E_hi] */
    SL_KEY_REGIME_E2_VEL = 4, /* 3D: [log E_lo, log E_hi, log E_dOL] */
    SL_KEY_EXTERNAL      = 5  /* External 1D SHM key stream (<= SL_MAX_KEY_DIM) */
} SL_KEY_MODE;

typedef struct
{
    uint32_t n_modes;      /* Number of controlled/tweaked modes M */
    uint32_t key_mode;     /* SL_KEY_MODE */
    uint32_t ext_key_dim;  /* Used only when key_mode == SL_KEY_EXTERNAL */
    uint32_t k_max;        /* Max GRIC clusters (1..SL_MAX_CLUSTERS) */
    uint32_t q_shared;     /* Shared low-order cross-mode velocity features (0..SL_MAX_QSHARED) */
    uint32_t n_warmup;     /* Per-cluster shrinkage warmup count */
    uint32_t key_warmup;   /* Frames to stabilize online key/feature std before freezing scale */
    uint32_t blend_soft;   /* 1 = soft Gaussian LWPR blend across anchors, 0 = hard nearest */
    uint32_t out_mode;     /* 0 = PF (synchronous PSOL), 1 = auxDM (damped additive DM tweak) */
    uint32_t nl_mode;      /* 0 = linear phi, 1 = +odd PyWFS NL WFS terms, 2 = +shared WFS xtalk */
    double   rlim;         /* GRIC cluster spawn radius in standardized key space */
    double   gate_mult;    /* Fail-safe distance gate multiplier (0 out if dist > gate_mult*rlim) */
    double   blend_bw;     /* Soft kernel bandwidth multiplier relative to rlim (default 1.0) */
    double   tm_mix;       /* GRIC temporal Markov transition mixing coeff (0..1) */
    double   latency_fr;   /* Causal target horizon [frames] (>= 1.0) */
    double   dc_leak;      /* Stabilizing DC leak (< 1.0, e.g. 0.985) on ZOH PSOL base */
    double   aux_leak;     /* Damping leak (< 1.0, e.g. 0.75) in auxDM counterfactual target */
    double   wfs_to_dm;    /* Nominal WFS-to-DM factor in auxDM mode (e.g. 0.5) */
    double   max_wfs_gain; /* Hard physical stability bound on WFS residual weight (e.g. 0.5) */
    double   max_ar_norm2; /* Hard L2 norm^2 cap on AR increment filter (e.g. 2.0) */
    double   lam0;         /* Level-0 global RLS prior ridge */
    double   lam1;         /* Level-1 per-cluster RLS prior ridge (shrinkage toward L0) */
    double   gamma0;       /* Level-0 exponential forgetting factor (e.g. 0.998) */
    double   gamma1;       /* Level-1 exponential forgetting factor (e.g. 0.999) */
    double   gate_eta;     /* Causal OOS Stein gate EMA decay (e.g. 0.995) */
    double   max_tweak;    /* Hard per-mode absolute tweak clamp [modal units] */
    double   outlier_sig;  /* Update rejection threshold in feature/target sigma (e.g. 12.0) */
} SL_CONFIG;

typedef struct
{
    uint64_t frames;
    uint64_t updates;
    uint64_t outliers_skipped;
    uint32_t gric_active;    /* 1 if compiled & running with libgric, 0 if fallback leader */
    uint32_t n_clusters;     /* Active GRIC cluster count K */
    int32_t  last_cid;       /* Last assigned cluster ID */
    float    last_dist;      /* Last key distance to assigned anchor */
    float    alpha_l0;       /* Live causal OOS Stein gate for Level 0 [0..1] */
    float    alpha_l1;       /* Live causal OOS Stein gate for Level 1 [0..1] */
    float    oos_vr_l0_pct;  /* Causal OOS variance reduction of L0 over baseline [%] */
    float    oos_vr_l1_pct;  /* Causal OOS extra variance reduction of L1 over L0 [%] */
    float    oos_vr_tot_pct; /* Causal OOS total variance reduction over baseline [%] */
    float    tweak_rms;      /* Instantaneous RMS of applied tweak vector */
    float    l1_rms;         /* Instantaneous RMS of Level-1 cluster contribution */
} SL_DIAG;

typedef struct SL_ENGINE SL_ENGINE;

SL_ENGINE *sl_engine_create(const SL_CONFIG *cfg);
void       sl_engine_destroy(SL_ENGINE *eng);
void       sl_engine_reset(SL_ENGINE *eng);

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
                    SL_DIAG     *diag);

int      sl_engine_save_gric(const SL_ENGINE *eng, const char *out_dir);
uint32_t sl_engine_get_cluster_summary(const SL_ENGINE *eng,
                                       float           *out_counts,
                                       uint32_t         max_k);

#ifdef __cplusplus
}
#endif

#endif /* CACAO_AOLOOPCONTROL_PC_SELFLEARN_CORE_H */
