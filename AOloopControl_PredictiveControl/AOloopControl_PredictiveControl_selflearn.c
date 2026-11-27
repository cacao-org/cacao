// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl_selflearn.c
 * @brief   Real-time GRIC clustering + 2-level hierarchical RLS self-learning AO tweak unit
 *
 * Continuously clusters the low-dimensional AO state/condition key with `libgric`, maintains
 * per-cluster online forgetting RLS affine/linear models with hierarchical shrinkage toward a
 * global online RLS base and causal OOS Stein gates, and publishes a bounded real-time tweak
 * either synchronously into the `mfilt` predictive-filter path (`out_mode=0`) or directly into
 * `aol{N}_modevalauxDM` (`out_mode=1`).
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"
#include "AOloopControl_PredictiveControl/selflearn_core.h"

static uint64_t p_loopindex                               = 1;
static char     p_inwfs[FUNCTION_PARAMETER_STRMAXLEN]     = "aol1_modevalWFS";
static char     p_inol[FUNCTION_PARAMETER_STRMAXLEN]      = "aol1_modevalOL";
static char     p_indm[FUNCTION_PARAMETER_STRMAXLEN]      = "aol1_modevalDM";
static char     p_inbasepf[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_inbasestat[FUNCTION_PARAMETER_STRMAXLEN]= "";
static char     p_inkey[FUNCTION_PARAMETER_STRMAXLEN]     = "";
static char     p_outdata[FUNCTION_PARAMETER_STRMAXLEN]   = "aol1_modevalPF";
static char     p_outstat[FUNCTION_PARAMETER_STRMAXLEN]   = "aol1_outPFstat";
static char     p_outtweak[FUNCTION_PARAMETER_STRMAXLEN]  = "aol1_selflearn_tweak";
static char     p_savedir[FUNCTION_PARAMETER_STRMAXLEN]   = "";

static uint32_t p_out_mode   = 0;      /* 0=PF (synchronous PSOL), 1=auxDM */
static uint32_t p_nl_mode    = 0;      /* 0=linear phi, 1=+odd PyWFS NL, 2=+shared WFS xtalk */
static uint32_t p_key_mode   = 0;      /* 0=TT2_E2, 1=E1, 2=LO4, 3=TT2_DM2_E2, 4=REGIME, 5=EXT */
static uint32_t p_nbmodemax  = 64;     /* Max controlled modes tweaked */
static uint32_t p_kmax       = 16;     /* Max GRIC clusters (1..64) */
static uint32_t p_qshared    = 2;      /* Shared low-order cross-talk features (0..8) */
static uint32_t p_nwarmup    = 40;     /* Per-cluster shrinkage warmup count */
static uint32_t p_keywarmup  = 200;    /* Key standardization warmup frames */
static uint32_t p_rampsteps  = 64;     /* Smooth PF active-status ramp steps */
static int32_t  p_blend_soft = 1;      /* 1=soft Gaussian LWPR blend, 0=hard nearest */
static float    p_rlim       = 2.0f;   /* GRIC cluster spawn radius */
static float    p_gate_mult  = 1.5f;   /* Fail-safe distance gate multiplier */
static float    p_blend_bw   = 1.0f;   /* Soft kernel bandwidth multiplier */
static float    p_tm_mix     = 0.5f;   /* GRIC Markov transition mixing coeff */
static float    p_latency      = 2.1f;   /* Causal target horizon [frames] */
static float    p_dc_leak      = 0.985f; /* Stabilizing DC leak (<1.0) on standalone PSOL */
static float    p_aux_leak     = 0.75f;  /* Damping leak (<1.0) in auxDM mode */
static float    p_wfs_to_dm    = 0.5f;   /* Nominal WFS->DM scale (auxDM mode) */
static float    p_max_wfs_gain = 0.5f;   /* Physical stability cap on WFS weight */
static float    p_max_ar_norm2 = 2.0f;   /* L2 norm^2 cap on AR increment filter */
static float    p_lam0         = 20.0f;  /* Level-0 global RLS ridge */
static float    p_lam1         = 200.0f; /* Level-1 per-cluster RLS ridge */
static float    p_gamma0       = 0.998f; /* Level-0 forgetting factor */
static float    p_gamma1       = 0.995f; /* Level-1 per-cluster forgetting factor */
static float    p_gate_eta     = 0.995f; /* Causal OOS Stein gate EMA decay */
static float    p_max_tweak    = 0.25f;  /* Per-mode absolute tweak cap [modal um] */
static float    p_outlier      = 12.0f;  /* Outlier rejection threshold [sigma] */

/* Live runtime toggles (FPFLAG_WRITERUN) for online A/B testing & safety */
static int32_t p_learn_on   = 1;
static int32_t p_shadow     = 0;
static int32_t p_enable_l0  = 1;
static int32_t p_enable_l1  = 1;
static float   p_apply_gain = 1.0f;
static int32_t p_reset      = 0;

/* Live diagnostic outputs */
static uint64_t o_frames         = 0;
static uint64_t o_updates        = 0;
static uint64_t o_outliers       = 0;
static uint32_t o_gric_active    = 0;
static uint32_t o_n_clusters     = 0;
static int32_t  o_last_cid       = -1;
static float    o_last_dist      = 0.0f;
static float    o_alpha_l0       = 0.0f;
static float    o_alpha_l1       = 0.0f;
static float    o_oos_vr_l0_pct  = 0.0f;
static float    o_oos_vr_l1_pct  = 0.0f;
static float    o_oos_vr_tot_pct = 0.0f;
static float    o_tweak_rms      = 0.0f;
static float    o_l1_rms         = 0.0f;

#define FPS_PARAMS_SELFLEARN(X)                                                                   \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".inwfs", p_inwfs, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,   \
      "input WFS modal residual stream")                                                          \
    X(".inol", p_inol, FPTYPE_STREAMNAME, 1,                                                      \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL modal stream")        \
    X(".indm", p_indm, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "optional DM modal command stream")                                                         \
    X(".inbasePF", p_inbasepf, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "optional external Stage-1 PF prediction stream")                                           \
    X(".inbaseStat", p_inbasestat, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "optional external Stage-1 PF status stream [0..1]")                                        \
    X(".inkey", p_inkey, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "optional external 1D key stream (key_mode=5)")                                             \
    X(".outdata", p_outdata, FPTYPE_STREAMNAME, 1,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,                                            \
      "output predicted PF (out_mode=0) or auxDM (out_mode=1)")                                   \
    X(".outPFstat", p_outstat, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "output per-mode active status [0..1] (out_mode=0)")                                        \
    X(".outtweak", p_outtweak, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "output pure learned tweak telemetry stream")                                               \
    X(".savedir", p_savedir, FPTYPE_DIRNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "optional directory to save GRIC anchors on stop")                                          \
    X(".out_mode", &p_out_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "0=PF synchronous PSOL, 1=auxDM additive tweak")                                            \
    X(".nl_mode", &p_nl_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "0=linear phi, 1=+odd PyWFS NL terms, 2=+shared WFS xtalk")                                 \
    X(".key_mode", &p_key_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "0=TT2_E2, 1=E1, 2=WFS_LO4, 3=TT2_DM2_E2, 4=REGIME, 5=EXT")                                 \
    X(".NBmodeMax", &p_nbmodemax, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "max modal dimension tweaked")                                                              \
    X(".kmax", &p_kmax, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                \
      "max GRIC cluster anchors (1..64)")                                                         \
    X(".qshared", &p_qshared, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "shared low-order cross-talk features (0..8)")                                              \
    X(".nwarmup", &p_nwarmup, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "per-cluster shrinkage warmup count")                                                       \
    X(".keywarmup", &p_keywarmup, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "frames to adapt online key standardization")                                               \
    X(".rampsteps", &p_rampsteps, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "PF active-status fade-in ramp steps")                                                      \
    X(".blend_soft", &p_blend_soft, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "soft Gaussian LWPR blending across GRIC anchors")                                          \
    X(".rlim", &p_rlim, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "GRIC cluster spawn radius in standardized key space")                                      \
    X(".gate_mult", &p_gate_mult, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "fail-safe distance gate multiplier (dist <= gate_mult*rlim)")                              \
    X(".blend_bw", &p_blend_bw, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "soft Gaussian kernel bandwidth multiplier vs rlim")                                        \
    X(".tm_mix", &p_tm_mix, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "GRIC Markov temporal transition mixing coeff")                                             \
    X(".latency", &p_latency, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,         \
      "causal prediction horizon [frames]")                                                       \
    X(".dc_leak", &p_dc_leak, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,         \
      "stabilizing DC leak (<1.0) on standalone PSOL base")                                       \
    X(".aux_leak", &p_aux_leak, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "damping leak (<1.0) in auxDM counterfactual target")                                       \
    X(".wfs_to_dm", &p_wfs_to_dm, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "nominal WFS-to-DM factor (auxDM mode)")                                                    \
    X(".max_wfs_gain", &p_max_wfs_gain, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "physical stability cap on WFS weight")            \
    X(".max_ar_norm2", &p_max_ar_norm2, FPTYPE_FLOAT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "L2 norm^2 cap on AR increment filter")            \
    X(".lam0", &p_lam0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "Level-0 global RLS prior ridge")                                                           \
    X(".lam1", &p_lam1, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "Level-1 per-cluster RLS prior ridge")                                                      \
    X(".gamma0", &p_gamma0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "Level-0 RLS exponential forgetting factor")                                                \
    X(".gamma1", &p_gamma1, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "Level-1 RLS exponential forgetting factor")                                                \
    X(".gate_eta", &p_gate_eta, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,       \
      "causal OOS Stein gate EMA decay")                                                          \
    X(".max_tweak", &p_max_tweak, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "hard per-mode absolute tweak clamp [modal units]")                                         \
    X(".outlier_sig", &p_outlier, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "outlier update rejection threshold [sigma]")                                               \
    X(".learnON", &p_learn_on, FPTYPE_ONOFF, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "enable continuous online learning")                                                        \
    X(".shadow", &p_shadow, FPTYPE_ONOFF, 0,                                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "shadow mode: learn & score OOS without applying tweak")                                    \
    X(".enable_l0", &p_enable_l0, FPTYPE_ONOFF, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "apply Level-0 global online RLS correction")                                               \
    X(".enable_l1", &p_enable_l1, FPTYPE_ONOFF, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "apply Level-1 GRIC cluster-specific correction")                                           \
    X(".apply_gain", &p_apply_gain, FPTYPE_FLOAT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "live scalar blend multiplier [0..1] on tweak")                                             \
    X(".reset", &p_reset, FPTYPE_ONOFF, 0,                                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                  \
      "one-shot reset of GRIC clusters and RLS state")                                            \
    X(".out.frames", &o_frames, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT, "frames processed")      \
    X(".out.updates", &o_updates, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "causal delayed RLS updates")                                                               \
    X(".out.outliers", &o_outliers, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "outlier frames skipped")                                                                   \
    X(".out.gric_active", &o_gric_active, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "1 if libgric C API is active")                                                             \
    X(".out.n_clusters", &o_n_clusters, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "active GRIC cluster count")                                                                \
    X(".out.last_cid", &o_last_cid, FPTYPE_INT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "last assigned cluster ID")                                                                 \
    X(".out.last_dist", &o_last_dist, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "last key distance to assigned anchor")                                                     \
    X(".out.alpha_l0", &o_alpha_l0, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "live causal OOS Stein gate for Level 0 [0..1]")                                            \
    X(".out.alpha_l1", &o_alpha_l1, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "live causal OOS Stein gate for Level 1 [0..1]")                                            \
    X(".out.oos_vr_l0_pct", &o_oos_vr_l0_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "causal OOS Level-0 variance reduction [%]")                                                \
    X(".out.oos_vr_l1_pct", &o_oos_vr_l1_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "causal OOS Level-1 extra variance reduction over L0 [%]")                                  \
    X(".out.oos_vr_tot_pct", &o_oos_vr_tot_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,         \
      "causal OOS total variance reduction [%]")                                                  \
    X(".out.tweak_rms", &o_tweak_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "instantaneous applied tweak RMS")                                                          \
    X(".out.l1_rms", &o_l1_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "instantaneous Level-1 cluster contribution RMS")

MILK_PARAMS_DEFINE(cacaopc_selflearn, FPS_PARAMS_SELFLEARN);

static MILK_HOT errno_t selflearn_compute(const MILK_RUNNABLE *self)
{
    IMGID imgwfs = imgid_make_from_fpskey(p_inwfs, dcfpsname, ".inwfs");
    resolveIMGID(&imgwfs, ERRMODE_WARN, dcimg, dcnimg);
    if (imgwfs.ID == -1)
    {
        return RETURN_FAILURE;
    }

    IMGID imgol = imgid_make_from_fpskey(p_inol, dcfpsname, ".inol");
    resolveIMGID(&imgol, ERRMODE_WARN, dcimg, dcnimg);
    if (imgol.ID == -1 && p_out_mode == 0u)
    {
        return RETURN_FAILURE;
    }

    IMGID imgdm = imgid_make_from_name("");
    if (p_indm[0] != '\0')
    {
        imgdm = imgid_make_from_fpskey(p_indm, dcfpsname, ".indm");
        resolveIMGID(&imgdm, ERRMODE_NULL, dcimg, dcnimg);
    }

    IMGID imgbase = imgid_make_from_name("");
    if (p_inbasepf[0] != '\0')
    {
        imgbase = imgid_make_from_fpskey(p_inbasepf, dcfpsname, ".inbasePF");
        resolveIMGID(&imgbase, ERRMODE_NULL, dcimg, dcnimg);
    }

    IMGID imgbasestat = imgid_make_from_name("");
    if (p_inbasestat[0] != '\0')
    {
        imgbasestat = imgid_make_from_fpskey(p_inbasestat, dcfpsname, ".inbaseStat");
        resolveIMGID(&imgbasestat, ERRMODE_NULL, dcimg, dcnimg);
    }

    IMGID imgkey = imgid_make_from_name("");
    uint32_t ext_dim = 0u;
    if (p_key_mode == (uint32_t) SL_KEY_EXTERNAL && p_inkey[0] != '\0')
    {
        imgkey = imgid_make_from_fpskey(p_inkey, dcfpsname, ".inkey");
        resolveIMGID(&imgkey, ERRMODE_WARN, dcimg, dcnimg);
        if (imgkey.ID != -1)
        {
            ext_dim = imgkey.md[0].size[0] *
                      ((imgkey.md[0].naxis >= 2) ? imgkey.md[0].size[1] : 1u);
        }
    }

    uint32_t NBmode = imgwfs.md[0].size[0] *
                      ((imgwfs.md[0].naxis >= 2) ? imgwfs.md[0].size[1] : 1u);
    if (NBmode == 0u)
    {
        return RETURN_FAILURE;
    }
    uint32_t eff_modes = (p_nbmodemax > 0u && p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;

    IMGID imgout   = stream_connect_create_2Df32(p_outdata, NBmode, 1);
    IMGID imgtweak = stream_connect_create_2Df32(p_outtweak, NBmode, 1);
    IMGID imgstat  = imgid_make_from_name("");
    if (p_out_mode == 0u && p_outstat[0] != '\0')
    {
        imgstat = stream_connect_create_2Df32(p_outstat, NBmode, 1);
        memset(imgstat.im->array.F, 0, sizeof(float) * NBmode);
        ImageStreamIO_UpdateIm(imgstat.im);
    }
    memset(imgout.im->array.F, 0, sizeof(float) * NBmode);
    memset(imgtweak.im->array.F, 0, sizeof(float) * NBmode);

    SL_CONFIG cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.n_modes     = eff_modes;
    cfg.key_mode    = p_key_mode;
    cfg.ext_key_dim = ext_dim;
    cfg.k_max       = p_kmax;
    cfg.q_shared    = p_qshared;
    cfg.n_warmup    = p_nwarmup;
    cfg.key_warmup  = p_keywarmup;
    cfg.blend_soft  = (p_blend_soft != 0) ? 1u : 0u;
    cfg.out_mode    = p_out_mode;
    cfg.nl_mode     = p_nl_mode;
    cfg.rlim        = (double) p_rlim;
    cfg.gate_mult   = (double) p_gate_mult;
    cfg.blend_bw    = (double) p_blend_bw;
    cfg.tm_mix      = (double) p_tm_mix;
    cfg.latency_fr   = (double) p_latency;
    cfg.dc_leak      = (double) p_dc_leak;
    cfg.aux_leak     = (double) p_aux_leak;
    cfg.wfs_to_dm    = (double) p_wfs_to_dm;
    cfg.max_wfs_gain = (double) p_max_wfs_gain;
    cfg.max_ar_norm2 = (double) p_max_ar_norm2;
    cfg.lam0        = (double) p_lam0;
    cfg.lam1        = (double) p_lam1;
    cfg.gamma0      = (double) p_gamma0;
    cfg.gamma1      = (double) p_gamma1;
    cfg.gate_eta    = (double) p_gate_eta;
    cfg.max_tweak   = (double) p_max_tweak;
    cfg.outlier_sig = (double) p_outlier;

    SL_ENGINE *eng = sl_engine_create(&cfg);
    if (eng == NULL)
    {
        fprintf(stderr, "cacaopc-selflearn: failed to create engine (M=%u, K=%u)\n",
                eff_modes, p_kmax);
        return RETURN_FAILURE;
    }

    float *out_twk_buf = (float *) calloc(NBmode, sizeof(float));
    float *out_tot_buf = (float *) calloc(NBmode, sizeof(float));
    if (out_twk_buf == NULL || out_tot_buf == NULL)
    {
        free(out_twk_buf);
        free(out_tot_buf);
        sl_engine_destroy(eng);
        return RETURN_FAILURE;
    }

    float prev_stat = -1.0f;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        if (p_reset != 0)
        {
            sl_engine_reset(eng);
            p_reset   = 0;
            prev_stat = -1.0f;
        }

        const float *wfs_ptr  = imgwfs.im->array.F;
        const float *ol_ptr   = (imgol.ID != -1) ? imgol.im->array.F : NULL;
        const float *dm_ptr   = (imgdm.ID != -1) ? imgdm.im->array.F : NULL;
        const float *base_ptr = (imgbase.ID != -1) ? imgbase.im->array.F : NULL;
        const float *key_ptr  = (imgkey.ID != -1) ? imgkey.im->array.F : NULL;

        /* Pass through untouched higher modes [eff_modes..NBmode-1] */
        if (eff_modes < NBmode)
        {
            memset(out_twk_buf + eff_modes, 0, sizeof(float) * (NBmode - eff_modes));
            if (p_out_mode == 0u)
            {
                const float *hi_src = (base_ptr != NULL) ? base_ptr : ol_ptr;
                if (hi_src != NULL)
                {
                    memcpy(out_tot_buf + eff_modes, hi_src + eff_modes,
                           sizeof(float) * (NBmode - eff_modes));
                }
                else
                {
                    memset(out_tot_buf + eff_modes, 0, sizeof(float) * (NBmode - eff_modes));
                }
            }
            else
            {
                memset(out_tot_buf + eff_modes, 0, sizeof(float) * (NBmode - eff_modes));
            }
        }

        SL_DIAG diag;
        sl_engine_step(eng, wfs_ptr, dm_ptr, ol_ptr, base_ptr, key_ptr,
                       p_learn_on != 0, p_shadow != 0, p_enable_l0 != 0, p_enable_l1 != 0,
                       p_apply_gain, out_twk_buf, out_tot_buf, &diag);

        o_frames         = diag.frames;
        o_updates        = diag.updates;
        o_outliers       = diag.outliers_skipped;
        o_gric_active    = diag.gric_active;
        o_n_clusters     = diag.n_clusters;
        o_last_cid       = diag.last_cid;
        o_last_dist      = diag.last_dist;
        o_alpha_l0       = diag.alpha_l0;
        o_alpha_l1       = diag.alpha_l1;
        o_oos_vr_l0_pct  = diag.oos_vr_l0_pct;
        o_oos_vr_l1_pct  = diag.oos_vr_l1_pct;
        o_oos_vr_tot_pct = diag.oos_vr_tot_pct;
        o_tweak_rms      = diag.tweak_rms;
        o_l1_rms         = diag.l1_rms;

        if (p_out_mode == 0u && imgstat.ID != -1)
        {
            if (imgbasestat.ID != -1)
            {
                imgstat.md[0].write = 1;
                memcpy(imgstat.im->array.F, imgbasestat.im->array.F, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgstat.im, NULL);
            }
            else
            {
                uint32_t rsteps = (p_rampsteps > 0u) ? p_rampsteps : 1u;
                float    sval   = (diag.frames >= (uint64_t) rsteps)
                                      ? 1.0f
                                      : ((float) diag.frames / (float) rsteps);
                if (base_ptr != NULL && p_shadow != 0)
                {
                    sval = 1.0f;
                }
                if (sval != prev_stat)
                {
                    prev_stat           = sval;
                    imgstat.md[0].write = 1;
                    uint32_t active_m   = (base_ptr != NULL) ? NBmode : eff_modes;
                    for (uint32_t m = 0; m < active_m; m++)
                    {
                        imgstat.im->array.F[m] = sval;
                    }
                    for (uint32_t m = active_m; m < NBmode; m++)
                    {
                        imgstat.im->array.F[m] = 0.0f;
                    }
                    processinfo_update_output_stream(processinfo, imgstat.im, NULL);
                }
            }
        }

        imgtweak.md[0].write = 1;
        memcpy(imgtweak.im->array.F, out_twk_buf, sizeof(float) * NBmode);
        processinfo_update_output_stream(processinfo, imgtweak.im, NULL);

        imgout.md[0].write = 1;
        memcpy(imgout.im->array.F, out_tot_buf, sizeof(float) * NBmode);
        processinfo_update_output_stream(processinfo, imgout.im, NULL);
    }
    MILK_PROCINFO_END(self)

    if (p_savedir[0] != '\0')
    {
        (void) sl_engine_save_gric(eng, p_savedir);
    }

    free(out_twk_buf);
    free(out_tot_buf);
    sl_engine_destroy(eng);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_selflearn = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "selflearn",
                 .cmdkey      = "selflearn",
                 .description = "online GRIC-clustered self-learning AO tweak controller",
                 .description_long =
                     "Online state-space clustering via libgric paired with 2-level hierarchical "
                     "factorized forgetting RLS and causal OOS Stein gating, learning and "
                     "exercising bounded corrective tweaks in real time." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaopc_selflearn),
    .compute = selflearn_compute,
};
EXPORT_FPSEXEC(cacao, "cacaopc-selflearn", runnable_AOloopControl_PredictiveControl_selflearn);
