// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl_mkARPF.c
 * @brief   Online Wiener / ridge auto-regressive predictive filter builder from PSOL telemetry
 */

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"

#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"

#define ARPF_MAX_ORDER 64

static uint64_t p_loopindex                             = 0;
static char     p_inbuff[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalOL_buff";
static char     p_outPF[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_arPFmat";
static uint32_t p_pforder                               = 8;
static float    p_pflatency                             = 2.3f;
static uint32_t p_nbtrain                               = 2048;
static uint32_t p_minsamples                            = 512;
static uint32_t p_nbmodemax                             = 4096;
static float    p_reglambda                             = 0.005f;
static float    p_priorgain                             = 0.00f;
static float    p_priordecay                            = 0.85f;
static float    p_maxnorm2                              = 1.35f;
static float    p_maxdcgain                             = 0.985f;
static float    p_loopgain                              = 0.30f;

static uint64_t o_n_updates   = 0;
static uint32_t o_n_samples   = 0;
static float    o_rms_ol      = 0.0f;
static float    o_rms_zoh     = 0.0f;
static float    o_rms_pred    = 0.0f;
static float    o_var_red_pct = 0.0f;

#define FPS_PARAMS_MKARPF(X)                                                                      \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".inbuff", p_inbuff, FPTYPE_STREAMNAME, 1,                                                  \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL telemetry buffer")    \
    X(".outPF", p_outPF, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,   \
      "output AR predictive filter [NBmode x PForder]")                                           \
    X(".PForder", &p_pforder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "AR filter tap order (1..64)")                                                              \
    X(".PFlatency", &p_pflatency, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "prediction horizon [frames]")   \
    X(".NBtrain", &p_nbtrain, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "sliding PSOL training window [frames]")                                                    \
    X(".minsamples", &p_minsamples, FPTYPE_UINT32, 0,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min samples before 1st solve")  \
    X(".NBmodeMax", &p_nbmodemax, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max modes predicted")           \
    X(".reglambda", &p_reglambda, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "relative Tikhonov damping")     \
    X(".priorgain", &p_priorgain, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "DC gain of low-pass prior")     \
    X(".priordecay", &p_priordecay, FPTYPE_FLOAT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "tap decay of low-pass prior")   \
    X(".maxnorm2", &p_maxnorm2, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max ||w||_2^2 noise-gain cap")  \
    X(".maxdcgain", &p_maxdcgain, FPTYPE_FLOAT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max |sum(w)| DC gain (<1)")     \
    X(".loopgain", &p_loopgain, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "filter EMA update gain")        \
    X(".out.n_updates", &o_n_updates, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "completed filter updates")                                                                 \
    X(".out.n_samples", &o_n_samples, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "active PSOL training samples")                                                             \
    X(".out.rms_ol", &o_rms_ol, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "quadrature open-loop modal RMS")                                                           \
    X(".out.rms_zoh", &o_rms_zoh, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "quadrature pure-lag modal RMS")                                                            \
    X(".out.rms_pred", &o_rms_pred, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "quadrature AR predicted modal RMS")                                                        \
    X(".out.var_red_pct", &o_var_red_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "variance reduction vs pure lag [%]")

MILK_PARAMS_DEFINE(cacaopc_mkARPF, FPS_PARAMS_MKARPF);

/**
 * @brief Solve symmetric positive-definite P x P system A * x = b via Cholesky in double precision.
 */
static int solve_spd_small(uint32_t P, const double *A, const double *b, double *x)
{
    double L[ARPF_MAX_ORDER * ARPF_MAX_ORDER];
    double y[ARPF_MAX_ORDER];

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

static void build_prior_weights(uint32_t P, float dc_gain, float decay, double *w_prior)
{
    if (dc_gain <= 0.0f)
    {
        for (uint32_t k = 0; k < P; k++)
        {
            w_prior[k] = 0.0;
        }
        return;
    }
    double d   = (decay > 0.05f && decay < 0.999f) ? (double) decay : 0.85;
    double sum = 0.0;
    double cur = 1.0;
    for (uint32_t k = 0; k < P; k++)
    {
        w_prior[k] = cur;
        sum += cur;
        cur *= d;
    }
    double scale = (double) dc_gain / (sum > 1.0e-12 ? sum : 1.0);
    for (uint32_t k = 0; k < P; k++)
    {
        w_prior[k] *= scale;
    }
}

static MILK_HOT errno_t mkARPF_compute(const MILK_RUNNABLE *self)
{
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
    if (P > ARPF_MAX_ORDER)
    {
        P = ARPF_MAX_ORDER;
    }

    uint32_t cap_train = (p_nbtrain >= 128) ? p_nbtrain : 2048;
    IMGID    imgoutPF  = stream_connect_create_2Df32(p_outPF, NBmode, P);

    // Store mode-major [NBmode, cap_train] so each mode's history is contiguous in L1/L2 cache
    float *hist_mm = (float *) calloc((size_t) NBmode * cap_train, sizeof(float));
    float *w_stage = (float *) calloc((size_t) NBmode * P, sizeof(float));
    float *tmp_in  = NULL;
    if (!hist_mm || !w_stage)
    {
        free(hist_mm);
        free(w_stage);
        return RETURN_FAILURE;
    }

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
    tmp_in = (float *) calloc((size_t) NBmode * max_slice_frames, sizeof(float));
    if (!tmp_in)
    {
        free(hist_mm);
        free(w_stage);
        return RETURN_FAILURE;
    }

    uint32_t n_valid    = 0;
    o_n_updates         = 0;
    o_n_samples         = 0;
    o_rms_ol            = 0.0f;
    o_rms_zoh           = 0.0f;
    o_rms_pred          = 0.0f;
    o_var_red_pct       = 0.0f;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        // 1. Snapshot incoming telemetry slice
        uint32_t nslice = 1;
        const float *src_ptr = imgin.im->array.F;
        if (imgin.md[0].naxis == 3 && imgin.md[0].size[2] == 2 && imgin.md[0].size[1] > 1)
        {
            nslice       = imgin.md[0].size[1];
            uint32_t sl  = (uint32_t) (imgin.md[0].cnt1 & 1ULL);
            src_ptr      = &imgin.im->array.F[(size_t) sl * nslice * NBmode];
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
        memcpy(tmp_in, src_ptr, sizeof(float) * NBmode * nslice);

        // Filter out leading dead/zero frames (e.g. before loop closes)
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
#pragma omp parallel for schedule(static)
                for (uint32_t m = 0; m < NBmode; m++)
                {
                    float *hm = &hist_mm[(size_t) m * cap_train];
                    memmove(hm, &hm[drop], sizeof(float) * keep);
                }
                n_valid = keep;
            }

#pragma omp parallel for schedule(static)
            for (uint32_t m = 0; m < NBmode; m++)
            {
                float *hm = &hist_mm[(size_t) m * cap_train + n_valid];
                for (uint32_t t = 0; t < n_add; t++)
                {
                    hm[t] = tmp_in[(size_t) (first_nz + t) * NBmode + m];
                }
            }
            n_valid += n_add;
            o_n_samples = n_valid;
        }

        float    lat_fr  = (p_pflatency >= 0.05f && p_pflatency <= 24.0f) ? p_pflatency : 2.3f;
        uint32_t lat_i   = (uint32_t) lat_fr;
        double   lat_a   = (double) lat_fr - (double) lat_i;
        uint32_t min_req = (p_minsamples >= P + lat_i + 16) ? p_minsamples : (P + lat_i + 16);

        if (n_valid >= min_req && n_valid > P + lat_i + 4)
        {
            uint32_t eff_modes = (p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;
            uint32_t t0_idx    = P - 1;
            uint32_t t1_idx    = n_valid - lat_i - 1;
            uint32_t nrows     = (t1_idx > t0_idx) ? (t1_idx - t0_idx) : 0;

            if (nrows >= 16)
            {
                double w_prior[ARPF_MAX_ORDER];
                build_prior_weights(P, p_priorgain, p_priordecay, w_prior);

                double reg_rel   = (p_reglambda > 1.0e-8f) ? (double) p_reglambda : 1.0e-5;
                double max_norm2 = (p_maxnorm2 > 0.1f) ? (double) p_maxnorm2 : 1.35;
                double max_dc    = (p_maxdcgain > 0.1f && p_maxdcgain < 0.999f)
                                       ? (double) p_maxdcgain
                                       : 0.985;

                double sum_var_ol   = 0.0;
                double sum_var_zoh  = 0.0;
                double sum_var_pred = 0.0;

                memset(w_stage, 0, sizeof(float) * NBmode * P);

#pragma omp parallel for schedule(static) reduction(+ : sum_var_ol, sum_var_zoh, sum_var_pred)
                for (uint32_t m = 0; m < eff_modes; m++)
                {
                    const float *restrict ts = &hist_mm[(size_t) m * cap_train];
                    double G[ARPF_MAX_ORDER * ARPF_MAX_ORDER];
                    double rhs[ARPF_MAX_ORDER];
                    double A[ARPF_MAX_ORDER * ARPF_MAX_ORDER];
                    double b_reg[ARPF_MAX_ORDER];
                    double w_sol[ARPF_MAX_ORDER];

                    memset(G, 0, sizeof(double) * P * P);
                    memset(rhs, 0, sizeof(double) * P);

                    double s_y    = 0.0;
                    double s_y2   = 0.0;
                    double s_zoh2 = 0.0;

                    for (uint32_t r = 0; r < nrows; r++)
                    {
                        uint32_t tk = t0_idx + r;
                        double   yk = (1.0 - lat_a) * (double) ts[tk + lat_i] +
                                      lat_a * (double) ts[tk + lat_i + 1];
                        double x0   = (double) ts[tk];
                        double dz   = yk - x0;
                        s_y    += yk;
                        s_y2   += yk * yk;
                        s_zoh2 += dz * dz;

                        for (uint32_t i = 0; i < P; i++)
                        {
                            double xi = (double) ts[tk - i];
                            rhs[i] += xi * yk;
                            for (uint32_t j = 0; j <= i; j++)
                            {
                                G[i * P + j] += xi * (double) ts[tk - j];
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

                    if (tr <= 1.0e-26)
                    {
                        for (uint32_t k = 0; k < P; k++)
                        {
                            w_sol[k] = w_prior[k];
                        }
                    }
                    else
                    {
                        double lam = reg_rel * tr;
                        int    ok  = 0;
                        for (int attempt = 0; attempt < 12; attempt++)
                        {
                            for (uint32_t i = 0; i < P * P; i++)
                            {
                                A[i] = G[i];
                            }
                            for (uint32_t i = 0; i < P; i++)
                            {
                                A[i * P + i] += lam;
                                b_reg[i]      = rhs[i] + lam * w_prior[i];
                            }
                            if (!solve_spd_small(P, A, b_reg, w_sol))
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
                            for (uint32_t k = 0; k < P; k++)
                            {
                                w_sol[k] = w_prior[k];
                            }
                        }
                    }

                    // Strict DC gain cap (< 1.0) prevents positive-feedback drift on null modes
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

                    // Quadratic form residual: y'y - 2 w' rhs + w' G w
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

                    // Dynamic (demeaned) variance comparison against ZOH
                    double y_mean    = s_y / (double) nrows;
                    double s_ydyn2   = s_y2 - (double) nrows * y_mean * y_mean;
                    if (s_ydyn2 < 0.0)
                    {
                        s_ydyn2 = 0.0;
                    }
                    if (s_pred2 > s_y2 && s_pred2 > s_zoh2)
                    {
                        for (uint32_t k = 0; k < P; k++)
                        {
                            w_sol[k] = 0.0;
                        }
                        s_pred2 = s_y2;
                    }

                    sum_var_ol   += s_ydyn2 / (double) nrows;
                    sum_var_zoh  += s_zoh2 / (double) nrows;
                    sum_var_pred += s_pred2 / (double) nrows;

                    for (uint32_t k = 0; k < P; k++)
                    {
                        w_stage[(size_t) k * NBmode + m] = (float) w_sol[k];
                    }
                }

                float g_cfg = (p_loopgain > 0.0f && p_loopgain <= 1.0f) ? p_loopgain : 0.30f;
                float g_upd = (o_n_updates == 0) ? 1.0f : g_cfg;
                float g_old = 1.0f - g_upd;

                imgoutPF.md[0].write = 1;
                float *restrict pf_dst = imgoutPF.im->array.F;
                size_t n_elem          = (size_t) NBmode * P;
                for (size_t idx = 0; idx < n_elem; idx++)
                {
                    pf_dst[idx] = g_old * pf_dst[idx] + g_upd * w_stage[idx];
                }
                processinfo_update_output_stream(processinfo, imgoutPF.im, NULL);

                o_n_updates++;
                o_rms_ol   = (float) sqrt(sum_var_ol);
                o_rms_zoh  = (float) sqrt(sum_var_zoh);
                o_rms_pred = (float) sqrt(sum_var_pred);
                o_var_red_pct = (sum_var_zoh > 1.0e-24)
                                    ? (float) ((1.0 - sum_var_pred / sum_var_zoh) * 100.0)
                                    : 0.0f;

                processinfo_WriteMessage_fmt(processinfo, "PF#%lu N=%u lag=%.2f dVar=%.1f%%",
                                             o_n_updates, n_valid, lat_fr, o_var_red_pct);
            }
        }
    }
    MILK_PROCINFO_END(self)

    free(tmp_in);
    free(w_stage);
    free(hist_mm);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_mkARPF = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "mkARPF",
                 .cmdkey      = "mkARPF",
                 .description = "build modal AR predictive filter from PSOL telemetry",
                 .description_long =
                     "Continuously learn regularized Wiener auto-regressive (AR) modal predictive "
                     "filters from pseudo-open-loop (PSOL) telemetry buffers." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaopc_mkARPF),
    .compute = mkARPF_compute,
};
EXPORT_FPSEXEC(cacao, "cacaopc-mkARPF", runnable_AOloopControl_PredictiveControl_mkARPF);
