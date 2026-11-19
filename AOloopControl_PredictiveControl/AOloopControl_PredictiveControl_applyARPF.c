// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl_applyARPF.c
 * @brief   Low-latency SIMD real-time application of modal AR predictive filter on PSOL stream
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

static uint64_t p_loopindex                              = 0;
static char     p_indata[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_modevalOL";
static char     p_PFmat[FUNCTION_PARAMETER_STRMAXLEN]    = "aol0_arPFmat";
static char     p_outdata[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalPF";
static char     p_outstat[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_outPFstat";
static uint32_t p_pforder                                = 8;
static uint32_t p_nbmodemax                              = 4096;
static uint32_t p_rampsteps                              = 64;

static uint64_t o_active_frames = 0;

#define FPS_PARAMS_APPLYARPF(X)                                                                   \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".indata", p_indata, FPTYPE_STREAMNAME, 1,                                                  \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL mode stream")         \
    X(".PFmat", p_PFmat, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,   \
      "input AR predictive filter [NBmode x PForder]")                                            \
    X(".outdata", p_outdata, FPTYPE_STREAMNAME, 1,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "output predicted mode stream")            \
    X(".outPFstat", p_outstat, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "output per-mode active status [0..1]")                                                     \
    X(".PForder", &p_pforder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "AR filter tap order (1..64)")                                                              \
    X(".NBmodeMax", &p_nbmodemax, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max modes predicted")           \
    X(".rampsteps", &p_rampsteps, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fade-in ramp steps")            \
    X(".out.active_frames", &o_active_frames, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,            \
      "frames predicted with trained filter")

MILK_PARAMS_DEFINE(cacaopc_applyARPF, FPS_PARAMS_APPLYARPF);

static MILK_HOT errno_t applyARPF_compute(const MILK_RUNNABLE *self)
{
    IMGID imgin = imgid_make_from_fpskey(p_indata, dcfpsname, ".indata");
    resolveIMGID(&imgin, ERRMODE_WARN, dcimg, dcnimg);
    if (imgin.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode = imgin.md[0].size[0] * imgin.md[0].size[1];
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

    IMGID imgPFmat = stream_connect_create_2Df32(p_PFmat, NBmode, P);
    if (imgPFmat.md[0].size[1] >= 1 && imgPFmat.md[0].size[1] <= ARPF_MAX_ORDER)
    {
        P = imgPFmat.md[0].size[1];
    }

    IMGID imgout     = stream_connect_create_2Df32(p_outdata, NBmode, 1);
    IMGID imgoutstat = stream_connect_create_2Df32(p_outstat, NBmode, 1);

    memset(imgout.im->array.F, 0, sizeof(float) * NBmode);
    memset(imgoutstat.im->array.F, 0, sizeof(float) * NBmode);
    ImageStreamIO_UpdateIm(imgoutstat.im);

    float *hist_ring = NULL;
    if (posix_memalign((void **) &hist_ring, 64, sizeof(float) * NBmode * P) != 0 ||
        hist_ring == NULL)
    {
        return RETURN_FAILURE;
    }
    memset(hist_ring, 0, sizeof(float) * NBmode * P);

    float *out_buf = NULL;
    if (posix_memalign((void **) &out_buf, 64, sizeof(float) * NBmode) != 0 || out_buf == NULL)
    {
        free(hist_ring);
        return RETURN_FAILURE;
    }
    memset(out_buf, 0, sizeof(float) * NBmode);

    uint32_t head          = 0;
    uint64_t n_hist        = 0;
    uint32_t prev_effmodes = 0;
    float    prev_stat_val = -1.0f;
    o_active_frames        = 0;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        // 1. Insert latest PSOL modal vector into circular tap buffer
        memcpy(&hist_ring[(size_t) head * NBmode], imgin.im->array.F, sizeof(float) * NBmode);
        n_hist++;

        uint32_t eff_modes = (p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;
        int      pf_ready  = (imgPFmat.md[0].cnt0 > 0) && (n_hist >= (uint64_t) P);

        if (pf_ready && eff_modes > 0)
        {
            memset(out_buf, 0, sizeof(float) * NBmode);
            const float *restrict pf_w = imgPFmat.im->array.F;

            for (uint32_t k = 0; k < P; k++)
            {
                uint32_t slot              = (head + P - k) % P;
                const float *restrict wk   = &pf_w[(size_t) k * NBmode];
                const float *restrict hk   = &hist_ring[(size_t) slot * NBmode];
#pragma omp simd
                for (uint32_t m = 0; m < eff_modes; m++)
                {
                    out_buf[m] += wk[m] * hk[m];
                }
            }

            o_active_frames++;
            uint32_t rsteps   = (p_rampsteps > 0) ? p_rampsteps : 1;
            float    stat_val = (o_active_frames >= (uint64_t) rsteps)
                                    ? 1.0f
                                    : ((float) o_active_frames / (float) rsteps);

            if (stat_val != prev_stat_val || eff_modes != prev_effmodes)
            {
                prev_stat_val = stat_val;
                prev_effmodes = eff_modes;
                imgoutstat.md[0].write = 1;
                for (uint32_t m = 0; m < eff_modes; m++)
                {
                    imgoutstat.im->array.F[m] = stat_val;
                }
                for (uint32_t m = eff_modes; m < NBmode; m++)
                {
                    imgoutstat.im->array.F[m] = 0.0f;
                }
                processinfo_update_output_stream(processinfo, imgoutstat.im, NULL);
            }

            imgout.md[0].write = 1;
            memcpy(imgout.im->array.F, out_buf, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imgout.im, NULL);
        }
        else
        {
            if (prev_stat_val != 0.0f)
            {
                prev_stat_val          = 0.0f;
                prev_effmodes          = 0;
                imgoutstat.md[0].write = 1;
                memset(imgoutstat.im->array.F, 0, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgoutstat.im, NULL);
            }
            imgout.md[0].write = 1;
            memset(imgout.im->array.F, 0, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imgout.im, NULL);
        }

        head = (head + 1) % P;
    }
    MILK_PROCINFO_END(self)

    free(out_buf);
    free(hist_ring);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_applyARPF = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "applyARPF",
                 .cmdkey      = "applyARPF",
                 .description = "apply modal AR predictive filter in real time",
                 .description_long =
                     "Apply learned Wiener auto-regressive (AR) modal predictive filter to live "
                     "pseudo-open-loop (PSOL) modes and publish predicted modal values." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaopc_applyARPF),
    .compute = applyARPF_compute,
};
EXPORT_FPSEXEC(cacao, "cacaopc-applyARPF", runnable_AOloopControl_PredictiveControl_applyARPF);
