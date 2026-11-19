// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl_applyEOFPF.c
 * @brief   Low-latency SIMD/OpenMP real-time application of block cross-mode EOF filter
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

#define EOFPF_MAX_ORDER 64
#define EOFPF_MAX_BLOCK 512

static uint64_t p_loopindex                             = 0;
static char     p_indata[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalOL";
static char     p_PFmat[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_eofPFmat";
static char     p_outdata[FUNCTION_PARAMETER_STRMAXLEN] = "aol0_modevalPF";
static char     p_outstat[FUNCTION_PARAMETER_STRMAXLEN] = "aol0_outPFstat";
static uint32_t p_pforder                               = 8;
static uint32_t p_diagorder                             = 16;
static uint32_t p_blocksize                             = 128;
static uint32_t p_nbmodemax                             = 4096;
static uint32_t p_rampsteps                             = 64;
static uint32_t p_nthreads                              = 4;

static uint64_t o_active_frames = 0;

#define FPS_PARAMS_APPLYEOFPF(X)                                                                  \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".indata", p_indata, FPTYPE_STREAMNAME, 1,                                                  \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input PSOL mode stream")         \
    X(".PFmat", p_PFmat, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT,   \
      "input block-EOF matrix [(PForder*B) x NBmode]")                                            \
    X(".outdata", p_outdata, FPTYPE_STREAMNAME, 1,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "output predicted mode stream")            \
    X(".outPFstat", p_outstat, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "output per-mode active status [0..1]")                                                     \
    X(".PForder", &p_pforder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "EOF lookback tap order (1..64)")                                                           \
    X(".diagorder", &p_diagorder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "bootstrap diag AR order (1..64)")                                                          \
    X(".blocksize", &p_blocksize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "coupled mode block size B (1..512)")                                                       \
    X(".NBmodeMax", &p_nbmodemax, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max modes predicted")           \
    X(".rampsteps", &p_rampsteps, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fade-in ramp steps")            \
    X(".nthreads", &p_nthreads, FPTYPE_UINT32, 0,                                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "OpenMP threads across blocks")  \
    X(".out.active_frames", &o_active_frames, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,            \
      "frames predicted with trained filter")

MILK_PARAMS_DEFINE(cacaopc_applyEOFPF, FPS_PARAMS_APPLYEOFPF);

static MILK_HOT errno_t applyEOFPF_compute(const MILK_RUNNABLE *self)
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

    uint32_t pb_len   = P * B;
    uint32_t stride   = pb_len + diag_tail;
    IMGID    imgPFmat = stream_connect_create_2Df32(p_PFmat, stride, NBmode);
    uint32_t mat_cols = imgPFmat.md[0].size[0];
    if (mat_cols >= pb_len)
    {
        uint32_t dt = mat_cols - pb_len;
        if (P + dt <= EOFPF_MAX_ORDER)
        {
            diag_tail = dt;
            P_diag    = P + diag_tail;
            stride    = mat_cols;
        }
    }
    else if (mat_cols >= B && (mat_cols % B == 0))
    {
        uint32_t p_live = mat_cols / B;
        if (p_live >= 1 && p_live <= EOFPF_MAX_ORDER)
        {
            P         = p_live;
            P_diag    = P;
            diag_tail = 0;
            pb_len    = P * B;
            stride    = pb_len;
        }
    }
    uint32_t P_ring = P_diag;

    IMGID imgout     = stream_connect_create_2Df32(p_outdata, NBmode, 1);
    IMGID imgoutstat = stream_connect_create_2Df32(p_outstat, NBmode, 1);

    memset(imgout.im->array.F, 0, sizeof(float) * NBmode);
    memset(imgoutstat.im->array.F, 0, sizeof(float) * NBmode);
    ImageStreamIO_UpdateIm(imgoutstat.im);

    float *hist_ring = NULL;
    if (posix_memalign((void **) &hist_ring, 64, sizeof(float) * (size_t) NBmode * P_ring) != 0 ||
        hist_ring == NULL)
    {
        return RETURN_FAILURE;
    }
    memset(hist_ring, 0, sizeof(float) * (size_t) NBmode * P_ring);

    float *out_buf = NULL;
    if (posix_memalign((void **) &out_buf, 64, sizeof(float) * NBmode) != 0 || out_buf == NULL)
    {
        free(hist_ring);
        return RETURN_FAILURE;
    }
    memset(out_buf, 0, sizeof(float) * NBmode);

    uint32_t max_blocks = (NBmode + B - 1) / B;
    float   *x_pack     = NULL;
    if (posix_memalign((void **) &x_pack, 64, sizeof(float) * (size_t) max_blocks * pb_len) != 0 ||
        x_pack == NULL)
    {
        free(out_buf);
        free(hist_ring);
        return RETURN_FAILURE;
    }
    memset(x_pack, 0, sizeof(float) * (size_t) max_blocks * pb_len);

    uint32_t head          = 0;
    uint64_t n_hist        = 0;
    uint32_t prev_effmodes = 0;
    float    prev_stat_val = -1.0f;
    o_active_frames        = 0;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        // 1. Insert latest PSOL modal vector into circular tap ring
        memcpy(&hist_ring[(size_t) head * NBmode], imgin.im->array.F, sizeof(float) * NBmode);
        n_hist++;

        uint32_t eff_modes = (p_nbmodemax < NBmode) ? p_nbmodemax : NBmode;
        int      pf_ready  = (imgPFmat.md[0].cnt0 > 0) && (n_hist >= (uint64_t) P_ring);

        if (pf_ready && eff_modes > 0)
        {
            const float *restrict pf_w = imgPFmat.im->array.F;
            uint32_t nblocks           = (eff_modes + B - 1) / B;
            int      nt                = (p_nthreads >= 1 && p_nthreads <= 16) ? (int) p_nthreads : 4;
            if ((uint32_t) nt > nblocks)
            {
                nt = (int) nblocks;
            }

            if (eff_modes < NBmode)
            {
                memset(&out_buf[eff_modes], 0, sizeof(float) * (NBmode - eff_modes));
            }

            size_t tail_off[EOFPF_MAX_ORDER];
            for (uint32_t t = 0; t < diag_tail; t++)
            {
                uint32_t slot = (head + P_ring - (P + t)) % P_ring;
                tail_off[t]   = (size_t) slot * NBmode;
            }

#pragma omp parallel for schedule(static) num_threads(nt) if (nt > 1)
            for (uint32_t blk = 0; blk < nblocks; blk++)
            {
                uint32_t s0   = blk * B;
                uint32_t b    = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
                float   *xblk = &x_pack[(size_t) blk * pb_len];

                for (uint32_t k = 0; k < P; k++)
                {
                    uint32_t     slot = (head + P_ring - k) % P_ring;
                    const float *src  = &hist_ring[(size_t) slot * NBmode + s0];
                    float       *dst  = &xblk[(size_t) k * B];
                    memcpy(dst, src, sizeof(float) * b);
                    if (b < B)
                    {
                        memset(&dst[b], 0, sizeof(float) * (B - b));
                    }
                }

                for (uint32_t r = 0; r < b; r++)
                {
                    uint32_t     m            = s0 + r;
                    const float *restrict row = &pf_w[(size_t) m * stride];
                    float acc                 = 0.0f;
#pragma omp simd reduction(+ : acc)
                    for (uint32_t d = 0; d < pb_len; d++)
                    {
                        acc += row[d] * xblk[d];
                    }
                    for (uint32_t t = 0; t < diag_tail; t++)
                    {
                        acc += row[pb_len + t] * hist_ring[tail_off[t] + m];
                    }
                    out_buf[m] = acc;
                }
            }

            o_active_frames++;
            uint32_t rsteps   = (p_rampsteps > 0) ? p_rampsteps : 1;
            float    stat_val = (o_active_frames >= (uint64_t) rsteps)
                                    ? 1.0f
                                    : ((float) o_active_frames / (float) rsteps);

            if (stat_val != prev_stat_val || eff_modes != prev_effmodes)
            {
                prev_stat_val          = stat_val;
                prev_effmodes          = eff_modes;
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

        head = (head + 1) % P_ring;
    }
    MILK_PROCINFO_END(self)

    free(x_pack);
    free(out_buf);
    free(hist_ring);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_applyEOFPF = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "applyEOFPF",
                 .cmdkey      = "applyEOFPF",
                 .description = "apply block cross-mode EOF predictive filter in real time",
                 .description_long =
                     "Apply learned block cross-mode Empirical Orthogonal Functions (VAR(P)) "
                     "transition matrix to live pseudo-open-loop (PSOL) modes and publish "
                     "predicted modal values." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaopc_applyEOFPF),
    .compute = applyEOFPF_compute,
};
EXPORT_FPSEXEC(cacao, "cacaopc-applyEOFPF", runnable_AOloopControl_PredictiveControl_applyEOFPF);
