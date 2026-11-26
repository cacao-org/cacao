// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    pyWFStlm.c
 * @brief   Live 2D stream & offline 3D cube Pyramid WFS telemetry engine (`cacao-fpsexec-pyrwfstlm`)
 *
 * Subscribes to a 2D dark-subtracted PyWFS camera stream (typically `aol<N>_imWFS0` before
 * masking or reference subtraction) or processes a 3D FITS/SHM telemetry cube in a single pass.
 *
 * Outputs (prefix `<outprefix>`, default `aol<N>_pyr`):
 *   <p>_tlm        1D [29] (live) or 2D [29 x Nframes] (cube) telemetry table
 *   <p>_pupgeom    2D [4 x 4] per-pupil geometry rows: [xc, yc, rout, corr]
 *   <p>_pupdof     1D [8] 6-DOF affine alignment decomposition + residual
 *   <p>_isum       2D [npix x npix] registered 4-pupil sum (if `.pub_maps=1`)
 *   <p>_sx, <p>_sy 2D [npix x npix] registered pyramid slopes (if `.pub_maps=1`)
 *   <p>_scint      2D [npix x npix] pupil-plane scintillation map (if `.pub_maps=1`)
 *   <p>_lvar       2D [npix x npix] local phase-variance / vignetting map (if `.pub_maps=1`)
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "fps_cli_sync.h"
#include "processinfo.h"

#include "pyramidWFStools.h"
#include "pyrwfs_tlm_core.h"

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint32_t p_loopindex                                 = 0;
static char     p_inwfs[FUNCTION_PARAMETER_STRMAXLEN]       = "";
static char     p_inref[FUNCTION_PARAMETER_STRMAXLEN]       = "";
static char     p_inzones[FUNCTION_PARAMETER_STRMAXLEN]     = "";
static char     p_innomgeom[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static char     p_ineven[FUNCTION_PARAMETER_STRMAXLEN]      = "";
static char     p_outprefix[FUNCTION_PARAMETER_STRMAXLEN]   = "";

static uint32_t p_stride     = 1;
static uint32_t p_reg_stride = 20;
static uint32_t p_npix       = 0;
static int32_t  p_sub_bg     = 1;
static int32_t  p_norm_mode  = 0;
static int32_t  p_edge_mode  = 1;
static int32_t  p_maxshift   = 6;
static int32_t  p_pub_maps   = 1;
static int32_t  p_reset      = 0;

static float p_sat_hi     = 60000.0f;
static float p_dark_floor = 0.0f;
static float p_flux_min   = 100.0f;
static float p_f_pup0     = 0.0f;
static float p_a_slow     = 0.02f;
static float p_a_fast     = 0.10f;
static float p_a_reg      = 0.10f;
static float p_corr_min   = 0.35f;
static float p_sp_ang     = 45.0f;
static float p_sp_hw      = 1.5f;

/* Structured FPS output telemetry scalars */
static uint64_t o_nframes       = 0;
static uint64_t o_ntorn         = 0;
static float    o_ftot          = 0.0f;
static float    o_bg_mean       = 0.0f;
static float    o_bg_std        = 0.0f;
static float    o_sat_hi_frac   = 0.0f;
static float    o_sat_dark_frac = 0.0f;
static int32_t  o_flux_ok       = 0;

static float o_f_pup    = 0.0f;
static float o_f_diam   = 0.0f;
static float o_f_bH     = 0.0f;
static float o_f_bV     = 0.0f;
static float o_f_out    = 0.0f;
static float o_df_pup   = 0.0f;
static float o_aniso    = 0.0f;
static float o_nl_ratio = 0.0f;

static float o_qx_mean = 0.0f;
static float o_qy_mean = 0.0f;
static float o_qx_rms  = 0.0f;
static float o_qy_rms  = 0.0f;
static float o_qdiag   = 0.0f;

static float o_scint_idx  = 0.0f;
static float o_curl_ratio = 0.0f;
static float o_lwe_metric = 0.0f;

static float o_pup_dx    = 0.0f;
static float o_pup_dy    = 0.0f;
static float o_pup_ds    = 0.0f;
static float o_pup_dast  = 0.0f;
static float o_pup_rot   = 0.0f;
static float o_pup_resid = 0.0f;
static float o_pup_corr  = 0.0f;

#define FPS_PARAMS_PYWFSTLM(X)                                                                   \
    X(".AOloopindex", &p_loopindex, FPTYPE_UINT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "AO loop index")                                  \
    X(".inwfs", p_inwfs, FPTYPE_STREAMNAME, 1,                                                   \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT,                                  \
      "input WFS 2D stream or 3D cube")                                                          \
    X(".inref", p_inref, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "optional flat-WF reference image")                                                        \
    X(".inzones", p_inzones, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "optional packed zone-label image")                                                        \
    X(".innomgeom", p_innomgeom, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "optional nominal [4x4] pupil geometry")                                                   \
    X(".ineven", p_ineven, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "optional even-mode Strehl basis")                                                         \
    X(".outprefix", p_outprefix, FPTYPE_STRING, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "output stream prefix (empty=aolN_pyr)")                                                   \
    X(".stride", &p_stride, FPTYPE_UINT32, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "process every Nth frame")      \
    X(".reg_stride", &p_reg_stride, FPTYPE_UINT32, 0,                                            \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                 \
      "pupil registration cadence [processed fr]")                                               \
    X(".npix", &p_npix, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,               \
      "registered pupil grid size (0=auto)")                                                     \
    X(".sub_bg", &p_sub_bg, FPTYPE_ONOFF, 0,                                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                 \
      "subtract corner background before budget")                                                \
    X(".norm_mode", &p_norm_mode, FPTYPE_INT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                 \
      "slope norm: 0=global mean, 1=local Isum")                                                 \
    X(".edge_mode", &p_edge_mode, FPTYPE_INT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "reg edge: 0=full Sobel, 1=taper inner rim")                                               \
    X(".maxshift", &p_maxshift, FPTYPE_INT32, 0,                                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                 \
      "pupil registration search half-width [pix]")                                              \
    X(".pub_maps", &p_pub_maps, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "publish 2D isum/sx/sy/scint/lvar maps")                                                   \
    X(".reset", &p_reset, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,               \
      "reset EMA accumulators")                                                                  \
    X(".sat_hi", &p_sat_hi, FPTYPE_FLOAT32, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "high saturation floor [ADU]")  \
    X(".dark_floor", &p_dark_floor, FPTYPE_FLOAT32, 0,                                           \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "dark/dropout floor [ADU]")     \
    X(".flux_min", &p_flux_min, FPTYPE_FLOAT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min valid total flux")         \
    X(".f_pup0", &p_f_pup0, FPTYPE_FLOAT32, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                 \
      "reference flat in-pupil fraction (0=auto)")                                               \
    X(".a_slow", &p_a_slow, FPTYPE_FLOAT32, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "slow EMA alpha")               \
    X(".a_fast", &p_a_fast, FPTYPE_FLOAT32, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "fast jitter EMA alpha")        \
    X(".a_reg", &p_a_reg, FPTYPE_FLOAT32, 0,                                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "registration frame EMA alpha") \
    X(".corr_min", &p_corr_min, FPTYPE_FLOAT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "min xcorr to track pupils")    \
    X(".sp_ang", &p_sp_ang, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "default spider angle [deg]")                                                              \
    X(".sp_hw", &p_sp_hw, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,            \
      "default spider band half-width [pix]")                                                    \
    X(".health.nframes", &o_nframes, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "processed frames")                                                                        \
    X(".health.ntorn", &o_ntorn, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "torn snapshot retries")                                                                   \
    X(".health.ftot", &o_ftot, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT, "total net flux")       \
    X(".health.bg_mean", &o_bg_mean, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "corner background mean")                                                                  \
    X(".health.bg_std", &o_bg_std, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "corner background std")                                                                   \
    X(".health.sat_hi_frac", &o_sat_hi_frac, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "saturated pupil pixel fraction")                                                          \
    X(".health.sat_dark_frac", &o_sat_dark_frac, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,       \
      "dark-floor pupil pixel fraction")                                                         \
    X(".health.flux_ok", &o_flux_ok, FPTYPE_INT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "1 when flux & saturation interlocks pass")                                                \
    X(".flux.pup", &o_f_pup, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "in-pupil flux fraction")                                                                  \
    X(".flux.diam", &o_f_diam, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "central apex diamond flux fraction")                                                      \
    X(".flux.bH", &o_f_bH, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                             \
      "horizontal inter-pupil bridge fraction")                                                  \
    X(".flux.bV", &o_f_bV, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                             \
      "vertical inter-pupil bridge fraction")                                                    \
    X(".flux.out", &o_f_out, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "outer halo + obscuration flux fraction")                                                  \
    X(".flux.df_pup", &o_df_pup, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "f_pup - f_pup0 (defocus-inv Strehl proxy)")                                               \
    X(".flux.aniso", &o_aniso, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "bridge anisotropy (bH-bV)/(bH+bV)")                                                       \
    X(".flux.nl_ratio", &o_nl_ratio, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "even-mode / total pupil signal ratio")                                                    \
    X(".jitter.qx_mean", &o_qx_mean, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "EMA quad-cell X centering")                                                               \
    X(".jitter.qy_mean", &o_qy_mean, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "EMA quad-cell Y centering")                                                               \
    X(".jitter.qx_rms", &o_qx_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "EMA quad-cell X jitter RMS")                                                              \
    X(".jitter.qy_rms", &o_qy_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                     \
      "EMA quad-cell Y jitter RMS")                                                              \
    X(".jitter.qdiag", &o_qdiag, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "quad-cell diagonal imbalance")                                                            \
    X(".wf.scint_idx", &o_scint_idx, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                   \
      "pupil-plane scintillation index")                                                         \
    X(".wf.curl_ratio", &o_curl_ratio, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "slope curl / divergence RMS ratio")                                                       \
    X(".wf.lwe_metric", &o_lwe_metric, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                 \
      "spider-flanking LWE / petaling RMS")                                                      \
    X(".align.dx", &o_pup_dx, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "common pupil X shift [pix]")                                                              \
    X(".align.dy", &o_pup_dy, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "common pupil Y shift [pix]")                                                              \
    X(".align.ds", &o_pup_ds, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "radial pupil separation shift [pix]")                                                     \
    X(".align.dast", &o_pup_dast, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "astigmatic separation (dsx-dsy) [pix]")                                                   \
    X(".align.rot", &o_pup_rot, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "pupil rotation [mrad]")                                                                   \
    X(".align.resid", &o_pup_resid, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "non-rigid 4-pupil fit residual [pix]")                                                    \
    X(".align.corr", &o_pup_corr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "min 4-pupil edge xcorr peak")

MILK_PARAMS_DEFINE(pyWFStlm, FPS_PARAMS_PYWFSTLM);

/* ================================================================
 * 2.  HELPERS: IMAGE TYPE CONVERSION & TORN-SAFE SNAPSHOT
 * ============================================================= */

static int pywt_slice_to_f32(const IMAGE *im, uint32_t slice, float *dst)
{
    size_t npix = (size_t) im->md[0].size[0] * im->md[0].size[1];
    size_t off  = (size_t) slice * npix;

    switch (im->md[0].datatype)
    {
    case _DATATYPE_FLOAT:
        memcpy(dst, im->array.F + off, npix * sizeof(float));
        return 0;
    case _DATATYPE_DOUBLE:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.D[off + i];
        }
        return 0;
    case _DATATYPE_UINT16:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.UI16[off + i];
        }
        return 0;
    case _DATATYPE_INT16:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.SI16[off + i];
        }
        return 0;
    case _DATATYPE_UINT32:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.UI32[off + i];
        }
        return 0;
    case _DATATYPE_INT32:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.SI32[off + i];
        }
        return 0;
    case _DATATYPE_UINT8:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (float) im->array.UI8[off + i];
        }
        return 0;
    default:
        return -1;
    }
}

static int pywt_snapshot_f32(const IMAGE *im, float *dst, uint64_t *ntorn)
{
    for (int attempt = 0; attempt < 4; attempt++)
    {
        uint64_t c0 = im->md[0].cnt0;
        uint8_t  w0 = im->md[0].write;
        if (pywt_slice_to_f32(im, 0u, dst) != 0)
        {
            return -1;
        }
        uint8_t  w1 = im->md[0].write;
        uint64_t c1 = im->md[0].cnt0;
        if (w0 == 0u && w1 == 0u && c0 == c1)
        {
            return 0;
        }
        if (ntorn != NULL)
        {
            (*ntorn)++;
        }
    }
    return 0;
}

static int pywt_read_zones_u8(const IMAGE *im, uint32_t nx, uint32_t ny, uint8_t *dst)
{
    if (im->md[0].size[0] != nx || im->md[0].size[1] != ny)
    {
        return -1;
    }
    size_t npix = (size_t) nx * ny;
    switch (im->md[0].datatype)
    {
    case _DATATYPE_UINT8:
        memcpy(dst, im->array.UI8, npix);
        return 0;
    case _DATATYPE_INT32:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (uint8_t) (im->array.SI32[i] & 0xFF);
        }
        return 0;
    case _DATATYPE_UINT16:
        for (size_t i = 0; i < npix; i++)
        {
            dst[i] = (uint8_t) (im->array.UI16[i] & 0xFFu);
        }
        return 0;
    case _DATATYPE_FLOAT:
        for (size_t i = 0; i < npix; i++)
        {
            long v = lroundf(im->array.F[i]);
            dst[i] = (uint8_t) ((v >= 0 && v <= 255) ? v : 0);
        }
        return 0;
    case _DATATYPE_DOUBLE:
        for (size_t i = 0; i < npix; i++)
        {
            long v = lround(im->array.D[i]);
            dst[i] = (uint8_t) ((v >= 0 && v <= 255) ? v : 0);
        }
        return 0;
    default:
        return -1;
    }
}

static int pywt_read_nomgeom(const IMAGE *im, pyrwfs_pupgeom_t *g)
{
    /* Expect [4 x 3] or [4 x 4] (row = pupil q=0..3, col = [xc, yc, rout, (rin)]) */
    if (im->md[0].naxis < 2 || im->md[0].size[0] < 3u || im->md[0].size[1] < 4u)
    {
        return -1;
    }
    uint32_t ncol = im->md[0].size[0];
    float   *tmp  = (float *) malloc((size_t) ncol * im->md[0].size[1] * sizeof(float));
    if (tmp == NULL)
    {
        return -1;
    }
    if (pywt_slice_to_f32(im, 0u, tmp) != 0)
    {
        free(tmp);
        return -1;
    }
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        const float *row = tmp + (size_t) q * ncol;
        g->xc[q]         = row[0];
        g->yc[q]         = row[1];
        g->rout[q]       = row[2];
        g->rin[q]        = (ncol >= 4u) ? row[3] : 0.0f;
        g->corr[q]       = 1.0f;
    }
    free(tmp);
    return 0;
}

static void pywt_write_pupgeom_stream(PROCESSINFO              *pinfo,
                                      IMAGE                    *in_im,
                                      IMGID                    *out_geom,
                                      IMGID                    *out_dof,
                                      const pyrwfs_tlm_state_t *st)
{
    if (out_geom->ID != -1)
    {
        out_geom->md->write = 1;
        for (int q = 0; q < PYRWFS_NPUP; q++)
        {
            float *row = out_geom->im->array.F + (size_t) q * 4u;
            row[0]     = st->live.xc[q];
            row[1]     = st->live.yc[q];
            row[2]     = st->live.rout[q];
            row[3]     = st->live.corr[q];
        }
        processinfo_update_output_stream(pinfo, out_geom->im, in_im);
    }
    if (out_dof->ID != -1)
    {
        out_dof->md->write      = 1;
        out_dof->im->array.F[0] = st->dof.dx;
        out_dof->im->array.F[1] = st->dof.dy;
        out_dof->im->array.F[2] = st->dof.dsx;
        out_dof->im->array.F[3] = st->dof.dsy;
        out_dof->im->array.F[4] = st->dof.rot;
        out_dof->im->array.F[5] = st->dof.scale;
        out_dof->im->array.F[6] = st->dof.skew;
        out_dof->im->array.F[7] = st->dof.resid;
        processinfo_update_output_stream(pinfo, out_dof->im, in_im);
    }
}

static void pywt_update_fps_outputs(const pyrwfs_tlm_state_t *st, const float tlm[PYRWFS_TLM_NCOL])
{
    o_nframes       = st->nframe;
    o_ftot          = tlm[PYRWFS_TLM_FTOT];
    o_bg_mean       = tlm[PYRWFS_TLM_BG_MEAN];
    o_bg_std        = tlm[PYRWFS_TLM_BG_STD];
    o_sat_hi_frac   = tlm[PYRWFS_TLM_SAT_HI_FRAC];
    o_sat_dark_frac = tlm[PYRWFS_TLM_SAT_DARK_FRAC];
    o_flux_ok       = (tlm[PYRWFS_TLM_FLUX_OK] > 0.5f) ? 1 : 0;

    o_f_pup    = tlm[PYRWFS_TLM_F_PUP];
    o_f_diam   = tlm[PYRWFS_TLM_F_DIAM];
    o_f_bH     = tlm[PYRWFS_TLM_F_BH];
    o_f_bV     = tlm[PYRWFS_TLM_F_BV];
    o_f_out    = tlm[PYRWFS_TLM_F_OUT];
    o_df_pup   = tlm[PYRWFS_TLM_DF_PUP];
    o_aniso    = tlm[PYRWFS_TLM_ANISO];
    o_nl_ratio = tlm[PYRWFS_TLM_NL_RATIO];

    o_qx_mean = tlm[PYRWFS_TLM_QX_MEAN];
    o_qy_mean = tlm[PYRWFS_TLM_QY_MEAN];
    o_qx_rms  = tlm[PYRWFS_TLM_QX_RMS];
    o_qy_rms  = tlm[PYRWFS_TLM_QY_RMS];
    o_qdiag   = tlm[PYRWFS_TLM_QDIAG];

    o_scint_idx  = tlm[PYRWFS_TLM_SCINT_IDX];
    o_curl_ratio = tlm[PYRWFS_TLM_CURL_RATIO];
    o_lwe_metric = tlm[PYRWFS_TLM_LWE_METRIC];

    o_pup_dx    = tlm[PYRWFS_TLM_PUP_DX];
    o_pup_dy    = tlm[PYRWFS_TLM_PUP_DY];
    o_pup_ds    = tlm[PYRWFS_TLM_PUP_DS];
    o_pup_dast  = tlm[PYRWFS_TLM_PUP_DAST];
    o_pup_rot   = tlm[PYRWFS_TLM_PUP_ROT];
    o_pup_resid = st->dof.resid;
    o_pup_corr  = tlm[PYRWFS_TLM_PUP_CORR];
}

static double pywt_frame_timestamp(const IMAGE *im)
{
    if (im->md[0].atime.tv_sec > 0)
    {
        return (double) im->md[0].atime.tv_sec + 1e-9 * (double) im->md[0].atime.tv_nsec;
    }
    if (im->md[0].writetime.tv_sec > 0)
    {
        return (double) im->md[0].writetime.tv_sec + 1e-9 * (double) im->md[0].writetime.tv_nsec;
    }
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double) ts.tv_sec + 1e-9 * (double) ts.tv_nsec;
}

/* ================================================================
 * 3.  COMPUTE ENTRY POINT (2D LIVE STREAM OR 3D OFFLINE CUBE)
 * ============================================================= */

static MILK_HOT errno_t pyWFStlm_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    IMGID img_wfs = imgid_make_from_name(p_inwfs);
    resolveIMGID(&img_wfs, ERRMODE_WARN, dcimg, dcnimg);
    if (img_wfs.ID == -1)
    {
        printf("[pyWFStlm] ERROR: cannot resolve input WFS stream '%s'\n", p_inwfs);
        return RETURN_FAILURE;
    }

    const uint32_t nx     = img_wfs.md->size[0];
    const uint32_t ny     = img_wfs.md->size[1];
    const int      iscube = (img_wfs.md->naxis >= 3 && img_wfs.md->size[2] > 1u);
    const uint32_t ncube  = iscube ? img_wfs.md->size[2] : 1u;
    const size_t   nimg   = (size_t) nx * ny;

    if (nx < 8u || ny < 8u)
    {
        printf("[pyWFStlm] ERROR: input image too small (%ux%u)\n", nx, ny);
        return RETURN_FAILURE;
    }

    char pfx[48];
    if (p_outprefix[0] != '\0')
    {
        snprintf(pfx, sizeof(pfx), "%.47s", p_outprefix);
    }
    else
    {
        snprintf(pfx, sizeof(pfx), "aol%u_pyr", p_loopindex);
    }

    /* Optional inputs: reference, zones, nominal geometry, even-mode basis */
    float   *buf_frame = (float *) malloc(nimg * sizeof(float));
    float   *buf_ref   = NULL;
    uint8_t *buf_zones = NULL;
    float   *buf_even  = NULL;
    if (buf_frame == NULL)
    {
        return RETURN_FAILURE;
    }

    if (p_inref[0] != '\0')
    {
        IMGID iref = imgid_make_from_name(p_inref);
        resolveIMGID(&iref, ERRMODE_WARN, dcimg, dcnimg);
        if (iref.ID != -1 && iref.md->size[0] == nx && iref.md->size[1] == ny)
        {
            buf_ref = (float *) malloc(nimg * sizeof(float));
            if (buf_ref != NULL && pywt_slice_to_f32(iref.im, 0u, buf_ref) != 0)
            {
                free(buf_ref);
                buf_ref = NULL;
            }
        }
    }

    if (p_inzones[0] != '\0')
    {
        IMGID iz = imgid_make_from_name(p_inzones);
        resolveIMGID(&iz, ERRMODE_WARN, dcimg, dcnimg);
        if (iz.ID != -1)
        {
            buf_zones = (uint8_t *) malloc(nimg);
            if (buf_zones != NULL && pywt_read_zones_u8(iz.im, nx, ny, buf_zones) != 0)
            {
                free(buf_zones);
                buf_zones = NULL;
            }
        }
    }

    if (p_ineven[0] != '\0')
    {
        IMGID iev = imgid_make_from_name(p_ineven);
        resolveIMGID(&iev, ERRMODE_WARN, dcimg, dcnimg);
        if (iev.ID != -1 && iev.md->size[0] == nx && iev.md->size[1] == ny)
        {
            buf_even = (float *) malloc(nimg * sizeof(float));
            if (buf_even != NULL && pywt_slice_to_f32(iev.im, 0u, buf_even) != 0)
            {
                free(buf_even);
                buf_even = NULL;
            }
        }
    }

    pyrwfs_pupgeom_t nom;
    memset(&nom, 0, sizeof(nom));
    int have_nom = 0;

    if (p_innomgeom[0] != '\0')
    {
        IMGID ig = imgid_make_from_name(p_innomgeom);
        resolveIMGID(&ig, ERRMODE_WARN, dcimg, dcnimg);
        if (ig.ID != -1 && pywt_read_nomgeom(ig.im, &nom) == 0)
        {
            have_nom = 1;
        }
    }
    if (!have_nom)
    {
        /* Auto-detect nominal 4 pupils from reference or first WFS frame */
        const float *det_src = buf_ref;
        if (det_src == NULL)
        {
            if (pywt_slice_to_f32(img_wfs.im, 0u, buf_frame) == 0)
            {
                det_src = buf_frame;
            }
        }
        if (det_src != NULL && pyrwfs_detect_pupils(det_src, nx, ny, 0.25f, &nom) == 0)
        {
            have_nom = 1;
        }
        else
        {
            /* Geometric fallback: 4 quadrants */
            float qx = 0.25f * (float) nx;
            float qy = 0.25f * (float) ny;
            float r0 = 0.20f * (float) ((nx < ny) ? nx : ny);
            nom.xc[0] = qx;            nom.yc[0] = qy;
            nom.xc[1] = qx;            nom.yc[1] = 3.0f * qy;
            nom.xc[2] = 3.0f * qx;     nom.yc[2] = qy;
            nom.xc[3] = 3.0f * qx;     nom.yc[3] = 3.0f * qy;
            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                nom.rout[q] = r0;
                nom.rin[q]  = 0.0f;
                nom.corr[q] = 1.0f;
            }
        }
    }

    if (buf_zones == NULL)
    {
        buf_zones = (uint8_t *) malloc(nimg);
        if (buf_zones == NULL)
        {
            free(buf_frame);
            free(buf_ref);
            free(buf_even);
            return RETURN_FAILURE;
        }
        pyrwfs_build_default_zones(nx, ny, &nom, p_sp_ang, p_sp_hw, buf_zones);
    }

    pyrwfs_tlm_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.nx         = nx;
    cfg.ny         = ny;
    cfg.npix       = p_npix;
    cfg.sub_bg     = p_sub_bg;
    cfg.norm_mode  = p_norm_mode;
    cfg.edge_mode  = p_edge_mode;
    cfg.maxshift   = p_maxshift;
    cfg.sat_hi     = (double) p_sat_hi;
    cfg.dark_floor = (double) p_dark_floor;
    cfg.flux_min   = (double) p_flux_min;
    cfg.f_pup0     = (double) p_f_pup0;
    cfg.a_slow     = (double) p_a_slow;
    cfg.a_fast     = (double) p_a_fast;
    cfg.a_reg      = (double) p_a_reg;
    cfg.corr_min   = (double) p_corr_min;

    pyrwfs_tlm_state_t st;
    if (pyrwfs_tlm_init(&st, &cfg, &nom, buf_zones, buf_ref, buf_even) != 0)
    {
        printf("[pyWFStlm] ERROR: failed to initialize telemetry core\n");
        pyrwfs_tlm_free(&st);
        free(buf_frame);
        free(buf_ref);
        free(buf_zones);
        free(buf_even);
        return RETURN_FAILURE;
    }

    /* Create output SHM streams */
    char  sname[STRINGMAXLEN_IMGNAME];
    IMGID out_tlm   = { .ID = -1 };
    IMGID out_geom  = { .ID = -1 };
    IMGID out_dof   = { .ID = -1 };
    IMGID out_isum  = { .ID = -1 };
    IMGID out_sx    = { .ID = -1 };
    IMGID out_sy    = { .ID = -1 };
    IMGID out_scint = { .ID = -1 };
    IMGID out_lvar  = { .ID = -1 };

    uint32_t tlm_rows = iscube ? ncube : 1u;
    snprintf(sname, sizeof(sname), "%s_tlm", pfx);
    out_tlm = stream_connect_create_2Df32(sname, PYRWFS_TLM_NCOL, tlm_rows);

    snprintf(sname, sizeof(sname), "%s_pupgeom", pfx);
    out_geom = stream_connect_create_2Df32(sname, 4u, PYRWFS_NPUP);

    snprintf(sname, sizeof(sname), "%s_pupdof", pfx);
    out_dof = stream_connect_create_2Df32(sname, PYRWFS_DOF_NCOL, 1u);

    const uint32_t npix = st.cfg.npix;
    if (p_pub_maps)
    {
        snprintf(sname, sizeof(sname), "%s_isum", pfx);
        out_isum = stream_connect_create_2Df32(sname, npix, npix);
        snprintf(sname, sizeof(sname), "%s_sx", pfx);
        out_sx = stream_connect_create_2Df32(sname, npix, npix);
        snprintf(sname, sizeof(sname), "%s_sy", pfx);
        out_sy = stream_connect_create_2Df32(sname, npix, npix);
        snprintf(sname, sizeof(sname), "%s_scint", pfx);
        out_scint = stream_connect_create_2Df32(sname, npix, npix);
        snprintf(sname, sizeof(sname), "%s_lvar", pfx);
        out_lvar = stream_connect_create_2Df32(sname, npix, npix);
    }

    pywt_write_pupgeom_stream(NULL, img_wfs.im, &out_geom, &out_dof, &st);

    o_nframes = 0;
    o_ntorn   = 0;

    uint64_t tick_cnt = 0;
    errno_t  ret      = RETURN_SUCCESS;

    MILK_PROCINFO_INIT(self)
    if (iscube && processinfo != NULL)
    {
        /* Static 3D cube: execute immediately without blocking on a stream semaphore */
        processinfo->triggermode = PROCESSINFO_TRIGGERMODE_IMMEDIATE;
    }
    MILK_PROCINFO_LOOPSTART(self)
    {
        /* Sync live-tunable FPS parameters */
        st.cfg.sub_bg     = p_sub_bg;
        st.cfg.norm_mode  = p_norm_mode;
        st.cfg.maxshift   = p_maxshift;
        st.cfg.sat_hi     = (double) p_sat_hi;
        st.cfg.dark_floor = (double) p_dark_floor;
        st.cfg.flux_min   = (double) p_flux_min;
        if (p_f_pup0 > 0.0f)
        {
            st.cfg.f_pup0 = (double) p_f_pup0;
        }
        st.cfg.a_slow   = (double) p_a_slow;
        st.cfg.a_fast   = (double) p_a_fast;
        st.cfg.a_reg    = (double) p_a_reg;
        st.cfg.corr_min = (double) p_corr_min;

        if (p_reset)
        {
            st.nframe = 0;
            p_reset   = 0;
        }

        if (iscube)
        {
            /* Offline 3D cube mode: process all slices in one pass and exit */
            float last_tlm[PYRWFS_TLM_NCOL];
            memset(last_tlm, 0, sizeof(last_tlm));

            if (out_tlm.ID != -1)
            {
                out_tlm.md->write = 1;
            }
            for (uint32_t k = 0; k < ncube; k++)
            {
                if (pywt_slice_to_f32(img_wfs.im, k, buf_frame) != 0)
                {
                    ret = RETURN_FAILURE;
                    break;
                }
                int do_reg = (p_reg_stride > 0u) && ((k % p_reg_stride) == 0u);
                float *row = (out_tlm.ID != -1)
                                 ? (out_tlm.im->array.F + (size_t) k * PYRWFS_TLM_NCOL)
                                 : last_tlm;
                pyrwfs_tlm_step(&st, buf_frame, (double) k, do_reg, NULL, row);
                memcpy(last_tlm, row, sizeof(last_tlm));
            }
            if (out_tlm.ID != -1)
            {
                processinfo_update_output_stream(processinfo, out_tlm.im, img_wfs.im);
            }
            pywt_write_pupgeom_stream(processinfo, img_wfs.im, &out_geom, &out_dof, &st);
            if (p_pub_maps)
            {
                size_t nb = (size_t) npix * npix * sizeof(float);
                if (out_isum.ID != -1)
                {
                    out_isum.md->write = 1;
                    memcpy(out_isum.im->array.F, st.isum_avg, nb);
                    processinfo_update_output_stream(processinfo, out_isum.im, img_wfs.im);
                }
                if (out_sx.ID != -1)
                {
                    out_sx.md->write = 1;
                    memcpy(out_sx.im->array.F, st.sx, nb);
                    processinfo_update_output_stream(processinfo, out_sx.im, img_wfs.im);
                }
                if (out_sy.ID != -1)
                {
                    out_sy.md->write = 1;
                    memcpy(out_sy.im->array.F, st.sy, nb);
                    processinfo_update_output_stream(processinfo, out_sy.im, img_wfs.im);
                }
                if (out_scint.ID != -1)
                {
                    out_scint.md->write = 1;
                    memcpy(out_scint.im->array.F, st.scint_map, nb);
                    processinfo_update_output_stream(processinfo, out_scint.im, img_wfs.im);
                }
                if (out_lvar.ID != -1)
                {
                    out_lvar.md->write = 1;
                    memcpy(out_lvar.im->array.F, st.lvar_map, nb);
                    processinfo_update_output_stream(processinfo, out_lvar.im, img_wfs.im);
                }
            }
            pywt_update_fps_outputs(&st, last_tlm);
            if (processinfo != NULL)
            {
                processinfo_WriteMessage_fmt(
                    processinfo, "cube N=%u f_pup=%.3f df=%.3f dx=%.2f dy=%.2f ds=%.2f",
                    ncube, o_f_pup, o_df_pup, o_pup_dx, o_pup_dy, o_pup_ds);
            }
            processloopOK = 0;
        }
        else
        {
            /* Live 2D stream mode */
            uint32_t stride = (p_stride > 0u) ? p_stride : 1u;
            if ((tick_cnt++ % stride) == 0u)
            {
                if (pywt_snapshot_f32(img_wfs.im, buf_frame, &o_ntorn) != 0)
                {
                    ret           = RETURN_FAILURE;
                    processloopOK = 0;
                }
                else
                {
                    double ts     = pywt_frame_timestamp(img_wfs.im);
                    int    do_reg = (p_reg_stride > 0u) && ((st.nframe % p_reg_stride) == 0u);
                    float  tlm[PYRWFS_TLM_NCOL];

                    pyrwfs_tlm_step(&st, buf_frame, ts, do_reg, NULL, tlm);

                    if (out_tlm.ID != -1)
                    {
                        out_tlm.md->write = 1;
                        memcpy(out_tlm.im->array.F, tlm, sizeof(tlm));
                        processinfo_update_output_stream(processinfo, out_tlm.im, img_wfs.im);
                    }
                    if (do_reg)
                    {
                        pywt_write_pupgeom_stream(processinfo, img_wfs.im, &out_geom, &out_dof, &st);
                    }
                    if (p_pub_maps)
                    {
                        size_t nb = (size_t) npix * npix * sizeof(float);
                        if (out_isum.ID != -1)
                        {
                            out_isum.md->write = 1;
                            memcpy(out_isum.im->array.F, st.isum, nb);
                            processinfo_update_output_stream(processinfo, out_isum.im, img_wfs.im);
                        }
                        if (out_sx.ID != -1)
                        {
                            out_sx.md->write = 1;
                            memcpy(out_sx.im->array.F, st.sx, nb);
                            processinfo_update_output_stream(processinfo, out_sx.im, img_wfs.im);
                        }
                        if (out_sy.ID != -1)
                        {
                            out_sy.md->write = 1;
                            memcpy(out_sy.im->array.F, st.sy, nb);
                            processinfo_update_output_stream(processinfo, out_sy.im, img_wfs.im);
                        }
                        if (out_scint.ID != -1 && (st.nframe % 10u) == 0u)
                        {
                            out_scint.md->write = 1;
                            memcpy(out_scint.im->array.F, st.scint_map, nb);
                            processinfo_update_output_stream(processinfo, out_scint.im, img_wfs.im);
                        }
                        if (out_lvar.ID != -1 && (st.nframe % 10u) == 0u)
                        {
                            out_lvar.md->write = 1;
                            memcpy(out_lvar.im->array.F, st.lvar_map, nb);
                            processinfo_update_output_stream(processinfo, out_lvar.im, img_wfs.im);
                        }
                    }

                    pywt_update_fps_outputs(&st, tlm);
                    if (processinfo != NULL && (st.nframe % 20u) == 1u)
                    {
                        processinfo_WriteMessage_fmt(
                            processinfo,
                            "fpup=%.3f df=%+.3f aniso=%+.2f dx=%+.2f dy=%+.2f ds=%+.2f ok=%d",
                            o_f_pup, o_df_pup, o_aniso, o_pup_dx, o_pup_dy, o_pup_ds, o_flux_ok);
                    }
                }
            }
        }
    }
    MILK_PROCINFO_END(self)

    if (dcfpsptr != NULL)
    {
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    pyrwfs_tlm_free(&st);
    free(buf_frame);
    free(buf_ref);
    free(buf_zones);
    free(buf_even);

    DEBUG_TRACE_FEXIT();
    return ret;
}

const MILK_RUNNABLE runnable_pyramidWFStools_pyWFStlm = {
    .kind = MILK_RUNNABLE_FPS,
    .app  = { .fps_name         = "pyWFStlm",
              .cmdkey           = "pyWFStlm",
              .description      = "Pyramid WFS live & cube telemetry, flux budget, and pupil tracker",
              .description_long = "Computes pyramid-specific telemetry from a 2D live WFS stream "
                                  "or 3D cube: packed-zone flux budget (in-pupil, apex diamond, "
                                  "H/V bridges, outer halo), defocus-invariant Strehl proxy, "
                                  "quad-cell centering & TT jitter RMS, edge-based 6-DOF 4-pupil "
                                  "alignment tracking, registered 4-pupil sum & slopes, "
                                  "scintillation index, slope curl/div ratio, and LWE metric." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(pyWFStlm),
    .compute = pyWFStlm_compute,
};
EXPORT_FPSEXEC(cacao, "pyrwfstlm", runnable_pyramidWFStools_pyWFStlm);
