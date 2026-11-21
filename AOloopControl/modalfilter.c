// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalfilter.c
 * @brief   Real-time modal filtering, pseudo-open-loop (PSOL) reconstruction, and predictive mixing
 */

// MILK_CMAKE_REQUEST_BLAS

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef HAVE_BLAS
#    include "milk_blas_lapacke.h"
#endif

#include "milk_runnable.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "timeutils.h"

#include "fps.h"
#include "fps_globals.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"
#include "processtools.h"

#include "AOloopControl.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#define MFILT_NBDMTSTEP    64
#define MFILT_PFCBUFF_SIZE 32

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     inmval[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalWFS";
static char     outmval[FUNCTION_PARAMETER_STRMAXLEN] = "aol0_modevalDM";
static float    loopgain                              = 0.01f;
static float    loopmult                              = 0.95f;
static float    looplimit                             = 1.0f;
static int32_t  loopZERO                              = 0;
static uint64_t AOloopindex                           = 0;
static int32_t  loopON                                = 0;
static int64_t  loopNBstep                            = -1;
static int32_t  compOL                                = 0;
static float    psol_WFSfact                          = 1.0f;
static float    latencyhardwfr                        = 1.7f;
static float    latencysoftwfr                        = 1.5f;
static int32_t  dynpsol_enable                        = 0;
static char     dynpsol_kernel[FUNCTION_PARAMETER_STRMAXLEN] = "";
static int32_t  dynpsol_reload                        = 0;
static uint32_t dynpsol_mode                          = 0;
static uint32_t dynpsol_ntaps                         = 0;
static uint32_t dynpsol_nmode                         = 0;
static int32_t  ogprobe_enable                        = 0;
static float    ogprobe_ampl                          = 0.002f;
static uint32_t ogprobe_nbmode                        = 64;
static uint32_t ogprobe_stride                        = 1;
static uint32_t ogprobe_hold                          = 3;
static uint64_t ogprobe_seed                          = 0x506F6B654F47ULL;
static int32_t  comptbuff                             = 0;
static int32_t  autolim                               = 0;
static float    autolimprobegain                      = 0.1f;
static float    autolimsigmafact                      = 2.0f;
static uint32_t tbuffsize                             = 512;
static int32_t  auxDMmvalenable                       = 0;
static float    auxDMmvalmixfact                      = 1.0f;
static int32_t  auxDMmvalmodulate                     = 0;
static float    auxDMmvalmodperiod                    = 20.0f;
static int32_t  enablePF                              = 0;
static uint32_t PF_NBblock                            = 0;
static uint32_t PF_maxwaitus                          = 500;
static float    PFmixcoeff                            = 1.0f;
static uint64_t PF_timeout_cnt                        = 0;
static int32_t  autoloopenable                        = 0;
static float    autoloopsleep                         = 0.001f;
static int32_t  selfRMenable                          = 0;
static uint32_t selfRMnbmode                          = 32;
static float    selfRMpokeampl                        = 0.01f;
static uint32_t selfRMzsize                           = 20;
static uint32_t selfRMnbiter                          = 8;
static uint32_t selfRMnbsettlestep                    = 1;
static int32_t  selfRMsaveFITS                        = 1;
static int32_t  testOL                                = 0;
static int32_t  testOLloop                            = 0;
static float    testOLupdategain                      = 0.5f;
static uint32_t testOLmode                            = 0;
static float    testOLampl                            = 0.01f;
static uint32_t testOLnbsample                        = 400;
static uint32_t testOLcnt                             = 0;
static int32_t  offload                               = 0;
static float    offloadloopgain                       = 0.01f;
static float    offloadloopmult                       = 0.95f;
static float    offloadlooplimit                      = 1.0f;
static int32_t  recburst                              = 0;
static uint32_t recburst_mode                         = 5;
static uint32_t recburst_nbsample                     = 1000;

static uint64_t processinfo_change_cnt_local = 0;

typedef struct
{
    float *mvalDMc;
    float *mvalout;
    float *mvaloutapply;

    // Pseudo-Open-Loop (PSOL) reconstruction
    float   *mvalDMbuff;
    float   *mvalDMOL;
    int      DMtstep;
    IMGID    imgOLmval;
    IMGID    imgmWFSfact;
    int      dyn_ready;
    uint32_t dyn_mode;   /* 0=legacy 2-tap, 1=diag FIR, 2=MIMO FIR */
    uint32_t dyn_ntaps;
    uint32_t dyn_nmode;
    float   *dyn_H_diag; /* [dyn_ntaps * dyn_nmode] */
    float   *dyn_H_mimo; /* [dyn_ntaps * dyn_nmode * dyn_nmode] */
    uint8_t  dyn_tap_nz[MFILT_NBDMTSTEP];
    char     dyn_loaded_path[FUNCTION_PARAMETER_STRMAXLEN];
    int      dyn_prev_enable;

    // Aux DM
    IMGID  imgauxmDM;
    double modpha;

    // Predictive Filter (PF)
    IMGID    imgmvalPF;
    IMGID    imgoutPFstat;
    IMGID    imgmPFmixfact;
    IMGID    imgmPFmix;
    IMGID    imgcbuff_mvalPF;
    int      semindex_PF;
    uint32_t PFcbuff_index;
    double  *mvalPFres;
    double  *mvalPFold;

    // Gain/Mult/Limit/Zero
    IMGID    imgmgain;
    IMGID    imgmgainfact;
    IMGID    imgmmult;
    IMGID    imgmmultfact;
    IMGID    imgmlimit;
    IMGID    imgmlimitfact;
    IMGID    imgmzeropoint;
    float    prev_loopgain;
    float    prev_loopmult;
    float    prev_looplimit;
    float    prev_pfmix;
    uint64_t prev_mgf_cnt0;
    uint64_t prev_mmf_cnt0;
    uint64_t prev_mlf_cnt0;
    uint64_t prev_mpf_cnt0;

    // Offload
    IMGID imgoffloadmgain;
    IMGID imgoffloadmgainfact;
    IMGID imgoffloadmmult;
    IMGID imgoffloadmmultfact;
    IMGID imgoffloadmlimit;
    IMGID imgoffloadmlimitfact;
    IMGID imgmvaloffloadDM;

    // Counters
    long *mlimitcntarray;
    long  modal_limit_counter;
    IMGID imgmlimitcntfrac;

    // Telemetry Buffers & Running Stats
    IMGID    imgtbuff_mvalDM;
    IMGID    imgtbuff_mvalWFS;
    IMGID    imgtbuff_mvalOL;
    IMGID    imgtbuff_mvalprobe;
    uint32_t tbuff_alloc_size;
    uint32_t tbuffindex;
    int      tbuffslice;
    uint64_t stats_accum_cnt;

    // Online optical-gain exogenous probe generator
    IMGID    imgmogprobew;
    float   *ogprobecmd;
    uint64_t ogprobe_rng;
    uint32_t ogprobe_hold_cnt;
    int      ogprobe_was_on;

    double *mvalDMave;
    double *mvalDMrms;
    double *mvalWFSave;
    double *mvalWFSrms;
    double *mvalOLave;
    double *mvalOLrms;
    double *mvalPFresave;
    double *mvalPFresrms;

    IMGID imgmvalDMave;
    IMGID imgmvalDMrms;
    IMGID imgmvalWFSave;
    IMGID imgmvalWFSrms;
    IMGID imgmvalOLave;
    IMGID imgmvalOLrms;
    IMGID imgmvalPFresave;
    IMGID imgmvalPFresrms;

    double *autolimDMsigma;

    // SelfRM
    IMGID    imgselfRM;
    float   *selfRMpokecmd;
    int      blockcnt;
    int      selfRMpokeparity;
    uint32_t selfRMiter;
    uint32_t selfRM_pokemode;
    uint32_t selfRM_pokecnt;
    float    selfRMpokesign;

    // TestOL (in-loop latency & optical-gain identification)
    float   *psOL_probe;
    float   *psOL_wfs;
    float   *psOL_dm;
    uint32_t psOL_cap;
    float    testOL_pokecmd;
    uint32_t testOL_active_mode;

    // Burst telemetry log
    uint64_t recburstsample;
    FILE    *fprec;

    // External DM modeval update detection
    IMGID    imgmodevalDMf;
    uint64_t imgmodevalDMfcnt0old;

} MFILT_STATE;

/* ============================================================================================== */
/* COMPUTE LOGIC                                                                                  */
/* ============================================================================================== */

static void init_ones_if_all_zero(IMGID img, uint32_t n)
{
    int any_nonzero = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        if (fabsf(img.im->array.F[i]) > 1.0e-20f)
        {
            any_nonzero = 1;
            break;
        }
    }
    if (!any_nonzero)
    {
        for (uint32_t i = 0; i < n; i++)
        {
            img.im->array.F[i] = 1.0f;
        }
        ImageStreamIO_UpdateIm(img.im);
    }
}

static MFILT_STATE *modal_filter_init(uint32_t NBmode)
{
    MFILT_STATE *state = (MFILT_STATE *) calloc(1, sizeof(MFILT_STATE));

    state->mvalDMc      = (float *) calloc(NBmode, sizeof(float));
    state->mvalout      = (float *) calloc(NBmode, sizeof(float));
    state->mvaloutapply = (float *) calloc(NBmode, sizeof(float));

    // Pseudo-Open-Loop (PSOL)
    state->mvalDMbuff = (float *) calloc((size_t) NBmode * MFILT_NBDMTSTEP, sizeof(float));
    state->mvalDMOL   = (float *) calloc(NBmode, sizeof(float));

    char name[STRINGMAXLEN_STREAMNAME];

    WRITE_IMAGENAME(name, "aol%lu_modevalOL", AOloopindex);
    state->imgOLmval = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mWFSfact", AOloopindex);
    state->imgmWFSfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmWFSfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_modevalauxDM", AOloopindex);
    state->imgauxmDM = stream_connect_create_2Df32(name, NBmode, 1);

    // Predictive Filter (PF)
    WRITE_IMAGENAME(name, "aol%lu_modevalPF", AOloopindex);
    state->imgmvalPF   = stream_connect_create_2Df32(name, NBmode, 1);
    state->semindex_PF = ImageStreamIO_getsemwaitindex(state->imgmvalPF.im, -1);

    WRITE_IMAGENAME(name, "aol%lu_outPFstat", AOloopindex);
    state->imgoutPFstat = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mPFmixfact", AOloopindex);
    state->imgmPFmixfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmPFmixfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_mPFmix", AOloopindex);
    state->imgmPFmix = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_modevalPF_cbuff", AOloopindex);
    state->imgcbuff_mvalPF = stream_connect_create_2Df32(name, MFILT_PFCBUFF_SIZE, NBmode);

    state->mvalPFres = (double *) calloc(NBmode, sizeof(double));
    state->mvalPFold = (double *) calloc(NBmode, sizeof(double));

    // Gain / Mult / Limit / Zeropoint
    WRITE_IMAGENAME(name, "aol%lu_mgain", AOloopindex);
    state->imgmgain = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mgainfact", AOloopindex);
    state->imgmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmgainfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_mmult", AOloopindex);
    state->imgmmult = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mmultfact", AOloopindex);
    state->imgmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmmultfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_mzeropoint", AOloopindex);
    state->imgmzeropoint = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mlimit", AOloopindex);
    state->imgmlimit = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mlimitfact", AOloopindex);
    state->imgmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmlimitfact, NBmode);

    state->prev_loopgain  = -1.0f;
    state->prev_loopmult  = -1.0f;
    state->prev_looplimit = -1.0f;
    state->prev_pfmix     = -1.0f;

    // Offload
    WRITE_IMAGENAME(name, "aol%lu_offloadmgain", AOloopindex);
    state->imgoffloadmgain = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_offloadmgainfact", AOloopindex);
    state->imgoffloadmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgoffloadmgainfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_offloadmmult", AOloopindex);
    state->imgoffloadmmult = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_offloadmmultfact", AOloopindex);
    state->imgoffloadmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgoffloadmmultfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_offloadmlimit", AOloopindex);
    state->imgoffloadmlimit = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_offloadmlimitfact", AOloopindex);
    state->imgoffloadmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgoffloadmlimitfact, NBmode);

    WRITE_IMAGENAME(name, "aol%lu_mvaloffloadDM", AOloopindex);
    state->imgmvaloffloadDM = stream_connect_create_2Df32(name, NBmode, 1);

    // Limit Counter
    state->mlimitcntarray = (long *) calloc(NBmode, sizeof(long));
    WRITE_IMAGENAME(name, "aol%lu_mlimitcntfrac", AOloopindex);
    state->imgmlimitcntfrac = stream_connect_create_2Df32(name, NBmode, 1);

    // Running Stats
    state->mvalDMave      = (double *) calloc(NBmode, sizeof(double));
    state->mvalDMrms      = (double *) calloc(NBmode, sizeof(double));
    state->mvalWFSave     = (double *) calloc(NBmode, sizeof(double));
    state->mvalWFSrms     = (double *) calloc(NBmode, sizeof(double));
    state->mvalOLave      = (double *) calloc(NBmode, sizeof(double));
    state->mvalOLrms      = (double *) calloc(NBmode, sizeof(double));
    state->mvalPFresave   = (double *) calloc(NBmode, sizeof(double));
    state->mvalPFresrms   = (double *) calloc(NBmode, sizeof(double));
    state->autolimDMsigma = (double *) calloc(NBmode, sizeof(double));

    WRITE_IMAGENAME(name, "aol%lu_mvalDMave", AOloopindex);
    state->imgmvalDMave = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalDMrms", AOloopindex);
    state->imgmvalDMrms = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalWFSave", AOloopindex);
    state->imgmvalWFSave = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalWFSrms", AOloopindex);
    state->imgmvalWFSrms = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalOLave", AOloopindex);
    state->imgmvalOLave = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalOLrms", AOloopindex);
    state->imgmvalOLrms = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalPFresave", AOloopindex);
    state->imgmvalPFresave = stream_connect_create_2Df32(name, NBmode, 1);
    WRITE_IMAGENAME(name, "aol%lu_mvalPFresrms", AOloopindex);
    state->imgmvalPFresrms = stream_connect_create_2Df32(name, NBmode, 1);

    // Telemetry Double Buffers
    uint32_t tbsize         = (tbuffsize > 0) ? tbuffsize : 512;
    state->tbuff_alloc_size = tbsize;
    WRITE_IMAGENAME(name, "aol%lu_modevalDM_buff", AOloopindex);
    state->imgtbuff_mvalDM = stream_connect_create_3Df32(name, NBmode, tbsize, 2);
    WRITE_IMAGENAME(name, "aol%lu_modevalWFS_buff", AOloopindex);
    state->imgtbuff_mvalWFS = stream_connect_create_3Df32(name, NBmode, tbsize, 2);
    WRITE_IMAGENAME(name, "aol%lu_modevalOL_buff", AOloopindex);
    state->imgtbuff_mvalOL = stream_connect_create_3Df32(name, NBmode, tbsize, 2);
    WRITE_IMAGENAME(name, "aol%lu_modevalprobe_buff", AOloopindex);
    state->imgtbuff_mvalprobe = stream_connect_create_3Df32(name, NBmode, tbsize, 2);

    // Online optical-gain probe weights & state
    WRITE_IMAGENAME(name, "aol%lu_mogprobe_w", AOloopindex);
    state->imgmogprobew = stream_connect_create_2Df32(name, NBmode, 1);
    init_ones_if_all_zero(state->imgmogprobew, NBmode);
    state->ogprobecmd  = (float *) calloc(NBmode, sizeof(float));
    state->ogprobe_rng = (ogprobe_seed != 0ULL) ? ogprobe_seed : 0x506F6B654F47ULL;

    // SelfRM
    uint32_t srmzsize = (selfRMzsize > 0) ? selfRMzsize : 20;
    WRITE_IMAGENAME(name, "aol%lu_mfiltselfRM", AOloopindex);
    state->imgselfRM      = stream_connect_create_3Df32(name, NBmode, NBmode, srmzsize);
    state->selfRMpokecmd  = (float *) calloc(NBmode, sizeof(float));
    state->selfRMpokesign = 1.0f;

    // External DMf
    WRITE_IMAGENAME(name, "aol%lu_modevalDMf", AOloopindex);
    state->imgmodevalDMf      = stream_connect_create_2Df32(name, NBmode, 1);
    state->imgmodevalDMfcnt0old = state->imgmodevalDMf.md[0].cnt0;

    return state;
}

static void modal_filter_cleanup(MFILT_STATE *state)
{
    if (!state)
    {
        return;
    }
    if (state->fprec)
    {
        fclose(state->fprec);
        state->fprec = NULL;
    }
    free(state->mvalDMc);
    free(state->mvalout);
    free(state->mvaloutapply);
    free(state->mvalDMbuff);
    free(state->mvalDMOL);
    free(state->dyn_H_diag);
    free(state->dyn_H_mimo);
    free(state->mvalPFres);
    free(state->mvalPFold);
    free(state->mlimitcntarray);
    free(state->mvalDMave);
    free(state->mvalDMrms);
    free(state->mvalWFSave);
    free(state->mvalWFSrms);
    free(state->mvalOLave);
    free(state->mvalOLrms);
    free(state->mvalPFresave);
    free(state->mvalPFresrms);
    free(state->autolimDMsigma);
    free(state->ogprobecmd);
    free(state->selfRMpokecmd);
    free(state->psOL_probe);
    free(state->psOL_wfs);
    free(state->psOL_dm);
    free(state);
}

static void solve_and_apply_testOL(PROCESSINFO *processinfo, FPS *fps, MFILT_STATE *state,
                                   uint32_t nbspl)
{
    float lat_curr = latencyhardwfr + latencysoftwfr;
    if (lat_curr < 1.0f)
    {
        lat_curr = 2.0f;
    }
    float lat_min  = (lat_curr * 0.4f > 1.0f) ? (lat_curr * 0.4f) : 1.0f;
    float lat_max  = (lat_curr * 1.8f < 12.0f) ? (lat_curr * 1.8f) : 12.0f;
    float wfs_base = (psol_WFSfact > 0.1f) ? psol_WFSfact : 1.0f;
    float wfs_min  = 0.65f * wfs_base;
    float wfs_max  = 1.45f * wfs_base;

    double best_err = 1.0e38;
    float  opt_lat  = lat_curr;
    float  opt_wf   = wfs_base;

    for (float lat_cand = lat_min; lat_cand <= lat_max; lat_cand += 0.02f)
    {
        float lag_act  = lat_cand;
        int   l_int    = (int) lag_act;
        float l_frac   = lag_act - (float) l_int;
        int   t_start  = l_int + 3;
        int   t_end    = (int) nbspl - (l_int + 3);
        if (t_end <= t_start + 16)
        {
            continue;
        }
        for (float wf_cand = wfs_min; wf_cand <= wfs_max; wf_cand += 0.01f)
        {
            double err = 0.0;
            for (int ii = t_start; ii < t_end; ii++)
            {
                float tf   = (float) ii + lag_act;
                int   jj0  = (int) tf;
                float jfr  = tf - (float) jj0;
                int   jj1  = jj0 + 1;
                float ol0  = wf_cand * state->psOL_wfs[jj0] -
                             ((1.0f - l_frac) * state->psOL_dm[jj0 - l_int] +
                              l_frac * state->psOL_dm[jj0 - l_int - 1]);
                float ol1  = wf_cand * state->psOL_wfs[jj1] -
                             ((1.0f - l_frac) * state->psOL_dm[jj1 - l_int] +
                              l_frac * state->psOL_dm[jj1 - l_int - 1]);
                double olv = (1.0 - (double) jfr) * (double) ol0 + (double) jfr * (double) ol1;
                double res = olv - (double) state->psOL_probe[ii];
                err += res * res;
            }
            if (err < best_err)
            {
                best_err = err;
                opt_lat  = lat_cand;
                opt_wf   = wf_cand;
            }
        }
    }

    float g1 =
        (testOLupdategain >= 0.0f && testOLupdategain <= 1.0f) ? testOLupdategain : 0.5f;
    float g0       = 1.0f - g1;
    latencysoftwfr = g0 * latencysoftwfr + g1 * (opt_lat - latencyhardwfr);
    psol_WFSfact   = g0 * psol_WFSfact + g1 * opt_wf;

    if (fps != NULL && fps->md != NULL)
    {
        char ffname[STRINGMAXLEN_FULLFILENAME];
        WRITE_FULLFILENAME(ffname, "%s/cacaovars.bash", fps->md->datadir);
        FILE *fpout = fopen(ffname, "w");
        if (fpout != NULL)
        {
            fprintf(fpout, "export CACAO_PSOL_WFSFACT=%.4f\n", psol_WFSfact);
            fprintf(fpout, "export CACAO_LATENCYSOFTWFR=%.4f\n", latencysoftwfr);
            fprintf(fpout, "export CACAO_LATENCYFR=%.4f\n", latencyhardwfr + latencysoftwfr);
            fclose(fpout);
        }
    }
    processinfo_WriteMessage_fmt(processinfo, "testOL lag=%.3ffr WFSf=%.3f",
                                 latencyhardwfr + latencysoftwfr, psol_WFSfact);
}

/**
 * @brief Load 2D H_diag [N_in, L] or 3D H_mimo [N_in, N_in, L] FITS kernel for Dynamic PSOL.
 */
static int mfilt_load_dyn_kernel(PROCESSINFO *processinfo,
                                 MFILT_STATE *state,
                                 uint32_t     NBmode,
                                 const char  *raw_path)
{
    free(state->dyn_H_diag);
    free(state->dyn_H_mimo);
    state->dyn_H_diag = NULL;
    state->dyn_H_mimo = NULL;
    state->dyn_ready  = 0;
    state->dyn_mode   = 0;
    state->dyn_ntaps  = 0;
    state->dyn_nmode  = 0;
    memset(state->dyn_tap_nz, 0, sizeof(state->dyn_tap_nz));
    snprintf(state->dyn_loaded_path, sizeof(state->dyn_loaded_path), "%s",
             (raw_path != NULL) ? raw_path : "");

    if (raw_path == NULL || raw_path[0] == '\0' || strcmp(raw_path, "none") == 0)
    {
        return -1;
    }

#ifdef USE_CFITSIO
    const char *path   = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    fitsfile   *fptr   = NULL;
    int         status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        processinfo_WriteMessage_fmt(processinfo, "dynpsol: cannot open %s", path);
        return -1;
    }
    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || (naxis != 2 && naxis != 3))
    {
        fits_close_file(fptr, &status);
        processinfo_WriteMessage_fmt(processinfo, "dynpsol: bad naxis=%d", naxis);
        return -1;
    }
    fits_get_img_size(fptr, naxis, naxes, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    LONGLONG nelem = (LONGLONG) naxes[0] * (LONGLONG) naxes[1] *
                     ((naxis == 3) ? (LONGLONG) naxes[2] : 1LL);
    float *raw = (float *) malloc(sizeof(float) * (size_t) nelem);
    if (raw == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, nelem, NULL, raw, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(raw);
        return -1;
    }

    if (naxis == 2)
    {
        uint32_t f_nin = (uint32_t) naxes[0];
        uint32_t f_L   = (uint32_t) naxes[1];
        if (f_nin == 0 || f_L < 2 || f_L >= MFILT_NBDMTSTEP)
        {
            free(raw);
            processinfo_WriteMessage_fmt(processinfo, "dynpsol: bad 2D dims %ux%u", f_nin, f_L);
            return -1;
        }
        uint32_t n_dyn = (f_nin == 1) ? NBmode : ((f_nin < NBmode) ? f_nin : NBmode);
        float   *hd    = (float *) calloc((size_t) f_L * n_dyn, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            float maxv = 0.0f;
            for (uint32_t m = 0; m < n_dyn; m++)
            {
                float v                  = (f_nin == 1) ? raw[l] : raw[(size_t) l * f_nin + m];
                hd[(size_t) l * n_dyn + m] = v;
                float av                 = fabsf(v);
                if (av > maxv)
                {
                    maxv = av;
                }
            }
            state->dyn_tap_nz[l] = (maxv > 1.0e-7f) ? 1u : 0u;
        }
        free(raw);
        state->dyn_H_diag = hd;
        state->dyn_mode   = 1;
        state->dyn_ntaps  = f_L;
        state->dyn_nmode  = n_dyn;
        state->dyn_ready  = 1;
        processinfo_WriteMessage_fmt(processinfo, "dynpsol: DIAG N=%u L=%u", n_dyn, f_L);
        return 0;
    }

    // 3D MIMO kernel [N_in, N_in, L]
    uint32_t f_nin = (uint32_t) naxes[0];
    uint32_t f_ns  = (uint32_t) naxes[1];
    uint32_t f_L   = (uint32_t) naxes[2];
    if (f_nin == 0 || f_nin != f_ns || f_L < 2 || f_L >= MFILT_NBDMTSTEP)
    {
        free(raw);
        processinfo_WriteMessage_fmt(processinfo, "dynpsol: bad 3D %ux%ux%u", f_nin, f_ns, f_L);
        return -1;
    }
    if (f_nin == 1)
    {
        float *hd = (float *) calloc((size_t) f_L * NBmode, sizeof(float));
        if (hd == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < f_L; l++)
        {
            float v = raw[l];
            for (uint32_t m = 0; m < NBmode; m++)
            {
                hd[(size_t) l * NBmode + m] = v;
            }
            state->dyn_tap_nz[l] = (fabsf(v) > 1.0e-7f) ? 1u : 0u;
        }
        free(raw);
        state->dyn_H_diag = hd;
        state->dyn_mode   = 1;
        state->dyn_ntaps  = f_L;
        state->dyn_nmode  = NBmode;
        state->dyn_ready  = 1;
        processinfo_WriteMessage_fmt(processinfo, "dynpsol: DIAG(bcast) N=%u L=%u", NBmode, f_L);
        return 0;
    }

    uint32_t n_dyn = (f_nin < NBmode) ? f_nin : NBmode;
    float   *hm    = (float *) calloc((size_t) f_L * n_dyn * n_dyn, sizeof(float));
    if (hm == NULL)
    {
        free(raw);
        return -1;
    }
    for (uint32_t l = 0; l < f_L; l++)
    {
        float maxv = 0.0f;
        for (uint32_t s = 0; s < n_dyn; s++)
        {
            const float *src = &raw[((size_t) l * f_nin + s) * f_nin];
            float       *dst = &hm[((size_t) l * n_dyn + s) * n_dyn];
            for (uint32_t q = 0; q < n_dyn; q++)
            {
                float v  = src[q];
                dst[q]   = v;
                float av = fabsf(v);
                if (av > maxv)
                {
                    maxv = av;
                }
            }
        }
        state->dyn_tap_nz[l] = (maxv > 1.0e-7f) ? 1u : 0u;
    }
    free(raw);
    state->dyn_H_mimo = hm;
    state->dyn_mode   = 2;
    state->dyn_ntaps  = f_L;
    state->dyn_nmode  = n_dyn;
    state->dyn_ready  = 1;
    processinfo_WriteMessage_fmt(processinfo, "dynpsol: MIMO N=%u L=%u", n_dyn, f_L);
    return 0;
#else
    (void) processinfo;
    (void) NBmode;
    return -1;
#endif
}

/**
 * @brief Real-time modal filtering, PSOL reconstruction, and predictive control step.
 */
static MILK_HOT void modal_filter_step(PROCESSINFO *processinfo,
                                       FPS         *fps,
                                       IMAGE       *imginWFS,
                                       IMAGE       *imgout,
                                       MFILT_STATE *state)
{
    if (fps)
    {
        if (fps->md->processinfo_change_cnt != processinfo_change_cnt_local)
        {
            fps_to_processinfo(fps, processinfo);
            processinfo_change_cnt_local = fps->md->processinfo_change_cnt;
        }
    }

    if (!imginWFS || !imgout || !state)
    {
        return;
    }

    uint32_t NBmode = imginWFS->md[0].size[0];

    // Zero loop
    if (loopZERO)
    {
        memset(state->mvalDMc, 0, sizeof(float) * NBmode);
        memset(state->mvalout, 0, sizeof(float) * NBmode);
        memset(state->mvaloutapply, 0, sizeof(float) * NBmode);
        memset(state->mvalDMbuff, 0, sizeof(float) * NBmode * MFILT_NBDMTSTEP);
        memset(state->mvalDMOL, 0, sizeof(float) * NBmode);
        memset(state->ogprobecmd, 0, sizeof(float) * NBmode);
        state->ogprobe_hold_cnt = 0;
        imgout->md[0].write = 1;
        memset(imgout->array.F, 0, sizeof(float) * NBmode);
        processinfo_update_output_stream(processinfo, imgout, NULL);
        loopZERO = 0;
    }

    if (loopON == 1)
    {
        if (loopNBstep > 0)
        {
            loopNBstep = loopNBstep - 1;
        }
        if (loopNBstep == 0)
        {
            loopON     = 0;
            loopNBstep = 1;
        }

        // Aux DM modulation factor
        float auxDMfact = auxDMmvalmixfact;
        if (auxDMmvalmodulate == 1 && auxDMmvalmodperiod > 0.1f)
        {
            state->modpha += 1.0 / (double) auxDMmvalmodperiod;
            if (state->modpha > 1.0)
            {
                state->modpha -= 1.0;
            }
            auxDMfact *= sinf(2.0f * (float) M_PI * (float) state->modpha);
        }

        // External DM modal state update
        uint64_t imgmodevalDMfcnt0 = state->imgmodevalDMf.md[0].cnt0;
        if (imgmodevalDMfcnt0 != state->imgmodevalDMfcnt0old)
        {
            state->imgmodevalDMfcnt0old = imgmodevalDMfcnt0;
            memcpy(state->mvalDMc, state->imgmodevalDMf.im->array.F, sizeof(float) * NBmode);
        }

        // Refresh Gain / Mult / Limit streams when scalar or per-mode factors change
        int upd_gml = 0;
        if (loopgain != state->prev_loopgain || loopmult != state->prev_loopmult ||
            looplimit != state->prev_looplimit ||
            state->imgmgainfact.md[0].cnt0 != state->prev_mgf_cnt0 ||
            state->imgmmultfact.md[0].cnt0 != state->prev_mmf_cnt0 ||
            state->imgmlimitfact.md[0].cnt0 != state->prev_mlf_cnt0)
        {
            upd_gml               = 1;
            state->prev_loopgain  = loopgain;
            state->prev_loopmult  = loopmult;
            state->prev_looplimit = looplimit;
            state->prev_mgf_cnt0  = state->imgmgainfact.md[0].cnt0;
            state->prev_mmf_cnt0  = state->imgmmultfact.md[0].cnt0;
            state->prev_mlf_cnt0  = state->imgmlimitfact.md[0].cnt0;
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                state->imgmgain.im->array.F[mi]  = state->imgmgainfact.im->array.F[mi] * loopgain;
                state->imgmmult.im->array.F[mi]  = state->imgmmultfact.im->array.F[mi] * loopmult;
                state->imgmlimit.im->array.F[mi] = state->imgmlimitfact.im->array.F[mi] * looplimit;
            }
            ImageStreamIO_UpdateIm(state->imgmgain.im);
            ImageStreamIO_UpdateIm(state->imgmmult.im);
            ImageStreamIO_UpdateIm(state->imgmlimit.im);
        }
        (void) upd_gml;

        // Prepare optional testOL chirp probe (excluded from PSOL DM history)
        state->testOL_pokecmd     = 0.0f;
        state->testOL_active_mode = (testOLmode < NBmode) ? testOLmode : 0;
        if (testOL == 1 && compOL == 1)
        {
            uint32_t nbspl = (testOLnbsample >= 64) ? testOLnbsample : 400;
            if (testOLcnt == 0 && state->psOL_cap < nbspl)
            {
                free(state->psOL_probe);
                free(state->psOL_wfs);
                free(state->psOL_dm);
                state->psOL_probe = (float *) calloc(nbspl, sizeof(float));
                state->psOL_wfs   = (float *) calloc(nbspl, sizeof(float));
                state->psOL_dm    = (float *) calloc(nbspl, sizeof(float));
                state->psOL_cap   = nbspl;
            }
            float x               = (float) testOLcnt / (float) nbspl;
            float freqfact        = 0.02f + 1.98f * x;
            state->testOL_pokecmd = testOLampl * sinf((float) testOLcnt * 0.16f * freqfact);
        }

        // Optional low-RMS orthogonal exogenous probe for online optical-gain ID
        if (ogprobe_enable == 1)
        {
            uint32_t hold = (ogprobe_hold >= 1) ? ogprobe_hold : 1;
            if (state->ogprobe_hold_cnt == 0 || !state->ogprobe_was_on)
            {
                uint32_t eff_nb =
                    (ogprobe_nbmode > 0 && ogprobe_nbmode < NBmode) ? ogprobe_nbmode : NBmode;
                uint32_t st = (ogprobe_stride >= 1) ? ogprobe_stride : 1;
                const float *restrict pw = state->imgmogprobew.im->array.F;
                memset(state->ogprobecmd, 0, sizeof(float) * NBmode);
                uint64_t rng = state->ogprobe_rng;
                for (uint32_t mi = 0; mi < eff_nb; mi += st)
                {
                    float w = pw[mi];
                    if (w == 0.0f)
                    {
                        continue;
                    }
                    rng ^= rng >> 12;
                    rng ^= rng << 25;
                    rng ^= rng >> 27;
                    float sgn = ((rng * 0x2545F4914F6CDD1DULL) >> 63) ? 1.0f : -1.0f;
                    state->ogprobecmd[mi] = sgn * ogprobe_ampl * w;
                }
                state->ogprobe_rng      = rng;
                state->ogprobe_hold_cnt = hold;
            }
            state->ogprobe_hold_cnt--;
            state->ogprobe_was_on = 1;
        }
        else if (state->ogprobe_was_on)
        {
            memset(state->ogprobecmd, 0, sizeof(float) * NBmode);
            state->ogprobe_hold_cnt = 0;
            state->ogprobe_was_on   = 0;
        }

        // Standard Modal Leaky-Integrator Step
        const float *restrict wfs_in = imginWFS->array.F;
        const float *restrict zpo    = state->imgmzeropoint.im->array.F;
        const float *restrict mg     = state->imgmgain.im->array.F;
        const float *restrict mm     = state->imgmmult.im->array.F;
        const float *restrict ml     = state->imgmlimit.im->array.F;
        const float *restrict aux    = state->imgauxmDM.im->array.F;

        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            float dmval = (zpo[mi] - wfs_in[mi]) * mg[mi];
            float dmc   = dmval + state->mvalDMc[mi] * mm[mi];
            float limit = ml[mi];
            if (dmc > limit)
            {
                dmc = limit;
                state->mlimitcntarray[mi]++;
            }
            else if (dmc < -limit)
            {
                dmc = -limit;
                state->mlimitcntarray[mi]++;
            }
            state->mvalDMc[mi] = dmc;

            if (auxDMmvalenable == 1)
            {
                state->mvalout[mi] = dmc + auxDMfact * aux[mi];
            }
            else
            {
                state->mvalout[mi] = dmc;
            }
            state->mvaloutapply[mi] =
                state->mvalout[mi] + state->selfRMpokecmd[mi] + state->ogprobecmd[mi];
        }
        if (testOL == 1 && compOL == 1)
        {
            state->mvaloutapply[state->testOL_active_mode] += state->testOL_pokecmd;
        }
        state->modal_limit_counter++;

        int use_pf = (enablePF == 1) && (compOL == 1);

        // Emit immediately on critical path when PF is disabled
        if (!use_pf)
        {
            imgout->md[0].write = 1;
            memcpy(imgout->array.F, state->mvaloutapply, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imgout, NULL);
        }

        // Pseudo-Open-Loop (PSOL) Reconstruction & Optional Predictive Control
        if (compOL == 1)
        {
            int   next_tstep     = (state->DMtstep + 1) % MFILT_NBDMTSTEP;
            float latencytotalfr = latencyhardwfr + latencysoftwfr;
            if (latencytotalfr < 1.0f)
            {
                latencytotalfr = 1.0f;
            }
            if (latencytotalfr > (float) (MFILT_NBDMTSTEP - 3))
            {
                latencytotalfr = (float) (MFILT_NBDMTSTEP - 3);
            }
            int   latint   = (int) latencytotalfr;
            float latfrac  = latencytotalfr - (float) latint;
            int   DMtstep1 = state->DMtstep - latint;
            int   DMtstep0 = DMtstep1 - 1;
            while (DMtstep1 < 0)
            {
                DMtstep1 += MFILT_NBDMTSTEP;
            }
            while (DMtstep0 < 0)
            {
                DMtstep0 += MFILT_NBDMTSTEP;
            }

            uint64_t PFcnt = state->imgmvalPF.md[0].cnt0;
            if (use_pf && state->semindex_PF >= 0)
            {
                while (ImageStreamIO_semtrywait(state->imgmvalPF.im, state->semindex_PF) == 0)
                {
                }
            }

            const float *restrict dm_b0  = &state->mvalDMbuff[(size_t) DMtstep0 * NBmode];
            const float *restrict dm_b1  = &state->mvalDMbuff[(size_t) DMtstep1 * NBmode];
            const float *restrict mwfsf  = state->imgmWFSfact.im->array.F;
            float       *restrict ol_out = state->imgOLmval.im->array.F;
            float w_b1                   = 1.0f - latfrac;

            // Lazy-load or reload Dynamic PSOL kernel off hot path when enabled/changed
            if (dynpsol_enable == 1)
            {
                int need_load = (dynpsol_reload == 1) ||
                                (strcmp(dynpsol_kernel, state->dyn_loaded_path) != 0) ||
                                (!state->dyn_ready && state->dyn_prev_enable == 0);
                if (need_load)
                {
                    (void) mfilt_load_dyn_kernel(processinfo, state, NBmode, dynpsol_kernel);
                    dynpsol_reload = 0;
                }
                dynpsol_mode  = state->dyn_ready ? state->dyn_mode : 0u;
                dynpsol_ntaps = state->dyn_ready ? state->dyn_ntaps : 0u;
                dynpsol_nmode = state->dyn_ready ? state->dyn_nmode : 0u;
            }
            else if (state->dyn_prev_enable != 0)
            {
                dynpsol_mode  = 0;
                dynpsol_ntaps = 0;
                dynpsol_nmode = 0;
            }
            state->dyn_prev_enable = dynpsol_enable;

            state->imgOLmval.md[0].write = 1;
            if (dynpsol_enable == 1 && state->dyn_ready)
            {
                uint32_t n_dyn = state->dyn_nmode;
                uint32_t L     = state->dyn_ntaps;
                memset(state->mvalDMOL, 0, sizeof(float) * n_dyn);

                if (state->dyn_mode == 1)
                {
                    for (uint32_t l = 1; l < L; l++)
                    {
                        if (!state->dyn_tap_nz[l])
                        {
                            continue;
                        }
                        uint32_t slot =
                            (uint32_t) (state->DMtstep - (int) l) & (MFILT_NBDMTSTEP - 1);
                        const float *restrict u_l = &state->mvalDMbuff[(size_t) slot * NBmode];
                        const float *restrict h_l = &state->dyn_H_diag[(size_t) l * n_dyn];
                        for (uint32_t mi = 0; mi < n_dyn; mi++)
                        {
                            state->mvalDMOL[mi] += h_l[mi] * u_l[mi];
                        }
                    }
                }
                else if (state->dyn_mode == 2)
                {
                    for (uint32_t l = 1; l < L; l++)
                    {
                        if (!state->dyn_tap_nz[l])
                        {
                            continue;
                        }
                        uint32_t slot =
                            (uint32_t) (state->DMtstep - (int) l) & (MFILT_NBDMTSTEP - 1);
                        const float *restrict u_l = &state->mvalDMbuff[(size_t) slot * NBmode];
                        const float *restrict h_l =
                            &state->dyn_H_mimo[(size_t) l * n_dyn * n_dyn];
#ifdef HAVE_BLAS
                        cblas_sgemv(CblasRowMajor, CblasNoTrans, (int) n_dyn, (int) n_dyn, 1.0f,
                                    h_l, (int) n_dyn, u_l, 1, 1.0f, state->mvalDMOL, 1);
#else
                        for (uint32_t s = 0; s < n_dyn; s++)
                        {
                            const float *restrict h_row = &h_l[(size_t) s * n_dyn];
                            float acc                   = 0.0f;
                            for (uint32_t q = 0; q < n_dyn; q++)
                            {
                                acc += h_row[q] * u_l[q];
                            }
                            state->mvalDMOL[s] += acc;
                        }
#endif
                    }
                }

                // Hybrid fallback for uncalibrated high-order modes (mi >= n_dyn)
                for (uint32_t mi = n_dyn; mi < NBmode; mi++)
                {
                    state->mvalDMOL[mi] = latfrac * dm_b0[mi] + w_b1 * dm_b1[mi];
                }
                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    ol_out[mi] = (psol_WFSfact * mwfsf[mi]) * wfs_in[mi] - state->mvalDMOL[mi];
                }
            }
            else
            {
                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpmDMval     = latfrac * dm_b0[mi] + w_b1 * dm_b1[mi];
                    state->mvalDMOL[mi] = tmpmDMval;
                    ol_out[mi]          = (psol_WFSfact * mwfsf[mi]) * wfs_in[mi] - tmpmDMval;
                }
            }
            processinfo_update_output_stream(processinfo, state->imgOLmval.im, NULL);

            if (use_pf)
            {
                if (PF_NBblock > 0 && PF_maxwaitus > 0)
                {
                    struct timespec ts_dl;
                    clock_gettime(CLOCK_REALTIME, &ts_dl);
                    uint64_t add_ns  = (uint64_t) PF_maxwaitus * 1000ULL;
                    ts_dl.tv_sec    += (time_t) ((ts_dl.tv_nsec + add_ns) / 1000000000ULL);
                    ts_dl.tv_nsec    = (long) ((ts_dl.tv_nsec + add_ns) % 1000000000ULL);
                    uint64_t PFcntOK = PFcnt + (uint64_t) PF_NBblock;

                    while (state->imgmvalPF.md[0].cnt0 < PFcntOK)
                    {
                        if (state->semindex_PF >= 0)
                        {
                            if (ImageStreamIO_semtimedwait(state->imgmvalPF.im,
                                                           state->semindex_PF, &ts_dl) != 0)
                            {
                                break;
                            }
                        }
                        else
                        {
                            struct timespec now;
                            clock_gettime(CLOCK_REALTIME, &now);
                            if (now.tv_sec > ts_dl.tv_sec ||
                                (now.tv_sec == ts_dl.tv_sec && now.tv_nsec >= ts_dl.tv_nsec))
                            {
                                break;
                            }
                            usleep(5);
                        }
                    }
                    if (state->imgmvalPF.md[0].cnt0 < PFcntOK)
                    {
                        PF_timeout_cnt++;
                    }
                }

                const float *restrict mval_pf  = state->imgmvalPF.im->array.F;
                const float *restrict pf_stat  = state->imgoutPFstat.im->array.F;
                const float *restrict mix_fact = state->imgmPFmixfact.im->array.F;
                float       *restrict mix_out  = state->imgmPFmix.im->array.F;

                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float st = pf_stat[mi];
                    if (st < 0.0f)
                    {
                        st = 0.0f;
                    }
                    else if (st > 1.0f)
                    {
                        st = 1.0f;
                    }
                    float mixf = PFmixcoeff * mix_fact[mi] * st;
                    if (mixf < 0.0f)
                    {
                        mixf = 0.0f;
                    }
                    else if (mixf > 1.0f)
                    {
                        mixf = 1.0f;
                    }
                    mix_out[mi] = mixf;

                    if (mixf > 0.0f)
                    {
                        // PSOL modevalOL/PF has +phi_turb sign in DM units; scale WFS zpo by 1/g
                        float wfs_to_dm = psol_WFSfact * mwfsf[mi];
                        float cmd_pf    = wfs_to_dm * zpo[mi] - mval_pf[mi];
                        float dmc_new   = mixf * cmd_pf + (1.0f - mixf) * state->mvalDMc[mi];
                        float limit   = ml[mi];
                        if (dmc_new > limit)
                        {
                            dmc_new = limit;
                            state->mlimitcntarray[mi]++;
                        }
                        else if (dmc_new < -limit)
                        {
                            dmc_new = -limit;
                            state->mlimitcntarray[mi]++;
                        }
                        state->mvalDMc[mi] = dmc_new;
                        if (auxDMmvalenable == 1)
                        {
                            state->mvalout[mi] = dmc_new + auxDMfact * aux[mi];
                        }
                        else
                        {
                            state->mvalout[mi] = dmc_new;
                        }
                        state->mvaloutapply[mi] =
                            state->mvalout[mi] + state->selfRMpokecmd[mi] + state->ogprobecmd[mi];
                    }
                }
                if (testOL == 1)
                {
                    state->mvaloutapply[state->testOL_active_mode] += state->testOL_pokecmd;
                }

                imgout->md[0].write = 1;
                memcpy(imgout->array.F, state->mvaloutapply, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgout, NULL);

                if (PFmixcoeff != state->prev_pfmix ||
                    state->imgmPFmixfact.md[0].cnt0 != state->prev_mpf_cnt0)
                {
                    state->prev_pfmix   = PFmixcoeff;
                    state->prev_mpf_cnt0 = state->imgmPFmixfact.md[0].cnt0;
                    ImageStreamIO_UpdateIm(state->imgmPFmix.im);
                }

                // Store prediction history and evaluate delayed prediction residual
                memcpy(&state->imgcbuff_mvalPF.im->array.F[(size_t) state->PFcbuff_index * NBmode],
                       mval_pf, sizeof(float) * NBmode);
                state->PFcbuff_index = (state->PFcbuff_index + 1) % MFILT_PFCBUFF_SIZE;

                int ti1 = (int) state->PFcbuff_index - 1 - latint;
                int ti0 = ti1 - 1;
                while (ti1 < 0)
                {
                    ti1 += MFILT_PFCBUFF_SIZE;
                }
                while (ti0 < 0)
                {
                    ti0 += MFILT_PFCBUFF_SIZE;
                }
                const float *restrict pf_b0 =
                    &state->imgcbuff_mvalPF.im->array.F[(size_t) ti0 * NBmode];
                const float *restrict pf_b1 =
                    &state->imgcbuff_mvalPF.im->array.F[(size_t) ti1 * NBmode];
                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    double pfold       = (double) (latfrac * pf_b0[mi] + w_b1 * pf_b1[mi]);
                    state->mvalPFold[mi] = pfold;
                    state->mvalPFres[mi] = (double) ol_out[mi] - pfold;
                }
            }

            // Record actual applied DM command (ctrl + selfRM/testOL probes) into PSOL ring buffer
            memcpy(&state->mvalDMbuff[(size_t) state->DMtstep * NBmode], state->mvaloutapply,
                   sizeof(float) * NBmode);
            state->DMtstep = next_tstep;

            // Optional testOL identification step
            if (testOL == 1 && state->psOL_probe != NULL)
            {
                uint32_t nbspl = state->psOL_cap;
                uint32_t tm    = state->testOL_active_mode;
                if (testOLcnt < nbspl)
                {
                    state->psOL_probe[testOLcnt] = state->testOL_pokecmd;
                    state->psOL_wfs[testOLcnt]   = wfs_in[tm];
                    state->psOL_dm[testOLcnt]    = state->mvalout[tm];
                    testOLcnt++;
                }
                if (testOLcnt >= nbspl)
                {
                    solve_and_apply_testOL(processinfo, fps, state, nbspl);
                    testOLcnt = 0;
                    if (testOLloop == 0)
                    {
                        testOL = 0;
                    }
                }
            }
        }

        // Offload Loop
        if (offload == 1)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                state->imgoffloadmgain.im->array.F[mi] =
                    state->imgoffloadmgainfact.im->array.F[mi] * offloadloopgain;
                state->imgoffloadmmult.im->array.F[mi] =
                    state->imgoffloadmmultfact.im->array.F[mi] * offloadloopmult;
                state->imgoffloadmlimit.im->array.F[mi] =
                    state->imgoffloadmlimitfact.im->array.F[mi] * offloadlooplimit;

                float val = state->imgmvaloffloadDM.im->array.F[mi];
                val += state->imgoffloadmgain.im->array.F[mi] * imgout->array.F[mi];
                val *= state->imgoffloadmmult.im->array.F[mi];
                float limit = state->imgoffloadmlimit.im->array.F[mi];
                if (val > limit)
                {
                    val = limit;
                }
                else if (val < -limit)
                {
                    val = -limit;
                }
                state->imgmvaloffloadDM.im->array.F[mi] = val;
            }
            processinfo_update_output_stream(processinfo, state->imgmvaloffloadDM.im, NULL);
        }

        // Telemetry Double-Buffering & Modal RMS Statistics
        if (comptbuff == 1 && state->tbuff_alloc_size > 0)
        {
            uint32_t eff_tb = state->tbuff_alloc_size;
            size_t   kkoff  = ((size_t) state->tbuffslice * eff_tb + state->tbuffindex) * NBmode;
            memcpy(&state->imgtbuff_mvalWFS.im->array.F[kkoff], wfs_in, sizeof(float) * NBmode);
            memcpy(&state->imgtbuff_mvalOL.im->array.F[kkoff], state->imgOLmval.im->array.F,
                   sizeof(float) * NBmode);
            memcpy(&state->imgtbuff_mvalprobe.im->array.F[kkoff], state->ogprobecmd,
                   sizeof(float) * NBmode);
            if (auxDMmvalenable == 1)
            {
                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    state->imgtbuff_mvalDM.im->array.F[kkoff + mi] =
                        state->mvalout[mi] - auxDMfact * aux[mi];
                }
            }
            else
            {
                memcpy(&state->imgtbuff_mvalDM.im->array.F[kkoff], state->mvalout,
                       sizeof(float) * NBmode);
            }

            const float *restrict ol_cur = state->imgOLmval.im->array.F;
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                double dv = (double) state->mvalout[mi];
                double wv = (double) wfs_in[mi];
                double ov = (double) ol_cur[mi];
                double pv = state->mvalPFres[mi];
                state->mvalDMave[mi]    += dv;
                state->mvalDMrms[mi]    += dv * dv;
                state->mvalWFSave[mi]   += wv;
                state->mvalWFSrms[mi]   += wv * wv;
                state->mvalOLave[mi]    += ov;
                state->mvalOLrms[mi]    += ov * ov;
                state->mvalPFresave[mi] += pv;
                state->mvalPFresrms[mi] += pv * pv;
            }
            state->stats_accum_cnt++;
            state->tbuffindex++;

            if (state->tbuffindex >= eff_tb)
            {
                int done_slice    = state->tbuffslice;
                state->tbuffindex = 0;
                state->tbuffslice = 1 - done_slice;

                state->imgtbuff_mvalDM.md[0].cnt1    = (uint64_t) done_slice;
                state->imgtbuff_mvalprobe.md[0].cnt1 = (uint64_t) done_slice;
                state->imgtbuff_mvalWFS.md[0].cnt1   = (uint64_t) done_slice;
                state->imgtbuff_mvalOL.md[0].cnt1    = (uint64_t) done_slice;
                processinfo_update_output_stream(processinfo, state->imgtbuff_mvalDM.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgtbuff_mvalprobe.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgtbuff_mvalWFS.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgtbuff_mvalOL.im, NULL);

                double inv_n =
                    (state->stats_accum_cnt > 0) ? (1.0 / (double) state->stats_accum_cnt) : 1.0;
                double inv_l = (state->modal_limit_counter > 0)
                                   ? (1.0 / (double) state->modal_limit_counter)
                                   : 1.0;
                for (uint32_t mi = 0; mi < NBmode; mi++)
                {
                    state->imgmlimitcntfrac.im->array.F[mi] =
                        (float) ((double) state->mlimitcntarray[mi] * inv_l);
                    state->mlimitcntarray[mi] = 0;

                    state->imgmvalDMave.im->array.F[mi] = (float) (state->mvalDMave[mi] * inv_n);
                    state->imgmvalDMrms.im->array.F[mi] =
                        (float) sqrt(state->mvalDMrms[mi] * inv_n);
                    state->imgmvalWFSave.im->array.F[mi] = (float) (state->mvalWFSave[mi] * inv_n);
                    state->imgmvalWFSrms.im->array.F[mi] =
                        (float) sqrt(state->mvalWFSrms[mi] * inv_n);
                    state->imgmvalOLave.im->array.F[mi] = (float) (state->mvalOLave[mi] * inv_n);
                    state->imgmvalOLrms.im->array.F[mi] =
                        (float) sqrt(state->mvalOLrms[mi] * inv_n);
                    state->imgmvalPFresave.im->array.F[mi] =
                        (float) (state->mvalPFresave[mi] * inv_n);
                    state->imgmvalPFresrms.im->array.F[mi] =
                        (float) sqrt(state->mvalPFresrms[mi] * inv_n);

                    state->mvalDMave[mi]    = 0.0;
                    state->mvalDMrms[mi]    = 0.0;
                    state->mvalWFSave[mi]   = 0.0;
                    state->mvalWFSrms[mi]   = 0.0;
                    state->mvalOLave[mi]    = 0.0;
                    state->mvalOLrms[mi]    = 0.0;
                    state->mvalPFresave[mi] = 0.0;
                    state->mvalPFresrms[mi] = 0.0;
                }
                state->modal_limit_counter = 0;
                state->stats_accum_cnt     = 0;

                processinfo_update_output_stream(processinfo, state->imgmlimitcntfrac.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalDMave.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalDMrms.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalWFSave.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalWFSrms.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalOLave.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalOLrms.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalPFresave.im, NULL);
                processinfo_update_output_stream(processinfo, state->imgmvalPFresrms.im, NULL);

                if (autolim == 1 && looplimit > 1.0e-9f)
                {
                    for (uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        double dm_rms_i = (double) state->imgmvalDMrms.im->array.F[mi];
                        state->autolimDMsigma[mi] =
                            (1.0 - (double) autolimprobegain) * state->autolimDMsigma[mi] +
                            (double) autolimprobegain * dm_rms_i;
                        float cliplim = autolimsigmafact * (float) state->autolimDMsigma[mi];
                        state->imgmlimitfact.im->array.F[mi] = cliplim / looplimit;
                    }
                    ImageStreamIO_UpdateIm(state->imgmlimitfact.im);
                }
            }
        }
    }
    else if (selfRMenable == 1 && enablePF == 0)
    {
        // Open-loop selfRM: propagate modal probe perturbations while integrator is frozen
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            state->mvaloutapply[mi] = state->mvalout[mi] + state->selfRMpokecmd[mi];
        }
        imgout->md[0].write = 1;
        memcpy(imgout->array.F, state->mvaloutapply, sizeof(float) * NBmode);
        processinfo_update_output_stream(processinfo, imgout, NULL);
    }

    if (selfRMenable == 1)
    {
        uint32_t max_z      = state->imgselfRM.md[0].size[2];
        uint32_t eff_zsize  = (selfRMzsize > 0 && selfRMzsize <= max_z) ? selfRMzsize : max_z;
        uint32_t eff_nbmode = (selfRMnbmode > 0 && selfRMnbmode < NBmode) ? selfRMnbmode : NBmode;
        uint32_t eff_nbiter = (selfRMnbiter > 0) ? selfRMnbiter : 1;
        float    eff_ampl   = (fabsf(selfRMpokeampl) > 1.0e-9f) ? selfRMpokeampl : 0.01f;

        if (state->selfRM_pokecnt == 0 && state->selfRM_pokemode == 0 && state->blockcnt == 0 &&
            state->selfRMiter == 0)
        {
            processinfo_WriteMessage(processinfo, "init selfRM");
            memset(state->imgselfRM.im->array.F, 0, sizeof(float) * NBmode * NBmode * max_z);
            memset(state->selfRMpokecmd, 0, sizeof(float) * NBmode);
        }

        if (state->selfRM_pokecnt == 0 && state->selfRM_pokemode == 0)
        {
            if (state->blockcnt == 0)
            {
                state->selfRMpokesign = 1.0f - 2.0f * (float) ((state->selfRMiter / 4) % 2);
            }
            processinfo_WriteMessage_fmt(processinfo, "sRM %u/%u ppol %d sign %+3.1f",
                                         state->selfRMiter, eff_nbiter, state->selfRMpokeparity,
                                         state->selfRMpokesign);
        }

        uint32_t pkmode   = state->selfRM_pokemode;
        float    signmult = ((pkmode % 2 == 0) && (state->selfRMpokeparity == 1)) ? -1.0f : 1.0f;

        if (state->selfRM_pokecnt < eff_zsize)
        {
            state->selfRMpokecmd[pkmode] = state->selfRMpokesign * eff_ampl * signmult;
            float scale = 0.5f * signmult * state->selfRMpokesign / eff_ampl / (float) eff_nbiter;
            long  base  = (long) NBmode * NBmode * state->selfRM_pokecnt + (long) NBmode * pkmode;
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                state->imgselfRM.im->array.F[base + mi] += scale * imginWFS->array.F[mi];
            }
        }
        else
        {
            state->selfRMpokecmd[pkmode] = 0.0f;
        }

        state->selfRM_pokecnt++;
        if (state->selfRM_pokecnt > eff_zsize + selfRMnbsettlestep)
        {
            state->selfRMpokecmd[pkmode] = 0.0f;
            if (((state->selfRMiter % 8) < 2) || ((state->selfRMiter % 8) > 5))
            {
                state->selfRMpokesign *= -1.0f;
            }
            state->blockcnt++;
            if (state->blockcnt == 2)
            {
                state->selfRM_pokemode++;
                state->blockcnt = 0;
            }
            state->selfRM_pokecnt = 0;
        }

        if (state->selfRM_pokemode >= eff_nbmode)
        {
            state->selfRMpokeparity = 1 - state->selfRMpokeparity;
            state->selfRM_pokemode  = 0;
            state->selfRM_pokecnt   = 0;
            state->selfRMiter++;
        }

        if (state->selfRMiter >= eff_nbiter)
        {
            state->selfRMiter       = 0;
            state->selfRM_pokemode  = 0;
            state->selfRM_pokecnt   = 0;
            state->blockcnt         = 0;
            state->selfRMpokeparity = 0;
            memset(state->selfRMpokecmd, 0, sizeof(float) * NBmode);
            if (loopON == 0 && enablePF == 0)
            {
                imgout->md[0].write = 1;
                memcpy(imgout->array.F, state->mvalout, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgout, NULL);
            }
            processinfo_update_output_stream(processinfo, state->imgselfRM.im, NULL);
            if (selfRMsaveFITS)
            {
                save_fits(state->imgselfRM.name, "!selfRM.fits");
            }
            selfRMenable = 0;
            processinfo_WriteMessage(processinfo, "selfRM done");
        }
    }

    if (recburst == 1)
    {
        uint32_t rmode = (recburst_mode < NBmode) ? recburst_mode : 0;
        if (state->recburstsample == 0)
        {
            char recfname[STRINGMAXLEN_FILENAME];
            WRITE_FILENAME(recfname, "mfilt.rec.m%u.log", rmode);
            state->fprec = fopen(recfname, "w");
            if (state->fprec != NULL)
            {
                fprintf(state->fprec, "# 1:step 2:WFS 3:DM 4:OL 5:PF\n");
            }
        }
        if (state->fprec != NULL)
        {
            fprintf(state->fprec, "%05lu  %+8.6f  %+8.6f  %+8.6f  %+8.6f\n",
                    state->recburstsample, imginWFS->array.F[rmode], imgout->array.F[rmode],
                    state->imgOLmval.im->array.F[rmode], state->imgmvalPF.im->array.F[rmode]);
        }
        state->recburstsample++;
        if (state->recburstsample >= (uint64_t) recburst_nbsample)
        {
            state->recburstsample = 0;
            recburst              = 0;
            if (state->fprec != NULL)
            {
                fclose(state->fprec);
                state->fprec = NULL;
            }
        }
    }
    else
    {
        state->recburstsample = 0;
    }
}

/* ================================================================
 * 2.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X)                                                                             \
    X(".inmval", inmval, FPTYPE_STREAMNAME, 1,                                                    \
      FPFLAG_DEFAULT_TRIGGER_STREAM | FPFLAG_PRIMARY_CLI_INPUT, "input mode values from WFS")     \
    X(".outmval", outmval, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, \
      "output mode values to DM")                                                                 \
    X(".loopgain", &loopgain, FPTYPE_FLOAT32, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop gain")                     \
    X(".loopmult", &loopmult, FPTYPE_FLOAT32, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop mult")                     \
    X(".looplimit", &looplimit, FPTYPE_FLOAT32, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop limit")                    \
    X(".loopZERO", &loopZERO, FPTYPE_ONOFF, 0,                                                    \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop zero")                     \
    X(".AOloopindex", &AOloopindex, FPTYPE_UINT64, 1,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "AO loop index")                           \
    X(".loopON", &loopON, FPTYPE_ONOFF, 0,                                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop on/off (off=freeze)")      \
    X(".loopNBstep", &loopNBstep, FPTYPE_INT64, 0,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "loop nb steps (-1 = inf)")      \
    X(".comp.OLmodes", &compOL, FPTYPE_ONOFF, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "compute open loop modes")       \
    X(".comp.WFSfact", &psol_WFSfact, FPTYPE_FLOAT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "ampl correction factor WFS")    \
    X(".comp.latencyhardwfr", &latencyhardwfr, FPTYPE_FLOAT32, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "hw DM-to-WFS latency [fr]")     \
    X(".comp.latencysoftwfr", &latencysoftwfr, FPTYPE_FLOAT32, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "sw latency [frame]")            \
    X(".comp.dynpsol.enable", &dynpsol_enable, FPTYPE_ONOFF, 0,                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "enable 3D FIR dynamic PSOL")    \
    X(".comp.dynpsol.kernel", dynpsol_kernel, FPTYPE_FITSFILENAME, 0,                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "H_diag/H_mimo FITS kernel")     \
    X(".comp.dynpsol.reload", &dynpsol_reload, FPTYPE_ONOFF, 0,                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "reload dynamic PSOL kernel")    \
    X(".comp.dynpsol.mode", &dynpsol_mode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,               \
      "active PSOL mode (0=2tap 1=diag 2=MIMO)")                                                  \
    X(".comp.dynpsol.ntaps", &dynpsol_ntaps, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "active dynamic PSOL tap length L")                                                         \
    X(".comp.dynpsol.nmode", &dynpsol_nmode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT,             \
      "active dynamic PSOL modal span")                                                           \
    X(".comp.ogprobe.enable", &ogprobe_enable, FPTYPE_ONOFF, 0,                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "enable online OG DM probe")     \
    X(".comp.ogprobe.ampl", &ogprobe_ampl, FPTYPE_FLOAT32, 0,                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "online OG probe amplitude")     \
    X(".comp.ogprobe.nbmode", &ogprobe_nbmode, FPTYPE_UINT32, 0,                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "leading modes probed (0=all)")  \
    X(".comp.ogprobe.stride", &ogprobe_stride, FPTYPE_UINT32, 0,                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mode stride for sparse probes") \
    X(".comp.ogprobe.hold", &ogprobe_hold, FPTYPE_UINT32, 0,                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "frames per probe chip (>=1)")   \
    X(".comp.ogprobe.seed", &ogprobe_seed, FPTYPE_UINT64, 0,                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "PRNG seed for Rademacher OG probe")               \
    X(".comp.tbuff", &comptbuff, FPTYPE_ONOFF, 0,                                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "compute telemetry buffers")     \
    X(".comp.autolim", &autolim, FPTYPE_ONOFF, 0,                                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "automatic modal limits")        \
    X(".comp.autolimprobegain", &autolimprobegain, FPTYPE_FLOAT32, 0,                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "sigma measurement gain")        \
    X(".comp.autolimsigmafact", &autolimsigmafact, FPTYPE_FLOAT32, 0,                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "autolimit sigma clip factor")   \
    X(".comp.tbuffsize", &tbuffsize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "buffer time size")                                                                         \
    X(".auxDMmval.enable", &auxDMmvalenable, FPTYPE_ONOFF, 0,                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mixing aux DM mode vals")       \
    X(".auxDMmval.mixfact", &auxDMmvalmixfact, FPTYPE_FLOAT32, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mixing mult factor")            \
    X(".auxDMmval.modulate", &auxDMmvalmodulate, FPTYPE_ONOFF, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "modulate auxDM temporally")     \
    X(".auxDMmval.modperiod", &auxDMmvalmodperiod, FPTYPE_FLOAT32, 0,                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "auxDM mod period [frame]")      \
    X(".PF.enable", &enablePF, FPTYPE_ONOFF, 0,                                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "enable predictive filter")      \
    X(".PF.NBblock", &PF_NBblock, FPTYPE_UINT32, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "nb blocks to wait from")        \
    X(".PF.maxwaitus", &PF_maxwaitus, FPTYPE_UINT32, 0,                                           \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "max wait time blks [us]")       \
    X(".PF.mixcoeff", &PFmixcoeff, FPTYPE_FLOAT32, 0,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mixing coeff")                  \
    X(".PF.timeout_cnt", &PF_timeout_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "PF wait timeouts")                                                                         \
    X(".autoloop.enable", &autoloopenable, FPTYPE_ONOFF, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "autoloop self-test")                              \
    X(".autoloop.sleep", &autoloopsleep, FPTYPE_FLOAT32, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "loop sleep time")                                 \
    X(".selfRM.enable", &selfRMenable, FPTYPE_ONOFF, 0,                                           \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "start self RM measurement")     \
    X(".selfRM.NBmode", &selfRMnbmode, FPTYPE_UINT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "number of mode poked")          \
    X(".selfRM.pokeampl", &selfRMpokeampl, FPTYPE_FLOAT32, 0,                                     \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "poke amplitude")                \
    X(".selfRM.zsize", &selfRMzsize, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,   \
      "nb time steps recorded")                                                                   \
    X(".selfRM.nbiter", &selfRMnbiter, FPTYPE_UINT32, 0,                                          \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "nb iterations averaged")        \
    X(".selfRM.nbsettle", &selfRMnbsettlestep, FPTYPE_UINT32, 0,                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "nb loop iter to settle")        \
    X(".selfRM.saveFITS", &selfRMsaveFITS, FPTYPE_ONOFF, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "save !selfRM.fits on done")     \
    X(".testOL.enable", &testOL, FPTYPE_ONOFF, 0,                                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "OL reconstruction test")        \
    X(".testOL.loop", &testOLloop, FPTYPE_ONOFF, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "run inloop")                    \
    X(".testOL.updategain", &testOLupdategain, FPTYPE_FLOAT32, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "update gain")                   \
    X(".testOL.mode", &testOLmode, FPTYPE_UINT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mode index")                    \
    X(".testOL.ampl", &testOLampl, FPTYPE_FLOAT32, 0,                                             \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "amplitude")                     \
    X(".testOL.nbsample", &testOLnbsample, FPTYPE_UINT32, 0,                                      \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "number of samples")             \
    X(".testOL.cnt", &testOLcnt, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_OUTPUT, "samples count")        \
    X(".offload.enable", &offload, FPTYPE_ONOFF, 0,                                               \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "offload output ON/OFF")         \
    X(".offload.loopgain", &offloadloopgain, FPTYPE_FLOAT32, 0,                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "offload loop gain")             \
    X(".offload.loopmult", &offloadloopmult, FPTYPE_FLOAT32, 0,                                   \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "offload loop mult")             \
    X(".offload.looplimit", &offloadlooplimit, FPTYPE_FLOAT32, 0,                                 \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "offload loop limit")            \
    X(".rec.enable", &recburst, FPTYPE_ONOFF, 0,                                                  \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "telemetry burst write")         \
    X(".rec.mode", &recburst_mode, FPTYPE_UINT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "mode index")                    \
    X(".rec.nbstep", &recburst_nbsample, FPTYPE_UINT32, 0,                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN, "number of steps recorded")


/* ================================================================
 * 3.  COMPUTE WRAPPER
 * ============================================================= */

MILK_PARAMS_DEFINE(modalfilter, FPS_PARAMS);

static MILK_HOT errno_t modalfilter_compute(const MILK_RUNNABLE *self)
{
    IMGID imginWFS = imgid_make_from_fpskey(inmval, dcfpsname, ".inmval");
    resolveIMGID(&imginWFS, ERRMODE_WARN, dcimg, dcnimg);
    if (imginWFS.ID == -1)
    {
        return RETURN_FAILURE;
    }

    uint32_t NBmode = imginWFS.md[0].size[0];

    IMGID imgout = stream_connect_create_2Df32(outmval, NBmode, 1);

    MFILT_STATE *state = modal_filter_init(NBmode);

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    modal_filter_step(processinfo, milk_data.fpsptr, imginWFS.im, imgout.im, state);
    MILK_PROCINFO_END(self)

    modal_filter_cleanup(state);
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_AOloopControl_modalfilter = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "mfilt",
                 .cmdkey      = "modalfilter",
                 .description = "Modal Filtering AO processing",
                 .description_long =
                     "Apply modal filtering, pseudo-open-loop (PSOL) reconstruction, and "
                     "predictive control mixing in a real-time AO control loop." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(modalfilter),
    .compute = modalfilter_compute,
};
EXPORT_FPSEXEC(cacao, "mfilt", runnable_AOloopControl_modalfilter);
