/**
 * @file    modalfilter.c
 * @brief   Apply modal filtering following FPS practices
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "timeutils.h"

#include "fps.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"
#include "processtools.h"

#include "modalfilter.h"

/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "mfilt",
    .cmdkey      = "modalfilter",
    .description =
        "Modal Filtering AO processing"
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     *inmval_ptr             = NULL;
static char     *outmval_ptr            = NULL;
static float    *loopgain_ptr           = NULL;
static float    *loopmult_ptr           = NULL;
static float    *looplimit_ptr          = NULL;
static uint64_t *loopZERO_ptr           = NULL;
static uint64_t *AOloopindex_ptr        = NULL;
static uint64_t *loopON_ptr             = NULL;
static int64_t  *loopNBstep_ptr         = NULL;
static uint64_t *compOL_ptr             = NULL;
static float    *psol_WFSfact_ptr       = NULL;
static float    *latencyhardwfr_ptr     = NULL;
static float    *latencysoftwfr_ptr     = NULL;
static uint64_t *comptbuff_ptr          = NULL;
static uint64_t *autolim_ptr            = NULL;
static float    *autolimprobegain_ptr   = NULL;
static float    *autolimsigmafact_ptr   = NULL;
static uint32_t *tbuffsize_ptr          = NULL;
static uint64_t *auxDMmvalenable_ptr    = NULL;
static float    *auxDMmvalmixfact_ptr   = NULL;
static uint64_t *auxDMmvalmodulate_ptr  = NULL;
static float    *auxDMmvalmodperiod_ptr = NULL;
static uint64_t *enablePF_ptr           = NULL;
static uint32_t *PF_NBblock_ptr         = NULL;
static uint32_t *PF_maxwaitus_ptr       = NULL;
static float    *PFmixcoeff_ptr         = NULL;
static uint64_t *autoloopenable_ptr     = NULL;
static float    *autoloopsleep_ptr      = NULL;
static uint64_t *selfRMenable_ptr       = NULL;
static uint32_t *selfRMnbmode_ptr       = NULL;
static float    *selfRMpokeampl_ptr     = NULL;
static uint32_t *selfRMzsize_ptr        = NULL;
static uint32_t *selfRMnbiter_ptr       = NULL;
static uint32_t *selfRMnbsettlestep_ptr = NULL;
static uint64_t *testOL_ptr             = NULL;
static uint64_t *testOLloop_ptr         = NULL;
static float    *testOLupdategain_ptr   = NULL;
static uint32_t *testOLmode_ptr         = NULL;
static float    *testOLampl_ptr         = NULL;
static uint32_t *testOLnbsample_ptr     = NULL;
static uint32_t *testOLcnt_ptr          = NULL;
static uint64_t *offload_ptr            = NULL;
static float    *offloadloopgain_ptr    = NULL;
static float    *offloadloopmult_ptr    = NULL;
static float    *offloadlooplimit_ptr   = NULL;
static uint64_t *recburst_ptr           = NULL;
static uint32_t *recburst_mode_ptr      = NULL;
static uint32_t *recburst_nbsample_ptr  = NULL;

static uint64_t processinfo_change_cnt_local = 0;

typedef struct {
    float *mvalDMc;
    float *mvalout;
    float *mvaloutapply;
    
    // OL
    float *mvalDMbuff;
    float *mvalDMOL;
    int DMtstep;
    IMGID imgOLmval;
    
    // Aux
    IMGID imgauxmDM;
    
    // PF
    IMGID imgmvalPF;
    IMGID imgmPFmixfact;
    IMGID imgmPFmix;
    IMGID imgcbuff_mvalPF;
    uint32_t PFcbuff_index;
    double *mvalPFres;
    double *mvalPFold;
    
    // Gain/Mult/Limit/Zero
    IMGID imgmgain;
    IMGID imgmgainfact;
    IMGID imgmmult;
    IMGID imgmmultfact;
    IMGID imgmlimit;
    IMGID imgmlimitfact;
    IMGID imgmzeropoint;
    
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
    long modal_limit_counter;
    IMGID imgmlimitcntfrac;
    
    // Telemetry/Stats
    IMGID imgtbuff_mvalDM;
    IMGID imgtbuff_mvalWFS;
    IMGID imgtbuff_mvalOL;
    uint32_t tbuffindex;
    int tbuffslice;
    
    double *mvalDMave;
    double *mvalDMrms;
    double *mvalWFSave;
    double *mvalWFSrms;
    double *mvalOLave;
    double *mvalOLrms;
    double *mvalPFresave;
    double *mvalPFresrms;
    
    double *autolimDMsigma;

    // SelfRM
    IMGID imgselfRM;
    float *selfRMpokecmd;
    int blockcnt;
    int selfRMpokeparity;
    uint32_t selfRMiter;
    uint32_t selfRM_pokemode;
    uint32_t selfRM_pokecnt;
    float selfRMpokesign;
    
    // TestOL
    float *psOL_probe;
    float *psOL_estimate;
    
    // Burst
    uint64_t recburstsample;
    
    // External DM modeval update detection
    IMGID imgmodevalDMf;
    uint64_t imgmodevalDMfcnt0old;

} MFILT_STATE;

/* =============================================================================================== */
/* COMPUTE LOGIC                                                                                   */
/* =============================================================================================== */

static MFILT_STATE* modal_filter_init(uint32_t NBmode) {
    MFILT_STATE *state = (MFILT_STATE*) calloc(1, sizeof(MFILT_STATE));
    
    state->mvalDMc = (float *) calloc(NBmode, sizeof(float));
    state->mvalout = (float *) calloc(NBmode, sizeof(float));
    state->mvaloutapply = (float *) calloc(NBmode, sizeof(float));

    // OL
    int NB_DMtstep = 10;
    state->mvalDMbuff = (float *) calloc(NBmode * NB_DMtstep, sizeof(float));
    state->mvalDMOL = (float *) calloc(NBmode, sizeof(float));
    
    char name[STRINGMAXLEN_STREAMNAME];
    
    WRITE_IMAGENAME(name, "aol%lu_modevalOL", *AOloopindex_ptr);
    state->imgOLmval = stream_connect_create_2Df32(name, NBmode, 1);
    
    WRITE_IMAGENAME(name, "aol%lu_modevalauxDM", *AOloopindex_ptr);
    state->imgauxmDM = stream_connect_create_2Df32(name, NBmode, 1);
    
    // PF
    WRITE_IMAGENAME(name, "aol%lu_modevalPF", *AOloopindex_ptr);
    state->imgmvalPF = stream_connect_create_2Df32(name, NBmode, 1);
    
    WRITE_IMAGENAME(name, "aol%lu_mPFmixfact", *AOloopindex_ptr);
    state->imgmPFmixfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgmPFmixfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgmPFmixfact.im);

    WRITE_IMAGENAME(name, "aol%lu_mPFmix", *AOloopindex_ptr);
    state->imgmPFmix = stream_connect_create_2Df32(name, NBmode, 1);
    
    int PFcbuff_size = 20;
    WRITE_IMAGENAME(name, "aol%lu_modevalPF_cbuff", *AOloopindex_ptr);
    state->imgcbuff_mvalPF = stream_connect_create_2Df32(name,
        PFcbuff_size,
        NBmode);
    
    state->mvalPFres = (double*) calloc(NBmode, sizeof(double));
    state->mvalPFold = (double*) calloc(NBmode, sizeof(double));
    
    // GAIN/MULT/LIMIT
    WRITE_IMAGENAME(name, "aol%lu_mgain", *AOloopindex_ptr);
    state->imgmgain = stream_connect_create_2Df32(name, NBmode, 1);
    
    WRITE_IMAGENAME(name, "aol%lu_mgainfact", *AOloopindex_ptr);
    state->imgmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgmgainfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgmgainfact.im);

    WRITE_IMAGENAME(name, "aol%lu_mmult", *AOloopindex_ptr);
    state->imgmmult = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mmultfact", *AOloopindex_ptr);
    state->imgmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgmmultfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgmmultfact.im);
    
    WRITE_IMAGENAME(name, "aol%lu_mzeropoint", *AOloopindex_ptr);
    state->imgmzeropoint = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mlimit", *AOloopindex_ptr);
    state->imgmlimit = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_mlimitfact", *AOloopindex_ptr);
    state->imgmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgmlimitfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgmlimitfact.im);

    // Offload
    WRITE_IMAGENAME(name, "aol%lu_offloadmgain", *AOloopindex_ptr);
    state->imgoffloadmgain = stream_connect_create_2Df32(name, NBmode, 1);
    
    WRITE_IMAGENAME(name, "aol%lu_offloadmgainfact", *AOloopindex_ptr);
    state->imgoffloadmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgoffloadmgainfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgoffloadmgainfact.im);

    WRITE_IMAGENAME(name, "aol%lu_offloadmmult", *AOloopindex_ptr);
    state->imgoffloadmmult = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_offloadmmultfact", *AOloopindex_ptr);
    state->imgoffloadmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgoffloadmmultfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgoffloadmmultfact.im);

    WRITE_IMAGENAME(name, "aol%lu_offloadmlimit", *AOloopindex_ptr);
    state->imgoffloadmlimit = stream_connect_create_2Df32(name, NBmode, 1);

    WRITE_IMAGENAME(name, "aol%lu_offloadmlimitfact", *AOloopindex_ptr);
    state->imgoffloadmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
    for(uint32_t mi=0; mi<NBmode; mi++) state->imgoffloadmlimitfact.im->array.F[mi] = 1.0;
    ImageStreamIO_UpdateIm(state->imgoffloadmlimitfact.im);
    
    WRITE_IMAGENAME(name, "aol%lu_mvaloffloadDM", *AOloopindex_ptr);
    state->imgmvaloffloadDM = stream_connect_create_2Df32(name, NBmode, 1);

    // Limit Counter
    state->mlimitcntarray = (long*) calloc(NBmode, sizeof(long));
    WRITE_IMAGENAME(name, "aol%lu_mlimitcntfrac", *AOloopindex_ptr);
    state->imgmlimitcntfrac = stream_connect_create_2Df32(name, NBmode, 1);
    
    // Stats
    state->mvalDMave = (double*) calloc(NBmode, sizeof(double));
    state->mvalDMrms = (double*) calloc(NBmode, sizeof(double));
    state->mvalWFSave = (double*) calloc(NBmode, sizeof(double));
    state->mvalWFSrms = (double*) calloc(NBmode, sizeof(double));
    state->mvalOLave = (double*) calloc(NBmode, sizeof(double));
    state->mvalOLrms = (double*) calloc(NBmode, sizeof(double));
    state->mvalPFresave = (double*) calloc(NBmode, sizeof(double));
    state->mvalPFresrms = (double*) calloc(NBmode, sizeof(double));
    state->autolimDMsigma = (double*) calloc(NBmode, sizeof(double));

    // Telemetry Buffers
    WRITE_IMAGENAME(name, "aol%lu_modevalDM_buff", *AOloopindex_ptr);
    state->imgtbuff_mvalDM = stream_connect_create_3Df32(name,
        NBmode,
        *tbuffsize_ptr,
        2);
    WRITE_IMAGENAME(name, "aol%lu_modevalWFS_buff", *AOloopindex_ptr);
    state->imgtbuff_mvalWFS = stream_connect_create_3Df32(name,
        NBmode,
        *tbuffsize_ptr,
        2);
    WRITE_IMAGENAME(name, "aol%lu_modevalOL_buff", *AOloopindex_ptr);
    state->imgtbuff_mvalOL = stream_connect_create_3Df32(name,
        NBmode,
        *tbuffsize_ptr,
        2);
    
    // SelfRM
    WRITE_IMAGENAME(name, "aol%lu_mfiltselfRM", *AOloopindex_ptr);
    state->imgselfRM = stream_connect_create_3Df32(name,
        NBmode,
        NBmode,
        *selfRMzsize_ptr);
    state->selfRMpokecmd = (float*) calloc(NBmode, sizeof(float));
    state->selfRMpokesign = 1.0;
    
    // External DMf
    WRITE_IMAGENAME(name, "aol%lu_modevalDMf", *AOloopindex_ptr);
    state->imgmodevalDMf = stream_connect_create_2Df32(name, NBmode, 1);
    
    return state;
}

static void modal_filter_cleanup(MFILT_STATE *state) {
    if(!state) return;
    free(state->mvalDMc);
    free(state->mvalout);
    free(state->mvaloutapply);
    free(state->mvalDMbuff);
    free(state->mvalDMOL);
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
    free(state->selfRMpokecmd);
    if(state->psOL_probe) free(state->psOL_probe);
    if(state->psOL_estimate) free(state->psOL_estimate);
    free(state);
}

/**
 * @brief Modal filtering computation.
 */
static void modal_filter_step(
    PROCESSINFO *processinfo,
    FUNCTION_PARAMETER_STRUCT *fps,
    IMAGE *imginWFS,
    IMAGE *imgout,
    MFILT_STATE *state)
{
    // Sync external FPS changes to local ProcessInfo
    if (fps) {
        if(fps->md->processinfo_change_cnt != processinfo_change_cnt_local) {
            fps_to_processinfo(fps, processinfo);
            processinfo_change_cnt_local = fps->md->processinfo_change_cnt;
        }
    }

    if (!imginWFS || !imgout || !state) return;

    uint32_t NBmode = imginWFS->md[0].size[0];

    // Zero loop
    if(loopZERO_ptr && ((*loopZERO_ptr) & FPFLAG_ONOFF)) {
        for(uint32_t mi = 0; mi < NBmode; mi++) {
             state->mvalDMc[mi] = 0.0;
             state->mvalout[mi] = 0.0;
             state->mvaloutapply[mi] = 0.0;
        }
        memset(imgout->array.F, 0, sizeof(float) * NBmode);
        processinfo_update_output_stream(processinfo, imgout, NULL);
        (*loopZERO_ptr) &= ~FPFLAG_ONOFF;
    }

    if((*loopON_ptr) == 1) {
        if(*loopNBstep_ptr > 0) {
            *loopNBstep_ptr = *loopNBstep_ptr - 1;
        }
        if(*loopNBstep_ptr == 0) {
            *loopON_ptr = 0;
            (*loopON_ptr) &= ~FPFLAG_ONOFF;
            *loopNBstep_ptr = 1;
        }

        // Aux DM factor
        float auxDMfact = (*auxDMmvalmixfact_ptr);
        if((*auxDMmvalmodulate_ptr) == 1) {
            static double modpha = 0.0;
            modpha += 1.0 / (*auxDMmvalmodperiod_ptr);
            if(modpha > 1.0) modpha -= 1.0;
            auxDMfact *= sinf(2.0f * M_PI * modpha);
        }

        // External DM update
        uint64_t imgmodevalDMfcnt0 = state->imgmodevalDMf.md->cnt0;
        if(imgmodevalDMfcnt0 != state->imgmodevalDMfcnt0old) {
            state->imgmodevalDMfcnt0old = imgmodevalDMfcnt0;
            for(uint32_t mi=0; mi<NBmode; mi++) {
                state->mvalDMc[mi] = state->imgmodevalDMf.im->array.F[mi];
            }
        }

        // Update Gain/Mult/Limit arrays
        for(uint32_t mi=0; mi<NBmode; mi++) {
            state->imgmgain.im->array.F[mi] = state->imgmgainfact.im->array.F[mi] * (*loopgain_ptr);
            state->imgmmult.im->array.F[mi] = state->imgmmultfact.im->array.F[mi] * (*loopmult_ptr);
            state->imgmlimit.im->array.F[mi] = state->imgmlimitfact.im->array.F[mi] * (*looplimit_ptr);
        }
        
        // Modal Control
        for(uint32_t mi = 0; mi < NBmode; mi++) {
            float mvalWFS = imginWFS->array.F[mi];
            float dmval = state->imgmzeropoint.im->array.F[mi] - mvalWFS;
            dmval *= state->imgmgain.im->array.F[mi];
            state->mvalDMc[mi] = dmval + state->mvalDMc[mi] * state->imgmmult.im->array.F[mi];
            
            float limit = state->imgmlimit.im->array.F[mi];
            if(state->mvalDMc[mi] > limit) {
                state->mvalDMc[mi] = limit;
                state->mlimitcntarray[mi]++;
            }
            if(state->mvalDMc[mi] < -limit) {
                state->mvalDMc[mi] = -limit;
                state->mlimitcntarray[mi]++;
            }
            
            if((*auxDMmvalenable_ptr) == 1) {
                state->mvalout[mi] = state->mvalDMc[mi] + (auxDMfact * state->imgauxmDM.im->array.F[mi]);
            } else {
                state->mvalout[mi] = state->mvalDMc[mi];
            }
            state->mvaloutapply[mi] = state->mvalout[mi] + state->selfRMpokecmd[mi];
        }
        state->modal_limit_counter++;

        // Output to stream if PF not enabled
        if(*enablePF_ptr == 0) {
            memcpy(imgout->array.F,
                state->mvaloutapply,
                sizeof(float) * NBmode);
        }

        // Offload Loop
        if((*offload_ptr) == 1) {
            for(uint32_t mi=0; mi<NBmode; mi++) {
                state->imgoffloadmgain.im->array.F[mi] = state->imgoffloadmgainfact.im->array.F[mi] * (*offloadloopgain_ptr);
                state->imgoffloadmmult.im->array.F[mi] = state->imgoffloadmmultfact.im->array.F[mi] * (*offloadloopmult_ptr);
                state->imgoffloadmlimit.im->array.F[mi] = state->imgoffloadmlimitfact.im->array.F[mi] * (*offloadlooplimit_ptr);

                float val = state->imgmvaloffloadDM.im->array.F[mi];
                val += state->imgoffloadmgain.im->array.F[mi] * imgout->array.F[mi];
                val *= state->imgoffloadmmult.im->array.F[mi];
                float limit = state->imgoffloadmlimit.im->array.F[mi];
                 if(val > limit) val = limit;
                 if(val < -limit) val = -limit;
                 state->imgmvaloffloadDM.im->array.F[mi] = val;
            }
            processinfo_update_output_stream(processinfo,
                state->imgmvaloffloadDM.im,
                NULL);
        }

        // Compute OL
        if((*compOL_ptr) == 1) {
            for(uint32_t mi=0; mi<NBmode; mi++) state->mvalDMbuff[state->DMtstep * NBmode + mi] = state->mvalDMc[mi];
            state->DMtstep++;
            if(state->DMtstep == 10) state->DMtstep = 0;

            float latencytotalfr = (*latencyhardwfr_ptr) + (*latencysoftwfr_ptr);
            int latint = (int)latencytotalfr;
            float latfrac = latencytotalfr - latint;
            int DMtstep1 = state->DMtstep - latint;
            int DMtstep0 = DMtstep1 - 1;
            while(DMtstep1 < 0) DMtstep1 += 10;
            while(DMtstep0 < 0) DMtstep0 += 10;

            for(uint32_t mi=0; mi<NBmode; mi++) {
                float tmpmDMval = latfrac * state->mvalDMbuff[DMtstep0*NBmode + mi] + (1.0-latfrac)*state->mvalDMbuff[DMtstep1*NBmode + mi];
                state->mvalDMOL[mi] = tmpmDMval;
                float tmpmWFSval = imginWFS->array.F[mi];
                state->imgOLmval.im->array.F[mi] = (*psol_WFSfact_ptr) * tmpmWFSval - state->mvalDMOL[mi];
            }
            processinfo_update_output_stream(processinfo,
                state->imgOLmval.im,
                NULL);
        }
    }
}

/**
 * @brief Basic parameter validation.
 */
static void __attribute__((unused)) modalfilter_validate() {
    if (loopgain_ptr && *loopgain_ptr < 0) *loopgain_ptr = 0;
    if (loopmult_ptr && *loopmult_ptr < 0) *loopmult_ptr = 0;
    if (loopmult_ptr && *loopmult_ptr > 1.0) *loopmult_ptr = 1.0;
    if (looplimit_ptr && *looplimit_ptr < 0) *looplimit_ptr = 0;
}


/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".inmval", &inmval_ptr, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_PRIMARY_CLI_INPUT \
          | FPFLAG_STREAM_RUN_REQUIRED \
          | FPFLAG_CHECKSTREAM, \
      "input mode values from WFS") \
    X(".outmval", &outmval_ptr, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_PRIMARY_CLI_INPUT, \
      "output mode values to DM") \
    X(".loopgain", &loopgain_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop gain") \
    X(".loopmult", &loopmult_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop mult") \
    X(".looplimit", &looplimit_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop limit") \
    X(".loopZERO", &loopZERO_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop zero") \
    X(".AOloopindex", &AOloopindex_ptr, \
      FPTYPE_UINT64, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_PRIMARY_CLI_INPUT, \
      "AO loop index") \
    X(".loopON", &loopON_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop on/off (off=freeze)") \
    X(".loopNBstep", &loopNBstep_ptr, \
      FPTYPE_INT64, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_WRITERUN, \
      "loop nb steps (-1 = inf)") \
    X(".comp.OLmodes", &compOL_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "compute open loop modes") \
    X(".comp.WFSfact", &psol_WFSfact_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "ampl correction factor WFS") \
    X(".comp.latencyhardwfr", \
      &latencyhardwfr_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "hw DM-to-WFS latency [fr]") \
    X(".comp.latencysoftwfr", \
      &latencysoftwfr_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "sw latency [frame]") \
    X(".comp.tbuff", &comptbuff_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "compute telemetry buffers") \
    X(".comp.autolim", &autolim_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "automatic modal limits") \
    X(".comp.autolimprobegain", \
      &autolimprobegain_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "sigma measurement gain") \
    X(".comp.autolimsigmafact", \
      &autolimsigmafact_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "autolimit sigma clip factor") \
    X(".comp.tbuffsize", &tbuffsize_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "buffer time size") \
    X(".auxDMmval.enable", \
      &auxDMmvalenable_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "mixing aux DM mode vals") \
    X(".auxDMmval.mixfact", \
      &auxDMmvalmixfact_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "mixing mult factor") \
    X(".auxDMmval.modulate", \
      &auxDMmvalmodulate_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "modulate auxDM temporally") \
    X(".auxDMmval.modperiod", \
      &auxDMmvalmodperiod_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "auxDM mod period [frame]") \
    X(".PF.enable", &enablePF_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "enable predictive filter") \
    X(".PF.NBblock", &PF_NBblock_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "nb blocks to wait from") \
    X(".PF.maxwaitus", &PF_maxwaitus_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "max wait time blks [us]") \
    X(".PF.mixcoeff", &PFmixcoeff_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "mixing coeff") \
    X(".autoloop.enable", \
      &autoloopenable_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "autoloop self-test") \
    X(".autoloop.sleep", \
      &autoloopsleep_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "loop sleep time") \
    X(".selfRM.enable", &selfRMenable_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "start self RM measurement") \
    X(".selfRM.NBmode", &selfRMnbmode_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "number of mode poked") \
    X(".selfRM.pokeampl", \
      &selfRMpokeampl_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "poke amplitude") \
    X(".selfRM.zsize", &selfRMzsize_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "nb time steps recorded") \
    X(".selfRM.nbiter", &selfRMnbiter_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "nb iterations averaged") \
    X(".selfRM.nbsettle", \
      &selfRMnbsettlestep_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "nb loop iter to settle") \
    X(".testOL.enable", &testOL_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "OL reconstruction test") \
    X(".testOL.loop", &testOLloop_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "run inloop") \
    X(".testOL.updategain", \
      &testOLupdategain_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "update gain") \
    X(".testOL.mode", &testOLmode_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "mode index") \
    X(".testOL.ampl", &testOLampl_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "amplitude") \
    X(".testOL.nbsample", \
      &testOLnbsample_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "number of samples") \
    X(".testOL.cnt", &testOLcnt_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "samples count") \
    X(".offload.enable", &offload_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "offload output ON/OFF") \
    X(".offload.loopgain", \
      &offloadloopgain_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "offload loop gain") \
    X(".offload.loopmult", \
      &offloadloopmult_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "offload loop mult") \
    X(".offload.looplimit", \
      &offloadlooplimit_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "offload loop limit") \
    X(".rec.enable", &recburst_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "telemetry burt write") \
    X(".rec.mode", &recburst_mode_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "mode index") \
    X(".rec.nbstep", \
      &recburst_nbsample_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "number of steps recorded")


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

FPS_V2_SECTION5(FPS_PARAMS)


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static errno_t compute_function()
{
    IMGID imginWFS =
        imgid_make_from_name(inmval_ptr);
    resolveIMGID(&imginWFS, ERRMODE_ABORT,
        data.core.image, data.core.NB_MAX_IMAGE);

    uint32_t NBmode = imginWFS.md[0].size[0];

    IMGID imgout =
        stream_connect_create_2Df32(
            outmval_ptr, NBmode, 1);

    MFILT_STATE *state =
        modal_filter_init(NBmode);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    modal_filter_step(
        processinfo, data.core.fpsptr,
        imginWFS.im, imgout.im, state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    modal_filter_cleanup(state);
    return RETURN_SUCCESS;
}


/* ================================================================
 * 7.  MILK MODULE REGISTRATION
 * ============================================================= */

#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

errno_t
CLIADDCMD_AOloopControl__modalfilter()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif


/* ================================================================
 * 8.  STANDALONE ENTRY POINT
 * ============================================================= */

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
