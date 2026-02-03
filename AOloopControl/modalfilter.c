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

#include "CommandLineInterface/CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "timeutils.h"

#include "fps.h"
#include "processinfo.h"
#include "processtools.h"

#include "modalfilter.h"

/* =============================================================================================== */
/* PARAMETERS DEFINITION                                                                           */
/* =============================================================================================== */

#define MFILT_PARAMS(X)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_STREAMNAME, char*, ".inmval", "input mode values from WFS", "aol0_modevalWFS", inmval_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_STREAMNAME, char*, ".outmval", "output mode values to DM", "aol0_modevalDM", outmval_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_FLOAT32, float*, ".loopgain", "loop gain", "0.01", loopgain_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_FLOAT32, float*, ".loopmult", "loop mult", "0.95", loopmult_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_FLOAT32, float*, ".looplimit", "loop limit", "1.0", looplimit_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".loopZERO", "loop zero", "OFF", loopZERO_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_VISIBLE_DEFAULT, FPTYPE_UINT64, uint64_t*, ".AOloopindex", "AO loop index", "0", AOloopindex_ptr, GetParamPtr_UINT64, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".loopON", "loop on/off (off=freeze)", "ON", loopON_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_INT64, int64_t*, ".loopNBstep", "loop nb steps (-1 = inf)", "-1", loopNBstep_ptr, GetParamPtr_INT64, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".comp.OLmodes", "compute open loop modes", "OFF", compOL_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".comp.WFSfact", "amplitude correction factor on WFS", "0.893", psol_WFSfact_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".comp.latencyhardwfr", "hardware DM to WFS latency [frame]", "1.7", latencyhardwfr_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".comp.latencysoftwfr", "software latency [frame]", "1.5", latencysoftwfr_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".comp.tbuff", "compute telemetry buffer(s)", "OFF", comptbuff_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".comp.autolim", "automatic modal limits", "OFF", autolim_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".comp.autolimprobegain", "sigma measurement gain", "0.1", autolimprobegain_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".comp.autolimsigmafact", "autolimit sigma clipping factor", "2.0", autolimsigmafact_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".comp.tbuffsize", "buffer time size", "512", tbuffsize_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".auxDMmval.enable", "mixing aux DM mode vals", "OFF", auxDMmvalenable_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".auxDMmval.mixfact", "mixing multiplicative factor", "1.0", auxDMmvalmixfact_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".auxDMmval.modulate", "modulate auxDM temporally ?", "OFF", auxDMmvalmodulate_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".auxDMmval.modperiod", "auxDM modulation period [frame]", "20.0", auxDMmvalmodperiod_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".PF.enable", "enable predictive filter", "OFF", enablePF_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".PF.NBblock", "number of blocks to wait from", "0", PF_NBblock_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".PF.maxwaitus", "maximum wait time for blocks [us]", "500", PF_maxwaitus_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".PF.mixcoeff", "mixing coeff", "0.3", PFmixcoeff_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".autoloop.enable", "autoloop self-test", "OFF", autoloopenable_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".autoloop.sleep", "loop sleep time", "0.001", autoloopsleep_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".selfRM.enable", "Start self response matrix measurement", "OFF", selfRMenable_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".selfRM.NBmode", "number of mode poked", "32", selfRMnbmode_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".selfRM.pokeampl", "poke amplitude", "0.01", selfRMpokeampl_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".selfRM.zsize", "number of time steps recorded", "20", selfRMzsize_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".selfRM.nbiter", "number of iterations averaged", "8", selfRMnbiter_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".selfRM.nbsettle", "number of loop iteration to settle", "1", selfRMnbsettlestep_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".testOL.enable", "OL reconstruction test ON/OFF", "OFF", testOL_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".testOL.loop", "run inloop", "OFF", testOLloop_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".testOL.updategain", "update gain", "0.1", testOLupdategain_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".testOL.mode", "mode index", "0", testOLmode_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".testOL.ampl", "amplitude", "0.01", testOLampl_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".testOL.nbsample", "number of samples", "1000", testOLnbsample_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".testOL.cnt", "samples count", "0", testOLcnt_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".offload.enable", "offload output ON/OFF", "OFF", offload_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".offload.loopgain", "offload loop gain", "0.01", offloadloopgain_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".offload.loopmult", "offload loop mult", "0.95", offloadloopmult_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".offload.looplimit", "offload loop limit", "1.0", offloadlooplimit_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".rec.enable", "telemetry ASCII file burt write", "OFF", recburst_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".rec.mode", "mode index", "5", recburst_mode_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)     X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".rec.nbstep", "number of steps recorded", "1000", recburst_nbsample_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)

/* Global parameter pointers */
#define X_PTR_DECL(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...)     static c_type ptr_name = NULL;
MFILT_PARAMS(X_PTR_DECL)
#undef X_PTR_DECL

static uint64_t processinfo_change_cnt_local = 0;

static uint64_t fpi_inmval;
static uint64_t fpi_outmval;
static uint64_t fpi_loopZERO;
static uint64_t fpi_loopgain;
static uint64_t fpi_loopmult;
static uint64_t fpi_looplimit;

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
    state->imgcbuff_mvalPF = stream_connect_create_2Df32(name, PFcbuff_size, NBmode);
    
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
    state->imgtbuff_mvalDM = stream_connect_create_3Df32(name, NBmode, *tbuffsize_ptr, 2);
    WRITE_IMAGENAME(name, "aol%lu_modevalWFS_buff", *AOloopindex_ptr);
    state->imgtbuff_mvalWFS = stream_connect_create_3Df32(name, NBmode, *tbuffsize_ptr, 2);
    WRITE_IMAGENAME(name, "aol%lu_modevalOL_buff", *AOloopindex_ptr);
    state->imgtbuff_mvalOL = stream_connect_create_3Df32(name, NBmode, *tbuffsize_ptr, 2);
    
    // SelfRM
    WRITE_IMAGENAME(name, "aol%lu_mfiltselfRM", *AOloopindex_ptr);
    state->imgselfRM = stream_connect_create_3Df32(name, NBmode, NBmode, *selfRMzsize_ptr);
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
            auxDMfact *= sin(2.0 * M_PI * modpha);
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
            memcpy(imgout->array.F, state->mvaloutapply, sizeof(float) * NBmode);
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
            processinfo_update_output_stream(processinfo, state->imgmvaloffloadDM.im, NULL);
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
            processinfo_update_output_stream(processinfo, state->imgOLmval.im, NULL);
        }
    }
}

/**
 * @brief Basic parameter validation.
 */
static void modalfilter_validate() {
    if (loopgain_ptr && *loopgain_ptr < 0) *loopgain_ptr = 0;
    if (loopmult_ptr && *loopmult_ptr < 0) *loopmult_ptr = 0;
    if (loopmult_ptr && *loopmult_ptr > 1.0) *loopmult_ptr = 1.0;
    if (looplimit_ptr && *looplimit_ptr < 0) *looplimit_ptr = 0;
}

#ifndef FPS_STANDALONE

static CLICMDARGDEF farg[] = {
    { CLIARG_UINT64, ".AOloopindex", "AO loop index", "0", CLIARG_VISIBLE_DEFAULT, (void **) &AOloopindex_ptr, NULL },
    { CLIARG_STREAM, ".inmval", "input mode values from WFS", "aol0_modevalWFS", CLIARG_VISIBLE_DEFAULT, (void **) &inmval_ptr, (long*) &fpi_inmval },
    { CLIARG_STREAM, ".outmval", "output mode values to DM", "aol0_modevalDM", CLIARG_VISIBLE_DEFAULT, (void **) &outmval_ptr, (long*) &fpi_outmval },
    { CLIARG_FLOAT32, ".loopgain", "loop gain", "0.01", CLIARG_VISIBLE_DEFAULT, (void **) &loopgain_ptr, (long*) &fpi_loopgain },
    { CLIARG_FLOAT32, ".loopmult", "loop mult", "0.95", CLIARG_VISIBLE_DEFAULT, (void **) &loopmult_ptr, (long*) &fpi_loopmult },
    { CLIARG_FLOAT32, ".looplimit", "loop limit", "1.0", CLIARG_VISIBLE_DEFAULT, (void **) &looplimit_ptr, (long*) &fpi_looplimit },
    { CLIARG_ONOFF, ".loopZERO", "loop zero", "OFF", CLIARG_VISIBLE_DEFAULT, (void **) &loopZERO_ptr, (long*) &fpi_loopZERO }
};

static CLICMDDATA CLIcmddata = { 
    "modalfilter", "modal filtering", "", 
    sizeof(farg) / sizeof(CLICMDARGDEF), farg, 
    CLICMDFLAG_FPS, NULL, NULL, NULL 
};

static errno_t help_function() {
    printf("Modal gain for adaptive optics control\n");
    return RETURN_SUCCESS;
}

static errno_t compute_function()
{
    IMGID imginWFS = imgid_make_from_name(inmval_ptr); resolveIMGID(&imginWFS, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
    uint32_t NBmode = imginWFS.md[0].size[0];
    IMGID imgout = stream_connect_create_2Df32(outmval_ptr, NBmode, 1);
    
    MFILT_STATE *state = modal_filter_init(NBmode);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    modal_filter_step(processinfo, data.fpsptr, imginWFS.im, imgout.im, state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END
    
    modal_filter_cleanup(state);
    return RETURN_SUCCESS;
}

static errno_t customCONFsetup() {
    if(data.fpsptr != NULL) {
        data.fpsptr->parray[fpi_inmval].fpflag |= FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
    }
    return RETURN_SUCCESS;
}

static errno_t customCONFcheck() { return RETURN_SUCCESS; }

#define INSERT_STD_FPSCONFfunction_local                                           static errno_t FPSCONFfunction()                                               {                                                                                  FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);                               if (CLIcmddata.flags & CLICMDFLAG_PROCINFO)                                    {                                                                                  fps_add_processinfo_entries(&fps);                                         }                                                                              data.fpsptr = &fps;                                                            CMDargs_to_FPSparams_create(&fps);                                             if (CLIcmddata.FPS_customCONFsetup != NULL)                                    {                                                                                  CLIcmddata.FPS_customCONFsetup();                                          }                                                                              FPS_CONFLOOP_START                                                             if (CLIcmddata.FPS_customCONFcheck != NULL)                                        CLIcmddata.FPS_customCONFcheck();                                          FPS_CONFLOOP_END                                                               data.fpsptr = NULL;                                                            return RETURN_SUCCESS;                                                     }

INSERT_STD_FPSCONFfunction_local
INSERT_STD_FPSRUNfunction
INSERT_STD_FPSCLIfunction

errno_t CLIADDCMD_AOloopControl__modalfilter() {
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}

#endif

#ifdef FPS_STANDALONE

int FPSINIT_modalfilter(const char *fps_name, const char *keywords, const char *description) {
    FUNCTION_PARAMETER_STRUCT fps;
    FPS_INIT_STD_PREAMBLE(fps, fps_name, keywords, description, "Modal Filtering AO processing");
    FPS_INIT_PROCINFO_DEFAULTS(fps, "aol0_modevalWFS", 10);
#define X_FPS_INIT(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...)     {         if(fps_type == FPTYPE_FLOAT32) { float val = (float)atof(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); }         else if(fps_type == FPTYPE_UINT64) { uint64_t val = (uint64_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); }         else if(fps_type == FPTYPE_INT64) { int64_t val = (int64_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); }         else if(fps_type == FPTYPE_UINT32) { uint32_t val = (uint32_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); }         else if(fps_type == FPTYPE_STREAMNAME) { char val[FUNCTION_PARAMETER_STRMAXLEN]; strncpy(val, def_str, FUNCTION_PARAMETER_STRMAXLEN-1); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val, NULL); }         else { function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, NULL, NULL); }     }
    MFILT_PARAMS(X_FPS_INIT)
#undef X_FPS_INIT
    fps_add_processinfo_entries(&fps); function_parameter_FPCONFexit(&fps); return 0;
}

#define X_FPS_MAP(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...)             ptr_name = (c_type)functionparameter_##get_func(&fps, key);

int FPSCONF_modalfilter(const char *fps_name, int loop) {
    FPS_CONF_STD_BODY(fps_name, loop, { MFILT_PARAMS(X_FPS_MAP) }, { modalfilter_validate(); });
    return 0;
}
FPS_MAKE_STANDALONE_CONFSTOP(modalfilter)
FPS_MAKE_STANDALONE_RUNSTOP(modalfilter)
int FPSRUN_modalfilter(const char *fps_name) {
    FUNCTION_PARAMETER_STRUCT fps;
    FPS_RUN_STD_PREAMBLE(fps_name, fps, { MFILT_PARAMS(X_FPS_MAP) });
    IMAGE imginWFS;
    if (ImageStreamIO_read_sharedmem_image_toIMAGE(inmval_ptr, &imginWFS) != 0) return 1;
    uint32_t NBmode = imginWFS.md[0].size[0];
    IMAGE imgout;
    uint32_t dims[2] = {NBmode, 1};
    if (ImageStreamIO_createIm_gpu(&imgout, outmval_ptr, 2, dims, _DATATYPE_FLOAT, -1, 1, 10, 0, 0, 0) != 0) return 1;
    
    MFILT_STATE *state = modal_filter_init(NBmode);
    
    PROCESSINFO *pinfo;
    FPS_RUN_PROCESSINFO_SETUP(pinfo, fps_name, "Run", "Looping", &imginWFS, fps);
    while(processinfo_loopstep(pinfo)) {
        processinfo_waitoninputstream(pinfo);
        if (pinfo->triggerstatus == PROCESSINFO_TRIGGERSTATUS_TIMEDOUT) continue;
        processinfo_exec_start(pinfo);
        modal_filter_step(pinfo, &fps, &imginWFS, &imgout, state);
        processinfo_exec_end(pinfo);
        processinfo_update_output_stream(pinfo, &imgout, &imginWFS);
    }
    modal_filter_cleanup(state);
    processinfo_cleanExit(pinfo); function_parameter_struct_disconnect(&fps); return 0;
}
FPS_MAIN_STANDALONE("mfilt", modalfilter, "Modal Filtering AO processing", MFILT_PARAMS)
#endif
