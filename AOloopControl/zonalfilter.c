#include "ImageStreamIO/ImageStruct.h"
/**
 * @file    zonalfilter.c
 * @brief   Apply zonal filtering in DM space
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "CommandLineInterface/CLIcore.h"
#include "timeutils.h"
#include "zonalfilter.h"
#include "fps.h"
#include "processinfo.h"
#include "ImageStreamIO.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

// Global variables defined in header
uint64_t *AOloopindex = NULL;
char     *inzval      = NULL;
char     *outzval     = NULL;
int64_t  *loopON      = NULL;
int64_t  *loopNBstep  = NULL;
int64_t  *loopZERO    = NULL;
float    *loopgain    = NULL;
float    *loopmult    = NULL;
float    *looplimit   = NULL;

static uint64_t fpi_inzval;
static uint64_t fpi_outzval;
static uint64_t fpi_loopON;
static uint64_t fpi_loopNBstep;
static uint64_t fpi_loopZERO;
static uint64_t fpi_loopgain;
static uint64_t fpi_loopmult;
static uint64_t fpi_looplimit;

// Internal state
static float *zvalDMc = NULL;

static errno_t help_function()
{
    if (data.fpsptr && data.fpsptr->md) printf("%s\n", data.fpsptr->md->helptext);
    else {
        printf("Zonal filtering\n");
        printf("Main input/output streams :\n [STREAM]   <.inzval>    input zonal values\n [STREAM]   <.outzval>   output zonal values\n");
    }
    return RETURN_SUCCESS;
}

// Optional custom configuration setup.
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_inzval].fpflag |= FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_loopON].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopZERO].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopNBstep].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_looplimit].fpflag |= FPFLAG_WRITERUN;
    }
    return RETURN_SUCCESS;
}

static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static void zonal_filter_step(PROCESSINFO *processinfo, FUNCTION_PARAMETER_STRUCT *fps, IMAGE *imginDM, IMAGE *imgout, IMAGE *imgzgain, IMAGE *imgzgainfact, IMAGE *imgzmult, IMAGE *imgzmultfact, IMAGE *imgzzeropoint, IMAGE *imgzlimit, IMAGE *imgzlimitfact)
{
    uint32_t dmxysize = imginDM->md[0].size[0] * imginDM->md[0].size[1];
    if(fps && (fps->parray[fpi_loopZERO].fpflag & FPFLAG_ONOFF)) {
        for(uint32_t act = 0; act < dmxysize; act++) zvalDMc[act] = 0.0;
        memcpy(imgout->array.F, zvalDMc, sizeof(float) * dmxysize); processinfo_update_output_stream(processinfo, imgout, NULL);
        fps->parray[fpi_loopZERO].fpflag &= ~FPFLAG_ONOFF;
    }
    if((*loopON) == 1) {
        if(*loopNBstep > 0) { (*loopNBstep)--; if(fps) fps->parray[fpi_loopNBstep].val.i64[0] = *loopNBstep; }
        if(*loopNBstep == 0) { *loopON = 0; if(fps) fps->parray[fpi_loopON].fpflag &= ~FPFLAG_ONOFF; *loopNBstep = 1; }
        for(uint32_t act = 0; act < dmxysize; act++) {
            float zvalin = imginDM->array.F[act] - imgzzeropoint->array.F[act];
            zvalDMc[act] = (1.0 - imgzgain->array.F[act]) * zvalDMc[act] + imgzgain->array.F[act] * zvalin;
            zvalDMc[act] *= imgzmult->array.F[act];
            float limit = imgzlimit->array.F[act];
            if(zvalDMc[act] > limit) zvalDMc[act] = limit;
            if(zvalDMc[act] < -limit) zvalDMc[act] = -limit;
        }
        for(uint32_t act = 0; act < dmxysize; act++) imgout->array.F[act] = zvalDMc[act] + imgzzeropoint->array.F[act];
        ImageStreamIO_UpdateIm(imgout);
        for(uint32_t act = 0; act < dmxysize; act++) imgzgain->array.F[act] = imgzgainfact->array.F[act] * (*loopgain);
        processinfo_update_output_stream(processinfo, imgzgain, NULL);
        for(uint32_t act = 0; act < dmxysize; act++) imgzmult->array.F[act] = imgzmultfact->array.F[act] * (*loopmult);
        processinfo_update_output_stream(processinfo, imgzmult, NULL);
        for(uint32_t act = 0; act < dmxysize; act++) imgzlimit->array.F[act] = imgzlimitfact->array.F[act] * (*looplimit);
        processinfo_update_output_stream(processinfo, imgzlimit, NULL);
    }
}

#ifndef FPS_STANDALONE
static CLICMDARGDEF farg[] = {
    { CLIARG_UINT64, ".AOloopindex", "AO loop index", "0", CLIARG_VISIBLE_DEFAULT, (void **) &AOloopindex, NULL },
    { CLIARG_STREAM, ".inzval", "input DM zonal values", "aol0_actvalDM", CLIARG_VISIBLE_DEFAULT, (void **) &inzval, (long*) &fpi_inzval },
    { CLIARG_STREAM, ".outzval", "output DM zonal values", "aol0_actvalDMf", CLIARG_VISIBLE_DEFAULT, (void **) &outzval, (long*) &fpi_outzval },
    { CLIARG_ONOFF, ".loopON", "loop on/off (off=freeze)", "ON", CLIARG_HIDDEN_DEFAULT, (void **) &loopON, (long*) &fpi_loopON },
    { CLIARG_INT64, ".loopNBstep", "loop nb steps (-1 = inf)", "-1", CLIARG_HIDDEN_DEFAULT, (void **) &loopNBstep, (long*) &fpi_loopNBstep },
    { CLIARG_ONOFF, ".loopZERO", "loop zero", "OFF", CLIARG_HIDDEN_DEFAULT, (void **) &loopZERO, (long*) &fpi_loopZERO },
    { CLIARG_FLOAT32, ".loopgain", "loop gain (speed)", "0.01", CLIARG_HIDDEN_DEFAULT, (void **) &loopgain, (long*) &fpi_loopgain },
    { CLIARG_FLOAT32, ".loopmult", "loop mult (attenuation)", "0.95", CLIARG_HIDDEN_DEFAULT, (void **) &loopmult, (long*) &fpi_loopmult },
    { CLIARG_FLOAT32, ".looplimit", "loop limit", "1.0", CLIARG_HIDDEN_DEFAULT, (void **) &looplimit, (long*) &fpi_looplimit }
};
static CLICMDDATA CLIcmddata = { "zonalfilter", "zonal filtering", CLICMD_FIELDS_DEFAULTS };

static errno_t compute_function()
{
    IMGID imginDM = imgid_make_from_name(inzval); resolveIMGID(&imginDM, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
    uint32_t dmxsize = imginDM.md->size[0], dmysize = imginDM.md->size[1], dmxysize = dmxsize * dmysize;
    zvalDMc = (float *) calloc(dmxysize, sizeof(float));
    IMGID imgout = stream_connect_create_2Df32(outzval, dmxsize, dmysize);
    char name[STRINGMAXLEN_STREAMNAME];
    WRITE_IMAGENAME(name, "aol%lu_zgain", *AOloopindex); IMGID imgzgain = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zgainfact", *AOloopindex); IMGID imgzgainfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zmult", *AOloopindex); IMGID imgzmult = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zmultfact", *AOloopindex); IMGID imgzmultfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zzeropoint", *AOloopindex); IMGID imgzzeropoint = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zlimit", *AOloopindex); IMGID imgzlimit = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zlimitfact", *AOloopindex); IMGID imgzlimitfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    zonal_filter_step(processinfo, data.fpsptr, imginDM.im, imgout.im, imgzgain.im, imgzgainfact.im, imgzmult.im, imgzmultfact.im, imgzzeropoint.im, imgzlimit.im, imgzlimitfact.im);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END
    free(zvalDMc); return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions
errno_t CLIADDCMD_AOloopControl__zonalfilter() { CLIcmddata.FPS_customCONFsetup = customCONFsetup; CLIcmddata.FPS_customCONFcheck = customCONFcheck; INSERT_STD_CLIREGISTERFUNC return RETURN_SUCCESS; }
#endif

#ifdef FPS_STANDALONE
int FPSINIT_zonalfilter(const char *fps_name, const char *keywords, const char *description) {
    FUNCTION_PARAMETER_STRUCT fps; FPS_INIT_STD_PREAMBLE(fps, fps_name, keywords, description, ZONALFILTER_HELPTEXT); FPS_INIT_PROCINFO_DEFAULTS(fps, "aol0_actvalDM", 1);
#define X_FPS_INIT(cli_type, fps_type, c_type, key, descr, def_str, def_val, ptr_addr, val_expr, cli_flags) { c_type val = def_val; function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val_expr, NULL); }
    ZONALFILTER_PARAMS(X_FPS_INIT)
#undef X_FPS_INIT
    fps_add_processinfo_entries(&fps); function_parameter_FPCONFexit(&fps); return 0;
}
int FPSCONF_zonalfilter(const char *fps_name, int loop) { FPS_CONF_STD_BODY(fps_name, loop, { AOloopindex = functionparameter_GetParamPtr_UINT64(&fps, ".AOloopindex"); inzval = functionparameter_GetParamPtr_STRING(&fps, ".inzval"); outzval = functionparameter_GetParamPtr_STRING(&fps, ".outzval"); loopON = functionparameter_GetParamPtr_INT64(&fps, ".loopON"); loopNBstep = functionparameter_GetParamPtr_INT64(&fps, ".loopNBstep"); loopZERO = functionparameter_GetParamPtr_INT64(&fps, ".loopZERO"); loopgain = functionparameter_GetParamPtr_FLOAT32(&fps, ".loopgain"); loopmult = functionparameter_GetParamPtr_FLOAT32(&fps, ".loopmult"); looplimit = functionparameter_GetParamPtr_FLOAT32(&fps, ".looplimit"); }, { }); return 0; }
FPS_MAKE_STANDALONE_CONFSTOP(zonalfilter)
FPS_MAKE_STANDALONE_RUNSTOP(zonalfilter)
int FPSRUN_zonalfilter(const char *fps_name) {
    FUNCTION_PARAMETER_STRUCT fps; FPS_RUN_STD_PREAMBLE(fps_name, fps, { AOloopindex = functionparameter_GetParamPtr_UINT64(&fps, ".AOloopindex"); inzval = functionparameter_GetParamPtr_STRING(&fps, ".inzval"); outzval = functionparameter_GetParamPtr_STRING(&fps, ".outzval"); loopON = functionparameter_GetParamPtr_INT64(&fps, ".loopON"); loopNBstep = functionparameter_GetParamPtr_INT64(&fps, ".loopNBstep"); loopZERO = functionparameter_GetParamPtr_INT64(&fps, ".loopZERO"); loopgain = functionparameter_GetParamPtr_FLOAT32(&fps, ".loopgain"); loopmult = functionparameter_GetParamPtr_FLOAT32(&fps, ".loopmult"); looplimit = functionparameter_GetParamPtr_FLOAT32(&fps, ".looplimit"); });
    IMGID imginDM = imgid_make_from_name(inzval); resolveIMGID(&imginDM, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
    uint32_t dmxsize = imginDM.md->size[0], dmysize = imginDM.md->size[1], dmxysize = dmxsize * dmysize; zvalDMc = (float *) calloc(dmxysize, sizeof(float));
    IMGID imgout = stream_connect_create_2Df32(outzval, dmxsize, dmysize);
    char name[STRINGMAXLEN_STREAMNAME];
    WRITE_IMAGENAME(name, "aol%lu_zgain", *AOloopindex); IMGID imgzgain = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zgainfact", *AOloopindex); IMGID imgzgainfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zmult", *AOloopindex); IMGID imgzmult = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zmultfact", *AOloopindex); IMGID imgzmultfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zzeropoint", *AOloopindex); IMGID imgzzeropoint = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zlimit", *AOloopindex); IMGID imgzlimit = stream_connect_create_2Df32(name, dmxsize, dmysize);
    WRITE_IMAGENAME(name, "aol%lu_zlimitfact", *AOloopindex); IMGID imgzlimitfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
    PROCESSINFO *pinfo = processinfo_setup((char*)fps_name, "Run", "Looping", __FUNCTION__, __FILE__, __LINE__);
    processinfo_waitoninputstream_init(pinfo, imginDM.im, PROCESSINFO_TRIGGERMODE_SEMAPHORE, -1);
    fps_to_processinfo(&fps, pinfo); processinfo_loopstart(pinfo);
    while(processinfo_loopstep(pinfo)) { processinfo_waitoninputstream(pinfo); if (pinfo->triggerstatus == PROCESSINFO_TRIGGERSTATUS_TIMEDOUT) continue;
        processinfo_exec_start(pinfo); zonal_filter_step(pinfo, &fps, imginDM.im, imgout.im, imgzgain.im, imgzgainfact.im, imgzmult.im, imgzmultfact.im, imgzzeropoint.im, imgzlimit.im, imgzlimitfact.im); processinfo_exec_end(pinfo);
    }
    free(zvalDMc); processinfo_cleanExit(pinfo); function_parameter_struct_disconnect(&fps); return 0;
}
FPS_MAIN_STANDALONE("zonalfilter", zonalfilter, ZONALFILTER_HELPTEXT, ZONALFILTER_PARAMS)
#endif
