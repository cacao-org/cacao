// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    zonalfilter.c
 * @brief   Apply zonal filtering in DM space
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "timeutils.h"
#include "zonalfilter.h"
#include "fps.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"
#include "ImageStreamIO/ImageStreamIO.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "zonalfilter",
    .cmdkey      = "zonalfilter",
    .description = "zonal filtering",
    .description_long =
        "Apply zonal (per-actuator) filtering in a real-time AO control loop. Processes WFS signals directly in actuator space without modal decomposition."
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint64_t *AOloopindex = NULL;
static char     *inzval      = NULL;
static char     *outzval     = NULL;
static int64_t  *loopON      = NULL;
static int64_t  *loopNBstep  = NULL;
static int64_t  *loopZERO    = NULL;
static float    *loopgain    = NULL;
static float    *loopmult    = NULL;
static float    *looplimit   = NULL;

/* Internal state */
static float *zvalDMc = NULL;


/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, \
      FPTYPE_UINT64, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT, \
      "AO loop index") \
    X(".inzval", &inzval, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT \
          | FPFLAG_STREAM_RUN_REQUIRED \
          | FPFLAG_CHECKSTREAM, \
      "input DM zonal values") \
    X(".outzval", &outzval, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_CLI_INPUT, \
      "output DM zonal values") \
    X(".loopON", &loopON, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop on/off (off=freeze)") \
    X(".loopNBstep", &loopNBstep, \
      FPTYPE_INT64, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop nb steps (-1 = inf)") \
    X(".loopZERO", &loopZERO, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop zero") \
    X(".loopgain", &loopgain, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop gain (speed)") \
    X(".loopmult", &loopmult, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop mult (attenuation)") \
    X(".looplimit", &looplimit, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_WRITERUN, \
      "loop limit")


/* ================================================================
 * 4.  COMPUTATION LOGIC
 * ============================================================= */

static void zonal_filter_step(
    PROCESSINFO              *processinfo,
    FPS *fps __attribute__((unused)),
    IMAGE *imginDM,
    IMAGE *imgout,
    IMAGE *imgzgain,
    IMAGE *imgzgainfact,
    IMAGE *imgzmult,
    IMAGE *imgzmultfact,
    IMAGE *imgzzeropoint,
    IMAGE *imgzlimit,
    IMAGE *imgzlimitfact)
{
    uint32_t dmxysize =
        imginDM->md[0].size[0]
        * imginDM->md[0].size[1];

    if (loopZERO
        && ((*loopZERO) & FPFLAG_ONOFF))
    {
        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            zvalDMc[act] = 0.0;
        }
        memcpy(imgout->array.F, zvalDMc,
               sizeof(float) * dmxysize);
        processinfo_update_output_stream(
            processinfo, imgout, NULL);
        (*loopZERO) &= ~FPFLAG_ONOFF;
    }

    if ((*loopON) == 1)
    {
        if (*loopNBstep > 0)
        {
            (*loopNBstep)--;
        }
        if (*loopNBstep == 0)
        {
            *loopON = 0;
            (*loopON) &= ~FPFLAG_ONOFF;
            *loopNBstep = 1;
        }

        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            float zvalin =
                imginDM->array.F[act]
                - imgzzeropoint->array.F[act];
            zvalDMc[act] =
                (1.0 - imgzgain->array.F[act])
                    * zvalDMc[act]
                + imgzgain->array.F[act] * zvalin;
            zvalDMc[act] *=
                imgzmult->array.F[act];
            float limit =
                imgzlimit->array.F[act];
            if (zvalDMc[act] > limit)
                zvalDMc[act] = limit;
            if (zvalDMc[act] < -limit)
                zvalDMc[act] = -limit;
        }

        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            imgout->array.F[act] =
                zvalDMc[act]
                + imgzzeropoint->array.F[act];
        }
        ImageStreamIO_UpdateIm(imgout);

        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            imgzgain->array.F[act] =
                imgzgainfact->array.F[act]
                * (*loopgain);
        }
        processinfo_update_output_stream(
            processinfo, imgzgain, NULL);

        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            imgzmult->array.F[act] =
                imgzmultfact->array.F[act]
                * (*loopmult);
        }
        processinfo_update_output_stream(
            processinfo, imgzmult, NULL);

        for (uint32_t act = 0;
             act < dmxysize; act++)
        {
            imgzlimit->array.F[act] =
                imgzlimitfact->array.F[act]
                * (*looplimit);
        }
        processinfo_update_output_stream(
            processinfo, imgzlimit, NULL);
    }
}


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

FPS_V2_SECTION5(FPS_PARAMS)


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static errno_t compute_function()
{
    IMGID imginDM =
        imgid_make_from_name(inzval);
    resolveIMGID(&imginDM, ERRMODE_WARN,
        dcimg, dcnimg);
        if (imginDM.ID == -1) return RETURN_FAILURE;

    uint32_t dmxsize = imginDM.md->size[0];
    uint32_t dmysize = imginDM.md->size[1];
    uint32_t dmxysize = dmxsize * dmysize;

    zvalDMc = (float *)
        calloc(dmxysize, sizeof(float));

    IMGID imgout =
        stream_connect_create_2Df32(
            outzval, dmxsize, dmysize);

    char name[STRINGMAXLEN_STREAMNAME];

    WRITE_IMAGENAME(name, "aol%lu_zgain",
        *AOloopindex);
    IMGID imgzgain =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zgainfact",
        *AOloopindex);
    IMGID imgzgainfact =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zmult",
        *AOloopindex);
    IMGID imgzmult =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zmultfact",
        *AOloopindex);
    IMGID imgzmultfact =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zzeropoint",
        *AOloopindex);
    IMGID imgzzeropoint =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zlimit",
        *AOloopindex);
    IMGID imgzlimit =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    WRITE_IMAGENAME(name, "aol%lu_zlimitfact",
        *AOloopindex);
    IMGID imgzlimitfact =
        stream_connect_create_2Df32(
            name, dmxsize, dmysize);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    zonal_filter_step(
        processinfo, milk_data.fpsptr,
        imginDM.im, imgout.im,
        imgzgain.im, imgzgainfact.im,
        imgzmult.im, imgzmultfact.im,
        imgzzeropoint.im,
        imgzlimit.im, imgzlimitfact.im);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(zvalDMc);
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
CLIADDCMD_AOloopControl__zonalfilter()
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
