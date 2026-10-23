// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_PredictiveControl.c
 * @brief   AO predictive control
 *
 * Uses FPS V2 framework.
 */

#define MODULE_SHORTNAME_DEFAULT "cacaopc"
#define MODULE_DESCRIPTION "AO loop control predictive control"

#define _GNU_SOURCE

#include "milk_runnable.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"


/* =============================================================
 *  Parameters
 * ============================================================ */

static long long p_loop                                    = 0;
static long long p_pfblock                                 = 2;
static long long p_pfstart                                 = 0;
static long long p_pfend                                   = 0;
static long long p_nbbuff                                  = 0;
static char      p_modecoeff[FUNCTION_PARAMETER_STRMAXLEN] = "coeffim";
static long long p_modeout                                 = 23;
static double    p_delayfr                                 = 2.4;
static long long p_filtsize                                = 20;
static char      p_filtname[FUNCTION_PARAMETER_STRMAXLEN]  = "filt23";
static char      p_pfname[FUNCTION_PARAMETER_STRMAXLEN]    = "outPFb0";
static double    p_decaycoeff                              = 0.5;


/* =============================================================
 *  CMD 1: aolPFwatchin (5 args, primary)
 * ============================================================ */

#define FPS_PARAMS(X)                                                                   \
    X(".loop", &p_loop, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "loop number")           \
    X(".pfblock", &p_pfblock, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "PF block number") \
    X(".pfstart", &p_pfstart, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "PF block start")  \
    X(".pfend", &p_pfend, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "PF block end")        \
    X(".nbbuff", &p_nbbuff, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "number of buffers")

MILK_PARAMS_DEFINE(pfwatchin, FPS_PARAMS);

static errno_t pfwatchin_compute(const MILK_RUNNABLE *self)
{
    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    AOloopControl_PredictiveControl_builPFloop_WatchInput(p_loop, p_pfblock, p_pfstart, p_pfend,
                                                          p_nbbuff);
    MILK_PROCINFO_END(self)
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_pfwatchin = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "aolPFwatchin",
                 .cmdkey           = "aolPFwatchin",
                 .description      = "watch telemetry for PF input",
                 .description_long = "Monitor AO telemetry and prepare input buffers for the "
                                     "predictive filter engine. Watches WFS and DM streams and "
                                     "triggers filter updates." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(pfwatchin),
    .compute = pfwatchin_compute,
};


/* =============================================================
 *  CMD 2: aolmappfilt (3 args)
 * ============================================================ */

#define FPS_PARAMS_MAP(X)                                                                          \
    X(".modecoeff", p_modecoeff, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "input coefficients") \
    X(".modeout", &p_modeout, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "mode number")                \
    X(".delayfr", &p_delayfr, FPTYPE_FLOAT64, 1, FPFLAG_DEFAULT_INPUT, "delay in frames")

MILK_PARAMS_DEFINE(mappfilt, FPS_PARAMS_MAP);

static errno_t mappfilt_compute(const MILK_RUNNABLE *self)
{
    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    AOloopControl_PredictiveControl_mapPredictiveFilter(p_modecoeff, p_modeout, p_delayfr);
    MILK_PROCINFO_END(self)
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_mappfilt = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "aolmappfilt",
                 .cmdkey           = "aolmappfilt",
                 .description      = "map/search predictive filter",
                 .description_long = "Monitor AO telemetry and prepare input buffers for the "
                                     "predictive filter engine. Watches WFS and DM streams and "
                                     "triggers filter updates." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(mappfilt),
    .compute = mappfilt_compute,
};


/* =============================================================
 *  CMD 3: aolmkpfilt (5 args)
 * ============================================================ */

#define FPS_PARAMS_MK(X)                                                                  \
    X(".trname", p_modecoeff, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "trace image")  \
    X(".modeout", &p_modeout, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "mode number")       \
    X(".delayfr", &p_delayfr, FPTYPE_FLOAT64, 1, FPFLAG_DEFAULT_INPUT, "delay in frames") \
    X(".filtsize", &p_filtsize, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "filter size")     \
    X(".filtname", p_filtname, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "output filter name")

MILK_PARAMS_DEFINE(mkpfilt, FPS_PARAMS_MK);

static errno_t mkpfilt_compute(const MILK_RUNNABLE *self)
{
    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    AOloopControl_PredictiveControl_testPredictiveFilter(p_modecoeff, p_modeout, p_delayfr,
                                                         p_filtsize, p_filtname, 1e-10);
    MILK_PROCINFO_END(self)
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_mkpfilt = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "aolmkpfilt",
                 .cmdkey           = "aolmkpfilt",
                 .description      = "test predictive filter",
                 .description_long = "Monitor AO telemetry and prepare input buffers for the "
                                     "predictive filter engine. Watches WFS and DM streams and "
                                     "triggers filter updates." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(mkpfilt),
    .compute = mkpfilt_compute,
};


/* =============================================================
 *  CMD 4: aolpfsetave (2 args)
 * ============================================================ */

#define FPS_PARAMS_AVE(X)                                                                   \
    X(".pfname", p_pfname, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "predictive filter") \
    X(".decaycoeff", &p_decaycoeff, FPTYPE_FLOAT64, 1, FPFLAG_DEFAULT_INPUT, "decay coefficient")

MILK_PARAMS_DEFINE(pfsetave, FPS_PARAMS_AVE);

static errno_t pfsetave_compute(const MILK_RUNNABLE *self)
{
    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    AOloopControl_PredictiveControl_setPFsimpleAve(p_pfname, p_decaycoeff);
    MILK_PROCINFO_END(self)
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_PredictiveControl_pfsetave = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "aolpfsetave",
                 .cmdkey           = "aolpfsetave",
                 .description      = "set PF to integrator",
                 .description_long = "Monitor AO telemetry and prepare input buffers for the "
                                     "predictive filter engine. Watches WFS and DM streams and "
                                     "triggers filter updates." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(pfsetave),
    .compute = pfsetave_compute,
};


/* =============================================================
 *  REGISTRATION
 * ============================================================ */

static const MILK_RUNNABLE *const runnables[] = {
    &runnable_AOloopControl_PredictiveControl_pfwatchin,
    &runnable_AOloopControl_PredictiveControl_mappfilt,
    &runnable_AOloopControl_PredictiveControl_mkpfilt,
    &runnable_AOloopControl_PredictiveControl_pfsetave,
};

MILK_MODULE_RUNNABLES(AOloopControl_PredictiveControl, NULL, runnables);
