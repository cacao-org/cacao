/**
 * @file    AOloopControl_PredictiveControl.c
 * @brief   AO predictive control
 *
 * Uses FPS V2 framework.
 */

#define MODULE_SHORTNAME_DEFAULT "cacaopc"
#define MODULE_DESCRIPTION \
    "AO loop control predictive control"
#define MODULE_APPLICATION "cacao-cli"

#define _GNU_SOURCE

#include "CLIcore/CLIcore.h"
#include "fps.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"

INIT_MODULE_LIB(AOloopControl_PredictiveControl)


/* =============================================================
 *  Parameters
 * ============================================================ */

static long long p_loop     = 0;
static long long p_pfblock  = 2;
static long long p_pfstart  = 0;
static long long p_pfend    = 0;
static long long p_nbbuff   = 0;
static char p_modecoeff[
    FUNCTION_PARAMETER_STRMAXLEN]
    = "coeffim";
static long long p_modeout  = 23;
static double    p_delayfr  = 2.4;
static long long p_filtsize = 20;
static char p_filtname[
    FUNCTION_PARAMETER_STRMAXLEN]
    = "filt23";
static char p_pfname[
    FUNCTION_PARAMETER_STRMAXLEN]
    = "outPFb0";
static double    p_decaycoeff = 0.5;


/* =============================================================
 *  CMD 1: aolPFwatchin (5 args, primary)
 * ============================================================ */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "aolPFwatchin",
    .cmdkey      = "aolPFwatchin",
    .description =
        "watch telemetry for PF input"
};

#define FPS_PARAMS(X) \
    X(".loop", &p_loop, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "loop number") \
    X(".pfblock", &p_pfblock, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "PF block number") \
    X(".pfstart", &p_pfstart, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "PF block start") \
    X(".pfend", &p_pfend, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "PF block end") \
    X(".nbbuff", &p_nbbuff, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "number of buffers")

static FPS_CLI_BINDING my_bindings[] = {
    FPS_PARAMS(FPS_X_BINDING)
};
static const int nb_bindings =
    sizeof(my_bindings) /
    sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

static CLICMDDATA CLIcmddata = {
    "", "", CLICMD_FIELDS_DEFAULTS
};
static CMDSETTINGS cms = {0};

static __attribute__((constructor))
void init_cms(void)
{
    strncpy(CLIcmddata.key,
            FPS_app_info.cmdkey,
            sizeof(CLIcmddata.key) - 1);
    strncpy(CLIcmddata.description,
            FPS_app_info.description,
            sizeof(CLIcmddata.description)
            - 1);
    if (CLIcmddata.cmdsettings == NULL) {
        CLIcmddata.cmdsettings = &cms;
    }
}

static errno_t compute_function()
{
    AOloopControl_PredictiveControl_builPFloop_WatchInput(
        p_loop, p_pfblock,
        p_pfstart, p_pfend,
        p_nbbuff);
    return RETURN_SUCCESS;
}


/* =============================================================
 *  CMD 2: aolmappfilt (3 args)
 * ============================================================ */

static FPS_APP_INFO FPS_app_info_map = {
    .fps_name    = "aolmappfilt",
    .cmdkey      = "aolmappfilt",
    .description =
        "map/search predictive filter"
};

#define FPS_PARAMS_MAP(X) \
    X(".modecoeff", p_modecoeff, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "input coefficients") \
    X(".modeout", &p_modeout, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "mode number") \
    X(".delayfr", &p_delayfr, \
      FPTYPE_FLOAT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "delay in frames")

static CLICMDDATA CLIcmddata_map = {
    "", "", CLICMD_FIELDS_NOPARAM
};
static CMDSETTINGS cms_map = {0};

static __attribute__((constructor))
void init_cms_map(void)
{
    strncpy(CLIcmddata_map.key,
            FPS_app_info_map.cmdkey,
            sizeof(CLIcmddata_map.key)
            - 1);
    strncpy(CLIcmddata_map.description,
            FPS_app_info_map.description,
            sizeof(
                CLIcmddata_map
                .description) - 1);
    if (CLIcmddata_map.cmdsettings
        == NULL) {
        CLIcmddata_map.cmdsettings =
            &cms_map;
    }
}

static errno_t compute_map()
{
    AOloopControl_PredictiveControl_mapPredictiveFilter(
        p_modecoeff, p_modeout,
        p_delayfr);
    return RETURN_SUCCESS;
}


/* =============================================================
 *  CMD 3: aolmkpfilt (5 args)
 * ============================================================ */

static FPS_APP_INFO FPS_app_info_mk = {
    .fps_name    = "aolmkpfilt",
    .cmdkey      = "aolmkpfilt",
    .description =
        "test predictive filter"
};

#define FPS_PARAMS_MK(X) \
    X(".trname", p_modecoeff, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "trace image") \
    X(".modeout", &p_modeout, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "mode number") \
    X(".delayfr", &p_delayfr, \
      FPTYPE_FLOAT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "delay in frames") \
    X(".filtsize", &p_filtsize, \
      FPTYPE_INT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "filter size") \
    X(".filtname", p_filtname, \
      FPTYPE_STRING, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "output filter name")

static CLICMDDATA CLIcmddata_mk = {
    "", "", CLICMD_FIELDS_NOPARAM
};
static CMDSETTINGS cms_mk = {0};

static __attribute__((constructor))
void init_cms_mk(void)
{
    strncpy(CLIcmddata_mk.key,
            FPS_app_info_mk.cmdkey,
            sizeof(CLIcmddata_mk.key)
            - 1);
    strncpy(CLIcmddata_mk.description,
            FPS_app_info_mk.description,
            sizeof(
                CLIcmddata_mk
                .description) - 1);
    if (CLIcmddata_mk.cmdsettings
        == NULL) {
        CLIcmddata_mk.cmdsettings =
            &cms_mk;
    }
}

static errno_t compute_mk()
{
    AOloopControl_PredictiveControl_testPredictiveFilter(
        p_modecoeff, p_modeout,
        p_delayfr, p_filtsize,
        p_filtname, 1e-10);
    return RETURN_SUCCESS;
}


/* =============================================================
 *  CMD 4: aolpfsetave (2 args)
 * ============================================================ */

static FPS_APP_INFO FPS_app_info_ave = {
    .fps_name    = "aolpfsetave",
    .cmdkey      = "aolpfsetave",
    .description =
        "set PF to integrator"
};

#define FPS_PARAMS_AVE(X) \
    X(".pfname", p_pfname, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "predictive filter") \
    X(".decaycoeff", &p_decaycoeff, \
      FPTYPE_FLOAT64, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "decay coefficient")

static CLICMDDATA CLIcmddata_ave = {
    "", "", CLICMD_FIELDS_NOPARAM
};
static CMDSETTINGS cms_ave = {0};

static __attribute__((constructor))
void init_cms_ave(void)
{
    strncpy(CLIcmddata_ave.key,
            FPS_app_info_ave.cmdkey,
            sizeof(CLIcmddata_ave.key)
            - 1);
    strncpy(CLIcmddata_ave.description,
            FPS_app_info_ave.description,
            sizeof(
                CLIcmddata_ave
                .description) - 1);
    if (CLIcmddata_ave.cmdsettings
        == NULL) {
        CLIcmddata_ave.cmdsettings =
            &cms_ave;
    }
}

static errno_t compute_ave()
{
    AOloopControl_PredictiveControl_setPFsimpleAve(
        p_pfname, p_decaycoeff);
    return RETURN_SUCCESS;
}


/* =============================================================
 *  REGISTRATION
 * ============================================================ */

static errno_t CLIfunction_main(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

static FPS_CLI_BINDING bindings_map[] = {
    FPS_PARAMS_MAP(FPS_X_BINDING)
};
static CLICMDARGDEF farg_map[] = {
    FPS_PARAMS_MAP(FPS_X_FARG)
};

static errno_t CLIfunction_map(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info_map,
        farg_map, &CLIcmddata_map,
        bindings_map,
        sizeof(bindings_map) /
        sizeof(FPS_CLI_BINDING),
        compute_map);
}

static FPS_CLI_BINDING bindings_mk[] = {
    FPS_PARAMS_MK(FPS_X_BINDING)
};
static CLICMDARGDEF farg_mk[] = {
    FPS_PARAMS_MK(FPS_X_FARG)
};

static errno_t CLIfunction_mk(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info_mk,
        farg_mk, &CLIcmddata_mk,
        bindings_mk,
        sizeof(bindings_mk) /
        sizeof(FPS_CLI_BINDING),
        compute_mk);
}

static FPS_CLI_BINDING bindings_ave[] = {
    FPS_PARAMS_AVE(FPS_X_BINDING)
};
static CLICMDARGDEF farg_ave[] = {
    FPS_PARAMS_AVE(FPS_X_FARG)
};

static errno_t CLIfunction_ave(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info_ave,
        farg_ave, &CLIcmddata_ave,
        bindings_ave,
        sizeof(bindings_ave) /
        sizeof(FPS_CLI_BINDING),
        compute_ave);
}

static errno_t init_module_CLI()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);
    safe_fps_fill_farg_examples(
        farg_map, bindings_map,
        sizeof(bindings_map) /
        sizeof(FPS_CLI_BINDING));
    safe_fps_fill_farg_examples(
        farg_mk, bindings_mk,
        sizeof(bindings_mk) /
        sizeof(FPS_CLI_BINDING));
    safe_fps_fill_farg_examples(
        farg_ave, bindings_ave,
        sizeof(bindings_ave) /
        sizeof(FPS_CLI_BINDING));

    {
        int cmdi = RegisterCLIcmd(
            CLIcmddata,
            CLIfunction_main);
        CLIcmddata.cmdsettings =
            &data.cmd[cmdi].cmdsettings;
    }
    {
        int cmdi = RegisterCLIcmd(
            CLIcmddata_map,
            CLIfunction_map);
        CLIcmddata_map.cmdsettings =
            &data.cmd[cmdi].cmdsettings;
    }
    {
        int cmdi = RegisterCLIcmd(
            CLIcmddata_mk,
            CLIfunction_mk);
        CLIcmddata_mk.cmdsettings =
            &data.cmd[cmdi].cmdsettings;
    }
    {
        int cmdi = RegisterCLIcmd(
            CLIcmddata_ave,
            CLIfunction_ave);
        CLIcmddata_ave.cmdsettings =
            &data.cmd[cmdi].cmdsettings;
    }

    return RETURN_SUCCESS;
}
