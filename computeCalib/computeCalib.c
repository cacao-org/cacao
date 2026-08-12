// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib.c
 * @brief   AO loop compute calibration
 *
 * Uses FPS V2 framework.
 */

#define MODULE_SHORTNAME_DEFAULT "cacaocc"
#define MODULE_DESCRIPTION "AO loop control compute calibration"
#define MODULE_APPLICATION "cacao-cli"

#define _GNU_SOURCE

#include "CLIcore.h"
#include "fps.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "actmap_sample2D.h"
#include "compute_control_modes.h"
#include "compute_masksWFSDM.h"
#include "compute_straight_CM.h"
#include "generateRMWFS.h"
#include "computeHadamard.h"
#include "maskextrapolate.h"

#include "RM2zonal.h"

#include "computeCalib.h"


INIT_MODULE_LIB(AOloopControl_computeCalib)


/* =============================================================
 *  CMD 1: aolHaddec (4 args)
 * ============================================================ */

static char p_inrm[FUNCTION_PARAMETER_STRMAXLEN]   = "imRMh";
static char p_hmat[FUNCTION_PARAMETER_STRMAXLEN]   = "Hmat";
static char p_pixidx[FUNCTION_PARAMETER_STRMAXLEN] = "pixiind";
static char p_outrm[FUNCTION_PARAMETER_STRMAXLEN]  = "imRM";

static FPS_APP_INFO FPS_app_info = {
    .fps_name         = "aolHaddec",
    .cmdkey           = "aolHaddec",
    .description      = "decode Hadamard matrix",
    .description_long = "Decode Hadamard-encoded response matrix measurements. Applies the inverse "
                        "Hadamard transform to recover per-actuator responses."
};

#define FPS_PARAMS(X)                                                                          \
    X(".inrm", p_inrm, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "input RM")                 \
    X(".hmat", p_hmat, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "Hadamard matrix")          \
    X(".pixidx", p_pixidx, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "DM pixel index frame") \
    X(".outrm", p_outrm, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "output RM")

static FPS_CLI_BINDING my_bindings[] = { FPS_PARAMS(FPS_X_BINDING) };
static const int       nb_bindings   = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = { FPS_PARAMS(FPS_X_FARG) };

static CLICMDDATA CLIcmddata = { "", "", CLICMD_FIELDS_DEFAULTS };
FPS_CMDSETTINGS_INIT(main, CLIcmddata, FPS_app_info)

static errno_t compute_function()
{
    AOloopControl_computeCalib_Hadamard_decodeRM(p_inrm, p_hmat, p_pixidx, p_outrm);
    return RETURN_SUCCESS;
}

static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(&FPS_app_info, farg, &CLIcmddata, my_bindings, nb_bindings,
                                        compute_function);
}


/* =============================================================
 *  REGISTRATION
 * ============================================================ */

static errno_t init_module_CLI()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);
    {
        int cmdi               = RegisterCLIcmd(CLIcmddata, CLIfunction);
        CLIcmddata.cmdsettings = &data.cmd[cmdi].cmdsettings;
    }

    CLIADDCMD_cacao_computeCalib__compute_control_modes();

    CLIADDCMD_AOloopControl_computeCalib__compsCM();

    CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM();

    CLIADDCMD_AOloopControl_computeCalib__generateRMWFS();

    CLIADDCMD_AOloopControl_computeCalib__mkHadamard();

    CLIADDCMD_AOloopControl_computeCalib__maskextrapolate();

    CLIADDCMD_AOloopControl_computeCalib__RM2zonal();

    CLIADDCMD_AOloopControl_computeCalib__mkCMsvd();

    CLIADDCMD_AOloopControl_computeCalib__sample2D();

    return RETURN_SUCCESS;
}
