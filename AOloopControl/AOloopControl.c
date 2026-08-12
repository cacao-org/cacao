// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl.c
 * @brief   Adaptive Optics Control loop engine
 *
 * AO engine uses stream data structure
 *
 * # Files
 *
 * ## Main files
 *
 *
 *
 * @see
 * http://oguyon.github.io/AdaptiveOpticsControl/src/AOloopControl/doc/AOloopControl.html
 *
 * @defgroup AOloopControl_streams Image streams
 * @defgroup AOloopControl_AOLOOPCONTROL_CONF AOloopControl main data structure
 *
 */

#define MODULE_SHORTNAME_DEFAULT "cacao"
#define MODULE_DESCRIPTION "AO loop control"

#define _GNU_SOURCE


#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "modalfilter.h"
#include "modalfilter_test.h"

#include "modalCTRL_stats.h"
#include "modalstatsTUI.h"

#include "zonalfilter.h"


static errno_t init_module_CLI()
{
    CLIADDCMD_AOloopControl__modalfilter();
    CLIADDCMD_AOloopControl__modalfilter_test();
    CLIADDCMD_AOloopControl__modalCTRL_stats();
    CLIADDCMD_AOloopControl__modalstatsTUI();

    CLIADDCMD_AOloopControl__zonalfilter();

    return RETURN_SUCCESS;
}

MILK_MODULE(AOloopControl, init_module_CLI, NULL);
