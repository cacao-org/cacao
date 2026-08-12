// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_DM.c
 * @brief   DM control
 *
 * To be used for AOloopControl module
 *
 *
 *
 */


// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaodm"

// Module short description
#define MODULE_DESCRIPTION "AO loop Control DM operation"

// Application to which module belongs
#define MODULE_APPLICATION "cacao-cli"


#include <string.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl_DM/AOloopControl_DM.h"


#include "AOloopControl_DM_comb.h"

#include "DMturbulence.h"
#include "mk3Ddmgrid.h"
#include "pokerndmodes.h"


static errno_t init_module_CLI()
{

    CLIADDCMD_AOloopControl_DM__comb();

    CLIADDCMD_AOloopControl_DM__mk3Ddmgrid();

    CLIADDCMD_AOloopControl_DM__atmturbulence();

    CLIADDCMD_AOloopControl_DM__pokerndmodes();

    return RETURN_SUCCESS;
}

MILK_MODULE(AOloopControl_DM, init_module_CLI, NULL);
