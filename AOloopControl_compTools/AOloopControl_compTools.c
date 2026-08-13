// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_compTools.c
 * @brief   Adaptive Optics Control loop engine misc computation tools
 *
 * AO engine uses stream data structure
 *
 */

#define MODULE_SHORTNAME_DEFAULT "cacaoct"
#define MODULE_DESCRIPTION "AO loop control - computation tools"

// Application to which module belongs
#define MODULE_APPLICATION "cacao-cli"

#define _GNU_SOURCE

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "AOloopControl_compTools.h"

static errno_t init_module_CLI()
{
    return RETURN_SUCCESS;
}

MILK_MODULE(AOloopControl_compTools, init_module_CLI, NULL);
