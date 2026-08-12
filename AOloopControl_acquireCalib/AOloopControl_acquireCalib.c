// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_acquireCalib.c
 * @brief   Adaptive Optics Control loop engine acquire calibration
 *
 * Acquire AO system calibration assuming linear response
 *
 *
 *
 */


// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaoac"

// Module short description
#define MODULE_DESCRIPTION "AO loop control acquire calibration"
// Application to which module belongs
#define MODULE_APPLICATION "cacao-cli"

#define _GNU_SOURCE


#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "computeCalib/computeCalib.h"

#include "acquireWFSlincalib.h"
#include "measure_linear_resp.h"


INIT_MODULE_LIB(AOloopControl_acquireCalib)


/** @name AOloopControl_IOtools functions */

static errno_t init_module_CLI()
{
    DEBUG_TRACE_FSTART();


    CLIADDCMD_milk_AOloopControl_acquireCalib__acquireWFSlincalib();
    CLIADDCMD_AOloopControl__measure_linear_resp();

    // add atexit functions here
    // atexit((void*) myfunc);

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}
