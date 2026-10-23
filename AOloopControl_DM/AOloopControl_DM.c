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

#include "milk_runnable.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

// Module registry: CLI exposure = membership here.
static const MILK_RUNNABLE *const runnables[] = {
    &runnable_AOloopControl_DM_comb,
    &runnable_AOloopControl_DM_mk3Ddmgrid,
    &runnable_AOloopControl_DM_DMturbulence,
    &runnable_AOloopControl_DM_pokerndmodes,
};

MILK_MODULE_RUNNABLES(AOloopControl_DM, NULL, runnables);
