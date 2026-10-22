// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_IOtools.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 *
 * AO engine uses stream data structure
 *
 * @bug No known bugs.
 *
 *
 */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaoio"

// Module short description
#define MODULE_DESCRIPTION "AO loop control IO tools"

#include "milk_runnable.h"

#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

static const MILK_RUNNABLE *const runnables[] = {
    &runnable_AOloopControl_IOtools_acquireWFSim,
    &runnable_AOloopControl_IOtools_spotpos,
    &runnable_AOloopControl_IOtools_WFScamsim,
    &runnable_AOloopControl_IOtools_WFSmap,
    &runnable_AOloopControl_IOtools_findspots,
    &runnable_AOloopControl_IOtools_ao188_preprocessor,
    &runnable_AOloopControl_IOtools_acquireWFSspec,
};

MILK_MODULE_RUNNABLES(AOloopControl_IOtools, NULL, runnables);
