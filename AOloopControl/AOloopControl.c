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


#include "milk_runnable.h"

#include "AOloopControl.h"

static const MILK_RUNNABLE *const runnables[] = {
    &runnable_AOloopControl_modalfilter,
    &runnable_AOloopControl_modalfilter_test,
    &runnable_AOloopControl_modalCTRL_stats,
    &runnable_AOloopControl_zonalfilter,
};

MILK_MODULE_RUNNABLES(AOloopControl, NULL, runnables);
