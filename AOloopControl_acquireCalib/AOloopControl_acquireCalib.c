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
#define _GNU_SOURCE


#include "milk_runnable.h"

#include "AOloopControl_acquireCalib.h"

static const MILK_RUNNABLE *const runnables[] = {
    &runnable_AOloopControl_acquireCalib_acqWFSlincal,
    &runnable_AOloopControl_acquireCalib_measlinresp,
};

MILK_MODULE_RUNNABLES(AOloopControl_acquireCalib, NULL, runnables);
