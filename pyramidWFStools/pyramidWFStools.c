// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaopyrwfs"

// Module short description
#define MODULE_DESCRIPTION "pyramid WFS tools"

#define _GNU_SOURCE

#include "milk_runnable.h"

#include "pyramidWFStools.h"

static const MILK_RUNNABLE *const runnables[] = { &runnable_pyramidWFStools_pyWFSgridmatch };

MILK_MODULE_RUNNABLES(pyramidWFStools, NULL, runnables);
