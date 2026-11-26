// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    pyramidWFStools.c
 * @brief   Pyramid WFS calibration and telemetry module registration (`cacaopyrwfs`)
 */

#define MODULE_SHORTNAME_DEFAULT "cacaopyrwfs"
#define MODULE_DESCRIPTION       "pyramid WFS tools"

#define _GNU_SOURCE

#include "milk_runnable.h"

#include "pyramidWFStools.h"

static const MILK_RUNNABLE *const runnables[] = { &runnable_pyramidWFStools_pyWFSgridmatch,
                                                  &runnable_pyramidWFStools_pyWFStlm };

MILK_MODULE_RUNNABLES(pyramidWFStools, NULL, runnables);
