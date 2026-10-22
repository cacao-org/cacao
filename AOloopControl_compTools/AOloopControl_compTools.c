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

#define _GNU_SOURCE

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl_compTools.h"

static const MILK_RUNNABLE *const runnables[] = { NULL };

MILK_MODULE_RUNNABLES(AOloopControl_compTools, NULL, runnables);
