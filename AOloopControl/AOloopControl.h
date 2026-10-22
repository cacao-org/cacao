// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine
 *
 *
 * AO engine uses stream data structure
 *
 *
 */


#ifndef _AOLOOPCONTROL_H
#define _AOLOOPCONTROL_H

#include "milk_runnable.h"

/** @brief Apply modal filtering in mode space (fpsexec "mfilt"). */
extern const MILK_RUNNABLE runnable_AOloopControl_modalfilter;

/** @brief Test input sequence for modal filter. */
extern const MILK_RUNNABLE runnable_AOloopControl_modalfilter_test;

/** @brief Modal control statistics. */
extern const MILK_RUNNABLE runnable_AOloopControl_modalCTRL_stats;

/** @brief Apply zonal filtering in DM space (fpsexec "cacao-zonalfilter"). */
extern const MILK_RUNNABLE runnable_AOloopControl_zonalfilter;

#endif
