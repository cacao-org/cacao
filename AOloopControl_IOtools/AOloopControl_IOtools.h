// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_IOtools.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine I/O
 * tools
 *
 * AO engine uses stream data structure
 *
 *
 * @bug No known bugs.
 *
 */

#ifndef _AOLOOPCONTROL_IOTOOLS_H
#define _AOLOOPCONTROL_IOTOOLS_H

#include "milk_runnable.h"

extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_acquireWFSim;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_acquireWFSspec;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_ao188_preprocessor;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_findspots;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_spotpos;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_WFScamsim;
extern const MILK_RUNNABLE runnable_AOloopControl_IOtools_WFSmap;

#endif
