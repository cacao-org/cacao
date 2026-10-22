// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_acquireCalib.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine acquire
 * calibration
 *
 * AO engine uses stream data structure
 *
 */

#ifndef _AOLOOPCONTROL_ACQUIRECALIB_H
#define _AOLOOPCONTROL_ACQUIRECALIB_H

#include "milk_runnable.h"

extern const MILK_RUNNABLE runnable_AOloopControl_acquireCalib_acqWFSlincal;
extern const MILK_RUNNABLE runnable_AOloopControl_acquireCalib_measlinresp;

#endif
