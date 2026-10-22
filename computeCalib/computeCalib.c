// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib.c
 * @brief   AO loop compute calibration
 *
 * Module registry and the aolHaddec unit.
 */

#define MODULE_SHORTNAME_DEFAULT "cacaocc"
#define MODULE_DESCRIPTION "AO loop control compute calibration"

#define _GNU_SOURCE

#include "milk_runnable.h"

#include "computeCalib.h"

// Mandated units (LAPACKE, BLAS): NULL when their file is dropped from the build
MILK_RUNNABLE_WEAK_DECL(runnable_computeCalib_RM2zonal);
MILK_RUNNABLE_WEAK_DECL(runnable_computeCalib_compute_straight_CM);


/* =============================================================
 *  CMD 1: aolHaddec (4 args)
 * ============================================================ */

static char p_inrm[FUNCTION_PARAMETER_STRMAXLEN]   = "imRMh";
static char p_hmat[FUNCTION_PARAMETER_STRMAXLEN]   = "Hmat";
static char p_pixidx[FUNCTION_PARAMETER_STRMAXLEN] = "pixiind";
static char p_outrm[FUNCTION_PARAMETER_STRMAXLEN]  = "imRM";

#define FPS_PARAMS(X)                                                                          \
    X(".inrm", p_inrm, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "input RM")                 \
    X(".hmat", p_hmat, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "Hadamard matrix")          \
    X(".pixidx", p_pixidx, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT, "DM pixel index frame") \
    X(".outrm", p_outrm, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "output RM")

MILK_PARAMS_DEFINE(aolHaddec, FPS_PARAMS);

static MILK_HOT errno_t aolHaddec_compute(const MILK_RUNNABLE *self)
{
    (void) self;
    AOloopControl_computeCalib_Hadamard_decodeRM(p_inrm, p_hmat, p_pixidx, p_outrm);
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_computeCalib_aolHaddec = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "aolHaddec",
                 .cmdkey           = "aolHaddec",
                 .description      = "decode Hadamard matrix",
                 .description_long = "Decode Hadamard-encoded response matrix measurements. "
                                     "Applies the inverse Hadamard transform to recover "
                                     "per-actuator responses." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(aolHaddec),
    .compute = aolHaddec_compute,
};


/* =============================================================
 *  REGISTRATION
 * ============================================================ */

// Module registry: CLI exposure = membership here.
static const MILK_RUNNABLE *const runnables[] = {
    &runnable_computeCalib_aolHaddec,
    &runnable_computeCalib_compute_control_modes,
    &runnable_computeCalib_compute_straight_CM,
    &runnable_computeCalib_compute_masksWFSDM,
    &runnable_computeCalib_generateRMWFS,
    &runnable_computeCalib_computeHadamard,
    &runnable_computeCalib_maskextrapolate,
    &runnable_computeCalib_RM2zonal,
    &runnable_computeCalib_mkCMsvd,
    &runnable_computeCalib_actmap_sample2D,
};

MILK_MODULE_RUNNABLES(AOloopControl_computeCalib, NULL, runnables);
