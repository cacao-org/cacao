// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine compute
 * calibration
 *
 * AO engine uses stream data structure
 *
 * @bug No known bugs.
 *
 */

#ifndef _AOLOOPCONTROL_COMPUTECALIB_H
#define _AOLOOPCONTROL_COMPUTECALIB_H

#include "milk_runnable.h"

// RM2zonal and compute_straight_CM are mandated units: declared weak in computeCalib.c
extern const MILK_RUNNABLE runnable_computeCalib_maskextrapolate;
extern const MILK_RUNNABLE runnable_computeCalib_actmap_sample2D;
extern const MILK_RUNNABLE runnable_computeCalib_computeHadamard;
extern const MILK_RUNNABLE runnable_computeCalib_generateRMWFS;
extern const MILK_RUNNABLE runnable_computeCalib_compute_masksWFSDM;
extern const MILK_RUNNABLE runnable_computeCalib_compute_control_modes;
extern const MILK_RUNNABLE runnable_computeCalib_mkCMsvd;
extern const MILK_RUNNABLE runnable_computeCalib_aolHaddec;


imageID AOloopControl_computeCalib_mkHadamardModes(const char *DMmask_name, const char *outname);

imageID AOloopControl_computeCalib_Hadamard_decodeRM(const char *inname,
                                                     const char *Hmatname,
                                                     const char *indexname,
                                                     const char *outname);

imageID AOloopControl_computeCalib_mkloDMmodes(const char *ID_name,
                                               uint32_t    msizex,
                                               uint32_t    msizey,
                                               float       CPAmax,
                                               float       deltaCPA,
                                               double      xc,
                                               double      yc,
                                               double      r0,
                                               double      r1,
                                               int         MaskMode);

errno_t AOcontrolLoop_computeCalib_ComputeCM_FPCONF();
errno_t AOcontrolLoop_computeCalib_ComputeCM_RUN();

errno_t AOloopControl_computeCalib_mkCalib_map_mask(const char *zrespm_name,
                                                    const char *WFSmap_name,
                                                    const char *DMmap_name,
                                                    float       dmmask_perclow,
                                                    float       dmmask_coefflow,
                                                    float       dmmask_perchigh,
                                                    float       dmmask_coeffhigh,
                                                    float       wfsmask_perclow,
                                                    float       wfsmask_coefflow,
                                                    float       wfsmask_perchigh,
                                                    float       wfsmask_coeffhigh);

errno_t AOloopControl_computeCalib_Process_zrespM(const char *IDzrespm0_name,
                                                  const char *IDwfsref_name,
                                                  const char *IDzrespm_name,
                                                  const char *WFSmap_name,
                                                  const char *DMmap_name);

errno_t AOloopControl_computeCalib_ProcessZrespM_medianfilt(const char *zrespm_name,
                                                            const char *WFSref0_name,
                                                            const char *WFSmap_name,
                                                            const char *DMmap_name,
                                                            double      rmampl,
                                                            int         normalize);

errno_t modes_spatial_extrapolate(IMGID imgmodes, IMGID imgmask, IMGID imgcpa, IMGID *imgoutmodes);

long AOloopControl_computeCalib_mkSlavedAct(const char *IDmaskRM_name,
                                            float       pixrad,
                                            const char *IDout_name);

long AOloopControl_computeCalib_DMedgeDetect(const char *IDmaskRM_name, const char *IDout_name);

long AOloopControl_computeCalib_DMextrapolateModes(const char *IDin_name,
                                                   const char *IDmask_name,
                                                   const char *IDcpa_name,
                                                   const char *IDout_name);

long AOloopControl_computeCalib_DMslaveExt(const char *IDin_name,
                                           const char *IDmask_name,
                                           const char *IDsl_name,
                                           const char *IDout_name,
                                           float       r0);

imageID AOloopControl_computeCalib_mkModes(const char *ID_name,
                                           uint32_t    msizex,
                                           uint32_t    msizey,
                                           float       CPAmax,
                                           float       deltaCPA,
                                           double      xc,
                                           double      yc,
                                           double      r0,
                                           double      r1,
                                           int         MaskMode,
                                           int         BlockNB,
                                           float       SVDlim,
                                           char       *stagedir);

imageID AOloopControl_computeCalib_mkModes_Simple(const char *IDin_name,
                                                  long        NBmblock,
                                                  long        Cmblock,
                                                  float       SVDlim);

imageID AOloopControl_computeCalib_compute_ControlMatrix(long        loop,
                                                         long        NB_MODE_REMOVED,
                                                         const char *ID_Rmatrix_name,
                                                         const char *ID_Cmatrix_name,
                                                         const char *ID_VTmatrix_name,
                                                         double      Beta,
                                                         long        NB_MODE_REMOVED_STEP,
                                                         float       eigenvlim);

errno_t AOloopControl_computeCalib_compute_CombinedControlMatrix(const char *IDcmat_name,
                                                                 const char *IDmodes_name,
                                                                 const char *IDwfsmask_name,
                                                                 const char *IDdmmask_name,
                                                                 const char *IDcmatc_name,
                                                                 const char *IDcmatc_active_name);

imageID AOloopControl_computeCalib_loadCM(long loop, const char *CMfname);

#endif
