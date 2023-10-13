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

/** @brief Initialize module */
void __attribute__((constructor)) libinit_AOloopControl_computeCalib();



imageID AOloopControl_computeCalib_mkHadamardModes(const char *DMmask_name,
        const char *outname);

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

errno_t AOloopControl_computeCalib_mkCalib_map_mask(
    const char *zrespm_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    float       dmmask_perclow,
    float       dmmask_coefflow,
    float       dmmask_perchigh,
    float dmmask_coeffhigh,
    float wfsmask_perclow,
    float wfsmask_coefflow,
    float wfsmask_perchigh,
    float wfsmask_coeffhigh);

errno_t AOloopControl_computeCalib_Process_zrespM(
    const char *IDzrespm0_name,
    const char *IDwfsref_name,
    const char *IDzrespm_name,
    const char *WFSmap_name,
    const char *DMmap_name);

errno_t
AOloopControl_computeCalib_ProcessZrespM_medianfilt(
    const char *zrespm_name,
    const char *WFSref0_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    double      rmampl,
    int         normalize);

errno_t AOloopControl_computeCalib_mkCM_FPCONF();
errno_t AOloopControl_computeCalib_mkCM_RUN();
errno_t AOloopControl_computeCalib_mkCM(const char *respm_name, float SVDlim);

long AOloopControl_computeCalib_mkSlavedAct(const char *IDmaskRM_name,
        float       pixrad,
        const char *IDout_name);

long AOloopControl_computeCalib_DMedgeDetect(const char *IDmaskRM_name,
        const char *IDout_name);

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

imageID
AOloopControl_computeCalib_compute_ControlMatrix(long        loop,
        long        NB_MODE_REMOVED,
        const char *ID_Rmatrix_name,
        const char *ID_Cmatrix_name,
        const char *ID_VTmatrix_name,
        double      Beta,
        long  NB_MODE_REMOVED_STEP,
        float eigenvlim);

errno_t AOloopControl_computeCalib_compute_CombinedControlMatrix(
    const char *IDcmat_name,
    const char *IDmodes_name,
    const char *IDwfsmask_name,
    const char *IDdmmask_name,
    const char *IDcmatc_name,
    const char *IDcmatc_active_name);

imageID AOloopControl_computeCalib_loadCM(long loop, const char *CMfname);

#endif
