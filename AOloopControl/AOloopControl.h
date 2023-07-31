/**
 * @file    AOloopControl.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine
 *
 *
 * AO engine uses stream data structure
 *
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <time.h>


#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

#ifndef _AOLOOPCONTROL_H
#define _AOLOOPCONTROL_H



#define MAX_NUMBER_TIMER 100






void __attribute__((constructor)) libinit_AOloopControl();







/** @brief CPU based matrix-vector multiplication - when no GPU */
errno_t ControlMatrixMultiply(
    float *cm_array, float *imarray, long m, long n, float *outvect);

/** @brief Sends modal commands to DM by matrix-vector multiplication */
errno_t set_DM_modes(long loop);

/** @brief Response Matrix DM-WFS */
errno_t set_DM_modesRM(long loop);

/** @brief Main computation function, runs once per loop iteration */
errno_t AOcompute(long loop, int normalize);

/** @brief Main computation function, runs once per loop iteration */
errno_t AOloopControl_CompModes_loop(const char *ID_CM_name,
                                     const char *ID_WFSref_name,
                                     const char *ID_WFSim_name,
                                     const char *ID_WFSimtot_name,
                                     const char *ID_coeff_name);

/** @brief Matrix multiplication on GPU to transfom modes coefficients into DM
 * shape */
errno_t AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex,
        const char *modecoeffs_name,
        const char *DMmodes_name,
        int         semTrigg,
        const char *out_name,
        int         GPUindex,
        long        loop,
        int         offloadMode);

/** @brief CPU matrix multiplication to transfom WFS signal into modes
 * coefficients */
long AOloopControl_sig2Modecoeff(const char *WFSim_name,
                                 const char *IDwfsref_name,
                                 const char *WFSmodes_name,
                                 const char *outname);

/** @brief Compute WFS residual image */
long AOloopControl_computeWFSresidualimage(long loop, char *IDalpha_name);

/** @brief Compute modes in open loop */
imageID __attribute__((hot)) AOloopControl_ProcessModeCoefficients(long loop);

/** @brief Auto tune gains of the closed loop */
errno_t AOloopControl_AutoTuneGains(long        loop,
                                    const char *IDout_name,
                                    float       GainCoeff,
                                    long        NBsamples);

/** @brief Mixes streamin into streamout, in order to make streamout converge to
 * streamin  */
long AOloopControl_dm2dm_offload(const char *streamin,
                                 const char *streamout,
                                 float       twait,
                                 float       offcoeff,
                                 float       multcoeff);










/* =========================================================================================================
 */
/** @name AOloopControl - 3.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS
 * - AOloopControl_loop_param.c */
/* =========================================================================================================
 */

errno_t AOloopControl_setRTLOG_ON();
errno_t AOloopControl_setRTLOG_OFF();

/** @brief Set gain of the loop  */
errno_t AOloopControl_setgain(float gain);

/** @brief Set ARPF gain (auto regressive predictive filter)
 * Ex : a gain of 0.5 will correct 50% of the predicted WF
 */
errno_t AOloopControl_setARPFgain(float gain);

/** @brief Set ARPF */
errno_t AOloopControl_setARPFgainAutoMin(float val);

/** @brief Set ARPF */
errno_t AOloopControl_setARPFgainAutoMax(float val);

/** @brief Coefficient attenuates AO correction in low loght level */
errno_t AOloopControl_setWFSnormfloor(float WFSnormfloor);



/** @brief Scan block gains */
errno_t AOloopControl_scanGainBlock(
    long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 4. FOCAL PLANE SPECKLE MODULATION / CONTROL -
 * AOloopControl_fpspeckle_mod.c custom FP AO routines */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief Optimize PSF low order */
errno_t AOloopControl_OptimizePSF_LO(const char *psfstream_name,
                                     const char *IDmodes_name,
                                     const char *dmstream_name,
                                     long        delayframe,
                                     long        NBframes);

/** @brief Experimental dm modulation  */
errno_t AOloopControl_DMmodulateAB(const char *IDprobeA_name,
                                   const char *IDprobeB_name,
                                   const char *IDdmstream_name,
                                   const char *IDrespmat_name,
                                   const char *IDwfsrefstream_name,
                                   double      delay,
                                   long        NBprobes);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 5. PROCESS LOG FILES - AOloopControl_process_files.c
 *  process log files */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief Log the process of the mode evaluation  */
errno_t AOloopControl_logprocess_modeval(const char *IDname);

/** @brief tweak zonal response matrix in accordance to WFS response to modes */
errno_t AOloopControl_TweakRM(char *ZRMinname,
                              char *DMinCname,
                              char *WFSinCname,
                              char *DMmaskname,
                              char *WFSmaskname,
                              char *RMoutname);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 6. REAL-TIME LOGGING - AOloopControl_RTstreamLOG.c
 *  Log real-time streams */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

errno_t AOloopControl_RTstreamLOG_init(int loop);

errno_t
AOloopControl_RTstreamLOG_setup(long loop, long rtlindex, char *streamname);

void AOloopControl_RTstreamLOG_update(long            loop,
                                      long            rtlindex,
                                      struct timespec tnow);

int AOloopControl_RTstreamLOG_printstatus(int loop);

int AOloopControl_RTstreamLOG_GUI(int loop);
int AOloopControl_RTstreamLOG_saveloop(int loop, char *dirname);

int AOloopControl_RTstreamLOG_set_saveON(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_saveOFF(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_ON(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_OFF(int loop, int rtlindex);



#endif
