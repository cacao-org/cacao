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


#include "AOloopControl/AOloopControl_ProcessModeCoefficients.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

#ifndef _AOLOOPCONTROL_H
#define _AOLOOPCONTROL_H

// OVERVIEW

// AOloopControl_var
// structure unique to specific process
// used for convenience to store process-specific values, for example image IDs

// AOLOOPCONTROL_CONF
// shared memory structure for sharing between processes and provide outside
// control and visibility

// PROCESSES
/*
 * AO loop can launch a number of processes
 *
 *
 *
 * All AO loop instances MUST start a AOloopControl_aorun() function call
 * FUNCTION: AOloopControl_aorun()
 * in file : AOloopControl_aorun.c
 * AOloopControl_aorun() -> AOcompute() -> Read_cam_frame()
 *
 *
 *
 *
 * FUNCTION: AOloopControl_ProcessModeCoefficients()
 * in file : AOloopControl_aorun.c
 *
 *
 *
 */

#define MAX_NUMBER_TIMER 100

// REAL TIME DATA LOGGING

//
// Real-time streams are updated once per loop iteration
// Realtime logging buffers are synchronized
// Two buffers are created for each stream to be logged
//
// Saving to disk is handled outside of cacao real-time
//

#define MAX_NUMBER_RTLOGSTREAM 20

#define RTSLOGindex_wfsim               0
#define RTSLOGindex_imWFS0              1
#define RTSLOGindex_imWFS1              2
#define RTSLOGindex_imWFS2              3
#define RTSLOGindex_modeval             4
#define RTSLOGindex_modeval_dm          5
#define RTSLOGindex_modeval_dm_corr     6
#define RTSLOGindex_modeval_dm_now      7
#define RTSLOGindex_modeval_dm_now_filt 8
#define RTSLOGindex_modevalPF           9
#define RTSLOGindex_modevalPFsync       10
#define RTSLOGindex_modevalPFres        11
#define RTSLOGindex_modeval_ol          12
#define RTSLOGindex_dmC                 13
#define RTSLOGindex_dmdisp              14

// Real-time streams use this struc to hold relevant info

// int   RTLOG_ON;                        // set to 1 to start logging all RT
// streams -> ensures synchronization int   RTstreamLOG_buff;                //
// Which buffer is currently being written (0 or 1) long  RTstreamLOG_frame; //
// Which frame is to be written in buffer int   RTstreamLOG_buffSwitch; // Goes
// to 1 when buffer switches. Goes back to zero on next iteration.

// For each stream, there are two data buffers and two timing buffers
// Buffer names:
//     aol%ld_<stream>_logbuff0
//     aol%ld_<stream>_logbuff1
//	   aol%ld_<stream>_logbuffinfo0
//     aol%ld_<stream>_logbuffinfo1
//
// Timing buffer contains loop iteration, frame counter cnt0 and time stamps
// when entry is written
//

// ENABLES, ON and save are read/set up in AOloopControl_loadconfigure()

typedef struct
{
    int  active;    // 1 if used
    char name[100]; // stream name (excludes aol#_)
    int ENABLE; // Is logging enabled ? This needs to be specified at startup, if
    // set to zero, no RT logging will be performed
    int      INIT;           // 1 if memory is initiated
    int      ON;             // Is logging ON ?
    uint32_t SIZE;           // Max number of samples per buffer
    int      buffindex;      // which buffer (0 or 1)
    long     frameindex;     // frame index
    long     frameindexend0; // last frame in buffer 0
    long     frameindexend1; // last frame in buffer 1
    int      save;   // 0: do not save, 1: save data+timing, 2: save timing only
    int memcpToggle; // 1 if file buffer #0 ready to be memcpied, 2 if file buffer
    // #1 ready to be memcpied, 0 otherwise

    long IDbuff;
    long IDbuff0; // local identifier
    long IDbuff1; // local identifier

    long IDbuffinfo;
    long IDbuffinfo0; // local identifier
    long IDbuffinfo1; // local identifier

    float *srcptr; // source stream pointer
    long   IDsrc;  // source ID

    char *destptr;  // destination pointer
    char *destptr0; // destination pointer 0
    char *destptr1; // destination pointer 1

    size_t memsize;     // size of stream frame (byte)
    int    NBcubeSaved; // Number of cubes to save, default = -1

    int  NBFileBuffer;     // Number of buffers per big file
    int  FileBuffer;       // File buffer index
    char timestring[100];  // current timestring
    char timestring0[100]; // timestring for file to be saved to

    int tActive; // 1: save thread is active, 0 otherwise

} RTstreamLOG;













void __attribute__((constructor)) libinit_AOloopControl();

int AOloopControl_bogusfunc();

int AOloopControl_aorun_GUI(long loop, double frequ);

/* ===============================================================================================
 */
/** @brief Read parameter value - float, char or int -
 * AOloopControl_read_param.c                */
/* ===============================================================================================
 */
/** @brief read parameters float */
float AOloopControl_readParam_float(char *paramname,
                                    float defaultValue,
                                    FILE *fplog);

/** @brief read parameters int */
int AOloopControl_readParam_int(char *paramname, int defaultValue, FILE *fplog);

/** @brief read parameters string */
char *AOloopControl_readParam_string(char *paramname,
                                     char *defaultValue,
                                     FILE *fplog);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 1. INITIALIZATION, configurations
 *
 * Allocate memory, import/export configurations
 *  AOloopControl_loadconfigure.c - AOloopControl_initmem.c
 */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief Load configuation parameters from disk -
 * AOloopControl_loadconfigure.c*/
errno_t AOloopControl_loadconfigure(long loop, int mode, int level);

/** @brief Initialize memory - function called within C code only (no CLI call)
 */
errno_t AOloopControl_InitializeMemory(int mode);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 2. REAL TIME COMPUTING ROUTINES - AOloopControl.c,
 * AOloopControl_wfs.c calls CPU and GPU processing */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief WFS zero point update */
errno_t AOloopControl_WFSzpupdate_loop(const char *IDzpdm_name,
                                       const char *IDzrespM_name,
                                       const char *IDwfszp_name);

/** @brief WFS sum zero point update */
errno_t AOloopControl_WFSzeropoint_sum_update_loop(long        loopnb,
        const char *ID_WFSzp_name,
        int         NBzp,
        const char *IDwfsref0_name,
        const char *IDwfsref_name);





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







/* ===============================================================================================
 */
/** @name AOloopControl - 3.2. LOOP CONTROL INTERFACE - DATA LOGGING */
/* ===============================================================================================
 */

// NB : doesn't exist anywhere ??
/** @brief Log on the AO interface */
errno_t AOloopControl_logon();

/** @brief Log off AO interface */
errno_t AOloopControl_logoff();




/* ========================================================================================================
 */
/** @name AOloopControl - 3.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF
 * AOloopControl_arpf_onoff.c */
/* ========================================================================================================
 */

/** @brief ARPF = auto regressive predictive filter: on */
errno_t AOloopControl_ARPFon();

/** @brief ARPF = auto regressive predictive filter: off */
errno_t AOloopControl_ARPFoff();

/* ===================================================================================================
 */
/** @name AOloopControl - 3.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS -
 * AOloopControl_time_param.c */
/* ===================================================================================================
 */

/** @brief Set AO loop frequency */
errno_t AOloopControl_set_loopfrequ(float loopfrequ);

/** @brief Set hardware latency in unity of frame */
errno_t AOloopControl_set_hardwlatency_frame(float hardwlatency_frame);

/** @brief Set computation latency of primary DM write in unity of frame */
errno_t AOloopControl_set_complatency_frame(float complatency_frame);

/** @brief Set computation latency of filtered DM write mode
 * time between the moment where the WF arrives at the WFS, and when it's
 * written in the DM
 */
errno_t AOloopControl_set_wfsmextrlatency_frame(float wfsmextrlatency_frame);

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

/** @brief Set the limit maximum */
errno_t AOloopControl_setmaxlimit(float maxlimit);

/** @brief Multiplying coefficient, close to 1, in order to avoid divergence */
errno_t AOloopControl_setmult(float multcoeff);

/** @brief Set an average of frames */
errno_t AOloopControl_setframesAve(long nbframes);

/** @brief Set gain of block of modes */
errno_t AOloopControl_set_modeblock_gain(long  loop,
        long  blocknb,
        float gain,
        int   add); // modal blocks

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
