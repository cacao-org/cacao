/**
 * @file    AOloopControl.c
 * @brief   Adaptive Optics Control loop engine
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    10 Sept 2017 -- 
 *
 * @bug No known bugs.
 * 
 * @see http://oguyon.github.io/AdaptiveOpticsControl/src/AOloopControl/doc/AOloopControl.html
 *  
 * @defgroup AOloopControl_streams Image streams
 * @defgroup AOloopControl_AOLOOPCONTROL_CONF AOloopControl main data structure
 * 
 */


#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/syscall.h> // needed for tid = syscall(SYS_gettid);



#ifdef __MACH__   // for Mac OS X - 
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t) {
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time;
    time = mach_absolute_time();
    double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
    double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
    t->tv_sec = seconds;
    t->tv_nsec = nseconds;
    return 0;
}
#else
#include <time.h>
#endif

#include <math.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <err.h>
#include <fcntl.h>
#include <sched.h>
//#include <ncurses.h>
#include <semaphore.h>


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>
#include <pthread.h>

#include <fitsio.h>


//libraries created by O. Guyon 
#include "CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "linopt_imtools/linopt_imtools.h"
#include "AOloopControl/AOloopControl.h"
#include "image_filter/image_filter.h"
#include "info/info.h"
#include "ZernikePolyn/ZernikePolyn.h"
#include "linopt_imtools/linopt_imtools.h"
#include "image_gen/image_gen.h"
#include "statistic/statistic.h"
#include "fft/fft.h"


#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"



#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif



/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */




#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)

// data passed to each thread
//typedef struct
//{
//   long nelem;
//    float *arrayptr;
//    float *result; // where to white status
//} THDATA_IMTOTAL;




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/*                				    LOGGING ACCESS TO FUNCTIONS           	                       */
/* =============================================================================================== */

// uncomment at compilation time to enable logging of function entry/exit
//#define AOLOOPCONTROL_LOGFUNC
static int AOLOOPCONTROL_logfunc_level = 0;
static int AOLOOPCONTROL_logfunc_level_max = 2; // log all levels equal or below this number
static char AOLOOPCONTROL_logfunc_fname[] = "AOloopControl.fcall.log";
static char flogcomment[200];


// GPU MultMat indexes
//
// 0: main loop CM multiplication
//
// 1: set DM modes:
//         int set_DM_modes(long loop)
//
// 2: compute modes loop
//         int AOloopControl_CompModes_loop(char *ID_CM_name, char *ID_WFSref_name, char *ID_WFSim_name, char *ID_WFSimtot_name, char *ID_coeff_name)
//
// 3: coefficients to DM shape
//         int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg, char *out_name, int GPUindex, long loop, int offloadMode)
//
// 4: Predictive control (in modules linARfilterPred)
//
// 5: coefficients to DM shape, PF
// 			int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg, char *out_name, int GPUindex, long loop, int offloadMode)
//
// 6: coefficients to DM shape, OL
//			int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg, char *out_name, int GPUindex, long loop, int offloadMode)
//




// TIMING
static struct timespec tnow;
static struct timespec tdiff;
static double tdiffv;



static int initWFSref_GPU[100] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int initcontrMcact_GPU[100] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float GPU_alpha = 0.0;
float GPU_beta = 0.0;


static int COMPUTE_PIXELSTREAMING = 0; // multiple pixel groups
int PIXSTREAM_NBSLICES = 1; // number of image slices (= pixel groups)
int PIXSTREAM_SLICE; // slice index 0 = all pixels




// static int MATRIX_COMPUTATION_MODE = 0;
// 0: compute sequentially modes and DM commands
// 1: use combined control matrix





/* =============================================================================================== */
/*                    aoconfID are global variables for convenience                                */
/*  aoconfID can be used in other modules as well (with extern)                                   */
/* =============================================================================================== */


// Hardware connections
long aoconfID_wfsim = -1;
uint8_t WFSatype;
long aoconfID_dmC = -1;
long aoconfID_dmRM = -1;

long aoconfID_wfsdark = -1;
long aoconfID_imWFS0 = -1;
long aoconfID_imWFS0tot = -1;
long aoconfID_imWFS1 = -1;
long aoconfID_imWFS2 = -1;
static long aoconfID_wfsref0 = -1;
long aoconfID_wfsref = -1;
static long long aoconfcnt0_wfsref_current = -1;

long aoconfID_DMmodes = -1;
static long aoconfID_dmdisp = -1;  // to notify DMcomb that DM maps should be summed



// Control Modes
long aoconfID_cmd_modes = -1;
long aoconfID_meas_modes = -1; // measured
long aoconfID_RMS_modes = -1;
long aoconfID_AVE_modes = -1;
long aoconfID_modeARPFgainAuto = -1;
long aoconfID_modevalPF = -1;

// mode gains, multf, limit are set in 3 tiers
// global gain
// block gain
// individual gains

// blocks
long aoconfID_gainb = -1; // block modal gains
long aoconfID_multfb = -1; // block modal gains
long aoconfID_limitb = -1; // block modal gains

// individual modes
long aoconfID_DMmode_GAIN = -1;
long aoconfID_LIMIT_modes = -1;
long aoconfID_MULTF_modes = -1;

long aoconfID_cmd_modesRM = -1;

long aoconfID_wfsmask = -1;
static long aoconfID_dmmask = -1;

static long aoconfID_respM = -1;
static long aoconfID_contrM = -1; // pixels -> modes
static long long aoconfcnt0_contrM_current = -1;
static long aoconfID_contrMc = -1; // combined control matrix: pixels -> DM actuators
static long aoconfID_meas_act = -1;
static long aoconfID_contrMcact[100] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

// pixel streaming
long aoconfID_pixstream_wfspixindex; // index of WFS pixels

// timing
long aoconfID_looptiming = -1; // control loop timing data. Pixel values correspond to time offset
// currently has 20 timing slots
// beginning of iteration is defined when entering "wait for image"
// md[0].atime.ts is absolute time at beginning of iteration
//
// pixel 0 is dt since last iteration
//
// pixel 1 is time from beginning of loop to status 01
// pixel 2 is time from beginning of loop to status 02
// ...
long AOcontrolNBtimers = 35;

static long aoconfIDlogdata = -1;
static long aoconfIDlog0 = -1;
static long aoconfIDlog1 = -1;



int *WFS_active_map; // used to map WFS pixels into active array
int *DM_active_map; // used to map DM actuators into active array
static long aoconfID_meas_act_active;
static long aoconfID_imWFS2_active[100];

float normfloorcoeff = 1.0;

static long wfsrefcnt0 = -1;
static long contrMcactcnt0[100] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};;


// variables used by functions

static int GPUcntMax = 100;
static int *GPUset0;
static int *GPUset1;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

#define NB_AOloopcontrol 10 // max number of loops
long LOOPNUMBER = 0; // current loop index

int AOloopcontrol_meminit = 0;
static int AOlooploadconf_init = 0;

#define AOconfname "/tmp/AOconf.shm"
AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. INITIALIZATION, configurations
 *  Allocate memory, import/export configurations */
/* =============================================================================================== */
/* =============================================================================================== */





/** @brief CLI function for AOloopControl_loadconfigure */
int_fast8_t AOloopControl_loadconfigure_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_loadconfigure(data.cmdargtoken[1].val.numl, 1, 10);
        return 0;
    }
    else return 1;
}


/** @brief CLI function for AOloopControl_stream3Dto2D */
/*int_fast8_t AOloopControl_stream3Dto2D_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0) {
        AOloopControl_stream3Dto2D(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else return 1;
}*/




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 6. REAL TIME COMPUTING ROUTINES
 *  calls CPU and GPU processing */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_WFSzpupdate_loop */
int_fast8_t AOloopControl_WFSzpupdate_loop_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)==0) {
        AOloopControl_WFSzpupdate_loop(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_WFSzeropoint_sum_update_loop */
int_fast8_t AOloopControl_WFSzeropoint_sum_update_loop_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,4)+CLI_checkarg(4,4)==0) {
        AOloopControl_WFSzeropoint_sum_update_loop(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_CompModes_loop */
int_fast8_t AOloopControl_CompModes_loop_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(5,3)==0) {
        AOloopControl_CompModes_loop(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_GPUmodecoeffs2dm_filt */
int_fast8_t AOloopControl_GPUmodecoeffs2dm_filt_loop_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,2)+CLI_checkarg(5,4)+CLI_checkarg(6,2)+CLI_checkarg(7,2)+CLI_checkarg(8,2)==0) {
        AOloopControl_GPUmodecoeffs2dm_filt_loop(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.numl, data.cmdargtoken[7].val.numl, data.cmdargtoken[8].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_computeWFSresidualimage */
int_fast8_t AOloopControl_computeWFSresidualimage_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,4)==0) {
        AOloopControl_computeWFSresidualimage(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_ComputeOpenLoopModes */
int_fast8_t AOloopControl_ComputeOpenLoopModes_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_ComputeOpenLoopModes(data.cmdargtoken[1].val.numl);
        return 0;
    } else return 1;
}

/** @brief CLI function for AOloopControl_AutoTuneGains */
int_fast8_t AOloopControl_AutoTuneGains_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,3)+CLI_checkarg(3,1)+CLI_checkarg(4,2)==0) {
        AOloopControl_AutoTuneGains(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_dm2dm_offload */
int_fast8_t AOloopControl_dm2dm_offload_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,1)+CLI_checkarg(4,1)+CLI_checkarg(5,1)==0) {
        AOloopControl_dm2dm_offload(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_sig2Modecoeff */
int_fast8_t AOloopControl_sig2Modecoeff_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,3)==0) {
        AOloopControl_sig2Modecoeff(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string,  data.cmdargtoken[4].val.string);
        return 0;
    } else return 1;
}



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 8.   LOOP CONTROL INTERFACE */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_setLoopNumber */
int_fast8_t AOloopControl_setLoopNumber_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_setLoopNumber(data.cmdargtoken[1].val.numl);
        return 0;
    }
    else return 1;
}


/* =============================================================================================== */
/** @name AOloopControl - 8.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET */
/* =============================================================================================== */


/* =============================================================================================== */
/** @name AOloopControl - 8.2. LOOP CONTROL INTERFACE - DATA LOGGING                               */
/* =============================================================================================== */


/* =============================================================================================== */
/** @name AOloopControl - 8.3. LOOP CONTROL INTERFACE - PRIMARY DM WRITE                           */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING                     */
/* =============================================================================================== */


/* =============================================================================================== */
/** @name AOloopControl - 8.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF                   */
/* =============================================================================================== */


/* =============================================================================================== */
/** @name AOloopControl - 8.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS                          */
/* =============================================================================================== */


/* =============================================================================================== */
/** @name AOloopControl - 8.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS                    */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_set_modeblock_gain */
int_fast8_t AOloopControl_set_modeblock_gain_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)+CLI_checkarg(3,2)==0) {
        AOloopControl_set_modeblock_gain(LOOPNUMBER, data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_loopstep */
int_fast8_t AOloopControl_loopstep_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_loopstep(LOOPNUMBER, data.cmdargtoken[1].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_loopfrequ */
int_fast8_t AOloopControl_set_loopfrequ_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_loopfrequ(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_hardwlatency_frame */
int_fast8_t AOloopControl_set_hardwlatency_frame_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_hardwlatency_frame(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_complatency_frame */
int_fast8_t AOloopControl_set_complatency_frame_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_complatency_frame(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_wfsmextrlatency_frame */
int_fast8_t AOloopControl_set_wfsmextrlatency_frame_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_wfsmextrlatency_frame(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_delta */
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_delta_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_AUTOTUNE_LIMITS_delta(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_perc */
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_perc_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_AUTOTUNE_LIMITS_perc(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_mcoeff */
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_mcoeff_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setgain */
int_fast8_t AOloopControl_setgain_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setgain(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setARPFgain */
int_fast8_t AOloopControl_setARPFgain_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setARPFgain(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setARPFgain */
int_fast8_t AOloopControl_setARPFgainAutoMin_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setARPFgainAutoMin(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setARPFgain */
int_fast8_t AOloopControl_setARPFgainAutoMax_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setARPFgainAutoMax(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}



/** @brief CLI function for AOloopControl_setWFSnormfloor */
int_fast8_t AOloopControl_setWFSnormfloor_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setWFSnormfloor(data.cmdargtoken[1].val.numf);
        return 0;
    } else return 1;
}

/** @brief CLI function for AOloopControl_setmaxlimit */
int_fast8_t AOloopControl_setmaxlimit_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setmaxlimit(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setmult */
int_fast8_t AOloopControl_setmult_cli() {
    if(CLI_checkarg(1,1)==0) {
        AOloopControl_setmult(data.cmdargtoken[1].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setframesAve */
int_fast8_t AOloopControl_setframesAve_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_setframesAve(data.cmdargtoken[1].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setgainrange */
int_fast8_t AOloopControl_setgainrange_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,1)==0) {
        AOloopControl_setgainrange(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setlimitrange */
int_fast8_t AOloopControl_setlimitrange_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,1)==0) {
        AOloopControl_setlimitrange(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setmultfrange */
int_fast8_t AOloopControl_setmultfrange_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,1)==0) {
        AOloopControl_setmultfrange(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setgainblock */
int_fast8_t AOloopControl_setgainblock_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0) {
        AOloopControl_setgainblock(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setlimitblock */
int_fast8_t AOloopControl_setlimitblock_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0) {
        AOloopControl_setlimitblock(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_setmultfblock */
int_fast8_t AOloopControl_setmultfblock_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0) {
        AOloopControl_setmultfblock(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_scanGainBlock */
int_fast8_t AOloopControl_scanGainBlock_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,1)+CLI_checkarg(4,1)+CLI_checkarg(5,2)==0) {
        AOloopControl_scanGainBlock(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numl);
        return 0;
    }
    else    return 1;
}



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 10. FOCAL PLANE SPECKLE MODULATION / CONTROL                             */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_DMmodulateAB */
int_fast8_t AOloopControl_DMmodulateAB_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(5,4)+CLI_checkarg(6,1)+CLI_checkarg(7,2)==0)   {
        AOloopControl_DMmodulateAB(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numl);
        return 0;
    }
    else        return 1;
}


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 11. PROCESS LOG FILES                                                    */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_logprocess_modeval */
int_fast8_t AOloopControl_logprocess_modeval_cli() {
    if(CLI_checkarg(1,4)==0) {
        AOloopControl_logprocess_modeval(data.cmdargtoken[1].val.string);
        return 0;
    }
    else return 1;
}




// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string




// OBSOLETE ??


int_fast8_t AOloopControl_setparam_cli()
{
    if(CLI_checkarg(1,3)+CLI_checkarg(2,1)==0)
    {
        AOloopControl_setparam(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else
        return 1;
}


/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl functions */


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. INITIALIZATION, configurations                                        */
/* =============================================================================================== */
/* =============================================================================================== */


void __attribute__ ((constructor)) libinit_AOloopControl()
{
	init_AOloopControl();
	printf(" ...... Loading module %s\n", __FILE__);
}


int AOloopControl_bogusfunc()
{
	printf("This function does NOTHING !\n");
	return 0;
}


// CODING STANDARD NOTE: minimal required documentation for doxygen
/**
 *  ## Purpose
 * 
 * Initialization of the AO loop Control with all of the CLI command line interface commands
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * mode		INT
 * 			mode sets up what function does
 * -		does nothing
 * -		also does nothing
 * 
 */
// CODING STANDARD NOTE: function name start with module name


void init_AOloopControl()
{
    FILE *fp;



#ifdef AOLOOPCONTROL_LOGFUNC
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    if((fp=fopen("LOOPNUMBER","r"))!=NULL)
    {
        if(fscanf(fp,"%8ld", &LOOPNUMBER) != 1)
            printERROR(__FILE__,__func__,__LINE__, "Cannot read LOOPNUMBER");

        printf("LOOP NUMBER = %ld\n", LOOPNUMBER);
        fclose(fp);
    }
    else
        LOOPNUMBER = 0;



    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "AO loop control");
    data.NBmodule++;


    RegisterCLIcommand("aolloadconf",__FILE__, AOloopControl_loadconfigure_cli, "load AO loop configuration", "<loop #>", "AOlooploadconf 1", "int AOloopControl_loadconfigure(long loopnb, 1, 10)");



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 2. LOW LEVEL UTILITIES & TOOLS                                           */
/* =============================================================================================== */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 2.1. LOW LEVEL UTILITIES & TOOLS - LOAD DATA STREAMS                     */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 2.2. LOW LEVEL UTILITIES & TOOLS - DATA STREAMS PROCESSING               */
/* =============================================================================================== */

/*    RegisterCLIcommand("aveACshmim", __FILE__, AOloopControl_AveStream_cli, "average and AC shared mem image", "<input image> <coeff> <output image ave> <output AC> <output RMS>" , "aveACshmim imin 0.01 outave outAC outRMS", "int AOloopControl_AveStream(char *IDname, double alpha, char *IDname_out_ave, char *IDname_out_AC, char *IDname_out_RMS)");

    RegisterCLIcommand("aolstream3Dto2D", __FILE__, AOloopControl_stream3Dto2D_cli, "remaps 3D cube into 2D image", "<input 3D stream> <output 2D stream> <# cols> <sem trigger>" , "aolstream3Dto2D in3dim out2dim 4 1", "long AOloopControl_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem)");
*/



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 6. REAL TIME COMPUTING ROUTINES                                          */
/* =============================================================================================== */
/* =============================================================================================== */
    

    RegisterCLIcommand("aolrun", __FILE__, AOloopControl_run, "run AO loop", "no arg", "aolrun", "int AOloopControl_run()"); // run AO loop

    RegisterCLIcommand("aolzpwfsloop",__FILE__, AOloopControl_WFSzpupdate_loop_cli, "WFS zero point offset loop", "<dm offset [shared mem]> <zonal resp M [shared mem]> <nominal WFS reference>  <modified WFS reference>", "aolzpwfsloop dmZP zrespM wfszp", "int AOloopControl_WFSzpupdate_loop(char *IDzpdm_name, char *IDzrespM_name, char *IDwfszp_name)"); //WFS zero point offset loop

    RegisterCLIcommand("aolzpwfscloop", __FILE__, AOloopControl_WFSzeropoint_sum_update_loop_cli, "WFS zero point offset loop: combine multiple input channels", "<name prefix> <number of channels> <wfsref0> <wfsref>", "aolzpwfscloop wfs2zpoffset 4 wfsref0 wfsref", "int AOloopControl_WFSzeropoint_sum_update_loop(long loopnb, char *ID_WFSzp_name, int NBzp, char *IDwfsref0_name, char *IDwfsref_name)"); //WFS zero point offset loop: combine multiple input channels

    RegisterCLIcommand("aocmlrun", __FILE__, AOloopControl_CompModes_loop_cli, "run AO compute modes loop", "<CM> <wfsref> <WFS image stream> <WFS image total stream> <output stream>", "aocmlrun CM wfsref wfsim wfsimtot aomodeval", "int AOloopControl_CompModes_loop(char *ID_CM_name, char *ID_WFSref_name, char *ID_WFSim_name, char *ID_WFSimtot, char *ID_coeff_name)"); // run AO compute modes loop

    RegisterCLIcommand("aolmc2dmfilt", __FILE__, AOloopControl_GPUmodecoeffs2dm_filt_loop_cli, "convert mode coefficients to DM map", "<GPUMATMULTindex> <mode coeffs> <DMmodes> <sem trigg number> <out> <GPUindex> <loopnb> <offloadMode>", "aolmc2dmfilt aolmodeval DMmodesC 2 dmmapc 0.2 1 2 1", "int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg, char *out_name, int GPUindex, long loop, long offloadMode)"); //convert mode coefficients to DM map

    RegisterCLIcommand("aolsig2mcoeff", __FILE__, AOloopControl_sig2Modecoeff_cli, "convert signals to mode coeffs", "<signal data cube> <reference> <Modes data cube> <output image>", "aolsig2mcoeff wfsdata wfsref wfsmodes outim", "long AOloopControl_sig2Modecoeff(char *WFSim_name, char *IDwfsref_name, char *WFSmodes_name, char *outname)"); // convert signals to mode coeffs


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 8.   LOOP CONTROL INTERFACE                                              */
/* =============================================================================================== */
/* =============================================================================================== */
    
    RegisterCLIcommand("aolnb", __FILE__, AOloopControl_setLoopNumber_cli, "set AO loop #", "<loop nb>", "AOloopnb 0", "int AOloopControl_setLoopNumber(long loop)"); // set AO loop 

/* =============================================================================================== */
/** @name AOloopControl - 8.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.2. LOOP CONTROL INTERFACE - DATA LOGGING                               */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.3. LOOP CONTROL INTERFACE - PRIMARY DM WRITE                           */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING                     */
/* =============================================================================================== */
  

/* =============================================================================================== */
/** @name AOloopControl - 8.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF                   */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS                          */
/* =============================================================================================== */

/* =============================================================================================== */
/** @name AOloopControl - 8.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS                    */
/* =============================================================================================== */

    RegisterCLIcommand("aolsetgain", __FILE__, AOloopControl_setgain_cli, "set gain", "<gain value>", "aolsetgain 0.1", "int AOloopControl_setgain(float gain)");

    RegisterCLIcommand("aolsetARPFgain", __FILE__, AOloopControl_setARPFgain_cli, "set auto-regressive predictive filter gain", "<gain value>", "aolsetARPFgain 0.1", "int AOloopControl_setARPFgain(float gain)");

    RegisterCLIcommand("aolsetARPFgainAmin", __FILE__, AOloopControl_setARPFgainAutoMin_cli, "set ARPF gain min", "<gain value>", "aolsetARPFgainAmin 0.1", "int AOloopControl_setARPFgainAutoMin(float val)");

    RegisterCLIcommand("aolsetARPFgainAmax", __FILE__, AOloopControl_setARPFgainAutoMax_cli, "set ARPF gain max", "<gain value>", "aolsetARPFgainAmax 9.0", "int AOloopControl_setARPFgainAutoMax(float val)");

    RegisterCLIcommand("aolkill", __FILE__, AOloopControl_loopkill, "kill AO loop", "no arg", "aolkill", "int AOloopControl_setLoopNumber()");

    RegisterCLIcommand("aolon", __FILE__, AOloopControl_loopon, "turn loop on", "no arg", "aolon", "int AOloopControl_loopon()");

    RegisterCLIcommand("aoloff", __FILE__, AOloopControl_loopoff, "turn loop off", "no arg", "aoloff", "int AOloopControl_loopoff()");

    RegisterCLIcommand("aolstep",__FILE__, AOloopControl_loopstep_cli, "turn loop on for N steps", "<nbstep>", "aolstep", "int AOloopControl_loopstep(long loop, long NBstep)");

    RegisterCLIcommand("aolreset", __FILE__, AOloopControl_loopreset, "reset loop, and turn it off", "no arg", "aolreset", "int AOloopControl_loopreset()");

    RegisterCLIcommand("aolsetmbgain",__FILE__, AOloopControl_set_modeblock_gain_cli, "set modal block gain", "<loop #> <gain> <compute sum flag>", "aolsetmbgain 2 0.2 1", "int AOloopControl_set_modeblock_gain(long loop, long blocknb, float gain, int add)");

    RegisterCLIcommand("aolDMprimWon", __FILE__, AOloopControl_DMprimaryWrite_on, "turn DM primary write on", "no arg", "aolDMprimWon", "int AOloopControl_DMprimaryWrite_on()");

    RegisterCLIcommand("aolDMprimWoff", __FILE__, AOloopControl_DMprimaryWrite_off, "turn DM primary write off", "no arg", "aolDMprimWoff", "int AOloopControl_DMprimaryWrite_off()");

    RegisterCLIcommand("aolDMfiltWon", __FILE__, AOloopControl_DMfilteredWrite_on, "turn DM filtered write on", "no arg", "aolDMfiltWon", "int AOloopControl_DMfilteredWrite_on()");

    RegisterCLIcommand("aolDMfiltWoff", __FILE__, AOloopControl_DMfilteredWrite_off, "turn DM filtered write off", "no arg", "aolDMfiltWoff", "int AOloopControl_DMfilteredWrite_off()");

    RegisterCLIcommand("aolAUTOTUNELIMon", __FILE__, AOloopControl_AUTOTUNE_LIMITS_on, "turn auto-tuning modal limits on", "no arg", "aolAUTOTUNELIMon", "int AOloopControl_AUTOTUNE_LIMITS_on()");

    RegisterCLIcommand("aolAUTOTUNELIMoff", __FILE__, AOloopControl_AUTOTUNE_LIMITS_off, "turn auto-tuning modal limits off", "no arg", "aolAUTOTUNELIMoff", "int AOloopControl_AUTOTUNE_LIMITS_off()");

    RegisterCLIcommand("aolsetATlimd", __FILE__, AOloopControl_set_AUTOTUNE_LIMITS_delta_cli, "set auto-tuning modal limits delta", "<delta value [um]>", "aolsetATlimd 0.0001", "int AOloopControl_set_AUTOTUNE_LIMITS_delta(float AUTOTUNE_LIMITS_delta)");

    RegisterCLIcommand("aolsetATlimp", __FILE__, AOloopControl_set_AUTOTUNE_LIMITS_perc_cli, "set auto-tuning modal limits percentile", "<percentile value [percent]>", "aolsetATlimp 1.0", "int AOloopControl_set_AUTOTUNE_LIMITS_perc(float AUTOTUNE_LIMITS_perc)");

    RegisterCLIcommand("aolsetATlimm", __FILE__, AOloopControl_set_AUTOTUNE_LIMITS_mcoeff_cli, "set auto-tuning modal limits multiplicative coeff", "<multiplicative coeff [float]>", "aolsetATlimm 1.5", "int AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(float AUTOTUNE_LIMITS_mcoeff)");

    RegisterCLIcommand("aolAUTOTUNEGAINon", __FILE__, AOloopControl_AUTOTUNE_GAINS_on, "turn auto-tuning modal gains on", "no arg", "aolAUTOTUNEGAINon", "int AOloopControl_AUTOTUNE_GAINS_on()");

    RegisterCLIcommand("aolAUTOTUNEGAINoff", __FILE__, AOloopControl_AUTOTUNE_GAINS_off, "turn auto-tuning modal gains off", "no arg", "aolAUTOTUNEGAINoff", "int AOloopControl_AUTOTUNE_GAINS_off()");

    RegisterCLIcommand("aolARPFon", __FILE__, AOloopControl_ARPFon, "turn auto-regressive predictive filter on", "no arg", "aolARPFon", "int AOloopControl_ARPFon()");

    RegisterCLIcommand("aolARPFoff", __FILE__, AOloopControl_ARPFoff, "turn auto-regressive predictive filter off", "no arg", "aolARPFoff", "int AOloopControl_ARPFoff()");




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 10. FOCAL PLANE SPECKLE MODULATION / CONTROL                             */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 11. PROCESS LOG FILES                                                    */
/* =============================================================================================== */
/* =============================================================================================== */



    RegisterCLIcommand("aolsetloopfrequ",
                       __FILE__,
                       AOloopControl_set_loopfrequ_cli,
                       "set loop frequency",
                       "<loop frequ [Hz]>",
                       "aolsetloopfrequ 2000",
                       "int AOloopControl_set_loopfrequ(float loopfrequ)");



    RegisterCLIcommand("aolsethlat", __FILE__, AOloopControl_set_hardwlatency_frame_cli, "set hardware latency", "<hardware latency [frame]>", "aolsethlat 2.7", "int AOloopControl_set_hardwlatency_frame(float hardwlatency_frame)");

    RegisterCLIcommand("aolsetclat",__FILE__, AOloopControl_set_complatency_frame_cli,"set computation latency", "<computation latency [frame]>", "aolsetclat 0.6", "int AOloopControl_set_complatency_frame(float complatency_frame)");

    RegisterCLIcommand("aolsetwlat", __FILE__, AOloopControl_set_wfsmextrlatency_frame_cli, "set WFS mode extraction latency", "<latency [frame]>", "aolsetwlat 0.8", "int AOloopControl_set_wfsmextrlatency_frame(float wfsmextrlatency_frame)");

    RegisterCLIcommand("aolsetwfsnormf", __FILE__, AOloopControl_setWFSnormfloor_cli, "set WFS normalization floor", "<floor value (total flux)>", "aolsetwfsnormf 10000.0", "int AOloopControl_setWFSnormfloor(float WFSnormfloor)");

    RegisterCLIcommand("aolsetmaxlim", __FILE__, AOloopControl_setmaxlimit_cli, "set max limit for AO mode correction", "<limit value>", "aolsetmaxlim 0.01", "int AOloopControl_setmaxlimit(float maxlimit)");

    RegisterCLIcommand("aolsetmult", __FILE__, AOloopControl_setmult_cli, "set mult coeff for AO mode correction", "<mult value>", "aolsetmult 0.98", "int AOloopControl_setmult(float multcoeff)");

    RegisterCLIcommand("aolsetnbfr",__FILE__, AOloopControl_setframesAve_cli, "set number of frames to be averaged", "<nb frames>", "aolsetnbfr 10", "int AOloopControl_setframesAve(long nbframes)");



    RegisterCLIcommand("aollogprocmodeval",__FILE__, AOloopControl_logprocess_modeval_cli, "process log image modeval", "<modeval image>", "aollogprocmodeval imc", "int AOloopControl_logprocess_modeval(const char *IDname);");



    RegisterCLIcommand("aolsetgainr", __FILE__, AOloopControl_setgainrange_cli, "set modal gains from m0 to m1 included", "<modemin [long]> <modemax [long]> <gainval>", "aolsetgainr 20 30 0.2", "int AOloopControl_setgainrange(long m0, long m1, float gainval)");

    RegisterCLIcommand("aolsetlimitr",__FILE__, AOloopControl_setlimitrange_cli, "set modal limits", "<modemin [long]> <modemax [long]> <limval>", "aolsetlimitr 20 30 0.02", "int AOloopControl_setlimitrange(long m0, long m1, float gainval)");

    RegisterCLIcommand("aolsetmultfr", __FILE__, AOloopControl_setmultfrange_cli, "set modal multf", "<modemin [long]> <modemax [long]> <multfval>", "aolsetmultfr 10 30 0.98", "int AOloopControl_setmultfrange(long m0, long m1, float multfval)");

    RegisterCLIcommand("aolsetgainb", __FILE__, AOloopControl_setgainblock_cli, "set modal gains by block", "<block [long]> <gainval>", "aolsetgainb 2 0.2", "int AOloopControl_setgainblock(long m0, long m1, float gainval)");

    RegisterCLIcommand("aolsetlimitb",__FILE__, AOloopControl_setlimitblock_cli, "set modal limits by block", "<block [long]> <limval>", "aolsetlimitb 2 0.02", "int AOloopControl_setlimitblock(long mb, float limitval)");

    RegisterCLIcommand("aolsetmultfb", __FILE__, AOloopControl_setmultfblock_cli, "set modal multf by block", "<block [long]> <multfval>", "aolsetmultfb 2 0.98", "int AOloopControl_setmultfblock(long mb, float multfval)");

    RegisterCLIcommand("aolscangainb", __FILE__, AOloopControl_scanGainBlock_cli, "scan gain for block", "<blockNB> <NBAOsteps> <gainstart> <gainend> <NBgainpts>", "aolscangainb", "int AOloopControl_scanGainBlock(long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain)");

    RegisterCLIcommand("aolmkwfsres", __FILE__, AOloopControl_computeWFSresidualimage_cli, "compute WFS residual real time", "<loopnb> <averaging coeff image>", "aolmkwfsres 2 coeffim", "long AOloopControl_computeWFSresidualimage(long loop, char *IDalpha_name)");



    RegisterCLIcommand("aolcompolm", __FILE__, AOloopControl_ComputeOpenLoopModes_cli, "compute open loop mode values", "<loop #>", "aolcompolm 2", "long AOloopControl_ComputeOpenLoopModes(long loop)");

    RegisterCLIcommand("aolautotunegains", __FILE__, AOloopControl_AutoTuneGains_cli, "compute optimal gains", "<loop #> <gain stream> <gaincoeff> <NBsamples>", "aolautotunegains 0 autogain 0.1 20000", "long AOloopControl_AutoTuneGains(long loop, const char *IDout_name, float GainCoeff, long NBsamples)");

    RegisterCLIcommand("aoldm2dmoffload", __FILE__, AOloopControl_dm2dm_offload_cli, "slow offload from dm to dm", "<streamin> <streamout> <timestep[sec]> <offloadcoeff> <multcoeff>", "aoldm2dmoffload dmin dmout 0.5 -0.01 0.999", "long AOloopControl_dm2dm_offload(const char *streamin, const char *streamout, float twait, float offcoeff, float multcoeff)");

    RegisterCLIcommand("aolautotune",  __FILE__, AOloopControl_AutoTune, "auto tuning of loop parameters", "no arg", "aolautotune", "int_fast8_t AOloopControl_AutoTune()");

    RegisterCLIcommand("aolset", __FILE__, AOloopControl_setparam_cli, "set parameter", "<parameter> <value>" , "aolset", "int AOloopControl_setparam(long loop, const char *key, double value)");

    RegisterCLIcommand("aoldmmodAB", __FILE__, AOloopControl_DMmodulateAB_cli, "module DM with linear combination of probes A and B", "<probeA> <probeB> <dmstream> <WFS resp mat> <WFS ref stream> <delay [sec]> <NB probes>", "aoldmmodAB probeA probeB wfsrespmat wfsref 0.1 6","int AOloopControl_DMmodulateAB(const char *IDprobeA_name, const char *IDprobeB_name, const char *IDdmstream_name, const char *IDrespmat_name, const char *IDwfsrefstream_name, double delay, long NBprobes)");


    // add atexit functions here
    // atexit((void*) myfunc); atexit = starts a function once the program exits (only if it is not a crash exit)
}



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. INITIALIZATION, configurations                                        */
/* =============================================================================================== */
/* =============================================================================================== */


/* =============================================================================================== */
/** @brief Read parameter value - float, char or int                                               */
/* =============================================================================================== */

/**
 * ## Purpose
 * 
 * Read parameter value (float) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	FLOAT
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
float AOloopControl_readParam_float(char *paramname, float defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	float value;
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        value = defaultValue;
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%50f", &value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			value = defaultValue;
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%f", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %f\n", paramname, value);
    
	
	return value;
}



/**
 * ## Purpose
 * 
 * Read parameter value (int) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	INT
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
int AOloopControl_readParam_int(char *paramname, int defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	int value;
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        value = defaultValue;
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%50d", &value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			value = defaultValue;
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%d", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %d\n", paramname, value);
    
	
	return value;
}



/**
 * ## Purpose
 * 
 * Read parameter value (char*) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	CHAR*
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
char* AOloopControl_readParam_string(char *paramname, char* defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	char* value = " ";
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        strcpy(value, defaultValue);
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%200s", value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			strcpy(value, defaultValue);
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%s", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %s\n", paramname, value);
    
	
	return value;
}





/* =============================================================================================== */
/** @brief Load / Setup configuration                                                              */
/* =============================================================================================== */

/**
 * ## Purpose
 * 
 * load / setup configuration - amazingly loooong function, I am proud of you Boss ! 
 *
 * ## Arguments
 * 
 * @param[in]
 * loop		INT
 * 			Loop number
 * 
 * @param[in]
 * mode		INT 
 * - 1 loads from ./conf/ directory to shared memory
 * - 0 simply connects to shared memory
 * 
 * @param[in]
 * level	INT
 * - 2 zonal only
 * - 10+ load all
 * 
 * 
 * 
 * @ingroup AOloopControl_streams
 */
int_fast8_t AOloopControl_loadconfigure(long loop, int mode, int level)
{
    FILE *fp;
    char content[201];
    char name[201];
    char fname[201];
    uint32_t *sizearray;
    int kw;
    long k;
    int r;
    int sizeOK;
    char command[501];
    int CreateSMim;
    long ii;
    long tmpl;
    char testdirname[201];

    int initwfsref;

    FILE *fplog; // human-readable log of load sequence


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


	// Create logfile for this function
	//
    if((fplog=fopen("logdir/loadconf.log", "w"))==NULL)
    {
        printf("ERROR: cannot create logdir/loadconf.log\n");
        exit(0);
    }
    loadcreateshm_log = 1;
    loadcreateshm_fplog = fplog;

	/** --- */
	/** # Details */
	
	/** ## 1. Initial setup from configuration files */


	/** - 1.1. Initialize memory */
	fprintf(fplog, "\n\n============== 1.1. Initialize memory ===================\n\n");
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(0);


	
	
	//
    /** ### 1.2. Set names of key streams */
    // Here we define names of key streams used by loop

	fprintf(fplog, "\n\n============== 1.2. Set names of key streams ===================\n\n");

	/** - dmC stream  : DM control */
    if(sprintf(name, "aol%ld_dmC", loop)<1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("DM control file name : %s\n", name);
    strcpy(AOconf[loop].dmCname, name);

	/** - dmdisp stream : total DM displacement */
	// used to notify dm combine that a new displacement should be computed
    if(sprintf(name, "aol%ld_dmdisp", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("DM displacement file name : %s\n", name);
    strcpy(AOconf[loop].dmdispname, name);

	/** - dmRM stream : response matrix */
    if(sprintf(name, "aol%ld_dmRM", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("DM RM file name : %s\n", name);
    strcpy(AOconf[loop].dmRMname, name);

	/** - wfsim : WFS image */
    if(sprintf(name, "aol%ld_wfsim", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("WFS file name: %s\n", name);
    strcpy(AOconf[loop].WFSname, name);



    // Modal control

	/** - DMmodes : control modes */
    if(sprintf(name, "aol%ld_DMmodes", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("DMmodes file name: %s\n", name);
    strcpy(AOconf[loop].DMmodesname, name);

	/** - respM : response matrix */
    if(sprintf(name, "aol%ld_respM", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("respM file name: %s\n", name);
    strcpy(AOconf[loop].respMname, name);

	/** - contrM : control matrix */
    if(sprintf(name, "aol%ld_contrM", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    printf("contrM file name: %s\n", name);
    strcpy(AOconf[loop].contrMname, name);




    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);


    /** ### 1.3. Read loop name
     * 
     * - ./conf/conf_LOOPNAME.txt -> AOconf[loop].name 
     */
	fprintf(fplog, "\n\n============== 1.3. Read loop name ===================\n\n");

    if((fp=fopen("./conf/conf_LOOPNAME.txt","r"))==NULL)
    {
        printf("ERROR: file ./conf/conf_LOOPNAME.txt missing\n");
        exit(0);
    }
    if(fscanf(fp, "%200s", content) != 1)
    {
        printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
		exit(0);
	}

    printf("loop name : %s\n", content);
    fflush(stdout);
    fprintf(fplog, "AOconf[%ld].name = %s\n", loop, AOconf[loop].name);
    fclose(fp);
    strcpy(AOconf[loop].name, content);


    /** ### 1.4. Define WFS image normalization mode 
     * 
     * - conf/param_WFSnorm.txt -> AOconf[loop].WFSnormalize
     */ 
    fprintf(fplog, "\n\n============== 1.4. Define WFS image normalization mode ===================\n\n");
    
    AOconf[loop].WFSnormalize = AOloopControl_readParam_int("WFSnorm", 1, fplog);
   


    /** ### 1.5. Read Timing info
     * 
     * - ./conf/param_loopfrequ.txt    -> AOconf[loop].loopfrequ
     * - ./conf/param_hardwlatency.txt -> AOconf[loop].hardwlatency
     * - AOconf[loop].hardwlatency_frame = AOconf[loop].hardwlatency * AOconf[loop].loopfrequ
     * - ./conf/param_complatency.txt  -> AOconf[loop].complatency
     * - AOconf[loop].complatency_frame = AOconf[loop].complatency * AOconf[loop].loopfrequ;
     * - ./conf/param_wfsmextrlatency.txt -> AOconf[loop].wfsmextrlatency
     */
     fprintf(fplog, "\n\n============== 1.5. Read Timing info ===================\n\n");
    
    
    
    AOconf[loop].loopfrequ = AOloopControl_readParam_float("loopfrequ", 1000.0, fplog);
	AOconf[loop].hardwlatency = AOloopControl_readParam_float("hardwlatency", 0.0, fplog);  
    AOconf[loop].hardwlatency_frame = AOconf[loop].hardwlatency * AOconf[loop].loopfrequ;

	AOconf[loop].complatency = AOloopControl_readParam_float("complatency", 0.0, fplog);
    AOconf[loop].complatency_frame = AOconf[loop].complatency * AOconf[loop].loopfrequ;

	AOconf[loop].wfsmextrlatency = AOloopControl_readParam_float("wfsmextrlatency", 0.0, fplog);
    AOconf[loop].wfsmextrlatency_frame = AOconf[loop].wfsmextrlatency * AOconf[loop].loopfrequ;



    /** ### 1.6. Define GPU use
     * 
     * - ./conf/param_GPU0.txt           > AOconf[loop].GPU0 (0 if missing)
     * - ./conf/param_GPU1.txt           > AOconf[loop].GPU1 (0 if missing)
     * - ./conf/param_GPUall.txt        -> AOconf[loop].GPUall
     * - ./conf/param_DMprimWriteON.txt -> AOconf[loop].DMprimaryWriteON
     * 
     */ 
	fprintf(fplog, "\n\n============== 1.6. Define GPU use ===================\n\n");
	
	AOconf[loop].GPU0 = AOloopControl_readParam_int("GPU0", 0, fplog);
	AOconf[loop].GPU1 = AOloopControl_readParam_int("GPU1", 0, fplog);
	AOconf[loop].GPUall = AOloopControl_readParam_int("GPUall", 0, fplog); // Skip CPU image scaling and go straight to GPUs ?
	AOconf[loop].DMprimaryWriteON = AOloopControl_readParam_int("DMprimaryWriteON", 0, fplog);    // Direct DM write ?
	AOconf[loop].DMfilteredWriteON = AOloopControl_readParam_int("DMfilteredWriteON", 0, fplog);    // Filtered DM write ?
    

	/** ### 1.7. WFS image total flux computation mode
	 * 
	 * 
	 */
	 fprintf(fplog, "\n\n============== 1.7. WFS image total flux computation mode ===================\n\n");

    // TOTAL image done in separate thread ?
    AOconf[loop].AOLCOMPUTE_TOTAL_ASYNC = AOloopControl_readParam_int("COMPUTE_TOTAL_ASYNC", 1, fplog);
 

    /** ### 1.8. Read CMatrix mult mode
     * 
     * - ./conf/param_CMMMODE.txt -> CMMODE
     * 		- 0 : WFS signal -> Mode coeffs -> DM act values  (2 sequential matrix multiplications)
     * 		- 1 : WFS signal -> DM act values  (1 combined matrix multiplication)
     */ 

 	fprintf(fplog, "\n\n============== 1.8. Read CMatrix mult mode ===================\n\n");

	AOconf[loop].CMMODE = AOloopControl_readParam_int("CMMODE", 1, fplog);



	/** ### 1.9. Setup loop timing array 
	 */
	fprintf(fplog, "\n\n============== 1.9. Setup loop timing array ===================\n\n");
	// LOOPiteration is written in cnt1 
    if(sprintf(name, "aol%ld_looptiming", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOcontrolNBtimers, 1, 0.0);


	/** ## 2. Read/load shared memory arrays
	 * 
	 */ 
	fprintf(fplog, "\n\n============== 2. Read/load shared memory arrays ===================\n\n");

    /**
     * ### 2.1. CONNECT to existing streams
     * 
     * Note: these streams MUST exist
     * 
     *  - AOconf[loop].dmdispname  : this image is read to notify when new dm displacement is ready
     *  - AOconf[loop].WFSname     : connect to WFS camera. This is where the size of the WFS is read 
     */
     
     fprintf(fplog, "\n\n============== 2.1. CONNECT to existing streams  ===================\n\n");
     
    aoconfID_dmdisp = read_sharedmem_image(AOconf[loop].dmdispname);
    if(aoconfID_dmdisp==-1)
        fprintf(fplog, "ERROR : cannot read shared memory stream %s\n", AOconf[loop].dmdispname);
    else
        fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].dmdispname, aoconfID_dmdisp);

 
    aoconfID_wfsim = read_sharedmem_image(AOconf[loop].WFSname);
    if(aoconfID_wfsim == -1)
        fprintf(fplog, "ERROR : cannot read shared memory stream %s\n", AOconf[loop].WFSname);
    else
        fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].WFSname, aoconfID_wfsim);

    AOconf[loop].sizexWFS = data.image[aoconfID_wfsim].md[0].size[0];
    AOconf[loop].sizeyWFS = data.image[aoconfID_wfsim].md[0].size[1];
    AOconf[loop].sizeWFS = AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS;

    fprintf(fplog, "WFS stream size = %ld x %ld\n", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS);


    /**
     * 
     * ### 2.2. Read file to stream or connect to existing stream
     * 
     *  The AOloopControl_xDloadcreate_shmim functions are used, and follows these rules:
     * 
     * If file already loaded, use it (we assume it's already been properly loaded) \n
     * If not, attempt to read it from shared memory \n
     * If not available in shared memory, create it in shared memory \n
     * if "fname" exists, attempt to load it into the shared memory image
     *
     * Stream names are fixed: 
     * - aol_wfsdark
     * - aol_imWFS0
     * - aol_imWFS0tot
     * - aol_imWFS1
     * - aol_imWFS2
     * - aol_wfsref0
     * - aol_wfsref
     */
     
     
	fprintf(fplog, "\n\n============== 2.2. Read file to stream or connect to existing stream  ===================\n\n");

    if(sprintf(name, "aol%ld_wfsdark", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    sprintf(fname, "./conf/shmim_wfsdark.fits");
    aoconfID_wfsdark = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);



    if(sprintf(name, "aol%ld_imWFS0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoconfID_imWFS0 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(sprintf(name, "aol%ld_imWFS0tot", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoconfID_imWFS0tot = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", 1, 1, 0.0);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(sprintf(name, "aol%ld_imWFS1", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoconfID_imWFS1 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);

    if(sprintf(name, "aol%ld_imWFS2", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoconfID_imWFS2 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);




    initwfsref = AOconf[loop].init_wfsref0;

    if(sprintf(name, "aol%ld_wfsref0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(fname, "./conf/shmim_wfsref0.fits") < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoconfID_wfsref0 = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);
    AOconf[loop].init_wfsref0 = 1;

    if(sprintf(name, "aol%ld_wfsref", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(fname, "./conf/shmim_wfsref.fits") < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoconfID_wfsref = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);

    if(initwfsref==0)
    {
        char name1[200];

        if(sprintf(name1, "aol%ld_wfsref0", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        copy_image_ID(name1, name, 1);
    }




    /** ### 2.3. Connect to DM
     * 
     * - AOconf[loop].dmCname : DM control channel
     * 
     *  Here the DM size is read -> Oconf[loop].sizexDM, AOconf[loop].sizeyDM
     */


	AOconf[loop].DMMODE = AOloopControl_readParam_int("DMMODE", 0, fplog); // zonal DM by default

    aoconfID_dmC = image_ID(AOconf[loop].dmCname);
    if(aoconfID_dmC==-1)
    {
        printf("connect to %s\n", AOconf[loop].dmCname);
        aoconfID_dmC = read_sharedmem_image(AOconf[loop].dmCname);
        if(aoconfID_dmC==-1)
        {
            printf("ERROR: cannot connect to shared memory %s\n", AOconf[loop].dmCname);
            exit(0);
        }
    }
    AOconf[loop].sizexDM = data.image[aoconfID_dmC].md[0].size[0];
    AOconf[loop].sizeyDM = data.image[aoconfID_dmC].md[0].size[1];
    AOconf[loop].sizeDM = AOconf[loop].sizexDM*AOconf[loop].sizeyDM;

    fprintf(fplog, "Connected to DM %s, size = %ld x %ld\n", AOconf[loop].dmCname, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);



	/**
	 * - AOconf[loop].dmRMname : DM response matrix channel
	 * 
	 */
    aoconfID_dmRM = image_ID(AOconf[loop].dmRMname);
    if(aoconfID_dmRM==-1)
    {
        printf("connect to %s\n", AOconf[loop].dmRMname);
        aoconfID_dmRM = read_sharedmem_image(AOconf[loop].dmRMname);
        if(aoconfID_dmRM==-1)
        {
            printf("ERROR: cannot connect to shared memory %s\n", AOconf[loop].dmRMname);
            exit(0);
        }
    }
    fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].dmRMname, aoconfID_dmRM);



	/// Connect to DM modes shared mem
	///  continue if not successful
	///
	aoconfID_DMmodes = image_ID(AOconf[loop].DMmodesname);
	if(aoconfID_DMmodes==-1)
    {
        printf("connect to %s\n", AOconf[loop].DMmodesname);
        aoconfID_DMmodes = read_sharedmem_image(AOconf[loop].DMmodesname);
        if(aoconfID_DMmodes==-1)
        {
            printf("WARNING: cannot connect to shared memory %s\n", AOconf[loop].DMmodesname);
//			exit(0);
        }
    }
	if(aoconfID_DMmodes!=-1)
	{
		fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].DMmodesname, aoconfID_DMmodes);
		AOconf[loop].NBDMmodes = data.image[aoconfID_DMmodes].md[0].size[2];
		printf("NBmodes = %ld\n", AOconf[loop].NBDMmodes);
	}
	

	/** 
	 * ## 3. Load DM modes (if level >= 10)
	 * 
	 * */

	fprintf(fplog, "\n\n============== 3. Load DM modes (if level >= 10)  ===================\n\n");

	
    if(level>=10) // Load DM modes (will exit if not successful)
    {				
		/** 
		 * Load AOconf[loop].DMmodesname \n
		 * if already exists in local memory, trust it and adopt it \n
		 * if not, load from ./conf/shmim_DMmodes.fits \n
		 * 
		 */
		
        aoconfID_DMmodes = image_ID(AOconf[loop].DMmodesname); 

        if(aoconfID_DMmodes == -1) // If not, check file
        {
            long ID1tmp, ID2tmp;
            int vOK;

			

            if(sprintf(fname, "./conf/shmim_DMmodes.fits") < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            printf("Checking file \"%s\"\n", fname);

            // GET SIZE FROM FILE
            ID1tmp = load_fits(fname, "tmp3Dim", 1);
            if(ID1tmp==-1)
            {
                printf("WARNING: no file \"%s\" -> loading zonal modes\n", fname);

                if(sprintf(fname, "./conf/shmim_DMmodes_zonal.fits") <1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                ID1tmp = load_fits(fname, "tmp3Dim", 1);
                if(ID1tmp==-1)
                {
                    printf("ERROR: cannot read zonal modes \"%s\"\n", fname);
                    exit(0);
                }
            }


            // check size
            if(data.image[ID1tmp].md[0].naxis != 3)
            {
                printf("ERROR: File \"%s\" is not a 3D image (cube)\n", fname);
                exit(0);
            }
            if(data.image[ID1tmp].md[0].size[0] != AOconf[loop].sizexDM)
            {
                printf("ERROR: File \"%s\" has wrong x size: should be %ld, is %ld\n", fname, AOconf[loop].sizexDM, (long) data.image[ID1tmp].md[0].size[0]);
                exit(0);
            }
            if(data.image[ID1tmp].md[0].size[1] != AOconf[loop].sizeyDM)
            {
                printf("ERROR: File \"%s\" has wrong y size: should be %ld, is %ld\n", fname, AOconf[loop].sizeyDM, (long) data.image[ID1tmp].md[0].size[1]);
                exit(0);
            }
            AOconf[loop].NBDMmodes = data.image[ID1tmp].md[0].size[2];

            printf("NUMBER OF MODES = %ld\n", AOconf[loop].NBDMmodes);

            // try to read it from shared memory
            ID2tmp = read_sharedmem_image(AOconf[loop].DMmodesname);
            vOK = 0;
            if(ID2tmp != -1) // if shared memory exists, check its size
            {
                vOK = 1;
                if(data.image[ID2tmp].md[0].naxis != 3)
                {
                    printf("ERROR: Shared memory File %s is not a 3D image (cube)\n", AOconf[loop].DMmodesname);
                    vOK = 0;
                }
                if(data.image[ID2tmp].md[0].size[0] != AOconf[loop].sizexDM)
                {
                    printf("ERROR: Shared memory File %s has wrong x size: should be %ld, is %ld\n", AOconf[loop].DMmodesname, AOconf[loop].sizexDM, (long) data.image[ID2tmp].md[0].size[0]);
                    vOK = 0;
                }
                if(data.image[ID2tmp].md[0].size[1] != AOconf[loop].sizeyDM)
                {
                    printf("ERROR: Shared memory File %s has wrong y size: should be %ld, is %ld\n", AOconf[loop].DMmodesname, AOconf[loop].sizeyDM, (long) data.image[ID2tmp].md[0].size[1]);
                    vOK = 0;
                }
                if(data.image[ID2tmp].md[0].size[2] != AOconf[loop].NBDMmodes)
                {
                    printf("ERROR: Shared memory File %s has wrong y size: should be %ld, is %ld\n", AOconf[loop].DMmodesname, AOconf[loop].NBDMmodes, (long) data.image[ID2tmp].md[0].size[2]);
                    vOK = 0;
                }

                if(vOK==1) // if size is OK, adopt it
                    aoconfID_DMmodes = ID2tmp;
                else // if not, erase shared memory
                {
                    printf("SHARED MEM IMAGE HAS WRONG SIZE -> erasing it\n");
                    delete_image_ID(AOconf[loop].DMmodesname);
                }
            }


            if(vOK==0) // create shared memory
            {

                sizearray[0] = AOconf[loop].sizexDM;
                sizearray[1] = AOconf[loop].sizeyDM;
                sizearray[2] = AOconf[loop].NBDMmodes;
                printf("Creating %s   [%ld x %ld x %ld]\n", AOconf[loop].DMmodesname, (long) sizearray[0], (long) sizearray[1], (long) sizearray[2]);
                fflush(stdout);
                aoconfID_DMmodes = create_image_ID(AOconf[loop].DMmodesname, 3, sizearray, _DATATYPE_FLOAT, 1, 0);
            }

            // put modes into shared memory

            switch (data.image[ID1tmp].md[0].atype) {
            case _DATATYPE_FLOAT :
                memcpy(data.image[aoconfID_DMmodes].array.F, data.image[ID1tmp].array.F, sizeof(float)*AOconf[loop].sizexDM*AOconf[loop].sizeyDM*AOconf[loop].NBDMmodes);
                break;
            case _DATATYPE_DOUBLE :
                for(ii=0; ii<AOconf[loop].sizexDM*AOconf[loop].sizeyDM*AOconf[loop].NBDMmodes; ii++)
                    data.image[aoconfID_DMmodes].array.F[ii] = data.image[ID1tmp].array.D[ii];
                break;
            default :
                printf("ERROR: TYPE NOT RECOGNIZED FOR MODES\n");
                exit(0);
                break;
            }

            delete_image_ID("tmp3Dim");
        }

        fprintf(fplog, "stream %s loaded as ID = %ld, size %ld %ld %ld\n", AOconf[loop].DMmodesname, aoconfID_DMmodes, AOconf[loop].sizexDM, AOconf[loop].sizeyDM, AOconf[loop].NBDMmodes);
    }



    // TO BE CHECKED

    // AOconf[loop].NBMblocks = AOconf[loop].DMmodesNBblock;
    // printf("NBMblocks : %ld\n", AOconf[loop].NBMblocks);
    // fflush(stdout);


    AOconf[loop].AveStats_NBpt = 100;
    for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
    {
        AOconf[loop].block_OLrms[k] = 0.0;
        AOconf[loop].block_Crms[k] = 0.0;
        AOconf[loop].block_WFSrms[k] = 0.0;
        AOconf[loop].block_limFrac[k] = 0.0;

        AOconf[loop].blockave_OLrms[k] = 0.0;
        AOconf[loop].blockave_Crms[k] = 0.0;
        AOconf[loop].blockave_WFSrms[k] = 0.0;
        AOconf[loop].blockave_limFrac[k] = 0.0;
    }

    printf("%ld modes\n", AOconf[loop].NBDMmodes);


    if(level>=10)
    {
        long ID;

        // Load/create modal command vector memory
        if(sprintf(name, "aol%ld_DMmode_cmd", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_cmd_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);


        if(sprintf(name, "aol%ld_DMmode_meas", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_meas_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_AVE", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_AVE_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_RMS", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_RMS_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_GAIN", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_DMmode_GAIN = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);

        if(sprintf(name, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_LIMIT_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);

        if(sprintf(name, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_MULTF_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);


        if(sprintf(name, "aol%ld_wfsmask", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        sprintf(fname, "conf/%s.fits", name);
        aoconfID_wfsmask = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 1.0);
        AOconf[loop].activeWFScnt = 0;
        for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS; ii++)
            if(data.image[aoconfID_wfsmask].array.F[ii]>0.5)
                AOconf[loop].activeWFScnt++;

        if(sprintf(name, "aol%ld_dmmask", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/%s.fits", name) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_dmmask = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexDM, AOconf[loop].sizeyDM, 1.0);
        AOconf[loop].activeDMcnt = 0;
        for(ii=0; ii<AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
            if(data.image[aoconfID_dmmask].array.F[ii]>0.5)
                AOconf[loop].activeDMcnt++;

        printf(" AOconf[loop].activeWFScnt = %ld\n", AOconf[loop].activeWFScnt );
        printf(" AOconf[loop].activeDMcnt = %ld\n", AOconf[loop].activeDMcnt );


        AOconf[loop].init_RM = 0;
        if(sprintf(fname, "conf/shmim_respM.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_respM = AOloopControl_IOtools_3Dloadcreate_shmim(AOconf[loop].respMname, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBDMmodes, 0.0);
        AOconf[loop].init_RM = 1;


        AOconf[loop].init_CM = 0;
        if(sprintf(fname, "conf/shmim_contrM.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_contrM = AOloopControl_IOtools_3Dloadcreate_shmim(AOconf[loop].contrMname, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBDMmodes, 0.0);
        AOconf[loop].init_CM = 1;

        if((fp=fopen("conf/param_NBmodeblocks.txt", "r"))==NULL)
        {
            printf("Cannot open conf/param_NBmodeblocks.txt.... assuming 1 block\n");
            AOconf[loop].DMmodesNBblock = 1;
        }
        else
        {
            if(fscanf(fp, "%50ld", &tmpl) == 1)
                AOconf[loop].DMmodesNBblock = tmpl;
            else
            {
                printf("Cannot read conf/param_NBmodeblocks.txt.... assuming 1 block\n");
                AOconf[loop].DMmodesNBblock = 1;
            }
            fclose(fp);
        }


        if(sprintf(name, "aol%ld_contrMc", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_contrMc.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_contrMc = AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].sizeDM, 0.0);

        if(sprintf(name, "aol%ld_contrMcact", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_contrMcact_00.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_contrMcact[0] = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt, 0.0);



        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_gainb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_gainb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);

		if(sprintf(name, "aol%ld_modeARPFgainAuto", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_modeARPFgainAuto.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_modeARPFgainAuto = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].NBDMmodes, 1, 1.0);


        if(sprintf(name, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_multfb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_multfb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);

        if(sprintf(name, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_limitb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_limitb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);


#ifdef _PRINT_TEST
        printf("TEST - INITIALIZE contrMc, contrMcact\n");
        fflush(stdout);
#endif


        uint_fast16_t kk;
        int mstart = 0;
        
        for(kk=0; kk<AOconf[loop].DMmodesNBblock; kk++)
        {
            long ID;

#ifdef _PRINT_TEST
            printf("TEST - BLOCK %3ld gain = %f\n", kk, data.image[aoconfID_gainb].array.F[kk]);
            fflush(stdout);
#endif

            if(sprintf(name, "aol%ld_DMmodes%02ld", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            printf("FILE = %s\n", fname);
            printf("====== LOADING %s to %s\n", fname, name);
            fflush(stdout);
            if((ID=AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexDM, AOconf[loop].sizeyDM, 0, 0.0))!=-1)
                AOconf[loop].NBmodes_block[kk] = data.image[ID].md[0].size[2];

			int m;
			for(m=mstart; m<(mstart+AOconf[loop].NBmodes_block[kk]); m++)
				AOconf[loop].modeBlockIndex[m] = kk;
			mstart += AOconf[loop].NBmodes_block[kk];


            if(sprintf(name, "aol%ld_respM%02ld", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            printf("====== LOADING %s to %s\n", fname, name);
            fflush(stdout);
            AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBmodes_block[kk], 0.0);


            if(sprintf(name, "aol%ld_contrM%02ld", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            printf("====== LOADING %s to %s\n", fname, name);
            fflush(stdout);
            AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBmodes_block[kk], 0.0);


            if(sprintf(name, "aol%ld_contrMc%02ld", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            ID = AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].sizexDM*AOconf[loop].sizeyDM, 0.0);
            if(kk==0)
                for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
                    data.image[aoconfID_contrMc].array.F[ii] = 0.0;
            for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
                data.image[aoconfID_contrMc].array.F[ii] += data.image[aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];


            if(sprintf(name, "aol%ld_contrMcact%02ld_00", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            //   sprintf(fname, "conf/shmim_contrMcact%02ld_00", kk);
            printf("====== LOADING %s to %s  size %ld %ld\n", fname, name,  AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt);
            ID = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt, 0.0);

            if(kk==0)
                for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                    data.image[aoconfID_contrMcact[0]].array.F[ii] = 0.0;

            for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                data.image[aoconfID_contrMcact[0]].array.F[ii] += data.image[aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];

        }
    }
    free(sizearray);



    if(AOconf[loop].DMmodesNBblock==1)
        AOconf[loop].indexmaxMB[0] = AOconf[loop].NBDMmodes;
    else
    {
        AOconf[loop].indexmaxMB[0] = AOconf[loop].NBmodes_block[0];
        for(k=1; k<AOconf[loop].DMmodesNBblock; k++)
            AOconf[loop].indexmaxMB[k] = AOconf[loop].indexmaxMB[k-1] + AOconf[loop].NBmodes_block[k];
    }

    if(sprintf(fname, "./conf/param_blockoffset_%02ld.txt", (long) 0) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    fp = fopen(fname, "w");
    fprintf(fp, "   0\n");
    fprintf(fp, "%4ld\n", AOconf[loop].NBmodes_block[0]);
    fclose(fp);
    for(k=1; k<AOconf[loop].DMmodesNBblock; k++)
    {
        if(sprintf(fname, "./conf/param_blockoffset_%02ld.txt", k) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        fp = fopen(fname, "w");
        fprintf(fp, "%4ld\n", AOconf[loop].indexmaxMB[k-1]);
        fprintf(fp, "%4ld\n", AOconf[loop].NBmodes_block[k]);
        fclose(fp);
    }



    list_image_ID();
    printf(" AOconf[loop].activeWFScnt = %ld\n", AOconf[loop].activeWFScnt );
    printf(" AOconf[loop].activeDMcnt = %ld\n", AOconf[loop].activeDMcnt );
    printf("   init_WFSref0    %d\n", AOconf[loop].init_wfsref0);
    printf("   init_RM        %d\n", AOconf[loop].init_RM);
    printf("   init_CM        %d\n", AOconf[loop].init_CM);


    AOconf[loop].init = 1;

    loadcreateshm_log = 0;
    fclose(fplog);

#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    return(0);
}


/* ================== END HUGE FUNCTON =========================================================== */




/**
 * ## Purpose
 * 
 * Initialize memory of the loop  
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	int
 * 				value of the mode
 * 
 *
 *  
 */

/***  */

int_fast8_t AOloopControl_InitializeMemory(int mode)
{
    int SM_fd;
    struct stat file_stat;
    int create = 0;
    long loop;
    int tmpi;
	char imname[200];


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    loop = LOOPNUMBER;



    SM_fd = open(AOconfname, O_RDWR);
    if(SM_fd==-1)
    {
        printf("Cannot import file \"%s\" -> creating file\n", AOconfname);
        create = 1;
    }
    else
    {
        fstat(SM_fd, &file_stat);
        printf("File %s size: %zd\n", AOconfname, file_stat.st_size);
        if(file_stat.st_size!=sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol)
        {
            printf("File \"%s\" size is wrong -> recreating file\n", AOconfname);
            create = 1;
            close(SM_fd);
        }
    }

    if(create==1)
    {
        int result;

        SM_fd = open(AOconfname, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);

        if (SM_fd == -1) {
            perror("Error opening file for writing");
            exit(0);
        }

        result = lseek(SM_fd, sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol-1, SEEK_SET);
        if (result == -1) {
            close(SM_fd);
            perror("Error calling lseek() to 'stretch' the file");
            exit(0);
        }

        result = write(SM_fd, "", 1);
        if (result != 1) {
            close(SM_fd);
            perror("Error writing last byte of the file");
            exit(0);
        }
    }


    AOconf = (AOLOOPCONTROL_CONF*) mmap(0, sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol, PROT_READ | PROT_WRITE, MAP_SHARED, SM_fd, 0);
    if (AOconf == MAP_FAILED) {
        close(SM_fd);
        perror("Error mmapping the file");
        exit(0);
    }


    if((mode==0)||(create==1))
    {
        char cntname[200];

        AOconf[loop].on = 0;
        AOconf[loop].DMprimaryWriteON = 0;
        AOconf[loop].DMfilteredWriteON = 0;
        AOconf[loop].AUTOTUNE_LIMITS_ON = 0;
        AOconf[loop].AUTOTUNE_GAINS_ON = 0;
        AOconf[loop].ARPFon = 0;
        AOconf[loop].ARPFgainAutoMin = 0.99;
        AOconf[loop].ARPFgainAutoMax = 1.01;
        AOconf[loop].LOOPiteration = 0;
        AOconf[loop].cnt = 0;
        AOconf[loop].cntmax = 0;
        AOconf[loop].init_CMc = 0;

        if(sprintf(cntname, "aol%ld_logdata", loop) < 1) // contains loop count (cnt0) and loop gain
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if((aoconfIDlogdata = image_ID(cntname))==-1)
        {
            uint32_t *sizearray;
            sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
            sizearray[0] = 1;
            sizearray[1] = 1;
            aoconfIDlogdata = create_image_ID(cntname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            free(sizearray);
        }
    }


    if(create==1)
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
        {
            AOconf[loop].init = 0;
            AOconf[loop].on = 0;
            AOconf[loop].DMprimaryWriteON = 0;
            AOconf[loop].DMfilteredWriteON = 0;
            AOconf[loop].ARPFon = 0;
            AOconf[loop].LOOPiteration = 0;
            AOconf[loop].cnt = 0;
            AOconf[loop].cntmax = 0;
            AOconf[loop].maxlimit = 0.3;
            AOconf[loop].mult = 1.00;
            AOconf[loop].gain = 0.0;
            AOconf[loop].AUTOTUNE_LIMITS_perc = 1.0; // percentile threshold
            AOconf[loop].AUTOTUNE_LIMITS_mcoeff = 1.0; // multiplicative coeff
            AOconf[loop].AUTOTUNE_LIMITS_delta = 1.0e-3;
            AOconf[loop].ARPFgain = 0.0;
			AOconf[loop].ARPFgainAutoMin = 0.99;
			AOconf[loop].ARPFgainAutoMax = 1.01;
            AOconf[loop].WFSnormfloor = 0.0;
            AOconf[loop].framesAve = 1;
            AOconf[loop].DMmodesNBblock = 1;
            AOconf[loop].GPUusesem = 1;

            AOconf[loop].loopfrequ = 2000.0;
            AOconf[loop].hardwlatency = 0.0011;
            AOconf[loop].hardwlatency_frame = 2.2;
            AOconf[loop].complatency = 0.0001;
            AOconf[loop].complatency_frame = 0.2;
            AOconf[loop].wfsmextrlatency = 0.0003;
            AOconf[loop].wfsmextrlatency_frame = 0.6;
        }
    }
    else
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
            if(AOconf[loop].init == 1)
            {
                printf("LIST OF ACTIVE LOOPS:\n");
                printf("----- Loop %ld   (%s) ----------\n", loop, AOconf[loop].name);
                printf("  WFS:  %s  [%ld]  %ld x %ld\n", AOconf[loop].WFSname, aoconfID_wfsim, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS);
                printf("   DM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmCname, aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
                printf("DM RM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmRMname, aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
            }
    }

    if(AOloopcontrol_meminit==0)
    {

        printf("INITIALIZING GPUset ARRAYS\n");
        fflush(stdout);

        GPUset0 = (int*) malloc(sizeof(int)*GPUcntMax);

        uint_fast16_t k;

        for(k=0; k<GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_GPUset0dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                GPUset0[k] = tmpi;
            }
            else
                GPUset0[k] = k;
        }


        GPUset1 = (int*) malloc(sizeof(int)*GPUcntMax);
        for(k=0; k<GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_GPUset1dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                GPUset1[k] = tmpi;
            }
            else
                GPUset1[k] = k;
        }
    }

    AOloopcontrol_meminit = 1;


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    return 0;
}




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 6. REAL TIME COMPUTING ROUTINES                                          */
/* =============================================================================================== */
/* =============================================================================================== */



// zero point offset loop
//
// args:
//  DM offset channel (shared memory)
//  zonal resp matrix (shared memory)
//  nominal wfs reference without offset (shared memory)
//  wfs reference to be updated (shared memory)
//
// computation triggered on semaphore wait on semaphore #1 of DM offset
//
// will run until SIGUSR1 received
//
// read LOOPiteration from shared memory stream "aol#_LOOPiteration" if available 
//
int_fast8_t AOloopControl_WFSzpupdate_loop(const char *IDzpdm_name, const char *IDzrespM_name, const char *IDwfszp_name)
{
    long IDzpdm, IDzrespM, IDwfszp;
    uint32_t dmxsize, dmysize, dmxysize;
    long wfsxsize, wfsysize, wfsxysize;
    long IDtmp;
    long elem, act;
    long zpcnt = 0;
    long zpcnt0;
    int semval;
    struct timespec t1;
    struct timespec t2;

	char imname[200];
	

	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}

    IDzpdm = image_ID(IDzpdm_name);

    if(data.image[IDzpdm].md[0].sem<2) // if semaphore #1 does not exist, create it
        COREMOD_MEMORY_image_set_createsem(IDzpdm_name, 2);


    IDzrespM = image_ID(IDzrespM_name);
    IDwfszp = image_ID(IDwfszp_name);


    // array sizes extracted from IDzpdm and IDwfsref

    dmxsize = data.image[IDzpdm].md[0].size[0];
    dmysize = data.image[IDzpdm].md[0].size[1];
    dmxysize = dmxsize*dmysize;
    wfsxsize = data.image[IDwfszp].md[0].size[0];
    wfsysize = data.image[IDwfszp].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;

    // VERIFY SIZES

    // verify zrespM
    if(data.image[IDzrespM].md[0].size[0]!=wfsxsize)
    {
        printf("ERROR: zrespM xsize %ld does not match wfsxsize %ld\n", (long) data.image[IDzrespM].md[0].size[0], (long) wfsxsize);
        exit(0);
    }
    if(data.image[IDzrespM].md[0].size[1]!=wfsysize)
    {
        printf("ERROR: zrespM ysize %ld does not match wfsysize %ld\n", (long) data.image[IDzrespM].md[0].size[1], (long) wfsysize);
        exit(0);
    }
    if(data.image[IDzrespM].md[0].size[2]!=dmxysize)
    {
        printf("ERROR: zrespM zsize %ld does not match wfsxysize %ld\n", (long) data.image[IDzrespM].md[0].size[1], (long) wfsxysize);
        exit(0);
    }


    IDtmp = create_2Dimage_ID("wfsrefoffset", wfsxsize, wfsysize);


    zpcnt0 = 0;

    if(data.image[IDzpdm].md[0].sem > 1) // drive semaphore #1 to zero
        while(sem_trywait(data.image[IDzpdm].semptr[1])==0) {}
    else
    {
        printf("ERROR: semaphore #1 missing from image %s\n", IDzpdm_name);
        exit(0);
    }

    while(data.signal_USR1==0)
    {
        memset(data.image[IDtmp].array.F, '\0', sizeof(float)*wfsxysize);

        while(zpcnt0 == data.image[IDzpdm].md[0].cnt0)
            usleep(10);

        zpcnt0 = data.image[IDzpdm].md[0].cnt0;

        // TO BE DONE
        //  sem_wait(data.image[IDzpdm].semptr[1]);


        printf("WFS zero point offset update  # %8ld       (%s -> %s)  ", zpcnt, data.image[IDzpdm].name, data.image[IDwfszp].name);
        fflush(stdout);


        clock_gettime(CLOCK_REALTIME, &t1);

# ifdef _OPENMP
        #pragma omp parallel for private(elem)
# endif
        for(act=0; act<dmxysize; act++)
            for(elem=0; elem<wfsxysize; elem++)
                data.image[IDtmp].array.F[elem] += data.image[IDzpdm].array.F[act]*data.image[IDzrespM].array.F[act*wfsxysize+elem];


        clock_gettime(CLOCK_REALTIME, &t2);
        tdiff = info_time_diff(t1, t2);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;

        printf(" [ %10.3f ms]\n", 1e3*tdiffv);
        fflush(stdout);


        // copy results to IDwfszpo
        data.image[IDwfszp].md[0].write = 1;
        memcpy(data.image[IDwfszp].array.F, data.image[IDtmp].array.F, sizeof(float)*wfsxysize);
        COREMOD_MEMORY_image_set_sempost_byID(IDwfszp, -1);
        data.image[IDwfszp].md[0].cnt0 ++;
        data.image[IDwfszp].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDwfszp].md[0].write = 0;

        zpcnt++;
    }

    return 0;
}




//
// Create zero point WFS channels
// watch semaphore 1 on output (IDwfsref_name) -> sum all channels to update WFS zero point
// runs in separate process from RT computation
//
//
//
int_fast8_t AOloopControl_WFSzeropoint_sum_update_loop(long loopnb, const char *ID_WFSzp_name, int NBzp, const char *IDwfsref0_name, const char *IDwfsref_name)
{
    long wfsxsize, wfsysize, wfsxysize;
    long IDwfsref, IDwfsref0;
    long *IDwfszparray;
    long cntsumold;
    int RT_priority = 95; //any number from 0-99
    struct sched_param schedpar;
    long nsecwait = 10000; // 10 us
    struct timespec semwaitts;
    long ch;
    long IDtmp;
    long ii;
    char imname[200];
    int semval;



	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    if(seteuid(data.euid) != 0) // This goes up to maximum privileges
        printERROR(__FILE__, __func__, __LINE__, "seteuid() returns non-zero value");

    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster

    if(seteuid(data.ruid) != 0) // Go back to normal privileges
       printERROR(__FILE__, __func__, __LINE__, "seteuid() returns non-zero value");
#endif

    IDwfsref = image_ID(IDwfsref_name);
    wfsxsize = data.image[IDwfsref].md[0].size[0];
    wfsysize = data.image[IDwfsref].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;
    IDtmp = create_2Dimage_ID("wfsrefoffset", wfsxsize, wfsysize);
    IDwfsref0 = image_ID(IDwfsref0_name);


    if(data.image[IDwfsref].md[0].sem > 1) // drive semaphore #1 to zero
        while(sem_trywait(data.image[IDwfsref].semptr[1])==0) {}
    else
    {
        printf("ERROR: semaphore #1 missing from image %s\n", IDwfsref_name);
        exit(0);
    }

    IDwfszparray = (long*) malloc(sizeof(long)*NBzp);
    // create / read the zero point WFS channels
    for(ch=0; ch<NBzp; ch++)
    {
        if(sprintf(imname, "%s%ld", ID_WFSzp_name, ch) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        AOloopControl_IOtools_2Dloadcreate_shmim(imname, "", wfsxsize, wfsysize, 0.0);
        COREMOD_MEMORY_image_set_createsem(imname, 10);
        IDwfszparray[ch] = image_ID(imname);
    }

    cntsumold = 0;
    for(;;)
    {
        if (clock_gettime(CLOCK_REALTIME, &semwaitts) == -1) {
            perror("clock_gettime");
            exit(EXIT_FAILURE);
        }
        semwaitts.tv_nsec += nsecwait;
        if(semwaitts.tv_nsec >= 1000000000)
            semwaitts.tv_sec = semwaitts.tv_sec + 1;

        sem_timedwait(data.image[IDwfsref].semptr[1], &semwaitts);

        long cntsum = 0;
        for(ch=0; ch<NBzp; ch++)
            cntsum += data.image[IDwfszparray[ch]].md[0].cnt0;


        if(cntsum != cntsumold)
        {
			// copy wfsref0 to tmp
            memcpy(data.image[IDtmp].array.F, data.image[IDwfsref0].array.F, sizeof(float)*wfsxysize);

            for(ch=0; ch<NBzp; ch++)
                for(ii=0; ii<wfsxysize; ii++)
                    data.image[IDtmp].array.F[ii] += data.image[IDwfszparray[ch]].array.F[ii];

            // copy results to IDwfsref
            data.image[IDwfsref].md[0].write = 1;
            memcpy(data.image[IDwfsref].array.F, data.image[IDtmp].array.F, sizeof(float)*wfsxysize);
            data.image[IDwfsref].md[0].cnt0 ++;
            data.image[IDwfsref].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
            data.image[IDwfsref].md[0].write = 0;

/*            sem_getvalue(data.image[IDwfsref].semptr[0], &semval); // do not update sem 1
            if(semval<SEMAPHORE_MAXVAL)
                COREMOD_MEMORY_image_set_sempost(IDwfsref_name, 0);*/
            COREMOD_MEMORY_image_set_sempost_excl_byID(IDwfsref, 1);
            

            cntsumold = cntsum;
        }
    }

    free(IDwfszparray);


    return(0);
}






/**
 * ## Purpose
 * 
 * Main AO loop function
 * 
 * 
 * ## Details
 * 
 */ 
int_fast8_t AOloopControl_run()
{
    FILE *fp;
    char fname[200];
    long loop;
    int vOK;
    long ii;
    long ID;
    long j, m;
    struct tm *uttime;
    time_t t;
    struct timespec *thetime = (struct timespec *)malloc(sizeof(struct timespec));
    char logfname[1000];
    char command[1000];
    int r;
    int RT_priority = 90; //any number from 0-99
    struct sched_param schedpar;
    double a;
    long cnttest;
    float tmpf1;


    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    int semval;



    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    r = seteuid(data.euid); //This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid);//Go back to normal privileges
#endif


    loop = LOOPNUMBER;


    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(0);


	/** ### STEP 1: Setting up 
	 * 
	 * Load arrays
	 * */
    printf("SETTING UP...\n");
    AOloopControl_loadconfigure(LOOPNUMBER, 1, 10);

	
    // pixel streaming ?
    COMPUTE_PIXELSTREAMING = 1;

    if(AOconf[loop].GPUall == 0)
        COMPUTE_PIXELSTREAMING = 0;


    printf("============ pixel streaming ? =============\n");
    fflush(stdout);

    if(COMPUTE_PIXELSTREAMING == 1)
        aoconfID_pixstream_wfspixindex = load_fits("pixstream_wfspixindex.fits", "pixstream", 1);

    if(aoconfID_pixstream_wfspixindex == -1)
        COMPUTE_PIXELSTREAMING = 0;
    else
    {
        printf("Testing data type\n");
        fflush(stdout);
        if(data.image[aoconfID_pixstream_wfspixindex].md[0].atype != _DATATYPE_UINT16)
            COMPUTE_PIXELSTREAMING = 0;
    }

    if(COMPUTE_PIXELSTREAMING == 1)
    {
        long xsize = data.image[aoconfID_pixstream_wfspixindex].md[0].size[0];
        long ysize = data.image[aoconfID_pixstream_wfspixindex].md[0].size[1];
        PIXSTREAM_NBSLICES = 0;
        for(ii=0; ii<xsize*ysize; ii++)
            if(data.image[aoconfID_pixstream_wfspixindex].array.UI16[ii] > PIXSTREAM_NBSLICES)
                PIXSTREAM_NBSLICES = data.image[aoconfID_pixstream_wfspixindex].array.UI16[ii];
        PIXSTREAM_NBSLICES++;
        printf("PIXEL STREAMING:   %d image slices\n", PIXSTREAM_NBSLICES);
    }


    printf("============ FORCE pixel streaming = 0\n");
    fflush(stdout);
    COMPUTE_PIXELSTREAMING = 0; // TEST


    printf("GPU0 = %d\n", AOconf[loop].GPU0);
    if(AOconf[loop].GPU0>1)
    {
        uint8_t k;
        for(k=0; k<AOconf[loop].GPU0; k++)
            printf("stream %2d      GPUset0 = %2d\n", (int) k, GPUset0[k]);
    }

    printf("GPU1 = %d\n", AOconf[loop].GPU1);
    if(AOconf[loop].GPU1>1)
    {
        uint8_t k;
        for(k=0; k<AOconf[loop].GPU1; k++)
            printf("stream %2d      GPUset1 = %2d\n", (int) k, GPUset1[k]);
    }



    vOK = 1;
    if(AOconf[loop].init_wfsref0==0)
    {
        printf("ERROR: CANNOT RUN LOOP WITHOUT WFS REFERENCE\n");
        vOK = 0;
    }
    if(AOconf[loop].init_CM==0)
    {
        printf("ERROR: CANNOT RUN LOOP WITHOUT CONTROL MATRIX\n");
        vOK = 0;
    }

	aoconfcnt0_wfsref_current = data.image[aoconfID_wfsref].md[0].cnt0;
	aoconfcnt0_contrM_current = data.image[aoconfID_contrM].md[0].cnt0;

    AOconf[loop].initmapping = 0;
    AOconf[loop].init_CMc = 0;
    clock_gettime(CLOCK_REALTIME, &t1);


    if(vOK==1)
    {
		AOconf[loop].LOOPiteration = 0;
        AOconf[loop].kill = 0;
        AOconf[loop].on = 0;
        AOconf[loop].DMprimaryWriteON = 0;
        AOconf[loop].DMfilteredWriteON = 0;
        AOconf[loop].ARPFon = 0;
        printf("entering loop ...\n");
        fflush(stdout);

        int timerinit = 0;

        while( AOconf[loop].kill == 0)
        {
            if(timerinit==1)
            {
                clock_gettime(CLOCK_REALTIME, &t1);
                printf("timer init\n");
            }
            clock_gettime(CLOCK_REALTIME, &t2);

            tdiff = info_time_diff(t1, t2);
            double tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;

            printf(" WAITING     %20.3lf sec         \r", tdiffv);
            fflush(stdout);
            usleep(1000);


            timerinit = 0;
            while(AOconf[loop].on == 1)
            {
                if(timerinit==0)
                {
                    //      Read_cam_frame(loop, 0, AOconf[loop].WFSnormalize, 0, 1);
                    clock_gettime(CLOCK_REALTIME, &t1);
                    timerinit = 1;
                }

                AOcompute(loop, AOconf[loop].WFSnormalize);

                AOconf[loop].status = 12; // 12
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoconfID_looptiming].array.F[19] = tdiffv;


                if(AOconf[loop].CMMODE==0)  // 2-step : WFS -> mode coeffs -> DM act
                {
                    if(AOconf[loop].DMprimaryWriteON==1) // if Writing to DM
                    {


                        if(fabs(AOconf[loop].gain)>1.0e-6)
                            set_DM_modes(loop);
                    }

                }
                else // 1 step: WFS -> DM act
                {
                    if(AOconf[loop].DMprimaryWriteON==1) // if Writing to DM
                    {
                        data.image[aoconfID_dmC].md[0].write = 1;

                        for(ii=0; ii<AOconf[loop].sizeDM; ii++)//TEST
                        {
                            if(isnan(data.image[aoconfID_meas_act].array.F[ii])!=0)
                            {
                                printf("image aol2_meas_act  element %ld is NAN -> replacing by 0\n", ii);
                                data.image[aoconfID_meas_act].array.F[ii] = 0.0;
                            }
                        }


                        AOconf[loop].status = 13; // enforce limits
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                        data.image[aoconfID_looptiming].array.F[20] = tdiffv;


                        for(ii=0; ii<AOconf[loop].sizeDM; ii++)
                        {
                            data.image[aoconfID_dmC].array.F[ii] -= AOconf[loop].gain * data.image[aoconfID_meas_act].array.F[ii];

                            data.image[aoconfID_dmC].array.F[ii] *= AOconf[loop].mult;

                            if(data.image[aoconfID_dmC].array.F[ii] > AOconf[loop].maxlimit)
                                data.image[aoconfID_dmC].array.F[ii] = AOconf[loop].maxlimit;
                            if(data.image[aoconfID_dmC].array.F[ii] < -AOconf[loop].maxlimit)
                                data.image[aoconfID_dmC].array.F[ii] = -AOconf[loop].maxlimit;
                        }


                        AOconf[loop].status = 14; // write to DM
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                        data.image[aoconfID_looptiming].array.F[21] = tdiffv;



                      /*  int semnb;
                        for(semnb=0; semnb<data.image[aoconfID_dmC].md[0].sem; semnb++)
                        {
                            sem_getvalue(data.image[aoconfID_dmC].semptr[semnb], &semval);
                            if(semval<SEMAPHORE_MAXVAL)
                                sem_post(data.image[aoconfID_dmC].semptr[semnb]);
                        }*/
                        
                        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_dmC, -1);
						data.image[aoconfID_dmC].md[0].cnt1 = AOconf[loop].LOOPiteration;
                        data.image[aoconfID_dmC].md[0].cnt0++;
                        data.image[aoconfID_dmC].md[0].write = 0;
                        // inform dmdisp that new command is ready in one of the channels
                        if(aoconfID_dmdisp!=-1)
                            if(data.image[aoconfID_dmdisp].md[0].sem > 1)
                            {
                                sem_getvalue(data.image[aoconfID_dmdisp].semptr[0], &semval);
                                if(semval<SEMAPHORE_MAXVAL)
                                    sem_post(data.image[aoconfID_dmdisp].semptr[1]);
                            }
                        AOconf[loop].DMupdatecnt ++;
                    }
                }

                AOconf[loop].status = 18; // 18
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoconfID_looptiming].array.F[22] = tdiffv;

                AOconf[loop].cnt++;

		
				AOconf[loop].LOOPiteration++;
				data.image[aoconfID_looptiming].md[0].cnt1 = AOconf[loop].LOOPiteration;
				

                data.image[aoconfIDlogdata].md[0].cnt0 = AOconf[loop].cnt;
                data.image[aoconfIDlogdata].md[0].cnt1 = AOconf[loop].LOOPiteration;
                data.image[aoconfIDlogdata].array.F[0] = AOconf[loop].gain;


                if(AOconf[loop].cnt == AOconf[loop].cntmax)
                    AOconf[loop].on = 0;
            }

        }
    }

    free(thetime);

    return(0);
}



int_fast8_t ControlMatrixMultiply( float *cm_array, float *imarray, long m, long n, float *outvect)
{
    long i;

    cblas_sgemv (CblasRowMajor, CblasNoTrans, m, n, 1.0, cm_array, n, imarray, 1, 0.0, outvect, 1);

    return(0);
}



/**
 * ## Purpose
 * 
 * Send modal commands to DM. \n
 * Converts mode coefficient to DM map by matrix-vector multiplication \n
 * Runs in CPU or GPU.
 * 
 * Takes mode values from aol_DMmode_cmd (ID = aoconfID_cmd_modes)
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 */ 
int_fast8_t set_DM_modes(long loop)
{
    double a;
    long cnttest;
    int semval;



	

    if(AOconf[loop].GPU1 == 0)
    {
        float *arrayf;
        long i, j, k;

        arrayf = (float*) malloc(sizeof(float)*AOconf[loop].sizeDM);

        for(j=0; j<AOconf[loop].sizeDM; j++)
            arrayf[j] = 0.0;

        for(i=0; i<AOconf[loop].sizeDM; i++)
            for(k=0; k < AOconf[loop].NBDMmodes; k++)
                arrayf[i] += data.image[aoconfID_cmd_modes].array.F[k] * data.image[aoconfID_DMmodes].array.F[k*AOconf[loop].sizeDM+i];

        data.image[aoconfID_dmC].md[0].write = 1;
        memcpy (data.image[aoconfID_dmC].array.F, arrayf, sizeof(float)*AOconf[loop].sizeDM);
        if(data.image[aoconfID_dmC].md[0].sem > 0)
        {
            sem_getvalue(data.image[aoconfID_dmC].semptr[0], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoconfID_dmC].semptr[0]);
        }
        data.image[aoconfID_dmC].md[0].cnt0++;
		data.image[aoconfID_dmC].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoconfID_dmC].md[0].write = 0;

        free(arrayf);
    }
    else
    {
#ifdef HAVE_CUDA


        GPU_loop_MultMat_setup(1, data.image[aoconfID_DMmodes].name, data.image[aoconfID_cmd_modes].name, data.image[aoconfID_dmC].name, AOconf[loop].GPU1, GPUset1, 1, AOconf[loop].GPUusesem, 1, loop);
        AOconf[loop].status = 12;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[32] = tdiffv;

        GPU_loop_MultMat_execute(1, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 30);
#endif
    }

    if(aoconfID_dmdisp!=-1)
        if(data.image[aoconfID_dmdisp].md[0].sem > 1)
        {
            sem_getvalue(data.image[aoconfID_dmdisp].semptr[1], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoconfID_dmdisp].semptr[1]);
        }

    AOconf[loop].DMupdatecnt ++;

    return(0);
}





/**
 * ## Purpose
 * 
 * Set deformable mirror modes related to the response matrix 
 * 
 * Takes mode values from ????????,
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 */ 

int_fast8_t set_DM_modesRM(long loop)
{
    long k;
    long i, j;
    float *arrayf;


    arrayf = (float*) malloc(sizeof(float)*AOconf[loop].sizeDM);

    for(j=0; j<AOconf[loop].sizeDM; j++)
        arrayf[j] = 0.0;

    for(k=0; k < AOconf[loop].NBDMmodes; k++)
    {
        for(i=0; i<AOconf[loop].sizeDM; i++)
            arrayf[i] += data.image[aoconfID_cmd_modesRM].array.F[k] * data.image[aoconfID_DMmodes].array.F[k*AOconf[loop].sizeDM+i];
    }


    data.image[aoconfID_dmRM].md[0].write = 1;
    memcpy (data.image[aoconfID_dmRM].array.F, arrayf, sizeof(float)*AOconf[loop].sizeDM);
    data.image[aoconfID_dmRM].md[0].cnt0++;
    data.image[aoconfID_dmRM].md[0].write = 0;

    free(arrayf);
    AOconf[loop].DMupdatecnt ++;

    return(0);
}



/**
 * ## Purpose
 * 
 * 
 * 
 * Takes mode values from ????????
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 *
 * @param[in]
 * paramname	int
 * 				normalize 
 * 
 */ 
int_fast8_t __attribute__((hot)) AOcompute(long loop, int normalize)
{
    long k1, k2;
    long ii;
    long i;
    long m, n;
    long index;
    //  long long wcnt;
    // long long wcntmax;
    double a;

    float *matrix_cmp;
    long wfselem, act, mode;

    struct timespec t1;
    struct timespec t2;

    float *matrix_Mc, *matrix_DMmodes;
    long n_sizeDM, n_NBDMmodes, n_sizeWFS;

    long IDmask;
    long act_active, wfselem_active;
    float *matrix_Mc_active;
    long IDcmatca_shm;
    int r;
    float imtot;

    int slice;
    int semnb;
    int semval;

	uint64_t LOOPiter;
	
	
	
	// lock loop iteration into variable so that it cannot increment 
	LOOPiter = AOconf[loop].LOOPiteration;


    // waiting for dark-subtracted image
    AOconf[loop].status = 19;  //  19: WAITING FOR IMAGE
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoconfID_looptiming].array.F[23] = tdiffv;



    // md[0].atime.ts is absolute time at beginning of iteration
    //
    // pixel 0 is dt since last iteration
    //
    // pixel 1 is time from beginning of loop to status 01
    // pixel 2 is time from beginning of loop to status 02


    Read_cam_frame(loop, 0, normalize, 0, 0);

    slice = PIXSTREAM_SLICE;
    if(COMPUTE_PIXELSTREAMING==0) // no pixel streaming
        PIXSTREAM_SLICE = 0;
    //    else
    //        PIXSTREAM_SLICE = 1 + slice;

    //    printf("slice = %d  ->  %d\n", slice, PIXSTREAM_SLICE);
    //    fflush(stdout);

    AOconf[loop].status = 4;  // 4: REMOVING REF
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoconfID_looptiming].array.F[15] = tdiffv;


    if(AOconf[loop].GPUall==0)
    {
        data.image[aoconfID_imWFS2].md[0].write = 1;
        for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
            data.image[aoconfID_imWFS2].array.F[ii] = data.image[aoconfID_imWFS1].array.F[ii] - normfloorcoeff*data.image[aoconfID_wfsref].array.F[ii];
        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS2, -1);
        data.image[aoconfID_imWFS2].md[0].cnt0 ++;
        data.image[aoconfID_imWFS2].md[0].cnt1 = LOOPiter;
        data.image[aoconfID_imWFS2].md[0].write = 0;
    }


    AOconf[loop].status = 5; // 5 MULTIPLYING BY CONTROL MATRIX -> MODE VALUES
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoconfID_looptiming].array.F[16] = tdiffv;


    if(AOconf[loop].initmapping == 0) // compute combined control matrix or matrices
    {
        printf("COMPUTING MAPPING ARRAYS .... \n");
        fflush(stdout);

        clock_gettime(CLOCK_REALTIME, &t1);

        //
        // There is one mapping array per WFS slice
        // WFS slice 0 = all active pixels
        //
        WFS_active_map = (int*) malloc(sizeof(int)*AOconf[loop].sizeWFS*PIXSTREAM_NBSLICES);
        if(aoconfID_wfsmask != -1)
        {
            for(slice=0; slice<PIXSTREAM_NBSLICES; slice++)
            {
                long ii1 = 0;
                for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
                    if(data.image[aoconfID_wfsmask].array.F[ii]>0.1)
                    {
                        if(slice==0)
                        {
                            WFS_active_map[slice*AOconf[loop].sizeWFS+ii1] = ii;
                            ii1++;
                        }
                        else if (data.image[aoconfID_pixstream_wfspixindex].array.UI16[ii]==slice+1)
                        {
                            WFS_active_map[slice*AOconf[loop].sizeWFS+ii1] = ii;
                            ii1++;
                        }
                    }
                AOconf[loop].sizeWFS_active[slice] = ii1;

                char imname[200];
                if(sprintf(imname, "aol%ld_imWFS2active_%02d", LOOPNUMBER, slice) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                uint32_t *sizearray;
                sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
                sizearray[0] =  AOconf[loop].sizeWFS_active[slice];
                sizearray[1] =  1;
                aoconfID_imWFS2_active[slice] = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
                free(sizearray);
                //aoconfID_imWFS2_active[slice] = create_2Dimage_ID(imname, AOconf[loop].sizeWFS_active[slice], 1);
            }
        }
        else
        {
            printf("ERROR: aoconfID_wfsmask = -1\n");
            fflush(stdout);
            exit(0);
        }



        // create DM active map
        DM_active_map = (int*) malloc(sizeof(int)*AOconf[loop].sizeDM);
        if(aoconfID_dmmask != -1)
        {
            long ii1 = 0;
            for(ii=0; ii<AOconf[loop].sizeDM; ii++)
                if(data.image[aoconfID_dmmask].array.F[ii]>0.5)
                {
                    DM_active_map[ii1] = ii;
                    ii1++;
                }
            AOconf[loop].sizeDM_active = ii1;
        }



        uint32_t *sizearray;
        sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
        sizearray[0] = AOconf[loop].sizeDM_active;
        sizearray[1] = 1;

        char imname[200];
        if(sprintf(imname, "aol%ld_meas_act_active", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_meas_act_active = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
        free(sizearray);



        if(aoconfID_meas_act==-1)
        {
            sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
            sizearray[0] = AOconf[loop].sizexDM;
            sizearray[1] = AOconf[loop].sizeyDM;

            if(sprintf(imname, "aol%ld_meas_act", LOOPNUMBER) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            aoconfID_meas_act = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            COREMOD_MEMORY_image_set_createsem(imname, 10);
            free(sizearray);
        }

        clock_gettime(CLOCK_REALTIME, &t2);
        tdiff = info_time_diff(t1, t2);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        printf("\n");
        printf("TIME TO COMPUTE MAPPING ARRAYS = %f sec\n", tdiffv);
        AOconf[loop].initmapping = 1;
    }




    if(AOconf[loop].GPU0 == 0)   // no GPU -> run in CPU
    {
        if(AOconf[loop].CMMODE==0)  // goes explicitely through modes, slower but required for access to mode values
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=0, CMMODE=0 - %s x %s -> %s\n", data.image[aoconfID_contrM].md[0].name, data.image[aoconfID_imWFS2].md[0].name, data.image[aoconfID_meas_modes].md[0].name);
            fflush(stdout);
#endif

            data.image[aoconfID_meas_modes].md[0].write = 1;
            ControlMatrixMultiply( data.image[aoconfID_contrM].array.F, data.image[aoconfID_imWFS2].array.F, AOconf[loop].NBDMmodes, AOconf[loop].sizeWFS, data.image[aoconfID_meas_modes].array.F);
            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_meas_modes, -1);
            data.image[aoconfID_meas_modes].md[0].cnt0 ++;
            data.image[aoconfID_meas_modes].md[0].cnt1 = LOOPiter;
            data.image[aoconfID_meas_modes].md[0].write = 0;
        }
        else // (*)
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=0, CMMODE=1 - using matrix %s\n", data.image[aoconfID_contrMc].md[0].name);
            fflush(stdout);
#endif

            data.image[aoconfID_meas_modes].md[0].write = 1;
            ControlMatrixMultiply( data.image[aoconfID_contrMc].array.F, data.image[aoconfID_imWFS2].array.F, AOconf[loop].sizeDM, AOconf[loop].sizeWFS, data.image[aoconfID_meas_act].array.F);
            data.image[aoconfID_meas_modes].md[0].cnt0 ++;
            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_meas_modes, -1);
            data.image[aoconfID_meas_modes].md[0].cnt0 ++;
			data.image[aoconfID_meas_modes].md[0].cnt1 = LOOPiter;
            data.image[aoconfID_meas_modes].md[0].write = 0;
        }
    }
    else  // run in GPU if possible 
    {
#ifdef HAVE_CUDA
        if(AOconf[loop].CMMODE==0)  // goes explicitely through modes, slower but required for access to mode values
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=1, CMMODE=0 - using matrix %s    GPU alpha beta = %f %f\n", data.image[aoconfID_contrM].md[0].name, GPU_alpha, GPU_beta);
            fflush(stdout);
#endif

            initWFSref_GPU[PIXSTREAM_SLICE] = 1; // default: do not re-compute reference output

            if(AOconf[loop].GPUall == 1)
            {
                // TEST IF contrM or wfsref have changed
                if((data.image[aoconfID_wfsref].md[0].cnt0 != aoconfcnt0_wfsref_current) || (data.image[aoconfID_contrM].md[0].cnt0 != aoconfcnt0_contrM_current))
					{
						printf("NEW wfsref [%10ld] or contrM [%10ld]\n", data.image[aoconfID_wfsref].md[0].cnt0, data.image[aoconfID_contrM].md[0].cnt0);
						aoconfcnt0_wfsref_current = data.image[aoconfID_wfsref].md[0].cnt0;
						aoconfcnt0_contrM_current = data.image[aoconfID_contrM].md[0].cnt0;
						initWFSref_GPU[PIXSTREAM_SLICE] = 0;
					}


                if(initWFSref_GPU[PIXSTREAM_SLICE]==0) // initialize WFS reference
                {
#ifdef _PRINT_TEST
                    printf("\nINITIALIZE WFS REFERENCE: COPY NEW REF (WFSREF) TO imWFS0\n"); //TEST
                    fflush(stdout);
#endif

                    data.image[aoconfID_imWFS0].md[0].write = 1;
                    memcpy(data.image[aoconfID_imWFS0].array.F, data.image[aoconfID_wfsref].array.F, sizeof(float)*AOconf[loop].sizeWFS);
             //       for(wfselem=0; wfselem<AOconf[loop].sizeWFS; wfselem++)
             //           data.image[aoconfID_imWFS0].array.F[wfselem] = data.image[aoconfID_wfsref].array.F[wfselem];
                    COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS0, -1);
                    data.image[aoconfID_imWFS0].md[0].cnt0++;
                    data.image[aoconfID_imWFS0].md[0].cnt1 = LOOPiter;
                    data.image[aoconfID_imWFS0].md[0].write = 0;
                    fflush(stdout);
                }
            }


            if(AOconf[loop].GPUall == 1)
                GPU_loop_MultMat_setup(0, data.image[aoconfID_contrM].name, data.image[aoconfID_imWFS0].name, data.image[aoconfID_meas_modes].name, AOconf[loop].GPU0, GPUset0, 0, AOconf[loop].GPUusesem, initWFSref_GPU[PIXSTREAM_SLICE], loop);
            else
                GPU_loop_MultMat_setup(0, data.image[aoconfID_contrM].name, data.image[aoconfID_imWFS2].name, data.image[aoconfID_meas_modes].name, AOconf[loop].GPU0, GPUset0, 0, AOconf[loop].GPUusesem, 1, loop);

			

            initWFSref_GPU[PIXSTREAM_SLICE] = 1;

            AOconf[loop].status = 6; // 6 execute
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoconfID_looptiming].array.F[17] = tdiffv;


            if(AOconf[loop].GPUall == 1)
                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], GPU_alpha, GPU_beta, 1, 25);
            else
                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 25);
        }
        else // direct pixel -> actuators linear transformation
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=1, CMMODE=1\n");
            fflush(stdout);
#endif

			// depreciated: use all pixels
/*            if(1==0) 
            {
                GPU_loop_MultMat_setup(0, data.image[aoconfID_contrMc].name, data.image[aoconfID_imWFS2].name, data.image[aoconfID_meas_act].name, AOconf[loop].GPU0, GPUset0, 0, AOconf[loop].GPUusesem, 1, loop);
                AOconf[loop].status = 6; 
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoconfID_looptiming].array.F[6] = tdiffv;

                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1);
            }
            else // only use active pixels and actuators (**)
            {*/
             
                // re-map input vector into imWFS2_active

                if(AOconf[loop].GPUall == 1) // (**)
                {
#ifdef _PRINT_TEST
                    printf("TEST - CM mult: GPU=1, CMMODE=1, GPUall = 1\n");
                    fflush(stdout);
#endif

                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 1;
                    for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[PIXSTREAM_SLICE]; wfselem_active++)
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoconfID_imWFS0].array.F[WFS_active_map[PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                    COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS2_active[PIXSTREAM_SLICE], -1);
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt0++;
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 0;
                }
                else
                {
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 1;
                    for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[PIXSTREAM_SLICE]; wfselem_active++)
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoconfID_imWFS2].array.F[WFS_active_map[PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                    COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS2_active[PIXSTREAM_SLICE], -1);
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt0++;
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                    data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 0;
                }

                // look for updated control matrix or reference
                if(AOconf[loop].GPUall == 1) // (**)
                {
                    if(data.image[aoconfID_contrMcact[PIXSTREAM_SLICE]].md[0].cnt0 != contrMcactcnt0[PIXSTREAM_SLICE])
                    {
                        printf("NEW CONTROL MATRIX DETECTED (%s) -> RECOMPUTE REFERENCE x MATRIX\n", data.image[aoconfID_contrMcact[PIXSTREAM_SLICE]].md[0].name);
                        fflush(stdout);

                        initWFSref_GPU[PIXSTREAM_SLICE] = 0;
                        contrMcactcnt0[PIXSTREAM_SLICE] = data.image[aoconfID_contrMcact[PIXSTREAM_SLICE]].md[0].cnt0;
                    }

                    if(data.image[aoconfID_wfsref].md[0].cnt0 != wfsrefcnt0)  // (*)
                    {
                        printf("NEW REFERENCE WFS DETECTED (%s) [ %ld %ld ]\n", data.image[aoconfID_wfsref].md[0].name, data.image[aoconfID_wfsref].md[0].cnt0, wfsrefcnt0);
                        fflush(stdout);

                        initWFSref_GPU[PIXSTREAM_SLICE] = 0;
                        wfsrefcnt0 = data.image[aoconfID_wfsref].md[0].cnt0;
                    }
                    if(initWFSref_GPU[PIXSTREAM_SLICE]==0) // initialize WFS reference
                    {
                        printf("\nINITIALIZE WFS REFERENCE: COPY NEW REF (WFSREF) TO imWFS2_active\n"); //TEST
                        fflush(stdout);
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 1;
                        for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[PIXSTREAM_SLICE]; wfselem_active++)
                            data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoconfID_wfsref].array.F[WFS_active_map[PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS2_active[PIXSTREAM_SLICE], -1);
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt0++;
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                        data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].md[0].write = 0;
                        fflush(stdout);
                    }
                }

                if(initcontrMcact_GPU[PIXSTREAM_SLICE]==0)
                    initWFSref_GPU[PIXSTREAM_SLICE] = 0;


                GPU_loop_MultMat_setup(0, data.image[aoconfID_contrMcact[PIXSTREAM_SLICE]].name, data.image[aoconfID_imWFS2_active[PIXSTREAM_SLICE]].name, data.image[aoconfID_meas_act_active].name, AOconf[loop].GPU0, GPUset0, 0, AOconf[loop].GPUusesem, initWFSref_GPU[PIXSTREAM_SLICE], loop);


                initWFSref_GPU[PIXSTREAM_SLICE] = 1;
                initcontrMcact_GPU[PIXSTREAM_SLICE] = 1;
                
                AOconf[loop].status = 6; // 6 execute
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoconfID_looptiming].array.F[17] = tdiffv;


                if(AOconf[loop].GPUall == 1)
                    GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], GPU_alpha, GPU_beta, 1, 25);
                else
                    GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 25);

                // re-map output vector
                data.image[aoconfID_meas_act].md[0].write = 1;
                for(act_active=0; act_active<AOconf[loop].sizeDM_active; act_active++)
                    data.image[aoconfID_meas_act].array.F[DM_active_map[act_active]] = data.image[aoconfID_meas_act_active].array.F[act_active];

				COREMOD_MEMORY_image_set_sempost_byID(aoconfID_meas_act, -1);
            /*    for(semnb=0; semnb<data.image[aoconfID_meas_act].md[0].sem; semnb++)
                {
                    sem_getvalue(data.image[aoconfID_meas_act].semptr[semnb], &semval);
                    if(semval<SEMAPHORE_MAXVAL)
                        sem_post(data.image[aoconfID_meas_act].semptr[semnb]);
                }*/
                data.image[aoconfID_meas_act].md[0].cnt0++;
                data.image[aoconfID_meas_act].md[0].cnt1 = LOOPiter;
                data.image[aoconfID_meas_act].md[0].write = 0;
            //}
        }
#endif
    }

    AOconf[loop].status = 11; // 11 MULTIPLYING BY GAINS
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoconfID_looptiming].array.F[18] = tdiffv;

    if(AOconf[loop].CMMODE==0)
    {
		int block;
		long k;
		
		
        AOconf[loop].RMSmodes = 0;
        for(k=0; k<AOconf[loop].NBDMmodes; k++)
            AOconf[loop].RMSmodes += data.image[aoconfID_meas_modes].array.F[k]*data.image[aoconfID_meas_modes].array.F[k];

        AOconf[loop].RMSmodesCumul += AOconf[loop].RMSmodes;
        AOconf[loop].RMSmodesCumulcnt ++;

		data.image[aoconfID_cmd_modes].md[0].write = 1;

        for(k=0; k<AOconf[loop].NBDMmodes; k++)
        {	
            data.image[aoconfID_RMS_modes].array.F[k] = 0.99*data.image[aoconfID_RMS_modes].array.F[k] + 0.01*data.image[aoconfID_meas_modes].array.F[k]*data.image[aoconfID_meas_modes].array.F[k];
            data.image[aoconfID_AVE_modes].array.F[k] = 0.99*data.image[aoconfID_AVE_modes].array.F[k] + 0.01*data.image[aoconfID_meas_modes].array.F[k];
			
			
			// apply gain
            
            data.image[aoconfID_cmd_modes].array.F[k] -= AOconf[loop].gain * data.image[aoconfID_gainb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoconfID_DMmode_GAIN].array.F[k] * data.image[aoconfID_meas_modes].array.F[k];


			// apply limits
			
			float limitval;
			limitval = AOconf[loop].maxlimit * data.image[aoconfID_limitb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoconfID_LIMIT_modes].array.F[k];
            
            if(data.image[aoconfID_cmd_modes].array.F[k] < -limitval)
                data.image[aoconfID_cmd_modes].array.F[k] = -limitval;

            if(data.image[aoconfID_cmd_modes].array.F[k] > limitval)
                data.image[aoconfID_cmd_modes].array.F[k] = limitval;


			// apply mult factor
			
            data.image[aoconfID_cmd_modes].array.F[k] *= AOconf[loop].mult * data.image[aoconfID_multfb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoconfID_MULTF_modes].array.F[k];

            
            
            // update total gain
            //     data.image[aoconfID_DMmode_GAIN].array.F[k+AOconf[loop].NBDMmodes] = AOconf[loop].gain * data.image[aoconfID_DMmode_GAIN].array.F[k];
        }


        data.image[aoconfID_cmd_modes].md[0].cnt0 ++;
        data.image[aoconfID_cmd_modes].md[0].cnt1 = LOOPiter;
        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_cmd_modes, -1);
        data.image[aoconfID_cmd_modes].md[0].write = 0;
        
    }

    return(0);
}





































int_fast8_t AOloopControl_CompModes_loop(const char *ID_CM_name, const char *ID_WFSref_name, const char *ID_WFSim_name, const char *ID_WFSimtot_name, const char *ID_coeff_name)
{
#ifdef HAVE_CUDA

    int *GPUsetM;
    long ID_CM;
    long ID_WFSref;
    long ID_coeff;
    long GPUcnt;
    int k;
    int_fast8_t GPUstatus[100];
    int_fast8_t status;
    long NBmodes;
    uint32_t *sizearray;

    long ID_WFSim;
    long ID_WFSim_n;
    long wfsxsize, wfsysize;
    int m;
    long IDcoeff0;

    long ID_WFSimtot;
    double totfluxave;
    long ID_coefft;

    double alpha = 0.1;
	char imname[200];


	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    GPUcnt = 2;

    GPUsetM = (int*) malloc(sizeof(int)*GPUcnt);
    for(k=0; k<GPUcnt; k++)
        GPUsetM[k] = k+5;


    ID_CM = image_ID(ID_CM_name);
    wfsxsize = data.image[ID_CM].md[0].size[0];
    wfsysize = data.image[ID_CM].md[0].size[1];
    NBmodes = data.image[ID_CM].md[0].size[2];

    ID_WFSref = image_ID(ID_WFSref_name);


    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = NBmodes;
    sizearray[1] = 1;

    ID_coeff = create_image_ID(ID_coeff_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(ID_coeff_name, 10);
    data.image[ID_coeff].md[0].cnt0 = 0;

    ID_coefft = create_image_ID("coefftmp", 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem("coefftmp", 10);


    IDcoeff0 = create_image_ID("coeff0", 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    ID_WFSim_n = create_2Dimage_ID("wfsim_n", wfsxsize, wfsysize);
    COREMOD_MEMORY_image_set_createsem("wfsim_n", 10);




    ID_WFSim = image_ID(ID_WFSim_name);
    ID_WFSimtot = image_ID(ID_WFSimtot_name);


    GPU_loop_MultMat_setup(2, ID_CM_name, "wfsim_n", "coefftmp", GPUcnt, GPUsetM, 0, 1, 1, 0);

    totfluxave = 1.0;
    int initWFSref;
    for(;;)
    {
        if(initWFSref==0)
        {
            printf("Computing reference\n");
            fflush(stdout);
            memcpy(data.image[ID_WFSim_n].array.F, data.image[ID_WFSref].array.F, sizeof(float)*wfsxsize*wfsysize);
            GPU_loop_MultMat_execute(2, &status, &GPUstatus[0], 1.0, 0.0, 0, 0);
            for(m=0; m<NBmodes; m++)
            {
                data.image[IDcoeff0].array.F[m] = data.image[ID_coefft].array.F[m];
            }
            printf("\n");
            initWFSref = 1;
            printf("reference computed\n");
            fflush(stdout);
        }

        memcpy(data.image[ID_WFSim_n].array.F, data.image[ID_WFSim].array.F, sizeof(float)*wfsxsize*wfsysize);
        COREMOD_MEMORY_image_set_semwait(ID_WFSim_name, 0);

        GPU_loop_MultMat_execute(2, &status, &GPUstatus[0], 1.0, 0.0, 0, 0);
        totfluxave = (1.0-alpha)*totfluxave + alpha*data.image[ID_WFSimtot].array.F[0];

        data.image[ID_coeff].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[ID_coeff].array.F[m] = data.image[ID_coefft].array.F[m]/totfluxave - data.image[IDcoeff0].array.F[m];
        data.image[ID_coeff].md[0].cnt0 ++;
        data.image[ID_coeff].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[ID_coeff].md[0].write = 0;
    }


    delete_image_ID("coeff0");
    free(sizearray);

    free(GPUsetM);



#endif

    return(0);
}





//
// compute DM map from mode values
// this is a separate process 
//
// if offloadMode = 1, apply correction to aol#_dmC
//
int_fast8_t AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, const char *modecoeffs_name, const char *DMmodes_name, int semTrigg, const char *out_name, int GPUindex, long loop, int offloadMode)
{
#ifdef HAVE_CUDA
    long IDmodecoeffs;
    int GPUcnt, k;
    int *GPUsetM;
    int_fast8_t GPUstatus[100];
    int_fast8_t status;
    float alpha = 1.0;
    float beta = 0.0;
    int initWFSref = 0;
    int orientation = 1;
    int use_sem = 1;
    long IDout;
    int write_timing = 0;
    long NBmodes, m;

    float x, x2, x4, x8;
    float gamma;

    uint32_t *sizearray;
    char imnameInput[200];
    long IDmodesC;

    long IDc;
    long dmxsize, dmysize;
    long ii;


    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;

	char imname[200];


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


	if(GPUMATMULTCONFindex==0)
    {
		// read AO loop gain, mult
		if(AOloopcontrol_meminit==0)
			AOloopControl_InitializeMemory(1);
	}


    GPUcnt = 1;
    GPUsetM = (int*) malloc(sizeof(int)*GPUcnt);
    for(k=0; k<GPUcnt; k++)
        GPUsetM[k] = k+GPUindex;

    IDout = image_ID(out_name);
    IDmodecoeffs = image_ID(modecoeffs_name);

    NBmodes = data.image[IDmodecoeffs].md[0].size[0];


   // sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);

    //if(sprintf(imnameInput, "aol%ld_mode_limcorr", loop) < 1)
    //    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	

   // sizearray[0] = NBmodes;
   // sizearray[1] = 1;
  //  IDmodesC = create_image_ID(imnameInput, 2, sizearray, _DATATYPE_FLOAT, 0, 0);
  //  COREMOD_MEMORY_image_set_createsem(imnamecorr, 10);
  //  free(sizearray);




    GPU_loop_MultMat_setup(GPUMATMULTCONFindex, DMmodes_name, modecoeffs_name, out_name, GPUcnt, GPUsetM, orientation, use_sem, initWFSref, 0);


    for(k=0; k<GPUcnt; k++)
        printf(" ====================     USING GPU %d\n", GPUsetM[k]);

	list_image_ID();

    if(offloadMode==1)
    {
	    char imnamedmC[200];

		
		
        if(sprintf(imnamedmC, "aol%ld_dmC", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        IDc = image_ID(imnamedmC);
        dmxsize = data.image[IDc].md[0].size[0];
        dmysize = data.image[IDc].md[0].size[1];


        printf("offloadMode = %d  %ld %ld\n", offloadMode, dmxsize, dmysize);
        fflush(stdout);
    }
    else
		printf("offloadMode = %d\n", offloadMode);

	printf("out_name = %s \n", out_name);
	printf("IDout    = %ld\n", IDout);
	



    for(;;)
    {
        COREMOD_MEMORY_image_set_semwait(modecoeffs_name, semTrigg);
	
//		if(GPUMATMULTCONFindex==0)
			AOconf[loop].statusM = 10;               
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoconfID_looptiming].array.F[7] = tdiffv;

      //  for(m=0; m<NBmodes; m++)
      //      data.image[IDmodesC].array.F[m] = data.image[IDmodecoeffs].array.F[m];


        GPU_loop_MultMat_execute(GPUMATMULTCONFindex, &status, &GPUstatus[0], alpha, beta, write_timing, 0);

        if(offloadMode==1) // offload back to dmC
        {
            data.image[IDc].md[0].write = 1;
            for(ii=0; ii<dmxsize*dmysize; ii++)
                data.image[IDc].array.F[ii] = data.image[IDout].array.F[ii];

			data.image[IDc].md[0].cnt0++;
			data.image[IDc].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
            COREMOD_MEMORY_image_set_sempost_byID(IDc, -1);
            data.image[IDc].md[0].write = 0;
        }
      
  
  //		if(GPUMATMULTCONFindex==0)  
			AOconf[loop].statusM = 20;
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoconfID_looptiming].array.F[8] = tdiffv;
    }



    free(GPUsetM);

#endif

    return(0);
}





//
// assumes the WFS mode basis is already orthogonall
// removes reference from each frame
//
long AOloopControl_sig2Modecoeff(const char *WFSim_name, const char *IDwfsref_name, const char *WFSmodes_name, const char *outname)
{
    long IDout;
    long IDwfs, IDmodes, IDwfsref;
    long wfsxsize, wfsysize, wfssize, NBmodes, NBframes;
    double totref;
    float coeff;
    long ii, m, kk;
    FILE *fp;
    double *mcoeff_ave;
    double *mcoeff_rms;


    IDwfs = image_ID(WFSim_name);
    wfsxsize = data.image[IDwfs].md[0].size[0];
    wfsysize = data.image[IDwfs].md[0].size[1];
    NBframes = data.image[IDwfs].md[0].size[2];
    wfssize = wfsxsize*wfsysize;




    IDwfsref = image_ID(IDwfsref_name);

    IDmodes = image_ID(WFSmodes_name);
    NBmodes = data.image[IDmodes].md[0].size[2];

    mcoeff_ave = (double*) malloc(sizeof(double)*NBmodes);
    mcoeff_rms = (double*) malloc(sizeof(double)*NBmodes);



    IDout = create_2Dimage_ID(outname, NBframes, NBmodes);

    totref = 0.0;

    for(ii=0; ii<wfssize; ii++)
        totref += data.image[IDwfsref].array.F[ii];
    for(ii=0; ii<wfssize; ii++)
        data.image[IDwfsref].array.F[ii] /= totref;

    for(kk=0; kk<NBframes; kk++)
    {
		double totim = 0.0;
		
        for(ii=0; ii<wfssize; ii++)
            totim += data.image[IDwfs].array.F[kk*wfssize+ii];
        for(ii=0; ii<wfssize; ii++)
        {
            data.image[IDwfs].array.F[kk*wfssize+ii] /= totim;
            data.image[IDwfs].array.F[kk*wfssize+ii] -= data.image[IDwfsref].array.F[ii];
        }


        for(m=0; m<NBmodes; m++)
        {
            coeff = 0.0;
            for(ii=0; ii<wfssize; ii++)
                coeff += data.image[IDmodes].array.F[m*wfssize+ii] * data.image[IDwfs].array.F[kk*wfssize+ii];
            data.image[IDout].array.F[m*NBframes+kk] = coeff;
            mcoeff_ave[m] += coeff;
            mcoeff_rms[m] += coeff*coeff;
        }
    }


    fp  = fopen("mode_stats.txt", "w");
    for(m=0; m<NBmodes; m++)
    {
        mcoeff_rms[m] = sqrt( mcoeff_rms[m]/NBframes );
        mcoeff_ave[m] /= NBframes;
        fprintf(fp, "%4ld  %12g %12g\n", m, mcoeff_ave[m], mcoeff_rms[m]);
    }
    fclose(fp);

    free(mcoeff_ave);
    free(mcoeff_rms);

    return(IDout);
}




/**
 * ## Purpose
 * 
 * Computes average of residual in WFS
 * 
 * ## Arguments
 * 
 * @param[in]
 * loop		INT
 * 			loop number
 * 
 * @param[in]
 * alpha	FLOAT
 * 			averaging coefficient
 * 
 * 
 * ## Output files
 * 
 * - aol_wfsres_ave
 * - aol_wfsres_ave
 * - aol_wfsresm
 * - aol_wfsresm_ave
 * - aol_wfsres_rms
 * 
 * 
 * 
 */

long AOloopControl_computeWFSresidualimage(long loop, char *IDalpha_name)
{
    long IDwfsref, IDwfsmask, IDtot, IDout, IDoutave, IDoutm, IDoutmave, IDoutrms;
    char imname[200];
    uint32_t *sizearray;
    long wfsxsize, wfsysize, wfsxysize;
    long cnt;
    long ii;
	long IDalpha;

	IDalpha = image_ID(IDalpha_name);
	

    if(sprintf(imname, "aol%ld_imWFS0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	aoconfID_imWFS0 = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_wfsref", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	
    IDwfsref = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_wfsmask", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDwfsmask = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_imWFS0tot", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDtot = read_sharedmem_image(imname);
	
	

    wfsxsize = data.image[aoconfID_imWFS0].md[0].size[0];
    wfsysize = data.image[aoconfID_imWFS0].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = wfsxsize;
    sizearray[1] = wfsysize;


    if(sprintf(imname, "aol%ld_wfsres", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDout = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


    if(sprintf(imname, "aol%ld_wfsres_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutave = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutave].array.F[ii] = 0.0;


    if(sprintf(imname, "aol%ld_wfsresm", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutm = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


    if(sprintf(imname, "aol%ld_wfsresm_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutmave = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutave].array.F[ii] = 0.0;


    if(sprintf(imname, "aol%ld_wfsres_rms", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutrms = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutrms].array.F[ii] = 0.0;

    free(sizearray);



	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    
    for(;;)
    {
		
        if(data.image[aoconfID_imWFS0].md[0].sem==0)
        {
            while(cnt==data.image[aoconfID_imWFS0].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[aoconfID_imWFS0].md[0].cnt0;
        }
        else
        {
            sem_wait(data.image[aoconfID_imWFS0].semptr[3]);
		}

		//
		// instantaneous WFS residual
        // imWFS0/tot0 - WFSref -> out
        //
		//printf("  %20f\n", data.image[IDtot].array.F[0]);//TEST
		
        data.image[IDout].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDout].array.F[ii] = data.image[aoconfID_imWFS0].array.F[ii]/data.image[IDtot].array.F[0] - data.image[IDwfsref].array.F[ii];
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDout].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);


        // apply mask

        data.image[IDoutm].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutm].array.F[ii] = data.image[IDout].array.F[ii] * data.image[IDwfsmask].array.F[ii];
        data.image[IDoutm].md[0].cnt0++;
        data.image[IDoutm].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutm].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutm, -1);


        // apply gain -> outave

        data.image[IDoutave].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutave].array.F[ii] = (1.0-data.image[IDalpha].array.F[0])*data.image[IDoutave].array.F[ii] + data.image[IDalpha].array.F[0]*data.image[IDout].array.F[ii];
        data.image[IDoutave].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutave].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutave, -1);

        // apply mask

        data.image[IDoutmave].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutmave].array.F[ii] = data.image[IDoutave].array.F[ii] * data.image[IDwfsmask].array.F[ii];
        data.image[IDoutmave].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutmave].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutmave, -1);

        // compute RMS

        data.image[IDoutrms].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutrms].array.F[ii] = (1.0-data.image[IDalpha].array.F[0])*data.image[IDoutrms].array.F[ii] + data.image[IDalpha].array.F[0]*(data.image[IDout].array.F[ii]-data.image[IDoutave].array.F[ii])*(data.image[IDout].array.F[ii]-data.image[IDoutave].array.F[ii]);
        data.image[IDoutrms].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutrms].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutrms, -1);

    }


    return(IDout);
}


















// includes mode filtering (limits, multf)
//
long __attribute__((hot)) AOloopControl_ComputeOpenLoopModes(long loop)
{
    long IDout;
    long IDmodeval; // WFS measurement

    long modeval_bsize = 20; // circular buffer size (valid for both DM and PF)

    long IDmodevalDM_C; // DM correction, circular buffer to include history
    long modevalDMindexl = 0;
    long modevalDMindex = 0; // index in the circular buffer     
    
    long IDmodevalDM; // DM correction at WFS measurement time
    long IDmodevalDMcorr; // current DM correction
    long IDmodevalDMnow; // DM correction after predictiv control 
    long IDmodevalDMnowfilt; // current DM correction filtered
    float alpha;
    long IDmodevalPFsync;

	long IDmodeARPFgain; // predictive filter mixing ratio per gain (0=non-predictive, 1=predictive)
   // long IDmodevalPF; // predictive filter output
	long IDmodevalPFres; // predictive filter measured residual (real-time)
	long IDmodeWFSnoise; // WFS noise
    long IDmodevalPF_C; // modal prediction, circular buffer to include history
    long modevalPFindexl = 0;
    long modevalPFindex = 0; // index in the circular buffer


    long IDblknb;
    char imname[200];
    float *modegain;
    float *modemult;
    float *modelimit;
    long *modeblock;
    long i, m, blk, NBmodes;
    unsigned int blockNBmodes[100];
    uint32_t *sizeout;
    float framelatency = 2.8;
    long framelatency0, framelatency1;
    long IDgainb;
    long cnt;


    // FILTERING
    int FILTERMODE = 1;
    //long IDmodeLIMIT;
    //long IDmodeMULT;

    // TELEMETRY
    long block;
    long blockstatcnt = 0;

	double blockavePFresrms[100];
    double blockaveOLrms[100];
    double blockaveCrms[100]; // correction RMS
    double blockaveWFSrms[100]; // WFS residual RMS
    double blockaveWFSnoise[100]; // WFS noise
    double blockavelimFrac[100];

	double allavePFresrms;
    double allaveOLrms;
    double allaveCrms;
    double allaveWFSrms;
    double allaveWFSnoise;
    double allavelimFrac;

    float limitblockarray[100];
    long IDatlimbcoeff;

	long IDautogain = -1; // automatic gain input
	long long autogainCnt = 0;

    float coeff;

    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;

	long long loopPFcnt;
	FILE *fptest;


	uint64_t LOOPiter;


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif



    // read AO loop gain, mult
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    // INPUT
    if(sprintf(imname, "aol%ld_modeval", loop) < 1)// measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(imname);
    NBmodes = data.image[IDmodeval].md[0].size[0];

    modegain = (float*) malloc(sizeof(float)*NBmodes);
    modemult = (float*) malloc(sizeof(float)*NBmodes);
    modelimit = (float*) malloc(sizeof(float)*NBmodes);

    modeblock = (long*) malloc(sizeof(long)*NBmodes);




	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}



    // CONNECT to dm control channel
    if(aoconfID_dmC == -1)
    {
        if(sprintf(imname, "aol%ld_dmC", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_dmC = read_sharedmem_image(imname);
    }



    // CONNECT to arrays holding gain, limit, and multf values for blocks
    if(aoconfID_gainb == -1)
    {
        if(sprintf(imname, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_gainb = read_sharedmem_image(imname);
    }

    if(aoconfID_multfb == -1)
    {
        if(sprintf(imname, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_multfb = read_sharedmem_image(imname);
    }

    if(aoconfID_limitb == -1)
    {
        if(sprintf(imname, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_limitb = read_sharedmem_image(imname);
    }



    // CONNECT to arrays holding gain, limit and multf values for individual modes
    if(aoconfID_DMmode_GAIN == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_GAIN", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_DMmode_GAIN = read_sharedmem_image(imname);
    }
    printf("aoconfID_DMmode_GAIN = %ld\n", aoconfID_DMmode_GAIN);


    if(aoconfID_LIMIT_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_LIMIT_modes = read_sharedmem_image(imname);
    }
    if(aoconfID_LIMIT_modes == -1)
        FILTERMODE = 0;


    if(aoconfID_MULTF_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_MULTF_modes = read_sharedmem_image(imname);
    }
    if(aoconfID_MULTF_modes == -1)
        FILTERMODE = 0;










    // predictive control output
    if(aoconfID_modevalPF == -1)
    {
		if(sprintf(imname, "aol%ld_modevalPF", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_modevalPF = read_sharedmem_image(imname);
		if(aoconfID_modevalPF != -1)
		{
			long ii;
			for(ii=0; ii<data.image[aoconfID_modevalPF].md[0].size[0]*data.image[aoconfID_modevalPF].md[0].size[1]; ii++)
				data.image[aoconfID_modevalPF].array.F[ii] = 0.0;
		}
	}

    // OUPUT
    sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizeout[0] = NBmodes;
    sizeout[1] = 1;

	// all images below are vectors of dimension NBmodes x 1

	// load/create aol_modeval_ol (pseudo-open loop mode values)
    if(sprintf(imname, "aol%ld_modeval_ol", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDout = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 20);


	// load/create aol_mode_blknb (block index for each mode)
    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDblknb = create_image_ID(imname, 2, sizeout, _DATATYPE_UINT16, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_corr (current modal DM correction)
    if(sprintf(imname, "aol%ld_modeval_dm_corr", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMcorr = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_now (current modal DM correction after mixing with predicitiv control)
    if(sprintf(imname, "aol%ld_modeval_dm_now", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnow = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_now_filt (current modal DM correction, filtered)
    if(sprintf(imname, "aol%ld_modeval_dm_now_filt", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnowfilt = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modeval_dm", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDM = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFsync", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodevalPFsync = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	

	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFres", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodevalPFres = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    

	// load/create WFS noise estimate
	if(sprintf(imname, "aol%ld_modeWFSnoise", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodeWFSnoise = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    

	
	
	//
	// load/create aol_mode_ARPFgain (mixing ratio between non-predictive and predictive mode values)
	// 0: adopt non-predictive value
	// 1: adopt predictive value
	//
	// Set values to 0 when predictive filter is off
	// set to 1 (or intermediate value) when predictive filter for corresponding mode is on
	//
    if(sprintf(imname, "aol%ld_mode_ARPFgain", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodeARPFgain = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	// initialize the gain to zero for all modes
	for(m=0;m<NBmodes;m++)
		data.image[IDmodeARPFgain].array.F[m] = 0.0;

	if(aoconfID_modeARPFgainAuto == -1)
	{
		// multiplicative auto ratio on top of gain above
		if(sprintf(imname, "aol%ld_mode_ARPFgainAuto", loop) < 1) 
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_modeARPFgainAuto = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
		COREMOD_MEMORY_image_set_createsem(imname, 10);
		// initialize the gain to zero for all modes
		for(m=0;m<NBmodes;m++)
			data.image[aoconfID_modeARPFgainAuto].array.F[m] = 1.0;
	}


    sizeout[1] = modeval_bsize;
    if(sprintf(imname, "aol%ld_modeval_dm_C", loop) < 1) // modal DM correction, circular buffer
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalDM_C = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);

 

	// modal prediction, circular buffer
    sizeout[1] = modeval_bsize;
    if(sprintf(imname, "aol%ld_modevalPF_C", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalPF_C = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	




    // auto limit tuning
    sizeout[0] = AOconf[loop].DMmodesNBblock;
    sizeout[1] = 1;
    if(sprintf(imname, "aol%ld_autotune_lim_bcoeff", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDatlimbcoeff = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);

    free(sizeout);

    printf("%ld modes\n", NBmodes);


    // Read from shared mem the DM mode files to indentify blocks
    data.image[IDblknb].md[0].write = 1;
    m = 0;
    blk = 0;
    while(m<NBmodes)
    {
		long n;
		long ID;
		
        if(sprintf(imname, "aol%ld_DMmodes%02ld", loop, blk) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        printf("Loading %s  (%2ld/%2ld)\n", imname, m, NBmodes);
        fflush(stdout);

        ID = read_sharedmem_image(imname);
        if(ID==-1)
        {
            printf("ERROR: could not load %s from shared memory\n", imname);
            exit(0);
        }
        n = data.image[ID].md[0].size[2];
        printf(" -> found %2ld modes\n", n);
        blockNBmodes[blk] = n;

        for(i=0; i<n; i++)
        {
            modeblock[m] = blk;
            data.image[IDblknb].array.UI16[m] = blk;
            m++;
        }
        blk++;
    }
    COREMOD_MEMORY_image_set_sempost_byID(IDblknb, -1);
    data.image[IDblknb].md[0].cnt0++;
    data.image[IDblknb].md[0].cnt1 = AOconf[loop].LOOPiteration;
    data.image[IDblknb].md[0].write = 0;






    framelatency = AOconf[loop].hardwlatency_frame + AOconf[loop].wfsmextrlatency_frame;
    framelatency0 = (long) framelatency;
    framelatency1 = framelatency0 + 1;
    alpha = framelatency - framelatency0;

    // initialize arrays
    data.image[IDmodevalDM].md[0].write = 1;
    data.image[IDmodevalDMcorr].md[0].write = 1;
    data.image[IDmodevalDMnow].md[0].write = 1;
    data.image[IDmodevalDM_C].md[0].write = 1;
    data.image[IDmodevalPF_C].md[0].write = 1;
    for(m=0; m<NBmodes; m++)
    {
        data.image[IDmodevalDM].array.F[m] = 0.0;
        data.image[IDmodevalDMcorr].array.F[m] = 0.0;
        data.image[IDmodevalDMnow].array.F[m] = 0.0;
        for(modevalDMindex=0; modevalDMindex<modeval_bsize; modevalDMindex++)
            data.image[IDmodevalDM_C].array.F[modevalDMindex*NBmodes+m] = 0;
        for(modevalPFindex=0; modevalPFindex<modeval_bsize; modevalPFindex++)
            data.image[IDmodevalPF_C].array.F[modevalPFindex*NBmodes+m] = 0;
        data.image[IDout].array.F[m] = 0.0;
    }
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMcorr, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnow, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM_C, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPF_C, -1);
    data.image[IDmodevalDM].md[0].cnt0++;
    data.image[IDmodevalDMcorr].md[0].cnt0++;
    data.image[IDmodevalDMnow].md[0].cnt0++;
    data.image[IDmodevalDM_C].md[0].cnt0++;
    data.image[IDmodevalPF_C].md[0].cnt0++;
    data.image[IDmodevalDM].md[0].write = 0;
    data.image[IDmodevalDMcorr].md[0].write = 0;
    data.image[IDmodevalDMnow].md[0].write = 0;
    data.image[IDmodevalDM_C].md[0].write = 0;
    data.image[IDmodevalPF_C].md[0].write = 0;

    printf("FILTERMODE = %d\n", FILTERMODE);
 

    modevalDMindex = 0;
    modevalDMindexl = 0;

    modevalPFindex = 0;
    modevalPFindexl = 0;

    cnt = 0;

    blockstatcnt = 0;
    for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
    {
		blockavePFresrms[block] = 0.0;
        blockaveOLrms[block] = 0.0;
        blockaveCrms[block] = 0.0;
        blockaveWFSrms[block] = 0.0;
        blockavelimFrac[block] = 0.0;
    }
    allaveOLrms = 0.0;
    allaveCrms = 0.0;
    allaveWFSrms = 0.0;
    allavelimFrac = 0.0;



	loopPFcnt = 0;
    for(;;)
    {		
		long modevalDMindex0, modevalDMindex1;
		long modevalPFindex0, modevalPFindex1;
		
		
        // read WFS measured modes (residual)
        if(data.image[IDmodeval].md[0].sem==0)
        {
            while(cnt==data.image[IDmodeval].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDmodeval].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDmodeval].semptr[4]);

        // drive sem4 to zero
        while(sem_trywait(data.image[IDmodeval].semptr[4])==0) {}
        AOconf[loop].statusM = 3;
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[3] = tdiffv;

		LOOPiter = data.image[IDmodeval].md[0].cnt1;


        // write gain, mult, limit into arrays
        for(m=0; m<NBmodes; m++)
        {
            modegain[m] = AOconf[loop].gain * data.image[aoconfID_gainb].array.F[modeblock[m]] * data.image[aoconfID_DMmode_GAIN].array.F[m];
            modemult[m] = AOconf[loop].mult * data.image[aoconfID_multfb].array.F[modeblock[m]] * data.image[aoconfID_MULTF_modes].array.F[m];
            modelimit[m] = AOconf[loop].maxlimit * data.image[aoconfID_limitb].array.F[modeblock[m]] * data.image[aoconfID_LIMIT_modes].array.F[m];
        }


        //
        // UPDATE CURRENT DM MODES STATE
        //
        //  current state =   modemult   x   ( last state   - modegain * WFSmodeval  )
        //
        // modevalDMindexl = last index in the IDmodevalDM_C buffer
        //
        
        data.image[IDmodevalDMcorr].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDMcorr].array.F[m] = modemult[m]*(data.image[IDmodevalDM_C].array.F[modevalDMindexl*NBmodes+m] - modegain[m]*data.image[IDmodeval].array.F[m]);
		COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMcorr, -1);
		data.image[IDmodevalDMcorr].md[0].cnt1 = LOOPiter; //modevalPFindex; //TBC
		data.image[IDmodevalDMcorr].md[0].cnt0++;
		data.image[IDmodevalDMcorr].md[0].write = 0;


        AOconf[loop].statusM = 4;
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[4] = tdiffv;

	

		int ARPF_ok;
		ARPF_ok = 0;

        //
        //  MIX PREDICTION WITH CURRENT DM STATE
        //
        if(AOconf[loop].ARPFon==1)
        {
		//	printf("%s  %s  %d\n",__FILE__, __func__, __LINE__);fflush(stdout); //TEST
			
            if(aoconfID_modevalPF==-1)
            {
                if(sprintf(imname, "aol%ld_modevalPF", loop) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                aoconfID_modevalPF = read_sharedmem_image(imname);
            }
            else
            {
				ARPF_ok=1;
                // don't wait for modevalPF... assume it's here early
                
                // if waiting for modevalPF, uncomment this line, and the "drive semaphore to zero" line after the next loop
                //sem_wait(data.image[IDmodevalPF].semptr[3]);

				//
				// prediction is mixed here with non-predictive output of WFS
				// minus sign required to apply correction on DM (correction should be opposite of WFS measurement)
				// note that second term (non-predictive) does not have minus sign, as it was already applied above
				//
				
				
                for(m=0; m<NBmodes; m++)
                {
					float mixratio;
					
					mixratio = AOconf[loop].ARPFgain*data.image[IDmodeARPFgain].array.F[m] * data.image[aoconfID_modeARPFgainAuto].array.F[m];
				    data.image[IDmodevalDMnow].array.F[m] = -mixratio*data.image[aoconfID_modevalPF].array.F[m]  + (1.0-mixratio)*data.image[IDmodevalDMcorr].array.F[m];
                }
             
                // drive semaphore to zero
				//  while(sem_trywait(data.image[aoconfID_modevalPF].semptr[3])==0) {}


				//
				// update current location of prediction circular buffer
				//
				data.image[IDmodevalPF_C].md[0].write = 1;
				for(m=0; m<NBmodes; m++)
					data.image[IDmodevalPF_C].array.F[modevalPFindex*NBmodes+m] = data.image[aoconfID_modevalPF].array.F[m];
				COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPF_C, -1);
				data.image[IDmodevalPF_C].md[0].cnt1 = modevalPFindex; // NEEDS TO CONTAIN WRITTEN SLICE ?
				data.image[IDmodevalPF_C].md[0].cnt0++;
				data.image[IDmodevalPF_C].md[0].write = 0;
					
					
					
				// autotune ARPFgainAuto
				data.image[aoconfID_modeARPFgainAuto].md[0].write = 1;
				for(m=0; m<NBmodes; m++)
				{
					float minVal = AOconf[loop].ARPFgainAutoMin;
					float maxVal = AOconf[loop].ARPFgainAutoMax;
					
					
					if (data.image[IDmodevalPFres].array.F[m]*data.image[IDmodevalPFres].array.F[m] < data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m])
						data.image[aoconfID_modeARPFgainAuto].array.F[m] *= 1.001;
					else
						data.image[aoconfID_modeARPFgainAuto].array.F[m] *= 0.999;
						
					if (data.image[aoconfID_modeARPFgainAuto].array.F[m] > maxVal)
						data.image[aoconfID_modeARPFgainAuto].array.F[m] = maxVal;
					if (data.image[aoconfID_modeARPFgainAuto].array.F[m] < minVal)
						data.image[aoconfID_modeARPFgainAuto].array.F[m] = minVal;
				}
				data.image[aoconfID_modeARPFgainAuto].md[0].cnt1 = LOOPiter; //modevalPFindex;
				data.image[aoconfID_modeARPFgainAuto].md[0].cnt0++;
				data.image[aoconfID_modeARPFgainAuto].md[0].write = 0;
								
				loopPFcnt++;
            }
        }

		if (ARPF_ok==0)
		{ 
			memcpy(data.image[IDmodevalDMnow].array.F, data.image[IDmodevalDMcorr].array.F, sizeof(float)*NBmodes);
		}






        AOconf[loop].statusM = 5;
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoconfID_looptiming].array.F[5] = tdiffv;

        data.image[IDmodevalDMnowfilt].md[0].write = 1;
        // FILTERING MODE VALUES
        // THIS FILTERING GOES TOGETHER WITH THE SECONDARY WRITE ON DM TO KEEP FILTERED AND ACTUAL VALUES IDENTICAL
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDMnowfilt].array.F[m] = data.image[IDmodevalDMnow].array.F[m];


        if(AOconf[loop].AUTOTUNE_LIMITS_ON==1) // automatically adjust modal limits
        {
            data.image[IDatlimbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
                limitblockarray[block] = 0.0;

            data.image[aoconfID_LIMIT_modes].md[0].write = 1;
            data.image[aoconfID_limitb].md[0].write = 1;

			// Adjust limit for EACH mode
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                if(  fabs(AOconf[loop].AUTOTUNE_LIMITS_mcoeff*data.image[IDmodevalDMnowfilt].array.F[m]) > modelimit[m])
                    data.image[aoconfID_LIMIT_modes].array.F[m] *= (1.0 + AOconf[loop].AUTOTUNE_LIMITS_delta);
                else
                    data.image[aoconfID_LIMIT_modes].array.F[m] *= (1.0 - AOconf[loop].AUTOTUNE_LIMITS_delta*0.01*AOconf[loop].AUTOTUNE_LIMITS_perc);

                limitblockarray[block] += data.image[aoconfID_LIMIT_modes].array.F[m];
            }
            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_LIMIT_modes, -1);
            data.image[aoconfID_LIMIT_modes].md[0].cnt0++;
            data.image[aoconfID_LIMIT_modes].md[0].cnt1 = LOOPiter;
            data.image[aoconfID_LIMIT_modes].md[0].write = 0;

			// update block limits to drive average limit coefficients to 1
            data.image[IDatlimbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
                data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AUTOTUNE_LIMITS_delta*0.1 );
                if(coeff < 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta;
                if(coeff> 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta;
                data.image[aoconfID_limitb].array.F[block] = data.image[aoconfID_limitb].array.F[block] * coeff;
            }
            COREMOD_MEMORY_image_set_sempost_byID(IDatlimbcoeff, -1);
            data.image[IDatlimbcoeff].md[0].cnt0++;
            data.image[IDatlimbcoeff].md[0].cnt1 = LOOPiter;
            data.image[IDatlimbcoeff].md[0].write = 0;


            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_limitb, -1);
            data.image[aoconfID_limitb].md[0].cnt0++;
            data.image[aoconfID_limitb].md[0].cnt1 = LOOPiter;
            data.image[aoconfID_limitb].md[0].write = 0;

        }


		if(AOconf[loop].AUTOTUNE_GAINS_ON==1)
		{
			// CONNECT to auto gain input
			if(IDautogain == -1)
			{
				if(sprintf(imname, "aol%ld_autogain", loop) < 1)
					printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

				IDautogain = read_sharedmem_image(imname);				
			}
			
			if(IDautogain != -1)
			{
				if(data.image[IDautogain].md[0].cnt0 != autogainCnt)
				{
					float maxGainVal = 0.0;
					float globalgain = 0.0;
					char command[500];
					
					// New gains available - updating
					printf("[%ld %s] Updated autogain [%12ld  %12ld] -> applying gains\n", IDautogain, data.image[IDautogain].md[0].name, (long) autogainCnt, (long) data.image[IDautogain].md[0].cnt0);
					fflush(stdout);
					
					// Set global gain to highest gain
					for(m=0;m<NBmodes;m++)
					{
						if(data.image[IDautogain].array.F[m] > maxGainVal)
							maxGainVal = data.image[IDautogain].array.F[m];
					}					
					globalgain = maxGainVal;
					printf("     Setting  global gain = %f\n", maxGainVal);
					AOconf[loop].gain = maxGainVal;

					sprintf(command, "echo \"%6.4f\" > conf/param_loopgain.txt", AOconf[loop].gain);
					system(command);


					
					// Set block gain to max gain within block, scaled to global gain
					for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
					{
						maxGainVal = 0.0;
						for(m=0; m<NBmodes; m++)
							if(data.image[IDblknb].array.UI16[m] == block)
								if(data.image[IDautogain].array.F[m] > maxGainVal)
									maxGainVal = data.image[IDautogain].array.F[m];
					
						printf("Set block %2ld gain to  %f\n", block, maxGainVal/globalgain);
					
						data.image[aoconfID_gainb].array.F[block] = maxGainVal/AOconf[loop].gain;

						
						sprintf(command, "echo \"%6.4f\" > conf/param_gainb%02ld.txt", data.image[aoconfID_gainb].array.F[block], block);
						system(command);
					}
					
					// Set individual gain
					for(m=0;m<NBmodes;m++)
					{
						data.image[aoconfID_DMmode_GAIN].array.F[m] = data.image[IDautogain].array.F[m]/data.image[aoconfID_gainb].array.F[modeblock[m]]/AOconf[loop].gain;
						
						if(m<20)
							printf("Mode %3ld   %12f  %12f  %12f ->   %12f  %12f\n", m, AOconf[loop].gain, data.image[aoconfID_gainb].array.F[modeblock[m]], data.image[aoconfID_DMmode_GAIN].array.F[m], AOconf[loop].gain*data.image[aoconfID_gainb].array.F[modeblock[m]]*data.image[aoconfID_DMmode_GAIN].array.F[m], data.image[IDautogain].array.F[m]);
					}
					
					
					
					autogainCnt = data.image[IDautogain].md[0].cnt0;
				}
				
			}
	
/*			float alphagain = 0.1;
			
			// if update available
			
			data.image[aoconfID_GAIN_modes].md[0].write = 1;
            data.image[aoconfID_gaiinb].md[0].write = 1;

			// Adjust gain for EACH mode
			
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                data.image[aoconfID_GAIN_modes].array.F[m] = (1.0-alphagain)*data.image[aoconfID_GAIN_modes].array.F[m] + alphagain*data.image[autogain].array.F[m];

                gainblockarray[block] += data.image[aoconfID_GAIN_modes].array.F[m];
            }
            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_GAIN_modes, -1);
            data.image[aoconfID_GAIN_modes].md[0].cnt0++;
            data.image[aoconfID_GAIN_modes].md[0].write = 0;

			// update block gains to drive average gain coefficients to 1
            data.image[IDatgainbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
                data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AUTOTUNE_LIMITS_delta*0.1 );
                if(coeff < 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta;
                if(coeff> 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta;
                data.image[aoconfID_limitb].array.F[block] = data.image[aoconfID_limitb].array.F[block] * coeff;
            }
            COREMOD_MEMORY_image_set_sempost_byID(IDatlimbcoeff, -1);
            data.image[IDatlimbcoeff].md[0].cnt0++;
            data.image[IDatlimbcoeff].md[0].write = 0;


            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_limitb, -1);
            data.image[aoconfID_limitb].md[0].cnt0++;
            data.image[aoconfID_limitb].md[0].write = 0;
            */
		}



        if(FILTERMODE == 1)
        {
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                if(data.image[IDmodevalDMnowfilt].array.F[m] > modelimit[m])
                {
                    blockavelimFrac[block] += 1.0;
                    data.image[IDmodevalDMnowfilt].array.F[m] = modelimit[m];
                }
                if(data.image[IDmodevalDMnowfilt].array.F[m] < -modelimit[m])
                {
                    blockavelimFrac[block] += 1.0;
                    data.image[IDmodevalDMnowfilt].array.F[m] = -modelimit[m];
                }
            }
        }



        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnow, -1);
        data.image[IDmodevalDMnow].md[0].cnt1 = LOOPiter; //modevalDMindex; //TBC
        data.image[IDmodevalDMnow].md[0].cnt0++;
        data.image[IDmodevalDMnow].md[0].write = 0;

		if(AOconf[loop].DMfilteredWriteON==1)
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnowfilt, -1);
        else
			{
				// DM write triggered by sem 2
				// posting all sems except 2 to block DM write
				COREMOD_MEMORY_image_set_sempost_excl_byID(IDmodevalDMnowfilt, 2);
			}
        data.image[IDmodevalDMnowfilt].md[0].cnt1 = LOOPiter; //modevalDMindex; //TBC
        data.image[IDmodevalDMnowfilt].md[0].cnt0++;
        data.image[IDmodevalDMnowfilt].md[0].write = 0;

		// IF MODAL DM, AND FILTERED DM WRITE IS ON, SEND TO DM
		if((AOconf[loop].DMfilteredWriteON==1) && (AOconf[loop].DMMODE==1))
		{
			data.image[aoconfID_dmC].md[0].write = 1;
			memcpy(data.image[aoconfID_dmC].array.F, data.image[IDmodevalDMnowfilt].array.F, sizeof(float)*NBmodes);
			COREMOD_MEMORY_image_set_sempost_byID(aoconfID_dmC, -1);
			data.image[aoconfID_dmC].md[0].cnt1 = LOOPiter;
			data.image[aoconfID_dmC].md[0].cnt0++;
			data.image[aoconfID_dmC].md[0].write = 0;			
		}


        AOconf[loop].statusM = 6;
        
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[6] = tdiffv;
        
		AOconf[loop].statusM1 = 0;


        //
        // update current location of dm correction circular buffer
        //
        data.image[IDmodevalDM_C].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDM_C].array.F[modevalDMindex*NBmodes+m] = data.image[IDmodevalDMnowfilt].array.F[m];
        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM_C, -1);
        data.image[IDmodevalDM_C].md[0].cnt1 = LOOPiter; //modevalDMindex;
        data.image[IDmodevalDM_C].md[0].cnt0++;
        data.image[IDmodevalDM_C].md[0].write = 0;



        AOconf[loop].statusM1 = 1;

        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[9] = tdiffv;

        //
        // COMPUTE DM STATE AT TIME OF WFS MEASUREMENT
        // LINEAR INTERPOLATION BETWEEN NEAREST TWO VALUES
        //
        modevalDMindex0 = modevalDMindex - framelatency0;
        if(modevalDMindex0<0)
            modevalDMindex0 += modeval_bsize;
        modevalDMindex1 = modevalDMindex - framelatency1;
        if(modevalDMindex1<0)
            modevalDMindex1 += modeval_bsize;

        data.image[IDmodevalDM].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDM].array.F[m] = (1.0-alpha)*data.image[IDmodevalDM_C].array.F[modevalDMindex0*NBmodes+m] + alpha*data.image[IDmodevalDM_C].array.F[modevalDMindex1*NBmodes+m];
        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM, -1);
        data.image[IDmodevalDM].md[0].cnt0++;
        data.image[IDmodevalDM].md[0].cnt1 = LOOPiter;
        data.image[IDmodevalDM].md[0].write = 0;

		AOconf[loop].statusM1 = 2;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[10] = tdiffv;



		if(AOconf[loop].ARPFon==1)
		{
			//
			// COMPUTE OPEN LOOP PREDICTION AT TIME OF WFS MEASUREMENT
			// LINEAR INTERPOLATION BETWEEN NEAREST TWO VALUES
			//
        
			modevalPFindex0 = modevalPFindex - framelatency0;
			if(modevalPFindex0<0)
				modevalPFindex0 += modeval_bsize;
			modevalPFindex1 = modevalPFindex - framelatency1;
			if(modevalPFindex1<0)
				modevalPFindex1 += modeval_bsize;

			data.image[IDmodevalPFsync].md[0].write = 1;
			for(m=0; m<NBmodes; m++)
				data.image[IDmodevalPFsync].array.F[m] = (1.0-alpha)*data.image[IDmodevalPF_C].array.F[modevalPFindex0*NBmodes+m] + alpha*data.image[IDmodevalPF_C].array.F[modevalPFindex1*NBmodes+m];
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPFsync, -1);
			data.image[IDmodevalPFsync].md[0].cnt0++;
			data.image[IDmodevalPFsync].md[0].cnt1 = LOOPiter;
			data.image[IDmodevalPFsync].md[0].write = 0;
		}



        AOconf[loop].statusM1 = 3;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[11] = tdiffv;


        //
        // OPEN LOOP STATE = most recent WFS reading - time-lagged DM
        //
        data.image[IDout].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDout].array.F[m] = data.image[IDmodeval].array.F[m] - data.image[IDmodevalDM].array.F[m];
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = LOOPiter;
        data.image[IDout].md[0].write = 0;


		if(AOconf[loop].ARPFon==1)
		{
			//
			// OPEN LOOP PREDICTION RESIDUAL = most recent OL - time-lagged PF
			//
			data.image[IDmodevalPFres].md[0].write = 1;
			for(m=0; m<NBmodes; m++)
				data.image[IDmodevalPFres].array.F[m] = data.image[IDout].array.F[m] - data.image[IDmodevalPFsync].array.F[m];
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPFres, -1);
			data.image[IDmodevalPFres].md[0].cnt0++;
			data.image[IDmodevalPFres].md[0].cnt1 = LOOPiter;
			data.image[IDmodevalPFres].md[0].write = 0;
		}



        AOconf[loop].statusM1 = 4;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[12] = tdiffv;


		// increment modevalDMindex
        modevalDMindexl = modevalDMindex;
        modevalDMindex++;
        if(modevalDMindex==modeval_bsize)
            modevalDMindex = 0;

		// increment modevalPFindex
        modevalPFindexl = modevalPFindex;
        modevalPFindex++;
        if(modevalPFindex==modeval_bsize)
            modevalPFindex = 0;





        // TELEMETRY
        for(m=0; m<NBmodes; m++)
        {
            block = data.image[IDblknb].array.UI16[m];
			
			
			blockavePFresrms[block] += data.image[IDmodevalPFres].array.F[m]*data.image[IDmodevalPFres].array.F[m];
            blockaveOLrms[block] += data.image[IDout].array.F[m]*data.image[IDout].array.F[m];
            blockaveCrms[block] += data.image[IDmodevalDMnow].array.F[m]*data.image[IDmodevalDMnow].array.F[m];
            blockaveWFSrms[block] += data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m];          
			
			blockaveWFSnoise[block] += data.image[IDmodeWFSnoise].array.F[m];
        }
        

        blockstatcnt ++;
        if(blockstatcnt == AOconf[loop].AveStats_NBpt)
        {
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
				AOconf[loop].blockave_PFresrms[block] = sqrt(blockavePFresrms[block]/blockstatcnt);
                AOconf[loop].blockave_OLrms[block] = sqrt(blockaveOLrms[block]/blockstatcnt);
                AOconf[loop].blockave_Crms[block] = sqrt(blockaveCrms[block]/blockstatcnt);
                AOconf[loop].blockave_WFSrms[block] = sqrt(blockaveWFSrms[block]/blockstatcnt);
				AOconf[loop].blockave_WFSnoise[block] = sqrt(blockaveWFSnoise[block]/blockstatcnt);
                AOconf[loop].blockave_limFrac[block] = (blockavelimFrac[block])/blockstatcnt;

				allavePFresrms += blockavePFresrms[block];
                allaveOLrms += blockaveOLrms[block];
                allaveCrms += blockaveCrms[block];
                allaveWFSrms += blockaveWFSrms[block];
                allaveWFSnoise += blockaveWFSnoise[block];
                allavelimFrac += blockavelimFrac[block];

				blockavePFresrms[block] = 0.0;
                blockaveOLrms[block] = 0.0;
                blockaveCrms[block] = 0.0;
                blockaveWFSrms[block] = 0.0;
                blockaveWFSnoise[block] = 0.0;
                blockavelimFrac[block] = 0.0;                
            }

            AOconf[loop].ALLave_OLrms = sqrt(allaveOLrms/blockstatcnt);
            AOconf[loop].ALLave_Crms = sqrt(allaveCrms/blockstatcnt);
            AOconf[loop].ALLave_WFSrms = sqrt(allaveWFSrms/blockstatcnt);
			AOconf[loop].ALLave_WFSnoise = sqrt(allaveWFSnoise/blockstatcnt);
            AOconf[loop].ALLave_limFrac = allavelimFrac/blockstatcnt;

			allavePFresrms = 0.0;
            allaveOLrms = 0.0;
            allaveCrms = 0.0;
            allaveWFSrms = 0.0;
            allavelimFrac = 0.0;

            blockstatcnt = 0;
        }

		


        AOconf[loop].statusM1 = 5;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[13] = tdiffv;

    }

    free(modegain);
    free(modemult);
    free(modelimit);

    free(modeblock);

    return(IDout);
}







//
// gains autotune
//
// input: modeval_ol
// APPLIES new gain values if AUTOTUNE_GAINS_ON
//
int_fast8_t AOloopControl_AutoTuneGains(long loop, const char *IDout_name, float GainCoeff, long NBsamples)
{
    long IDmodevalOL;
    long IDmodeval;
    long IDmodeval_dm;
    long IDmodeval_dm_now;
    long IDmodeval_dm_now_filt;
	long IDmodeWFSnoise;

    long NBmodes;
    char imname[200];
    long m;
    double diff1, diff2, diff3, diff4;
    float *array_mvalOL1;
    float *array_mvalOL2;
    float *array_mvalOL3;
    float *array_mvalOL4;
    double *array_sig1;
    double *array_sig2;
    double *array_sig3;
    double *array_sig4;
    float *array_sig;
    float *array_asq;
    long double *ave0;
    long double *sig0;
    long double *sig1;
    long double *sig2;
    long double *sig3;
    long double *sig4;
    float *stdev;

    float gain;
    long NBgain;
    long kk;
    float *errarray;
    float mingain = 0.01;
    float maxgain = 0.3;
    float gainFactor = 0.6; // advise user to be at 60% of optimal gain
    float gainfactstep = 1.02;
    float *gainval_array;
    float *gainval1_array;
    float *gainval2_array;

    long long cnt = 0;
    float latency;
    FILE *fp;

    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;


    long IDout;
    uint32_t *sizearray;

    float gain0; // corresponds to evolution timescale
    long cntstart;



    long IDblk;
    float *modegain;
    float *modemult;


    float *NOISEfactor;
    long IDsync;


    int TESTMODE = 1;
    int TEST_m = 30;
    FILE *fptest;
    
    long iter;
    float GainCoeff1 = 1.0;
    
    


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


    printf("AUTO GAIN\n");
    fflush(stdout);


    // read AO loop gain, mult
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


	AOconf[loop].AUTOTUNEGAINS_updateGainCoeff = GainCoeff;
	AOconf[loop].AUTOTUNEGAINS_NBsamples = NBsamples;



    gain0 = 1.0/(AOconf[loop].loopfrequ*AOconf[loop].AUTOTUNEGAINS_evolTimescale);




    // CONNECT to arrays holding gain, limit, and multf values for blocks

    if(aoconfID_gainb == -1)
    {
        if(sprintf(imname, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_gainb = read_sharedmem_image(imname);
    }

    if(aoconfID_multfb == -1)
    {
        if(sprintf(imname, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_multfb = read_sharedmem_image(imname);
    }

    // CONNECT to arrays holding gain, limit and multf values for individual modes

    if(aoconfID_DMmode_GAIN == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_GAIN", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_DMmode_GAIN = read_sharedmem_image(imname);
    }
    printf("aoconfID_DMmode_GAIN = %ld\n", aoconfID_DMmode_GAIN);

    if(aoconfID_MULTF_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoconfID_MULTF_modes = read_sharedmem_image(imname);
    }



    // INPUT
    if(sprintf(imname, "aol%ld_modeval_ol", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalOL = read_sharedmem_image(imname);
    NBmodes = data.image[IDmodevalOL].md[0].size[0];

    if(sprintf(imname, "aol%ld_modeval", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm_now", loop) < 1) // current modal DM correction
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm_now = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm_now_filt", loop) < 1) // current modal DM correction, filtered
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm_now_filt = read_sharedmem_image(imname);



	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
	sizearray = (uint32_t *) malloc(sizeof(uint32_t)*2);
	sizearray[0] = NBmodes;
	sizearray[1] = 1;
	if(sprintf(imname, "aol%ld_modeWFSnoise", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodeWFSnoise = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	free(sizearray);

    // blocks
    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) // block indices
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblk = read_sharedmem_image(imname);


    modegain = (float*) malloc(sizeof(float)*NBmodes);
    modemult = (float*) malloc(sizeof(float)*NBmodes);
    NOISEfactor = (float*) malloc(sizeof(float)*NBmodes);





    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);
    sizearray[0] = NBmodes;
    sizearray[1] = 1;
    IDout = create_image_ID(IDout_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDout_name, 10);
    free(sizearray);




    // last open loop move values
    array_mvalOL1 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL2 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL3 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL4 = (float*) malloc(sizeof(float)*NBmodes);
    array_sig1 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig2 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig3 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig4 = (double*) malloc(sizeof(double)*NBmodes);

    array_sig = (float*) malloc(sizeof(float)*NBmodes);
    array_asq = (float*) malloc(sizeof(float)*NBmodes);
    ave0 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig0 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig1 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig2 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig3 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig4 = (long double*) malloc(sizeof(long double)*NBmodes);
    stdev = (float*) malloc(sizeof(float)*NBmodes);


    gainval_array = (float*) malloc(sizeof(float)*NBgain);
    gainval1_array = (float*) malloc(sizeof(float)*NBgain);
    gainval2_array = (float*) malloc(sizeof(float)*NBgain);

    errarray = (float*) malloc(sizeof(float)*NBgain);



	iter = 0;
    for(;;)
    {

        // write gain, mult into arrays
        for(m=0; m<NBmodes; m++)
        {
            unsigned short block;

            block = data.image[IDblk].array.UI16[m];
            modegain[m] = AOconf[loop].gain * data.image[aoconfID_gainb].array.F[block] * data.image[aoconfID_DMmode_GAIN].array.F[m];
            modemult[m] = AOconf[loop].mult * data.image[aoconfID_multfb].array.F[block] * data.image[aoconfID_MULTF_modes].array.F[m];
            NOISEfactor[m] = 1.0 + modemult[m]*modemult[m]*modegain[m]*modegain[m]/(1.0-modemult[m]*modemult[m]);
        }







        // prepare gain array
        latency = AOconf[loop].hardwlatency_frame + AOconf[loop].wfsmextrlatency_frame;
        //printf("latency = %f frame\n", latency);
        NBgain = 0;
        gain = mingain;
        while(gain<maxgain/gainFactor)
        {
            gain *= gainfactstep;
            NBgain++;
        }


        kk = 0;
        gain = mingain;
        while(kk<NBgain)
        {
            gainval_array[kk] = gain;
            gainval1_array[kk] = (latency + 1.0/gain)*(latency + 1.0/(gain+gain0));
            gainval2_array[kk] = (gain/(1.0-gain));

            //printf("gain   %4ld  %12f   %12f  %12f\n", kk, gainval_array[kk], gainval1_array[kk], gainval2_array[kk]);
            gain *= gainfactstep;
            kk++;
        }



        // drive sem5 to zero
        while(sem_trywait(data.image[IDmodevalOL].semptr[5])==0) {}



        for(m=0; m<NBmodes; m++)
        {
            array_mvalOL1[m] = 0.0;
            array_mvalOL2[m] = 0.0;
            array_sig1[m] = 0.0;
            array_sig2[m] = 0.0;
            ave0[m] = 0.0;
            sig0[m] = 0.0;
            sig1[m] = 0.0;
            sig2[m] = 0.0;
            sig3[m] = 0.0;
            sig4[m] = 0.0;
            stdev[m] = 0.0;
        }




        if(TESTMODE==1)
            fptest = fopen("test_autotunegain.dat", "w");

        cnt = 0;
        cntstart = 10;
        while(cnt<AOconf[loop].AUTOTUNEGAINS_NBsamples)
        {
            sem_wait(data.image[IDmodevalOL].semptr[5]);


            data.image[IDout].md[0].write = 1;

            for(m=0; m<NBmodes; m++)
            {
                diff1 = data.image[IDmodevalOL].array.F[m] - array_mvalOL1[m];
                diff2 = data.image[IDmodevalOL].array.F[m] - array_mvalOL2[m];
                diff3 = data.image[IDmodevalOL].array.F[m] - array_mvalOL3[m];
                diff4 = data.image[IDmodevalOL].array.F[m] - array_mvalOL4[m];
                array_mvalOL4[m] = array_mvalOL3[m];
                array_mvalOL3[m] = array_mvalOL2[m];
                array_mvalOL2[m] = array_mvalOL1[m];
                array_mvalOL1[m] = data.image[IDmodevalOL].array.F[m];

                if(cnt>cntstart)
                {
                    ave0[m] += data.image[IDmodevalOL].array.F[m];
                    sig0[m] += data.image[IDmodevalOL].array.F[m]*data.image[IDmodevalOL].array.F[m];
                    sig1[m] += diff1*diff1;
                    sig2[m] += diff2*diff2;
                    sig3[m] += diff3*diff3;
                    sig4[m] += diff4*diff4;
                }
            }

            if(TESTMODE==1)
                fprintf(fptest, "%5lld %+12.10f %+12.10f %+12.10f %+12.10f %+12.10f\n", cnt, data.image[IDmodeval].array.F[TEST_m], data.image[IDmodevalOL].array.F[TEST_m], data.image[IDmodeval_dm].array.F[TEST_m], data.image[IDmodeval_dm_now].array.F[TEST_m], data.image[IDmodeval_dm_now_filt].array.F[TEST_m]);

            cnt++;
        }
        if(TESTMODE==1)
            fclose(fptest);

		

		GainCoeff1 = 1.0/(iter+1);
		if(GainCoeff1 < AOconf[loop].AUTOTUNEGAINS_updateGainCoeff)
			GainCoeff1 = AOconf[loop].AUTOTUNEGAINS_updateGainCoeff;
		

        data.image[IDout].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
        {
            long kkmin;
            float errmin;

            ave0[m] /= cnt-cntstart;
            sig0[m] /= cnt-cntstart;
            array_sig1[m] = sig1[m]/(cnt-cntstart);
            array_sig2[m] = sig2[m]/(cnt-cntstart);
            array_sig3[m] = sig3[m]/(cnt-cntstart);
            array_sig4[m] = sig4[m]/(cnt-cntstart);


            //		array_asq[m] = (array_sig2[m]-array_sig1[m])/3.0;
            //        array_asq[m] = (array_sig4[m]-array_sig1[m])/15.0;

            // This formula is compatible with astromgrid, which alternates between patterns
            array_asq[m] = (array_sig4[m]-array_sig2[m])/12.0;
            if(array_asq[m]<0.0)
                array_asq[m] = 0.0;

			// WFS variance
            //array_sig[m] = (4.0*array_sig1[m] - array_sig2[m])/6.0;
            // This formula is compatible with astromgrid, which alternates between patterns
            array_sig[m] = (4.0*array_sig2[m] - array_sig4[m])/6.0;

            stdev[m] = sig0[m] - NOISEfactor[m]*array_sig[m] - ave0[m]*ave0[m];
            if(stdev[m]<0.0)
                stdev[m] = 0.0;
            stdev[m] = sqrt(stdev[m]);

            for(kk=0; kk<NBgain; kk++)
                errarray[kk] = array_asq[m] * gainval1_array[kk] + array_sig[m] * gainval2_array[kk];

            errmin = errarray[0];
            kkmin = 0;

            for(kk=0; kk<NBgain; kk++)
                if(errarray[kk]<errmin)
                {
                    errmin = errarray[kk];
                    kkmin = kk;
                }
			
			
            data.image[IDout].array.F[m] = (1.0-GainCoeff1) * data.image[IDout].array.F[m]   +  GainCoeff1 * (gainFactor*gainval_array[kkmin]);
            
        }

        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[IDout].md[0].write = 0;


		// write noise
		data.image[IDmodeWFSnoise].md[0].write = 1;
		data.image[IDmodeWFSnoise].md[0].cnt0++;
		for(m=0;m<NBmodes;m++)
			data.image[IDmodeWFSnoise].array.F[m] = array_sig[m];
		COREMOD_MEMORY_image_set_sempost_byID(IDmodeWFSnoise, -1);
		data.image[IDmodeWFSnoise].md[0].cnt1 = AOconf[loop].LOOPiteration;
		data.image[IDmodeWFSnoise].md[0].write = 0;


        if(AOconf[loop].AUTOTUNE_GAINS_ON==1) // automatically adjust gain values
        {

        }


        fp = fopen("optgain.dat", "w");
        for(m=0; m<NBmodes; m++)
            fprintf(fp, "%5ld   %+12.10f %12.10f %12.10f %12.10f %12.10f   %6.4f  %16.14f %16.14f  %6.2f\n", m, (float) ave0[m], (float) sig0[m], stdev[m], sqrt(array_asq[m]), sqrt(array_sig[m]), data.image[IDout].array.F[m], array_sig1[m], array_sig4[m], NOISEfactor[m]);
        fclose(fp);
        
        printf("[%8ld]  %8ld   %8.6f -> %8.6f\n", iter, AOconf[loop].AUTOTUNEGAINS_NBsamples, AOconf[loop].AUTOTUNEGAINS_updateGainCoeff, GainCoeff1);
        
        
        
     
        iter++;

    }

    free(gainval_array);
    free(gainval1_array);
    free(gainval2_array);
    free(errarray);

    free(array_mvalOL1);
    free(array_mvalOL2);
    free(array_mvalOL3);
    free(array_mvalOL4);
    free(ave0);
    free(sig0);
    free(sig1);
    free(sig2);
    free(sig3);
    free(sig4);
    free(array_sig1);
    free(array_sig2);
    free(array_sig3);
    free(array_sig4);
    free(array_sig);
    free(array_asq);

    free(modegain);
    free(modemult);
    free(NOISEfactor);

    free(stdev);

    return(0);
}










long AOloopControl_dm2dm_offload(const char *streamin, const char *streamout, float twait, float offcoeff, float multcoeff)
{
    long IDin, IDout;
    long cnt = 0;
    long xsize, ysize, xysize;
    long ii;
    //long IDtmp;
	char imname[200];

    IDin = image_ID(streamin);
    IDout = image_ID(streamout);

    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
    xysize = xsize*ysize;
    


	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    while(1)
    {
        printf("%8ld : offloading   %s -> %s\n", cnt, streamin, streamout);

        data.image[IDout].md[0].write = 1;
        for(ii=0; ii<xysize; ii++)
            data.image[IDout].array.F[ii] = multcoeff*(data.image[IDout].array.F[ii] + offcoeff*data.image[IDin].array.F[ii]);
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDout].md[0].write = 0;

        usleep((long) (1000000.0*twait));
        cnt++;
    }

    return(IDout);
}









/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 8.   LOOP CONTROL INTERFACE                                              */
/* =============================================================================================== */
/* =============================================================================================== */




int_fast8_t AOloopControl_setLoopNumber(long loop)
{


    printf("LOOPNUMBER = %ld\n", loop);
    LOOPNUMBER = loop;

    /** append process name with loop number */


    return 0;
}


int_fast8_t AOloopControl_setparam(long loop, const char *key, double value)
{
    int pOK=0;
    char kstring[200];



    strcpy(kstring, "PEperiod");
    if((strncmp (key, kstring, strlen(kstring)) == 0)&&(pOK==0))
    {
        //AOconf[loop].WFScamPEcorr_period = (long double) value;
        pOK = 1;
    }

    if(pOK==0)
        printf("Parameter not found\n");



    return (0);
}





/* =============================================================================================== */
/** @name AOloopControl - 8.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET  */
/* =============================================================================================== */


int_fast8_t AOloopControl_loopon()
{

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].cntmax = AOconf[LOOPNUMBER].cnt-1;

    AOconf[LOOPNUMBER].on = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopoff()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].on = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopkill()
{

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].kill = 1;

    return 0;
}


int_fast8_t AOloopControl_loopstep(long loop, long NBstep)
{

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[loop].cntmax = AOconf[loop].cnt + NBstep;
    AOconf[LOOPNUMBER].RMSmodesCumul = 0.0;
    AOconf[LOOPNUMBER].RMSmodesCumulcnt = 0;

    AOconf[loop].on = 1;

    while(AOconf[loop].on==1)
        usleep(100); // THIS WAITING IS OK


    return 0;
}


int_fast8_t AOloopControl_loopreset()
{
    long k;
    long mb;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_cmd_modes==-1)
    {
        char name[200];
        if(sprintf(name, "DMmode_cmd_%ld", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    AOconf[LOOPNUMBER].on = 0;
    for(k=0; k<AOconf[LOOPNUMBER].NBDMmodes; k++)
        data.image[aoconfID_cmd_modes].array.F[k] = 0.0;

    for(mb=0; mb<AOconf[LOOPNUMBER].DMmodesNBblock; mb)
    {
        AOloopControl_setgainblock(mb, 0.0);
        AOloopControl_setlimitblock(mb, 0.01);
        AOloopControl_setmultfblock(mb, 0.95);
    }

    return 0;
}



/* =============================================================================================== */
/** @name AOloopControl - 8.2. LOOP CONTROL INTERFACE - DATA LOGGING                               */
/* =============================================================================================== */



/* =============================================================================================== */
/** @name AOloopControl - 8.3. LOOP CONTROL INTERFACE - PRIMARY AND FILTERED DM WRITE                           */
/* =============================================================================================== */

int_fast8_t AOloopControl_DMprimaryWrite_on()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].DMprimaryWriteON = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_DMprimaryWrite_off()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].DMprimaryWriteON = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_DMfilteredWrite_on()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].DMfilteredWriteON = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_DMfilteredWrite_off()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].DMfilteredWriteON = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}



/* =============================================================================================== */
/** @name AOloopControl - 8.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING                     */
/* =============================================================================================== */



int_fast8_t AOloopControl_AUTOTUNE_LIMITS_on()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_LIMITS_ON = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}



int_fast8_t AOloopControl_AUTOTUNE_LIMITS_off()
{
	int block;
	int NBblock;
	
	
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_LIMITS_ON = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);
    
    
   if(aoconfID_limitb == -1)
    {
		char imname[200];
		
        if(sprintf(imname, "aol%ld_limitb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_limitb = read_sharedmem_image(imname);
    }

    NBblock = data.image[aoconfID_limitb].md[0].size[0];

    // Save Limits
    for(block=0; block<NBblock; block++)
		{
			FILE *fp;
			char fname[200];
			
			sprintf(fname, "conf/param_limitb%02d.txt", block);
			
			if((fp=fopen(fname, "w"))==NULL)
				printERROR(__FILE__, __func__, __LINE__, "Cannot open file");
			else
			{
				fprintf(fp, "%7.5f\n", data.image[aoconfID_limitb].array.F[block]);
			}
			
			fclose(fp);
		}

    return 0;
}




int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_delta(float AUTOTUNE_LIMITS_delta)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_LIMITS_delta = AUTOTUNE_LIMITS_delta;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}



int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_perc(float AUTOTUNE_LIMITS_perc)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_LIMITS_perc = AUTOTUNE_LIMITS_perc;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}

int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(float AUTOTUNE_LIMITS_mcoeff)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_LIMITS_mcoeff = AUTOTUNE_LIMITS_mcoeff;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}

int_fast8_t AOloopControl_AUTOTUNE_GAINS_on()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_GAINS_ON = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_AUTOTUNE_GAINS_off()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].AUTOTUNE_GAINS_ON = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}





/* =============================================================================================== */
/** @name AOloopControl - 8.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF                   */
/* =============================================================================================== */


int_fast8_t AOloopControl_ARPFon()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].ARPFon = 1;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_ARPFoff()
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].ARPFon = 0;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}



/* =============================================================================================== */
/** @name AOloopControl - 8.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS                          */
/* =============================================================================================== */



int_fast8_t AOloopControl_set_loopfrequ(float loopfrequ)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].loopfrequ = loopfrequ;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_hardwlatency_frame(float hardwlatency_frame)
{

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].hardwlatency_frame = hardwlatency_frame;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_complatency_frame(float complatency_frame)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].complatency_frame = complatency_frame;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_wfsmextrlatency_frame(float wfsmextrlatency_frame)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].wfsmextrlatency_frame = wfsmextrlatency_frame;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}



/* =============================================================================================== */
/** @name AOloopControl - 8.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS                    */
/* =============================================================================================== */



int_fast8_t AOloopControl_setgain(float gain)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].gain = gain;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setARPFgain(float gain)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].ARPFgain = gain;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setARPFgainAutoMin(float val)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].ARPFgainAutoMin = val;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}

int_fast8_t AOloopControl_setARPFgainAutoMax(float val)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].ARPFgainAutoMax = val;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}




int_fast8_t AOloopControl_setWFSnormfloor(float WFSnormfloor)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].WFSnormfloor = WFSnormfloor;
    printf("SHOWING PARAMETERS ...\n");
    fflush(stdout);
    AOloopControl_perfTest_showparams(LOOPNUMBER);
    printf("DONE ...\n");
    fflush(stdout);

    return 0;
}


int_fast8_t AOloopControl_setmaxlimit(float maxlimit)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].maxlimit = maxlimit;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setmult(float multcoeff)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].mult = multcoeff;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setframesAve(long nbframes)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].framesAve = nbframes;
    AOloopControl_perfTest_showparams(LOOPNUMBER);

    return 0;
}






int_fast8_t AOloopControl_set_modeblock_gain(long loop, long blocknb, float gain, int add)
{
    long IDcontrM0; // local storage
    char name2[200];
    char name3[200];
    long ID;
    long m1;


    printf("AOconf[loop].DMmodesNBblock = %ld\n", AOconf[loop].DMmodesNBblock);
    fflush(stdout);

    /*if(AOconf[loop].CMMODE==0)
    {
        printf("Command has no effect: modeblock gain not compatible with CMMODE = 0\n");
        fflush(stdout);
    }
    else*/
     
    if (AOconf[loop].DMmodesNBblock<2)
    {
        if(sprintf(name2, "aol%ld_contrMc00", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if(sprintf(name3, "aol%ld_contrMcact00_00", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        // for CPU mode
        printf("UPDATING Mc matrix (CPU mode)\n");
        ID = image_ID(name2);
        data.image[aoconfID_contrMc].md[0].write = 1;
        memcpy(data.image[aoconfID_contrMc].array.F, data.image[ID].array.F, sizeof(float)*AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM);
        data.image[aoconfID_contrMc].md[0].cnt0++;
        data.image[aoconfID_contrMc].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoconfID_contrMc].md[0].write = 0;

        // for GPU mode
        printf("UPDATING Mcact matrix (GPU mode)\n");
        ID = image_ID(name3);
        data.image[aoconfID_contrMcact[0]].md[0].write = 1;
        memcpy(data.image[aoconfID_contrMcact[0]].array.F, data.image[ID].array.F, sizeof(float)*AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt);
        data.image[aoconfID_contrMcact[0]].md[0].cnt0++;
        data.image[aoconfID_contrMcact[0]].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoconfID_contrMcact[0]].md[0].write = 0;
    }
    else
    {
		long kk;
	    char name[200];
	    long NBmodes = 0;
        
        for(kk=0; kk<AOconf[loop].DMmodesNBblock; kk++)
            NBmodes += AOconf[loop].NBmodes_block[kk];



        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_gainb = image_ID(name);
        if((blocknb<AOconf[loop].DMmodesNBblock)&&(blocknb>-1))
            data.image[aoconfID_gainb].array.F[blocknb] = gain;


        if(add==1)
        {
			long IDcontrMc0; // local storage
			long IDcontrMcact0; // local storage			
			
			
            IDcontrMc0 = image_ID("contrMc0");
            if(IDcontrMc0==-1)
                IDcontrMc0 = create_3Dimage_ID("contrMc0", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].sizexDM*AOconf[loop].sizeyDM);


            IDcontrMcact0 = image_ID("contrMcact0");
            if(IDcontrMcact0==-1)
                IDcontrMcact0 = create_2Dimage_ID("contrMcact0", AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt);

            //arith_image_zero("contrM0");
            arith_image_zero("contrMc0");
            arith_image_zero("contrMcact0");


            for(kk=0; kk<AOconf[loop].DMmodesNBblock; kk++)
            {
			    double eps=1e-6;
				
				
                if(sprintf(name2, "aol%ld_contrMc%02ld", loop, kk) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(name3, "aol%ld_contrMcact%02ld_00", loop, kk) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                printf("Adding %4ld / %4ld  (%5.3f)   %s   [%ld]\n", kk, AOconf[loop].DMmodesNBblock, data.image[aoconfID_gainb].array.F[kk], name, aoconfID_gainb);

                

                //printf("updating %ld modes  [%ld]\n", data.image[ID].md[0].size[2], aoconfID_gainb);
                //	fflush(stdout); // TEST



                if(data.image[aoconfID_gainb].array.F[kk]>eps)
                {
					long ii;
					
                    ID = image_ID(name2);
# ifdef _OPENMP
                    #pragma omp parallel for
# endif
                    for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
                        data.image[IDcontrMc0].array.F[ii] += data.image[aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];

                    ID = image_ID(name3);
# ifdef _OPENMP
                    #pragma omp parallel for
# endif
                    for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                        data.image[IDcontrMcact0].array.F[ii] += data.image[aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];
                }

            }

            // for CPU mode
            printf("UPDATING Mc matrix (CPU mode)\n");
            data.image[aoconfID_contrMc].md[0].write = 1;
            memcpy(data.image[aoconfID_contrMc].array.F, data.image[IDcontrMc0].array.F, sizeof(float)*AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM);
            data.image[aoconfID_contrMc].md[0].cnt0++;
			data.image[aoconfID_contrMc].md[0].cnt1 = AOconf[loop].LOOPiteration;
			data.image[aoconfID_contrMc].md[0].write = 0;


            // for GPU mode
            printf("UPDATING Mcact matrix (GPU mode)\n");
            data.image[aoconfID_contrMcact[0]].md[0].write = 1;
            memcpy(data.image[aoconfID_contrMcact[0]].array.F, data.image[IDcontrMcact0].array.F, sizeof(float)*AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt);
            data.image[aoconfID_contrMcact[0]].md[0].cnt0++;
            data.image[aoconfID_contrMcact[0]].md[0].cnt1 = AOconf[loop].LOOPiteration;
            data.image[aoconfID_contrMcact[0]].md[0].write = 0;

            initcontrMcact_GPU[0] = 0;
        }
    }

    return(0);
}







int_fast8_t AOloopControl_scanGainBlock(long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain)
{
    long k, kg;
    float bestgain= 0.0;
    float bestval = 10000000.0;



    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_cmd_modes==-1)
    {
		char name[200];
		    
        if(sprintf(name, "aol%ld_DMmode_cmd", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_cmd_modes = read_sharedmem_image(name);
    }


    printf("Block: %ld, NBstep: %ld, gain: %f->%f (%ld septs)\n", NBblock, NBstep, gainStart, gainEnd, NBgain);

    for(kg=0; kg<NBgain; kg++)
    {
	    float gain;
		float val;
		
        for(k=0; k<AOconf[LOOPNUMBER].NBDMmodes; k++)
            data.image[aoconfID_cmd_modes].array.F[k] = 0.0;

        gain = gainStart + 1.0*kg/(NBgain-1)*(gainEnd-gainStart);
        AOloopControl_setgainblock(NBblock, gain);
        AOloopControl_loopstep(LOOPNUMBER, NBstep);
        val = sqrt(AOconf[LOOPNUMBER].RMSmodesCumul/AOconf[LOOPNUMBER].RMSmodesCumulcnt);
        printf("%2ld  %6.4f  %10.8lf\n", kg, gain, val);

        if(val<bestval)
        {
            bestval = val;
            bestgain = gain;
        }
    }
    printf("BEST GAIN = %f\n", bestgain);

    AOloopControl_setgainblock(NBblock, bestgain);

    return(0);
}








/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 10. FOCAL PLANE SPECKLE MODULATION / CONTROL                             */
/* =============================================================================================== */
/* =============================================================================================== */





// optimize LO - uses simulated downhill simplex
int_fast8_t AOloopControl_OptimizePSF_LO(const char *psfstream_name, const char *IDmodes_name, const char *dmstream_name, long delayframe, long NBframes)
{
    long IDmodes;
    long IDdmstream;
    long IDdm;
//    long psfxsize, psfysize;
    long dmxsize, dmysize;
    long NBmodes;
    long mode;
    double ampl;
    double x;
    long ii, jj;

    long IDdmbest;
    long IDpsfarray;

	char imname[200];
	
	
	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    ampl = 0.01; // modulation amplitude

 //   IDpsf = image_ID(psfstream_name);
    IDmodes = image_ID(IDmodes_name);
    IDdmstream = image_ID(dmstream_name);

//    psfxsize = data.image[IDpsf].md[0].size[0];
//    psfysize = data.image[IDpsf].md[0].size[1];

    IDdmbest = create_2Dimage_ID("dmbest", dmxsize, dmysize);
    IDdm = create_2Dimage_ID("dmcurr", dmxsize, dmysize);

    dmxsize = data.image[IDdm].md[0].size[0];
    dmysize = data.image[IDdm].md[0].size[1];

    NBmodes = data.image[IDmodes].md[0].size[2];

    for(ii=0; ii<dmxsize*dmysize; ii++)
        data.image[IDdmbest].array.F[ii] = data.image[IDdm].array.F[ii];


    for(mode=0; mode<NBmodes; mode ++)
    {
        for(x=-ampl; x<1.01*ampl; x += ampl)
        {
            // apply DM pattern
            for(ii=0; ii<dmxsize*dmysize; ii++)
                data.image[IDdm].array.F[ii] = data.image[IDdmbest].array.F[ii]+ampl*data.image[IDmodes].array.F[dmxsize*dmysize*mode+ii];

            data.image[IDdmstream].md[0].write = 1;
            memcpy(data.image[IDdmstream].array.F, data.image[IDdm].array.F, sizeof(float)*dmxsize*dmysize);
            data.image[IDdmstream].md[0].cnt0++;
			data.image[IDdmstream].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
            data.image[IDdmstream].md[0].write = 0;



        }
    }


    return(0);
}


//
// modulate using linear combination of two probes A and B
//
//
// delay is in sec
//
int_fast8_t AOloopControl_DMmodulateAB(const char *IDprobeA_name, const char *IDprobeB_name, const char *IDdmstream_name, const char *IDrespmat_name, const char *IDwfsrefstream_name, double delay, long NBprobes)
{
    long IDprobeA;
    long IDprobeB;
    long dmxsize, dmysize;
    long dmsize;
    long IDdmstream;

    long IDrespmat;
    long IDwfsrefstream;
    long wfsxsize, wfsysize;
    long wfssize;

    long IDdmC;
    long IDwfsrefC;

    float *coeffA;
    float *coeffB;
    int k;
    long act, wfselem;
    
    char imname[200];

    FILE *fp;
    char flogname[200];
    int loopOK;
    long dmframesize, wfsframesize;
    char timestr[200];
    time_t t;
    struct tm *uttime;
    struct timespec *thetime = (struct timespec *)malloc(sizeof(struct timespec));
    long ii;
    int semval;



	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}


    IDprobeA = image_ID(IDprobeA_name);
    dmxsize = data.image[IDprobeA].md[0].size[0];
    dmysize = data.image[IDprobeA].md[0].size[1];
    dmsize = dmxsize*dmysize;

    IDprobeB = image_ID(IDprobeB_name);
    IDdmstream = image_ID(IDdmstream_name);
    IDrespmat = image_ID(IDrespmat_name);
    IDwfsrefstream = image_ID(IDwfsrefstream_name);
    wfsxsize = data.image[IDwfsrefstream].md[0].size[0];
    wfsysize = data.image[IDwfsrefstream].md[0].size[1];
    wfssize = wfsxsize*wfsysize;

    coeffA = (float*) malloc(sizeof(float)*NBprobes);
    coeffB = (float*) malloc(sizeof(float)*NBprobes);

    IDdmC = create_3Dimage_ID("MODdmC", dmxsize, dmysize, NBprobes);
    IDwfsrefC = create_3Dimage_ID("WFSrefC", wfsxsize, wfsysize, NBprobes);

    coeffA[0] = 0.0;
    coeffB[0] = 0.0;
    for(k=1; k<NBprobes; k++)
    {
        coeffA[k] = cos(2.0*M_PI*(k-1)/(NBprobes-1));
        coeffB[k] = sin(2.0*M_PI*(k-1)/(NBprobes-1));
    }


    // prepare MODdmC and WFSrefC
    for(k=0; k<NBprobes; k++)
    {
        for(act=0; act<dmsize; act++)
            data.image[IDdmC].array.F[k*dmsize+act] = coeffA[k]*data.image[IDprobeA].array.F[act] + coeffB[k]*data.image[IDprobeB].array.F[act];

        for(wfselem=0; wfselem<wfssize; wfselem++)
            for(act=0; act<dmsize; act++)
                data.image[IDwfsrefC].array.F[k*wfssize+wfselem] += data.image[IDdmC].array.F[k*dmsize+act]*data.image[IDrespmat].array.F[act*wfssize+wfselem];
    }

    save_fl_fits("MODdmC", "!test_MODdmC.fits");
    save_fl_fits("WFSrefC", "!test_WFSrefC.fits");

    t = time(NULL);
    uttime = gmtime(&t);
    if(sprintf(flogname, "logfpwfs_%04d-%02d-%02d_%02d:%02d:%02d.txt", 1900+uttime->tm_year, 1+uttime->tm_mon, uttime->tm_mday, uttime->tm_hour, uttime->tm_min, uttime->tm_sec) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if((fp=fopen(flogname,"w"))==NULL)
    {
        printf("ERROR: cannot create file \"%s\"\n", flogname);
        exit(0);
    }
    fclose(fp);


    dmframesize = sizeof(float)*dmsize;
    wfsframesize = sizeof(float)*wfssize;

    list_image_ID();



    if (sigaction(SIGINT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGTERM, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGBUS, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    /*   if (sigaction(SIGSEGV, &data.sigact, NULL) == -1) {
           perror("sigaction");
           exit(EXIT_FAILURE);
       }*/
    if (sigaction(SIGABRT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGHUP, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGPIPE, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }



    k = 0;
    loopOK = 1;

    while(loopOK == 1)
    {
        printf("Applying probe # %d   %ld %ld\n", k, IDdmstream, IDwfsrefstream);
        fflush(stdout);

        // apply probe
        char *ptr0;
        ptr0 = (char*) data.image[IDdmC].array.F;
        ptr0 += k*dmframesize;
        data.image[IDdmstream].md[0].write = 1;
        memcpy(data.image[IDdmstream].array.F, (void*) ptr0, dmframesize);
        sem_getvalue(data.image[IDdmstream].semptr[0], &semval);
        if(semval<SEMAPHORE_MAXVAL)
            sem_post(data.image[IDdmstream].semptr[0]);
        data.image[IDdmstream].md[0].cnt0++;
        data.image[IDdmstream].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDdmstream].md[0].write = 0;

        // apply wfsref offset
        ptr0 = (char*) data.image[IDwfsrefC].array.F;
        ptr0 += k*wfsframesize;
        data.image[IDwfsrefstream].md[0].write = 1;
        memcpy(data.image[IDwfsrefstream].array.F, (void*) ptr0, wfsframesize);
        sem_getvalue(data.image[IDwfsrefstream].semptr[0], &semval);
        if(semval<SEMAPHORE_MAXVAL)
            sem_post(data.image[IDwfsrefstream].semptr[0]);
        data.image[IDwfsrefstream].md[0].cnt0++;
        data.image[IDwfsrefstream].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        data.image[IDwfsrefstream].md[0].write = 0;

        // write time in log
        t = time(NULL);
        uttime = gmtime(&t);
        clock_gettime(CLOCK_REALTIME, thetime);

        if(sprintf(timestr, "%02d %02d %02d.%09ld", uttime->tm_hour, uttime->tm_min, uttime->tm_sec, thetime->tv_nsec) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        printf("time = %s\n", timestr);
        if((fp = fopen(flogname, "a"))==NULL)
        {
            printf("ERROR: cannot open file \"%s\"\n", flogname);
            exit(0);
        }
        fprintf(fp, "%s %2d %10f %10f\n", timestr, k, coeffA[k], coeffB[k]);
        fclose(fp);

        usleep((long) (1.0e6*delay));
        k++;
        if(k==NBprobes)
            k = 0;

        if((data.signal_INT == 1)||(data.signal_TERM == 1)||(data.signal_ABRT==1)||(data.signal_BUS==1)||(data.signal_SEGV==1)||(data.signal_HUP==1)||(data.signal_PIPE==1))
            loopOK = 0;
    }

    data.image[IDdmstream].md[0].write = 1;
    for(ii=0; ii<dmsize; ii++)
        data.image[IDdmstream].array.F[ii] = 0.0;
    sem_getvalue(data.image[IDdmstream].semptr[0], &semval);
    if(semval<SEMAPHORE_MAXVAL)
        sem_post(data.image[IDdmstream].semptr[0]);
    data.image[IDdmstream].md[0].cnt0++;
    data.image[IDdmstream].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
    data.image[IDdmstream].md[0].write = 0;


    data.image[IDwfsrefstream].md[0].write = 1;
    for(ii=0; ii<wfssize; ii++)
        data.image[IDwfsrefstream].array.F[ii] = 0.0;
    sem_getvalue(data.image[IDwfsrefstream].semptr[0], &semval);
    if(semval<SEMAPHORE_MAXVAL)
        sem_post(data.image[IDwfsrefstream].semptr[0]);
    data.image[IDwfsrefstream].md[0].cnt0++;
    data.image[IDwfsrefstream].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
    data.image[IDwfsrefstream].md[0].write = 0;



    free(coeffA);
    free(coeffB);


    return(0);
}






/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 11. PROCESS LOG FILES                                                    */
/* =============================================================================================== */
/* =============================================================================================== */


int_fast8_t AOloopControl_logprocess_modeval(const char *IDname)
{
    long ID;
    long NBmodes;
    long NBframes;

    long IDout_ave;
    long IDout_rms;

    long m;
    long ID1dtmp;
    FILE *fp;

    long ID1dPSD;
    char fname[200];



    ID = image_ID(IDname);
    NBmodes = data.image[ID].md[0].size[0]*data.image[ID].md[0].size[1];
    NBframes = data.image[ID].md[0].size[2];

    IDout_ave = create_2Dimage_ID("modeval_ol_ave", data.image[ID].md[0].size[0], data.image[ID].md[0].size[1]);
    IDout_rms = create_2Dimage_ID("modeval_ol_rms", data.image[ID].md[0].size[0], data.image[ID].md[0].size[1]);

    ID1dtmp = create_2Dimage_ID("modeval1d", data.image[ID].md[0].size[2], 1);
    ID1dPSD = create_2Dimage_ID("modevalPSD", data.image[ID].md[0].size[2]/2, 1);

    if(system("mkdir -p modePSD") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    fp = fopen("moveval_stats.dat", "w");
    for(m=0; m<NBmodes; m++)
    {
        double ave = 0.0;
        double rms;
        long kk;
		FILE *fpPSD;
		long IDft;


        for(kk=0; kk<NBframes; kk++)
            ave += data.image[ID].array.F[kk*NBmodes+m];
        ave /= NBframes;
        data.image[IDout_ave].array.F[m] = ave;
        rms = 0.0;
        for(kk=0; kk<NBframes; kk++)
        {
			double tmpv;
			
            tmpv = (data.image[ID].array.F[kk*NBmodes+m]-ave);
            rms += tmpv*tmpv;
        }
        rms = sqrt(rms/NBframes);
        data.image[IDout_rms].array.F[m] = rms;


        for(kk=0; kk<NBframes; kk++)
            data.image[ID1dtmp].array.F[kk] = data.image[ID].array.F[kk*NBmodes+m];
        do1drfft("modeval1d", "modeval1d_FT");
        IDft = image_ID("modeval1d_FT");

        if(sprintf(fname, "./modePSD/modevalPSD_%04ld.dat", m) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        fpPSD = fopen(fname, "w");
        for(kk=0; kk<NBframes/2; kk++)
        {
            data.image[ID1dPSD].array.F[kk] = data.image[IDft].array.CF[kk].re*data.image[IDft].array.CF[kk].re + data.image[IDft].array.CF[kk].im*data.image[IDft].array.CF[kk].im;
            fprintf(fpPSD, "%03ld %g\n", kk, data.image[ID1dPSD].array.F[kk]);
        }
        delete_image_ID("modeval1d_FT");
        fclose(fpPSD);


        fprintf(fp, "%4ld  %12.8f  %12.8f\n", m, data.image[IDout_ave].array.F[m], data.image[IDout_rms].array.F[m]);
    }
    fclose(fp);



    return 0;
}

















































































































































































//
// tweak zonal response matrix in accordance to WFS response to modes
//
// INPUT :
//    ZRMimname   : starting response matrix
//    DMimCname   : cube of DM displacements
//    WFSimCname  : cube of WFS signal
//    DMmaskname  : DM pixel mask
//    WFSmaskname : WFS pixel mask
//
// OUTPUT:
//    RMoutname   : output response matrix
//

long AOloopControl_TweakRM(char *ZRMinname, char *DMinCname, char *WFSinCname, char *DMmaskname, char *WFSmaskname, char *RMoutname)
{
    long IDout, IDzrmin, IDdmin, IDwfsin, IDwfsmask, IDdmmask;
    long wfsxsize, wfsysize, wfssize;
    long dmxsize, dmysize, dmsize;
    long NBframes;


    // input response matrix
    IDzrmin = image_ID(ZRMinname);
    wfsxsize = data.image[IDzrmin].md[0].size[0];
    wfsysize = data.image[IDzrmin].md[0].size[1];

    // DM input frames
    IDdmin = image_ID(DMinCname);
    dmxsize = data.image[IDdmin].md[0].size[0];
    dmysize = data.image[IDdmin].md[0].size[1];
    dmsize = dmxsize*dmysize;

    if(dmsize != data.image[IDzrmin].md[0].size[2])
    {
        printf("ERROR: total number of DM actuators (%ld) does not match zsize of RM (%ld)\n", dmsize, (long) data.image[IDzrmin].md[0].size[2]);
        exit(0);
    }

    NBframes = data.image[IDdmin].md[0].size[2];


    // input WFS frames
    IDwfsin = image_ID(WFSinCname);
    if((data.image[IDwfsin].md[0].size[0] != wfsxsize) || (data.image[IDwfsin].md[0].size[1] != wfsysize) || (data.image[IDwfsin].md[0].size[2] != NBframes))
    {
        printf("ERROR: size of WFS mask image \"%s\" (%ld %ld %ld) does not match expected size (%ld %ld %ld)\n", WFSmaskname, (long) data.image[IDwfsin].md[0].size[0], (long) data.image[IDwfsin].md[0].size[1], (long) data.image[IDwfsin].md[0].size[2], wfsxsize, wfsysize, NBframes);
        exit(0);
    }

    // DM mask
    IDdmmask = image_ID(DMmaskname);
    if((data.image[IDdmmask].md[0].size[0] != dmxsize) || (data.image[IDdmmask].md[0].size[1] != dmysize))
    {
        printf("ERROR: size of DM mask image \"%s\" (%ld %ld) does not match expected size (%ld %ld)\n", DMmaskname, (long) data.image[IDdmmask].md[0].size[0], (long) data.image[IDdmmask].md[0].size[1], dmxsize, dmysize);
        exit(0);
    }



    // ARRANGE DATA IN MATRICES





    return(0);
}












































































































/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 12. OBSOLETE ?                                                           */ 
/* =============================================================================================== */
/* =============================================================================================== */




int_fast8_t AOloopControl_setgainrange(long m0, long m1, float gainval)
{
    long k;
    long kmax;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_DMmode_GAIN == -1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_GAIN", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoconfID_DMmode_GAIN].array.F[k] = gainval;

    return 0;
}




int_fast8_t AOloopControl_setlimitrange(long m0, long m1, float limval)
{
    long k;
    long kmax;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_LIMIT_modes==-1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_LIMIT", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoconfID_LIMIT_modes].array.F[k] = limval;

    return 0;
}




int_fast8_t AOloopControl_setmultfrange(long m0, long m1, float multfval)
{
    long k;
    long kmax;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_MULTF_modes==-1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_MULTF", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_MULTF_modes = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoconfID_MULTF_modes].array.F[k] = multfval;

    return 0;
}




int_fast8_t AOloopControl_setgainblock(long mb, float gainval)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_gainb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_gainb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_gainb = read_sharedmem_image(imname);
    }


    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoconfID_gainb].array.F[mb] = gainval;

    return 0;
}




int_fast8_t AOloopControl_setlimitblock(long mb, float limitval)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_limitb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_limitb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_limitb = read_sharedmem_image(imname);
    }

    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoconfID_limitb].array.F[mb] = limitval;

    return 0;
}




int_fast8_t AOloopControl_setmultfblock(long mb, float multfval)
{
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_multfb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_multfb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_multfb = read_sharedmem_image(imname);
    }

    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoconfID_multfb].array.F[mb] = multfval;

    return 0;
}






























int_fast8_t AOloopControl_AutoTune()
{
    long block;
    long NBstep = 10000;
    char name[200];
    long k;
    float bestgain= 0.0;
    float val;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_cmd_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_cmd", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    // initialize
    for(block=0; block<AOconf[LOOPNUMBER].DMmodesNBblock; block++)
    {
        AOloopControl_setgainblock(block, 0.0);
        AOloopControl_setlimitblock(block, 0.1);
        AOloopControl_setmultfblock(block, 0.8);
    }


    for(block=0; block<AOconf[LOOPNUMBER].DMmodesNBblock; block++)
    {
        float gainStart = 0.0;
        float gainEnd = 1.0;
        int gOK = 1;
        float gain;
        float bestval = 10000000.0;


        // tune block gain
        gain = gainStart;
        while((gOK==1)&&(gain<gainEnd))
        {
            for(k=0; k<AOconf[LOOPNUMBER].NBDMmodes; k++)
                data.image[aoconfID_cmd_modes].array.F[k] = 0.0;

            gain += 0.01;
            gain *= 1.1;

            AOloopControl_setgainblock(block, gain);
            AOloopControl_loopstep(LOOPNUMBER, NBstep);
            val = sqrt(AOconf[LOOPNUMBER].RMSmodesCumul/AOconf[LOOPNUMBER].RMSmodesCumulcnt);
            printf("%6.4f  %10.8lf\n", gain, val);

            if(val<bestval)
            {
                bestval = val;
                bestgain = gain;
            }
            else
                gOK = 0;
        }
        printf("BLOCK %ld  : BEST GAIN = %f\n", block, bestgain);

        AOloopControl_setgainblock(block, bestgain);
    }


    return(0);
}






