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
//#include <mach/mach_time.h>
//#define CLOCK_REALTIME 0
//#define CLOCK_MONOTONIC 0
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
#include "CommandLineInterface/CLIcore.h"
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



// static int MATRIX_COMPUTATION_MODE = 0;
// 0: compute sequentially modes and DM commands
// 1: use combined control matrix





/* =============================================================================================== */
/*                    aoconfID are global variables for convenience                                */
/*  aoconfID can be used in other modules as well (with extern)                                    */
/* =============================================================================================== */

/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

#define NB_AOloopcontrol 10 // max number of loops
long LOOPNUMBER = 0; // current loop index


static int AOlooploadconf_init = 0;

#define AOconfname "/tmp/AOconf.shm"
AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
//#define AOloopcontrol_varname "/tmp/aoloopcontrol_var.shm" ??
AOloopControl_var aoloopcontrol_var;

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
//	printf(" ...... Loading module %s\n", __FILE__);
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
    strcpy(data.module[data.NBmodule].info, "cacao   - AO loop control");
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
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
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
    aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);


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
     
    aoloopcontrol_var.aoconfID_dmdisp = read_sharedmem_image(AOconf[loop].dmdispname);
    if(aoloopcontrol_var.aoconfID_dmdisp==-1)
        fprintf(fplog, "ERROR : cannot read shared memory stream %s\n", AOconf[loop].dmdispname);
    else
        fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].dmdispname, aoloopcontrol_var.aoconfID_dmdisp);

 
    aoloopcontrol_var.aoconfID_wfsim = read_sharedmem_image(AOconf[loop].WFSname);
    if(aoloopcontrol_var.aoconfID_wfsim == -1)
        fprintf(fplog, "ERROR : cannot read shared memory stream %s\n", AOconf[loop].WFSname);
    else
        fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].WFSname, aoloopcontrol_var.aoconfID_wfsim);

    AOconf[loop].sizexWFS = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].size[0];
    AOconf[loop].sizeyWFS = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].size[1];
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
    aoloopcontrol_var.aoconfID_wfsdark = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);



    if(sprintf(name, "aol%ld_imWFS0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoloopcontrol_var.aoconfID_imWFS0 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(sprintf(name, "aol%ld_imWFS0tot", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoloopcontrol_var.aoconfID_imWFS0tot = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", 1, 1, 0.0);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(sprintf(name, "aol%ld_imWFS1", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoloopcontrol_var.aoconfID_imWFS1 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);

    if(sprintf(name, "aol%ld_imWFS2", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    aoloopcontrol_var.aoconfID_imWFS2 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);




    initwfsref = AOconf[loop].init_wfsref0;

    if(sprintf(name, "aol%ld_wfsref0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(fname, "./conf/shmim_wfsref0.fits") < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoloopcontrol_var.aoconfID_wfsref0 = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);
    AOconf[loop].init_wfsref0 = 1;

    if(sprintf(name, "aol%ld_wfsref", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(fname, "./conf/shmim_wfsref.fits") < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoloopcontrol_var.aoconfID_wfsref = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 0.0);

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

    aoloopcontrol_var.aoconfID_dmC = image_ID(AOconf[loop].dmCname);
    if(aoloopcontrol_var.aoconfID_dmC==-1)
    {
        printf("connect to %s\n", AOconf[loop].dmCname);
        aoloopcontrol_var.aoconfID_dmC = read_sharedmem_image(AOconf[loop].dmCname);
        if(aoloopcontrol_var.aoconfID_dmC==-1)
        {
            printf("ERROR: cannot connect to shared memory %s\n", AOconf[loop].dmCname);
            exit(0);
        }
    }
    AOconf[loop].sizexDM = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].size[0];
    AOconf[loop].sizeyDM = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].size[1];
    AOconf[loop].sizeDM = AOconf[loop].sizexDM*AOconf[loop].sizeyDM;

    fprintf(fplog, "Connected to DM %s, size = %ld x %ld\n", AOconf[loop].dmCname, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);



	/**
	 * - AOconf[loop].dmRMname : DM response matrix channel
	 * 
	 */
    aoloopcontrol_var.aoconfID_dmRM = image_ID(AOconf[loop].dmRMname);
    if(aoloopcontrol_var.aoconfID_dmRM==-1)
    {
        printf("connect to %s\n", AOconf[loop].dmRMname);
        aoloopcontrol_var.aoconfID_dmRM = read_sharedmem_image(AOconf[loop].dmRMname);
        if(aoloopcontrol_var.aoconfID_dmRM==-1)
        {
            printf("ERROR: cannot connect to shared memory %s\n", AOconf[loop].dmRMname);
            exit(0);
        }
    }
    fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].dmRMname, aoloopcontrol_var.aoconfID_dmRM);



	/// Connect to DM modes shared mem
	///  continue if not successful
	///
	aoloopcontrol_var.aoconfID_DMmodes = image_ID(AOconf[loop].DMmodesname);
	if(aoloopcontrol_var.aoconfID_DMmodes==-1)
    {
        printf("connect to %s\n", AOconf[loop].DMmodesname);
        aoloopcontrol_var.aoconfID_DMmodes = read_sharedmem_image(AOconf[loop].DMmodesname);
        if(aoloopcontrol_var.aoconfID_DMmodes==-1)
        {
            printf("WARNING: cannot connect to shared memory %s\n", AOconf[loop].DMmodesname);
//			exit(0);
        }
    }
	if(aoloopcontrol_var.aoconfID_DMmodes!=-1)
	{
		fprintf(fplog, "stream %s loaded as ID = %ld\n", AOconf[loop].DMmodesname, aoloopcontrol_var.aoconfID_DMmodes);
		AOconf[loop].NBDMmodes = data.image[aoloopcontrol_var.aoconfID_DMmodes].md[0].size[2];
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
		
        aoloopcontrol_var.aoconfID_DMmodes = image_ID(AOconf[loop].DMmodesname); 

        if(aoloopcontrol_var.aoconfID_DMmodes == -1) // If not, check file
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
                    aoloopcontrol_var.aoconfID_DMmodes = ID2tmp;
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
                aoloopcontrol_var.aoconfID_DMmodes = create_image_ID(AOconf[loop].DMmodesname, 3, sizearray, _DATATYPE_FLOAT, 1, 0);
            }

            // put modes into shared memory

            switch (data.image[ID1tmp].md[0].atype) {
            case _DATATYPE_FLOAT :
                memcpy(data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F, data.image[ID1tmp].array.F, sizeof(float)*AOconf[loop].sizexDM*AOconf[loop].sizeyDM*AOconf[loop].NBDMmodes);
                break;
            case _DATATYPE_DOUBLE :
                for(ii=0; ii<AOconf[loop].sizexDM*AOconf[loop].sizeyDM*AOconf[loop].NBDMmodes; ii++)
                    data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F[ii] = data.image[ID1tmp].array.D[ii];
                break;
            default :
                printf("ERROR: TYPE NOT RECOGNIZED FOR MODES\n");
                exit(0);
                break;
            }

            delete_image_ID("tmp3Dim");
        }

        fprintf(fplog, "stream %s loaded as ID = %ld, size %ld %ld %ld\n", AOconf[loop].DMmodesname, aoloopcontrol_var.aoconfID_DMmodes, AOconf[loop].sizexDM, AOconf[loop].sizeyDM, AOconf[loop].NBDMmodes);
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
        aoloopcontrol_var.aoconfID_cmd_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);


        if(sprintf(name, "aol%ld_DMmode_meas", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_meas_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_AVE", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_AVE_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_RMS", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_RMS_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 0.0);

        if(sprintf(name, "aol%ld_DMmode_GAIN", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_DMmode_GAIN = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);

        if(sprintf(name, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_LIMIT_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);

        if(sprintf(name, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_MULTF_modes = AOloopControl_IOtools_2Dloadcreate_shmim(name, "", AOconf[loop].NBDMmodes, 1, 1.0);


        if(sprintf(name, "aol%ld_wfsmask", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        sprintf(fname, "conf/%s.fits", name);
        aoloopcontrol_var.aoconfID_wfsmask = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, 1.0);
        AOconf[loop].activeWFScnt = 0;
        for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS; ii++)
            if(data.image[aoloopcontrol_var.aoconfID_wfsmask].array.F[ii]>0.5)
                AOconf[loop].activeWFScnt++;

        if(sprintf(name, "aol%ld_dmmask", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/%s.fits", name) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_dmmask = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].sizexDM, AOconf[loop].sizeyDM, 1.0);
        AOconf[loop].activeDMcnt = 0;
        for(ii=0; ii<AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
            if(data.image[aoloopcontrol_var.aoconfID_dmmask].array.F[ii]>0.5)
                AOconf[loop].activeDMcnt++;

        printf(" AOconf[loop].activeWFScnt = %ld\n", AOconf[loop].activeWFScnt );
        printf(" AOconf[loop].activeDMcnt = %ld\n", AOconf[loop].activeDMcnt );


        AOconf[loop].init_RM = 0;
        if(sprintf(fname, "conf/shmim_respM.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_respM = AOloopControl_IOtools_3Dloadcreate_shmim(AOconf[loop].respMname, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBDMmodes, 0.0);
        AOconf[loop].init_RM = 1;


        AOconf[loop].init_CM = 0;
        if(sprintf(fname, "conf/shmim_contrM.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_contrM = AOloopControl_IOtools_3Dloadcreate_shmim(AOconf[loop].contrMname, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].NBDMmodes, 0.0);
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
        aoloopcontrol_var.aoconfID_contrMc = AOloopControl_IOtools_3Dloadcreate_shmim(name, fname, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].sizeDM, 0.0);

        if(sprintf(name, "aol%ld_contrMcact", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_contrMcact_00.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_contrMcact[0] = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt, 0.0);



        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_gainb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_gainb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);

		if(sprintf(name, "aol%ld_modeARPFgainAuto", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_modeARPFgainAuto.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_modeARPFgainAuto = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].NBDMmodes, 1, 1.0);


        if(sprintf(name, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_multfb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_multfb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);

        if(sprintf(name, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        if(sprintf(fname, "conf/shmim_limitb.fits") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_limitb = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].DMmodesNBblock, 1, 0.0);


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
            printf("TEST - BLOCK %3ld gain = %f\n", kk, data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]);
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
                    data.image[aoloopcontrol_var.aoconfID_contrMc].array.F[ii] = 0.0;
            for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
                data.image[aoloopcontrol_var.aoconfID_contrMc].array.F[ii] += data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];


            if(sprintf(name, "aol%ld_contrMcact%02ld_00", loop, kk) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            if(sprintf(fname, "conf/%s.fits", name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            //   sprintf(fname, "conf/shmim_contrMcact%02ld_00", kk);
            printf("====== LOADING %s to %s  size %ld %ld\n", fname, name,  AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt);
            ID = AOloopControl_IOtools_2Dloadcreate_shmim(name, fname, AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt, 0.0);

            if(kk==0)
                for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                    data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F[ii] = 0.0;

            for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F[ii] += data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];

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

        if((aoloopcontrol_var.aoconfIDlogdata = image_ID(cntname))==-1)
        {
            uint32_t *sizearray;
            sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
            sizearray[0] = 1;
            sizearray[1] = 1;
            aoloopcontrol_var.aoconfIDlogdata = create_image_ID(cntname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
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
                printf("  WFS:  %s  [%ld]  %ld x %ld\n", AOconf[loop].WFSname, aoloopcontrol_var.aoconfID_wfsim, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS);
                printf("   DM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmCname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
                printf("DM RM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmRMname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
            }
    }

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
    {

        printf("INITIALIZING GPUset ARRAYS\n");
        fflush(stdout);

        aoloopcontrol_var.GPUset0 = (int*) malloc(sizeof(int)*aoloopcontrol_var.GPUcntMax);

        uint_fast16_t k;

        for(k=0; k<aoloopcontrol_var.GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_aoloopcontrol_var.GPUset0dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                aoloopcontrol_var.GPUset0[k] = tmpi;
            }
            else
                aoloopcontrol_var.GPUset0[k] = k;
        }


        aoloopcontrol_var.GPUset1 = (int*) malloc(sizeof(int)*aoloopcontrol_var.GPUcntMax);
        for(k=0; k<aoloopcontrol_var.GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_aoloopcontrol_var.GPUset1dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                aoloopcontrol_var.GPUset1[k] = tmpi;
            }
            else
                aoloopcontrol_var.GPUset1[k] = k;
        }
    }

    aoloopcontrol_var.AOloopcontrol_meminit = 1;


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
// cf AOloopControl_wfs_dm.c 



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 8.   LOOP CONTROL INTERFACE                                              */
/* =============================================================================================== */
/* =============================================================================================== */
// cf AOloopControl_loop_ctr.c


/* =============================================================================================== */
/** @name AOloopControl - 8.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET  */
/* =============================================================================================== */
//cf AOloopControl_loop_onoff.c


/* =============================================================================================== */
/** @name AOloopControl - 8.2. LOOP CONTROL INTERFACE - DATA LOGGING                               */
/* =============================================================================================== */



/* =============================================================================================== */
/** @name AOloopControl - 8.3. LOOP CONTROL INTERFACE - PRIMARY AND FILTERED DM WRITE                           */
/* =============================================================================================== */
// cf AOloopControl_dmwrite.c


/* =============================================================================================== */
/** @name AOloopControl - 8.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING                     */
/* =============================================================================================== */
// cf AOloopControl_autotune.c



/* =============================================================================================== */
/** @name AOloopControl - 8.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF                   */
/* =============================================================================================== */
// cf AOloopControl_arpf_onoff.c



/* =============================================================================================== */
/** @name AOloopControl - 8.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS                          */
/* =============================================================================================== */
// cf AOloopControl_time_param.c



/* =============================================================================================== */
/** @name AOloopControl - 8.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS                    */
/* =============================================================================================== */
// cf AOloopControl_loop_param.c



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 10. FOCAL PLANE SPECKLE MODULATION / CONTROL                             */
/* =============================================================================================== */
/* =============================================================================================== */
// cf AOloopControl_fpspeckle_mod.c



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 11. PROCESS LOG FILES                                                    */
/* =============================================================================================== */
/* =============================================================================================== */
// cf AOloopControl_process_files.c


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 12. OBSOLETE ?                                                           */ 
/* =============================================================================================== */
/* =============================================================================================== */




int_fast8_t AOloopControl_setgainrange(long m0, long m1, float gainval)
{
    long k;
    long kmax;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_DMmode_GAIN == -1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_GAIN", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k] = gainval;

    return 0;
}




int_fast8_t AOloopControl_setlimitrange(long m0, long m1, float limval)
{
    long k;
    long kmax;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_LIMIT_modes==-1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_LIMIT", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k] = limval;

    return 0;
}




int_fast8_t AOloopControl_setmultfrange(long m0, long m1, float multfval)
{
    long k;
    long kmax;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_MULTF_modes==-1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_MULTF", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(name);
    }

    kmax = m1+1;
    if(kmax>AOconf[LOOPNUMBER].NBDMmodes)
        kmax = AOconf[LOOPNUMBER].NBDMmodes-1;

    for(k=m0; k<kmax; k++)
        data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[k] = multfval;

    return 0;
}




int_fast8_t AOloopControl_setgainblock(long mb, float gainval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_gainb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_gainb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_gainb = read_sharedmem_image(imname);
    }


    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoloopcontrol_var.aoconfID_gainb].array.F[mb] = gainval;

    return 0;
}




int_fast8_t AOloopControl_setlimitblock(long mb, float limitval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_limitb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_limitb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_limitb = read_sharedmem_image(imname);
    }

    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoloopcontrol_var.aoconfID_limitb].array.F[mb] = limitval;

    return 0;
}




int_fast8_t AOloopControl_setmultfblock(long mb, float multfval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_multfb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_multfb", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_multfb = read_sharedmem_image(imname);
    }

    if(mb<AOconf[LOOPNUMBER].DMmodesNBblock)
        data.image[aoloopcontrol_var.aoconfID_multfb].array.F[mb] = multfval;

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

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_cmd_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_cmd", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
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
                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = 0.0;

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






