/**
 * @file    AOloopControl_computeCalib_makemodes.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    26 Dec 2017
 *
 * 
 * @bug No known bugs.
 * 
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




#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>

#ifdef __MACH__
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


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>


#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"
#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"


#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c


long aoconfID_imWFS2_active[100];

#define MAX_MBLOCK 20

/*** \brief creates AO control modes
 *
 *
 * creates image "modesfreqcpa" which contains CPA value for each mode
 *
 *
 * if Mmask exists, measure xc, yc from it, otherwise use values given to function
 *
 * INPUT (optional): "dmmaskRM" DM actuators directly controlled
 * INPUT (optional): "dmslaved" force these actuators to be slaved to neighbors
 * OUTPUT :          "dmmask" is the union of "dmmaskRM" and "dmslaved"
 *
 * MaskMode = 0  : tapered masking
 * MaskMode = 1  : STRICT masking
 *
 * if BlockNB < 0 : do all blocks
 * if BlockNB >= 0 : only update single block (untested)
 *
 * SVDlim = 0.05 works well
 *
 * OPTIONAL : if file zrespM exists, WFS modes will be computed
 *
 */

