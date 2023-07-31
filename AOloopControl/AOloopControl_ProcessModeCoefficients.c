/**
 * @file    AOloopControl_ProcessModeCoefficients.c
 * @brief   AO loop Control compute functions
 *
 * REAL TIME COMPUTING ROUTINES
 *
 *
 *
 * @bug No known bugs.
 *
 */

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

#include <time.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "ImageStreamIO/ImageStreamIO.h"

#include "CommandLineInterface/timeutils.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "info/info.h"

#include "info/info.h"

//extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
//extern AOloopControl_var   aoloopcontrol_var;

// includes mode filtering (limits, multf)
//