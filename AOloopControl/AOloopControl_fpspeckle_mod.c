/**
 * @file    AOloopControl_fpspeckle_mod.c
 * @brief   AO loop control - FOCAL PLANE SPECKLE MODULATION / CONTROL
 *
 * REAL TIME COMPUTING ROUTINES
 *
 *
 *
 */

#define _GNU_SOURCE

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include <math.h>
#include <string.h>

// defined in AOloopControl.c
//extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
//extern AOloopControl_var aoloopcontrol_var;

