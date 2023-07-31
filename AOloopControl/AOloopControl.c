/**
 * @file    AOloopControl.c
 * @brief   Adaptive Optics Control loop engine
 *
 * AO engine uses stream data structure
 *
 * # Files
 *
 * ## Main files
 *
 *
 *
 * @see
 * http://oguyon.github.io/AdaptiveOpticsControl/src/AOloopControl/doc/AOloopControl.html
 *
 * @defgroup AOloopControl_streams Image streams
 * @defgroup AOloopControl_AOLOOPCONTROL_CONF AOloopControl main data structure
 *
 */

/* ================================================================== */
/* ================================================================== */
/*            MODULE INFO                                             */
/* ================================================================== */
/* ================================================================== */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacao"

// Module short description
#define MODULE_DESCRIPTION "AO loop control"

#define _GNU_SOURCE




#include <time.h>

#include "CommandLineInterface/CLIcore.h"


#include "modalfilter.h"
#include "modalCTRL_stats.h"
#include "modalstatsTUI.h"





#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)



INIT_MODULE_LIB(AOloopControl)






static errno_t init_module_CLI()
{


    CLIADDCMD_AOloopControl__modalfilter();
    CLIADDCMD_AOloopControl__modalCTRL_stats();



    return RETURN_SUCCESS;
}


