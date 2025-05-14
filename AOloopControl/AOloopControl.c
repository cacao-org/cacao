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




#include "CommandLineInterface/CLIcore.h"


#include "modalfilter.h"
#include "modalfilter_test.h"

#include "modalCTRL_stats.h"
#include "modalstatsTUI.h"

#include "zonalfilter.h"


INIT_MODULE_LIB(AOloopControl)






static errno_t init_module_CLI()
{
    CLIADDCMD_AOloopControl__modalfilter();
    CLIADDCMD_AOloopControl__modalfilter_test();
    CLIADDCMD_AOloopControl__modalCTRL_stats();
    CLIADDCMD_AOloopControl__modalstatsTUI();

    CLIADDCMD_AOloopControl__zonalfilter();

    return RETURN_SUCCESS;
}
