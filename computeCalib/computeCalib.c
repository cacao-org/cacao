/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaocc"

// Module short description
#define MODULE_DESCRIPTION "AO loop control compute calibration"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"

#define _GNU_SOURCE



#include "CommandLineInterface/CLIcore.h"


#include "compute_control_modes.h"
#include "compute_masksWFSDM.h"
#include "compute_straight_CM.h"
#include "generateRMWFS.h"
#include "computeHadamard.h"
#include "maskextrapolate.h"



INIT_MODULE_LIB(AOloopControl_computeCalib)



static errno_t init_module_CLI()
{

    CLIADDCMD_cacao_computeCalib__compute_control_modes();

    CLIADDCMD_AOloopControl_computeCalib__compsCM();

    CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM();

    CLIADDCMD_AOloopControl_computeCalib__generateRMWFS();

    CLIADDCMD_AOloopControl_computeCalib__mkHadamard();

    CLIADDCMD_AOloopControl_computeCalib__maskextrapolate();


    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}
