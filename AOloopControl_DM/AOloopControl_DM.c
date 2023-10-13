/**
 * @file    AOloopControl_DM.c
 * @brief   DM control
 *
 * To be used for AOloopControl module
 *
 *
 *
 */


// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaodm"

// Module short description
#define MODULE_DESCRIPTION "AO loop Control DM operation"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"



#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl_DM/AOloopControl_DM.h"



#include "AOloopControl_DM_comb.h"

#include "DMturbulence.h"
#include "mk3Ddmgrid.h"
#include "pokerndmodes.h"




// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl_DM)





static errno_t init_module_CLI()
{

    CLIADDCMD_AOloopControl_DM__comb();

    CLIADDCMD_AOloopControl_DM__mk3Ddmgrid();

    CLIADDCMD_AOloopControl_DM__atmturbulence();

    CLIADDCMD_AOloopControl_DM__pokerndmodes();

    return RETURN_SUCCESS;
}
