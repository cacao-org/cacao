/**
 * @file    AOloopControl_compTools.c
 * @brief   Adaptive Optics Control loop engine misc computation tools
 *
 * AO engine uses stream data structure
 *
 *
 */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaoct"

// Module short description
#define MODULE_DESCRIPTION "AO loop control - computation tools"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"

#define _GNU_SOURCE




#include "CommandLineInterface/CLIcore.h"


#include "AOloopControl_compTools/AOloopControl_compTools.h"






INIT_MODULE_LIB(AOloopControl_compTools)



static errno_t init_module_CLI()
{



    return RETURN_SUCCESS;
}



