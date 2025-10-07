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
#define MODULE_SHORTNAME_DEFAULT "cacaopyrwfs"

// Module short description
#define MODULE_DESCRIPTION "pyramid WFS tools"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"

#define _GNU_SOURCE



#include "CommandLineInterface/CLIcore.h"

#include "pyWFSgridmatch.h"




INIT_MODULE_LIB(pyramidWFStools)



static errno_t init_module_CLI()
{

    CLIADDCMD_cacao_pyramidWFStools__pyWFSgridmatch();

    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}
