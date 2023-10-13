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

#include "actmap_sample2D.h"
#include "compute_control_modes.h"
#include "compute_masksWFSDM.h"
#include "compute_straight_CM.h"
#include "generateRMWFS.h"
#include "computeHadamard.h"
#include "maskextrapolate.h"

#include "RM2zonal.h"



INIT_MODULE_LIB(AOloopControl_computeCalib)

/** @brief CLI function for AOloopControl_Hadamard_decodeRM */
errno_t AOloopControl_computeCalib_Hadamard_decodeRM_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 4) + CLI_checkarg(3, 4) +
            CLI_checkarg(4, 3) ==
            0)
    {
        AOloopControl_computeCalib_Hadamard_decodeRM(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

static errno_t init_module_CLI()
{
    RegisterCLIcommand(
        "aolHaddec",
        __FILE__,
        AOloopControl_computeCalib_Hadamard_decodeRM_cli,
        "decode Hadamard matrix",
        "<input RM> <Hadamard matrix> <DMpix index frame> <output RM>",
        "aolHaddec imRMh Hmat pixiind imRM",
        "long AOloopControl_computeCalib_Hadamard_decodeRM(char *inname, char "
        "*Hmatname, char "
        "*indexname, char *outname)");

    CLIADDCMD_cacao_computeCalib__compute_control_modes();

    CLIADDCMD_AOloopControl_computeCalib__compsCM();

    CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM();

    CLIADDCMD_AOloopControl_computeCalib__generateRMWFS();

    CLIADDCMD_AOloopControl_computeCalib__mkHadamard();

    CLIADDCMD_AOloopControl_computeCalib__maskextrapolate();

    CLIADDCMD_AOloopControl_computeCalib__RM2zonal();

    CLIADDCMD_AOloopControl_computeCalib__sample2D();


    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}
