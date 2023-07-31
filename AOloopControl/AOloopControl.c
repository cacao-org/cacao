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


#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/syscall.h> // needed for tid = syscall(SYS_gettid);
#include <sys/types.h>

#include <gsl/gsl_math.h>

#include <time.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "ZernikePolyn/ZernikePolyn.h"
#include "fft/fft.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"

#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "computeCalib/computeCalib.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

#include "modalfilter.h"
#include "modalCTRL_stats.h"
#include "modalstatsTUI.h"

#include "linalgebra/linalgebra.h"


#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                      DEFINES, MACROS */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// data passed to each thread
// typedef struct
//{
//   long nelem;
//    float *arrayptr;
//    float *result; // where to white status
//} THDATA_IMTOTAL;

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                  GLOBAL DATA DECLARATION */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/*                				    LOGGING ACCESS TO FUNCTIONS
 */
/* ===============================================================================================
 */

// uncomment at compilation time to enable logging of function entry/exit
//#define AOLOOPCONTROL_LOGFUNC
// static int AOLOOPCONTROL_logfunc_level = 0;
// static int AOLOOPCONTROL_logfunc_level_max = 2; // log all levels equal or
// below this number static char AOLOOPCONTROL_logfunc_fname[] =
// "AOloopControl.fcall.log"; static char flogcomment[200];

// GPU MultMat indexes
//
// 0: main loop CM multiplication
//
// 1: set DM modes:
//         int set_DM_modes(long loop)
//
// 2: compute modes loop
//         int AOloopControl_CompModes_loop(char *ID_CM_name, char
//         *ID_WFSref_name, char *ID_WFSim_name, char *ID_WFSimtot_name, char
//         *ID_coeff_name)
//
// 3: coefficients to DM shape
//         int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int
//         GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int
//         semTrigg, char *out_name, int GPUindex, long loop, int offloadMode)
//
// 4: Predictive control (in modules linARfilterPred)
//
// 5: coefficients to DM shape, PF
// 			int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int
// GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg,
// char *out_name, int GPUindex, long loop, int offloadMode)
//
// 6: coefficients to DM shape, OL
//			int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int
// GPUMATMULTCONFindex, char *modecoeffs_name, char *DMmodes_name, int semTrigg,
// char *out_name, int GPUindex, long loop, int offloadMode)
//

// TIMING
// static struct timespec tnow;
// static struct timespec tdiff;
// static double tdiffv;

// static int MATRIX_COMPUTATION_MODE = 0;
// 0: compute sequentially modes and DM commands
// 1: use combined control matrix

/* ===============================================================================================
 */
/*                    aoconfID are global variables for convenience */
/*  aoconfID can be used in other modules as well (with extern) */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/*                                     MAIN DATA STRUCTURES */
/* ===============================================================================================
 */

static int DISABLE_CLI_AOloopControl = 0; // set to 1 to disable CLI

#define NB_AOloopcontrol 10 // max number of loops
long LOOPNUMBER = 0;        // current loop index

// static int AOlooploadconf_init = 0;

//AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

//AOloopControl_var aoloopcontrol_var;

/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl)





/** @brief CLI function for AOloopControl_WFSzpupdate_loop */
errno_t AOloopControl_WFSzpupdate_loop_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_IMG) ==
            0)
    {
        AOloopControl_WFSzpupdate_loop(data.cmdargtoken[1].val.string,
                                       data.cmdargtoken[2].val.string,
                                       data.cmdargtoken[3].val.string);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_WFSzeropoint_sum_update_loop */
errno_t AOloopControl_WFSzeropoint_sum_update_loop_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_STR_NOT_IMG) +
            CLI_checkarg(2, CLIARG_INT64) + CLI_checkarg(3, CLIARG_IMG) +
            CLI_checkarg(4, CLIARG_IMG) ==
            0)
    {
        AOloopControl_WFSzeropoint_sum_update_loop(
            LOOPNUMBER,
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_CompModes_loop */
errno_t AOloopControl_CompModes_loop_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_IMG) +
            CLI_checkarg(4, CLIARG_IMG) + CLI_checkarg(5, CLIARG_STR_NOT_IMG) ==
            0)
    {
        AOloopControl_CompModes_loop(data.cmdargtoken[1].val.string,
                                     data.cmdargtoken[2].val.string,
                                     data.cmdargtoken[3].val.string,
                                     data.cmdargtoken[4].val.string,
                                     data.cmdargtoken[5].val.string);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_GPUmodecoeffs2dm_filt */
errno_t AOloopControl_GPUmodecoeffs2dm_filt_loop_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_IMG) +
            CLI_checkarg(4, CLIARG_INT64) + CLI_checkarg(5, CLIARG_IMG) +
            CLI_checkarg(6, CLIARG_INT64) + CLI_checkarg(7, CLIARG_INT64) +
            CLI_checkarg(8, CLIARG_INT64) ==
            0)
    {
        AOloopControl_GPUmodecoeffs2dm_filt_loop(data.cmdargtoken[1].val.numl,
                data.cmdargtoken[2].val.string,
                data.cmdargtoken[3].val.string,
                data.cmdargtoken[4].val.numl,
                data.cmdargtoken[5].val.string,
                data.cmdargtoken[6].val.numl,
                data.cmdargtoken[7].val.numl,
                data.cmdargtoken[8].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_computeWFSresidualimage */
errno_t AOloopControl_computeWFSresidualimage_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_IMG) ==
            0)
    {
        AOloopControl_computeWFSresidualimage(data.cmdargtoken[1].val.numl,
                                              data.cmdargtoken[2].val.string);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_ProcessModeCoefficients */
errno_t AOloopControl_ProcessModeCoefficients_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_ProcessModeCoefficients(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_dm2dm_offload */
errno_t AOloopControl_dm2dm_offload_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_FLOAT64) +
            CLI_checkarg(4, CLIARG_FLOAT64) + CLI_checkarg(5, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_dm2dm_offload(data.cmdargtoken[1].val.string,
                                    data.cmdargtoken[2].val.string,
                                    data.cmdargtoken[3].val.numf,
                                    data.cmdargtoken[4].val.numf,
                                    data.cmdargtoken[5].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_sig2Modecoeff */
errno_t AOloopControl_sig2Modecoeff_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_IMG) +
            CLI_checkarg(4, CLIARG_STR_NOT_IMG) ==
            0)
    {
        AOloopControl_sig2Modecoeff(data.cmdargtoken[1].val.string,
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



/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 10. FOCAL PLANE SPECKLE MODULATION / CONTROL */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_DMmodulateAB */
errno_t AOloopControl_DMmodulateAB_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_IMG) + CLI_checkarg(3, CLIARG_IMG) +
            CLI_checkarg(4, CLIARG_IMG) + CLI_checkarg(5, CLIARG_IMG) +
            CLI_checkarg(6, CLIARG_FLOAT64) + CLI_checkarg(7, CLIARG_INT64) ==
            0)
    {
        AOloopControl_DMmodulateAB(data.cmdargtoken[1].val.string,
                                   data.cmdargtoken[2].val.string,
                                   data.cmdargtoken[3].val.string,
                                   data.cmdargtoken[4].val.string,
                                   data.cmdargtoken[5].val.string,
                                   data.cmdargtoken[6].val.numf,
                                   data.cmdargtoken[7].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 11. PROCESS LOG FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_logprocess_modeval */
errno_t AOloopControl_logprocess_modeval_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) == 0)
    {
        AOloopControl_logprocess_modeval(data.cmdargtoken[1].val.string);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 6. REAL-TIME LOGGING - AOloopControl_RTstreamLOG.c */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

errno_t AOloopControl_RTstreamLOG_init_cli()
{
    AOloopControl_RTstreamLOG_init(LOOPNUMBER);
    return CLICMD_SUCCESS;
}

errno_t AOloopControl_RTstreamLOG_printstatus_cli()
{
    AOloopControl_RTstreamLOG_printstatus(LOOPNUMBER);
    return CLICMD_SUCCESS;
}

errno_t AOloopControl_RTstreamLOG_GUI_cli()
{
    AOloopControl_RTstreamLOG_GUI(LOOPNUMBER);
    return CLICMD_SUCCESS;
}

errno_t AOloopControl_RTstreamLOG_saveloop_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_STR) == 0)
    {
        AOloopControl_RTstreamLOG_saveloop(LOOPNUMBER,
                                           data.cmdargtoken[1].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_RTstreamLOG_set_saveON_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_RTstreamLOG_set_saveON(LOOPNUMBER,
                                             data.cmdargtoken[1].val.numl);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_RTstreamLOG_set_saveOFF_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_RTstreamLOG_set_saveOFF(LOOPNUMBER,
                                              data.cmdargtoken[1].val.numl);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_RTstreamLOG_set_ON_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_RTstreamLOG_set_ON(LOOPNUMBER,
                                         data.cmdargtoken[1].val.numl);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_RTstreamLOG_set_OFF_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_RTstreamLOG_set_OFF(LOOPNUMBER,
                                          data.cmdargtoken[1].val.numl);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

// OBSOLETE ??
/*
errno_t AOloopControl_setparam_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_STR_NOT_IMG) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setparam(LOOPNUMBER,
                               data.cmdargtoken[1].val.string,
                               data.cmdargtoken[2].val.numf);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}
*/


errno_t AOloopControl_modalstatsTUI_cli()
{
    if(CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_modalstatsTUI(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}



/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                    FUNCTIONS SOURCE CODE */
/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl functions */

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 1. INITIALIZATION, configurations */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

int AOloopControl_bogusfunc()
{
    printf("This function does NOTHING !\n");
    return 0;
}

// CODING STANDARD NOTE: minimal required documentation for doxygen
/**
 *  ## Purpose
 *
 * Initialization of the AO loop Control with all of the CLI command line
 * interface commands
 *
 *
 * ## Arguments
 *
 * @param[in]
 * mode		INT
 * 			mode sets up what function does
 * -		does nothing
 * -		also does nothing
 *
 */
// CODING STANDARD NOTE: function name start with module name

static errno_t init_module_CLI()
{
    FILE *fp;

#ifdef AOLOOPCONTROL_LOGFUNC
    CORE_logFunctionCall(AOLOOPCONTROL_logfunc_level,
                         AOLOOPCONTROL_logfunc_level_max,
                         0,
                         __FILE__,
                         __FUNCTION__,
                         __LINE__,
                         "");
#endif

    if((fp = fopen("LOOPNUMBER", "r")) != NULL)
    {
        if(fscanf(fp, "%8ld", &LOOPNUMBER) != 1)
        {
            PRINT_ERROR("Cannot read LOOPNUMBER");
        }

        printf("LOOP NUMBER = %ld\n", LOOPNUMBER);
        fclose(fp);
    }
    else
    {
        LOOPNUMBER = 0;
    }

\




    /* ===============================================================================================
    */
    /* ===============================================================================================
    */
    /* 3.   LOOP CONTROL INTERFACE - AOloopControl_loop_ctr.c */
    /* ===============================================================================================
    */
    /* ===============================================================================================
    */

    /* ===============================================================================================
    */
    /* 3.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF
    * START/STOP/STEP/RESET                  */
    /* ===============================================================================================
    */



    RegisterCLIcommand(
        "aolzpwfscloop",
        __FILE__,
        AOloopControl_WFSzeropoint_sum_update_loop_cli,
        "WFS zero point offset loop: combine multiple input channels",
        "<name prefix> <number of channels> <wfsref0> <wfsref>",
        "aolzpwfscloop wfs2zpoffset 4 wfsref0 wfsref",
        "int AOloopControl_WFSzeropoint_sum_update_loop(long loopnb, char "
        "*ID_WFSzp_name, int NBzp, char "
        "*IDwfsref0_name, char *IDwfsref_name)"); // WFS zero point offset loop:
    // combine multiple input
    // channels

    RegisterCLIcommand(
        "aocmlrun",
        __FILE__,
        AOloopControl_CompModes_loop_cli,
        "run AO compute modes loop",
        "<CM> <wfsref> <WFS image stream> <WFS image total stream> <output "
        "stream>",
        "aocmlrun CM wfsref wfsim wfsimtot aomodeval",
        "int AOloopControl_CompModes_loop(char *ID_CM_name, char "
        "*ID_WFSref_name, char *ID_WFSim_name, "
        "char *ID_WFSimtot, char *ID_coeff_name)"); // run AO compute modes loop

    RegisterCLIcommand(
        "aolmc2dmfilt",
        __FILE__,
        AOloopControl_GPUmodecoeffs2dm_filt_loop_cli,
        "convert mode coefficients to DM map",
        "<GPUMATMULTindex> <mode coeffs> <DMmodes> <sem trigg number> <out> "
        "<GPUindex> <loopnb> "
        "<offloadMode>",
        "aolmc2dmfilt aolmodeval DMmodesC 2 dmmapc 0.2 1 2 1",
        "int AOloopControl_GPUmodecoeffs2dm_filt_loop(const int "
        "GPUMATMULTCONFindex, char "
        "*modecoeffs_name, char *DMmodes_name, int semTrigg, char *out_name, "
        "int GPUindex, long loop, "
        "long offloadMode)"); // convert mode coefficients to DM map

    RegisterCLIcommand(
        "aolsig2mcoeff",
        __FILE__,
        AOloopControl_sig2Modecoeff_cli,
        "convert signals to mode coeffs",
        "<signal data cube> <reference> <Modes data cube> <output image>",
        "aolsig2mcoeff wfsdata wfsref wfsmodes outim",
        "long AOloopControl_sig2Modecoeff(char *WFSim_name, char "
        "*IDwfsref_name, char *WFSmodes_name, "
        "char *outname)"); // convert signals to mode coeffs







    /* ===============================================================================================
    */
    /* ===============================================================================================
    */
    /** @name AOloopControl - 11. PROCESS LOG FILES */
    /* ===============================================================================================
    */
    /* ===============================================================================================
    */


    RegisterCLIcommand(
        "aollogprocmodeval",
        __FILE__,
        AOloopControl_logprocess_modeval_cli,
        "process log image modeval",
        "<modeval image>",
        "aollogprocmodeval imc",
        "int AOloopControl_logprocess_modeval(const char *IDname);");

/*    RegisterCLIcommand(
        "aolsetgainr",
        __FILE__,
        AOloopControl_setgainrange_cli,
        "set modal gains from m0 to m1 included",
        "<modemin [long]> <modemax [long]> <gainval>",
        "aolsetgainr 20 30 0.2",
        "int AOloopControl_setgainrange(long m0, long m1, float gainval)");
*/
/*    RegisterCLIcommand(
        "aolsetlimitr",
        __FILE__,
        AOloopControl_setlimitrange_cli,
        "set modal limits",
        "<modemin [long]> <modemax [long]> <limval>",
        "aolsetlimitr 20 30 0.02",
        "int AOloopControl_setlimitrange(long m0, long m1, float gainval)");
*/

/*    RegisterCLIcommand(
        "aolsetmultfr",
        __FILE__,
        AOloopControl_setmultfrange_cli,
        "set modal multf",
        "<modemin [long]> <modemax [long]> <multfval>",
        "aolsetmultfr 10 30 0.98",
        "int AOloopControl_setmultfrange(long m0, long m1, float multfval)");

    RegisterCLIcommand(
        "aolsetgainb",
        __FILE__,
        AOloopControl_setgainblock_cli,
        "set modal gains by block",
        "<block [long]> <gainval>",
        "aolsetgainb 2 0.2",
        "int AOloopControl_setgainblock(long m0, long m1, float gainval)");

    RegisterCLIcommand(
        "aolsetlimitb",
        __FILE__,
        AOloopControl_setlimitblock_cli,
        "set modal limits by block",
        "<block [long]> <limval>",
        "aolsetlimitb 2 0.02",
        "int AOloopControl_setlimitblock(long mb, float limitval)");

    RegisterCLIcommand(
        "aolsetmultfb",
        __FILE__,
        AOloopControl_setmultfblock_cli,
        "set modal multf by block",
        "<block [long]> <multfval>",
        "aolsetmultfb 2 0.98",
        "int AOloopControl_setmultfblock(long mb, float multfval)");

    RegisterCLIcommand(
        "aolscangainb",
        __FILE__,
        AOloopControl_scanGainBlock_cli,
        "scan gain for block",
        "<blockNB> <NBAOsteps> <gainstart> <gainend> <NBgainpts>",
        "aolscangainb",
        "int AOloopControl_scanGainBlock(long NBblock, long NBstep, float "
        "gainStart, float gainEnd, long NBgain)");

    RegisterCLIcommand("aolmkwfsres",
                       __FILE__,
                       AOloopControl_computeWFSresidualimage_cli,
                       "compute WFS residual real time",
                       "<loopnb> <averaging coeff image>",
                       "aolmkwfsres 2 coeffim",
                       "long AOloopControl_computeWFSresidualimage(long loop, "
                       "char *IDalpha_name)");

    RegisterCLIcommand("aolcompolm",
                       __FILE__,
                       AOloopControl_ProcessModeCoefficients_cli,
                       "process mode coefficients, incl open loop mode comp",
                       "<loop #>",
                       "aolcompolm 2",
                       "long AOloopControl_ProcessModeCoefficients(long loop)");

*/
/*    RegisterCLIcommand(
        "aoldm2dmoffload",
        __FILE__,
        AOloopControl_dm2dm_offload_cli,
        "slow offload from dm to dm",
        "<streamin> <streamout> <timestep[sec]> <offloadcoeff> <multcoeff>",
        "aoldm2dmoffload dmin dmout 0.5 -0.01 0.999",
        "long AOloopControl_dm2dm_offload(const char *streamin, const char "
        "*streamout, float twait, "
        "float offcoeff, float multcoeff)");
*/


/*    RegisterCLIcommand(
        "aolset",
        __FILE__,
        AOloopControl_setparam_cli,
        "set parameter",
        "<parameter> <value>",
        "aolset",
        "int AOloopControl_setparam(long loop, const char *key, double value)");
*/
    RegisterCLIcommand(
        "aoldmmodAB",
        __FILE__,
        AOloopControl_DMmodulateAB_cli,
        "module DM with linear combination of probes A and B",
        "<probeA> <probeB> <dmstream> <WFS resp mat> <WFS ref stream> <delay "
        "[sec]> <NB probes>",
        "aoldmmodAB probeA probeB wfsrespmat wfsref 0.1 6",
        "int AOloopControl_DMmodulateAB(const char *IDprobeA_name, const char "
        "*IDprobeB_name, const char "
        "*IDdmstream_name, const char *IDrespmat_name, const char "
        "*IDwfsrefstream_name, double delay, long NBprobes)");

    /* ===============================================================================================
    */
    /* ===============================================================================================
    */
    /* 6. REAL-TIME LOGGING - AOloopControl_RTstreamLOG.c */
    /* ===============================================================================================
    */
    /* ===============================================================================================
    */

    RegisterCLIcommand("aolrtloginit",
                       __FILE__,
                       AOloopControl_RTstreamLOG_init_cli,
                       "Init Real-Time logging",
                       "no arg",
                       "aolrtloginit",
                       "AOloopControl_RTstreamLOG_init(int loop)");

    RegisterCLIcommand("aolrtlogstat",
                       __FILE__,
                       AOloopControl_RTstreamLOG_printstatus_cli,
                       "Print status of Real-Time logging",
                       "no arg",
                       "aolrtlogstat",
                       "AOloopControl_RTstreamLOG_printstatus(int loop)");

    RegisterCLIcommand("aolrtlogGUI",
                       __FILE__,
                       AOloopControl_RTstreamLOG_GUI_cli,
                       "Simple GUI for Real-Time logging",
                       "no arg",
                       "aolrtlogGUI",
                       "AOloopControl_RTstreamLOG_GUI(int loop)");

    RegisterCLIcommand(
        "aolrtlogsavel",
        __FILE__,
        AOloopControl_RTstreamLOG_saveloop_cli,
        "Save files for Real-Time logging",
        "<directory>",
        "aolrtlogsavel",
        "AOloopControl_RTstreamLOG_saveloop(int loop, char *dirname)");

    RegisterCLIcommand(
        "aolrtlogsetsaveON",
        __FILE__,
        AOloopControl_RTstreamLOG_set_saveON_cli,
        "Real-Time logging: set save to ON",
        "<rtlindex>",
        "aolrtlogsetsaveON 2",
        "int AOloopControl_RTstreamLOG_set_saveON(int loop, int rtlindex)");

    RegisterCLIcommand(
        "aolrtlogsetsaveOFF",
        __FILE__,
        AOloopControl_RTstreamLOG_set_saveOFF_cli,
        "Real-Time logging: set save to OFF",
        "<rtlindex>",
        "aolrtlogsetsaveOFF 2",
        "int AOloopControl_RTstreamLOG_set_saveOFF(int loop, int rtlindex)");

    RegisterCLIcommand(
        "aolrtlogsetON",
        __FILE__,
        AOloopControl_RTstreamLOG_set_ON_cli,
        "Real-Time logging: set to ON",
        "<rtlindex>",
        "aolrtlogsetON 2",
        "int AOloopControl_RTstreamLOG_set_ON(int loop, int rtlindex)");

    RegisterCLIcommand(
        "aolrtlogsetOFF",
        __FILE__,
        AOloopControl_RTstreamLOG_set_OFF_cli,
        "Real-Time logging: set to OFF",
        "<rtlindex>",
        "aolrtlogsetOFF 2",
        "int AOloopControl_RTstreamLOG_set_OFF(int loop, int rtlindex)");


    CLIADDCMD_AOloopControl__modalfilter();
    CLIADDCMD_AOloopControl__modalCTRL_stats();

/*    RegisterCLIcommand(
        "modalstatsTUI",
        __FILE__,
        AOloopControl_modalstatsTUI_cli,
        "AO loop modal stats TUI",
        "<loopindex>",
        "aomodalstatsTIU 2",
        "AOloopControl_modalstatsTUI(int loopindex)");
*/


    // add atexit functions here
    // atexit((void*) myfunc); atexit = starts a function once the program exits
    // (only if it is not a crash exit)

    return RETURN_SUCCESS;
}






/*-------------------------------------------------------------------------------*/


/*
errno_t AOloopControl_setgainrange(long m0, long m1, float gainval)
{

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_DMmode_GAIN == -1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_GAIN", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    unsigned long kmax = m1 + 1;
    if(kmax > AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes)
    {
        kmax = AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes - 1;
    }

    for(unsigned long k = m0; k < kmax; k++)
    {
        data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k] = gainval;
    }

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setlimitrange(long m0, long m1, float limval)
{

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_LIMIT", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    unsigned long kmax = m1 + 1;
    if(kmax > AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes)
    {
        kmax = AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes - 1;
    }

    for(unsigned long k = m0; k < kmax; k++)
    {
        data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k] = limval;
    }

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setmultfrange(long m0, long m1, float multfval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_MULTF_modes == -1)
    {
        char name[200];
        if(sprintf(name, "aol%ld_DMmode_MULTF", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(name);
    }

    unsigned long kmax = m1 + 1;
    if(kmax > AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes)
    {
        kmax = AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes - 1;
    }

    for(unsigned long k = m0; k < kmax; k++)
    {
        data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[k] =
            multfval;
    }

    return RETURN_SUCCESS;
}
*/


/*
errno_t AOloopControl_setgainblock(long mb, float gainval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_gainb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_gainb", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_gainb = read_sharedmem_image(imname);
    }

    if(mb < (long) AOconf[LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock)
    {
        data.image[aoloopcontrol_var.aoconfID_gainb].array.F[mb] = gainval;
    }

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setlimitblock(long mb, float limitval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_limitb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_limitb", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_limitb = read_sharedmem_image(imname);
    }

    if(mb < (long) AOconf[LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock)
    {
        data.image[aoloopcontrol_var.aoconfID_limitb].array.F[mb] = limitval;
    }

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setmultfblock(long mb, float multfval)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_multfb == -1)
    {
        char imname[200];
        if(sprintf(imname, "aol%ld_multfb", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_multfb = read_sharedmem_image(imname);
    }

    if(mb < (long) AOconf[LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock)
    {
        data.image[aoloopcontrol_var.aoconfID_multfb].array.F[mb] = multfval;
    }

    return RETURN_SUCCESS;
}
*/

