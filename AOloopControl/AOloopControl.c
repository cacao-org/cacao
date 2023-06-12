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
 * AOloopControl_initmem.c                   Initialize memory
 * AOloopControl_loadconfigure.c             Load and save configuration
 * AOloopControl_aorun.c                     Main level loop function, calls
 * AOcompute AOloopControl_AOcompute.c                 AO Compute function, WFS
 * to DM commands AOloopControl_wfs.c                       Read WFS data, low
 * level processing of WFS data AOloopControl_dm.c Deformable mirror control,
 * write modes to DM AOloopControl_dmwrite.c                   Turn DM write
 * on/off AOloopControl_loop_ctr.c                  Loop control user interface
 * AOloopControl_loop_onoff.c                Turn loop on/off, pause, step
 * AOloopControl_param.c                     Set parameters
 * AOloopControl_read_param.c                Read parameters
 * AOloopControl_ProcessModeCoefficients.c   Modal control
 * AOloopControl_CompModes_loop.c            Compute modes (loop)
 *
 *
 * ## I/O, telemetry
 *
 * AOloopControl_RTstreamLOG.c               Log real-time telemetry
 *
 *
 * ## Performance tuning / optimization
 *
 * AOloopControl_arpf_onoff.c                Autoregressive predictive control
 * On/Off AOloopControl_predfilter_onoff.c          Turn predictive control
 * on/off AOloopControl_autotune.c                  Automatic loop tuning
 * (mostly gain values) AOloopControl_process_files.c             --
 * EXPERIMENTAL -- RM optimization from telemetry
 *
 *
 * AOloopControl_time_param.c                ??
 * AOloopControl_sig2Modecoeff.c             ??
 * AOloopControl_fpspeckle_mod.c             Speckle modulation (experimental)
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

AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

AOloopControl_var aoloopcontrol_var;

/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl)

/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 1. INITIALIZATION, configurations
 *  Allocate memory, import/export configurations */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_loadconfigure */

errno_t AOloopControl_loadconfigure_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_loadconfigure(data.cmdargtoken[1].val.numl, 1, 10);
        return 0;
    }
    else
    {
        return 1;
    }
}

/** @brief CLI function for AOloopControl_stream3Dto2D */
/*errno_t AOloopControl_stream3Dto2D_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0)
{ AOloopControl_stream3Dto2D(data.cmdargtoken[1].val.string,
data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl,
data.cmdargtoken[4].val.numl); return 0;
    }
    else return 1;
}*/

/** @brief CLI function for AOloopControl_aorun*/
errno_t AOloopControl_aorun_cli()
{
    function_parameter_getFPSargs_from_CLIfunc("aorun");

    if(data.FPS_CMDCODE != 0)  // use FPS implementation
    {
        // set pointers to CONF and RUN functions
        data.FPS_CONFfunc = AOloopControl_aorun_FPCONF;
        data.FPS_RUNfunc  = AOloopControl_aorun_RUN;
        function_parameter_execFPScmd();
        return RETURN_SUCCESS;
    }

    // call non FPS implementation - all parameters specified at function launch
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_IMG) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_aorun(data.cmdargtoken[1].val.string,
                            data.cmdargtoken[3].val.numf);
        return RETURN_SUCCESS;
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
/** @name AOloopControl - 6. REAL TIME COMPUTING ROUTINES
 *  calls CPU and GPU processing */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_AOcompute_GUI */
errno_t AOloopControl_AOcompute_GUI_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_AOcompute_GUI(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numf);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_aorun_GUI */
errno_t AOloopControl_aorun_GUI_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_aorun_GUI(data.cmdargtoken[1].val.numl,
                                data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

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

/** @brief CLI function for AOloopControl_AutoTuneGains */
errno_t AOloopControl_AutoTuneGains_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_STR_NOT_IMG) +
            CLI_checkarg(3, CLIARG_FLOAT64) + CLI_checkarg(4, CLIARG_INT64) ==
            0)
    {
        AOloopControl_AutoTuneGains(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.string,
                                    data.cmdargtoken[3].val.numf,
                                    data.cmdargtoken[4].val.numl);
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
/** @name AOloopControl - 8.   LOOP CONTROL INTERFACE */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_setLoopNumber */
errno_t AOloopControl_setLoopNumber_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_setLoopNumber(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/* ===============================================================================================
 */
/** @name AOloopControl - 8.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP
 * ON/OFF START/STOP/STEP/RESET */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.2. LOOP CONTROL INTERFACE - DATA LOGGING */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.3. LOOP CONTROL INTERFACE - PRIMARY DM WRITE */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING
 */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF
 */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS */
/* ===============================================================================================
 */

/* ===============================================================================================
 */
/** @name AOloopControl - 8.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_set_modeblock_gain */
errno_t AOloopControl_set_modeblock_gain_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_FLOAT64) + CLI_checkarg(3, CLIARG_INT64) ==
            0)
    {
        AOloopControl_set_modeblock_gain(LOOPNUMBER,
                                         data.cmdargtoken[1].val.numl,
                                         data.cmdargtoken[2].val.numf,
                                         data.cmdargtoken[3].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_loopstep */
errno_t AOloopControl_loopstep_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) == 0)
    {
        AOloopControl_loopstep(LOOPNUMBER, data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_loopfrequ */
errno_t AOloopControl_set_loopfrequ_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_loopfrequ(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_hardwlatency_frame */
errno_t AOloopControl_set_hardwlatency_frame_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_hardwlatency_frame(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_complatency_frame */
errno_t AOloopControl_set_complatency_frame_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_complatency_frame(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_wfsmextrlatency_frame */
errno_t AOloopControl_set_wfsmextrlatency_frame_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_wfsmextrlatency_frame(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_delta */
errno_t AOloopControl_set_AUTOTUNE_LIMITS_delta_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_AUTOTUNE_LIMITS_delta(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_perc */
errno_t AOloopControl_set_AUTOTUNE_LIMITS_perc_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_AUTOTUNE_LIMITS_perc(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_set_AUTOTUNE_LIMITS_mcoeff */
errno_t AOloopControl_set_AUTOTUNE_LIMITS_mcoeff_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setgain */
errno_t AOloopControl_setgain_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setgain(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setARPFgain */
errno_t AOloopControl_setARPFgain_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setARPFgain(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setARPFgain */
errno_t AOloopControl_setARPFgainAutoMin_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setARPFgainAutoMin(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setARPFgain */
errno_t AOloopControl_setARPFgainAutoMax_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setARPFgainAutoMax(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setWFSnormfloor */
errno_t AOloopControl_setWFSnormfloor_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setWFSnormfloor(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setmaxlimit */
errno_t AOloopControl_setmaxlimit_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setmaxlimit(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setmult */
errno_t AOloopControl_setmult_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setmult(data.cmdargtoken[1].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setframesAve */
/*errno_t AOloopControl_setframesAve_cli() {
    if(CLI_checkarg(1,2)==0) {
        AOloopControl_setframesAve(data.cmdargtoken[1].val.numl);
        return 0;
    }
    else return 1;
}
*
*/

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c

/** @brief CLI function for AOloopControl_setgainrange */
errno_t AOloopControl_setgainrange_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_INT64) + CLI_checkarg(3, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setgainrange(data.cmdargtoken[1].val.numl,
                                   data.cmdargtoken[2].val.numl,
                                   data.cmdargtoken[3].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setlimitrange */
errno_t AOloopControl_setlimitrange_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_INT64) + CLI_checkarg(3, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setlimitrange(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numl,
                                    data.cmdargtoken[3].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setmultfrange */
errno_t AOloopControl_setmultfrange_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_INT64) + CLI_checkarg(3, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setmultfrange(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numl,
                                    data.cmdargtoken[3].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setgainblock */
errno_t AOloopControl_setgainblock_cli()
{
    if(CLI_checkarg(1, CLIARG_INT64) + CLI_checkarg(2, CLIARG_FLOAT64) == 0)
    {
        AOloopControl_setgainblock(data.cmdargtoken[1].val.numl,
                                   data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setlimitblock */
errno_t AOloopControl_setlimitblock_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setlimitblock(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_setmultfblock */
errno_t AOloopControl_setmultfblock_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_FLOAT64) ==
            0)
    {
        AOloopControl_setmultfblock(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_scanGainBlock */
errno_t AOloopControl_scanGainBlock_cli()
{
    if(DISABLE_CLI_AOloopControl + CLI_checkarg(1, CLIARG_INT64) +
            CLI_checkarg(2, CLIARG_INT64) + CLI_checkarg(3, CLIARG_FLOAT64) +
            CLI_checkarg(4, CLIARG_FLOAT64) + CLI_checkarg(5, CLIARG_INT64) ==
            0)
    {
        AOloopControl_scanGainBlock(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numl,
                                    data.cmdargtoken[3].val.numf,
                                    data.cmdargtoken[4].val.numf,
                                    data.cmdargtoken[5].val.numl);
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

    aoloopcontrol_var.AOloopcontrol_meminit = 0;
    aoloopcontrol_var.init_RM_local         = 0;
    aoloopcontrol_var.init_CM_local         = 0;
    aoloopcontrol_var.init_CMc_local        = 0;

    aoloopcontrol_var.GPU_alpha = 0.0;
    aoloopcontrol_var.GPU_beta  = 0.0;

    aoloopcontrol_var.COMPUTE_PIXELSTREAMING    = 0;
    aoloopcontrol_var.PIXSTREAM_NBSLICES        = 1;
    aoloopcontrol_var.aoconfID_wfsim            = -1;
    aoloopcontrol_var.aoconfID_dmC              = -1;
    aoloopcontrol_var.aoconfID_dmRM             = -1;
    aoloopcontrol_var.aoconfID_wfsdark          = -1;
    aoloopcontrol_var.aoconfID_imWFS0           = -1;
    aoloopcontrol_var.aoconfID_imWFS0tot        = -1;
    aoloopcontrol_var.aoconfID_imWFS1           = -1;
    aoloopcontrol_var.aoconfID_imWFS2           = -1;
    aoloopcontrol_var.aoconfID_imWFSlinlimit    = -1;
    aoloopcontrol_var.aoconfID_wfsref0          = -1;
    aoloopcontrol_var.aoconfID_wfsref           = -1;
    aoloopcontrol_var.aoconfcnt0_wfsref_current = -1;
    aoloopcontrol_var.aoconfID_DMmodes          = -1;
    aoloopcontrol_var.aoconfID_dmdisp           = -1;
    aoloopcontrol_var.aoconfID_cmd_modes        = -1;
    aoloopcontrol_var.aoconfID_meas_modes       = -1;
    aoloopcontrol_var.aoconfID_RMS_modes        = -1;
    aoloopcontrol_var.aoconfID_AVE_modes        = -1;
    aoloopcontrol_var.aoconfID_modeARPFgainAuto = -1;
    aoloopcontrol_var.aoconfID_modevalPF        = -1;
    aoloopcontrol_var.aoconfID_gainb            = -1;
    aoloopcontrol_var.aoconfID_multfb           = -1;
    aoloopcontrol_var.aoconfID_limitb           = -1;
    aoloopcontrol_var.aoconfID_DMmode_GAIN      = -1;
    aoloopcontrol_var.aoconfID_LIMIT_modes      = -1;
    aoloopcontrol_var.aoconfID_MULTF_modes      = -1;
    aoloopcontrol_var.aoconfID_cmd_modesRM      = -1;
    aoloopcontrol_var.aoconfID_wfsmask          = -1;
    aoloopcontrol_var.aoconfID_dmmask           = -1;
    aoloopcontrol_var.aoconfID_contrM           = -1;
    aoloopcontrol_var.aoconfID_contrMc          = -1;
    aoloopcontrol_var.aoconfID_meas_act         = -1;

    int i;
    for(i = 0; i < 100; i++)
    {
        aoloopcontrol_var.aoconfID_contrMcact[i] = -1;
        aoloopcontrol_var.initcontrMcact_GPU[i]  = -1;
    }

    aoloopcontrol_var.aoconfID_looptiming = -1;
    aoloopcontrol_var.AOcontrolNBtimers   = 35;

    aoloopcontrol_var.aoconfIDlogdata          = -1;
    aoloopcontrol_var.aoconfID_meas_act_active = -1;

    for(i = 0; i < MAX_NUMBER_RTLOGSTREAM; i++)
    {
        aoloopcontrol_var.RTSLOGarrayInitFlag[i] = 0;
    }

    RegisterCLIcommand("aolloadconf",
                       __FILE__,
                       AOloopControl_loadconfigure_cli,
                       "load AO loop configuration",
                       "<loop #>",
                       "AOlooploadconf 1",
                       "int AOloopControl_loadconfigure(long loopnb, 1, 10)");

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

    RegisterCLIcommand("aolrun",
                       __FILE__,
                       AOloopControl_aorun_cli, // Run AO loop
                       "run AO loop",
                       "no arg",
                       "aolrun",
                       "int AOloopControl_aorun()");

    RegisterCLIcommand(
        "aolcompGUI",
        __FILE__,
        AOloopControl_AOcompute_GUI_cli, // AOcompute GUI
        "AOcompute GUI",
        "<loop> <frequ [Hz]>",
        "aolcompGUI 0 20",
        "int AOloopControl_AOcompute_GUI(long loop, double frequ)");

    RegisterCLIcommand("aolrunGUI",
                       __FILE__,
                       AOloopControl_aorun_GUI_cli, // AOcompute GUI
                       "aorun GUI",
                       "<loop> <frequ [Hz]>",
                       "aolrunGUI 0 20",
                       "int AOloopControl_aorun_GUI(long loop, double frequ)");

    RegisterCLIcommand(
        "aolzpwfsloop",
        __FILE__,
        AOloopControl_WFSzpupdate_loop_cli, // WFS zero point offset loop
        "WFS zero point offset loop",
        "<dm offset [shared mem]> <zonal resp M [shared mem]> <nominal WFS "
        "reference>  <modified WFS reference>",
        "aolzpwfsloop dmZP zrespM wfszp",
        "int AOloopControl_WFSzpupdate_loop(char *IDzpdm_name, char "
        "*IDzrespM_name, char *IDwfszp_name)");

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

    RegisterCLIcommand(
        "aolnb",
        __FILE__,
        AOloopControl_setLoopNumber_cli,
        "set AO loop #",
        "<loop nb>",
        "AOloopnb 0",
        "int AOloopControl_setLoopNumber(long loop)"); // set AO loop

    RegisterCLIcommand("aolsetgain",
                       __FILE__,
                       AOloopControl_setgain_cli,
                       "set gain",
                       "<gain value>",
                       "aolsetgain 0.1",
                       "int AOloopControl_setgain(float gain)");

    RegisterCLIcommand("aolsetARPFgain",
                       __FILE__,
                       AOloopControl_setARPFgain_cli,
                       "set auto-regressive predictive filter gain",
                       "<gain value>",
                       "aolsetARPFgain 0.1",
                       "int AOloopControl_setARPFgain(float gain)");

    RegisterCLIcommand("aolsetARPFgainAmin",
                       __FILE__,
                       AOloopControl_setARPFgainAutoMin_cli,
                       "set ARPF gain min",
                       "<gain value>",
                       "aolsetARPFgainAmin 0.1",
                       "int AOloopControl_setARPFgainAutoMin(float val)");

    RegisterCLIcommand("aolsetARPFgainAmax",
                       __FILE__,
                       AOloopControl_setARPFgainAutoMax_cli,
                       "set ARPF gain max",
                       "<gain value>",
                       "aolsetARPFgainAmax 9.0",
                       "int AOloopControl_setARPFgainAutoMax(float val)");

    RegisterCLIcommand("aolkill",
                       __FILE__,
                       AOloopControl_loopkill,
                       "kill AO loop",
                       "no arg",
                       "aolkill",
                       "int AOloopControl_setLoopNumber()");

    RegisterCLIcommand("aolon",
                       __FILE__,
                       AOloopControl_loopon,
                       "turn loop on",
                       "no arg",
                       "aolon",
                       "int AOloopControl_loopon()");

    RegisterCLIcommand("aoloff",
                       __FILE__,
                       AOloopControl_loopoff,
                       "turn loop off",
                       "no arg",
                       "aoloff",
                       "int AOloopControl_loopoff()");

    RegisterCLIcommand("aolWFScompon",
                       __FILE__,
                       AOloopControl_loopWFScompon,
                       "turn loop WFScomp on",
                       "no arg",
                       "aolWFScompon",
                       "int AOloopControl_loopWFScompon()");

    RegisterCLIcommand("aolWFScompoff",
                       __FILE__,
                       AOloopControl_loopWFScompoff,
                       "turn loop WFScomp off",
                       "no arg",
                       "aolWFScompoff",
                       "int AOloopControl_loopWFScompoff()");

    RegisterCLIcommand("aolstep",
                       __FILE__,
                       AOloopControl_loopstep_cli,
                       "turn loop on for N steps",
                       "<nbstep>",
                       "aolstep",
                       "int AOloopControl_loopstep(long loop, long NBstep)");

    RegisterCLIcommand("aolreset",
                       __FILE__,
                       AOloopControl_loopreset,
                       "reset loop, and turn it off",
                       "no arg",
                       "aolreset",
                       "int AOloopControl_loopreset()");

    RegisterCLIcommand("aolsetmbgain",
                       __FILE__,
                       AOloopControl_set_modeblock_gain_cli,
                       "set modal block gain",
                       "<loop #> <gain> <compute sum flag>",
                       "aolsetmbgain 2 0.2 1",
                       "int AOloopControl_set_modeblock_gain(long loop, long "
                       "blocknb, float gain, int add)");

    RegisterCLIcommand("aolDMprimWon",
                       __FILE__,
                       AOloopControl_DMprimaryWrite_on,
                       "turn DM primary write on",
                       "no arg",
                       "aolDMprimWon",
                       "int AOloopControl_DMprimaryWrite_on()");

    RegisterCLIcommand("aolDMprimWoff",
                       __FILE__,
                       AOloopControl_DMprimaryWrite_off,
                       "turn DM primary write off",
                       "no arg",
                       "aolDMprimWoff",
                       "int AOloopControl_DMprimaryWrite_off()");

    RegisterCLIcommand("aolDMfiltWon",
                       __FILE__,
                       AOloopControl_DMfilteredWrite_on,
                       "turn DM filtered write on",
                       "no arg",
                       "aolDMfiltWon",
                       "int AOloopControl_DMfilteredWrite_on()");

    RegisterCLIcommand("aolDMfiltWoff",
                       __FILE__,
                       AOloopControl_DMfilteredWrite_off,
                       "turn DM filtered write off",
                       "no arg",
                       "aolDMfiltWoff",
                       "int AOloopControl_DMfilteredWrite_off()");

    RegisterCLIcommand("aolAUTOTUNELIMon",
                       __FILE__,
                       AOloopControl_AUTOTUNE_LIMITS_on,
                       "turn auto-tuning modal limits on",
                       "no arg",
                       "aolAUTOTUNELIMon",
                       "int AOloopControl_AUTOTUNE_LIMITS_on()");

    RegisterCLIcommand("aolAUTOTUNELIMoff",
                       __FILE__,
                       AOloopControl_AUTOTUNE_LIMITS_off,
                       "turn auto-tuning modal limits off",
                       "no arg",
                       "aolAUTOTUNELIMoff",
                       "int AOloopControl_AUTOTUNE_LIMITS_off()");

    RegisterCLIcommand("aolsetATlimd",
                       __FILE__,
                       AOloopControl_set_AUTOTUNE_LIMITS_delta_cli,
                       "set auto-tuning modal limits delta",
                       "<delta value [um]>",
                       "aolsetATlimd 0.0001",
                       "int AOloopControl_set_AUTOTUNE_LIMITS_delta(float "
                       "AUTOTUNE_LIMITS_delta)");

    RegisterCLIcommand("aolsetATlimp",
                       __FILE__,
                       AOloopControl_set_AUTOTUNE_LIMITS_perc_cli,
                       "set auto-tuning modal limits percentile",
                       "<percentile value [percent]>",
                       "aolsetATlimp 1.0",
                       "int AOloopControl_set_AUTOTUNE_LIMITS_perc(float "
                       "AUTOTUNE_LIMITS_perc)");

    RegisterCLIcommand("aolsetATlimm",
                       __FILE__,
                       AOloopControl_set_AUTOTUNE_LIMITS_mcoeff_cli,
                       "set auto-tuning modal limits multiplicative coeff",
                       "<multiplicative coeff [float]>",
                       "aolsetATlimm 1.5",
                       "int AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(float "
                       "AUTOTUNE_LIMITS_mcoeff)");

    RegisterCLIcommand("aolAUTOTUNEGAINon",
                       __FILE__,
                       AOloopControl_AUTOTUNE_GAINS_on,
                       "turn auto-tuning modal gains on",
                       "no arg",
                       "aolAUTOTUNEGAINon",
                       "int AOloopControl_AUTOTUNE_GAINS_on()");

    RegisterCLIcommand("aolAUTOTUNEGAINoff",
                       __FILE__,
                       AOloopControl_AUTOTUNE_GAINS_off,
                       "turn auto-tuning modal gains off",
                       "no arg",
                       "aolAUTOTUNEGAINoff",
                       "int AOloopControl_AUTOTUNE_GAINS_off()");

    RegisterCLIcommand("aolARPFon",
                       __FILE__,
                       AOloopControl_ARPFon,
                       "turn auto-regressive predictive filter on",
                       "no arg",
                       "aolARPFon",
                       "int AOloopControl_ARPFon()");

    RegisterCLIcommand("aolARPFoff",
                       __FILE__,
                       AOloopControl_ARPFoff,
                       "turn auto-regressive predictive filter off",
                       "no arg",
                       "aolARPFoff",
                       "int AOloopControl_ARPFoff()");

    /* ===============================================================================================
    */
    /* ===============================================================================================
    */
    /** @name AOloopControl - 11. PROCESS LOG FILES */
    /* ===============================================================================================
    */
    /* ===============================================================================================
    */

    RegisterCLIcommand("aolsetloopfrequ",
                       __FILE__,
                       AOloopControl_set_loopfrequ_cli,
                       "set loop frequency",
                       "<loop frequ [Hz]>",
                       "aolsetloopfrequ 2000",
                       "int AOloopControl_set_loopfrequ(float loopfrequ)");

    RegisterCLIcommand(
        "aolsethlat",
        __FILE__,
        AOloopControl_set_hardwlatency_frame_cli,
        "set hardware latency",
        "<hardware latency [frame]>",
        "aolsethlat 2.7",
        "int AOloopControl_set_hardwlatency_frame(float hardwlatency_frame)");

    RegisterCLIcommand(
        "aolsetclat",
        __FILE__,
        AOloopControl_set_complatency_frame_cli,
        "set computation latency",
        "<computation latency [frame]>",
        "aolsetclat 0.6",
        "int AOloopControl_set_complatency_frame(float complatency_frame)");

    RegisterCLIcommand("aolsetwlat",
                       __FILE__,
                       AOloopControl_set_wfsmextrlatency_frame_cli,
                       "set WFS mode extraction latency",
                       "<latency [frame]>",
                       "aolsetwlat 0.8",
                       "int AOloopControl_set_wfsmextrlatency_frame(float "
                       "wfsmextrlatency_frame)");

    RegisterCLIcommand("aolsetwfsnormf",
                       __FILE__,
                       AOloopControl_setWFSnormfloor_cli,
                       "set WFS normalization floor",
                       "<floor value (total flux)>",
                       "aolsetwfsnormf 10000.0",
                       "int AOloopControl_setWFSnormfloor(float WFSnormfloor)");

    RegisterCLIcommand("aolsetmaxlim",
                       __FILE__,
                       AOloopControl_setmaxlimit_cli,
                       "set max limit for AO mode correction",
                       "<limit value>",
                       "aolsetmaxlim 0.01",
                       "int AOloopControl_setmaxlimit(float maxlimit)");

    RegisterCLIcommand("aolsetmult",
                       __FILE__,
                       AOloopControl_setmult_cli,
                       "set mult coeff for AO mode correction",
                       "<mult value>",
                       "aolsetmult 0.98",
                       "int AOloopControl_setmult(float multcoeff)");

    RegisterCLIcommand(
        "aollogprocmodeval",
        __FILE__,
        AOloopControl_logprocess_modeval_cli,
        "process log image modeval",
        "<modeval image>",
        "aollogprocmodeval imc",
        "int AOloopControl_logprocess_modeval(const char *IDname);");

    RegisterCLIcommand(
        "aolsetgainr",
        __FILE__,
        AOloopControl_setgainrange_cli,
        "set modal gains from m0 to m1 included",
        "<modemin [long]> <modemax [long]> <gainval>",
        "aolsetgainr 20 30 0.2",
        "int AOloopControl_setgainrange(long m0, long m1, float gainval)");

    RegisterCLIcommand(
        "aolsetlimitr",
        __FILE__,
        AOloopControl_setlimitrange_cli,
        "set modal limits",
        "<modemin [long]> <modemax [long]> <limval>",
        "aolsetlimitr 20 30 0.02",
        "int AOloopControl_setlimitrange(long m0, long m1, float gainval)");

    RegisterCLIcommand(
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

    RegisterCLIcommand("aolautotunegains",
                       __FILE__,
                       AOloopControl_AutoTuneGains_cli,
                       "compute optimal gains",
                       "<loop #> <gain stream> <gaincoeff> <NBsamples>",
                       "aolautotunegains 0 autogain 0.1 20000",
                       "long AOloopControl_AutoTuneGains(long loop, const char "
                       "*IDout_name, float GainCoeff, long NBsamples)");

    RegisterCLIcommand(
        "aoldm2dmoffload",
        __FILE__,
        AOloopControl_dm2dm_offload_cli,
        "slow offload from dm to dm",
        "<streamin> <streamout> <timestep[sec]> <offloadcoeff> <multcoeff>",
        "aoldm2dmoffload dmin dmout 0.5 -0.01 0.999",
        "long AOloopControl_dm2dm_offload(const char *streamin, const char "
        "*streamout, float twait, "
        "float offcoeff, float multcoeff)");

    RegisterCLIcommand("aolautotune",
                       __FILE__,
                       AOloopControl_AutoTune,
                       "auto tuning of loop parameters",
                       "no arg",
                       "aolautotune",
                       "errno_t AOloopControl_AutoTune()");

    RegisterCLIcommand(
        "aolset",
        __FILE__,
        AOloopControl_setparam_cli,
        "set parameter",
        "<parameter> <value>",
        "aolset",
        "int AOloopControl_setparam(long loop, const char *key, double value)");

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

    RegisterCLIcommand(
        "modalstatsTUI",
        __FILE__,
        AOloopControl_modalstatsTUI_cli,
        "AO loop modal stats TUI",
        "<loopindex>",
        "aomodalstatsTIU 2",
        "AOloopControl_modalstatsTUI(int loopindex)");



    // add atexit functions here
    // atexit((void*) myfunc); atexit = starts a function once the program exits
    // (only if it is not a crash exit)

    return RETURN_SUCCESS;
}

/*-------------------------------------------------------------------------------*/

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

errno_t AOloopControl_AutoTune()
{
    long  NBstep = 10000;
    char  name[200];
    float bestgain = 0.0;
    float val;

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_cmd_modes == -1)
    {
        if(sprintf(name, "aol%ld_DMmode_cmd", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    // initialize
    for(unsigned long block = 0;
            block < AOconf[LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock;
            block++)
    {
        AOloopControl_setgainblock(block, 0.0);
        AOloopControl_setlimitblock(block, 0.1);
        AOloopControl_setmultfblock(block, 0.8);
    }

    for(unsigned long block = 0;
            block < AOconf[LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock;
            block++)
    {
        float gainStart = 0.0;
        float gainEnd   = 1.0;
        int   gOK       = 1;
        float gain;
        float bestval = 10000000.0;

        // tune block gain
        gain = gainStart;
        while((gOK == 1) && (gain < gainEnd))
        {
            for(unsigned long k = 0;
                    k < AOconf[LOOPNUMBER].AOpmodecoeffs.NBDMmodes;
                    k++)
            {
                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] =
                    0.0;
            }

            gain += 0.01;
            gain *= 1.1;

            AOloopControl_setgainblock(block, gain);
            AOloopControl_loopstep(LOOPNUMBER, NBstep);
            val = sqrt(AOconf[LOOPNUMBER].AOpmodecoeffs.RMSmodesCumul /
                       AOconf[LOOPNUMBER].AOpmodecoeffs.RMSmodesCumulcnt);
            printf("%6.4f  %10.8lf\n", gain, val);

            if(val < bestval)
            {
                bestval  = val;
                bestgain = gain;
            }
            else
            {
                gOK = 0;
            }
        }
        printf("BLOCK %ld  : BEST GAIN = %f\n", block, bestgain);

        AOloopControl_setgainblock(block, bestgain);
    }

    return RETURN_SUCCESS;
}
