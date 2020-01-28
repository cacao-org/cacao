/**
 * @file    AOloopControl_PredictiveControl.c
 * @brief   Adaptive Optics Control loop engine Predictive Control
 * 
 * Adaptive Optics predictive control
 *
 * 
 * 
 */



#define _GNU_SOURCE




/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */



#include "CommandLineInterface/CLIcore.h"


#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"







static int INITSTATUS_AOloopControl_PredictiveControl = 0;











/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL
 *  Predictive control using WFS telemetry */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_builPFloop_WatchInput */
errno_t AOloopControl_PredictiveControl_builPFloop_WatchInput_cli() {
    if(
        CLI_checkarg(1,2)+
        CLI_checkarg(2,2)+
        CLI_checkarg(3,2)+
        CLI_checkarg(4,2)+
        CLI_checkarg(5,2)
        == 0 )
    {
        AOloopControl_PredictiveControl_builPFloop_WatchInput(
            data.cmdargtoken[1].val.numl,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.numl,
            data.cmdargtoken[4].val.numl,
            data.cmdargtoken[5].val.numl
        );

        return CLICMD_SUCCESS;
    }
    else {
        return CLICMD_INVALID_ARG;
    }
}


/** @brief CLI function for AOloopControl_mapPredictiveFilter */
errno_t AOloopControl_PredictiveControl_mapPredictiveFilter_cli() {
    if(
    CLI_checkarg(1,4)+
    CLI_checkarg(2,2)+
    CLI_checkarg(3,1)
    == 0 ) 
    {
        AOloopControl_PredictiveControl_mapPredictiveFilter(
        data.cmdargtoken[1].val.string, 
        data.cmdargtoken[2].val.numl, 
        data.cmdargtoken[3].val.numf
        );
        
        return CLICMD_SUCCESS;
    }
    else return CLICMD_INVALID_ARG;
}


/** @brief CLI function for AOloopControl_testPredictiveFilter */
errno_t AOloopControl_PredictiveControl_testPredictiveFilter_cli() {
    if(
        CLI_checkarg(1,4) +
        CLI_checkarg(2,2) +
        CLI_checkarg(3,1) +
        CLI_checkarg(4,2) +
        CLI_checkarg(5,3)
        == 0 )
    {
        AOloopControl_PredictiveControl_testPredictiveFilter(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.numf,
            data.cmdargtoken[4].val.numl,
            data.cmdargtoken[5].val.string,
            1e-10
        );

        return CLICMD_SUCCESS;
    }
    else {
        return CLICMD_INVALID_ARG;
    }
}



errno_t AOloopControl_PredictiveControl_setPFsimpleAve_cli() {
    if(
        CLI_checkarg(1,4) +
        CLI_checkarg(2,1)
        == 0 )
    {
        AOloopControl_PredictiveControl_setPFsimpleAve(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numf
        );

        return CLICMD_SUCCESS;
    }
    else {
        return CLICMD_INVALID_ARG;
    }
}






/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl functions */


void __attribute__ ((constructor)) libinit_AOloopControl_PredictiveControl()
{
	if(INITSTATUS_AOloopControl_PredictiveControl == 0)
	{
		init_AOloopControl_PredictiveControl();
		RegisterModule(__FILE__, "cacao", "AO loop control predictive control");
		INITSTATUS_AOloopControl_PredictiveControl = 1;
	}
}


errno_t init_AOloopControl_PredictiveControl()
{

    /* =============================================================================================== */
    /* =============================================================================================== */
    /** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL                                                    */
    /* =============================================================================================== */
    /* =============================================================================================== */

    RegisterCLIcommand(
        "aolPFwatchin",
        __FILE__,
        AOloopControl_PredictiveControl_builPFloop_WatchInput_cli,
        "watch telemetry for predictive filter input",
        "<loop #> <PFblock #> <start> <end>", "aolPFwatchin 0 2",
        "long AOloopControl_builPFloop_WatchInput(long loop, long PFblock, long PFblockStart, long PFblockEnd, long NBbuff)"
    );

    RegisterCLIcommand(
        "aolmappfilt",
        __FILE__,
        AOloopControl_PredictiveControl_mapPredictiveFilter_cli,
        "map/search predictive filter",
        "<input coeffs> <mode number> <delay [frames]>", "aolmkapfilt coeffim 23 2.4",
        "long AOloopControl_mapPredictiveFilter(char *IDmodecoeff_name, long modeout, double delayfr)"
    );

    RegisterCLIcommand(
        "aolmkpfilt",
        __FILE__,
        AOloopControl_PredictiveControl_testPredictiveFilter_cli,
        "test predictive filter",
        "<trace im> <mode number> <delay [frames]> <filter size> <out filter name>",
        "aolmkpfilt traceim 23 2.4 20 filt23","long AOloopControl_testPredictiveFilter(char *IDtrace_name, long mode, double delayfr, long filtsize, char *IDfilt_name, double SVDeps)"
    );


    RegisterCLIcommand(
        "aolpfsetave",
        __FILE__,
        AOloopControl_PredictiveControl_setPFsimpleAve_cli,
        "set predictive filter to integrator",
        "<PredFilter> <DecayCoeff>", "aolpfsetave outPFb0 0.5",
        "long AOloopControl_PredictiveControl_setPFsimpleAve(char *IDPF_name, float DecayCoeff)"
    );



    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}



































