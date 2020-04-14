/**
 * @file    AOloopControl_perfTest_LinSim.c
 * @brief   Adaptive Optics Control loop linear simulator
 * 
 * Uses response matrix for linear simulation
 *  
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */
#include <string.h>
#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

#include "linopt_imtools/linopt_imtools.h"
#include "info/info.h"
#include "AOloopControl/AOloopControl.h"

#include "AOloopControl_perfTest/AOloopControl_perfTest.h"


/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */



# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */





errno_t AOcontrolLoop_perfTest_LinearSimulator_FPCONF(
    const char *fpsname,
    uint32_t    CMDmode
)
{
    //uint16_t loopstatus;

    // SETUP FPS
    /*int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps = function_parameter_FPCONFsetup(fpsname, CMDmode, &loopstatus, &SMfd);
    strncpy(fps.md->sourcefname, __FILE__, FPS_SRCDIR_STRLENMAX);
    fps.md->sourceline = __LINE__;
    */
    FPS_SETUP_INIT(fpsname, CMDmode);


    // ALLOCATE ENTRIES
    void * pNull = NULL;

    __attribute__((unused)) long fpi_DMxsize =
        function_parameter_add_entry(&fps, ".DMxsize", "Deformable mirror X size", FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, pNull);

    __attribute__((unused)) long fpi_DMysize =
        function_parameter_add_entry(&fps, ".DMysize", "Deformable mirror Y size", FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, pNull);


    if( fps.loopstatus == 0 ) {// stop fps
        return RETURN_SUCCESS;
    }

    // RUN UPDATE LOOP
    while( fps.loopstatus == 1 )
    {
        if( function_parameter_FPCONFloopstep(&fps) == 1)
        {
            // here goes the logic
            functionparameter_CheckParametersAll(&fps);  // check all parameter values
        }
    }
    function_parameter_FPCONFexit( &fps );


    return RETURN_SUCCESS;
}








errno_t AOcontrolLoop_perfTest_LinearSimulator_RUN(
	const char *fpsname
)
{
	FUNCTION_PARAMETER_STRUCT fps;
	//int SMfd = -1;
	
	int FPSINTERFACE = 1;
	
	if(function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_RUN) == -1)
	{
		printf("ERROR: fps \"%s\" does not exist -> running without FPS interface\n", fpsname);
		FPSINTERFACE = 0;
	}
	else
	{
		FPSINTERFACE = 1;
	}


	if(FPSINTERFACE == 1)
		function_parameter_struct_disconnect(&fps);


    return(0);
}







