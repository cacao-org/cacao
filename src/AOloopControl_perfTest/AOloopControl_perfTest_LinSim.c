/**
 * @file    AOloopControl_perfTest_LinSim.c
 * @brief   Adaptive Optics Control loop linear simulator
 * 
 * Uses response matrix for linear simulation
 *  
 *
 * 
 * @bug No known bugs.
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
#include "00CORE/00CORE.h"

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





int_fast8_t AOcontrolLoop_perfTest_LinearSimulator_FPCONF(
    const char *fpsname,
    uint32_t CMDmode
)
{
	uint16_t loopstatus;
	
	// SETUP FPS
    FUNCTION_PARAMETER_STRUCT fps = function_parameter_FPCONFsetup(fpsname, CMDmode, &loopstatus);
    if( loopstatus == 0 ) // stop fps
        return 0;    

	// ALLOCATE ENTRIES
	void * pNull = NULL;
    int fpi;  // function parameter index
    fpi = function_parameter_add_entry(&fps, ".DMxsize", "Deformable mirror X size", FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, pNull);
    fpi = function_parameter_add_entry(&fps, ".DMysize", "Deformable mirror Y size", FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, pNull);

	// RUN UPDATE LOOP
	while( loopstatus == 1 )
	{
		if( function_parameter_FPCONFloopstep(&fps, CMDmode, &loopstatus) == 1)
		{
			// here goes the logic
			functionparameter_CheckParametersAll(&fps);  // check all parameter values
		}		
	}
	function_parameter_FPCONFexit( &fps );


    return(0);
}








int_fast8_t AOcontrolLoop_perfTest_LinearSimulator_RUN(
	const char *fpsname
)
{
	FUNCTION_PARAMETER_STRUCT fps;
	int FPSINTERFACE = 1;
	
	if(function_parameter_struct_connect(fpsname, &fps) == -1)
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







