#include <stdio.h>
#include <stdlib.h>
#include <CommandLineInterface/CLIcore.h>



#include <image_basic/image_basic.h>
#include <image_format/image_format.h>
#include <img_reduce/img_reduce.h>
#include <psf/psf.h>
#include <ZernikePolyn/ZernikePolyn.h>


// cacao includes for inits
#include <AOloopControl/AOloopControl.h>
#include <AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h>
#include <linARfilterPred/linARfilterPred.h>
#include <AOloopControl_computeCalib/AOloopControl_computeCalib.h>
#include <FPAOloopControl/FPAOloopControl.h>


#define STYLE_BOLD    "\033[1m"
#define STYLE_NO_BOLD "\033[22m"



int main(int argc, char *argv[])
{
	char *AppName = "cacao";

	printf(STYLE_BOLD);
	printf("\n        Compute And Control for Adaptive Optics (cacao)\n");
	printf(STYLE_NO_BOLD);


	// initialize milk modules for which no function calls is included by default
	init_image_basic();
	init_image_format();
	init_psf();
	init_img_reduce();
	init_linARfilterPred();
	init_ZernikePolyn();
	
	// initialize modules specific to cacao
	init_AOloopControl();
	init_AOloopControl_PredictiveControl();
	init_linARfilterPred();
	init_AOloopControl_computeCalib();
	init_FPAOloopControl();
	
	runCLI(argc, argv, AppName);

	return 0;
}
