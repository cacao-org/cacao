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
	libinit_image_basic();
	libinit_image_format();
	libinit_psf();
	libinit_img_reduce();
	libinit_linARfilterPred();
	libinit_ZernikePolyn();
	
	// initialize modules specific to cacao
	libinit_AOloopControl();
	libinit_AOloopControl_PredictiveControl();
	libinit_linARfilterPred();
	libinit_AOloopControl_computeCalib();
	libinit_FPAOloopControl();
	
	runCLI(argc, argv, AppName);

	return 0;
}
