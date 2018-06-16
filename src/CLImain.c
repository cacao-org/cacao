#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <CommandLineInterface/CLIcore.h>



#include <image_basic/image_basic.h>
#include <image_format/image_format.h>
#include <img_reduce/img_reduce.h>
#include <psf/psf.h>
#include <ZernikePolyn/ZernikePolyn.h>

#include <linopt_imtools/linopt_imtools.h>
#include <cudacomp/cudacomp.h>
#include <linARfilterPred/linARfilterPred.h>

// cacao includes for inits
#include <AOloopControl/AOloopControl.h>
#include <AOloopControl_DM/AOloopControl_DM.h>
#include <AOloopControl_acquireCalib/AOloopControl_acquireCalib.h>
#include <AOloopControl_compTools/AOloopControl_compTools.h>
#include <AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h>
#include <linARfilterPred/linARfilterPred.h>
#include <AOloopControl_computeCalib/AOloopControl_computeCalib.h>
#include <FPAOloopControl/FPAOloopControl.h>



#define STYLE_BOLD    "\033[1m"
#define STYLE_NO_BOLD "\033[22m"


DATA data;


int main(int argc, char *argv[])
{
	char *AppName = "cacao";

	printf(STYLE_BOLD);
	printf("\n        Compute And Control for Adaptive Optics (cacao)\n");
	printf(STYLE_NO_BOLD);

	
	strcpy(data.package_name, PACKAGE_NAME);
	strcpy(data.package_version, PACKAGE_VERSION);
	strcpy(data.sourcedir, SOURCEDIR);
	strcpy(data.configdir, CONFIGDIR);



	printf("\n");
	printf("        %s version %s\n", data.package_name, data.package_version);
	printf("        GNU General Public License v3.0\n");
	printf("        Report bugs to : %s\n", PACKAGE_BUGREPORT);
    printf("        Type \"help\" for instructions\n");
	printf("        \n");



	// initialize milk modules for which no function calls is included by default
	libinit_image_basic();
	libinit_image_format();
	libinit_psf();
	libinit_img_reduce();
	libinit_linARfilterPred();
	libinit_ZernikePolyn();
	libinit_linopt_imtools();
	libinit_cudacomp();
	libinit_linARfilterPred();

	
	// initialize modules specific to cacao
	libinit_AOloopControl();
	libinit_AOloopControl_PredictiveControl();
	libinit_linARfilterPred();
	libinit_AOloopControl_computeCalib();
	libinit_FPAOloopControl();
	libinit_AOloopControl_DM();
	libinit_AOloopControl_compTools();
	libinit_AOloopControl_acquireCalib();

	
	printf("Starting CLI ...\n");
	fflush(stdout);
	
	runCLI(argc, argv, AppName);

	return 0;
}
