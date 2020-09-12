#include "cacao_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sched.h>
#include <omp.h>
#include <assert.h>
#include <pthread.h>
#include <CommandLineInterface/CLIcore.h>



#include <image_basic/image_basic.h>
#include <image_format/image_format.h>
#include <psf/psf.h>
#include <img_reduce/img_reduce.h>
#include <linARfilterPred/linARfilterPred.h>
#include <ZernikePolyn/ZernikePolyn.h>
#include <linopt_imtools/linopt_imtools.h>
#include <cudacomp/cudacomp.h>


// cacao includes for inits
#include <AOloopControl/AOloopControl.h>
#include <AOloopControl_DM/AOloopControl_DM.h>
#include <AOloopControl_acquireCalib/AOloopControl_acquireCalib.h>
#include <AOloopControl_compTools/AOloopControl_compTools.h>
#include <AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h>
#include <linARfilterPred/linARfilterPred.h>
#include <AOloopControl_computeCalib/AOloopControl_computeCalib.h>
#include <FPAOloopControl/FPAOloopControl.h>


//#include <WFS_ShackHartmann/WFS_ShackHartmann.h>


#define STYLE_BOLD    "\033[1m"
#define STYLE_NO_BOLD "\033[22m"


DATA __attribute__((used)) data;





int main(
    int argc,
    char *argv[]
)
{
    char *AppName = "cacao";

    if(getenv("MILK_QUIET"))
    {
        data.quiet = 1;
    }
    else
    {
        data.quiet = 0;
    }


    if(data.quiet == 0)
    {
        printf(STYLE_BOLD);
        printf("\n        Compute And Control for Adaptive Optics (cacao)\n");
#ifndef NDEBUG
        printf("        === DEBUG MODE : assert()         enabled ==========\n");
        printf("        === DEBUG MODE : DEBUG_TRACEPOINT enabled ==========\n");
#endif
        printf(STYLE_NO_BOLD);
    }

    strcpy(data.package_name, PACKAGE_NAME);

    char versionstring[200];
    sprintf(versionstring, "%d.%d.%02d%s",
            VERSION_MAJOR, VERSION_MINOR,
            VERSION_PATCH, VERSION_OPTION);
    strcpy(data.package_version, versionstring);
    strcpy(data.sourcedir, SOURCEDIR);
    strcpy(data.configdir, CONFIGDIR);


    if(data.quiet == 0)
    {
        printf("\n");
        printf("        %s version %s\n", data.package_name, data.package_version);
#ifdef IMAGESTRUCT_VERSION
        printf("        Using ImageStreamIO version %s\n", IMAGESTRUCT_VERSION);
#endif
        printf("        GNU General Public License v3.0\n");
        printf("        Report bugs to : %s\n", PACKAGE_BUGREPORT);
        printf("        Type \"help\" for instructions\n");
        printf("        \n");
    }




    // initialize milk modules for which no function calls is included by default


    libinit_image_basic();
    libinit_image_format();
    libinit_psf();
    libinit_img_reduce();
    libinit_linARfilterPred();
    libinit_ZernikePolyn();
    libinit_linopt_imtools();
    libinit_cudacomp();


    // initialize modules specific to cacao

    libinit_AOloopControl();
    libinit_AOloopControl_PredictiveControl();
    libinit_linARfilterPred();
    libinit_AOloopControl_computeCalib();
    //    libinit_FPAOloopControl();
    libinit_AOloopControl_DM();
    libinit_AOloopControl_compTools();
    libinit_AOloopControl_acquireCalib();

    //libinit_WFS_ShackHartmann();


    runCLI(argc, argv, AppName);

    if(data.quiet == 0)
    {
        printf("NORMAL EXIT\n");
    }

    // clean-up calling thread
    //pthread_exit(NULL);

    return RETURN_SUCCESS;
}
