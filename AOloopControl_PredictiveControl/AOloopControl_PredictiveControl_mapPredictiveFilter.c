/**
 * @file    AOloopControl_PredictiveControl_mapPredictiveFilter.c
 * @brief   Adaptive Optics Control loop engine Predictive Control
 *
 *
 *
 *
 *
 */

#define _GNU_SOURCE

#include <stdio.h>

#ifdef MILK_NO_CLI
#include "CLIcore_standalone.h"
#else
#include "CLIcore.h"
#endif

#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"

//
// IDcoeff_name is AO telemetry file
// size:   #modes, 1, #samples
//
errno_t AOloopControl_PredictiveControl_mapPredictiveFilter(
    const char *IDmodecoeff_name, long modeout, double delayfr)
{
    imageID IDmodecoeff;
    long    NBsamples;
    long    NBmodes;
    double  SVDeps = 0.001;

    long IDtrace;

    long modesize = 15;
    long modeoffset;
    long modeouto;
    long filtsize = 20;

    long ii, m;

    modeoffset = modeout - (long)(modesize / 2);
    modeouto   = modeout - modeoffset;

    IDmodecoeff = image_ID(IDmodecoeff_name,
        dcimg, dcnimg);
    NBmodes     = dcimg[IDmodecoeff].md[0].size[0];
    NBsamples   = dcimg[IDmodecoeff].md[0].size[2];

    // reformat measurements
    create_2Dimage_ID("trace", NBsamples, modesize, &IDtrace);

    for(ii = 0; ii < NBsamples; ii++)
        for(m = 0; m < modesize; m++)
            dcimg[IDtrace].array.F[m * NBsamples + ii] =
                dcimg[IDmodecoeff].array.F[ii * NBmodes + m];

    AOloopControl_PredictiveControl_testPredictiveFilter("trace",
            modeouto,
            delayfr,
            filtsize,
            "filt",
            SVDeps);
    delete_image_ID("filt", DELETE_IMAGE_ERRMODE_WARNING);

    return RETURN_SUCCESS;
}
