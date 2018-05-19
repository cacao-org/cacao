/**
 * @file    AOloopControl_PredictiveControl_mapPredictiveFilter.c
 * @brief   Adaptive Optics Control loop engine Predictive Control
 * 
 * 
 *
 * 
 * ## Change log
 * - 20180518	Guyon	File created 
 * 
 *  
 * @author  O. Guyon
 *
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE






#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"










//
// IDcoeff_name is AO telemetry file
// size:   #modes, 1, #samples
//
int_fast8_t AOloopControl_PredictiveControl_mapPredictiveFilter(
	const char *IDmodecoeff_name, 
	long modeout, 
	double delayfr
	)
{
    long IDmodecoeff;
    long NBsamples;
    long NBmodes;
    double SVDeps = 0.001;

    long IDtrace;

    long modesize = 15;
    long modeoffset;
    long modeouto;
    long filtsize = 20;
    double val;

    long ii, jj, m;


    modeoffset = modeout - (long) (modesize/2);
    modeouto = modeout-modeoffset;

    IDmodecoeff = image_ID(IDmodecoeff_name);
    NBmodes = data.image[IDmodecoeff].md[0].size[0];
    NBsamples = data.image[IDmodecoeff].md[0].size[2];


    // reformat measurements
    IDtrace = create_2Dimage_ID("trace", NBsamples, modesize);

    for(ii=0; ii<NBsamples; ii++)
        for(m=0; m<modesize; m++)
            data.image[IDtrace].array.F[m*NBsamples+ii] = data.image[IDmodecoeff].array.F[ii*NBmodes+m];


    val = AOloopControl_PredictiveControl_testPredictiveFilter("trace", modeouto, delayfr, filtsize, "filt", SVDeps);
    delete_image_ID("filt");

    return(0);
}









