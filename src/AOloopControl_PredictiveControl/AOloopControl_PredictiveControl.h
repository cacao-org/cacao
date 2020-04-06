/**
 * @file    AOloopControl_PredictiveControl.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine predictive control
 * 
 * AO engine uses stream data structure
 * 
 * 
 */

#ifndef _AOLOOPCONTROL_PREDICTIVECONTROL_H
#define _AOLOOPCONTROL_PREDICTIVECONTROL_H




/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_PredictiveControl();



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL
 *  Predictive control using WFS telemetry */
/* =============================================================================================== */
/* =============================================================================================== */

errno_t AOloopControl_PredictiveControl_mapPredictiveFilter(
	const char *IDmodecoeff_name, 
	long        modeout, 
	double      delayfr
);

double AOloopControl_PredictiveControl_testPredictiveFilter(
	const char *IDtrace_name, 
	long        modeout, 
	double      delayfr, 
	long        filtsize, 
	const char *IDfilt_name, 
	double      SVDeps
);

imageID AOloopControl_PredictiveControl_builPFloop_WatchInput(
    long loop,
    long PFblock,
    long PFblockStart,
    long PFblockEnd,
    long NBbuff
);

/** @brief Set predictive filter to simple average of previous measures */
imageID AOloopControl_PredictiveControl_setPFsimpleAve(
	char *IDPF_name, 
	float DecayCoeff
);


#endif
