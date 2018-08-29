#ifndef _AOLOOPCONTROL_ProcessModeCoefficients_H
#define _AOLOOPCONTROL_ProcessModeCoefficients_H



typedef struct
{
	 // COMPUTED BY OPEN LOOP RETRIEVAL PROCESS
    double RMSmodes;
    double RMSmodesCumul;
    uint_fast64_t RMSmodesCumulcnt;



	// block statistics (instantaneous)
	double block_PFresrms[100]; // Prediction residual, meas RMS
	double block_OLrms[100]; // open loop RMS
	double block_Crms[100]; // correction RMS
	double block_WFSrms[100]; // WFS residual RMS
	double block_WFSnoise[100]; // WFS measurement noise
	double block_limFrac[100]; // fraction of mode coefficients exceeding limit
	
	double ALL_OLrms; // open loop RMS
	double ALL_Crms; // correction RMS
	double ALL_WFSrms; // WFS residual RMS
	double ALL_WFSnoise; // WFS noise
	double ALL_limFrac; // fraction of mode coefficients exceeding limit
	
	
	// averaged
	uint_fast32_t AveStats_NBpt; // averaging interval
	double blockave_PFresrms[100]; // open loop RMS
	double blockave_OLrms[100]; // open loop RMS
	double blockave_Crms[100]; // correction RMS
	double blockave_WFSrms[100]; // WFS residual RMS
	double blockave_WFSnoise[100]; // WFS noise
	double blockave_limFrac[100]; // fraction of mode coefficients exceeding limit

	double ALLave_OLrms; // open loop RMS
	double ALLave_Crms; // correction RMS
	double ALLave_WFSrms; // WFS residual RMS
	double ALLave_WFSnoise; // WFS noise
	double ALLave_limFrac; // fraction of mode coefficients exceeding limit
	
} AOLOOPCONF_ProcessModeCoefficients;


#endif
