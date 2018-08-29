#ifndef _AOLOOPCONTROL_ProcessModeCoefficients_H
#define _AOLOOPCONTROL_ProcessModeCoefficients_H


#define maxNBMB 100			// maximum number of mode blocks
#define MAXNBMODES 10000	// maximum number of control modes


typedef struct
{

    char DMmodesname[80];
     // BLOCKS OF MODES
    uint_fast16_t DMmodesNBblock;             /**< number of mode blocks (read from parameter) */
    uint_fast16_t NBmodes_block[100];         /**< number of modes within each block (computed from files by AOloopControl_loadconfigure) */
    uint_fast16_t modeBlockIndex[MAXNBMODES]; /**< block index to which each mode belongs (computed by AOloopControl_loadconfigure) */
    uint_fast16_t indexmaxMB[maxNBMB]; 
	uint_fast16_t NBDMmodes;



	 // COMPUTED BY OPEN LOOP RETRIEVAL PROCESS
    double           RMSmodes;
    double           RMSmodesCumul;
    uint_fast64_t    RMSmodesCumulcnt;


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
