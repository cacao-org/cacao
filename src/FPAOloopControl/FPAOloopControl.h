#ifndef _FPAOLOOPCONTROL_H
#define _FPAOLOOPCONTROL_H




typedef struct
{
    char name[80];
	
	// DM stream
    char dmCname[80];
    char dmRMname[80];
	long sizexDM;
	long sizeyDM;
	long sizeDM;
	long long DMupdatecnt;
	
	// Focal plane image stream
	char WFSname[80];
	long sizexWFS;
	long sizeyWFS;
	long sizeWFS;
	long long WFScnt;
	
	// timing info
	float loopfrequ; // Hz
	float hardwlatency; // hardware latency between DM command and WFS response [sec] 
	float hardwlatency_frame; // hardware latency between DM command and WFS response 
	
	
	// ============= RESPONSE CALIBRATION ===================
	float fpim_normFlux; // total focal plane flux in the absence of a coronagraph
	float fpim_Xcent;
	float fpim_Ycent;
	

	// ======= LEVEL 1 CALIBRATION ==============
	// Each actuator influence function has the same amplitude, phase is ramp set accordingly to actuator position
	// to be acquired without coronagraph

} FPAOLOOPCONTROL_CONF;



void __attribute__ ((constructor)) libinit_FPAOloopControl();




errno_t FPAOloopControl_InitializeMemory(
    int mode
);


errno_t FPAOloopControl_loadconfigure(
    long loop,
    int  mode,
    int  level
);


int FPAOloopControl_showparams(long loop);

int FPAOloopControl_set_hardwlatency_frame(float hardwlatency_frame);





// RM Calibration

long FPAO_Measure_WFSrespC(long loop, long delayfr, long delayRM1us, long NBave, long NBexcl, char *IDpokeC_name, char *IDoutC_name, int FPAOinitMode, long NBcycle);


// level 1
// Each actuator influence function has the same amplitude, phase is ramp set accordingly to actuator position
// to be acquired without coronagraph
long FPAOloopControl_MeasureResp_level1(float ampl, long delayfr, long delayRM1us, long NBave, long NBexcl, int FPAOinitMode, long NBiter);

long FPAOloopControl_MakeLinComb_seq(char *IDpC_name, long xsize0, long ysize0, long NBmaster0, long N, char *IDout_name);

#endif
