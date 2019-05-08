/**
 * @file    AOloopControl_perfTest.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine performance Tests
 * 
 * AO engine uses stream data structure
 * 
 * @author  O. Guyon
 * @date    27 Aug 2017
 *
 * @bug No known bugs. 
 * 
 */

#ifndef _AOLOOPCONTROL_PERFTEST_H
#define _AOLOOPCONTROL_PERFTEST_H


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_perfTest();

/** @brief Initialize command line interface. */
int_fast8_t init_AOloopControl_perfTest();





/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. STATUS / TESTING / PERF MEASUREMENT
 *  Measure loop behavior */
/* =============================================================================================== */
/* =============================================================================================== */

int_fast8_t AOloopControl_perfTest_printloopstatus(long loop, long nbcol, long IDmodeval_dm, long IDmodeval, long IDmodevalave, long IDmodevalrms, long ksize);

int_fast8_t AOloopControl_perfTest_loopMonitor(long loop, double frequ, long nbcol);

int_fast8_t AOloopControl_perfTest_statusStats(int updateconf, long NBsample);

int_fast8_t AOloopControl_perfTest_resetRMSperf();

int_fast8_t AOloopControl_perfTest_showparams(long loop);

int_fast8_t AOcontrolLoop_perfTest_TestDMSpeed(const char *dmname, long delayus, long NBpts, float ampl);


errno_t AOcontrolLoop_perfTest_TestSystemLatency_FPCONF(char *fpsname, uint32_t CMDmode);
int_fast8_t AOcontrolLoop_perfTest_TestSystemLatency_RUN(char *fpsname);
int_fast8_t AOcontrolLoop_perfTest_TestSystemLatency(const char *dmname, char *wfsname, float OPDamp, long NBiter);

long AOloopControl_perfTest_blockstats(long loop, const char *IDout_name);

int_fast8_t AOloopControl_perfTest_InjectMode( long index, float ampl );

long AOloopControl_perfTest_TestDMmodeResp(const char *DMmodes_name, long index, float ampl, float fmin, float fmax, float fmultstep, float avetime, long dtus, const char *DMmask_name, const char *DMstream_in_name, const char *DMstream_out_name, const char *IDout_name);

long AOloopControl_perfTest_TestDMmodes_Recovery(const char *DMmodes_name, float ampl, const char *DMmask_name, const char *DMstream_in_name, const char *DMstream_out_name, const char *DMstream_meas_name, long tlagus, long NBave, const char *IDout_name, const char *IDoutrms_name, const char *IDoutmeas_name, const char *IDoutmeasrms_name);

long AOloopControl_perfTest_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes, long StartMode);


int_fast8_t AOloopControl_perfTest_AnalyzeRM_sensitivity(const char *IDdmmodes_name, const char *IDdmmask_name, const char *IDwfsref_name, const char *IDwfsresp_name, const char *IDwfsmask_name, float amplimitnm, float lambdanm, const char *foutname);



long AOloopControl_LoopTimer_Analysis(char *IDname, char *fnametxt, char *outfname);



int AOloopControl_perfTest_mkSyncStreamFiles2(char *datadir, char *stream0, char *stream1, double tstart, double tend, double dt, double dtlag);

int AOloopControl_perfTest_ComputeSimilarityMatrix(char *IDname, char *IDname_out);

int AOloopControl_perfTest_StatAnalysis_2streams(
	char *IDname_stream0,
	char *IDname_stream1,
	char *IDname_simM0,
	char *IDname_simM1,
	long dtmin,
	long NBselected
	);

int AOloopControl_perfTest_SelectWFSframes_from_PSFframes(char *IDnameWFS, char *IDnamePSF, float frac, long x0, long x1, long y0, long y1, int EvalMode, float alpha);

#endif
