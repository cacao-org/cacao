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







/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. STATUS / TESTING / PERF MEASUREMENT
 *  Measure loop behavior */
/* =============================================================================================== */
/* =============================================================================================== */

errno_t AOloopControl_perfTest_printloopstatus(long loop, long nbcol, long IDmodeval_dm, long IDmodeval, long IDmodevalave, long IDmodevalrms, long ksize);

errno_t AOloopControl_perfTest_loopMonitor(long loop, double frequ, long nbcol);

errno_t AOloopControl_perfTest_statusStats(int updateconf, long NBsample);

errno_t AOloopControl_perfTest_resetRMSperf();

errno_t AOloopControl_perfTest_showparams(long loop);

errno_t AOcontrolLoop_perfTest_TestDMSpeed(const char *dmname, long delayus, long NBpts, float ampl);







imageID AOloopControl_perfTest_TestDMmodeResp(
    const char *DMmodes_name,
    long index,
    float ampl,
    float fmin,
    float fmax,
    float fmultstep,
    float avetime,
    long dtus,
    const char *DMmask_name,
    const char *DMstream_in_name,
    const char *DMstream_out_name,
    const char *IDout_name
);


imageID AOloopControl_perfTest_TestDMmodes_Recovery(
    const char *DMmodes_name,
    float ampl,
    const char *DMmask_name,
    const char *DMstream_in_name,
    const char *DMstream_out_name,
    const char *DMstream_meas_name,
    long tlagus,
    long NBave,
    const char *IDout_name,
    const char *IDoutrms_name,
    const char *IDoutmeas_name,
    const char *IDoutmeasrms_name
);




errno_t AOcontrolLoop_perfTest_TestSystemLatency_FPCONF(
    char *fpsname,
    uint32_t CMDmode
);

errno_t AOcontrolLoop_perfTest_TestSystemLatency_RUN(
    char *fpsname
);

errno_t AOcontrolLoop_perfTest_TestSystemLatency(
    const char *dmname,
    char *wfsname,
    float OPDamp,
    long NBiter
) ;


imageID AOloopControl_perfTest_blockstats(
    long loop,
    const char *IDout_name
);


errno_t AOloopControl_perfTest_InjectMode(
    long index,
    float ampl
);

errno_t AOloopControl_perfTest_AnalyzeRM_sensitivity(
    const char *IDdmmodes_name,
    const char *IDdmmask_name,
    const char *IDwfsref_name,
    const char *IDwfsresp_name,
    const char *IDwfsmask_name,
    float amplimitnm,
    float lambdanm, 
    const char *foutname
);


imageID AOloopControl_perfTest_mkTestDynamicModeSeq(
    const char *IDname_out,
    long NBpt,
    long NBmodes,
    long StartMode
);




errno_t AOloopControl_LoopTimer_Analysis(
    char *IDname,
    char *fnametxt,
    char *outfname
);



errno_t AOloopControl_perfTest_mkSyncStreamFiles2(
    char   *datadir,
    char   *stream0,
    char   *stream1,
    double  tstart,
    double  tend,
    double  dt,
    double  dtlag
);

errno_t AOloopControl_perfTest_ComputeSimilarityMatrix(
    char *IDname,
    char *IDname_out
);

int AOloopControl_perfTest_StatAnalysis_2streams(
	char *IDname_stream0,
	char *IDname_stream1,
	char *IDname_simM0,
	char *IDname_simM1,
	long  dtmin,
	unsigned long  NBselected
);


errno_t AOloopControl_perfTest_SelectWFSframes_from_PSFframes(
    char *IDnameWFS,
    char *IDnamePSF,
    float frac,
    long x0,
    long x1,
    long y0,
    long y1,
    int EvalMode,
    float alpha
);


#endif
