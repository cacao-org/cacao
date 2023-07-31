/**
 * @file    AOloopControl_perfTest.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine
 * performance Tests
 *
 *
 */

#ifndef _AOLOOPCONTROL_PERFTEST_H
#define _AOLOOPCONTROL_PERFTEST_H


errno_t AOloopControl_perfTest_mkSyncStreamFiles2(char  *datadir,
        char  *stream0,
        char  *stream1,
        double tstart,
        double tend,
        double dt,
        double dtlag);

errno_t AOloopControl_perfTest_ComputeSimilarityMatrix(char *IDname,
        char *IDname_out);

int AOloopControl_perfTest_StatAnalysis_2streams(char         *IDname_stream0,
        char         *IDname_stream1,
        char         *IDname_simM0,
        char         *IDname_simM1,
        long          dtmin,
        unsigned long NBselected);

errno_t AOloopControl_perfTest_SelectWFSframes_from_PSFframes(char *IDnameWFS,
        char *IDnamePSF,
        float frac,
        long  x0,
        long  x1,
        long  y0,
        long  y1,
        int   EvalMode,
        float alpha);

#endif
