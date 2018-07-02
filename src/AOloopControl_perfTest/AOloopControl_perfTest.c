/**
 * @file    AOloopControl_perfTest.c
 * @brief   Adaptive Optics Control loop engine testing
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    27 Aug 2017
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <dirent.h> 
#include <stdio.h> 
#include <string.h> /* strrchr */


#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif

#define MaxNBdatFiles 100000

/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */

//extern long aoloopcontrol_var.aoconfID_wfsim;              // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_dmC;                // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_dmRM;               // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_DMmodes;            // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_gainb;              // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_limitb;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_multfb;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_DMmode_GAIN;        // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_LIMIT_modes;        // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_MULTF_modes;        // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_cmd_modes;          // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_meas_modes;         // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_RMS_modes;          // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_AVE_modes;          // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_modeARPFgainAuto;   // declared in AOloopControl.c
     

static int wcol, wrow; // window size

// TIMING
static struct timespec tnow;
static struct timespec tdiff;


typedef struct {
		char   name[500];
		double tstart;
		double tend;
		long cnt;
	} StreamDataFile;  



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

static int INITSTATUS_AOloopControl_perfTest = 0;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c



// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 9. STATUS / TESTING / PERF MEASUREMENT                                   */
/* =============================================================================================== */
/* =============================================================================================== */

//long AOloopControl_perfTesT_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes, long StartMode);

/** @brief CLI function for AOcontrolLoop_TestDMSpeed */
int_fast8_t AOcontrolLoop_perfTest_TestDMSpeed_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)==0) {
        AOcontrolLoop_perfTest_TestDMSpeed( data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOcontrolLoop_TestSystemLatency */
int_fast8_t AOcontrolLoop_perfTest_TestSystemLatency_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,1)+CLI_checkarg(4,2)==0) {
        AOcontrolLoop_perfTest_TestSystemLatency(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_TestDMmodeResp */
int_fast8_t AOloopControl_perfTest_TestDMmodeResp_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,1)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,2)+CLI_checkarg(9,4)+CLI_checkarg(10,4)+CLI_checkarg(11,4)+CLI_checkarg(12,3)==0) {
        AOloopControl_perfTest_TestDMmodeResp(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numl, data.cmdargtoken[9].val.string, data.cmdargtoken[10].val.string, data.cmdargtoken[11].val.string, data.cmdargtoken[12].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_TestDMmodes_Recovery */
int_fast8_t AOloopControl_perfTest_TestDMmodes_Recovery_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(5,4)+CLI_checkarg(6,4)+CLI_checkarg(7,1)+CLI_checkarg(8,2)+CLI_checkarg(9,3)+CLI_checkarg(10,3)+CLI_checkarg(11,3)+CLI_checkarg(12,3)==0) {
        AOloopControl_perfTest_TestDMmodes_Recovery(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.string, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numl, data.cmdargtoken[9].val.string, data.cmdargtoken[10].val.string, data.cmdargtoken[11].val.string, data.cmdargtoken[12].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_blockstats */
int_fast8_t AOloopControl_perfTest_blockstats_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,5)==0) {
        AOloopControl_perfTest_blockstats(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_InjectMode */
int_fast8_t AOloopControl_perfTest_InjectMode_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0)    {
        AOloopControl_perfTest_InjectMode(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else    return 1;
}

/** @brief CLI function for AOloopControl_loopMonitor */
int_fast8_t AOloopControl_perfTest_loopMonitor_cli() {
    if(CLI_checkarg(1,1)+CLI_checkarg(2,2)==0) {
        AOloopControl_perfTest_loopMonitor(LOOPNUMBER, data.cmdargtoken[1].val.numf, data.cmdargtoken[2].val.numl);
        return 0;
    } else {
        AOloopControl_perfTest_loopMonitor(LOOPNUMBER, 1.0, 8);
        return 0;
    }
}

/** @brief CLI function for AOloopControl_statusStats */
int_fast8_t AOloopControl_perfTest_statusStats_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0) {
        AOloopControl_perfTest_statusStats(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkTestDynamicModeSeq */
int_fast8_t AOloopControl_perfTest_mkTestDynamicModeSeq_cli()
{
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0) {
        AOloopControl_perfTest_mkTestDynamicModeSeq(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else  return 1;
}

/** @brief CLI function for AOloopControl_AnalyzeRM_sensitivity */
int_fast8_t AOloopControl_perfTest_AnalyzeRM_sensitivity_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(4,4)+CLI_checkarg(5,4)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,3)==0)    {
        AOloopControl_perfTest_AnalyzeRM_sensitivity(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.string);
        return 0;
    }
    else        return 1;
}



int_fast8_t AOloopControl_LoopTimer_Analysis_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,5)+CLI_checkarg(3,5)==0)    
    {	
		AOloopControl_LoopTimer_Analysis(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string);
		return 0;
	}
	else
		return 1;
}


int_fast8_t AOloopControl_perfTest_mkSyncStreamFiles2_cli()
{
	if(CLI_checkarg(1,5)+CLI_checkarg(2,5)+CLI_checkarg(3,5)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)==0)    
    {
		AOloopControl_perfTest_mkSyncStreamFiles2(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf);
		return 0;
	}
	else
		return 1;
}








/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl functions */







/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. INITIALIZATION, configurations                                        */
/* =============================================================================================== */
/* =============================================================================================== */

void __attribute__ ((constructor)) libinit_AOloopControl_perfTest()
{
	if ( INITSTATUS_AOloopControl_perfTest == 0 )
	{
		init_AOloopControl_perfTest();
		RegisterModule(__FILE__, "cacao", "AO loop control performance monitoring and testing");
		INITSTATUS_AOloopControl_perfTest = 1;
	}
}


int_fast8_t init_AOloopControl_perfTest()
{
    FILE *fp;

/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 9. STATUS / TESTING / PERF MEASUREMENT                                   */
/* =============================================================================================== */
/* =============================================================================================== */

    RegisterCLIcommand("aoldmtestsp", __FILE__, AOcontrolLoop_perfTest_TestDMSpeed_cli, "test DM speed by sending circular tip-tilt", "<dmname> <delay us [long]> <NB pts> <ampl>", "aoldmtestsp dmdisp2 100 20 0.1", "long AOcontrolLoop_perfTest_TestDMSpeed(char *dmname, long delayus, long NBpts, float ampl)");

    RegisterCLIcommand("aoltestlat", __FILE__, AOcontrolLoop_perfTest_TestSystemLatency_cli, "test system latency", "<dm stream> <wfs stream> <ampl [um]> <NBiter>", "aoltestlat dmC wfsim 0.1 5000", "long AOcontrolLoop_perfTest_TestSystemLatency(char *dmname, char *wfsname, float OPDamp, long NBiter)");

    RegisterCLIcommand("aoltestmresp", __FILE__, AOloopControl_perfTest_TestDMmodeResp_cli, "Measure system response for a single mode", "<DM modes [3D im]> <mode #> <ampl [um]> <fmin [Hz]> <fmax [Hz]> <fstep> <meas. time [sec]> <time step [us]> <DM mask> <DM in [2D stream]> <DM out [2D stream]>  <output [2D im]>", "aoltestmresp DMmodesC 5 0.05 10.0 100.0 1.2 1.0 1000 dmmask dmdisp3 dmC out", "long AOloopControl_perfTest_TestDMmodeResp(char *DMmodes_name, long index, float ampl, float fmin, float fmax, float fmultstep, float avetime, long dtus, char *DMmask_name, char *DMstream_in_name, char *DMstream_out_name, char *IDout_name)");

    RegisterCLIcommand("aoltestdmrec", __FILE__, AOloopControl_perfTest_TestDMmodes_Recovery_cli, "Test system DM modes recovery", "<DM modes [3D im]> <ampl [um]> <DM mask [2D im]> <DM in [2D stream]> <DM out [2D stream]> <meas out [2D stream]> <lag time [us]>  <NB averages [long]>  <out ave [2D im]> <out rms [2D im]> <out meas ave [2D im]> <out meas rms [2D im]>", "aoltestdmrec DMmodesC 0.05 DMmask dmsisp2 dmoutr 2000  20 outave outrms outmave outmrms", "long AOloopControl_perfTest_TestDMmodes_Recovery(char *DMmodes_name, float ampl, char *DMmask_name, char *DMstream_in_name, char *DMstream_out_name, char *DMstream_meas_name, long tlagus, long NBave, char *IDout_name, char *IDoutrms_name, char *IDoutmeas_name, char *IDoutmeasrms_name)");

    RegisterCLIcommand("aolresetrms", __FILE__, AOloopControl_perfTest_resetRMSperf, "reset RMS performance monitor", "no arg", "aolresetrms", "int AOloopControl_perfTest_resetRMSperf()");

    RegisterCLIcommand("aolinjectmode",__FILE__, AOloopControl_perfTest_InjectMode_cli, "inject single mode error into RM channel", "<index> <ampl>", "aolinjectmode 20 0.1", "int AOloopControl_perfTest_InjectMode()");

    RegisterCLIcommand("aolstatusstats", __FILE__, AOloopControl_perfTest_statusStats_cli, "measures distribution of status values", "<update flag [int]> <NBsample [long]>", "aolstatusstats 0 100000", "int AOloopControl_perfTest_statusStats(int updateconf, long NBsample)");

    RegisterCLIcommand("aolmon", __FILE__, AOloopControl_perfTest_loopMonitor_cli, "monitor loop", "<frequ> <Nbcols>", "aolmon 10.0 3", "int AOloopControl_perfTest_loopMonitor(long loop, double frequ)");

    RegisterCLIcommand("aolblockstats", __FILE__, AOloopControl_perfTest_blockstats_cli, "measures mode stats per block", "<loopnb> <outim>", "aolblockstats 2 outstats", "long AOloopControl_perfTest_blockstats(long loop, const char *IDout_name)");

    RegisterCLIcommand("aolmktestmseq", __FILE__, AOloopControl_perfTest_mkTestDynamicModeSeq_cli, "make modal periodic test sequence", "<outname> <number of slices> <number of modes> <firstmode>", "aolmktestmseq outmc 100 50 0", "long AOloopControl_perfTest_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes, long StartMode)");

    RegisterCLIcommand("aolzrmsens", __FILE__, AOloopControl_perfTest_AnalyzeRM_sensitivity_cli, "Measure zonal RM sensitivity", "<DMmodes> <DMmask> <WFSref> <WFSresp> <WFSmask> <amplitude[nm]> <lambda[nm]> <outname>", "aolzrmsens DMmodes dmmask wfsref0 zrespmat wfsmask 0.1 outfile.txt", "long AOloopControl_perfTest_AnalyzeRM_sensitivity(const char *IDdmmodes_name, const char *IDdmmask_name, const char *IDwfsref_name, const char *IDwfsresp_name, const char *IDwfsmask_name, float amplimitnm, float lambdanm, const char *foutname)");

	RegisterCLIcommand("aoltimingstat", __FILE__, AOloopControl_LoopTimer_Analysis_cli, "Analysis of loop timing data", "<TimingImage> <TimingTXTfile> <outFile>", "aoltimingstat aol0_looptiming timing.txt outfile.txt", "long AOloopControl_LoopTimer_Analysis(char *IDname, char *fnametxt, char *outfname)");


	RegisterCLIcommand("aolptmksyncs2",
		__FILE__,
		AOloopControl_perfTest_mkSyncStreamFiles2_cli,
		"synchronize two streams from disk telemetry",
		"<datadir> <stream0name> <stream1name> <tstart> <tend> <dt> <dtlag>",
		"aolptmksyncs2 \"/media/data/20180701/\" aol2_wfsim aol3_wfsim 1530410732.0 1530410733.0 0.001 0.00001",
		"int AOloopControl_perfTest_mkSyncStreamFiles2(char *datadir, char *stream0, char *stream1, double tstart, double tend, double dt, double dtlag)");
}





/**
 *  ## Purpose
 * 
 * Measure hardware latency between DM and WFS streams
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * dmname	char*
 * 			DM actuation stream to which function sends pokes
 * 
 * @param[in]
 * wfsname	char*
 * -		WFS image stream
 * 
 * @param[in]
 * OPDamp	FLOAT
 * 			Poke amplitude \[um\]
 * 
 * @param[in]
 * NBiter	LONG
 * 			Number of poke cycles
 * 
 */
int_fast8_t AOcontrolLoop_perfTest_TestSystemLatency(const char *dmname, char *wfsname, float OPDamp, long NBiter)
{
    long IDdm;
    long dmxsize, dmysize;
    long IDwfs;
    long wfsxsize, wfsysize, wfssize;
//    long twait0us = 100000;

    double tdouble_start;
    double tdouble_end;
    long wfscntstart;
    long wfscntend;

    struct timespec tstart;
//    struct timespec tnow;
    struct timespec *tarray;
    double tdouble;
    double dtmax = 1.0;  // Max running time per iteration
    double dt, dt1;
    double *dtarray;
    double a, b;
    char command[200];
    long IDdm0, IDdm1; // DM shapes
    long ii, jj;
    float x, y;

    long IDwfsc;
    long wfs_NBframesmax = 20;
    long wfsframe;
    long twaitus = 30000; // initial wait [us]
    double dtoffset0 = 0.002; // 2 ms
    long wfsframeoffset = 10;

    long IDwfsref;

    char *ptr;
    long kk, kkmax;
    double *valarray;
    double tmp;
    double dtoffset;
    long kkoffset;

    long iter;

    float *latencyarray;
    float *latencysteparray;
    float latencyave, latencystepave;

    FILE *fp;
    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;
    float minlatency, maxlatency;
    double wfsdt;

    uint8_t atype;
    uint32_t naxes[3];


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
	int r;
	
    r = seteuid(data.euid); //This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid);//Go back to normal privileges
#endif

    latencyarray = (float*) malloc(sizeof(float)*NBiter);
    latencysteparray = (float*) malloc(sizeof(float)*NBiter);

    IDdm = image_ID(dmname);
    dmxsize = data.image[IDdm].md[0].size[0];
    dmysize = data.image[IDdm].md[0].size[1];

    IDdm0 = create_2Dimage_ID("_testdm0", dmxsize, dmysize);
    IDdm1 = create_2Dimage_ID("_testdm1", dmxsize, dmysize);
    for(ii=0; ii<dmxsize; ii++)
        for(jj=0; jj<dmysize; jj++)
        {
            x = (2.0*ii-1.0*dmxsize)/dmxsize;
            y = (2.0*jj-1.0*dmxsize)/dmysize;
            data.image[IDdm0].array.F[jj*dmxsize+ii] = 0.0;
            data.image[IDdm1].array.F[jj*dmxsize+ii] = OPDamp*(sin(8.0*x)+sin(8.0*y));
        }

    //system("mkdir -p tmp");
    //save_fits("_testdm0", "!tmp/_testdm0.fits");
    //save_fits("_testdm1", "!tmp/_testdm1.fits");

    IDwfs = image_ID(wfsname);
    wfsxsize = data.image[IDwfs].md[0].size[0];
    wfsysize = data.image[IDwfs].md[0].size[1];
    wfssize = wfsxsize*wfsysize;
    atype = data.image[IDwfs].md[0].atype;

    naxes[0] = wfsxsize;
    naxes[1] = wfsysize;
    naxes[2] = wfs_NBframesmax;
    IDwfsc = create_image_ID("_testwfsc", 3, naxes, atype, 0, 0);


    // coarse estimage of frame rate
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdouble_start = 1.0*tnow.tv_sec + 1.0e-9*tnow.tv_nsec;
    wfscntstart = data.image[IDwfs].md[0].cnt0;
    sleep(5.0);
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdouble_end = 1.0*tnow.tv_sec + 1.0e-9*tnow.tv_nsec;
    wfscntend = data.image[IDwfs].md[0].cnt0;
    wfsdt = (tdouble_end - tdouble_start)/(wfscntend-wfscntstart);

    printf("wfs dt = %f sec\n", wfsdt);


    // update times
    dtmax = wfsdt*wfs_NBframesmax*1.2 + 0.5;
    twaitus = 1000000.0*wfsdt;
    dtoffset0 = 1.5*wfsdt;


    tarray = (struct timespec *) malloc(sizeof(struct timespec)*wfs_NBframesmax);
    dtarray = (double*) malloc(sizeof(double)*wfs_NBframesmax);

    if(system("mkdir -p timingstats") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    if ((fp=fopen("timingstats/hardwlatency.txt", "w"))==NULL)
    {
        printf("ERROR: cannot create file \"timingstats/hardwlatency.txt\"\\n");
        exit(0);
    }

    clock_gettime(CLOCK_REALTIME, &tnow);
    tdouble_start = 1.0*tnow.tv_sec + 1.0e-9*tnow.tv_nsec;
    wfscntstart = data.image[IDwfs].md[0].cnt0;
    wfsframeoffset = (long) (0.3*wfs_NBframesmax);


	printf("WFS size : %ld %ld\n", wfsxsize, wfsysize);
	if(atype == _DATATYPE_FLOAT)
		printf("data type  :  _DATATYPE_FLOAT\n");
	if(atype == _DATATYPE_UINT16)
		printf("data type  :  _DATATYPE_UINT16\n");
	if(atype == _DATATYPE_INT16)
		printf("data type  :  _DATATYPE_INT16\n");

	list_image_ID();

    for(iter=0; iter<NBiter; iter++)
    {
		//double tlastdouble;
		double tstartdouble;
		long NBwfsframe;
	    unsigned long wfscnt0;
        double latencymax = 0.0;
	    double latency;
		
        printf(" - ITERATION %5ld / %5ld\n", iter, NBiter);
        fflush(stdout);

		for(ii=0;ii<10;ii++)
			printf("  %5ld  ->  %f\n", ii, (float) data.image[IDwfs].array.SI16[ii]);


        printf("write to %s\n", dmname);
        fflush(stdout);
        copy_image_ID("_testdm0", dmname, 1);

        unsigned int dmstate = 0;

        // waiting time
        usleep(twaitus);


        // and waiting frames
        wfscnt0 = data.image[IDwfs].md[0].cnt0;
        for(wfsframe=0; wfsframe<wfs_NBframesmax; wfsframe++)
        {
            while(wfscnt0==data.image[IDwfs].md[0].cnt0)
            {
                usleep(50);
            }
            wfscnt0 = data.image[IDwfs].md[0].cnt0;
        }

        dt = 0.0;
        clock_gettime(CLOCK_REALTIME, &tstart);
        tstartdouble = 1.0*tstart.tv_sec + 1.0e-9*tstart.tv_nsec;
    //    tlastdouble = tstartdouble;



        wfsframe = 0;
        wfscnt0 = data.image[IDwfs].md[0].cnt0;
        printf("\n");
        while( (dt < dtmax) && (wfsframe<wfs_NBframesmax) )
        {
            // WAITING for image
            while(wfscnt0==data.image[IDwfs].md[0].cnt0)
            {
                usleep(10);
            }
            wfscnt0 = data.image[IDwfs].md[0].cnt0;
            printf("[%8ld / %8ld]  %f  %f\n", wfsframe, wfs_NBframesmax, dt, dtmax);
            fflush(stdout);

            if(atype == _DATATYPE_FLOAT)
            {
                // copy image to cube slice
                ptr = (char*) data.image[IDwfsc].array.F;
                ptr += sizeof(float)*wfsframe*wfssize;
                memcpy(ptr, data.image[IDwfs].array.F, sizeof(float)*wfssize);
            }

            if(atype == _DATATYPE_UINT16)
            {
                // copy image to cube slice
                ptr = (char*) data.image[IDwfsc].array.UI16;
                ptr += sizeof(short)*wfsframe*wfssize;
                memcpy(ptr, data.image[IDwfs].array.UI16, sizeof(short)*wfssize);
            }
            
            if(atype == _DATATYPE_INT16)
            {
                // copy image to cube slice
                ptr = (char*) data.image[IDwfsc].array.SI16;
                ptr += sizeof(short)*wfsframe*wfssize;
                memcpy(ptr, data.image[IDwfs].array.SI16, sizeof(short)*wfssize);
            }

            clock_gettime(CLOCK_REALTIME, &tarray[wfsframe]);

            tdouble = 1.0*tarray[wfsframe].tv_sec + 1.0e-9*tarray[wfsframe].tv_nsec;
            dt = tdouble - tstartdouble;
            //  dt1 = tdouble - tlastdouble;
            dtarray[wfsframe] = dt;
       //     tlastdouble = tdouble;

            // apply DM pattern #1
            if((dmstate==0)&&(dt>dtoffset0)&&(wfsframe>wfsframeoffset))
            {
                usleep((long) (ran1()*1000000.0*wfsdt));
                printf("\nDM STATE CHANGED ON ITERATION %ld\n\n", wfsframe);
                kkoffset = wfsframe;
                dmstate = 1;
                copy_image_ID("_testdm1", dmname, 1);

                clock_gettime(CLOCK_REALTIME, &tnow);
                tdouble = 1.0*tnow.tv_sec + 1.0e-9*tnow.tv_nsec;
                dt = tdouble - tstartdouble;
                dtoffset = dt; // time at which DM command is sent
            }
            wfsframe++;
        }
        printf("\n\n %ld frames recorded\n", wfsframe);
        fflush(stdout);
        copy_image_ID("_testdm0", dmname, 1);
        dmstate = 0;


        // Computing difference between consecutive images
        NBwfsframe = wfsframe;
        

        valarray = (double*) malloc(sizeof(double)*NBwfsframe);
        double valmax = 0.0;
        double valmaxdt = 0.0;
        for(kk=1; kk<NBwfsframe; kk++)
        {
            valarray[kk] = 0.0;
            
            if(atype == _DATATYPE_FLOAT)
                for(ii=0; ii<wfssize; ii++)
                {
                    tmp = data.image[IDwfsc].array.F[kk*wfssize+ii] - data.image[IDwfsc].array.F[(kk-1)*wfssize+ii];
                    valarray[kk] += tmp*tmp;
                }
                
            if(atype == _DATATYPE_UINT16) 
                for(ii=0; ii<wfssize; ii++)
                {
                    tmp = data.image[IDwfsc].array.UI16[kk*wfssize+ii] - data.image[IDwfsc].array.UI16[(kk-1)*wfssize+ii];
                    valarray[kk] += 1.0*tmp*tmp;
                }
                
            if(atype == _DATATYPE_INT16) 
                for(ii=0; ii<wfssize; ii++)
                {
					tmp = 0.0;
                    tmp = data.image[IDwfsc].array.SI16[kk*wfssize+ii] - data.image[IDwfsc].array.SI16[(kk-1)*wfssize+ii];
                    valarray[kk] += 1.0*tmp*tmp;
                }
            
            if(valarray[kk]>valmax)
            {
                valmax = valarray[kk];
                valmaxdt = 0.5*(dtarray[kk-1]+dtarray[kk]);
                kkmax = kk-kkoffset;
            }
        }


        //
        //
        //
        for(wfsframe=1; wfsframe<NBwfsframe; wfsframe++)
            fprintf(fp, "%ld   %10.2f     %g\n", wfsframe-kkoffset, 1.0e6*(0.5*(dtarray[wfsframe]+dtarray[wfsframe-1])-dtoffset), valarray[wfsframe]);

        printf("mean interval =  %10.2f ns\n", 1.0e9*(dt-dtoffset)/NBwfsframe);
        fflush(stdout);

        free(valarray);

        latency = valmaxdt-dtoffset;
        // latencystep = kkmax;

        printf("... Hardware latency = %f ms  = %ld frames\n", 1000.0*latency, kkmax);
        if(latency > latencymax)
        {
            latencymax = latency;
            save_fl_fits("_testwfsc", "!./timingstats/maxlatencyseq.fits");
        }

        fprintf(fp, "# %5ld  %8.6f\n", iter, (valmaxdt-dtoffset));

        latencysteparray[iter] = 1.0*kkmax;
        latencyarray[iter] = (valmaxdt-dtoffset);

    
    }
    fclose(fp);

    clock_gettime(CLOCK_REALTIME, &tnow);
    tdouble_end = 1.0*tnow.tv_sec + 1.0e-9*tnow.tv_nsec;
    wfscntend = data.image[IDwfs].md[0].cnt0;



    free(dtarray);
    free(tarray);

    latencyave = 0.0;
    latencystepave = 0.0;
    minlatency = latencyarray[0];
    maxlatency = latencyarray[0];
    for(iter=0; iter<NBiter; iter++)
    {
        if(latencyarray[iter]>maxlatency)
            maxlatency = latencyarray[iter];

        if(latencyarray[iter]<minlatency)
            minlatency = latencyarray[iter];

        latencyave += latencyarray[iter];
        latencystepave += latencysteparray[iter];
    }
    latencyave /= NBiter;
    latencystepave /= NBiter;

	//save__fl_fits("_testwfsc", "!./timingstats/maxlatencyseq.fits");


    quick_sort_float(latencyarray, NBiter);

    printf("AVERAGE LATENCY = %8.3f ms   %f frames\n", latencyave*1000.0, latencystepave);
    printf("min / max over %ld measurements: %8.3f ms / %8.3f ms\n", NBiter, minlatency*1000.0, maxlatency*1000.0);

    if(sprintf(command, "echo %8.6f > conf/param_hardwlatency.txt", latencyarray[NBiter/2]) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(system(command) != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    if(sprintf(command, "echo %f %f %f %f %f > timinstats/hardwlatencyStats.txt", latencyarray[NBiter/2], latencyave, minlatency, maxlatency, latencystepave) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(system(command) != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");


    dt = tdouble_end - tdouble_start;
    printf("FRAME RATE = %.3f Hz\n", 1.0*(wfscntend-wfscntstart)/dt);

    if(sprintf(command, "echo %.3f > conf/param_mloopfrequ.txt", 1.0*(wfscntend-wfscntstart)/dt ) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(system(command) != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    free(latencyarray);
    free(latencysteparray);

    return 0;
}












// waits on semaphore 3

long AOloopControl_perfTest_blockstats(long loop, const char *IDout_name)
{
    long IDout;
    uint32_t *sizeout;
    long NBmodes;
    char fname[200];
    long IDmodeval;
    long m, blk, i;
    long cnt;
    long IDblockRMS, IDblockRMS_ave;
    long NBblock;

    float *rmsarray;
    int *indexarray;

    float alpha = 0.0001;


    if(sprintf(fname, "aol%ld_modeval", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(fname);
    NBmodes = data.image[IDmodeval].md[0].size[0];

    sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizeout[0] = NBmodes;
    sizeout[1] = 1;
    IDout = create_image_ID(IDout_name, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDout_name, 10);

    printf("%ld modes\n", NBmodes);


    m = 0;
    blk = 0;
    while(m<NBmodes)
    {
		long ID;
		long n;
		
        if(sprintf(fname, "aol%ld_DMmodes%02ld", loop, blk) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        ID = read_sharedmem_image(fname);
        n = data.image[ID].md[0].size[2];
        for(i=0; i<n; i++)
        {
            data.image[IDout].array.F[m] = blk;
            m++;
        }
        blk++;
    }
    NBblock = blk;

    rmsarray = (float*) malloc(sizeof(float)*NBblock);


    indexarray = (int*) malloc(sizeof(int)*NBmodes);
    for(m=0; m<NBmodes; m++)
        indexarray[m] = (int) (0.1 + data.image[IDout].array.F[m]);


    printf("NBblock = %ld\n", NBblock);
    sizeout[0] = NBblock;
    sizeout[1] = 1;

    if(sprintf(fname, "aol%ld_blockRMS", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblockRMS = create_image_ID(fname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(fname, 10);

    if(sprintf(fname, "aol%ld_blockRMS_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblockRMS_ave = create_image_ID(fname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(fname, 10);


    cnt =  0;
    for(;;)
    {
        if(data.image[IDmodeval].md[0].sem==0)
        {
            while(cnt==data.image[IDmodeval].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDmodeval].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDmodeval].semptr[3]);

        for(blk=0; blk<NBblock; blk++)
            rmsarray[blk] = 0.0;

        for(m=0; m<NBmodes; m++)
            rmsarray[indexarray[m]] += data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m];

        data.image[IDblockRMS].md[0].write = 1;
        for(blk=0; blk<NBblock; blk++)
            data.image[IDblockRMS].array.F[blk] = rmsarray[blk];
        COREMOD_MEMORY_image_set_sempost_byID(IDblockRMS, -1);
        data.image[IDblockRMS].md[0].cnt0++;
        data.image[IDblockRMS].md[0].write = 0;

        data.image[IDblockRMS_ave].md[0].write = 1;
        for(blk=0; blk<NBblock; blk++)
            data.image[IDblockRMS_ave].array.F[blk] = (1.0-alpha)* data.image[IDblockRMS_ave].array.F[blk] + alpha*rmsarray[blk];
        COREMOD_MEMORY_image_set_sempost_byID(IDblockRMS_ave, -1);
        data.image[IDblockRMS_ave].md[0].cnt0++;
        data.image[IDblockRMS_ave].md[0].write = 0;

    }

    free(sizeout);
    free(rmsarray);
    free(indexarray);


    return(IDout);
}





int_fast8_t AOloopControl_perfTest_InjectMode( long index, float ampl )
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_DMmodes==-1)
    {
		char name[200];
		
        if(sprintf(name, "aol%ld_DMmodes", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_DMmodes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_dmRM==-1)
        aoloopcontrol_var.aoconfID_dmRM = read_sharedmem_image(AOconf[LOOPNUMBER].dmRMname);


    if((index<0)||(index>AOconf[LOOPNUMBER].NBDMmodes-1))
    {
        printf("Invalid mode index... must be between 0 and %ld\n", AOconf[LOOPNUMBER].NBDMmodes);
    }
    else
    {
        float *arrayf;
		long i;
		
        arrayf = (float*) malloc(sizeof(float)*AOconf[LOOPNUMBER].sizeDM);

        for(i=0; i<AOconf[LOOPNUMBER].sizeDM; i++)
            arrayf[i] = ampl*data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F[index*AOconf[LOOPNUMBER].sizeDM+i];



        data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].write = 1;
        memcpy (data.image[aoloopcontrol_var.aoconfID_dmRM].array.F, arrayf, sizeof(float)*AOconf[LOOPNUMBER].sizeDM);
        data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].write = 0;

        free(arrayf);
        AOconf[LOOPNUMBER].DMupdatecnt ++;
    }

    return(0);
}



//
// measure response matrix sensitivity
//
int_fast8_t AOloopControl_perfTest_AnalyzeRM_sensitivity(const char *IDdmmodes_name, const char *IDdmmask_name, const char *IDwfsref_name, const char *IDwfsresp_name, const char *IDwfsmask_name, float amplimitnm, float lambdanm, const char *foutname)
{
    FILE *fp;
    long IDdmmodes;
    long IDdmmask;
    long IDwfsref;
    long IDwfsresp;
    long IDwfsmask;
	double dmmodermscnt;
    long dmxsize, dmysize, dmxysize;
    long NBmodes;
    long wfsxsize, wfsysize, wfsxysize;
    long mode, mode1;

    long ii;


    double wfsmodermscnt;
    double tmp1;

    double wfsreftot, wfsmasktot;
    long IDoutXP, IDoutXP_WFS;
    double XPval;




    printf("amplimit = %f nm\n", amplimitnm);


    IDdmmodes = image_ID(IDdmmodes_name);
    dmxsize = data.image[IDdmmodes].md[0].size[0];
    dmysize = data.image[IDdmmodes].md[0].size[1];
    NBmodes = data.image[IDdmmodes].md[0].size[2];
    dmxysize = dmxsize * dmysize;

    IDdmmask = image_ID(IDdmmask_name);

    IDwfsref = image_ID(IDwfsref_name);
    wfsxsize = data.image[IDwfsref].md[0].size[0];
    wfsysize = data.image[IDwfsref].md[0].size[1];
    wfsxysize = wfsxsize * wfsysize;

    IDwfsresp = image_ID(IDwfsresp_name);
    IDwfsmask = image_ID(IDwfsmask_name);

    wfsreftot = 0.0;
    for(ii=0; ii<wfsxysize; ii++)
        wfsreftot += data.image[IDwfsref].array.F[ii];

    wfsmasktot = 0.0;
    for(ii=0; ii<wfsxysize; ii++)
        wfsmasktot += data.image[IDwfsmask].array.F[ii];


    list_image_ID();
    printf("NBmodes = %ld\n", NBmodes);
    printf("wfs size = %ld %ld\n", wfsxsize, wfsysize);
    printf("wfs resp ID : %ld\n", IDwfsresp);
    printf("wfs mask ID : %ld\n", IDwfsmask);

    printf("wfsmasktot = %f\n", wfsmasktot);

    fp = fopen(foutname, "w");

    fprintf(fp, "# col 1 : mode index\n");
    fprintf(fp, "# col 2 : average value (should be zero)\n");
    fprintf(fp, "# col 3 : DM mode RMS\n");
    fprintf(fp, "# col 4 : WFS mode RMS\n");
    fprintf(fp, "# col 5 : SNR for a 1um DM motion with 1 ph\n");
    fprintf(fp, "# col 6 : fraction of flux used in measurement\n");
    fprintf(fp, "# col 7 : Photon Efficiency\n");
    fprintf(fp, "\n");



    for(mode=0; mode<NBmodes; mode++)
    {
		double dmmoderms;
		double aveval;
		double SNR, SNR1; // single pixel SNR
		float frac = 0.0;
		float pcnt;
		double sigmarad;
		double eff; // efficiency
		double wfsmoderms;
		
		
        dmmoderms = 0.0;
        dmmodermscnt = 0.0;
        aveval = 0.0;
        for(ii=0; ii<dmxysize; ii++)
        {
            tmp1 = data.image[IDdmmodes].array.F[mode*dmxysize+ii]*data.image[IDdmmask].array.F[ii];
            aveval += tmp1;
            dmmoderms += tmp1*tmp1;
            dmmodermscnt += data.image[IDdmmask].array.F[ii];
        }
        dmmoderms = sqrt(dmmoderms/dmmodermscnt);
        aveval /= dmmodermscnt;

        SNR = 0.0;
        wfsmoderms = 0.0;
        wfsmodermscnt = 0.0;
        pcnt = 0.0;
        for(ii=0; ii<wfsxysize; ii++)
        {
            tmp1 = data.image[IDwfsresp].array.F[mode*wfsxysize+ii]*data.image[IDwfsmask].array.F[ii];
            wfsmoderms += tmp1*tmp1;
            wfsmodermscnt = 1.0;
            wfsmodermscnt += data.image[IDwfsmask].array.F[ii];

            if(data.image[IDwfsmask].array.F[ii]>0.1)
                if(data.image[IDwfsref].array.F[ii]>fabs(data.image[IDwfsresp].array.F[mode*wfsxysize+ii]*amplimitnm*0.001))
                {
                    SNR1 = data.image[IDwfsresp].array.F[mode*wfsxysize+ii]/sqrt(data.image[IDwfsref].array.F[ii]);
                    SNR1 /= wfsreftot;
                    SNR += SNR1*SNR1;
                    pcnt += data.image[IDwfsref].array.F[ii];
                }
        }
        frac = pcnt/wfsreftot;

        wfsmoderms = sqrt(wfsmoderms/wfsmodermscnt);
        SNR = sqrt(SNR); // SNR for 1 ph, 1um DM actuation
        // -> sigma for 1ph = 1/SNR [DMum]

        // 1umDM act = 2.0*M_PI * ( 2.0 / (lambdanm*0.001) ) rad WF
        // -> sigma for 1ph = (1/SNR) * 2.0*M_PI * ( 2.0 / (lambdanm*0.001) ) rad WF
        sigmarad = (1.0/SNR) * 2.0*M_PI * ( 2.0 / (lambdanm*0.001) );

        // SNR is in DMum per sqrt(Nph)
        // factor 2.0 for DM reflection

        eff = 1.0/(sigmarad*sigmarad);


        fprintf(fp, "%5ld   %16f   %16f   %16f    %16g      %12g        %12.10f\n", mode, aveval, dmmoderms, wfsmoderms, SNR, frac, eff);
    }

    fclose(fp);


    // computing DM space cross-product
    IDoutXP = create_2Dimage_ID("DMmodesXP", NBmodes, NBmodes);

    for(mode=0; mode<NBmodes; mode++)
        for(mode1=0; mode1<mode+1; mode1++)
        {
            XPval = 0.0;
            for(ii=0; ii<dmxysize; ii++)
                XPval += data.image[IDdmmask].array.F[ii]*data.image[IDdmmodes].array.F[mode*dmxysize+ii]*data.image[IDdmmodes].array.F[mode1*dmxysize+ii];

            data.image[IDoutXP].array.F[mode*NBmodes+mode1] = XPval/dmmodermscnt;
        }
    save_fits("DMmodesXP", "!DMmodesXP.fits");


    // computing WFS space cross-product
    IDoutXP_WFS = create_2Dimage_ID("WFSmodesXP", NBmodes, NBmodes);
    for(mode=0; mode<NBmodes; mode++)
        for(mode1=0; mode1<mode+1; mode1++)
        {
            XPval = 0.0;
            for(ii=0; ii<wfsxysize; ii++)
                XPval += data.image[IDwfsresp].array.F[mode*wfsxysize+ii]*data.image[IDwfsresp].array.F[mode1*wfsxysize+ii];

            data.image[IDoutXP_WFS].array.F[mode*NBmodes+mode1] = XPval/wfsxysize;
        }
    save_fits("WFSmodesXP", "!WFSmodesXP.fits");


    return(0);
}



//
// create dynamic test sequence
//
long AOloopControl_perfTest_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes, long StartMode)
{
    long IDout;
    long xsize, ysize, xysize;
    long ii, kk;
    float ampl0;
    float ampl;
    float pha0;
    char name[200];
    long m, m1;
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_DMmodes==-1)
    {
        if(sprintf(name, "aol%ld_DMmodes", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_DMmodes = read_sharedmem_image(name);
    }
    xsize = data.image[aoloopcontrol_var.aoconfID_DMmodes].md[0].size[0];
    ysize = data.image[aoloopcontrol_var.aoconfID_DMmodes].md[0].size[1];
    xysize = xsize*ysize;

    IDout = create_3Dimage_ID(IDname_out, xsize, ysize, NBpt);

    for(kk=0; kk<NBpt; kk++)
    {
        for(ii=0; ii<xysize; ii++)
            data.image[IDout].array.F[kk*xysize+ii] = 0.0;

        for(m=0; m<NBmodes; m++)
        {
			m1 = m + StartMode;
            ampl0 = 1.0;
            pha0 = M_PI*(1.0*m/NBmodes);
            ampl = ampl0 * sin(2.0*M_PI*(1.0*kk/NBpt)+pha0);
            for(ii=0; ii<xysize; ii++)
                data.image[IDout].array.F[kk*xysize+ii] += ampl * data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F[m1*xysize+ii];
        }        
    }

    return(IDout);
}




//
// analysis of timing data
// Takes two args: 
//  looptiming FITS file
//  looptiming txt file
//

long AOloopControl_LoopTimer_Analysis(char *IDname, char *fnametxt, char *outfname)
{
	long ID;
	int NBtimer;
	long NBsample;
	FILE *fpout;
	FILE *fptxt;
	
	long frameNB;
	uint64_t *cnt0array;
	uint64_t *cnt1array;
	double *frameTimearray;
	long sp;
	long frNB;
	
	int timer;
	double timerval;
	
	
	// analysis
	long missedFrames;
	
	double *timer_ave;
	double *timer_min;
	double *timer_max;
	double *timer_dev;
	
	double rms;
	int ret;
	
	
	ID = image_ID(IDname);
	
	NBtimer = data.image[ID].md[0].size[0];
	NBsample = data.image[ID].md[0].size[2];
	fflush(stdout);
	
	cnt0array = (uint64_t *) malloc(sizeof(uint64_t) * NBsample);
	cnt1array = (uint64_t *) malloc(sizeof(uint64_t) * NBsample);
	frameTimearray = (double *) malloc(sizeof(double) * NBsample);
	
	timer_ave = (double*) malloc(sizeof(double) * NBtimer);
	timer_min = (double*) malloc(sizeof(double) * NBtimer);
	timer_max = (double*) malloc(sizeof(double) * NBtimer);
	timer_dev = (double*) malloc(sizeof(double) * NBtimer);
	
	double f1;
	long l1, l2;
	
	
	
	printf("%d timers\n", NBtimer);
	printf("%ld samples\n", NBsample);
	
	
	
	printf("Creating file \"%s\"\n", outfname);
	fflush(stdout);
	if( (fpout = fopen(outfname, "w")) == NULL)
	{
		printf("ERROR: cannot create file %s\n", outfname);
		exit(0);
	}
	
	printf("Opening file \"%s\"\n", fnametxt);
	fflush(stdout);
	if( (fptxt=fopen(fnametxt, "r")) == NULL)
	{
		printf("ERROR: cannot open file %s\n", fnametxt);
		exit(0);
	}
	
	fprintf(fpout, "# AOloopControl timing\n\n");
	
	list_image_ID();
	
	
	printf("Reading File %s\n\n", fnametxt);
	printf("\n");
	for(sp=0; sp< NBsample; sp++)
	{
		printf("\r     sample %ld / %ld                ", sp, NBsample);
		fflush(stdout);
		
		ret = fscanf(fptxt, "%ld %ld %ld %lf\n", &frNB, &l1, &l2, &f1); 

		cnt0array[sp] = l1;
		cnt1array[sp] = l2;
		frameTimearray[sp] = f1;
		
		fprintf(fpout, "%5ld  %10lu  %10lu  %18.9lf    ", sp, cnt0array[sp], cnt1array[sp], frameTimearray[sp]);
		
		
		if(sp==0)
		{
			for(timer=0; timer<NBtimer; timer++)
			{
				timer_min[timer] = data.image[ID].array.F[sp*NBtimer + timer];
				timer_max[timer] = data.image[ID].array.F[sp*NBtimer + timer];
			}
		}
		
		for(timer=0; timer<NBtimer; timer++)
		{	
			timerval = data.image[ID].array.F[sp*NBtimer + timer];
			fprintf(fpout, "  %12.9f", timerval);
		
			timer_ave[timer] += timerval;
			if(timerval < timer_min[timer])
				timer_min[timer] = timerval;
			if(timerval > timer_max[timer])
				timer_max[timer] = timerval;
		}
		fprintf(fpout, "\n");				
	}
	
	missedFrames = (cnt1array[NBsample-1]-cnt1array[0]) - NBsample;
	
	
	for(timer=0; timer<NBtimer; timer++)
	{
		timer_ave[timer] /= NBsample;
		
		rms = 0.0;
		for(sp=0; sp< NBsample; sp++)
		{
			timerval = data.image[ID].array.F[sp*NBtimer + timer];
			rms += (timerval - timer_ave[timer]) * (timerval - timer_ave[timer]);
		}
		timer_dev[timer] = sqrt(rms/NBsample);
	}
	printf("\n\n");
	
	
	// Print report
	printf("Missed frames   :   %5ld / %10ld  = %.6f\n", missedFrames, NBsample, 100.0*missedFrames/NBsample);
	printf("-------------------------------------------------\n");
	printf("| TIMER |   min   -   ave   -   max   | std dev |\n");
	printf("|  XXX  | xxxx.xx - xxxx.xx - xxxx.xx | xxxx.xx |\n");
	printf("-------------------------------------------------\n");
	for(timer=0; timer<NBtimer; timer++)
		printf("|  %3d  | %7.2f - %7.2f - %7.2f | %7.2f |\n", timer, timer_min[timer]*1e6, timer_ave[timer]*1e6, timer_max[timer]*1e6, timer_dev[timer]*1e6);
	printf("-------------------------------------------------\n");
	
	fclose(fpout);
	
	free(timer_ave);
	free(timer_min);
	free(timer_max);
	free(timer_dev);
	
	free(cnt0array);
	free(cnt1array);
	free(frameTimearray);
	
	return(0);
}






char *remove_ext (char* mystr, char dot, char sep) {
    char *retstr, *lastdot, *lastsep;

    // Error checks and allocate string.

    if (mystr == NULL)
        return NULL;
    if ((retstr = malloc (strlen (mystr) + 1)) == NULL)
        return NULL;

    // Make a copy and find the relevant characters.

    strcpy (retstr, mystr);
    lastdot = strrchr (retstr, dot);
    lastsep = (sep == 0) ? NULL : strrchr (retstr, sep);

    // If it has an extension separator.

    if (lastdot != NULL) {
        // and it's before the extenstion separator.

        if (lastsep != NULL) {
            if (lastsep < lastdot) {
                // then remove it.

                *lastdot = '\0';
            }
        } else {
            // Has extension separator with no path separator.

            *lastdot = '\0';
        }
    }

    // Return the modified string.

    return retstr;
}



//
// WARNING: right=NBelem-1
//
void quicksort_StreamDataFile(StreamDataFile *datfile, long left, long right)
{
    register long i,j;
    StreamDataFile x, y;

    i = left;
    j = right;
    x.tstart = datfile[(left+right)/2].tstart;

    do {
        while(datfile[i].tstart < x.tstart && i<right) i++;
        while(x.tstart < datfile[j].tstart && j>left) j--;

        if(i<=j) {
			y.tstart = datfile[i].tstart;
            y.tend = datfile[i].tend;
            y.cnt = datfile[i].cnt;
            strcpy(y.name, datfile[i].name);
            
            datfile[i].tstart = datfile[j].tstart;
            datfile[i].tend = datfile[j].tend;
            datfile[i].cnt = datfile[j].cnt;
            strcpy(datfile[i].name, datfile[j].name);
            
            datfile[j].tstart = y.tstart;
            datfile[j].tend = y.tend;
            datfile[j].cnt = y.cnt;
            strcpy(datfile[j].name, y.name);
            
            i++;
            j--;
        }
    } while(i<=j);

    if(left<j) quicksort_StreamDataFile(datfile,left,j);
    if(i<right) quicksort_StreamDataFile(datfile,i,right);
}








//
// savedir is, for example /media/data/20180202
//
// dtlag: positive when stream0 is earlier than stream1
//
int AOloopControl_perfTest_mkSyncStreamFiles2(
    char *datadir,
    char *stream0,
    char *stream1,
    double tstart,
    double tend,
    double dt,
    double dtlag
)
{
    DIR *d0;
    struct dirent *dir;
    char datadirstream[500];
    char *ext;
    char *tmpstring;



    StreamDataFile *datfile;
    long NBdatFiles;

    FILE *fp;
    char fname[500];
    long cnt;
    double valf1, valf2;
    long vald1, vald2, vald3, vald4;
    long i;

    uint32_t stream0xsize;
    uint32_t stream0ysize;
    uint32_t zsize;
    double *tstartarray;
    double *tendarray;
    double *exparray;
    long tstep;

    double *intarray_start;
    double *intarray_end;
    double *dtarray;

    long xysize;

	double dtoffset;


    // compute exposure start for each slice of output

    zsize = (tend-tstart)/dt;
    printf("zsize = %ld\n", (long) zsize);
    fflush(stdout);



    // Allocate Working arrays and populate timing arrays

    tstartarray = (double*) malloc(sizeof(double)*zsize);
    tendarray   = (double*) malloc(sizeof(double)*zsize);
    exparray    = (double*) malloc(sizeof(double)*zsize);   // exposure time accumulated, in unit of input frame(s)
    for(tstep=0; tstep<zsize; tstep++)
    {
        tstartarray[tstep] = tstart + 1.0*tstep*(tend-tstart)/zsize;
        tendarray[tstep] = tstart + 1.0*(tstep+1)*(tend-tstart)/zsize;
        exparray[tstep] = 0.0;
    }



    printf("tstart = %20.8f\n", tstart);
    printf("tend   = %20.8f\n", tend);

	int stream;
    for(stream=0; stream<1; stream++)
    {
		if(stream==0)
		{
			dtoffset = 0.0; // stream 0 is used as reference
			sprintf(datadirstream, "%s/%s", datadir, stream0);
		}
		else
		{
			dtoffset = +dtlag; // stream 1 is lagging behind by dtlag, so we bring it back in time
			// this is achieved by pushing/delaying the output timing window  
			sprintf(datadirstream, "%s/%s", datadir, stream1);
		}
		
		datfile = (StreamDataFile*) malloc(sizeof(StreamDataFile)*MaxNBdatFiles);
        //
        // Identify relevant files in directory
        //
        NBdatFiles = 0;
        d0 = opendir(datadirstream);
        if (d0) {
            while ((dir = readdir(d0)) != NULL) {
                ext = strrchr(dir->d_name, '.');
                if (!ext) {
                    // printf("no extension\n");
                } else {
                    if(strcmp(ext+1, "dat")==0)
                    {
                        tmpstring = remove_ext(dir->d_name, '.', '/');
                        sprintf(fname, "%s/%s", datadirstream, dir->d_name);
                        if((fp = fopen(fname, "r"))==NULL)
                        {
                            printf("Cannot open file \"%s\"\n", dir->d_name);
                            exit(0);
                        }
                        else
                        {
                            cnt = 0;
                            while(fscanf(fp, "%ld %ld %lf %lf %ld %ld\n", &vald1, &vald2, &valf1, &valf2, &vald3, &vald4)==6)
                            {
                                if(cnt == 0)
                                    datfile[NBdatFiles].tstart = valf2;
                                cnt++;
                            }
                            fclose(fp);
                            datfile[NBdatFiles].tend = valf2;
                            datfile[NBdatFiles].cnt = cnt;
							strcpy(datfile[NBdatFiles].name, tmpstring);
						}

                        if((datfile[NBdatFiles].tend > tstart) && (datfile[NBdatFiles].tstart < tend))
                        {
                            printf("%20s       %20.9f -> %20.9f   [%10ld]  %10.3f Hz\n", datfile[NBdatFiles].name, datfile[NBdatFiles].tstart, datfile[NBdatFiles].tend, datfile[NBdatFiles].cnt, datfile[NBdatFiles].cnt/(datfile[NBdatFiles].tend-datfile[NBdatFiles].tstart));
                            NBdatFiles++;
                        }
                    }
                }
            }
            closedir(d0);
        }
        printf("NBdatFiles = %ld\n", NBdatFiles);
		
        for(i=0; i<NBdatFiles; i++)
        {
            printf("FILE [%ld]: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f Hz\n", i,
                   datfile[i].name,
                   datfile[i].tstart,
                   datfile[i].tend,
                   datfile[i].cnt,
                   datfile[i].cnt/(datfile[i].tend-datfile[i].tstart));
		}


		printf("==========================================================\n");


        // sort files according to time
        quicksort_StreamDataFile(datfile, 0, NBdatFiles-1);

        for(i=0; i<NBdatFiles; i++)
        {
            printf("FILE [%ld]: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f Hz\n", i,
                   datfile[i].name,
                   datfile[i].tstart,
                   datfile[i].tend,
                   datfile[i].cnt,
                   datfile[i].cnt/(datfile[i].tend-datfile[i].tstart));
		}


		printf("==========================================================\n");




        int initOutput = 0;
        long xsize, ysize;
        long IDout;

        for(i=0; i<NBdatFiles; i++)
        {
            printf("FILE: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f Hz\n",
                   datfile[i].name,
                   datfile[i].tstart,
                   datfile[i].tend,
                   datfile[i].cnt,
                   datfile[i].cnt/(datfile[i].tend-datfile[i].tstart));

            // LOAD FITS FILE
            long IDc;
            sprintf(fname, "%s/%s.fits", datadirstream, datfile[i].name);
            printf("----------------------[%ld] LOADING FILE %s\n", i, fname);
            IDc = load_fits(fname, "im0C", 2);
            

            // CREATE OUTPUT CUBE IF FIRST FILE
            if(initOutput == 0)
            {
                xsize = data.image[IDc].md[0].size[0];
                ysize = data.image[IDc].md[0].size[1];
                xysize = xsize*ysize;
                if(stream==0)
					IDout = create_3Dimage_ID("outC0", xsize, ysize, zsize);
                else
					IDout = create_3Dimage_ID("outC1", xsize, ysize, zsize);
                initOutput = 1;
            }

            // start and end time for input exposures
            intarray_start = (double*) malloc(sizeof(double)*datfile[i].cnt);
            intarray_end   = (double*) malloc(sizeof(double)*datfile[i].cnt);
            dtarray = (double*) malloc(sizeof(double)*datfile[i].cnt);


            long j;

            sprintf(fname, "%s/%s.dat", datadirstream, datfile[i].name);
            printf("fname = %s\n", fname);
            fflush(stdout);

            if((fp = fopen(fname, "r"))==NULL)
            {
                printf("Cannot open file \"%s.dat\"\n", datfile[i].name);
                exit(0);
            }
            else
            {
                for(j=0; j<datfile[i].cnt; j++)
                {

                    if(fscanf(fp, "%ld %ld %lf %lf %ld %ld\n", &vald1, &vald2, &valf1, &valf2, &vald3, &vald4)!=6)
                    {
                        printf("fscanf error, %s line %d\n", __FILE__, __LINE__);
                        exit(0);
                    }
                    else
                        intarray_end[j] = valf2;
                }
                fclose(fp);
            }


            for(j=0; j<datfile[i].cnt-1; j++)
                dtarray[j] = intarray_end[j+1] - intarray_end[j];

            double dtmedian;
            qs_double(dtarray, 0, datfile[i].cnt-1);
            dtmedian = dtarray[(datfile[i].cnt-1)/2];
            printf("   dtmedian = %10.3f us\n", 1.0e6*dtmedian);

            // we assume here that every frame as the same exposure time, with 100% duty cycle
            for(j=0; j<datfile[i].cnt; j++)
                intarray_start[j] = intarray_end[j] - dtmedian;


            int j0 = 0;
            double expfrac;

			
            for(tstep=0; tstep<zsize; tstep++)
            {
                while((intarray_end[j0] < (tstartarray[tstep]+dtoffset) ) && (j0 < datfile[i].cnt))
                    j0++;
                j = j0;

                while( (intarray_start[j] < (tendarray[tstep]+dtoffset)) && (j<datfile[i].cnt) )
                {
                    expfrac = 1.0;

                    if((tstartarray[tstep]+dtoffset)>intarray_start[j])
                        expfrac -= ((tstartarray[tstep]+dtoffset)-intarray_start[j])/dtmedian;

                    if((tendarray[tstep]+dtoffset)<intarray_end[j])
                        expfrac -= (intarray_end[j]-(tendarray[tstep]+dtoffset))/dtmedian;

                    exparray[tstep] += expfrac;
                    
//                    printf("  FILE %d        %5ld   %8.6f  [%20.6f] -> %5ld\n", i, j, expfrac, intarray_start[j], tstep);

                    long ii;

                    switch(data.image[IDc].md[0].atype)
                    {
                    case _DATATYPE_UINT8 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.UI8[xysize*j+ii];
                        break;

                    case _DATATYPE_INT8 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.SI8[xysize*j+ii];
                        break;

                    case _DATATYPE_UINT16 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.UI16[xysize*j+ii];
                        break;

                    case _DATATYPE_INT16 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.SI16[xysize*j+ii];
                        break;

                    case _DATATYPE_UINT32 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.UI32[xysize*j+ii];
                        break;

                    case _DATATYPE_INT32 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.SI32[xysize*j+ii];
                        break;

                    case _DATATYPE_UINT64 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.UI64[xysize*j+ii];
                        break;

                    case _DATATYPE_INT64 :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.SI64[xysize*j+ii];
                        break;

                    case _DATATYPE_FLOAT :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.F[xysize*j+ii];
                        break;

                    case _DATATYPE_DOUBLE :
                        for (ii = 0; ii < xysize; ii++)
                            data.image[IDout].array.F[xysize*tstep+ii] += expfrac*data.image[IDc].array.D[xysize*j+ii];
                        break;

                    default :
                        list_image_ID();
                        printERROR(__FILE__,__func__,__LINE__,"atype value not recognised");
                        printf("ID %ld  atype = %d\n", IDc, data.image[IDc].md[0].atype);
                        exit(0);
                        break;
                    }
                    j++;
                }
            }


            delete_image_ID("im0C");
        }

        for(tstep=0; tstep<zsize; tstep++)
        {
            if(exparray[tstep] > 0.01)
            {
                long ii;
                for(ii=0; ii<xysize; ii++)
                    data.image[IDout].array.F[xysize*tstep+ii] /= exparray[tstep];
            }
        }



        free(datfile);
        free(intarray_start);
        free(intarray_end);
        free(dtarray);
    }

    free(tstartarray);
    free(tendarray);
    free(exparray);

    return 0;
}

