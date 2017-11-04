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

#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>

#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t) {
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time;
    time = mach_absolute_time();
    double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
    double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
    t->tv_sec = seconds;
    t->tv_nsec = nseconds;
    return 0;
}
#else
#include <time.h>
#endif

#include <ncurses.h>



#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"

#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
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




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */

extern long aoconfID_wfsim;              // declared in AOloopControl.c
extern long aoconfID_dmC;                // declared in AOloopControl.c
extern long aoconfID_dmRM;               // declared in AOloopControl.c
extern long aoconfID_DMmodes;            // declared in AOloopControl.c
extern long aoconfID_gainb;              // declared in AOloopControl.c
extern long aoconfID_limitb;             // declared in AOloopControl.c
extern long aoconfID_multfb;             // declared in AOloopControl.c
extern long aoconfID_DMmode_GAIN;        // declared in AOloopControl.c
extern long aoconfID_LIMIT_modes;        // declared in AOloopControl.c
extern long aoconfID_MULTF_modes;        // declared in AOloopControl.c
extern long aoconfID_cmd_modes;          // declared in AOloopControl.c
extern long aoconfID_meas_modes;         // declared in AOloopControl.c
extern long aoconfID_RMS_modes;          // declared in AOloopControl.c
extern long aoconfID_AVE_modes;          // declared in AOloopControl.c
extern long aoconfID_modeARPFgainAuto;   // declared in AOloopControl.c
     

static int wcol, wrow; // window size

// TIMING
static struct timespec tnow;
static struct timespec tdiff;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c

extern int AOloopcontrol_meminit;













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
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)==0) {
        AOloopControl_perfTest_mkTestDynamicModeSeq(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl);
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
	init_AOloopControl_perfTest();
	printf(" ...... Loading module %s\n", __FILE__);
}


int_fast8_t init_AOloopControl_perfTest()
{
    FILE *fp;

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "AO loop control");
    data.NBmodule++;


	
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

    RegisterCLIcommand("aolmktestmseq", __FILE__, AOloopControl_perfTest_mkTestDynamicModeSeq_cli, "make modal periodic test sequence", "<outname> <number of slices> <number of modes>", "aolmktestmseq outmc 100 50", "long AOloopControl_perfTest_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes)");

    RegisterCLIcommand("aolzrmsens", __FILE__, AOloopControl_perfTest_AnalyzeRM_sensitivity_cli, "Measure zonal RM sensitivity", "<DMmodes> <DMmask> <WFSref> <WFSresp> <WFSmask> <amplitude[nm]> <lambda[nm]> <outname>", "aolzrmsens DMmodes dmmask wfsref0 zrespmat wfsmask 0.1 outfile.txt", "long AOloopControl_perfTest_AnalyzeRM_sensitivity(const char *IDdmmodes_name, const char *IDdmmask_name, const char *IDwfsref_name, const char *IDwfsresp_name, const char *IDwfsmask_name, float amplimitnm, float lambdanm, const char *foutname)");

	RegisterCLIcommand("aoltimingstat", __FILE__, AOloopControl_LoopTimer_Analysis_cli, "Analysis of loop timing data", "<TimingImage> <TimingTXTfile> <outFile>", "aoltimingstat aol0_looptiming timing.txt outfile.txt", "long AOloopControl_LoopTimer_Analysis(char *IDname, char *fnametxt, char *outfname)");

}













/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. STATUS / TESTING / PERF MEASUREMENT                                   */
/* =============================================================================================== */
/* =============================================================================================== */





int_fast8_t AOloopControl_perfTest_printloopstatus(long loop, long nbcol, long IDmodeval_dm, long IDmodeval, long IDmodevalave, long IDmodevalrms, long ksize)
{
    long k, kmin, kmax;
    long col;
//    long nbl = 1;
    float AVElim = 0.01; // [um]
    float RMSlim = 0.01; // [um]
    char imname[200];
	float ratio0, ratio;
	int color;
    long IDblknb;
    long block;
    float valPFres, valOL, valWFS;
	long m;
	uint32_t *sizeout;
	float ARPFgainAutob[100];
	float ARPFgainAutob_tot[100];


    printw("    loop number %ld    ", loop);


    if(AOconf[loop].on == 1)
        printw("loop is ON     ");
    else
        printw("loop is OFF    ");
        
     printw(" [%12lu]", AOconf[loop].LOOPiteration);

    /*  if(AOconf[loop].logon == 1)
          printw("log is ON   ");
      else
          printw("log is OFF  ");

    */


    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) // block indices
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblknb = image_ID(imname);

    if(IDblknb==-1)
        IDblknb = read_sharedmem_image(imname);

	
	if(AOconf[loop].ARPFon==1)
	{
		if(aoconfID_modeARPFgainAuto == -1)
		{
		// multiplicative auto ratio on top of gain above
		sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
		sizeout[0] = AOconf[loop].NBDMmodes;
		sizeout[1] = 1;
		
		if(sprintf(imname, "aol%ld_mode_ARPFgainAuto", loop) < 1) 
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_modeARPFgainAuto = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
		COREMOD_MEMORY_image_set_createsem(imname, 10);
		// initialize the gain to zero for all modes
		for(m=0;m<AOconf[loop].NBDMmodes; m++)
			data.image[aoconfID_modeARPFgainAuto].array.F[m] = 1.0;
		free(sizeout);
		}
		
		for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
			{
				ARPFgainAutob[k] = 0.0;
				ARPFgainAutob_tot[k] = 0.0;
			}
		
        for(m=0; m<AOconf[loop].NBDMmodes; m++)
        {
            block = data.image[IDblknb].array.UI16[m];
			ARPFgainAutob[block] += data.image[aoconfID_modeARPFgainAuto].array.F[m];
			ARPFgainAutob_tot[block] += 1.0;
        }
		
		for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
			ARPFgainAutob[k] /= ARPFgainAutob_tot[k];
		
	}
	


    if(aoconfID_LIMIT_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_LIMIT_modes = read_sharedmem_image(imname);
    }


    printw("   STATUS = %3d %3d    ", AOconf[loop].status, AOconf[loop].statusM);

    kmax = (wrow-28)*(nbcol);

	printw("IMAGE TOTAL = %10f\n", AOconf[loop].WFStotalflux);
    printw("    Gain = %5.3f   maxlim = %5.3f     GPU = %d    kmax=%ld\n", AOconf[loop].gain, AOconf[loop].maxlimit, AOconf[loop].GPU0, kmax);
    printw("    DMprimWrite = %d   Predictive control state: %d        ARPF gain = %5.3f   AUTOTUNE LIM = %d (perc = %.2f %%  delta = %.3f nm mcoeff=%4.2f) GAIN = %d\n", AOconf[loop].DMprimaryWriteON, AOconf[loop].ARPFon, AOconf[loop].ARPFgain, AOconf[loop].AUTOTUNE_LIMITS_ON, AOconf[loop].AUTOTUNE_LIMITS_perc, 1000.0*AOconf[loop].AUTOTUNE_LIMITS_delta, AOconf[loop].AUTOTUNE_LIMITS_mcoeff, AOconf[loop].AUTOTUNE_GAINS_ON);
    printw(" TIMIMNG :  lfr = %9.3f Hz    hw lat = %5.3f fr   comp lat = %5.3f fr  wfs extr lat = %5.3f fr\n", AOconf[loop].loopfrequ, AOconf[loop].hardwlatency_frame, AOconf[loop].complatency_frame, AOconf[loop].wfsmextrlatency_frame);
    printw("loop iteration CNT : %lld\n", AOconf[loop].cnt);

    printw("\n");





    printw("=========== %6ld modes, %3ld blocks ================|------------ Telemetry [nm] ----------------|    |     LIMITS         |", AOconf[loop].NBDMmodes, AOconf[loop].DMmodesNBblock);
	if(AOconf[loop].ARPFon == 1)
		printw("  PFres  |  Ratio  |");
	printw("\n");

    printw("BLOCK  #modes [ min - max ]    gain   limit   multf  |       dmC     Input  ->       WFS   Ratio  |    | hits/step    perc  |");
	if(AOconf[loop].ARPFon==1)
		printw("         |         |");
	printw("\n");
	printw("\n");

    for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
    {
        if(k==0)
            kmin = 0;
        else
            kmin = AOconf[loop].indexmaxMB[k-1];

        attron(A_BOLD);
        printw("%3ld", k);
        attroff(A_BOLD);

        printw("    %4ld [ %4ld - %4ld ]   %5.3f  %7.5f  %5.3f", AOconf[loop].NBmodes_block[k], kmin, AOconf[loop].indexmaxMB[k]-1, data.image[aoconfID_gainb].array.F[k], data.image[aoconfID_limitb].array.F[k], data.image[aoconfID_multfb].array.F[k]);
        
        
        printw("  |  %8.2f  %8.2f  ->  %8.2f", 1000.0*(AOconf[loop].blockave_Crms[k]), 1000.0*AOconf[loop].blockave_OLrms[k], 1000.0*AOconf[loop].blockave_WFSrms[k]);
		

		
       
        ratio0 = AOconf[loop].blockave_WFSrms[k]/AOconf[loop].blockave_OLrms[k];
		if(ratio0>0.999)
			color=2;
		else
			color=3;
			
		attron(A_BOLD | COLOR_PAIR(color));
        printw("   %5.3f  ", ratio0);
        attroff(A_BOLD | COLOR_PAIR(color));

        if( AOconf[loop].blockave_limFrac[k] > 0.01 )
            attron(A_BOLD | COLOR_PAIR(2));

        printw("| %2ld | %9.3f  %6.2f\% |", k, AOconf[loop].blockave_limFrac[k],  100.0*AOconf[loop].blockave_limFrac[k]/AOconf[loop].NBmodes_block[k]);
        attroff(A_BOLD | COLOR_PAIR(2));
        
        if(AOconf[loop].ARPFon==1){
			printw("%8.2f |", 1000.0*AOconf[loop].blockave_PFresrms[k]);
			
			
			ratio = AOconf[loop].blockave_PFresrms[k]/AOconf[loop].blockave_OLrms[k];
			color = 0;
			if(ratio>1.0)
				color=2;
			if(ratio<ratio0)
				color=3;
				
			attron(A_BOLD | COLOR_PAIR(color));
			printw("  %5.3f |", ratio);
			attroff(A_BOLD | COLOR_PAIR(color));
			
			printw(" %6.4f", ARPFgainAutob[k]);
		}
	


		
		// WFS noise corrected	
	/*
		printw("\n");
		
		
		
		printw("          WFS noise removed ------->               ");
		
		valOL = AOconf[loop].blockave_OLrms[k]*AOconf[loop].blockave_OLrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
		if(valOL>0.0)
			valOL = sqrt(valOL);
		else
			valOL = 0.0;
		
		valWFS = AOconf[loop].blockave_WFSrms[k]*AOconf[loop].blockave_WFSrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
		if(valWFS>0.0)
			valWFS = sqrt(valWFS);
		else
			valWFS = 0.0;
			
		printw("  |            %8.2f  ->  %8.2f", 1000.0*valOL, 1000.0*valWFS);
		ratio0 = valWFS/valOL;
		if(ratio0>0.999)
			color=2;
		else
			color=3;
			
		attron(A_BOLD | COLOR_PAIR(color));
        printw("   %5.3f  ", ratio0);
        attroff(A_BOLD | COLOR_PAIR(color));

        if( AOconf[loop].blockave_limFrac[k] > 0.01 )
            attron(A_BOLD | COLOR_PAIR(2));

        printw("|    |                    |", k, AOconf[loop].blockave_limFrac[k],  100.0*AOconf[loop].blockave_limFrac[k]/AOconf[loop].NBmodes_block[k]);
        attroff(A_BOLD | COLOR_PAIR(2));
        
        if(AOconf[loop].ARPFon==1){
			valPFres = AOconf[loop].blockave_PFresrms[k]*AOconf[loop].blockave_PFresrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
			if(valPFres>0.0)
				valPFres = sqrt(valPFres);
			else
				valPFres = 0.0;
			printw("%8.2f |", 1000.0*valPFres);
			
			
			ratio = valPFres/valOL;
			color = 0;
			if(ratio>1.0)
				color=2;
			if(ratio<ratio0)
				color=3;
				
			attron(A_BOLD | COLOR_PAIR(color));
			printw("  %5.3f |", ratio);
			attroff(A_BOLD | COLOR_PAIR(color));
		}*/
		printw("\n");

    }


    printw("\n");

    printw(" ALL   %4ld                                        ", AOconf[loop].NBDMmodes);
    printw("  |  %8.2f  %8.2f  ->  %8.2f", 1000.0*AOconf[loop].ALLave_Crms, 1000.0*AOconf[loop].ALLave_OLrms, 1000.0*AOconf[loop].ALLave_WFSrms);

    attron(A_BOLD);
    printw("   %5.3f  ", AOconf[loop].ALLave_WFSrms/AOconf[loop].ALLave_OLrms);
    attroff(A_BOLD);

    printw("| %2ld | %9.3f  %6.2f\% |\n", k, AOconf[loop].ALLave_limFrac,  100.0*AOconf[loop].ALLave_limFrac/AOconf[loop].NBDMmodes);

    printw("\n");

    //printw("            MODAL RMS (ALL MODES) : %6.4lf     AVERAGE :  %8.6lf       ( %20g / %8lld )\n", sqrt(AOconf[loop].RMSmodes), sqrt(AOconf[loop].RMSmodesCumul/AOconf[loop].RMSmodesCumulcnt), AOconf[loop].RMSmodesCumul, AOconf[loop].RMSmodesCumulcnt);



	// ====================================================================
	//                    SHOW INDIVIDUAL MODES
	// ====================================================================


    print_header(" [ gain 1000xlimit  mult ] MODES [nm]    DM correction -- WFS value -- WFS average -- WFS RMS     ", '-');


    if(kmax>AOconf[loop].NBDMmodes)
        kmax = AOconf[loop].NBDMmodes;

    col = 0;
    for(k=0; k<kmax; k++)
    {
	    float val;
		
		
        attron(A_BOLD);
        printw("%4ld ", k);
        attroff(A_BOLD);

        printw("[%5.3f %8.4f %5.3f] ", AOconf[loop].gain * data.image[aoconfID_gainb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoconfID_DMmode_GAIN].array.F[k], 1000.0 * data.image[aoconfID_limitb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoconfID_LIMIT_modes].array.F[k], AOconf[loop].mult * data.image[aoconfID_multfb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoconfID_MULTF_modes].array.F[k]);

        // print current value on DM
        val = data.image[IDmodeval_dm].array.F[k];
        if(fabs(val)>0.99*AOconf[loop].maxlimit)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%+8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
        {
            if(fabs(val)>0.99*AOconf[loop].maxlimit*data.image[aoconfID_LIMIT_modes].array.F[k])
            {
                attron(COLOR_PAIR(1));
                printw("%+8.3f ", 1000.0*val);
                attroff(COLOR_PAIR(1));
            }
            else
                printw("%+8.3f ", 1000.0*val);
        }

        // last reading from WFS
        printw("%+8.3f ", 1000.0*data.image[IDmodeval].array.F[k]);


        // Time average
        val = data.image[IDmodevalave].array.F[(ksize-1)*AOconf[loop].NBDMmodes+k];
        if(fabs(val)>AVElim)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%+8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
            printw("%+8.3f ", 1000.0*val);


        // RMS variation
        val = sqrt(data.image[IDmodevalrms].array.F[(ksize-1)*AOconf[loop].NBDMmodes+k]);
        if(fabs(val)>RMSlim)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
            printw("%8.3f ", 1000.0*val);

        col++;
        if(col==nbcol)
        {
            col = 0;
            printw("\n");
        }
        else
            printw(" | ");
    }

    return(0);
}




int_fast8_t AOloopControl_perfTest_loopMonitor(long loop, double frequ, long nbcol)
{
    char name[200];
    // DM mode values
    long IDmodeval_dm;

    // WFS modes values
    long IDmodeval;
    long ksize;
    long IDmodevalave;
    long IDmodevalrms;
    char fname[200];


    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    printf("MEMORY HAS BEEN INITIALIZED\n");
    fflush(stdout);

    // load arrays that are required
    if(aoconfID_cmd_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_cmd", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    if(aoconfID_meas_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_meas", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_meas_modes = read_sharedmem_image(name);
    }


    if(aoconfID_RMS_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_RMS", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_RMS_modes = read_sharedmem_image(name);
    }

    if(aoconfID_AVE_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_AVE", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_AVE_modes = read_sharedmem_image(name);
    }


    // blocks
    if(aoconfID_gainb == -1)
    {
        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_gainb = read_sharedmem_image(name);
    }

    if(aoconfID_multfb == -1)
    {
        if(sprintf(name, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_multfb = read_sharedmem_image(name);
    }

    if(aoconfID_limitb == -1)
    {
        if(sprintf(name, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_limitb = read_sharedmem_image(name);
    }


    // individual modes

    if(aoconfID_DMmode_GAIN==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_GAIN", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    if(aoconfID_LIMIT_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    if(aoconfID_MULTF_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_MULTF_modes = read_sharedmem_image(name);
    }








    // real-time DM mode value

    if(sprintf(fname, "aol%ld_modeval_dm_now", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm = read_sharedmem_image(fname);

    // real-time WFS mode value
    if(sprintf(fname, "aol%ld_modeval", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(fname);

    // averaged WFS residual modes, computed by CUDACOMP_extractModesLoop
    if(sprintf(fname, "aol%ld_modeval_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalave = read_sharedmem_image(fname);
    ksize = data.image[IDmodevalave].md[0].size[1]; // number of averaging line, each line is 2x averaged of previous line

    // averaged WFS residual modes RMS, computed by CUDACOMP_extractModesLoop
    if(sprintf(fname, "aol%ld_modeval_rms", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalrms = read_sharedmem_image(fname);




    initscr();
    getmaxyx(stdscr, wrow, wcol);


    start_color();
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    while( !kbdhit() )
    {
        usleep((long) (1000000.0/frequ));
        clear();
        attron(A_BOLD);
        print_header(" PRESS ANY KEY TO STOP MONITOR ", '-');
        attroff(A_BOLD);

        AOloopControl_perfTest_printloopstatus(loop, nbcol, IDmodeval_dm, IDmodeval, IDmodevalave, IDmodevalrms, ksize);

        refresh();
    }
    endwin();

    return 0;
}







// if updateconf=1, update configuration
int_fast8_t AOloopControl_perfTest_statusStats(int updateconf, long NBsample)
{
    long k;
    long statusmax = 21;
    long *statuscnt;
    long *statusMcnt;
    long *statusM1cnt;
    float usec0, usec1;
    int st;
    int RT_priority = 91; //any number from 0-99
    struct sched_param schedpar;
    const char *statusdef[21];
    const char *statusMdef[21];
    const char *statusM1def[21];
    int gpu;
    int nbgpu;
    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    double tdiffv;
    long *statusgpucnt;
    long *statusgpucnt2;
    double loopiterus;

    long long loopcnt;
    char imname[200];
    long long wfsimcnt;
    long long dmCcnt;

    int ret;


    float loopfrequ_measured, complatency_measured, wfsmextrlatency_measured;
    float complatency_frame_measured, wfsmextrlatency_frame_measured;


    FILE *fp;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);



    statusdef[0] = "LOAD IMAGE";
    statusdef[1] = "DARK SUBTRACT";
    statusdef[2] = "COMPUTE WFS IMAGE TOTAL";
    statusdef[3] = "NORMALIZE WFS IMAGE";
    statusdef[4] = "SUBTRACT REFERENCE";
    statusdef[5] = "MULTIPLYING BY CONTROL MATRIX -> MODE VALUES : SETUP";
    statusdef[6] = "START CONTROL MATRIX MULTIPLICATION: CHECK IF NEW CM EXISTS";
    statusdef[7] = "CONTROL MATRIX MULT: CREATE COMPUTING THREADS";
    statusdef[8] = "CONTROL MATRIX MULT: WAIT FOR THREADS TO COMPLETE";
    statusdef[9] = "CONTROL MATRIX MULT: COMBINE TRHEADS RESULTS";
    statusdef[10] = "CONTROL MATRIX MULT: INCREMENT COUNTER AND EXIT FUNCTION";
    statusdef[11] = "MULTIPLYING BY GAINS";

    if(AOconf[LOOPNUMBER].CMMODE==0)
    {
        statusdef[12] = "ENTER SET DM MODES";
        statusdef[13] = "START DM MODES MATRIX MULTIPLICATION";
        statusdef[14] = "MATRIX MULT: CREATE COMPUTING THREADS";
        statusdef[15] = "MATRIX MULT: WAIT FOR THREADS TO COMPLETE";
        statusdef[16] = "MATRIX MULT: COMBINE TRHEADS RESULTS";
        statusdef[17] = "MATRIX MULT: INCREMENT COUNTER AND EXIT FUNCTION";
    }
    else
    {
        statusdef[12] = "REMOVE NAN VALUES";
        statusdef[13] = "ENFORCE STROKE LIMITS";
        statusdef[14] = "-";
        statusdef[15] = "-";
        statusdef[16] = "-";
        statusdef[17] = "-";
    }

    statusdef[18] = "LOG DATA";
    statusdef[19] = "READING IMAGE";
    statusdef[20] = "... WAITING FOR IMAGE";




    statusMdef[0] = "DARK SUBTRACT";
    statusMdef[1] = "NORMALIZE";
    statusMdef[2] = "EXTRACT WFS MODES";
    statusMdef[3] = "UPDATE CURRENT DM STATE";
    statusMdef[4] = "MIX PREDICTION WITH CURRENT DM STATE";
    statusMdef[5] = "MODAL FILTERING / CLIPPING";
    statusMdef[6] = "INTER-PROCESS LATENCY";
    statusMdef[7] = "";
    statusMdef[8] = "";
    statusMdef[9] = "";
    statusMdef[10] = "MODES TO DM ACTUATORS (GPU)";
    statusMdef[11] = "";
    statusMdef[12] = "";
    statusMdef[13] = "";
    statusMdef[14] = "";
    statusMdef[15] = "";
    statusMdef[16] = "";
    statusMdef[17] = "";
    statusMdef[18] = "";
    statusMdef[19] = "";
    statusMdef[20] = "... WAITING FOR IMAGE imWFS0";


    statusM1def[0] = "WRITING MODAL CORRECTION IN CIRCULAR BUFFER";
    statusM1def[1] = "COMPUTING TIME-DELAYED MODAL CORRECTION";
    statusM1def[2] = "COMPUTING TIME-DELAYED PREDICTED CORRECTION";
    statusM1def[3] = "COMPUTING OPEN LOOP WF";
    statusM1def[4] = "COMPUTING TELEMETRY";
    statusM1def[5] = "... WAITING FOR INPUT";
    statusM1def[6] = "";
    statusM1def[7] = "";
    statusM1def[8] = "";
    statusM1def[9] = "";
    statusM1def[10] = "";
    statusM1def[11] = "";
    statusM1def[12] = "";
    statusM1def[13] = "";
    statusM1def[14] = "";
    statusM1def[15] = "";
    statusM1def[16] = "";
    statusM1def[17] = "";
    statusM1def[18] = "";
    statusM1def[19] = "";
    statusM1def[20] = "";




    usec0 = 50.0;
    usec1 = 150.0;



    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


    nbgpu = AOconf[LOOPNUMBER].GPU0;


    printf("Measuring loop status distribution \n");
    fflush(stdout);

    statuscnt = (long*) malloc(sizeof(long)*statusmax);
    statusMcnt = (long*) malloc(sizeof(long)*statusmax);
    statusM1cnt = (long*) malloc(sizeof(long)*statusmax);
    statusgpucnt = (long*) malloc(sizeof(long)*nbgpu*10);
    statusgpucnt2 = (long*) malloc(sizeof(long)*nbgpu*10);


    for(st=0; st<statusmax; st++)
    {
        statuscnt[st] = 0;
        statusMcnt[st] = 0;
        statusM1cnt[st] = 0;
    }

    for(st=0; st<nbgpu*10; st++)
    {
        statusgpucnt[st] = 0;
        statusgpucnt2[st] = 0;
    }


    if(sprintf(imname, "aol%ld_wfsim", LOOPNUMBER) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoconfID_wfsim = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_dmC", LOOPNUMBER) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoconfID_dmC = read_sharedmem_image(imname);


    wfsimcnt = data.image[aoconfID_wfsim].md[0].cnt0;
    dmCcnt = data.image[aoconfID_dmC].md[0].cnt0;



    loopcnt = AOconf[LOOPNUMBER].cnt;
    clock_gettime(CLOCK_REALTIME, &t1);
    for(k=0; k<NBsample; k++)
    {
		int stM;
		int stM1;
		
        usleep((long) (usec0 + usec1*(1.0*k/NBsample)));
        st = AOconf[LOOPNUMBER].status;
        stM = AOconf[LOOPNUMBER].statusM;
        stM1 = AOconf[LOOPNUMBER].statusM1;
        
        if(st<statusmax)
            statuscnt[st]++;
        if(stM<statusmax)
            statusMcnt[stM]++;
         if(stM1<statusmax)
            statusM1cnt[stM1]++;       
        
        
        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            // 1st matrix mult
            st = 10*gpu + AOconf[LOOPNUMBER].GPUstatus[gpu];
            statusgpucnt[st]++;

            // 2nd matrix mult
            st = 10*gpu + AOconf[LOOPNUMBER].GPUstatus[10+gpu];
            statusgpucnt2[st]++;
        }
    }
    loopcnt = AOconf[LOOPNUMBER].cnt - loopcnt;
    wfsimcnt = data.image[aoconfID_wfsim].md[0].cnt0 - wfsimcnt;
    dmCcnt = data.image[aoconfID_dmC].md[0].cnt0 - dmCcnt;

    clock_gettime(CLOCK_REALTIME, &t2);
    tdiff = info_time_diff(t1, t2);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    printf("\n");
    loopiterus = 1.0e6*tdiffv/loopcnt;
    printf("Time diff = %f sec \n", tdiffv);
    printf("Loop freq = %8.2f Hz   -> single interation = %8.3f us\n", 1.0*loopcnt/tdiffv, loopiterus);
    printf("Number of iterations    loop: %10lld   wfs: %lld   dmC : %lld\n", loopcnt, wfsimcnt, dmCcnt);
    printf("MISSED FRAMES = %lld    fraction = %7.4f %%\n", wfsimcnt-loopcnt, 100.0*(wfsimcnt-loopcnt)/wfsimcnt);

    printf("\n");


    loopfrequ_measured = 1.0*loopcnt/tdiffv;
    if(updateconf==1)
        AOconf[LOOPNUMBER].loopfrequ = loopfrequ_measured;

	// Primary control matrix computation latency
    complatency_frame_measured = 1.0-1.0*statuscnt[20]/NBsample;
    if(updateconf==1)
        AOconf[LOOPNUMBER].complatency_frame = complatency_frame_measured;

    complatency_measured = complatency_frame_measured/loopfrequ_measured;
    if(updateconf==1)
        AOconf[LOOPNUMBER].complatency = complatency_measured;



    wfsmextrlatency_frame_measured = 1.0-1.0*statusMcnt[20]/NBsample;
    printf("==========> %ld %ld -> %f\n", statusMcnt[20], NBsample, wfsmextrlatency_frame_measured);
    if(updateconf==1)
        AOconf[LOOPNUMBER].wfsmextrlatency_frame = wfsmextrlatency_frame_measured;

    wfsmextrlatency_measured = wfsmextrlatency_frame_measured / loopfrequ_measured;
    if(updateconf==1)
        AOconf[LOOPNUMBER].wfsmextrlatency = wfsmextrlatency_measured;

    if(updateconf==1)
    {
        fp = fopen("conf/param_loopfrequ.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].loopfrequ);
        fclose(fp);
    }

    if((fp=fopen("./conf/param_hardwlatency.txt", "r"))==NULL)
    {
        printf("WARNING: file ./conf/param_hardwlatency.txt missing\n");
    }
    else
    {
        if(fscanf(fp, "%50f", &AOconf[LOOPNUMBER].hardwlatency) != 1)
            printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

        printf("hardware latency = %f\n", AOconf[LOOPNUMBER].hardwlatency);
        fclose(fp);
        fflush(stdout);
    }

    printf("hardwlatency = %f\n", AOconf[LOOPNUMBER].hardwlatency);
    if(updateconf==1)
    {
        AOconf[LOOPNUMBER].hardwlatency_frame = AOconf[LOOPNUMBER].hardwlatency * AOconf[LOOPNUMBER].loopfrequ;

        fp = fopen("conf/param_hardwlatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].hardwlatency_frame);
        fclose(fp);

        fp = fopen("conf/param_complatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].complatency);
        fclose(fp);

        fp = fopen("conf/param_complatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].complatency_frame);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].wfsmextrlatency);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].wfsmextrlatency_frame);
        fclose(fp);
    }



    for(st=0; st<statusmax; st++)
        printf("STATUS %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statuscnt[st]/NBsample, statuscnt[st], NBsample, loopiterus*statuscnt[st]/NBsample , statusdef[st]);


	

    if(AOconf[LOOPNUMBER].GPU0!=0)
    {
        printf("\n");
        printf("          ----1--------2--------3--------4--------5--------6----\n");
        printf("                   wait im | ->GPU |     COMPUTE     |   ->CPU  \n");
        printf("          ------------------------------------------------------\n");

        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+1]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+2]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+3]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+4]/NBsample);
            printf("  %5.2f %%",   100.0*statusgpucnt[10*gpu+5]/NBsample);
            printf("  %5.2f %%\n",  100.0*statusgpucnt[10*gpu+6]/NBsample);
        }
        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+1]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+2]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+3]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+4]/NBsample);
            printf(" %5.2f us",   loopiterus*statusgpucnt[10*gpu+5]/NBsample);
            printf(" %5.2f us\n",  loopiterus*statusgpucnt[10*gpu+6]/NBsample);
        }

        printf("\n");
        if(AOconf[LOOPNUMBER].CMMODE == 0)
        {
            printf("          ----1--------2--------3--------4--------5--------6----\n");
            for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
            {
                printf("GPU %2d  : ", gpu);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+1]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+2]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+3]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+4]/NBsample);
                printf("  %5.2f %%",   100.0*statusgpucnt2[10*gpu+5]/NBsample);
                printf("  %5.2f %%\n",  100.0*statusgpucnt2[10*gpu+6]/NBsample);
            }
        }
    }


	printf("\n--------------- MODAL STRING -------------------------------------------------------------\n");
    for(st=0; st<statusmax; st++)
        if(strlen(statusMdef[st])>0)
            printf("STATUSM  %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statusMcnt[st]/NBsample, statusMcnt[st], NBsample, loopiterus*statusMcnt[st]/NBsample , statusMdef[st]);




	printf("\n--------------- AUX MODAL STRING ---------------------------------------------------------\n");
    for(st=0; st<statusmax; st++)
        if(strlen(statusM1def[st])>0)
            printf("STATUSM1 %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statusM1cnt[st]/NBsample, statusM1cnt[st], NBsample, loopiterus*statusM1cnt[st]/NBsample , statusM1def[st]);



    free(statuscnt);
    free(statusMcnt);
    free(statusgpucnt);
    free(statusgpucnt2);


    return 0;
}






int_fast8_t AOloopControl_perfTest_resetRMSperf()
{
    long k;
    char name[200];
    long kmin, kmax;


    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].RMSmodesCumul = 0.0;
    AOconf[LOOPNUMBER].RMSmodesCumulcnt = 0;


    return 0;
}






int_fast8_t AOloopControl_perfTest_showparams(long loop)
{

    printf("loop number %ld\n", loop);

    if(AOconf[loop].on == 1)
        printf("loop is ON\n");
    else
        printf("loop is OFF\n");

    printf("Global gain = %f   maxlim = %f\n  multcoeff = %f  GPU = %d\n", AOconf[loop].gain, AOconf[loop].maxlimit, AOconf[loop].mult, AOconf[loop].GPU0);
    printf("    Predictive control state: %d        ARPF gain = %5.3f   AUTOTUNE: lim %d gain %d\n", AOconf[loop].ARPFon, AOconf[loop].ARPFgain, AOconf[loop].AUTOTUNE_LIMITS_ON,  AOconf[loop].AUTOTUNE_GAINS_ON);
    printf("WFS norm floor = %f\n", AOconf[loop].WFSnormfloor);

    printf("loopfrequ               =  %8.2f Hz\n", AOconf[loop].loopfrequ);
    printf("hardwlatency_frame      =  %8.2f fr\n", AOconf[loop].hardwlatency_frame);
    printf("complatency_frame       =  %8.2f fr\n", AOconf[loop].complatency_frame);
    printf("wfsmextrlatency_frame   =  %8.2f fr\n", AOconf[loop].wfsmextrlatency_frame);


    return 0;
}







int_fast8_t AOcontrolLoop_perfTest_TestDMSpeed(const char *dmname, long delayus, long NBpts, float ampl)
{
    long IDdm;
    long dmxsize, dmysize, dmsize;
    long ii, jj, kk;
    long ID1;
    float x, y, x1;
    char *ptr;

    long IDdm0, IDdm1; // DM shapes



    IDdm = image_ID(dmname);
    dmxsize = data.image[IDdm].md[0].size[0];
    dmysize = data.image[IDdm].md[0].size[1];
    dmsize = dmxsize*dmysize;



    ID1 = create_3Dimage_ID("dmpokeseq", dmxsize, dmysize, NBpts);
    for(kk=0; kk<NBpts; kk++)
    {
		float pha;
		
        pha = 2.0*M_PI*kk/NBpts;
        for(ii=0; ii<dmxsize; ii++)
            for(jj=0; jj<dmysize; jj++)
            {
                x = (2.0*ii/dmxsize)-1.0;
                y = (2.0*jj/dmysize)-1.0;
                x1 = x*cos(pha)-y*sin(pha);
                data.image[ID1].array.F[kk*dmsize+jj*dmxsize+ii] = ampl*x1;
            }
    }

    while(1)
    {
        for(kk=0; kk<NBpts; kk++)
        {
            ptr = (char*) data.image[ID1].array.F;
            ptr += sizeof(float)*dmsize*kk;
            data.image[IDdm].md[0].write = 1;
            memcpy(data.image[IDdm].array.F, ptr, sizeof(float)*dmsize);
            data.image[IDdm].md[0].write = 0;
            data.image[IDdm].md[0].cnt0 ++;
            usleep(delayus);
        }
    }

    return(0);
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
    //    IDwfsc = create_3Dimage_ID("_testwfsc", wfsxsize, wfsysize, wfs_NBframesmax);


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



    for(iter=0; iter<NBiter; iter++)
    {
		//double tlastdouble;
		double tstartdouble;
		long NBwfsframe;
	    unsigned long wfscnt0;
        double latencymax = 0.0;
	    double latency;
		
        printf("ITERATION %5ld / %5ld\n", iter, NBiter);
        fflush(stdout);


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
            printf("[%8ld]  %f  %f\n", wfsframe, dt, dtmax);
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

        printf("Hardware latency = %f ms  = %ld frames\n", 1000.0*latency, kkmax);
        if(latency > latencymax)
        {
            latencymax = latency;
            save_fits("_testwfsc", "!./timingstats/maxlatencyseq.fits");
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

    if(sprintf(command, "echo %.3f > conf/param_loopfrequ.txt", 1.0*(wfscntend-wfscntstart)/dt ) < 1)
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
    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_DMmodes==-1)
    {
		char name[200];
		
        if(sprintf(name, "aol%ld_DMmodes", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_DMmodes = read_sharedmem_image(name);
    }

    if(aoconfID_dmRM==-1)
        aoconfID_dmRM = read_sharedmem_image(AOconf[LOOPNUMBER].dmRMname);


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
            arrayf[i] = ampl*data.image[aoconfID_DMmodes].array.F[index*AOconf[LOOPNUMBER].sizeDM+i];



        data.image[aoconfID_dmRM].md[0].write = 1;
        memcpy (data.image[aoconfID_dmRM].array.F, arrayf, sizeof(float)*AOconf[LOOPNUMBER].sizeDM);
        data.image[aoconfID_dmRM].md[0].cnt0++;
        data.image[aoconfID_dmRM].md[0].write = 0;

        free(arrayf);
        AOconf[LOOPNUMBER].DMupdatecnt ++;
    }

    return(0);
}








//
// Measures mode temporal response (measurement and rejection)
//
long AOloopControl_perfTest_TestDMmodeResp(const char *DMmodes_name, long index, float ampl, float fmin, float fmax, float fmultstep, float avetime, long dtus, const char *DMmask_name, const char *DMstream_in_name, const char *DMstream_out_name, const char *IDout_name)
{
    long IDout;
    long IDmodes, IDdmmask, IDdmin, IDdmout;
    long dmxsize, dmysize, dmsize, NBmodes;
    float f;
    struct timespec tstart;
    long nbf;
    long IDrec_dmout;
    long ii, kk, kmax;
    long IDdmtmp;
    float pha, coeff;
    float *timearray;
    char *ptr;
    long IDcoeff;
    float SVDeps = 1.0e-3;
    int SVDreuse = 0;
    long IDcoeffarray;
    long m;
    FILE *fp;
    char fname[200];
    long kmaxmax = 100000;
    long ID;


    kk = index;

    IDmodes = image_ID(DMmodes_name);
    IDdmin = image_ID(DMstream_in_name);
    IDdmout = image_ID(DMstream_out_name);
    IDdmmask = image_ID(DMmask_name);

    dmxsize = data.image[IDmodes].md[0].size[0];
    dmysize = data.image[IDmodes].md[0].size[1];
    dmsize = dmxsize*dmysize;
    NBmodes = data.image[IDmodes].md[0].size[2];


    if(data.image[IDdmin].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMstream_in_name, (long) data.image[IDdmin].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmin].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMstream_in_name, (long) data.image[IDdmin].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMstream_out_name, (long) data.image[IDdmout].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMstream_out_name, (long) data.image[IDdmout].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMmask_name, (long) data.image[IDdmmask].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMmask_name, (long) data.image[IDdmmask].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }



    nbf = 0;
    for(f=fmin; f<fmax; f*=fmultstep)
        nbf++;



    // TEST
    // Save DM mode
    ID = create_2Dimage_ID("testmrespm", dmxsize, dmysize);
    for(ii=0; ii<dmsize; ii++)
        data.image[ID].array.F[ii] = data.image[IDmodes].array.F[kk*dmsize+ii];
    save_fits("testmrespm", "!testmrespm.fits");


    // SET UP RECORDING CUBES
    kmax = (long) (avetime/(1.0e-6*dtus));
    if(kmax>kmaxmax)
        kmax = kmaxmax;

    timearray = (float*) malloc(sizeof(float)*kmax);
    IDrec_dmout = create_3Dimage_ID("_tmprecdmout", dmxsize, dmysize, kmax);

    IDcoeffarray = create_2Dimage_ID("_tmpcoeffarray", kmax, NBmodes);

    if(sprintf(fname, "mode%03ld_PSD.txt", kk) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if( (fp = fopen(fname, "w"))==NULL)
    {
        printf("ERROR: cannot create file \"%s\"", fname);
        exit(0);
    }

    IDout = create_2Dimage_ID(IDout_name, nbf, NBmodes);
    IDdmtmp = create_2Dimage_ID("_tmpdm", dmxsize, dmysize);

    for(f=fmin; f<fmax; f*=fmultstep)
    {
        float runtime = 0.0;
		long k = 0;
		long k1;
	    float coscoeff, sincoeff;
        float PSDamp, PSDpha;


		

        clock_gettime(CLOCK_REALTIME, &tstart);
        while((runtime < avetime)&&(k<kmax))
        {
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(tstart, tnow); 
            runtime = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            pha = 2.0*M_PI*runtime*f;
            coeff = ampl*cos(pha);

            printf("mode %4ld  f = %f ( %f -> %f)   runtime = %10.3f sec    ampl = %f   pha = %f   coeff = %f\n", kk, f, fmin, fmax, runtime, ampl, pha, coeff);
            fflush(stdout);

            // APPLY MODE TO DM
            data.image[IDdmin].md[0].write = 1;
            for(ii=0; ii<dmsize; ii++)
                data.image[IDdmin].array.F[ii] = coeff*data.image[IDmodes].array.F[kk*dmsize+ii];
            data.image[IDdmin].md[0].cnt0++;
            data.image[IDdmin].md[0].write = 0;

            // RECORD
            ptr = (char*) data.image[IDrec_dmout].array.F;
            ptr += sizeof(float)*k*dmsize;
            memcpy(ptr, data.image[IDdmout].array.F, sizeof(float)*dmsize); //out->in

            timearray[k] = runtime;

            usleep(dtus);
            k++;
        }

        // ZERO DM
        data.image[IDdmin].md[0].write = 1;
        for(ii=0; ii<dmsize; ii++)
            data.image[IDdmin].array.F[ii] = 0.0;
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;


        k1 = k;
        //    save_fits("_tmprecdmout", "!_tmprecdmout.fits");


        printf("\n\n");

        // PROCESS RECORDED DATA
        for(k=0; k<k1; k++)
        {
            printf("\r  %5ld / %5ld     ", k, k1);
            fflush(stdout);

            ptr = (char*) data.image[IDrec_dmout].array.F;
            ptr += sizeof(float)*k*dmsize;
            memcpy(data.image[IDdmtmp].array.F, ptr, sizeof(float)*dmsize);
            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
            SVDreuse = 1;
            IDcoeff = image_ID("dmcoeffs");
            for(m=0; m<NBmodes; m++)
                data.image[IDcoeffarray].array.F[m*kmax+k] = data.image[IDcoeff].array.F[m];
            delete_image_ID("dmcoeffs");

        }
        printf("\n\n");

        save_fits("_tmpcoeffarray", "!_tmpcoeffarray.fits");

        // EXTRACT AMPLITUDE AND PHASE
        coscoeff = 0.0;
        sincoeff = 0.0;
        for(k=k1/4; k<k1; k++)
        {
            pha = 2.0*M_PI*timearray[k]*f;
            coscoeff += cos(pha)*data.image[IDcoeffarray].array.F[kk*kmax+k];
            sincoeff += sin(pha)*data.image[IDcoeffarray].array.F[kk*kmax+k];
        }
        coscoeff /= (0.5*k1*0.75);
        sincoeff /= (0.5*k1*0.75);

        PSDamp = coscoeff*coscoeff + sincoeff*sincoeff;
        PSDpha = atan2(-sincoeff, -coscoeff);
        fp = fopen(fname, "a");
        fprintf(fp, "    %20f %20.18f %20f\n", f, sqrt(PSDamp)/ampl, PSDpha);
        fclose(fp);
    }

    delete_image_ID("_tmpdm");

    free(timearray);


    return(IDout);
}




//
//
//
long AOloopControl_perfTest_TestDMmodes_Recovery(const char *DMmodes_name, float ampl, const char *DMmask_name, const char *DMstream_in_name, const char *DMstream_out_name, const char *DMstream_meas_name, long tlagus, long NBave, const char *IDout_name, const char *IDoutrms_name, const char *IDoutmeas_name, const char *IDoutmeasrms_name)
{
    long IDout, IDoutrms, IDoutmeas, IDoutmeasrms;
    long IDmodes, IDdmmask, IDdmin, IDmeasout, IDdmout;
    long dmxsize, dmysize, dmsize, NBmodes;
    long kk;
    long IDdmtmp, IDmeastmp;
    int SVDreuse = 0;
    float SVDeps = 1.0e-3;
    long IDcoeffarray;
    long IDcoeffarraymeas;
    long IDcoeff;
    long ii, kk1;


    IDmodes = image_ID(DMmodes_name);
    IDdmin = image_ID(DMstream_in_name);
    IDdmout = image_ID(DMstream_out_name);
    IDmeasout = image_ID(DMstream_meas_name);
    IDdmmask = image_ID(DMmask_name);

    dmxsize = data.image[IDmodes].md[0].size[0];
    dmysize = data.image[IDmodes].md[0].size[1];
    dmsize = dmxsize*dmysize;
    NBmodes = data.image[IDmodes].md[0].size[2];


    //
    // CHECK IMAGE SIZES
    //
    if(data.image[IDdmin].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMstream_in_name, (long) data.image[IDdmin].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmin].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMstream_in_name, (long) data.image[IDdmin].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMstream_out_name, (long) data.image[IDdmout].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMstream_out_name, (long) data.image[IDdmout].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDmeasout].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMstream_meas_name, (long) data.image[IDmeasout].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDmeasout].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMstream_meas_name, (long) data.image[IDmeasout].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[0]!=data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n", DMmask_name, (long) data.image[IDdmmask].md[0].size[0], DMmodes_name, (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[1]!=data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n", DMmask_name, (long) data.image[IDdmmask].md[0].size[1], DMmodes_name, (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }


    IDout = create_2Dimage_ID(IDout_name, NBmodes, NBmodes);
    IDoutrms = create_2Dimage_ID(IDoutrms_name, NBmodes, NBmodes);
    IDoutmeas = create_2Dimage_ID(IDoutmeas_name, NBmodes, NBmodes);
    IDoutmeasrms = create_2Dimage_ID(IDoutmeasrms_name, NBmodes, NBmodes);
    IDdmtmp = create_2Dimage_ID("_tmpdm", dmxsize, dmysize);
    IDmeastmp = create_2Dimage_ID("_tmpmeas", dmxsize, dmysize);

    IDcoeffarray = create_2Dimage_ID("_coeffarray", NBmodes, NBave);
    IDcoeffarraymeas = create_2Dimage_ID("_coeffarraymeas", NBmodes, NBave);

    printf("Initialize SVD ... ");
    fflush(stdout);
    linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
    SVDreuse = 1;
    printf("done\n");
    fflush(stdout);

    printf("\n\n");

    for(kk=0; kk<NBmodes; kk++)
    {
		long cntdmout;
		long i;
		
        printf("\r Mode %5ld / %5ld       ", kk, NBmodes);
        fflush(stdout);

        // APPLY MODE TO DM
        data.image[IDdmin].md[0].write = 1;
        for(ii=0; ii<dmsize; ii++)
            data.image[IDdmin].array.F[ii] = ampl*data.image[IDmodes].array.F[kk*dmsize+ii];
        COREMOD_MEMORY_image_set_sempost_byID(IDdmin, -1);
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;

        // WAIT
        usleep(tlagus);


        // RECORD DM SHAPES INTO MODES

        // POSITIVE
        cntdmout = 0;
        i = 0;
        while(i<NBave)
        {
            while(cntdmout==data.image[IDdmout].md[0].cnt0)
                usleep(20);

            cntdmout =  data.image[IDdmout].md[0].cnt0;


            memcpy(data.image[IDdmtmp].array.F, data.image[IDdmout].array.F, sizeof(float)*dmsize);
            memcpy(data.image[IDmeastmp].array.F, data.image[IDmeasout].array.F, sizeof(float)*dmsize);

            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(kk1=0; kk1<NBmodes; kk1++)
                data.image[IDcoeffarray].array.F[kk1*NBave+i] = 0.5*data.image[IDcoeff].array.F[kk1];
            delete_image_ID("dmcoeffs");

            linopt_imtools_image_fitModes("_tmpmeas", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(kk1=0; kk1<NBmodes; kk1++)
                data.image[IDcoeffarraymeas].array.F[kk1*NBave+i] = 0.5*data.image[IDcoeff].array.F[kk1];
            delete_image_ID("dmcoeffs");


            i++;
        }

        // NEGATIVE

        // APPLY MODE TO DM
        data.image[IDdmin].md[0].write = 1;
        for(ii=0; ii<dmsize; ii++)
            data.image[IDdmin].array.F[ii] = -ampl*data.image[IDmodes].array.F[kk*dmsize+ii];
        COREMOD_MEMORY_image_set_sempost_byID(IDdmin, -1);
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;

        // WAIT
        usleep(tlagus);

        cntdmout = 0;
        i = 0;
        while(i<NBave)
        {
            while(cntdmout==data.image[IDdmout].md[0].cnt0)
                usleep(20);

            cntdmout =  data.image[IDdmout].md[0].cnt0;

            memcpy(data.image[IDdmtmp].array.F, data.image[IDdmout].array.F, sizeof(float)*dmsize);
            memcpy(data.image[IDmeastmp].array.F, data.image[IDmeasout].array.F, sizeof(float)*dmsize);

            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(kk1=0; kk1<NBmodes; kk1++)
                data.image[IDcoeffarray].array.F[kk1*NBave+i] -= 0.5*data.image[IDcoeff].array.F[kk1];
            delete_image_ID("dmcoeffs");
            i++;

            linopt_imtools_image_fitModes("_tmpmeas", DMmodes_name, DMmask_name, SVDeps, "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(kk1=0; kk1<NBmodes; kk1++)
                data.image[IDcoeffarraymeas].array.F[kk1*NBave+i] = 0.5*data.image[IDcoeff].array.F[kk1];
            delete_image_ID("dmcoeffs");
        }


        // PROCESSS

        for(kk1=0; kk1<NBmodes; kk1++)
        {
            data.image[IDout].array.F[kk1*NBmodes+kk] = 0.0;
            data.image[IDoutrms].array.F[kk1*NBmodes+kk] = 0.0;
            data.image[IDoutmeas].array.F[kk1*NBmodes+kk] = 0.0;
            data.image[IDoutmeasrms].array.F[kk1*NBmodes+kk] = 0.0;
        }
        for(kk1=0; kk1<NBmodes; kk1++)
        {
            for(i=0; i<NBave; i++)
            {
                data.image[IDout].array.F[kk1*NBmodes+kk] += data.image[IDcoeffarray].array.F[kk1*NBave+i];
                data.image[IDoutrms].array.F[kk1*NBmodes+kk] += data.image[IDcoeffarray].array.F[kk1*NBave+i]*data.image[IDcoeffarray].array.F[kk1*NBave+i];
                data.image[IDoutmeas].array.F[kk1*NBmodes+kk] += data.image[IDcoeffarraymeas].array.F[kk1*NBave+i];
                data.image[IDoutmeasrms].array.F[kk1*NBmodes+kk] += data.image[IDcoeffarraymeas].array.F[kk1*NBave+i]*data.image[IDcoeffarraymeas].array.F[kk1*NBave+i];
            }
            data.image[IDout].array.F[kk1*NBmodes+kk] /= NBave*ampl;
            data.image[IDoutrms].array.F[kk1*NBmodes+kk] = sqrt(data.image[IDoutrms].array.F[kk1*NBmodes+kk]/NBave);
            data.image[IDoutmeas].array.F[kk1*NBmodes+kk] /= NBave*ampl;
            data.image[IDoutmeasrms].array.F[kk1*NBmodes+kk] = sqrt(data.image[IDoutmeasrms].array.F[kk1*NBmodes+kk]/NBave);
        }
    }
    printf("\n\n");
    fflush(stdout);

    delete_image_ID("_tmpdm");
    delete_image_ID("_tmpmeas");
    delete_image_ID("_coeffarray");



    return IDout;
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
long AOloopControl_perfTest_mkTestDynamicModeSeq(const char *IDname_out, long NBpt, long NBmodes)
{
    long IDout;
    long xsize, ysize, xysize;
    long ii, kk;
    float ampl0;
    float ampl;
    float pha0;
    char name[200];
    long m;

    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoconfID_DMmodes==-1)
    {
        if(sprintf(name, "aol%ld_DMmodes", LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoconfID_DMmodes = read_sharedmem_image(name);
    }
    xsize = data.image[aoconfID_DMmodes].md[0].size[0];
    ysize = data.image[aoconfID_DMmodes].md[0].size[1];
    xysize = xsize*ysize;

    IDout = create_3Dimage_ID(IDname_out, xsize, ysize, NBpt);

    for(kk=0; kk<NBpt; kk++)
    {
        for(ii=0; ii<xysize; ii++)
            data.image[IDout].array.F[kk*xysize+ii] = 0.0;

        for(m=0; m<NBmodes; m++)
        {
            ampl0 = 1.0;
            pha0 = M_PI*(1.0*m/NBmodes);
            ampl = ampl0 * sin(2.0*M_PI*(1.0*kk/NBpt)+pha0);
            for(ii=0; ii<xysize; ii++)
                data.image[IDout].array.F[kk*xysize+ii] += ampl * data.image[aoconfID_DMmodes].array.F[m*xysize+ii];
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


