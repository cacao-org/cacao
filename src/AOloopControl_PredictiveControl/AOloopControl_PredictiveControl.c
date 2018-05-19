/**
 * @file    AOloopControl_PredictiveControl.c
 * @brief   Adaptive Optics Control loop engine Predictive Control
 * 
 * Adaptive Optics predictive control
 *
 * 
 * ## Change log
 * - 20180518	Guyon	Improved documentation 
 * 
 *  
 * @author  O. Guyon
 *
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE




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
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"



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



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c


static int INITSTATUS_AOloopControl_PredictiveControl = 0;











/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL
 *  Predictive control using WFS telemetry */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_builPFloop_WatchInput */
int_fast8_t AOloopControl_PredictiveControl_builPFloop_WatchInput_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0) {
        AOloopControl_PredictiveControl_builPFloop_WatchInput(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl);
        return 0;
    } else return 1;
}

/** @brief CLI function for AOloopControl_mapPredictiveFilter */
int_fast8_t AOloopControl_PredictiveControl_mapPredictiveFilter_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,1)==0) {
        AOloopControl_PredictiveControl_mapPredictiveFilter(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_testPredictiveFilter */
int_fast8_t AOloopControl_PredictiveControl_testPredictiveFilter_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,1)+CLI_checkarg(4,2)+CLI_checkarg(5,3)==0) {
        AOloopControl_PredictiveControl_testPredictiveFilter(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf, data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.string, 1e-10);
        return 0;
    }
    else return 1;
}


int_fast8_t AOloopControl_PredictiveControl_setPFsimpleAve_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)==0) {
        AOloopControl_PredictiveControl_setPFsimpleAve(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf);
        return 0;
    }
    else return 1;
}






/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl functions */


void __attribute__ ((constructor)) libinit_AOloopControl_PredictiveControl()
{
	if(INITSTATUS_AOloopControl_PredictiveControl == 0)
	{
		init_AOloopControl_PredictiveControl();
		RegisterModule(__FILE__, "cacao", "AO loop control predictive control");
		INITSTATUS_AOloopControl_PredictiveControl = 1;
	}
}


int_fast8_t init_AOloopControl_PredictiveControl()
{

/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL                                                    */
/* =============================================================================================== */
/* =============================================================================================== */

    RegisterCLIcommand("aolPFwatchin",__FILE__, AOloopControl_PredictiveControl_builPFloop_WatchInput_cli, "watch telemetry for predictive filter input", "<loop #> <PFblock #> <start> <end>", "aolPFwatchin 0 2", "long AOloopControl_builPFloop_WatchInput(long loop, long PFblock, long PFblockStart, long PFblockEnd)");

    RegisterCLIcommand("aolmappfilt", __FILE__, AOloopControl_PredictiveControl_mapPredictiveFilter_cli, "map/search predictive filter", "<input coeffs> <mode number> <delay [frames]>", "aolmkapfilt coeffim 23 2.4", "long AOloopControl_mapPredictiveFilter(char *IDmodecoeff_name, long modeout, double delayfr)");

    RegisterCLIcommand("aolmkpfilt", __FILE__, AOloopControl_PredictiveControl_testPredictiveFilter_cli, "test predictive filter", "<trace im> <mode number> <delay [frames]> <filter size> <out filter name>", "aolmkpfilt traceim 23 2.4 20 filt23","long AOloopControl_testPredictiveFilter(char *IDtrace_name, long mode, double delayfr, long filtsize, char *IDfilt_name, double SVDeps)");


	RegisterCLIcommand("aolpfsetave", __FILE__, AOloopControl_PredictiveControl_setPFsimpleAve_cli, "set predictive filter to integrator", "<PredFilter> <DecayCoeff>", "aolpfsetave outPFb0 0.5", "long AOloopControl_PredictiveControl_setPFsimpleAve(char *IDPF_name, float DecayCoeff)");



    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}



























/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_PredictiveControl - 1. PREDICTIVE CONTROL                                                    */
/* =============================================================================================== */
/* =============================================================================================== */




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






///
/// predictive control based on SVD
///
/// input:
///     mode values trace  [ii: time, jj: mode number]
///     mode index
///     delayfr [delay in frame unit]
///     filtsize [number of samples in filter]
///
double AOloopControl_PredictiveControl_testPredictiveFilter(
	const char *IDtrace_name, 
	long modeout, 
	double delayfr, 
	long filtsize, 
	const char *IDfilt_name, 
	double SVDeps
	)
{
    long IDtrace;
    long IDmatA;
    long NBtraceVec; // number of measurement vectors in trace
    long NBmvec; // number of measurements in measurement matrix
    long NBch; // number of channels in measurement
    long IDmatC;
    long IDfilt;
    long l,m;
    float *marray; // measurement array
    FILE *fp;
    float tmpv;
    long delayfr_int;
    float delayfr_x;
    long ch, l1;
    double err0, err1;
    float v0;
    float NoiseAmpl = 0.02;



    IDtrace = image_ID(IDtrace_name);

    NBtraceVec = data.image[IDtrace].md[0].size[0];
    NBch = data.image[IDtrace].md[0].size[1];


    NBmvec = NBtraceVec - filtsize - (long) (delayfr+1.0);




    // build measurement matrix

    fp = fopen("tracepts.txt","w");
    IDmatA = create_2Dimage_ID("WFPmatA", NBmvec, filtsize*NBch);
    // each column is a measurement
    for(m=0; m<NBmvec; m++) // column index
    {
        fprintf(fp, "%5ld %f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout+m+filtsize]);
        for(l=0; l<filtsize; l++)
            for(ch=0; ch<NBch; ch++)
            {
                l1 = ch*filtsize + l;
                data.image[IDmatA].array.F[l1*NBmvec+m] = data.image[IDtrace].array.F[NBtraceVec*ch + (m+l)];
            }
    }
    fclose(fp);






    // build measurement vector
    delayfr_int = (int) delayfr;
    delayfr_x = delayfr - delayfr_int;
    printf("%f  = %ld + %f\n", delayfr, delayfr_int, delayfr_x);
    marray = (float*) malloc(sizeof(float)*NBmvec);
    fp = fopen("tracepts1.txt","w");
    for(m=0; m<NBmvec; m++)
    {
        marray[m] = data.image[IDtrace].array.F[NBtraceVec*modeout+(m+filtsize+delayfr_int)]*(1.0-delayfr_x) + data.image[IDtrace].array.F[NBtraceVec*modeout+(m+filtsize+delayfr_int+1)]*delayfr_x;
        fprintf(fp, "%5ld %f %f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout+m+filtsize], marray[m]);
    }
    fclose(fp);


    linopt_compute_SVDpseudoInverse("WFPmatA", "WFPmatC", SVDeps, 10000, "WFP_VTmat");

    save_fits("WFPmatA", "!WFPmatA.fits");
    save_fits("WFPmatC", "!WFPmatC.fits");
    IDmatC = image_ID("WFPmatC");

    IDfilt = create_2Dimage_ID(IDfilt_name, filtsize, NBch);
    for(l=0; l<filtsize; l++)
        for(ch=0; ch<NBch; ch++)
        {
            tmpv = 0.0;
            for(m=0; m<NBmvec; m++)
                tmpv += data.image[IDmatC].array.F[(ch*filtsize+l)*NBmvec+m] * marray[m];
            data.image[IDfilt].array.F[ch*filtsize+l] = tmpv;
        }

    fp = fopen("filt.txt", "w");
    tmpv = 0.0;
    for(l=0; l<filtsize; l++)
        for(ch=0; ch<NBch; ch++)
        {
            tmpv += data.image[IDfilt].array.F[ch*filtsize+l];
            fprintf(fp, "%3ld %3ld %f %f\n", ch, l, data.image[IDfilt].array.F[l], tmpv);
        }
    fclose(fp);
    printf("filter TOTAL = %f\n", tmpv);

    // TEST FILTER

    // col #1 : time index m
    // col #2 : value at index m
    // col #3 : predicted value at m+delay
    // col #4 : actual value at m+delay
    fp = fopen("testfilt.txt", "w");
    err0 = 0.0;
    err1 = 0.0;
    for(m=filtsize; m<NBtraceVec-(delayfr_int+1); m++)
    {
        tmpv = 0.0;
        for(l=0; l<filtsize; l++)
            for(ch=0; ch<NBch; ch++)
                tmpv += data.image[IDfilt].array.F[ch*filtsize+l]*data.image[IDtrace].array.F[NBtraceVec*ch + (m-filtsize+l)];

        fprintf(fp, "%5ld %20f %20f %20f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout + m], tmpv, marray[m-filtsize]);

        v0 = tmpv - marray[m-filtsize];
        err0 += v0*v0;

        v0 = data.image[IDtrace].array.F[NBtraceVec*modeout + m] - marray[m-filtsize];
        err1 += v0*v0;
    }
    fclose(fp);
    free(marray);

    err0 = sqrt(err0/(NBtraceVec-filtsize-(delayfr_int+1)));
    err1 = sqrt(err1/(NBtraceVec-filtsize-(delayfr_int+1)));
    printf("Prediction error (using optimal filter)   :   %f\n", err0);
    printf("Prediction error (using last measurement) :   %f\n", err1);

    return(err1);
}








/**
 * 
 * DecayCoeff is betweeen 0 and 1
 * 1 : no decay, pure average
 * 
 * This is used to give more weigth to most recent measurements
 * 
 */
long AOloopControl_PredictiveControl_setPFsimpleAve(
	char *IDPF_name, 
	float DecayCoeff
	)
{
	long IDPF;
	int xsize, ysize; 
	int FilterOrder;
	int ii, jj, kk; 
	float *coeff;
	float total;
	
	IDPF = image_ID(IDPF_name);
	xsize = data.image[IDPF].md[0].size[0];
	ysize = data.image[IDPF].md[0].size[1];
	FilterOrder = xsize/ysize;
	
	coeff = (float*) malloc(sizeof(float)*FilterOrder);
	
	// set up coeffs and compute their sum
	total = 0.0;
	for(kk=0;kk<FilterOrder;kk++)
	{
		coeff[kk] = pow(DecayCoeff, kk);
		total += coeff[kk];
	}
	// normalize such that sum of coeffs is 1
	for(kk=0;kk<FilterOrder;kk++)
		coeff[kk] /= total;
	
	
	printf("Filter order = %d\n", FilterOrder);
	for(kk=0;kk<FilterOrder;kk++)
	{
		for(ii=0;ii<ysize;ii++)
			for(jj=0;jj<ysize;jj++)
				{
					data.image[IDPF].array.F[jj*xsize + ii+kk*ysize] = 0.0;
				}
		for(ii=0;ii<ysize;ii++)
			data.image[IDPF].array.F[ii*xsize + ii+kk*ysize] = coeff[kk];
	}
	
	
	free(coeff);
	
	return(IDPF);
}
