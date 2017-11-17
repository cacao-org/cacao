/**
 * @file    AOloopControl_PredictiveControl.c
 * @brief   Adaptive Optics Control loop engine Predictive Control
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    25 Aug 2017
 *
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

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c














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
	init_AOloopControl_PredictiveControl();
	printf(" ...... Loading module %s\n", __FILE__);
}


int_fast8_t init_AOloopControl_PredictiveControl()
{
    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "cacao   - AO loop control predictive control");
    data.NBmodule++;



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
int_fast8_t AOloopControl_PredictiveControl_mapPredictiveFilter(const char *IDmodecoeff_name, long modeout, double delayfr)
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
double AOloopControl_PredictiveControl_testPredictiveFilter(const char *IDtrace_name, long modeout, double delayfr, long filtsize, const char *IDfilt_name, double SVDeps)
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







long AOloopControl_PredictiveControl_builPFloop_WatchInput(long loop, long PFblock, long PFblockStart, long PFblockEnd)
{
    long IDinb0;
    long IDinb1;
    char imnameb0[500];
    char imnameb1[500];
    long cnt0, cnt1;
    long cnt0_old, cnt1_old;
    long IDinb;

    long twaitus = 100000; // 0.1 sec


    long PFblockSize;
    long PFblockOrder;
    float PFblockLag;
    float PFblockdgain;
    FILE *fp;
    char fname[500];
    int ret;

    int Tupdate = 0;
    time_t t;
    struct tm *uttime;
    struct timespec timenow;
    long xsize, ysize, zsize, xysize;
    int cube;

    long IDout;
    uint32_t *imsizearray;
    uint8_t atype;
    char imnameout[500];
    long ii, kk;
    long ave;

    char inmaskname[200];
    char inmaskfname[200];
    char outmaskfname[200];
    long IDinmask;


    PFblockSize = PFblockEnd - PFblockStart;


    if(sprintf(imnameb0, "aol%ld_modeval_ol_logbuff0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(imnameb1, "aol%ld_modeval_ol_logbuff1", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDinb0 = read_sharedmem_image(imnameb0);
    IDinb1 = read_sharedmem_image(imnameb1);

    cnt0_old = data.image[IDinb0].md[0].cnt0;
    cnt1_old = data.image[IDinb1].md[0].cnt0;

    xsize = data.image[IDinb0].md[0].size[0];
    ysize = data.image[IDinb0].md[0].size[1];
    xysize = xsize*ysize;
    zsize = data.image[IDinb0].md[0].size[2];
    atype = data.image[IDinb0].md[0].atype;


    list_image_ID();


    if(system("mkdir -p PredictiveControl") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    if(sprintf(inmaskname, "inmaskPFb%ld", PFblock) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDinmask = create_2Dimage_ID(inmaskname, xysize, 1);
    for(ii=0; ii<xysize; ii++)
        data.image[IDinmask].array.F[ii] = 0.0;
    for(ii=PFblockStart; ii<PFblockEnd; ii++)
        data.image[IDinmask].array.F[ii] = 1.0;

    if(sprintf(inmaskfname, "!./PredictiveControl/inmaskPF%ld.fits", PFblock) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    save_fits(inmaskname, inmaskfname);
    if(sprintf(outmaskfname, "!./PredictiveControl/outmaskPF%ld.fits", PFblock) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    save_fits(inmaskname, outmaskfname);





    printf("Create aol%ld_modevalol_PFb%ld  : %ld x 1 x %ld\n", loop, PFblock, PFblockSize, zsize);
    fflush(stdout);
    imsizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);
    imsizearray[0] = PFblockSize;
    imsizearray[1] = 1;
    imsizearray[2] = zsize;

    if(sprintf(imnameout, "aol%ld_modevalol_PFb%ld", loop, PFblock) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDout = create_image_ID(imnameout, 3, imsizearray, atype, 1, 1);
    free(imsizearray);
    COREMOD_MEMORY_image_set_semflush(imnameout, -1);
    printf("Done\n");
    fflush(stdout);


    for(;;)
    {
        cnt0 = data.image[IDinb0].md[0].cnt0;
        cnt1 = data.image[IDinb1].md[0].cnt0;

        if(cnt0!=cnt0_old)
        {
            cube = 0;
            cnt0_old = cnt0;
            IDinb = IDinb0;
            Tupdate = 1;
        }

        if(cnt1!=cnt1_old)
        {
            cube = 1;
            cnt1_old = cnt1;
            IDinb = IDinb1;
            Tupdate = 1;
        }

        if(Tupdate == 1)
        {
            t = time(NULL);
            uttime = gmtime(&t);
            clock_gettime(CLOCK_REALTIME, &timenow);
            printf("%02d:%02d:%02ld.%09ld  NEW TELEMETRY BUFFER AVAILABLE [%d]\n", uttime->tm_hour, uttime->tm_min, timenow.tv_sec % 60, timenow.tv_nsec, cube);


            data.image[IDout].md[0].write = 1;

            for(kk=0; kk<zsize; kk++)
                for(ii=0; ii<PFblockSize; ii++)
                    data.image[IDout].array.F[kk*PFblockSize + ii] = data.image[IDinb].array.F[kk*xysize + (ii+PFblockStart)];

            for(ii=0; ii<PFblockSize; ii++)
            {
                ave = 0.0;
                for(kk=0; kk<zsize; kk++)
                    ave += data.image[IDout].array.F[kk*PFblockSize + ii];

                ave /= zsize;
                for(kk=0; kk<zsize; kk++)
                    data.image[IDout].array.F[kk*PFblockSize + ii] -= ave;
            }


            COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
            data.image[IDout].md[0].cnt0++;
            data.image[IDout].md[0].write = 0;

            Tupdate = 0;
        }


        usleep(twaitus);
    }

    return (IDout);
}





/**
 * 
 * DecayCoeff is betweeen 0 and 1
 * 1 : no decay, pure average
 * 
 * This is used to give more weigth to most recent measurements
 * 
 */
long AOloopControl_PredictiveControl_setPFsimpleAve(char *IDPF_name, float DecayCoeff)
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
