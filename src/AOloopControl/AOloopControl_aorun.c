/**
 * @file    AOloopControl_aorun.c 
 * @brief   AO loop Control compute functions 
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * @author  O. Guyon
 * @date    2018-01-04
 *
 * 
 * @bug No known bugs.
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

#ifdef __MACH__   // for Mac OS X - 
//#include <mach/mach_time.h>
//#define CLOCK_REALTIME 0
//#define CLOCK_MONOTONIC 0
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


#include <string.h>
#include <sched.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <pthread.h>
#include "info/info.h" 

//libraries created by O. Guyon 
#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif

#define NB_AOloopcontrol 10 // max number of loops

static int AOlooploadconf_init = 0;

// TIMING
static struct timespec tnow;
static struct timespec tdiff;

static double tdiffv;
static double tdiffv02;








#define AOconfname "/tmp/AOconf.shm"
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;















/**
 * ## Purpose
 * 
 * Main AO loop function
 * 
 * 
 * ## Details
 * 
 */ 
int_fast8_t __attribute__((hot)) AOloopControl_run()
{
    FILE *fp;
    char fname[200];
    long loop;
    int vOK;
    long ii;
    long ID;
    long j, m;
    struct tm *uttime;
    time_t t;
    struct timespec *thetime = (struct timespec *)malloc(sizeof(struct timespec));
    char logfname[1000];
    char command[1000];
    int r;
    int RT_priority = 90; //any number from 0-99
    struct sched_param schedpar;
    double a;
    long cnttest;
    float tmpf1;


    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    int semval;


	struct timespec functionTestTimerStart;
	struct timespec functionTestTimerEnd;

	struct timespec functionTestTimer00;
	struct timespec functionTestTimer01;
	struct timespec functionTestTimer02;
	struct timespec functionTestTimer03;
	struct timespec functionTestTimer04;




    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    r = seteuid(data.euid); //This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid);//Go back to normal privileges
#endif

    loop = aoloopcontrol_var.LOOPNUMBER;
    
    
	// LOG function start
	int logfunc_level = 0;
	int logfunc_level_max = 1;
	char commentstring[200];
	sprintf(commentstring, "Main function, loop %ld", loop);
	CORE_logFunctionCall( logfunc_level, logfunc_level_max, 0, __FILE__, __func__, __LINE__, commentstring);
	
	

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(0);


	/** ### STEP 1: Setting up 
	 * 
	 * Load arrays
	 * */
    printf("SETTING UP...\n");
    AOloopControl_loadconfigure(aoloopcontrol_var.LOOPNUMBER, 1, 10);

	
    // pixel streaming ?
    aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 1;

    if(AOconf[loop].GPUall == 0)
        aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;


    printf("============ pixel streaming ? =============\n");
    fflush(stdout);

    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING == 1)
        aoloopcontrol_var.aoconfID_pixstream_wfspixindex = load_fits("pixstream_wfspixindex.fits", "pixstream", 1);

    if(aoloopcontrol_var.aoconfID_pixstream_wfspixindex == -1)
        aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;
    else
    {
        printf("Testing data type\n");
        fflush(stdout);
        if(data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].atype != _DATATYPE_UINT16)
            aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;
    }

    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING == 1)
    {
        long xsize = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].size[0];
        long ysize = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].size[1];
        aoloopcontrol_var.PIXSTREAM_NBSLICES = 0;
        for(ii=0; ii<xsize*ysize; ii++)
            if(data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii] > aoloopcontrol_var.PIXSTREAM_NBSLICES)
                aoloopcontrol_var.PIXSTREAM_NBSLICES = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii];
        aoloopcontrol_var.PIXSTREAM_NBSLICES++;
        printf("PIXEL STREAMING:   %d image slices\n", aoloopcontrol_var.PIXSTREAM_NBSLICES);
    }


    printf("============ FORCE pixel streaming = 0\n");
    fflush(stdout);
    aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0; // TEST


    printf("GPU0 = %d\n", AOconf[loop].GPU0);
    if(AOconf[loop].GPU0>1)
    {
        uint8_t k;
        for(k=0; k<AOconf[loop].GPU0; k++)
            printf("stream %2d      aoloopcontrol_var.GPUset0 = %2d\n", (int) k, aoloopcontrol_var.GPUset0[k]);
    }

    printf("GPU1 = %d\n", AOconf[loop].GPU1);
    if(AOconf[loop].GPU1>1)
    {
        uint8_t k;
        for(k=0; k<AOconf[loop].GPU1; k++)
            printf("stream %2d      aoloopcontrol_var.GPUset1 = %2d\n", (int) k, aoloopcontrol_var.GPUset1[k]);
    }



    vOK = 1;
    if(AOconf[loop].init_wfsref0==0)
    {
        printf("ERROR: CANNOT RUN LOOP WITHOUT WFS REFERENCE\n");
        vOK = 0;
    }
//    if(AOconf[loop].init_CM==0)
    if(aoloopcontrol_var.init_CM_local==0)
    {
        printf("ERROR: CANNOT RUN LOOP WITHOUT CONTROL MATRIX\n");
        printf("aoloopcontrol_var.init_CM_local = 0\n");
        printf("FILE %s  line %d\n", __FILE__, __LINE__);
        vOK = 0;
    }

	aoloopcontrol_var.aoconfcnt0_wfsref_current = data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0;
	//aoconfcnt0_contrM_current = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0;

    AOconf[loop].initmapping = 0;
    AOconf[loop].init_CMc = 0;
    clock_gettime(CLOCK_REALTIME, &t1);


    if(vOK==1)
    {
		AOconf[loop].LOOPiteration = 0;
        AOconf[loop].kill = 0;
        AOconf[loop].on = 0;
        AOconf[loop].DMprimaryWriteON = 0;
        AOconf[loop].DMfilteredWriteON = 0;
        AOconf[loop].ARPFon = 0;
        
        #ifdef _PRINT_TEST
		printf("[%s] [%d]  AOloopControl_run: Entering loop\n", __FILE__, __LINE__);
		fflush(stdout);
		#endif

        int timerinit = 0;

        while( AOconf[loop].kill == 0)
        {
            if(timerinit==1)
            {
                clock_gettime(CLOCK_REALTIME, &t1);
                printf("timer init\n");
            }
            clock_gettime(CLOCK_REALTIME, &t2);

            tdiff = info_time_diff(t1, t2);
            double tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;

            printf(" WAITING     %20.3lf sec         \r", tdiffv);
            fflush(stdout);
            usleep(1000);


            timerinit = 0;
            while(AOconf[loop].on == 1)
            {
				clock_gettime(CLOCK_REALTIME, &functionTestTimer00); //TEST timing in function
                if(timerinit==0)
                {
                    //      Read_cam_frame(loop, 0, AOconf[loop].WFSnormalize, 0, 1);
                    clock_gettime(CLOCK_REALTIME, &t1);
                    timerinit = 1;
                }
                
            #ifdef _PRINT_TEST
			printf("[%s] [%d]  AOloopControl_run: Starting AOcompute, AOconf[%d].WFSnormalize = %d\n", __FILE__, __LINE__, loop, AOconf[loop].WFSnormalize);
			fflush(stdout);
			#endif
                

                AOcompute(loop, AOconf[loop].WFSnormalize);
                
				clock_gettime(CLOCK_REALTIME, &functionTestTimerStart); //TEST timing in function


                AOconf[loop].status = 12; // 12
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[19] = tdiffv;


                if(AOconf[loop].CMMODE==0)  // 2-step : WFS -> mode coeffs -> DM act
                {
                    if(AOconf[loop].DMprimaryWriteON==1) // if Writing to DM
                    {


                        if(fabs(AOconf[loop].gain)>1.0e-6)
                            set_DM_modes(loop);
                    }

                }
                else // 1 step: WFS -> DM act
                {
                    if(AOconf[loop].DMprimaryWriteON==1) // if Writing to DM
                    {
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 1;

                        for(ii=0; ii<AOconf[loop].sizeDM; ii++)//TEST
                        {
                            if(isnan(data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii])!=0)
                            {
                                printf("image aol2_meas_act  element %ld is NAN -> replacing by 0\n", ii);
                                data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii] = 0.0;
                            }
                        }


                        AOconf[loop].status = 13; // enforce limits
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[20] = tdiffv;


                        for(ii=0; ii<AOconf[loop].sizeDM; ii++)
                        {
                            data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] -= AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii];

                            data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] *= AOconf[loop].mult;

                            if(data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] > AOconf[loop].maxlimit)
                                data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] = AOconf[loop].maxlimit;
                            if(data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] < -AOconf[loop].maxlimit)
                                data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] = -AOconf[loop].maxlimit;
                        }


                        AOconf[loop].status = 14; // write to DM
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[21] = tdiffv;



                      /*  int semnb;
                        for(semnb=0; semnb<data.image[aoloopcontrol_var.aoconfID_dmC].md[0].sem; semnb++)
                        {
                            sem_getvalue(data.image[aoloopcontrol_var.aoconfID_dmC].semptr[semnb], &semval);
                            if(semval<SEMAPHORE_MAXVAL)
                                sem_post(data.image[aoloopcontrol_var.aoconfID_dmC].semptr[semnb]);
                        }*/
                        
                        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_dmC, -1);
						data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = AOconf[loop].LOOPiteration;
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0++;
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 0;
                        // inform dmdisp that new command is ready in one of the channels
                        if(aoloopcontrol_var.aoconfID_dmdisp!=-1)
                            if(data.image[aoloopcontrol_var.aoconfID_dmdisp].md[0].sem > 1)
                            {
                                sem_getvalue(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[0], &semval);
                                if(semval<SEMAPHORE_MAXVAL)
                                    sem_post(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[1]);
                            }
                        AOconf[loop].DMupdatecnt ++;
                    }
                }

                AOconf[loop].status = 18; // 18
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[22] = tdiffv;

                AOconf[loop].cnt++;

		
				AOconf[loop].LOOPiteration++;
				data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1 = AOconf[loop].LOOPiteration;
				
				
				
				
				// REAL TIME LOGGING
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt0 = AOconf[loop].cnt;
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt1 = AOconf[loop].LOOPiteration;
                data.image[aoloopcontrol_var.aoconfIDlogdata].array.F[0] = AOconf[loop].gain;


                if(AOconf[loop].cnt == AOconf[loop].cntmax)
                    AOconf[loop].on = 0;
            
				clock_gettime(CLOCK_REALTIME, &functionTestTimerEnd);
				
				
				tdiff = info_time_diff(functionTestTimerStart, functionTestTimerEnd);
				tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
				tdiffv02 = tdiffv;
				//TEST TIMING
				/*
				if(tdiffv > 30.0e-6)
				{
					printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_run() - excluding AOcompute\n", tdiffv*1.0e6, (long) AOconf[loop].LOOPiteration);
					fflush(stdout);
				}*/
				
				tdiff = info_time_diff(functionTestTimer00, functionTestTimerEnd);
				tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
				
				//TEST TIMING
				/*
				if(tdiffv > 600.0e-6)
				{
					printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_run()\n", tdiffv*1.0e6, (long) AOconf[loop].LOOPiteration);
					printf("    AOcompute()            read cam        : %12.3f us \n", tdiffv00*1.0e6);
					printf("    AOcompute()            post read cam   : %12.3f us \n", tdiffv01*1.0e6);
					printf("    AOloopControl_run()    post-AOcompute  : %12.3f us \n", tdiffv02*1.0e6);
					
					fflush(stdout);
					
					
				}
				*/ 
            
            }

        }
    }

    free(thetime);

	// LOG function end
	CORE_logFunctionCall( logfunc_level, logfunc_level_max, 1, __FILE__, __func__, __LINE__, commentstring);

    return(0);
}











int_fast8_t AOloopControl_CompModes_loop(const char *ID_CM_name, const char *ID_WFSref_name, const char *ID_WFSim_name, const char *ID_WFSimtot_name, const char *ID_coeff_name)
{
#ifdef HAVE_CUDA

    int *GPUsetM;
    long ID_CM;
    long ID_WFSref;
    long ID_coeff;
    long GPUcnt;
    int k;
    int_fast8_t GPUstatus[100];
    int_fast8_t status;
    long NBmodes;
    uint32_t *sizearray;

    long ID_WFSim;
    long ID_WFSim_n;
    long wfsxsize, wfsysize;
    int m;
    long IDcoeff0;

    long ID_WFSimtot;
    double totfluxave;
    long ID_coefft;

    double alpha = 0.1;
	char imname[200];



	// LOG function start
	int logfunc_level = 0;
	int logfunc_level_max = 1;
	char commentstring[200];
	sprintf(commentstring, "loop %ld", aoloopcontrol_var.LOOPNUMBER);
	CORE_logFunctionCall( logfunc_level, logfunc_level_max, 0, __FILE__, __func__, __LINE__, commentstring);
	
	
	
	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    GPUcnt = 2;

    GPUsetM = (int*) malloc(sizeof(int)*GPUcnt);
    for(k=0; k<GPUcnt; k++)
        GPUsetM[k] = k+5;


    ID_CM = image_ID(ID_CM_name);
    wfsxsize = data.image[ID_CM].md[0].size[0];
    wfsysize = data.image[ID_CM].md[0].size[1];
    NBmodes = data.image[ID_CM].md[0].size[2];

    ID_WFSref = image_ID(ID_WFSref_name);


    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = NBmodes;
    sizearray[1] = 1;

    ID_coeff = create_image_ID(ID_coeff_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(ID_coeff_name, 10);
    data.image[ID_coeff].md[0].cnt0 = 0;

    ID_coefft = create_image_ID("coefftmp", 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem("coefftmp", 10);


    IDcoeff0 = create_image_ID("coeff0", 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    ID_WFSim_n = create_2Dimage_ID("wfsim_n", wfsxsize, wfsysize);
    COREMOD_MEMORY_image_set_createsem("wfsim_n", 10);




    ID_WFSim = image_ID(ID_WFSim_name);
    ID_WFSimtot = image_ID(ID_WFSimtot_name);


    GPU_loop_MultMat_setup(2, ID_CM_name, "wfsim_n", "coefftmp", GPUcnt, GPUsetM, 0, 1, 1, 0);

    totfluxave = 1.0;
    int initWFSref;
    for(;;)
    {
        if(initWFSref==0)
        {
            printf("Computing reference\n");
            fflush(stdout);
            memcpy(data.image[ID_WFSim_n].array.F, data.image[ID_WFSref].array.F, sizeof(float)*wfsxsize*wfsysize);
            GPU_loop_MultMat_execute(2, &status, &GPUstatus[0], 1.0, 0.0, 0, 0);
            for(m=0; m<NBmodes; m++)
            {
                data.image[IDcoeff0].array.F[m] = data.image[ID_coefft].array.F[m];
            }
            printf("\n");
            initWFSref = 1;
            printf("reference computed\n");
            fflush(stdout);
        }

        memcpy(data.image[ID_WFSim_n].array.F, data.image[ID_WFSim].array.F, sizeof(float)*wfsxsize*wfsysize);
        COREMOD_MEMORY_image_set_semwait(ID_WFSim_name, 0);

        GPU_loop_MultMat_execute(2, &status, &GPUstatus[0], 1.0, 0.0, 0, 0);
        totfluxave = (1.0-alpha)*totfluxave + alpha*data.image[ID_WFSimtot].array.F[0];

        data.image[ID_coeff].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[ID_coeff].array.F[m] = data.image[ID_coefft].array.F[m]/totfluxave - data.image[IDcoeff0].array.F[m];
        data.image[ID_coeff].md[0].cnt0 ++;
        data.image[ID_coeff].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[ID_coeff].md[0].write = 0;
    }


    delete_image_ID("coeff0");
    free(sizearray);

    free(GPUsetM);


	// LOG function start
	CORE_logFunctionCall( logfunc_level, logfunc_level_max, 1, __FILE__, __func__, __LINE__, commentstring);


#endif

    return(0);
}







//
// gains autotune
//
// input: modeval_ol
// APPLIES new gain values if AUTOTUNE_GAINS_ON
//
int_fast8_t AOloopControl_AutoTuneGains(long loop, const char *IDout_name, float GainCoeff, long NBsamples)
{
    long IDmodevalOL;
    long IDmodeval;
    long IDmodeval_dm;
    long IDmodeval_dm_now;
    long IDmodeval_dm_now_filt;
	long IDmodeWFSnoise;




    long NBmodes;
    char imname[200];
    long m;
    double diff1, diff2, diff3, diff4;
    float *array_mvalOL1;
    float *array_mvalOL2;
    float *array_mvalOL3;
    float *array_mvalOL4;
    double *array_sig1;
    double *array_sig2;
    double *array_sig3;
    double *array_sig4;
    float *array_sig;
    float *array_asq;
    long double *ave0;
    long double *sig0;
    long double *sig1;
    long double *sig2;
    long double *sig3;
    long double *sig4;
    float *stdev;

    float gain;
    long NBgain;
    long kk;
    float *errarray;
    float mingain = 0.01;
    float maxgain = 0.3;
    float gainFactor = 0.6; // advise user to be at 60% of optimal gain
    float gainfactstep = 1.02;
    float *gainval_array;
    float *gainval1_array;
    float *gainval2_array;

    long long cnt = 0;
    float latency;
    FILE *fp;

    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;


    long IDout;
    uint32_t *sizearray;

    float gain0; // corresponds to evolution timescale
    long cntstart;



    long IDblk;
    float *modegain;
    float *modemult;


    float *NOISEfactor;
    long IDsync;


    int TESTMODE = 0;
    int TEST_m = 30;
    FILE *fptest;
    
    long iter;
    float GainCoeff1 = 1.0;
    
    


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


    printf("AUTO GAIN\n");
    fflush(stdout);


    // read AO loop gain, mult
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


	AOconf[loop].AUTOTUNEGAINS_updateGainCoeff = GainCoeff;
	AOconf[loop].AUTOTUNEGAINS_NBsamples = NBsamples;



    gain0 = 1.0/(AOconf[loop].loopfrequ*AOconf[loop].AUTOTUNEGAINS_evolTimescale);




    // CONNECT to arrays holding gain, limit, and multf values for blocks

    if(aoloopcontrol_var.aoconfID_gainb == -1)
    {
        if(sprintf(imname, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_gainb = read_sharedmem_image(imname);
    }

    if(aoloopcontrol_var.aoconfID_multfb == -1)
    {
        if(sprintf(imname, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_multfb = read_sharedmem_image(imname);
    }

    // CONNECT to arrays holding gain, limit and multf values for individual modes

    if(aoloopcontrol_var.aoconfID_DMmode_GAIN == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_GAIN", aoloopcontrol_var.LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(imname);
    }
    printf("aoloopcontrol_var.aoconfID_DMmode_GAIN = %ld\n", aoloopcontrol_var.aoconfID_DMmode_GAIN);

    if(aoloopcontrol_var.aoconfID_MULTF_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(imname);
    }



    // INPUT
    if(sprintf(imname, "aol%ld_modeval_ol", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalOL = read_sharedmem_image(imname);
    NBmodes = data.image[IDmodevalOL].md[0].size[0];

    if(sprintf(imname, "aol%ld_modeval", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm", loop) < 1) // measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm_now", loop) < 1) // current modal DM correction
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm_now = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_modeval_dm_now_filt", loop) < 1) // current modal DM correction, filtered
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm_now_filt = read_sharedmem_image(imname);



	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
	sizearray = (uint32_t *) malloc(sizeof(uint32_t)*2);
	sizearray[0] = NBmodes;
	sizearray[1] = 1;
	if(sprintf(imname, "aol%ld_modeWFSnoise", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodeWFSnoise = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	free(sizearray);

    // blocks
    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) // block indices
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblk = read_sharedmem_image(imname);


    modegain = (float*) malloc(sizeof(float)*NBmodes);
    modemult = (float*) malloc(sizeof(float)*NBmodes);
    NOISEfactor = (float*) malloc(sizeof(float)*NBmodes);





    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);
    sizearray[0] = NBmodes;
    sizearray[1] = 1;
    IDout = create_image_ID(IDout_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDout_name, 10);
    free(sizearray);




    // last open loop move values
    array_mvalOL1 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL2 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL3 = (float*) malloc(sizeof(float)*NBmodes);
    array_mvalOL4 = (float*) malloc(sizeof(float)*NBmodes);
    array_sig1 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig2 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig3 = (double*) malloc(sizeof(double)*NBmodes);
    array_sig4 = (double*) malloc(sizeof(double)*NBmodes);

    array_sig = (float*) malloc(sizeof(float)*NBmodes);
    array_asq = (float*) malloc(sizeof(float)*NBmodes);
    ave0 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig0 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig1 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig2 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig3 = (long double*) malloc(sizeof(long double)*NBmodes);
    sig4 = (long double*) malloc(sizeof(long double)*NBmodes);
    stdev = (float*) malloc(sizeof(float)*NBmodes);


    gainval_array = (float*) malloc(sizeof(float)*NBgain);
    gainval1_array = (float*) malloc(sizeof(float)*NBgain);
    gainval2_array = (float*) malloc(sizeof(float)*NBgain);

    errarray = (float*) malloc(sizeof(float)*NBgain);



	iter = 0;
    for(;;)
    {

        // write gain, mult into arrays
        for(m=0; m<NBmodes; m++)
        {
            unsigned short block;

            block = data.image[IDblk].array.UI16[m];
            modegain[m] = AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_gainb].array.F[block] * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m];
            modemult[m] = AOconf[loop].mult * data.image[aoloopcontrol_var.aoconfID_multfb].array.F[block] * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[m];
            NOISEfactor[m] = 1.0 + modemult[m]*modemult[m]*modegain[m]*modegain[m]/(1.0-modemult[m]*modemult[m]);
        }







        // prepare gain array
        latency = AOconf[loop].hardwlatency_frame + AOconf[loop].wfsmextrlatency_frame;
        //printf("latency = %f frame\n", latency);
        NBgain = 0;
        gain = mingain;
        while(gain<maxgain/gainFactor)
        {
            gain *= gainfactstep;
            NBgain++;
        }


        kk = 0;
        gain = mingain;
        while(kk<NBgain)
        {
            gainval_array[kk] = gain;
            gainval1_array[kk] = (latency + 1.0/gain)*(latency + 1.0/(gain+gain0));
            gainval2_array[kk] = (gain/(1.0-gain));

            //printf("gain   %4ld  %12f   %12f  %12f\n", kk, gainval_array[kk], gainval1_array[kk], gainval2_array[kk]);
            gain *= gainfactstep;
            kk++;
        }

        // drive sem5 to zero
        while(sem_trywait(data.image[IDmodevalOL].semptr[5])==0) {}



        for(m=0; m<NBmodes; m++)
        {
            array_mvalOL1[m] = 0.0;
            array_mvalOL2[m] = 0.0;
            array_sig1[m] = 0.0;
            array_sig2[m] = 0.0;
            ave0[m] = 0.0;
            sig0[m] = 0.0;
            sig1[m] = 0.0;
            sig2[m] = 0.0;
            sig3[m] = 0.0;
            sig4[m] = 0.0;
            stdev[m] = 0.0;
        }


        if(TESTMODE==1)
            fptest = fopen("test_autotunegain.dat", "w");

        cnt = 0;
        cntstart = 10;
        while(cnt<AOconf[loop].AUTOTUNEGAINS_NBsamples)
        {
            sem_wait(data.image[IDmodevalOL].semptr[5]);


            data.image[IDout].md[0].write = 1;

            for(m=0; m<NBmodes; m++)
            {
                diff1 = data.image[IDmodevalOL].array.F[m] - array_mvalOL1[m];
                diff2 = data.image[IDmodevalOL].array.F[m] - array_mvalOL2[m];
                diff3 = data.image[IDmodevalOL].array.F[m] - array_mvalOL3[m];
                diff4 = data.image[IDmodevalOL].array.F[m] - array_mvalOL4[m];
                array_mvalOL4[m] = array_mvalOL3[m];
                array_mvalOL3[m] = array_mvalOL2[m];
                array_mvalOL2[m] = array_mvalOL1[m];
                array_mvalOL1[m] = data.image[IDmodevalOL].array.F[m];

                if(cnt>cntstart)
                {
                    ave0[m] += data.image[IDmodevalOL].array.F[m];
                    sig0[m] += data.image[IDmodevalOL].array.F[m]*data.image[IDmodevalOL].array.F[m];
                    sig1[m] += diff1*diff1;
                    sig2[m] += diff2*diff2;
                    sig3[m] += diff3*diff3;
                    sig4[m] += diff4*diff4;
                }
            }

            if(TESTMODE==1)
                fprintf(fptest, "%5lld %+12.10f %+12.10f %+12.10f %+12.10f %+12.10f\n", cnt, data.image[IDmodeval].array.F[TEST_m], data.image[IDmodevalOL].array.F[TEST_m], data.image[IDmodeval_dm].array.F[TEST_m], data.image[IDmodeval_dm_now].array.F[TEST_m], data.image[IDmodeval_dm_now_filt].array.F[TEST_m]);

            cnt++;
        }
        if(TESTMODE==1)
            fclose(fptest);

		

		GainCoeff1 = 1.0/(iter+1);
		if(GainCoeff1 < AOconf[loop].AUTOTUNEGAINS_updateGainCoeff)
			GainCoeff1 = AOconf[loop].AUTOTUNEGAINS_updateGainCoeff;
		

        data.image[IDout].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
        {
            long kkmin;
            float errmin;

            ave0[m] /= cnt-cntstart;
            sig0[m] /= cnt-cntstart;
            array_sig1[m] = sig1[m]/(cnt-cntstart);
            array_sig2[m] = sig2[m]/(cnt-cntstart);
            array_sig3[m] = sig3[m]/(cnt-cntstart);
            array_sig4[m] = sig4[m]/(cnt-cntstart);


            //		array_asq[m] = (array_sig2[m]-array_sig1[m])/3.0;
            //        array_asq[m] = (array_sig4[m]-array_sig1[m])/15.0;

            // This formula is compatible with astromgrid, which alternates between patterns
            array_asq[m] = (array_sig4[m]-array_sig2[m])/12.0;
            if(array_asq[m]<0.0)
                array_asq[m] = 0.0;

			// WFS variance
            //array_sig[m] = (4.0*array_sig1[m] - array_sig2[m])/6.0;
            // This formula is compatible with astromgrid, which alternates between patterns
            array_sig[m] = (4.0*array_sig2[m] - array_sig4[m])/6.0;

            stdev[m] = sig0[m] - NOISEfactor[m]*array_sig[m] - ave0[m]*ave0[m];
            if(stdev[m]<0.0)
                stdev[m] = 0.0;
            stdev[m] = sqrt(stdev[m]);

            for(kk=0; kk<NBgain; kk++)
                errarray[kk] = array_asq[m] * gainval1_array[kk] + array_sig[m] * gainval2_array[kk];

            errmin = errarray[0];
            kkmin = 0;

            for(kk=0; kk<NBgain; kk++)
                if(errarray[kk]<errmin)
                {
                    errmin = errarray[kk];
                    kkmin = kk;
                }
			
			
            data.image[IDout].array.F[m] = (1.0-GainCoeff1) * data.image[IDout].array.F[m]   +  GainCoeff1 * (gainFactor*gainval_array[kkmin]);
            
        }

        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[IDout].md[0].write = 0;


		// write noise
		data.image[IDmodeWFSnoise].md[0].write = 1;
		data.image[IDmodeWFSnoise].md[0].cnt0++;
		for(m=0;m<NBmodes;m++)
			data.image[IDmodeWFSnoise].array.F[m] = array_sig[m];
		COREMOD_MEMORY_image_set_sempost_byID(IDmodeWFSnoise, -1);
		data.image[IDmodeWFSnoise].md[0].cnt1 = AOconf[loop].LOOPiteration;
		data.image[IDmodeWFSnoise].md[0].write = 0;


        if(AOconf[loop].AUTOTUNE_GAINS_ON==1) // automatically adjust gain values
        {

        }


        fp = fopen("optgain.dat", "w");
        for(m=0; m<NBmodes; m++)
            fprintf(fp, "%5ld   %+12.10f %12.10f %12.10f %12.10f %12.10f   %6.4f  %16.14f %16.14f  %6.2f\n", m, (float) ave0[m], (float) sig0[m], stdev[m], sqrt(array_asq[m]), sqrt(array_sig[m]), data.image[IDout].array.F[m], array_sig1[m], array_sig4[m], NOISEfactor[m]);
        fclose(fp);
        
        printf("[%8ld]  %8ld   %8.6f -> %8.6f\n", iter, AOconf[loop].AUTOTUNEGAINS_NBsamples, AOconf[loop].AUTOTUNEGAINS_updateGainCoeff, GainCoeff1);
        
        
        
     
        iter++;

    }

    free(gainval_array);
    free(gainval1_array);
    free(gainval2_array);
    free(errarray);

    free(array_mvalOL1);
    free(array_mvalOL2);
    free(array_mvalOL3);
    free(array_mvalOL4);
    free(ave0);
    free(sig0);
    free(sig1);
    free(sig2);
    free(sig3);
    free(sig4);
    free(array_sig1);
    free(array_sig2);
    free(array_sig3);
    free(array_sig4);
    free(array_sig);
    free(array_asq);

    free(modegain);
    free(modemult);
    free(NOISEfactor);

    free(stdev);

    return(0);
}



//
// assumes the WFS mode basis is already orthogonall
// removes reference from each frame
//
long AOloopControl_sig2Modecoeff(const char *WFSim_name, const char *IDwfsref_name, const char *WFSmodes_name, const char *outname)
{
    long IDout;
    long IDwfs, IDmodes, IDwfsref;
    long wfsxsize, wfsysize, wfssize, NBmodes, NBframes;
    double totref;
    float coeff;
    long ii, m, kk;
    FILE *fp;
    double *mcoeff_ave;
    double *mcoeff_rms;


    IDwfs = image_ID(WFSim_name);
    wfsxsize = data.image[IDwfs].md[0].size[0];
    wfsysize = data.image[IDwfs].md[0].size[1];
    NBframes = data.image[IDwfs].md[0].size[2];
    wfssize = wfsxsize*wfsysize;




    IDwfsref = image_ID(IDwfsref_name);

    IDmodes = image_ID(WFSmodes_name);
    NBmodes = data.image[IDmodes].md[0].size[2];

    mcoeff_ave = (double*) malloc(sizeof(double)*NBmodes);
    mcoeff_rms = (double*) malloc(sizeof(double)*NBmodes);



    IDout = create_2Dimage_ID(outname, NBframes, NBmodes);

    totref = 0.0;

    for(ii=0; ii<wfssize; ii++)
        totref += data.image[IDwfsref].array.F[ii];
    for(ii=0; ii<wfssize; ii++)
        data.image[IDwfsref].array.F[ii] /= totref;

    for(kk=0; kk<NBframes; kk++)
    {
		double totim = 0.0;
		
        for(ii=0; ii<wfssize; ii++)
            totim += data.image[IDwfs].array.F[kk*wfssize+ii];
        for(ii=0; ii<wfssize; ii++)
        {
            data.image[IDwfs].array.F[kk*wfssize+ii] /= totim;
            data.image[IDwfs].array.F[kk*wfssize+ii] -= data.image[IDwfsref].array.F[ii];
        }


        for(m=0; m<NBmodes; m++)
        {
            coeff = 0.0;
            for(ii=0; ii<wfssize; ii++)
                coeff += data.image[IDmodes].array.F[m*wfssize+ii] * data.image[IDwfs].array.F[kk*wfssize+ii];
            data.image[IDout].array.F[m*NBframes+kk] = coeff;
            mcoeff_ave[m] += coeff;
            mcoeff_rms[m] += coeff*coeff;
        }
    }


    fp  = fopen("mode_stats.txt", "w");
    for(m=0; m<NBmodes; m++)
    {
        mcoeff_rms[m] = sqrt( mcoeff_rms[m]/NBframes );
        mcoeff_ave[m] /= NBframes;
        fprintf(fp, "%4ld  %12g %12g\n", m, mcoeff_ave[m], mcoeff_rms[m]);
    }
    fclose(fp);

    free(mcoeff_ave);
    free(mcoeff_rms);

    return(IDout);
}
