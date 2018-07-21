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








extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;










static int AOloopControl_aorun_ProcessInit_Value = 0; // toggles to 1 when AOloopControl_aorun() started

// ********************************************************************
// This initialization runs once per process
// ********************************************************************

int AOloopControl_aorun_ProcessInit()
{
	
	
	
	return 0;
}







/**
 * ## Purpose
 * 
 * Main AO loop function
 * 
 * 
 * ## Details
 * 
 */ 
int_fast8_t __attribute__((hot)) AOloopControl_aorun()
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

	struct timespec tnow;
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


	if(AOloopControl_aorun_ProcessInit_Value == 0)
	{
		AOloopControl_aorun_ProcessInit();
		AOloopControl_aorun_ProcessInit_Value = 1;
	}



	/** ### STEP 1: Setting up 
	 * 
	 * Load arrays
	 * */
    printf("SETTING UP...\n");
    AOloopControl_loadconfigure(aoloopcontrol_var.LOOPNUMBER, 1, 10);





	
    // pixel streaming ?
    aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 1;

    if(AOconf[loop].AOcompute.GPUall == 0)
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
	

    AOconf[loop].initmapping = 0;
    AOconf[loop].init_CMc = 0;
    clock_gettime(CLOCK_REALTIME, &t1);


    if(vOK==1)
    {
		AOconf[loop].aorun.LOOPiteration = 0;
        AOconf[loop].aorun.kill = 0;
        AOconf[loop].aorun.on = 0;
        AOconf[loop].aorun.DMprimaryWriteON = 0;
        AOconf[loop].aorun.DMfilteredWriteON = 0;
        AOconf[loop].aorun.ARPFon = 0;
        
        #ifdef _PRINT_TEST
		printf("[%s] [%d]  AOloopControl_aorun: Entering loop\n", __FILE__, __LINE__);
		fflush(stdout);
		#endif

        int timerinit = 0;

        while( AOconf[loop].aorun.kill == 0)
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
            while(AOconf[loop].aorun.on == 1)
            {
				clock_gettime(CLOCK_REALTIME, &functionTestTimer00); //TEST timing in function
                if(timerinit==0)
                {
                    //      Read_cam_frame(loop, 0, AOconf[loop].WFSnormalize, 0, 1);
                    clock_gettime(CLOCK_REALTIME, &t1);
                    timerinit = 1;
                }
                
            #ifdef _PRINT_TEST
			printf("[%s] [%d]  AOloopControl_aorun: Starting AOcompute, AOconf[%d].WFSnormalize = %d\n", __FILE__, __LINE__, loop, AOconf[loop].WFSnormalize);
			fflush(stdout);
			#endif
                

                AOcompute(loop, AOconf[loop].WFSnormalize);
                
				clock_gettime(CLOCK_REALTIME, &functionTestTimerStart); //TEST timing in function


                AOconf[loop].status = 12; // 12
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[19] = tdiffv;


                if(AOconf[loop].aorun.CMMODE==0)  // 2-step : WFS -> mode coeffs -> DM act
                {
                    if(AOconf[loop].aorun.DMprimaryWriteON==1) // if Writing to DM
                    {


                        if(fabs(AOconf[loop].gain)>1.0e-6)
                            set_DM_modes(loop);
                    }

                }
                else // 1 step: WFS -> DM act
                {
                    if(AOconf[loop].aorun.DMprimaryWriteON==1) // if Writing to DM
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
						data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
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

		
				AOconf[loop].aorun.LOOPiteration++;
				data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
				
				
				
				
				// REAL TIME LOGGING
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt0 = AOconf[loop].cnt;
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
                data.image[aoloopcontrol_var.aoconfIDlogdata].array.F[0] = AOconf[loop].gain;


                if(AOconf[loop].cnt == AOconf[loop].cntmax)
                    AOconf[loop].aorun.on = 0;
            
				clock_gettime(CLOCK_REALTIME, &functionTestTimerEnd);
				
				
				tdiff = info_time_diff(functionTestTimerStart, functionTestTimerEnd);
				tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
				double tdiffv02 = tdiffv;
				//TEST TIMING
				/*
				if(tdiffv > 30.0e-6)
				{
					printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_aorun() - excluding AOcompute\n", tdiffv*1.0e6, (long) AOconf[loop].aorun.LOOPiteration);
					fflush(stdout);
				}*/
				
				tdiff = info_time_diff(functionTestTimer00, functionTestTimerEnd);
				tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
				
				//TEST TIMING
				/*
				if(tdiffv > 600.0e-6)
				{
					printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_aorun()\n", tdiffv*1.0e6, (long) AOconf[loop].aorun.LOOPiteration);
					printf("    AOcompute()            read cam        : %12.3f us \n", tdiffv00*1.0e6);
					printf("    AOcompute()            post read cam   : %12.3f us \n", tdiffv01*1.0e6);
					printf("    AOloopControl_aorun()    post-AOcompute  : %12.3f us \n", tdiffv02*1.0e6);
					
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























