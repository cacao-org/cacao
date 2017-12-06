/**
 * @file    AOloopControl_wfs_dm.c 
 * @brief   AO loop Control functions wave front sensor and deformable mirror 
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * @author  O. Guyon
 * @date    24 nov 2017
 *
 * 
 * @bug No known bugs.
 * 
 */



#define _GNU_SOURCE



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


#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif

extern DATA data;
#define NB_AOloopcontrol 10 // max number of loops
//long aoloopcontrol_var.LOOPNUMBER = 0; // current loop index


static int AOlooploadconf_init = 0;

// TIMING
static struct timespec tnow;
static struct timespec tdiff;
static double tdiffv;


#define AOconfname "/tmp/AOconf.shm"
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;

// zero point offset loop
//
// args:
//  DM offset channel (shared memory)
//  zonal resp matrix (shared memory)
//  nominal wfs reference without offset (shared memory)
//  wfs reference to be updated (shared memory)
//
// computation triggered on semaphore wait on semaphore #1 of DM offset
//
// will run until SIGUSR1 received
//
// read LOOPiteration from shared memory stream "aol#_LOOPiteration" if available 
//
int_fast8_t AOloopControl_WFSzpupdate_loop(const char *IDzpdm_name, const char *IDzrespM_name, const char *IDwfszp_name)
{
    long IDzpdm, IDzrespM, IDwfszp;
    uint32_t dmxsize, dmysize, dmxysize;
    long wfsxsize, wfsysize, wfsxysize;
    long IDtmp;
    long elem, act;
    long zpcnt = 0;
    long zpcnt0;
    int semval;
    struct timespec t1;
    struct timespec t2;

	char imname[200];
	

	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}

    IDzpdm = image_ID(IDzpdm_name);

    if(data.image[IDzpdm].md[0].sem<2) // if semaphore #1 does not exist, create it
        COREMOD_MEMORY_image_set_createsem(IDzpdm_name, 2);


    IDzrespM = image_ID(IDzrespM_name);
    IDwfszp = image_ID(IDwfszp_name);


    // array sizes extracted from IDzpdm and IDwfsref

    dmxsize = data.image[IDzpdm].md[0].size[0];
    dmysize = data.image[IDzpdm].md[0].size[1];
    dmxysize = dmxsize*dmysize;
    wfsxsize = data.image[IDwfszp].md[0].size[0];
    wfsysize = data.image[IDwfszp].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;

    // VERIFY SIZES

    // verify zrespM
    if(data.image[IDzrespM].md[0].size[0]!=wfsxsize)
    {
        printf("ERROR: zrespM xsize %ld does not match wfsxsize %ld\n", (long) data.image[IDzrespM].md[0].size[0], (long) wfsxsize);
        exit(0);
    }
    if(data.image[IDzrespM].md[0].size[1]!=wfsysize)
    {
        printf("ERROR: zrespM ysize %ld does not match wfsysize %ld\n", (long) data.image[IDzrespM].md[0].size[1], (long) wfsysize);
        exit(0);
    }
    if(data.image[IDzrespM].md[0].size[2]!=dmxysize)
    {
        printf("ERROR: zrespM zsize %ld does not match wfsxysize %ld\n", (long) data.image[IDzrespM].md[0].size[1], (long) wfsxysize);
        exit(0);
    }


    IDtmp = create_2Dimage_ID("wfsrefoffset", wfsxsize, wfsysize);


    zpcnt0 = 0;

    if(data.image[IDzpdm].md[0].sem > 1) // drive semaphore #1 to zero
        while(sem_trywait(data.image[IDzpdm].semptr[1])==0) {}
    else
    {
        printf("ERROR: semaphore #1 missing from image %s\n", IDzpdm_name);
        exit(0);
    }

    while(data.signal_USR1==0)
    {
        memset(data.image[IDtmp].array.F, '\0', sizeof(float)*wfsxysize);

        while(zpcnt0 == data.image[IDzpdm].md[0].cnt0)
            usleep(10);

        zpcnt0 = data.image[IDzpdm].md[0].cnt0;

        // TO BE DONE
        //  sem_wait(data.image[IDzpdm].semptr[1]);


        printf("WFS zero point offset update  # %8ld       (%s -> %s)  ", zpcnt, data.image[IDzpdm].name, data.image[IDwfszp].name);
        fflush(stdout);


        clock_gettime(CLOCK_REALTIME, &t1);

# ifdef _OPENMP
        #pragma omp parallel for private(elem)
# endif
        for(act=0; act<dmxysize; act++)
            for(elem=0; elem<wfsxysize; elem++)
                data.image[IDtmp].array.F[elem] += data.image[IDzpdm].array.F[act]*data.image[IDzrespM].array.F[act*wfsxysize+elem];


        clock_gettime(CLOCK_REALTIME, &t2);
        tdiff = info_time_diff(t1, t2);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;

        printf(" [ %10.3f ms]\n", 1e3*tdiffv);
        fflush(stdout);


        // copy results to IDwfszpo
        data.image[IDwfszp].md[0].write = 1;
        memcpy(data.image[IDwfszp].array.F, data.image[IDtmp].array.F, sizeof(float)*wfsxysize);
        COREMOD_MEMORY_image_set_sempost_byID(IDwfszp, -1);
        data.image[IDwfszp].md[0].cnt0 ++;
        data.image[IDwfszp].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDwfszp].md[0].write = 0;

        zpcnt++;
    }

    return 0;
}




//
// Create zero point WFS channels
// watch semaphore 1 on output (IDwfsref_name) -> sum all channels to update WFS zero point
// runs in separate process from RT computation
//
//
//
int_fast8_t AOloopControl_WFSzeropoint_sum_update_loop(long loopnb, const char *ID_WFSzp_name, int NBzp, const char *IDwfsref0_name, const char *IDwfsref_name)
{
    long wfsxsize, wfsysize, wfsxysize;
    long IDwfsref, IDwfsref0;
    long *IDwfszparray;
    long cntsumold;
    int RT_priority = 95; //any number from 0-99
    struct sched_param schedpar;
    long nsecwait = 10000; // 10 us
    struct timespec semwaitts;
    long ch;
    long IDtmp;
    long ii;
    char imname[200];
    int semval;



	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    if(seteuid(data.euid) != 0) // This goes up to maximum privileges
        printERROR(__FILE__, __func__, __LINE__, "seteuid() returns non-zero value");

    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster

    if(seteuid(data.ruid) != 0) // Go back to normal privileges
       printERROR(__FILE__, __func__, __LINE__, "seteuid() returns non-zero value");
#endif

    IDwfsref = image_ID(IDwfsref_name);
    wfsxsize = data.image[IDwfsref].md[0].size[0];
    wfsysize = data.image[IDwfsref].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;
    IDtmp = create_2Dimage_ID("wfsrefoffset", wfsxsize, wfsysize);
    IDwfsref0 = image_ID(IDwfsref0_name);


    if(data.image[IDwfsref].md[0].sem > 1) // drive semaphore #1 to zero
        while(sem_trywait(data.image[IDwfsref].semptr[1])==0) {}
    else
    {
        printf("ERROR: semaphore #1 missing from image %s\n", IDwfsref_name);
        exit(0);
    }

    IDwfszparray = (long*) malloc(sizeof(long)*NBzp);
    // create / read the zero point WFS channels
    for(ch=0; ch<NBzp; ch++)
    {
        if(sprintf(imname, "%s%ld", ID_WFSzp_name, ch) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        AOloopControl_IOtools_2Dloadcreate_shmim(imname, "", wfsxsize, wfsysize, 0.0);
        COREMOD_MEMORY_image_set_createsem(imname, 10);
        IDwfszparray[ch] = image_ID(imname);
    }

    cntsumold = 0;
    for(;;)
    {
        if (clock_gettime(CLOCK_REALTIME, &semwaitts) == -1) {
            perror("clock_gettime");
            exit(EXIT_FAILURE);
        }
        semwaitts.tv_nsec += nsecwait;
        if(semwaitts.tv_nsec >= 1000000000)
            semwaitts.tv_sec = semwaitts.tv_sec + 1;

        sem_timedwait(data.image[IDwfsref].semptr[1], &semwaitts);

        long cntsum = 0;
        for(ch=0; ch<NBzp; ch++)
            cntsum += data.image[IDwfszparray[ch]].md[0].cnt0;


        if(cntsum != cntsumold)
        {
			// copy wfsref0 to tmp
            memcpy(data.image[IDtmp].array.F, data.image[IDwfsref0].array.F, sizeof(float)*wfsxysize);

            for(ch=0; ch<NBzp; ch++)
                for(ii=0; ii<wfsxysize; ii++)
                    data.image[IDtmp].array.F[ii] += data.image[IDwfszparray[ch]].array.F[ii];

            // copy results to IDwfsref
            data.image[IDwfsref].md[0].write = 1;
            memcpy(data.image[IDwfsref].array.F, data.image[IDtmp].array.F, sizeof(float)*wfsxysize);
            data.image[IDwfsref].md[0].cnt0 ++;
            data.image[IDwfsref].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
            data.image[IDwfsref].md[0].write = 0;

/*            sem_getvalue(data.image[IDwfsref].semptr[0], &semval); // do not update sem 1
            if(semval<SEMAPHORE_MAXVAL)
                COREMOD_MEMORY_image_set_sempost(IDwfsref_name, 0);*/
            COREMOD_MEMORY_image_set_sempost_excl_byID(IDwfsref, 1);
            

            cntsumold = cntsum;
        }
    }

    free(IDwfszparray);


    return(0);
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
int_fast8_t AOloopControl_run()
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



    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    r = seteuid(data.euid); //This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid);//Go back to normal privileges
#endif


    loop = aoloopcontrol_var.LOOPNUMBER;


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
    if(AOconf[loop].init_CM==0)
    {
        printf("ERROR: CANNOT RUN LOOP WITHOUT CONTROL MATRIX\n");
        vOK = 0;
    }

	aoloopcontrol_var.aoconfcnt0_wfsref_current = data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0;
	aoloopcontrol_var.aoconfcnt0_contrM_current = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0;

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
        printf("entering loop ...\n");
        fflush(stdout);

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
                if(timerinit==0)
                {
                    //      Read_cam_frame(loop, 0, AOconf[loop].WFSnormalize, 0, 1);
                    clock_gettime(CLOCK_REALTIME, &t1);
                    timerinit = 1;
                }

                AOcompute(loop, AOconf[loop].WFSnormalize);

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
				

                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt0 = AOconf[loop].cnt;
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt1 = AOconf[loop].LOOPiteration;
                data.image[aoloopcontrol_var.aoconfIDlogdata].array.F[0] = AOconf[loop].gain;


                if(AOconf[loop].cnt == AOconf[loop].cntmax)
                    AOconf[loop].on = 0;
            }

        }
    }

    free(thetime);

    return(0);
}



int_fast8_t ControlMatrixMultiply( float *cm_array, float *imarray, long m, long n, float *outvect)
{
    long i;

    cblas_sgemv (CblasRowMajor, CblasNoTrans, m, n, 1.0, cm_array, n, imarray, 1, 0.0, outvect, 1);

    return(0);
}



/**
 * ## Purpose
 * 
 * Send modal commands to DM. \n
 * Converts mode coefficient to DM map by matrix-vector multiplication \n
 * Runs in CPU or GPU.
 * 
 * Takes mode values from aol_DMmode_cmd (ID = aoloopcontrol_var.aoconfID_cmd_modes)
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 */ 
int_fast8_t set_DM_modes(long loop)
{
    double a;
    long cnttest;
    int semval;



	

    if(AOconf[loop].GPU1 == 0)
    {
        float *arrayf;
        long i, j, k;

        arrayf = (float*) malloc(sizeof(float)*AOconf[loop].sizeDM);

        for(j=0; j<AOconf[loop].sizeDM; j++)
            arrayf[j] = 0.0;

        for(i=0; i<AOconf[loop].sizeDM; i++)
            for(k=0; k < AOconf[loop].NBDMmodes; k++)
                arrayf[i] += data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] * data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F[k*AOconf[loop].sizeDM+i];

        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 1;
        memcpy (data.image[aoloopcontrol_var.aoconfID_dmC].array.F, arrayf, sizeof(float)*AOconf[loop].sizeDM);
        if(data.image[aoloopcontrol_var.aoconfID_dmC].md[0].sem > 0)
        {
            sem_getvalue(data.image[aoloopcontrol_var.aoconfID_dmC].semptr[0], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoloopcontrol_var.aoconfID_dmC].semptr[0]);
        }
        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0++;
		data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 0;

        free(arrayf);
    }
    else
    {
#ifdef HAVE_CUDA


        GPU_loop_MultMat_setup(1, data.image[aoloopcontrol_var.aoconfID_DMmodes].name, data.image[aoloopcontrol_var.aoconfID_cmd_modes].name, data.image[aoloopcontrol_var.aoconfID_dmC].name, AOconf[loop].GPU1, aoloopcontrol_var.GPUset1, 1, AOconf[loop].GPUusesem, 1, loop);
        AOconf[loop].status = 12;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[32] = tdiffv;

        GPU_loop_MultMat_execute(1, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 30);
#endif
    }

    if(aoloopcontrol_var.aoconfID_dmdisp!=-1)
        if(data.image[aoloopcontrol_var.aoconfID_dmdisp].md[0].sem > 1)
        {
            sem_getvalue(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[1], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[1]);
        }

    AOconf[loop].DMupdatecnt ++;

    return(0);
}





/**
 * ## Purpose
 * 
 * Set deformable mirror modes related to the response matrix 
 * 
 * Takes mode values from ????????,
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 */ 

int_fast8_t set_DM_modesRM(long loop)
{
    long k;
    long i, j;
    float *arrayf;


    arrayf = (float*) malloc(sizeof(float)*AOconf[loop].sizeDM);

    for(j=0; j<AOconf[loop].sizeDM; j++)
        arrayf[j] = 0.0;

    for(k=0; k < AOconf[loop].NBDMmodes; k++)
    {
        for(i=0; i<AOconf[loop].sizeDM; i++)
            arrayf[i] += data.image[aoloopcontrol_var.aoconfID_cmd_modesRM].array.F[k] * data.image[aoloopcontrol_var.aoconfID_DMmodes].array.F[k*AOconf[loop].sizeDM+i];
    }


    data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].write = 1;
    memcpy (data.image[aoloopcontrol_var.aoconfID_dmRM].array.F, arrayf, sizeof(float)*AOconf[loop].sizeDM);
    data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].cnt0++;
    data.image[aoloopcontrol_var.aoconfID_dmRM].md[0].write = 0;

    free(arrayf);
    AOconf[loop].DMupdatecnt ++;

    return(0);
}





/**
 * ## Purpose
 * 
 * 
 * 
 * Takes mode values from ????????
 * 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	long
 * 				number of the loop 
 *
 *
 * @param[in]
 * paramname	int
 * 				normalize 
 * 
 */ 
int_fast8_t __attribute__((hot)) AOcompute(long loop, int normalize)
{
    long k1, k2;
    long ii;
    long i;
    long m, n;
    long index;
    //  long long wcnt;
    // long long wcntmax;
    double a;

    float *matrix_cmp;
    long wfselem, act, mode;

    struct timespec t1;
    struct timespec t2;

    float *matrix_Mc, *matrix_DMmodes;
    long n_sizeDM, n_NBDMmodes, n_sizeWFS;

    long IDmask;
    long act_active, wfselem_active;
    float *matrix_Mc_active;
    long IDcmatca_shm;
    int r;
    float imtot;

    int slice;
    int semnb;
    int semval;

	uint64_t LOOPiter;
	
	
	
	// lock loop iteration into variable so that it cannot increment 
	LOOPiter = AOconf[loop].LOOPiteration;


    // waiting for dark-subtracted image
    AOconf[loop].status = 19;  //  19: WAITING FOR IMAGE
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[23] = tdiffv;



    // md[0].atime.ts is absolute time at beginning of iteration
    //
    // pixel 0 is dt since last iteration
    //
    // pixel 1 is time from beginning of loop to status 01
    // pixel 2 is time from beginning of loop to status 02


    Read_cam_frame(loop, 0, normalize, 0, 0);

    slice = aoloopcontrol_var.PIXSTREAM_SLICE;
    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING==0) // no pixel streaming
        aoloopcontrol_var.PIXSTREAM_SLICE = 0;
    //    else
    //        aoloopcontrol_var.PIXSTREAM_SLICE = 1 + slice;

    //    printf("slice = %d  ->  %d\n", slice, aoloopcontrol_var.PIXSTREAM_SLICE);
    //    fflush(stdout);

    AOconf[loop].status = 4;  // 4: REMOVING REF
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[15] = tdiffv;


    if(AOconf[loop].GPUall==0)
    {
        data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 1;
        for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
            data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F[ii] = data.image[aoloopcontrol_var.aoconfID_imWFS1].array.F[ii] - aoloopcontrol_var.normfloorcoeff*data.image[aoloopcontrol_var.aoconfID_wfsref].array.F[ii];
        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS2, -1);
        data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt0 ++;
        data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt1 = LOOPiter;
        data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 0;
    }


    AOconf[loop].status = 5; // 5 MULTIPLYING BY CONTROL MATRIX -> MODE VALUES
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[16] = tdiffv;


    if(AOconf[loop].initmapping == 0) // compute combined control matrix or matrices
    {
        printf("COMPUTING MAPPING ARRAYS .... \n");
        fflush(stdout);

        clock_gettime(CLOCK_REALTIME, &t1);

        //
        // There is one mapping array per WFS slice
        // WFS slice 0 = all active pixels
        //
        aoloopcontrol_var.WFS_active_map = (int*) malloc(sizeof(int)*AOconf[loop].sizeWFS*aoloopcontrol_var.PIXSTREAM_NBSLICES);
        if(aoloopcontrol_var.aoconfID_wfsmask != -1)
        {
            for(slice=0; slice<aoloopcontrol_var.PIXSTREAM_NBSLICES; slice++)
            {
                long ii1 = 0;
                for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
                    if(data.image[aoloopcontrol_var.aoconfID_wfsmask].array.F[ii]>0.1)
                    {
                        if(slice==0)
                        {
                            aoloopcontrol_var.WFS_active_map[slice*AOconf[loop].sizeWFS+ii1] = ii;
                            ii1++;
                        }
                        else if (data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii]==slice+1)
                        {
                            aoloopcontrol_var.WFS_active_map[slice*AOconf[loop].sizeWFS+ii1] = ii;
                            ii1++;
                        }
                    }
                AOconf[loop].sizeWFS_active[slice] = ii1;

                char imname[200];
                if(sprintf(imname, "aol%ld_imWFS2active_%02d", aoloopcontrol_var.LOOPNUMBER, slice) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                uint32_t *sizearray;
                sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
                sizearray[0] =  AOconf[loop].sizeWFS_active[slice];
                sizearray[1] =  1;
                aoloopcontrol_var.aoconfID_imWFS2_active[slice] = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
                free(sizearray);
                //aoloopcontrol_var.aoconfID_imWFS2_active[slice] = create_2Dimage_ID(imname, AOconf[loop].sizeWFS_active[slice], 1);
            }
        }
        else
        {
            printf("ERROR: aoloopcontrol_var.aoconfID_wfsmask = -1\n");
            fflush(stdout);
            exit(0);
        }



        // create DM active map
        aoloopcontrol_var.DM_active_map = (int*) malloc(sizeof(int)*AOconf[loop].sizeDM);
        if(aoloopcontrol_var.aoconfID_dmmask != -1)
        {
            long ii1 = 0;
            for(ii=0; ii<AOconf[loop].sizeDM; ii++)
                if(data.image[aoloopcontrol_var.aoconfID_dmmask].array.F[ii]>0.5)
                {
                    aoloopcontrol_var.DM_active_map[ii1] = ii;
                    ii1++;
                }
            AOconf[loop].sizeDM_active = ii1;
        }



        uint32_t *sizearray;
        sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
        sizearray[0] = AOconf[loop].sizeDM_active;
        sizearray[1] = 1;

        char imname[200];
        if(sprintf(imname, "aol%ld_meas_act_active", aoloopcontrol_var.LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_meas_act_active = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
        free(sizearray);



        if(aoloopcontrol_var.aoconfID_meas_act==-1)
        {
            sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
            sizearray[0] = AOconf[loop].sizexDM;
            sizearray[1] = AOconf[loop].sizeyDM;

            if(sprintf(imname, "aol%ld_meas_act", aoloopcontrol_var.LOOPNUMBER) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            aoloopcontrol_var.aoconfID_meas_act = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            COREMOD_MEMORY_image_set_createsem(imname, 10);
            free(sizearray);
        }

        clock_gettime(CLOCK_REALTIME, &t2);
        tdiff = info_time_diff(t1, t2);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        printf("\n");
        printf("TIME TO COMPUTE MAPPING ARRAYS = %f sec\n", tdiffv);
        AOconf[loop].initmapping = 1;
    }




    if(AOconf[loop].GPU0 == 0)   // no GPU -> run in CPU
    {
        if(AOconf[loop].CMMODE==0)  // goes explicitely through modes, slower but required for access to mode values
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=0, CMMODE=0 - %s x %s -> %s\n", data.image[aoloopcontrol_var.aoconfID_contrM].md[0].name, data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].name, data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].name);
            fflush(stdout);
#endif

            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write = 1;
            ControlMatrixMultiply( data.image[aoloopcontrol_var.aoconfID_contrM].array.F, data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F, AOconf[loop].NBDMmodes, AOconf[loop].sizeWFS, data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F);
            COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_meas_modes, -1);
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt0 ++;
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write = 0;
        }
        else // (*)
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=0, CMMODE=1 - using matrix %s\n", data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].name);
            fflush(stdout);
#endif

            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write = 1;
            ControlMatrixMultiply( data.image[aoloopcontrol_var.aoconfID_contrMc].array.F, data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F, AOconf[loop].sizeDM, AOconf[loop].sizeWFS, data.image[aoloopcontrol_var.aoconfID_meas_act].array.F);
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt0 ++;
            COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_meas_modes, -1);
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt0 ++;
			data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write = 0;
        }
    }
    else  // run in GPU if possible 
    {
#ifdef HAVE_CUDA
        if(AOconf[loop].CMMODE==0)  // goes explicitely through modes, slower but required for access to mode values
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=1, CMMODE=0 - using matrix %s    GPU alpha beta = %f %f\n", data.image[aoloopcontrol_var.aoconfID_contrM].md[0].name, aoloopcontrol_var.GPU_alpha, aoloopcontrol_var.GPU_beta);
            fflush(stdout);
#endif

            aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1; // default: do not re-compute reference output

            if(AOconf[loop].GPUall == 1)
            {
                // TEST IF contrM or wfsref have changed
                if((data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0 != aoloopcontrol_var.aoconfcnt0_wfsref_current) || (data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0 != aoloopcontrol_var.aoconfcnt0_contrM_current))
					{
						printf("NEW wfsref [%10ld] or contrM [%10ld]\n", data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0, data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0);
						aoloopcontrol_var.aoconfcnt0_wfsref_current = data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0;
						aoloopcontrol_var.aoconfcnt0_contrM_current = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0;
						aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
					}


                if(aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE]==0) // initialize WFS reference
                {
#ifdef _PRINT_TEST
                    printf("\nINITIALIZE WFS REFERENCE: COPY NEW REF (WFSREF) TO imWFS0\n"); //TEST
                    fflush(stdout);
#endif

                    data.image[aoloopcontrol_var.aoconfID_contrM].md[0].write = 1;
                    memcpy(data.image[aoloopcontrol_var.aoconfID_contrM].array.F, data.image[aoloopcontrol_var.aoconfID_wfsref].array.F, sizeof(float)*AOconf[loop].sizeWFS);
             //       for(wfselem=0; wfselem<AOconf[loop].sizeWFS; wfselem++)
             //           data.image[aoloopcontrol_var.aoconfID_contrM].array.F[wfselem] = data.image[aoloopcontrol_var.aoconfID_wfsref].array.F[wfselem];
                    COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_contrM, -1);
                    data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0++;
                    data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt1 = LOOPiter;
                    data.image[aoloopcontrol_var.aoconfID_contrM].md[0].write = 0;
                    fflush(stdout);
                }
            }


            if(AOconf[loop].GPUall == 1)
                GPU_loop_MultMat_setup(0, data.image[aoloopcontrol_var.aoconfID_contrM].name, data.image[aoloopcontrol_var.aoconfID_contrM].name, data.image[aoloopcontrol_var.aoconfID_meas_modes].name, AOconf[loop].GPU0, aoloopcontrol_var.GPUset0, 0, AOconf[loop].GPUusesem, aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE], loop);
            else
                GPU_loop_MultMat_setup(0, data.image[aoloopcontrol_var.aoconfID_contrM].name, data.image[aoloopcontrol_var.aoconfID_imWFS2].name, data.image[aoloopcontrol_var.aoconfID_meas_modes].name, AOconf[loop].GPU0, aoloopcontrol_var.GPUset0, 0, AOconf[loop].GPUusesem, 1, loop);

			

            aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;

            AOconf[loop].status = 6; // 6 execute
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[17] = tdiffv;


            if(AOconf[loop].GPUall == 1)
                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], aoloopcontrol_var.GPU_alpha, aoloopcontrol_var.GPU_beta, 1, 25);
            else
                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 25);
        }
        else // direct pixel -> actuators linear transformation
        {
#ifdef _PRINT_TEST
            printf("TEST - CM mult: GPU=1, CMMODE=1\n");
            fflush(stdout);
#endif

			// depreciated: use all pixels
/*            if(1==0) 
            {
                GPU_loop_MultMat_setup(0, data.image[aoloopcontrol_var.aoconfID_contrMc].name, data.image[aoloopcontrol_var.aoconfID_imWFS2].name, data.image[aoloopcontrol_var.aoconfID_meas_act].name, AOconf[loop].GPU0, aoloopcontrol_var.GPUset0, 0, AOconf[loop].GPUusesem, 1, loop);
                AOconf[loop].status = 6; 
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[6] = tdiffv;

                GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1);
            }
            else // only use active pixels and actuators (**)
            {*/
             
                // re-map input vector into imWFS2_active

                if(AOconf[loop].GPUall == 1) // (**)
                {
#ifdef _PRINT_TEST
                    printf("TEST - CM mult: GPU=1, CMMODE=1, GPUall = 1\n");
                    fflush(stdout);
#endif

                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 1;
                    for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[aoloopcontrol_var.PIXSTREAM_SLICE]; wfselem_active++)
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoloopcontrol_var.aoconfID_contrM].array.F[aoloopcontrol_var.WFS_active_map[aoloopcontrol_var.PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                    COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE], -1);
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt0++;
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 0;
                }
                else
                {
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 1;
                    for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[aoloopcontrol_var.PIXSTREAM_SLICE]; wfselem_active++)
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F[aoloopcontrol_var.WFS_active_map[aoloopcontrol_var.PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                    COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE], -1);
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt0++;
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                    data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 0;
                }

                // look for updated control matrix or reference
                if(AOconf[loop].GPUall == 1) // (**)
                {
                    if(data.image[aoloopcontrol_var.aoconfID_contrMcact[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt0 != aoloopcontrol_var.contrMcactcnt0[aoloopcontrol_var.PIXSTREAM_SLICE])
                    {
                        printf("NEW CONTROL MATRIX DETECTED (%s) -> RECOMPUTE REFERENCE x MATRIX\n", data.image[aoloopcontrol_var.aoconfID_contrMcact[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].name);
                        fflush(stdout);

                        aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                        aoloopcontrol_var.contrMcactcnt0[aoloopcontrol_var.PIXSTREAM_SLICE] = data.image[aoloopcontrol_var.aoconfID_contrMcact[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt0;
                    }

                    if(data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0 != aoloopcontrol_var.wfsrefcnt0)  // (*)
                    {
                        printf("NEW REFERENCE WFS DETECTED (%s) [ %ld %ld ]\n", data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].name, data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0, aoloopcontrol_var.wfsrefcnt0);
                        fflush(stdout);

                        aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                        aoloopcontrol_var.wfsrefcnt0 = data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0;
                    }
                    if(aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE]==0) // initialize WFS reference
                    {
                        printf("\nINITIALIZE WFS REFERENCE: COPY NEW REF (WFSREF) TO imWFS2_active\n"); //TEST
                        fflush(stdout);
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 1;
                        for(wfselem_active=0; wfselem_active<AOconf[loop].sizeWFS_active[aoloopcontrol_var.PIXSTREAM_SLICE]; wfselem_active++)
                            data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].array.F[wfselem_active] = data.image[aoloopcontrol_var.aoconfID_wfsref].array.F[aoloopcontrol_var.WFS_active_map[aoloopcontrol_var.PIXSTREAM_SLICE*AOconf[loop].sizeWFS+wfselem_active]];
                        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE], -1);
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt0++;
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].cnt1 = LOOPiter;
                        data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].md[0].write = 0;
                        fflush(stdout);
                    }
                }

                if(aoloopcontrol_var.initcontrMcact_GPU[aoloopcontrol_var.PIXSTREAM_SLICE]==0)
                    aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;


                GPU_loop_MultMat_setup(0, data.image[aoloopcontrol_var.aoconfID_contrMcact[aoloopcontrol_var.PIXSTREAM_SLICE]].name, data.image[aoloopcontrol_var.aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]].name, data.image[aoloopcontrol_var.aoconfID_meas_act_active].name, AOconf[loop].GPU0, aoloopcontrol_var.GPUset0, 0, AOconf[loop].GPUusesem, aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE], loop);


                aoloopcontrol_var.initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;
                aoloopcontrol_var.initcontrMcact_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;
                
                AOconf[loop].status = 6; // 6 execute
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
                tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[17] = tdiffv;


                if(AOconf[loop].GPUall == 1)
                    GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], aoloopcontrol_var.GPU_alpha, aoloopcontrol_var.GPU_beta, 1, 25);
                else
                    GPU_loop_MultMat_execute(0, &AOconf[loop].status, &AOconf[loop].GPUstatus[0], 1.0, 0.0, 1, 25);

                // re-map output vector
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 1;
                for(act_active=0; act_active<AOconf[loop].sizeDM_active; act_active++)
                    data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[aoloopcontrol_var.DM_active_map[act_active]] = data.image[aoloopcontrol_var.aoconfID_meas_act_active].array.F[act_active];

				COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_meas_act, -1);
            /*    for(semnb=0; semnb<data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].sem; semnb++)
                {
                    sem_getvalue(data.image[aoloopcontrol_var.aoconfID_meas_act].semptr[semnb], &semval);
                    if(semval<SEMAPHORE_MAXVAL)
                        sem_post(data.image[aoloopcontrol_var.aoconfID_meas_act].semptr[semnb]);
                }*/
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt1 = LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 0;
            //}
        }
#endif
    }

    AOconf[loop].status = 11; // 11 MULTIPLYING BY GAINS
    clock_gettime(CLOCK_REALTIME, &tnow);
    tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[18] = tdiffv;

    if(AOconf[loop].CMMODE==0)
    {
		int block;
		long k;
		
		
        AOconf[loop].RMSmodes = 0;
        for(k=0; k<AOconf[loop].NBDMmodes; k++)
            AOconf[loop].RMSmodes += data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k]*data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k];

        AOconf[loop].RMSmodesCumul += AOconf[loop].RMSmodes;
        AOconf[loop].RMSmodesCumulcnt ++;

		data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 1;

        for(k=0; k<AOconf[loop].NBDMmodes; k++)
        {	
            data.image[aoloopcontrol_var.aoconfID_RMS_modes].array.F[k] = 0.99*data.image[aoloopcontrol_var.aoconfID_RMS_modes].array.F[k] + 0.01*data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k]*data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k];
            data.image[aoloopcontrol_var.aoconfID_AVE_modes].array.F[k] = 0.99*data.image[aoloopcontrol_var.aoconfID_AVE_modes].array.F[k] + 0.01*data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k];
			
			
			// apply gain
            
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] -= AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_gainb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k] * data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F[k];


			// apply limits
			
			float limitval;
			limitval = AOconf[loop].maxlimit * data.image[aoloopcontrol_var.aoconfID_limitb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k];
            
            if(data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] < -limitval)
                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = -limitval;

            if(data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] > limitval)
                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = limitval;


			// apply mult factor
			
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] *= AOconf[loop].mult * data.image[aoloopcontrol_var.aoconfID_multfb].array.F[AOconf[loop].modeBlockIndex[k]] * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[k];

            
            
            // update total gain
            //     data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k+AOconf[loop].NBDMmodes] = AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k];
        }


        data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt0 ++;
        data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt1 = LOOPiter;
        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_cmd_modes, -1);
        data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 0;
        
    }

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



#endif

    return(0);
}





//
// compute DM map from mode values
// this is a separate process 
//
// if offloadMode = 1, apply correction to aol#_dmC
//
int_fast8_t AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, const char *modecoeffs_name, const char *DMmodes_name, int semTrigg, const char *out_name, int GPUindex, long loop, int offloadMode)
{
#ifdef HAVE_CUDA
    long IDmodecoeffs;
    int GPUcnt, k;
    int *GPUsetM;
    int_fast8_t GPUstatus[100];
    int_fast8_t status;
    float alpha = 1.0;
    float beta = 0.0;
    int initWFSref = 0;
    int orientation = 1;
    int use_sem = 1;
    long IDout;
    int write_timing = 0;
    long NBmodes, m;

    float x, x2, x4, x8;
    float gamma;

    uint32_t *sizearray;
    char imnameInput[200];
    long IDmodesC;

    long IDc;
    long dmxsize, dmysize;
    long ii;


    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;

	char imname[200];


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


	if(GPUMATMULTCONFindex==0)
    {
		// read AO loop gain, mult
		if(aoloopcontrol_var.AOloopcontrol_meminit==0)
			AOloopControl_InitializeMemory(1);
	}


    GPUcnt = 1;
    GPUsetM = (int*) malloc(sizeof(int)*GPUcnt);
    for(k=0; k<GPUcnt; k++)
        GPUsetM[k] = k+GPUindex;

    IDout = image_ID(out_name);
    IDmodecoeffs = image_ID(modecoeffs_name);

    NBmodes = data.image[IDmodecoeffs].md[0].size[0];


   // sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);

    //if(sprintf(imnameInput, "aol%ld_mode_limcorr", loop) < 1)
    //    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	

   // sizearray[0] = NBmodes;
   // sizearray[1] = 1;
  //  IDmodesC = create_image_ID(imnameInput, 2, sizearray, _DATATYPE_FLOAT, 0, 0);
  //  COREMOD_MEMORY_image_set_createsem(imnamecorr, 10);
  //  free(sizearray);




    GPU_loop_MultMat_setup(GPUMATMULTCONFindex, DMmodes_name, modecoeffs_name, out_name, GPUcnt, GPUsetM, orientation, use_sem, initWFSref, 0);


    for(k=0; k<GPUcnt; k++)
        printf(" ====================     USING GPU %d\n", GPUsetM[k]);

	list_image_ID();

    if(offloadMode==1)
    {
	    char imnamedmC[200];

		
		
        if(sprintf(imnamedmC, "aol%ld_dmC", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        IDc = image_ID(imnamedmC);
        dmxsize = data.image[IDc].md[0].size[0];
        dmysize = data.image[IDc].md[0].size[1];


        printf("offloadMode = %d  %ld %ld\n", offloadMode, dmxsize, dmysize);
        fflush(stdout);
    }
    else
		printf("offloadMode = %d\n", offloadMode);

	printf("out_name = %s \n", out_name);
	printf("IDout    = %ld\n", IDout);
	



    for(;;)
    {
        COREMOD_MEMORY_image_set_semwait(modecoeffs_name, semTrigg);
	
//		if(GPUMATMULTCONFindex==0)
			AOconf[loop].statusM = 10;               
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[7] = tdiffv;

      //  for(m=0; m<NBmodes; m++)
      //      data.image[IDmodesC].array.F[m] = data.image[IDmodecoeffs].array.F[m];


        GPU_loop_MultMat_execute(GPUMATMULTCONFindex, &status, &GPUstatus[0], alpha, beta, write_timing, 0);

        if(offloadMode==1) // offload back to dmC
        {
            data.image[IDc].md[0].write = 1;
            for(ii=0; ii<dmxsize*dmysize; ii++)
                data.image[IDc].array.F[ii] = data.image[IDout].array.F[ii];

			data.image[IDc].md[0].cnt0++;
			data.image[IDc].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
            COREMOD_MEMORY_image_set_sempost_byID(IDc, -1);
            data.image[IDc].md[0].write = 0;
        }
      
  
  //		if(GPUMATMULTCONFindex==0)  
			AOconf[loop].statusM = 20;
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[8] = tdiffv;
    }



    free(GPUsetM);

#endif

    return(0);
}





/**
 * ## Purpose
 * 
 * Computes average of residual in WFS
 * 
 * ## Arguments
 * 
 * @param[in]
 * loop		INT
 * 			loop number
 * 
 * @param[in]
 * alpha	FLOAT
 * 			averaging coefficient
 * 
 * 
 * ## Output files
 * 
 * - aol_wfsres_ave
 * - aol_wfsres_ave
 * - aol_wfsresm
 * - aol_wfsresm_ave
 * - aol_wfsres_rms
 * 
 * 
 * 
 */

long AOloopControl_computeWFSresidualimage(long loop, char *IDalpha_name)
{
    long IDwfsref, IDwfsmask, IDtot, IDout, IDoutave, IDoutm, IDoutmave, IDoutrms;
    char imname[200];
    uint32_t *sizearray;
    long wfsxsize, wfsysize, wfsxysize;
    long cnt;
    long ii;
	long IDalpha;

	IDalpha = image_ID(IDalpha_name);
	

    if(sprintf(imname, "aol%ld_imWFS0", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	aoloopcontrol_var.aoconfID_contrM = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_wfsref", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	
    IDwfsref = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_wfsmask", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDwfsmask = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_imWFS0tot", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDtot = read_sharedmem_image(imname);
	
	

    wfsxsize = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].size[0];
    wfsysize = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].size[1];
    wfsxysize = wfsxsize*wfsysize;

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = wfsxsize;
    sizearray[1] = wfsysize;


    if(sprintf(imname, "aol%ld_wfsres", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDout = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


    if(sprintf(imname, "aol%ld_wfsres_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutave = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutave].array.F[ii] = 0.0;


    if(sprintf(imname, "aol%ld_wfsresm", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutm = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


    if(sprintf(imname, "aol%ld_wfsresm_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutmave = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutave].array.F[ii] = 0.0;


    if(sprintf(imname, "aol%ld_wfsres_rms", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDoutrms = create_image_ID(imname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    for(ii=0; ii<wfsxysize; ii++)
        data.image[IDoutrms].array.F[ii] = 0.0;

    free(sizearray);



	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    
    for(;;)
    {
		
        if(data.image[aoloopcontrol_var.aoconfID_contrM].md[0].sem==0)
        {
            while(cnt==data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0;
        }
        else
        {
            sem_wait(data.image[aoloopcontrol_var.aoconfID_contrM].semptr[3]);
		}

		//
		// instantaneous WFS residual
        // imWFS0/tot0 - WFSref -> out
        //
		//printf("  %20f\n", data.image[IDtot].array.F[0]);//TEST
		
        data.image[IDout].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDout].array.F[ii] = data.image[aoloopcontrol_var.aoconfID_contrM].array.F[ii]/data.image[IDtot].array.F[0] - data.image[IDwfsref].array.F[ii];
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDout].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);


        // apply mask

        data.image[IDoutm].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutm].array.F[ii] = data.image[IDout].array.F[ii] * data.image[IDwfsmask].array.F[ii];
        data.image[IDoutm].md[0].cnt0++;
        data.image[IDoutm].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutm].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutm, -1);


        // apply gain -> outave

        data.image[IDoutave].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutave].array.F[ii] = (1.0-data.image[IDalpha].array.F[0])*data.image[IDoutave].array.F[ii] + data.image[IDalpha].array.F[0]*data.image[IDout].array.F[ii];
        data.image[IDoutave].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutave].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutave, -1);

        // apply mask

        data.image[IDoutmave].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutmave].array.F[ii] = data.image[IDoutave].array.F[ii] * data.image[IDwfsmask].array.F[ii];
        data.image[IDoutmave].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutmave].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutmave, -1);

        // compute RMS

        data.image[IDoutrms].md[0].write = 1;
        for(ii=0; ii<wfsxysize; ii++)
            data.image[IDoutrms].array.F[ii] = (1.0-data.image[IDalpha].array.F[0])*data.image[IDoutrms].array.F[ii] + data.image[IDalpha].array.F[0]*(data.image[IDout].array.F[ii]-data.image[IDoutave].array.F[ii])*(data.image[IDout].array.F[ii]-data.image[IDoutave].array.F[ii]);
        data.image[IDoutrms].md[0].cnt0++;
        data.image[IDoutave].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDoutrms].md[0].write = 0;
        COREMOD_MEMORY_image_set_sempost_byID(IDoutrms, -1);

    }


    return(IDout);
}


















// includes mode filtering (limits, multf)
//
long __attribute__((hot)) AOloopControl_ComputeOpenLoopModes(long loop)
{
    long IDout;
    long IDmodeval; // WFS measurement

    long modeval_bsize = 20; // circular buffer size (valid for both DM and PF)

    long IDmodevalDM_C; // DM correction, circular buffer to include history
    long modevalDMindexl = 0;
    long modevalDMindex = 0; // index in the circular buffer     
    
    long IDmodevalDM; // DM correction at WFS measurement time
    long IDmodevalDMcorr; // current DM correction
    long IDmodevalDMnow; // DM correction after predictiv control 
    long IDmodevalDMnowfilt; // current DM correction filtered
    float alpha;
    long IDmodevalPFsync;

	long IDmodeARPFgain; // predictive filter mixing ratio per gain (0=non-predictive, 1=predictive)
   // long IDmodevalPF; // predictive filter output
	long IDmodevalPFres; // predictive filter measured residual (real-time)
	long IDmodeWFSnoise; // WFS noise
    long IDmodevalPF_C; // modal prediction, circular buffer to include history
    long modevalPFindexl = 0;
    long modevalPFindex = 0; // index in the circular buffer


    long IDblknb;
    char imname[200];
    float *modegain;
    float *modemult;
    float *modelimit;
    long *modeblock;
    long i, m, blk, NBmodes;
    unsigned int blockNBmodes[100];
    uint32_t *sizeout;
    float framelatency = 2.8;
    long framelatency0, framelatency1;
    long IDgainb;
    long cnt;


    // FILTERING
    int FILTERMODE = 1;
    //long IDmodeLIMIT;
    //long IDmodeMULT;

    // TELEMETRY
    long block;
    long blockstatcnt = 0;

	double blockavePFresrms[100];
    double blockaveOLrms[100];
    double blockaveCrms[100]; // correction RMS
    double blockaveWFSrms[100]; // WFS residual RMS
    double blockaveWFSnoise[100]; // WFS noise
    double blockavelimFrac[100];

	double allavePFresrms;
    double allaveOLrms;
    double allaveCrms;
    double allaveWFSrms;
    double allaveWFSnoise;
    double allavelimFrac;

    float limitblockarray[100];
    long IDatlimbcoeff;

	long IDautogain = -1; // automatic gain input
	long long autogainCnt = 0;

    float coeff;

    int RT_priority = 80; //any number from 0-99
    struct sched_param schedpar;

	long long loopPFcnt;
	FILE *fptest;


	uint64_t LOOPiter;


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif



    // read AO loop gain, mult
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    // INPUT
    if(sprintf(imname, "aol%ld_modeval", loop) < 1)// measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(imname);
    NBmodes = data.image[IDmodeval].md[0].size[0];

    modegain = (float*) malloc(sizeof(float)*NBmodes);
    modemult = (float*) malloc(sizeof(float)*NBmodes);
    modelimit = (float*) malloc(sizeof(float)*NBmodes);

    modeblock = (long*) malloc(sizeof(long)*NBmodes);




	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}



    // CONNECT to dm control channel
    if(aoloopcontrol_var.aoconfID_dmC == -1)
    {
        if(sprintf(imname, "aol%ld_dmC", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_dmC = read_sharedmem_image(imname);
    }



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

    if(aoloopcontrol_var.aoconfID_limitb == -1)
    {
        if(sprintf(imname, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_limitb = read_sharedmem_image(imname);
    }



    // CONNECT to arrays holding gain, limit and multf values for individual modes
    if(aoloopcontrol_var.aoconfID_DMmode_GAIN == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_GAIN", aoloopcontrol_var.LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(imname);
    }
    printf("aoloopcontrol_var.aoconfID_DMmode_GAIN = %ld\n", aoloopcontrol_var.aoconfID_DMmode_GAIN);


    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(imname);
    }
    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
        FILTERMODE = 0;


    if(aoloopcontrol_var.aoconfID_MULTF_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(imname);
    }
    if(aoloopcontrol_var.aoconfID_MULTF_modes == -1)
        FILTERMODE = 0;










    // predictive control output
    if(aoloopcontrol_var.aoconfID_modevalPF == -1)
    {
		if(sprintf(imname, "aol%ld_modevalPF", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_modevalPF = read_sharedmem_image(imname);
		if(aoloopcontrol_var.aoconfID_modevalPF != -1)
		{
			long ii;
			for(ii=0; ii<data.image[aoloopcontrol_var.aoconfID_modevalPF].md[0].size[0]*data.image[aoloopcontrol_var.aoconfID_modevalPF].md[0].size[1]; ii++)
				data.image[aoloopcontrol_var.aoconfID_modevalPF].array.F[ii] = 0.0;
		}
	}

    // OUPUT
    sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizeout[0] = NBmodes;
    sizeout[1] = 1;

	// all images below are vectors of dimension NBmodes x 1

	// load/create aol_modeval_ol (pseudo-open loop mode values)
    if(sprintf(imname, "aol%ld_modeval_ol", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDout = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 20);


	// load/create aol_mode_blknb (block index for each mode)
    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDblknb = create_image_ID(imname, 2, sizeout, _DATATYPE_UINT16, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_corr (current modal DM correction)
    if(sprintf(imname, "aol%ld_modeval_dm_corr", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMcorr = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_now (current modal DM correction after mixing with predicitiv control)
    if(sprintf(imname, "aol%ld_modeval_dm_now", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnow = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm_now_filt (current modal DM correction, filtered)
    if(sprintf(imname, "aol%ld_modeval_dm_now_filt", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnowfilt = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modeval_dm", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDM = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);


	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFsync", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodevalPFsync = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	

	// load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFres", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodevalPFres = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    

	// load/create WFS noise estimate
	if(sprintf(imname, "aol%ld_modeWFSnoise", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
	IDmodeWFSnoise = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    

	
	
	//
	// load/create aol_mode_ARPFgain (mixing ratio between non-predictive and predictive mode values)
	// 0: adopt non-predictive value
	// 1: adopt predictive value
	//
	// Set values to 0 when predictive filter is off
	// set to 1 (or intermediate value) when predictive filter for corresponding mode is on
	//
    if(sprintf(imname, "aol%ld_mode_ARPFgain", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodeARPFgain = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	// initialize the gain to zero for all modes
	for(m=0;m<NBmodes;m++)
		data.image[IDmodeARPFgain].array.F[m] = 0.0;

	if(aoloopcontrol_var.aoconfID_modeARPFgainAuto == -1)
	{
		// multiplicative auto ratio on top of gain above
		if(sprintf(imname, "aol%ld_mode_ARPFgainAuto", loop) < 1) 
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_modeARPFgainAuto = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
		COREMOD_MEMORY_image_set_createsem(imname, 10);
		// initialize the gain to zero for all modes
		for(m=0;m<NBmodes;m++)
			data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = 1.0;
	}


    sizeout[1] = modeval_bsize;
    if(sprintf(imname, "aol%ld_modeval_dm_C", loop) < 1) // modal DM correction, circular buffer
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalDM_C = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);

 

	// modal prediction, circular buffer
    sizeout[1] = modeval_bsize;
    if(sprintf(imname, "aol%ld_modevalPF_C", loop) < 1) 
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalPF_C = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
	




    // auto limit tuning
    sizeout[0] = AOconf[loop].DMmodesNBblock;
    sizeout[1] = 1;
    if(sprintf(imname, "aol%ld_autotune_lim_bcoeff", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDatlimbcoeff = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);

    free(sizeout);

    printf("%ld modes\n", NBmodes);


    // Read from shared mem the DM mode files to indentify blocks
    data.image[IDblknb].md[0].write = 1;
    m = 0;
    blk = 0;
    while(m<NBmodes)
    {
		long n;
		long ID;
		
        if(sprintf(imname, "aol%ld_DMmodes%02ld", loop, blk) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        printf("Loading %s  (%2ld/%2ld)\n", imname, m, NBmodes);
        fflush(stdout);

        ID = read_sharedmem_image(imname);
        if(ID==-1)
        {
            printf("ERROR: could not load %s from shared memory\n", imname);
            exit(0);
        }
        n = data.image[ID].md[0].size[2];
        printf(" -> found %2ld modes\n", n);
        blockNBmodes[blk] = n;

        for(i=0; i<n; i++)
        {
            modeblock[m] = blk;
            data.image[IDblknb].array.UI16[m] = blk;
            m++;
        }
        blk++;
    }
    COREMOD_MEMORY_image_set_sempost_byID(IDblknb, -1);
    data.image[IDblknb].md[0].cnt0++;
    data.image[IDblknb].md[0].cnt1 = AOconf[loop].LOOPiteration;
    data.image[IDblknb].md[0].write = 0;






    framelatency = AOconf[loop].hardwlatency_frame + AOconf[loop].wfsmextrlatency_frame;
    framelatency0 = (long) framelatency;
    framelatency1 = framelatency0 + 1;
    alpha = framelatency - framelatency0;

    // initialize arrays
    data.image[IDmodevalDM].md[0].write = 1;
    data.image[IDmodevalDMcorr].md[0].write = 1;
    data.image[IDmodevalDMnow].md[0].write = 1;
    data.image[IDmodevalDM_C].md[0].write = 1;
    data.image[IDmodevalPF_C].md[0].write = 1;
    for(m=0; m<NBmodes; m++)
    {
        data.image[IDmodevalDM].array.F[m] = 0.0;
        data.image[IDmodevalDMcorr].array.F[m] = 0.0;
        data.image[IDmodevalDMnow].array.F[m] = 0.0;
        for(modevalDMindex=0; modevalDMindex<modeval_bsize; modevalDMindex++)
            data.image[IDmodevalDM_C].array.F[modevalDMindex*NBmodes+m] = 0;
        for(modevalPFindex=0; modevalPFindex<modeval_bsize; modevalPFindex++)
            data.image[IDmodevalPF_C].array.F[modevalPFindex*NBmodes+m] = 0;
        data.image[IDout].array.F[m] = 0.0;
    }
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMcorr, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnow, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM_C, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPF_C, -1);
    data.image[IDmodevalDM].md[0].cnt0++;
    data.image[IDmodevalDMcorr].md[0].cnt0++;
    data.image[IDmodevalDMnow].md[0].cnt0++;
    data.image[IDmodevalDM_C].md[0].cnt0++;
    data.image[IDmodevalPF_C].md[0].cnt0++;
    data.image[IDmodevalDM].md[0].write = 0;
    data.image[IDmodevalDMcorr].md[0].write = 0;
    data.image[IDmodevalDMnow].md[0].write = 0;
    data.image[IDmodevalDM_C].md[0].write = 0;
    data.image[IDmodevalPF_C].md[0].write = 0;

    printf("FILTERMODE = %d\n", FILTERMODE);
 

    modevalDMindex = 0;
    modevalDMindexl = 0;

    modevalPFindex = 0;
    modevalPFindexl = 0;

    cnt = 0;

    blockstatcnt = 0;
    for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
    {
		blockavePFresrms[block] = 0.0;
        blockaveOLrms[block] = 0.0;
        blockaveCrms[block] = 0.0;
        blockaveWFSrms[block] = 0.0;
        blockavelimFrac[block] = 0.0;
    }
    allaveOLrms = 0.0;
    allaveCrms = 0.0;
    allaveWFSrms = 0.0;
    allavelimFrac = 0.0;



	loopPFcnt = 0;
    for(;;)
    {		
		long modevalDMindex0, modevalDMindex1;
		long modevalPFindex0, modevalPFindex1;
		
		
        // read WFS measured modes (residual)
        if(data.image[IDmodeval].md[0].sem==0)
        {
            while(cnt==data.image[IDmodeval].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDmodeval].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDmodeval].semptr[4]);

        // drive sem4 to zero
        while(sem_trywait(data.image[IDmodeval].semptr[4])==0) {}
        AOconf[loop].statusM = 3;
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[3] = tdiffv;

		LOOPiter = data.image[IDmodeval].md[0].cnt1;


        // write gain, mult, limit into arrays
        for(m=0; m<NBmodes; m++)
        {
            modegain[m] = AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m];
            modemult[m] = AOconf[loop].mult * data.image[aoloopcontrol_var.aoconfID_multfb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[m];
            modelimit[m] = AOconf[loop].maxlimit * data.image[aoloopcontrol_var.aoconfID_limitb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m];
        }


        //
        // UPDATE CURRENT DM MODES STATE
        //
        //  current state =   modemult   x   ( last state   - modegain * WFSmodeval  )
        //
        // modevalDMindexl = last index in the IDmodevalDM_C buffer
        //
        
        data.image[IDmodevalDMcorr].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDMcorr].array.F[m] = modemult[m]*(data.image[IDmodevalDM_C].array.F[modevalDMindexl*NBmodes+m] - modegain[m]*data.image[IDmodeval].array.F[m]);
		COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMcorr, -1);
		data.image[IDmodevalDMcorr].md[0].cnt1 = LOOPiter; //modevalPFindex; //TBC
		data.image[IDmodevalDMcorr].md[0].cnt0++;
		data.image[IDmodevalDMcorr].md[0].write = 0;


        AOconf[loop].statusM = 4;
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[4] = tdiffv;

	

		int ARPF_ok;
		ARPF_ok = 0;

        //
        //  MIX PREDICTION WITH CURRENT DM STATE
        //
        if(AOconf[loop].ARPFon==1)
        {
		//	printf("%s  %s  %d\n",__FILE__, __func__, __LINE__);fflush(stdout); //TEST
			
            if(aoloopcontrol_var.aoconfID_modevalPF==-1)
            {
                if(sprintf(imname, "aol%ld_modevalPF", loop) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                aoloopcontrol_var.aoconfID_modevalPF = read_sharedmem_image(imname);
            }
            else
            {
				ARPF_ok=1;
                // don't wait for modevalPF... assume it's here early
                
                // if waiting for modevalPF, uncomment this line, and the "drive semaphore to zero" line after the next loop
                //sem_wait(data.image[IDmodevalPF].semptr[3]);

				//
				// prediction is mixed here with non-predictive output of WFS
				// minus sign required to apply correction on DM (correction should be opposite of WFS measurement)
				// note that second term (non-predictive) does not have minus sign, as it was already applied above
				//
				
				
                for(m=0; m<NBmodes; m++)
                {
					float mixratio;
					
					mixratio = AOconf[loop].ARPFgain*data.image[IDmodeARPFgain].array.F[m] * data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m];
				    data.image[IDmodevalDMnow].array.F[m] = -mixratio*data.image[aoloopcontrol_var.aoconfID_modevalPF].array.F[m]  + (1.0-mixratio)*data.image[IDmodevalDMcorr].array.F[m];
                }
             
                // drive semaphore to zero
				//  while(sem_trywait(data.image[aoloopcontrol_var.aoconfID_modevalPF].semptr[3])==0) {}


				//
				// update current location of prediction circular buffer
				//
				data.image[IDmodevalPF_C].md[0].write = 1;
				for(m=0; m<NBmodes; m++)
					data.image[IDmodevalPF_C].array.F[modevalPFindex*NBmodes+m] = data.image[aoloopcontrol_var.aoconfID_modevalPF].array.F[m];
				COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPF_C, -1);
				data.image[IDmodevalPF_C].md[0].cnt1 = modevalPFindex; // NEEDS TO CONTAIN WRITTEN SLICE ?
				data.image[IDmodevalPF_C].md[0].cnt0++;
				data.image[IDmodevalPF_C].md[0].write = 0;
					
					
					
				// autotune ARPFgainAuto
				data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].md[0].write = 1;
				for(m=0; m<NBmodes; m++)
				{
					float minVal = AOconf[loop].ARPFgainAutoMin;
					float maxVal = AOconf[loop].ARPFgainAutoMax;
					
					
					if (data.image[IDmodevalPFres].array.F[m]*data.image[IDmodevalPFres].array.F[m] < data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m])
						data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] *= 1.001;
					else
						data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] *= 0.999;
						
					if (data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] > maxVal)
						data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = maxVal;
					if (data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] < minVal)
						data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = minVal;
				}
				data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].md[0].cnt1 = LOOPiter; //modevalPFindex;
				data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].md[0].cnt0++;
				data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].md[0].write = 0;
								
				loopPFcnt++;
            }
        }

		if (ARPF_ok==0)
		{ 
			memcpy(data.image[IDmodevalDMnow].array.F, data.image[IDmodevalDMcorr].array.F, sizeof(float)*NBmodes);
		}






        AOconf[loop].statusM = 5;
			clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[5] = tdiffv;

        data.image[IDmodevalDMnowfilt].md[0].write = 1;
        // FILTERING MODE VALUES
        // THIS FILTERING GOES TOGETHER WITH THE SECONDARY WRITE ON DM TO KEEP FILTERED AND ACTUAL VALUES IDENTICAL
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDMnowfilt].array.F[m] = data.image[IDmodevalDMnow].array.F[m];


        if(AOconf[loop].AUTOTUNE_LIMITS_ON==1) // automatically adjust modal limits
        {
            data.image[IDatlimbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
                limitblockarray[block] = 0.0;

            data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].write = 1;
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].write = 1;

			// Adjust limit for EACH mode
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                if(  fabs(AOconf[loop].AUTOTUNE_LIMITS_mcoeff*data.image[IDmodevalDMnowfilt].array.F[m]) > modelimit[m])
                    data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m] *= (1.0 + AOconf[loop].AUTOTUNE_LIMITS_delta);
                else
                    data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m] *= (1.0 - AOconf[loop].AUTOTUNE_LIMITS_delta*0.01*AOconf[loop].AUTOTUNE_LIMITS_perc);

                limitblockarray[block] += data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m];
            }
            COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_LIMIT_modes, -1);
            data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].write = 0;

			// update block limits to drive average limit coefficients to 1
            data.image[IDatlimbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
                data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AUTOTUNE_LIMITS_delta*0.1 );
                if(coeff < 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta;
                if(coeff> 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta;
                data.image[aoloopcontrol_var.aoconfID_limitb].array.F[block] = data.image[aoloopcontrol_var.aoconfID_limitb].array.F[block] * coeff;
            }
            COREMOD_MEMORY_image_set_sempost_byID(IDatlimbcoeff, -1);
            data.image[IDatlimbcoeff].md[0].cnt0++;
            data.image[IDatlimbcoeff].md[0].cnt1 = LOOPiter;
            data.image[IDatlimbcoeff].md[0].write = 0;


            COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_limitb, -1);
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].write = 0;

        }


		if(AOconf[loop].AUTOTUNE_GAINS_ON==1)
		{
			// CONNECT to auto gain input
			if(IDautogain == -1)
			{
				if(sprintf(imname, "aol%ld_autogain", loop) < 1)
					printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

				IDautogain = read_sharedmem_image(imname);				
			}
			
			if(IDautogain != -1)
			{
				if(data.image[IDautogain].md[0].cnt0 != autogainCnt)
				{
					float maxGainVal = 0.0;
					float globalgain = 0.0;
					char command[500];
					
					// New gains available - updating
					printf("[%ld %s] Updated autogain [%12ld  %12ld] -> applying gains\n", IDautogain, data.image[IDautogain].md[0].name, (long) autogainCnt, (long) data.image[IDautogain].md[0].cnt0);
					fflush(stdout);
					
					// Set global gain to highest gain
					for(m=0;m<NBmodes;m++)
					{
						if(data.image[IDautogain].array.F[m] > maxGainVal)
							maxGainVal = data.image[IDautogain].array.F[m];
					}					
					globalgain = maxGainVal;
					printf("     Setting  global gain = %f\n", maxGainVal);
					AOconf[loop].gain = maxGainVal;

					sprintf(command, "echo \"%6.4f\" > conf/param_loopgain.txt", AOconf[loop].gain);
					system(command);


					
					// Set block gain to max gain within block, scaled to global gain
					for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
					{
						maxGainVal = 0.0;
						for(m=0; m<NBmodes; m++)
							if(data.image[IDblknb].array.UI16[m] == block)
								if(data.image[IDautogain].array.F[m] > maxGainVal)
									maxGainVal = data.image[IDautogain].array.F[m];
					
						printf("Set block %2ld gain to  %f\n", block, maxGainVal/globalgain);
					
						data.image[aoloopcontrol_var.aoconfID_gainb].array.F[block] = maxGainVal/AOconf[loop].gain;

						
						sprintf(command, "echo \"%6.4f\" > conf/param_gainb%02ld.txt", data.image[aoloopcontrol_var.aoconfID_gainb].array.F[block], block);
						system(command);
					}
					
					// Set individual gain
					for(m=0;m<NBmodes;m++)
					{
						data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m] = data.image[IDautogain].array.F[m]/data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]]/AOconf[loop].gain;
						
						if(m<20)
							printf("Mode %3ld   %12f  %12f  %12f ->   %12f  %12f\n", m, AOconf[loop].gain, data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]], data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m], AOconf[loop].gain*data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]]*data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m], data.image[IDautogain].array.F[m]);
					}
					
					
					
					autogainCnt = data.image[IDautogain].md[0].cnt0;
				}
				
			}
	
/*			float alphagain = 0.1;
			
			// if update available
			
			data.image[aoconfID_GAIN_modes].md[0].write = 1;
            data.image[aoconfID_gaiinb].md[0].write = 1;

			// Adjust gain for EACH mode
			
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                data.image[aoconfID_GAIN_modes].array.F[m] = (1.0-alphagain)*data.image[aoconfID_GAIN_modes].array.F[m] + alphagain*data.image[autogain].array.F[m];

                gainblockarray[block] += data.image[aoconfID_GAIN_modes].array.F[m];
            }
            COREMOD_MEMORY_image_set_sempost_byID(aoconfID_GAIN_modes, -1);
            data.image[aoconfID_GAIN_modes].md[0].cnt0++;
            data.image[aoconfID_GAIN_modes].md[0].write = 0;

			// update block gains to drive average gain coefficients to 1
            data.image[IDatgainbcoeff].md[0].write = 1;
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
                data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AUTOTUNE_LIMITS_delta*0.1 );
                if(coeff < 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0-AOconf[loop].AUTOTUNE_LIMITS_delta;
                if(coeff> 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta )
                    coeff = 1.0+AOconf[loop].AUTOTUNE_LIMITS_delta;
                data.image[aoloopcontrol_var.aoconfID_limitb].array.F[block] = data.image[aoloopcontrol_var.aoconfID_limitb].array.F[block] * coeff;
            }
            COREMOD_MEMORY_image_set_sempost_byID(IDatlimbcoeff, -1);
            data.image[IDatlimbcoeff].md[0].cnt0++;
            data.image[IDatlimbcoeff].md[0].write = 0;


            COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_limitb, -1);
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_limitb].md[0].write = 0;
            */
		}



        if(FILTERMODE == 1)
        {
            for(m=0; m<NBmodes; m++)
            {
                block = data.image[IDblknb].array.UI16[m];

                if(data.image[IDmodevalDMnowfilt].array.F[m] > modelimit[m])
                {
                    blockavelimFrac[block] += 1.0;
                    data.image[IDmodevalDMnowfilt].array.F[m] = modelimit[m];
                }
                if(data.image[IDmodevalDMnowfilt].array.F[m] < -modelimit[m])
                {
                    blockavelimFrac[block] += 1.0;
                    data.image[IDmodevalDMnowfilt].array.F[m] = -modelimit[m];
                }
            }
        }



        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnow, -1);
        data.image[IDmodevalDMnow].md[0].cnt1 = LOOPiter; //modevalDMindex; //TBC
        data.image[IDmodevalDMnow].md[0].cnt0++;
        data.image[IDmodevalDMnow].md[0].write = 0;

		if(AOconf[loop].DMfilteredWriteON==1)
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDMnowfilt, -1);
        else
			{
				// DM write triggered by sem 2
				// posting all sems except 2 to block DM write
				COREMOD_MEMORY_image_set_sempost_excl_byID(IDmodevalDMnowfilt, 2);
			}
        data.image[IDmodevalDMnowfilt].md[0].cnt1 = LOOPiter; //modevalDMindex; //TBC
        data.image[IDmodevalDMnowfilt].md[0].cnt0++;
        data.image[IDmodevalDMnowfilt].md[0].write = 0;

		// IF MODAL DM, AND FILTERED DM WRITE IS ON, SEND TO DM
		if((AOconf[loop].DMfilteredWriteON==1) && (AOconf[loop].DMMODE==1))
		{
			data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 1;
			memcpy(data.image[aoloopcontrol_var.aoconfID_dmC].array.F, data.image[IDmodevalDMnowfilt].array.F, sizeof(float)*NBmodes);
			COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_dmC, -1);
			data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = LOOPiter;
			data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0++;
			data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 0;			
		}


        AOconf[loop].statusM = 6;
        
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[6] = tdiffv;
        
		AOconf[loop].statusM1 = 0;


        //
        // update current location of dm correction circular buffer
        //
        data.image[IDmodevalDM_C].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDM_C].array.F[modevalDMindex*NBmodes+m] = data.image[IDmodevalDMnowfilt].array.F[m];
        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM_C, -1);
        data.image[IDmodevalDM_C].md[0].cnt1 = LOOPiter; //modevalDMindex;
        data.image[IDmodevalDM_C].md[0].cnt0++;
        data.image[IDmodevalDM_C].md[0].write = 0;



        AOconf[loop].statusM1 = 1;

        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[9] = tdiffv;

        //
        // COMPUTE DM STATE AT TIME OF WFS MEASUREMENT
        // LINEAR INTERPOLATION BETWEEN NEAREST TWO VALUES
        //
        modevalDMindex0 = modevalDMindex - framelatency0;
        if(modevalDMindex0<0)
            modevalDMindex0 += modeval_bsize;
        modevalDMindex1 = modevalDMindex - framelatency1;
        if(modevalDMindex1<0)
            modevalDMindex1 += modeval_bsize;

        data.image[IDmodevalDM].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDmodevalDM].array.F[m] = (1.0-alpha)*data.image[IDmodevalDM_C].array.F[modevalDMindex0*NBmodes+m] + alpha*data.image[IDmodevalDM_C].array.F[modevalDMindex1*NBmodes+m];
        COREMOD_MEMORY_image_set_sempost_byID(IDmodevalDM, -1);
        data.image[IDmodevalDM].md[0].cnt0++;
        data.image[IDmodevalDM].md[0].cnt1 = LOOPiter;
        data.image[IDmodevalDM].md[0].write = 0;

		AOconf[loop].statusM1 = 2;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[10] = tdiffv;



		if(AOconf[loop].ARPFon==1)
		{
			//
			// COMPUTE OPEN LOOP PREDICTION AT TIME OF WFS MEASUREMENT
			// LINEAR INTERPOLATION BETWEEN NEAREST TWO VALUES
			//
        
			modevalPFindex0 = modevalPFindex - framelatency0;
			if(modevalPFindex0<0)
				modevalPFindex0 += modeval_bsize;
			modevalPFindex1 = modevalPFindex - framelatency1;
			if(modevalPFindex1<0)
				modevalPFindex1 += modeval_bsize;

			data.image[IDmodevalPFsync].md[0].write = 1;
			for(m=0; m<NBmodes; m++)
				data.image[IDmodevalPFsync].array.F[m] = (1.0-alpha)*data.image[IDmodevalPF_C].array.F[modevalPFindex0*NBmodes+m] + alpha*data.image[IDmodevalPF_C].array.F[modevalPFindex1*NBmodes+m];
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPFsync, -1);
			data.image[IDmodevalPFsync].md[0].cnt0++;
			data.image[IDmodevalPFsync].md[0].cnt1 = LOOPiter;
			data.image[IDmodevalPFsync].md[0].write = 0;
		}



        AOconf[loop].statusM1 = 3;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[11] = tdiffv;


        //
        // OPEN LOOP STATE = most recent WFS reading - time-lagged DM
        //
        data.image[IDout].md[0].write = 1;
        for(m=0; m<NBmodes; m++)
            data.image[IDout].array.F[m] = data.image[IDmodeval].array.F[m] - data.image[IDmodevalDM].array.F[m];
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = LOOPiter;
        data.image[IDout].md[0].write = 0;


		if(AOconf[loop].ARPFon==1)
		{
			//
			// OPEN LOOP PREDICTION RESIDUAL = most recent OL - time-lagged PF
			//
			data.image[IDmodevalPFres].md[0].write = 1;
			for(m=0; m<NBmodes; m++)
				data.image[IDmodevalPFres].array.F[m] = data.image[IDout].array.F[m] - data.image[IDmodevalPFsync].array.F[m];
			COREMOD_MEMORY_image_set_sempost_byID(IDmodevalPFres, -1);
			data.image[IDmodevalPFres].md[0].cnt0++;
			data.image[IDmodevalPFres].md[0].cnt1 = LOOPiter;
			data.image[IDmodevalPFres].md[0].write = 0;
		}



        AOconf[loop].statusM1 = 4;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[12] = tdiffv;


		// increment modevalDMindex
        modevalDMindexl = modevalDMindex;
        modevalDMindex++;
        if(modevalDMindex==modeval_bsize)
            modevalDMindex = 0;

		// increment modevalPFindex
        modevalPFindexl = modevalPFindex;
        modevalPFindex++;
        if(modevalPFindex==modeval_bsize)
            modevalPFindex = 0;





        // TELEMETRY
        for(m=0; m<NBmodes; m++)
        {
            block = data.image[IDblknb].array.UI16[m];
			
			
			blockavePFresrms[block] += data.image[IDmodevalPFres].array.F[m]*data.image[IDmodevalPFres].array.F[m];
            blockaveOLrms[block] += data.image[IDout].array.F[m]*data.image[IDout].array.F[m];
            blockaveCrms[block] += data.image[IDmodevalDMnow].array.F[m]*data.image[IDmodevalDMnow].array.F[m];
            blockaveWFSrms[block] += data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m];          
			
			blockaveWFSnoise[block] += data.image[IDmodeWFSnoise].array.F[m];
        }
        

        blockstatcnt ++;
        if(blockstatcnt == AOconf[loop].AveStats_NBpt)
        {
            for(block=0; block<AOconf[loop].DMmodesNBblock; block++)
            {
				AOconf[loop].blockave_PFresrms[block] = sqrt(blockavePFresrms[block]/blockstatcnt);
                AOconf[loop].blockave_OLrms[block] = sqrt(blockaveOLrms[block]/blockstatcnt);
                AOconf[loop].blockave_Crms[block] = sqrt(blockaveCrms[block]/blockstatcnt);
                AOconf[loop].blockave_WFSrms[block] = sqrt(blockaveWFSrms[block]/blockstatcnt);
				AOconf[loop].blockave_WFSnoise[block] = sqrt(blockaveWFSnoise[block]/blockstatcnt);
                AOconf[loop].blockave_limFrac[block] = (blockavelimFrac[block])/blockstatcnt;

				allavePFresrms += blockavePFresrms[block];
                allaveOLrms += blockaveOLrms[block];
                allaveCrms += blockaveCrms[block];
                allaveWFSrms += blockaveWFSrms[block];
                allaveWFSnoise += blockaveWFSnoise[block];
                allavelimFrac += blockavelimFrac[block];

				blockavePFresrms[block] = 0.0;
                blockaveOLrms[block] = 0.0;
                blockaveCrms[block] = 0.0;
                blockaveWFSrms[block] = 0.0;
                blockaveWFSnoise[block] = 0.0;
                blockavelimFrac[block] = 0.0;                
            }

            AOconf[loop].ALLave_OLrms = sqrt(allaveOLrms/blockstatcnt);
            AOconf[loop].ALLave_Crms = sqrt(allaveCrms/blockstatcnt);
            AOconf[loop].ALLave_WFSrms = sqrt(allaveWFSrms/blockstatcnt);
			AOconf[loop].ALLave_WFSnoise = sqrt(allaveWFSnoise/blockstatcnt);
            AOconf[loop].ALLave_limFrac = allavelimFrac/blockstatcnt;

			allavePFresrms = 0.0;
            allaveOLrms = 0.0;
            allaveCrms = 0.0;
            allaveWFSrms = 0.0;
            allavelimFrac = 0.0;

            blockstatcnt = 0;
        }

		


        AOconf[loop].statusM1 = 5;
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[13] = tdiffv;

    }

    free(modegain);
    free(modemult);
    free(modelimit);

    free(modeblock);

    return(IDout);
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


    int TESTMODE = 1;
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










long AOloopControl_dm2dm_offload(const char *streamin, const char *streamout, float twait, float offcoeff, float multcoeff)
{
    long IDin, IDout;
    long cnt = 0;
    long xsize, ysize, xysize;
    long ii;
    //long IDtmp;
	char imname[200];

    IDin = image_ID(streamin);
    IDout = image_ID(streamout);

    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
    xysize = xsize*ysize;
    


	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    while(1)
    {
        printf("%8ld : offloading   %s -> %s\n", cnt, streamin, streamout);

        data.image[IDout].md[0].write = 1;
        for(ii=0; ii<xysize; ii++)
            data.image[IDout].array.F[ii] = multcoeff*(data.image[IDout].array.F[ii] + offcoeff*data.image[IDin].array.F[ii]);
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDout].md[0].write = 0;

        usleep((long) (1000000.0*twait));
        cnt++;
    }

    return(IDout);
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
