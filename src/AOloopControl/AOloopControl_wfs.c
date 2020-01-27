/**
 * @file    AOloopControl_wfs.c 
 * @brief   AO loop Control functions wave front sensor and deformable mirror 
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * 
 */




#define _GNU_SOURCE



#include <string.h>
#include <gsl/gsl_blas.h>
#include <pthread.h>


#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "info/info.h" 

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

static int initWFSref_GPU[100] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static long long aoconfcnt0_contrM_current= -1; 

static long wfsrefcnt0 = -1; 
static long contrMcactcnt0[100] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};;



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






errno_t AOloopControl_WFSzpupdate_loop(
    const char *IDzpdm_name,
    const char *IDzrespM_name,
    const char *IDwfszp_name
)
{
    imageID IDzpdm, IDzrespM, IDwfszp;
    uint32_t dmxsize, dmysize, dmxysize;
    long wfsxsize, wfsysize, wfsxysize;
    imageID IDtmp;
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

    return RETURN_SUCCESS;
}





//
// Create zero point WFS channels
// watch semaphore 1 on output (IDwfsref_name) -> sum all channels to update WFS zero point
// runs in separate process from RT computation
//
//
//
errno_t AOloopControl_WFSzeropoint_sum_update_loop(
    long        loopnb,
    const char *ID_WFSzp_name,
    int         NBzp,
    const char *IDwfsref0_name,
    const char *IDwfsref_name
)
{
    long wfsxsize, wfsysize, wfsxysize;
    imageID IDwfsref, IDwfsref0;
    imageID *IDwfszparray;
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

    IDwfszparray = (long*) malloc(sizeof(long)*(NBzp+1));
    // create / read the zero point WFS channels
    for(ch=0; ch<NBzp; ch++)
    {
        if(sprintf(imname, "%s%ld", ID_WFSzp_name, ch) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        AOloopControl_IOtools_2Dloadcreate_shmim(imname, "", wfsxsize, wfsysize, 0.0);
        COREMOD_MEMORY_image_set_createsem(imname, 10);
        IDwfszparray[ch] = image_ID(imname);
    }
    // extra special zp channel
    ch = NBzp;
    if(sprintf(imname, "%s_00", ID_WFSzp_name) < 1)
		printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    AOloopControl_IOtools_2Dloadcreate_shmim(imname, "", wfsxsize, wfsysize, 0.0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    IDwfszparray[ch] = image_ID(imname);
    

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
        for(ch=0; ch<(NBzp+1); ch++)
            cntsum += data.image[IDwfszparray[ch]].md[0].cnt0;


        if(cntsum != cntsumold)
        {
            // copy wfsref0 to tmp
            memcpy(data.image[IDtmp].array.F, data.image[IDwfsref0].array.F, sizeof(float)*wfsxysize);

            for(ch=0; ch<(NBzp+1); ch++)
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


    return RETURN_SUCCESS;
}



errno_t ControlMatrixMultiply(
    float *cm_array,
    float *imarray,
    long m,
    long n,
    float *outvect
)
{
    long i;

    cblas_sgemv (CblasRowMajor, CblasNoTrans, m, n, 1.0, cm_array, n, imarray, 1, 0.0, outvect, 1);

    return RETURN_SUCCESS;
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

imageID AOloopControl_computeWFSresidualimage(
    long loop,
    char *IDalpha_name
)
{
    imageID IDwfsref, IDwfsmask, IDtot, IDout, IDoutave, IDoutm, IDoutmave, IDoutrms;
    char imname[200];
    uint32_t *sizearray;
    long wfsxsize, wfsysize, wfsxysize;
    long cnt;
    long ii;
    imageID IDalpha;

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

