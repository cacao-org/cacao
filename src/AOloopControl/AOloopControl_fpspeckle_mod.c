/**
 * @file    AOloopControl_fpspeckle_mod.c
 * @brief   AO loop control - FOCAL PLANE SPECKLE MODULATION / CONTROL   
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 *
 * 
 */
 
 
#define _GNU_SOURCE

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

#include <string.h>
#include <math.h>

// defined in AOloopControl.c
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var;





// optimize LO - uses simulated downhill simplex
errno_t AOloopControl_OptimizePSF_LO(
    __attribute__((unused)) const char *psfstream_name,
    const char *IDmodes_name,
    const char *dmstream_name,
    __attribute__((unused)) long        delayframe,
    __attribute__((unused)) long        NBframes
)
{
    imageID IDmodes;
    imageID IDdmstream;
    imageID IDdm;
    //    long psfxsize, psfysize;
    uint32_t dmxsize, dmysize;
    uint32_t NBmodes;
    double ampl;
    double x;
   
    imageID IDdmbest;

    char imname[200];


	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    ampl = 0.01; // modulation amplitude

 //   IDpsf = image_ID(psfstream_name);
    IDmodes    = image_ID(IDmodes_name);
    IDdmstream = image_ID(dmstream_name);

    dmxsize = data.image[IDdmstream].md[0].size[0];
    dmysize = data.image[IDdmstream].md[0].size[1];

//    psfxsize = data.image[IDpsf].md[0].size[0];
//    psfysize = data.image[IDpsf].md[0].size[1];

    IDdmbest = create_2Dimage_ID("dmbest", dmxsize, dmysize);
    IDdm     = create_2Dimage_ID("dmcurr", dmxsize, dmysize);



    NBmodes = data.image[IDmodes].md[0].size[2];

    for(uint64_t ii=0; ii<dmxsize*dmysize; ii++)
        data.image[IDdmbest].array.F[ii] = data.image[IDdm].array.F[ii];


    for(uint32_t mode=0; mode<NBmodes; mode ++)
    {
        for(x=-ampl; x<1.01*ampl; x += ampl)
        {
            // apply DM pattern
            for(uint64_t ii=0; ii<dmxsize*dmysize; ii++)
                data.image[IDdm].array.F[ii] = data.image[IDdmbest].array.F[ii]+ampl*data.image[IDmodes].array.F[dmxsize*dmysize*mode+ii];

            data.image[IDdmstream].md[0].write = 1;
            memcpy(data.image[IDdmstream].array.F, data.image[IDdm].array.F, sizeof(float)*dmxsize*dmysize);
            data.image[IDdmstream].md[0].cnt0++;
			data.image[IDdmstream].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
            data.image[IDdmstream].md[0].write = 0;



        }
    }


    return RETURN_SUCCESS;
}


//
// modulate using linear combination of two probes A and B
//
//
// delay is in sec
//
errno_t AOloopControl_DMmodulateAB(
    const char *IDprobeA_name,
    const char *IDprobeB_name,
    const char *IDdmstream_name,
    const char *IDrespmat_name,
    const char *IDwfsrefstream_name,
    double      delay,
    long        NBprobes
)
{
    imageID IDprobeA;
    imageID IDprobeB;
    long    dmxsize, dmysize;
    long    dmsize;
    imageID IDdmstream;

    imageID IDrespmat;
    imageID IDwfsrefstream;
    long    wfsxsize, wfsysize;
    long    wfssize;

    imageID IDdmC;
    imageID IDwfsrefC;

    float *coeffA;
    float *coeffB;
    int k;
    long act, wfselem;

    char imname[200];

    FILE *fp;
    char flogname[200];
    int loopOK;
    long dmframesize, wfsframesize;
    char timestr[200];
    time_t t;
    struct tm *uttime;
    struct timespec *thetime = (struct timespec *)malloc(sizeof(struct timespec));
    long ii;
    int semval;



	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", aoloopcontrol_var.LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}


    IDprobeA = image_ID(IDprobeA_name);
    dmxsize = data.image[IDprobeA].md[0].size[0];
    dmysize = data.image[IDprobeA].md[0].size[1];
    dmsize = dmxsize*dmysize;

    IDprobeB = image_ID(IDprobeB_name);
    IDdmstream = image_ID(IDdmstream_name);
    IDrespmat = image_ID(IDrespmat_name);
    IDwfsrefstream = image_ID(IDwfsrefstream_name);
    wfsxsize = data.image[IDwfsrefstream].md[0].size[0];
    wfsysize = data.image[IDwfsrefstream].md[0].size[1];
    wfssize = wfsxsize*wfsysize;

    coeffA = (float*) malloc(sizeof(float)*NBprobes);
    coeffB = (float*) malloc(sizeof(float)*NBprobes);

    IDdmC = create_3Dimage_ID("MODdmC", dmxsize, dmysize, NBprobes);
    IDwfsrefC = create_3Dimage_ID("WFSrefC", wfsxsize, wfsysize, NBprobes);

    coeffA[0] = 0.0;
    coeffB[0] = 0.0;
    for(k=1; k<NBprobes; k++)
    {
        coeffA[k] = cos(2.0*M_PI*(k-1)/(NBprobes-1));
        coeffB[k] = sin(2.0*M_PI*(k-1)/(NBprobes-1));
    }


    // prepare MODdmC and WFSrefC
    for(k=0; k<NBprobes; k++)
    {
        for(act=0; act<dmsize; act++)
            data.image[IDdmC].array.F[k*dmsize+act] = coeffA[k]*data.image[IDprobeA].array.F[act] + coeffB[k]*data.image[IDprobeB].array.F[act];

        for(wfselem=0; wfselem<wfssize; wfselem++)
            for(act=0; act<dmsize; act++)
                data.image[IDwfsrefC].array.F[k*wfssize+wfselem] += data.image[IDdmC].array.F[k*dmsize+act]*data.image[IDrespmat].array.F[act*wfssize+wfselem];
    }

    save_fl_fits("MODdmC", "!test_MODdmC.fits");
    save_fl_fits("WFSrefC", "!test_WFSrefC.fits");

    t = time(NULL);
    uttime = gmtime(&t);
    if(sprintf(flogname, "logfpwfs_%04d-%02d-%02d_%02d:%02d:%02d.txt", 1900+uttime->tm_year, 1+uttime->tm_mon, uttime->tm_mday, uttime->tm_hour, uttime->tm_min, uttime->tm_sec) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if((fp=fopen(flogname,"w"))==NULL)
    {
        printf("ERROR: cannot create file \"%s\"\n", flogname);
        exit(0);
    }
    fclose(fp);


    dmframesize = sizeof(float)*dmsize;
    wfsframesize = sizeof(float)*wfssize;

    list_image_ID();



    if (sigaction(SIGINT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGTERM, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGBUS, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    /*   if (sigaction(SIGSEGV, &data.sigact, NULL) == -1) {
           perror("sigaction");
           exit(EXIT_FAILURE);
       }*/
    if (sigaction(SIGABRT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGHUP, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGPIPE, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }



    k = 0;
    loopOK = 1;

    while(loopOK == 1)
    {
        printf("Applying probe # %d   %ld %ld\n", k, IDdmstream, IDwfsrefstream);
        fflush(stdout);

        // apply probe
        char *ptr0;
        ptr0 = (char*) data.image[IDdmC].array.F;
        ptr0 += k*dmframesize;
        data.image[IDdmstream].md[0].write = 1;
        memcpy(data.image[IDdmstream].array.F, (void*) ptr0, dmframesize);
        sem_getvalue(data.image[IDdmstream].semptr[0], &semval);
        if(semval<SEMAPHORE_MAXVAL)
            sem_post(data.image[IDdmstream].semptr[0]);
        data.image[IDdmstream].md[0].cnt0++;
        data.image[IDdmstream].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDdmstream].md[0].write = 0;

        // apply wfsref offset
        ptr0 = (char*) data.image[IDwfsrefC].array.F;
        ptr0 += k*wfsframesize;
        data.image[IDwfsrefstream].md[0].write = 1;
        memcpy(data.image[IDwfsrefstream].array.F, (void*) ptr0, wfsframesize);
        sem_getvalue(data.image[IDwfsrefstream].semptr[0], &semval);
        if(semval<SEMAPHORE_MAXVAL)
            sem_post(data.image[IDwfsrefstream].semptr[0]);
        data.image[IDwfsrefstream].md[0].cnt0++;
        data.image[IDwfsrefstream].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        data.image[IDwfsrefstream].md[0].write = 0;

        // write time in log
        t = time(NULL);
        uttime = gmtime(&t);
        clock_gettime(CLOCK_REALTIME, thetime);

        if(sprintf(timestr, "%02d %02d %02d.%09ld", uttime->tm_hour, uttime->tm_min, uttime->tm_sec, thetime->tv_nsec) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        printf("time = %s\n", timestr);
        if((fp = fopen(flogname, "a"))==NULL)
        {
            printf("ERROR: cannot open file \"%s\"\n", flogname);
            exit(0);
        }
        fprintf(fp, "%s %2d %10f %10f\n", timestr, k, coeffA[k], coeffB[k]);
        fclose(fp);

        usleep((long) (1.0e6*delay));
        k++;
        if(k==NBprobes)
            k = 0;

        if((data.signal_INT == 1)||(data.signal_TERM == 1)||(data.signal_ABRT==1)||(data.signal_BUS==1)||(data.signal_SEGV==1)||(data.signal_HUP==1)||(data.signal_PIPE==1))
            loopOK = 0;
    }

    data.image[IDdmstream].md[0].write = 1;
    for(ii=0; ii<dmsize; ii++)
        data.image[IDdmstream].array.F[ii] = 0.0;
    sem_getvalue(data.image[IDdmstream].semptr[0], &semval);
    if(semval<SEMAPHORE_MAXVAL)
        sem_post(data.image[IDdmstream].semptr[0]);
    data.image[IDdmstream].md[0].cnt0++;
    data.image[IDdmstream].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
    data.image[IDdmstream].md[0].write = 0;


    data.image[IDwfsrefstream].md[0].write = 1;
    for(ii=0; ii<wfssize; ii++)
        data.image[IDwfsrefstream].array.F[ii] = 0.0;
    sem_getvalue(data.image[IDwfsrefstream].semptr[0], &semval);
    if(semval<SEMAPHORE_MAXVAL)
        sem_post(data.image[IDwfsrefstream].semptr[0]);
    data.image[IDwfsrefstream].md[0].cnt0++;
    data.image[IDwfsrefstream].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
    data.image[IDwfsrefstream].md[0].write = 0;



    free(coeffA);
    free(coeffB);


    return RETURN_SUCCESS;
}


