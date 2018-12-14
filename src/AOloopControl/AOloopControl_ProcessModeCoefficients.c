/**
 * @file    AOloopControl_ProcessModeCoefficients.c 
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


#include <time.h>



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







extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;

















// includes mode filtering (limits, multf)
//
long __attribute__((hot)) AOloopControl_ProcessModeCoefficients(long loop)
{
    // TIMING
    struct timespec tnow;
    struct timespec tdiff;
    double tdiffv;

    long IDout;
    long IDmodeval; // WFS measurement

    int ret;

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

    PROCESSINFO *processinfo;
    if(data.processinfo==1)
    {
        // CREATE PROCESSINFO ENTRY
        // see processtools.c in module CommandLineInterface for details
        //
        char pinfoname[200];
        sprintf(pinfoname, "aol%ld-ProcessModeCoefficients", loop);
        processinfo = processinfo_shm_create(pinfoname, 0);
        processinfo->loopstat = 0; // loop initialization

        strcpy(processinfo->source_FUNCTION, __FUNCTION__);
        strcpy(processinfo->source_FILE,     __FILE__);
        processinfo->source_LINE = __LINE__;
        
        sprintf(processinfo->description, "Modal Processing");

        char msgstring[200];
        sprintf(msgstring, "Initialization");
        processinfo_WriteMessage(processinfo, msgstring);
    }


    // CATCH SIGNALS

    if (sigaction(SIGTERM, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGTERM\n");

    if (sigaction(SIGINT, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGINT\n");

    if (sigaction(SIGABRT, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGABRT\n");

    if (sigaction(SIGBUS, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGBUS\n");

    if (sigaction(SIGSEGV, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGSEGV\n");

    if (sigaction(SIGHUP, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGHUP\n");

    if (sigaction(SIGPIPE, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGPIPE\n");




    // LOG function start
    int logfunc_level = 0;
    int logfunc_level_max = 1;
    char commentstring[200];
    sprintf(commentstring, "loop %ld", loop);
    CORE_logFunctionCall( logfunc_level, logfunc_level_max, 0, __FILE__, __func__, __LINE__, commentstring);



    // read AO loop gain, mult
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    // INPUT
    if(sprintf(imname, "aol%ld_modeval", loop) < 1)// measured from WFS
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodeval = read_sharedmem_image(imname);
    int modeval_semwaitindex = ImageStreamIO_getsemwaitindex(data.image[IDmodeval], 4);
    //data.image[IDmodeval].semReadPID[modeval_semwaitindex] = getpid();


    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval] = 1;

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
    if(sprintf(imname, "aol%ld_dmC", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    if(aoloopcontrol_var.aoconfID_dmC == -1)
        aoloopcontrol_var.aoconfID_dmC = read_sharedmem_image(imname);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_dmC, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_dmC] = 1;



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
        if(sprintf(imname, "aol%ld_DMmode_GAIN", loop) < 1)
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



    //TEST
    FILE *fptestmPF; //TEST
    fptestmPF = fopen("modevalPF.test", "w");//TEST

    fprintf(fptestmPF, "%4d  aoloopcontrol_var.aoconfID_modevalPF =  %ld\n", __LINE__, aoloopcontrol_var.aoconfID_modevalPF);//TEST

    // predictive control output
    // Try to read existing modevalPF if exists
    if(aoloopcontrol_var.aoconfID_modevalPF == -1)
    {

        if(sprintf(imname, "aol%ld_modevalPF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_modevalPF = read_sharedmem_image(imname);


        fprintf(fptestmPF, "%4d  aoloopcontrol_var.aoconfID_modevalPF =  %ld\n", __LINE__, aoloopcontrol_var.aoconfID_modevalPF);//TEST

        if(aoloopcontrol_var.aoconfID_modevalPF != -1)
        {
            fprintf(fptestmPF, "%4d  aoloopcontrol_var.aoconfID_modevalPF =  %ld\n", __LINE__, aoloopcontrol_var.aoconfID_modevalPF);//TEST

            long ii;
            for(ii=0; ii<data.image[aoloopcontrol_var.aoconfID_modevalPF].md[0].size[0]*data.image[aoloopcontrol_var.aoconfID_modevalPF].md[0].size[1]; ii++)
                data.image[aoloopcontrol_var.aoconfID_modevalPF].array.F[ii] = 0.0;

            AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modevalPF, imname);
            aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modevalPF] = 1;
        }
    }
    fclose(fptestmPF);//TEST

    // OUPUT
    sizeout = (uint32_t*) malloc(sizeof(uint32_t)*3);
    sizeout[0] = NBmodes;
    sizeout[1] = 1;

    // all images below are vectors of dimension NBmodes x 1


    // load/create aol_modeval_ol (pseudo-open loop mode values)
    if(sprintf(imname, "aol%ld_modeval_ol", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDout = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 20);

    // setup RTstreamLOG modeval_ol
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval_ol, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval_ol] = 1;




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
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval_dm_corr, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval_dm_corr] = 1;


    // load/create aol_modeval_dm_now (current modal DM correction after mixing with predicitiv control)
    if(sprintf(imname, "aol%ld_modeval_dm_now", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnow = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval_dm_now, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval_dm_now] = 1;


    // load/create aol_modeval_dm_now_filt (current modal DM correction, filtered)
    if(sprintf(imname, "aol%ld_modeval_dm_now_filt", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDMnowfilt = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval_dm_now_filt, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval_dm_now_filt] = 1;


    // load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modeval_dm", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalDM = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modeval_dm, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval_dm] = 1;


    // load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFsync", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalPFsync = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modevalPFsync, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modevalPFsync] = 1;


    // load/create aol_modeval_dm (modal DM correction at time of currently available WFS measurement)
    if(sprintf(imname, "aol%ld_modevalPFres", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
    IDmodevalPFres = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(imname, 10);
    AOloopControl_RTstreamLOG_setup(loop, RTSLOGindex_modevalPFres, imname);
    aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modevalPFres] = 1;


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
    for(m=0; m<NBmodes; m++)
        data.image[IDmodeARPFgain].array.F[m] = 0.0;

    if(aoloopcontrol_var.aoconfID_modeARPFgainAuto == -1)
    {
        // multiplicative auto ratio on top of gain above
        if(sprintf(imname, "aol%ld_mode_ARPFgainAuto", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
        aoloopcontrol_var.aoconfID_modeARPFgainAuto = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
        COREMOD_MEMORY_image_set_createsem(imname, 10);
        // initialize the gain to zero for all modes
        for(m=0; m<NBmodes; m++)
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
    sizeout[0] = AOconf[loop].AOpmodecoeffs.DMmodesNBblock;
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
    data.image[IDblknb].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
    data.image[IDblknb].md[0].write = 0;






    framelatency = AOconf[loop].AOtiminginfo.hardwlatency_frame + AOconf[loop].AOtiminginfo.wfsmextrlatency_frame;

    if(data.processinfo==1)
    {
        char msgstring[200];
        sprintf(msgstring, "latency = %.3f", framelatency);
        strcpy(processinfo->statusmsg, msgstring);
    }

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
    for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
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


    if(data.processinfo==1)
    {
        processinfo->loopstat = 1; // loop running
        processinfo_WriteMessage(processinfo, "Running loop");
    }
    int loopOK = 1;
    int loopCTRLexit = 0; // toggles to 1 when loop is set to exit cleanly
    long loopcnt = 0;

    loopPFcnt = 0;
    while(loopOK==1)
    {
        long modevalDMindex0, modevalDMindex1;
        long modevalPFindex0, modevalPFindex1;


        // processinfo control
        if(data.processinfo==1)
        {
            while(processinfo->CTRLval == 1)  // pause
                usleep(50);

            if(processinfo->CTRLval == 2) // single iteration
                processinfo->CTRLval = 1;

            if(processinfo->CTRLval == 3) // exit loop
                loopOK = 0; //loopCTRLexit = 1;
        }


        if(loopOK == 1)
        {

            // read WFS measured modes (residual)
            if(data.image[IDmodeval].md[0].sem==0)
            {
                while(cnt==data.image[IDmodeval].md[0].cnt0) // test if new frame exists
                    usleep(5);
                cnt = data.image[IDmodeval].md[0].cnt0;
            }
            else
				ImageStreamIO_semwait(data.image[IDmodeval], modeval_semwaitindex);
//                sem_wait(data.image[IDmodeval].semptr[modeval_semwait]);


            // drive sem4 to zero
            //TEST
            /*        while(sem_trywait(data.image[IDmodeval].semptr[4])==0) {
            			printf("WARNING %s %d  : sem_trywait on modeval\n", __FILE__, __LINE__);
            			fflush(stdout);
            			}*/
            int semval, semcnt;

            sem_getvalue(data.image[IDmodeval].semptr[4], &semval);

            //TEST

            for(semcnt=0; semcnt<semval; semcnt++)
            {
                printf("WARNING %s %d  : [%d] sem_trywait on data.image[IDmodeval]\n", __FILE__, __LINE__, semval-semcnt);
                fflush(stdout);
                sem_trywait(data.image[IDmodeval].semptr[4]);
            }




            if((data.processinfo==1)&&(processinfo->MeasureTiming==1))
                processinfo_exec_start(processinfo);




            AOconf[loop].AOtiminginfo.statusM = 3;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[3] = tdiffv;

            LOOPiter = data.image[IDmodeval].md[0].cnt1;

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval, tnow);

            // write gain, mult, limit into arrays
            for(m=0; m<NBmodes; m++)
            {
                modegain[m] = AOconf[loop].aorun.gain * data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m];
                modemult[m] = AOconf[loop].aorun.mult * data.image[aoloopcontrol_var.aoconfID_multfb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[m];
                modelimit[m] = AOconf[loop].aorun.maxlimit * data.image[aoloopcontrol_var.aoconfID_limitb].array.F[modeblock[m]] * data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m];
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

            AOconf[loop].AOtiminginfo.statusM = 4;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[4] = tdiffv;

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval_dm_corr, tnow);


            int ARPF_ok;
            ARPF_ok = 0;

            //
            //  MIX PREDICTION WITH CURRENT DM STATE
            //
            if(AOconf[loop].aorun.ARPFon==1)
            {
                //	printf("%s  %s  %d\n",__FILE__, __func__, __LINE__);fflush(stdout); //TEST

                if(aoloopcontrol_var.aoconfID_modevalPF == -1)
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

                    AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modevalPF, tnow);




                    for(m=0; m<NBmodes; m++)
                    {
                        float mixratio;

                        mixratio = AOconf[loop].aorun.ARPFgain * data.image[IDmodeARPFgain].array.F[m] * data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m];
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
                        float minVal = AOconf[loop].aorun.ARPFgainAutoMin;
                        float maxVal = AOconf[loop].aorun.ARPFgainAutoMax;

                        if(data.image[IDmodeARPFgain].array.F[m]>0.5) // if mode is predictive-controlled
                        {
                            if (data.image[IDmodevalPFres].array.F[m]*data.image[IDmodevalPFres].array.F[m] < data.image[IDmodeval].array.F[m]*data.image[IDmodeval].array.F[m])
                                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] += 0.001;
                            else
                                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] -= 0.001;

                            if (data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] > maxVal)
                                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = maxVal;
                            if (data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] < minVal)
                                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = minVal;
                        }
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

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval_dm_now, tnow);



            AOconf[loop].AOtiminginfo.statusM = 5;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[5] = tdiffv;

            data.image[IDmodevalDMnowfilt].md[0].write = 1;
            // FILTERING MODE VALUES
            // THIS FILTERING GOES TOGETHER WITH THE SECONDARY WRITE ON DM TO KEEP FILTERED AND ACTUAL VALUES IDENTICAL
            for(m=0; m<NBmodes; m++)
                data.image[IDmodevalDMnowfilt].array.F[m] = data.image[IDmodevalDMnow].array.F[m];


            if(AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_ON==1) // automatically adjust modal limits
            {
                data.image[IDatlimbcoeff].md[0].write = 1;
                for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
                    limitblockarray[block] = 0.0;

                data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].write = 1;
                data.image[aoloopcontrol_var.aoconfID_limitb].md[0].write = 1;

                // Adjust limit for EACH mode
                for(m=0; m<NBmodes; m++)
                {
                    block = data.image[IDblknb].array.UI16[m];

                    if(  fabs(AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_mcoeff*data.image[IDmodevalDMnowfilt].array.F[m]) > modelimit[m])
                        data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m] *= (1.0 + AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta);
                    else
                        data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m] *= (1.0 - AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta*0.01*AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_perc);

                    limitblockarray[block] += data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[m];
                }
                COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_LIMIT_modes, -1);
                data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].cnt1 = LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].md[0].write = 0;

                // update block limits to drive average limit coefficients to 1
                data.image[IDatlimbcoeff].md[0].write = 1;
                for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
                {
                    data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                    coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta*0.1 );
                    if(coeff < 1.0-AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta )
                        coeff = 1.0-AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta;
                    if(coeff> 1.0+AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta )
                        coeff = 1.0+AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta;
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


            if(AOconf[loop].AOAutoTune.AUTOTUNE_GAINS_ON==1)
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
                        for(m=0; m<NBmodes; m++)
                        {
                            if(data.image[IDautogain].array.F[m] > maxGainVal)
                                maxGainVal = data.image[IDautogain].array.F[m];
                        }
                        globalgain = maxGainVal;
                        printf("     Setting  global gain = %f\n", maxGainVal);
                        AOconf[loop].aorun.gain = maxGainVal;

                        sprintf(command, "echo \"%6.4f\" > conf/param_loopgain.txt", AOconf[loop].aorun.gain);
                        ret = system(command);



                        // Set block gain to max gain within block, scaled to global gain
                        for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
                        {
                            maxGainVal = 0.0;
                            for(m=0; m<NBmodes; m++)
                                if(data.image[IDblknb].array.UI16[m] == block)
                                    if(data.image[IDautogain].array.F[m] > maxGainVal)
                                        maxGainVal = data.image[IDautogain].array.F[m];

                            printf("Set block %2ld gain to  %f\n", block, maxGainVal/globalgain);

                            data.image[aoloopcontrol_var.aoconfID_gainb].array.F[block] = maxGainVal/AOconf[loop].aorun.gain;


                            sprintf(command, "echo \"%6.4f\" > conf/param_gainb%02ld.txt", data.image[aoloopcontrol_var.aoconfID_gainb].array.F[block], block);
                            ret = system(command);
                        }

                        // Set individual gain
                        for(m=0; m<NBmodes; m++)
                        {
                            data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m] = data.image[IDautogain].array.F[m]/data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]]/AOconf[loop].aorun.gain;

                            if(m<20)
                                printf("Mode %3ld   %12f  %12f  %12f ->   %12f  %12f\n", m, AOconf[loop].aorun.gain, data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]], data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m], AOconf[loop].aorun.gain*data.image[aoloopcontrol_var.aoconfID_gainb].array.F[modeblock[m]]*data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[m], data.image[IDautogain].array.F[m]);
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
                            for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
                            {
                                data.image[IDatlimbcoeff].array.F[block] = limitblockarray[block] / blockNBmodes[block];
                                coeff = ( 1.0 + (data.image[IDatlimbcoeff].array.F[block]-1.0)*AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta*0.1 );
                                if(coeff < 1.0-AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta )
                                    coeff = 1.0-AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta;
                                if(coeff> 1.0+AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta )
                                    coeff = 1.0+AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta;
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

            if(AOconf[loop].aorun.DMfilteredWriteON==1)
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

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval_dm_now_filt, tnow);






            // IF MODAL DM, AND FILTERED DM WRITE IS ON, SEND TO DM
            if((AOconf[loop].aorun.DMfilteredWriteON==1) && (AOconf[loop].DMctrl.DMMODE==1))
            {
                data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 1;
                memcpy(data.image[aoloopcontrol_var.aoconfID_dmC].array.F, data.image[IDmodevalDMnowfilt].array.F, sizeof(float)*NBmodes);
                COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_dmC, -1);
                data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 0;

                AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_dmC, tnow);
            }


            AOconf[loop].AOtiminginfo.statusM = 6;

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[6] = tdiffv;

            AOconf[loop].AOtiminginfo.statusM1 = 0;


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



            AOconf[loop].AOtiminginfo.statusM1 = 1;

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

            AOconf[loop].AOtiminginfo.statusM1 = 2;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[10] = tdiffv;

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval_dm, tnow);



            if(AOconf[loop].aorun.ARPFon==1)
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

                AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modevalPFsync, tnow);
            }



            AOconf[loop].AOtiminginfo.statusM1 = 3;
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

            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modeval_ol, tnow);


            if(AOconf[loop].aorun.ARPFon==1)
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

                AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_modevalPFres, tnow);
            }



            AOconf[loop].AOtiminginfo.statusM1 = 4;
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
            if(blockstatcnt == AOconf[loop].AOpmodecoeffs.AveStats_NBpt)
            {
                for(block=0; block<AOconf[loop].AOpmodecoeffs.DMmodesNBblock; block++)
                {
                    AOconf[loop].AOpmodecoeffs.blockave_PFresrms[block] = sqrt(blockavePFresrms[block]/blockstatcnt);
                    AOconf[loop].AOpmodecoeffs.blockave_OLrms[block] = sqrt(blockaveOLrms[block]/blockstatcnt);
                    AOconf[loop].AOpmodecoeffs.blockave_Crms[block] = sqrt(blockaveCrms[block]/blockstatcnt);
                    AOconf[loop].AOpmodecoeffs.blockave_WFSrms[block] = sqrt(blockaveWFSrms[block]/blockstatcnt);
                    AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[block] = sqrt(blockaveWFSnoise[block]/blockstatcnt);
                    AOconf[loop].AOpmodecoeffs.blockave_limFrac[block] = (blockavelimFrac[block])/blockstatcnt;

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

                AOconf[loop].AOpmodecoeffs.ALLave_OLrms = sqrt(allaveOLrms/blockstatcnt);
                AOconf[loop].AOpmodecoeffs.ALLave_Crms = sqrt(allaveCrms/blockstatcnt);
                AOconf[loop].AOpmodecoeffs.ALLave_WFSrms = sqrt(allaveWFSrms/blockstatcnt);
                AOconf[loop].AOpmodecoeffs.ALLave_WFSnoise = sqrt(allaveWFSnoise/blockstatcnt);
                AOconf[loop].AOpmodecoeffs.ALLave_limFrac = allavelimFrac/blockstatcnt;

                allavePFresrms = 0.0;
                allaveOLrms = 0.0;
                allaveCrms = 0.0;
                allaveWFSrms = 0.0;
                allavelimFrac = 0.0;

                blockstatcnt = 0;
            }




            AOconf[loop].AOtiminginfo.statusM1 = 5;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[13] = tdiffv;


            if((data.processinfo==1)&&(processinfo->MeasureTiming==1))
                processinfo_exec_end(processinfo);

        }
        // process signals

        if(data.signal_TERM == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGTERM);
        }

        if(data.signal_INT == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGINT);
        }

        if(data.signal_ABRT == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGABRT);
        }

        if(data.signal_BUS == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGBUS);
        }

        if(data.signal_SEGV == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGSEGV);
        }

        if(data.signal_HUP == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGHUP);
        }

        if(data.signal_PIPE == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGPIPE);
        }

        loopcnt++;
        if(data.processinfo==1)
            processinfo->loopcnt = loopcnt;

    }

    free(modegain);
    free(modemult);
    free(modelimit);
	
    
    free(modeblock);

    // LOG function start
    CORE_logFunctionCall( logfunc_level, logfunc_level_max, 1, __FILE__, __func__, __LINE__, commentstring);

    if((data.processinfo==1)&&(processinfo->loopstat != 4))
        processinfo_cleanExit(processinfo);

    
    return(IDout);
}


