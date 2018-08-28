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



    gain0 = 1.0/(AOconf[loop].AOtiminginfo.loopfrequ*AOconf[loop].AUTOTUNEGAINS_evolTimescale);




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
        latency = AOconf[loop].AOtiminginfo.hardwlatency_frame + AOconf[loop].AOtiminginfo.wfsmextrlatency_frame;
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
        data.image[IDout].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
        data.image[IDout].md[0].write = 0;


		// write noise
		data.image[IDmodeWFSnoise].md[0].write = 1;
		data.image[IDmodeWFSnoise].md[0].cnt0++;
		for(m=0;m<NBmodes;m++)
			data.image[IDmodeWFSnoise].array.F[m] = array_sig[m];
		COREMOD_MEMORY_image_set_sempost_byID(IDmodeWFSnoise, -1);
		data.image[IDmodeWFSnoise].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
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

