/**
 * @file    AOloopControl_loop_param.c 
 * @brief   AO loop control - CONTROL LOOP PARAMETERS 
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

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define AOconfname "/tmp/AOconf.shm"
AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
AOloopControl_var aoloopcontrol_var;

extern DATA data;




int_fast8_t AOloopControl_setgain(float gain)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].gain = gain;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setARPFgain(float gain)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].ARPFgain = gain;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setARPFgainAutoMin(float val)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].ARPFgainAutoMin = val;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}

int_fast8_t AOloopControl_setARPFgainAutoMax(float val)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].ARPFgainAutoMax = val;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}




int_fast8_t AOloopControl_setWFSnormfloor(float WFSnormfloor)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].WFSnormfloor = WFSnormfloor;
    printf("SHOWING PARAMETERS ...\n");
    fflush(stdout);
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);
    printf("DONE ...\n");
    fflush(stdout);

    return 0;
}


int_fast8_t AOloopControl_setmaxlimit(float maxlimit)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].maxlimit = maxlimit;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setmult(float multcoeff)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].mult = multcoeff;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_setframesAve(long nbframes)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].framesAve = nbframes;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}






int_fast8_t AOloopControl_set_modeblock_gain(long loop, long blocknb, float gain, int add)
{
    long IDcontrM0; // local storage
    char name2[200];
    char name3[200];
    long ID;
    long m1;


    printf("AOconf[loop].DMmodesNBblock = %ld\n", AOconf[loop].DMmodesNBblock);
    fflush(stdout);

    /*if(AOconf[loop].CMMODE==0)
    {
        printf("Command has no effect: modeblock gain not compatible with CMMODE = 0\n");
        fflush(stdout);
    }
    else*/
     
    if (AOconf[loop].DMmodesNBblock<2)
    {
        if(sprintf(name2, "aol%ld_contrMc00", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if(sprintf(name3, "aol%ld_contrMcact00_00", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        // for CPU mode
        printf("UPDATING Mc matrix (CPU mode)\n");
        ID = image_ID(name2);
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 1;
        memcpy(data.image[aoloopcontrol_var.aoconfID_contrMc].array.F, data.image[ID].array.F, sizeof(float)*AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM);
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 0;

        // for GPU mode
        printf("UPDATING Mcact matrix (GPU mode)\n");
        ID = image_ID(name3);
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 1;
        memcpy(data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F, data.image[ID].array.F, sizeof(float)*AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt);
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt1 = AOconf[loop].LOOPiteration;
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 0;
    }
    else
    {
		long kk;
	    char name[200];
	    long NBmodes = 0;
        
        for(kk=0; kk<AOconf[loop].DMmodesNBblock; kk++)
            NBmodes += AOconf[loop].NBmodes_block[kk];



        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_gainb = image_ID(name);
        if((blocknb<AOconf[loop].DMmodesNBblock)&&(blocknb>-1))
            data.image[aoloopcontrol_var.aoconfID_gainb].array.F[blocknb] = gain;


        if(add==1)
        {
			long IDcontrMc0; // local storage
			long IDcontrMcact0; // local storage			
			
			
            IDcontrMc0 = image_ID("contrMc0");
            if(IDcontrMc0==-1)
                IDcontrMc0 = create_3Dimage_ID("contrMc0", AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS, AOconf[loop].sizexDM*AOconf[loop].sizeyDM);


            IDcontrMcact0 = image_ID("contrMcact0");
            if(IDcontrMcact0==-1)
                IDcontrMcact0 = create_2Dimage_ID("contrMcact0", AOconf[loop].activeWFScnt, AOconf[loop].activeDMcnt);

            //arith_image_zero("contrM0");
            arith_image_zero("contrMc0");
            arith_image_zero("contrMcact0");


            for(kk=0; kk<AOconf[loop].DMmodesNBblock; kk++)
            {
			    double eps=1e-6;
				
				
                if(sprintf(name2, "aol%ld_contrMc%02ld", loop, kk) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(name3, "aol%ld_contrMcact%02ld_00", loop, kk) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                printf("Adding %4ld / %4ld  (%5.3f)   %s   [%ld]\n", kk, AOconf[loop].DMmodesNBblock, data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk], name, aoloopcontrol_var.aoconfID_gainb);

                

                //printf("updating %ld modes  [%ld]\n", data.image[ID].md[0].size[2], aoloopcontrol_var.aoconfID_gainb);
                //	fflush(stdout); // TEST



                if(data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]>eps)
                {
					long ii;
					
                    ID = image_ID(name2);
# ifdef _OPENMP
                    #pragma omp parallel for
# endif
                    for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM; ii++)
                        data.image[IDcontrMc0].array.F[ii] += data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];

                    ID = image_ID(name3);
# ifdef _OPENMP
                    #pragma omp parallel for
# endif
                    for(ii=0; ii<AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt; ii++)
                        data.image[IDcontrMcact0].array.F[ii] += data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk]*data.image[ID].array.F[ii];
                }

            }

            // for CPU mode
            printf("UPDATING Mc matrix (CPU mode)\n");
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 1;
            memcpy(data.image[aoloopcontrol_var.aoconfID_contrMc].array.F, data.image[IDcontrMc0].array.F, sizeof(float)*AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].sizexDM*AOconf[loop].sizeyDM);
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt0++;
			data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt1 = AOconf[loop].LOOPiteration;
			data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 0;


            // for GPU mode
            printf("UPDATING Mcact matrix (GPU mode)\n");
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 1;
            memcpy(data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F, data.image[IDcontrMcact0].array.F, sizeof(float)*AOconf[loop].activeWFScnt*AOconf[loop].activeDMcnt);
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt1 = AOconf[loop].LOOPiteration;
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 0;

            aoloopcontrol_var.initcontrMcact_GPU[0] = 0;
        }
    }

    return(0);
}







int_fast8_t AOloopControl_scanGainBlock(long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain)
{
    long k, kg;
    float bestgain= 0.0;
    float bestval = 10000000.0;



    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_cmd_modes==-1)
    {
		char name[200];
		    
        if(sprintf(name, "aol%ld_DMmode_cmd", aoloopcontrol_var.LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }


    printf("Block: %ld, NBstep: %ld, gain: %f->%f (%ld septs)\n", NBblock, NBstep, gainStart, gainEnd, NBgain);

    for(kg=0; kg<NBgain; kg++)
    {
	    float gain;
		float val;
		
        for(k=0; k<AOconf[aoloopcontrol_var.LOOPNUMBER].NBDMmodes; k++)
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = 0.0;

        gain = gainStart + 1.0*kg/(NBgain-1)*(gainEnd-gainStart);
        AOloopControl_setgainblock(NBblock, gain);
        AOloopControl_loopstep(aoloopcontrol_var.LOOPNUMBER, NBstep);
        val = sqrt(AOconf[aoloopcontrol_var.LOOPNUMBER].RMSmodesCumul/AOconf[aoloopcontrol_var.LOOPNUMBER].RMSmodesCumulcnt);
        printf("%2ld  %6.4f  %10.8lf\n", kg, gain, val);

        if(val<bestval)
        {
            bestval = val;
            bestgain = gain;
        }
    }
    printf("BEST GAIN = %f\n", bestgain);

    AOloopControl_setgainblock(NBblock, bestgain);

    return(0);
}

