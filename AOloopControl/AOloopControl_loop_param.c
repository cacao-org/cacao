/**
 * @file    AOloopControl_loop_param.c
 * @brief   AO loop control - CONTROL LOOP PARAMETERS
 *
 * REAL TIME COMPUTING ROUTINES
 *
 *
 */

#define _GNU_SOURCE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_memory/COREMOD_memory.h"

// defined in AOloopControl.c
//extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
//extern AOloopControl_var aoloopcontrol_var;


/*
errno_t AOloopControl_setgain(float gain)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.gain = gain;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setARPFgain(float gain)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.ARPFgain = gain;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setARPFgainAutoMin(float val)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.ARPFgainAutoMin = val;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setARPFgainAutoMax(float val)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.ARPFgainAutoMax = val;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setWFSnormfloor(float WFSnormfloor)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].WFSim.WFSnormfloor = WFSnormfloor;
    printf("SHOWING PARAMETERS ...\n");
    fflush(stdout);
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);
    printf("DONE ...\n");
    fflush(stdout);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setmaxlimit(float maxlimit)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.maxlimit = maxlimit;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_setmult(float multcoeff)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.mult = multcoeff;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}
*/


/*
errno_t
AOloopControl_set_modeblock_gain(long loop, long blocknb, float gain, int add)
{
    char    name2[200];
    char    name3[200];
    imageID ID;

    printf("AOconf[loop].AOpmodecoeffs.DMmodesNBblock = %u\n",
           AOconf[loop].AOpmodecoeffs.DMmodesNBblock);
    fflush(stdout);


    if(AOconf[loop].AOpmodecoeffs.DMmodesNBblock < 2)
    {
        if(sprintf(name2, "aol%ld_contrMc00", loop) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        if(sprintf(name3, "aol%ld_contrMcact00_00", loop) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        // for CPU mode
        printf("UPDATING Mc matrix (CPU mode)\n");
        ID = image_ID(name2);
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 1;
        memcpy(data.image[aoloopcontrol_var.aoconfID_contrMc].array.F,
               data.image[ID].array.F,
               sizeof(float) * AOconf[loop].WFSim.sizexWFS *
               AOconf[loop].WFSim.sizeyWFS * AOconf[loop].DMctrl.sizexDM *
               AOconf[loop].DMctrl.sizeyDM);
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt1 =
            AOconf[loop].aorun.LOOPiteration;
        data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 0;

        // for GPU mode
        printf("UPDATING Mcact matrix (GPU mode)\n");
        ID = image_ID(name3);
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 1;
        memcpy(data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F,
               data.image[ID].array.F,
               sizeof(float) * AOconf[loop].WFSim.activeWFScnt *
               AOconf[loop].DMctrl.activeDMcnt);
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt1 =
            AOconf[loop].aorun.LOOPiteration;
        data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write = 0;
    }
    else
    {
        uint32_t kk;
        char     name[200];
        long     NBmodes = 0;

        for(kk = 0; kk < AOconf[loop].AOpmodecoeffs.DMmodesNBblock; kk++)
        {
            NBmodes += AOconf[loop].AOpmodecoeffs.NBmodes_block[kk];
        }

        if(sprintf(name, "aol%ld_gainb", loop) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_gainb = image_ID(name);
        if((blocknb < (long) AOconf[loop].AOpmodecoeffs.DMmodesNBblock) &&
                (blocknb > -1))
        {
            data.image[aoloopcontrol_var.aoconfID_gainb].array.F[blocknb] =
                gain;
        }

        if(add == 1)
        {
            long IDcontrMc0;    // local storage
            long IDcontrMcact0; // local storage

            IDcontrMc0 = image_ID("contrMc0");
            if(IDcontrMc0 == -1)
            {
                create_3Dimage_ID("contrMc0",
                                  AOconf[loop].WFSim.sizexWFS,
                                  AOconf[loop].WFSim.sizeyWFS,
                                  AOconf[loop].DMctrl.sizexDM *
                                  AOconf[loop].DMctrl.sizeyDM,
                                  &IDcontrMc0);
            }

            IDcontrMcact0 = image_ID("contrMcact0");
            if(IDcontrMcact0 == -1)
            {
                create_2Dimage_ID("contrMcact0",
                                  AOconf[loop].WFSim.activeWFScnt,
                                  AOconf[loop].DMctrl.activeDMcnt,
                                  &IDcontrMcact0);
            }

            // arith_image_zero("contrM0");
            arith_image_zero("contrMc0");
            arith_image_zero("contrMcact0");

            for(kk = 0; kk < AOconf[loop].AOpmodecoeffs.DMmodesNBblock; kk++)
            {
                double eps = 1e-6;

                if(sprintf(name2, "aol%ld_contrMc%02u", loop, kk) < 1)
                {
                    PRINT_ERROR("sprintf wrote <1 char");
                }

                if(sprintf(name3, "aol%ld_contrMcact%02u_00", loop, kk) < 1)
                {
                    PRINT_ERROR("sprintf wrote <1 char");
                }

                printf("Adding %4u / %4u  (%5.3f)   %s   [%ld]\n",
                       kk,
                       AOconf[loop].AOpmodecoeffs.DMmodesNBblock,
                       data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk],
                       name,
                       aoloopcontrol_var.aoconfID_gainb);

                // printf("updating %ld modes  [%ld]\n",
                // data.image[ID].md[0].size[2],
                // aoloopcontrol_var.aoconfID_gainb); 	fflush(stdout); // TEST

                if(data.image[aoloopcontrol_var.aoconfID_gainb].array.F[kk] >
                        eps)
                {
                    uint64_t ii;

                    ID = image_ID(name2);
#ifdef _OPENMP
                    #pragma omp parallel for
#endif
                    for(ii = 0; ii < AOconf[loop].WFSim.sizexWFS *
                            AOconf[loop].WFSim.sizeyWFS *
                            AOconf[loop].DMctrl.sizexDM *
                            AOconf[loop].DMctrl.sizeyDM;
                            ii++)
                    {
                        data.image[IDcontrMc0].array.F[ii] +=
                            data.image[aoloopcontrol_var.aoconfID_gainb]
                            .array.F[kk] *
                            data.image[ID].array.F[ii];
                    }

                    ID = image_ID(name3);
#ifdef _OPENMP
                    #pragma omp parallel for
#endif
                    for(ii = 0; ii < AOconf[loop].WFSim.activeWFScnt *
                            AOconf[loop].DMctrl.activeDMcnt;
                            ii++)
                    {
                        data.image[IDcontrMcact0].array.F[ii] +=
                            data.image[aoloopcontrol_var.aoconfID_gainb]
                            .array.F[kk] *
                            data.image[ID].array.F[ii];
                    }
                }
            }

            // for CPU mode
            printf("UPDATING Mc matrix (CPU mode)\n");
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 1;
            memcpy(data.image[aoloopcontrol_var.aoconfID_contrMc].array.F,
                   data.image[IDcontrMc0].array.F,
                   sizeof(float) * AOconf[loop].WFSim.sizexWFS *
                   AOconf[loop].WFSim.sizeyWFS *
                   AOconf[loop].DMctrl.sizexDM *
                   AOconf[loop].DMctrl.sizeyDM);
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].cnt1 =
                AOconf[loop].aorun.LOOPiteration;
            data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].write = 0;

            // for GPU mode
            printf("UPDATING Mcact matrix (GPU mode)\n");
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write =
                1;
            memcpy(data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].array.F,
                   data.image[IDcontrMcact0].array.F,
                   sizeof(float) * AOconf[loop].WFSim.activeWFScnt *
                   AOconf[loop].DMctrl.activeDMcnt);
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].cnt1 =
                AOconf[loop].aorun.LOOPiteration;
            data.image[aoloopcontrol_var.aoconfID_contrMcact[0]].md[0].write =
                0;

            aoloopcontrol_var.initcontrMcact_GPU[0] = 0;
        }
    }

    return RETURN_SUCCESS;
}
*/




/*
errno_t AOloopControl_scanGainBlock(
    long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain)
{
    long  kg;
    float bestgain = 0.0;
    float bestval  = 10000000.0;

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    if(aoloopcontrol_var.aoconfID_cmd_modes == -1)
    {
        char name[200];

        if(sprintf(name, "aol%ld_DMmode_cmd", aoloopcontrol_var.LOOPNUMBER) <
                1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    printf("Block: %ld, NBstep: %ld, gain: %f->%f (%ld septs)\n",
           NBblock,
           NBstep,
           gainStart,
           gainEnd,
           NBgain);

    for(kg = 0; kg < NBgain; kg++)
    {
        float gain;
        float val;

        for(uint32_t k = 0;
                k < AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.NBDMmodes;
                k++)
        {
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = 0.0;
        }

        gain = gainStart + 1.0 * kg / (NBgain - 1) * (gainEnd - gainStart);
        AOloopControl_setgainblock(NBblock, gain);
        AOloopControl_loopstep(aoloopcontrol_var.LOOPNUMBER, NBstep);
        val = sqrt(
                  AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.RMSmodesCumul /
                  AOconf[aoloopcontrol_var.LOOPNUMBER]
                  .AOpmodecoeffs.RMSmodesCumulcnt);
        printf("%2ld  %6.4f  %10.8lf\n", kg, gain, val);

        if(val < bestval)
        {
            bestval  = val;
            bestgain = gain;
        }
    }
    printf("BEST GAIN = %f\n", bestgain);

    AOloopControl_setgainblock(NBblock, bestgain);

    return RETURN_SUCCESS;
}
*/