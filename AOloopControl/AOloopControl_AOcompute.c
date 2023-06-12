/**
 * @file    AOloopControl_AOcompute.c
 * @brief   AO loop Control compute functions
 *
 * Low level compute
 *
 *
 */

// uncomment for test print statements to stdout
//#define _PRINT_TEST

#define _GNU_SOURCE

#include <time.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "CommandLineInterface/timeutils.h"
#include "ImageStreamIO/ImageStreamIO.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "info/info.h"

#include <ncurses.h>

#include "linalgebra/linalgebra.h"


#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

// TIMING
static struct timespec tnow;
// static struct timespec tdiff;

static double tdiffv;
static double tdiffv00;
static double tdiffv01;

#ifdef HAVE_CUDA
static long long wfsrefcnt0          = -1;
static int       initWFSref_GPU[100] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
static long contrMcactcnt0[100] = {[0 ... 99] = -1};
#endif

static long long aoconfcnt0_contrM_current = -1;
static long      aoconfID_imWFS2_active[100];

//{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//-1, -1, -1, -1, -1};; int array[1024] = { [ 0 ... 99 ] = -1 };

extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var   aoloopcontrol_var;

// TEST
//  normal timer values
double ntimerval[50];
// last timer values
double ltimerval[50];

static int wcol, wrow; // window size

// TO BE MOVED INTO AOcompute structure
static int AOcompute_WFSlinlimit = 0;

static int AOloopControl_AOcompute_ProcessInit_Value =
    0; // toggles to 1 when AOcompute

// ********************************************************************
// This initialization runs once per process
// ********************************************************************

int AOloopControl_AOcompute_ProcessInit(long loop)
{

    aoconfcnt0_contrM_current =
        data.image[aoloopcontrol_var.aoconfID_contrM].md[0].cnt0;

    printf("GPU0 = %d\n", AOconf[loop].AOcompute.GPU0);
    if(AOconf[loop].AOcompute.GPU0 > 0)
    {
        uint8_t k;
        for(k = 0; k < AOconf[loop].AOcompute.GPU0; k++)
        {
            printf("stream %2d      aoloopcontrol_var.GPUset0 = %2d\n",
                   (int) k,
                   aoloopcontrol_var.GPUset0[k]);
        }
    }

    printf("GPU1 = %d\n", AOconf[loop].AOcompute.GPU1);
    if(AOconf[loop].AOcompute.GPU1 > 0)
    {
        uint8_t k;
        for(k = 0; k < AOconf[loop].AOcompute.GPU1; k++)
        {
            printf("stream %2d      aoloopcontrol_var.GPUset1 = %2d\n",
                   (int) k,
                   aoloopcontrol_var.GPUset1[k]);
        }
    }

    return 0;
}

int printstatus_AOloopControl_AOcompute(int loop)
{
    printw("ComputeWFSsol_FLAG      %d\n",
           AOconf[loop].AOcompute.ComputeWFSsol_FLAG);
    printw("ComputeFLAG0            %d\n", AOconf[loop].AOcompute.ComputeFLAG0);
    printw("ComputeFLAG1            %d\n", AOconf[loop].AOcompute.ComputeFLAG1);
    printw("ComputeFLAG2            %d\n", AOconf[loop].AOcompute.ComputeFLAG2);
    printw("ComputeFLAG3            %d\n", AOconf[loop].AOcompute.ComputeFLAG3);

    printw("GPU0                    %d\n", AOconf[loop].AOcompute.GPU0);
    printw("GPU1                    %d\n", AOconf[loop].AOcompute.GPU1);
    printw("GPU2                    %d\n", AOconf[loop].AOcompute.GPU2);
    printw("GPU3                    %d\n", AOconf[loop].AOcompute.GPU3);

    printw("GPUall                  %d\n", AOconf[loop].AOcompute.GPUall);
    printw("GPUusesem               %d\n", AOconf[loop].AOcompute.GPUusesem);
    printw("AOLCOMPUTE_TOTAL_ASYNC  %d\n",
           AOconf[loop].AOcompute.AOLCOMPUTE_TOTAL_ASYNC);

    return 0;
}

int AOloopControl_AOcompute_GUI(long loop, double frequ)
{
    char monstring[200];
    int  loopOK = 1;
    int  freeze = 0;
    long cnt    = 0;

    // Connect to shared memory
    AOloopControl_InitializeMemory(1);

    /*  Initialize ncurses  */
    if(initscr() == NULL)
    {
        fprintf(stderr, "Error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }

    getmaxyx(stdscr, wrow, wcol); /* get the number of rows and columns */
    cbreak();
    keypad(stdscr, TRUE); /* We get F1, F2 etc..		*/
    nodelay(stdscr, TRUE);
    curs_set(0);
    noecho(); /* Don't echo() while we do getch */

    start_color();
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_BLACK, COLOR_RED);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_YELLOW, COLOR_BLACK);
    init_pair(5, COLOR_RED, COLOR_BLACK);
    init_pair(6, COLOR_BLACK, COLOR_RED);

    while(loopOK == 1)
    {
        usleep((long)(1000000.0 / frequ));
        int ch = getch();

        if(freeze == 0)
        {
            attron(A_BOLD);
            sprintf(monstring, "PRESS x TO STOP MONITOR");
            print_header(monstring, '-');
            attroff(A_BOLD);
        }

        switch(ch)
        {
            case 'f':
                if(freeze == 0)
                {
                    freeze = 1;
                }
                else
                {
                    freeze = 0;
                }
                break;

            case 'x':
                loopOK = 0;
                break;
        }

        if(freeze == 0)
        {
            clear();
            printstatus_AOloopControl_AOcompute(loop);

            refresh();

            cnt++;
        }
    }
    endwin();

    return (0);
}

// TODO : zero DMmode_cmd upon loop startup

/**
 * ## Purpose
 *
 * Main computation routine.\n
 * AOcompute() is called inside the aorun loop.\n
 *
 * AOcompute main steps are:
 * - Read WFS image (call to Read_cam_frame())
 * - Process WFS frame
 * - Multiply by control matrix
 *
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
errno_t __attribute__((hot)) AOcompute(long loop, int normalize)
{
    struct timespec t1;
    struct timespec t2;

    int slice;

    uint64_t LOOPiter;

    struct timespec functionTestTimerStart;
    struct timespec functionTestTimerEnd;

    struct timespec functionTestTimer00;
    struct timespec functionTestTimer01;
    struct timespec functionTestTimer02;
    struct timespec functionTestTimer03;
    struct timespec functionTestTimer04;

    // import processinfo from upstream
    // PROCESSINFO *processinfo = data.pinfo;

#ifdef _PRINT_TEST
    printf("[%s] [%d]  AOcompute start, loop %ld\n",
           __FILE__,
           __LINE__,
           AOconf[loop].aorun.LOOPiteration);
    fflush(stdout);
#endif

    if(AOloopControl_AOcompute_ProcessInit_Value == 0)
    {
        AOloopControl_AOcompute_ProcessInit(loop);
        AOloopControl_AOcompute_ProcessInit_Value = 1;
    }

    // lock loop iteration into variable so that it cannot increment in case loop
    // interations overlap
    LOOPiter = AOconf[loop].aorun.LOOPiteration;

    // waiting for dark-subtracted image
    AOconf[loop].AOtiminginfo.status = 19; //  19: WAITING FOR IMAGE
    clock_gettime(CLOCK_MILK, &tnow);
    tdiffv = timespec_diff_double(
                 data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                 tnow);
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[23] = tdiffv;

    // md[0].atime is absolute time at beginning of iteration
    //
    // pixel 0 is dt since last iteration
    //
    // pixel 1 is time from beginning of loop to status 01
    // pixel 2 is time from beginning of loop to status 02

    clock_gettime(CLOCK_MILK,
                  &functionTestTimer04); // TEST timing in function

    // Read WFS image
    // processinfo_WriteMessage(processinfo, "Wait for WFS image");
    Read_cam_frame(loop, 0, normalize, 0, 0); // in AOloopControl_IOtools
    // processinfo_WriteMessage(processinfo, "Process WFS image");

#ifdef _PRINT_TEST
    printf("[%s] [%d]  AOcompute: Input image acquired\n", __FILE__, __LINE__);
    fflush(stdout);
#endif

    clock_gettime(CLOCK_MILK,
                  &functionTestTimerStart); // TEST timing in function

    slice = aoloopcontrol_var.PIXSTREAM_SLICE;
    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING == 0)  // no pixel streaming
    {
        aoloopcontrol_var.PIXSTREAM_SLICE = 0;
    }
    //    else
    //        aoloopcontrol_var.PIXSTREAM_SLICE = 1 + slice;

    //    printf("slice = %d  ->  %d\n", slice,
    //    aoloopcontrol_var.PIXSTREAM_SLICE); fflush(stdout);

    AOconf[loop].AOtiminginfo.status = 4; // 4: REMOVING REF
    clock_gettime(CLOCK_MILK, &tnow);
    tdiffv = timespec_diff_double(
                 data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                 tnow);
    data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[15] = tdiffv;

    if(AOconf[loop].AOcompute.ComputeWFSsol_FLAG == 1)  // Process WFS frame
    {
#ifdef _PRINT_TEST
        printf("[%s] [%d]  AOcompute: Process WFS frame\n", __FILE__, __LINE__);
        fflush(stdout);
#endif

        if(AOconf[loop].AOcompute.GPUall == 0)
        {

            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 1;

            if(AOconf[loop].WFSim.WFSrefzero ==
                    0) // if WFS reference is NOT zero
            {

                for(unsigned long ii = 0; ii < AOconf[loop].WFSim.sizeWFS;
                        ii++)
                {

                    data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F[ii] =
                        data.image[aoloopcontrol_var.aoconfID_imWFS1]
                        .array.F[ii] -
                        aoloopcontrol_var.normfloorcoeff *
                        data.image[aoloopcontrol_var.aoconfID_wfsref]
                        .array.F[ii];
                }

#ifdef _PRINT_TEST
                printf(
                    "[%s] [%d]  AOcompute: aoloopcontrol_var.normfloorcoeff "
                    "= %f\n",
                    __FILE__,
                    __LINE__,
                    aoloopcontrol_var.normfloorcoeff);
                //                for(ii=0; ii<AOconf[loop].WFSim.sizeWFS; ii+=10)
                //                    printf("    %4ld  %16f  %16f\n", ii,
                //                    data.image[aoloopcontrol_var.aoconfID_imWFS1].array.F[ii],
                //                    data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F[ii]);
#endif
            }
            else
            {
                memcpy(data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F,
                       data.image[aoloopcontrol_var.aoconfID_imWFS1].array.F,
                       sizeof(float) * AOconf[loop].WFSim.sizeWFS);
            }

            if(AOcompute_WFSlinlimit == 1)
            {
                if(aoloopcontrol_var.aoconfID_imWFSlinlimit != -1)
                {
                    for(unsigned long ii = 0; ii < AOconf[loop].WFSim.sizeWFS;
                            ii++)
                    {
                        if(data.image[aoloopcontrol_var.aoconfID_imWFS2]
                                .array.F[ii] >
                                data.image[aoloopcontrol_var.aoconfID_imWFSlinlimit]
                                .array.F[ii])
                        {
                            data.image[aoloopcontrol_var.aoconfID_imWFS2]
                            .array.F[ii] =
                                data.image[aoloopcontrol_var
                                           .aoconfID_imWFSlinlimit]
                                .array.F[ii];
                        }
                        else if(data.image[aoloopcontrol_var.aoconfID_imWFS2]
                                .array.F[ii] <
                                -data.image[aoloopcontrol_var
                                            .aoconfID_imWFSlinlimit]
                                .array.F[ii])
                        {
                            data.image[aoloopcontrol_var.aoconfID_imWFS2]
                            .array.F[ii] =
                                -data.image[aoloopcontrol_var
                                            .aoconfID_imWFSlinlimit]
                                .array.F[ii];
                        }

#ifdef _PRINT_TEST
                        printf(
                            "[%s] [%d]  AOcompute: APPLY LINEARITY LIMIT   "
                            "%ld\n",
                            __FILE__,
                            __LINE__,
                            aoloopcontrol_var.aoconfID_imWFSlinlimit);
                        //                        for(ii=0;
                        //                        ii<AOconf[loop].WFSim.sizeWFS;
                        //                        ii+=10)
                        //                            printf("    %4ld  %16f
                        //                            %16f\n", ii,
                        //                            data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F[ii],
                        //                            data.image[aoloopcontrol_var.aoconfID_imWFSlinlimit].array.F[ii]);
                        fflush(stdout);
#endif
                    }
                }
            }

            COREMOD_MEMORY_image_set_sempost_byID(
                aoloopcontrol_var.aoconfID_imWFS2,
                -1);
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 0;
        }

        AOconf[loop].AOtiminginfo.status =
            5; // 5 MULTIPLYING BY CONTROL MATRIX -> MODE VALUES
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[16] = tdiffv;

        if(AOconf[loop].aorun.initmapping ==
                0) // compute combined control matrix or matrices
        {
            printf("COMPUTING MAPPING ARRAYS .... \n");
            fflush(stdout);

            clock_gettime(CLOCK_MILK, &t1);

            //
            // There is one mapping array per WFS slice
            // WFS slice 0 = all active pixels
            //
            aoloopcontrol_var.WFS_active_map =
                (int *) malloc(sizeof(int) * AOconf[loop].WFSim.sizeWFS *
                               aoloopcontrol_var.PIXSTREAM_NBSLICES);
            if(aoloopcontrol_var.aoconfID_wfsmask != -1)
            {
                for(slice = 0; slice < aoloopcontrol_var.PIXSTREAM_NBSLICES;
                        slice++)
                {
                    long ii1 = 0;
                    for(unsigned long ii = 0; ii < AOconf[loop].WFSim.sizeWFS;
                            ii++)
                        if(data.image[aoloopcontrol_var.aoconfID_wfsmask]
                                .array.F[ii] > 0.1)
                        {
                            if(slice == 0)
                            {
                                aoloopcontrol_var.WFS_active_map
                                [slice * AOconf[loop].WFSim.sizeWFS + ii1] =
                                    ii;
                                ii1++;
                            }
                            else if(data.image
                                    [aoloopcontrol_var
                                     .aoconfID_pixstream_wfspixindex]
                                    .array.UI16[ii] == slice + 1)
                            {
                                aoloopcontrol_var.WFS_active_map
                                [slice * AOconf[loop].WFSim.sizeWFS + ii1] =
                                    ii;
                                ii1++;
                            }
                        }
                    AOconf[loop].WFSim.sizeWFS_active[slice] = ii1;

                    char imname[200];
                    if(sprintf(imname,
                               "aol%ld_imWFS2active_%02d",
                               aoloopcontrol_var.LOOPNUMBER,
                               slice) < 1)
                    {
                        PRINT_ERROR("sprintf wrote <1 char");
                    }

                    uint32_t *sizearray;
                    sizearray    = (uint32_t *) malloc(sizeof(uint32_t) * 2);
                    sizearray[0] = AOconf[loop].WFSim.sizeWFS_active[slice];
                    sizearray[1] = 1;
                    create_image_ID(imname,
                                    2,
                                    sizearray,
                                    _DATATYPE_FLOAT,
                                    1,
                                    0,
                                    0,
                                    &(aoconfID_imWFS2_active[slice]));
                    free(sizearray);
                    // aoconfID_imWFS2_active[slice] = create_2Dimage_ID(imname,
                    // AOconf[loop].WFSim.sizeWFS_active[slice], 1);
                }
            }
            else
            {
                printf("ERROR: aoloopcontrol_var.aoconfID_wfsmask = -1\n");
                fflush(stdout);
                exit(0);
            }

            // create DM active map
            aoloopcontrol_var.DM_active_map =
                (int *) malloc(sizeof(int) * AOconf[loop].DMctrl.sizeDM);
            if(aoloopcontrol_var.aoconfID_dmmask != -1)
            {
                long ii1 = 0;
                for(unsigned long ii = 0; ii < AOconf[loop].DMctrl.sizeDM;
                        ii++)
                    if(data.image[aoloopcontrol_var.aoconfID_dmmask]
                            .array.F[ii] > 0.5)
                    {
                        aoloopcontrol_var.DM_active_map[ii1] = ii;
                        ii1++;
                    }
                AOconf[loop].DMctrl.sizeDM_active = ii1;
            }

            uint32_t *sizearray;
            sizearray    = (uint32_t *) malloc(sizeof(uint32_t) * 2);
            sizearray[0] = AOconf[loop].DMctrl.sizeDM_active;
            sizearray[1] = 1;

            char imname[200];
            if(sprintf(imname,
                       "aol%ld_meas_act_active",
                       aoloopcontrol_var.LOOPNUMBER) < 1)
            {
                PRINT_ERROR("sprintf wrote <1 char");
            }

            create_image_ID(imname,
                            2,
                            sizearray,
                            _DATATYPE_FLOAT,
                            1,
                            0,
                            0,
                            &(aoloopcontrol_var.aoconfID_meas_act_active));
            free(sizearray);

            if(aoloopcontrol_var.aoconfID_meas_act == -1)
            {
                sizearray    = (uint32_t *) malloc(sizeof(uint32_t) * 2);
                sizearray[0] = AOconf[loop].DMctrl.sizexDM;
                sizearray[1] = AOconf[loop].DMctrl.sizeyDM;

                if(sprintf(imname,
                           "aol%ld_meas_act",
                           aoloopcontrol_var.LOOPNUMBER) < 1)
                {
                    PRINT_ERROR("sprintf wrote <1 char");
                }

                create_image_ID(imname,
                                2,
                                sizearray,
                                _DATATYPE_FLOAT,
                                1,
                                0,
                                0,
                                &(aoloopcontrol_var.aoconfID_meas_act));
                COREMOD_MEMORY_image_set_createsem(imname, 10);
                free(sizearray);
            }

            clock_gettime(CLOCK_MILK, &t2);
            tdiffv = timespec_diff_double(t1, t2);
            printf("\n");
            printf("TIME TO COMPUTE MAPPING ARRAYS = %f sec\n", tdiffv);
            AOconf[loop].aorun.initmapping = 1;
        }

        if(AOconf[loop].AOcompute.GPU0 == 0)  // no GPU -> run in CPU
        {

            if(AOconf[loop].aorun.CMMODE ==
                    0) // goes explicitly through modes, slower but required for
                // access to mode values
            {
#ifdef _PRINT_TEST
                printf("[%s] [%d] - CM mult: GPU=0, CMMODE=0 - %s x %s -> %s\n",
                       __FILE__,
                       __LINE__,
                       data.image[aoloopcontrol_var.aoconfID_contrM].md[0].name,
                       data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].name,
                       data.image[aoloopcontrol_var.aoconfID_meas_modes]
                       .md[0]
                       .name);
                fflush(stdout);
#endif

                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write =
                    1;

                ControlMatrixMultiply(
                    data.image[aoloopcontrol_var.aoconfID_contrM].array.F,
                    data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F,
                    AOconf[loop].AOpmodecoeffs.NBDMmodes,
                    AOconf[loop].WFSim.sizeWFS,
                    data.image[aoloopcontrol_var.aoconfID_meas_modes].array.F);

                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_meas_modes,
                    -1);
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt1 =
                    LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write =
                    0;
            }
            else // (*)
            {
#ifdef _PRINT_TEST
                printf(
                    "[%s] [%d] - CM mult: GPU=0, CMMODE=1 - using matrix %s\n",
                    __FILE__,
                    __LINE__,
                    data.image[aoloopcontrol_var.aoconfID_contrMc].md[0].name);
                printf("  aoloopcontrol_var.aoconfID_contrMc = %ld\n",
                       aoloopcontrol_var.aoconfID_contrMc);
                printf("  aoloopcontrol_var.aoconfID_meas_act = %ld\n",
                       aoloopcontrol_var.aoconfID_meas_act);
                printf("  AOconf[loop].DMctrl.sizeDM  = %ld\n",
                       AOconf[loop].DMctrl.sizeDM);
                printf("  AOconf[loop].WFSim.sizeWFS = %ld\n",
                       AOconf[loop].WFSim.sizeWFS);
                list_image_ID();
                fflush(stdout);
#endif

                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 1;

                ControlMatrixMultiply(
                    data.image[aoloopcontrol_var.aoconfID_contrMc].array.F,
                    data.image[aoloopcontrol_var.aoconfID_imWFS2].array.F,
                    AOconf[loop].DMctrl.sizeDM,
                    AOconf[loop].WFSim.sizeWFS,
                    data.image[aoloopcontrol_var.aoconfID_meas_act].array.F);

                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt0++;
                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_meas_act,
                    -1);
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt1 =
                    LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 0;
            }
        }
        else // run in GPU if possible
        {
#ifdef HAVE_CUDA
            if(AOconf[loop].aorun.CMMODE ==
                    0) // goes explicitly through modes, slower but required for
                // access to mode values
            {
#ifdef _PRINT_TEST
                printf(
                    "[%s] [%d] - CM mult: GPU=1, CMMODE=0 - using matrix %s "
                    "   GPU alpha beta = %f %f\n",
                    __FILE__,
                    __LINE__,
                    data.image[aoloopcontrol_var.aoconfID_contrM].md[0].name,
                    aoloopcontrol_var.GPU_alpha,
                    aoloopcontrol_var.GPU_beta);
                fflush(stdout);
#endif

                initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] =
                    1; // default: do not re-compute reference output

                if(AOconf[loop].AOcompute.GPUall == 1)
                {
                    // TEST IF contrM or wfsref have changed
                    if(((long long) data
                            .image[aoloopcontrol_var.aoconfID_wfsref]
                            .md[0]
                            .cnt0 !=
                            aoloopcontrol_var.aoconfcnt0_wfsref_current) ||
                            ((long long) data
                             .image[aoloopcontrol_var.aoconfID_contrM]
                             .md[0]
                             .cnt0 != aoconfcnt0_contrM_current))
                    {
                        printf("NEW wfsref [%10ld] or contrM [%10ld]\n",
                               data.image[aoloopcontrol_var.aoconfID_wfsref]
                               .md[0]
                               .cnt0,
                               data.image[aoloopcontrol_var.aoconfID_contrM]
                               .md[0]
                               .cnt0);
                        aoloopcontrol_var.aoconfcnt0_wfsref_current =
                            data.image[aoloopcontrol_var.aoconfID_wfsref]
                            .md[0]
                            .cnt0;
                        aoconfcnt0_contrM_current =
                            data.image[aoloopcontrol_var.aoconfID_contrM]
                            .md[0]
                            .cnt0;
                        initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                    }

                    if(initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] ==
                            0) // initialize WFS reference
                    {
#ifdef _PRINT_TEST
                        printf(
                            "\nINITIALIZE WFS REFERENCE: COPY NEW REF "
                            "(WFSREF) TO imWFS0\n"); // TEST
                        fflush(stdout);
#endif

                        data.image[aoloopcontrol_var.aoconfID_contrM]
                        .md[0]
                        .write = 1;
                        memcpy(data.image[aoloopcontrol_var.aoconfID_contrM]
                               .array.F,
                               data.image[aoloopcontrol_var.aoconfID_wfsref]
                               .array.F,
                               sizeof(float) * AOconf[loop].WFSim.sizeWFS);
                        COREMOD_MEMORY_image_set_sempost_byID(
                            aoloopcontrol_var.aoconfID_contrM,
                            -1);
                        data.image[aoloopcontrol_var.aoconfID_contrM]
                        .md[0]
                        .cnt0++;
                        data.image[aoloopcontrol_var.aoconfID_contrM]
                        .md[0]
                        .cnt1 = LOOPiter;
                        data.image[aoloopcontrol_var.aoconfID_contrM]
                        .md[0]
                        .write = 0;
                        fflush(stdout);
                    }
                }

#ifdef _PRINT_TEST
                printf("[%s] [%d] - AOconf[loop].AOcompute.GPUall = %d\n",
                       __FILE__,
                       __LINE__,
                       AOconf[loop].AOcompute.GPUall);
                fflush(stdout);
#endif

                if(AOconf[loop].AOcompute.GPUall == 1)
                {

                    GPU_loop_MultMat_setup(
                        0,
                        data.image[aoloopcontrol_var.aoconfID_contrM].name,
                        data.image[aoloopcontrol_var.aoconfID_contrM].name,
                        data.image[aoloopcontrol_var.aoconfID_meas_modes].name,
                        AOconf[loop].AOcompute.GPU0,
                        aoloopcontrol_var.GPUset0,
                        0,
                        AOconf[loop].AOcompute.GPUusesem,
                        initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE],
                        loop);
                }
                else
                {

                    GPU_loop_MultMat_setup(
                        0,
                        data.image[aoloopcontrol_var.aoconfID_contrM].name,
                        data.image[aoloopcontrol_var.aoconfID_imWFS2].name,
                        data.image[aoloopcontrol_var.aoconfID_meas_modes].name,
                        AOconf[loop].AOcompute.GPU0,
                        aoloopcontrol_var.GPUset0,
                        0,
                        AOconf[loop].AOcompute.GPUusesem,
                        1,
                        loop);
                }

#ifdef _PRINT_TEST
                printf("[%s] [%d] - \n", __FILE__, __LINE__);
                fflush(stdout);
#endif

                initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;

                AOconf[loop].AOtiminginfo.status = 6; // 6 execute
                clock_gettime(CLOCK_MILK, &tnow);
                tdiffv = timespec_diff_double(
                             data.image[aoloopcontrol_var.aoconfID_looptiming]
                             .md[0]
                             .atime,
                             tnow);
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[17] =
                    tdiffv;

#ifdef _PRINT_TEST
                printf("[%s] [%d] - \n", __FILE__, __LINE__);
                fflush(stdout);
#endif

                if(AOconf[loop].AOcompute.GPUall == 1)
                {

                    GPU_loop_MultMat_execute(
                        0,
                        &AOconf[loop].AOtiminginfo.status,
                        &AOconf[loop].AOtiminginfo.GPUstatus[0],
                        aoloopcontrol_var.GPU_alpha,
                        aoloopcontrol_var.GPU_beta,
                        1,
                        25);
                }
                else
                {

                    GPU_loop_MultMat_execute(
                        0,
                        &AOconf[loop].AOtiminginfo.status,
                        &AOconf[loop].AOtiminginfo.GPUstatus[0],
                        1.0,
                        0.0,
                        1,
                        25);
                }

#ifdef _PRINT_TEST
                printf("[%s] [%d] - \n", __FILE__, __LINE__);
                fflush(stdout);
#endif
            }
            else // direct pixel -> actuators linear transformation
            {
#ifdef _PRINT_TEST
                printf("[%s] [%d] - CM mult: GPU=1, CMMODE=1\n",
                       __FILE__,
                       __LINE__);
                fflush(stdout);
#endif

                // depreciated: use all pixels
                /*            if(1==0)
                          {
                              GPU_loop_MultMat_setup(0,
                 data.image[aoloopcontrol_var.aoconfID_contrMc].name,
                 data.image[aoloopcontrol_var.aoconfID_imWFS2].name,
                 data.image[aoloopcontrol_var.aoconfID_meas_act].name,
                 AOconf[loop].AOcompute.GPU0, aoloopcontrol_var.GPUset0, 0,
                 AOconf[loop].AOcompute.GPUusesem, 1, loop);
                              AOconf[loop].AOtiminginfo.status = 6;
                              clock_gettime(CLOCK_MILK, &tnow);
                              tdiffv =
                 timespec_diff_double(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                 tnow);
                 data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[6] =
                 tdiffv;

                              GPU_loop_MultMat_execute(0,
                 &AOconf[loop].AOtiminginfo.status,
                 &AOconf[loop].AOtiminginfo.GPUstatus[0], 1.0, 0.0, 1);
                          }
                          else // only use active pixels and actuators (**)
                          {*/

                // re-map input vector into imWFS2_active

                if(AOconf[loop].AOcompute.GPUall == 1)  // (**)
                {
#ifdef _PRINT_TEST
                    printf("[%s] [%d] - CM mult: GPU=1, CMMODE=1, GPUall = 1\n",
                           __FILE__,
                           __LINE__);
                    fflush(stdout);
#endif

                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .write = 1;
                    for(uint64_t wfselem_active = 0;
                            wfselem_active <
                            AOconf[loop]
                            .WFSim
                            .sizeWFS_active[aoloopcontrol_var.PIXSTREAM_SLICE];
                            wfselem_active++)
                    {
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .array.F[wfselem_active] =
                            data.image[aoloopcontrol_var.aoconfID_contrM]
                            .array
                            .F[aoloopcontrol_var.WFS_active_map
                               [aoloopcontrol_var.PIXSTREAM_SLICE *
                                AOconf[loop].WFSim.sizeWFS +
                                wfselem_active]];
                    }
                    COREMOD_MEMORY_image_set_sempost_byID(
                        aoconfID_imWFS2_active[aoloopcontrol_var
                                               .PIXSTREAM_SLICE],
                        -1);
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .cnt0++;
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .cnt1 = LOOPiter;
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .write = 0;
                }
                else
                {
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .write = 1;
                    for(uint64_t wfselem_active = 0;
                            wfselem_active <
                            AOconf[loop]
                            .WFSim
                            .sizeWFS_active[aoloopcontrol_var.PIXSTREAM_SLICE];
                            wfselem_active++)
                    {
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .array.F[wfselem_active] =
                            data.image[aoloopcontrol_var.aoconfID_imWFS2]
                            .array
                            .F[aoloopcontrol_var.WFS_active_map
                               [aoloopcontrol_var.PIXSTREAM_SLICE *
                                AOconf[loop].WFSim.sizeWFS +
                                wfselem_active]];
                    }
                    COREMOD_MEMORY_image_set_sempost_byID(
                        aoconfID_imWFS2_active[aoloopcontrol_var
                                               .PIXSTREAM_SLICE],
                        -1);
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .cnt0++;
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .cnt1 = LOOPiter;
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .md[0]
                    .write = 0;
                }

                // look for updated control matrix or reference
                if(AOconf[loop].AOcompute.GPUall == 1)  // (**)
                {
                    if((long long) data
                            .image[aoloopcontrol_var.aoconfID_contrMcact
                                   [aoloopcontrol_var.PIXSTREAM_SLICE]]
                            .md[0]
                            .cnt0 !=
                            contrMcactcnt0[aoloopcontrol_var.PIXSTREAM_SLICE])
                    {

                        printf(
                            "NEW CONTROL MATRIX DETECTED (%s) -> RECOMPUTE "
                            "REFERENCE x MATRIX\n",
                            data.image[aoloopcontrol_var.aoconfID_contrMcact
                                       [aoloopcontrol_var.PIXSTREAM_SLICE]]
                            .md[0]
                            .name);
                        fflush(stdout);

                        initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                        contrMcactcnt0[aoloopcontrol_var.PIXSTREAM_SLICE] =
                            data.image[aoloopcontrol_var.aoconfID_contrMcact
                                       [aoloopcontrol_var.PIXSTREAM_SLICE]]
                            .md[0]
                            .cnt0;
                    }

                    if((long long)(data.image[aoloopcontrol_var
                                              .aoconfID_wfsref]
                                   .md[0]
                                   .cnt0) != wfsrefcnt0) // (*)
                    {
                        printf("NEW REFERENCE WFS DETECTED (%s) [ %ld %llu ]\n",
                               data.image[aoloopcontrol_var.aoconfID_wfsref]
                               .md[0]
                               .name,
                               data.image[aoloopcontrol_var.aoconfID_wfsref]
                               .md[0]
                               .cnt0,
                               wfsrefcnt0);
                        fflush(stdout);

                        initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                        wfsrefcnt0 =
                            data.image[aoloopcontrol_var.aoconfID_wfsref]
                            .md[0]
                            .cnt0;
                    }
                    if(initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] ==
                            0) // initialize WFS reference
                    {
                        printf(
                            "\nINITIALIZE WFS REFERENCE: COPY NEW REF "
                            "(WFSREF) TO "
                            "imWFS2_active\n"); // TEST
                        fflush(stdout);
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .md[0]
                        .write = 1;
                        for(uint64_t wfselem_active = 0;
                                wfselem_active <
                                AOconf[loop].WFSim.sizeWFS_active
                                [aoloopcontrol_var.PIXSTREAM_SLICE];
                                wfselem_active++)
                        {
                            data.image[aoconfID_imWFS2_active
                                       [aoloopcontrol_var.PIXSTREAM_SLICE]]
                            .array.F[wfselem_active] =
                                data.image[aoloopcontrol_var.aoconfID_wfsref]
                                .array
                                .F[aoloopcontrol_var.WFS_active_map
                                   [aoloopcontrol_var.PIXSTREAM_SLICE *
                                    AOconf[loop].WFSim.sizeWFS +
                                    wfselem_active]];
                        }
                        COREMOD_MEMORY_image_set_sempost_byID(
                            aoconfID_imWFS2_active[aoloopcontrol_var
                                                   .PIXSTREAM_SLICE],
                            -1);
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .md[0]
                        .cnt0++;
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .md[0]
                        .cnt1 = LOOPiter;
                        data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                          .PIXSTREAM_SLICE]]
                        .md[0]
                        .write = 0;
                        fflush(stdout);
                    }
                }

                if(aoloopcontrol_var.initcontrMcact_GPU
                        [aoloopcontrol_var.PIXSTREAM_SLICE] == 0)
                {
                    initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 0;
                }

                GPU_loop_MultMat_setup(
                    0,
                    data.image[aoloopcontrol_var.aoconfID_contrMcact
                               [aoloopcontrol_var.PIXSTREAM_SLICE]]
                    .name,
                    data.image[aoconfID_imWFS2_active[aoloopcontrol_var
                                                      .PIXSTREAM_SLICE]]
                    .name,
                    data.image[aoloopcontrol_var.aoconfID_meas_act_active].name,
                    AOconf[loop].AOcompute.GPU0,
                    aoloopcontrol_var.GPUset0,
                    0,
                    AOconf[loop].AOcompute.GPUusesem,
                    initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE],
                    loop);

                initWFSref_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;
                aoloopcontrol_var
                .initcontrMcact_GPU[aoloopcontrol_var.PIXSTREAM_SLICE] = 1;

                AOconf[loop].AOtiminginfo.status = 6; // 6 execute
                clock_gettime(CLOCK_MILK, &tnow);
                tdiffv = timespec_diff_double(
                             data.image[aoloopcontrol_var.aoconfID_looptiming]
                             .md[0]
                             .atime,
                             tnow);
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[17] =
                    tdiffv;

                if(AOconf[loop].AOcompute.GPUall == 1)
                {

                    GPU_loop_MultMat_execute(
                        0,
                        &AOconf[loop].AOtiminginfo.status,
                        &AOconf[loop].AOtiminginfo.GPUstatus[0],
                        aoloopcontrol_var.GPU_alpha,
                        aoloopcontrol_var.GPU_beta,
                        1,
                        25);
                }
                else
                {

                    GPU_loop_MultMat_execute(
                        0,
                        &AOconf[loop].AOtiminginfo.status,
                        &AOconf[loop].AOtiminginfo.GPUstatus[0],
                        1.0,
                        0.0,
                        1,
                        25);
                }

                // re-map output vector
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 1;
                for(uint64_t act_active = 0;
                        act_active < AOconf[loop].DMctrl.sizeDM_active;
                        act_active++)
                {
                    data.image[aoloopcontrol_var.aoconfID_meas_act]
                    .array.F[aoloopcontrol_var.DM_active_map[act_active]] =
                        data.image[aoloopcontrol_var.aoconfID_meas_act_active]
                        .array.F[act_active];
                }

                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_meas_act,
                    -1);
                /*    for(semnb=0;
                 semnb<data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].sem;
                 semnb++)
                  {
                      sem_getvalue(data.image[aoloopcontrol_var.aoconfID_meas_act].semptr[semnb],
                 &semval); if(semval<SEMAPHORE_MAXVAL)
                          sem_post(data.image[aoloopcontrol_var.aoconfID_meas_act].semptr[semnb]);
                  }*/
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt1 =
                    LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 0;
                //}
            }
#endif
        }

        AOconf[loop].AOtiminginfo.status = 11; // 11 MULTIPLYING BY GAINS
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[18] = tdiffv;

        if(AOconf[loop].aorun.CMMODE == 0)
        {

            clock_gettime(CLOCK_MILK,
                          &functionTestTimer00); // TEST timing in function

            AOconf[loop].AOpmodecoeffs.RMSmodes = 0;
            for(unsigned int k = 0; k < AOconf[loop].AOpmodecoeffs.NBDMmodes;
                    k++)
            {
                AOconf[loop].AOpmodecoeffs.RMSmodes +=
                    data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k] *
                    data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k];
            }

            AOconf[loop].AOpmodecoeffs.RMSmodesCumul +=
                AOconf[loop].AOpmodecoeffs.RMSmodes;
            AOconf[loop].AOpmodecoeffs.RMSmodesCumulcnt++;

            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 1;

            clock_gettime(CLOCK_MILK,
                          &functionTestTimer01); // TEST timing in function

            // TEST TIMING -> COMMENT THIS SECTION
            for(unsigned int k = 0; k < AOconf[loop].AOpmodecoeffs.NBDMmodes;
                    k++)
            {

                data.image[aoloopcontrol_var.aoconfID_RMS_modes].array.F[k] =
                    0.99 * data.image[aoloopcontrol_var.aoconfID_RMS_modes]
                    .array.F[k] +
                    0.01 *
                    data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k] *
                    data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k];

                data.image[aoloopcontrol_var.aoconfID_AVE_modes].array.F[k] =
                    0.99 * data.image[aoloopcontrol_var.aoconfID_AVE_modes]
                    .array.F[k] +
                    0.01 * data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k];

                // apply gain

                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] -=
                    AOconf[loop].aorun.gain *
                    data.image[aoloopcontrol_var.aoconfID_gainb]
                    .array.F[AOconf[loop].AOpmodecoeffs.modeBlockIndex[k]] *
                    data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN]
                    .array.F[k] *
                    data.image[aoloopcontrol_var.aoconfID_meas_modes]
                    .array.F[k];

                // apply limits

                float limitval;
                limitval =
                    AOconf[loop].aorun.maxlimit *
                    data.image[aoloopcontrol_var.aoconfID_limitb]
                    .array.F[AOconf[loop].AOpmodecoeffs.modeBlockIndex[k]] *
                    data.image[aoloopcontrol_var.aoconfID_LIMIT_modes]
                    .array.F[k];

                if(data.image[aoloopcontrol_var.aoconfID_cmd_modes]
                        .array.F[k] < -limitval)
                {
                    data.image[aoloopcontrol_var.aoconfID_cmd_modes]
                    .array.F[k] = -limitval;
                }

                if(data.image[aoloopcontrol_var.aoconfID_cmd_modes]
                        .array.F[k] > limitval)
                {
                    data.image[aoloopcontrol_var.aoconfID_cmd_modes]
                    .array.F[k] = limitval;
                }

                // apply mult factor

                data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] *=
                    AOconf[loop].aorun.mult *
                    data.image[aoloopcontrol_var.aoconfID_multfb]
                    .array.F[AOconf[loop].AOpmodecoeffs.modeBlockIndex[k]] *
                    data.image[aoloopcontrol_var.aoconfID_MULTF_modes]
                    .array.F[k];

                // update total gain
                //     data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k+AOconf[loop].AOpmodecoeffs.NBDMmodes]
                //     = AOconf[loop].aorun.gain *
                //     data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k];
            }

            clock_gettime(CLOCK_MILK,
                          &functionTestTimer02); // TEST timing in function

            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt1 =
                LOOPiter;
            COREMOD_MEMORY_image_set_sempost_byID(
                aoloopcontrol_var.aoconfID_cmd_modes,
                -1);
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 0;
        }

        clock_gettime(CLOCK_MILK,
                      &functionTestTimer03); // TEST timing in function
    }
    else
    {
        if(AOconf[loop].AOcompute.GPUall == 0)
        {
            // Update imWFS2

            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 1;
            COREMOD_MEMORY_image_set_sempost_byID(
                aoloopcontrol_var.aoconfID_imWFS2,
                -1);
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].cnt1 = LOOPiter;
            data.image[aoloopcontrol_var.aoconfID_imWFS2].md[0].write = 0;
        }

        AOconf[loop].AOtiminginfo.status = 11;
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[16] = tdiffv;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[18] = tdiffv;

        if(AOconf[loop].AOcompute.GPU0 == 0)  // no GPU -> run in CPU
        {
            // -to be done
        }
        else
        {
#ifdef HAVE_CUDA
            if(AOconf[loop].aorun.CMMODE ==
                    0) // goes explicitly through modes, slower but required for
                // access to mode values
            {
                // Update meas_modes

                // -> aoloopcontrol_var.aoconfID_meas_modes
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write =
                    1;
                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_meas_modes,
                    -1);
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].cnt1 =
                    LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_modes].md[0].write =
                    0;
            }
            else // direct pixel -> actuators linear transformation
            {
                data.image
                [aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]]
                .md[0]
                .write = 1;
                COREMOD_MEMORY_image_set_sempost_byID(
                    aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE],
                    -1);
                data.image
                [aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]]
                .md[0]
                .cnt0++;
                data.image
                [aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]]
                .md[0]
                .cnt1 = LOOPiter;
                data.image
                [aoconfID_imWFS2_active[aoloopcontrol_var.PIXSTREAM_SLICE]]
                .md[0]
                .write = 0;

                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[17] =
                    tdiffv;

                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 1;
                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_meas_act,
                    -1);
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt0++;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].cnt1 =
                    LOOPiter;
                data.image[aoloopcontrol_var.aoconfID_meas_act].md[0].write = 0;
            }
#endif
        }

        if(AOconf[loop].aorun.CMMODE == 0)
        {
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 1;
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt0++;
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].cnt1 =
                LOOPiter;
            COREMOD_MEMORY_image_set_sempost_byID(
                aoloopcontrol_var.aoconfID_cmd_modes,
                -1);
            data.image[aoloopcontrol_var.aoconfID_cmd_modes].md[0].write = 0;
        }
    }

    // DETECT AND REPORT TIMING ANOMALY
    // TEST

    clock_gettime(CLOCK_MILK,
                  &functionTestTimerEnd); // TEST timing in function
    tdiffv = timespec_diff_double(functionTestTimerStart, functionTestTimerEnd);
    tdiffv01 = tdiffv;
    // TEST TIMING
    /*
    if(tdiffv > 350.0e-6)
    {
      printf("TIMING WARNING: %12.3f us  %10ld   AOcompute() after
    Read_cam_frame()\n", tdiffv*1.0e6, (long) LOOPiter); fflush(stdout);

      tdiffv = timespec_diff_double(functionTestTimerStart,
    functionTestTimer00); printf("Timer 00 : %12.3f us\n", tdiffv*1.0e6);

      tdiffv = timespec_diff_double(functionTestTimerStart,
    functionTestTimer01); printf("Timer 01 : %12.3f us\n", tdiffv*1.0e6);

      tdiffv = timespec_diff_double(functionTestTimerStart,
    functionTestTimer02); printf("Timer 02 : %12.3f us\n", tdiffv*1.0e6);

      tdiffv = timespec_diff_double(functionTestTimerStart,
    functionTestTimer03); printf("Timer 03 : %12.3f us\n", tdiffv*1.0e6);



      int timerindex1[] = { 2, 15, 16, 17, 25, 26, 27, 28, 29, 30, 31, 32, 33,
    18, 3, 4, 5, 6, 9, 10, 11, 12, 13 };


      printf("TIMING GLITCH DETECTED: %f us   [%09ld]\n", tdiffv*1.0e6,
    tnow.tv_nsec); fflush(stdout); for(i=0; i<23; i++)
      {
          int i1 = timerindex1[i];
          if((i1>26)&&(i1<32))
              printf("    ");
          printf("   %2d = %10.6f ms   Expecting %10.6f ms   Last %10.6f\n", i1,
    1000.0*data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[i1],
    1000.0*ntimerval[i1], 1000.0*ltimerval[i1]);
      }
      printf("\n");
    }*/
    /*    else
      {
          for(i=0; i<aoloopcontrol_var.AOcontrolNBtimers; i++)
          {
              if((i!=7)&&(i!=8)&&(i!=20)&&(i!=21)&&(i!=34))
              {
                  ltimerval[i] =
     data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[i]; ntimerval[i]
     = ntimerval[i]*0.99 +
     0.01*data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[i];
              }
          }
      }*/

    tdiffv = timespec_diff_double(functionTestTimer04, functionTestTimerStart);
    tdiffv00 = tdiffv;

    // TEST TIMING
    /*
    if(tdiffv > 600.0e-6)
    {
      printf("TIMING WARNING: %12.3f us  %10ld   Read_cam_frame()\n",
    tdiffv*1.0e6, (long) LOOPiter); fflush(stdout);
    }*/

    return RETURN_SUCCESS;
}
