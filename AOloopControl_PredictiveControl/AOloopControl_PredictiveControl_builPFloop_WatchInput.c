/**
 * @file    AOloopControl_PredictiveControl_builPFloop_WatchInput.c
 * @brief   Assemble input data from telemetry stream
 *
 *
 *
 *
 * ## Change log
 * - 20180518	Guyon	File creation (split from
 * AOloopControl_PredictiveControl)
 *
 *
 */

#define _GNU_SOURCE

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                        HEADER FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#include <malloc.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <time.h>

#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_PredictiveControl/AOloopControl_PredictiveControl.h"

/**
 *  ## Purpose
 *
 * Listens to real-time telemetry streams buffers aol<loop>_modeval_ol_logbuff0
 * and aol<loop>_modeval_ol_logbuff1
 *
 * When new buffer is complete, extract the requested values from it
 * and write to aol<loop>_modevalol_PFb<PFblock>
 *
 * ## Arguments
 *
 * @param[in]
 * loop 		LONG
 * 			loop index
 *
 * @param[in]
 * PFblock		LONG
 * 			Block number
 *
 * @param[in]
 * PFblockStart	LONG
 * 			Block start index
 *
 * @param[in]
 * PFblockEnd	LONG
 * 			Block end index
 *
 * #param[in]
 * NBbuff       LONG
 *          Number of input buffers to merge for each output
 *
 */
imageID AOloopControl_PredictiveControl_builPFloop_WatchInput(
    long loop, long PFblock, long PFblockStart, long PFblockEnd, long NBbuff)
{
    imageID IDinb0;
    imageID IDinb1;
    char    imnameb0[500];
    char    imnameb1[500];
    long    cnt0, cnt1;
    long    cnt0_old, cnt1_old;
    imageID IDinb;

    long twaitus = 10000; // 0.01 sec

    long PFblockSize;

    int             Tupdate = 0;
    time_t          t;
    struct tm      *uttime;
    struct timespec timenow;
    long            xsize, ysize, zsize, xysize;
    long            zsizein;
    int             cube;

    long      IDout;
    uint32_t *imsizearray;
    uint8_t   datatype;
    char      imnameout[500];
    long      ii, kk;
    long      ave;

    char inmaskname[200];
    char inmaskfname[200];
    char outmaskfname[200];
    long IDinmask;

    PROCESSINFO *processinfo;

    if(data.processinfo == 1)
    {
        // CREATE PROCESSINFO ENTRY
        // see processtools.c in module CommandLineInterface for details
        //

        char pinfoname[200]; // short name for the processinfo instance
        // avoid spaces, name should be human-readable

        sprintf(pinfoname, "PFwatchInput-loop%ld-block%ld", loop, PFblock);
        processinfo           = processinfo_shm_create(pinfoname, 0);
        processinfo->loopstat = 0; // loop initialization
        strcpy(processinfo->source_FUNCTION, __FUNCTION__);
        strcpy(processinfo->source_FILE, __FILE__);
        processinfo->source_LINE = __LINE__;

        char msgstring[200];
        sprintf(msgstring,
                "%ld->%ld %ld buffers",
                PFblockStart,
                PFblockEnd,
                NBbuff);
        processinfo_WriteMessage(processinfo, msgstring);
    }

    // CATCH SIGNALS

    if(sigaction(SIGTERM, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGTERM\n");
    }

    if(sigaction(SIGINT, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGINT\n");
    }

    if(sigaction(SIGABRT, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGABRT\n");
    }

    if(sigaction(SIGBUS, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGBUS\n");
    }

    if(sigaction(SIGSEGV, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGSEGV\n");
    }

    if(sigaction(SIGHUP, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGHUP\n");
    }

    if(sigaction(SIGPIPE, &data.sigact, NULL) == -1)
    {
        printf("\ncan't catch SIGPIPE\n");
    }

    PFblockSize = PFblockEnd - PFblockStart;

    if(sprintf(imnameb0, "aol%ld_modeval_ol_logbuff0", loop) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    if(sprintf(imnameb1, "aol%ld_modeval_ol_logbuff1", loop) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    IDinb0 = read_sharedmem_image(imnameb0);
    IDinb1 = read_sharedmem_image(imnameb1);

    cnt0_old = data.image[IDinb0].md[0].cnt0;
    cnt1_old = data.image[IDinb1].md[0].cnt0;

    xsize    = data.image[IDinb0].md[0].size[0];
    ysize    = data.image[IDinb0].md[0].size[1];
    xysize   = xsize * ysize;
    zsizein  = data.image[IDinb0].md[0].size[2];
    zsize    = data.image[IDinb0].md[0].size[2] * NBbuff;
    datatype = data.image[IDinb0].md[0].datatype;

    list_image_ID();

    EXECUTE_SYSTEM_COMMAND("mkdir -p PredictiveControl");

    if(sprintf(inmaskname, "inmaskPFb%ld", PFblock) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    create_2Dimage_ID(inmaskname, xysize, 1, &IDinmask);
    for(ii = 0; ii < xysize; ii++)
    {
        data.image[IDinmask].array.F[ii] = 0.0;
    }
    for(ii = PFblockStart; ii < PFblockEnd; ii++)
    {
        data.image[IDinmask].array.F[ii] = 1.0;
    }

    if(sprintf(inmaskfname, "./PredictiveControl/inmaskPF%ld.fits", PFblock) <
            1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    save_fits(inmaskname, inmaskfname);
    if(sprintf(outmaskfname,
               "./PredictiveControl/outmaskPF%ld.fits",
               PFblock) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    save_fits(inmaskname, outmaskfname);

    printf("Create aol%ld_modevalol_PFb%ld  : %ld x 1 x %ld\n",
           loop,
           PFblock,
           PFblockSize,
           zsize);
    fflush(stdout);
    imsizearray = (uint32_t *) malloc(sizeof(uint32_t) * 3);
    if(imsizearray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    imsizearray[0] = PFblockSize;
    imsizearray[1] = 1;
    imsizearray[2] = zsize;

    if(sprintf(imnameout, "aol%ld_modevalol_PFb%ld", loop, PFblock) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    create_image_ID(imnameout, 3, imsizearray, datatype, 1, 1, 0, &IDout);
    free(imsizearray);
    COREMOD_MEMORY_image_set_semflush(imnameout, -1);
    printf("Done\n");
    fflush(stdout);

    if(data.processinfo == 1)
        processinfo->loopstat =
            1; // Notify processinfo that we are entering loop

    long buffindex = 0;
    long outcnt    = 0;
    long loopcnt   = 0;

    int loopOK = 1;

    while(loopOK == 1)
    {

        if(data.processinfo == 1)
        {
            while(processinfo->CTRLval == 1)  // pause
            {
                usleep(50);
            }

            if(processinfo->CTRLval == 2)  // single iteration
            {
                processinfo->CTRLval = 1;
            }

            if(processinfo->CTRLval == 3)  // exit loop
            {
                loopOK = 0;
            }
        }

        cnt0 = data.image[IDinb0].md[0].cnt0;
        cnt1 = data.image[IDinb1].md[0].cnt0;

        if(cnt0 != cnt0_old)
        {
            cube     = 0;
            cnt0_old = cnt0;
            IDinb    = IDinb0;
            Tupdate  = 1;
        }

        if(cnt1 != cnt1_old)
        {
            cube     = 1;
            cnt1_old = cnt1;
            IDinb    = IDinb1;
            Tupdate  = 1;
        }

        if(Tupdate == 1)
        {
            data.image[IDout].md[0].write = 1;
            long kkin;
            for(kkin = 0; kkin < zsizein; kkin++)
            {
                kk = buffindex * zsizein + kkin;
                for(ii = 0; ii < PFblockSize; ii++)
                    data.image[IDout].array.F[kk * PFblockSize + ii] =
                        data.image[IDinb]
                        .array.F[kkin * xysize + (ii + PFblockStart)];
            }
            data.image[IDout].md[0].write = 0;

            printf("[%3ld/%3ld  %d]\n", buffindex, NBbuff, cube);
            Tupdate = 0;
            buffindex++;
        }

        if(buffindex == NBbuff)  // write output
        {
            t      = time(NULL);
            uttime = gmtime(&t);
            clock_gettime(CLOCK_REALTIME, &timenow);
            printf(
                "%02d:%02d:%02ld.%09ld  NEW TELEMETRY BUFFER AVAILABLE [%ld]\n",
                uttime->tm_hour,
                uttime->tm_min,
                timenow.tv_sec % 60,
                timenow.tv_nsec,
                outcnt);

            data.image[IDout].md[0].write = 1;
            for(ii = 0; ii < PFblockSize; ii++)  // Remove time averaged value
            {
                ave = 0.0;
                for(kk = 0; kk < zsize; kk++)
                {
                    ave += data.image[IDout].array.F[kk * PFblockSize + ii];
                }

                ave /= zsize;
                for(kk = 0; kk < zsize; kk++)
                {
                    data.image[IDout].array.F[kk * PFblockSize + ii] -= ave;
                }
            }

            COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
            data.image[IDout].md[0].cnt0++;
            data.image[IDout].md[0].write = 0;

            buffindex = 0;
            outcnt++;
        }

        usleep(twaitus);

        // process signals

        if(data.signal_TERM == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGTERM);
            }
        }

        if(data.signal_INT == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGINT);
            }
        }

        if(data.signal_ABRT == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGABRT);
            }
        }

        if(data.signal_BUS == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGBUS);
            }
        }

        if(data.signal_SEGV == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGSEGV);
            }
        }

        if(data.signal_HUP == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGHUP);
            }
        }

        if(data.signal_PIPE == 1)
        {
            loopOK = 0;
            if(data.processinfo == 1)
            {
                processinfo_SIGexit(processinfo, SIGPIPE);
            }
        }

        loopcnt++;
        if(data.processinfo == 1)
        {
            processinfo->loopcnt = loopcnt;
        }
    }

    if(data.processinfo == 1)
    {
        processinfo_cleanExit(processinfo);
    }

    return (IDout);
}
