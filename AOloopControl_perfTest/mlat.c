/**
 * @file    mlat.c
 * @brief   measure hardware latency
 *
 * Measure latency between DM and WFS
 *
 *
 */

#include <math.h>

#include <time.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "COREMOD_tools/COREMOD_tools.h" // quicksort
#include "statistic/statistic.h"         // ran1()

// Local variables pointers
static char *dmstream;
long         fpi_dmstream;

static char *wfsstream;
long         fpi_wfsstream;

static float *frameratewait;
long          fpi_frameratewait;

static float *OPDamp;
long          fpi_OPDamp;

static uint32_t *NBiter;
long             fpi_NBiter;

static uint32_t *wfsNBframemax;
long             fpi_wfsNBframemax;

static float *wfsdt;
long          fpi_wfsdt;

static float *twaitus;
long          fpi_twaitus;

static float *refdtoffset;
long          fpi_refdtoffset;

static float *dtoffset;
long          fpi_dtoffset;

static float *framerateHz;
long          fpi_framerateHz;

static float *latencyfr;
long          fpi_latencyfr;

static int64_t *saveraw;
long            fpi_saveraw;


static CLICMDARGDEF farg[] = {{
        CLIARG_STREAM,
        ".dmstream",
        "DM stream",
        "null",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmstream,
        &fpi_dmstream
    },
    {
        CLIARG_STREAM,
        ".wfsstream",
        "WFS stream",
        "null",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsstream,
        &fpi_wfsstream
    },
    {
        CLIARG_FLOAT32,
        ".OPDamp",
        "poke amplitude [um]",
        "0.1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &OPDamp,
        &fpi_OPDamp
    },
    {
        CLIARG_FLOAT32,
        ".frameratewait",
        "time period for frame rate measurement",
        "5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &frameratewait,
        &fpi_frameratewait
    },
    {
        CLIARG_UINT32,
        ".NBiter",
        "Number of iteration",
        "100",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBiter,
        &fpi_NBiter
    },
    {
        CLIARG_UINT32,
        ".wfsNBframemax",
        "Number frames in measurement sequence",
        "50",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsNBframemax,
        &fpi_wfsNBframemax
    },
    {
        CLIARG_FLOAT32,
        ".status.wfsdt",
        "WFS frame interval",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &wfsdt,
        &fpi_wfsdt
    },
    {
        CLIARG_FLOAT32,
        ".status.twaitus",
        "initial wait [us]",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &twaitus,
        &fpi_twaitus
    },
    {
        CLIARG_FLOAT32,
        ".status.refdtoffset",
        "baseline time offset to poke",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &refdtoffset,
        &fpi_refdtoffset
    },
    {
        CLIARG_FLOAT32,
        ".status.dtoffset",
        "actual time offset to poke",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &dtoffset,
        &fpi_dtoffset
    },
    {
        CLIARG_FLOAT32,
        ".out.framerateHz",
        "WFS frame rate [Hz]",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &framerateHz,
        &fpi_framerateHz
    },
    {
        CLIARG_FLOAT32,
        ".out.latencyfr",
        "hardware latency [frame]",
        "0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &latencyfr,
        &fpi_latencyfr
    },
    {
        CLIARG_ONOFF,
        ".option.saveraw",
        "Save raw image cubes",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &saveraw,
        &fpi_saveraw
    },
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_dmstream].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_wfsstream].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_saveraw].fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.fpsptr != NULL)
    {}

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "mlat", "measure latency between DM and WFS", CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{
    printf("Measure latency between two streams\n");

    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // uint32_t dmxsize;
    // uint32_t dmysize;

    // connect to DM
    IMGID imgdm = mkIMGID_from_name(dmstream);
    resolveIMGID(&imgdm, ERRMODE_ABORT);
    printf("DM size : %u %u\n", imgdm.md->size[0], imgdm.md->size[1]);

    // connect to WFS
    IMGID imgwfs = mkIMGID_from_name(wfsstream);
    resolveIMGID(&imgwfs, ERRMODE_ABORT);
    printf("WFS size : %u %u\n", imgwfs.md->size[0], imgwfs.md->size[1]);

    // create wfs image cube for storage
    imageID IDwfsc;
    {
        uint32_t naxes[3];
        naxes[0] = imgwfs.md->size[0];
        naxes[1] = imgwfs.md->size[1];
        naxes[2] = *wfsNBframemax;

        create_image_ID("_testwfsc",
                        3,
                        naxes,
                        imgwfs.datatype,
                        0,
                        0,
                        0,
                        &IDwfsc);
    }

    float *latencyarray = (float *) malloc(sizeof(float) * *NBiter);
    if(latencyarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    float *latencysteparray = (float *) malloc(sizeof(float) * *NBiter);
    if(latencysteparray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    // Create DM patterns
    long IDdm0 = -1;
    long IDdm1 = -1;
    {
        uint32_t dmxsize = imgdm.md->size[0];
        uint32_t dmysize = imgdm.md->size[1];

        create_2Dimage_ID("_testdm0", dmxsize, dmysize, &IDdm0);
        create_2Dimage_ID("_testdm1", dmxsize, dmysize, &IDdm1);

        float RMStot = 0.0;
        for(uint32_t ii = 0; ii < dmxsize; ii++)
            for(uint32_t jj = 0; jj < dmysize; jj++)
            {
                float x = (2.0 * ii - 1.0 * dmxsize) / dmxsize;
                float y = (2.0 * jj - 1.0 * dmxsize) / dmysize;
                data.image[IDdm0].array.F[jj * dmxsize + ii] = 0.0;
                data.image[IDdm1].array.F[jj * dmxsize + ii] =
                    (*OPDamp) * (sin(20.0 * x) * sin(20.0 * y));
                RMStot += data.image[IDdm1].array.F[jj * dmxsize + ii] *
                          data.image[IDdm1].array.F[jj * dmxsize + ii];
            }
        RMStot = sqrt(RMStot / dmxsize / dmysize);

        printf("RMStot = %f", RMStot);

        for(uint32_t ii = 0; ii < dmxsize; ii++)
            for(uint32_t jj = 0; jj < dmysize; jj++)
            {
                data.image[IDdm1].array.F[jj * dmxsize + ii] *=
                    (*OPDamp) / RMStot;
            }

        // save output
        fps_write_RUNoutput_image(data.fpsptr, "_testdm0", "pokeDM0");
        fps_write_RUNoutput_image(data.fpsptr, "_testdm1", "pokeDM1");
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        // Measure coarse estimate of frame rate
        int framerateOK = 0;
        {
            double          tdouble_start;
            double          tdouble_end;
            long            wfscntstart;
            long            wfscntend;
            struct timespec tnow;

            long stringmaxlen = 200;
            char msgstring[stringmaxlen];
            snprintf(msgstring,
                     stringmaxlen,
                     "Measuring approx frame rate over %.1f sec",
                     *frameratewait);
            processinfo_WriteMessage(processinfo, msgstring);

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdouble_start = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
            wfscntstart   = imgwfs.md->cnt0;

            {
                long nsec           = (long)(1000000000 * (*frameratewait));
                long nsec_remaining = nsec % 1000000000;
                long sec            = nsec / 1000000000;

                struct timespec timesleep;
                timesleep.tv_sec  = sec;
                timesleep.tv_nsec = nsec_remaining;

                nanosleep(&timesleep, NULL);
            }

            //        usleep( (long)  (1000000 * (*frameratewait)) );

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdouble_end = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
            wfscntend   = imgwfs.md->cnt0;
            *wfsdt      = (tdouble_end - tdouble_start) / (wfscntend - wfscntstart);

            printf("wfs dt = %f sec\n", *wfsdt);

            if(wfscntend - wfscntstart < 5)
            {
                snprintf(msgstring,
                         stringmaxlen,
                         "Number of frames %ld too small -> cannot proceed",
                         wfscntend - wfscntstart);
                processinfo_error(processinfo, msgstring);
                printf("%s\n", msgstring);
            }
            else
            {
                framerateOK = 1;
                snprintf(msgstring,
                         stringmaxlen,
                         "frame period wfsdt = %f sec  ( %f Hz )\n",
                         *wfsdt,
                         1.0 / *wfsdt);
                processinfo_WriteMessage(processinfo, msgstring);

                // This is approximate, will be measured more precisely later on
                *framerateHz = 1.0 / (*wfsdt);
            }
        }

        if(framerateOK == 1)
        {
            // Measure latency
            double tdouble_start;
            double tdouble_end;

            double dtmax; // Max running time per iteration

            // update timing parameters for poke test
            dtmax        = *wfsdt * (*wfsNBframemax) * 1.2 + 0.5;
            *twaitus     = 1000000.0 * *wfsdt * 3.0; // wait 3 frames
            *refdtoffset = 0.5 * *wfsdt;

            struct timespec *tarray;
            tarray = (struct timespec *) malloc(sizeof(struct timespec) *
                                                (*wfsNBframemax));
            if(tarray == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }

            double *dtarray;
            dtarray = (double *) malloc(sizeof(double) * (*wfsNBframemax));
            if(dtarray == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }

            FILE *fphwlat =
                fps_write_RUNoutput_file(data.fpsptr, "hardwlatency", "dat");

            struct timespec tnow;
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdouble_start       = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
            long wfscntstart    = imgwfs.md->cnt0;
            long wfsframeoffset = (long)(0.1 * (*wfsNBframemax));

            // Measurement loop

            uint32_t iter   = 0;
            int      loopOK = 1;
            while(loopOK == 1)
            {
                // double tlastdouble;
                double        tstartdouble;
                long          NBwfsframe;
                unsigned long wfscnt0;
                double        latencymax = 0.0;
                double        latency;

                int  stringmaxlen = 500;
                char msgstring[stringmaxlen];

                uint64_t wfssize = imgwfs.md->size[0] * imgwfs.md->size[1];

                struct timespec tstart;
                long            kkoffset = 0;
                long            kkmax    = 0;

                snprintf(msgstring,
                         stringmaxlen,
                         "iteration %5u / %5u",
                         iter,
                         *NBiter);
                processinfo_WriteMessage(processinfo, msgstring);

                printf(" - ITERATION %u / %u\n", iter, *NBiter);
                fflush(stdout);

                printf("write to %s\n", dmstream);
                fflush(stdout);
                copy_image_ID("_testdm0", dmstream, 1);

                unsigned int dmstate = 0;

                // waiting time
                // usleep(*twaitus);

                {
                    long nsec = (long)(1000 * (*twaitus));

                    long nsec_remaining = nsec % 1000000000;
                    long sec            = nsec / 1000000000;

                    struct timespec timesleep;
                    timesleep.tv_sec  = sec;
                    timesleep.tv_nsec = nsec_remaining;

                    nanosleep(&timesleep, NULL);
                }

                // and waiting frames
                wfscnt0 = imgwfs.md->cnt0;
                for(uint32_t wfsframe = 0; wfsframe < *wfsNBframemax; wfsframe++)
                {
                    while(wfscnt0 == imgwfs.md->cnt0)
                    {
                        long nsec = (long)(1000 * 50);  // 50 usec

                        long nsec_remaining = nsec % 1000000000;
                        long sec            = nsec / 1000000000;

                        struct timespec timesleep;
                        timesleep.tv_sec  = sec;
                        timesleep.tv_nsec = nsec_remaining;

                        nanosleep(&timesleep, NULL);
                    }
                    wfscnt0 = imgwfs.md->cnt0;
                }

                double dt = 0.0;
                clock_gettime(CLOCK_REALTIME, &tstart);
                tstartdouble = 1.0 * tstart.tv_sec + 1.0e-9 * tstart.tv_nsec;
                //    tlastdouble = tstartdouble;

                long wfsframe = 0;
                int  wfsslice = 0;
                wfscnt0       = imgwfs.md->cnt0;
                printf("\n");
                while((dt < dtmax) && (wfsframe < *wfsNBframemax))
                {
                    // WAITING for image
                    while(wfscnt0 == imgwfs.md->cnt0)
                    {
                        long nsec = (long)(1000 * 2);  // 2 usec

                        long nsec_remaining = nsec % 1000000000;
                        long sec            = nsec / 1000000000;

                        struct timespec timesleep;
                        timesleep.tv_sec  = sec;
                        timesleep.tv_nsec = nsec_remaining;

                        nanosleep(&timesleep, NULL);
                    }

                    wfscnt0 = imgwfs.md->cnt0;
                    printf("[%8ld / %8u]  %lf  %lf\n",
                           wfsframe,
                           *wfsNBframemax,
                           dt,
                           dtmax);
                    fflush(stdout);

                    /*
                    if(CIRCBUFFER == 1) //TODO
                    {
                      wfsslice = imgwfs.md->cnt1;
                    }
                    else
                    {*/
                    wfsslice = 0;
                    //}

                    int   datatype_size = ImageStreamIO_typesize(imgwfs.datatype);
                    char *ptr0          = ImageStreamIO_get_image_d_ptr(imgwfs.im);
                    ptr0 += datatype_size * wfsslice * wfssize;

                    char *ptr = ImageStreamIO_get_image_d_ptr(&data.image[IDwfsc]);
                    ptr += datatype_size * wfsframe * wfssize;

                    memcpy(ptr, ptr0, datatype_size * wfssize);
                    //


                    clock_gettime(CLOCK_REALTIME, &tarray[wfsframe]);

                    double tdouble = 1.0 * tarray[wfsframe].tv_sec +
                                     1.0e-9 * tarray[wfsframe].tv_nsec;
                    dt = tdouble - tstartdouble;
                    //  dt1 = tdouble - tlastdouble;
                    dtarray[wfsframe] = dt;
                    //     tlastdouble = tdouble;

                    // apply DM pattern #1
                    if((dmstate == 0) && (dt > *refdtoffset) &&
                            (wfsframe > wfsframeoffset))
                    {
                        //                    usleep((long)(ran1() * 1000000.0 *
                        //                    *wfsdt));
                        {
                            long nsec = (long)(1000000000.0 * ran1() * (*wfsdt));

                            long nsec_remaining = nsec % 1000000000;
                            long sec            = nsec / 1000000000;

                            struct timespec timesleep;
                            timesleep.tv_sec  = sec;
                            timesleep.tv_nsec = nsec_remaining;

                            nanosleep(&timesleep, NULL);
                        }

                        printf("\nDM STATE CHANGED ON ITERATION %ld   / %ld\n\n",
                               wfsframe,
                               wfsframeoffset);
                        kkoffset = wfsframe;

                        dmstate = 1;
                        copy_image_ID("_testdm1", dmstream, 1);

                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdouble   = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
                        dt        = tdouble - tstartdouble;
                        *dtoffset = dt; // time at which DM command is sent
                        printf("    dt = %lf\n\n", dt);
                    }
                    wfsframe++;
                }
                printf("\n\n %ld frames recorded\n", wfsframe);
                fflush(stdout);

                copy_image_ID("_testdm0", dmstream, 1);
                dmstate = 0;


                if(data.fpsptr->parray[fpi_saveraw].fpflag & FPFLAG_ONOFF)
                {
                    // Save each datacube
                    //
                    char ffnameC[STRINGMAXLEN_FULLFILENAME];
                    WRITE_FULLFILENAME(ffnameC,
                                       "mlat-testC-%04d.fits", iter);
                    fps_write_RUNoutput_image(data.fpsptr, "_testwfsc", ffnameC);
                }

                // Computing difference between consecutive images
                NBwfsframe = wfsframe;

                double *valarray = (double *) malloc(sizeof(double) * NBwfsframe);
                if(valarray == NULL)
                {
                    PRINT_ERROR("malloc returns NULL pointer");
                    abort();
                }

                double valmax   = 0.0;
                double valmaxdt = 0.0;

// Define a macro for the type switching that follows
#define IMAGE_SUMMING_CASE(IMG_PTR_ID)                        \
    {                                                                          \
        for (uint64_t ii = 0; ii < wfssize; ii++)                              \
        {                                                                      \
            double tmp =                                                       \
                1.0*data.image[IDwfsc].array.IMG_PTR_ID[kk * wfssize + ii] -       \
                1.0*data.image[IDwfsc].array.IMG_PTR_ID[(kk - 1) * wfssize + ii];  \
            valarray[kk] += 1.0 * tmp * tmp;                                   \
        }                                                                      \
    }

                for(long kk = 1; kk < NBwfsframe; kk++)
                {
                    valarray[kk] = 0.0;

                    switch(imgwfs.datatype)
                    {
                        case _DATATYPE_FLOAT:
                            IMAGE_SUMMING_CASE(F);
                            break;
                        case _DATATYPE_DOUBLE:
                            IMAGE_SUMMING_CASE(D);
                            break;
                        case _DATATYPE_UINT16:
                            IMAGE_SUMMING_CASE(UI16);
                            break;
                        case _DATATYPE_INT16:
                            IMAGE_SUMMING_CASE(SI16);
                            break;
                        case _DATATYPE_UINT32:
                            IMAGE_SUMMING_CASE(UI32);
                            break;
                        case _DATATYPE_INT32:
                            IMAGE_SUMMING_CASE(SI32);
                            break;
                        case _DATATYPE_UINT64:
                            IMAGE_SUMMING_CASE(UI64);
                            break;
                        case _DATATYPE_INT64:
                            IMAGE_SUMMING_CASE(SI64);
                            break;
                        case _DATATYPE_COMPLEX_FLOAT:
                        case _DATATYPE_COMPLEX_DOUBLE:
                        default:
                            PRINT_ERROR("COMPLEX TYPES UNSUPPORTED");
                            return RETURN_FAILURE;
                    }

                    valarray[kk] = sqrt(valarray[kk] / wfssize / 2);

                    if(valarray[kk] > valmax)
                    {
                        valmax   = valarray[kk];
                        valmaxdt = 0.5 * (dtarray[kk - 1] + dtarray[kk]);
                        kkmax    = kk - kkoffset;
                    }
                }

                //
                //
                //
                for(wfsframe = 1; wfsframe < NBwfsframe; wfsframe++)
                {
                    fprintf(fphwlat,
                            "%ld   %10.2f     %g\n",
                            wfsframe - kkoffset,
                            1.0e6 *
                            (0.5 * (dtarray[wfsframe] + dtarray[wfsframe - 1]) -
                             *dtoffset),
                            valarray[wfsframe]);
                }

                printf("mean interval =  %10.2f ns\n",
                       1.0e9 * (dt - *dtoffset) / NBwfsframe);
                fflush(stdout);

                free(valarray);

                latency = valmaxdt - *dtoffset;
                // latencystep = kkmax;

                printf("... Hardware latency = %f ms  = %ld frames\n",
                       1000.0 * latency,
                       kkmax);

                if(latency > latencymax)
                {
                    latencymax = latency;
                    // WRITE_FULLFILENAME(ffname, "%s/maxlatencyseq.fits",
                    // outdirname); save_fl_fits("_testwfsc", ffname);
                }

                fprintf(fphwlat, "# %5u  %8.6f\n", iter, (valmaxdt - *dtoffset));

                latencysteparray[iter] = 1.0 * kkmax;
                latencyarray[iter]     = (valmaxdt - *dtoffset);

                // process signals, increment loop counter
                iter++;
                if(iter == (*NBiter))
                {
                    loopOK = 0;
                }
            }
            fclose(fphwlat);

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdouble_end    = 1.0 * tnow.tv_sec + 1.0e-9 * tnow.tv_nsec;
            long wfscntend = imgwfs.md->cnt0;

            free(tarray);
            free(dtarray);

            // PROCESS RESULTS


            processinfo_WriteMessage_fmt(processinfo, "Processing Data (%u iterations)",
                                         (*NBiter));


            copy_image_ID("_testdm0", dmstream, 1);

            float latencyave     = 0.0;
            float latencystepave = 0.0;
            float minlatency     = latencyarray[0];
            float maxlatency     = latencyarray[0];
            for(uint32_t iter = 0; iter < (*NBiter); iter++)
            {
                if(latencyarray[iter] > maxlatency)
                {
                    maxlatency = latencyarray[iter];
                }

                if(latencyarray[iter] < minlatency)
                {
                    minlatency = latencyarray[iter];
                }

                latencyave += latencyarray[iter];
                latencystepave += latencysteparray[iter];
            }
            latencyave /= (*NBiter);
            latencystepave /= (*NBiter);

            // measure precise frame rate
            double dt = tdouble_end - tdouble_start;
            printf("FRAME RATE = %.3f Hz\n", 1.0 * (wfscntend - wfscntstart) / dt);
            *framerateHz = 1.0 * (wfscntend - wfscntstart) / dt;
            functionparameter_SaveParam2disk(data.fpsptr, ".out.framerateHz");

            // update latencystepave from framerate
            latencystepave = latencyave * (1.0 * (wfscntend - wfscntstart) / dt);

            quick_sort_float(latencyarray, (*NBiter));

            printf("AVERAGE LATENCY = %8.3f ms   %f frames\n",
                   latencyave * 1000.0,
                   latencystepave);
            printf("min / max over %u measurements: %8.3f ms / %8.3f ms\n",
                   (*NBiter),
                   minlatency * 1000.0,
                   maxlatency * 1000.0);

            *latencyfr = latencystepave;
            functionparameter_SaveParam2disk(data.fpsptr, ".out.latencyfr");

            FILE *fpout;
            fpout =
                fps_write_RUNoutput_file(data.fpsptr, "param_hardwlatency", "txt");
            fprintf(fpout, "%8.6f", 1.01);
            fclose(fpout);
        }
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(latencyarray);
    free(latencysteparray);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_perfTest__mlat()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
