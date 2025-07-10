/**
 * @file    timesample.c
 * @brief   measure hardware latency
 *
 * Build time sample
 *
 *
 */

#include <math.h>

#include <time.h>
#include <dirent.h>


#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "COREMOD_tools/COREMOD_tools.h" // quicksort
#include "statistic/statistic.h"         // ran1()


typedef struct
{
    char   name[500];
    double tstart;
    double tend;
    uint64_t cnt0start;
    uint64_t cnt0end;
    long   cnt;
} StreamDataFile;

#define MaxNBdatFiles 100000




// Resampled frame data
// Keeps track of what has gone into the frame
//
typedef struct
{
    double tstart;       // start time
    double tend;         // end time
    double avTime;       // average time stamp
    double etimesec;     // exposure time accumulated [sec]
    double etimeframe;   // exposure time accumulated [frame]
} ResampledFrame;


// PROCESSTIMINGFLAG

#define PROCESSTIMINGFLAG_LOAD         0x00000001
#define PROCESSTIMINGFLAG_WRITE        0x00000002
#define PROCESSTIMINGFLAG_LINTIMING    0x00000004  // overwrite timings to force linear timing








// Local variables pointers

// Start time: sec, nanosec
static uint32_t *tstartsec;
long          fpi_tstartsec;

static uint32_t *tstartnsec;
long          fpi_tstartnsec;


// End time: sec, nanosec
static uint32_t *tendsec;
long          fpi_tendsec;

static uint32_t *tendnsec;
long          fpi_tendnsec;



// Synchro mode
// -1: adopt custom timing
// 0+ : inherit timing from stream
static int64_t *timingmode;
long            fpi_timingmode;

// time interval
static float *timingdt;
long          fpi_timingdt;

// logging directory
static char *logdir;
long         fpi_logdir;



// stream 0

// name
// NULL if inactive
static char *(sname[4]);
long         fpi_sname[4];

// name tag
// allows for processed copy of cubes to be ingested
// for example, tag could be ".crop.darksub" for cropped dark subtracted image
static char *(stag[4]);
long         fpi_stag[4];

// FUll name
// FITS   : ./<LOGDIR>/UTDATE/<SNAME>/<SNAME>_HH:MM:SS.sssssssss<TAG>.fits
// TIMING : ./<LOGDIR>/UTDATE/<SNAME>/<SNAME>_HH:MM:SS.sssssssss.txt

// convention: positive if frames arrive late
// real time = reported time - latency
static float *(slatency[4]);
long         fpi_slatency[4];

// Force linear timing
// Assumes input stream is acquired with regular timing
static int64_t *(lintiming[4]);
long         fpi_lintiming[4];



#define CLISTREAMLOGENTRY(INDEX) \
    {\
        CLIARG_STR,\
        ".s"#INDEX"name",\
        "stream "#INDEX" name",\
        "null",\
        CLIARG_HIDDEN_DEFAULT,\
        (void **) &sname[INDEX],\
        &fpi_sname[INDEX]\
    },\
    {\
        CLIARG_STR,\
        ".s"#INDEX"tag",\
        "stream "#INDEX" tag",\
        "",\
        CLIARG_HIDDEN_DEFAULT,\
        (void **) &stag[INDEX],\
        &fpi_stag[INDEX]\
    },\
    {\
        CLIARG_FLOAT32,\
        ".s"#INDEX"latency",\
        "stream "#INDEX" latency [float]",\
        "0.0",\
        CLIARG_HIDDEN_DEFAULT,\
        (void **) &slatency[INDEX],\
        &fpi_slatency[INDEX]\
    },\
    {\
        CLIARG_ONOFF,\
        ".s"#INDEX"lint",\
        "stream "#INDEX" linearize timing",\
        "1",\
        CLIARG_HIDDEN_DEFAULT,\
        (void **) &lintiming[INDEX],\
        &fpi_lintiming[INDEX]\
    }






static CLICMDARGDEF farg[] = {
    {
        CLIARG_UINT32,
        ".tstartsec",
        "tstartsec",
        "1728797840",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &tstartsec,
        &fpi_tstartsec
    },
    {
        CLIARG_UINT32,
        ".tstartnsec",
        "tstartnsec",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &tstartnsec,
        &fpi_tstartnsec
    },
    {
        CLIARG_UINT32,
        ".tendsec",
        "tendsec",
        "1728797850",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &tendsec,
        &fpi_tendsec
    },
    {
        CLIARG_UINT32,
        ".tendnsec",
        "tendnsec",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &tendnsec,
        &fpi_tendnsec
    },
    {
        CLIARG_INT32,
        ".timingmode",
        "timing mode (0+: inherit from stream)",
        "-1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &timingmode,
        &fpi_timingmode
    },
    {
        CLIARG_FLOAT32,
        ".timingdt",
        "output frame interval",
        "0.001",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &timingdt,
        &fpi_timingdt
    },
    {
        CLIARG_STR,
        ".logdir",
        "log directory",
        ".",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &logdir,
        &fpi_logdir
    },
    CLISTREAMLOGENTRY(0),
    CLISTREAMLOGENTRY(1),
    CLISTREAMLOGENTRY(2),
    CLISTREAMLOGENTRY(3)
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

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
    "slogtsample", "resample streams to common clock", CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    printf("resample streams to common clock\n");

    printf(
        "Convention\n"
    );

    return RETURN_SUCCESS;
}





static char *remove_ext(
    char *mystr,
    char dot,
    char sep
)
{
    char *retstr, *lastdot, *lastsep;

    // Error checks and allocate string.

    if(mystr == NULL)
    {
        return NULL;
    }
    if((retstr = malloc(strlen(mystr) + 1)) == NULL)
    {
        return NULL;
    }

    // Make a copy and find the relevant characters.

    strcpy(retstr, mystr);
    lastdot = strrchr(retstr, dot);
    lastsep = (sep == 0) ? NULL : strrchr(retstr, sep);

    // If it has an extension separator.

    if(lastdot != NULL)
    {
        // and it's before the extenstion separator.

        if(lastsep != NULL)
        {
            if(lastsep < lastdot)
            {
                // then remove it.

                *lastdot = '\0';
            }
        }
        else
        {
            // Has extension separator with no path separator.

            *lastdot = '\0';
        }
    }

    // Return the modified string.

    return retstr;
}




static void quicksort_StreamDataFile(
    StreamDataFile *datfile,
    long left,
    long right
)
{
    register long  i, j;
    StreamDataFile x, y;

    i        = left;
    j        = right;
    x.tstart = datfile[(left + right) / 2].tstart;

    do
    {
        while(datfile[i].tstart < x.tstart && i < right)
        {
            i++;
        }
        while(x.tstart < datfile[j].tstart && j > left)
        {
            j--;
        }

        if(i <= j)
        {
            y.tstart = datfile[i].tstart;
            y.tend   = datfile[i].tend;
            y.cnt    = datfile[i].cnt;
            strcpy(y.name, datfile[i].name);

            datfile[i].tstart = datfile[j].tstart;
            datfile[i].tend   = datfile[j].tend;
            datfile[i].cnt    = datfile[j].cnt;
            strcpy(datfile[i].name, datfile[j].name);

            datfile[j].tstart = y.tstart;
            datfile[j].tend   = y.tend;
            datfile[j].cnt    = y.cnt;
            strcpy(datfile[j].name, y.name);

            i++;
            j--;
        }
    }
    while(i <= j);

    if(left < j)
    {
        quicksort_StreamDataFile(datfile, left, j);
    }
    if(i < right)
    {
        quicksort_StreamDataFile(datfile, i, right);
    }
}






static errno_t processTimingFile(
    char *inTimingfname,
    char *outTimingfname,
    char *fnamestring,
    uint64_t PROCESSTIMINGFLAG,
    double* timingarray
)
{
    FILE *fp;
    if((fp = fopen(inTimingfname, "r")) == NULL)
    {
        printf("Cannot open file \"%s\"\n", inTimingfname);
        exit(0);
    }
    else
    {
        double tfirst  = 0.0;
        double tlast  = 0.0;
        uint64_t cnt0first = 0;
        uint64_t cnt0last = 0;

        int    tOK    = 1;
        int    scanOK = 1;

        double  MaxNBsample = 1000000;
        double *tarray = (double *) malloc(sizeof(double) * MaxNBsample);
        if(tarray == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort(); // or handle error in other ways
        }


        long *cnt0array = (long*) malloc(sizeof(long) * MaxNBsample);
        if(cnt0array == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort(); // or handle error in other ways
        }

        long cnt = 0;


        double  cubetimesec, abslogtimesec, absacqtimesec;
        long    cubeframenumber, framenumber, framecnt0, framecnt1;
        while(scanOK == 1)
        {
            char line[512];
            if(fgets(line, sizeof(line), fp) == NULL)
            {
                scanOK = 0;
            }
            else
            {
                if(line[0] != '#')
                {
                    scanOK = 1;
                }

                if(scanOK == 1)
                {
                    if((sscanf(line,
                               "%ld %ld %lf %lf %lf %ld %ld\n",
                               &cubeframenumber,
                               &framenumber,
                               &cubetimesec,
                               &abslogtimesec,
                               &absacqtimesec,
                               &framecnt0,
                               &framecnt1) == 7) &&
                            (tOK == 1))
                    {
                        // Use logtime instead of acqtime
                        double abstimesec = abslogtimesec;

                        tarray[cnt] = abstimesec;
                        cnt0array[cnt] = framecnt0;

                        if(PROCESSTIMINGFLAG & PROCESSTIMINGFLAG_LOAD)
                        {
                            timingarray[cnt] = abstimesec;
                        }

                        if(cnt == 0)
                        {
                            tfirst         = abstimesec;
                            tlast          = abstimesec;
                            cnt0first      = framecnt0;
                            cnt0last       = framecnt0;
                        }
                        else
                        {
                            // if enforcing monotonic time
                            if(abstimesec > tlast)
                            {
                                tOK = 1;
                            }
                            else
                            {
                                tOK = 0;
                            }
                            tlast = abstimesec;
                            cnt0last = framecnt0;
                        }
                        cnt++;
                    }
                }
            }

            if(tOK == 0)
            {
                scanOK = 0;
            }
        }
        fclose(fp);


        if(PROCESSTIMINGFLAG & PROCESSTIMINGFLAG_LOAD)
        {
            if(PROCESSTIMINGFLAG & PROCESSTIMINGFLAG_LINTIMING)
            {
                // linear extrapolation
                printf("Apply linear extrapolation\n");
                fflush(stdout);

                // average first and last 1/4 point
                double ave0_cnt0 = 0.0;
                double ave1_cnt0 = 0.0;
                long ave0cnt = 0;

                double ave0_time = 0.0;
                double ave1_time = 0.0;
                long ave1cnt = 0;

                for(long pti=0; pti<cnt; pti++)
                {
                    if(pti < 0.25*cnt)
                    {
                        ave0_cnt0 += cnt0array[pti];
                        ave0_time += tarray[pti];
                        ave0cnt++;
                    }
                    if(pti > 0.75*cnt)
                    {
                        ave1_cnt0 += cnt0array[pti];
                        ave1_time += tarray[pti];
                        ave1cnt++;
                    }
                }
                ave0_cnt0 /= ave0cnt;
                ave0_time /= ave0cnt;

                ave1_cnt0 /= ave1cnt;
                ave1_time /= ave1cnt;

                // replace values with linear extrapolation
                //

                // keep track of delta relative to lin extrapolation

                double * timingdelta = (double*) malloc(sizeof(double) * cnt);

                {
                    //FILE *fptest = fopen("timingtext.txt", "w");

                    double slope = (ave1_time-ave0_time) / (ave1_cnt0-ave0_cnt0);
                    for(long pti=0; pti<cnt; pti++)
                    {
                        double x = cnt0array[pti];
                        double y = ave0_time + (x-ave0_cnt0) * slope;
                        //fprintf(fptest, "%5ld %5ld %f %f\n", pti, cnt0array[pti], timingarray[pti], y);
                        timingdelta[pti] = timingarray[pti] - y;
                        timingarray[pti] = y;
                    }
                    //fclose(fp);
                }

                quick_sort_double(timingdelta, cnt);
                double mediandelta = timingdelta[cnt/2];
                free(timingdelta);

                {
                    //FILE *fptest = fopen("timingtext1.txt", "w");
                    for(long pti=0; pti<cnt; pti++)
                    {
                        timingarray[pti] += mediandelta;
                        //fprintf(fptest, "%5ld %f  %f\n", pti, timingarray[pti], mediandelta);
                    }

                    //fclose(fp);
                }


            }
        }


        free(tarray);
        free(cnt0array);




        // write timing summary file
        if(PROCESSTIMINGFLAG & PROCESSTIMINGFLAG_WRITE)
        {
            StreamDataFile datfile;

            datfile.tstart = tfirst;
            datfile.cnt0start = cnt0first;
            datfile.tend = tlast;
            datfile.cnt0end = cnt0last;
            datfile.cnt  = cnt;
            strcpy(datfile.name, fnamestring);

            FILE *fpout;
            if((fpout = fopen(outTimingfname, "w")) == NULL)
            {
                printf("Cannot write file \"%s\"\n", outTimingfname);
                exit(0);
            }
            else
            {
                fprintf(fpout,
                        "%s   %lu %20.9f   %lu %20.9f   %10ld  %10.3f\n",
                        fnamestring,
                        datfile.cnt0start,
                        datfile.tstart,
                        datfile.cnt0end,
                        datfile.tend,
                        datfile.cnt,
                        datfile.cnt / (datfile.tend - datfile.tstart));
                fclose(fpout);
            }
        }
    }


    return RETURN_SUCCESS;
}



















static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    printf("START TIME = %u.%09u\n", *tstartsec, *tstartnsec);
    printf(" END  TIME = %u.%09u\n", *tendsec, *tendnsec);
    printf("\n");




    // loop over streams
    for(int sindex=0; sindex < 4; sindex++)
    {
        int ouputimginit = 0;

        // skip steams is name = null
        if ( strcmp(sname[sindex], "null") )
        {
            printf("STREAM %d : %s\n", sindex, sname[sindex]);


            char datadirstream[STRINGMAXLEN_DIRNAME];
            {
                time_t timetstart = *tstartsec;
                struct tm* tminfo;

                tminfo = gmtime(&timetstart);

                char datestr[9];
                strftime(datestr, sizeof(datestr), "%Y%m%d", tminfo);
                printf("UT date : %s\n", datestr);
                WRITE_DIRNAME(datadirstream, "%s/%s/%s/", logdir, datestr, sname[sindex]);
            }



            printf("Scanning directory : %s\n", datadirstream);

            StreamDataFile *datfile;
            datfile = (StreamDataFile *) malloc(sizeof(StreamDataFile) * MaxNBdatFiles);
            if(datfile == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }




            // Find timing files overlapping with time interval
            //
            int NBdatFiles = 0;

            DIR *d0;
            d0  = opendir(datadirstream);
            if(d0)
            {
                struct dirent *dir;
                while((dir = readdir(d0)) != NULL)
                {
                    char          *ext;
                    ext = strrchr(dir->d_name, '.');
                    if(!ext)
                    {
                        // printf("no extension\n");
                    }
                    else
                    {
                        int datfileOK = 0;
                        if(strcmp(ext + 1, "dat") == 0)
                        {
                            datfileOK = 1;
                        }
                        if(strcmp(ext + 1, "txt") == 0)
                        {
                            datfileOK = 2;
                        }

                        if(datfileOK != 0)
                        {
                            printf("\n    processing file %s\n", dir->d_name);
                            // int mkTiming;
                            float tmpv;
                            int   ret;

                            char          *tmpstring;
                            tmpstring = remove_ext(dir->d_name, '.', '/');

                            // Does timing file exist ?
                            char  timingfname[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(timingfname,
                                               "%s/%s.timing",
                                               datadirstream,
                                               tmpstring);



                            FILE *fp;
                            if((fp = fopen(timingfname, "r")) == NULL)
                            {
                                char fnamein[STRINGMAXLEN_FULLFILENAME];

                                printf(
                                    "    timing file %s missing -> creating\n",
                                    timingfname);

                                if(datfileOK == 1)
                                {
                                    snprintf(fnamein,
                                             STRINGMAXLEN_FULLFILENAME,
                                             "%s/%s.dat",
                                             datadirstream,
                                             tmpstring);
                                }
                                else
                                {
                                    snprintf(fnamein,
                                             STRINGMAXLEN_FULLFILENAME,
                                             "%s/%s.txt",
                                             datadirstream,
                                             tmpstring);
                                }

                                printf("input  : %s\n", fnamein);
                                printf("output : %s\n", timingfname);

                                processTimingFile(fnamein,
                                                  timingfname,
                                                  tmpstring,
                                                  PROCESSTIMINGFLAG_WRITE, NULL);

                                if((fp = fopen(timingfname, "r")) == NULL)
                                {
                                    printf("ERROR: can't open file %s\n", timingfname);
                                    exit(0);
                                }
                            }
                            else
                            {
                                printf(
                                    "    timing file %s found\n",
                                    timingfname);
                            }


                            // read timing file
                            int scanOK = 1; // keep scanning file
                            int readOK = 0; // read successful

                            while(scanOK == 1)
                            {
                                char line[512];
                                if(fgets(line, sizeof(line), fp) == NULL)
                                {
                                    scanOK = 0;
                                }

                                if(line[0] != '#')
                                {
                                    ret = sscanf(line,
                                                 "%s   %lu %lf   %lu %lf   %ld  %f\n",
                                                 tmpstring,
                                                 &datfile[NBdatFiles].cnt0start,
                                                 &datfile[NBdatFiles].tstart,
                                                 &datfile[NBdatFiles].cnt0end,
                                                 &datfile[NBdatFiles].tend,
                                                 &datfile[NBdatFiles].cnt,
                                                 &tmpv);

                                    if(ret == 7)
                                    {
                                        // mkTiming = 0;
                                        strcpy(datfile[NBdatFiles].name, tmpstring);
                                        // printf("File %s : timing info found\n",
                                        // fname);
                                        scanOK = 0; // done reading
                                        readOK = 1;
                                    }
                                }
                            }
                            fclose(fp);

                            if(readOK == 0)
                            {
                                printf("File %s corrupted \n", timingfname);
                                exit(0);
                            }

                            if((datfile[NBdatFiles].tstart < 1.0*(*tendsec) + 1e-9*(*tendnsec)) &&
                                    (datfile[NBdatFiles].tend > 1.0*(*tstartsec) + 1.0e-9*(*tstartnsec)) &&
                                    (datfile[NBdatFiles].cnt > 0))
                            {
                                NBdatFiles++;
                            }
                        }
                    }
                }
                closedir(d0);
            }





            printf("NBdatFiles = %d\n", NBdatFiles);

            if(NBdatFiles > 1)
            {
                quicksort_StreamDataFile(datfile, 0, NBdatFiles - 1);
            }





            // prepare output frame array
            double outtimestart = *tstartsec + 1e-9 * (*tstartnsec);
            double outtimeend = *tendsec + 1e-9 * (*tendnsec);
            printf("    outtimestart = %.9lf sec\n", outtimestart);
            printf("    outtimeend   = %.9lf sec\n", outtimeend);
            long zsizeout = (outtimeend - outtimestart) / *timingdt;
            printf("zsizeout = %ld\n", zsizeout);
            ResampledFrame *outframearray = (ResampledFrame*) malloc(sizeof(ResampledFrame)*zsizeout);


            for(long tstep = 0; tstep < zsizeout; tstep++)
            {
                outframearray[tstep].tstart = outtimestart + 1.0 * tstep * (outtimeend - outtimestart) / zsizeout;
                outframearray[tstep].tend = outtimestart + 1.0 * (tstep + 1) * (outtimeend - outtimestart) / zsizeout;
                outframearray[tstep].etimesec  = 0.0;
                outframearray[tstep].etimeframe = 0.0;
            }



            IMGID imgout = makeIMGID_blank();

            for(int idatfile = 0; idatfile < NBdatFiles; idatfile++)
            {
                printf("FILE %d / %d\n", idatfile, NBdatFiles);
                fflush(stdout);

                printf(
                    "FILE [%d]: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f "
                    "Hz\n",
                    idatfile,
                    datfile[idatfile].name,
                    datfile[idatfile].tstart,
                    datfile[idatfile].tend,
                    datfile[idatfile].cnt,
                    datfile[idatfile].cnt / (datfile[idatfile].tend - datfile[idatfile].tstart));

                printf("LOADING TXT FILE\n");
                fflush(stdout);

                char fnameTXT[STRINGMAXLEN_FULLFILENAME];
                WRITE_FILENAME(fnameTXT,
                               "%s/%s.txt",
                               datadirstream,
                               datfile[idatfile].name);
                printf("----------------------[%d] LOADING FILE %s\n", idatfile, fnameTXT);
                double dtin = (datfile[idatfile].tend - datfile[idatfile].tstart)/(datfile[idatfile].cnt-1);
                printf("    dtin    = %.9lf sec  (%f Hz)\n", dtin, 1.0/dtin);
                printf("    latency = %.9f sec  (%f frame)\n", *slatency[sindex], *slatency[sindex]/dtin);






                // PREPARE MAPPING COMMANDS
                //

                long frameinmin = datfile[idatfile].cnt;
                long frameinmax = 0;

                long maxNBcmd = 100000;
                long cmdindex = 0;
                long *mapping_orig = (long*) malloc(sizeof(long)*maxNBcmd); // input
                long *mapping_dest = (long*) malloc(sizeof(long)*maxNBcmd); // output
                double *mapping_coeff = (double*) malloc(sizeof(double)*maxNBcmd);


                double *tarrayin = (double*) malloc(sizeof(double)*datfile[idatfile].cnt);


                {
                    char fnameTXTout[STRINGMAXLEN_FULLFILENAME];
                    WRITE_FILENAME(fnameTXTout,
                                   "%s.out.txt",
                                   datfile[idatfile].name);
                    if(*lintiming[sindex] == 1)
                    {
                        processTimingFile(fnameTXT, fnameTXTout, sname[sindex], PROCESSTIMINGFLAG_LOAD|PROCESSTIMINGFLAG_LINTIMING|PROCESSTIMINGFLAG_WRITE, tarrayin);
                    }
                    else
                    {
                        processTimingFile(fnameTXT, fnameTXTout, sname[sindex], PROCESSTIMINGFLAG_LOAD|PROCESSTIMINGFLAG_WRITE, tarrayin);
                    }
                }


                // increments if input frame falls withing output cube
                long NBinframeOK = 0;
                for ( long framein=0; framein < datfile[idatfile].cnt; framein++)
                {
                    // Unix times
                    double inframetimestart = (tarrayin[framein] - dtin) - *slatency[sindex];
                    double inframetimeend   = (tarrayin[framein]) - *slatency[sindex];

                    printf("%4ld   %.3f\n", framein, tarrayin[framein]);
                    printf("  inframetimestart/end:  %.3f %.3f\n", inframetimestart, inframetimeend);

                    // remap timing to frame index
                    double findexframestart = (inframetimestart - outtimestart)/(*timingdt);
                    double findexframeend   = (inframetimeend   - outtimestart)/(*timingdt);

                    // if frame falls within output cube
                    if((findexframeend > 0) && (findexframestart < zsizeout))
                    {
                        NBinframeOK++;
                        //printf("input file %3d frame %4ld maps to output frame range [%f - %f]\n",
                        //       idatfile, framein, findexframestart, findexframeend);
                        long frameout0 = (long) (findexframestart);
                        long frameout1 = (long) (findexframeend+1.0);
                        for(long frameout=frameout0; frameout < frameout1; frameout++)
                        {
                            double istart = 1.0*frameout;
                            double iend = 1.0*(frameout+1);
                            if (findexframestart > istart)
                            {
                                istart = findexframestart;
                            }
                            if (findexframeend < iend)
                            {
                                iend = findexframeend;
                            }
                            double expfrac = iend - istart;

                            if((frameout>-1)&&(frameout < zsizeout))
                            {
                                printf("  [%3d / %3d]  %4ld/%4ld  -> %4ld/%4ld    %4ld:%4ld    %8.6f  \n",
                                       idatfile,
                                       NBdatFiles,
                                       framein, datfile[idatfile].cnt,
                                       frameout, zsizeout,
                                       frameout0, frameout1,
                                       expfrac);
                                mapping_orig[cmdindex] = framein;
                                mapping_dest[cmdindex] = frameout;
                                mapping_coeff[cmdindex] = expfrac;

                                outframearray[frameout].etimeframe += expfrac;
                                outframearray[frameout].etimesec += expfrac*dtin;

                                cmdindex ++;
                            }

                            if( framein < frameinmin)
                            {
                                frameinmin = framein;
                            }

                            if( framein > frameinmax)
                            {
                                frameinmax = framein;
                            }
                        }
                    }
                }
                maxNBcmd = cmdindex;
                free(tarrayin);

                printf("NBinframeOK = %ld\n", NBinframeOK);
                printf("datfile[idatfile].cnt = %ld\n", datfile[idatfile].cnt);
                printf("frameinmin = %ld\n", frameinmin);
                printf("frameinmax = %ld\n", frameinmax);



                // RUN MAPPING COMMANDS
                //
                if(NBinframeOK>0)
                {

                    // load relevant section of input data cube
                    //
                    imageID IDc;
                    char fnameFITS[STRINGMAXLEN_FULLFILENAME];
                    WRITE_FILENAME(fnameFITS,
                                   "%s/%s%s.fits[*,*,%ld:%ld]",
                                   datadirstream,
                                   datfile[idatfile].name,
                                   stag[sindex],
                                   frameinmin+1,
                                   frameinmax+1);

                    printf("----------------------[%d] LOADING FILE %s\n", idatfile, fnameFITS);
                    load_fits(fnameFITS, "im0C", 1, &IDc);



                    uint32_t xsize = data.image[IDc].md->size[0];
                    uint32_t ysize = data.image[IDc].md->size[1];
                    uint32_t zsizein = data.image[IDc].md->size[2];
                    uint64_t xysize = xsize;
                    xysize *= ysize;

                    if(ouputimginit == 0)
                    {
                        imgout = makeIMGID_3D(sname[sindex], xsize, ysize, zsizeout);
                        createimagefromIMGID(&imgout);
                        ouputimginit = 1;
                    }
                    printf("Writing to image %s\n", sname[sindex]);


                    list_image_ID();

                    for ( long cmdindex=0; cmdindex < maxNBcmd; cmdindex++)
                    {
                        mapping_orig[cmdindex] -= frameinmin;
                        if((mapping_dest[cmdindex]>-1)&&(mapping_dest[cmdindex]<zsizeout))

                            printf("mapping slice %5ld/%5d (%d x %d) to %5ld/%5ld (%d x %d)\n",
                                   mapping_orig[cmdindex], zsizein, data.image[IDc].md->size[0], data.image[IDc].md->size[1],
                                   mapping_dest[cmdindex], zsizeout, imgout.im->md->size[0], imgout.im->md->size[1] );

                        //printf("CMD %4ld / %4ld : %3ld -> %3ld\n", cmdindex, maxNBcmd, mapping_orig[cmdindex], mapping_dest[cmdindex]);

                        switch(data.image[IDc].md->datatype)
                        {
                        case _DATATYPE_UINT8:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.UI8[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_INT8:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.SI8[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_UINT16:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.UI16[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_INT16:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.SI16[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_UINT32:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.UI32[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_INT32:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.SI32[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_UINT64:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.UI64[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_INT64:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.SI64[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_FLOAT:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.F[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        case _DATATYPE_DOUBLE:
                            for(uint64_t ii = 0; ii < xysize; ii++)
                            {
                                imgout.im->array.F[xysize * mapping_dest[cmdindex] + ii] +=
                                    mapping_coeff[cmdindex] *
                                    data.image[IDc].array.D[xysize * mapping_orig[cmdindex] + ii];
                            }
                            break;

                        default:
                            list_image_ID();
                            PRINT_ERROR("datatype value not recognised");
                            printf("ID %ld  datatype = %d\n",
                                   IDc,
                                   data.image[IDc].md[0].datatype);
                            exit(0);
                            break;
                        }


                    }

                    list_image_ID();

                    printf("Removing image im0C\n");
                    fflush(stdout);

                    delete_image_ID("im0C", DELETE_IMAGE_ERRMODE_WARNING);
                }


                printf("Freeing memory\n");
                fflush(stdout);

                free(mapping_orig);
                free(mapping_dest);
                free(mapping_coeff);

                printf("Memory freed\n");
                fflush(stdout);

                printf("INPUT FRAME RANGE : %ld - %ld\n", frameinmin, frameinmax);
                printf("===================================\n\n\n");
            }

            {
                // write output timing file
                //
                FILE *fptimingout = fopen("timing.sync.txt", "w");
                fprintf(fptimingout, "# outframe   etimeframe   etimesec\n");

                for(long tstep = 0; tstep < zsizeout; tstep++)
                {
                    fprintf(fptimingout, "%4ld   %6.3f  %9.6f\n",
                           tstep,
                           outframearray[tstep].etimeframe,
                           outframearray[tstep].etimesec);
                }
                fclose(fptimingout);
            }

            printf("Free outframearray\n");
            fflush(stdout);
            free(outframearray);

            printf("Free datfile\n");
            fflush(stdout);
            free(datfile);

        }
    }









    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END



    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions


// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_perfTest__streamlogtimesample()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
