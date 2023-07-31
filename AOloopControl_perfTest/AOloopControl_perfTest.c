/**
 * @file    AOloopControl_perfTest.c
 * @brief   Adaptive Optics Control loop engine testing
 *
 * AO engine uses stream data structure
 *
 *
 * ## Main files
 *
 * AOloopControl_perfTest_DM.c               Test DM speed and examine DM modes
 * AOloopControl_perfTest_status.c           Report loop metrics, loop
 * performance monitor
 *
 *
 *
 */

/* ================================================================== */
/* ================================================================== */
/*            MODULE INFO                                             */
/* ================================================================== */
/* ================================================================== */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaopt"

// Module short description
#define MODULE_DESCRIPTION "AO loop control performance monitoring and testing"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                        HEADER FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */
#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string.h> /* strrchr */
#include <sys/stat.h>
#include <unistd.h> /* chdir */

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

#include "compRMsensitivity.h"
#include "mlat.h"



#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

#define MaxNBdatFiles 100000



// TIMING
static struct timespec tnow;
// static struct timespec tdiff;

typedef struct
{
    char   name[500];
    double tstart;
    double tend;
    long   cnt;
} StreamDataFile;



INIT_MODULE_LIB(AOloopControl_perfTest)





static errno_t init_module_CLI()
{




    CLIADDCMD_AOloopControl_perfTest__compRMsensitivity();
    CLIADDCMD_AOloopControl_perfTest__mlat();

    return RETURN_SUCCESS;
}


char *remove_ext(char *mystr, char dot, char sep)
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

//
// WARNING: right=NBelem-1
//
void quicksort_StreamDataFile(StreamDataFile *datfile, long left, long right)
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

/**
 * # Purpose
 *
 * Create timing summary file
 *
 */
errno_t AOloopControl_perfTest_mkTimingFile(char *inTimingfname,
        char *outTimingfname,
        char *tmpstring)
{
    FILE          *fp;
    FILE          *fpout;
    StreamDataFile datfile;
    long           cnt;
    double         valf1, valf2;
    long           vald1, vald2, vald3, vald4;
    char           line[512];
    long           linecnt = 0;

    double *tarray;
    double  MaxNBsample = 1000000;

    if((fp = fopen(inTimingfname, "r")) == NULL)
    {
        printf("Cannot open file \"%s\"\n", inTimingfname);
        exit(0);
    }
    else
    {
        double tlast  = 0.0;
        int    tOK    = 1;
        int    scanOK = 1;

        tarray = (double *) malloc(sizeof(double) * MaxNBsample);
        if(tarray == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort(); // or handle error in other ways
        }

        cnt = 0;

        while(scanOK == 1)
        {
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
                               "%ld %ld %lf %lf %ld %ld\n",
                               &vald1,
                               &vald2,
                               &valf1,
                               &valf2,
                               &vald3,
                               &vald4) == 6) &&
                            (tOK == 1))
                    {
                        // printf("cnt %5ld read\n", cnt);//TEST
                        tarray[cnt] = valf2;

                        if(cnt == 0)
                        {
                            datfile.tstart = valf2;
                            tlast          = valf2;
                        }
                        else
                        {
                            if(valf2 > tlast)
                            {
                                tOK = 1;
                            }
                            else
                            {
                                tOK = 0;
                            }
                            tlast = valf2;
                        }
                        cnt++;
                    }
                }
            }

            if(tOK == 0)
            {
                scanOK = 0;
            }

            // printf("[%5ld] [%d] LINE: \"%s\"\n", linecnt, scanOK, line);
            linecnt++;
        }
        fclose(fp);
        datfile.tend = valf2;
        datfile.cnt  = cnt;
        strcpy(datfile.name, tmpstring);

        free(tarray);
    }

    // printf("datfile.tstart  = %f\n", datfile.tstart);
    // printf("datfile.tend    = %f\n", datfile.tend);

    // write timing summary file

    if((fpout = fopen(outTimingfname, "w")) == NULL)
    {
        printf("Cannot write file \"%s\"\n", outTimingfname);
        exit(0);
    }
    else
    {
        fprintf(fpout,
                "%s   %20.9f %20.9f   %10ld  %10.3f\n",
                tmpstring,
                datfile.tstart,
                datfile.tend,
                datfile.cnt,
                datfile.cnt / (datfile.tend - datfile.tstart));
        fclose(fpout);
    }

    return RETURN_SUCCESS;
}

/**
 * # Purpose
 *
 * Synchronize two telemetry streams
 *
 * # Arguments
 *
 * savedir is the location of the telemetry, for example /media/data/20180202
 *
 * dtlag: positive when stream0 is earlier than stream1
 *
 */
errno_t AOloopControl_perfTest_mkSyncStreamFiles2(char  *datadir,
        char  *stream0,
        char  *stream1,
        double tstart,
        double tend,
        double dt,
        double dtlag)
{
    int stringmaxlen = 500;

    DIR           *d0;
    struct dirent *dir;
    char           datadirstream[stringmaxlen];
    char          *ext;
    char          *tmpstring;
    char           line[stringmaxlen];

    StreamDataFile *datfile;
    long            NBdatFiles;

    FILE *fp;
    char  fname[STRINGMAXLEN_FULLFILENAME];
    // long cnt;
    double valf1, valf2;
    long   vald1, vald2, vald3, vald4;
    long   i;

    // uint32_t stream0xsize;
    // uint32_t stream0ysize;
    uint32_t zsize;
    double  *tstartarray;
    double  *tendarray;
    double  *exparray;
    double  *exparray0;
    double  *exparray1;
    long     tstep;

    double *intarray_start;
    double *intarray_end;
    double *dtarray;

    long xysize;

    double dtoffset;

    long IDout0, IDout1;
    long xysize0, xysize1;
    long xsize0, ysize0, xsize1, ysize1;

    // double dtlagarray[10]; // maximum 10 streams
    double medianexptimearray[10];

    // compute exposure start for each slice of output

    // How many frames are expected in the output ?
    zsize = (tend - tstart) / dt;
    printf("zsize = %ld\n", (long) zsize);
    fflush(stdout);

    // Should frame be kept or not ?
    int *frameOKarray;
    frameOKarray = (int *) malloc(sizeof(double) * zsize);
    if(frameOKarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    // Allocate Working arrays and populate timing arrays

    tstartarray = (double *) malloc(sizeof(double) * zsize);
    if(tstartarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    tendarray = (double *) malloc(sizeof(double) * zsize);
    if(tendarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    exparray = (double *) malloc(
                   sizeof(double) *
                   zsize); // exposure time accumulated, in unit of input frame(s)
    if(exparray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    exparray0 = (double *) malloc(sizeof(double) * zsize);
    if(exparray0 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    exparray1 = (double *) malloc(sizeof(double) * zsize);
    if(exparray1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }
    for(tstep = 0; tstep < zsize; tstep++)
    {
        tstartarray[tstep] = tstart + 1.0 * tstep * (tend - tstart) / zsize;
        tendarray[tstep] = tstart + 1.0 * (tstep + 1) * (tend - tstart) / zsize;
        exparray[tstep]  = 0.0;
        frameOKarray[tstep] = 1;
    }

    printf("tstart = %20.8f\n", tstart);
    printf("tend   = %20.8f\n", tend);

    // dtlagarray[0] = dtlag;

    int stream;
    int NBstream = 2;
    for(stream = 0; stream < NBstream; stream++)
    {
        for(tstep = 0; tstep < zsize; tstep++)
        {
            exparray[tstep] = 0.0;
        }

        if(stream == 0)
        {
            dtoffset = 0.0; // stream 0 is used as reference
            snprintf(datadirstream, stringmaxlen, "%s/%s", datadir, stream0);
        }
        else
        {
            dtoffset =
                +dtlag; // stream 1 is lagging behind by dtlag, so we bring
            // it back in time
            // this is achieved by pushing/delaying the output timing window
            snprintf(datadirstream, stringmaxlen, "%s/%s", datadir, stream1);
        }

        datfile =
            (StreamDataFile *) malloc(sizeof(StreamDataFile) * MaxNBdatFiles);
        if(datfile == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort(); // or handle error in other ways
        }
        //
        // Identify relevant files in directory
        //

        printf("SCANNING directory (stream %d)  %s ... \n",
               stream,
               datadirstream);
        fflush(stdout);

        NBdatFiles = 0;
        d0         = opendir(datadirstream);
        if(d0)
        {
            while((dir = readdir(d0)) != NULL)
            {
                printf("found file %s\n", dir->d_name);
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
                        printf("    processing file %s\n", dir->d_name);
                        // int mkTiming;
                        float tmpv;
                        int   ret;

                        tmpstring = remove_ext(dir->d_name, '.', '/');

                        // TIMING FILE
                        //

                        // Does timing file exist ?
                        WRITE_FULLFILENAME(fname,
                                           "%s/%s.timing",
                                           datadirstream,
                                           tmpstring);
                        if((fp = fopen(fname, "r")) == NULL)
                        {
                            char fnamein[stringmaxlen];

                            printf(
                                "File %s : No timing info found -> creating\n",
                                fname);

                            if(datfileOK == 1)
                            {
                                snprintf(fnamein,
                                         stringmaxlen,
                                         "%s/%s.dat",
                                         datadirstream,
                                         tmpstring);
                            }
                            else
                            {
                                snprintf(fnamein,
                                         stringmaxlen,
                                         "%s/%s.txt",
                                         datadirstream,
                                         tmpstring);
                            }

                            printf("input  : %s\n", fnamein);
                            printf("output : %s\n", fname);

                            AOloopControl_perfTest_mkTimingFile(fnamein,
                                                                fname,
                                                                tmpstring);

                            if((fp = fopen(fname, "r")) == NULL)
                            {
                                printf("ERROR: can't open file %s\n", fname);
                                exit(0);
                            }
                        }

                        // read timing file

                        int scanOK = 1; // keep scanning file
                        int readOK = 0; // read successful
                        // int linenb = 0;
                        while(scanOK == 1)
                        {
                            // printf("Reading line %d\n", linenb); //TEST

                            if(fgets(line, sizeof(line), fp) == NULL)
                            {
                                scanOK = 0;
                            }

                            if(line[0] != '#')
                            {
                                ret = sscanf(line,
                                             "%s   %lf %lf   %ld  %f\n",
                                             tmpstring,
                                             &datfile[NBdatFiles].tstart,
                                             &datfile[NBdatFiles].tend,
                                             &datfile[NBdatFiles].cnt,
                                             &tmpv);

                                if(ret == 5)
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
                            printf("File %s corrupted \n", fname);
                            exit(0);
                        }

                        if((datfile[NBdatFiles].tstart < tend) &&
                                (datfile[NBdatFiles].tend > tstart) &&
                                (datfile[NBdatFiles].cnt > 0))
                        {
                            NBdatFiles++;
                        }
                    }
                }
            }
            closedir(d0);
        }

        printf("\ndone\n");
        fflush(stdout);

        printf("NBdatFiles = %ld\n", NBdatFiles);

        for(i = 0; i < NBdatFiles; i++)
        {
            printf(
                "FILE [%ld]: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f "
                "Hz\n",
                i,
                datfile[i].name,
                datfile[i].tstart,
                datfile[i].tend,
                datfile[i].cnt,
                datfile[i].cnt / (datfile[i].tend - datfile[i].tstart));
        }

        printf("==========================================================\n");

        // sort files according to time
        if(NBdatFiles > 1)
        {
            quicksort_StreamDataFile(datfile, 0, NBdatFiles - 1);
        }

        for(i = 0; i < NBdatFiles; i++)
        {
            printf(
                "FILE [%ld]: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f "
                "Hz\n",
                i,
                datfile[i].name,
                datfile[i].tstart,
                datfile[i].tend,
                datfile[i].cnt,
                datfile[i].cnt / (datfile[i].tend - datfile[i].tstart));
        }

        printf("==========================================================\n");

        int  initOutput = 0;
        long xsize, ysize;
        long IDout;

        for(i = 0; i < NBdatFiles; i++)
        {
            printf("FILE: %20s       %20.9f -> %20.9f   [%10ld]  %10.3f Hz\n",
                   datfile[i].name,
                   datfile[i].tstart,
                   datfile[i].tend,
                   datfile[i].cnt,
                   datfile[i].cnt / (datfile[i].tend - datfile[i].tstart));

            // LOAD FITS FILE
            long IDc;
            WRITE_FILENAME(fname,
                           "%s/%s.fits",
                           datadirstream,
                           datfile[i].name);
            printf("----------------------[%ld] LOADING FILE %s\n", i, fname);
            load_fits(fname, "im0C", 1, &IDc);

            // CREATE OUTPUT CUBE IF FIRST FILE
            if(initOutput == 0)
            {
                xsize  = data.image[IDc].md[0].size[0];
                ysize  = data.image[IDc].md[0].size[1];
                xysize = xsize * ysize;
                if(stream == 0)
                {
                    create_3Dimage_ID("out0", xsize, ysize, zsize, &IDout);
                    IDout0  = IDout;
                    xysize0 = xysize;
                    xsize0  = xsize;
                    ysize0  = ysize;
                }
                else
                {
                    create_3Dimage_ID("out1", xsize, ysize, zsize, &IDout);
                    IDout1  = IDout;
                    xysize1 = xysize;
                    xsize1  = xsize;
                    ysize1  = ysize;
                }
                initOutput = 1;
            }

            // start and end time for input exposures
            intarray_start = (double *) malloc(sizeof(double) * datfile[i].cnt);
            if(intarray_start == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }

            intarray_end = (double *) malloc(sizeof(double) * datfile[i].cnt);
            if(intarray_end == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }

            dtarray = (double *) malloc(sizeof(double) * datfile[i].cnt);
            if(dtarray == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort(); // or handle error in other ways
            }

            long j;

            WRITE_FILENAME(fname,
                           "%s/%s.dat",
                           datadirstream,
                           datfile[i].name);

            printf("READING file \"%s\" ... ", fname);
            fflush(stdout);

            if((fp = fopen(fname, "r")) == NULL)
            {
                WRITE_FILENAME(fname, "%s/%s.txt",
                               datadirstream,
                               datfile[i].name);


                if((fp = fopen(fname, "r")) == NULL)
                {
                    printf("Cannot open file \"%s.dat\" or \"%s.txt\"\n",
                           datfile[i].name,
                           datfile[i].name);
                    exit(0);
                }
            }

            printf("DONE\n");
            fflush(stdout);

            int scanOK = 1;
            j          = 0;
            while(scanOK == 1)
            {
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
                        if(sscanf(line,
                                  "%ld %ld %lf %lf %ld %ld\n",
                                  &vald1,
                                  &vald2,
                                  &valf1,
                                  &valf2,
                                  &vald3,
                                  &vald4) == 6)
                        {
                            intarray_end[j] = valf2;
                            j++;
                            if(j == datfile[i].cnt)
                            {
                                scanOK = 0;
                            }
                        }
                    }
                }
            }

            /*
                      for(j=0; j<datfile[i].cnt; j++)
                      {
                          if(fscanf(fp, "%ld %ld %lf %lf %ld %ld\n", &vald1,
             &vald2, &valf1, &valf2, &vald3, &vald4)!=6)
                          {
                              printf("fscanf error, %s line %d\n", __FILE__,
             __LINE__); exit(0);
                          }
                          else
                              intarray_end[j] = valf2;
                      }*/
            fclose(fp);

            printf(" %ld lines processed\n", j);
            fflush(stdout);

            for(j = 0; j < datfile[i].cnt - 1; j++)
            {
                dtarray[j] = intarray_end[j + 1] - intarray_end[j];
            }

            double dtmedian;
            printf("Sorting %ld interval values ...", datfile[i].cnt - 1);
            fflush(stdout);
            qs_double(dtarray, 0, (unsigned long)(datfile[i].cnt - 1));
            dtmedian = dtarray[(datfile[i].cnt - 1) / 2];
            printf("   dtmedian = %10.3f us\n", 1.0e6 * dtmedian);
            fflush(stdout);

            // we assume here that every frame has the same exposure time, with
            // 100% duty cycle
            for(j = 0; j < datfile[i].cnt; j++)
            {
                intarray_start[j] = intarray_end[j] - dtmedian;
            }

            int    j0 = 0;
            double expfrac;

            for(tstep = 0; tstep < zsize; tstep++)
            {
                while((intarray_end[j0] < (tstartarray[tstep] + dtoffset)) &&
                        (j0 < datfile[i].cnt))
                {
                    j0++;
                }
                j = j0;

                while((intarray_start[j] < (tendarray[tstep] + dtoffset)) &&
                        (j < datfile[i].cnt))
                {
                    expfrac = 1.0;

                    if((tstartarray[tstep] + dtoffset) > intarray_start[j])
                    {
                        expfrac -= ((tstartarray[tstep] + dtoffset) -
                                    intarray_start[j]) /
                                   dtmedian;
                    }

                    if((tendarray[tstep] + dtoffset) < intarray_end[j])
                    {
                        expfrac -=
                            (intarray_end[j] - (tendarray[tstep] + dtoffset)) /
                            dtmedian;
                    }

                    exparray[tstep] += expfrac;

                    //                    printf("  FILE %d        %5ld   %8.6f
                    //                    [%20.6f]
                    //                    -> %5ld\n", i, j, expfrac,
                    //                    intarray_start[j], tstep);

                    long ii;

                    switch(data.image[IDc].md[0].datatype)
                    {
                    case _DATATYPE_UINT8:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.UI8[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_INT8:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.SI8[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_UINT16:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.UI16[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_INT16:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.SI16[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_UINT32:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.UI32[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_INT32:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.SI32[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_UINT64:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.UI64[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_INT64:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.SI64[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_FLOAT:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.F[xysize * j + ii];
                        }
                        break;

                    case _DATATYPE_DOUBLE:
                        for(ii = 0; ii < xysize; ii++)
                        {
                            data.image[IDout].array.F[xysize * tstep + ii] +=
                                expfrac *
                                data.image[IDc].array.D[xysize * j + ii];
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
                    j++;
                }
            }

            delete_image_ID("im0C", DELETE_IMAGE_ERRMODE_WARNING);

            free(intarray_start);
            free(intarray_end);
            free(dtarray);
        }

        for(tstep = 0; tstep < zsize; tstep++)
        {
            if(exparray[tstep] > 0.01)
            {
                long ii;
                for(ii = 0; ii < xysize; ii++)
                {
                    data.image[IDout].array.F[xysize * tstep + ii] /=
                        exparray[tstep];
                }
            }
        }

        printf("zsize = %ld\n", (long) zsize);
        fflush(stdout);

        // COMPUTE MEDIAN EXPTIME
        if(stream == 0)
        {
            memcpy(exparray0, exparray, sizeof(double) * zsize);
        }
        else
        {
            memcpy(exparray1, exparray, sizeof(double) * zsize);
        }

        quick_sort_double(exparray, zsize);

        double exptmedian;
        exptmedian                 = exparray[zsize / 2];
        medianexptimearray[stream] = exptmedian;
        printf("Median Exp Time = %6.3f\n", exptmedian);

        if(fabs(exptmedian) < 0.0000001)
        {
            printf(
                "Median Exp Time = 0 , exiting the process: check your data "
                "and times");
            exit(0);
        }

        if(stream == 0)
        {
            memcpy(exparray, exparray0, sizeof(double) * zsize);
        }
        else
        {
            memcpy(exparray, exparray1, sizeof(double) * zsize);
        }

        // SELECTION
        for(tstep = 0; tstep < zsize; tstep++)
        {
            if(exparray[tstep] < 0.8 * exptmedian)
            {
                frameOKarray[tstep] = 0;
            }
            if(exparray[tstep] > 1.2 * exptmedian)
            {
                frameOKarray[tstep] = 0;
            }
        }

        free(datfile);
    }

    long NBmissingFrame = 0;
    for(tstep = 0; tstep < zsize; tstep++)
        if(frameOKarray[tstep] == 0)
        {
            NBmissingFrame++;
        }

    snprintf(fname, stringmaxlen, "exptime.dat");
    fp = fopen(fname, "w");

    fprintf(fp, "# Exposure time per output frame, unit = input frame\n");
    fprintf(fp, "#\n");
    fprintf(fp,
            "# Generated by function %s in file %s\n",
            __FUNCTION__,
            __FILE__);
    fprintf(fp, "# stream0 : %s\n", stream0);
    fprintf(fp, "# stream1 : %s\n", stream1);
    fprintf(fp, "# tstart  : %f\n", tstart);
    fprintf(fp, "# tend    : %f\n", tend);
    fprintf(fp, "# dt      : %f\n", dt);
    fprintf(fp, "#\n");
    fprintf(fp,
            "# stream0 median exp time : %6.3f frame -> %8.3f Hz\n",
            medianexptimearray[0],
            medianexptimearray[0] / dt);
    fprintf(fp,
            "# stream1 median exp time : %6.3f frame -> %8.3f Hz\n",
            medianexptimearray[1],
            medianexptimearray[1] / dt);
    fprintf(fp,
            "# missing frames : %6ld / %ld  ( %10.6f %%)\n",
            NBmissingFrame,
            (long) zsize,
            100.0 * NBmissingFrame / zsize);
    fprintf(fp, "#\n");
    fprintf(fp, "# col 1 :   time step\n");
    fprintf(fp, "# col 2 :   output frame index (valid if OK flag = 1)\n");
    fprintf(fp, "# col 3 :   OK flag\n");
    fprintf(fp, "# col 4 :   time (stream0)\n");
    fprintf(fp, "# col 5 :   stream0 exposure time\n");
    fprintf(fp, "# col 6 :   stream1 exposure time\n");
    fprintf(fp, "#\n");

    long NBframeOK = 0;
    for(tstep = 0; tstep < zsize; tstep++)
    {
        fprintf(fp,
                "%5ld %5ld %d %10.6f %10.6f %10.6f\n",
                tstep,
                NBframeOK,
                frameOKarray[tstep],
                tstartarray[tstep],
                exparray0[tstep],
                exparray1[tstep]);
        if(frameOKarray[tstep] == 1)
        {
            if(tstep != NBframeOK)
            {
                void *ptr0;
                void *ptr1;

                ptr0 = (char *) data.image[IDout0].array.F +
                       sizeof(float) * xysize0 * tstep;
                ptr1 = (char *) data.image[IDout0].array.F +
                       sizeof(float) * xysize0 * NBframeOK;
                memcpy((void *) ptr1, (void *) ptr0, sizeof(float) * xysize0);

                ptr0 = (char *) data.image[IDout1].array.F +
                       sizeof(float) * xysize1 * tstep;
                ptr1 = (char *) data.image[IDout1].array.F +
                       sizeof(float) * xysize1 * NBframeOK;
                memcpy((void *) ptr1, (void *) ptr0, sizeof(float) * xysize1);
            }
            NBframeOK++;
        }
    }
    fclose(fp);

    free(tstartarray);
    free(tendarray);
    free(exparray);
    free(exparray0);
    free(exparray1);
    free(frameOKarray);

    if(NBframeOK > 0)
    {
        imageID IDoutc0;
        create_3Dimage_ID("outC0", xsize0, ysize0, NBframeOK, &IDoutc0);

        memcpy(data.image[IDoutc0].array.F,
               data.image[IDout0].array.F,
               sizeof(float) * xysize0 * NBframeOK);

        imageID IDoutc1;
        create_3Dimage_ID("outC1", xsize1, ysize1, NBframeOK, &IDoutc1);

        memcpy(data.image[IDoutc1].array.F,
               data.image[IDout1].array.F,
               sizeof(float) * xysize1 * NBframeOK);
    }
    delete_image_ID("out0", DELETE_IMAGE_ERRMODE_WARNING);
    delete_image_ID("out1", DELETE_IMAGE_ERRMODE_WARNING);

    printf("function %s end\n", __FUNCTION__);
    list_image_ID();

    return RETURN_SUCCESS;
}

/**
 * # Purpose
 *
 * Compute similarity matrix between frames of a datacube
 *
 */

errno_t AOloopControl_perfTest_ComputeSimilarityMatrix(char *IDname,
        char *IDname_out)
{
    long     ID, IDout;
    uint32_t xsize, ysize, xysize, zsize;
    long     k1, k2;
    long     cnt = 0;
    float   *array1;
    float   *array2;
    char    *srcptr;

    int perccomplete;
    int perccompletelast = 0;

    ID     = image_ID(IDname);
    xsize  = data.image[ID].md[0].size[0];
    ysize  = data.image[ID].md[0].size[1];
    xysize = xsize * ysize;
    zsize  = data.image[ID].md[0].size[2];

    array1 = (float *) malloc(sizeof(float) * xysize);
    if(array1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    array2 = (float *) malloc(sizeof(float) * xysize);
    if(array2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    create_2Dimage_ID(IDname_out, zsize, zsize, &IDout);
    printf("\n");
    for(k1 = 0; k1 < zsize; k1++)
    {
        srcptr = (char *) data.image[ID].array.F + sizeof(float) * xysize * k1;

        memcpy(array1, srcptr, sizeof(float) * xysize);

        for(k2 = 0; k2 < k1; k2++)
        {
            double val = 0.0;
            long   ii;
            double v0;

            srcptr =
                (char *) data.image[ID].array.F + sizeof(float) * xysize * k2;
            memcpy(array2, srcptr, sizeof(float) * xysize);

            perccomplete = (int)(100.0 * cnt / (zsize * (zsize - 1) / 2));
            if(perccompletelast < perccomplete)
            {
                printf("\r [%5.2f %%]   %5ld / %5ld    %5ld / %5ld     ",
                       100.0 * cnt / (zsize * (zsize - 1) / 2),
                       k1,
                       (long) zsize,
                       k2,
                       (long) zsize);
                fflush(stdout);
                perccompletelast = perccomplete;
            }

            for(ii = 0; ii < xysize; ii++)
            {
                v0 = (array1[ii] - array2[ii]);
                val += v0 * v0;
            }

            data.image[IDout].array.F[k1 * zsize + k2] = val;
            cnt++;
        }
    }
    printf("\n");

    free(array1);
    free(array2);

    return RETURN_SUCCESS;
}

/**
 * # Purpose
 *
 * Perform statistical analysis of two streams from similarity matrices
 *
 * # Details
 *
 * Selects the NBselected most similar pairs in stream0 and stream1 separated by
 * at least dtmin frames
 *
 * Computes the differences between the corresponding pairs in the other stream
 *
 * # Output
 *
 * sim0pairs.txt  : best NBselected stream0 pairs\n
 * sim1pairs.txt  : best NBselected stream1 pairs\n
 * sim2Ddistrib   : 2D similarity distribution image\n
 *
 * sim0diff0      : best sim pairs 0, differences stream 0 images\n
 * sim0diff1      : best sim pairs 0, differences stream 1 images\n
 * sim1diff0      : best sim pairs 1, differences stream 0 images\n
 * sim1diff1      : best sim pairs 1, differences stream 1 images\n
 *
 */

errno_t AOloopControl_perfTest_StatAnalysis_2streams(char *IDname_stream0,
        char *IDname_stream1,
        char *IDname_simM0,
        char *IDname_simM1,
        long  dtmin,
        unsigned long NBselected)
{
    imageID IDstream0;
    imageID IDstream1;
    imageID IDsimM0;
    imageID IDsimM1;

    unsigned long long NBpairMax;

    // similarity pairs extracted from stream0
    unsigned long *sim0pair_k1;
    unsigned long *sim0pair_k2;
    double        *sim0pair_val;

    // similarity pairs extracted from stream1
    unsigned long *sim1pair_k1;
    unsigned long *sim1pair_k2;
    double        *sim1pair_val;

    uint32_t           xsize0, ysize0;
    unsigned long      NBframe0, xysize0;
    uint32_t           xsize1, ysize1;
    unsigned long      NBframe1, xysize1;
    unsigned long      k1, k2;
    unsigned long long paircnt;

    // ouput
    unsigned long pair;
    FILE         *fpout0;
    FILE         *fpout1;

    double mediansim0, mediansim1;

    IDstream0 = image_ID(IDname_stream0);
    xsize0    = data.image[IDstream0].md[0].size[0];
    ysize0    = data.image[IDstream0].md[0].size[1];
    xysize0   = xsize0 * ysize0;
    NBframe0  = data.image[IDstream0].md[0].size[2];

    NBpairMax =
        (unsigned long long) NBframe0; // data.image[IDsimM0].md[0].size[0];
    NBpairMax *= (unsigned long long)(NBframe0 - 1) / 2;
    printf("NBpairMax = %llu x %llu =  %llu\n",
           (unsigned long long) NBframe0,
           (unsigned long long)(NBframe0 - 1) / 2,
           NBpairMax);

    IDstream1 = image_ID(IDname_stream1);
    xsize1    = data.image[IDstream1].md[0].size[0];
    ysize1    = data.image[IDstream1].md[0].size[1];
    xysize1   = xsize1 * ysize1;
    NBframe1  = data.image[IDstream1].md[0].size[2];

    IDsimM0 = image_ID(IDname_simM0);
    IDsimM1 = image_ID(IDname_simM1);

    // a few checks before proceeding
    if(NBframe0 != NBframe1)
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != NBframe1 (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               NBframe1);
        exit(0);
    }

    if(NBframe0 != data.image[IDsimM0].md[0].size[0])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != simM0 xsize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               (long) data.image[IDsimM0].md[0].size[0]);
        exit(0);
    }

    if(NBframe0 != data.image[IDsimM0].md[0].size[1])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != simM0 ysize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               (long) data.image[IDsimM0].md[0].size[1]);
        exit(0);
    }

    if(NBframe1 != data.image[IDsimM1].md[0].size[0])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe1 (%ld) != simM1 xsize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe1,
               (long) data.image[IDsimM1].md[0].size[0]);
        exit(0);
    }

    if(NBframe1 != data.image[IDsimM1].md[0].size[1])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe1 (%ld) != simM1 ysize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe1,
               (long) data.image[IDsimM1].md[0].size[1]);
        exit(0);
    }

    sim0pair_k1 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim0pair_k1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim0pair_k2 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim0pair_k2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim0pair_val = (double *) malloc(sizeof(double) * NBpairMax);
    if(sim0pair_val == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_k1 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim1pair_k1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_k2 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim1pair_k2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_val = (double *) malloc(sizeof(double) * NBpairMax);
    if(sim1pair_val == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    paircnt = 0;
    for(k1 = 0; k1 < NBframe0; k1++)
    {
        for(k2 = 0; k2 < k1; k2++)
        {
            if(((long) k1 - (long) k2) > dtmin)
            {
                if(paircnt > NBpairMax - 1)
                {
                    printf(
                        "[%s] [%s] [%d]  ERROR: paircnt (%llu) >= NBpairMax "
                        "(%llu)\n",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__,
                        paircnt,
                        NBpairMax);
                    printf("NBframe0 = %ld\n", NBframe0);

                    exit(0);
                }

                sim0pair_k1[paircnt] = k1;
                sim0pair_k2[paircnt] = k2;
                sim0pair_val[paircnt] =
                    data.image[IDsimM0].array.F[k1 * NBframe0 + k2];

                sim1pair_k1[paircnt] = k1;
                sim1pair_k2[paircnt] = k2;
                sim1pair_val[paircnt] =
                    data.image[IDsimM1].array.F[k1 * NBframe1 + k2];

                paircnt++;
            }
        }
    }

    printf("Running quicksort sim0 (%llu elements) ... ", paircnt);
    fflush(stdout);
    quick_sort3ulul_double(sim0pair_val,
                           sim0pair_k1,
                           sim0pair_k2,
                           (long) paircnt);
    printf("Done\n");
    fflush(stdout);

    printf("Running quicksort sim1 (%llu elements) ... ", paircnt);
    fflush(stdout);
    quick_sort3ulul_double(sim1pair_val,
                           sim1pair_k1,
                           sim1pair_k2,
                           (long) paircnt);
    printf("Done\n");
    fflush(stdout);

    mediansim0 = sim0pair_val[paircnt / 2];
    mediansim1 = sim1pair_val[paircnt / 2];

    if((fpout0 = fopen("sim0pairs.txt", "w")) == NULL)
    {
        printf("[%s] [%s] [%d]  ERROR: cannot create file\n",
               __FILE__,
               __FUNCTION__,
               __LINE__);
        exit(0);
    }
    for(pair = 0; pair < NBselected; pair++)
    {
        k1 = sim0pair_k1[pair];
        k2 = sim0pair_k2[pair];
        fprintf(fpout0,
                "%5ld  %5ld  %5ld  %8.6f  %8.6f\n",
                pair,
                k1,
                k2,
                data.image[IDsimM0].array.F[k1 * NBframe0 + k2] / mediansim0,
                data.image[IDsimM1].array.F[k1 * NBframe0 + k2] / mediansim1);
    }
    fclose(fpout0);

    if((fpout1 = fopen("sim1pairs.txt", "w")) == NULL)
    {
        printf("[%s] [%s] [%d]  ERROR: cannot create file\n",
               __FILE__,
               __FUNCTION__,
               __LINE__);
        exit(0);
    }
    for(pair = 0; pair < NBselected; pair++)
    {
        k1 = sim1pair_k1[pair];
        k2 = sim1pair_k2[pair];
        fprintf(fpout1,
                "%5ld  %5ld  %5ld  %8.6f  %8.6f\n",
                pair,
                k1,
                k2,
                data.image[IDsimM0].array.F[k1 * NBframe0 + k2] / mediansim0,
                data.image[IDsimM1].array.F[k1 * NBframe1 + k2] / mediansim1);
    }
    fclose(fpout1);

    // Create 2D distribution of similarities

    uint32_t xsize2Ddistrib = 512;
    uint32_t ysize2Ddistrib = 512;

    imageID IDsim2Ddistrib;
    create_2Dimage_ID("sim2Ddistrib",
                      xsize2Ddistrib,
                      ysize2Ddistrib,
                      &IDsim2Ddistrib);

    for(k1 = 0; k1 < NBframe0; k1++)
        for(k2 = 0; k2 < k1; k2++)
        {
            if(((int) k1 - (int) k2) > dtmin)
            {
                float         x, y;
                unsigned long ii, jj;

                x = data.image[IDsimM0].array.F[k1 * NBframe0 + k2] /
                    mediansim0;
                y = data.image[IDsimM1].array.F[k1 * NBframe1 + k2] /
                    mediansim1;

                ii = (uint32_t)(0.5 * x * xsize2Ddistrib);
                jj = (uint32_t)(0.5 * y * ysize2Ddistrib);

                if((ii < xsize2Ddistrib) && (jj < ysize2Ddistrib))
                {
                    data.image[IDsim2Ddistrib]
                    .array.F[jj * xsize2Ddistrib + ii] += 1.0;
                }
            }
        }

    imageID IDsim0diff0;
    create_3Dimage_ID("sim0diff0", xsize0, ysize0, NBselected, &IDsim0diff0);

    imageID IDsim0diff1;
    create_3Dimage_ID("sim0diff1", xsize1, ysize1, NBselected, &IDsim0diff1);

    imageID IDsim0pair0;
    create_3Dimage_ID("sim0pair0",
                      xsize0 * 3,
                      ysize0,
                      NBselected,
                      &IDsim0pair0);
    imageID IDsim0pair1;
    create_3Dimage_ID("sim0pair1",
                      xsize1 * 3,
                      ysize1,
                      NBselected,
                      &IDsim0pair1);

    for(pair = 0; pair < NBselected; pair++)
    {
        unsigned long ii, jj;

        k1 = sim0pair_k1[pair];
        k2 = sim0pair_k2[pair];

        for(ii = 0; ii < xysize0; ii++)
        {
            data.image[IDsim0diff0].array.F[pair * xysize0 + ii] =
                data.image[IDstream0].array.F[k1 * xysize0 + ii] -
                data.image[IDstream0].array.F[k2 * xysize0 + ii];
        }
        for(ii = 0; ii < xysize1; ii++)
        {
            data.image[IDsim0diff1].array.F[pair * xysize1 + ii] =
                data.image[IDstream1].array.F[k1 * xysize1 + ii] -
                data.image[IDstream1].array.F[k2 * xysize1 + ii];
        }

        for(ii = 0; ii < xsize0; ii++)
            for(jj = 0; jj < ysize0; jj++)
            {
                data.image[IDsim0pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii] =
                                                    data.image[IDstream0]
                                                    .array.F[k1 * xysize0 + jj * xsize0 + ii];
                data.image[IDsim0pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii + xsize0] =
                                                    data.image[IDstream0]
                                                    .array.F[k2 * xysize0 + jj * xsize0 + ii];
                data.image[IDsim0pair0]
                .array.F[pair * ysize0 * xsize0 * 3 + jj * xsize0 * 3 + ii +
                              xsize0 * 2] =
                             data.image[IDstream0]
                             .array.F[k1 * xysize0 + jj * xsize0 + ii] -
                             data.image[IDstream0]
                             .array.F[k2 * xysize0 + jj * xsize0 + ii];
            }

        for(ii = 0; ii < xsize1; ii++)
            for(jj = 0; jj < ysize1; jj++)
            {
                data.image[IDsim0pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii] =
                                                    data.image[IDstream1]
                                                    .array.F[k1 * xysize1 + jj * xsize1 + ii];
                data.image[IDsim0pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii + xsize1] =
                                                    data.image[IDstream1]
                                                    .array.F[k2 * xysize1 + jj * xsize1 + ii];
                data.image[IDsim0pair1]
                .array.F[pair * ysize1 * xsize1 * 3 + jj * xsize1 * 3 + ii +
                              xsize1 * 2] =
                             data.image[IDstream1]
                             .array.F[k1 * xysize1 + jj * xsize1 + ii] -
                             data.image[IDstream1]
                             .array.F[k2 * xysize1 + jj * xsize1 + ii];
            }
    }

    imageID IDsim1diff0;
    create_3Dimage_ID("sim1diff0", xsize0, ysize0, NBselected, &IDsim1diff0);

    imageID IDsim1diff1;
    create_3Dimage_ID("sim1diff1", xsize1, ysize1, NBselected, &IDsim1diff1);

    imageID IDsim1pair0;
    create_3Dimage_ID("sim1pair0",
                      xsize0 * 3,
                      ysize0,
                      NBselected,
                      &IDsim1pair0);
    imageID IDsim1pair1;
    create_3Dimage_ID("sim1pair1",
                      xsize1 * 3,
                      ysize1,
                      NBselected,
                      &IDsim1pair1);

    for(pair = 0; pair < NBselected; pair++)
    {
        unsigned long ii, jj;

        k1 = sim1pair_k1[pair];
        k2 = sim1pair_k2[pair];

        for(ii = 0; ii < xysize0; ii++)
        {
            data.image[IDsim1diff0].array.F[pair * xysize0 + ii] =
                data.image[IDstream0].array.F[k1 * xysize0 + ii] -
                data.image[IDstream0].array.F[k2 * xysize0 + ii];
        }
        for(ii = 0; ii < xysize1; ii++)
        {
            data.image[IDsim1diff1].array.F[pair * xysize1 + ii] =
                data.image[IDstream1].array.F[k1 * xysize1 + ii] -
                data.image[IDstream1].array.F[k2 * xysize1 + ii];
        }

        for(ii = 0; ii < xsize0; ii++)
            for(jj = 0; jj < ysize0; jj++)
            {
                data.image[IDsim1pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii] =
                                                    data.image[IDstream0]
                                                    .array.F[k1 * xysize0 + jj * xsize0 + ii];
                data.image[IDsim1pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii + xsize0] =
                                                    data.image[IDstream0]
                                                    .array.F[k2 * xysize0 + jj * xsize0 + ii];
                data.image[IDsim1pair0]
                .array.F[pair * ysize0 * xsize0 * 3 + jj * xsize0 * 3 + ii +
                              xsize0 * 2] =
                             data.image[IDstream0]
                             .array.F[k1 * xysize0 + jj * xsize0 + ii] -
                             data.image[IDstream0]
                             .array.F[k2 * xysize0 + jj * xsize0 + ii];
            }

        for(ii = 0; ii < xsize1; ii++)
            for(jj = 0; jj < ysize1; jj++)
            {
                data.image[IDsim1pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii] =
                                                    data.image[IDstream1]
                                                    .array.F[k1 * xysize1 + jj * xsize1 + ii];
                data.image[IDsim1pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii + xsize1] =
                                                    data.image[IDstream1]
                                                    .array.F[k2 * xysize1 + jj * xsize1 + ii];
                data.image[IDsim1pair1]
                .array.F[pair * ysize1 * xsize1 * 3 + jj * xsize1 * 3 + ii +
                              xsize1 * 2] =
                             data.image[IDstream1]
                             .array.F[k1 * xysize1 + jj * xsize1 + ii] -
                             data.image[IDstream1]
                             .array.F[k2 * xysize1 + jj * xsize1 + ii];
            }
    }

    free(sim0pair_k1);
    free(sim0pair_k2);
    free(sim0pair_val);

    free(sim1pair_k1);
    free(sim1pair_k2);
    free(sim1pair_val);

    return RETURN_SUCCESS;
}

/**
 *
 * PSF evaluation window is (x0,y0) to (x1,y1)
 *
 * Optional input: PSFmask, to be multiplied by PSF
 *
 *
 * EvalMode = 0  : Maximize Energy concentration
 * EvalMode = 1  : Maximize flux
 * EvalMode = 2  : Minimize flux
 *
 * output:
 *
 * imwfsbest
 * imwfsall
 *
 * impsfbest
 * impsfall
 *
 *
 *
 */

errno_t AOloopControl_perfTest_SelectWFSframes_from_PSFframes(char *IDnameWFS,
        char *IDnamePSF,
        float frac,
        long  x0,
        long  x1,
        long  y0,
        long  y1,
        int   EvalMode,
        float alpha)
{
    imageID IDwfs;
    imageID IDpsf;
    imageID IDpsfmask; // optional

    long NBframe;
    long xsizewfs, ysizewfs, xysizewfs;
    long xsizepsf, ysizepsf, xysizepsf;

    double *evalarray;
    long   *indexarray;

    IDwfs = image_ID(IDnameWFS);
    IDpsf = image_ID(IDnamePSF);

    xsizewfs  = data.image[IDwfs].md[0].size[0];
    ysizewfs  = data.image[IDwfs].md[0].size[1];
    xysizewfs = xsizewfs * ysizewfs;

    xsizepsf  = data.image[IDpsf].md[0].size[0];
    ysizepsf  = data.image[IDpsf].md[0].size[1];
    xysizepsf = xsizepsf * ysizepsf;

    NBframe = data.image[IDwfs].md[0].size[2];

    evalarray = (double *) malloc(sizeof(double) * NBframe);
    if(evalarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    indexarray = (long *) malloc(sizeof(long) * NBframe);
    if(indexarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    long x0t, y0t, x1t, y1t;
    x0t = x0;
    x1t = x1;
    y0t = y0;
    y1t = y1;

    if(x0 < 0)
    {
        x0t = x0;
    }
    if(x1 > xsizepsf - 1)
    {
        x1t = xsizepsf - 1;
    }
    if(y0 < 0)
    {
        y0t = y0;
    }
    if(y1 > ysizepsf - 1)
    {
        y1t = ysizepsf - 1;
    }

    printf("WINDOW: %ld - %ld     %ld -%ld\n", x0t, x1t, y0t, y1t);

    long kk;
    IDpsfmask = image_ID("PSFmask");
    if(IDpsfmask != -1)
    {
        for(kk = 0; kk < NBframe; kk++)
        {
            long ii, jj;

            for(ii = x0t; ii < x1t; ii++)
                for(jj = y0t; jj < y1t; jj++)
                {
                    data.image[IDpsf]
                    .array.F[kk * xysizepsf + jj * xsizepsf + ii] *=
                        data.image[IDpsfmask].array.F[jj * xsizepsf + ii];
                }
        }
    }

    for(kk = 0; kk < NBframe; kk++)
    {
        long   ii, jj;
        double sum  = 0.0;
        double ssum = 0.0;

        indexarray[kk] = kk;

        for(ii = x0t; ii < x1t; ii++)
            for(jj = y0t; jj < y1t; jj++)
            {
                float tval;
                tval = data.image[IDpsf]
                       .array.F[kk * xysizepsf + jj * xsizepsf + ii];
                if(tval < 0.0)
                {
                    tval = 0.0;
                }
                sum += tval;
                ssum += pow(tval, alpha);
            }

        // best frame
        switch(EvalMode)
        {
        case 0:
            evalarray[kk] = -(ssum / (pow(sum, alpha)));
            break;

        case 1:
            evalarray[kk] = -sum;
            break;

        case 2:
            evalarray[kk] = sum;
            break;

        default:
            evalarray[kk] = -sum;
            break;
        }
    }

    quick_sort2l(evalarray, indexarray, NBframe);

    long IDwfsbest, IDwfsall;
    long IDpsfbest, IDpsfall;

    create_2Dimage_ID("imwfsbest", xsizewfs, ysizewfs, &IDwfsbest);
    create_2Dimage_ID("imwfsall", xsizewfs, ysizewfs, &IDwfsall);

    create_2Dimage_ID("impsfbest", xsizepsf, ysizepsf, &IDpsfbest);
    create_2Dimage_ID("impsfall", xsizepsf, ysizepsf, &IDpsfall);

    long kklim;
    kklim = (long)(frac * NBframe);

    printf("kklim = %ld     %ld %ld\n", kklim, xysizewfs, xysizepsf);

    FILE *fp = fopen("fptest.txt", "w");
    for(kk = 0; kk < NBframe; kk++)
    {
        long ii;

        fprintf(fp, "%6ld  %6ld  %g\n", kk, indexarray[kk], evalarray[kk]);

        if(kk < kklim)
        {
            for(ii = 0; ii < xysizewfs; ii++)
            {
                data.image[IDwfsbest].array.F[ii] +=
                    data.image[IDwfs].array.F[indexarray[kk] * xysizewfs + ii];
            }

            for(ii = 0; ii < xysizepsf; ii++)
            {
                data.image[IDpsfbest].array.F[ii] +=
                    data.image[IDpsf].array.F[indexarray[kk] * xysizepsf + ii];
            }
        }

        for(ii = 0; ii < xysizewfs; ii++)
        {
            data.image[IDwfsall].array.F[ii] +=
                data.image[IDwfs].array.F[kk * xysizewfs + ii];
        }

        for(ii = 0; ii < xysizepsf; ii++)
        {
            data.image[IDpsfall].array.F[ii] +=
                data.image[IDpsf].array.F[kk * xysizepsf + ii];
        }
    }
    fclose(fp);

    long ii;

    for(ii = 0; ii < xysizewfs; ii++)
    {
        data.image[IDwfsbest].array.F[ii] /= kklim;
    }

    for(ii = 0; ii < xysizepsf; ii++)
    {
        data.image[IDpsfbest].array.F[ii] /= kklim;
    }

    for(ii = 0; ii < xysizewfs; ii++)
    {
        data.image[IDwfsall].array.F[ii] /= NBframe;
    }

    for(ii = 0; ii < xysizepsf; ii++)
    {
        data.image[IDpsfall].array.F[ii] /= NBframe;
    }

    free(evalarray);
    free(indexarray);

    return RETURN_SUCCESS;
}
