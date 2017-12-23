/**
 * @file    AOloopControl_computeCalib_Hadamard.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    26 Aug 2017
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>

#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t) {
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time;
    time = mach_absolute_time();
    double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
    double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
    t->tv_sec = seconds;
    t->tv_nsec = nseconds;
    return 0;
}
#else
#include <time.h>
#endif


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>


#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"
#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"


#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif


/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

#define MAX_MBLOCK 20

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif

extern DATA data;


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



// output:
// Hadamard modes (outname)
// Hadamard matrix ("Hmat.fits")
// pixel indexes ("Hpixindex.fits", float, to be converted to long)
long AOloopControl_computeCalib_mkHadamardModes(const char *DMmask_name, const char *outname)
{
    long IDout;
    long xsize, ysize, xysize;
    //    long IDdisk;
    long cnt;

    long Hsize;
    int n2max;

    long *indexarray;
    long index;
    long IDtest;
    int *Hmat;
    long k, ii, jj, n, n2, i, j;
    long IDindex;
    uint32_t *sizearray;

    long IDmask;



    IDmask = image_ID(DMmask_name);
    xsize = data.image[IDmask].md[0].size[0];
    ysize = data.image[IDmask].md[0].size[1];
    xysize = xsize*ysize;

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize;
    sizearray[1] = ysize;
    IDindex = create_image_ID("Hpixindex", 2, sizearray, _DATATYPE_FLOAT, 0, 0);
    free(sizearray);

    cnt = 0;
    for(ii=0; ii<xysize; ii++)
        if(data.image[IDmask].array.F[ii]>0.5)
            cnt++;

    Hsize = 1;
    n2max = 0;
    while(Hsize<cnt)
    {
        Hsize *= 2;
        n2max++;
    }
    n2max++;

    printf("Hsize n2max = %ld  %d\n", Hsize, n2max);
    fflush(stdout);

    for(ii=0; ii<xysize; ii++)
        data.image[IDindex].array.F[ii] = -10.0;

    index = 0;

    indexarray = (long*) malloc(sizeof(long)*Hsize);
    for(k=0; k<Hsize; k++)
        indexarray[k] = -1;
    for(ii=0; ii<xysize; ii++)
        if((data.image[IDmask].array.F[ii]>0.5)&&(index<Hsize))
        {

            indexarray[index] = ii;
            // printf("(%ld %ld)  ", index, ii);

            data.image[IDindex].array.F[ii] = 1.0*index;

            index++;
        }
    save_fits("Hpixindex", "!Hpixindex.fits.gz");

    Hmat = (int*) malloc(sizeof(int)*Hsize*Hsize);



    // n = 0

    ii = 0;
    jj = 0;
    Hmat[jj*Hsize+ii] = 1;
    n2=1;
    for(n=1; n<n2max; n++)
    {
        for(ii=0; ii<n2; ii++)
            for(jj=0; jj<n2; jj++)
            {
                Hmat[ jj*Hsize + (ii+n2)] = Hmat[ jj*Hsize + ii];
                Hmat[ (jj+n2)*Hsize + (ii+n2)] = -Hmat[ jj*Hsize + ii];
                Hmat[ (jj+n2)*Hsize + ii] = Hmat[ jj*Hsize + ii];
            }
        n2 *= 2;
    }

    printf("n2 = %ld\n", n2);
    fflush(stdout);

    IDtest = create_2Dimage_ID("Htest", Hsize, Hsize);

    for(ii=0; ii<Hsize; ii++)
        for(jj=0; jj<Hsize; jj++)
            data.image[IDtest].array.F[jj*Hsize+ii] = Hmat[jj*Hsize+ii];

    save_fits("Htest", "!Hmat.fits.gz");


    IDout = create_3Dimage_ID(outname, xsize, ysize, Hsize);
    for(k=0; k<Hsize; k++)
    {
        for(index=0; index<Hsize; index++)
        {
            ii = indexarray[index];
            data.image[IDout].array.F[k*xysize+ii] = Hmat[k*Hsize+index];
        }
    }

    free(Hmat);

    free(indexarray);


    return(IDout);
}




long AOloopControl_computeCalib_Hadamard_decodeRM(const char *inname, const char *Hmatname, const char *indexname, const char *outname)
{
    long IDin, IDhad, IDout, IDindex;
    long NBact, NBframes, sizexwfs, sizeywfs, sizewfs;
    long kk, kk1, ii;
    uint32_t zsizeout;



    IDin = image_ID(inname);
    sizexwfs = data.image[IDin].md[0].size[0];
    sizeywfs = data.image[IDin].md[0].size[1];
    sizewfs = sizexwfs*sizeywfs;
    NBframes = data.image[IDin].md[0].size[2];

    IDindex = image_ID(indexname);



    IDhad = image_ID(Hmatname);
    if((data.image[IDhad].md[0].size[0]!=NBframes)||(data.image[IDhad].md[0].size[1]!=NBframes))
    {
        printf("ERROR: size of Hadamard matrix [%ld x %ld] does not match available number of frames [%ld]\n", (long) data.image[IDhad].md[0].size[0], (long) data.image[IDhad].md[0].size[1], NBframes);
        exit(0);
    }

    zsizeout = data.image[IDindex].md[0].size[0]*data.image[IDindex].md[0].size[1];
    IDout = create_3Dimage_ID(outname, sizexwfs, sizeywfs, zsizeout);

    long kk0;
# ifdef _OPENMP
    #pragma omp parallel for private(kk0,kk1,ii)
# endif
    for(kk=0; kk<zsizeout; kk++) // output frame
    {
        kk0 = (long) (data.image[IDindex].array.F[kk]+0.1);
        if(kk0 > -1)
        {   printf("\r  frame %5ld / %5ld     ", kk0, NBframes);
            fflush(stdout);
            for(kk1=0; kk1<NBframes; kk1++)
            {
                for(ii=0; ii<sizewfs; ii++)
                    data.image[IDout].array.F[kk*sizewfs+ii] += data.image[IDin].array.F[kk1*sizewfs+ii]*data.image[IDhad].array.F[kk0*NBframes+kk1];
            }
        }
    }

    for(kk=0; kk<zsizeout; kk++)
    {
        for(ii=0; ii<sizewfs; ii++)
            data.image[IDout].array.F[kk*sizewfs+ii] /= NBframes;

    }

    printf("\n\n");


    return(IDout);
}

