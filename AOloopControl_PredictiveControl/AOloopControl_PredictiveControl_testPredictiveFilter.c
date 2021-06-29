/**
 * @file    AOloopControl_PredictiveControl_testPredictiveFilter.c
 * @brief   Test Predictive Control
 *
 *
 *
 *
 */



#define _GNU_SOURCE



#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"

#include "linopt_imtools/linopt_imtools.h"







///
/// predictive control based on SVD
///
/// input:
///     mode values trace  [ii: time, jj: mode number]
///     mode index
///     delayfr [delay in frame unit]
///     filtsize [number of samples in filter]
///
double AOloopControl_PredictiveControl_testPredictiveFilter(
    const char *IDtrace_name,
    long        modeout,
    double      delayfr,
    long        filtsize,
    const char *IDfilt_name,
    double      SVDeps
)
{
    imageID  IDtrace;
    imageID  IDmatA;
    long     NBtraceVec; // number of measurement vectors in trace
    long     NBmvec; // number of measurements in measurement matrix
    long     NBch; // number of channels in measurement
    imageID  IDmatC;
    imageID  IDfilt;
    long     l,m;
    float   *marray; // measurement array
    FILE    *fp;
    float    tmpv;
    long     delayfr_int;
    float    delayfr_x;
    long     ch, l1;
    double   err0, err1;
    float    v0;
    //float    NoiseAmpl = 0.02;



    IDtrace = image_ID(IDtrace_name);

    NBtraceVec = data.image[IDtrace].md[0].size[0];
    NBch = data.image[IDtrace].md[0].size[1];


    NBmvec = NBtraceVec - filtsize - (long) (delayfr+1.0);




    // build measurement matrix

    fp = fopen("tracepts.txt","w");
    create_2Dimage_ID("WFPmatA", NBmvec, filtsize*NBch, &IDmatA);
    // each column is a measurement
    for(m=0; m<NBmvec; m++) // column index
    {
        fprintf(fp, "%5ld %f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout+m+filtsize]);
        for(l=0; l<filtsize; l++)
            for(ch=0; ch<NBch; ch++)
            {
                l1 = ch*filtsize + l;
                data.image[IDmatA].array.F[l1*NBmvec+m] = data.image[IDtrace].array.F[NBtraceVec*ch + (m+l)];
            }
    }
    fclose(fp);






    // build measurement vector
    delayfr_int = (int) delayfr;
    delayfr_x = delayfr - delayfr_int;
    printf("%f  = %ld + %f\n", delayfr, delayfr_int, delayfr_x);
    marray = (float*) malloc(sizeof(float)*NBmvec);
    if(marray == NULL) {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    fp = fopen("tracepts1.txt","w");
    for(m=0; m<NBmvec; m++)
    {
        marray[m] = data.image[IDtrace].array.F[NBtraceVec*modeout+(m+filtsize+delayfr_int)]*(1.0-delayfr_x) + data.image[IDtrace].array.F[NBtraceVec*modeout+(m+filtsize+delayfr_int+1)]*delayfr_x;
        fprintf(fp, "%5ld %f %f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout+m+filtsize], marray[m]);
    }
    fclose(fp);


    linopt_compute_SVDpseudoInverse("WFPmatA", "WFPmatC", SVDeps, 10000, "WFP_VTmat");

    save_fits("WFPmatA", "!WFPmatA.fits");
    save_fits("WFPmatC", "!WFPmatC.fits");
    IDmatC = image_ID("WFPmatC");

    create_2Dimage_ID(IDfilt_name, filtsize, NBch, &IDfilt);
    for(l=0; l<filtsize; l++)
        for(ch=0; ch<NBch; ch++)
        {
            tmpv = 0.0;
            for(m=0; m<NBmvec; m++)
                tmpv += data.image[IDmatC].array.F[(ch*filtsize+l)*NBmvec+m] * marray[m];
            data.image[IDfilt].array.F[ch*filtsize+l] = tmpv;
        }

    fp = fopen("filt.txt", "w");
    tmpv = 0.0;
    for(l=0; l<filtsize; l++)
        for(ch=0; ch<NBch; ch++)
        {
            tmpv += data.image[IDfilt].array.F[ch*filtsize+l];
            fprintf(fp, "%3ld %3ld %f %f\n", ch, l, data.image[IDfilt].array.F[l], tmpv);
        }
    fclose(fp);
    printf("filter TOTAL = %f\n", tmpv);

    // TEST FILTER

    // col #1 : time index m
    // col #2 : value at index m
    // col #3 : predicted value at m+delay
    // col #4 : actual value at m+delay
    fp = fopen("testfilt.txt", "w");
    err0 = 0.0;
    err1 = 0.0;
    for(m=filtsize; m<NBtraceVec-(delayfr_int+1); m++)
    {
        tmpv = 0.0;
        for(l=0; l<filtsize; l++)
            for(ch=0; ch<NBch; ch++)
                tmpv += data.image[IDfilt].array.F[ch*filtsize+l]*data.image[IDtrace].array.F[NBtraceVec*ch + (m-filtsize+l)];

        fprintf(fp, "%5ld %20f %20f %20f\n", m, data.image[IDtrace].array.F[NBtraceVec*modeout + m], tmpv, marray[m-filtsize]);

        v0 = tmpv - marray[m-filtsize];
        err0 += v0*v0;

        v0 = data.image[IDtrace].array.F[NBtraceVec*modeout + m] - marray[m-filtsize];
        err1 += v0*v0;
    }
    fclose(fp);
    free(marray);

    err0 = sqrt(err0/(NBtraceVec-filtsize-(delayfr_int+1)));
    err1 = sqrt(err1/(NBtraceVec-filtsize-(delayfr_int+1)));
    printf("Prediction error (using optimal filter)   :   %f\n", err0);
    printf("Prediction error (using last measurement) :   %f\n", err1);

    return(err1);
}





