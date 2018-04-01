/**
 * @file    AOloopControl_computeCalib_loDMmodes.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    26 Dec 2017
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
/*
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
*/


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "linopt_imtools/linopt_imtools.h"
#include "image_filter/image_filter.h"
#include "ZernikePolyn/ZernikePolyn.h"
/*
#include "00CORE/00CORE.h"

#include "COREMOD_tools/COREMOD_tools.h"
#include "info/info.h"
#include "statistic/statistic.h"
#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"

*/
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



// make low order DM modes
long AOloopControl_computeCalib_mkloDMmodes(const char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yc, double r0, double r1, int MaskMode)
{
    long IDmask;
    long ID, ID0, IDtm, IDem, IDtmpg, IDslaved;
    long ii, jj;
    double x, y, r, PA, xc1, yc1, totm, offset, rms, sigma;

    long NBZ, m;
    long zindex[10];
    double zcpa[10];  /// CPA for each Zernike (somewhat arbitrary... used to sort modes in CPA)
    long IDfreq, IDmfcpa;
    long k;

    double coeff;
    long kelim = 20;
    long conviter;




    zindex[0] = 1; // tip
    zcpa[0] = 0.0;

    zindex[1] = 2; // tilt
    zcpa[1] = 0.0;

    zindex[2] = 4; // focus
    zcpa[2] = 0.25;

    zindex[3] = 3; // astig
    zcpa[3] = 0.4;

    zindex[4] = 5; // astig
    zcpa[4] = 0.4;

    zindex[5] = 7; // coma
    zcpa[5] = 0.6;

    zindex[6] = 8; // coma
    zcpa[6] = 0.6;

    zindex[7] = 6; // trefoil
    zcpa[7] = 1.0;

    zindex[8] = 9; // trefoil
    zcpa[8] = 1.0;

    zindex[9] = 12;
    zcpa[9] = 1.5;


    printf("msizexy = %ld %ld\n", msizex, msizey);
    list_image_ID();
    IDmask = image_ID("dmmask");
    if(IDmask==-1)
    {
        double val0, val1;
        double a0=0.88;
        double b0=40.0;
        double a1=1.2;
        double b1=12.0;

        IDmask = create_2Dimage_ID("dmmask", msizex, msizey);
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                x = 1.0*ii-xc;
                y = 1.0*jj-yc;
                r = sqrt(x*x+y*y)/r1;
                val1 = 1.0-exp(-pow(a1*r,b1));
                r = sqrt(x*x+y*y)/r0;
                val0 = exp(-pow(a0*r,b0));
                data.image[IDmask].array.F[jj*msizex+ii] = val0*val1;
            }
        save_fits("dmmask", "!dmmask.fits");
        xc1 = xc;
        yc1 = yc;
    }
    else /// extract xc and yc from mask
    {
        xc1 = 0.0;
        yc1 = 0.0;
        totm = 0.0;
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                xc1 += 1.0*ii*data.image[IDmask].array.F[jj*msizex+ii];
                yc1 += 1.0*jj*data.image[IDmask].array.F[jj*msizex+ii];
                totm += data.image[IDmask].array.F[jj*msizex+ii];
            }
        // printf("xc1 yc1    %f  %f     %f\n", xc1, yc1, totm);
        xc1 /= totm;
        yc1 /= totm;
    }




    totm = arith_image_total("dmmask");
    if((msizex != data.image[IDmask].md[0].size[0])||(msizey != data.image[IDmask].md[0].size[1]))
    {
        printf("ERROR: file dmmask size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmask].md[0].size[0], (long) data.image[IDmask].md[0].size[1], (long) msizex, (long) msizey);
        exit(0);
    }


    NBZ = 0;

    for(m=0; m<10; m++)
    {
        if(zcpa[m]<CPAmax)
            NBZ++;
    }



    linopt_imtools_makeCPAmodes("CPAmodes", msizex, CPAmax, deltaCPA, 0.5*msizex, 1.2, 0);
    ID0 = image_ID("CPAmodes");
    IDfreq = image_ID("cpamodesfreq");



    printf("  %ld %ld %ld\n", msizex, msizey, (long) data.image[ID0].md[0].size[2]-1 );
    ID = create_3Dimage_ID(ID_name, msizex, msizey, data.image[ID0].md[0].size[2]-1+NBZ);





    IDmfcpa = create_2Dimage_ID("modesfreqcpa", data.image[ID0].md[0].size[2]-1+NBZ, 1);

    /*** Create TTF first */
    zernike_init();
    printf("r1 = %f    %f %f\n", r1, xc1, yc1);
    for(k=0; k<NBZ; k++)
    {
        data.image[IDmfcpa].array.F[k] = zcpa[k];
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                x = 1.0*ii-xc1;
                y = 1.0*jj-yc1;
                r = sqrt(x*x+y*y)/r1;
                PA = atan2(y,x);
                data.image[ID].array.F[k*msizex*msizey+jj*msizex+ii] = Zernike_value(zindex[k], r, PA);
            }
    }




    for(k=0; k<data.image[ID0].md[0].size[2]-1; k++)
    {
        data.image[IDmfcpa].array.F[k+NBZ] = data.image[IDfreq].array.F[k+1];
        for(ii=0; ii<msizex*msizey; ii++)
            data.image[ID].array.F[(k+NBZ)*msizex*msizey+ii] = data.image[ID0].array.F[(k+1)*msizex*msizey+ii];
    }


    for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
    {
        /// Remove excluded modes
        long IDeModes = image_ID("emodes");
        if(IDeModes!=-1)
        {
            IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

            for(ii=0; ii<msizex*msizey; ii++)
                data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
            linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmask", 1.0e-2, "lcoeff", 0);
            linopt_imtools_image_construct("emodes", "lcoeff", "em00");
            delete_image_ID("lcoeff");
            IDem = image_ID("em00");

            coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));
            if(k>2.0*kelim)
                coeff = 1.0;
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

            delete_image_ID("em00");
            delete_image_ID("tmpmode");
        }


        double totvm = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            //	  data.image[ID].array.F[k*msize*msize+ii] = data.image[ID0].array.F[(k+1)*msize*msize+ii];
            totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
        }
        offset = totvm/totm;

        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset;
            data.image[ID].array.F[k*msizex*msizey+ii] *= data.image[IDmask].array.F[ii];
        }

        offset = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
            offset += data.image[ID].array.F[k*msizex*msizey+ii];

        rms = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset/msizex/msizey;
            rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii];
        }
        rms = sqrt(rms/totm);
        printf("Mode %ld   RMS = %lf  (%f)\n", k, rms, totm);
        for(ii=0; ii<msizex*msizey; ii++)
            data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
    }


    for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
    {
        rms = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset/msizex/msizey;
            rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii];
        }
        rms = sqrt(rms/totm);
        printf("Mode %ld   RMS = %lf\n", k, rms);
    }



    if(MaskMode==1)
    {
        long kernsize = 5;
        long NBciter = 200;
        long citer;

        if(2*kernsize>msizex)
            kernsize = msizex/2;
        for(citer=0; citer<NBciter; citer++)
        {
            long IDg;

            printf("Convolution [%3ld/%3ld]\n", citer, NBciter);
            gauss_filter(ID_name, "modeg", 4.0*pow(1.0*(NBciter-citer)/NBciter,0.5), kernsize);
            IDg = image_ID("modeg");
            for(k=0; k<data.image[ID].md[0].size[2]; k++)
            {
                for(ii=0; ii<msizex*msizey; ii++)
                    if(data.image[IDmask].array.F[ii]<0.98)
                        data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDg].array.F[k*msizex*msizey+ii];
            }
            delete_image_ID("modeg");
        }
    }




    /// SLAVED ACTUATORS
    IDslaved = image_ID("dmslaved");
    ID = image_ID(ID_name);
    if((IDslaved != -1)&&(IDmask!=-1))
    {
        long IDtmp = create_2Dimage_ID("_tmpinterpol", msizex, msizey);
        long IDtmp1 = create_2Dimage_ID("_tmpcoeff1", msizex, msizey);
        long IDtmp2 = create_2Dimage_ID("_tmpcoeff2", msizex, msizey);
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            // write input DM mode
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];
                data.image[IDtmp1].array.F[ii] = data.image[IDmask].array.F[ii] * (1.0-data.image[IDslaved].array.F[ii]);
                data.image[IDtmp2].array.F[ii] = data.image[IDtmp1].array.F[ii];
            }

            long pixcnt = 1;
            float vxp, vxm, vyp, vym, cxp, cxm, cyp, cym;
            float ctot;

            while(pixcnt>0)
            {
                pixcnt = 0;
                for(ii=1; ii<msizex-1; ii++)
                    for(jj=1; jj<msizey-1; jj++)
                    {
                        if((data.image[IDtmp1].array.F[jj*msizex+ii]<0.5) && (data.image[IDslaved].array.F[jj*msizex+ii]>0.5))
                        {
                            pixcnt ++;
                            vxp = data.image[IDtmp].array.F[jj*msizex+(ii+1)];
                            cxp = data.image[IDtmp1].array.F[jj*msizex+(ii+1)];

                            vxm = data.image[IDtmp].array.F[jj*msizex+(ii-1)];
                            cxm = data.image[IDtmp1].array.F[jj*msizex+(ii-1)];

                            vyp = data.image[IDtmp].array.F[(jj+1)*msizex+ii];
                            cyp = data.image[IDtmp1].array.F[(jj+1)*msizex+ii];

                            vym = data.image[IDtmp].array.F[(jj-1)*msizex+ii];
                            cym = data.image[IDtmp1].array.F[(jj-1)*msizex+ii];

                            ctot = (cxp+cxm+cyp+cym);

                            if(ctot>0.5)
                            {
                                data.image[IDtmp].array.F[jj*msizex+ii] = (vxp*cxp+vxm*cxm+vyp*cyp+vym*cym)/ctot;
                                data.image[IDtmp2].array.F[jj*msizex+ii] = 1.0;
                            }
                        }
                    }
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[IDtmp1].array.F[ii] = data.image[IDtmp2].array.F[ii];
            }
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[ii];






            /*

                    IDtmp = create_2Dimage_ID("_tmpinterpol", msizex, msizey);
                    for(m=0; m<data.image[ID].md[0].size[2]; m++)
                    {
                        for(ii=0; ii<msizex*msizey; ii++)
                            data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];

                        for(conviter=0; conviter<NBconviter; conviter++)
                        {
                            sigma = 0.5*NBconviter/(1.0+conviter);
                            gauss_filter("_tmpinterpol", "_tmpinterpolg", 1.0, 2);
                            IDtmpg = image_ID("_tmpinterpolg");
                            for(ii=0; ii<msizex*msizey; ii++)
                            {
                                if((data.image[IDmask].array.F[ii]>0.5)&&(data.image[IDslaved].array.F[ii]<0.5))
                                    data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];
                                else
                                    data.image[IDtmp].array.F[ii] = data.image[IDtmpg].array.F[ii];
                            }
                            delete_image_ID("_tmpinterpolg");
                        }
                        for(ii=0; ii<msizex*msizey; ii++)
                            if(data.image[IDmask].array.F[ii]>0.5)
                                data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[ii];
                    */
        }
        delete_image_ID("_tmpinterpol");
        delete_image_ID("_tmpcoeff1");
        delete_image_ID("_tmpcoeff2");
    }



    return(ID);
}
