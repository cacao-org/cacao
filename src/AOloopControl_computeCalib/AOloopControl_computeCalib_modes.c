/**
 * @file    AOloopControl_computeCalib_makemodes.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
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



extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c


long aoconfID_imWFS2_active[100];

#define MAX_MBLOCK 20

/*** \brief creates AO control modes
 *
 *
 * creates image "modesfreqcpa" which contains CPA value for each mode
 *
 *
 * if Mmask exists, measure xc, yc from it, otherwise use values given to function
 *
 * INPUT (optional): "dmmaskRM" DM actuators directly controlled
 * INPUT (optional): "dmslaved" force these actuators to be slaved to neighbors
 * OUTPUT :          "dmmask" is the union of "dmmaskRM" and "dmslaved"
 *
 * MaskMode = 0  : tapered masking
 * MaskMode = 1  : STRICT masking
 *
 * if BlockNB < 0 : do all blocks
 * if BlockNB >= 0 : only update single block (untested)
 *
 * SVDlim = 0.05 works well
 *
 * OPTIONAL : if file zrespM exists, WFS modes will be computed
 *
 */

long AOloopControl_computeCalib_mkModes(
    const char *ID_name,
    long msizex,
    long msizey,
    float CPAmax,
    float deltaCPA,
    double xc,
    double yc,
    double r0,
    double r1,
    int MaskMode,
    int BlockNB,
    float SVDlim
)
{
    FILE *fp;
    long ID = -1;
    long ii, jj;

    long IDmaskRM; // DM mask

    double totm;

    double x, y, r, xc1, yc1;
    double rms;

    long IDz;

    long zindex[10];
    double zcpa[10];  /// CPA for each Zernike (somewhat arbitrary... used to sort modes in CPA)

    long mblock, m;
    long NBmblock;

    long MBLOCK_NBmode[MAX_MBLOCK]; // number of blocks
    long MBLOCK_ID[MAX_MBLOCK];
    //    long MBLOCK_IDwfs[MAX_MBLOCK];
    float MBLOCK_CPA[MAX_MBLOCK];


    char *ptr0;
    char *ptr1;

    char imname[200];
    char imname1[200];


    char fname[200];
    char fname1[200];


    float value, value0, value1, value1cnt, valuen;
    long msizexy;
    long m0, mblock0;

    long iter;

    long IDmask;

    long IDzrespM;
    long wfsxsize, wfsysize, wfssize;
    long wfselem, act;

    long IDout, ID1, ID2, ID2b;
    long zsize1, zsize2;
    long z1, z2;
    long xysize1, xysize2;



    float SVDlim00;// DM filtering step 0
    float SVDlim01; // DM filtering step 1


    float rmslim1 = 0.1;
    long IDm;


    int *mok;
    long NBmm = 2000; // max number of modes per block
    long cnt;


    int reuse;
    long IDSVDmodein, IDSVDmode1, IDSVDcoeff, IDSVDmask;
    long m1;
    long IDnewmodeC;

    long IDtmp, IDtmpg;
    long conviter;
    float sigma;

    int MODAL; // 1 if "pixels" of DM are already modes

    long IDRMMmodes = -1;
    long IDRMMresp  = -1;
    long ID_imfit = -1;
    long IDRMM_coeff = -1;
    long IDcoeffmat = -1;

    long linfitsize;
    int linfitreuse;
    double res, res1, v0;


    double resn, vn;
    double LOcoeff;
    FILE *fpLOcoeff;
    long IDwfstmp;

    long pixcnt;
    float vxp, vxm, vyp, vym, cxp, cxm, cyp, cym, ctot;
    long IDtmp1, IDtmp2;


    int COMPUTE_DM_MODES = 1; // compute DM modes (initial step) fmode2b_xxx and fmodes2ball


    long ii1, jj1;
    float dx, dy, dist, dist0, val1cnt;
    long IDprox, IDprox1;
    float gain;

    FILE *fpcoeff;
    char fnameSVDcoeff[400];


    // extra block
    long extrablockIndex;



    // SET LIMITS
    SVDlim00 = SVDlim; // DM filtering step 0
    SVDlim01 = SVDlim; // DM filtering step 1




    MODAL = 0;
    if(msizey==1)
        MODAL = 1;


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



    if(system("mkdir -p mkmodestmp") < 1)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    msizexy = msizex*msizey;

    /// STEP 1: CREATE STARTING POINT : ZERNIKES + FOURIER MODES

    /// if Mmask exists, use it, otherwise create it
    if(MODAL == 0)
    {
        IDmaskRM = image_ID("dmmaskRM");
        if(IDmaskRM==-1)
        {
            double val0, val1;
            double a0=0.88;
            double b0=40.0;
            double a1=1.2;
            double b1=12.0;

            IDmaskRM = create_2Dimage_ID("dmmaskRM", msizex, msizey);
            for(ii=0; ii<msizex; ii++)
                for(jj=0; jj<msizey; jj++)
                {
                    x = 1.0*ii-xc;
                    y = 1.0*jj-yc;
                    r = sqrt(x*x+y*y)/r1;
                    val1 = 1.0-exp(-pow(a1*r,b1));
                    r = sqrt(x*x+y*y)/r0;
                    val0 = exp(-pow(a0*r,b0));
                    data.image[IDmaskRM].array.F[jj*msizex+ii] = val0*val1;
                }
            save_fits("dmmaskRM", "!dmmaskRM.fits");
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
                    xc1 += 1.0*ii*data.image[IDmaskRM].array.F[jj*msizex+ii];
                    yc1 += 1.0*jj*data.image[IDmaskRM].array.F[jj*msizex+ii];
                    totm += data.image[IDmaskRM].array.F[jj*msizex+ii];
                }
            xc1 /= totm;
            yc1 /= totm;
        }

        totm = arith_image_total("dmmaskRM");
        if((msizex != data.image[IDmaskRM].md[0].size[0])||(msizey != data.image[IDmaskRM].md[0].size[1]))
        {
            printf("ERROR: file dmmaskRM size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmaskRM].md[0].size[0], (long) data.image[IDmaskRM].md[0].size[1], (long) msizex, (long) msizey);
            exit(0);
        }
    }
    else
        totm = 1.0;



    COMPUTE_DM_MODES = 0;
    ID2b = image_ID("fmodes2ball");

    if(ID2b == -1)
        COMPUTE_DM_MODES = 1;



    if(COMPUTE_DM_MODES==1) // DM modes fmodes2b
    {
        long ID0 = -1;
        long NBZ = 0;
        long IDmfcpa;
        float CPAblocklim[MAX_MBLOCK]; // defines CPA limits for blocks
        long IDslaved;



        if(MODAL==0)
        {
            long IDmaskRMin;
            long IDmaskRMedge;


            // AOloopControl_mkloDMmodes(ID_name, msizex, msizey, CPAmax, deltaCPA, xc, yc, r0, r1, MaskMode);
            //NBZ = 5; /// 3: tip, tilt, focus
            NBZ = 0;
            for(m=0; m<10; m++)
            {
                if(zcpa[m]<CPAmax)
                    NBZ++;
            }


            // here we create simple Fourier modes
            linopt_imtools_makeCPAmodes("CPAmodes", msizex, CPAmax, deltaCPA, 0.5*msizex, 1.2, 0);
            ID0 = image_ID("CPAmodes");

            long IDfreq = image_ID("cpamodesfreq");



            printf("  %ld %ld %ld\n", msizex, msizey, (long) (data.image[ID0].md[0].size[2]-1) );
            ID = create_3Dimage_ID(ID_name, msizex, msizey, data.image[ID0].md[0].size[2]-1+NBZ);

            IDmfcpa = create_2Dimage_ID("modesfreqcpa", data.image[ID0].md[0].size[2]-1+NBZ, 1);


            zernike_init();

            double PA;
            uint_fast32_t k;
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



            fp = fopen("rmscomp.dat", "w");



            for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
            {

                // set RMS = 1 over mask
                rms = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    //data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                    rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];
                }
                rms = sqrt(rms/totm);
                printf("Mode %ld   RMS = %lf\n", k, rms);

                fprintf(fp, "%5ld  %g ", k, rms);

                /// Remove excluded modes if they exist
                /*          IDeModes = image_ID("emodes");
                          if(IDeModes!=-1)
                          {
                              IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

                              for(ii=0; ii<msizex*msizey; ii++)
                                  data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
                              linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmaskRM", 1.0e-3, "lcoeff", 0);
                              linopt_imtools_image_construct("emodes", "lcoeff", "em00");
                              delete_image_ID("lcoeff");
                              IDem = image_ID("em00");

                //					coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));

                			if(k>kelim)
                				coeff = 1.0;
                			else
                				coeff = 0.0;


                              for(ii=0; ii<msizex*msizey; ii++)
                                  data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

                              delete_image_ID("em00");
                              delete_image_ID("tmpmode");
                          }*/


                // Compute total of image over mask -> totvm
                double totvm = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                    totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

                // compute DC offset in mode
                double offset = totvm/totm;

                // remove DM offset
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] -= offset;

                offset = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                    offset += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

                // set RMS = 1 over mask
                rms = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                    rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];
                }
                rms = sqrt(rms/totm);
                printf("\r Mode %ld   RMS = %lf   ", k, rms);
                fprintf(fp, " %g\n", rms);

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
            }
            fclose(fp);
            printf("\n");


            if(MaskMode==1)
            {
                long kernsize = 5;
                if(2*kernsize>msizex)
                    kernsize = msizex/2;
                long citer;
                long NBciter = 200;
                for(citer=0; citer<NBciter; citer++)
                {
                    printf("Convolution [%3ld/%3ld]\n", citer, NBciter);
                    gauss_filter(ID_name, "modeg", 4.0*pow(1.0*(NBciter-citer)/NBciter,0.5), kernsize);
                    long IDg = image_ID("modeg");
                    uint_fast32_t  k;
                    for(k=0; k<data.image[ID].md[0].size[2]; k++)
                    {
                        for(ii=0; ii<msizex*msizey; ii++)
                            if(data.image[IDmaskRM].array.F[ii]<0.98)
                                data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDg].array.F[k*msizex*msizey+ii];
                    }
                    delete_image_ID("modeg");
                }
            }







            // MAKE MASKS FOR EDGE EXTRAPOLATION


            IDslaved = image_ID("dmslaved");
            // load or create DM mask : union of dmslaved and dmmaskRM
            //IDmask = load_fits("dmmask.fits", "dmmask", 1);

            printf("Create DM mask\n");
            fflush(stdout);


            //IDmask = -1;
            //if(IDmask == -1)
            //{
            IDmask = create_2Dimage_ID("dmmask", msizex, msizey);
            printf("IDs: %ld %ld %ld\n", IDmask, IDmaskRM, IDslaved);
            fflush(stdout);
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[IDmask].array.F[ii] = 1.0 - (1.0-data.image[IDmaskRM].array.F[ii])*(1.0-data.image[IDslaved].array.F[ii]);
                //    data.image[IDmask].array.F[ii] = 1.0 - (1.0-data.image[IDslaved].array.F[ii]);
                if(data.image[IDmask].array.F[ii]>1.0)
                    data.image[IDmask].array.F[ii] = 1.0;
            }
            save_fits("dmmask", "!dmmask.fits");
            //}

            // EDGE PIXELS IN IDmaskRM
            printf("Create dmmaskRMedge\n");
            fflush(stdout);
            IDmaskRMedge = AOloopControl_computeCalib_DMedgeDetect(data.image[IDmaskRM].md[0].name, "dmmaskRMedge");
            save_fits("dmmaskRMedge", "!dmmaskRMedge.fits");

			
            // IDmaskRM pixels excluding edge
            printf("Create dmmaskRMin\n");
            fflush(stdout);
            IDmaskRMin = create_2Dimage_ID("dmmaskRMin", msizex, msizey);
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[IDmaskRMin].array.F[ii] = data.image[IDmaskRM].array.F[ii] * (1.0 - data.image[IDmaskRMedge].array.F[ii]);
            save_fits("dmmaskRMin", "!dmmaskRMin.fits");


            save_fits(ID_name, "!./mkmodestmp/_test_fmodes0all00.fits");

			printf("Running AOloopControl_computeCalib_DMextrapolateModes\n");
            fflush(stdout);
            IDtmp = AOloopControl_computeCalib_DMextrapolateModes(ID_name, "dmmaskRMin", "modesfreqcpa", "fmodes0test");
            save_fits("fmodes0test", "!fmodes0test.fits");

			printf("Applying DM mask on %ud modes\n", data.image[ID].md[0].size[2]);
			fflush(stdout);
            for(m=0; m<data.image[ID].md[0].size[2]; m++)
            {
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[m*msizex*msizey+ii] * data.image[IDmask].array.F[ii];
            }
        }
        else
        {
            ID = create_3Dimage_ID(ID_name, msizex, msizey, msizex);
            IDmfcpa = create_2Dimage_ID("modesfreqcpa", msizex, 1);

            for(m=0; m<data.image[ID].md[0].size[2]; m++)
            {
                if(m<10)
                    data.image[IDmfcpa].array.F[m] = zcpa[m];
                else
                    data.image[IDmfcpa].array.F[m] = zcpa[9];

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[m*msizex*msizey+ii] = 0.0;
                data.image[ID].array.F[m*msizex*msizey+m] = 1.0;
            }
        }



        printf("SAVING MODES : %s...\n", ID_name);
        save_fits(ID_name, "!./mkmodestmp/fmodes0all_00.fits");






        // remove modes
        uint_fast32_t k;
        for(k=0; k<data.image[ID0].md[0].size[2]-1 + NBZ; k++)
        {
            /// Remove excluded modes if they exist
            long IDeModes = image_ID("emodes");
            if(IDeModes!=-1)
            {
                long kelim = 5;
                long IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
                linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmask", 1.0e-3, "lcoeff", 0);
                linopt_imtools_image_construct("emodes", "lcoeff", "em00");
                delete_image_ID("lcoeff");
                long IDem = image_ID("em00");

                double coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));

                if(k>kelim)
                    coeff = 1.0;
                else
                    coeff = 0.0;


                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

                delete_image_ID("em00");
                delete_image_ID("tmpmode");
            }

            // Compute total of image over mask -> totvm
            double totvm = 0.0;
            totm = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
            {
                totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
                totm += data.image[IDmask].array.F[ii];
            }

            // compute DC offset in mode
            double offset = totvm/totm;

            // remove DM offset
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] -= offset;

            offset = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
                offset += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];

            // set RMS = 1 over mask
            rms = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
            }
            rms = sqrt(rms/totm);
            printf("Mode %ld   RMS = %lf\n", k, rms);

            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
        }

        save_fits(ID_name, "!./mkmodestmp/fmodes0all.fits");









        long IDmodes0all = image_ID(ID_name);
        printf("DONE SAVING\n");

        // time : 0:04



        /// COMPUTE WFS RESPONSE TO MODES -> fmodesWFS00all.fits
        msizexy = msizex*msizey;
        ID = image_ID(ID_name);
        IDzrespM = image_ID("zrespM");
        save_fits("zrespM", "!_test_zrespM.fits");
        save_fits(ID_name, "!_test_name.fits");
        if(data.image[IDzrespM].md[0].size[2]!=msizexy)
        {
            printf("ERROR: zrespM has wrong z size : %ld, should be %ld\n", (long) data.image[IDzrespM].md[0].size[2], (long) msizexy);
            exit(0);
        }

        wfsxsize = data.image[IDzrespM].md[0].size[0];
        wfsysize = data.image[IDzrespM].md[0].size[1];
        wfssize = wfsxsize*wfsysize;
        IDm = create_3Dimage_ID("fmodesWFS00all", wfsxsize, wfsysize, data.image[ID].md[0].size[2]);

        printf("size: %ld %ld %ld\n", (long) data.image[ID].md[0].size[2], msizexy, wfssize);
        printf("\n");

        long act1, act2;
# ifdef _OPENMP
        #pragma omp parallel for private(m,m1,act,act1,act2,wfselem)
# endif

        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            m1 = m*wfssize;

            printf("\r %5ld / %5ld   ", m, (long) data.image[ID].md[0].size[2]);
            fflush(stdout);
            for(act=0; act<msizexy; act++)
            {
                act1 = m*msizexy+act;
                act2 = act*wfssize;
                for(wfselem=0; wfselem<wfssize; wfselem++)
                {
                    data.image[IDm].array.F[m1+wfselem] += data.image[ID].array.F[act1] * data.image[IDzrespM].array.F[act2+wfselem];
                }
            }
        }



        // if modal response matrix exists, use it
        IDRMMmodes = image_ID("RMMmodes"); // modal resp matrix modes
        IDRMMresp = image_ID("RMMresp"); // modal resp matrix

        fpLOcoeff = fopen("./mkmodestmp/LOcoeff.txt", "w");
        if(fpLOcoeff == NULL)
        {
            printf("ERROR: cannot create file \"LOcoeff.txt\"\n");
            exit(0);
        }
        save_fits("fmodesWFS00all", "!./mkmodestmp/fmodesWFS00all.HO.fits");

        if((IDRMMmodes!=-1)&&(IDRMMresp!=-1))
        {
            linfitsize = data.image[IDRMMmodes].md[0].size[2];
            IDRMM_coeff = create_2Dimage_ID("linfitcoeff", linfitsize, 1);

            ID_imfit = create_2Dimage_ID("imfitim", msizex, msizey);

            IDcoeffmat = create_2Dimage_ID("imfitmat", linfitsize, data.image[ID].md[0].size[2]);

            linfitreuse = 0;

            IDwfstmp = create_2Dimage_ID("wfsimtmp", wfsxsize, wfsysize);

            for(m=0; m<data.image[IDmodes0all].md[0].size[2]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[ID_imfit].array.F[ii] = data.image[IDmodes0all].array.F[m*msizexy+ii];

                linopt_imtools_image_fitModes("imfitim", "RMMmodes", "dmmaskRM", 1.0e-2, "linfitcoeff", linfitreuse);
                linfitreuse = 1;

                for(jj=0; jj<linfitsize; jj++)
                    data.image[IDcoeffmat].array.F[m*linfitsize+jj] = data.image[IDRMM_coeff].array.F[jj];

                // construct linear fit result (DM)
                IDtmp = create_2Dimage_ID("testrc", msizex, msizey);
                for(jj=0; jj<linfitsize; jj++)
                    for(ii=0; ii<msizex*msizey; ii++)
                        data.image[IDtmp].array.F[ii] += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMMmodes].array.F[jj*msizex*msizey+ii];

                res = 0.0;
                resn = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    v0 = data.image[IDtmp].array.F[ii]-data.image[ID_imfit].array.F[ii];
                    vn = data.image[ID_imfit].array.F[ii];
                    res += v0*v0;
                    resn += vn*vn;
                }
                res /= resn;

                res1 = 0.0;
                for(jj=0; jj<linfitsize; jj++)
                    res1 += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMM_coeff].array.F[jj];

                delete_image_ID("testrc");


                LOcoeff = 1.0/(1.0+pow(10.0*res, 4.0));

                if(res1>1.0)
                    LOcoeff *= 1.0/(1.0+pow((res1-1.0)*0.1, 2.0));


                fprintf(fpLOcoeff, "%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
                // printf("%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);

                if(LOcoeff>0.01)
                {
                    // construct linear fit (WFS space)
                    for(wfselem=0; wfselem<wfssize; wfselem++)
                        data.image[IDwfstmp].array.F[wfselem] = 0.0;
                    for(jj=0; jj<linfitsize; jj++)
                        for(wfselem=0; wfselem<wfssize; wfselem++)
                            data.image[IDwfstmp].array.F[wfselem] += data.image[IDRMM_coeff].array.F[jj] * data.image[IDRMMresp].array.F[jj*wfssize+wfselem];

                    for(wfselem=0; wfselem<wfssize; wfselem++)
                        data.image[IDm].array.F[m*wfssize+wfselem] = LOcoeff*data.image[IDwfstmp].array.F[wfselem] + (1.0-LOcoeff)*data.image[IDm].array.F[m*wfssize+wfselem];
                }
            }

            delete_image_ID("linfitcoeff");
            delete_image_ID("imfitim");
            delete_image_ID("wfsimtmp");
            save_fits("imfitmat", "!imfitmat.fits");
            delete_image_ID("imfitmat");
        }
        fclose(fpLOcoeff);

        printf("\n");
        save_fits("fmodesWFS00all", "!./mkmodestmp/fmodesWFS00all.fits");


        //    exit(0);




        // time : 0:42





        /// STEP 2: SEPARATE DM MODES INTO BLOCKS AND MASK
        msizexy = msizex*msizey;

        CPAblocklim[0] = 0.1; // tip and tilt
        CPAblocklim[1] = 0.3; // focus
        CPAblocklim[2] = 1.6; // other Zernikes
        CPAblocklim[3] = 3.0;
        CPAblocklim[4] = 5.0;
        CPAblocklim[5] = 7.0;
        CPAblocklim[6] = 9.0;
        CPAblocklim[7] = 11.0;
        CPAblocklim[8] = 13.0;
        CPAblocklim[9] = 15.0;
        CPAblocklim[10] = 17.0;
        CPAblocklim[11] = 19.0;
        CPAblocklim[12] = 21.0;
        CPAblocklim[13] = 100.0;

        for(mblock=0; mblock<MAX_MBLOCK; mblock++)
            MBLOCK_NBmode[mblock] = 0;



        NBmblock = 0;
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            float cpa = data.image[IDmfcpa].array.F[m];
            mblock = 0;
            while (cpa > CPAblocklim[mblock])
            {
                //    printf("[%ld  %f %f -> +]\n", mblock, cpa, CPAblocklim[mblock]);
                mblock++;
            }

            MBLOCK_NBmode[mblock]++;

            if(mblock>NBmblock)
                NBmblock = mblock;

            //    printf("%ld %f  -> %ld\n", m, cpa, mblock);
        }

        NBmblock++;


        long IDextrablock = image_ID("extrablockM");
        if(IDextrablock != -1)
        {
            extrablockIndex = 4;

            fp = fopen("./conf/param_extrablockIndex.txt", "r");
            if(fp != NULL)
            {
                if(fscanf(fp, "%50ld", &extrablockIndex) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "cannot read parameter from file");
                fclose(fp);
            }
        }



        for(mblock=0; mblock<NBmblock; mblock++)
        {
            long mblock1;

            if(IDextrablock != -1)
            {
                mblock1 = mblock;
                if(mblock>extrablockIndex-1)
                    mblock1 = mblock+1;
            }
            else
                mblock1 = mblock;

            if(sprintf(imname, "fmodes0_%02ld", mblock1) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            MBLOCK_ID[mblock1] = create_3Dimage_ID(imname, msizex, msizey, MBLOCK_NBmode[mblock]);
            MBLOCK_ID[mblock1] = image_ID(imname);
        }



        for(mblock=0; mblock<MAX_MBLOCK; mblock++)
            MBLOCK_NBmode[mblock] = 0;


        ID = image_ID("fmodes");
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            long mblock1;

            float cpa = data.image[IDmfcpa].array.F[m];

            mblock = 0;
            while (cpa > CPAblocklim[mblock])
                mblock++;

            if(IDextrablock!= -1)
            {
                mblock1 = mblock;
                if(mblock>extrablockIndex-1)
                    mblock1 = mblock+1;
            }
            else
                mblock1 = mblock;


            for(ii=0; ii<msizex*msizey; ii++)
                data.image[MBLOCK_ID[mblock1]].array.F[MBLOCK_NBmode[mblock1]*msizex*msizey+ii] = data.image[ID].array.F[m*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

            MBLOCK_NBmode[mblock1]++;
        }


        if(IDextrablock != -1)
        {
            mblock = extrablockIndex;

            if(sprintf(imname, "fmodes0_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            MBLOCK_NBmode[mblock] = data.image[IDextrablock].md[0].size[2];
            MBLOCK_ID[mblock] = create_3Dimage_ID(imname, msizex, msizey, MBLOCK_NBmode[mblock]);

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[MBLOCK_ID[mblock]].array.F[m*msizex*msizey+ii] = data.image[IDextrablock].array.F[m*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

            NBmblock++;
        }

        // time : 00:42

        /// STEP 3: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim00 FOR CUTOFF -> fmodes1all.fits  (DM space)
        printf("STEP 3: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim00 FOR CUTOFF -> fmodes1all.fits  (DM space)\n");
        fflush(stdout);

        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf("\nMODE BLOCK %ld\n", mblock);
            fflush(stdout);

            if(sprintf(imname, "fmodes0_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            //TEST
            //sprintf(fname, "!./mkmodestmp/fmodes0_%02ld.fits", mblock);
            //save_fits(imname, fname);

            printf("SVD decomp ... (%ld) .... ", (long) data.image[image_ID(imname)].md[0].size[2]);
            fflush(stdout);
            linopt_compute_SVDdecomp(imname, "svdmodes", "svdcoeff");
            printf("DONE\n");
            fflush(stdout);
            cnt = 0;
            IDSVDcoeff = image_ID("svdcoeff");
            float svdcoeff0 = data.image[IDSVDcoeff].array.F[0];
            for(m=0; m<data.image[IDSVDcoeff].md[0].size[0]; m++)
            {
                //printf("( %ld -> %g )\n", m, data.image[IDSVDcoeff].array.F[m]);
                if(data.image[IDSVDcoeff].array.F[m] > SVDlim00*svdcoeff0)
                    cnt++;
            }
            printf("STEP3  -  BLOCK %ld/%ld: keeping %ld / %ld modes  ( %f %f ) [%ld  %ld %ld]\n", mblock, NBmblock, cnt, m, SVDlim00, svdcoeff0, (long) data.image[IDSVDcoeff].md[0].size[0], msizex, msizey);
            fflush(stdout);

            if(sprintf(imname1, "fmodes1_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDm = create_3Dimage_ID(imname1, msizex, msizey, cnt);
            long IDSVDmodes = image_ID("svdmodes");
            for(ii=0; ii<cnt*msizex*msizey; ii++)
                data.image[IDm].array.F[ii] = data.image[IDSVDmodes].array.F[ii];

            MBLOCK_NBmode[mblock] = cnt;
            MBLOCK_ID[mblock] = IDm;

            if(sprintf(fname1, "!./mkmodestmp/fmodes1_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname1);

            delete_image_ID("svdmodes");
            delete_image_ID("svdcoeff");
        }


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes1all", msizex, msizey, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                // printf("Writing cnt %ld    %ld of %ld  [%ld -> %ld]\n", cnt, m, mblock, MBLOCK_ID[mblock], IDm);
                cnt++;
            }
        }
        save_fits("fmodes1all", "!./mkmodestmp/fmodes1all.fits");


        /// STEP 4: REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes2all.fits  (DM space)
        /// fmodes1all -> fmodes2all
        printf("STEP 4: REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes2all.fits  (DM space)\n");
        fflush(stdout);

        IDSVDmask = create_2Dimage_ID("SVDmask", msizex, msizey);
        for(ii=0; ii<msizexy; ii++)
            data.image[IDSVDmask].array.F[ii] = data.image[IDmaskRM].array.F[ii];
        IDSVDmodein = create_2Dimage_ID("SVDmodein", msizex, msizey);

        mok = (int*) malloc(sizeof(int)*NBmm);
        for(m=0; m<NBmm; m++)
            mok[m] = 1;

        for(mblock=0; mblock<NBmblock; mblock++)   // outer block loop
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                mok[m] = 1;
            for(mblock0=0; mblock0<mblock; mblock0++) // inner block loop
            {
                reuse = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    //  printf("STEP 4: REMOVING BLOCK %ld from   block %ld mode %ld/%ld      ", mblock0, mblock, m, MBLOCK_NBmode[mblock]);
                    //  fflush(stdout);

                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDSVDmodein].array.F[ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];

                    if(sprintf(imname, "fmodes1_%02ld", mblock0) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    linopt_imtools_image_fitModes("SVDmodein", imname, "SVDmask", 1.0e-2, "modecoeff", reuse);


                    reuse = 1;
                    linopt_imtools_image_construct(imname, "modecoeff", "SVDmode1");
                    IDSVDmode1 = image_ID("SVDmode1");
                    delete_image_ID("modecoeff");
                    value1 = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                    {
                        data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii] -= data.image[IDSVDmode1].array.F[ii];;
                        value1 += data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii]*data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                    }
                    delete_image_ID("SVDmode1");

                    rms = sqrt(value1/totm);
                    float rmslim0 = 0.01;
                    if(rms>rmslim0)
                    {
                        //       for(ii=0; ii<msizexy; ii++)
                        //         data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii] /= rms;
                    }
                    else
                        mok[m] = 0;

                    //                    printf("->  %12g (%g %g)\n", rms, value1, totm);
                    //					fflush(stdout);
                }
            }
            cnt = 0;
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                cnt += mok[m];
            printf("====== STEP4  -  BLOCK %ld : keeping %ld / %ld modes\n", mblock, cnt, MBLOCK_NBmode[mblock]);
            fflush(stdout);
            if(cnt>0)
            {
                if(sprintf(imname, "fmodes2_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                printf("saving result %s \n", imname);
                fflush(stdout);
                IDm = create_3Dimage_ID(imname, msizex, msizey, cnt);
                m1 = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    if(mok[m]==1)
                    {
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDm].array.F[m1*msizex*msizey+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                        printf("BLOCK %ld   [%ld]  m1 = %ld / %ld\n", mblock, IDm, m1, cnt);
                        fflush(stdout);
                        m1++;
                    }
                }
                MBLOCK_ID[mblock] = IDm;

                char fname2[200];
                if(sprintf(fname2, "!./mkmodestmp/fmodes2_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imname, fname2);
            }
            MBLOCK_NBmode[mblock] = cnt;
        }

        delete_image_ID("SVDmask");
        delete_image_ID("SVDmodein");

        free(mok);


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes2all", msizex, msizey, cnt);


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                cnt++;
            }
        }
        save_fits("fmodes2all", "!./mkmodestmp/fmodes2all.fits");



        // TRUCATE NUMBER OF BLOCKS TO LAST NON-ZERO SIZED BLOCK
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if( MBLOCK_NBmode[mblock] == 0 )
                NBmblock = mblock;
        }



        /// STEP 5: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim01 FOR CUTOFF -> fmodes2ball.fits  (DM space)
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf("====== STEP5  -  MODE BLOCK %ld\n", mblock);
            fflush(stdout);

            if(sprintf(imname, "fmodes2_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            printf("SVD decomp ...");
            fflush(stdout);
            linopt_compute_SVDdecomp(imname, "svdmodes", "svdcoeff");
            printf("DONE\n");
            fflush(stdout);
            cnt = 0;
            IDSVDcoeff = image_ID("svdcoeff");
            float svdcoeff0 = data.image[IDSVDcoeff].array.F[0];

            if(sprintf(fnameSVDcoeff, "./mkmodestmp/SVDcoeff01_%02ld.txt", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            fpcoeff = fopen(fnameSVDcoeff, "w");
            for(m=0; m<data.image[IDSVDcoeff].md[0].size[0]; m++)
            {
                fprintf(fpcoeff, "%5ld   %12g   %12g  %5ld     %10.8f  %10.8f\n", m, data.image[IDSVDcoeff].array.F[m], data.image[IDSVDcoeff].array.F[0], cnt, data.image[IDSVDcoeff].array.F[m]/data.image[IDSVDcoeff].array.F[0], SVDlim01);

                if(data.image[IDSVDcoeff].array.F[m]>SVDlim01*svdcoeff0)
                    cnt++;
            }
            fclose(fpcoeff);

            printf("BLOCK %ld/%ld: keeping %ld / %ld modes\n", mblock, NBmblock, cnt, m);
            fflush(stdout);

            if(sprintf(imname1, "fmodes2b_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDm = create_3Dimage_ID(imname1, msizex, msizey, cnt);
            long IDSVDmodes = image_ID("svdmodes");
            for(ii=0; ii<cnt*msizex*msizey; ii++)
                data.image[IDm].array.F[ii] = data.image[IDSVDmodes].array.F[ii];

            for(m=0; m<cnt; m++)
            {
                value1 = 0.0;
                value1cnt = 0.0;
                for(ii=0; ii<msizexy; ii++)
                {
                    value1 += data.image[IDm].array.F[m*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                    value1cnt += data.image[IDmaskRM].array.F[ii];
                }
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] -= value1/value1cnt;

                value1 = 0.0;
                for(ii=0; ii<msizexy; ii++)
                    value1 += data.image[IDm].array.F[m*msizexy+ii]*data.image[IDm].array.F[m*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                rms = sqrt(value1/value1cnt);
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] /= rms;
            }

            // Extrapolate outside maskRM
            IDtmp = AOloopControl_computeCalib_DMslaveExt(data.image[IDm].md[0].name, data.image[IDmaskRM].md[0].name, "dmslaved", "fmodesext", 100.0);
            for(m=0; m<cnt; m++)
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] = data.image[IDtmp].array.F[m*msizexy+ii];
            delete_image_ID("fmodesext");


            MBLOCK_NBmode[mblock] = cnt;
            MBLOCK_ID[mblock] = IDm;

            if(sprintf(fname1, "!./mkmodestmp/fmodes2b_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname1);

            delete_image_ID("svdmodes");
            delete_image_ID("svdcoeff");
        }


        fp = fopen("./mkmodestmp/NBblocks.txt", "w");
        fprintf(fp, "%ld\n", NBmblock);
        fclose(fp);

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes2ball", msizex, msizey, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                // printf("Writing cnt %ld    %ld of %ld  [%ld -> %ld]\n", cnt, m, mblock, MBLOCK_ID[mblock], IDm);
                cnt++;
            }
        }
        save_fits("fmodes2ball", "!./mkmodestmp/fmodes2ball.fits");
    }
    else
    {
        fp = fopen("./mkmodestmp/NBblocks.txt", "r");
        if(fscanf(fp, "%50ld", &NBmblock) != 1)
            printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

        fclose(fp);
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(fname, "./mkmodestmp/fmodes2b_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imname, "fmodes2b_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            ID = load_fits(fname, imname, 1);
            MBLOCK_NBmode[mblock] = data.image[ID].md[0].size[2];
            MBLOCK_ID[mblock] = ID;
        }
    }


    // 1:25

    // ==================================================

    // WFS modes
    IDzrespM = image_ID("zrespM");
    if(IDzrespM!=-1) // compute WFS response to DM modes
    {
        /// STEP 6: COMPUTE WFS RESPONSE TO MODES
        /// fmodes2ball -> fmodesWFS0all.fits

        char imnameDM[200];
        char imnameDM1[200];
        long MBLOCK_IDwfs[MAX_MBLOCK];



        if(BlockNB<0)
        {   // check size
            if(data.image[IDzrespM].md[0].size[2]!=msizexy)
            {
                printf("ERROR: zrespM has wrong z size : %ld, should be %ld\n", (long) data.image[IDzrespM].md[0].size[2], (long) msizexy);
                exit(0);
            }

            wfsxsize = data.image[IDzrespM].md[0].size[0];
            wfsysize = data.image[IDzrespM].md[0].size[1];
            wfssize = wfsxsize*wfsysize;


            /// Load ... or create WFS mask
            long IDwfsmask = image_ID("wfsmask");
            if((wfsxsize!=data.image[IDwfsmask].md[0].size[0])||(wfsysize!=data.image[IDwfsmask].md[0].size[1]))
            {
                printf("ERROR: File wfsmask has wrong size\n");
                exit(0);
            }
            if(IDwfsmask==-1)
            {
                IDwfsmask = create_2Dimage_ID("wfsmask", wfsxsize, wfsysize);
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDwfsmask].array.F[ii] = 1.0;
            }



            for(mblock=0; mblock<NBmblock; mblock++)
            {
                printf("BLOCK %ld has %ld modes\n", mblock, MBLOCK_NBmode[mblock]);
                fflush(stdout);


                if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(MBLOCK_NBmode[mblock]>0)
                {
                    long IDwfsMresp = create_3Dimage_ID(imname, wfsxsize, wfsysize, MBLOCK_NBmode[mblock]);

# ifdef _OPENMP
                    #pragma omp parallel for private(m,act,wfselem)
# endif
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        for(act=0; act<msizexy; act++)
                        {
                            for(wfselem=0; wfselem<wfssize; wfselem++)
                            {
                                data.image[IDwfsMresp].array.F[m*wfssize+wfselem] += data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+act] * data.image[IDzrespM].array.F[act*wfssize+wfselem];
                            }
                        }
                    }

                    if((IDRMMmodes!=-1)&&(IDRMMresp!=-1))
                    {
                        char fnameLOcoeff[200];
                        if(sprintf(fnameLOcoeff, "./mkmodestmp/LOcoeff_%02ld.txt", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        fpLOcoeff = fopen(fnameLOcoeff, "w");
                        if(fpLOcoeff == NULL)
                        {
                            printf("ERROR: cannot create file \"LOcoeff1.txt\"\n");
                            exit(0);
                        }



                        linfitsize = data.image[IDRMMmodes].md[0].size[2];
                        IDRMM_coeff = create_2Dimage_ID("linfitcoeff", linfitsize, 1);

                        ID_imfit = create_2Dimage_ID("imfitim", msizex, msizey);

                        IDcoeffmat = create_2Dimage_ID("imfitmat", linfitsize, data.image[ID].md[0].size[2]);

                        linfitreuse = 0;

                        IDwfstmp = create_2Dimage_ID("wfsimtmp", wfsxsize, wfsysize);

                        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                        {
                            for(ii=0; ii<msizexy; ii++)
                                data.image[ID_imfit].array.F[ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];

                            linopt_imtools_image_fitModes("imfitim", "RMMmodes", "dmmaskRM", 1.0e-2, "linfitcoeff", linfitreuse);
                            linfitreuse = 1;

                            for(jj=0; jj<linfitsize; jj++)
                                data.image[IDcoeffmat].array.F[m*linfitsize+jj] = data.image[IDRMM_coeff].array.F[jj];

                            // prevent large coefficients (noise propagation)


                            // construct linear fit result (DM)
                            IDtmp = create_2Dimage_ID("testrc", msizex, msizey);
                            for(jj=0; jj<linfitsize; jj++)
                                for(ii=0; ii<msizex*msizey; ii++)
                                    data.image[IDtmp].array.F[ii] += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMMmodes].array.F[jj*msizex*msizey+ii];

                            res = 0.0;
                            resn = 0.0;
                            for(ii=0; ii<msizex*msizey; ii++)
                            {
                                v0 = data.image[IDtmp].array.F[ii]-data.image[ID_imfit].array.F[ii];
                                vn = data.image[ID_imfit].array.F[ii];
                                res += v0*v0;
                                resn += vn*vn;
                            }
                            res /= resn;

                            res1 = 0.0;  // norm squared of linear vector
                            for(jj=0; jj<linfitsize; jj++)
                                res1 += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMM_coeff].array.F[jj];

                            delete_image_ID("testrc");


                            LOcoeff = 1.0/(1.0+pow(10.0*res, 4.0));

                            if(res1>1.0)
                                LOcoeff *= 1.0/(1.0+pow((res1-1.0)*0.1, 2.0));


                            fprintf(fpLOcoeff, "%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
                            // printf("%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);

                            if(LOcoeff>0.01)
                            {
                                // construct linear fit (WFS space)
                                for(wfselem=0; wfselem<wfssize; wfselem++)
                                    data.image[IDwfstmp].array.F[wfselem] = 0.0;
                                for(jj=0; jj<linfitsize; jj++)
                                    for(wfselem=0; wfselem<wfssize; wfselem++)
                                        data.image[IDwfstmp].array.F[wfselem] += data.image[IDRMM_coeff].array.F[jj] * data.image[IDRMMresp].array.F[jj*wfssize+wfselem];

                                for(wfselem=0; wfselem<wfssize; wfselem++)
                                    data.image[IDwfsMresp].array.F[m*wfssize+wfselem] = LOcoeff*data.image[IDwfstmp].array.F[wfselem] + (1.0-LOcoeff)*data.image[IDwfsMresp].array.F[m*wfssize+wfselem];
                            }
                        }

                        delete_image_ID("linfitcoeff");
                        delete_image_ID("imfitim");

                        save_fits("imfitmat", "!imfitmat.fits");
                        delete_image_ID("imfitmat");

                        fclose(fpLOcoeff);
                    }


                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                        for(wfselem=0; wfselem<wfssize; wfselem++)
                            data.image[IDwfsMresp].array.F[m*wfssize+wfselem] *= data.image[IDwfsmask].array.F[wfselem];


                    if(sprintf(fname, "!./mkmodestmp/fmodesWFS0_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname, fname);
                }
            }

            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
                cnt += MBLOCK_NBmode[mblock];
            IDm = create_3Dimage_ID("fmodesWFS0all", wfsxsize, wfsysize, cnt);
            cnt = 0;


            for(mblock=0; mblock<NBmblock; mblock++)
            {
                if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs = image_ID(imname);
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    for(ii=0; ii<wfssize; ii++)
                        data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                    cnt++;
                }
            }
            save_fits("fmodesWFS0all", "!./mkmodestmp/fmodesWFS0all.fits");


            // time : 02:00


            /// STEP 7: REMOVE WFS MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE WFS-SPACE ORTHOGONALITY BETWEEN BLOCKS
            /// Input: fmodesWFS0all (corresponding to fmodes2ball)
            /// Output -> fmodesWFS1all / fmodes3all

            IDSVDmask = create_2Dimage_ID("SVDmask", wfsxsize, wfsysize);
            for(ii=0; ii<wfssize; ii++)
                data.image[IDSVDmask].array.F[ii] = 1.0;
            IDSVDmodein = create_2Dimage_ID("SVDmodein", wfsxsize, wfsysize);

            mok = (int*) malloc(sizeof(int)*NBmm);
            for(m=0; m<NBmm; m++)
                mok[m] = 1;



            for(mblock=0; mblock<NBmblock; mblock++)
            {
                float *rmsarray;
                rmsarray = (float*) malloc(sizeof(float)*MBLOCK_NBmode[mblock]);
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);
                    value1 = 0.0;
                    for(ii=0; ii<wfssize; ii++)
                        value1 += data.image[IDmwfs].array.F[m*wfssize+ii]*data.image[IDmwfs].array.F[m*wfssize+ii];
                    rmsarray[m] = sqrt(value1/wfssize);
                }

                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    mok[m] = 1;



                // REMOVE WFS MODES FROM PREVIOUS BLOCKS

                for(mblock0=0; mblock0<mblock; mblock0++)
                {

                    reuse = 0;
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        long IDmwfs = image_ID(imname);

                        if(sprintf(imnameDM, "fmodes2b_%02ld", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        IDm = image_ID(imnameDM);


                        for(ii=0; ii<wfsxsize*wfsysize; ii++)
                            data.image[IDSVDmodein].array.F[ii] = data.image[IDmwfs].array.F[m*wfssize+ii];

                        if(sprintf(imname, "fmodesWFS0_%02ld", mblock0) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        if(sprintf(imnameDM, "fmodes2b_%02ld", mblock0) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        linopt_imtools_image_fitModes("SVDmodein", imname, "SVDmask", 1.0e-2, "modecoeff", reuse);
                        IDSVDcoeff = image_ID("modecoeff");
                        reuse = 1;
                        linopt_imtools_image_construct(imname, "modecoeff", "SVDmode1");
                        linopt_imtools_image_construct(imnameDM, "modecoeff", "SVDmode1DM");
                        IDSVDmode1 = image_ID("SVDmode1");

                        long IDSVDmode1DM = image_ID("SVDmode1DM");

                        delete_image_ID("modecoeff");

                        value1 = 0.0;
                        for(ii=0; ii<wfssize; ii++)
                        {
                            data.image[IDmwfs].array.F[m*wfssize+ii] -= data.image[IDSVDmode1].array.F[ii];
                            value1 += data.image[IDmwfs].array.F[m*wfssize+ii]*data.image[IDmwfs].array.F[m*wfssize+ii];
                        }
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDm].array.F[m*msizexy+ii] -= data.image[IDSVDmode1DM].array.F[ii];


                        delete_image_ID("SVDmode1");
                        delete_image_ID("SVDmode1DM");

                        rms = sqrt(value1/wfssize);

                        if(rms<rmsarray[m]*rmslim1)
                        {
                            mok[m] = 0;
                        }
                        printf("RMS RATIO  %3ld :   %12g\n", m, rms/rmsarray[m]);
                    }
                }

                cnt = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    cnt += mok[m];
                printf("====== WFS BLOCK %ld : keeping %ld / %ld modes\n", mblock, cnt, MBLOCK_NBmode[mblock]);

                if(cnt>0)
                {
                    if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");


                    long IDmwfs1 = create_3Dimage_ID(imname, wfsxsize, wfsysize, cnt);
                    long IDmdm1 = create_3Dimage_ID(imnameDM, msizex, msizey, cnt);
                    m1 = 0;

                    if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);

                    if(sprintf(imnameDM, "fmodes2b_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmdm = image_ID(imnameDM);
                    if(IDmdm==-1)
                    {
                        printf("ERROR: image %s does not exist\n", imnameDM);
                        exit(0);
                    }
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        if(mok[m]==1)
                        {
                            printf("writing %ld / %ld  ->  %ld / %ld        \n", m, (long) data.image[IDmwfs].md[0].size[2], m1, (long) data.image[IDm].md[0].size[2]);

                            printf("max index IDmwfs1 %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m1, (long) (m1*wfssize+wfssize-1), (long) (data.image[IDmwfs1].md[0].size[0]*data.image[IDmwfs1].md[0].size[1]*data.image[IDmwfs1].md[0].size[2]), (long) data.image[IDmwfs1].md[0].size[0], (long) data.image[IDmwfs1].md[0].size[1], (long) data.image[IDmwfs1].md[0].size[2]);
                            printf("max index IDmwfs  %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m, (long) (m*wfssize+wfssize-1), (long) (data.image[IDmwfs].md[0].size[0]*data.image[IDmwfs].md[0].size[1]*data.image[IDmwfs].md[0].size[2]), (long) data.image[IDmwfs].md[0].size[0], (long) data.image[IDmwfs].md[0].size[1], (long) data.image[IDmwfs].md[0].size[2]);

                            printf("max index IDmdm1  %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m1, (long) (m1*msizexy+msizexy-1), (long) (data.image[IDmdm1].md[0].size[0]*data.image[IDmdm1].md[0].size[1]*data.image[IDmdm1].md[0].size[2]), (long) data.image[IDmdm1].md[0].size[0], (long) data.image[IDmdm1].md[0].size[1], (long) data.image[IDmdm1].md[0].size[2]);
                            printf("max index IDmdm   %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m, (long) (m*msizexy+msizexy-1), (long) (data.image[IDmdm].md[0].size[0]*data.image[IDmdm].md[0].size[1]*data.image[IDmdm].md[0].size[2]), (long) data.image[IDmdm].md[0].size[0], (long) data.image[IDmdm].md[0].size[1], (long) data.image[IDmdm].md[0].size[2]);


                            fflush(stdout);//TEST
                            for(ii=0; ii<wfssize; ii++)
                                data.image[IDmwfs1].array.F[m1*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                            for(ii=0; ii<msizexy; ii++)
                                data.image[IDmdm1].array.F[m1*msizexy+ii] = data.image[IDmdm].array.F[m*msizexy+ii];
                            value1 = 0.0;
                            m1++;
                        }
                        else
                        {
                            printf("Skipping %ld / %ld\n", m, (long) data.image[IDmwfs].md[0].size[2]);
                            fflush(stdout);
                        }
                    }
                    printf("STEP 0000\n");
                    fflush(stdout);//TEST

                    if(sprintf(imname1, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(fname1, "!./mkmodestmp/fmodesWFS1_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    printf("   saving   %s -> %s\n", imname1, fname1);
                    fflush(stdout);//TEST

                    save_fits(imname1, fname1);

                    printf("STEP 0001\n");
                    fflush(stdout);//TEST

                    if(sprintf(imname1, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(fname1, "!./mkmodestmp/fmodes3_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname1, fname1);
                    MBLOCK_ID[mblock] = IDmdm1;
                    printf("STEP 0002\n");
                    fflush(stdout);//TEST
                }
                else
                {
                    printf("ERROR: keeping no mode in block !!!\n");
                    exit(0);
                }
                printf("STEP 0010\n");
                fflush(stdout);//TEST

                MBLOCK_NBmode[mblock] = cnt;
                free(rmsarray);
            }
            delete_image_ID("SVDmask");
            delete_image_ID("SVDmodein");

            printf("STEP 0020\n");
            fflush(stdout);//TEST

            free(mok);


            // time : 04:34


            list_image_ID();
            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
                cnt += MBLOCK_NBmode[mblock];
            IDm = create_3Dimage_ID("fmodesWFS1all", wfsxsize, wfsysize, cnt);
            long IDmdm1 = create_3Dimage_ID("fmodes3all", msizex, msizey, cnt);

            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
            {
                if(MBLOCK_NBmode[mblock]>0)
                {
                    if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);

                    if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmdm = image_ID(imnameDM);

                    if(IDmwfs==-1)
                    {
                        printf("ERROR: image %s does not exit\n", imname);
                        exit(0);
                    }
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        // printf("writing %ld / %ld  ->  %ld / %ld\n", m, data.image[IDmwfs].md[0].size[2], cnt, data.image[IDm].md[0].size[2]);
                        // fflush(stdout);
                        for(ii=0; ii<wfssize; ii++)
                            data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDmdm1].array.F[cnt*msizexy+ii] = data.image[IDmdm].array.F[m*msizexy+ii];
                        cnt++;
                    }
                }
            }
            save_fits("fmodesWFS1all", "!./mkmodestmp/fmodesWFS1all.fits");
            save_fits("fmodes3all", "!./mkmodestmp/fmodes3all.fits");


        }


        // time : 04:36

        if(BlockNB<0)
        {
            char command[1000];
            if(sprintf(command, "echo \"%ld\" > ./conf_staged/param_NBmodeblocks.txt", NBmblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");
        }
        else
        {
            if((fp = fopen("./conf/param_NBmodeblocks.txt", "r"))==NULL)
            {
                printf("ERROR: cannot read file ./conf_staged/param_NBmodeblocks.txt\n");
                exit(0);
            }
            if(fscanf(fp, "%50ld", &NBmblock) != 1)
                printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");
            fclose(fp);
        }

        printf("%ld blocks\n", NBmblock);



        /// STEP 8: SVD WFS SPACE IN EACH BLOCK
        /// fmodesWFS1all, fmodes3 -> fmodesall

        // fmodesWFS1_##, fmodes3_## -> fmodes_##

        for(mblock=0; mblock<NBmblock; mblock++)
        {
            long IDmask;

            if(BlockNB>-1) // LOAD & VERIFY SIZE
            {
                if(sprintf(imname1, "fmodesWFS1_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(fname1, "./mkmodestmp/fmodesWFS1_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                ID = load_fits(fname1, imname1, 1);
                wfsxsize = data.image[ID].md[0].size[0];
                wfsysize = data.image[ID].md[0].size[1];
                wfssize = wfsxsize*wfsysize;

                if(sprintf(imname1, "fmodes3_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(fname1, "./mkmodestmp/fmodes3_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                ID = load_fits(fname1, imname1, 1);
                if((data.image[ID].md[0].size[0] != msizex) && (msizey != data.image[ID].md[0].size[0]))
                {
                    printf("ERROR: file dmmaskRM size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmask].md[0].size[0], (long) data.image[IDmask].md[0].size[1], (long) msizex, (long) msizey);
                    exit(0);
                }
                msizexy = data.image[ID].md[0].size[0]*data.image[ID].md[0].size[1];
            }


            if((BlockNB<0)||(BlockNB==mblock))
            {
                char command[1000];


                if(sprintf(command, "echo \"%f\" > ./conf_staged/block%02ld_SVDlim.txt", SVDlim, mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command) != 0)
                    printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");


                //if(MBLOCK_NBmode[mblock]>-1)
                //{

                if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs = image_ID(imname);
                if(IDmwfs==-1)
                {
                    printf("ERROR: image %s does not exit\n", imname);
                    exit(0);
                }

                if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmdm = image_ID(imnameDM);
                if(IDmdm==-1)
                {
                    printf("ERROR: image %s does not exit\n", imnameDM);
                    exit(0);
                }

                if(sprintf(imnameDM1, "fmodes_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");


                linopt_compute_SVDdecomp(imname, "SVDout", "modecoeff"); // SVD
                IDSVDcoeff = image_ID("modecoeff");

                cnt = 0;

                if(sprintf(fnameSVDcoeff, "./mkmodestmp/SVDcoeff_%02ld.txt", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                fpcoeff = fopen(fnameSVDcoeff, "w");
                uint_fast16_t kk;
                for(kk=0; kk<data.image[IDSVDcoeff].md[0].size[0]; kk++)
                {
                    fprintf(fpcoeff, "%5ld   %12g   %12g  %5ld     %10.8f  %10.8f\n", kk, data.image[IDSVDcoeff].array.F[kk], data.image[IDSVDcoeff].array.F[0], cnt, data.image[IDSVDcoeff].array.F[kk]/data.image[IDSVDcoeff].array.F[0], SVDlim);
                    printf("==== %ld %12g %12g  %3ld\n", kk, data.image[IDSVDcoeff].array.F[kk], data.image[IDSVDcoeff].array.F[0], cnt);
                    if(data.image[IDSVDcoeff].array.F[kk]>SVDlim*data.image[IDSVDcoeff].array.F[0])
                        cnt++;
                }
                fclose(fpcoeff);


                long IDmdm1 = create_3Dimage_ID(imnameDM1, msizex, msizey, cnt);

                char imnameWFS1[200];
                if(sprintf(imnameWFS1, "fmodesWFS_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs1 = create_3Dimage_ID(imnameWFS1, wfsxsize, wfsysize, cnt);
                long ID_VTmatrix = image_ID("SVD_VTm");


                for(kk=0; kk<cnt; kk++) /// eigen mode index
                {
                    long kk1;
                    for(kk1=0; kk1<data.image[IDSVDcoeff].md[0].size[0]; kk1++)
                    {
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDmdm1].array.F[kk*msizexy + ii] += data.image[ID_VTmatrix].array.F[kk1*data.image[IDSVDcoeff].md[0].size[0]+kk]*data.image[IDmdm].array.F[kk1*msizexy + ii];

                        for(ii=0; ii<wfssize; ii++)
                            data.image[IDmwfs1].array.F[kk*wfssize + ii] += data.image[ID_VTmatrix].array.F[kk1*data.image[IDSVDcoeff].md[0].size[0]+kk]*data.image[IDmwfs].array.F[kk1*wfssize + ii];
                    }

                    value1 = 0.0;
                    value1cnt = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                    {
                        value1 += data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                        value1cnt += data.image[IDmaskRM].array.F[ii];
                    }
                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDmdm1].array.F[kk*msizexy+ii] -= value1/value1cnt;

                    value1 = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                        value1 += data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                    rms = sqrt(value1/value1cnt);

                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDmdm1].array.F[kk*msizexy+ii] /= rms;

                    for(ii=0; ii<wfssize; ii++)
                        data.image[IDmwfs1].array.F[kk*wfssize+ii] /= rms;


                    /*     value1 = 0.0;
                         for(ii=0; ii<msizexy; ii++)
                             value1 += data.image[IDmdm1].array.F[kk*msizexy + ii]*data.image[IDmdm1].array.F[kk*msizexy + ii];
                         rms = sqrt(value1/totm);
                         */


                    // for(ii=0; ii<msizexy; ii++)
                    //     data.image[IDmdm1].array.F[kk*msizexy + ii] /= rms;
                }
                delete_image_ID("SVDout");
                delete_image_ID("modecoeff");

                if(sprintf(fname, "!./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imnameDM1, fname);

                if(sprintf(fname, "!./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imnameWFS1, fname);
                MBLOCK_ID[mblock] = IDmdm1;
                MBLOCK_IDwfs[mblock] = IDmwfs1;
                MBLOCK_NBmode[mblock] = cnt;
                //}
            }
            else
            {
                if(sprintf(fname, "./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(imnameDM1, "fmodes_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmdm1 = load_fits(fname, imnameDM1, 1);
                MBLOCK_ID[mblock] = IDmdm1;
                //MBLOCK_IDwfs[mblock] = IDmwfs1;
                MBLOCK_NBmode[mblock] = data.image[IDmdm1].md[0].size[2];
            }
        }

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodesall", msizex, msizey, cnt);
        long IDwfs = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        long cnt1 = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(MBLOCK_NBmode[mblock]>0)
                cnt1++;

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDwfs].array.F[cnt*wfssize+ii] = data.image[MBLOCK_IDwfs[mblock]].array.F[m*wfssize+ii];


                cnt++;
            }
        }

        save_fits("fmodesall", "!./mkmodestmp/fmodesall.fits");
        save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");

        NBmblock = cnt1;


        /// WFS MODES, MODAL CONTROL MATRICES
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf(".... BLOCK %ld has %ld modes\n", mblock, MBLOCK_NBmode[mblock]);
            fflush(stdout);

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCM[200]; // modal control matrix
            if(sprintf(imnameCM, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCMc[200]; // zonal ("combined") control matrix
            if(sprintf(imnameCMc, "cmatc_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCMcact[200]; // zonal control matrix masked
            if(sprintf(imnameCMcact, "cmatcact_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if((BlockNB<0)||(BlockNB==mblock))
            {
                if(MBLOCK_NBmode[mblock]>0)
                {



                    printf("-- COMPUTE MODAL CONTROL MATRICES\n");
                    fflush(stdout);

                    // COMPUTE MODAL CONTROL MATRICES
                    printf("COMPUTE CONTROL MATRIX\n");
                    float SVDlim1 = 0.01; // WFS filtering (ONLY USED FOR FULL SINGLE STEP INVERSION)
#ifdef HAVE_MAGMA
                    CUDACOMP_magma_compute_SVDpseudoInverse(imname, imnameCM, SVDlim1, 10000, "VTmat", 0, 0, 1.e-4, 1.e-7, 0);
#else
                    linopt_compute_SVDpseudoInverse(imname, imnameCM, SVDlim1, 10000, "VTmat");
#endif

                    delete_image_ID("VTmat");

                    if(sprintf(fname, "!./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imnameCM, fname);

                    printf("-- COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX\n");
                    fflush(stdout);

                    // COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX
                    sprintf(imname, "fmodes_%02ld", mblock);
                    AOloopControl_computeCalib_compute_CombinedControlMatrix(imnameCM, imname, "wfsmask", "dmmask", imnameCMc, imnameCMcact);


                    if(sprintf(fname, "!./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imnameCMc, fname);

                    if(sprintf(fname, "!./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(imname1, "%s_00", imnameCMcact) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname1, fname);

                    list_image_ID();
                }

            }
            else
            {
                printf("LOADING WFS MODES, MODAL CONTROL MATRICES: block %ld\n", mblock);
                fflush(stdout);

                //	list_image_ID();

                if(sprintf(fname, "./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imname, 1);

                if(sprintf(fname, "./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCM, 1);

                if(sprintf(fname, "./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCMc, 1);

                if(sprintf(fname, "./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCMcact, 1);
            }
        }

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            char command[1000];
            if(sprintf(command, "echo \"%ld\" > ./conf_staged/block%02ld_NBmodes.txt", MBLOCK_NBmode[mblock], mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            long IDmwfs = image_ID(imname);
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                cnt++;
            }
        }
        save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");


        fp = fopen("./mkmodestmp/NBmodes.txt", "w");
        fprintf(fp, "%ld\n", cnt);
        fclose(fp);


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        long IDcmatall = create_3Dimage_ID("cmatall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(imname, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            long IDcmat = image_ID(imname);
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDcmatall].array.F[cnt*wfssize+ii] = data.image[IDcmat].array.F[m*wfssize+ii];
                cnt++;
            }
        }
        save_fits("cmatall", "!./mkmodestmp/cmatall.fits");




        // COMPUTE OVERALL CONTROL MATRIX
        /*    int COMPUTE_FULL_CMAT = 0;
            if(COMPUTE_FULL_CMAT == 1)
            {
                printf("COMPUTE OVERALL CONTROL MATRIX\n");
                float SVDlim1 = 0.01; // WFS filtering (ONLY USED FOR FULL SINGLE STEP INVERSION)
                #ifdef HAVE_MAGMA
                    CUDACOMP_magma_compute_SVDpseudoInverse("fmodesWFSall", "cmat", SVDlim1, 100000, "VTmat", 0);
                #else
                    linopt_compute_SVDpseudoInverse("fmodesWFSall", "cmat", SVDlim1, 10000, "VTmat");
        		#endif

                delete_image_ID("VTmat");
                save_fits("cmat", "!./mkmodestmp/cmat.fits");

        	}

        		char command[1000];
                if(sprintf(command, "echo \"%ld\" > ./conf_staged/param_NBmodes.txt", cnt) < 1)
        			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command) != 0)
        			printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

            */
    }
    // time : 07:43


    return(ID);
}


/*** \brief Creates control matrices per block, using native modes
 */
long AOloopControl_computeCalib_mkModes_Simple(
    const char *IDin_name,
    long NBmblock,
    long Cmblock,
    float SVDlim
)
{
    long IDin; // input WFS responses
    FILE *fp;
    long mblock;
    long *MBLOCK_NBmode;
    long *MBLOCK_blockstart;
    long *MBLOCK_blockend;
    char fname[500];

    char imname[500];
    char imname1[500];
    long ID;
    long wfsxsize, wfsysize;
    long wfssize;
    long ii, kk;
    char imnameCM[500];
    char imnameCMc[500];
    char imnameCMcact[500];
    long IDwfsmask;
    long IDdmmask;
    long IDmodes;
    long NBmodes;
    long cnt;
    long IDm;
    long m;
    long IDcmatall;
    char command[500];


    printf("Function AOloopControl_mkModes_Simple - Cmblock = %ld / %ld\n", Cmblock, NBmblock);
    fflush(stdout);

    if(system("mkdir -p mkmodestmp") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");



    MBLOCK_NBmode = (long*) malloc(sizeof(long)*NBmblock);
    MBLOCK_blockstart = (long*) malloc(sizeof(long)*NBmblock);
    MBLOCK_blockend = (long*) malloc(sizeof(long)*NBmblock);


    IDin = image_ID(IDin_name);
    wfsxsize = data.image[IDin].md[0].size[0];
    wfsysize = data.image[IDin].md[0].size[1];
    wfssize = wfsxsize*wfsysize;
    NBmodes = data.image[IDin].md[0].size[2];

    // read block ends
    if(NBmblock==1)
    {
        MBLOCK_blockend[0] = NBmodes;

        if(sprintf(fname, "./conf_staged/param_block00end.txt") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        fp = fopen(fname, "w");
        fprintf(fp, "%03ld\n", NBmodes);
        fclose(fp);
    }
    else
    {
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(fname, "./conf_staged/param_block%02ldend.txt", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            fp = fopen(fname, "r");
            if(fp==NULL)
            {
                printf("ERROR: File \"%s\" not found\n", fname);
                exit(0);
            }
            if(fscanf(fp, "%50ld", &MBLOCK_blockend[mblock]) != 1)
                printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");
            fclose(fp);

            printf("Block end %ld = %ld\n", mblock, MBLOCK_blockend[mblock]);
            fflush(stdout);
        }
    }

    MBLOCK_NBmode[0] = MBLOCK_blockend[0];
    MBLOCK_blockstart[0] = 0;
    for(mblock=1; mblock<NBmblock; mblock++)
    {
        MBLOCK_NBmode[mblock] = MBLOCK_blockend[mblock] - MBLOCK_blockend[mblock-1];
        MBLOCK_blockstart[mblock] =  MBLOCK_blockstart[mblock-1] + MBLOCK_NBmode[mblock-1];
    }




    IDmodes = create_3Dimage_ID("fmodesall", NBmodes, 1, NBmodes);
    for(kk=0; kk<NBmodes*NBmodes; kk++)
        data.image[IDmodes].array.F[kk] = 0.0;
    for(kk=0; kk<NBmodes; kk++)
        data.image[IDmodes].array.F[kk*NBmodes+kk] = 1.0;
    save_fits("fmodesall", "!./mkmodestmp/fmodesall.fits");

    for(mblock=0; mblock<NBmblock; mblock++)
    {
        printf("mblock %02ld  : %ld modes\n", mblock, MBLOCK_NBmode[mblock]);


        if( (Cmblock == mblock) || (Cmblock == -1) )
        {
            printf("Reconstructing block %ld\n", mblock);

            if(sprintf(command, "echo \"%f\" > ./conf_staged/block%02ld_SVDlim.txt", SVDlim, mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");


            IDdmmask = create_2Dimage_ID("dmmask", NBmodes, 1);
            for(kk=0; kk<NBmodes; kk++)
                data.image[IDdmmask].array.F[kk] = 1.0;

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            ID = create_3Dimage_ID(imname, wfsxsize, wfsysize, MBLOCK_NBmode[mblock]);
            for(kk=0; kk<MBLOCK_NBmode[mblock]; kk++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[ID].array.F[kk*wfssize+ii] = data.image[IDin].array.F[(kk+MBLOCK_blockstart[mblock])*wfssize+ii];
            }

            if(sprintf(fname, "!./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname, fname);


            if(sprintf(imnameCM, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imnameCMc, "cmatc_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imnameCMcact, "cmatcact_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            // COMPUTE MODAL CONTROL MATRICES
            printf("COMPUTE CONTROL MATRIX\n");
#ifdef HAVE_MAGMA
            CUDACOMP_magma_compute_SVDpseudoInverse(imname, imnameCM, SVDlim, 10000, "VTmat", 0, 0, 1.e-4, 1.e-7, 0);
#else
            linopt_compute_SVDpseudoInverse(imname, imnameCM, SVDlim, 10000, "VTmat");
#endif

            delete_image_ID("VTmat");

            if(sprintf(fname, "!./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imnameCM, fname);

            printf("-- COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX\n");
            fflush(stdout);

            // COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX
            if(sprintf(imname, "fmodes_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDmodes = create_3Dimage_ID(imname, NBmodes, 1, MBLOCK_NBmode[mblock]);
            list_image_ID();
            for(kk=0; kk<MBLOCK_NBmode[mblock]; kk++)
            {
                for(ii=0; ii<NBmodes; ii++)
                    data.image[IDmodes].array.F[kk*NBmodes+ii] = 0.0;
                data.image[IDmodes].array.F[kk*NBmodes+(kk+MBLOCK_blockstart[mblock])] = 1.0;
            }

            if(sprintf(fname, "!./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname, fname);


            AOloopControl_computeCalib_compute_CombinedControlMatrix(imnameCM, imname, "wfsmask", "dmmask", imnameCMc, imnameCMcact);
            delete_image_ID("dmmask");

            if(sprintf(fname, "!./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imnameCMc, fname);

            if(sprintf(fname, "!./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imname1, "%s_00", imnameCMcact) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname);
        }

        else
        {
            printf("LOADING WFS MODES, MODAL CONTROL MATRICES: block %ld\n", mblock);
            fflush(stdout);

            if(sprintf(fname, "./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imname, 1);

            if(sprintf(fname, "./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCM, 1);

            if(sprintf(fname, "./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCMc, 1);

            if(sprintf(fname, "./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCMcact, 1);
        }
    }


    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
        cnt += MBLOCK_NBmode[mblock];
    IDm = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
    {
        if(sprintf(command, "echo \"%ld\" > ./conf_staged/block%02ld_NBmodes.txt", MBLOCK_NBmode[mblock], mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if(system(command) != 0)
            printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

        if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDmwfs = image_ID(imname);
        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
        {
            for(ii=0; ii<wfssize; ii++)
                data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
            cnt++;
        }
    }
    save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");


    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
        cnt += MBLOCK_NBmode[mblock];
    IDcmatall = create_3Dimage_ID("cmatall", wfsxsize, wfsysize, cnt);
    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
    {
        if(sprintf(imname, "cmat_%02ld", mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDcmat = image_ID(imname);
        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
        {
            for(ii=0; ii<wfssize; ii++)
                data.image[IDcmatall].array.F[cnt*wfssize+ii] = data.image[IDcmat].array.F[m*wfssize+ii];
            cnt++;
        }
    }
    save_fits("cmatall", "!./mkmodestmp/cmatall.fits");





    free(MBLOCK_NBmode);
    free(MBLOCK_blockstart);
    free(MBLOCK_blockend);


    return(IDin);
}



int_fast8_t AOloopControl_computeCalib_mkCalib_map_mask(
    long loop,
    const char *zrespm_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    float dmmask_perclow,
    float dmmask_coefflow,
    float dmmask_perchigh,
    float dmmask_coeffhigh,
    float wfsmask_perclow,
    float wfsmask_coefflow,
    float wfsmask_perchigh,
    float wfsmask_coeffhigh
)
{
    long IDWFSmap, IDDMmap;
    long IDWFSmask, IDDMmask;
    long IDzrm;
    long ii;
    float lim, rms;
    double tmpv;
    long sizexWFS, sizeyWFS, sizeWFS;
    long sizexDM, sizeyDM;
    long IDdm;
    char name[200];
    long NBpoke, poke;
    long IDDMmap1;
    float lim0;
    long IDtmp;


    IDzrm = image_ID(zrespm_name);
    sizexWFS = data.image[IDzrm].md[0].size[0];
    sizeyWFS = data.image[IDzrm].md[0].size[1];
    NBpoke = data.image[IDzrm].md[0].size[2];

    if(sprintf(name, "aol%ld_dmC", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDdm = read_sharedmem_image(name);
    sizexDM = data.image[IDdm].md[0].size[0];
    sizeyDM = data.image[IDdm].md[0].size[1];

    sizeWFS = sizexWFS*sizeyWFS;

    IDWFSmap = create_2Dimage_ID(WFSmap_name, sizexWFS, sizeyWFS);
    IDDMmap = create_2Dimage_ID(DMmap_name, sizexDM, sizeyDM);
    IDWFSmask = create_2Dimage_ID("wfsmask", sizexWFS, sizeyWFS);
    IDDMmask = create_2Dimage_ID("dmmask", sizexDM, sizeyDM);




    printf("Preparing DM map ... ");
    fflush(stdout);
    for(poke=0; poke<NBpoke; poke++)
    {
        rms = 0.0;
        for(ii=0; ii<sizeWFS; ii++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDDMmap].array.F[poke] = rms;
    }
    printf("done\n");
    fflush(stdout);



    printf("Preparing WFS map ... ");
    fflush(stdout);
    for(ii=0; ii<sizeWFS; ii++)
    {
        rms = 0.0;
        for(poke=0; poke<NBpoke; poke++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDWFSmap].array.F[ii] = rms;
    }
    printf("done\n");
    fflush(stdout);




    printf("Preparing DM mask ... ");
    fflush(stdout);

    // pre-filtering
    // gauss_filter(DMmap_name, "dmmapg", 5.0, 8);
    // IDDMmap1 = image_ID("dmmapg");

    // (map/map1)*pow(map,0.25)

    // DMmask: select pixels
    lim0 = dmmask_coefflow*img_percentile(DMmap_name, dmmask_perclow);
    IDtmp = create_2Dimage_ID("_tmpdmmap", sizexDM, sizeyDM);
    for(ii=0; ii<sizexDM*sizeyDM; ii++)
        data.image[IDtmp].array.F[ii] = data.image[IDDMmap].array.F[ii] - lim0;
    lim = dmmask_coeffhigh*img_percentile("_tmpdmmap", dmmask_perchigh);

    for(poke=0; poke<NBpoke; poke++)
    {
        if(data.image[IDtmp].array.F[poke]<lim)
            data.image[IDDMmask].array.F[poke] = 0.0;
        else
            data.image[IDDMmask].array.F[poke] = 1.0;
    }
    delete_image_ID("_tmpdmmap");
    printf("done\n");
    fflush(stdout);



    // WFSmask : select pixels
    printf("Preparing WFS mask ... ");
    fflush(stdout);

    lim0 = wfsmask_coefflow*img_percentile(WFSmap_name, wfsmask_perclow);
    IDtmp = create_2Dimage_ID("_tmpwfsmap", sizexWFS, sizeyWFS);
    for(ii=0; ii<sizexWFS*sizeyWFS; ii++)
        data.image[IDtmp].array.F[ii] = data.image[IDWFSmap].array.F[ii] - lim0;
    lim = wfsmask_coeffhigh*img_percentile("_tmpwfsmap", wfsmask_perchigh);

    for(ii=0; ii<sizeWFS; ii++)
    {
        if(data.image[IDWFSmap].array.F[ii]<lim)
            data.image[IDWFSmask].array.F[ii] = 0.0;
        else
            data.image[IDWFSmask].array.F[ii] = 1.0;
    }
    delete_image_ID("_tmpwfsmap");
    printf("done\n");
    fflush(stdout);

    return(0);
}

