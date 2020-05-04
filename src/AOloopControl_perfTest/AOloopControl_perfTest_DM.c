/**
 * @file    AOloopControl_perfTest_DM.c
 * @brief   Adaptive Optics Control loop engine testing
 *
 * AO engine uses stream data structure
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
#include <string.h>
#include <math.h>
#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "linopt_imtools/linopt_imtools.h"
#include "info/info.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"


/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */



# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */



//static int wcol, wrow; // window size

// TIMING
static struct timespec tnow;
static struct timespec tdiff;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c






errno_t AOcontrolLoop_perfTest_TestDMSpeed(
    const char *dmname,
    long        delayus,
    long        NBpts,
    float       ampl
)
{
    imageID IDdm;
    uint32_t dmxsize;
    uint32_t dmysize;
    uint64_t dmsize;
    imageID ID1;
    float x, y, x1;
    char *ptr;

//    imageID IDdm0;
//    imageID IDdm1; // DM shapes



    IDdm    = image_ID(dmname);
    dmxsize = data.image[IDdm].md[0].size[0];
    dmysize = data.image[IDdm].md[0].size[1];
    dmsize  = dmxsize * dmysize;



    ID1 = create_3Dimage_ID("dmpokeseq", dmxsize, dmysize, NBpts);
    for(uint32_t kk = 0; kk < NBpts; kk++)
    {
        float pha;

        pha = 2.0 * M_PI * kk / NBpts;
        for(uint32_t ii = 0; ii < dmxsize; ii++)
            for(uint32_t jj = 0; jj < dmysize; jj++)
            {
                x = (2.0 * ii / dmxsize) - 1.0;
                y = (2.0 * jj / dmysize) - 1.0;
                x1 = x * cos(pha) - y * sin(pha);
                data.image[ID1].array.F[kk * dmsize + jj * dmxsize + ii] = ampl * x1;
            }
    }

    while(1)
    {
        for(uint32_t kk = 0; kk < NBpts; kk++)
        {
            ptr = (char *) data.image[ID1].array.F;
            ptr += sizeof(float) * dmsize * kk;
            data.image[IDdm].md[0].write = 1;
            memcpy(data.image[IDdm].array.F, ptr, sizeof(float)*dmsize);
            data.image[IDdm].md[0].write = 0;
            data.image[IDdm].md[0].cnt0 ++;
            usleep(delayus);
        }
    }

    return RETURN_SUCCESS;
}


//
// Measures mode temporal response (measurement and rejection)
//
imageID AOloopControl_perfTest_TestDMmodeResp(
    const char *DMmodes_name,
    long index,
    float ampl,
    float fmin,
    float fmax,
    float fmultstep,
    float avetime,
    long  dtus,
    const char *DMmask_name,
    const char *DMstream_in_name,
    const char *DMstream_out_name,
    const char *IDout_name
)
{
    imageID IDout;
    imageID IDmodes, IDdmmask, IDdmin, IDdmout;
    long dmxsize, dmysize, dmsize, NBmodes;
    float f;
    struct timespec tstart;
    long nbf;
    long IDrec_dmout;
    long ii, kk, kmax;
    imageID IDdmtmp;
    float pha, coeff;
    float *timearray;
    char *ptr;
    long IDcoeff;
    float SVDeps = 1.0e-3;
    int SVDreuse = 0;
    long IDcoeffarray;
    long m;
    FILE *fp;
    char fname[STRINGMAXLEN_FILENAME];
    long kmaxmax = 100000;
    imageID ID;


    kk = index;

    IDmodes = image_ID(DMmodes_name);
    IDdmin = image_ID(DMstream_in_name);
    IDdmout = image_ID(DMstream_out_name);
    IDdmmask = image_ID(DMmask_name);

    dmxsize = data.image[IDmodes].md[0].size[0];
    dmysize = data.image[IDmodes].md[0].size[1];
    dmsize = dmxsize * dmysize;
    NBmodes = data.image[IDmodes].md[0].size[2];


    if(data.image[IDdmin].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMstream_in_name, (long) data.image[IDdmin].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmin].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMstream_in_name, (long) data.image[IDdmin].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMstream_out_name, (long) data.image[IDdmout].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMstream_out_name, (long) data.image[IDdmout].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMmask_name, (long) data.image[IDdmmask].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMmask_name, (long) data.image[IDdmmask].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }



    nbf = 0;
    for(f = fmin; f < fmax; f *= fmultstep)
    {
        nbf++;
    }



    // TEST
    // Save DM mode
    ID = create_2Dimage_ID("testmrespm", dmxsize, dmysize);
    for(ii = 0; ii < dmsize; ii++)
    {
        data.image[ID].array.F[ii] = data.image[IDmodes].array.F[kk * dmsize + ii];
    }
    save_fits("testmrespm", "!testmrespm.fits");


    // SET UP RECORDING CUBES
    kmax = (long)(avetime / (1.0e-6 * dtus));
    if(kmax > kmaxmax)
    {
        kmax = kmaxmax;
    }

    timearray = (float *) malloc(sizeof(float) * kmax);
    IDrec_dmout = create_3Dimage_ID("_tmprecdmout", dmxsize, dmysize, kmax);

    IDcoeffarray = create_2Dimage_ID("_tmpcoeffarray", kmax, NBmodes);

    WRITE_FILENAME(fname, "mode%03ld_PSD.txt", kk);

    if((fp = fopen(fname, "w")) == NULL)
    {
        printf("ERROR: cannot create file \"%s\"", fname);
        exit(0);
    }

    IDout = create_2Dimage_ID(IDout_name, nbf, NBmodes);
    IDdmtmp = create_2Dimage_ID("_tmpdm", dmxsize, dmysize);

    for(f = fmin; f < fmax; f *= fmultstep)
    {
        float runtime = 0.0;
        long k = 0;
        long k1;
        float coscoeff, sincoeff;
        float PSDamp, PSDpha;




        clock_gettime(CLOCK_REALTIME, &tstart);
        while((runtime < avetime) && (k < kmax))
        {
            clock_gettime(CLOCK_REALTIME, &tnow);
            runtime = timespec_diff_double(tstart, tnow);
            pha = 2.0 * M_PI * runtime * f;
            coeff = ampl * cos(pha);

            printf("mode %4ld  f = %f ( %f -> %f)   runtime = %10.3f sec    ampl = %f   pha = %f   coeff = %f\n",
                   kk, f, fmin, fmax, runtime, ampl, pha, coeff);
            fflush(stdout);

            // APPLY MODE TO DM
            data.image[IDdmin].md[0].write = 1;
            for(ii = 0; ii < dmsize; ii++)
            {
                data.image[IDdmin].array.F[ii] = coeff * data.image[IDmodes].array.F[kk * dmsize
                                                 + ii];
            }
            data.image[IDdmin].md[0].cnt0++;
            data.image[IDdmin].md[0].write = 0;

            // RECORD
            ptr = (char *) data.image[IDrec_dmout].array.F;
            ptr += sizeof(float) * k * dmsize;
            memcpy(ptr, data.image[IDdmout].array.F, sizeof(float)*dmsize); //out->in

            timearray[k] = runtime;

            usleep(dtus);
            k++;
        }

        // ZERO DM
        data.image[IDdmin].md[0].write = 1;
        for(ii = 0; ii < dmsize; ii++)
        {
            data.image[IDdmin].array.F[ii] = 0.0;
        }
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;


        k1 = k;
        //    save_fits("_tmprecdmout", "!_tmprecdmout.fits");


        printf("\n\n");

        // PROCESS RECORDED DATA
        for(k = 0; k < k1; k++)
        {
            printf("\r  %5ld / %5ld     ", k, k1);
            fflush(stdout);

            ptr = (char *) data.image[IDrec_dmout].array.F;
            ptr += sizeof(float) * k * dmsize;
            memcpy(data.image[IDdmtmp].array.F, ptr, sizeof(float)*dmsize);
            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps,
                                          "dmcoeffs", SVDreuse);
            SVDreuse = 1;
            IDcoeff = image_ID("dmcoeffs");
            for(m = 0; m < NBmodes; m++)
            {
                data.image[IDcoeffarray].array.F[m * kmax + k] =
                    data.image[IDcoeff].array.F[m];
            }
            delete_image_ID("dmcoeffs");

        }
        printf("\n\n");

        save_fits("_tmpcoeffarray", "!_tmpcoeffarray.fits");

        // EXTRACT AMPLITUDE AND PHASE
        coscoeff = 0.0;
        sincoeff = 0.0;
        for(k = k1 / 4; k < k1; k++)
        {
            pha = 2.0 * M_PI * timearray[k] * f;
            coscoeff += cos(pha) * data.image[IDcoeffarray].array.F[kk * kmax + k];
            sincoeff += sin(pha) * data.image[IDcoeffarray].array.F[kk * kmax + k];
        }
        coscoeff /= (0.5 * k1 * 0.75);
        sincoeff /= (0.5 * k1 * 0.75);

        PSDamp = coscoeff * coscoeff + sincoeff * sincoeff;
        PSDpha = atan2(-sincoeff, -coscoeff);
        fp = fopen(fname, "a");
        fprintf(fp, "    %20f %20.18f %20f\n", f, sqrt(PSDamp) / ampl, PSDpha);
        fclose(fp);
    }

    delete_image_ID("_tmpdm");

    free(timearray);


    return(IDout);
}





imageID AOloopControl_perfTest_TestDMmodes_Recovery(
    const char *DMmodes_name,
    float ampl,
    const char *DMmask_name,
    const char *DMstream_in_name,
    const char *DMstream_out_name,
    const char *DMstream_meas_name,
    long tlagus,
    long NBave,
    const char *IDout_name,
    const char *IDoutrms_name,
    const char *IDoutmeas_name,
    const char *IDoutmeasrms_name
)
{
    imageID  IDout, IDoutrms, IDoutmeas, IDoutmeasrms;
    imageID  IDmodes, IDdmmask, IDdmin, IDmeasout, IDdmout;
    uint32_t dmxsize, dmysize;
    uint64_t dmsize;
    uint32_t NBmodes;

    imageID IDdmtmp, IDmeastmp;
    int     SVDreuse = 0;
    float   SVDeps = 1.0e-3;
    imageID IDcoeffarray;
    imageID IDcoeffarraymeas;
    imageID IDcoeff;



    IDmodes = image_ID(DMmodes_name);
    IDdmin = image_ID(DMstream_in_name);
    IDdmout = image_ID(DMstream_out_name);
    IDmeasout = image_ID(DMstream_meas_name);
    IDdmmask = image_ID(DMmask_name);

    dmxsize = data.image[IDmodes].md[0].size[0];
    dmysize = data.image[IDmodes].md[0].size[1];
    dmsize = dmxsize * dmysize;
    NBmodes = data.image[IDmodes].md[0].size[2];


    //
    // CHECK IMAGE SIZES
    //
    if(data.image[IDdmin].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: x size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMstream_in_name, (long) data.image[IDdmin].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmin].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMstream_in_name, (long) data.image[IDdmin].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMstream_out_name, (long) data.image[IDdmout].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmout].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMstream_out_name, (long) data.image[IDdmout].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDmeasout].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMstream_meas_name, (long) data.image[IDmeasout].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDmeasout].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMstream_meas_name, (long) data.image[IDmeasout].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[0] != data.image[IDmodes].md[0].size[0])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match x size of \"%s\" (%ld)\n",
               DMmask_name, (long) data.image[IDdmmask].md[0].size[0], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[0]);
        exit(0);
    }

    if(data.image[IDdmmask].md[0].size[1] != data.image[IDmodes].md[0].size[1])
    {
        printf("ERROR: y size of \"%s\"  (%ld) does not match y size of \"%s\" (%ld)\n",
               DMmask_name, (long) data.image[IDdmmask].md[0].size[1], DMmodes_name,
               (long) data.image[IDmodes].md[0].size[1]);
        exit(0);
    }


    IDout = create_2Dimage_ID(IDout_name, NBmodes, NBmodes);
    IDoutrms = create_2Dimage_ID(IDoutrms_name, NBmodes, NBmodes);
    IDoutmeas = create_2Dimage_ID(IDoutmeas_name, NBmodes, NBmodes);
    IDoutmeasrms = create_2Dimage_ID(IDoutmeasrms_name, NBmodes, NBmodes);
    IDdmtmp = create_2Dimage_ID("_tmpdm", dmxsize, dmysize);
    IDmeastmp = create_2Dimage_ID("_tmpmeas", dmxsize, dmysize);

    IDcoeffarray = create_2Dimage_ID("_coeffarray", NBmodes, NBave);
    IDcoeffarraymeas = create_2Dimage_ID("_coeffarraymeas", NBmodes, NBave);

    printf("Initialize SVD ... ");
    fflush(stdout);
    linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps,
                                  "dmcoeffs", SVDreuse);
    SVDreuse = 1;
    printf("done\n");
    fflush(stdout);

    printf("\n\n");

    for(uint32_t kk = 0; kk < NBmodes; kk++)
    {
        uint64_t cntdmout;
        long i;

        printf("\r Mode %5u / %5u       ", kk, NBmodes);
        fflush(stdout);

        // APPLY MODE TO DM
        data.image[IDdmin].md[0].write = 1;
        for(uint64_t ii = 0; ii < dmsize; ii++)
        {
            data.image[IDdmin].array.F[ii] = ampl * data.image[IDmodes].array.F[kk * dmsize
                                             + ii];
        }
        COREMOD_MEMORY_image_set_sempost_byID(IDdmin, -1);
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;

        // WAIT
        usleep(tlagus);


        // RECORD DM SHAPES INTO MODES

        // POSITIVE
        cntdmout = 0;
        i = 0;
        while(i < NBave)
        {
            while(cntdmout == data.image[IDdmout].md[0].cnt0)
            {
                usleep(20);
            }

            cntdmout =  data.image[IDdmout].md[0].cnt0;


            memcpy(data.image[IDdmtmp].array.F, data.image[IDdmout].array.F,
                   sizeof(float)*dmsize);
            memcpy(data.image[IDmeastmp].array.F, data.image[IDmeasout].array.F,
                   sizeof(float)*dmsize);

            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps,
                                          "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
            {
                data.image[IDcoeffarray].array.F[kk1 * NBave + i] = 0.5 *
                        data.image[IDcoeff].array.F[kk1];
            }
            delete_image_ID("dmcoeffs");

            linopt_imtools_image_fitModes("_tmpmeas", DMmodes_name, DMmask_name, SVDeps,
                                          "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
            {
                data.image[IDcoeffarraymeas].array.F[kk1 * NBave + i] = 0.5 *
                        data.image[IDcoeff].array.F[kk1];
            }
            delete_image_ID("dmcoeffs");


            i++;
        }

        // NEGATIVE

        // APPLY MODE TO DM
        data.image[IDdmin].md[0].write = 1;
        for(uint64_t ii = 0; ii < dmsize; ii++)
        {
            data.image[IDdmin].array.F[ii] = -ampl * data.image[IDmodes].array.F[kk * dmsize
                                             + ii];
        }
        COREMOD_MEMORY_image_set_sempost_byID(IDdmin, -1);
        data.image[IDdmin].md[0].cnt0++;
        data.image[IDdmin].md[0].write = 0;

        // WAIT
        usleep(tlagus);

        cntdmout = 0;
        i = 0;
        while(i < NBave)
        {
            while(cntdmout == data.image[IDdmout].md[0].cnt0)
            {
                usleep(20);
            }

            cntdmout =  data.image[IDdmout].md[0].cnt0;

            memcpy(data.image[IDdmtmp].array.F, data.image[IDdmout].array.F,
                   sizeof(float)*dmsize);
            memcpy(data.image[IDmeastmp].array.F, data.image[IDmeasout].array.F,
                   sizeof(float)*dmsize);

            // decompose in modes
            linopt_imtools_image_fitModes("_tmpdm", DMmodes_name, DMmask_name, SVDeps,
                                          "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
            {
                data.image[IDcoeffarray].array.F[kk1 * NBave + i] -= 0.5 *
                        data.image[IDcoeff].array.F[kk1];
            }
            delete_image_ID("dmcoeffs");
            i++;

            linopt_imtools_image_fitModes("_tmpmeas", DMmodes_name, DMmask_name, SVDeps,
                                          "dmcoeffs", SVDreuse);
            IDcoeff = image_ID("dmcoeffs");
            for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
            {
                data.image[IDcoeffarraymeas].array.F[kk1 * NBave + i] = 0.5 *
                        data.image[IDcoeff].array.F[kk1];
            }
            delete_image_ID("dmcoeffs");
        }


        // PROCESSS

        for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
        {
            data.image[IDout].array.F[kk1 * NBmodes + kk] = 0.0;
            data.image[IDoutrms].array.F[kk1 * NBmodes + kk] = 0.0;
            data.image[IDoutmeas].array.F[kk1 * NBmodes + kk] = 0.0;
            data.image[IDoutmeasrms].array.F[kk1 * NBmodes + kk] = 0.0;
        }
        for(uint32_t kk1 = 0; kk1 < NBmodes; kk1++)
        {
            for(uint32_t i = 0; i < NBave; i++)
            {
                data.image[IDout].array.F[kk1 * NBmodes + kk] +=
                    data.image[IDcoeffarray].array.F[kk1 * NBave + i];
                data.image[IDoutrms].array.F[kk1 * NBmodes + kk] +=
                    data.image[IDcoeffarray].array.F[kk1 * NBave + i] *
                    data.image[IDcoeffarray].array.F[kk1 * NBave + i];
                data.image[IDoutmeas].array.F[kk1 * NBmodes + kk] +=
                    data.image[IDcoeffarraymeas].array.F[kk1 * NBave + i];
                data.image[IDoutmeasrms].array.F[kk1 * NBmodes + kk] +=
                    data.image[IDcoeffarraymeas].array.F[kk1 * NBave + i] *
                    data.image[IDcoeffarraymeas].array.F[kk1 * NBave + i];
            }
            data.image[IDout].array.F[kk1 * NBmodes + kk] /= NBave * ampl;
            data.image[IDoutrms].array.F[kk1 * NBmodes + kk] = sqrt(
                        data.image[IDoutrms].array.F[kk1 * NBmodes + kk] / NBave);
            data.image[IDoutmeas].array.F[kk1 * NBmodes + kk] /= NBave * ampl;
            data.image[IDoutmeasrms].array.F[kk1 * NBmodes + kk] = sqrt(
                        data.image[IDoutmeasrms].array.F[kk1 * NBmodes + kk] / NBave);
        }
    }
    printf("\n\n");
    fflush(stdout);

    delete_image_ID("_tmpdm");
    delete_image_ID("_tmpmeas");
    delete_image_ID("_coeffarray");



    return IDout;
}
