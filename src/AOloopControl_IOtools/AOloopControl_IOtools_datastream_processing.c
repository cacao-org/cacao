/**
 * @file    AOloopControl_IOtools_datastream_processing.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    22 Dec 2017
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

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "statistic/statistic.h"
#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "fft/fft.h"

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


//extern long aoloopcontrol_var.AOcontrolNBtimers;           // declared in AOloopControl.c

//extern long aoloopcontrol_var.aoconfID_wfsim;              // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS0;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS0tot;          // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS1;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_wfsdark;            // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_wfsmask;            // declared in AOloopControl.c

//extern uint8_t aoloopcontrol_var.WFSatype;                 // declared in AOloopControl.c

//extern long aoloopcontrol_var.aoconfID_looptiming;         // declared in AOloopControl.c


extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c

static sem_t AOLCOMPUTE_TOTAL_ASYNC_sem_name;

static long long imtotalcnt;
static int AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 0;
static int COMPUTE_DARK_SUBTRACT_NBTHREADS = 1;
static sem_t AOLCOMPUTE_DARK_SUBTRACT_sem_name[32];
static sem_t AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[32];


static int avcamarraysInit = 0;
static unsigned short *arrayutmp;

static char Average_cam_frames_dname[200];
static long Average_cam_frames_IDdark = -1;
static long Average_cam_frames_nelem = 1;


static float *arrayftmp;


// TIMING
static struct timespec tnow;
static struct timespec tdiff;
static double tdiffv;

//extern int aoloopcontrol_var.PIXSTREAM_SLICE;

static long ti; // thread index

static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;
static int AOLCOMPUTE_TOTAL_INIT = 0; // toggles to 1 AFTER total for first image is computed


//extern float aoloopcontrol_var.normfloorcoeff;


//extern float aoloopcontrol_var.GPU_alpha;
//extern float aoloopcontrol_var.GPU_beta;








/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c





/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING      
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */

/**
 * ## Purpose
 * 
 * Averages input image stream
 * 
 * ## Arguments
 * 
 * @param[in]
 * IDname	CHAR*
 * 			Input stream name
 * 
 * @param[in]
 * alpha	DOUBLE
 * 			Averaging coefficient
 * 			new average = old average * (1-alpha) + alpha * new image
 * 
 * @param[out]
 * fIDname_out_ave	CHAR*
 * 			Stream name for output average image
 * 
 * @param[in]
 * IDname_out_AC	CHAR*
 * 			Stream name for output AC component (average-subtracted)
 * 
 * @param[in]
 * IDname_out_RMS	CHAR*
 * 			Stream name for output RMS component
 * 
 * 
 * 
 */

int_fast8_t AOloopControl_IOtools_AveStream(const char *IDname, double alpha, const char *IDname_out_ave, const char *IDname_out_AC, const char *IDname_out_RMS)
{
    long IDin;
    long IDout_ave;
    long IDout_AC, IDout_RMS;
    long xsize, ysize;
    uint32_t *sizearray;
    long cnt0old = 0;
    long delayus = 10;




    IDin = image_ID(IDname);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize;
    sizearray[1] = ysize;

    IDout_ave = create_image_ID(IDname_out_ave, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDname_out_ave, 10);

    IDout_AC = create_image_ID(IDname_out_AC, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDname_out_ave, 10);

    IDout_RMS = create_image_ID(IDname_out_RMS, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDname_out_RMS, 10);


    free(sizearray);

    for(;;)
    {
        if(data.image[IDin].md[0].cnt0 != cnt0old)
        {
            data.image[IDout_ave].md[0].write = 1;
            data.image[IDout_AC].md[0].write = 1;
            data.image[IDout_RMS].md[0].write = 1;
            uint_fast64_t ii;
            for(ii=0; ii<xsize*ysize; ii++)
            {
                data.image[IDout_ave].array.F[ii] = (1.0-alpha)*data.image[IDout_ave].array.F[ii] + alpha * data.image[IDin].array.F[ii];
                data.image[IDout_RMS].array.F[ii] = (1.0-alpha)*data.image[IDout_RMS].array.F[ii] + alpha * (data.image[IDin].array.F[ii] - data.image[IDout_ave].array.F[ii]) * (data.image[IDin].array.F[ii] - data.image[IDout_ave].array.F[ii]);
                data.image[IDout_AC].array.F[ii] = data.image[IDin].array.F[ii] - data.image[IDout_ave].array.F[ii];
            }
            data.image[IDout_ave].md[0].cnt0++;
            data.image[IDout_AC].md[0].cnt0++;
            data.image[IDout_RMS].md[0].cnt0++;
            data.image[IDout_ave].md[0].write = 0;
            data.image[IDout_AC].md[0].write = 0;
            data.image[IDout_RMS].md[0].write = 0;
            cnt0old = data.image[IDin].md[0].cnt0;
        }
        usleep(delayus);
    }


    return(0);
}






/**
 * ## Purpose
 * 
 * Align image stream in real-time\n
 * 
 * ## Arguments
 * 
 * IDname is input stream \n
 * The alignment is computed using a rectangular box of starting at (xbox0,ybox0)\n
 * Reference image used for alignment is provided by IDref_name\n
 * 
 * 
 * ## Use
 * 
 * Function runs a loop. Reacts to updates to stream IDname\n
 * 
 * 
 * ## Details
 * 
 * 
 * @return number of iteration [int]
 * 
 * 
 * 
 * 
 * \ingroup RTfunctions
 */


int_fast8_t AOloopControl_IOtools_imAlignStream(
    const char    *IDname,
    int      xbox0,
    int      ybox0,
    const char    *IDref_name,
    const char    *IDout_name,
    int      insem
)
{
    uint32_t IDin, IDref, IDtmp;
    uint32_t xboxsize, yboxsize;
    uint32_t xsize, ysize;
    long cnt;

    long IDdark;

    float xoffset, yoffset;

    IDin = image_ID(IDname);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];

    IDref = image_ID(IDref_name);
    xboxsize = data.image[IDref].md[0].size[0];
    yboxsize = data.image[IDref].md[0].size[1];

    // is there a dark ?
    IDdark = image_ID("dark");


    IDtmp = create_2Dimage_ID("imAlign_tmp", xboxsize, yboxsize);

    // dark-subtracted full frame image
    uint32_t IDin1;
    IDin1 = create_2Dimage_ID("alignintmpim", xsize, ysize);

    uint8_t datatype;
    datatype = data.image[IDin].md[0].datatype;

	// create output stream
	uint32_t IDout;
	uint32_t *sizearray;
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize;
    sizearray[1] = ysize;
    IDout = create_image_ID(IDout_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDout_name, 10);
    free(sizearray);



    for(;;)
    {
        if(IDdark != -1)
        {
            if(datatype == _DATATYPE_FLOAT)
            {
                long ii;
                for(ii=0; ii<xsize*ysize; ii++)
                    data.image[IDin1].array.F[ii] = data.image[IDin].array.F[ii] - data.image[IDdark].array.F[ii];
            }
            if(datatype == _DATATYPE_INT16)
            {
                long ii;
                for(ii=0; ii<xsize*ysize; ii++)
                    data.image[IDin1].array.F[ii] = 1.0*data.image[IDin].array.SI16[ii] - data.image[IDdark].array.F[ii];
            }
        }
        else
        {
            if(datatype == _DATATYPE_FLOAT)
            {
                long ii;
                for(ii=0; ii<xsize*ysize; ii++)
                    data.image[IDin1].array.F[ii] = data.image[IDin].array.F[ii];
            }
            if(datatype == _DATATYPE_INT16)
            {
                long ii;
                for(ii=0; ii<xsize*ysize; ii++)
                    data.image[IDin1].array.F[ii] = 1.0*data.image[IDin].array.SI16[ii];
            }
        }



        if(data.image[IDin].md[0].sem==0)
        {
            while(cnt==data.image[IDin].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDin].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDin].semptr[insem]);




        // copy box into tmp image
        long ii, jj;

        for(ii=0; ii<xboxsize; ii++)
            for(jj=0; jj<yboxsize; jj++)
            {
                long ii1, jj1;

                ii1 = ii + xbox0;
                jj1 = jj + ybox0;
                data.image[IDtmp].array.F[jj*xboxsize+ii] = 1.0*data.image[IDin1].array.F[jj1*xsize+ii1];
            }


        // compute cross correlation
        fft_correlation("imAlign_tmp", IDref_name, "tmpCorr");

        // find the correlation peak
        float vmax = 0.0;
        long ID;
        long xoffset0, yoffset0;
        ID = image_ID("tmpCorr");
        for(ii=0; ii<xboxsize; ii++)
            for(jj=0; jj<yboxsize; jj++)
            {
                if(data.image[ID].array.F[jj*xboxsize+ii] > vmax)
                {
                    vmax = data.image[ID].array.F[jj*xboxsize+ii];
                    xoffset0 = ii;
                    yoffset0 = jj;
                }
            }

        xoffset = 1.0*xoffset0;
        yoffset = 1.0*yoffset0;
        float krad;
        krad = 0.2*xboxsize;
        float krad2;
        krad2 = krad*krad;

        int kiter;
        int NBkiter = 3;
        for(kiter=0; kiter<NBkiter; kiter++)
        {
            double tmpxs = 0.0;
            double tmpys = 0.0;
            double tmps = 0.0;
            for(ii=0; ii<xboxsize; ii++)
                for(jj=0; jj<yboxsize; jj++)
                {
                    float dx, dy, dx2, dy2;
                    float kcoeff;

                    dx = 1.0*ii - xoffset;
                    dy = 1.0*jj - yoffset;
                    dx2 = dx*dx;
                    dy2 = dy*dy;
                    kcoeff = exp(-(dx2+dy2)/krad2);

                    tmpxs += kcoeff*ii*data.image[ID].array.F[jj*xboxsize+ii];
                    tmpys += kcoeff*jj*data.image[ID].array.F[jj*xboxsize+ii];
                    tmps += kcoeff*data.image[ID].array.F[jj*xboxsize+ii];
                }
            xoffset = tmpxs/tmps;
            yoffset = tmpys/tmps;
          //  printf("%2d center = %4.2f %4.2f\n", kiter, xoffset, yoffset);
        }
        delete_image_ID("tmpCorr");


        xoffset = - (xoffset - 0.5*xboxsize);
        yoffset = - (yoffset - 0.5*yboxsize);

       // printf("offset = %4.2f %4.2f\n", xoffset, yoffset);
       // fflush(stdout);

        fft_image_translate("alignintmpim", "alignouttmp", xoffset, yoffset);
        
        // write to IDout
        long IDouttmp;
        long framesize = sizeof(float)*xsize*ysize;
        IDouttmp = image_ID("alignouttmp");
        data.image[IDout].md[0].write = 1;
        memcpy(data.image[IDout].array.F, data.image[IDouttmp].array.F, framesize);
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].write = 0;


        delete_image_ID("alignouttmp");
    }
    delete_image_ID("alignintmpim");
    delete_image_ID("imAlign_tmp");

    return 0;
}







long AOloopControl_IOtools_frameDelay(
	const char *IDin_name, 
	const char *IDkern_name, 
	const char *IDout_name, 
	int insem
	)
{
    long IDout;
    long IDin;
    long IDkern;
    long ksize;
    long IDbuff;
    long xsize, ysize;
    long kindex = 0;
    long cnt;
    long framesize;
    long IDtmp;
    long xysize;
    float eps=1.0e-8;
    long ii, jj, kk, k1;
    uint32_t *sizearray;



    IDin = image_ID(IDin_name);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
    IDtmp = create_2Dimage_ID("_tmpfr", xsize, ysize);
    xysize = xsize*ysize;

    printf("xsize = %ld\n", xsize);
    printf("ysize = %ld\n", ysize);
    fflush(stdout);





    IDkern = image_ID(IDkern_name);
    ksize = data.image[IDkern].md[0].size[0];
    printf("ksize = %ld\n", ksize);
    fflush(stdout);


    IDbuff = create_3Dimage_ID("_tmpbuff", xsize, ysize, ksize);

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize;
    sizearray[1] = ysize;
    IDout = create_image_ID(IDout_name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
    COREMOD_MEMORY_image_set_createsem(IDout_name, 10);
    free(sizearray);


    framesize = sizeof(float)*xsize*ysize;


    kindex = 0;
    cnt = 0;

    for(;;)
    {
        if(data.image[IDin].md[0].sem==0)
        {
            while(cnt==data.image[IDin].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDin].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDin].semptr[insem]);

        char *ptr0;

        ptr0 = (char*) data.image[IDbuff].array.F;
        ptr0 += kindex*framesize;

        data.image[IDbuff].md[0].write = 1;
        memcpy((void*) ptr0, data.image[IDin].array.F, framesize);
        data.image[IDbuff].md[0].cnt0++;
        data.image[IDbuff].md[0].write = 0;


        data.image[IDout].md[0].write = 1;

        for(ii=0; ii<xysize; ii++)
            data.image[IDtmp].array.F[ii] = 0.0;
        for(kk=0; kk<ksize; kk++)
        {
            if(fabs(data.image[IDkern].array.F[kk])>eps)
            {
                k1 = kindex-kk;
                if(k1<0)
                    k1 += ksize;

                for(ii=0; ii<xysize; ii++)
                    data.image[IDtmp].array.F[ii] += data.image[IDkern].array.F[kk] * data.image[IDbuff].array.F[k1*xysize + ii];
            }
        }
        memcpy(data.image[IDout].array.F, data.image[IDtmp].array.F, framesize);
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].write = 0;


        kindex++;
        if(kindex == ksize)
            kindex = 0;
    }


    return IDout;
}




long AOloopControl_IOtools_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem)
{
    long IDin, IDout;
    uint_fast16_t xsize0, ysize0, zsize0;
    uint_fast32_t xysize0;
    uint_fast16_t xsize1, ysize1;
    uint_fast16_t ii0, jj0, kk0, ii1, jj1, kk;
    uint_fast16_t Xindex, Yindex;
    uint_fast16_t iioffset, jjoffset;
    long long cnt;
    uint32_t *sizearray;
    uint8_t datatype;
    char out0name[200]; // noise-free image, in contrast
    long IDout0;

    float ContrastCoeff = 0.0379;
    float Flux = 5.22e8; // flux per spectral channel [NBph]
    // spectral channel 1% broad = 0.00638 um
    // mR = 5
    // 6m radius disk (12m diam)





    Flux *= 0.4; // efficiency
    Flux *= 3600.0; // second -> hr



    IDin = image_ID(in_name);
    xsize0 = data.image[IDin].md[0].size[0];
    ysize0 = data.image[IDin].md[0].size[1];
    zsize0 = data.image[IDin].md[0].size[2];
    xysize0 = xsize0*ysize0;

    xsize1 = xsize0*NBcols;
    ysize1 = ysize0*(1 + (long) (1.0*zsize0/NBcols-0.00001));

    if(sprintf(out0name, "%sc", out_name) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    datatype = _DATATYPE_FLOAT;
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize1;
    sizearray[1] = ysize1;
    IDout = create_image_ID(out_name, 2, sizearray, datatype, 1, 0);
    IDout0 = create_image_ID(out0name, 2, sizearray, datatype, 1, 0);
    free(sizearray);

    for(;;)
    {
        if(data.image[IDin].md[0].sem==0)
        {
            while(cnt==data.image[IDin].md[0].cnt0) // test if new frame exists
                usleep(5);
            cnt = data.image[IDin].md[0].cnt0;
        }
        else
            sem_wait(data.image[IDin].semptr[insem]);


        printf("Updating image %s ...", out_name);
        fflush(stdout);

        data.image[IDout].md[0].write = 1;

        for(kk0=0; kk0<zsize0; kk0++)
        {
            kk = 0;
            Xindex = 0;
            Yindex = 0;
            while(kk<kk0)
            {
                Xindex++;
                if(Xindex==NBcols)
                {
                    Xindex = 0;
                    Yindex++;
                }
                kk++;
            }
            iioffset = Xindex * xsize0;
            jjoffset = Yindex * ysize0;

            for(ii0=0; ii0<xsize0; ii0++)
                for(jj0=0; jj0<ysize0; jj0++)
                {
                    ii1 = ii0+iioffset;
                    jj1 = jj0+jjoffset;
                    //data.image[IDout].array.F[jj1*xsize1+ii1] = data.image[IDin].array.F[kk0*xysize0+jj0*xsize0+ii0]/ContrastCoeff;
                    data.image[IDout].array.F[jj1*xsize1+ii1] = poisson(data.image[IDin].array.F[kk0*xysize0+jj0*xsize0+ii0]*Flux)/Flux/ContrastCoeff;

                    data.image[IDout0].array.F[jj1*xsize1+ii1] = data.image[IDin].array.F[kk0*xysize0+jj0*xsize0+ii0]/ContrastCoeff;
                }
        }
        COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
        data.image[IDout].md[0].cnt0++;
        data.image[IDout].md[0].write = 0;

        printf("done\n");
        fflush(stdout);
    }

    return(IDout);
}
