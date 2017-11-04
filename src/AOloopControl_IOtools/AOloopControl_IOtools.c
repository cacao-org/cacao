/**
 * @file    AOloopControl_IOtools.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    22 Aug 2017
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



#include <fitsio.h>

#include "CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"

#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"



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


extern long AOcontrolNBtimers;           // declared in AOloopControl.c

extern long aoconfID_wfsim;              // declared in AOloopControl.c
extern long aoconfID_imWFS0;             // declared in AOloopControl.c
extern long aoconfID_imWFS0tot;          // declared in AOloopControl.c
extern long aoconfID_imWFS1;             // declared in AOloopControl.c
extern long aoconfID_wfsdark;            // declared in AOloopControl.c
extern long aoconfID_wfsmask;            // declared in AOloopControl.c

extern uint8_t WFSatype;                 // declared in AOloopControl.c

extern long aoconfID_looptiming;         // declared in AOloopControl.c



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

extern int PIXSTREAM_SLICE;

static long ti; // thread index

static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;
static int AOLCOMPUTE_TOTAL_INIT = 0; // toggles to 1 AFTER total for first image is computed


extern float normfloorcoeff;


extern float GPU_alpha;
extern float GPU_beta;








/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c















// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_camimage_extract2D_sharedmem_loop */
int_fast8_t AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,5)+CLI_checkarg(3,3)+CLI_checkarg(4,2)+CLI_checkarg(5,2)+CLI_checkarg(6,2)+CLI_checkarg(7,2)==0) {
        AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string , data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numl, data.cmdargtoken[7].val.numl);
        return 0;
    }
    else return 1;
}




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 2. LOAD DATA STREAMS     
 *  Load 2D and 3D shared memory images */
/* =============================================================================================== */
/* =============================================================================================== */

// No command line hooks to functions in this section


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING      
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_AveStream */
int_fast8_t AOloopControl_IOtools_AveStream_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,3)==0) {
        AOloopControl_IOtools_AveStream(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string);
        return 0;
    }
    else return 1;
}


/** @brief CLI function for AOloopControl_frameDelay */
int_fast8_t AOloopControl_IOtools_frameDelay_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,5)+CLI_checkarg(4,2)==0)    {
        AOloopControl_IOtools_frameDelay(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else        return 1;
}


/** @brief CLI function for AOloopControl_stream3Dto2D */
int_fast8_t AOloopControl_IOtools_stream3Dto2D_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0) {
        AOloopControl_IOtools_stream3Dto2D(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else return 1;
}















/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools functions */


void __attribute__ ((constructor)) libinit_AOloopControl_IOtools()
{
	init_AOloopControl_IOtools();
	printf(" ...... Loading module %s\n", __FILE__);
}


int_fast8_t init_AOloopControl_IOtools()
{

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "AO loop control IO tools");
    data.NBmodule++;



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */

    RegisterCLIcommand("cropshim", __FILE__, AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli, "crop shared mem image", "<input image> <optional dark> <output image> <sizex> <sizey> <xstart> <ystart>" , "cropshim imin null imout 32 32 153 201", "int AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(char *in_name, const char *dark_name, char *out_name, long size_x, long size_y, long xstart, long ystart)");




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING      
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */


    RegisterCLIcommand("aveACshmim", __FILE__, AOloopControl_IOtools_AveStream_cli, "average and AC shared mem image", "<input image> <coeff> <output image ave> <output AC> <output RMS>" , "aveACshmim imin 0.01 outave outAC outRMS", "int AOloopControl_IOtools_AveStream(char *IDname, double alpha, char *IDname_out_ave, char *IDname_out_AC, char *IDname_out_RMS)");

    RegisterCLIcommand("aolframedelay", __FILE__, AOloopControl_IOtools_frameDelay_cli, "introduce temporal delay", "<in> <temporal kernel> <out> <sem index>","aolframedelay in kern out 0","long AOloopControl_IOtools_frameDelay(const char *IDin_name, const char *IDkern_name, const char *IDout_name, int insem)");

    RegisterCLIcommand("aolstream3Dto2D", __FILE__, AOloopControl_IOtools_stream3Dto2D_cli, "remaps 3D cube into 2D image", "<input 3D stream> <output 2D stream> <# cols> <sem trigger>" , "aolstream3Dto2D in3dim out2dim 4 1", "long AOloopControl_IOtools_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem)");





    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}


















/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */







//
// every time im_name changes (counter increments), crop it to out_name in shared memory
//
int_fast8_t AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(const char *in_name, const char *dark_name, const char *out_name, long size_x, long size_y, long xstart, long ystart)
{
    long iiin,jjin, iiout, jjout;
    long IDin, IDout, IDdark;
    uint8_t atype;
    uint8_t atypeout;
    uint32_t *sizeout;
    long long cnt0;
    long IDmask;
    long sizeoutxy;
    long ii;


    sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizeout[0] = size_x;
    sizeout[1] = size_y;
    sizeoutxy = size_x*size_y;

    IDin = image_ID(in_name);
    atype = data.image[IDin].md[0].atype;



    // Check if there is a mask
    IDmask = image_ID("csmask");
    if(IDmask!=-1)
        if((data.image[IDmask].md[0].size[0]!=size_x)||(data.image[IDmask].md[0].size[1]!=size_y))
        {
            printf("ERROR: csmask has wrong size\n");
            exit(0);
        }

    // Check dark
    IDdark = image_ID(dark_name);

    if(IDdark!=-1)
    {
        if((data.image[IDdark].md[0].size[0]!=data.image[IDin].md[0].size[0])||(data.image[IDdark].md[0].size[1]!=data.image[IDin].md[0].size[1]))
        {
            printf("ERROR: csmask has wrong size\n");
            exit(0);
        }
        if(data.image[IDdark].md[0].atype != _DATATYPE_FLOAT)
        {
            printf("ERROR: csmask has wrong type\n");
            exit(0);
        }
        atypeout = _DATATYPE_FLOAT;
    }
    else
        atypeout = atype;


    // Create shared memory output image
    IDout = create_image_ID(out_name, 2, sizeout, atypeout, 1, 0);

    cnt0 = -1;

    switch (atype) {
    case _DATATYPE_UINT16 :
        while(1)
        {
            usleep(10); // OK FOR NOW (NOT USED BY FAST WFS)
            if(data.image[IDin].md[0].cnt0!=cnt0)
            {
                data.image[IDout].md[0].write = 1;
                cnt0 = data.image[IDin].md[0].cnt0;
                if(atypeout == _DATATYPE_UINT16)
                {
                    for(iiout=0; iiout<size_x; iiout++)
                        for(jjout=0; jjout<size_y; jjout++)
                        {
                            iiin = xstart + iiout;
                            jjin = ystart + jjout;
                            data.image[IDout].array.UI16[jjout*size_x+iiout] = data.image[IDin].array.UI16[jjin*data.image[IDin].md[0].size[0]+iiin];
                        }
                    if(IDmask!=-1)
                        for(ii=0; ii<sizeoutxy; ii++)
                            data.image[IDout].array.UI16[ii] *= (int) data.image[IDmask].array.F[ii];
                }
                else // FLOAT
                {
                    if(IDdark==-1)
                    {
                        for(iiout=0; iiout<size_x; iiout++)
                            for(jjout=0; jjout<size_y; jjout++)
                            {
                                iiin = xstart + iiout;
                                jjin = ystart + jjout;
                                data.image[IDout].array.F[jjout*size_x+iiout] = data.image[IDin].array.UI16[jjin*data.image[IDin].md[0].size[0]+iiin];
                            }
                    }
                    else
                    {
                        for(iiout=0; iiout<size_x; iiout++)
                            for(jjout=0; jjout<size_y; jjout++)
                            {
                                iiin = xstart + iiout;
                                jjin = ystart + jjout;
                                data.image[IDout].array.F[jjout*size_x+iiout] = 1.0*data.image[IDin].array.UI16[jjin*data.image[IDin].md[0].size[0]+iiin] - data.image[IDdark].array.F[jjin*data.image[IDdark].md[0].size[0]+iiin];
                            }
                    }

                    if(IDmask!=-1)
                        for(ii=0; ii<sizeoutxy; ii++)
                            data.image[IDout].array.F[ii] *= data.image[IDmask].array.F[ii];
                }
                data.image[IDout].md[0].cnt0 = cnt0;
                data.image[IDout].md[0].write = 0;
            }
        }
        break;
    case _DATATYPE_FLOAT :
        while(1)
        {
            usleep(50); // OK FOR NOW (NOT USED BY FAST WFS)
            if(data.image[IDin].md[0].cnt0!=cnt0)
            {
                data.image[IDout].md[0].write = 1;
                cnt0 = data.image[IDin].md[0].cnt0;
                if(IDdark==-1)
                {
                    for(iiout=0; iiout<size_x; iiout++)
                        for(jjout=0; jjout<size_y; jjout++)
                        {
                            iiin = xstart + iiout;
                            jjin = ystart + jjout;
                            data.image[IDout].array.F[jjout*size_x+iiout] = data.image[IDin].array.F[jjin*data.image[IDin].md[0].size[0]+iiin];
                        }
                }
                else
                {
                    for(iiout=0; iiout<size_x; iiout++)
                        for(jjout=0; jjout<size_y; jjout++)
                        {
                            iiin = xstart + iiout;
                            jjin = ystart + jjout;
                            data.image[IDout].array.F[jjout*size_x+iiout] = data.image[IDin].array.F[jjin*data.image[IDin].md[0].size[0]+iiin] - data.image[IDdark].array.F[jjin*data.image[IDdark].md[0].size[0]+iiin];
                        }
                }

                if(IDmask!=-1)
                    for(ii=0; ii<sizeoutxy; ii++)
                        data.image[IDout].array.F[ii] *= data.image[IDmask].array.F[ii];

                data.image[IDout].md[0].cnt0 = cnt0;
                data.image[IDout].md[0].write = 0;
            }
        }
        break;
    default :
        printf("ERROR: DATA TYPE NOT SUPPORTED\n");
        exit(0);
        break;
    }
    free(sizeout);

    return(0);
}











static void *compute_function_imtotal( void *ptr )
{
    long ii;
    long nelem;
    int semval;
	float IMTOTAL;
	char imname[200];


	printf("TEST - =========== ENTERING compute_function_imtotal ===================\n");
	fflush(stdout);



	if(aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", AOcontrolNBtimers, 1, 0.0);
	}




    nelem = data.image[aoconfID_imWFS0].md[0].size[0]*data.image[aoconfID_imWFS0].md[0].size[1];

    for(;;)
    {
		#ifdef _PRINT_TEST
		printf("TEST - Waiting for semaphore\n");
		fflush(stdout);
		#endif

        sem_wait(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);

		#ifdef _PRINT_TEST
		printf("TEST - COMPUTING TOTAL FOR IMAGE ID %ld : %s\n", aoconfID_imWFS0, data.image[aoconfID_imWFS0].md[0].name);
		fflush(stdout);
		#endif
	
		imtotalcnt++;
		
        data.image[aoconfID_imWFS0tot].md[0].write = 1;
        IMTOTAL = 0.0;
        if(aoconfID_wfsmask!=-1)
        {
            for(ii=0; ii<nelem; ii++)
                IMTOTAL += data.image[aoconfID_imWFS0].array.F[ii]*data.image[aoconfID_wfsmask].array.F[ii];
        }
        else
        {
            for(ii=0; ii<nelem; ii++)
                IMTOTAL += data.image[aoconfID_imWFS0].array.F[ii];
        }
        data.image[aoconfID_imWFS0tot].array.F[0] = IMTOTAL;
        
        AOconf[LOOPNUMBER].WFStotalflux = IMTOTAL;
        
        
        data.image[aoconfID_imWFS0tot].md[0].cnt0++;
        data.image[aoconfID_imWFS0tot].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
        
        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS0tot, -1);
        data.image[aoconfID_imWFS0tot].md[0].write = 0;
    }

}




static void *compute_function_dark_subtract( void *ptr )
{
    long ii, iistart, iiend;
    long nelem;
    long *index;
    int sval;
    long threadindex;
    int semval;


    nelem = data.image[aoconfID_imWFS0].md[0].size[0]*data.image[aoconfID_imWFS0].md[0].size[1];
    index = (long*) ptr;
    threadindex = *index;

    iistart = (long) ((threadindex)*nelem/COMPUTE_DARK_SUBTRACT_NBTHREADS);
    iiend = (long) ((threadindex+1)*nelem/COMPUTE_DARK_SUBTRACT_NBTHREADS);

    while(1)
    {
        sem_wait(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[threadindex]);

        switch ( WFSatype ) {
        case _DATATYPE_UINT16 :
            for(ii=iistart; ii<iiend; ii++)
                data.image[aoconfID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
            break;
        case _DATATYPE_FLOAT :
            for(ii=iistart; ii<iiend; ii++)
                data.image[aoconfID_imWFS0].array.F[ii] = ((float) arrayftmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
            break;
        default :
            printf("ERROR: WFS data type not recognized\n");
            exit(0);
            break;
        }

        sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[threadindex], &semval);
        if(semval<SEMAPHORE_MAXVAL)
            sem_post(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[threadindex]);
    }

}












/** @brief Read image from WFS camera
 *
 * supports ring buffer
 * puts image from camera buffer aoconfID_wfsim into aoconfID_imWFS1 (supplied by user)
 *
 * RM = 1 if response matrix
 *
 * if normalize == 1, image is normalized by dividing by (total + AOconf[loop].WFSnormfloor)*AOconf[loop].WFSsize
 * if PixelStreamMode = 1, read on semaphore 1, return slice index
 *
 */

int_fast8_t Read_cam_frame(long loop, int RM, int normalize, int PixelStreamMode, int InitSem)
{
    long imcnt;
    long ii;
    double totalinv;
    char name[200];
    int slice;
    char *ptrv;
    long double tmplv1;
    double tmpf;
    long IDdark;
    char dname[200];
    long nelem;
    pthread_t thread_computetotal_id;
    pthread_t thread_dark_subtract[20];
    float resulttotal;
    int sval0, sval;
    void *status = 0;
    long i;
    int semval;
    int s;

    int semindex = 0;


	//usleep(1000000);// TEST


    if(RM==0)
        semindex = 0;
    else
        semindex = 1;


    WFSatype = data.image[aoconfID_wfsim].md[0].atype;

    if(avcamarraysInit==0)
    {
        arrayftmp = (float*) malloc(sizeof(float)*AOconf[loop].sizeWFS);
        arrayutmp = (unsigned short*) malloc(sizeof(unsigned short)*AOconf[loop].sizeWFS);

        if(sprintf(Average_cam_frames_dname, "aol%ld_wfsdark", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        Average_cam_frames_IDdark = image_ID(Average_cam_frames_dname);
        Average_cam_frames_nelem = AOconf[loop].sizeWFS;

        // set semaphore to 0
        sem_getvalue(data.image[aoconfID_wfsim].semptr[semindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n", semindex, data.image[aoconfID_wfsim].md[0].name, semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoconfID_wfsim].semptr[semindex]);

        //PIXSTREAM_SLICE = data.image[aoconfID_wfsim].md[0].cnt1;    // set semaphore 1 to 0

        avcamarraysInit = 1;
    }

    if(InitSem==1)
    {
        sem_getvalue(data.image[aoconfID_wfsim].semptr[semindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n", semindex, data.image[aoconfID_wfsim].md[0].name, semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoconfID_wfsim].semptr[semindex]);
    }

#ifdef _PRINT_TEST
    printf("TEST - SEMAPHORE INITIALIZED\n");
    fflush(stdout);
#endif

    if(RM==0)
    {
        AOconf[loop].status = 20;  // 020: WAIT FOR IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[24] = tdiffv;
    }
    else
        data.status1 = 2;

    //   usleep(20);

#ifdef _PRINT_TEST
    printf("TEST - WAITING FOR IMAGE %s\n", data.image[aoconfID_wfsim].md[0].name);
    fflush(stdout);
#endif

    if(data.image[aoconfID_wfsim].md[0].sem==0)
    {
        if(RM==0)
            while(AOconf[loop].WFScnt==data.image[aoconfID_wfsim].md[0].cnt0) // test if new frame exists
                usleep(5);
        else
            while(AOconf[loop].WFScntRM==data.image[aoconfID_wfsim].md[0].cnt0) // test if new frame exists
                usleep(5);
    }
    else
    {
#ifdef _PRINT_TEST
        printf("TEST - waiting on semindex = %d\n", semindex);
        fflush(stdout);
#endif

        sem_wait(data.image[aoconfID_wfsim].semptr[semindex]);

        sem_getvalue(data.image[aoconfID_wfsim].semptr[semindex], &semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoconfID_wfsim].semptr[semindex]);


#ifdef _PRINT_TEST
        printf("TEST - semaphore posted\n");
        fflush(stdout);
#endif
    }

    if(RM==0)
        AOconf[loop].status = 0;  // LOAD IMAGE

    AOconf[loop].statusM = 0;


    slice = 0;
    if(data.image[aoconfID_wfsim].md[0].naxis==3) // ring buffer
    {
        slice = data.image[aoconfID_wfsim].md[0].cnt1;
        if(slice==-1)
            slice = data.image[aoconfID_wfsim].md[0].size[2];
    }

    switch (WFSatype) {
    case _DATATYPE_FLOAT :
        ptrv = (char*) data.image[aoconfID_wfsim].array.F;
        ptrv += sizeof(float)*slice* AOconf[loop].sizeWFS;
        memcpy(arrayftmp, ptrv,  sizeof(float)*AOconf[loop].sizeWFS);
        break;
    case _DATATYPE_UINT16 :
        ptrv = (char*) data.image[aoconfID_wfsim].array.UI16;
        ptrv += sizeof(unsigned short)*slice* AOconf[loop].sizeWFS;
        memcpy (arrayutmp, ptrv, sizeof(unsigned short)*AOconf[loop].sizeWFS);
        break;
    default :
        printf("ERROR: DATA TYPE NOT SUPPORTED\n");
        exit(0);
        break;
    }
    if(RM==0)
        AOconf[loop].WFScnt = data.image[aoconfID_wfsim].md[0].cnt0;
    else
        AOconf[loop].WFScntRM = data.image[aoconfID_wfsim].md[0].cnt0;


    //   if(COMPUTE_PIXELSTREAMING==1) // multiple pixel groups
    PIXSTREAM_SLICE = data.image[aoconfID_wfsim].md[0].cnt1;

	
	
	
	

	// ===================================================================
	//
    // THIS IS THE STARTING POINT FOR THE LOOP
    //
    // ===================================================================
    if(RM==0)
    {
        AOconf[loop].status = 1;  // 3->001: DARK SUBTRACT
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[0] = tdiffv;
		
		data.image[aoconfID_looptiming].md[0].write = 1;
        data.image[aoconfID_looptiming].md[0].atime.ts = tnow;
		COREMOD_MEMORY_image_set_sempost_byID(aoconfID_looptiming, -1);
        data.image[aoconfID_looptiming].md[0].cnt0++;
		data.image[aoconfID_looptiming].md[0].write = 0;
    }



#ifdef _PRINT_TEST
    printf("TEST - DARK SUBTRACT\n");
    fflush(stdout);
#endif

    // Dark subtract and compute total

    if((loop==0)||(RM == 1)) // single thread, in CPU
    {
        switch ( WFSatype ) {
        case _DATATYPE_UINT16 :
# ifdef _OPENMP
            #pragma omp parallel num_threads(8) if (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
        {
# endif

# ifdef _OPENMP
            #pragma omp for
# endif
            for(ii=0; ii<Average_cam_frames_nelem; ii++)
                data.image[aoconfID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
# ifdef _OPENMP
        }
# endif
        break;
        case _DATATYPE_FLOAT :
# ifdef _OPENMP
            #pragma omp parallel num_threads(8) if (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
        {
# endif

# ifdef _OPENMP
            #pragma omp for
# endif
            for(ii=0; ii<Average_cam_frames_nelem; ii++)
                data.image[aoconfID_imWFS0].array.F[ii] = arrayftmp[ii] - data.image[Average_cam_frames_IDdark].array.F[ii];
# ifdef _OPENMP
        }
# endif
        break;
        default :
            printf("ERROR: WFS data type not recognized\n");
            exit(0);
            break;
        }
		data.image[aoconfID_imWFS0].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
		COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS0, -1);
        
        /*for(s=0; s<data.image[aoconfID_imWFS0].md[0].sem; s++)
        {
            sem_getvalue(data.image[aoconfID_imWFS0].semptr[s], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoconfID_imWFS0].semptr[s]);
        }*/
    }
    else
    {
#ifdef _PRINT_TEST
        printf("TEST - DARK SUBTRACT - START  (init = %d, %d threads)\n", AOLCOMPUTE_DARK_SUBTRACT_THREADinit, COMPUTE_DARK_SUBTRACT_NBTHREADS);
        fflush(stdout);
#endif

        if(AOLCOMPUTE_DARK_SUBTRACT_THREADinit==0)
        {
#ifdef _PRINT_TEST
            printf("TEST - DARK SUBTRACT - CREATE %d THREADS\n", COMPUTE_DARK_SUBTRACT_NBTHREADS);
            fflush(stdout);
#endif

            ti = 0;

            while(ti<COMPUTE_DARK_SUBTRACT_NBTHREADS)
            {
                pthread_create( &thread_dark_subtract[ti], NULL, compute_function_dark_subtract, (void*) &ti);
                sem_init(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], 0, 0);
                sem_init(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[ti], 0, 0);
                usleep(100);
                ti++;
            }
            AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 1;
        }


        for(ti=0; ti<COMPUTE_DARK_SUBTRACT_NBTHREADS; ti++)
        {
            sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], &sval0);
            if(sval0<SEMAPHORE_MAXVAL)
                sem_post(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti]);

            sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], &sval);

#ifdef _PRINT_TEST
            printf("TEST - DARK SUBTRACT - WAITING ON THREAD %ld\n", ti);
            fflush(stdout);
#endif
            sem_wait(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[ti]);
        }

		data.image[aoconfID_imWFS0].md[0].cnt1 = data.image[aoconfID_looptiming].md[0].cnt1;
		COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS0, -1);
		
      /*  for(s=0; s<data.image[aoconfID_imWFS0].md[0].sem; s++)
        {
            sem_getvalue(data.image[aoconfID_imWFS0].semptr[s], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoconfID_imWFS0].semptr[s]);
        }*/
#ifdef _PRINT_TEST
        printf("TEST - DARK SUBTRACT - END\n");
        fflush(stdout);
#endif
    }

    //  if(IDdark!=-1)
    // {
    //    for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
    //       data.image[aoconfID_imWFS0].array.F[ii] -= data.image[IDdark].array.F[ii];
    //}
    AOconf[loop].statusM = 1;
    if(RM==0)
    {
        AOconf[loop].status = 2; // 4 -> 002 : COMPUTE TOTAL OF IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[1] = tdiffv;
    }

#ifdef _PRINT_TEST
    printf("TEST - NORMALIZE = %d\n", normalize);
    fflush(stdout);
#endif


	//
    // Normalize: imWFS0 -> imWFS1
    //
    if(normalize==1)
    {
        if((AOconf[loop].AOLCOMPUTE_TOTAL_ASYNC==0)||(AOLCOMPUTE_TOTAL_INIT==0)||(RM == 1)) // do it in main thread
        {
			float IMTOTAL;
			
            nelem = data.image[aoconfID_imWFS0].md[0].size[0]*data.image[aoconfID_imWFS0].md[0].size[1];
            IMTOTAL = 0.0;
            if(aoconfID_wfsmask!=-1)
            {
                for(ii=0; ii<nelem; ii++)
                    IMTOTAL += data.image[aoconfID_imWFS0].array.F[ii]*data.image[aoconfID_wfsmask].array.F[ii];
            }
            else
            {
                for(ii=0; ii<nelem; ii++)
                    IMTOTAL += data.image[aoconfID_imWFS0].array.F[ii];
            }

            //            AOconf[loop].WFStotalflux = arith_image_total(data.image[aoconfID_imWFS0].name);
            AOconf[loop].WFStotalflux = IMTOTAL;

            AOLCOMPUTE_TOTAL_INIT = 1;
            //            IMTOTAL = AOconf[loop].WFStotalflux;
            if(aoconfID_imWFS0tot!=-1)
            {
                data.image[aoconfID_imWFS0tot].array.F[0] = IMTOTAL;
                COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS0tot, -1);
                //                sem_getvalue(data.image[aoconfID_imWFS0tot].semptr[0], &semval);
                //               if(semval<SEMAPHORE_MAXVAL)
                //                  sem_post(data.image[aoconfID_imWFS0tot].semptr[0]);
            }
        }
        else  // do it in other threads
        {
			#ifdef _PRINT_TEST
			printf("TEST - compute total in separate thread  AOLCOMPUTE_TOTAL_ASYNC_THREADinit = %d\n", AOLCOMPUTE_TOTAL_ASYNC_THREADinit);
			fflush(stdout);
			#endif
			
           // AOconf[loop].WFStotalflux = data.image[aoconfID_imWFS0tot].array.F[0]; // from last loop
            if(AOLCOMPUTE_TOTAL_ASYNC_THREADinit==0)
            {
				
				printf("Starting Image Total Thread \n");
				fflush(stdout);
				
				
                pthread_create( &thread_computetotal_id, NULL, compute_function_imtotal, NULL);
                AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 1;
                imtotalcnt = 0;
                sem_init(&AOLCOMPUTE_TOTAL_ASYNC_sem_name, 0, 0);
            }
            sem_getvalue(&AOLCOMPUTE_TOTAL_ASYNC_sem_name, &semval);
            
            #ifdef _PRINT_TEST
			printf("TEST - semaphore = %d / %d\n", semval, SEMAPHORE_MAXVAL);	
			fflush(stdout);
			#endif
			data.image[aoconfID_imWFS0tot].md[0].cnt1 = imtotalcnt;
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);
        }
    }


    if(RM==0)
    {
        AOconf[loop].status = 3;  // 5 -> 003: NORMALIZE WFS IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[14] = tdiffv;
    }

    data.image[aoconfID_imWFS0].md[0].cnt0 ++;

    nelem = AOconf[loop].sizeWFS;

    if(normalize==1)
    {
        totalinv=1.0/(AOconf[loop].WFStotalflux + AOconf[loop].WFSnormfloor*AOconf[loop].sizeWFS);
        normfloorcoeff = AOconf[loop].WFStotalflux / (AOconf[loop].WFStotalflux + AOconf[loop].WFSnormfloor*AOconf[loop].sizeWFS);
    }
    else
    {
        totalinv = 1.0;
        normfloorcoeff = 1.0;
    }

    GPU_alpha = totalinv;

    GPU_beta = -normfloorcoeff;





    if( ((AOconf[loop].GPUall==0)&&(RM==0)) || (RM==1))  // normalize WFS image by totalinv
    {
#ifdef _PRINT_TEST
        printf("TEST - Normalize [%d]: IMTOTAL = %g    totalinv = %g\n", AOconf[loop].WFSnormalize, data.image[aoconfID_imWFS0tot].array.F[0], totalinv);
        fflush(stdout);
#endif

        data.image[aoconfID_imWFS1].md[0].write = 1;
# ifdef _OPENMP
        #pragma omp parallel num_threads(8) if (nelem>OMP_NELEMENT_LIMIT)
        {
# endif

# ifdef _OPENMP
            #pragma omp for
# endif
            for(ii=0; ii<nelem; ii++)
                data.image[aoconfID_imWFS1].array.F[ii] = data.image[aoconfID_imWFS0].array.F[ii]*totalinv;
# ifdef _OPENMP
        }
# endif
        COREMOD_MEMORY_image_set_sempost_byID(aoconfID_imWFS1, -1);
        data.image[aoconfID_imWFS1].md[0].cnt0 ++;
        data.image[aoconfID_imWFS1].md[0].write = 0;
    }

#ifdef _PRINT_TEST
    printf("TEST - READ CAM DONE\n");
    fflush(stdout);
#endif
	
	AOconf[loop].statusM = 2;
	if(RM==0)
    {
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoconfID_looptiming].array.F[2] = tdiffv;
	}

    return(0);
}











/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 2. LOAD DATA STREAMS     
 *  Load 2D and 3D shared memory images */
/* =============================================================================================== */
/* =============================================================================================== */




long AOloopControl_IOtools_2Dloadcreate_shmim(const char *name, const char *fname, long xsize, long ysize, float DefaultValue)
{
    long ID;
    int CreateSMim = 0;
    int sizeOK;
    uint32_t *sizearray;
	long ii;

    int loadcreatestatus = -1;
    // value of loadcreatestatus :
    // 0 : existing stream has wrong size -> recreating stream
    // 1 : new stream created and content loaded
    // 2 : existing stream updated
    // 3 : FITS image <fname> has wrong size -> do nothing
    // 4 : FITS image <fname> does not exist, stream <name> exists -> do nothing
    // 5 : FITS image <fname> does not exist, stream <name> does not exist -> create empty stream



#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    ID = image_ID(name);
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);

    if(ID==-1) // if <name> is not loaded in memory
    {
        CreateSMim = 0;
        ID = read_sharedmem_image(name);
        if(ID!=-1)  // ... and <name> does not exist as a memory stream
        {
            sizeOK = COREMOD_MEMORY_check_2Dsize(name, xsize, ysize);
            if(sizeOK==0)  // if size is different, delete stream -> create new one
            {
                char command[500];

                printf("\n========== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s ===========\n\n", name, name);
                delete_image_ID(name);

                if(sprintf(command, "rm /tmp/%s.im.shm", name) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command)!=0)
                    printERROR(__FILE__,__func__,__LINE__, "system() error");

                CreateSMim = 1;
                loadcreatestatus = 0;
            }
        }
        else   //  ... and <name> does not exist as a stream -> create new stream
        {
            CreateSMim = 1;
            loadcreatestatus = 1;
        }

        if(CreateSMim == 1)
        {
            sizearray[0] =  xsize;
            sizearray[1] =  ysize;
            if(xsize*ysize>0)
                ID = create_image_ID(name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            for(ii=0;ii<xsize*ysize;ii++)
				data.image[ID].array.F[ii] = DefaultValue;
        }
    }
    free(sizearray);


    if(ID==-1)
    {
        printf("ERROR: could not load/create %s\n", name);
        exit(0);
    }
    else
    {
        long ID1;

        ID1 = load_fits(fname, "tmp2Dim", 3);
        
        if(ID1!=-1)
        {
            sizeOK = COREMOD_MEMORY_check_2Dsize("tmp2Dim", xsize, ysize);
            if(sizeOK==1)
            {
                memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize*ysize);
                printf("loaded file \"%s\" to shared memory \"%s\"\n", fname, name);
                loadcreatestatus = 2;
            }
            else
            {
                printf("File \"%s\" has wrong size (should be 2-D %ld x %ld,  is %d-D %ld x %ld): ignoring\n", fname, xsize, ysize, (int) data.image[ID1].md[0].naxis, (long) data.image[ID1].md[0].size[0], (long) data.image[ID1].md[0].size[1]);
                loadcreatestatus = 3;
            }
            delete_image_ID("tmp2Dim");
        }
        else
        {
            if(CreateSMim==0)
                loadcreatestatus = 4;
            else
                loadcreatestatus = 5;
        }
    }

    // logging


    if(loadcreateshm_log == 1) // results should be logged in ASCII file
    {
        switch ( loadcreatestatus ) {
        case 0 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream has wrong size -> recreating stream\n", fname, name);
            break;
        case 1 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: new stream created and content loaded\n", fname, name);
            break;
        case 2 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream updated\n", fname, name);
            break;
        case 3 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image has wrong size -> do nothing\n", fname, name);
            break;
        case 4 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream exists -> do nothing\n", fname, name);
            break;
        case 5 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream does not exist -> create empty stream\n", fname, name);
            break;
        default:
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: UNKNOWN ERROR CODE\n", fname, name);
            break;
        }
    }


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    return ID;
}








long AOloopControl_IOtools_3Dloadcreate_shmim(const char *name, const char *fname, long xsize, long ysize, long zsize, float DefaultValue)
{
    long ID;
    int CreateSMim;
    int sizeOK;
    uint32_t *sizearray;
    long ID1;
    int creashmimfromFITS = 0;
    long ii;

    int loadcreatestatus = -1;
    // value of loadcreatestatus :
    // 0 : existing stream has wrong size -> recreating stream
    // 1 : new stream created and content loaded
    // 2 : existing stream updated
    // 3 : FITS image <fname> has wrong size -> do nothing
    // 4 : FITS image <fname> does not exist, stream <name> exists -> do nothing
    // 5 : FITS image <fname> does not exist, stream <name> does not exist -> create empty stream
    // 6 : stream exists, size is correct



#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    printf("-------- ENTERING AOloopControl_3Dloadcreate_shmim   name = %s ----------\n", name);
    fflush(stdout);
    
    list_image_ID();


    ID = image_ID(name);
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);

    printf("        ENTERING AOloopControl_3Dloadcreate_shmim: ============== %ld  %ld  %ld ===== %ld ======\n", xsize, ysize, zsize, ID);
    fflush(stdout);

    if(ID==-1)
    {
        CreateSMim = 0;
        ID = read_sharedmem_image(name);

        printf("        AOloopControl_3Dloadcreate_shmim: ============== %ld  ======\n", ID);
        fflush(stdout);


        if(ID!=-1) // stream exists
        {

            sizeOK = COREMOD_MEMORY_check_3Dsize(name, xsize, ysize, zsize);
            if(sizeOK==0)
            {
                char command[500];

                //               printf("\n========== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s ===========\n\n", name, name);
                printf("        AOloopControl_3Dloadcreate_shmim: ===== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s\n", name, name);
                fflush(stdout);

                delete_image_ID(name);

                if(sprintf(command, "rm /tmp/%s.im.shm", name) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command) != 0)
                    printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

                CreateSMim = 1;
                loadcreatestatus = 0;
            }
            else // SIZE OK
            {
                printf("        AOloopControl_3Dloadcreate_shmim: ===== SIZE OK\n");
                fflush(stdout);
                CreateSMim = 0;
                loadcreatestatus = 2;
            }
        }
        else
        {
            CreateSMim = 1;
            loadcreatestatus = 1;
        }

        if(CreateSMim == 1)
        {
            sizearray[0] = xsize;
            sizearray[1] = ysize;
            sizearray[2] = zsize;
            if(xsize*ysize*zsize>0)
            {
                printf("        AOloopControl_3Dloadcreate_shmim: ===== create_image_ID\n");
                fflush(stdout);
                ID = create_image_ID(name, 3, sizearray, _DATATYPE_FLOAT, 1, 0);
                for(ii=0;ii<xsize*ysize*zsize;ii++)
					data.image[ID].array.F[ii] = DefaultValue;
                creashmimfromFITS = 0;
            }
            else
                creashmimfromFITS = 1;
        }
    }
    free(sizearray);


    printf("        AOloopControl_3Dloadcreate_shmim: ===== TEST pt\n");
    fflush(stdout);

    // here, ID is either loaded, or it should be created from FITS image
    if((ID==-1)&&(creashmimfromFITS==0))
    {
        printf("ERROR: could not load/create %s\n", name);
        exit(0);
    }

    ID1 = load_fits(fname, "tmp3Dim", 3);
    printf("        AOloopControl_3Dloadcreate_shmim: ===== ID1 = %ld\n", ID1);
    fflush(stdout);
    if(ID1!=-1)
    {
        if(creashmimfromFITS == 1) // create shared mem from FITS
        {
            sizeOK = COREMOD_MEMORY_check_3Dsize("tmp3Dim", xsize, ysize, zsize);
            printf("        AOloopControl_3Dloadcreate_shmim: ===== sizeOK = %d\n", (int) sizeOK);
            fflush(stdout);
            if(sizeOK==1)
            {
                long xsize1, ysize1, zsize1;

                xsize1 = data.image[ID1].md[0].size[0];
                ysize1 = data.image[ID1].md[0].size[1];
                zsize1 = data.image[ID1].md[0].size[2];
                sizearray[0] = xsize1;
                sizearray[1] = ysize1;
                sizearray[2] = zsize1;
                ID = create_image_ID(name, 3, sizearray, _DATATYPE_FLOAT, 1, 0);

                printf("        AOloopControl_3Dloadcreate_shmim: ===== [1] memcpy  %ld %ld %ld\n", xsize1, ysize1, zsize1);
                fflush(stdout);

                memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize1*ysize1*zsize1);

                printf("        AOloopControl_3Dloadcreate_shmim: ===== [1] memcpy  DONE\n");
                fflush(stdout);

                loadcreatestatus = 1;
            }
            else
            {
                printf("File \"%s\" has wrong size (should be 3-D %ld x %ld, x %ld  is %d-D %ld x %ld x %ld): ignoring\n", fname, xsize, ysize, zsize, (int) data.image[ID1].md[0].naxis, (long) data.image[ID1].md[0].size[0], (long) data.image[ID1].md[0].size[1], (long) data.image[ID1].md[0].size[2]);
                loadcreatestatus = 3;
            }
        }
        else
        {
            printf("        AOloopControl_3Dloadcreate_shmim: ===== [2] memcpy %ld <- %ld     %ld %ld %ld\n", ID, ID1, xsize, ysize, zsize);
            fflush(stdout);
            list_image_ID();

            memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize*ysize*zsize);

            printf("        AOloopControl_3Dloadcreate_shmim: ===== [2] memcpy  DONE\n");
            fflush(stdout);

            loadcreatestatus = 2;
        }
        delete_image_ID("tmp3Dim");
    }
    else
    {
        if(CreateSMim==0)
            loadcreatestatus = 4;
        else
            loadcreatestatus = 5;
    }


    if(loadcreateshm_log == 1) // results should be logged in ASCII file
    {
        switch ( loadcreatestatus ) {
        case 0 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream has wrong size -> recreating stream\n", fname, name);
            break;
        case 1 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: new stream created and content loaded\n", fname, name);
            break;
        case 2 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream updated\n", fname, name);
            break;
        case 3 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image has wrong size -> do nothing\n", fname, name);
            break;
        case 4 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream exists -> do nothing\n", fname, name);
            break;
        case 5 :
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream does not exist -> create empty stream\n", fname, name);
            break;
        default:
            fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: UNKNOWN ERROR CODE\n", fname, name);
            break;
        }
    }


    printf("-------- EXITING AOloopControl_3Dloadcreate_shmim ----------\n");
    fflush(stdout);


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    return ID;
}






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





long AOloopControl_IOtools_frameDelay(const char *IDin_name, const char *IDkern_name, const char *IDout_name, int insem)
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
    uint8_t atype;
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

    atype = _DATATYPE_FLOAT;
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize1;
    sizearray[1] = ysize1;
    IDout = create_image_ID(out_name, 2, sizearray, atype, 1, 0);
    IDout0 = create_image_ID(out0name, 2, sizearray, atype, 1, 0);
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


