/**
 * @file    AOloopControl_IOtools_camerainput.c
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
#include <stdio.h>
#include <pthread.h>

#include "CommandLineInterface/CLIcore.h"

#include "info/info.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "00CORE/00CORE.h"

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
                COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
            }
        }
        break;
    case _DATATYPE_FLOAT :
        while(1)
        {
            usleep(10); // OK FOR NOW (NOT USED BY FAST WFS)
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
                COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
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



	if(aoloopcontrol_var.aoconfID_looptiming == -1)
	{
		// LOOPiteration is written in cnt1 of loop timing array
		if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_looptiming = AOloopControl_IOtools_2Dloadcreate_shmim(imname, " ", aoloopcontrol_var.AOcontrolNBtimers, 1, 0.0);
	}




    nelem = data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[0]*data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[1];

    for(;;)
    {
		#ifdef _PRINT_TEST
		printf("TEST - Waiting for semaphore\n");
		fflush(stdout);
		#endif

        sem_wait(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);

		#ifdef _PRINT_TEST
		printf("TEST - COMPUTING TOTAL FOR IMAGE ID %ld : %s\n", aoloopcontrol_var.aoconfID_imWFS0, data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].name);
		fflush(stdout);
		#endif
	
		imtotalcnt++;
		
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].write = 1;
        IMTOTAL = 0.0;
        if(aoloopcontrol_var.aoconfID_wfsmask!=-1)
        {
            for(ii=0; ii<nelem; ii++)
                IMTOTAL += data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii]*data.image[aoloopcontrol_var.aoconfID_wfsmask].array.F[ii];
        }
        else
        {
            for(ii=0; ii<nelem; ii++)
                IMTOTAL += data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii];
        }
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0] = IMTOTAL;
        
        AOconf[LOOPNUMBER].WFStotalflux = IMTOTAL;
        
        
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        
        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS0tot, -1);
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].write = 0;
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


    nelem = data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[0]*data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[1];
    index = (long*) ptr;
    threadindex = *index;

    iistart = (long) ((threadindex)*nelem/COMPUTE_DARK_SUBTRACT_NBTHREADS);
    iiend = (long) ((threadindex+1)*nelem/COMPUTE_DARK_SUBTRACT_NBTHREADS);

    while(1)
    {
        sem_wait(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[threadindex]);

        switch ( aoloopcontrol_var.WFSatype ) {
        case _DATATYPE_UINT16 :
            for(ii=iistart; ii<iiend; ii++)
                data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
            break;
        case _DATATYPE_FLOAT :
            for(ii=iistart; ii<iiend; ii++)
                data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] = ((float) arrayftmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
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
 * puts image from camera buffer aoloopcontrol_var.aoconfID_wfsim into aoloopcontrol_var.aoconfID_imWFS1 (supplied by user)
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


    aoloopcontrol_var.WFSatype = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].atype;

    if(avcamarraysInit==0)
    {
        arrayftmp = (float*) malloc(sizeof(float)*AOconf[loop].sizeWFS);
        arrayutmp = (unsigned short*) malloc(sizeof(unsigned short)*AOconf[loop].sizeWFS);

        if(sprintf(Average_cam_frames_dname, "aol%ld_wfsdark", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        Average_cam_frames_IDdark = image_ID(Average_cam_frames_dname);
        Average_cam_frames_nelem = AOconf[loop].sizeWFS;

        // set semaphore to 0
        sem_getvalue(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n", semindex, data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].name, semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex]);

        //aoloopcontrol_var.PIXSTREAM_SLICE = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt1;    // set semaphore 1 to 0

        avcamarraysInit = 1;
    }

    if(InitSem==1)
    {
        sem_getvalue(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n", semindex, data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].name, semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex]);
    }

#ifdef _PRINT_TEST
    printf("TEST - SEMAPHORE INITIALIZED\n");
    fflush(stdout);
#endif

    if(RM==0)
    {
        AOconf[loop].status = 20;  // 020: WAIT FOR IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[24] = tdiffv;
    }
    else
        data.status1 = 2;

    //   usleep(20);

#ifdef _PRINT_TEST
    printf("TEST - WAITING FOR IMAGE %s\n", data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].name);
    fflush(stdout);
#endif

    if(data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].sem==0)
    {
        if(RM==0)
            while(AOconf[loop].WFScnt==data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0) // test if new frame exists
                usleep(5);
        else
            while(AOconf[loop].WFScntRM==data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0) // test if new frame exists
                usleep(5);
    }
    else
    {
#ifdef _PRINT_TEST
        printf("TEST - waiting on semindex = %d\n", semindex);
        fflush(stdout);
#endif

        sem_wait(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex]);

        sem_getvalue(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex], &semval);
        for(i=0; i<semval; i++)
            sem_trywait(data.image[aoloopcontrol_var.aoconfID_wfsim].semptr[semindex]);


#ifdef _PRINT_TEST
        printf("TEST - semaphore posted\n");
        fflush(stdout);
#endif
    }

	
	if(RM==0)
	{
		clock_gettime(CLOCK_REALTIME, &tnow);
		
		printf("------ SETUP wfsim RTlog bugger\n"); //TEST
		fflush(stdout);
		sleep(100.0);
	
		AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_wfsim, tnow);

        AOconf[loop].status = 0;  // LOAD IMAGE
	}
	
    AOconf[loop].statusM = 0;


    slice = 0;
    if(data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].naxis==3) // ring buffer
    {
        slice = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt1;
        if(slice==-1)
            slice = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].size[2];
    }

    switch (aoloopcontrol_var.WFSatype) {
    case _DATATYPE_FLOAT :
        ptrv = (char*) data.image[aoloopcontrol_var.aoconfID_wfsim].array.F;
        ptrv += sizeof(float)*slice* AOconf[loop].sizeWFS;
        memcpy(arrayftmp, ptrv,  sizeof(float)*AOconf[loop].sizeWFS);
        break;
    case _DATATYPE_UINT16 :
        ptrv = (char*) data.image[aoloopcontrol_var.aoconfID_wfsim].array.UI16;
        ptrv += sizeof(unsigned short)*slice* AOconf[loop].sizeWFS;
        memcpy (arrayutmp, ptrv, sizeof(unsigned short)*AOconf[loop].sizeWFS);
        break;
    default :
        printf("ERROR: DATA TYPE NOT SUPPORTED\n");
        exit(0);
        break;
    }
    if(RM==0)
        AOconf[loop].WFScnt = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0;
    else
        AOconf[loop].WFScntRM = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0;


    //   if(COMPUTE_PIXELSTREAMING==1) // multiple pixel groups
    aoloopcontrol_var.PIXSTREAM_SLICE = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt1;

	
	
	
	

	// ===================================================================
	//
    // THIS IS THE STARTING POINT FOR THE LOOP
    //
    // ===================================================================
    if(RM==0)
    {
        AOconf[loop].status = 1;  // 3->001: DARK SUBTRACT
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[0] = tdiffv;
		
		data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].write = 1;
        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts = tnow;
		COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_looptiming, -1);
        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt0++;
		data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].write = 0;
    }



#ifdef _PRINT_TEST
    printf("TEST - DARK SUBTRACT\n");
    fflush(stdout);
#endif

    // Dark subtract and compute total

    if((loop==0)||(RM == 1)) // single thread, in CPU
    {
        switch ( aoloopcontrol_var.WFSatype ) {
        case _DATATYPE_UINT16 :
# ifdef _OPENMP
            #pragma omp parallel num_threads(8) if (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
        {
# endif

# ifdef _OPENMP
            #pragma omp for
# endif
            for(ii=0; ii<Average_cam_frames_nelem; ii++)
                data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]) - data.image[Average_cam_frames_IDdark].array.F[ii];
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
            #pragma omp parallel for
# endif
            for(ii=0; ii<Average_cam_frames_nelem; ii++)
                data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] = arrayftmp[ii] - data.image[Average_cam_frames_IDdark].array.F[ii];
# ifdef _OPENMP
        }
# endif
        break;
        default :
            printf("ERROR: WFS data type not recognized\n");
            exit(0);
            break;
        }
		data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
		COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS0, -1);
        
        clock_gettime(CLOCK_REALTIME, &tnow);
		AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_wfsim, tnow);
        
        /*for(s=0; s<data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].sem; s++)
        {
            sem_getvalue(data.image[aoloopcontrol_var.aoconfID_imWFS0].semptr[s], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoloopcontrol_var.aoconfID_imWFS0].semptr[s]);
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

		data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
		COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS0, -1);
		
      /*  for(s=0; s<data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].sem; s++)
        {
            sem_getvalue(data.image[aoloopcontrol_var.aoconfID_imWFS0].semptr[s], &semval);
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(data.image[aoloopcontrol_var.aoconfID_imWFS0].semptr[s]);
        }*/
#ifdef _PRINT_TEST
        printf("TEST - DARK SUBTRACT - END\n");
        fflush(stdout);
#endif
    }

    //  if(IDdark!=-1)
    // {
    //    for(ii=0; ii<AOconf[loop].sizeWFS; ii++)
    //       data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] -= data.image[IDdark].array.F[ii];
    //}
    AOconf[loop].statusM = 1;
    if(RM==0)
    {
        AOconf[loop].status = 2; // 4 -> 002 : COMPUTE TOTAL OF IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[1] = tdiffv;
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
			
            nelem = data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[0]*data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[1];
            IMTOTAL = 0.0;
            if(aoloopcontrol_var.aoconfID_wfsmask!=-1)
            {
                for(ii=0; ii<nelem; ii++)
                    IMTOTAL += data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii]*data.image[aoloopcontrol_var.aoconfID_wfsmask].array.F[ii];
            }
            else
            {
                for(ii=0; ii<nelem; ii++)
                    IMTOTAL += data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii];
            }

            //            AOconf[loop].WFStotalflux = arith_image_total(data.image[aoloopcontrol_var.aoconfID_imWFS0].name);
            AOconf[loop].WFStotalflux = IMTOTAL;

            AOLCOMPUTE_TOTAL_INIT = 1;
            //            IMTOTAL = AOconf[loop].WFStotalflux;
            if(aoloopcontrol_var.aoconfID_imWFS0tot!=-1)
            {
                data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0] = IMTOTAL;
                COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS0tot, -1);
                //                sem_getvalue(data.image[aoloopcontrol_var.aoconfID_imWFS0tot].semptr[0], &semval);
                //               if(semval<SEMAPHORE_MAXVAL)
                //                  sem_post(data.image[aoloopcontrol_var.aoconfID_imWFS0tot].semptr[0]);
            }
        }
        else  // do it in other threads
        {
			#ifdef _PRINT_TEST
			printf("TEST - compute total in separate thread  AOLCOMPUTE_TOTAL_ASYNC_THREADinit = %d\n", AOLCOMPUTE_TOTAL_ASYNC_THREADinit);
			fflush(stdout);
			#endif
			
           // AOconf[loop].WFStotalflux = data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0]; // from last loop
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
			data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt1 = imtotalcnt;
            if(semval<SEMAPHORE_MAXVAL)
                sem_post(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);
        }
    }


    if(RM==0)
    {
        AOconf[loop].status = 3;  // 5 -> 003: NORMALIZE WFS IMAGE
        clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[14] = tdiffv;
    }

    data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].cnt0 ++;

    nelem = AOconf[loop].sizeWFS;

    if(normalize==1)
    {
        totalinv=1.0/(AOconf[loop].WFStotalflux + AOconf[loop].WFSnormfloor*AOconf[loop].sizeWFS);
        aoloopcontrol_var.normfloorcoeff = AOconf[loop].WFStotalflux / (AOconf[loop].WFStotalflux + AOconf[loop].WFSnormfloor*AOconf[loop].sizeWFS);
    }
    else
    {
        totalinv = 1.0;
        aoloopcontrol_var.normfloorcoeff = 1.0;
    }

    aoloopcontrol_var.GPU_alpha = totalinv;

    aoloopcontrol_var.GPU_beta = -aoloopcontrol_var.normfloorcoeff;





    if( ((AOconf[loop].GPUall==0)&&(RM==0)) || (RM==1))  // normalize WFS image by totalinv
    {
#ifdef _PRINT_TEST
        printf("TEST - Normalize [%d]: IMTOTAL = %g    totalinv = %g\n", AOconf[loop].WFSnormalize, data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0], totalinv);
        fflush(stdout);
#endif

        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].write = 1;
# ifdef _OPENMP
        #pragma omp parallel num_threads(8) if (nelem>OMP_NELEMENT_LIMIT)
        {
# endif

# ifdef _OPENMP
            #pragma omp for
# endif
            for(ii=0; ii<nelem; ii++)
                data.image[aoloopcontrol_var.aoconfID_imWFS1].array.F[ii] = data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii]*totalinv;
# ifdef _OPENMP
        }
# endif
        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS1, -1);
        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].cnt0 ++;
        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].write = 0;                
    }

#ifdef _PRINT_TEST
    printf("TEST - READ CAM DONE\n");
    fflush(stdout);
#endif
	
	AOconf[loop].statusM = 2;
	if(RM==0)
    {
		clock_gettime(CLOCK_REALTIME, &tnow);
        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime.ts, tnow);
        tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[2] = tdiffv;
        
        if(AOconf[loop].GPUall==0)
			AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_imWFS1, tnow);
	}

    return(0);
}




