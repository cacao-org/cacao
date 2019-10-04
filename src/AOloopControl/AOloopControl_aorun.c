/**
 * @file    AOloopControl_aorun.c 
 * @brief   AO loop Control compute functions 
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * @bug No known bugs.
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

#ifdef __MACH__   // for Mac OS X - 
//#include <mach/mach_time.h>
//#define CLOCK_REALTIME 0
//#define CLOCK_MONOTONIC 0
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


#include <string.h>
#include <sched.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <pthread.h>
#include "info/info.h" 


#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"

#include <ncurses.h>


#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif

#define NB_AOloopcontrol 10 // max number of loops

static int AOlooploadconf_init = 0;


static int wcol, wrow; // window size





extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;










static int AOloopControl_aorun_ProcessInit_Value = 0; // toggles to 1 when AOloopControl_aorun() started

// ********************************************************************
// This initialization runs once per process
// ********************************************************************

int AOloopControl_aorun_ProcessInit()
{
	
	
	
	return 0;
}






int printstatus_AOloopControl_aorun(int loop)
{
	
	
	printw("LOOPiteration     %8ld\n", AOconf[loop].aorun.LOOPiteration);
	printw("kill              %d\n",   AOconf[loop].aorun.kill);
	printw("on                %d\n", AOconf[loop].aorun.on);
	printw("DMprimaryWriteON  %d\n", AOconf[loop].aorun.DMprimaryWriteON);
	printw("CMMODE            %d\n", AOconf[loop].aorun.CMMODE);
	printw("DMfilteredWriteON %d\n", AOconf[loop].aorun.DMfilteredWriteON);
	printw("ARPFon            %d\n", AOconf[loop].aorun.ARPFon);
	
	return 0;
}




int AOloopControl_aorun_GUI(
    long loop,
    double frequ
)
{
    char monstring[200];
    int loopOK = 1;
    int freeze = 0;
    long cnt = 0;



    // Connect to shared memory
    AOloopControl_InitializeMemory(1);


    /*  Initialize ncurses  */
    if ( initscr() == NULL ) {
        fprintf(stderr, "Error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }

    getmaxyx(stdscr, wrow, wcol);		/* get the number of rows and columns */
    cbreak();
    keypad(stdscr, TRUE);		        /* We get F1, F2 etc..		*/
    nodelay(stdscr, TRUE);
    curs_set(0);
    noecho();			                /* Don't echo() while we do getch */

    start_color();
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_BLACK, COLOR_RED);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_YELLOW, COLOR_BLACK);
    init_pair(5, COLOR_RED, COLOR_BLACK);
    init_pair(6, COLOR_BLACK, COLOR_RED);

    while( loopOK == 1 )
    {
        usleep((long) (1000000.0/frequ));
        int ch = getch();

        if(freeze==0)
        {
            attron(A_BOLD);
            sprintf(monstring, "PRESS x TO STOP MONITOR");
            print_header(monstring, '-');
            attroff(A_BOLD);
        }

        switch (ch)
        {
        case 'f':
            if(freeze==0)
                freeze = 1;
            else
                freeze = 0;
            break;
           
        case 'x':
            loopOK=0;
            break;
        }

        if(freeze==0)
        {
            clear();
            printstatus_AOloopControl_aorun(loop);

            refresh();

            cnt++;
        }
    }
    endwin();


    return(0);
}








int AOloopControl_aorun_FPCONF(
    char *fpsname,
    uint32_t CMDmode
)
{
	uint16_t loopstatus;
	
    // ===========================
    // SETUP FPS
    // ===========================

    FUNCTION_PARAMETER_STRUCT fps = function_parameter_FPCONFsetup(fpsname, CMDmode, &loopstatus);


    // ===========================
    // ALLOCATE FPS ENTRIES 
    // ===========================

    void *pNull = NULL;
    uint64_t FPFLAG;
    
    
    // configuration
    int64_t loopindex_default[4] = { 0, 0, 99, 0 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
    long fpi_loopindex = function_parameter_add_entry(&fps, ".loopindex", "Loop index", 
                                     FPTYPE_INT64, FPFLAG, &loopindex_default);

    
    int64_t RTpriority_default[4] = { 90, 0, 99, 90 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
    long fpi_RTpriority = function_parameter_add_entry(&fps, ".RTpriority", "Real Time priority", 
                                     FPTYPE_INT64, FPFLAG, &RTpriority_default);
    

    long semwaitindex_default[4] = { 1, 0, 10, 1 };
    long fpi_semwaitindex = function_parameter_add_entry(&fps, ".semwaitindex",
                       "input semaphore index",
                       FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, &semwaitindex_default);


    long WFSrefON_default[4] = { 0, 0, 1, 0 };
    long fpi_WFSrefON = function_parameter_add_entry(&fps, ".wfsrefON",
                       "Use WFS reference",
                       FPTYPE_ONOFF, FPFLAG_DEFAULT_INPUT, &WFSrefON_default);
    
    
    // stream that needs to be loaded on startup
    FPFLAG = FPFLAG_DEFAULT_INPUT_STREAM | FPFLAG_STREAM_RUN_REQUIRED;
    long fpi_streamname_wfs       = function_parameter_add_entry(&fps, ".sn_wfs",  "WFS stream name",
                                     FPTYPE_STREAMNAME, FPFLAG, "NULL");
    
    FPFLAG = FPFLAG_DEFAULT_INPUT_STREAM | FPFLAG_STREAM_RUN_REQUIRED;
    long fpi_streamname_cmat       = function_parameter_add_entry(&fps, ".sn_cmat",  "Control Matrix",
                                     FPTYPE_STREAMNAME, FPFLAG, "NULL");   
    
    // required to get DM size
    FPFLAG = FPFLAG_DEFAULT_INPUT_STREAM | FPFLAG_STREAM_RUN_REQUIRED;
    long fpi_streamname_DMout      = function_parameter_add_entry(&fps, ".sn_DMout",  "output stream",
                                     FPTYPE_STREAMNAME, FPFLAG, "NULL");   
                                         
    
  
    
    // main control 
    
    double loopgaindefault[4] = { 0.001, 0.0, 1.5, 0.001 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;  // required to enforce the min and max limits
    FPFLAG |= FPFLAG_WRITERUN;
    long fpi_loopgain = function_parameter_add_entry(&fps, ".loopgain", "Main loop gain", 
                                     FPTYPE_FLOAT64, FPFLAG, &loopgaindefault);

    double loopmultdefault[4] = { 0.001, 0.0, 1.5, 0.001 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;  // required to enforce the min and max limits
    FPFLAG |= FPFLAG_WRITERUN;
    long fpi_loopmult = function_parameter_add_entry(&fps, ".loopmult", "Main loop mult coeff", 
                                     FPTYPE_FLOAT64, FPFLAG, &loopmultdefault);


    double maxlimdefault[4] = { 1.0, 0.0, 1000.0, 1.0 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;  // required to enforce the min and max limits
    FPFLAG |= FPFLAG_WRITERUN;
    long fpi_maxlim = function_parameter_add_entry(&fps, ".maxlim", "Maximum limit", 
                                     FPTYPE_FLOAT64, FPFLAG, &maxlimdefault);

	FPFLAG = FPFLAG_DEFAULT_INPUT;
	FPFLAG |= FPFLAG_WRITERUN;
	long fpi_loopON = function_parameter_add_entry(&fps, ".loopON", "loop ON/OFF", 
                                     FPTYPE_ONOFF, FPFLAG, pNull);       
    
    
    
    // =====================================
    // PARAMETER LOGIC AND UPDATE LOOP
    // =====================================

    while ( loopstatus == 1 )
    {
       usleep(50);
        if( function_parameter_FPCONFloopstep(&fps, CMDmode, &loopstatus) == 1) // Apply logic if update is needed
        {
            // here goes the logic
          

            functionparameter_CheckParametersAll(&fps);  // check all parameter values
        }       

    }
    
    

	function_parameter_FPCONFexit( &fps );
	    
	return RETURN_SUCCESS;
}








int AOloopControl_aorun_RUN(
    char *fpsname
)
{
    // ===========================
    // CONNECT TO FPS
    // ===========================

    FUNCTION_PARAMETER_STRUCT fps;

    if(function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_RUN) == -1)
    {
        printf("ERROR: fps \"%s\" does not exist -> running without FPS interface\n", fpsname);
        return RETURN_FAILURE;
    }



    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================

    long loop = functionparameter_GetParamValue_ONOFF(&fps, ".loopindex");

    char snameWFS[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(snameWFS, functionparameter_GetParamPtr_STRING(&fps, ".sn_wfs"), FUNCTION_PARAMETER_STRMAXLEN);

    char snameCM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(snameCM, functionparameter_GetParamPtr_STRING(&fps, ".sn_cmat"), FUNCTION_PARAMETER_STRMAXLEN);

    char snameDMout[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(snameDMout, functionparameter_GetParamPtr_STRING(&fps, ".sn_DMout"), FUNCTION_PARAMETER_STRMAXLEN);

    long *semwaitindex;
    semwaitindex = functionparameter_GetParamPtr_INT64(&fps, ".semwaitindex");



    int wfsrefON = functionparameter_GetParamValue_ONOFF(&fps, ".wfsrefON");

    // This parameter is a ON / OFF toggle
    uint64_t *loopONflag = functionparameter_GetParamPtr_fpflag(&fps, ".loopON");

    // This parameter value will be tracked during loop run, so we create a pointer for it
    // The corresponding function is functionparameter_GetParamPtr_<TYPE>
    //
    double *loopgain = functionparameter_GetParamPtr_FLOAT64(&fps, ".loopgain");
    double *loopmult = functionparameter_GetParamPtr_FLOAT64(&fps, ".loopmult");
    double *maxlim = functionparameter_GetParamPtr_FLOAT64(&fps, ".maxlim");


    int RTpriority = functionparameter_GetParamValue_INT64(&fps, ".RTpriority");





    // ===========================
    // ### processinfo support
    // ===========================

    PROCESSINFO *processinfo;


    char pinfoname[200];
    sprintf(pinfoname, "aol%ld-aorun", loop);

    char pinfodescr[200];
    sprintf(pinfodescr, "run AOloop %ld", loop);

    char pinfomsg[200];
    sprintf(pinfomsg, "Initialize AO loop %ld", loop);

    processinfo = processinfo_setup(
                      pinfoname,             // short name for the processinfo instance, no spaces, no dot, name should be human-readable
                      pinfodescr,    // description
                      pinfomsg,  // message on startup
                      __FUNCTION__, __FILE__, __LINE__
                  );


    // OPTIONAL SETTINGS
    processinfo->MeasureTiming = 1; // Measure timing
    processinfo->RT_priority = RTpriority;  // RT_priority, 0-99. Larger number = higher priority. If <0, ignore





    // connect to WFS image
    long IDimWFS1 = read_sharedmem_image(snameWFS);
    // set semaphore to 0
    int          semval;
    sem_getvalue(data.image[IDimWFS1].semptr[*semwaitindex], &semval);
    printf("INITIALIZING SEMAPHORE %ld   %s   (%d)\n", *semwaitindex, data.image[IDimWFS1].md[0].name, semval);
    for(int i=0; i<semval; i++)
        sem_trywait(data.image[IDimWFS1].semptr[*semwaitindex]);

    long sizeWFS = data.image[IDimWFS1].md[0].size[0] * data.image[IDimWFS1].md[0].size[1];

    // create imWFS2
    char imWFS2sname[200];
    sprintf(imWFS2sname, "aol%ld_imWFS2", loop);

    long naxis = 2;
    uint32_t *imsizearray;
    imsizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    imsizearray[0] = data.image[IDimWFS1].md[0].size[0];
    imsizearray[1] = data.image[IDimWFS1].md[0].size[1];
    long IDimWFS2 = create_image_ID(imWFS2sname, naxis, imsizearray, _DATATYPE_FLOAT, 1, 10);
    free(imsizearray);


    // control matrix
    long ID_CM = read_sharedmem_image(snameCM);


    // output DM
    long ID_DMout = read_sharedmem_image(snameDMout);
    long sizeDM;
    sizeDM = data.image[ID_DMout].md[0].size[0]*data.image[ID_DMout].md[0].size[1];


    // output MVM
    char outMVMsname[200];
    sprintf(outMVMsname, "aol%ld_outMVM", loop);

    naxis = 2;
    imsizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    imsizearray[0] = data.image[ID_DMout].md[0].size[0];
    imsizearray[1] = data.image[ID_DMout].md[0].size[1];
    long ID_MVMout = create_image_ID(outMVMsname, naxis, imsizearray, _DATATYPE_FLOAT, 1, 10);
    free(imsizearray);






    long IDwfsref = -1;

    float normfloorcoeff = 1.0;

    // ==================================
    // STARTING LOOP
    // ==================================
    processinfo_loopstart(processinfo); // Notify processinfo that we are entering loop

    // ===============================
    // RUN LOOP
    // ===============================
    long long WFScnt  = 0;
    int loopOK = 1;
    while( loopOK == 1 )
    {
        loopOK = processinfo_loopstep(processinfo);



        // ===========================================
        // WAIT FOR INPUT
        // ===========================================
        if(data.image[IDimWFS1].md[0].sem == 0) { // don't use semaphore
            // use counter to test if new WFS frame is ready
            while(WFScnt == data.image[IDimWFS1].md[0].cnt0) // test if new frame exists
                usleep(5);
        }
        else
        {
            sem_getvalue(data.image[IDimWFS1].semptr[*semwaitindex], &semval);
            if(semval>0)
            {
                if(semval>1) {
                    printf("\n\033[31;1m WARNING [%d] WFS SEMAPHORE already posted - Missed frame\033[0m\n", semval);
                }
                fflush(stdout);
            }


            int rval;
            rval = ImageStreamIO_semwait(&data.image[IDimWFS1], *semwaitindex);

            if (rval == -1)
                perror("semwait");


            sem_getvalue(data.image[IDimWFS1].semptr[*semwaitindex], &semval);
            for(int i=0; i<semval; i++)
            {
                //			printf("WARNING: [%d] sem_trywait on ID_wfsim\n", (int) (semval - i));
                //			fflush(stdout);
                //sem_trywait(data.image[ID_wfsim].semptr[*semindex]);
                ImageStreamIO_semtrywait(&data.image[IDimWFS1], *semwaitindex);
            }
        }



        processinfo_exec_start(processinfo);
        if(processinfo_compute_status(processinfo)==1)
        {
            // Subtract reference
            data.image[IDimWFS2].md[0].write = 1;
            if(wfsrefON == 1) { // if WFS reference is NOT zero
                for(long ii = 0; ii < sizeWFS; ii++) {
                    data.image[IDimWFS2].array.F[ii] = data.image[IDimWFS1].array.F[ii] - normfloorcoeff * data.image[IDwfsref].array.F[ii];
                }
            } else {
                memcpy(data.image[IDimWFS2].array.F, data.image[IDimWFS1].array.F, sizeof(float)*sizeWFS);
            }
            COREMOD_MEMORY_image_set_sempost_byID(IDimWFS2, -1);
            data.image[IDimWFS2].md[0].cnt0 ++;
            data.image[IDimWFS2].md[0].cnt1 = WFScnt;
            data.image[IDimWFS2].md[0].write = 0;


            data.image[ID_MVMout].md[0].write = 1;
            ControlMatrixMultiply(data.image[ID_CM].array.F, data.image[IDimWFS2].array.F, sizeDM, sizeWFS, data.image[ID_MVMout].array.F);
            COREMOD_MEMORY_image_set_sempost_byID(ID_MVMout, -1);
            data.image[ID_MVMout].md[0].cnt0 ++;
            data.image[ID_MVMout].md[0].cnt1 = WFScnt;
            data.image[ID_MVMout].md[0].write = 0;


            if( *loopONflag & FPFLAG_ONOFF )
            {
                data.image[ID_DMout].md[0].write = 1;
                for(long ii=0; ii<sizeDM; ii++) {
					float tmpval = data.image[ID_DMout].array.F[ii] - (*loopgain) * data.image[ID_MVMout].array.F[ii];
					tmpval *= (*loopmult);
					if(tmpval > *maxlim) {
						tmpval = (*maxlim);
					}
					if(tmpval < - *maxlim) {
						tmpval = - (*maxlim);
					}
                    data.image[ID_DMout].array.F[ii] = tmpval;
                }
                data.image[ID_DMout].md[0].cnt0 ++;
                COREMOD_MEMORY_image_set_sempost_byID(ID_DMout, -1);
                data.image[ID_DMout].md[0].cnt0 ++;
                data.image[ID_DMout].md[0].cnt1 = WFScnt;
                data.image[ID_DMout].md[0].write = 0;
            }


        }



        // process signals, increment loop counter
        processinfo_exec_end(processinfo);

    }

    // ==================================
    // ### ENDING LOOP
    // ==================================

    processinfo_cleanExit(processinfo);
	function_parameter_RUNexit( &fps );


    return RETURN_SUCCESS;
}








/**
 * ## Purpose
 *
 * Main AO loop function
 *
 * ## Overview
 *
 * Runs the AO loop\n
 * Calls AOcompute(), which computes the correction to the applied.\n
 * Then, the correction is applied by calling set_DM_modes() if modal correction, or by direct write to the DM otherwise.\n
 *
 *
 * ## Details
 *
 */
int_fast8_t __attribute__((hot)) AOloopControl_aorun() {
    FILE *fp;
    char fname[200];
    long loop;
    int vOK;
    long ii;
    long ID;
    long j, m;
    struct tm *uttime;
    time_t t;
    struct timespec *thetime = (struct timespec *)malloc(sizeof(struct timespec));
    char logfname[1000];
    char command[1000];
    int r;
    int RT_priority = 90; //any number from 0-99
    struct sched_param schedpar;
    double a;
    long cnttest;
    float tmpf1;

    struct timespec tnow;
    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    int semval;


    struct timespec functionTestTimerStart;
    struct timespec functionTestTimerEnd;

    struct timespec functionTestTimer00;
    struct timespec functionTestTimer01;
    struct timespec functionTestTimer02;
    struct timespec functionTestTimer03;
    struct timespec functionTestTimer04;



    loop = aoloopcontrol_var.LOOPNUMBER;




    PROCESSINFO *processinfo;

    char pinfoname[200];
    sprintf(pinfoname, "aol%ld-aorun", loop);

    char pinfodescr[200];
    sprintf(pinfodescr, "run AOloop %ld", loop);

    char pinfomsg[200];
    sprintf(pinfomsg, "Initialize AO loop %ld", loop);

    processinfo = processinfo_setup(
                      pinfoname,             // short name for the processinfo instance, no spaces, no dot, name should be human-readable
                      pinfodescr,    // description
                      pinfomsg,  // message on startup
                      __FUNCTION__, __FILE__, __LINE__
                  );
    // OPTIONAL SETTINGS
    processinfo->MeasureTiming = 1; // Measure timing
    processinfo->RT_priority = RT_priority;  // RT_priority, 0-99. Larger number = higher priority. If <0, ignore

    int loopOK = 1;





    // LOG function start
    int logfunc_level = 0;
    int logfunc_level_max = 1;
    char commentstring[200];
    sprintf(commentstring, "Main function, loop %ld", loop);
    CORE_logFunctionCall(logfunc_level, logfunc_level_max, 0, __FILE__, __func__, __LINE__, commentstring);



    if(aoloopcontrol_var.AOloopcontrol_meminit == 0) {
        AOloopControl_InitializeMemory(0);
    }


    if(AOloopControl_aorun_ProcessInit_Value == 0) {
        AOloopControl_aorun_ProcessInit();
        AOloopControl_aorun_ProcessInit_Value = 1;
    }



    /** ### STEP 1: Setting up
     *
     * Load arrays
     * */
    printf("SETTING UP...\n");
    AOloopControl_loadconfigure(aoloopcontrol_var.LOOPNUMBER, 1, 10);






    // pixel streaming ?
    aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 1;

    if(AOconf[loop].AOcompute.GPUall == 0) {
        aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;
    }


    printf("============ pixel streaming ? =============\n");
    fflush(stdout);

    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING == 1) {
        aoloopcontrol_var.aoconfID_pixstream_wfspixindex = load_fits("pixstream_wfspixindex.fits", "pixstream", 1);
    }

    if(aoloopcontrol_var.aoconfID_pixstream_wfspixindex == -1) {
        aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;
    } else {
        printf("Testing data type\n");
        fflush(stdout);
        if(data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].datatype != _DATATYPE_UINT16) {
            aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0;
        }
    }

    if(aoloopcontrol_var.COMPUTE_PIXELSTREAMING == 1) {
        long xsize = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].size[0];
        long ysize = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].md[0].size[1];
        aoloopcontrol_var.PIXSTREAM_NBSLICES = 0;
        for(ii = 0; ii < xsize * ysize; ii++)
            if(data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii] > aoloopcontrol_var.PIXSTREAM_NBSLICES) {
                aoloopcontrol_var.PIXSTREAM_NBSLICES = data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii];
            }
        aoloopcontrol_var.PIXSTREAM_NBSLICES++;
        printf("PIXEL STREAMING:   %d image slices\n", aoloopcontrol_var.PIXSTREAM_NBSLICES);
    }

    printf("============ FORCE pixel streaming = 0\n");
    fflush(stdout);
    aoloopcontrol_var.COMPUTE_PIXELSTREAMING = 0; // TEST















    vOK = 1;
    if(AOconf[loop].aorun.init_wfsref0 == 0) {

        processinfo_error(processinfo, "ERROR: no WFS reference");
        loopOK = 0;
        vOK = 0;
    }


    //    if(AOconf[loop].init_CM==0)
    if(aoloopcontrol_var.init_CM_local == 0) {
        char msgstring[200];

        printf("ERROR: CANNOT RUN LOOP WITHOUT CONTROL MATRIX\n");
        printf("aoloopcontrol_var.init_CM_local = 0\n");
        printf("FILE %s  line %d\n", __FILE__, __LINE__);

        processinfo_error(processinfo, "ERROR: no control matrix");
        loopOK = 0;
        vOK = 0;
    }

    aoloopcontrol_var.aoconfcnt0_wfsref_current = data.image[aoloopcontrol_var.aoconfID_wfsref].md[0].cnt0;


    AOconf[loop].aorun.initmapping = 0;
    AOconf[loop].aorun.init_CMc = 0;
    clock_gettime(CLOCK_REALTIME, &t1);





    if(vOK == 1) {
        AOconf[loop].aorun.LOOPiteration = 0;
        AOconf[loop].aorun.kill = 0;
        AOconf[loop].aorun.on = 0;
        AOconf[loop].aorun.DMprimaryWriteON = 0;
        AOconf[loop].aorun.DMfilteredWriteON = 0;
        AOconf[loop].aorun.ARPFon = 0;

#ifdef _PRINT_TEST
        printf("[%s] [%d]  AOloopControl_aorun: Entering loop\n", __FILE__, __LINE__);
        fflush(stdout);
#endif

        int timerinit = 0;
        if(data.processinfo == 1) {
            processinfo->loopstat = 1;
        }




        processinfo_WriteMessage(processinfo, "Entering loop");

        int processinfoUpdate = 0;



        // ==================================
        // STARTING LOOP
        // ==================================
        processinfo_loopstart(processinfo); // Notify processinfo that we are entering loop


        while(AOconf[loop].aorun.kill == 0) {
            if(timerinit == 1) {
                processinfoUpdate = 1;
                clock_gettime(CLOCK_REALTIME, &t1);
                printf("timer init\n");
                if(data.processinfo == 1) {
                    char msgstring[200];
                    sprintf(msgstring, "Waiting");
                    processinfo_WriteMessage(processinfo, msgstring);
                }

            }
            clock_gettime(CLOCK_REALTIME, &t2);

            tdiff = info_time_diff(t1, t2);
            double tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

            printf(" WAITING     %20.3lf sec         \r", tdiffv);
            fflush(stdout);
            usleep(1000);

            processinfoUpdate = 1;

            timerinit = 0;




            sprintf(pinfomsg, "WAITING (on=%d   loopOK=%d) [%d]", AOconf[loop].aorun.on, loopOK, AOconf[loop].aorun.CMMODE);
            processinfo_WriteMessage(processinfo, pinfomsg);

            long loopcnt = 0;
            while((AOconf[loop].aorun.on == 1) && (loopOK == 1)) {

                loopOK = processinfo_loopstep(processinfo);


                if(loopOK == 0) {
                    AOconf[loop].aorun.on = 0;
                    AOconf[loop].aorun.kill = 1;
                }


                if(processinfoUpdate == 1) {
                    processinfo_WriteMessage(processinfo, "LOOP RUNNING-");
                }


                clock_gettime(CLOCK_REALTIME, &functionTestTimer00); //TEST timing in function
                if(timerinit == 0) {
                    //      Read_cam_frame(loop, 0, AOconf[loop].WFSim.WFSnormalize, 0, 1);
                    clock_gettime(CLOCK_REALTIME, &t1);
                    timerinit = 1;
                }

#ifdef _PRINT_TEST
                printf("[%s] [%d]  AOloopControl_aorun: Starting AOcompute, AOconf[%d].WFSim.WFSnormalize = %d\n", __FILE__, __LINE__, loop, AOconf[loop].WFSim.WFSnormalize);
                fflush(stdout);
#endif


                // CTRLval = 5 will disable computations in loop (usually for testing)
                int doComputation = 1;
                if(data.processinfo == 1)
                    if(processinfo->CTRLval == 5) {
                        doComputation = 0;
                    }




                //
                // Most computation are performed inside AOcompute
                //
                // note: processinfo_exec_start is launched in Read_cam_frame() in AOcompute()
                //

                if(doComputation == 1) {
                    //processinfo_WriteMessage(processinfo, "start AOcompute");
                    AOcompute(loop, AOconf[loop].WFSim.WFSnormalize);
                    //processinfo_WriteMessage(processinfo, "end AOcompute");
                } else {
					//processinfo_WriteMessage(processinfo, "doComputation = 0");
                    processinfo_exec_start(processinfo);
                }


                clock_gettime(CLOCK_REALTIME, &functionTestTimerStart); //TEST timing in function


                AOconf[loop].AOtiminginfo.status = 12; // 12
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime, tnow);
                tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[19] = tdiffv;


                if(AOconf[loop].aorun.CMMODE == 0) { // 2-step : WFS -> mode coeffs -> DM act
                    if(AOconf[loop].aorun.DMprimaryWriteON == 1) { // if Writing to DM
                        if(doComputation == 1) {
                            if(fabs(AOconf[loop].aorun.gain) > 1.0e-6) {
                                set_DM_modes(loop);
                            }
                        }
                    }

                } else { // 1 step: WFS -> DM act
                    if(AOconf[loop].aorun.DMprimaryWriteON == 1) { // if Writing to DM
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 1;

                        for(ii = 0; ii < AOconf[loop].DMctrl.sizeDM; ii++) { //TEST
                            if(isnan(data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii]) != 0) {
                                printf("image aol2_meas_act  element %ld is NAN -> replacing by 0\n", ii);
                                data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii] = 0.0;
                            }
                        }


                        AOconf[loop].AOtiminginfo.status = 13; // enforce limits
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime, tnow);
                        tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;
                        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[20] = tdiffv;


                        if(doComputation == 1) {
                            for(ii = 0; ii < AOconf[loop].DMctrl.sizeDM; ii++) {
                                data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] -= AOconf[loop].aorun.gain * data.image[aoloopcontrol_var.aoconfID_meas_act].array.F[ii];

                                data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] *= AOconf[loop].aorun.mult;

                                if(data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] > AOconf[loop].aorun.maxlimit) {
                                    data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] = AOconf[loop].aorun.maxlimit;
                                }
                                if(data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] < -AOconf[loop].aorun.maxlimit) {
                                    data.image[aoloopcontrol_var.aoconfID_dmC].array.F[ii] = -AOconf[loop].aorun.maxlimit;
                                }
                            }
                        }

                        AOconf[loop].AOtiminginfo.status = 14; // write to DM
                        clock_gettime(CLOCK_REALTIME, &tnow);
                        tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime, tnow);
                        tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;
                        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[21] = tdiffv;

                        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_dmC, -1);
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0++;
                        data.image[aoloopcontrol_var.aoconfID_dmC].md[0].write = 0;

                        // inform dmdisp that new command is ready in one of the channels
                        if(aoloopcontrol_var.aoconfID_dmdisp != -1)
                            if(data.image[aoloopcontrol_var.aoconfID_dmdisp].md[0].sem > 1) {
                                sem_getvalue(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[1], &semval);
                                if(semval < SEMAPHORE_MAXVAL) {
                                    sem_post(data.image[aoloopcontrol_var.aoconfID_dmdisp].semptr[1]);
                                }
                            }
                        AOconf[loop].aorun.DMupdatecnt ++;
                    }
                }

                AOconf[loop].AOtiminginfo.status = 18; // 18
                clock_gettime(CLOCK_REALTIME, &tnow);
                tdiff = info_time_diff(data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime, tnow);
                tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;
                data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[22] = tdiffv;

                AOconf[loop].aorun.cnt++;


                AOconf[loop].aorun.LOOPiteration++;
                data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;




                // REAL TIME LOGGING
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt0 = AOconf[loop].aorun.cnt;
                data.image[aoloopcontrol_var.aoconfIDlogdata].md[0].cnt1 = AOconf[loop].aorun.LOOPiteration;
                data.image[aoloopcontrol_var.aoconfIDlogdata].array.F[0] = AOconf[loop].aorun.gain;


                if(AOconf[loop].aorun.cnt == AOconf[loop].aorun.cntmax) {
                    AOconf[loop].aorun.on = 0;
                }

                clock_gettime(CLOCK_REALTIME, &functionTestTimerEnd);


                tdiff = info_time_diff(functionTestTimerStart, functionTestTimerEnd);
                tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;
                double tdiffv02 = tdiffv;
                //TEST TIMING
                /*
                if(tdiffv > 30.0e-6)
                {
                	printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_aorun() - excluding AOcompute\n", tdiffv*1.0e6, (long) AOconf[loop].aorun.LOOPiteration);
                	fflush(stdout);
                }*/

                tdiff = info_time_diff(functionTestTimer00, functionTestTimerEnd);
                tdiffv = 1.0 * tdiff.tv_sec + 1.0e-9 * tdiff.tv_nsec;

                //TEST TIMING
                /*
                if(tdiffv > 600.0e-6)
                {
                	printf("TIMING WARNING: %12.3f us  %10ld   AOloopControl_aorun()\n", tdiffv*1.0e6, (long) AOconf[loop].aorun.LOOPiteration);
                	printf("    AOcompute()            read cam        : %12.3f us \n", tdiffv00*1.0e6);
                	printf("    AOcompute()            post read cam   : %12.3f us \n", tdiffv01*1.0e6);
                	printf("    AOloopControl_aorun()    post-AOcompute  : %12.3f us \n", tdiffv02*1.0e6);

                	fflush(stdout);


                }
                */


                processinfo_exec_end(processinfo);

                loopcnt++;
            }

            if(AOconf[loop].aorun.kill == 1) {
                loopOK = 0;
            }

        } // loop has been killed (normal exit)

        processinfo_cleanExit(processinfo);
    }

    free(thetime);

    // LOG function end
    CORE_logFunctionCall(logfunc_level, logfunc_level_max, 1, __FILE__, __func__, __LINE__, commentstring);

    return(0);
}























