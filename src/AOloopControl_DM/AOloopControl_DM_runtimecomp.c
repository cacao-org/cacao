/**
 * @file    AOloopControl_DM_runtimecomp.c
 * @brief   DM control
 * 
 * To be used for AOloopControl module
 *  
 * @author  O. Guyon
 * @date    10 Jul 2017
 *
 *
 * 
 */

#include <malloc.h>

#include <string.h>

#include <sched.h>

#include <math.h>

#include <sys/types.h>
#include <unistd.h>


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "00CORE/00CORE.h"
#include "statistic/statistic.h"

#include "AOloopControl_DM/AOloopControl_DM.h"


#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t){
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





//#define DMSTROKE100 0.7 // um displacement for 100V

extern long NB_DMindex ;

extern AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
extern int dmdispcomb_loaded ;
extern int SMfd;


extern AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
extern int dmturb_loaded ;
extern int SMturbfd;




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

// DM type
//
// 0 undef
// 1 linear    : V =     ( x / DMstroke ) * 100
// 2 quadratic : V = sqrt( x / DMstroke ) * 100
//


int AOloopControl_DM_disp2V(long DMindex)
{
    long ii;
    float volt;
    long IDvolt;
    
	static int qmapinit = 0;
	static char qmapname[100];
	static long IDqmap;

    int QUANTIZATION_RANDOM = 2; 
    // 1: remove quantization error by probabilistic value, recomputed for each new value
	// 2: remove quantization error by adding an external random map between 0 and 1, named dmXXquant
	//    USER SHOULD UPDATE THIS MAP WHEN REQUIRED
	
    IDvolt = dmdispcombconf[DMindex].IDvolt;

    data.image[IDvolt].md[0].write = 1;


	if(QUANTIZATION_RANDOM == 2)
	{
		if(qmapinit == 0)
		{
			uint32_t *sizearray;
			
			sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
			sizearray[0] = dmdispcombconf[DMindex].xsize;
			sizearray[1] = dmdispcombconf[DMindex].ysize;
			sprintf(qmapname, "dm%02ldquant", DMindex);
			IDqmap = create_image_ID(qmapname, 2, sizearray, _DATATYPE_FLOAT, 1, 10);
			free(sizearray);
			
			printf("ARRAY %s CREATED\n", qmapname);
			
			qmapinit = 1;
		}
	}

    if(dmdispcombconf[DMindex].voltON==1)
    {
        if(dmdispcombconf[DMindex].volttype == 1) // linear bipolar, output is float
        {
            for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
            {
                volt = 100.0*(data.image[dmdispcombconf[DMindex].IDdisp].array.F[ii]/dmdispcombconf[DMindex].stroke100);
                if(volt>dmdispcombconf[DMindex].MAXVOLT)
                    volt = dmdispcombconf[DMindex].MAXVOLT;
                if(volt<-dmdispcombconf[DMindex].MAXVOLT)
                    volt = -dmdispcombconf[DMindex].MAXVOLT;
                data.image[IDvolt].array.F[ii] = volt;
            }
        }
        else if (dmdispcombconf[DMindex].volttype == 2) // quadratic unipolar, output is UI16
        {
            for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
            {
                volt = 100.0*sqrt(data.image[dmdispcombconf[DMindex].IDdisp].array.F[ii]/dmdispcombconf[DMindex].stroke100);
                if(volt>dmdispcombconf[DMindex].MAXVOLT)
                    volt = dmdispcombconf[DMindex].MAXVOLT;

                if(QUANTIZATION_RANDOM == 1)
                {
                    float fval;
                    unsigned short int uval;
                    float fracval;

                    fval = volt/300.0*16384.0;
                    uval = (unsigned short int) (fval);
                    fracval = fval-uval; // between 0 and 1
                    
                    if(ran1()<fracval)
						uval++;
                    
                    data.image[IDvolt].array.UI16[ii] = uval;
                }
                else if (QUANTIZATION_RANDOM == 2)
                {
					data.image[IDvolt].array.UI16[ii] = (unsigned short int) (volt/300.0*16384.0 + data.image[IDqmap].array.F[ii]);
				}
                else
                    data.image[IDvolt].array.UI16[ii] = (unsigned short int) volt/300.0*16384.0;

            }
        }
    }
    else
    {
        if(dmdispcombconf[DMindex].volttype == 1) // linear bipolar, output is float
        {
            for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                data.image[IDvolt].array.F[ii] = 0;
        }

        if (dmdispcombconf[DMindex].volttype == 2)
        {
            for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                data.image[IDvolt].array.UI16[ii] = 0;
        }
    }

    data.image[IDvolt].md[0].write = 0;
    data.image[IDvolt].md[0].cnt0++;


    //    COREMOD_MEMORY_image_set_sempost(data.image[dmdispcombconf[DMindex].IDdisp].name, -1);
    COREMOD_MEMORY_image_set_sempost_byID(dmdispcombconf[DMindex].IDdisp, -1);
    COREMOD_MEMORY_image_set_sempost_byID(IDvolt, -1);

    return 0;
}









//
// manages configuration parameters
// initializes configuration parameters structure
//
int AOloopControl_DM_CombineChannels_FPCONF(
    uint32_t CMDmode,
    long DMindex
)
{
    // define function parameters
    int fpi;  // function parameter index
    int NBparam = FUNCTION_PARAMETER_NBPARAM_DEFAULT;
    char sname[200];
    char pname[200];

    void * pNull = NULL;


	if(data.processnameflag == 0)
		sprintf(sname, "DMcomb-%02ld", DMindex);
	else // Set fps name to be process name up to first instance of character '.'
		strcpy(sname, data.processname0);


    FUNCTION_PARAMETER_STRUCT fps;
    


    if(CMDmode & CMDCODE_CONFINIT) // (re-)create fps even if it exists
    {
        function_parameter_struct_create(NBparam, sname);
        function_parameter_struct_connect(sname, &fps);
    }
    else // load existing fps if exists
    {
        if(function_parameter_struct_connect(sname, &fps) == -1)
        {
            function_parameter_struct_create(NBparam, sname);
            function_parameter_struct_connect(sname, &fps);
        }
    }


    if( CMDmode & CMDCODE_CONFSTOP ) // stop fps
    {
        fps.md->signal &= ~FUNCTION_PARAMETER_STRUCT_SIGNAL_CONFRUN;
        function_parameter_struct_disconnect(&fps);
        return 0;
    }



    fpi = function_parameter_add_entry(&fps, "AOCONF.DMindex", "Deformable mirror index", FPTYPE_INT64, pNull);
    fps.parray[fpi].val.l[0] = DMindex;
    fps.parray[fpi].val.l[3] = DMindex;
    fps.parray[fpi].cnt0 = 1;
    fps.parray[fpi].status &= ~FPFLAG_WRITECONF;
    fps.parray[fpi].status &= ~FPFLAG_WRITERUN;
    fps.parray[fpi].status |= FPFLAG_MINLIMIT;
    fps.parray[fpi].val.l[1] = 0;  // min
    fps.parray[fpi].status |= FPFLAG_MAXLIMIT;
    fps.parray[fpi].val.l[2] = 9;  // max
    long fp_DMindex = fpi;


    fpi = function_parameter_add_entry(&fps, "AOCONF.DMxsize", "Deformable mirror X size", FPTYPE_INT64, pNull);
    fps.parray[fpi].val.l[1] = 1;  // min
    fps.parray[fpi].status |= FPFLAG_MINLIMIT;
    fps.parray[fpi].status &= ~FPFLAG_WRITERUN;
    long fp_xsize = fpi;


    fpi = function_parameter_add_entry(&fps, "AOCONF.DMysize", "Deformable mirror Y size", FPTYPE_INT64, pNull);
    fps.parray[fpi].val.l[1] = 1;  // min
    fps.parray[fpi].status |= FPFLAG_MINLIMIT;
    fps.parray[fpi].status &= ~FPFLAG_WRITERUN;
    long fp_ysize = fpi;


    long NBchannel_default = 12;
    fpi = function_parameter_add_entry(&fps, ".NBchannel", "Number of channels", FPTYPE_INT64, &NBchannel_default);
    fps.parray[fpi].val.l[1] = 1;  // min
    fps.parray[fpi].status |= FPFLAG_MINLIMIT;
    fps.parray[fpi].status &= ~FPFLAG_WRITERUN;
    long fp_NBchannel = fpi;


    long AveMode_default = 0;
    fpi = function_parameter_add_entry(&fps, ".AveMode", "Averaging mode", FPTYPE_INT64, &AveMode_default);
    fps.parray[fpi].val.l[1] = 0;  // min
    fps.parray[fpi].val.l[2] = 2;  // max
    fps.parray[fpi].status |= FPFLAG_MINLIMIT;
    fps.parray[fpi].status |= FPFLAG_MAXLIMIT;
    fps.parray[fpi].status &= ~FPFLAG_WRITERUN;
    long fp_AveMode = fpi;


    long dm2dm_mode_default = 0;
    long fp_dm2dm_mode = function_parameter_add_entry(&fps, ".option.dm2dm_mode", "DM to DM offset mode", FPTYPE_ONOFF, &dm2dm_mode_default);


    long fp_dm2dm_DMmodes = function_parameter_add_entry(&fps, ".option.dm2dm_DMmodes", "Output stream DM to DM", FPTYPE_STREAMNAME, pNull);
    
    long fp_dm2dm_outdisp = function_parameter_add_entry(&fps, ".option.dm2dm_outdisp", "data stream to which output DM is written", FPTYPE_STREAMNAME, pNull);



    long wfsrefmode_default = 0;
    long fp_wfsrefmode = function_parameter_add_entry(&fps, ".option.wfsrefmode", "WFS ref mode", FPTYPE_ONOFF, &wfsrefmode_default);

    long fp_wfsref_WFSRespMat = function_parameter_add_entry(&fps, ".option.wfsref_WFSRespMat", "Output WFS resp matrix", FPTYPE_STREAMNAME, pNull);

    long fp_wfsref_out = function_parameter_add_entry(&fps, ".option.wfsref_out", "Output WFS", FPTYPE_STREAMNAME, pNull);



    long voltmode_default = 0;
    long fp_voltmode = function_parameter_add_entry(&fps, ".option.voltmode", "Volt mode", FPTYPE_ONOFF, &voltmode_default);

    long fp_volttype = function_parameter_add_entry(&fps, ".option.volttype", "Volt type", FPTYPE_INT64, pNull);

    long fp_stroke100 = function_parameter_add_entry(&fps, ".option.stroke100", "Stroke for 100 V [um]", FPTYPE_FLOAT64, pNull);

    long fp_voltname = function_parameter_add_entry(&fps, ".option.voltname", "Stream name for volt output", FPTYPE_STREAMNAME, pNull);

    double DClevel_default = 0.0;
    long fp_DClevel = function_parameter_add_entry(&fps, ".option.DClevel", "DC level [um]", FPTYPE_FLOAT64, &DClevel_default);

    long fp_maxvolt = function_parameter_add_entry(&fps, ".option.maxvolt", "Maximum voltage", FPTYPE_FLOAT64, pNull);

	// status (RO)
    fpi = function_parameter_add_entry(&fps, ".status.loopcnt", "Loop counter", FPTYPE_INT64, pNull);
    fps.parray[fpi].status = FPFLAG_DFT_STATUS;
    long fp_loopcnt = fpi;







    // update on first loop iteration
    fps.md->signal |= FUNCTION_PARAMETER_STRUCT_SIGNAL_UPDATE;

    fps.md->confpid = getpid();

    if( CMDmode & CMDCODE_CONFSTART )  // parameter configuration loop
    {
		fps.md->signal |= FUNCTION_PARAMETER_STRUCT_SIGNAL_CONFRUN;
		
        fps.md->confpid = getpid();


        while(fps.md->signal & FUNCTION_PARAMETER_STRUCT_SIGNAL_CONFRUN)
        {

			// Test if CONF process is running
            if((getpgid(fps.md->confpid) >= 0)&&(fps.md->confpid>0))
                fps.md->status |= FUNCTION_PARAMETER_STRUCT_STATUS_CONF;
            else
                fps.md->status &= ~FUNCTION_PARAMETER_STRUCT_STATUS_CONF;

			// Test if RUN process is running
            if((getpgid(fps.md->runpid) >= 0)&&(fps.md->runpid>0))
                fps.md->status |= FUNCTION_PARAMETER_STRUCT_STATUS_RUN;
            else
                fps.md->status &= ~FUNCTION_PARAMETER_STRUCT_STATUS_RUN;




            if( fps.md->signal & FUNCTION_PARAMETER_STRUCT_SIGNAL_UPDATE ) // GUI to update display
            {

                // Here we insert logical links between parameters

                if ( fps.parray[fp_dm2dm_mode].status & FPFLAG_ONOFF )  // ON state
                {
                    fps.parray[fp_dm2dm_DMmodes].status |= FPFLAG_USED;
                    fps.parray[fp_dm2dm_outdisp].status |= FPFLAG_USED;
                    fps.parray[fp_dm2dm_DMmodes].status |= FPFLAG_VISIBLE;
                    fps.parray[fp_dm2dm_outdisp].status |= FPFLAG_VISIBLE;
                }
                else // OFF state
                {
                    fps.parray[fp_dm2dm_DMmodes].status &= ~FPFLAG_USED;
                    fps.parray[fp_dm2dm_outdisp].status &= ~FPFLAG_USED;
                    fps.parray[fp_dm2dm_DMmodes].status &= ~FPFLAG_VISIBLE;
                    fps.parray[fp_dm2dm_outdisp].status &= ~FPFLAG_VISIBLE;
                }


                if ( fps.parray[fp_wfsrefmode].status & FPFLAG_ONOFF ) // ON state
                {
                    fps.parray[fp_wfsref_WFSRespMat].status |= FPFLAG_USED;
                    fps.parray[fp_wfsref_WFSRespMat].status |= FPFLAG_VISIBLE;
                    fps.parray[fp_wfsref_out].status |= FPFLAG_USED;
                    fps.parray[fp_wfsref_out].status |= FPFLAG_VISIBLE;
                }
                else // OFF state
                {
                    fps.parray[fp_wfsref_WFSRespMat].status &= ~FPFLAG_USED;
                    fps.parray[fp_wfsref_WFSRespMat].status &= ~FPFLAG_VISIBLE;
                    fps.parray[fp_wfsref_out].status &= ~FPFLAG_USED;
                    fps.parray[fp_wfsref_out].status &= ~FPFLAG_VISIBLE;
                }



                if ( fps.parray[fp_voltmode].status & FPFLAG_ONOFF ) // ON state
                {
                    fps.parray[fp_voltname].status |= FPFLAG_USED;
                    fps.parray[fp_voltname].status |= FPFLAG_VISIBLE;
                }
                else // OFF state
                {
                    fps.parray[fp_voltname].status &= ~FPFLAG_USED;
                    fps.parray[fp_voltname].status &= ~FPFLAG_VISIBLE;
                }


                functionparameter_CheckParametersAll(&fps);  // check all parameter values
                
                fps.md->signal &= ~FUNCTION_PARAMETER_STRUCT_SIGNAL_UPDATE;

            }

            usleep(fps.md->confwaitus);
        }


        fps.md->confpid = 0;
    }

    fps.md->status &= ~FUNCTION_PARAMETER_STRUCT_STATUS_CONF;
    function_parameter_struct_disconnect(&fps);


    return(0);
}










//
// DMindex is a unique DM identifier (0-9), so multiple instances can coexist
//
// xsize, ysize is DM pixel size
//
// NBchannel : number of channels. All channels co-added
//
// AveMode: averaging mode
//      0: do not appy DC offset command to average, but offset combined average to mid-range, and clip displacement at >0.0
//      1: apply DC offset to remove average
//      2: do not apply DC offset, do not offset sum, do not clip
//
// NOTE: DM displacement is biased to mid displacement
// NOTE: responds immediately to sem[1] in dmdisp
// dmdisp files have 10 semaphores
//
// dm2dm_mode: 1 if this DM controls an output DM
//
// dm2dm_DMmodes: data cube containting linear relationship between current DM and the output DM it controls
//
// dm2dm_outdisp: data stream to which output DM is written
//
// wfsrefmode: 1 if offset to WFS reference
//
// wfsref_WFSRespMat: response matrix of output loop
//
// wfsref_out : output wfsreference
//
// voltmode = 1 if DM volt computed
//
// IDvolt_name : name of DM volt stream
//
// maxvolt: maximum volt for DM volt
//


int AOloopControl_DM_CombineChannels_RUN(
    long DMindex
)
{
    uint8_t naxis = 2;
    uint32_t *size;
    long ch;
    char name[200];
    long cnt = 0;
    long long cntold;
    long long cntsumold;
    long long cntsum;
    long ii;
    long IDdisp;
    long IDvolt;
    double ave;
    long ID1;
    int RT_priority = 95; //any number from 0-99
    struct sched_param schedpar;
    int r;
    long sizexy;
    float *dmdispptr;
    float *dmdispptr_array[20];
    long IDdispt;
    char sname[200];

    int vOK;
    float maxmaxvolt = 150.0;
    char errstr[200];
    int semnb, semval;
    long sizexyDMout;
    long IDtmpoutdm;
    long kk;
    long sizexywfsref;
    long IDtmpoutref;
    long cntch;

    long IDvar;
    long DMtwaitus = 0; // optional time interval between successive commands [us]
    // if 0, do not wait
    // read from variable name DMTWAIT


    // timing
    struct timespec ttrig;
    struct timespec t1;
    struct timespec tnow;
    struct timespec tdiff;
    double tdiffv;

    int DMupdate;






	// manage configuration parameters
	if(data.processnameflag == 0)
		sprintf(sname, "DMcomb-%02ld", DMindex);
	else
		strcpy(sname, data.processname0);
	
	FUNCTION_PARAMETER_STRUCT fps;
	
	if(function_parameter_struct_connect(sname, &fps) == -1)
	{
		printf("ERROR: fps \"%s\" does not exist\n", sname);
		exit(0);
	}

	fps.md->runpid = getpid();
	
	// GET FUNCTION PARAMETER VALUES
	long xsize = functionparameter_GetParamValue_INT64(&fps, "AOCONF.DMxsize");
	long ysize = functionparameter_GetParamValue_INT64(&fps, "AOCONF.DMysize");	
	int NBchannel = functionparameter_GetParamValue_INT64(&fps, "NBchannel");	
	int AveMode = functionparameter_GetParamValue_INT64(&fps, ".AveMode");		
	
	int dm2dm_mode = functionparameter_GetParamValue_INT64(&fps, ".option.dm2dm_mode");			
	char dm2dm_DMmodes[FUNCTION_PARAMETER_STRMAXLEN];
    char dm2dm_outdisp[FUNCTION_PARAMETER_STRMAXLEN];
	if(dm2dm_mode == 1)
	{
		strncpy(dm2dm_DMmodes, functionparameter_GetParamPtr_STRING(&fps, ".option.dm2dm_DMmodes"), FUNCTION_PARAMETER_STRMAXLEN);
		strncpy(dm2dm_outdisp, functionparameter_GetParamPtr_STRING(&fps, ".option.dm2dm_outdisp"), FUNCTION_PARAMETER_STRMAXLEN);
	}
	
	
	int wfsrefmode = functionparameter_GetParamValue_INT64(&fps, ".option.wfsrefmode");		
    char wfsref_WFSRespMat[FUNCTION_PARAMETER_STRMAXLEN];
    char wfsref_out[FUNCTION_PARAMETER_STRMAXLEN];
	if(wfsrefmode == 1)
	{
		strncpy(wfsref_WFSRespMat, functionparameter_GetParamPtr_STRING(&fps, ".option.wfsref_WFSRespMat"), FUNCTION_PARAMETER_STRMAXLEN);
		strncpy(wfsref_out, functionparameter_GetParamPtr_STRING(&fps, ".option.wfsref_out"), FUNCTION_PARAMETER_STRMAXLEN);
	}

	
	int voltmode = functionparameter_GetParamValue_ONOFF(&fps, ".option.voltmode");		
	
    char IDvolt_name[FUNCTION_PARAMETER_STRMAXLEN];
	if(voltmode == 1)
	{
		strncpy(IDvolt_name, functionparameter_GetParamPtr_STRING(&fps, ".option.voltname"), FUNCTION_PARAMETER_STRMAXLEN);
	}


	int volttype = functionparameter_GetParamValue_INT64(&fps, ".option.volttype");

		
	float stroke100 = functionparameter_GetParamValue_FLOAT64(&fps, ".option.stroke100");
	float DClevel = functionparameter_GetParamValue_FLOAT64(&fps, ".option.DClevel");
	float maxvolt = functionparameter_GetParamValue_FLOAT64(&fps, ".option.maxvolt");
	
	
	// STATUS
	long *addr_loopcnt = functionparameter_GetParamPtr_INT64(&fps, ".status.loopcnt");












    PROCESSINFO *processinfo;
    if(data.processinfo==1)
    {
        // CREATE PROCESSINFO ENTRY
        // see processtools.c in module CommandLineInterface for details
        //
        
        char pinfoname[200];
        char pinfostring[200];
        
        sprintf(pinfoname, "dm%02ld-comb", DMindex);
        processinfo = processinfo_shm_create(pinfoname, 0);
        
        
        strcpy(processinfo->source_FUNCTION, __FUNCTION__);
        strcpy(processinfo->source_FILE,     __FILE__);
        processinfo->source_LINE = __LINE__;
        
        processinfo->loopstat = 0; // loop initialization

        char msgstring[200];
        sprintf(msgstring, "DMindex %ld (%ld x %ld), %d channels", DMindex, xsize, ysize, NBchannel);
		processinfo_WriteMessage(processinfo, msgstring);
    }



    if(DMindex>NB_DMindex-1)
    {
        printf("ERROR: requested DMindex (%02ld) exceeds maximum number of DMs (%02ld)\n", DMindex, NB_DMindex);
        exit(0);
    }


    printf("Setting up DM #%ld\n", DMindex);

    list_variable_ID();
    IDvar = variable_ID("DMTWAIT");
    if(IDvar!=-1)
        DMtwaitus = (long) (data.variable[IDvar].value.f);
    printf("Using DMtwaitus = %ld us\n", DMtwaitus);


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    r = seteuid(data.euid); // This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid); //Go back to normal privileges
#endif

    // AOloopControl_DM_createconf();



    AOloopControl_DM_loadconf();

    dmdispcombconf[DMindex].ON = 1;
    dmdispcombconf[DMindex].xsize = xsize;
    dmdispcombconf[DMindex].ysize = ysize;
    dmdispcombconf[DMindex].xysize = xsize*ysize;
    dmdispcombconf[DMindex].NBchannel = NBchannel;
    dmdispcombconf[DMindex].voltmode = voltmode;
    dmdispcombconf[DMindex].volttype = volttype;
    dmdispcombconf[DMindex].stroke100 = stroke100;
    dmdispcombconf[DMindex].voltON = 1;
    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    dmdispcombconf[DMindex].AveMode = AveMode;
    sprintf(dmdispcombconf[DMindex].voltname, "%s", IDvolt_name);
    dmdispcombconf[DMindex].status = 0;

    dmdispcombconf[DMindex].DClevel = DClevel; //0.5*(DMSTROKE100*dmdispcombconf[DMindex].MAXVOLT/100.0*dmdispcombconf[DMindex].MAXVOLT/100.0);

    printf("maxvolt = %f\n", maxvolt);


    size = (uint32_t*) malloc(sizeof(uint32_t)*naxis);
    size[0] = xsize;
    size[1] = ysize;
    sizexy = xsize*ysize;


    dmdispcombconf[DMindex].xsizeout = 0;
    dmdispcombconf[DMindex].ysizeout = 0;

    dmdispcombconf[DMindex].dm2dm_mode = dm2dm_mode;


    if(dm2dm_mode == 1)
    {
        printf("INITIALIZATION AND VERIFICATION FOR dm2dm MODE ...\n");
        fflush(stdout);

        dmdispcombconf[DMindex].ID_dm2dm_DMmodes = image_ID(dm2dm_DMmodes);
        sprintf(dmdispcombconf[DMindex].dm2dm_DMmodes_name, "%s", dm2dm_DMmodes);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].naxis != 3)
        {
            sprintf(errstr, "image \"%s\" should have naxis = 3", dm2dm_DMmodes);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        dmdispcombconf[DMindex].xsizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[0];
        dmdispcombconf[DMindex].ysizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[1];

        dmdispcombconf[DMindex].ID_dm2dm_outdisp = image_ID(dm2dm_outdisp);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[0] != dmdispcombconf[DMindex].xsizeout)
        {
            sprintf(errstr, "image \"%s\" should have x axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].xsizeout);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[1] != dmdispcombconf[DMindex].ysizeout)
        {
            sprintf(errstr, "image \"%s\" should have y axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].ysizeout);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }

        IDtmpoutdm = create_2Dimage_ID("_tmpoutdm", dmdispcombconf[DMindex].xsizeout, dmdispcombconf[DMindex].ysizeout);
        sizexyDMout = dmdispcombconf[DMindex].xsizeout*dmdispcombconf[DMindex].ysizeout;
        printf("done\n\n");
        fflush(stdout);
    }



    list_image_ID(); //TEST

    dmdispcombconf[DMindex].wfsrefmode = wfsrefmode;
    if(wfsrefmode == 1)
    {
        printf("INITIALIZATION AND VERIFICATION FOR wfsref MODE ...\n");
        fflush(stdout);

        printf("wfsref_WFSRespMat = %s\n", wfsref_WFSRespMat);
        fflush(stdout);

        dmdispcombconf[DMindex].ID_wfsref_RespMat = image_ID(wfsref_WFSRespMat);
        if(dmdispcombconf[DMindex].ID_wfsref_RespMat==-1)
        {
            printf("ERROR: cannot find image \"%s\"\n", wfsref_WFSRespMat);
            exit(0);
        }

        if(data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].naxis != 3)
        {
            sprintf(errstr, "image \"%s\" should have naxis = 3", wfsref_WFSRespMat);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        dmdispcombconf[DMindex].xsizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[0];
        dmdispcombconf[DMindex].ysizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[1];

        printf("xsizewfsref = %ld\n", dmdispcombconf[DMindex].xsizewfsref);
        printf("ysizewfsref = %ld\n", dmdispcombconf[DMindex].ysizewfsref);
        fflush(stdout);

        dmdispcombconf[DMindex].ID_wfsref_out = image_ID(wfsref_out);
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[0] != dmdispcombconf[DMindex].xsizewfsref)
        {
            sprintf(errstr, "image \"%s\" should have x axis = %ld", wfsref_out, dmdispcombconf[DMindex].xsizewfsref);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[1] != dmdispcombconf[DMindex].ysizewfsref)
        {
            sprintf(errstr, "image \"%s\" should have y axis = %ld", wfsref_out, dmdispcombconf[DMindex].ysizewfsref);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        printf("Creating image %s   %ld x %ld\n", "_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        fflush(stdout);
        IDtmpoutref = create_2Dimage_ID("_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        sizexywfsref = dmdispcombconf[DMindex].xsizewfsref*dmdispcombconf[DMindex].ysizewfsref;

        COREMOD_MEMORY_image_set_createsem(wfsref_out, 10);

        printf("done\n\n");
        fflush(stdout);
    }

    printf("Initialize channels\n");
    printf("Max DM stroke = %f um\n", dmdispcombconf[DMindex].stroke100*dmdispcombconf[DMindex].MAXVOLT/100.0*dmdispcombconf[DMindex].MAXVOLT/100.0);
    fflush(stdout);

    for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
    {
        sprintf(name, "dm%02lddisp%02ld", DMindex, ch);
        printf("Channel %ld \n", ch);
        dmdispcombconf[DMindex].dmdispID[ch] = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
        COREMOD_MEMORY_image_set_createsem(name, 10);
        dmdispptr_array[ch] = data.image[dmdispcombconf[DMindex].dmdispID[ch]].array.F;
    }


    sprintf(name, "dm%02lddisp", DMindex);
    dmdispcombconf[DMindex].IDdisp = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    sprintf(name, "dm%02lddispt", DMindex);
    IDdispt = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 0, 0);
    dmdispptr = data.image[IDdispt].array.F;

    if(dmdispcombconf[DMindex].voltmode==1)
    {
        IDvolt = image_ID(dmdispcombconf[DMindex].voltname);

        vOK = 0;
        if(IDvolt!=-1)
        {
            if((data.image[IDvolt].md[0].naxis==2)&&(data.image[IDvolt].md[0].size[0]==xsize)&&(data.image[IDvolt].md[0].size[1]==ysize))
            {
                if((dmdispcombconf[DMindex].volttype==1)&&(data.image[IDvolt].md[0].atype==_DATATYPE_FLOAT))
                    vOK = 1;
                if((dmdispcombconf[DMindex].volttype==2)&&(data.image[IDvolt].md[0].atype==_DATATYPE_UINT16))
                    vOK = 1;
                if(vOK==0)
                    delete_image_ID(dmdispcombconf[DMindex].voltname);
            }
        }

        printf("vOK = %d\n", vOK);
        if(vOK==0)
        {
            printf("CREATING stream %s  %d axis, size = %u x %u\n", dmdispcombconf[DMindex].voltname, naxis, size[0], size[1]);

            if(dmdispcombconf[DMindex].volttype==1)
                dmdispcombconf[DMindex].IDvolt = create_image_ID(dmdispcombconf[DMindex].voltname, naxis, size, _DATATYPE_FLOAT, 1, 10);

            if(dmdispcombconf[DMindex].volttype==2)
                dmdispcombconf[DMindex].IDvolt = create_image_ID(dmdispcombconf[DMindex].voltname, naxis, size, _DATATYPE_UINT16, 1, 10);
            COREMOD_MEMORY_image_set_createsem(dmdispcombconf[DMindex].voltname, 10);
        }
        else
            dmdispcombconf[DMindex].IDvolt = image_ID(dmdispcombconf[DMindex].voltname);
    }

    cntsumold = 0;


    dmdispcombconf[0].status = 1;

    sprintf(name, "dm%02lddisp", DMindex);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem<2)
    {
        printf("ERROR: image %s semaphore %d missing\n", data.image[dmdispcombconf[DMindex].IDdisp].name, 1);
        exit(0);
    }

    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    if(dmdispcombconf[DMindex].MAXVOLT>maxmaxvolt)
        dmdispcombconf[DMindex].MAXVOLT = maxvolt;


    AOloopControl_printDMconf();





    int loopCTRLexit = 0; // toggles to 1 when loop is set to exit cleanly
    if(data.processinfo==1)
        processinfo->loopstat = 1;


    while(dmdispcombconf[DMindex].ON == 1)
    {
        struct timespec semwaitts;

        if(data.processinfo==1)
        {
            while(processinfo->CTRLval == 1)  // pause
                usleep(50);

            if(processinfo->CTRLval == 2) // single iteration
                processinfo->CTRLval = 1;

            if(processinfo->CTRLval == 3) // exit loop
                loopCTRLexit = 1;
        }



        dmdispcombconf[DMindex].status = 2;

        if(DMtwaitus>0)
            usleep(DMtwaitus);

        if (clock_gettime(CLOCK_REALTIME, &semwaitts) == -1) {
            perror("clock_gettime");
            exit(EXIT_FAILURE);
        }

        semwaitts.tv_nsec += dmdispcombconf[DMindex].nsecwait;
        if(semwaitts.tv_nsec >= 1000000000)
            semwaitts.tv_sec = semwaitts.tv_sec + 1;

        DMupdate = 0;

        if(dmdispcombconf[DMindex].TrigMode==0)
        {
            //
            // this is semaphore that triggers the write to the DM
            //
            sem_timedwait(data.image[dmdispcombconf[DMindex].IDdisp].semptr[1], &semwaitts);

            cntsum = 0;
            for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            {
                cntch = data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
                dmdispcombconf[DMindex].dmdispcnt[ch] = cntch;
                cntsum += data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
            }
            if(cntsum != cntsumold)
                DMupdate = 1;
        }
        else
        {
            sem_timedwait(data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].semptr[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigSem]], &semwaitts);
            cnt = data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].md[0].cnt0;
            if(cnt!=cntold)
            {
                DMupdate = 1;
                cntold=cnt;
            }
        }

        if(DMupdate==1)
        {
            clock_gettime(CLOCK_REALTIME, &ttrig);

            dmdispcombconf[0].status = 3;
            cnt++;

            memcpy (data.image[IDdispt].array.F, dmdispptr_array[0], sizeof(float)*sizexy);
            for(ch=1; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            {
                for(ii=0; ii<sizexy; ii++)
                    dmdispptr[ii] += dmdispcombconf[DMindex].dmdispgain[ch]*dmdispptr_array[ch][ii];
            }

            dmdispcombconf[DMindex].status = 4;


            ave = 0.0;
            if(dmdispcombconf[DMindex].AveMode == 1) // REMOVE AVERAGE
            {
                for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                    ave += data.image[IDdispt].array.F[ii];
                ave /= dmdispcombconf[DMindex].xysize;
            }
            dmdispcombconf[DMindex].status = 5;

            if(dmdispcombconf[DMindex].AveMode < 2) // OFFSET BY DClevel
            {
                for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                {
                    data.image[IDdispt].array.F[ii] += (dmdispcombconf[DMindex].DClevel - ave);

                    // remove negative values
                    if(dmdispcombconf[DMindex].voltmode==1)
                        if(data.image[IDdispt].array.F[ii]<0.0)
                            data.image[IDdispt].array.F[ii] = 0.0;
                }
            }
            dmdispcombconf[DMindex].status = 6;

            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 1;
            memcpy (data.image[dmdispcombconf[DMindex].IDdisp].array.F,data.image[IDdispt].array.F, sizeof(float)*data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].cnt0++;
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 0;

            /*     for(semnb=0;semnb<data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem;semnb++)
                    {
                        sem_getvalue(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb], &semval);
                        if(semval<SEMAPHORE_MAXVAL)
                        sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb]);
                     }*/
            COREMOD_MEMORY_image_set_sempost_byID(dmdispcombconf[DMindex].IDdisp, -1);
            //      sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[0]);


            if(dm2dm_mode==1)
            {
                memset(data.image[IDtmpoutdm].array.F, '\0', sizeof(float)*sizexyDMout);
                for(kk=0; kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement; kk++)
                {
                    for(ii=0; ii<sizexyDMout; ii++)
                        data.image[IDtmpoutdm].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].array.F[kk*sizexyDMout+ii];
                }

                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].array.F,data.image[IDtmpoutdm].array.F, sizeof(float)*sizexyDMout);
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 0;
                sem_post(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].semptr[0]);
            }


            if(wfsrefmode==1)
            {
                memset(data.image[IDtmpoutref].array.F, '\0', sizeof(float)*sizexywfsref);
                list_image_ID();
                printf("kkmax = %ld\n", data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
                printf("iimax = %ld\n", sizexywfsref);
                printf("ID RespMat = %ld  (%ld)\n", dmdispcombconf[DMindex].ID_wfsref_RespMat, (data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement-1)*sizexywfsref + sizexywfsref-1);
                fflush(stdout);
                save_fits(wfsref_WFSRespMat, "!_test_wfsref_WFSRespMat.fits");
                for(kk=0; kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement; kk++)
                {
                    printf("(%ld %g) ", kk, data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk]);
                    for(ii=0; ii<sizexywfsref; ii++)
                        data.image[IDtmpoutref].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].array.F[kk*sizexywfsref+ii];
                }
                printf("\n");
                printf("Updating Zero Point  %ld <- %ld\n", dmdispcombconf[DMindex].ID_wfsref_out, IDtmpoutref);
                fflush(stdout);
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_wfsref_out].array.F,data.image[IDtmpoutref].array.F, sizeof(float)*sizexywfsref);
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 0;
                sem_post(data.image[dmdispcombconf[DMindex].ID_wfsref_out].semptr[0]);
                printf("Done\n");
                fflush(stdout);
            }



            dmdispcombconf[DMindex].status = 7;


            clock_gettime(CLOCK_REALTIME, &t1);
            if(dmdispcombconf[DMindex].voltmode==1)
                AOloopControl_DM_disp2V(DMindex);


            dmdispcombconf[DMindex].status = 8;

            cntsumold = cntsum;
            dmdispcombconf[DMindex].updatecnt++;
			*addr_loopcnt = dmdispcombconf[DMindex].updatecnt;

            if(data.processinfo==1)
                processinfo->loopcnt = dmdispcombconf[DMindex].updatecnt;

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = time_diff(ttrig, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            dmdispcombconf[DMindex].tdelay = tdiffv;


            tdiff = time_diff(t1, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            dmdispcombconf[DMindex].time_disp2V = tdiffv;

        }

        if((data.signal_INT == 1)||(data.signal_TERM == 1)||(data.signal_ABRT==1)||(data.signal_BUS==1)||(data.signal_SEGV==1)||(data.signal_HUP==1)||(data.signal_PIPE==1))
        {
            if(data.processinfo==1)
            {
                struct timespec tstop;
                struct tm *tstoptm;
                char msgstring[200];
                char timestring[200];


                clock_gettime(CLOCK_REALTIME, &tstop);
                tstoptm = gmtime(&tstop.tv_sec);
                sprintf(timestring, "%02d:%02d:%02d.%03d", tstoptm->tm_hour, tstoptm->tm_min, tstoptm->tm_sec, (int) (0.000001*tstop.tv_nsec));

                sprintf(msgstring, "Unknown signal at %s", timestring); // default

                if(data.signal_INT == 1)
                    sprintf(msgstring, "SIGINT at %s", timestring);

                if(data.signal_TERM == 1)
                    sprintf(msgstring, "SIGTERM at %s", timestring);

                if(data.signal_ABRT == 1)
                    sprintf(msgstring, "SIGABRT at %s", timestring);

                if(data.signal_BUS == 1)
                    sprintf(msgstring, "SIGBUS at %s", timestring);

                if(data.signal_SEGV == 1)
                    sprintf(msgstring, "SIGSEGV at %s", timestring);

                if(data.signal_HUP == 1)
                    sprintf(msgstring, "SIGHUP at %s", timestring);

                if(data.signal_PIPE == 1)
                    sprintf(msgstring, "SIGPIPE at %s", timestring);

                //strncpy(processinfo->statusmsg, msgstring, 200);
                
                processinfo_WriteMessage(processinfo, msgstring);
                processinfo->loopstat = 3; // clean exit
            }

            dmdispcombconf[DMindex].ON = 0;
           // exit(0);
        }

        if(loopCTRLexit == 1)
        {
            dmdispcombconf[DMindex].ON = 0;
            if(data.processinfo==1)
				processinfo->loopstat = 3;
        }
    }

    //  if(voltmode==1)
    //    arith_image_zero(dmdispcombconf[DMindex].voltname);


    if(data.processinfo==1)
        processinfo_cleanExit(processinfo);
    
    
    

    printf("LOOP STOPPED\n");
    fflush(stdout);

    free(size);


    return 0;
}












//
// DMindex is a unique DM identifier (0-9), so multiple instances can coexist
//
// xsize, ysize is DM pixel size
//
// NBchannel : number of channels. All channels co-added
//
// AveMode: averaging mode
//      0: do not appy DC offset command to average, but offset combined average to mid-range, and clip displacement at >0.0
//      1: apply DC offset to remove average
//      2: do not apply DC offset, do not offset sum, do not clip
//
// NOTE: DM displacement is biased to mid displacement
// NOTE: responds immediately to sem[1] in dmdisp
// dmdisp files have 10 semaphores
//
// dm2dm_mode: 1 if this DM controls an output DM
//
// dm2dm_DMmodes: data cube containting linear relationship between current DM and the output DM it controls
//
// dm2dm_outdisp: data stream to which output DM is written
//
// wfsrefmode: 1 if offset to WFS reference
//
// wfsref_WFSRespMat: response matrix of output loop
//
// wfsref_out : output wfsreference
//
// voltmode = 1 if DM volt computed
//
// IDvolt_name : name of DM volt stream
//
// maxvolt: maximum volt for DM volt
//


int AOloopControl_DM_CombineChannels(
    long DMindex,
    long xsize,
    long ysize,
    int NBchannel,
    int AveMode,
    int dm2dm_mode,
    const char *dm2dm_DMmodes,
    const char *dm2dm_outdisp,
    int wfsrefmode,
    const char *wfsref_WFSRespMat,
    const char *wfsref_out,
    int voltmode,
    int volttype,
    float stroke100,
    const char *IDvolt_name,
    float DClevel,
    float maxvolt
)
{
    uint8_t naxis = 2;
    uint32_t *size;
    long ch;
    char name[200];
    long cnt = 0;
    long long cntold;
    long long cntsumold;
    long long cntsum;
    long ii;
    long IDdisp;
    long IDvolt;
    double ave;
    long ID1;
    int RT_priority = 95; //any number from 0-99
    struct sched_param schedpar;
    int r;
    long sizexy;
    float *dmdispptr;
    float *dmdispptr_array[20];
    long IDdispt;
    char sname[200];

    int vOK;
    float maxmaxvolt = 150.0;
    char errstr[200];
    int semnb, semval;
    long sizexyDMout;
    long IDtmpoutdm;
    long kk;
    long sizexywfsref;
    long IDtmpoutref;
    long cntch;

    long IDvar;
    long DMtwaitus = 0; // optional time interval between successive commands [us]
    // if 0, do not wait
    // read from variable name DMTWAIT


    // timing
    struct timespec ttrig;
    struct timespec t1;
    struct timespec tnow;
    struct timespec tdiff;
    double tdiffv;

    int DMupdate;


	// manage configuration parameters
	//AOloopControl_DM_CombineChannels_FPCONF(FUNCTION_PARAMETER_CMD_RUNLOOP, DMindex);



    PROCESSINFO *processinfo;
    if(data.processinfo==1)
    {
        // CREATE PROCESSINFO ENTRY
        // see processtools.c in module CommandLineInterface for details
        //
        
        char pinfoname[200];
        char pinfostring[200];
        
        sprintf(pinfoname, "dm%02ld-comb", DMindex);
        processinfo = processinfo_shm_create(pinfoname, 0);
        
        
        strcpy(processinfo->source_FUNCTION, __FUNCTION__);
        strcpy(processinfo->source_FILE,     __FILE__);
        processinfo->source_LINE = __LINE__;
        
        processinfo->loopstat = 0; // loop initialization

        char msgstring[200];
        sprintf(msgstring, "DMindex %ld (%ld x %ld), %d channels", DMindex, xsize, ysize, NBchannel);
		processinfo_WriteMessage(processinfo, msgstring);
    }



    if(DMindex>NB_DMindex-1)
    {
        printf("ERROR: requested DMindex (%02ld) exceeds maximum number of DMs (%02ld)\n", DMindex, NB_DMindex);
        exit(0);
    }


    printf("Setting up DM #%ld\n", DMindex);

    list_variable_ID();
    IDvar = variable_ID("DMTWAIT");
    if(IDvar!=-1)
        DMtwaitus = (long) (data.variable[IDvar].value.f);
    printf("Using DMtwaitus = %ld us\n", DMtwaitus);


    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    r = seteuid(data.euid); // This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid); //Go back to normal privileges
#endif

    // AOloopControl_DM_createconf();

    AOloopControl_DM_loadconf();

    dmdispcombconf[DMindex].ON = 1;
    dmdispcombconf[DMindex].xsize = xsize;
    dmdispcombconf[DMindex].ysize = ysize;
    dmdispcombconf[DMindex].xysize = xsize*ysize;
    dmdispcombconf[DMindex].NBchannel = NBchannel;
    dmdispcombconf[DMindex].voltmode = voltmode;
    dmdispcombconf[DMindex].volttype = volttype;
    dmdispcombconf[DMindex].stroke100 = stroke100;
    dmdispcombconf[DMindex].voltON = 1;
    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    dmdispcombconf[DMindex].AveMode = AveMode;
    sprintf(dmdispcombconf[DMindex].voltname, "%s", IDvolt_name);
    dmdispcombconf[DMindex].status = 0;

    dmdispcombconf[DMindex].DClevel = DClevel; //0.5*(DMSTROKE100*dmdispcombconf[DMindex].MAXVOLT/100.0*dmdispcombconf[DMindex].MAXVOLT/100.0);

    printf("maxvolt = %f\n", maxvolt);


    size = (uint32_t*) malloc(sizeof(uint32_t)*naxis);
    size[0] = xsize;
    size[1] = ysize;
    sizexy = xsize*ysize;


    dmdispcombconf[DMindex].xsizeout = 0;
    dmdispcombconf[DMindex].ysizeout = 0;

    dmdispcombconf[DMindex].dm2dm_mode = dm2dm_mode;


    if(dm2dm_mode == 1)
    {
        printf("INITIALIZATION AND VERIFICATION FOR dm2dm MODE ...\n");
        fflush(stdout);

        dmdispcombconf[DMindex].ID_dm2dm_DMmodes = image_ID(dm2dm_DMmodes);
        sprintf(dmdispcombconf[DMindex].dm2dm_DMmodes_name, "%s", dm2dm_DMmodes);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].naxis != 3)
        {
            sprintf(errstr, "image \"%s\" should have naxis = 3", dm2dm_DMmodes);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        dmdispcombconf[DMindex].xsizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[0];
        dmdispcombconf[DMindex].ysizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[1];

        dmdispcombconf[DMindex].ID_dm2dm_outdisp = image_ID(dm2dm_outdisp);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[0] != dmdispcombconf[DMindex].xsizeout)
        {
            sprintf(errstr, "image \"%s\" should have x axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].xsizeout);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[1] != dmdispcombconf[DMindex].ysizeout)
        {
            sprintf(errstr, "image \"%s\" should have y axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].ysizeout);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }

        IDtmpoutdm = create_2Dimage_ID("_tmpoutdm", dmdispcombconf[DMindex].xsizeout, dmdispcombconf[DMindex].ysizeout);
        sizexyDMout = dmdispcombconf[DMindex].xsizeout*dmdispcombconf[DMindex].ysizeout;
        printf("done\n\n");
        fflush(stdout);
    }



    list_image_ID(); //TEST

    dmdispcombconf[DMindex].wfsrefmode = wfsrefmode;
    if(wfsrefmode == 1)
    {
        printf("INITIALIZATION AND VERIFICATION FOR wfsref MODE ...\n");
        fflush(stdout);

        printf("wfsref_WFSRespMat = %s\n", wfsref_WFSRespMat);
        fflush(stdout);

        dmdispcombconf[DMindex].ID_wfsref_RespMat = image_ID(wfsref_WFSRespMat);
        if(dmdispcombconf[DMindex].ID_wfsref_RespMat==-1)
        {
            printf("ERROR: cannot find image \"%s\"\n", wfsref_WFSRespMat);
            exit(0);
        }

        if(data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].naxis != 3)
        {
            sprintf(errstr, "image \"%s\" should have naxis = 3", wfsref_WFSRespMat);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        dmdispcombconf[DMindex].xsizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[0];
        dmdispcombconf[DMindex].ysizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[1];

        printf("xsizewfsref = %ld\n", dmdispcombconf[DMindex].xsizewfsref);
        printf("ysizewfsref = %ld\n", dmdispcombconf[DMindex].ysizewfsref);
        fflush(stdout);

        dmdispcombconf[DMindex].ID_wfsref_out = image_ID(wfsref_out);
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[0] != dmdispcombconf[DMindex].xsizewfsref)
        {
            sprintf(errstr, "image \"%s\" should have x axis = %ld", wfsref_out, dmdispcombconf[DMindex].xsizewfsref);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[1] != dmdispcombconf[DMindex].ysizewfsref)
        {
            sprintf(errstr, "image \"%s\" should have y axis = %ld", wfsref_out, dmdispcombconf[DMindex].ysizewfsref);
            printERROR(__FILE__,__func__,__LINE__, errstr);
            exit(0);
        }
        printf("Creating image %s   %ld x %ld\n", "_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        fflush(stdout);
        IDtmpoutref = create_2Dimage_ID("_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        sizexywfsref = dmdispcombconf[DMindex].xsizewfsref*dmdispcombconf[DMindex].ysizewfsref;

        COREMOD_MEMORY_image_set_createsem(wfsref_out, 10);

        printf("done\n\n");
        fflush(stdout);
    }

    printf("Initialize channels\n");
    printf("Max DM stroke = %f um\n", dmdispcombconf[DMindex].stroke100*dmdispcombconf[DMindex].MAXVOLT/100.0*dmdispcombconf[DMindex].MAXVOLT/100.0);
    fflush(stdout);

    for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
    {
        sprintf(name, "dm%02lddisp%02ld", DMindex, ch);
        printf("Channel %ld \n", ch);
        dmdispcombconf[DMindex].dmdispID[ch] = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
        COREMOD_MEMORY_image_set_createsem(name, 10);
        dmdispptr_array[ch] = data.image[dmdispcombconf[DMindex].dmdispID[ch]].array.F;
    }


    sprintf(name, "dm%02lddisp", DMindex);
    dmdispcombconf[DMindex].IDdisp = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    sprintf(name, "dm%02lddispt", DMindex);
    IDdispt = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 0, 0);
    dmdispptr = data.image[IDdispt].array.F;

    if(dmdispcombconf[DMindex].voltmode==1)
    {
        IDvolt = image_ID(dmdispcombconf[DMindex].voltname);

        vOK = 0;
        if(IDvolt!=-1)
        {
            if((data.image[IDvolt].md[0].naxis==2)&&(data.image[IDvolt].md[0].size[0]==xsize)&&(data.image[IDvolt].md[0].size[1]==ysize))
            {
                if((dmdispcombconf[DMindex].volttype==1)&&(data.image[IDvolt].md[0].atype==_DATATYPE_FLOAT))
                    vOK = 1;
                if((dmdispcombconf[DMindex].volttype==2)&&(data.image[IDvolt].md[0].atype==_DATATYPE_UINT16))
                    vOK = 1;
                if(vOK==0)
                    delete_image_ID(dmdispcombconf[DMindex].voltname);
            }
        }

        printf("vOK = %d\n", vOK);
        if(vOK==0)
        {
            printf("CREATING stream %s  %d axis, size = %u x %u\n", dmdispcombconf[DMindex].voltname, naxis, size[0], size[1]);

            if(dmdispcombconf[DMindex].volttype==1)
                dmdispcombconf[DMindex].IDvolt = create_image_ID(dmdispcombconf[DMindex].voltname, naxis, size, _DATATYPE_FLOAT, 1, 10);

            if(dmdispcombconf[DMindex].volttype==2)
                dmdispcombconf[DMindex].IDvolt = create_image_ID(dmdispcombconf[DMindex].voltname, naxis, size, _DATATYPE_UINT16, 1, 10);
            COREMOD_MEMORY_image_set_createsem(dmdispcombconf[DMindex].voltname, 10);
        }
        else
            dmdispcombconf[DMindex].IDvolt = image_ID(dmdispcombconf[DMindex].voltname);
    }

    cntsumold = 0;


    dmdispcombconf[0].status = 1;

    sprintf(name, "dm%02lddisp", DMindex);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem<2)
    {
        printf("ERROR: image %s semaphore %d missing\n", data.image[dmdispcombconf[DMindex].IDdisp].name, 1);
        exit(0);
    }

    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    if(dmdispcombconf[DMindex].MAXVOLT>maxmaxvolt)
        dmdispcombconf[DMindex].MAXVOLT = maxvolt;


    AOloopControl_printDMconf();





    int loopCTRLexit = 0; // toggles to 1 when loop is set to exit cleanly
    if(data.processinfo==1)
        processinfo->loopstat = 1;


    while(dmdispcombconf[DMindex].ON == 1)
    {
        struct timespec semwaitts;

        if(data.processinfo==1)
        {
            while(processinfo->CTRLval == 1)  // pause
                usleep(50);

            if(processinfo->CTRLval == 2) // single iteration
                processinfo->CTRLval = 1;

            if(processinfo->CTRLval == 3) // exit loop
                loopCTRLexit = 1;
        }



        dmdispcombconf[DMindex].status = 2;

        if(DMtwaitus>0)
            usleep(DMtwaitus);

        if (clock_gettime(CLOCK_REALTIME, &semwaitts) == -1) {
            perror("clock_gettime");
            exit(EXIT_FAILURE);
        }

        semwaitts.tv_nsec += dmdispcombconf[DMindex].nsecwait;
        if(semwaitts.tv_nsec >= 1000000000)
            semwaitts.tv_sec = semwaitts.tv_sec + 1;

        DMupdate = 0;

        if(dmdispcombconf[DMindex].TrigMode==0)
        {
            //
            // this is semaphore that triggers the write to the DM
            //
            sem_timedwait(data.image[dmdispcombconf[DMindex].IDdisp].semptr[1], &semwaitts);

            cntsum = 0;
            for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            {
                cntch = data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
                dmdispcombconf[DMindex].dmdispcnt[ch] = cntch;
                cntsum += data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
            }
            if(cntsum != cntsumold)
                DMupdate = 1;
        }
        else
        {
            sem_timedwait(data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].semptr[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigSem]], &semwaitts);
            cnt = data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].md[0].cnt0;
            if(cnt!=cntold)
            {
                DMupdate = 1;
                cntold=cnt;
            }
        }

        if(DMupdate==1)
        {
            clock_gettime(CLOCK_REALTIME, &ttrig);

            dmdispcombconf[0].status = 3;
            cnt++;

            memcpy (data.image[IDdispt].array.F, dmdispptr_array[0], sizeof(float)*sizexy);
            for(ch=1; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            {
                for(ii=0; ii<sizexy; ii++)
                    dmdispptr[ii] += dmdispcombconf[DMindex].dmdispgain[ch]*dmdispptr_array[ch][ii];
            }

            dmdispcombconf[DMindex].status = 4;


            ave = 0.0;
            if(dmdispcombconf[DMindex].AveMode == 1) // REMOVE AVERAGE
            {
                for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                    ave += data.image[IDdispt].array.F[ii];
                ave /= dmdispcombconf[DMindex].xysize;
            }
            dmdispcombconf[DMindex].status = 5;

            if(dmdispcombconf[DMindex].AveMode < 2) // OFFSET BY DClevel
            {
                for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                {
                    data.image[IDdispt].array.F[ii] += (dmdispcombconf[DMindex].DClevel - ave);

                    // remove negative values
                    if(dmdispcombconf[DMindex].voltmode==1)
                        if(data.image[IDdispt].array.F[ii]<0.0)
                            data.image[IDdispt].array.F[ii] = 0.0;
                }
            }
            dmdispcombconf[DMindex].status = 6;

            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 1;
            memcpy (data.image[dmdispcombconf[DMindex].IDdisp].array.F,data.image[IDdispt].array.F, sizeof(float)*data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].cnt0++;
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 0;

            /*     for(semnb=0;semnb<data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem;semnb++)
                    {
                        sem_getvalue(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb], &semval);
                        if(semval<SEMAPHORE_MAXVAL)
                        sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb]);
                     }*/
            COREMOD_MEMORY_image_set_sempost_byID(dmdispcombconf[DMindex].IDdisp, -1);
            //      sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[0]);


            if(dm2dm_mode==1)
            {
                memset(data.image[IDtmpoutdm].array.F, '\0', sizeof(float)*sizexyDMout);
                for(kk=0; kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement; kk++)
                {
                    for(ii=0; ii<sizexyDMout; ii++)
                        data.image[IDtmpoutdm].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].array.F[kk*sizexyDMout+ii];
                }

                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].array.F,data.image[IDtmpoutdm].array.F, sizeof(float)*sizexyDMout);
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 0;
                sem_post(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].semptr[0]);
            }


            if(wfsrefmode==1)
            {
                memset(data.image[IDtmpoutref].array.F, '\0', sizeof(float)*sizexywfsref);
                list_image_ID();
                printf("kkmax = %ld\n", data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
                printf("iimax = %ld\n", sizexywfsref);
                printf("ID RespMat = %ld  (%ld)\n", dmdispcombconf[DMindex].ID_wfsref_RespMat, (data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement-1)*sizexywfsref + sizexywfsref-1);
                fflush(stdout);
                save_fits(wfsref_WFSRespMat, "!_test_wfsref_WFSRespMat.fits");
                for(kk=0; kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement; kk++)
                {
                    printf("(%ld %g) ", kk, data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk]);
                    for(ii=0; ii<sizexywfsref; ii++)
                        data.image[IDtmpoutref].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].array.F[kk*sizexywfsref+ii];
                }
                printf("\n");
                printf("Updating Zero Point  %ld <- %ld\n", dmdispcombconf[DMindex].ID_wfsref_out, IDtmpoutref);
                fflush(stdout);
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_wfsref_out].array.F,data.image[IDtmpoutref].array.F, sizeof(float)*sizexywfsref);
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 0;
                sem_post(data.image[dmdispcombconf[DMindex].ID_wfsref_out].semptr[0]);
                printf("Done\n");
                fflush(stdout);
            }



            dmdispcombconf[DMindex].status = 7;


            clock_gettime(CLOCK_REALTIME, &t1);
            if(dmdispcombconf[DMindex].voltmode==1)
                AOloopControl_DM_disp2V(DMindex);


            dmdispcombconf[DMindex].status = 8;

            cntsumold = cntsum;
            dmdispcombconf[DMindex].updatecnt++;
            

            if(data.processinfo==1)
                processinfo->loopcnt = dmdispcombconf[DMindex].updatecnt;

            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = time_diff(ttrig, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            dmdispcombconf[DMindex].tdelay = tdiffv;


            tdiff = time_diff(t1, tnow);
            tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            dmdispcombconf[DMindex].time_disp2V = tdiffv;

        }

        if((data.signal_INT == 1)||(data.signal_TERM == 1)||(data.signal_ABRT==1)||(data.signal_BUS==1)||(data.signal_SEGV==1)||(data.signal_HUP==1)||(data.signal_PIPE==1))
        {
            if(data.processinfo==1)
            {
                struct timespec tstop;
                struct tm *tstoptm;
                char msgstring[200];
                char timestring[200];


                clock_gettime(CLOCK_REALTIME, &tstop);
                tstoptm = gmtime(&tstop.tv_sec);
                sprintf(timestring, "%02d:%02d:%02d.%03d", tstoptm->tm_hour, tstoptm->tm_min, tstoptm->tm_sec, (int) (0.000001*tstop.tv_nsec));

                sprintf(msgstring, "Unknown signal at %s", timestring); // default

                if(data.signal_INT == 1)
                    sprintf(msgstring, "SIGINT at %s", timestring);

                if(data.signal_TERM == 1)
                    sprintf(msgstring, "SIGTERM at %s", timestring);

                if(data.signal_ABRT == 1)
                    sprintf(msgstring, "SIGABRT at %s", timestring);

                if(data.signal_BUS == 1)
                    sprintf(msgstring, "SIGBUS at %s", timestring);

                if(data.signal_SEGV == 1)
                    sprintf(msgstring, "SIGSEGV at %s", timestring);

                if(data.signal_HUP == 1)
                    sprintf(msgstring, "SIGHUP at %s", timestring);

                if(data.signal_PIPE == 1)
                    sprintf(msgstring, "SIGPIPE at %s", timestring);

                //strncpy(processinfo->statusmsg, msgstring, 200);
                
                processinfo_WriteMessage(processinfo, msgstring);
                processinfo->loopstat = 3; // clean exit
            }

            dmdispcombconf[DMindex].ON = 0;
           // exit(0);
        }

        if(loopCTRLexit == 1)
        {
            dmdispcombconf[DMindex].ON = 0;
            if(data.processinfo==1)
				processinfo->loopstat = 3;
        }
    }

    //  if(voltmode==1)
    //    arith_image_zero(dmdispcombconf[DMindex].voltname);


    if(data.processinfo==1)
        processinfo_cleanExit(processinfo);
    
    
    

    printf("LOOP STOPPED\n");
    fflush(stdout);

    free(size);


    return 0;
}





int AOloopControl_DM_dmdispcomboff(long DMindex)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].ON = 0;
	AOloopControl_printDMconf();
	
    return 0;
}


int AOloopControl_DM_dmtrigoff(long DMindex)
{
	AOloopControl_DM_loadconf();
	data.image[dmdispcombconf[DMindex].IDvolt].md[0].status = 101;
	AOloopControl_printDMconf();
    
    return 0;
}

