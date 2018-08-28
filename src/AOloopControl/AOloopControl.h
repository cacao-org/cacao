/**
 * @file    AOloopControl.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine
 * 
 * 
 * AO engine uses stream data structure
 * 
 * @bug No known bugs. 
 * 
 */




#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "AOloopControl/AOloopControl_AOcompute.h" 
#include "AOloopControl/AOloopControl_aorun.h"              // AOLOOPCONF_WFSim
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"  

#ifndef _AOLOOPCONTROL_H
#define _AOLOOPCONTROL_H





// OVERVIEW


// AOloopControl_var
// structure unique to specific process
// used for convenience to store process-specific values, for example image IDs

// AOLOOPCONTROL_CONF
// shared memory structure for sharing between processes and provide outside control and visibility




// PROCESSES
/*
 * AO loop can launch a number of processes
 * 
 * 
 * 
 * All AO loop instances MUST start a AOloopControl_aorun() function call
 * FUNCTION: AOloopControl_aorun()
 * in file : AOloopControl_aorun.c
 * AOloopControl_aorun() -> AOcompute() -> Read_cam_frame()
 * 
 * 
 * 
 * 
 * FUNCTION: AOloopControl_ComputeOpenLoopModes()
 * in file : AOloopControl_aorun.c
 * 
 * 
 * 
 */













#define maxNBMB 100			// maximum number of mode blocks
#define MAXNBMODES 10000	// maximum number of control modes
#define MAX_NUMBER_TIMER 100







// REAL TIME DATA LOGGING

#define MAX_NUMBER_RTLOGSTREAM 20

#define RTSLOGindex_wfsim                  0
#define RTSLOGindex_imWFS0                 1
#define RTSLOGindex_imWFS1                 2
#define RTSLOGindex_imWFS2                 3
#define RTSLOGindex_modeval                4
#define RTSLOGindex_modeval_dm             5
#define RTSLOGindex_modeval_dm_corr        6
#define RTSLOGindex_modeval_dm_now         7
#define RTSLOGindex_modeval_dm_now_filt    8
#define RTSLOGindex_modevalPF              9
#define RTSLOGindex_modevalPFsync         10
#define RTSLOGindex_modevalPFres          11
#define RTSLOGindex_modeval_ol            12
#define RTSLOGindex_dmC                   13
#define RTSLOGindex_dmdisp                14



// Real-time streams use this struc to hold relevant info
typedef struct
{
	int active;                   // 1 if used 
	char name[100];               // stream name (excludes aol#_)
	int ENABLE;                   // Is logging enabled ? This needs to be specified at startup, if set to zero, no RT logging will be performed
	int INIT;                     // 1 if memory is initiated
	int ON;                       // Is logging ON ?
	int SIZE;                     // Max number of samples per buffer
	int buffindex;                // which buffer (0 or 1)
	long frameindex;              // frame index
	long frameindexend0;          // last frame in buffer 0
	long frameindexend1;          // last frame in buffer 1
	int save;                     // 0: do not save, 1: save data+timing, 2: save timing only
	int saveToggle;               // 1 if buffer #0 ready to be saved, 2 if buffer #1 ready to be saved, 0 otherwise
	
	long IDbuff;
	long IDbuff0;                 // local identifier
	long IDbuff1;                 // local identifier

	long IDbuffinfo;
	long IDbuffinfo0;             // local identifier
	long IDbuffinfo1;             // local identifier
	
	float *srcptr;                // source stream pointer
	long IDsrc;                   // source ID

	char *destptr;                // destination pointer
	char *destptr0;               // destination pointer 0
	char *destptr1;               // destination pointer 1

	size_t memsize;               // size of stream
	
	int NBcubeSaved;              // Number of cubes to save, default = -1
	
} RTstreamLOG;









// LOGGING
//#define RT_LOGsize 30000



// logging
static int loadcreateshm_log = 0; // 1 if results should be logged in ASCII file
static FILE *loadcreateshm_fplog;










/**
 * Main AOloopControl structure. 
 *
 * Holds key parameters for the AO engine as well as higher level variables describing the state of the control loop
 * 
 * @ingroup AOloopControl_AOLOOPCONTROL_CONF
 */
typedef struct
{	
	// These structures are always part of AO loop control (not optional)

	AOLOOPCONF_aorun       aorun;         // structure defined in AOloopControl_aorun.h
	AOLOOPCONF_AOcompute   AOcompute;     // structure defined in AOloopControl_AOcompute.h	
	AOLOOPCONF_WFSim       WFSim;         // structure defined in AOloopControl_AOcompute.h

	AOloopTimingInfo       AOtiminginfo;  // structure defined in AOloopControl_aorun.h
	
	
	
	
	



    /* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: SETUP & INITIALIZATION STATE 
	 * 
	 */

    int_fast8_t   init;                         /**< Has the structure been initialized ? */
    uint_fast64_t cnt;                        /**<  loop step counter, set to zero every time loop is stopped */
    uint_fast64_t cntmax;                     /**<  max value of counter, used to step loop */
    uint_fast64_t DMupdatecnt;                /**<  */
//    int_fast8_t   aorun__kill;                         /**<  set to 1 to kill computation loop */
    char name[80];

    int_fast8_t init_RM;                      /**< Response Matrix loaded */
    int_fast8_t init_CM;                      /**< Control Matrix loaded */
    int_fast8_t init_CMc;                     /**< Combined control matrix computed */
    int_fast8_t initmapping;
    char respMname[80];
    char contrMname[80];
  
    /* =============================================================================================== */



    /* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: WFS CAMERA
	 * 
	 */
	
    /** @brief WFS stream name
     * 
     * Usually set to aol_wfsim by function AOloopControl_loadconfigure
     */

    /* =============================================================================================== */



    /* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: DEFORMABLE MIRROR
	 * 
	 */
	 
    char dmCname[80];
    char dmdispname[80];
    char dmRMname[80];
    uint_fast8_t DMMODE;                      /**< 0: zonal DM, 1: modal DM */
    uint_fast32_t sizexDM;                    /**< DM x size*/
    uint_fast32_t sizeyDM;                    /**< DM y size*/
    uint_fast32_t sizeDM;                     /**< DM total image (= x size * y size) */
    uint_fast32_t activeDMcnt;                /**< number of active actuators */
    uint_fast32_t sizeDM_active;              /**< only takes into account DM actuators that are active/in use */
    
    /* =============================================================================================== */



	/* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: CONTROL MODES 
	 * 
	 */

    char DMmodesname[80];
     // BLOCKS OF MODES
    uint_fast16_t DMmodesNBblock;             /**< number of mode blocks (read from parameter) */
    uint_fast16_t NBmodes_block[100];         /**< number of modes within each block (computed from files by AOloopControl_loadconfigure) */
    uint_fast16_t modeBlockIndex[MAXNBMODES]; /**< block index to which each mode belongs (computed by AOloopControl_loadconfigure) */
    uint_fast16_t indexmaxMB[maxNBMB]; 

	uint_fast16_t NBDMmodes;

    int_fast8_t init_wfsref0;    // WFS reference image loaded

    float maxlimit; // maximum absolute value for mode values
    float mult; // multiplication coefficient to be applied at each loop iteration
 
	/* =============================================================================================== */




	/* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: LOOP CONTROL
	 * 
	 */
		
    float gain;                               /**< overall loop gain */
    uint_fast16_t framesAve;                  /**< number of WFS frames to average */
 
	// MODAL AUTOTUNING 
	// limits
	int_fast8_t AUTOTUNE_LIMITS_ON;
	float AUTOTUNE_LIMITS_perc; // percentile limit for autotuning
	float AUTOTUNE_LIMITS_mcoeff; // multiplicative coeff 
	float AUTOTUNE_LIMITS_delta; // autotune loop increment 

	int_fast8_t AUTOTUNE_GAINS_ON;
	float AUTOTUNEGAINS_updateGainCoeff;      /**< Averaging coefficient (usually about 0.1) */
	float AUTOTUNEGAINS_evolTimescale;        /**< Evolution timescale, beyond which errors stop growing */
	long AUTOTUNEGAINS_NBsamples;            /**< Number of samples */
   
	/* =============================================================================================== */

 
 
	/* =============================================================================================== */
	/** @name AOLOOPCONTROL_CONF: PREDICTIVE CONTROL
	 * 
	 */
	///@{  	
	float ARPFgain; 
	float ARPFgainAutoMin;
	float ARPFgainAutoMax;
    ///@}
	/* =============================================================================================== */



	/* =============================================================================================== */

    

    


	/* =============================================================================================== */
 	/** @name AOLOOPCONTROL_CONF: BASIC LOOP TELEMETRY AND PERFORMANCE
	 * 
	 */

    // COMPUTED BY OPEN LOOP RETRIEVAL PROCESS
    double RMSmodes;
    double RMSmodesCumul;
    uint_fast64_t RMSmodesCumulcnt;

	// WFS 
	

	// block statistics (instantaneous)
	double block_PFresrms[100]; // Prediction residual, meas RMS
	double block_OLrms[100]; // open loop RMS
	double block_Crms[100]; // correction RMS
	double block_WFSrms[100]; // WFS residual RMS
	double block_WFSnoise[100]; // WFS measurement noise
	double block_limFrac[100]; // fraction of mode coefficients exceeding limit
	
	double ALL_OLrms; // open loop RMS
	double ALL_Crms; // correction RMS
	double ALL_WFSrms; // WFS residual RMS
	double ALL_WFSnoise; // WFS noise
	double ALL_limFrac; // fraction of mode coefficients exceeding limit
	
	// averaged
	uint_fast32_t AveStats_NBpt; // averaging interval
	double blockave_PFresrms[100]; // open loop RMS
	double blockave_OLrms[100]; // open loop RMS
	double blockave_Crms[100]; // correction RMS
	double blockave_WFSrms[100]; // WFS residual RMS
	double blockave_WFSnoise[100]; // WFS noise
	double blockave_limFrac[100]; // fraction of mode coefficients exceeding limit

	double ALLave_OLrms; // open loop RMS
	double ALLave_Crms; // correction RMS
	double ALLave_WFSrms; // WFS residual RMS
	double ALLave_WFSnoise; // WFS noise
	double ALLave_limFrac; // fraction of mode coefficients exceeding limit

	/* =============================================================================================== */
    
    
    /* =============================================================================================== */
 	/** @name AOLOOPCONTROL_CONF: REAL TIME LOGGING
	 * 
	 */
//LOG

	//
	// Real-time streams are updated once per loop iteration
	// Realtime logging buffers are synchronized
	// Two buffers are created for each stream to be logged
	//
	// Saving to disk is handled outside of cacao real-time
	//
	long  RTLOGsize;                       // Number of samples per shared memory stream
	//int   RTLOG_ON;                        // set to 1 to start logging all RT streams -> ensures synchronization
	//int   RTstreamLOG_buff;                // Which buffer is currently being written (0 or 1)
	//long  RTstreamLOG_frame;               // Which frame is to be written in buffer
	//int   RTstreamLOG_buffSwitch;          // Goes to 1 when buffer switches. Goes back to zero on next iteration.
	
	// For each stream, there are two data buffers and two timing buffers
	// Buffer names:
	//     aol%ld_<stream>_logbuff0
	//     aol%ld_<stream>_logbuff1
	//	   aol%ld_<stream>_logbuffinfo0   
	//     aol%ld_<stream>_logbuffinfo1
	//
	// Timing buffer contains loop iteration, frame counter cnt0 and time stamps when entry is written
	// 
	
	// ENABLES, ON and save are read/set up in AOloopControl_loadconfigure()
	
	
	// Realtime logging
	RTstreamLOG RTSLOGarray[MAX_NUMBER_RTLOGSTREAM];


} AOLOOPCONTROL_CONF;















typedef struct
{
    /* =============================================================================================== */
    /*                    aoconfID are global variables for convenience                                */
    /*  These variables are LOCAL to each process, and not shared between processes                    */
    /*  aoconfID can be used in other modules as well (with extern)                                    */
    /* =============================================================================================== */

    long LOOPNUMBER;// = 0; // current loop index

    // TIMING
    //struct timespec tnow; //static
    //struct timespec tdiff; //static
    //double tdiffv; //static

    //int initWFSref_GPU[100];// = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int initcontrMcact_GPU[100];// = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    //used in AOloopControl_loop_param.c
    // static

    float GPU_alpha;
    float GPU_beta;


    int AOloopcontrol_meminit;
	// local copies to avoid reset to zero by other process 
	int_fast8_t init_RM_local;                      /**< Response Matrix loaded */
    int_fast8_t init_CM_local;                      /**< Control Matrix loaded */
    int_fast8_t init_CMc_local;                     /**< Combined control matrix computed */


    int COMPUTE_PIXELSTREAMING; // static. multiple pixel groups
    int PIXSTREAM_NBSLICES;// number of image slices (= pixel groups)
    int PIXSTREAM_SLICE; // slice index 0 = all pixels


	// Logging REAL TIME data (internal logging)
	// (note: non real time streams are logged externally)
	//
	// To optimize logging speed, each stream can be logged in a 3D data cube within the function where it is created.
	// 3rd axis is time
	// A timing array is also created.
	//
	// List of REAL TIME streams logged with the internal process
	//
	//
	// wfsim              logged in function  ??
	// modeval_ol         logged in function  AOloopControl_ComputeOpenLoopModes
	//
	
	

	//
	// Shared between REAL TIME STREAMS:
	//
	// streamLOG_buff                   which buffer to use (0 or 1)   -> in AOLOOPCONTROL_CONF (needs to be shared and visible)
	// streamLOG_frame                  frame index within buffer      -> in AOLOOPCONTROL_CONF (needs to be shared and visible)
	// streamLOG_saveToggle             1 to force a save now          -> in AOLOOPCONTROL_CONF (needs to be shared and visible)
	//
	//
	// For each REAL TIME stream <sname> :
	//
	// aoconfLOG_<sname>_databuffID0	    logging data buffer 0 ID
	// aoconfLOG_<sname>_databuffID1	    logging data buffer 1 ID
	// aoconfLOG_<sname>_infobuff0          logging info buffer 0
	// aoconfLOG_<sname>_infobuff1          logging info buffer 1
	// aoconfLOG_<sname>_databuffID         active data buffer ID
	// aoconfLOG_<sname>_infobuff           active info buffer pointer
	//
	// streamLOG_<sname>_ON  - NEEDS TO BE SET UP AT STARTUP           -> in AOLOOPCONTROL_CONF (needs to be shared and visible)
	//		0 : No logging, no writing into 3D cube
	//		1 : Create 3D cube and write into it
	// streamLOG_<sname>_save           1 if saved to disk             -> in AOLOOPCONTROL_CONF (needs to be shared and visible)





    // Hardware connections
    
    // WFS image
    long aoconfID_wfsim;                 // identifier to stream
    uint8_t WFSatype;    

/*LOG         

    long aoconfLOG_wfsim_buffID0;        // buffer 0
    long aoconfLOG_wfsim_buffID1;        // buffer 1    
    long aoconfLOG_wfsim_timerbuffID0;   // timer buffer 0
    long aoconfLOG_wfsim_timerbuffID1;   // timer buffer 1
    long aoconfLOG_wfsim_buffID;         // active buffer ID
    long aoconfLOG_wfsim_timerbuffID;    // active timer buffer ID
  */  

    long aoconfID_dmC;
    long aoconfID_dmRM;

    
    
    
    
    long aoconfID_wfsdark;
    
    long aoconfID_imWFS0;        // identifier to stream
    
    long aoconfID_imWFS0tot;
    
    long aoconfID_imWFS1;
    
    long aoconfID_imWFS2;


    long aoconfID_imWFSlinlimit; // WFS linearity limit

    long aoconfID_wfsref0;
    long aoconfID_wfsref;
    long long aoconfcnt0_wfsref_current;

    long aoconfID_DMmodes;
    long aoconfID_dmdisp; // to notify DMcomb that DM maps should be summed



    // Control Modes
    long aoconfID_cmd_modes;
    long aoconfID_meas_modes; // measured
    long aoconfID_RMS_modes;
    long aoconfID_AVE_modes;
    long aoconfID_modeARPFgainAuto;
    long aoconfID_modevalPF;

    // mode gains, multf, limit are set in 3 tiers
    // global gain
    // block gain
    // individual gains

    // blocks
    long aoconfID_gainb; // block modal gains
    long aoconfID_multfb; // block modal gains
    long aoconfID_limitb; // block modal gains

    // individual modesaol0PFb1comp:
    long aoconfID_DMmode_GAIN;
    long aoconfID_LIMIT_modes;
    long aoconfID_MULTF_modes;

    long aoconfID_cmd_modesRM;

    long aoconfID_wfsmask;
    long aoconfID_dmmask;

    //long aoconfID_respM;//  = -1; static
    //only in loadconfigure.c

    long aoconfID_contrM;//   pixels -> modes

    //long long aoconfcnt0_contrM_current;//  = -1; static

    long aoconfID_contrMc;// combined control matrix: pixels -> DM actuators
    long aoconfID_meas_act;
    long aoconfID_contrMcact[100];

    //static
    // pixel streaming
    long aoconfID_pixstream_wfspixindex; // index of WFS pixels

    // timing
    long aoconfID_looptiming;// control loop timing data. Pixel values correspond to time offset
    // currently has 20 timing slots
    // beginning of iteration is defined when entering "wait for image"
    // md[0].atime.ts is absolute time at beginning of iteration
    //
    // pixel 0 is dt since last iteration
    //
    // pixel 1 is time from beginning of loop to status 01
    // pixel 2 is time from beginning of loop to status 02
    // ...
    long AOcontrolNBtimers;

    long aoconfIDlogdata;
    //long aoconfIDlog0;//  = -1; static
    //long aoconfIDlog1;//  = -1; static
    // those two last don't exist in any functions


    int *WFS_active_map; // used to map WFS pixels into active array
    int *DM_active_map; // used to map DM actuators into active array
    long aoconfID_meas_act_active;
    //long aoconfID_imWFS2_active[100]; wfs_dm.c & computeCalib.c

    float normfloorcoeff;//  = 1.0; static

    //long wfsrefcnt0;//  = -1; static
    //long contrMcactcnt0[100];// static = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};;
    //both in wfs_dm.c

    // variables used by functions

    //int GPUcntMax;//  = 100; static initmem.c
    int *GPUset0; //static
    int *GPUset1; //static
    
    
    
    // Realtime logging, local init flag
    // will be equal to 1 only if the current process "owns" the realtime logging
	int RTSLOGarrayInitFlag[MAX_NUMBER_RTLOGSTREAM];
    

} AOloopControl_var;






void __attribute__ ((constructor)) libinit_AOloopControl();

/** @brief Initialize AOloopControl command line interface. */
void init_AOloopControl();

int AOloopControl_bogusfunc();














/* =============================================================================================== */
/** @brief Read parameter value - float, char or int - AOloopControl_read_param.c                */
/* =============================================================================================== */
/** @brief read parameters float */
float AOloopControl_readParam_float(char *paramname, float defaultValue, FILE *fplog);

/** @brief read parameters int */
int AOloopControl_readParam_int(char *paramname, int defaultValue, FILE *fplog);

/** @brief read parameters string */
char* AOloopControl_readParam_string(char *paramname, char* defaultValue, FILE *fplog);


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. INITIALIZATION, configurations                                        
 * 
 * Allocate memory, import/export configurations
 *  AOloopControl_loadconfigure.c - AOloopControl_initmem.c
 */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief Load configuation parameters from disk - AOloopControl_loadconfigure.c*/
int_fast8_t AOloopControl_loadconfigure(long loop, int mode, int level);


/** @brief Initialize memory - function called within C code only (no CLI call) */
int_fast8_t AOloopControl_InitializeMemory();



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 2. REAL TIME COMPUTING ROUTINES - AOloopControl.c, AOloopControl_wfs.c
 *  calls CPU and GPU processing */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief WFS zero point update */
int_fast8_t AOloopControl_WFSzpupdate_loop(const char *IDzpdm_name, const char *IDzrespM_name, const char *IDwfszp_name);

/** @brief WFS sum zero point update */
int_fast8_t AOloopControl_WFSzeropoint_sum_update_loop(long loopnb, const char *ID_WFSzp_name, int NBzp, const char *IDwfsref0_name, const char *IDwfsref_name);

/** @brief Main loop function */
int_fast8_t AOloopControl_aorun();

/** @brief CPU based matrix-vector multiplication - when no GPU */
int_fast8_t ControlMatrixMultiply( float *cm_array, float *imarray, long m, long n, float *outvect);

/** @brief Sends modal commands to DM by matrix-vector multiplication */
int_fast8_t set_DM_modes(long loop);

/** @brief Response Matrix DM-WFS */
int_fast8_t set_DM_modesRM(long loop);

/** @brief Main computation function, runs once per loop iteration */
int_fast8_t AOcompute(long loop, int normalize);

/** @brief Main computation function, runs once per loop iteration */
int_fast8_t AOloopControl_CompModes_loop(const char *ID_CM_name, const char *ID_WFSref_name, const char *ID_WFSim_name, const char *ID_WFSimtot_name, const char *ID_coeff_name);

/** @brief Matrix multiplication on GPU to transfom modes coefficients into DM shape */
int_fast8_t AOloopControl_GPUmodecoeffs2dm_filt_loop(const int GPUMATMULTCONFindex, const char *modecoeffs_name, const char *DMmodes_name, int semTrigg, const char *out_name, int GPUindex, long loop, int offloadMode);

/** @brief CPU matrix multiplication to transfom WFS signal into modes coefficients */
long AOloopControl_sig2Modecoeff(const char *WFSim_name, const char *IDwfsref_name, const char *WFSmodes_name, const char *outname);

/** @brief Compute WFS residual image */
long AOloopControl_computeWFSresidualimage(long loop, char *IDalpha_name);

/** @brief Compute modes in open loop */
long AOloopControl_ComputeOpenLoopModes(long loop);

/** @brief Auto tune gains of the closed loop */
int_fast8_t AOloopControl_AutoTuneGains(long loop, const char *IDout_name, float GainCoeff, long NBsamples);

/** @brief Mixes streamin into streamout, in order to make streamout converge to streamin  */
long AOloopControl_dm2dm_offload(const char *streamin, const char *streamout, float twait, float offcoeff, float multcoeff);



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 3.   LOOP CONTROL INTERFACE - AOloopControl_loop_ctr.c
 *  Set parameters */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief Set loop number. Ex : for the Pyramid WFS, loop number = 0  */
int_fast8_t AOloopControl_setLoopNumber(long loop);

/** @brief Set one function for many parameters  */
int_fast8_t AOloopControl_setparam(long loop, const char *key, double value);


/* =============================================================================================== */
/** @name AOloopControl - 3.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET
 *  Set parameters - AOloopControl_loop_onoff.c*/
/* =============================================================================================== */

/** @brief Close AO loop : AO on */
int_fast8_t AOloopControl_loopon();

/** @brief  Open AO loop : AO off  */
int_fast8_t AOloopControl_loopoff();

/** @brief Close AO loop : AO on */
int_fast8_t AOloopControl_loopWFScompon();

/** @brief  Open AO loop : AO off  */
int_fast8_t AOloopControl_loopWFScompoff();

/** @brief Kill AO loop : finish the process of the run */
int_fast8_t AOloopControl_loopkill();

/** @brief Close loop for finite number of steps */
int_fast8_t AOloopControl_loopstep(long loop, long NBstep);

/** @brief Reset the AO loop */
int_fast8_t AOloopControl_loopreset();


/* =============================================================================================== */
/** @name AOloopControl - 3.2. LOOP CONTROL INTERFACE - DATA LOGGING                               */
/* =============================================================================================== */

// NB : doesn't exist anywhere ?? 
/** @brief Log on the AO interface */
int_fast8_t AOloopControl_logon();

/** @brief Log off AO interface */
int_fast8_t AOloopControl_logoff();


/* ============================================================================================================ */
/** @name AOloopControl - 3.3. LOOP CONTROL INTERFACE - PRIMARY AND FILTERED DM WRITE - AOloopControl_dmwrite.c */
/* ============================================================================================================ */

/** @brief Writing on DM, unfiltered actuators (primary) : on */
int_fast8_t AOloopControl_DMprimaryWrite_on();

/** @brief Writing on DM, unfiltered actuators (primary) : off */
int_fast8_t AOloopControl_DMprimaryWrite_off();

/** @brief Writing on DM, after filtering : on */
int_fast8_t AOloopControl_DMfilteredWrite_on();

/** @brief Writing on DM, after filtering : off */
int_fast8_t AOloopControl_DMfilteredWrite_off();


/* ====================================================================================================== */
/** @name AOloopControl - 3.4. LOOP CONTROL INTERFACE - INTEGRATOR AUTO TUNING - AOloopControl_autotune.c */
/* ====================================================================================================== */

/** @brief Set limit auto tune : on */
int_fast8_t AOloopControl_AUTOTUNE_LIMITS_on();

/** @brief Set limit auto tune : off */
int_fast8_t AOloopControl_AUTOTUNE_LIMITS_off();

/** @brief Options auto tune limit 
* The limit is fixed at the beginning. 
* When the fraction of mode values higher than the current limit times mcoeff is larger than perc (percentile);
* then the limit increases by delta. Otherwise, it decreases by delta.
*/
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_delta(float AUTOTUNE_LIMITS_delta);
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_perc(float AUTOTUNE_LIMITS_perc);
int_fast8_t AOloopControl_set_AUTOTUNE_LIMITS_mcoeff(float AUTOTUNE_LIMITS_mcoeff);

/** @brief Set gain auto tune : on */
int_fast8_t AOloopControl_AUTOTUNE_GAINS_on();
 
/** @brief Set gain auto tune : off */
int_fast8_t AOloopControl_AUTOTUNE_GAINS_off();

/* ======================================================================================================== */
/** @name AOloopControl - 3.5. LOOP CONTROL INTERFACE - PREDICTIVE FILTER ON/OFF AOloopControl_arpf_onoff.c */
/* ======================================================================================================== */

/** @brief ARPF = auto regressive predictive filter: on */
int_fast8_t AOloopControl_ARPFon();

/** @brief ARPF = auto regressive predictive filter: off */
int_fast8_t AOloopControl_ARPFoff();

/* =================================================================================================== */
/** @name AOloopControl - 3.6. LOOP CONTROL INTERFACE - TIMING PARAMETERS - AOloopControl_time_param.c */
/* =================================================================================================== */

/** @brief Set AO loop frequency */
int_fast8_t AOloopControl_set_loopfrequ(float loopfrequ);

/** @brief Set hardware latency in unity of frame */
int_fast8_t AOloopControl_set_hardwlatency_frame(float hardwlatency_frame);

/** @brief Set computation latency of primary DM write in unity of frame */
int_fast8_t AOloopControl_set_complatency_frame(float complatency_frame);

/** @brief Set computation latency of filtered DM write mode
* time between the moment where the WF arrives at the WFS, and when it's written in the DM
*/
int_fast8_t AOloopControl_set_wfsmextrlatency_frame(float wfsmextrlatency_frame);

/* ========================================================================================================= */
/** @name AOloopControl - 3.7. LOOP CONTROL INTERFACE - CONTROL LOOP PARAMETERS - AOloopControl_loop_param.c */
/* ========================================================================================================= */

int_fast8_t AOloopControl_setRTLOG_ON();
int_fast8_t AOloopControl_setRTLOG_OFF();

/** @brief Set gain of the loop  */
int_fast8_t AOloopControl_setgain(float gain);

/** @brief Set ARPF gain (auto regressive predictive filter) 
* Ex : a gain of 0.5 will correct 50% of the predicted WF
*/
int_fast8_t AOloopControl_setARPFgain(float gain);


/** @brief Set ARPF */
int_fast8_t AOloopControl_setARPFgainAutoMin(float val);

/** @brief Set ARPF */
int_fast8_t AOloopControl_setARPFgainAutoMax(float val);


/** @brief Coefficient attenuates AO correction in low loght level */
int_fast8_t AOloopControl_setWFSnormfloor(float WFSnormfloor);

/** @brief Set the limit maximum */
int_fast8_t AOloopControl_setmaxlimit(float maxlimit);

/** @brief Multiplying coefficient, close to 1, in order to avoid divergence */
int_fast8_t AOloopControl_setmult(float multcoeff);

/** @brief Set an average of frames */
int_fast8_t AOloopControl_setframesAve(long nbframes);

/** @brief Set gain of block of modes */
int_fast8_t AOloopControl_set_modeblock_gain(long loop, long blocknb, float gain, int add);// modal blocks

/** @brief Scan block gains */
int_fast8_t AOloopControl_scanGainBlock(long NBblock, long NBstep, float gainStart, float gainEnd, long NBgain);



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 4. FOCAL PLANE SPECKLE MODULATION / CONTROL - AOloopControl_fpspeckle_mod.c
 *  custom FP AO routines */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief Optimize PSF low order */
int_fast8_t AOloopControl_OptimizePSF_LO(const char *psfstream_name, const char *IDmodes_name, const char *dmstream_name, long delayframe, long NBframes);

/** @brief Experimental dm modulation  */
int_fast8_t AOloopControl_DMmodulateAB(const char *IDprobeA_name, const char *IDprobeB_name, const char *IDdmstream_name, const char *IDrespmat_name, const char *IDwfsrefstream_name, double delay, long NBprobes);




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 5. PROCESS LOG FILES - AOloopControl_process_files.c
 *  process log files */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief Log the process of the mode evaluation  */
int_fast8_t AOloopControl_logprocess_modeval(const char *IDname);

/** @brief tweak zonal response matrix in accordance to WFS response to modes */
long AOloopControl_TweakRM(char *ZRMinname, char *DMinCname, char *WFSinCname, char *DMmaskname, char *WFSmaskname, char *RMoutname);





/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 6. REAL-TIME LOGGING - AOloopControl_RTstreamLOG.c
 *  Log real-time streams */
/* =============================================================================================== */
/* =============================================================================================== */

int AOloopControl_RTstreamLOG_init(int loop);
int AOloopControl_RTstreamLOG_setup(long loop, long rtlindex, char *streamname);
void AOloopControl_RTstreamLOG_update(long loop, long rtlindex, struct timespec tnow);
int AOloopControl_RTstreamLOG_printstatus(int loop);
int AOloopControl_RTstreamLOG_GUI(int loop);
int AOloopControl_RTstreamLOG_saveloop(int loop, char *dirname);

int AOloopControl_RTstreamLOG_set_saveON(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_saveOFF(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_ON(int loop, int rtlindex);
int AOloopControl_RTstreamLOG_set_OFF(int loop, int rtlindex);

/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 7. OBSOLETE ?                                                           */ 
/* =============================================================================================== */
/* =============================================================================================== */

// "old" blocks (somewhat obsolete)
int_fast8_t AOloopControl_setgainrange(long m0, long m1, float gainval);
int_fast8_t AOloopControl_setlimitrange(long m0, long m1, float limval);
int_fast8_t AOloopControl_setmultfrange(long m0, long m1, float multfval);
int_fast8_t AOloopControl_setgainblock(long mb, float gainval);
int_fast8_t AOloopControl_setlimitblock(long mb, float limitval);
int_fast8_t AOloopControl_setmultfblock(long mb, float multfval);

int_fast8_t AOloopControl_AutoTune();



#endif
