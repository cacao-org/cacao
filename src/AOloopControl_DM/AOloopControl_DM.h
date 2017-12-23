#ifndef _AOLOOPCONTROL_DM_H
#define _AOLOOPCONTROL_DM_H



#define DISPCOMB_FILENAME_CONF "/tmp/dmdispcombconf.conf.shm"
#define DMTURBCONF_FILENAME "/tmp/dmturb.conf.shm"

#define DM_NUMBER_CHANMAX 20 // max number of channel per DM




/* =============================================================================================== */
/*
 * DM description and status
 * contains DM physical characteristics, settings
 * defines how DM is controlled (input and output)
 * sets up DM channels, allowing DM control to be split in parallel channels
 */
 /* =============================================================================================== */
 
typedef struct
{
    int ON;
    
    long xsize;        // DM xsize
    long ysize;        // DM ysize
    long xysize;       // total number of actuators
    long NBchannel;    // number of control channels
    
    long loopcnt;
    long updatecnt;    // DM update counter 
    int busy;          // if set to 1, hold off and wait

    int voltmode;      // 1 if DM drives voltmap
    long IDvolt;
    char voltname[200];
    int voltON;        // 1 if applying voltage 
    float MAXVOLT;     // maximum voltage on DM
	int AveMode;
    float DClevel;

	int TrigMode; // 0 (std) : any channel update triggers disp update, 1: use specific channel and semaphore
	int TrigChan; // if TrigMode = 1, use this channel for trigger
	int TrigSem;  // if TrigMode = 1, use this semaphore for trigger
	

	long nsecwait; // inner wait loop duration, interrupted if sem[1] of disp posted
    struct timespec tstart;
    struct timespec tend;
	double tdelay;
	double time_disp2V;



    long dmdispID[DM_NUMBER_CHANMAX];
    float dmdispgain[DM_NUMBER_CHANMAX];
    long dmdispcnt[DM_NUMBER_CHANMAX];
    long IDdisp;
 
    int dm2dm_mode; // 1 if output disp should be remapped to output DM disp
    // following only applies of dm2dm_mode = 1
    long xsizeout;
    long ysizeout;
    long ID_dm2dm_DMmodes;
    char dm2dm_DMmodes_name[200];
    long ID_dm2dm_outdisp;
    char dm2dm_outdisp_name[200];
    
    int wfsrefmode; // 1 if wfsref offset should be computed
    long xsizewfsref;
    long ysizewfsref;
    long ID_wfsref_RespMat;
    char wfsref_RespMat_name[200];
    long ID_wfsref_out;
    char wfsref_out_name[200];

    int status;
    long moninterval; // [us]

} AOLOOPCONTROL_DM_DISPCOMB_CONF;





typedef struct
{
    int on;
    long cnt;

    double wspeed; // wind speed [m/s]
    double ampl; // [um RMS]
    double LOcoeff; // 0 for full correction of low orders, 1 for no correction

    long tint; // interval between consecutive DM updates [us]


    double simtime;

    struct timespec tstart;
    struct timespec tend;

} AOLOOPCONTROL_DMTURBCONF;






int init_AOloopControl_DM();



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/*  TOOLBOX                                                                                        */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

struct timespec time_diff(struct timespec start, struct timespec end);

int make_master_turbulence_screen_local(const char *ID_name1, const char *ID_name2, long size, float outerscale, float innerscale);




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 1. INITIALIZATION, LOAD/CREATE                                                                  */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

static int AOloopControl_DM_createconf();

int AOloopControl_DM_loadconf();

int AOloopControl_DM_unloadconf();





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int AOloopControl_DM_disp2V(long DMindex);

int AOloopControl_DM_CombineChannels(long DMindex, long xsize, long ysize, int NBchannel, int AveMode, int dm2dm_mode, const char *dm2dm_DMmodes, const char *dm2dm_outdisp, int wfsrefmode, const char *wfsref_WFSRespMat, const char *wfsref_out, int voltmode, const char *IDvolt_name, float DClevel, float maxvolt);


int AOloopControl_DM_dmdispcomboff(long DMindex);

int AOloopControl_DM_dmtrigoff(long DMindex);





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 3. CONFIGURATION                                                                                */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int AOloopControl_printDMconf();

int AOloopControl_DM_dmdispcombstatus(long DMindex);

int AOloopControl_DM_chan_setgain(long DMindex, int ch, float gain);

int AOloopControl_DM_setvoltON(long DMindex);

int AOloopControl_DM_setvoltOFF(long DMindex);

int AOloopControl_DM_setMAXVOLT(long DMindex, float maxvolt);

int AOloopControl_DM_setDClevel(long DMindex, float DClevel);

int AOloopControl_DM_setAveMode(long DMindex, int AveMode);

int AOloopControl_DM_setTrigMode(long DMindex, int mode);

int AOloopControl_DM_setTrigChan(long DMindex, int chan);

int AOloopControl_DM_setTrigSem(long DMindex, int sem);



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int_fast8_t AOloopControl_printDMturbconf();

int AOloopControl_DMturb_createconf();

int AOloopControl_DMturb_loadconf();

int AOloopControl_DM_dmturboff(long DMindex);

int AOloopControl_DM_dmturb_wspeed(long DMindex, double wspeed);

int AOloopControl_DM_dmturb_ampl(long DMindex, double ampl);

int AOloopControl_DM_dmturb_LOcoeff(long DMindex, double LOcoeff);

int AOloopControl_DM_dmturb_tint(long DMindex, long tint);

int AOloopControl_DM_dmturb_printstatus(long DMindex);

int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples);



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 5. MISC TESTS & UTILS                                                                           */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

long AOloopControl_mkDM_TT_circle(char *IDoutname, long DMindex, long NBpts, float ampl);

long AOloopControl_DM_mkAstroGrid_seq(char *IDoutname, long DMindex, int XYmode, int bin, long NBcycle);


#endif

