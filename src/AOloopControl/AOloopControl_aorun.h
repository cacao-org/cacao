#ifndef _AOLOOPCONTROL_aorun_H
#define _AOLOOPCONTROL_aorun_H



#define MAX_NUMBER_TIMER 100



typedef struct
{
	/* SETTINGS / CONF  ********************************************** */

    char             respMname[80];
    char             contrMname[80];	
    char             name[80];
	int             CMMODE;                     /**< Combined matrix. 0: matrix is WFS pixels -> modes, 1: matrix is WFS pixels -> DM actuators */


	/* STATUS ******************************************************** */
	
	int         init;                       /**< Has the structure been initialized ? */
    uint64_t    cnt;                        /**<  loop step counter, set to zero every time loop is stopped */
    uint64_t    cntmax;                     /**<  max value of counter, used to step loop */
    uint64_t    DMupdatecnt;                /**<  number of DM updates */

    int      init_RM;                    /**< Response Matrix loaded */
    int      init_CM;                    /**< Control Matrix loaded */
    int      init_CMc;                   /**< Combined control matrix computed */
    int      init_wfsref0;               /**< WFS reference image loaded */
    int      initmapping;
	uint64_t         LOOPiteration;              /**< Loop iteration - set to zero on creation on aolrun start */
	
	
	/* CONTROL ******************************************************** */
	
	int      kill;                       /**<  set to 1 to kill computation loop */
	int      on;                         /**< goes to 1 when loop starts, put to 0 to turn loop off */
	int      DMprimaryWriteON;           /**< primary DM write */
	int      DMfilteredWriteON;          /**< Filtered write to DM */
	
	float            maxlimit;                   /**< maximum absolute value for mode values */
    float            mult;                       /**< multiplication coefficient to be applied at each loop iteration */
	float            gain;                       /**< overall loop gain */

	// predictive control (auto-regressive predictive filter)
	int      ARPFon;                     /**< 1 if auto-regressive predictive filter is ON */
	float            ARPFgain; 
	float            ARPFgainAutoMin;
	float            ARPFgainAutoMax;
	
} AOLOOPCONF_aorun;







typedef struct
{
    float            loopfrequ;                  /**< Loop frequency [Hz] */

    // Hardware latency = time from DM command issued to WFS response changed
    float            hardwlatency;               /**< hardware latency between DM command and WFS response [sec] */
    float            hardwlatency_frame;         /**< hardware latency between DM command and WFS response [frame] */

    // Computation time for direct WFS->DM mode through single matrix multiplication
    float            complatency;                /**< Computation latency [sec] */
    float            complatency_frame;          /**< Computation latency (main loop) from WFS image reception to DM command output [frame] */

    // Computation time for full computation including open loop computation
    float            wfsmextrlatency;            /**< WFS mode extraction latency [sec] */
    float            wfsmextrlatency_frame;      /**< WFS mode extraction latency [frame] */

    int      status;                     /**< loop status for main loop */
    int      statusM;                    /**< loop status for modal loop */
    int      statusM1;                   /**< loop status for modal loop */

    int      GPUstatus[50];              /**<  GPU status index */
    int      NBtimer;                    /**<  Number of active timers - 1 timer per status value */
    struct timespec  timer[MAX_NUMBER_TIMER];    /**<  Timers */

} AOloopTimingInfo;




typedef struct
{
    char WFSname[80];                        
    
    uint32_t    sizexWFS;                   /**< WFS image x size */
    uint32_t    sizeyWFS;                   /**< WFS image y size */
    uint32_t    sizeWFS;                    /**< WFS total image (= x size * y size) */
    uint32_t    activeWFScnt;               /**< Number of active WFS pixels */
    uint32_t    sizeWFS_active[100];        /**< Only takes into account WFS pixels in use/active for each slice */
    uint64_t    WFScnt;                     /**< WFS stream counter 0 value at WFS image read */
    uint64_t    WFScntRM;                   /**< WFS stream counter 0 value at WFS image read (RM acqu mode) */

    int      WFSnormalize;               /**< 1 if each WFS frame should be normalized to 1 */
    int      WFSrefzero;                 /**< 1 if WFS reference is zero */
    float            WFSnormfloor;               /**< normalized by dividing by (total + AOconf[loop].WFSnormfloor)*AOconf[loop].WFSsize */
    float            WFStotalflux;               /**< Total WFS flux after dark subtraction */

} AOLOOPCONF_WFSim;





#endif
