#ifndef _AOLOOPCONTROL_aorun_H
#define _AOLOOPCONTROL_aorun_H



#define MAX_NUMBER_TIMER 100



typedef struct
{
	int_fast8_t   init;                         /**< Has the structure been initialized ? */
    uint_fast64_t cnt;                        /**<  loop step counter, set to zero every time loop is stopped */
    uint_fast64_t cntmax;                     /**<  max value of counter, used to step loop */
    uint_fast64_t DMupdatecnt;                /**<  */
    char name[80];

    int_fast8_t init_RM;                      /**< Response Matrix loaded */
    int_fast8_t init_CM;                      /**< Control Matrix loaded */
    int_fast8_t init_CMc;                     /**< Combined control matrix computed */
    int_fast8_t initmapping;
    char respMname[80];
    char contrMname[80];
	
	
	
	uint64_t      LOOPiteration;                   /**< Loop iteration - set to zero on creation on aolrun start */
	int_fast8_t   kill;                            /**<  set to 1 to kill computation loop */
	int_fast8_t   on;                              /**< goes to 1 when loop starts, put to 0 to turn loop off */
	int_fast8_t   DMprimaryWriteON;                /**< primary DM write */
	int_fast8_t   CMMODE;                          /**< Combined matrix. 0: matrix is WFS pixels -> modes, 1: matrix is WFS pixels -> DM actuators */
	int_fast8_t   DMfilteredWriteON;               /**< Filtered write to DM */
	int_fast8_t   ARPFon;                          /**< 1 if auto-regressive predictive filter is ON */
	
} AOLOOPCONF_aorun;





typedef struct
{
    float loopfrequ;                          /**< Loop frequency [Hz] */

    // Hardware latency = time from DM command issued to WFS response changed
    float hardwlatency;                       /**< hardware latency between DM command and WFS response [sec] */
    float hardwlatency_frame;                 /**< hardware latency between DM command and WFS response [frame] */

    // Computation time for direct WFS->DM mode through single matrix multiplication
    float complatency;                        /**< Computation latency [sec] */
    float complatency_frame;                  /**< Computation latency (main loop) from WFS image reception to DM command output [frame] */

    // Computation time for full computation including open loop computation
    float wfsmextrlatency;                    /**< WFS mode extraction latency [sec] */
    float wfsmextrlatency_frame;              /**< WFS mode extraction latency [frame] */

    int_fast8_t status;                       /**< loop status for main loop */
    int_fast8_t statusM;                      /**< loop status for modal loop */
    int_fast8_t statusM1;                     /**< loop status for modal loop */

    int_fast8_t GPUstatus[50];                /**<  GPU status index */
    uint_fast16_t NBtimer;                    /**<  Number of active timers - 1 timer per status value */
    struct timespec timer[MAX_NUMBER_TIMER];  /**<  Timers */

} AOloopTimingInfo;




typedef struct
{
    char WFSname[80];                        
    
    uint_fast32_t sizexWFS;                   /**< WFS image x size*/
    uint_fast32_t sizeyWFS;                   /**< WFS image y size */
    uint_fast32_t sizeWFS;                    /**< WFS total image (= x size * y size) */
    uint_fast32_t activeWFScnt;               /**< Number of active WFS pixels */
    uint_fast32_t sizeWFS_active[100];        /**< Only takes into account WFS pixels in use/active for each slice */
    uint_fast64_t WFScnt;                     /**< WFS stream counter 0 value at WFS image read */
    uint_fast64_t WFScntRM;                   /**< WFS stream counter 0 value at WFS image read (RM acqu mode) */

    int_fast8_t WFSnormalize;                 /**< 1 if each WFS frame should be normalized to 1 */
    int_fast8_t WFSrefzero;                   /**< 1 if WFS reference is zero */
    float WFSnormfloor;                       /**< normalized by dividing by (total + AOconf[loop].WFSnormfloor)*AOconf[loop].WFSsize */
    float WFStotalflux;                       /**< Total WFS flux after dark subtraction */

} AOLOOPCONF_WFSim;





#endif
