#ifndef _AOLOOPCONTROL_aorun_H
#define _AOLOOPCONTROL_aorun_H



#define MAX_NUMBER_TIMER 100



typedef struct
{
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




#endif
