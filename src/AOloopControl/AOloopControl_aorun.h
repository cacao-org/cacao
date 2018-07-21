#ifndef _AOLOOPCONTROL_aorun_H
#define _AOLOOPCONTROL_aorun_H



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




#endif
