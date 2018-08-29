#ifndef _AOLOOPCONTROL_autotune_H
#define _AOLOOPCONTROL_autotune_H





typedef struct
{
	// MODAL AUTOTUNING 
	// limits
	int_fast8_t AUTOTUNE_LIMITS_ON;
	float       AUTOTUNE_LIMITS_perc;              // percentile limit for autotuning
	float       AUTOTUNE_LIMITS_mcoeff;            // multiplicative coeff 
	float       AUTOTUNE_LIMITS_delta;             // autotune loop increment 

	int_fast8_t AUTOTUNE_GAINS_ON;
	float       AUTOTUNEGAINS_updateGainCoeff;      /**< Averaging coefficient (usually about 0.1) */
	float       AUTOTUNEGAINS_evolTimescale;        /**< Evolution timescale, beyond which errors stop growing */
	long        AUTOTUNEGAINS_NBsamples;            /**< Number of samples */	
} AOLOOPCONF_AutoTune;





#endif
