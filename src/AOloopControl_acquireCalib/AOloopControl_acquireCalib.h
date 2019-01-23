/**
 * @file    AOloopControl_acquireCalib.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine acquire calibration
 * 
 * AO engine uses stream data structure
 * 
 * @author  O. Guyon
 * @date    25 Aug 2017
 *
 * @bug No known bugs. 
 * 
 */

#ifndef _AOLOOPCONTROL_ACQUIRECALIB_H
#define _AOLOOPCONTROL_ACQUIRECALIB_H


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_acquireCalib();

/** @brief Initialize command line interface. */
int_fast8_t init_AOloopControl_acquireCalib();



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_acquireCalib - 1. ACQUIRING CALIBRATION
 *  Measure system response */
/* =============================================================================================== */
/* =============================================================================================== */

long AOloopControl_acquireCalib_mkRandomLinPokeSequence(char *IDmodeC_name, long NBpokemap, char *IDpokemapC_name, char *IDpokeC_name);

/** @brief Acquire WFS response to a series of DM pattern */
long AOloopControl_acquireCalib_Measure_WFSrespC(long loop, long delayfr, long delayRM1us, long NBave, long NBexcl, const char *IDpokeC_name, const char *IDoutC_name, int normalize, int AOinitMode, long NBcycle, uint32_t SequInitMode);


/** @brief Measure linear response to set of DM modes/patterns */
long AOloopControl_acquireCalib_Measure_WFS_linResponse(long loop, float ampl, long delayfr, long delayRM1us, long NBave, long NBexcl, const char *IDpokeC_name, const char *IDrespC_name, const char *IDwfsref_name, int normalize, int AOinitMode, long NBcycle, long NBinnerCycle);


long AOloopControl_acquireCalib_Measure_zonalRM(long loop, double ampl, long delayfr, long delayRM1us, long NBave, long NBexcl, const char *zrespm_name, const char *WFSref_name, const char *WFSmap_name, const char *DMmap_name, long mode, int normalize, int AOinitMode, long NBcycle);


int_fast8_t AOloopControl_acquireCalib_Measure_Resp_Matrix(long loop, long NbAve, float amp, long nbloop, long fDelay, long NBiter);


long AOloopControl_acquireCalib_RespMatrix_Fast(const char *DMmodes_name, const char *dmRM_name, const char *imWFS_name, long semtrig, float HardwareLag, float loopfrequ, float ampl, const char *outname);


long AOloopControl_acquireCalib_RMseries_deinterlace(int NBRM, int refstart, int refend, char *IDout_name);


#endif
