/**
 * @file    AOloopControl_acquireCalib.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine acquire calibration
 * 
 * AO engine uses stream data structure
 * 
 */

#ifndef _AOLOOPCONTROL_ACQUIRECALIB_H
#define _AOLOOPCONTROL_ACQUIRECALIB_H


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_acquireCalib();

/** @brief Initialize command line interface. */
errno_t init_AOloopControl_acquireCalib();



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_acquireCalib - 1. ACQUIRING CALIBRATION
 *  Measure system response */
/* =============================================================================================== */
/* =============================================================================================== */

imageID AOloopControl_acquireCalib_mkRandomLinPokeSequence(
    char  *IDmodeC_name,  // input
    long   NBpokemap,  // input
    char  *IDpokemapC_name,
    char  *IDpokeC_name
);

/** @brief Acquire WFS response to a series of DM pattern */
imageID AOloopControl_acquireCalib_Measure_WFSrespC(
    long        loop,
    long        delayfr,
    long        delayRM1us,
    uint32_t    NBave,
    uint32_t    NBexcl,
    const char *IDpokeC_name,
    const char *IDoutC_name,
    int         normalize,
    int         AOinitMode,
    uint32_t    NBcycle,
    uint32_t    SequInitMode
);

/** @brief Measure linear response to set of DM modes/patterns */
errno_t AOcontrolLoop_acquireCalib_Measure_WFS_linResponse_FPCONF();
errno_t AOcontrolLoop_acquireCalib_Measure_WFS_linResponse_RUN();

errno_t AOloopControl_acquireCalib_Measure_WFS_linResponse(
    __attribute__((unused)) long        loop,
    __attribute__((unused)) float       ampl,
    __attribute__((unused)) long        delayfr,          /// Frame delay [# of frame]
    __attribute__((unused)) long        delayRM1us,       /// Sub-frame delay [us]
    __attribute__((unused)) long        NBave,            /// Number of frames averaged for a single poke measurement
    __attribute__((unused)) long        NBexcl,           /// Number of frames excluded
    __attribute__((unused)) const char *IDpokeC_name,
    __attribute__((unused)) const char *IDrespC_name,
    __attribute__((unused)) const char *IDwfsref_name,
    __attribute__((unused)) int         normalize,
    __attribute__((unused)) int         AOinitMode,
    __attribute__((unused)) long        NBcycle,         /// Number of measurement cycles to be repeated
    __attribute__((unused)) long        NBinnerCycle     /// Number of inner cycles (how many consecutive times should a single +/- poke be repeated)
);

imageID AOloopControl_acquireCalib_Measure_zonalRM(
    long        loop,
    double      ampl,
    long        delayfr,
    long        delayRM1us,
    uint32_t    NBave,
    uint32_t    NBexcl,
    const char *zrespm_name,
    const char *WFSref0_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    long        mode,
    int         normalize,
    int         AOinitMode,
    uint32_t    NBcycle
);

errno_t AOloopControl_acquireCalib_Measure_Resp_Matrix(
    long      loop,
    uint32_t  NbAve,
    float     amp,
    long      nbloop,
    long      fDelay,
    uint64_t  NBiter
);

long AOloopControl_acquireCalib_RespMatrix_Fast(
    const char *DMmodes_name,
    const char *dmRM_name,
    const char *imWFS_name,
    long        semtrig,
    float       HardwareLag,
    float       loopfrequ,
    float       ampl,
    const char *outname
);

long AOloopControl_acquireCalib_RMseries_deinterlace(int NBRM, int refstart, int refend, char *IDout_name, int dmode, int NBtstep);


#endif
