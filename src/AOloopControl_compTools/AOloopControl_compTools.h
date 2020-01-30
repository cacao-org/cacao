/**
 * @file    AOloopControl_compTools.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine misc computation tools
 * 
 * AO engine uses stream data structure
 * . 
 * 
 */

#ifndef _AOLOOPCONTROL_COMPTOOLS_H
#define _AOLOOPCONTROL_COMPTOOLS_H


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_compTools();

/** @brief Initialize command line interface. */
errno_t init_AOloopControl_compTools();




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS      
 *  Useful tools */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief compute cross product between two 3D arrays */
imageID AOloopControl_compTools_CrossProduct(
    const char *ID1_name,
    const char *ID2_name,
    const char *IDout_name
);


/** @brief Create simple zonal poke cube */
imageID AOloopControl_compTools_mkSimpleZpokeM(
    uint32_t  dmxsize,
    uint32_t  dmysize,
    char     *IDout_name
);



#endif
