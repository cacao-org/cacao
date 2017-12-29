/**
 * @file    AOloopControl_compTools.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine misc computation tools
 * 
 * AO engine uses stream data structure
 * 
 * @author  O. Guyon
 * @date    28 Aug 2017
 *
 * @bug No known bugs. 
 * 
 */

#ifndef _AOLOOPCONTROL_COMPTOOLS_H
#define _AOLOOPCONTROL_COMPTOOLS_H


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_compTools();

/** @brief Initialize command line interface. */
int_fast8_t init_AOloopControl_compTools();




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS      
 *  Useful tools */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief compute cross product between two 3D arrays */
long AOloopControl_compTools_CrossProduct(const char *ID1_name, const char *ID2_name, const char *IDout_name);


/** @brief Create simple zonal poke cube */
long AOloopControl_compTools_mkSimpleZpokeM( long dmxsize, long dmysize, char *IDout_name);



#endif
