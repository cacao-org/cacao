/**
 * @file    AOloopControl_IOtools.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine I/O tools
 * 
 * AO engine uses stream data structure
 * 
 * @author  O. Guyon
 * @date    25 Aug 2017
 *
 * @bug No known bugs. 
 * 
 */

#ifndef _AOLOOPCONTROL_IOTOOLS_H
#define _AOLOOPCONTROL_IOTOOLS_H
#include <AOloopControl/AOloopControl.h>


/** @brief Initialize module. */
void __attribute__ ((constructor)) libinit_AOloopControl_IOtools();

/** @brief Initialize command line interface. */
int_fast8_t init_AOloopControl_IOtools();




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */


int_fast8_t AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(const char *in_name, const char *dark_name, const char *out_name, long size_x, long size_y, long xstart, long ystart);

/** @brief compute sum of image pixels */
static void *compute_function_imtotal( void *ptr );

/** @brief Subtract dark */
static void *compute_function_dark_subtract( void *ptr );

/** @brief Read image from WFS camera */
int_fast8_t Read_cam_frame(long loop, int RM, int normalize, int PixelStreamMode, int InitSem);







/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 2. LOAD DATA STREAMS     
 *  Load 2D and 3D shared memory images */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief Load 2D image in shared memory */
long AOloopControl_IOtools_2Dloadcreate_shmim(const char *name, const char *fname, long xsize, long ysize, float DefaultValue);

/** @brief Load 3D image in shared memory */
long AOloopControl_IOtools_3Dloadcreate_shmim(const char *name, const char *fname, long xsize, long ysize, long zsize, float DefaultValue);







/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING      
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief Average data stream */
int_fast8_t AOloopControl_IOtools_AveStream(const char *IDname, double alpha, const char *IDname_out_ave, const char *IDname_out_AC, const char *IDname_out_RMS);

/** @brief Induces temporal offset between input and output streams */
long AOloopControl_IOtools_frameDelay(const char *IDin_name, const char *IDkern_name, const char *IDout_name, int insem);

/** @brief Re-arrange a 3D cube into an array of images into a single 2D frame */
long AOloopControl_IOtools_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem);



#endif
