/**
 * @file    AOloopControl_IOtools.h
 * @brief   Function prototypes for Adaptive Optics Control loop engine I/O
 * tools
 *
 * AO engine uses stream data structure
 *
 *
 * @bug No known bugs.
 *
 */

#ifndef _AOLOOPCONTROL_IOTOOLS_H
#define _AOLOOPCONTROL_IOTOOLS_H
#include <AOloopControl/AOloopControl.h>

#ifndef __STDC_LIB_EXT1__
typedef int errno_t;
#endif

/** @brief Initialize module. */
void __attribute__((constructor)) libinit_AOloopControl_IOtools();



errno_t
AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(const char *in_name,
        const char *dark_name,
        const char *out_name,
        long        size_x,
        long        size_y,
        long        xstart,
        long        ystart);

/** @brief compute sum of image pixels */
// static void *compute_function_imtotal( void *ptr );






/** @brief Load 2D image in shared memory */
long AOloopControl_IOtools_2Dloadcreate_shmim(const char *name,
        const char *fname,
        long        xsize,
        long        ysize,
        float       DefaultValue);

/** @brief Load 3D image in shared memory */
long AOloopControl_IOtools_3Dloadcreate_shmim(const char *name,
        const char *fname,
        long        xsize,
        long        ysize,
        long        zsize,
        float       DefaultValue);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING
 *  Data streams real-time processing */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief Average data stream */
errno_t AOloopControl_IOtools_AveStream(const char *IDname,
                                        double      alpha,
                                        const char *IDname_out_ave,
                                        const char *IDname_out_AC,
                                        const char *IDname_out_RMS);

/** @brief Aligns data stream */
errno_t AOloopControl_IOtools_imAlignStream(const char *IDname,
        int         xbox0,
        int         ybox0,
        const char *IDref_name,
        const char *IDout_name,
        int         insem);

/** @brief Induces temporal offset between input and output streams */
long AOloopControl_IOtools_frameDelay(const char *IDin_name,
                                      const char *IDkern_name,
                                      const char *IDout_name,
                                      int         insem);

/** @brief Re-arrange a 3D cube into an array of images into a single 2D frame
 */
long AOloopControl_IOtools_stream3Dto2D(const char *in_name,
                                        const char *out_name,
                                        int         NBcols,
                                        int         insem);

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl_IOtools - 4. SAVE REAL-TIME TELEMETRY BUFFER
 *  Save to disk telemetry packaged in alternate buffers */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief Save telemetry */
errno_t AOloopControl_IOtools_RTLOGsave(long        loop,
                                        const char *streamname,
                                        const char *dirname);

#endif
