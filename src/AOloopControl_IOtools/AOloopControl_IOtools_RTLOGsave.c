/**
 * @file    AOloopControl_IOtools_RTLOGsave.c
 * @brief   Save realtime buffers
 * 
 */



#define _GNU_SOURCE



#include <string.h>
#include <stdint.h>
#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "statistic/statistic.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"








extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c






errno_t AOloopControl_IOtools_RTLOGsave(
    long        loop,
    const char *streamname,
    __attribute__((unused)) const char *dirname
)
{
    // data buffers
    char imnameb0[STRINGMAXLEN_IMGNAME];
    char imnameb1[STRINGMAXLEN_IMGNAME];
    __attribute__((unused)) imageID IDinb0;
    __attribute__((unused)) imageID IDinb1;

    // info buffers
    char imnamebinfo0[STRINGMAXLEN_IMGNAME];
    char imnamebinfo1[STRINGMAXLEN_IMGNAME];
    __attribute__((unused)) imageID IDinbinfo0;
    __attribute__((unused)) imageID IDinbinfo1;


    WRITE_IMAGENAME(imnameb0, "aol%ld_%s_logbuff0", loop, streamname);

    WRITE_IMAGENAME(imnameb1, "aol%ld_%s_logbuff1", loop, streamname);

    WRITE_IMAGENAME(imnamebinfo0, "aol%ld_%s_logbuffinfo0", loop, streamname);

    WRITE_IMAGENAME(imnamebinfo1, "aol%ld_%s_logbuffinfo1", loop, streamname);

    printf("DATA buffers:  %s  %s\n", imnameb0, imnameb1);
    printf("INFO buffers:  %s  %s\n", imnamebinfo0, imnamebinfo1);

    IDinb0 = read_sharedmem_image(imnameb0);
    IDinb1 = read_sharedmem_image(imnameb1);
    IDinbinfo0 = read_sharedmem_image(imnamebinfo0);
    IDinbinfo1 = read_sharedmem_image(imnamebinfo1);

    list_image_ID();

    /*    cnt0_old = data.image[IDinb0].md[0].cnt0;
        cnt1_old = data.image[IDinb1].md[0].cnt0;

        xsize = data.image[IDinb0].md[0].size[0];
        ysize = data.image[IDinb0].md[0].size[1];
        xysize = xsize*ysize;
        zsize = data.image[IDinb0].md[0].size[2];
        atype = data.image[IDinb0].md[0].atype;
      */

    return RETURN_SUCCESS;
}
