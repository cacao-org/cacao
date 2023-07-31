/**
 * @file    AOloopControl_IOtools_RTLOGsave.c
 * @brief   Save realtime buffers
 *
 */

#define _GNU_SOURCE

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "statistic/statistic.h"

// extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c


/*errno_t AOloopControl_IOtools_RTLOGsave(long        loop,
                                        const char *streamname,
                                        __attribute__((unused))
                                        const char *dirname)
{
    // data buffers
    char                            imnameb0[STRINGMAXLEN_IMGNAME];
    char                            imnameb1[STRINGMAXLEN_IMGNAME];
    __attribute__((unused)) imageID IDinb0;
    __attribute__((unused)) imageID IDinb1;

    // info buffers
    char                            imnamebinfo0[STRINGMAXLEN_IMGNAME];
    char                            imnamebinfo1[STRINGMAXLEN_IMGNAME];
    __attribute__((unused)) imageID IDinbinfo0;
    __attribute__((unused)) imageID IDinbinfo1;

    WRITE_IMAGENAME(imnameb0, "aol%ld_%s_logbuff0", loop, streamname);

    WRITE_IMAGENAME(imnameb1, "aol%ld_%s_logbuff1", loop, streamname);

    WRITE_IMAGENAME(imnamebinfo0, "aol%ld_%s_logbuffinfo0", loop, streamname);

    WRITE_IMAGENAME(imnamebinfo1, "aol%ld_%s_logbuffinfo1", loop, streamname);

    printf("DATA buffers:  %s  %s\n", imnameb0, imnameb1);
    printf("INFO buffers:  %s  %s\n", imnamebinfo0, imnamebinfo1);

    IDinb0     = read_sharedmem_image(imnameb0);
    IDinb1     = read_sharedmem_image(imnameb1);
    IDinbinfo0 = read_sharedmem_image(imnamebinfo0);
    IDinbinfo1 = read_sharedmem_image(imnamebinfo1);

    list_image_ID();


    return RETURN_SUCCESS;
}
*/
