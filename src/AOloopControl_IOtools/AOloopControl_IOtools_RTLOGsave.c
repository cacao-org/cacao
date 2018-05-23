/**
 * @file    AOloopControl_IOtools_RTLOGsave.c
 * @brief   Save realtime buffers
 * 
 * @author  O. Guyon
 *
 * 
 */



#define _GNU_SOURCE



#include <string.h>
#include <stdint.h>
#include <math.h>
#include "statistic/statistic.h"
#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"








extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c






int_fast8_t AOloopControl_IOtools_RTLOGsave(long loop, const char *streamname, const char *dirname)
{
	// data buffers
	char imnameb0[500];
    char imnameb1[500];
	
	// info buffers
	char imnamebinfo0[500];
    char imnamebinfo1[500];
	

    if(sprintf(imnameb0, "aol%ld_%s_logbuff0", loop, streamname) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(imnameb1, "aol%ld_%s_logbuff1", loop, streamname) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(imnamebinfo0, "aol%ld_%s_logbuffinfo0", loop, streamname) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if(sprintf(imnamebinfo1, "aol%ld_%s_logbuffinfo1", loop, streamname) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

   
   
   
   
    return(0);
}
