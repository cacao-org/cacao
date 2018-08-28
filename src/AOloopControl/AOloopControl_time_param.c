/**
 * @file    AOloopControl_time_param.c 
 * @brief   AO loop control - Timing Parameter
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * @author  O. Guyon
 * @date    24 nov 2017
 *
 * 
 * @bug No known bugs.
 * 
 */
 
 
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"

#define AOconfname "/tmp/AOconf.shm"

// defined in AOloopControl.c
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var;





int_fast8_t AOloopControl_set_loopfrequ(float loopfrequ)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOtiminginfo.loopfrequ = loopfrequ;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_hardwlatency_frame(float hardwlatency_frame)
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOtiminginfo.hardwlatency_frame = hardwlatency_frame;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_complatency_frame(float complatency_frame)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOtiminginfo.complatency_frame = complatency_frame;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_set_wfsmextrlatency_frame(float wfsmextrlatency_frame)
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOtiminginfo.wfsmextrlatency_frame = wfsmextrlatency_frame;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}

