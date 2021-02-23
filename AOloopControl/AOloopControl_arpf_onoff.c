/**
 * @file    AOloopControl_arpf_onoff.c 
 * @brief   AO loop control - Predictive Filter ON/OFF through the ARPF multiplicator
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * 
 */
 
 
#define _GNU_SOURCE

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

#include "AOloopControl/AOloopControl.h"


// defined in AOloopControl.c
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var;




errno_t AOloopControl_ARPFon()
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.ARPFon = 1;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}


errno_t AOloopControl_ARPFoff()
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.ARPFon = 0;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return RETURN_SUCCESS;
}
