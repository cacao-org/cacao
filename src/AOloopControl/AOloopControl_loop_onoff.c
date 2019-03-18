/**
 * @file    AOloopControl_loop_onoff.c 
 * @brief   AO loop Control functions wave front sensor and deformable mirror 
 * 
 * REAL TIME COMPUTING ROUTINES
 * 
 * @bug No known bugs.
 * 
 */

#define _GNU_SOURCE

#include "AOloopControl.h"



#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;


//aoloopcontrol_var.LOOPNUMBER = 0; // current loop index



/* =============================================================================================== */
/** @name AOloopControl - 3.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET
 *  Set parameters */
/* =============================================================================================== */



int_fast8_t AOloopControl_loopon()
{
	int rtlindex;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.cntmax = AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.cnt-1;

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.on = 1;
    
    // initialize RT logging frame indices
    for (rtlindex=0; rtlindex<MAX_NUMBER_RTLOGSTREAM; rtlindex++ )
		AOconf[aoloopcontrol_var.LOOPNUMBER].RTSLOGarray[rtlindex].frameindex = 0;
	
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopoff()
{
	int rtlindex;
		
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.on = 0;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);
	
	

    return 0;
}


int_fast8_t AOloopControl_loopWFScompon()
{
	int rtlindex;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.cntmax = AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.cnt-1;

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOcompute.ComputeWFSsol_FLAG = 1;
    
    // initialize RT logging frame indices
    for (rtlindex=0; rtlindex<MAX_NUMBER_RTLOGSTREAM; rtlindex++ )
		AOconf[aoloopcontrol_var.LOOPNUMBER].RTSLOGarray[rtlindex].frameindex = 0;
	
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopWFScompoff()
{
	int rtlindex;
		
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].AOcompute.ComputeWFSsol_FLAG = 0;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);
	

    return 0;
}




int_fast8_t AOloopControl_loopkill()
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.kill = 1;

    return 0;
}


int_fast8_t AOloopControl_loopstep(long loop, long NBstep)
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[loop].aorun.cntmax = AOconf[loop].aorun.cnt + NBstep;
    AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.RMSmodesCumul = 0.0;
    AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.RMSmodesCumulcnt = 0;

    AOconf[loop].aorun.on = 1;

    while(AOconf[loop].aorun.on==1)
        usleep(100); // THIS WAITING IS OK


    return 0;
}


int_fast8_t AOloopControl_loopreset()
{
    long k;
    long mb;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    if(aoloopcontrol_var.aoconfID_cmd_modes==-1)
    {
        char name[200];
        if(sprintf(name, "DMmode_cmd_%ld", aoloopcontrol_var.LOOPNUMBER) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    AOconf[aoloopcontrol_var.LOOPNUMBER].aorun.on = 0;
    for(k=0; k<AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.NBDMmodes; k++)
        data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = 0.0;

    for(mb=0; mb<AOconf[aoloopcontrol_var.LOOPNUMBER].AOpmodecoeffs.DMmodesNBblock; mb)
    {
        AOloopControl_setgainblock(mb, 0.0);
        AOloopControl_setlimitblock(mb, 0.01);
        AOloopControl_setmultfblock(mb, 0.95);
    }

    return 0;
}
