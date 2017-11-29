/**
 * @file    AOloopControl_wfs_dm.c 
 * @brief   AO loop Control functions wave front sensor and deformable mirror 
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

#include "AOloopControl.h"


//libraries created by O. Guyon 
#include "CommandLineInterface/CLIcore.h"


#define AOconfname "/tmp/AOconf.shm"
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;
extern DATA data;


//aoloopcontrol_var.LOOPNUMBER = 0; // current loop index



/* =============================================================================================== */
/** @name AOloopControl - 3.1. LOOP CONTROL INTERFACE - MAIN CONTROL : LOOP ON/OFF START/STOP/STEP/RESET
 *  Set parameters */
/* =============================================================================================== */



int_fast8_t AOloopControl_loopon()
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].cntmax = AOconf[aoloopcontrol_var.LOOPNUMBER].cnt-1;

    AOconf[aoloopcontrol_var.LOOPNUMBER].on = 1;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopoff()
{
    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].on = 0;
    AOloopControl_perfTest_showparams(aoloopcontrol_var.LOOPNUMBER);

    return 0;
}


int_fast8_t AOloopControl_loopkill()
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[aoloopcontrol_var.LOOPNUMBER].kill = 1;

    return 0;
}


int_fast8_t AOloopControl_loopstep(long loop, long NBstep)
{

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[loop].cntmax = AOconf[loop].cnt + NBstep;
    AOconf[aoloopcontrol_var.LOOPNUMBER].RMSmodesCumul = 0.0;
    AOconf[aoloopcontrol_var.LOOPNUMBER].RMSmodesCumulcnt = 0;

    AOconf[loop].on = 1;

    while(AOconf[loop].on==1)
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

    AOconf[aoloopcontrol_var.LOOPNUMBER].on = 0;
    for(k=0; k<AOconf[aoloopcontrol_var.LOOPNUMBER].NBDMmodes; k++)
        data.image[aoloopcontrol_var.aoconfID_cmd_modes].array.F[k] = 0.0;

    for(mb=0; mb<AOconf[aoloopcontrol_var.LOOPNUMBER].DMmodesNBblock; mb)
    {
        AOloopControl_setgainblock(mb, 0.0);
        AOloopControl_setlimitblock(mb, 0.01);
        AOloopControl_setmultfblock(mb, 0.95);
    }

    return 0;
}
