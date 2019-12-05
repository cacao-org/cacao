/**
 * @file    AOloopControl_DM.c
 * @brief   DM control
 * 
 * To be used for AOloopControl module
 *  
 *
 * 
 */





#define AOLOOPCONTROL_DM_LOGDEBUG 1



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */




#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl_DM/AOloopControl_DM.h"






/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

#if !defined(AOLOOPCONTROL_DM_LOGDEBUG) || defined(STANDALONE)
#define TESTPOINT(...)
#endif






static int INITSTATUS_AOloopControl_DM = 0;

long NB_DMindex = 9;

//AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
int dmdispcomb_loaded = 0;
int SMfd;


//AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
int dmturb_loaded = 0;
int SMturbfd;



// CLI commands
//
// function CLI_checkarg used to check arguments
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string or existing image








/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 1. INITIALIZATION, LOAD/CREATE                                                                  */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



int_fast8_t AOloopControl_DM_CombineChannels_cli() {
    // 1  long DMindex
    // 2  long xsize
    // 3  long ysize
    // 4  int NBchannel
    // 5  int AveMode
    // 6  int dm2dm_mode
    // 7  char *dm2dm_DMmodes
    // 8  char *dm2dm_outdisp
    // 9  int wfsrefmode
    // 10 char *wfsref_WFSRespMat
    // 11 char *wfsref_out
    // 12 int voltmode
    // 13 int volttype     : voltage command type (1: linear bipolar, output is float) (2: quadratic unipolar, output is UI16)
    // 14 float stroke100  : displacement [um] for 100 V
    // 15 char *IDvolt_name
    // 16 float DCum
    // 17 float maxvolt

	int stringlenmax = 200;
    char fpsname[stringlenmax];



    if(CLI_checkarg(1, 5) + CLI_checkarg(2, 2) == 0) {
        unsigned int DMindex = (unsigned int) data.cmdargtoken[2].val.numl;

        // FPS interface name
        if(data.processnameflag == 0) { // name fps to something different than the process name
            snprintf(fpsname, stringlenmax, "DMcomb-%s", data.cmdargtoken[2].val.string);
            //sprintf(fpsname, "DMcomb-%06u", DMindex);
        } else { // Set fps name to be process name up to first instance of character '.'
            strcpy(fpsname, data.processname0);
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_FPSINIT_") == 0) && (CLI_checkarg(2, 2) == 0)) {  // init FPS
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, CMDCODE_FPSINIT, data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_CONFSTART_") == 0) && (CLI_checkarg(2, 2) == 0)) {  // Start conf process
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, CMDCODE_CONFSTART, data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_CONFSTOP_") == 0) && (CLI_checkarg(2, 2) == 0)) { // Stop conf process
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, CMDCODE_CONFSTOP, data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_RUNSTART_") == 0) && (CLI_checkarg(2, 2) == 0)) { // Run process
            printf("Run function\n");
            AOloopControl_DM_CombineChannels_RUN(fpsname);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_RUNSTOP_") == 0) && (CLI_checkarg(2, 2) == 0)) { // Run process
            printf("Run function\n");
            AOloopControl_DM_dmdispcomboff(data.cmdargtoken[2].val.numl);
            return 0;
        }
    }

    // FPS-free implementation - all parameters specified at function launch
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) + CLI_checkarg(3, 2) + CLI_checkarg(4, 2) + CLI_checkarg(5, 2) + CLI_checkarg(6, 2) + CLI_checkarg(7, 5) + CLI_checkarg(8, 5) + CLI_checkarg(9, 2) + CLI_checkarg(10, 5) + CLI_checkarg(11, 5) + CLI_checkarg(12, 2) + CLI_checkarg(13, 2) + CLI_checkarg(14, 1) + CLI_checkarg(15, 5) + CLI_checkarg(16, 1) + CLI_checkarg(17, 1) == 0) {
        AOloopControl_DM_CombineChannels(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numl, data.cmdargtoken[7].val.string, data.cmdargtoken[8].val.string, data.cmdargtoken[9].val.numl, data.cmdargtoken[10].val.string, data.cmdargtoken[11].val.string, data.cmdargtoken[12].val.numl, data.cmdargtoken[13].val.numl, data.cmdargtoken[14].val.numf, data.cmdargtoken[15].val.string, data.cmdargtoken[16].val.numf, data.cmdargtoken[17].val.numf);
        return 0;
    } else {
        // launch configuration process
        return 1;
    }

    //   {// DEFAULT: no dm2dm, no wfsref, dmvolt output
    //printf("AUTO volt_name = %s\n", data.cmdargtoken[13].val.string);
    //     AOloopControl_DM_CombineChannels(0, 50, 50, 12, 1, 0, "dmmodes", "outdisp", 0, "wfsrm", "refout", 1, 2, 0.7, "dmvolt", 0.0, 150.0);
    // }

}




int_fast8_t AOloopControl_DM_dmdispcomboff_cli() {
    if(CLI_checkarg(1, 2) == 0) {
        AOloopControl_DM_dmdispcomboff(data.cmdargtoken[1].val.numl);
        return 0;
    }    else {
        return 1;
    }
}

int_fast8_t AOloopControl_DM_dmtrigoff_cli() {
    if(CLI_checkarg(1, 2) == 0) {
        AOloopControl_DM_dmtrigoff(data.cmdargtoken[1].val.numl);
        return 0;
    }    else {
        return 1;
    }
}



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 3. CONFIGURATION                                                                                */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int_fast8_t AOloopControl_DM_dmdispcombstatus_cli(){
    if(CLI_checkarg(1,2)==0){
        AOloopControl_DM_dmdispcombstatus(data.cmdargtoken[1].val.numl);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_chan_setgain_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,1)==0) {
        AOloopControl_DM_chan_setgain(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return 0;}    else      return 1; }

int_fast8_t AOloopControl_DM_setvoltON_cli() {
	if(CLI_checkarg(1,2)==0) {
        AOloopControl_DM_setvoltON(data.cmdargtoken[1].val.numl);
        return 0; }    else       return 1; }

int_fast8_t AOloopControl_DM_setvoltOFF_cli() {
	if(CLI_checkarg(1,2)==0) {
        AOloopControl_DM_setvoltOFF(data.cmdargtoken[1].val.numl);
	return 0;}    else       return 1;}

int_fast8_t AOloopControl_DM_setMAXVOLT_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0) {
        AOloopControl_DM_setMAXVOLT(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
	return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_setDClevel_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0) {
        AOloopControl_DM_setDClevel(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_setAveMode_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0) {
        AOloopControl_DM_setAveMode(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;}    else        return 1;}
        
int_fast8_t AOloopControl_DM_setTrigMode_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0) {
        AOloopControl_DM_setTrigMode(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_setTrigChan_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0) {
        AOloopControl_DM_setTrigChan(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_setTrigSem_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0) {
        AOloopControl_DM_setTrigSem(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;}    else        return 1;}





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int_fast8_t AOloopControl_DM_dmturb_cli(){
    if(CLI_checkarg(1,2)==0){
        AOloopControl_DM_dmturb(data.cmdargtoken[1].val.numl, 0, "NULL", 0);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturb2im_cli(){
    if(CLI_checkarg(1,2)+CLI_checkarg(3,2)+CLI_checkarg(3,2)==0){
        AOloopControl_DM_dmturb(data.cmdargtoken[1].val.numl, 1, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturboff_cli(){
    if(CLI_checkarg(1,2)==0){
        AOloopControl_DM_dmturboff(data.cmdargtoken[1].val.numl);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturb_wspeed_cli(){
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0){
        AOloopControl_DM_dmturb_wspeed(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturb_ampl_cli(){
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0){
        AOloopControl_DM_dmturb_ampl(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturb_LOcoeff_cli(){
    if(CLI_checkarg(1,2)+CLI_checkarg(2,1)==0){
        AOloopControl_DM_dmturb_LOcoeff(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numf);
        return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmturb_tint_cli(){
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)==0){
        AOloopControl_DM_dmturb_tint(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl);
        return 0;}    else        return 1;}



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 5. MISC TESTS                                                                                   */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int_fast8_t AOloopControl_mkDM_TT_circle_cli(){
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)==0){
        AOloopControl_mkDM_TT_circle(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf);
        return 0;}    else        return 1;}

 int_fast8_t AOloopControl_DM_mkAstroGrid_seq_cli(){
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,2)+CLI_checkarg(5,2)==0){
        AOloopControl_DM_mkAstroGrid_seq(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl);
        return 0;}    else        return 1;}














void __attribute__ ((constructor)) libinit_AOloopControl_DM()
{
	if ( INITSTATUS_AOloopControl_DM == 0)
	{
		init_AOloopControl_DM();
		RegisterModule(__FILE__, "cacao", "AO loop Control DM operation");
		INITSTATUS_AOloopControl_DM = 1;
	}
}


int init_AOloopControl_DM()
{

/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 1. INITIALIZATION, LOAD/CREATE                                                                  */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


    strcpy(data.cmd[data.NBcmd].key,"aolcontrolDMcomb");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_CombineChannels_cli;
    strcpy(data.cmd[data.NBcmd].info,"create and combine DM channels");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <xsize> <ysize> <NBchannel> <AveMode (1=if average level removed)> <dm2dm mode> <DMmodes> <outdm stream> <wfsref mode> <WFS resp mat> <wfsref stream> <voltmode (1=dmvolt computed)> <dmvolttype> <dmstroke100> <dmvoltname> <DClevel> <maxvolt [V]>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontrolDMcomb 0 50 50 8 0 1 dmmodes outdm 1 wfsrm wfsrefout 1 1 1.0 dmvolt 0.78 120.0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_CombineChannels(long DMindex, long xsize, long ysize, int NBchannel, int AveMode, int dm2dm_mode, const char *dm2dm_DMmodes, const char *dm2dm_outdisp, int wfsrefmode, const char *wfsref_WFSRespMat, const char *wfsref_out, int voltmode, int volttype, float stroke100,const char *IDvolt_name, float DClevel, float maxvolt)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmcomboff");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp =  AOloopControl_DM_dmdispcomboff_cli;
    strcpy(data.cmd[data.NBcmd].info,"turn off DM combine");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmcomboff 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmdispcomboff(long DMindex)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmtrigoff");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp =  AOloopControl_DM_dmtrigoff_cli;
    strcpy(data.cmd[data.NBcmd].info,"turn off DM trigger");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmtrigoff 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmtrigoff(long DMindex)");
    data.NBcmd++;
    


/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 3. CONFIGURATION                                                                                */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmcombmon");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp =  AOloopControl_DM_dmdispcombstatus_cli;
    strcpy(data.cmd[data.NBcmd].info,"monitor DM comb program");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmcombmon 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmdispcombstatus(long DMindex)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aolcontroldmchgain");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_chan_setgain_cli;
    strcpy(data.cmd[data.NBcmd].info,"set gain for DM displacement channel");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <chan#> <gain>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmchgain 0 3 0.2");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_chan_setgain(long DMindex, int ch, float gain)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoldmvoltON");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setvoltON_cli;
    strcpy(data.cmd[data.NBcmd].info,"turn on DM voltage");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoldmvoltON 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setvoltON(long DMindex)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoldmvoltOFF");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setvoltOFF_cli;
    strcpy(data.cmd[data.NBcmd].info,"turn off DM voltage");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoldmvoltOFF 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setvoltOFF(long DMindex)");
    data.NBcmd++;
 
    strcpy(data.cmd[data.NBcmd].key,"aolsetdmvoltmax");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setMAXVOLT_cli;
    strcpy(data.cmd[data.NBcmd].info,"set maximum DM voltage");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <max voltage [V]>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmvoltmax 120.0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setMAXVOLT(long DMindex, float maxvolt)");
    data.NBcmd++;

	strcpy(data.cmd[data.NBcmd].key,"aolsetdmDC");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setDClevel_cli;
    strcpy(data.cmd[data.NBcmd].info,"set DM DC level [um]");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <DC level [um]>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmDC 0.5");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setDClevel(long DMindex, float DClevel)");
    data.NBcmd++;

	strcpy(data.cmd[data.NBcmd].key,"aolsetdmAveM");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setAveMode_cli;
    strcpy(data.cmd[data.NBcmd].info,"set DM averaging mode [0,1 or 2]");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <AveMode>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmAveM 00 1");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setAveMode(long DMindex, int AveMode)");
    data.NBcmd++;

	strcpy(data.cmd[data.NBcmd].key,"aolsetdmTrigMode");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setTrigMode_cli;
    strcpy(data.cmd[data.NBcmd].info,"set DM trigger mode (0:std, 1:use single channel)");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <TrigMode [0, 1]>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmTrigMode 0 1");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setTrigMode(long DMindex, int mode)");
    data.NBcmd++;

	strcpy(data.cmd[data.NBcmd].key,"aolsetdmTrigChan");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setTrigChan_cli;
    strcpy(data.cmd[data.NBcmd].info,"set DM trigger channel");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <TrigChan>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmTrigChan 0 3");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setTrigChan(long DMindex, int chan)");
    data.NBcmd++;

	strcpy(data.cmd[data.NBcmd].key,"aolsetdmTrigSem");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_setTrigSem_cli;
    strcpy(data.cmd[data.NBcmd].info,"set DM trigger semaphore");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <TrigSem [0-9]>");
    strcpy(data.cmd[data.NBcmd].example,"aolsetdmTrigSem 0 4");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_setTrigSem(long DMindex, int sem)");
    data.NBcmd++;




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturbprint");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp =  AOloopControl_printDMturbconf;
    strcpy(data.cmd[data.NBcmd].info,"print DM turb configuration");
    strcpy(data.cmd[data.NBcmd].syntax,"no arg");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturbprint");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_printDMturbconf()");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturb");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb_cli;
    strcpy(data.cmd[data.NBcmd].info,"DM turbulence");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturb 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturb2im");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb2im_cli;
    strcpy(data.cmd[data.NBcmd].info,"DM turbulence to image");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (00-09) <imoutname> <NBsamples>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturb2im 00 wftout 100000");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturboff");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp =  AOloopControl_DM_dmturboff_cli;
    strcpy(data.cmd[data.NBcmd].info,"turn off DM turbulence");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturboff 0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturboff(long DMindex)");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturws");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb_wspeed_cli;
    strcpy(data.cmd[data.NBcmd].info,"set turbulence wind speed");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <wind speed [m/s]>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturws 0 5.2");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb_wspeed(long DMindex, double wspeed);");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturampl");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb_ampl_cli;
    strcpy(data.cmd[data.NBcmd].info,"set turbulence amplitude");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <amplitude [um]>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturampl 0 0.1");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb_ampl(long DMindex, double ampl);");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturlo");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb_LOcoeff_cli;
    strcpy(data.cmd[data.NBcmd].info,"set turbulence low order coefficient");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <coeff>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturlo 0 0.2");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb_LOcoeff(long DMindex, double LOcoeff);");
    data.NBcmd++;

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmturtint");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_dmturb_tint_cli;
    strcpy(data.cmd[data.NBcmd].info,"set turbulence interval time");
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <interval time [us] long>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmturtint 0 200");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_dmturb_tint(long DMindex, long tint);");
    data.NBcmd++;


/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 5. MISC TESTS & UTILS                                                                           */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmmkttcirc");
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_mkDM_TT_circle_cli;
    strcpy(data.cmd[data.NBcmd].info,"make DM TT circle file");
    strcpy(data.cmd[data.NBcmd].syntax,"<outfname> <DMindex (0-9)> <NBpt> <ampl>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmmkttcirc ttcirc 0 20 0.5");
    strcpy(data.cmd[data.NBcmd].Ccall,"long AOloopControl_mkDM_TT_circle(char *IDoutname, long DMindex, long NBpts, float ampl)");
    data.NBcmd++;


    strcpy(data.cmd[data.NBcmd].key,"aoloopcontroldmastrogseq");
    strcpy(data.cmd[data.NBcmd].module, __FILE__);
    data.cmd[data.NBcmd].fp = AOloopControl_DM_mkAstroGrid_seq_cli;
    strcpy(data.cmd[data.NBcmd].info,"make astrogrid sequence");
    strcpy(data.cmd[data.NBcmd].syntax,"<outfname> <DMindex (0-9)> <mode (0-6)> <bin(>1)> <NBcycle>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontroldmastrogseq astrogridseq 0 0 1 1");
    strcpy(data.cmd[data.NBcmd].Ccall,"long AOloopControl_DM_mkAstroGrid_seq(char *IDoutname, long DMindex, int XYmode, int bin, long NBcycle)");
    data.NBcmd++;



    // add atexit functions here
    atexit((void*) AOloopControl_DM_unloadconf);

    return 0;
}

