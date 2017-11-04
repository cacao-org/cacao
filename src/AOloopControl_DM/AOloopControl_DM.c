/**
 * @file    AOloopControl_DM.c
 * @brief   DM control
 * 
 * To be used for AOloopControl module
 *  
 * @author  O. Guyon
 * @date    10 Jul 2017
 *
 *
 * 
 */



#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <err.h>
#include <fcntl.h>
#include <sched.h>
#include <ncurses.h>
#include <semaphore.h>

#include <fitsio.h>

#include "CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "fft/fft.h"
#include "info/info.h"
#include "statistic/statistic.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t){
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time;
    time = mach_absolute_time();
    double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
    double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
    t->tv_sec = seconds;
    t->tv_nsec = nseconds;
    return 0;
}
#else
#include <time.h>
#endif



extern DATA data;



int wcol, wrow; // window size


struct timespec semwaitts;





#define DMSTROKE100 0.7 // um displacement for 100V

long NB_DMindex = 9;

AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
int dmdispcomb_loaded = 0;
int SMfd;


AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
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



int_fast8_t AOloopControl_DM_CombineChannels_cli()
{
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
    // 13 char *IDvolt_name       
    // 14 float maxvolt
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,2)+CLI_checkarg(5,2)+CLI_checkarg(6,2)+CLI_checkarg(7,5)+CLI_checkarg(8,5)+CLI_checkarg(9,2)+CLI_checkarg(10,5)+CLI_checkarg(11,5)+CLI_checkarg(12,2)+CLI_checkarg(13,5)+CLI_checkarg(14,1)+CLI_checkarg(15,1)==0)
        AOloopControl_DM_CombineChannels(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numl, data.cmdargtoken[7].val.string, data.cmdargtoken[8].val.string, data.cmdargtoken[9].val.numl, data.cmdargtoken[10].val.string, data.cmdargtoken[11].val.string, data.cmdargtoken[12].val.numl, data.cmdargtoken[13].val.string, data.cmdargtoken[14].val.numf, data.cmdargtoken[15].val.numf);
    else
        {// DEFAULT: no dm2dm, no wfsref, dmvolt output
            AOloopControl_DM_CombineChannels(0, 50, 50, 8, 1, 0, "dmmodes", "outdisp", 0, "wfsrm", "refout", 1, "dmvolt", 0.0, 150.0);
        }
        
    return 1;
}

int_fast8_t AOloopControl_DM_dmdispcomboff_cli(){
        if(CLI_checkarg(1,2)==0){
        AOloopControl_DM_dmdispcomboff(data.cmdargtoken[1].val.numl);
	return 0;}    else        return 1;}

int_fast8_t AOloopControl_DM_dmtrigoff_cli(){
    if(CLI_checkarg(1,2)==0){
        AOloopControl_DM_dmtrigoff(data.cmdargtoken[1].val.numl);
        return 0;}    else        return 1;}



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
	init_AOloopControl_DM();
	printf(" ...... Loading module %s\n", __FILE__);
}


int init_AOloopControl_DM()
{
    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "AO loop Control DM operation");
    data.NBmodule++;


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
    strcpy(data.cmd[data.NBcmd].syntax,"<DMindex (0-9)> <xsize> <ysize> <NBchannel> <AveMode (1=if average level removed)> <dm2dm mode> <DMmodes> <outdm stream> <wfsref mode> <WFS resp mat> <wfsref stream> <voltmode (1=dmvolt computed)> <dmvoltname> <DClevel> <maxvolt [V]>");
    strcpy(data.cmd[data.NBcmd].example,"aoloopcontrolDMcomb 0 50 50 8 0 1 dmmodes outdm 1 wfsrm wfsrefout 1 dmvolt 0.78 120.0");
    strcpy(data.cmd[data.NBcmd].Ccall,"int AOloopControl_DM_CombineChannels(long DMindex, long xsize, long ysize, int NBchannel, int AveMode, int dm2dm_mode, const char *dm2dm_DMmodes, const char *dm2dm_outdisp, int wfsrefmode, const char *wfsref_WFSRespMat, const char *wfsref_out, int voltmode, const char *IDvolt_name, float DClevel, float maxvolt)");
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
    strcpy(data.cmd[data.NBcmd].module,__FILE__);
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






/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/*  TOOLBOX                                                                                        */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



static struct timespec time_diff(struct timespec start, struct timespec end)
{
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}




// local copy of function in module AtmosphericTurbulence

//
// innerscale and outerscale in pixel
// von Karman spectrum
//
static int make_master_turbulence_screen_local(const char *ID_name1, const char *ID_name2, long size, float outerscale, float innerscale)
{
    long ID,ii,jj;
    float value,C1,C2;
    long cnt;
    long Dlim = 3;
    long IDv;

    int OUTERSCALE_MODE = 1; // 1 if outer scale
    double OUTERscale_f0;
    double INNERscale_f0;
    double dx, dy, r;
    double rlim = 0.0;
    int RLIMMODE = 0;
    double iscoeff;

    /*  IDv = variable_ID("OUTERSCALE");
      if(IDv!=-1)
        {
          outerscale = data.variable[IDv].value.f;
          printf("Outer scale = %f pix\n", outerscale);
        }
     */

    IDv = variable_ID("RLIM");
    if(IDv!=-1)
    {
        RLIMMODE = 1;
        rlim = data.variable[IDv].value.f;
        printf("R limit = %f pix\n",rlim);
    }

    OUTERscale_f0 = 1.0*size/outerscale; // [1/pix] in F plane
    INNERscale_f0 = (5.92/(2.0*M_PI))*size/innerscale;

    make_rnd("tmppha",size,size,"");
    arith_image_cstmult("tmppha", 2.0*PI,"tmppha1");
    delete_image_ID("tmppha");
    //  make_dist("tmpd",size,size,size/2,size/2);
    ID = create_2Dimage_ID("tmpd",size,size);
    for(ii=0; ii<size; ii++)
        for(jj=0; jj<size; jj++)
        {
            dx = 1.0*ii-size/2;
            dy = 1.0*jj-size/2;

            if(RLIMMODE==1)
            {
                r = sqrt(dx*dx + dy*dy);
                if(r<rlim)
                    data.image[ID].array.F[jj*size+ii] = 0.0;
                else
                    data.image[ID].array.F[jj*size+ii] = sqrt(dx*dx + dy*dy + OUTERscale_f0*OUTERscale_f0);
            }
            else
                data.image[ID].array.F[jj*size+ii] = sqrt(dx*dx + dy*dy + OUTERscale_f0*OUTERscale_f0);
        }
    //  data.image[ID].array.F[size/2*size+size/2+10] = 1.0;

    // period [pix] = size/sqrt(dx*dx+dy*dy)
    // f [1/pix] = sqrt(dx*dx+dy*dy)/size
    // f [1/pix] * size = sqrt(dx*dx+dy*dy)

    make_rnd("tmpg",size,size,"-gauss");
    ID = image_ID("tmpg");
    for(ii=0; ii<size; ii++)
        for(jj=0; jj<size; jj++)
        {
            dx = 1.0*ii-size/2;
            dy = 1.0*jj-size/2;
            iscoeff = exp(-(dx*dx+dy*dy)/INNERscale_f0/INNERscale_f0);
            data.image[ID].array.F[jj*size+ii] *= sqrt(iscoeff); // power -> amplitude : sqrt 
        }

    arith_image_cstpow("tmpd", 11.0/6.0, "tmpd1");
    delete_image_ID("tmpd");
    arith_image_div("tmpg", "tmpd1", "tmpamp");
    delete_image_ID("tmpg");
    delete_image_ID("tmpd1");
    arith_set_pixel("tmpamp", 0.0, size/2, size/2);
    mk_complex_from_amph("tmpamp", "tmppha1", "tmpc", 0);
    delete_image_ID("tmpamp");
    delete_image_ID("tmppha1");
    permut("tmpc");
    do2dfft("tmpc","tmpcf");
    delete_image_ID("tmpc");
    mk_reim_from_complex("tmpcf", "tmpo1", "tmpo2", 0);
    delete_image_ID("tmpcf");

    /* compute the scaling factor in the power law of the structure function */
    fft_structure_function("tmpo1", "strf");
    ID = image_ID("strf");
    value = 0.0;
    cnt = 0;
    for(ii = 1; ii<Dlim; ii++)
        for(jj = 1; jj<Dlim; jj++)
        {
            value += log10(data.image[ID].array.F[jj*size+ii])-5.0/3.0*log10(sqrt(ii*ii+jj*jj));
            cnt++;
        }
    // save_fl_fits("strf","!strf.fits");
    delete_image_ID("strf");
    C1 = pow(10.0,value/cnt);

    fft_structure_function("tmpo2", "strf");
    ID=image_ID("strf");
    value = 0.0;
    cnt = 0;
    for(ii=1; ii<Dlim; ii++)
        for(jj=1; jj<Dlim; jj++)
        {
            value += log10(data.image[ID].array.F[jj*size+ii])-5.0/3.0*log10(sqrt(ii*ii+jj*jj));
            cnt++;
        }
    delete_image_ID("strf");
    C2 = pow(10.0,value/cnt);

    printf("%f %f\n", C1, C2);

    arith_image_cstmult("tmpo1",1.0/sqrt(C1),ID_name1);
    arith_image_cstmult("tmpo2",1.0/sqrt(C2),ID_name2);
    delete_image_ID("tmpo1");
    delete_image_ID("tmpo2");

    return(0);
}












/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 1. INITIALIZATION, LOAD/CREATE                                                                  */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



static int AOloopControl_DM_createconf()
{
    int result;
    int ch;
    char fname[200];
    long DMindex;
    char errstr[200];

    sprintf(fname, "/tmp/dmdispcombconf.conf.shm");

    if( dmdispcomb_loaded == 0 )
    {
        printf("Create/read DM configuration, %ld entries\n", NB_DMindex);

        SMfd = open(fname, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
        if (SMfd == -1) {
            sprintf(errstr, "Error opening (O_RDWR | O_CREAT | O_TRUNC) file \"%s\", function AOloopControl_DM_createconf", fname);
            perror(errstr);
            exit(EXIT_FAILURE);
        }

        result = lseek(SMfd, sizeof(AOLOOPCONTROL_DM_DISPCOMB_CONF)*NB_DMindex-1, SEEK_SET);
        if (result == -1) {
            close(SMfd);
            perror("Error calling lseek() to 'stretch' the file");
            exit(EXIT_FAILURE);
        }

        result = write(SMfd, "", 1);
        if (result != 1) {
            close(SMfd);
            perror("Error writing last byte of the file");
            exit(EXIT_FAILURE);
        }

        dmdispcombconf = (AOLOOPCONTROL_DM_DISPCOMB_CONF*)mmap(0, sizeof(AOLOOPCONTROL_DM_DISPCOMB_CONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMfd, 0);
        if (dmdispcombconf == MAP_FAILED) {
            close(SMfd);
            perror("Error mmapping the file");
            exit(EXIT_FAILURE);
        }

        for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
            dmdispcombconf[DMindex].ON = 0;
            dmdispcombconf[DMindex].xsize = 0;
            dmdispcombconf[DMindex].ysize = 0;
            dmdispcombconf[DMindex].xysize = 0;
            dmdispcombconf[DMindex].NBchannel = 0;
            dmdispcombconf[DMindex].busy = 0;
            dmdispcombconf[DMindex].MAXVOLT = 150.0;
            dmdispcombconf[DMindex].moninterval = 30000; // 33Hz
            dmdispcombconf[DMindex].status = 0;
			dmdispcombconf[DMindex].nsecwait = 1000; // 3 us
			
			dmdispcombconf[DMindex].TrigMode = 0;
			dmdispcombconf[DMindex].TrigChan = 0;
			dmdispcombconf[DMindex].TrigSem = 0;


            dmdispcombconf[DMindex].IDdisp = -1;
            dmdispcombconf[DMindex].IDvolt = -1;
            sprintf(dmdispcombconf[DMindex].voltname, " ");

            for(ch=0; ch<DM_NUMBER_CHANMAX; ch++)
                {
                    dmdispcombconf[DMindex].dmdispID[ch] = -1;
                    dmdispcombconf[DMindex].dmdispgain[ch] = 1.0;
                    dmdispcombconf[DMindex].dmdispcnt[ch] = 0;
                }
        }
        dmdispcomb_loaded = 1;

    }
    AOloopControl_printDMconf();
    
    return 0;
}




static int AOloopControl_DM_loadconf()
{
    int result;
    char fname[200];
    char errstr[200];

    sprintf(fname, "/tmp/dmdispcombconf.conf.shm");



    if( dmdispcomb_loaded == 0 )
    {
        printf("Create/read DM configuration\n");

        SMfd = open(fname, O_RDWR, (mode_t)0600);
        if (SMfd == -1) {
            AOloopControl_DM_createconf();
        }
        else
        {
            dmdispcombconf = (AOLOOPCONTROL_DM_DISPCOMB_CONF*)mmap(0, sizeof(AOLOOPCONTROL_DM_DISPCOMB_CONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMfd, 0);
            if (dmdispcombconf == MAP_FAILED) {
                close(SMfd);
                printf("Error mmapping the file -> creating it\n");
                AOloopControl_DM_createconf();
                //            exit(EXIT_FAILURE);
            }
        }

        dmdispcomb_loaded = 1;
    }
    AOloopControl_printDMconf();

    return 0;
}





static int AOloopControl_DM_unloadconf()
{
    if( dmdispcomb_loaded == 1 )
    {
        if (munmap(dmdispcombconf, sizeof(AOLOOPCONTROL_DM_DISPCOMB_CONF)*NB_DMindex) == -1)
            perror("Error un-mmapping the file");
        close(SMfd);
        dmdispcomb_loaded = 0;
    }
    return 0;
}






/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

int AOloopControl_DM_disp2V(long DMindex)
{
    long ii;
    float volt;


	data.image[dmdispcombconf[DMindex].IDvolt].md[0].write = 1;
		
	if(dmdispcombconf[DMindex].voltON==1)
		{
			for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
				{
					volt = 100.0*sqrt(data.image[dmdispcombconf[DMindex].IDdisp].array.F[ii]/DMSTROKE100);
					if(volt>dmdispcombconf[DMindex].MAXVOLT)
						volt = dmdispcombconf[DMindex].MAXVOLT;
					data.image[dmdispcombconf[DMindex].IDvolt].array.UI16[ii] = (unsigned short int) (volt/300.0*16384.0); //65536.0);
				}
		}
	else
		for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
			data.image[dmdispcombconf[DMindex].IDvolt].array.UI16[ii] = 0;
			

	data.image[dmdispcombconf[DMindex].IDvolt].md[0].write = 0;
	data.image[dmdispcombconf[DMindex].IDvolt].md[0].cnt0++;
    
    
//    COREMOD_MEMORY_image_set_sempost(data.image[dmdispcombconf[DMindex].IDdisp].name, -1);
	COREMOD_MEMORY_image_set_sempost_byID(dmdispcombconf[DMindex].IDdisp, -1);

    return 0;
}




//
// DMindex is a unique DM identifier (0-9), so multiple instances can coexist
//
// xsize, ysize is DM pixel size
//
// NBchannel : number of channels. All channels co-added
//
// AveMode: averaging mode 
//      0: do not appy DC offset command to average, but offset combined average to mid-range, and clip displacement at >0.0
//      1: apply DC offset to remove average
//      2: do not apply DC offset, do not offset sum, do not clip
//
// NOTE: DM displacement is biased to mid displacement
// NOTE: responds immediately to sem[1] in dmdisp
// dmdisp files have 10 semaphores
//
// dm2dm_mode: 1 if this DM controls an output DM
// 
// dm2dm_DMmodes: data cube containting linear relationship between current DM and the output DM it controls
//
// dm2dm_outdisp: data stream to which output DM is written
//
// wfsrefmode: 1 if offset to WFS reference
//
// wfsref_WFSRespMat: response matrix of output loop
//
// wfsref_out : output wfsreference
//
// voltmode = 1 if DM volt computed
// 
// IDvolt_name : name of DM volt stream
//
// maxvolt: maximum volt for DM volt
// 


int AOloopControl_DM_CombineChannels(long DMindex, long xsize, long ysize, int NBchannel, int AveMode, int dm2dm_mode, const char *dm2dm_DMmodes, const char *dm2dm_outdisp, int wfsrefmode, const char *wfsref_WFSRespMat, const char *wfsref_out, int voltmode, const char *IDvolt_name, float DClevel, float maxvolt)
{
    long naxis = 2;
    uint32_t *size;
    long ch;
    char name[200];
    long cnt = 0;
    long long cntold;
    long long cntsumold;
    long long cntsum;
    long ii;
    long IDdisp;
    long IDvolt;
    double ave;
    long ID1;
    int RT_priority = 95; //any number from 0-99
    struct sched_param schedpar;
    int r;
    long sizexy;
    float *dmdispptr;
    float *dmdispptr_array[20];
    long IDdispt;
    char sname[200];

    int vOK;
    float maxmaxvolt = 150.0;
    char errstr[200];
    int semnb, semval;
    long sizexyDMout;
    long IDtmpoutdm;
    long kk;
    long sizexywfsref;
    long IDtmpoutref;
    long cntch;
    
	long IDvar;
    long DMtwaitus = 0; // optional time interval between successive commands [us]
    // if 0, do not wait
    // read from variable name DMTWAIT
    
    
    // timing
    struct timespec ttrig;
    struct timespec t1;
    struct timespec tnow;
	struct timespec tdiff;
	double tdiffv;
    
    int DMupdate;
    
    
    
    if(DMindex>NB_DMindex-1)
    {
        printf("ERROR: requested DMindex (%02ld) exceeds maximum number of DMs (%02ld)\n", DMindex, NB_DMindex);
        exit(0);
    }
    
    
    printf("Setting up DM #%ld\n", DMindex); 
    
    list_variable_ID();
    IDvar = variable_ID("DMTWAIT");
    if(IDvar!=-1)
		DMtwaitus = (long) (data.variable[IDvar].value.f);
    
    
    
    schedpar.sched_priority = RT_priority;
    #ifndef __MACH__
    r = seteuid(data.euid); // This goes up to maximum privileges
    sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
    r = seteuid(data.ruid); //Go back to normal privileges
	#endif

   // AOloopControl_DM_createconf();
    
    AOloopControl_DM_loadconf();
    
    dmdispcombconf[DMindex].ON = 1;
    dmdispcombconf[DMindex].xsize = xsize;
    dmdispcombconf[DMindex].ysize = ysize;
    dmdispcombconf[DMindex].xysize = xsize*ysize;
    dmdispcombconf[DMindex].NBchannel = NBchannel;
    dmdispcombconf[DMindex].voltmode = voltmode;
    dmdispcombconf[DMindex].voltON = 1;
    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    dmdispcombconf[DMindex].AveMode = AveMode;
    sprintf(dmdispcombconf[DMindex].voltname, "%s", IDvolt_name);
    dmdispcombconf[DMindex].status = 0;
    
    dmdispcombconf[DMindex].DClevel = DClevel; //0.5*(DMSTROKE100*dmdispcombconf[DMindex].MAXVOLT/100.0*dmdispcombconf[DMindex].MAXVOLT/100.0);

    printf("maxvolt = %f\n", maxvolt);


    size = (uint32_t*) malloc(sizeof(uint32_t)*naxis);
    size[0] = xsize;
    size[1] = ysize;
    sizexy = xsize*ysize;


    dmdispcombconf[DMindex].xsizeout = 0;
    dmdispcombconf[DMindex].ysizeout = 0;

    dmdispcombconf[DMindex].dm2dm_mode = dm2dm_mode;
    

   if(dm2dm_mode == 1) 
   {
        printf("INITIALIZATION AND VERIFICATION FOR dm2dm MODE ...\n");
        fflush(stdout);

        dmdispcombconf[DMindex].ID_dm2dm_DMmodes = image_ID(dm2dm_DMmodes);
        sprintf(dmdispcombconf[DMindex].dm2dm_DMmodes_name, "%s", dm2dm_DMmodes);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].naxis != 3)
            {
                sprintf(errstr, "image \"%s\" should have naxis = 3", dm2dm_DMmodes);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
        dmdispcombconf[DMindex].xsizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[0];
        dmdispcombconf[DMindex].ysizeout = data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].md[0].size[1];        
   
        dmdispcombconf[DMindex].ID_dm2dm_outdisp = image_ID(dm2dm_outdisp);
        if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[0] != dmdispcombconf[DMindex].xsizeout)
            {
                sprintf(errstr, "image \"%s\" should have x axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].xsizeout);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
         if(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].size[1] != dmdispcombconf[DMindex].ysizeout)
            {
                sprintf(errstr, "image \"%s\" should have y axis = %ld", dm2dm_outdisp, dmdispcombconf[DMindex].ysizeout);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
            
        IDtmpoutdm = create_2Dimage_ID("_tmpoutdm", dmdispcombconf[DMindex].xsizeout, dmdispcombconf[DMindex].ysizeout); 
        sizexyDMout = dmdispcombconf[DMindex].xsizeout*dmdispcombconf[DMindex].ysizeout;
        printf("done\n\n");
        fflush(stdout);
   }
   
   
   
	list_image_ID(); //TEST
	
    dmdispcombconf[DMindex].wfsrefmode = wfsrefmode;
    if(wfsrefmode == 1) 
    {
        printf("INITIALIZATION AND VERIFICATION FOR wfsref MODE ...\n");
        fflush(stdout);
    
		printf("wfsref_WFSRespMat = %s\n", wfsref_WFSRespMat);
		fflush(stdout);
		
        dmdispcombconf[DMindex].ID_wfsref_RespMat = image_ID(wfsref_WFSRespMat);
        if(dmdispcombconf[DMindex].ID_wfsref_RespMat==-1)
			{
				printf("ERROR: cannot find image \"%s\"\n", wfsref_WFSRespMat);
				exit(0);
			}
        
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].naxis != 3)
            {
                sprintf(errstr, "image \"%s\" should have naxis = 3", wfsref_WFSRespMat);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
        dmdispcombconf[DMindex].xsizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[0];
        dmdispcombconf[DMindex].ysizewfsref = data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].md[0].size[1];
		
		printf("xsizewfsref = %ld\n", dmdispcombconf[DMindex].xsizewfsref);
		printf("ysizewfsref = %ld\n", dmdispcombconf[DMindex].ysizewfsref);
		fflush(stdout);

        dmdispcombconf[DMindex].ID_wfsref_out = image_ID(wfsref_out);
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[0] != dmdispcombconf[DMindex].xsizewfsref)
            {
                sprintf(errstr, "image \"%s\" should have x axis = %ld", wfsref_out, dmdispcombconf[DMindex].xsizewfsref);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
        if(data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].size[1] != dmdispcombconf[DMindex].ysizewfsref)
            {
                sprintf(errstr, "image \"%s\" should have y axis = %ld", wfsref_out, dmdispcombconf[DMindex].ysizewfsref);
                printERROR(__FILE__,__func__,__LINE__, errstr);
                exit(0);
            }
        printf("Creating image %s   %ld x %ld\n", "_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        fflush(stdout);
        IDtmpoutref = create_2Dimage_ID("_tmpoutref", dmdispcombconf[DMindex].xsizewfsref, dmdispcombconf[DMindex].ysizewfsref);
        sizexywfsref = dmdispcombconf[DMindex].xsizewfsref*dmdispcombconf[DMindex].ysizewfsref;
		
       COREMOD_MEMORY_image_set_createsem(wfsref_out, 10);

        printf("done\n\n");
        fflush(stdout);
    }

    printf("Initialize channels\n");
    printf("Max DM stroke = %f um\n", DMSTROKE100*dmdispcombconf[0].MAXVOLT/100.0*dmdispcombconf[0].MAXVOLT/100.0);
    fflush(stdout);

    for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
    {
        sprintf(name, "dm%02lddisp%02ld", DMindex, ch);
        printf("Channel %ld \n", ch);
        dmdispcombconf[DMindex].dmdispID[ch] = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
        COREMOD_MEMORY_image_set_createsem(name, 10);
        dmdispptr_array[ch] = data.image[dmdispcombconf[DMindex].dmdispID[ch]].array.F;
    }


    sprintf(name, "dm%02lddisp", DMindex);
    dmdispcombconf[DMindex].IDdisp = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 1, 10);
    COREMOD_MEMORY_image_set_createsem(name, 10);
    
    sprintf(name, "dm%02lddispt", DMindex);
    IDdispt = create_image_ID(name, naxis, size, _DATATYPE_FLOAT, 0, 0);
    dmdispptr = data.image[IDdispt].array.F;

    if(dmdispcombconf[DMindex].voltmode==1)
    {
        IDvolt = image_ID(dmdispcombconf[DMindex].voltname);
        
        vOK = 0;
        if(IDvolt!=-1)
            {
                if((data.image[IDvolt].md[0].atype==_DATATYPE_UINT16)&&(data.image[IDvolt].md[0].naxis==2)&&(data.image[IDvolt].md[0].size[0]==xsize)&&(data.image[IDvolt].md[0].size[1]==ysize))
                        vOK = 1;
                else
                    delete_image_ID(dmdispcombconf[DMindex].voltname);
            }
        
        printf("vOK = %d\n", vOK);
        if(vOK==0)
        {
            dmdispcombconf[DMindex].IDvolt = create_image_ID(dmdispcombconf[DMindex].voltname, naxis, size, _DATATYPE_UINT16, 1, 10);
            COREMOD_MEMORY_image_set_createsem(dmdispcombconf[DMindex].voltname, 10);
         }
         else
            dmdispcombconf[DMindex].IDvolt = image_ID(dmdispcombconf[DMindex].voltname);
    }

    cntsumold = 0;

    
    dmdispcombconf[0].status = 1;

    sprintf(name, "dm%02lddisp", DMindex);
    COREMOD_MEMORY_image_set_createsem(name, 10);

    if(data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem<2)
    {
        printf("ERROR: image %s semaphore %d missing\n", data.image[dmdispcombconf[DMindex].IDdisp].name, 1);
        exit(0);
    }

    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    if(dmdispcombconf[DMindex].MAXVOLT>maxmaxvolt)
        dmdispcombconf[DMindex].MAXVOLT = maxvolt;
    
    
     AOloopControl_printDMconf();

    if (sigaction(SIGINT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGTERM, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGBUS, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGSEGV, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGABRT, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGHUP, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGPIPE, &data.sigact, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    
    
    
    list_image_ID();
    
    
    while(dmdispcombconf[DMindex].ON == 1)
    {
		
        dmdispcombconf[DMindex].status = 2;

		if(DMtwaitus>0)
			usleep(DMtwaitus);

		if (clock_gettime(CLOCK_REALTIME, &semwaitts) == -1) {
			perror("clock_gettime");
			exit(EXIT_FAILURE);
		}
		semwaitts.tv_nsec += dmdispcombconf[DMindex].nsecwait;
		if(semwaitts.tv_nsec >= 1000000000)
			semwaitts.tv_sec = semwaitts.tv_sec + 1;


		DMupdate = 0;
		
		if(dmdispcombconf[DMindex].TrigMode==0)
		{
			//
			// this is semaphore that triggers the write to the DM
			// 
			sem_timedwait(data.image[dmdispcombconf[DMindex].IDdisp].semptr[1], &semwaitts);

			cntsum = 0;
			for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
			{
				cntch = data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
				dmdispcombconf[DMindex].dmdispcnt[ch] = cntch;
				cntsum += data.image[dmdispcombconf[DMindex].dmdispID[ch]].md[0].cnt0;
			}
			if(cntsum != cntsumold)
				DMupdate = 1;
        }
        else
		{
			sem_timedwait(data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].semptr[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigSem]], &semwaitts);
			cnt = data.image[dmdispcombconf[DMindex].dmdispID[dmdispcombconf[DMindex].TrigChan]].md[0].cnt0;
			if(cnt!=cntold)
				{
					DMupdate = 1;
					cntold=cnt;
				}
        }
        
            
        if(DMupdate==1)
        {
			clock_gettime(CLOCK_REALTIME, &ttrig);
			
            dmdispcombconf[0].status = 3;
            cnt++;

            memcpy (data.image[IDdispt].array.F, dmdispptr_array[0], sizeof(float)*sizexy);
            for(ch=1; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            {
                for(ii=0; ii<sizexy; ii++)
                    dmdispptr[ii] += dmdispcombconf[DMindex].dmdispgain[ch]*dmdispptr_array[ch][ii];
            }

            dmdispcombconf[DMindex].status = 4;

            ave = 0.0;
            if(dmdispcombconf[DMindex].AveMode == 1) // REMOVE AVERAGE 
                {
                    for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                        ave += data.image[IDdispt].array.F[ii];
                    ave /= dmdispcombconf[DMindex].xysize;
                }
            dmdispcombconf[DMindex].status = 5;

            if(dmdispcombconf[DMindex].AveMode < 2) // OFFSET BY DClevel
            {
                for(ii=0; ii<dmdispcombconf[DMindex].xysize; ii++)
                {
                    data.image[IDdispt].array.F[ii] += (dmdispcombconf[DMindex].DClevel - ave);
                
					// remove negative values
					if(dmdispcombconf[DMindex].voltmode==1)
						if(data.image[IDdispt].array.F[ii]<0.0)
							data.image[IDdispt].array.F[ii] = 0.0;
                }
            }
            dmdispcombconf[DMindex].status = 6;

            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 1;
            memcpy (data.image[dmdispcombconf[DMindex].IDdisp].array.F,data.image[IDdispt].array.F, sizeof(float)*data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].cnt0++;
            data.image[dmdispcombconf[DMindex].IDdisp].md[0].write = 0;            
 
       /*     for(semnb=0;semnb<data.image[dmdispcombconf[DMindex].IDdisp].md[0].sem;semnb++)
               {
                   sem_getvalue(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb], &semval);
                   if(semval<SEMAPHORE_MAXVAL)
                   sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[semnb]);
                }*/
            COREMOD_MEMORY_image_set_sempost_byID(dmdispcombconf[DMindex].IDdisp, -1);      
                   //      sem_post(data.image[dmdispcombconf[DMindex].IDdisp].semptr[0]);
 
 
 
            if(dm2dm_mode==1)
            {
                memset(data.image[IDtmpoutdm].array.F, '\0', sizeof(float)*sizexyDMout);
                for(kk=0;kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement;kk++)
                    {
                        for(ii=0;ii<sizexyDMout;ii++)
                            data.image[IDtmpoutdm].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_dm2dm_DMmodes].array.F[kk*sizexyDMout+ii];
                    }
                
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].array.F,data.image[IDtmpoutdm].array.F, sizeof(float)*sizexyDMout);
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].md[0].write = 0;            
                sem_post(data.image[dmdispcombconf[DMindex].ID_dm2dm_outdisp].semptr[0]);                
            }
            
            if(wfsrefmode==1)
            {
                memset(data.image[IDtmpoutref].array.F, '\0', sizeof(float)*sizexywfsref);
                list_image_ID();
                printf("kkmax = %ld\n", data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement);
                printf("iimax = %ld\n", sizexywfsref);
                printf("ID RespMat = %ld  (%ld)\n", dmdispcombconf[DMindex].ID_wfsref_RespMat, (data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement-1)*sizexywfsref + sizexywfsref-1);
                fflush(stdout);
                save_fits(wfsref_WFSRespMat, "!_test_wfsref_WFSRespMat.fits");
                for(kk=0;kk<data.image[dmdispcombconf[DMindex].IDdisp].md[0].nelement;kk++)
                    {
                        printf("(%ld %g) ", kk, data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk]);
                        for(ii=0;ii<sizexywfsref;ii++)
                            data.image[IDtmpoutref].array.F[ii] += data.image[dmdispcombconf[DMindex].IDdisp].array.F[kk] * data.image[dmdispcombconf[DMindex].ID_wfsref_RespMat].array.F[kk*sizexywfsref+ii];
                    }
                printf("\n");
                printf("Updating Zero Point  %ld <- %ld\n", dmdispcombconf[DMindex].ID_wfsref_out, IDtmpoutref);
                fflush(stdout);   
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 1;
                memcpy (data.image[dmdispcombconf[DMindex].ID_wfsref_out].array.F,data.image[IDtmpoutref].array.F, sizeof(float)*sizexywfsref);
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].cnt0++;
                data.image[dmdispcombconf[DMindex].ID_wfsref_out].md[0].write = 0;            
                sem_post(data.image[dmdispcombconf[DMindex].ID_wfsref_out].semptr[0]);   
                printf("Done\n");
                fflush(stdout);
            }
            
            
            dmdispcombconf[DMindex].status = 7;

			
			clock_gettime(CLOCK_REALTIME, &t1);
			if(dmdispcombconf[DMindex].voltmode==1)
				AOloopControl_DM_disp2V(DMindex);
			

            dmdispcombconf[DMindex].status = 8;

            cntsumold = cntsum;
            dmdispcombconf[DMindex].updatecnt++;
            
            clock_gettime(CLOCK_REALTIME, &tnow);
            tdiff = time_diff(ttrig, tnow);
			tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
            dmdispcombconf[DMindex].tdelay = tdiffv;

			tdiff = time_diff(t1, tnow);
			tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
			dmdispcombconf[DMindex].time_disp2V = tdiffv;
        }
    
         if((data.signal_INT == 1)||(data.signal_TERM == 1)||(data.signal_ABRT==1)||(data.signal_BUS==1)||(data.signal_SEGV==1)||(data.signal_HUP==1)||(data.signal_PIPE==1))
            dmdispcombconf[DMindex].ON = 0;
    }

  //  if(voltmode==1)
    //    arith_image_zero(dmdispcombconf[DMindex].voltname);



    printf("LOOP STOPPED\n");
    fflush(stdout);

    free(size);
 

    return 0;
}



int AOloopControl_DM_dmdispcomboff(long DMindex)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].ON = 0;
	AOloopControl_printDMconf();
	
    return 0;
}


int AOloopControl_DM_dmtrigoff(long DMindex)
{
	AOloopControl_DM_loadconf();
	data.image[dmdispcombconf[DMindex].IDvolt].md[0].status = 101;
	AOloopControl_printDMconf();
    
    return 0;
}




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 3. CONFIGURATION                                                                                */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


int AOloopControl_printDMconf()
{
    long DMindex;
    char IDvolt_str[4];
    char maxvolt_str[7];
    char voltname_str[12];
    
    printf("DM | on |  x |  y | Nbch | busy | ave | DClevel | monint  | stat | IDdisp | voltmode | IDvolt | maxvolt |   voltname  |\n");
    for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
			if(dmdispcombconf[DMindex].voltmode==1)
				{
					sprintf(IDvolt_str, "%3ld", dmdispcombconf[DMindex].IDvolt);
					sprintf(maxvolt_str, "%6.2f", dmdispcombconf[DMindex].MAXVOLT);
					sprintf(voltname_str, "%11s", dmdispcombconf[DMindex].voltname);
				}
				else
				{
					sprintf(IDvolt_str, "---");
					sprintf(maxvolt_str, "------");
					sprintf(voltname_str, "-----------");
				}
				
			printf("%02ld |  %1d |%3ld |%3ld |  %02ld  |   %1d  |  %1d  | %6.2f  |%8ld |   %02d |   %3ld  |    %4d  |   %3s  |  %6s | %11s |\n", DMindex, dmdispcombconf[DMindex].ON, dmdispcombconf[DMindex].xsize, dmdispcombconf[DMindex].ysize, dmdispcombconf[DMindex].NBchannel, dmdispcombconf[DMindex].busy, dmdispcombconf[DMindex].AveMode, dmdispcombconf[DMindex].DClevel, dmdispcombconf[DMindex].moninterval, dmdispcombconf[DMindex].status, dmdispcombconf[DMindex].IDdisp, dmdispcombconf[DMindex].voltmode, IDvolt_str, maxvolt_str, voltname_str);
        }
    
    return(0);
}





int AOloopControl_DM_dmdispcombstatus(long DMindex)
{
    long long mcnt = 0;
    int ch;

    AOloopControl_DM_loadconf();

    initscr();
    getmaxyx(stdscr, wrow, wcol);

    start_color();
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_BLACK, COLOR_RED);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    while( !kbdhit() )
    {
        usleep(dmdispcombconf[DMindex].moninterval);
        clear();
        attron(A_BOLD);
        print_header(" PRESS ANY KEY TO STOP MONITOR ", '-');
        attroff(A_BOLD);
        printw("monitor sample %ld\n", mcnt);
        printw("\n"); 

        printw("=========== DM %d ==============================================\n", DMindex);
        printw("\n");
        printw("ON                = %10d\n", dmdispcombconf[DMindex].ON);
        printw("size              = %ld x %ld = %ld\n", dmdispcombconf[DMindex].xsize, dmdispcombconf[DMindex].ysize, dmdispcombconf[DMindex].xysize);
        printw("NBchannel         = %10d      Number of DM channels\n", dmdispcombconf[DMindex].NBchannel);
        printw("\n");

        printw("loopcnt           = %10ld     AO loop index\n", dmdispcombconf[DMindex].loopcnt);
        printw("updatecnt         = %10ld     Number of DM updates\n", dmdispcombconf[DMindex].updatecnt);
        printw("busy              = %10d   \n", dmdispcombconf[DMindex].busy);
        printw("nsecwait          = %10ld ns\n", dmdispcombconf[DMindex].nsecwait);
        printw("delay time        = %10.3f us\n", dmdispcombconf[DMindex].tdelay*1.0e6);
        printw("disp->V time      = %10.3f us\n", dmdispcombconf[DMindex].time_disp2V*1.0e6);

		
		
		if(dmdispcombconf[DMindex].TrigMode==1)
			attron(A_BOLD);
        printw("\n");
        printw("=========== TRIGGER MODE = %d ====================================\n", dmdispcombconf[DMindex].TrigMode); 		
		printw("TrigChan          = %10d      DM trigger channel\n", dmdispcombconf[DMindex].TrigChan);
		printw("TrigSem           = %10d      DM trigger semaphore\n", dmdispcombconf[DMindex].TrigSem);
		printw("================================================================\n");
		if(dmdispcombconf[DMindex].TrigMode==1)
			attroff(A_BOLD);
        printw("\n");     



		if(dmdispcombconf[DMindex].voltmode==1)
			attron(A_BOLD);			
		printw("=========== OUTPUT VOLT ========================================\n");
        printw("voltmode          = %10d      Configured for output voltage ?\n", dmdispcombconf[DMindex].voltmode);
        printw("voltON            = %10d      DM voltage ouptut activated ?\n", dmdispcombconf[DMindex].voltON);
        printw("MAXVOLT           = %10.2f V    Maximum voltage\n", dmdispcombconf[DMindex].MAXVOLT);
        printw("AveMode           = %10d      Averaging mode for combined displacement\n", dmdispcombconf[DMindex].AveMode);
        printw("    0: Offset combined to DC level\n");
        printw("       Clip displacement at 0.0\n");
        printw("    1: Remove average and apply DC offset.\n");
        printw("       Clip at 0.0.\n");
        printw("    2: Do not apply DC offset, do not offset sum, do not clip\n");
        printw("DClevel           = %10.5f um   Displacement DC offset\n", dmdispcombconf[DMindex].DClevel);
		printw("================================================================\n");
		if(dmdispcombconf[DMindex].voltmode==1)
			attroff(A_BOLD);
        printw("\n"); 

        for(ch=0; ch<dmdispcombconf[DMindex].NBchannel; ch++)
            printw(" CHANNEL %2d  gain = %10.3f   dm%02lddisp%02ld   %10ld\n", ch, dmdispcombconf[DMindex].dmdispgain[ch], DMindex, ch, dmdispcombconf[DMindex].dmdispcnt[ch]);
        printw("\n"); 

		if(dmdispcombconf[DMindex].dm2dm_mode==1)
			attron(A_BOLD);
		printw("=========== DM-to-DM OUPUT (CPU-based) ==========================\n");
        printw("dm2dm_mode        = %10d      DM controls an output DM ?\n", dmdispcombconf[DMindex].dm2dm_mode);
        printw("xsizeout          = %10ld      x size of output DM\n", dmdispcombconf[DMindex].xsizeout);
        printw("ysizeout          = %10ld      y size of output DM\n", dmdispcombconf[DMindex].ysizeout);
        printw("dm2dm_DMmodes     = %10s      output DM modes\n", dmdispcombconf[DMindex].dm2dm_DMmodes_name);
        printw("dm2dm_outdisp     = %10s      ouput DM displacement\n", dmdispcombconf[DMindex].dm2dm_outdisp_name);
        printw("================================================================\n");
		if(dmdispcombconf[DMindex].dm2dm_mode==1)
			attroff(A_BOLD);
        printw("\n");

		if(dmdispcombconf[DMindex].dm2dm_mode==1)
			attron(A_BOLD);
		printw("======== DM CONTROL TO OUTPUT WFS REFERENCE (CPU-based) ========\n");
		printw("wfsrefmode        = %10d      DM controls output wfsref ?\n", dmdispcombconf[DMindex].wfsrefmode);
        printw("xsizewfsref       = %10ld      x size of output WFS ref\n", dmdispcombconf[DMindex].xsizeout);
        printw("ysizewfsref       = %10ld      y size of output WFS ref\n", dmdispcombconf[DMindex].ysizeout);
		printw("wfsref_RespMat    = %10s      DM-to-WFSref matrix\n", dmdispcombconf[DMindex].wfsref_RespMat_name);
		printw("wfsref_out        = %10s      output WFSref\n", dmdispcombconf[DMindex].wfsref_out_name);
        printw("================================================================\n");
		if(dmdispcombconf[DMindex].dm2dm_mode==1)
			attroff(A_BOLD);
        printw("\n");
		
        printw("status            = %10d\n",  dmdispcombconf[DMindex].status);
        printw("moninterval       = %10d us\n", dmdispcombconf[DMindex].moninterval);
        printw("\n");
     
        mcnt++;
        refresh();
    }
    endwin();

    return 0;
}





int AOloopControl_DM_chan_setgain(long DMindex, int ch, float gain)
{
    
    AOloopControl_DM_loadconf();
 
    if(ch<dmdispcombconf[DMindex].NBchannel)
        dmdispcombconf[DMindex].dmdispgain[ch] = gain;

    return 0;
}




int AOloopControl_DM_setvoltON(long DMindex)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].voltON = 1;
	AOloopControl_printDMconf();
	
    return 0;
}


int AOloopControl_DM_setvoltOFF(long DMindex)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].voltON = 0;
	AOloopControl_printDMconf();
	
    return 0;
}


int AOloopControl_DM_setMAXVOLT(long DMindex, float maxvolt)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].MAXVOLT = maxvolt;
	AOloopControl_printDMconf();
	
    return 0;
}


int AOloopControl_DM_setDClevel(long DMindex, float DClevel)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].DClevel = DClevel;
	AOloopControl_printDMconf();

    return 0;
}


int AOloopControl_DM_setAveMode(long DMindex, int AveMode)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].AveMode = AveMode;
	AOloopControl_printDMconf();

    return 0;
}


int AOloopControl_DM_setTrigMode(long DMindex, int mode)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].TrigMode = mode;
	AOloopControl_printDMconf();

    return 0;
}

int AOloopControl_DM_setTrigChan(long DMindex, int chan)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].TrigChan = chan;
	AOloopControl_printDMconf();

    return 0;
}

int AOloopControl_DM_setTrigSem(long DMindex, int sem)
{
    AOloopControl_DM_loadconf();
    dmdispcombconf[DMindex].TrigSem = sem;
	AOloopControl_printDMconf();

    return 0;
}











/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */




int_fast8_t AOloopControl_printDMturbconf()
{
    long DMindex;
    
    if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

//    AOloopControl_DMturb_loadconf(0);
    
    printf("ind on  ampl [um]  tint [us]  simtime [s]  wspeed [m/s]  LOcoeff\n");
    for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
            printf("%ld  %d  %10f  %10ld  %10f %5f  %5f\n", DMindex, dmturbconf[DMindex].on, dmturbconf[DMindex].ampl, dmturbconf[DMindex].tint, dmturbconf[DMindex].simtime, dmturbconf[DMindex].wspeed, dmturbconf[DMindex].LOcoeff);
        }
    
    return 0;
}




//
// create configuration shared memory structure for moving turbulence screen for DM
// one configuration per DM
//
int AOloopControl_DMturb_createconf()
{
    int result;
    long IDc1;
    char name[200];
    long DMindex;
    char errstr[200];

	printf("ENTERING FUNCTION AOloopControl_DMturb_createconf\n");
	fflush(stdout);

	if( dmdispcomb_loaded == 0 )
	{
		printf("============== AOloopControl_DM_loadconf\n");
		fflush(stdout);
		AOloopControl_DM_loadconf();    
		printf("=====>\n");
		fflush(stdout);
	}
	

	if( dmturb_loaded == 0 )
	{
		printf("=============== AOloopControl_DMturb_loadconf\n");
		fflush(stdout);
		AOloopControl_DMturb_loadconf();
		printf("=====>\n");
		fflush(stdout);
	}
	
	
    if( dmturb_loaded == 0 )
    {
        printf("Create/read DMturb configuration\n");
        fflush(stdout);

        SMturbfd = open(DMTURBCONF_FILENAME, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
        if (SMturbfd == -1) {
            sprintf(errstr, "Error opening (O_RDWR | O_CREAT | O_TRUNC) file \"%s\"", name);
            perror(errstr);
            exit(EXIT_FAILURE);
        }

        result = lseek(SMturbfd, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex-1, SEEK_SET);
        if (result == -1) {
            close(SMturbfd);
            perror("Error calling lseek() to 'stretch' the file");
            exit(EXIT_FAILURE);
        }

        result = write(SMturbfd, "", 1);
        if (result != 1) {
            close(SMturbfd);
            perror("Error writing last byte of the file");
            exit(EXIT_FAILURE);
        }

        dmturbconf = (AOLOOPCONTROL_DMTURBCONF*)mmap(0, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMturbfd, 0);
        if (dmturbconf == MAP_FAILED) {
            close(SMturbfd);
            perror("Error mmapping the file");
            exit(EXIT_FAILURE);
        }


        for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
            dmturbconf[DMindex].on = 0;
            
            dmturbconf[DMindex].wspeed = 10.0; // [m/s]            
            dmturbconf[DMindex].ampl = 0.01; // [um]
            dmturbconf[DMindex].LOcoeff = 0.2;            
            
            dmturbconf[DMindex].tint = 100; // [us]
            
            dmturbconf[DMindex].simtime = 0.0; // sec
        }
        dmturb_loaded = 1;

    }
    AOloopControl_printDMturbconf();
    
   	printf("EXITING FUNCTION AOloopControl_DMturb_createconf\n");
	fflush(stdout);
    
    return 0;
}








int AOloopControl_DMturb_loadconf(long DMindex)
{
    int result;
    char errstr[200];

    if( dmturb_loaded == 0 )
    {
        printf("Read configuration\n");

        SMturbfd = open(DMTURBCONF_FILENAME, O_RDWR, (mode_t)0600);
        if (SMturbfd == -1) {
            sprintf(errstr, "Error opening (O_RDWR) file \"%s\" in function AOloopControl_DMturb_loadconf", DMTURBCONF_FILENAME);
            perror(errstr);
		}
        else
        {
        //    exit(EXIT_FAILURE);
        

			dmturbconf = (AOLOOPCONTROL_DMTURBCONF*)mmap(0, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMturbfd, 0);
			if (dmturbconf == MAP_FAILED) {
				close(SMturbfd);
				printf("Error mmapping the file -> creating it\n");
				AOloopControl_DMturb_createconf();
			}
			dmturb_loaded = 1;
		}
	}

    return 0;
}





int AOloopControl_DM_dmturboff(long DMindex)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

    dmturbconf[DMindex].on = 0;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_wspeed(long DMindex, double wspeed)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].wspeed = wspeed;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_ampl(long DMindex, double ampl)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].ampl = ampl;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_LOcoeff(long DMindex, double LOcoeff)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

    dmturbconf[DMindex].LOcoeff = LOcoeff;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_tint(long DMindex, long tint)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].tint = tint;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}



int AOloopControl_DM_dmturb_printstatus(long DMindex)
{
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
//    AOloopControl_DMturb_loadconf(DMindex);

    printf("Run time = %.3f sec\n", dmturbconf[DMindex].simtime);
    printf("\n");
    printf("cnt              : %ld   (ave frequ = %.2f kHz)\n", dmturbconf[DMindex].cnt, 0.001*dmturbconf[DMindex].cnt/dmturbconf[DMindex].simtime);
    printf("\n");

    if(dmturbconf[DMindex].on == 1)
        printf("LOOP IS ON\n");
    else
        printf("LOOP IS OFF\n");

    printf("ampl    =  %.2f um\n", dmturbconf[DMindex].ampl);
    printf("wspeed  =  %.2f m/s\n", dmturbconf[DMindex].wspeed);
    printf("tint    =  %ld us\n", dmturbconf[DMindex].tint);
    printf("LOcoeff =  %.2f\n", dmturbconf[DMindex].LOcoeff);
    printf("Requested uptdate frequ = %.2f kHz\n", 0.001/(1.0e-6*dmturbconf[DMindex].tint));
    printf("\n");
    printf("\n");

    return(0);
}










// mode = 0 : send to DM
// mode = 1 : write to file, so that it can be later sent to DM


int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)
{
	float DMsizeM = 10.0; // DM size in meter
    long size_sx; // screen size
    long size_sy;
    long IDs1, IDs2;
    char name[200];
    long imsize = 2048;
    struct timespec tlast;
    struct timespec tdiff;
    struct timespec tdiff1;
    double tdiff1v;

    float screen0_X;
    float screen0_Y;
    long ii, jj, ii1;
    float x, y;
    float xpix, ypix;
    float xpixf, ypixf;
    long xpix1, xpix2, ypix1, ypix2;
    float ave;

    double angle = 1.0;
    double coeff = 0.001;
    double x1, fx1;
    
    double RMSval;
    long RMSvalcnt;
    double r;
    
    float pixscale = 0.1; // [m/pix]
    // Subaru pupil ~ 80 pix diam
    // Single actuator ~7 pix
    
    long DM_Xsize, DM_Ysize;

    long IDturbs1;
    long IDturb;
    double totim;
    long IDk;

	long k0 = 100;
	int k0init = 0;
	
	long k;
	int turbON;
	long IDout;
	FILE *fp;
	double dX, dY;
	double wspeedx, wspeedy;
	double RMSvaltot;
	long RMSvaltotcnt;


	int tint;
	long iter;

	if( dmturb_loaded == 0 )
	{
		printf("========= START AOloopControl_DMturb_createconf\n");
		fflush(stdout);
		AOloopControl_DMturb_createconf();
		printf("========= END AOloopControl_DMturb_createconf\n");
		fflush(stdout);
	}
	
    IDs1 = load_fits("turbscreen1.fits", "screen1", 1);
    IDs2 = load_fits("turbscreen2.fits", "screen2", 1);
    list_image_ID();
    
    if(IDs1==-1)
    {
        make_master_turbulence_screen_local("screen1", "screen2", imsize, 200.0, 1.0);
        IDs1 = image_ID("screen1");
        IDk = make_gauss("kernim", imsize, imsize, 20.0, 1.0);
        totim = 0.0;
        for(ii=0;ii<imsize*imsize;ii++)
            totim += data.image[IDk].array.F[ii];
        for(ii=0;ii<imsize*imsize;ii++)
            data.image[IDk].array.F[ii] /= totim;
        IDs2 = fconvolve("screen1", "kernim", "screen2");
        delete_image_ID("kernim");
        save_fits("screen1", "!turbscreen1.fits");
        save_fits("screen2", "!turbscreen2.fits");
    }
    

    printf("ARRAY SIZE = %ld %ld\n", (long) data.image[IDs1].md[0].size[0], (long) data.image[IDs1].md[0].size[1]);
    size_sx = data.image[IDs1].md[0].size[0];
    size_sy = data.image[IDs1].md[0].size[1];

	if(mode==0)
	{
		clock_gettime(CLOCK_REALTIME, &dmturbconf[DMindex].tstart);
		dmturbconf[DMindex].tend = dmturbconf[DMindex].tstart;
	}

    DM_Xsize = dmdispcombconf[DMindex].xsize;
    DM_Ysize = dmdispcombconf[DMindex].ysize;
    printf("DM %ld array size : %ld %ld\n", DMindex, DM_Xsize, DM_Ysize);
    list_image_ID();
    sprintf(name, "dm%02lddisp10", DMindex);
    read_sharedmem_image(name);
    list_image_ID();
    
    if(mode==0)
		dmturbconf[DMindex].on = 1;

    IDturbs1 = create_2Dimage_ID("turbs1", DM_Xsize, DM_Ysize);
    IDturb = create_2Dimage_ID("turbs", DM_Xsize, DM_Ysize);


	if(mode==1)
		IDout = create_3Dimage_ID(IDout_name, DM_Xsize, DM_Ysize, NBsamples);
		

	k = 0;
	if(mode == 0)
		turbON = dmturbconf[DMindex].on;
	else
		turbON = 1;
		
	printf("MODE = %d\n  DMindex = %ld", mode, DMindex);
	
	
	if(mode==1) // force periodic sequence if wind speed is sufficiently large
	{
		printf("Wind speed = %f m/s\n", dmturbconf[DMindex].wspeed);
		printf("angle      = %f rad\n", angle);
		printf("time interval = %.6f sec\n", 1.0e-6*dmturbconf[DMindex].tint);
		printf("NBsamples = %ld\n", NBsamples);
		printf("sequence time = %f sec\n", 1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		dX = dmturbconf[DMindex].wspeed*1.0e-6*dmturbconf[DMindex].tint*NBsamples*cos(angle);
		dY = dmturbconf[DMindex].wspeed*1.0e-6*dmturbconf[DMindex].tint*NBsamples*sin(angle);
		printf("dX x dY  =    %20f x %20f m\n", dX, dY);
		printf("turb screen size = %f m\n", size_sx*pixscale);
		printf("->   %10.5f  x  %10.5f screen\n", dX/(size_sx*pixscale), dY/(size_sy*pixscale));
	
		dX = (floor( dX/(size_sx*pixscale) + 1000.5 ) - 1000.0) * size_sx*pixscale;
		dY = (floor( dY/(size_sy*pixscale) + 1000.5 ) - 1000.0) * size_sy*pixscale;

		printf("dX x dY  =    %20f x %20f m\n", dX, dY);
		wspeedx = dX / (1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		wspeedy = dY / (1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		
		if(sqrt(wspeedx*wspeedx+wspeedy*wspeedy)<0.0001)
			{				
				wspeedx = dmturbconf[DMindex].wspeed*cos(angle);
				wspeedy = dmturbconf[DMindex].wspeed*sin(angle);
			}
		
		printf("wspeed = %f x %f m/s  -> %f m/s\n", wspeedx, wspeedy, sqrt(wspeedx*wspeedx+wspeedy*wspeedy));
	}

	
//	fp = fopen("test.txt", "w");
	
	RMSvaltot = 0.0;
	RMSvaltotcnt = 0;
	tint = dmturbconf[DMindex].tint;
	
    while(turbON == 1) // computation loop
    {

		if(mode==0)
		{
			usleep(dmturbconf[DMindex].tint);

			tlast = dmturbconf[DMindex].tend;
			clock_gettime(CLOCK_REALTIME, &dmturbconf[DMindex].tend);
			tdiff = time_diff(dmturbconf[DMindex].tstart, dmturbconf[DMindex].tend);
			tdiff1 =  time_diff(tlast, dmturbconf[DMindex].tend);
			tdiff1v = 1.0*tdiff1.tv_sec + 1.0e-9*tdiff1.tv_nsec;
		
			 dmturbconf[DMindex].simtime = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
		}
		else
		{
			tdiff1v = 1.0e-6*tint*k;
			dmturbconf[DMindex].simtime = tdiff1v;
		}
		
		if(mode == 0)
		{
			screen0_X += dmturbconf[DMindex].wspeed*tdiff1v*cos(angle); // [m]
			screen0_Y += dmturbconf[DMindex].wspeed*tdiff1v*sin(angle); // [m]
		}
		else
		{
			screen0_X = wspeedx*tdiff1v; // [m]
			screen0_Y = wspeedy*tdiff1v; // [m]
		}

		

        //dmturbconf[DMindex].simtime = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;


        for(ii=0; ii<DM_Xsize; ii++)
            for(jj=0; jj<DM_Ysize; jj++)
            {
                ii1 = jj*DM_Xsize+ii;

                x = DMsizeM*ii/DM_Xsize + screen0_X; // [m]
                y = DMsizeM*jj/DM_Ysize + screen0_Y; // [m]

                xpix = 0.5*size_sx + x/pixscale;
                ypix = 0.5*size_sy + y/pixscale;

                xpix1 = ((long) xpix)%size_sx;
                xpix2 = (xpix1+1)%size_sx;
                xpixf = xpix- (long) xpix;
                ypix1 = ((long) ypix)%size_sy;
                ypix2 = (ypix1+1)%size_sy;
                ypixf = ypix - (long) ypix;

                while(xpix1<0)
                    xpix1 = 0;
                while(xpix1>size_sx-1)
                    xpix1 = size_sx-1;

                if(ypix1<0)
                    ypix1 = 0;
                if(ypix1>size_sy-1)
                    ypix1 = size_sy-1;

			//	if((k%100==0))
			//	{
			//		if(((ii==0)&&(jj==0))) //||((ii==DM_Xsize-1)&&(jj==DM_Ysize-1)))
			//			printf("%05ld %20f (%6ld)   %5ld %5ld   %20f %20f   %4ld %4ld   %ld  [%ld %ld] [%ld %ld]\n", k, tdiff1v, dmturbconf[DMindex].tint, ii, jj, screen0_X, screen0_Y, xpix1, ypix1, IDs1, DM_Xsize, DM_Ysize, size_sx, size_sy);
			//	}

				
                data.image[IDturbs1].array.F[ii1] = 1.0*xpix1;

                data.image[IDturb].array.F[ii1] = (1.0-xpixf)*(1.0-ypixf)*(data.image[IDs1].array.F[ypix1*size_sx+xpix1] - (1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix1*size_sx+xpix1]);
                data.image[IDturb].array.F[ii1]  +=  (xpixf)*(1.0-ypixf)*(data.image[IDs1].array.F[ypix1*size_sx+xpix2]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix1*size_sx+xpix2]);
                data.image[IDturb].array.F[ii1]  += (1.0-xpixf)*(ypixf)*(data.image[IDs1].array.F[ypix2*size_sx+xpix1]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix2*size_sx+xpix1]);
                data.image[IDturb].array.F[ii1]  += xpixf*ypixf*(data.image[IDs1].array.F[ypix2*size_sx+xpix2]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix2*size_sx+xpix2]);
            }

        // proccess array
        
        ave = 0.0;
        for(ii1=0; ii1<DM_Xsize*DM_Ysize; ii1++)
            ave += data.image[IDturb].array.F[ii1];
        ave /= dmdispcombconf[DMindex].xysize;
        for(ii1=0; ii1<DM_Xsize*DM_Ysize; ii1++)
        {
            data.image[IDturb].array.F[ii1] -= ave;
            data.image[IDturb].array.F[ii1] *= coeff;
        }

        RMSval = 0.0;
        RMSvalcnt = 0;

        for(ii=0; ii<DM_Xsize; ii++)
            for(jj=0; jj<DM_Ysize; jj++)
            {
                ii1 = DM_Xsize*jj+ii;
                x = 0.5*DM_Xsize - 0.5 - ii;
                y = 0.5*DM_Ysize - 0.5 - jj;
                r = sqrt(x*x+y*y);
                if(r<DM_Xsize*0.5-1.0)
                {
                    RMSval += data.image[IDturb].array.F[ii1]*data.image[IDturb].array.F[ii1];
                    RMSvalcnt++;
                }
            }
        RMSval = sqrt(RMSval/RMSvalcnt);

	if(mode == 0)
	{
        x1 = log10(RMSval/dmturbconf[DMindex].ampl);
        fx1 = 1.0 + 50.0*exp(-5.0*x1*x1);
        coeff /= pow(10.0,x1/fx1);
	}
	else
	{	if(k0init==1)
		{
			RMSvaltot += RMSval;
			RMSvaltotcnt ++;
		}
	}
        
//        printf("STEP 001  %f %f\n", screen0_X, screen0_Y);
//        fflush(stdout);
        
		if(mode == 0)
		{
			sprintf(name, "dm%02lddisp10", DMindex);
			copy_image_ID("turbs", name, 0);
		}
		else
		{
			if(k0init==1)
			{
			//printf("STEP %5ld / %5ld       time = %12.6f    coeff = %18g   RMSval = %18g    %18f x %18f\n", k, NBsamples, tdiff1v, coeff, RMSval, screen0_X, screen0_Y);
			//fflush(stdout);
			//fprintf(fp, "%5ld  %12.6f      %18g     %18g    %18f  %18f  %18f\n", k, tdiff1v, coeff, RMSval, screen0_X, screen0_Y, dmturbconf[DMindex].wspeed);
			
			for(ii=0;ii<DM_Xsize*DM_Ysize;ii++)
				data.image[IDout].array.F[k*DM_Xsize*DM_Ysize+ii] = data.image[IDturb].array.F[ii];
			}
		//	usleep(dmturbconf[DMindex].tint);
		//	sprintf(name, "dm%02lddisp10", DMindex);
		//	copy_image_ID("turbs", name, 0);
		}
   
   //     save_fits("turbs", "!turbs.fits");
   //     save_fits("turbs1", "!turbs1.fits");
    
		
		if(mode==0)
			turbON = dmturbconf[DMindex].on;
		else
			{
				k ++;
				if((k==k0)&&(k0init==0))
				{
					printf("------ k->0 ----------\n");
					k0init = 1;
					k = 0;
				}
				
				if(k<NBsamples)
					turbON = 1;
				else
					turbON = 0;
			}
		
    }
	//fclose(fp);


	RMSval = RMSvaltot/RMSvaltotcnt;
	for(k=0;k<NBsamples;k++)
		for(ii=0;ii<DM_Xsize*DM_Ysize;ii++)
			data.image[IDout].array.F[k*DM_Xsize*DM_Ysize+ii] *= dmturbconf[DMindex].ampl/RMSval;


//	for(iter=0;iter<100;iter++)
	//	{
	AOloopControl_printDMturbconf();
		//	usleep(1000000);
	//	}

	//dmturbconf[DMindex].tint = tint;


    return(0);
}



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 5. MISC TESTS & UTILS                                                                           */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


long AOloopControl_mkDM_TT_circle(char *IDoutname, long DMindex, long NBpts, float ampl)
{
	long xsize, ysize, zsize, xysize;
	long IDout;
	long ii, jj, kk;
	float x, y, xslope, yslope;
	
	AOloopControl_DM_loadconf();
	xsize = dmdispcombconf[DMindex].xsize;
	ysize = dmdispcombconf[DMindex].ysize;
	zsize = NBpts;
	xysize = xsize*ysize;
	
	IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
	
	for(kk=0;kk<zsize;kk++)
	{
		xslope = ampl*cos(2.0*M_PI*kk/zsize);
		yslope = ampl*sin(2.0*M_PI*kk/zsize);
		
		for(ii=0;ii<xsize;ii++)
		{
			x = 2.0*ii/xsize-1.0;
			for(jj=0;jj<ysize;jj++)
			{
				y = 2.0*jj/ysize-1.0;
				data.image[IDout].array.F[kk*xysize+jj*xsize+ii] = x*xslope + y*yslope;
			}
		}
	}
	
	return(IDout);
}


//
// XYmode 
//	0 : single XYdiag pattern
//	1 : single X pattern
//	2 : single Y pattern
//	3 : single Xdiag pattern
//	4 : single Ydiag pattern
//
//	5 : XYdiag -> OFF ->
//	6 : X -> OFF ->
//	7 : Y -> OFF ->
//	8 : X -> Y ->
//	9 : Xdiag -> OFF ->
// 10 : Ydiag -> OFF ->
// 11 : Xdiag -> Ydiag ->

long AOloopControl_DM_mkAstroGrid_seq(char *IDoutname, long DMindex, int XYmode, int bin, long NBcycle)
{
	long xsize, ysize, zsize, xysize;
	long IDout;
	long ii, jj, kk, kk1;
	long IDx, IDy, IDxy, IDxd, IDyd;
	int sign = 1;
	
	AOloopControl_DM_loadconf();
	xsize = dmdispcombconf[DMindex].xsize;
	ysize = dmdispcombconf[DMindex].ysize;
	xysize = xsize*ysize;
	
	IDx = create_2Dimage_ID("_tmpX", xsize, ysize);
	for(ii=0;ii<xsize;ii++)
		for(jj=0;jj<ysize;jj++)
			{
				if((ii/bin)%2==0)
					data.image[IDx].array.F[jj*xsize+ii] = 1.0;
				else
					data.image[IDx].array.F[jj*xsize+ii] = -1.0;
			}
	
	IDy = create_2Dimage_ID("_tmpY", xsize, ysize);
	for(ii=0;ii<xsize;ii++)
		for(jj=0;jj<ysize;jj++)
			{
				if((jj/bin)%2==0)
					data.image[IDy].array.F[jj*xsize+ii] = 1.0;
				else
					data.image[IDy].array.F[jj*xsize+ii] = -1.0;
			}
	
	IDxy = create_2Dimage_ID("_tmpXY", xsize, ysize);
	for(ii=0;ii<xsize;ii++)
		for(jj=0;jj<ysize;jj++)
			data.image[IDxy].array.F[jj*xsize+ii] = data.image[IDx].array.F[jj*xsize+ii] * data.image[IDy].array.F[jj*xsize+ii];
	


	IDxd = create_2Dimage_ID("_tmpXd", xsize, ysize);
	for(ii=0;ii<xsize;ii++)
		for(jj=0;jj<ysize;jj++)
			{
				if((ii/bin + jj/bin)%4==0)
					data.image[IDxd].array.F[jj*xsize+ii] = 1.0;
				if((ii/bin + jj/bin)%4==2)
					data.image[IDxd].array.F[jj*xsize+ii] = -1.0;
			}

	IDyd = create_2Dimage_ID("_tmpYd", xsize, ysize);
	for(ii=0;ii<xsize;ii++)
		for(jj=0;jj<ysize;jj++)
			{
				if((ii/bin - jj/bin + ysize/bin)%4==0)
					data.image[IDyd].array.F[jj*xsize+ii] = 1.0;
				if((ii/bin - jj/bin + ysize/bin)%4==2)
					data.image[IDyd].array.F[jj*xsize+ii] = -1.0;
			}
	
	
	
	switch (XYmode) {
		
		case 0: // single XY-diag pattern
		zsize = 2; // only 2 frames
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
		kk = 0;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = data.image[IDxy].array.F[jj*xsize+ii];
		kk = 1;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = -data.image[IDxy].array.F[jj*xsize+ii];
		break;
		
		
		case 1: // single X pattern
		zsize = 2; // only 2 frames
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
		kk = 0;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = data.image[IDx].array.F[jj*xsize+ii];
		kk = 1;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = -data.image[IDx].array.F[jj*xsize+ii];
		break;
		
		
		
		case 2: // single Y pattern
		zsize = 2; // only 2 frames
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
		kk = 0;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = data.image[IDy].array.F[jj*xsize+ii];
		kk = 1;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = -data.image[IDy].array.F[jj*xsize+ii];
		break;
		
		

		case 3: // single Xdiag pattern
		zsize = 2; // only 2 frames
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
		kk = 0;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = data.image[IDxd].array.F[jj*xsize+ii];
		kk = 1;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = -data.image[IDxd].array.F[jj*xsize+ii];
		break;
		
		
		
		case 4: // single Ydiag pattern
		zsize = 2; // only 2 frames
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);	
		kk = 0;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = data.image[IDyd].array.F[jj*xsize+ii];
		kk = 1;
		for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = -data.image[IDyd].array.F[jj*xsize+ii];
		break;
		







		case 5: // XYdiag -> OFF ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		for(kk=0;kk<2*NBcycle;kk++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDxy].array.F[jj*xsize+ii];
			sign *= -1;
		}		
		break;
		
		
		case 6: // X -> OFF ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		for(kk=0;kk<2*NBcycle;kk++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDx].array.F[jj*xsize+ii];
			sign *= -1;
		}		
		break;
		
		
		case 7: // Y -> OFF ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		for(kk=0;kk<2*NBcycle;kk++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDy].array.F[jj*xsize+ii];
			sign *= -1;
		}		
		break;
		
		
		case 8: // X -> Y ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		kk = 0;
		for(kk1=0;kk1<2*NBcycle;kk1++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDx].array.F[jj*xsize+ii];
			sign *= -1;
			kk++;
		}		
		for(kk1=0;kk1<2*NBcycle;kk1++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDy].array.F[jj*xsize+ii];
			sign *= -1;
			kk++;
		}		
		break;
		

		case 9: // Xdiag -> OFF ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		for(kk=0;kk<2*NBcycle;kk++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDxd].array.F[jj*xsize+ii];
			sign *= -1;
		}		
		break;
		
		
		case 10: // Ydiag -> OFF ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		for(kk=0;kk<2*NBcycle;kk++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDyd].array.F[jj*xsize+ii];
			sign *= -1;
		}		
		break;
		
		
		case 11: // Xdiag -> Ydiag ->
		zsize = 2*NBcycle*2;
		IDout = create_3Dimage_ID(IDoutname, xsize, ysize, zsize);
		sign = 1;
		kk = 0;
		for(kk1=0;kk1<2*NBcycle;kk1++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDxd].array.F[jj*xsize+ii];
			sign *= -1;
			kk++;
		}		
		for(kk1=0;kk1<2*NBcycle;kk1++)
		{
			for(ii=0;ii<xsize;ii++)
			for(jj=0;jj<ysize;jj++)
				data.image[IDout].array.F[kk*xysize+jj*ysize+ii] = sign*data.image[IDyd].array.F[jj*xsize+ii];
			sign *= -1;
			kk++;
		}		
		break;

		
		
	}
	
	
	
	return(IDout);
}

