/**
 * @file    AOloopControl_perfTest_status.c
 * @brief   Adaptive Optics Control loop engine testing
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    21 Dec 2017
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <math.h>
#include <string.h>
#include <pthread.h>
#include <ncurses.h>

#include "CommandLineInterface/CLIcore.h"
#include "info/info.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "00CORE/00CORE.h"

/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif


/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */


static int wcol, wrow; // window size

// TIMING
static struct timespec tnow;
static struct timespec tdiff;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c








/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 1. STATUS / TESTING / PERF MEASUREMENT                                   */
/* =============================================================================================== */
/* =============================================================================================== */





int_fast8_t AOloopControl_perfTest_printloopstatus(long loop, long nbcol, long IDmodeval_dm, long IDmodeval, long IDmodevalave, long IDmodevalrms, long ksize)
{
    long k, kmin, kmax;
    long col;
//    long nbl = 1;
    float AVElim = 0.01; // [um]
    float RMSlim = 0.01; // [um]
    char imname[200];
	float ratio0, ratio;
	int color;
    long IDblknb;
    long block;
    float valPFres, valOL, valWFS;
	long m;
	uint32_t *sizeout;
	float ARPFgainAutob[100];
	float ARPFgainAutob_tot[100];


    printw("    loop number %ld    ", loop);


    if(AOconf[loop].on == 1)
        printw("loop is ON     ");
    else
        printw("loop is OFF    ");
        
     printw(" [%12lu]", AOconf[loop].LOOPiteration);

    /*  if(AOconf[loop].logon == 1)
          printw("log is ON   ");
      else
          printw("log is OFF  ");

    */


    if(sprintf(imname, "aol%ld_mode_blknb", loop) < 1) // block indices
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDblknb = image_ID(imname);

    if(IDblknb==-1)
        IDblknb = read_sharedmem_image(imname);

	
	if(AOconf[loop].ARPFon==1)
	{
		if(aoloopcontrol_var.aoconfID_modeARPFgainAuto == -1)
		{
		// multiplicative auto ratio on top of gain above
		sizeout = (uint32_t*) malloc(sizeof(uint32_t)*2);
		sizeout[0] = AOconf[loop].NBDMmodes;
		sizeout[1] = 1;
		
		if(sprintf(imname, "aol%ld_mode_ARPFgainAuto", loop) < 1) 
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		aoloopcontrol_var.aoconfID_modeARPFgainAuto = create_image_ID(imname, 2, sizeout, _DATATYPE_FLOAT, 1, 0);
		COREMOD_MEMORY_image_set_createsem(imname, 10);
		// initialize the gain to zero for all modes
		for(m=0;m<AOconf[loop].NBDMmodes; m++)
			data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m] = 1.0;
		free(sizeout);
		}
		
		for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
			{
				ARPFgainAutob[k] = 0.0;
				ARPFgainAutob_tot[k] = 0.0;
			}
		
        for(m=0; m<AOconf[loop].NBDMmodes; m++)
        {
            block = data.image[IDblknb].array.UI16[m];
			ARPFgainAutob[block] += data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto].array.F[m];
			ARPFgainAutob_tot[block] += 1.0;
        }
		
		for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
			ARPFgainAutob[k] /= ARPFgainAutob_tot[k];
		
	}
	


    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
    {
        if(sprintf(imname, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(imname);
    }


    printw("   STATUS = %3d %3d    ", AOconf[loop].status, AOconf[loop].statusM);

    kmax = (wrow-28)*(nbcol);

	printw("IMAGE TOTAL = %10f\n", AOconf[loop].WFStotalflux);
    printw("    Gain = %5.3f   maxlim = %5.3f     GPU = %d    kmax=%ld\n", AOconf[loop].gain, AOconf[loop].maxlimit, AOconf[loop].GPU0, kmax);
    printw("    DMprimWrite = %d   Predictive control state: %d        ARPF gain = %5.3f   AUTOTUNE LIM = %d (perc = %.2f %%  delta = %.3f nm mcoeff=%4.2f) GAIN = %d\n", AOconf[loop].DMprimaryWriteON, AOconf[loop].ARPFon, AOconf[loop].ARPFgain, AOconf[loop].AUTOTUNE_LIMITS_ON, AOconf[loop].AUTOTUNE_LIMITS_perc, 1000.0*AOconf[loop].AUTOTUNE_LIMITS_delta, AOconf[loop].AUTOTUNE_LIMITS_mcoeff, AOconf[loop].AUTOTUNE_GAINS_ON);
    printw(" TIMIMNG :  lfr = %9.3f Hz    hw lat = %5.3f fr   comp lat = %5.3f fr  wfs extr lat = %5.3f fr\n", AOconf[loop].loopfrequ, AOconf[loop].hardwlatency_frame, AOconf[loop].complatency_frame, AOconf[loop].wfsmextrlatency_frame);
    printw("loop iteration CNT : %lld   ", AOconf[loop].cnt);
    printw("\n");





    printw("=========== %6ld modes, %3ld blocks ================|------------ Telemetry [nm] ----------------|    |     LIMITS         |", AOconf[loop].NBDMmodes, AOconf[loop].DMmodesNBblock);
	if(AOconf[loop].ARPFon == 1)
		printw("---- Predictive Control ----- |");
	printw("\n");

    printw("BLOCK  #modes [ min - max ]    gain   limit   multf  |       dmC     Input  ->       WFS   Ratio  |    | hits/step    perc  |");
	if(AOconf[loop].ARPFon==1)
		printw("  PFres  |  Ratio  | autogain |");
	printw("\n");
	printw("\n");

    for(k=0; k<AOconf[loop].DMmodesNBblock; k++)
    {
        if(k==0)
            kmin = 0;
        else
            kmin = AOconf[loop].indexmaxMB[k-1];

        attron(A_BOLD);
        printw("%3ld", k);
        attroff(A_BOLD);

        printw("    %4ld [ %4ld - %4ld ]   %5.3f  %7.5f  %5.3f", AOconf[loop].NBmodes_block[k], kmin, AOconf[loop].indexmaxMB[k]-1, data.image[aoloopcontrol_var.aoconfID_gainb].array.F[k], data.image[aoloopcontrol_var.aoconfID_limitb].array.F[k], data.image[aoloopcontrol_var.aoconfID_multfb].array.F[k]);
        
        
        printw("  |  %8.2f  %8.2f  ->  %8.2f", 1000.0*(AOconf[loop].blockave_Crms[k]), 1000.0*AOconf[loop].blockave_OLrms[k], 1000.0*AOconf[loop].blockave_WFSrms[k]);
		

		
       
        ratio0 = AOconf[loop].blockave_WFSrms[k]/AOconf[loop].blockave_OLrms[k];
		if(ratio0>0.999)
			color=2;
		else
			color=3;
			
		attron(A_BOLD | COLOR_PAIR(color));
        printw("   %5.3f  ", ratio0);
        attroff(A_BOLD | COLOR_PAIR(color));

        if( AOconf[loop].blockave_limFrac[k] > 0.01 )
            attron(A_BOLD | COLOR_PAIR(2));

        printw("| %2ld | %9.3f  %6.2f\% |", k, AOconf[loop].blockave_limFrac[k],  100.0*AOconf[loop].blockave_limFrac[k]/AOconf[loop].NBmodes_block[k]);
        attroff(A_BOLD | COLOR_PAIR(2));
        
        
        //
		// PREDICTIVE CONTROL
		//
        if(AOconf[loop].ARPFon==1){
			printw("%8.2f |", 1000.0*AOconf[loop].blockave_PFresrms[k]);
			
			
			ratio = AOconf[loop].blockave_PFresrms[k]/AOconf[loop].blockave_OLrms[k];
			color = 0;
			if(ratio>1.0)
				color=2;
			if(ratio<ratio0)
				color=3;
				
			attron(A_BOLD | COLOR_PAIR(color));
			printw("  %5.3f |", ratio);
			attroff(A_BOLD | COLOR_PAIR(color));
			
			printw(" %6.4f", ARPFgainAutob[k]);
		}
	


		
		// WFS noise corrected	
	/*
		printw("\n");
		
		
		
		printw("          WFS noise removed ------->               ");
		
		valOL = AOconf[loop].blockave_OLrms[k]*AOconf[loop].blockave_OLrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
		if(valOL>0.0)
			valOL = sqrt(valOL);
		else
			valOL = 0.0;
		
		valWFS = AOconf[loop].blockave_WFSrms[k]*AOconf[loop].blockave_WFSrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
		if(valWFS>0.0)
			valWFS = sqrt(valWFS);
		else
			valWFS = 0.0;
			
		printw("  |            %8.2f  ->  %8.2f", 1000.0*valOL, 1000.0*valWFS);
		ratio0 = valWFS/valOL;
		if(ratio0>0.999)
			color=2;
		else
			color=3;
			
		attron(A_BOLD | COLOR_PAIR(color));
        printw("   %5.3f  ", ratio0);
        attroff(A_BOLD | COLOR_PAIR(color));

        if( AOconf[loop].blockave_limFrac[k] > 0.01 )
            attron(A_BOLD | COLOR_PAIR(2));

        printw("|    |                    |", k, AOconf[loop].blockave_limFrac[k],  100.0*AOconf[loop].blockave_limFrac[k]/AOconf[loop].NBmodes_block[k]);
        attroff(A_BOLD | COLOR_PAIR(2));
        
        if(AOconf[loop].ARPFon==1){
			valPFres = AOconf[loop].blockave_PFresrms[k]*AOconf[loop].blockave_PFresrms[k] - AOconf[loop].blockave_WFSnoise[k]*AOconf[loop].blockave_WFSnoise[k];
			if(valPFres>0.0)
				valPFres = sqrt(valPFres);
			else
				valPFres = 0.0;
			printw("%8.2f |", 1000.0*valPFres);
			
			
			ratio = valPFres/valOL;
			color = 0;
			if(ratio>1.0)
				color=2;
			if(ratio<ratio0)
				color=3;
				
			attron(A_BOLD | COLOR_PAIR(color));
			printw("  %5.3f |", ratio);
			attroff(A_BOLD | COLOR_PAIR(color));
		}*/
		printw("\n");

    }


    printw("\n");

    printw(" ALL   %4ld                                        ", AOconf[loop].NBDMmodes);
    printw("  |  %8.2f  %8.2f  ->  %8.2f", 1000.0*AOconf[loop].ALLave_Crms, 1000.0*AOconf[loop].ALLave_OLrms, 1000.0*AOconf[loop].ALLave_WFSrms);

    attron(A_BOLD);
    printw("   %5.3f  ", AOconf[loop].ALLave_WFSrms/AOconf[loop].ALLave_OLrms);
    attroff(A_BOLD);

    printw("| %2ld | %9.3f  %6.2f\% |\n", k, AOconf[loop].ALLave_limFrac,  100.0*AOconf[loop].ALLave_limFrac/AOconf[loop].NBDMmodes);

    printw("\n");

    //printw("            MODAL RMS (ALL MODES) : %6.4lf     AVERAGE :  %8.6lf       ( %20g / %8lld )\n", sqrt(AOconf[loop].RMSmodes), sqrt(AOconf[loop].RMSmodesCumul/AOconf[loop].RMSmodesCumulcnt), AOconf[loop].RMSmodesCumul, AOconf[loop].RMSmodesCumulcnt);



	// ====================================================================
	//                    SHOW INDIVIDUAL MODES
	// ====================================================================


    print_header(" [ gain 1000xlimit  mult ] MODES [nm]    DM correction -- WFS value -- WFS average -- WFS RMS     ", '-');


    if(kmax>AOconf[loop].NBDMmodes)
        kmax = AOconf[loop].NBDMmodes;

    col = 0;
    for(k=0; k<kmax; k++)
    {
	    float val;
		
		
        attron(A_BOLD);
        printw("%4ld ", k);
        attroff(A_BOLD);

        printw("[%5.3f %8.4f %5.3f] ", AOconf[loop].gain * data.image[aoloopcontrol_var.aoconfID_gainb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k], 1000.0 * data.image[aoloopcontrol_var.aoconfID_limitb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k], AOconf[loop].mult * data.image[aoloopcontrol_var.aoconfID_multfb].array.F[data.image[IDblknb].array.UI16[k]] * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[k]);

        // print current value on DM
        val = data.image[IDmodeval_dm].array.F[k];
        if(fabs(val)>0.99*AOconf[loop].maxlimit)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%+8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
        {
            if(fabs(val)>0.99*AOconf[loop].maxlimit*data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k])
            {
                attron(COLOR_PAIR(1));
                printw("%+8.3f ", 1000.0*val);
                attroff(COLOR_PAIR(1));
            }
            else
                printw("%+8.3f ", 1000.0*val);
        }

        // last reading from WFS
        printw("%+8.3f ", 1000.0*data.image[IDmodeval].array.F[k]);


        // Time average
        val = data.image[IDmodevalave].array.F[(ksize-1)*AOconf[loop].NBDMmodes+k];
        if(fabs(val)>AVElim)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%+8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
            printw("%+8.3f ", 1000.0*val);


        // RMS variation
        val = sqrt(data.image[IDmodevalrms].array.F[(ksize-1)*AOconf[loop].NBDMmodes+k]);
        if(fabs(val)>RMSlim)
        {
            attron(A_BOLD | COLOR_PAIR(2));
            printw("%8.3f ", 1000.0*val);
            attroff(A_BOLD | COLOR_PAIR(2));
        }
        else
            printw("%8.3f ", 1000.0*val);

        col++;
        if(col==nbcol)
        {
            col = 0;
            printw("\n");
        }
        else
            printw(" | ");
    }

    return(0);
}




int_fast8_t AOloopControl_perfTest_loopMonitor(long loop, double frequ, long nbcol)
{
    char name[200];
    // DM mode values
    long IDmodeval_dm;

    // WFS modes values
    long IDmodeval;
    long ksize;
    long IDmodevalave;
    long IDmodevalrms;
    char fname[200];


    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    printf("MEMORY HAS BEEN INITIALIZED\n");
    fflush(stdout);

    // load arrays that are required
    if(aoloopcontrol_var.aoconfID_cmd_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_cmd", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_meas_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_meas", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_meas_modes = read_sharedmem_image(name);
    }


    if(aoloopcontrol_var.aoconfID_RMS_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_RMS", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_RMS_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_AVE_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_AVE", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_AVE_modes = read_sharedmem_image(name);
    }


    // blocks
    if(aoloopcontrol_var.aoconfID_gainb == -1)
    {
        if(sprintf(name, "aol%ld_gainb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_gainb = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_multfb == -1)
    {
        if(sprintf(name, "aol%ld_multfb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_multfb = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_limitb == -1)
    {
        if(sprintf(name, "aol%ld_limitb", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_limitb = read_sharedmem_image(name);
    }


    // individual modes

    if(aoloopcontrol_var.aoconfID_DMmode_GAIN==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_GAIN", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_LIMIT_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_LIMIT", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_MULTF_modes==-1)
    {
        if(sprintf(name, "aol%ld_DMmode_MULTF", loop) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(name);
    }








    // real-time DM mode value

    if(sprintf(fname, "aol%ld_modeval_dm_now", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval_dm = read_sharedmem_image(fname);

    // real-time WFS mode value
    if(sprintf(fname, "aol%ld_modeval", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodeval = read_sharedmem_image(fname);

    // averaged WFS residual modes, computed by CUDACOMP_extractModesLoop
    if(sprintf(fname, "aol%ld_modeval_ave", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalave = read_sharedmem_image(fname);
    ksize = data.image[IDmodevalave].md[0].size[1]; // number of averaging line, each line is 2x averaged of previous line

    // averaged WFS residual modes RMS, computed by CUDACOMP_extractModesLoop
    if(sprintf(fname, "aol%ld_modeval_rms", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDmodevalrms = read_sharedmem_image(fname);




    initscr();
    getmaxyx(stdscr, wrow, wcol);


    start_color();
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    while( !kbdhit() )
    {
        usleep((long) (1000000.0/frequ));
        clear();
        attron(A_BOLD);
        print_header(" PRESS ANY KEY TO STOP MONITOR ", '-');
        attroff(A_BOLD);

        AOloopControl_perfTest_printloopstatus(loop, nbcol, IDmodeval_dm, IDmodeval, IDmodevalave, IDmodevalrms, ksize);

        refresh();
    }
    endwin();

    return 0;
}







// if updateconf=1, update configuration
int_fast8_t AOloopControl_perfTest_statusStats(int updateconf, long NBsample)
{
    long k;
    long statusmax = 21;
    long *statuscnt;
    long *statusMcnt;
    long *statusM1cnt;
    float usec0, usec1;
    int st;
    int RT_priority = 91; //any number from 0-99
    struct sched_param schedpar;
    const char *statusdef[21];
    const char *statusMdef[21];
    const char *statusM1def[21];
    int gpu;
    int nbgpu;
    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    double tdiffv;
    long *statusgpucnt;
    long *statusgpucnt2;
    double loopiterus;

    long long loopcnt;
    char imname[200];
    long long wfsimcnt;
    long long dmCcnt;

    int ret;


    float loopfrequ_measured, complatency_measured, wfsmextrlatency_measured;
    float complatency_frame_measured, wfsmextrlatency_frame_measured;


    FILE *fp;

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);



    statusdef[0] = "LOAD IMAGE";
    statusdef[1] = "DARK SUBTRACT";
    statusdef[2] = "COMPUTE WFS IMAGE TOTAL";
    statusdef[3] = "NORMALIZE WFS IMAGE";
    statusdef[4] = "SUBTRACT REFERENCE";
    statusdef[5] = "MULTIPLYING BY CONTROL MATRIX -> MODE VALUES : SETUP";
    statusdef[6] = "START CONTROL MATRIX MULTIPLICATION: CHECK IF NEW CM EXISTS";
    statusdef[7] = "CONTROL MATRIX MULT: CREATE COMPUTING THREADS";
    statusdef[8] = "CONTROL MATRIX MULT: WAIT FOR THREADS TO COMPLETE";
    statusdef[9] = "CONTROL MATRIX MULT: COMBINE TRHEADS RESULTS";
    statusdef[10] = "CONTROL MATRIX MULT: INCREMENT COUNTER AND EXIT FUNCTION";
    statusdef[11] = "MULTIPLYING BY GAINS";

    if(AOconf[LOOPNUMBER].CMMODE==0)
    {
        statusdef[12] = "ENTER SET DM MODES";
        statusdef[13] = "START DM MODES MATRIX MULTIPLICATION";
        statusdef[14] = "MATRIX MULT: CREATE COMPUTING THREADS";
        statusdef[15] = "MATRIX MULT: WAIT FOR THREADS TO COMPLETE";
        statusdef[16] = "MATRIX MULT: COMBINE TRHEADS RESULTS";
        statusdef[17] = "MATRIX MULT: INCREMENT COUNTER AND EXIT FUNCTION";
    }
    else
    {
        statusdef[12] = "REMOVE NAN VALUES";
        statusdef[13] = "ENFORCE STROKE LIMITS";
        statusdef[14] = "-";
        statusdef[15] = "-";
        statusdef[16] = "-";
        statusdef[17] = "-";
    }

    statusdef[18] = "LOG DATA";
    statusdef[19] = "READING IMAGE";
    statusdef[20] = "... WAITING FOR IMAGE";




    statusMdef[0] = "DARK SUBTRACT";
    statusMdef[1] = "NORMALIZE";
    statusMdef[2] = "EXTRACT WFS MODES";
    statusMdef[3] = "UPDATE CURRENT DM STATE";
    statusMdef[4] = "MIX PREDICTION WITH CURRENT DM STATE";
    statusMdef[5] = "MODAL FILTERING / CLIPPING";
    statusMdef[6] = "INTER-PROCESS LATENCY";
    statusMdef[7] = "";
    statusMdef[8] = "";
    statusMdef[9] = "";
    statusMdef[10] = "MODES TO DM ACTUATORS (GPU)";
    statusMdef[11] = "";
    statusMdef[12] = "";
    statusMdef[13] = "";
    statusMdef[14] = "";
    statusMdef[15] = "";
    statusMdef[16] = ""; 
    statusMdef[17] = "";
    statusMdef[18] = "";
    statusMdef[19] = "";
    statusMdef[20] = "... WAITING FOR IMAGE imWFS0";


    statusM1def[0] = "WRITING MODAL CORRECTION IN CIRCULAR BUFFER";
    statusM1def[1] = "COMPUTING TIME-DELAYED MODAL CORRECTION";
    statusM1def[2] = "COMPUTING TIME-DELAYED PREDICTED CORRECTION";
    statusM1def[3] = "COMPUTING OPEN LOOP WF";
    statusM1def[4] = "COMPUTING TELEMETRY";
    statusM1def[5] = "... WAITING FOR INPUT";
    statusM1def[6] = "";
    statusM1def[7] = "";
    statusM1def[8] = "";
    statusM1def[9] = "";
    statusM1def[10] = "";
    statusM1def[11] = "";
    statusM1def[12] = "";
    statusM1def[13] = "";
    statusM1def[14] = "";
    statusM1def[15] = "";
    statusM1def[16] = "";
    statusM1def[17] = "";
    statusM1def[18] = "";
    statusM1def[19] = "";
    statusM1def[20] = "";




    usec0 = 50.0;
    usec1 = 150.0;



    schedpar.sched_priority = RT_priority;
#ifndef __MACH__
    sched_setscheduler(0, SCHED_FIFO, &schedpar);
#endif


    nbgpu = AOconf[LOOPNUMBER].GPU0;


    printf("Measuring loop status distribution \n");
    fflush(stdout);

    statuscnt = (long*) malloc(sizeof(long)*statusmax);
    statusMcnt = (long*) malloc(sizeof(long)*statusmax);
    statusM1cnt = (long*) malloc(sizeof(long)*statusmax);
    statusgpucnt = (long*) malloc(sizeof(long)*nbgpu*10);
    statusgpucnt2 = (long*) malloc(sizeof(long)*nbgpu*10);


    for(st=0; st<statusmax; st++)
    {
        statuscnt[st] = 0;
        statusMcnt[st] = 0;
        statusM1cnt[st] = 0;
    }

    for(st=0; st<nbgpu*10; st++)
    {
        statusgpucnt[st] = 0;
        statusgpucnt2[st] = 0;
    }


    if(sprintf(imname, "aol%ld_wfsim", LOOPNUMBER) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoloopcontrol_var.aoconfID_wfsim = read_sharedmem_image(imname);

    if(sprintf(imname, "aol%ld_dmC", LOOPNUMBER) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    aoloopcontrol_var.aoconfID_dmC = read_sharedmem_image(imname);


    wfsimcnt = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0;
    dmCcnt = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0;



    loopcnt = AOconf[LOOPNUMBER].cnt;
    clock_gettime(CLOCK_REALTIME, &t1);
    for(k=0; k<NBsample; k++)
    {
		int stM;
		int stM1;
		
        usleep((long) (usec0 + usec1*(1.0*k/NBsample)));
        st = AOconf[LOOPNUMBER].status;
        stM = AOconf[LOOPNUMBER].statusM;
        stM1 = AOconf[LOOPNUMBER].statusM1;
        
        if(st<statusmax)
            statuscnt[st]++;
        if(stM<statusmax)
            statusMcnt[stM]++;
         if(stM1<statusmax)
            statusM1cnt[stM1]++;       
        
        
        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            // 1st matrix mult
            st = 10*gpu + AOconf[LOOPNUMBER].GPUstatus[gpu];
            statusgpucnt[st]++;

            // 2nd matrix mult
            st = 10*gpu + AOconf[LOOPNUMBER].GPUstatus[10+gpu];
            statusgpucnt2[st]++;
        }
    }
    loopcnt = AOconf[LOOPNUMBER].cnt - loopcnt;
    wfsimcnt = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0 - wfsimcnt;
    dmCcnt = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0 - dmCcnt;

    clock_gettime(CLOCK_REALTIME, &t2);
    tdiff = info_time_diff(t1, t2);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    printf("\n");
    loopiterus = 1.0e6*tdiffv/loopcnt;
    printf("Time diff = %f sec \n", tdiffv);
    printf("Loop freq = %8.2f Hz   -> single interation = %8.3f us\n", 1.0*loopcnt/tdiffv, loopiterus);
    printf("Number of iterations    loop: %10lld   wfs: %lld   dmC : %lld\n", loopcnt, wfsimcnt, dmCcnt);
    printf("MISSED FRAMES = %lld    fraction = %7.4f %%\n", wfsimcnt-loopcnt, 100.0*(wfsimcnt-loopcnt)/wfsimcnt);

    printf("\n");


    loopfrequ_measured = 1.0*loopcnt/tdiffv;
    if(updateconf==1)
        AOconf[LOOPNUMBER].loopfrequ = loopfrequ_measured;

	// Primary control matrix computation latency
    complatency_frame_measured = 1.0-1.0*statuscnt[20]/NBsample;
    if(updateconf==1)
        AOconf[LOOPNUMBER].complatency_frame = complatency_frame_measured;

    complatency_measured = complatency_frame_measured/loopfrequ_measured;
    if(updateconf==1)
        AOconf[LOOPNUMBER].complatency = complatency_measured;



    wfsmextrlatency_frame_measured = 1.0-1.0*statusMcnt[20]/NBsample;
    printf("==========> %ld %ld -> %f\n", statusMcnt[20], NBsample, wfsmextrlatency_frame_measured);
    if(updateconf==1)
        AOconf[LOOPNUMBER].wfsmextrlatency_frame = wfsmextrlatency_frame_measured;

    wfsmextrlatency_measured = wfsmextrlatency_frame_measured / loopfrequ_measured;
    if(updateconf==1)
        AOconf[LOOPNUMBER].wfsmextrlatency = wfsmextrlatency_measured;

    if(updateconf==1)
    {
        fp = fopen("conf/param_mloopfrequ.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].loopfrequ);
        fclose(fp);
    }

    if((fp=fopen("./conf/param_hardwlatency.txt", "r"))==NULL)
    {
        printf("WARNING: file ./conf/param_hardwlatency.txt missing\n");
    }
    else
    {
        if(fscanf(fp, "%50f", &AOconf[LOOPNUMBER].hardwlatency) != 1)
            printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

        printf("hardware latency = %f\n", AOconf[LOOPNUMBER].hardwlatency);
        fclose(fp);
        fflush(stdout);
    }

    printf("hardwlatency = %f\n", AOconf[LOOPNUMBER].hardwlatency);
    if(updateconf==1)
    {
        AOconf[LOOPNUMBER].hardwlatency_frame = AOconf[LOOPNUMBER].hardwlatency * AOconf[LOOPNUMBER].loopfrequ;

        fp = fopen("conf/param_hardwlatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].hardwlatency_frame);
        fclose(fp);

        fp = fopen("conf/param_complatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].complatency);
        fclose(fp);

        fp = fopen("conf/param_complatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].complatency_frame);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].wfsmextrlatency);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].wfsmextrlatency_frame);
        fclose(fp);
    }



    for(st=0; st<statusmax; st++)
        printf("STATUS %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statuscnt[st]/NBsample, statuscnt[st], NBsample, loopiterus*statuscnt[st]/NBsample , statusdef[st]);


	

    if(AOconf[LOOPNUMBER].GPU0!=0)
    {
        printf("\n");
        printf("          ----1--------2--------3--------4--------5--------6----\n");
        printf("                   wait im | ->GPU |     COMPUTE     |   ->CPU  \n");
        printf("          ------------------------------------------------------\n");

        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+1]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+2]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+3]/NBsample);
            printf("  %5.2f %%",  100.0*statusgpucnt[10*gpu+4]/NBsample);
            printf("  %5.2f %%",   100.0*statusgpucnt[10*gpu+5]/NBsample);
            printf("  %5.2f %%\n",  100.0*statusgpucnt[10*gpu+6]/NBsample);
        }
        for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+1]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+2]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+3]/NBsample);
            printf(" %5.2f us",  loopiterus*statusgpucnt[10*gpu+4]/NBsample);
            printf(" %5.2f us",   loopiterus*statusgpucnt[10*gpu+5]/NBsample);
            printf(" %5.2f us\n",  loopiterus*statusgpucnt[10*gpu+6]/NBsample);
        }

        printf("\n");
        if(AOconf[LOOPNUMBER].CMMODE == 0)
        {
            printf("          ----1--------2--------3--------4--------5--------6----\n");
            for(gpu=0; gpu<AOconf[LOOPNUMBER].GPU0; gpu++)
            {
                printf("GPU %2d  : ", gpu);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+1]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+2]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+3]/NBsample);
                printf("  %5.2f %%",  100.0*statusgpucnt2[10*gpu+4]/NBsample);
                printf("  %5.2f %%",   100.0*statusgpucnt2[10*gpu+5]/NBsample);
                printf("  %5.2f %%\n",  100.0*statusgpucnt2[10*gpu+6]/NBsample);
            }
        }
    }


	printf("\n--------------- MODAL STRING -------------------------------------------------------------\n");
    for(st=0; st<statusmax; st++)
        if(strlen(statusMdef[st])>0)
            printf("STATUSM  %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statusMcnt[st]/NBsample, statusMcnt[st], NBsample, loopiterus*statusMcnt[st]/NBsample , statusMdef[st]);




	printf("\n--------------- AUX MODAL STRING ---------------------------------------------------------\n");
    for(st=0; st<statusmax; st++)
        if(strlen(statusM1def[st])>0)
            printf("STATUSM1 %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] %s\n", st, 100.0*statusM1cnt[st]/NBsample, statusM1cnt[st], NBsample, loopiterus*statusM1cnt[st]/NBsample , statusM1def[st]);



    free(statuscnt);
    free(statusMcnt);
    free(statusgpucnt);
    free(statusgpucnt2);


    return 0;
}



int_fast8_t AOloopControl_perfTest_resetRMSperf()
{
    long k;
    char name[200];
    long kmin, kmax;


    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    AOconf[LOOPNUMBER].RMSmodesCumul = 0.0;
    AOconf[LOOPNUMBER].RMSmodesCumulcnt = 0;


    return 0;
}


int_fast8_t AOloopControl_perfTest_showparams(long loop)
{

    printf("loop number %ld\n", loop);

    if(AOconf[loop].on == 1)
        printf("loop is ON\n");
    else
        printf("loop is OFF\n");

    printf("Global gain = %f   maxlim = %f\n  multcoeff = %f  GPU = %d\n", AOconf[loop].gain, AOconf[loop].maxlimit, AOconf[loop].mult, AOconf[loop].GPU0);
    printf("    Predictive control state: %d        ARPF gain = %5.3f   AUTOTUNE: lim %d gain %d\n", AOconf[loop].ARPFon, AOconf[loop].ARPFgain, AOconf[loop].AUTOTUNE_LIMITS_ON,  AOconf[loop].AUTOTUNE_GAINS_ON);
    printf("WFS norm floor = %f\n", AOconf[loop].WFSnormfloor);

    printf("loopfrequ               =  %8.2f Hz\n", AOconf[loop].loopfrequ);
    printf("hardwlatency_frame      =  %8.2f fr\n", AOconf[loop].hardwlatency_frame);
    printf("complatency_frame       =  %8.2f fr\n", AOconf[loop].complatency_frame);
    printf("wfsmextrlatency_frame   =  %8.2f fr\n", AOconf[loop].wfsmextrlatency_frame);


    return 0;
}
