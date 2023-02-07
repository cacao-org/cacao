/**
 * @file    AOloopControl_DM_config.c
 * @brief   DM control
 *
 * To be used for AOloopControl module
 *
 *
 *
 */

#include <err.h>
#include <fcntl.h>
#include <malloc.h>
#include <math.h>
#include <ncurses.h>
#include <sched.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "fft/fft.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"
#include "info/info.h"
#include "statistic/statistic.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

#include <time.h>

extern long NB_DMindex;

AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
extern int                      dmdispcomb_loaded;
extern int                      SMfd;

AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
extern int                dmturb_loaded;
extern int                SMturbfd;

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                                                                                 */
/* 3. CONFIGURATION */
/*                                                                                                 */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

errno_t AOloopControl_printDMconf()
{
    long DMindex;
    char IDvolt_str[4];
    char maxvolt_str[7];
    char voltname_str[64];

    printf(
        "DM | on |  x |  y | Nbch | busy | ave | DClevel | monint  | stat | "
        "IDdisp | voltmode | volttype | "
        "stroke100 | IDvolt | maxvolt |   voltname  |\n");
    for(DMindex = 0; DMindex < NB_DMindex; DMindex++)
    {
        if(dmdispcombconf[DMindex].voltmode == 1)
        {
            sprintf(IDvolt_str, "%3ld", dmdispcombconf[DMindex].IDvolt);
            sprintf(maxvolt_str, "%6.2f", dmdispcombconf[DMindex].MAXVOLT);
            sprintf(voltname_str, "%s", dmdispcombconf[DMindex].voltname);
        }
        else
        {
            sprintf(IDvolt_str, "---");
            sprintf(maxvolt_str, "------");
            sprintf(voltname_str, "-----------");
        }

        printf(
            "%02ld |  %1d |%3u |%3u |  %02ld  |   %1d  |  %1d  | %6.2f  "
            "|%8ld |   %02d |   %3ld  |    %4d  |    "
            "%4d "
            " | %9.2f |   %3s  |  %6s | %11s |\n",
            DMindex,
            dmdispcombconf[DMindex].ON,
            dmdispcombconf[DMindex].xsize,
            dmdispcombconf[DMindex].ysize,
            dmdispcombconf[DMindex].NBchannel,
            dmdispcombconf[DMindex].busy,
            dmdispcombconf[DMindex].AveMode,
            dmdispcombconf[DMindex].DClevel,
            dmdispcombconf[DMindex].moninterval,
            dmdispcombconf[DMindex].status,
            dmdispcombconf[DMindex].IDdisp,
            dmdispcombconf[DMindex].voltmode,
            dmdispcombconf[DMindex].volttype,
            dmdispcombconf[DMindex].stroke100,
            IDvolt_str,
            maxvolt_str,
            voltname_str);
    }

    return RETURN_SUCCESS;
}

int AOloopControl_DM_dmdispcombstatus(long DMindex)
{
    long long mcnt = 0;
    int       ch;
    // int wcol, wrow; // window size

    AOloopControl_DM_loadconf();

    initscr();
    //    getmaxyx(stdscr, wrow, wcol);

    start_color();
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_BLACK, COLOR_RED);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    while(!kbdhit())
    {
        {
            struct timespec tim;
            tim.tv_sec  = 0;
            tim.tv_nsec = (long)(1000 * dmdispcombconf[DMindex].moninterval);
            nanosleep(&tim, NULL);
            // usleep(dmdispcombconf[DMindex].moninterval);
        }

        clear();
        attron(A_BOLD);
        print_header(" PRESS ANY KEY TO STOP MONITOR ", '-');
        attroff(A_BOLD);
        printw("monitor sample %ld\n", mcnt);
        printw("\n");

        printw(
            "=========== DM %d "
            "==============================================\n",
            DMindex);
        printw("\n");
        printw("ON                = %10d\n", dmdispcombconf[DMindex].ON);
        printw("size              = %ld x %ld = %ld\n",
               dmdispcombconf[DMindex].xsize,
               dmdispcombconf[DMindex].ysize,
               dmdispcombconf[DMindex].xysize);
        printw("NBchannel         = %10d      Number of DM channels\n",
               dmdispcombconf[DMindex].NBchannel);
        printw("\n");

        printw("loopcnt           = %10ld     AO loop index\n",
               dmdispcombconf[DMindex].loopcnt);
        printw("updatecnt         = %10ld     Number of DM updates\n",
               dmdispcombconf[DMindex].updatecnt);
        printw("busy              = %10d   \n", dmdispcombconf[DMindex].busy);
        printw("nsecwait          = %10ld ns\n",
               dmdispcombconf[DMindex].nsecwait);
        printw("delay time        = %10.3f us\n",
               dmdispcombconf[DMindex].tdelay * 1.0e6);
        printw("disp->V time      = %10.3f us\n",
               dmdispcombconf[DMindex].time_disp2V * 1.0e6);

        if(dmdispcombconf[DMindex].TrigMode == 1)
        {
            attron(A_BOLD);
        }
        printw("\n");
        printw(
            "=========== TRIGGER MODE = %d "
            "====================================\n",
            dmdispcombconf[DMindex].TrigMode);
        printw("TrigChan          = %10d      DM trigger channel\n",
               dmdispcombconf[DMindex].TrigChan);
        printw("TrigSem           = %10d      DM trigger semaphore\n",
               dmdispcombconf[DMindex].TrigSem);
        printw(
            "==============================================================="
            "=\n");
        if(dmdispcombconf[DMindex].TrigMode == 1)
        {
            attroff(A_BOLD);
        }
        printw("\n");

        if(dmdispcombconf[DMindex].voltmode == 1)
        {
            attron(A_BOLD);
        }
        printw(
            "=========== OUTPUT VOLT "
            "========================================\n");
        printw(
            "voltmode          = %10d      Configured for output voltage ?\n",
            dmdispcombconf[DMindex].voltmode);
        printw("voltON            = %10d      DM voltage ouptut activated ?\n",
               dmdispcombconf[DMindex].voltON);
        printw(
            "volttype          = %10d      DM type (1:linear bipolar, 2: "
            "quadratic unipolar)\n",
            dmdispcombconf[DMindex].volttype);
        printw("stroke100         = %10.3f um   Displacement [um] for 100 V\n",
               dmdispcombconf[DMindex].stroke100);
        printw("MAXVOLT           = %10.2f V    Maximum voltage\n",
               dmdispcombconf[DMindex].MAXVOLT);
        printw(
            "AveMode           = %10d      Averaging mode for combined "
            "displacement\n",
            dmdispcombconf[DMindex].AveMode);
        printw("    0: Offset combined to DC level\n");
        printw("       Clip displacement at 0.0\n");
        printw("    1: Remove average and apply DC offset.\n");
        printw("       Clip at 0.0.\n");
        printw(
            "    2: Do not apply DC offset, do not offset sum, do not clip\n");
        printw("DClevel           = %10.5f um   Displacement DC offset\n",
               dmdispcombconf[DMindex].DClevel);
        printw(
            "==============================================================="
            "=\n");
        if(dmdispcombconf[DMindex].voltmode == 1)
        {
            attroff(A_BOLD);
        }
        printw("\n");

        for(ch = 0; ch < dmdispcombconf[DMindex].NBchannel; ch++)
        {
            printw(" CHANNEL %2d  gain = %10.3f   dm%02lddisp%02ld   %10ld\n",
                   ch,
                   dmdispcombconf[DMindex].dmdispgain[ch],
                   DMindex,
                   ch,
                   (long) dmdispcombconf[DMindex].dmdispcnt[ch]);
        }
        printw("\n");

        if(dmdispcombconf[DMindex].dm2dm_mode == 1)
        {
            attron(A_BOLD);
        }
        printw(
            "=========== DM-to-DM OUPUT (CPU-based) "
            "==========================\n");
        printw("dm2dm_mode        = %10d      DM controls an output DM ?\n",
               dmdispcombconf[DMindex].dm2dm_mode);
        printw("xsizeout          = %10ld      x size of output DM\n",
               dmdispcombconf[DMindex].xsizeout);
        printw("ysizeout          = %10ld      y size of output DM\n",
               dmdispcombconf[DMindex].ysizeout);
        printw("dm2dm_DMmodes     = %10s      output DM modes\n",
               dmdispcombconf[DMindex].dm2dm_DMmodes_name);
        printw("dm2dm_outdisp     = %10s      ouput DM displacement\n",
               dmdispcombconf[DMindex].dm2dm_outdisp_name);
        printw(
            "==============================================================="
            "=\n");
        if(dmdispcombconf[DMindex].dm2dm_mode == 1)
        {
            attroff(A_BOLD);
        }
        printw("\n");

        if(dmdispcombconf[DMindex].dm2dm_mode == 1)
        {
            attron(A_BOLD);
        }
        printw(
            "======== DM CONTROL TO OUTPUT WFS REFERENCE (CPU-based) "
            "========\n");
        printw("wfsrefmode        = %10d      DM controls output wfsref ?\n",
               dmdispcombconf[DMindex].wfsrefmode);
        printw("xsizewfsref       = %10ld      x size of output WFS ref\n",
               dmdispcombconf[DMindex].xsizeout);
        printw("ysizewfsref       = %10ld      y size of output WFS ref\n",
               dmdispcombconf[DMindex].ysizeout);
        printw("wfsref_RespMat    = %10s      DM-to-WFSref matrix\n",
               dmdispcombconf[DMindex].wfsref_RespMat_name);
        printw("wfsref_out        = %10s      output WFSref\n",
               dmdispcombconf[DMindex].wfsref_out_name);
        printw(
            "==============================================================="
            "=\n");
        if(dmdispcombconf[DMindex].dm2dm_mode == 1)
        {
            attroff(A_BOLD);
        }
        printw("\n");

        printw("status            = %10d\n", dmdispcombconf[DMindex].status);
        printw("moninterval       = %10d us\n",
               dmdispcombconf[DMindex].moninterval);
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

    if(ch < dmdispcombconf[DMindex].NBchannel)
    {
        dmdispcombconf[DMindex].dmdispgain[ch] = gain;
    }

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
