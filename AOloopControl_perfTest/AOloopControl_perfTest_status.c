/**
 * @file    AOloopControl_perfTest_status.c
 * @brief   Adaptive Optics Control loop engine testing
 *
 * AO engine uses stream data structure
 *
 *
 *
 *
 */

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                        HEADER FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#include <math.h>
#include <ncurses.h>
#include <pthread.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "CommandLineInterface/timeutils.h"

#include "info/info.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                      DEFINES, MACROS */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                  GLOBAL DATA DECLARATION */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

static int wcol, wrow; // window size

// TIMING
// static struct timespec tnow;
// static struct timespec tdiff;

/* ===============================================================================================
 */
/*                                     MAIN DATA STRUCTURES */
/* ===============================================================================================
 */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf;            // declared in AOloopControl.c
extern AOloopControl_var   aoloopcontrol_var; // declared in AOloopControl.c

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 1. STATUS / TESTING / PERF MEASUREMENT */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

errno_t AOloopControl_perfTest_printloopstatus(
    long                            loop,
    long                            nbcol,
    __attribute__((unused)) imageID IDmodeval_dm,
    __attribute__((unused)) imageID IDmodeval,
    __attribute__((unused)) imageID IDmodevalave,
    __attribute__((unused)) imageID IDmodevalrms,
    __attribute__((unused)) long    ksize)
{
    long kmin, kmax;
    // long col;
    //     long nbl = 1;
    // float AVElim = 0.01; // [um]
    // float RMSlim = 0.01; // [um]
    char  imname[STRINGMAXLEN_IMGNAME];
    float ratio0, ratio;
    int   color;
    long  IDblknb;
    long  block;
    //    float valPFres, valOL, valWFS;
    uint32_t *sizeout;
    float     ARPFgainAutob[100];
    float     ARPFgainAutob_tot[100];

    printw("    loop number %ld    ", loop);

    if(AOconf[loop].aorun.on == 1)
    {
        printw("loop is ON     ");
    }
    else
    {
        printw("loop is OFF    ");
    }

    printw(" [%12lu]", AOconf[loop].aorun.LOOPiteration);

    /*  if(AOconf[loop].logon == 1)
        printw("log is ON   ");
    else
        printw("log is OFF  ");

    */

    WRITE_IMAGENAME(imname, "aol%ld_mode_blknb", loop);
    IDblknb = image_ID(imname);

    if(IDblknb == -1)
    {
        IDblknb = read_sharedmem_image(imname);
    }

    if(AOconf[loop].aorun.ARPFon == 1)
    {
        if(aoloopcontrol_var.aoconfID_modeARPFgainAuto == -1)
        {
            // multiplicative auto ratio on top of gain above
            sizeout = (uint32_t *) malloc(sizeof(uint32_t) * 2);
            if(sizeout == NULL)
            {
                PRINT_ERROR("malloc returns NULL pointer");
                abort();
            }
            sizeout[0] = AOconf[loop].AOpmodecoeffs.NBDMmodes;
            sizeout[1] = 1;

            WRITE_IMAGENAME(imname, "aol%ld_mode_ARPFgainAuto", loop);
            create_image_ID(imname,
                            2,
                            sizeout,
                            _DATATYPE_FLOAT,
                            1,
                            0,
                            0,
                            &(aoloopcontrol_var.aoconfID_modeARPFgainAuto));
            COREMOD_MEMORY_image_set_createsem(imname, 10);
            // initialize the gain to zero for all modes
            for(unsigned int m = 0; m < AOconf[loop].AOpmodecoeffs.NBDMmodes;
                    m++)
            {
                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto]
                .array.F[m] = 1.0;
            }
            free(sizeout);
        }

        for(unsigned int k = 0; k < AOconf[loop].AOpmodecoeffs.DMmodesNBblock;
                k++)
        {
            ARPFgainAutob[k]     = 0.0;
            ARPFgainAutob_tot[k] = 0.0;
        }

        for(unsigned int m = 0; m < AOconf[loop].AOpmodecoeffs.NBDMmodes; m++)
        {
            block = data.image[IDblknb].array.UI16[m];
            ARPFgainAutob[block] +=
                data.image[aoloopcontrol_var.aoconfID_modeARPFgainAuto]
                .array.F[m];
            ARPFgainAutob_tot[block] += 1.0;
        }

        for(unsigned int k = 0; k < AOconf[loop].AOpmodecoeffs.DMmodesNBblock;
                k++)
        {
            ARPFgainAutob[k] /= ARPFgainAutob_tot[k];
        }
    }

    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
    {
        WRITE_IMAGENAME(imname, "aol%ld_DMmode_LIMIT", loop);
        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(imname);
    }

    printw("   STATUS = %3d %3d    ",
           AOconf[loop].AOtiminginfo.status,
           AOconf[loop].AOtiminginfo.statusM);

    kmax = (wrow - 28) * (nbcol);

    printw("WFS IMAGE TOTAL = %10f -> AVE = %10.3f\n",
           AOconf[loop].WFSim.WFStotalflux,
           AOconf[loop].WFSim.WFStotalflux / AOconf[loop].WFSim.sizeWFS);
    printw("    Gain = %5.3f   maxlim = %5.3f     GPU = %d    kmax=%ld\n",
           AOconf[loop].aorun.gain,
           AOconf[loop].aorun.maxlimit,
           AOconf[loop].AOcompute.GPU0,
           kmax);

    printw(
        "    DMprimWrite = %d   Predictive control state: %d        ARPF "
        "gain = %5.3f   AUTOTUNE LIM = %d (perc = "
        "%.2f %%  delta = %.3f nm mcoeff=%4.2f) GAIN = %d\n",
        AOconf[loop].aorun.DMprimaryWriteON,
        AOconf[loop].aorun.ARPFon,
        AOconf[loop].aorun.ARPFgain,
        AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_ON,
        AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_perc,
        1000.0 * AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta,
        AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_mcoeff,
        AOconf[loop].AOAutoTune.AUTOTUNE_GAINS_ON);

    printw(
        " TIMIMNG :  lfr = %9.3f Hz    hw lat = %5.3f fr   comp lat = %5.3f "
        "fr  wfs extr lat = %5.3f fr\n",
        AOconf[loop].AOtiminginfo.loopfrequ,
        AOconf[loop].AOtiminginfo.hardwlatency_frame,
        AOconf[loop].AOtiminginfo.complatency_frame,
        AOconf[loop].AOtiminginfo.wfsmextrlatency_frame);
    printw("loop iteration CNT : %lld   ", AOconf[loop].aorun.cnt);
    printw("\n");

    printw(
        "=========== %6ld modes, %3ld blocks ================|------------ "
        "Telemetry [nm] ----------------|    |    "
        " LIMITS         |",
        AOconf[loop].AOpmodecoeffs.NBDMmodes,
        AOconf[loop].AOpmodecoeffs.DMmodesNBblock);
    if(AOconf[loop].aorun.ARPFon == 1)
    {
        printw("---- Predictive Control ----- |");
    }
    printw("\n");

    printw(
        "BLOCK  #modes [ min - max ]    gain   limit   multf  |       dmC  "
        "Input[OL] ->    WFS[CL]  Ratio  |    | "
        "hits/step    perc  |");
    if(AOconf[loop].aorun.ARPFon == 1)
    {
        printw("  PFres  |  Ratio | PFautog |");
    }
    printw("\n");
    printw("\n");

    for(unsigned int k = 0; k < AOconf[loop].AOpmodecoeffs.DMmodesNBblock; k++)
    {
        if(k == 0)
        {
            kmin = 0;
        }
        else
        {
            kmin = AOconf[loop].AOpmodecoeffs.indexmaxMB[k - 1];
        }

        attron(A_BOLD);
        printw("%3ld", k);
        attroff(A_BOLD);

        printw("    %4ld [ %4ld - %4ld ]   %5.3f  %7.5f  %5.3f",
               AOconf[loop].AOpmodecoeffs.NBmodes_block[k],
               kmin,
               AOconf[loop].AOpmodecoeffs.indexmaxMB[k] - 1,
               data.image[aoloopcontrol_var.aoconfID_gainb].array.F[k],
               data.image[aoloopcontrol_var.aoconfID_limitb].array.F[k],
               data.image[aoloopcontrol_var.aoconfID_multfb].array.F[k]);

        printw("  |  %8.2f  %8.2f  ->  %8.2f",
               1000.0 * (AOconf[loop].AOpmodecoeffs.blockave_Crms[k]),
               1000.0 * AOconf[loop].AOpmodecoeffs.blockave_OLrms[k],
               1000.0 * AOconf[loop].AOpmodecoeffs.blockave_WFSrms[k]);

        ratio0 = AOconf[loop].AOpmodecoeffs.blockave_WFSrms[k] /
                 AOconf[loop].AOpmodecoeffs.blockave_OLrms[k];
        if(ratio0 > 0.999)
        {
            color = 2;
        }
        else
        {
            color = 3;
        }

        attron(A_BOLD | COLOR_PAIR(color));
        printw("   %5.3f  ", ratio0);
        attroff(A_BOLD | COLOR_PAIR(color));

        if(AOconf[loop].AOpmodecoeffs.blockave_limFrac[k] > 0.01)
        {
            attron(A_BOLD | COLOR_PAIR(2));
        }

        printw("| %2ld | %9.3f  %6.2f\% |",
               k,
               AOconf[loop].AOpmodecoeffs.blockave_limFrac[k],
               100.0 * AOconf[loop].AOpmodecoeffs.blockave_limFrac[k] /
               AOconf[loop].AOpmodecoeffs.NBmodes_block[k]);
        attroff(A_BOLD | COLOR_PAIR(2));

        //
        // PREDICTIVE CONTROL
        //
        if(AOconf[loop].aorun.ARPFon == 1)
        {
            printw("%8.2f |",
                   1000.0 * AOconf[loop].AOpmodecoeffs.blockave_PFresrms[k]);

            ratio = AOconf[loop].AOpmodecoeffs.blockave_PFresrms[k] /
                    AOconf[loop].AOpmodecoeffs.blockave_OLrms[k];
            color = 0;
            if(ratio > 1.0)
            {
                color = 2;
            }
            if(ratio < ratio0)
            {
                color = 3;
            }

            attron(A_BOLD | COLOR_PAIR(color));
            printw("  %5.3f |", ratio);
            attroff(A_BOLD | COLOR_PAIR(color));

            printw(" %6.4f", ARPFgainAutob[k]);
        }

        // WFS noise corrected
        /*
              printw("\n");



              printw("          WFS noise removed ------->               ");

              valOL =
         AOconf[loop].AOpmodecoeffs.blockave_OLrms[k]*AOconf[loop].AOpmodecoeffs.blockave_OLrms[k]
         -
         AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k]*AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k];
              if(valOL>0.0)
                      valOL = sqrt(valOL);
              else
                      valOL = 0.0;

              valWFS =
         AOconf[loop].AOpmodecoeffs.blockave_WFSrms[k]*AOconf[loop].AOpmodecoeffs.blockave_WFSrms[k]
         -
         AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k]*AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k];
              if(valWFS>0.0)
                      valWFS = sqrt(valWFS);
              else
                      valWFS = 0.0;

              printw("  |            %8.2f  ->  %8.2f", 1000.0*valOL,
         1000.0*valWFS); ratio0 = valWFS/valOL; if(ratio0>0.999) color=2; else
                      color=3;

              attron(A_BOLD | COLOR_PAIR(color));
          printw("   %5.3f  ", ratio0);
          attroff(A_BOLD | COLOR_PAIR(color));

          if( AOconf[loop].AOpmodecoeffs.blockave_limFrac[k] > 0.01 )
              attron(A_BOLD | COLOR_PAIR(2));

          printw("|    |                    |", k,
         AOconf[loop].AOpmodecoeffs.blockave_limFrac[k],
         100.0*AOconf[loop].AOpmodecoeffs.blockave_limFrac[k]/AOconf[loop].AOpmodecoeffs.NBmodes_block[k]);
          attroff(A_BOLD | COLOR_PAIR(2));

          if(AOconf[loop].aorun.ARPFon==1){
                      valPFres =
         AOconf[loop].AOpmodecoeffs.blockave_PFresrms[k]*AOconf[loop].AOpmodecoeffs.blockave_PFresrms[k]
         -
         AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k]*AOconf[loop].AOpmodecoeffs.blockave_WFSnoise[k];
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

    printw(" ALL   %4ld                                        ",
           AOconf[loop].AOpmodecoeffs.NBDMmodes);
    printw("  |  %8.2f  %8.2f  ->  %8.2f",
           1000.0 * AOconf[loop].AOpmodecoeffs.ALLave_Crms,
           1000.0 * AOconf[loop].AOpmodecoeffs.ALLave_OLrms,
           1000.0 * AOconf[loop].AOpmodecoeffs.ALLave_WFSrms);

    attron(A_BOLD);
    printw("   %5.3f  ",
           AOconf[loop].AOpmodecoeffs.ALLave_WFSrms /
           AOconf[loop].AOpmodecoeffs.ALLave_OLrms);
    attroff(A_BOLD);

    printw("| %2ld | %9.3f  %6.2f\% |\n",
           0,
           AOconf[loop].AOpmodecoeffs.ALLave_limFrac,
           100.0 * AOconf[loop].AOpmodecoeffs.ALLave_limFrac /
           AOconf[loop].AOpmodecoeffs.NBDMmodes);

    printw("\n");

    // printw("            MODAL RMS (ALL MODES) : %6.4lf     AVERAGE :  %8.6lf (
    // %20g / %8lld )\n", sqrt(AOconf[loop].AOpmodecoeffs.RMSmodes),
    // sqrt(AOconf[loop].AOpmodecoeffs.RMSmodesCumul/AOconf[loop].AOpmodecoeffs.RMSmodesCumulcnt),
    // AOconf[loop].AOpmodecoeffs.RMSmodesCumul,
    // AOconf[loop].AOpmodecoeffs.RMSmodesCumulcnt);

    // ====================================================================
    //                    SHOW INDIVIDUAL MODES
    // ====================================================================

    /*
      print_header(" [ gain 1000xlimit  mult ] MODES [nm]    DM correction --
     WFS value -- WFS average -- WFS RMS     ", '-');


      if(kmax>AOconf[loop].AOpmodecoeffs.NBDMmodes)
          kmax = AOconf[loop].AOpmodecoeffs.NBDMmodes;

      col = 0;
      for(k=0; k<kmax; k++)
      {
          float val;


          attron(A_BOLD);
          printw("%4ld ", k);
          attroff(A_BOLD);

          printw("[%5.3f %8.4f %5.3f] ", AOconf[loop].aorun.gain *
     data.image[aoloopcontrol_var.aoconfID_gainb].array.F[data.image[IDblknb].array.UI16[k]]
     * data.image[aoloopcontrol_var.aoconfID_DMmode_GAIN].array.F[k], 1000.0 *
     data.image[aoloopcontrol_var.aoconfID_limitb].array.F[data.image[IDblknb].array.UI16[k]]
     * data.image[aoloopcontrol_var.aoconfID_LIMIT_modes].array.F[k],
     AOconf[loop].aorun.mult *
     data.image[aoloopcontrol_var.aoconfID_multfb].array.F[data.image[IDblknb].array.UI16[k]]
     * data.image[aoloopcontrol_var.aoconfID_MULTF_modes].array.F[k]);

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
          val =
     data.image[IDmodevalave].array.F[(ksize-1)*AOconf[loop].AOpmodecoeffs.NBDMmodes+k];
          if(fabs(val)>AVElim)
          {
              attron(A_BOLD | COLOR_PAIR(2));
              printw("%+8.3f ", 1000.0*val);
              attroff(A_BOLD | COLOR_PAIR(2));
          }
          else
              printw("%+8.3f ", 1000.0*val);


          // RMS variation
          val =
     sqrt(data.image[IDmodevalrms].array.F[(ksize-1)*AOconf[loop].AOpmodecoeffs.NBDMmodes+k]);
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
    */
    return RETURN_SUCCESS;
}

/**
 * ## Purpose
 *
 * Monitors AO loop performance \n
 * Prints status on screen\n
 *
 *
 */

errno_t AOloopControl_perfTest_loopMonitor(long loop, double frequ, long nbcol)
{
    char name[STRINGMAXLEN_IMGNAME];
    // DM mode values
    imageID IDmodeval_dm;

    // WFS modes values
    imageID IDmodeval;
    long    ksize;
    imageID IDmodevalave;
    imageID IDmodevalrms;
    //    char fname[STRINGMAXLEN_FILENAME];

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    printf("MEMORY HAS BEEN INITIALIZED\n");
    fflush(stdout);

    // load arrays that are required
    if(aoloopcontrol_var.aoconfID_cmd_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_cmd", loop);
        aoloopcontrol_var.aoconfID_cmd_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_meas_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_meas", loop);
        aoloopcontrol_var.aoconfID_meas_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_RMS_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_RMS", loop);
        aoloopcontrol_var.aoconfID_RMS_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_AVE_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_AVE", loop);
        aoloopcontrol_var.aoconfID_AVE_modes = read_sharedmem_image(name);
    }

    // blocks
    if(aoloopcontrol_var.aoconfID_gainb == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_gainb", loop);
        aoloopcontrol_var.aoconfID_gainb = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_multfb == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_multfb", loop);
        aoloopcontrol_var.aoconfID_multfb = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_limitb == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_limitb", loop);
        aoloopcontrol_var.aoconfID_limitb = read_sharedmem_image(name);
    }

    // individual modes

    if(aoloopcontrol_var.aoconfID_DMmode_GAIN == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_GAIN", loop);
        aoloopcontrol_var.aoconfID_DMmode_GAIN = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_LIMIT_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_LIMIT", loop);
        aoloopcontrol_var.aoconfID_LIMIT_modes = read_sharedmem_image(name);
    }

    if(aoloopcontrol_var.aoconfID_MULTF_modes == -1)
    {
        WRITE_IMAGENAME(name, "aol%ld_DMmode_MULTF", loop);
        aoloopcontrol_var.aoconfID_MULTF_modes = read_sharedmem_image(name);
    }

    // real-time DM mode value

    WRITE_IMAGENAME(name, "aol%ld_modeval_dm_now", loop);
    IDmodeval_dm = read_sharedmem_image(name);

    // real-time WFS mode value
    WRITE_IMAGENAME(name, "aol%ld_modeval", loop);
    IDmodeval = read_sharedmem_image(name);

    // averaged WFS residual modes, computed by CUDACOMP_extractModesLoop
    WRITE_IMAGENAME(name, "aol%ld_modeval_ave", loop);
    IDmodevalave = read_sharedmem_image(name);
    ksize =
        data.image[IDmodevalave]
        .md[0]
        .size[1]; // number of averaging line, each line is 2x averaged of
    // previous line

    // averaged WFS residual modes RMS, computed by CUDACOMP_extractModesLoop
    WRITE_IMAGENAME(name, "aol%ld_modeval_rms", loop);
    IDmodevalrms = read_sharedmem_image(name);

    initscr();
    getmaxyx(stdscr, wrow, wcol);

    start_color();
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);

    while(!kbdhit())
    {
        usleep((long)(1000000.0 / frequ));
        clear();

        attron(A_BOLD);
        print_header(" PRESS ANY KEY TO STOP MONITOR ", '-');
        attroff(A_BOLD);

        AOloopControl_perfTest_printloopstatus(loop,
                                               nbcol,
                                               IDmodeval_dm,
                                               IDmodeval,
                                               IDmodevalave,
                                               IDmodevalrms,
                                               ksize);

        refresh();
    }
    endwin();

    return RETURN_SUCCESS;
}

// if updateconf=1, update configuration
errno_t AOloopControl_perfTest_statusStats(int updateconf, long NBsample)
{
    long               k;
    long               statusmax = 21;
    long              *statuscnt;
    long              *statusMcnt;
    long              *statusM1cnt;
    float              usec0, usec1;
    int                st;
    int                RT_priority = 91; // any number from 0-99
    struct sched_param schedpar;
    const char        *statusdef[21];
    const char        *statusMdef[21];
    const char        *statusM1def[21];
    int                gpu;
    int                nbgpu;
    struct timespec    t1;
    struct timespec    t2;
    //    struct timespec tdiff;
    double tdiffv;
    long  *statusgpucnt;
    long  *statusgpucnt2;
    double loopiterus;

    long long loopcnt;
    char      imname[STRINGMAXLEN_IMGNAME];
    long long wfsimcnt;
    long long dmCcnt;

    float loopfrequ_measured, complatency_measured, wfsmextrlatency_measured;
    float complatency_frame_measured, wfsmextrlatency_frame_measured;

    FILE *fp;

    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    statusdef[0] = "LOAD IMAGE";
    statusdef[1] = "DARK SUBTRACT";
    statusdef[2] = "COMPUTE WFS IMAGE TOTAL";
    statusdef[3] = "NORMALIZE WFS IMAGE";
    statusdef[4] = "SUBTRACT REFERENCE";
    statusdef[5] = "MULTIPLYING BY CONTROL MATRIX -> MODE VALUES : SETUP";
    statusdef[6] =
        "START CONTROL MATRIX MULTIPLICATION: CHECK IF NEW CM EXISTS";
    statusdef[7]  = "CONTROL MATRIX MULT: CREATE COMPUTING THREADS";
    statusdef[8]  = "CONTROL MATRIX MULT: WAIT FOR THREADS TO COMPLETE";
    statusdef[9]  = "CONTROL MATRIX MULT: COMBINE TRHEADS RESULTS";
    statusdef[10] = "CONTROL MATRIX MULT: INCREMENT COUNTER AND EXIT FUNCTION";
    statusdef[11] = "MULTIPLYING BY GAINS";

    if(AOconf[LOOPNUMBER].aorun.CMMODE == 0)
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

    statusMdef[0]  = "DARK SUBTRACT";
    statusMdef[1]  = "NORMALIZE";
    statusMdef[2]  = "EXTRACT WFS MODES";
    statusMdef[3]  = "UPDATE CURRENT DM STATE";
    statusMdef[4]  = "MIX PREDICTION WITH CURRENT DM STATE";
    statusMdef[5]  = "MODAL FILTERING / CLIPPING";
    statusMdef[6]  = "INTER-PROCESS LATENCY";
    statusMdef[7]  = "";
    statusMdef[8]  = "";
    statusMdef[9]  = "";
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

    statusM1def[0]  = "WRITING MODAL CORRECTION IN CIRCULAR BUFFER";
    statusM1def[1]  = "COMPUTING TIME-DELAYED MODAL CORRECTION";
    statusM1def[2]  = "COMPUTING TIME-DELAYED PREDICTED CORRECTION";
    statusM1def[3]  = "COMPUTING OPEN LOOP WF";
    statusM1def[4]  = "COMPUTING TELEMETRY";
    statusM1def[5]  = "... WAITING FOR INPUT";
    statusM1def[6]  = "";
    statusM1def[7]  = "";
    statusM1def[8]  = "";
    statusM1def[9]  = "";
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
    sched_setscheduler(0, SCHED_FIFO, &schedpar);

    nbgpu = AOconf[LOOPNUMBER].AOcompute.GPU0;

    printf("Measuring loop status distribution \n");
    fflush(stdout);

    statuscnt = (long *) malloc(sizeof(long) * statusmax);
    if(statuscnt == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    statusMcnt = (long *) malloc(sizeof(long) * statusmax);
    if(statusMcnt == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    statusM1cnt = (long *) malloc(sizeof(long) * statusmax);
    if(statusM1cnt == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    statusgpucnt = (long *) malloc(sizeof(long) * nbgpu * 10);
    if(statusgpucnt == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    statusgpucnt2 = (long *) malloc(sizeof(long) * nbgpu * 10);
    if(statusgpucnt2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    for(st = 0; st < statusmax; st++)
    {
        statuscnt[st]   = 0;
        statusMcnt[st]  = 0;
        statusM1cnt[st] = 0;
    }

    for(st = 0; st < nbgpu * 10; st++)
    {
        statusgpucnt[st]  = 0;
        statusgpucnt2[st] = 0;
    }

    WRITE_IMAGENAME(imname, "aol%ld_wfsim", LOOPNUMBER);
    aoloopcontrol_var.aoconfID_wfsim = read_sharedmem_image(imname);

    WRITE_IMAGENAME(imname, "aol%ld_dmC", LOOPNUMBER);
    aoloopcontrol_var.aoconfID_dmC = read_sharedmem_image(imname);

    wfsimcnt = data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0;
    dmCcnt   = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0;

    loopcnt = AOconf[LOOPNUMBER].aorun.cnt;
    clock_gettime(CLOCK_MILK, &t1);
    for(k = 0; k < NBsample; k++)
    {
        int stM;
        int stM1;

        usleep((long)(usec0 + usec1 * (1.0 * k / NBsample)));
        st   = AOconf[LOOPNUMBER].AOtiminginfo.status;
        stM  = AOconf[LOOPNUMBER].AOtiminginfo.statusM;
        stM1 = AOconf[LOOPNUMBER].AOtiminginfo.statusM1;

        if(st < statusmax)
        {
            statuscnt[st]++;
        }
        if(stM < statusmax)
        {
            statusMcnt[stM]++;
        }
        if(stM1 < statusmax)
        {
            statusM1cnt[stM1]++;
        }

        for(gpu = 0; gpu < AOconf[LOOPNUMBER].AOcompute.GPU0; gpu++)
        {
            // 1st matrix mult
            st = 10 * gpu + AOconf[LOOPNUMBER].AOtiminginfo.GPUstatus[gpu];
            statusgpucnt[st]++;

            // 2nd matrix mult
            st = 10 * gpu + AOconf[LOOPNUMBER].AOtiminginfo.GPUstatus[10 + gpu];
            statusgpucnt2[st]++;
        }
    }
    loopcnt = AOconf[LOOPNUMBER].aorun.cnt - loopcnt;
    wfsimcnt =
        data.image[aoloopcontrol_var.aoconfID_wfsim].md[0].cnt0 - wfsimcnt;
    dmCcnt = data.image[aoloopcontrol_var.aoconfID_dmC].md[0].cnt0 - dmCcnt;

    clock_gettime(CLOCK_MILK, &t2);
    tdiffv = timespec_diff_double(t1, t2);
    printf("\n");
    loopiterus = 1.0e6 * tdiffv / loopcnt;
    printf("Time diff = %f sec \n", tdiffv);
    printf("Loop freq = %8.2f Hz   -> single interation = %8.3f us\n",
           1.0 * loopcnt / tdiffv,
           loopiterus);
    printf("Number of iterations    loop: %10lld   wfs: %lld   dmC : %lld\n",
           loopcnt,
           wfsimcnt,
           dmCcnt);
    printf("MISSED FRAMES = %lld    fraction = %7.4f %%\n",
           wfsimcnt - loopcnt,
           100.0 * (wfsimcnt - loopcnt) / wfsimcnt);

    printf("\n");

    loopfrequ_measured = 1.0 * loopcnt / tdiffv;
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.loopfrequ = loopfrequ_measured;
    }

    // Primary control matrix computation latency
    complatency_frame_measured = 1.0 - 1.0 * statuscnt[20] / NBsample;
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.complatency_frame =
            complatency_frame_measured;
    }

    complatency_measured = complatency_frame_measured / loopfrequ_measured;
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.complatency = complatency_measured;
    }

    wfsmextrlatency_frame_measured = 1.0 - 1.0 * statusMcnt[20] / NBsample;
    printf("==========> %ld %ld -> %f\n",
           statusMcnt[20],
           NBsample,
           wfsmextrlatency_frame_measured);
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.wfsmextrlatency_frame =
            wfsmextrlatency_frame_measured;
    }

    wfsmextrlatency_measured =
        wfsmextrlatency_frame_measured / loopfrequ_measured;
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.wfsmextrlatency =
            wfsmextrlatency_measured;
    }

    if(updateconf == 1)
    {
        fp = fopen("conf/param_mloopfrequ.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].AOtiminginfo.loopfrequ);
        fclose(fp);
    }

    if((fp = fopen("./conf/param_hardwlatency.txt", "r")) == NULL)
    {
        printf("WARNING: file ./conf/param_hardwlatency.txt missing\n");
    }
    else
    {
        if(fscanf(fp, "%50f", &AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency) !=
                1)
        {
            PRINT_ERROR("Cannot read parameter from file");
        }

        printf("hardware latency = %f\n",
               AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency);
        fclose(fp);
        fflush(stdout);
    }

    printf("hardwlatency = %f\n", AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency);
    if(updateconf == 1)
    {
        AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency_frame =
            AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency *
            AOconf[LOOPNUMBER].AOtiminginfo.loopfrequ;

        fp = fopen("conf/param_hardwlatency_frame.txt", "w");
        fprintf(fp,
                "%8.3f",
                AOconf[LOOPNUMBER].AOtiminginfo.hardwlatency_frame);
        fclose(fp);

        fp = fopen("conf/param_complatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].AOtiminginfo.complatency);
        fclose(fp);

        fp = fopen("conf/param_complatency_frame.txt", "w");
        fprintf(fp, "%8.3f", AOconf[LOOPNUMBER].AOtiminginfo.complatency_frame);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency.txt", "w");
        fprintf(fp, "%8.6f", AOconf[LOOPNUMBER].AOtiminginfo.wfsmextrlatency);
        fclose(fp);

        fp = fopen("conf/param_wfsmextrlatency_frame.txt", "w");
        fprintf(fp,
                "%8.3f",
                AOconf[LOOPNUMBER].AOtiminginfo.wfsmextrlatency_frame);
        fclose(fp);
    }

    for(st = 0; st < statusmax; st++)
    {
        printf(
            "STATUS %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ %9.3f us] "
            "%s\n",
            st,
            100.0 * statuscnt[st] / NBsample,
            statuscnt[st],
            NBsample,
            loopiterus * statuscnt[st] / NBsample,
            statusdef[st]);
    }

    if(AOconf[LOOPNUMBER].AOcompute.GPU0 != 0)
    {
        printf("\n");
        printf(
            "          "
            "----1--------2--------3--------4--------5--------6----\n");
        printf(
            "                   wait im | ->GPU |     COMPUTE     |   ->CPU "
            " \n");
        printf(
            "          "
            "------------------------------------------------------\n");

        for(gpu = 0; gpu < AOconf[LOOPNUMBER].AOcompute.GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf("  %5.2f %%", 100.0 * statusgpucnt[10 * gpu + 1] / NBsample);
            printf("  %5.2f %%", 100.0 * statusgpucnt[10 * gpu + 2] / NBsample);
            printf("  %5.2f %%", 100.0 * statusgpucnt[10 * gpu + 3] / NBsample);
            printf("  %5.2f %%", 100.0 * statusgpucnt[10 * gpu + 4] / NBsample);
            printf("  %5.2f %%", 100.0 * statusgpucnt[10 * gpu + 5] / NBsample);
            printf("  %5.2f %%\n",
                   100.0 * statusgpucnt[10 * gpu + 6] / NBsample);
        }
        for(gpu = 0; gpu < AOconf[LOOPNUMBER].AOcompute.GPU0; gpu++)
        {
            printf("GPU %2d  : ", gpu);
            printf(" %5.2f us",
                   loopiterus * statusgpucnt[10 * gpu + 1] / NBsample);
            printf(" %5.2f us",
                   loopiterus * statusgpucnt[10 * gpu + 2] / NBsample);
            printf(" %5.2f us",
                   loopiterus * statusgpucnt[10 * gpu + 3] / NBsample);
            printf(" %5.2f us",
                   loopiterus * statusgpucnt[10 * gpu + 4] / NBsample);
            printf(" %5.2f us",
                   loopiterus * statusgpucnt[10 * gpu + 5] / NBsample);
            printf(" %5.2f us\n",
                   loopiterus * statusgpucnt[10 * gpu + 6] / NBsample);
        }

        printf("\n");
        if(AOconf[LOOPNUMBER].aorun.CMMODE == 0)
        {
            printf(
                "          "
                "----1--------2--------3--------4--------5--------6----\n");
            for(gpu = 0; gpu < AOconf[LOOPNUMBER].AOcompute.GPU0; gpu++)
            {
                printf("GPU %2d  : ", gpu);
                printf("  %5.2f %%",
                       100.0 * statusgpucnt2[10 * gpu + 1] / NBsample);
                printf("  %5.2f %%",
                       100.0 * statusgpucnt2[10 * gpu + 2] / NBsample);
                printf("  %5.2f %%",
                       100.0 * statusgpucnt2[10 * gpu + 3] / NBsample);
                printf("  %5.2f %%",
                       100.0 * statusgpucnt2[10 * gpu + 4] / NBsample);
                printf("  %5.2f %%",
                       100.0 * statusgpucnt2[10 * gpu + 5] / NBsample);
                printf("  %5.2f %%\n",
                       100.0 * statusgpucnt2[10 * gpu + 6] / NBsample);
            }
        }
    }

    printf(
        "\n--------------- MODAL STRING "
        "-------------------------------------------------------------\n");
    for(st = 0; st < statusmax; st++)
        if(strlen(statusMdef[st]) > 0)
        {
            printf(
                "STATUSM  %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ "
                "%9.3f us] %s\n",
                st,
                100.0 * statusMcnt[st] / NBsample,
                statusMcnt[st],
                NBsample,
                loopiterus * statusMcnt[st] / NBsample,
                statusMdef[st]);
        }

    printf(
        "\n--------------- AUX MODAL STRING "
        "---------------------------------------------------------\n");
    for(st = 0; st < statusmax; st++)
        if(strlen(statusM1def[st]) > 0)
        {
            printf(
                "STATUSM1 %2d     %5.2f %%    [   %6ld  /  %6ld  ]   [ "
                "%9.3f us] %s\n",
                st,
                100.0 * statusM1cnt[st] / NBsample,
                statusM1cnt[st],
                NBsample,
                loopiterus * statusM1cnt[st] / NBsample,
                statusM1def[st]);
        }

    free(statuscnt);
    free(statusMcnt);
    free(statusgpucnt);
    free(statusgpucnt2);

    return RETURN_SUCCESS;
}

errno_t AOloopControl_perfTest_resetRMSperf()
{
    if(aoloopcontrol_var.AOloopcontrol_meminit == 0)
    {
        AOloopControl_InitializeMemory(1);
    }

    AOconf[LOOPNUMBER].AOpmodecoeffs.RMSmodesCumul    = 0.0;
    AOconf[LOOPNUMBER].AOpmodecoeffs.RMSmodesCumulcnt = 0;

    return RETURN_SUCCESS;
}

errno_t AOloopControl_perfTest_showparams(long loop)
{

    printf("loop number %ld\n", loop);

    if(AOconf[loop].aorun.on == 1)
    {
        printf("loop is ON\n");
    }
    else
    {
        printf("loop is OFF\n");
    }

    printf("Global gain = %f   maxlim = %f\n  multcoeff = %f  GPU = %d\n",
           AOconf[loop].aorun.gain,
           AOconf[loop].aorun.maxlimit,
           AOconf[loop].aorun.mult,
           AOconf[loop].AOcompute.GPU0);
    printf(
        "    Predictive control state: %d        ARPF gain = %5.3f   "
        "AUTOTUNE: lim %d gain %d\n",
        AOconf[loop].aorun.ARPFon,
        AOconf[loop].aorun.ARPFgain,
        AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_ON,
        AOconf[loop].AOAutoTune.AUTOTUNE_GAINS_ON);
    printf("WFS norm floor = %f\n", AOconf[loop].WFSim.WFSnormfloor);

    printf("loopfrequ               =  %8.2f Hz\n",
           AOconf[loop].AOtiminginfo.loopfrequ);
    printf("hardwlatency_frame      =  %8.2f fr\n",
           AOconf[loop].AOtiminginfo.hardwlatency_frame);
    printf("complatency_frame       =  %8.2f fr\n",
           AOconf[loop].AOtiminginfo.complatency_frame);
    printf("wfsmextrlatency_frame   =  %8.2f fr\n",
           AOconf[loop].AOtiminginfo.wfsmextrlatency_frame);

    return RETURN_SUCCESS;
}
