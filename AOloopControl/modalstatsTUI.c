// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalstatsTUI.c
 * @brief   modal statistics TUI — ANSI-native, no ncurses dependency
 *
 * Terminal output uses raw ANSI escape sequences via fpsCTRL_ansi.h,
 * the same primitive layer used by milk-fpsCTRL and milk-streamCTRL.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

/* ANSI raw-terminal helpers — key codes, color, cursor movement */
#include "fpsCTRL/fpsCTRL_ansi.h"

/* Storage for ansi raw-mode state (required by fpsCTRL_ansi.h) */
struct termios ansi__orig_termios;
int            ansi__raw_active = 0;


/* =========================================================
 * Local ANSI print helpers (replace TUI_printfw / TUI_newline)
 * ========================================================= */

/** ansi_printfw - printf to stdout (drop-in for TUI_printfw). */
#define ansi_printfw(fmt, ...)  printf(fmt, ##__VA_ARGS__)

/** ansi_newline - emit CR+LF (raw mode requires explicit CR). */
static inline void ansi_newline(void)
{
    if(write(STDOUT_FILENO, "\r\n", 2) < 0) {}
}

/** ansi_clearscreen - erase display and home cursor. */
static inline void ansi_clearscreen(void)
{
    if(write(STDOUT_FILENO, "\033[2J\033[H", 7) < 0) {}
}



/** ansi_get_termsize - query terminal rows/cols via ioctl. */
static inline void ansi_get_termsize(
    unsigned short *rows,
    unsigned short *cols
)
{
    struct winsize ws;

    if(ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0)
    {
        *rows = ws.ws_row;
        *cols = ws.ws_col;
    }
    else
    {
        *rows = 24;
        *cols = 80;
    }
}


/* =========================================================
 * Display state
 * ========================================================= */

static int MODALTUI_PF     = 0;
static int MODALTUI_DMfilt = 0;


typedef struct
{
    long  modeindex;
    long  NBmode;
    long  pscaleindex;
    float pscale;
} MODALSTATSTRUCT;


/* =========================================================
 * FPS boilerplate
 * ========================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "modalstatsTUI",
    .cmdkey      = "modalstatsTUI",
    .description = "modal stats TUI",
    .description_long =
        "TUI (Text User Interface) for real-time display of modal AO control statistics. Shows per-mode gains, RMS, and temporal evolution."
};

static uint64_t *AOloopindex;

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, \
      FPTYPE_UINT64, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "AO loop index")

FPS_V2_SECTION5(FPS_PARAMS)


static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


/* =========================================================
 * Key handler
 * ========================================================= */

static int modalstats_TUI_process_user_key(
    int               ch,
    MODALSTATSTRUCT  *mstatstruct
)
{
    DEBUG_TRACE_FSTART();

    int loopOK = 1;

    switch(ch)
    {

    case 'x':
        loopOK = 0;
        break;

    case ANSI_KEY_UP:
        mstatstruct->modeindex--;
        if(mstatstruct->modeindex < 0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case ANSI_KEY_DOWN:
        mstatstruct->modeindex++;
        if(mstatstruct->modeindex > mstatstruct->NBmode - 1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode - 1;
        }
        break;

    case ANSI_KEY_PGUP:
        mstatstruct->modeindex -= 10;
        if(mstatstruct->modeindex < 0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case ANSI_KEY_PGDN:
        mstatstruct->modeindex += 10;
        if(mstatstruct->modeindex > mstatstruct->NBmode - 1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode - 1;
        }
        break;

    case '+':
        mstatstruct->pscaleindex++;
        mstatstruct->pscale = powf(10.0f, mstatstruct->pscaleindex);
        break;

    case '-':
        mstatstruct->pscaleindex--;
        mstatstruct->pscale = powf(10.0f, mstatstruct->pscaleindex);
        break;

    case 'P':
        MODALTUI_PF = 1;
        break;

    case 'p':
        MODALTUI_PF = 0;
        break;

    case 'F':
        MODALTUI_DMfilt = 1;
        break;

    case 'f':
        MODALTUI_DMfilt = 0;
        break;

    default:
        break;
    }

    DEBUG_TRACE_FEXIT();
    return loopOK;
}


/* =========================================================
 * Fixed-width value printers
 * ========================================================= */

inline static void printfixedlen(
    float             val,
    MODALSTATSTRUCT  *mstatstruct
)
{
    long tmpl = (long)(mstatstruct->pscale * val);

    if((tmpl < 100000) && (tmpl > -100000))
    {
        ansi_printfw("%+5ld", tmpl);
    }
    else
    {
        ansi_setcolor(3);  /* yellow — overflow */
        ansi_printfw("+++++");
        ansi_unsetcolor(3);
    }
}


inline static void printfixedlen_unsigned(
    float             val,
    MODALSTATSTRUCT  *mstatstruct
)
{
    long tmpl = (long)(mstatstruct->pscale * val);

    if((tmpl < 100000) && (tmpl > -100000))
    {
        ansi_printfw("%5ld", tmpl);
    }
    else
    {
        ansi_setcolor(3);
        ansi_printfw("+++++");
        ansi_unsetcolor(3);
    }
}


/* =========================================================
 * Main TUI function
 * ========================================================= */

errno_t AOloopControl_modalstatsTUI(
    int loopindex
)
{
    DEBUG_TRACE_FSTART();
    printf("Modal stats TUI\n");

    MODALSTATSTRUCT mstatstruct;
    mstatstruct.modeindex   = 0;
    mstatstruct.pscaleindex = 0;
    mstatstruct.pscale      = 1.0f;

    uint32_t NBmode = 1;

    /* ---- connect to streams ---- */

    IMGID imgDMmodes;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_CMmodesDM", loopindex);
        imgDMmodes = imgid_make_from_name(name);
        resolveIMGID(&imgDMmodes, ERRMODE_WARN,
                     dcimg, dcnimg);
        NBmode = imgDMmodes.md->size[2];
    }
    mstatstruct.NBmode = NBmode;

    IMGID imgmodevalWFS;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalWFS", loopindex);
        imgmodevalWFS = imgid_make_from_name(name);
        resolveIMGID(&imgmodevalWFS, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmodevalWFS.ID == -1) return RETURN_FAILURE;
        NBmode = imgmodevalWFS.md->size[0];
    }
    mstatstruct.NBmode = NBmode;

    IMGID imgmodevalDM;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalDM", loopindex);
        imgmodevalDM = imgid_make_from_name(name);
        resolveIMGID(&imgmodevalDM, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmodevalDM.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmodevalDMf;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalDMf", loopindex);
        imgmodevalDMf = imgid_make_from_name(name);
        resolveIMGID(&imgmodevalDMf, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmodevalDMf.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmodevalOL;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalOL", loopindex);
        imgmodevalOL = imgid_make_from_name(name);
        resolveIMGID(&imgmodevalOL, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmodevalOL.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmgain;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mgain", loopindex);
        imgmgain = imgid_make_from_name(name);
        resolveIMGID(&imgmgain, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmgain.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmmult;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mmult", loopindex);
        imgmmult = imgid_make_from_name(name);
        resolveIMGID(&imgmmult, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmmult.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmlimit;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mlimit", loopindex);
        imgmlimit = imgid_make_from_name(name);
        resolveIMGID(&imgmlimit, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmlimit.ID == -1) return RETURN_FAILURE;
    }

    IMGID imgmlimitcntfrac;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mlimitcntfrac", loopindex);
        imgmlimitcntfrac = imgid_make_from_name(name);
        resolveIMGID(&imgmlimitcntfrac, ERRMODE_WARN,
                     dcimg, dcnimg);
                     if (imgmlimitcntfrac.ID == -1) return RETURN_FAILURE;
    }

    /* ---- stat accumulation streams ---- */

    IMGID imgmvalDMave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalDMave", loopindex);
        imgmvalDMave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalDMrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalDMrms", loopindex);
        imgmvalDMrms = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalWFSave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalWFSave", loopindex);
        imgmvalWFSave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalWFSrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalWFSrms", loopindex);
        imgmvalWFSrms = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalOLave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalOLave", loopindex);
        imgmvalOLave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalOLrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalOLrms", loopindex);
        imgmvalOLrms = stream_connect_create_2Df32(name, NBmode, 1);
    }

    /* ---- predictive control streams ---- */

    IMGID imgmPFmix;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mPFmix", loopindex);
        imgmPFmix = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalPFresrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mvalPFresrms", loopindex);
        imgmvalPFresrms = stream_connect_create_2Df32(name, NBmode, 1);
    }

    /* Unused direct reference needed to avoid unused-variable warnings */
    (void) imgmvalWFSrms;
    (void) imgmvalOLrms;

    /* ---- working buffers ---- */

    double *WFSave    = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *WFSrms    = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *DMave     = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *DMrms     = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *OLave     = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *OLrms     = (double *) malloc(sizeof(double) * mstatstruct.NBmode);
    double *DMmodenorm = (double *) malloc(sizeof(double) * NBmode);

    /* ---- DM mode norms ---- */

    if(imgDMmodes.ID == -1)
    {
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            DMmodenorm[mi] = 1.0;
        }
    }
    else
    {
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            double val    = 0.0;
            double valcnt = 0.0;
            uint64_t npix = (uint64_t)imgDMmodes.md->size[0]
                            * imgDMmodes.md->size[1];

            for(uint64_t ii = 0; ii < npix; ii++)
            {
                float v = imgDMmodes.im->array.F[mi * npix + ii];
                val    += (double)(v * v);
                valcnt += 1.0;
            }
            DMmodenorm[mi] = sqrt(val / valcnt);
        }
    }

    /* ---- enter raw ANSI terminal mode ---- */

    ansi_raw_mode_enter();

    unsigned short wrow = 24, wcol = 80;
    ansi_get_termsize(&wrow, &wcol);

    int  loopOK  = 1;
    long loopcnt = 0;

    int  mirange = (int)(wrow - 5);
    int  mimin   = 0;
    int  mimax   = mirange;
    long mioffset = 0;


    while(loopOK == 1)
    {
        usleep(1000);
        int ch = ansi_get_key();

        loopOK = modalstats_TUI_process_user_key(ch, &mstatstruct);

        /* refresh terminal size each frame */
        ansi_get_termsize(&wrow, &wcol);
        mirange = (int)(wrow - 5);

        ansi_clearscreen();

        long mi = mstatstruct.modeindex;

        /* ---- scroll window ---- */

        if(mimax > mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }
        if(mstatstruct.modeindex > mimax - 10)
        {
            mioffset++;
            mimin = mioffset;
            mimax = mioffset + mirange;
            if(mimax > mstatstruct.NBmode)
            {
                mioffset--;
                mimin = mioffset;
                mimax = mioffset + mirange;
            }
        }
        if(mimax > mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }
        if(mstatstruct.modeindex < mimin + 10)
        {
            mioffset--;
            mimin = mioffset;
            mimax = mioffset + mirange;
            if(mimin < 0)
            {
                mioffset = 0;
                mimin    = 0;
                mimax    = mimin + mirange;
            }
        }
        if(mimax > mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }

        /* ---- header ---- */

        ansi_printfw(" PRESS x to exit, +/- change display scale,"
                     " UP/DOWN PGUP PGDOWN");
        ansi_newline();
        ansi_printfw(" [P/p] Predictive filter  [F/f] DM filtering");
        ansi_newline();
        ansi_printfw("Loop %d  -  Mode %5ld / %5ld [%5d-%5d]"
                     "  loopcnt %ld",
                     loopindex,
                     mstatstruct.modeindex,
                     mstatstruct.NBmode,
                     mimin, mimax,
                     loopcnt);
        ansi_newline();
        ansi_printfw("scale = %f", mstatstruct.pscale);
        ansi_newline();

        ansi_printfw("MODE [ gain  mult    lim  ]"
                     "        WFS          |"
                     "          DM       |");
        if(MODALTUI_DMfilt)
        {
            ansi_printfw("    DMf       |");
        }
        ansi_printfw("          OL       | LIMTRUC WFS/OL   DM/OL");
        if(MODALTUI_PF)
        {
            ansi_printfw("  [ mPFmix ]    res    res/WFS   res/pOL");
        }
        ansi_newline();

        /* ---- update stat buffers ---- */

        {
            static long buffindex0 = -1;
            long buffindex = imgmvalOLrms.md->cnt0;

            if(buffindex != buffindex0)
            {
                for(int32_t mii = mimin; mii < mimax; mii++)
                {
                    WFSave[mii] = imgmvalWFSave.im->array.F[mii];
                    WFSrms[mii] = imgmvalWFSrms.im->array.F[mii];
                    WFSrms[mii] = sqrt(WFSrms[mii] * WFSrms[mii]
                                       - WFSave[mii] * WFSave[mii]);

                    DMave[mii] = imgmvalDMave.im->array.F[mii];
                    DMrms[mii] = imgmvalDMrms.im->array.F[mii];
                    DMrms[mii] = sqrt(DMrms[mii] * DMrms[mii]
                                      - DMave[mii] * DMave[mii]);

                    OLave[mii] = imgmvalOLave.im->array.F[mii];
                    OLrms[mii] = imgmvalOLrms.im->array.F[mii];
                    OLrms[mii] = sqrt(OLrms[mii] * OLrms[mii]
                                      - OLave[mii] * OLave[mii]);
                }
                buffindex0 = buffindex;
            }
        } // buffindex update

        /* ---- per-mode rows ---- */

        for(mi = mimin; mi < mimax; mi++)
        {
            if(mi == mstatstruct.modeindex)
            {
                ansi_bold_on();
            }

            ansi_printfw("%4ld [%5.3f %5.3f %8.6f]   ",
                         mi,
                         imgmgain.im->array.F[mi],
                         imgmmult.im->array.F[mi],
                         imgmlimit.im->array.F[mi]);

            /* WFS telemetry */
            printfixedlen(imgmodevalWFS.im->array.F[mi] * DMmodenorm[mi],
                          &mstatstruct);
            ansi_printfw(" ");

            {
                int color = (WFSave[mi] > WFSrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen(WFSave[mi] * DMmodenorm[mi], &mstatstruct);
                ansi_unsetcolor(color);
            }

            ansi_printfw(" ");
            printfixedlen_unsigned(WFSrms[mi] * DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" | ");

            /* DM telemetry */
            printfixedlen(imgmodevalDM.im->array.F[mi] * DMmodenorm[mi],
                          &mstatstruct);
            ansi_printfw(" ");

            {
                int color = (DMave[mi] > DMrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen(DMave[mi] * DMmodenorm[mi], &mstatstruct);
                ansi_unsetcolor(color);
            }

            ansi_printfw(" ");
            printfixedlen_unsigned(DMrms[mi] * DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" | ");

            /* DMf telemetry (optional) */
            if(MODALTUI_DMfilt)
            {
                printfixedlen(imgmodevalDMf.im->array.F[mi] * DMmodenorm[mi],
                              &mstatstruct);
                ansi_printfw(" ");
                printfixedlen((imgmodevalDMf.im->array.F[mi]
                               - imgmodevalDM.im->array.F[mi])
                              * DMmodenorm[mi],
                              &mstatstruct);
                ansi_printfw("  | ");
            }

            /* Open-loop telemetry */
            printfixedlen(imgmodevalOL.im->array.F[mi] * DMmodenorm[mi],
                          &mstatstruct);
            ansi_printfw(" ");

            {
                int color = (OLave[mi] > OLrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen(OLave[mi] * DMmodenorm[mi], &mstatstruct);
                ansi_unsetcolor(color);
            }

            ansi_printfw(" ");
            printfixedlen_unsigned(OLrms[mi] * DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" | ");

            /* Limit truncation fraction */
            {
                float truncfract = imgmlimitcntfrac.im->array.F[mi];

                int color = 0;
                if(truncfract > 0.01f)
                {
                    color = 3;  /* yellow */
                }
                if(truncfract > 0.1f)
                {
                    color = 4;  /* red */
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", truncfract);
                ansi_unsetcolor(color);
                ansi_printfw("  ");
            }

            /* WFS/OL and DM/OL ratios */
            {
                float WFSoverOL = WFSrms[mi] / OLrms[mi];
                float DMoverOL  = DMrms[mi]  / OLrms[mi];

                int color = 0;
                if(WFSoverOL < 0.9f)
                {
                    color = 2;  /* green */
                }
                if(WFSoverOL > 1.0f)
                {
                    color = 4;  /* red */
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", WFSoverOL);
                ansi_unsetcolor(color);
                ansi_printfw("  ");

                color = 0;
                if(DMoverOL > 0.5f)
                {
                    color = 2;
                }
                if(DMoverOL > 1.0f)
                {
                    color = 4;
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", DMoverOL);
                ansi_unsetcolor(color);
            }

            /* Predictive filter (optional) */
            if(MODALTUI_PF)
            {
                ansi_printfw("  [ %6.4f ]", imgmPFmix.im->array.F[mi]);
                ansi_printfw("   %6.4f ",   imgmvalPFresrms.im->array.F[mi]);
                ansi_printfw("  %5.3f ",
                             imgmvalPFresrms.im->array.F[mi]
                             / imgmvalWFSrms.im->array.F[mi]);
                ansi_printfw("  %8.6f ",
                             imgmvalPFresrms.im->array.F[mi]
                             / imgmvalOLrms.im->array.F[mi]);
            }

            ansi_newline();

            if(mi == mstatstruct.modeindex)
            {
                ansi_bold_off();
            }
        } // for mi

        fflush(stdout);
        loopcnt++;
    } // while loopOK

    ansi_raw_mode_exit();

    free(WFSave);
    free(WFSrms);
    free(DMave);
    free(DMrms);
    free(OLave);
    free(OLrms);
    free(DMmodenorm);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    AOloopControl_modalstatsTUI((int) *AOloopindex);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

errno_t
CLIADDCMD_AOloopControl__modalstatsTUI()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
