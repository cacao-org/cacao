// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    modalstatsTUI.c
 * @brief   Standalone modal statistics TUI — ANSI-native, no CLIcore/ncurses
 *
 * Connects directly to ImageStreamIO shared-memory streams and uses raw
 * ANSI escape sequences via fpsCTRL_ansi.h with poll()-paced input.
 */

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ImageStreamIO/ImageStreamIO.h"
#include "fpsCTRL/fpsCTRL_ansi.h"
#include "milk_config.h"
#include "milk_help.h"
#include "modalstatsTUI.h"

/* Storage for ansi raw-mode state (required by fpsCTRL_ansi.h) */
struct termios ansi__orig_termios;
int            ansi__raw_active = 0;
int            ansi__orig_flags = -1;

static volatile sig_atomic_t g_keep_running = 1;

static void tui_signal_handler(int sig)
{
    if (sig == SIGINT || sig == SIGTERM || sig == SIGHUP)
    {
        g_keep_running = 0;
        return;
    }

    static const char reset_seq[] = "\033[?25h\033[?7h\033[0m\n";
    if (write(STDERR_FILENO, reset_seq, sizeof(reset_seq) - 1) < 0)
    {
    }
    if (ansi__raw_active)
    {
        if (ansi__orig_flags >= 0)
        {
            fcntl(STDIN_FILENO, F_SETFL, ansi__orig_flags);
        }
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &ansi__orig_termios);
    }

    struct sigaction sa_dfl;
    sa_dfl.sa_handler = SIG_DFL;
    sigemptyset(&sa_dfl.sa_mask);
    sa_dfl.sa_flags = 0;
    sigaction(sig, &sa_dfl, NULL);
    raise(sig);
}

static void install_signal_handlers(void)
{
    struct sigaction sa;
    sa.sa_handler = tui_signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
}

/* =========================================================
 * Local ANSI print helpers
 * ========================================================= */

#define ansi_printfw(fmt, ...) printf(fmt, ##__VA_ARGS__)

static inline void ansi_newline(void)
{
    if (write(STDOUT_FILENO, "\r\n", 2) < 0)
    {
    }
}

static inline void ansi_clearscreen(void)
{
    if (write(STDOUT_FILENO, "\033[2J\033[H", 7) < 0)
    {
    }
}

static inline void ansi_get_termsize(unsigned short *rows, unsigned short *cols)
{
    struct winsize ws;

    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0 && ws.ws_col > 0)
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
 * Stream helpers (ImageStreamIO only)
 * ========================================================= */

static int connect_stream_fmt(IMAGE *im, int required, const char *fmt, int loopindex)
{
    char name[256];
    snprintf(name, sizeof(name), fmt, loopindex);
    memset(im, 0, sizeof(*im));
    if (ImageStreamIO_read_sharedmem_image_toIMAGE(name, im) != IMAGESTREAMIO_SUCCESS)
    {
        im->used = 0;
        if (required)
        {
            fprintf(stderr, "cacao-modalstatsTUI-bin: cannot open stream '%s'\n", name);
            return -1;
        }
        return 0;
    }
    return 0;
}

static int connect_or_create_2Df32(IMAGE      *im,
                                   uint32_t    nbmode,
                                   const char *fmt,
                                   int         loopindex)
{
    char name[256];
    snprintf(name, sizeof(name), fmt, loopindex);
    memset(im, 0, sizeof(*im));
    if (ImageStreamIO_read_sharedmem_image_toIMAGE(name, im) == IMAGESTREAMIO_SUCCESS)
    {
        return 0;
    }
    uint32_t size[2] = { nbmode, 1 };
    if (ImageStreamIO_createIm_gpu(im, name, 2, size, _DATATYPE_FLOAT, -1, 1, 10, 0, MATH_DATA, 0) !=
        IMAGESTREAMIO_SUCCESS)
    {
        im->used = 0;
        fprintf(stderr, "cacao-modalstatsTUI-bin: cannot create stream '%s'\n", name);
        return -1;
    }
    return 0;
}

static void close_stream(IMAGE *im)
{
    if (im->used)
    {
        ImageStreamIO_closeIm(im);
    }
}

/* =========================================================
 * Key handler
 * ========================================================= */

static int modalstats_TUI_process_user_key(int ch, MODALSTATSTRUCT *mstatstruct)
{
    int loopOK = 1;

    switch (ch)
    {
    case 'x':
    case 'q':
        loopOK = 0;
        break;

    case ANSI_KEY_UP:
        mstatstruct->modeindex--;
        if (mstatstruct->modeindex < 0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case ANSI_KEY_DOWN:
        mstatstruct->modeindex++;
        if (mstatstruct->modeindex > mstatstruct->NBmode - 1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode - 1;
        }
        break;

    case ANSI_KEY_PGUP:
        mstatstruct->modeindex -= 10;
        if (mstatstruct->modeindex < 0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case ANSI_KEY_PGDN:
        mstatstruct->modeindex += 10;
        if (mstatstruct->modeindex > mstatstruct->NBmode - 1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode - 1;
        }
        break;

    case '+':
        mstatstruct->pscaleindex++;
        mstatstruct->pscale = powf(10.0f, (float) mstatstruct->pscaleindex);
        break;

    case '-':
        mstatstruct->pscaleindex--;
        mstatstruct->pscale = powf(10.0f, (float) mstatstruct->pscaleindex);
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

    return loopOK;
}

/* =========================================================
 * Fixed-width value printers
 * ========================================================= */

static inline void printfixedlen(float val, const MODALSTATSTRUCT *mstatstruct)
{
    long tmpl = (long) (mstatstruct->pscale * val);

    if ((tmpl < 100000) && (tmpl > -100000))
    {
        ansi_printfw("%+5ld", tmpl);
    }
    else
    {
        ansi_setcolor(3);
        ansi_printfw("+++++");
        ansi_unsetcolor(3);
    }
}

static inline void printfixedlen_unsigned(float val, const MODALSTATSTRUCT *mstatstruct)
{
    long tmpl = (long) (mstatstruct->pscale * val);

    if ((tmpl < 100000) && (tmpl > -100000))
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

int AOloopControl_modalstatsTUI(int loopindex)
{
    IMAGE imDMmodes, imWFS, imDM, imDMf, imOL;
    IMAGE imGain, imMult, imLimit, imLimitFrac;
    IMAGE imDMave, imDMrms, imWFSave, imWFSrms, imOLave, imOLrms;
    IMAGE imPFmix, imPFresrms;

    memset(&imDMmodes, 0, sizeof(imDMmodes));
    memset(&imWFS, 0, sizeof(imWFS));
    memset(&imDM, 0, sizeof(imDM));
    memset(&imDMf, 0, sizeof(imDMf));
    memset(&imOL, 0, sizeof(imOL));
    memset(&imGain, 0, sizeof(imGain));
    memset(&imMult, 0, sizeof(imMult));
    memset(&imLimit, 0, sizeof(imLimit));
    memset(&imLimitFrac, 0, sizeof(imLimitFrac));
    memset(&imDMave, 0, sizeof(imDMave));
    memset(&imDMrms, 0, sizeof(imDMrms));
    memset(&imWFSave, 0, sizeof(imWFSave));
    memset(&imWFSrms, 0, sizeof(imWFSrms));
    memset(&imOLave, 0, sizeof(imOLave));
    memset(&imOLrms, 0, sizeof(imOLrms));
    memset(&imPFmix, 0, sizeof(imPFmix));
    memset(&imPFresrms, 0, sizeof(imPFresrms));

    MODALSTATSTRUCT mstatstruct;
    mstatstruct.modeindex   = 0;
    mstatstruct.pscaleindex = 0;
    mstatstruct.pscale      = 1.0f;

    connect_stream_fmt(&imDMmodes, 0, "aol%d_CMmodesDM", loopindex);
    if (connect_stream_fmt(&imWFS, 1, "aol%d_modevalWFS", loopindex) != 0 ||
        connect_stream_fmt(&imDM, 1, "aol%d_modevalDM", loopindex) != 0 ||
        connect_stream_fmt(&imDMf, 1, "aol%d_modevalDMf", loopindex) != 0 ||
        connect_stream_fmt(&imOL, 1, "aol%d_modevalOL", loopindex) != 0 ||
        connect_stream_fmt(&imGain, 1, "aol%d_mgain", loopindex) != 0 ||
        connect_stream_fmt(&imMult, 1, "aol%d_mmult", loopindex) != 0 ||
        connect_stream_fmt(&imLimit, 1, "aol%d_mlimit", loopindex) != 0 ||
        connect_stream_fmt(&imLimitFrac, 1, "aol%d_mlimitcntfrac", loopindex) != 0)
    {
        goto cleanup_streams;
    }

    uint32_t NBmode    = imWFS.md->size[0];
    mstatstruct.NBmode = (long) NBmode;

    if (connect_or_create_2Df32(&imDMave, NBmode, "aol%d_mvalDMave", loopindex) != 0 ||
        connect_or_create_2Df32(&imDMrms, NBmode, "aol%d_mvalDMrms", loopindex) != 0 ||
        connect_or_create_2Df32(&imWFSave, NBmode, "aol%d_mvalWFSave", loopindex) != 0 ||
        connect_or_create_2Df32(&imWFSrms, NBmode, "aol%d_mvalWFSrms", loopindex) != 0 ||
        connect_or_create_2Df32(&imOLave, NBmode, "aol%d_mvalOLave", loopindex) != 0 ||
        connect_or_create_2Df32(&imOLrms, NBmode, "aol%d_mvalOLrms", loopindex) != 0 ||
        connect_or_create_2Df32(&imPFmix, NBmode, "aol%d_mPFmix", loopindex) != 0 ||
        connect_or_create_2Df32(&imPFresrms, NBmode, "aol%d_mvalPFresrms", loopindex) != 0)
    {
        goto cleanup_streams;
    }

    double *WFSave     = calloc(NBmode, sizeof(double));
    double *WFSrms     = calloc(NBmode, sizeof(double));
    double *DMave      = calloc(NBmode, sizeof(double));
    double *DMrms      = calloc(NBmode, sizeof(double));
    double *OLave      = calloc(NBmode, sizeof(double));
    double *OLrms      = calloc(NBmode, sizeof(double));
    double *DMmodenorm = calloc(NBmode, sizeof(double));
    if (!WFSave || !WFSrms || !DMave || !DMrms || !OLave || !OLrms || !DMmodenorm)
    {
        free(WFSave);
        free(WFSrms);
        free(DMave);
        free(DMrms);
        free(OLave);
        free(OLrms);
        free(DMmodenorm);
        goto cleanup_streams;
    }

    if (!imDMmodes.used)
    {
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            DMmodenorm[mi] = 1.0;
        }
    }
    else
    {
        uint64_t npix = (uint64_t) imDMmodes.md->size[0] * imDMmodes.md->size[1];
        uint32_t nmodes_dm = imDMmodes.md->size[2];
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            if (mi < nmodes_dm && npix > 0)
            {
                double val = 0.0;
                for (uint64_t ii = 0; ii < npix; ii++)
                {
                    float v = imDMmodes.array.F[mi * npix + ii];
                    val += (double) v * (double) v;
                }
                DMmodenorm[mi] = sqrt(val / (double) npix);
            }
            else
            {
                DMmodenorm[mi] = 1.0;
            }
        }
    }

    install_signal_handlers();
    ansi_raw_mode_enter();

    unsigned short wrow = 24, wcol = 80;
    ansi_get_termsize(&wrow, &wcol);

    int      loopOK     = 1;
    long     loopcnt    = 0;
    int      mirange    = (int) (wrow > 5 ? wrow - 5 : 1);
    int      mimin      = 0;
    int      mimax      = mirange;
    long     mioffset   = 0;
    uint64_t buffindex0 = UINT64_MAX;

    while (loopOK == 1 && g_keep_running)
    {
        struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN, .revents = 0 };
        poll(&pfd, 1, 20);

        int ch = ansi_get_key();
        loopOK = modalstats_TUI_process_user_key(ch, &mstatstruct);

        ansi_get_termsize(&wrow, &wcol);
        mirange = (int) (wrow > 5 ? wrow - 5 : 1);

        ansi_clearscreen();

        if (mimax > mstatstruct.NBmode)
        {
            mimax = (int) mstatstruct.NBmode;
        }
        if (mstatstruct.modeindex > mimax - 10)
        {
            mioffset++;
            mimin = (int) mioffset;
            mimax = (int) (mioffset + mirange);
            if (mimax > mstatstruct.NBmode)
            {
                mioffset--;
                if (mioffset < 0)
                {
                    mioffset = 0;
                }
                mimin = (int) mioffset;
                mimax = (int) (mioffset + mirange);
            }
        }
        if (mimax > mstatstruct.NBmode)
        {
            mimax = (int) mstatstruct.NBmode;
        }
        if (mstatstruct.modeindex < mimin + 10)
        {
            mioffset--;
            mimin = (int) mioffset;
            mimax = (int) (mioffset + mirange);
            if (mimin < 0)
            {
                mioffset = 0;
                mimin    = 0;
                mimax    = mimin + mirange;
            }
        }
        if (mimax > mstatstruct.NBmode)
        {
            mimax = (int) mstatstruct.NBmode;
        }

        ansi_printfw(" PRESS x to exit, +/- change display scale, UP/DOWN PGUP PGDOWN");
        ansi_newline();
        ansi_printfw(" [P/p] Predictive filter  [F/f] DM filtering");
        ansi_newline();
        ansi_printfw("Loop %d  -  Mode %5ld / %5ld [%5d-%5d]  loopcnt %ld",
                     loopindex, mstatstruct.modeindex, mstatstruct.NBmode, mimin, mimax, loopcnt);
        ansi_newline();
        ansi_printfw("scale = %f", mstatstruct.pscale);
        ansi_newline();

        ansi_printfw("MODE [ gain  mult    lim  ]        WFS          |          DM       |");
        if (MODALTUI_DMfilt)
        {
            ansi_printfw("    DMf       |");
        }
        ansi_printfw("          OL       | LIMTRUC WFS/OL   DM/OL");
        if (MODALTUI_PF)
        {
            ansi_printfw("  [ mPFmix ]    res    res/WFS   res/pOL");
        }
        ansi_newline();

        uint64_t buffindex = imOLrms.md->cnt0;
        if (buffindex != buffindex0)
        {
            for (int32_t mii = mimin; mii < mimax; mii++)
            {
                double d;
                WFSave[mii] = imWFSave.array.F[mii];
                d           = (double) imWFSrms.array.F[mii] * imWFSrms.array.F[mii] -
                    WFSave[mii] * WFSave[mii];
                WFSrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;

                DMave[mii] = imDMave.array.F[mii];
                d          = (double) imDMrms.array.F[mii] * imDMrms.array.F[mii] -
                    DMave[mii] * DMave[mii];
                DMrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;

                OLave[mii] = imOLave.array.F[mii];
                d          = (double) imOLrms.array.F[mii] * imOLrms.array.F[mii] -
                    OLave[mii] * OLave[mii];
                OLrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;
            }
            buffindex0 = buffindex;
        }

        for (long mi = mimin; mi < mimax; mi++)
        {
            if (mi == mstatstruct.modeindex)
            {
                ansi_bold_on();
            }

            ansi_printfw("%4ld [%5.3f %5.3f %8.6f]   ", mi, imGain.array.F[mi],
                         imMult.array.F[mi], imLimit.array.F[mi]);

            printfixedlen(imWFS.array.F[mi] * (float) DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (WFSave[mi] > WFSrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen((float) (WFSave[mi] * DMmodenorm[mi]), &mstatstruct);
                ansi_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (WFSrms[mi] * DMmodenorm[mi]), &mstatstruct);
            ansi_printfw(" | ");

            printfixedlen(imDM.array.F[mi] * (float) DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (DMave[mi] > DMrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen((float) (DMave[mi] * DMmodenorm[mi]), &mstatstruct);
                ansi_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (DMrms[mi] * DMmodenorm[mi]), &mstatstruct);
            ansi_printfw(" | ");

            if (MODALTUI_DMfilt)
            {
                printfixedlen(imDMf.array.F[mi] * (float) DMmodenorm[mi], &mstatstruct);
                ansi_printfw(" ");
                printfixedlen((imDMf.array.F[mi] - imDM.array.F[mi]) * (float) DMmodenorm[mi],
                              &mstatstruct);
                ansi_printfw("  | ");
            }

            printfixedlen(imOL.array.F[mi] * (float) DMmodenorm[mi], &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (OLave[mi] > OLrms[mi]) ? 3 : 0;
                ansi_setcolor(color);
                printfixedlen((float) (OLave[mi] * DMmodenorm[mi]), &mstatstruct);
                ansi_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (OLrms[mi] * DMmodenorm[mi]), &mstatstruct);
            ansi_printfw(" | ");

            {
                float truncfract = imLimitFrac.array.F[mi];
                int   color      = 0;
                if (truncfract > 0.01f)
                {
                    color = 3;
                }
                if (truncfract > 0.1f)
                {
                    color = 4;
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", truncfract);
                ansi_unsetcolor(color);
                ansi_printfw("  ");
            }

            {
                float WFSoverOL = (OLrms[mi] > 0.0) ? (float) (WFSrms[mi] / OLrms[mi]) : 0.0f;
                float DMoverOL  = (OLrms[mi] > 0.0) ? (float) (DMrms[mi] / OLrms[mi]) : 0.0f;
                int   color     = 0;
                if (WFSoverOL < 0.9f)
                {
                    color = 2;
                }
                if (WFSoverOL > 1.0f)
                {
                    color = 4;
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", WFSoverOL);
                ansi_unsetcolor(color);
                ansi_printfw("  ");

                color = 0;
                if (DMoverOL > 0.5f)
                {
                    color = 2;
                }
                if (DMoverOL > 1.0f)
                {
                    color = 4;
                }
                ansi_setcolor(color);
                ansi_printfw("%6.4f", DMoverOL);
                ansi_unsetcolor(color);
            }

            if (MODALTUI_PF)
            {
                float pfres = imPFresrms.array.F[mi];
                float wfsr  = imWFSrms.array.F[mi];
                float olr   = imOLrms.array.F[mi];
                ansi_printfw("  [ %6.4f ]", imPFmix.array.F[mi]);
                ansi_printfw("   %6.4f ", pfres);
                ansi_printfw("  %5.3f ", (wfsr != 0.0f) ? pfres / wfsr : 0.0f);
                ansi_printfw("  %8.6f ", (olr != 0.0f) ? pfres / olr : 0.0f);
            }

            ansi_newline();
            if (mi == mstatstruct.modeindex)
            {
                ansi_bold_off();
            }
        }

        fflush(stdout);
        loopcnt++;
    }

    ansi_raw_mode_exit();

    free(WFSave);
    free(WFSrms);
    free(DMave);
    free(DMrms);
    free(OLave);
    free(OLrms);
    free(DMmodenorm);

cleanup_streams:
    close_stream(&imDMmodes);
    close_stream(&imWFS);
    close_stream(&imDM);
    close_stream(&imDMf);
    close_stream(&imOL);
    close_stream(&imGain);
    close_stream(&imMult);
    close_stream(&imLimit);
    close_stream(&imLimitFrac);
    close_stream(&imDMave);
    close_stream(&imDMrms);
    close_stream(&imWFSave);
    close_stream(&imWFSrms);
    close_stream(&imOLave);
    close_stream(&imOLrms);
    close_stream(&imPFmix);
    close_stream(&imPFresrms);
    return imWFS.used ? 0 : 1;
}

static void print_usage(const char *progname, int mh_color)
{
    printf("%s %s (commit %s)\n", MH(MH_TITLE, "Usage:"), progname, MILK_GIT_COMMIT);
    printf("  %s [loopindex]\n", progname);
    printf("\n%s\n", MH(MH_HDR, "Options:"));
    printf("  " MH_PAD_FMT "Display this help\n", MH_PAD(MH_OPT, "-h, --help", 18));
    printf("  " MH_PAD_FMT "One-line description\n", MH_PAD(MH_OPT, "-h1", 18));
    printf("  " MH_PAD_FMT "Monochrome help\n", MH_PAD(MH_OPT, "-hm", 18));
    printf("\nDefaults loopindex to $CACAO_LOOPNUMBER (or 0).\n");
}

int main(int argc, char *argv[])
{
    int action = milk_help_init(argc, argv, "interactive ANSI TUI for modal AO control statistics",
                                "Real-time terminal display of per-mode WFS, DM, open-loop, "
                                "gain/mult/limit, and predictive-filter statistics.");
    if (action == MH_ACTION_H1 || action == MH_ACTION_H2)
    {
        return 0;
    }
    if (action == MH_ACTION_HELP || action == MH_ACTION_MONO)
    {
        print_usage(argv[0], action == MH_ACTION_HELP);
        return 0;
    }

    int loopindex = 0;
    const char *env_loop = getenv("CACAO_LOOPNUMBER");
    if (env_loop && env_loop[0] != '\0')
    {
        loopindex = atoi(env_loop);
    }
    if (argc >= 2 && argv[1][0] != '-')
    {
        loopindex = atoi(argv[1]);
    }

    return AOloopControl_modalstatsTUI(loopindex);
}
