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
#include <stdarg.h>
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
 * Atomic single-syscall ANSI frame buffer
 * ========================================================= */

#define TUI_FRAMEBUF_SIZE (256 * 1024)

static char   g_framebuf[TUI_FRAMEBUF_SIZE];
static size_t g_framebuf_len = 0;

static inline void fb_reset(void)
{
    g_framebuf_len = 0;
}

static inline void fb_append_buf(const char *s, size_t n)
{
    if (n == 0 || g_framebuf_len >= TUI_FRAMEBUF_SIZE - 1)
    {
        return;
    }
    size_t avail = (TUI_FRAMEBUF_SIZE - 1) - g_framebuf_len;
    if (n > avail)
    {
        n = avail;
    }
    __builtin_memcpy(g_framebuf + g_framebuf_len, s, n);
    g_framebuf_len += n;
}

#define fb_append_lit(lit) fb_append_buf((lit), sizeof(lit) - 1)

__attribute__((format(printf, 1, 2))) static inline void ansi_printfw(const char *fmt, ...)
{
    if (g_framebuf_len >= TUI_FRAMEBUF_SIZE - 1)
    {
        return;
    }
    size_t  avail = TUI_FRAMEBUF_SIZE - g_framebuf_len;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(g_framebuf + g_framebuf_len, avail, fmt, ap);
    va_end(ap);
    if (n > 0)
    {
        size_t written = (size_t) n;
        if (written >= avail)
        {
            written = avail - 1;
        }
        g_framebuf_len += written;
    }
}

static inline void ansi_newline(void)
{
    fb_append_lit("\033[K\r\n");
}

static inline void fb_home(void)
{
    fb_append_lit("\033[H");
}

static inline void fb_bold_on(void)
{
    fb_append_lit("\033[1m");
}

static inline void fb_bold_off(void)
{
    fb_append_lit("\033[22m");
}

static inline void fb_setcolor(int idx)
{
    if (idx == 0)
    {
        return;
    }
    ansi_detect_color_level();
    if (ansi__color_level >= 3)
    {
        switch (idx)
        {
        case 2:
            fb_append_lit("\033[38;2;80;220;80m");
            break;
        case 3:
            fb_append_lit("\033[38;2;220;200;0m");
            break;
        case 4:
            fb_append_lit("\033[38;2;240;60;60m");
            break;
        case 5:
            fb_append_lit("\033[38;2;200;80;220m");
            break;
        case 7:
            fb_append_lit("\033[38;2;0;200;220m");
            break;
        case 9:
            fb_append_lit("\033[38;2;255;140;0m");
            break;
        case 12:
            fb_append_lit("\033[38;2;100;255;100m");
            break;
        default:
            break;
        }
    }
    else if (ansi__color_level == 2)
    {
        switch (idx)
        {
        case 2:
            fb_append_lit("\033[38;5;114m");
            break;
        case 3:
            fb_append_lit("\033[38;5;220m");
            break;
        case 4:
            fb_append_lit("\033[38;5;203m");
            break;
        case 5:
            fb_append_lit("\033[38;5;176m");
            break;
        case 7:
            fb_append_lit("\033[38;5;44m");
            break;
        case 9:
            fb_append_lit("\033[38;5;208m");
            break;
        case 12:
            fb_append_lit("\033[38;5;119m");
            break;
        default:
            break;
        }
    }
    else
    {
        switch (idx)
        {
        case 2:
            fb_append_lit("\033[32m");
            break;
        case 3:
            fb_append_lit("\033[33m");
            break;
        case 4:
            fb_append_lit("\033[31m");
            break;
        case 5:
            fb_append_lit("\033[35m");
            break;
        case 7:
            fb_append_lit("\033[36m");
            break;
        case 9:
            fb_append_lit("\033[33m");
            break;
        case 12:
            fb_append_lit("\033[92m");
            break;
        default:
            break;
        }
    }
}

static inline void fb_unsetcolor(int idx)
{
    if (idx != 0)
    {
        fb_append_lit("\033[39m");
    }
}

static inline void fb_flush(void)
{
    fb_append_lit("\033[J");
    size_t off = 0;
    while (off < g_framebuf_len)
    {
        ssize_t w = write(STDOUT_FILENO, g_framebuf + off, g_framebuf_len - off);
        if (w < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            break;
        }
        off += (size_t) w;
    }
    g_framebuf_len = 0;
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

static int connect_loop_stream(IMAGE *im, int required, int loopindex, const char *sfx)
{
    char name[256];
    snprintf(name, sizeof(name), "aol%d_%s", loopindex, sfx);
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

static int connect_or_create_2Df32(IMAGE *im, uint32_t nbmode, int loopindex, const char *sfx)
{
    char name[256];
    snprintf(name, sizeof(name), "aol%d_%s", loopindex, sfx);
    memset(im, 0, sizeof(*im));
    if (ImageStreamIO_read_sharedmem_image_toIMAGE(name, im) == IMAGESTREAMIO_SUCCESS)
    {
        return 0;
    }
    uint32_t size[2] = { nbmode, 1 };
    if (ImageStreamIO_createIm_gpu(im, name, 2, size, _DATATYPE_FLOAT, -1, 1, 10, 0, MATH_DATA,
                                   0) != IMAGESTREAMIO_SUCCESS)
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
        fb_setcolor(3);
        ansi_printfw("+++++");
        fb_unsetcolor(3);
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
        fb_setcolor(3);
        ansi_printfw("+++++");
        fb_unsetcolor(3);
    }
}

/* =========================================================
 * Main TUI function
 * ========================================================= */

int AOloopControl_modalstatsTUI(int loopindex)
{
    int   ret = 1;
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

    connect_loop_stream(&imDMmodes, 0, loopindex, "CMmodesDM");
    if (connect_loop_stream(&imWFS, 1, loopindex, "modevalWFS") != 0 ||
        connect_loop_stream(&imDM, 1, loopindex, "modevalDM") != 0 ||
        connect_loop_stream(&imDMf, 1, loopindex, "modevalDMf") != 0 ||
        connect_loop_stream(&imOL, 1, loopindex, "modevalOL") != 0 ||
        connect_loop_stream(&imGain, 1, loopindex, "mgain") != 0 ||
        connect_loop_stream(&imMult, 1, loopindex, "mmult") != 0 ||
        connect_loop_stream(&imLimit, 1, loopindex, "mlimit") != 0 ||
        connect_loop_stream(&imLimitFrac, 1, loopindex, "mlimitcntfrac") != 0)
    {
        goto cleanup_streams;
    }

    uint32_t NBmode    = imWFS.md->size[0];
    mstatstruct.NBmode = (long) NBmode;

    if (connect_or_create_2Df32(&imDMave, NBmode, loopindex, "mvalDMave") != 0 ||
        connect_or_create_2Df32(&imDMrms, NBmode, loopindex, "mvalDMrms") != 0 ||
        connect_or_create_2Df32(&imWFSave, NBmode, loopindex, "mvalWFSave") != 0 ||
        connect_or_create_2Df32(&imWFSrms, NBmode, loopindex, "mvalWFSrms") != 0 ||
        connect_or_create_2Df32(&imOLave, NBmode, loopindex, "mvalOLave") != 0 ||
        connect_or_create_2Df32(&imOLrms, NBmode, loopindex, "mvalOLrms") != 0 ||
        connect_or_create_2Df32(&imPFmix, NBmode, loopindex, "mPFmix") != 0 ||
        connect_or_create_2Df32(&imPFresrms, NBmode, loopindex, "mvalPFresrms") != 0)
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
        const float *restrict dmmodes = imDMmodes.array.F;
        uint64_t npix                 = (uint64_t) imDMmodes.md->size[0] * imDMmodes.md->size[1];
        uint32_t nmodes_dm            = imDMmodes.md->size[2];
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            if (mi < nmodes_dm && npix > 0)
            {
                const float *restrict mode_ptr = dmmodes + mi * npix;
                double val                     = 0.0;
                for (uint64_t ii = 0; ii < npix; ii++)
                {
                    double v = (double) mode_ptr[ii];
                    val += v * v;
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
    if (write(STDOUT_FILENO, "\033[2J\033[H", 7) < 0)
    {
    }

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

        fb_reset();
        fb_home();

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
        ansi_printfw("Loop %d  -  Mode %5ld / %5ld [%5d-%5d]  loopcnt %ld", loopindex,
                     mstatstruct.modeindex, mstatstruct.NBmode, mimin, mimax, loopcnt);
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
            const float *restrict wfs_ave_arr = imWFSave.array.F;
            const float *restrict wfs_rms_arr = imWFSrms.array.F;
            const float *restrict dm_ave_arr  = imDMave.array.F;
            const float *restrict dm_rms_arr  = imDMrms.array.F;
            const float *restrict ol_ave_arr  = imOLave.array.F;
            const float *restrict ol_rms_arr  = imOLrms.array.F;

            for (int32_t mii = mimin; mii < mimax; mii++)
            {
                double d;
                WFSave[mii] = wfs_ave_arr[mii];
                d = (double) wfs_rms_arr[mii] * wfs_rms_arr[mii] - WFSave[mii] * WFSave[mii];
                WFSrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;

                DMave[mii] = dm_ave_arr[mii];
                d          = (double) dm_rms_arr[mii] * dm_rms_arr[mii] - DMave[mii] * DMave[mii];
                DMrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;

                OLave[mii] = ol_ave_arr[mii];
                d          = (double) ol_rms_arr[mii] * ol_rms_arr[mii] - OLave[mii] * OLave[mii];
                OLrms[mii] = (d > 0.0) ? sqrt(d) : 0.0;
            }
            buffindex0 = buffindex;
        }

        const float *restrict gain_arr    = imGain.array.F;
        const float *restrict mult_arr    = imMult.array.F;
        const float *restrict lim_arr     = imLimit.array.F;
        const float *restrict limfrac_arr = imLimitFrac.array.F;
        const float *restrict wfs_arr     = imWFS.array.F;
        const float *restrict dm_arr      = imDM.array.F;
        const float *restrict dmf_arr     = imDMf.array.F;
        const float *restrict ol_arr      = imOL.array.F;
        const float *restrict pfmix_arr   = imPFmix.array.F;
        const float *restrict pfres_arr   = imPFresrms.array.F;
        const float *restrict wfsrms_arr  = imWFSrms.array.F;
        const float *restrict olrms_arr   = imOLrms.array.F;

        for (long mi = mimin; mi < mimax; mi++)
        {
            double mnorm  = DMmodenorm[mi];
            float  mnormf = (float) mnorm;

            if (mi == mstatstruct.modeindex)
            {
                fb_bold_on();
            }

            ansi_printfw("%4ld [%5.3f %5.3f %8.6f]   ", mi, gain_arr[mi], mult_arr[mi],
                         lim_arr[mi]);

            printfixedlen(wfs_arr[mi] * mnormf, &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (WFSave[mi] > WFSrms[mi]) ? 3 : 0;
                fb_setcolor(color);
                printfixedlen((float) (WFSave[mi] * mnorm), &mstatstruct);
                fb_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (WFSrms[mi] * mnorm), &mstatstruct);
            ansi_printfw(" | ");

            printfixedlen(dm_arr[mi] * mnormf, &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (DMave[mi] > DMrms[mi]) ? 3 : 0;
                fb_setcolor(color);
                printfixedlen((float) (DMave[mi] * mnorm), &mstatstruct);
                fb_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (DMrms[mi] * mnorm), &mstatstruct);
            ansi_printfw(" | ");

            if (MODALTUI_DMfilt)
            {
                printfixedlen(dmf_arr[mi] * mnormf, &mstatstruct);
                ansi_printfw(" ");
                printfixedlen((dmf_arr[mi] - dm_arr[mi]) * mnormf, &mstatstruct);
                ansi_printfw("  | ");
            }

            printfixedlen(ol_arr[mi] * mnormf, &mstatstruct);
            ansi_printfw(" ");
            {
                int color = (OLave[mi] > OLrms[mi]) ? 3 : 0;
                fb_setcolor(color);
                printfixedlen((float) (OLave[mi] * mnorm), &mstatstruct);
                fb_unsetcolor(color);
            }
            ansi_printfw(" ");
            printfixedlen_unsigned((float) (OLrms[mi] * mnorm), &mstatstruct);
            ansi_printfw(" | ");

            {
                float truncfract = limfrac_arr[mi];
                int   color      = 0;
                if (truncfract > 0.01f)
                {
                    color = 3;
                }
                if (truncfract > 0.1f)
                {
                    color = 4;
                }
                fb_setcolor(color);
                ansi_printfw("%6.4f", truncfract);
                fb_unsetcolor(color);
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
                fb_setcolor(color);
                ansi_printfw("%6.4f", WFSoverOL);
                fb_unsetcolor(color);
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
                fb_setcolor(color);
                ansi_printfw("%6.4f", DMoverOL);
                fb_unsetcolor(color);
            }

            if (MODALTUI_PF)
            {
                float pfres = pfres_arr[mi];
                float wfsr  = wfsrms_arr[mi];
                float olr   = olrms_arr[mi];
                ansi_printfw("  [ %6.4f ]", pfmix_arr[mi]);
                ansi_printfw("   %6.4f ", pfres);
                ansi_printfw("  %5.3f ", (wfsr != 0.0f) ? pfres / wfsr : 0.0f);
                ansi_printfw("  %8.6f ", (olr != 0.0f) ? pfres / olr : 0.0f);
            }

            if (mi == mstatstruct.modeindex)
            {
                fb_bold_off();
            }
            ansi_newline();
        }

        fb_flush();
        loopcnt++;
    }

    ansi_raw_mode_exit();
    ret = 0;

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
    return ret;
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

    int         loopindex = 0;
    const char *env_loop  = getenv("CACAO_LOOPNUMBER");
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
