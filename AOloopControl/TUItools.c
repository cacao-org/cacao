/**
 * @file    TUItools.c
 * @brief   Text User Interface tools
 */

#define TUITOOLS_C

#include <sys/ioctl.h> // for terminal size
#include <termios.h>

#ifdef USE_NCURSES
#include <ncurses.h>
//#include <curses.h>
#include <ncursesw/ncurses.h>
#else
// Define some ncurses constants if not using ncurses
#define KEY_UP    0403
#define KEY_DOWN  0402
#define KEY_LEFT  0404
#define KEY_RIGHT 0405
#define KEY_F(n)  (0410+(n))
#endif

#include <locale.h>
#include <wchar.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include "TUItools.h"

#ifndef RETURN_SUCCESS
#define RETURN_SUCCESS 0
#endif

#ifndef DEBUG_TRACEPOINT
#define DEBUG_TRACEPOINT(...)
#endif

#ifndef DEBUG_TRACE_FSTART
#define DEBUG_TRACE_FSTART(...)
#endif

#ifndef DEBUG_TRACE_FEXIT
#define DEBUG_TRACE_FEXIT(...)
#endif

static struct winsize     w;
static short unsigned int wrow, wcol;
static int                wresizecnt = 0;

// Current column position for line truncation
static int curcol = 0;

/*
 * Defines printfw output
 *
 * SCREENPRINT_STDIO     printf to stdout
 * SCREENPRINT_NCURSES   printw
 * SCREENPRINT_NONE      don't print (silent)
 */

static int screenprintmode = SCREENPRINT_STDIO;

struct termios orig_termios;
struct termios new_termios;

static int printAEC = 0;

// Foreground color
static int printAECfgcolor = AEC_FGCOLOR_WHITE;

// Background color
static int printAECbgcolor = AEC_BGCOLOR_BLACK;

void TUI_set_screenprintmode(int mode)
{
    screenprintmode = mode;
}

int TUI_get_screenprintmode()
{
    return screenprintmode;
}

/**
 * @brief print to stdout or through ncurses
 *
 * If screenprintmode :\n
 * is SCREENPRINT_STDIO, use stdio\n
 * is SCREENPRINT_NCURSES, use ncurses\n
 *
 * @param fmt
 * @param ...
 */
void TUI_printfw(const char *fmt, ...)
{
    // Skip if already past terminal width
    if(wcol > 1 && curcol >= (wcol - 1))
    {
        return;
    }

    va_list args;
    char buf[1024];

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if(len < 0)
    {
        return;
    }
    if(len >= (int) sizeof(buf))
    {
        len = (int) sizeof(buf) - 1;
    }

    // Truncate to fit within terminal width.
    // Use wcol-1 to avoid writing the last column,
    // which causes ncurses to wrap the cursor.
    int avail = len;
    if(wcol > 1)
    {
        int remaining = (wcol - 1) - curcol;
        if(remaining <= 0)
        {
            return;
        }
        if(avail > remaining)
        {
            avail = remaining;
        }
    }

    if(screenprintmode == SCREENPRINT_STDIO)
    {
        fwrite(buf, 1, avail, stdout);
    }

#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        addnstr(buf, avail);
    }
#endif

    // Update curcol, resetting on newlines
    for(int i = 0; i < avail; i++)
    {
        if(buf[i] == '\n')
        {
            curcol = 0;
        }
        else
        {
            curcol++;
        }
    }
}


void TUI_newline()
{
    if(screenprintmode == SCREENPRINT_STDIO)
    {
        printf("\n");
    }
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        clrtoeol();
        printw("\n");
    }
#endif
    curcol = 0;
}


void screenprint_setcolor(int colorcode)
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attron(COLOR_PAIR(colorcode));
    }
    else
#endif
    {
        switch(colorcode)
        {
            case 1:
                printAECfgcolor = AEC_FGCOLOR_WHITE;
                printAECbgcolor = AEC_BGCOLOR_BLACK;
                break;

            case 2:
                printAECfgcolor = AEC_FGCOLOR_BLACK;
                printAECbgcolor = AEC_BGCOLOR_GREEN;
                break;

            case 3:
                printAECfgcolor = AEC_FGCOLOR_BLACK;
                printAECbgcolor = AEC_BGCOLOR_YELLOW;
                break;

            case 4:
                printAECfgcolor = AEC_FGCOLOR_WHITE;
                printAECbgcolor = AEC_BGCOLOR_RED;
                break;

            case 5:
                printAECfgcolor = AEC_FGCOLOR_WHITE;
                printAECbgcolor = AEC_BGCOLOR_BLUE;
                break;

            case 6:
                printAECfgcolor = AEC_FGCOLOR_BLACK;
                printAECbgcolor = AEC_BGCOLOR_GREEN;
                break;

            case 7:
                printAECfgcolor = AEC_FGCOLOR_WHITE;
                printAECbgcolor = AEC_BGCOLOR_YELLOW;
                break;

            case 8:
                printAECfgcolor = AEC_FGCOLOR_BLACK;
                printAECbgcolor = AEC_BGCOLOR_RED;
                break;

            case 9:
                printAECfgcolor = AEC_FGCOLOR_RED;
                printAECbgcolor = AEC_BGCOLOR_BLACK;
                break;

            case 10:
                printAECfgcolor = AEC_FGCOLOR_BLACK;
                printAECbgcolor = AEC_BGCOLOR_BLUE + 60;
                break;

            case 13:
                printAECfgcolor = AEC_FGCOLOR_WHITE;
                printAECbgcolor = AEC_BGCOLOR_GREEN;
                break;
        }

        printf("\033[%d;%dm", printAECfgcolor, printAECbgcolor);
    }
}

void screenprint_unsetcolor(int colorcode)
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attroff(COLOR_PAIR(colorcode));
    }
    else
#endif
    {
        printAEC        = AEC_NORMAL;
        printAECfgcolor = AEC_FGCOLOR_WHITE;
        printAECbgcolor = AEC_BGCOLOR_BLACK;
        printf("\033[%dm", printAEC); //, printAECbgcolor);
    }
}

void screenprint_setbold()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attron(A_BOLD);
    }
    else
#endif
    {
        printAEC = AEC_BOLD;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_unsetbold()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attroff(A_BOLD);
    }
    else
#endif
    {
        printAEC = AEC_NORMAL; //AEC_BOLDOFF;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_setblink()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attron(A_BLINK);
    }
    else
#endif
    {
        printAEC = AEC_FASTBLINK;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_unsetblink()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attroff(A_BLINK);
    }
    else
#endif
    {
        printAEC = AEC_NORMAL; //AEC_BLINKOFF;
        printf("\033[%dm", AEC_NORMAL);
    }
}

void screenprint_setdim()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attron(A_DIM);
    }
    else
#endif
    {
        printAEC = AEC_FAINT;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_unsetdim()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attroff(A_DIM);
    }
    else
#endif
    {
        printAEC = AEC_NORMAL; //AEC_FAINTOFF;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_setreverse()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attron(A_REVERSE);
    }
    else
#endif
    {
        printAEC = AEC_REVERSE;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_unsetreverse()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        attroff(A_REVERSE);
    }
    else
#endif
    {
        printAEC = AEC_NORMAL; //AEC_REVERSEOFF;
        printf("\033[%dm", printAEC);
    }
}

void screenprint_setnormal()
{
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        //attron(A_REVERSE);
    }
    else
    {
        printAEC        = AEC_NORMAL;
        printAECfgcolor = AEC_FGCOLOR_WHITE;
        printAECbgcolor = AEC_BGCOLOR_BLACK;
        printf("\033[%d;%d;%dm", printAEC, printAECfgcolor, printAECbgcolor);
    }
}


/**
 * @brief Print header line
 *
 * @param str     content string
 * @param c       filler character to be printed on either side of content
 * @return errno_t
 */
errno_t TUI_print_header(const char *str, char c)
{
    long n = strlen(str);

    screenprint_setbold();

    int strl = wcol - 1;
    if(n > wcol)
    {
        strl = n + 1;
    }
    char linestring[strl];
    int  spos = 0;

    for(long i = 0; i < (wcol - n) / 2; i++)
    {
        linestring[spos] = c;
        spos++;
    }

    for(size_t i = 0; i < strlen(str); i++)
    {
        linestring[spos] = str[i];
        spos++;
    }

    for(long i = 0; i < (wcol - n) / 2 - 1; i++)
    {
        linestring[spos] = c;
        spos++;
    }

    linestring[spos] = '\0';
    TUI_printfw("%s", linestring);

    TUI_newline();
    screenprint_unsetbold();

    return RETURN_SUCCESS;
}

/** @brief restore terminal settings
 */
void TUI_reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

errno_t TUI_inittermios(short unsigned int *wrowptr,
                        short unsigned int *wcolptr)
{
    tcgetattr(0, &orig_termios);

    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    //cfmakeraw(&new_termios);
    new_termios.c_lflag &= ~ICANON;
    new_termios.c_lflag &= ~ECHO;
    new_termios.c_lflag &= ~ISIG;
    new_termios.c_cc[VMIN]  = 0;
    new_termios.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_termios);

    // get terminal size
    struct winsize w;
    memset(&w, 0, sizeof(w));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == -1)
    {
        w.ws_row = 24;
        w.ws_col = 80;
    }

    if(w.ws_row == 0) w.ws_row = 24;
    if(w.ws_col == 0) w.ws_col = 80;

    wrow = w.ws_row;
    wcol = w.ws_col;

    *wrowptr = wrow;
    *wcolptr = wcol;

    atexit(TUI_reset_terminal_mode);

    return RETURN_SUCCESS;
}

void TUI_clearscreen(short unsigned int *wrowptr, short unsigned int *wcolptr)
{
    curcol = 0;

    if(screenprintmode == SCREENPRINT_STDIO)  // stdio mode
    {
        printf("\e[1;1H\e[2J");
        //printf("[%12lld  %d %d %d ]  ", loopcnt, buffd[0], buffd[1], buffd[2]);

        // update terminal size
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

        *wrowptr = w.ws_row;
        *wcolptr = w.ws_col;
    }
    else
    {
        (void) *wrowptr;
        (void) *wcolptr;
    }
}

#ifdef USE_NCURSES
void TUI_handle_winch(int sig)
{
    wresizecnt++;
    DEBUG_TRACEPOINT("wresizecnt = %d", wresizecnt);
    (void) sig;

    endwin();

    // Needs to be called after an endwin() so ncurses will initialize
    // itself with the new terminal dimensions.
    refresh();

    clear();
    wrow = LINES;
    wcol = COLS;

    DEBUG_TRACEPOINT("window size %d %d", wrow, wcol);

    refresh();
}


/** @brief INITIALIZE ncurses
 *
 */
errno_t TUI_initncurses(short unsigned int *wrowptr,
                        short unsigned int *wcolptr)
{
    DEBUG_TRACE_FSTART();

    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        DEBUG_TRACEPOINT("Initializing TUI ncurses ");

        setlocale(LC_ALL, "");
        if(initscr() == NULL)
        {
            fprintf(stderr, "Error initialising ncurses.\n");
            exit(EXIT_FAILURE);
        }
        DEBUG_TRACEPOINT("Initializing TUI ncurses ");

        getmaxyx(stdscr, wrow, wcol); /* get the number of rows and columns */

        DEBUG_TRACEPOINT("wrow wcol = %d %d", wrow, wcol);

        *wrowptr = wrow;
        *wcolptr = wcol;
        DEBUG_TRACEPOINT("wrow wcol = %d %d", wrow, wcol);

        cbreak();
        // disables line buffering and erase/kill character-processing (interrupt and flow control characters are unaffected),
        // making characters typed by the user immediately available to the program

        DEBUG_TRACEPOINT(" ");

        keypad(stdscr, TRUE);
        // enable F1, F2 etc..

        DEBUG_TRACEPOINT(" ");
        nodelay(stdscr, TRUE);
        curs_set(0);

        DEBUG_TRACEPOINT(" ");
        noecho();
        // Don't echo() while we do getch

        //nonl();
        // Do not translates newline into return and line-feed on output

        DEBUG_TRACEPOINT(" ");
        //init_color(COLOR_GREEN, 400, 1000, 400);
        //init_color(COLOR_GREEN, 700, 1000, 700);
        //init_color(COLOR_YELLOW, 1000, 1000, 700);
        start_color();
        DEBUG_TRACEPOINT(" ");

        //  colored background
        init_pair(1, COLOR_BLACK, COLOR_WHITE);
        init_pair(2, COLOR_BLACK, COLOR_GREEN);  // all good
        init_pair(3, COLOR_BLACK, COLOR_YELLOW); // parameter out of sync
        init_pair(4, COLOR_WHITE, COLOR_RED);
        init_pair(5, COLOR_WHITE, COLOR_BLUE); // DIRECTORY
        init_pair(6, COLOR_GREEN, COLOR_BLACK);
        init_pair(7, COLOR_YELLOW, COLOR_BLACK);
        init_pair(8, COLOR_RED, COLOR_BLACK);
        init_pair(9, COLOR_BLACK, COLOR_RED);
        init_pair(10, COLOR_BLACK, COLOR_CYAN);
        init_pair(12, COLOR_GREEN,
                  COLOR_WHITE); // highlighted version of #2
        init_pair(13, COLOR_WHITE, COLOR_GREEN); // White on Green

        // handle window resize
        /*
        struct sigaction sa;
        memset(&sa, 0, sizeof(struct sigaction));
        sa.sa_handler = TUI_handle_winch;
        sigaction(SIGWINCH, &sa, NULL);
        */
    }

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}
#endif


errno_t TUI_init_terminal(short unsigned int *wrowptr,
                          short unsigned int *wcolptr)
{
    DEBUG_TRACE_FSTART();
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)  // ncurses mode
    {
        TUI_initncurses(wrowptr, wcolptr);
        DEBUG_TRACEPOINT("init terminal ncurses mode %d %d",
                         *wrowptr,
                         *wcolptr);
        atexit(TUI_atexit);
        clear();
    }
    else
#endif
    {
        TUI_inittermios(wrowptr, wcolptr);
        DEBUG_TRACEPOINT("init terminal stdio mode %d %d", *wrowptr, *wcolptr);
    }
    
    // Final assignment to ensure pointers are updated
    *wrowptr = wrow;
    *wcolptr = wcol;
    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifdef USE_NCURSES
errno_t TUI_get_terminal_size(short unsigned int *wrowptr,
                              short unsigned int *wcolptr)
{
    *wrowptr = wrow;
    *wcolptr = wcol;

    return RETURN_SUCCESS;
}
#endif

errno_t TUI_exit()
{
#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        endwin();
    }
#endif

    return RETURN_SUCCESS;
}

void TUI_atexit()
{
    //printf("exiting CTRLscreen\n");

#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        endwin();
    }
#endif
}

#ifdef USE_NCURSES
errno_t TUI_ncurses_refresh()
{
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        refresh();
    }

    return RETURN_SUCCESS;
}

errno_t TUI_ncurses_erase()
{
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        erase();
    }

    return RETURN_SUCCESS;
}
#endif

errno_t TUI_stdio_clear()
{
    if(screenprintmode == SCREENPRINT_STDIO)
    {
        printf("\e[1;1H\e[2J");
    }

    return RETURN_SUCCESS;
}

int get_singlechar_nonblock()
{
    static char stdio_buffer[64];
    static int stdio_buf_len = 0;
    static int stdio_buf_pos = 0;

    int ch = -1;

#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        ch = getch(); // ncurses function, non-blocking
    }
    else
#endif
    {
        if (stdio_buf_pos >= stdio_buf_len)
        {
             stdio_buf_pos = 0;
             stdio_buf_len = read(STDIN_FILENO, stdio_buffer, 64);
             if (stdio_buf_len <= 0)
             {
                 stdio_buf_len = 0;
                 return -1;
             }
        }

        ch = stdio_buffer[stdio_buf_pos];

        if (ch == 13) // Enter
        {
            ch = 10;
            stdio_buf_pos++;
            return ch;
        }

        if (ch == 27) // Escape
        {
            int remaining = stdio_buf_len - stdio_buf_pos;
            
            if (remaining >= 3)
            {
                char c1 = stdio_buffer[stdio_buf_pos+1];
                char c2 = stdio_buffer[stdio_buf_pos+2];

                if (c1 == 91) // [
                {
                    switch(c2)
                    {
                        case 'A': ch = KEY_UP; stdio_buf_pos+=3; return ch;
                        case 'B': ch = KEY_DOWN; stdio_buf_pos+=3; return ch;
                        case 'C': ch = KEY_RIGHT; stdio_buf_pos+=3; return ch;
                        case 'D': ch = KEY_LEFT; stdio_buf_pos+=3; return ch;
                    }
                    
                    // Check for CTRL+Arrow (needs 6 bytes)
                    if (remaining >= 6)
                    {
                        if (c2 == '1' && stdio_buffer[stdio_buf_pos+3] == ';' && stdio_buffer[stdio_buf_pos+4] == '5')
                        {
                            char c5 = stdio_buffer[stdio_buf_pos+5];
                            if (c5 == 'C') { // CTRL+RIGHT
                                ch = 561; stdio_buf_pos+=6; return ch;
                            }
                            if (c5 == 'D') { // CTRL+LEFT
                                ch = 545; stdio_buf_pos+=6; return ch;
                            }
                        }
                    }
                }
                else if (c1 == 79) // O
                {
                    switch(c2)
                    {
                        case 80: ch = KEY_F(1); stdio_buf_pos+=3; return ch;
                        case 81: ch = KEY_F(2); stdio_buf_pos+=3; return ch;
                        case 82: ch = KEY_F(3); stdio_buf_pos+=3; return ch;
                    }
                }
            }
        }
        
        // If no sequence matched, return char and advance
        stdio_buf_pos++;
    }

    return ch;
}


int get_singlechar_block()
{
    int ch;

#ifdef USE_NCURSES
    if(screenprintmode == SCREENPRINT_NCURSES)
    {
        ch = getchar();
    }
    else
#endif
    {
        int getchardt_us = 1000; // 1 ms

        ch = get_singlechar_nonblock();
        while(ch == -1)
        {
            usleep(getchardt_us);
            ch = get_singlechar_nonblock();
        }
    }
    return ch;
}
