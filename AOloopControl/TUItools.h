/**
 * @file    TUItools.h
 * @brief   Text User Interface tools
 */

#ifndef TUITOOLS_H
#define TUITOOLS_H

#ifndef __STDC_LIB_EXT1__
typedef int errno_t;
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief print to screen, or not
 *
 * mode=0 : printf (stdout)
 * mode=1 : printw
 * mode=3 : don't print (silent)
 */

#define SCREENPRINT_STDIO   0
#define SCREENPRINT_NCURSES 1
#define SCREENPRINT_NONE    2

// ANSI ESCAPE CODES

//static int printAEC = 0;

#define AEC_NORMAL    0
#define AEC_BOLD      1
#define AEC_FAINT     2
#define AEC_ITALIC    3
#define AEC_UNDERLINE 4
#define AEC_SLOWBLINK 5
#define AEC_FASTBLINK 6
#define AEC_REVERSE   7

#define AEC_BOLDOFF      22
#define AEC_FAINTOFF     22
#define AEC_ITALICOFF    3
#define AEC_UNDERLINEOFF 24
#define AEC_BLINKOFF     25
#define AEC_REVERSEOFF   27


// Foreground color

//static int printAECfgcolor = AEC_FGCOLOR_WHITE;

#define AEC_FGCOLOR_BLACK   30
#define AEC_FGCOLOR_RED     31
#define AEC_FGCOLOR_GREEN   32
#define AEC_FGCOLOR_YELLOW  33
#define AEC_FGCOLOR_BLUE    34
#define AEC_FGCOLOR_MAGENTA 35
#define AEC_FGCOLOR_CYAN    36
#define AEC_FGCOLOR_WHITE   37
// note : +60 for high intensity

// Background color

//static int printAECbgcolor = AEC_BGCOLOR_BLACK;

#define AEC_BGCOLOR_BLACK   40
#define AEC_BGCOLOR_RED     41
#define AEC_BGCOLOR_GREEN   42
#define AEC_BGCOLOR_YELLOW  43
#define AEC_BGCOLOR_BLUE    44
#define AEC_BGCOLOR_MAGENTA 45
#define AEC_BGCOLOR_CYAN    46
#define AEC_BGCOLOR_WHITE   47
// note : +60 for high intensity

#define AECBOLDHIGREEN "\033[1;92;40m"
#define AECBOLDHIRED   "\033[1;91;40m"
#define AECNORMAL      "\033[37;40;0m"

#define COLOR_NONE      1
#define COLOR_OK        2
#define COLOR_WARNING   3
#define COLOR_ERROR     4
#define COLOR_DIRECTORY 7

// 5 : cyan bg
// 6 : green fg
// 7 : yellow fg
// 8 : red fg
// 9 : ref bg




typedef struct
{
    int  index;
    int  keych;
    char name[16];
} TUISCREEN;



void TUI_set_screenprintmode(int mode);
int  TUI_get_screenprintmode();

void TUI_printfw(const char *fmt, ...);
void TUI_newline();

void screenprint_setcolor(int colorcode);

void screenprint_unsetcolor(int colorcode);

void screenprint_setbold();

void screenprint_unsetbold();

void screenprint_setblink();

void screenprint_unsetblink();

void screenprint_setdim();

void screenprint_unsetdim();

void screenprint_setreverse();

void screenprint_unsetreverse();

void screenprint_setnormal();

errno_t TUI_print_header(const char *str, char c);

void TUI_reset_terminal_mode();

errno_t TUI_inittermios(short unsigned int *wrow, short unsigned int *wcol);

void TUI_clearscreen(short unsigned int *wrow, short unsigned int *wcol);

#ifdef USE_NCURSES
void TUI_handle_winch(int sig);

errno_t TUI_initncurses(short unsigned int *wrowptr, short unsigned int *wcolptr);
#endif

errno_t TUI_init_terminal(short unsigned int *wrowptr,
                          short unsigned int *wcolptr);

errno_t TUI_exit();

void TUI_atexit();

#ifdef USE_NCURSES
errno_t TUI_ncurses_refresh();

errno_t TUI_ncurses_erase();

errno_t TUI_get_terminal_size(short unsigned int *wrowptr,
                              short unsigned int *wcolptr);
#else
static inline errno_t TUI_ncurses_refresh() { return 0; }
static inline errno_t TUI_ncurses_erase() { return 0; }
static inline errno_t TUI_get_terminal_size(short unsigned int *wrowptr, short unsigned int *wcolptr) { (void)wrowptr; (void)wcolptr; return 0; }
#endif

errno_t TUI_stdio_clear();

int get_singlechar_nonblock();

int get_singlechar_block();



// useful macros

#ifndef TUITOOLS_C
#ifdef USE_NCURSES
#define INSERT_TUI_SETUP                                                       \
    TUI_set_screenprintmode(SCREENPRINT_NCURSES);                              \
    if(getenv("MILK_TUIPRINT_STDIO"))                                          \
    {                                                                          \
        TUI_set_screenprintmode(SCREENPRINT_STDIO);                            \
        printf("\e[1;1H\e[2J");                                                \
    }                                                                          \
                                                                           \
    if(getenv("MILK_TUIPRINT_NONE"))                                           \
    {                                                                          \
        TUI_set_screenprintmode(SCREENPRINT_NONE);                             \
    }                                                                          \
                                                                           \
    TUI_init_terminal(&wrow, &wcol);                                           \
    if(TUI_get_screenprintmode() == SCREENPRINT_NCURSES)                       \
    {                                                                          \
        keypad(stdscr, TRUE);                                                  \
        nodelay(stdscr, TRUE);                                                 \
        curs_set(0);                                                           \
    }                                                                          \
                                                                           \
    int       TUIpause __attribute__((unused)) = 0;                                                    \
    TUISCREEN __attribute__((unused)) TUIscreenarray[10];

#define INSTERT_TUI_KEYCONTROLS                                                \
    int TUIinputkch  = -1;                                                     \
    {                                                                          \
        int TUIinputkch0 = getch();                                            \
        while(TUIinputkch0 != -1)                                              \
        {                                                                      \
            switch(TUIinputkch0)                                               \
            {                                                                  \
                case 'x':                                                      \
                    processinfo->CTRLval = 3;                                  \
                    break;                                                     \
                case 'p':                                                      \
                    if(TUIpause == 1)                                          \
                    {                                                          \
                        TUIpause = 0;                                          \
                    }                                                          \
                    else                                                       \
                    {                                                          \
                        TUIpause = 1;                                          \
                    }                                                          \
                    break;                                                     \
                default:                                                       \
                    TUIinputkch = TUIinputkch0;                                \
                    break;                                                     \
            }                                                                  \
            TUIinputkch0 = getch();                                            \
        }                                                                      \
        for(int scrindex = 0; scrindex < NBTUIscreen; scrindex++)              \
        {                                                                      \
            if(TUIinputkch == TUIscreenarray[scrindex].keych)                  \
            {                                                                  \
                TUIscreen = TUIscreenarray[scrindex].index;                    \
            }                                                                  \
        }                                                                      \
    }
#else
#define INSERT_TUI_SETUP                                                       \
    TUI_set_screenprintmode(SCREENPRINT_STDIO);                                \
    TUI_init_terminal(&wrow, &wcol);                                           \
    int       TUIpause = 0;                                                    \
    TUISCREEN TUIscreenarray[10];

#define INSTERT_TUI_KEYCONTROLS                                                \
    int TUIinputkch  = -1;                                                     \
    {                                                                          \
        int TUIinputkch0 = get_singlechar_nonblock();                          \
        while(TUIinputkch0 != -1)                                              \
        {                                                                      \
            switch(TUIinputkch0)                                               \
            {                                                                  \
                case 'x':                                                      \
                    processinfo->CTRLval = 3;                                  \
                    break;                                                     \
                case 'p':                                                      \
                    TUIpause = !TUIpause;                                          \
                    break;                                                     \
                default:                                                       \
                    TUIinputkch = TUIinputkch0;                                \
                    break;                                                     \
            }                                                                  \
            TUIinputkch0 = get_singlechar_nonblock();                          \
        }                                                                      \
        for(int scrindex = 0; scrindex < NBTUIscreen; scrindex++)              \
        {                                                                      \
            if(TUIinputkch == TUIscreenarray[scrindex].keych)                  \
            {                                                                  \
                TUIscreen = TUIscreenarray[scrindex].index;                    \
            }                                                                  \
        }                                                                      \
    }
#endif
#endif


#define INSERT_TUI_SCREEN_MENU                                                 \
    for (int scr = 0; scr < NBTUIscreen; scr++)                                \
    {                                                                          \
        if (TUIscreenarray[scr].index == TUIscreen)                            \
        {                                                                      \
            screenprint_setreverse();                                          \
        }                                                                      \
        TUI_printfw(" %s ", TUIscreenarray[scr].name);                         \
        if (TUIscreenarray[scr].index == TUIscreen)                            \
        {                                                                      \
            screenprint_unsetreverse();                                        \
        }                                                                      \
    }                                                                          \
    TUI_newline();



inline static void print_help_entry(char *key, char *descr)
{
    screenprint_setbold();
    TUI_printfw("    %10s", key);
    screenprint_unsetbold();
    TUI_printfw("   %s", descr);
    TUI_newline();
}



#endif