/**
 * @file    AOloopControl_RTstreamLOG.c
 * @brief   Logging of real-time streams
 *
 * Real-time stream logging
 *
 *
 */

#define _GNU_SOURCE

#include <malloc.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <fcntl.h>
#include <ncurses.h>
#include <termios.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "ImageStreamIO/ImageStruct.h"

#include "AOloopControl/AOloopControl.h"

//extern AOLOOPCONTROL_CONF *AOconf;
//extern AOloopControl_var   aoloopcontrol_var; // declared in AOloopControl.c



static int print_header_line(const char *str, char c, int wcol)
{
    long n;
    long i;

    attron(A_BOLD);
    n = strlen(str);
    for(i = 0; i < (wcol - n) / 2; i++)
    {
        printw("%c", c);
    }
    printw("%s", str);
    for(i = 0; i < (wcol - n) / 2 - 1; i++)
    {
        printw("%c", c);
    }
    printw("\n");
    attroff(A_BOLD);

    return (0);
}

