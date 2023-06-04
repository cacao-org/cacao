/**
 * @file    modalstatsTUI.c
 * @brief   modal statistics TUI
 *
 *
 *
 */

#include <math.h>
#include <ncurses.h>

#include "CommandLineInterface/CLIcore.h"
#include "CommandLineInterface/TUItools.h"


static short unsigned int wrow, wcol;



typedef struct
{
    long modeindex;
    long NBmode;
} MODALSTATSTRUCT;




static int modalstats_TUI_process_user_key(
    int ch,
    MODALSTATSTRUCT *mstatstruct
)
{
    DEBUG_TRACE_FSTART();

    int loopOK = 1;

    switch(ch)
    {

    case 'x': // Exit control screen
        loopOK = 0;
        break;

    case KEY_UP:
        mstatstruct->modeindex --;
        if(mstatstruct->modeindex<0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case KEY_DOWN:
        mstatstruct->modeindex ++;
        if(mstatstruct->modeindex > mstatstruct->NBmode-1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode-1;
        }
        break;

    case KEY_PPAGE:
        mstatstruct->modeindex -= 10;
        if(mstatstruct->modeindex<0)
        {
            mstatstruct->modeindex = 0;
        }
        break;

    case KEY_NPAGE:
        mstatstruct->modeindex += 10;
        if(mstatstruct->modeindex > mstatstruct->NBmode-1)
        {
            mstatstruct->modeindex = mstatstruct->NBmode-1;
        }
        break;

    }

    DEBUG_TRACE_FEXIT();
    return loopOK;
}







errno_t AOloopControl_modalstatsTUI(
    int loopindex
)
{
    DEBUG_TRACE_FSTART();
    printf("Modal stats TUI\n");


    MODALSTATSTRUCT mstatstruct;
    mstatstruct.modeindex = 0;


    // Connect to streams
    //
    uint32_t NBmode = 1;

    IMGID imgmodevalWFS;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalWFS", loopindex);
        imgmodevalWFS = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalWFS, ERRMODE_ABORT);
        NBmode = imgmodevalWFS.md->size[0];
    }
    mstatstruct.NBmode = NBmode;


    IMGID imgmodevalDM;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalDM", loopindex);
        imgmodevalDM = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalDM, ERRMODE_ABORT);
    }


    IMGID imgmgain;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mgain", loopindex);
        imgmgain = mkIMGID_from_name(name);
        resolveIMGID(&imgmgain, ERRMODE_ABORT);
    }

    IMGID imgmmult;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mmult", loopindex);
        imgmmult = mkIMGID_from_name(name);
        resolveIMGID(&imgmmult, ERRMODE_ABORT);
    }

    IMGID imgmlimit;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mlimit", loopindex);
        imgmlimit = mkIMGID_from_name(name);
        resolveIMGID(&imgmlimit, ERRMODE_ABORT);
    }


    // catch signals (CTRL-C etc)
    //
    set_signal_catch();



    // default:     use ncurses
    TUI_set_screenprintmode(SCREENPRINT_NCURSES);

    if(getenv("MILK_TUIPRINT_STDIO"))
    {
        // use stdio instead of ncurses
        TUI_set_screenprintmode(SCREENPRINT_STDIO);
    }

    if(getenv("MILK_TUIPRINT_NONE"))
    {
        TUI_set_screenprintmode(SCREENPRINT_NONE);
    }


    TUI_init_terminal(&wrow, &wcol);


    int loopOK = 1;
    long loopcnt = 0;

    while(loopOK == 1)
    {

        int ch = -1;
        int getchardt_us = 1000; // check often
        usleep(getchardt_us);
        ch = get_singlechar_nonblock();

        // read keyboard input
        //
        loopOK = modalstats_TUI_process_user_key(ch, &mstatstruct);



        TUI_clearscreen(&wrow, &wcol);

        TUI_ncurses_erase();





        TUI_printfw(" PRESS x to exit");
        TUI_newline();
        TUI_printfw("Loop %ld  -  Mode %5ld / %5ld - loopcnt %ld",
                    loopindex,
                    mstatstruct.modeindex,
                    mstatstruct.NBmode,
                    loopcnt
                   );
        TUI_newline();
        TUI_newline();

        long mi = mstatstruct.modeindex;

        TUI_printfw("MODE   WFS   DM   LIM  OL");
        TUI_newline();
        TUI_printfw("%4ld  %12f  %12f  %12f",
                    mstatstruct.modeindex,
                    imgmodevalWFS.im->array.F[mi],
                    imgmodevalDM.im->array.F[mi],
                    imgmlimit.im->array.F[mi]
                   );

        TUI_newline();




        screenprint_setcolor(9);
        TUI_printfw("============ SCREENS");
        TUI_newline();
        TUI_printfw("============ SCREENS");
        TUI_newline();
        TUI_printfw("============ SCREENS");
        TUI_newline();
        screenprint_unsetcolor(9);


        TUI_ncurses_refresh();
        loopcnt++;
    }

    TUI_exit();


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}