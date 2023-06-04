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

    IMGID imgmodevalOL;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalOL", loopindex);
        imgmodevalOL = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalOL, ERRMODE_ABORT);
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



    // Telemetry buffers

    long buffsize = 0;
    IMGID imgmodevalWFSbuff;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalWFS_buff", loopindex);
        imgmodevalWFSbuff = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalWFSbuff, ERRMODE_ABORT);
        buffsize = imgmodevalWFSbuff.md->size[1];
    }
    double *WFSave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *WFSrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);


    IMGID imgmodevalDMbuff;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalDM_buff", loopindex);
        imgmodevalDMbuff = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalDMbuff, ERRMODE_ABORT);
    }
    double *DMave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *DMrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);


    IMGID imgmodevalOLbuff;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalOL_buff", loopindex);
        imgmodevalOLbuff = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalOLbuff, ERRMODE_ABORT);
    }
    double *OLave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *OLrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);







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

    // min / max of mode index to be displayed
    int mirange = (wrow-5);
    int mimin = 0;
    int mimax = mirange;
    long mioffset = 0;

    // buffer indices (old)
    long buffWFSindex0 = 0;
    long buffDMindex0 = 0;
    long buffOLindex0 = 0;

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
        TUI_printfw("Loop %ld  -  Mode %5ld / %5ld [%5ld-%5ld] - loopcnt %ld",
                    loopindex,
                    mstatstruct.modeindex,
                    mstatstruct.NBmode,
                    mimin, mimax,
                    loopcnt
                   );
        TUI_newline();
        TUI_newline();

        long mi = mstatstruct.modeindex;


        // update mimin and mimax


        if(mimax>mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }

        if(mstatstruct.modeindex > mimax-10)
        {
            // if within 10 lines of bottom
            mioffset ++;
            mimin = mioffset;
            mimax = mioffset + mirange;
            if(mimax > mstatstruct.NBmode)
            {
                mioffset --;
                mimin = mioffset;
                mimax = mioffset + mirange;
            }
        }

        if(mimax>mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }

        if(mstatstruct.modeindex < mimin+10)
        {
            // if within 10 lines of top
            mioffset --;
            mimin = mioffset;
            mimax = mioffset + mirange;
            if(mimin < 0)
            {
                mioffset = 0;
                mimin = 0;
                mimax = mimin + mirange;
            }
        }




        TUI_printfw("MODE   [ gain  mult  lim ]       WFS        DM          OL");
        TUI_newline();

        long buffWFSindex = imgmodevalWFSbuff.md->cnt0;
        long buffDMindex = imgmodevalDMbuff.md->cnt0;
        long buffOLindex = imgmodevalOLbuff.md->cnt0;



        if(buffWFSindex != buffWFSindex0)
        {
            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                WFSave[mi] = 0.0;
                WFSrms[mi] = 0.0;
            }
            for(uint32_t tstep=0; tstep<buffsize; tstep++)
            {
                for(uint32_t mi=0; mi<mstatstruct.NBmode; mi++)
                {
                    long index2 = imgmodevalWFSbuff.md->cnt1 * buffsize * mstatstruct.NBmode;
                    long index = index2 + tstep*mstatstruct.NBmode + mi;
                    float val = imgmodevalWFSbuff.im->array.F[index];
                    WFSave[mi] += val;
                    WFSrms[mi] += val*val;
                }
            }

            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                WFSave[mi] /= buffsize;
                WFSrms[mi] = sqrt( WFSrms[mi]/buffsize - WFSave[mi]*WFSave[mi] );
            }

            buffWFSindex0 = buffWFSindex;
        }


        if(buffDMindex != buffDMindex0)
        {
            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                DMave[mi] = 0.0;
                DMrms[mi] = 0.0;
            }
            for(uint32_t tstep=0; tstep<buffsize; tstep++)
            {
                for(uint32_t mi=mimin; mi<mimax; mi++)
                {
                    long index2 = imgmodevalDMbuff.md->cnt1 * buffsize * mstatstruct.NBmode;
                    long index = index2 + tstep*mstatstruct.NBmode + mi;
                    float val = imgmodevalDMbuff.im->array.F[index];
                    DMave[mi] += val;
                    DMrms[mi] += val*val;
                }
            }

            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                DMave[mi] /= buffsize;
                DMrms[mi] = sqrt( DMrms[mi]/buffsize - DMave[mi]*DMave[mi] );
            }

            buffDMindex0 = buffDMindex;
        }


        if(buffOLindex != buffOLindex0)
        {
            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                OLave[mi] = 0.0;
                OLrms[mi] = 0.0;
            }
            for(uint32_t tstep=0; tstep<buffsize; tstep++)
            {
                for(uint32_t mi=mimin; mi<mimax; mi++)
                {
                    long index2 = imgmodevalOLbuff.md->cnt1 * buffsize * mstatstruct.NBmode;
                    long index = index2 + tstep*mstatstruct.NBmode + mi;
                    float val = imgmodevalOLbuff.im->array.F[index];
                    OLave[mi] += val;
                    OLrms[mi] += val*val;
                }
            }

            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                OLave[mi] /= buffsize;
                OLrms[mi] = sqrt( OLrms[mi]/buffsize - OLave[mi]*OLave[mi] );
            }

            buffOLindex0 = buffOLindex;
        }





        TUI_printfw("buffers   WFS %5ld %5ld   DM %5ld %5ld   OL %5ld %5ld",
                    buffWFSindex, buffWFSindex0,
                    buffDMindex, buffDMindex0,
                    buffOLindex, buffOLindex0
                   );
        TUI_newline();




        for(mi=mimin; mi<mimax; mi++)
        {
            if(mi == mstatstruct.modeindex)
            {
                screenprint_setbold();
            }
            TUI_printfw("%4ld [%5.3f %5.3f %4f]   %+4f | %+4f | %+4f  WFS %+4f %4f  DM %+4f %4f  OL %+4f %4f",
                        mi,
                        imgmgain.im->array.F[mi],
                        imgmmult.im->array.F[mi],
                        imgmlimit.im->array.F[mi],
                        imgmodevalWFS.im->array.F[mi],
                        imgmodevalDM.im->array.F[mi],
                        imgmodevalOL.im->array.F[mi],
                        WFSave[mi], WFSrms[mi],
                        DMave[mi], DMrms[mi],
                        OLave[mi], OLrms[mi]
                       );
            TUI_newline();
            if(mi == mstatstruct.modeindex)
            {
                screenprint_unsetbold();
            }
        }



        //screenprint_setcolor(9);
        //screenprint_unsetcolor(9);


        TUI_ncurses_refresh();
        loopcnt++;
    }

    TUI_exit();

    free(WFSave);
    free(WFSrms);
    free(DMave);
    free(DMrms);
    free(OLave);
    free(OLrms);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}