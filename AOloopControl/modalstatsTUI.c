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
    long pscaleindex;
    float pscale;
} MODALSTATSTRUCT;





static uint64_t *AOloopindex;




static CLICMDARGDEF farg[] = {{
        CLIARG_UINT64,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    }
};



static errno_t customCONFsetup()
{

    return RETURN_SUCCESS;
}


static errno_t customCONFcheck()
{

    return RETURN_SUCCESS;
}


static CLICMDDATA CLIcmddata =
{
    "modalstatsTUI", "modal stats TUI", CLICMD_FIELDS_DEFAULTS
};





// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}








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

    case '+':
        mstatstruct->pscaleindex++;
        mstatstruct->pscale = pow(10.0, mstatstruct->pscaleindex);
        break;

    case '-':
        mstatstruct->pscaleindex--;
        mstatstruct->pscale = pow(10.0, mstatstruct->pscaleindex);
        break;

    }

    DEBUG_TRACE_FEXIT();
    return loopOK;
}




inline static void printfixedlen(
    float val,
    MODALSTATSTRUCT *mstatstruct
)
{
    long tmpl =  (long) (mstatstruct->pscale*val);
    if( (tmpl < 100000) && (tmpl > -100000) )
    {
        TUI_printfw("%+5ld", tmpl);
    }
    else
    {
        screenprint_setcolor(3);
        TUI_printfw("+++++");
        screenprint_unsetcolor(3);
    }
}


inline static void printfixedlen_unsigned(
    float val,
    MODALSTATSTRUCT *mstatstruct
)
{
    long tmpl =  (long) (mstatstruct->pscale*val);
    if( (tmpl < 100000) && (tmpl > -100000) )
    {
        TUI_printfw("%5ld", tmpl);
    }
    else
    {
        screenprint_setcolor(3);
        TUI_printfw("+++++");
        screenprint_unsetcolor(3);
    }
}






errno_t AOloopControl_modalstatsTUI(
    int loopindex
)
{
    DEBUG_TRACE_FSTART();
    printf("Modal stats TUI\n");


    MODALSTATSTRUCT mstatstruct;
    mstatstruct.modeindex = 0;
    mstatstruct.pscaleindex = 0;
    mstatstruct.pscale = 1.0;


    // Connect to streams
    //
    uint32_t NBmode = 1;



    IMGID imgDMmodes;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_DMmodes", loopindex);
        imgDMmodes = mkIMGID_from_name(name);
        resolveIMGID(&imgDMmodes, ERRMODE_ABORT);
        NBmode = imgDMmodes.md->size[2];
    }
    mstatstruct.NBmode = NBmode;


    IMGID imgmodevalWFS;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_modevalWFS", loopindex);
        imgmodevalWFS = mkIMGID_from_name(name);
        resolveIMGID(&imgmodevalWFS, ERRMODE_ABORT);
        NBmode = imgmodevalWFS.md->size[0];
    }
//    mstatstruct.NBmode = NBmode;


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


    // ========================= MODAL LIMIT COUNTER ==================
    IMGID imgmlimitcntfrac;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%d_mlimitcntfrac", loopindex);
        imgmlimitcntfrac = mkIMGID_from_name(name);
        resolveIMGID(&imgmlimitcntfrac, ERRMODE_ABORT);
    }


    // ========================= MODAL STATS ==================
    // accumulated and reported for each log buffer duration
    //

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






    double *WFSave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *WFSrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);

    double *DMave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *DMrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);

    double *OLave = (double*) malloc(sizeof(double)*mstatstruct.NBmode);
    double *OLrms = (double*) malloc(sizeof(double)*mstatstruct.NBmode);







    // Compute DMmodes norm
    double *DMmodenorm = (double*) malloc(sizeof(double)*NBmode);
    for(uint32_t mi=0; mi<NBmode; mi++)
    {
        double val = 0.0;
        double valcnt = 0.0;
        for(uint64_t ii=0; ii<imgDMmodes.md->size[0]*imgDMmodes.md->size[1]; ii++)
        {
            val += imgDMmodes.im->array.F[mi*imgDMmodes.md->size[0]*imgDMmodes.md->size[1] + ii]
                   * imgDMmodes.im->array.F[mi*imgDMmodes.md->size[0]*imgDMmodes.md->size[1] + ii];
            valcnt += 1.0;
        }
        DMmodenorm[mi] = sqrt(val/valcnt);
    }



    // catch signals (CTRL-C etc)
    //
    //set_signal_catch();



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

        if(mimax>mstatstruct.NBmode)
        {
            mimax = mstatstruct.NBmode;
        }




        TUI_printfw(" PRESS x to exit, +/- change display scale, UP/DOWN PGUP PGDOWN");
        TUI_newline();
        TUI_printfw("Loop %ld  -  Mode %5ld / %5ld [%5ld-%5ld] - loopcnt %ld",
                    loopindex,
                    mstatstruct.modeindex,
                    mstatstruct.NBmode,
                    mimin, mimax,
                    loopcnt
                   );
        TUI_newline();
        TUI_printfw("scale = %f", mstatstruct.pscale);
        TUI_newline();





        TUI_printfw("MODE [ gain  mult  lim ]           WFS       |          DM       |          OL       | LIMTRUC WFS/OL  DM/OL");
        TUI_newline();

        /*  long buffWFSindex = imgmodevalWFSbuff.md->cnt0;
          long buffDMindex = imgmodevalDMbuff.md->cnt0;
          long buffOLindex = imgmodevalOLbuff.md->cnt0;*/


        long buffindex0 = -1;
        long buffindex = imgmvalOLrms.md->cnt0;


        if(buffindex != buffindex0)
        {
            for(int32_t mi=mimin; mi<mimax; mi++)
            {
                WFSave[mi] = imgmvalWFSave.im->array.F[mi];
                WFSrms[mi] = imgmvalWFSrms.im->array.F[mi];
                // remove DC from rms
                WFSrms[mi] = sqrt ( WFSrms[mi]*WFSrms[mi] - WFSave[mi]*WFSave[mi]);

                DMave[mi] = imgmvalDMave.im->array.F[mi];
                DMrms[mi] = imgmvalDMrms.im->array.F[mi];
                // remove DC from rms
                DMrms[mi] = sqrt ( DMrms[mi]*DMrms[mi] - DMave[mi]*DMave[mi]);

                OLave[mi] = imgmvalOLave.im->array.F[mi];
                OLrms[mi] = imgmvalOLrms.im->array.F[mi];
                // remove DC from rms
                OLrms[mi] = sqrt ( OLrms[mi]*OLrms[mi] - OLave[mi]*OLave[mi]);
            }


            buffindex0 = buffindex;
        }








        for(mi=mimin; mi<mimax; mi++)
        {
            if(mi == mstatstruct.modeindex)
            {
                screenprint_setbold();
            }

            TUI_printfw("%4ld [%5.3f %5.3f ",
                        mi,
                        imgmgain.im->array.F[mi],
                        imgmmult.im->array.F[mi]
                       );

            printfixedlen_unsigned(imgmlimit.im->array.F[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw("]   ");




            printfixedlen(imgmodevalWFS.im->array.F[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" ");

            {
                int color = 0;

                if(WFSave[mi] > WFSrms[mi])
                {
                    color = 3;
                }
                screenprint_setcolor(color);
                printfixedlen(WFSave[mi]*DMmodenorm[mi], &mstatstruct);
                screenprint_unsetcolor(color);
            }

            TUI_printfw(" ");
            printfixedlen_unsigned(WFSrms[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" | ");



            printfixedlen(imgmodevalDM.im->array.F[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" ");
            {
                int color = 0;

                if(DMave[mi] > DMrms[mi])
                {
                    color = 3;
                }
                screenprint_setcolor(color);
                printfixedlen(DMave[mi]*DMmodenorm[mi], &mstatstruct);
                screenprint_unsetcolor(color);
            }
            TUI_printfw(" ");
            printfixedlen_unsigned(DMrms[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" | ");

            printfixedlen(imgmodevalOL.im->array.F[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" ");
            {
                int color = 0;

                if(OLave[mi] > OLrms[mi])
                {
                    color = 3;
                }
                screenprint_setcolor(color);
                printfixedlen(OLave[mi]*DMmodenorm[mi], &mstatstruct);
                screenprint_unsetcolor(color);
            }
            TUI_printfw(" ");
            printfixedlen_unsigned(OLrms[mi]*DMmodenorm[mi], &mstatstruct);
            TUI_printfw(" | ");



            // fraction of commands truncated by modal limit
            {
                float truncfract = imgmlimitcntfrac.im->array.F[mi];

                int color = 0;
                if(truncfract > 0.01)
                {
                    color = 3; // ORANGE
                }
                if(truncfract > 0.1)
                {
                    color = 4;  // RED
                }
                screenprint_setcolor(color);
                TUI_printfw("%6.4f", truncfract);
                screenprint_unsetcolor(color);
                TUI_printfw("  ");
            }


            float WFSoverOL = WFSrms[mi] / OLrms[mi];
            float DMoverOL  = DMrms[mi]  / OLrms[mi];

            int color = 0;
            if(WFSoverOL < 0.9)
            {
                color = 2; // GREEN
            }
            if(WFSoverOL > 1.0)
            {
                color = 4; // RED
            }
            screenprint_setcolor(color);
            TUI_printfw("%6.4f", WFSoverOL);
            screenprint_unsetcolor(color);

            TUI_printfw("  ");

            color = 0;
            if(DMoverOL > 0.5)
            {
                color = 2;
            }
            if(DMoverOL > 1.0)
            {
                color = 4;
            }
            screenprint_setcolor(color);
            TUI_printfw("%6.4f", DMoverOL);
            screenprint_unsetcolor(color);



            /*
                        screenprint_setcolor(0);
                        TUI_printfw("0");
                        screenprint_unsetcolor(0);

                        screenprint_setcolor(1);
                        TUI_printfw("1");
                        screenprint_unsetcolor(1);

                        screenprint_setcolor(2);
                        TUI_printfw("2");
                        screenprint_unsetcolor(2);

                        screenprint_setcolor(3);
                        TUI_printfw("3");
                        screenprint_unsetcolor(3);

                        screenprint_setcolor(4);
                        TUI_printfw("4");
                        screenprint_unsetcolor(4);

                        screenprint_setcolor(5);
                        TUI_printfw("5");
                        screenprint_unsetcolor(5);

                        screenprint_setcolor(6);
                        TUI_printfw("6");
                        screenprint_unsetcolor(6);

                        screenprint_setcolor(7);
                        TUI_printfw("7");
                        screenprint_unsetcolor(7);

                        screenprint_setcolor(8);
                        TUI_printfw("8");
                        screenprint_unsetcolor(8);

                        screenprint_setcolor(9);
                        TUI_printfw("9");
                        screenprint_unsetcolor(9);
            */

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

    free(DMmodenorm);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    AOloopControl_modalstatsTUI(*AOloopindex);


    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}





INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__modalstatsTUI()
{

    //CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    //CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
