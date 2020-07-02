/**
 * @file    AOloopControl_DM.c
 * @brief   DM control
 *
 * To be used for AOloopControl module
 *
 *
 *
 */


/* ================================================================== */
/* ================================================================== */
/*            MODULE INFO                                             */
/* ================================================================== */
/* ================================================================== */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaodm"

// Module short description
#define MODULE_DESCRIPTION       "AO loop Control DM operation"

// Application to which module belongs
#define MODULE_APPLICATION       "cacao"





/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */




#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl_DM/AOloopControl_DM.h"






/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */







long NB_DMindex = 9;

//AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
int dmdispcomb_loaded = 0;
int SMfd;


//AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
int dmturb_loaded = 0;
int SMturbfd;







/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl_DM)


/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 1. INITIALIZATION, LOAD/CREATE                                                                  */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 2. RUNTIME COMPUTATION                                                                          */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



errno_t AOloopControl_DM_CombineChannels_cli()
{
    // 1  long DMindex
    // 2  long xsize
    // 3  long ysize
    // 4  int NBchannel
    // 5  int AveMode
    // 6  int dm2dm_mode
    // 7  char *dm2dm_DMmodes
    // 8  char *dm2dm_outdisp
    // 9  int wfsrefmode
    // 10 char *wfsref_WFSRespMat
    // 11 char *wfsref_out
    // 12 int voltmode
    // 13 int volttype     : voltage command type (1: linear bipolar, output is float) (2: quadratic unipolar, output is UI16)
    // 14 float stroke100  : displacement [um] for 100 V
    // 15 char *IDvolt_name
    // 16 float DCum
    // 17 float maxvolt

    int stringlenmax = 200;
    char fpsname[stringlenmax];



    if(CLI_checkarg(1, CLIARG_STR) + CLI_checkarg(2, CLIARG_LONG) == 0)
    {
        //unsigned int DMindex = (unsigned int) data.cmdargtoken[2].val.numl;

        // FPS interface name
        if(data.processnameflag ==
                0)   // name fps to something different than the process name
        {
            snprintf(fpsname, stringlenmax, "DMcomb-%s", data.cmdargtoken[2].val.string);
            //sprintf(fpsname, "DMcomb-%06u", DMindex);
        }
        else     // Set fps name to be process name up to first instance of character '.'
        {
            strcpy(fpsname, data.processname0);
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_FPSINIT_") == 0)
                && (CLI_checkarg(2, CLIARG_LONG) == 0))    // init FPS
        {
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, FPSCMDCODE_FPSINIT,
                                                    data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_CONFSTART_") == 0)
                && (CLI_checkarg(2, CLIARG_LONG) == 0))    // Start conf process
        {
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, FPSCMDCODE_CONFSTART,
                                                    data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_CONFSTOP_") == 0)
                && (CLI_checkarg(2, CLIARG_LONG) == 0))   // Stop conf process
        {
            printf("Function parameters configure\n");
            AOloopControl_DM_CombineChannels_FPCONF(fpsname, FPSCMDCODE_CONFSTOP,
                                                    data.cmdargtoken[2].val.numl);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_RUNSTART_") == 0)
                && (CLI_checkarg(2, CLIARG_LONG) == 0))   // Run process
        {
            printf("Run function\n");
            AOloopControl_DM_CombineChannels_RUN(fpsname);
            return 0;
        }

        if((strcmp(data.cmdargtoken[1].val.string, "_RUNSTOP_") == 0)
                && (CLI_checkarg(2, CLIARG_LONG) == 0))   // Run process
        {
            printf("Run function STOP\n");
            AOloopControl_DM_dmdispcomboff(data.cmdargtoken[2].val.numl);
            return 0;
        }
    }

    // FPS-free implementation - all parameters specified at function launch
    if(0
            + CLI_checkarg(1, CLIARG_LONG)
            + CLI_checkarg(2, CLIARG_LONG)
            + CLI_checkarg(3, CLIARG_LONG)
            + CLI_checkarg(4, CLIARG_LONG)
            + CLI_checkarg(5, CLIARG_LONG)
            + CLI_checkarg(6, CLIARG_LONG)
            + CLI_checkarg(7, CLIARG_STR)
            + CLI_checkarg(8, CLIARG_STR)
            + CLI_checkarg(9, CLIARG_LONG)
            + CLI_checkarg(10, CLIARG_STR)
            + CLI_checkarg(11, CLIARG_STR)
            + CLI_checkarg(12, CLIARG_LONG)
            + CLI_checkarg(13, CLIARG_LONG)
            + CLI_checkarg(14, CLIARG_FLOAT)
            + CLI_checkarg(15, CLIARG_STR)
            + CLI_checkarg(16, CLIARG_FLOAT)
            + CLI_checkarg(17, CLIARG_FLOAT)
            == 0)
    {
        AOloopControl_DM_CombineChannels(
            data.cmdargtoken[1].val.numl,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.numl,
            data.cmdargtoken[4].val.numl,
            data.cmdargtoken[5].val.numl,
            data.cmdargtoken[6].val.numl,
            data.cmdargtoken[7].val.string,
            data.cmdargtoken[8].val.string,
            data.cmdargtoken[9].val.numl,
            data.cmdargtoken[10].val.string,
            data.cmdargtoken[11].val.string,
            data.cmdargtoken[12].val.numl,
            data.cmdargtoken[13].val.numl,
            data.cmdargtoken[14].val.numf,
            data.cmdargtoken[15].val.string,
            data.cmdargtoken[16].val.numf,
            data.cmdargtoken[17].val.numf
        );
        return 0;
    }
    else
    {
        // launch configuration process
        return CLICMD_INVALID_ARG;
    }

    //   {// DEFAULT: no dm2dm, no wfsref, dmvolt output
    //printf("AUTO volt_name = %s\n", data.cmdargtoken[13].val.string);
    //     AOloopControl_DM_CombineChannels(0, 50, 50, 12, 1, 0, "dmmodes", "outdisp", 0, "wfsrm", "refout", 1, 2, 0.7, "dmvolt", 0.0, 150.0);
    // }

}




errno_t AOloopControl_DM_dmdispcomboff_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_dmdispcomboff(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmtrigoff_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_dmtrigoff(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 3. CONFIGURATION                                                                                */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

errno_t AOloopControl_DM_dmdispcombstatus_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_dmdispcombstatus(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_chan_setgain_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) + CLI_checkarg(3, 1) == 0)
    {
        AOloopControl_DM_chan_setgain(data.cmdargtoken[1].val.numl,
                                      data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setvoltON_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_setvoltON(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setvoltOFF_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_setvoltOFF(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setMAXVOLT_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 1) == 0)
    {
        AOloopControl_DM_setMAXVOLT(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setDClevel_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 1) == 0)
    {
        AOloopControl_DM_setDClevel(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setAveMode_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) == 0)
    {
        AOloopControl_DM_setAveMode(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setTrigMode_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) == 0)
    {
        AOloopControl_DM_setTrigMode(data.cmdargtoken[1].val.numl,
                                     data.cmdargtoken[2].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setTrigChan_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) == 0)
    {
        AOloopControl_DM_setTrigChan(data.cmdargtoken[1].val.numl,
                                     data.cmdargtoken[2].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_setTrigSem_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) == 0)
    {
        AOloopControl_DM_setTrigSem(data.cmdargtoken[1].val.numl,
                                    data.cmdargtoken[2].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}





/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

errno_t AOloopControl_DM_dmturb_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_dmturb(data.cmdargtoken[1].val.numl, 0, "NULL", 0);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturb2im_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(3, 2) + CLI_checkarg(3, 2) == 0)
    {
        AOloopControl_DM_dmturb(data.cmdargtoken[1].val.numl, 1,
                                data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturboff_cli()
{
    if(CLI_checkarg(1, 2) == 0)
    {
        AOloopControl_DM_dmturboff(data.cmdargtoken[1].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturb_wspeed_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 1) == 0)
    {
        AOloopControl_DM_dmturb_wspeed(data.cmdargtoken[1].val.numl,
                                       data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturb_ampl_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 1) == 0)
    {
        AOloopControl_DM_dmturb_ampl(data.cmdargtoken[1].val.numl,
                                     data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturb_LOcoeff_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 1) == 0)
    {
        AOloopControl_DM_dmturb_LOcoeff(data.cmdargtoken[1].val.numl,
                                        data.cmdargtoken[2].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_dmturb_tint_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 2) == 0)
    {
        AOloopControl_DM_dmturb_tint(data.cmdargtoken[1].val.numl,
                                     data.cmdargtoken[2].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}



/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 5. MISC TESTS                                                                                   */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */

errno_t AOloopControl_mkDM_TT_circle_cli()
{
    if(CLI_checkarg(1, 3) + CLI_checkarg(2, 2) + CLI_checkarg(3,
            2) + CLI_checkarg(4, 1) == 0)
    {
        AOloopControl_mkDM_TT_circle(data.cmdargtoken[1].val.string,
                                     data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl,
                                     data.cmdargtoken[4].val.numf);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

errno_t AOloopControl_DM_mkAstroGrid_seq_cli()
{
    if(CLI_checkarg(1, 3) + CLI_checkarg(2, 2) + CLI_checkarg(3,
            2) + CLI_checkarg(4, 2) + CLI_checkarg(5, 2) == 0)
    {
        AOloopControl_DM_mkAstroGrid_seq(data.cmdargtoken[1].val.string,
                                         data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl,
                                         data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}













static errno_t init_module_CLI()
{


    /* =============================================================================================== */
    /* =============================================================================================== */
    /*                                                                                                 */
    /* 2. RUNTIME COMPUTATION                                                                          */
    /*                                                                                                 */
    /* =============================================================================================== */
    /* =============================================================================================== */

    RegisterCLIcommand(
        "aolcontrolDMcomb",
        __FILE__,
        AOloopControl_DM_CombineChannels_cli,
        "create and combine DM channels",
        "<DMindex (0-9)> <xsize> <ysize> <NBchannel> <AveMode (1=if average level removed)> <dm2dm mode> <DMmodes> <outdm stream> <wfsref mode> <WFS resp mat> <wfsref stream> <voltmode (1=dmvolt computed)> <dmvolttype> <dmstroke100> <dmvoltname> <DClevel> <maxvolt [V]>",
        "aoloopcontrolDMcomb 0 50 50 8 0 1 dmmodes outdm 1 wfsrm wfsrefout 1 1 1.0 dmvolt 0.78 120.0",
        "int AOloopControl_DM_CombineChannels(long DMindex, long xsize, long ysize, int NBchannel, int AveMode, int dm2dm_mode, const char *dm2dm_DMmodes, const char *dm2dm_outdisp, int wfsrefmode, const char *wfsref_WFSRespMat, const char *wfsref_out, int voltmode, int volttype, float stroke100,const char *IDvolt_name, float DClevel, float maxvolt)");


    RegisterCLIcommand(
        "aoloopcontroldmcomboff",
        __FILE__,
        AOloopControl_DM_dmdispcomboff_cli,
        "turn off DM combine",
        "<DMindex (0-9)>",
        "aoloopcontroldmcomboff 0",
        "int AOloopControl_DM_dmdispcomboff(long DMindex)");


    RegisterCLIcommand(
        "aoloopcontroldmtrigoff",
        __FILE__,
        AOloopControl_DM_dmtrigoff_cli,
        "turn off DM trigger",
        "<DMindex (0-9)>",
        "aoloopcontroldmtrigoff 0",
        "int AOloopControl_DM_dmtrigoff(long DMindex)");



    /* =============================================================================================== */
    /* =============================================================================================== */
    /*                                                                                                 */
    /* 3. CONFIGURATION                                                                                */
    /*                                                                                                 */
    /* =============================================================================================== */
    /* =============================================================================================== */



    RegisterCLIcommand(
        "aoloopcontroldmcombmon",
        __FILE__,
        AOloopControl_DM_dmdispcombstatus_cli,
        "monitor DM comb program",
        "<DMindex (0-9)>",
        "aoloopcontroldmcombmon 0",
        "int AOloopControl_DM_dmdispcombstatus(long DMindex)");


    RegisterCLIcommand(
        "aolcontroldmchgain",
        __FILE__,
        AOloopControl_DM_chan_setgain_cli,
        "set gain for DM displacement channel",
        "<DMindex (0-9)> <chan#> <gain>",
        "aoloopcontroldmchgain 0 3 0.2",
        "int AOloopControl_DM_chan_setgain(long DMindex, int ch, float gain)");


    RegisterCLIcommand(
        "aoldmvoltON",
        __FILE__,
        AOloopControl_DM_setvoltON_cli,
        "turn on DM voltage",
        "<DMindex (0-9)>",
        "aoldmvoltON 0",
        "int AOloopControl_DM_setvoltON(long DMindex)");

    RegisterCLIcommand(
        "aoldmvoltOFF",
        __FILE__,
        AOloopControl_DM_setvoltOFF_cli,
        "turn off DM voltage",
        "<DMindex (0-9)>",
        "aoldmvoltOFF 0",
        "int AOloopControl_DM_setvoltOFF(long DMindex)");

    RegisterCLIcommand(
        "aolsetdmvoltmax",
        __FILE__,
        AOloopControl_DM_setMAXVOLT_cli,
        "set maximum DM voltage",
        "<DMindex (0-9)> <max voltage [V]>",
        "aolsetdmvoltmax 120.0",
        "int AOloopControl_DM_setMAXVOLT(long DMindex, float maxvolt)");


    RegisterCLIcommand(
        "aolsetdmDC",
        __FILE__,
        AOloopControl_DM_setDClevel_cli,
        "set DM DC level [um]",
        "<DMindex (0-9)> <DC level [um]>",
        "aolsetdmDC 0.5",
        "int AOloopControl_DM_setDClevel(long DMindex, float DClevel)");


    RegisterCLIcommand(
        "aolsetdmAveM",
        __FILE__,
        AOloopControl_DM_setAveMode_cli,
        "set DM averaging mode [0,1 or 2]",
        "<DMindex (0-9)> <AveMode>",
        "aolsetdmAveM 00 1",
        "int AOloopControl_DM_setAveMode(long DMindex, int AveMode)");


    RegisterCLIcommand(
        "aolsetdmTrigMode",
        __FILE__,
        AOloopControl_DM_setTrigMode_cli,
        "set DM trigger mode (0:std, 1:use single channel)",
        "<DMindex (0-9)> <TrigMode [0, 1]>",
        "aolsetdmTrigMode 0 1",
        "int AOloopControl_DM_setTrigMode(long DMindex, int mode)");


    RegisterCLIcommand(
        "aolsetdmTrigChan",
        __FILE__,
        AOloopControl_DM_setTrigChan_cli,
        "set DM trigger channel",
        "<DMindex (0-9)> <TrigChan>",
        "aolsetdmTrigChan 0 3",
        "int AOloopControl_DM_setTrigChan(long DMindex, int chan)");


    RegisterCLIcommand(
        "aolsetdmTrigSem",
        __FILE__,
        AOloopControl_DM_setTrigSem_cli,
        "set DM trigger semaphore",
        "<DMindex (0-9)> <TrigSem [0-9]>",
        "aolsetdmTrigSem 0 4",
        "int AOloopControl_DM_setTrigSem(long DMindex, int sem)");



    /* =============================================================================================== */
    /* =============================================================================================== */
    /*                                                                                                 */
    /* 4. TURBULENCE SIMULATOR                                                                         */
    /*                                                                                                 */
    /* =============================================================================================== */
    /* =============================================================================================== */


    RegisterCLIcommand(
        "aoloopcontroldmturbprint",
        __FILE__,
        AOloopControl_printDMturbconf,
        "print DM turb configuration",
        "no arg",
        "aoloopcontroldmturbprint",
        "int AOloopControl_printDMturbconf()");


    RegisterCLIcommand(
        "aoloopcontroldmturb",
        __FILE__,
        AOloopControl_DM_dmturb_cli,
        "DM turbulence",
        "<DMindex (0-9)>",
        "aoloopcontroldmturb 0",
        "int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)");


    RegisterCLIcommand(
        "aoloopcontroldmturb2im",
        __FILE__,
        AOloopControl_DM_dmturb2im_cli,
        "DM turbulence to image",
        "<DMindex (00-09) <imoutname> <NBsamples>",
        "aoloopcontroldmturb2im 00 wftout 100000",
        "int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)");

    RegisterCLIcommand(
        "aoloopcontroldmturboff",
        __FILE__,
        AOloopControl_DM_dmturboff_cli,
        "turn off DM turbulence",
        "<DMindex (0-9)>",
        "aoloopcontroldmturboff 0",
        "int AOloopControl_DM_dmturboff(long DMindex)");



    RegisterCLIcommand(
        "aoloopcontroldmturws",
        __FILE__,
        AOloopControl_DM_dmturb_wspeed_cli,
        "set turbulence wind speed",
        "<DMindex (0-9)> <wind speed [m/s]>",
        "aoloopcontroldmturws 0 5.2",
        "int AOloopControl_DM_dmturb_wspeed(long DMindex, double wspeed)");


    RegisterCLIcommand(
        "aoloopcontroldmturampl",
        __FILE__,
        AOloopControl_DM_dmturb_ampl_cli,
        "set turbulence amplitude",
        "<DMindex (0-9)> <amplitude [um]>",
        "aoloopcontroldmturampl 0 0.1",
        "int AOloopControl_DM_dmturb_ampl(long DMindex, double ampl)");


    RegisterCLIcommand(
        "aoloopcontroldmturlo",
        __FILE__,
        AOloopControl_DM_dmturb_LOcoeff_cli,
        "set turbulence low order coefficient",
        "<DMindex (0-9)> <coeff>",
        "aoloopcontroldmturlo 0 0.2",
        "int AOloopControl_DM_dmturb_LOcoeff(long DMindex, double LOcoeff)");


    RegisterCLIcommand(
        "aoloopcontroldmturtint",
        __FILE__,
        AOloopControl_DM_dmturb_tint_cli,
        "set turbulence interval time",
        "<DMindex (0-9)> <interval time [us] long>",
        "aoloopcontroldmturtint 0 200",
        "int AOloopControl_DM_dmturb_tint(long DMindex, long tint)");



    /* =============================================================================================== */
    /* =============================================================================================== */
    /*                                                                                                 */
    /* 5. MISC TESTS & UTILS                                                                           */
    /*                                                                                                 */
    /* =============================================================================================== */
    /* =============================================================================================== */


    RegisterCLIcommand(
        "aoloopcontroldmmkttcirc",
        __FILE__,
        AOloopControl_mkDM_TT_circle_cli,
        "make DM TT circle file",
        "<outfname> <DMindex (0-9)> <NBpt> <ampl>",
        "aoloopcontroldmmkttcirc ttcirc 0 20 0.5",
        "long AOloopControl_mkDM_TT_circle(char *IDoutname, long DMindex, long NBpts, float ampl)");


    RegisterCLIcommand(
        "aoloopcontroldmastrogseq",
        __FILE__,
        AOloopControl_DM_mkAstroGrid_seq_cli,
        "make astrogrid sequence",
        "<outfname> <DMindex (0-9)> <mode (0-6)> <bin(>1)> <NBcycle>",
        "aoloopcontroldmastrogseq astrogridseq 0 0 1 1",
        "long AOloopControl_DM_mkAstroGrid_seq(char *IDoutname, long DMindex, int XYmode, int bin, long NBcycle)");



    // add atexit functions here
    atexit((void *) AOloopControl_DM_unloadconf);

    return RETURN_SUCCESS;
}

