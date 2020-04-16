/**
 * @file    AOloopControl_IOtools.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 *
 * AO engine uses stream data structure
 *
 * @bug No known bugs.
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
#define MODULE_SHORTNAME_DEFAULT "cacaoio"

// Module short description
#define MODULE_DESCRIPTION       "AO loop control IO tools"

// Application to which module belongs
#define MODULE_APPLICATION       "cacao"





#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */
#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"



/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */



# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */


//static sem_t AOLCOMPUTE_TOTAL_ASYNC_sem_name;

//static long long imtotalcnt;
//static int AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 0;
//static int COMPUTE_DARK_SUBTRACT_NBTHREADS = 1;
//static sem_t AOLCOMPUTE_DARK_SUBTRACT_sem_name[32];
//static sem_t AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[32];


//static int avcamarraysInit = 0;
//static unsigned short *arrayutmp;

//static char Average_cam_frames_dname[200];
//static long Average_cam_frames_IDdark = -1;
//static long Average_cam_frames_nelem = 1;


//static float *arrayftmp;


// TIMING
//static struct timespec tnow;
//static struct timespec tdiff;
//static double tdiffv;

//extern int aoloopcontrol_var.PIXSTREAM_SLICE;

//static long ti; // thread index

//static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;
//static int AOLCOMPUTE_TOTAL_INIT = 0; // toggles to 1 AFTER total for first image is computed


//extern float aoloopcontrol_var.normfloorcoeff;


//extern float aoloopcontrol_var.GPU_alpha;
//extern float aoloopcontrol_var.GPU_beta;








/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */


extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c











/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl_IOtools)


/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */





errno_t AOloopControl_IOtools_acquireWFSloop_cli()
{
    int stringmaxlen = 200;
    char fpsname[stringmaxlen];

    // First, we try to execute function through FPS interface
    if(CLI_checkarg(1, 5) == 0)   // check that first arg is string
    {
        // unsigned int OptionalArg00 = data.cmdargtoken[2].val.numl;
        // Set FPS interface name
        // By convention, if there are optional arguments, they should be appended to the fps name
        //
        if(data.processnameflag ==
                0)   // name fps to something different than the process name
        {
            if(strlen(data.cmdargtoken[2].val.string) > 0)
            {
                snprintf(fpsname, stringmaxlen, "acquWFS-%s", data.cmdargtoken[2].val.string);
            }
            else
            {
                sprintf(fpsname, "acquWFS");
            }
        }
        else     // Automatically set fps name to be process name up to first instance of character '.'
        {
            strcpy(fpsname, data.processname0);
        }
        if(strcmp(data.cmdargtoken[1].val.string,
                  "_FPSINIT_") == 0)    // Initialize FPS
        {
            AOcontrolLoop_IOtools_acquireWFSloop_FPCONF(fpsname, FPSCMDCODE_FPSINIT);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string,
                  "_CONFSTART_") == 0)    // Start conf process
        {
            AOcontrolLoop_IOtools_acquireWFSloop_FPCONF(fpsname, FPSCMDCODE_CONFSTART);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string,
                  "_CONFSTOP_") == 0)   // Stop conf process
        {
            AOcontrolLoop_IOtools_acquireWFSloop_FPCONF(fpsname, FPSCMDCODE_CONFSTOP);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_RUNSTART_") == 0)   // Run process
        {
            AOcontrolLoop_IOtools_acquireWFSloop_RUN(fpsname);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_RUNSTOP_") == 0)   // Stop process
        {
            //
            return RETURN_SUCCESS;
        }
        //}
        // non FPS implementation - all parameters specified at function launch
        //if(CLI_checkarg(1,1)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,2)+CLI_checkarg(5,2)+CLI_checkarg(6,4)+CLI_checkarg(7,5)+CLI_checkarg(8,5)+CLI_checkarg(9,2)+CLI_checkarg(10,2)+CLI_checkarg(11,2)+CLI_checkarg(12,2)==0) {
        //AOloopControl_Otools_acquireWFSloop();
        return RETURN_SUCCESS;
    }
    else
    {
        return RETURN_FAILURE;
    }
}






/** @brief CLI function for AOloopControl_camimage_extract2D_sharedmem_loop */
errno_t AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli()
{
    if(
        CLI_checkarg(1, 4)
        + CLI_checkarg(2, 5)
        + CLI_checkarg(3, 3)
        + CLI_checkarg(4, 2)
        + CLI_checkarg(5, 2)
        + CLI_checkarg(6, 2)
        + CLI_checkarg(7, 2)
        == 0)
    {
        AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.numl,
            data.cmdargtoken[5].val.numl,
            data.cmdargtoken[6].val.numl,
            data.cmdargtoken[7].val.numl
        );
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 2. LOAD DATA STREAMS
 *  Load 2D and 3D shared memory images */
/* =============================================================================================== */
/* =============================================================================================== */

// No command line hooks to functions in this section


/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_AveStream */
errno_t AOloopControl_IOtools_AveStream_cli()
{
    if(
        CLI_checkarg(1, 4) +
        CLI_checkarg(2, 1) +
        CLI_checkarg(3, 3) +
        CLI_checkarg(4, 3) +
        CLI_checkarg(5, 3)
        == 0)
    {
        AOloopControl_IOtools_AveStream(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numf,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string,
            data.cmdargtoken[5].val.string
        );

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}


/** @brief Aligns data stream */
errno_t AOloopControl_IOtools_imAlignStream_cli()
{
    if(
        CLI_checkarg(1, 4) +
        CLI_checkarg(2, 2) +
        CLI_checkarg(3, 2) +
        CLI_checkarg(4, 4) +
        CLI_checkarg(5, 3) +
        CLI_checkarg(6, 2)
        == 0)
    {
        AOloopControl_IOtools_imAlignStream(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.numl,
            data.cmdargtoken[4].val.string,
            data.cmdargtoken[5].val.string,
            data.cmdargtoken[6].val.numl
        );

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}


/** @brief CLI function for AOloopControl_frameDelay */
errno_t AOloopControl_IOtools_frameDelay_cli()
{
    if(
        CLI_checkarg(1, 4) +
        CLI_checkarg(2, 4) +
        CLI_checkarg(3, 5) +
        CLI_checkarg(4, 2)
        == 0)
    {
        AOloopControl_IOtools_frameDelay(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.numl
        );

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}



/** @brief CLI function for AOloopControl_stream3Dto2D */
errno_t AOloopControl_IOtools_stream3Dto2D_cli()
{
    if(
        CLI_checkarg(1, 4) +
        CLI_checkarg(2, 3) +
        CLI_checkarg(3, 2) +
        CLI_checkarg(4, 2)
        == 0)
    {
        AOloopControl_IOtools_stream3Dto2D(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.numl,
            data.cmdargtoken[4].val.numl
        );

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 4. SAVE REAL-TIME TELEMETRY BUFFER
 *  Save to disk telemetry packaged in alternate buffers */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief Save telemetry */
errno_t AOloopControl_IOtools_RTLOGsave_cli()
{
    if(
        CLI_checkarg(1, 2) +
        CLI_checkarg(2, 5) +
        CLI_checkarg(3, 5)
        == 0)
    {
        AOloopControl_IOtools_RTLOGsave(
            data.cmdargtoken[1].val.numl,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string
        );

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}











/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools functions */



static errno_t init_module_CLI()
{


    /* =============================================================================================== */
    /* =============================================================================================== */
    /** @name AOloopControl_IOtools - 1. CAMERA INPUT
     *  Read camera imates */
    /* =============================================================================================== */
    /* =============================================================================================== */

    RegisterCLIcommand(
        "aolacquireWFSloop",
        __FILE__,
        AOloopControl_IOtools_acquireWFSloop_cli,
        "acquire WFS loop",
        "<loopindex>",
        "aolacquireWFSloop 2",
        "int AOloopControl_IOtools_acquireWFSloop(long loop)");



    RegisterCLIcommand(
        "cropshim",
        __FILE__,
        AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli,
        "crop shared mem image",
        "<input image> <optional dark> <output image> <sizex> <sizey> <xstart> <ystart>",
        "cropshim imin null imout 32 32 153 201",
        "int AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(char *in_name, const char *dark_name, char *out_name, long size_x, long size_y, long xstart, long ystart)");




    /* =============================================================================================== */
    /* =============================================================================================== */
    /** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING
     *  Data streams real-time processing */
    /* =============================================================================================== */
    /* =============================================================================================== */


    RegisterCLIcommand(
        "aveACshmim",
        __FILE__,
        AOloopControl_IOtools_AveStream_cli,
        "average and AC shared mem image",
        "<input image> <coeff> <output image ave> <output AC> <output RMS>",
        "aveACshmim imin 0.01 outave outAC outRMS",
        "int AOloopControl_IOtools_AveStream(char *IDname, double alpha, char *IDname_out_ave, char *IDname_out_AC, char *IDname_out_RMS)"
    );

    RegisterCLIcommand(
        "alignshmim",
        __FILE__,
        AOloopControl_IOtools_imAlignStream_cli,
        "align image stream to reference",
        "<input stream> <box x offset> <box y offset> <ref stream> <output stream> <sem index>",
        "alignshmim imin 100 100 imref imout 3",
        "errno_t AOloopControl_IOtools_imAlignStream(const char *IDname, int xbox0, int ybox0, const char *IDref_name, const char *IDout_name, int insem)"
    );

    RegisterCLIcommand(
        "aolframedelay",
        __FILE__, AOloopControl_IOtools_frameDelay_cli,
        "introduce temporal delay", "<in> <temporal kernel> <out> <sem index>",
        "aolframedelay in kern out 0",
        "long AOloopControl_IOtools_frameDelay(const char *IDin_name, const char *IDkern_name, const char *IDout_name, int insem)"
    );

    RegisterCLIcommand(
        "aolstream3Dto2D",
        __FILE__,
        AOloopControl_IOtools_stream3Dto2D_cli,
        "remaps 3D cube into 2D image",
        "<input 3D stream> <output 2D stream> <# cols> <sem trigger>",
        "aolstream3Dto2D in3dim out2dim 4 1",
        "long AOloopControl_IOtools_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem)"
    );


    /* =============================================================================================== */
    /* =============================================================================================== */
    /** @name AOloopControl_IOtools - 4. SAVE REAL-TIME TELEMETRY BUFFER
     *  Save to disk telemetry packaged in alternate buffers */
    /* =============================================================================================== */
    /* =============================================================================================== */

    RegisterCLIcommand(
        "aolrtlogbuffsave",
        __FILE__,
        AOloopControl_IOtools_RTLOGsave_cli,
        "log realtime buffer stream",
        "<loopnumber> <streamname> <dirname>",
        "aolrtlogbuffsave modeval \"/media/data/\"",
        "int_fast8_t AOloopControl_IOtools_RTLOGsave(long loop, const char *streamname, const char *dirname)"
    );


    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}
