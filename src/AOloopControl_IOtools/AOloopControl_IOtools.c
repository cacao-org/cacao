/**
 * @file    AOloopControl_IOtools.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    22 Aug 2017
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



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


static sem_t AOLCOMPUTE_TOTAL_ASYNC_sem_name;

static long long imtotalcnt;
static int AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 0;
static int COMPUTE_DARK_SUBTRACT_NBTHREADS = 1;
static sem_t AOLCOMPUTE_DARK_SUBTRACT_sem_name[32];
static sem_t AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[32];


static int avcamarraysInit = 0;
static unsigned short *arrayutmp;

static char Average_cam_frames_dname[200];
static long Average_cam_frames_IDdark = -1;
static long Average_cam_frames_nelem = 1;


static float *arrayftmp;


// TIMING
static struct timespec tnow;
static struct timespec tdiff;
static double tdiffv;

//extern int aoloopcontrol_var.PIXSTREAM_SLICE;

static long ti; // thread index

static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;
static int AOLCOMPUTE_TOTAL_INIT = 0; // toggles to 1 AFTER total for first image is computed


//extern float aoloopcontrol_var.normfloorcoeff;


//extern float aoloopcontrol_var.GPU_alpha;
//extern float aoloopcontrol_var.GPU_beta;








/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c














// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */


/** @brief CLI function for AOloopControl_camimage_extract2D_sharedmem_loop */
int_fast8_t AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,5)+CLI_checkarg(3,3)+CLI_checkarg(4,2)+CLI_checkarg(5,2)+CLI_checkarg(6,2)+CLI_checkarg(7,2)==0) {
        AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string , data.cmdargtoken[4].val.numl, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numl, data.cmdargtoken[7].val.numl);
        return 0;
    }
    else return 1;
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
int_fast8_t AOloopControl_IOtools_AveStream_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,3)==0) {
        AOloopControl_IOtools_AveStream(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string);
        return 0;
    }
    else return 1;
}


/** @brief CLI function for AOloopControl_frameDelay */
int_fast8_t AOloopControl_IOtools_frameDelay_cli()
{
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,5)+CLI_checkarg(4,2)==0)    {
        AOloopControl_IOtools_frameDelay(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else        return 1;
}


/** @brief CLI function for AOloopControl_stream3Dto2D */
int_fast8_t AOloopControl_IOtools_stream3Dto2D_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,2)+CLI_checkarg(4,2)==0) {
        AOloopControl_IOtools_stream3Dto2D(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numl);
        return 0;
    }
    else return 1;
}















/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools functions */


void __attribute__ ((constructor)) libinit_AOloopControl_IOtools()
{
	init_AOloopControl_IOtools();
//	printf(" ...... Loading module %s\n", __FILE__);
}


int_fast8_t init_AOloopControl_IOtools()
{

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].package, "cacao");;
    strcpy(data.module[data.NBmodule].info, "AO loop control IO tools");
    data.NBmodule++;



/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 1. CAMERA INPUT
 *  Read camera imates */
/* =============================================================================================== */
/* =============================================================================================== */

    RegisterCLIcommand("cropshim", __FILE__, AOloopControl_IOtools_camimage_extract2D_sharedmem_loop_cli, "crop shared mem image", "<input image> <optional dark> <output image> <sizex> <sizey> <xstart> <ystart>" , "cropshim imin null imout 32 32 153 201", "int AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(char *in_name, const char *dark_name, char *out_name, long size_x, long size_y, long xstart, long ystart)");




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 3. DATA STREAMS PROCESSING      
 *  Data streams real-time processing */
/* =============================================================================================== */
/* =============================================================================================== */


    RegisterCLIcommand("aveACshmim", __FILE__, AOloopControl_IOtools_AveStream_cli, "average and AC shared mem image", "<input image> <coeff> <output image ave> <output AC> <output RMS>" , "aveACshmim imin 0.01 outave outAC outRMS", "int AOloopControl_IOtools_AveStream(char *IDname, double alpha, char *IDname_out_ave, char *IDname_out_AC, char *IDname_out_RMS)");

    RegisterCLIcommand("aolframedelay", __FILE__, AOloopControl_IOtools_frameDelay_cli, "introduce temporal delay", "<in> <temporal kernel> <out> <sem index>","aolframedelay in kern out 0","long AOloopControl_IOtools_frameDelay(const char *IDin_name, const char *IDkern_name, const char *IDout_name, int insem)");

    RegisterCLIcommand("aolstream3Dto2D", __FILE__, AOloopControl_IOtools_stream3Dto2D_cli, "remaps 3D cube into 2D image", "<input 3D stream> <output 2D stream> <# cols> <sem trigger>" , "aolstream3Dto2D in3dim out2dim 4 1", "long AOloopControl_IOtools_stream3Dto2D(const char *in_name, const char *out_name, int NBcols, int insem)");





    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}
