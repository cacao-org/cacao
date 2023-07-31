/**
 * @file    AOloopControl_IOtools_camerainput.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 *
 * OBSOLETE CODE
 * new code in acquireWFS
 */

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "CommandLineInterface/timeutils.h"

#include "COREMOD_tools/COREMOD_tools.h"
#include "info/info.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"

#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

static sem_t AOLCOMPUTE_TOTAL_ASYNC_sem_name;

static long long imtotalcnt;
static int       AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 0;
static int       COMPUTE_DARK_SUBTRACT_NBTHREADS     = 1;
static sem_t     AOLCOMPUTE_DARK_SUBTRACT_sem_name[32];
static sem_t     AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[32];

// static int avcamarraysInit = 0;
static unsigned short *arrayutmp;
static signed short   *arraystmp;

static char          Average_cam_frames_dname[200];
static imageID       Average_cam_frames_IDdark = -1;
static unsigned long Average_cam_frames_nelem  = 1;

static float *arrayftmp;

// TIMING
static struct timespec tnow;
static double          tdiffv;

static long ti; // thread index

static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;

static int AOLCOMPUTE_TOTAL_INIT = 0;
// toggles to 1 AFTER total for first image is computed

/* ===============================================================================================
 */
/*                                     MAIN DATA STRUCTURES */
/* ===============================================================================================
 */

extern long LOOPNUMBER; // current loop index

//extern AOLOOPCONTROL_CONF *AOconf;            // declared in AOloopControl.c
//extern AOloopControl_var   aoloopcontrol_var; // declared in AOloopControl.c

//
// every time im_name changes (counter increments), crop it to out_name in
// shared memory
//
errno_t
AOloopControl_IOtools_camimage_extract2D_sharedmem_loop(const char *in_name,
        const char *dark_name,
        const char *out_name,
        long        size_x,
        long        size_y,
        long        xstart,
        long        ystart)
{
    long               iiin, jjin, iiout, jjout;
    imageID            IDin, IDout, IDdark;
    uint8_t            datatype;
    uint8_t            datatypeout;
    uint32_t          *sizeout;
    unsigned long long cnt0;
    imageID            IDmask;
    uint64_t           sizeoutxy;

    sizeout = (uint32_t *) malloc(sizeof(uint32_t) * 2);
    if(sizeout == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    sizeout[0] = size_x;
    sizeout[1] = size_y;
    sizeoutxy  = size_x * size_y;

    IDin     = image_ID(in_name);
    datatype = data.image[IDin].md[0].datatype;

    // Check if there is a mask
    IDmask = image_ID("csmask");
    if(IDmask != -1)
        if((data.image[IDmask].md[0].size[0] != size_x) ||
                (data.image[IDmask].md[0].size[1] != size_y))
        {
            printf("ERROR: csmask has wrong size\n");
            exit(EXIT_FAILURE);
        }

    // Check dark
    IDdark = image_ID(dark_name);

    if(IDdark != -1)
    {
        if((data.image[IDdark].md[0].size[0] !=
                data.image[IDin].md[0].size[0]) ||
                (data.image[IDdark].md[0].size[1] !=
                 data.image[IDin].md[0].size[1]))
        {
            printf("ERROR: csmask has wrong size\n");
            exit(EXIT_FAILURE);
        }
        if(data.image[IDdark].md[0].datatype != _DATATYPE_FLOAT)
        {
            printf("ERROR: csmask has wrong type\n");
            exit(EXIT_FAILURE);
        }
        datatypeout = _DATATYPE_FLOAT;
    }
    else
    {
        datatypeout = datatype;
    }

    // Create shared memory output image
    create_image_ID(out_name, 2, sizeout, datatypeout, 1, 0, 0, &IDout);

    cnt0 = -1;

    switch(datatype)
    {
        case _DATATYPE_UINT16:
            while(1)
            {
                usleep(10); // OK FOR NOW (NOT USED BY FAST WFS)
                if(data.image[IDin].md[0].cnt0 != cnt0)
                {
                    data.image[IDout].md[0].write = 1;
                    cnt0                          = data.image[IDin].md[0].cnt0;
                    if(datatypeout == _DATATYPE_UINT16)
                    {
                        for(iiout = 0; iiout < size_x; iiout++)
                            for(jjout = 0; jjout < size_y; jjout++)
                            {
                                iiin = xstart + iiout;
                                jjin = ystart + jjout;
                                data.image[IDout]
                                .array.UI16[jjout * size_x + iiout] =
                                    data.image[IDin].array.UI16
                                    [jjin * data.image[IDin].md[0].size[0] +
                                          iiin];
                            }
                        if(IDmask != -1)
                            for(uint64_t ii = 0; ii < sizeoutxy; ii++)
                            {
                                data.image[IDout].array.UI16[ii] *=
                                    (int) data.image[IDmask].array.F[ii];
                            }
                    }
                    else // FLOAT
                    {
                        if(IDdark == -1)
                        {
                            for(iiout = 0; iiout < size_x; iiout++)
                                for(jjout = 0; jjout < size_y; jjout++)
                                {
                                    iiin = xstart + iiout;
                                    jjin = ystart + jjout;
                                    data.image[IDout]
                                    .array.F[jjout * size_x + iiout] =
                                        data.image[IDin].array.UI16
                                        [jjin * data.image[IDin].md[0].size[0] +
                                              iiin];
                                }
                        }
                        else
                        {
                            for(iiout = 0; iiout < size_x; iiout++)
                                for(jjout = 0; jjout < size_y; jjout++)
                                {
                                    iiin = xstart + iiout;
                                    jjin = ystart + jjout;
                                    data.image[IDout]
                                    .array.F[jjout * size_x + iiout] =
                                        1.0 *
                                        data.image[IDin]
                                        .array.UI16[jjin * data.image[IDin]
                                                         .md[0]
                                                         .size[0] +
                                                         iiin] -
                                        data.image[IDdark].array.F
                                        [jjin *
                                              data.image[IDdark].md[0].size[0] +
                                              iiin];
                                }
                        }

                        if(IDmask != -1)
                            for(uint64_t ii = 0; ii < sizeoutxy; ii++)
                            {
                                data.image[IDout].array.F[ii] *=
                                    data.image[IDmask].array.F[ii];
                            }
                    }

                    data.image[IDout].md[0].cnt0  = cnt0;
                    data.image[IDout].md[0].write = 0;
                    COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
                }
            }
            break;
        case _DATATYPE_FLOAT:
            while(1)
            {
                usleep(10); // OK FOR NOW (NOT USED BY FAST WFS)
                if(data.image[IDin].md[0].cnt0 != cnt0)
                {
                    data.image[IDout].md[0].write = 1;
                    cnt0                          = data.image[IDin].md[0].cnt0;
                    if(IDdark == -1)
                    {
                        for(iiout = 0; iiout < size_x; iiout++)
                            for(jjout = 0; jjout < size_y; jjout++)
                            {
                                iiin = xstart + iiout;
                                jjin = ystart + jjout;
                                data.image[IDout].array.F[jjout * size_x + iiout] =
                                    data.image[IDin]
                                    .array
                                    .F[jjin * data.image[IDin].md[0].size[0] +
                                            iiin];
                            }
                    }
                    else
                    {
                        for(iiout = 0; iiout < size_x; iiout++)
                            for(jjout = 0; jjout < size_y; jjout++)
                            {
                                iiin = xstart + iiout;
                                jjin = ystart + jjout;
                                data.image[IDout].array.F[jjout * size_x + iiout] =
                                    data.image[IDin]
                                    .array
                                    .F[jjin * data.image[IDin].md[0].size[0] +
                                            iiin] -
                                    data.image[IDdark]
                                    .array
                                    .F[jjin * data.image[IDdark].md[0].size[0] +
                                            iiin];
                            }
                    }

                    if(IDmask != -1)
                        for(uint64_t ii = 0; ii < sizeoutxy; ii++)
                        {
                            data.image[IDout].array.F[ii] *=
                                data.image[IDmask].array.F[ii];
                        }

                    data.image[IDout].md[0].cnt0  = cnt0;
                    data.image[IDout].md[0].write = 0;
                    COREMOD_MEMORY_image_set_sempost_byID(IDout, -1);
                }
            }
            break;
        default:
            printf("ERROR: DATA TYPE NOT SUPPORTED\n");
            exit(0);
            break;
    }
    free(sizeout);

    return (0);
}




/*
static void *compute_function_imtotal(__attribute__((unused)) void *ptr)
{
    long  ii;
    long  nelem;
    float IMTOTAL;
    char  imname[200];

    printf(
        "=========== STARTING compute_function_imtotal loop "
        "===================\n");
    fflush(stdout);

    // LOG function / process start
    // int logfunc_level = 0;
    // int logfunc_level_max = 1;
    char commentstring[200];
    sprintf(commentstring, "Compute image total flux, loop %ld", LOOPNUMBER);
    // CORE_logFunctionCall( logfunc_level, logfunc_level_max, 0, __FILE__,
    // __func__, __LINE__, commentstring);

    if(aoloopcontrol_var.aoconfID_looptiming == -1)
    {
        // LOOPiteration is written in cnt1 of loop timing array
        if(sprintf(imname, "aol%ld_looptiming", LOOPNUMBER) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }
        aoloopcontrol_var.aoconfID_looptiming =
            AOloopControl_IOtools_2Dloadcreate_shmim(
                imname,
                " ",
                aoloopcontrol_var.AOcontrolNBtimers,
                1,
                0.0);
    }

    nelem = data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[0] *
            data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].size[1];

    for(;;)
    {
#ifdef _PRINT_TEST
        printf("TEST - Waiting for semaphore\n");
        fflush(stdout);
#endif

        sem_wait(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);

#ifdef _PRINT_TEST
        printf("TEST - COMPUTING TOTAL FOR IMAGE ID %ld : %s\n",
               aoloopcontrol_var.aoconfID_imWFS0,
               data.image[aoloopcontrol_var.aoconfID_imWFS0].md[0].name);
        fflush(stdout);
#endif

        imtotalcnt++;

        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].write = 1;
        IMTOTAL                                                      = 0.0;
        if(aoloopcontrol_var.aoconfID_wfsmask != -1)
        {
            for(ii = 0; ii < nelem; ii++)
            {
                IMTOTAL +=
                    data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii] *
                    data.image[aoloopcontrol_var.aoconfID_wfsmask].array.F[ii];
            }
        }
        else
        {
            for(ii = 0; ii < nelem; ii++)
            {
                IMTOTAL +=
                    data.image[aoloopcontrol_var.aoconfID_imWFS0].array.F[ii];
            }
        }
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0] = IMTOTAL;

        AOconf[LOOPNUMBER].WFSim.WFStotalflux = IMTOTAL;

        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt1 =
            data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;

        COREMOD_MEMORY_image_set_sempost_byID(
            aoloopcontrol_var.aoconfID_imWFS0tot,
            -1);
        data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].write = 0;
    }

    // LOG function / process end
    // CORE_logFunctionCall( logfunc_level, logfunc_level_max, 1, __FILE__,
    // __func__, __LINE__, commentstring);

    return NULL;
}
*/



/*
static void *compute_function_dark_subtract(void *ptr)
{
    long  ii, iistart, iiend;
    long *index;
    long  threadindex;
    int   semval;

    long nelem;
    int  WFSatype;

    // connect to WFS image
    char WFSname[100];
    sprintf(WFSname, "aol%ld_wfsim", LOOPNUMBER);
    long ID_wfsim = read_sharedmem_image(WFSname);
    if(ID_wfsim == -1)
    {
        printf("ERROR: cannot connect to WFS stream\n");
        exit(0);
    }

    WFSatype = data.image[ID_wfsim].md[0].datatype;

    // connect to imWFS0
    char sname[100];
    sprintf(sname, "aol%ld_imWFS0", LOOPNUMBER);
    long ID_imWFS0 = read_sharedmem_image(sname);
    if(ID_imWFS0 == -1)
    {
        printf("ERROR: cannot connect to WFS stream");
        exit(0);
    }

    nelem = data.image[ID_imWFS0].md[0].size[0] *
            data.image[ID_imWFS0].md[0].size[1];

    WFSatype = data.image[ID_wfsim].md[0].datatype;

    index       = (long *) ptr;
    threadindex = *index;

    iistart = (long)((threadindex) * nelem / COMPUTE_DARK_SUBTRACT_NBTHREADS);
    iiend =
        (long)((threadindex + 1) * nelem / COMPUTE_DARK_SUBTRACT_NBTHREADS);

    // LOG function / process start
    // int logfunc_level = 0;
    // int logfunc_level_max = 1;
    char commentstring[200];
    sprintf(commentstring, "Dark subtract WFS image, loop %ld", LOOPNUMBER);
    // CORE_logFunctionCall( logfunc_level, logfunc_level_max, 0, __FILE__,
    // __func__, __LINE__, commentstring);

    while(1)
    {
        sem_wait(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[threadindex]);

        switch(WFSatype)
        {
            case _DATATYPE_UINT16:
                for(ii = iistart; ii < iiend; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        ((float) arrayutmp[ii]) -
                        data.image[Average_cam_frames_IDdark].array.F[ii];
                }
                break;
            case _DATATYPE_INT16:
                for(ii = iistart; ii < iiend; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        ((float) arraystmp[ii]) -
                        data.image[Average_cam_frames_IDdark].array.F[ii];
                }
                break;
            case _DATATYPE_FLOAT:
                for(ii = iistart; ii < iiend; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        ((float) arrayftmp[ii]) -
                        data.image[Average_cam_frames_IDdark].array.F[ii];
                }
                break;
            default:
                printf("ERROR: WFS data type not recognized\n File %s, line %d\n",
                       __FILE__,
                       __LINE__);
                printf("datatype = %d\n", aoloopcontrol_var.WFSatype);
                exit(0);
                break;
        }

        sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[threadindex],
                     &semval);
        if(semval < SEMAPHORE_MAXVAL)
        {
            sem_post(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[threadindex]);
        }
    }

    // LOG function / process end
    // CORE_logFunctionCall( logfunc_level, logfunc_level_max, 1, __FILE__,
    // __func__, __LINE__, commentstring);
}

*/






errno_t AOcontrolLoop_IOtools_acquireWFSloop_FPCONF()
{
    // ===========================
    // SETUP FPS
    // ===========================
    FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);

    //FPS2PROCINFOMAP fps2procinfo;
    fps_add_processinfo_entries(&fps); // include real-time settings

    // ===========================
    // ALLOCATE FPS ENTRIES IF NOT ALREADY EXIST
    // ===========================

    long loop_default[4] = {0, 0, 10, 0};
    // long fpi_loop =
    function_parameter_add_entry(&fps,
                                 ".loop",
                                 "loop index",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &loop_default,
                                 NULL);

    long semindex_default[4] = {1, 0, 10, 1};
    // long fpi_semindex =
    function_parameter_add_entry(&fps,
                                 ".semindex",
                                 "input semaphore index",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &semindex_default,
                                 NULL);

    // long fpi_out_fluxtotal =
    function_parameter_add_entry(&fps,
                                 ".out.fluxtotal",
                                 "total flux in WFS image",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_OUTPUT,
                                 NULL,
                                 NULL);

    // long fpi_out_GPUalpha =
    function_parameter_add_entry(&fps,
                                 ".out.GPUalpha",
                                 "GPU alpha coefficient",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_OUTPUT,
                                 NULL,
                                 NULL);

    // long fpi_out_GPUbeta =
    function_parameter_add_entry(&fps,
                                 ".out.GPUbeta",
                                 "GPU beta coefficient",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_OUTPUT,
                                 NULL,
                                 NULL);

    long wfsnormalize_default[4] = {1, 0, 1, 1};
    // long fpi_wfsnormalize =
    function_parameter_add_entry(&fps,
                                 ".WFSnormalize",
                                 "normalize WFS frames",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,
                                 &wfsnormalize_default,
                                 NULL);

    double wfsnormfloor_default[4] = {0.0, 0, 100000, 0.01};
    // long fpi_wfsnormfloor =
    function_parameter_add_entry(&fps,
                                 ".WFSnormfloor",
                                 "WFS flux floor for normalize",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,
                                 &wfsnormfloor_default,
                                 NULL);

    long compdark_default[4] = {1, 0, 1, 1};
    // long fpi_comp_darksub =
    function_parameter_add_entry(&fps,
                                 ".comp.darksub",
                                 "Subtract Dark",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,
                                 &compdark_default,
                                 NULL);

    long compimtotal_default[4] = {1, 0, 1, 1};
    // long fpi_comp_imtotal =
    function_parameter_add_entry(&fps,
                                 ".comp.imtotal",
                                 "Compute WFS frame total flux",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,
                                 &compimtotal_default,
                                 NULL);

    long compnormwfsim_default[4] = {1, 0, 1, 1};
    // long fpi_comp_normwfsim =
    function_parameter_add_entry(&fps,
                                 ".comp.normwfsim",
                                 "Compute normalized WFS frame",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,
                                 &compnormwfsim_default,
                                 NULL);

    FPS_CONFLOOP_START // macro in function_parameter.h

    FPS_CONFLOOP_END // macro in function_parameter.h

    return RETURN_SUCCESS;
}





/*errno_t AOcontrolLoop_IOtools_acquireWFSloop_RUN()
{
    // ===========================
    // CONNECT TO FPS
    // ===========================


    FPS_CONNECT(data.FPS_name, FPSCONNECT_RUN);

    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================
    long loop = functionparameter_GetParamValue_INT64(&fps, ".loop");

    long *semindex;
    semindex = functionparameter_GetParamPtr_INT64(&fps, ".semindex");

    double *WFSfluxtotal;
    WFSfluxtotal =
        functionparameter_GetParamPtr_FLOAT64(&fps, ".out.fluxtotal");

    double *GPU_alpha;
    GPU_alpha = functionparameter_GetParamPtr_FLOAT64(&fps, ".out.GPUalpha");

    double *GPU_beta;
    GPU_beta = functionparameter_GetParamPtr_FLOAT64(&fps, ".out.GPUbeta");

    uint64_t *FPFLAG_IMNORMALIZE;
    FPFLAG_IMNORMALIZE =
        functionparameter_GetParamPtr_fpflag(&fps, ".WFSnormalize");

    double *WFSnormfloor;
    WFSnormfloor = functionparameter_GetParamPtr_FLOAT64(&fps, ".WFSnormfloor");

    uint64_t *FPFLAG_COMPUTE_DARKSUBTRACT;
    FPFLAG_COMPUTE_DARKSUBTRACT =
        functionparameter_GetParamPtr_fpflag(&fps, ".comp.darksub");

    uint64_t *FPFLAG_COMPUTE_IMTOTAL;
    FPFLAG_COMPUTE_IMTOTAL =
        functionparameter_GetParamPtr_fpflag(&fps, ".comp.imtotal");

    uint64_t *FPFLAG_COMPUTE_NORMWFSIM;
    FPFLAG_COMPUTE_NORMWFSIM =
        functionparameter_GetParamPtr_fpflag(&fps, ".comp.normwfsim");

    // ===========================
    // ### processinfo support
    // ===========================

    PROCESSINFO *processinfo;

    char pinfodescr[200];
    sprintf(pinfodescr, "process aol%ld_wfsim", loop);

    processinfo =
        processinfo_setup(data.FPS_name, // re-use fpsname as processinfo name
                          pinfodescr,    // description
                          "writes imWFS0 imWFS1", // message on startup
                          __FUNCTION__,
                          __FILE__,
                          __LINE__);

    // OPTIONAL SETTINGS
    processinfo->MeasureTiming = 1; // Measure timing

    // RT_priority, 0-99. Larger number = higher priority. If <0, ignore
    processinfo->RT_priority = 20;

    processinfo->loopcntMax = -1; // max number of iterations. -1 if infinite

    float          *arrayftmp;
    unsigned short *arrayutmp;
    signed short   *arraystmp;

    // long    imWaitTimeAvecnt = 0;
    // long    imWaitTimeAvecnt0 = 1000;
    // double  imWaitTimeAve = 0.0;

    char *ptrv;

    // =============================================
    // INIT AND TESTING
    // =============================================
    // Pre-loop testing, anything that would prevent loop from starting should
    // issue message
    int loopOK = 1;

    // connect to WFS image
    char WFSname[100];
    sprintf(WFSname, "aol%ld_wfsim", loop);
    long ID_wfsim = read_sharedmem_image(WFSname);
    if(ID_wfsim == -1)
    {
        processinfo_error(processinfo, "ERROR: no WFS input");
        return RETURN_FAILURE;
    }
    uint32_t sizexWFS = data.image[ID_wfsim].md[0].size[0];
    uint32_t sizeyWFS = data.image[ID_wfsim].md[0].size[1];
    uint64_t sizeWFS  = sizexWFS * sizeyWFS;
    uint8_t  WFSatype = data.image[ID_wfsim].md[0].datatype;

    char name[200];
    if(sprintf(name, "aol%ld_imWFS0", loop) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }
    long ID_imWFS0 = AOloopControl_IOtools_2Dloadcreate_shmim(name,
                     " ",
                     sizexWFS,
                     sizeyWFS,
                     0.0);
    if(sprintf(name, "aol%ld_imWFS1", loop) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }
    long ID_imWFS1 = AOloopControl_IOtools_2Dloadcreate_shmim(name,
                     " ",
                     sizexWFS,
                     sizeyWFS,
                     0.0);

    int wfsim_semwaitindex =
        ImageStreamIO_getsemwaitindex(&data.image[ID_wfsim], *semindex);
    if(wfsim_semwaitindex > -1)
    {
        *semindex = wfsim_semwaitindex;
    }

    // initialize camera averaging arrays if not already done
    if(WFSatype == _DATATYPE_FLOAT)
    {
        arrayftmp = (float *) malloc(sizeof(float) * sizeWFS);
        if(arrayftmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }

    if(WFSatype == _DATATYPE_UINT16)
    {
        arrayutmp = (unsigned short *) malloc(sizeof(unsigned short) * sizeWFS);
        if(arrayutmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }

    if(WFSatype == _DATATYPE_INT16)
    {
        arraystmp = (signed short *) malloc(sizeof(signed short) * sizeWFS);
        if(arraystmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }

    if(sprintf(Average_cam_frames_dname, "aol%ld_wfsdark", loop) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    Average_cam_frames_IDdark = image_ID(Average_cam_frames_dname);
    Average_cam_frames_nelem  = sizeWFS;

    // set semaphore to 0
    int semval;
    sem_getvalue(data.image[ID_wfsim].semptr[*semindex], &semval);
    printf("INITIALIZING SEMAPHORE %ld   %s   (%d)\n",
           *semindex,
           data.image[ID_wfsim].md[0].name,
           semval);
    for(int i = 0; i < semval; i++)
    {
        sem_trywait(data.image[ID_wfsim].semptr[*semindex]);
    }

    // Specify input stream trigger
    processinfo_waitoninputstream_init(processinfo,
                                       ID_wfsim,
                                       PROCESSINFO_TRIGGERMODE_SEMAPHORE,
                                       -1);

    // ===========================
    // ### Start loop
    // ===========================

    processinfo_loopstart(
        processinfo); // Notify processinfo that we are entering loop

    // unsigned long long WFScnt  = 0;
    int slice;

    while(loopOK == 1)
    {
        loopOK = processinfo_loopstep(processinfo);
        processinfo_waitoninputstream(processinfo);


        processinfo_exec_start(processinfo);
        if(processinfo_compute_status(processinfo) == 1)
        {

            // ===========================================
            // COPY FRAME TO LOCAL MEMORY BUFFER
            // ===========================================

            slice = 0;
            if(data.image[ID_wfsim].md[0].naxis == 3)  // ring buffer
            {
                slice = data.image[ID_wfsim].md[0].cnt1;
                if(slice == -1)
                {
                    slice = data.image[ID_wfsim].md[0].size[2];
                }
            }

            DEBUG_TRACEPOINT(" ");

            switch(WFSatype)
            {

                case _DATATYPE_FLOAT:
                    ptrv = (char *) data.image[ID_wfsim].array.F;
                    ptrv += sizeof(float) * slice * sizeWFS;
                    memcpy(arrayftmp, ptrv, sizeof(float) * sizeWFS);
                    break;

                case _DATATYPE_UINT16:
                    ptrv = (char *) data.image[ID_wfsim].array.UI16;
                    ptrv += sizeof(unsigned short) * slice * sizeWFS;
                    memcpy(arrayutmp, ptrv, sizeof(unsigned short) * sizeWFS);
                    break;

                case _DATATYPE_INT16:
                    ptrv = (char *) data.image[ID_wfsim].array.SI16;
                    ptrv += sizeof(signed short) * slice * sizeWFS;
                    memcpy(arraystmp, ptrv, sizeof(signed short) * sizeWFS);
                    break;

                default:
                    printf("ERROR: DATA TYPE NOT SUPPORTED\n");
                    exit(0);
                    break;
            }

            // WFScnt = data.image[ID_wfsim].md[0].cnt0;

            // ===========================================
            // SUBTRACT DARK -> imWFS0
            // ===========================================
            DEBUG_TRACEPOINT(" ");
            if((*FPFLAG_COMPUTE_DARKSUBTRACT) & FPFLAG_ONOFF)
            {
                data.image[ID_imWFS0].md[0].write = 1;
                DEBUG_TRACEPOINT(" ");
                switch(WFSatype)
                {

                    case _DATATYPE_UINT16:
                        if(Average_cam_frames_IDdark == -1)
                        {
                            for(uint64_t ii = 0; ii < Average_cam_frames_nelem;
                                    ii++)
                            {
                                data.image[ID_imWFS0].array.F[ii] =
                                    ((float) arrayutmp[ii]);
                            }
                        }
                        else
                        {
                            for(uint64_t ii = 0; ii < Average_cam_frames_nelem;
                                    ii++)
                            {
                                data.image[ID_imWFS0].array.F[ii] =
                                    ((float) arrayutmp[ii]) -
                                    data.image[Average_cam_frames_IDdark]
                                    .array.F[ii];
                            }
                        }
                        break;

                    case _DATATYPE_INT16:

                        if(Average_cam_frames_IDdark == -1)
                        {
                            for(uint64_t ii = 0; ii < Average_cam_frames_nelem;
                                    ii++)
                            {
                                data.image[ID_imWFS0].array.F[ii] =
                                    ((float) arraystmp[ii]);
                            }
                        }
                        else
                        {
                            for(uint64_t ii = 0; ii < Average_cam_frames_nelem;
                                    ii++)
                            {
                                data.image[ID_imWFS0].array.F[ii] =
                                    ((float) arraystmp[ii]) -
                                    data.image[Average_cam_frames_IDdark]
                                    .array.F[ii];
                            }
                        }
                        break;

                    case _DATATYPE_FLOAT:
                        if(Average_cam_frames_IDdark == -1)
                        {
                            memcpy(data.image[ID_imWFS0].array.F,
                                   arrayftmp,
                                   sizeof(float) * Average_cam_frames_nelem);
                        }
                        else
                        {
                            for(uint64_t ii = 0; ii < Average_cam_frames_nelem;
                                    ii++)
                            {
                                data.image[ID_imWFS0].array.F[ii] =
                                    arrayftmp[ii] -
                                    data.image[Average_cam_frames_IDdark]
                                    .array.F[ii];
                            }
                        }
                        break;

                    default:
                        printf(
                            "ERROR: WFS data type not recognized\n File %s, "
                            "line %d\n",
                            __FILE__,
                            __LINE__);
                        printf("datatype = %d\n", WFSatype);
                        exit(0);
                        break;
                }
                processinfo_update_output_stream(processinfo, ID_imWFS0);
                // data.image[ID_imWFS0].md[0].cnt1 =
                // data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
                //                 COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS0,
                //                 -1); data.image[ID_imWFS0].md[0].cnt0 ++;
                //                 data.image[ID_imWFS0].md[0].write = 0;
            }

            DEBUG_TRACEPOINT(" ");

            // ===========================================
            // COMPUTE TOTAL -> WFSfluxtotal
            // ===========================================
            double IMTOTAL;
            if((*FPFLAG_COMPUTE_IMTOTAL) & FPFLAG_ONOFF)
            {
                uint64_t nelem = data.image[ID_imWFS0].md[0].size[0] *
                                 data.image[ID_imWFS0].md[0].size[1];
                IMTOTAL = 0.0;
                if(aoloopcontrol_var.aoconfID_wfsmask != -1)
                {
                    for(uint64_t ii = 0; ii < nelem; ii++)
                    {
                        IMTOTAL +=
                            data.image[ID_imWFS0].array.F[ii] *
                            data.image[aoloopcontrol_var.aoconfID_wfsmask]
                            .array.F[ii];
                    }
                }
                else
                {
                    for(uint64_t ii = 0; ii < nelem; ii++)
                    {
                        IMTOTAL += data.image[ID_imWFS0].array.F[ii];
                    }
                }
                *WFSfluxtotal = IMTOTAL;
            }

            DEBUG_TRACEPOINT(" ");

            // ===========================================
            // NORMALIZE -> imWFS1
            // ===========================================

            double totalinv;
            double normfloorcoeff;

            if((*FPFLAG_IMNORMALIZE) & FPFLAG_ONOFF)
            {
                DEBUG_TRACEPOINT(" ");
                totalinv = 1.0 / (*WFSfluxtotal + *WFSnormfloor * sizeWFS);
                normfloorcoeff =
                    *WFSfluxtotal / (*WFSfluxtotal + *WFSnormfloor * sizeWFS);
            }
            else
            {
                DEBUG_TRACEPOINT(" ");
                totalinv       = 1.0;
                normfloorcoeff = 1.0;
            }

            DEBUG_TRACEPOINT(" ");
            *GPU_alpha = totalinv;

            *GPU_beta = -normfloorcoeff;

            if((*FPFLAG_COMPUTE_NORMWFSIM) &
                    FPFLAG_ONOFF) // normalize WFS image by totalinv
            {

                data.image[ID_imWFS1].md[0].write = 1;

                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS1].array.F[ii] =
                        data.image[ID_imWFS0].array.F[ii] * totalinv;
                }

                processinfo_update_output_stream(processinfo, ID_imWFS1);

                //                COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS1,
                //                -1); data.image[ID_imWFS1].md[0].cnt0 ++;
                //                data.image[ID_imWFS1].md[0].write = 0;
            }
        }

        DEBUG_TRACEPOINT(" ");

        // Post semaphore(s) and counter(s)
        // computation done

        // process signals, increment loop counter
        processinfo_exec_end(processinfo);

        // OPTIONAL: MESSAGE WHILE LOOP RUNNING
        processinfo_WriteMessage(processinfo, "loop running");
    }
    // ==================================
    // ### ENDING LOOP
    // ==================================

    processinfo_cleanExit(processinfo);
    function_parameter_RUNexit(&fps);

    if(WFSatype == _DATATYPE_FLOAT)
    {
        free(arrayftmp);
    }

    if(WFSatype == _DATATYPE_UINT16)
    {
        free(arrayutmp);
    }

    if(WFSatype == _DATATYPE_INT16)
    {
        free(arraystmp);
    }

    return RETURN_SUCCESS;
}



















errno_t AOcontrolLoop_IOtools_acquireWFSloop(long loop)
{
    char fpsname[200];

    long pindex = (long)
                  getpid(); // index used to differentiate multiple calls to function
    // if we don't have anything more informative, we use PID

    // int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps;

    // create FPS
    sprintf(fpsname, "acquWFS-%06ld", pindex);
    AOcontrolLoop_IOtools_acquireWFSloop_FPCONF(fpsname, FPSCMDCODE_FPSINIT);

    function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_SIMPLE);
    functionparameter_SetParamValue_INT64(&fps, ".loop", loop);
    function_parameter_struct_disconnect(&fps);

    AOcontrolLoop_IOtools_acquireWFSloop_RUN(fpsname);

    return RETURN_SUCCESS;
}

/** @brief Read image from WFS camera
 *
 * ## Purpose
 *
 * Reads WFS image and performs some basic processing
 *
 * Outputs are :
 * imWFS0  dark-subtracted
 * imWFS1  dark-subtracted and normalized, but not reference-subtracted.
 * imWFS2  dark-subtracted, normzlized and reference-subtracted
 * imWFS3 time-averaged
 *
 * supports ring buffer
 * puts image from camera buffer aoloopcontrol_var.aoconfID_wfsim into
 * aoloopcontrol_var.aoconfID_imWFS1 (supplied by user)
 *
 * RM = 1 if response matrix
 *
 * if normalize == 1, image is normalized by dividing by (total +
 * AOconf[loop].WFSim.WFSnormfloor)*AOconf[loop].WFSim.WFSsize if
 * PixelStreamMode = 1, read on semaphore 1, return slice index
 *
 */
/*
errno_t __attribute__((hot))
Read_cam_frame(long                        loop,
               int                         RM,
               int                         normalize,
               __attribute__((unused)) int PixelStreamMode,
               int                         InitSem)
{
    // long         imcnt;
    double totalinv;
    char   name[200];
    int    slice;
    char  *ptrv;
    // long double  tmplv1;
    // double       tmpf;
    // imageID      IDdark;
    // char         dname[200];
    uint64_t  nelem;
    pthread_t thread_computetotal_id;
    pthread_t thread_dark_subtract[20];
    // float        resulttotal;
    int sval0, sval;
    // void        *status = 0;
    long i;
    int  semval;
    // int          s;

    int semindex = 1;

    struct timespec functionTestTimerStart;
    struct timespec functionTestTimerEnd;
    struct timespec functionTestTimer00;
    // struct timespec functionTestTimer01;
    // struct timespec functionTestTimer02;
    // struct timespec functionTestTimer03;
    // struct timespec functionTestTimer04;

    static long   imWaitTimeAvecnt  = 0;
    static long   imWaitTimeAvecnt0 = 1000;
    static double imWaitTimeAve     = 0.0;

    if(RM == 0)
    {
        semindex = 8;
    }
    else
    {
        semindex = 9;
    }

    static int wfsim_semwaitindex = -1;

    // when first called, the function needs to initialize static variables
    static int  functionINIT = 0;
    static long ID_wfsim     = -1;
    static long sizexWFS     = 0;
    static long sizeyWFS     = 0;
    static long sizeWFS      = 0;
    static int  WFSatype     = -1;

    static long ID_imWFS0;

    static unsigned long long WFScnt   = 0;
    static unsigned long long WFScntRM = 0;

    DEBUG_TRACEPOINT(" ");

    // ======================= INITIALIZATION ========================
    if(functionINIT == 0)
    {
        // connect to WFS image
        char WFSname[100];
        sprintf(WFSname, "aol%ld_wfsim", loop);
        ID_wfsim = read_sharedmem_image(WFSname);
        if(ID_wfsim == -1)
        {
            printf("ERROR: cannot connect to WFS stream\n");
            exit(0);
        }
        sizexWFS = data.image[ID_wfsim].md[0].size[0];
        sizeyWFS = data.image[ID_wfsim].md[0].size[1];
        sizeWFS  = sizexWFS * sizeyWFS;
        WFSatype = data.image[ID_wfsim].md[0].datatype;

        if(sprintf(name, "aol%ld_imWFS0", loop) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }
        ID_imWFS0 = AOloopControl_IOtools_2Dloadcreate_shmim(name,
                    " ",
                    sizexWFS,
                    sizeyWFS,
                    0.0);

        if(wfsim_semwaitindex == -1)
        {
            wfsim_semwaitindex =
                ImageStreamIO_getsemwaitindex(&data.image[ID_wfsim], semindex);
        }
        if(wfsim_semwaitindex > -1)
        {
            semindex = wfsim_semwaitindex;
        }

        // initialize camera averaging arrays if not already done
        arrayftmp = (float *) malloc(sizeof(float) * sizeWFS);
        if(arrayftmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
        arrayutmp = (unsigned short *) malloc(sizeof(unsigned short) * sizeWFS);
        if(arrayutmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
        arraystmp = (signed short *) malloc(sizeof(signed short) * sizeWFS);
        if(arraystmp == NULL)
        {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }

        if(sprintf(Average_cam_frames_dname, "aol%ld_wfsdark", loop) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        Average_cam_frames_IDdark = image_ID(Average_cam_frames_dname);
        Average_cam_frames_nelem  = sizeWFS;

        // set semaphore to 0
        sem_getvalue(data.image[ID_wfsim].semptr[semindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n",
               semindex,
               data.image[ID_wfsim].md[0].name,
               semval);
        for(i = 0; i < semval; i++)
        {
            sem_trywait(data.image[ID_wfsim].semptr[semindex]);
        }

        functionINIT = 1;
    }

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    DEBUG_TRACEPOINT(" ");



    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    DEBUG_TRACEPOINT(" ");

    // initialize camera averaging arrays if not already done
  
    DEBUG_TRACEPOINT(" ");

    if(InitSem == 1)
    {
        sem_getvalue(data.image[ID_wfsim].semptr[wfsim_semwaitindex], &semval);
        printf("INITIALIZING SEMAPHORE %d   %s   (%d)\n",
               semindex,
               data.image[ID_wfsim].md[0].name,
               semval);
        for(i = 0; i < semval; i++)
        {
            sem_trywait(data.image[ID_wfsim].semptr[wfsim_semwaitindex]);
        }
    }

    DEBUG_TRACEPOINT(" ");

#ifdef _PRINT_TEST
    printf("TEST - SEMAPHORE INITIALIZED\n");
    fflush(stdout);
#endif

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    if(RM == 0)
    {
        //  AOconf[loop].AOtiminginfo.status = 20;  // 020: WAIT FOR IMAGE
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[24] = tdiffv;
    }
    else
    {
        data.status1 = 2;
    }

    clock_gettime(CLOCK_MILK, &functionTestTimer00);

    // char pmsg[200];
    // sprintf(pmsg, "waiting %s update [sem %d]",
    // data.image[ID_wfsim].md[0].name, wfsim_semwaitindex);
    // processinfo_WriteMessage(data.pinfo, pmsg);

    // ***********************************************************************************************
    // WAITING FOR WFS FRAME
    // listening for counter or semaphore in wfsim
    // ***********************************************************************************************

    DEBUG_TRACEPOINT(" ");

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

#ifdef _PRINT_TEST
    printf("TEST - WAITING FOR IMAGE %s\n", data.image[ID_wfsim].md[0].name);
    fflush(stdout);
#endif

    if(data.image[ID_wfsim].md[0].sem == 0)  // don't use semaphore
    {
#ifdef _PRINT_TEST
        printf("TEST - NOT USING SEMAPHORE\n");
        fflush(stdout);
#endif

        // if not using semaphore, use counter to test if new WFS frame is ready
        if(RM == 0)
            while(WFScnt ==
                    data.image[ID_wfsim].md[0].cnt0) // test if new frame exists
            {
                usleep(5);
            }
        else
            while(WFScntRM ==
                    data.image[ID_wfsim].md[0].cnt0) // test if new frame exists
            {
                usleep(5);
            }
    }
    else
    {
#ifdef _PRINT_TEST
        printf("TEST - waiting on semindex = %d\n", semindex);
        fflush(stdout);
#endif

        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);

        sem_getvalue(data.image[ID_wfsim].semptr[semindex], &semval);
        if(semval > 0)
        {
            if(semval > 1)
            {
                // printf("\n\033[31;1m[%12ld] WARNING [%d] WFS SEMAPHORE already
                // posted
                // - Missed frame\033[0m\n", AOconf[loop].aorun.LOOPiteration,
                // semval);
                printf(
                    "\n\033[31;1m WARNING [%d] WFS SEMAPHORE already posted "
                    "- Missed frame\033[0m\n",
                    semval);
            }
            fflush(stdout);
        }

        int rval;
        rval = ImageStreamIO_semwait(&data.image[ID_wfsim], wfsim_semwaitindex);

        if(rval == -1)
        {
            perror("sem_timedwait");
        }

        sem_getvalue(data.image[ID_wfsim].semptr[semindex], &semval);
        for(i = 0; i < semval; i++)
        {
            //			printf("WARNING: [%d] sem_trywait on
            // ID_wfsim\n", (int) (semval - i)); 			fflush(stdout);
            // sem_trywait(data.image[ID_wfsim].semptr[semindex]);
            ImageStreamIO_semtrywait(&data.image[ID_wfsim], wfsim_semwaitindex);
        }

        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);

#ifdef _PRINT_TEST
        printf("TEST - semaphore posted\n");
        fflush(stdout);
#endif
    }

#ifdef _PRINT_TEST
    printf("TEST - IMAGE RECEIVED - PROCEEDING\n");
    fflush(stdout);
#endif

    DEBUG_TRACEPOINT(" ");

    if(data.processinfo == 1)
        if(data.pinfo->MeasureTiming == 1)
        {
            processinfo_exec_start(data.pinfo);
        }

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    DEBUG_TRACEPOINT(" ");

    clock_gettime(CLOCK_MILK, &functionTestTimerStart);

    // ***********************************************************************************************
    // WHEN NEW IMAGE IS READY, COPY IT TO LOCAL ARRAY (arrayftmp, arrayutmp or
    // arraystmp)
    // ***********************************************************************************************

    if(RM == 0)
    {
        clock_gettime(CLOCK_MILK, &tnow);
        aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_wfsim] =
            1; // there must only be one such process
        AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_wfsim, tnow);

        // AOconf[loop].AOtiminginfo.status = 0;  // LOAD IMAGE
    }

    // AOconf[loop].AOtiminginfo.statusM = 0;

    slice = 0;
    if(data.image[ID_wfsim].md[0].naxis == 3)  // ring buffer
    {
        slice = data.image[ID_wfsim].md[0].cnt1;
        if(slice == -1)
        {
            slice = data.image[ID_wfsim].md[0].size[2];
        }
    }

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    DEBUG_TRACEPOINT(" ");

    switch(WFSatype)
    {

        case _DATATYPE_FLOAT:
            ptrv = (char *) data.image[ID_wfsim].array.F;
            ptrv += sizeof(float) * slice * sizeWFS; // AOconf[loop].WFSim.sizeWFS;
            memcpy(arrayftmp,
                   ptrv,
                   sizeof(float) * sizeWFS); // AOconf[loop].WFSim.sizeWFS);
            break;

        case _DATATYPE_UINT16:
            ptrv = (char *) data.image[ID_wfsim].array.UI16;
            ptrv += sizeof(unsigned short) * slice *
                    sizeWFS; // AOconf[loop].WFSim.sizeWFS;
            memcpy(arrayutmp,
                   ptrv,
                   sizeof(unsigned short) *
                   sizeWFS); // AOconf[loop].WFSim.sizeWFS);
            break;

        case _DATATYPE_INT16:
            ptrv = (char *) data.image[ID_wfsim].array.SI16;
            ptrv += sizeof(signed short) * slice *
                    sizeWFS; // AOconf[loop].WFSim.sizeWFS;
            memcpy(arraystmp,
                   ptrv,
                   sizeof(signed short) * sizeWFS); // AOconf[loop].WFSim.sizeWFS);
            break;

        default:
            printf("ERROR: DATA TYPE NOT SUPPORTED\n");
            exit(0);
            break;
    }

    //	printf("WFS size = %ld\n", AOconf[loop].WFSim.sizeWFS);
    //	fflush(stdout);

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    if(RM == 0)
    {
        WFScnt = data.image[ID_wfsim].md[0].cnt0;
    }
    else
    {
        WFScntRM = data.image[ID_wfsim].md[0].cnt0;
    }

    //   if(COMPUTE_PIXELSTREAMING==1) // multiple pixel groups
    aoloopcontrol_var.PIXSTREAM_SLICE = data.image[ID_wfsim].md[0].cnt1;

    DEBUG_TRACEPOINT(" ");

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    // ===================================================================
    //
    // THIS IS THE STARTING POINT FOR THE AO LOOP TIMING
    //
    // ===================================================================
    if(RM == 0)  // some timing work
    {
        AOconf[loop].AOtiminginfo.status = 1; // 3->001: DARK SUBTRACT
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[0] = tdiffv;

        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].write = 1;
        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime = tnow;
        COREMOD_MEMORY_image_set_sempost_byID(
            aoloopcontrol_var.aoconfID_looptiming,
            -1);
        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].write = 0;
    }

#ifdef _PRINT_TEST
    printf("TEST - DARK SUBTRACT\n");
    fflush(stdout);
#endif

    DEBUG_TRACEPOINT(" ");

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    // ***********************************************************************************************
    // DARK SUBTRACT AND COMPUTE IMAGE TOTAL
    // output is imWFS0 (dark subtracted) and imWFS1 (normalized)
    // ***********************************************************************************************

    if((loop == 0) ||
            (RM == 1)) // single thread, in CPU  //WHY do CPU-based if loop=0 ?
    {
        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);

#ifdef _PRINT_TEST
        printf(
            "TEST - DARK SUBTRACT - single thread, in CPU   loop=%ld  RM=%d\n",
            loop,
            RM);
        fflush(stdout);
#endif

        switch(WFSatype)
        {

            case _DATATYPE_UINT16:
                printf("================== TEST POINT LINE %d\n", __LINE__);
                fflush(stdout);
                if(Average_cam_frames_IDdark == -1)
                {
                    for(uint64_t ii = 0; ii < Average_cam_frames_nelem; ii++)
                    {
                        data.image[ID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]);
                    }
                }
                else
                {
                    //# ifdef _OPENMP
                    //            #pragma omp parallel num_threads(4) if
                    //            (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
                    //        {
                    //# endif

                    //# ifdef _OPENMP
                    //            #pragma omp for
                    //# endif
                    for(uint64_t ii = 0; ii < Average_cam_frames_nelem; ii++)
                    {
                        data.image[ID_imWFS0].array.F[ii] =
                            ((float) arrayutmp[ii]) -
                            data.image[Average_cam_frames_IDdark].array.F[ii];
                    }
                    //# ifdef _OPENMP
                    //        }
                    //# endif
                }
                break;

            case _DATATYPE_INT16:
                printf("================== TEST POINT LINE %d\n", __LINE__);
                fflush(stdout);
                if(Average_cam_frames_IDdark == -1)
                {
                    for(uint64_t ii = 0; ii < Average_cam_frames_nelem; ii++)
                    {
                        data.image[ID_imWFS0].array.F[ii] = ((float) arraystmp[ii]);
                    }
                }
                else
                {
                    //# ifdef _OPENMP
                    //            #pragma omp parallel num_threads(4) if
                    //            (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
                    //        {
                    //# endif

                    //# ifdef _OPENMP
                    //            #pragma omp for
                    //# endif
                    for(uint64_t ii = 0; ii < Average_cam_frames_nelem; ii++)
                    {
                        data.image[ID_imWFS0].array.F[ii] =
                            ((float) arraystmp[ii]) -
                            data.image[Average_cam_frames_IDdark].array.F[ii];
                    }
                    //# ifdef _OPENMP
                    //        }
                    //# endif
                }
                break;

            case _DATATYPE_FLOAT:
                printf("================== TEST POINT LINE %d   ID dark = %ld\n",
                       __LINE__,
                       Average_cam_frames_IDdark);
                fflush(stdout);
                if(Average_cam_frames_IDdark == -1)
                {
                    memcpy(data.image[ID_imWFS0].array.F,
                           arrayftmp,
                           sizeof(float) * Average_cam_frames_nelem);
                }
                else
                {
                    //# ifdef _OPENMP
                    //            #pragma omp parallel num_threads(4) if
                    //            (Average_cam_frames_nelem>OMP_NELEMENT_LIMIT)
                    //        {
                    //# endif

                    //# ifdef _OPENMP
                    //            #pragma omp parallel for
                    //# endif
                    for(uint64_t ii = 0; ii < Average_cam_frames_nelem; ii++)
                    {
                        data.image[ID_imWFS0].array.F[ii] =
                            arrayftmp[ii] -
                            data.image[Average_cam_frames_IDdark].array.F[ii];
                    }
                    //# ifdef _OPENMP
                    //        }
                    //# endif
                }
                break;

            default:
                printf("================== TEST POINT LINE %d\n", __LINE__);
                fflush(stdout);
                printf("ERROR: WFS data type not recognized\n File %s, line %d\n",
                       __FILE__,
                       __LINE__);
                printf("datatype = %d\n", WFSatype);
                exit(0);
                break;
        }

        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);

        // data.image[ID_imWFS0].md[0].cnt1 =
        // data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS0, -1);

        clock_gettime(CLOCK_MILK, &tnow);



        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);
    }
    else
    {
        printf("================== TEST POINT LINE %d\n", __LINE__);
        fflush(stdout);

#ifdef _PRINT_TEST
        printf("TEST - DARK SUBTRACT - START  (init = %d, %d threads)\n",
               AOLCOMPUTE_DARK_SUBTRACT_THREADinit,
               COMPUTE_DARK_SUBTRACT_NBTHREADS);
        fflush(stdout);
#endif

        if(AOLCOMPUTE_DARK_SUBTRACT_THREADinit == 0)
        {
#ifdef _PRINT_TEST
            printf("TEST - DARK SUBTRACT - CREATE %d THREADS\n",
                   COMPUTE_DARK_SUBTRACT_NBTHREADS);
            fflush(stdout);
#endif

            ti = 0;

            while(ti < COMPUTE_DARK_SUBTRACT_NBTHREADS)
            {
                pthread_create(&thread_dark_subtract[ti],
                               NULL,
                               compute_function_dark_subtract,
                               (void *) &ti);
                sem_init(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], 0, 0);
                sem_init(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[ti], 0, 0);
                usleep(100);
                ti++;
            }
            AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 1;
        }

        for(ti = 0; ti < COMPUTE_DARK_SUBTRACT_NBTHREADS; ti++)
        {
            sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], &sval0);
            if(sval0 < SEMAPHORE_MAXVAL)
            {
                sem_post(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti]);
            }

            sem_getvalue(&AOLCOMPUTE_DARK_SUBTRACT_sem_name[ti], &sval);

#ifdef _PRINT_TEST
            printf("TEST - DARK SUBTRACT - WAITING ON THREAD %ld\n", ti);
            fflush(stdout);
#endif
            sem_wait(&AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[ti]);
        }

        data.image[ID_imWFS0].md[0].cnt1 =
            data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
        COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS0, -1);

#ifdef _PRINT_TEST
        printf("TEST - DARK SUBTRACT - END\n");
        fflush(stdout);
#endif
    }

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    if(RM == 0)
    {
        aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_imWFS0] =
            1; // there must only be one such process
        AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_imWFS0, tnow);
    }

    DEBUG_TRACEPOINT(" ");

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    //  if(IDdark!=-1)
    // {
    //    for(ii=0; ii<AOconf[loop].WFSim.sizeWFS; ii++)
    //       data.image[ID_imWFS0].array.F[ii] -= data.image[IDdark].array.F[ii];
    //}
    //    AOconf[loop].AOtiminginfo.statusM = 1;
    if(RM == 0)
    {
        AOconf[loop].AOtiminginfo.status =
            2; // 4 -> 002 : COMPUTE TOTAL OF IMAGE
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[1] = tdiffv;
    }

#ifdef _PRINT_TEST
    printf("TEST - NORMALIZE = %d\n", normalize);
    fflush(stdout);
#endif

    DEBUG_TRACEPOINT(" ");

    printf("================== TEST POINT LINE %d\n", __LINE__);
    fflush(stdout);

    //
    // Normalize: imWFS0 -> imWFS1
    //
    if(normalize == 1)
    {
        if((AOconf[loop].AOcompute.AOLCOMPUTE_TOTAL_ASYNC == 0) ||
                (AOLCOMPUTE_TOTAL_INIT == 0) || (RM == 1)) // do it in main thread
        {
            float IMTOTAL;

            nelem = data.image[ID_imWFS0].md[0].size[0] *
                    data.image[ID_imWFS0].md[0].size[1];
            IMTOTAL = 0.0;
            if(aoloopcontrol_var.aoconfID_wfsmask != -1)
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    IMTOTAL += data.image[ID_imWFS0].array.F[ii] *
                               data.image[aoloopcontrol_var.aoconfID_wfsmask]
                               .array.F[ii];
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    IMTOTAL += data.image[ID_imWFS0].array.F[ii];
                }
            }

            //            AOconf[loop].WFStotalflux =
            //            arith_image_total(data.image[ID_imWFS0].name);
            AOconf[loop].WFSim.WFStotalflux = IMTOTAL;

            AOLCOMPUTE_TOTAL_INIT = 1;
            //            IMTOTAL = AOconf[loop].WFSim.WFStotalflux;
            if(aoloopcontrol_var.aoconfID_imWFS0tot != -1)
            {
                data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0] =
                    IMTOTAL;
                COREMOD_MEMORY_image_set_sempost_byID(
                    aoloopcontrol_var.aoconfID_imWFS0tot,
                    -1);
                //                sem_getvalue(data.image[aoloopcontrol_var.aoconfID_imWFS0tot].semptr[0],
                //                &semval);
                //               if(semval<SEMAPHORE_MAXVAL)
                //                  sem_post(data.image[aoloopcontrol_var.aoconfID_imWFS0tot].semptr[0]);
            }
        }
        else // do it in other threads
        {
#ifdef _PRINT_TEST
            printf(
                "TEST - compute total in separate thread  "
                "AOLCOMPUTE_TOTAL_ASYNC_THREADinit = %d\n",
                AOLCOMPUTE_TOTAL_ASYNC_THREADinit);
            fflush(stdout);
#endif

            // AOconf[loop].WFSim.WFStotalflux =
            // data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0]; //
            // from last loop
            if(AOLCOMPUTE_TOTAL_ASYNC_THREADinit == 0)
            {

                printf("Starting Image Total Thread \n");
                fflush(stdout);

                pthread_create(&thread_computetotal_id,
                               NULL,
                               compute_function_imtotal,
                               NULL);
                AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 1;
                imtotalcnt                        = 0;
                sem_init(&AOLCOMPUTE_TOTAL_ASYNC_sem_name, 0, 0);
            }
            sem_getvalue(&AOLCOMPUTE_TOTAL_ASYNC_sem_name, &semval);

#ifdef _PRINT_TEST
            printf("TEST - semaphore = %d / %d\n", semval, SEMAPHORE_MAXVAL);
            fflush(stdout);
#endif
            data.image[aoloopcontrol_var.aoconfID_imWFS0tot].md[0].cnt1 =
                imtotalcnt;
            if(semval < SEMAPHORE_MAXVAL)
            {
                sem_post(&AOLCOMPUTE_TOTAL_ASYNC_sem_name);
            }
        }
    }

    if(RM == 0)
    {
        //        AOconf[loop].AOtiminginfo.status = 3;  // 5 -> 003: NORMALIZE
        //        WFS IMAGE
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[14] = tdiffv;
    }

    data.image[ID_imWFS0].md[0].cnt0++;

    nelem = AOconf[loop].WFSim.sizeWFS;

    if(normalize == 1)
    {
        totalinv = 1.0 / (AOconf[loop].WFSim.WFStotalflux +
                          AOconf[loop].WFSim.WFSnormfloor *
                          AOconf[loop].WFSim.sizeWFS);
        aoloopcontrol_var.normfloorcoeff =
            AOconf[loop].WFSim.WFStotalflux /
            (AOconf[loop].WFSim.WFStotalflux +
             AOconf[loop].WFSim.WFSnormfloor * AOconf[loop].WFSim.sizeWFS);
    }
    else
    {
        totalinv                         = 1.0;
        aoloopcontrol_var.normfloorcoeff = 1.0;
    }

    aoloopcontrol_var.GPU_alpha = totalinv;

    aoloopcontrol_var.GPU_beta = -aoloopcontrol_var.normfloorcoeff;

    DEBUG_TRACEPOINT(" ");

    if(((AOconf[loop].AOcompute.GPUall == 0) && (RM == 0)) ||
            (RM == 1)) // normalize WFS image by totalinv
    {
#ifdef _PRINT_TEST
        printf("TEST - Normalize [%d]: IMTOTAL = %g    totalinv = %g\n",
               AOconf[loop].WFSim.WFSnormalize,
               data.image[aoloopcontrol_var.aoconfID_imWFS0tot].array.F[0],
               totalinv);
        fflush(stdout);
#endif

        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].write = 1;
        //# ifdef _OPENMP
        //        #pragma omp parallel num_threads(4) if
        //        (nelem>OMP_NELEMENT_LIMIT)
        //        {
        //# endif

        //# ifdef _OPENMP
        //            #pragma omp for
        //# endif
        for(uint64_t ii = 0; ii < nelem; ii++)
        {
            data.image[aoloopcontrol_var.aoconfID_imWFS1].array.F[ii] =
                data.image[ID_imWFS0].array.F[ii] * totalinv;
        }
        //# ifdef _OPENMP
        //        }
        //# endif
        COREMOD_MEMORY_image_set_sempost_byID(aoloopcontrol_var.aoconfID_imWFS1,
                                              -1);
        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].cnt0++;
        data.image[aoloopcontrol_var.aoconfID_imWFS1].md[0].write = 0;
    }

#ifdef _PRINT_TEST
    printf("TEST - READ CAM DONE\n");
    fflush(stdout);
#endif

    DEBUG_TRACEPOINT(" ");

    // AOconf[loop].AOtiminginfo.statusM = 2;
    if(RM == 0)
    {
        clock_gettime(CLOCK_MILK, &tnow);
        tdiffv = timespec_diff_double(
                     data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].atime,
                     tnow);
        data.image[aoloopcontrol_var.aoconfID_looptiming].array.F[2] = tdiffv;

        if(AOconf[loop].AOcompute.GPUall == 0)
        {
            aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_imWFS1] =
                1; // there must only be one such process
            AOloopControl_RTstreamLOG_update(loop, RTSLOGindex_imWFS1, tnow);
        }
    }

    clock_gettime(CLOCK_MILK, &functionTestTimerEnd);

    // processing time
    tdiffv = timespec_diff_double(functionTestTimerStart, functionTestTimerEnd);


    // Total time
    tdiffv = timespec_diff_double(functionTestTimer00, functionTestTimerEnd);

    // Cam wait time
    if(imWaitTimeAvecnt < imWaitTimeAvecnt0)
    {
        imWaitTimeAve += 1.0 * tdiffv / imWaitTimeAvecnt0;
        imWaitTimeAvecnt++;
    }
    else
    {
        float gain    = 1.0 / imWaitTimeAvecnt0;
        imWaitTimeAve = imWaitTimeAve * (1.0 - gain) + gain * tdiffv;
    }

    DEBUG_TRACEPOINT(" ");



    return (0);
}
*/