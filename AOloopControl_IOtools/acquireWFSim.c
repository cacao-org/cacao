/**
 * @file acquireWFSim.c
 * @brief Acquirewfsim module
 */


#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "acquireWFS",
    .cmdkey      = "acquireWFS",
    .description = "acquire WFS image"
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     *insname          = NULL;
static uint32_t *AOloopindex      = NULL;
static uint32_t *semindex         = NULL;
static float    *fluxtotal        = NULL;
static float    *GPUalpha         = NULL;
static float    *GPUbeta          = NULL;
static float    *WFSnormfloor     = NULL;
static float    *WFStaveragegain  = NULL;
static float    *WFStaveragemult  = NULL;
static float    *WFSrefcgain      = NULL;
static float    *WFSrefcmult      = NULL;
static int64_t  *compWFSsubdark   = NULL;
static int64_t  *compWFSnormalize = NULL;
static int64_t  *compWFSmask      = NULL;
static int64_t  *compWFSrefsub    = NULL;
static int64_t  *compWFSsigav     = NULL;
static int64_t  *compWFSrefc      = NULL;
static int64_t  *resetWFSrefc     = NULL;
static char     *wfszposname      = NULL;


/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".insname", &insname, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "input stream name") \
    X(".AOloopindex", &AOloopindex, \
      FPTYPE_UINT32, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "loop index") \
    X(".semindex", &semindex, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "input semaphore index") \
    X(".WFStaveragegain", &WFStaveragegain, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "tmlt*(1-tg)*imwfs3+tg*imwfs2") \
    X(".WFStaveragemult", &WFStaveragemult, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "tmlt*(1-tg)*imwfs3+tg*imwfs2") \
    X(".WFSrefcmult", &WFSrefcmult, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "mlt*(wfsref-zpo)+(1-m)*refc") \
    X(".WFSrefcgain", &WFSrefcgain, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "wfsrefc+gain*imwfs3->wfsrefc") \
    X(".out.fluxtotal", &fluxtotal, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_OUTPUT, \
      "total flux") \
    X(".out.GPUalpha", &GPUalpha, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_OUTPUT, \
      "GPU alpha coefficient") \
    X(".out.GPUbeta", &GPUbeta, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_OUTPUT, \
      "GPU beta coefficient") \
    X(".WFSnormfloor", &WFSnormfloor, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS flux floor for normalize") \
    X(".comp.darksub", &compWFSsubdark, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "-wfsdark, x wfsmult->imWFS0") \
    X(".comp.WFSnormalize", &compWFSnormalize, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "normalize over wfsmask->imWFS1") \
    X(".comp.compWFSmask", &compWFSmask, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "x wfsmask ?") \
    X(".comp.WFSrefsub", &compWFSrefsub, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "subtract WFS ref ->imWFS2") \
    X(".comp.WFSsigav", &compWFSsigav, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "average WFS signal") \
    X(".comp.WFSrefc", &compWFSrefc, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS reference correction") \
    X(".comp.resetWFSrefc", &resetWFSrefc, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "reset WFS reference correction") \
    X(".wfszpo", &wfszposname, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS zero point offset")


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

FPS_V2_SECTION5(FPS_PARAMS)


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    // connect to WFS image
    IMGID imgwfsim = stream_connect(insname);
    if(imgwfsim.ID == -1)
    {
        printf("ERROR: no WFS input\n");
        return RETURN_FAILURE;
    }
    uint32_t sizexWFS = imgwfsim.md->size[0];
    uint32_t sizeyWFS = imgwfsim.md->size[1];
    uint64_t sizeWFS  = sizexWFS * sizeyWFS;
    uint8_t  WFSatype = imgwfsim.md->datatype;


    // create/read images
    IMGID imgimWFS0;
    IMGID imgimWFS1;
    IMGID imgimWFS2;
    IMGID imgimWFS3;
    IMGID imgwfsref;
    IMGID imgwfsrefc;
    IMGID imgwfsmask;
    {
        char name[STRINGMAXLEN_IMGNAME];

        WRITE_IMAGENAME(name, "aol%u_imWFS0", *AOloopindex);
        imgimWFS0 = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_imWFS1", *AOloopindex);
        imgimWFS1 = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_imWFS2", *AOloopindex);
        imgimWFS2 = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_imWFS3", *AOloopindex);
        imgimWFS3 = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_wfsref", *AOloopindex);
        imgwfsref = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_wfsrefc", *AOloopindex);
        imgwfsrefc = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);

        WRITE_IMAGENAME(name, "aol%u_wfsmask", *AOloopindex);
        imgwfsmask = stream_connect_create_2Df32(name, sizexWFS, sizeyWFS);
    }


    if(imgwfsmask.md->creatorPID == getpid())
    {
        // if wfsmask created here, initialize it to 1
        printf("INITIALIZING wfsmask to 1\n");
        for(uint64_t ii; ii < imgwfsmask.md->nelement; ii++)
        {
            imgwfsmask.im->array.F[ii] = 1.0;
        }
    }


    list_image_ID();

    int wfsim_semwaitindex =
        ImageStreamIO_getsemwaitindex(imgwfsim.im, *semindex);
    if(wfsim_semwaitindex > -1)
    {
        *semindex = wfsim_semwaitindex;
    }

    // initialize camera averaging arrays if not already done
    void *__restrict array_tmp;
    array_tmp = malloc(sizeof(float) * sizeWFS);
    if(array_tmp == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    float *__restrict arrayftmp = (float *) array_tmp;
    uint16_t *__restrict arrayutmp = (uint16_t *) array_tmp;
    int16_t *__restrict arraystmp = (int16_t *) array_tmp;

    // LOAD DARK
    IMGID imgwfsdark;
    {
        char wfsdarkname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsdarkname, "aol%u_wfsdark", *AOloopindex);
        imgwfsdark = stream_connect(wfsdarkname);
    }


    // LOAD WFS MULT
    IMGID imgwfsmult;
    {
        char wfsmultname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsmultname, "aol%u_wfsmult", *AOloopindex);
        imgwfsmult = stream_connect(wfsmultname);
    }


    // WFS zero point offset
    //
    IMGID imgdispzpo;
    {
        imgdispzpo =
            stream_connect_create_2Df32(wfszposname, sizexWFS, sizeyWFS);
    }


    struct timespec time1, time2;
    long n_print_timings = 5000;

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        // ===========================================
        // COPY FRAME TO LOCAL MEMORY BUFFER
        // ===========================================

        int slice = 0;


        DEBUG_TRACEPOINT(" ");

        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }

        void *ptrv = NULL;
        switch(WFSatype)
        {
        case _DATATYPE_FLOAT:
        case _DATATYPE_UINT16:
        case _DATATYPE_INT16:
        {
            int ts = ImageStreamIO_typesize(imgwfsim.md->datatype);
            ptrv = imgwfsim.im->array.raw + ts * slice * sizeWFS;
            memcpy(array_tmp, ptrv, ts * sizeWFS);
        }
        break;

        default:
            PRINT_ERROR("DATA TYPE NOT SUPPORTED");
            abort();
            break;
        }

        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Pre-copy time: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }


        // ===================================================
        // SUBTRACT WFSDARK AND MULTIPLY BY WFSMULT-> imWFS0
        // ===================================================
        DEBUG_TRACEPOINT(" ");

        // check wfsdark is to be subtracted
        int status_darksub = 0;


        // check if wfsmult to be applied
        //int status_wfsmult = 0;

        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.darksub") == 1 &&
                imgwfsdark.ID != -1)
        {
            status_darksub = 1;
        }

        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }

        SHMIM_WRITE_ACQUIRE(imgimWFS0.md);

        switch(WFSatype)
        {
        case _DATATYPE_UINT16:
            if(status_darksub == 0)
            {
                // no dark subtraction, convert data to float
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] = ((float) arrayutmp[ii]);
                }
            }
            else
            {
                // dark subtraction
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] =
                        ((float) arrayutmp[ii]) -
                        imgwfsdark.im->array.F[ii];
                }
            }
            break;

        case _DATATYPE_INT16:
            if(status_darksub == 0)
            {
                // no dark subtraction, convert data to float
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] = ((float) arraystmp[ii]);
                }
            }
            else
            {
                // dark subtraction
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] =
                        ((float) arraystmp[ii]) -
                        imgwfsdark.im->array.F[ii];
                }
            }
            break;

        case _DATATYPE_FLOAT:
            if(status_darksub == 0)
            {
                // no dark subtraction, copy data to imWFS0
                memcpy(imgimWFS0.im->array.F,
                       arrayftmp,
                       sizeof(float) * sizeWFS);
            }
            else
            {
                // dark subtraction
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] =
                        arrayftmp[ii] - imgwfsdark.im->array.F[ii];
                }
            }
            break;

        default:
            printf("ERROR: WFS data type not recognized\n File %s, line %d\n",
                   __FILE__,
                   __LINE__);
            printf("datatype = %d\n", WFSatype);
            exit(0);
            break;
        }

        if(status_darksub == 1)
        {
            if(imgwfsmult.ID != -1)
            {
                for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS0.im->array.F[ii] *= imgwfsmult.im->array.F[ii];
                }
            }
        }

        processinfo_update_output_stream(processinfo, imgimWFS0.im, NULL);
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Dark sub to imWFS0: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }


        DEBUG_TRACEPOINT(" ");


        // ===========================================
        // NORMALIZE imWFS0 -> imWFS1
        // ===========================================
        int status_normalize = 0;

        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }
        SHMIM_WRITE_ACQUIRE(imgimWFS1.md);

        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.WFSnormalize") == 1)
        {
            status_normalize = 1;

            // Compute image total over wfsmask
            //
            double imtotal = 0.0;
            uint64_t nelem = imgimWFS0.md->size[0] *
                             imgimWFS0.md->size[1];

            if(imgwfsmask.ID != -1)
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imtotal += imgimWFS0.im->array.F[ii] *
                               imgwfsmask.im->array.F[ii];
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imtotal += imgimWFS0.im->array.F[ii];
                }
            }
            *fluxtotal = imtotal;


            // avoiding division by zero
            //
            double fluxtotpos = *fluxtotal;
            if(fluxtotpos < 0.0)
            {
                fluxtotpos = 0.0;
            }
            double totalinv       = 1.0 / (*fluxtotal + *WFSnormfloor * sizeWFS);


            if((imgwfsmask.ID != -1)
                    && (functionparameter_GetParamValue_ONOFF(
                        data.core.fpsptr, ".comp.compWFSmask") == 1))
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS1.im->array.F[ii] =
                        imgimWFS0.im->array.F[ii] * totalinv * imgwfsmask.im->array.F[ii];
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS1.im->array.F[ii] =
                        imgimWFS0.im->array.F[ii] * totalinv;
                }
            }

        }
        else
        {
            uint64_t nelem = imgimWFS0.md->size[0] *
                             imgimWFS0.md->size[1];
            if(imgwfsmask.ID != -1)
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imgimWFS1.im->array.F[ii] = imgimWFS0.im->array.F[ii] *
                                                imgwfsmask.im->array.F[ii];
                }
            }
            else
            {

                memcpy(imgimWFS1.im->array.F,
                       imgimWFS0.im->array.F,
                       sizeof(float) * sizeWFS);
            }
        }
        processinfo_update_output_stream(processinfo, imgimWFS1.im, NULL);
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Renorm to imWFS1: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }


        // ===========================================
        // REFERENCE SUBTRACT imWFS2 -> imWFS2
        // ===========================================

        int status_refsub = 0;
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }
        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.WFSrefsub") == 1)
        {
            // subtract reference
            status_refsub = 1;
            SHMIM_WRITE_ACQUIRE(imgimWFS2.md);

            if(imgwfsrefc.ID != -1)
            {

                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS2.im->array.F[ii] =
                        imgimWFS1.im->array.F[ii] -
                        imgwfsrefc.im->array.F[ii];
                }
            }

            processinfo_update_output_stream(processinfo, imgimWFS2.im, NULL);
        }
        else
        {
            SHMIM_WRITE_ACQUIRE(imgimWFS2.md);
            memcpy(imgimWFS2.im->array.F,
                   imgimWFS1.im->array.F,
                   sizeof(float) * sizeWFS);

            processinfo_update_output_stream(processinfo, imgimWFS2.im, NULL);
        }
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Refsub to imWFS2: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }


        // ===========================================
        // AVERAGE -> imWFS3
        // ===========================================

        int status_ave = 0;
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }
        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.WFSsigav") == 1)
        {
            status_ave = 1;
            SHMIM_WRITE_ACQUIRE(imgimWFS3.md);
            float tave_gain = *WFStaveragegain;
            float tave_mult = *WFStaveragemult;
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                float valf =
                    tave_mult *
                    ((1.0 - tave_gain) * imgimWFS3.im->array.F[ii] +
                     tave_gain * imgimWFS2.im->array.F[ii]);

                // clean any NaN or inf, as they would loop back to wfsrefc
                if(isnormal(valf))
                {
                    imgimWFS3.im->array.F[ii] = valf;
                }
                else
                {
                    imgimWFS3.im->array.F[ii] = 0.0;
                }
            }
            processinfo_update_output_stream(processinfo, imgimWFS3.im, NULL);
        }
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Av to imWFS3: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }

        // ===========================================
        // UPDATE wfsrefc
        // ===========================================

        int status_wfsrefc = 0;
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time1);
        }


        // Reset imWFS3, wfsrefc and wfszpo to zero
        //
        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.resetWFSrefc") == 1)
        {
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                imgwfsrefc.im->array.F[ii] = imgwfsref.im->array.F[ii];
                imgdispzpo.im->array.F[ii] = 0.0;
                imgimWFS3.im->array.F[ii] = 0.0;
            }

            // toggle back to OFF
            functionparameter_SetParamValue_ONOFF(
                data.core.fpsptr, ".comp.resetWFSrefc", 0);
        }

        if(functionparameter_GetParamValue_ONOFF(
                data.core.fpsptr, ".comp.WFSrefc") == 1)
        {
            status_wfsrefc = 1;
            SHMIM_WRITE_ACQUIRE(imgwfsrefc.md);
            float refcgain = *WFSrefcgain;
            float refcmult = *WFSrefcmult;
            if(imgwfsref.ID != -1)
            {
                // refcmult is pulling refc toward ref-wfszpo
                // if refcmult = 1, then refc=ref
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgwfsrefc.im->array.F[ii] =
                        imgwfsmask.im->array.F[ii] *
                        refcmult * (imgwfsref.im->array.F[ii] +
                                    imgdispzpo.im->array.F[ii]) +
                        (1.0 - refcmult) * imgwfsrefc.im->array.F[ii];
                }
            }

            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                // refcgain is zeroing residual
                //
                imgwfsrefc.im->array.F[ii] =
                    imgwfsrefc.im->array.F[ii] +
                    refcgain * imgimWFS3.im->array.F[ii];
            }

            // normalize
            if(functionparameter_GetParamValue_ONOFF(
                    data.core.fpsptr, ".comp.WFSnormalize") == 1)
            {
                // Compute image total
                double imtotal = 0.0;
                uint64_t nelem = imgwfsrefc.md->size[0] *
                                 imgwfsrefc.md->size[1];

                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imtotal +=  imgwfsrefc.im->array.F[ii];
                }
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    float valf = imgwfsrefc.im->array.F[ii];
                    valf /= imtotal;

                    if(isnormal(valf))
                    {
                        imgwfsrefc.im->array.F[ii] = valf;
                    }
                    else
                    {
                        imgwfsrefc.im->array.F[ii] = 0.0;
                    }
                }
            }

            // clean any NaN or inf, as they would loop back to wfsrefc
            for(uint64_t ii = 0; ii < imgwfsrefc.md->size[0] *
                    imgwfsrefc.md->size[1]; ii++)
            {
                float valf = imgwfsrefc.im->array.F[ii];
                if(isnormal(valf))
                {
                    imgwfsrefc.im->array.F[ii] = valf;
                }
                else
                {
                    imgwfsrefc.im->array.F[ii] = 0.0;
                }
            }


            processinfo_update_output_stream(processinfo, imgwfsrefc.im, NULL);
        }
        if(processinfo->loopcnt % n_print_timings == 0)
        {
            clock_gettime(CLOCK_MILK, &time2);
            printf("refc to imgwfsrefc: %f us\n", timespec_diff_double(time1, time2) * 1e6);
            fflush(stdout);
        }

        processinfo_WriteMessage_fmt(
            processinfo, "d%d n%d s%d a%d c%d",
            status_darksub,
            status_normalize,
            status_refsub,
            status_ave,
            status_wfsrefc
        );
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

/* ================================================================
 * 7.  MILK MODULE REGISTRATION
 * ============================================================= */

#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

errno_t
CLIADDCMD_AOloopControl_IOtools__acquireWFSim()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif


/* ================================================================
 * 8.  STANDALONE ENTRY POINT
 * ============================================================= */

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
