/**
 * @file    acquireWFS.c
 * @brief   acquire and preprocess WFS image
 *
 */

#include "CommandLineInterface/CLIcore.h"

// Local variables pointers

static char *insname;
long fpi_insname;

static uint32_t *AOloopindex;
static long      fpi_AOloopindex;

static uint32_t *semindex;
static long      fpi_semindex;

static float *fluxtotal;
static long   fpi_fluxtotal;

static float *GPUalpha;
static long   fpi_GPUalpha;

static float *GPUbeta;
static long   fpi_GPUbeta;

static float *WFSnormfloor;
static long   fpi_WFSnormfloor;

static float *WFStaveragegain;
static long   fpi_WFStaveragegain;

static float *WFStaveragemult;
static long   fpi_WFStaveragemult;

static float *WFSrefcgain;
static long   fpi_WFSrefcgain;

static float *WFSrefcmult;
static long   fpi_WFSrefcmult;

static int64_t *compWFSsubdark;
static long     fpi_compWFSsubdark;

static int64_t *compWFSnormalize;
static long     fpi_compWFSnormalize;

static int64_t *compWFSrefsub;
static long     fpi_compWFSrefsub;

static int64_t *compWFSsigav;
static long     fpi_compWFSsigav;

// compute corrected WFS reference
static int64_t *compWFSrefc;
static long     fpi_compWFSrefc;

static char *wfszposname;
static long  fpi_wfszposname;



static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STREAM,
        ".insname",
        "input stream name",
        "inV",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &insname,
        &fpi_insname
    },
    {
        CLIARG_UINT32,
        ".AOloopindex",
        "loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        &fpi_AOloopindex
    },
    {
        CLIARG_UINT32,
        ".semindex",
        "input semaphore index",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &semindex,
        &fpi_semindex
    },
    {
        CLIARG_FLOAT32,
        ".WFStaveragegain",
        "tmult*(1-tgain)*imwfs3 + tgain*imwfs2 -> imwfs3",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFStaveragegain,
        &fpi_WFStaveragegain
    },
    {
        CLIARG_FLOAT32,
        ".WFStaveragemult",
        "tmult*(1-tgain)*imwfs3 + tgain*imwfs2 -> imwfs3",
        "0.999",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFStaveragemult,
        &fpi_WFStaveragemult
    },
    {
        CLIARG_FLOAT32,
        ".WFSrefcmult",
        "mult*(wfsref-wfszpo)+(1-mult)*wfsrefc -> wfsrefc",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFSrefcmult,
        &fpi_WFSrefcmult
    },
    {
        CLIARG_FLOAT32,
        ".WFSrefcgain",
        "wfsrefc + gain*imwfs3 -> wfsrefc",
        "0.00",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFSrefcgain,
        &fpi_WFSrefcgain
    },
    {
        CLIARG_FLOAT32,
        ".out.fluxtotal",
        "total flux",
        "0.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &fluxtotal,
        &fpi_fluxtotal
    },
    {
        CLIARG_FLOAT32,
        ".out.GPUalpha",
        "GPU alpha coefficient",
        "0.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &GPUalpha,
        &fpi_GPUalpha
    },
    {
        CLIARG_FLOAT32,
        ".out.GPUbeta",
        "GPU beta coefficient",
        "0.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &GPUbeta,
        &fpi_GPUbeta
    },
    {
        CLIARG_FLOAT32,
        ".WFSnormfloor",
        "WFS flux floor for normalize",
        "0.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &WFSnormfloor,
        &fpi_WFSnormfloor
    },
    {
        CLIARG_ONOFF,
        ".comp.darksub",
        "sub aolX_wfsdark mult aolX_wfsgain -> imWFS0",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSsubdark,
        &fpi_compWFSsubdark
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSnormalize",
        "normalize WFS frames -> imWFS1",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSnormalize,
        &fpi_compWFSnormalize
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSrefsub",
        "subtract WFS reference aolX_wfsrefc -> imWFS2",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSrefsub,
        &fpi_compWFSrefsub
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSsigav",
        "average WFS signal",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSsigav,
        &fpi_compWFSsigav
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSrefc",
        "WFS reference correction",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSrefc,
        &fpi_compWFSrefc
    },
    {
        CLIARG_STREAM,
        ".wfszpo",
        "Wavefront sensor zero point offset",
        "aolX_wfszpo",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfszposname,
        &fpi_wfszposname
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_insname].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;


        data.fpsptr->parray[fpi_WFStaveragegain].fpflag  |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFStaveragemult].fpflag  |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSnormfloor].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSsubdark].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSnormalize].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSrefsub].fpflag    |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSsigav].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSrefc].fpflag      |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSrefcgain].fpflag      |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSrefcmult].fpflag      |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "acquireWFS", "acquire WFS image", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
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
        char name[STRINGMAXLEN_STREAMNAME];

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
    float           *__restrict arrayftmp;
    unsigned short *__restrict arrayutmp;
    signed short    *__restrict arraystmp;
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

        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);

        void* ptrv = NULL;
        switch(WFSatype)
        {
        case _DATATYPE_FLOAT:
            ptrv = (char *) imgwfsim.im->array.F;
            ptrv += sizeof(float) * slice * sizeWFS;
            memcpy(arrayftmp, ptrv, sizeof(float) * sizeWFS);
            break;

        case _DATATYPE_UINT16:
            ptrv = (char *) imgwfsim.im->array.UI16;
            ptrv += sizeof(unsigned short) * slice * sizeWFS;
            memcpy(arrayutmp, ptrv, sizeof(unsigned short) * sizeWFS);
            break;

        case _DATATYPE_INT16:
            ptrv = (char *) imgwfsim.im->array.SI16;
            ptrv += sizeof(signed short) * slice * sizeWFS;
            memcpy(arraystmp, ptrv, sizeof(signed short) * sizeWFS);
            break;

        default:
            PRINT_ERROR("DATA TYPE NOT SUPPORTED");
            abort();
            break;
        }

        if (processinfo->loopcnt % n_print_timings == 0) {
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

        if(data.fpsptr->parray[fpi_compWFSsubdark].fpflag & FPFLAG_ONOFF &&
            imgwfsdark.ID != -1)
        {
            status_darksub = 1;
        }

        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);
        
        imgimWFS0.md->write = 1;

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

        processinfo_update_output_stream(processinfo, imgimWFS0.ID);
        if (processinfo->loopcnt % n_print_timings == 0) {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Dark sub to imWFS0: %f us\n", timespec_diff_double(time1, time2)*1e6);
        }


        DEBUG_TRACEPOINT(" ");






        // ===========================================
        // NORMALIZE -> imWFS1
        // ===========================================
        int status_normalize = 0;

        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);
        imgimWFS1.md->write = 1;

        if(data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF)
        {
            status_normalize = 1;

            // Compute image total
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

            if(imgwfsmask.ID != -1)
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
        processinfo_update_output_stream(processinfo, imgimWFS1.ID);
        if (processinfo->loopcnt % n_print_timings == 0) {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Renorm to imWFS1: %f us\n", timespec_diff_double(time1, time2)*1e6);
        }



        // ===========================================
        // REFERENCE SUBTRACT -> imWFS2
        // ===========================================

        int status_refsub = 0;
        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);
        if(data.fpsptr->parray[fpi_compWFSrefsub].fpflag & FPFLAG_ONOFF)
        {
            // subtract reference
            status_refsub = 1;
            imgimWFS2.md->write = 1;

            if(imgwfsrefc.ID != -1)
            {

                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS2.im->array.F[ii] =
                        imgimWFS1.im->array.F[ii] -
                        imgwfsrefc.im->array.F[ii];
                }
            }

            processinfo_update_output_stream(processinfo, imgimWFS2.ID);
        }
        else
        {
            imgimWFS2.md->write = 1;
            memcpy(imgimWFS2.im->array.F,
                   imgimWFS1.im->array.F,
                   sizeof(float) * sizeWFS);

            processinfo_update_output_stream(processinfo, imgimWFS2.ID);
        }
        if (processinfo->loopcnt % n_print_timings == 0) {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Refsub to imWFS2: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }


        // ===========================================
        // AVERAGE -> imWFS3
        // ===========================================

        int status_ave = 0;
        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);
        if(data.fpsptr->parray[fpi_compWFSsigav].fpflag & FPFLAG_ONOFF)
        {
            status_ave = 1;
            imgimWFS3.md->write = 1;
            float tave_gain = *WFStaveragegain;
            float tave_mult = *WFStaveragemult;
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                imgimWFS3.im->array.F[ii] =
                    tave_mult *
                    ((1.0 - tave_gain) * imgimWFS3.im->array.F[ii] +
                     tave_gain * imgimWFS2.im->array.F[ii]);
            }
            processinfo_update_output_stream(processinfo, imgimWFS3.ID);
        }
        if (processinfo->loopcnt % n_print_timings == 0) {
            clock_gettime(CLOCK_MILK, &time2);
            printf("Av to imWFS3: %f us\n", timespec_diff_double(time1, time2) * 1e6);
        }

        // ===========================================
        // UPDATE wfsrefc
        // ===========================================

        int status_wfsrefc = 0;
        if (processinfo->loopcnt % n_print_timings == 0) clock_gettime(CLOCK_MILK, &time1);
        if(data.fpsptr->parray[fpi_compWFSrefc].fpflag & FPFLAG_ONOFF)
        {
            status_wfsrefc = 1;
            imgwfsrefc.md->write = 1;
            float refcgain = *WFSrefcgain;
            float refcmult = *WFSrefcmult;
            if(imgwfsref.ID != -1)
            {
                // refcmult is pulling refc toward ref-wfszpo
                // if refcmult = 1, then refc=ref
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgwfsrefc.im->array.F[ii] =
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

            processinfo_update_output_stream(processinfo, imgwfsrefc.ID);
        }
        if (processinfo->loopcnt % n_print_timings == 0) {
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



INSERT_STD_FPSCLIfunctions




// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__acquireWFSim()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
