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
        "Subtract Dark aolX_wfsdark -> imWFS0",
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
        "subtract WFS reference aol0_wfsref -> imWFS2",
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
        "aol9_wfszpo",
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


        data.fpsptr->parray[fpi_WFStaveragegain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFStaveragemult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSnormfloor].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSsubdark].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSnormalize].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSrefsub].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSsigav].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSrefc].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSrefcgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_WFSrefcmult].fpflag |= FPFLAG_WRITERUN;
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
    //char WFSname[100];
    //sprintf(WFSname, "aol%u_wfsim", *AOloopindex);
    long ID_wfsim = read_sharedmem_image(insname);
    if(ID_wfsim == -1)
    {
        printf("ERROR: no WFS input\n");
        return RETURN_FAILURE;
    }
    uint32_t sizexWFS = data.image[ID_wfsim].md[0].size[0];
    uint32_t sizeyWFS = data.image[ID_wfsim].md[0].size[1];
    uint64_t sizeWFS  = sizexWFS * sizeyWFS;
    uint8_t  WFSatype = data.image[ID_wfsim].md[0].datatype;

    // create/read images
    imageID ID_imWFS0 = -1;
    imageID ID_imWFS1 = -1;
    imageID ID_imWFS2 = -1;
    imageID ID_imWFS3 = -1;
    imageID IDwfsrefc = -1;
    {
        char     name[STRINGMAXLEN_STREAMNAME];
        uint32_t naxes[2];
        naxes[0] = sizexWFS;
        naxes[1] = sizeyWFS;

        WRITE_IMAGENAME(name, "aol%u_imWFS0", *AOloopindex);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS0);

        WRITE_IMAGENAME(name, "aol%u_imWFS1", *AOloopindex);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS1);

        WRITE_IMAGENAME(name, "aol%u_imWFS2", *AOloopindex);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS2);

        WRITE_IMAGENAME(name, "aol%u_imWFS3", *AOloopindex);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS3);


        IMGID imgwfsref;
        {
            char wfsrefname[STRINGMAXLEN_STREAMNAME];
            WRITE_IMAGENAME(wfsrefname, "aol%u_wfsref", *AOloopindex);
            imgwfsref = stream_connect_create_2Df32(wfsrefname, sizexWFS, sizeyWFS);
        }

        WRITE_IMAGENAME(name, "aol%u_wfsrefc", *AOloopindex);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &IDwfsrefc);

        //        AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", sizexWFS,
        //        sizeyWFS, 0.0);
        // ID_imWFS1 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", sizexWFS,
        // sizeyWFS, 0.0);
    }



    imageID IDwfsmask = -1;
    IMGID imgwfsmask;
    {
        char wfsmaskname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsmaskname, "aol%u_wfsmask", *AOloopindex);
        imgwfsmask = stream_connect_create_2Df32(wfsmaskname, sizexWFS, sizeyWFS);
        IDwfsmask = imgwfsmask.ID;
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
        ImageStreamIO_getsemwaitindex(&data.image[ID_wfsim], *semindex);
    if(wfsim_semwaitindex > -1)
    {
        *semindex = wfsim_semwaitindex;
    }

    // initialize camera averaging arrays if not already done
    float          *arrayftmp;
    unsigned short *arrayutmp;
    signed short   *arraystmp;
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
    imageID IDwfsdark = -1;
    {
        char wfsdarkname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsdarkname, "aol%u_wfsdark", *AOloopindex);
        IDwfsdark = image_ID(wfsdarkname);
        if(IDwfsdark == -1)
        {
            IDwfsdark = read_sharedmem_image(wfsdarkname);
        }
        printf("IDwfsdark = %ld\n", IDwfsdark);
    }

    // LOAD REFERENCE
    imageID IDwfsref = -1;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%u_wfsref", *AOloopindex);
        IDwfsref = read_sharedmem_image(name);
        printf("reading image %s -> ID = %ld\n", name, IDwfsref);
    }



    // WFS zero point offset
    //
    IMGID imgdispzpo;
    {
        imgdispzpo =
            stream_connect_create_2Df32(wfszposname, sizexWFS, sizeyWFS);
    }




    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        // ===========================================
        // COPY FRAME TO LOCAL MEMORY BUFFER
        // ===========================================

        int slice = 0;
        /*
        if(data.image[ID_wfsim].md[0].naxis == 3) // ring buffer
        {
          slice = data.image[ID_wfsim].md[0].cnt1;
          if(slice == -1)
          {
              slice = data.image[ID_wfsim].md[0].size[2];
          }
        }*/

        DEBUG_TRACEPOINT(" ");



        char *ptrv;
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



        // ===========================================
        // SUBTRACT DARK -> imWFS0
        // ===========================================
        DEBUG_TRACEPOINT(" ");

        // check dark is to be subtracted
        int status_darksub = 0;
        if(data.fpsptr->parray[fpi_compWFSsubdark].fpflag & FPFLAG_ONOFF)
        {
            if(IDwfsdark != -1)
            {
                status_darksub = 1;
            }
        }

        data.image[ID_imWFS0].md[0].write = 1;



        switch(WFSatype)
        {
        case _DATATYPE_UINT16:
            if(status_darksub == 0)
            {
                // no dark subtraction, convert data to float
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]);
                }
            }
            else
            {
                // dark subtraction
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        ((float) arrayutmp[ii]) -
                        data.image[IDwfsdark].array.F[ii];
                }
            }
            break;

        case _DATATYPE_INT16:

            if(status_darksub == 0)
            {
                // no dark subtraction, convert data to float
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arraystmp[ii]);
                }
            }
            else
            {
                // dark subtraction
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        ((float) arraystmp[ii]) -
                        data.image[IDwfsdark].array.F[ii];
                }
            }
            break;

        case _DATATYPE_FLOAT:
            if(status_darksub == 0)
            {
                // no dark subtraction, copy data to imWFS0
                memcpy(data.image[ID_imWFS0].array.F,
                       arrayftmp,
                       sizeof(float) * sizeWFS);
            }
            else
            {
                // dark subtraction
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] =
                        arrayftmp[ii] - data.image[IDwfsdark].array.F[ii];
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



        processinfo_update_output_stream(processinfo, ID_imWFS0);



        DEBUG_TRACEPOINT(" ");






        // ===========================================
        // NORMALIZE -> imWFS1
        // ===========================================
        int status_normalize = 0;

        data.image[ID_imWFS1].md->write = 1;

        if(data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF)
        {
            status_normalize = 1;

            // Compute image total
            double imtotal = 0.0;
            uint64_t nelem = data.image[ID_imWFS0].md[0].size[0] *
                             data.image[ID_imWFS0].md[0].size[1];

            if(IDwfsmask != -1)
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imtotal += data.image[ID_imWFS0].array.F[ii] *
                               data.image[IDwfsmask].array.F[ii];
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < nelem; ii++)
                {
                    imtotal += data.image[ID_imWFS0].array.F[ii];
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
            //double normfloorcoeff = *fluxtotal / (*fluxtotal + *WFSnormfloor * sizeWFS);

            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                data.image[ID_imWFS1].array.F[ii] =
                    data.image[ID_imWFS0].array.F[ii] * totalinv;
            }

        }
        else
        {
            memcpy(data.image[ID_imWFS1].array.F,
                   data.image[ID_imWFS0].array.F,
                   sizeof(float) * sizeWFS);
        }
        processinfo_update_output_stream(processinfo, ID_imWFS1);

//                *GPUalpha = totalinv;
//                *GPUbeta = -normfloorcoeff;



        // ===========================================
        // REFERENCE SUBTRACT -> imWFS2
        // ===========================================

        int status_refsub = 0;
        if(data.fpsptr->parray[fpi_compWFSrefsub].fpflag & FPFLAG_ONOFF)
        {
            // subtract reference
            status_refsub = 1;
            data.image[ID_imWFS2].md[0].write = 1;

            if(IDwfsrefc != -1)
            {

                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS2].array.F[ii] =
                        data.image[ID_imWFS1].array.F[ii] -
                        data.image[IDwfsrefc].array.F[ii];
                }
            }

            processinfo_update_output_stream(processinfo, ID_imWFS2);

        }
        else
        {
            data.image[ID_imWFS2].md->write = 1;
            memcpy(data.image[ID_imWFS2].array.F,
                   data.image[ID_imWFS1].array.F,
                   sizeof(float) * sizeWFS);

            processinfo_update_output_stream(processinfo, ID_imWFS2);
        }


        // ===========================================
        // AVERAGE -> imWFS3
        // ===========================================

        int status_ave = 0;
        if(data.fpsptr->parray[fpi_compWFSsigav].fpflag & FPFLAG_ONOFF)
        {
            status_ave = 1;
            data.image[ID_imWFS3].md[0].write = 1;
            float tave_gain                   = *WFStaveragegain;
            float tave_mult                   = *WFStaveragemult;
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                data.image[ID_imWFS3].array.F[ii] =
                    tave_mult *
                    ((1.0 - tave_gain) * data.image[ID_imWFS3].array.F[ii] +
                     tave_gain * data.image[ID_imWFS2].array.F[ii]);
            }
            processinfo_update_output_stream(processinfo, ID_imWFS3);
        }




        // ===========================================
        // UPDATE wfsrefc
        // ===========================================

        printf("IDwfsref = %ld\n", IDwfsref);
        fflush(stdout);

        printf("IDwfsrefc = %ld\n", IDwfsrefc);
        fflush(stdout);

        printf("ID_imWFS3 = %ld\n", ID_imWFS3);
        fflush(stdout);


        int status_wfsrefc = 0;
        if(data.fpsptr->parray[fpi_compWFSrefc].fpflag & FPFLAG_ONOFF)
        {
            status_wfsrefc = 1;
            data.image[IDwfsrefc].md[0].write = 1;
            float refcgain                    = *WFSrefcgain;
            float refcmult                    = *WFSrefcmult;
            if(IDwfsref != -1)
            {
                // refcmult is pulling refc toward ref-wfszpo
                // if refcmult = 1, then refc=ref
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[IDwfsrefc].array.F[ii] =
                        refcmult * (data.image[IDwfsref].array.F[ii] +
                                    imgdispzpo.im->array.F[ii]) +
                        (1.0 - refcmult) * data.image[IDwfsrefc].array.F[ii];
                }
            }
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                // refcgain is zeroing residual
                //
                data.image[IDwfsrefc].array.F[ii] =
                    data.image[IDwfsrefc].array.F[ii] +
                    refcgain * data.image[ID_imWFS3].array.F[ii];
            }

            processinfo_update_output_stream(processinfo, IDwfsrefc);
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
