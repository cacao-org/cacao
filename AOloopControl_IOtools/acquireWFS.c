/**
 * @file    acquireWFS.c
 * @brief   acquire and preprocess WFS image
 *
 */



#include "CommandLineInterface/CLIcore.h"




// Local variables pointers
static uint32_t *AOloop;
long fpi_AOloop;

static uint32_t *semindex;
long fpi_semindex;


static float *fluxtotal;
long fpi_fluxtotal;

static float *GPUalpha;
long fpi_GPUalpha;

static float *GPUbeta;
long fpi_GPUbeta;

static int64_t *WFSnormalize;
long fpi_WFSnormalize;

static float *WFSnormfloor;
long fpi_WFSnormfloor;

static int64_t *compdark;
long fpi_compdark;

static int64_t *compimtotal;
long fpi_compimtotal;

static int64_t *compnormwfsim;
long fpi_compnormwfsim;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32, ".AOloop", "loop index", "0",
        CLIARG_VISIBLE_DEFAULT, (void **) &AOloop, &fpi_AOloop
    },
    {
        CLIARG_UINT32, ".semindex", "input semaphore index", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &semindex, &fpi_semindex
    },
    {
        CLIARG_FLOAT32, ".out.fluxtotal", "total flux", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &fluxtotal, &fpi_fluxtotal
    },
    {
        CLIARG_FLOAT32, ".out.GPUalpha", "GPU alpha coefficient", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &GPUalpha, &fpi_GPUalpha
    },
    {
        CLIARG_FLOAT32, ".out.GPUbeta", "GPU beta coefficient", "0.0",
        CLIARG_OUTPUT_DEFAULT, (void **) &GPUbeta, &fpi_GPUbeta
    },
    {
        CLIARG_ONOFF, ".WFSnormalize", "normalize WFS frames", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &WFSnormalize, &fpi_WFSnormalize
    },
    {
        CLIARG_FLOAT32, ".WFSnormfloor", "WFS flux floor for normalize", "0.0",
        CLIARG_HIDDEN_DEFAULT, (void **) &WFSnormfloor, &fpi_WFSnormfloor
    },
    {
        CLIARG_ONOFF, ".comp.darksub", "Subtract Dark", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &compdark, &fpi_compdark
    },
    {
        CLIARG_ONOFF, ".comp.imtotal", "Compute WFS frame total flux", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &compimtotal, &fpi_compimtotal
    },
    {
        CLIARG_ONOFF, ".comp.normwfsim", "Compute normalized WFS frame", "1",
        CLIARG_HIDDEN_DEFAULT, (void **) &compnormwfsim, &fpi_compnormwfsim
    }
};






// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
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
    "acquireWFS",
    "acquire WFS image",
    CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    printf(">>>>>>>>>>>> [%u] %d\n", *AOloop, __LINE__);

    // connect to WFS image
    char WFSname[100];
    sprintf(WFSname, "aol%u_wfsim", *AOloop);
    long ID_wfsim = read_sharedmem_image(WFSname);
    if(ID_wfsim == -1)
    {
        printf("ERROR: no WFS input\n");
        return RETURN_FAILURE;
    }
    uint32_t sizexWFS = data.image[ID_wfsim].md[0].size[0];
    uint32_t sizeyWFS = data.image[ID_wfsim].md[0].size[1];
    uint64_t sizeWFS = sizexWFS * sizeyWFS;
    uint8_t  WFSatype = data.image[ID_wfsim].md[0].datatype;

    printf(">>>>>>>>>>>> %d\n", __LINE__);

    // create/read images
    imageID ID_imWFS0 = -1;
    imageID ID_imWFS1 = -1;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        uint32_t naxes[2];
        naxes[0] = sizexWFS;
        naxes[1] = sizeyWFS;

        WRITE_IMAGENAME(name, "aol%u_imWFS0", *AOloop);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS0);

        WRITE_IMAGENAME(name, "aol%u_imWFS1", *AOloop);
        create_image_ID(name, 2, naxes, _DATATYPE_FLOAT, 1, 0, 0, &ID_imWFS1);


        //        AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", sizexWFS, sizeyWFS, 0.0);
        // ID_imWFS1 = AOloopControl_IOtools_2Dloadcreate_shmim(name, " ", sizexWFS, sizeyWFS, 0.0);
    }

    printf(">>>>>>>>>>>> %d\n", __LINE__);

    long IDwfsmask;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%u_wfsmask", *AOloop);
        IDwfsmask = read_sharedmem_image(name);
    }

    printf(">>>>>>>>>>>> %d\n", __LINE__);

    list_image_ID();



    int wfsim_semwaitindex = ImageStreamIO_getsemwaitindex(&data.image[ID_wfsim],
                             *semindex);
    if(wfsim_semwaitindex > -1)
    {
        *semindex = wfsim_semwaitindex;
    }


    // initialize camera averaging arrays if not already done
    float *arrayftmp;
    unsigned short *arrayutmp;
    signed short *arraystmp;
    if(WFSatype == _DATATYPE_FLOAT)
    {
        arrayftmp = (float *) malloc(sizeof(float) * sizeWFS);
        if(arrayftmp == NULL) {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }

    if(WFSatype == _DATATYPE_UINT16)
    {
        arrayutmp = (unsigned short *) malloc(sizeof(unsigned short) * sizeWFS);
        if(arrayutmp == NULL) {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }

    if(WFSatype == _DATATYPE_INT16)
    {
        arraystmp = (signed short *)   malloc(sizeof(signed short) *   sizeWFS);
        if(arraystmp == NULL) {
            PRINT_ERROR("malloc returns NULL pointer");
            abort();
        }
    }


    // LOAD DARK
    long IDwfsdark = -1;
    {
        char wfsdarkname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsdarkname, "aol%u_wfsdark", *AOloop);
        IDwfsdark = image_ID(wfsdarkname);
        //Average_cam_frames_nelem = sizeWFS;
    }


    // set semaphore to 0
    int          semval;
    sem_getvalue(data.image[ID_wfsim].semptr[*semindex], &semval);
    printf("INITIALIZING SEMAPHORE %u   %s   (%d)\n", *semindex,
           data.image[ID_wfsim].md[0].name, semval);
    for(int i = 0; i < semval; i++)
    {
        sem_trywait(data.image[ID_wfsim].semptr[*semindex]);
    }



    INSERT_STD_PROCINFO_COMPUTEFUNC_START

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
    case _DATATYPE_FLOAT :
        ptrv = (char *) data.image[ID_wfsim].array.F;
        ptrv += sizeof(float) * slice * sizeWFS;
        memcpy(arrayftmp, ptrv,  sizeof(float)*sizeWFS);
        break;

    case _DATATYPE_UINT16 :
        ptrv = (char *) data.image[ID_wfsim].array.UI16;
        ptrv += sizeof(unsigned short) * slice * sizeWFS;
        memcpy(arrayutmp, ptrv, sizeof(unsigned short)*sizeWFS);
        break;

    case _DATATYPE_INT16 :
        ptrv = (char *) data.image[ID_wfsim].array.SI16;
        ptrv += sizeof(signed short) * slice * sizeWFS;
        memcpy(arraystmp, ptrv, sizeof(signed short)*sizeWFS);
        break;

    default :
        printf("ERROR: DATA TYPE NOT SUPPORTED\n");
        exit(0);
        break;
    }

    //WFScnt = data.image[ID_wfsim].md[0].cnt0;


    // ===========================================
    // SUBTRACT DARK -> imWFS0
    // ===========================================
    DEBUG_TRACEPOINT(" ");
    if( data.fpsptr->parray[fpi_compdark].fpflag & FPFLAG_ONOFF)
    {
        data.image[ID_imWFS0].md[0].write = 1;
        DEBUG_TRACEPOINT(" ");
        switch(WFSatype)
        {

        case _DATATYPE_UINT16 :
            if(IDwfsdark == -1)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]);
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arrayutmp[ii]) -
                                                        data.image[IDwfsdark].array.F[ii];
                }
            }
            break;



        case _DATATYPE_INT16 :

            if(IDwfsdark == -1)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arraystmp[ii]);
                }
            }
            else
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = ((float) arraystmp[ii]) -
                                                        data.image[IDwfsdark].array.F[ii];
                }

            }
            break;


        case _DATATYPE_FLOAT :
            if(IDwfsdark == -1)
            {
                memcpy(data.image[ID_imWFS0].array.F, arrayftmp,
                       sizeof(float)*sizeWFS);
            }
            else
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    data.image[ID_imWFS0].array.F[ii] = arrayftmp[ii] -
                                                        data.image[IDwfsdark].array.F[ii];
                }
            }
            break;

        default :
            printf("ERROR: WFS data type not recognized\n File %s, line %d\n", __FILE__,
                   __LINE__);
            printf("datatype = %d\n", WFSatype);
            exit(0);
            break;
        }
        processinfo_update_output_stream(processinfo, ID_imWFS0);
        //data.image[ID_imWFS0].md[0].cnt1 = data.image[aoloopcontrol_var.aoconfID_looptiming].md[0].cnt1;
//                COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS0, -1);
//                data.image[ID_imWFS0].md[0].cnt0 ++;
//                data.image[ID_imWFS0].md[0].write = 0;
    }


    DEBUG_TRACEPOINT(" ");

    // ===========================================
    // COMPUTE TOTAL -> WFSfluxtotal
    // ===========================================
    double IMTOTAL;
    if(data.fpsptr->parray[fpi_compimtotal].fpflag & FPFLAG_ONOFF)
    {
        uint64_t nelem = data.image[ID_imWFS0].md[0].size[0] *
                         data.image[ID_imWFS0].md[0].size[1];
        IMTOTAL = 0.0;
        if(IDwfsmask != -1)
        {
            for(uint64_t ii = 0; ii < nelem; ii++)
            {
                IMTOTAL += data.image[ID_imWFS0].array.F[ii] *
                           data.image[IDwfsmask].array.F[ii];
            }
        }
        else
        {
            for(uint64_t ii = 0; ii < nelem; ii++)
            {
                IMTOTAL += data.image[ID_imWFS0].array.F[ii];
            }
        }
        *fluxtotal = IMTOTAL;
    }

    DEBUG_TRACEPOINT(" ");

    // ===========================================
    // NORMALIZE -> imWFS1
    // ===========================================


    double totalinv;
    double normfloorcoeff;

    if(data.fpsptr->parray[fpi_WFSnormalize].fpflag & FPFLAG_ONOFF)
    {
        DEBUG_TRACEPOINT(" ");
        totalinv = 1.0 / (*fluxtotal + *WFSnormfloor * sizeWFS);
        normfloorcoeff = *fluxtotal / (*fluxtotal + *WFSnormfloor * sizeWFS);
    }
    else
    {
        DEBUG_TRACEPOINT(" ");
        totalinv = 1.0;
        normfloorcoeff = 1.0;
    }

    DEBUG_TRACEPOINT(" ");
    *GPUalpha = totalinv;

    *GPUbeta = -normfloorcoeff;


    if(data.fpsptr->parray[fpi_compnormwfsim].fpflag  &
            FPFLAG_ONOFF)    // normalize WFS image by totalinv
    {

        data.image[ID_imWFS1].md[0].write = 1;

        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            data.image[ID_imWFS1].array.F[ii] = data.image[ID_imWFS0].array.F[ii] *
                                                totalinv;
        }

        processinfo_update_output_stream(processinfo, ID_imWFS1);

//                COREMOD_MEMORY_image_set_sempost_byID(ID_imWFS1, -1);
//                data.image[ID_imWFS1].md[0].cnt0 ++;
//                data.image[ID_imWFS1].md[0].write = 0;
    }





    printf(">>>>>>>>>>>>>>>>>>>> acquire WFS iteration done\n");

    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__acquireWFS()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}





