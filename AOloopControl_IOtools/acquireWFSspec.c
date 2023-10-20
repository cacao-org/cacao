/**
 * @file    acquireWFSspec.c
 * @brief   acquire spectra - a stripped-down version of acquireWFSim for dispersed WFS
 *
 */

#include <math.h>
#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_tools/COREMOD_tools.h"

// Local variables pointers
static char *input_shm_name; // input shared memory
static long  fpi_inputshmname;

static char *specmask_shm_name; // mask shared memory
static long  fpi_specmaskshmname;

static int64_t *binning;
static long     fpi_binning;

static uint32_t *AOloopindex;
static long      fpi_AOloopindex;

static uint32_t *semindex;
static long      fpi_semindex;

static int64_t *compWFSsubdark;
static long     fpi_compWFSsubdark;

static int64_t *compWFSnormalize;
static long     fpi_compWFSnormalize;

static int64_t *compWFSrefsub;
static long     fpi_compWFSrefsub;

static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".wfsin",
        "Wavefront sensor input",
        "wfsin",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &input_shm_name,
        &fpi_inputshmname
    },
    {
        CLIARG_IMG,
        ".wfsmask",
        "wfs spectral extraction mask",
        "wfsspecmask",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &specmask_shm_name,
        &fpi_specmaskshmname
    },
    {
        CLIARG_UINT32,
        ".binning",
        "spectral trace binning",
        "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &binning,
        &fpi_binning
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
        CLIARG_ONOFF,
        ".comp.darksub",
        "sub aolX_wfsdark -> imWFS0",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSsubdark,
        &fpi_compWFSsubdark
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSnormalize",
        "normalize WFS frames -> imWFS1",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSnormalize,
        &fpi_compWFSnormalize
    },
    {
        CLIARG_ONOFF,
        ".comp.WFSrefsub",
        "subtract WFS reference aolX_wfsref -> imWFS2",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compWFSrefsub,
        &fpi_compWFSrefsub
    },
};

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_inputshmname].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_compWFSsubdark].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSnormalize].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compWFSrefsub].fpflag    |= FPFLAG_WRITERUN;
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
    "acquire_spectra", "acquire spectra", CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS; // no help for you
}

static errno_t extract_traces(
    IMGID wfsin,
    IMGID specmask,
    IMGID wfsout,
    uint32_t binning
)
{
    uint32_t sizeWFSx = wfsin.size[0];
    uint32_t sizeWFSy = wfsin.size[1];
    uint64_t sizeWFSraw  = sizeWFSx * sizeWFSy;

    uint32_t sizeWFSoutx = sizeWFSx/binning;

    uint8_t  WFSatype = wfsin.md->datatype;

    uint32_t numtraces = specmask.size[2];
    uint64_t sizeWFSout  = sizeWFSoutx * numtraces;

    for (uint32_t k = 0; k < numtraces; k++){
        uint32_t xpix_index = 0;
        for (uint32_t i = 0; i < sizeWFSx; i=i+binning){
            float tot = 0.0;
            for (uint32_t _i = 0; _i < binning; _i++){
                for (uint32_t j = 0; j < sizeWFSy; j++) {
                    uint64_t mpixindex = k * sizeWFSraw +  j * sizeWFSx + i + _i;
                    uint64_t pixindex = j * sizeWFSx + i + _i;
                    
                    // handle different input types, ultimately cast to float
                    switch(WFSatype)
                    {
                        case _DATATYPE_UINT16: 
                            tot += wfsin.im->array.UI16[pixindex] * specmask.im->array.UI16[mpixindex];
                            break;
                        case _DATATYPE_INT16:
                            tot += wfsin.im->array.SI16[pixindex] * specmask.im->array.UI16[mpixindex];
                            break;
                        case _DATATYPE_FLOAT:
                            tot += wfsin.im->array.F[pixindex] * specmask.im->array.UI16[mpixindex];
                            break;
                        case _DATATYPE_UINT32:
                            tot += wfsin.im->array.UI32[pixindex] * specmask.im->array.UI16[mpixindex];
                            break;
                        default:
                            printf("ERROR: WFS data type not recognized\n File %s, line %d\n",
                                __FILE__,
                                __LINE__);
                            printf("datatype = %d\n", WFSatype);
                            exit(0);
                            break;
                    }
                }
            }
            wfsout.im->array.F[k * sizeWFSoutx + xpix_index] = tot;
            xpix_index += 1;
        }
    }  
    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

static errno_t dark_sub(
    IMGID wfsin,
    IMGID wfsdark,
    IMGID wfsout
)
{
    uint8_t  darkWFSatype = wfsdark.md->datatype;
    uint64_t sizeWFS = wfsin.size[0]*wfsin.size[1]
    
    // dark subtraction
    for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
    {
        switch(darkWFSatype)
            {
                case _DATATYPE_UINT16: 
                    wfsout.im->array.F[ii] = wfsin.im->array.F[ii] - wfsdark.im->array.UI16[ii];
                    break;
                case _DATATYPE_INT16:
                    wfsout.im->array.F[ii] = wfsin.im->array.F[ii] - wfsdark.im->array.SI16[ii];
                    break;
                case _DATATYPE_FLOAT:
                    wfsout.im->array.F[ii] = wfsin.im->array.F[ii] - wfsdark.im->array.F[ii];
                    break;
                case _DATATYPE_UINT32:
                    wfsout.im->array.F[ii] = wfsin.im->array.F[ii] - wfsdark.im->array.UI32[ii];
                    break;
                default:
                    printf("ERROR: WFS DARK data type not recognized\n File %s, line %d\n",
                        __FILE__,
                        __LINE__);
                    printf("datatype = %d\n", darkWFSatype);
                    exit(0);
                    break;
            }
    }
}

static errno_t spec_norm(
    IMGID wfsin,
    IMGID wfsout
)
{
    uint32_t j;
    uint32_t sizeWFSx = wfsin.size[0];
    uint32_t num_traces = wfsin.size[1];

    for (uint_fast32_t i = 0; i < sizeWFSx; i++) {
        float tot = 0.0;
        for (j = 0; j < numtraces; j++){
            tot += wfsin.im->array.F[j*sizeWFSx + i];
        }
        float normval = 0.;
        if (tot > 0){
            normval = 1./tot;
        }
        for (j = 0; j < numtraces; j++) {
            wfsout.im->array.F[j*sizeWFSx + i] = wfsin.im->array.F[j*sizeWFSx + i]*normval;
        }
    }
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID wfsin = mkIMGID_from_name(input_shm_name); // input raw wfs image
    resolveIMGID(&wfsin, ERRMODE_ABORT);

    uint32_t sizeWFSx = wfsin.size[0];
    uint32_t sizeWFSy = wfsin.size[1];
    uint64_t sizeWFSraw  = sizeWFSx * sizeWFSy;
    uint8_t  WFSatype = wfsin.md->datatype;

    IMGID specmask = mkIMGID_from_name(specmask_shm_name);
    resolveIMGID(&specmask, ERRMODE_ABORT);
    uint32_t numtraces = specmask.size[2];
    uint64_t sizeWFS  = sizeWFSx * numtraces;


    // size is  (image shape) * z, z is # of traces
    // each z-slice looks like a bar that covers one trace

    // create/read images
    IMGID imgimWFSm; // mapped (extracted spectra) 
    IMGID imgimWFS0; // dark subtracted
    IMGID imgimWFS1; // normalized
    IMGID imgimWFS2; // ref subtracted - this is ultimately the output
    IMGID imgwfsref; // the ref

    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name, "aol%u_imWFSm", *AOloopindex);
        imgimWFSm = stream_connect_create_2Df32(name, sizeWFSx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS0", *AOloopindex);
        imgimWFS0 = stream_connect_create_2Df32(name, sizeWFSx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS1", *AOloopindex);
        imgimWFS1 = stream_connect_create_2Df32(name, sizeWFSx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS2", *AOloopindex);
        imgimWFS2 = stream_connect_create_2Df32(name, sizeWFSx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_wfsref", *AOloopindex);
        imgwfsref = stream_connect_create_2Df32(name, sizeWFSx, numtraces);
    }

    list_image_ID();

    int wfsim_semwaitindex =
        ImageStreamIO_getsemwaitindex(wfsin.im, *semindex);
    if(wfsim_semwaitindex > -1)
    {
        *semindex = wfsim_semwaitindex;
    }

    // LOAD DARK
    IMGID imgwfsdark;
    {
        char wfsdarkname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsdarkname, "aol%u_wfsdark", *AOloopindex);
        imgwfsdark = stream_connect(wfsdarkname);
    }

    // This is the while(True) {
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // STEP 1: extract spectra -> aolx_imWFSm
        extract_traces(wfsin,specmask,imgimWFSm);
        
        // Done and post downstream.
        processinfo_update_output_stream(processinfo, imgimWFSm.ID);

        // STEP 2: DARK SUB -> aolx_imWFS0
        // check wfsdark is to be subtracted
        int status_darksub = 0;
        if(data.fpsptr->parray[fpi_compWFSsubdark].fpflag & FPFLAG_ONOFF)
        {
            if(imgwfsdark.ID != -1)
            {
                status_darksub = 1;
            }
        }
        
        imgimWFS0.md->write = 1;

        if(status_darksub == 0)
        {
            // no dark subtraction, pass through
            for(uint_fast64_t ii = 0; ii < sizeWFS; ii++)
            {
                memcpy(imgimWFS0.im->array.F , imgimWFSm.im->array.F , sizeof(float) * sizeWFS);
            }
        }
        else
        {
            dark_sub(imgimWFSm,imgWFSdark,imgimWFS0); // dark sub to imWFS0
        }
        processinfo_update_output_stream(processinfo, imgimWFS0.ID); // post

        // STEP 3: NORMALIZATION
        int status_normalize = 0;
        imgimWFS1.md->write = 1;

        if(data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF)
        {
            status_normalize = 1;
            spec_norm(imgimWFS0,imgimWFS1);
        }
        else
        {
            memcpy(imgimWFS1.im->array.F,
                   imgimWFS0.im->array.F,
                   sizeof(float) * sizeWFS);
        }
        processinfo_update_output_stream(processinfo, imgimWFS1.ID);
        
        // STEP 4: REFERENCE SUBTRACTION

        int status_refsub = 0;
        imgimWFS2.md->write = 1;
        if(data.fpsptr->parray[fpi_compWFSrefsub].fpflag & FPFLAG_ONOFF)
        {
            // subtract reference
            status_refsub = 1;

            if(imgwfsref.ID != -1)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imgimWFS2.im->array.F[ii] =
                        imgimWFS1.im->array.F[ii] - imgwfsref.im->array.F[ii];
                }
            }
        }
        else
        {
            memcpy(imgimWFS2.im->array.F,
                   imgimWFS1.im->array.F,
                   sizeof(float) * sizeWFS);
        }
        processinfo_update_output_stream(processinfo, imgimWFS2.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__acquirespectra()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
