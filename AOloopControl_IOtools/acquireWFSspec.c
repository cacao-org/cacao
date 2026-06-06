/**
 * @file    acquireWFSspec.c
 * @brief   acquire spectra - a stripped-down version of acquireWFSim for dispersed WFS
 *
 */
#include "ImageStreamIO/ImageStruct.h"

#include <math.h>
#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "acquire_spectra",
    .cmdkey      = "acquire_spectra",
    .description = "acquire spectra",
    .description_long =
        "Acquire spectral data from a wavefront sensor for chromatic characterization and dispersed fringe tracking."
};

// Local variables
static char input_shm_name[FUNCTION_PARAMETER_STRMAXLEN];
static char specmask_shm_name[FUNCTION_PARAMETER_STRMAXLEN];
static uint32_t binning;
static uint32_t AOloopindex;
static uint32_t semindex;
static uint64_t compWFSsubdark;
static uint64_t compWFSnormalize;
static uint64_t compWFSrefsub;

#define FPS_PARAMS(X) \
    X(".wfsin", input_shm_name, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "Wavefront sensor input") \
    X(".wfsmask", specmask_shm_name, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "wfs spectral extraction mask") \
    X(".binning", &binning, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "spectral trace binning") \
    X(".AOloopindex", &AOloopindex, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "loop index") \
    X(".semindex", &semindex, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "input semaphore index") \
    X(".comp.darksub", &compWFSsubdark, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "sub aolX_wfsdark -> imWFS0") \
    X(".comp.WFSnormalize", &compWFSnormalize, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "normalize WFS frames -> imWFS1") \
    X(".comp.WFSrefsub", &compWFSrefsub, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "subtract WFS reference aolX_wfsref -> imWFS2")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(milk_data.fpsptr != NULL)
    {
        long fpi_inputshmname = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".wfsin");
        long fpi_compWFSsubdark = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.darksub");
        long fpi_compWFSnormalize = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.WFSnormalize");
        long fpi_compWFSrefsub = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.WFSrefsub");

        if(fpi_inputshmname > -1) milk_data.fpsptr->parray[fpi_inputshmname].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        if(fpi_compWFSsubdark > -1)
            milk_data.fpsptr
                ->parray[fpi_compWFSsubdark]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_compWFSnormalize > -1)
            milk_data.fpsptr
                ->parray[fpi_compWFSnormalize]
                .fpflag |= FPFLAG_WRITERUN;
        if(fpi_compWFSrefsub > -1)
            milk_data.fpsptr
                ->parray[fpi_compWFSrefsub]
                .fpflag |= FPFLAG_WRITERUN;
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

// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS; // no help for you
}

/**
 * extract_traces() - extract spectral traces from a
 *                    WFS image
 * @wfsin:   input WFS image
 * @specmask: 3D spectral extraction mask
 *            (xsize x ysize x numtraces)
 * @wfsout:  output extracted spectra
 *            (xsize/binning x numtraces)
 * @binning: spectral binning factor
 *
 * Multiplies each WFS pixel by the corresponding
 * mask slice and sums along the spatial axis with
 * @binning-pixel bins.  Supports UINT16, INT16,
 * FLOAT, and UINT32 input types.
 */
static errno_t extract_traces(
    IMGID wfsin,
    IMGID specmask,
    IMGID wfsout,
    uint32_t binning
)
{
    DEBUG_TRACE_FSTART();

    uint32_t sizeWFSx = wfsin.md->size[0];
    uint32_t sizeWFSy = wfsin.md->size[1];
    uint64_t sizeWFSraw  = sizeWFSx * sizeWFSy;

    uint32_t sizeWFSoutx = sizeWFSx / binning;

    uint8_t  WFSatype = wfsin.md->datatype;

    uint32_t numtraces = specmask.md->size[2];
    uint64_t sizeWFSout __attribute__((unused)) = sizeWFSoutx * numtraces;

    for(uint32_t k = 0; k < numtraces; k++)
    {
        uint32_t xpix_index = 0;
        for(uint32_t i = 0; i < sizeWFSx; i = i + binning)
        {
            float tot = 0.0;
            for(uint32_t _i = 0; _i < binning; _i++)
            {
                for(uint32_t j = 0; j < sizeWFSy; j++)
                {
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

/**
 * dark_sub() - subtract dark frame from WFS image
 * @wfsin:   input WFS image (float)
 * @wfsdark: dark frame (any supported type)
 * @wfsout:  output dark-subtracted image (float)
 *
 * Subtracts the dark frame pixel-by-pixel, handling
 * UINT16, INT16, FLOAT, and UINT32 dark types.
 */
static errno_t dark_sub(
    IMGID wfsin,
    IMGID wfsdark,
    IMGID wfsout
)
{
    uint8_t  darkWFSatype = wfsdark.md->datatype;
    uint64_t sizeWFS = wfsin.md->size[0] * wfsin.md->size[1];

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
    return RETURN_SUCCESS;
}

/**
 * spec_norm() - normalize spectral traces
 * @wfsin:  input spectra (xsize x numtraces)
 * @wfsout: output normalized spectra
 *
 * For each spectral bin (column), divides each
 * trace value by the sum of all traces at that
 * bin, producing fractional flux values.
 */
static errno_t spec_norm(
    IMGID wfsin,
    IMGID wfsout
)
{
    uint32_t j;
    uint32_t sizeWFSx = wfsin.md->size[0];
    uint32_t numtraces = wfsin.md->size[1];

    for(uint_fast32_t i = 0; i < sizeWFSx; i++)
    {
        float tot = 0.0;
        for(j = 0; j < numtraces; j++)
        {
            tot += wfsin.im->array.F[j * sizeWFSx + i];
        }
        float normval = 0.;
        if(tot > 0)
        {
            normval = 1. / tot;
        }
        for(j = 0; j < numtraces; j++)
        {
            wfsout.im->array.F[j * sizeWFSx + i] = wfsin.im->array.F[j * sizeWFSx + i] *
                                                   normval;
        }
    }
    return RETURN_SUCCESS;
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID wfsin = imgid_make_from_name(input_shm_name); // input raw wfs image
    resolveIMGID(
        &wfsin, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (wfsin.ID == -1) return RETURN_FAILURE;

    uint32_t sizeWFSx = wfsin.md->size[0];
    uint32_t sizeWFSy = wfsin.md->size[1];
    uint64_t sizeWFSraw __attribute__((unused)) = sizeWFSx * sizeWFSy;
    uint8_t  WFSatype __attribute__((unused)) = wfsin.md->datatype;

    IMGID specmask = imgid_make_from_name(specmask_shm_name);
    resolveIMGID(
        &specmask, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (specmask.ID == -1) return RETURN_FAILURE;
    uint32_t numtraces = specmask.md->size[2];
    uint64_t sizeWFS  = sizeWFSx * numtraces;
    uint32_t sizeWFSoutx = sizeWFSx / binning;


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

        WRITE_IMAGENAME(name, "aol%u_imWFSm", AOloopindex);
        imgimWFSm = stream_connect_create_2Df32(name, sizeWFSoutx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS0", AOloopindex);
        imgimWFS0 = stream_connect_create_2Df32(name, sizeWFSoutx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS1", AOloopindex);
        imgimWFS1 = stream_connect_create_2Df32(name, sizeWFSoutx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_imWFS2", AOloopindex);
        imgimWFS2 = stream_connect_create_2Df32(name, sizeWFSoutx, numtraces);

        WRITE_IMAGENAME(name, "aol%u_wfsref", AOloopindex);
        imgwfsref = stream_connect_create_2Df32(name, sizeWFSoutx, numtraces);
    }

    list_image_ID();

    int wfsim_semwaitindex =
        ImageStreamIO_getsemwaitindex(wfsin.im, semindex);
    if(wfsim_semwaitindex > -1)
    {
        semindex = wfsim_semwaitindex;
    }

    // LOAD DARK
    IMGID imgWFSdark;
    {
        char wfsdarkname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(wfsdarkname, "aol%u_wfsdark", AOloopindex);
        imgWFSdark = stream_connect(wfsdarkname);
    }

    // This is the while(True) {
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        // STEP 1: extract spectra -> aolx_imWFSm
        extract_traces(wfsin, specmask, imgimWFSm, binning);

        // Done and post downstream.
        processinfo_update_output_stream(processinfo, imgimWFSm.im, NULL);

        // STEP 2: DARK SUB -> aolx_imWFS0
        // check wfsdark is to be subtracted
        int status_darksub = 0;
        long fpi_compWFSsubdark = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.darksub");
        if(fpi_compWFSsubdark > -1 && (milk_data.fpsptr->parray[fpi_compWFSsubdark].fpflag & FPFLAG_ONOFF))
        {
            if(imgWFSdark.ID != -1)
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
                memcpy(imgimWFS0.im->array.F,
                    imgimWFSm.im->array.F,
                    sizeof(float) * sizeWFS);
            }
        }
        else
        {
            dark_sub(imgimWFSm, imgWFSdark, imgimWFS0); // dark sub to imWFS0
        }
        processinfo_update_output_stream(processinfo, imgimWFS0.im, NULL); // post

        // STEP 3: NORMALIZATION
        int status_normalize __attribute__((unused)) = 0;
        imgimWFS1.md->write = 1;

        long fpi_compWFSnormalize = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.WFSnormalize");
        if(fpi_compWFSnormalize > -1 && (milk_data.fpsptr->parray[fpi_compWFSnormalize].fpflag & FPFLAG_ONOFF))
        {
            status_normalize = 1;
            spec_norm(imgimWFS0, imgimWFS1);
        }
        else
        {
            memcpy(imgimWFS1.im->array.F,
                   imgimWFS0.im->array.F,
                   sizeof(float) * sizeWFS);
        }
        processinfo_update_output_stream(processinfo, imgimWFS1.im, NULL);

        // STEP 4: REFERENCE SUBTRACTION

        int status_refsub __attribute__((unused)) = 0;
        imgimWFS2.md->write = 1;
        long fpi_compWFSrefsub = 
            functionparameter_GetParamIndex(
                milk_data.fpsptr, ".comp.WFSrefsub");
        if(fpi_compWFSrefsub > -1 && (milk_data.fpsptr->parray[fpi_compWFSrefsub].fpflag & FPFLAG_ONOFF))
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
        processinfo_update_output_stream(processinfo, imgimWFS2.im, NULL);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&wfsin);
    imgid_free(&specmask);
    imgid_free(&imgimWFSm);
    imgid_free(&imgimWFS0);
    imgid_free(&imgimWFS1);
    imgid_free(&imgimWFS2);
    imgid_free(&imgwfsref);
    imgid_free(&imgWFSdark);

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__acquirespectra()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(
    FPS_app_info,
    FPS_PARAMS,
    compute_function,
    customCONFsetup,
    customCONFcheck)
#endif
