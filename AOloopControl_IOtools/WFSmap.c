// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    WFSmap.c
 * @brief   remap WFS image
 *
 */
#include "ImageStreamIO/ImageStruct.h"

#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "mapWFS",
    .cmdkey      = "mapWFS",
    .description = "remap WFS image",
    .description_long =
        "Remap wavefront sensor pixels using a geometric transformation map. Corrects optical distortion or aligns subapertures."
};

static char wfsinsname[FUNCTION_PARAMETER_STRMAXLEN];
static char mapsname[FUNCTION_PARAMETER_STRMAXLEN];
static char wfsoutsname[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X) \
    X(".wfsin", wfsinsname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "Wavefront sensor input") \
    X(".map", mapsname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "WFS mapping") \
    X(".wfsout", wfsoutsname, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "Wavefront sensor output")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(milk_data.fpsptr != NULL)
    {

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
    return RETURN_SUCCESS;
}


errno_t image_pixremap(
    IMGID inimg,
    IMGID mapimg,
    IMGID outimg,
    int reuse
)
{
    DEBUG_TRACE_FSTART();

    static int initialize = 1;
    static uint64_t mapNBpix = 0;
    static uint64_t *map_inpixindex = NULL;
    static uint64_t *map_outpixindex = NULL;
    static float *map_pixcoeff = NULL;

    if(initialize == 1)
    {
        float eps = 1.0e-6;

        //uint32_t *mapNBpix = (uint32_t *) malloc(sizeof(mapNBpix) * mapimg.md->size[2]);
        printf("%u output pixels\n", mapimg.md->size[2]);


        // scan map to count pixels
        mapNBpix = 0;
        uint64_t xysize = (uint64_t) mapimg.md->size[0];
        xysize *= mapimg.md->size[1];
        for(uint64_t ii = 0; ii < (uint64_t)mapimg.md->size[0]*mapimg.md->size[1]*mapimg.md->size[2]; ii++)
        {
            if(fabsf(mapimg.im->array.F[ii]) > eps)
            {
                mapNBpix++;
            }
        }
        printf("%lu active pixels in map\n", mapNBpix);

        // allocate mapping arrays
        map_inpixindex = (uint64_t *) malloc(sizeof(uint64_t) * mapNBpix);
        map_outpixindex = (uint64_t *) malloc(sizeof(uint64_t) * mapNBpix);
        map_pixcoeff = (float *) malloc(sizeof(float) * mapNBpix);

        // fill mapping arrays
        uint64_t mappix = 0;


        for(uint32_t kk = 0; kk < mapimg.md->size[2]; kk++)
        {
            for(uint64_t ii = 0; ii < (uint64_t)mapimg.md->size[0]*mapimg.md->size[1]; ii++)
            {
                uint64_t pixindex = (uint64_t)kk * mapimg.md->size[0] * mapimg.md->size[1] + ii;
                if(fabsf(mapimg.im->array.F[pixindex]) > eps)
                {
                    map_inpixindex[mappix] = ii;
                    map_outpixindex[mappix] = kk;
                    map_pixcoeff[mappix] = mapimg.im->array.F[pixindex];

                    mappix++;
                }
            }
        }


        printf("mapping arrays initialized\n");
        fflush(stdout);
        initialize = 0;
    }

    DEBUG_TRACEPOINT("Initializing output array, size %u", mapimg.md->size[2]);

    double *tmpvarray = (double *) malloc(sizeof(double) * mapimg.md->size[2]);
    for(uint32_t kk = 0; kk < mapimg.md->size[2]; kk++)
    {
        tmpvarray[kk] = 0.0;
    }

    DEBUG_TRACEPOINT("Applying mapping, %lu pixels", mapNBpix);


    switch(inimg.md->datatype)
    {

    case _DATATYPE_FLOAT :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.F[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_DOUBLE :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.D[map_inpixindex[mapii]];
        }
        break;


    case _DATATYPE_INT8 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.SI8[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_INT16 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.SI16[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_UINT16 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.UI16[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_INT32 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.SI32[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_UINT32 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.UI32[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_INT64 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.SI64[map_inpixindex[mapii]];
        }
        break;

    case _DATATYPE_UINT64 :
        for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
        {
            tmpvarray[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                                                 inimg.im->array.UI64[map_inpixindex[mapii]];
        }
        break;


    }

    for(uint32_t kk = 0; kk < mapimg.md->size[2]; kk++)
    {
        outimg.im->array.F[kk] = (float) tmpvarray[kk];
    }

    free(tmpvarray);

    if(reuse == 0)
    {
        free(map_inpixindex);
        free(map_outpixindex);
        free(map_pixcoeff);
        initialize = 1;
    }

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID wfsinimg = imgid_make_from_name(wfsinsname);
    resolveIMGID(
        &wfsinimg, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (wfsinimg.ID == -1) return RETURN_FAILURE;

    IMGID mapimg = imgid_make_from_name(mapsname);
    resolveIMGID(
        &mapimg, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (mapimg.ID == -1) return RETURN_FAILURE;


    uint32_t sizeout = mapimg.md->size[2];

    // Create output
    //
    IMGID wfsoutimg;
    wfsoutimg =
        stream_connect_create_2D(wfsoutsname, sizeout, 1, _DATATYPE_FLOAT);


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        image_pixremap(wfsinimg, mapimg, wfsoutimg, 1);
        processinfo_update_output_stream(processinfo, wfsoutimg.im, NULL);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&wfsinimg);
    imgid_free(&mapimg);
    imgid_free(&wfsoutimg);

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
errno_t
CLIADDCMD_AOloopControl_IOtools__WFSmap()
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
