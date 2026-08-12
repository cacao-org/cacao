// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file spotpos.c
 * @brief Measure spot position, photocenter
 *
 *
 */
#include "ImageStreamIO/ImageStruct.h"
#include <float.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

// quicksort
#include "COREMOD_tools/COREMOD_tools.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name         = "spotpos",
    .cmdkey           = "spotpos",
    .description      = "measure spot position, photocenter",
    .description_long = "Measure the precise centroid position of spots in a WFS image using "
                        "photocenter calculation with configurable weighting."
};

static char inimname[FUNCTION_PARAMETER_STRMAXLEN];
static char indarkname[FUNCTION_PARAMETER_STRMAXLEN];

// approximate spot size
//
static float spotsize;

// approximate spot location
// search will be centered around this coords
static float spotx0;
static float spoty0;

// Search radius around spotx0, spoty0
static float searchrad;

// position data
// xrel, yrel, xabs, yabs, flux, pixcnt
static char outspotpos[FUNCTION_PARAMETER_STRMAXLEN];


// 2D transformation matrix between pixel pos and TT value
static float mappingXX;
static float mappingYY;
static float mappingXY;
static float mappingYX;

// 2D position vector matching control TT
static char outTTvec[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X)                                                                         \
    X(".insname", inimname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),  \
      "input image")                                                                          \
    X(".indark_name", indarkname, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT,                 \
      "input image dark (optional)")                                                          \
    X(".spotsize", &spotsize, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),   \
      "approximate spot size [pix]")                                                          \
    X(".spotx0", &spotx0, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),       \
      "approx spot location x")                                                               \
    X(".spoty0", &spoty0, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),       \
      "approx spot location y")                                                               \
    X(".searchrad", &searchrad, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "search radius")                                                                        \
    X(".outspotpos", outspotpos, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT,                  \
      "output spot position data")                                                            \
    X(".mappingXX", &mappingXX, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "mapping XX coeff")                                                                     \
    X(".mappingYY", &mappingYY, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "mapping YY coeff")                                                                     \
    X(".mappingXY", &mappingXY, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "mapping XY coeff")                                                                     \
    X(".mappingYX", &mappingYX, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "mapping YX coeff")                                                                     \
    X(".outTTvec", outTTvec, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT,                      \
      "output 2D TT vector (control TT)")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup
// Runs once at conf startup
//
// To use this function, set :
// CLIcmddata.FPS_customCONFsetup = customCONFsetup
// when registering function
// (see end of this file)
//
static errno_t customCONFsetup()
{
    return RETURN_SUCCESS;
}

// Optional custom configuration checks
// Runs at every configuration check loop iteration
//
// To use this function, set :
// CLIcmddata.FPS_customCONFcheck = customCONFcheck
// when registering function
// (see end of this file)
//
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
        long fpi_mappingXX = functionparameter_GetParamIndex(milk_data.fpsptr, ".mappingXX");
        long fpi_mappingYY = functionparameter_GetParamIndex(milk_data.fpsptr, ".mappingYY");
        long fpi_mappingXY = functionparameter_GetParamIndex(milk_data.fpsptr, ".mappingXY");
        long fpi_mappingYX = functionparameter_GetParamIndex(milk_data.fpsptr, ".mappingYX");
        long fpi_spotx0    = functionparameter_GetParamIndex(milk_data.fpsptr, ".spotx0");
        long fpi_spoty0    = functionparameter_GetParamIndex(milk_data.fpsptr, ".spoty0");
        long fpi_searchrad = functionparameter_GetParamIndex(milk_data.fpsptr, ".searchrad");
        long fpi_spotsize  = functionparameter_GetParamIndex(milk_data.fpsptr, ".spotsize");

        // allow for change of parameter during runtime
        if (fpi_mappingXX > -1)
        {
            milk_data.fpsptr->parray[fpi_mappingXX].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_mappingYY > -1)
        {
            milk_data.fpsptr->parray[fpi_mappingYY].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_mappingXY > -1)
        {
            milk_data.fpsptr->parray[fpi_mappingXY].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_mappingYX > -1)
        {
            milk_data.fpsptr->parray[fpi_mappingYX].fpflag |= FPFLAG_WRITERUN;
        }

        if (fpi_spotx0 > -1)
        {
            milk_data.fpsptr->parray[fpi_spotx0].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_spoty0 > -1)
        {
            milk_data.fpsptr->parray[fpi_spoty0].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_searchrad > -1)
        {
            milk_data.fpsptr->parray[fpi_searchrad].fpflag |= FPFLAG_WRITERUN;
        }
        if (fpi_spotsize > -1)
        {
            milk_data.fpsptr->parray[fpi_spotsize].fpflag |= FPFLAG_WRITERUN;
        }
    }

    return RETURN_SUCCESS;
}

// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


static errno_t spot_position(IMGID *inimg,
                             IMGID *indarkimg,
                             float  spot_size __attribute__((unused)),
                             float  spot_x0,
                             float  spot_y0,
                             float  spot_searchrad,
                             IMGID *outdatimg,
                             float  mappingXX,
                             float  mappingYY,
                             float  mappingXY,
                             float  mappingYX,
                             IMGID *outvecimg)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code

    // check input image exists
    resolveIMGID(inimg, ERRMODE_WARN, dcimg, dcnimg);
    if (inimg->ID == -1)
    {
        return RETURN_FAILURE;
    }

    // get image size
    uint32_t xsize  = inimg->md->size[0];
    uint32_t ysize  = inimg->md->size[1];
    uint64_t xysize = (uint64_t) xsize;
    xysize *= ysize;


    // check if dark image exists
    resolveIMGID(indarkimg, ERRMODE_NULL, dcimg, dcnimg);

    // Checko output
    //
    resolveIMGID(outdatimg, ERRMODE_WARN, dcimg, dcnimg);
    if (outdatimg->ID == -1)
    {
        return RETURN_FAILURE;
    }
    resolveIMGID(outvecimg, ERRMODE_WARN, dcimg, dcnimg);
    if (outvecimg->ID == -1)
    {
        return RETURN_FAILURE;
    }


    float    xstart  = spot_x0 - spot_searchrad;
    uint32_t iistart = 0;
    if (xstart > 0.0)
    {
        iistart = (uint32_t) xstart;
    }

    float    xend  = spot_x0 + spot_searchrad;
    uint32_t iiend = xsize;
    if (xend < xsize)
    {
        iiend = (uint32_t) xend;
    }


    float    ystart  = spot_y0 - spot_searchrad;
    uint32_t jjstart = 0;
    if (ystart > 0.0)
    {
        jjstart = (uint32_t) ystart;
    }

    float    yend  = spot_y0 + spot_searchrad;
    uint32_t jjend = ysize;
    if (yend < ysize)
    {
        jjend = (uint32_t) yend;
    }

    double xpos   = 0.0;
    double ypos   = 0.0;
    double sumval = 0.0;
    double pixcnt = 0.0;


    // If dark image is present, subtract it from inimg
    if (indarkimg->ID != -1)
    {
        for (uint32_t ii = iistart; ii < iiend; ii++)
        {
            for (uint32_t jj = jjstart; jj < jjend; jj++)
            {
                float x = 1.0 * ii - spot_x0;
                float y = 1.0 * jj - spot_y0;
                float v =
                    inimg->im->array.F[jj * xsize + ii] - indarkimg->im->array.F[jj * xsize + ii];

                xpos += x * v;
                ypos += y * v;
                sumval += v;
                pixcnt += 1.0;
            }
        }
    }
    else
    {
        for (uint32_t ii = iistart; ii < iiend; ii++)
        {
            for (uint32_t jj = jjstart; jj < jjend; jj++)
            {
                float x = 1.0 * ii - spot_x0;
                float y = 1.0 * jj - spot_y0;
                float v = inimg->im->array.F[jj * xsize + ii];

                xpos += x * v;
                ypos += y * v;
                sumval += v;
                pixcnt += 1.0;
            }
        }
    }

    xpos /= sumval;
    ypos /= sumval;

    outdatimg->im->array.F[0] = xpos;
    outdatimg->im->array.F[1] = ypos;
    outdatimg->im->array.F[2] = xpos + spot_x0;
    outdatimg->im->array.F[3] = ypos + spot_y0;
    outdatimg->im->array.F[4] = sumval;
    outdatimg->im->array.F[5] = pixcnt;

    outvecimg->im->array.F[0] = xpos * mappingXX + ypos * mappingYX;
    outvecimg->im->array.F[1] = xpos * mappingXY + ypos * mappingYY;

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // resolve image and create IMGID
    IMGID inimg = imgid_make_from_name(inimname);
    resolveIMGID(&inimg, ERRMODE_WARN, dcimg, dcnimg);
    if (inimg.ID == -1)
    {
        return RETURN_FAILURE;
    }

    // resolve dark image and create IMGID (optional)
    IMGID indarkimg = imgid_make_from_name(indarkname);
    resolveIMGID(&indarkimg, ERRMODE_NULL, dcimg, dcnimg);

    // Create output
    //
    IMGID outposimg;
    {
        outposimg = stream_connect_create_2D(outspotpos, 6, 1, _DATATYPE_FLOAT);
    }

    IMGID outTTvecimg;
    {
        outTTvecimg = stream_connect_create_2D(outTTvec, 2, 1, _DATATYPE_FLOAT);
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        spot_position(&inimg, &indarkimg, spotsize, spotx0, spoty0, searchrad, &outposimg,
                      mappingXX, mappingYY, mappingXY, mappingYX, &outTTvecimg);

        // stream is updated here, and not in the function called above, so that
        // the above function can be chained with others
        processinfo_update_output_stream(processinfo, outposimg.im, NULL);   // outposimg
        processinfo_update_output_stream(processinfo, outTTvecimg.im, NULL); // outTTvecimg
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&inimg);
    imgid_free(&indarkimg);
    imgid_free(&outposimg);
    imgid_free(&outTTvecimg);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(&FPS_app_info, farg, &CLIcmddata, my_bindings, nb_bindings,
                                        compute_function);
}

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_IOtools__spotpos()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(FPS_app_info,
                                 FPS_PARAMS,
                                 compute_function,
                                 customCONFsetup,
                                 customCONFcheck)
#endif
