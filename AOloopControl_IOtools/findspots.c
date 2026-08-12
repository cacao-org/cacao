// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file findspots.c
 * @brief Find spots in WFS image
 *
 *
 */
#include <float.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

// quicksort
#include "COREMOD_tools/COREMOD_tools.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name         = "findspots",
    .cmdkey           = "findspots",
    .description      = "find spots in inmage",
    .description_long = "Detect and locate bright spots in a wavefront sensor image. Uses "
                        "thresholding and centroiding to find subaperture positions."
};

static char     inimname[FUNCTION_PARAMETER_STRMAXLEN];
static float    spotsize     = 0;
static float    spotexcldist = 0;
static uint32_t maxnbspot    = 0;
static char     outmapcname[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X)                                                                        \
    X(".in_name", inimname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "input image")                                                                         \
    X(".spotsize", &spotsize, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),  \
      "approximate spot size")                                                               \
    X(".spotexcldist", &spotexcldist, FPTYPE_FLOAT32, 1,                                     \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "exclusion distance")                       \
    X(".maxnbspot", &maxnbspot, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "max number of spots")                                                                 \
    X(".outmapc", outmapcname, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),  \
      "output mapping cube")

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
    }

    return RETURN_SUCCESS;
}


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


/**
 * find_image_spots() - locate bright spots in an image
 * @inimg:          input image to scan
 * @spot_size:      approximate spot radius [pix]
 * @spot_excl_dist: exclusion distance between found
 *                  spots [pix]
 * @nb_spot_max:    maximum number of spots to find
 *
 * Scans the input image using a median filter of
 * radius @spot_size, then iteratively finds the
 * brightest peak, records its position, and masks
 * a region of @spot_excl_dist around it before
 * searching for the next peak.
 *
 * Creates output streams "spotscan" (median-filtered)
 * and the mapping cube specified by the outmapc FPS
 * parameter.
 */
static errno_t find_image_spots(IMGID    inimg,
                                float    spot_size,
                                float    spot_excl_dist,
                                uint32_t nb_spot_max)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code


    // get image size
    uint32_t xsize  = inimg.md->size[0];
    uint32_t ysize  = inimg.md->size[1];
    uint64_t xysize = (uint64_t) xsize;
    xysize *= ysize;


    // Create output
    //
    IMGID spotoutimg;
    spotoutimg = stream_connect_create_2D("spotscan", xsize, ysize, _DATATYPE_FLOAT);

    // median scan
    float *valarray =
        (float *) malloc(sizeof(float) * 4 * ((int) spot_size + 1) * ((int) spot_size + 1));

    for (uint32_t ii = 0; ii < xsize; ii++)
    {
        for (uint32_t jj = 0; jj < ysize; jj++)
        {
            uint32_t nbpix = 0;

            int ii1min = ii - (int) (spot_size + 1);
            if (ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = ii + (int) (spot_size + 1);
            if (ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jj - (int) (spot_size + 1);
            if (jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jj + (int) (spot_size + 1);
            if (jj1max > (int) ysize)
            {
                jj1max = ysize;
            }

            for (int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = (double) ii - ii1;
                    float dy = (double) jj - jj1;
                    float r2 = dx * dx + dy * dy;
                    if (r2 < spot_size * spot_size)
                    {
                        valarray[nbpix] = inimg.im->array.F[jj1 * xsize + ii1];
                        nbpix++;
                    }
                }
            }

            quick_sort_float(valarray, nbpix);

            float val                               = valarray[(int) (0.5 * nbpix)];
            spotoutimg.im->array.F[jj * xsize + ii] = val;
        }
    }
    free(valarray);


    // Create output map cube
    //
    IMGID mapcimg;
    mapcimg = stream_connect_create_3Df32(outmapcname, xsize, ysize, nb_spot_max);

    // zero out array
    //
    for (uint64_t ii = 0; ii < mapcimg.md->nelement; ii++)
    {
        mapcimg.im->array.F[ii] = 0.0;
    }


    // find spots
    //
    float *spotxarray = (float *) malloc(sizeof(float) * nb_spot_max);
    float *spotyarray = (float *) malloc(sizeof(float) * nb_spot_max);
    float *spotvarray = (float *) malloc(sizeof(float) * nb_spot_max);

    uint32_t spotindex = 0;
    while (spotindex < nb_spot_max)
    {
        // Find strongest peak
        //
        float    vpeak  = -FLT_MAX;
        uint32_t iipeak = 0;
        uint32_t jjpeak = 0;
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                if (spotoutimg.im->array.F[jj * xsize + ii] > vpeak)
                {
                    iipeak = ii;
                    jjpeak = jj;
                    vpeak  = spotoutimg.im->array.F[jj * xsize + ii];
                }
            }
        }
        // report spot
        printf("SPOT %2d   %4u x %4u    %f\n", spotindex, iipeak, jjpeak, vpeak);
        spotxarray[spotindex] = (double) iipeak;
        spotyarray[spotindex] = (double) jjpeak;
        spotvarray[spotindex] = vpeak;


        // zero area around spot
        {
            int ii1min = iipeak - (int) (spot_excl_dist + 1);
            if (ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = iipeak + (int) (spot_excl_dist + 1);
            if (ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jjpeak - (int) (spot_excl_dist + 1);
            if (jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jjpeak + (int) (spot_excl_dist + 1);
            if (jj1max > (int) ysize)
            {
                jj1max = ysize;
            }
            for (int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = (double) iipeak - ii1;
                    float dy = (double) jjpeak - jj1;
                    float r2 = dx * dx + dy * dy;
                    if (r2 < spot_excl_dist * spot_excl_dist)
                    {
                        spotoutimg.im->array.F[jj1 * xsize + ii1] = -FLT_MAX;
                    }
                }
            }
        }


        // write mapc slice
        //
        {
            int ii1min = iipeak - (int) (spot_size + 1);
            if (ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = iipeak + (int) (spot_size + 1);
            if (ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jjpeak - (int) (spot_size + 1);
            if (jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jjpeak + (int) (spot_size + 1);
            if (jj1max > (int) ysize)
            {
                jj1max = ysize;
            }
            double spotflux = 0.0;
            for (int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = (double) iipeak - ii1;
                    float dy = (double) jjpeak - jj1;
                    float r2 = dx * dx + dy * dy;
                    if (r2 < spot_size * spot_size)
                    {
                        mapcimg.im->array.F[xysize * spotindex + jj1 * xsize + ii1] =
                            inimg.im->array.F[jj1 * xsize + ii1];
                        spotflux += inimg.im->array.F[jj1 * xsize + ii1];
                    }
                }
            }
            for (int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    mapcimg.im->array.F[xysize * spotindex + jj1 * xsize + ii1] /= spotflux;
                }
            }
        }

        spotindex++;
    }


    free(spotxarray);
    free(spotyarray);
    free(spotvarray);
    imgid_free(&spotoutimg);
    imgid_free(&mapcimg);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID inimg = imgid_make_from_name(inimname);
    resolveIMGID(&inimg, ERRMODE_WARN, dcimg, dcnimg);
    if (inimg.ID == -1)
    {
        return RETURN_FAILURE;
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        find_image_spots(inimg, spotsize, spotexcldist, maxnbspot);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&inimg);

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
errno_t CLIADDCMD_AOloopControl_IOtools__findspots()
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
