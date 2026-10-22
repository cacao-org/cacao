// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_DM_TTcircle_astrogrid.c
 * @brief   Create DM grid patterns for calibrations
 *
 * Pattens are saved as 3D cube, for use by DMcomb
 *
 */

#include <math.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

// Local variables pointers

// output img name
static char     outname[FUNCTION_PARAMETER_STRMAXLEN];
static uint32_t xsize     = 0;
static uint32_t ysize     = 0;
static uint32_t XYpattern = 0;
static uint32_t binfactor = 1;

#define FPS_PARAMS(X)                                                                          \
    X(".outname", outname, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),        \
      "output image name")                                                                     \
    X(".xsize", &xsize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "x size") \
    X(".ysize", &ysize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "y size") \
    X(".XYpattern", &XYpattern, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),   \
      "grid pattern")                                                                          \
    X(".binfact", &binfactor, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),     \
      "binning factor")

MILK_PARAMS_DEFINE(mk3Ddmgrid, FPS_PARAMS);


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


//
// patterns
//	0 : XYdiag   [2 frames]
//	1 : X        [2 frames]
//	2 : Y        [2 frames]
//	3 : Xdiag    [2 frames]
//	4 : Ydiag    [2 frames]
//  5 : XYdiag0  [4 frames]
//

long make_3Dgrid_DMsequ(char    *IDoutname,
                        uint32_t xsize,
                        uint32_t ysize,
                        uint32_t XYmode,
                        uint32_t bin)
{
    uint64_t xysize = xsize;
    xysize *= ysize;


    uint32_t zsize = 2;
    if (XYmode == 5)
    {
        zsize = 4;
    }

    IMGID imgout = stream_connect_create_3Df32(IDoutname, xsize, ysize, zsize);

    float map4[4] = { 0.0, 1.0, 0.0, -1.0 };

    switch (XYmode)
    {
    case 0: // XYdiag
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] =
                    2.0 * ((((ii / bin) % 2 + (jj / bin) % 2)) % 2) - 1;
            }
        }
        break;

    case 1: // X
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] = 2.0 * ((ii / bin) % 2) - 1;
            }
        }
        break;

    case 2: // Y
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] = 2.0 * ((jj / bin) % 2) - 1;
            }
        }
        break;

    case 3: // Xdiag
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] = map4[(((ii + jj) / bin) % 4)];
            }
        }
        break;

    case 4: // Ydiag
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] = map4[(((ysize + ii - jj) / bin) % 4)];
            }
        }
        break;

    case 5: // X then Y
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                // X
                imgout.im->array.F[jj * xsize + ii] = 2.0 * ((ii / bin) % 2) - 1;

                // Y
                imgout.im->array.F[2 * xysize + jj * xsize + ii] = 2.0 * ((jj / bin) % 2) - 1;
            }
        }
        break;
    }


    // repeat pattern to slices > 0
    for (uint64_t ii = 0; ii < xysize; ii++)
    {
        imgout.im->array.F[xysize + ii] = -imgout.im->array.F[ii];
    }

    if (zsize == 4)
    {
        for (uint64_t ii = 0; ii < xysize; ii++)
        {
            imgout.im->array.F[3 * xysize + ii] = -imgout.im->array.F[2 * xysize + ii];
        }
    }


    return RETURN_SUCCESS;
}


/**
 * @brief Wrapper function, used by all CLI calls
 */
static MILK_HOT errno_t mk3Ddmgrid_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        make_3Dgrid_DMsequ(outname, xsize, ysize, XYpattern, binfactor);
    }
    MILK_PROCINFO_END(self)

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_AOloopControl_DM_mk3Ddmgrid = {
    .kind = MILK_RUNNABLE_FPS,
    .app = {
        .fps_name         = "mk3Ddmgrid",
        .cmdkey           = "mk3Ddmgrid",
        .description      = "create DM calibration pattern sequence",
        .description_long = "Create a 3D sequence of DM calibration patterns. Generates systematic "
                            "poke patterns for response matrix acquisition.",
    },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(mk3Ddmgrid),
    .compute = mk3Ddmgrid_compute,
};
