/**
 * @file    AOloopControl_DM_TTcircle_astrogrid.c
 * @brief   Create DM grid patterns for calibrations
 *
 * Pattens are saved as 3D cube, for use by DMcomb
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"




// Local variables pointers

// output img name
static char *outname;

static uint32_t *xsize;
static uint32_t *ysize;

static uint32_t *XYpattern;

static uint32_t *binfactor;




static CLICMDARGDEF farg[] = {{CLIARG_STR_NOT_IMG,
                               ".outname",
                               "output image name",
                               "DMgridc",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &outname,
                               NULL},
                              {CLIARG_UINT32,
                               ".xsize",
                               "x size",
                               "50",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &xsize,
                               NULL},
                              {CLIARG_UINT32,
                               ".ysize",
                               "y size",
                               "50",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &ysize,
                               NULL},
                              {CLIARG_UINT32,
                               ".XYpattern",
                               "grid pattern",
                               "3",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &XYpattern,
                               NULL},
                              {CLIARG_UINT32,
                               ".binfact",
                               "binning factor",
                               "2",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &binfactor,
                               NULL}};




static CLICMDDATA CLIcmddata = {"mk3Ddmgrid",
                                "create DM calibration pattern sequence",
                                CLICMD_FIELDS_DEFAULTS};



// detailed help
static errno_t help_function()
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

    float map4[4] = {0.0, 1.0, 0.0, -1.0};

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
                imgout.im->array.F[jj * xsize + ii] =
                    2.0 * ((ii / bin) % 2) - 1;
            }
        }
        break;

    case 2: // Y
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] =
                    2.0 * ((jj / bin) % 2) - 1;
            }
        }
        break;

    case 3: // Xdiag
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] =
                    map4[(((ii + jj) / bin) % 4)];
            }
        }
        break;

    case 4: // Ydiag
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                imgout.im->array.F[jj * xsize + ii] =
                    map4[(((ysize + ii - jj) / bin) % 4)];
            }
        }
        break;

    case 5: // X then Y
        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < ysize; jj++)
            {
                // X
                imgout.im->array.F[jj * xsize + ii] =
                    2.0 * ((ii / bin) % 2) - 1;

                // Y
                imgout.im->array.F[2 * xysize + jj * xsize + ii] =
                    2.0 * ((jj / bin) % 2) - 1;
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
            imgout.im->array.F[3 * xysize + ii] =
                -imgout.im->array.F[2 * xysize + ii];
        }
    }


    return RETURN_SUCCESS;
}




/**
 * @brief Wrapper function, used by all CLI calls
 *
 * INSERT_STD_PROCINFO statements enable processinfo support
 */
static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    //    IMGID img = makeIMGID(inimname);
    //inim.name);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    make_3Dgrid_DMsequ(outname, *xsize, *ysize, *XYpattern, *binfactor);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions

    errno_t
    CLIADDCMD_AOloopControl_DM__mk3Ddmgrid()
{
    INSERT_STD_CLIREGISTERFUNC

    // Optional custom settings for this function can be included
    // CLIcmddata.cmdsettings->procinfo_loopcntMax = 9;

    return RETURN_SUCCESS;
}
