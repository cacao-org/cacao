// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file maskextrapolate.c
 * @brief Maskextrapolate module
 */

/**
 * @file maskextrapolate.c
 *
 */

#include <math.h>


#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"


static FPS_APP_INFO FPS_app_info = {
    .fps_name         = "maskextrapolate",
    .cmdkey           = "maskextrapolate",
    .description      = "mask and extrapolate modes",
    .description_long = "Extrapolate mode shapes beyond the pupil boundary using smooth "
                        "interpolation. Prevents edge discontinuities in DM commands."
};

static char  inmodeC[FUNCTION_PARAMETER_STRMAXLEN];
static char  maskim[FUNCTION_PARAMETER_STRMAXLEN];
static char  extmaskim[FUNCTION_PARAMETER_STRMAXLEN];
static char  outmodeC[FUNCTION_PARAMETER_STRMAXLEN];
static float edgeapo = 0;

#define FPS_PARAMS(X)                                                                           \
    X(".inmodeC", inmodeC, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),     \
      "input modes")                                                                            \
    X(".maskim", maskim, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),       \
      "input mask")                                                                             \
    X(".extmaskim", extmaskim, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "extended input mask")                                                                    \
    X(".outmodeC", outmodeC, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),       \
      "output modes")                                                                           \
    X(".edgeapo", &edgeapo, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "edge apodization strength")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imginmodeC = imgid_make_from_name(inmodeC);
    resolveIMGID(&imginmodeC, ERRMODE_WARN, dcimg, dcnimg);
    if (imginmodeC.ID == -1)
    {
        return RETURN_FAILURE;
    }
    uint32_t xsize  = imginmodeC.md->size[0];
    uint32_t ysize  = imginmodeC.md->size[1];
    uint64_t xysize = xsize;
    xysize *= ysize;
    uint32_t NBmodes = imginmodeC.md->size[2];
    printf("%u modes\n", NBmodes);

    IMGID imgmask = imgid_make_from_name(maskim);
    resolveIMGID(&imgmask, ERRMODE_WARN, dcimg, dcnimg);
    if (imgmask.ID == -1)
    {
        return RETURN_FAILURE;
    }

    IMGID imgextmask = imgid_make_from_name(extmaskim);
    resolveIMGID(&imgextmask, ERRMODE_WARN, dcimg, dcnimg);
    if (imgextmask.ID == -1)
    {
        return RETURN_FAILURE;
    }


    IMGID imgoutmoudeC = imgid_make_from_name_3D(outmodeC, xsize, ysize, NBmodes);
    createimagefromIMGID(&imgoutmoudeC);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        // allocate nearest pixels array
        double *npix_dist2 = (double *) malloc(sizeof(double) * xysize);
        double *npix_coeff = (double *) malloc(sizeof(double) * xysize);
        long   *npix_index = (long *) malloc(sizeof(long) * xysize);

        for (uint32_t ii = 0; ii < xsize; ii++)
        {
            for (uint32_t jj = 0; jj < xsize; jj++)
            {
                if (imgmask.im->array.F[jj * xsize + ii] > 0.5)
                {
                    // in mask -> copy pixel value to output
                    for (uint32_t mi = 0; mi < NBmodes; mi++)
                    {
                        imgoutmoudeC.im->array.F[xysize * mi + jj * xsize + ii] =
                            imginmodeC.im->array.F[xysize * mi + jj * xsize + ii];
                    }
                }
                else if (imgextmask.im->array.F[jj * xsize + ii] > 0.5)
                {
                    // pixel is in extmask, but not in mask -> run extrapolation

                    // find nearest active pixel
                    float nearest_dist2 = xysize;

                    for (uint32_t ii1 = 0; ii1 < xsize; ii1++)
                    {
                        for (uint32_t jj1 = 0; jj1 < ysize; jj1++)
                        {
                            if (imgmask.im->array.F[jj1 * xsize + ii1] > 0.5)
                            {
                                float dx  = (float) ii - ii1;
                                float dy  = (float) jj - jj1;
                                float dr2 = dx * dx + dy * dy;

                                if (dr2 < nearest_dist2)
                                {
                                    nearest_dist2 = dr2;
                                }
                            }
                        }
                    }

                    // Kernel radius
                    int kradint = (int) (sqrtf(nearest_dist2) + 3.0f);

                    int iimin = ii - kradint;
                    if (iimin < 0)
                    {
                        iimin = 0;
                    }
                    int iimax = ii + kradint;
                    if (iimax > (int) xsize)
                    {
                        iimax = xsize;
                    }

                    int jjmin = jj - kradint;
                    if (jjmin < 0)
                    {
                        jjmin = 0;
                    }
                    int jjmax = jj + kradint;
                    if (jjmax > (int) ysize)
                    {
                        jjmax = ysize;
                    }

                    // find nearest pixels
                    //
                    long  npixcnt    = 0;
                    float coefftotal = 0.0f;
                    float alpha1     = 1.0f / (edgeapo * nearest_dist2);
                    for (int ii1 = iimin; ii1 < iimax; ii1++)
                    {
                        for (int jj1 = jjmin; jj1 < jjmax; jj1++)
                        {
                            float dx  = (float) ii - ii1;
                            float dy  = (float) jj - jj1;
                            float dr2 = dx * dx + dy * dy;

                            //if(dr2 < nearest_dist2 + 0.2) // only consider nearest pixels
                            //{
                            npix_dist2[npixcnt] = dr2;
                            npix_index[npixcnt] = jj1 * xsize + ii1;
                            npix_coeff[npixcnt] = expf(-alpha1 * dr2);
                            coefftotal += npix_coeff[npixcnt];
                            npixcnt++;
                            //}
                        }
                    }


                    // nearest pixel
                    //
                    for (uint32_t mi = 0; mi < NBmodes; mi++)
                    {
                        for (long npixi = 0; npixi < npixcnt; npixi++)
                        {
                            imgoutmoudeC.im->array.F[xysize * mi + jj * xsize + ii] +=
                                imginmodeC.im->array.F[xysize * mi + npix_index[npixi]] *
                                npix_coeff[npixi];
                        }
                        imgoutmoudeC.im->array.F[xysize * mi + jj * xsize + ii] /= coefftotal;
                    }
                }
            }
        }

        free(npix_dist2);
        free(npix_coeff);
        free(npix_index);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


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
errno_t CLIADDCMD_AOloopControl_computeCalib__maskextrapolate()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(FPS_app_info,
                                 FPS_PARAMS,
                                 compute_function,

                                 customCONFcheck)
#endif
