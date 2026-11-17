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
#include <stdlib.h>

#ifdef _OPENMP
#    include <omp.h>
#endif

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "computeCalib.h"


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

MILK_PARAMS_DEFINE(maskextrapolate, FPS_PARAMS);


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


static MILK_HOT errno_t maskextrapolate_compute(const MILK_RUNNABLE *self)
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


    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        /* Precompute compact coordinate table of active mask pixels once */
        uint32_t *act_x = (uint32_t *) malloc(sizeof(uint32_t) * xysize);
        uint32_t *act_y = (uint32_t *) malloc(sizeof(uint32_t) * xysize);
        if (act_x == NULL || act_y == NULL)
        {
            free(act_x);
            free(act_y);
            PRINT_ERROR("malloc returns NULL pointer");
            return RETURN_FAILURE;
        }

        uint64_t     nact   = 0;
        const float *mask_p = imgmask.im->array.F;
        const float *ext_p  = imgextmask.im->array.F;
        const float *in_p   = imginmodeC.im->array.F;
        float       *out_p  = imgoutmoudeC.im->array.F;

        for (uint32_t jj1 = 0; jj1 < ysize; jj1++)
        {
            for (uint32_t ii1 = 0; ii1 < xsize; ii1++)
            {
                if (mask_p[(uint64_t) jj1 * xsize + ii1] > 0.5f)
                {
                    act_x[nact] = ii1;
                    act_y[nact] = jj1;
                    nact++;
                }
            }
        }

#ifdef _OPENMP
#    pragma omp parallel
#endif
        {
            long   *npix_index = (long *) malloc(sizeof(long) * xysize);
            double *npix_coeff = (double *) malloc(sizeof(double) * xysize);

#ifdef _OPENMP
#    pragma omp for schedule(dynamic, 32)
#endif
            for (uint64_t pix = 0; pix < xysize; pix++)
            {
                uint32_t ii = (uint32_t) (pix % xsize);
                uint32_t jj = (uint32_t) (pix / xsize);

                if (mask_p[pix] > 0.5f)
                {
                    for (uint32_t mi = 0; mi < NBmodes; mi++)
                    {
                        out_p[xysize * mi + pix] = in_p[xysize * mi + pix];
                    }
                }
                else if (ext_p[pix] > 0.5f && nact > 0 && npix_index != NULL &&
                         npix_coeff != NULL)
                {
                    float nearest_dist2 = (float) xysize;
                    for (uint64_t k = 0; k < nact; k++)
                    {
                        float dx  = (float) ii - (float) act_x[k];
                        float dy  = (float) jj - (float) act_y[k];
                        float dr2 = dx * dx + dy * dy;
                        if (dr2 < nearest_dist2)
                        {
                            nearest_dist2 = dr2;
                        }
                    }

                    int kradint = (int) (sqrtf(nearest_dist2) + 3.0f);
                    int iimin   = ((int) ii > kradint) ? ((int) ii - kradint) : 0;
                    int iimax =
                        ((int) ii + kradint < (int) xsize) ? ((int) ii + kradint) : (int) xsize;
                    int jjmin = ((int) jj > kradint) ? ((int) jj - kradint) : 0;
                    int jjmax =
                        ((int) jj + kradint < (int) ysize) ? ((int) jj + kradint) : (int) ysize;

                    long  npixcnt    = 0;
                    float coefftotal = 0.0f;
                    float alpha1     = 1.0f / (edgeapo * nearest_dist2);
                    for (int ii1 = iimin; ii1 < iimax; ii1++)
                    {
                        for (int jj1 = jjmin; jj1 < jjmax; jj1++)
                        {
                            float dx     = (float) ii - (float) ii1;
                            float dy     = (float) jj - (float) jj1;
                            float dr2    = dx * dx + dy * dy;
                            double coeff = (double) expf(-alpha1 * dr2);

                            npix_index[npixcnt] = (long) jj1 * (long) xsize + (long) ii1;
                            npix_coeff[npixcnt] = coeff;
                            coefftotal += (float) coeff;
                            npixcnt++;
                        }
                    }

                    for (uint32_t mi = 0; mi < NBmodes; mi++)
                    {
                        double       acc   = 0.0;
                        const float *src_m = &in_p[xysize * mi];
                        for (long npixi = 0; npixi < npixcnt; npixi++)
                        {
                            acc += (double) src_m[npix_index[npixi]] * npix_coeff[npixi];
                        }
                        out_p[xysize * mi + pix] =
                            (coefftotal > 0.0f) ? (float) (acc / (double) coefftotal) : 0.0f;
                    }
                }
                else
                {
                    for (uint32_t mi = 0; mi < NBmodes; mi++)
                    {
                        out_p[xysize * mi + pix] = 0.0f;
                    }
                }
            }

            free(npix_coeff);
            free(npix_index);
        }

        free(act_y);
        free(act_x);
    }
    MILK_PROCINFO_END(self)


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_computeCalib_maskextrapolate = {
    .kind = MILK_RUNNABLE_FPS,
    .app  = { .fps_name         = "maskextrapolate",
              .cmdkey           = "maskextrapolate",
              .description      = "mask and extrapolate modes",
              .description_long = "Extrapolate mode shapes beyond the pupil boundary using smooth "
                                  "interpolation. Prevents edge discontinuities in DM commands." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(maskextrapolate),
    .compute   = maskextrapolate_compute,
    .confcheck = customCONFcheck,
};
EXPORT_FPSEXEC(cacao, "cacaocc-maskextrapolate", runnable_computeCalib_maskextrapolate);
