/**
 * @file modes_spatial_extrapolate.c
 * @brief Modes spatial extrapolate module
 */

#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


errno_t modes_spatial_extrapolate(IMGID imgmodes,
                                  IMGID imgmask,
                                  IMGID imgcpa,
                                  IMGID *imgoutmodes)
{
    DEBUG_TRACE_FSTART();

    printf("extrapolate ...\n");

    resolveIMGID(
        &imgmodes, ERRMODE_ABORT,
        data.core.image,
        data.core.NB_MAX_IMAGE);
    resolveIMGID(
        &imgmask, ERRMODE_ABORT,
        data.core.image,
        data.core.NB_MAX_IMAGE);
    resolveIMGID(
        &imgcpa, ERRMODE_ABORT,
        data.core.image,
        data.core.NB_MAX_IMAGE);

    imcreatelikewiseIMGID(imgoutmodes, &imgmodes);

    IMGID imgpixmdist = imgid_make_from_name("pmindist");
    imcreatelikewiseIMGID(&imgpixmdist, &imgmask);

    uint32_t xsize  = imgmodes.md->size[0];
    uint32_t ysize  = imgmodes.md->size[1];
    uint64_t xysize = ((uint64_t) xsize) * ysize;

    for(uint32_t ii = 0; ii < xsize; ii++)
    {
        for(uint32_t jj = 0; jj < ysize; jj++)
        {
            // initialize mindist to maximum possible distance
            double mindist = 1.0 * xsize + 1.0 * ysize;

            for(uint32_t ii1 = 0; ii1 < xsize; ii1++)
            {
                for(uint32_t jj1 = 0; jj1 < ysize; jj1++)
                {
                    if(imgmask.im->array.F[jj1 * xsize + ii1] > 0.5)
                    {
                        double dii = (double) ii1 - (double) ii;
                        double djj = (double) jj1 - (double) jj;

                        double dii2 = dii * dii;
                        double djj2 = djj * djj;

                        double r2 = 1.0 * dii2 + 1.0 * djj2;

                        double r = sqrt(r2);

                        if(r < mindist)
                        {
                            mindist = r;
                        }
                    }
                }
            }
            imgpixmdist.im->array.F[jj * xsize + ii] = mindist;
        }
    }

    // save_fits("pmindist", "pmindist.fits");


    for(uint32_t kk = 0; kk < imgmodes.md->size[2]; kk++)
    {
        for(uint32_t ii = 0; ii < xsize; ii++)
        {
            for(uint32_t jj = 0; jj < ysize; jj++)
            {
                uint64_t pindex = ((uint64_t) jj) * xsize + ii;

                double coeff = imgpixmdist.im->array.F[pindex];
                coeff /= (1.0 * xsize / (imgcpa.im->array.F[kk] + 0.1) * 0.8);
                coeff = (exp(-coeff * coeff) - exp(-1.0)) / (1.0 - exp(-1.0));
                if(coeff < 0.0)
                {
                    coeff = 0.0;
                }

                imgoutmodes->im->array.F[kk * xysize + jj * xsize + ii] =
                    coeff * imgmodes.im->array.F[kk * xysize + jj * xsize + ii];
            }
        }
    }

    delete_image(&imgpixmdist, ERRMODE_WARN);
    imgid_free(&imgpixmdist);


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}
