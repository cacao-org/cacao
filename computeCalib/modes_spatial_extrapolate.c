#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"




errno_t modes_spatial_extrapolate(IMGID imgmodes,
                                  IMGID imgmask,
                                  IMGID imgcpa,
                                  IMGID *imgoutmodes)
{
    DEBUG_TRACE_FSTART();

    printf("extrapolate ...\n");

    resolveIMGID(&imgmodes, ERRMODE_ABORT);
    resolveIMGID(&imgmask, ERRMODE_ABORT);
    resolveIMGID(&imgcpa, ERRMODE_ABORT);

    imcreatelikewiseIMGID(imgoutmodes, &imgmodes);

    IMGID imgpixmdist = mkIMGID_from_name("pmindist");
    imcreatelikewiseIMGID(&imgpixmdist, &imgmask);

    uint32_t xsize  = imgmodes.size[0];
    uint32_t ysize  = imgmodes.size[1];
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
                        double dii = 1.0 * ii1 - 1.0 * ii;
                        double djj = 1.0 * jj1 - 1.0 * jj;

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


    for(uint32_t kk = 0; kk < imgmodes.size[2]; kk++)
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


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


/*
long AOloopControl_computeCalib_DMextrapolateModes(
    const char *IDin_name,
    const char *IDmask_name,
    const char *IDcpa_name,
    const char *IDout_name
)
{
    imageID IDin, IDmask, IDcpa, IDout;
    long    xsize, ysize, zsize, xysize;
    long    IDpixdist;
    long    ii, jj, ii1, jj1, dii, djj, dii2, djj2;
    float   r, dist;
    float   coeff;
    long    index;
    long    kk;


    // save_fits("pixmaskdist", "_tmp_pixmaskdist.fits");
    // save_fits(IDcpa_name, "_tmp_cpa.fits");
    for (kk = 0; kk < zsize; kk++)
    {
        for (ii = 0; ii < xsize; ii++)
            for (jj = 0; jj < ysize; jj++)
            {
                index = jj * xsize + ii;
                coeff = data.image[IDpixdist].array.F[index] /
                        ((1.0 * xsize / (data.image[IDcpa].array.F[kk] + 0.1)) *
                         0.8);

                coeff = (exp(-coeff * coeff) - exp(-1.0)) / (1.0 - exp(-1.0));
                if (coeff < 0.0)
                {
                    coeff = 0.0;
                }
                data.image[IDout].array.F[kk * xysize + index] =
                    coeff * data.image[IDin].array.F[kk * xysize + index] *
                    coeff;
            }
    }
    delete_image_ID("pixmaskdist", DELETE_IMAGE_ERRMODE_WARNING);

    return (IDout);
}

*/
