/**
 * @file    AOloopControl_computeCalib_dm.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                        HEADER FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#include <malloc.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <time.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>

#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "computeCalib/computeCalib.h"

#include "linalgebra/linalgebra.h"




/**
 * @brief      Detects the edge of a deformable mirror (DM) mask.
 *
 * @param[in]  IDmaskRM_name The name of the input DM response matrix mask image.
 * @param[in]  IDout_name    The name for the output image where the detected edge will be stored.
 * @return     The imageID of the newly created output image containing the edge.
 *
 * This function identifies the pixels that form the boundary of the active region
 * in a DM mask. It iterates through each pixel of the input mask (`IDmaskRM_name`).
 * A pixel is considered part of the edge if its value is greater than 0.5 (active)
 * and it has at least 5 neighboring pixels (within a 24-pixel neighborhood)
 * with a value less than 0.5 (inactive). The resulting binary edge mask is
 * stored in a new image created with the name `IDout_name`.
 */
imageID AOloopControl_computeCalib_DMedgeDetect(
    const char *IDmaskRM_name,
    const char *IDout_name)
{
    imageID IDout;
    imageID IDmaskRM;
    long    ii, jj;
    float   val1;
    long    xsize, ysize;

    // Offsets for 24-neighborhood (distance 1 and 2)
    const int dx[24] = {  1, -1,  0,  0,  1,  1, -1, -1,
              2, -2,  0,  0,  1,  1, -1, -1,
             -1, -1,  1,  1,  2, -2,  2, -2 };
    const int dy[24] = {  0,  0,  1, -1,  1, -1,  1, -1,
              0,  0,  2, -2,  2, -2,  2, -2,
              2, -2,  2, -2,  1,  1, -1, -1 };

    IDmaskRM = image_ID(IDmaskRM_name);
    xsize    = data.image[IDmaskRM].md[0].size[0];
    ysize    = data.image[IDmaskRM].md[0].size[1];

    create_2Dimage_ID(IDout_name, xsize, ysize, &IDout);

    for(ii = 2; ii < xsize - 2; ii++)
        for(jj = 2; jj < ysize - 2; jj++)
        {
            val1 = 0.0;
            // Access the mask value once per pixel
            float mask_val = data.image[IDmaskRM].array.F[jj * xsize + ii];
            if(mask_val > 0.5)
            {
                for(int k = 0; k < 24; ++k)
                {
                    int ni = ii + dx[k];
                    int nj = jj + dy[k];
                    // Access neighbor mask value once
                    if(data.image[IDmaskRM].array.F[nj * xsize + ni] < 0.5)
                        val1 += 1.0f; // Use float literal for consistency
                }
            }
            data.image[IDout].array.F[jj * xsize + ii] = (val1 > 4.9f) ? 1.0f : 0.0f; // Use float literals
        }

    return IDout;
}





/**
 * @brief Extrapolates DM modes based on a mask and CPA values.
 *
 * This function takes an input image of DM (`IDin_name`), a mask image (`IDmask_name`),
 * and an image containing CPA (Cycles Per Aperture) values for each mode (`IDcpa_name`).
 * It then extrapolates the DM to regions outside the mask based on the distance
 * to the mask and the CPA value of each mode. The extrapolated modes are stored
 * in a new image named `IDout_name`.
 *
 * @param IDin_name The name of the input image containing DM modes (2D or 3D).
 * @param IDmask_name The name of the mask image (2D). Pixels > 0.5 are considered part of the mask.
 * @param IDcpa_name The name of the image containing CPA values for each mode (1D or 2D, where the first dimension corresponds to modes).
 * @param IDout_name The name of the output image to store the extrapolated DM modes.
 * @return The imageID of the newly created output image.
 */
long AOloopControl_computeCalib_DMextrapolateModes(
    const char *IDin_name,
    const char *IDmask_name,
    const char *IDcpa_name,
    const char *IDout_name)
{
    imageID IDin = image_ID(IDin_name);
    long xsize = data.image[IDin].md[0].size[0];
    long ysize = data.image[IDin].md[0].size[1];
    long zsize;
    imageID IDout = -1; // Initialize to -1 to indicate no image created yet

    if(data.image[IDin].md[0].naxis == 3)
    {
        zsize = data.image[IDin].md[0].size[2];
        create_3Dimage_ID(IDout_name, xsize, ysize, zsize, &IDout);
    }
    else
    {
        zsize = 1;
        create_2Dimage_ID(IDout_name, xsize, ysize, &IDout);
    }
    long xysize = xsize * ysize;

    imageID IDmask = image_ID(IDmask_name); // Scope: used only in this function
    imageID IDcpa  = image_ID(IDcpa_name);  // Scope: used only in this function

    // Measure pixel distance to the active region of the mask
    long IDpixdist = -1; // Scope: used only in this function
    create_2Dimage_ID("pixmaskdist", xsize, ysize, &IDpixdist);
    for(long ii = 0; ii < xsize; ii++)
    {
        for(long jj = 0; jj < ysize; jj++)
        {
            float dist = 1.0 * xsize + 1.0 * ysize;
            for(long ii1 = 0; ii1 < xsize; ii1++)
            {
                for(long jj1 = 0; jj1 < ysize; jj1++)
                {
                    if(data.image[IDmask].array.F[jj1 * xsize + ii1] > 0.5)
                    {
                        long dii  = ii1 - ii;
                        long djj  = jj1 - jj;
                        long dii2 = dii * dii;
                        long djj2 = djj * djj;
                        float r = sqrt(dii2 + djj2);
                        if(r < dist)
                        {
                            dist = r;
                        }
                    }
                }
            }
            data.image[IDpixdist].array.F[jj * xsize + ii] = dist;
        }
    }

    // Apply extrapolation for each mode
    for(long kk = 0; kk < zsize; kk++)
    {
        for(long ii = 0; ii < xsize; ii++)
        {
            for(long jj = 0; jj < ysize; jj++)
            {
                long index = jj * xsize + ii;
                // Calculate a coefficient based on pixel distance and CPA
                // The CPA value (data.image[IDcpa].array.F[kk]) influences the effective "radius"
                // for extrapolation. Smaller CPA means a larger effective radius, leading to
                // more aggressive extrapolation.
                float coeff = data.image[IDpixdist].array.F[index] /
                        ((1.0 * xsize / (data.image[IDcpa].array.F[kk] + 0.1)) *
                         0.8);

                // Transform the coefficient using an exponential function.
                // This creates a smooth fall-off effect for extrapolation.
                coeff = (exp(-coeff * coeff) - exp(-1.0)) / (1.0 - exp(-1.0));
                if(coeff < 0.0)
                {
                    coeff = 0.0;
                }
                data.image[IDout].array.F[kk * xysize + index] =
                    coeff * data.image[IDin].array.F[kk * xysize + index] *
                    coeff;
            }
        }
    }
    delete_image_ID("pixmaskdist", DELETE_IMAGE_ERRMODE_WARNING);

    return (IDout);
}




long AOloopControl_computeCalib_DMslaveExt(
    const char *IDin_name,
    const char *IDmask_name,
    const char *IDsl_name,
    const char *IDout_name,
    float       r0)
{
    long IDin = image_ID(IDin_name);
    long xsize = data.image[IDin].md[0].size[0];
    long ysize = data.image[IDin].md[0].size[1];
    long zsize;
    long index; // Declare index here
    long kk;    // Declare kk here
    float rfactor = 2.0;
    float val1, val1cnt;
    float pixrad;
    long  pixradl;
    long  ii1min, ii1max, jj1min, jj1max;
    float dx, dy, r, r1;
    float coeff;
    float valr;

    long IDout;
    if(data.image[IDin].md[0].naxis == 3)
    {
        zsize = data.image[IDin].md[0].size[2];
        create_3Dimage_ID(IDout_name, xsize, ysize, zsize, &IDout);
    }
    else
    {
        zsize = 1;
        create_2Dimage_ID(IDout_name, xsize, ysize, &IDout); // IDout is local to this block
    }
    long xysize = xsize * ysize;

    long IDmask = image_ID(IDmask_name); // IDmask is local to this function
    long IDsl = image_ID(IDsl_name);     // IDsl is local to this function

    for(long ii = 0; ii < xsize; ii++)
        for(long jj = 0; jj < ysize; jj++)
        {
            index = jj * xsize + ii;
            if (data.image[IDmask].array.F[index] > 0.5)
            {
                for(kk = 0; kk < zsize; kk++)
                {
                    data.image[IDout].array.F[kk * xysize + index] =
                        data.image[IDin].array.F[kk * xysize + index];
                }
            }
            else if (data.image[IDsl].array.F[index] > 0.5)
            {
                for (kk = 0; kk < zsize; kk++)
                {
                    val1    = 0.0;
                    val1cnt = 0.0;
                    pixrad  = (rfactor * data.image[IDsl].array.F[index] + 1.0);
                    pixradl = (long) pixrad + 1;

                    ii1min = ii - pixradl;
                    if(ii1min < 0)
                        ii1min = 0;
                    ii1max = ii + pixradl;
                    if(ii1max > (xsize - 1))
                        ii1max = xsize - 1;

                    jj1min = jj - pixradl;
                    if(jj1min < 0)
                        jj1min = 0;
                    jj1max = jj + pixradl;
                    if(jj1max > (ysize - 1))
                        jj1max = ysize - 1;

                    valr = 0.0;
                    for(long ii1 = ii1min; ii1 < ii1max + 1; ii1++)
                    {
                        for(long jj1 = jj1min; jj1 < jj1max + 1; jj1++)
                        {
                            dx = 1.0 * (ii - ii1);
                            dy = 1.0 * (jj - jj1);
                            r  = sqrt(dx * dx + dy * dy);
                            if ((r < pixrad) &&
                                (data.image[IDmask].array.F[jj1 * xsize + ii1] >
                                 0.5))
                            {
                                r1    = r / pixrad;
                                coeff = exp(-10.0 * r1 * r1);
                                valr += r * coeff;
                                val1 +=
                                    data.image[IDin]
                                    .array
                                    .F[kk * xysize + jj1 * xsize + ii1] *
                                    coeff;
                                val1cnt += coeff;
                            }
                        } // jj1
                    }     // ii1
                    valr /= val1cnt;
                    if (val1cnt > 0.0001)
                    {
                        data.image[IDout].array.F[kk * xysize + index] =
                            (val1 / val1cnt) * exp(-(valr / r0) * (valr / r0));
                    }
                }
            }
            else // if not in mask and not slaved
            {
                for (kk = 0; kk < zsize; kk++)
                {
                    data.image[IDout].array.F[kk * xysize + index] = 0.0;
                }
            }
        }

    return (IDout);
}
