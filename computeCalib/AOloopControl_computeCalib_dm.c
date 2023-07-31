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



imageID AOloopControl_computeCalib_DMedgeDetect(const char *IDmaskRM_name,
        const char *IDout_name)
{
    imageID IDout;
    imageID IDmaskRM;
    long    ii, jj;
    float   val1;
    long    xsize, ysize;

    IDmaskRM = image_ID(IDmaskRM_name);
    xsize    = data.image[IDmaskRM].md[0].size[0];
    ysize    = data.image[IDmaskRM].md[0].size[1];

    create_2Dimage_ID(IDout_name, xsize, ysize, &IDout);

    for(ii = 1; ii < xsize - 1; ii++)
        for(jj = 1; jj < ysize - 1; jj++)
        {
            val1 = 0.0;
            if(data.image[IDmaskRM].array.F[jj * xsize + ii] > 0.5)
            {
                if(data.image[IDmaskRM].array.F[jj * xsize + ii + 1] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[jj * xsize + ii - 1] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj + 1) * xsize + ii] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 1) * xsize + ii] < 0.5)
                {
                    val1 += 1.0;
                }

                if(data.image[IDmaskRM].array.F[(jj + 1) * xsize + ii + 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj + 1) * xsize + ii - 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 1) * xsize + ii + 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 1) * xsize + ii - 1] <
                        0.5)
                {
                    val1 += 1.0;
                }

                if(data.image[IDmaskRM].array.F[jj * xsize + ii + 2] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[jj * xsize + ii - 2] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj + 2) * xsize + ii] < 0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 2) * xsize + ii] < 0.5)
                {
                    val1 += 1.0;
                }

                if(data.image[IDmaskRM].array.F[(jj + 1) * xsize + ii + 2] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj + 1) * xsize + ii - 2] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 1) * xsize + ii + 2] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 1) * xsize + ii - 2] <
                        0.5)
                {
                    val1 += 1.0;
                }

                if(data.image[IDmaskRM].array.F[(jj + 2) * xsize + ii - 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 2) * xsize + ii - 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj + 2) * xsize + ii + 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
                if(data.image[IDmaskRM].array.F[(jj - 2) * xsize + ii + 1] <
                        0.5)
                {
                    val1 += 1.0;
                }
            }
            if(val1 > 4.9)
            {
                val1 = 1.0;
            }
            else
            {
                val1 = 0.0;
            }
            data.image[IDout].array.F[jj * xsize + ii] = val1;
        }

    return IDout;
}




long AOloopControl_computeCalib_DMextrapolateModes(const char *IDin_name,
        const char *IDmask_name,
        const char *IDcpa_name,
        const char *IDout_name)
{
    imageID IDin, IDmask, IDcpa, IDout;
    long    xsize, ysize, zsize, xysize;
    long    IDpixdist;
    long    ii, jj, ii1, jj1, dii, djj, dii2, djj2;
    float   r, dist;
    float   coeff;
    long    index;
    long    kk;

    IDin  = image_ID(IDin_name);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
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
    xysize = xsize * ysize;

    IDmask = image_ID(IDmask_name);
    IDcpa  = image_ID(IDcpa_name);

    // measure pixel distance to mask
    create_2Dimage_ID("pixmaskdist", xsize, ysize, &IDpixdist);
    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
        {
            dist = 1.0 * xsize + 1.0 * ysize;
            for(ii1 = 0; ii1 < xsize; ii1++)
                for(jj1 = 0; jj1 < ysize; jj1++)
                {
                    if(data.image[IDmask].array.F[jj1 * xsize + ii1] > 0.5)
                    {
                        dii  = ii1 - ii;
                        djj  = jj1 - jj;
                        dii2 = dii * dii;
                        djj2 = djj * djj;
                        r    = sqrt(dii2 + djj2);
                        if(r < dist)
                        {
                            dist = r;
                        }
                    }
                }
            data.image[IDpixdist].array.F[jj * xsize + ii] = dist;
        }
    // save_fits("pixmaskdist", "_tmp_pixmaskdist.fits");
    // save_fits(IDcpa_name, "_tmp_cpa.fits");
    for(kk = 0; kk < zsize; kk++)
    {
        for(ii = 0; ii < xsize; ii++)
            for(jj = 0; jj < ysize; jj++)
            {
                index = jj * xsize + ii;
                coeff = data.image[IDpixdist].array.F[index] /
                        ((1.0 * xsize / (data.image[IDcpa].array.F[kk] + 0.1)) *
                         0.8);

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
    delete_image_ID("pixmaskdist", DELETE_IMAGE_ERRMODE_WARNING);

    return (IDout);
}




long AOloopControl_computeCalib_DMslaveExt(const char *IDin_name,
        const char *IDmask_name,
        const char *IDsl_name,
        const char *IDout_name,
        float       r0)
{
    long  IDin, IDmask, IDsl, IDout;
    long  ii, jj, kk, ii1, jj1;
    long  xsize, ysize, zsize, xysize;
    long  index;
    float rfactor = 2.0;
    float val1, val1cnt;
    float pixrad;
    long  pixradl;
    long  ii1min, ii1max, jj1min, jj1max;
    float dx, dy, r, r1;
    float coeff;
    float valr;

    IDin  = image_ID(IDin_name);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
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
    xysize = xsize * ysize;

    IDmask = image_ID(IDmask_name);
    IDsl   = image_ID(IDsl_name);

    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
        {
            index = jj * xsize + ii;
            if(data.image[IDmask].array.F[index] > 0.5)
            {
                for(kk = 0; kk < zsize; kk++)
                {
                    data.image[IDout].array.F[kk * xysize + index] =
                        data.image[IDin].array.F[kk * xysize + index];
                }
            }
            else if(data.image[IDsl].array.F[index] > 0.5)
            {
                for(kk = 0; kk < zsize; kk++)
                {
                    val1    = 0.0;
                    val1cnt = 0.0;
                    pixrad  = (rfactor * data.image[IDsl].array.F[index] + 1.0);
                    pixradl = (long) pixrad + 1;

                    ii1min = ii - pixradl;
                    if(ii1min < 0)
                    {
                        ii1min = 0;
                    }
                    ii1max = ii + pixradl;
                    if(ii1max > (xsize - 1))
                    {
                        ii1max = xsize - 1;
                    }

                    jj1min = jj - pixradl;
                    if(jj1min < 0)
                    {
                        jj1min = 0;
                    }
                    jj1max = jj + pixradl;
                    if(jj1max > (ysize - 1))
                    {
                        jj1max = ysize - 1;
                    }

                    valr = 0.0;
                    for(ii1 = ii1min; ii1 < ii1max + 1; ii1++)
                        for(jj1 = jj1min; jj1 < jj1max + 1; jj1++)
                        {
                            dx = 1.0 * (ii - ii1);
                            dy = 1.0 * (jj - jj1);
                            r  = sqrt(dx * dx + dy * dy);
                            if((r < pixrad) &&
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
                        }
                    valr /= val1cnt;
                    if(val1cnt > 0.0001)
                    {
                        data.image[IDout].array.F[kk * xysize + index] =
                            (val1 / val1cnt) * exp(-(valr / r0) * (valr / r0));
                    }
                }
            }
            else
                for(kk = 0; kk < zsize; kk++)
                {
                    data.image[IDout].array.F[kk * xysize + index] = 0.0;
                }
        }

    return (IDout);
}
