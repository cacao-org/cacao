// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_computeCalib_processRM.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

// MILK_CMAKE_REQUEST_MAGMA
// MILK_CMAKE_REQUEST_CUDA

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

#include "milk_runnable.h"

#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "info/info.h"

#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"
#include "linopt_imtools/compute_SVDpseudoInverse.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "computeCalib/computeCalib.h"

#include "linalgebra/linalgebra.h"


//extern AOLOOPCONTROL_CONF *AOconf;            // declared in AOloopControl.c
//extern AOloopControl_var   aoloopcontrol_var; // declared in AOloopControl.c

// static long aoconfID_imWFS2_active[100];

/**
 * ====================================================
 * FPS section: mkCMsvd (make control matrix)
 * ====================================================
 */

// Section 2: Local variables for FPS parameters
static int64_t mkCM_AOloopindex;
static double  mkCM_SVDlim;
static char    mkCM_fname_respM[FUNCTION_PARAMETER_STRMAXLEN];
static int64_t mkCM_GPUmode;
static char    mkCM_out_label[FUNCTION_PARAMETER_STRMAXLEN];

// Section 3: FPS_PARAMS X-macro
#define FPS_PARAMS(X)                                                                         \
    X(".AOloopindex", &mkCM_AOloopindex, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "loop index") \
    X(".SVDlim", &mkCM_SVDlim, FPTYPE_FLOAT64, 1,                                             \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT), "SVD limit value")          \
    X(".fname_respM", mkCM_fname_respM, FPTYPE_FILENAME, 1,                                   \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED), "response matrix")                   \
    X(".GPUmode", &mkCM_GPUmode, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT, "Using GPU ?")        \
    X(".out.label", mkCM_out_label, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "output label")

MILK_PARAMS_DEFINE(mkCMsvd, FPS_PARAMS);


// Section 4: compute function (merged CONF+RUN)
static MILK_HOT errno_t mkCMsvd_compute(const MILK_RUNNABLE *self)
{
    (void) self;
    DEBUG_TRACE_FSTART();

    // Get output directory from FPS
    char outdirname[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(outdirname, functionparameter_GetParamPtr_STRING(milk_data.fpsptr, ".conf.datadir"),
            FUNCTION_PARAMETER_STRMAXLEN);
    EXECUTE_SYSTEM_COMMAND("mkdir -p %s", outdirname);

    load_fits(mkCM_fname_respM, "respM", 1, NULL);

    char cm_name[] = "sCMat";

#ifdef HAVE_MAGMA
    if (mkCM_GPUmode)
    {
        LINALGEBRA_magma_compute_SVDpseudoInverse("respM", cm_name, mkCM_SVDlim, 100000, "VTmat", 0,
                                                  1, 64, 0, NULL);
    }
    else
    {
#endif
        linopt_compute_SVDpseudoInverse("respM", cm_name, mkCM_SVDlim, 10000, "VTmat", NULL);
#ifdef HAVE_MAGMA
    }
#endif

    {
        char ffname[STRINGMAXLEN_FULLFILENAME];

        WRITE_FULLFILENAME(ffname, "%s/VTmat.fits", outdirname);
        save_fits("VTmat", ffname);

        imageID  ID_VTmat = image_ID("VTmat", dcimg, dcnimg);
        imageID  ID_DMmodes;
        uint32_t DMxsize  = atoi(getenv("CACAO_DMxsize"));
        uint32_t DMysize  = atoi(getenv("CACAO_DMysize"));
        uint32_t DMxysize = DMxsize * DMysize;
        create_3Dimage_ID("DMmodes", DMxsize, DMysize, DMxysize, &ID_DMmodes);
        list_image_ID();
        for (uint32_t kk = 0; kk < DMxysize; kk++)
        {
            for (uint32_t ii = 0; ii < DMxysize; ii++)
            {
                dcimg[ID_DMmodes].array.F[kk * DMxysize + ii] =
                    dcimg[ID_VTmat].array.F[ii * DMxysize + kk];
            }
        }
        delete_image_ID("VTmat", DELETE_IMAGE_ERRMODE_WARNING);
        WRITE_FULLFILENAME(ffname, "%s/DMmodes.fits", outdirname);
        save_fits("DMmodes", ffname);

        imageID  ID_WFSmodes;
        uint32_t WFSxsize;
        uint32_t WFSysize;

        imageID IDrespM = image_ID("respM", dcimg, dcnimg);
        WFSxsize        = dcimg[IDrespM].md->size[0];
        WFSysize        = dcimg[IDrespM].md->size[1];

        uint32_t WFSxysize = WFSxsize * WFSysize;
        create_3Dimage_ID("WFSmodes", WFSxsize, WFSysize, DMxysize, &ID_WFSmodes);
        printf("Computing WFS modes ...\n");
        fflush(stdout);
        int mimax = 50;
        for (int mi = 0; mi < mimax; mi++)
        {
            printf("Mode %5d / %5d\n", mi, DMxysize);
            for (uint32_t ii = 0; ii < WFSxysize; ii++)
            {
                dcimg[ID_WFSmodes].array.F[mi * WFSxysize + ii] = 0.0;
                for (uint32_t jj = 0; jj < DMxysize; jj++)
                {
                    dcimg[ID_WFSmodes].array.F[mi * WFSxysize + ii] +=
                        dcimg[ID_DMmodes].array.F[mi * DMxysize + jj] *
                        dcimg[IDrespM].array.F[jj * WFSxysize + ii];
                }
            }
        }
        printf(" DONE\n");
        fflush(stdout);
        WRITE_FULLFILENAME(ffname, "%s/WFSmodes.fits", outdirname);
        save_fits("WFSmodes", ffname);

        WRITE_FULLFILENAME(ffname, "%s/sCMat00.fits", outdirname);
        save_fits(cm_name, ffname);
    }

    delete_image_ID(cm_name, DELETE_IMAGE_ERRMODE_WARNING);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_computeCalib_mkCMsvd = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "mkCMsvd",
                 .cmdkey           = "mkCMsvd",
                 .description      = "compute SVD control matrix from RM",
                 .description_long = "Process a response matrix via SVD to compute the control "
                                     "matrix. Applies singular value filtering and modal "
                                     "truncation." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(mkCMsvd),
    .compute = mkCMsvd_compute,
};

//
// make slave actuators from maskRM
//
long AOloopControl_computeCalib_mkSlavedAct(const char *IDmaskRM_name,
                                            float       pixrad,
                                            const char *IDout_name)
{
    long  IDout;
    long  IDmaskRM;
    long  ii, jj;
    long  ii1, jj1;
    long  xsize, ysize;
    long  pixradl;
    long  ii1min, ii1max, jj1min, jj1max;
    float dx, dy, r;

    IDmaskRM = image_ID(IDmaskRM_name, dcimg, dcnimg);
    xsize    = dcimg[IDmaskRM].md[0].size[0];
    ysize    = dcimg[IDmaskRM].md[0].size[1];

    pixradl = (long) pixrad + 1;

    create_2Dimage_ID(IDout_name, xsize, ysize, &IDout);
    for (ii = 0; ii < xsize * ysize; ii++)
    {
        dcimg[IDout].array.F[ii] = xsize + ysize;
    }

    for (ii = 0; ii < xsize; ii++)
    {
        for (jj = 0; jj < ysize; jj++)
        {
            if (dcimg[IDmaskRM].array.F[jj * xsize + ii] < 0.5)
            {
                ii1min = ii - pixradl;
                if (ii1min < 0)
                {
                    ii1min = 0;
                }
                ii1max = ii + pixradl;
                if (ii1max > (xsize - 1))
                {
                    ii1max = xsize - 1;
                }

                jj1min = jj - pixradl;
                if (jj1min < 0)
                {
                    jj1min = 0;
                }
                jj1max = jj + pixradl;
                if (jj1max > (ysize - 1))
                {
                    jj1max = ysize - 1;
                }

                for (ii1 = ii1min; ii1 < ii1max + 1; ii1++)
                {
                    for (jj1 = jj1min; jj1 < jj1max + 1; jj1++)
                    {
                        if (dcimg[IDmaskRM].array.F[jj1 * xsize + ii1] > 0.5)
                        {
                            dx = 1.0 * (ii - ii1);
                            dy = 1.0 * (jj - jj1);
                            r  = sqrt(dx * dx + dy * dy);
                            if (r < pixrad)
                            {
                                if (r < dcimg[IDout].array.F[jj * xsize + ii])
                                {
                                    dcimg[IDout].array.F[jj * xsize + ii] = r;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for (ii = 0; ii < xsize; ii++)
    {
        for (jj = 0; jj < ysize; jj++)
        {
            if (dcimg[IDout].array.F[jj * xsize + ii] > (xsize + ysize) / 2)
            {
                dcimg[IDout].array.F[jj * xsize + ii] = 0.0;
            }
        }
    }

    return (IDout);
}
