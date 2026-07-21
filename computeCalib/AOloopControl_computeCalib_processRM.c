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

#include "CLIcore.h"
#include "fps.h"

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

/** @brief Process response matrix
 *
 *
 * If images "Hmat" AND "pixindexim" are provided, decode the image
 *  TEST: if "RMpokeC" exists, decode it as well
 */
errno_t AOloopControl_computeCalib_Process_zrespM(
    const char *IDzrespm0_name,
    __attribute__((unused))
    const char *IDwfsref_name,
    const char *IDzrespm_name,
    const char *WFSmap_name,
    const char *DMmap_name)
{
    imageID  IDzrm;
    uint64_t sizeWFS;
    char     name[200];


    long loopnumber = 0;
    if(getenv("CACAO_LOOPNUMBER"))
    {
        loopnumber = atol(getenv("CACAO_LOOPNUMBER"));
    }


    printf(">>>>>>>>>>> loop = %ld\n", loopnumber);

    double  rms, tmpv;
    imageID IDDMmap, IDWFSmap, IDdm;

    // DECODE MAPS (IF REQUIRED)
    IDzrm = image_ID(IDzrespm0_name, dcimg, dcnimg);
    if((image_ID("RMmat", dcimg, dcnimg) != -1) &&
            (image_ID("pixindexim", dcimg, dcnimg) != -1)) // start decoding
    {
        // save_fits(IDzrespm0_name, "zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM(IDzrespm0_name,
                "RMmat",
                "pixindexim",
                IDzrespm_name);
        IDzrm = image_ID(IDzrespm_name,
            dcimg,
            dcnimg);

        if(image_ID("RMpokeC", dcimg, dcnimg) != -1)
        {
            AOloopControl_computeCalib_Hadamard_decodeRM("RMpokeC",
                    "RMmat",
                    "pixindexim",
                    "RMpokeC1");
            // save_fits("RMpokeC1", "tmp/test_RMpokeC1.fits");
        }
    }
    else // NO DECODING
    {
        copy_image_ID(IDzrespm0_name, IDzrespm_name, 0);
    }

    // create sensitivity maps

    uint32_t sizexWFS = dcimg[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = dcimg[IDzrm].md[0].size[1];
    uint32_t NBpoke   = dcimg[IDzrm].md[0].size[2];

    if(snprintf(name, sizeof(name), "aol%ld_dmC", loopnumber) < 1)
    {
        PRINT_ERROR("snprintf wrote <1 char");
    }

    IDdm             = read_sharedmem_image(name,
        dcimg,
        dcnimg);
    uint32_t sizexDM = dcimg[IDdm].md[0].size[0];
    uint32_t sizeyDM = dcimg[IDdm].md[0].size[1];

    sizeWFS = sizexWFS * sizeyWFS;

    create_2Dimage_ID(WFSmap_name, sizexWFS, sizeyWFS, &IDWFSmap);
    create_2Dimage_ID(DMmap_name, sizexDM, sizeyDM, &IDDMmap);

    printf("Preparing DM map ... ");
    fflush(stdout);
    for(uint32_t poke = 0; poke < NBpoke; poke++)
    {
        rms = 0.0;
        for(uint32_t ii = 0; ii < sizeWFS; ii++)
        {
            tmpv = dcimg[IDzrm].array.F[poke * sizeWFS + ii];
            rms += tmpv * tmpv;
        }
        dcimg[IDDMmap].array.F[poke] = rms;
    }
    printf("done\n");
    fflush(stdout);

    printf("Preparing WFS map ... ");
    fflush(stdout);
    for(uint32_t ii = 0; ii < sizeWFS; ii++)
    {
        rms = 0.0;
        for(uint32_t poke = 0; poke < NBpoke; poke++)
        {
            tmpv = dcimg[IDzrm].array.F[poke * sizeWFS + ii];
            rms += tmpv * tmpv;
        }
        dcimg[IDWFSmap].array.F[ii] = rms;
    }
    printf("done\n");
    fflush(stdout);

    /*
    IDWFSmask = image_ID("wfsmask", dcimg, dcnimg);


    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
      tot += dcimg[IDWFSref].array.F[ii]*dcimg[IDWFSmask].array.F[ii];

    totm = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
      totm += dcimg[IDWFSmask].array.F[ii];

    for(ii=0; ii<sizeWFS; ii++)
      dcimg[IDWFSref].array.F[ii] /= tot;

    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke=0;poke<NBpoke;poke++)
    {
      tot = 0.0;
      for(ii=0; ii<sizeWFS; ii++)
              tot +=
    dcimg[IDzrm].array.F[poke*sizeWFS+ii]*dcimg[IDWFSmask].array.F[ii];

      for(ii=0; ii<sizeWFS; ii++)
              dcimg[IDzrm].array.F[poke*sizeWFS+ii] -=
    tot*dcimg[IDWFSmask].array.F[ii]/totm;

      tot1 = 0.0;
      for(ii=0; ii<sizeWFS; ii++)
              tot1 +=
    dcimg[IDzrm].array.F[poke*sizeWFS+ii]*dcimg[IDWFSmask].array.F[ii];
      fprintf(fp, "%6ld %06ld %20f %20f\n", poke, NBpoke, tot, tot1);
    }
    fclose(fp);

    */

    return RETURN_SUCCESS;
}

// CANDIDATE FOR RETIREMENT
//
// median-averages multiple response matrices to create a better one
//
// if images "Hmat" AND "pixindexim" are provided, decode the image
// TEST: if "RMpokeC" exists, decode it as well
//
errno_t
AOloopControl_computeCalib_ProcessZrespM_medianfilt(
    const char *zrespm_name,
    const char *WFSref0_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    double      rmampl,
    int         normalize)
{
    long  NBmat; // number of matrices to average
    FILE *fp;
    // int r;
    char     name[200];
    char     fname[200];
    char     zrname[200];
    long     kmat;
    long     sizexWFS, sizeyWFS, sizeWFS;
    imageID *IDzresp_array;
    long     ii;
    double   fluxpos, fluxneg;
    float   *pixvalarray;
    long     k, kmin, kmax, kband;
    imageID  IDzrm;
    float    ave;
    imageID *IDWFSrefc_array;
    imageID  IDWFSref;
    // imageID IDWFSmap;
    // imageID IDDMmap;
    imageID IDWFSmask;
    // imageID IDDMmask;
    //     float lim, rms;
    //     double tmpv;
    long   NBmatlim = 3;
    long   NBpoke, poke;
    double tot, totm;


    long loopnumber = 0;
    if(getenv("CACAO_LOOPNUMBER"))
    {
        loopnumber = atol(getenv("CACAO_LOOPNUMBER"));
    }


    if(snprintf(fname, sizeof(fname), "./zresptmp/%s_nbiter.txt", zrespm_name) < 1)
    {
        PRINT_ERROR("snprintf wrote <1 char");
    }

    if((fp = fopen(fname, "r")) == NULL)
    {
        printf("ERROR: cannot open file \"%s\"\n", fname);
        exit(0);
    }
    else
    {
        if(fscanf(fp, "%50ld", &NBmat) != 1)
        {
            PRINT_ERROR("Cannot read parameter for file");
        }

        fclose(fp);
    }

    if(NBmat < NBmatlim)
    {
        printf("ERROR: insufficient number of input matrixes:\n");
        printf(" NBmat = %ld, should be at least %ld\n",
               (long) NBmat,
               (long) NBmatlim);
        exit(0);
    }
    else
    {
        printf("Processing %ld matrices\n", NBmat);
    }

    if(snprintf(name, sizeof(name), "aol%ld_dmC", loopnumber) < 1)
    {
        PRINT_ERROR("snprintf wrote <1 char");
    }

    IDzresp_array = (imageID *) malloc(sizeof(imageID) * NBmat);
    if(IDzresp_array == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    IDWFSrefc_array = (imageID *) malloc(sizeof(imageID) * NBmat);
    if(IDWFSrefc_array == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    // STEP 1: build individually cleaned RM
    for(kmat = 0; kmat < NBmat; kmat++)
    {
        if(snprintf(fname, sizeof(fname), "./zresptmp/%s_pos_%03ld.fits", zrespm_name, kmat) <
                1)
        {
            PRINT_ERROR("snprintf wrote <1 char");
        }

        imageID IDzrespfp = -1;
        load_fits(fname, "zrespfp", 2, &IDzrespfp);

        if(snprintf(fname, sizeof(fname), "./zresptmp/%s_neg_%03ld.fits", zrespm_name, kmat) <
                1)
        {
            PRINT_ERROR("snprintf wrote <1 char");
        }

        imageID IDzrespfm = -1;
        load_fits(fname, "zrespfm", 2, &IDzrespfm);

        sizexWFS = dcimg[IDzrespfp].md[0].size[0];
        sizeyWFS = dcimg[IDzrespfp].md[0].size[1];
        NBpoke   = dcimg[IDzrespfp].md[0].size[2];
        sizeWFS  = sizexWFS * sizeyWFS;

        if(snprintf(name, sizeof(name), "wfsrefc%03ld", kmat) < 1)
        {
            PRINT_ERROR("snprintf wrote <1 char");
        }

        create_3Dimage_ID(name,
                          sizexWFS,
                          sizeyWFS,
                          NBpoke,
                          &(IDWFSrefc_array[kmat]));

        if(snprintf(zrname, sizeof(zrname), "zrespm%03ld", kmat) < 1)
        {
            PRINT_ERROR("snprintf wrote <1 char");
        }

        create_3Dimage_ID(zrname,
                          sizexWFS,
                          sizeyWFS,
                          NBpoke,
                          &(IDzresp_array[kmat]));

#ifdef _OPENMP
        #pragma omp parallel for private(fluxpos, fluxneg, ii)
#endif
        for(poke = 0; poke < NBpoke; poke++)
        {
            fluxpos = 0.0;
            fluxneg = 0.0;
            for(ii = 0; ii < sizeWFS; ii++)
            {
                if(isnan(dcimg[IDzrespfp].array.F[poke * sizeWFS + ii]) !=
                        0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzrespfp,
                           poke * sizeWFS + ii);
                    dcimg[IDzrespfp].array.F[poke * sizeWFS + ii] = 0.0;
                }
                fluxpos += dcimg[IDzrespfp].array.F[poke * sizeWFS + ii];
            }

            for(ii = 0; ii < sizeWFS; ii++)
            {
                if(isnan(dcimg[IDzrespfm].array.F[poke * sizeWFS + ii]) !=
                        0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzrespfm,
                           poke * sizeWFS + ii);
                    dcimg[IDzrespfm].array.F[poke * sizeWFS + ii] = 0.0;
                }
                fluxneg += dcimg[IDzrespfm].array.F[poke * sizeWFS + ii];
            }

            for(ii = 0; ii < sizeWFS; ii++)
            {
                if(normalize == 1)
                {
                    dcimg[IDzrespfp].array.F[poke * sizeWFS + ii] /=
                        fluxpos;
                    dcimg[IDzrespfm].array.F[poke * sizeWFS + ii] /=
                        fluxneg;
                }
                dcimg[IDzresp_array[kmat]].array.F[poke * sizeWFS + ii] =
                    0.5 * (dcimg[IDzrespfp].array.F[poke * sizeWFS + ii] -
                           dcimg[IDzrespfm].array.F[poke * sizeWFS + ii]);
                dcimg[IDWFSrefc_array[kmat]].array.F[poke * sizeWFS + ii] =
                    0.5 * (dcimg[IDzrespfp].array.F[poke * sizeWFS + ii] +
                           dcimg[IDzrespfm].array.F[poke * sizeWFS + ii]);

                if(isnan(dcimg[IDzresp_array[kmat]]
                         .array.F[poke * sizeWFS + ii]) != 0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzresp_array[kmat],
                           poke * sizeWFS + ii);
                    dcimg[IDzresp_array[kmat]]
                    .array.F[poke * sizeWFS + ii] = 0.0;
                }
                if(isnan(dcimg[IDWFSrefc_array[kmat]]
                         .array.F[poke * sizeWFS + ii]) != 0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDWFSrefc_array[kmat],
                           poke * sizeWFS + ii);
                    dcimg[IDWFSrefc_array[kmat]]
                    .array.F[poke * sizeWFS + ii] = 0.0;
                }
            }
        }

        delete_image_ID("zrespfp", DELETE_IMAGE_ERRMODE_WARNING);
        delete_image_ID("zrespfm", DELETE_IMAGE_ERRMODE_WARNING);
    }

    // STEP 2: average / median each pixel
    create_3Dimage_ID(zrespm_name, sizexWFS, sizeyWFS, NBpoke, &IDzrm);
    create_2Dimage_ID(WFSref0_name, sizexWFS, sizeyWFS, &IDWFSref);

    kband = (long)(0.2 * NBmat);

    kmin = kband;
    kmax = NBmat - kband;

#ifdef _OPENMP
    #pragma omp parallel for private(ii, kmat, ave, k, pixvalarray)
#endif
    for(poke = 0; poke < NBpoke; poke++)
    {
        printf("\r act %ld / %ld        ", poke, NBpoke);
        fflush(stdout);

        if((pixvalarray = (float *) malloc(sizeof(float) * NBmat)) == NULL)
        {
            printf("ERROR: cannot allocate pixvalarray, size = %ld\n",
                   (long) NBmat);
            exit(0);
        }

        for(ii = 0; ii < sizeWFS; ii++)
        {
            for(kmat = 0; kmat < NBmat; kmat++)
            {
                pixvalarray[kmat] = dcimg[IDzresp_array[kmat]]
                                    .array.F[poke * sizeWFS + ii];
            }
            quick_sort_float(pixvalarray, kmat);
            ave = 0.0;
            for(k = kmin; k < kmax; k++)
            {
                ave += pixvalarray[k];
            }
            ave /= (kmax - kmin);
            dcimg[IDzrm].array.F[poke * sizeWFS + ii] = ave / rmampl;
        }
        free(pixvalarray);
    }

    printf("\n");

    kband = (long)(0.2 * NBmat * NBpoke);
    kmin  = kband;
    kmax  = NBmat * NBpoke - kband;

#ifdef _OPENMP
    #pragma omp parallel for private(poke, kmat, pixvalarray, ave, k)
#endif
    for(ii = 0; ii < sizeWFS; ii++)
    {
        printf("\r wfs pix %ld / %ld        ", ii, sizeWFS);
        fflush(stdout);
        if((pixvalarray = (float *) malloc(sizeof(float) * NBmat * NBpoke)) ==
                NULL)
        {
            printf("ERROR: cannot allocate pixvalarray, size = %ld x %ld\n",
                   (long) NBmat,
                   (long) NBpoke);
            exit(0);
        }

        for(poke = 0; poke < NBpoke; poke++)
            for(kmat = 0; kmat < NBmat; kmat++)
            {
                pixvalarray[kmat * NBpoke + poke] =
                    dcimg[IDWFSrefc_array[kmat]]
                    .array.F[poke * sizeWFS + ii];
            }

        quick_sort_float(pixvalarray, NBpoke * NBmat);

        ave = 0.0;
        for(k = kmin; k < kmax; k++)
        {
            ave += pixvalarray[k];
        }
        ave /= (kmax - kmin);
        dcimg[IDWFSref].array.F[ii] = ave;

        // printf("free pixvalarray : %ld x %ld\n", NBmat, NBpoke);
        // fflush(stdout);
        free(pixvalarray);
        // printf("done\n");
        // fflush(stdout);
    }
    free(IDzresp_array);
    free(IDWFSrefc_array);

    // DECODE MAPS (IF REQUIRED)

    if((image_ID("Hmat", dcimg, dcnimg) != -1) && (image_ID("pixindexim", dcimg, dcnimg) != -1))
    {
        chname_image_ID(zrespm_name, "tmprm");
        save_fits("tmprm", "zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM("tmprm",
                "Hmat",
                "pixindexim",
                zrespm_name);
        delete_image_ID("tmprm", DELETE_IMAGE_ERRMODE_WARNING);

        IDzrm = image_ID(zrespm_name, dcimg, dcnimg);

        if(image_ID("RMpokeC", dcimg, dcnimg) != -1)
        {
            AOloopControl_computeCalib_Hadamard_decodeRM("RMpokeC",
                    "Hmat",
                    "pixindexim",
                    "RMpokeC1");
            save_fits("RMpokeC1", "test_RMpokeC1.fits");
        }
    }

    NBpoke = dcimg[IDzrm].md[0].size[2];

    AOloopControl_computeCalib_mkCalib_map_mask(
        zrespm_name,
        WFSmap_name,
        DMmap_name,
        0.2,
        1.0,
        0.7,
        0.3,
        0.05,
        1.0,
        0.65,
        0.3);

    //	list_image_ID();
    // printf("========== STEP 000 ============\n");
    //	fflush(stdout);

    IDWFSmask = image_ID("wfsmask", dcimg, dcnimg);
    //	printf("ID   %ld %ld\n", IDWFSmask, IDWFSref);

    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii = 0; ii < sizeWFS; ii++)
    {
        tot += dcimg[IDWFSref].array.F[ii] *
               dcimg[IDWFSmask].array.F[ii];
    }

    totm = 0.0;
    for(ii = 0; ii < sizeWFS; ii++)
    {
        totm += dcimg[IDWFSmask].array.F[ii];
    }

    for(ii = 0; ii < sizeWFS; ii++)
    {
        dcimg[IDWFSref].array.F[ii] /= tot;
    }

    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke = 0; poke < NBpoke; poke++)
    {
        tot = 0.0;
        for(ii = 0; ii < sizeWFS; ii++)
        {
            tot += dcimg[IDzrm].array.F[poke * sizeWFS + ii] *
                   dcimg[IDWFSmask].array.F[ii];
        }

        for(ii = 0; ii < sizeWFS; ii++)
        {
            dcimg[IDzrm].array.F[poke * sizeWFS + ii] -=
                tot * dcimg[IDWFSmask].array.F[ii] / totm;
        }

        double tot1 = 0.0;
        for(ii = 0; ii < sizeWFS; ii++)
        {
            tot1 += dcimg[IDzrm].array.F[poke * sizeWFS + ii] *
                    dcimg[IDWFSmask].array.F[ii];
        }
        fprintf(fp, "%6ld %06ld %20f %20f\n", poke, NBpoke, tot, tot1);
    }
    fclose(fp);

    return RETURN_SUCCESS;
}


/**
 * ====================================================
 * V2 FPS section: mkCM (make control matrix)
 * ====================================================
 */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "mkCMsvd",
    .cmdkey      = "mkCMsvd",
    .description =
        "compute SVD control matrix from RM",
    .description_long =
        "Process a response matrix via SVD to compute the control matrix. Applies singular value filtering and modal truncation."
};

// Section 2: Local variables for FPS parameters
static int64_t mkCM_AOloopindex;
static double  mkCM_SVDlim;
static char    mkCM_fname_respM[
    FUNCTION_PARAMETER_STRMAXLEN];
static int64_t mkCM_GPUmode;
static char    mkCM_out_label[
    FUNCTION_PARAMETER_STRMAXLEN];

// Section 3: FPS_PARAMS X-macro
#define FPS_PARAMS(X) \
    X(".AOloopindex", \
      &mkCM_AOloopindex, \
      FPTYPE_INT64, \
      1, \
      FPFLAG_DEFAULT_INPUT, \
      "loop index") \
    X(".SVDlim", \
      &mkCM_SVDlim, \
      FPTYPE_FLOAT64, \
      1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT \
       | FPFLAG_MAXLIMIT), \
      "SVD limit value") \
    X(".fname_respM", \
      mkCM_fname_respM, \
      FPTYPE_FILENAME, \
      1, \
      (FPFLAG_DEFAULT_INPUT \
       | FPFLAG_FILE_RUN_REQUIRED), \
      "response matrix") \
    X(".GPUmode", \
      &mkCM_GPUmode, \
      FPTYPE_INT64, \
      1, \
      FPFLAG_DEFAULT_INPUT, \
      "Using GPU ?") \
    X(".out.label", \
      mkCM_out_label, \
      FPTYPE_STRING, \
      1, \
      FPFLAG_DEFAULT_INPUT, \
      "output label")

FPS_V2_SECTION5(FPS_PARAMS)


// Section 4: compute function (merged CONF+RUN)
static errno_t compute_function_mkCM()
{
    DEBUG_TRACE_FSTART();

    // Get output directory from FPS
    char outdirname[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(outdirname,
            functionparameter_GetParamPtr_STRING(
                milk_data.fpsptr,
                ".conf.datadir"),
            FUNCTION_PARAMETER_STRMAXLEN);
    EXECUTE_SYSTEM_COMMAND(
        "mkdir -p %s", outdirname);

    load_fits(
        mkCM_fname_respM, "respM", 1, NULL);

    char cm_name[] = "sCMat";

#ifdef HAVE_MAGMA
    if(mkCM_GPUmode)
    {
        LINALGEBRA_magma_compute_SVDpseudoInverse(
            "respM",
            cm_name,
            mkCM_SVDlim,
            100000,
            "VTmat",
            0,
            1,
            64,
            0,
            NULL);
    }
    else
    {
#endif
        linopt_compute_SVDpseudoInverse(
            "respM",
            cm_name,
            mkCM_SVDlim,
            10000,
            "VTmat",
            NULL);
#ifdef HAVE_MAGMA
    }
#endif

    {
        char ffname[STRINGMAXLEN_FULLFILENAME];

        WRITE_FULLFILENAME(ffname,
                           "%s/VTmat.fits",
                           outdirname);
        save_fits("VTmat", ffname);

        imageID ID_VTmat = image_ID(
            "VTmat",
            dcimg,
            dcnimg);
        imageID ID_DMmodes;
        uint32_t DMxsize =
            atoi(getenv("CACAO_DMxsize"));
        uint32_t DMysize =
            atoi(getenv("CACAO_DMysize"));
        uint32_t DMxysize = DMxsize * DMysize;
        create_3Dimage_ID("DMmodes",
                          DMxsize,
                          DMysize,
                          DMxysize,
                          &ID_DMmodes);
        list_image_ID();
        for(uint32_t kk = 0;
                kk < DMxysize; kk++)
        {
            for(uint32_t ii = 0;
                    ii < DMxysize; ii++)
            {
                dcimg[ID_DMmodes]
                .array.F[
                    kk * DMxysize + ii] =
                    dcimg[ID_VTmat]
                    .array.F[
                        ii * DMxysize + kk];
            }
        }
        delete_image_ID(
            "VTmat",
            DELETE_IMAGE_ERRMODE_WARNING);
        WRITE_FULLFILENAME(ffname,
                           "%s/DMmodes.fits",
                           outdirname);
        save_fits("DMmodes", ffname);

        imageID ID_WFSmodes;
        uint32_t WFSxsize;
        uint32_t WFSysize;

        imageID IDrespM = image_ID(
            "respM",
            dcimg,
            dcnimg);
        WFSxsize =
            dcimg[IDrespM]
            .md->size[0];
        WFSysize =
            dcimg[IDrespM]
            .md->size[1];

        uint32_t WFSxysize =
            WFSxsize * WFSysize;
        create_3Dimage_ID("WFSmodes",
                          WFSxsize,
                          WFSysize,
                          DMxysize,
                          &ID_WFSmodes);
        printf("Computing WFS modes ...\n");
        fflush(stdout);
        int mimax = 50;
        for(int mi = 0; mi < mimax; mi++)
        {
            printf("Mode %5d / %5d\n",
                   mi, DMxysize);
            for(uint32_t ii = 0;
                    ii < WFSxysize; ii++)
            {
                dcimg[ID_WFSmodes]
                .array.F[
                    mi * WFSxysize + ii]
                    = 0.0;
                for(uint32_t jj = 0;
                        jj < DMxysize; jj++)
                {
                    dcimg
                    [ID_WFSmodes]
                    .array.F[
                        mi * WFSxysize + ii]
                        +=
                        dcimg
                        [ID_DMmodes]
                        .array.F[
                            mi * DMxysize
                            + jj]
                        * dcimg
                        [IDrespM]
                        .array.F[
                            jj * WFSxysize
                            + ii];
                }
            }
        }
        printf(" DONE\n");
        fflush(stdout);
        WRITE_FULLFILENAME(ffname,
                           "%s/WFSmodes.fits",
                           outdirname);
        save_fits("WFSmodes", ffname);

        WRITE_FULLFILENAME(ffname,
                           "%s/sCMat00.fits",
                           outdirname);
        save_fits(cm_name, ffname);
    }

    delete_image_ID(
        cm_name,
        DELETE_IMAGE_ERRMODE_WARNING);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


// Section 7: CLI registration
#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info,
        farg,
        &CLIcmddata,
        my_bindings,
        nb_bindings,
        compute_function_mkCM);
}

errno_t
CLIADDCMD_AOloopControl_computeCalib__mkCMsvd()
{
    safe_fps_fill_farg_examples(
        farg,
        my_bindings,
        nb_bindings);

    CLIcmddata.FPS_customCONFsetup = NULL;
    CLIcmddata.FPS_customCONFcheck = NULL;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

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
    for(ii = 0; ii < xsize * ysize; ii++)
    {
        dcimg[IDout].array.F[ii] = xsize + ysize;
    }

    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
        {
            if(dcimg[IDmaskRM].array.F[jj * xsize + ii] < 0.5)
            {
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

                for(ii1 = ii1min; ii1 < ii1max + 1; ii1++)
                    for(jj1 = jj1min; jj1 < jj1max + 1; jj1++)
                        if(dcimg[IDmaskRM].array.F[jj1 * xsize + ii1] >
                                0.5)
                        {
                            dx = 1.0 * (ii - ii1);
                            dy = 1.0 * (jj - jj1);
                            r  = sqrt(dx * dx + dy * dy);
                            if(r < pixrad)
                                if(r <
                                        dcimg[IDout].array.F[jj * xsize + ii])
                                {
                                    dcimg[IDout].array.F[jj * xsize + ii] =
                                        r;
                                }
                        }
            }
        }

    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
            if(dcimg[IDout].array.F[jj * xsize + ii] >
                    (xsize + ysize) / 2)
            {
                dcimg[IDout].array.F[jj * xsize + ii] = 0.0;
            }

    return (IDout);
}
