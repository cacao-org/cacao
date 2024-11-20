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

#include "CommandLineInterface/CLIcore.h"

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
    IDzrm = image_ID(IDzrespm0_name);
    if((image_ID("RMmat") != -1) &&
            (image_ID("pixindexim") != -1)) // start decoding
    {
        // save_fits(IDzrespm0_name, "zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM(IDzrespm0_name,
                "RMmat",
                "pixindexim",
                IDzrespm_name);
        IDzrm = image_ID(IDzrespm_name);

        if(image_ID("RMpokeC") != -1)
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

    uint32_t sizexWFS = data.image[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = data.image[IDzrm].md[0].size[1];
    uint32_t NBpoke   = data.image[IDzrm].md[0].size[2];

    if(sprintf(name, "aol%ld_dmC", loopnumber) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
    }

    IDdm             = read_sharedmem_image(name);
    uint32_t sizexDM = data.image[IDdm].md[0].size[0];
    uint32_t sizeyDM = data.image[IDdm].md[0].size[1];

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
            tmpv = data.image[IDzrm].array.F[poke * sizeWFS + ii];
            rms += tmpv * tmpv;
        }
        data.image[IDDMmap].array.F[poke] = rms;
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
            tmpv = data.image[IDzrm].array.F[poke * sizeWFS + ii];
            rms += tmpv * tmpv;
        }
        data.image[IDWFSmap].array.F[ii] = rms;
    }
    printf("done\n");
    fflush(stdout);

    /*
    IDWFSmask = image_ID("wfsmask");


    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
      tot += data.image[IDWFSref].array.F[ii]*data.image[IDWFSmask].array.F[ii];

    totm = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
      totm += data.image[IDWFSmask].array.F[ii];

    for(ii=0; ii<sizeWFS; ii++)
      data.image[IDWFSref].array.F[ii] /= tot;

    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke=0;poke<NBpoke;poke++)
    {
      tot = 0.0;
      for(ii=0; ii<sizeWFS; ii++)
              tot +=
    data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];

      for(ii=0; ii<sizeWFS; ii++)
              data.image[IDzrm].array.F[poke*sizeWFS+ii] -=
    tot*data.image[IDWFSmask].array.F[ii]/totm;

      tot1 = 0.0;
      for(ii=0; ii<sizeWFS; ii++)
              tot1 +=
    data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];
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



    if(sprintf(fname, "./zresptmp/%s_nbiter.txt", zrespm_name) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
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

    if(sprintf(name, "aol%ld_dmC", loopnumber) < 1)
    {
        PRINT_ERROR("sprintf wrote <1 char");
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
        if(sprintf(fname, "./zresptmp/%s_pos_%03ld.fits", zrespm_name, kmat) <
                1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        imageID IDzrespfp = -1;
        load_fits(fname, "zrespfp", 2, &IDzrespfp);

        if(sprintf(fname, "./zresptmp/%s_neg_%03ld.fits", zrespm_name, kmat) <
                1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        imageID IDzrespfm = -1;
        load_fits(fname, "zrespfm", 2, &IDzrespfm);

        sizexWFS = data.image[IDzrespfp].md[0].size[0];
        sizeyWFS = data.image[IDzrespfp].md[0].size[1];
        NBpoke   = data.image[IDzrespfp].md[0].size[2];
        sizeWFS  = sizexWFS * sizeyWFS;

        if(sprintf(name, "wfsrefc%03ld", kmat) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
        }

        create_3Dimage_ID(name,
                          sizexWFS,
                          sizeyWFS,
                          NBpoke,
                          &(IDWFSrefc_array[kmat]));

        if(sprintf(zrname, "zrespm%03ld", kmat) < 1)
        {
            PRINT_ERROR("sprintf wrote <1 char");
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
                if(isnan(data.image[IDzrespfp].array.F[poke * sizeWFS + ii]) !=
                        0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzrespfp,
                           poke * sizeWFS + ii);
                    data.image[IDzrespfp].array.F[poke * sizeWFS + ii] = 0.0;
                }
                fluxpos += data.image[IDzrespfp].array.F[poke * sizeWFS + ii];
            }

            for(ii = 0; ii < sizeWFS; ii++)
            {
                if(isnan(data.image[IDzrespfm].array.F[poke * sizeWFS + ii]) !=
                        0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzrespfm,
                           poke * sizeWFS + ii);
                    data.image[IDzrespfm].array.F[poke * sizeWFS + ii] = 0.0;
                }
                fluxneg += data.image[IDzrespfm].array.F[poke * sizeWFS + ii];
            }

            for(ii = 0; ii < sizeWFS; ii++)
            {
                if(normalize == 1)
                {
                    data.image[IDzrespfp].array.F[poke * sizeWFS + ii] /=
                        fluxpos;
                    data.image[IDzrespfm].array.F[poke * sizeWFS + ii] /=
                        fluxneg;
                }
                data.image[IDzresp_array[kmat]].array.F[poke * sizeWFS + ii] =
                    0.5 * (data.image[IDzrespfp].array.F[poke * sizeWFS + ii] -
                           data.image[IDzrespfm].array.F[poke * sizeWFS + ii]);
                data.image[IDWFSrefc_array[kmat]].array.F[poke * sizeWFS + ii] =
                    0.5 * (data.image[IDzrespfp].array.F[poke * sizeWFS + ii] +
                           data.image[IDzrespfm].array.F[poke * sizeWFS + ii]);

                if(isnan(data.image[IDzresp_array[kmat]]
                         .array.F[poke * sizeWFS + ii]) != 0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDzresp_array[kmat],
                           poke * sizeWFS + ii);
                    data.image[IDzresp_array[kmat]]
                    .array.F[poke * sizeWFS + ii] = 0.0;
                }
                if(isnan(data.image[IDWFSrefc_array[kmat]]
                         .array.F[poke * sizeWFS + ii]) != 0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n",
                           IDWFSrefc_array[kmat],
                           poke * sizeWFS + ii);
                    data.image[IDWFSrefc_array[kmat]]
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
                pixvalarray[kmat] = data.image[IDzresp_array[kmat]]
                                    .array.F[poke * sizeWFS + ii];
            }
            quick_sort_float(pixvalarray, kmat);
            ave = 0.0;
            for(k = kmin; k < kmax; k++)
            {
                ave += pixvalarray[k];
            }
            ave /= (kmax - kmin);
            data.image[IDzrm].array.F[poke * sizeWFS + ii] = ave / rmampl;
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
                    data.image[IDWFSrefc_array[kmat]]
                    .array.F[poke * sizeWFS + ii];
            }

        quick_sort_float(pixvalarray, NBpoke * NBmat);

        ave = 0.0;
        for(k = kmin; k < kmax; k++)
        {
            ave += pixvalarray[k];
        }
        ave /= (kmax - kmin);
        data.image[IDWFSref].array.F[ii] = ave;

        // printf("free pixvalarray : %ld x %ld\n", NBmat, NBpoke);
        // fflush(stdout);
        free(pixvalarray);
        // printf("done\n");
        // fflush(stdout);
    }
    free(IDzresp_array);
    free(IDWFSrefc_array);

    // DECODE MAPS (IF REQUIRED)

    if((image_ID("Hmat") != -1) && (image_ID("pixindexim") != -1))
    {
        chname_image_ID(zrespm_name, "tmprm");
        save_fits("tmprm", "zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM("tmprm",
                "Hmat",
                "pixindexim",
                zrespm_name);
        delete_image_ID("tmprm", DELETE_IMAGE_ERRMODE_WARNING);

        IDzrm = image_ID(zrespm_name);

        if(image_ID("RMpokeC") != -1)
        {
            AOloopControl_computeCalib_Hadamard_decodeRM("RMpokeC",
                    "Hmat",
                    "pixindexim",
                    "RMpokeC1");
            save_fits("RMpokeC1", "test_RMpokeC1.fits");
        }
    }

    NBpoke = data.image[IDzrm].md[0].size[2];

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

    IDWFSmask = image_ID("wfsmask");
    //	printf("ID   %ld %ld\n", IDWFSmask, IDWFSref);

    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii = 0; ii < sizeWFS; ii++)
    {
        tot += data.image[IDWFSref].array.F[ii] *
               data.image[IDWFSmask].array.F[ii];
    }

    totm = 0.0;
    for(ii = 0; ii < sizeWFS; ii++)
    {
        totm += data.image[IDWFSmask].array.F[ii];
    }

    for(ii = 0; ii < sizeWFS; ii++)
    {
        data.image[IDWFSref].array.F[ii] /= tot;
    }

    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke = 0; poke < NBpoke; poke++)
    {
        tot = 0.0;
        for(ii = 0; ii < sizeWFS; ii++)
        {
            tot += data.image[IDzrm].array.F[poke * sizeWFS + ii] *
                   data.image[IDWFSmask].array.F[ii];
        }

        for(ii = 0; ii < sizeWFS; ii++)
        {
            data.image[IDzrm].array.F[poke * sizeWFS + ii] -=
                tot * data.image[IDWFSmask].array.F[ii] / totm;
        }

        double tot1 = 0.0;
        for(ii = 0; ii < sizeWFS; ii++)
        {
            tot1 += data.image[IDzrm].array.F[poke * sizeWFS + ii] *
                    data.image[IDWFSmask].array.F[ii];
        }
        fprintf(fp, "%6ld %06ld %20f %20f\n", poke, NBpoke, tot, tot1);
    }
    fclose(fp);

    return RETURN_SUCCESS;
}






errno_t AOloopControl_computeCalib_mkCM_FPCONF()
{
    // ===========================
    // SETUP FPS
    // ===========================
    FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);

    //FPS2PROCINFOMAP fps2procinfo;
    fps_add_processinfo_entries(&fps);

    // ===========================
    // ALLOCATE FPS ENTRIES
    // ===========================

    void    *pNull = NULL;
    uint64_t FPFLAG;

    long loop_default[4] = {0, 0, 10, 0};
    // __attribute__((unused)) long fpi_loop =
    function_parameter_add_entry(&fps,
                                 ".AOloopindex",
                                 "loop index",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &loop_default,
                                 NULL);

    double SVDlimdefault[4] = {0.001, 0.0, 1.0, 0.001};
    FPFLAG                  = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT |
                              FPFLAG_MAXLIMIT; // required to enforce the min and max limits
    //__attribute__((unused)) long fpi_SVDlim =
    function_parameter_add_entry(&fps,
                                 ".SVDlim",
                                 "SVD limit value",
                                 FPTYPE_FLOAT64,
                                 FPFLAG,
                                 &SVDlimdefault,
                                 NULL);

    // Input file name
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED;
    // __attribute__((unused)) long fpi_filename_respm        =
    function_parameter_add_entry(&fps,
                                 ".fname_respM",
                                 "response matrix",
                                 FPTYPE_FILENAME,
                                 FPFLAG,
                                 pNull,
                                 NULL);

    long GPUmode_default[4] = {0, 0, 1, 0};
    //  long fpi_GPUmode = 0;
    function_parameter_add_entry(&fps,
                                 ".GPUmode",
                                 "Using GPU ?",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &GPUmode_default,
                                 NULL);
    //    (void) fpi_GPUmode;

    // settings for output files and dir

    /*   long fpi_out_dirname      =
         function_parameter_add_entry(&fps, ".out.dirname",
                                      "output directory",
                                      FPTYPE_DIRNAME, FPFLAG_DEFAULT_INPUT,
     pNull); (void) fpi_out_dirname;*/

    //__attribute__((unused)) long fpi_out_label      =
    function_parameter_add_entry(&fps,
                                 ".out.label",
                                 "output label",
                                 FPTYPE_STRING,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);

    long fpi_out_timestring = 0;
    function_parameter_add_entry(&fps,
                                 ".out.timestring",
                                 "output timestring",
                                 FPTYPE_STRING,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 &fpi_out_timestring);

    // External scripts (post)
    /*    long fpi_exec_logdata =
          function_parameter_add_entry(&fps, ".log2fs",
                                       "log to filesystem",
                                       FPTYPE_EXECFILENAME,
     FPFLAG_DEFAULT_INPUT, pNull); (void) fpi_exec_logdata;
    */

    // =====================================
    // PARAMETER LOGIC AND UPDATE LOOP
    // =====================================
    FPS_CONFLOOP_START // macro in function_parameter.h

    FPS_CONFLOOP_END // macro in function_parameter.h

    return RETURN_SUCCESS;
}










errno_t AOloopControl_computeCalib_mkCM_RUN()
{
    FPS_CONNECT(data.FPS_name, FPSCONNECT_RUN);

    // Write time string
    /*    char timestring[100];
      mkUTtimestring_millisec_now(timestring);
      functionparameter_SetParamValue_STRING(
          &fps,
          ".out.timestring",
          timestring);
    */

    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================

    __attribute__((unused)) long loop =
        functionparameter_GetParamValue_INT64(&fps, ".AOloopindex");

    float SVDlim = functionparameter_GetParamValue_FLOAT64(&fps, ".SVDlim");

    long GPUmode = functionparameter_GetParamValue_INT64(&fps, ".GPUmode");

    char respMname[FUNCTION_PARAMETER_STRMAXLEN + 1];
    strncpy(respMname,
            functionparameter_GetParamPtr_STRING(&fps, ".fname_respM"),
            FUNCTION_PARAMETER_STRMAXLEN);

    char outdirname[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(outdirname,
            functionparameter_GetParamPtr_STRING(&fps, ".conf.datadir"),
            FUNCTION_PARAMETER_STRMAXLEN);
    EXECUTE_SYSTEM_COMMAND("mkdir -p %s", outdirname);

    load_fits(respMname, "respM", 1, NULL);

    char cm_name[] = "sCMat";

#ifdef HAVE_MAGMA
    if(GPUmode)
    {
        LINALGEBRA_magma_compute_SVDpseudoInverse("respM",
                cm_name,
                SVDlim,
                100000,
                "VTmat",
                0,
                1,
                64,
                0, // GPU device
                NULL);
    }
    else
    {
#endif // HAVE_MAGMA
        linopt_compute_SVDpseudoInverse("respM",
                                        cm_name,
                                        SVDlim,
                                        10000,
                                        "VTmat",
                                        NULL);
#ifdef HAVE_MAGMA
    }
#endif // HAVE_MAGMA

    {
        char ffname[STRINGMAXLEN_FULLFILENAME];

        WRITE_FULLFILENAME(ffname, "%s/VTmat.fits", outdirname);
        save_fits("VTmat", ffname);
        //"./mkmodestmp/VTmat.fits");

        // save as 3D cube
        imageID ID_VTmat = image_ID("VTmat");
        imageID ID_DMmodes;
        uint32_t DMxsize = atoi(getenv("CACAO_DMxsize"));
        uint32_t DMysize = atoi(getenv("CACAO_DMysize"));
        uint32_t DMxysize = DMxsize * DMysize;
        create_3Dimage_ID("DMmodes", DMxsize, DMysize, DMxysize, &ID_DMmodes);
        list_image_ID();
        for(uint32_t kk = 0; kk < DMxysize; kk++)
        {
            for(uint32_t ii = 0; ii < DMxysize; ii++)
            {
                data.image[ID_DMmodes].array.F[kk * DMxysize + ii] =
                    data.image[ID_VTmat].array.F[ii * DMxysize + kk];
            }
        }
        delete_image_ID("VTmat", DELETE_IMAGE_ERRMODE_WARNING);
        WRITE_FULLFILENAME(ffname, "%s/DMmodes.fits", outdirname);
        save_fits("DMmodes", ffname);

        imageID ID_WFSmodes;
        uint32_t WFSxsize;
        uint32_t WFSysize;

        imageID IDrespM = image_ID("respM");
        WFSxsize = data.image[IDrespM].md->size[0];
        WFSysize = data.image[IDrespM].md->size[1];

        uint32_t WFSxysize = WFSxsize * WFSysize;
        create_3Dimage_ID("WFSmodes", WFSxsize, WFSysize, DMxysize, &ID_WFSmodes);
        printf("Computing WFS modes ...\n");
        fflush(stdout);
        int mimax = 50;
        for(int mi = 0; mi < mimax; mi++)
        {
            printf("Mode %5d / %5d\n", mi, DMxysize);
            for(uint32_t ii = 0; ii < WFSxysize; ii++)
            {
                data.image[ID_WFSmodes].array.F[mi * WFSxysize + ii] = 0.0;
                for(uint32_t jj = 0; jj < DMxysize; jj++)
                {
                    data.image[ID_WFSmodes].array.F[mi * WFSxysize + ii] +=
                        data.image[ID_DMmodes].array.F[mi * DMxysize + jj]
                        * data.image[IDrespM].array.F[jj * WFSxysize + ii];
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

    functionparameter_SaveFPS2disk(&fps);

    //    functionparameter_SaveFPS2disk_dir(&fps, outdirname);
    //    EXECUTE_SYSTEM_COMMAND("rm %s/loglist.dat 2> /dev/null", outdirname);
    //    EXECUTE_SYSTEM_COMMAND("echo \"sCMat.fits\" >> %s/loglist.dat",
    //    outdirname);

    // create archive script
    //    functionparameter_write_archivescript(&fps, "../aoldatadir");

    function_parameter_RUNexit(&fps);
    delete_image_ID(cm_name, DELETE_IMAGE_ERRMODE_WARNING);

    return RETURN_SUCCESS;
}












// make control matrix
//
errno_t AOloopControl_computeCalib_mkCM(__attribute__((unused))
                                        const char *respm_name,
                                        float       SVDlim)
{
    char fpsname[200];

    long pindex = (long)
                  getpid(); // index used to differentiate multiple calls to function
    // if we don't have anything more informative, we use PID

    // int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps;

    // create FPS
    sprintf(data.FPS_name, "compsCM-%06ld", pindex);
    data.FPS_CMDCODE = FPSCMDCODE_FPSINIT;
    AOloopControl_computeCalib_mkCM_FPCONF();

    function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_SIMPLE);

    functionparameter_SetParamValue_FLOAT64(&fps, ".SVDlim", SVDlim);

    function_parameter_struct_disconnect(&fps);

    AOloopControl_computeCalib_mkCM_RUN();

    return RETURN_SUCCESS;
}

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

    IDmaskRM = image_ID(IDmaskRM_name);
    xsize    = data.image[IDmaskRM].md[0].size[0];
    ysize    = data.image[IDmaskRM].md[0].size[1];

    pixradl = (long) pixrad + 1;

    create_2Dimage_ID(IDout_name, xsize, ysize, &IDout);
    for(ii = 0; ii < xsize * ysize; ii++)
    {
        data.image[IDout].array.F[ii] = xsize + ysize;
    }

    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
        {
            if(data.image[IDmaskRM].array.F[jj * xsize + ii] < 0.5)
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
                        if(data.image[IDmaskRM].array.F[jj1 * xsize + ii1] >
                                0.5)
                        {
                            dx = 1.0 * (ii - ii1);
                            dy = 1.0 * (jj - jj1);
                            r  = sqrt(dx * dx + dy * dy);
                            if(r < pixrad)
                                if(r <
                                        data.image[IDout].array.F[jj * xsize + ii])
                                {
                                    data.image[IDout].array.F[jj * xsize + ii] =
                                        r;
                                }
                        }
            }
        }

    for(ii = 0; ii < xsize; ii++)
        for(jj = 0; jj < ysize; jj++)
            if(data.image[IDout].array.F[jj * xsize + ii] >
                    (xsize + ysize) / 2)
            {
                data.image[IDout].array.F[jj * xsize + ii] = 0.0;
            }

    return (IDout);
}
