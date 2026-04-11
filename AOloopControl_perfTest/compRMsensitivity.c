/**
 * @file    compRMsensitivity.c
 * @brief   mcompute response matrix sensitivity
 *
 *
 *
 */

#include <math.h>

#include <time.h>

#include "CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "fps.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "processinfo.h"


/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "compRMsensitivity",
    .cmdkey      = "compRMsensitivity",
    .description =
        "Compute response matrix sensitivity"
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char  *dmmodes   = NULL;
static char  *dmmask    = NULL;
static char  *wfsref    = NULL;
static char  *wfsmodes  = NULL;
static char  *wfsmask   = NULL;
static float *amplum    = NULL;
static float *lambdaum  = NULL;


/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".DMmodes", &dmmodes, \
      FPTYPE_FILENAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "DM modes") \
    X(".DMmask", &dmmask, \
      FPTYPE_FILENAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "DM mask") \
    X(".WFSref", &wfsref, \
      FPTYPE_FILENAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS reference") \
    X(".WFSmodes", &wfsmodes, \
      FPTYPE_FILENAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS modes") \
    X(".WFSmask", &wfsmask, \
      FPTYPE_FILENAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS mask") \
    X(".ampl", &amplum, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "RM modes ampl limit [um]") \
    X(".lambdaum", &lambdaum, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "wavelength [um]")


/* ================================================================
 * 4.  COMPUTATION LOGIC
 * ============================================================= */

/**
 * measure response matrix sensitivity
 */
static errno_t
AOloopControl_perfTest_computeRM_sensitivity(
    const char *IDdmmodes_name,
    const char *IDdmmask_name,
    const char *IDwfsref_name,
    const char *IDwfsresp_name,
    const char *IDwfsmask_name,
    float       amplimitum,
    float       lambdaum_val,
    const char *foutname)
{
    FILE   *fp;
    imageID IDdmmodes;
    imageID IDdmmask;
    imageID IDwfsref;
    imageID IDwfsresp;
    imageID IDwfsmask;
    double  dmmodermscnt;
    long    dmxsize, dmysize, dmxysize;
    long    NBmodes;
    long    wfsxsize, wfsysize, wfsxysize;

    long ii;

    double wfsmodermscnt;
    double tmp1;

    double wfsreftot, wfsmasktot;
    long   IDoutXP, IDoutXP_WFS;
    double XPval;

    printf("amplimit = %f um\n", amplimitum);

    IDdmmodes = image_ID(IDdmmodes_name,
        data.core.image, data.core.NB_MAX_IMAGE);
    dmxsize   = data.core.image[IDdmmodes].md[0].size[0];
    dmysize   = data.core.image[IDdmmodes].md[0].size[1];
    NBmodes   = data.core.image[IDdmmodes].md[0].size[2];
    dmxysize  = dmxsize * dmysize;

    IDdmmask = image_ID(IDdmmask_name,
        data.core.image, data.core.NB_MAX_IMAGE);

    IDwfsref  = image_ID(IDwfsref_name,
        data.core.image, data.core.NB_MAX_IMAGE);
    wfsxsize  = data.core.image[IDwfsref].md[0].size[0];
    wfsysize  = data.core.image[IDwfsref].md[0].size[1];
    wfsxysize = wfsxsize * wfsysize;

    IDwfsresp = image_ID(IDwfsresp_name,
        data.core.image, data.core.NB_MAX_IMAGE);
    IDwfsmask = image_ID(IDwfsmask_name,
        data.core.image, data.core.NB_MAX_IMAGE);

    wfsreftot = 0.0;
    for (ii = 0; ii < wfsxysize; ii++)
    {
        wfsreftot += data.core.image[IDwfsref].array.F[ii];
    }

    wfsmasktot = 0.0;
    for (ii = 0; ii < wfsxysize; ii++)
    {
        wfsmasktot +=
            data.core.image[IDwfsmask].array.F[ii];
    }

    list_image_ID();
    printf("NBmodes = %ld\n", NBmodes);
    printf("wfs size = %ld %ld\n",
           wfsxsize, wfsysize);
    printf("wfs resp ID : %ld\n", IDwfsresp);
    printf("wfs mask ID : %ld\n", IDwfsmask);
    printf("wfsmasktot = %f\n", wfsmasktot);

    fp = fopen(foutname, "w");

    fprintf(fp, "# col 1 : mode index\n");
    fprintf(fp,
        "# col 2 : avg DM value (should be 0)\n");
    fprintf(fp, "# col 3 : DM mode RMS\n");
    fprintf(fp, "# col 4 : WFS mode RMS\n");
    fprintf(fp,
        "# col 5 : SNR for 1um DM / 1 ph\n");
    fprintf(fp,
        "# col 6 : fraction of flux used\n");
    fprintf(fp,
        "# col 7 : Photon Efficiency\n");
    fprintf(fp, "\n");

    for (int mode = 0; mode < NBmodes; mode++)
    {
        double dmmoderms;
        double aveval;
        double SNR, SNR1;
        float  frac = 0.0;
        float  pcnt;
        double sigmarad;
        double eff;
        double wfsmoderms;

        dmmoderms    = 0.0;
        dmmodermscnt = 0.0;
        aveval       = 0.0;
        for (ii = 0; ii < dmxysize; ii++)
        {
            tmp1 =
                data.core.image[IDdmmodes]
                    .array.F[mode * dmxysize + ii]
                * data.core.image[IDdmmask].array.F[ii];
            aveval += tmp1;
            dmmoderms += tmp1 * tmp1;
            dmmodermscnt +=
                data.core.image[IDdmmask].array.F[ii];
        }
        dmmoderms =
            sqrt(dmmoderms / dmmodermscnt);
        aveval /= dmmodermscnt;

        SNR           = 0.0;
        wfsmoderms    = 0.0;
        wfsmodermscnt = 0.0;
        pcnt          = 0.0;
        for (ii = 0; ii < wfsxysize; ii++)
        {
            tmp1 =
                data.core.image[IDwfsresp]
                    .array.F[mode * wfsxysize + ii]
                * data.core.image[IDwfsmask].array.F[ii];
            wfsmoderms += tmp1 * tmp1;
            wfsmodermscnt = 1.0;
            wfsmodermscnt +=
                data.core.image[IDwfsmask].array.F[ii];

            if (data.core.image[IDwfsmask]
                    .array.F[ii] > 0.1)
            {
                float wv =
                    data.core.image[IDwfsresp]
                        .array.F[mode * wfsxysize
                                 + ii];
                if (data.core.image[IDwfsref]
                        .array.F[ii]
                    > fabsf(wv * amplimitum))
                {
                    SNR1 = wv
                        / sqrt(data.core.image[IDwfsref]
                                   .array.F[ii]);
                    SNR1 /= wfsreftot;
                    SNR += SNR1 * SNR1;
                    pcnt +=
                        data.core.image[IDwfsref]
                            .array.F[ii];
                }
            }
        }
        frac = pcnt / wfsreftot;

        wfsmoderms =
            sqrt(wfsmoderms / wfsmodermscnt);
        SNR = sqrt(SNR);

        sigmarad = (1.0 / SNR) * 2.0 * M_PI
                   * (2.0 / (lambdaum_val));

        eff = 1.0 / (sigmarad * sigmarad);

        fprintf(fp,
            "%5d   %16.06f   %16.06f"
            "   %16.06f    %16.06g"
            "      %12.06g"
            "      %12.010f\n",
            mode, aveval, dmmoderms,
            wfsmoderms, SNR, frac, eff);
    }

    fclose(fp);

    /* computing DM space cross-product */
    create_2Dimage_ID("DMmodesXP",
        NBmodes, NBmodes, &IDoutXP);

    for (int mode = 0; mode < NBmodes; mode++)
    {
        for (int mode1 = 0;
             mode1 < mode + 1; mode1++)
        {
            XPval = 0.0;
            for (ii = 0; ii < dmxysize; ii++)
            {
                XPval +=
                    data.core.image[IDdmmask].array.F[ii]
                    * data.core.image[IDdmmodes]
                          .array.F[mode * dmxysize
                                   + ii]
                    * data.core.image[IDdmmodes]
                          .array.F[mode1 * dmxysize
                                   + ii];
            }
            data.core.image[IDoutXP]
                .array.F[mode * NBmodes + mode1] =
                XPval / dmmodermscnt;
        }
    }
    save_fits("DMmodesXP", "DMmodesXP.fits");

    /* computing WFS space cross-product */
    create_2Dimage_ID("WFSmodesXP",
        NBmodes, NBmodes, &IDoutXP_WFS);
    for (int mode = 0; mode < NBmodes; mode++)
    {
        for (int mode1 = 0;
             mode1 < mode + 1; mode1++)
        {
            XPval = 0.0;
            for (ii = 0; ii < wfsxysize; ii++)
            {
                XPval +=
                    data.core.image[IDwfsresp]
                        .array.F[mode * wfsxysize
                                 + ii]
                    * data.core.image[IDwfsresp]
                          .array.F[mode1 * wfsxysize
                                   + ii];
            }
            data.core.image[IDoutXP_WFS]
                .array.F[mode * NBmodes + mode1] =
                XPval / wfsxysize;
        }
    }
    save_fits("WFSmodesXP", "WFSmodesXP.fits");

    return RETURN_SUCCESS;
}


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

FPS_V2_SECTION5(FPS_PARAMS)


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    AOloopControl_perfTest_computeRM_sensitivity(
        dmmodes, dmmask,
        wfsref, wfsmodes, wfsmask,
        *amplum, *lambdaum,
        "RMsens.txt");

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


/* ================================================================
 * 7.  MILK MODULE REGISTRATION
 * ============================================================= */

#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

errno_t
CLIADDCMD_AOloopControl_perfTest__compRMsensitivity()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif


/* ================================================================
 * 8.  STANDALONE ENTRY POINT
 * ============================================================= */

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
