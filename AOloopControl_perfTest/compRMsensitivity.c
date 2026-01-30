/**
 * @file    compRMsensitivity.c
 * @brief   mcompute response matrix sensitivity
 *
 *
 *
 */

#include <math.h>

#include <time.h>

#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "fps.h"
#include "processinfo.h"

#define COMPRMSENSITIVITY_HELPTEXT "Compute response matrix sensitivity"

// Changed ptr_addr to pass identifier (e.g. dmmodes) instead of address (&dmmodes)
#define COMPRMSENSITIVITY_PARAMS(X) \
    X(CLIARG_IMG, FPTYPE_FILENAME, char*, ".DMmodes", "DM modes", "dmmodes", NULL, dmmodes, functionparameter_GetParamPtr_STRING, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_IMG, FPTYPE_FILENAME, char*, ".DMmask", "DM mask", "dmmask", NULL, dmmask, functionparameter_GetParamPtr_STRING, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_IMG, FPTYPE_FILENAME, char*, ".WFSref", "WFS reference", "wfsref", NULL, wfsref, functionparameter_GetParamPtr_STRING, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_IMG, FPTYPE_FILENAME, char*, ".WFSmodes", "WFS modes", "wfsmodes", NULL, wfsmodes, functionparameter_GetParamPtr_STRING, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_IMG, FPTYPE_FILENAME, char*, ".WFSmask", "WFS mask", "wfsmask", NULL, wfsmask, functionparameter_GetParamPtr_STRING, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_FLOAT32, FPTYPE_FLOAT32, float, ".ampl", "RM modes ampl limit [um]", "1.0", 1.0, amplum, functionparameter_GetParamPtr_FLOAT32, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_FLOAT32, FPTYPE_FLOAT32, float, ".lambdaum", "wavelength [um]", "0.8", 0.8, lambdaum, functionparameter_GetParamPtr_FLOAT32, FPFLAG_DEFAULT_INPUT)


// Local variables pointers
static char *dmmodes;
long         fpi_dmmodes;

static char *dmmask;
long         fpi_dmmask;

static char *wfsref;
long         fpi_wfsref;

static char *wfsmodes;
long         fpi_wfsmodes;

static char *wfsmask;
long         fpi_wfsmask;

static float *amplum;
long          fpi_amplum;

static float *lambdaum;
long          fpi_lambdaum;


//
// measure response matrix sensitivity
//
static errno_t
AOloopControl_perfTest_computeRM_sensitivity(const char *IDdmmodes_name,
        const char *IDdmmask_name,
        const char *IDwfsref_name,
        const char *IDwfsresp_name,
        const char *IDwfsmask_name,
        float       amplimitum,
        float       lambdaum,
        const char *foutname);


/* ================================================================== */
/* STANDALONE IMPLEMENTATION                                          */

int FPSINIT_compRMsensitivity(const char *fps_name, const char *keywords, const char *description) {
    FUNCTION_PARAMETER_STRUCT fps;
    FPS_INIT_STD_PREAMBLE(fps, fps_name, keywords, description, COMPRMSENSITIVITY_HELPTEXT);
    FPS_INIT_PROCINFO_DEFAULTS(fps, "stream", 10); 

#define X_FPS_INIT(cli_type, fps_type, c_type, key, descr, def_str, def_val, ptr_addr, val_expr, cli_flags) \
    { c_type val = def_val; function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val_expr, NULL); }
    COMPRMSENSITIVITY_PARAMS(X_FPS_INIT)
#undef X_FPS_INIT

    fps_add_processinfo_entries(&fps);
    function_parameter_FPCONFexit(&fps);
    return 0;
}

#define X_FPS_CONF(cli_type, fps_type, c_type, key, descr, def_str, def_val, ptr_addr, val_expr, cli_flags) \
    ptr_addr = val_expr(&fps, key);

int FPSCONF_compRMsensitivity(const char *fps_name, int loop) {
    FPS_CONF_STD_BODY(fps_name, loop,
        {
            COMPRMSENSITIVITY_PARAMS(X_FPS_CONF)
        },
        {
            // Validation logic
        }
    );
    return 0;
}
#undef X_FPS_CONF

FPS_MAKE_STANDALONE_CONFSTOP(compRMsensitivity)
FPS_MAKE_STANDALONE_RUNSTOP(compRMsensitivity)

#define X_FPS_RUN(cli_type, fps_type, c_type, key, descr, def_str, def_val, ptr_addr, val_expr, cli_flags) \
    ptr_addr = val_expr(&fps, key);

int FPSRUN_compRMsensitivity(const char *fps_name) {
    FUNCTION_PARAMETER_STRUCT fps;
    
    FPS_RUN_STD_PREAMBLE(fps_name, fps, {
        COMPRMSENSITIVITY_PARAMS(X_FPS_RUN)
    });

    AOloopControl_perfTest_computeRM_sensitivity(dmmodes,
            dmmask,
            wfsref,
            wfsmodes,
            wfsmask,
            *amplum,
            *lambdaum,
            "RMsens.txt");

    function_parameter_struct_disconnect(&fps);
    return 0;
}
#undef X_FPS_RUN

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE("compRMsensitivity", compRMsensitivity, COMPRMSENSITIVITY_HELPTEXT, COMPRMSENSITIVITY_PARAMS)
#endif


#ifndef FPS_STANDALONE
// Adjusted X_CLI_DEF to align with CLICMDARGDEF structure
#define X_CLI_DEF(cli_type, fps_type, c_type, key, descr, def_str, def_val, ptr_addr, val_expr, cli_flags) \
    { cli_type, key, descr, def_str, CLICMDARG_FLAG_DEFAULT, (uint64_t)fps_type, (uint64_t)cli_flags, (void **) &ptr_addr, NULL },

static CLICMDARGDEF farg[] = {
    COMPRMSENSITIVITY_PARAMS(X_CLI_DEF)
};
#undef X_CLI_DEF


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "compRMsens", "compute RM sensitivity", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}
#endif


//
// measure response matrix sensitivity
//
static errno_t
AOloopControl_perfTest_computeRM_sensitivity(const char *IDdmmodes_name,
        const char *IDdmmask_name,
        const char *IDwfsref_name,
        const char *IDwfsresp_name,
        const char *IDwfsmask_name,
        float       amplimitum,
        float       lambdaum,
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

    IDdmmodes = image_ID(IDdmmodes_name);
    dmxsize   = data.image[IDdmmodes].md[0].size[0];
    dmysize   = data.image[IDdmmodes].md[0].size[1];
    NBmodes   = data.image[IDdmmodes].md[0].size[2];
    dmxysize  = dmxsize * dmysize;

    IDdmmask = image_ID(IDdmmask_name);

    IDwfsref  = image_ID(IDwfsref_name);
    wfsxsize  = data.image[IDwfsref].md[0].size[0];
    wfsysize  = data.image[IDwfsref].md[0].size[1];
    wfsxysize = wfsxsize * wfsysize;

    IDwfsresp = image_ID(IDwfsresp_name);
    IDwfsmask = image_ID(IDwfsmask_name);

    wfsreftot = 0.0;
    for(ii = 0; ii < wfsxysize; ii++)
    {
        wfsreftot += data.image[IDwfsref].array.F[ii];
    }

    wfsmasktot = 0.0;
    for(ii = 0; ii < wfsxysize; ii++)
    {
        wfsmasktot += data.image[IDwfsmask].array.F[ii];
    }

    list_image_ID();
    printf("NBmodes = %ld\n", NBmodes);
    printf("wfs size = %ld %ld\n", wfsxsize, wfsysize);
    printf("wfs resp ID : %ld\n", IDwfsresp);
    printf("wfs mask ID : %ld\n", IDwfsmask);

    printf("wfsmasktot = %f\n", wfsmasktot);

    fp = fopen(foutname, "w");

    fprintf(fp, "# col 1 : mode index\n");
    fprintf(fp, "# col 2 : average DM value (should be zero)\n");
    fprintf(fp, "# col 3 : DM mode RMS\n");
    fprintf(fp, "# col 4 : WFS mode RMS\n");
    fprintf(fp, "# col 5 : SNR for a 1um DM motion with 1 ph\n");
    fprintf(fp, "# col 6 : fraction of flux used in measurement\n");
    fprintf(fp, "# col 7 : Photon Efficiency\n");
    fprintf(fp, "\n");

    for(int mode = 0; mode < NBmodes; mode++)
    {
        double dmmoderms;
        double aveval;
        double SNR, SNR1; // single pixel SNR
        float  frac = 0.0;
        float  pcnt;
        double sigmarad;
        double eff; // efficiency
        double wfsmoderms;

        dmmoderms    = 0.0;
        dmmodermscnt = 0.0;
        aveval       = 0.0;
        for(ii = 0; ii < dmxysize; ii++)
        {
            tmp1 = data.image[IDdmmodes].array.F[mode * dmxysize + ii] *
                   data.image[IDdmmask].array.F[ii];
            aveval += tmp1;
            dmmoderms += tmp1 * tmp1;
            dmmodermscnt += data.image[IDdmmask].array.F[ii];
        }
        dmmoderms = sqrt(dmmoderms / dmmodermscnt);
        aveval /= dmmodermscnt;

        SNR           = 0.0;
        wfsmoderms    = 0.0;
        wfsmodermscnt = 0.0;
        pcnt          = 0.0;
        for(ii = 0; ii < wfsxysize; ii++)
        {
            tmp1 = data.image[IDwfsresp].array.F[mode * wfsxysize + ii] *
                   data.image[IDwfsmask].array.F[ii];
            wfsmoderms += tmp1 * tmp1;
            wfsmodermscnt = 1.0;
            wfsmodermscnt += data.image[IDwfsmask].array.F[ii];

            if(data.image[IDwfsmask].array.F[ii] > 0.1)
            {
                if(data.image[IDwfsref].array.F[ii] >
                        fabs(data.image[IDwfsresp].array.F[mode * wfsxysize + ii] *
                             amplimitum))
                {
                    SNR1 =
                        data.image[IDwfsresp].array.F[mode * wfsxysize + ii] /
                        sqrt(data.image[IDwfsref].array.F[ii]);
                    SNR1 /= wfsreftot;
                    SNR += SNR1 * SNR1;
                    pcnt += data.image[IDwfsref].array.F[ii];
                }
            }
        }
        frac = pcnt / wfsreftot;

        wfsmoderms = sqrt(wfsmoderms / wfsmodermscnt);
        SNR        = sqrt(SNR); // SNR for 1 ph, 1um DM actuation
        // -> sigma for 1ph = 1/SNR [DMum]

        // 1umDM act = 2.0*M_PI * ( 2.0 / (lambdanm*0.001) ) rad WF
        // -> sigma for 1ph = (1/SNR) * 2.0*M_PI * ( 2.0 / (lambdanm*0.001) ) rad
        // WF
        sigmarad = (1.0 / SNR) * 2.0 * M_PI * (2.0 / (lambdaum));

        // SNR is in DMum per sqrt(Nph)
        // factor 2.0 for DM reflection

        eff = 1.0 / (sigmarad * sigmarad);

        fprintf(fp,
                "%5d   %16.06f   %16.06f   %16.06f    %16.06g      %12.06g     "
                "   %12.010f\n",
                mode,
                aveval,
                dmmoderms,
                wfsmoderms,
                SNR,
                frac,
                eff);
    }

    fclose(fp);

    // computing DM space cross-product
    create_2Dimage_ID("DMmodesXP", NBmodes, NBmodes, &IDoutXP);

    for(int mode = 0; mode < NBmodes; mode++)
        for(int mode1 = 0; mode1 < mode + 1; mode1++)
        {
            XPval = 0.0;
            for(ii = 0; ii < dmxysize; ii++)
            {
                XPval += data.image[IDdmmask].array.F[ii] *
                         data.image[IDdmmodes].array.F[mode * dmxysize + ii] *
                         data.image[IDdmmodes].array.F[mode1 * dmxysize + ii];
            }

            data.image[IDoutXP].array.F[mode * NBmodes + mode1] =
                XPval / dmmodermscnt;
        }
    save_fits("DMmodesXP", "DMmodesXP.fits");

    // computing WFS space cross-product
    create_2Dimage_ID("WFSmodesXP", NBmodes, NBmodes, &IDoutXP_WFS);
    for(int mode = 0; mode < NBmodes; mode++)
        for(int mode1 = 0; mode1 < mode + 1; mode1++)
        {
            XPval = 0.0;
            for(ii = 0; ii < wfsxysize; ii++)
            {
                XPval += data.image[IDwfsresp].array.F[mode * wfsxysize + ii] *
                         data.image[IDwfsresp].array.F[mode1 * wfsxysize + ii];
            }

            data.image[IDoutXP_WFS].array.F[mode * NBmodes + mode1] =
                XPval / wfsxysize;
        }
    save_fits("WFSmodesXP", "WFSmodesXP.fits");

    return RETURN_SUCCESS;
}



#ifndef FPS_STANDALONE
static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    AOloopControl_perfTest_computeRM_sensitivity(dmmodes,
            dmmask,
            wfsref,
            wfsmodes,
            wfsmask,
            *amplum,
            *lambdaum,
            "RMsens.txt");

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_perfTest__compRMsensitivity()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif
