/**
 * @file compute_control_modes.c
 * @brief Compute AO control modes in both input (WFS) and output (DM) space
 *
 */

#include "CommandLineInterface/CLIcore.h"


#include "mkmodes.h"


// Local variables pointers

static uint32_t *AOloopindex;

static float *svdlim;
static float *CPAmax;
static float *deltaCPA;

static float *alignCX; // X center
static float *alignCY; // Y center
static float *alignID; // Inner diameter
static float *alignOD; // Outer diameter

static uint32_t *DMxsize;
static uint32_t *DMysize;


static long                      fpi_FPS_zRMacqu = 0;
static FUNCTION_PARAMETER_STRUCT FPS_zRMacqu;

static long                      fpi_FPS_loRMacqu = 0;
static FUNCTION_PARAMETER_STRUCT FPS_loRMacqu;

static long                      fpi_FPS_DMcomb = 0;
static FUNCTION_PARAMETER_STRUCT FPS_DMcomb;

static char *fname_DMslaved;
static char *fname_zrespM;
static char *fname_DMmaskRM;
static char *fname_WFSmask;
static char *fname_loRM;
static char *fname_loRMmodes;

// Toggles
static int64_t *update_RMfiles;
static long     fpi_update_RMfiles;

static int64_t *update_align;
static long     fpi_update_align;


static CLICMDARGDEF farg[] = {{CLIARG_INT32,
                               ".AOloopindex",
                               "AO loop index",
                               "0",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &AOloopindex,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".svdlim",
                               "SVD limit",
                               "0.01",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &svdlim,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".CPAmax",
                               "max cycles per aperture (CPA)",
                               "20.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &CPAmax,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".deltaCPA",
                               "CPA increment",
                               "0.8",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &deltaCPA,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".align.CX",
                               "beam X center on DM",
                               "10.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &alignCX,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".align.CY",
                               "beam Y center on DM",
                               "10.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &alignCY,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".align.ID",
                               "beam inner diameter",
                               "5.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &alignID,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".align.OD",
                               "beam outer diameter",
                               "10.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &alignOD,
                               NULL},
                              {CLIARG_UINT32,
                               ".DMxsize",
                               "DM x size",
                               "32",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &DMxsize,
                               NULL},
                              {CLIARG_UINT32,
                               ".DMysize",
                               "DM y size",
                               "32",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &DMysize,
                               NULL},
                              {CLIARG_FPSNAME,
                               ".FPS_zRMacqu",
                               "FPS zonal RM acquisition",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FPSNAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &FPS_zRMacqu,
                               &fpi_FPS_zRMacqu},
                              {CLIARG_FPSNAME,
                               ".FPS_loRMacqu",
                               "FPS low order modal RM acquisition",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FPSNAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &FPS_loRMacqu,
                               &fpi_FPS_loRMacqu},
                              {CLIARG_FPSNAME,
                               ".FPS_DMcomb",
                               "FPS DM comb",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FPSNAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &FPS_DMcomb,
                               &fpi_FPS_DMcomb},
                              {CLIARG_STR,
                               ".DMslaved",
                               "DM slaved actuators",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_DMslaved,
                               NULL},
                              {CLIARG_STR,
                               ".zrespM",
                               "zonal response matrix",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_zrespM,
                               NULL},
                              {CLIARG_STR,
                               ".DMmaskRM",
                               "actuators directly controlled",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_DMmaskRM,
                               NULL},
                              {CLIARG_STR,
                               ".WFSmask",
                               "WFS mask",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_WFSmask,
                               NULL},
                              {CLIARG_STR,
                               ".loRM",
                               "low order modal response matrix",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_loRM,
                               NULL},
                              {CLIARG_STR,
                               ".loRMmodes",
                               "low order RM modes",
                               "NULL",
                               CLICMDARG_FLAG_NOCLI,
                               FPTYPE_FITSFILENAME,
                               FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
                               (void **) &fname_loRMmodes,
                               NULL},
                              {CLIARG_ONOFF,
                               ".upRMfiles",
                               "update RM files",
                               "OFF",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &update_RMfiles,
                               &fpi_update_RMfiles},
                              {CLIARG_ONOFF,
                               ".upAlign",
                               "update default align (if no DMmaskRM)",
                               "OFF",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &update_align,
                               &fpi_update_align}

};



static errno_t customCONFsetup()
{
    return RETURN_SUCCESS;
}




static errno_t customCONFcheck()
{
    if (data.fpsptr != NULL)
    {

        if (FPS_zRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_zRMacqu,
                                                 &FPS_zRMacqu);
        }

        if (FPS_loRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_loRMacqu,
                                                 &FPS_loRMacqu);
        }

        if (FPS_DMcomb.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_DMcomb,
                                                 &FPS_DMcomb);
        }


        // Update RM files
        if (data.fpsptr->parray[fpi_update_RMfiles].fpflag & FPFLAG_ONOFF)
        {

            if (FPS_zRMacqu.SMfd > 0)
            {
                char datadir[FUNCTION_PARAMETER_STRMAXLEN];
                char fname[FUNCTION_PARAMETER_STRMAXLEN];

                strncpy(datadir,
                        functionparameter_GetParamPtr_STRING(&FPS_zRMacqu,
                                                             ".conf.datadir"),
                        FUNCTION_PARAMETER_STRMAXLEN);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/dmslaved.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".DMslaved",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/zrespM_mn.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".zrespM",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/dmmask_mksl.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".DMmaskRM",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/wfsmask_mkm.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".WFSmask",
                                                       fname);
            }

            if (FPS_loRMacqu.SMfd > 0)
            {
                char datadir[FUNCTION_PARAMETER_STRMAXLEN];
                char fname[FUNCTION_PARAMETER_STRMAXLEN];

                strncpy(datadir,
                        functionparameter_GetParamPtr_STRING(&FPS_loRMacqu,
                                                             ".conf.datadir"),
                        FUNCTION_PARAMETER_STRMAXLEN);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/respM.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".loRM",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/RMpokeCube.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".loRMmodes",
                                                       fname);
            }

            // set back to OFF
            data.fpsptr->parray[fpi_update_RMfiles].fpflag &= ~FPFLAG_ONOFF;
        }


        // update align params for auto mask
        if (data.fpsptr->parray[fpi_update_align].fpflag & FPFLAG_ONOFF)
        {
            if (FPS_DMcomb.SMfd > 0)
            {
                int DMxsize = functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                                                    ".DMxsize");
                int DMysize = functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                                                    ".DMysize");
                __attribute__((unused)) int DMMODE =
                    functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                                          ".DMMODE");

                float cx = 0.5 * DMxsize - 0.5;
                float cy = 0.5 * DMysize - 0.5;
                float od = 0.45 * DMxsize;
                float id = 0.05 * DMxsize;

                functionparameter_SetParamValue_INT64(data.fpsptr,
                                                      ".DMxsize",
                                                      DMxsize);
                functionparameter_SetParamValue_INT64(data.fpsptr,
                                                      ".DMysize",
                                                      DMysize);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".align.CX",
                                                        cx);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".align.CY",
                                                        cy);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".align.OD",
                                                        od);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".align.ID",
                                                        id);
            }
            data.fpsptr->parray[fpi_update_align].fpflag &= ~FPFLAG_ONOFF;
        }
    }

    return RETURN_SUCCESS;
}




static CLICMDDATA CLIcmddata = {"compctrlmodes",
                                "compute AO control modes in WFS and DM space",
                                CLICMD_FIELDS_DEFAULTS};


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();



    /*IMGID inimg = makeIMGID(inimname);
    resolveIMGID(&inimg, ERRMODE_ABORT);

    IMGID outimg = makeIMGID(outimname);
    resolveIMGID(&outimg, ERRMODE_ABORT);
    */
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // custom initialization
    if (CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
    {
        // procinfo is accessible here
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART


    // MaskMode = 0  : tapered masking
    // MaskMode = 1  : STRICT masking
    //
    // if BlockNB < 0 : do all blocks
    // if BlockNB >= 0 : only update single block (untested)
    int MaskMode = 0;
    int BlockNB  = -1;

    AOloopControl_computeCalib_mkModes("fmodes",
                                       *DMxsize,
                                       *DMysize,
                                       *CPAmax,
                                       *deltaCPA,
                                       *alignCX,
                                       *alignCY,
                                       *alignID,
                                       *alignOD,
                                       MaskMode,
                                       BlockNB,
                                       *svdlim,
                                       data.fpsptr->md->datadir);

    // streamprocess(inimg, outimg);
    // processinfo_update_output_stream(processinfo, outimg.ID);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions




    // Register function in CLI
    errno_t
    CLIADDCMD_cacao_computeCalib__compute_control_modes()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
