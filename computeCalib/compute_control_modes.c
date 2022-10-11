/**
 * @file compute_control_modes.c
 * @brief Compute AO control modes in both input (WFS) and output (DM) space
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "linopt_imtools/linopt_imtools.h"



#include "mkmodes.h"
#include "modes_spatial_extrapolate.h"





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


static char *fname_DMmaskCTRL;
static long  fpi_fname_DMmaskCTRL;
static char *fname_DMmaskEXTR;
static long  fpi_fname_DMmaskEXTR;

static char *fname_zrespM;
static long fpi_fname_zrespM;
static char *fname_WFSmask;
static long fpi_fname_WFSmask;
static char *fname_loRM;
static long fpi_fname_loRM;
static char *fname_loRMmodes;
static long fpi_fname_loRMmodes;

// Toggles
static int64_t *update_RMfiles;
static long     fpi_update_RMfiles;

static int64_t *update_align;
static long     fpi_update_align;



static CLICMDARGDEF farg[] =
{
    {
        CLIARG_INT32,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".svdlim",
        "SVD limit",
        "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &svdlim,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".CPAmax",
        "max cycles per aperture (CPA)",
        "20.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &CPAmax,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".deltaCPA",
        "CPA increment",
        "0.8",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &deltaCPA,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".DMgeom.align.CX",
        "beam X center on DM",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &alignCX,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".DMgeom.align.CY",
        "beam Y center on DM",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &alignCY,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".DMgeom.align.ID",
        "beam inner diameter",
        "5.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &alignID,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".DMgeom.align.OD",
        "beam outer diameter",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &alignOD,
        NULL
    },
    {
        CLIARG_UINT32,
        ".DMgeom.DMxsize",
        "DM x size",
        "32",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMxsize,
        NULL
    },
    {
        CLIARG_UINT32,
        ".DMgeom.DMysize",
        "DM y size",
        "32",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMysize,
        NULL
    },
    {
        CLIARG_FPSNAME,
        ".FPS_zRMacqu",
        "FPS zonal RM acquisition",
        "NULL",
        CLICMDARG_FLAG_NOCLI,
        FPTYPE_FPSNAME,
        FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
        (void **) &FPS_zRMacqu,
        &fpi_FPS_zRMacqu
    },
    {
        CLIARG_FPSNAME,
        ".DMgeom.FPS_DMcomb",
        "FPS DM comb",
        "NULL",
        CLICMDARG_FLAG_NOCLI,
        FPTYPE_FPSNAME,
        FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED,
        (void **) &FPS_DMcomb,
        &fpi_FPS_DMcomb
    },
    {
        CLIARG_FITSFILENAME,
        ".DMgeom.DMmaskCTRL",
        "DM actuators controlled",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_DMmaskCTRL,
        &fpi_fname_DMmaskCTRL
    },
    {
        CLIARG_FITSFILENAME,
        ".DMgeom.DMmaskEXTR",
        "DM actuators extrapolated",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_DMmaskEXTR,
        &fpi_fname_DMmaskEXTR
    },
    {
        CLIARG_STR,
        ".zrespM",
        "zonal response matrix",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_zrespM,
        &fpi_fname_zrespM
    },
    {
        CLIARG_STR,
        ".WFSmask",
        "WFS mask",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_WFSmask,
        &fpi_fname_WFSmask
    },
// aux low-order RM
    {
        CLIARG_FPSNAME,
        ".auxRM.FPS_loRMacqu",
        "FPS low order modal RM acquisition",
        "NULL",
        CLICMDARG_FLAG_NOCLI,
        FPTYPE_FPSNAME,
        FPFLAG_DEFAULT_INPUT,
        (void **) &FPS_loRMacqu,
        &fpi_FPS_loRMacqu
    },
    {
        CLIARG_STR,
        ".auxRM.loRM",
        "low order modal response matrix",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_loRM,
        &fpi_fname_loRM
    },
    {
        CLIARG_FITSFILENAME,
        ".auxRM.loRMmodes",
        "low order RM modes",
        "NULL",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fname_loRMmodes,
        &fpi_fname_loRMmodes
    },
    {
        CLIARG_ONOFF,
        ".upRMfiles",
        "update RM files from FPSs",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &update_RMfiles,
        &fpi_update_RMfiles
    },
    {
        CLIARG_ONOFF,
        ".DMgeom.upAlign",
        "update default align (if no DMmaskRM)",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &update_align,
        &fpi_update_align
    }
};


static CLICMDDATA CLIcmddata = {"compctrlmodes",
                                "compute AO control modes in WFS and DM space",
                                CLICMD_FIELDS_DEFAULTS
                               };



static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        // FPS are not required

        data.fpsptr->parray[fpi_fname_zrespM].fpflag &=
            ~FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_fname_WFSmask].fpflag &=
            ~FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_FPS_loRMacqu].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

        data.fpsptr->parray[fpi_FPS_zRMacqu].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

        data.fpsptr->parray[fpi_FPS_DMcomb].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

    }

    return RETURN_SUCCESS;
}





static errno_t customCONFcheck()
{
    if(data.fpsptr != NULL)
    {

        data.fpsptr->parray[fpi_fname_zrespM].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_fname_WFSmask].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;



        if(FPS_zRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_zRMacqu,
                                                 &FPS_zRMacqu);
        }

        if(FPS_loRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_loRMacqu,
                                                 &FPS_loRMacqu);
        }

        if(FPS_DMcomb.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_DMcomb,
                                                 &FPS_DMcomb);
        }


        // Update RM files
        if(data.fpsptr->parray[fpi_update_RMfiles].fpflag & FPFLAG_ONOFF)
        {

            if(FPS_zRMacqu.SMfd > 0)
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
                                                       ".DMgeom.DMmaskEXTR",
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
                                                       ".DMgeom.DMmaskCTRL",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/wfsmask_mkm.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.fpsptr,
                                                       ".WFSmask",
                                                       fname);
            }

            if(FPS_loRMacqu.SMfd > 0)
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
        if(data.fpsptr->parray[fpi_update_align].fpflag & FPFLAG_ONOFF)
        {
            if(FPS_DMcomb.SMfd > 0)
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
                                                      ".DMgeom.DMxsize",
                                                      DMxsize);
                functionparameter_SetParamValue_INT64(data.fpsptr,
                                                      ".DMgeom.DMysize",
                                                      DMysize);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".DMgeom.align.CX",
                                                        cx);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".DMgeom.align.CY",
                                                        cy);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".DMgeom.align.OD",
                                                        od);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".DMgeom.align.ID",
                                                        id);
            }
            data.fpsptr->parray[fpi_update_align].fpflag &= ~FPFLAG_ONOFF;
        }


        data.fpsptr->parray[fpi_fname_DMmaskCTRL].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
        data.fpsptr->parray[fpi_fname_DMmaskEXTR].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}







// detailed help
static errno_t help_function()
{
    printf(
        "Compute AO control modes\n"
        "Corresponding pairs of DM and WFS modes are computed\n"
    );

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
    if(CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
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

    // full set of DM actuators to be controlled
    //
    load_fits(fname_DMmaskCTRL, "DMmaskCTRL", LOADFITS_ERRMODE_ERROR, NULL);
    IMGID imgDMmaskCTRL = mkIMGID_from_name("DMmaskCTRL");
    resolveIMGID(&imgDMmaskCTRL, ERRMODE_ABORT);

    // DM actuators to be extrapolated from neighbors
    // this is a subset of DMmaskCTRL
    //
    load_fits(fname_DMmaskEXTR, "DMmaskEXTR", LOADFITS_ERRMODE_ERROR, NULL);
    IMGID imgDMmaskEXTR = mkIMGID_from_name("DMmaskEXTR");
    resolveIMGID(&imgDMmaskEXTR, ERRMODE_ABORT);



    // CREATE ZERNIKE+FOURIER DM MODES BASIS
    // Zernike modes are centered on (alignCX, alignCY)
    // output : imgDMmodesZF ("DMmodesZFm")
    //
    IMGID imgDMmodesZF = mkIMGID_from_name("DMmodesZFm");
    mk_ZernikeFourier_modal_basis(*DMxsize,
                                  *DMysize,
                                  *CPAmax,
                                  *deltaCPA,
                                  *alignCX,
                                  *alignCY,
                                  *alignID,
                                  *alignOD,
                                  &imgDMmodesZF);

    // Adjust mode amplitude to have RMS=1 over DMmaskCTRL
    // in-place computation
    //
    modes_mask_normalize(imgDMmodesZF, imgDMmaskCTRL);

    // save to disk
    fps_write_RUNoutput_image(data.fpsptr, "DMmodesZFm", "DMmodesZFm");


    // EXTRAPOLATE DM MODES
    // output : imgDMmodesZFe
    //
    IMGID imgDMmodesZFe = mkIMGID_from_name("DMmodesZFe");
    IMGID imgcpa      = mkIMGID_from_name("modesfreqcpa");

    modes_spatial_extrapolate(
        imgDMmodesZF,
        imgDMmaskCTRL,
        imgcpa,
        &imgDMmodesZFe
    );

    // save to disk
    fps_write_RUNoutput_image(data.fpsptr, "DMmodesZFe", "DMmodesZFe");




    // TAG LINE 889




    // set pixels to zero if neither part of DMmaskCTRL or DMmaskEXT
    // output (in-place) : imgDMmodesZFe

    uint32_t msizex = imgDMmaskCTRL.size[0];
    uint32_t msizey = imgDMmaskCTRL.size[1];
    uint64_t msizexy = (uint64_t) msizex;
    msizexy *= msizey;
    printf("size %ld %ld %ld\n", (long) msizex, (long) msizey,
           (long) imgDMmodesZFe.size[2]);
    for(uint32_t kk = 0; kk < imgDMmodesZFe.size[2]; kk++)
    {
        for(uint64_t ii = 0; ii < msizexy; ii++)
        {
            float mval = 1.0 - (imgDMmaskCTRL.im->array.F[ii] - 1.0) *
                         (imgDMmaskEXTR.im->array.F[ii] - 1.0);
            imgDMmodesZFe.im->array.F[kk * msizexy + ii] *= mval;
        }
    }
    // save to disk
    fps_write_RUNoutput_image(data.fpsptr, "DMmodesZFe", "DMmodesZFem");



    // TAG LINE 934

    // TODO Remove modes

    // TAG LINE 1040





    // zonal response matrix
    load_fits(fname_zrespM, "zrespM", LOADFITS_ERRMODE_ERROR, NULL);
    IMGID imgzrespM = mkIMGID_from_name("zrespM");
    resolveIMGID(&imgzrespM, ERRMODE_ABORT);


    // COMPUTE WFS RESPONSE TO MODES
    // output : imgWFSmodesZFe
    //
    uint32_t wfssizex = imgzrespM.size[0];
    uint32_t wfssizey = imgzrespM.size[1];
    uint64_t wfssizexy = wfssizex;
    wfssizexy *= wfssizey;
    IMGID imgWFSmodesZFe = makeIMGID_3D("WFSmodesZFe", wfssizex, wfssizey,
                                        imgDMmodesZFe.size[2]);
    createimagefromIMGID(&imgWFSmodesZFe);


    {
        uint64_t act;
        uint64_t act1, act2;
        uint32_t m;
        uint64_t wfselem;
        uint64_t m1;
#ifdef _OPENMP
        #pragma omp parallel for private(m, m1, act, act1, act2, wfselem)
#endif

        for(m = 0; m < imgDMmodesZFe.size[2]; m++)
        {
            m1 = m * wfssizexy;

            printf("\r %5u / %5u   ", m, imgDMmodesZFe.size[2]);
            fflush(stdout);
            for(act = 0; act < msizexy; act++)
            {
                act1 = m * msizexy + act;
                act2 = act * wfssizexy;
                for(wfselem = 0; wfselem < wfssizexy; wfselem++)
                {
                    imgWFSmodesZFe.im->array.F[m1 + wfselem] +=
                        imgDMmodesZFe.im->array.F[act1] *
                        imgzrespM.im->array.F[act2 + wfselem];
                }
            }
        }
    }

    // save to disk
    fps_write_RUNoutput_image(data.fpsptr, "WFSmodesZFe", "WFSmodesZFe");




    // TAG LINE 1102



    // APPLY MODAL RESPONSE MATRIX IF IT EXISTS
    // output (in-place) : imgWFSmodesZFe
    //
    imageID IDloRM = -1;
    load_fits(fname_loRM, "loRM", LOADFITS_ERRMODE_WARNING, &IDloRM);
    imageID IDloDMmodes = -1;
    load_fits(fname_loRMmodes, "loDMmodes", LOADFITS_ERRMODE_WARNING, &IDloDMmodes);

    if((IDloRM != -1) && (IDloDMmodes != -1))
    {
        FILE   *fpLOcoeff;
        {
            char ffname[STRINGMAXLEN_FULLFILENAME];
            WRITE_FULLFILENAME(ffname, "./%s/LOcoeff.txt", data.fpsptr->md->datadir);
            fpLOcoeff = fopen(ffname, "w");
            if(fpLOcoeff == NULL)
            {
                printf("ERROR: cannot create file \"LOcoeff.txt\"\n");
                exit(0);
            }
        }



        IMGID imgloRM = mkIMGID_from_name("loRM");
        resolveIMGID(&imgloRM, ERRMODE_ABORT);

        IMGID imgloDMmodes = mkIMGID_from_name("loDMmodes");
        resolveIMGID(&imgloDMmodes, ERRMODE_ABORT);


        printf("Using low-order modal response [%ld %ld]\n",
               IDloRM,
               IDloDMmodes);

        uint32_t linfitsize = imgloDMmodes.size[2];

        imageID IDRMM_coeff = -1;
        create_2Dimage_ID("linfitcoeff", linfitsize, 1, &IDRMM_coeff);

        imageID ID_imfit = -1;
        create_2Dimage_ID("imfitim", msizex, msizey, &ID_imfit);

        imageID IDcoeffmat = -1;
        create_2Dimage_ID("imfitmat",
                          linfitsize,
                          imgWFSmodesZFe.size[2],
                          &IDcoeffmat);

        int linfitreuse = 0;

        imageID IDwfstmp = -1;
        create_2Dimage_ID("wfsimtmp", wfssizex, wfssizey, &IDwfstmp);

        for(uint32_t m = 0; m < imgDMmodesZFe.size[2]; m++)
        {

            printf("processing mode %u / %u\n", m, imgDMmodesZFe.size[2]);


            // copy DM mode slice into input 2D array
            //
            for(uint64_t ii = 0; ii < msizexy; ii++)
            {
                data.image[ID_imfit].array.F[ii] =
                    imgDMmodesZFe.im->array.F[m * msizexy + ii];
            }


            // Decompose DM mode m input (imfitim) as a linear sum (linfitcoeff) of modes (loDMmodes)
            //
            linopt_imtools_image_fitModes("imfitim",
                                          "loDMmodes",
                                          "DMmaskCTRL",
                                          0.1,
                                          "linfitcoeff",
                                          linfitreuse,
                                          NULL);

            // Re-use decomposition setup for loop iteration > 0
            linfitreuse = 1;

            // Copy 1D output into 2D matrix
            //
            for(uint32_t jj = 0; jj < linfitsize; jj++)
            {
                data.image[IDcoeffmat].array.F[m * linfitsize + jj] =
                    data.image[IDRMM_coeff].array.F[jj];
            }

            // Reconstruct linear fit result (DM)
            //
            imageID IDtmp = -1;
            create_2Dimage_ID("testrc", msizex, msizey, &IDtmp);
            for(uint32_t jj = 0; jj < linfitsize; jj++)
            {
                for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                {
                    data.image[IDtmp].array.F[ii] +=
                        data.image[IDRMM_coeff].array.F[jj] *
                        imgloDMmodes.im->array.F[jj * msizex * msizey + ii];
                }
            }

            // Measure fit quality
            // res : residual normalized to input mode
            // res=0 : mode can be expressed as linear combination of lo modes
            // res is fraction of mode that cannot be expressed with lo modes
            double res  = 0.0;
            double resn = 0.0;
            for(uint64_t ii = 0; ii < msizex * msizey; ii++)
            {
                float v0 = data.image[IDtmp].array.F[ii] -
                           data.image[ID_imfit].array.F[ii];
                float vn = data.image[ID_imfit].array.F[ii];
                res += v0 * v0;
                resn += vn * vn;
            }
            res /= resn;

            // Measure fit vector power
            //
            double res1 = 0.0;
            for(uint32_t jj = 0; jj < linfitsize; jj++)
            {
                res1 += data.image[IDRMM_coeff].array.F[jj] *
                        data.image[IDRMM_coeff].array.F[jj];
            }

            delete_image_ID("testrc", DELETE_IMAGE_ERRMODE_WARNING);

            // LOcoeff: mixing fraction from lo modes
            //
            double LOcoeff = 1.0 / (1.0 + pow(10.0 * res, 4.0));

            if(res1 > 1.0)
            {
                LOcoeff *= 1.0 / (1.0 + pow((res1 - 1.0) * 0.1, 2.0));
            }

            fprintf(fpLOcoeff,
                    "%5u   %20g  %20g   %f\n",
                    m,
                    res,
                    res1,
                    LOcoeff);

            printf("    %5u   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
            fflush(stdout);

            if(LOcoeff < 0.01)
            {
                // construct linear fit (WFS space)
                for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                {
                    data.image[IDwfstmp].array.F[wfselem] = 0.0;
                }
                for(uint32_t jj = 0; jj < linfitsize; jj++)
                    for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                    {
                        data.image[IDwfstmp].array.F[wfselem] +=
                            data.image[IDRMM_coeff].array.F[jj] *
                            imgloRM.im->array.F[jj * wfssizexy + wfselem];
                    }

                for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                {
                    imgWFSmodesZFe.im->array.F[m * wfssizexy + wfselem] =
                        LOcoeff * data.image[IDwfstmp].array.F[wfselem] +
                        (1.0 - LOcoeff) *
                        imgWFSmodesZFe.im->array.F[m * wfssizexy + wfselem];
                }
            }
        }

        delete_image_ID("linfitcoeff", DELETE_IMAGE_ERRMODE_WARNING);
        delete_image_ID("imfitim", DELETE_IMAGE_ERRMODE_WARNING);
        delete_image_ID("wfsimtmp", DELETE_IMAGE_ERRMODE_WARNING);

        // save to disk
        fps_write_RUNoutput_image(data.fpsptr, "imfitmat", "imfitmat");

        //save_fits("imfitmat", "imfitmat.fits");
        delete_image_ID("imfitmat", DELETE_IMAGE_ERRMODE_WARNING);

        fclose(fpLOcoeff);
    }


    // TAG LINE 1247

    // save to disk
    //
    fps_write_RUNoutput_image(data.fpsptr, "WFSmodesZFe", "WFSmodesZFec");





    list_image_ID();


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
