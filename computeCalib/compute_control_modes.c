/**
 * @file compute_control_modes.c
 * @brief Compute AO control modes in both input (WFS) and output (DM) space
 *
 */
#include "ImageStreamIO/ImageStruct.h"

#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "linopt_imtools/linopt_imtools.h"

#include "ZernikePolyn/zernike_value.h"


#include "modes_spatial_extrapolate.h"


static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "compctrlmodes",
    .cmdkey      = "compctrlmodes",
    .description = "compute AO control modes in WFS and DM space",
    .description_long =
        "Compute AO control modes in both WFS and DM coordinate spaces. Generates the modal basis for closed-loop control."
};

// Local variables

static uint32_t AOloopindex;

static float svdlim;
static float CPAmax;
static float deltaCPA;

static float alignCX; // X center
static float alignCY; // Y center
static float alignID; // Inner diameter
static float alignOD; // Outer diameter

static uint32_t DMxsize;
static uint32_t DMysize;

static FPS FPS_zRMacqu;
static FPS FPS_loRMacqu;
static FPS FPS_DMcomb;

static char fname_DMmaskCTRL[FUNCTION_PARAMETER_STRMAXLEN];
static char fname_DMmaskEXTR[FUNCTION_PARAMETER_STRMAXLEN];
static char fname_zrespM[FUNCTION_PARAMETER_STRMAXLEN];
static char fname_WFSmask[FUNCTION_PARAMETER_STRMAXLEN];
static char fname_loRM[FUNCTION_PARAMETER_STRMAXLEN];
static char fname_loRMmodes[FUNCTION_PARAMETER_STRMAXLEN];

// Toggles
static int64_t update_RMfiles;
static int64_t update_align;

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, FPTYPE_INT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "AO loop index") \
    X(".svdlim", &svdlim, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "SVD limit") \
    X(".CPAmax", &CPAmax, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "max cycles per aperture (CPA)") \
    X(".deltaCPA", &deltaCPA, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "CPA increment") \
    X(".DMgeom.align.CX", &alignCX, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "beam X center on DM") \
    X(".DMgeom.align.CY", &alignCY, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "beam Y center on DM") \
    X(".DMgeom.align.ID", &alignID, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "beam inner diameter") \
    X(".DMgeom.align.OD", &alignOD, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "beam outer diameter") \
    X(".DMgeom.DMxsize", &DMxsize, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "DM x size") \
    X(".DMgeom.DMysize", &DMysize, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "DM y size") \
    X(".FPS_zRMacqu", &FPS_zRMacqu, FPTYPE_FPSNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED), "FPS zonal RM acquisition") \
    X(".DMgeom.FPS_DMcomb", &FPS_DMcomb, FPTYPE_FPSNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_FPS_RUN_REQUIRED), "FPS DM comb") \
    X(".DMgeom.DMmaskCTRL", fname_DMmaskCTRL, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT, "DM actuators controlled") \
    X(".DMgeom.DMmaskEXTR", fname_DMmaskEXTR, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT, "DM actuators extrapolated") \
    X(".zrespM", fname_zrespM, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "zonal response matrix") \
    X(".WFSmask", fname_WFSmask, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "WFS mask") \
    X(".auxRM.FPS_loRMacqu", &FPS_loRMacqu, FPTYPE_FPSNAME, 1, FPFLAG_DEFAULT_INPUT, "FPS low order modal RM acquisition") \
    X(".auxRM.loRM", fname_loRM, FPTYPE_STRING, 1, FPFLAG_DEFAULT_INPUT, "low order modal response matrix") \
    X(".auxRM.loRMmodes", fname_loRMmodes, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT, "low order RM modes") \
    X(".upRMfiles", &update_RMfiles, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "update RM files from FPSs") \
    X(".DMgeom.upAlign", &update_align, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "update default align (if no DMmaskRM)")

FPS_V2_SECTION5(FPS_PARAMS)


/**
 * @brief Create mode basis consisting of Zernike + Fourier modes
 *
 * @param msizex      x size of output array
 * @param msizey      y size of output array
 * @param CPAmax      maximum cycles per apeture
 * @param deltaCPA    increment
 * @param xc          beam center in array
 * @param yc          beam center in array
 * @param r0          central aperture radius
 * @param r1          outer radius
 * @param imgZFmodes  output image
 * @return errno_t
 */
static errno_t mk_ZernikeFourier_modal_basis(
    uint32_t msizex,
    uint32_t msizey,
    float    CPAmax,
    float    deltaCPA,
    double   xc,
    double   yc,
    double   r0,
    double   r1,
    IMGID   *imgZFmodes)
{
    DEBUG_TRACE_FSTART();

    (void) r0; // use of r0 to be implemented

    // Zernike modes
    long   zindex[10];
    double zcpa[10];
    /// assigning CPA for each Zernike (somewhat arbitrary... used to sort
    /// modes in CPA)

    zindex[0] = 1; // tip
    zcpa[0]   = 0.0;

    zindex[1] = 2; // tilt
    zcpa[1]   = 0.0;

    zindex[2] = 4; // focus
    zcpa[2]   = 0.25;

    zindex[3] = 3; // astig
    zcpa[3]   = 0.4;

    zindex[4] = 5; // astig
    zcpa[4]   = 0.4;

    zindex[5] = 7; // coma
    zcpa[5]   = 0.6;

    zindex[6] = 8; // coma
    zcpa[6]   = 0.6;

    zindex[7] = 6; // trefoil
    zcpa[7]   = 1.0;

    zindex[8] = 9; // trefoil
    zcpa[8]   = 1.0;

    zindex[9] = 12;
    zcpa[9]   = 1.5;

    // Number of Zernike modes
    //
    int NBZ = 0;
    for(uint32_t m = 0; m < 10; m++)
    {
        if(zcpa[m] < CPAmax)
        {
            NBZ++;
        }
    }


    // Here we create simple Fourier modes

    {
        IMGID imgoutm = imgid_make_from_name("CPAmodes");

        // optional mask
        //
        IMGID imgmask = imgid_make_from_name("modesZFmask");
        resolveIMGID(
            &imgmask, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);

        linopt_imtools_makeCPAmodes(&imgoutm,
                                    msizex,
                                    msizex, // sizey (assuming square as before)
                                    0.5 * msizex, // xcenter
                                    0.5 * msizex, // ycenter
                                    0, // rCPAmin
                                    CPAmax, // rCPAmax
                                    CPAmax, // CPAmax
                                    deltaCPA, // deltaCPA
                                    0.5 * msizex, // radius
                                    1.2, // radfactlim
                                    0.0, // fpowerlaw
                                    0.0, // fpowerlaw_minf
                                    0.0, // fpowerlaw_maxf
                                    0, // writeMfile
                                    NULL, // outNBmax
                                    imgmask, // imgmask
                                    0.0, // extrfactor
                                    0.0 // extroffset
                                   );
    }

    imageID ID0 = image_ID("CPAmodes", data.core.image, data.core.NB_MAX_IMAGE);

    imageID IDfreq = image_ID("cpamodesfreq", data.core.image, data.core.NB_MAX_IMAGE);

    printf("  %u %u %ld\n",
           msizex,
           msizey,
           (long)(data.core.image[ID0].md[0].size[2] - 1));


    imgZFmodes->mdt->naxis   = 3;
    imgZFmodes->mdt->size[0] = msizex;
    imgZFmodes->mdt->size[1] = msizey;
    imgZFmodes->mdt->size[2] = data.core.image[ID0].md[0].size[2] - 1 + NBZ;
    createimagefromIMGID(imgZFmodes);

    imageID IDmfcpa;
    create_2Dimage_ID("modesfreqcpa",
                      data.core.image[ID0].md[0].size[2] - 1 + NBZ,
                      1,
                      &IDmfcpa);

    zernike_init();

    // First NBZ modes are Zernike modes
    //
    for(int k = 0; k < NBZ; k++)
    {
        data.core.image[IDmfcpa].array.F[k] = zcpa[k];
        for(uint32_t ii = 0; ii < msizex; ii++)
            for(uint32_t jj = 0; jj < msizey; jj++)
            {
                double x = (double) ii - xc;
                double y = (double) jj - yc;
                double r = sqrt(x * x + y * y) / r1;
                double PA  = atan2(y, x);

                imgZFmodes->im->array
                .F[k * msizex * msizey + jj * msizex + ii] =
                    Zernike_value(zindex[k], r, PA);
            }
    }


    // Copy Fourier modes into basis
    //
    for(uint32_t k = 0; k < data.core.image[ID0].md[0].size[2] - 1; k++)
    {
        data.core.image[IDmfcpa].array.F[k + NBZ] =
            data.core.image[IDfreq].array.F[k + 1];
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            imgZFmodes->im->array.F[(k + NBZ) * msizex * msizey + ii] =
                data.core.image[ID0].array.F[(k + 1) * msizex * msizey + ii];
        }
    }

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


/**
 * modes_mask_normalize() - normalize modal basis
 *                          over a mask
 * @imgmodeC: 3D mode cube (xsize x ysize x nmodes),
 *            modified in-place
 * @imgmask:  2D mask defining active region
 *
 * For each mode slice: removes the DC offset over
 * the mask, then scales to RMS = 1 over the mask.
 * Writes per-mode RMS values to "rmscomp.dat".
 */
static errno_t modes_mask_normalize(IMGID imgmodeC, IMGID imgmask)
{
    DEBUG_TRACE_FSTART();

    FILE *fp = fopen("rmscomp.dat", "w");

    uint32_t sizex  = imgmodeC.md->size[0];
    uint32_t sizey  = imgmodeC.md->size[1];
    uint64_t sizexy = (uint64_t) sizex * sizey;

    for(uint32_t k = 0; k < imgmodeC.md->size[2]; k++)
    {
        // set RMS = 1 over mask
        double rms     = 0.0;
        double totmask = 0.0;
        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            // data.core.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
            rms += imgmodeC.im->array.F[ii] * imgmodeC.im->array.F[ii] *
                   imgmask.im->array.F[ii];
            totmask += imgmask.im->array.F[ii];
        }
        rms = sqrt(rms / totmask);
        printf("[%5d] Mode %u   RMS = %lf\n", __LINE__, k, rms);

        fprintf(fp, "%5u  %g ", k, rms);


        // Compute total of image over mask -> totvm
        double totvm = 0.0;
        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            totvm +=
                imgmodeC.im->array.F[k * sizexy + ii] * imgmask.im->array.F[ii];
        }

        // compute DC offset in mode
        double offset = totvm / totmask;

        // remove DM offset
        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            imgmodeC.im->array.F[k * sizexy + ii] -= offset;
        }

        offset = 0.0;
        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            offset +=
                imgmodeC.im->array.F[k * sizexy + ii] * imgmask.im->array.F[ii];
        }

        // set RMS = 1 over mask
        rms = 0.0;
        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            imgmodeC.im->array.F[k * sizexy + ii] -= offset / totmask;
            rms += imgmodeC.im->array.F[k * sizexy + ii] *
                   imgmodeC.im->array.F[k * sizexy + ii] *
                   imgmask.im->array.F[ii];
        }
        rms = sqrt(rms / totmask);
        printf("\r Mode %u   RMS = %lf   ", k, rms);
        fprintf(fp, " %g\n", rms);

        for(uint64_t ii = 0; ii < sizexy; ii++)
        {
            imgmodeC.im->array.F[k * sizexy + ii] /= rms;
        }
    }
    fclose(fp);
    printf("\n");

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static __attribute__((unused)) errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        // FPS are not required

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".zrespM")].fpflag &=
            ~FPFLAG_STREAM_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".WFSmask")].fpflag &=
            ~FPFLAG_STREAM_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".auxRM.FPS_loRMacqu")].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".FPS_zRMacqu")].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.FPS_DMcomb")].fpflag &=
            ~FPFLAG_FPS_RUN_REQUIRED;

    }

    return RETURN_SUCCESS;
}


static errno_t customCONFcheck()
{
    if(data.core.fpsptr != NULL)
    {

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".zrespM")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".WFSmask")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;


        if(FPS_zRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.core.fpsptr,
                                                 functionparameter_GetParamIndex(data.core.fpsptr, ".FPS_zRMacqu"),
                                                 &FPS_zRMacqu);
        }

        if(FPS_loRMacqu.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.core.fpsptr,
                                                 functionparameter_GetParamIndex(data.core.fpsptr, ".auxRM.FPS_loRMacqu"),
                                                 &FPS_loRMacqu);
        }

        if(FPS_DMcomb.SMfd < 1)
        {
            functionparameter_ConnectExternalFPS(data.core.fpsptr,
                                                 functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.FPS_DMcomb"),
                                                 &FPS_DMcomb);
        }


        // Update RM files
        if(data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".upRMfiles")].fpflag & FPFLAG_ONOFF)
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
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                       ".DMgeom.DMmaskEXTR",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/zrespM_mn.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                       ".zrespM",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/dmmask_mksl.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                       ".DMgeom.DMmaskCTRL",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/wfsmask_mkm.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
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
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                       ".loRM",
                                                       fname);

                SNPRINTF_CHECK(fname,
                               FUNCTION_PARAMETER_STRMAXLEN,
                               "%s/RMpokeCube.fits",
                               datadir);
                functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                       ".loRMmodes",
                                                       fname);
            }

            // set back to OFF
            data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".upRMfiles")].fpflag &= ~FPFLAG_ONOFF;
        }


        // update align params for auto mask
        if(data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.upAlign")].fpflag & FPFLAG_ONOFF)
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

                functionparameter_SetParamValue_INT64(data.core.fpsptr,
                                                      ".DMgeom.DMxsize",
                                                      DMxsize);
                functionparameter_SetParamValue_INT64(data.core.fpsptr,
                                                      ".DMgeom.DMysize",
                                                      DMysize);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".DMgeom.align.CX",
                                                        cx);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".DMgeom.align.CY",
                                                        cy);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".DMgeom.align.OD",
                                                        od);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".DMgeom.align.ID",
                                                        id);
            }
            data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.upAlign")].fpflag &= ~FPFLAG_ONOFF;
        }


        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.DMmaskCTRL")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
        data.core.fpsptr->parray[functionparameter_GetParamIndex(data.core.fpsptr, ".DMgeom.DMmaskEXTR")].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    printf(
        "Compute AO control modes\n"
        "Corresponding pairs of DM and WFS modes are computed\n"
        "Intermediate Results (in datadir) and steps:\n"
        "(1) Construct DM-space modes\n"
        "    DMmodesZF      DM Zernike+Fourier modes, normalized over active actuators DMmaskCTRL\n"
        "    DMmodesZFe     extrapolated over DMmaskEXT\n"
        "    WFSmodesZFe    WFS response to DMmodesZFe\n"
        "(2) Apply aux DM modes\n"
        "    imfitmat       linear mapping between DMmodesZFw and auxDMmodes\n"
        "    LOcoeff.txt    stats of linear mapping\n"
    );

    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    /*IMGID inimg = makeIMGID(inimname);
    resolveIMGID(
        &inimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (inimg.ID == -1) return RETURN_FAILURE;

    IMGID outimg = makeIMGID(outimname);
    resolveIMGID(
        &outimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (outimg.ID == -1) return RETURN_FAILURE;
    */
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // custom initialization
    if(CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
    {
        // procinfo is accessible here
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {


        // MaskMode = 0  : tapered masking
        // MaskMode = 1  : STRICT masking
        //
        // if BlockNB < 0 : do all blocks
        // if BlockNB >= 0 : only update single block (untested)
        //int MaskMode = 0;
        //int BlockNB  = -1;

        // full set of DM actuators to be controlled
        //
        load_fits(fname_DMmaskCTRL, "DMmaskCTRL", LOADFITS_ERRMODE_ERROR, NULL);
        IMGID imgDMmaskCTRL = imgid_make_from_name("DMmaskCTRL");
        resolveIMGID(
            &imgDMmaskCTRL, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgDMmaskCTRL.ID == -1) return RETURN_FAILURE;

        // DM actuators to be extrapolated from neighbors
        // this is a subset of DMmaskCTRL
        //
        load_fits(fname_DMmaskEXTR, "DMmaskEXTR", LOADFITS_ERRMODE_ERROR, NULL);
        IMGID imgDMmaskEXTR = imgid_make_from_name("DMmaskEXTR");
        resolveIMGID(
            &imgDMmaskEXTR, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgDMmaskEXTR.ID == -1) return RETURN_FAILURE;


        // CREATE ZERNIKE+FOURIER DM MODES BASIS
        // Zernike modes are centered on (alignCX, alignCY)
        // output : imgDMmodesZF ("DMmodesZF")
        //
        IMGID imgDMmodesZF = imgid_make_from_name("DMmodesZF");
        mk_ZernikeFourier_modal_basis(DMxsize,
                                      DMysize,
                                      CPAmax,
                                      deltaCPA,
                                      alignCX,
                                      alignCY,
                                      alignID,
                                      alignOD,
                                      &imgDMmodesZF);

        // Adjust mode amplitude to have RMS=1 over DMmaskCTRL
        // in-place computation
        //
        modes_mask_normalize(imgDMmodesZF, imgDMmaskCTRL);

        // save to disk
        fps_write_RUNoutput_image(data.core.fpsptr, "DMmodesZF", "DMmodesZF");


        // EXTRAPOLATE DM MODES
        // output : imgDMmodesZFe
        //
        IMGID imgDMmodesZFe = imgid_make_from_name("DMmodesZFe");
        IMGID imgcpa      = imgid_make_from_name("modesfreqcpa");

        modes_spatial_extrapolate(
            imgDMmodesZF,
            imgDMmaskCTRL,
            imgcpa,
            &imgDMmodesZFe
        );

        // save to disk
        fps_write_RUNoutput_image(data.core.fpsptr, "DMmodesZFe", "DMmodesZFe");


        // TAG LINE 889


        fps_write_RUNoutput_image(data.core.fpsptr, "DMmaskCTRL", "DMmaskCTRL"); // test

        // set pixels to zero if neither part of DMmaskCTRL or DMmaskEXT
        // output (in-place) : imgDMmodesZFe

        uint32_t msizex = imgDMmaskCTRL.md->size[0];
        uint32_t msizey = imgDMmaskCTRL.md->size[1];
        uint64_t msizexy = (uint64_t) msizex;
        msizexy *= msizey;
        printf("size %ld %ld %ld\n", (long) msizex, (long) msizey,
               (long) imgDMmodesZFe.md->size[2]);
        for(uint32_t kk = 0; kk < imgDMmodesZFe.md->size[2]; kk++)
        {
            for(uint64_t ii = 0; ii < msizexy; ii++)
            {
                float mval = 1.0 - (imgDMmaskCTRL.im->array.F[ii] - 1.0) *
                (imgDMmaskEXTR.im->array.F[ii] - 1.0);
                imgDMmodesZFe.im->array.F[kk * msizexy + ii] *= mval;
            }
        }
        // save to disk
        fps_write_RUNoutput_image(data.core.fpsptr, "DMmodesZFe", "DMmodesZFem");


        // TAG LINE 934

        // TODO Remove modes

        // TAG LINE 1040


        // zonal response matrix
        load_fits(fname_zrespM, "zrespM", LOADFITS_ERRMODE_ERROR, NULL);
        IMGID imgzrespM = imgid_make_from_name("zrespM");
        resolveIMGID(
            &imgzrespM, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgzrespM.ID == -1) return RETURN_FAILURE;


        // COMPUTE WFS RESPONSE TO MODES
        // output : imgWFSmodesZFe
        //
        uint32_t wfssizex = imgzrespM.md->size[0];
        uint32_t wfssizey = imgzrespM.md->size[1];
        uint64_t wfssizexy = wfssizex;
        wfssizexy *= wfssizey;
        IMGID imgWFSmodesZFe = imgid_make_from_name_3D("WFSmodesZFe", wfssizex, wfssizey,
                                            imgDMmodesZFe.md->size[2]);
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

            for(m = 0; m < imgDMmodesZFe.md->size[2]; m++)
            {
                m1 = m * wfssizexy;

                printf("\r %5u / %5u   ", m, imgDMmodesZFe.md->size[2]);
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
        fps_write_RUNoutput_image(data.core.fpsptr, "WFSmodesZFe", "WFSmodesZFe");


        // TAG LINE 1102


        // APPLY MODAL (AUX) RESPONSE MATRIX IF IT EXISTS
        // output (in-place) : imgWFSmodesZFe
        //
        imageID IDloRM = -1;
        load_fits(fname_loRM, "loRM", LOADFITS_ERRMODE_WARNING, &IDloRM);
        imageID IDloDMmodes = -1;
        load_fits(fname_loRMmodes, "loDMmodes", LOADFITS_ERRMODE_WARNING, &IDloDMmodes);

        fps_write_RUNoutput_image(data.core.fpsptr, "loDMmodes", "loDMmodes"); // test


        if((IDloRM != -1) && (IDloDMmodes != -1))
        {
            FILE   *fpLOcoeff;
            {
                char ffname[STRINGMAXLEN_FULLFILENAME];
                WRITE_FULLFILENAME(ffname, "./%s/LOcoeff.txt", data.core.fpsptr->md->datadir);
                fpLOcoeff = fopen(ffname, "w");
                if(fpLOcoeff == NULL)
                {
                    printf("ERROR: cannot create file \"LOcoeff.txt\"\n");
                    exit(0);
                }
            }


            IMGID imgloRM = imgid_make_from_name("loRM");
            resolveIMGID(
                &imgloRM, ERRMODE_WARN,
                data.core.image,
                data.core.NB_MAX_IMAGE);
                if (imgloRM.ID == -1) return RETURN_FAILURE;

            IMGID imgloDMmodes = imgid_make_from_name("loDMmodes");
            resolveIMGID(
                &imgloDMmodes, ERRMODE_WARN,
                data.core.image,
                data.core.NB_MAX_IMAGE);
                if (imgloDMmodes.ID == -1) return RETURN_FAILURE;


            printf("Using low-order modal response [%ld %ld]\n",
                   IDloRM,
                   IDloDMmodes);

            uint32_t linfitsize = imgloDMmodes.md->size[2];

            imageID IDRMM_coeff = -1;
            create_2Dimage_ID("linfitcoeff", linfitsize, 1, &IDRMM_coeff);

            imageID ID_imfit = -1;
            create_2Dimage_ID("imfitim", msizex, msizey, &ID_imfit);

            imageID IDcoeffmat = -1;
            create_2Dimage_ID("imfitmat",
                              linfitsize,
                              imgWFSmodesZFe.md->size[2],
                              &IDcoeffmat);

            // Reconstructed DM modes from aux
            //
            imageID IDauxDMmodesrec = -1;
            create_3Dimage_ID("auxDMmodesrec", msizex, msizey, imgDMmodesZFe.md->size[2],
                              &IDauxDMmodesrec);
            // null space (complement of above)
            imageID IDauxDMmodesnull = -1;
            create_3Dimage_ID("auxDMmodesnull", msizex, msizey, imgDMmodesZFe.md->size[2],
                              &IDauxDMmodesnull);


            int linfitreuse = 0;

            imageID IDwfstmp = -1;
            create_2Dimage_ID("wfsimtmp", wfssizex, wfssizey, &IDwfstmp);


            fprintf(fpLOcoeff, "# col1   Input DM mode index\n");
            fprintf(fpLOcoeff,
                    "# col2   Fit residual: fraction of input DM mode that cannot be expressed by aux modes\n");
            fprintf(fpLOcoeff, "# col3   Fit vector power (squared morm)\n");
            fprintf(fpLOcoeff,
                    "# col4   Mixing fraction (a): new mode equal to a x aux + (1-a) x previous\n");
            for(uint32_t m = 0; m < imgDMmodesZFe.md->size[2]; m++)
            {

                //printf("processing mode %u / %u\n", m, imgDMmodesZFe.size[2]);


                // copy DM mode slice into input 2D array
                //
                for(uint64_t ii = 0; ii < msizexy; ii++)
                {
                    data.core.image[ID_imfit].array.F[ii] =
                        imgDMmodesZFe.im->array.F[m * msizexy + ii];
                }

                {
                    char fnameimout[STRINGMAXLEN_FILENAME];
                    WRITE_FILENAME(fnameimout, "linfit_input.%04d", m);
                    fps_write_RUNoutput_image(data.core.fpsptr, "imfitim", fnameimout);
                }

                // Decompose DM mode m input (imfitim) as a linear sum (linfitcoeff) of modal modes (loDMmodes)
                //
                linopt_imtools_image_fitModes("imfitim",
                                              "loDMmodes",
                                              "DMmaskCTRL",
                                              0.001,
                                              "linfitcoeff",
                                              linfitreuse,
                                              NULL);

                // Re-use decomposition setup for loop iteration > 0
                linfitreuse = 1;

                // Copy 1D output into 2D matrix
                //
                for(uint32_t jj = 0; jj < linfitsize; jj++)
                {
                    data.core.image[IDcoeffmat].array.F[m * linfitsize + jj] =
                        data.core.image[IDRMM_coeff].array.F[jj];
                }


                // Reconstruct linear fit result (DM)
                //
                for(uint32_t jj = 0; jj < linfitsize; jj++)
                {
                    for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                    {
                        data.core.image[IDauxDMmodesrec].array.F[m * msizex * msizey + ii] +=
                            data.core.image[IDRMM_coeff].array.F[jj] *
                            imgloDMmodes.im->array.F[jj * msizex * msizey + ii];
                    }
                }
                // ... and it complement (null)
                for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                {
                    data.core.image[IDauxDMmodesnull].array.F[m * msizex * msizey + ii] =
                        data.core.image[ID_imfit].array.F[ii]
                        - data.core.image[IDauxDMmodesrec].array.F[m * msizex * msizey + ii];
                }


                // Measure fit quality
                // res : residual normalized to input mode
                // res=0 : mode can be expressed as linear combination of lo modes
                // res is fraction of mode that cannot be expressed with lo modes
                double res  = 0.0;
                double resn = 0.0;
                for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                {
                    float v0 = data.core.image[IDauxDMmodesrec].array.F[m * msizex * msizey + ii] -
                               data.core.image[ID_imfit].array.F[ii];
                    float vn = data.core.image[ID_imfit].array.F[ii];
                    float mcoeff = imgDMmaskCTRL.im->array.F[ii];
                    res += v0 * v0 * mcoeff;
                    resn += vn * vn * mcoeff;
                }
                res /= resn;

                // Measure fit vector power
                //
                double res1 = 0.0;
                for(uint32_t jj = 0; jj < linfitsize; jj++)
                {
                    res1 += data.core.image[IDRMM_coeff].array.F[jj] *
                            data.core.image[IDRMM_coeff].array.F[jj];
                }


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

                //printf("    %5u   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
                //fflush(stdout);

                if(LOcoeff > 0.01)
                {
                    // construct linear fit (WFS space)
                    for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                    {
                        data.core.image[IDwfstmp].array.F[wfselem] = 0.0;
                    }
                    for(uint32_t jj = 0; jj < linfitsize; jj++)
                        for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                        {
                            data.core.image[IDwfstmp].array.F[wfselem] +=
                                data.core.image[IDRMM_coeff].array.F[jj] *
                                imgloRM.im->array.F[jj * wfssizexy + wfselem];
                        }

                    for(uint64_t wfselem = 0; wfselem < wfssizexy; wfselem++)
                    {
                        imgWFSmodesZFe.im->array.F[m * wfssizexy + wfselem] =
                            LOcoeff * data.core.image[IDwfstmp].array.F[wfselem] +
                            (1.0 - LOcoeff) *
                            imgWFSmodesZFe.im->array.F[m * wfssizexy + wfselem];
                    }
                }
            }

            delete_image_ID("linfitcoeff", DELETE_IMAGE_ERRMODE_WARNING);
            delete_image_ID("imfitim", DELETE_IMAGE_ERRMODE_WARNING);
            delete_image_ID("wfsimtmp", DELETE_IMAGE_ERRMODE_WARNING);

            // save to disk

            fps_write_RUNoutput_image(data.core.fpsptr, "imfitmat", "imfitmat");

            fps_write_RUNoutput_image(data.core.fpsptr, "auxDMmodesrec", "auxDMmodesrec");
            delete_image_ID("auxDMmodesrec", DELETE_IMAGE_ERRMODE_WARNING);

            fps_write_RUNoutput_image(data.core.fpsptr, "auxDMmodesnull", "auxDMmodesnull");
            delete_image_ID("auxDMmodesnull", DELETE_IMAGE_ERRMODE_WARNING);

            //save_fits("imfitmat", "imfitmat.fits");
            delete_image_ID("imfitmat", DELETE_IMAGE_ERRMODE_WARNING);

            fclose(fpLOcoeff);
            imgid_free(&imgloRM);
            imgid_free(&imgloDMmodes);
        }


        // TAG LINE 1247

        // save to disk
        //
        fps_write_RUNoutput_image(data.core.fpsptr, "WFSmodesZFe", "WFSmodesZFec");

        imgid_free(&imgDMmaskCTRL);
        imgid_free(&imgDMmaskEXTR);
        imgid_free(&imgDMmodesZF);
        imgid_free(&imgDMmodesZFe);
        imgid_free(&imgcpa);
        imgid_free(&imgzrespM);
        imgid_free(&imgWFSmodesZFe);


        //list_image_ID();


        // streamprocess(inimg, outimg);
        // processinfo_update_output_stream(processinfo, outimg.im, NULL);

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t
CLIADDCMD_cacao_computeCalib__compute_control_modes()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(
    FPS_app_info,
    FPS_PARAMS,
    compute_function,

    customCONFcheck)
#endif
