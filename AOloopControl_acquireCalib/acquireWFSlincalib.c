/**
 * @file acquireWFSlincalib.c
 * @brief Acquire linear WFS response
 *
 */

#include "CLIcore/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


#include "AOloopControl_compTools/AOloopControl_compTools.h"
#include "computeCalib/computeHadamard.h"

#include "image_gen/image_gen.h"


static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "acqWFSlincal",
    .cmdkey      = "acqWFSlincal",
    .description = "acquire linear WFS calibration"
};

// Local variables pointers
static uint32_t *AOloopindex;
static float    *pokeampl;

static char *dmstream;

// timing params
static FUNCTION_PARAMETER_STRUCT FPS_mlat;

// Toggles
static uint64_t *update_mlat;
static uint64_t *update_RMDMmask;

// WFS frame rate [Hz]
static float *WFSfrequ;

// Hardware latency in unit of WFS frame
static float *hardwlatfr;

static uint32_t *delayfr;
static uint32_t *delayRM1us;
static uint32_t *NBave;
static uint32_t *NBexcl;
static uint32_t *NBcycle;
static uint32_t *NBinnerCycle;

static uint64_t *MaskMode;

static FUNCTION_PARAMETER_STRUCT FPS_DMcomb;

static uint32_t *DMMODE;

static float *maskDMp0;
static float *maskDMc0;
static float *maskDMp1;
static float *maskDMc1;

static float *DMproxrad;

static float *maskWFSp0;
static float *maskWFSc0;
static float *maskWFSp1;
static float *maskWFSc1;

static char *fn_pokeC;
static char *fn_RMDMmask;

static float *RMDMmaskCx;
static float *RMDMmaskCy;
static float *RMDMmaskR;

static uint64_t *normalize;

static uint64_t *Hpokemode;

static uint64_t *autotiming;

static uint64_t *compPokeMat;

// executable scripts
static char *exec_post_RMdecode;
static char *exec_post_mkDMWFSmasks;
static char *exec_post_mkDMslaveact;
static char *exec_post_mkLODMmodes;

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "loop index") \
    X(".dmstream", &dmstream, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "DM stream") \
    X(".ampl", &pokeampl, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "RM poke amplitude") \
    X(".timing.FPS_mlat", &FPS_mlat, FPTYPE_FPSNAME, 1, FPFLAG_DEFAULT_INPUT, "hardware latency") \
    X(".timing.upmlat", &update_mlat, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "update latency from FPS") \
    X(".timing.WFSfrequ", &WFSfrequ, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "WFS frame rate [Hz]") \
    X(".timing.hardwlatfr", &hardwlatfr, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "hardware latency [fr]") \
    X(".timing.autoTiming", &autotiming, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "Auto Timing") \
    X(".timing.delayfr", &delayfr, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "frame delay, whole part") \
    X(".timing.delayRM1us", &delayRM1us, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "Sub-frame delay [us]") \
    X(".timing.NBave", &NBave, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "Number of frames averaged for a single poke measurement") \
    X(".timing.NBexcl", &NBexcl, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "Number of frames excluded") \
    X(".timing.NBcycle", &NBcycle, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "Number of measurement cycles to be repeated") \
    X(".timing.NBinnerCycle", &NBinnerCycle, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "Number of inner cycles") \
    X(".RMDMmask.FPS_DMcomb", &FPS_DMcomb, FPTYPE_FPSNAME, 1, FPFLAG_DEFAULT_INPUT, "DM control process") \
    X(".RMDMmask.DMMODE", &DMMODE, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "0:spatial, 1:modal") \
    X(".RMDMmask.Cx", &RMDMmaskCx, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "X center") \
    X(".RMDMmask.Cy", &RMDMmaskCy, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "Y center") \
    X(".RMDMmask.R", &RMDMmaskR, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "radius") \
    X(".RMDMmask.upmlat", &update_RMDMmask, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "update RMDMmask from FPS") \
    X(".MaskMode", &MaskMode, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "Mask mode, DM and WFS") \
    X(".DMmask.RMp0", &maskDMp0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask, point0 percentile point") \
    X(".DMmask.RMc0", &maskDMc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask, point0 coefficient") \
    X(".DMmask.RMp1", &maskDMp1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask, point1 percentile point") \
    X(".DMmask.RMc1", &maskDMc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM mask, point1 coefficient") \
    X(".DMmask.proxrad", &DMproxrad, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "DM actuator proximity radius") \
    X(".WFSmask.RMp0", &maskWFSp0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask, point0 percentile point") \
    X(".WFSmask.RMc0", &maskWFSc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask, point0 coefficient") \
    X(".WFSmask.RMp1", &maskWFSp1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask, point1 percentile point") \
    X(".WFSmask.RMc1", &maskWFSc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "WFS mask, point1 coefficient") \
    X(".fn_pokeC", &fn_pokeC, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT, "Poke sequence cube") \
    X(".fn_RMDMmask", &fn_RMDMmask, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT, "RM active DM actuators mask") \
    X(".normalize", &normalize, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "Normalize WFS frames") \
    X(".Hpoke", &Hpokemode, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "Hadamard poke mode") \
    X(".compPokeMat", &compPokeMat, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "(re)compute poke matrix") \
    X(".exec.RMdecode", &exec_post_RMdecode, FPTYPE_FILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED, "RM decode script") \
    X(".exec.mkDMWFSmasks", &exec_post_mkDMWFSmasks, FPTYPE_FILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED, "Make DM and WFS masks") \
    X(".exec.mkDMslaveact", &exec_post_mkDMslaveact, FPTYPE_FILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED, "Make DM slaved actuators") \
    X(".exec.mkLODMmodes", &exec_post_mkLODMmodes, FPTYPE_FILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED, "Make DM low order modes")


FPS_V2_SECTION5(FPS_PARAMS)


static errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        long fpi;

        // DM stream is required
        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".dmstream");
        if(fpi > -1)
            data.core.fpsptr
                ->parray[fpi]
                .fpflag |= FPFLAG_STREAM_RUN_REQUIRED;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".timing.FPS_mlat");
        if(fpi > -1)
            data.core.fpsptr
                ->parray[fpi]
                .fpflag &= ~FPFLAG_FPS_RUN_REQUIRED;

        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".RMDMmask.FPS_DMcomb");
        if(fpi > -1)
            data.core.fpsptr
                ->parray[fpi]
                .fpflag &= ~FPFLAG_FPS_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}


// create simple poke matrix
static imageID mkSimpleZpokeM(uint32_t dmxsize,
                              uint32_t dmysize,
                              char    *IDout_name)
{
    imageID  IDout;
    uint64_t dmxysize;

    dmxysize = dmxsize * dmysize;

    create_3Dimage_ID(IDout_name, dmxsize, dmysize, dmxysize, &IDout);

    for(uint64_t kk = 0; kk < dmxysize; kk++)
    {
        data.core.image[IDout].array.F[kk * dmxysize + kk] = 1.0;
    }

    return IDout;
}


static errno_t customCONFcheck()
{
    if(data.core.fpsptr != NULL)
    {

        long fpi_FPS_mlat = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".timing.FPS_mlat");
        long fpi_FPS_DMcomb = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".RMDMmask.FPS_DMcomb");
        long fpi_update_mlat = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".timing.upmlat");
        long fpi_autotiming = 
            functionparameter_GetParamIndex(
                data.core.fpsptr, ".timing.autoTiming");

        if(FPS_mlat.SMfd < 1)
        {
            printf("Connecting to mlat FPS\n");

            if(fpi_FPS_mlat > -1)
            {
                functionparameter_ConnectExternalFPS(data.core.fpsptr,
                                                     fpi_FPS_mlat,
                                                     &FPS_mlat);
            }
        }


        if(FPS_DMcomb.SMfd < 1)
        {
            printf("Connecting to DMcomb FPS\n");

            if(fpi_FPS_DMcomb > -1)
            {
                functionparameter_ConnectExternalFPS(data.core.fpsptr,
                                                     fpi_FPS_DMcomb,
                                                     &FPS_DMcomb);
            }
        }


        // update hardware latency
        //
        if(fpi_update_mlat > -1 && (data.core.fpsptr->parray[fpi_update_mlat].fpflag & FPFLAG_ONOFF))
        {
            printf("Updating from mlat FPS\n");

            if(FPS_mlat.SMfd > 0)
            {
                float WFSfrequ = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                 ".framerateHz");

                float latencyfr = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                  ".latencyfr");

                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".timing.WFSfrequ",
                                                        WFSfrequ);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                        ".timing.hardwlatfr",
                                                        latencyfr);
            }
            data.core.fpsptr->parray[fpi_update_mlat].fpflag &= ~FPFLAG_ONOFF;
        }


        // Auto timing
        //
        if(fpi_autotiming > -1 && (data.core.fpsptr->parray[fpi_autotiming].fpflag & FPFLAG_ONOFF))  // ON state
        {
            printf("UPDATE TIMING >>>>>>>>>\n");


            // long delayfr = (long) (1000000.0*latfr);

            // RMdelay = hardwlaten - 0.5 - excl/2
            double RMdelay =
                *hardwlatfr - 0.5 - 0.5 * (*NBexcl);

            int val_RMdelayfr =
                ((int)((*hardwlatfr) - 0.5 -
                       0.5 * (*NBexcl) +
                       10.0)) +
                1 - 10;

            int val_delayRM1us = (int)((1.0 * val_RMdelayfr - RMdelay) /
                                       (*WFSfrequ) * 1000000.0);

            if(RMdelay > 0)
            {
                *delayfr = val_RMdelayfr;
                *delayRM1us = val_delayRM1us;
            }
            else
            {
                *delayfr = 0;
                *delayRM1us = 0;
            }

        }


        imageID IDdmRM = image_ID(dmstream,
            data.core.image,
            data.core.NB_MAX_IMAGE);
        if(IDdmRM != -1)
        {
            uint32_t DMxsize = data.core.image[IDdmRM].md->size[0];
            uint32_t DMysize = data.core.image[IDdmRM].md->size[1];
            printf("DM size : %u x %u\n", DMxsize, DMysize);


            // update RM DM mask
            //
            long fpi_update_RMDMmask = 
                functionparameter_GetParamIndex(
                    data.core.fpsptr, ".RMDMmask.upmlat");
            if(fpi_update_RMDMmask > -1 && (data.core.fpsptr->parray[fpi_update_RMDMmask].fpflag & FPFLAG_ONOFF))
            {
                printf("Updating RM DM mask\n");


                uint32_t DMMODEin = 1;
                if(FPS_DMcomb.SMfd > 0)
                {
                    DMMODEin = functionparameter_GetParamValue_UINT32(&FPS_DMcomb,
                               ".DMmode");
                }

                // Update values
                //
                functionparameter_SetParamValue_UINT32(data.core.fpsptr, ".RMDMmask.DMMODE",
                                                       DMMODEin);

                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr, ".RMDMmask.Cx",
                                                        0.5 * DMxsize);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr, ".RMDMmask.Cy",
                                                        0.5 * DMysize);
                functionparameter_SetParamValue_FLOAT32(data.core.fpsptr, ".RMDMmask.R",
                                                        0.5 * DMxsize + 0.6);

                // load or create RMDMmask
                // this is the map of active actuatores to be poked in RM
                //
                {
                    char fnameRMDMmask[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(fnameRMDMmask,
                            functionparameter_GetParamPtr_STRING(data.core.fpsptr, ".sn_RMDMmask"),
                            FUNCTION_PARAMETER_STRMAXLEN);

                    imageID ID_RMDMmask;
                    load_fits(fnameRMDMmask, "RMDMmask", 1, &ID_RMDMmask);
                    if(ID_RMDMmask == -1)
                    {
                        // create it
                        //
                        if(*DMMODE == 0)
                        {
                            // spatial DM
                            // make centered disk
                            //
                            make_disk("RMDMmask",
                                      DMxsize,
                                      DMysize,
                                      *RMDMmaskCx,
                                      *RMDMmaskCy,
                                      *RMDMmaskR);
                            //0.5 * (DMxsize) + 0.6);
                        }
                        else
                        {
                            // modal DM
                            // all pixels to 1
                            //
                            imageID ID;
                            create_2Dimage_ID(
                                "RMDMmask",
                                DMxsize,
                                DMysize,
                                &ID);
                            for(uint64_t ii = 0; ii < DMxsize * DMysize; ii++)
                            {
                                data.core.image[ID].array.F[ii] = 1.0;
                            }
                        }
                        fps_write_RUNoutput_image(data.core.fpsptr, "RMDMmask", "RMDMmask");
                    }

                    char fname_RMDMmask[STRINGMAXLEN_FULLFILENAME];
                    WRITE_FULLFILENAME(fname_RMDMmask,
                                       "./%s/RMDMmask.fits",
                                       data.core.fpsptr->md->datadir);
                    functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                           ".fn_RMDMmask",
                                                           fname_RMDMmask);
                }

                if(FPS_mlat.SMfd > 0)
                {
                    float WFSfrequ = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                     ".framerateHz");

                    float latencyfr = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                      ".latencyfr");

                    functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                            ".timing.WFSfrequ",
                                                            WFSfrequ);
                    functionparameter_SetParamValue_FLOAT32(data.core.fpsptr,
                                                            ".timing.hardwlatfr",
                                                            latencyfr);
                }
                data.core.fpsptr->parray[fpi_update_RMDMmask].fpflag &= ~FPFLAG_ONOFF;
            }


            // Poke cube
            //
            // Compute action: make Spoke and Hpoke
            //
            long fpi_compPokeMat = 
                functionparameter_GetParamIndex(
                    data.core.fpsptr, ".compPokeMat");
            if(fpi_compPokeMat > -1 && (data.core.fpsptr->parray[fpi_compPokeMat].fpflag & FPFLAG_ONOFF))
            {

                imageID IDdmRM = image_ID(dmstream,
                    data.core.image,
                    data.core.NB_MAX_IMAGE);

                if(IDdmRM != -1)
                {

                    long fpi_Hpokemode = 
                        functionparameter_GetParamIndex(
                            data.core.fpsptr, ".Hpoke");
                    if(fpi_Hpokemode > -1 && (data.core.fpsptr->parray[fpi_Hpokemode].fpflag & FPFLAG_ONOFF))
                    {

                        AOloopControl_computeCalib_mkHadamardModes(
                            "RMDMmask",
                            "Hpoke");

                        fps_write_RUNoutput_image(data.core.fpsptr, "Hpoke", "Hpoke");

                        fps_write_RUNoutput_image(data.core.fpsptr, "Hpixindex", "Hpixindex");

                        fps_write_RUNoutput_image(data.core.fpsptr, "Hmat", "Hmat");

                        // create compressed files
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hmat.fits",
                                               data.core.fpsptr->md->datadir);
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hpixindex.fits",
                                               data.core.fpsptr->md->datadir);
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hpoke.fits",
                                               data.core.fpsptr->md->datadir);


                        {
                            // update poke file entry
                            //
                            char fname_Hpoke[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(fname_Hpoke,
                                               "./%s/Hpoke.fits",
                                               data.core.fpsptr->md->datadir);
                            functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                                   ".fn_pokeC",
                                                                   fname_Hpoke);
                        }


                    }
                    else
                    {
                        // simple poke matrix
                        //

                        mkSimpleZpokeM(DMxsize,
                                       DMysize,
                                       "Spoke");
                        fps_write_RUNoutput_image(data.core.fpsptr, "Spoke", "Spoke");

                        {
                            // update poke file entry
                            //
                            char fname_Spoke[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(fname_Spoke,
                                               "./%s/Spoke.fits",
                                               data.core.fpsptr->md->datadir);
                            functionparameter_SetParamValue_STRING(data.core.fpsptr,
                                                                   ".fn_pokeC",
                                                                   fname_Spoke);
                        }
                    }
                }

                data.core.fpsptr->parray[fpi_compPokeMat].fpflag &= ~FPFLAG_ONOFF;
            }
        }
    }

    return RETURN_SUCCESS;
}


// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {


        // Compute Poke Matrix if required
        //
        if(*compPokeMat)
        {
            printf("Computing Poke Matrix\n");

            imageID IDdmRM = image_ID(dmstream,
                data.core.image,
                data.core.NB_MAX_IMAGE);

            if(IDdmRM != -1)
            {
                uint32_t DMxsize = data.core.image[IDdmRM].md->size[0];
                uint32_t DMysize = data.core.image[IDdmRM].md->size[1];
                printf("DM size : %u x %u\n", DMxsize, DMysize);
            }
            else
            {
                DEBUG_TRACE_FEXIT("Cannot connect to DM");
                return RETURN_FAILURE;
            }


        }

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
CLIADDCMD_milk_AOloopControl_acquireCalib__acquireWFSlincalib()
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
    customCONFsetup,
    customCONFcheck)
#endif
