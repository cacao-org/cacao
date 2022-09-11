/**
 * @file acquireWFSlincalib.c
 * @brief Acquire linear WFS response
 *
 */

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


#include "AOloopControl_compTools/AOloopControl_compTools.h"
#include "computeCalib/computeHadamard.h"

#include "image_gen/image_gen.h"


// Local variables pointers
static uint32_t *AOloopindex;
static float    *pokeampl;



static char *dmstream;
static long fpi_dmstream;



// timing params

static long                      fpi_FPS_mlat = 0;
static FUNCTION_PARAMETER_STRUCT FPS_mlat;


// Toggles
static int64_t *update_mlat;
static long     fpi_update_mlat;


static int64_t *update_RMDMmask;
static long     fpi_update_RMDMmask;



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



static long                      fpi_FPS_DMcomb = 0;
static FUNCTION_PARAMETER_STRUCT FPS_DMcomb;

static uint32_t *DMMODE;
static long fpi_DMMODE;

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

static uint32_t *RMDMmaskCx;
static uint32_t *RMDMmaskCy;
static uint32_t *RMDMmaskR;

static uint64_t *normalize;

static uint64_t *Hpokemode;
static long fpi_Hpokemode;

static uint64_t *autotiming;
static long fpi_autotiming;


static uint64_t *compPokeMat;
static long fpi_compPokeMat;


// executable scripts

static char * exec_post_RMdecode;
static long fpi_exec_post_RMdecode;

static char * exec_post_mkDMWFSmasks;
static long fpi_exec_post_mkDMWFSmasks;

static char * exec_post_mkDMslaveact;
static long fpi_exec_post_mkDMslaveact;

static char * exec_post_mkLODMmodes;
static long fpi_exec_post_mkLODMmodes;



static CLICMDARGDEF farg[] = {
    {   CLIARG_UINT32,
        ".AOloopindex",
        "loop index",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {   CLIARG_STREAM,
        ".dmstream",
        "DM stream",
        "NULL",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmstream,
        &fpi_dmstream
    },
    {   CLIARG_FLOAT32,
        ".ampl",
        "RM poke amplitude",
        "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &pokeampl,
        NULL
    },
    // ============= TIMING =========================
    {   CLIARG_FPSNAME,
        ".timing.FPS_mlat",
        "hardware latency",
        "NULL",
        CLICMDARG_FLAG_NOCLI,
        FPTYPE_FPSNAME,
        FPFLAG_DEFAULT_INPUT,
        (void **) &FPS_mlat,
        &fpi_FPS_mlat
    },
    {   CLIARG_ONOFF,
        ".timing.upmlat",
        "update latency from FPS",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &update_mlat,
        &fpi_update_mlat
    },
    {   CLIARG_FLOAT32,
        ".timing.WFSfrequ",
        "WFS frame rate [Hz]",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &WFSfrequ,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".timing.hardwlatfr",
        "hardware latency [fr]",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &hardwlatfr,
        NULL
    },
    {   CLIARG_ONOFF,
        ".timing.autoTiming",
        "Auto Timing",
        "ON",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autotiming,
        &fpi_autotiming
    },
    {   CLIARG_UINT32,
        ".timing.delayfr",
        "frame delay, whole part",
        "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayfr,
        NULL
    },
    {   CLIARG_UINT32,
        ".timing.delayRM1us",
        "Sub-frame delay [us]",
        "100",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayRM1us,
        NULL
    },
    {   CLIARG_UINT32,
        ".timing.NBave",
        "Number of frames averaged for a single poke measurement",
        "5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBave,
        NULL
    },
    {   CLIARG_UINT32,
        ".timing.NBexcl",
        "Number of frames excluded",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBexcl,
        NULL
    },
    {   CLIARG_UINT32,
        ".timing.NBcycle",
        "Number of measurement cycles to be repeated",
        "10",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBcycle,
        NULL
    },
    {   CLIARG_UINT32,
        ".timing.NBinnerCycle",
        "Number of inner cycles (how many consecutive times should a single +/- "
        "poke be repeated)",
        "10",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBinnerCycle,
        NULL
    },
    {   CLIARG_ONOFF,
        ".timing.upmlat",
        "update latency from FPS",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &update_mlat,
        &fpi_update_mlat
    },
    // ============= RM DM MASK ======================
    {   CLIARG_FPSNAME,
        ".RMDMmask.FPS_DMcomb",
        "DM control process",
        "NULL",
        CLICMDARG_FLAG_NOCLI, FPTYPE_FPSNAME, FPFLAG_DEFAULT_INPUT,
        (void **) &FPS_DMcomb,
        &fpi_FPS_DMcomb
    },
    {   CLIARG_UINT32,
        ".RMDMmask.DMMODE",
        "0:spatial, 1:modal",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMMODE,
        &fpi_DMMODE
    },
    {   CLIARG_FLOAT32,
        ".RMDMmask.Cx",
        "X center",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &RMDMmaskCx,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".RMDMmask.Cy",
        "Y center",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &RMDMmaskCy,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".RMDMmask.R",
        "radius",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &RMDMmaskR,
        NULL
    },
    {   CLIARG_ONOFF,
        ".RMDMmask.upmlat",
        "update RMDMmask from FPS",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &update_RMDMmask,
        &fpi_update_RMDMmask
    },
    // ============= MASKiNG =========================
    {   CLIARG_ONOFF,
        ".MaskMode",
        "Mask mode, DM and WFS",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &MaskMode,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".DMmask.RMp0",
        "DM mask, point0 percentile point",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMp0,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".DMmask.RMc0",
        "DM mask, point0 coefficient",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMc0,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".DMmask.RMp1",
        "DM mask, point1 percentile point",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMp1,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".DMmask.RMc1",
        "DM mask, point1 coefficient",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskDMc1,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".DMmask.proxrad",
        "DM actuator proximity radius",
        "2.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMproxrad,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".WFSmask.RMp0",
        "WFS mask, point0 percentile point",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSp0,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".WFSmask.RMc0",
        "WFS mask, point0 coefficient",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSc0,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".WFSmask.RMp1",
        "WFS mask, point1 percentile point",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSp1,
        NULL
    },
    {   CLIARG_FLOAT32,
        ".WFSmask.RMc1",
        "WFS mask, point1 coefficient",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maskWFSc1,
        NULL
    },
    {   CLIARG_FITSFILENAME,
        ".fn_pokeC",
        "Poke sequence cube",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fn_pokeC,
        NULL
    },
    {   CLIARG_FITSFILENAME,
        ".fn_RMDMmask",
        "RM active DM actuators mask",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &fn_RMDMmask,
        NULL
    },
    {   CLIARG_ONOFF,
        ".normalize",
        "Normalize WFS frames",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &normalize,
        NULL
    },
    {   CLIARG_ONOFF,
        ".Hpoke",
        "Hadamard poke mode",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &Hpokemode,
        &fpi_Hpokemode
    },
    {   CLIARG_ONOFF,
        ".compPokeMat",
        "(re)compute poke matrix",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compPokeMat,
        &fpi_compPokeMat
    },
    // ============== executables ====================
    {
        CLIARG_FILENAME,
        ".exec.RMdecode",
        "RM decode script",
        "NULL",
        CLICMDARG_FLAG_NOCLI, FPTYPE_EXECFILENAME, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED,
        (void **) &exec_post_RMdecode,
        &fpi_exec_post_RMdecode
    },
    {
        CLIARG_FILENAME,
        ".exec.mkDMWFSmasks",
        "Make DM and WFS masks",
        "NULL",
        CLICMDARG_FLAG_NOCLI, FPTYPE_EXECFILENAME, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED,
        (void **) &exec_post_mkDMWFSmasks,
        &fpi_exec_post_mkDMWFSmasks
    },
    {
        CLIARG_FILENAME,
        ".exec.mkDMslaveact",
        "Make DM slaved actuators",
        "NULL",
        CLICMDARG_FLAG_NOCLI, FPTYPE_EXECFILENAME, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED,
        (void **) &exec_post_mkDMslaveact,
        &fpi_exec_post_mkDMslaveact
    },
    {
        CLIARG_FILENAME,
        ".exec.mkLODMmodes",
        "Make DM low order modes",
        "NULL",
        CLICMDARG_FLAG_NOCLI, FPTYPE_EXECFILENAME, FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED,
        (void **) &exec_post_mkLODMmodes,
        &fpi_exec_post_mkLODMmodes
    }
};




static CLICMDDATA CLIcmddata = {
    "acqWFSlincal", "acquire linear WFS calibration", CLICMD_FIELDS_DEFAULTS
};




static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
        // DM stream is required
        data.fpsptr->parray[fpi_dmstream].fpflag |= FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_FPS_mlat].fpflag &= ~FPFLAG_FPS_RUN_REQUIRED;

        data.fpsptr->parray[fpi_FPS_DMcomb].fpflag &= ~FPFLAG_FPS_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}








static errno_t customCONFcheck()
{
    if (data.fpsptr != NULL)
    {

        if (FPS_mlat.SMfd < 1)
        {
            printf("Connecting to mlat FPS\n");

            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_mlat,
                                                 &FPS_mlat);
        }


        if (FPS_DMcomb.SMfd < 1)
        {
            printf("Connecting to DMcomb FPS\n");

            functionparameter_ConnectExternalFPS(data.fpsptr,
                                                 fpi_FPS_DMcomb,
                                                 &FPS_DMcomb);
        }


        // update hardware latency
        //
        if (data.fpsptr->parray[fpi_update_mlat].fpflag & FPFLAG_ONOFF)
        {
            printf("Updating from mlat FPS\n");

            if (FPS_mlat.SMfd > 0)
            {
                float WFSfrequ = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                 ".framerateHz");

                float latencyfr = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                  ".latencyfr");

                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".timing.WFSfrequ",
                                                        WFSfrequ);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                        ".timing.hardwlatfr",
                                                        latencyfr);
            }
            data.fpsptr->parray[fpi_update_mlat].fpflag &= ~FPFLAG_ONOFF;
        }


        // Auto timing
        //
        if (data.fpsptr->parray[fpi_autotiming].fpflag & FPFLAG_ONOFF) // ON state
        {
            printf("UPDATE TIMING >>>>>>>>>\n");


            // long delayfr = (long) (1000000.0*latfr);

            // RMdelay = hardwlaten - 0.5 - excl/2
            double RMdelay =
                *hardwlatfr - 0.5 - 0.5 * (*NBexcl);

            int val_RMdelayfr =
                ((int) ((*hardwlatfr) - 0.5 -
                        0.5 * (*NBexcl) +
                        10.0)) +
                1 - 10;

            int val_delayRM1us = (int) ((1.0 * val_RMdelayfr - RMdelay) /
                                        (*WFSfrequ) * 1000000.0);

            if (RMdelay > 0)
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







        imageID IDdmRM = image_ID(dmstream);
        if(IDdmRM != -1)
        {
            uint32_t DMxsize = data.image[IDdmRM].md->size[0];
            uint32_t DMysize = data.image[IDdmRM].md->size[1];
            printf("DM size : %u x %u\n", DMxsize, DMysize);


            // update RM DM mask
            //
            if (data.fpsptr->parray[fpi_update_RMDMmask].fpflag & FPFLAG_ONOFF)
            {
                printf("Updating RM DM mask\n");


                uint32_t DMMODEin = 1;
                if (FPS_DMcomb.SMfd > 0)
                {
                    DMMODEin = functionparameter_GetParamValue_UINT32(&FPS_DMcomb,
                               ".DMmode");
                }

                // Update values
                //
                functionparameter_SetParamValue_UINT32(data.fpsptr, ".RMDMmask.DMMODE", DMMODEin);

                functionparameter_SetParamValue_FLOAT32(data.fpsptr, ".RMDMmask.Cx", 0.5*DMxsize);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr, ".RMDMmask.Cy", 0.5*DMysize);
                functionparameter_SetParamValue_FLOAT32(data.fpsptr, ".RMDMmask.R", 0.5*DMxsize+0.6);

                // load or create RMDMmask
                // this is the map of active actuatores to be poked in RM
                //
                {
                    char fnameRMDMmask[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(fnameRMDMmask,
                            functionparameter_GetParamPtr_STRING(data.fpsptr, ".sn_RMDMmask"),
                            FUNCTION_PARAMETER_STRMAXLEN);

                    imageID ID_RMDMmask;
                    load_fits(fnameRMDMmask, "RMDMmask", 1, &ID_RMDMmask);
                    if( ID_RMDMmask == -1)
                    {
                        // create it
                        //
                        if(*DMMODE == 0 )
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
                            for(uint64_t ii=0; ii<DMxsize*DMysize; ii++)
                            {
                                data.image[ID].array.F[ii] = 1.0;
                            }
                        }
                        fps_write_RUNoutput_image(data.fpsptr, "RMDMmask", "RMDMmask");
                    }

                    char fname_RMDMmask[STRINGMAXLEN_FULLFILENAME];
                    WRITE_FULLFILENAME(fname_RMDMmask,
                                       "./%s/RMDMmask.fits",
                                       data.fpsptr->md->datadir);
                    functionparameter_SetParamValue_STRING(data.fpsptr,
                                                           ".fn_RMDMmask",
                                                           fname_RMDMmask);
                }

                if (FPS_mlat.SMfd > 0)
                {
                    float WFSfrequ = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                     ".framerateHz");

                    float latencyfr = functionparameter_GetParamValue_FLOAT32(&FPS_mlat,
                                      ".latencyfr");

                    functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                            ".timing.WFSfrequ",
                                                            WFSfrequ);
                    functionparameter_SetParamValue_FLOAT32(data.fpsptr,
                                                            ".timing.hardwlatfr",
                                                            latencyfr);
                }
                data.fpsptr->parray[fpi_update_RMDMmask].fpflag &= ~FPFLAG_ONOFF;
            }







            // Poke cube
            //
            // Compute action: make Spoke and Hpoke
            //
            if (data.fpsptr->parray[fpi_compPokeMat].fpflag & FPFLAG_ONOFF)
            {

                imageID IDdmRM = image_ID(dmstream);

                if(IDdmRM != -1)
                {

                    if (data.fpsptr->parray[fpi_Hpokemode].fpflag & FPFLAG_ONOFF)
                    {

                        AOloopControl_computeCalib_mkHadamardModes(
                            "RMDMmask",
                            "Hpoke");

                        fps_write_RUNoutput_image(data.fpsptr, "Hpoke", "Hpoke");

                        fps_write_RUNoutput_image(data.fpsptr, "Hpixindex", "Hpixindex");

                        fps_write_RUNoutput_image(data.fpsptr, "Hmat", "Hmat");

                        // create compressed files
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hmat.fits",
                                               data.fpsptr->md->datadir);
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hpixindex.fits",
                                               data.fpsptr->md->datadir);
                        EXECUTE_SYSTEM_COMMAND("gzip -kf ./%s/Hpoke.fits",
                                               data.fpsptr->md->datadir);


                        {
                            // update poke file entry
                            //
                            char fname_Hpoke[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(fname_Hpoke,
                                               "./%s/Hpoke.fits",
                                               data.fpsptr->md->datadir);
                            functionparameter_SetParamValue_STRING(data.fpsptr,
                                                                   ".fn_pokeC",
                                                                   fname_Hpoke);
                        }


                    }
                    else
                    {
                        // simple poke matrix
                        //

                        AOloopControl_compTools_mkSimpleZpokeM(DMxsize,
                                                               DMysize,
                                                               "Spoke");
                        fps_write_RUNoutput_image(data.fpsptr, "Spoke", "Spoke");

                        {
                            // update poke file entry
                            //
                            char fname_Spoke[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(fname_Spoke,
                                               "./%s/Spoke.fits",
                                               data.fpsptr->md->datadir);
                            functionparameter_SetParamValue_STRING(data.fpsptr,
                                                                   ".fn_pokeC",
                                                                   fname_Spoke);
                        }
                    }
                }

                data.fpsptr->parray[fpi_compPokeMat].fpflag &= ~FPFLAG_ONOFF;
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


    // Compute Poke Matrix if required
    //
    if(*compPokeMat)
    {
        printf("Computing Poke Matrix\n");

        imageID IDdmRM = image_ID(dmstream);

        if(IDdmRM != -1)
        {
            uint32_t DMxsize = data.image[IDdmRM].md->size[0];
            uint32_t DMysize = data.image[IDdmRM].md->size[1];
            printf("DM size : %u x %u\n", DMxsize, DMysize);
        }
        else
        {
            DEBUG_TRACE_FEXIT("Cannot connect to DM");
            return RETURN_FAILURE;
        }


    }


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions




// Register function in CLI
errno_t
CLIADDCMD_milk_AOloopControl_acquireCalib__acquireWFSlincalib()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
