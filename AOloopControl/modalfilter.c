/**
 * @file    modalfilter.c
 * @brief   Apply modal filtering
 *
 *
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"
#include "CommandLineInterface/timeutils.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


// Local variables pointers
static uint64_t *AOloopindex;

static char *inmval;
static long  fpi_inmval;

static char *outmval;
static long  fpi_outmval;


// convenient loop on/off toggle
static int64_t *loopON;
static long     fpi_loopON;

// keep loop open for NBstep
// -1 is infinite
static int64_t *loopNBstep;
static long     fpi_loopNBstep;

// convenient loop on/off toggle
static int64_t *loopZERO;
static long     fpi_loopZERO;




static float *loopgain;
static long   fpi_loopgain;

static float *loopmult;
static long   fpi_loopmult;

static float *looplimit;
static long   fpi_looplimit;




// Compute open loop modes
static int64_t *compOL;
static long     fpi_compOL;

// Latency between DM and WFS
static float *latencyfr;
static long   fpi_latencyfr;


// Shared memory telemetry buffers
static int64_t *comptbuff;
static long     fpi_comptbuff;

static uint32_t *tbuffsize;




// Auxillary output modes to be mixed with std output

static uint64_t *auxDMmvalenable;
static long      fpi_auxDMmvalenable;

// mixing factoor
static float *auxDMmvalmixfact;
static long   fpi_auxDMmvalmixfact;

// modulation on/off
static uint64_t *auxDMmvalmodulate;
static long      fpi_auxDMmvalmodulate;

// modulation period
static float *auxDMmvalmodperiod;
static long   fpi_auxDMmvalmodperiod;

static uint64_t *enablePF;
static long      fpi_enablePF;


// nummber of prediction blocks to wait for
// this is by how much modevalOL counter should increment
//
static uint32_t *PF_NBblock;
static long      fpi_PF_NBblock;

// how long to wait for PF blocks ?
static uint32_t *PF_maxwaitus;
static long      fpi_PF_maxwaitus;

// PF mixing coeff
static float *PFmixcoeff;
static long   fpi_PFmixcoeff;




// autoloop self-test : write output back to input
static uint64_t *autoloopenable;
static long      fpi_autoloopenable;

static float *autoloopsleep;
static long   fpi_autoloopsleep;




// self-test: modal response matrix
static uint64_t *selfRMenable;
static long      fpi_selfRMenable;

// Number modes
static uint32_t *selfRMnbmode;
static long   fpi_selfRMnbmode;

// poke amplitude
static float *selfRMpokeampl;
static long   fpi_selfRMpokeampl;

// number of time stamp recorded
static uint32_t *selfRMzsize;
static long      fpi_selfRMzsize;

// number of iterations averaged
static uint32_t *selfRMnbiter;
static long      fpi_selfRMnbiter;

// time to settle after poke
static uint32_t *selfRMnbsettlestep;
static long      fpi_selfRMnbsettlestep;
;




static CLICMDARGDEF farg[] =
{
    {
        // AO loop index. Used for naming streams aolX_
        CLIARG_UINT64,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {
        CLIARG_STREAM,
        ".inmval",
        "input mode values from WFS",
        "aol0_modevalWFS",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inmval,
        &fpi_inmval
    },
    {
        CLIARG_STREAM,
        ".outmval",
        "output mode values to DM",
        "aol0_modevalDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outmval,
        &fpi_outmval
    },
    {
        CLIARG_ONOFF,
        ".loopON",
        "loop on/off (off=freeze)",
        "ON",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopON,
        &fpi_loopON
    },
    {
        CLIARG_INT64,
        ".loopNBstep",
        "loop nb steps (-1 = inf)",
        "-1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopNBstep,
        &fpi_loopNBstep
    },
    {
        CLIARG_ONOFF,
        ".loopZERO",
        "loop zero",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopZERO,
        &fpi_loopZERO
    },
    {
        CLIARG_FLOAT32,
        ".loopgain",
        "loop gain",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopgain,
        &fpi_loopgain
    },
    {
        CLIARG_FLOAT32,
        ".loopmult",
        "loop mult",
        "0.95",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopmult,
        &fpi_loopmult
    },
    {
        CLIARG_FLOAT32,
        ".looplimit",
        "loop limit",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &looplimit,
        &fpi_looplimit
    },
    {
        CLIARG_ONOFF,
        ".comp.OLmodes",
        "compute open loop modes",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compOL,
        &fpi_compOL
    },
    {
        CLIARG_FLOAT32,
        ".comp.latencyfr",
        "DM to WFS latency [frame]",
        "1.7",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &latencyfr,
        &fpi_latencyfr
    },
    {
        CLIARG_ONOFF,
        ".comp.tbuff",
        "compute telemetry buffer(s)",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &comptbuff,
        &fpi_comptbuff
    },
    {
        CLIARG_UINT32,
        ".comp.tbuffsize",
        "buffer time size",
        "512",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &tbuffsize,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".auxDMmval.enable",
        "mixing aux DM mode vals from stream aolx_modevalauxDM ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &auxDMmvalenable,
        &fpi_auxDMmvalenable
    },
    {
        CLIARG_FLOAT32,
        ".auxDMmval.mixfact",
        "mixing multiplicative factor (0:no mixing)",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &auxDMmvalmixfact,
        &fpi_auxDMmvalmixfact
    },
    {
        CLIARG_ONOFF,
        ".auxDMmval.modulate",
        "modulate auxDM temporally ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &auxDMmvalmodulate,
        &fpi_auxDMmvalmodulate
    },
    {
        CLIARG_FLOAT32,
        ".auxDMmval.modperiod",
        "auxDM modulation period [frame]",
        "20.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &auxDMmvalmodperiod,
        &fpi_auxDMmvalmodperiod
    },
    {
        // enable predictive filter, listen to aolX_modevalPF
        CLIARG_ONOFF,
        ".PF.enable",
        "enable predictive filter",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &enablePF,
        &fpi_enablePF
    },
    {
        // enable predictive filter, listen to aolX_modevalPF
        CLIARG_UINT32,
        ".PF.NBblock",
        "number of blocks to wait from",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &PF_NBblock,
        &fpi_PF_NBblock
    },
    {
        // enable predictive filter, listen to aolX_modevalPF
        CLIARG_UINT32,
        ".PF.maxwaitus",
        "maximum wait time for blocks [us]",
        "500",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &PF_maxwaitus,
        &fpi_PF_maxwaitus
    },
    {
        // predictive filter mult coeff
        CLIARG_FLOAT32,
        ".PF.mixcoeff",
        "mixing coeff",
        "0.3",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &PFmixcoeff,
        &fpi_PFmixcoeff
    },
    {
        CLIARG_ONOFF,
        ".autoloop.enable",
        "autoloop self-test: loop back output to input ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autoloopenable,
        &fpi_autoloopenable
    },
    {
        CLIARG_FLOAT32,
        ".autoloop.sleep",
        "loop sleep time",
        "0.001",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autoloopsleep,
        &fpi_autoloopsleep
    },
    {
        CLIARG_ONOFF,
        ".selfRM.enable",
        "Start self response matrix measurement",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMenable,
        &fpi_selfRMenable
    },
    {
        CLIARG_UINT32,
        ".selfRM.NBmode",
        "number of mode poked",
        "32",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMnbmode,
        &fpi_selfRMnbmode
    },
    {
        CLIARG_FLOAT32,
        ".selfRM.pokeampl",
        "poke amplitude",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMpokeampl,
        &fpi_selfRMpokeampl
    },
    {
        CLIARG_UINT32,
        ".selfRM.zsize",
        "number of time steps recorded",
        "6",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMzsize,
        &fpi_selfRMzsize
    },
    {
        CLIARG_UINT32,
        ".selfRM.nbiter",
        "number of iterations averaged, ideally 8n",
        "8",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMnbiter,
        &fpi_selfRMnbiter
    },
    {
        CLIARG_UINT32,
        ".selfRM.nbsettle",
        "number of loop iteration to settle between pokes",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selfRMnbsettlestep,
        &fpi_selfRMnbsettlestep
    }
};




// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_inmval].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;


        data.fpsptr->parray[fpi_loopON].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopZERO].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopNBstep].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_looplimit].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_comptbuff].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compOL].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_latencyfr].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_auxDMmvalenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmixfact].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmodulate].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmodperiod].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_enablePF].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_PF_NBblock].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_PF_maxwaitus].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_autoloopenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_autoloopsleep].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_selfRMenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbmode].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbsettlestep].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMpokeampl].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbiter].fpflag |= FPFLAG_WRITERUN;
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
    "modalfilter", "modal filtering", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{
    printf("Modal gain for adaptive optics control\n");


    printf(
        "Main input/output streams :\n"
        "[STREAM]   <.inmval>    input mode values\n"
        "[STREAM]   <.outmval>   output mode values\n");

    printf(
        "Auxillary input, added to output\n"
        "[STREAM]  aolx_modevalauxDM  auxillary mode values\n"
        "If enabled, add mode values to output");

    return RETURN_SUCCESS;
}




/**
 * @brief Modal filtering AO processing
 *
 * Basic modal control. Each mode controlled independently.
 *
 * Control parameters for each mode are:
 * - (g) gain
 * - (m) mult
 * - (z) zeropt
 * - (l) limit
 *
 * PROCESSING
 * Output (o) is computed from input (i) according to following steps :
 *
 * Apply gain :
 * o += (z-i)*g
 *
 * Apply mult :
 * o = z + m*(o-z)
 *
 * Apply limit:
 * if o>z+l -> o = z+l
 * if o<z-l -> o = z-l
 *
 *
 *
 * @return errno_t
 */
static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to input mode values array and get number of modes
    //
    IMGID imgin = mkIMGID_from_name(inmval);
    resolveIMGID(&imgin, ERRMODE_ABORT);
    printf("%u modes\n", imgin.md->size[0]);
    uint32_t NBmode = imgin.md->size[0];


    int selfRM_NBmode = (*selfRMnbmode);
    if(selfRM_NBmode > NBmode)
    {
        selfRM_NBmode = NBmode;
    }
    // selfRM initialization
    //
    int      blockcnt         = 0;
    int      selfRMpokeparity = 0; // 0 or 1
    uint32_t selfRMiter       = 0;
    uint32_t selfRM_pokemode  = 0;
    uint32_t selfRM_pokecnt   = 0;
    float    selfRMpokesign   = 1.0;
    // create selfRM image
    //
    IMGID imgselfRM;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mfiltselfRM", *AOloopindex);
        imgselfRM =
            stream_connect_create_3Df32(name, NBmode, NBmode, (*selfRMzsize));
        for(uint32_t mi = 0; mi < NBmode * NBmode * (*selfRMzsize); mi++)
        {
            data.image[imgselfRM.ID].array.F[mi] = 0.0;
        }
    }
    float *selfRMpokecmd = (float *) malloc(sizeof(float) * NBmode);
    for(uint32_t mi = 0; mi < NBmode; mi++)
    {
        selfRMpokecmd[mi] = 0.0;
    }

    // allocate memory for temporary output mode values

    // current control values
    float *mvalDMc = (float *) malloc(sizeof(float) * NBmode);
    for(uint32_t mi; mi < NBmode; mi++)
    {
        mvalDMc[mi] = 0.0;
    }

    // output (computed)
    float *mvalout = (float *) malloc(sizeof(float) * NBmode);
    // output (applied, includes selfRM poke)
    float *mvaloutapply = (float *) malloc(sizeof(float) * NBmode);



    // OPEN LOOP MODE VALUES
    //
    // allocate memory for DM modes history
    int    NB_DMtstep = 10; // history buffer size
    int    DMtstep    = 0;  // current index
    float *mvalDMbuff = (float *) malloc(sizeof(float) * NBmode * NB_DMtstep);

    IMGID imgOLmval;
    {
        char OLmvalname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(OLmvalname, "aol%lu_modevalOL", *AOloopindex);
        imgOLmval = stream_connect_create_2Df32(OLmvalname, NBmode, 1);
    }



    // TELEMETRY BUFFERS
    //
    uint32_t tbuffindex = 0;
    int      tbuffslice = 0;
    IMGID    imgtbuff_mvalDM;
    IMGID    imgtbuff_mvalWFS;
    IMGID    imgtbuff_mvalOL;
    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name, "aol%lu_modevalDM_buff", *AOloopindex);
        imgtbuff_mvalDM =
            stream_connect_create_3Df32(name, NBmode, (*tbuffsize), 2);

        WRITE_IMAGENAME(name, "aol%lu_modevalWFS_buff", *AOloopindex);
        imgtbuff_mvalWFS =
            stream_connect_create_3Df32(name, NBmode, (*tbuffsize), 2);

        WRITE_IMAGENAME(name, "aol%lu_modevalOL_buff", *AOloopindex);
        imgtbuff_mvalOL =
            stream_connect_create_3Df32(name, NBmode, (*tbuffsize), 2);
    }


    // connect/create output mode coeffs
    IMGID imgout = stream_connect_create_2Df32(outmval, NBmode, 1);
    for(uint32_t mi = 0; mi < NBmode; mi++)
    {
        data.image[imgout.ID].array.F[mi] = 0.0;
    }



    // connect/create aux DM control mode coeffs
    //
    IMGID imgauxmDM;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_modevalauxDM", *AOloopindex);
        imgauxmDM = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            data.image[imgauxmDM.ID].array.F[mi] = 0.0;
        }
    }



    // connect/create predictive filter (PF) control mode coeffs
    //
    IMGID imgPF;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_modevalPF", *AOloopindex);
        imgPF = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgPF.im->array.F[mi] = 0.0;
        }
    }




    // ========================= MODAL GAIN ===========================
    printf("Setting up modal gain\n");

    IMGID imgmgain;
    {
        char mgainname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mgainname, "aol%lu_mgain", *AOloopindex);
        imgmgain = stream_connect_create_2Df32(mgainname, NBmode, 1);
    }
    list_image_ID();
    printf(" mgain ID = %ld\n", imgmgain.ID);
    fflush(stdout);

    // modal gains factors
    // to be multiplied by overal gain to become mgain
    // allows for single-parameter gain tuning
    IMGID imgmgainfact;
    {
        char mgainfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mgainfactname, "aol%lu_mgainfact", *AOloopindex);
        imgmgainfact = stream_connect_create_2Df32(mgainfactname, NBmode, 1);
        printf("%s  ID = %ld\n", imgmgainfact.name, imgmgainfact.ID);
        list_image_ID();
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmgainfact.im->array.F[mi] = 1.0;
        }
    }



    // ========================= MODAL MULT ==========================
    printf("Setting up modal mult\n");

    IMGID imgmmult;
    {
        char mmultname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mmultname, "aol%lu_mmult", *AOloopindex);
        imgmmult = stream_connect_create_2Df32(mmultname, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmmultfact;
    {
        char mmultfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mmultfactname, "aol%lu_mmultfact", *AOloopindex);
        imgmmultfact = stream_connect_create_2Df32(mmultfactname, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmmultfact.im->array.F[mi] = 1.0;
        }
    }


    // ========================= MODAL ZEROPOINT ==========================
    printf("Setting up modal zero point\n");

    IMGID imgmzeropoint;
    {
        char mzeropointname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mzeropointname, "aol%lu_mzeropoint", *AOloopindex);
        imgmzeropoint = stream_connect_create_2Df32(mzeropointname, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmzeropoint.im->array.F[mi] = 0.0;
        }
    }



    // ========================= MODAL LIMIT ==========================
    printf("Setting up modal limit\n");

    IMGID imgmlimit;
    {
        char mlimitname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mlimitname, "aol%lu_mlimit", *AOloopindex);
        imgmlimit = stream_connect_create_2Df32(mlimitname, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmlimitfact;
    {
        char mlimitfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mlimitfactname, "aol%lu_mlimitfact", *AOloopindex);
        imgmlimitfact = stream_connect_create_2Df32(mlimitfactname, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmlimitfact.im->array.F[mi] = 1.0;
        }
    }



    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        // zero loop
        if(data.fpsptr->parray[fpi_loopZERO].fpflag & FPFLAG_ONOFF)
        {

            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                // set goal position to zero
                mvalDMc[mi] = 0.0;

                mvalout[mi]      = 0.0;
                mvaloutapply[mi] = 0.0;
            }

            memcpy(imgout.im->array.F, mvaloutapply, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imgout.ID);

            // toggle back to OFF
            data.fpsptr->parray[fpi_loopZERO].fpflag &= ~FPFLAG_ONOFF;
        }




        if((*loopON) == 1)
        {
            if(*loopNBstep > 0)
            {
                *loopNBstep                                    = *loopNBstep - 1;
                data.fpsptr->parray[fpi_loopNBstep].val.i64[0] = *loopNBstep;
            }
            if(*loopNBstep == 0)
            {
                *loopON = 0;
                // set loop to OFF
                data.fpsptr->parray[fpi_loopON].fpflag &= ~FPFLAG_ONOFF;
                *loopNBstep = 1;
            }


            // Pre-allocations for modal loop
            double mvalWFS;
            double dmval;
            float  limit;


            float auxDMfact = (*auxDMmvalmixfact);
            if((*auxDMmvalmodulate) == 1)
            {
                static double modpha = 0.0;
                modpha += 1 / (*auxDMmvalmodperiod);
                if(modpha > 1.0)
                {
                    modpha -= 1.0;
                }

                auxDMfact *= sin(2.0 * M_PI * modpha);
                //printf("%7.5f  auxDMfact = %7.5f\n", modpha, auxDMfact);
            }

            // Apply modal control filtering
            //
            for(uint32_t mi = 0; mi < NBmode; mi++)
            {

                // grab input value from WFS
                mvalWFS = imgin.im->array.F[mi];

                // offset from mval to zero point
                // this is the input zero point
                dmval = imgmzeropoint.im->array.F[mi] - mvalWFS;

                // multiply by GAIN
                dmval *= imgmgain.im->array.F[mi];


                // this is the goal position
                mvalDMc[mi] += dmval;

                // multiply goal position by MULT
                mvalDMc[mi] *= imgmmult.im->array.F[mi];

                // apply LIMIT
                limit = imgmlimit.im->array.F[mi];
                if(mvalDMc[mi] > limit)
                {
                    mvalDMc[mi] = limit;
                }
                if(mvalDMc[mi] < -limit)
                {
                    mvalDMc[mi] = -limit;
                }


                if((*auxDMmvalenable) == 1)
                {
                    // add mode values from aux stream
                    mvalout[mi] =
                        mvalDMc[mi] + (auxDMfact * imgauxmDM.im->array.F[mi]);
                }
                else
                {
                    mvalout[mi] = mvalDMc[mi];
                }
                mvaloutapply[mi] = mvalout[mi] + selfRMpokecmd[mi];
            }



            if(*enablePF == 0)
            {
                memcpy(imgout.im->array.F, mvaloutapply, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgout.ID);
            }


            // Compute pseudo open-loop mode coefficients
            //
            if((*compOL) == 1)
            {
                // write to DM history
                //
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    mvalDMbuff[DMtstep * NBmode + mi] = mvalDMc[mi];
                }


                DMtstep++;
                if(DMtstep == NB_DMtstep)
                {
                    DMtstep = 0;
                }

                int   latint  = (int)(*latencyfr);
                float latfrac = (*latencyfr) - latint;

                int DMtstep1 = DMtstep - latint;
                int DMtstep0 = DMtstep1 - 1;
                while(DMtstep1 < 0)
                {
                    DMtstep1 += NB_DMtstep;
                }
                while(DMtstep0 < 0)
                {
                    DMtstep0 += NB_DMtstep;
                }

                imgOLmval.md->write = 1;
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpmDMval = latfrac * mvalDMbuff[DMtstep0 * NBmode + mi];
                    tmpmDMval +=
                        (1.0 - latfrac) * mvalDMbuff[DMtstep1 * NBmode + mi];

                    float tmpmWFSval = imgin.im->array.F[mi];
                    ;

                    imgOLmval.im->array.F[mi] = tmpmWFSval - tmpmDMval;
                }

                uint64_t PFcnt = imgPF.md->cnt0;

                processinfo_update_output_stream(processinfo, imgOLmval.ID);


                if(*enablePF == 1)
                {
                    // wait for PF blocks to complete
                    //

                    struct timespec t0;
                    struct timespec t1;
                    clock_gettime(CLOCK_MILK, &t0);
                    clock_gettime(CLOCK_MILK, &t1);
                    uint64_t PFcntOK = PFcnt + *PF_NBblock;
                    while(
                        (imgPF.md->cnt0 < PFcntOK) &&
                        (timespec_diff_double(t0, t1) < 1.0e-6 * (*PF_maxwaitus)))
                    {
                        // busy waiting
                        clock_gettime(CLOCK_MILK, &t1);
                    }

                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        mvalout[mi] = imgPF.im->array.F[mi] * (*PFmixcoeff) +
                                      mvalout[mi] * (1.0 - *PFmixcoeff);
                        mvaloutapply[mi] = mvalout[mi] + selfRMpokecmd[mi];
                    }

                    memcpy(imgout.im->array.F,
                           mvaloutapply,
                           sizeof(float) * NBmode);
                    processinfo_update_output_stream(processinfo, imgout.ID);
                }
            }




            // Update individual gain, mult and limit values
            // This is done AFTER computing mode values to minimize latency
            //
            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgmgain.im->array.F[mi] =
                    imgmgainfact.im->array.F[mi] * (*loopgain);
            }
            processinfo_update_output_stream(processinfo, imgmgain.ID);


            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgmmult.im->array.F[mi] =
                    imgmmultfact.im->array.F[mi] * (*loopmult);
            }
            processinfo_update_output_stream(processinfo, imgmmult.ID);


            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgmlimit.im->array.F[mi] =
                    imgmlimitfact.im->array.F[mi] * (*looplimit);
            }
            processinfo_update_output_stream(processinfo, imgmlimit.ID);


            // Fill telemetry buffers
            //
            if((*comptbuff) == 1)
            {

                uint64_t kkoffset =
                    tbuffslice * (*tbuffsize) * NBmode + tbuffindex * NBmode;
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    imgtbuff_mvalWFS.im->array.F[kkoffset + mi] =
                        imgin.im->array.F[mi];
                    imgtbuff_mvalOL.im->array.F[kkoffset + mi] =
                        imgOLmval.im->array.F[mi];
                }


                if((*auxDMmvalenable) == 1)
                {
                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgtbuff_mvalDM.im->array.F[kkoffset + mi] =
                            mvalout[mi] - (auxDMfact * imgauxmDM.im->array.F[mi]);
                    }
                }
                else
                {
                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgtbuff_mvalDM.im->array.F[kkoffset + mi] = mvalout[mi];
                    }
                }


                tbuffindex++;
                if(tbuffindex == (*tbuffsize))
                {
                    tbuffindex = 0;

                    imgtbuff_mvalDM.md->cnt1 = tbuffslice;
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalDM.ID);

                    imgtbuff_mvalWFS.md->cnt1 = tbuffslice;
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalWFS.ID);

                    imgtbuff_mvalOL.md->cnt1 = tbuffslice;
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalOL.ID);

                    tbuffslice++;
                    if(tbuffslice == 2)
                    {
                        tbuffslice = 0;
                    }
                }
            }
        }



        if(*autoloopenable == 1)
        {
            // write output back to input
            //
            struct timespec twait;

            double x = *autoloopsleep;
            x += 0.5e-9; // minimize rounding error
            twait.tv_sec  = (long) x;
            twait.tv_nsec = (x - twait.tv_sec) * 1000000000L;

            nanosleep(&twait, NULL);

            memcpy(imgin.im->array.F, imgout.im->array.F, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imgin.ID);
        }




        if(*selfRMenable == 1)
        {
            // initialization
            if((selfRM_pokecnt == 0) && (selfRM_pokemode == 0) &&
                    (blockcnt == 0) && (selfRMiter == 0))
            {
                processinfo_WriteMessage(processinfo, "init selfRM");
                printf("INITIALIZING selfRM\n");
                for(uint32_t mi = 0; mi < NBmode * NBmode * (*selfRMzsize); mi++)
                {
                    data.image[imgselfRM.ID].array.F[mi] = 0.0;
                }
            }

            if((selfRM_pokecnt == 0) && (selfRM_pokemode == 0))
            {
                if(blockcnt == 0)
                {
                    // start poke sign
                    selfRMpokesign = 1.0 - 2.0 * ((selfRMiter / 4) % 2);
                }

                processinfo_WriteMessage_fmt(processinfo,
                                             "sRM %u/%u  ppol %d  sign "
                                             "%+3.1f ",
                                             selfRMiter,
                                             *selfRMnbiter,
                                             selfRMpokeparity,
                                             selfRMpokesign);
            }



            int pkmode = 0;
            pkmode     = selfRM_pokemode;

            float signmult;
            if((selfRM_pokemode % 2 == 0) && (selfRMpokeparity == 1))
            {
                signmult = -1.0;
            }
            else
            {
                signmult = 1.0;
            }


            if(selfRM_pokecnt < *selfRMzsize)
            {
                selfRMpokecmd[pkmode] =
                    selfRMpokesign * (*selfRMpokeampl) * signmult;
            }
            else
            {
                selfRMpokecmd[pkmode] = 0.0;
            }


            if(selfRM_pokecnt < *selfRMzsize)
            {
                // write result in output 3D selfRM
                //
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    long pindex = NBmode * NBmode * selfRM_pokecnt;
                    pindex += NBmode * pkmode;
                    pindex += mi;
                    imgselfRM.im->array.F[pindex] +=
                        0.5 * signmult * selfRMpokesign * imgin.im->array.F[mi] /
                        (*selfRMpokeampl) / (*selfRMnbiter);
                }
            }
            selfRM_pokecnt++;

            if(selfRM_pokecnt > (*selfRMzsize) + (*selfRMnbsettlestep))
            {

                if(((selfRMiter % 8) < 2) || ((selfRMiter % 8) > 5))
                {
                    selfRMpokesign *= -1.0;
                }

                blockcnt++;
                if(blockcnt == 2)
                {
                    selfRM_pokemode++;
                    blockcnt = 0;
                }
                selfRM_pokecnt = 0;
            }


            if(selfRM_pokemode == selfRM_NBmode)
            {
                selfRMpokeparity = 1 - selfRMpokeparity;
                selfRM_pokemode  = 0;
                selfRM_pokecnt   = 0;

                selfRMiter++;
            }

            if(selfRMiter == *selfRMnbiter)
            {

                selfRMiter      = 0;
                selfRM_pokemode = 0;
                selfRM_pokecnt  = 0;

                data.fpsptr->parray[fpi_selfRMenable].fpflag &= ~FPFLAG_ONOFF;
                *selfRMenable = 0;

                // testing
                save_fits(imgselfRM.name, "selfRM.fits");

                processinfo_WriteMessage(processinfo, "selfRM done");
            }
        }


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(selfRMpokecmd);

    free(mvalout);
    free(mvaloutapply);
    free(mvalDMc);
    free(mvalDMbuff);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__modalfilter()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
