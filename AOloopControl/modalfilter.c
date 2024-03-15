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

// amplitude correction factor on WFS signal
static float    *psol_WFSfact;
static long     fpi_psol_WFSfact;


// Latency between DM and WFS
static float *latencyhardwfr;
static long   fpi_latencyhardwfr;

// software latency (usually around 1.5 frame)
// 1 frame + compute time
static float *latencysoftwfr;
static long   fpi_latencysoftwfr;


// Shared memory telemetry buffers
static int64_t *comptbuff;
static long     fpi_comptbuff;

static uint32_t *tbuffsize;


// autoset modal limits

// toggle ON / OFF
static uint64_t *autolim;
static long      fpi_autolim;

// gain on sigma probing
static float *autolimprobegain;
static long   fpi_autolimprobegain;

// sigma-clipping factor
static float *autolimsigmafact;
static long   fpi_autolimsigmafact;








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





static uint64_t *testOL;
static long      fpi_testOL;

static float *testOLupdategain;
static long      fpi_testOLupdategain;


static uint32_t *testOLmode;
static long      fpi_testOLmode;

static float *testOLampl;
static long      fpi_testOLampl;

static uint32_t *testOLnbsample;
static long      fpi_testOLnbsample;

static uint32_t *testOLcnt;
static long      fpi_testOLcnt;



// offload modal output
//
static uint64_t *offload;
static long      fpi_offload;

static float *offloadloopgain;
static long   fpi_offloadloopgain;

static float *offloadloopmult;
static long   fpi_offloadloopmult;

static float *offloadlooplimit;
static long   fpi_offloadlooplimit;





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
        ".comp.WFSfact",
        "amplitude correction factor on WFS",
        "0.893",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &psol_WFSfact,
        &fpi_psol_WFSfact
    },
    {
        CLIARG_FLOAT32,
        ".comp.latencyhardwfr",
        "hardware DM to WFS latency [frame]",
        "1.7",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &latencyhardwfr,
        &fpi_latencyhardwfr
    },
    {
        CLIARG_FLOAT32,
        ".comp.latencysoftwfr",
        "software latency [frame]",
        "1.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &latencysoftwfr,
        &fpi_latencysoftwfr
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
        CLIARG_ONOFF,
        ".comp.autolim",
        "automatic modal limits",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autolim,
        &fpi_autolim
    },
    {
        CLIARG_FLOAT32,
        ".comp.autolimprobegain",
        "sigma measurement gain",
        "0.1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autolimprobegain,
        &fpi_autolimprobegain
    },
    {
        CLIARG_FLOAT32,
        ".comp.autolimsigmafact",
        "autolimit sigma clipping factor",
        "2.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autolimsigmafact,
        &fpi_autolimsigmafact
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
    },
    {
        CLIARG_ONOFF,
        ".testOL.enable",
        "OL reconstruction test ON/OFF",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOL,
        &fpi_testOL
    },
    {
        CLIARG_FLOAT32,
        ".testOL.updategain",
        "update gain (0=check only)",
        "0.1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOLupdategain,
        &fpi_testOLupdategain
    },
    {
        CLIARG_UINT32,
        ".testOL.mode",
        "mode index",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOLmode,
        &fpi_testOLmode
    },
    {
        CLIARG_FLOAT32,
        ".testOL.ampl",
        "amplitude",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOLampl,
        &fpi_testOLampl
    },
    {
        CLIARG_UINT32,
        ".testOL.nbsample",
        "number of samples",
        "1000",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOLnbsample,
        &fpi_testOLnbsample
    },
    {
        CLIARG_UINT32,
        ".testOL.cnt",
        "samples count",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &testOLcnt,
        &fpi_testOLcnt
    },
    {
        CLIARG_ONOFF,
        ".offload.enable",
        "offload output ON/OFF",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &offload,
        &fpi_offload
    },
    {
        CLIARG_FLOAT32,
        ".offload.loopgain",
        "offload loop gain",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &offloadloopgain,
        &fpi_offloadloopgain
    },
    {
        CLIARG_FLOAT32,
        ".offload.loopmult",
        "offload loop mult",
        "0.95",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &offloadloopmult,
        &fpi_offloadloopmult
    },
    {
        CLIARG_FLOAT32,
        ".offload.looplimit",
        "offload loop limit",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &offloadlooplimit,
        &fpi_offloadlooplimit
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
        data.fpsptr->parray[fpi_psol_WFSfact].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_latencyhardwfr].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_latencysoftwfr].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_autolim].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_autolimprobegain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_autolimsigmafact].fpflag |= FPFLAG_WRITERUN;


        data.fpsptr->parray[fpi_auxDMmvalenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmixfact].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmodulate].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_auxDMmvalmodperiod].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_enablePF].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_PF_NBblock].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_PF_maxwaitus].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_PFmixcoeff].fpflag |= FPFLAG_WRITERUN;


        data.fpsptr->parray[fpi_autoloopenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_autoloopsleep].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_selfRMenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbmode].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbsettlestep].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMpokeampl].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_selfRMnbiter].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_testOL].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_testOLupdategain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_testOLampl].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_testOLmode].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_testOLnbsample].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_offload].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_offloadloopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_offloadloopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_offloadlooplimit].fpflag |= FPFLAG_WRITERUN;
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
    IMGID imginWFS = mkIMGID_from_name(inmval);
    resolveIMGID(&imginWFS, ERRMODE_ABORT);
    printf("%u modes\n", imginWFS.md->size[0]);
    uint32_t NBmode = imginWFS.md->size[0];


    int selfRM_NBmode = (*selfRMnbmode);
    if(selfRM_NBmode > (int) NBmode)
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
        imgselfRM = stream_connect_create_3Df32(name, NBmode, NBmode, (*selfRMzsize));
        for(uint32_t mi = 0; mi < NBmode * NBmode * (*selfRMzsize); mi++)
        {
            data.image[imgselfRM.ID].array.F[mi] = 0.0;
        }

        ImageStreamIO_UpdateIm(imgselfRM.im);
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
    float *mvalDMOL = (float*) malloc(sizeof(float)*NBmode);

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
    //
    IMGID imgout = stream_connect_create_2Df32(outmval, NBmode, 1);
    {
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            data.image[imgout.ID].array.F[mi] = 0.0;
        }

        ImageStreamIO_UpdateIm(imgout.im);
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
        ImageStreamIO_UpdateIm(imgauxmDM.im);
    }




    // ======================= PRDICTIVE FILTER ==========================

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
        ImageStreamIO_UpdateIm(imgPF.im);
    }


    IMGID imgmPFmix;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mPFmix", *AOloopindex);
        imgmPFmix = stream_connect_create_2Df32(name, NBmode, 1);
    }


    IMGID imgmPFmixfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mPFmixfact", *AOloopindex);
        imgmPFmixfact = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmPFmixfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgmPFmixfact.im);
    }



    // PREDICTIVE RECONSTRUCTION SMALL CIRCULAR TELEMETRY BUFFERS
    //
    IMGID imgcbuff_mvalPF;
    uint32_t PFcbuff_index = 0;
    int      PFcbuff_size = 20;
    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name, "aol%lu_modevalPF_cbuff", *AOloopindex);
        imgcbuff_mvalPF =
            stream_connect_create_2Df32(name, NBmode, PFcbuff_size);
    }











    // connect/create output offload mode coeffs to DM
    //
    IMGID imgmvaloffloadDM;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvaloffloadDM", *AOloopindex);
        imgmvaloffloadDM = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmvaloffloadDM.im->array.F[mi] = 0.0;
        }
        ImageStreamIO_UpdateIm(imgmvaloffloadDM.im);
    }






    // ========================= MODAL GAIN ===========================
    printf("Setting up modal gain\n");

    IMGID imgmgain;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mgain", *AOloopindex);
        imgmgain = stream_connect_create_2Df32(name, NBmode, 1);
    }
    list_image_ID();
    printf(" mgain ID = %ld\n", imgmgain.ID);
    fflush(stdout);

    // modal gains factors
    // to be multiplied by overal gain to become mgain
    // allows for single-parameter gain tuning
    IMGID imgmgainfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mgainfact", *AOloopindex);
        imgmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
        printf("%s  ID = %ld\n", imgmgainfact.name, imgmgainfact.ID);
        list_image_ID();
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmgainfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgmgainfact.im);
    }


    IMGID imgoffloadmgain;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmgain", *AOloopindex);
        imgoffloadmgain = stream_connect_create_2Df32(name, NBmode, 1);
    }
    list_image_ID();
    printf(" offloadmgain ID = %ld\n", imgoffloadmgain.ID);
    fflush(stdout);

    // offload modal gains factors
    // to be multiplied by overal gain to become offloadmgain
    // allows for single-parameter gain tuning
    IMGID imgoffloadmgainfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmgainfact", *AOloopindex);
        imgoffloadmgainfact = stream_connect_create_2Df32(name, NBmode, 1);
        printf("%s  ID = %ld\n", imgoffloadmgainfact.name, imgoffloadmgainfact.ID);
        list_image_ID();
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgoffloadmgainfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgoffloadmgainfact.im);
    }











    // ========================= MODAL MULT ==========================
    printf("Setting up modal mult\n");

    IMGID imgmmult;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mmult", *AOloopindex);
        imgmmult = stream_connect_create_2Df32(name, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmmultfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mmultfact", *AOloopindex);
        imgmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmmultfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgmmultfact.im);
    }


    IMGID imgoffloadmmult;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmmult", *AOloopindex);
        imgoffloadmmult = stream_connect_create_2Df32(name, NBmode, 1);
    }

    // offload modal multiiplicative factors
    // to be multiplied by overal mult to become offloadmmult
    // allows for single-parameter offloadmult tuning
    IMGID imgoffloadmmultfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmmultfact", *AOloopindex);
        imgoffloadmmultfact = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgoffloadmmultfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgoffloadmmultfact.im);
    }




    // ========================= MODAL ZEROPOINT ==========================
    printf("Setting up modal zero point\n");

    IMGID imgmzeropoint;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mzeropoint", *AOloopindex);
        imgmzeropoint = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmzeropoint.im->array.F[mi] = 0.0;
        }
        ImageStreamIO_UpdateIm(imgmzeropoint.im);
    }



    // ========================= MODAL LIMIT ==========================
    printf("Setting up modal limit\n");

    IMGID imgmlimit;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mlimit", *AOloopindex);
        imgmlimit = stream_connect_create_2Df32(name, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmlimitfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mlimitfact", *AOloopindex);
        imgmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmlimitfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgmlimitfact.im);
    }


    IMGID imgoffloadmlimit;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmlimit", *AOloopindex);
        imgoffloadmlimit = stream_connect_create_2Df32(name, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgoffloadmlimitfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_offloadmlimitfact", *AOloopindex);
        imgoffloadmlimitfact = stream_connect_create_2Df32(name, NBmode, 1);
        for(uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgoffloadmlimitfact.im->array.F[mi] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgoffloadmlimitfact.im);
    }





    // ========================= MODAL LIMIT COUNTER ==================
    long * mlimitcntarray = (long*) malloc(sizeof(long)*NBmode);
    long modal_limit_counter = 0;
    for(uint32_t mi = 0; mi < NBmode; mi++)
    {
        mlimitcntarray[mi] = 0;
    }

    IMGID imgmlimitcntfrac;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mlimitcntfrac", *AOloopindex);
        imgmlimitcntfrac = stream_connect_create_2Df32(name, NBmode, 1);
    }




    // ========================= STATS ==================
    // accumulated and reported for each log buffer duration
    //
    double * mvalDMave = (double*) malloc(sizeof(double)*NBmode);
    double * mvalDMrms = (double*) malloc(sizeof(double)*NBmode);
    double * mvalWFSave = (double*) malloc(sizeof(double)*NBmode);
    double * mvalWFSrms = (double*) malloc(sizeof(double)*NBmode);
    double * mvalOLave = (double*) malloc(sizeof(double)*NBmode);
    double * mvalOLrms = (double*) malloc(sizeof(double)*NBmode);

    IMGID imgmvalDMave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalDMave", *AOloopindex);
        imgmvalDMave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalDMrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalDMrms", *AOloopindex);
        imgmvalDMrms = stream_connect_create_2Df32(name, NBmode, 1);
    }

    IMGID imgmvalWFSave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalWFSave", *AOloopindex);
        imgmvalWFSave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalWFSrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalWFSrms", *AOloopindex);
        imgmvalWFSrms = stream_connect_create_2Df32(name, NBmode, 1);
    }

    IMGID imgmvalOLave;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalOLave", *AOloopindex);
        imgmvalOLave = stream_connect_create_2Df32(name, NBmode, 1);
    }
    IMGID imgmvalOLrms;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_mvalOLrms", *AOloopindex);
        imgmvalOLrms = stream_connect_create_2Df32(name, NBmode, 1);
    }



    double * autolimDMsigma = (double*) malloc(sizeof(double)*NBmode);









    // initialization for testOL
    *testOLcnt = 0;


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
                mvalWFS = imginWFS.im->array.F[mi];

                // offset from mval to zero point
                // this is the input zero point
                dmval = imgmzeropoint.im->array.F[mi] - mvalWFS;

                // multiply by GAIN
                dmval *= imgmgain.im->array.F[mi];

                //add the new delta command to the integrated command with leak: this is the goal position
                mvalDMc[mi] = dmval + mvalDMc[mi]*imgmmult.im->array.F[mi];

                // apply LIMIT
                limit = imgmlimit.im->array.F[mi];
                if(mvalDMc[mi] > limit)
                {
                    mvalDMc[mi] = limit;
                    mlimitcntarray[mi] ++;
                }
                if(mvalDMc[mi] < -limit)
                {
                    mvalDMc[mi] = -limit;
                    mlimitcntarray[mi] ++;
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
            // increment modal limit step counter
            modal_limit_counter ++;



            if(*enablePF == 0)
            {
                // if not running PF, apply modes to output
                //
                memcpy(imgout.im->array.F, mvaloutapply, sizeof(float) * NBmode);
                processinfo_update_output_stream(processinfo, imgout.ID);
            }





            // OFFLOAD LOOP
            //
            if((*offload) == 1)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float val = imgmvaloffloadDM.im->array.F[mi];
                    val += imgoffloadmgain.im->array.F[mi] * imgout.im->array.F[mi];
                    val *= imgoffloadmmult.im->array.F[mi];
                    // apply LIMIT
                    limit = imgoffloadmlimit.im->array.F[mi];
                    if(val > limit)
                    {
                        val = limit;
                    }
                    if(val < -limit)
                    {
                        val = -limit;
                    }
                    imgmvaloffloadDM.im->array.F[mi] = val;

                }
                processinfo_update_output_stream(processinfo, imgmvaloffloadDM.ID);
            }






            // Compute pseudo open-loop mode coefficients
            //
            if((*compOL) == 1)
            {
                // write to DM command history
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

                float latencytotalfr = (*latencyhardwfr) + (*latencysoftwfr);

                int   latint  = (int) latencytotalfr;
                float latfrac = latencytotalfr - latint;

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
                    // mvalDMOL is the DM command in the past (lookback time = latency)
                    float tmpmDMval = latfrac * mvalDMbuff[DMtstep0 * NBmode + mi];
                    tmpmDMval +=
                        (1.0 - latfrac) * mvalDMbuff[DMtstep1 * NBmode + mi];
                    mvalDMOL[mi] = tmpmDMval;

                    // last WFS measurement
                    float tmpmWFSval = imginWFS.im->array.F[mi];

                    // OL value
                    imgOLmval.im->array.F[mi] = (*psol_WFSfact)*tmpmWFSval - mvalDMOL[mi];
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





                // OL reconstruction test
                // to be run with low gain and small ampl, and then check with higher gain
                //
                if(*testOL == 1)
                {
                    processinfo_WriteMessage_fmt(processinfo, "testOL ON %u", *testOLcnt);

                    float *psOL_probe;
                    float *psOL_estimate;

                    if(*testOLcnt == 0)
                    {
                        // initialization

                        psOL_probe = (float*) malloc(sizeof(float)*(*testOLnbsample));
                        psOL_estimate = (float*) malloc(sizeof(float)*(*testOLnbsample));


                        // lock parameters while running
                        data.fpsptr->parray[fpi_testOLampl].fpflag &= ~FPFLAG_WRITERUN;
                        data.fpsptr->parray[fpi_testOLmode].fpflag &= ~FPFLAG_WRITERUN;
                        data.fpsptr->parray[fpi_testOLnbsample].fpflag &= ~FPFLAG_WRITERUN;
                    }

                    // increase probe temporal frequency
                    float x = 1.0*(*testOLcnt)/(*testOLnbsample);
                    float freqfact = 0.01 + 1.99*x;

                    // Write probe to auxDM channel
                    imgauxmDM.im->array.F[*testOLmode] =
                        (*testOLampl) * sin( 1.0*(*testOLcnt)/M_PI/2 * freqfact);

                    // Record probe
                    psOL_probe[*testOLcnt] = imgauxmDM.im->array.F[*testOLmode];
                    // Record pOL
                    psOL_estimate[*testOLcnt] = imgOLmval.im->array.F[*testOLmode];


                    (*testOLcnt) ++;
                    if(*testOLcnt == *testOLnbsample)
                    {
                        // end of acquisition
                        /// wrap up and process
                        //
                        *testOL = 0;
                        data.fpsptr->parray[fpi_testOL].fpflag &= ~FPFLAG_ONOFF;
                        *testOLcnt = 0;
                        imgauxmDM.im->array.F[*testOLmode] = 0.0;

                        // unlock parameters
                        data.fpsptr->parray[fpi_testOLampl].fpflag |= FPFLAG_WRITERUN;
                        data.fpsptr->parray[fpi_testOLmode].fpflag |= FPFLAG_WRITERUN;
                        data.fpsptr->parray[fpi_testOLnbsample].fpflag |= FPFLAG_WRITERUN;




                        // process output - compute residual and optimize delay
                        //
                        float latencytotalfr = (*latencyhardwfr) + (*latencysoftwfr);
                        float latencyoptimal = latencytotalfr;
                        float WFSfactoptimal = (*psol_WFSfact);
                        float optval = __FLT32_MAX__;
                        float psOLdelay_fr_min = 0.5*latencytotalfr;
                        float psOLdelay_fr_max = 1.5*latencytotalfr;
                        for(float psOLdelay_fr=psOLdelay_fr_min; psOLdelay_fr < psOLdelay_fr_max; psOLdelay_fr += 0.01)
                        {
                            for(float WFSfactval = 0.9*WFSfactoptimal; WFSfactval < 1.1*WFSfactoptimal; WFSfactval += 0.01)
                            {
                                double psOLresidual = 0.0;
                                for(long ii=0; ii < (*testOLnbsample); ii++)
                                {
                                    float tframeOL = 1.0*ii + psOLdelay_fr;
                                    long jj0 = (long) tframeOL;
                                    float jjfrac = tframeOL - jj0;
                                    long jj1 = jj0+1;
                                    if(jj1 < (*testOLnbsample))
                                    {
                                        // pull OLval from past
                                        double OLval = (1.0-jjfrac)*psOL_estimate[jj0] + jjfrac*psOL_estimate[jj1];

                                        double resval = OLval*WFSfactval - psOL_probe[ii];
                                        psOLresidual += resval*resval;
                                    }
                                }
                                if( psOLresidual < optval)
                                {
                                    optval = psOLresidual;
                                    latencyoptimal = psOLdelay_fr;
                                    WFSfactoptimal = WFSfactval;
                                }
                                //printf("DELAY %8f   RESIDUAL  = %g\n", psOLdelay_fr, psOLresidual);
                            }
                        }


                        printf("OPTIMAL LATENCY = %f fr ->  latencysoft = %f fr\n",
                               latencyoptimal, latencyoptimal - (*latencyhardwfr));
                        printf("OPTIMAL WFSfact = %f\n", WFSfactoptimal);


                        // update solution
                        float g0 = 1.0 - (*testOLupdategain);
                        float g1 = (*testOLupdategain);
                        (*latencysoftwfr) = g0*(*latencysoftwfr) + g1*(latencyoptimal - (*latencyhardwfr));
                        (*psol_WFSfact) = g0*(*psol_WFSfact) + g1*WFSfactoptimal;


                        // Write time series to file
                        {
                            FILE * testOLfp = fopen("testOL.log", "w");
                            float psOLdelay_fr = (*latencysoftwfr)+(*latencyhardwfr);
                            //float WFSfactval = (*psol_WFSfact);

                            for(long ii=0; ii < (*testOLnbsample); ii++)
                            {
                                float tframeOL = 1.0*ii + psOLdelay_fr;
                                long jj0 = (long) tframeOL;
                                float jjfrac = tframeOL - jj0;
                                long jj1 = jj0+1;
                                if(jj1 < (*testOLnbsample))
                                {
                                    double OLval = (1.0-jjfrac)*psOL_estimate[jj0] + jjfrac*psOL_estimate[jj1];
                                    double resval = OLval - psOL_probe[ii];

                                    fprintf(testOLfp, "%5ld  %g %g %g\n",
                                            ii,               // frame counter
                                            psOL_probe[ii],   // probe applied
                                            OLval,            // OL reconstruction
                                            resval            // residual
                                           );

                                }
                            }
                            fclose(testOLfp);
                        }

                        free(psOL_probe);
                        free(psOL_estimate);


                        // write results as env variables
                        {
                            // file will be sourced by cacao-check-cacaovars
                            //
                            char ffname[STRINGMAXLEN_FULLFILENAME];
                            WRITE_FULLFILENAME(ffname, "%s/cacaovars.bash", data.fpsptr->md->datadir);

                            printf("SAVING TO %s\n", ffname);

                            FILE *fpout;
                            fpout = fopen(ffname, "w");

                            char timestring[TIMESTRINGLEN];
                            mkUTtimestring_microsec_now(timestring);
                            fprintf(fpout, "# %s\n", timestring);

                            fprintf(fpout, "export CACAO_PSOL_WFSFACT=%.3f\n", (*psol_WFSfact));
                            fprintf(fpout, "export CACAO_LATENCYSOFTWFR=%.3f\n", (*latencysoftwfr));
                            fprintf(fpout, "export CACAO_LATENCYFR=%.3f\n", (*latencysoftwfr)+(*latencyhardwfr) );
                            fclose(fpout);
                        }

                        processinfo_WriteMessage(processinfo, "testOL done");
                    }
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



            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgoffloadmgain.im->array.F[mi] =
                    imgoffloadmgainfact.im->array.F[mi] * (*offloadloopgain);
            }
            processinfo_update_output_stream(processinfo, imgoffloadmgain.ID);


            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgoffloadmmult.im->array.F[mi] =
                    imgoffloadmmultfact.im->array.F[mi] * (*offloadloopmult);
            }
            processinfo_update_output_stream(processinfo, imgoffloadmmult.ID);


            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgoffloadmlimit.im->array.F[mi] =
                    imgoffloadmlimitfact.im->array.F[mi] * (*offloadlooplimit);
            }
            processinfo_update_output_stream(processinfo, imgoffloadmlimit.ID);



            // predictive filter mixing
            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                imgmPFmix.im->array.F[mi] =
                    imgmPFmixfact.im->array.F[mi] * (*PFmixcoeff);
            }
            processinfo_update_output_stream(processinfo, imgmPFmix.ID);




            // Fill telemetry buffers
            //
            if((*comptbuff) == 1)
            {

                uint64_t kkoffset =
                    tbuffslice * (*tbuffsize) * NBmode + tbuffindex * NBmode;
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    imgtbuff_mvalWFS.im->array.F[kkoffset + mi] =
                        imginWFS.im->array.F[mi];
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



                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    mvalDMave[mi] += mvalout[mi];
                    mvalDMrms[mi] += mvalout[mi]*mvalout[mi];

                    mvalWFSave[mi] += imginWFS.im->array.F[mi];
                    mvalWFSrms[mi] += imginWFS.im->array.F[mi]*imginWFS.im->array.F[mi];

                    mvalOLave[mi] += imgOLmval.im->array.F[mi];
                    mvalOLrms[mi] += imgOLmval.im->array.F[mi]*imgOLmval.im->array.F[mi];
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



                    // Measure fraction of commands truncated by limit
                    //
                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmlimitcntfrac.im->array.F[mi] = (1.0*mlimitcntarray[mi]) / modal_limit_counter;
                        mlimitcntarray[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmlimitcntfrac.ID);



                    // Update buffer stats

                    // DM buffer stats


                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalDMave.im->array.F[mi] = mvalDMave[mi] / modal_limit_counter;
                        mvalDMave[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalDMave.ID);

                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalDMrms.im->array.F[mi] = sqrt ( mvalDMrms[mi] / modal_limit_counter );
                        mvalDMrms[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalDMrms.ID);


                    // WFS buffer stats


                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalWFSave.im->array.F[mi] = mvalWFSave[mi] / modal_limit_counter;
                        mvalWFSave[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalWFSave.ID);

                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalWFSrms.im->array.F[mi] = sqrt ( mvalWFSrms[mi] / modal_limit_counter );
                        mvalWFSrms[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalWFSrms.ID);



                    // OL buffer stats

                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalOLave.im->array.F[mi] = mvalOLave[mi] / modal_limit_counter;
                        mvalOLave[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalOLave.ID);

                    for(uint32_t mi = 0; mi < NBmode; mi++)
                    {
                        imgmvalOLrms.im->array.F[mi] = sqrt ( mvalOLrms[mi] / modal_limit_counter );
                        mvalOLrms[mi] = 0;
                    }
                    processinfo_update_output_stream(processinfo, imgmvalOLrms.ID);


                    modal_limit_counter = 0;





                    if((*autolim) == 1)
                    {
                        // autolimit

                        for(uint32_t mi = 0; mi < NBmode; mi++)
                        {

                            autolimDMsigma[mi] = (1.0 - (*autolimprobegain))* autolimDMsigma[mi]
                                                 + (*autolimprobegain)*imgmvalDMrms.im->array.F[mi];

                            float cliplim = (*autolimsigmafact) * autolimDMsigma[mi];


                            imgmlimitfact.im->array.F[mi] = cliplim / (*looplimit);
                        }
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

            memcpy(imginWFS.im->array.F, imgout.im->array.F, sizeof(float) * NBmode);
            processinfo_update_output_stream(processinfo, imginWFS.ID);
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
                        0.5 * signmult * selfRMpokesign * imginWFS.im->array.F[mi] /
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


            if((int) selfRM_pokemode == selfRM_NBmode)
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
    free(mvalDMOL);

    free(mlimitcntarray);

    free(mvalDMave);
    free(mvalDMrms);
    free(mvalWFSave);
    free(mvalWFSrms);
    free(mvalOLave);
    free(mvalOLrms);

    free(autolimDMsigma);

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
