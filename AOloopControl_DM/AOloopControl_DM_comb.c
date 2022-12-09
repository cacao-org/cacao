/**
 * @file    AOloopControl_DM_comb.c
 * @brief   DM control
 *
 * Combine DM channels
 *
 *
 *
 */

#include <math.h>
#include <time.h>

#include "CommandLineInterface/CLIcore.h"

// includes AOLOOPCONTROL_DM_DISPCOMB_CONF
//#include "AOloopControl_DM.h"

//#include <time.h>

static int DMdisp_add_disp_from_circular_buffer_init = 0;


// Local variables pointers
static uint32_t *DMindex = NULL;
long             fpi_DMindex;

// output DM displacement stream
// can be read or created
static char *DMcombout = NULL;
long         fpi_DMcombout;

static uint32_t *DMxsize = NULL;
long             fpi_DMxsize;

static uint32_t *DMysize = NULL;
long             fpi_DMysize;

static uint32_t *NBchannel = NULL;

static uint32_t *DMmode = NULL;

static uint32_t *AveMode = NULL;

static int64_t *dm2dm_mode = NULL;
long            fpi_dm2dm_mode;

static char *dm2dm_DMmodes = NULL;
long         fpi_dm2dm_DMmodes;

static char *dm2dm_outdisp = NULL;
long         fpi_dm2dm_outdisp;

static int64_t *wfsrefmode = NULL;
long            fpi_wfsrefmode;

static char *wfsref_WFSRespMat = NULL;
long         fpi_wfsref_WFSRespMat;

static char *wfsref_out = NULL;
long         fpi_wfsref_out;

static int64_t *voltmode = NULL;
static long     fpi_voltmode = -1;

static uint32_t *volttype = NULL;

static float *stroke100 = NULL; // stroke [um] for 100V
static long   fpi_stroke100;

static char *voltname = NULL;
long         fpi_voltname;

static float *DClevel = NULL;
static long   fpi_DClevel;

static float *maxvolt = NULL;
static long   fpi_maxvolt;

static uint64_t *loopcnt = NULL;




// optional additive circular buffer

// on / off
static int64_t *astrogrid = NULL;
long            fpi_astrogrid;

// channel
static uint32_t *astrogridchan = NULL;
long             fpi_astrogridchan;

// stream contaning circular buffer
static char *astrogridsname = NULL;
long         fpi_astrogridsname;


// multiplicative coeff
static float *astrogridmult = NULL;
long          fpi_astrogridmult;

static uint32_t *astrogridtdelay = NULL;
long             fpi_astrogridtdelay;

// number of consecutive frames with same slice
static uint32_t *astrogridNBframe = NULL;
long             fpi_astrogridNBframe;




// zero point offset to WFS
static int64_t *zpoffsetenable = NULL;
long            fpi_zpoffsetenable;

static char *DMcomboutzpo = NULL;
long         fpi_DMcomboutzpo;


#define NB_ZEROPOINT_CH_MAX 12
static int      zpoffset_channel[NB_ZEROPOINT_CH_MAX];
static long     zpo_chan_fpi_map[NB_ZEROPOINT_CH_MAX];
static int64_t *zpoffsetch00 = NULL;
long            fpi_zpoffsetch00;
static int64_t *zpoffsetch01 = NULL;
long            fpi_zpoffsetch01;
static int64_t *zpoffsetch02 = NULL;
long            fpi_zpoffsetch02;
static int64_t *zpoffsetch03 = NULL;
long            fpi_zpoffsetch03;
static int64_t *zpoffsetch04 = NULL;
long            fpi_zpoffsetch04;
static int64_t *zpoffsetch05 = NULL;
long            fpi_zpoffsetch05;
static int64_t *zpoffsetch06 = NULL;
long            fpi_zpoffsetch06;
static int64_t *zpoffsetch07 = NULL;
long            fpi_zpoffsetch07;
static int64_t *zpoffsetch08 = NULL;
long            fpi_zpoffsetch08;
static int64_t *zpoffsetch09 = NULL;
long            fpi_zpoffsetch09;
static int64_t *zpoffsetch10 = NULL;
long            fpi_zpoffsetch10;
static int64_t *zpoffsetch11 = NULL;
long            fpi_zpoffsetch11;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_UINT32,
        ".DMindex",
        "Deformable mirror index",
        "5",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMindex,
        &fpi_DMindex
    },
    {
        CLIARG_STREAM,
        ".DMcombout",
        "output stream for combined command",
        "dm99disp",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMcombout,
        &fpi_DMcombout
    },
    {
        CLIARG_UINT32,
        ".DMxsize",
        "x size",
        "20",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMxsize,
        &fpi_DMxsize
    },
    {
        CLIARG_UINT32,
        ".DMysize",
        "y size",
        "20",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMysize,
        &fpi_DMysize
    },
    {
        CLIARG_UINT32,
        ".NBchannel",
        "number of DM channels",
        "12",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBchannel,
        NULL
    },
    {
        CLIARG_UINT32,
        ".DMmode",
        "0:SquareGrid, 1:Generic",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMmode,
        NULL
    },
    {
        CLIARG_UINT32,
        ".AveMode",
        "Averaging mode",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &AveMode,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".option.dm2dm_mode",
        "DM to DM offset mode",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dm2dm_mode,
        &fpi_dm2dm_mode
    },
    {
        CLIARG_STREAM,
        ".option.dm2dm_DMmodes",
        "Output stream DM to DM",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dm2dm_DMmodes,
        &fpi_dm2dm_DMmodes
    },
    {
        CLIARG_STREAM,
        ".option.dm2dm_outdisp",
        "data stream to which output DM is written",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &dm2dm_outdisp,
        &fpi_dm2dm_outdisp
    },
    {
        CLIARG_ONOFF,
        ".option.wfsrefmode",
        "WFS ref mode",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsrefmode,
        &fpi_wfsrefmode
    },
    {
        CLIARG_STREAM,
        ".option.wfsref_WFSRespMat",
        "Output WFS resp matrix",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsref_WFSRespMat,
        &fpi_wfsref_WFSRespMat
    },
    {
        CLIARG_STREAM,
        ".option.wfsref_out",
        "Output WFS",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &wfsref_out,
        &fpi_wfsref_out
    },
    {
        CLIARG_ONOFF,
        ".option.voltmode",
        "Volt mode",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &voltmode,
        &fpi_voltmode
    },
    {
        CLIARG_UINT32,
        ".option.volttype",
        "volt type",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &volttype,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".option.stroke100",
        "Stroke for 100 V [um]",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &stroke100,
        &fpi_stroke100
    },
    {
        CLIARG_STREAM,
        ".option.voltname",
        "Stream name for volt output",
        "dmvolt",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &voltname,
        &fpi_voltname
    },
    {
        CLIARG_FLOAT32,
        ".option.DClevel",
        "DC level [um]",
        "0.5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DClevel,
        &fpi_DClevel
    },
    {
        CLIARG_FLOAT32,
        ".option.maxvolt",
        "Maximum voltage",
        "100.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &maxvolt,
        &fpi_maxvolt
    },
    {
        CLIARG_UINT64,
        ".status.loopcnt",
        "Loop counter",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopcnt,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".astrogrid.mode",
        "circular buffer on/off",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogrid,
        &fpi_astrogrid
    },
    {
        CLIARG_UINT32,
        ".astrogrid.chan",
        "astrogrid DM channel",
        "9",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogridchan,
        &fpi_astrogridchan
    },
    {
        CLIARG_STREAM,
        ".astrogrid.sname",
        "astrogrid cube name",
        "dmCBcube",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogridsname,
        &fpi_astrogridsname
    },
    {
        CLIARG_FLOAT32,
        ".astrogrid.mult",
        "astrogrid multiplicative coeff",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogridmult,
        &fpi_astrogridmult
    },
    {
        CLIARG_UINT32,
        ".astrogrid.delay",
        "time delay between main update and astrogrid update [us]",
        "100",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogridtdelay,
        &fpi_astrogridtdelay
    },
    {
        CLIARG_UINT32,
        ".astrogrid.nbframe",
        "astrogrid number of frame per slice",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &astrogridNBframe,
        &fpi_astrogridNBframe
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.enable",
        "zero point offset enable",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetenable,
        &fpi_zpoffsetenable
    },
    {
        CLIARG_STREAM,
        ".zpoffset.DMcomboutzpo",
        "output stream for combined zero point offset",
        "dm99zpo",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &DMcomboutzpo,
        &fpi_DMcomboutzpo
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch00",
        "channel 00 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch00,
        &fpi_zpoffsetch00
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch01",
        "channel 01 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch01,
        &fpi_zpoffsetch01
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch02",
        "channel 02 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch02,
        &fpi_zpoffsetch02
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch03",
        "channel 03 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch03,
        &fpi_zpoffsetch03
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch04",
        "channel 04 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch04,
        &fpi_zpoffsetch04
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch05",
        "channel 05 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch05,
        &fpi_zpoffsetch05
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch06",
        "channel 06 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch06,
        &fpi_zpoffsetch06
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch07",
        "channel 07 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch07,
        &fpi_zpoffsetch07
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch08",
        "channel 08 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch08,
        &fpi_zpoffsetch08
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch09",
        "channel 09 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch09,
        &fpi_zpoffsetch09
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch10",
        "channel 10 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch10,
        &fpi_zpoffsetch10
    },
    {
        CLIARG_ONOFF,
        ".zpoffset.ch11",
        "channel 11 zpoffset ?",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &zpoffsetch11,
        &fpi_zpoffsetch11
    }
};




// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

        data.fpsptr->parray[fpi_DMindex].fpflag =
            FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
        data.fpsptr->parray[fpi_DMindex].fpflag &= ~FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_DMindex].val.ui32[1] = 0;  // min value
        data.fpsptr->parray[fpi_DMindex].val.ui32[2] = 99; // max value

        data.fpsptr->parray[fpi_voltmode].fpflag  |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_stroke100].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_DClevel].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_maxvolt].fpflag   |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_astrogrid].fpflag        |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_astrogridmult].fpflag    |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_astrogridtdelay].fpflag  |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_astrogridNBframe].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_zpoffsetenable].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch00].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch01].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch02].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch03].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch04].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch05].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch06].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch07].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch08].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch09].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch10].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_zpoffsetch11].fpflag   |= FPFLAG_WRITERUN;
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
        if(data.fpsptr->parray[fpi_dm2dm_mode].fpflag &
                FPFLAG_ONOFF) // ON state
        {
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag |= FPFLAG_VISIBLE;
        }
        else // OFF state
        {
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_dm2dm_DMmodes].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dm2dm_outdisp].fpflag &= ~FPFLAG_VISIBLE;
        }



        if(data.fpsptr->parray[fpi_wfsrefmode].fpflag &
                FPFLAG_ONOFF) // ON state
        {
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_wfsref_out].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_out].fpflag |= FPFLAG_VISIBLE;
        }
        else // OFF state
        {
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_WFSRespMat].fpflag &=
                ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_wfsref_out].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_wfsref_out].fpflag &= ~FPFLAG_VISIBLE;
        }



        if(data.fpsptr->parray[fpi_voltmode].fpflag & FPFLAG_ONOFF)  // ON state
        {
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_voltname].fpflag |=
                FPFLAG_STREAM_RUN_REQUIRED;
        }
        else // OFF state
        {
            data.fpsptr->parray[fpi_voltname].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_voltname].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_voltname].fpflag &=
                ~FPFLAG_STREAM_RUN_REQUIRED;
        }


        if(data.fpsptr->parray[fpi_astrogrid].fpflag &
                FPFLAG_ONOFF) // ON state
        {
            data.fpsptr->parray[fpi_astrogridsname].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_astrogridsname].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_astrogridsname].fpflag |=
                FPFLAG_STREAM_RUN_REQUIRED;
        }
        else // OFF state
        {
            data.fpsptr->parray[fpi_astrogridsname].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_astrogridsname].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_astrogridsname].fpflag &=
                ~FPFLAG_STREAM_RUN_REQUIRED;
        }

        zpo_chan_fpi_map[0] = fpi_zpoffsetch00;
        zpo_chan_fpi_map[1] = fpi_zpoffsetch01;
        zpo_chan_fpi_map[2] = fpi_zpoffsetch02;
        zpo_chan_fpi_map[3] = fpi_zpoffsetch03;
        zpo_chan_fpi_map[4] = fpi_zpoffsetch04;
        zpo_chan_fpi_map[5] = fpi_zpoffsetch05;
        zpo_chan_fpi_map[6] = fpi_zpoffsetch06;
        zpo_chan_fpi_map[7] = fpi_zpoffsetch07;
        zpo_chan_fpi_map[8] = fpi_zpoffsetch08;
        zpo_chan_fpi_map[9] = fpi_zpoffsetch09;
        zpo_chan_fpi_map[10] = fpi_zpoffsetch10;
        zpo_chan_fpi_map[11] = fpi_zpoffsetch11;

        for(long zpo_chan = 0; zpo_chan < *NBchannel; zpo_chan++)
        {
            data.fpsptr->parray[zpo_chan_fpi_map[zpo_chan]].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[zpo_chan_fpi_map[zpo_chan]].fpflag |= FPFLAG_VISIBLE;

        }
        for(long zpo_chan = *NBchannel; zpo_chan < NB_ZEROPOINT_CH_MAX; zpo_chan++)
        {
            data.fpsptr->parray[zpo_chan_fpi_map[zpo_chan]].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[zpo_chan_fpi_map[zpo_chan]].fpflag &= ~FPFLAG_VISIBLE;
        }

        if(data.fpsptr->parray[fpi_zpoffsetch00].fpflag & FPFLAG_ONOFF)
        {
            *zpoffsetch00 = 1;
        }
        else
        {
            *zpoffsetch00 = 0;
        }

        zpoffset_channel[0]  = *zpoffsetch00;
        zpoffset_channel[1]  = *zpoffsetch01;
        zpoffset_channel[2]  = *zpoffsetch02;
        zpoffset_channel[3]  = *zpoffsetch03;
        zpoffset_channel[4]  = *zpoffsetch04;
        zpoffset_channel[5]  = *zpoffsetch05;
        zpoffset_channel[6]  = *zpoffsetch06;
        zpoffset_channel[7]  = *zpoffsetch07;
        zpoffset_channel[8]  = *zpoffsetch08;
        zpoffset_channel[9]  = *zpoffsetch09;
        zpoffset_channel[10] = *zpoffsetch10;
        zpoffset_channel[11] = *zpoffsetch11;
    }

    return RETURN_SUCCESS;
}



static CLICMDDATA CLIcmddata =
{
    "DMcomb", "Deformable mirror combine channels", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t DMdisp_add_disp_from_circular_buffer(IMGID dispchout)
{
    static uint32_t sliceindex = 0;
    static IMGID    imgdispbuffer = {0};

    static uint32_t framecnt = 0;
    static uint64_t xysize;

    if(DMdisp_add_disp_from_circular_buffer_init == 0)
    {
        printf("(re-)initializing DMdisp_add_disp_from_circular_buffer\n");
        delete_image_ID(astrogridsname, DELETE_IMAGE_ERRMODE_WARNING);
        read_sharedmem_image(astrogridsname);
        imgdispbuffer = mkIMGID_from_name(astrogridsname);
        resolveIMGID(&imgdispbuffer, ERRMODE_ABORT);
        xysize = (uint64_t)(*DMxsize) * (*DMysize);

        DMdisp_add_disp_from_circular_buffer_init = 1;
    }


    if((*astrogrid) == 1)
    {
        /* printf("Apply circular buffer slice %u / %u\n",
               sliceindex,
               imgdispbuffer.size[2]);*/

        framecnt++;
        if(framecnt >= (*astrogridNBframe))
        {
            framecnt = 0;
            sliceindex++;

            if(sliceindex >= imgdispbuffer.size[2])
            {
                sliceindex = 0;
            }

            for(uint64_t ii = 0; ii < xysize; ii++)
            {
                dispchout.im->array.F[ii] =
                    (*astrogridmult) *
                    imgdispbuffer.im->array.F[sliceindex * xysize + ii];
            }
        }
    }
    return RETURN_SUCCESS;
}



static errno_t DM_displ2V(IMGID imgdisp, IMGID imgvolt)
{
    //    printf("DISP -> VOLT  %lu actuators\n",
    //           ((uint64_t) (*DMxsize)) * (*DMysize));


    //int QUANTIZATION_RANDOM = 2;
    // 1: remove quantization error by probabilistic value, recomputed for each
    // new value
    // 2: remove quantization error by adding an external random map
    // between 0 and 1, named dmXXquant
    //    USER SHOULD UPDATE THIS MAP WHEN REQUIRED



    if((*volttype) == 1)
    {
        // linear bipolar, output is float
        for(uint64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            float voltvalue = 100.0 * imgdisp.im->array.F[ii] / (*stroke100);
            if(voltvalue > (*maxvolt))
            {
                voltvalue = (*maxvolt);
            }
            if(voltvalue < -(*maxvolt))
            {
                voltvalue = -(*maxvolt);
            }
            imgvolt.im->array.F[ii] = voltvalue;
        }
    }
    else if((*volttype) == 2)
    {
        // quadratic unipolar, output is UI16
        for(uint64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            float volt = 100.0 * sqrt(imgdisp.im->array.F[ii] / (*stroke100));
            if(volt > (*maxvolt))
            {
                volt = (*maxvolt);
            }
            // TODO add quantization code
            imgvolt.im->array.UI16[ii] =
                (unsigned short int)(volt / 300.0 * 16384.0);
        }
    }

    return RETURN_SUCCESS;
}




static errno_t update_dmdisp(IMGID imgdisp, IMGID *imgch, float *dmdisptmp)
{
    memcpy(dmdisptmp,
           imgch[0].im->array.F,
           sizeof(float) * (*DMxsize) * (*DMysize));
    for(uint32_t ch = 1; ch < *NBchannel; ch++)
    {
        for(uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            dmdisptmp[ii] += imgch[ch].im->array.F[ii];
        }
    }

    // Remove average
    //
    double ave = 0.0;
    if(*AveMode == 1)
    {
        for(uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            ave += dmdisptmp[ii];
        }
        ave /= (*DMxsize) * (*DMysize);
    }

    if(*AveMode < 2)  // OFFSET BY DClevel
    {
        for(uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            dmdisptmp[ii] += (*DClevel - ave);

            // remove negative values
            if(*voltmode == 1)
                if(dmdisptmp[ii] < 0.0)
                {
                    dmdisptmp[ii] = 0.0;
                }
        }
    }

    memcpy(imgdisp.im->array.F,
           dmdisptmp,
           sizeof(float) * (*DMxsize) * (*DMysize));

    return RETURN_SUCCESS;
}



static errno_t update_dmdispzpo(IMGID imgdisp, IMGID *imgch, float *dmdisptmp)
{
    for(uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
    {
        dmdisptmp[ii] = 0.0;
    }

    for(uint32_t ch = 0; ch < *NBchannel; ch++)
    {
        if(zpoffset_channel[ch] == 1)
        {
            for(uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
            {
                dmdisptmp[ii] += imgch[ch].im->array.F[ii];
            }
        }
    }

    memcpy(imgdisp.im->array.F,
           dmdisptmp,
           sizeof(float) * (*DMxsize) * (*DMysize));

    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // Connect to or (re)create DM channel streams
    //
    IMGID *imgch = calloc(*NBchannel, sizeof(IMGID));
    printf("This is DM comb, index = %ld\n", (long) *DMindex);
    printf("Initialize channels\n");
    for(uint32_t ch = 0; ch < *NBchannel; ch++)
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "dm%02udisp%02u", *DMindex, ch);
        read_sharedmem_image(name);
        imgch[ch] = stream_connect_create_2Df32(name, *DMxsize, *DMysize);
    }

    // Combined DM channel
    IMGID imgdisp = stream_connect_create_2Df32(DMcombout, *DMxsize, *DMysize);

    // Combined DM channel zero point offset
    IMGID imgdispzpo =
        stream_connect_create_2Df32(DMcomboutzpo, *DMxsize, *DMysize);


    // Create temporaray storage to compute summed displacement
    //
    float *dmdisptmp = malloc(sizeof(*dmdisptmp) * (*DMxsize) * (*DMysize));

    IMGID imgdmvolt;
    if(*voltmode == 1)
    {
        if(image_ID(voltname) == -1)
        {
            read_sharedmem_image(voltname);
        }
        imgdmvolt = mkIMGID_from_name(voltname);
        resolveIMGID(&imgdmvolt, ERRMODE_ABORT);
    }
    list_image_ID();



    long cntsumref    = 0;
    long cntsumrefzpo = 0;

    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    {

        int zpooffsetchange = 0;
        int zpoval;
        int fpi_zpoch;

        for(int ch = 0; ch < NB_ZEROPOINT_CH_MAX; ++ch)
        {
            fpi_zpoch = zpo_chan_fpi_map[ch];
            zpoval = zpoffset_channel[ch];
            if(data.fpsptr->parray[fpi_zpoch].fpflag & FPFLAG_ONOFF)
            {
                zpoffset_channel[ch] = 1;
                zpooffsetchange += 1 - zpoval;
            }
            else
            {
                zpoffset_channel[ch] = 0;
                zpooffsetchange += zpoval;
            }
        }

        // Check if DM needs updating
        // DMupdate toggles to 1 if DM must be updated
        //
        // DMupdatezpo is for zero point offset
        //
        int DMupdate    = 0;
        int DMupdatezpo = 0;
        {
            long cnt0sum    = 0;
            long cnt0sumzpo = 0;

            if(*astrogrid == 1)
            {
                // exclude astrogridchan
                for(uint32_t ch = 0; ch < *NBchannel; ch++)
                {
                    if(ch != *astrogridchan)
                    {
                        cnt0sum += imgch[ch].md->cnt0;

                        if((*zpoffsetenable == 1) && (zpoffset_channel[ch] == 1))
                        {
                            cnt0sumzpo += imgch[ch].md->cnt0;
                        }
                    }
                }
            }
            else
            {
                for(uint32_t ch = 0; ch < *NBchannel; ch++)
                {
                    cnt0sum += imgch[ch].md->cnt0;
                    /*if (*zpoffsetenable == 1)
                    {
                        printf(" %d", zpoffset_channel[ch]);
                    }*/

                    if((*zpoffsetenable == 1) && (zpoffset_channel[ch] == 1))
                    {
                        cnt0sumzpo += imgch[ch].md->cnt0;
                    }
                }
                /*            if (*zpoffsetenable == 1)
                            {
                                printf("\n");
                            }*/
            }

            if(cnt0sum != cntsumref)
            {
                //printf("cnt0sum = %ld\n", cnt0sum);
                cntsumref = cnt0sum;
                DMupdate  = 1;
            }
            if(cnt0sumzpo != cntsumrefzpo)
            {
                //printf("cnt0sumzpo = %ld\n", cnt0sumzpo);
                cntsumrefzpo = cnt0sumzpo;
                DMupdatezpo  = 1;
            }
        }

        if(*zpoffsetenable == 1)
        {
            if(zpooffsetchange != 0)
            {
                DMupdatezpo = 1;
            }
        }



        if(DMupdate == 1)
        {
            // Update DM disp
            //printf("Updating dmdisp\n\n");

            if((*astrogrid) == 0)
            {
                DMdisp_add_disp_from_circular_buffer_init = 0;
            }


            // Sum all channels
            //
            if(((*astrogrid) == 1) && ((*astrogridtdelay) == 0))
            {
                DMdisp_add_disp_from_circular_buffer(imgch[(*astrogridchan)]);
                processinfo_update_output_stream(processinfo,
                                                 imgch[(*astrogridchan)].ID);
            }
            update_dmdisp(imgdisp, imgch, dmdisptmp);
            processinfo_update_output_stream(processinfo, imgdisp.ID);

            if(*voltmode == 1)
            {
                imgdmvolt.md->write = 1;
                DM_displ2V(imgdisp, imgdmvolt);
                processinfo_update_output_stream(processinfo, imgdmvolt.ID);
            }



            if(((*astrogrid) == 1) && ((*astrogridtdelay) != 0))
            {
                DMdisp_add_disp_from_circular_buffer(imgch[(*astrogridchan)]);
                processinfo_update_output_stream(processinfo,
                                                 imgch[(*astrogridchan)].ID);

                // Add time delay
                {
                    long nsec = (long)(1000 * (*astrogridtdelay));

                    long nsec_remaining = nsec % 1000000000;
                    long sec            = nsec / 1000000000;

                    struct timespec timesleep;
                    timesleep.tv_sec  = sec;
                    timesleep.tv_nsec = nsec_remaining;

                    nanosleep(&timesleep, NULL);
                }

                update_dmdisp(imgdisp, imgch, dmdisptmp);
                processinfo_update_output_stream(processinfo, imgdisp.ID);
                // take into account update to astrogrid channel
                cntsumref++;

                if(*voltmode == 1)
                {
                    imgdmvolt.md->write = 1;
                    DM_displ2V(imgdisp, imgdmvolt);
                    processinfo_update_output_stream(processinfo, imgdmvolt.ID);
                }
            }
        }

        if(DMupdatezpo == 1)
        {
            // printf("Updating zpo %d\n", zpooffsetchange);
            update_dmdispzpo(imgdispzpo, imgch, dmdisptmp);
            processinfo_update_output_stream(processinfo, imgdispzpo.ID);
        }



    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(dmdisptmp);
    free(imgch);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_DM__comb()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
