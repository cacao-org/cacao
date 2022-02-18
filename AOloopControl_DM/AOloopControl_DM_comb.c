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

#include "CommandLineInterface/CLIcore.h"

// includes AOLOOPCONTROL_DM_DISPCOMB_CONF
//#include "AOloopControl_DM.h"

//#include <time.h>

// Local variables pointers
static uint32_t *DMindex;
long             fpi_DMindex;

// output DM displacement stream
// can be read or created
static char *DMcombout;
long         fpi_DMcombout;

static uint32_t *DMxsize;
long             fpi_DMxsize;

static uint32_t *DMysize;
long             fpi_DMysize;

static uint32_t *NBchannel;

static uint32_t *DMmode;

static uint32_t *AveMode;

static int64_t *dm2dm_mode;
long            fpi_dm2dm_mode;

static char *dm2dm_DMmodes;
long         fpi_dm2dm_DMmodes;

static char *dm2dm_outdisp;
long         fpi_dm2dm_outdisp;

static int64_t *wfsrefmode;
long            fpi_wfsrefmode;

static char *wfsref_WFSRespMat;
long         fpi_wfsref_WFSRespMat;

static char *wfsref_out;
long         fpi_wfsref_out;

static int64_t *voltmode;
static long     fpi_voltmode = -1;

static uint32_t *volttype;

static float *stroke100; // stroke [um] for 100V

static char *voltname;
long         fpi_voltname;

static float *DClevel;

static float *maxvolt;

static uint64_t *loopcnt;




// optional additive circular buffer

// on / off
static int64_t *dmdispcircbuff;
long            fpi_dmdispcircbuff;

// channel
static uint32_t *dmdispCBchan;
long             fpi_dmdispCBchan;

// stream contaning circular buffer
static char *dmdispCBsname;
long         fpi_dmdispCBsname;


// multiplicative coeff
static float *dmdispCBmult;
long          fpi_dmdispCBmult;


// number of consecutive frames with same slice
static uint32_t *dmdispCBnbframe;
long             fpi_dmdispCBnbframe;




static CLICMDARGDEF farg[] = {{CLIARG_UINT32,
                               ".DMindex",
                               "Deformable mirror index",
                               "5",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &DMindex,
                               &fpi_DMindex},
                              {CLIARG_STREAM,
                               ".DMcombout",
                               "output stream for combined command",
                               "dm99disp",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &DMcombout,
                               &fpi_DMcombout},
                              {CLIARG_UINT32,
                               ".DMxsize",
                               "x size",
                               "20",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &DMxsize,
                               &fpi_DMxsize},
                              {CLIARG_UINT32,
                               ".DMysize",
                               "y size",
                               "20",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &DMysize,
                               &fpi_DMysize},
                              {CLIARG_UINT32,
                               ".NBchannel",
                               "number of DM channels",
                               "12",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &NBchannel,
                               NULL},
                              {CLIARG_UINT32,
                               ".DMmode",
                               "0:SquareGrid, 1:Generic",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &DMmode,
                               NULL},
                              {CLIARG_UINT32,
                               ".AveMode",
                               "Averaging mode",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &AveMode,
                               NULL},
                              {CLIARG_ONOFF,
                               ".option.dm2dm_mode",
                               "DM to DM offset mode",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dm2dm_mode,
                               &fpi_dm2dm_mode},
                              {CLIARG_STREAM,
                               ".option.dm2dm_DMmodes",
                               "Output stream DM to DM",
                               "null",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dm2dm_DMmodes,
                               &fpi_dm2dm_DMmodes},
                              {CLIARG_STREAM,
                               ".option.dm2dm_outdisp",
                               "data stream to which output DM is written",
                               "null",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dm2dm_outdisp,
                               &fpi_dm2dm_outdisp},
                              {CLIARG_ONOFF,
                               ".option.wfsrefmode",
                               "WFS ref mode",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &wfsrefmode,
                               &fpi_wfsrefmode},
                              {CLIARG_STREAM,
                               ".option.wfsref_WFSRespMat",
                               "Output WFS resp matrix",
                               "null",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &wfsref_WFSRespMat,
                               &fpi_wfsref_WFSRespMat},
                              {CLIARG_STREAM,
                               ".option.wfsref_out",
                               "Output WFS",
                               "null",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &wfsref_out,
                               &fpi_wfsref_out},
                              {CLIARG_ONOFF,
                               ".option.voltmode",
                               "Volt mode",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &voltmode,
                               &fpi_voltmode},
                              {CLIARG_UINT32,
                               ".option.volttype",
                               "volt type",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &volttype,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".option.stroke100",
                               "Stroke for 100 V [um]",
                               "1.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &stroke100,
                               NULL},
                              {CLIARG_STREAM,
                               ".option.voltname",
                               "Stream name for volt output",
                               "dmvolt",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &voltname,
                               &fpi_voltname},
                              {CLIARG_FLOAT32,
                               ".option.DClevel",
                               "DC level [um]",
                               "0.5",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &DClevel,
                               NULL},
                              {CLIARG_FLOAT32,
                               ".option.maxvolt",
                               "Maximum voltage",
                               "100.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &maxvolt,
                               NULL},
                              {CLIARG_UINT64,
                               ".status.loopcnt",
                               "Loop counter",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &loopcnt,
                               NULL},
                              {CLIARG_ONOFF,
                               ".dispCB.mode",
                               "circular buffer on/off",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dmdispcircbuff,
                               &fpi_dmdispcircbuff},
                              {CLIARG_UINT32,
                               ".dispCB.chan",
                               "circular buffer DM channel",
                               "9",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dmdispCBchan,
                               &fpi_dmdispCBchan},
                              {CLIARG_STREAM,
                               ".dispCB.sname",
                               "circular buffer cube name",
                               "dmCBcube",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dmdispCBsname,
                               &fpi_dmdispCBsname},
                              {CLIARG_FLOAT32,
                               ".dispCB.mult",
                               "circular buffer multiplicative coeff",
                               "1.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dmdispCBmult,
                               &fpi_dmdispCBmult},
                              {CLIARG_UINT32,
                               ".dispCB.nbframe",
                               "number of frame per slice",
                               "1",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &dmdispCBnbframe,
                               &fpi_dmdispCBnbframe}};

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {

        data.fpsptr->parray[fpi_DMindex].fpflag =
            FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
        data.fpsptr->parray[fpi_DMindex].fpflag &= ~FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_DMindex].val.ui32[1] = 0;  // min value
        data.fpsptr->parray[fpi_DMindex].val.ui32[2] = 99; // max value
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if (data.fpsptr != NULL)
    {
        if (data.fpsptr->parray[fpi_dm2dm_mode].fpflag &
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



        if (data.fpsptr->parray[fpi_wfsrefmode].fpflag &
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



        if (data.fpsptr->parray[fpi_voltmode].fpflag & FPFLAG_ONOFF) // ON state
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


        if (data.fpsptr->parray[fpi_dmdispcircbuff].fpflag &
            FPFLAG_ONOFF) // ON state
        {
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag |= FPFLAG_USED;
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag |= FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag |=
                FPFLAG_STREAM_RUN_REQUIRED;
        }
        else // OFF state
        {
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag &= ~FPFLAG_USED;
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag &= ~FPFLAG_VISIBLE;
            data.fpsptr->parray[fpi_dmdispCBsname].fpflag &=
                ~FPFLAG_STREAM_RUN_REQUIRED;
        }
    }

    return RETURN_SUCCESS;
}



static CLICMDDATA CLIcmddata = {
    "DMcomb", "Deformable mirror combine channels", CLICMD_FIELDS_DEFAULTS};




// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t DMdisp_add_disp_from_circular_buffer(IMGID dispchout)
{
    static int funcinit = 0;

    static uint32_t sliceindex = 0;
    static IMGID    imgdispbuffer;

    static uint32_t framecnt = 0;
    static uint64_t xysize;

    if (funcinit == 0)
    {
        read_sharedmem_image(dmdispCBsname);
        imgdispbuffer = mkIMGID_from_name(dmdispCBsname);
        resolveIMGID(&imgdispbuffer, ERRMODE_ABORT);

        xysize = (uint64_t) (*DMxsize) * (*DMysize);

        funcinit = 1;
    }


    if ((*dmdispcircbuff) == 1)
    {
        printf("Apply circular buffer slice %u\n", sliceindex);

        framecnt++;
        if (framecnt == (*dmdispCBnbframe))
        {
            framecnt = 0;
            sliceindex++;

            if (sliceindex >= imgdispbuffer.size[2])
            {
                sliceindex = 0;
            }

            for (uint64_t ii = 0; ii < xysize; ii++)
            {
                dispchout.im->array.F[ii] =
                    (*dmdispCBmult) *
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



    if ((*volttype) == 1)
    {
        // linear bipolar, output is float
        printf("volttype 1\n");

        for (uint64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            float voltvalue = 100.0 * imgdisp.im->array.F[ii] / (*stroke100);
            if (voltvalue > (*maxvolt))
            {
                voltvalue = (*maxvolt);
            }
            if (voltvalue < -(*maxvolt))
            {
                voltvalue = -(*maxvolt);
            }
            imgvolt.im->array.F[ii] = voltvalue;
        }
    }
    else if ((*volttype) == 2)
    {
        // quadratic unipolar, output is UI16
        printf("volttype 2\n");
        for (uint64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
        {
            float volt = 100.0 * sqrt(imgdisp.im->array.F[ii] / (*stroke100));
            if (volt > (*maxvolt))
            {
                volt = (*maxvolt);
            }
            // TODO add quantization code
            imgvolt.im->array.UI16[ii] =
                (unsigned short int) (volt / 300.0 * 16384.0);
            printf("    %3ld    %10.f   %10.3f      %u\n",
                   ii,
                   (*stroke100),
                   volt,
                   imgvolt.im->array.UI16[ii]);
        }
    }

    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // Connect to or (re)create DM channel streams
    //
    IMGID imgch[*NBchannel];
    printf("This is DM comb, index = %ld\n", (long) *DMindex);
    printf("Initialize channels\n");
    for (uint32_t ch = 0; ch < *NBchannel; ch++)
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "dm%02udisp%02u", *DMindex, ch);
        read_sharedmem_image(name);
        imgch[ch] = stream_connect_create_2Df32(name, *DMxsize, *DMysize);
    }

    // Combined DM channel
    //
    IMGID imgdisp;
    {
        //char name[STRINGMAXLEN_STREAMNAME];
        //WRITE_IMAGENAME(name, "dm%02udisp", *DMindex);
        imgdisp = stream_connect_create_2Df32(DMcombout, *DMxsize, *DMysize);
    }

    // Create temporaray storage to compute summed displacement
    //
    float *dmdisptmp =
        (float *) malloc(sizeof(float) * (*DMxsize) * (*DMysize));



    IMGID imgdmvolt;
    if (*voltmode == 1)
    {

        if (image_ID(voltname) == -1)
        {
            read_sharedmem_image(voltname);
        }

        imgdmvolt = mkIMGID_from_name(voltname);
        resolveIMGID(&imgdmvolt, ERRMODE_ABORT);
    }
    list_image_ID();



    long cntsumref = 0;

    INSERT_STD_PROCINFO_COMPUTEFUNC_START


    // Check if DM needs updating
    // DMupdate toggles to 1 if DM must be updated
    //
    int DMupdate = 0;
    {
        long cnt0sum = 0;

        for (uint32_t ch = 0; ch < *NBchannel; ch++)
        {
            cnt0sum += imgch[ch].md->cnt0;
        }

        if (cnt0sum != cntsumref)
        {
            printf("cnt0sum = %ld\n", cnt0sum);
            cntsumref = cnt0sum;
            DMupdate  = 1;
        }
    }


    if (DMupdate == 1)
    {
        // Update DM disp

        // Sum all channels
        //
        memcpy(dmdisptmp,
               imgch[0].im->array.F,
               sizeof(float) * (*DMxsize) * (*DMysize));
        for (uint32_t ch = 1; ch < *NBchannel; ch++)
        {
            for (uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
            {
                dmdisptmp[ii] += imgch[ch].im->array.F[ii];
            }
        }

        // Remove average
        //
        double ave = 0.0;
        if (*AveMode == 1)
        {
            for (uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
            {
                ave += dmdisptmp[ii];
            }
            ave /= (*DMxsize) * (*DMysize);
        }

        if (*AveMode < 2) // OFFSET BY DClevel
        {
            for (uint_fast64_t ii = 0; ii < (*DMxsize) * (*DMysize); ii++)
            {
                dmdisptmp[ii] += (*DClevel - ave);

                // remove negative values
                if (*voltmode == 1)
                    if (dmdisptmp[ii] < 0.0)
                    {
                        dmdisptmp[ii] = 0.0;
                    }
            }
        }

        memcpy(imgdisp.im->array.F,
               dmdisptmp,
               sizeof(float) * (*DMxsize) * (*DMysize));

        processinfo_update_output_stream(processinfo, imgdisp.ID);


        if (*voltmode == 1)
        {
            DM_displ2V(imgdisp, imgdmvolt);
            processinfo_update_output_stream(processinfo, imgdmvolt.ID);
        }

        if ((*dmdispcircbuff) == 1)
        {
            DMdisp_add_disp_from_circular_buffer(imgch[(*dmdispCBchan)]);
            processinfo_update_output_stream(processinfo,
                                             imgch[(*dmdispCBchan)].ID);
        }
    }


    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(dmdisptmp);

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
