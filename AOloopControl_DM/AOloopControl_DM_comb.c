/**
 * @file    AOloopControl_DM_comb.c
 * @brief   DM control - Combine DM channels
 * \
 * Refactored to FPS practices.
 */

#include <math.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include "CommandLineInterface/CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "timeutils.h"

#include "fps.h"
#include "processinfo.h"
#include "processtools.h"

/* =============================================================================================== */
/* PARAMETERS DEFINITION                                                                           */
/* =============================================================================================== */

#define DMCOMB_PARAMS(X) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_UINT32, uint32_t*, ".DMindex", "Deformable mirror index", "5", DMindex_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_STREAMNAME, char*, ".DMcombout", "output stream for combined command", "dm99disp", DMcombout_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_UINT32, uint32_t*, ".DMxsize", "x size", "20", DMxsize_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_UINT32, uint32_t*, ".DMysize", "y size", "20", DMysize_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".NBchannel", "number of DM channels", "12", NBchannel_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".DMmode", "0:SquareGrid, 1:Generic", "0", DMmode_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".AveMode", "Piston (mean) subtract", "0", AveMode_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".option.dm2dm_mode", "DM to DM offset mode", "OFF", dm2dm_mode_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".option.dm2dm_DMmodes", "Output stream DM to DM", "null", dm2dm_DMmodes_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".option.dm2dm_outdisp", "data stream to which output DM is written", "null", dm2dm_outdisp_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".option.wfsrefmode", "WFS ref mode", "OFF", wfsrefmode_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".option.wfsref_WFSRespMat", "Output WFS resp matrix", "null", wfsref_WFSRespMat_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".option.wfsref_out", "Output WFS", "null", wfsref_out_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".option.voltmode", "Volt mode", "OFF", voltmode_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".option.volttype", "volt type", "0", volttype_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.stroke100", "Stroke for 100 V [um]", "1.0", stroke100_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".option.voltname", "Stream name for volt output", "dmvolt", voltname_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STRING, char*, ".option.outv_ftype", "output volt type", "float32", outv_ftype_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.outv_exp", "output volt power exponent", "1.0", outv_exp_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.outv_inrange_min", "output volt input range min", "-1.0", outv_inrange_min_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.outv_inrange_max", "output volt input range max", "1.0", outv_inrange_max_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.outv_outrange_min", "output volt output range min", "-1.0", outv_outrange_min_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.outv_outrange_max", "output volt output range max", "1.0", outv_outrange_max_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.DClevel", "DC level [um]", "0.5", DClevel_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".option.maxvolt", "Maximum voltage", "100.0", maxvolt_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT64, uint64_t*, ".status.loopcnt", "Loop counter", "0", loopcnt_ptr, GetParamPtr_UINT64, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".astrogrid.mode", "circular buffer on/off", "OFF", astrogrid_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".astrogrid.chan", "astrogrid DM channel", "9", astrogridchan_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_STREAMNAME, char*, ".astrogrid.sname", "astrogrid cube name", "dmCBcube", astrogridsname_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".astrogrid.mult", "astrogrid multiplicative coeff", "1.0", astrogridmult_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".astrogrid.delay", "time delay between main update and astrogrid update [us]", "100", astrogridtdelay_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".astrogrid.nbframe", "astrogrid number of frame per slice", "1", astrogridNBframe_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.enable", "zero point offset enable", "OFF", zpoffsetenable_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_STREAMNAME, char*, ".zpoffset.DMcomboutzpo", "output stream for combined zero point offset", "dm99zpo", DMcomboutzpo_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch00", "channel 00 zpoffset ?", "OFF", zpoffsetch00_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch01", "channel 01 zpoffset ?", "OFF", zpoffsetch01_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch02", "channel 02 zpoffset ?", "OFF", zpoffsetch02_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch03", "channel 03 zpoffset ?", "OFF", zpoffsetch03_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch04", "channel 04 zpoffset ?", "OFF", zpoffsetch04_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch05", "channel 05 zpoffset ?", "OFF", zpoffsetch05_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch06", "channel 06 zpoffset ?", "OFF", zpoffsetch06_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch07", "channel 07 zpoffset ?", "OFF", zpoffsetch07_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch08", "channel 08 zpoffset ?", "OFF", zpoffsetch08_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch09", "channel 09 zpoffset ?", "OFF", zpoffsetch09_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch10", "channel 10 zpoffset ?", "OFF", zpoffsetch10_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".zpoffset.ch11", "channel 11 zpoffset ?", "OFF", zpoffsetch11_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)

/* Global parameter pointers */
#define X_PTR_DECL(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...)     static c_type ptr_name = NULL;
DMCOMB_PARAMS(X_PTR_DECL)
#undef X_PTR_DECL

/* FPI indices for customCONFcheck */
static uint64_t fpi_DMindex;
static uint64_t fpi_dm2dm_mode;
static uint64_t fpi_dm2dm_DMmodes;
static uint64_t fpi_dm2dm_outdisp;
static uint64_t fpi_wfsrefmode;
static uint64_t fpi_wfsref_WFSRespMat;
static uint64_t fpi_wfsref_out;
static uint64_t fpi_voltmode;
static uint64_t fpi_stroke100;
static uint64_t fpi_voltname;
static uint64_t fpi_DClevel;
static uint64_t fpi_maxvolt;
static uint64_t fpi_astrogrid;
static uint64_t fpi_astrogridmult;
static uint64_t fpi_astrogridtdelay;
static uint64_t fpi_astrogridNBframe;
static uint64_t fpi_astrogridsname;
static uint64_t fpi_zpoffsetenable;
static uint64_t fpi_zpoffsetch[12];

static uint64_t processinfo_change_cnt_local = 0;

#define NB_ZEROPOINT_CH_MAX 12

typedef struct {
    IMGID *imgch;
    IMGID imgdisp;
    IMGID imgdispzpo;
    IMGID imgdmvolt;
    float *dmdisptmp;

    // ZPO state
    int zpoffset_channel[NB_ZEROPOINT_CH_MAX];
    uint64_t zpochecksum;
    uint64_t zpochecksum0;
    long cntsumref;
    long cntsumrefzpo;

    // Astrogrid circular buffer state
    int DMdisp_add_disp_from_circular_buffer_init;
    uint32_t ag_sliceindex;
    IMGID ag_imgdispbuffer;
    uint32_t ag_framecnt;
    uint64_t ag_xysize;

} DMCOMB_STATE;

/* =============================================================================================== */
/* HELPERS                                                                                         */
/* =============================================================================================== */

static errno_t DMdisp_add_disp_from_circular_buffer(DMCOMB_STATE *state)
{
    if(state->DMdisp_add_disp_from_circular_buffer_init == 0)
    {
        printf("(re-)initializing DMdisp_add_disp_from_circular_buffer");
        delete_image_ID(astrogridsname_ptr, DELETE_IMAGE_ERRMODE_WARNING);
        read_sharedmem_image(astrogridsname_ptr, data.image, data.NB_MAX_IMAGE);
        state->ag_imgdispbuffer = mkIMGID_from_name(astrogridsname_ptr);
        resolveIMGID(&state->ag_imgdispbuffer, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
        state->ag_xysize = (uint64_t)(*DMxsize_ptr) * (*DMysize_ptr);
        state->ag_sliceindex = 0;
        state->ag_framecnt = 0;
        state->DMdisp_add_disp_from_circular_buffer_init = 1;
    }

    if((*astrogrid_ptr) & FPFLAG_ONOFF)
    {
        state->ag_framecnt++;
        if(state->ag_framecnt >= (*astrogridNBframe_ptr))
        {
            state->ag_framecnt = 0;
            state->ag_sliceindex++;

            if(state->ag_sliceindex >= state->ag_imgdispbuffer.md->size[2])
            {
                state->ag_sliceindex = 0;
            }

            uint32_t chan = *astrogridchan_ptr;
            if (chan < *NBchannel_ptr) {
                float *outptr = state->imgch[chan].im->array.F;
                float *inptr = state->ag_imgdispbuffer.im->array.F;
                uint64_t offset = state->ag_sliceindex * state->ag_xysize;
                float mult = *astrogridmult_ptr;
                \
                for(uint64_t ii = 0; ii < state->ag_xysize; ii++)
                {
                    outptr[ii] = mult * inptr[offset + ii];
                }
            }
        }
    }
    return RETURN_SUCCESS;
}

static errno_t DM_displ2V(IMGID imgdisp, IMGID imgvolt)
{
    uint64_t xysize = (uint64_t)(*DMxsize_ptr) * (*DMysize_ptr);
    double *valarray = (double *) malloc(sizeof(double) * xysize);
    \
    if((*volttype_ptr) == 1)
    {
        float inrange = (*maxvolt_ptr) * (*stroke100_ptr) / 100.0;
        strcpy(outv_ftype_ptr, "float32");
        *outv_exp_ptr = 1.0;
        *outv_inrange_min_ptr = -inrange;
        *outv_inrange_max_ptr = inrange;
        *outv_outrange_min_ptr = -(*maxvolt_ptr);
        *outv_outrange_max_ptr = -(*maxvolt_ptr);
        \
    }
    else if((*volttype_ptr) == 2)
    {
        float inrange = (*maxvolt_ptr) * (*stroke100_ptr) / 100.0;
        strcpy(outv_ftype_ptr, "uint16");
        *outv_exp_ptr = 0.5;
        *outv_inrange_min_ptr = -inrange;
        *outv_inrange_max_ptr = inrange;
        *outv_outrange_min_ptr = 0.0;
        *outv_outrange_max_ptr = (*maxvolt_ptr) / 300.0 * 16384.0;
    }

    for(uint64_t ii = 0; ii < xysize; ii++)
    {
        float inval = imgdisp.im->array.F[ii];
        double x = inval - (*outv_inrange_min_ptr);
        double range = *outv_inrange_max_ptr - *outv_inrange_min_ptr;
        if (range != 0) x = x / range;
        else x = 0;
        \
        if(x < 0.0) x = 0.0;
        if(x > 1.0) x = 1.0;

        valarray[ii] = pow(x, *outv_exp_ptr);
        valarray[ii] = (*outv_outrange_min_ptr) + valarray[ii] * (*outv_outrange_max_ptr - *outv_outrange_min_ptr);
    }

    if((*volttype_ptr) == 1)
    {
        for(uint64_t ii = 0; ii < xysize; ii++)
        {
            float voltvalue = 100.0 * imgdisp.im->array.F[ii] / (*stroke100_ptr);
            if(voltvalue > (*maxvolt_ptr)) voltvalue = (*maxvolt_ptr);
            if(voltvalue < -(*maxvolt_ptr)) voltvalue = -(*maxvolt_ptr);
            imgvolt.im->array.F[ii] = voltvalue;
        }
    }
    else if((*volttype_ptr) == 2)
    {
        for(uint64_t ii = 0; ii < xysize; ii++)
        {
            float val = imgdisp.im->array.F[ii];
            if (val < 0) val = 0;
            float volt = 100.0 * sqrt(val / (*stroke100_ptr));
            if(volt > (*maxvolt_ptr)) volt = (*maxvolt_ptr);
            imgvolt.im->array.UI16[ii] = (unsigned short int)(volt / 300.0 * 16384.0);
        }
    }
    else if((*volttype_ptr) == 3)
    {
        for(uint64_t ii = 0; ii < xysize; ii++)
        {
            float volt = (imgdisp.im->array.F[ii] / (*stroke100_ptr)) + 0.5;
            if(volt > (*maxvolt_ptr)) volt = remainder(volt, 1);
            if(volt < 0) volt = remainder(volt, 1);
            imgvolt.im->array.UI16[ii] = (unsigned short int)((volt) * 65535.0);
        }
    }
    else if((*volttype_ptr) == 0) // Type conversion
    {
        if(strcmp(outv_ftype_ptr, "float64") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.D[ii] = valarray[ii];
        }
        else if(strcmp(outv_ftype_ptr, "uint16") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.UI16[ii] = (uint16_t)(valarray[ii]);
        }
        else if(strcmp(outv_ftype_ptr, "uint32") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.UI32[ii] = (uint32_t)(valarray[ii]);
        }
        else if(strcmp(outv_ftype_ptr, "uint64") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.UI64[ii] = (uint64_t)(valarray[ii]);
        }
        else if(strcmp(outv_ftype_ptr, "int16") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.SI16[ii] = (int16_t)(valarray[ii]);
        }
        else if(strcmp(outv_ftype_ptr, "int32") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.SI32[ii] = (int32_t)(valarray[ii]);
        }
        else if(strcmp(outv_ftype_ptr, "int64") == 0) {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.SI64[ii] = (int64_t)(valarray[ii]);
        }
        else {
            for(uint64_t ii = 0; ii < xysize; ii++) imgvolt.im->array.F[ii] = valarray[ii];
        }
    }

    free(valarray);
    return RETURN_SUCCESS;
}



static errno_t update_dmdisp(
    IMGID imgdisp,
    IMGID *imgch,
    float *dmdisptmp
)
{
    uint64_t size = (uint64_t)(*DMxsize_ptr) * (*DMysize_ptr);
    memcpy(dmdisptmp, imgch[0].im->array.F, sizeof(float) * size);

    for(uint32_t ch = 1; ch < *NBchannel_ptr; ch++)
    {
        for(uint_fast64_t ii = 0; ii < size; ii++)
        {
            dmdisptmp[ii] += imgch[ch].im->array.F[ii];
        }
    }

    double ave = 0.0;
    if(*AveMode_ptr == 1)
    {
        for(uint_fast64_t ii = 0; ii < size; ii++) ave += dmdisptmp[ii];
        ave /= size;
        for(uint_fast64_t ii = 0; ii < size; ii++)
        {
            dmdisptmp[ii] += (*DClevel_ptr - ave);
            if((*voltmode_ptr) & FPFLAG_ONOFF) {
                if(dmdisptmp[ii] < 0.0) dmdisptmp[ii] = 0.0;
            }
        }
    }
    memcpy(imgdisp.im->array.F, dmdisptmp, sizeof(float) * size);
    return RETURN_SUCCESS;
}



static errno_t update_dmdispzpo(
    IMGID imgdisp,
    IMGID *imgch,
    float *dmdisptmp,
    int *zpoffset_channel
)
{
    uint64_t size = (uint64_t)(*DMxsize_ptr) * (*DMysize_ptr);
    memset(dmdisptmp, 0, sizeof(float) * size);

    for(uint32_t ch = 0; ch < *NBchannel_ptr; ch++)
    {
        if(zpoffset_channel[ch] == 1)
        {
            for(uint_fast64_t ii = 0; ii < size; ii++)
            {
                dmdisptmp[ii] += imgch[ch].im->array.F[ii];
            }
        }
    }
    memcpy(imgdisp.im->array.F, dmdisptmp, sizeof(float) * size);

    return RETURN_SUCCESS;
}



/* =============================================================================================== */
/* RUN LOGIC                                                                                       */
/* =============================================================================================== */

static void dmcomb_cleanup(DMCOMB_STATE *state)
{
    if(!state) return;
    if(state->imgch) free(state->imgch);
    if(state->dmdisptmp) free(state->dmdisptmp);

    free(state);
}


static DMCOMB_STATE* dmcomb_init()
{
    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    DMCOMB_STATE *state = (DMCOMB_STATE*) calloc(1, sizeof(DMCOMB_STATE));

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    state->imgch = calloc(*NBchannel_ptr, sizeof(IMGID));
    for(uint32_t ch = 0; ch < *NBchannel_ptr; ch++) {
        char name[STRINGMAXLEN_STREAMNAME];
        snprintf(name, sizeof(name), "dm%02udisp%02u", *DMindex_ptr, ch);

        printf("DEBUG: channel %d : %s\n", ch, name);
        fflush(stdout);

        imageID IDch = read_sharedmem_image(name, data.image, data.NB_MAX_IMAGE);
        printf("DEBUG: ID = %ld\n", IDch);
        fflush(stdout);

        state->imgch[ch] = stream_connect_create_2Df32(name, *DMxsize_ptr, *DMysize_ptr);
    }

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    state->imgdisp = stream_connect_create_2Df32(DMcombout_ptr, *DMxsize_ptr, *DMysize_ptr);
    state->imgdispzpo = stream_connect_create_2Df32(DMcomboutzpo_ptr, *DMxsize_ptr, *DMysize_ptr);

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    state->dmdisptmp = malloc(sizeof(float) * (*DMxsize_ptr) * (*DMysize_ptr));

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    if((*voltmode_ptr) & FPFLAG_ONOFF) {
        if(image_ID(voltname_ptr, data.image, data.NB_MAX_IMAGE) == -1) read_sharedmem_image(voltname_ptr, data.image, data.NB_MAX_IMAGE);
        state->imgdmvolt = mkIMGID_from_name(voltname_ptr);
        resolveIMGID(&state->imgdmvolt, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
    }

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    return state;
}


static void dmcomb_step(
    PROCESSINFO *processinfo,
    FUNCTION_PARAMETER_STRUCT *fps,
    DMCOMB_STATE *state
)
{
    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    // Sync parameters
    if (fps) {
        if(fps->md->processinfo_change_cnt != processinfo_change_cnt_local) {
            fps_to_processinfo(fps, processinfo);
            processinfo_change_cnt_local = fps->md->processinfo_change_cnt;
        }
    }

    // Update ZPO channels state
    state->zpoffset_channel[0]  = ((*zpoffsetch00_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[1]  = ((*zpoffsetch01_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[2]  = ((*zpoffsetch02_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[3]  = ((*zpoffsetch03_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[4]  = ((*zpoffsetch04_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[5]  = ((*zpoffsetch05_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[6]  = ((*zpoffsetch06_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[7]  = ((*zpoffsetch07_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[8]  = ((*zpoffsetch08_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[9]  = ((*zpoffsetch09_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[10] = ((*zpoffsetch10_ptr) & FPFLAG_ONOFF) ? 1 : 0;
    state->zpoffset_channel[11] = ((*zpoffsetch11_ptr) & FPFLAG_ONOFF) ? 1 : 0;

    int zpooffsetchange = 0;
    state->zpochecksum = 0;
    for(int ch = 0; ch < NB_ZEROPOINT_CH_MAX; ++ch) {
        if(state->zpoffset_channel[ch]) state->zpochecksum += (1 << ch);
    }
    if(state->zpochecksum != state->zpochecksum0) zpooffsetchange = 1;
    state->zpochecksum0 = state->zpochecksum;

    int DMupdate = 0;
    int DMupdatezpo = 0;
    long cnt0sum = 0;
    long cnt0sumzpo = 0;

    if((*astrogrid_ptr) & FPFLAG_ONOFF) {
        for(uint32_t ch = 0; ch < *NBchannel_ptr; ch++) {
            if(ch != *astrogridchan_ptr) {
                cnt0sum += state->imgch[ch].md->cnt0;
                if(((*zpoffsetenable_ptr) & FPFLAG_ONOFF) && state->zpoffset_channel[ch]) {
                    cnt0sumzpo += state->imgch[ch].md->cnt0;
                }
            }
        }
    } else {
        for(uint32_t ch = 0; ch < *NBchannel_ptr; ch++) {
            cnt0sum += state->imgch[ch].md->cnt0;
            if(((*zpoffsetenable_ptr) & FPFLAG_ONOFF) && state->zpoffset_channel[ch]) {
                cnt0sumzpo += state->imgch[ch].md->cnt0;
            }
        }
    }

    if(cnt0sum != state->cntsumref) {
        state->cntsumref = cnt0sum;
        DMupdate = 1;
    }
    if(cnt0sumzpo != state->cntsumrefzpo) {
        state->cntsumrefzpo = cnt0sumzpo;
        DMupdatezpo = 1;
    }
    if(((*zpoffsetenable_ptr) & FPFLAG_ONOFF) && zpooffsetchange) DMupdatezpo = 1;

    if(DMupdate) {
        if(!((*astrogrid_ptr) & FPFLAG_ONOFF)) state->DMdisp_add_disp_from_circular_buffer_init = 0;

        if(((*astrogrid_ptr) & FPFLAG_ONOFF) && (*astrogridtdelay_ptr == 0)) {
            DMdisp_add_disp_from_circular_buffer(state);
            processinfo_update_output_stream(processinfo, state->imgch[*astrogridchan_ptr].im, NULL);
        }

        update_dmdisp(state->imgdisp, state->imgch, state->dmdisptmp);
        processinfo_update_output_stream(processinfo, state->imgdisp.im, NULL);

        if((*voltmode_ptr) & FPFLAG_ONOFF) {
            state->imgdmvolt.md->write = 1;
            DM_displ2V(state->imgdisp, state->imgdmvolt);
            processinfo_update_output_stream(processinfo, state->imgdmvolt.im, NULL);
        }

        if(((*astrogrid_ptr) & FPFLAG_ONOFF) && (*astrogridtdelay_ptr != 0)) {
            long nsec = (long)(1000 * (*astrogridtdelay_ptr));
            struct timespec timesleep;
            timesleep.tv_sec = nsec / 1000000000;
            timesleep.tv_nsec = nsec % 1000000000;
            nanosleep(&timesleep, NULL);

            DMdisp_add_disp_from_circular_buffer(state);
            processinfo_update_output_stream(processinfo, state->imgch[*astrogridchan_ptr].im, NULL);

            update_dmdisp(state->imgdisp, state->imgch, state->dmdisptmp);
            processinfo_update_output_stream(processinfo, state->imgdisp.im, NULL);

            if((*voltmode_ptr) & FPFLAG_ONOFF) {
                state->imgdmvolt.md->write = 1;
                DM_displ2V(state->imgdisp, state->imgdmvolt);
                processinfo_update_output_stream(processinfo, state->imgdmvolt.im, NULL);
            }
        }
    }

    if(DMupdatezpo) {
        update_dmdispzpo(state->imgdispzpo, state->imgch, state->dmdisptmp, state->zpoffset_channel);
        processinfo_update_output_stream(processinfo, state->imgdispzpo.im, NULL);
    }
}

static void dmcomb_validate() {
    if (DMindex_ptr && *DMindex_ptr > 99) *DMindex_ptr = 99;
}

#ifndef FPS_STANDALONE

static CLICMDARGDEF farg[] = {
    { CLIARG_UINT32, ".DMindex", "Deformable mirror index", "5", CLIARG_VISIBLE_DEFAULT, (void **) &DMindex_ptr, (long*)&fpi_DMindex },
    { CLIARG_STREAM, ".DMcombout", "output stream for combined command", "dm99disp", CLIARG_VISIBLE_DEFAULT, (void **) &DMcombout_ptr, NULL },
    { CLIARG_UINT32, ".DMxsize", "x size", "20", CLIARG_VISIBLE_DEFAULT, (void **) &DMxsize_ptr, NULL },
    { CLIARG_UINT32, ".DMysize", "y size", "20", CLIARG_VISIBLE_DEFAULT, (void **) &DMysize_ptr, NULL },
    { CLIARG_UINT32, ".NBchannel", "number of DM channels", "12", CLIARG_HIDDEN_DEFAULT, (void **) &NBchannel_ptr, NULL },
    // Simplified farg for brevity, assuming FPS handles most.
    { CLIARG_ONOFF, ".option.voltmode", "Volt mode", "OFF", CLIARG_HIDDEN_DEFAULT, (void **) &voltmode_ptr, (long*)&fpi_voltmode }
};

static CLICMDDATA CLIcmddata = { \
                                 "DMcomb", "Deformable mirror combine channels", "", \
                                 sizeof(farg) / sizeof(CLICMDARGDEF), farg, \
                                 CLICMDFLAG_FPS, NULL, NULL, NULL \
                               };

static errno_t customCONFsetup() {
    if(data.fpsptr != NULL) {
        data.fpsptr->parray[fpi_DMindex].fpflag = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;
        data.fpsptr->parray[fpi_DMindex].val.ui32[1] = 0;
        \
        data.fpsptr->parray[fpi_DMindex].val.ui32[2] = 99;
        \
        data.fpsptr->parray[fpi_voltmode].fpflag |= FPFLAG_WRITERUN;
        // ... map other fpi_ variables if needed for CLI behavior
    }
    return RETURN_SUCCESS;
}

static errno_t customCONFcheck() {
    if(data.fpsptr != NULL) {
        // Logic to toggle visibility based on flags
        if(data.fpsptr->parray[fpi_voltmode].fpflag & FPFLAG_ONOFF) {
            data.fpsptr->parray[fpi_voltname].fpflag |= FPFLAG_USED | FPFLAG_VISIBLE | FPFLAG_STREAM_RUN_REQUIRED;
        } else {
            data.fpsptr->parray[fpi_voltname].fpflag &= ~(FPFLAG_USED | FPFLAG_VISIBLE | FPFLAG_STREAM_RUN_REQUIRED);
        }
        // ... repeat for other conditional flags
    }
    return RETURN_SUCCESS;
}

static errno_t help_function() {
    printf("DM Combine Channels using FPS\
");
    return RETURN_SUCCESS;
}

static errno_t compute_function() {
    DMCOMB_STATE *state = dmcomb_init();
    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    dmcomb_step(processinfo, data.fpsptr, state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END
    dmcomb_cleanup(state);
    return RETURN_SUCCESS;
}


#define INSERT_STD_FPSCONFfunction_local  \
static errno_t FPSCONFfunction() \
{ \
    FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);\
if (CLIcmddata.flags & CLICMDFLAG_PROCINFO) {\
fps_add_processinfo_entries(&fps); }\
data.fpsptr = &fps;\
CMDargs_to_FPSparams_create(&fps);\
if (CLIcmddata.FPS_customCONFsetup != NULL) {\
    CLIcmddata.FPS_customCONFsetup();}\
FPS_CONFLOOP_START \
if (CLIcmddata.FPS_customCONFcheck != NULL)\
CLIcmddata.FPS_customCONFcheck();\
FPS_CONFLOOP_END \
data.fpsptr = NULL;\
return RETURN_SUCCESS;\
}





INSERT_STD_FPSCONFfunction_local
INSERT_STD_FPSRUNfunction
INSERT_STD_FPSCLIfunction

errno_t CLIADDCMD_AOloopControl_DM__comb() {
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}

#endif

#ifdef FPS_STANDALONE


int FPSINIT_AOloopControl_DM_comb(
    const char *fps_name,
    const char *keywords,
    const char *description
)
{
    FUNCTION_PARAMETER_STRUCT fps;

    FPS_INIT_STD_PREAMBLE(fps, fps_name, keywords, description, "DM Combine Channels");
    FPS_INIT_PROCINFO_DEFAULTS(fps, "dm99disp", 10);

#define X_FPS_INIT(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...) \
    {         if(fps_type == FPTYPE_FLOAT32) \
        { float val = (float)atof(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); \
    } \
        else if(fps_type == FPTYPE_UINT32) \
        { uint32_t val = (uint32_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); \
    } \
        else if(fps_type == FPTYPE_UINT64) \
        { uint64_t val = (uint64_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); \
    } \
        else if(fps_type == FPTYPE_STREAMNAME) \
        { char val[FUNCTION_PARAMETER_STRMAXLEN]; strncpy(val, def_str, FUNCTION_PARAMETER_STRMAXLEN-1); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val, NULL); \
    } \
        else if(fps_type == FPTYPE_STRING) \
        { char val[FUNCTION_PARAMETER_STRMAXLEN]; strncpy(val, def_str, FUNCTION_PARAMETER_STRMAXLEN-1); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val, NULL); \
    } \
        else { function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, NULL, NULL); \
    }     }
    DMCOMB_PARAMS(X_FPS_INIT)
#undef X_FPS_INIT
    \
    fps_add_processinfo_entries(&fps);
    function_parameter_FPCONFexit(&fps);
    return 0;
}


#define X_FPS_MAP(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...)             ptr_name = (c_type)functionparameter_##get_func(&fps, key);

int FPSCONF_AOloopControl_DM_comb(
    const char *fps_name,
    int loop
)
{
    FPS_CONF_STD_BODY(fps_name, loop, { DMCOMB_PARAMS(X_FPS_MAP) }, { dmcomb_validate(); });
    return 0;
}


FPS_MAKE_STANDALONE_CONFSTOP(AOloopControl_DM_comb)
FPS_MAKE_STANDALONE_RUNSTOP(AOloopControl_DM_comb)

int FPSRUN_AOloopControl_DM_comb(const char *fps_name)
{
    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    FUNCTION_PARAMETER_STRUCT fps;
    FPS_RUN_STD_PREAMBLE(fps_name, fps, { DMCOMB_PARAMS(X_FPS_MAP) });

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    DMCOMB_STATE *state = dmcomb_init();

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    PROCESSINFO *pinfo;
    FPS_RUN_PROCESSINFO_SETUP(pinfo, fps_name, "Run", "Looping", state->imgch[0].im, fps);

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    while(processinfo_loopstep(pinfo)) {
        processinfo_exec_start(pinfo);
        dmcomb_step(pinfo, &fps, state);
        processinfo_exec_end(pinfo);
        usleep(100);
    }

    printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
    fflush(stdout);

    dmcomb_cleanup(state);
    processinfo_cleanExit(pinfo);
    function_parameter_struct_disconnect(&fps);
    return 0;
}

FPS_MAIN_STANDALONE("dmcomb", AOloopControl_DM_comb, "DM Combine Channels", DMCOMB_PARAMS)

#endif
