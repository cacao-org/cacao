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

/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "dmcomb",
    .cmdkey      = "dmcomb",
    .description = "Combine DM channels"
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint32_t *DMindex_ptr          = NULL;
static char     *DMcombout_ptr        = NULL;
static uint32_t *DMxsize_ptr          = NULL;
static uint32_t *DMysize_ptr          = NULL;
static uint32_t *NBchannel_ptr        = NULL;
static uint32_t *DMmode_ptr           = NULL;
static uint32_t *AveMode_ptr          = NULL;
static uint64_t *dm2dm_mode_ptr       = NULL;
static char     *dm2dm_DMmodes_ptr    = NULL;
static char     *dm2dm_outdisp_ptr    = NULL;
static uint64_t *wfsrefmode_ptr       = NULL;
static char     *wfsref_WFSRespMat_ptr = NULL;
static char     *wfsref_out_ptr       = NULL;
static uint64_t *voltmode_ptr         = NULL;
static uint32_t *volttype_ptr         = NULL;
static float    *stroke100_ptr        = NULL;
static char     *voltname_ptr         = NULL;
static char     *outv_ftype_ptr       = NULL;
static float    *outv_exp_ptr         = NULL;
static float    *outv_inrange_min_ptr = NULL;
static float    *outv_inrange_max_ptr = NULL;
static float    *outv_outrange_min_ptr = NULL;
static float    *outv_outrange_max_ptr = NULL;
static float    *DClevel_ptr          = NULL;
static float    *maxvolt_ptr          = NULL;
static uint64_t *loopcnt_ptr          = NULL;
static uint64_t *astrogrid_ptr        = NULL;
static uint32_t *astrogridchan_ptr    = NULL;
static char     *astrogridsname_ptr   = NULL;
static float    *astrogridmult_ptr    = NULL;
static uint32_t *astrogridtdelay_ptr  = NULL;
static uint32_t *astrogridNBframe_ptr = NULL;
static uint64_t *zpoffsetenable_ptr   = NULL;
static char     *DMcomboutzpo_ptr     = NULL;
static uint64_t *zpoffsetch00_ptr     = NULL;
static uint64_t *zpoffsetch01_ptr     = NULL;
static uint64_t *zpoffsetch02_ptr     = NULL;
static uint64_t *zpoffsetch03_ptr     = NULL;
static uint64_t *zpoffsetch04_ptr     = NULL;
static uint64_t *zpoffsetch05_ptr     = NULL;
static uint64_t *zpoffsetch06_ptr     = NULL;
static uint64_t *zpoffsetch07_ptr     = NULL;
static uint64_t *zpoffsetch08_ptr     = NULL;
static uint64_t *zpoffsetch09_ptr     = NULL;
static uint64_t *zpoffsetch10_ptr     = NULL;
static uint64_t *zpoffsetch11_ptr     = NULL;

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
        state->ag_imgdispbuffer = imgid_make_from_name(astrogridsname_ptr);
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
        state->imgdmvolt = imgid_make_from_name(voltname_ptr);
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




/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".DMindex", &DMindex_ptr, \
      FPTYPE_UINT32, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_MINLIMIT \
          | FPFLAG_MAXLIMIT, \
      "DM index") \
    X(".DMcombout", &DMcombout_ptr, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT, \
      "output combined cmd stream") \
    X(".DMxsize", &DMxsize_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "x size") \
    X(".DMysize", &DMysize_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "y size") \
    X(".NBchannel", &NBchannel_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "number of DM channels") \
    X(".DMmode", &DMmode_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "0:SquareGrid 1:Generic") \
    X(".AveMode", &AveMode_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Piston (mean) subtract") \
    X(".option.dm2dm_mode", \
      &dm2dm_mode_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "DM to DM offset mode") \
    X(".option.dm2dm_DMmodes", \
      &dm2dm_DMmodes_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "output stream DM to DM") \
    X(".option.dm2dm_outdisp", \
      &dm2dm_outdisp_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "DM output data stream") \
    X(".option.wfsrefmode", \
      &wfsrefmode_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "WFS ref mode") \
    X(".option.wfsref_WFSRespMat", \
      &wfsref_WFSRespMat_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Output WFS resp matrix") \
    X(".option.wfsref_out", \
      &wfsref_out_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Output WFS") \
    X(".option.voltmode", \
      &voltmode_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Volt mode") \
    X(".option.volttype", \
      &volttype_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "volt type") \
    X(".option.stroke100", \
      &stroke100_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Stroke for 100V [um]") \
    X(".option.voltname", \
      &voltname_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Stream name volt output") \
    X(".option.outv_ftype", \
      &outv_ftype_ptr, \
      FPTYPE_STRING, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "output volt type") \
    X(".option.outv_exp", \
      &outv_exp_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "output volt power exp") \
    X(".option.outv_inrange_min", \
      &outv_inrange_min_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "inrange min") \
    X(".option.outv_inrange_max", \
      &outv_inrange_max_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "inrange max") \
    X(".option.outv_outrange_min", \
      &outv_outrange_min_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "outrange min") \
    X(".option.outv_outrange_max", \
      &outv_outrange_max_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "outrange max") \
    X(".option.DClevel", \
      &DClevel_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "DC level [um]") \
    X(".option.maxvolt", \
      &maxvolt_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Maximum voltage") \
    X(".status.loopcnt", \
      &loopcnt_ptr, \
      FPTYPE_UINT64, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "Loop counter") \
    X(".astrogrid.mode", \
      &astrogrid_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "circular buffer on/off") \
    X(".astrogrid.chan", \
      &astrogridchan_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "astrogrid DM channel") \
    X(".astrogrid.sname", \
      &astrogridsname_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "astrogrid cube name") \
    X(".astrogrid.mult", \
      &astrogridmult_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "astrogrid mult coeff") \
    X(".astrogrid.delay", \
      &astrogridtdelay_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "astrogrid delay [us]") \
    X(".astrogrid.nbframe", \
      &astrogridNBframe_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "nb frame per slice") \
    X(".zpoffset.enable", \
      &zpoffsetenable_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "zero point offset enable") \
    X(".zpoffset.DMcomboutzpo", \
      &DMcomboutzpo_ptr, \
      FPTYPE_STREAMNAME, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "output combined ZPO") \
    X(".zpoffset.ch00", \
      &zpoffsetch00_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 00 zpoffset") \
    X(".zpoffset.ch01", \
      &zpoffsetch01_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 01 zpoffset") \
    X(".zpoffset.ch02", \
      &zpoffsetch02_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 02 zpoffset") \
    X(".zpoffset.ch03", \
      &zpoffsetch03_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 03 zpoffset") \
    X(".zpoffset.ch04", \
      &zpoffsetch04_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 04 zpoffset") \
    X(".zpoffset.ch05", \
      &zpoffsetch05_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 05 zpoffset") \
    X(".zpoffset.ch06", \
      &zpoffsetch06_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 06 zpoffset") \
    X(".zpoffset.ch07", \
      &zpoffsetch07_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 07 zpoffset") \
    X(".zpoffset.ch08", \
      &zpoffsetch08_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 08 zpoffset") \
    X(".zpoffset.ch09", \
      &zpoffsetch09_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 09 zpoffset") \
    X(".zpoffset.ch10", \
      &zpoffsetch10_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 10 zpoffset") \
    X(".zpoffset.ch11", \
      &zpoffsetch11_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "channel 11 zpoffset")


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

static FPS_CLI_BINDING my_bindings[] = {
    FPS_PARAMS(FPS_X_BINDING)
};

static const int nb_bindings =
    sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

#ifdef FPS_STANDALONE
CLICMDDATA CLIcmddata = {
#else
static CLICMDDATA CLIcmddata = {
#endif
    "",
    "",
    CLICMD_FIELDS_DEFAULTS
};

static CMDSETTINGS default_cmdsettings = {0};

static __attribute__((constructor))
void init_cmdsettings(void)
{
    strncpy(CLIcmddata.key,
            FPS_app_info.cmdkey,
            sizeof(CLIcmddata.key) - 1);
    strncpy(CLIcmddata.description,
            FPS_app_info.description,
            sizeof(CLIcmddata.description) - 1);
    if (CLIcmddata.cmdsettings == NULL) {
        CLIcmddata.cmdsettings =
            &default_cmdsettings;
    }
}


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static errno_t compute_function()
{
    DMCOMB_STATE *state = dmcomb_init();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    dmcomb_step(processinfo, data.fpsptr,
        state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    dmcomb_cleanup(state);
    return RETURN_SUCCESS;
}


/* ================================================================
 * 7.  MILK MODULE REGISTRATION
 * ============================================================= */

#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

errno_t
CLIADDCMD_AOloopControl_DM__comb()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif


/* ================================================================
 * 8.  STANDALONE ENTRY POINT
 * ============================================================= */

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
