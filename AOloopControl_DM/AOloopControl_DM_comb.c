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

#include "CLIcore/CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "libmilkdata/milk_compiler.h"
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
    double *valarray;

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
        read_sharedmem_image(astrogridsname_ptr,
            data.core.image,
            data.core.NB_MAX_IMAGE);
        state->ag_imgdispbuffer = imgid_make_from_name(astrogridsname_ptr);
        resolveIMGID(&state->ag_imgdispbuffer,
            ERRMODE_ABORT,
            data.core.image,
            data.core.NB_MAX_IMAGE);
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
                float * MILK_RESTRICT outptr = MILK_ASSUME_ALIGNED(state->imgch[chan].im->array.F);
                const float * MILK_RESTRICT inptr = MILK_ASSUME_ALIGNED(state->ag_imgdispbuffer.im->array.F);
                uint64_t offset = state->ag_sliceindex * state->ag_xysize;
                float mult = *astrogridmult_ptr;
                
                #pragma omp simd
                for(uint64_t ii = 0; ii < state->ag_xysize; ii++)
                {
                    outptr[ii] = mult * inptr[offset + ii];
                }
            }
        }
    }
    return RETURN_SUCCESS;
}

/**
 * @brief Convert DM displacement to voltage
 *
 * Uses pre-allocated valarray from state to
 * avoid per-call malloc. Specializes pow()
 * for common exponents (0.5→sqrtf, 1.0→nop)
 * to eliminate expensive transcendentals
 * from the hot loop.
 */
static errno_t DM_displ2V(
    IMGID imgdisp,
    IMGID imgvolt,
    DMCOMB_STATE *state)
{
    uint64_t xysize =
        (uint64_t)(*DMxsize_ptr)
        * (*DMysize_ptr);
    double *valarray = state->valarray;

    if((*volttype_ptr) == 1)
    {
        float inrange =
            (*maxvolt_ptr)
            * (*stroke100_ptr) / 100.0f;
        strcpy(outv_ftype_ptr, "float32");
        *outv_exp_ptr = 1.0f;
        *outv_inrange_min_ptr = -inrange;
        *outv_inrange_max_ptr = inrange;
        *outv_outrange_min_ptr =
            -(*maxvolt_ptr);
        *outv_outrange_max_ptr =
            -(*maxvolt_ptr);
    }
    else if((*volttype_ptr) == 2)
    {
        float inrange =
            (*maxvolt_ptr)
            * (*stroke100_ptr) / 100.0f;
        strcpy(outv_ftype_ptr, "uint16");
        *outv_exp_ptr = 0.5f;
        *outv_inrange_min_ptr = -inrange;
        *outv_inrange_max_ptr = inrange;
        *outv_outrange_min_ptr = 0.0f;
        *outv_outrange_max_ptr =
            (*maxvolt_ptr)
            / 300.0f * 16384.0f;
    }

    {
        float exp_val = *outv_exp_ptr;
        float inmin = *outv_inrange_min_ptr;
        float inmax = *outv_inrange_max_ptr;
        float outmin = *outv_outrange_min_ptr;
        float outmax = *outv_outrange_max_ptr;
        float range = inmax - inmin;
        float inv_range =
            (range != 0.0f)
            ? (1.0f / range) : 0.0f;
        float outscale = outmax - outmin;

        /* Specialize pow() for common
         * exponents to avoid expensive
         * transcendental in inner loop */
        if(fabsf(exp_val - 1.0f) < 1.0e-6f)
        {
            /* exp == 1.0: linear, no pow */
            for(uint64_t ii = 0;
                ii < xysize; ii++)
            {
                float x =
                    (imgdisp.im->array.F[ii]
                     - inmin) * inv_range;
                if(x < 0.0f) x = 0.0f;
                if(x > 1.0f) x = 1.0f;
                valarray[ii] =
                    outmin + x * outscale;
            }
        }
        else if(fabsf(exp_val - 0.5f)
                < 1.0e-6f)
        {
            /* exp == 0.5: use sqrtf */
            for(uint64_t ii = 0;
                ii < xysize; ii++)
            {
                float x =
                    (imgdisp.im->array.F[ii]
                     - inmin) * inv_range;
                if(x < 0.0f) x = 0.0f;
                if(x > 1.0f) x = 1.0f;
                valarray[ii] =
                    outmin
                    + sqrtf(x) * outscale;
            }
        }
        else
        {
            /* General exponent: use powf */
            for(uint64_t ii = 0;
                ii < xysize; ii++)
            {
                float x =
                    (imgdisp.im->array.F[ii]
                     - inmin) * inv_range;
                if(x < 0.0f) x = 0.0f;
                if(x > 1.0f) x = 1.0f;
                valarray[ii] =
                    outmin
                    + powf(x, exp_val)
                      * outscale;
            }
        }
    }

    if((*volttype_ptr) == 1)
    {
        float scale =
            100.0f / (*stroke100_ptr);
        float maxv = *maxvolt_ptr;
        for(uint64_t ii = 0;
            ii < xysize; ii++)
        {
            float v =
                scale
                * imgdisp.im->array.F[ii];
            if(v > maxv) v = maxv;
            if(v < -maxv) v = -maxv;
            imgvolt.im->array.F[ii] = v;
        }
    }
    else if((*volttype_ptr) == 2)
    {
        float inv_s = 1.0f / (*stroke100_ptr);
        float maxv = *maxvolt_ptr;
        float vscale =
            16384.0f / 300.0f;
        for(uint64_t ii = 0;
            ii < xysize; ii++)
        {
            float val =
                imgdisp.im->array.F[ii];
            if(val < 0.0f) val = 0.0f;
            float volt =
                100.0f
                * sqrtf(val * inv_s);
            if(volt > maxv) volt = maxv;
            imgvolt.im->array.UI16[ii] =
                (unsigned short int)
                (volt * vscale);
        }
    }
    else if((*volttype_ptr) == 3)
    {
        float inv_s = 1.0f / (*stroke100_ptr);
        float maxv = *maxvolt_ptr;
        for(uint64_t ii = 0;
            ii < xysize; ii++)
        {
            float volt =
                imgdisp.im->array.F[ii]
                * inv_s + 0.5f;
            if(volt > maxv)
                volt = remainderf(
                    volt, 1.0f);
            if(volt < 0.0f)
                volt = remainderf(
                    volt, 1.0f);
            imgvolt.im->array.UI16[ii] =
                (unsigned short int)
                (volt * 65535.0f);
        }
    }
    else if((*volttype_ptr) == 0)
    {
        if(strcmp(
               outv_ftype_ptr,
               "float64") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.D[ii] =
                    valarray[ii];
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "uint16") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.UI16[ii] =
                    (uint16_t)(valarray[ii]);
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "uint32") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.UI32[ii] =
                    (uint32_t)(valarray[ii]);
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "uint64") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.UI64[ii] =
                    (uint64_t)(valarray[ii]);
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "int16") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.SI16[ii] =
                    (int16_t)(valarray[ii]);
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "int32") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.SI32[ii] =
                    (int32_t)(valarray[ii]);
        }
        else if(strcmp(
                    outv_ftype_ptr,
                    "int64") == 0)
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.SI64[ii] =
                    (int64_t)(valarray[ii]);
        }
        else
        {
            for(uint64_t ii = 0;
                ii < xysize; ii++)
                imgvolt.im->array.F[ii] =
                    valarray[ii];
        }
    }

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
        const float * MILK_RESTRICT inptr = MILK_ASSUME_ALIGNED(imgch[ch].im->array.F);
        #pragma omp simd
        for(uint_fast64_t ii = 0; ii < size; ii++)
        {
            dmdisptmp[ii] += inptr[ii];
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
            const float * MILK_RESTRICT inptr = MILK_ASSUME_ALIGNED(imgch[ch].im->array.F);
            #pragma omp simd
            for(uint_fast64_t ii = 0; ii < size; ii++)
            {
                dmdisptmp[ii] += inptr[ii];
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
    if(state->valarray) free(state->valarray);

    free(state);
}


static DMCOMB_STATE* dmcomb_init()
{
    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    DMCOMB_STATE *state = (DMCOMB_STATE*) calloc(1, sizeof(DMCOMB_STATE));

    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    state->imgch = calloc(*NBchannel_ptr, sizeof(IMGID));
    for(uint32_t ch = 0; ch < *NBchannel_ptr; ch++) {
        char name[STRINGMAXLEN_STREAMNAME];
        snprintf(name, sizeof(name), "dm%02udisp%02u", *DMindex_ptr, ch);

        if (UNLIKELY(data.core.Debug > 0)) {
            printf("DEBUG: channel %d : %s\n", ch, name);
            fflush(stdout);
        }

        imageID IDch = read_sharedmem_image(name,
            data.core.image,
            data.core.NB_MAX_IMAGE);
        if (UNLIKELY(data.core.Debug > 0)) {
            printf("DEBUG: ID = %ld\n", IDch);
            fflush(stdout);
        }

        state->imgch[ch] = stream_connect_create_2Df32(name,
            *DMxsize_ptr,
            *DMysize_ptr);
    }

    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    state->imgdisp = stream_connect_create_2Df32(DMcombout_ptr,
        *DMxsize_ptr,
        *DMysize_ptr);
    state->imgdispzpo = stream_connect_create_2Df32(DMcomboutzpo_ptr,
        *DMxsize_ptr,
        *DMysize_ptr);

    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    state->dmdisptmp = malloc(
        sizeof(float)
        * (*DMxsize_ptr) * (*DMysize_ptr));
    state->valarray = malloc(
        sizeof(double)
        * (*DMxsize_ptr) * (*DMysize_ptr));

    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    if((*voltmode_ptr) & FPFLAG_ONOFF) {
        if(
            image_ID(voltname_ptr, data.core.image, data.core.NB_MAX_IMAGE) == -1) read_sharedmem_image(voltname_ptr,
            data.core.image,
            data.core.NB_MAX_IMAGE);
        state->imgdmvolt = imgid_make_from_name(voltname_ptr);
        resolveIMGID(&state->imgdmvolt,
            ERRMODE_ABORT,
            data.core.image,
            data.core.NB_MAX_IMAGE);
    }

    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    return state;
}


static void dmcomb_step(
    PROCESSINFO *processinfo,
    FUNCTION_PARAMETER_STRUCT *fps,
    DMCOMB_STATE *state
)
{
    if (UNLIKELY(data.core.Debug > 0)) {
        printf("DEBUG  %s [%d] %s\n", __FILE__, __LINE__, __FUNCTION__);
        fflush(stdout);
    }

    // Sync parameters
    if (fps) {
        if(UNLIKELY(fps->md->processinfo_change_cnt != processinfo_change_cnt_local)) {
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

    if(LIKELY(DMupdate)) {
        if(!((*astrogrid_ptr) & FPFLAG_ONOFF)) state->DMdisp_add_disp_from_circular_buffer_init = 0;

        if(((*astrogrid_ptr) & FPFLAG_ONOFF) && (*astrogridtdelay_ptr == 0)) {
            DMdisp_add_disp_from_circular_buffer(state);
            processinfo_update_output_stream(processinfo,
                state->imgch[*astrogridchan_ptr].im,
                NULL);
        }

        update_dmdisp(state->imgdisp, state->imgch, state->dmdisptmp);
        processinfo_update_output_stream(processinfo, state->imgdisp.im, NULL);

        if((*voltmode_ptr) & FPFLAG_ONOFF) {
            state->imgdmvolt.md->write = 1;
            DM_displ2V(state->imgdisp, state->imgdmvolt, state);
            processinfo_update_output_stream(processinfo,
                state->imgdmvolt.im,
                NULL);
        }

        if(((*astrogrid_ptr) & FPFLAG_ONOFF) && (*astrogridtdelay_ptr != 0)) {
            long nsec = (long)(1000 * (*astrogridtdelay_ptr));
            struct timespec timesleep;
            timesleep.tv_sec = nsec / 1000000000;
            timesleep.tv_nsec = nsec % 1000000000;
            nanosleep(&timesleep, NULL);

            DMdisp_add_disp_from_circular_buffer(state);
            processinfo_update_output_stream(processinfo,
                state->imgch[*astrogridchan_ptr].im,
                NULL);

            update_dmdisp(state->imgdisp, state->imgch, state->dmdisptmp);
            processinfo_update_output_stream(processinfo,
                state->imgdisp.im,
                NULL);

            if((*voltmode_ptr) & FPFLAG_ONOFF) {
                state->imgdmvolt.md->write = 1;
                DM_displ2V(state->imgdisp, state->imgdmvolt, state);
                processinfo_update_output_stream(processinfo,
                    state->imgdmvolt.im,
                    NULL);
            }
        }
    }

    if(UNLIKELY(DMupdatezpo)) {
        update_dmdispzpo(state->imgdispzpo,
            state->imgch,
            state->dmdisptmp,
            state->zpoffset_channel);
        processinfo_update_output_stream(processinfo,
            state->imgdispzpo.im,
            NULL);
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

FPS_V2_SECTION5(FPS_PARAMS)


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static MILK_HOT errno_t compute_function()
{
    DMCOMB_STATE *state = dmcomb_init();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    dmcomb_step(processinfo, data.core.fpsptr,
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
