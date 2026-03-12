/**
 * @file    DMturbulence.c
 * @brief   DM turbulence simulation
 * 
 * Refactored to FPS practices.
 */

#include <math.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include "CLIcore/CLIcore.h"
#include "ImageStreamIO/ImageStruct.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "timeutils.h"

#include "fft/fft.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"
#include "image_basic/image_basic.h"

#include "fps.h"
#include "processinfo.h"
#include "processtools.h"

/* ================================================================
 * 1.  FPS COMPONENT IDENTITY
 * ============================================================= */

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "dmturb",
    .cmdkey      = "atmturbulence",
    .description = "DM turbulence simulation"
};


/* ================================================================
 * 2.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static uint64_t *turbON_ptr              = NULL;
static uint64_t *turbZERO_ptr            = NULL;
static uint64_t *seedZERO_ptr            = NULL;
static char     *dmstream_ptr            = NULL;
static float    *DMpixscale_ptr          = NULL;
static float    *turbwspeed_ptr          = NULL;
static float    *turbwangle_ptr          = NULL;
static float    *turbampl_ptr            = NULL;
static uint64_t *compTurbSeed_ptr        = NULL;
static uint32_t *turbseedsize_ptr        = NULL;
static float    *turbseedpixscale_ptr    = NULL;
static float    *turbseedinnerscale_ptr  = NULL;
static float    *turbseedouterscale_ptr  = NULL;

static uint64_t processinfo_change_cnt_local = 0;


/* ================================================================
 * 3.  UNIFIED PARAMETER TABLE (X-Macro)
 * ============================================================= */

#define FPS_PARAMS(X) \
    X(".turbON", &turbON_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, \
      "turbulence on/off (off=freeze)") \
    X(".turbZERO", &turbZERO_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN \
          | FPFLAG_VISIBLE, \
      "turbulence zero") \
    X(".seedZERO", &seedZERO_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN \
          | FPFLAG_VISIBLE, \
      "set seed pos to zero") \
    X(".dmstream", &dmstream_ptr, \
      FPTYPE_STREAMNAME, 1, \
      FPFLAG_DEFAULT_INPUT \
          | FPFLAG_STREAM_RUN_REQUIRED \
          | FPFLAG_CHECKSTREAM \
          | FPFLAG_PRIMARY_CLI_INPUT, \
      "output DM turbulence stream") \
    X(".DMpixscale", &DMpixscale_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_VISIBLE, \
      "DM pixel scale [m/pix]") \
    X(".wspeed", &turbwspeed_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN \
          | FPFLAG_VISIBLE, \
      "wind speed [m/s]") \
    X(".wangle", &turbwangle_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN \
          | FPFLAG_VISIBLE, \
      "wind angle [rad]") \
    X(".ampl", &turbampl_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN \
          | FPFLAG_VISIBLE, \
      "amplitude across aperture [um]") \
    X(".turbseed.comp", &compTurbSeed_ptr, \
      FPTYPE_ONOFF, 0, \
      FPFLAG_DEFAULT_INPUT, \
      "(re)compute turbulence seed") \
    X(".turbseed.size", &turbseedsize_ptr, \
      FPTYPE_UINT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_VISIBLE, \
      "screen seed size") \
    X(".turbseed.pixscale", \
      &turbseedpixscale_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_VISIBLE, \
      "screen pixel scale [m/pix]") \
    X(".turbseed.innerscale", \
      &turbseedinnerscale_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_VISIBLE, \
      "screen inner scale [m]") \
    X(".turbseed.outerscale", \
      &turbseedouterscale_ptr, \
      FPTYPE_FLOAT32, 0, \
      FPFLAG_DEFAULT_INPUT | FPFLAG_VISIBLE, \
      "screen outer scale [m]")

typedef struct {
    IMGID imgDM;
    float *turbimarray;
    imageID IDts0;
    
    double phystime;
    double phystimeprev;
    double x0m;
    double y0m;
    double amplcoeff;
    
    struct timespec tstart;

} DMTURB_STATE;

/* =============================================================================================== */
/* HELPERS                                                                                         */
/* =============================================================================================== */

static errno_t make_seed_turbulence_screen(
    const char *ID_name1,
    const char *ID_name2,
    long        size,
    float       outerscale,
    float       innerscale
)
{
    imageID ID;
    float   value, C1, C2;
    long    cnt;
    long    Dlim = 3;
    imageID IDv;

    // int OUTERSCALE_MODE = 1; // 1 if outer scale
    double OUTERscale_f0;
    double INNERscale_f0;
    double dx, dy, r;
    double rlim     = 0.0;
    int    RLIMMODE = 0;
    double iscoeff;

    /*  IDv = variable_ID("OUTERSCALE");
    if(IDv!=-1)
      {
        outerscale = data.core.variable[IDv].value.f;
        printf("Outer scale = %f pix\n", outerscale);
      }
    */

    IDv = variable_ID("RLIM");
    if(IDv != -1)
    {
        RLIMMODE = 1;
        rlim     = data.core.variable[IDv].value.f;
        printf("R limit = %f pix\n", rlim);
    }

    OUTERscale_f0 = 1.0 * size / outerscale; // [1/pix] in F plane
    INNERscale_f0 = (5.92 / (2.0 * M_PI)) * size / innerscale;

    make_rnd("tmppha", size, size, "");
    arith_image_cstmult("tmppha", 2.0 * M_PI, "tmppha1");
    delete_image_ID("tmppha", DELETE_IMAGE_ERRMODE_WARNING);
    //  make_dist("tmpd",size,size,size/2,size/2);
    create_2Dimage_ID("tmpd", size, size, &ID);


    for(uint32_t ii = 0; ii < size; ii++)
        for(uint32_t jj = 0; jj < size; jj++)
        {
            dx = (double) ii - size / 2;
            dy = (double) jj - size / 2;

            if(RLIMMODE == 1)
            {
                r = sqrt(dx * dx + dy * dy);
                if(r < rlim)
                {
                    data.core.image[ID].array.F[jj * size + ii] = 0.0;
                }
                else
                {
                    data.core.image[ID].array.F[jj * size + ii] =
                        sqrt(dx * dx + dy * dy + OUTERscale_f0 * OUTERscale_f0);
                }
            }
            else
            {
                data.core.image[ID].array.F[jj * size + ii] =
                    sqrt(dx * dx + dy * dy + OUTERscale_f0 * OUTERscale_f0);
            }
        }
    //  data.core.image[ID].array.F[size/2*size+size/2+10] = 1.0;

    // period [pix] = size/sqrt(dx*dx+dy*dy)
    // f [1/pix] = sqrt(dx*dx+dy*dy)/size
    // f [1/pix] * size = sqrt(dx*dx+dy*dy)

    make_rnd("tmpg", size, size, "-gauss");
    ID = image_ID("tmpg", data.core.image, data.core.NB_MAX_IMAGE);
    for(uint32_t ii = 0; ii < size; ii++)
        for(uint32_t jj = 0; jj < size; jj++)
        {
            dx      = (double) ii - size / 2;
            dy      = (double) jj - size / 2;
            iscoeff = exp(-(dx * dx + dy * dy) / INNERscale_f0 / INNERscale_f0);
            data.core.image[ID].array.F[jj * size + ii] *=
                sqrt(iscoeff); // power -> amplitude : sqrt
        }

    arith_image_cstpow("tmpd", 11.0 / 6.0, "tmpd1");
    delete_image_ID("tmpd", DELETE_IMAGE_ERRMODE_WARNING);
    arith_image_div("tmpg", "tmpd1", "tmpamp");
    delete_image_ID("tmpg", DELETE_IMAGE_ERRMODE_WARNING);
    delete_image_ID("tmpd1", DELETE_IMAGE_ERRMODE_WARNING);

    {
        IMGID imgtmpamp = imgid_make_from_name("tmpamp");
        resolveIMGID(
            &imgtmpamp, ERRMODE_ABORT,
            data.core.image,
            data.core.NB_MAX_IMAGE);
        uint32_t cx = (uint32_t)(size / 2);
        uint32_t cy = (uint32_t)(size / 2);
        uint32_t w = imgtmpamp.md->size[0];
        imgtmpamp.im->array.F[cy * w + cx] = 0.0f;
    }

    mk_complex_from_amph("tmpamp", "tmppha1", "tmpc", 0);
    delete_image_ID("tmpamp", DELETE_IMAGE_ERRMODE_WARNING);
    delete_image_ID("tmppha1", DELETE_IMAGE_ERRMODE_WARNING);
    permut("tmpc");
    do2dfft("tmpc", "tmpcf");
    delete_image_ID("tmpc", DELETE_IMAGE_ERRMODE_WARNING);
    mk_reim_from_complex("tmpcf", "tmpo1", "tmpo2", 0);
    delete_image_ID("tmpcf", DELETE_IMAGE_ERRMODE_WARNING);

    /* compute the scaling factor in the power law of the structure function */
    fft_structure_function("tmpo1", "strf");
    ID    = image_ID("strf", data.core.image, data.core.NB_MAX_IMAGE);
    value = 0.0;
    cnt   = 0;
    for(uint32_t ii = 1; ii < Dlim; ii++)
        for(uint32_t jj = 1; jj < Dlim; jj++)
        {
            value += log10(data.core.image[ID].array.F[jj * size + ii]) -
                     5.0 / 3.0 * log10(sqrt(ii * ii + jj * jj));
            cnt++;
        }
    // save_fl_fits("strf","strf.fits");
    delete_image_ID("strf", DELETE_IMAGE_ERRMODE_WARNING);
    C1 = pow(10.0, value / cnt);

    fft_structure_function("tmpo2", "strf");
    ID    = image_ID("strf", data.core.image, data.core.NB_MAX_IMAGE);
    value = 0.0;
    cnt   = 0;
    for(uint32_t ii = 1; ii < Dlim; ii++)
        for(uint32_t jj = 1; jj < Dlim; jj++)
        {
            value += log10(data.core.image[ID].array.F[jj * size + ii]) -
                     5.0 / 3.0 * log10(sqrt(ii * ii + jj * jj));
            cnt++;
        }
    delete_image_ID("strf", DELETE_IMAGE_ERRMODE_WARNING);
    C2 = pow(10.0, value / cnt);

    printf("%f %f\n", C1, C2);

    arith_image_cstmult("tmpo1", 1.0 / sqrt(C1), ID_name1);
    arith_image_cstmult("tmpo2", 1.0 / sqrt(C2), ID_name2);
    delete_image_ID("tmpo1", DELETE_IMAGE_ERRMODE_WARNING);
    delete_image_ID("tmpo2", DELETE_IMAGE_ERRMODE_WARNING);

    return RETURN_SUCCESS;
}

static errno_t check_recompute_seed() {
    if(compTurbSeed_ptr && (*compTurbSeed_ptr) & FPFLAG_ONOFF) {
        printf("RECOMPUTING DM TURB SEED\n");
        make_seed_turbulence_screen(
            "tseed0",
            "tseed1",
            *turbseedsize_ptr,
            (*turbseedouterscale_ptr) / (*turbseedpixscale_ptr),
            (*turbseedinnerscale_ptr) / (*turbseedpixscale_ptr)
        );
        save_fits("tseed0", "../conf/turbseed0.fits");
        save_fits("tseed1", "../conf/turbseed1.fits");
        *compTurbSeed_ptr &= ~FPFLAG_ONOFF;
    }
    return RETURN_SUCCESS;
}

/* =============================================================================================== */
/* RUN LOGIC                                                                                       */
/* =============================================================================================== */

static void dmturb_cleanup(DMTURB_STATE *state) {
    if(!state) return;
    if(state->turbimarray) free(state->turbimarray);
    free(state);
}

static DMTURB_STATE* dmturb_init() {
    DMTURB_STATE *state = (DMTURB_STATE*) calloc(1, sizeof(DMTURB_STATE));
    
    // Connect to DM stream
    state->imgDM = imgid_make_from_name(dmstream_ptr);
    resolveIMGID(&state->imgDM,
        ERRMODE_ABORT,
        data.core.image,
        data.core.NB_MAX_IMAGE);
    printf("%u x %u actuator\n", state->imgDM.md->size[0], state->imgDM.md->size[1]);
    
    uint32_t xsize = state->imgDM.md->size[0];
    uint32_t ysize = state->imgDM.md->size[1];
    state->turbimarray = (float*) malloc(sizeof(float) * xsize * ysize);
    
    // Load seed 0
    if (load_fits("../conf/turbseed0.fits", "tseed0", 1, &state->IDts0) != 0) {
        // If fail, create it?
        // Original code seemed to expect it or create it in customCONFcheck
        // Let's force creation if not exists
        if (compTurbSeed_ptr) *compTurbSeed_ptr |= FPFLAG_ONOFF;
        check_recompute_seed();
        load_fits("../conf/turbseed0.fits", "tseed0", 1, &state->IDts0);
    }
    
    state->amplcoeff = 1.0;
    state->phystime = 0.0;
    state->phystimeprev = 0.0;
    state->x0m = 0.0;
    state->y0m = 0.0;
    
    clock_gettime(CLOCK_MILK, &state->tstart);
    
    return state;
}

static void dmturb_step(PROCESSINFO *processinfo, FUNCTION_PARAMETER_STRUCT *fps, DMTURB_STATE *state) {
    // Sync parameters
    if (fps) {
        if(fps->md->processinfo_change_cnt != processinfo_change_cnt_local) {
            fps_to_processinfo(fps, processinfo);
            processinfo_change_cnt_local = fps->md->processinfo_change_cnt;
        }
    }
    
    // Check seed recompute
    check_recompute_seed();
    
    uint32_t xsize = state->imgDM.md->size[0];
    uint32_t ysize = state->imgDM.md->size[1];
    
    if(turbZERO_ptr && ((*turbZERO_ptr) & FPFLAG_ONOFF)) {
        for(uint64_t ii=0; ii<xsize*ysize; ii++) state->turbimarray[ii] = 0.0;
        memcpy(state->imgDM.im->array.F,
            state->turbimarray,
            sizeof(float)*xsize*ysize);
        processinfo_update_output_stream(processinfo, state->imgDM.im, NULL);
        *turbZERO_ptr &= ~FPFLAG_ONOFF;
    }
    
    if(seedZERO_ptr && ((*seedZERO_ptr) & FPFLAG_ONOFF)) {
        state->x0m = 0.0;
        state->y0m = 0.0;
        *seedZERO_ptr &= ~FPFLAG_ONOFF;
    }
    
    if((*turbON_ptr) & FPFLAG_ONOFF) {
        struct timespec tnow;
        clock_gettime(CLOCK_MILK, &tnow);
        long tdiffsec = tnow.tv_sec - state->tstart.tv_sec;
        long tdiffnsec = tnow.tv_nsec - state->tstart.tv_nsec;
        double tdiff = 1.0 * tdiffsec + 1.0e-9 * tdiffnsec;
        
        state->phystimeprev = state->phystime;
        state->phystime = tdiff;
        double dt = state->phystime - state->phystimeprev;
        
        state->x0m += dt * (*turbwspeed_ptr) * cos(*turbwangle_ptr);
        state->y0m += dt * (*turbwspeed_ptr) * sin(*turbwangle_ptr);
        
        uint32_t Sxsize = data.core.image[state->IDts0].md->size[0];
        uint32_t Sysize = data.core.image[state->IDts0].md->size[1];
        double seedscreensizem = (*turbseedpixscale_ptr) * Sxsize;
        
        while(state->x0m < 0) state->x0m += seedscreensizem;
        while(state->x0m > seedscreensizem) state->x0m -= seedscreensizem;
        while(state->y0m < 0) state->y0m += seedscreensizem;
        while(state->y0m > seedscreensizem) state->y0m -= seedscreensizem;
        
        double total = 0.0;
        for(uint32_t ii=0; ii<xsize; ii++) {
            double xm = state->x0m + (*DMpixscale_ptr) * ii;
            double xpix = xm / (*turbseedpixscale_ptr);
            uint32_t xpix0 = (uint32_t) xpix;
            double xfrac = xpix - xpix0;
            xpix0 = xpix0 % (*turbseedsize_ptr);
            uint32_t xpix1 = (xpix0 + 1) % Sxsize;
            
            for(uint32_t jj=0; jj<ysize; jj++) {
                double ym = state->y0m + (*DMpixscale_ptr) * jj;
                double ypix = ym / (*turbseedpixscale_ptr);
                uint32_t ypix0 = (uint32_t) ypix;
                double yfrac = ypix - ypix0;
                ypix0 = ypix0 % (*turbseedsize_ptr);
                uint32_t ypix1 = (ypix0 + 1) % Sysize;
                
                double v00 = data.core.image[state->IDts0].array.F[ypix0 * Sxsize + xpix0];
                double v10 = data.core.image[state->IDts0].array.F[ypix0 * Sxsize + xpix1];
                double v01 = data.core.image[state->IDts0].array.F[ypix1 * Sxsize + xpix0];
                double v11 = data.core.image[state->IDts0].array.F[ypix1 * Sxsize + xpix1];
                
                float val = v00 * (1.0-xfrac)*(1.0-yfrac) + v10*xfrac*(1.0-yfrac) + v01*(1.0-xfrac)*yfrac + v11*xfrac*yfrac;
                val *= state->amplcoeff;
                state->turbimarray[jj*xsize + ii] = val;
                total += val;
            }
        }
        
        double total2 = 0.0;
        for(uint64_t ii=0; ii<xsize*ysize; ii++) {
            state->turbimarray[ii] -= total / (xsize * ysize);
            total2 += state->turbimarray[ii] * state->turbimarray[ii];
        }
        double RMSval = sqrt(total2 / (xsize * ysize));
        
        if (RMSval > 0) {
            double coeffstep = (*turbampl_ptr) / RMSval;
            double logdiff = log10(coeffstep);
            double logdiff3abs = pow(fabs(logdiff), 3.0);
            double amplloopgain = 1.0e-4 + logdiff3abs / (logdiff3abs + 1.0);
            state->amplcoeff *= pow(10.0, amplloopgain * logdiff);
        }
        
        memcpy(state->imgDM.im->array.F,
            state->turbimarray,
            sizeof(float)*xsize*ysize);
        processinfo_update_output_stream(processinfo, state->imgDM.im, NULL);
    }
}

static void dmturb_validate() {
    if (turbseedsize_ptr && *turbseedsize_ptr == 0) *turbseedsize_ptr = 1024;
}


/* ================================================================
 * 5.  BINDINGS, FARG, AND CLI DATA
 * ============================================================= */

FPS_V2_SECTION5(FPS_PARAMS)


/* ================================================================
 * 6.  COMPUTE WRAPPER
 * ============================================================= */

static MILK_HOT errno_t compute_function()
{
    DMTURB_STATE *state = dmturb_init();
    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    dmturb_step(processinfo, data.core.fpsptr,
                state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END
    dmturb_cleanup(state);
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
CLIADDCMD_AOloopControl_DM__atmturbulence()
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
