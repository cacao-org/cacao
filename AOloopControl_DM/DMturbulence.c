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

#include "CommandLineInterface/CLIcore.h"
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

/* =============================================================================================== */
/* PARAMETERS DEFINITION                                                                           */
/* =============================================================================================== */

#define DMTURB_PARAMS(X) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".turbON", "turbulence on/off (off=freeze)", "ON", turbON_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".turbZERO", "turbulence zero", "OFF", turbZERO_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".seedZERO", "set seed pos to zero", "OFF", seedZERO_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_VISIBLE_DEFAULT, FPTYPE_STREAMNAME, char*, ".dmstream", "output DM turbulence stream", "dm00disp09", dmstream_ptr, GetParamPtr_STRING, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".DMpixscale", "DM pixel scale [m/pix]", "0.2", DMpixscale_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".wspeed", "wind speed [m/s]", "10.0", turbwspeed_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".wangle", "wind angle [rad]", "1.2", turbwangle_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".ampl", "amplitude across aperture [um]", "0.2", turbampl_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_ONOFF, uint64_t*, ".turbseed.comp", "(re)compute turbulence seed screen", "OFF", compTurbSeed_ptr, GetParamPtr_fpflag, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_UINT32, uint32_t*, ".turbseed.size", "screen seed size", "1024", turbseedsize_ptr, GetParamPtr_UINT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".turbseed.pixscale", "screen pixel scale [m/pix]", "0.1", turbseedpixscale_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".turbseed.innerscale", "screen inner scale [m]", "0.01", turbseedinnerscale_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT) \
    X(CLIARG_HIDDEN_DEFAULT, FPTYPE_FLOAT32, float*, ".turbseed.outerscale", "screen outer scale [m]", "20", turbseedouterscale_ptr, GetParamPtr_FLOAT32, CLICMDARG_FLAG_DEFAULT, FPTYPE_AUTO, FPFLAG_DEFAULT_INPUT)

/* Global parameter pointers */
#define X_PTR_DECL(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...) \
    static c_type ptr_name = NULL;
DMTURB_PARAMS(X_PTR_DECL)
#undef X_PTR_DECL

/* FPI indices for customCONFcheck */
static uint64_t fpi_turbON;
static uint64_t fpi_turbZERO;
static uint64_t fpi_seedZERO;
static uint64_t fpi_dmstream;
static uint64_t fpi_DMpixscale;
static uint64_t fpi_turbwspeed;
static uint64_t fpi_turbwangle;
static uint64_t fpi_turbampl;
static uint64_t fpi_compTurbSeed;
static uint64_t fpi_turbseedsize;
static uint64_t fpi_turbseedpixscale;
static uint64_t fpi_turbseedinnerscale;
static uint64_t fpi_turbseedouterscale;

static uint64_t processinfo_change_cnt_local = 0;

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
        outerscale = data.variable[IDv].value.f;
        printf("Outer scale = %f pix\n", outerscale);
      }
    */

    IDv = variable_ID("RLIM");
    if(IDv != -1)
    {
        RLIMMODE = 1;
        rlim     = data.variable[IDv].value.f;
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
            dx = 1.0 * ii - size / 2;
            dy = 1.0 * jj - size / 2;

            if(RLIMMODE == 1)
            {
                r = sqrt(dx * dx + dy * dy);
                if(r < rlim)
                {
                    data.image[ID].array.F[jj * size + ii] = 0.0;
                }
                else
                {
                    data.image[ID].array.F[jj * size + ii] =
                        sqrt(dx * dx + dy * dy + OUTERscale_f0 * OUTERscale_f0);
                }
            }
            else
            {
                data.image[ID].array.F[jj * size + ii] =
                    sqrt(dx * dx + dy * dy + OUTERscale_f0 * OUTERscale_f0);
            }
        }
    //  data.image[ID].array.F[size/2*size+size/2+10] = 1.0;

    // period [pix] = size/sqrt(dx*dx+dy*dy)
    // f [1/pix] = sqrt(dx*dx+dy*dy)/size
    // f [1/pix] * size = sqrt(dx*dx+dy*dy)

    make_rnd("tmpg", size, size, "-gauss");
    ID = image_ID("tmpg");
    for(uint32_t ii = 0; ii < size; ii++)
        for(uint32_t jj = 0; jj < size; jj++)
        {
            dx      = 1.0 * ii - size / 2;
            dy      = 1.0 * jj - size / 2;
            iscoeff = exp(-(dx * dx + dy * dy) / INNERscale_f0 / INNERscale_f0);
            data.image[ID].array.F[jj * size + ii] *=
                sqrt(iscoeff); // power -> amplitude : sqrt
        }

    arith_image_cstpow("tmpd", 11.0 / 6.0, "tmpd1");
    delete_image_ID("tmpd", DELETE_IMAGE_ERRMODE_WARNING);
    arith_image_div("tmpg", "tmpd1", "tmpamp");
    delete_image_ID("tmpg", DELETE_IMAGE_ERRMODE_WARNING);
    delete_image_ID("tmpd1", DELETE_IMAGE_ERRMODE_WARNING);

    {
        IMGID imgtmpamp = mkIMGID_from_name("tmpamp");
        resolveIMGID(&imgtmpamp, ERRMODE_ABORT);
        image_set_2Dpix(imgtmpamp, 0.0, size / 2, size / 2);
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
    ID    = image_ID("strf");
    value = 0.0;
    cnt   = 0;
    for(uint32_t ii = 1; ii < Dlim; ii++)
        for(uint32_t jj = 1; jj < Dlim; jj++)
        {
            value += log10(data.image[ID].array.F[jj * size + ii]) -
                     5.0 / 3.0 * log10(sqrt(ii * ii + jj * jj));
            cnt++;
        }
    // save_fl_fits("strf","strf.fits");
    delete_image_ID("strf", DELETE_IMAGE_ERRMODE_WARNING);
    C1 = pow(10.0, value / cnt);

    fft_structure_function("tmpo2", "strf");
    ID    = image_ID("strf");
    value = 0.0;
    cnt   = 0;
    for(uint32_t ii = 1; ii < Dlim; ii++)
        for(uint32_t jj = 1; jj < Dlim; jj++)
        {
            value += log10(data.image[ID].array.F[jj * size + ii]) -
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
    state->imgDM = mkIMGID_from_name(dmstream_ptr);
    resolveIMGID(&state->imgDM, ERRMODE_ABORT);
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
        memcpy(state->imgDM.im->array.F, state->turbimarray, sizeof(float)*xsize*ysize);
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
        
        uint32_t Sxsize = data.image[state->IDts0].md->size[0];
        uint32_t Sysize = data.image[state->IDts0].md->size[1];
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
                
                double v00 = data.image[state->IDts0].array.F[ypix0 * Sxsize + xpix0];
                double v10 = data.image[state->IDts0].array.F[ypix0 * Sxsize + xpix1];
                double v01 = data.image[state->IDts0].array.F[ypix1 * Sxsize + xpix0];
                double v11 = data.image[state->IDts0].array.F[ypix1 * Sxsize + xpix1];
                
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
        
        memcpy(state->imgDM.im->array.F, state->turbimarray, sizeof(float)*xsize*ysize);
        processinfo_update_output_stream(processinfo, state->imgDM.im, NULL);
    }
}

static void dmturb_validate() {
    if (turbseedsize_ptr && *turbseedsize_ptr == 0) *turbseedsize_ptr = 1024;
}

#ifndef FPS_STANDALONE

static CLICMDARGDEF farg[] = {
    { CLIARG_ONOFF, ".turbON", "turbulence on/off (off=freeze)", "ON", CLIARG_HIDDEN_DEFAULT, (void **) &turbON_ptr, (long*)&fpi_turbON },
    { CLIARG_ONOFF, ".turbZERO", "turbulence zero", "OFF", CLIARG_HIDDEN_DEFAULT, (void **) &turbZERO_ptr, (long*)&fpi_turbZERO },
    { CLIARG_ONOFF, ".seedZERO", "set seed pos to zero", "OFF", CLIARG_HIDDEN_DEFAULT, (void **) &seedZERO_ptr, (long*)&fpi_seedZERO },
    { CLIARG_STREAM, ".dmstream", "output DM turbulence stream", "dm00disp09", CLIARG_VISIBLE_DEFAULT, (void **) &dmstream_ptr, (long*)&fpi_dmstream },
    { CLIARG_FLOAT32, ".DMpixscale", "DM pixel scale [m/pix]", "0.2", CLIARG_HIDDEN_DEFAULT, (void **) &DMpixscale_ptr, (long*)&fpi_DMpixscale },
    { CLIARG_FLOAT32, ".wspeed", "wind speed [m/s]", "10.0", CLIARG_HIDDEN_DEFAULT, (void **) &turbwspeed_ptr, (long*)&fpi_turbwspeed },
    { CLIARG_FLOAT32, ".wangle", "wind angle [rad]", "1.2", CLIARG_HIDDEN_DEFAULT, (void **) &turbwangle_ptr, (long*)&fpi_turbwangle },
    { CLIARG_FLOAT32, ".ampl", "amplitude across aperture [um]", "0.2", CLIARG_HIDDEN_DEFAULT, (void **) &turbampl_ptr, (long*)&fpi_turbampl },
    { CLIARG_ONOFF, ".turbseed.comp", "(re)compute turbulence seed screen", "OFF", CLIARG_HIDDEN_DEFAULT, (void **) &compTurbSeed_ptr, (long*)&fpi_compTurbSeed },
    { CLIARG_UINT32, ".turbseed.size", "screen seed size", "1024", CLIARG_HIDDEN_DEFAULT, (void **) &turbseedsize_ptr, (long*)&fpi_turbseedsize },
    { CLIARG_FLOAT32, ".turbseed.pixscale", "screen pixel scale [m/pix]", "0.1", CLIARG_HIDDEN_DEFAULT, (void **) &turbseedpixscale_ptr, (long*)&fpi_turbseedpixscale },
    { CLIARG_FLOAT32, ".turbseed.innerscale", "screen inner scale [m]", "0.01", CLIARG_HIDDEN_DEFAULT, (void **) &turbseedinnerscale_ptr, (long*)&fpi_turbseedinnerscale },
    { CLIARG_FLOAT32, ".turbseed.outerscale", "screen outer scale [m]", "20", CLIARG_HIDDEN_DEFAULT, (void **) &turbseedouterscale_ptr, (long*)&fpi_turbseedouterscale }
};

static CLICMDDATA CLIcmddata = { 
    "dmturb", "DM turbulence", "", 
    sizeof(farg) / sizeof(CLICMDARGDEF), farg, 
    CLICMDFLAG_FPS, NULL, NULL, NULL 
};

static errno_t customCONFsetup() {
    if(data.fpsptr != NULL) {
        data.fpsptr->parray[fpi_turbON].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbZERO].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_seedZERO].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_dmstream].fpflag |= FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_turbwspeed].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbwangle].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbampl].fpflag |= FPFLAG_WRITERUN;
    }
    return RETURN_SUCCESS;
}

static errno_t customCONFcheck() {
    if(data.fpsptr != NULL) check_recompute_seed();
    return RETURN_SUCCESS;
}

static errno_t help_function() {
    return RETURN_SUCCESS;
}

static errno_t compute_function() {
    DMTURB_STATE *state = dmturb_init();
    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    dmturb_step(processinfo, data.fpsptr, state);
    INSERT_STD_PROCINFO_COMPUTEFUNC_END
    dmturb_cleanup(state);
    return RETURN_SUCCESS;
}

#define INSERT_STD_FPSCONFfunction_local                                       \
    static errno_t FPSCONFfunction()                                           \
    {                                                                          \
        FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);                       \
        if (CLIcmddata.flags & CLICMDFLAG_PROCINFO)                            \
        {                                                                      \
            fps_add_processinfo_entries(&fps);                                 \
        }                                                                      \
        data.fpsptr = &fps;                                                    \
        CMDargs_to_FPSparams_create(&fps);                                     \
        if (CLIcmddata.FPS_customCONFsetup != NULL)                            \
        {                                                                      \
            CLIcmddata.FPS_customCONFsetup();                                  \
        }                                                                      \
        FPS_CONFLOOP_START                                                     \
        if (CLIcmddata.FPS_customCONFcheck != NULL)                            \
            CLIcmddata.FPS_customCONFcheck();                                  \
        FPS_CONFLOOP_END                                                       \
        data.fpsptr = NULL;                                                    \
        return RETURN_SUCCESS;                                                 \
    }

INSERT_STD_FPSCONFfunction_local
INSERT_STD_FPSRUNfunction
INSERT_STD_FPSCLIfunction

errno_t CLIADDCMD_AOloopControl_DM__atmturbulence() {
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}

#endif

#ifdef FPS_STANDALONE

int FPSINIT_AOloopControl_DM_atmturbulence(const char *fps_name, const char *keywords, const char *description) {
    FUNCTION_PARAMETER_STRUCT fps;
    FPS_INIT_STD_PREAMBLE(fps, fps_name, keywords, description, "DM Turbulence");
    FPS_INIT_PROCINFO_DEFAULTS(fps, "dm00disp09", 10);
    #define X_FPS_INIT(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...) \
    { \
        if(fps_type == FPTYPE_FLOAT32) { float val = (float)atof(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); } \
        else if(fps_type == FPTYPE_UINT32) { uint32_t val = (uint32_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); } \
        else if(fps_type == FPTYPE_UINT64) { uint64_t val = (uint64_t)atoll(def_str); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, &val, NULL); } \
        else if(fps_type == FPTYPE_STREAMNAME) { char val[FUNCTION_PARAMETER_STRMAXLEN]; strncpy(val, def_str, FUNCTION_PARAMETER_STRMAXLEN-1); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val, NULL); } \
        else if(fps_type == FPTYPE_STRING) { char val[FUNCTION_PARAMETER_STRMAXLEN]; strncpy(val, def_str, FUNCTION_PARAMETER_STRMAXLEN-1); function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, val, NULL); } \
        else { function_parameter_add_entry(&fps, key, descr, fps_type, FPFLAG_DEFAULT_INPUT, NULL, NULL); } \
    }
    DMTURB_PARAMS(X_FPS_INIT)
    #undef X_FPS_INIT
    
    fps_add_processinfo_entries(&fps); function_parameter_FPCONFexit(&fps); return 0;
}

#define X_FPS_MAP(cli_type, fps_type, c_type, key, descr, def_str, ptr_name, get_func, ...) \
            ptr_name = (c_type)functionparameter_##get_func(&fps, key);

int FPSCONF_AOloopControl_DM_atmturbulence(const char *fps_name, int loop) {
    FPS_CONF_STD_BODY(fps_name, loop, { DMTURB_PARAMS(X_FPS_MAP) }, { dmturb_validate(); check_recompute_seed(); });
    return 0;
}
FPS_MAKE_STANDALONE_CONFSTOP(AOloopControl_DM_atmturbulence)
FPS_MAKE_STANDALONE_RUNSTOP(AOloopControl_DM_atmturbulence)

int FPSRUN_AOloopControl_DM_atmturbulence(const char *fps_name) {
    FUNCTION_PARAMETER_STRUCT fps;
    FPS_RUN_STD_PREAMBLE(fps_name, fps, { DMTURB_PARAMS(X_FPS_MAP) });
    
    DMTURB_STATE *state = dmturb_init();
    PROCESSINFO *pinfo;
    FPS_RUN_PROCESSINFO_SETUP(pinfo, fps_name, "Run", "Looping", state->imgDM.im, fps);
    
    while(processinfo_loopstep(pinfo)) {
        processinfo_exec_start(pinfo);
        dmturb_step(pinfo, &fps, state);
        processinfo_exec_end(pinfo);
        usleep(100); 
    }
    
    dmturb_cleanup(state);
    processinfo_cleanExit(pinfo); function_parameter_struct_disconnect(&fps); return 0;
}

FPS_MAIN_STANDALONE("dmturb", AOloopControl_DM_atmturbulence, "DM Turbulence", DMTURB_PARAMS)
#endif
