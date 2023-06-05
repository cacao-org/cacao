
#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_arith/COREMOD_arith.h"

#include "fft/fft.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"
#include "image_basic/image_basic.h"


// Local variables pointers


// turbulence on/off toggle
static int64_t *turbON;
static long     fpi_turbON;

// turbulence on/off toggle
static int64_t *turbZERO;
static long     fpi_turbZERO;



// output stream name
static char *dmstream;
static long  fpi_dmstream;


// DM pixel scale
static float *DMpixscale;
static long  fpi_DMpixscale;


// turbulence 3D cube
static char *turbfname;
static long  fpi_turbfname;


// Wind speed [m/s]
static float *turbwspeed;
static long   fpi_turbwspeed;


// Wind angle [rad]
static float *turbwangle;
static long   fpi_turbwangle;

// Wind amplitude [um]
static float *turbampl;
static long   fpi_turbampl;



// number of time samples in turbulence cube
static uint32_t *NBsamples;
static long      fpi_NBsamples;



// Compute turb seed screens
static uint64_t *compTurbSeed;
static long      fpi_compTurbSeed;

static uint32_t *turbseedsize;
static long      fpi_turbseedsize;

// pixel scale [m/pix]
static float *turbseedpixscale;
static long   fpi_turbseedpixscale;

// inner scale [m]
static float *turbseedinnerscale;
static long   fpi_turbseedinnerscale;

// outer scale [m]
static float *turbseedouterscale;
static long   fpi_turbseedouterscale;






// Compute turb cube
static uint64_t *compTurbCube;
static long      fpi_compTurbCube;



static CLICMDARGDEF farg[] =
{
    {
        CLIARG_ONOFF,
        ".turbON",
        "turbulence on/off (off=freeze)",
        "ON",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbON,
        &fpi_turbON
    },
    {
        CLIARG_ONOFF,
        ".turbZERO",
        "turbulence zero",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbZERO,
        &fpi_turbZERO
    },
    {
        CLIARG_STREAM,
        ".dmstream",
        "output DM turbulence stream",
        "null",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmstream,
        &fpi_dmstream
    },
    {
        CLIARG_FLOAT32,
        ".DMpixscale",
        "DM pixel scale [m/pix]",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &DMpixscale,
        &fpi_DMpixscale
    },
    {
        CLIARG_FLOAT32,
        ".wspeed",
        "wind speed [m/s]",
        "10.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbwspeed,
        &fpi_turbwspeed
    },
    {
        CLIARG_FLOAT32,
        ".wangle",
        "wind angle [rad]",
        "1.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbwangle,
        &fpi_turbwangle
    },
    {
        CLIARG_FLOAT32,
        ".ampl",
        "amplitude across aperture [um]",
        "0.2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbampl,
        &fpi_turbampl
    },
    {
        CLIARG_ONOFF,
        ".turbseed.comp",
        "(re)compute turbulence seed screen (../conf/turbseedX.fits)",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compTurbSeed,
        &fpi_compTurbSeed
    },
    {
        CLIARG_UINT32,
        ".turbseed.size",
        "screen seed size",
        "1024",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbseedsize,
        &fpi_turbseedsize
    },
    {
        CLIARG_FLOAT32,
        ".turbseed.pixscale",
        "screen pixel scale [m/pix]",
        "0.1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbseedpixscale,
        &fpi_turbseedpixscale
    },
    {
        CLIARG_FLOAT32,
        ".turbseed.innerscale",
        "screen inner scale [m]",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbseedinnerscale,
        &fpi_turbseedinnerscale
    },
    {
        CLIARG_FLOAT32,
        ".turbseed.outerscale",
        "screen outer scale [m]",
        "20",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &turbseedouterscale,
        &fpi_turbseedouterscale
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

        data.fpsptr->parray[fpi_turbON].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbZERO].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_dmstream].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_turbwspeed].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbwangle].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_turbampl].fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}







//
// innerscale and outerscale in pixel
// von Karman spectrum
//
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
    arith_image_cstmult("tmppha", 2.0 * PI, "tmppha1");
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
    arith_set_pixel("tmpamp", 0.0, size / 2, size / 2);
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









// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.fpsptr != NULL)
    {
        if(data.fpsptr->parray[fpi_compTurbSeed].fpflag & FPFLAG_ONOFF)
        {
            printf("RECOMPUTING DM TURB SEED\n");

            make_seed_turbulence_screen(
                "tseed0",
                "tseed1",
                *turbseedsize,
                (*turbseedouterscale) / (*turbseedpixscale),
                (*turbseedinnerscale) / (*turbseedpixscale)
            );

            list_image_ID();

            save_fits("tseed0", "../conf/turbseed0.fits");
            save_fits("tseed1", "../conf/turbseed1.fits");

            data.fpsptr->parray[fpi_compTurbSeed].fpflag &= ~FPFLAG_ONOFF;
        }



        if(data.fpsptr->parray[fpi_compTurbCube].fpflag & FPFLAG_ONOFF)
        {
            printf("RECOMPUTING DM TURB CUBE\n");
            data.fpsptr->parray[fpi_compTurbCube].fpflag &= ~FPFLAG_ONOFF;
        }


    }

    return RETURN_SUCCESS;
}




static CLICMDDATA CLIcmddata =
{
    "dmturb", "DM turbulence", CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}








/**
 * @brief Wrapper function, used by all CLI calls
 *
 * INSERT_STD_PROCINFO statements enable processinfo support
 */
static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to DM stream to get size
    //
    IMGID imgDM = mkIMGID_from_name(dmstream);
    resolveIMGID(&imgDM, ERRMODE_ABORT);
    printf("%u x %u actuator\n", imgDM.md->size[0], imgDM.md->size[1]);
    uint32_t xsize = imgDM.md->size[0];
    uint32_t ysize = imgDM.md->size[1];


    // temporary DM array
    float *turbimarray = (float *) malloc(sizeof(float) * xsize * ysize);

    imageID IDts0;
    load_fits("../conf/turbseed0.fits", "tseed0", 1, &IDts0);
    uint32_t Sxsize = data.image[IDts0].md->size[0];
    uint32_t Sysize = data.image[IDts0].md->size[1];

    list_image_ID();

    // DM pixel scale
    float psDM = *DMpixscale;
    printf("psDM = %f\n", psDM);

    // physical time
    double phystime = 0.0;
    double phystimeprev = 0.0;

    // position on seed screen 0
    double x0m = 0.0;
    double y0m = 0.0;

    double amplcoeff = 1.0; // amplitude adjustment coeff


    struct timespec tstart;
    struct timespec tnow;
    clock_gettime(CLOCK_MILK, &tstart);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {


        // zero loop
        if(data.fpsptr->parray[fpi_turbZERO].fpflag & FPFLAG_ONOFF)
        {

            for(uint64_t ii = 0; ii < xsize *ysize; ii++)
            {
                turbimarray[ii] = 0.0;
            }

            imgDM.md->write = 1;
            memcpy(imgDM.im->array.F, turbimarray, sizeof(float)*xsize * ysize);
            processinfo_update_output_stream(processinfo, imgDM.ID);

            // toggle back to OFF
            data.fpsptr->parray[fpi_turbZERO].fpflag &= ~FPFLAG_ONOFF;
        }



        if((*turbON) == 1)
        {
            clock_gettime(CLOCK_MILK, &tnow);
            long tdiffsec = tnow.tv_sec - tstart.tv_sec;
            long tdiffnsec = tnow.tv_nsec - tstart.tv_nsec;
            double tdiff = 1.0 * tdiffsec + 1.0e-9 * tdiffnsec;

            phystimeprev = phystime;
            phystime = tdiff;
            double dt = phystime - phystimeprev;

            // position on seed screen
            x0m += dt * (*turbwspeed) * cos(*turbwangle);
            y0m += dt * (*turbwspeed) * sin(*turbwangle);

            double seedscreensizem = (*turbseedpixscale) * Sxsize;

            while(x0m < 0)
            {
                x0m += seedscreensizem;
            }
            while(x0m > seedscreensizem)
            {
                x0m -= seedscreensizem;
            }

            while(y0m < 0)
            {
                y0m += seedscreensizem;
            }
            while(y0m > seedscreensizem)
            {
                y0m -= seedscreensizem;
            }

            // x0m and y0m are positive


            double total = 0.0;
            for(uint32_t ii = 0; ii < xsize; ii++)
            {
                double xm = x0m + (*DMpixscale) * ii;
                double xpix = xm / (*turbseedpixscale);

                uint32_t xpix0 = (uint32_t) xpix;
                double xfrac = xpix - xpix0;

                xpix0 = xpix0 % (*turbseedsize);
                uint32_t xpix1 = xpix0 + 1;
                xpix1 = xpix1 % Sxsize;


                for(uint32_t jj = 0; jj < ysize; jj++)
                {
                    double ym = y0m + (*DMpixscale) * jj;
                    double ypix = ym / (*turbseedpixscale);

                    uint32_t ypix0 = (uint32_t) ypix;
                    double yfrac = ypix - ypix0;

                    ypix0 = ypix0 % (*turbseedsize);
                    uint32_t ypix1 = ypix0 + 1;
                    ypix1 = ypix1 % Sysize;

                    double v00 = data.image[IDts0].array.F[ypix0 * Sxsize + xpix0];
                    double v10 = data.image[IDts0].array.F[ypix0 * Sxsize + xpix1];
                    double v01 = data.image[IDts0].array.F[ypix1 * Sxsize + xpix0];
                    double v11 = data.image[IDts0].array.F[ypix1 * Sxsize + xpix1];

                    // bilinear interpolation
                    float val = v00 * (1.0 - xfrac) * (1.0 - yfrac) + v10 * xfrac *
                                (1.0 - yfrac) + v01 * (1.0 - xfrac) * yfrac + v11 * xfrac * yfrac;
                    val *= amplcoeff;
                    turbimarray[jj * xsize + ii] = val;
                    total += val;
                }
            }
            double total2 = 0.0;
            for(uint64_t ii = 0; ii < xsize *ysize; ii++)
            {
                turbimarray[ii] -= total / (xsize * ysize);
                total2 += turbimarray[ii] * turbimarray[ii];
            }
            double RMSval = sqrt(total2 / (xsize * ysize));

            processinfo_WriteMessage_fmt(processinfo, "pht %.3lf s (+ %.0f us) RMS %.3f", phystime, 1e6 * dt, RMSval);

            // tweak amplcoeff to match desired RMS
            // large discrepancy lead ot large correction
            //
            double coeffstep = (*turbampl) / RMSval;
            double logdiff = log10(coeffstep);
            double logdiff3abs = pow(fabs(logdiff), 3.0);
            double amplloopgain = 1.0e-4 + logdiff3abs / (logdiff3abs + 1.0);
            amplcoeff *= pow(10.0, amplloopgain * logdiff);

            printf("RMS= %6.3f / %6.3f  logdiff = %6.3f  factor = %6.3f\n", RMSval, (*turbampl), logdiff, pow(10.0, logdiff));





            imgDM.md->write = 1;
            memcpy(imgDM.im->array.F, turbimarray, sizeof(float)*xsize * ysize);
            processinfo_update_output_stream(processinfo, imgDM.ID);

        }
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(turbimarray);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions

errno_t
CLIADDCMD_AOloopControl_DM__atmturbulence()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    // Optional custom settings for this function can be included
    // CLIcmddata.cmdsettings->procinfo_loopcntMax = 9;

    return RETURN_SUCCESS;
}
