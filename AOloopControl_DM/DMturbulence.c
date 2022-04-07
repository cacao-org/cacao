
#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_arith/COREMOD_arith.h"

#include "fft/fft.h"
#include "image_filter/image_filter.h"
#include "image_gen/image_gen.h"
#include "image_basic/image_basic.h"


// Local variables pointers

// output stream name
static char *dmstream;
static long  fpi_dmstream;

// turbulence 3D cube
static char *turbfname;
static long  fpi_turbfname;


// Wind speed m/s
static float *turbwspeed;
static long   fpi_turbwspeed;


// number of time samples in turbulence cube
static uint32_t *NBsamples;
static long      fpi_NBsamples;




static CLICMDARGDEF farg[] = {{CLIARG_STREAM,
                               ".dmstream",
                               "output DM turbulence stream",
                               "null",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &dmstream,
                               &fpi_dmstream},
                              {CLIARG_FILENAME,
                               ".turbfname",
                               "turbulence file cube",
                               "null",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &turbfname,
                               &fpi_turbfname},
                              {CLIARG_FLOAT32,
                               ".wspeed",
                               "wind speed [m/s]",
                               "10.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &turbwspeed,
                               &fpi_turbwspeed},
                              {CLIARG_UINT32,
                               ".NBsamples",
                               "number of samples in cube",
                               "10000",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &NBsamples,
                               &fpi_NBsamples}};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_dmstream].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
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
    }

    return RETURN_SUCCESS;
}




static CLICMDDATA CLIcmddata = {
    "dmturb", "DM turbulence", CLICMD_FIELDS_DEFAULTS};



// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




//
// innerscale and outerscale in pixel
// von Karman spectrum
//
static errno_t make_master_turbulence_screen(const char *ID_name1,
                                             const char *ID_name2,
                                             long        size,
                                             float       outerscale,
                                             float       innerscale)
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
    if (IDv != -1)
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
    for (uint32_t ii = 0; ii < size; ii++)
        for (uint32_t jj = 0; jj < size; jj++)
        {
            dx = 1.0 * ii - size / 2;
            dy = 1.0 * jj - size / 2;

            if (RLIMMODE == 1)
            {
                r = sqrt(dx * dx + dy * dy);
                if (r < rlim)
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
    for (uint32_t ii = 0; ii < size; ii++)
        for (uint32_t jj = 0; jj < size; jj++)
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
    for (uint32_t ii = 1; ii < Dlim; ii++)
        for (uint32_t jj = 1; jj < Dlim; jj++)
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
    for (uint32_t ii = 1; ii < Dlim; ii++)
        for (uint32_t jj = 1; jj < Dlim; jj++)
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




long make_DMturbcube(char    *IDoutname,
                     uint32_t xsize,
                     uint32_t ysize,
                     uint32_t NBsamples)
{
    uint32_t imsize = 2048;

    uint64_t xysize = xsize;
    xysize *= ysize;


    uint32_t zsize = NBsamples;


    imageID IDs1;
    imageID IDs2;
    load_fits("turbscreen1.fits", "screen1", 1, &IDs1);
    load_fits("turbscreen2.fits", "screen2", 1, &IDs2);
    list_image_ID();

    if (IDs1 == -1)
    {
        make_master_turbulence_screen("screen1", "screen2", imsize, 200.0, 1.0);
        IDs1          = image_ID("screen1");
        imageID IDk   = make_gauss("kernim", imsize, imsize, 20.0, 1.0);
        double  totim = 0.0;
        for (uint64_t ii = 0; ii < imsize * imsize; ii++)
        {
            totim += data.image[IDk].array.F[ii];
        }
        for (uint64_t ii = 0; ii < imsize * imsize; ii++)
        {
            data.image[IDk].array.F[ii] /= totim;
        }
        IDs2 = fconvolve("screen1", "kernim", "screen2");
        delete_image_ID("kernim", DELETE_IMAGE_ERRMODE_WARNING);
        save_fits("screen1", "turbscreen1.fits");
        save_fits("screen2", "turbscreen2.fits");
    }

    printf("ARRAY SIZE = %ld %ld\n",
           (long) data.image[IDs1].md[0].size[0],
           (long) data.image[IDs1].md[0].size[1]);
    uint32_t size_sx = data.image[IDs1].md[0].size[0];
    uint32_t size_sy = data.image[IDs1].md[0].size[1];


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

    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    make_DMturbcube(turbfname, xsize, ysize, *NBsamples);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

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
