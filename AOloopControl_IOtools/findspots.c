/**
 * @file findspots.c
 * @brief Find spots in WFS image
 *
 *
 */

#include "CommandLineInterface/CLIcore.h"

// quicksort
#include "COREMOD_tools/COREMOD_tools.h"

static char *inimname;

// approximate spot size
//
static float *spotsize;
static long      fpi_spotsize = -1;

// exclusion distance
// minimum distance between spots
//
static float *spotexcldist;
static long      fpi_spotexcldist = -1;

// max number of spots
//
static uint32_t *maxnbspot;
static long      fpi_maxnbspot = -1;


static char *outmapcname;

static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".in_name",
        "input image",
        "im1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inimname,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".spotsize",
        "approximate spot size",
        "3.0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &spotsize,
        &fpi_spotsize
    },
    {
        CLIARG_FLOAT32,
        ".spotexcldist",
        "exclusion distance",
        "10.0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &spotexcldist,
        &fpi_spotexcldist
    },
    {
        CLIARG_UINT32,
        ".maxnbspot",
        "max number of spots",
        "19",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &maxnbspot,
        &fpi_maxnbspot
    },
    {
        CLIARG_STR,
        ".outmapc",
        "output mapping cube",
        "mapc",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outmapcname,
        NULL
    }
};



// Optional custom configuration setup
// Runs once at conf startup
//
// To use this function, set :
// CLIcmddata.FPS_customCONFsetup = customCONFsetup
// when registering function
// (see end of this file)
//
static errno_t customCONFsetup()
{
    return RETURN_SUCCESS;
}

// Optional custom configuration checks
// Runs at every configuration check loop iteration
//
// To use this function, set :
// CLIcmddata.FPS_customCONFcheck = customCONFcheck
// when registering function
// (see end of this file)
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
    "findspots",
    "find spots in inmage",
    CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}



static errno_t find_image_spots(
    IMGID inimg,
    float spot_size,
    float spot_excl_dist,
    uint32_t nb_spot_max
)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code


    printf("Looking for spots ...\n");


    // get image size
    uint32_t xsize = inimg.size[0];
    uint32_t ysize = inimg.size[1];
    uint64_t xysize = (uint64_t) xsize;
    xysize *= ysize;



    // Create output
    //
    IMGID spotoutimg;
    spotoutimg =
        stream_connect_create_2D("spotscan", xsize, ysize, _DATATYPE_FLOAT);

    // median scan
    float *valarray = (float *) malloc(sizeof(float) * 4 * ((
                                           int) spot_size + 1) * ((int) spot_size + 1));

    for(uint32_t ii = 0; ii < xsize; ii++)
    {
        for(uint32_t jj = 0; jj < ysize; jj++)
        {
            uint32_t nbpix = 0;

            int ii1min = ii - (int)(spot_size + 1);
            if(ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = ii + (int)(spot_size + 1);
            if(ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jj - (int)(spot_size + 1);
            if(jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jj + (int)(spot_size + 1);
            if(jj1max > (int) ysize)
            {
                jj1max = ysize;
            }

            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = 1.0 * ii - ii1;
                    float dy = 1.0 * jj - jj1;
                    float r2 = dx * dx + dy * dy;
                    if(r2 < spot_size * spot_size)
                    {
                        valarray[nbpix] = inimg.im->array.F[jj1 * xsize + ii1];
                        nbpix ++;
                    }
                }
            }
            quick_sort_float(valarray, nbpix);

            float val = valarray[(int)(0.5 * nbpix)];
            spotoutimg.im->array.F[jj * xsize + ii] = val;
        }
    }
    free(valarray);



    // Create output map cube
    //
    IMGID mapcimg;
    mapcimg =
        stream_connect_create_3Df32(outmapcname, xsize, ysize, nb_spot_max);

    // zero out array
    //

    for (uint64_t ii=0; ii<mapcimg.md->nelement; ii++)
    {
        mapcimg.im->array.F[ii] = 0.0;
    }


    // find spots
    //
    float *spotxarray = (float *) malloc(sizeof(float) * nb_spot_max);
    float *spotyarray = (float *) malloc(sizeof(float) * nb_spot_max);
    float *spotvarray = (float *) malloc(sizeof(float) * nb_spot_max);

    uint32_t spotindex = 0;
    while(spotindex < nb_spot_max)
    {
        // Find strongest peak
        //
        float vpeak = 0.0;
        uint32_t iipeak = 0;
        uint32_t jjpeak = 0;
        for(uint32_t ii = 0; ii < xsize; ii++)
        {
            for(uint32_t jj = 0; jj < ysize; jj++)
            {
                if(spotoutimg.im->array.F[jj * xsize + ii] > vpeak)
                {
                    iipeak = ii;
                    jjpeak = jj;
                    vpeak = spotoutimg.im->array.F[jj * xsize + ii];
                }
            }
        }
        // report spot
        printf("SPOT %2d   %4u x %4u    %f\n", spotindex, iipeak, jjpeak, vpeak);
        spotxarray[spotindex] = 1.0 * iipeak;
        spotyarray[spotindex] = 1.0 * jjpeak;
        spotvarray[spotindex] = vpeak;




        // zero area around spot
        {
            int ii1min = iipeak - (int)(spot_excl_dist + 1);
            if(ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = iipeak + (int)(spot_excl_dist + 1);
            if(ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jjpeak - (int)(spot_excl_dist + 1);
            if(jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jjpeak + (int)(spot_excl_dist + 1);
            if(jj1max > (int) ysize)
            {
                jj1max = ysize;
            }
            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = 1.0 * iipeak - ii1;
                    float dy = 1.0 * jjpeak - jj1;
                    float r2 = dx * dx + dy * dy;
                    if(r2 < spot_excl_dist * spot_excl_dist)
                    {
                        spotoutimg.im->array.F[jj1 * xsize + ii1] = 0.0;
                    }
                }
            }
        }



        // write mapc slice
        //
        {
            int ii1min = iipeak - (int)(spot_size + 1);
            if(ii1min < 0)
            {
                ii1min = 0;
            }
            int ii1max = iipeak + (int)(spot_size + 1);
            if(ii1max > (int) xsize)
            {
                ii1max = xsize;
            }

            int jj1min = jjpeak - (int)(spot_size + 1);
            if(jj1min < 0)
            {
                jj1min = 0;
            }
            int jj1max = jjpeak + (int)(spot_size + 1);
            if(jj1max > (int) ysize)
            {
                jj1max = ysize;
            }
            double spotflux = 0.0;
            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    float dx = 1.0 * iipeak - ii1;
                    float dy = 1.0 * jjpeak - jj1;
                    float r2 = dx * dx + dy * dy;
                    if(r2 < spot_size * spot_size)
                    {
                        mapcimg.im->array.F[xysize * spotindex + jj1 * xsize + ii1] =
                            inimg.im->array.F[jj1 * xsize + ii1];
                        spotflux += inimg.im->array.F[jj1 * xsize + ii1];
                    }
                }
            }
            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
            {
                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                {
                    mapcimg.im->array.F[xysize * spotindex + jj1 * xsize + ii1] /= spotflux;
                }
            }
        }

        spotindex++;
    }




    free(spotxarray);
    free(spotyarray);
    free(spotvarray);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID inimg = mkIMGID_from_name(inimname);
    resolveIMGID(&inimg, ERRMODE_ABORT);

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {

        find_image_spots(inimg, *spotsize, *spotexcldist, *maxnbspot);

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__findspots()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
