/**
 * @file spotpos.c
 * @brief Measure spot position, photocenter
 *
 *
 */
#include <float.h>

#include "CommandLineInterface/CLIcore.h"

// quicksort
#include "COREMOD_tools/COREMOD_tools.h"

static char *inimname;

// approximate spot size
//
static float *spotsize;
static long      fpi_spotsize = -1;

// approximate spot location
// search will be centered around this coords
static float *spotx0;
static long      fpi_spotx0 = -1;
static float *spoty0;
static long      fpi_spoty0 = -1;

// Search radius around spotx0, spoty0
static float *searchrad;
static long      fpi_searchrad = -1;


static char *outspotpos;

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
        "approximate spot size [pix]",
        "3.0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &spotsize,
        &fpi_spotsize
    },
    {
        CLIARG_FLOAT32,
        ".spotx0",
        "approx spot location x",
        "100.0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &spotx0,
        &fpi_spotx0
    },
    {
        CLIARG_FLOAT32,
        ".spoty0",
        "approx spot location y",
        "60.0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &spoty0,
        &fpi_spoty0
    },
    {
        CLIARG_UINT32,
        ".searchrad",
        "search radius",
        "10",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &searchrad,
        &fpi_searchrad
    },
    {
        CLIARG_STR,
        ".outspotpos",
        "output spot position vector",
        "ttvect",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outspotpos,
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
    "spotpos",
    "measure spot position, photocenter",
    CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}



static errno_t spot_position(
    IMGID *inimg,
    float spot_size,
    float spot_x0,
    float spot_y0,
    float spot_searchrad,
    IMGID *outimg
)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code

    // get image size
    uint32_t xsize = inimg->size[0];
    uint32_t ysize = inimg->size[1];
    uint64_t xysize = (uint64_t) xsize;
    xysize *= ysize;

    // Create/connect to output
    //
    IMGID spotposimg;
    spotposimg =
        stream_connect_create_2D(outimg->name, 2, 1, _DATATYPE_FLOAT);


    float xstart = spot_x0 - spot_searchrad;
    uint32_t iistart = 0;
    if(xstart > 0.0)
    {
        iistart = (uint32_t) xstart;
    }

    float xend = spot_x0 + spot_searchrad;
    uint32_t iiend = xsize;
    if(xend < xsize)
    {
        iiend = (uint32_t) xend;
    }


    float ystart = spot_y0 - spot_searchrad;
    uint32_t jjstart = 0;
    if(ystart > 0.0)
    {
        jjstart = (uint32_t) ystart;
    }

    float yend = spot_y0 + spot_searchrad;
    uint32_t jjend = ysize;
    if(yend < ysize)
    {
        jjend = (uint32_t) yend;
    }




    double xpos = 0.0;
    double ypos = 0.0;
    double sumval = 0.0;
    for(uint32_t ii = iistart; ii < iiend; ii++)
    {
        for(uint32_t jj = jjstart; jj < jjend; jj++)
        {
            float x = 1.0*ii - spot_x0;
            float y = 1.0*jj - spot_y0;
            float v = inimg->im->array.F[jj * xsize + ii];

            xpos += x*v;
            ypos += y*v;
            sumval += v;
        }
    }
    xpos /= sumval;
    ypos /= sumval;

    spotposimg.im->array.F[0] = xpos;
    spotposimg.im->array.F[0] = ypos;


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID inimg = mkIMGID_from_name(inimname);
    resolveIMGID(&inimg, ERRMODE_ABORT);

    // Create output
    //
    IMGID outposimg;
    {
        printf("CONNECTING / CREATING output stream\n");
        outposimg =
            stream_connect_create_2D(outspotpos, 2, 1, _DATATYPE_FLOAT);
    }


    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {

        spot_position(
            &inimg,
            *spotsize,
            *spotx0,
            *spoty0,
            *searchrad,
            &outposimg
        );

        // stream is updated here, and not in the function called above, so that
        // the above function can be chained with others
        processinfo_update_output_stream(processinfo, outposimg.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__spotpos()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
