#include "ImageStreamIO/ImageStruct.h"
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
static char *indarkname;

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

// position data
// xrel, yrel, xabs, yabs, flux, pixcnt
static char *outspotpos;


// 2D transformation matrix between pixel pos and TT value
static float *mappingXX;
static long      fpi_mappingXX = -1;
static float *mappingYY;
static long      fpi_mappingYY = -1;
static float *mappingXY;
static long      fpi_mappingXY = -1;
static float *mappingYX;
static long      fpi_mappingYX = -1;

// 2D position vector matching control TT
static char *outTTvec;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".insname",
        "input image",
        "im1",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &inimname,
        NULL
    },
    {
        CLIARG_IMG,
        ".indark_name",
        "input image dark (optional)",
        "imdark",
        FPFLAG_DEFAULT_INPUT,
        (void **) &indarkname,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".spotsize",
        "approximate spot size [pix]",
        "3.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &spotsize,
        &fpi_spotsize
    },
    {
        CLIARG_FLOAT32,
        ".spotx0",
        "approx spot location x",
        "100.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &spotx0,
        &fpi_spotx0
    },
    {
        CLIARG_FLOAT32,
        ".spoty0",
        "approx spot location y",
        "60.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &spoty0,
        &fpi_spoty0
    },
    {
        CLIARG_FLOAT32,
        ".searchrad",
        "search radius",
        "10",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &searchrad,
        &fpi_searchrad
    },
    {
        CLIARG_STR,
        ".outspotpos",
        "output spot position data",
        "ttdat",
        FPFLAG_DEFAULT_INPUT,
        (void **) &outspotpos,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".mappingXX",
        "mapping XX coeff",
        "1.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &mappingXX,
        &fpi_mappingXX
    },
    {
        CLIARG_FLOAT32,
        ".mappingYY",
        "mapping YY coeff",
        "1.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &mappingYY,
        &fpi_mappingYY
    },
    {
        CLIARG_FLOAT32,
        ".mappingXY",
        "mapping XY coeff",
        "0.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &mappingXY,
        &fpi_mappingXY
    },
    {
        CLIARG_FLOAT32,
        ".mappingYX",
        "mapping YX coeff",
        "0.0",
        (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),
        (void **) &mappingYX,
        &fpi_mappingYX
    },
    {
        CLIARG_STR,
        ".outTTvec",
        "output 2D TT vector (control TT)",
        "ttvec",
        FPFLAG_DEFAULT_INPUT,
        (void **) &outTTvec,
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
        // allow for change of parameter during runtime
        data.fpsptr->parray[fpi_mappingXX].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_mappingYY].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_mappingXY].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_mappingYX].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_spotx0].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_spoty0].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_searchrad].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_spotsize].fpflag |= FPFLAG_WRITERUN;
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
    IMGID *indarkimg,
    float spot_size,
    float spot_x0,
    float spot_y0,
    float spot_searchrad,
    IMGID *outdatimg,
    float mappingXX,
    float mappingYY,
    float mappingXY,
    float mappingYX,
    IMGID *outvecimg
)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code

    // check input image exists
    resolveIMGID(inimg, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);

    // get image size
    uint32_t xsize = inimg->md->size[0];
    uint32_t ysize = inimg->md->size[1];
    uint64_t xysize = (uint64_t) xsize;
    xysize *= ysize;


    // check if dark image exists
    resolveIMGID(indarkimg, ERRMODE_NULL, data.image, data.NB_MAX_IMAGE);

    // Checko output
    //
    resolveIMGID(outdatimg, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);
    resolveIMGID(outvecimg, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);


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
    double pixcnt = 0.0;


    // If dark image is present, subtract it from inimg
    if(indarkimg->ID != -1)
    {
        for(uint32_t ii = iistart; ii < iiend; ii++)
            for(uint32_t jj = jjstart; jj < jjend; jj++)
            {
                float x = 1.0*ii - spot_x0;
                float y = 1.0*jj - spot_y0;
                float v = inimg->im->array.F[jj * xsize + ii]- indarkimg->im->array.F[jj * xsize + ii];

                xpos += x*v;
                ypos += y*v;
                sumval += v;
                pixcnt += 1.0;
            }
    }
    else
    {
        for(uint32_t ii = iistart; ii < iiend; ii++)
            for(uint32_t jj = jjstart; jj < jjend; jj++)
            {
                float x = 1.0*ii - spot_x0;
                float y = 1.0*jj - spot_y0;
                float v = inimg->im->array.F[jj * xsize + ii];

                xpos += x*v;
                ypos += y*v;
                sumval += v;
                pixcnt += 1.0;
            }
    }

    xpos /= sumval;
    ypos /= sumval;

    outdatimg->im->array.F[0] = xpos;
    outdatimg->im->array.F[1] = ypos;
    outdatimg->im->array.F[2] = xpos + spot_x0;
    outdatimg->im->array.F[3] = ypos + spot_y0;
    outdatimg->im->array.F[4] = sumval;
    outdatimg->im->array.F[5] = pixcnt;

    outvecimg->im->array.F[0] = xpos * mappingXX + ypos * mappingYX;
    outvecimg->im->array.F[1] = xpos * mappingXY + ypos * mappingYY;

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // resolve image and create IMGID
    IMGID inimg = imgid_make_from_name(inimname);
    resolveIMGID(&inimg, ERRMODE_ABORT, data.image, data.NB_MAX_IMAGE);

    // resolve dark image and create IMGID (optional)
    IMGID indarkimg = imgid_make_from_name(indarkname);
    resolveIMGID(&indarkimg, ERRMODE_NULL, data.image, data.NB_MAX_IMAGE);

    // Create output
    //
    IMGID outposimg;
    {
        printf("CONNECTING / CREATING output stream\n");
        outposimg =
            stream_connect_create_2D(outspotpos, 6, 1, _DATATYPE_FLOAT);
    }

    IMGID outTTvecimg;
    {
        printf("CONNECTING / CREATING output stream\n");
        outTTvecimg =
            stream_connect_create_2D(outTTvec, 2, 1, _DATATYPE_FLOAT);
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {

        spot_position(
            &inimg,
            &indarkimg,
            *spotsize,
            *spotx0,
            *spoty0,
            *searchrad,
            &outposimg,
            *mappingXX,
            *mappingYY,
            *mappingXY,
            *mappingYX,
            &outTTvecimg
        );

        // stream is updated here, and not in the function called above, so that
        // the above function can be chained with others
        processinfo_update_output_stream(processinfo, outposimg.im, NULL); // outposimg
        processinfo_update_output_stream(processinfo, outTTvecimg.im, NULL); // outTTvecimg
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&inimg);
    imgid_free(&indarkimg);
    imgid_free(&outposimg);
    imgid_free(&outTTvecimg);

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
