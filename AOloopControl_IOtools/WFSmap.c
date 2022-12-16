/**
 * @file    WFSmap.c
 * @brief   remap WFS image
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

// Local variables pointers
static char *wfsinsname;
static long  fpi_wfsinsname;

static char *mapsname;
static long  fpi_mapsname;

static char *wfsoutsname;
static long  fpi_wfsoutsname;





static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".wfsin",
        "Wavefront sensor input",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsinsname,
        &fpi_wfsinsname
    },
    {
        CLIARG_IMG,
        ".map",
        "WFS mapping",
        "mapim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &mapsname,
        &fpi_mapsname
    },
    {
        CLIARG_STR,
        ".wfsout",
        "Wavefront sensor output",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsoutsname,
        &fpi_wfsoutsname
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "mapWFS", "remap WFS image", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}





errno_t image_pixremap(
    IMGID inimg,
    IMGID mapimg,
    IMGID outimg,
    int reuse
)
{
    DEBUG_TRACE_FSTART();

    static int initialize = 1;
    static uint64_t mapNBpix = 0;
    static uint64_t *map_inpixindex = NULL;
    static uint64_t *map_outpixindex = NULL;
    static float *map_pixcoeff = NULL;
    static uint64_t xysize = 0;

    if(initialize == 1)
    {
        float eps = 1.0e-6;

        //uint32_t *mapNBpix = (uint32_t *) malloc(sizeof(mapNBpix) * mapimg.size[2]);
        printf("%u output pixels\n", mapimg.size[2]);


        // scan map to count pixels
        mapNBpix = 0;
        xysize = mapimg.size[0] * mapimg.size[1];
        for(uint64_t ii = 0; ii < mapimg.size[0]*mapimg.size[1]*mapimg.size[2]; ii++)
        {
            if(fabs(mapimg.im->array.F[ii]) > eps)
            {
                mapNBpix++;
            }
        }
        printf("%lu active pixels in map\n", mapNBpix);

        // allocate mapping arrays
        map_inpixindex = (uint64_t *) malloc(sizeof(uint64_t) * mapNBpix);
        map_outpixindex = (uint64_t *) malloc(sizeof(uint64_t) * mapNBpix);
        map_pixcoeff = (float *) malloc(sizeof(float) * mapNBpix);

        // fill mapping arrays
        uint64_t mappix = 0;
        for(uint32_t kk = 0; kk < mapimg.size[2]; kk++)
        {
            for(uint64_t ii = 0; ii < mapimg.size[0]*mapimg.size[1]; ii++)
            {
                uint64_t pixindex = kk * mapimg.size[0] * mapimg.size[1] + ii;
                if(fabs(mapimg.im->array.F[pixindex]) > eps)
                {
                    map_inpixindex[mappix] = ii;
                    map_outpixindex[mappix] = kk;
                    map_pixcoeff[mappix] = mapimg.im->array.F[pixindex];

                    mappix++;
                }
            }
        }
        initialize = 0;
    }


    for(uint32_t kk = 0; kk < mapimg.size[2]; kk++)
    {
        outimg.im->array.F[kk] = 0.0;
    }
    for(uint64_t mapii = 0; mapii < mapNBpix; mapii ++)
    {
        outimg.im->array.F[map_outpixindex[mapii]] += map_pixcoeff[mapii] *
                inimg.im->array.F[map_inpixindex[mapii]];
    }

    if(reuse == 0)
    {
        free(map_inpixindex);
        free(map_outpixindex);
        free(map_pixcoeff);
        initialize = 1;
    }

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}





static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID wfsinimg = mkIMGID_from_name(wfsinsname);
    resolveIMGID(&wfsinimg, ERRMODE_ABORT);

    IMGID mapimg = mkIMGID_from_name(mapsname);
    resolveIMGID(&mapimg, ERRMODE_ABORT);


    uint32_t sizeout = mapimg.size[2];

    // Create output
    //
    IMGID wfsoutimg;
    wfsoutimg =
        stream_connect_create_2D(wfsoutsname, sizeout, 1, _DATATYPE_FLOAT);




    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        image_pixremap(wfsinimg, mapimg, wfsoutimg, 1);
        processinfo_update_output_stream(processinfo, wfsoutimg.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__WFSmap()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
