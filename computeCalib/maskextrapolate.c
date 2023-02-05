/**
 * @file maskextrapolate.c
 *
 */

#include <math.h>


#include "CommandLineInterface/CLIcore.h"




// Input modes to be masked/extrapolated
//
static char *inmodeC;
static long  fpi_inmodeC;

// mask image
//
static char *maskim;
static long  fpi_maskim;


// extended mask image
//
static char *extmaskim;
static long  fpi_extmaskim;


static char *outmodeC;
static long  fpi_outmodeC;


static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".inmodeC",
        "input modes",
        "inmodeC",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inmodeC,
        &fpi_inmodeC
    },
    {
        CLIARG_IMG,
        ".maskim",
        "input mask",
        "maskim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &maskim,
        &fpi_maskim
    },
    {
        CLIARG_IMG,
        ".extmaskim",
        "extended input mask",
        "extmaskim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &extmaskim,
        &fpi_extmaskim
    },
    {
        CLIARG_STR,
        ".outmodeC",
        "output modes",
        "outmodeC",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outmodeC,
        &fpi_outmodeC
    }
};




// Optional custom configuration setup. comptbuff
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

    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "maskextrapolate", "mask and extrapolate modes", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imginmodeC = mkIMGID_from_name(inmodeC);
    resolveIMGID(&imginmodeC, ERRMODE_ABORT);
    uint32_t xsize = imginmodeC.md->size[0];
    uint32_t ysize = imginmodeC.md->size[1];
    uint64_t xysize = xsize;
    xysize *= ysize;
    uint32_t NBmodes = imginmodeC.md->size[2];
    printf("%u modes\n", NBmodes);

    IMGID imgmask = mkIMGID_from_name(maskim);
    resolveIMGID(&imgmask, ERRMODE_ABORT);

    IMGID imgextmask = mkIMGID_from_name(extmaskim);
    resolveIMGID(&imgextmask, ERRMODE_ABORT);



    IMGID imgoutmoudeC = makeIMGID_3D(outmodeC, xsize, ysize, NBmodes);
    createimagefromIMGID(&imgoutmoudeC);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        for(uint32_t ii = 0; ii < xsize; ii++)
        {
            for(uint32_t jj = 0; jj < xsize; jj++)
            {
                if(imgmask.im->array.F[jj*xsize+ii] > 0.5)
                {
                    // in mask -> copy pixel value to output
                    for(uint32_t mi=0; mi<NBmodes; mi++)
                    {
                        imgoutmoudeC.im->array.F[xysize*mi + jj*xsize + ii] = imginmodeC.im->array.F[xysize*mi + jj*xsize + ii];
                    }
                }
                else if (imgextmask.im->array.F[jj*xsize+ii] > 0.5)
                {
                    // pixel is in extmask, but not in mask -> run extrapolation

                    // find nearest active pixel
                    float nearest_dist2 = xysize;
                    uint32_t nearest_ii = 0;
                    uint32_t nearest_jj = 0;
                    for(uint32_t ii1=0; ii1<xsize; ii1++)
                    {
                        for(uint32_t jj1=0; jj1<ysize; jj1++)
                        {
                            if(imgmask.im->array.F[jj1*xsize+ii1] > 0.5)
                            {
                                float dx = ii-ii1;
                                float dy = jj-jj1;
                                float dr2 = dx*dx + dy*dy;

                                if( dr2 < nearest_dist2 )
                                {
                                    nearest_dist2 = dr2;
                                    nearest_ii = ii1;
                                    nearest_jj = jj1;
                                }
                            }
                        }
                    }

                    // Kernel radius
                    int kradint = (int) (sqrt(nearest_dist2)+1.0);

                    int iimin = ii - kradint;
                    if(iimin < 0)
                    {
                        iimin = 0;
                    }
                    int iimax = ii + kradint;
                    if(iimax > xsize)
                    {
                        iimax = xsize;
                    }

                    int jjmin = jj - kradint;
                    if(jjmin < 0)
                    {
                        jjmin = 0;
                    }
                    int jjmax = jj + kradint;
                    if(jjmax > ysize)
                    {
                        jjmax = ysize;
                    }

                    // nearest pixel
                    //
                    for(uint32_t mi=0; mi<NBmodes; mi++)
                    {
                        imgoutmoudeC.im->array.F[xysize*mi + jj*xsize + ii] = imginmodeC.im->array.F[xysize*mi + nearest_jj*xsize + nearest_ii];
                    }


                }

            }
        }

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__maskextrapolate()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
