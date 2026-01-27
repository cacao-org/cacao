#include "ImageStreamIO/ImageStruct.h"
/**
 * @file    zonalfilter.c
 * @brief   Apply zonal filtering in DM space
 *
 *
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"
#include "timeutils.h"

#include "COREMOD_iofits/COREMOD_iofits.h"


// Local variables pointers
static uint64_t *AOloopindex;

static char *inzval;
static long  fpi_inzval;

static char *outzval;
static long  fpi_outzval;


// convenient loop on/off toggle
static int64_t *loopON;
static long     fpi_loopON;

// keep loop open for NBstep
// -1 is infinite
static int64_t *loopNBstep;
static long     fpi_loopNBstep;

// convenient loop on/off toggle
static int64_t *loopZERO;
static long     fpi_loopZERO;




static float *loopgain;
static long   fpi_loopgain;

static float *loopmult;
static long   fpi_loopmult;

static float *looplimit;
static long   fpi_looplimit;










static CLICMDARGDEF farg[] =
{
    {
        // AO loop index. Used for naming streams aolX_
        CLIARG_UINT64,
        ".AOloopindex",
        "AO loop index",
        "0",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &AOloopindex,
        NULL
    },
    {
        CLIARG_STREAM,
        ".inzval",
        "input DM zonal values",
        "aol0_actvalDM",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inzval,
        &fpi_inzval
    },
    {
        CLIARG_STREAM,
        ".outzval",
        "output DM zonal values",
        "aol0_actvalDMf",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outzval,
        &fpi_outzval
    },
    {
        CLIARG_ONOFF,
        ".loopON",
        "loop on/off (off=freeze)",
        "ON",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopON,
        &fpi_loopON
    },
    {
        CLIARG_INT64,
        ".loopNBstep",
        "loop nb steps (-1 = inf)",
        "-1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopNBstep,
        &fpi_loopNBstep
    },
    {
        CLIARG_ONOFF,
        ".loopZERO",
        "loop zero",
        "OFF",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopZERO,
        &fpi_loopZERO
    },
    {
        CLIARG_FLOAT32,
        ".loopgain",
        "loop gain (speed)",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopgain,
        &fpi_loopgain
    },
    {
        CLIARG_FLOAT32,
        ".loopmult",
        "loop mult (attenuation)",
        "0.95",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &loopmult,
        &fpi_loopmult
    },
    {
        CLIARG_FLOAT32,
        ".looplimit",
        "loop limit",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &looplimit,
        &fpi_looplimit
    }
};




// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_inzval].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;


        data.fpsptr->parray[fpi_loopON].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopZERO].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopNBstep].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_looplimit].fpflag |= FPFLAG_WRITERUN;
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
    "zonalfilter", "zonal filtering", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{
    printf("Zonal filtering\n");


    printf(
        "Main input/output streams :\n"
        "[STREAM]   <.inzval>    input zonal values\n"
        "[STREAM]   <.outzval>   output zonal values\n");

    return RETURN_SUCCESS;
}




/**
 * @brief Zonal filtering AO processing
 *
 * Basic modal control. Each mode controlled independently.
 *
 * Control parameters for each mode are:
 * - (g) gain
 * - (m) mult
 * - (z) zeropt
 * - (l) limit
 *
 * PROCESSING
 * Output (o) is computed from input (i) according to following steps :
 *
 * Apply gain :
 * o += (z-i)*g
 *
 * Apply mult :
 * o = z + m*(o-z)
 *
 * Apply limit:
 * if o>z+l -> o = z+l
 * if o<z-l -> o = z-l
 *
 *
 *
 * @return errno_t
 */
static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to input DM array and get number of actuators
    //
    IMGID imginDM = mkIMGID_from_name(inzval);
    resolveIMGID(&imginDM, ERRMODE_ABORT);
    printf("%u modes\n", imginDM.md->size[0]);
    uint32_t dmxsize = imginDM.md->size[0];
    uint32_t dmysize = imginDM.md->size[1];

    uint32_t dmxysize = dmxsize * dmysize;



    // allocate memory for temporary output mode values

    // current control values
    float *zvalDMc = (float *) malloc(sizeof(float) * dmxysize);
    for(uint32_t act; act < dmxysize; act++)
    {
        zvalDMc[act] = 0.0;
    }



    // connect/create output
    //
    IMGID imgout = stream_connect_create_2Df32(outzval, dmxsize, dmysize);
    {
        for(uint32_t act = 0; act < dmxysize; act++)
        {
            data.image[imgout.ID].array.F[act] = 0.0;
        }
        ImageStreamIO_UpdateIm(imgout.im);
    }









    // ========================= ZODAL GAIN ===========================
    printf("Setting up zonal gain\n");

    IMGID imgzgain;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zgain", *AOloopindex);
        imgzgain = stream_connect_create_2Df32(name, dmxsize, dmysize);
    }
    list_image_ID();
    printf(" zgain ID = %ld\n", imgzgain.ID);
    fflush(stdout);

    // zonal gains factors
    // to be multiplied by overal gain to become mgain
    // allows for single-parameter gain tuning
    IMGID imgzgainfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zgainfact", *AOloopindex);
        imgzgainfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
        printf("%s  ID = %ld\n", imgzgainfact.name, imgzgainfact.ID);
        list_image_ID();
        for(uint32_t act = 0; act < dmxysize; act++)
        {
            imgzgainfact.im->array.F[act] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgzgainfact.im);
    }




    // ========================= MODAL MULT ==========================
    printf("Setting up zonal mult\n");

    IMGID imgzmult;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zmult", *AOloopindex);
        imgzmult = stream_connect_create_2Df32(name, dmxsize, dmysize);
    }

    // zonal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgzmultfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zmultfact", *AOloopindex);
        imgzmultfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
        for(uint32_t act = 0; act < dmxysize; act++)
        {
            imgzmultfact.im->array.F[act] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgzmultfact.im);
    }





    // ========================= ZONAL ZEROPOINT ==========================
    printf("Setting up zonal zero point\n");

    IMGID imgzzeropoint;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zzeropoint", *AOloopindex);
        imgzzeropoint = stream_connect_create_2Df32(name, dmxsize, dmysize);
        for(uint32_t act = 0; act < dmxysize; act++)
        {
            imgzzeropoint.im->array.F[act] = 0.0;
        }
        ImageStreamIO_UpdateIm(imgzzeropoint.im);
    }



    // ========================= ZONAL LIMIT ==========================
    printf("Setting up zonal limit\n");

    IMGID imgzlimit;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zlimit", *AOloopindex);
        imgzlimit = stream_connect_create_2Df32(name, dmxsize, dmysize);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgzlimitfact;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zlimitfact", *AOloopindex);
        imgzlimitfact = stream_connect_create_2Df32(name, dmxsize, dmysize);
        for(uint32_t act = 0; act < dmxysize; act++)
        {
            imgzlimitfact.im->array.F[act] = 1.0;
        }
        ImageStreamIO_UpdateIm(imgzlimitfact.im);
    }






    // ========================= MODAL LIMIT COUNTER ==================
    long *zlimitcntarray = (long *) malloc(sizeof(long) * dmxysize);
    long modal_limit_counter = 0;
    for(uint32_t act = 0; act < dmxysize; act++)
    {
        zlimitcntarray[act] = 0;
    }

    IMGID imgzlimitcntfrac;
    {
        char name[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(name, "aol%lu_zlimitcntfrac", *AOloopindex);
        imgzlimitcntfrac = stream_connect_create_2Df32(name, dmxsize, dmysize);
    }





    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        // zero loop
        if(data.fpsptr->parray[fpi_loopZERO].fpflag & FPFLAG_ONOFF)
        {
            for(uint32_t act = 0; act < dmxysize; act++)
            {
                // set goal position to zero
                zvalDMc[act] = 0.0;
            }
            memcpy(imgout.im->array.F, zvalDMc, sizeof(float) * dmxysize);
            processinfo_update_output_stream(processinfo, imgout.im, NULL);

            // toggle back to OFF
            data.fpsptr->parray[fpi_loopZERO].fpflag &= ~FPFLAG_ONOFF;
        }


        int act0 = 1000;

        if((*loopON) == 1)
        {
            if(*loopNBstep > 0)
            {
                *loopNBstep                                    = *loopNBstep - 1;
                data.fpsptr->parray[fpi_loopNBstep].val.i64[0] = *loopNBstep;
            }
            if(*loopNBstep == 0)
            {
                *loopON = 0;
                // set loop to OFF
                data.fpsptr->parray[fpi_loopON].fpflag &= ~FPFLAG_ONOFF;
                *loopNBstep = 1;
            }

            // Pre-allocations for modal loop
            double zvalin;
            double zvalout;
            float  limit;

            // Apply zonal control filtering
            //
            for(uint32_t act = 0; act < dmxysize; act++)
            {
                // grab input value, relative to zeropt
                zvalin = imginDM.im->array.F[act] - imgzzeropoint.im->array.F[act];


                //add the new delta command to the integrated command with leak: this is the goal position
                zvalDMc[act] = (1.0 - imgzgain.im->array.F[act]) * zvalDMc[act] +
                               imgzgain.im->array.F[act] * zvalin;

                // Apply mult coeff
                zvalDMc[act]  *= imgzmult.im->array.F[act];

                // apply LIMIT
                limit = imgzlimit.im->array.F[act];
                if(zvalDMc[act] > limit)
                {
                    zvalDMc[act] = limit;
                    zlimitcntarray[act] ++;
                }
                if(zvalDMc[act] < -limit)
                {
                    zvalDMc[act] = -limit;
                    zlimitcntarray[act] ++;
                }
            }


            for(uint32_t act = 0; act < dmxysize; act++)
            {
                data.image[imgout.ID].array.F[act] = zvalDMc[act] +
                                                     imgzzeropoint.im->array.F[act];
            }
            ImageStreamIO_UpdateIm(imgout.im);



            // Update individual gain, mult and limit values
            // This is done AFTER computing mode values to minimize latency
            //
            for(uint32_t act = 0; act < dmxysize; act++)
            {
                imgzgain.im->array.F[act] =
                    imgzgainfact.im->array.F[act] * (*loopgain);
            }
            processinfo_update_output_stream(processinfo, imgzgain.im, NULL);


            for(uint32_t act = 0; act < dmxysize; act++)
            {
                imgzmult.im->array.F[act] =
                    imgzmultfact.im->array.F[act] * (*loopmult);
            }
            processinfo_update_output_stream(processinfo, imgzmult.im, NULL);


            for(uint32_t act = 0; act < dmxysize; act++)
            {
                imgzlimit.im->array.F[act] =
                    imgzlimitfact.im->array.F[act] * (*looplimit);
            }
            processinfo_update_output_stream(processinfo, imgzlimit.im, NULL);

        }


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    free(zvalDMc);

    free(zlimitcntarray);


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__zonalfilter()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
