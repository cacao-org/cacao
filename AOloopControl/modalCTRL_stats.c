/**
 * @file    modaloptimize.c
 * @brief   Optimize modal control parameters
 *
 *
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

// Local variables pointers
static uint64_t *AOloopindex;

static uint64_t *samplesize;
static long      fpi_samplesize;



static CLICMDARGDEF farg[] = {{CLIARG_UINT64,
                               ".AOloopindex",
                               "AO loop index",
                               "0",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &AOloopindex,
                               NULL},
                              {CLIARG_UINT64,
                               ".samplesize",
                               "number of point per set",
                               "10000",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &samplesize,
                               &fpi_samplesize}};

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_samplesize].fpflag |= FPFLAG_WRITERUN;
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
    "modalCTRLstats", "compute modal control stats", CLICMD_FIELDS_DEFAULTS};




// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();
    /*
    // connect to input mode values array and get number of modes
    //
    IMGID imgin = mkIMGID_from_name(inmval);
    resolveIMGID(&imgin, ERRMODE_ABORT);
    printf("%u modes\n", imgin.md->size[0]);
    uint32_t NBmode = imgin.md->size[0];



    // allocate memory for temporary output mode values
    float *mvalout = (float *) malloc(sizeof(float) * NBmode);




    // connect/create output mode coeffs
    IMGID imgout = stream_connect_create_2Df32(outmval, NBmode, 1);
    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        data.image[imgout.ID].array.F[mi] = 0.0;
    }




    // ========================= MODAL GAIN ===========================
    printf("Setting up modal gain\n");

    IMGID imgmgain;
    {
        char mgainname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mgainname, "aol%lu_mgain", *AOloopindex);
        imgmgain = stream_connect_create_2Df32(mgainname, NBmode, 1);
    }
    list_image_ID();
    printf(" mgain ID = %ld\n", imgmgain.ID);
    fflush(stdout);

    // modal gains factors
    // to be multiplied by overal gain to become mgain
    // allows for single-parameter gain tuning
    IMGID imgmgainfact;
    {
        char mgainfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mgainfactname, "aol%lu_mgainfact", *AOloopindex);
        imgmgainfact = stream_connect_create_2Df32(mgainfactname, NBmode, 1);
        printf("%s  ID = %ld\n", imgmgainfact.name, imgmgainfact.ID);
        list_image_ID();
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmgainfact.im->array.F[mi] = 1.0;
        }
    }



    // ========================= MODAL MULT ==========================
    printf("Setting up modal mult\n");

    IMGID imgmmult;
    {
        char mmultname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mmultname, "aol%lu_mmult", *AOloopindex);
        imgmmult = stream_connect_create_2Df32(mmultname, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmmultfact;
    {
        char mmultfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mmultfactname, "aol%lu_mmultfact", *AOloopindex);
        imgmmultfact = stream_connect_create_2Df32(mmultfactname, NBmode, 1);
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmmultfact.im->array.F[mi] = 1.0;
        }
    }


    // ========================= MODAL ZEROPOINT ==========================
    printf("Setting up modal zero point\n");

    IMGID imgmzeropoint;
    {
        char mzeropointname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mzeropointname, "aol%lu_mzeropoint", *AOloopindex);
        imgmzeropoint = stream_connect_create_2Df32(mzeropointname, NBmode, 1);
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmzeropoint.im->array.F[mi] = 0.0;
        }
    }



    // ========================= MODAL LIMIT ==========================
    printf("Setting up modal limit\n");

    IMGID imgmlimit;
    {
        char mlimitname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mlimitname, "aol%lu_mlimit", *AOloopindex);
        imgmlimit = stream_connect_create_2Df32(mlimitname, NBmode, 1);
    }

    // modal multiiplicative factors
    // to be multiplied by overal mult to become mmult
    // allows for single-parameter mult tuning
    IMGID imgmlimitfact;
    {
        char mlimitfactname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(mlimitfactname, "aol%lu_mlimitfact", *AOloopindex);
        imgmlimitfact = stream_connect_create_2Df32(mlimitfactname, NBmode, 1);
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmlimitfact.im->array.F[mi] = 1.0;
        }
    }




    INSERT_STD_PROCINFO_COMPUTEFUNC_START

    for (uint32_t mi = 0; mi < NBmode; mi++)
    {

        // grab input value from WFS
        double mvalWFS = imgin.im->array.F[mi];

        // offset from mval to zp
        double dmval = imgmzeropoint.im->array.F[mi] - mvalWFS;

        // GAIN
        dmval *= imgmgain.im->array.F[mi];

        // get current DM position
        double mvalDM = imgout.im->array.F[mi];

        // apply scaled offset
        mvalDM += dmval;
        // MULT
        mvalDM *= imgmmult.im->array.F[mi];


        // LIMIT
        float limit = imgmlimit.im->array.F[mi];
        if (mvalDM > limit)
        {
            mvalDM = limit;
        }
        if (mvalDM < -limit)
        {
            mvalDM = -limit;
        }

        mvalout[mi] = mvalDM;
    }

    memcpy(imgout.im->array.F, mvalout, sizeof(float) * NBmode);
    processinfo_update_output_stream(processinfo, imgout.ID);


    // Update individual gain, mult and limit values
    // This is done AFTER computing mode values to minimize latency
    //
    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        imgmgain.im->array.F[mi] = imgmgainfact.im->array.F[mi] * (*loopgain);
    }
    processinfo_update_output_stream(processinfo, imgmgain.ID);


    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        imgmmult.im->array.F[mi] = imgmmultfact.im->array.F[mi] * (*loopmult);
    }
    processinfo_update_output_stream(processinfo, imgmmult.ID);


    for (uint32_t mi = 0; mi < NBmode; mi++)
    {
        imgmlimit.im->array.F[mi] =
            imgmlimitfact.im->array.F[mi] * (*looplimit);
    }
    processinfo_update_output_stream(processinfo, imgmlimit.ID);



    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalout);
    */
    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



    // Register function in CLI
    errno_t
    CLIADDCMD_AOloopControl__modalCTRL_stats()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
