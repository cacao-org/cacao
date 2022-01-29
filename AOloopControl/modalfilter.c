/**
 * @file    modalfilter.c
 * @brief   Apply modal filtering
 *
 *
 *
 */

#include <math.h>

#include "CommandLineInterface/CLIcore.h"

// Local variables pointers
static uint64_t *AOloopindex;

static char *inmval;
long         fpi_inmval;

static char *outmval;
long         fpi_outmval;

static float *loopgain;
long          fpi_loopgain;

static float *loopmult;
long          fpi_loopmult;

static float *looplimit;
long          fpi_looplimit;




static CLICMDARGDEF farg[] = {{CLIARG_UINT64,
                               ".AOloopindex",
                               "AO loop index",
                               "0",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &AOloopindex,
                               NULL},
                              {CLIARG_STREAM,
                               ".inmval",
                               "input mode values from WFS",
                               "aol0_modevalWFS",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &inmval,
                               &fpi_inmval},
                              {CLIARG_STREAM,
                               ".outmval",
                               "output mode values to DM",
                               "aol0_modevalDM",
                               CLIARG_VISIBLE_DEFAULT,
                               (void **) &outmval,
                               &fpi_outmval},
                              {CLIARG_FLOAT32,
                               ".loopgain",
                               "loop gain",
                               "0.01",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &loopgain,
                               &fpi_loopgain},
                              {CLIARG_FLOAT32,
                               ".loopmult",
                               "loop mult",
                               "0.95",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &loopmult,
                               &fpi_loopmult}};

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
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

    if (data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata = {
    "modalfilter", "modal filtering", CLICMD_FIELDS_DEFAULTS};




// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}



/**
 * @brief Modal filtering AO processing
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

        // grab input value
        double mval = imgin.im->array.F[mi];
        //        printf(" 0 mval = %lf\n", mval);
        //        fflush(stdout);

        // offset from zp to mval
        double dmval = imgmzeropoint.im->array.F[mi] - mval;
        //        printf(" 1 dmval = %lf\n", dmval);
        //        fflush(stdout);


        // GAIN
        mval += dmval * imgmgain.im->array.F[mi];
        //        printf(" 2 val = %lf\n", mval);
        //        fflush(stdout);


        // MULT
        dmval = mval - imgmzeropoint.im->array.F[mi];
        dmval *= imgmmult.im->array.F[mi];
        //        printf(" 3 dmval = %lf\n", dmval);
        //        fflush(stdout);


        // LIMIT
        float limit = imgmlimit.im->array.F[mi];
        if (dmval > limit)
        {
            dmval = limit;
        }
        if (dmval < -limit)
        {
            dmval = -limit;
        }
        mval = imgmzeropoint.im->array.F[mi] + dmval;
        //        printf(" 4 mval = %lf\n", mval);
        //        fflush(stdout);

        mvalout[mi] = mval;
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

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



    // Register function in CLI
    errno_t
    CLIADDCMD_AOloopControl__modalfilter()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
