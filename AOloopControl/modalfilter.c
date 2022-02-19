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
static long  fpi_inmval;

static char *outmval;
static long  fpi_outmval;

static float *loopgain;
static long   fpi_loopgain;

static float *loopmult;
static long   fpi_loopmult;

static float *looplimit;
static long   fpi_looplimit;




// Compute open loop modes
static int64_t *compOL;
static long     fpi_compOL;

// Latency between DM and WFS
static float *latencyfr;
static long   fpi_latencyfr;


// Shared memory telemetry buffers
static int64_t *comptbuff;
static long     fpi_comptbuff;

// telemetry buffer log2 size range
// buffer sizes will go as 2^n, with n from min to max
static uint32_t *tbuff_l2minsize;
static uint32_t *tbuff_l2maxsize;




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
                               &fpi_loopmult},
                              {CLIARG_FLOAT32,
                               ".looplimit",
                               "loop limit",
                               "1.0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &looplimit,
                               &fpi_looplimit},
                              {CLIARG_ONOFF,
                               ".comp.OLmodes",
                               "compute open loop modes",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &compOL,
                               &fpi_compOL},
                              {CLIARG_FLOAT32,
                               ".comp.latencyfr",
                               "DM to WFS latency [frame]",
                               "1.7",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &latencyfr,
                               &fpi_latencyfr},
                              {CLIARG_ONOFF,
                               ".comp.tbuff",
                               "compute telemetry buffer(s)",
                               "0",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &comptbuff,
                               &fpi_comptbuff},
                              {CLIARG_UINT32,
                               ".comp.tbuffl2minsize",
                               "log2 min size",
                               "2",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &tbuff_l2minsize,
                               NULL},
                              {CLIARG_UINT32,
                               ".comp.tbuffl2maxsize",
                               "log2 max size",
                               "8",
                               CLIARG_HIDDEN_DEFAULT,
                               (void **) &tbuff_l2maxsize,
                               NULL}};




// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if (data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_loopgain].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_loopmult].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_looplimit].fpflag |= FPFLAG_WRITERUN;

        data.fpsptr->parray[fpi_comptbuff].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compOL].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_latencyfr].fpflag |= FPFLAG_WRITERUN;
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



    // OPEN LOOP MODE VALUES
    //
    // allocate memory for DM modes history
    int    NB_DMtstep = 10; // history buffer size
    int    DMtstep    = 0;  // current index
    float *mvalDMbuff = (float *) malloc(sizeof(float) * NBmode * NB_DMtstep);

    IMGID imgOLmval;
    {
        char OLmvalname[STRINGMAXLEN_STREAMNAME];
        WRITE_IMAGENAME(OLmvalname, "aol%lu_mval_ol", *AOloopindex);
        imgOLmval = stream_connect_create_2Df32(OLmvalname, NBmode, 1);
    }



    // TELEMETRY BUFFERS
    //
    uint32_t TBsize  = 1;
    uint32_t TBindex = 0;
    uint32_t tbuffsize[(*tbuff_l2maxsize)];
    uint32_t tbuffzcnt[(*tbuff_l2maxsize)];
    IMGID    imgtbuff_mvalDM[(*tbuff_l2maxsize)];
    IMGID    imgtbuff_mvalWFS[(*tbuff_l2maxsize)];
    IMGID    imgtbuff_mvalOL[(*tbuff_l2maxsize)];
    for (uint32_t i = 0; i < (*tbuff_l2maxsize); i++)
    {
        tbuffzcnt[i] = 0;
        TBsize *= 2;
        tbuffsize[i] = TBsize;
        if (i >= (*tbuff_l2minsize))
        {
            char name[STRINGMAXLEN_STREAMNAME];

            WRITE_IMAGENAME(name, "aol%lu_mvalDM_buff%d", *AOloopindex, i);
            imgtbuff_mvalDM[i] =
                stream_connect_create_2Df32(name, tbuffsize[i], NBmode);

            WRITE_IMAGENAME(name, "aol%lu_mvalWFS_buff%d", *AOloopindex, i);
            imgtbuff_mvalWFS[i] =
                stream_connect_create_2Df32(name, tbuffsize[i], NBmode);

            WRITE_IMAGENAME(name, "aol%lu_mvalOL_buff%d", *AOloopindex, i);
            imgtbuff_mvalOL[i] =
                stream_connect_create_2Df32(name, tbuffsize[i], NBmode);
        }
    }
    float *buffmvalDM  = (float *) malloc(sizeof(float) * TBsize * NBmode);
    float *buffmvalWFS = (float *) malloc(sizeof(float) * TBsize * NBmode);
    float *buffmvalOL  = (float *) malloc(sizeof(float) * TBsize * NBmode);



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

    {
        // Pre-allocations for modal loop
        double mvalWFS;
        double dmval;
        double mvalDM;
        float  limit;


        // Apply modal control filtering
        //
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {

            // grab input value from WFS
            mvalWFS = imgin.im->array.F[mi];

            // offset from mval to zp
            dmval = imgmzeropoint.im->array.F[mi] - mvalWFS;

            // GAIN
            dmval *= imgmgain.im->array.F[mi];

            // get current DM position
            mvalDM = imgout.im->array.F[mi];

            // apply scaled offset
            mvalDM += dmval;
            // MULT
            mvalDM *= imgmmult.im->array.F[mi];


            // LIMIT
            limit = imgmlimit.im->array.F[mi];
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


        // Compute pseudo open-loop mode coefficients
        //
        if ((*compOL) == 1)
        {
            // write to DM history
            //
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                mvalDMbuff[DMtstep * NBmode + mi] = mvalout[mi];
            }
            DMtstep++;
            if (DMtstep == NB_DMtstep)
            {
                DMtstep = 0;
            }

            int   latint  = (int) (*latencyfr);
            float latfrac = (*latencyfr) - latint;

            int DMtstep1 = DMtstep - latint;
            int DMtstep0 = DMtstep1 - 1;
            while (DMtstep1 < 0)
            {
                DMtstep1 += NB_DMtstep;
            }
            while (DMtstep0 < 0)
            {
                DMtstep0 += NB_DMtstep;
            }

            imgOLmval.md->write = 1;
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpmDMval = latfrac * mvalDMbuff[DMtstep0 * NBmode + mi];
                tmpmDMval +=
                    (1.0 - latfrac) * mvalDMbuff[DMtstep1 * NBmode + mi];

                float tmpmWFSval = imgin.im->array.F[mi];
                ;

                imgOLmval.im->array.F[mi] = tmpmWFSval + tmpmDMval;
            }
            processinfo_update_output_stream(processinfo, imgOLmval.ID);
        }


        // Update individual gain, mult and limit values
        // This is done AFTER computing mode values to minimize latency
        //
        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmgain.im->array.F[mi] =
                imgmgainfact.im->array.F[mi] * (*loopgain);
        }
        processinfo_update_output_stream(processinfo, imgmgain.ID);


        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmmult.im->array.F[mi] =
                imgmmultfact.im->array.F[mi] * (*loopmult);
        }
        processinfo_update_output_stream(processinfo, imgmmult.ID);


        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            imgmlimit.im->array.F[mi] =
                imgmlimitfact.im->array.F[mi] * (*looplimit);
        }
        processinfo_update_output_stream(processinfo, imgmlimit.ID);


        // Fill telemetry buffers
        //
        if ((*comptbuff) == 1)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                buffmvalDM[TBindex * NBmode + mi]  = mvalout[mi];
                buffmvalWFS[TBindex * NBmode + mi] = imgin.im->array.F[mi];
                buffmvalOL[TBindex * NBmode + mi]  = imgOLmval.im->array.F[mi];
            }
            for (uint32_t i = (*tbuff_l2minsize); i < (*tbuff_l2maxsize); i++)
            {
                tbuffzcnt[i]++;
                if (tbuffzcnt[i] == tbuffsize[i])
                {
                    tbuffzcnt[i] = 0;
                    void *tbuffptr;

                    tbuffptr = buffmvalDM;
                    tbuffptr += sizeof(float) * TBindex * NBmode;
                    imgtbuff_mvalDM[i].md->write = 1;
                    memcpy(imgtbuff_mvalDM[i].im->array.F,
                           tbuffptr,
                           sizeof(float) * tbuffsize[i] * NBmode);
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalDM[i].ID);

                    /*
                    tbuffptr = &buffmvalWFS;
                    tbuffptr += sizeof(float) * TBindex * NBmode;
                    imgtbuff_mvalWFS[i].md->write = 1;
                    memcpy(imgtbuff_mvalWFS[i].im->array.F,
                           tbuffptr,
                           tbuffsize[i] * NBmode);
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalWFS[i].ID);


                    tbuffptr = &buffmvalOL;
                    tbuffptr += sizeof(float) * TBindex * NBmode;
                    imgtbuff_mvalOL[i].md->write = 1;
                    memcpy(imgtbuff_mvalOL[i].im->array.F,
                           tbuffptr,
                           tbuffsize[i] * NBmode);
                    processinfo_update_output_stream(processinfo,
                                                     imgtbuff_mvalOL[i].ID);
                                                     */
                }
                TBindex = tbuffzcnt[i];
            }
        }
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    free(mvalout);
    free(mvalDMbuff);

    free(buffmvalDM);
    free(buffmvalWFS);
    free(buffmvalOL);

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
