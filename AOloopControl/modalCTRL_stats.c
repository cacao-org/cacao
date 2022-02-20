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

    uint32_t mblksizemax = 100;


    IMGID imgtbuff_mvalDM;
    IMGID imgtbuff_mvalWFS;
    IMGID imgtbuff_mvalOL;
    IMGID imgblock;
    // Connect to telemetry buffers
    //
    uint32_t NBmode   = 0;
    uint32_t NBsample = 0;
    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name, "aol%lu_mvalDM_buff", *AOloopindex);
        read_sharedmem_image(name);
        imgtbuff_mvalDM = mkIMGID_from_name(name);
        resolveIMGID(&imgtbuff_mvalDM, ERRMODE_ABORT);

        WRITE_IMAGENAME(name, "aol%lu_mvalWFS_buff", *AOloopindex);
        read_sharedmem_image(name);
        imgtbuff_mvalWFS = mkIMGID_from_name(name);
        resolveIMGID(&imgtbuff_mvalWFS, ERRMODE_ABORT);

        WRITE_IMAGENAME(name, "aol%lu_mvalOL_buff", *AOloopindex);
        read_sharedmem_image(name);
        imgtbuff_mvalOL = mkIMGID_from_name(name);
        resolveIMGID(&imgtbuff_mvalOL, ERRMODE_ABORT);

        NBmode   = imgtbuff_mvalOL.md->size[0];
        NBsample = imgtbuff_mvalOL.md->size[1];
        WRITE_IMAGENAME(name, "aol%lu_mblk", *AOloopindex);
        imgblock = stream_connect_create_2D(name, NBmode, 1, _DATATYPE_INT32);
    }


    list_image_ID();


    // allocate arrays
    double *mvalDM_ave = (double *) malloc(sizeof(double) * NBmode);
    double *mvalDM_rms = (double *) malloc(sizeof(double) * NBmode);

    double *mvalWFS_ave = (double *) malloc(sizeof(double) * NBmode);
    double *mvalWFS_rms = (double *) malloc(sizeof(double) * NBmode);

    double *mvalOL_ave = (double *) malloc(sizeof(double) * NBmode);
    double *mvalOL_rms = (double *) malloc(sizeof(double) * NBmode);


    double *block_DMrms  = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_WFSrms = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_OLrms  = (double *) malloc(sizeof(double) * mblksizemax);
    long   *block_cnt    = (long *) malloc(sizeof(long) * mblksizemax);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START



    printf("%5ld ==== %u modes ==== %u samples ==========\n",
           processinfo->loopcnt,
           NBmode,
           NBsample);
    {
        int slice;

        slice = imgtbuff_mvalDM.md->cnt1;
        printf("    DM  buffer slice = %d\n", slice);

        slice = imgtbuff_mvalWFS.md->cnt1;
        printf("    WFS buffer slice = %d\n", slice);

        slice = imgtbuff_mvalOL.md->cnt1;
        printf("    OL  buffer slice = %d\n", slice);

        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            mvalDM_ave[mi] = 0.0;
            mvalDM_rms[mi] = 0.0;

            mvalWFS_ave[mi] = 0.0;
            mvalWFS_rms[mi] = 0.0;

            mvalOL_ave[mi] = 0.0;
            mvalOL_rms[mi] = 0.0;
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalDM.im->array.F[sample * NBmode + mi];
                mvalDM_ave[mi] += tmpv;
                mvalDM_rms[mi] += tmpv * tmpv;
            }
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalWFS.im->array.F[sample * NBmode + mi];
                mvalWFS_ave[mi] += tmpv;
                mvalWFS_rms[mi] += tmpv * tmpv;
            }
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalOL.im->array.F[sample * NBmode + mi];
                mvalOL_ave[mi] += tmpv;
                mvalOL_rms[mi] += tmpv * tmpv;
            }
        }


        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            mvalDM_rms[mi] -= mvalDM_ave[mi] * mvalDM_ave[mi];
            mvalDM_rms[mi] = sqrt(mvalDM_rms[mi] / NBsample);
            mvalDM_ave[mi] /= NBsample;

            mvalWFS_rms[mi] -= mvalWFS_ave[mi] * mvalWFS_ave[mi];
            mvalWFS_rms[mi] = sqrt(mvalWFS_rms[mi] / NBsample);
            mvalWFS_ave[mi] /= NBsample;

            mvalOL_rms[mi] -= mvalOL_ave[mi] * mvalOL_ave[mi];
            mvalOL_rms[mi] = sqrt(mvalOL_rms[mi] / NBsample);
            mvalOL_ave[mi] /= NBsample;
        }

        for (uint32_t block = 0; block < mblksizemax; block++)
        {
            block_cnt[block]    = 0;
            block_DMrms[block]  = 0.0;
            block_WFSrms[block] = 0.0;
            block_OLrms[block]  = 0.0;
        }

        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            int32_t block = imgblock.im->array.SI32[mi];

            block_cnt[block]++;
            block_DMrms[block] += mvalDM_rms[mi] * mvalDM_rms[mi];
            block_WFSrms[block] += mvalWFS_rms[mi] * mvalWFS_rms[mi];
            block_OLrms[block] += mvalOL_rms[mi] * mvalOL_rms[mi];
        }


        for (uint32_t block = 0; block < mblksizemax; block++)
        {
            if (block_cnt[block] > 0)
            {
                block_DMrms[block] =
                    sqrt(block_DMrms[block] / block_cnt[block]);
                block_WFSrms[block] =
                    sqrt(block_WFSrms[block] / block_cnt[block]);
                block_OLrms[block] =
                    sqrt(block_OLrms[block] / block_cnt[block]);
                printf(
                    "BLOCK %2d (%5ld modes)   WFS = %7.3f   DM = %7.3f   OL = "
                    "%7.3f\n",
                    block,
                    block_cnt[block],
                    block_WFSrms[block],
                    block_DMrms[block],
                    block_OLrms[block]);
            }
        }
    }




    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    free(block_DMrms);
    free(block_WFSrms);
    free(block_OLrms);
    free(block_cnt);

    free(mvalDM_ave);
    free(mvalDM_rms);

    free(mvalWFS_ave);
    free(mvalWFS_rms);

    free(mvalOL_ave);
    free(mvalOL_rms);

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
