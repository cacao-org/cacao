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
    double *mvalDM_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalDM_rms2 = (double *) malloc(sizeof(double) * NBmode);

    double *mvalWFS_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalWFS_rms2 = (double *) malloc(sizeof(double) * NBmode);

    double *mvalOL_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalOL_rms2 = (double *) malloc(sizeof(double) * NBmode);


    double *block_DMrms2  = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_WFSrms2 = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_OLrms2  = (double *) malloc(sizeof(double) * mblksizemax);
    long   *block_cnt     = (long *) malloc(sizeof(long) * mblksizemax);

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
            mvalDM_ave[mi]  = 0.0;
            mvalDM_rms2[mi] = 0.0;

            mvalWFS_ave[mi]  = 0.0;
            mvalWFS_rms2[mi] = 0.0;

            mvalOL_ave[mi]  = 0.0;
            mvalOL_rms2[mi] = 0.0;
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalDM.im->array.F[sample * NBmode + mi];
                mvalDM_ave[mi] += tmpv;
                mvalDM_rms2[mi] += tmpv * tmpv;
            }
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalWFS.im->array.F[sample * NBmode + mi];
                mvalWFS_ave[mi] += tmpv;
                mvalWFS_rms2[mi] += tmpv * tmpv;
            }
        }

        for (uint32_t sample = 0; sample < NBsample; sample++)
        {
            for (uint32_t mi = 0; mi < NBmode; mi++)
            {
                float tmpv = imgtbuff_mvalOL.im->array.F[sample * NBmode + mi];
                mvalOL_ave[mi] += tmpv;
                mvalOL_rms2[mi] += tmpv * tmpv;
            }
        }



        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            if (mi < 5)
            {
                printf("mode %2u  WFS = %7.3f   DM = %7.3f   OL = %7.3f\n",
                       mi,
                       mvalWFS_rms2[mi],
                       mvalDM_rms2[mi],
                       mvalOL_rms2[mi]);
            }

            mvalDM_rms2[mi] -= mvalDM_ave[mi] * mvalDM_ave[mi];
            mvalDM_rms2[mi] /= NBsample;
            mvalDM_ave[mi] /= NBsample;

            mvalWFS_rms2[mi] -= mvalWFS_ave[mi] * mvalWFS_ave[mi];
            mvalWFS_rms2[mi] /= NBsample;
            mvalWFS_ave[mi] /= NBsample;

            mvalOL_rms2[mi] -= mvalOL_ave[mi] * mvalOL_ave[mi];
            mvalOL_rms2[mi] /= NBsample;
            mvalOL_ave[mi] /= NBsample;

            if (mi < 5)
            {
                printf("mode %2u  WFS = %7.3f   DM = %7.3f   OL = %7.3f\n",
                       mi,
                       mvalWFS_rms2[mi],
                       mvalDM_rms2[mi],
                       mvalOL_rms2[mi]);
            }
        }

        for (uint32_t block = 0; block < mblksizemax; block++)
        {
            block_cnt[block]     = 0;
            block_DMrms2[block]  = 0.0;
            block_WFSrms2[block] = 0.0;
            block_OLrms2[block]  = 0.0;
        }

        for (uint32_t mi = 0; mi < NBmode; mi++)
        {
            int32_t block = imgblock.im->array.SI32[mi];

            block_cnt[block]++;
            block_DMrms2[block] += mvalDM_rms2[mi];
            block_WFSrms2[block] += mvalWFS_rms2[mi];
            block_OLrms2[block] += mvalOL_rms2[mi];
        }


        for (uint32_t block = 0; block < mblksizemax; block++)
        {
            if (block_cnt[block] > 0)
            {
                block_DMrms2[block] /= block_cnt[block];
                block_WFSrms2[block] /= block_cnt[block];
                block_OLrms2[block] /= block_cnt[block];
                printf(
                    "BLOCK %2d (%5ld modes)   WFS = %7.3f   DM = %7.3f   OL ="
                    "%7.3f\n",
                    block,
                    block_cnt[block],
                    block_WFSrms2[block],
                    block_DMrms2[block],
                    block_OLrms2[block]);
            }
        }
    }




    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    free(block_DMrms2);
    free(block_WFSrms2);
    free(block_OLrms2);
    free(block_cnt);

    free(mvalDM_ave);
    free(mvalDM_rms2);

    free(mvalWFS_ave);
    free(mvalWFS_rms2);

    free(mvalOL_ave);
    free(mvalOL_rms2);


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
