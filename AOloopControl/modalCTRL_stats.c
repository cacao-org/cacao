/**
 * @file    modaloptimize.c
 * @brief   Optimize modal control parameters
 *
 *
 *
 */
#include "ImageStreamIO/ImageStruct.h"

#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "modalCTRLstats",
    .cmdkey      = "modalCTRLstats",
    .description = "compute modal control stats"
};

// Local variables pointers
static uint64_t *AOloopindex;

// blocks sizes
static uint32_t *block0NBmode;
static uint32_t *block0NBsample;

static uint32_t *block1NBmode;
static uint32_t *block1NBsample;

static uint32_t *block2NBmode;
static uint32_t *block2NBsample;

static uint32_t *block3NBmode;
static uint32_t *block3NBsample;

static uint32_t *block4NBmode;
static uint32_t *block4NBsample;

static uint32_t *block5NBmode;
static uint32_t *block5NBsample;

static uint32_t *block6NBmode;
static uint32_t *block6NBsample;

static uint32_t *block7NBmode;
static uint32_t *block7NBsample;

static uint32_t *block8NBmode;
static uint32_t *block8NBsample;

static uint32_t *block9NBmode;
static uint32_t *block9NBsample;

static uint64_t *compstatswrite;

#define FPS_PARAMS(X) \
    X(".AOloopindex", &AOloopindex, FPTYPE_UINT64, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "AO loop index") \
    X(".block.blk0NBmode", &block0NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 0 number of modes") \
    X(".block.blk0NBsample", &block0NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 0 number of samples") \
    X(".block.blk1NBmode", &block1NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 1 number of modes") \
    X(".block.blk1NBsample", &block1NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 1 number of samples") \
    X(".block.blk2NBmode", &block2NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 2 number of modes") \
    X(".block.blk2NBsample", &block2NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 2 number of samples") \
    X(".block.blk3NBmode", &block3NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 3 number of modes") \
    X(".block.blk3NBsample", &block3NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 3 number of samples") \
    X(".block.blk4NBmode", &block4NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 4 number of modes") \
    X(".block.blk4NBsample", &block4NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 4 number of samples") \
    X(".block.blk5NBmode", &block5NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 5 number of modes") \
    X(".block.blk5NBsample", &block5NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 5 number of samples") \
    X(".block.blk6NBmode", &block6NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 6 number of modes") \
    X(".block.blk6NBsample", &block6NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 6 number of samples") \
    X(".block.blk7NBmode", &block7NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 7 number of modes") \
    X(".block.blk7NBsample", &block7NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 7 number of samples") \
    X(".block.blk8NBmode", &block8NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 8 number of modes") \
    X(".block.blk8NBsample", &block8NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 8 number of samples") \
    X(".block.blk9NBmode", &block9NBmode, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 9 number of modes") \
    X(".block.blk9NBsample", &block9NBsample, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT, "block 9 number of samples") \
    X(".comp.statswrite", &compstatswrite, FPTYPE_ONOFF, 1, FPFLAG_DEFAULT_INPUT, "Write stats to file")

FPS_V2_SECTION5(FPS_PARAMS)


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.core.fpsptr != NULL)
    {
        long fpi;
        fpi = functionparameter_GetParamIndex(data.core.fpsptr, ".comp.statswrite");
        if(fpi > -1) data.core.fpsptr->parray[fpi].fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.core.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}


// detailed help
static errno_t __attribute__((unused)) help_function()
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

        WRITE_IMAGENAME(name, "aol%lu_modevalDM_buff", *AOloopindex);
        read_sharedmem_image(name, data.core.image, data.core.NB_MAX_IMAGE);
        imgtbuff_mvalDM = imgid_make_from_name(name);
        resolveIMGID(
            &imgtbuff_mvalDM, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgtbuff_mvalDM.ID == -1) return RETURN_FAILURE;

        WRITE_IMAGENAME(name, "aol%lu_modevalWFS_buff", *AOloopindex);
        read_sharedmem_image(name, data.core.image, data.core.NB_MAX_IMAGE);
        imgtbuff_mvalWFS = imgid_make_from_name(name);
        resolveIMGID(
            &imgtbuff_mvalWFS, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgtbuff_mvalWFS.ID == -1) return RETURN_FAILURE;

        WRITE_IMAGENAME(name, "aol%lu_modevalOL_buff", *AOloopindex);
        read_sharedmem_image(name, data.core.image, data.core.NB_MAX_IMAGE);
        imgtbuff_mvalOL = imgid_make_from_name(name);
        resolveIMGID(
            &imgtbuff_mvalOL, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgtbuff_mvalOL.ID == -1) return RETURN_FAILURE;

        NBmode   = imgtbuff_mvalOL.md->size[0];
        NBsample = imgtbuff_mvalOL.md->size[1];
        WRITE_IMAGENAME(name, "aol%lu_mblk", *AOloopindex);
        imgblock = stream_connect_create_2D(name, NBmode, 1, _DATATYPE_INT32);
    }


    list_image_ID();


    // how many blocks ?
    uint32_t NBblk  = 0;
    int      MAXBLK = 10;
    uint32_t blksize[MAXBLK];
    uint32_t blkoffset[MAXBLK];
    blkoffset[0] = 0;
    blksize[0]   = *block0NBmode;
    blksize[1]   = *block1NBmode;
    blksize[2]   = *block2NBmode;
    blksize[3]   = *block3NBmode;
    blksize[4]   = *block4NBmode;
    blksize[5]   = *block5NBmode;
    blksize[6]   = *block6NBmode;
    blksize[7]   = *block7NBmode;
    blksize[8]   = *block8NBmode;
    blksize[9]   = *block9NBmode;

    int32_t blksamplesize[MAXBLK];
    blksamplesize[0] = *block0NBsample;
    blksamplesize[1] = *block1NBsample;
    blksamplesize[2] = *block2NBsample;
    blksamplesize[3] = *block3NBsample;
    blksamplesize[4] = *block4NBsample;
    blksamplesize[5] = *block5NBsample;
    blksamplesize[6] = *block6NBsample;
    blksamplesize[7] = *block7NBsample;
    blksamplesize[8] = *block8NBsample;
    blksamplesize[9] = *block9NBsample;


    uint32_t blki             = 0;
    int32_t  NBmode_available = NBmode;
    while((NBmode_available > 0) && (NBblk < (uint32_t) MAXBLK))
    {
        NBmode_available -= blksize[blki];
        printf("%u  available : %d\n", blki, NBmode_available);
        fflush(stdout);
        if(NBmode_available < 0)
        {
            blksize[blki] += NBmode_available;
        }
        printf(" --> size   : %u\n", blksize[blki]);
        fflush(stdout);
        blki++;
        blkoffset[blki] = blkoffset[blki - 1] + blksize[blki - 1];
        printf(" --> offset : %u\n", blkoffset[blki]);
        fflush(stdout);
        NBblk++;
    }
    for(uint32_t blki1 = blki; blki1 < (uint32_t) MAXBLK; blki1++)
    {
        blksize[blki1]   = 0;
        blkoffset[blki1] = blkoffset[blki1 - 1];
    }

    for(uint32_t blki = 0; blki < NBblk; blki++)
    {
        printf("BLOCK %u  size %4u  range: %4u - %4u\n",
               blki,
               blksize[blki],
               blkoffset[blki],
               blkoffset[blki] + blksize[blki] - 1);
    }
    *block0NBmode = blksize[0];
    *block1NBmode = blksize[1];
    *block2NBmode = blksize[2];
    *block3NBmode = blksize[3];
    *block4NBmode = blksize[4];
    *block5NBmode = blksize[5];
    *block6NBmode = blksize[6];
    *block7NBmode = blksize[7];
    *block8NBmode = blksize[8];
    *block9NBmode = blksize[9];


    // block buffers
    //
    IMGID imgmvalOLbuffblk[NBblk];
    for(uint32_t blki = 0; blki < NBblk; blki++)
    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name,
                        "aol%lu_modevalOLbuff_blk%02u",
                        *AOloopindex,
                        blki);
        imgmvalOLbuffblk[blki] =
            stream_connect_create_3Df32(name,
                                        blksize[blki],
                                        1,
                                        blksamplesize[blki]);
    }
    // local memory for block buffers
    //
    int32_t blksampleindex[NBblk];
    float  *mvalOLbuffarray[NBblk];
    for(uint32_t blki = 0; blki < NBblk; blki++)
    {
        blksampleindex[blki]  = 0;
        mvalOLbuffarray[blki] = (float *) malloc(sizeof(float) * blksize[blki] *
                                blksamplesize[blki]);
    }


    list_image_ID();


    // allocate arrays
    double *mvalDM_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalDM_rms2 = (double *) malloc(sizeof(double) * NBmode);

    double *mvalWFS_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalWFS_rms2 = (double *) malloc(sizeof(double) * NBmode);

    double *mvalOL_ave  = (double *) malloc(sizeof(double) * NBmode);
    double *mvalOL_rms2 = (double *) malloc(sizeof(double) * NBmode);

    // WFS meassurement noise
    // using 3 points (linear)
    double *mvalWFS_mrms2 = (double *) malloc(sizeof(double) * NBmode);
    // using 4 points (quadratic)
    double *mvalWFS_mqrms2 = (double *) malloc(sizeof(double) * NBmode);


    double *block_DMrms2    = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_WFSrms2   = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_WFSmrms2  = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_WFSmqrms2 = (double *) malloc(sizeof(double) * mblksizemax);
    double *block_OLrms2    = (double *) malloc(sizeof(double) * mblksizemax);
    long   *block_cnt       = (long *) malloc(sizeof(long) * mblksizemax);


    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // block masks
    //
    IMGID imgblkmask[NBblk];
    for(uint32_t blki = 0; blki < NBblk; blki++)
    {
        char name[STRINGMAXLEN_STREAMNAME];

        WRITE_IMAGENAME(name, "aol%lu_blkmask%02u", *AOloopindex, blki);
        imgblkmask[blki] =
            stream_connect_create_2D(name, NBmode, 1, _DATATYPE_INT8);
        for(uint32_t mi = blkoffset[blki];
                mi < blkoffset[blki] + blksize[blki];
                mi++)
        {
            imgblkmask[blki].im->array.SI8[mi] = 1;
        }
        processinfo_update_output_stream(processinfo,
            imgblkmask[blki].im,
            NULL);
    }


    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {

        printf("Stepping ...\n");


        // fill up block telemetry streams
        //
        {
            int slice = imgtbuff_mvalOL.md->cnt1;
            for(uint32_t sample = 0; sample < NBsample; sample++)
            {
                for(uint32_t blki = 0; blki < NBblk; blki++)
                {
                    for(uint32_t mirel = 0; mirel < blksize[blki]; mirel++)
                    {
                        uint32_t mi = mirel + blkoffset[blki];

                        mvalOLbuffarray[blki][blksize[blki] * blksampleindex[blki] +
                                              mirel] =
                        imgtbuff_mvalOL.im->array.F[slice * NBsample * NBmode +
                                                          sample * NBmode + mi];
                    }
                    blksampleindex[blki]++;
                    if(blksampleindex[blki] == blksamplesize[blki])
                    {
                        memcpy(imgmvalOLbuffblk[blki].im->array.F,
                               mvalOLbuffarray[blki],
                               sizeof(float) * blksize[blki] * blksamplesize[blki]);
                        processinfo_update_output_stream(processinfo,
                                                         imgmvalOLbuffblk[blki].im, NULL);
                        blksampleindex[blki] = 0;
                    }
                }
            }
        }

        {
            int slice;

            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                mvalDM_ave[mi]  = 0.0;
                mvalDM_rms2[mi] = 0.0;

                mvalWFS_ave[mi]  = 0.0;
                mvalWFS_rms2[mi] = 0.0;

                mvalOL_ave[mi]  = 0.0;
                mvalOL_rms2[mi] = 0.0;

                mvalWFS_mrms2[mi]  = 0.0;
                mvalWFS_mqrms2[mi] = 0.0;
            }

            slice = imgtbuff_mvalDM.md->cnt1;
            for(uint32_t sample = 0; sample < NBsample; sample++)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpv =
                    imgtbuff_mvalDM.im->array
                    .F[slice * NBsample * NBmode + sample * NBmode + mi];
                    mvalDM_ave[mi] += tmpv;
                    mvalDM_rms2[mi] += tmpv * tmpv;
                }
            }

            slice = imgtbuff_mvalWFS.md->cnt1;
            for(uint32_t sample = 0; sample < NBsample; sample++)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpv =
                        imgtbuff_mvalWFS.im->array
                        .F[slice * NBsample * NBmode + sample * NBmode + mi];
                    mvalWFS_ave[mi] += tmpv;
                    mvalWFS_rms2[mi] += tmpv * tmpv;
                }
            }

            // linear noise derivation
            //
            for(uint32_t sample = 1; sample < NBsample - 1; sample++)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpv0 =
                        imgtbuff_mvalWFS.im->array.F[slice * NBsample * NBmode +
                                                           (sample - 1) * NBmode + mi];
                    float tmpv1 =
                        imgtbuff_mvalWFS.im->array
                        .F[slice * NBsample * NBmode + (sample) * NBmode + mi];
                    float tmpv2 =
                        imgtbuff_mvalWFS.im->array.F[slice * NBsample * NBmode +
                                                           (sample + 1) * NBmode + mi];

                    float tmpv = 0.5 * (tmpv0 + tmpv2) - tmpv1;
                    mvalWFS_mrms2[mi] += tmpv * tmpv;
                }
            }

            // linear noise derivation
            //
            for(uint32_t sample = 1; sample < NBsample - 2; sample++)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpv0 =
                        imgtbuff_mvalWFS.im->array.F[slice * NBsample * NBmode +
                                                           (sample - 1) * NBmode + mi];
                    float tmpv1 =
                        imgtbuff_mvalWFS.im->array
                        .F[slice * NBsample * NBmode + (sample) * NBmode + mi];
                    float tmpv2 =
                        imgtbuff_mvalWFS.im->array.F[slice * NBsample * NBmode +
                                                           (sample + 1) * NBmode + mi];
                    float tmpv3 =
                        imgtbuff_mvalWFS.im->array.F[slice * NBsample * NBmode +
                                                           (sample + 1) * NBmode + mi];

                    float tmpv =
                        -0.5 * tmpv0 + 1.5 * tmpv1 - 1.5 * tmpv2 + 0.5 * tmpv3;
                    mvalWFS_mqrms2[mi] += tmpv * tmpv;
                }
            }


            slice = imgtbuff_mvalOL.md->cnt1;
            for(uint32_t sample = 0; sample < NBsample; sample++)
            {
                for(uint32_t mi = 0; mi < NBmode; mi++)
                {
                    float tmpv =
                        imgtbuff_mvalOL.im->array
                        .F[slice * NBsample * NBmode + sample * NBmode + mi];
                    mvalOL_ave[mi] += tmpv;
                    mvalOL_rms2[mi] += tmpv * tmpv;
                }
            }


            for(uint32_t mi = 0; mi < NBmode; mi++)
            {

                mvalDM_ave[mi] /= NBsample;
                mvalWFS_ave[mi] /= NBsample;
                mvalOL_ave[mi] /= NBsample;

                mvalDM_rms2[mi] /= NBsample;
                mvalWFS_rms2[mi] /= NBsample;
                mvalOL_rms2[mi] /= NBsample;

                // factor 1.5 excess variance is from linear comb of 3 values with coeffs 0.5, 1, 0.5
                // variance = 0.25 + 1 + 0.25 = 1.5
                mvalWFS_mrms2[mi] /= (NBsample - 2) * 1.5;
                mvalWFS_mqrms2[mi] /= (NBsample - 3) * 5;


                mvalDM_rms2[mi] -= (mvalDM_ave[mi] * mvalDM_ave[mi]);

                mvalWFS_rms2[mi] -= (mvalWFS_ave[mi] * mvalWFS_ave[mi]);

                mvalOL_rms2[mi] -= (mvalOL_ave[mi] * mvalOL_ave[mi]);
            }


            for(uint32_t block = 0; block < mblksizemax; block++)
            {
                block_cnt[block]       = 0;
                block_DMrms2[block]    = 0.0;
                block_WFSrms2[block]   = 0.0;
                block_WFSmrms2[block]  = 0.0;
                block_WFSmqrms2[block] = 0.0;
                block_OLrms2[block]    = 0.0;
            }

            for(uint32_t mi = 0; mi < NBmode; mi++)
            {
                // remove noise
                mvalWFS_rms2[mi] -= mvalWFS_mqrms2[mi];
                mvalOL_rms2[mi] -= mvalWFS_mqrms2[mi];

                // add to corresponding block
                int32_t block = imgblock.im->array.SI32[mi];
                block_cnt[block]++;
                block_DMrms2[block] += mvalDM_rms2[mi];
                block_WFSrms2[block] += mvalWFS_rms2[mi];
                block_WFSmrms2[block] += mvalWFS_mrms2[mi];
                block_WFSmqrms2[block] += mvalWFS_mqrms2[mi];
                block_OLrms2[block] += mvalOL_rms2[mi];
            }


            if(*compstatswrite == 1)
            {
                for(uint32_t block = 0; block < mblksizemax; block++)
                {
                    if(block_cnt[block] > 0)
                    {
                        //block_DMrms2[block] /= block_cnt[block];
                        //block_WFSrms2[block] /= block_cnt[block];
                        //block_OLrms2[block] /= block_cnt[block];
                        printf(
                            "BLOCK %2d (%5ld modes) RMS  WFS = %7.3f (noise = "
                            "%7.3f "
                            "%7.3f)  DM = "
                            "%7.3f   "
                            "OL = "
                            "%7.3f   [nm]  --> %5.3f\n",
                            block,
                            block_cnt[block],
                            1000.0 * sqrtf(block_WFSrms2[block]),
                            1000.0 * sqrtf(block_WFSmrms2[block]),
                            1000.0 * sqrtf(block_WFSmqrms2[block]),
                            1000.0 * sqrtf(block_DMrms2[block]),
                            1000.0 * sqrtf(block_OLrms2[block]),
                            sqrtf(block_WFSrms2[block]) / sqrtf(block_OLrms2[block]));

                        char ffname[STRINGMAXLEN_FULLFILENAME];
                        WRITE_FULLFILENAME(ffname, "AOmodalstat.dat");
                        FILE *fp = fopen(ffname, "a");
                        fprintf(fp,
                                "%5ld  %02d   %7.3f %7.3f %7.3f %7.3f  %5.3f\n",
                                processinfo->loopcnt,
                                block,
                                1000.0 * sqrtf(block_WFSrms2[block]),
                                1000.0 * sqrtf(block_WFSmqrms2[block]),
                                1000.0 * sqrtf(block_DMrms2[block]),
                                1000.0 * sqrtf(block_OLrms2[block]),
                                sqrtf(block_WFSrms2[block]) /
                                sqrtf(block_OLrms2[block]));
                        fclose(fp);
                    }
                }
            }
        }

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    for(uint32_t blki = 0; blki < NBblk; blki++)
    {
        free(mvalOLbuffarray[blki]);
    }


    free(block_DMrms2);
    free(block_WFSrms2);
    free(block_WFSmrms2);
    free(block_WFSmqrms2);
    free(block_OLrms2);
    free(block_cnt);

    free(mvalDM_ave);
    free(mvalDM_rms2);

    free(mvalWFS_ave);
    free(mvalWFS_rms2);
    free(mvalWFS_mrms2);
    free(mvalWFS_mqrms2);

    free(mvalOL_ave);
    free(mvalOL_rms2);

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl__modalCTRL_stats()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2_CONFCHECK(
    FPS_app_info,
    FPS_PARAMS,
    compute_function,
    customCONFsetup,
    customCONFcheck)
#endif
