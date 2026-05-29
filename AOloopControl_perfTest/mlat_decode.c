/**
 * @file mlat_decode.c
 * @brief Mlat decode module
 */

#include "ImageStreamIO/ImageStruct.h"
/**
 * @file mlat_decode.c
 *
 * Decode mlat diffseq to time series
 */

#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "linalgebra/SingularValueDecomp.h"
#include "linalgebra/SGEMM.h"

// input image names
static char diffseqname[FUNCTION_PARAMETER_STRMAXLEN] = "";

static char outseqname[FUNCTION_PARAMETER_STRMAXLEN] = "";


static uint32_t *oversamp;

static uint32_t *nb0start;

static uint32_t *nb0end;


static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "mlatdsdecode",
    .cmdkey      = "mlatdsdecode",
    .description = "mlat diff sequence decode",
    .description_long =
        "Decode a latency measurement sequence to extract the frame-by-frame delay between DM and WFS streams."
};

#define FPS_PARAMS(X) \
    X(".diffseqname", diffseqname, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "input difference sequence cube") \
    X(".outseq",      outseqname,  FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "output time seq cube") \
    X(".oversamp",    &oversamp,    FPTYPE_UINT32,     1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "samples per frame exposure time") \
    X(".nb0start",    &nb0start,    FPTYPE_UINT32,     1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "samples set to zero at start") \
    X(".nb0end",      &nb0end,      FPTYPE_UINT32,     1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "samples set to zero at end")

errno_t mlat_diffseq_decode(
    IMGID inimg,
    IMGID *outimg,
    uint32_t samplingfactor,
    uint32_t framezero_start,
    uint32_t framezero_end
)
{
    DEBUG_TRACE_FSTART();

    resolveIMGID(
        &inimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (inimg.ID == -1) return RETURN_FAILURE;


    // m: number of samples in diffseq
    long xsize = inimg.md->size[0];
    long ysize = inimg.md->size[1];
    long zsize = inimg.md->size[2];
    long xysize = xsize * ysize;

    // output
    imgid_copy(&inimg, outimg);
    createimagefromIMGID(outimg);


    // reconstructed input
    IMGID imgrec  = imgid_make_from_name("imrec");
    imgid_copy(&inimg, &imgrec);
    createimagefromIMGID(&imgrec);


    // initialize imgrec to inimg
    for(int ii = 0; ii < xysize * zsize; ii++)
    {
        imgrec.im->array.F[ii] = inimg.im->array.F[ii];
    }


    // build timing kernel
    // time goes back with index
    float *tkern = (float *) malloc(sizeof(float) * 2 * samplingfactor);
    for(int tstep = 0; tstep < samplingfactor; tstep++)
    {
        tkern[tstep] = 1.0;
        tkern[tstep + samplingfactor] = -1.0;
    }

    float *imv = (float *) malloc(sizeof(float) * xysize);


    int NBloopiter = 1;
    double loopgain1 = 0.99; // enforce consistency with input

    double loopgaintconv __attribute__((unused)) = 0.02;
    double loopgainmult = 0.99;


    for(int loopiter = 0; loopiter < NBloopiter; loopiter++)
    {
        printf("LOOP iteration %4d  ", loopiter);
        for(int kk = 0; kk < zsize; kk++)
        {
            for(int ii = 0; ii < xysize; ii++)
            {
                outimg->im->array.F[kk * xysize + ii] = imgrec.im->array.F[kk * xysize + ii];
            }

            for(int ii = 0; ii < xysize; ii++)
            {
                imv[ii] = 0.0;
            }
            for(int kstep = 1; kstep < 2 * samplingfactor; kstep++)
            {
                int k1 = kk - kstep;
                if(k1 >= 0)
                {
                    for(int ii = 0; ii < xysize; ii++)
                    {
                        imv[ii] += tkern[kstep] * outimg->im->array.F[k1 * xysize + ii];
                    }
                }
            }
            for(int ii = 0; ii < xysize; ii++)
            {
                outimg->im->array.F[kk * xysize + ii] -= imv[ii];
            }
        }


        // Apply temporal smoothing filter
        if(1)
        {
            //printf("Temporal convolution ");
            float *varray = (float *) malloc(sizeof(float) * zsize);

            for(int ii = 0; ii < xysize; ii++)
            {
                for(int kk = 0; kk < zsize; kk++)
                {
                    varray[kk] = outimg->im->array.F[kk * xysize + ii];
                }


                /*outimg->im->array.F[0*xysize + ii] = (1.0-loopgaintconv)*varray[0] + loopgaintconv*varray[1];
                for ( int kk=1; kk<zsize-1; kk++ )
                {
                    outimg->im->array.F[kk*xysize + ii] = loopgaintconv*varray[kk-1] + (1.0-2.0*loopgaintconv)*varray[kk] + loopgaintconv*varray[kk+1];
                }
                outimg->im->array.F[(zsize-1)*xysize + ii] = (1.0-loopgaintconv)*varray[zsize-1] + loopgaintconv*varray[zsize-2];*/


                for(int kk = 0; kk < zsize; kk++)
                {
                    outimg->im->array.F[kk * xysize + ii] *= loopgainmult;
                }
            }

            free(varray);
        }


        // Reconstruct input
        for(int kk = 0; kk < zsize; kk++)
        {
            for(int ii = 0; ii < xysize; ii++)
            {
                imgrec.im->array.F[kk * xysize + ii] = 0.0;
            }
            for(int kstep = 0; kstep < 2 * samplingfactor; kstep++)
            {
                int k1 = kk - kstep;
                if(k1 >= 0)
                {
                    for(int ii = 0; ii < xysize; ii++)
                    {
                        imgrec.im->array.F[kk * xysize + ii] += tkern[kstep] * outimg->im->array.F[k1 *
                                                                xysize + ii];
                    }
                }
            }
        }

        if(1)
        {
            double resval = 0.0;
            double resval0 = 0.0;
            for(int kk = 0; kk < framezero_end; kk++)
            {
                for(int ii = 0; ii < xysize; ii++)
                {
                    double dv = inimg.im->array.F[kk * xysize + ii] - imgrec.im->array.F[kk * xysize
                                + ii];
                    double dv0 = inimg.im->array.F[kk * xysize + ii];

                    resval += dv * dv;
                    resval0 += dv0 * dv0;

                    imgrec.im->array.F[kk * xysize + ii] += loopgain1 * dv;
                }
            }

            for(int kk = framezero_end; kk < zsize; kk++)
            {
                for(int ii = 0; ii < xysize; ii++)
                {
                    imgrec.im->array.F[kk * xysize + ii]  *= 0.9;
                }
            }
            printf("  %12.9f", resval / resval0);
        }


        printf("\n");
    }


    free(imv);
    free(tkern);


    if(0)
    {
        long m = inimg.md->size[2];
        // n: number of samples in reconstructed seq
        long n = (m - 1) + 2 * samplingfactor; // - framezero_start - framezero_end;


        printf("m = %ld\n", m);
        printf("n = %ld (%u %u)\n", n, framezero_start, framezero_end);


        // Construct timing matrix
        IMGID imgtmat;
        imgtmat = imgid_make_from_name_2D("mlattimingmat",
                               n,
                               m);
        createimagefromIMGID(&imgtmat);


        for(uint32_t ii = 0; ii < m; ii++)
        {
            for(uint32_t jj = 0; jj < n; jj++)
            {
                imgtmat.im->array.F[ii * n + jj] = 0.0;
            }

            int jpos = ii; // - framezero_start;
            int jpos1 = 0;

            // negative
            for(int j = 0; j < samplingfactor; j++)
            {
                if(jpos < 0)
                {
                    jpos1 = 0;
                }
                else if(jpos > (n - 1))
                {
                    jpos1 = n - 1;
                }
                else
                {
                    jpos1 = jpos;
                }
                if(jpos >= 0)
                {
                    imgtmat.im->array.F[ii * n + jpos1] -= 1.0;
                }
                jpos ++;
            }

            // positive
            for(int j = 0; j < samplingfactor; j++)
            {
                if(jpos < 0)
                {
                    jpos1 = 0;
                }
                else if(jpos > (n - 1))
                {
                    jpos1 = n - 1;
                }
                else
                {
                    jpos1 = jpos;
                }
                imgtmat.im->array.F[ii * n + jpos1] += 1.0;
                jpos ++;
            }
        }


        int GPUdev = 0;
        uint32_t Vdim0 = 0;
        float svdlim = 0.0001;
        int maxNBmode = 1000;

        IMGID imgU  = imgid_make_from_name("outU");
        IMGID imgS  = imgid_make_from_name("outS");
        IMGID imgV  = imgid_make_from_name("outV");
        compute_SVD(imgtmat, &imgU, &imgS, &imgV, Vdim0, svdlim, maxNBmode, GPUdev, 6, "SVDunmodes", "SVDvnmodes");

        IMGID imgpsinv = imgid_make_from_name("psinv");
        resolveIMGID(
            &imgpsinv, ERRMODE_WARN,
            data.core.image,
            data.core.NB_MAX_IMAGE);
            if (imgpsinv.ID == -1) return RETURN_FAILURE;


        double loopgain = 0.1;

        IMGID imgrec_alt  = imgid_make_from_name("recinput");

        IMGID imgres  = imgid_make_from_name("loopres");
        imgid_copy(&inimg, &imgres);
        createimagefromIMGID(&imgres);

        IMGID imgoutres  = imgid_make_from_name("loopoutres");


        long NBloopiter = 100;


        computeSGEMM(inimg, imgpsinv, outimg, 0, 0, GPUdev);


        for(int loopiter = 0; loopiter < NBloopiter; loopiter++)
        {

            {
                // set reference to fist framezero_start slices reference
                int xsize = outimg->md->size[0];
                int ysize = outimg->md->size[1];
                int zsize = outimg->md->size[2];
                float *refarray = (float *) malloc(sizeof(float) * xsize * ysize);
                for(int ii = 0; ii < xsize * ysize; ii++)
                {
                    refarray[ii] = 0.0;
                }
                for(int kk = 0; kk < framezero_start; kk++)
                {
                    for(int ii = 0; ii < xsize * ysize; ii++)
                    {
                        refarray[ii] += outimg->im->array.F[kk * xsize * ysize + ii];
                    }
                }
                for(int ii = 0; ii < xsize * ysize; ii++)
                {
                    refarray[ii] /= framezero_start;
                }


                for(int kk = 0; kk < zsize; kk++)
                {
                    for(int ii = 0; ii < xsize * ysize; ii++)
                    {
                        outimg->im->array.F[kk * xsize * ysize + ii] -= refarray[ii];
                    }
                }

                free(refarray);

                double resval = 0.0;
                for(int kk = 0; kk < framezero_start; kk++)
                {
                    for(int ii = 0; ii < xsize * ysize; ii++)
                    {
                        float dv =  outimg->im->array.F[kk * xsize * ysize + ii] * loopgain;
                        resval += dv * dv;
                        outimg->im->array.F[kk * xsize * ysize + ii] -= dv;
                    }
                }
                for(int kk = framezero_start; kk < zsize; kk++)
                {
                    for(int ii = 0; ii < xsize * ysize; ii++)
                    {
                        float dv =  outimg->im->array.F[kk * xsize * ysize + ii] * loopgain * 0.01;
                        resval += dv * dv;
                        outimg->im->array.F[kk * xsize * ysize + ii] -= dv;
                    }
                }
                printf(">>>>>>>>>>>>>>>>>>> OUTPUT SPACE RESIDUAL = %g\n", resval);
            }


            // Recompute input from solution
            //
            computeSGEMM(*outimg, imgtmat, &imgrec_alt, 0, 0, GPUdev);

            // Residual = input - rec

            {
                double resval = 0.0;
                for(long ii = 0; ii < imgrec_alt.md->size[0]*imgrec_alt.md->size[1]*imgrec_alt.md->size[2];
                        ii++)
                {
                    double v = inimg.im->array.F[ii] - imgrec_alt.im->array.F[ii];
                    imgres.im->array.F[ii] = v;
                    resval += v * v;
                }
                printf(">>>>>>>>>>>>>>>>>>>  INPUT SPACE RESIDUAL = %g\n", resval);
            }


            computeSGEMM(imgres, imgpsinv, &imgoutres, 0, 0, GPUdev);
            {
                int xsize = imgoutres.md->size[0];
                int ysize = imgoutres.md->size[1];
                int zsize = imgoutres.md->size[2];
                for(int kk = 0; kk < zsize; kk++)
                {
                    for(int ii = 0; ii < xsize * ysize; ii++)
                    {
                        outimg->im->array.F[kk * xsize * ysize + ii] += loopgain *
                                imgoutres.im->array.F[kk * xsize * ysize + ii];
                    }
                }
            }
        }
        imgid_free(&imgtmat);
        imgid_free(&imgU);
        imgid_free(&imgS);
        imgid_free(&imgV);
        imgid_free(&imgpsinv);
        imgid_free(&imgrec_alt);
        imgid_free(&imgres);
        imgid_free(&imgoutres);
    }
    imgid_free(&imgrec);


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static FPS_CLI_BINDING my_bindings[] __attribute__((unused)) = {
    FPS_PARAMS(FPS_X_BINDING)
};
static int nb_bindings __attribute__((unused)) = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

static CLICMDDATA CLIcmddata = {
    "mlatdsdecode", "mlat diff sequence decode", CLICMD_FIELDS_DEFAULTS
};

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID inimg = imgid_make_from_name(diffseqname);
    resolveIMGID(
        &inimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (inimg.ID == -1) return RETURN_FAILURE;


    IMGID outimg = imgid_make_from_name(outseqname);

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT


    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {

        mlat_diffseq_decode(
            inimg,
            &outimg,
            *oversamp,
            *nb0start,
            *nb0end
        );

        processinfo_update_output_stream(processinfo, outimg.im, NULL);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    imgid_free(&inimg);
    imgid_free(&outimg);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction() {
    return safe_fps_generic_CLIfunction(&FPS_app_info, farg, &CLIcmddata, my_bindings, nb_bindings, compute_function);
}

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_perfTest__mlat_decode()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(FPS_app_info, FPS_PARAMS, compute_function)
#endif
