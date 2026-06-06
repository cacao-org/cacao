/**
 * @file    wfsrefoptimselec.c
 * @brief   Optimize WFS reference by PSF-based selection
 *
 *
 *
 */

#include <math.h>

#include <time.h>
#include <dirent.h>


#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "COREMOD_tools/COREMOD_tools.h" // quicksort


#define OPTMODE_MAXN 1
#define OPTMODE_MAXF 2
#define OPTMODE_MINF 3


// Local variables pointers

static char selinput[FUNCTION_PARAMETER_STRMAXLEN] = "";

static char wfsinput[FUNCTION_PARAMETER_STRMAXLEN] = "";

static char dminput[FUNCTION_PARAMETER_STRMAXLEN] = "";

static uint32_t *optmode;

static float *selnormplaw;


static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "wfsroptsel",
    .cmdkey      = "wfsroptsel",
    .description = "WFS ref optimize by PSF selection",
    .description_long =
        "Optimize WFS reference selection based on PSF quality metrics. Selects the reference that maximizes Strehl ratio."
};

#define FPS_PARAMS(X) \
    X(".selinput",   selinput,   FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "selection input (PSF)") \
    X(".wfsinput",   wfsinput,   FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "WFS input") \
    X(".dminput",    dminput,    FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "DM input") \
    X(".optmode",    &optmode,    FPTYPE_UINT32,     0, FPFLAG_DEFAULT_INPUT, "1 maxn, 2 maxf, 3 minf") \
    X(".selnormplaw",&selnormplaw,FPTYPE_FLOAT32,    0, FPFLAG_DEFAULT_INPUT, "selection norm power law")

static errno_t WFSref_optimizeWFS_PSFselect(
    IMGID psfimg,
    IMGID wfsimg,
    IMGID dmimg,
    int optmode,
    float selnorm_powerlaw
)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code


    uint32_t psfxsize  = psfimg.md->size[0];
    uint32_t psfysize  = psfimg.md->size[1];
    uint64_t psfxysize = psfxsize * psfysize;
    uint32_t zsize  = psfimg.md->size[2];

    double *psfvalue = (double *) malloc(sizeof(double)*zsize);

    switch (optmode) {

    case OPTMODE_MAXF:
        printf("OPTMODE: Max Flux\n");
        for(uint32_t frame=0; frame < zsize; frame++)
        {
            double total = 0.0;
            for(uint64_t ii = 0; ii < psfxysize; ii++)
            {
                double pval = psfimg.im->array.F[psfxysize*frame+ii];
                total += pval;
            }
            psfvalue[frame] = total;

            printf("%5d   %g\n", frame, total);
        }
        break;

    case OPTMODE_MINF:
        printf("OPTMODE: Min Flux\n");
        for(uint32_t frame=0; frame < zsize; frame++)
        {
            double total = 0.0;
            for(uint64_t ii = 0; ii < psfxysize; ii++)
            {
                double pval = psfimg.im->array.F[psfxysize*frame+ii];
                total += pval;
            }
            psfvalue[frame] = -total;

            printf("%5d   %g\n", frame, -total);
        }
        break;

    default:
        printf("OPTMODE: Max norm\n");
        for(uint32_t frame=0; frame < zsize; frame++)
        {
            double totalpow = 0.0;
            double total = 0.0;
            for(uint64_t ii = 0; ii < psfxysize; ii++)
            {
                double pval = psfimg.im->array.F[psfxysize*frame+ii];
                if(pval > 0.0)
                {
                    totalpow += pow(pval, selnorm_powerlaw);
                    total += pval;
                }
            }

            double fluxconc = totalpow / pow(total, selnorm_powerlaw);
            psfvalue[frame] = fluxconc;

            printf("%5d   %g\n", frame, fluxconc);
        }
        break;

    }

    {
        // Write values to file
        FILE *fppsfval = fopen("psfval.txt", "w");
        for(uint32_t frame=0; frame < zsize; frame++)
        {
            fprintf(fppsfval, "%5d  %g\n", frame, psfvalue[frame]);
        }
        fclose(fppsfval);
    }


    // sort images according to optimization metric
    //
    long *imindex = (long *) malloc(sizeof(long)*zsize);
    for(long i=0; i<zsize; i++) {
        imindex[i] = i;
    }

    quick_sort2l(psfvalue, imindex, zsize);


    // create 3D outputs
    //
    IMGID imgpsfsorted  = imgid_make_from_name_3D("psf_sorted", psfxsize, psfysize, zsize);
    createimagefromIMGID(&imgpsfsorted);


    for(uint32_t frame=0; frame < zsize; frame++)
    {
        long slice = imindex[zsize-frame-1];
        printf("frame %5d  slice %5ld   val %g\n", frame, slice, psfvalue[frame]);

        char *ptr0 = (char*) psfimg.im->array.F;
        ptr0 += sizeof(float)*psfxysize*slice;

        char *ptr1 = (char*) imgpsfsorted.im->array.F;
        ptr1 += sizeof(float)*psfxysize*frame;

        memcpy(ptr1, ptr0, sizeof(float)*psfxysize);
    }


    // max value for lambdai
    // lambdai is the selection exp coeff
    // frames are given a weigth according to their order from best to worst
    // coefficient = exp(-lambdai*x*x)
    // where x is the order (0.0=best, 1.0=worst)
    //
    int lambdaimax = 100;


    // WFS frames
    if(wfsimg.ID != -1)
    {
        uint32_t wfsxsize  = wfsimg.md->size[0];
        uint32_t wfsysize  = wfsimg.md->size[1];
        uint64_t wfsxysize = wfsxsize * wfsysize;
        uint32_t wfszsize  = wfsimg.md->size[2];

        IMGID imgwfssorted  = imgid_make_from_name_3D("wfs_sorted", wfsxsize, wfsysize, wfszsize);
        createimagefromIMGID(&imgwfssorted);

        for(uint32_t frame=0; frame < wfszsize; frame++)
        {
            long slice = imindex[wfszsize-frame-1];

            char *ptr0 = (char*) wfsimg.im->array.F;
            ptr0 += sizeof(float)*wfsxysize*slice;

            char *ptr1 = (char*) imgwfssorted.im->array.F;
            ptr1 += sizeof(float)*wfsxysize*frame;

            memcpy(ptr1, ptr0, sizeof(float)*wfsxysize);
        }

        for(int lambdai=0; lambdai < lambdaimax; lambdai*=2)
        {
            char  imgname[STRINGMAXLEN_IMGNAME];
            WRITE_IMAGENAME(imgname,
                            "wfsrefopt%d",
                            lambdai);

            IMGID imgwfsrefopt  = imgid_make_from_name_2D(imgname,
                wfsxsize,
                wfsysize);
            createimagefromIMGID(&imgwfsrefopt);

            double sumcoeff = 0.0;
            for(uint32_t frame=0; frame < wfszsize; frame++)
            {
                double xs = 1.0*frame/wfszsize;
                double coeff = exp(-1.0*lambdai*xs*xs);
                sumcoeff += coeff;

                for(uint64_t ii=0; ii<wfsxysize; ii++)
                {
                    imgwfsrefopt.im->array.F[ii] += coeff * imgwfssorted.im->array.F[frame*wfsxysize + ii];
                }
            }
            for(uint64_t ii=0; ii<wfsxysize; ii++)
            {
                imgwfsrefopt.im->array.F[ii] /= sumcoeff;
            }

            if(lambdai == 0)
            {
                lambdai = 1;
            }
        }
    }


    // DM frames
    if(dmimg.ID != -1)
    {
        uint32_t dmxsize  = dmimg.md->size[0];
        uint32_t dmysize  = dmimg.md->size[1];
        uint64_t dmxysize = dmxsize * dmysize;
        uint32_t dmzsize  = dmimg.md->size[2];

        IMGID imgdmsorted  = imgid_make_from_name_3D("dm_sorted", dmxsize, dmysize, dmzsize);
        createimagefromIMGID(&imgdmsorted);


        for(uint32_t frame=0; frame < dmzsize; frame++)
        {
            long slice = imindex[dmzsize-frame-1];

            char *ptr0 = (char*) dmimg.im->array.F;
            ptr0 += sizeof(float)*dmxysize*slice;

            char *ptr1 = (char*) imgdmsorted.im->array.F;
            ptr1 += sizeof(float)*dmxysize*frame;

            memcpy(ptr1, ptr0, sizeof(float)*dmxysize);
        }


        for(int lambdai=0; lambdai < lambdaimax; lambdai*=2)
        {
            char  imgname[STRINGMAXLEN_IMGNAME];
            WRITE_IMAGENAME(imgname,
                            "dmrefopt%d",
                            lambdai);

            IMGID imgdmrefopt  = imgid_make_from_name_2D(imgname,
                dmxsize,
                dmysize);
            createimagefromIMGID(&imgdmrefopt);

            double sumcoeff = 0.0;
            for(uint32_t frame=0; frame < dmzsize; frame++)
            {
                double xs = 1.0*frame/dmzsize;
                double coeff = exp(-1.0*lambdai*xs*xs);
                sumcoeff += coeff;

                for(uint64_t ii=0; ii<dmxysize; ii++)
                {
                    imgdmrefopt.im->array.F[ii] += coeff * imgdmsorted.im->array.F[frame*dmxysize + ii];
                }
            }

            for(uint64_t ii=0; ii<dmxysize; ii++)
            {
                imgdmrefopt.im->array.F[ii] /= sumcoeff;
            }
            if(lambdai == 0)
            {
                lambdai = 1;
            }
        }
    }


    free(psfvalue);
    free(imindex);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


static FPS_CLI_BINDING my_bindings[] = {
    FPS_PARAMS(FPS_X_BINDING)
};
static int __attribute__((unused)) nb_bindings = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

static CLICMDDATA CLIcmddata = {
    "wfsroptsel", "WFS ref optimize by PSF selection", CLICMD_FIELDS_DEFAULTS
};

static errno_t compute_function()
{
    IMGID inpsfimg = imgid_make_from_name(selinput);
    resolveIMGID(
        &inpsfimg, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (inpsfimg.ID == -1) return RETURN_FAILURE;

    IMGID inwfsimg;
    if ( strcmp(wfsinput, "null") )
    {
        inwfsimg = imgid_make_from_name(wfsinput);
        resolveIMGID(
            &inwfsimg, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (inwfsimg.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        inwfsimg.ID = -1;
    }


    IMGID indmimg;
    if ( strcmp(dminput, "null") )
    {
        indmimg = imgid_make_from_name(dminput);
        resolveIMGID(
            &indmimg, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (indmimg.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        indmimg.ID = -1;
    }


    DEBUG_TRACE_FSTART();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        WFSref_optimizeWFS_PSFselect(
            inpsfimg,
            inwfsimg,
            indmimg,
            *optmode,
            *selnormplaw
        );
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction() {
    return safe_fps_generic_CLIfunction(&FPS_app_info, farg, &CLIcmddata, my_bindings, nb_bindings, compute_function);
}

// Register function in CLI
errno_t CLIADDCMD_AOloopControl_perfTest__WFSref_optimize_PSFselection()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(FPS_app_info, FPS_PARAMS, compute_function)
#endif
