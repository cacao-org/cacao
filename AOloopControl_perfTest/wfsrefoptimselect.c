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


#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "COREMOD_tools/COREMOD_tools.h" // quicksort




#define OPTMODE_MAXN 1
#define OPTMODE_MAXF 2
#define OPTMODE_MINF 3





// Local variables pointers

static char *selinput;

static char *wfsinput;

static char *dminput;

static uint32_t *optmode;

static float *selnormplaw;
static long      fpi_selnormplaw = -1;



static CLICMDARGDEF farg[] = {
    {
        CLIARG_IMG,
        ".selinput",
        "selection input (PSF)",
        "psfim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &selinput,
        NULL
    },
    {
        CLIARG_IMG,
        ".wfsinput",
        "WFS input",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsinput,
        NULL
    },
    {
        CLIARG_IMG,
        ".dminput",
        "DM input",
        "dmim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dminput,
        NULL
    },
    {
        CLIARG_UINT32,
        ".optmode",
        "1 maxn, 2 maxf, 3 minf",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optmode,
        NULL
    },
    {
        CLIARG_FLOAT32,
        ".selnormplaw",
        "selection norm power law",
        "1.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &selnormplaw,
        &fpi_selnormplaw
    }
};



// Optional custom configuration setup.
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
    {}

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "wfsroptsel", "WFS ref optimize by PSF selectionk", CLICMD_FIELDS_DEFAULTS
};


// detailed help
static errno_t help_function()
{
    printf("resample streams to common clock\n");

    printf(
        "Convention\n"
    );

    return RETURN_SUCCESS;
}



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


    // resolve images
    resolveIMGID(&psfimg, ERRMODE_ABORT);

    // resolve images
    resolveIMGID(&wfsimg, ERRMODE_ABORT);



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

            printf("%5d   %12.9f\n", frame, total);
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

            printf("%5d   %12.9f\n", frame, -total);
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

            printf("%5d   %12.9f\n", frame, fluxconc);
        }
        break;

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
    IMGID imgpsfsorted  = makeIMGID_3D("psf_sorted", psfxsize, psfysize, zsize);
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

    // WFS frames
    {
        uint32_t wfsxsize  = wfsimg.md->size[0];
        uint32_t wfsysize  = wfsimg.md->size[1];
        uint64_t wfsxysize = wfsxsize * wfsysize;
        uint32_t wfszsize  = wfsimg.md->size[2];

        IMGID imgwfssorted  = makeIMGID_3D("wfs_sorted", wfsxsize, wfsysize, wfszsize);
        createimagefromIMGID(&imgwfssorted);

        IMGID imgwfsrefopt  = makeIMGID_2D("wfsrefopt", wfsxsize, wfsysize);
        createimagefromIMGID(&imgwfsrefopt);

        double sumcoeff = 0.0;
        double lambda = 10.0;

        for(uint32_t frame=0; frame < wfszsize; frame++)
        {
            long slice = imindex[wfszsize-frame-1];

            char *ptr0 = (char*) wfsimg.im->array.F;
            ptr0 += sizeof(float)*wfsxysize*slice;

            char *ptr1 = (char*) imgwfssorted.im->array.F;
            ptr1 += sizeof(float)*wfsxysize*frame;

            memcpy(ptr1, ptr0, sizeof(float)*wfsxysize);

            double xs = 1.0*frame/wfszsize;
            double coeff = exp(-lambda*xs*xs);
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
    }


    // DM frames
    {
        uint32_t dmxsize  = dmimg.md->size[0];
        uint32_t dmysize  = dmimg.md->size[1];
        uint64_t dmxysize = dmxsize * dmysize;
        uint32_t dmzsize  = dmimg.md->size[2];

        IMGID imgdmsorted  = makeIMGID_3D("dm_sorted", dmxsize, dmysize, dmzsize);
        createimagefromIMGID(&imgdmsorted);

        IMGID imgdmrefopt  = makeIMGID_2D("dmrefopt", dmxsize, dmysize);
        createimagefromIMGID(&imgdmrefopt);

        double sumcoeff = 0.0;
        double lambda = 10.0;

        for(uint32_t frame=0; frame < dmzsize; frame++)
        {
            long slice = imindex[dmzsize-frame-1];

            char *ptr0 = (char*) dmimg.im->array.F;
            ptr0 += sizeof(float)*dmxysize*slice;

            char *ptr1 = (char*) imgdmsorted.im->array.F;
            ptr1 += sizeof(float)*dmxysize*frame;

            memcpy(ptr1, ptr0, sizeof(float)*dmxysize);

            double xs = 1.0*frame/dmzsize;
            double coeff = exp(-lambda*xs*xs);
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
    }


    free(psfvalue);
    free(imindex);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}





static errno_t compute_function()
{
    IMGID inpsfimg = mkIMGID_from_name(selinput);
    resolveIMGID(&inpsfimg, ERRMODE_ABORT);

    IMGID inwfsimg = mkIMGID_from_name(wfsinput);
    resolveIMGID(&inwfsimg, ERRMODE_ABORT);

    IMGID indmimg = mkIMGID_from_name(dminput);
    resolveIMGID(&indmimg, ERRMODE_ABORT);

    int optmode = OPTMODE_MAXN;

    DEBUG_TRACE_FSTART();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        WFSref_optimizeWFS_PSFselect(
            inpsfimg,
            inwfsimg,
            indmimg,
            optmode,
            *selnormplaw
        );
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


INSERT_STD_FPSCLIfunctions


// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_perfTest__WFSref_optimize_PSFselection()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
