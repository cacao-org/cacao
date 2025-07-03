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










// Local variables pointers

static char *selinput;

static char *wfsinput;


static float *selnormplaw;
static long      fpi_selnormplaw = -1;



static CLICMDARGDEF farg[] = {
    {
        CLIARG_IMG,
        ".selinput",
        "selection input (PSF)",
        "im1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &selinput,
        NULL
    },
    {
        CLIARG_IMG,
        ".wfsinput",
        "WFS input",
        "im1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsinput,
        NULL
    },
    {
        // argument is not part of CLI call, FPFLAG ignored
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
    float selnorm_powerlaw
)
{
    DEBUG_TRACE_FSTART();
    // custom stream process function code


    // resolve images
    resolveIMGID(&psfimg, ERRMODE_ABORT);

    // resolve images
    resolveIMGID(&wfsimg, ERRMODE_ABORT);



    uint32_t xsize  = psfimg.md->size[0];
    uint32_t ysize  = psfimg.md->size[1];
    uint64_t xysize = xsize * ysize;
    uint32_t zsize  = psfimg.md->size[2];

    double *psfnorm = (double *) malloc(sizeof(double)*zsize);

    for(uint32_t frame=0; frame < zsize; frame++)
    {
        double totalpow = 0.0;
        double total = 0.0;
        for(uint64_t ii = 0; ii < xysize; ii++)
        {
            double pval = psfimg.im->array.F[xysize*frame+ii];
            if(pval > 0.0)
            {
                totalpow += pow(pval, selnorm_powerlaw);
                total += pval;
            }
        }

        double fluxconc = totalpow / pow(total, selnorm_powerlaw);
        psfnorm[frame] = fluxconc;

        printf("%5d   %12.9f\n", frame, fluxconc);
    }


    // Create output image if needed
    //imcreateIMGID(outimg);


    // sort images according to psf norm
    //
    long *imindex = (long *) malloc(sizeof(long)*zsize);
    for(long i=0; i<zsize; i++) {
        imindex[i] = i;
    }

    quick_sort2l(psfnorm, imindex, zsize);


    // create 3D outputs
    IMGID imgpsfsorted  = makeIMGID_3D("psf_sorted", xsize, ysize, zsize);
    createimagefromIMGID(&imgpsfsorted);


    for(uint32_t frame=0; frame < zsize; frame++)
    {
        long slice = imindex[zsize-frame-1];
        printf("frome %5d  slice %5ld   val %11.9f\n", frame, slice, psfnorm[frame]);

        char *ptr0 = (char*) psfimg.im->array.F;
        ptr0 += sizeof(float)*xysize*slice;

        char *ptr1 = (char*) imgpsfsorted.im->array.F;
        ptr1 += sizeof(float)*xysize*frame;

        memcpy(ptr1, ptr0, sizeof(float)*xysize);
    }


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


    free(psfnorm);
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





    DEBUG_TRACE_FSTART();

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        WFSref_optimizeWFS_PSFselect(
            inpsfimg,
            inwfsimg,
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
