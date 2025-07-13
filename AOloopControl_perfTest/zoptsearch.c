/**
 * @file    zoptsearch.c
 * @brief   zonal optimizatoin search
 *
 * Search for otimal output by zonal actuation of input control stream
 * The optimization is from multidim input to scalar output
 */


#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_tools/quicksort.h"

// optimization modes

struct optimizationmode {
    int type; // see OPTMODE_TYPE_ defs
    int comp; // see OPTMODE_COMP_ defs
    int norm0; // normalize at step0
    int norm1; // normalize at step1
    float v0;
    float v1;
};


// minimize computed value
#define OPTMODE_TYPE_MIN 1

// maximize computed value
#define OPTMODE_TYPE_MAX 2

// minimize abs value of computed value
#define OPTMODE_TYPE_ABSMIN 3

// maximize abs value of computed value
#define OPTMODE_TYPE_ABSMAX 4



// Total flux
#define OPTMODE_COMP_TOTFLUX 1

// Norm alpha: sum(p^a)/sum(p)^a
// optparam0: alpha
#define OPTMODE_COMP_NORM_ALPHA 2

// Sum power alpha: sum(p^a)
// optparam0: alpha
#define OPTMODE_COMP_TOTPALPHA 3

// Difference between percentiles
// optparam0: percmin
// optparam1: percmax
#define OPTMODE_COMP_PERCRANGE 4





// Local variables pointers

static int64_t *optON;
static long     fpi_optON;


// input control stream
static char *ctrlsname;
long fpi_ctrlsname;

// actuation amplitude map
// actuation will be from -val to +val
static char *ctrlmapamp;
long fpi_ctrlmapamp;




// sensing stream
static char *senssname;
long fpi_senssname;


// output of image processing performed in this function
static char *sensproc;
long fpi_sensproc;


// Processing steps on sensing stream (optional)

// sensing stream reference
// will subtract if not "null"
// here this is before any masking or normalization
// for example, dark subtrzction
//
static char *sensref0;
long fpi_sensref0;

// sensing stream mask
// will apply if not "null"
static char *sensmask0;
long fpi_sensmask0;

// sensing stream normalize
// flag
//
static int64_t *sensnorm0;
static long     fpi_sensnorm0;



// sensing stream reference
// will subtract if not "null"
static char *sensref1;
long fpi_sensref1;

// sensing stream mask
// will apply if not "null"
static char *sensmask1;
long fpi_sensmask1;

// sensing stream normalize
// flag
static int64_t *sensnorm1;
static long     fpi_sensnorm1;



// Optimization metric
//
static int64_t *opttype;
static long     fpi_opttype;

static int64_t *optcomp;
static long     fpi_optcomp;


// optimization parameters
// meaning is spectific to optimizatoin comp modes
//
static float  *optparam0;
static long fpi_optparam0;

static float  *optparam1;
static long fpi_optparam1;


// Timing

// wait number of frames after actuation
uint32_t twaitframe;
static long     fpi_twaitframe;

// integrate sensing signal for number of frames
uint32_t tintframe;
static long     fpi_tintframe;;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_ONOFF,
        ".optON",
        "Optimization on/off",
        "1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &optON,
        &fpi_optON
    },
    {
        CLIARG_STREAM,
        ".ctrlsname",
        "control stream",
        "ctrl",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &ctrlsname,
        &fpi_ctrlsname
    },
    {
        CLIARG_STREAM,
        ".senssname",
        "sensing stream",
        "sens",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &senssname,
        &fpi_senssname
    },
    {
        CLIARG_STR,
        ".sensproc",
        "processed image output",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensproc,
        NULL
    },
    {
        CLIARG_STR,
        ".sensref0",
        "sensing reference 0",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensref0,
        NULL
    },
    {
        CLIARG_STR,
        ".sensmask0",
        "sensing mask 0 (float)",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensmask0,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".sensnorm0",
        "normalization 0 on/off",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensnorm0,
        &fpi_sensnorm0
    },
    {
        CLIARG_STR,
        ".sensref1",
        "sensing reference 1",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensref1,
        NULL
    },
    {
        CLIARG_STR,
        ".sensmask1",
        "sensing mask 1 (float)",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensmask1,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".sensnorm1",
        "normalization 1 on/off",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensnorm1,
        &fpi_sensnorm1
    },
    {
        CLIARG_UINT32,
        ".opttype",
        "1:min, 2:max, 3:absmin, 4:absmax",
        "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &opttype,
        &fpi_opttype
    },
    {
        CLIARG_UINT32,
        ".optcomp",
        "1:tot, 2:norma, 3:tota, 4:percr",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optcomp,
        &fpi_optcomp
    },
    {
        CLIARG_FLOAT32,
        ".optparam0",
        "optimization parameter 0",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optparam0,
        &fpi_optparam0
    },
    {
        CLIARG_FLOAT32,
        ".optparam1",
        "optimization parameter 1",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optparam1,
        &fpi_optparam1
    },
    {
        CLIARG_UINT32,
        ".twaitframe",
        "number of frames to wait before measurement",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &twaitframe,
        &fpi_twaitframe
    },
    {
        CLIARG_UINT32,
        ".tintframe",
        "number of frames to integrate per measurement",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &tintframe,
        &fpi_tintframe
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_ctrlsname].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_senssname].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_sensnorm0].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_sensnorm1].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_optparam0].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_optparam1].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_twaitframe].fpflag    |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_tintframe].fpflag     |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "zoptsearch", "stream zonal control optimize search", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}




// Returns optimization metric value
//
static double image_optvalue(
    IMGID imgin,
    struct optimizationmode optm,
    IMGID imgref0,
    IMGID imgmask0,
    IMGID imgref1,
    IMGID imgmask1,
    IMGID imgout
)
{
    uint32_t xsize = imgin.md->size[0];
    uint32_t ysize = imgin.md->size[1];
    uint32_t xysize = xsize*ysize;

    // allocate internal buffer
    float *imbuff = (float *) malloc(sizeof(float)*xsize*ysize);
    memcpy(imbuff, imgin.im->array.F, sizeof(float)*xysize);


    // IMAGE PREPROCESSING

    // ref0 subtraction
    if ( imgref0.ID != -1 )
    {
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] -= imgref0.im->array.F[ii];
        }
    }
    // mask0 multiplication
    if ( imgmask0.ID != -1 )
    {
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] *= imgmask0.im->array.F[ii];
        }
    }
    // normalization 0
    if ( optm.norm0 == 1 )
    {
        double imtot = 0.0;
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imtot += imbuff[ii];
        }
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] /= imtot;
        }
    }

    // ref1 subtraction
    if ( imgref1.ID != -1 )
    {
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] -= imgref1.im->array.F[ii];
        }
    }
    // mask0 multiplication
    if ( imgmask1.ID != -1 )
    {
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] *= imgmask1.im->array.F[ii];
        }
    }
    // normalization 0
    if ( optm.norm1 == 1 )
    {
        double imtot = 0.0;
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imtot += imbuff[ii];
        }
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            imbuff[ii] /= imtot;
        }
    }

    // copy to output if applicable
    if ( imgout.ID != -1 )
    {
        imgout.md->write = 1;
        memcpy(imgout.im->array.F, imbuff, sizeof(float)*xysize);
        ImageStreamIO_UpdateIm(imgout.im);
    }



    // COMPUTING OPTIMIZATION METRIC

    double optval = 0.0;

    switch (optm.comp)
    {

    case OPTMODE_COMP_TOTFLUX:
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            optval += imbuff[ii];
        }
        break;

    case OPTMODE_COMP_TOTPALPHA:
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            float val = imbuff[ii];
            if(val>0.0) {
                optval += pow(val, optm.v0);
            }
        }
        break;

    case OPTMODE_COMP_NORM_ALPHA:
        double imtot = 0.0;
        double imtotpow = 0.0;
        for(uint32_t ii=0; ii<xysize; ii++)
        {
            float val = imbuff[ii];
            if(val>0.0) {
                imtot += val;
                imtotpow += pow(val, optm.v0);
            }
        }
        optval = imtotpow/pow(imtot, optm.v0);
        break;

    case OPTMODE_COMP_PERCRANGE:
        quick_sort_float(imbuff, xysize);
        int iperc0 = (int) (xysize*optm.v0);
        int iperc1 = (int) (xysize*optm.v1);
        if(iperc0 < 0) {
            iperc0 = 0;
        }
        if(iperc0 > xysize-1) {
            iperc0 = xysize-1;
        }
        if(iperc1 < 0) {
            iperc1 = 0;
        }
        if(iperc1 > xysize-1) {
            iperc1 = xysize-1;
        }
        optval = imbuff[iperc1] - imbuff[iperc0];
        break;
    }

    free(imbuff);

    return optval;
}





static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to control stream
    //
    IMGID imgctrl = mkIMGID_from_name(ctrlsname);
    resolveIMGID(&imgctrl, ERRMODE_ABORT);

    // connect to sensing stream
    //
    IMGID imgsens = mkIMGID_from_name(senssname);
    resolveIMGID(&imgsens, ERRMODE_ABORT);

    // output of image processing
    IMGID imgsensproc;
    if ( strcmp(sensproc, "null") )
    {
        uint32_t xsize = imgsens.md->size[0];
        uint32_t ysize = imgsens.md->size[1];
        imgsensproc = stream_connect_create_2Df32(sensproc, xsize, ysize);
    }
    else
    {
        imgsensproc.ID = -1;
    }




    // connect to optional masks and references

    IMGID imgsensref0;
    if ( strcmp(sensref0, "null") )
    {
        imgsensref0 = mkIMGID_from_name(sensref0);
        resolveIMGID(&imgsensref0, ERRMODE_ABORT);
    }
    else
    {
        imgsensref0.ID = -1;
    }

    IMGID imgsensmask0;
    if ( strcmp(sensmask0, "null") )
    {
        imgsensmask0 = mkIMGID_from_name(sensmask0);
        resolveIMGID(&imgsensmask0, ERRMODE_ABORT);
    }
    else
    {
        imgsensmask0.ID = -1;
    }

    IMGID imgsensref1;
    if ( strcmp(sensref1, "null") )
    {
        imgsensref1 = mkIMGID_from_name(sensref1);
        resolveIMGID(&imgsensref1, ERRMODE_ABORT);
    }
    else
    {
        imgsensref1.ID = -1;
    }

    IMGID imgsensmask1;
    if ( strcmp(sensmask1, "null") )
    {
        imgsensmask1 = mkIMGID_from_name(sensmask1);
        resolveIMGID(&imgsensmask1, ERRMODE_ABORT);
    }
    else
    {
        imgsensmask1.ID = -1;
    }


    list_image_ID();





    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        struct optimizationmode optm;
        optm.type = *opttype;
        optm.comp = *optcomp;
        optm.v0 = *optparam0;
        optm.v1 = *optparam1;
        optm.norm0 = *sensnorm0;
        optm.norm1 = *sensnorm1;

        double optval = image_optvalue(
            imgsens,
            optm,
            imgsensref0,
            imgsensmask0,
            imgsensref1,
            imgsensmask1,
            imgsensproc
        );

        printf("Value = %g\n", optval);


//        if(data.fpsptr->parray[fpi_compWFSrefc].fpflag & FPFLAG_ONOFF)


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions




// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_perfTest__zoptsearch()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
