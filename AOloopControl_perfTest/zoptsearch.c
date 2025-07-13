/**
 * @file    zoptsearch.c
 * @brief   zonal optimizatoin search
 *
 * Search for otimal output by zonal actuation of input control stream
 * The optimization is from multidim input to scalar output
 */


#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "statistic/statistic.h" // ran1, gauss, gauss_trc
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



// input control stream
static char *ctrlsname;
long fpi_ctrlsname;


static uint32_t *nbpoke;
long fpi_nbpoke;


// actuation amplitude map
// actuation will be from -val to +val
static char *ctrlampmap;
long fpi_ctrlampmap;

// actuation amplitude
static float  *ctrlamp;
static long fpi_ctrlamp;



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
float *twaitframe;
static long     fpi_twaitframe;

// integrate sensing signal for number of frames
uint32_t *tintframe;
static long     fpi_tintframe;;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STR,
        ".ctrlsname",
        "control stream",
        "ctrl",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &ctrlsname,
        &fpi_ctrlsname
    },
    {
        CLIARG_UINT32,
        ".nbpoke",
        "number of pokes",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &nbpoke,
        &fpi_nbpoke
    },
    {
        CLIARG_FLOAT32,
        ".ctrlamp",
        "control amplitude",
        "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &ctrlamp,
        &fpi_ctrlamp
    },
    {
        CLIARG_STR,
        ".ctrlampmap",
        "control stream amplitude map",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &ctrlampmap,
        &fpi_ctrlampmap
    },
    {
        CLIARG_STR,
        ".senssname",
        "sensing stream",
        "sens",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &senssname,
        &fpi_senssname
    },
    {
        CLIARG_STR,
        ".sensprocout",
        "processed image output",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensproc,
        NULL
    },
    {
        CLIARG_STR,
        ".sproc.sensref0",
        "sensing reference 0",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensref0,
        NULL
    },
    {
        CLIARG_STR,
        ".sproc.sensmask0",
        "sensing mask 0 (float)",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensmask0,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".sproc.sensnorm0",
        "normalization 0 on/off",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensnorm0,
        &fpi_sensnorm0
    },
    {
        CLIARG_STR,
        ".sproc.sensref1",
        "sensing reference 1",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensref1,
        NULL
    },
    {
        CLIARG_STR,
        ".sproc.sensmask1",
        "sensing mask 1 (float)",
        "null",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensmask1,
        NULL
    },
    {
        CLIARG_ONOFF,
        ".sproc.sensnorm1",
        "normalization 1 on/off",
        "0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &sensnorm1,
        &fpi_sensnorm1
    },
    {
        CLIARG_UINT32,
        ".optm.opttype",
        "1:min, 2:max, 3:absmin, 4:absmax",
        "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &opttype,
        &fpi_opttype
    },
    {
        CLIARG_UINT32,
        ".optm.optcomp",
        "1:tot, 2:norma, 3:tota, 4:percr",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optcomp,
        &fpi_optcomp
    },
    {
        CLIARG_FLOAT32,
        ".optm.optparam0",
        "optimization parameter 0",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optparam0,
        &fpi_optparam0
    },
    {
        CLIARG_FLOAT32,
        ".optm.optparam1",
        "optimization parameter 1",
        "0.01",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &optparam1,
        &fpi_optparam1
    },
    {
        CLIARG_FLOAT32,
        ".twaitsec",
        "time to wait after poke before measurement",
        "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &twaitframe,
        &fpi_twaitframe
    },
    {
        CLIARG_UINT32,
        ".tintframe",
        "number of frames to integrate per measurement",
        "3",
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


    {
        double imtot = 0.0;
        double imtotpow = 0.0;
        int iperc0 = 0;
        int iperc1 = 0;
        float pixval = 0.0;

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
                pixval = imbuff[ii];
                if(pixval>0.0) {
                    optval += pow(pixval, optm.v0);
                }
            }
            break;

        case OPTMODE_COMP_NORM_ALPHA:
            for(uint32_t ii=0; ii<xysize; ii++)
            {
                pixval = imbuff[ii];
                if(pixval>0.0) {
                    imtot += pixval;
                    imtotpow += pow(pixval, optm.v0);
                }
            }
            optval = imtotpow/pow(imtot, optm.v0);
            break;

        case OPTMODE_COMP_PERCRANGE:
            quick_sort_float(imbuff, xysize);
            iperc0 = (int) (xysize*optm.v0);
            iperc1 = (int) (xysize*optm.v1);
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

    uint32_t ctrlxsize = imgctrl.md->size[0];
    uint32_t ctrlysize = imgctrl.md->size[1];
    uint32_t ctrlxysize = ctrlxsize*ctrlysize;


    // optional control amplitude map
    IMGID imgctrlamp;
    if ( strcmp(sensref0, "null") )
    {
        imgctrlamp = mkIMGID_from_name(ctrlampmap);
        resolveIMGID(&imgctrlamp, ERRMODE_ABORT);
    }
    else
    {
        imgctrlamp.ID = -1;
    }





    // connect to sensing stream
    //
    IMGID imgsens = mkIMGID_from_name(senssname);
    resolveIMGID(&imgsens, ERRMODE_ABORT);

    uint32_t sensxsize = imgsens.md->size[0];
    uint32_t sensysize = imgsens.md->size[1];
    uint32_t sensxysize = sensxsize*sensysize;
    IMGID imgacc = makeIMGID_2D("imacc", sensxsize, sensysize);
    createimagefromIMGID(&imgacc);

    // output of image processing
    IMGID imgsensproc;
    if ( strcmp(sensproc, "null") )
    {
        imgsensproc = stream_connect_create_2Df32(sensproc, sensxsize, sensysize);
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




    // prepare image cube buffers

    IMGID imgsenscube = makeIMGID_3D("imsenscube", sensxsize, sensysize, *nbpoke);
    createimagefromIMGID(&imgsenscube);

    IMGID imgctrlcube = makeIMGID_3D("imctrlcube", ctrlxsize, ctrlysize, *nbpoke);
    createimagefromIMGID(&imgctrlcube);




    list_image_ID();


    // frame index within poke
    //
    int framestep = 0;
    int framecollected = 0;
    uint32_t pokeindex = 0;

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        //printf("=== %3ld   framestep = %4d\n", processinfo->loopcnt, framestep);

        if(framestep == 0)
        {
            //printf("    Initialize\n");
            // apply control
            //
            {
                imgctrl.md->write = 1;
                if (imgctrlamp.ID == -1)
                {
                    // no amplitude map, assume range is from -1 to +1
                    for(uint32_t ii=0; ii<ctrlxysize; ii++)
                    {
                        imgctrl.im->array.F[ii] = (1.0 - 2.0*ran1()) * (*ctrlamp);
                    }
                }
                else
                {
                    for(uint32_t ii=0; ii<ctrlxysize; ii++)
                    {
                        imgctrl.im->array.F[ii] = (1.0 - 2.0*ran1()) * imgctrlamp.im->array.F[ii] * (*ctrlamp);
                    }
                }
                ImageStreamIO_UpdateIm(imgctrl.im);


                // copy to storage buffer
                printf("copying ctrl to buffer slize %d\n", pokeindex);
                if(pokeindex < *nbpoke)
                {
                    char * ptrdest;
                    ptrdest = (char*) imgctrlcube.im->array.F;
                    ptrdest += sizeof(float)*ctrlxysize*pokeindex;
                    memcpy(ptrdest, // + sizeof(float)*ctrlxysize*pokeindex,
                           imgctrl.im->array.F,
                           sizeof(float)*ctrlxysize
                          );
                }
            }
            framecollected = 0;

            // initialize accumulated frame
            for(uint32_t ii=0; ii<sensxysize; ii++)
            {
                imgacc.im->array.F[ii] = 0.0;
            }
        }

        if(framestep > *twaitframe)
        {
            // accumulate
            //printf("    Accumulate\n");
            for(uint32_t ii=0; ii<sensxysize; ii++)
            {
                imgacc.im->array.F[ii] += imgsens.im->array.F[ii];
            }

            framecollected++;
        }

        framestep++;

        if(framecollected == *tintframe)
        {
            // Average
            //printf(" >>>> Average and process\n");
            if(framecollected>1)
            {
                for(uint32_t ii=0; ii<sensxysize; ii++)
                {
                    imgacc.im->array.F[ii] /= framecollected;
                }
            }

            // copy to storage buffer
            printf("copying sens to buffer slize %d\n", pokeindex);
            if(pokeindex < *nbpoke)
            {
                char * ptrdest;
                ptrdest = (char*) imgsenscube.im->array.F;
                ptrdest += sizeof(float)*sensxysize*pokeindex;
                memcpy(ptrdest, // + sizeof(float)*ctrlxysize*pokeindex,
                       imgacc.im->array.F,
                       sizeof(float)*sensxysize
                      );
            }

            struct optimizationmode optm;
            optm.type = *opttype;
            optm.comp = *optcomp;
            optm.v0 = *optparam0;
            optm.v1 = *optparam1;
            optm.norm0 = *sensnorm0;
            optm.norm1 = *sensnorm1;

            double optval = image_optvalue(
                                imgacc,
                                optm,
                                imgsensref0,
                                imgsensmask0,
                                imgsensref1,
                                imgsensmask1,
                                imgsensproc
                            );

            printf("[%4d] %5ld  Value = %g\n", pokeindex, processinfo->loopcnt, optval);


            pokeindex ++;
            if(pokeindex == *nbpoke)
            {
                // exit loop
                printf("Reached pokeindex = %d -> Exiting loop\n", *nbpoke);
                processinfo->loopcntMax = 0;
            }

            framestep = 0;
        }


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
