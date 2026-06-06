/**
 * @file    zoptsearch.c
 * @brief   zonal optimizatoin search
 *
 * Search for otimal output by zonal actuation of input control stream
 * The optimization is from multidim input to scalar output
 */


#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "statistic/statistic.h" // ran1, gauss, gauss_trc
#include "quicksort.h"

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

static char     *ctrlsname;
static uint32_t *nbpoke;
static float    *ctrlamp;
static char     *ctrlampmap;
static char     *senssname;
static char     *sensproc;
static char     *sensref0;
static char     *sensmask0;
static int32_t  *sensnorm0;
static char     *sensref1;
static char     *sensmask1;
static int32_t  *sensnorm1;
static uint32_t *opttype;
static uint32_t *optcomp;
static float    *optparam0;
static float    *optparam1;
static float    *twaitframe;
static uint32_t *tintframe;

static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "zoptsearch",
    .cmdkey      = "zoptsearch",
    .description = "stream zonal control optimize search",
    .description_long =
        "Search for optimal zonal control parameters by systematically varying gains and analyzing closed-loop residuals."
};

#define FPS_PARAMS(X) \
    X(".ctrlsname",       &ctrlsname,       FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM, "control stream") \
    X(".nbpoke",          &nbpoke,          FPTYPE_UINT32,     1, FPFLAG_DEFAULT_INPUT, "number of pokes") \
    X(".ctrlamp",         &ctrlamp,         FPTYPE_FLOAT32,    1, FPFLAG_DEFAULT_INPUT, "control amplitude") \
    X(".ctrlampmap",      &ctrlampmap,      FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "control stream amplitude map") \
    X(".senssname",       &senssname,       FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM, "sensing stream") \
    X(".sensprocout",     &sensproc,        FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "processed image output") \
    X(".sproc.sensref0",  &sensref0,        FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "sensing reference 0") \
    X(".sproc.sensmask0", &sensmask0,       FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "sensing mask 0 (float)") \
    X(".sproc.sensnorm0", &sensnorm0,       FPTYPE_ONOFF,      0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "normalization 0 on/off") \
    X(".sproc.sensref1",  &sensref1,        FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "sensing reference 1") \
    X(".sproc.sensmask1", &sensmask1,       FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT, "sensing mask 1 (float)") \
    X(".sproc.sensnorm1", &sensnorm1,       FPTYPE_ONOFF,      0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "normalization 1 on/off") \
    X(".optm.opttype",    &opttype,         FPTYPE_UINT32,     0, FPFLAG_DEFAULT_INPUT, "1:min, 2:max, 3:absmin, 4:absmax") \
    X(".optm.optcomp",    &optcomp,         FPTYPE_UINT32,     0, FPFLAG_DEFAULT_INPUT, "1:tot, 2:norma, 3:tota, 4:percr") \
    X(".optm.optparam0",  &optparam0,       FPTYPE_FLOAT32,    0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "optimization parameter 0") \
    X(".optm.optparam1",  &optparam1,       FPTYPE_FLOAT32,    0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "optimization parameter 1") \
    X(".twaitsec",        &twaitframe,      FPTYPE_FLOAT32,    0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "time to wait after poke before measurement") \
    X(".tintframe",       &tintframe,       FPTYPE_UINT32,     0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN, "number of frames to integrate per measurement")
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


static FPS_CLI_BINDING my_bindings[] __attribute__((unused)) = {
    FPS_PARAMS(FPS_X_BINDING)
};
static int nb_bindings __attribute__((unused)) = sizeof(my_bindings) / sizeof(FPS_CLI_BINDING);

static CLICMDARGDEF farg[] = {
    FPS_PARAMS(FPS_X_FARG)
};

static CLICMDDATA CLIcmddata = { "zoptsearch", "stream zonal control optimize search", CLICMD_FIELDS_DEFAULTS };

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to control stream
    //
    IMGID imgctrl = imgid_make_from_name(ctrlsname);
    resolveIMGID(
        &imgctrl, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (imgctrl.ID == -1) return RETURN_FAILURE;

    uint32_t ctrlxsize = imgctrl.md->size[0];
    uint32_t ctrlysize = imgctrl.md->size[1];
    uint32_t ctrlxysize = ctrlxsize*ctrlysize;


    // optional control amplitude map
    IMGID imgctrlamp;
    if ((ctrlampmap != NULL) && (strcmp(ctrlampmap, "null") != 0))
    {
        imgctrlamp = imgid_make_from_name(ctrlampmap);
        resolveIMGID(
            &imgctrlamp, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (imgctrlamp.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        imgctrlamp.ID = -1;
    }


    // connect to sensing stream
    //
    IMGID imgsens = imgid_make_from_name(senssname);
    resolveIMGID(
        &imgsens, ERRMODE_WARN,
        dcimg,
        dcnimg);
        if (imgsens.ID == -1) return RETURN_FAILURE;

    uint32_t sensxsize = imgsens.md->size[0];
    uint32_t sensysize = imgsens.md->size[1];
    uint32_t sensxysize = sensxsize*sensysize;
    IMGID imgacc = imgid_make_from_name_2D("imacc", sensxsize, sensysize);
    createimagefromIMGID(&imgacc);

    // output of image processing
    IMGID imgsensproc;
    if ( strcmp(sensproc, "null") )
    {
        imgsensproc = stream_connect_create_2Df32(sensproc,
            sensxsize,
            sensysize);
    }
    else
    {
        imgsensproc.ID = -1;
    }


    // connect to optional masks and references

    IMGID imgsensref0;
    if ( strcmp(sensref0, "null") )
    {
        imgsensref0 = imgid_make_from_name(sensref0);
        resolveIMGID(
            &imgsensref0, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (imgsensref0.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        imgsensref0.ID = -1;
    }

    IMGID imgsensmask0;
    if ( strcmp(sensmask0, "null") )
    {
        imgsensmask0 = imgid_make_from_name(sensmask0);
        resolveIMGID(
            &imgsensmask0, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (imgsensmask0.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        imgsensmask0.ID = -1;
    }

    IMGID imgsensref1;
    if ( strcmp(sensref1, "null") )
    {
        imgsensref1 = imgid_make_from_name(sensref1);
        resolveIMGID(
            &imgsensref1, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (imgsensref1.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        imgsensref1.ID = -1;
    }

    IMGID imgsensmask1;
    if ( strcmp(sensmask1, "null") )
    {
        imgsensmask1 = imgid_make_from_name(sensmask1);
        resolveIMGID(
            &imgsensmask1, ERRMODE_WARN,
            dcimg,
            dcnimg);
            if (imgsensmask1.ID == -1) return RETURN_FAILURE;
    }
    else
    {
        imgsensmask1.ID = -1;
    }


    // prepare image cube buffers

    IMGID imgsenscube = imgid_make_from_name_3D("imsenscube", sensxsize, sensysize, *nbpoke);
    createimagefromIMGID(&imgsenscube);

    IMGID imgctrlcube = imgid_make_from_name_3D("imctrlcube", ctrlxsize, ctrlysize, *nbpoke);
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


//        if(milk_data.fpsptr->parray[fpi_compWFSrefc].fpflag & FPFLAG_ONOFF)


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


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
CLIADDCMD_AOloopControl_perfTest__zoptsearch()
{
    safe_fps_fill_farg_examples(farg, my_bindings, nb_bindings);
    INSERT_STD_CLIREGISTERFUNC
    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(FPS_app_info, FPS_PARAMS, compute_function)
#endif
