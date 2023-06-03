/**
 * @file CLIADDCMD_AOloopControl_computeCalib__sample2D
 *
 */


#include "CommandLineInterface/CLIcore.h"




// sample 2D WF to 1D actuator geometry
//
static char *inWF2D;
static long  fpi_inWF2D;


static char *map2D;
static long  fpi_map2D;


static char *outWF1D;
static long  fpi_outWF1D;

static CLICMDARGDEF farg[] =
{
    {
        // Input WF or cube
        CLIARG_IMG,
        ".inwf2D",
        "input 2D wavefront",
        "inwf2D",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inWF2D,
        &fpi_inWF2D
    },
    {
        CLIARG_IMG,
        ".mapfile",
        "mapping file, can be read from mapcoord2D.txt",
        "mapfile",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &map2D,
        &fpi_map2D
    },
    {
        CLIARG_IMG,
        ".outWF1D",
        "output WF 1D",
        "outWF1D",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outWF1D,
        &fpi_outWF1D
    }
};




// Optional custom configuration setup. comptbuff
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_inWF2D].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;

        data.fpsptr->parray[fpi_map2D].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED;
    }

    return RETURN_SUCCESS;
}



// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "sample2DWF", "sample 2D WF to act pos", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}



static IMGID load_actmapcoord2D(
    char *fname,
    char *outim
)
{
    FILE *fp = NULL;

    fp = fopen(fname, "r");
    if(fp == NULL)
    {
        printf("ERROR: cannot load file %s\n", fname);
        exit(0);
    }

    // count number of lines
    long actindex;
    float xcoord;
    float ycoord;
    long NBact = 0; // counter
    while( fscanf(fp, "%ld %f %f\n", &actindex, &xcoord, &ycoord) == 3 )
    {
        NBact++;
    }
    fclose(fp);



    IMGID imgout = makeIMGID_2D(outim, NBact, 2);
    createimagefromIMGID(&imgout);


    fp = fopen(fname, "r");
    if(fp == NULL)
    {
        printf("ERROR: cannot load file %s\n", fname);
        exit(0);
    }
    for(uint32_t act = 0; act < NBact; act++)
    {

        int ret = fscanf(fp, "%ld %f %f\n", &actindex, &xcoord, &ycoord);
        if(ret != 3)
        {
            printf("ERROR reading file %s\n", fname);
            exit(0);
        }

        imgout.im->array.F[act*2] = xcoord;
        imgout.im->array.F[act*2 + 1] = ycoord;
    }
    fclose(fp);


    return imgout;
}




static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID imgWF2D = mkIMGID_from_name(inWF2D);
    resolveIMGID(&imgWF2D, ERRMODE_ABORT);
    uint32_t wfxsize = imgWF2D.md->size[0];
    uint32_t wfysize = imgWF2D.md->size[1];
    uint64_t wfsize = wfxsize;
    wfsize *= wfysize;
    printf("wfsize = %lu\n", wfsize);


    IMGID imgmap2D = mkIMGID_from_name(map2D);
    resolveIMGID(&imgmap2D, ERRMODE_WARN);
    if(imgmap2D.ID == -1)
    {
        imgmap2D = load_actmapcoord2D("mapcoord2D.txt", map2D);
    }
    uint32_t mapsize = imgmap2D.md->size[0];
    // mapsizedim should be 2: x and y coord
    uint32_t mapsizedim = imgmap2D.md->size[1];
    printf("mapsize = %u\n", mapsize);




    uint32_t NBslice = imgWF2D.md->size[2];
    printf("%u slice\n", NBslice);


    IMGID imgoutWF1D = makeIMGID_3D(outWF1D, mapsize, 1, NBslice);
    createimagefromIMGID(&imgoutWF1D);


    printf("OUTPUT IMAGE CREATED\n");
    fflush(stdout);
    list_image_ID();

    float xcentf = 0.5*wfxsize;
    float ycentf = 0.5*wfysize;
    float radf   = 0.25*(wfxsize+wfysize);


    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        for(uint32_t slice=0; slice < NBslice; slice++)
        {
            printf(".");
            fflush(stdout);

            for(uint32_t act=0; act < mapsize; act++)
            {
                // actuator coordinates
                // relative to beam center, radius = 1
                //
                float xact = imgmap2D.im->array.F[act*2];
                float yact = imgmap2D.im->array.F[act*2+1];

                long iiact = (long) (xcentf + radf*xact);
                long jjact = (long) (ycentf + radf*yact);

                if(iiact < 0)
                {
                    iiact = 0;
                }
                if(iiact > wfxsize-1)
                {
                    iiact = wfxsize-1;
                }

                if(jjact < 0)
                {
                    jjact = 0;
                }
                if(jjact > wfysize-1)
                {
                    jjact = wfysize-1;
                }

                imgoutWF1D.im->array.F[slice*mapsize + act] =
                    imgWF2D.im->array.F[slice*mapsize + jjact*wfxsize + iiact];
            }
        }
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__sample2D()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
