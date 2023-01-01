/**
 * @file    AOloopControl_computeCalib_Hadamard.c
 * @brief   Compute Hadamard modes
 *
 */

#define _GNU_SOURCE



#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"

#define MAX_MBLOCK 20

#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif





// Local variables pointers

static char *inmask;
static long  fpi_inmask;

static char *outHcube;
static long  fpi_outHcube;






static CLICMDARGDEF farg[] =
{
    {
        // AO loop index. Used for naming streams aolX_
        CLIARG_STR,
        ".inmask",
        "pixel mask (0 and 1 vals)",
        "imm",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inmask,
        &fpi_inmask
    },
    {
        CLIARG_STR,
        ".outHcube",
        "output Hadamard cube",
        "Hcube",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outHcube,
        &fpi_outHcube
    }
};


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
    {
    }

    return RETURN_SUCCESS;
}





static CLICMDDATA CLIcmddata =
{
    "mkHadamard", "make Hadamard modes", CLICMD_FIELDS_DEFAULTS
};




// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}





// output:
// Hadamard modes (outname)
// Hadamard matrix ("Hmat.fits")
// pixel indexes ("Hpixindex.fits", float, to be converted to long)
imageID AOloopControl_computeCalib_mkHadamardModes(
    const char *DMmask_name,
    const char *outname
)
{
    imageID IDout;
    long    cnt;

    uint32_t Hsize;
    uint32_t n2max;

    long     *indexarray;
    long      index;
    imageID   IDmat;
    int      *Hmat;
    imageID   IDindex;
    uint32_t *sizearray;

    imageID  IDmask = image_ID(DMmask_name);
    uint32_t xsize  = data.image[IDmask].md[0].size[0];
    uint32_t ysize  = data.image[IDmask].md[0].size[1];
    uint64_t xysize = xsize * ysize;

    sizearray = (uint32_t *) malloc(sizeof(uint32_t) * 2);
    if(sizearray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }
    sizearray[0] = xsize;
    sizearray[1] = ysize;
    create_image_ID("Hpixindex",
                    2,
                    sizearray,
                    _DATATYPE_FLOAT,
                    0,
                    0,
                    0,
                    &IDindex);
    free(sizearray);

    cnt = 0;
    for(uint64_t ii = 0; ii < xysize; ii++)
        if(data.image[IDmask].array.F[ii] > 0.5)
        {
            cnt++;
        }

    Hsize = 1;
    n2max = 0;
    while(Hsize < cnt)
    {
        Hsize *= 2;
        n2max++;
    }
    n2max++;

    printf("Hsize n2max = %u  %u\n", Hsize, n2max);
    fflush(stdout);

    for(uint64_t ii = 0; ii < xysize; ii++)
    {
        data.image[IDindex].array.F[ii] = -10.0;
    }

    index = 0;

    indexarray = (long *) malloc(sizeof(long) * Hsize);
    if(indexarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }
    for(uint32_t k = 0; k < Hsize; k++)
    {
        indexarray[k] = -1;
    }
    for(uint64_t ii = 0; ii < xysize; ii++)
        if((data.image[IDmask].array.F[ii] > 0.5) && (index < Hsize))
        {

            indexarray[index] = ii;
            // printf("(%ld %ld)  ", index, ii);

            data.image[IDindex].array.F[ii] = 1.0 * index;

            index++;
        }
    // save_fits("Hpixindex", "./conf/Hpixindex.fits");

    Hmat = (int *) malloc(sizeof(int) * Hsize * Hsize);
    if(Hmat == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort();
    }

    // n = 0

    uint32_t ii           = 0;
    uint32_t jj           = 0;
    Hmat[jj * Hsize + ii] = 1;
    uint32_t n2           = 1;
    for(uint32_t n = 1; n < n2max; n++)
    {
        for(uint32_t ii = 0; ii < n2; ii++)
            for(uint32_t jj = 0; jj < n2; jj++)
            {
                Hmat[jj * Hsize + (ii + n2)]        = Hmat[jj * Hsize + ii];
                Hmat[(jj + n2) * Hsize + (ii + n2)] = -Hmat[jj * Hsize + ii];
                Hmat[(jj + n2) * Hsize + ii]        = Hmat[jj * Hsize + ii];
            }
        n2 *= 2;
    }

    DEBUG_TRACEPOINT("n2 = %u", n2);

    create_2Dimage_ID("Hmat", Hsize, Hsize, &IDmat);

    for(uint32_t ii = 0; ii < Hsize; ii++)
        for(uint32_t jj = 0; jj < Hsize; jj++)
        {
            data.image[IDmat].array.F[jj * Hsize + ii] = Hmat[jj * Hsize + ii];
        }

    //    save_fits("Htest", "./conf/Hmat.fits");

    DEBUG_TRACEPOINT("image %s size %u %u %u", outname, xsize, ysize, Hsize);
    create_3Dimage_ID(outname, xsize, ysize, Hsize, &IDout);
    list_image_ID();

    for(uint32_t k = 0; k < Hsize; k++)
    {
        for(uint32_t index = 0; index < Hsize; index++)
        {
            long ii = indexarray[index];

            if(ii >= 0)
            {
                DEBUG_TRACEPOINT("%u %u %ld", k, index, indexarray[index]);
                data.image[IDout].array.F[k * xysize + ii] =
                    Hmat[k * Hsize + index];
            }
        }
    }

    free(Hmat);

    free(indexarray);

    DEBUG_TRACEPOINT("exit function");

    return IDout;
}




imageID AOloopControl_computeCalib_Hadamard_decodeRM(
    const char *inname,
    const char *Hmatname,
    const char *indexname,
    const char *outname)
{
    imageID  IDin, IDhad, IDout, IDindex;
    long     NBframes, sizexwfs, sizeywfs, sizewfs;
    long     kk, kk1, ii;
    uint32_t zsizeout;

    IDin     = image_ID(inname);
    sizexwfs = data.image[IDin].md[0].size[0];
    sizeywfs = data.image[IDin].md[0].size[1];
    sizewfs  = sizexwfs * sizeywfs;
    NBframes = data.image[IDin].md[0].size[2];

    IDindex = image_ID(indexname);

    IDhad = image_ID(Hmatname);
    if((data.image[IDhad].md[0].size[0] != NBframes) ||
            (data.image[IDhad].md[0].size[1] != NBframes))
    {
        printf(
            "ERROR: size of Hadamard matrix [%ld x %ld] does not match "
            "available number of frames [%ld]\n",
            (long) data.image[IDhad].md[0].size[0],
            (long) data.image[IDhad].md[0].size[1],
            NBframes);
        exit(0);
    }

    zsizeout =
        data.image[IDindex].md[0].size[0] * data.image[IDindex].md[0].size[1];
    create_3Dimage_ID(outname, sizexwfs, sizeywfs, zsizeout, &IDout);

    long kk0;
#ifdef _OPENMP
    #pragma omp parallel for private(kk0, kk1, ii)
#endif
    for(kk = 0; kk < zsizeout; kk++)  // output frame
    {
        kk0 = (long)(data.image[IDindex].array.F[kk] + 0.1);
        if(kk0 > -1)
        {
            printf("\r  frame %5ld / %5ld     ", kk0, NBframes);
            fflush(stdout);
            for(kk1 = 0; kk1 < NBframes; kk1++)
            {
                for(ii = 0; ii < sizewfs; ii++)
                    data.image[IDout].array.F[kk * sizewfs + ii] +=
                        data.image[IDin].array.F[kk1 * sizewfs + ii] *
                        data.image[IDhad].array.F[kk0 * NBframes + kk1];
            }
        }
    }

    for(kk = 0; kk < zsizeout; kk++)
    {
        for(ii = 0; ii < sizewfs; ii++)
        {
            data.image[IDout].array.F[kk * sizewfs + ii] /= NBframes;
        }
    }

    printf("\n\n");

    return (IDout);
}



static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    printf("INPUT  : %s\n", inmask);
    printf("OUTPUT : %s\n", outHcube);

    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        AOloopControl_computeCalib_mkHadamardModes(
            inmask,
            outHcube);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions



// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_computeCalib__mkHadamard()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
