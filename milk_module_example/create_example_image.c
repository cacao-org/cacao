/**
 * @file    create_example_image.c
 * @brief   example : create image
 *
 *
 */


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"





// By convention, function name starts with <modulename>__
//
errno_t milk_module_example__create_image_with_value(
    char *imname,
    double value
)
{
    uint32_t xsize = 128;
    uint32_t ysize = 256;
    uint64_t xysize = xsize * ysize;

    // create 2D image
    imageID ID = create_2Dimage_ID(imname, xsize, ysize);

    for(uint64_t ii = 0; ii < xysize; ii++)
    {
        data.image[ID].array.F[ii] = value;
    }

    return RETURN_SUCCESS;
}





