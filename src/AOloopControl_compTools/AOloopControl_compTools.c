/**
 * @file    AOloopControl_compTools.c
 * @brief   Adaptive Optics Control loop engine misc computation tools
 * 
 * AO engine uses stream data structure
 * 
 * 
 */




/* ================================================================== */
/* ================================================================== */
/*            MODULE INFO                                             */
/* ================================================================== */
/* ================================================================== */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT ""

// Module short description 
#define MODULE_DESCRIPTION       "AO loop control - computation tools"

// Application to which module belongs
#define MODULE_APPLICATION       "cacao"







#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <string.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_compTools/AOloopControl_compTools.h"
#include "COREMOD_memory/COREMOD_memory.h"









/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */



#define NB_AOloopcontrol 10 // max number of loops


int AOloopcontrol_meminit = 0;
//static int AOlooploadconf_init = 0;


// defined in AOloopControl.c
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array









/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl_compTools)


/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */



/** @name AOloopControl functions */




/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_CrossProduct */
errno_t AOloopControl_compTools_CrossProduct_cli() {
    if(
        CLI_checkarg(1,4) +
        CLI_checkarg(2,4) +
        CLI_checkarg(3,3)
        == 0 )
    {
        AOloopControl_compTools_CrossProduct(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string
        );

        return CLICMD_SUCCESS;
    }
    else {
        return CLICMD_INVALID_ARG;
    }
}


/** @brief CLI function for AOloopControl_mkSimpleZpokeM */
errno_t AOloopControl_compTools_mkSimpleZpokeM_cli()
{
    if(
        CLI_checkarg(1,2) +
        CLI_checkarg(2,2) +
        CLI_checkarg(3,3)
        == 0 )
    {
        AOloopControl_compTools_mkSimpleZpokeM(
            data.cmdargtoken[1].val.numl,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.string
        );

        return CLICMD_SUCCESS;
    }
    else {
        return CLICMD_INVALID_ARG;
    }
}









static errno_t init_module_CLI()
{

    /* =============================================================================================== */
    /** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
    /* =============================================================================================== */

    RegisterCLIcommand(
        "aolcrossp",
        __FILE__,
        AOloopControl_compTools_CrossProduct_cli,
        "compute cross product between two cubes. Apply mask if image xpmask exists",
        "<cube1> <cube2> <output image>",
        "aolcrossp imc0 imc1 crosspout",
        "AOloopControl_compTools_CrossProduct(char *ID1_name, char *ID1_name, char *IDout_name)"
    );

    RegisterCLIcommand(
        "aolmksimplezpM",
        __FILE__,
        AOloopControl_compTools_mkSimpleZpokeM_cli,
        "make simple poke sequence",
        "<dmsizex> <dmsizey> <output image>",
        "aolmksimplezpM 50 50 pokeM",
        "long AOloopControl_compTools_mkSimpleZpokeM( long dmxsize, long dmysize, char *IDout_name)"
    );

    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}










/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
/* =============================================================================================== */




// measures cross product between 2 cubes
imageID AOloopControl_compTools_CrossProduct(
    const char *ID1_name,
    const char *ID2_name,
    const char *IDout_name
)
{
    imageID  ID1, ID2, IDout;
    uint64_t xysize1, xysize2;
    uint32_t zsize1, zsize2;
    imageID  IDmask;


    ID1 = image_ID(ID1_name);
    ID2 = image_ID(ID2_name);

    xysize1 = data.image[ID1].md[0].size[0]*data.image[ID1].md[0].size[1];
    xysize2 = data.image[ID2].md[0].size[0]*data.image[ID2].md[0].size[1];
    zsize1 = data.image[ID1].md[0].size[2];
    zsize2 = data.image[ID2].md[0].size[2];

    if(xysize1!=xysize2)
    {
        printf("ERROR: cubes %s and %s have different xysize: %ld %ld\n", ID1_name, ID2_name, xysize1, xysize2);
        exit(0);
    }

    IDmask = image_ID("xpmask");


    IDout = create_2Dimage_ID(IDout_name, zsize1, zsize2);
    for(uint64_t ii=0; ii<zsize1*zsize2; ii++)
        data.image[IDout].array.F[ii] = 0.0;

    if(IDmask==-1)
    {
        printf("No mask\n");
        fflush(stdout);


        for(uint32_t z1=0; z1<zsize1; z1++)
            for(uint32_t z2=0; z2<zsize2; z2++)
            {
                for(uint64_t ii=0; ii<xysize1; ii++)
                {
                    data.image[IDout].array.F[z2*zsize1+z1] += data.image[ID1].array.F[z1*xysize1+ii] * data.image[ID2].array.F[z2*xysize2+ii];
                }
            }
    }
    else
    {
        printf("Applying mask\n");
        fflush(stdout);

        for(uint32_t z1=0; z1<zsize1; z1++)
            for(uint32_t z2=0; z2<zsize2; z2++)
            {
                for(uint64_t ii=0; ii<xysize1; ii++)
                {
                    data.image[IDout].array.F[z2*zsize1+z1] += data.image[IDmask].array.F[ii]*data.image[IDmask].array.F[ii]*data.image[ID1].array.F[z1*xysize1+ii] * data.image[ID2].array.F[z2*xysize2+ii];
                }
            }
    }


    return IDout;
}






// create simple poke matrix
imageID AOloopControl_compTools_mkSimpleZpokeM(
    uint32_t  dmxsize,
    uint32_t  dmysize,
    char     *IDout_name
)
{
    imageID  IDout;
    uint64_t dmxysize;

    dmxysize = dmxsize * dmysize;

    IDout = create_3Dimage_ID(IDout_name, dmxsize, dmysize, dmxysize);

    for(uint64_t kk=0; kk<dmxysize; kk++)
        data.image[IDout].array.F[kk*dmxysize + kk] = 1.0;

    return IDout;
}



